// =============================================================================
//  physics/Gravity.cpp
// =============================================================================

#include "Gravity.h"
#include "BarnesHut.h"
#include <cmath>
#include <iostream>
#include <algorithm>
#include <execution>

#include "../render/GLHelper.h"

static BHTree g_bh_tree; // Re-used across frames to avoid alloc on the hot path

// GPU resources
struct GpuBodyData {
    float px, py;
    float vx, vy;
    float ax, ay;
    float mass;
    int   alive;
    int   is_passive;
    int   kind;
    float radius;
    float padding; // 48 bytes total (8-byte aligned)
};

static GLuint g_compute_program = 0;
static GLuint g_bodies_ssbo = 0;
static size_t g_ssbo_capacity = 0;
static bool   g_gpu_ready = false;
static const int GPU_BODY_THRESHOLD = 512; // Lowered to leverage GPU earlier

namespace Gravity
{

// Forward declare internal GPU helper
void dispatch_gpu(std::vector<Body>& bodies, double G, double softening_m);

// ── Core force accumulation ───────────────────────────────────────────────────

Vec2 acceleration_from(const Body& i, const Body& j,
                       double G, double softening_m)
{
    Vec2   r_ij  = j.pos - i.pos;                      // vector from i toward j
    double r2    = r_ij.norm_sq() + softening_m * softening_m; // softened r²
    double r3    = r2 * std::sqrt(r2);                  // r² * r  (= r_softened^3)
    double coeff = G * j.mass_kg / r3;
    return r_ij * coeff;                                // a_i contribution from j
}

void compute_accelerations(std::vector<Body>& bodies,
                           double G,
                           double softening_m)
{
    // Reset all accelerations
    for (auto& b : bodies)
        b.accel = Vec2{};

    // Count heavy bodies to choose algorithm
    int heavy_cnt = 0;
    for (const auto& b : bodies) if (b.alive && !b.flags.is_passive) ++heavy_cnt;

    // Algorithm Selection:
    // 1. GPU Compute (Optimized Tiled O(N²) on GPU) -> for N >= 512
    // 2. Barnes-Hut (O(N log N) on CPU)              -> for N >= 768 (if GPU disabled)
    // 3. Direct Sum (O(N²) on CPU)                   -> for small N
    
    if (g_gpu_ready && (int)bodies.size() >= GPU_BODY_THRESHOLD)
    {
        dispatch_gpu(bodies, G, softening_m);
        return;
    }

    if (heavy_cnt >= BH_DIRECT_THRESHOLD)
    {
        // ── O(n log n) Barnes-Hut (Parallel) ──────────────────────────────────
        g_bh_tree.build(bodies);
        
        std::for_each(std::execution::par, bodies.begin(), bodies.end(), [&](Body& b) {
            if (!b.alive || b.flags.immovable) return;
            // Note: acceleration_on is const, so concurrent reads from tree are safe
            b.accel = g_bh_tree.acceleration_on(static_cast<int>(&b - &bodies[0]), bodies, G, softening_m);
        });
    }
    else
    {
        // ── O(n × heavy_cnt) direct sum — only iterate over gravity-exerting bodies
        std::vector<size_t> heavy_idx;
        heavy_idx.reserve(std::min<size_t>(bodies.size(), 256));
        for (size_t i = 0; i < bodies.size(); ++i)
            if (bodies[i].alive && !bodies[i].flags.is_passive)
                heavy_idx.push_back(i);

        std::for_each(std::execution::par, bodies.begin(), bodies.end(), [&](Body& b) {
            if (!b.alive || b.flags.immovable) return;
            Vec2 total_a{};
            for (size_t j : heavy_idx)
            {
                const Body& other = bodies[j];
                if (&b == &other) continue;
                total_a += acceleration_from(b, other, G, softening_m);
            }
            b.accel = total_a;
        });
    }
}

// ── GPU Helper Functions ───────────────────────────────────────────────────────

void dispatch_gpu(std::vector<Body>& bodies, double G, double softening_m)
{
    const size_t n = bodies.size();

    // 1. Prepare buffer (resize if needed)
    if (n > g_ssbo_capacity) {
        if (g_bodies_ssbo) glDeleteBuffers(1, &g_bodies_ssbo);
        glGenBuffers(1, &g_bodies_ssbo);
        glBindBuffer(GL_SHADER_STORAGE_BUFFER, g_bodies_ssbo);
        // Over-allocate slightly to avoid constant resizing
        g_ssbo_capacity = (size_t)(n * 1.5);
        glBufferData(GL_SHADER_STORAGE_BUFFER, g_ssbo_capacity * sizeof(GpuBodyData), nullptr, GL_DYNAMIC_DRAW);
    }

    // 2. Upload body data to GPU
    static std::vector<GpuBodyData> s_gpu_data;
    if (s_gpu_data.size() < n) s_gpu_data.resize(n);
    
    for (size_t i = 0; i < n; ++i) {
        const auto& b = bodies[i];
        s_gpu_data[i] = {
            (float)b.pos.x, (float)b.pos.y,
            (float)b.vel.x, (float)b.vel.y,
            0.0f, 0.0f,
            (float)b.mass_kg,
            b.alive ? 1 : 0,
            b.flags.is_passive ? 1 : 0,
            (int)b.kind,
            (float)b.radius_m,
            0.0f
        };
    }
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, g_bodies_ssbo);
    // Use SubData to avoid re-allocating memory on the driver side
    glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, n * sizeof(GpuBodyData), s_gpu_data.data());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, g_bodies_ssbo);

    // 3. Dispatch Compute Shader
    glUseProgram(g_compute_program);
    glUniform1i(0, (int)n);
    glUniform1f(1, (float)G);
    glUniform1f(2, (float)(softening_m * softening_m));
    
    GLuint groups = (GLuint)((n + 255) / 256);
    glDispatchCompute(groups, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    // 4. Read back acceleration
    GpuBodyData* mapped = (GpuBodyData*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, n * sizeof(GpuBodyData), GL_MAP_READ_BIT);
    if (mapped)
    {
        for (size_t i = 0; i < n; ++i) {
            if (bodies[i].alive) {
                bodies[i].accel.x = (double)mapped[i].ax;
                bodies[i].accel.y = (double)mapped[i].ay;
            }
        }
        glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    }
}

bool InitGPU()
{
    if (!GLHelper::Init()) return false;
    
    // Check multiple possible paths for the shader
    const char* paths[] = { "physics/gravity.comp", "render/gravity.comp", "../physics/gravity.comp" };
    for (const char* p : paths) {
        g_compute_program = GLHelper::CreateComputeProgram(p);
        if (g_compute_program != 0) break;
    }
    
    if (g_compute_program == 0) {
        std::cerr << "[Physics] Failed to load gravity.comp from any known path.\n";
        return false;
    }
    
    g_gpu_ready = true;
    std::cout << "[Physics] GPU Compute Gravity Engine initialised.\n";
    return true;
}

void Cleanup()
{
    if (g_compute_program) glDeleteProgram(g_compute_program);
    if (g_bodies_ssbo) glDeleteBuffers(1, &g_bodies_ssbo);
    g_gpu_ready = false;
}

// ── Diagnostic quantities ─────────────────────────────────────────────────────

Vec2 total_momentum(const std::vector<Body>& bodies)
{
    Vec2 p;
    for (const auto& b : bodies)
        if (b.alive) p += b.vel * b.mass_kg;
    return p;
}

double total_kinetic_energy(const std::vector<Body>& bodies)
{
    double ke = 0.0;
    for (const auto& b : bodies)
        if (b.alive) ke += b.kinetic_energy();
    return ke;
}

double total_potential_energy(const std::vector<Body>& bodies,
                              double G, double softening_m)
{
    const size_t n = bodies.size();
    if (n < 2) return 0.0;

    std::vector<double> partial_sums(n, 0.0);
    std::for_each(std::execution::par, partial_sums.begin(), partial_sums.end(), [&](double& sum) {
        size_t i = &sum - &partial_sums[0];
        if (!bodies[i].alive) return;
        
        for (size_t j = i + 1; j < n; ++j)
        {
            if (!bodies[j].alive) continue;
            if (bodies[i].flags.is_passive && bodies[j].flags.is_passive) continue;

            double r = bodies[i].pos.dist_to(bodies[j].pos);
            double r_soft = std::sqrt(r * r + softening_m * softening_m);
            sum -= G * bodies[i].mass_kg * bodies[j].mass_kg / r_soft;
        }
    });

    double pe = 0.0;
    for (double s : partial_sums) pe += s;
    return pe;
}

double total_energy(const std::vector<Body>& bodies,
                    double G, double softening_m)
{
    return total_kinetic_energy(bodies) +
           total_potential_energy(bodies, G, softening_m);
}

// ── Unit tests ────────────────────────────────────────────────────────────────
bool RunTests()
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] Gravity::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] Gravity::" << name << "\n";
    };

    const double G    = 6.6743e-11;
    const double eps  = 0.0;   // no softening for exact tests

    // Two equal masses 1 AU apart: forces must be equal and opposite
    {
        Body a, b;
        a.mass_kg = 1e24; a.pos = { 0.0, 0.0 };
        b.mass_kg = 1e24; b.pos = { 1.5e11, 0.0 };

        Vec2 acc_a = acceleration_from(a, b, G, eps);
        Vec2 acc_b = acceleration_from(b, a, G, eps);

        // a_i and a_j should be equal magnitude, opposite direction
        check("equal_force_x",  std::abs(acc_a.x + acc_b.x) < 1e-30);
        check("equal_force_y",  std::abs(acc_a.y + acc_b.y) < 1e-30);
        check("accel_positive", acc_a.x > 0.0);    // a attracted toward b (right)
        check("accel_negative", acc_b.x < 0.0);    // b attracted toward a (left)
    }

    // Acceleration magnitude: F = G*m1*m2/r² → a = G*m2/r²
    {
        Body a, b;
        a.mass_kg = 1.0;              // test body
        b.mass_kg = 5.972e24;         // Earth
        b.pos     = { 6.371e6, 0.0 }; // Earth radius away

        Vec2 acc = acceleration_from(a, b, G, eps);
        double g_expected = G * b.mass_kg / (6.371e6 * 6.371e6);
        check("accel_magnitude", std::abs(acc.x - g_expected) < 0.01);
    }

    // Total momentum: two-body system, no forces yet → just check it doesn't crash
    {
        Body a, b;
        a.mass_kg = 1.0; a.vel = {  1.0, 0.0 };
        b.mass_kg = 1.0; b.vel = { -1.0, 0.0 };
        std::vector<Body> bods = { a, b };
        Vec2 p = total_momentum(bods);
        check("total_momentum_cancels", std::abs(p.x) < 1e-15 && std::abs(p.y) < 1e-15);
    }

    // No NaN/Inf on close approach with softening
    {
        Body a, b;
        a.mass_kg = 1e30; a.pos = { 0.0, 0.0 };
        b.mass_kg = 1e30; b.pos = { 0.0, 0.0 }; // exactly coincident
        double soft = 1e6;
        Vec2 acc = acceleration_from(a, b, G, soft);
        check("no_nan_close_approach", acc.is_finite());
    }

    return ok;
}

bool IsGpuReady() { return g_gpu_ready; }

} // namespace Gravity
