// =============================================================================
//  physics/Integrators.cpp
// =============================================================================

#include "Integrators.h"
#include "Gravity.h"
#include <cmath>
#include <cassert>
#include <iostream>
#include <stdexcept>
#include <vector>

// ── Helpers ───────────────────────────────────────────────────────────────────

const char* integrator_name(IntegratorType t)
{
    switch (t)
    {
    case IntegratorType::RK4:             return "RK4";
    case IntegratorType::SymplecticEuler: return "SymplecticEuler";
    case IntegratorType::VelocityVerlet:  return "VelocityVerlet";
    default:                              return "Unknown";
    }
}

IntegratorType integrator_from_str(const std::string& s)
{
    if (s == "SymplecticEuler") return IntegratorType::SymplecticEuler;
    if (s == "VelocityVerlet")  return IntegratorType::VelocityVerlet;
    return IntegratorType::RK4;
}

namespace Integrators
{

// ── Dispatcher ────────────────────────────────────────────────────────────────

void step(std::vector<Body>& bodies,
          double dt, double G, double softening_m,
          IntegratorType type)
{
    switch (type)
    {
    case IntegratorType::SymplecticEuler:
        step_symplectic_euler(bodies, dt, G, softening_m); break;
    case IntegratorType::VelocityVerlet:
        step_velocity_verlet(bodies, dt, G, softening_m);  break;
    default:
        step_rk4(bodies, dt, G, softening_m);              break;
    }
}

// ── RK4 ──────────────────────────────────────────────────────────────────────
// State vector per body: (pos, vel) — 4 doubles.
// k1..k4 Runge-Kutta stages require re-evaluating accelerations at shifted positions.

struct Derivative { Vec2 dpos; Vec2 dvel; };

// ── Drag Force Calculation ──────────────────────────────────────────────────
// potential_hosts: only bodies with has_atmos (caller passes pre-filtered list for perf)
static Vec2 compute_drag_accel(const Body& b, const std::vector<Body>& all_bodies,
                               const std::vector<size_t>& atmos_host_indices)
{
    if (b.flags.immovable) return Vec2{};
    if (b.mass_kg <= 0.0) return Vec2{};  // Avoid div-by-zero

    Vec2 total_drag_accel{};
    for (size_t idx : atmos_host_indices)
    {
        const Body& host = all_bodies[idx];
        if (&b == &host || !host.alive) continue;
        if (host.atmos.scale_height_m <= 0.0) continue;  // Avoid div-by-zero in exp

        double r = b.pos.dist_to(host.pos);
        double alt = r - host.radius_m;

        // Atmosphere only reaches a few scale heights realistically
        if (alt < 0.0 || alt > host.atmos.scale_height_m * 15.0) continue;

        // Density: rho = rho0 * exp(-h/H)
        double rho = host.atmos.surface_density_kg_m3 * std::exp(-alt / host.atmos.scale_height_m);

        // Relative velocity (relative to host translation)
        Vec2 v_rel = b.vel - host.vel;
        double speed_rel = v_rel.norm();
        if (speed_rel < 1e-3) continue;

        // Drag Force: F = -0.5 * rho * v^2 * Cd * Area
        double A = 3.1415926535 * b.radius_m * b.radius_m;
        double F_mag = 0.5 * rho * speed_rel * speed_rel * host.atmos.drag_coeff * A;

        double a_mag = F_mag / b.mass_kg;
        total_drag_accel -= v_rel.normalized() * a_mag;
    }
    return total_drag_accel;
}

// Build list of atmospheric host indices (bodies with has_atmos) — O(n) once per step
static std::vector<size_t> collect_atmos_hosts(const std::vector<Body>& bodies)
{
    std::vector<size_t> idx;
    idx.reserve(16);
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].alive && bodies[i].has_atmos)
            idx.push_back(i);
    return idx;
}

static std::vector<Derivative> eval_derivatives(
    const std::vector<Body>& base,
    const std::vector<Derivative>& delta,
    double scale,
    double G, double softening_m,
    const std::vector<size_t>& atmos_hosts)
{
    // High-Performance Shadow State: 
    // We avoid deep-copying 10,000 Body objects (strings, events, etc.) 
    // by using a persistent shadow vector and only updating kinematic fields.
    thread_local std::vector<Body> shadow_bodies;
    if (shadow_bodies.size() != base.size()) shadow_bodies = base;

    for (size_t i = 0; i < base.size(); ++i)
    {
        if (!base[i].alive) { shadow_bodies[i].alive = false; continue; }
        shadow_bodies[i].alive = true;
        
        if (base[i].flags.immovable) {
            shadow_bodies[i].pos = base[i].pos;
            shadow_bodies[i].vel = base[i].vel;
        } else {
            shadow_bodies[i].pos = base[i].pos + delta[i].dpos * scale;
            shadow_bodies[i].vel = base[i].vel + delta[i].dvel * scale;
        }

        // Update physical properties that might have changed (or differ between tests)
        shadow_bodies[i].has_atmos = base[i].has_atmos;
        shadow_bodies[i].atmos     = base[i].atmos;
        shadow_bodies[i].mass_kg   = base[i].mass_kg;
        shadow_bodies[i].radius_m  = base[i].radius_m;
    }

    // Compute accelerations on shadow state
    Gravity::compute_accelerations(shadow_bodies, G, softening_m);

    // Derivatives: dpos/dt = vel,  dvel/dt = (accel + drag) * dilation
    std::vector<Derivative> derivs(base.size());
    for (size_t i = 0; i < base.size(); ++i)
    {
        if (!base[i].alive) continue;
        double s = base[i].local_time_scale;
        derivs[i].dpos = shadow_bodies[i].vel * s;
        Vec2 drag_a = compute_drag_accel(shadow_bodies[i], shadow_bodies, atmos_hosts);
        derivs[i].dvel = (shadow_bodies[i].accel + drag_a) * s;
    }
    return derivs;
}

void step_rk4(std::vector<Body>& bodies,
              double dt, double G, double softening_m)
{
    const size_t n = bodies.size();
    if (n == 0) return;

    // Hoist expensive atmosphere scan once per RK4 step
    auto atmos_hosts = collect_atmos_hosts(bodies);

    // k1: derivatives at t
    std::vector<Derivative> zero(n, { Vec2{}, Vec2{} });
    auto k1 = eval_derivatives(bodies, zero, 0.0, G, softening_m, atmos_hosts);

    // k2: derivatives at t + dt/2, using k1
    auto k2 = eval_derivatives(bodies, k1, dt * 0.5, G, softening_m, atmos_hosts);

    // k3: derivatives at t + dt/2, using k2
    auto k3 = eval_derivatives(bodies, k2, dt * 0.5, G, softening_m, atmos_hosts);

    // k4: derivatives at t + dt, using k3
    auto k4 = eval_derivatives(bodies, k3, dt, G, softening_m, atmos_hosts);

    // Combine: state += dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    for (size_t i = 0; i < n; ++i)
    {
        if (!bodies[i].alive || bodies[i].flags.immovable) continue;
        
        bodies[i].pos += (k1[i].dpos + k2[i].dpos * 2.0 +
                          k3[i].dpos * 2.0 + k4[i].dpos) * (dt / 6.0);
        bodies[i].vel += (k1[i].dvel + k2[i].dvel * 2.0 +
                          k3[i].dvel * 2.0 + k4[i].dvel) * (dt / 6.0);
                          
        bodies[i].proper_time_s += dt * bodies[i].local_time_scale;
    }
    // Final acceleration update for diagnostics
    Gravity::compute_accelerations(bodies, G, softening_m);
}

// ── Symplectic Euler ──────────────────────────────────────────────────────────
// vel_new = vel + accel * dt
// pos_new = pos + vel_new * dt   (note: uses updated vel → symplectic)

void step_symplectic_euler(std::vector<Body>& bodies,
                           double dt, double G, double softening_m)
{
    Gravity::compute_accelerations(bodies, G, softening_m);
    std::vector<size_t> atmos = collect_atmos_hosts(bodies);
    for (auto& b : bodies)
    {
        if (!b.alive || b.flags.immovable) continue;
        double local_dt = dt * b.local_time_scale;
        Vec2 drag_a = compute_drag_accel(b, bodies, atmos);
        b.vel += (b.accel + drag_a) * local_dt;
        b.pos += b.vel   * local_dt;
        b.proper_time_s += local_dt;
    }
}

// ── Velocity Verlet ───────────────────────────────────────────────────────────
// pos_new = pos + vel*dt + 0.5 * accel * dt²
// accel_new = f(pos_new)
// vel_new = vel + 0.5 * (accel + accel_new) * dt

void step_velocity_verlet(std::vector<Body>& bodies,
                          double dt, double G, double softening_m)
{
    std::vector<size_t> atmos = collect_atmos_hosts(bodies);

    // Half-kick, then drift — accel and drag at old position
    Gravity::compute_accelerations(bodies, G, softening_m);
    std::vector<Vec2> accel_old(bodies.size());
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (!bodies[i].alive || bodies[i].flags.immovable) continue;
        Vec2 drag_a = compute_drag_accel(bodies[i], bodies, atmos);
        accel_old[i] = bodies[i].accel + drag_a;  // Store full accel (gravity + drag)
        double local_dt = dt * bodies[i].local_time_scale;
        bodies[i].pos += bodies[i].vel * local_dt + accel_old[i] * (0.5 * local_dt * local_dt);
        bodies[i].proper_time_s += local_dt;
    }

    // Re-evaluate gravity and drag at new positions
    Gravity::compute_accelerations(bodies, G, softening_m);

    // Full kick with averaged acceleration (gravity + drag)
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        if (!bodies[i].alive || bodies[i].flags.immovable) continue;
        Vec2 drag_new = compute_drag_accel(bodies[i], bodies, atmos);
        Vec2 accel_new = bodies[i].accel + drag_new;
        double local_dt = dt * bodies[i].local_time_scale;
        bodies[i].vel += (accel_old[i] + accel_new) * (0.5 * local_dt);
    }
}

// ── Unit tests ────────────────────────────────────────────────────────────────
bool RunTests()
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] Integrators::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] Integrators::" << name << "\n";
    };

    const double G    = 6.6743e-11;
    const double soft = 1e6;

    // One step of each integrator — verify no NaN/Inf and position changed
    auto make_two_body = [&]() -> std::vector<Body>
    {
        Body sun, earth;
        sun.mass_kg  = 1.989e30; sun.pos  = { 0.0, 0.0 };   sun.vel  = {};
        earth.mass_kg= 5.972e24; earth.pos= { 1.496e11, 0.0 };
        // Circular orbit velocity: v = sqrt(G*M/r)
        earth.vel = { 0.0, std::sqrt(G * sun.mass_kg / 1.496e11) };
        sun.flags.immovable = false;
        return { sun, earth };
    };

    double dt = 3600.0; // 1 hour

    for (auto itype : { IntegratorType::RK4,
                         IntegratorType::SymplecticEuler,
                         IntegratorType::VelocityVerlet })
    {
        auto bodies = make_two_body();
        Vec2 pos_before = bodies[1].pos;
        step(bodies, dt, G, soft, itype);
        Vec2 pos_after  = bodies[1].pos;

        std::string prefix = std::string("no_nan_") + integrator_name(itype);
        check(prefix.c_str(), bodies[1].pos.is_finite() && bodies[1].vel.is_finite());

        std::string prefix2 = std::string("pos_changed_") + integrator_name(itype);
        check(prefix2.c_str(), (pos_after - pos_before).norm() > 1.0); // moved at least 1m
    }

    // RK4: after 1 year of Earth orbit, Earth should be roughly back to start
    {
        auto bodies = make_two_body();
        double year_s = 365.25 * 24.0 * 3600.0;
        double step_s = 3600.0 * 6.0;  // 6-hour steps
        int    steps  = (int)(year_s / step_s);
        for (int i = 0; i < steps; ++i)
            step_rk4(bodies, step_s, G, soft);

        double dist = bodies[1].pos.dist_to({ 1.496e11, 0.0 });
        // Within 2% of AU
        check("rk4_1year_orbit", dist < 1.496e11 * 0.02);
    }

    // Phase 29: Drag Test (Orbit Decay)
    {
        auto bodies = make_two_body();
        // Earth atmosphere
        bodies[0].has_atmos = true;
        bodies[0].atmos.surface_density_kg_m3 = 1.225;
        bodies[0].atmos.scale_height_m = 8500.0;
        bodies[0].radius_m = 6.371e6; // Earth radius
        bodies[0].mass_kg = 5.972e24;
        bodies[0].pos = {0,0}; bodies[0].vel = {0,0};

        // Satellite in low orbit (100km) — within 15 scale-heights (~127km) for drag
        Body sat;
        sat.mass_kg = 1000.0;
        sat.radius_m = 5.0; // Large area to mass ratio for test visibility
        sat.pos = { 6.371e6 + 100000.0, 0.0 };
        double v_circ = std::sqrt(G * bodies[0].mass_kg / sat.pos.x);
        sat.vel = { 0.0, v_circ };
        
        bodies[1] = sat;

        double r_start = sat.pos.norm();
        // std::cout << "  [Debug] Drag Test Start: r=" << r_start << " alt=" << (r_start - bodies[0].radius_m) << "\n";
        
        // Step for a few minutes (shorter time, smaller steps for stability)
        for (int i = 0; i < 60; ++i) {
           step_rk4(bodies, 1.0, G, soft); // 1s steps
           // if (i % 10 == 0) std::cout << "    Step " << i << ": alt=" << (bodies[1].pos.norm() - bodies[0].radius_m) << " v=" << bodies[1].vel.norm() << "\n";
        }
        
        double r_end = bodies[1].pos.norm();
        // std::cout << "  [Debug] Drag Test End: r=" << r_end << " decay=" << (r_start - r_end) << "\n";
        check("drag_orbit_decay", r_end < r_start);
    }

    return ok;
}

} // namespace Integrators
