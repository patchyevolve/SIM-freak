// =============================================================================
//  sim/Simulation.cpp
// =============================================================================

#include "Simulation.h"
#include "StellarEvolution.h"
#include "../physics/Gravity.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

// ── Constructor ───────────────────────────────────────────────────────────────
Simulation::Simulation(PhysicsConfig cfg)
    : m_cfg(cfg)
{}

// ── Step (real-time driven) ───────────────────────────────────────────────────
void Simulation::step(double real_dt_s)
{
    if (m_paused) return;
    m_real_time_s += real_dt_s;
    double sim_dt = real_dt_s * m_cfg.base_dt_s * m_time_warp;
    step_sim(sim_dt);
}

// ── Step (sim-time driven) ────────────────────────────────────────────────────
void Simulation::step_sim(double sim_dt_s)
{
    if (m_bodies.empty()) { m_sim_time_s += sim_dt_s; return; }

    // ── 1. Dynamic Step Limit (Balanced) ────────────────────────────────────
    // Optimization: Instead of N^2, only check against heavy bodies (gravity sources)
    // Most bodies in large sims (Galaxy/Nebula) are passive/light.
    
    double min_tau = 1e30;
    const double G = m_cfg.G;
    const size_t nb = m_bodies.size();

    // Identify gravity sources (non-passive bodies)
    std::vector<size_t> source_idx;
    source_idx.reserve(512);
    for (size_t i = 0; i < nb; ++i) {
        if (!m_bodies[i].flags.is_passive && m_bodies[i].alive)
            source_idx.push_back(i);
    }

    if (!source_idx.empty()) {
        for (size_t i : source_idx) {
            for (size_t j = 0; j < nb; ++j) {
                if (i == j || !m_bodies[j].alive) continue;
                Vec2 d = m_bodies[i].pos - m_bodies[j].pos;
                double r2 = d.x*d.x + d.y*d.y;
                double mass_sum = std::max(1.0, m_bodies[i].mass_kg + m_bodies[j].mass_kg);
                // tau = r / v_orbit approx
                double tau = std::sqrt(r2 * std::sqrt(r2) / (G * mass_sum + 1e-10));
                if (tau < min_tau) min_tau = tau;
            }
        }
    } else {
        min_tau = 3600.0; // Fallback for drifting debris
    }
    
    // Safety clamp: stable dt should be roughly 10% of the orbital timescale
    // We remove the hard 'floor_dt' because it kills stability for tight systems (BHs).
    double safety_dt = std::max(0.1, min_tau * 0.1); 
    const double MAX_dt = 3600.0; // 1 hour max
    double target_sub_dt = std::min(MAX_dt, safety_dt);
    
    int steps = std::max(1, (int)std::ceil(sim_dt_s / target_sub_dt));
    
    // Performance cap: 128 sub-steps max per frame.
    // If time warp is too high, simulation slows down instead of exploding.
    if (steps > 128) steps = 128;
    
    double sub_dt = sim_dt_s / static_cast<double>(steps);
    m_step_count += steps;

    // ── Phase 21: Relativistic Dilation ─────────────────────────────────────
    const double c  = 299792458.0; 
    const double c2 = c * c;

    for (int s = 0; s < steps; ++s)
    {
        // Collect black hole indices once (O(n)) instead of O(n²) per-body scan
        std::vector<size_t> bh_indices;
        bh_indices.reserve(8);
        for (size_t i = 0; i < m_bodies.size(); ++i)
            if (m_bodies[i].alive && m_bodies[i].kind == BodyKind::BlackHole)
                bh_indices.push_back(i);

        // Update local time scales based on proximity to Black Holes
        for (auto& b : m_bodies) {
            if (!b.alive) continue;
            
            double drag_factor = 1.0;
            
            // 1. Kinetic Dilation (Special Relativity)
            double v2 = b.vel.norm_sq();
            double kinetic_scale = std::sqrt(std::max(0.0, 1.0 - v2 / c2));
            drag_factor *= kinetic_scale;
            
            // 2. Gravitational Dilation (Schwarzschild) — only check BHs
            for (size_t idx : bh_indices) {
                const auto& other = m_bodies[idx];
                double dist = b.dist_to(other);
                double Rs = (2.0 * m_cfg.G * other.mass_kg) / c2;
                
                if (dist > Rs) {
                    double grav_scale = std::sqrt(1.0 - Rs / dist);
                    drag_factor *= grav_scale;
                } else {
                    drag_factor = 0.0; // Crossed event horizon (frozen to distant observer)
                }
            }
            b.local_time_scale = drag_factor;
        }

        Integrators::step(m_bodies, sub_dt, m_cfg.G, m_cfg.softening_m, m_cfg.integrator);
        resolve_collisions();

        // Phase 25D: Event Horizon Consumption — only check against BHs
        const double c2_eh = 299792458.0 * 299792458.0;
        for (auto& b : m_bodies)
        {
            if (!b.alive || b.kind == BodyKind::BlackHole) continue;
            for (size_t idx : bh_indices)
            {
                const auto& bh = m_bodies[idx];
                if (!bh.alive) continue;
                double rs = (2.0 * m_cfg.G * bh.mass_kg) / c2_eh;
                if (b.dist_to(bh) < rs * 0.9)
                {
                    b.alive = false;
                    events.on_bh_absorption.emit({ b.id, bh.id });
                    break;
                }
            }
        }
        
        // ── Safe Insertion ───────────────────────────────────────────────────
        if (!m_pending_bodies.empty())
        {
            m_bodies.insert(m_bodies.end(), 
                            std::make_move_iterator(m_pending_bodies.begin()), 
                            std::make_move_iterator(m_pending_bodies.end()));
            m_pending_bodies.clear();
        }
    }

    // ── 2. NaN Firewall ──────────────────────────────────────────────────────
    // Remove any bodies that became non-finite (infinite energy explosion)
    m_bodies.erase(std::remove_if(m_bodies.begin(), m_bodies.end(), [](const Body& b){
        return !b.pos.is_finite() || !b.vel.is_finite();
    }), m_bodies.end());

    // Phase 25A: Stellar Evolution — run once per physics batch
    StellarEvolution::tick_all(m_bodies, sim_dt_s);

    sweep_dead_bodies();
    m_sim_time_s += sim_dt_s;
}

// ── Body management ───────────────────────────────────────────────────────────
void Simulation::add_body(Body b)
{
    events.on_body_added.emit({ b });
    m_bodies.push_back(std::move(b));
}

void Simulation::remove_body(const std::string& id)
{
    auto it = std::find_if(m_bodies.begin(), m_bodies.end(),
                           [&](const Body& b){ return b.id == id; });
    if (it == m_bodies.end()) return;

    events.on_body_removed.emit({ it->id, it->name });
    m_bodies.erase(it);
}

void Simulation::clear_bodies()
{
    m_bodies.clear();
    m_sim_time_s  = 0.0;
    m_real_time_s = 0.0;
    m_step_count  = 0;
}

// ── Collision resolution (inelastic merge) ────────────────────────────────────
static constexpr int MAX_COLLISION_ITERATIONS = 20;  // Cap to avoid freeze with many bodies

void Simulation::resolve_collisions()
{
    bool any_merged = true;
    int iterations = 0;

    // Repeat until no more merges in this sub-step (cascades)
    const size_t n = m_bodies.size();

    while (any_merged && iterations < MAX_COLLISION_ITERATIONS)
    {
        any_merged = false;

        // O(heavy × n): only heavy bodies need collision checks; passive–passive skip.
        std::vector<size_t> heavy_idx;
        heavy_idx.reserve(64);
        for (size_t i = 0; i < n; ++i)
            if (m_bodies[i].alive && !m_bodies[i].flags.is_passive)
                heavy_idx.push_back(i);

        for (size_t hi : heavy_idx)
        {
            if (!m_bodies[hi].alive || m_bodies[hi].flags.no_collide) continue;
            for (size_t j = 0; j < n; ++j)
            {
                if (j == hi) continue;
                if (!m_bodies[j].alive || m_bodies[j].flags.no_collide) continue;
                if (!m_bodies[j].flags.is_passive && j <= hi) continue;  // heavy–heavy once

                if (m_bodies[hi].overlaps(m_bodies[j]))
                {
                    size_t i = hi;
                    Vec2 v_rel = m_bodies[i].vel - m_bodies[j].vel;
                    double v_mag = v_rel.norm();

                    bool can_fragment = (m_bodies[i].kind != BodyKind::BlackHole && m_bodies[j].kind != BodyKind::BlackHole)
                                     && (!m_bodies[i].flags.is_passive && !m_bodies[j].flags.is_passive);
                    const size_t total_bodies = m_bodies.size() + m_pending_bodies.size();
                    constexpr size_t FRAGMENT_BODY_CAP = 600;
                    if (can_fragment && v_mag > 2000.0 && total_bodies < FRAGMENT_BODY_CAP)
                    {
                        if (m_bodies[i].mass_kg < m_bodies[j].mass_kg * 10.0 && m_bodies[i].mass_kg > 1e22)
                            fragment_body(m_bodies[i], -v_rel, m_bodies[j].mass_kg);
                        if (m_bodies[j].mass_kg < m_bodies[i].mass_kg * 10.0 && m_bodies[j].mass_kg > 1e22)
                            fragment_body(m_bodies[j], v_rel, m_bodies[i].mass_kg);
                    }

                    Body merged = merge_bodies(m_bodies[i], m_bodies[j]);
                    events.on_collision.emit({ merged,
                        (m_bodies[i].mass_kg >= m_bodies[j].mass_kg) ? m_bodies[j] : m_bodies[i] });

                    size_t survivor = (m_bodies[i].mass_kg >= m_bodies[j].mass_kg) ? i : j;
                    size_t absorbed = (survivor == i) ? j : i;
                    m_bodies[survivor] = merged;
                    m_bodies[absorbed].alive = false;
                    any_merged = true;
                    break;  // one merge per heavy per iteration
                }
            }
        }
        ++iterations;
    }
}

void Simulation::fragment_body(const Body& b, Vec2 impact_vel, double impact_mass_kg)
{
    // High energy impacts splinter the body into passive fragments
    int num_fragments = 8;
    if (b.kind == BodyKind::Planet) num_fragments = 16;
    if (b.kind == BodyKind::Star)   num_fragments = 32;

    double frag_mass = (b.mass_kg * 0.1) / num_fragments; // 10% mass becomes debris
    double frag_radius = b.radius_m * 0.1;

    for (int i = 0; i < num_fragments; ++i)
    {
        Body f = b; // Copy properties (composition, etc.)
        f.id = b.id + "_debris_" + std::to_string(m_step_count) + "_" + std::to_string(i);
        f.name = "Fragment";
        f.mass_kg = frag_mass;
        f.radius_m = frag_radius;
        f.flags.is_passive = true; // Optimization: debris doesn't exert gravity

        float angle = (i / (float)num_fragments) * 6.283f;
        Vec2 offset = { std::cos(angle), std::sin(angle) };
        // Spawn slightly outside the radius to prevent immediate re-collision
        f.pos = b.pos + offset * (b.radius_m * 1.2);
        
        double kick = 500.0 + (impact_vel.norm() * 0.2); // explosion "kick"
        f.vel = b.vel + (offset * kick);

        m_pending_bodies.push_back(std::move(f));
    }
}

// ── Sweep dead bodies ─────────────────────────────────────────────────────────
void Simulation::sweep_dead_bodies()
{
    m_bodies.erase(
        std::remove_if(m_bodies.begin(), m_bodies.end(),
                       [](const Body& b){ return !b.alive; }),
        m_bodies.end());
}

// ── Diagnostics ───────────────────────────────────────────────────────────────
static constexpr size_t DIAG_ENERGY_BODY_CAP = 500;  // Skip O(n²) energy when above

SimDiagnostics Simulation::diagnostics() const
{
    SimDiagnostics d;
    d.sim_time_s     = m_sim_time_s;
    d.real_time_s    = m_real_time_s;
    d.body_count     = m_bodies.size();
    if (m_bodies.size() <= DIAG_ENERGY_BODY_CAP) {
        d.total_energy_J = Gravity::total_energy(m_bodies, m_cfg.G, m_cfg.softening_m);
        d.energy_valid   = true;
    } else {
        d.total_energy_J = 0.0;
        d.energy_valid   = false;  // Skipped for perf
    }
    d.total_momentum = Gravity::total_momentum(m_bodies);
    d.step_count     = m_step_count;
    return d;
}

// ── Unit tests ────────────────────────────────────────────────────────────────
bool Simulation::RunTests()
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] Simulation::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] Simulation::" << name << "\n";
    };

    const double G    = 6.6743e-11;
    const double soft = 1e6;

    // ── Pause test ────────────────────────────────────────────────────────────
    {
        PhysicsConfig cfg; cfg.base_dt_s = 3600.0;
        Simulation sim(cfg);
        Body b; b.mass_kg=1e24; b.alive=true;
        sim.add_body(b);
        sim.set_paused(true);
        sim.step(1.0);  // real 1 second
        check("pause_no_advance", sim.sim_time() == 0.0);

        sim.set_paused(false);
        sim.step(1.0);
        check("unpause_advances", sim.sim_time() > 0.0);
    }

    // ── Collision merge ───────────────────────────────────────────────────────
    {
        PhysicsConfig cfg2; cfg2.G = G; cfg2.softening_m = soft;
        cfg2.sub_steps = 1; cfg2.base_dt_s = 1.0;
        Simulation sim(cfg2);

        Body a, b;
        a.id = "a"; a.name = "A"; a.mass_kg = 1e20; a.radius_m = 5e5;
        a.pos = { 0.0, 0.0 }; a.alive = true;

        b.id = "b"; b.name = "B"; b.mass_kg = 1e20; b.radius_m = 5e5;
        b.pos = { 5e5, 0.0 };   // overlapping
        b.alive = true;

        sim.add_body(a);
        sim.add_body(b);

        bool collision_fired = false;
        sim.events.on_collision.subscribe([&](const EvCollision&){ collision_fired = true; });

        sim.step_sim(1.0);

        check("collision_event_fired", collision_fired);
        check("bodies_merged",         sim.bodies().size() == 1);
        check("merged_mass",           std::abs(sim.bodies()[0].mass_kg - 2e20) < 1e10);
    }

    // ── No NaN after 100 steps (two-body orbit) ───────────────────────────────
    {
        PhysicsConfig cfg3; cfg3.G = G; cfg3.softening_m = 1e5;
        cfg3.sub_steps = 4; cfg3.base_dt_s = 3600.0;
        Simulation sim(cfg3);

        Body sun, earth;
        sun.id = "sun";     sun.mass_kg = 1.989e30;
        earth.id = "earth"; earth.mass_kg = 5.972e24;
        earth.pos = { 1.496e11, 0.0 };
        earth.vel = { 0.0, std::sqrt(G * sun.mass_kg / 1.496e11) };
        sim.add_body(sun); sim.add_body(earth);

        bool all_finite = true;
        for (int i = 0; i < 100; ++i)
        {
            sim.step_sim(3600.0);
            for (const auto& b2 : sim.bodies())
                if (!b2.pos.is_finite() || !b2.vel.is_finite()) all_finite = false;
        }
        check("no_nan_100steps", all_finite);
    }

    return ok;
}
