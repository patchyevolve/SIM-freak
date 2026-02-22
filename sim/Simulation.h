#pragma once
// =============================================================================
//  sim/Simulation.h  — Top-level simulation controller
// =============================================================================

#include "../domain/Body.h"
#include "../physics/Integrators.h"
#include "EventBus.h"
#include <vector>
#include <string>
#include <chrono>

// ── Physics configuration ─────────────────────────────────────────────────────
struct PhysicsConfig
{
    double         G            = 6.6743e-11;
    double         softening_m  = 1.0e6;
    double         base_dt_s    = 3600.0;       ///< sim seconds per real second at 1x warp
    int            sub_steps    = 8;
    IntegratorType integrator   = IntegratorType::RK4;
    double         stellar_evolution_speed = 1.0;  ///< Multiplier for fusion and aging
};

// ── Diagnostics snapshot ──────────────────────────────────────────────────────
struct SimDiagnostics
{
    double sim_time_s      = 0.0;
    double real_time_s     = 0.0;
    size_t body_count      = 0;
    double total_energy_J  = 0.0;
    bool   energy_valid    = true;  ///< false when skipped for perf (n > 500)
    Vec2   total_momentum  = {};
    int    step_count      = 0;
};

// ── Main simulation class ─────────────────────────────────────────────────────
class Simulation
{
public:
    explicit Simulation(PhysicsConfig cfg = {});

    // ── Step ──────────────────────────────────────────────────────────────────
    /// Advance by `real_dt_s` real seconds (time warp applied internally).
    void step(double real_dt_s);

    /// Advance by exactly `sim_dt_s` simulated seconds (ignores warp).
    void step_sim(double sim_dt_s);

    // ── Body management ───────────────────────────────────────────────────────
    void add_body(Body b);
    void remove_body(const std::string& id);
    void clear_bodies();

    // ── State queries ─────────────────────────────────────────────────────────
    const std::vector<Body>& bodies()   const { return m_bodies; }
    std::vector<Body>&       bodies_mut()     { return m_bodies; }
    double                   sim_time() const { return m_sim_time_s; }
    bool                     is_paused()const { return m_paused; }
    double                   time_warp()const { return m_time_warp; }
    const PhysicsConfig&     config()   const { return m_cfg; }
    PhysicsConfig&           config_mut()     { return m_cfg; }
    SimDiagnostics           diagnostics()const;

    // ── Controls ──────────────────────────────────────────────────────────────
    void set_paused(bool p)          { m_paused = p; }
    void set_time_warp(double w)     { m_time_warp = w; }
    void set_config(PhysicsConfig c) { m_cfg = c; }

    // ── Events ────────────────────────────────────────────────────────────────
    EventBus events;

    // ── Tests ─────────────────────────────────────────────────────────────────
    static bool RunTests();

private:
    std::vector<Body>  m_bodies;
    std::vector<Body>  m_pending_bodies; ///< Bodies to be added after current loop
    double             m_sim_time_s  = 0.0;
    double             m_real_time_s = 0.0;
    bool               m_paused      = false;
    double             m_time_warp   = 1.0;
    int                m_step_count  = 0;
    PhysicsConfig      m_cfg;

    void resolve_collisions();
    void fragment_body(const Body& b, Vec2 impact_vel, double impact_mass_kg);
    void sweep_dead_bodies();
};
