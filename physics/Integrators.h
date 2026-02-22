#pragma once
// =============================================================================
//  physics/Integrators.h  — Pluggable numerical integrators
//  Supported: RK4 (default), Symplectic Euler, Velocity Verlet
// =============================================================================

#include "../domain/Body.h"
#include <vector>
#include <functional>

enum class IntegratorType : uint8_t
{
    RK4             = 0,
    SymplecticEuler = 1,
    VelocityVerlet  = 2
};

const char*     integrator_name(IntegratorType t);
IntegratorType  integrator_from_str(const std::string& s);

namespace Integrators
{
    // ─────────────────────────────────────────────────────────────────────────
    /// Advance a system of bodies by `dt` seconds using the chosen integrator.
    /// `G` and `softening_m` are forwarded to the gravity kernel.
    void step(std::vector<Body>& bodies,
              double dt,
              double G,
              double softening_m,
              IntegratorType type = IntegratorType::RK4);

    // ─────────────────────────────────────────────────────────────────────────
    /// Individual integrator implementations (exposed for testing)
    void step_rk4(std::vector<Body>& bodies,
                  double dt, double G, double softening_m);

    void step_symplectic_euler(std::vector<Body>& bodies,
                               double dt, double G, double softening_m);

    void step_velocity_verlet(std::vector<Body>& bodies,
                              double dt, double G, double softening_m);

    // ─────────────────────────────────────────────────────────────────────────
    bool RunTests();
}
