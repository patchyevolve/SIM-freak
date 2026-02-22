#pragma once
// =============================================================================
//  sim/StellarEvolution.h  — Rules-based stellar lifecycle system
//  
//  Called every physics step. Inspects each body's composition + density
//  and transitions it through stellar stages: MainSequence → RedGiant →
//  WhiteDwarf → NeutronStar → BlackHole.
// =============================================================================

#include "../domain/Body.h"
#include <vector>

namespace StellarEvolution
{
    // ── Constants ──────────────────────────────────────────────────────────────

    /// Minimum hydrogen fraction before fusion stops (Red Giant trigger)
    static constexpr float H_DEPLETION_THRESHOLD   = 0.03f;

    /// Helium fraction needed to trigger helium flash (RGB → HB)
    static constexpr float HE_FLASH_THRESHOLD      = 0.65f;

    /// Core density thresholds (kg/m³)
    static constexpr double NEUTRON_STAR_DENSITY    = 1.0e17;
    static constexpr double BLACK_HOLE_DENSITY      = 5.0e17;

    /// Chandrasekhar-ish mass limit (in Solar masses, simplified)
    static constexpr double WHITE_DWARF_MASS_LIMIT  = 2.8e30; // ~1.4 M_sun
    
    /// Rate of hydrogen → helium fusion per year of sim time (fraction/s)
    /// Real rate: Sun consumes ~6e11 kg/s = ~3% per billion years
    static constexpr double H_FUSION_RATE_PER_S  = 2.0e-18; // very slow

    // ── Main interface ─────────────────────────────────────────────────────────

    /// Evolve a single body by `dt_s` seconds of simulation time.
    /// Returns true if the body's kind changed (triggers visual update).
    bool tick(Body& b, double dt_s);

    /// Evolve all bodies in the simulation.
    void tick_all(std::vector<Body>& bodies, double dt_s);

    /// Compute a body's mean density (kg/m³). Returns 0 if radius==0.
    double density(const Body& b);

    /// Get debug string for stellar stage
    const char* stellar_class_str(StellarClass sc);
}
