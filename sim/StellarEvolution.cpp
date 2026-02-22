// =============================================================================
//  sim/StellarEvolution.cpp  — Stellar lifecycle rules engine
// =============================================================================

#include "StellarEvolution.h"
#include <cmath>
#include <algorithm>

namespace StellarEvolution
{

// ── Helpers ───────────────────────────────────────────────────────────────────

double density(const Body& b)
{
    if (b.radius_m <= 0.0) return 0.0;
    double vol = (4.0 / 3.0) * 3.14159265358979 * b.radius_m * b.radius_m * b.radius_m;
    return b.mass_kg / vol;
}

const char* stellar_class_str(StellarClass sc)
{
    switch (sc) {
    case StellarClass::MainSequence: return "Main Seq";
    case StellarClass::RedGiant:     return "Red Giant";
    case StellarClass::WhiteDwarf:   return "White Dwarf";
    case StellarClass::NeutronStar:  return "Neutron Star";
    case StellarClass::Protostar:    return "Protostar";
    default:                         return "N/A";
    }
}

// ── Stellar temperature → color (blackbody approximation) ────────────────────
static uint32_t temperature_to_color(double T_K)
{
    // Very simplified Planckian locus approximation
    uint8_t r, g, b;
    if (T_K >= 25000) { r=155; g=176; b=255; }       // Blue-white (O/B)
    else if (T_K >= 10000) { r=170; g=191; b=255; }   // Blue-white (A)
    else if (T_K >= 7500)  { r=202; g=215; b=255; }   // White (F)
    else if (T_K >= 6000)  { r=255; g=244; b=234; }   // Yellow-white (G)
    else if (T_K >= 5000)  { r=255; g=229; b=207; }   // Yellow (G/K)
    else if (T_K >= 3700)  { r=255; g=180; b=107; }   // Orange (K)
    else                   { r=255; g=100; b= 50; }    // Red (M)

    return (static_cast<uint32_t>(r) << 24) |
           (static_cast<uint32_t>(g) << 16) |
           (static_cast<uint32_t>(b) <<  8) | 0xFF;
}

// ── Single-body evolution tick ─────────────────────────────────────────────────
bool tick(Body& b, double dt_s)
{
    if (!b.alive) return false;

    b.age_s += dt_s;
    bool changed = false;

    // ── Stars only ────────────────────────────────────────────────────────────
    if (b.kind == BodyKind::Star)
    {
        double rho = density(b);

        // 1. Hydrogen fusion: slowly convert H → He over sim time
        if (b.stellar_class == StellarClass::MainSequence && b.composition.hydrogen > 0.0f)
        {
            float burned = static_cast<float>(H_FUSION_RATE_PER_S * dt_s * b.mass_kg / b.mass_kg);
            burned = std::min(burned, b.composition.hydrogen);
            b.composition.hydrogen -= burned;
            b.composition.helium   += burned * 0.98f; // ~2% mass deficit → energy

            // Update temperature (hotter as He accumulates on main sequence)
            b.temperature_K = 5778.0 * (1.0 + 0.5 * (double)b.composition.helium);
        }

        // 2. Red Giant transition: H depleted, outer shell expands
        if (b.stellar_class == StellarClass::MainSequence &&
            b.composition.hydrogen < H_DEPLETION_THRESHOLD)
        {
            b.stellar_class = StellarClass::RedGiant;
            b.radius_m     *= 50.0;           // Dramatic expansion
            b.temperature_K = 3500.0;         // Cooler surface
            b.render.color  = 0xFF4400FF;     // Deep red-orange
            b.luminosity_L *= 1000.0;
            changed = true;
        }

        // 3. White Dwarf: Red Giant exhausts He → collapse
        if (b.stellar_class == StellarClass::RedGiant &&
            b.composition.helium < H_DEPLETION_THRESHOLD)
        {
            if (b.mass_kg < WHITE_DWARF_MASS_LIMIT)
            {
                b.stellar_class = StellarClass::WhiteDwarf;
                b.radius_m      = 7e6;         // ~Earth-sized
                b.temperature_K = 25000.0;     // Very hot surface initially
                b.render.color  = 0xCCCCFFFF;  // Blue-white
                b.luminosity_L  = 0.01;
                changed = true;
            }
        }

        // 4. Density-based collapse (massive stars)
        if (rho > NEUTRON_STAR_DENSITY && b.stellar_class != StellarClass::NeutronStar
            && b.kind != BodyKind::BlackHole)
        {
            if (rho > BLACK_HOLE_DENSITY)
            {
                // Collapse to Black Hole
                b.kind         = BodyKind::BlackHole;
                double r_s = 2.0 * 6.6743e-11 * b.mass_kg / (2.998e8 * 2.998e8);
                b.radius_m     = std::max(r_s, 1.0e4);
                b.render.color = 0x050505FF;   // Near black
                b.render.event_horizon_radius_m  = static_cast<float>(r_s);
                b.render.accretion_disk_radius_m = static_cast<float>(r_s * 6.0f);
                b.render.disk_color = 0xFF8822FF;
                b.temperature_K = 0.0; // Hawking is negligible
                changed = true;
            }
            else
            {
                b.stellar_class = StellarClass::NeutronStar;
                b.radius_m      = 1.2e4;       // 12 km
                b.temperature_K = 1.0e6;       // Million K — X-ray emitter
                b.render.color  = 0xAACCFFFF;
                b.luminosity_L  = 500.0;
                changed = true;
            }
        }

        // 5. Cooling of White Dwarf / Neutron Star over time
        if (b.stellar_class == StellarClass::WhiteDwarf)
        {
            b.temperature_K -= dt_s * 1.0e-10; // Slow cooling
            b.temperature_K  = std::max(b.temperature_K, 800.0);
            b.render.color   = temperature_to_color(b.temperature_K);
        }
    }

    // ── All bodies: update spin for visual effect ─────────────────────────────
    // Asteroids and moons get a visual spin based on their angular momentum
    if (b.kind == BodyKind::Asteroid || b.kind == BodyKind::Moon)
    {
        double omega = b.vel.norm() / std::max(b.radius_m, 1.0); // rough spin estimate
        b.spin_deg  += static_cast<float>(std::fmod(omega * dt_s * (180.0 / 3.14159265), 360.0));
        if (b.spin_deg > 360.0f) b.spin_deg -= 360.0f;
    }

    return changed;
}

// ── Batch tick ─────────────────────────────────────────────────────────────────
void tick_all(std::vector<Body>& bodies, double dt_s)
{
    for (auto& b : bodies)
        tick(b, dt_s);
}

} // namespace StellarEvolution
