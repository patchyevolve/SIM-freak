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

StellarClass stellar_class_from_str(const std::string& s)
{
    if (s == "Main Seq")     return StellarClass::MainSequence;
    if (s == "Red Giant")    return StellarClass::RedGiant;
    if (s == "White Dwarf")  return StellarClass::WhiteDwarf;
    if (s == "Neutron Star") return StellarClass::NeutronStar;
    if (s == "Protostar")    return StellarClass::Protostar;
    return StellarClass::None;
}

// ── Stellar temperature → color (blackbody approximation) ────────────────────
uint32_t temperature_to_color(double T_K)
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
bool tick(Body& b, double dt_s, double speed_mult)
{
    if (!b.alive) return false;

    // Convert dt to years for precision-safe age accumulation
    const double SEC_PER_YR = 31557600.0;
    double dt_yr = (dt_s / SEC_PER_YR) * speed_mult;
    b.age_yr += dt_yr;

    bool changed = false;

    // ── Stars only ────────────────────────────────────────────────────────────
    if (b.kind == BodyKind::Star)
    {
        double rho = density(b);

        if (b.stellar_class == StellarClass::MainSequence || b.stellar_class == StellarClass::RedGiant)
        {
            // 1A. Solar Wind Mass Loss (scaled by speed_mult)
            if (b.solar_wind_power > 0.0) {
                b.mass_kg -= b.solar_wind_power * dt_s * speed_mult;
                b.mass_kg = std::max(b.mass_kg, 1.0e10); // Floor
            }

            // 1B. Hydrogen fusion: Main sequence Class
            if (b.stellar_class == StellarClass::MainSequence && b.composition.hydrogen > 0.0f)
            {
                // Scaled fusion rate (massive stars burn MUCH faster: rate ~ M²)
                double mass_ratio = b.mass_kg / 1.989e30; 
                double scaled_rate = H_FUSION_RATE_PER_S * (1.0 + mass_ratio * mass_ratio * 0.1);
                
                float burned = static_cast<float>(scaled_rate * dt_s * speed_mult);
                burned = std::min(burned, b.composition.hydrogen);
                b.composition.hydrogen -= burned;
                b.composition.helium   += burned * 0.98f;

                // Update temperature (hotter as He accumulates on main sequence)
                b.temperature_K = 5778.0 * (1.0 + 0.5 * (double)b.composition.helium);
            }

            // 1C. Helium fusion: Red Giants burn He → C/O
            if (b.stellar_class == StellarClass::RedGiant && b.composition.helium > 0.0f)
            {
                double mass_ratio = b.mass_kg / 1.989e30;
                double scaled_rate = H_FUSION_RATE_PER_S * 10.0 * (1.0 + mass_ratio * mass_ratio * 0.5);
                
                float burned = static_cast<float>(scaled_rate * dt_s * speed_mult);
                burned = std::min(burned, b.composition.helium);
                b.composition.helium -= burned;
                b.composition.carbon += burned * 0.5f;
                b.composition.oxygen += burned * 0.45f;
                
                // Red Giants get hotter as they approach the end
                b.temperature_K += dt_s * 1e-8 * speed_mult; 
            }
        }

        // 2. Red Giant transition
        if (b.stellar_class == StellarClass::MainSequence &&
            b.composition.hydrogen < H_DEPLETION_THRESHOLD)
        {
            b.stellar_class = StellarClass::RedGiant;
            b.radius_m     *= 50.0;
            b.temperature_K = 3500.0;
            b.render.color  = 0xFF4400FF;
            b.luminosity_L *= 1000.0;
            changed = true;
        }

        // 3. White Dwarf collapse
        if (b.stellar_class == StellarClass::RedGiant &&
            b.composition.helium < H_DEPLETION_THRESHOLD)
        {
            if (b.mass_kg < WHITE_DWARF_MASS_LIMIT)
            {
                b.stellar_class = StellarClass::WhiteDwarf;
                b.radius_m      = 7e6;
                b.temperature_K = 25000.0;
                b.render.color  = 0xCCCCFFFF;
                b.luminosity_L  = 0.01;
                changed = true;
            }
        }

        // 4. Density-based collapse
        if (rho > NEUTRON_STAR_DENSITY && b.stellar_class != StellarClass::NeutronStar
            && b.kind != BodyKind::BlackHole)
        {
            if (rho > BLACK_HOLE_DENSITY)
            {
                b.kind         = BodyKind::BlackHole;
                double r_s = 2.0 * 6.6743e-11 * b.mass_kg / (2.998e8 * 2.998e8);
                b.radius_m     = std::max(r_s, 1.0e4);
                b.render.color = 0x050505FF;
                b.render.event_horizon_radius_m  = static_cast<float>(r_s);
                b.render.accretion_disk_radius_m = static_cast<float>(r_s * 6.0f);
                b.render.disk_color = 0xFF8822FF;
                b.temperature_K = 0.0;
                changed = true;
            }
            else
            {
                b.stellar_class = StellarClass::NeutronStar;
                b.radius_m      = 1.2e4;
                b.temperature_K = 1.0e6;
                b.render.color  = 0xAACCFFFF;
                b.luminosity_L  = 500.0;
                changed = true;
            }
        }

        // 5. Cooling
        if (b.stellar_class == StellarClass::WhiteDwarf)
        {
            b.temperature_K -= dt_s * 1.0e-10 * speed_mult;
            b.temperature_K  = std::max(b.temperature_K, 800.0);
            b.render.color   = temperature_to_color(b.temperature_K);
        }
    }

    if (b.kind == BodyKind::Asteroid || b.kind == BodyKind::Moon)
    {
        double omega = b.vel.norm() / std::max(b.radius_m, 1.0);
        b.spin_deg  += static_cast<float>(std::fmod(omega * dt_s * (180.0 / 3.14159265), 360.0));
        if (b.spin_deg > 360.0f) b.spin_deg -= 360.0f;
    }

    return changed;
}

// ── Batch tick ─────────────────────────────────────────────────────────────────
void tick_all(std::vector<Body>& bodies, double dt_s, double speed_mult)
{
    for (auto& b : bodies)
        tick(b, dt_s, speed_mult);
}

} // namespace StellarEvolution
