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
bool tick(Body& b, double dt_s, double speed_mult, const std::vector<Body>& all_bodies, const std::vector<size_t>& star_indices)
{
    if (!b.alive) return false;

    // Convert dt to years for precision-safe age accumulation
    const double SEC_PER_YR = 31557600.0;
    double dt_yr = (dt_s / SEC_PER_YR) * speed_mult;
    b.age_yr += dt_yr;

    bool changed = false;

    // ── Phase 30: Planetary Thermodynamics ────────────────────────────────────
    if (b.kind != BodyKind::Star && b.kind != BodyKind::BlackHole)
    {
        double total_flux = 0.0;
        const double sigma = 5.670373e-8; // Stefan-Boltzmann constant

        // Optimization: iterate only over pre-collected star indices
        for (size_t idx : star_indices)
        {
            const auto& other = all_bodies[idx];
            // No need to check kind here, it's pre-filtered
            double dist = b.dist_to(other);
            if (dist < other.radius_m) dist = other.radius_m;

            // Flux = L / (4 * pi * d²)
            // L_star = (L_solar_units) * 3.828e26 W
            double L_star = other.luminosity_L * 3.828e26;
            total_flux += L_star / (4.0 * 3.14159265 * dist * dist);
        }

        // Equilibrium Temp: T = (Flux * (1-albedo) / (4 * sigma))^0.25
        double albedo = 0.3; // average
        if (b.kind == BodyKind::Asteroid) albedo = 0.1;
        
        double target_T = std::pow((total_flux * (1.0 - albedo)) / (4.0 * sigma), 0.25);
        
        // Phase 30B: Tidal Heating
        // Friction from extreme gravity generates massive heat
        if (b.tidal_stress > 0.1) {
            double tidal_heat = b.tidal_stress * b.tidal_stress * 2500.0; 
            target_T += tidal_heat;
        }

        // Internal heat (radioactive decay + primordial)
        double internal_T = 50.0; 
        target_T = std::max(target_T, internal_T);

        // Thermal inertia (larger bodies change temp slower)
        double time_const = 1e5 * (b.mass_kg / 5.97e24); // Earth mass = 1e5 seconds
        time_const = std::clamp(time_const, 3600.0, 1e8);
        
        // Optimization: Exponential decay approach to prevent overshooting at high time warps
        double factor = std::exp(-(dt_s * speed_mult) / time_const);
        b.temperature_K = target_T + (b.temperature_K - target_T) * factor;

        // ── Phase 31: Atmospheric Evaporation & Ablation ─────────────────────
          // Passive bodies (dust/asteroids) are much more resistant to thermal ablation
          // Asteroids/fragments: 3500K threshold (iron/silica mix in vacuum)
          // Planets: 1500K threshold (active silicate melting/atmosphere loss)
          double melt_threshold = b.flags.is_passive ? 3500.0 : 1500.0;

          if (b.temperature_K > melt_threshold) 
          {
              // Loss of atmosphere first
              if (b.has_atmos) {
                  b.atmos.surface_density_kg_m3 *= (1.0 - 1e-7 * dt_s * speed_mult);
                  if (b.atmos.surface_density_kg_m3 < 0.01) b.has_atmos = false;
              }

              // Ablation: Mass loss + Radius shrinkage
              // Scaled mass loss: passive fragments lose mass 1000x slower to prevent instant vanishing
              double loss_scale = b.flags.is_passive ? 1e-13 : 1e-10;
              double mass_loss_rate = loss_scale * b.mass_kg * (b.temperature_K / melt_threshold);
              double lost_mass = mass_loss_rate * dt_s * speed_mult;
              b.mass_kg -= lost_mass;
              
              // Shrink radius: r = (3M / 4pi*rho)^(1/3)
              double rho = density(b);
              if (rho > 1.0) {
                  b.radius_m = std::pow((3.0 * b.mass_kg) / (4.0 * 3.14159 * rho), 0.33333);
              }

              // Survival threshold: Fragments can be much lighter than planets before vanishing
              double survival_mass = b.flags.is_passive ? 1e10 : 1e15;
              if (b.mass_kg < survival_mass) b.alive = false; 
          }
    }

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
                b.render.color  = temperature_to_color(b.temperature_K);
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
                b.render.color  = temperature_to_color(b.temperature_K);
            }
        }

        // 2. Red Giant transition
        if (b.stellar_class == StellarClass::MainSequence &&
            b.composition.hydrogen < H_DEPLETION_THRESHOLD)
        {
            b.stellar_class = StellarClass::RedGiant;
            b.radius_m     *= 100.0; // Significant expansion
            b.temperature_K = 3200.0; // Cools as it expands
            b.render.color  = temperature_to_color(b.temperature_K);
            b.luminosity_L *= 2000.0;
            changed = true;
        }

        // 3. Supernova / End of Life transition
        if (b.stellar_class == StellarClass::RedGiant &&
            b.composition.helium < H_DEPLETION_THRESHOLD)
        {
            // Low to medium mass stars (up to ~8-10 Solar Masses) -> White Dwarf
            if (b.mass_kg < 1.6e31) // ~8 M_sun threshold for supernova
            {
                b.stellar_class = StellarClass::WhiteDwarf;
                b.radius_m      = 7e6; // Earth-sized
                b.temperature_K = 25000.0;
                b.render.color  = 0xCCCCFFFF;
                b.luminosity_L  = 0.01;
                changed = true;
            }
            else // High mass stars -> Supernova remnant (Neutron Star or Black Hole)
            {
                // Determine remnant based on mass (Tolman-Oppenheimer-Volkoff limit-ish)
                if (b.mass_kg > 4.0e31) // > ~20 M_sun initial -> Black Hole
                {
                    b.kind         = BodyKind::BlackHole;
                    b.stellar_class = StellarClass::None;
                    double r_s = 2.0 * 6.6743e-11 * b.mass_kg / (2.998e8 * 2.998e8);
                    b.radius_m     = std::max(r_s, 1.0e4);
                    b.render.color = 0x050505FF;
                    b.render.event_horizon_radius_m  = static_cast<float>(r_s);
                    b.render.accretion_disk_radius_m = static_cast<float>(r_s * 6.0f);
                    b.render.disk_color = 0xFF8822FF;
                    b.temperature_K = 0.0;
                    b.luminosity_L  = 0.0;
                }
                else // 8-20 M_sun -> Neutron Star
                {
                    b.stellar_class = StellarClass::NeutronStar;
                    b.radius_m      = 1.2e4; // 12km
                    b.temperature_K = 1.0e6;
                    b.render.color  = 0xAACCFFFF;
                    b.luminosity_L  = 500.0;
                }
                changed = true;
            }
        }

        // 4. Density-based collapse (Safety check for accretion/mergers)
        if (rho > NEUTRON_STAR_DENSITY && b.stellar_class != StellarClass::NeutronStar
            && b.kind != BodyKind::BlackHole)
        {
            if (rho > BLACK_HOLE_DENSITY)
            {
                b.kind         = BodyKind::BlackHole;
                b.stellar_class = StellarClass::None;
                double r_s = 2.0 * 6.6743e-11 * b.mass_kg / (2.998e8 * 2.998e8);
                b.radius_m     = std::max(r_s, 1.0e4);
                b.render.color = 0x050505FF;
                b.render.event_horizon_radius_m  = static_cast<float>(r_s);
                b.render.accretion_disk_radius_m = static_cast<float>(r_s * 6.0f);
                b.render.disk_color = 0xFF8822FF;
                b.temperature_K = 0.0;
                b.luminosity_L  = 0.0;
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

        // 5. Cooling (Remnants)
        if (b.stellar_class == StellarClass::WhiteDwarf || b.stellar_class == StellarClass::NeutronStar)
        {
            double target_remnant_T = 2.73; // Cool towards background
            // Very slow cooling: billions of years
            double cooling_const = (b.stellar_class == StellarClass::WhiteDwarf) ? 1e17 : 1e18; 
            
            double factor = std::exp(-(dt_s * speed_mult) / cooling_const);
            b.temperature_K = target_remnant_T + (b.temperature_K - target_remnant_T) * factor;
            b.temperature_K = std::max(b.temperature_K, 2.73);
            b.render.color  = temperature_to_color(b.temperature_K);
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
    // Pre-collect active stars to avoid O(N^2) checks inside tick()
    std::vector<size_t> star_indices;
    star_indices.reserve(16);
    for (size_t i = 0; i < bodies.size(); ++i) {
        if (bodies[i].alive && bodies[i].kind == BodyKind::Star)
            star_indices.push_back(i);
    }

    for (auto& b : bodies)
        tick(b, dt_s, speed_mult, bodies, star_indices);
}

} // namespace StellarEvolution
