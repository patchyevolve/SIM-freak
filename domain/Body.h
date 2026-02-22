#pragma once
// =============================================================================
//  domain/Body.h  — Celestial body representation for simSUS
// =============================================================================

#include "../math/Vec2.h"
#include <cstdint>
#include <string>
#include <unordered_map>

// ── Body classification ───────────────────────────────────────────────────────
enum class BodyKind : uint8_t
{
    Star      = 0,
    Planet    = 1,
    Moon      = 2,
    Asteroid  = 3,
    Spacecraft= 4,
    Custom    = 5,
    BlackHole = 6
};

/// Convert BodyKind ↔ string for serialisation
const char* body_kind_to_str(BodyKind k);
BodyKind    body_kind_from_str(const std::string& s);

// ── Stellar classification ────────────────────────────────────────────────────
enum class StellarClass : uint8_t
{
    MainSequence  = 0, // Normal star (fusing H)
    RedGiant      = 1, // H exhausted, expanding shell
    WhiteDwarf    = 2, // Cooling remnant, no fusion
    NeutronStar   = 3, // Ultra-dense remnant
    Protostar     = 4, // Still collapsing
    None          = 5  // Not a star
};

// ── Elemental composition ─────────────────────────────────────────────────────
/// Fractions 0..1, should sum to ≤ 1.0. Remaining fraction = trace elements.
struct Composition
{
    float hydrogen = 0.0f;
    float helium   = 0.0f;
    float carbon   = 0.0f;
    float oxygen   = 0.0f;
    float iron     = 0.0f;
    float silicon  = 0.0f;
    float ice      = 0.0f;  // water ice (H₂O)
    float rock     = 0.0f;  // silicate rock

    float total() const { return hydrogen+helium+carbon+oxygen+iron+silicon+ice+rock; }

    /// Preset helpers
    static Composition make_star(float h=0.73f, float he=0.25f)  { Composition c; c.hydrogen=h; c.helium=he; c.carbon=0.01f; c.oxygen=0.01f; return c; }
    static Composition make_rocky(float fe=0.32f, float si=0.15f, float o=0.30f) { Composition c; c.iron=fe; c.silicon=si; c.oxygen=o; c.rock=1.0f-fe-si-o; return c; }
    static Composition make_gas_giant() { Composition c; c.hydrogen=0.90f; c.helium=0.09f; c.ice=0.01f; return c; }
    static Composition make_asteroid()  { Composition c; c.rock=0.60f; c.iron=0.25f; c.ice=0.15f; return c; }
    static Composition make_ice_giant() { Composition c; c.ice=0.55f; c.hydrogen=0.20f; c.helium=0.05f; c.rock=0.20f; return c; }
};

// ── Atmospheric properties ────────────────────────────────────────────────────
struct AtmosphereInfo
{
    double surface_density_kg_m3 = 1.225; // Earth-like
    double scale_height_m        = 8500.0; // Earth-like (8.5km)
    double drag_coeff             = 2.2;    // typical for satellites
};

// ── Visual representation ─────────────────────────────────────────────────────
struct RenderProps
{
    uint32_t color             = 0xFFFFFFFF; ///< RGBA packed
    float    base_radius_px    = 6.0f;       ///< screen radius (overridden by zoom)
    bool     draw_trail        = true;
    bool     draw_label        = true;

    // Phase 9: High-Fidelity Visuals
    uint32_t atmosphere_color  = 0x00000000; ///< 0x00... means no atmosphere
    bool     has_rings         = false;
    uint32_t ring_color        = 0xAAAAAA88;

    // Phase 20: Black Hole properties
    float    event_horizon_radius_m = 0.0f; 
    float    accretion_disk_radius_m = 0.0f; 
    uint32_t disk_color        = 0xFFAA44FF; 
    uint32_t magnetosphere_color = 0x64C8FF44; // Subtle blue glow
    float    aurora_intensity  = 0.0f;       // 0..1 based on solar wind interaction

    bool has_atmosphere() const { return (atmosphere_color & 0xFF) > 0; }
};

// ── Runtime flags ─────────────────────────────────────────────────────────────
struct BodyFlags
{
    bool immovable = false; ///< infinite-mass anchor (never moves)
    bool no_collide= false; ///< skip collision for this body
    bool is_passive = false; ///< respond to gravity but don't exert it (dust/debris)
};

// ── The main body struct ──────────────────────────────────────────────────────
struct Body
{
    // Identity
    std::string id;          ///< UUID or unique name string
    std::string name;
    BodyKind    kind   = BodyKind::Custom;

    // Physical state (SI)
    double mass_kg   = 1.0;
    double radius_m  = 1.0;
    Vec2   pos;              ///< metres
    Vec2   vel;              ///< m/s
    Vec2   accel;            ///< m/s² — computed by physics, not persisted

    // Phase 21: Relativistic state
    double local_time_scale = 1.0;   ///< Multiplier for delta time (dilation)
    double proper_time_s    = 0.0;   ///< Accumulated time in body's local frame

    // Phase 25: Composition & stellar state
    Composition   composition;                          ///< Elemental fractions
    StellarClass  stellar_class = StellarClass::None;   ///< For stars
    double        temperature_K = 5778.0;               ///< Surface temperature (K)
    double        luminosity_L  = 1.0;                  ///< In Solar luminosities
    double        age_yr        = 0.0;                  ///&lt; Simulated age in years
    double        spin_deg      = 0.0;                  ///< Visual spin angle (asteroids/moons)
    double        magnetic_field_T = 0.0;               ///< Magnetic field strength (Tesla)
    double        radiation_pressure = 0.0;             ///< For stars: Pa at 1 AU
    double        solar_wind_power   = 0.0;             ///< For stars: kg/s of mass loss

    // Rendering & flags
    RenderProps render;
    BodyFlags   flags;
    
    // Phase 29: Atmosphere
    bool           has_atmos = false; // flag to enable drag calc
    AtmosphereInfo atmos;

    // Extra metadata (fuel, temperature, atmosphere, …)
    std::unordered_map<std::string, double> extra;

    // Alive flag — bodies marked false are swept out after collision merge
    bool     alive = true;
    uint32_t bodies_merged_count = 0; 

    // ── Derived stats ─────────────────────────────────────────────────────────

    /// Current speed (m/s)
    double speed() const { return vel.norm(); }

    /// Kinetic energy (J)
    double kinetic_energy() const { return 0.5 * mass_kg * vel.norm_sq(); }

    /// Surface gravitational acceleration (m/s²)
    ///  g = G*m / r²
    double surface_gravity(double G = 6.6743e-11) const
    {
        return (radius_m > 0.0) ? G * mass_kg / (radius_m * radius_m) : 0.0;
    }

    /// Estimated circular orbital speed at distance r from this body (m/s)
    double orbital_speed_at(double r_m, double G = 6.6743e-11) const
    {
        return std::sqrt(G * mass_kg / r_m);
    }

    /// Distance to another body's centre (m)
    double dist_to(const Body& other) const { return pos.dist_to(other.pos); }

    /// True if this body overlaps with other (collision trigger)
    bool overlaps(const Body& other) const
    {
        return dist_to(other) <= (radius_m + other.radius_m);
    }

    // ── Tests ─────────────────────────────────────────────────────────────────
    static bool RunTests();
};

// ── Merge two bodies (inelastic collision) ────────────────────────────────────
/// Returns the merged body (survivor is whichever has larger mass).
/// The loser body should be marked alive=false by the caller.
Body merge_bodies(const Body& a, const Body& b);
