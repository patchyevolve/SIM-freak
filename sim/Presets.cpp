// =============================================================================
//  sim/Presets.cpp  — Built-in simulation initial conditions
//
//  All positions are in metres, all velocities in m/s, masses in kg.
//  Orbital velocities computed as v = sqrt(G * M_central / r).
// =============================================================================

#include "Presets.h"
#include "../physics/Integrators.h"
#include <cmath>
#include <iostream>
#include <random>

// ── Helper to build a colour from RGBA bytes ──────────────────────────────────
static constexpr uint32_t rgba(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255)
{
    return (static_cast<uint32_t>(r) << 24) |
           (static_cast<uint32_t>(g) << 16) |
           (static_cast<uint32_t>(b) <<  8) |
           a;
}
static constexpr uint32_t rgb(uint8_t r, uint8_t g, uint8_t b) { return rgba(r,g,b,255); }

// ── Name helpers ──────────────────────────────────────────────────────────────
const char* preset_name(PresetType p)
{
    switch (p)
    {
    case PresetType::SolarSystem: return "SolarSystem";
    case PresetType::BinaryStar:  return "BinaryStar";
    case PresetType::Figure8:     return "Figure8";
    case PresetType::BlackHole:   return "BlackHole";
    case PresetType::Collision:   return "CollisionTest";
    case PresetType::Nebula:      return "Nebula";
    case PresetType::GalaxySmall: return "GalaxySmall";
    default:                      return "Unknown";
    }
}

PresetType preset_from_str(const std::string& s)
{
    if (s == "BinaryStar") return PresetType::BinaryStar;
    if (s == "Figure8")    return PresetType::Figure8;
    if (s == "BlackHole")  return PresetType::BlackHole;
    if (s == "CollisionTest") return PresetType::Collision;
    if (s == "Nebula")     return PresetType::Nebula;
    if (s == "GalaxySmall") return PresetType::GalaxySmall;
    return PresetType::SolarSystem;
}

namespace Presets
{

// ── Dispatch ──────────────────────────────────────────────────────────────────
std::vector<Body> make(PresetType type, double G)
{
    switch (type)
    {
    case PresetType::BinaryStar:  return make_binary_star(G);
    case PresetType::Figure8:     return make_figure8(G);
    case PresetType::BlackHole:   return make_black_hole(G);
    case PresetType::Collision:   return make_collision(G);
    case PresetType::Nebula:      return make_nebula(G);
    case PresetType::GalaxySmall: return make_galaxy_small(G);
    default:                      return make_solar_system(G);
    }
}

// ── 1. Solar System (approximate, SI) ─────────────────────────────────────────
// Data source: NASA factsheets (approximate, for simulation not ephemerides)
std::vector<Body> make_solar_system(double G)
{
    auto circ_v = [&](double M_central, double r) -> double
    { return std::sqrt(G * M_central / r); };

    std::vector<Body> bodies;

    // ── Sun ──────────────────────────────────────────────────────────────────
    Body sun;
    sun.id = "sun"; sun.name = "Sun"; sun.kind = BodyKind::Star;
    sun.mass_kg = 1.989e30; sun.radius_m = 6.957e8;
    sun.pos = {}; sun.vel = {};
    sun.render = { rgb(255, 220, 50), 12.0f, false, true, 0, false };
    sun.composition = Composition::make_star(0.73f, 0.25f);
    sun.stellar_class = StellarClass::MainSequence;
    sun.temperature_K = 5778.0;
    sun.luminosity_L  = 1.0;
    bodies.push_back(sun);

    // ── Helper: create a planet ───────────────────────────────────────────────
    struct PlanetData {
        const char* id; const char* name;
        double mass_kg; double radius_m; double orbit_m;
        uint32_t color; float px_radius;
        uint32_t atmos_color; bool rings;
    };

    const PlanetData planets[] = {
        { "mercury", "Mercury", 3.301e23, 2.440e6, 5.791e10, rgb(169,169,169), 3.0f, 0,                    false },
        { "venus",   "Venus",   4.867e24, 6.052e6, 1.082e11, rgb(230,180, 80), 4.0f, rgba(255,255,150,120), false },
        { "earth",   "Earth",   5.972e24, 6.371e6, 1.496e11, rgb( 50,120,200), 4.0f, rgba(100,200,255,180), false },
        { "mars",    "Mars",    6.417e23, 3.390e6, 2.279e11, rgb(180, 80, 40), 3.5f, rgba(255,100, 50, 80), false },
        { "jupiter", "Jupiter", 1.898e27, 7.149e7, 7.783e11, rgb(200,160,100), 8.0f, rgba(255,220,150, 60), false },
        { "saturn",  "Saturn",  5.683e26, 6.027e7, 1.427e12, rgb(210,190,130), 7.0f, rgba(210,190,130, 40), true  },
        { "uranus",  "Uranus",  8.681e25, 2.556e7, 2.871e12, rgb(150,220,230), 5.5f, rgba(150,255,255, 60), false },
        { "neptune", "Neptune", 1.024e26, 2.476e7, 4.498e12, rgb( 60, 80,200), 5.5f, rgba(100,120,255, 60), false },
    };

    // Composition + temperatures per planet
    struct PlanetExtra { Composition comp; double temp_K; };
    const PlanetExtra extras[] = {
        { Composition::make_rocky(0.33f, 0.17f, 0.25f), 440.0 },   // Mercury
        { Composition::make_rocky(0.08f, 0.15f, 0.45f), 737.0 },   // Venus (thick CO2)
        { Composition::make_rocky(0.32f, 0.15f, 0.30f), 288.0 },   // Earth
        { Composition::make_rocky(0.20f, 0.18f, 0.40f), 210.0 },   // Mars
        { Composition::make_gas_giant(),                  165.0 },   // Jupiter
        { Composition::make_gas_giant(),                  134.0 },   // Saturn
        { Composition::make_ice_giant(),                   76.0 },   // Uranus
        { Composition::make_ice_giant(),                   72.0 },   // Neptune
    };

    double M_sun = sun.mass_kg;
    for (int pi = 0; pi < 8; ++pi)
    {
        const auto& pd = planets[pi];
        Body p;
        p.id = pd.id; p.name = pd.name; p.kind = BodyKind::Planet;
        p.mass_kg = pd.mass_kg; p.radius_m = pd.radius_m;
        p.pos = { pd.orbit_m, 0.0 };
        p.vel = { 0.0, circ_v(M_sun, pd.orbit_m) };  // counter-clockwise
        p.render = { pd.color, pd.px_radius, true, true, pd.atmos_color, pd.rings };
        p.composition = extras[pi].comp;
        p.temperature_K = extras[pi].temp_K;

        // Phase 29: Atmospheric Data
        if (p.id == "earth") {
            p.has_atmos = true;
            p.atmos.surface_density_kg_m3 = 1.225;
            p.atmos.scale_height_m = 8500.0;
        } else if (p.id == "venus") {
            p.has_atmos = true;
            p.atmos.surface_density_kg_m3 = 65.0; // very thick
            p.atmos.scale_height_m = 15900.0;
        } else if (p.id == "mars") {
            p.has_atmos = true;
            p.atmos.surface_density_kg_m3 = 0.020; // thin
            p.atmos.scale_height_m = 11100.0;
        }

        bodies.push_back(p);
    }

    return bodies;
}

// ── 2. Binary Star + Planet ───────────────────────────────────────────────────
std::vector<Body> make_binary_star(double G)
{
    std::vector<Body> bodies;

    const double M_star = 1.0e30;     // kg each
    const double sep    = 4.0e11;     // separation (m)
    const double r_orbit = sep / 2.0;

    double v_star = std::sqrt(G * M_star / (2.0 * sep));

    Body s1, s2;
    s1.id = "star1"; s1.name = "Star A"; s1.kind = BodyKind::Star;
    s1.mass_kg = M_star; s1.radius_m = 5e8;
    s1.pos = { -r_orbit, 0.0 }; s1.vel = { 0.0, -v_star };
    s1.render = { 0xFF8800FF, 10.0f, true, true, 0, false };

    s2.id = "star2"; s2.name = "Star B"; s2.kind = BodyKind::Star;
    s2.mass_kg = M_star; s2.radius_m = 5e8;
    s2.pos = {  r_orbit, 0.0 }; s2.vel = { 0.0,  v_star };
    s2.render = { 0x4488FFFF, 10.0f, true, true, 0, false };

    bodies.push_back(s1);
    bodies.push_back(s2);

    Body planet;
    planet.id = "bin_planet"; planet.name = "Planet C"; planet.kind = BodyKind::Planet;
    planet.mass_kg = 6.0e24; planet.radius_m = 7e6;
    double r_planet = 3.0 * sep;
    double M_total  = 2.0 * M_star;
    planet.pos = { r_planet, 0.0 };
    planet.vel = { 0.0, std::sqrt(G * M_total / r_planet) };
    planet.render = { 0x33CC33FF, 4.5f, true, true, rgba(100,255,200,100), false };

    bodies.push_back(planet);
    return bodies;
}

// ── 3. Figure-8 Three-Body (Chenciner & Montgomery, 2000) ─────────────────────
// Classic choreography: 3 equal masses chasing each other on a figure-8 path.
// ICs from: https://arxiv.org/abs/math/0011268  (normalised then scaled)
std::vector<Body> make_figure8(double G)
{
    // Normalised units: m=1, G=1, scale to physical
    // We scale: position by L=1e11 m, velocity so that G*m/L matches
    // With m=1e26 kg, L=1e11 m:
    //   time_unit = sqrt(L^3 / (G*m)) ≈ sqrt(1e33 / (6.67e-11 * 1e26)) ≈ 1.22e9 s
    const double m   = 1.0e26;   // kg
    const double L   = 1.0e11;   // m
    double T = std::sqrt(L*L*L / (G * m));  // time unit

    // Normalised positions and velocities (Chenciner & Montgomery)
    // Body 1: x=0.97000436, y=-0.24308753, vx=0.93240737/2, vy=0.86473146/2
    // Body 2: x=-0.97000436, y=0.24308753, vx= same
    // Body 3: x=0, y=0,                   vx=-0.93240737,  vy=-0.86473146
    struct IC { double x, y, vx, vy; };
    IC ics[3] = {
        {  0.97000436, -0.24308753,  0.93240737/2.0,  0.86473146/2.0 },
        { -0.97000436,  0.24308753,  0.93240737/2.0,  0.86473146/2.0 },
        {  0.0,          0.0,       -0.93240737,      -0.86473146      }
    };

    const uint32_t colors[3] = { 0xFF4444FF, 0x44FF44FF, 0x4444FFFF };
    const char* names[3]     = { "Body A", "Body B", "Body C" };
    const char* ids[3]       = { "fig8_a", "fig8_b", "fig8_c" };

    std::vector<Body> bodies;
    for (int i = 0; i < 3; ++i)
    {
        Body b;
        b.id = ids[i]; b.name = names[i]; b.kind = BodyKind::Custom;
        b.mass_kg = m; b.radius_m = 3e8;
        b.pos = { ics[i].x * L, ics[i].y * L };
        b.vel = { ics[i].vx * L / T, ics[i].vy * L / T };
        b.render = { colors[i], 5.0f, true, true };
        bodies.push_back(b);
    }
    return bodies;
}

// ── Unit tests ────────────────────────────────────────────────────────────────
bool RunTests()
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] Presets::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] Presets::" << name << "\n";
    };

    const double G     = 6.6743e-11;
    const double soft  = 1e6;

    for (auto ptype : { PresetType::SolarSystem, PresetType::BinaryStar, PresetType::Figure8 })
    {
        auto bodies = make(ptype, G);
        check((std::string("non_empty_") + preset_name(ptype)).c_str(), !bodies.empty());

        // All positions/velocities must be finite before stepping
        bool all_finite = true;
        for (const auto& b : bodies)
            if (!b.pos.is_finite() || !b.vel.is_finite()) all_finite = false;
        check((std::string("finite_ics_") + preset_name(ptype)).c_str(), all_finite);

        // Step 50 ticks — no NaN/Inf
        auto bods = bodies;
        bool no_nan = true;
        for (int i = 0; i < 50; ++i)
        {
            for (auto& b : bods) b.accel = Vec2{};
            // Gravity + RK4 step
            Integrators::step_rk4(bods, 3600.0, G, soft);
            for (const auto& b : bods)
                if (!b.pos.is_finite() || !b.vel.is_finite()) no_nan = false;
        }
        check((std::string("no_nan_50steps_") + preset_name(ptype)).c_str(), no_nan);
    }

    return ok;
}

// ── 4. Gargantua (Supermassive Black Hole) ──────────────────────────────────
std::vector<Body> make_black_hole(double G)
{
    std::vector<Body> bodies;

    // The Singularity (Supermassive)
    Body bh;
    bh.id = "gargantua"; bh.name = "Gargantua"; bh.kind = BodyKind::BlackHole;
    bh.mass_kg = 8.0e36; // SMBH
    bh.radius_m = 5.0e9; // Visual horizon size
    bh.pos = {}; bh.vel = {};
    bh.render = { 0x000000FF, 15.0f, false, true };
    bh.render.disk_color = 0xFFAA44FF; 
    bodies.push_back(bh);

    // Companion Star in stable orbit
    Body s;
    s.id = "companion"; s.name = "Companion Star"; s.kind = BodyKind::Star;
    s.mass_kg = 2.0e30;
    s.radius_m = 7.0e8;
    double orbit_r = 1.2e12; // AU range
    s.pos = { orbit_r, 0.0 };
    s.vel = { 0.0, std::sqrt(G * bh.mass_kg / orbit_r) };
    s.render = { rgb(150, 200, 255), 8.0f, true, true };
    bodies.push_back(s);

    // Some asteroids in the accretion zone
    for (int i = 0; i < 15; ++i) {
        float r_off = static_cast<float>(3.0e11 + i * 2e10);
        float angle = i * 0.4f;
        Body a;
        a.id = "acc_" + std::to_string(i); a.name = "Belt Piece"; a.kind = BodyKind::Asteroid;
        a.mass_kg = 1e22;
        a.pos = { std::cos(angle) * r_off, std::sin(angle) * r_off };
        double v = std::sqrt(G * bh.mass_kg / r_off);
        a.vel = { -std::sin(angle) * v, std::cos(angle) * v };
        a.render = { 0x888888FF, 3.0f, true, false };
        bodies.push_back(a);
    }

    return bodies;
}

std::vector<Body> make_collision(double G)
{
    std::vector<Body> bodies;

    // Body 1: Planet
    Body p1;
    p1.id = "PlanetA";
    p1.name = "Terra";
    p1.kind = BodyKind::Planet;
    p1.mass_kg = 5.972e24;
    p1.radius_m = 6.371e6;
    p1.pos = { -2e7, 0 };
    p1.vel = { 2500, 0 };
    p1.composition.oxygen = 0.45f;
    bodies.push_back(p1);

    // Body 2: Impacting Planet
    Body p2;
    p2.id = "PlanetB";
    p2.name = "Theia";
    p2.kind = BodyKind::Planet;
    p2.mass_kg = 4.867e24;
    p2.radius_m = 6.052e6;
    p2.pos = { 2e7, 1e6 }; // slight offset for realistic grazing
    p2.vel = { -2500, 100 };
    p2.composition.oxygen = 0.3f;
    bodies.push_back(p2);

    return bodies;
}

std::vector<Body> make_nebula(double G)
{
    std::vector<Body> bodies;

    // Central Star (Heavy)
    Body star;
    star.id = "Sol";
    star.name = "Proto-Sun";
    star.kind = BodyKind::Star;
    star.mass_kg = 1.989e30;
    star.radius_m = 6.957e8;
    star.pos = { 0, 0 };
    star.vel = { 0, 0 };
    bodies.push_back(star);

    // Proto-disc: 2000 passive fragments
    const int num_particles = 2000;
    for (int i = 0; i < num_particles; ++i)
    {
        double r = 4e11 + (rand() % 1000) * 1e9; // 4 to 14 AU
        double angle = (rand() % 3600) * 0.1 * (3.14159 / 180.0);
        
        Body p;
        p.id = "p_" + std::to_string(i);
        p.name = "Dust";
        p.kind = BodyKind::Asteroid;
        p.mass_kg = 1e20; // light
        p.radius_m = 5e6;
        p.pos = { r * std::cos(angle), r * std::sin(angle) };
        
        double v = std::sqrt(G * star.mass_kg / r);
        p.vel = { -v * std::sin(angle), v * std::cos(angle) };
        
        p.flags.is_passive = true; // Optimization: they don't exert gravity
        bodies.push_back(p);
    }

    return bodies;
}

std::vector<Body> make_galaxy_small(double G)
{
    std::vector<Body> bodies;
    bodies.reserve(10001);

    // Central Supermassive Black Hole
    Body bh;
    bh.id = "core"; bh.name = "Galactic Core"; bh.kind = BodyKind::BlackHole;
    bh.mass_kg = 2.0e37; // ~10 million solar masses (still huge, but smaller Rs)
    bh.radius_m = 1.0e10;
    bh.pos = {}; bh.vel = {};
    bh.render = { 0x000000FF, 15.0f, false, true };
    bh.render.disk_color = 0xAA66FFFF; // Purple accretion glow
    bodies.push_back(bh);

    // 10,000 orbiting stars (passive for gravity, but affected by BH)
    // We use a randomized distribution to prevent artificial lattice patterns.
    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> dist_angle(0.0, 2.0 * 3.14159265);
    // Megascale Expansion: R_min = 3e13 (~200 AU) to R_max = 5e15 (~33,000 AU)
    // Using sqrt(uniform) gives constant density per unit area (true disk feel).
    std::uniform_real_distribution<double> dist_r2(9e26, 2.5e31); 

    for (int i = 0; i < 10000; ++i) {
        double r = std::sqrt(dist_r2(rng));
        double angle = dist_angle(rng);
        
        Body s;
        s.id = "s" + std::to_string(i);
        s.name = "Star";
        s.kind = BodyKind::Star;
        s.mass_kg = 2.0e30;
        s.radius_m = 7.0e8;
        s.pos = { r * std::cos(angle), r * std::sin(angle) };
        
        // Circular orbital velocity: v = sqrt(G * M / r)
        double v = std::sqrt(G * bh.mass_kg / r);
        // Tangential velocity vector
        s.vel = { -v * std::sin(angle), v * std::cos(angle) };
        
        s.flags.is_passive = true;
        s.render = { rgb(180 + (i % 75), 200 + (i % 55), 255), 1.2f, false, false };
        bodies.push_back(s);
    }
    return bodies;
}

} // namespace Presets
