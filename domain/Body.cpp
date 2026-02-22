// =============================================================================
//  domain/Body.cpp
// =============================================================================

#include "Body.h"
#include <cmath>
#include <cassert>
#include <stdexcept>
#include <iostream>

// ── BodyKind helpers ──────────────────────────────────────────────────────────

const char* body_kind_to_str(BodyKind k)
{
    switch (k)
    {
    case BodyKind::Star:       return "Star";
    case BodyKind::Planet:     return "Planet";
    case BodyKind::Moon:       return "Moon";
    case BodyKind::Asteroid:   return "Asteroid";
    case BodyKind::Spacecraft: return "Spacecraft";
    case BodyKind::Custom:     return "Custom";
    case BodyKind::BlackHole:  return "BlackHole";
    default:                   return "Unknown";
    }
}

BodyKind body_kind_from_str(const std::string& s)
{
    if (s == "Star")       return BodyKind::Star;
    if (s == "Planet")     return BodyKind::Planet;
    if (s == "Moon")       return BodyKind::Moon;
    if (s == "Asteroid")   return BodyKind::Asteroid;
    if (s == "Spacecraft") return BodyKind::Spacecraft;
    if (s == "BlackHole")  return BodyKind::BlackHole;
    return BodyKind::Custom;
}

// ── Inelastic merge ───────────────────────────────────────────────────────────
Body merge_bodies(const Body& a, const Body& b)
{
    // Survivor is the heavier body
    const Body& heavy = (a.mass_kg >= b.mass_kg) ? a : b;
    const Body& light = (a.mass_kg >= b.mass_kg) ? b : a;

    Body merged        = heavy;           // copy identity, render, flags from heavier
    merged.mass_kg     = a.mass_kg + b.mass_kg;

    // Momentum-conserved velocity
    merged.vel = (a.vel * a.mass_kg + b.vel * b.mass_kg) / merged.mass_kg;

    // Radius by equal-density volume addition: r_new = cbrt(r1³ + r2³)
    merged.radius_m = std::cbrt(
        a.radius_m * a.radius_m * a.radius_m +
        b.radius_m * b.radius_m * b.radius_m);

    merged.accel = Vec2{};  // will be recomputed next step
    merged.alive = true;
    merged.bodies_merged_count = a.bodies_merged_count + b.bodies_merged_count + 1;
    (void)light;            // suppress unused-variable warning
    return merged;
}

// ── Unit tests ────────────────────────────────────────────────────────────────
bool Body::RunTests()
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] Body::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] Body::" << name << "\n";
    };

    // Kinetic energy: KE = 0.5 * m * v²
    {
        Body b;
        b.mass_kg = 2.0;
        b.vel     = { 3.0, 4.0 };   // speed = 5
        check("kinetic_energy", std::abs(b.kinetic_energy() - 25.0) < 1e-10);
    }

    // Surface gravity of Earth-like body
    {
        Body b;
        b.mass_kg  = 5.972e24;
        b.radius_m = 6.371e6;
        double g   = b.surface_gravity();
        check("surface_gravity_earth", std::abs(g - 9.82) < 0.05); // ~9.8 m/s²
    }

    // Overlap detection
    {
        Body a, bb2;
        a.pos    = { 0.0, 0.0 }; a.radius_m  = 5.0;
        bb2.pos  = { 8.0, 0.0 }; bb2.radius_m = 5.0;
        check("overlaps_true",  a.overlaps(bb2));
        bb2.pos  = { 11.0, 0.0 };
        check("overlaps_false", !a.overlaps(bb2));
    }

    // Merge: mass additive, momentum conserved
    {
        Body a, b;
        a.mass_kg = 3.0; a.vel = { 1.0, 0.0 }; a.radius_m = 1.0;
        b.mass_kg = 1.0; b.vel = { 0.0, 0.0 }; b.radius_m = 1.0;
        Body m = merge_bodies(a, b);
        check("merge_mass",     std::abs(m.mass_kg - 4.0) < 1e-12);
        check("merge_vel_x",    std::abs(m.vel.x  - 0.75) < 1e-12);
        check("merge_vel_y",    std::abs(m.vel.y  - 0.0)  < 1e-12);
        // r_new = cbrt(1 + 1) ≈ 1.2599
        double r_expected = std::cbrt(2.0);
        check("merge_radius",   std::abs(m.radius_m - r_expected) < 1e-12);
    }

    // BodyKind round-trip
    {
        check("kind_roundtrip", body_kind_from_str(body_kind_to_str(BodyKind::Planet)) == BodyKind::Planet);
    }

    return ok;
}
