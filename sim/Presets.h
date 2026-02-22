#pragma once
// =============================================================================
//  sim/Presets.h  — Built-in initial condition generators
// =============================================================================

#include "../domain/Body.h"
#include <vector>
#include <string>

enum class PresetType : uint8_t
{
    SolarSystem = 0,
    BinaryStar  = 1,
    Figure8     = 2,
    BlackHole   = 3,
    Collision   = 4,
    Nebula      = 5,
    GalaxySmall = 6,
    StellarDeath = 7
};

const char* preset_name(PresetType p);
PresetType  preset_from_str(const std::string& s);

namespace Presets
{
    /// Returns a vector of bodies for the chosen preset
    std::vector<Body> make(PresetType type, double G = 6.6743e-11);

    std::vector<Body> make_solar_system(double G = 6.6743e-11);
    std::vector<Body> make_binary_star (double G = 6.6743e-11);
    std::vector<Body> make_figure8     (double G = 6.6743e-11);
    std::vector<Body> make_black_hole (double G = 6.6743e-11);
    std::vector<Body> make_collision  (double G = 6.6743e-11);
    std::vector<Body> make_nebula     (double G = 6.6743e-11);
    std::vector<Body> make_galaxy_small(double G = 6.6743e-11);
    std::vector<Body> make_stellar_death(double G = 6.6743e-11);

    bool RunTests();
}
