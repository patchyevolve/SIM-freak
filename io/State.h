#pragma once
// =============================================================================
//  io/State.h  — Save / Load simulation state as versioned JSON
//  Uses nlohmann/json (single-header, MIT license)
//  Place json.hpp at:  third_party/nlohmann/json.hpp
// =============================================================================

#include "../sim/Simulation.h"
#include <string>
#include <stdexcept>

namespace IO
{
    /// Current schema version. Increment when the format changes.
    constexpr int SCHEMA_VERSION = 1;

    // ─────────────────────────────────────────────────────────────────────────
    /// Serialise the simulation to a JSON file at `path`.
    /// Throws std::runtime_error on write failure.
    void save_state(const Simulation& sim, const std::string& path);

    /// Deserialise a JSON file into an existing Simulation object.
    /// Throws std::runtime_error on parse error, schema mismatch or bad data.
    void load_state(Simulation& sim, const std::string& path);

    // ─────────────────────────────────────────────────────────────────────────
    bool RunTests(const std::string& tmp_dir = ".");
}
