// =============================================================================
//  main.cpp  — simSUS CLI demo + per-module test runner + window launcher
//
//  Usage:
//    simSUS.exe                        — opens SFML window (Solar System)
//    simSUS.exe --test                 — only unit tests (console)
//    simSUS.exe --cli                  — CLI demo only (no window)
//    simSUS.exe --preset solar         — 365-tick CLI demo
//    simSUS.exe --preset binary        — binary star CLI demo
//    simSUS.exe --preset figure8       — figure-8 CLI demo
//    simSUS.exe --save <path>          — save solar system snapshotD:\codeWorks\simPUS\render\grid.vert
//    simSUS.exe --load <path>          — load and run snapshot
//
//  Build in Visual Studio:
//    Phase 1 (CLI only):  add all .cpp files, C++17, Console subsystem
//    Phase 2 (window):    also add render/*.cpp + app/*.cpp, link SFML via NuGet
//                         (install SFML_VS2019 from NuGet Package Manager)
// =============================================================================

#include "math/Vec2.h"
#include "domain/Body.h"
#include "physics/Gravity.h"
#include "physics/Integrators.h"
#include "sim/Simulation.h"
#include "sim/Presets.h"
#include "io/State.h"

// Phase 2 window — compiled only when SFML headers are available.
// If you are on Phase 1 (CLI only), comment out the next two lines.
#define SIMSUS_WINDOW_ENABLED
#ifdef SIMSUS_WINDOW_ENABLED
#include "app/AppLoop.h"
#endif

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

// ── Helpers ───────────────────────────────────────────────────────────────────

static void print_separator(const char* title)
{
    std::cout << "\n======================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "======================================================\n";
}

static std::string format_sci(double v, int prec = 3)
{
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(prec) << v;
    return oss.str();
}

static void print_body(const Body& b, double G = 6.6743e-11)
{
    std::cout << std::left << std::setw(12) << b.name
              << "  mass="    << format_sci(b.mass_kg)
              << "  r="       << format_sci(b.radius_m)
              << "  pos=("    << format_sci(b.pos.x, 2)
              <<  ", "        << format_sci(b.pos.y, 2) << ")"
              << "  v="       << format_sci(b.speed())
              << "  g_surf="  << format_sci(b.surface_gravity(G))
              << "\n";
}

// ── Unit test runner ──────────────────────────────────────────────────────────

static bool run_all_tests()
{
    struct TestSuite { const char* name; bool(*fn)(); };

    std::vector<TestSuite> suites = {
        { "Vec2",        []{ return Vec2::RunTests();         } },
        { "Body",        []{ return Body::RunTests();         } },
        { "Gravity",     []{ return Gravity::RunTests();      } },
        { "Integrators", []{ return Integrators::RunTests();  } },
        { "Simulation",  []{ return Simulation::RunTests();   } },
        { "Presets",     []{ return Presets::RunTests();      } },
        { "IO",          []{ return IO::RunTests(".");        } },
    };

    int passed = 0, failed = 0;
    for (const auto& s : suites)
    {
        print_separator(s.name);
        bool ok = s.fn();
        std::cout << (ok ? "  --> ALL PASSED\n" : "  --> SOME FAILED\n");
        ok ? ++passed : ++failed;
    }

    print_separator("SUMMARY");
    std::cout << "  Suites passed: " << passed << " / " << (passed + failed) << "\n";
    return failed == 0;
}

// ── Preset demo runner ────────────────────────────────────────────────────────

static void run_preset_demo(PresetType ptype, int ticks = 365, double step_s = 3600.0 * 24.0)
{
    print_separator(preset_name(ptype));
    std::cout << "  Stepping " << ticks << " ticks of "
              << step_s / 3600.0 << " hours each...\n\n";

    PhysicsConfig cfg;
    cfg.sub_steps = 8;
    Simulation sim(cfg);

    auto bodies = Presets::make(ptype);
    for (auto& b : bodies) sim.add_body(b);

    // Wire up collision printer
    sim.events.on_collision.subscribe([](const EvCollision& ev)
    {
        std::cout << "  [COLLISION] " << ev.absorbed.name
                  << " absorbed into " << ev.survivor.name << "\n";
    });

    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < ticks; ++i)
        sim.step_sim(step_s);
    auto t1   = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Print final state
    auto diag = sim.diagnostics();
    std::cout << "  sim_time = "  << format_sci(diag.sim_time_s)   << " s\n";
    std::cout << "  bodies   = "  << diag.body_count               << "\n";
    std::cout << "  energy   = "  << format_sci(diag.total_energy_J) << " J\n";
    std::cout << "  momentum = (" << format_sci(diag.total_momentum.x)
              << ", " << format_sci(diag.total_momentum.y) << ") kg·m/s\n";
    std::cout << "  wallclock: "  << std::fixed << std::setprecision(2) << ms << " ms\n\n";

    std::cout << "  Final body states:\n";
    for (const auto& b : sim.bodies())
        print_body(b, cfg.G);
}

// ── Save/load demo ────────────────────────────────────────────────────────────

static void run_save_demo(const std::string& path)
{
    print_separator("SAVE DEMO");
    Simulation sim;
    auto bodies = Presets::make_solar_system();
    for (auto& b : bodies) sim.add_body(b);
    sim.step_sim(3600.0 * 24.0 * 30.0);   // 30 simulated days

    IO::save_state(sim, path);
    std::cout << "  Saved " << sim.bodies().size() << " bodies to: " << path << "\n";
}

static void run_load_demo(const std::string& path)
{
    print_separator("LOAD DEMO");
    Simulation sim;
    IO::load_state(sim, path);
    std::cout << "  Loaded " << sim.bodies().size() << " bodies from: " << path << "\n";
    for (const auto& b : sim.bodies())
        print_body(b);
}

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    std::vector<std::string> args(argv + 1, argv + argc);

    auto has_flag = [&](const std::string& f) {
        for (const auto& a : args) if (a == f) return true; return false;
    };
    auto arg_after = [&](const std::string& f) -> std::string {
        for (size_t i = 0; i + 1 < args.size(); ++i)
            if (args[i] == f) return args[i + 1];
        return {};
    };

    // ── Window mode (default when no args) ────────────────────────────────────
#ifdef SIMSUS_WINDOW_ENABLED
    if (args.empty() || has_flag("--window"))
    {
        AppLoop app(1280, 800);
        app.run();
        return 0;
    }
#endif

    std::cout << "=======================================================\n";
    std::cout << "  simSUS — C++ Simulation Engine  (CLI Mode)\n";
    std::cout << "=======================================================\n";

    bool tests_only = has_flag("--test");
    bool all_mode   = has_flag("--cli") || tests_only;

    bool all_ok = true;

    // ── Tests ────────────────────────────────────────────────────────────────
    if (all_mode || has_flag("--test"))
    {
        all_ok = run_all_tests();
        if (tests_only) return all_ok ? 0 : 1;
    }

    // ── Presets ───────────────────────────────────────────────────────────────
    if (all_mode || has_flag("--preset"))
    {
        std::string which = arg_after("--preset");
        if (which.empty() || which == "solar")
            run_preset_demo(PresetType::SolarSystem, 365, 3600.0 * 24.0);
        else if (which == "binary")
            run_preset_demo(PresetType::BinaryStar, 200, 3600.0 * 12.0);
        else if (which == "figure8")
            run_preset_demo(PresetType::Figure8,    300, 3600.0 * 6.0);
        else
            std::cerr << "  Unknown preset: " << which << "\n";
    }

    // ── Save / Load ───────────────────────────────────────────────────────────
    if (has_flag("--save"))
    {
        std::string path = arg_after("--save");
        if (path.empty()) path = "simsus_save.json";
        run_save_demo(path);
    }

    if (has_flag("--load"))
    {
        std::string path = arg_after("--load");
        if (path.empty()) { std::cerr << "  --load requires a path\n"; return 1; }
        run_load_demo(path);
    }

    print_separator("DONE");
    return all_ok ? 0 : 1;
}
