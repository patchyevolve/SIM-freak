// =============================================================================
//  io/State.cpp  — Versioned JSON save / load
// =============================================================================

#include "State.h"
#include "../sim/Presets.h"       // for RunTests bootstrap
#include "../third_party/nlohmann/json.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <cmath>

using json = nlohmann::json;

// ── Vec2 helpers ──────────────────────────────────────────────────────────────
static json vec2_to_json(const Vec2& v) { return { {"x", v.x}, {"y", v.y} }; }
static Vec2 json_to_vec2(const json& j) { return { j.at("x").get<double>(), j.at("y").get<double>() }; }

// ── Body serialisation ────────────────────────────────────────────────────────
static json body_to_json(const Body& b)
{
    json j;
    j["id"]        = b.id;
    j["name"]      = b.name;
    j["kind"]      = body_kind_to_str(b.kind);
    j["mass_kg"]   = b.mass_kg;
    j["radius_m"]  = b.radius_m;
    j["pos"]       = vec2_to_json(b.pos);
    j["vel"]       = vec2_to_json(b.vel);
    j["render"] = {
        {"color",             b.render.color},
        {"base_radius_px",    b.render.base_radius_px},
        {"draw_trail",        b.render.draw_trail},
        {"draw_label",        b.render.draw_label},
        {"atmosphere_color",  b.render.atmosphere_color},
        {"has_rings",         b.render.has_rings},
        {"ring_color",        b.render.ring_color}
    };
    j["flags"] = {
        {"immovable",  b.flags.immovable},
        {"no_collide", b.flags.no_collide}
    };
    // Extra metadata
    json extra = json::object();
    for (const auto& [k, v] : b.extra) extra[k] = v;
    j["extra"] = extra;
    return j;
}

static Body json_to_body(const json& j)
{
    Body b;
    b.id       = j.at("id").get<std::string>();
    b.name     = j.at("name").get<std::string>();
    b.kind     = body_kind_from_str(j.at("kind").get<std::string>());
    b.mass_kg  = j.at("mass_kg").get<double>();
    b.radius_m = j.at("radius_m").get<double>();
    b.pos      = json_to_vec2(j.at("pos"));
    b.vel      = json_to_vec2(j.at("vel"));

    if (j.contains("render"))
    {
        const auto& r = j["render"];
        b.render.color            = r.value("color",             0xFFFFFFFFu);
        b.render.base_radius_px    = r.value("base_radius_px",    6.0f);
        b.render.draw_trail       = r.value("draw_trail",        true);
        b.render.draw_label       = r.value("draw_label",        true);
        b.render.atmosphere_color = r.value("atmosphere_color",  0x00000000u);
        b.render.has_rings        = r.value("has_rings",         false);
        b.render.ring_color       = r.value("ring_color",        0xAAAAAA88u);
    }
    if (j.contains("flags"))
    {
        const auto& f = j["flags"];
        b.flags.immovable  = f.value("immovable",  false);
        b.flags.no_collide = f.value("no_collide", false);
    }
    if (j.contains("extra"))
    {
        for (const auto& [k, v] : j["extra"].items())
            if (v.is_number()) b.extra[k] = v.get<double>();
    }
    b.alive = true;
    return b;
}

// ── PhysicsConfig serialisation ───────────────────────────────────────────────
static json cfg_to_json(const PhysicsConfig& c)
{
    return {
        {"G",            c.G},
        {"softening_m",  c.softening_m},
        {"base_dt_s",    c.base_dt_s},
        {"sub_steps",    c.sub_steps},
        {"integrator",   integrator_name(c.integrator)}
    };
}

static PhysicsConfig json_to_cfg(const json& j)
{
    PhysicsConfig c;
    c.G           = j.value("G",           6.6743e-11);
    c.softening_m = j.value("softening_m", 1.0e6);
    c.base_dt_s   = j.value("base_dt_s",   3600.0);
    c.sub_steps   = j.value("sub_steps",   8);
    c.integrator  = integrator_from_str(j.value("integrator", "RK4"));
    return c;
}

// ── Save ──────────────────────────────────────────────────────────────────────
namespace IO
{

void save_state(const Simulation& sim, const std::string& path)
{
    json root;
    root["schema_version"] = SCHEMA_VERSION;
    root["sim"] = {
        {"sim_time_s",  sim.sim_time()},
        {"paused",      sim.is_paused()},
        {"time_warp",   sim.time_warp()},
        {"config",      cfg_to_json(sim.config())}
    };

    json bodies_arr = json::array();
    for (const auto& b : sim.bodies())
        if (b.alive) bodies_arr.push_back(body_to_json(b));
    root["bodies"] = bodies_arr;

    std::ofstream ofs(path);
    if (!ofs) throw std::runtime_error("IO::save_state: cannot open file: " + path);
    ofs << root.dump(2);   // pretty-print with 2-space indent
}

// ── Load ──────────────────────────────────────────────────────────────────────
void load_state(Simulation& sim, const std::string& path)
{
    std::ifstream ifs(path);
    if (!ifs) throw std::runtime_error("IO::load_state: file not found: " + path);

    json root;
    try { root = json::parse(ifs); }
    catch (const json::parse_error& e)
    { throw std::runtime_error(std::string("IO::load_state: JSON parse error: ") + e.what()); }

    int ver = root.value("schema_version", 0);
    if (ver != SCHEMA_VERSION)
        throw std::runtime_error("IO::load_state: unsupported schema_version " + std::to_string(ver));

    const auto& s = root.at("sim");
    sim.clear_bodies();
    sim.set_paused(s.value("paused", false));
    sim.set_time_warp(s.value("time_warp", 1.0));
    if (s.contains("config"))
        sim.set_config(json_to_cfg(s["config"]));

    for (const auto& jb : root.at("bodies"))
        sim.add_body(json_to_body(jb));
}

// ── Tests ─────────────────────────────────────────────────────────────────────
bool RunTests(const std::string& tmp_dir)
{
    bool ok = true;
    auto check = [&](const char* name, bool cond)
    {
        if (!cond) { std::cerr << "  [FAIL] IO::" << name << "\n"; ok = false; }
        else         std::cout << "  [PASS] IO::" << name << "\n";
    };

    std::string tmp_path = tmp_dir + "/simsus_test_save.json";

    // Build a small simulation
    Simulation sim;
    auto bodies = Presets::make_solar_system();
    for (auto& b : bodies) sim.add_body(b);
    sim.step_sim(3600.0 * 24.0);  // advance 1 day

    size_t       orig_count   = sim.bodies().size();
    double       orig_time    = sim.sim_time();
    Vec2         orig_sun_pos = sim.bodies()[0].pos;
    double       orig_sun_mass= sim.bodies()[0].mass_kg;

    // Save
    bool save_ok = true;
    try { save_state(sim, tmp_path); }
    catch (const std::exception& e) { std::cerr << "  save threw: " << e.what() << "\n"; save_ok = false; }
    check("save_no_throw", save_ok);

    // Load into a fresh simulation
    Simulation sim2;
    bool load_ok = true;
    try { load_state(sim2, tmp_path); }
    catch (const std::exception& e) { std::cerr << "  load threw: " << e.what() << "\n"; load_ok = false; }
    check("load_no_throw", load_ok);

    if (load_ok)
    {
        check("roundtrip_body_count",  sim2.bodies().size() == orig_count);
        check("roundtrip_sun_mass",    std::abs(sim2.bodies()[0].mass_kg - orig_sun_mass) < 1.0);
        check("roundtrip_sun_pos_x",   std::abs(sim2.bodies()[0].pos.x - orig_sun_pos.x) < 1.0);
    }

    // Missing file should throw
    bool threw = false;
    try { load_state(sim2, tmp_dir + "/does_not_exist_xyzzy.json"); }
    catch (...) { threw = true; }
    check("missing_file_throws", threw);

    return ok;
}

} // namespace IO
