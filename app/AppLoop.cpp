// =============================================================================
//  app/AppLoop.cpp  — SFML main loop implementation
// =============================================================================

#include "AppLoop.h"
#include "../physics/Gravity.h"
#include <iostream>
#include <cmath>
#include <random>

// ── Constructor ────────────────────────────────────────────────────────────────

AppLoop::AppLoop(unsigned width, unsigned height)
    : m_window(sf::VideoMode(width, height),
               "simSUS — N-Body Gravitational Simulation",
               sf::Style::Default),
      m_cam({ width, height }, 2.5e9),
      m_body_renderer(m_font),
      m_hud(m_font),
      m_input(m_sim, m_cam, m_trails),
      m_add_dialog(m_font),
      m_editor_panel(m_font),
      m_window_size(width, height)
{
    m_window.setFramerateLimit(60);

    // Initialise render texture for post-processing
    if (!m_scene_texture.create(width, height))
        std::cerr << "[Error] Could not create scene render texture.\n";

    // Load lensing shader
    if (!sf::Shader::isAvailable())
        std::cerr << "[Error] Shaders are not available on this system.\n";
    else if (!m_lensing_shader.loadFromFile("render/lensing.frag", sf::Shader::Fragment))
        std::cerr << "[Error] Could not load render/lensing.frag\n";

    // Load a system font — try several common locations
    bool font_ok = false;
    for (const char* path : {
        "C:/Windows/Fonts/consola.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/cour.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf" })
    {
        if (m_font.loadFromFile(path)) { font_ok = true; break; }
    }
    if (!font_ok)
        std::cerr << "[Warning] Could not load font — text will be absent.\n";

    generate_starfield(width, height);
    load_default_preset();

    // Init GPU Gravity if possible
    if (!Gravity::InitGPU()) {
        std::cerr << "[Warning] GPU Gravity Init failed. Falling back to CPU Physics.\n";
    }
}

AppLoop::~AppLoop()
{
    Gravity::Cleanup();
}

// ── Starfield ──────────────────────────────────────────────────────────────────

void AppLoop::generate_starfield(unsigned width, unsigned height)
{
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> rx(0.f, static_cast<float>(width));
    std::uniform_real_distribution<float> ry(0.f, static_cast<float>(height));
    std::uniform_real_distribution<float> rs(0.3f, 1.4f);
    std::uniform_int_distribution<int>    rb(160, 255);

    m_stars.reserve(300);
    for (int i = 0; i < 300; ++i)
    {
        sf::CircleShape star(rs(rng));
        star.setPosition(rx(rng), ry(rng));
        int b = rb(rng);
        star.setFillColor(sf::Color(b, b, b, 180));
        m_stars.push_back(star);
    }
}

// ── Default preset ─────────────────────────────────────────────────────────────

void AppLoop::load_default_preset()
{
    m_sim.clear_bodies();
    m_trails.clear();

    auto bodies = Presets::make_solar_system(m_sim.config().G);
    for (auto& b : bodies) m_sim.add_body(b);

    // Wire event: remove trails on collision merge
    m_sim.events.on_collision.subscribe([this](const EvCollision& ev)
    {
        m_trails.remove(ev.absorbed.id);
    });

    // Snapshot initial energy for drift diagnostics
    m_initial_energy = m_sim.diagnostics().total_energy_J;

    // Start at a lower warp for better initial control (100.0x)
    m_sim.set_time_warp(100.0);

    // Zoom to show inner solar system (Mars orbit ~2.3e11 m radius)
    m_cam.set_zoom(1.2e9);  // ~2.3e11 / 600px ≈ fits Mars orbit nicely
    m_cam.set_center({});
}

// ── Render ────────────────────────────────────────────────────────────────────

void AppLoop::draw_starfield(sf::RenderTarget& t)
{
    for (const auto& s : m_stars) t.draw(s);
}

void AppLoop::render_frame()
{
    // 1. Render scene to off-screen buffer
    m_scene_texture.clear(sf::Color(4, 4, 12));

    draw_starfield(m_scene_texture);
    m_grid.draw(m_scene_texture, m_cam, m_sim);
    m_trails.draw(m_scene_texture, m_cam);
    m_orbit_predictor.draw(m_scene_texture, m_cam);
    m_body_renderer.draw_all(m_scene_texture, m_sim.bodies(), m_cam, m_input.selected_id());
    
    m_scene_texture.display();

    // 2. Apply Lensing Post-Processing
    m_window.clear();
    
    // Find biggest black hole to focus lensing on
    const Body* bh = nullptr;
    double max_mass = 0.0;
    for (const auto& b : m_sim.bodies()) {
        if (b.alive && b.kind == BodyKind::BlackHole && b.mass_kg > max_mass) {
            max_mass = b.mass_kg;
            bh = &b;
        }
    }

    if (bh && sf::Shader::isAvailable()) {
        sf::Vector2f screen_pos = m_cam.world_to_screen(bh->pos);
        // Normalise to [0,1] for shader UVs
        sf::Vector2f uv_pos(screen_pos.x / m_window_size.x, 1.0f - (screen_pos.y / m_window_size.y));
        
        // Calculate screen-space radius of the event horizon
        float radius_pixels = m_cam.world_radius_to_screen(bh->radius_m);
        
        m_lensing_shader.setUniform("texture", sf::Shader::CurrentTexture);
        m_lensing_shader.setUniform("bh_pos", uv_pos);
        m_lensing_shader.setUniform("bh_radius", radius_pixels);
        m_lensing_shader.setUniform("aspect", static_cast<float>(m_window_size.x) / static_cast<float>(m_window_size.y));
        m_lensing_shader.setUniform("resolution", sf::Vector2f(static_cast<float>(m_window_size.x), static_cast<float>(m_window_size.y)));

        m_scene_sprite.setTexture(m_scene_texture.getTexture());
        m_window.draw(m_scene_sprite, &m_lensing_shader);
    } else {
        // No lensing, just draw raw scene
        m_scene_sprite.setTexture(m_scene_texture.getTexture());
        m_window.draw(m_scene_sprite);
    }

    // 3. Draw UI on top (not warped)
    m_hud.draw(m_window);
    m_add_dialog.draw(m_window);
    m_editor_panel.draw(m_window, m_sim);
    m_input.draw_launch_preview(m_window, m_font);

    m_window.display();
}

// ── Main loop ─────────────────────────────────────────────────────────────────

void AppLoop::run()
{
    while (m_window.isOpen())
    {
        float frame_dt = m_clock.restart().asSeconds();
        float fps      = (frame_dt > 0.0f) ? (1.0f / frame_dt) : 0.0f;

        // ── Event pump ────────────────────────────────────────────────────────
        sf::Event event;
        while (m_window.pollEvent(event))
        {
            // Let dialog consume events first when open
            if (m_add_dialog.is_open())
            {
                auto result = m_add_dialog.handle_event(event);
                if (result)
                {
                    m_sim.add_body(*result);
                    // Snapshot energy after adding body
                    m_initial_energy = m_sim.diagnostics().total_energy_J;
                }
                // Still handle Esc and window close through input handler
                if (event.type == sf::Event::Closed)
                    m_window.close();
                continue;
            }

            // 1. Editor Panel (Top-most UI)
            if (m_editor_panel.handle_event(event, m_window, m_sim))
                continue;

            // 2. Main Input (Simulation interactions)
            auto ir = m_input.handle_event(event, m_window);
            if (ir == InputResult::QuitRequested)
                m_window.close();
            else if (ir == InputResult::OpenAddBodyDialog)
            {
                auto place_at = m_input.take_place_at();
                m_add_dialog.open(m_cam, place_at);
            }
            
            // Numerical keys for fast preset switching 
            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Num7) {
                    m_sim.clear_bodies(); m_trails.clear();
                    auto bodies = Presets::make(PresetType::GalaxySmall); // 10k bodies
                    for (auto& b : bodies) m_sim.add_body(b);
                    m_initial_energy = m_sim.diagnostics().total_energy_J;
                    m_sim.set_time_warp(10.0);
                    m_cam.set_zoom(1e12);
                }
            }
        }

        // ── Continuous input (pan drag) ───────────────────────────────────────
        m_input.update(m_window);

        // ── Camera follow ─────────────────────────────────────────────────────
        m_cam.update_follow();

        // ── Physics step ──────────────────────────────────────────────────────
        if (!m_add_dialog.is_open())
            m_sim.step(static_cast<double>(frame_dt));

        // ── Trail recording (every N frames to keep deque manageable) ────────
        ++m_trail_tick;
        if (m_trail_tick >= TRAIL_RECORD_EVERY)
        {
            m_trails.record(m_sim.bodies(), m_sim.sim_time());
            m_trail_tick = 0;
        }

        // ── HUD update ────────────────────────────────────────────────────────
        const Body* sel = m_input.selected_body();
        m_hud.update(m_sim, sel, fps, m_initial_energy);
        m_editor_panel.set_selection(m_input.selected_id());

        // ── Orbit Prediction ──────────────────────────────────────────────────
        // Throttle to every 4 frames to save CPU, or update once if paused
        static int orbit_tick = 0;
        if (sel && (!m_sim.is_paused() ? (++orbit_tick >= 4) : (orbit_tick != -1)))
        {
            m_orbit_predictor.update(m_sim, sel->id);
            if (!m_sim.is_paused()) orbit_tick = 0;
            else orbit_tick = -1; // Flag that we updated while paused
        }
        if (!m_sim.is_paused() && orbit_tick == -1) orbit_tick = 0;

        // ── Render ────────────────────────────────────────────────────────────
        render_frame();
    }
}
