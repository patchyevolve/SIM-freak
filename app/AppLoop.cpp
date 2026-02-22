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
    m_window.setVerticalSyncEnabled(true);

    // Initialise render texture for post-processing
    if (!m_scene_texture.create(width, height))
        std::cerr << "[Error] Could not create scene render texture.\n";
    m_scene_sprite.setTexture(m_scene_texture.getTexture());

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

    // ── Event Subscriptions ──────────────────────────────────────────────────
    
    // Cleanup trails when a body is absorbed by a black hole
    m_sim.events.on_bh_absorption.subscribe([this](const EvBhAbsorption& ev) {
        m_trails.remove(ev.absorbed_id);
    });

    // Reset initial energy tracking when a new preset is loaded
    m_sim.events.on_preset_loaded.subscribe([this](const EvPresetLoaded& ev) {
        m_initial_energy = m_sim.diagnostics().total_energy_J;
    });

    // Init GPU Gravity if possible
    if (!Gravity::InitGPU()) {
        std::cerr << "[Warning] GPU Gravity Init failed. Falling back to CPU Physics.\n";
    }

    // ── Initial Load ──────────────────────────────────────────────────────────
    // Wire up trail cleanup here so it's globally active
    m_sim.events.on_collision.subscribe([this](const EvCollision& ev) {
        m_trails.remove(ev.absorbed.id);
    });

    // Use InputHandler to load the initial preset to centralize zoom/time settings
    m_input.load_preset(PresetType::SolarSystem);
}

AppLoop::~AppLoop()
{
    Gravity::Cleanup();
}

// ── Starfield ──────────────────────────────────────────────────────────────────

void AppLoop::generate_starfield(unsigned width, unsigned height)
{
    std::mt19937 rng(1337); 
    std::uniform_real_distribution<float> rx(0.f, static_cast<float>(width));
    std::uniform_real_distribution<float> ry(0.f, static_cast<float>(height));
    std::uniform_real_distribution<float> r01(0.f, 1.f);
    
    m_stars.clear();
    m_stars.reserve(500);
    
    for (int i = 0; i < 500; ++i)
    {
        Star s;
        s.pos = { rx(rng), ry(rng) };
        float type_roll = r01(rng);

        if (type_roll < 0.6f) {
            // Type A: Standard distant star
            s.radius = 0.2f + r01(rng) * 0.6f;
            int b = 150 + static_cast<int>(r01(rng) * 105.f);
            s.color = sf::Color(b, b, b - 20, 160); // Slightly warm white
            s.depth = 0.05f + r01(rng) * 0.1f;
        } 
        else if (type_roll < 0.85f) {
            // Type B: Distant Galaxy / Nebula patch
            s.radius = 0.8f + r01(rng) * 1.5f;
            // Tint: subtle blues/purples/pinks
            int r = 50 + static_cast<int>(r01(rng) * 50);
            int g = 50 + static_cast<int>(r01(rng) * 50);
            int b = 100 + static_cast<int>(r01(rng) * 80);
            s.color = sf::Color(r, g, b, 70); // Very dim/ethereal
            s.depth = 0.01f + r01(rng) * 0.03f; // Background (almost fixed)
        }
        else {
            // Type C: Near-plane Debris / Asteroids
            s.radius = 0.5f + r01(rng) * 1.2f;
            int g = 80 + static_cast<int>(r01(rng) * 40);
            s.color = sf::Color(g + 10, g, g - 10, 120); // Greyish-brown
            s.depth = 0.3f + r01(rng) * 0.4f; // Fore-ground (fast parallax)
        }
        m_stars.push_back(s);
    }
}

void AppLoop::on_resize(unsigned width, unsigned height)
{
    m_window_size = { width, height };
    m_window.setView(sf::View(sf::FloatRect(0.f, 0.f, (float)width, (float)height)));
    
    // 1. Update Camera
    m_cam.set_screen_size(m_window_size);

    // 2. Re-create off-screen buffer
    if (!m_scene_texture.create(width, height))
        std::cerr << "[Error] Could not resize scene render texture.\n";
    
    m_scene_texture.setSmooth(true);
    
    // 3. Update sprite for the new texture
    m_scene_sprite.setTexture(m_scene_texture.getTexture(), true);

    // 4. Regenerate starfield to cover the new area
    generate_starfield(width, height);
}
// ── Render ────────────────────────────────────────────────────────────────────

void AppLoop::draw_starfield(sf::RenderTarget& t)
{
    sf::CircleShape starShape;
    for (const auto& s : m_stars)
    {
        // Static position in screen space (wrapping removed as stars are fixed to screen)
        float px = s.pos.x;
        float py = s.pos.y;

        starShape.setRadius(s.radius);
        starShape.setOrigin(s.radius, s.radius);
        starShape.setPosition(px, py);
        starShape.setFillColor(s.color);
        
        t.draw(starShape);
    }
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

            // 2. HUD interaction (Clear Trails button etc)
            if (m_hud.handle_event(event, m_trails, m_orbit_predictor))
                continue;

            // 3. Main Input (Simulation interactions)
            auto ir = m_input.handle_event(event, m_window, m_orbit_predictor);
            
            // 3. App-level Global Keys (Resize/Fullscreen)
            if (event.type == sf::Event::Resized)
            {
                on_resize(event.size.width, event.size.height);
            }
            else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::F11)
            {
                m_is_fullscreen = !m_is_fullscreen;
                if (m_is_fullscreen) {
                    auto mode = sf::VideoMode::getDesktopMode();
                    m_window.create(mode, "simSUS — Fullscreen", sf::Style::Fullscreen);
                    on_resize(mode.width, mode.height);
                } else {
                    m_window.create(sf::VideoMode(1280, 800), "simSUS", sf::Style::Default);
                    on_resize(1280, 800);
                }
                m_window.setFramerateLimit(60);
                m_window.setVerticalSyncEnabled(true);
            }
            else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::H)
            {
                m_hud.toggle_help();
            }

            if (ir == InputResult::QuitRequested)
                m_window.close();
            else if (ir == InputResult::OpenAddBodyDialog)
            {
                auto place_at = m_input.take_place_at();
                m_add_dialog.open(m_cam, place_at);
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
        m_hud.update(m_sim, sel, fps, m_initial_energy, m_cam.meters_per_pixel());
        m_editor_panel.set_selection(m_input.selected_id());

        // ── Orbit Prediction ──────────────────────────────────────────────────
        // Throttle to every 4 frames to save CPU, or update once if paused
        static int orbit_tick = 0;
        if (sel && sel->alive)
        {
            if (!m_sim.is_paused() ? (++orbit_tick >= 4) : (orbit_tick != -1))
            {
                m_orbit_predictor.update(m_sim, sel->id);
                if (!m_sim.is_paused()) orbit_tick = 0;
                else orbit_tick = -1; // Flag that we updated while paused
            }
        }
        else
        {
            m_orbit_predictor.clear();
            orbit_tick = 0;
        }
        if (!m_sim.is_paused() && orbit_tick == -1) orbit_tick = 0;

        // ── Render ────────────────────────────────────────────────────────────
        render_frame();
    }
}
