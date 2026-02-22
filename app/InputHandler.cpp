// =============================================================================
//  app/InputHandler.cpp
// =============================================================================

#include "InputHandler.h"
#include "../io/State.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <sstream>
#include <iomanip>

InputHandler::InputHandler(Simulation& sim, Camera& cam, TrailSystem& trails)
    : m_sim(sim), m_cam(cam), m_trails(trails)
{}

// ── Selection ──────────────────────────────────────────────────────────────────

const Body* InputHandler::selected_body() const
{
    if (m_selected_id.empty()) return nullptr;
    for (const auto& b : m_sim.bodies())
        if (b.id == m_selected_id && b.alive) return &b;
    m_selected_id.clear();
    return nullptr;
}

// ── Left click: pick nearest body ─────────────────────────────────────────────

void InputHandler::on_left_click(sf::Vector2f screen_pos)
{
    Vec2   world_click = m_cam.screen_to_world(screen_pos);
    double best_dist   = 1e300;
    std::string best_id;

    for (const auto& b : m_sim.bodies())
    {
        if (!b.alive) continue;
        double dist = b.pos.dist_to(world_click);
        // Hit test using max of world radius and a minimum click area
        double hit_r = std::max(b.radius_m, m_cam.meters_per_pixel() * 8.0);
        if (dist < hit_r && dist < best_dist)
        {
            best_dist = dist;
            best_id   = b.id;
        }
    }

    if (!best_id.empty())
    {
        m_selected_id = best_id;
        // Sync camera follow if already following
        const Body* sel = selected_body();
        if (m_cam.is_following() && sel)
            m_cam.set_follow(sel);
    }
    else
    {
        m_selected_id.clear();
        m_cam.set_follow(nullptr);
    }
}

// ── Preset load ────────────────────────────────────────────────────────────────

void InputHandler::load_preset(PresetType p)
{
    m_sim.clear_bodies();
    m_trails.clear();
    m_selected_id.clear();
    m_cam.set_follow(nullptr);

    auto bodies = Presets::make(p, m_sim.config().G);
    for (auto& b : bodies) m_sim.add_body(b);

    // ── Preset-specific Tuning (Visuals, Time, Camera) ───────────────────────
    
    // Default values
    m_trails.max_age_s = TrailSystem::DEFAULT_MAX_AGE_S; // 90 days
    m_sim.set_time_warp(1.0);
    double zoom = 4e9; // 4 million km (Solar System scale)

    switch (p)
    {
    case PresetType::SolarSystem:
        zoom = 8e9;
        m_sim.set_time_warp(1.0); // 1s = 1s (realtime)
        break;
    case PresetType::BinaryStar:
        zoom = 1e11;
        m_sim.set_time_warp(3600.0); // 1h per sec
        break;
    case PresetType::Figure8:
        zoom = 10.0;
        m_sim.set_time_warp(1.0);
        break;
    case PresetType::BlackHole:
        zoom = 2e12;
        m_sim.set_time_warp(3600.0 * 24.0); // 1 day per sec
        m_trails.max_age_s = 2.5 * 24.0 * 3600.0; // Shorter trails for BH
        break;
    case PresetType::Collision:
        zoom = 1e11;
        m_sim.set_time_warp(3600.0 * 12.0);
        break;
    case PresetType::Nebula:
        zoom = 5e12;
        m_sim.set_time_warp(3600.0 * 24.0 * 7.0); // 1 week per sec
        break;
    case PresetType::GalaxySmall:
        zoom = 1e13; // 10,000 AU scale
        m_sim.set_time_warp(3600.0 * 24.0 * 365.0); // 1 year per sec
        break;
    }

    m_cam.set_zoom(zoom);
    m_sim.events.on_preset_loaded.emit({ preset_name(p), m_sim.bodies().size() });
    std::cout << "[Preset] Loaded: " << preset_name(p) << " (" << m_sim.bodies().size() << " bodies)\n";
}

// ── Save / Load ────────────────────────────────────────────────────────────────

void InputHandler::try_save()
{
    try
    {
        IO::save_state(m_sim, "quicksave.json");
        std::cout << "[IO] Saved quicksave.json\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[IO] Save failed: " << e.what() << "\n";
    }
}

void InputHandler::try_load()
{
    try
    {
        m_trails.clear();
        m_selected_id.clear();
        m_cam.set_follow(nullptr);
        IO::load_state(m_sim, "quicksave.json");
        std::cout << "[IO] Loaded quicksave.json (" << m_sim.bodies().size() << " bodies)\n";
    }
    catch (const std::exception& e)
    {
        std::cerr << "[IO] Load failed: " << e.what() << "\n";
    }
}

// ── Event pump ────────────────────────────────────────────────────────────────

InputResult InputHandler::handle_event(const sf::Event& event,
                                       sf::RenderWindow& window)
{
    // ── Window close ──────────────────────────────────────────────────────────
    if (event.type == sf::Event::Closed)
        return InputResult::QuitRequested;

    // ── Keyboard ──────────────────────────────────────────────────────────────
    if (event.type == sf::Event::KeyPressed)
    {
        switch (event.key.code)
        {
        case sf::Keyboard::Escape:
            m_selected_id.clear();
            m_cam.set_follow(nullptr);
            break;

        case sf::Keyboard::Space:
            m_sim.set_paused(!m_sim.is_paused());
            break;

        case sf::Keyboard::LBracket:
            m_sim.set_time_warp(std::max(0.125, m_sim.time_warp() * 0.25));
            break;

        case sf::Keyboard::RBracket:
            m_sim.set_time_warp(std::min(1'000'000.0, m_sim.time_warp() * 4.0));
            break;

        case sf::Keyboard::Num1: load_preset(PresetType::SolarSystem); break;
        case sf::Keyboard::Num2: load_preset(PresetType::BinaryStar);  break;
        case sf::Keyboard::Num3: load_preset(PresetType::Figure8);     break;
        case sf::Keyboard::Num4: load_preset(PresetType::BlackHole);   break;
        case sf::Keyboard::Num5: load_preset(PresetType::Collision);   break;
        case sf::Keyboard::Num6: load_preset(PresetType::Nebula);      break;
        case sf::Keyboard::Num7: load_preset(PresetType::GalaxySmall); break;

        case sf::Keyboard::F:
        {
            const Body* sel = selected_body();
            if (m_cam.is_following())
                m_cam.set_follow(nullptr);
            else if (sel)
                m_cam.set_follow(sel);
            break;
        }

        case sf::Keyboard::S: try_save(); break;
        case sf::Keyboard::L: try_load(); break;

        case sf::Keyboard::A:
            if (m_place_mode) {
                m_place_mode = false;
                return InputResult::OpenAddBodyDialog;
            }
            m_place_mode = true;
            return InputResult::None;

        default: break;
        }
    }

    // ── Mouse wheel — zoom ────────────────────────────────────────────────────
    if (event.type == sf::Event::MouseWheelScrolled)
    {
        double factor = (event.mouseWheelScroll.delta > 0) ? 0.8 : 1.25;
        m_cam.zoom_at(
            { static_cast<float>(event.mouseWheelScroll.x),
              static_cast<float>(event.mouseWheelScroll.y) },
            factor);
    }

    // ── Middle button — start pan ─────────────────────────────────────────────
    if (event.type == sf::Event::MouseButtonPressed &&
        event.mouseButton.button == sf::Mouse::Middle)
    {
        m_panning = true;
        m_last_mouse_px = { static_cast<float>(event.mouseButton.x),
                            static_cast<float>(event.mouseButton.y) };
        m_cam.set_follow(nullptr);
    }

    if (event.type == sf::Event::MouseButtonReleased &&
        event.mouseButton.button == sf::Mouse::Middle)
        m_panning = false;

    // ── Left click — place mode, select, or start launch ──────────────────────
    if (event.type == sf::Event::MouseButtonPressed &&
        event.mouseButton.button == sf::Mouse::Left)
    {
        sf::Vector2f mpos(static_cast<float>(event.mouseButton.x),
                          static_cast<float>(event.mouseButton.y));

        if (m_place_mode)
        {
            m_place_at = m_cam.screen_to_world(mpos);
            m_place_mode = false;
            return InputResult::OpenAddBodyDialog;
        }

        on_left_click(mpos);

        if (m_selected_id.empty())
        {
            m_launching = true;
            m_launch_start = mpos;
            m_launch_end = mpos;
        }
    }

    if (event.type == sf::Event::MouseButtonReleased &&
        event.mouseButton.button == sf::Mouse::Left)
    {
        if (m_launching)
        {
            m_launching = false;
            on_left_drag_end({ static_cast<float>(event.mouseButton.x),
                               static_cast<float>(event.mouseButton.y) });
        }
    }

    // ── Window resize ─────────────────────────────────────────────────────────
    if (event.type == sf::Event::Resized)
    {
        sf::FloatRect vis_area(0.f, 0.f,
                               static_cast<float>(event.size.width),
                               static_cast<float>(event.size.height));
        window.setView(sf::View(vis_area));
        m_cam.set_screen_size({ event.size.width, event.size.height });
    }

    return InputResult::None;
}

void InputHandler::on_left_drag_end(sf::Vector2f screen_pos)
{
    Vec2 start_world = m_cam.screen_to_world(m_launch_start);
    
    // Slingshot logic: Pull back to fire. 
    // Screen delta: start -> current
    // We want World velocity: pull_back_vector * scale
    // Note: World Y is inverted relative to screen Y.
    double dx = static_cast<double>(m_launch_start.x - screen_pos.x);
    double dy = static_cast<double>(screen_pos.y - m_launch_start.y); // Flipped screen delta
    
    // Calibrated multiplier: 800 m/s per screen pixel drag.
    // Significant buff to allow interplanetary/escape speeds more easily.
    Vec2 velocity(dx * 800.0, dy * 800.0);

    Body b;
    b.id = "spawned_" + std::to_string(std::rand());
    b.name = "Asteroid";
    b.kind = BodyKind::Custom;
    b.mass_kg = 7.347e22; // Moon mass ish
    b.radius_m = 1.737e6; // Moon radius
    b.pos = start_world;
    b.vel = velocity;
    b.render.color = 0x888888FF;
    b.render.base_radius_px = 8.0f;
    
    m_sim.add_body(b);
    m_launching = false;
}

void InputHandler::draw_launch_preview(sf::RenderTarget& target, const sf::Font& font) const
{
    if (!m_launching) return;

    // Line from start to current (visual "pull')
    sf::Vertex line[] = {
        sf::Vertex(m_launch_start, sf::Color::Green),
        sf::Vertex(m_launch_end,   sf::Color::Yellow)
    };
    target.draw(line, 2, sf::Lines);

    // Indicator of where the body will fire
    sf::Vector2f fire_vec = m_launch_start - m_launch_end;
    sf::Vertex arrow[] = {
        sf::Vertex(m_launch_start, sf::Color::Red),
        sf::Vertex(m_launch_start + fire_vec, sf::Color(255, 0, 0, 100))
    };
    target.draw(arrow, 2, sf::Lines);

    // Velocity Text Overlay
    double dx = static_cast<double>(m_launch_start.x - m_launch_end.x);
    double dy = static_cast<double>(m_launch_start.y - m_launch_end.y);
    double speed_ms = std::sqrt(dx*dx + dy*dy) * 800.0;
    
    std::ostringstream oss;
    oss << std::fixed;
    if (speed_ms < 1000.0)
        oss << std::setprecision(1) << speed_ms << " m/s";
    else
        oss << std::setprecision(2) << (speed_ms / 1000.0) << " km/s";

    sf::Text txt(oss.str(), font, 16);
    txt.setFillColor(sf::Color::Cyan);
    txt.setOutlineColor(sf::Color::Black);
    txt.setOutlineThickness(1.0f);
    // Position text above the mouse cursor
    txt.setPosition(m_launch_end.x + 10.f, m_launch_end.y - 30.f);
    target.draw(txt);

    // Dotted circle at start to show spawn point
    sf::CircleShape start(5.0f);
    start.setOrigin(5.f, 5.f);
    start.setPosition(m_launch_start);
    start.setFillColor(sf::Color::Transparent);
    start.setOutlineColor(sf::Color::Green);
    start.setOutlineThickness(1.0f);
    target.draw(start);
}

// ── Per-frame update (continuous pan) ─────────────────────────────────────────

std::optional<Vec2> InputHandler::take_place_at()
{
    auto out = m_place_at;
    m_place_at.reset();
    return out;
}

void InputHandler::update(sf::RenderWindow& window)
{
    if (m_panning)
    {
        sf::Vector2f cur = static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
        sf::Vector2f delta = cur - m_last_mouse_px;
        if (delta.x != 0.0f || delta.y != 0.0f)
            m_cam.pan(delta);
        m_last_mouse_px = cur;
    }

    if (m_launching)
    {
        m_launch_end = static_cast<sf::Vector2f>(sf::Mouse::getPosition(window));
    }
}
