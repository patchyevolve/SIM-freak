#pragma once
// =============================================================================
//  render/HUD.h  — On-screen overlay: diagnostics, selected body, controls
// =============================================================================

#include "../sim/Simulation.h"
#include "../domain/Body.h"
#include <SFML/Graphics.hpp>
#include <string>
#include <optional>

class TrailSystem;
class OrbitPredictor;

class HUD
{
public:
    explicit HUD(const sf::Font& font);

    /// Call every frame with current sim state + fps
    void update(const Simulation& sim,
                const Body*      selected_body,
                float            fps,
                double           initial_energy_J,
                double           meters_per_pixel);

    /// Draw overlay to render target
    void draw(sf::RenderTarget& target) const;

    bool handle_event(const sf::Event& event, TrailSystem& trails, OrbitPredictor& orbits);

    /// Toggle controls help panel
    void toggle_help() { m_show_help = !m_show_help; }
    bool show_help()  const { return m_show_help; }

    // ── Layout constants ──────────────────────────────────────────────────────
    static constexpr float PANEL_X        = 12.0f;
    static constexpr float PANEL_Y        = 12.0f;
    static constexpr float PANEL_W        = 260.0f;
    static constexpr float LINE_H         = 18.0f;
    static constexpr unsigned FONT_SIZE   = 13u;

private:
    const sf::Font& m_font;
    bool            m_show_help = false;

    // Cached text strings, rebuilt in update()
    std::string m_diag_text;
    std::string m_selected_text;
    std::string m_help_text;

    mutable sf::FloatRect m_clear_btn_bounds;

    void draw_panel(sf::RenderTarget& t,
                    float x, float y, float w,
                    const std::string& text,
                    sf::Color bg = sf::Color(10, 10, 20, 190)) const;

    static std::string format_time(double sim_s);
    static std::string format_num(double v, int decimals = 3);
};
