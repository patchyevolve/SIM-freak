#pragma once
// =============================================================================
//  app/BodyEditorPanel.h  — Side panel to edit selected body: kind, radius, mass, density
// =============================================================================

#include "../domain/Body.h"
#include "../sim/Simulation.h"
#include <SFML/Graphics.hpp>
#include <string>

class BodyEditorPanel
{
public:
    explicit BodyEditorPanel(const sf::Font& font);

    void set_selection(const std::string& body_id) { m_selected_id = body_id; }
    const std::string& selected_id() const { return m_selected_id; }

    /// Returns true if the event was consumed (e.g. dragging a slider or clicking a kind button)
    bool handle_event(const sf::Event& event, sf::RenderWindow& window, Simulation& sim);

    void draw(sf::RenderTarget& target, Simulation& sim);

    bool  contains(float x, float y) const;
    float get_width() const { return PANEL_WIDTH; }

    static constexpr float PANEL_WIDTH = 280.0f;

private:
    const sf::Font& m_font;
    std::string m_selected_id;

    int  m_dragging_slider = -1;  // 0 = radius, 1 = mass, -1 = none
    sf::Vector2f m_slider_bar_tl;
    float m_slider_bar_w = 200.0f;
    float m_slider_bar_h = 12.0f;
    float m_slider_y0    = 0.0f;  // Y position of radius slider (set during draw)
    float m_slider_y1    = 0.0f;  // Y position of mass slider   (set during draw)
    float m_panel_x      = 0.0f;  // X of panel left edge        (set during draw)

    static double log_lerp(double log_lo, double log_hi, double t);
    static double to_slider(double value, double log_lo, double log_hi);
    static double from_slider(double t, double log_lo, double log_hi);

    void draw_panel(sf::RenderTarget& t, float x, float y, float w, const std::string& title);
    void draw_slider(sf::RenderTarget& t, float x, float y, float val_01,
                     const char* label, const std::string& value_str);
    bool hit_slider(float mx, float my, float sx, float sy, int which);
    void apply_kind_preset(Body& b, BodyKind k);
};
