// =============================================================================
//  app/BodyEditorPanel.cpp
// =============================================================================

#include "BodyEditorPanel.h"
#include "../sim/StellarEvolution.h"
#include <cmath>
#include <sstream>
#include <iomanip>
#include <algorithm>

BodyEditorPanel::BodyEditorPanel(const sf::Font& font) : m_font(font) {}

double BodyEditorPanel::log_lerp(double log_lo, double log_hi, double t)
{
    return std::exp(log_lo + t * (log_hi - log_lo));
}

double BodyEditorPanel::to_slider(double value, double log_lo, double log_hi)
{
    if (value <= 0.0) return 0.0;
    double log_v = std::log(value);
    double t = (log_v - log_lo) / (log_hi - log_lo);
    return std::clamp(t, 0.0, 1.0);
}

double BodyEditorPanel::from_slider(double t, double log_lo, double log_hi)
{
    return log_lerp(log_lo, log_hi, std::clamp(t, 0.0, 1.0));
}

void BodyEditorPanel::apply_kind_preset(Body& b, BodyKind k)
{
    b.kind = k;
    switch (k) {
    case BodyKind::Star:
        b.render.color = 0xFFDC32FF;
        b.render.atmosphere_color = 0;
        b.render.has_rings = false;
        break;
    case BodyKind::Planet:
        b.render.color = 0x3278C8FF;
        b.render.atmosphere_color = 0x64C8FFB4;
        b.render.has_rings = false;
        break;
    case BodyKind::Moon:
        b.render.color = 0xAAAAAAFF;
        b.render.atmosphere_color = 0;
        b.render.has_rings = false;
        break;
    case BodyKind::Asteroid:
        b.render.color = 0x888888FF;
        b.render.atmosphere_color = 0;
        b.render.has_rings = false;
        break;
    case BodyKind::BlackHole:
        b.render.color = 0x000000FF;
        b.render.disk_color = 0xFFAA44FF;
        b.render.atmosphere_color = 0;
        b.render.has_rings = false;
        break;
    case BodyKind::Spacecraft:
        b.render.color = 0xC0C0C0FF;
        b.render.atmosphere_color = 0;
        b.render.has_rings = false;
        break;
    default:
        b.render.color = 0xAABBCCFF;
        break;
    }
}

void BodyEditorPanel::draw_panel(sf::RenderTarget& t, float x, float y, float w, const std::string& title)
{
    sf::RectangleShape bg(sf::Vector2f(w, 600.0f));
    bg.setPosition(x, y);
    bg.setFillColor(sf::Color(12, 14, 28, 240));
    bg.setOutlineColor(sf::Color(70, 90, 140, 220));
    bg.setOutlineThickness(2.0f);
    t.draw(bg);

    sf::Text tit;
    tit.setFont(m_font);
    tit.setString(title);
    tit.setCharacterSize(14);
    tit.setFillColor(sf::Color(200, 220, 255));
    tit.setPosition(x + 10.0f, y + 8.0f);
    t.draw(tit);
}

void BodyEditorPanel::draw_slider(sf::RenderTarget& t, float x, float y, float val_01,
                                  const char* label, const std::string& value_str)
{
    sf::Text lbl;
    lbl.setFont(m_font);
    lbl.setString(label);
    lbl.setCharacterSize(11);
    lbl.setFillColor(sf::Color(180, 190, 220));
    lbl.setPosition(x, y - 2.0f);
    t.draw(lbl);

    float bar_x = x;
    float bar_y = y + 14.0f;
    sf::RectangleShape bar(sf::Vector2f(m_slider_bar_w, m_slider_bar_h));
    bar.setPosition(bar_x, bar_y);
    bar.setFillColor(sf::Color(30, 35, 55, 255));
    bar.setOutlineColor(sf::Color(80, 100, 150, 200));
    bar.setOutlineThickness(1.0f);
    t.draw(bar);

    float thumb_w = 14.0f;
    float thumb_x = bar_x + val_01 * (m_slider_bar_w - thumb_w);
    sf::RectangleShape thumb(sf::Vector2f(thumb_w, m_slider_bar_h + 2.0f));
    thumb.setPosition(thumb_x, bar_y - 1.0f);
    thumb.setFillColor(sf::Color(100, 140, 220, 255));
    thumb.setOutlineColor(sf::Color(150, 180, 255, 200));
    thumb.setOutlineThickness(1.0f);
    t.draw(thumb);

    sf::Text val;
    val.setFont(m_font);
    val.setString(value_str);
    val.setCharacterSize(10);
    val.setFillColor(sf::Color(220, 230, 255));
    val.setPosition(bar_x + m_slider_bar_w + 8.0f, bar_y - 2.0f);
    t.draw(val);
}

bool BodyEditorPanel::contains(float mx, float my) const
{
    return mx >= m_slider_bar_tl.x - 20.0f; // Simplified boundary check (panel is on the right)
}

bool BodyEditorPanel::hit_slider(float mx, float my, float sx, float sy, int which)
{
    (void)which;
    // Wider hit box for sliders to make them easier to grab
    return mx >= sx - 10.0f && mx <= sx + m_slider_bar_w + 10.0f && 
           my >= sy - 15.0f && my <= sy + m_slider_bar_h + 15.0f;
}

bool BodyEditorPanel::handle_event(const sf::Event& event, sf::RenderWindow& window, Simulation& sim)
{
    if (m_selected_id.empty()) return false;

    auto sz = window.getSize();
    m_panel_x = static_cast<float>(sz.x) - PANEL_WIDTH;
    m_slider_bar_tl.x = m_panel_x + 10.0f;

    float base_y = 12.0f + 36.0f + 18.0f;

    if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left)
    {
        sf::Vector2f pos(static_cast<float>(event.mouseButton.x), static_cast<float>(event.mouseButton.y));

        // Block click-through: if click is within the panel, it's consumed
        if (pos.x >= m_panel_x)
        {
            // Preset Buttons
            for (int i = 0; i < 7; ++i)
            {
                float bx = m_panel_x + 10.0f + (i % 4) * 36.0f;
                float by = base_y + (i / 4) * 24.0f;
                if (pos.x >= bx && pos.x <= bx + 34.0f && pos.y >= by && pos.y <= by + 20.0f)
                {
                    Body* body = nullptr;
                    for (auto& b : sim.bodies_mut())
                        if (b.id == m_selected_id && b.alive) { body = &b; break; }
                    if (body) {
                        apply_kind_preset(*body, static_cast<BodyKind>(i));
                    }
                    return true;
                }
            }

            // Sliders — use y-positions cached by draw()
            for (int i = 0; i < 6; ++i) {
                if (m_slider_y[i] > 0.0f && hit_slider(pos.x, pos.y, m_slider_bar_tl.x, m_slider_y[i], i)) {
                    m_dragging_slider = i;
                    return true;
                }
            }

            return true; // Clicked on panel background, consume it
        }
    }

    if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left)
    {
        if (m_dragging_slider >= 0) { m_dragging_slider = -1; return true; }
    }

    if (event.type == sf::Event::MouseMoved && m_dragging_slider >= 0)
    {
        Body* body = nullptr;
        for (auto& b : sim.bodies_mut())
            if (b.id == m_selected_id && b.alive) { body = &b; break; }
        if (body)
        {
            float mx = static_cast<float>(event.mouseMove.x);
            float t = (mx - m_slider_bar_tl.x) / m_slider_bar_w;
            t = std::clamp(t, 0.0f, 1.0f);
            const double log_r_min = std::log(1.0e3), log_r_max = std::log(1.0e12);
            const double log_m_min = std::log(1.0e10), log_m_max = std::log(1.0e35);
            const double log_t_min = std::log(100.0), log_t_max = std::log(50000.0);
            const double log_b_min = std::log(1e-6), log_b_max = std::log(10.0);

            if (m_dragging_slider == 0)
                body->radius_m = from_slider(static_cast<double>(t), log_r_min, log_r_max);
            else if (m_dragging_slider == 1)
                body->mass_kg = from_slider(static_cast<double>(t), log_m_min, log_m_max);
            else if (m_dragging_slider == 2) {
                body->temperature_K = from_slider(static_cast<double>(t), log_t_min, log_t_max);
                if (body->kind == BodyKind::Star) {
                    body->render.color = StellarEvolution::temperature_to_color(body->temperature_K);
                }
            }
            else if (m_dragging_slider == 3) {
                body->magnetic_field_T = from_slider(static_cast<double>(t), log_b_min, log_b_max);
            }
            else if (m_dragging_slider == 4) { // Comp 1
                if (body->kind == BodyKind::Star) {
                    body->composition.hydrogen = static_cast<float>(t);
                    body->composition.helium = 1.0f - body->composition.hydrogen;
                } else {
                    body->composition.rock = static_cast<float>(t);
                    body->composition.ice = 1.0f - body->composition.rock;
                }
            }
        }
        return true;
    }

    // Capture hover events over the panel to prevent interaction elsewhere
    if (event.type == sf::Event::MouseMoved)
    {
        if (static_cast<float>(event.mouseMove.x) >= m_panel_x)
            return true;
    }

    return false;
}

void BodyEditorPanel::draw(sf::RenderTarget& target, Simulation& sim)
{
    auto sz = target.getSize();
    float x = static_cast<float>(sz.x) - PANEL_WIDTH;
    float y = 12.0f;

    if (m_selected_id.empty())
    {
        draw_panel(target, x, y, PANEL_WIDTH - 4.0f, "Edit body");
        sf::Text hint;
        hint.setFont(m_font);
        hint.setString("Select a body (click)\nto edit here.");
        hint.setCharacterSize(12);
        hint.setFillColor(sf::Color(120, 140, 180));
        hint.setPosition(x + 16.0f, y + 40.0f);
        target.draw(hint);
        return;
    }

    Body* body = nullptr;
    for (auto& b : sim.bodies_mut())
        if (b.id == m_selected_id && b.alive) { body = &b; break; }

    if (!body)
    {
        draw_panel(target, x, y, PANEL_WIDTH - 4.0f, "Edit body");
        sf::Text hint;
        hint.setFont(m_font);
        hint.setString("(Body not found)");
        hint.setCharacterSize(12);
        hint.setFillColor(sf::Color(180, 100, 100));
        hint.setPosition(x + 16.0f, y + 40.0f);
        target.draw(hint);
        return;
    }

    const double log_r_min = std::log(1.0e3);
    const double log_r_max = std::log(1.0e12);
    const double log_m_min = std::log(1.0e10);
    const double log_m_max = std::log(1.0e35);

    std::string title = "Edit: " + body->name;
    draw_panel(target, x, y, PANEL_WIDTH - 4.0f, title);

    float cy = y + 36.0f;

    sf::Text kindLabel;
    kindLabel.setFont(m_font);
    kindLabel.setString("Kind");
    kindLabel.setCharacterSize(11);
    kindLabel.setFillColor(sf::Color(180, 190, 220));
    kindLabel.setPosition(x + 10.0f, cy);
    target.draw(kindLabel);
    cy += 18.0f;

    const char* kinds[] = { "Star", "Planet", "Moon", "Asteroid", "Spacecraft", "Custom", "BlackHole" };
    float bx = x + 10.0f;
    float by = cy;
    for (int i = 0; i < 7; ++i)
    {
        BodyKind k = static_cast<BodyKind>(i);
        bool is_current = (body->kind == k);
        sf::RectangleShape btn(sf::Vector2f(34.0f, 20.0f));
        btn.setPosition(bx, by);
        btn.setFillColor(is_current ? sf::Color(80, 120, 200, 255) : sf::Color(40, 50, 80, 255));
        btn.setOutlineColor(is_current ? sf::Color(150, 200, 255) : sf::Color(70, 85, 120));
        btn.setOutlineThickness(1.0f);
        target.draw(btn);
        sf::Text kt;
        kt.setFont(m_font);
        kt.setString(kinds[i]);
        kt.setCharacterSize(8);
        kt.setFillColor(sf::Color(220, 225, 240));
        kt.setPosition(bx + 2.0f, by + 4.0f);
        target.draw(kt);
        bx += 36.0f;
        if ((i + 1) % 4 == 0) { bx = x + 10.0f; by += 24.0f; }
    }
    cy = by + 28.0f;

    float val_r = static_cast<float>(to_slider(body->radius_m, log_r_min, log_r_max));
    std::ostringstream rs;
    rs << std::scientific << std::setprecision(3) << body->radius_m;
    m_slider_y[0] = cy;
    draw_slider(target, x + 10.0f, cy, val_r, "Radius (m)", rs.str());
    cy += 48.0f;

    float val_m = static_cast<float>(to_slider(body->mass_kg, log_m_min, log_m_max));
    std::ostringstream ms;
    ms << std::scientific << std::setprecision(3) << body->mass_kg;
    m_slider_y[1] = cy;
    draw_slider(target, x + 10.0f, cy, val_m, "Mass (kg)", ms.str());
    cy += 48.0f;

    // New Sliders
    const double log_t_min = std::log(100.0), log_t_max = std::log(50000.0);
    float val_t = static_cast<float>(to_slider(body->temperature_K, log_t_min, log_t_max));
    std::ostringstream ts;
    ts << std::fixed << std::setprecision(0) << body->temperature_K << " K";
    m_slider_y[2] = cy;
    draw_slider(target, x + 10.0f, cy, val_t, "Temperature (K)", ts.str());
    cy += 48.0f;

    const double log_b_min = std::log(1e-6), log_b_max = std::log(10.0);
    float val_b = static_cast<float>(to_slider(body->magnetic_field_T, log_b_min, log_b_max));
    std::ostringstream bs;
    bs << std::scientific << std::setprecision(2) << body->magnetic_field_T << " T";
    m_slider_y[3] = cy;
    draw_slider(target, x + 10.0f, cy, val_b, "Magnetic Field (Tesla)", bs.str());
    cy += 48.0f;

    if (body->kind == BodyKind::Star || body->kind == BodyKind::Planet) {
        float val_c = (body->kind == BodyKind::Star) ? body->composition.hydrogen : body->composition.rock;
        std::ostringstream cs;
        if (body->kind == BodyKind::Star) cs << "H: " << std::fixed << std::setprecision(1) << val_c * 100.0f << "%";
        else cs << "Rock: " << std::fixed << std::setprecision(1) << val_c * 100.0f << "%";
        m_slider_y[4] = cy;
        draw_slider(target, x + 10.0f, cy, val_c, (body->kind == BodyKind::Star ? "Star: Hydrogen %" : "Planet: Rock %"), cs.str());
        cy += 48.0f;
    } else {
        m_slider_y[4] = 0;
    }
    m_slider_y[5] = 0; // Unused for now

    m_panel_x = x;     // Cache for contains() in handle_event

    double vol = (4.0 / 3.0) * 3.14159265358979 * body->radius_m * body->radius_m * body->radius_m;
    double density = (vol > 1e-30) ? (body->mass_kg / vol) : 0.0;
    std::ostringstream ds;
    ds << std::scientific << std::setprecision(3) << density << " kg/m³";
    sf::Text densLabel;
    densLabel.setFont(m_font);
    densLabel.setString("Density: " + ds.str());
    densLabel.setCharacterSize(11);
    densLabel.setFillColor(sf::Color(160, 220, 180));
    densLabel.setPosition(x + 10.0f, cy);
    target.draw(densLabel);
}
