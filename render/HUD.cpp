// =============================================================================
//  render/HUD.cpp
// =============================================================================

#include "HUD.h"
#include "../physics/Gravity.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include "TrailSystem.h"
#include "OrbitPredictor.h"

HUD::HUD(const sf::Font& font) : m_font(font) {}

// ── Static formatters ──────────────────────────────────────────────────────────

std::string HUD::format_time(double sim_s)
{
    if (sim_s < 0.0) return "0 s";
    double days  = sim_s / 86400.0;
    double years = days  / 365.25;
    std::ostringstream oss;
    if (years >= 1.0)
        oss << std::fixed << std::setprecision(2) << years << " yr";
    else if (days >= 1.0)
        oss << std::fixed << std::setprecision(1) << days  << " d";
    else
        oss << std::fixed << std::setprecision(0) << sim_s << " s";
    return oss.str();
}

std::string HUD::format_num(double v, int decimals)
{
    std::ostringstream oss;
    oss << std::scientific << std::setprecision(decimals) << v;
    return oss.str();
}

// ── Update ─────────────────────────────────────────────────────────────────────

void HUD::update(const Simulation& sim,
                 const Body*      selected_body,
                 float            fps,
                 double           initial_energy_J,
                 double           meters_per_pixel)
{
    auto diag = sim.diagnostics();

    // ── Diagnostics panel ─────────────────────────────────────────────────────
    {
        std::ostringstream oss;
        oss << std::fixed;
        oss << "FPS        " << std::setprecision(0) << fps << "\n";
        oss << "Sim time   " << format_time(diag.sim_time_s) << "\n";
        oss << "Time warp  " << std::setprecision(1) << sim.time_warp() << "x\n";
        oss << "Bodies     " << diag.body_count << "\n";
        oss << "Step       " << diag.step_count << "\n";
        oss << "GPU Phys   " << (Gravity::IsGpuReady() ? "ON (GPU)" : "OFF (CPU)") << "\n";

        // Energy drift — guard against NaN/Inf; skip when energy not computed (n > 500)
        if (diag.energy_valid && std::abs(initial_energy_J) > 1e-100)
        {
            double drift = (diag.total_energy_J - initial_energy_J)
                           / std::abs(initial_energy_J) * 100.0;
            oss << "E drift    ";
            if (!std::isfinite(drift) || std::abs(drift) > 1e10)
                oss << std::scientific << std::setprecision(2) << drift;
            else
                oss << std::fixed << std::setprecision(3) << drift;
            oss << " %\n";
        }
        else if (!diag.energy_valid)
            oss << "E drift    (n>" << 500 << ")\n";

        // Zoom display
        oss << "Scale      ";
        if (meters_per_pixel >= 1.5e14) // >= 1000 AU
            oss << std::setprecision(2) << meters_per_pixel / 1.496e11 << " AU/px\n";
        else if (meters_per_pixel >= 1e9)
            oss << std::setprecision(1) << meters_per_pixel / 1e9 << "M km/p\n";
        else if (meters_per_pixel >= 1e3)
            oss << std::setprecision(1) << meters_per_pixel / 1e3 << " km/px\n";
        else
            oss << std::setprecision(1) << meters_per_pixel << " m/px\n";

        oss << (sim.is_paused() ? "[PAUSED]" : "") ;
        m_diag_text = oss.str();
    }

    // ── Selected body panel ───────────────────────────────────────────────────
    m_selected_text.clear();
    if (selected_body)
    {
        std::ostringstream oss;
        oss << "=== " << selected_body->name << " ===\n";
        oss << "Kind       " << body_kind_to_str(selected_body->kind) << "\n";
        oss << "Mass       " << format_num(selected_body->mass_kg) << " kg\n";
        oss << "Radius     " << format_num(selected_body->radius_m) << " m\n";
        oss << "Speed      " << format_num(selected_body->speed())   << " m/s\n";
        oss << "KE         " << format_num(selected_body->kinetic_energy()) << " J\n";
        oss << "g-surf     " << format_num(selected_body->surface_gravity(), 2) << " m/s²\n";
        m_selected_text = oss.str();
    }

    // ── Controls help ─────────────────────────────────────────────────────────
    if (m_help_text.empty())
    {
        m_help_text =
            "=== Controls ===\n"
            "Scroll        Zoom\n"
            "Mid-drag      Pan\n"
            "Left click    Select\n"
            "Space         Pause/Resume\n"
            "[ / ]         Warp x0.5 / x2\n"
            "1 / 2 / 3 / 4 Solar/Binary/Fig-8/BH\n"
            "F             Follow selected\n"
            "C             Clear all trails & orbit path\n"
            "A             Place mode (then click to add body there)\n"
            "              A again = add at default pos\n"
            "              Right panel = edit selected body\n"
            "S / L         Save / Load\n"
            "H             Toggle this help\n";
    }
}

// ── Draw helpers ───────────────────────────────────────────────────────────────

void HUD::draw_panel(sf::RenderTarget& t,
                     float x, float y, float w,
                     const std::string& text,
                     sf::Color bg) const
{
    if (text.empty()) return;

    // Measure text to prevent overflow
    sf::Text measureTxt;
    measureTxt.setFont(m_font);
    measureTxt.setCharacterSize(FONT_SIZE);
    
    float maxWidth = 0.0f;
    std::string line;
    std::stringstream ss(text);
    int lines = 0;
    while (std::getline(ss, line)) {
        measureTxt.setString(line);
        float lw = measureTxt.getLocalBounds().width;
        if (lw > maxWidth) maxWidth = lw;
        lines++;
    }

    float finalW = std::max(w, maxWidth + 20.0f);
    float finalH = static_cast<float>(lines) * LINE_H + 10.0f;

    // Background
    sf::RectangleShape panel({ finalW, finalH });
    panel.setPosition(x, y);
    panel.setFillColor(bg);
    panel.setOutlineColor(sf::Color(60, 60, 80, 200));
    panel.setOutlineThickness(1.0f);
    t.draw(panel);

    // Text
    sf::Text txt;
    txt.setFont(m_font);
    txt.setString(text);
    txt.setCharacterSize(FONT_SIZE);
    txt.setFillColor(sf::Color(220, 220, 240));
    txt.setPosition(x + 8.0f, y + 5.0f);
    t.draw(txt);
}

// ── Draw ───────────────────────────────────────────────────────────────────────

bool HUD::handle_event(const sf::Event& event, TrailSystem& trails, OrbitPredictor& orbits)
{
    if (event.type == sf::Event::MouseButtonPressed && event.key.code == sf::Mouse::Left)
    {
        sf::Vector2f mpos(static_cast<float>(event.mouseButton.x), static_cast<float>(event.mouseButton.y));
        if (m_clear_btn_bounds.contains(mpos))
        {
            trails.clear();
            orbits.clear();
            return true;
        }
    }
    return false;
}

void HUD::draw(sf::RenderTarget& target) const
{
    float y = PANEL_Y;

    // Main diagnostics
    draw_panel(target, PANEL_X, y, PANEL_W, m_diag_text);

    // Count lines in diag for offset
    int lines = 1;
    for (char ch : m_diag_text) if (ch == '\n') ++lines;
    float diag_h = static_cast<float>(lines) * LINE_H + 10.0f;

    // Button: Clear Trails (Positioned below the diagnostics panel)
    {
        float btnW = 200.0f;
        float btnH = 22.0f;
        float btnX = PANEL_X + (PANEL_W - btnW) * 0.5f;
        float btnY = y + diag_h + 8.0f;

        m_clear_btn_bounds = sf::FloatRect(btnX, btnY, btnW, btnH);

        sf::RectangleShape clearBtn(sf::Vector2f(btnW, btnH));
        clearBtn.setPosition(btnX, btnY);
        clearBtn.setFillColor(sf::Color(100, 30, 30, 180));
        clearBtn.setOutlineColor(sf::Color(180, 60, 60, 150));
        clearBtn.setOutlineThickness(1.0f);
        target.draw(clearBtn);

        sf::Text clearTxt;
        clearTxt.setFont(m_font);
        clearTxt.setString("CLEAR ALL TRAILS ('C')");
        clearTxt.setCharacterSize(10);
        clearTxt.setFillColor(sf::Color::White);
        
        // Center text in button
        sf::FloatRect textBounds = clearTxt.getLocalBounds();
        clearTxt.setPosition(btnX + (btnW - textBounds.width) * 0.5f, 
                             btnY + (btnH - textBounds.height) * 0.5f - 2.0f);
        target.draw(clearTxt);
        
        y += diag_h + btnH + 18.0f; 
    }

    // Selected body
    if (!m_selected_text.empty())
    {
        draw_panel(target, PANEL_X, y, PANEL_W, m_selected_text,
                   sf::Color(10, 20, 40, 190));
        int sel_lines = 1;
        for (char ch : m_selected_text) if (ch == '\n') ++sel_lines;
        y += static_cast<float>(sel_lines) * LINE_H + 10.0f;
    }

    // Controls help
    if (m_show_help)
        draw_panel(target, PANEL_X, y, PANEL_W, m_help_text,
                   sf::Color(20, 10, 30, 200));
}
