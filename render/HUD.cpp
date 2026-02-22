// =============================================================================
//  render/HUD.cpp
// =============================================================================

#include "HUD.h"
#include "../physics/Gravity.h"
#include <sstream>
#include <iomanip>
#include <cmath>

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
                 double           initial_energy_J)
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
            "1 / 2 / 3     Solar/Binary/Fig-8\n"
            "F             Follow selected\n"
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

    // Count lines for height
    int lines = 1;
    for (char ch : text) if (ch == '\n') ++lines;
    float h = static_cast<float>(lines) * LINE_H + 10.0f;

    // Background
    sf::RectangleShape panel({ w, h });
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

void HUD::draw(sf::RenderTarget& target) const
{
    float y = PANEL_Y;

    // Main diagnostics
    draw_panel(target, PANEL_X, y, PANEL_W, m_diag_text);

    // Count lines in diag for offset
    int lines = 1;
    for (char ch : m_diag_text) if (ch == '\n') ++lines;
    y += static_cast<float>(lines) * LINE_H + 18.0f;

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
