// =============================================================================
//  app/AddBodyDialog.cpp
// =============================================================================

#include "AddBodyDialog.h"
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <cstdlib>
#include <chrono>

AddBodyDialog::AddBodyDialog(const sf::Font& font) : m_font(font) {}

// ── Field labels ──────────────────────────────────────────────────────────────
static const char* FIELD_LABELS[] = {
    "Name",
    "Kind (Star/Planet/Moon/Asteroid/Spacecraft)",
    "Mass (kg)",
    "Radius (m)",
    "Pos X (m)",
    "Pos Y (m)",
    "Vel X (m/s)",
    "Vel Y (m/s)"
};

// ── Open / close ──────────────────────────────────────────────────────────────

void AddBodyDialog::reset_fields()
{
    m_fields[0] = "NewBody";
    m_fields[1] = "Planet";
    m_fields[2] = "5.972e24";
    m_fields[3] = "6.371e6";
    m_fields[4] = "1.496e11";
    m_fields[5] = "0";
    m_fields[6] = "0";
    m_fields[7] = "29780";
    m_active_field = 0;
    m_error_msg.clear();
}

void AddBodyDialog::open(const Camera& /*cam*/, std::optional<Vec2> place_at)
{
    reset_fields();
    if (place_at)
    {
        std::ostringstream sx, sy;
        sx << std::scientific << std::setprecision(4) << place_at->x;
        sy << std::scientific << std::setprecision(4) << place_at->y;
        m_fields[4] = sx.str();
        m_fields[5] = sy.str();
    }
    m_open = true;
}

void AddBodyDialog::close()
{
    m_open = false;
    m_error_msg.clear();
}

// ── Build body from field strings ─────────────────────────────────────────────

std::optional<Body> AddBodyDialog::try_build_body() const
{
    Body b;
    b.id   = "user_" + std::to_string(
                 std::chrono::steady_clock::now().time_since_epoch().count());
    b.name = m_fields[0].empty() ? "Body" : m_fields[0];
    b.kind = body_kind_from_str(m_fields[1]);

    auto parse_d = [&](int idx) -> double {
        return std::stod(m_fields[idx]);
    };

    b.mass_kg  = parse_d(2);
    b.radius_m = parse_d(3);
    b.pos      = { parse_d(4), parse_d(5) };
    b.vel      = { parse_d(6), parse_d(7) };
    b.render.color          = 0xAABBCCFF;
    b.render.base_radius_px = 5.0f;
    b.render.draw_trail     = true;
    b.render.draw_label     = true;
    b.alive = true;
    return b;
}

// ── Event handling ─────────────────────────────────────────────────────────────

std::optional<Body> AddBodyDialog::handle_event(const sf::Event& event)
{
    if (!m_open) return std::nullopt;

    if (event.type == sf::Event::KeyPressed)
    {
        if (event.key.code == sf::Keyboard::Escape)
        {
            close();
            return std::nullopt;
        }
        if (event.key.code == sf::Keyboard::Tab)
        {
            m_active_field = (m_active_field + 1) %
                             static_cast<int>(Field::COUNT);
        }
        if (event.key.code == sf::Keyboard::Enter ||
            event.key.code == sf::Keyboard::Return)
        {
            try
            {
                auto b = try_build_body();
                close();
                return b;
            }
            catch (const std::exception& e)
            {
                m_error_msg = std::string("Error: ") + e.what();
            }
        }
        if (event.key.code == sf::Keyboard::BackSpace)
        {
            auto& f = m_fields[m_active_field];
            if (!f.empty()) f.pop_back();
        }
    }

    if (event.type == sf::Event::TextEntered)
    {
        uint32_t ch = event.text.unicode;
        // Accept printable ASCII except backspace/tab/enter (handled above)
        if (ch >= 32 && ch < 127 && ch != 127)
            m_fields[m_active_field] += static_cast<char>(ch);
    }

    return std::nullopt;
}

// ── Drawing ───────────────────────────────────────────────────────────────────

void AddBodyDialog::draw_field(sf::RenderTarget& t,
                                int idx, float x, float y,
                                const char* label) const
{
    bool active = (idx == m_active_field);

    // Background
    sf::RectangleShape bg({ DLG_W - 24.0f, 22.0f });
    bg.setPosition(x, y + 14.0f);
    bg.setFillColor(active ? sf::Color(50, 70, 100, 220)
                           : sf::Color(20, 20, 40, 200));
    bg.setOutlineColor(active ? sf::Color(120, 180, 255, 240)
                              : sf::Color(60, 60, 80, 180));
    bg.setOutlineThickness(1.0f);
    t.draw(bg);

    // Label
    sf::Text lbl;
    lbl.setFont(m_font);
    lbl.setString(label);
    lbl.setCharacterSize(10);
    lbl.setFillColor(sf::Color(160, 160, 200, 200));
    lbl.setPosition(x, y);
    t.draw(lbl);

    // Value + cursor
    sf::Text val;
    val.setFont(m_font);
    val.setString(m_fields[idx] + (active ? "|" : ""));
    val.setCharacterSize(12);
    val.setFillColor(sf::Color(230, 230, 255));
    val.setPosition(x + 4.0f, y + 16.0f);
    t.draw(val);
}

void AddBodyDialog::draw(sf::RenderTarget& target) const
{
    if (!m_open) return;

    auto sz = target.getSize();
    float cx = (sz.x - DLG_W) * 0.5f;
    float cy = (sz.y - DLG_H) * 0.5f;

    // Dialog background
    sf::RectangleShape bg({ DLG_W, DLG_H });
    bg.setPosition(cx, cy);
    bg.setFillColor(sf::Color(10, 12, 26, 235));
    bg.setOutlineColor(sf::Color(80, 120, 200, 220));
    bg.setOutlineThickness(2.0f);
    target.draw(bg);

    // Title
    sf::Text title;
    title.setFont(m_font);
    title.setString("Add Body   (Tab = next field | Enter = confirm | Esc = cancel)");
    title.setCharacterSize(11);
    title.setFillColor(sf::Color(180, 210, 255));
    title.setPosition(cx + 10.0f, cy + 6.0f);
    target.draw(title);

    float field_x = cx + 12.0f;
    float field_y = cy + 28.0f;
    float field_step = 34.0f;

    for (int i = 0; i < static_cast<int>(Field::COUNT); ++i)
        draw_field(target, i, field_x, field_y + i * field_step, FIELD_LABELS[i]);

    // Error message
    if (!m_error_msg.empty())
    {
        sf::Text err;
        err.setFont(m_font);
        err.setString(m_error_msg);
        err.setCharacterSize(11);
        err.setFillColor(sf::Color(255, 100, 100));
        err.setPosition(cx + 10.0f, cy + DLG_H - 20.0f);
        target.draw(err);
    }
}
