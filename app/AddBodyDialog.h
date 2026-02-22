#pragma once
// =============================================================================
//  app/AddBodyDialog.h  — In-window form for creating a new body at runtime
// =============================================================================

#include "../domain/Body.h"
#include "../math/Vec2.h"
#include "../render/Camera.h"
#include <SFML/Graphics.hpp>
#include <string>
#include <optional>

/// A lightweight immediate-mode input form drawn inside the SFML window.
/// The user types values for name, mass, radius, position, velocity, and kind.
class AddBodyDialog
{
public:
    explicit AddBodyDialog(const sf::Font& font);

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void open(const Camera& cam, std::optional<Vec2> place_at = std::nullopt);
    void close();
    bool is_open() const { return m_open; }

    // ── Event handling ────────────────────────────────────────────────────────
    /// Returns a completed Body if user pressed Enter, else nullopt.
    std::optional<Body> handle_event(const sf::Event& event);

    // ── Rendering ─────────────────────────────────────────────────────────────
    void draw(sf::RenderTarget& target) const;

private:
    const sf::Font& m_font;
    bool m_open = false;

    // Fields
    enum class Field : int
    { Name=0, Kind, Mass, Radius, PosX, PosY, VelX, VelY, COUNT };

    std::string m_fields[static_cast<int>(Field::COUNT)];
    int         m_active_field = 0;
    std::string m_error_msg;

    // Dialog layout
    static constexpr float DLG_W  = 340.0f;
    static constexpr float DLG_H  = 310.0f;

    // ── Helpers ───────────────────────────────────────────────────────────────
    std::optional<Body> try_build_body() const;
    void draw_field(sf::RenderTarget& t, int idx,
                    float x, float y, const char* label) const;
    void reset_fields();
};
