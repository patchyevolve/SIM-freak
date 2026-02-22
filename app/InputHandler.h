#pragma once
// =============================================================================
//  app/InputHandler.h  — Keyboard + mouse → simulation + camera commands
// =============================================================================

#include "../sim/Simulation.h"
#include "../sim/Presets.h"
#include "../render/Camera.h"
#include "../render/TrailSystem.h"
#include "../math/Vec2.h"
#include <SFML/Graphics.hpp>
#include <string>
#include <functional>
#include <optional>

class OrbitPredictor;

/// Result returned by InputHandler::handle_event()
enum class InputResult
{
    None,
    OpenAddBodyDialog,
    QuitRequested
};

class InputHandler
{
public:
    InputHandler(Simulation& sim, Camera& cam, TrailSystem& trails);

    /// Process one SFML event. Returns InputResult if a special action was triggered.
    InputResult handle_event(const sf::Event& event,
                             sf::RenderWindow& window,
                             OrbitPredictor& orbits);

    /// Called every frame (for continuous middle-button drag)
    void update(sf::RenderWindow& window);

    /// Draw the launch vector arrow and velocity overlay if currently launching
    void draw_launch_preview(sf::RenderTarget& target, const sf::Font& font) const;

    // ── Selection ─────────────────────────────────────────────────────────────
    const Body*       selected_body() const;
    const std::string& selected_id()  const { return m_selected_id; }
    void deselect() { m_selected_id.clear(); }

    /// If OpenAddBodyDialog was returned after place-mode click, returns that world position (and clears it).
    std::optional<Vec2> take_place_at();

    /// Load a specific simulation preset (sets camera zoom, time warp, etc.)
    void load_preset(PresetType p);

private:
    Simulation&  m_sim;
    Camera&      m_cam;
    TrailSystem& m_trails;

    mutable std::string  m_selected_id;

    // Middle-button pan state
    bool         m_panning       = false;
    sf::Vector2f m_last_mouse_px = {};

    // Point-and-place: after 'A', next click sets position for new body
    bool                m_place_mode = false;
    std::optional<Vec2> m_place_at;

    // Drag-launch state
    bool         m_launching     = false;
    sf::Vector2f m_launch_start  = {};
    sf::Vector2f m_launch_end    = {};

    // ── Helpers ───────────────────────────────────────────────────────────────
    void on_left_click(sf::Vector2f screen_pos);
    void on_left_drag_start(sf::Vector2f screen_pos);
    void on_left_drag_end(sf::Vector2f screen_pos);
    void try_save();
    void try_load();
};
