#pragma once
// =============================================================================
//  app/AppLoop.h  — SFML window + main event/render loop
// =============================================================================

#include "../sim/Simulation.h"
#include "../sim/Presets.h"
#include "../render/Camera.h"
#include "../render/TrailSystem.h"
#include "../render/BodyRenderer.h"
#include "../render/HUD.h"
#include "../render/GridRenderer.h"
#include "InputHandler.h"
#include "AddBodyDialog.h"
#include "BodyEditorPanel.h"
#include "../render/OrbitPredictor.h"

#include <SFML/Graphics.hpp>
#include <string>

class AppLoop
{
public:
    explicit AppLoop(unsigned width = 1280, unsigned height = 800);
    ~AppLoop();

    /// Blocking main loop — returns when window closes
    void run();

private:
    // ── SFML resources ────────────────────────────────────────────────────────
    sf::RenderWindow m_window;
    sf::Font         m_font;
    sf::Clock        m_clock;
    sf::Shader       m_lensing_shader;
    sf::RenderTexture m_scene_texture;
    sf::Sprite       m_scene_sprite;
    sf::Vector2u     m_window_size;

    // ── Simulation + render ───────────────────────────────────────────────────
    Simulation    m_sim;
    Camera        m_cam;
    GridRenderer  m_grid;
    TrailSystem   m_trails;
    BodyRenderer  m_body_renderer{ m_font };   // init after m_font is loaded
    HUD           m_hud{ m_font };
    InputHandler  m_input{ m_sim, m_cam, m_trails };
    AddBodyDialog m_add_dialog{ m_font };
    BodyEditorPanel m_editor_panel{ m_font };
    OrbitPredictor m_orbit_predictor;

    // ── State ─────────────────────────────────────────────────────────────────
    double m_initial_energy = 0.0;
    int    m_trail_tick     = 0;
    static constexpr int TRAIL_RECORD_EVERY = 3; // record trail every N frames

    // ── Sub-routines ──────────────────────────────────────────────────────────
    void load_default_preset();
    void render_frame();
    void draw_starfield(sf::RenderTarget& t);

    // Starfield (generated once)
    std::vector<sf::CircleShape> m_stars;
    void generate_starfield(unsigned width, unsigned height);
};
