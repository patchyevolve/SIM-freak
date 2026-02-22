#pragma once
// =============================================================================
//  render/BodyRenderer.h  — Draw a single body: circle, halo, vel arrow, label
// =============================================================================

#include "../domain/Body.h"
#include "Camera.h"
#include <SFML/Graphics.hpp>
#include <string>

class BodyRenderer
{
public:
    explicit BodyRenderer(const sf::Font& font);
    
    // Shader Injection
    void set_star_shader(sf::Shader* s) { m_star_shader = s; }
    void set_atmos_shader(sf::Shader* s) { m_atmos_shader = s; }
    void set_disk_shader(sf::Shader* s) { m_disk_shader = s; }
    void set_time(float t) { m_time = t; }

    /// Draw all bodies efficiently using batching and culling
    void draw_all(sf::RenderTarget& target,
                  const std::vector<Body>& bodies,
                  const Camera& cam,
                  const std::string& selected_id) const;

    /// Draw one body to the render target (kept for backward compatibility/selection)
    void draw(sf::RenderTarget& target,
              const Body&  body,
              const Camera& cam,
              const Body*  primary = nullptr,
              bool         selected = false) const;

    // ── Visual tuning ─────────────────────────────────────────────────────────
    /// Size is always from body radius and zoom (radius_m → screen px). Min/max only avoid invisible or oversized dots.
    static constexpr float MIN_RADIUS_PX       = 3.0f;  // far zoom out = visible dots (was 1px, barely visible)
    static constexpr float MAX_RADIUS_PX       = 60.0f; // cap when zoomed in
    static constexpr float HALO_SCALE          = 1.35f;  // selection ring is 35% larger
    static constexpr float VEL_ARROW_SCALE     = 3e-4f;  // metres/s → screen pixels
    static constexpr float LOD_BATCH_THRESHOLD = 4.0f;   // px: below this, bodies are batched quads

private:
    const sf::Font& m_font;
    sf::Shader*     m_star_shader = nullptr;
    sf::Shader*     m_atmos_shader = nullptr;
    sf::Shader*     m_disk_shader = nullptr;
    float           m_time = 0.0f;

    sf::Color       body_color(const Body& b, const Camera& cam) const;
    float           clamped_radius(const Body& b, const Camera& cam) const;

    void draw_body_circle(sf::RenderTarget& t, sf::Vector2f pos,
                          float radius, sf::Color color) const;
    void draw_selection_halo(sf::RenderTarget& t, sf::Vector2f pos,
                             float radius) const;
    void draw_velocity_arrow(sf::RenderTarget& t, sf::Vector2f pos,
                             const Body& b, const Camera& cam) const;
    void draw_label(sf::RenderTarget& t, sf::Vector2f pos,
                    float radius, const Body& b) const;

    // Phase 9
    void draw_atmosphere(sf::RenderTarget& t, sf::Vector2f pos,
                         const Body& b, float screen_radius) const;
    void draw_rings(sf::RenderTarget& t, sf::Vector2f pos,
                    const Body& b, float screen_radius) const;
    void draw_soi(sf::RenderTarget& t, sf::Vector2f pos,
                  const Body& b, const Body& primary, const Camera& cam) const;
    void draw_black_hole(sf::RenderTarget& t, sf::Vector2f pos,
                         const Body& b, float screen_radius) const;
    void draw_magnetosphere(sf::RenderTarget& t, sf::Vector2f pos,
                            const Body& b, float screen_radius, sf::Vector2f sun_pos) const;

    // Phase 25: Per-kind visual routines
    void draw_star(sf::RenderTarget& t, sf::Vector2f pos,
                   const Body& b, float radius, sf::Color color) const;
    void draw_planet(sf::RenderTarget& t, sf::Vector2f pos,
                     const Body& b, float radius, sf::Color color) const;
    void draw_moon(sf::RenderTarget& t, sf::Vector2f pos,
                   const Body& b, float radius, sf::Color color) const;
    void draw_asteroid(sf::RenderTarget& t, sf::Vector2f pos,
                       const Body& b, float radius, sf::Color color) const;
};
