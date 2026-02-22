#pragma once
// =============================================================================
//  render/Camera.h  — World ↔ Screen coordinate mapping
//  Physics uses metres (world).  SFML uses pixels (screen).
// =============================================================================

#include "../math/Vec2.h"
#include "../domain/Body.h"
#include <SFML/Graphics.hpp>

class Camera
{
public:
    // ── Construction ──────────────────────────────────────────────────────────
    Camera(sf::Vector2u screen_size,
           double meters_per_pixel = 2.0e9); // default: ~1 AU fills ~75 px

    // ── Coordinate transforms ─────────────────────────────────────────────────
    sf::Vector2f world_to_screen(Vec2 world_pos)        const;
    Vec2         screen_to_world(sf::Vector2f screen_pos) const;

    /// Convert a world-space radius (metres) to screen pixels
    float world_radius_to_screen(double radius_m) const;

    // ── Zoom ──────────────────────────────────────────────────────────────────
    /// Zoom so the world point under `screen_pt` stays fixed
    void zoom_at(sf::Vector2f screen_pt, double factor);
    void set_zoom(double meters_per_pixel);
    double meters_per_pixel() const { return m_mpp; }

    // ── Pan ───────────────────────────────────────────────────────────────────
    void pan(sf::Vector2f delta_screen_px);
    void set_center(Vec2 world_pos) { m_center = world_pos; }
    Vec2 center() const { return m_center; }

    // ── Follow mode ───────────────────────────────────────────────────────────
    void set_follow(const Body* body);   // nullptr = free camera
    void update_follow();                // call every frame before rendering
    bool is_following() const { return m_follow != nullptr; }
    void set_smooth_follow(bool s) { m_smooth_follow = s; }

    // ── Window resize ─────────────────────────────────────────────────────────
    void set_screen_size(sf::Vector2u sz) { m_screen = sz; }
    sf::Vector2u screen_size() const { return m_screen; }

    /// Returns the world-to-screen transform matrix
    sf::Transform get_transform() const;

    // ── Zoom limits ───────────────────────────────────────────────────────────
    static constexpr double MIN_MPP = 1.0e5;   // very zoomed-in
    static constexpr double MAX_MPP = 1.0e12;  // very zoomed-out

private:
    Vec2         m_center;   // world coordinates at screen centre
    double       m_mpp;      // metres per pixel
    sf::Vector2u m_screen;
    const Body*  m_follow  = nullptr;
    bool         m_smooth_follow = true;
};
