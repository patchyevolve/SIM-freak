// =============================================================================
//  render/Camera.cpp
// =============================================================================

#include "Camera.h"
#include <algorithm>
#include <cmath>

Camera::Camera(sf::Vector2u screen_size, double meters_per_pixel)
    : m_center{}, m_mpp(meters_per_pixel), m_screen(screen_size)
{}

// ── Coordinate transforms ──────────────────────────────────────────────────────

sf::Vector2f Camera::world_to_screen(Vec2 wp) const
{
    double sx = static_cast<double>(m_screen.x) * 0.5 + (wp.x - m_center.x) / m_mpp;
    double sy = static_cast<double>(m_screen.y) * 0.5 - (wp.y - m_center.y) / m_mpp;
    return { static_cast<float>(sx), static_cast<float>(sy) };
}

Vec2 Camera::screen_to_world(sf::Vector2f sp) const
{
    double wx = m_center.x + (static_cast<double>(sp.x) - m_screen.x * 0.5) * m_mpp;
    double wy = m_center.y - (static_cast<double>(sp.y) - m_screen.y * 0.5) * m_mpp;
    return { wx, wy };
}

float Camera::world_radius_to_screen(double radius_m) const
{
    return static_cast<float>(radius_m / m_mpp);
}

// ── Zoom ───────────────────────────────────────────────────────────────────────

void Camera::zoom_at(sf::Vector2f screen_pt, double factor)
{
    // World position under cursor before zoom
    Vec2 world_before = screen_to_world(screen_pt);

    double new_mpp = std::clamp(m_mpp * factor, MIN_MPP, MAX_MPP);
    m_mpp = new_mpp;

    // Shift centre so world_before stays under cursor after zoom
    Vec2 world_after = screen_to_world(screen_pt);
    m_center = m_center + (world_before - world_after);
}

void Camera::set_zoom(double mpp)
{
    m_mpp = std::clamp(mpp, MIN_MPP, MAX_MPP);
}

// ── Pan ────────────────────────────────────────────────────────────────────────

void Camera::pan(sf::Vector2f delta_screen)
{
    m_center.x -= static_cast<double>(delta_screen.x) * m_mpp;
    m_center.y += static_cast<double>(delta_screen.y) * m_mpp;
}

// ── Follow ─────────────────────────────────────────────────────────────────────

void Camera::set_follow(const Body* body)
{
    m_follow = body;
    if (body) m_center = body->pos;
}

void Camera::update_follow()
{
    if (!m_follow) return;

    if (m_smooth_follow)
    {
        // Exponential smoothing (lerp)
        // Adjust 0.1 for faster/slower follow
        double factor = 0.1;
        m_center.x += (m_follow->pos.x - m_center.x) * factor;
        m_center.y += (m_follow->pos.y - m_center.y) * factor;
    }
    else
    {
        m_center = m_follow->pos;
    }
}

sf::Transform Camera::get_transform() const
{
    sf::Transform t;
    // 1. Move to screen center
    t.translate(static_cast<float>(m_screen.x) * 0.5f, static_cast<float>(m_screen.y) * 0.5f);
    // 2. Scale world to pixels (note Y is flipped in SFML)
    t.scale(static_cast<float>(1.0 / m_mpp), static_cast<float>(-1.0 / m_mpp));
    // 3. Center on world camera position
    t.translate(static_cast<float>(-m_center.x), static_cast<float>(-m_center.y));
    return t;
}
