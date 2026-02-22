// =============================================================================
//  render/TrailSystem.cpp
// =============================================================================

#include "TrailSystem.h"
#include <cstdint>
#include <cmath>
#include <algorithm>

// ── Record ─────────────────────────────────────────────────────────────────────

void TrailSystem::record(const std::vector<Body>& bodies, double sim_time_s)
{
    // Build a set of IDs that are alive this tick — used for dead-trail cleanup
    std::unordered_set<std::string> alive_ids;
    alive_ids.reserve(bodies.size());

    for (const auto& b : bodies)
    {
        if (!b.alive || !b.render.draw_trail) continue;
        alive_ids.insert(b.id);

        auto& trail = m_trails[b.id];
        trail.alive = true;

        // Update colour from body render props
        uint32_t c = b.render.color;
        trail.color = sf::Color(
            (c >> 24) & 0xFF,
            (c >> 16) & 0xFF,
            (c >>  8) & 0xFF,
            255);

        // ── Min-move guard (world-space, zoom-independent) ────────────────────
        if (!trail.points.empty())
        {
            const Vec2& prev = trail.points.back().pos;
            double dx = b.pos.x - prev.x;
            double dy = b.pos.y - prev.y;
            double dist2 = dx * dx + dy * dy;
            if (dist2 < MIN_MOVE_M * MIN_MOVE_M) continue;
        }

        // ── Push new point with sim-time stamp ────────────────────────────────
        trail.points.push_back({ b.pos, sim_time_s });

        // ── Prune old points beyond max_age_s (lifetime expiry) ───────────────
        double cutoff = sim_time_s - max_age_s;
        while (!trail.points.empty() && trail.points.front().sim_time_s < cutoff)
            trail.points.pop_front();

        // ── Hard cap (safety) ─────────────────────────────────────────────────
        while (trail.points.size() > MAX_TRAIL)
            trail.points.pop_front();
    }

    // ── Lazy dead-body cleanup ────────────────────────────────────────────────
    // Mark trails whose body no longer appears in the alive set,
    // then erase them after a short grace period (keep for merge visual polish)
    for (auto it = m_trails.begin(); it != m_trails.end(); )
    {
        if (alive_ids.find(it->first) == alive_ids.end())
        {
            it->second.alive = false;
            // Once body is dead, drain the trail over time by popping the front
            // (gives a "vanishing tail" effect rather than instant disappearing)
            if (!it->second.points.empty())
                it->second.points.pop_front();

            if (it->second.points.empty())
                it = m_trails.erase(it);
            else
                ++it;
        }
        else
        {
            ++it;
        }
    }
}

// ── Remove / Clear ─────────────────────────────────────────────────────────────

void TrailSystem::remove(const std::string& id)
{
    m_trails.erase(id);
}

void TrailSystem::clear()
{
    m_trails.clear();
}

// ── Draw ───────────────────────────────────────────────────────────────────────

void TrailSystem::draw(sf::RenderTarget& target, const Camera& cam) const
{
    // Break the geometry line if two consecutive points are this far apart
    // in screen space (avoids long crossing segments after preset reload).
    constexpr float MAX_SEG_PX = 120.0f;

    for (const auto& [id, trail] : m_trails)
    {
        const size_t n = trail.points.size();
        if (n < 2) continue;

        // Time range for alpha normalisation
        double t_oldest = trail.points.front().sim_time_s;
        double t_newest = trail.points.back().sim_time_s;
        double t_range  = t_newest - t_oldest;
        if (t_range <= 0.0) t_range = 1.0;

        // Pre-convert all points to screen space
        std::vector<sf::Vector2f> screen(n);
        for (size_t i = 0; i < n; ++i)
            screen[i] = cam.world_to_screen(trail.points[i].pos);

        // Build vertex strips, breaking on large screen-space gaps
        // Use sf::Lines pairs so each segment has an independent colour on both
        // endpoints — avoids GPU-driver issues with LineStrip vertex interpolation.
        sf::RenderStates rs;
        rs.blendMode = sf::BlendAlpha;

        sf::VertexArray va(sf::Lines);  // pairs: (prev, cur)

        for (size_t i = 1; i < n; ++i)
        {
            float dx   = screen[i].x - screen[i - 1].x;
            float dy   = screen[i].y - screen[i - 1].y;
            float dist_seg = std::sqrt(dx * dx + dy * dy);
            if (dist_seg > MAX_SEG_PX) continue; // skip jump segments

            // ── Compute alpha for both endpoints of this segment ──────────────
            auto alpha_for = [&](size_t idx) -> uint8_t
            {
                double age_frac = (trail.points[idx].sim_time_s - t_oldest) / t_range;
                // Cubic curve (age_frac^3) ensures a very aggressive fade.
                // Halfway back (0.5), brightness is only (0.5^3) = 12.5%.
                // This makes the trail look like it's actually fading behind the body
                // rather than forming a solid ring at high warp.
                float a = static_cast<float>(age_frac * age_frac * age_frac) * ALPHA_TIP;
                if (!trail.alive) a *= 0.5f;
                return static_cast<uint8_t>(std::min(a, 255.0f));
            };

            uint8_t a0 = alpha_for(i - 1);
            uint8_t a1 = alpha_for(i);

            va.append({ screen[i - 1],
                         sf::Color(trail.color.r, trail.color.g, trail.color.b, a0) });
            va.append({ screen[i],
                         sf::Color(trail.color.r, trail.color.g, trail.color.b, a1) });
        }

        if (va.getVertexCount() >= 2)
            target.draw(va, rs);
    }
}

// ── Colour override ────────────────────────────────────────────────────────────

void TrailSystem::set_color(const std::string& id, sf::Color color)
{
    m_trails[id].color = color;
}

// ── Diagnostics ────────────────────────────────────────────────────────────────

size_t TrailSystem::total_points() const
{
    size_t total = 0;
    for (const auto& [id, trail] : m_trails)
        total += trail.points.size();
    return total;
}
