#pragma once
// =============================================================================
//  render/TrailSystem.h  — Per-body trail with sim-time lifetime management
//
//  Design:
//   • Each point stores a sim-time timestamp (seconds).
//   • Points older than `max_age_s` are pruned on every record() call.
//   • Alpha fades smoothly from 0 at the oldest point to ALPHA_TIP at the newest.
//   • Dead body entries are lazily pruned when the body is no longer alive in
//     a record() pass, and also immediately via remove().
//   • Min-move guard uses WORLD-SPACE distance (not screen-space) so it is
//     independent of camera zoom.
//   • Trail length also capped at MAX_TRAIL points for performance.
// =============================================================================

#include "../domain/Body.h"
#include "Camera.h"
#include <SFML/Graphics.hpp>
#include <deque>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class TrailSystem
{
public:
    // ── Configuration ─────────────────────────────────────────────────────────

    /// Hard cap on stored trail points per body (memory safety)
    static constexpr size_t MAX_TRAIL   = 400;

    /// Default trail lifetime: 90 sim-days — short enough to show a visible
    /// arc on Earth-like orbits (~25% of orbit) rather than a full-ring overlap.
    /// Adjust at runtime via max_age_s for highly eccentric / fast orbits.
    static constexpr double DEFAULT_MAX_AGE_S = 90.0 * 24.0 * 3600.0; // 90 days

    /// Minimum world-space movement (metres) before a new point is recorded.
    static constexpr double MIN_MOVE_M  = 5.0e5;  // 500 km

    /// Alpha at the newest (tip) point
    static constexpr uint8_t ALPHA_TIP  = 255;

    // ── Lifecycle config ──────────────────────────────────────────────────────

    /// How many sim-seconds of history to keep per trail (user-adjustable)
    double max_age_s = DEFAULT_MAX_AGE_S;

    // ── Public API ────────────────────────────────────────────────────────────

    /// Record positions for all alive bodies at sim-time `sim_time_s`.
    /// Also prunes points older than max_age_s and removes dead body entries.
    void record(const std::vector<Body>& bodies, double sim_time_s);

    /// Immediately remove the trail for a body (e.g. post-merge/absorption).
    void remove(const std::string& id);

    /// Clear all trails (preset reload, etc.)
    void clear();

    /// Draw all trails. Converts world-space points to screen at draw time.
    void draw(sf::RenderTarget& target, const Camera& cam) const;

    /// Override the recorded colour for a body's trail.
    void set_color(const std::string& id, sf::Color color);

    /// How many trails are currently alive
    size_t trail_count() const { return m_trails.size(); }

    /// Total points across all trails
    size_t total_points() const;

private:
    struct TrailPoint
    {
        Vec2   pos;          ///< World-space position (metres)
        double sim_time_s;   ///< Simulation time when this point was recorded
    };

    struct Trail
    {
        std::deque<TrailPoint> points;
        sf::Color              color = sf::Color::White;
        bool                   alive = true; ///< Set false when body dies
    };

    std::unordered_map<std::string, Trail> m_trails;
};
