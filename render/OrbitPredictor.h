#pragma once
// =============================================================================
//  render/OrbitPredictor.h  — Predict and draw future orbital paths
// =============================================================================

#include "../domain/Body.h"
#include "../sim/Simulation.h"
#include "Camera.h"
#include <SFML/Graphics.hpp>
#include <vector>

class OrbitPredictor
{
public:
    OrbitPredictor();

    /// Run a shadow simulation to predict the path of 'target_id'
    void update(const Simulation& sim, const std::string& target_id);

    /// Run a shadow simulation for a hypothetical body at pos/vel
    void update_preview(const Simulation& sim, Vec2 pos, Vec2 vel);

    /// Draw the predicted path
    void draw(sf::RenderTarget& target, const Camera& cam) const;

    /// Clear the current predicted path
    void clear() { m_path.clear(); }

private:
    struct PathNode {
        Vec2 pos;
        double time_s;
    };

    std::vector<PathNode> m_path;
    sf::Color             m_line_color = sf::Color(255, 255, 255, 120);

    // Performance throttling
    sf::Clock m_update_timer;
    static constexpr float UPDATE_INTERVAL_MS = 33.0f; // Update ~30 times per sec max
};
