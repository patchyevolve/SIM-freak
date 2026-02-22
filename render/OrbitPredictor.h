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

    /// Draw the predicted path
    void draw(sf::RenderTarget& target, const Camera& cam) const;

private:
    struct PathNode {
        Vec2 pos;
        double time_s;
    };

    std::vector<PathNode> m_path;
    sf::Color             m_line_color = sf::Color(255, 255, 255, 120);
};
