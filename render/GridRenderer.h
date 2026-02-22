#pragma once
// =============================================================================
//  render/GridRenderer.h
// =============================================================================

#include "../sim/Simulation.h"
#include "Camera.h"
#include <SFML/Graphics.hpp>
#include <vector>

class GridRenderer
{
public:
    GridRenderer();

    /// Update and draw the grid based on current simulation state
    void draw(sf::RenderTarget& target, const Camera& cam, const Simulation& sim) const;

private:
    struct GridVertex {
        Vec2 base_pos;   // Original grid intersection in world space
        sf::Color color;
    };

    mutable std::vector<GridVertex> m_grid;
    mutable sf::Shader              m_shader;
    mutable bool                    m_shader_loaded = false;
    
    // Grid settings
    int   m_cols = 40;
    int   m_rows = 30;
    double m_spacing = 1e11; // meters between lines (roughly solar system scale)

    struct HeavyBody {
        sf::Vector2f pos;
        float mass;
        bool is_bh;
        float radius;
    };
    mutable std::vector<HeavyBody> m_top_bodies;
};
