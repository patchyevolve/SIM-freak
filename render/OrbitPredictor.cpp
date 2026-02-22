// =============================================================================
//  render/OrbitPredictor.cpp
// =============================================================================

#include "OrbitPredictor.h"
#include "../physics/Integrators.h"
#include <cmath>
#include <algorithm>

OrbitPredictor::OrbitPredictor() {}

void OrbitPredictor::update(const Simulation& sim, const std::string& target_id)
{
    m_path.clear();
    if (target_id.empty()) return;

    // 1. Get snapshot — limit to top 20 bodies when many (avoids 300×RK4 with 10k bodies)
    auto all_bodies = sim.bodies();
    std::vector<Body> bodies;

    const Body* target_ptr = nullptr;
    for (const auto& b : all_bodies)
        if (b.id == target_id) { target_ptr = &b; break; }
    if (!target_ptr) return;

    if (all_bodies.size() <= 20) {
        bodies = all_bodies;
    } else {
        // Keep target + top 19 by mass (excluding target)
        bodies.reserve(20);
        bodies.push_back(*target_ptr);

        std::vector<const Body*> by_mass;
        for (const auto& b : all_bodies)
            if (b.id != target_id && b.alive) by_mass.push_back(&b);
        std::partial_sort(by_mass.begin(),
                          by_mass.begin() + std::min<size_t>(19, by_mass.size()),
                          by_mass.end(),
                          [](const Body* a, const Body* b) { return a->mass_kg > b->mass_kg; });

        for (size_t i = 0; i < 19 && i < by_mass.size(); ++i)
            bodies.push_back(*by_mass[i]);
    }

    size_t target_idx = 0;
    for (size_t i = 0; i < bodies.size(); ++i)
        if (bodies[i].id == target_id) { target_idx = i; break; }

    // 2. Shadow sim params
    const double G = sim.config().G;
    const double soft = sim.config().softening_m;
    const double predict_duration = 3600.0 * 24.0 * 365.25; // 1 year
    const int    max_points = 300;
    const double dt = predict_duration / max_points;

    // 3. Step forward
    m_path.reserve(max_points);
    m_path.push_back({ bodies[target_idx].pos, 0.0 });

    for (int i = 0; i < max_points; ++i)
    {
        Integrators::step(bodies, dt, G, soft, IntegratorType::RK4);
        m_path.push_back({ bodies[target_idx].pos, (i + 1) * dt });
    }
}

void OrbitPredictor::draw(sf::RenderTarget& target, const Camera& cam) const
{
    if (m_path.size() < 2) return;

    sf::VertexArray lines(sf::LineStrip);
    for (size_t i = 0; i < m_path.size(); ++i)
    {
        sf::Vector2f sp = cam.world_to_screen(m_path[i].pos);
        
        // Fade out dots based on "futureness"
        sf::Color c = m_line_color;
        float alpha_factor = 1.0f - (static_cast<float>(i) / m_path.size());
        c.a = static_cast<uint8_t>(c.a * alpha_factor);

        lines.append(sf::Vertex(sp, c));
    }
    target.draw(lines);
}
