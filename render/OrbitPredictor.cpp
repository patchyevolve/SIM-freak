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
    if (target_id.empty()) {
        m_path.clear();
        return;
    }

    // Performance Throttle: Only re-simulate if enough time has passed
    if (m_update_timer.getElapsedTime().asMilliseconds() < UPDATE_INTERVAL_MS && !m_path.empty()) {
        return;
    }
    m_update_timer.restart();

    m_path.clear();

    // 1. Get snapshot — limit to top 20 bodies when many (avoids 300×RK4 with 10k bodies)
    const auto& all_bodies = sim.bodies();
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
    
    // Adaptive prediction duration
    double predict_duration = 3600.0 * 24.0 * 365.25; // 1 year default
    
    // Estimate orbital period to avoid "boxy" paths in tight orbits
    double vel_mag = bodies[target_idx].vel.norm();
    if (bodies.size() > 1 && vel_mag > 1e-3)
    {
        // Find most influential body (M/r^2)
        double max_influence = 0;
        double best_period = predict_duration;
        
        for (size_t i = 0; i < bodies.size(); ++i) {
            if (i == target_idx) continue;
            double d2 = (bodies[i].pos - bodies[target_idx].pos).norm_sq();
            if (d2 < 1.0) continue;
            
            double influence = bodies[i].mass_kg / d2;
            if (influence > max_influence) {
                max_influence = influence;
                // Keplerian period estimate: T = 2pi * sqrt(r^3 / GM)
                double dist = std::sqrt(d2);
                double period = 2.0 * 3.14159 * std::sqrt(std::pow(dist, 3) / (G * (bodies[i].mass_kg + bodies[target_idx].mass_kg)));
                best_period = period;
            }
        }
        // Show at most 2 orbits or 1 year, whichever is smaller
        predict_duration = std::min(predict_duration, best_period * 2.0);
        // But at least 1 day
        predict_duration = std::max(predict_duration, 3600.0 * 24.0);
    }

    const int    max_points = 400; // Balanced resolution
    const int    sub_steps  = 4;   // Reduced sub-steps for speed
    const double dt_per_point = predict_duration / max_points;
    const double dt_step = dt_per_point / sub_steps;

    // 3. Step forward
    m_path.reserve(max_points);
    m_path.push_back({ bodies[target_idx].pos, 0.0 });

    for (int i = 0; i < max_points; ++i)
    {
        for (int s = 0; s < sub_steps; ++s) {
            Integrators::step(bodies, dt_step, G, soft, IntegratorType::RK4);
        }
        m_path.push_back({ bodies[target_idx].pos, (i + 1) * dt_per_point });
    }
}

void OrbitPredictor::update_preview(const Simulation& sim, Vec2 pos, Vec2 vel)
{
    // Performance Throttle: Only re-simulate if enough time has passed
    if (m_update_timer.getElapsedTime().asMilliseconds() < UPDATE_INTERVAL_MS && !m_path.empty()) {
        return;
    }
    m_update_timer.restart();

    m_path.clear();

    // 1. Prepare virtual body
    Body virtual_body;
    virtual_body.id = "__preview__";
    virtual_body.pos = pos;
    virtual_body.vel = vel;
    virtual_body.mass_kg = 1.0; // negligible mass
    virtual_body.alive = true;

    // 2. Get top 19 bodies by mass
    const auto& all_bodies = sim.bodies();
    std::vector<Body> bodies;
    bodies.reserve(20);
    bodies.push_back(virtual_body);

    std::vector<const Body*> by_mass;
    for (const auto& b : all_bodies)
        if (b.alive) by_mass.push_back(&b);
    
    std::partial_sort(by_mass.begin(),
                      by_mass.begin() + std::min<size_t>(19, by_mass.size()),
                      by_mass.end(),
                      [](const Body* a, const Body* b) { return a->mass_kg > b->mass_kg; });

    for (size_t i = 0; i < 19 && i < by_mass.size(); ++i)
        bodies.push_back(*by_mass[i]);

    // 3. Shadow sim params
    const double G = sim.config().G;
    const double soft = sim.config().softening_m;
    
    // Adaptive prediction duration
    double predict_duration = 3600.0 * 24.0 * 365.25; // 1 year default
    
    // Estimate orbital period to avoid "boxy" paths in tight orbits
    double vel_mag = bodies[0].vel.norm();
    if (bodies.size() > 1 && vel_mag > 1e-3)
    {
        double max_influence = 0;
        double best_period = predict_duration;
        
        for (size_t i = 1; i < bodies.size(); ++i) { // Skip virtual body at index 0
            double d2 = (bodies[i].pos - bodies[0].pos).norm_sq();
            if (d2 < 1.0) continue;
            
            double influence = bodies[i].mass_kg / d2;
            if (influence > max_influence) {
                max_influence = influence;
                double dist = std::sqrt(d2);
                double period = 2.0 * 3.14159 * std::sqrt(std::pow(dist, 3) / (G * (bodies[i].mass_kg + bodies[0].mass_kg)));
                best_period = period;
            }
        }
        predict_duration = std::min(predict_duration, best_period * 2.0);
        predict_duration = std::max(predict_duration, 3600.0 * 24.0);
    }

    const int    max_points = 400; // Balanced resolution
    const int    sub_steps  = 4;   // Reduced sub-steps for speed
    const double dt_per_point = predict_duration / max_points;
    const double dt_step = dt_per_point / sub_steps;

    // 4. Step forward
    m_path.reserve(max_points);
    m_path.push_back({ pos, 0.0 });

    for (int i = 0; i < max_points; ++i)
    {
        for (int s = 0; s < sub_steps; ++s) {
            Integrators::step(bodies, dt_step, G, soft, IntegratorType::RK4);
        }
        m_path.push_back({ bodies[0].pos, (i + 1) * dt_per_point });
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
