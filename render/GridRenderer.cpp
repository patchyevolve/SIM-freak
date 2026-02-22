// =============================================================================
//  render/GridRenderer.cpp
// =============================================================================

#include "GridRenderer.h"
#include "../math/Vec2.h"
#include <cmath>
#include <algorithm>

GridRenderer::GridRenderer()
{
    // We'll generate the grid dynamically in draw() based on camera view
    // to handle infinite panning/zooming efficiently.
}

void GridRenderer::draw(sf::RenderTarget& target, const Camera& cam, const Simulation& sim) const
{
    // ── 1. Lazy Shader Load ──────────────────────────────────────────────────
    if (!m_shader_loaded)
    {
        if (m_shader.loadFromFile("render/grid.vert", "render/grid.frag"))
            m_shader_loaded = true;
        else if (m_shader.loadFromFile("../render/grid.vert", "../render/grid.frag"))
            m_shader_loaded = true;
    }

    sf::Vector2u size = target.getSize();
    float view_w = static_cast<float>(size.x);
    float view_h = static_cast<float>(size.y);

    Vec2 tl = cam.screen_to_world({ 0.0f, 0.0f });
    Vec2 br = cam.screen_to_world({ view_w, view_h });

    // ── 2. Spacing & Bounds ──────────────────────────────────────────────────
    double world_w = std::abs(br.x - tl.x);
    double world_h = std::abs(br.y - tl.y);
    double base    = std::pow(10.0, std::floor(std::log10(world_w / 12.0)));
    double ratio   = (world_w / 12.0) / base;
    
    double minor_s = base;
    if (ratio > 5.0)      minor_s *= 5.0;
    else if (ratio > 2.5) minor_s *= 2.5;
    double major_s = minor_s * 5.0;

    double x_min = std::min(tl.x, br.x);
    double x_max = std::max(tl.x, br.x);
    double y_min = std::min(tl.y, br.y);
    double y_max = std::max(tl.y, br.y);

    long long ix_start = static_cast<long long>(std::floor(x_min / minor_s)) - 1;
    long long ix_end   = static_cast<long long>(std::ceil(x_max / minor_s)) + 1;
    long long iy_start = static_cast<long long>(std::floor(y_min / minor_s)) - 1;
    long long iy_end   = static_cast<long long>(std::ceil(y_max / minor_s)) + 1;

    // Safety: Cap lines
    if (ix_end - ix_start > 200) ix_end = ix_start + 200;
    if (iy_end - iy_start > 200) iy_end = iy_start + 200;

    // ── 3. Find Top Influencers (Top 32 heaviest bodies) ─────────────────────
    m_top_bodies.clear();
    const auto& bodies = sim.bodies();
    std::vector<const Body*> sorted;
    sorted.reserve(bodies.size());
    for(const auto& b : bodies) if(b.alive) sorted.push_back(&b);
    
    std::sort(sorted.begin(), sorted.end(), [](const Body* a, const Body* b){
        return a->mass_kg > b->mass_kg;
    });

    int num_to_send = std::min((int)sorted.size(), 32);
    static std::vector<sf::Vector2f> b_pos(32);
    static std::vector<float>        b_mass(32);
    static std::vector<float>        b_is_bh(32);
    static std::vector<float>        b_radius(32);

    for (int i = 0; i < num_to_send; ++i) {
        b_pos[i]    = sf::Vector2f(static_cast<float>(sorted[i]->pos.x), static_cast<float>(sorted[i]->pos.y));
        b_mass[i]   = static_cast<float>(sorted[i]->mass_kg);
        b_is_bh[i]  = (sorted[i]->kind == BodyKind::BlackHole) ? 1.0f : 0.0f;
        b_radius[i] = static_cast<float>(sorted[i]->radius_m);
    }

    // ── 4. Setup Shader ──────────────────────────────────────────────────────
    sf::RenderStates rs;
    if (m_shader_loaded)
    {
        sf::Transform mvp = target.getView().getTransform() * cam.get_transform();
        Vec2 center = cam.center();
        m_shader.setUniform("u_mvp", sf::Glsl::Mat4(mvp.getMatrix()));
        m_shader.setUniform("u_view_center", sf::Vector2f(static_cast<float>(center.x), static_cast<float>(center.y)));
        m_shader.setUniform("u_world_w", static_cast<float>(world_w));
        m_shader.setUniform("u_world_h", static_cast<float>(world_h));
        m_shader.setUniform("u_minor_s", static_cast<float>(minor_s));
        m_shader.setUniform("u_k_meters", 1.3e-11f);
        m_shader.setUniform("u_pinch_strength", 0.04f);
        m_shader.setUniform("u_G", static_cast<float>(sim.config().G));
        m_shader.setUniform("u_num_bodies", num_to_send);
        
        if (num_to_send > 0) {
            m_shader.setUniformArray("u_body_pos",  b_pos.data(),  num_to_send);
            m_shader.setUniformArray("u_body_mass", b_mass.data(), num_to_send);
            m_shader.setUniformArray("u_body_is_bh", b_is_bh.data(), num_to_send);
            m_shader.setUniformArray("u_body_radius", b_radius.data(), num_to_send);
        }
        rs.shader = &m_shader;
    }

    // ── 5. Build Flat Mesh ───────────────────────────────────────────────────
    sf::VertexArray batch(sf::Lines);
    
    auto add_line_flat = [&](Vec2 p1, Vec2 p2, bool major) {
        int segs = major ? 100 : 40; // High segment count for smooth 3D curves
        sf::Color c = major ? sf::Color(0, 180, 255, 80) : sf::Color(0, 100, 200, 25);
        
        Vec2 step = (p2 - p1) * (1.0 / segs);
        for (int i = 0; i < segs; ++i) {
            batch.append(sf::Vertex(sf::Vector2f(static_cast<float>(p1.x + step.x * i), 
                                                 static_cast<float>(p1.y + step.y * i)), c));
            batch.append(sf::Vertex(sf::Vector2f(static_cast<float>(p1.x + step.x * (i + 1)), 
                                                 static_cast<float>(p1.y + step.y * (i + 1))), c));
        }
    };

    bool show_minor = (world_w / minor_s < 120.0);

    for (long long i = ix_start; i <= ix_end; ++i) {
        double x = i * minor_s;
        bool major = (i % 5 == 0);
        if (major || show_minor) {
            add_line_flat({ x, (double)iy_start * minor_s }, { x, (double)iy_end * minor_s }, major);
        }
    }
    for (long long j = iy_start; j <= iy_end; ++j) {
        double y = j * minor_s;
        bool major = (j % 5 == 0);
        if (major || show_minor) {
            add_line_flat({ (double)ix_start * minor_s, y }, { (double)ix_end * minor_s, y }, major);
        }
    }

    target.draw(batch, rs);
}
