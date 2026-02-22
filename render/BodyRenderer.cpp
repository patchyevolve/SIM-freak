// =============================================================================
//  render/BodyRenderer.cpp
// =============================================================================

#include "BodyRenderer.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <array>
#include <sstream>
#include <iomanip>

// ── Constructor ────────────────────────────────────────────────────────────────
BodyRenderer::BodyRenderer(const sf::Font& font) : m_font(font) {}

// ── Colour helper ──────────────────────────────────────────────────────────────
sf::Color BodyRenderer::body_color(const Body& b, const Camera& cam) const
{
    uint32_t c = b.render.color;
    sf::Color base((c >> 24) & 0xFF, (c >> 16) & 0xFF, (c >> 8) & 0xFF, 255);

    // Phase 21: Relativistic Visuals
    const double c_speed = 299792458.0;

    // 1. Gravitational Redshift (Potential based)
    // z_grav = 1/scale - 1. 
    double shift_red = 0.0;
    if (b.local_time_scale < 0.99) {
        shift_red = (1.0 / std::max(0.01, b.local_time_scale)) - 1.0;
    }

    // 2. Doppler Shift (Velocity based)
    // Radial velocity relative to camera center
    Vec2 cam_world = cam.screen_to_world(sf::Vector2f(400, 300)); // approx center
    Vec2 to_cam = cam_world - b.pos;
    double dist = to_cam.norm();
    double shift_doppler = 0.0;
    if (dist > 1e6) {
        double v_radial = (b.vel.x * to_cam.x + b.vel.y * to_cam.y) / dist;
        shift_doppler = v_radial / c_speed; 
    }

    // Combined Z shift (simplified)
    double total_z = (shift_red * 2.0) - shift_doppler; // Positive total_z = Redder

    if (std::abs(total_z) < 0.001) return base;

    // Apply color shift (lerp towards Red or Blue)
    float r = static_cast<float>(base.r);
    float g = static_cast<float>(base.g);
    float b_val = static_cast<float>(base.b);

    if (total_z > 0) { // Redshift
        float factor = static_cast<float>(std::min(0.7, total_z * 0.5)); // Damped
        r = r + (255.0f - r) * factor;
        g = g * (1.0f - factor * 0.5f);
        b_val = b_val * (1.0f - factor * 0.8f);
    } else { // Blueshift
        float factor = static_cast<float>(std::min(0.7, -total_z * 1.5)); // Damped
        r = r * (1.0f - factor * 0.8f);
        g = g * (1.0f - factor * 0.3f);
        b_val = b_val + (255.0f - b_val) * factor;
    }

    return sf::Color(
        static_cast<uint8_t>(std::clamp(r, 0.0f, 255.0f)),
        static_cast<uint8_t>(std::clamp(g, 0.0f, 255.0f)),
        static_cast<uint8_t>(std::clamp(b_val, 0.0f, 255.0f)),
        255
    );
}

// ── Radius: purely from body.radius_m and camera zoom (far zoom out = way to see things; relative sizes stay correct)
float BodyRenderer::clamped_radius(const Body& b, const Camera& cam) const
{
    float r = cam.world_radius_to_screen(b.radius_m);
    if (r < MIN_RADIUS_PX) r = MIN_RADIUS_PX;
    if (r > MAX_RADIUS_PX) r = MAX_RADIUS_PX;
    return r;
}

// ── Sub-draw helpers ───────────────────────────────────────────────────────────

void BodyRenderer::draw_body_circle(sf::RenderTarget& t,
                                    sf::Vector2f pos,
                                    float radius,
                                    sf::Color color) const
{
    sf::CircleShape circle(radius);
    circle.setFillColor(color);
    circle.setOrigin(radius, radius);
    circle.setPosition(pos);
    t.draw(circle);
}

void BodyRenderer::draw_selection_halo(sf::RenderTarget& t,
                                       sf::Vector2f pos,
                                       float radius) const
{
    float halo_r = radius * HALO_SCALE;
    sf::CircleShape halo(halo_r, 48);
    halo.setFillColor(sf::Color::Transparent);
    halo.setOutlineColor(sf::Color(255, 255, 100, 200));
    halo.setOutlineThickness(1.5f);
    halo.setOrigin(halo_r, halo_r);
    halo.setPosition(pos);
    t.draw(halo);
}

void BodyRenderer::draw_velocity_arrow(sf::RenderTarget& t,
                                        sf::Vector2f pos,
                                        const Body& body,
                                        const Camera& cam) const
{
    // Arrow tip in screen space
    double scale    = VEL_ARROW_SCALE;
    float  tip_sx   = pos.x + static_cast<float>(body.vel.x * scale);
    float  tip_sy   = pos.y - static_cast<float>(body.vel.y * scale);

    sf::Vertex line[2];
    line[0].position = pos;
    line[0].color    = sf::Color(200, 200, 200, 180);
    line[1].position = { tip_sx, tip_sy };
    line[1].color    = sf::Color(255, 255, 120, 220);
    t.draw(line, 2, sf::Lines);

    // Small arrowhead dot at tip
    sf::CircleShape tip(2.5f);
    tip.setFillColor(sf::Color(255, 255, 120, 220));
    tip.setOrigin(2.5f, 2.5f);
    tip.setPosition({ tip_sx, tip_sy });
    t.draw(tip);
}

void BodyRenderer::draw_label(sf::RenderTarget& t,
                               sf::Vector2f pos,
                               float radius,
                               const Body& body) const
{
    if (!body.render.draw_label) return;

    sf::Text label;
    label.setFont(m_font);
    label.setString(body.name);
    label.setCharacterSize(11);
    label.setFillColor(sf::Color(220, 220, 220, 200));
    label.setOutlineColor(sf::Color(0, 0, 0, 150));
    label.setOutlineThickness(1.0f);

    // Phase 21: Proper Time Indicator
    std::string name_str = body.name;
    if (body.proper_time_s > 0.1) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(1);
        oss << " (T:" << body.proper_time_s << "s)";
        name_str += oss.str();
    }
    label.setString(name_str);

    // Centre horizontally above the circle
    auto bounds = label.getLocalBounds();
    label.setOrigin(bounds.left + bounds.width / 2.0f, bounds.top + bounds.height);
    label.setPosition(pos.x, pos.y - radius - 4.0f);
    t.draw(label);
}

// ── Phase 9: Detailed Visuals ───────────────────────────────────────────

void BodyRenderer::draw_atmosphere(sf::RenderTarget& t, sf::Vector2f pos,
                                   const Body& b, float screen_radius) const
{
    if (!b.render.has_atmosphere()) return;

    uint32_t ac = b.render.atmosphere_color;
    sf::Color color((ac >> 24) & 0xFF, (ac >> 16) & 0xFF, (ac >> 8) & 0xFF, 0);

    // Multiple layers for soft scattering
    // Outer glow
    int layers = 6;
    float step = screen_radius * 0.15f;
    for (int i = 1; i <= layers; ++i) {
        float r = screen_radius + (layers - i + 1) * step;
        sf::Color layer_color = color;
        layer_color.a = static_cast<uint8_t>(25.0f * i / layers); 
        
        sf::CircleShape glow(r);
        glow.setOrigin(r, r);
        glow.setPosition(pos);
        glow.setFillColor(layer_color);
        t.draw(glow);
    }
}

void BodyRenderer::draw_rings(sf::RenderTarget& t, sf::Vector2f pos,
                             const Body& b, float screen_radius) const
{
    if (!b.render.has_rings) return;

    uint32_t rc = b.render.ring_color;
    sf::Color color((rc >> 24) & 0xFF, (rc >> 16) & 0xFF, (rc >> 8) & 0xFF, (rc & 0xFF));

    // Simple elliptical ring using vertex array for tilt
    // Saturn rings are roughly 2.3x the planet radius
    float inner = screen_radius * 1.5f;
    float outer = screen_radius * 2.3f;

    sf::VertexArray va(sf::Lines, 128);
    for (int i = 0; i < 64; ++i) {
        float theta = (i / 64.0f) * 2.0f * 3.14159f;
        float next  = ((i + 1) / 64.0f) * 2.0f * 3.14159f;
        
        // Perspective "Squash" (0.4f) and Tilt
        auto get_pt = [&](float r, float angle) {
            float x = std::cos(angle) * r;
            float y = std::sin(angle) * r * 0.4f; 
            return pos + sf::Vector2f(x, y);
        };

        va[i*2].position = get_pt(inner, theta);
        va[i*2].color    = color;
        va[i*2+1].position = get_pt(outer, next); // messy connection but looks "dusty"
        va[i*2+1].color    = color;
    }
    t.draw(va);
}

void BodyRenderer::draw_soi(sf::RenderTarget& t, sf::Vector2f pos,
                           const Body& b, const Body& primary, const Camera& cam) const
{
    // r_soi = a * (m/M)^0.4
    double dist = b.dist_to(primary);
    double m = b.mass_kg;
    double M = primary.mass_kg;
    
    if (dist <= 0.0 || M <= 0.0) return;
    
    double r_soi_m = dist * std::pow(m / M, 0.4);
    float  r_soi_px = cam.world_radius_to_screen(r_soi_m);
    
    // Only draw if it's reasonably visible and not too huge 
    if (r_soi_px <= 2.0f || r_soi_px > 1000.0f) return;

    sf::CircleShape soi(r_soi_px);
    soi.setOrigin(r_soi_px, r_soi_px);
    soi.setPosition(pos);
    soi.setFillColor(sf::Color::Transparent);
    soi.setOutlineColor(sf::Color(255, 255, 255, 30)); // very faint white
    soi.setOutlineThickness(1.0f);
    t.draw(soi);
}

// ── Tiered Batch Draw ────────────────────────────────────────────────────────
void BodyRenderer::draw_all(sf::RenderTarget& target,
                            const std::vector<Body>& bodies,
                            const Camera& cam,
                            const std::string& selected_id) const
{
    // 1. Identify primary star (for SOI visuals)
    const Body* primary = nullptr;
    double max_mass = 0.0;
    for (const auto& b : bodies) {
        if (b.alive && b.kind == BodyKind::Star && b.mass_kg > max_mass) {
            max_mass = b.mass_kg;
            primary = &b;
        }
    }

    // 2. Prepare Viewport context for culling
    sf::Vector2u screen_size = target.getSize();
    float sw = static_cast<float>(screen_size.x);
    float sh = static_cast<float>(screen_size.y);
    float margin = 50.0f; // Culling margin

    // 3. Batch arrays for low-detail bodies (Particles as octagons)
    sf::VertexArray particle_batch(sf::Triangles);
    particle_batch.clear();

    // 4. Sort/Filter passes
    std::vector<const Body*> high_detail_list;

    for (const auto& b : bodies) {
        if (!b.alive) continue;

        sf::Vector2f sp = cam.world_to_screen(b.pos);
        float r = clamped_radius(b, cam);

        // Frustum Culling
        if (sp.x < -margin || sp.x > sw + margin ||
            sp.y < -margin || sp.y > sh + margin) continue;

        bool is_selected = (b.id == selected_id);
        
        // Stars and Black Holes are usually "high detail", but for 10,000 bodies
        // we MUST batch the stars if they are tiny, or we drop to 1 FPS.
        bool force_high_detail = (b.kind == BodyKind::BlackHole || is_selected);
        bool is_large = (r > LOD_BATCH_THRESHOLD);

        if (force_high_detail || is_large) {
            high_detail_list.push_back(&b);
        } else {
            sf::Color color = body_color(b, cam);
            
            // Speed optimization: Use 4-vertex diamonds (2 triangles) for low-detail stars
            // instead of 8-vertex octagons. This reduces vertex count from 24 to 6 per body.
            sf::Vector2f v_center = sp;
            
            // Triangle 1
            particle_batch.append(sf::Vertex(v_center + sf::Vector2f(0, -r), color));
            particle_batch.append(sf::Vertex(v_center + sf::Vector2f(r, 0), color));
            particle_batch.append(sf::Vertex(v_center + sf::Vector2f(-r, 0), color));
            
            // Triangle 2
            particle_batch.append(sf::Vertex(v_center + sf::Vector2f(r, 0), color));
            particle_batch.append(sf::Vertex(v_center + sf::Vector2f(0, r), color));
            particle_batch.append(sf::Vertex(v_center + sf::Vector2f(-r, 0), color));
        }
    }

    // 5. Execution
    // Draw particles in a single call
    if (particle_batch.getVertexCount() > 0) {
        target.draw(particle_batch);
    }

    // Draw high detail bodies individually
    for (const auto* b : high_detail_list) {
        const Body* p = (primary && primary->id != b->id) ? primary : nullptr;
        draw(target, *b, cam, p, b->id == selected_id);
    }
}

void BodyRenderer::draw_black_hole(sf::RenderTarget& t, sf::Vector2f pos,
                                 const Body& b, float screen_radius) const
{
    uint32_t dc = b.render.disk_color;
    sf::Color disk_color((dc >> 24) & 0xFF, (dc >> 16) & 0xFF, (dc >> 8) & 0xFF, (dc & 0xFF));

    float inner_r = screen_radius * 1.35f;
    float outer_r = screen_radius * 5.5f;

    static float disk_rotation = 0.0f;
    disk_rotation += 0.012f;

    const float PI = 3.14159265f;
    // Disk shadow: "back" half of disk (in BH shadow) — viewer sees top, so bottom half is shadowed
    const float shadow_angle_start = PI * 0.25f;
    const float shadow_angle_end   = PI * 0.75f;

    // 1. Strong outer glow (photon sphere / lensing halo) — many layers, exponential falloff
    for (int i = 12; i >= 1; --i) {
        float r = screen_radius + i * 8.0f;
        sf::CircleShape glow(r);
        glow.setOrigin(r, r);
        glow.setPosition(pos);
        int alpha_val = std::min(140 / i, 255);
        uint8_t alpha = static_cast<uint8_t>(alpha_val);
        int b_val = std::min(static_cast<int>(disk_color.b) + 40, 255);
        sf::Color g(disk_color.r, disk_color.g, static_cast<uint8_t>(b_val), alpha);
        glow.setFillColor(g);
        glow.setOutlineColor(sf::Color(255, 240, 200, alpha / 4));
        glow.setOutlineThickness(0.3f);
        t.draw(glow);
    }
    for (int i = 6; i >= 1; --i) {
        float r = screen_radius + i * 4.0f;
        sf::CircleShape glow(r);
        glow.setOrigin(r, r);
        glow.setPosition(pos);
        int alpha_val = std::min(100 / i, 255);
        uint8_t alpha = static_cast<uint8_t>(alpha_val);
        glow.setFillColor(sf::Color(255, 230, 180, alpha));
        t.draw(glow);
    }

    // 2. Accretion disk with shadow (dark crescent on the far side)
    sf::VertexArray va(sf::TriangleStrip);
    for (int i = 0; i <= 80; ++i) {
        float theta = (i / 80.0f) * 2.0f * PI + disk_rotation;
        auto get_v = [&](float r, float angle) {
            float x = std::cos(angle) * r;
            float y = std::sin(angle) * r * 0.38f;
            return pos + sf::Vector2f(x, y);
        };
        float norm_angle = std::fmod(theta + PI, 2.0f * PI);
        bool in_shadow = (norm_angle >= shadow_angle_start && norm_angle <= shadow_angle_end);
        sf::Color c = disk_color;
        if (in_shadow) {
            c.r = static_cast<uint8_t>(c.r / 4);
            c.g = static_cast<uint8_t>(c.g / 4);
            c.b = static_cast<uint8_t>(c.b / 4);
            c.a = static_cast<uint8_t>(120.0f * (1.1f - (float)i / 80.0f));
        } else {
            c.a = static_cast<uint8_t>(240.0f * (1.2f - (float)i / 80.0f));
        }
        va.append(sf::Vertex(get_v(inner_r, theta), c));
        sf::Color c_outer(c.r, c.g, c.b, in_shadow ? 25 : 50);
        va.append(sf::Vertex(get_v(outer_r, theta), c_outer));
    }
    t.draw(va);

    // 3. Hot inner ring (glare — hottest part of accretion disk)
    float glare_inner = screen_radius * 1.5f;
    float glare_outer = screen_radius * 2.4f;
    sf::VertexArray glare_va(sf::TriangleStrip);
    for (int i = 0; i <= 64; ++i) {
        float theta = (i / 64.0f) * 2.0f * PI + disk_rotation * 0.7f;
        float x0 = std::cos(theta) * glare_inner;
        float y0 = std::sin(theta) * glare_inner * 0.4f;
        float x1 = std::cos(theta) * glare_outer;
        float y1 = std::sin(theta) * glare_outer * 0.4f;
        glare_va.append(sf::Vertex(pos + sf::Vector2f(x0, y0), sf::Color(255, 255, 220, 230)));
        glare_va.append(sf::Vertex(pos + sf::Vector2f(x1, y1), sf::Color(255, 200, 100, 80)));
    }
    t.draw(glare_va);

    // 4. Hotspots (bright arcs on the disk)
    for (int arc = 0; arc < 3; ++arc) {
        float arc_center = disk_rotation * 1.3f + arc * 2.09f;
        float arc_r = inner_r + (outer_r - inner_r) * (0.2f + 0.3f * (arc * 0.33f));
        sf::VertexArray hot(sf::LineStrip);
        for (int i = 0; i <= 12; ++i) {
            float theta = arc_center - 0.15f + (i / 12.0f) * 0.3f;
            float x = std::cos(theta) * arc_r;
            float y = std::sin(theta) * arc_r * 0.4f;
            uint8_t a = static_cast<uint8_t>(255 - i * 18);
            hot.append(sf::Vertex(pos + sf::Vector2f(x, y), sf::Color(255, 255, 200, a)));
        }
        t.draw(hot);
    }

    // 5. Event horizon (solid black + bright rim)
    sf::CircleShape core(screen_radius);
    core.setOrigin(screen_radius, screen_radius);
    core.setPosition(pos);
    core.setFillColor(sf::Color::Black);
    core.setOutlineColor(sf::Color(255, 245, 200, 220));
    core.setOutlineThickness(3.0f);
    t.draw(core);

    // 6. Innermost dark ring (shadow boundary)
    sf::CircleShape shadow(screen_radius * 1.015f);
    shadow.setOrigin(shadow.getRadius(), shadow.getRadius());
    shadow.setPosition(pos);
    shadow.setFillColor(sf::Color(0, 0, 0, 0));
    shadow.setOutlineColor(sf::Color(20, 15, 10, 240));
    shadow.setOutlineThickness(1.5f);
    t.draw(shadow);
}

// ── Public draw ───────────────────────────────────────────────────────────────

void BodyRenderer::draw(sf::RenderTarget& target,
                         const Body&  body,
                         const Camera& cam,
                         const Body*  primary,
                         bool         selected) const
{
    if (!body.alive) return;

    sf::Vector2f screen_pos = cam.world_to_screen(body.pos);
    float        radius     = clamped_radius(body, cam);
    sf::Color    color      = body_color(body, cam);

    if (body.kind == BodyKind::BlackHole)
    {
        draw_black_hole(target, screen_pos, body, radius);
        if (selected) draw_selection_halo(target, screen_pos, radius);
        draw_label(target, screen_pos, radius, body);
        return;
    }

    // 1. Atmosphere / Glow (Background)
    if (body.kind == BodyKind::Star && radius >= MIN_RADIUS_PX)
    {
        sf::Color glow_color(color.r, color.g, color.b, 40);
        draw_body_circle(target, screen_pos, radius * 2.5f, glow_color);
        sf::Color mid_glow(color.r, color.g, color.b, 80);
        draw_body_circle(target, screen_pos, radius * 1.6f, mid_glow);
        // Note: For implementation simplicity, atmosphere is the same
        draw_atmosphere(target, screen_pos, body, radius);
    }
    else if (body.render.has_atmosphere())
    {
        draw_atmosphere(target, screen_pos, body, radius);
    }
    
    // 2. SOI Visualization (Phase 10)
    if (primary && body.kind != BodyKind::Star)
    {
        draw_soi(target, screen_pos, body, *primary, cam);
    }

    // 3. Rings (Background part)
    if (body.render.has_rings)
        draw_rings(target, screen_pos, body, radius);

    // 4. Selection halo
    if (selected)
        draw_selection_halo(target, screen_pos, radius);

    // 4. Main circle
    draw_body_circle(target, screen_pos, radius, color);

    // 5. Annotations
    if (selected || radius > 5.0f)
        draw_velocity_arrow(target, screen_pos, body, cam);

    draw_label(target, screen_pos, radius, body);
}
