#version 330 core

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec4 aColor;

uniform mat4 u_mvp;
uniform vec2 u_view_center;

uniform vec2  u_body_pos[32];
uniform float u_body_mass[32];
uniform float u_body_is_bh[32];
uniform float u_body_radius[32];
uniform int   u_num_bodies;

uniform float u_k_meters;
uniform float u_pinch_strength;
uniform float u_G;
uniform float u_world_w;
uniform float u_world_h;
uniform float u_minor_s;

out vec4 vColor;

float smoothstep_glsl(float edge0, float edge1, float x) {
    float t = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

void main()
{
    vec2 wp = aPos;
    float z_meters = 0.0;
    vec2 lateral_pinch = vec2(0.0);
    float total_influence = 0.0;
    float color_potential = 0.0;

    float influence_far_m = u_world_w * 0.5;
    float influence_near_m = influence_far_m * 0.25;
    float visual_soft_min = u_minor_s * 1.0;
    float visual_soft_sq = visual_soft_min * visual_soft_min;

    for (int i = 0; i < u_num_bodies; ++i)
    {
        vec2 diff = u_body_pos[i] - wp;
        float r = length(diff);

        float body_influence = 1.0 - smoothstep_glsl(influence_near_m, influence_far_m, r);
        total_influence = min(1.0, total_influence + body_influence * 2.0);

        if (u_body_is_bh[i] > 0.5)
        {
            float bh_soft = max(u_body_radius[i] * 2.0, u_world_w * 0.08);
            float z_bh = (u_k_meters * u_body_mass[i] * 0.9) / (r + bh_soft);
            z_bh *= body_influence;

            float depth_smooth = smoothstep_glsl(0.0, 1.0, 1.0 - r / (influence_far_m + 1.0));
            z_bh *= (0.6 + 0.4 * depth_smooth);
            z_meters += z_bh;

            float pinch_bh = (z_bh * u_pinch_strength * 0.7) / (r + bh_soft + 1e7);
            lateral_pinch += diff * pinch_bh;
        }
        else
        {
            float physical_s2 = pow(u_body_radius[i] * 5.0, 2.0);
            float s2 = max(physical_s2, visual_soft_sq);
            float s = sqrt(s2);

            float z_i = (u_k_meters * u_body_mass[i]) / (r + s);
            z_i *= body_influence;
            z_meters += z_i;

            float nudge_factor = (z_i * u_pinch_strength) / (r + s + 1e8);
            lateral_pinch += diff * nudge_factor;
        }
        color_potential += (u_G * u_body_mass[i]) / (r + u_body_radius[i] + 1e-10);
    }

    float blend = smoothstep_glsl(0.02, 0.35, total_influence);
    z_meters *= blend;
    lateral_pinch *= blend;

    float max_depth = u_world_h * 0.28;
    z_meters = min(z_meters, max_depth);

    float max_pinch = u_minor_s * 1.2;
    float pinch_mag = length(lateral_pinch);
    if (pinch_mag > max_pinch && pinch_mag > 1e-20)
        lateral_pinch *= (max_pinch / pinch_mag);

    vec2 deformed_wp = wp + lateral_pinch;
    deformed_wp.y -= z_meters;

    float depth_norm = min(1.0, z_meters / (max_depth * 0.6));
    float recede = smoothstep_glsl(0.0, 0.9, depth_norm) * 0.18;
    deformed_wp += (u_view_center - deformed_wp) * recede;

    gl_Position = u_mvp * vec4(deformed_wp, 0.0, 1.0);

    float c_val = color_potential * 2e-9;
    float r_c = clamp(40.0/255.0 + c_val * 30.0/255.0, 20.0/255.0, 160.0/255.0);
    float g_c = clamp(80.0/255.0 + c_val * 50.0/255.0, 40.0/255.0, 210.0/255.0);
    float b_c = clamp(150.0/255.0 + c_val * 60.0/255.0, 80.0/255.0, 1.0);

    float depth_alpha = smoothstep_glsl(0.0, max_depth * 0.3, z_meters);
    float edge_x = abs(wp.x - u_view_center.x) / (u_world_w * 0.5);
    float edge_y = abs(wp.y - u_view_center.y) / (u_world_h * 0.5);
    float alpha_fade = clamp(1.4 - max(edge_x, edge_y), 0.0, 1.0);
    float alpha = clamp(alpha_fade * (50.0/255.0 + depth_alpha * 120.0/255.0), 5.0/255.0, 200.0/255.0);

    vColor = vec4(r_c, g_c, b_c, alpha);
}
