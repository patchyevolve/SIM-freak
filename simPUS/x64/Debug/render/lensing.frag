#version 110

uniform sampler2D texture;
uniform vec2 bh_pos;        // Black hole center in UV [0,1]
uniform float bh_radius;    // Event horizon radius in pixels
uniform float aspect;       // width/height
uniform vec2 resolution;    // viewport (width, height) in pixels

void main()
{
    vec2 uv = gl_TexCoord[0].xy;
    vec2 dir = uv - bh_pos;
    dir.y *= aspect;
    float dist = length(dir);
    float rs_uv = bh_radius / resolution.x;
    
    if (dist < rs_uv * 0.99) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    
    vec2 warped_uv = uv;
    float safe_dist = max(dist, rs_uv * 0.15);
    float deflect = (rs_uv * 1.8) / safe_dist;
    deflect = deflect * deflect;
    vec2 dir_n = dir / safe_dist;
    warped_uv = uv - vec2(dir_n.x, dir_n.y / aspect) * deflect * 0.4;
    
    vec4 color = texture2D(texture, warped_uv);
    
    float rim = smoothstep(rs_uv * 0.4, rs_uv * 1.5, dist);
    color.rgb *= rim;
    
    float deflection_strength = smoothstep(rs_uv * 0.6, rs_uv * 2.2, dist);
    float glare_inner = rs_uv * 1.02;
    float glare_outer = rs_uv * 3.0;
    float glare = 0.0;
    if (dist > glare_inner && dist < glare_outer) {
        float t = (dist - glare_inner) / (glare_outer - glare_inner);
        glare = exp(-t * 2.2) * (0.6 + 0.4 * deflection_strength);
    }
    color.rgb += vec3(1.0, 0.92, 0.65) * glare * 0.9;
    color.a = max(color.a, glare * 0.5);
    
    float lens_fringe = smoothstep(rs_uv * 0.99, rs_uv * 1.12, dist) * (1.0 - smoothstep(rs_uv * 1.12, rs_uv * 1.7, dist));
    color.rgb += vec3(0.88, 0.9, 1.0) * lens_fringe * 0.5;
    
    gl_FragColor = color;
}
