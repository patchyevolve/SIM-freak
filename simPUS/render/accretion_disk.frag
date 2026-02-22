#version 110

uniform float time;
uniform vec4 base_color;
uniform float inner_radius;
uniform float outer_radius;

float noise(vec2 p) {
    return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

void main() {
    // Pixel-space UVs from [0, 2*outer_radius]
    vec2 uv = (gl_TexCoord[0].xy / (outer_radius * 2.0)) * 2.0 - 1.0;
    uv.y /= 0.38; // Account for the 0.38f flattening in BodyRenderer
    float dist = length(uv);
    
    float norm_dist = (dist * outer_radius - inner_radius) / (outer_radius - inner_radius);
    
    if (dist > 1.0 || dist < (inner_radius/outer_radius)) discard;

    float angle = atan(uv.y, uv.x) + time * 0.5;
    
    // Procedural banding
    float bands = sin(dist * 40.0 - time * 2.0) * 0.5 + 0.5;
    bands *= sin(dist * 100.0 + time) * 0.2 + 0.8;
    
    // Doppler beaming simulation: brighter on one side
    float doppler = 0.6 + 0.4 * cos(atan(uv.y, uv.x));
    
    // Falloff
    float alpha = smoothstep(0.0, 0.1, norm_dist) * (1.0 - smoothstep(0.8, 1.0, norm_dist));
    
    vec3 col = base_color.rgb * bands * doppler;
    col += vec3(1.0, 0.9, 0.7) * pow(1.0 - norm_dist, 4.0) * 0.5; // Inner heat glow
    
    gl_FragColor = vec4(col, alpha * base_color.a);
}
