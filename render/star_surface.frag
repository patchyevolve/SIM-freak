#version 110

uniform float time;
uniform vec4 base_color;
uniform float radius; // screen space radius pixels

float hash(vec2 p) {
    return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453);
}

float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = hash(i);
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float fbm(vec2 p) {
    float v = 0.0;
    float a = 0.5;
    for (int i = 0; i < 4; i++) {
        v += a * noise(p);
        p *= 2.02;
        a *= 0.5;
    }
    return v;
}

void main() {
    // SFML VertexArray TexCoords [0, 2*radius]
    vec2 uv = (gl_TexCoord[0].xy / (radius * 2.0)) * 2.0 - 1.0;
    float dist = length(uv);
    
    if (dist > 1.0) discard;

    float surface_r = 0.67;
    vec3 base = base_color.rgb;
    vec3 bright = vec3(1.0, 0.98, 0.8);
    vec3 core = base * 0.45;
    
    if (dist < surface_r) {
        // --- STAR SURFACE ---
        float limb = pow(1.0 - (dist / surface_r), 0.4);
        
        // Scale detail with radius so it stays sharp when zoomed in
        float detail = 4.0 + log(radius + 1.0) * 1.5;
        vec2 p = uv * detail;
        
        // Multi-layer plasma motion
        float n1 = fbm(p - time * 0.3);
        float n2 = fbm(p * 1.5 + n1 + time * 0.1);
        
        float plasma = smoothstep(0.1, 0.9, n2);
        vec3 col = mix(core, bright, plasma);
        
        // Add granulation / heat spikes
        col += bright * pow(plasma, 7.0) * 0.7;
        
        // Sunspots (deeper, darker)
        float spots = fbm(p * 2.8 + 15.0);
        col = mix(col, vec3(0.02, 0.01, 0.0), smoothstep(0.68, 0.85, spots) * 0.95);
        
        gl_FragColor = vec4(col * limb, 1.0);
    } else {
        // --- CORONA ---
        float corona_dist = (dist - surface_r) / (1.0 - surface_r);
        float fade = pow(1.0 - corona_dist, 4.0);
        
        float angle = atan(uv.y, uv.x);
        float rays = fbm(vec2(angle * 9.0, time * 0.6)) * 0.5 + 0.5;
        
        vec3 col = mix(base * 0.9, bright, fade * 0.5);
        gl_FragColor = vec4(col, fade * rays);
    }
}
