#version 110

uniform float time;
uniform vec2 resolution;
uniform vec2 cam_pos;

// Pseudo-random noise
float hash(float n) { return fract(sin(n) * 43758.5453123); }

float noise(vec3 x) {
    vec3 p = floor(x);
    vec3 f = fract(x);
    f = f * f * (3.0 - 2.0 * f);
    float n = p.x + p.y * 57.0 + 113.0 * p.z;
    return mix(mix(mix(hash(n + 0.0), hash(n + 1.0), f.x),
                   mix(hash(n + 57.0), hash(n + 58.0), f.x), f.y),
               mix(mix(hash(n + 113.0), hash(n + 114.0), f.x),
                   mix(hash(n + 170.0), hash(n + 171.0), f.x), f.y), f.z);
}

void main() {
    // SFML untextured shapes pass UVs in pixel coordinates [0, width]
    vec2 uv = gl_TexCoord[0].xy / resolution;
    vec3 ray_origin = vec3(cam_pos * 0.0001, 10.0);
    vec3 ray_dir = normalize(vec3(uv * 2.0 - 1.0, -1.0));
    
    // Raymarching
    float acc = 0.0;
    vec3 p = ray_origin;
    for (int i = 0; i < 16; ++i) {
        float d = noise(p * 0.5 + time * 0.05);
        acc += max(0.0, d - 0.5) * 0.2;
        p += ray_dir * 0.5;
    }
    
    vec3 color = vec3(0.1, 0.05, 0.2) * acc; // Deep purple nebula
    color += vec3(0.2, 0.1, 0.0) * pow(acc, 2.0); // Orange highlight
    
    gl_FragColor = vec4(color, acc * 0.3);
}
