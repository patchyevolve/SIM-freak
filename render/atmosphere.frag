#version 110

uniform float time;
uniform vec4 atmos_color;
uniform float planet_radius; // screen space
uniform float atmos_thickness; // screen space

void main() {
    float atmos_r = planet_radius + atmos_thickness;
    // SFML shapes use pixel-space UVs [0, 2*atmos_r]
    vec2 uv = (gl_TexCoord[0].xy / (atmos_r * 2.0)) * 2.0 - 1.0;
    float dist = length(uv);
    
    // The quad size is atmos_r * 2
    
    float surface_dist = planet_radius / atmos_r;
    if (dist < surface_dist) {
        // Inside the planet, though usually we draw the planet over this
        discard;
    }
    
    if (dist > 1.0) discard;
    
    // Simple scattering approximation: 
    // Density falls off from surface to outer edge
    float h = (dist - surface_dist) / (1.0 - surface_dist);
    float density = exp(-h * 4.0) * (1.0 - h);
    
    // Rayleigh-ish scattering: more blue/cyan at edges, thicker/richer at base
    vec3 col = atmos_color.rgb;
    
    gl_FragColor = vec4(col, density * atmos_color.a);
}
