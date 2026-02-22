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
    
    // Event horizon (slightly smaller to allow for smooth edge)
    if (dist < rs_uv * 0.98) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    
    // Schwarzschild Deflection Approximation
    // Light is bent more strongly near the event horizon.
    // The deflection angle goes to infinity at the photon sphere (1.5 * rs).
    float safe_dist = max(dist, rs_uv * 1.01);
    float deflection = (rs_uv * 2.5) / pow(safe_dist, 1.5);
    
    vec2 warped_uv = uv - (dir / dist / aspect) * deflection * rs_uv;
    
    // Bounds check to avoid wrapping or smearing at edges
    if (warped_uv.x < 0.0 || warped_uv.x > 1.0 || warped_uv.y < 0.0 || warped_uv.y > 1.0) {
        gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0);
        return;
    }
    
    vec4 color = texture2D(texture, warped_uv);
    
    // Smooth the black hole edge
    float edge = smoothstep(rs_uv * 0.98, rs_uv * 1.02, dist);
    color.rgb *= edge;
    
    gl_FragColor = color;
}
