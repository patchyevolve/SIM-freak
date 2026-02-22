#version 110

uniform sampler2D texture;
uniform vec2 direction; // (1, 0) for horizontal, (0, 1) for vertical
uniform vec2 resolution;

void main() {
    vec2 uv = gl_TexCoord[0].xy / resolution;
    vec2 off = direction / resolution;
    
    vec4 color = vec4(0.0);
    // Gaussian weights: 0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216
    color += texture2D(texture, uv) * 0.227027;
    color += texture2D(texture, uv + off * 1.3846153) * 0.3162162;
    color += texture2D(texture, uv - off * 1.3846153) * 0.3162162;
    color += texture2D(texture, uv + off * 3.2307692) * 0.0702703;
    color += texture2D(texture, uv - off * 3.2307692) * 0.0702703;
    
    gl_FragColor = color;
}
