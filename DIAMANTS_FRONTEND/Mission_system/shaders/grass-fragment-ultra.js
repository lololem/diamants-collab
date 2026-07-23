// Fragment shader ULTRA QUALITY — same green palette as stable + ultra effects
export default `
varying float vElevation;
varying float vSideGradient;
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

uniform float uTime;

// Cheap noise helper
float hash12(vec2 p) {
    return fract(sin(dot(p, vec2(127.1,311.7))) * 43758.5453123);
}
float valueNoise(vec2 x) {
    vec2 i = floor(x);
    vec2 f = fract(x);
    float a = hash12(i);
    float b = hash12(i + vec2(1.0, 0.0));
    float c = hash12(i + vec2(0.0, 1.0));
    float d = hash12(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(a, b, u.x) + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
}

void main() {
    // ===== PALETTE VERTE RÉALISTE (même base que stable) =====
    vec3 springGreenTip  = vec3(0.260, 0.420, 0.180);
    vec3 springGreenBase = vec3(0.180, 0.280, 0.130);

    vec3 forestGreenTip  = vec3(0.200, 0.320, 0.150);
    vec3 forestGreenBase = vec3(0.150, 0.220, 0.110);

    // Émeraude — 3e variété pour ultra
    vec3 emeraldGreenTip  = vec3(0.220, 0.380, 0.160);
    vec3 emeraldGreenBase = vec3(0.160, 0.260, 0.120);

    // Lime subtil — 4e variété pour ultra
    vec3 limeGreenTip  = vec3(0.300, 0.420, 0.190);
    vec3 limeGreenBase = vec3(0.200, 0.300, 0.140);

    // Bruns naturels (peu fréquents)
    vec3 brownGrassTip  = vec3(0.300, 0.240, 0.160);
    vec3 brownGrassBase = vec3(0.200, 0.160, 0.110);

    // ===== DISTRIBUTION: majorité verte =====
    float random  = fract(sin(dot(vPosition.xz, vec2(12.9898, 78.233))) * 43758.5453);
    float random2 = fract(sin(dot(vPosition.xz * 1.7, vec2(78.233, 12.9898))) * 43758.5453);

    vec3 finalTipColor, finalBaseColor;

    if (random < 0.35) {
        finalTipColor = springGreenTip;
        finalBaseColor = springGreenBase;
    } else if (random < 0.60) {
        finalTipColor = forestGreenTip;
        finalBaseColor = forestGreenBase;
    } else if (random < 0.80) {
        finalTipColor = emeraldGreenTip;
        finalBaseColor = emeraldGreenBase;
    } else if (random < 0.92) {
        finalTipColor = limeGreenTip;
        finalBaseColor = limeGreenBase;
    } else {
        finalTipColor = brownGrassTip;
        finalBaseColor = brownGrassBase;
    }

    // Subtle cross-mix for diversity (15%)
    if (random2 > 0.85) {
        vec3 mixTip  = (random2 < 0.93) ? emeraldGreenTip : forestGreenTip;
        vec3 mixBase = (random2 < 0.93) ? emeraldGreenBase : forestGreenBase;
        finalTipColor  = mix(finalTipColor,  mixTip,  0.25);
        finalBaseColor = mix(finalBaseColor, mixBase, 0.25);
    }

    // ===== GRADIENT DE HAUTEUR =====
    float elevationNorm = clamp(vElevation * 0.5, 0.0, 1.0);
    vec3 grassColor = mix(finalBaseColor, finalTipColor, pow(elevationNorm, 0.7));

    // ===== SOL AU PIED DU BRIN =====
    float soilRandom = fract(sin(dot(vPosition.xz * 2.1, vec2(45.164, 23.431))) * 43758.5453);
    vec3 brownSoil = vec3(0.295, 0.205, 0.137);
    vec3 claySoil  = vec3(0.382, 0.282, 0.180);
    vec3 darkSoil  = vec3(0.176, 0.137, 0.098);
    vec3 soilColor = mix(mix(brownSoil, claySoil, 0.5), darkSoil, soilRandom);

    if (elevationNorm < 0.25) {
        float soilBlend = smoothstep(0.01, 0.25, elevationNorm);
        grassColor = mix(soilColor, grassColor, soilBlend);
    }

    // ===== ULTRA EFFECT: organic time shimmer =====
    float timeEffect = sin(uTime * 0.0012 + vPosition.x * 0.12 + vPosition.z * 0.08) * 0.06 + 1.0;
    grassColor *= timeEffect;

    // ===== GRADIENT LATÉRAL (edges plus sombres) =====
    float edgeFactor = smoothstep(0.0, 0.1, vSideGradient) * smoothstep(1.0, 0.9, vSideGradient);
    grassColor *= mix(0.6, 1.0, edgeFactor);

    // ===== MICRO VARIATIONS =====
    float micro = valueNoise(vPosition.xz * 1.3);
    grassColor *= mix(vec3(0.96), vec3(1.04), micro);

    // ===== ÉCLAIRAGE RÉALISTE =====
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    float diffuse = max(0.0, dot(normalize(vFakeNormal), lightDir));
    float ambient = 0.4;
    float lighting = ambient + diffuse * 0.6;

    vec3 finalColor = grassColor * lighting;

    // ===== BROUILLARD LÉGER =====
    vec3 fogColor = vec3(0.42, 0.50, 0.36); // vert doux (fini le blanc délavant)
    float fogDistance = length(vPosition);
    float fogFactor = exp(-fogDistance * 0.004);
    finalColor = mix(fogColor, finalColor, fogFactor);

    // ===== SATURATION & CONTRASTE (même que stable) =====
    float luminance = dot(finalColor, vec3(0.299, 0.587, 0.114));
    finalColor = mix(vec3(luminance), finalColor, 1.6);
    finalColor = pow(finalColor, vec3(1.2));

    gl_FragColor = vec4(finalColor, 1.0);
}
`;
