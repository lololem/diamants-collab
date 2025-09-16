// Fragment shader MEDIUM QUALITY - Avec bruns d'herbe simplifiés
export default `
varying float vElevation;
varying float vSideGradient; 
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

void main() {
    // ===== PALETTE MEDIUM AVEC BRUNS D'HERBE =====
    
    // Vert standard avec base brune
    vec3 standardGreenTip = vec3(0.6, 0.75, 0.4);     // Vert moyen
    vec3 standardGreenBase = vec3(0.28, 0.2, 0.12);   // Base brun-vert
    
    // Vert forêt avec base terre
    vec3 forestGreenTip = vec3(0.5, 0.65, 0.35);      // Vert forêt
    vec3 forestGreenBase = vec3(0.22, 0.16, 0.09);    // Base terre brune
    
    // ===== DISTRIBUTION SIMPLE =====
    float random = fract(sin(dot(vPosition.xz, vec2(12.9898, 78.233))) * 43758.5453);
    
    vec3 finalTipColor, finalBaseColor;
    
    // Sélection simple (50%-50%)
    if (random < 0.5) {
        finalTipColor = standardGreenTip;
        finalBaseColor = standardGreenBase;
    } else {
        finalTipColor = forestGreenTip;
        finalBaseColor = forestGreenBase;
    }
    
    // ===== GRADIENT VERTICAL =====
    float elevationNorm = clamp(vElevation * 0.5, 0.0, 1.0);
    vec3 grassColor = mix(finalBaseColor, finalTipColor, pow(elevationNorm, 0.8));
    
    // ===== SOL BRUN =====
    vec3 brownSoil = vec3(0.35, 0.22, 0.14);          // Terre brune standard
    
    float soilFactor = 1.0 - pow(elevationNorm, 0.6);
    grassColor = mix(grassColor, brownSoil, soilFactor * 0.5);
    
    // ===== GRADIENT LATÉRAL =====
    float sideEffect = pow(vSideGradient, 1.2);
    grassColor = mix(grassColor, brownSoil, sideEffect * 0.3);
    
    // ===== ÉCLAIRAGE SIMPLE =====
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 normal = normalize(vFakeNormal);
    float NdotL = max(dot(normal, lightDir), 0.3);
    
    // ===== VARIATIONS MOYENNES =====
    float variation = sin(vPosition.x * 15.0) * cos(vPosition.z * 12.0);
    grassColor *= (1.0 + variation * 0.08);
    
    // ===== RENDU FINAL MEDIUM =====
    vec3 finalColor = grassColor * NdotL;
    finalColor = pow(finalColor, vec3(1.0/2.1)); // Gamma correction
    
    gl_FragColor = vec4(finalColor, 1.0);
}
`;
