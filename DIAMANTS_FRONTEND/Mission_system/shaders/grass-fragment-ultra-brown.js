// Fragment shader ULTRA QUALITY - Avec vrais bruns d'herbe + effets
export default `
varying float vElevation;
varying float vSideGradient; 
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

uniform float uTime;

void main() {
    // ===== PALETTE ULTRA AVEC BRUNS D'HERBE RÉALISTES =====
    
    // Vert printemps avec base très brune
    vec3 springGreenTip = vec3(0.8, 0.9, 0.6);        // Vert ultra-saturé
    vec3 springGreenBase = vec3(0.3, 0.2, 0.1);       // Base brun-rouge
    
    // Vert émeraude avec base terre
    vec3 emeraldGreenTip = vec3(0.6, 0.85, 0.5);      // Émeraude brillant
    vec3 emeraldGreenBase = vec3(0.25, 0.18, 0.08);   // Base terre sombre
    
    // Vert forêt avec base humus
    vec3 forestGreenTip = vec3(0.5, 0.75, 0.4);       // Vert forêt riche
    vec3 forestGreenBase = vec3(0.2, 0.15, 0.07);     // Base humus
    
    // Vert lime avec base dorée
    vec3 limeGreenTip = vec3(0.75, 0.85, 0.45);       // Lime ultra-vif
    vec3 limeGreenBase = vec3(0.35, 0.25, 0.12);      // Base dorée-brune
    
    // ===== DISTRIBUTION ULTRA AVEC VARIATIONS =====
    float random = fract(sin(dot(vPosition.xz, vec2(12.9898, 78.233))) * 43758.5453);
    float random2 = fract(sin(dot(vPosition.xz * 1.7, vec2(78.233, 12.9898))) * 43758.5453);
    
    vec3 finalTipColor, finalBaseColor;
    
    // Sélection du type d'herbe (25% chacun)
    if (random < 0.25) {
        finalTipColor = springGreenTip;
        finalBaseColor = springGreenBase;
    } else if (random < 0.5) {
        finalTipColor = emeraldGreenTip;
        finalBaseColor = emeraldGreenBase;
    } else if (random < 0.75) {
        finalTipColor = forestGreenTip;
        finalBaseColor = forestGreenBase;
    } else {
        finalTipColor = limeGreenTip;
        finalBaseColor = limeGreenBase;
    }
    
    // ===== MÉLANGE ULTRA POUR DIVERSITÉ =====
    if (random2 > 0.6) {
        // 40% de chance de mélange avec un autre type
        vec3 mixTip = (random2 < 0.8) ? emeraldGreenTip : forestGreenTip;
        vec3 mixBase = (random2 < 0.8) ? emeraldGreenBase : forestGreenBase;
        
        finalTipColor = mix(finalTipColor, mixTip, 0.3);
        finalBaseColor = mix(finalBaseColor, mixBase, 0.3);
    }
    
    // ===== GRADIENT VERTICAL AVEC SOL BRUN =====
    float elevationNorm = clamp(vElevation * 0.5, 0.0, 1.0);
    vec3 grassColor = mix(finalBaseColor, finalTipColor, pow(elevationNorm, 0.6));
    
    // ===== SOL ULTRA-BRUN À LA BASE =====
    float soilRandom = fract(sin(dot(vPosition.xz * 2.1, vec2(45.164, 23.431))) * 43758.5453);
    
    vec3 richBrownSoil = vec3(0.4, 0.25, 0.15);       // Terre riche brune
    vec3 darkBrownSoil = vec3(0.3, 0.18, 0.1);        // Terre sombre
    
    vec3 soilColor = mix(richBrownSoil, darkBrownSoil, soilRandom);
    
    // Zone de transition sol-herbe plus large pour ULTRA
    float soilFactor = 1.0 - pow(elevationNorm, 0.4);
    grassColor = mix(grassColor, soilColor, soilFactor * 0.7);
    
    // ===== EFFETS ULTRA: ANIMATION TEMPORELLE =====
    float timeEffect = sin(uTime * 0.0008 + vPosition.x * 0.1) * 0.08 + 1.0;
    grassColor *= timeEffect;
    
    // ===== GRADIENT LATÉRAL AVEC BRUN =====
    float sideEffect = pow(vSideGradient, 0.8);
    grassColor = mix(grassColor, soilColor * 1.1, sideEffect * 0.4);
    
    // ===== ÉCLAIRAGE ULTRA =====
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 normal = normalize(vFakeNormal);
    float NdotL = max(dot(normal, lightDir), 0.3);
    
    // ===== VARIATIONS MICRO-DÉTAILLÉES =====
    float microVar = sin(vPosition.x * 25.0) * cos(vPosition.z * 20.0);
    grassColor *= (1.0 + microVar * 0.12);
    
    // ===== RENDU FINAL ULTRA =====
    vec3 finalColor = grassColor * NdotL * 1.15; // Léger boost de luminosité
    finalColor = pow(finalColor, vec3(1.0/1.9)); // Gamma correction pour richesse
    
    gl_FragColor = vec4(finalColor, 1.0);
}
`;
