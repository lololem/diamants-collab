// Fragment shader LOW QUALITY - Avec bruns d'herbe basiques
export default `
varying float vElevation;
varying float vSideGradient; 
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

void main() {
    // ===== COULEURS LOW AVEC BRUN D'HERBE =====
    vec3 grassTip = vec3(0.4, 0.6, 0.2);              // Vert simple
    vec3 grassBase = vec3(0.25, 0.18, 0.1);           // Base brune simple
    vec3 brownSoil = vec3(0.3, 0.2, 0.12);            // Sol brun
    
    // ===== GRADIENT VERTICAL =====
    float normalizedHeight = clamp(vElevation / 2.0, 0.0, 1.0);
    vec3 grassColor = mix(grassBase, grassTip, pow(normalizedHeight, 1.2));
    
    // ===== SOL BRUN À LA BASE =====
    float soilFactor = 1.0 - pow(normalizedHeight, 0.8);
    grassColor = mix(grassColor, brownSoil, soilFactor * 0.4);
    
    // ===== GRADIENT LATÉRAL SIMPLE =====
    float sideEffect = pow(vSideGradient, 1.5);
    grassColor = mix(grassColor, brownSoil, sideEffect * 0.25);
    
    // ===== ÉCLAIRAGE BASIQUE =====
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 normal = normalize(vFakeNormal);
    float NdotL = max(dot(normal, lightDir), 0.4);
    
    // ===== RENDU FINAL LOW =====
    vec3 finalColor = grassColor * NdotL;
    finalColor = pow(finalColor, vec3(1.0/2.0)); // Gamma standard
    
    gl_FragColor = vec4(finalColor, 1.0);
}
`;
