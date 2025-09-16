// Fragment shader ULTRA QUALITY - Version simplifiée pour debug
export default `
varying float vElevation;
varying float vSideGradient; 
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

uniform float uTime;

void main() {
    // ===== COULEURS ULTRA SATURÉES =====
    vec3 grassTip = vec3(0.2, 0.8, 0.1);      // Vert ULTRA saturé
    vec3 grassBase = vec3(0.1, 0.5, 0.05);    // Base verte intense  
    vec3 grassBrown = vec3(0.6, 0.4, 0.1);    // Brun doré
    
    // ===== GRADIENT VERTICAL =====
    float normalizedHeight = clamp(vElevation / 2.0, 0.0, 1.0);
    vec3 grassColor = mix(grassBrown, grassBase, normalizedHeight);
    grassColor = mix(grassColor, grassTip, pow(normalizedHeight, 1.0));
    
    // ===== EFFET ULTRA: ANIMATION TEMPORELLE =====
    float timeEffect = sin(uTime * 0.001 + vPosition.x * 0.1) * 0.1 + 1.0;
    grassColor *= timeEffect;
    
    // ===== GRADIENT LATÉRAL ULTRA =====
    float sideEffect = pow(vSideGradient, 0.8);
    grassColor = mix(grassColor, grassBrown * 1.2, sideEffect * 0.3);
    
    // ===== ÉCLAIRAGE ULTRA =====
    vec3 lightDir = normalize(vec3(0.5, 1.0, 0.3));
    vec3 normal = normalize(vFakeNormal);
    float NdotL = max(dot(normal, lightDir), 0.4);
    
    // ===== SATURATION ULTRA =====
    grassColor *= 1.5; // Boost de saturation
    
    // ===== VARIATIONS ULTRA =====
    float variation = sin(vPosition.x * 20.0) * cos(vPosition.z * 15.0);
    grassColor *= (1.0 + variation * 0.15);
    
    // ===== RENDU FINAL ULTRA =====
    vec3 finalColor = grassColor * NdotL * 1.2; // Brightness boost
    finalColor = pow(finalColor, vec3(1.0/1.8)); // Gamma correction plus agressive
    
    gl_FragColor = vec4(finalColor, 1.0);
}
`;
