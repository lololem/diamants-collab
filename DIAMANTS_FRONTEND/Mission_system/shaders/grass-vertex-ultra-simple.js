// Vertex shader ULTRA QUALITY - Version simplifiée pour debug
export default `
uniform float uTime;
uniform float uSpeed;
uniform float uHalfWidth;

varying float vElevation;
varying float vSideGradient;
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

float rand(vec2 co) {
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

void main() {
    vec3 pos = position;
    vPosition = pos;
    
    // ===== ANIMATION DE VENT ULTRA COMPLEXE =====
    if (pos.y > 0.1) {
        float windTime = uTime * 0.0015 * uSpeed;
        float windStrength = 0.6; // Plus fort qu'HIGH
        
        // Vent principal ultra-fort
        pos.x += sin(windTime + pos.z * 0.8) * windStrength * pos.y;
        pos.z += sin(windTime * 0.8 + pos.x * 0.5) * windStrength * 0.4 * pos.y;
        
        // Turbulences ULTRA
        pos.x += sin(windTime * 1.7 + pos.z * 1.2) * 0.3 * pos.y;
        pos.z += cos(windTime * 1.3 + pos.x * 0.9) * 0.2 * pos.y;
        
        // Variation individuelle ULTRA
        float grassVar = rand(vec2(position.x, position.z));
        pos.x += sin(windTime * grassVar * 2.0 + grassVar * 6.28) * 0.15 * pos.y;
        pos.z += cos(windTime * grassVar * 1.5 + grassVar * 3.14) * 0.1 * pos.y;
        
        // Effet de courbure ULTRA
        float curvature = pow(pos.y / 2.0, 1.5);
        pos.x += curvature * 0.2;
    }
    
    // ===== NORMALES ULTRA DÉTAILLÉES =====
    vec3 normal = normalize(vec3(
        pos.x * 0.2,
        1.0,
        pos.z * 0.2
    ));
    vNormal = normalMatrix * normal;
    vFakeNormal = normalMatrix * normal;
    
    // ===== GRADIENT LATÉRAL =====
    vSideGradient = abs(pos.x / uHalfWidth);
    vElevation = pos.y;
    
    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 1.0);
}
`;
