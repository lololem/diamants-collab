// Vertex shader MEDIUM QUALITY - Animation équilibrée
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
    
    // ===== ANIMATION DE VENT ÉQUILIBRÉE (MEDIUM QUALITY) =====
    if (pos.y > 0.1) {
        float windTime = uTime * 0.0012 * uSpeed;
        float windStrength = 0.4;
        
        // Vent principal + légère turbulence
        pos.x += sin(windTime + pos.z * 0.5) * windStrength * pos.y;
        pos.z += sin(windTime * 0.7 + pos.x * 0.3) * windStrength * 0.3 * pos.y;
        
        // Variation individuelle simple
        float grassVar = rand(vec2(position.x, position.z));
        pos.x += sin(windTime * grassVar + grassVar) * 0.1 * pos.y;
    }
    
    // ===== NORMALES MOYENNES =====
    vec3 normal = normalize(vec3(
        pos.x * 0.1,
        1.0,
        pos.z * 0.1
    ));
    vNormal = normalMatrix * normal;
    vFakeNormal = normalMatrix * normal;
    
    // ===== GRADIENT LATÉRAL =====
    vSideGradient = abs(pos.x / uHalfWidth);
    vElevation = pos.y;
    
    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 1.0);
}
`;
