// Vertex shader LOW QUALITY - Animation minimal
export default `
uniform float uTime;
uniform float uSpeed;
uniform float uHalfWidth;

varying float vElevation;
varying float vSideGradient;
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

void main() {
    vec3 pos = position;
    vPosition = pos;
    
    // ===== ANIMATION DE VENT TRÈS SIMPLE (LOW QUALITY) =====
    if (pos.y > 0.1) {
        float windTime = uTime * 0.001;
        float windStrength = 0.2; // Vent faible
        
        // Mouvement simple uniquement en X, pas de turbulences
        pos.x += sin(windTime + pos.z * 0.3) * windStrength * pos.y;
    }
    
    // ===== NORMALES SIMPLIFIÉES =====
    vec3 normal = vec3(0.0, 1.0, 0.0); // Normale fixe vers le haut
    vNormal = normalMatrix * normal;
    vFakeNormal = normalMatrix * normal;
    
    // ===== GRADIENT LATÉRAL SIMPLE =====
    vSideGradient = abs(pos.x / uHalfWidth);
    vElevation = pos.y;
    
    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 1.0);
}
`;
