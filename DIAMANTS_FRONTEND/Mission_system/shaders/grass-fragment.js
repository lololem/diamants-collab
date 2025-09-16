export default `
uniform float uTime;
uniform vec3 uBaseColor;
uniform vec3 uTipColor;
uniform vec3 uFogColor;

varying vec3 vPosition;
varying vec3 vWorldPosition;
varying float vDistanceToCamera;
varying float vElevation;
varying vec2 vUv;

void main() {
    // Dégradé vertical base->tip
    float h = clamp(vElevation, 0.0, 1.0);
    float heightGradient = smoothstep(0.0, 1.0, h);
    vec3 grassColor = mix(uBaseColor, uTipColor, heightGradient);

    // Variation subtile le long de la lame (pas de blanc)
    float sideVar = abs(vUv.x - 0.5);
    grassColor *= 0.95 + 0.05 * (1.0 - sideVar * 2.0);

    // Brouillard doux de chaleur provençal
    float fogFactor = smoothstep(60.0, 140.0, vDistanceToCamera);
    vec3 finalColor = mix(grassColor, uFogColor, fogFactor * 0.25);

    gl_FragColor = vec4(finalColor, 1.0);
}`;
