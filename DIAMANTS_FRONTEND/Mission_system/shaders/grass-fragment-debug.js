/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
export default `
varying float vElevation;
varying float vSideGradient; 
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

void main() {
    // Simple green test shader
    vec3 green = vec3(0.2, 0.8, 0.3);
    
    // Height-based gradient
    float elevationNorm = clamp(vElevation * 0.5, 0.0, 1.0);
    vec3 tipColor = vec3(0.6, 0.9, 0.4);
    vec3 baseColor = vec3(0.1, 0.4, 0.1);
    
    vec3 grassColor = mix(baseColor, tipColor, pow(elevationNorm, 0.7));
    
    gl_FragColor = vec4(grassColor, 1.0);
}
`;
