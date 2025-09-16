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
