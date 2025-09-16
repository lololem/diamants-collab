export default `uniform float uTime;
uniform vec3 uBaseColor;
uniform vec3 uTipColor;
uniform vec3 uFogColor;

varying float vElevation;
varying float vSideGradient;
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

vec3 directionalLight(vec3 lightColor, float lightIntensity, vec3 normal, vec3 lightPosition, vec3 viewDirection, float specularPower) {
    vec3 lightDirection = normalize(lightPosition);
    vec3 lightReflection = reflect(-lightDirection, normal);

    //Shading
    float shading = dot(normal, lightDirection);
    shading = max(0.0, shading);

    //Specular
    float specular = -dot(lightReflection, viewDirection);
    specular = max(0.0, specular);
    specular = pow(specular, specularPower) * shading;

    return lightColor * lightIntensity * (shading + specular);
}

vec3 ambientLight(vec3 lightColor, float lightIntensity) {
    return lightColor * lightIntensity;
}

void main() {
    float gradient = smoothstep(0.2, 1.0, vElevation);
    float sideGradient = smoothstep(0.2, 1.0, vSideGradient);
    vec3 finalColor = mix(uBaseColor, uTipColor, gradient);

    vec3 light = vec3(0.0);
    vec3 normal = gl_FrontFacing ? vFakeNormal : -vFakeNormal;

    vec3 viewDirection = normalize(cameraPosition - vPosition);

    light += ambientLight(vec3(1.0, 1.0, 1.0), 0.5);
    light += directionalLight(
        uFogColor,              // light color
        1.0,                    // intensity
        normal,                 // normal
        vec3(2.0, 2.0, 2.0),    // light position
        viewDirection,          // view direction
        100.0                   // specular power
    );

    finalColor *= light;

    float dist = length(cameraPosition - vPosition);
    float fogFactor = smoothstep(15.0, 50.0, dist);
    finalColor = mix(finalColor, uFogColor, fogFactor);

    gl_FragColor = vec4(finalColor, 1.0);
}`;
