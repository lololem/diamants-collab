/**
 * EZ-Tree Inspired Skybox for DIAMANTS Provençal Environment
 * Based on: https://github.com/dgreenheck/ez-tree/blob/main/src/app/shaders/skybox.frag
 * Adapted for Mediterranean atmosphere with warm sunset lighting
 */

export const ProvencalSkyboxShaders = {
    vertex: `
        varying vec3 vPosition;
        varying vec3 vWorldPosition;
        
        void main() {
            vPosition = position;
            vec4 worldPosition = modelMatrix * vec4(position, 1.0);
            vWorldPosition = worldPosition.xyz;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
    `,
    
    fragment: `
        precision mediump float;
        
        varying vec3 vPosition;
        varying vec3 vWorldPosition;
        
        uniform float uSunAzimuth; // Sun azimuth angle (in degrees)
        uniform float uSunElevation; // Sun elevation angle (in degrees)
        uniform vec3 uSunColor;
        uniform vec3 uSkyColorLow;
        uniform vec3 uSkyColorHigh;
        uniform float uSunSize;
        uniform float uTime;
        uniform float uHazeIntensity;
        uniform vec3 uHazeColor;
        
        void main() {
            // Convert angles from degrees to radians
            float azimuth = radians(uSunAzimuth);
            float elevation = radians(uSunElevation);
            
            // Calculate the sun direction vector based on azimuth and elevation
            vec3 sunDirection = normalize(vec3(
                cos(elevation) * sin(azimuth),
                sin(elevation),
                cos(elevation) * cos(azimuth)
            ));
            
            // Normalize the fragment position
            vec3 direction = normalize(vPosition);
            
            // Enhanced gradient for Provençal sky with warm colors
            float t = direction.y * 0.5 + 0.5;
            
            // Add atmospheric perspective based on distance from horizon
            float horizonFade = smoothstep(-0.1, 0.4, direction.y);
            
            // Mediterranean haze effect (shimmer in hot air)
            float haze = uHazeIntensity * (1.0 - horizonFade) * 
                         (0.5 + 0.5 * sin(uTime * 0.5 + vWorldPosition.x * 0.01));
            
            // Blend sky colors with haze
            vec3 skyColor = mix(uSkyColorLow, uSkyColorHigh, t);
            skyColor = mix(skyColor, uHazeColor, haze);
            
            // Enhanced sun appearance with atmospheric scattering
            float sunDot = dot(direction, sunDirection);
            float sunIntensity = pow(max(sunDot, 0.0), 1000.0 / uSunSize);
            
            // Add sun corona effect for Mediterranean warmth
            float corona = pow(max(sunDot, 0.0), 50.0 / uSunSize) * 0.3;
            
            // Atmospheric scattering around sun (warm glow)
            float scattering = pow(max(sunDot, 0.0), 10.0) * 0.2;
            
            vec3 sunColor = uSunColor * (sunIntensity + corona + scattering);
            
            // Combine sun and sky color
            vec3 color = skyColor + sunColor;
            
            // Add subtle color temperature variation
            color += vec3(0.02, 0.01, -0.01) * sin(uTime * 0.1);
            
            gl_FragColor = vec4(color, 1.0);
        }
    `
};

export const ProvencalSkyboxDefaults = {
    // Authentic Provençal sunset colors
    sunAzimuth: 220.0,      // Late afternoon sun position
    sunElevation: 25.0,     // Golden hour elevation
    sunColor: [1.0, 0.9, 0.7],        // Warm golden sun
    skyColorLow: [1.0, 0.8, 0.6],     // Warm horizon (peachy)
    skyColorHigh: [0.4, 0.6, 0.9],    // Deep Mediterranean blue
    sunSize: 1.2,
    hazeIntensity: 0.15,
    hazeColor: [0.9, 0.85, 0.8]       // Warm atmospheric haze
};
