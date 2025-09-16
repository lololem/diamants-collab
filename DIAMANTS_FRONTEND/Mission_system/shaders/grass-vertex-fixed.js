export default `uniform float uTime;
uniform float uSpeed;
uniform float uHalfWidth;

varying float vElevation;
varying float vSideGradient;
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

// Random vector generator
float rand(vec2 co) {
    return fract(sin(dot(co, vec2(12.9898, 78.233))) * 43758.5453);
}

mat3 rotationY(float angle) {
    float c = cos(angle);
    float s = sin(angle);
    return mat3(
         c, 0.0, -s,
         0.0, 1.0, 0.0,
         s, 0.0,  c
    );
}

float bezier(float t, float p1) {
    float invT = 1.0 - t;
    return invT * invT * 0.0 + 2.0 * invT * t * p1 + t * t * 1.0;
}

//	============= Classic Perlin 2D Noise =============
vec4 permute(vec4 x) {
    return mod(((x * 34.0) + 1.0) * x, 289.0);
}

vec2 fade(vec2 t) {
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

float cnoise(vec2 P){
    vec4 Pi = floor(P.xyxy) + vec4(0.0, 0.0, 1.0, 1.0);
    vec4 Pf = fract(P.xyxy) - vec4(0.0, 0.0, 1.0, 1.0);
    Pi = mod(Pi, 289.0);
    vec4 ix = Pi.xzxz;
    vec4 iy = Pi.yyww;
    vec4 fx = Pf.xzxz;
    vec4 fy = Pf.yyww;
    vec4 i = permute(permute(ix) + iy);
    vec4 gx = 2.0 * fract(i * 0.0243902439) - 1.0;
    vec4 gy = abs(gx) - 0.5;
    vec4 tx = floor(gx + 0.5);
    gx = gx - tx;
    vec2 g00 = vec2(gx.x, gy.x);
    vec2 g10 = vec2(gx.y, gy.y);
    vec2 g01 = vec2(gx.z, gy.z);
    vec2 g11 = vec2(gx.w, gy.w);
    vec4 norm = 1.79284291400159 - 0.85373472095314 * vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11));
    g00 *= norm.x;
    g01 *= norm.y;
    g10 *= norm.z;
    g11 *= norm.w;
    float n00 = dot(g00, fade(Pf.xy));
    float n10 = dot(g10, fade(Pf.zy));
    float n01 = dot(g01, fade(Pf.xw));
    float n11 = dot(g11, fade(Pf.zw));
    vec2 fade_xy = fade(Pf.xy);
    vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
    float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
    return 2.3 * n_xy;
}

// Fonction de déformation avec vent synchronisé
vec4 deform(vec4 localPos) {
    vec3 instPos = instanceMatrix[3].xyz;
    
    // Normaliser la hauteur pour t
    float t = clamp(localPos.y / 1.0, 0.0, 1.0);
    
    // Vent provençal synchronisé avec l'environnement (vitesse similaire aux arbres)
    float windTime = uTime * 0.5; // Vitesse réduite pour vent plus naturel
    vec2 windPos = instPos.xz * 0.03; // Fréquence spatiale adaptée
    
    // Multi-fréquences pour vent réaliste
    float windNoise = cnoise(windPos + windTime) * 0.8 +
                      cnoise(windPos * 2.0 + windTime * 1.5) * 0.3 +
                      cnoise(windPos * 4.0 + windTime * 2.0) * 0.1;
    
    // Direction du vent principal (nord-ouest provençal)
    vec2 windDirection = normalize(vec2(-0.7, -0.3));
    
    // Intensité basée sur la hauteur (plus fort en haut)
    float windIntensity = t * t * 0.6; // Progression quadratique plus naturelle
    
    // Application du vent avec variations
    vec2 windOffset = windDirection * windNoise * windIntensity;
    
    // Courbure avec bézier pour mouvement naturel
    float bezierFactor = bezier(t, 0.7);
    vec2 finalOffset = windOffset * bezierFactor;
    
    // Billboarding simplifié vers la caméra
    vec3 worldPos = (modelMatrix * vec4(localPos.xyz, 1.0)).xyz;
    vec3 toCam = normalize(cameraPosition - worldPos);
    vec3 right = normalize(cross(vec3(0, 1, 0), toCam));
    
    // Application finale
    localPos.x += finalOffset.x + right.x * (localPos.x - instPos.x) * 0.1;
    localPos.z += finalOffset.y + right.z * (localPos.z - instPos.z) * 0.1;
    
    return localPos;
}

void main() {
    vec4 localPos = vec4(position, 1.0);
    
    // Déformation avec vent provençal
    localPos = deform(localPos);
    
    // Variables pour fragment shader
    vElevation = clamp(localPos.y / 1.0, 0.0, 1.0);
    vSideGradient = 1.0 - abs(localPos.x) / uHalfWidth;
    
    // Normale pour éclairage
    vNormal = normalize(normalMatrix * normal);
    vFakeNormal = normalize(normalMatrix * vec3(0.0, 1.0, 0.0));
    
    // Position mondiale
    vec4 worldPos = modelMatrix * instanceMatrix * localPos;
    vPosition = worldPos.xyz;
    
    gl_Position = projectionMatrix * modelViewMatrix * instanceMatrix * localPos;
}`;
