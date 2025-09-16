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
//	by Stefan Gustavson (https://github.com/stegu/webgl-noise)
vec4 permute(vec4 x) {
    return mod(((x * 34.0) + 1.0) * x, 289.0);
}

//To smooth the interpolations
vec2 fade(vec2 t) {
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

//Perlin Noise
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

    vec4 norm = 1.79284291400159 - 0.85373472095314 *
                vec4(dot(g00, g00), dot(g01, g01), dot(g10, g10), dot(g11, g11));
    g00 *= norm.x;
    g01 *= norm.y;
    g10 *= norm.z;
    g11 *= norm.w;

    float n00 = dot(g00, vec2(fx.x, fy.x));
    float n10 = dot(g10, vec2(fx.y, fy.y));
    float n01 = dot(g01, vec2(fx.z, fy.z));
    float n11 = dot(g11, vec2(fx.w, fy.w));

    vec2 fade_xy = fade(Pf.xy);
    vec2 n_x = mix(vec2(n00, n01), vec2(n10, n11), fade_xy.x);
    float n_xy = mix(n_x.x, n_x.y, fade_xy.y);
    return 2.3 * n_xy;
}


// ============= DEFORMATION FUNCTION =============
vec3 deform(vec3 pos) {
    vec3 localPosition = pos;

    // Bend direction in local space
    vec3 instanceZ = normalize(vec3(0.0, 0.0, instanceMatrix[0].z));

    float hash = rand(vec2(instanceMatrix[3].x, instanceMatrix[3].z));

    float bendStrength = mix(0.3, 0.6, hash);
    float bendStart = mix(0.0, 0.3, hash);
    float t = clamp((pos.y / 2.0 - bendStart) / (1.0 - bendStart), 0.0, 1.0);
    float topBendFactor = bezier(t, 0.1);

    // Gentle sway
    float gentleSway = sin(uTime * uSpeed * 0.8 + hash * 10.0) * 0.1;
    vec3 gentleDir = normalize(vec3(1.0, 0.0, 1.0));
    vec3 gentleOffset = gentleDir * gentleSway * t;

    // Strong wind
    vec3 worldPos = (instanceMatrix * vec4(pos, 1.0)).xyz;
    float wave = cnoise(worldPos.xz * 0.3 + vec2(uTime * uSpeed * 0.2, 0.0));
    float strongWind = wave * 0.65;
    vec3 strongDir = normalize(vec3(0.0, 0.0, 1.0));
    float y = pos.y;
    vec3 strongOffset = strongDir * strongWind * pow(y, 2.0);

    // Apply all displacements
    localPosition += instanceZ * bendStrength * topBendFactor;
    localPosition += gentleOffset;
    localPosition += strongOffset;
    localPosition.y -= 0.1 * strongOffset.z;

    // Billboard
    vec3 camPos = inverse(viewMatrix)[3].xyz;
    vec3 bladeWorldPos = instanceMatrix[3].xyz;
    vec2 toCamera2D = normalize(camPos.xz - bladeWorldPos.xz);
    float angleToCamera = atan(toCamera2D.y, toCamera2D.x);
    mat3 billboardRot = rotationY(angleToCamera);
    localPosition = billboardRot * localPosition;

    return localPosition;
}

// ============= MAIN FUNCTION =============
void main() {
    vec3 p = deform(position);

    vec3 offsetX = deform(position + vec3(0.01, 0.0, 0.0));
    vec3 offsetY = deform(position + vec3(0.0, 0.01, 0.0));

    vec4 worldPosition = instanceMatrix * vec4(p, 1.0);
    vec4 viewPosition = viewMatrix * worldPosition;
    gl_Position = projectionMatrix * viewPosition;

    // ===== SIDE GRADIENT AND ELEVATION =====
    vElevation = position.y;
    vPosition = worldPosition.xyz;
    vSideGradient = 1.0 - ((position.x + uHalfWidth) / (2.0 * uHalfWidth));

    // ===== TRUE NORMAL FROM DEFORMATION =====
    vec3 normalWS = normalize(cross(offsetX - p, offsetY - p));
    vNormal = normalWS;

    vec3 invNormal = vNormal;
    invNormal.x *= -1.0;

    vFakeNormal = mix(vNormal, invNormal, vSideGradient);
}`;
