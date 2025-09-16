export default `
// Camera-aware uniforms to limit brown blades to near distance
uniform vec3 uCameraPos;
uniform float uBrownNear; // distance at which browns are fully visible
uniform float uBrownFar;  // distance beyond which browns disappear

varying float vElevation;
varying float vSideGradient; 
varying vec3 vNormal;
varying vec3 vFakeNormal;
varying vec3 vPosition;

// --- Small hashing & value-noise helpers (cheap) ---
float hash12(vec2 p) {
    return fract(sin(dot(p, vec2(127.1,311.7))) * 43758.5453123);
}
float valueNoise(vec2 x) {
    vec2 i = floor(x);
    vec2 f = fract(x);
    float a = hash12(i);
    float b = hash12(i + vec2(1.0, 0.0));
    float c = hash12(i + vec2(0.0, 1.0));
    float d = hash12(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(a, b, u.x) + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
}

void main() {
    // ===== PALETTE HERBE OPTIMISÉE PERFORMANCE =====
    // PALETTE SIMPLIFIÉE POUR PERFORMANCE ZONE VISIBLE
    // Seulement 2 verts principaux + 2 bruns pour réduire les calculs
    vec3 springGreenTip = vec3(0.260, 0.420, 0.180);     // Vert prairie principal
    vec3 springGreenBase = vec3(0.180, 0.280, 0.130);    // Base verte principale
    
    vec3 forestGreenTip = vec3(0.200, 0.320, 0.150);     // Vert forêt secondaire
    vec3 forestGreenBase = vec3(0.150, 0.220, 0.110);    // Base forêt
    
    // SEULEMENT 2 BRUNS pour performance optimale
    vec3 brownGrassTip = vec3(0.300, 0.240, 0.160);      // Brun herbe principal
    vec3 brownGrassBase = vec3(0.200, 0.160, 0.110);     // Base brun principale
    
    vec3 goldenBrownTip = vec3(0.340, 0.280, 0.180);     // Brun doré secondaire
    vec3 goldenBrownBase = vec3(0.220, 0.180, 0.120);    // Base dorée
    
    vec3 fogColor = vec3(0.902, 0.921, 0.937);
    
    // ===== DISTRIBUTION SIMPLIFIÉE POUR PERFORMANCE ZONE VISIBLE =====
    float random = fract(sin(dot(vPosition.xz, vec2(12.9898, 78.233))) * 43758.5453);
    
    vec3 finalTipColor, finalBaseColor;
    
    // DISTRIBUTION OPTIMISÉE : très peu de bruns ET uniquement proches de la caméra
    // Calcule un facteur de visibilité des bruns selon la distance cam -> brin
    float camDist = length(vPosition.xz - uCameraPos.xz);
    float brownVisibility = clamp(smoothstep(uBrownFar, uBrownNear, camDist), 0.0, 1.0);

    float pPrairie = 0.70;
    float pForest = pPrairie + 0.20;
    float pBrown1 = pForest + 0.07 * brownVisibility; // 0..0.07
    float pBrown2 = pBrown1 + 0.03 * brownVisibility; // 0..0.10 total

    if (random < pPrairie) {
        finalTipColor = springGreenTip;
        finalBaseColor = springGreenBase;
    } else if (random < pForest) {
        finalTipColor = forestGreenTip;
        finalBaseColor = forestGreenBase;
    } else if (random < pBrown1) {
        finalTipColor = brownGrassTip;
        finalBaseColor = brownGrassBase;
    } else if (random < pBrown2) {
        finalTipColor = goldenBrownTip;
        finalBaseColor = goldenBrownBase;
    } else {
        // Loin de la caméra: retomber sur du vert
        finalTipColor = springGreenTip;
        finalBaseColor = springGreenBase;
    }
    
    // MÉLANGE TRÈS RÉDUIT - Priorité aux verts pour performance !
    float macro = valueNoise(vPosition.xz * 0.04);
    if (random > 0.85) {  // SEULEMENT 15% de mélange
        float mixWeight = 0.2 * smoothstep(0.4, 0.6, macro); // Poids réduit
        if (random < 0.92) {
            // Favoriser les mélanges verts
            finalTipColor = mix(finalTipColor, forestGreenTip, mixWeight);
            finalBaseColor = mix(finalBaseColor, forestGreenBase, mixWeight);
        } else if (brownVisibility > 0.0) {
            // Très peu de mélange brun, seulement proche caméra
            finalTipColor = mix(finalTipColor, brownGrassTip, mixWeight * 0.5 * brownVisibility);
            finalBaseColor = mix(finalBaseColor, brownGrassBase, mixWeight * 0.5 * brownVisibility);
        }
    }
    // Supprimer les mélanges coûteux et références obsolètes (random3, autumn*)
    
    // ===== GRADIENT DE HAUTEUR =====
    float elevationNorm = clamp(vElevation * 0.5, 0.0, 1.0);
    vec3 grassColor = mix(finalBaseColor, finalTipColor, pow(elevationNorm, 0.7));
    
    // ===== SOL PLUS NUANCÉ (bruns/ocres/galets) =====
    float soilRandom = fract(sin(dot(vPosition.xz * 2.1, vec2(45.164, 23.431))) * 43758.5453);
    float soilMacro  = valueNoise(vPosition.xz * 0.12);
    float soilMicro  = valueNoise(vPosition.xz * 2.5);

    // Palette sol assombrie pour harmoniser avec l'herbe
    vec3 brownSoil   = vec3(0.295, 0.205, 0.137);  // brun riche plus foncé
    vec3 claySoil    = vec3(0.382, 0.282, 0.180);  // ocre/argile assombrie
    vec3 darkSoil    = vec3(0.176, 0.137, 0.098);  // plus sombre
    vec3 reddishSoil = vec3(0.382, 0.222, 0.140);  // rougeâtre plus foncé
    vec3 greySoil    = vec3(0.290, 0.270, 0.260);  // limon/gris assombri

    // Mélange sol harmonieux avec l'herbe éclaircie
    vec3 soilAB = mix(brownSoil, claySoil, smoothstep(0.2, 0.8, soilMacro));
    vec3 soilCD = mix(darkSoil,  reddishSoil, smoothstep(0.3, 0.7, 1.0 - soilMacro));
    vec3 soilColor = mix(soilAB, soilCD, 0.35 + 0.35 * soilMacro);
    // Variation vers gris modérée
    soilColor = mix(soilColor, greySoil, 0.15 * smoothstep(0.6, 1.0, soilMacro));

    // Micro éclats façon "galets clairsemés" plus prononcés
    float pebble = step(0.982, hash12(vPosition.xz * 15.0));
    soilColor = mix(soilColor, soilColor * 1.35 + vec3(0.08), pebble * 0.7);

    // Application du sol au pied du brin avec transition plus marquée
    if (elevationNorm < 0.25) {
        float soilBlend = smoothstep(0.01, 0.25, elevationNorm);
        // Renforcer davantage le brun au tout début de la tige
        vec3 footTint = mix(soilColor, soilColor * vec3(1.08, 1.04, 0.96), soilMicro * 0.3);
        grassColor = mix(footTint, grassColor, soilBlend);
    }

    // Teinte sécheresse plus prononcée pour réalisme
    float dry = smoothstep(0.50, 0.80, valueNoise(vPosition.xz * 0.08 + 10.0));
    vec3 dryTint = vec3(1.08, 1.02, 0.85);  // Plus contrasté
    finalTipColor = mix(finalTipColor, finalTipColor * dryTint, 0.35 * dry);
    finalBaseColor = mix(finalBaseColor, finalBaseColor * dryTint, 0.35 * dry);
    
    // ===== ÉCLAIRAGE RÉALISTE (comme le repo) =====
    vec3 lightDirection = normalize(vec3(0.5, 1.0, 0.3));
    float diffuse = max(0.0, dot(vFakeNormal, lightDirection));
    
    // Éclairage ambiant plus sombre pour plus de réalisme
    float ambient = 0.4;
    float lighting = ambient + diffuse * 0.6;
    
    // ===== GRADIENT LATÉRAL (edges plus sombres) =====
    float edgeFactor = smoothstep(0.0, 0.1, vSideGradient) * smoothstep(1.0, 0.9, vSideGradient);
    grassColor *= mix(0.6, 1.0, edgeFactor); // Edges plus sombres
    
    // ===== VARIATIONS SUBTILES pour éviter le scintillement =====
    float micro = valueNoise(vPosition.xz * 1.3);
    vec3 colorVariation = mix(vec3(0.96), vec3(1.04), micro);
    grassColor *= colorVariation;
    
    // ===== APPLICATION DE L'ÉCLAIRAGE =====
    vec3 finalColor = grassColor * lighting;
    
    // ===== BROUILLARD/FOG SIMPLE =====
    float fogDistance = length(vPosition);
    float fogFactor = exp(-fogDistance * 0.003);
    finalColor = mix(fogColor, finalColor, fogFactor);
    
    // ===== SATURATION ET CONTRASTE RENFORCÉS =====
    // Augmentation MAJEURE de la saturation pour des verts très prononcés
    float saturation = 1.6;
    float luminance = dot(finalColor, vec3(0.299, 0.587, 0.114));
    finalColor = mix(vec3(luminance), finalColor, saturation);
    
    // Augmentation du contraste pour des verts plus vibrants
    finalColor = pow(finalColor, vec3(1.2));
    
    gl_FragColor = vec4(finalColor, 1.0);
}
`;
