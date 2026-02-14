/**
 * PRIMORDIAL - Module injectable pour la scène DIAMANTS
 * 
 * Usage: Dans la console du navigateur sur la page DIAMANTS:
 *   import('/core/primordial-inject.js').then(m => m.inject())
 * 
 * Ou: Copy-paste tout le contenu dans la console
 */

(function() {
    // Trouver la scène DIAMANTS
    const findScene = () => {
        if (window.DIAMANTS && window.DIAMANTS.app && window.DIAMANTS.app.scene) {
            return window.DIAMANTS.app.scene;
        }
        if (window.app && window.app.scene) {
            return window.app.scene;
        }
        // Chercher dans les objets globaux
        for (const key of Object.keys(window)) {
            if (window[key] && window[key].scene && window[key].scene.isScene) {
                return window[key].scene;
            }
        }
        return null;
    };
    
    const THREE = window.THREE;
    if (!THREE) {
        console.error('THREE.js non disponible');
        return;
    }
    
    const scene = findScene();
    if (!scene) {
        console.error('Scène DIAMANTS non trouvée');
        return;
    }
    
    console.log('✓ Scène trouvée:', scene.name);
    
    // ═══════════════════════════════════════════════════════════════
    // CHAMP PRIMORDIAL - Espace des possibles
    // ═══════════════════════════════════════════════════════════════
    
    const FIELD_SIZE = 32;
    const field = new Float32Array(FIELD_SIZE * FIELD_SIZE * FIELD_SIZE);
    let separationStrength = 0;
    let separationTarget = 0;
    
    // Initialiser avec du bruit
    for (let i = 0; i < field.length; i++) {
        field[i] = Math.random() - 0.5;
    }
    
    function getField(x, y, z) {
        const ix = ((Math.floor(x) % FIELD_SIZE) + FIELD_SIZE) % FIELD_SIZE;
        const iy = ((Math.floor(y) % FIELD_SIZE) + FIELD_SIZE) % FIELD_SIZE;
        const iz = ((Math.floor(z) % FIELD_SIZE) + FIELD_SIZE) % FIELD_SIZE;
        return field[ix + iy * FIELD_SIZE + iz * FIELD_SIZE * FIELD_SIZE];
    }
    
    // ═══════════════════════════════════════════════════════════════
    // PARTICULES PRIMORDIALES
    // ═══════════════════════════════════════════════════════════════
    
    const N = 3000;
    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(N * 3);
    const colors = new Float32Array(N * 3);
    const fieldPositions = [];
    
    for (let i = 0; i < N; i++) {
        fieldPositions.push({
            x: Math.random() * FIELD_SIZE,
            y: Math.random() * FIELD_SIZE,
            z: Math.random() * FIELD_SIZE
        });
        
        // Position dans l'espace de la scène (centrée, réduite)
        positions[i * 3] = (fieldPositions[i].x - FIELD_SIZE/2) * 0.3;
        positions[i * 3 + 1] = (fieldPositions[i].y - FIELD_SIZE/2) * 0.3 + 5; // +5 pour élever au-dessus du sol
        positions[i * 3 + 2] = (fieldPositions[i].z - FIELD_SIZE/2) * 0.3;
        
        colors[i * 3] = 0.3;
        colors[i * 3 + 1] = 0.5;
        colors[i * 3 + 2] = 0.7;
    }
    
    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    
    const mat = new THREE.PointsMaterial({
        size: 0.08,
        vertexColors: true,
        transparent: true,
        opacity: 0.7,
        blending: THREE.AdditiveBlending,
        depthWrite: false
    });
    
    const primordialPoints = new THREE.Points(geo, mat);
    primordialPoints.name = 'PrimordialField';
    scene.add(primordialPoints);
    
    console.log('✓ Champ primordial ajouté à la scène');
    
    // ═══════════════════════════════════════════════════════════════
    // ANIMATION
    // ═══════════════════════════════════════════════════════════════
    
    function updatePrimordial() {
        // Évolution de la séparation
        if (Math.random() < 0.002) {
            separationTarget = Math.random();
        }
        separationStrength += (separationTarget - separationStrength) * 0.01;
        
        // Évolution du champ
        for (let i = 0; i < field.length; i++) {
            // Fluctuation
            field[i] += (Math.random() - 0.5) * 0.01;
            // Séparation (amplification des valeurs)
            field[i] += Math.sign(field[i]) * Math.abs(field[i]) * 0.02 * separationStrength;
            // Limite
            field[i] = Math.max(-1, Math.min(1, field[i]));
        }
        
        const pos = geo.attributes.position.array;
        const col = geo.attributes.color.array;
        
        for (let i = 0; i < N; i++) {
            const fp = fieldPositions[i];
            const v = getField(fp.x, fp.y, fp.z);
            
            // Mouvement selon le champ
            const gx = getField(fp.x + 1, fp.y, fp.z) - getField(fp.x - 1, fp.y, fp.z);
            const gy = getField(fp.x, fp.y + 1, fp.z) - getField(fp.x, fp.y - 1, fp.z);
            const gz = getField(fp.x, fp.y, fp.z + 1) - getField(fp.x, fp.y, fp.z - 1);
            
            fp.x += gx * v * separationStrength * 0.3 + (Math.random() - 0.5) * 0.2;
            fp.y += gy * v * separationStrength * 0.3 + (Math.random() - 0.5) * 0.2;
            fp.z += gz * v * separationStrength * 0.3 + (Math.random() - 0.5) * 0.2;
            
            // Wrap
            fp.x = ((fp.x % FIELD_SIZE) + FIELD_SIZE) % FIELD_SIZE;
            fp.y = ((fp.y % FIELD_SIZE) + FIELD_SIZE) % FIELD_SIZE;
            fp.z = ((fp.z % FIELD_SIZE) + FIELD_SIZE) % FIELD_SIZE;
            
            // Position visuelle
            pos[i * 3] = (fp.x - FIELD_SIZE/2) * 0.3;
            pos[i * 3 + 1] = (fp.y - FIELD_SIZE/2) * 0.3 + 5;
            pos[i * 3 + 2] = (fp.z - FIELD_SIZE/2) * 0.3;
            
            // Couleur selon dualité
            const sep = separationStrength;
            if (v > 0) {
                col[i * 3] = 0.3 + v * sep * 0.5;
                col[i * 3 + 1] = 0.4 + v * sep * 0.2;
                col[i * 3 + 2] = 0.5 - v * sep * 0.2;
            } else {
                col[i * 3] = 0.3 + v * sep * 0.2;
                col[i * 3 + 1] = 0.4 - v * sep * 0.2;
                col[i * 3 + 2] = 0.5 - v * sep * 0.5;
            }
        }
        
        geo.attributes.position.needsUpdate = true;
        geo.attributes.color.needsUpdate = true;
        
        requestAnimationFrame(updatePrimordial);
    }
    
    updatePrimordial();
    
    console.log(`
    ════════════════════════════════════════
    ⊙ PRIMORDIAL INJECTÉ
    
    ${N} particules dans le champ des possibles
    Dualité émergente et dissolution
    
    Pour retirer:
      scene.remove(scene.getObjectByName('PrimordialField'))
    ════════════════════════════════════════
    `);
    
    // Exposer pour contrôle externe
    window.PRIMORDIAL = {
        points: primordialPoints,
        field: field,
        getSeparation: () => separationStrength,
        setSeparation: (v) => { separationTarget = v; }
    };
})();
