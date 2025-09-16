/**
 * DIAMANTS V3 - Fix Force Initialisation
 * =====================================
 * Script pour forcer l'initialisation et corriger les problèmes de rendu
 */

function forcerInitialisationDiamants() {
    console.log('🚀 FORÇAGE INITIALISATION DIAMANTS V3...');
    
    // 1. Ne pas créer l'UI de logs automatiquement (éviter pollution visuelle)

    // 2. Vérifier que THREE.js est chargé
    if (typeof THREE === 'undefined') {
        console.error('❌ THREE.js non chargé - impossible de continuer');
        return false;
    }

    // 3. Vérifier le container
    const container = document.getElementById('canvas_container');
    if (!container) {
        console.error('❌ Container canvas_container non trouvé');
        return false;
    }

    // 4. Nettoyer le container
    container.innerHTML = '';
    console.log('🧹 Container nettoyé');

    // 5. Corriger les dimensions du container avec le nouveau gestionnaire
    if (window.canvasResizeManager) {
        window.canvasResizeManager.ensureContainerDimensions(container);
    } else {
        // Fallback si le gestionnaire n'est pas chargé
        if (container.offsetWidth === 0 || container.offsetHeight === 0) {
            console.warn('⚠️ Container a des dimensions nulles - correction...');
            container.style.width = '100%';
            container.style.height = '600px';
            container.style.minHeight = '600px';
            container.style.display = 'block';
            container.style.position = 'relative';
        }
    }

    console.log('📐 Dimensions container:', {
        width: container.offsetWidth,
        height: container.offsetHeight
    });

    // 6. Initialiser manuellement si nécessaire
    if (!window.diamantsApp && typeof initializeDiamants === 'function') {
        console.log('🎯 Initialisation manuelle de DIAMANTS...');
        try {
            const app = initializeDiamants(container);
            window.diamantsApp = app;
            console.log('✅ DIAMANTS initialisé manuellement');
            
            // Appliquer le fix de resize si disponible
            if (window.canvasResizeManager && app.renderer && app.camera) {
                window.canvasResizeManager.fixCanvasResize(app.renderer, app.camera, container);
                console.log('✅ Fix resize appliqué');
            }
        } catch (error) {
            console.error('❌ Erreur initialisation manuelle:', error);
        }
    }

    // 7. Si toujours pas d'app, créer une scène de test
    if (!window.diamantsApp) {
        console.log('🧪 Création scène de test...');
        creerSceneTest(container);
    }

    // 8. Masquer l'écran de chargement
    const loadingScreen = document.getElementById('loading_screen');
    if (loadingScreen && loadingScreen.style.display !== 'none') {
        loadingScreen.style.display = 'none';
        console.log('👻 Écran de chargement masqué');
    }

    return true;
}

function creerSceneTest(container) {
    console.log('🧪 Création scène de test simple...');
    
    try {
        // Scène de test
        const scene = new THREE.Scene();
        const camera = new THREE.PerspectiveCamera(75, container.offsetWidth / container.offsetHeight, 0.1, 1000);
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        
        renderer.setSize(container.offsetWidth, container.offsetHeight);
    renderer.setClearColor(0x000000, 0); // transparent pour thème Mission Control
        
        // Ajouter éclairage
        const ambientLight = new THREE.AmbientLight(0x404040, 0.6);
        scene.add(ambientLight);
        
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(50, 50, 50);
        scene.add(directionalLight);
        
        // Ajouter sol de référence
        const groundGeometry = new THREE.PlaneGeometry(100, 100);
        const groundMaterial = new THREE.MeshLambertMaterial({ color: 0x8B4513 });
        const ground = new THREE.Mesh(groundGeometry, groundMaterial);
        ground.rotation.x = -Math.PI / 2;
        scene.add(ground);
        
        // Ajouter quelques cubes colorés comme drones de test
        const colors = [0xff0000, 0x00ff00, 0x0000ff, 0xffff00, 0xff00ff, 0x00ffff];
        const testDrones = [];
        
        for (let i = 0; i < 6; i++) {
            const geometry = new THREE.BoxGeometry(1, 1, 1);
            const material = new THREE.MeshLambertMaterial({ color: colors[i] });
            const cube = new THREE.Mesh(geometry, material);
            
            // Position en cercle
            const angle = (i / 6) * Math.PI * 2;
            cube.position.x = Math.cos(angle) * 10;
            cube.position.z = Math.sin(angle) * 10;
            cube.position.y = 2 + Math.sin(Date.now() * 0.001 + i) * 2;
            
            scene.add(cube);
            testDrones.push(cube);
        }
        
        camera.position.set(25, 15, 25);
        camera.lookAt(0, 0, 0);
        
        // Ajouter au container
        container.appendChild(renderer.domElement);
        
        // Appliquer le fix de resize
        if (window.canvasResizeManager) {
            window.canvasResizeManager.fixCanvasResize(renderer, camera, container);
            console.log('✅ Fix resize appliqué à la scène test');
        }
        
        // Animation
        function animate() {
            requestAnimationFrame(animate);
            
            // Animer les cubes de test
            testDrones.forEach((cube, index) => {
                cube.rotation.y += 0.01;
                const angle = (index / testDrones.length) * Math.PI * 2 + Date.now() * 0.001;
                cube.position.y = 2 + Math.sin(angle) * 2;
            });
            
            renderer.render(scene, camera);
        }
        animate();
        
        // Sauvegarder globalement
        window.testScene = scene;
        window.testCamera = camera;
        window.testRenderer = renderer;
        
        console.log('✅ Scène de test créée et animée');
        console.log('🎯 6 cubes colorés volent en cercle sur fond bleu/vert');
        console.log('🖥️ Resize automatique activé');
        
    } catch (error) {
        console.error('❌ Erreur création scène test:', error);
    }
}

function fixerChargementMeshes() {
    console.log('🔧 CORRECTION CHARGEMENT MESHES...');
    
    // Vérifier ColladaLoader
    if (!THREE.ColladaLoader) {
        console.error('❌ ColladaLoader non disponible');
        return false;
    }
    
    const loader = new THREE.ColladaLoader();
    const meshPaths = [
        './assets/crazyflie/meshes/cf2_assembly.dae',
        './assets/crazyflie/meshes/cw_prop.dae',
        './assets/crazyflie/meshes/ccw_prop.dae'
    ];
    
    console.log('🔍 Test de chargement des meshes...');
    
    meshPaths.forEach((path, index) => {
        loader.load(
            path,
            (collada) => {
                console.log(`✅ Mesh ${index + 1}/3 chargé: ${path}`);
                if (window.testScene) {
                    const mesh = collada.scene.clone();
                    mesh.position.set(index * 5 - 5, 10, 0);
                    mesh.scale.setScalar(0.1);
                    window.testScene.add(mesh);
                    console.log(`✅ Mesh ${index + 1} ajouté à la scène test`);
                }
            },
            (progress) => {
                console.log(`📊 Chargement ${path}: ${(progress.loaded / progress.total * 100).toFixed(0)}%`);
            },
            (error) => {
                console.error(`❌ Erreur chargement ${path}:`, error);
            }
        );
    });
    
    return true;
}

function creerPanneauControleRapide() {
    console.log('🎮 Création panneau de contrôle rapide...');
    
    const panel = document.createElement('div');
    panel.id = 'controle-rapide';
    panel.style.cssText = `
        position: fixed;
        bottom: 10px;
        left: 10px;
        background: rgba(0, 0, 0, 0.8);
        color: white;
        padding: 15px;
        border-radius: 5px;
        font-family: Arial, sans-serif;
        z-index: 9999;
        display: none;
    `;
    
    panel.innerHTML = `
        <h3 style="margin: 0 0 10px 0; color: #00ff00;">🎯 Contrôles Debug</h3>
        <button onclick="window.forcerInitialisationDiamants()" style="margin: 2px; padding: 5px;">🚀 Forcer Init</button><br>
        <button onclick="window.forcerRenduTest()" style="margin: 2px; padding: 5px;">🧪 Test Rendu</button><br>
        <button onclick="window.fixerChargementMeshes()" style="margin: 2px; padding: 5px;">🔧 Test Meshes</button><br>
        <button onclick="window.diagnosticRendu()" style="margin: 2px; padding: 5px;">🔍 Diagnostic</button><br>
        <button onclick="window.fixResize()" style="margin: 2px; padding: 5px;">🖥️ Fix Resize</button><br>
        <button onclick="this.parentElement.style.display='none'" style="margin: 2px; padding: 5px;">❌ Fermer</button>
    `;
    
    // Ne pas attacher par défaut pour ne pas polluer la vue
    // document.body.appendChild(panel);
    // console.log('✅ Panneau de contrôle créé');
    return panel;
}

// Fonction pour forcer le fix de resize
function fixResize() {
    console.log('🖥️ FORÇAGE FIX RESIZE...');
    
    const container = document.getElementById('canvas_container');
    if (!container) {
        console.error('❌ Container non trouvé');
        return;
    }
    
    if (window.canvasResizeManager) {
        window.canvasResizeManager.ensureContainerDimensions(container);
        
        // Appliquer aux instances existantes
        if (window.testRenderer && window.testCamera) {
            window.canvasResizeManager.fixCanvasResize(window.testRenderer, window.testCamera, container);
            console.log('✅ Fix resize appliqué à la scène test');
        }
        
        if (window.diamantsApp && window.diamantsApp.renderer && window.diamantsApp.camera) {
            window.canvasResizeManager.fixCanvasResize(window.diamantsApp.renderer, window.diamantsApp.camera, container);
            console.log('✅ Fix resize appliqué à l\'app DIAMANTS');
        }
        
        console.log('✅ Fix resize terminé');
    } else {
        console.error('❌ CanvasResizeManager non disponible');
    }
}

// Export global
window.forcerInitialisationDiamants = forcerInitialisationDiamants;
window.creerSceneTest = creerSceneTest;
window.fixerChargementMeshes = fixerChargementMeshes;
window.creerPanneauControleRapide = creerPanneauControleRapide;
window.fixResize = fixResize;

// Auto-exécution après chargement
document.addEventListener('DOMContentLoaded', () => {
    setTimeout(() => {
        console.log('🎯 Auto-fix démarré (mode propre)...');
        // Diagnostics silencieux seulement
        if (typeof window.diagnosticRendu === 'function') {
            try { window.diagnosticRendu(); } catch (e) {}
        }
        // Forçage possible via ?autoinit=1
        const enableAutoInit = (new URLSearchParams(location.search)).get('autoinit') === '1';
        if (enableAutoInit) {
            setTimeout(() => {
                if (!window.diamantsApp && !window.testScene) {
                    console.log('⚠️ Auto-init (flag) ...');
                    forcerInitialisationDiamants();
                }
            }, 3000);
        }
    }, 500);
});

console.log('🛠️ Fix initialisation chargé (non intrusif)');
