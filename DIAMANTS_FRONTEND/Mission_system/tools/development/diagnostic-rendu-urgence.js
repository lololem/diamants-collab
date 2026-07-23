/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Diagnostic Urgence Rendu et Meshes
 * ================================================
 * Script pour résoudre les problèmes de scène vide et chargement meshes
 */

// Sécurité: Assurer la disponibilité des fonctions de logging
if (typeof log === 'undefined') {
    var log = window.log || ((...args) => console.log(...args));
}
if (typeof warn === 'undefined') {
    var warn = window.warn || ((...args) => console.warn(...args));
}
if (typeof error === 'undefined') {
    var error = window.error || ((...args) => console.error(...args));
}

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

function diagnosticRenduUrgence() {
    log('🚨 === DIAGNOSTIC URGENCE RENDU DIAMANTS ===');
    
    const diagnostics = {
        timestamp: new Date().toISOString(),
        problems: [],
        solutions: []
    };

    // 1. Vérifier THREE.js et scène
    log('🔍 1. VÉRIFICATION THREE.JS ET SCÈNE:');
    
    if (typeof THREE === 'undefined') {
        diagnostics.problems.push('THREE.js non chargé');
        error('❌ THREE.js non disponible');
    } else {
        log('✅ THREE.js version:', THREE.REVISION);
    }

    // Vérifier les variables globales critiques
    const criticalVars = {
        scene: window.scene,
        camera: window.camera, 
        renderer: window.renderer,
        drones: window.drones,
        engineInstance: window.engineInstance
    };

    log('🔍 Variables globales critiques:');
    Object.entries(criticalVars).forEach(([name, value]) => {
        const exists = !!value;
        log(`   ${exists ? '✅' : '❌'} ${name}:`, exists ? 'OK' : 'MANQUANT');
        if (!exists) {
            diagnostics.problems.push(`Variable globale ${name} manquante`);
        }
    });

    // 2. Vérifier le DOM et canvas
    log('🔍 2. VÉRIFICATION DOM ET CANVAS:');
    
    const canvasContainer = document.getElementById('canvas_container');
    if (!canvasContainer) {
        diagnostics.problems.push('canvas_container DOM manquant');
        error('❌ canvas_container non trouvé dans le DOM');
    } else {
        log('✅ canvas_container trouvé');
        log('   Dimensions:', {
            width: canvasContainer.offsetWidth,
            height: canvasContainer.offsetHeight,
            style: canvasContainer.style.cssText
        });
        
        if (canvasContainer.offsetWidth === 0 || canvasContainer.offsetHeight === 0) {
            diagnostics.problems.push('canvas_container a des dimensions nulles');
        }
    }

    // Vérifier s'il y a un canvas WebGL
    const canvases = document.querySelectorAll('canvas');
    log('🔍 Canvas trouvés:', canvases.length);
    canvases.forEach((canvas, index) => {
        log(`   Canvas ${index}:`, {
            width: canvas.width,
            height: canvas.height,
            style: canvas.style.cssText
        });
    });

    // 3. Vérifier le chargement des meshes
    log('🔍 3. VÉRIFICATION CHARGEMENT MESHES:');
    
    const meshPaths = [
        'assets/crazyflie/meshes/cf2_assembly.dae',
        'assets/crazyflie/meshes/cw_prop.dae', 
        'assets/crazyflie/meshes/ccw_prop.dae'
    ];

    // Test d'accès aux fichiers mesh
    meshPaths.forEach(async (path) => {
        try {
            const response = await fetch(path);
            if (response.ok) {
                log(`✅ Mesh accessible: ${path}`);
            } else {
                error(`❌ Mesh non accessible: ${path} (${response.status})`);
                diagnostics.problems.push(`Mesh non accessible: ${path}`);
            }
        } catch (error) {
            error(`❌ Erreur accès mesh: ${path}`, error.message);
            diagnostics.problems.push(`Erreur accès mesh: ${path} - ${error.message}`);
        }
    });

    // 4. Vérifier ColladaLoader
    log('🔍 4. VÉRIFICATION COLLADALOADER:');
    
    if (window.ColladaLoader) {
        log('✅ ColladaLoader disponible');
    } else {
        error('❌ ColladaLoader non disponible');
        diagnostics.problems.push('ColladaLoader non chargé');
    }

    // 5. Vérifier l'initialisation du moteur
    log('🔍 5. VÉRIFICATION INITIALISATION MOTEUR:');
    
    if (window.engineInstance) {
        log('✅ Engine instance exists');
        if (window.engineInstance.isInitialized) {
            log('✅ Engine initialisé');
        } else {
            warn('⚠️ Engine non initialisé');
            diagnostics.problems.push('Engine non initialisé');
        }
    } else {
        error('❌ Aucune instance engine');
        diagnostics.problems.push('Aucune instance engine');
    }

    // 6. Solutions proposées
    log('🔧 SOLUTIONS PROPOSÉES:');
    
    if (diagnostics.problems.length === 0) {
        log('✅ Aucun problème détecté - système OK');
    } else {
        diagnostics.problems.forEach((problem, index) => {
            log(`${index + 1}. ❌ ${problem}`);
            
            // Proposer solutions
            if (problem.includes('THREE.js')) {
                diagnostics.solutions.push('Recharger THREE.js via three-bootstrap.js');
            }
            if (problem.includes('canvas_container')) {
                diagnostics.solutions.push('Vérifier CSS et dimensions du container');
            }
            if (problem.includes('mesh')) {
                diagnostics.solutions.push('Vérifier chemins des fichiers DAE');
            }
            if (problem.includes('engine')) {
                diagnostics.solutions.push('Forcer initialisation du moteur');
            }
        });
        
        log('🔧 Solutions recommandées:');
        diagnostics.solutions.forEach((solution, index) => {
            log(`${index + 1}. 🛠️ ${solution}`);
        });
    }

    return diagnostics;
}

// Fonction pour forcer le rendu de test
function forcerRenduTest() {
    log('🎯 FORÇAGE RENDU DE TEST...');
    
    // GUARD: Ne jamais écraser le renderer principal — cela crée un competing RAF loop
    // et provoque du flickering en remplaçant le canvas par innerHTML=''
    if (window.renderer || (window.DIAMANTS?.missionSystem)) {
        log('✅ Renderer principal déjà actif — forcerRenduTest() annulé (anti-flickering)');
        return;
    }
    
    if (!window.THREE) {
        error('❌ THREE.js non disponible pour test');
        return;
    }

    try {
        // Créer une scène de test simple
        const testScene = new THREE.Scene();
        const testCamera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        const testRenderer = new THREE.WebGLRenderer({ antialias: true });
        
        testRenderer.setSize(800, 600);
        testRenderer.setClearColor(0x87CEEB); // Bleu ciel pour voir le rendu
        
        // Ajouter un cube coloré de test
        const geometry = new THREE.BoxGeometry();
        const material = new THREE.MeshBasicMaterial({ color: 0xff0000 });
        const testCube = new THREE.Mesh(geometry, material);
        testScene.add(testCube);
        
        testCamera.position.z = 5;
        
        // Injecter dans le DOM
        const container = document.getElementById('canvas_container');
        if (container) {
            container.innerHTML = ''; // Vider
            container.appendChild(testRenderer.domElement);
            
            // Animer le cube
            function animateTest() {
                requestAnimationFrame(animateTest);
                testCube.rotation.x += 0.01;
                testCube.rotation.y += 0.01;
                testRenderer.render(testScene, testCamera);
            }
            animateTest();
            
            log('✅ Rendu de test forcé - cube rouge visible');
        } else {
            error('❌ Container canvas_container non trouvé');
        }
        
    } catch (error) {
        error('❌ Erreur lors du forçage rendu:', error);
    }
}

// Fonction pour créer une UI de logs visible
function creerUILogsVisible() {
    log('📋 CRÉATION UI LOGS VISIBLE...');
    
    // Supprimer ancienne UI de logs si existe
    const oldLogUI = document.getElementById('debug-logs-ui');
    if (oldLogUI) {
        oldLogUI.remove();
    }
    
    // Créer panneau de logs
    const logPanel = document.createElement('div');
    logPanel.id = 'debug-logs-ui';
    logPanel.style.cssText = `
        position: fixed;
        top: 10px;
        right: 10px;
        width: 400px;
        height: 300px;
        background: rgba(0, 0, 0, 0.9);
        color: #00ff00;
        font-family: 'Courier New', monospace;
        font-size: 12px;
        padding: 10px;
        border: 2px solid #00ff00;
        border-radius: 5px;
        overflow-y: auto;
        z-index: 5000;
        display: block;
    `;
    
    // Header du panneau
    const header = document.createElement('div');
    header.style.cssText = `
        color: #ffff00;
        font-weight: bold;
        margin-bottom: 10px;
        border-bottom: 1px solid #00ff00;
        padding-bottom: 5px;
    `;
    header.textContent = '🎯 DIAMANTS - Debug Logs';
    
    // Zone de contenu des logs
    const logContent = document.createElement('div');
    logContent.id = 'debug-logs-content';
    
    // Bouton masquer/afficher
    const toggleBtn = document.createElement('button');
    toggleBtn.textContent = '📋';
    toggleBtn.style.cssText = `
        position: absolute;
        top: 5px;
        right: 5px;
        background: #00ff00;
        color: black;
        border: none;
        width: 25px;
        height: 25px;
        cursor: pointer;
    `;
    
    let isVisible = true;
    toggleBtn.onclick = () => {
        isVisible = !isVisible;
        logContent.style.display = isVisible ? 'block' : 'none';
        header.style.display = isVisible ? 'block' : 'none';
    };
    
    logPanel.appendChild(toggleBtn);
    logPanel.appendChild(header);
    logPanel.appendChild(logContent);
    document.body.appendChild(logPanel);
    
    // Intercepter console.log pour afficher dans l'UI
    const originalLog = console.log;
    console.log = function(...args) {
        originalLog.apply(console, args);
        
        const logLine = document.createElement('div');
        logLine.style.marginBottom = '2px';
        logLine.textContent = `[${new Date().toLocaleTimeString()}] ${args.join(' ')}`;
        
        logContent.appendChild(logLine);
        logContent.scrollTop = logContent.scrollHeight;
        
        // Limiter à 50 lignes
        while (logContent.children.length > 50) {
            logContent.removeChild(logContent.firstChild);
        }
    };
    
    log('✅ UI de logs visible créée');
}

// Export des fonctions globalement
window.diagnosticRendu = diagnosticRenduUrgence;
window.forcerRenduTest = forcerRenduTest;
window.creerUILogs = creerUILogsVisible;

// Auto-exécution du diagnostic avec attente de l'initialisation pour éviter des faux négatifs
(function setupDeferredDiagnostic() {
    let ran = false;
    const MAX_WAIT_MS = 12000; // temps max d'attente avant exécution forcée
    const CHECK_INTERVAL_MS = 300;

    function readyConditions() {
        try {
            const engineOk = !!(window.engineInstance && window.engineInstance.isInitialized);
            const threeOk = typeof THREE !== 'undefined';
            const loaderOk = !threeOk || !!THREE.ColladaLoader; // si THREE pas prêt, ne bloque pas; sinon vérifier loader
            return engineOk && threeOk && loaderOk;
        } catch (_) { return false; }
    }

    function runOnce(reason) {
        if (ran) return;
        ran = true;
        log(`🎯 Auto-diagnostic rendu démarré (${reason})...`);
        try { diagnosticRenduUrgence(); } catch (e) { error('Diagnostic error:', e); }
    }

    // 1) Écouter les événements d'init émis par l’app
    try {
        const tryRunIfReady = (tag) => {
            // Ne lance que si les conditions sont vraiment remplies
            if (readyConditions()) runOnce(`event:${tag}`);
        };
        window.addEventListener('diamants:initialized', () => {
            // Cet event peut arriver tôt; on ne lance pas si non prêt
            setTimeout(() => tryRunIfReady('diamants:initialized'), 800);
        }, { once: true });
        window.addEventListener('diamants:engine-ready', () => {
            setTimeout(() => tryRunIfReady('diamants:engine-ready'), 200);
        }, { once: true });
    } catch (_) { /* noop */ }

    // 2) Polling de repli jusqu'à ce que le moteur soit prêt
    const start = Date.now();
    const timer = setInterval(() => {
        if (readyConditions()) {
            clearInterval(timer);
            runOnce('poll:ready');
        } else if (Date.now() - start > MAX_WAIT_MS) {
            clearInterval(timer);
            runOnce('timeout');
        }
    }, CHECK_INTERVAL_MS);

    // 3) Sécurité ultime: exécution forcée si rien ne s'est passé
    setTimeout(() => runOnce('fallback-timeout'), MAX_WAIT_MS + 1000);
})();

log('🚨 Script diagnostic rendu chargé - utilisez:');
log('   window.diagnosticRendu() - Diagnostic complet');
log('   window.forcerRenduTest() - Test rendu forcé');
log('   window.creerUILogs() - UI logs visible');
