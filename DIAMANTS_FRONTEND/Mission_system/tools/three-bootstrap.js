/**
 * THREE.js Bootstrap - Forcer le chargement et l'exposition globale
 * Résout les problèmes de modules ES6 et variables globales
 */

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Fonctions de logging silencieuses
const log = window.SILENT_MODE ? () => {} : (...args) => console.log(...args);
const warn = window.SILENT_MODE ? () => {} : (...args) => console.warn(...args);
const error = window.SILENT_MODE ? () => {} : (...args) => console.error(...args);

log('🚀 THREE.js Bootstrap - Chargement forcé...');

// Import THREE.js et exposition globale forcée
async function bootstrapTHREE() {
    try {
        log('📦 Chargement THREE.js depuis unpkg...');
        
        // Import dynamique de THREE.js
        const THREE = await import('https://unpkg.com/three@0.160.0/build/three.module.js');
        log('✅ THREE.js chargé:', THREE);
        
        // Exposition globale forcée
        window.THREE = THREE;
        
        // Vérifier les composants critiques
        const criticalComponents = [
            'Scene', 'PerspectiveCamera', 'WebGLRenderer', 'Vector3', 
            'BoxGeometry', 'MeshBasicMaterial', 'Mesh', 'Color', 'Group'
        ];
        
        const missing = criticalComponents.filter(comp => !THREE[comp]);
        if (missing.length > 0) {
            error('❌ Composants THREE.js manquants:', missing);
            throw new Error(`Composants critiques manquants: ${missing.join(', ')}`);
        }
        
        log('✅ Tous les composants THREE.js sont disponibles');
        
        // Test rapide WebGL avec THREE.js
        try {
            const testScene = new THREE.Scene();
            const testCamera = new THREE.PerspectiveCamera(75, 1, 0.1, 1000);
            const testRenderer = new THREE.WebGLRenderer();
            
            log('✅ Test THREE.js/WebGL réussi');
            
            // Silencer immédiatement tous les warnings THREE.js qui ralentissent
            const originalWarn = console.warn;
            const originalError = console.error;
            
            console.warn = function(...args) {
                const message = args.join(' ');
                // Silencer tous les warnings THREE.js connus qui ralentissent
                if (message.includes('Material: onBeforeRender() has been removed') ||
                    message.includes('Material: onBuild() has been removed') ||
                    message.includes('Multiple instances of Three.js being imported') ||
                    message.includes('roughness\' is not a property') ||
                    message.includes('THREE.Material:') ||
                    message.includes('WebGL')) {
                    return; // Silencer complètement
                }
                originalWarn.apply(console, args);
            };
            
            console.error = function(...args) {
                const message = args.join(' ');
                // Silencer les erreurs THREE.js non critiques
                if (message.includes('Material:') || 
                    message.includes('THREE.Material:') ||
                    message.includes('WebGL context')) {
                    return;
                }
                originalError.apply(console, args);
            };
            
            log('🔇 Warnings/Errors THREE.js silencés pour optimisation performances');
            
            // Nettoyage
            testRenderer.dispose();
        } catch (webglError) {
            error('❌ Test WebGL avec THREE.js échoué:', webglError);
            throw webglError;
        }
        
        // Import des addons critiques
        log('📦 Chargement ColladaLoader...');
        try {
            const ColladaLoaderModule = await import('https://unpkg.com/three@0.160.0/examples/jsm/loaders/ColladaLoader.js');
            window.ColladaLoader = ColladaLoaderModule.ColladaLoader;
            log('✅ ColladaLoader chargé et exposé');
        } catch (colladaError) {
            warn('⚠️ ColladaLoader optionnel non chargé:', colladaError.message);
            // Créer un ColladaLoader placeholder pour éviter les erreurs
            window.ColladaLoader = class {
                constructor() {
                    warn('⚠️ ColladaLoader placeholder - fichiers .dae non supportés');
                }
                load() {
                    warn('⚠️ Chargement .dae désactivé - ColladaLoader indisponible');
                }
            };
        }
        
        // Import OrbitControls
        log('📦 Chargement OrbitControls...');
        try {
            const OrbitControlsModule = await import('https://unpkg.com/three@0.160.0/examples/jsm/controls/OrbitControls.js');
            window.OrbitControls = OrbitControlsModule.OrbitControls;
            log('✅ OrbitControls chargé et exposé');
        } catch (orbitError) {
            error('❌ Erreur chargement OrbitControls:', orbitError);
        }
        
        // Signaler que THREE.js est prêt
        window.THREE_READY = true;
        log('🎯 THREE.js Bootstrap terminé - Système prêt !');
        
        // Déclencher événement personnalisé
        window.dispatchEvent(new CustomEvent('threeReady', { detail: { THREE } }));
        
        return THREE;
        
    } catch (error) {
        error('🚨 Erreur fatale bootstrap THREE.js:', error);
        
        // Affichage d'erreur utilisateur
        const errorDiv = document.createElement('div');
        errorDiv.style.cssText = `
            position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%);
            background: #1a1a2e; color: #FF6B6B; padding: 20px; border-radius: 10px;
            font-family: monospace; text-align: center; border: 2px solid #FF4757;
            z-index: 999999; max-width: 500px;
        `;
        errorDiv.innerHTML = `
            <h2>🛑 Erreur THREE.js</h2>
            <p style="margin: 10px 0;">Impossible de charger THREE.js:</p>
            <div style="background: #16213e; padding: 10px; border-radius: 5px; margin: 10px 0; font-size: 12px; text-align: left;">
                ${error.message}
            </div>
            <div style="margin-top: 15px;">
                <button onclick="location.reload()" style="margin: 5px; padding: 10px; background: #FF4757; color: white; border: none; border-radius: 5px; cursor: pointer;">
                    🔄 Recharger
                </button>
                <button onclick="this.parentElement.parentElement.style.display='none'" style="margin: 5px; padding: 10px; background: #575757; color: white; border: none; border-radius: 5px; cursor: pointer;">
                    ❌ Fermer
                </button>
            </div>
        `;
        document.body.appendChild(errorDiv);
        
        throw error;
    }
}

// Auto-démarrage
bootstrapTHREE();

// Fonction d'aide pour diagnostic
window.diagnosticTHREE = function() {
    log('🔍 === DIAGNOSTIC THREE.JS ===');
    log('window.THREE:', window.THREE);
    log('window.THREE_READY:', window.THREE_READY);
    log('window.ColladaLoader:', window.ColladaLoader);
    log('window.OrbitControls:', window.OrbitControls);
    
    if (window.THREE) {
        log('THREE.Scene:', window.THREE.Scene);
        log('THREE.WebGLRenderer:', window.THREE.WebGLRenderer);
    }
};
