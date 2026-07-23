/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * THREE.js Bootstrap - Forcer le chargement et l'exposition globale
 * Résout les problèmes de modules ES6 et variables globales
 */

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Fonctions de logging silencieuses
const _log = window.SILENT_MODE ? () => {} : (...args) => console.log(...args);
const _warn = window.SILENT_MODE ? () => {} : (...args) => console.warn(...args);
const _error = window.SILENT_MODE ? () => {} : (...args) => console.error(...args);

_log('🚀 THREE.js Bootstrap - Chargement forcé...');

// Import THREE.js et exposition globale forcée
async function bootstrapTHREE() {
    try {
        _log('📦 Chargement THREE.js depuis unpkg...');
        
        // Import dynamique de THREE.js
        const THREE = await import('https://unpkg.com/three@0.167.1/build/three.module.js');
        _log('✅ THREE.js chargé:', THREE);
        
        // Exposition globale forcée
        window.THREE = THREE;
        
        // Vérifier les composants critiques
        const criticalComponents = [
            'Scene', 'PerspectiveCamera', 'WebGLRenderer', 'Vector3', 
            'BoxGeometry', 'MeshBasicMaterial', 'Mesh', 'Color', 'Group'
        ];
        
        const missing = criticalComponents.filter(comp => !THREE[comp]);
        if (missing.length > 0) {
            _error('❌ Composants THREE.js manquants:', missing);
            throw new Error(`Composants critiques manquants: ${missing.join(', ')}`);
        }
        
        _log('✅ Tous les composants THREE.js sont disponibles');
        
        // Skip WebGL test — creating and losing a test context wastes GPU
        // resources and can cause "WebGL context was lost" warnings.
        // The real renderer will validate WebGL when it's created.

        // Silencer les warnings THREE.js qui ralentissent
        const originalWarn = console.warn;
        const originalError = console.error;
        
        console.warn = function(...args) {
            const message = args.join(' ');
            // Only suppress known THREE.js deprecation warnings, NOT critical WebGL errors
            if (message.includes('Material: onBeforeRender() has been removed') ||
                message.includes('Material: onBuild() has been removed') ||
                message.includes('Multiple instances of Three.js being imported') ||
                message.includes('roughness\' is not a property') ||
                message.includes('THREE.Material:')) {
                return;
            }
            originalWarn.apply(console, args);
        };
        
        console.error = function(...args) {
            const message = args.join(' ');
            // Only suppress known THREE.js material deprecation errors
            if (message.includes('THREE.Material:') ||
                message.includes('Material: onBeforeRender') ||
                message.includes('Material: onBuild')) {
                return;
            }
            originalError.apply(console, args);
        };
        
        _log('🔇 Warnings/Errors THREE.js silencés pour optimisation performances');
        
        // Import des addons critiques
        _log('📦 Chargement ColladaLoader...');
        try {
            const ColladaLoaderModule = await import('https://unpkg.com/three@0.167.1/examples/jsm/loaders/ColladaLoader.js');
            window.ColladaLoader = ColladaLoaderModule.ColladaLoader;
            _log('✅ ColladaLoader chargé et exposé');
        } catch (colladaError) {
            _warn('⚠️ ColladaLoader optionnel non chargé:', colladaError.message);
            // Créer un ColladaLoader placeholder pour éviter les erreurs
            window.ColladaLoader = class {
                constructor() {
                    _warn('⚠️ ColladaLoader placeholder - fichiers .dae non supportés');
                }
                load() {
                    _warn('⚠️ Chargement .dae désactivé - ColladaLoader indisponible');
                }
            };
        }
        
        // Import OrbitControls
        _log('📦 Chargement OrbitControls...');
        try {
            const OrbitControlsModule = await import('https://unpkg.com/three@0.167.1/examples/jsm/controls/OrbitControls.js');
            window.OrbitControls = OrbitControlsModule.OrbitControls;
            _log('✅ OrbitControls chargé et exposé');
        } catch (orbitError) {
            _error('❌ Erreur chargement OrbitControls:', orbitError);
        }
        
        // Signaler que THREE.js est prêt
        window.THREE_READY = true;
        _log('🎯 THREE.js Bootstrap terminé - Système prêt !');
        
        // Déclencher événement personnalisé
        window.dispatchEvent(new CustomEvent('threeReady', { detail: { THREE } }));
        
        return THREE;
        
    } catch (err) {
        _error('🚨 Erreur fatale bootstrap THREE.js:', err);
        
        // Affichage d'erreur utilisateur
        const errorDiv = document.createElement('div');
        errorDiv.style.cssText = `
            position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%);
            background: #1a1a2e; color: #FF6B6B; padding: 20px; border-radius: 10px;
            font-family: monospace; text-align: center; border: 2px solid #FF4757;
            z-index: 9000; max-width: 500px;
        `;
        errorDiv.innerHTML = `
            <h2>🛑 Erreur THREE.js</h2>
            <p style="margin: 10px 0;">Impossible de charger THREE.js:</p>
            <div style="background: #16213e; padding: 10px; border-radius: 5px; margin: 10px 0; font-size: 12px; text-align: left;">
                ${err.message}
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
        
        throw err;
    }
}

// Auto-démarrage
bootstrapTHREE();

// Fonction d'aide pour diagnostic
window.diagnosticTHREE = function() {
    _log('🔍 === DIAGNOSTIC THREE.JS ===');
    _log('window.THREE:', window.THREE);
    _log('window.THREE_READY:', window.THREE_READY);
    _log('window.ColladaLoader:', window.ColladaLoader);
    _log('window.OrbitControls:', window.OrbitControls);
    
    if (window.THREE) {
        _log('THREE.Scene:', window.THREE.Scene);
        _log('THREE.WebGLRenderer:', window.THREE.WebGLRenderer);
    }
};
