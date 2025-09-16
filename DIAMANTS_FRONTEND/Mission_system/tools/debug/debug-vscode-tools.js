/**
 * DIAMANTS V3 - Debug avec Extensions VS Code
 * ===========================================
 * Utilise Turbo Console Log + Console Ninja pour debug avancé
 */

// Configuration debug pour Console Ninja
const DEBUG_VSCODE = {
    consoleNinja: false, // ✅ Désactivé par défaut pour éviter la pollution
    turboLog: true,
    codeRunner: true,
    timestamp: () => new Date().toISOString().split('T')[1].split('.')[0]
};

// 🚀 Turbo Console Log - Logs automatiques pour fonctions critiques
function logFunction(functionName, args = {}, result = null) {
    const timestamp = DEBUG_VSCODE.timestamp();
    console.log(`🚀 [${timestamp}] ${functionName} →`, {
        arguments: args,
        result: result,
        stack: new Error().stack.split('\n').slice(2, 4).join('\n').trim()
    });
}

// 🎯 Console Ninja - Debug en temps réel pour DIAMANTS
window.DIAMANTS_DEBUG = {
    // Variables critiques à monitorer
    sceneStats: () => {
        if (window.scene) {
            return {
                children: window.scene.children.length,
                drones: window.drones?.length || 0,
                rendered: !!window.renderer,
                camera: !!window.camera
            };
        }
        return { status: 'Scene not initialized' };
    },

    // Monitor les drones en temps réel
    dronesStatus: () => {
        if (window.drones && window.drones.length > 0) {
            return window.drones.map((drone, index) => ({
                id: index,
                position: drone.position ? {
                    x: drone.position.x.toFixed(2),
                    y: drone.position.y.toFixed(2),
                    z: drone.position.z.toFixed(2)
                } : 'N/A',
                visible: drone.visible,
                mesh: !!drone.mesh,
                initialized: !!drone.initialized
            }));
        }
        return [];
    },

    // Test complet du système
    systemCheck: () => {
        logFunction('systemCheck');
        
        const checks = {
            three: !!window.THREE,
            scene: !!window.scene,
            camera: !!window.camera,
            renderer: !!window.renderer,
            drones: window.drones?.length || 0,
            logger: !!window.logger,
            debugConfig: !!window.DEBUG_CONFIG,
            consoleNinja: DEBUG_VSCODE.consoleNinja
        };

        console.table(checks);
        
        // Console Ninja va capturer automatiquement ces logs
        Object.entries(checks).forEach(([key, value]) => {
            if (!value) {
                console.error(`❌ [SYSTEM] ${key} manquant`);
            } else {
                console.log(`✅ [SYSTEM] ${key} OK`);
            }
        });

        return checks;
    },

    // Monitor performance en temps réel
    performance: () => {
        const perfData = {
            memory: performance.memory ? {
                used: Math.round(performance.memory.usedJSHeapSize / 1048576) + ' MB',
                total: Math.round(performance.memory.totalJSHeapSize / 1048576) + ' MB',
                limit: Math.round(performance.memory.jsHeapSizeLimit / 1048576) + ' MB'
            } : 'Non disponible',
            timing: {
                navigation: performance.timing.navigationStart,
                domLoaded: performance.timing.domContentLoadedEventEnd - performance.timing.navigationStart,
                loadComplete: performance.timing.loadEventEnd - performance.timing.navigationStart
            }
        };

        logFunction('performance', {}, perfData);
        return perfData;
    }
};

// 🔄 Auto-monitoring avec Console Ninja (désactivé par défaut)
function startAutoMonitoring() {
    if (window.DIAMANTS_DEBUG_MONITOR_INTERVAL) {
        clearInterval(window.DIAMANTS_DEBUG_MONITOR_INTERVAL);
    }
    
    window.DIAMANTS_DEBUG_MONITOR_INTERVAL = setInterval(() => {
        if (window.DEBUG_VSCODE?.consoleNinja && window.scene && window.drones) {
            const stats = window.DIAMANTS_DEBUG.sceneStats();
            const drones = window.DIAMANTS_DEBUG.dronesStatus();
            
            console.log('📊 [AUTO-MONITOR]', {
                scene: stats,
                activeDrones: drones.filter(d => d.visible).length,
                timestamp: DEBUG_VSCODE.timestamp()
            });
        }
    }, 5000); // Toutes les 5 secondes
}

function stopAutoMonitoring() {
    if (window.DIAMANTS_DEBUG_MONITOR_INTERVAL) {
        clearInterval(window.DIAMANTS_DEBUG_MONITOR_INTERVAL);
        window.DIAMANTS_DEBUG_MONITOR_INTERVAL = null;
    }
}

// 🎮 Code Runner - Tests rapides
window.runDebugTest = () => {
    console.log('🎮 [CODE-RUNNER] Démarrage test debug complet...');
    
    logFunction('runDebugTest');
    
    const systemCheck = window.DIAMANTS_DEBUG.systemCheck();
    const performance = window.DIAMANTS_DEBUG.performance();
    
    console.log('🎯 [RÉSULTAT] Test debug terminé', {
        system: systemCheck,
        performance: performance,
        consoleNinja: 'Monitoring actif',
        turboLog: 'Logs automatiques activés'
    });
    
    return { systemCheck, performance };
};

// Auto-initialisation
document.addEventListener('DOMContentLoaded', () => {
    console.log('🎯 [INIT] Debug VS Code Tools chargés');
    
    // ✅ CORRECTION: Attendre le signal d'initialisation au lieu d'un timeout arbitraire
    if (window.DIAMANTS_INITIALIZED) {
        // Déjà initialisé, lancer immédiatement
        window.DIAMANTS_DEBUG.systemCheck();
    } else {
        // Écouter l'événement d'initialisation
        window.addEventListener('diamants:initialized', () => {
            console.log('📡 Événement diamants:initialized reçu, lancement systemCheck');
            setTimeout(() => {
                window.DIAMANTS_DEBUG.systemCheck();
            }, 500); // Petit délai pour s'assurer que tout est en place
        });
        
        // Fallback au cas où l'événement ne se déclenche pas
        setTimeout(() => {
            if (!window.DIAMANTS_INITIALIZED) {
                console.warn('⚠️ Timeout fallback - lancement systemCheck sans signal');
                window.DIAMANTS_DEBUG.systemCheck();
            }
        }, 10000); // 10 secondes de fallback
    }
});

console.log('🛠️ [VSCODE-DEBUG] Outils debug VS Code prêts - utilisez window.DIAMANTS_DEBUG.* ou window.runDebugTest()');
