/**
 * Gestionnaire d'initialisation DIAMANTS
 * Coordonne le démarrage de tous les systèmes
 */

// Mode silencieux pour les logs - utilisation de window pour éviter les conflits
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Utiliser les fonctions globales définies dans index.html

log('🚀 === INITIALISATION DIAMANTS V3 ===');

class DiamantsInitializer {
    constructor() {
        this.initialized = {
            three: false,
            webgl: false,
            engine: false,
            drones: false,
            environment: false,
            ui: false
        };
        
        this.initPromise = null;
        this.startTime = performance.now();
    }
    
    /**
     * Initialisation complète du système
     */
    async initialize() {
        if (this.initPromise) {
            return this.initPromise;
        }
        
        this.initPromise = this._performInitialization();
        return this.initPromise;
    }
    
    async _performInitialization() {
        log('⏳ Démarrage initialisation système...');
        
        try {
            // Étape 1: THREE.js
            await this._waitForTHREE();
            this.initialized.three = true;
            log('✅ THREE.js initialisé');
            
            // Étape 2: WebGL
            await this._validateWebGL();
            this.initialized.webgl = true;
            log('✅ WebGL validé');
            
            // Étape 3: Attendre que l'engine soit créé
            await this._waitForEngine();
            this.initialized.engine = true;
            log('✅ Engine initialisé');
            
            // Étape 4: Drones
            await this._waitForDrones();
            this.initialized.drones = true;
            log('✅ Drones initialisés');
            
            // Étape 5: Environment
            this.initialized.environment = true;
            log('✅ Environnement initialisé');
            
            // Étape 6: UI
            this.initialized.ui = true;
            log('✅ Interface utilisateur initialisée');
            
            const totalTime = performance.now() - this.startTime;
            log(`🎯 DIAMANTS V3 initialisé en ${totalTime.toFixed(0)}ms`);
            
            // Signaler que tout est prêt
            window.dispatchEvent(new CustomEvent('diamantsReady', {
                detail: { 
                    initTime: totalTime,
                    systems: this.initialized
                }
            }));
            
            return true;
            
        } catch (error) {
            error('🚨 Erreur initialisation DIAMANTS V3:', error);
            throw error;
        }
    }
    
    async _waitForTHREE() {
        return new Promise((resolve, reject) => {
            if (window.THREE && window.THREE_READY) {
                resolve();
                return;
            }
            
            const checkTHREE = () => {
                if (window.THREE && window.THREE_READY) {
                    resolve();
                } else {
                    setTimeout(checkTHREE, 100);
                }
            };
            
            // Écouter l'événement threeReady aussi
            window.addEventListener('threeReady', resolve, { once: true });
            
            setTimeout(() => reject(new Error('THREE.js timeout')), 10000);
            checkTHREE();
        });
    }
    
    async _validateWebGL() {
        const canvas = document.createElement('canvas');
        const gl = canvas.getContext('webgl2') || 
                  canvas.getContext('webgl') || 
                  canvas.getContext('experimental-webgl');
        
        if (!gl) {
            throw new Error('WebGL indisponible');
        }
        
        return true;
    }
    
    async _waitForEngine() {
        return new Promise((resolve) => {
            const checkEngine = () => {
                if (window.engineInstance || 
                    (window.DIAMANTS && window.DIAMANTS.missionSystem)) {
                    resolve();
                } else {
                    setTimeout(checkEngine, 200);
                }
            };
            checkEngine();
        });
    }
    
    async _waitForDrones() {
        return new Promise((resolve) => {
            const checkDrones = () => {
                const hasDrones = (window.DIAMANTS && 
                                 window.DIAMANTS.missionSystem && 
                                 window.DIAMANTS.missionSystem.drones && 
                                 window.DIAMANTS.missionSystem.drones.length > 0) ||
                                (window.drones && window.drones.length > 0);
                
                if (hasDrones) {
                    resolve();
                } else {
                    setTimeout(checkDrones, 300);
                }
            };
            setTimeout(resolve, 5000); // Timeout après 5s
            checkDrones();
        });
    }
    
    /**
     * Vérifier l'état d'initialisation
     */
    getStatus() {
        const total = Object.keys(this.initialized).length;
        const ready = Object.values(this.initialized).filter(Boolean).length;
        
        return {
            progress: ready / total,
            ready: ready === total,
            systems: { ...this.initialized },
            elapsed: performance.now() - this.startTime
        };
    }
}

// Instance globale
window.diamantsInitializer = new DiamantsInitializer();

// Auto-démarrage
window.diamantsInitializer.initialize().catch(error => {
    error('💥 Échec initialisation DIAMANTS V3:', error);
});

// Fonction d'aide
window.checkDiamantsStatus = () => {
    const status = window.diamantsInitializer.getStatus();
    log('📊 État DIAMANTS V3:', status);
    return status;
};

log('🎯 Initializer DIAMANTS V3 créé - utilisez checkDiamantsStatus() pour vérifier');
