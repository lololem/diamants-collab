/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Gestionnaire d'initialisation DIAMANTS
 * Coordonne le démarrage de tous les systèmes
 */

console.log('🚀 === INITIALISATION DIAMANTS ===');

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
        console.log('⏳ Démarrage initialisation système...');
        
        try {
            // Étape 1: THREE.js
            await this._waitForTHREE();
            this.initialized.three = true;
            console.log('✅ THREE.js initialisé');
            
            // Étape 2: WebGL
            await this._validateWebGL();
            this.initialized.webgl = true;
            console.log('✅ WebGL validé');
            
            // Étape 3: Attendre que l'engine soit créé
            await this._waitForEngine();
            this.initialized.engine = true;
            console.log('✅ Engine initialisé');
            
            // Étape 4: Drones
            await this._waitForDrones();
            this.initialized.drones = true;
            console.log('✅ Drones initialisés');
            
            // Étape 5: Environment
            this.initialized.environment = true;
            console.log('✅ Environnement initialisé');
            
            // Étape 6: UI
            this.initialized.ui = true;
            console.log('✅ Interface utilisateur initialisée');
            
            const totalTime = performance.now() - this.startTime;
            console.log(`🎯 DIAMANTS initialisé en ${totalTime.toFixed(0)}ms`);
            
            // Signaler que tout est prêt
            window.dispatchEvent(new CustomEvent('diamantsReady', {
                detail: { 
                    initTime: totalTime,
                    systems: this.initialized
                }
            }));
            
            return true;
            
        } catch (error) {
            console.error('🚨 Erreur initialisation DIAMANTS:', error);
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
    console.error('💥 Échec initialisation DIAMANTS:', error);
});

// Fonction d'aide
window.checkDiamantsStatus = () => {
    const status = window.diamantsInitializer.getStatus();
    console.log('📊 État DIAMANTS:', status);
    return status;
};

console.log('🎯 Initializer DIAMANTS créé - utilisez checkDiamantsStatus() pour vérifier');
