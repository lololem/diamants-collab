/**
 * Système de statut et remise en état stationnaire des drones
 * Permet de remettre tous les drones dans leur état d'avant décollage
 */

class DroneStateManager {
    constructor() {
        this.initialStates = new Map(); // Stockage des états initiaux
        this.initialized = false;
        this.init();
    }
    
    init() {
        console.log('🔧 Initialisation du gestionnaire d\'état des drones...');
        
        // Attendre que le système soit prêt
        const checkSystem = () => {
            if (window.dronePanelController?.controller?.drones) {
                this.captureDroneInitialStates();
                this.setupStateManagement();
                this.initialized = true;
                console.log('✅ Gestionnaire d\'état des drones initialisé');
            } else {
                setTimeout(checkSystem, 1000);
            }
        };
        
        checkSystem();
    }
    
    /**
     * Capture l'état initial de tous les drones
     */
    captureDroneInitialStates() {
        const drones = window.dronePanelController.controller.drones;
        
        console.log(`📸 Capture des états initiaux de ${drones.length} drones...`);
        
        drones.forEach((drone, index) => {
            const initialState = {
                id: drone.id || `drone_${index}`,
                position: {
                    x: drone.mesh?.position?.x || 0,
                    y: drone.mesh?.position?.y || 0,
                    z: drone.mesh?.position?.z || 0
                },
                rotation: {
                    x: drone.mesh?.rotation?.x || 0,
                    y: drone.mesh?.rotation?.y || 0,
                    z: drone.mesh?.rotation?.z || 0
                },
                state: drone.state || 'IDLE',
                velocity: {
                    x: drone.velocity?.x || 0,
                    y: drone.velocity?.y || 0,
                    z: drone.velocity?.z || 0
                },
                propellerPower: 0,
                isActive: false
            };
            
            this.initialStates.set(drone.id || index, initialState);
            console.log(`📋 État initial drone ${initialState.id}:`, initialState);
        });
    }
    
    /**
     * Remet tous les drones dans leur état stationnaire d'avant décollage
     */
    async restoreToStationaryState() {
        console.log('🏠 === REMISE EN ÉTAT STATIONNAIRE ===');
        
        if (!this.initialized) {
            console.warn('⚠️ Gestionnaire non initialisé');
            return;
        }
        
        const drones = window.dronePanelController.controller.drones;
        
        // Étape 1: Atterrissage en douceur
        console.log('🛬 Étape 1: Atterrissage en douceur...');
        await this.gentleLanding(drones);
        
        // Étape 2: Remise en position initiale
        console.log('📍 Étape 2: Remise en position initiale...');
        await this.restoreInitialPositions(drones);
        
        // Étape 3: Arrêt complet des systèmes
        console.log('⏹️ Étape 3: Arrêt des systèmes...');
        this.stopAllSystems(drones);
        
        console.log('✅ Tous les drones sont maintenant en état stationnaire');
    }
    
    /**
     * Atterrissage en douceur
     */
    async gentleLanding(drones) {
        const landingPromises = drones.map((drone, index) => {
            return new Promise((resolve) => {
                if (!drone.mesh) {
                    resolve();
                    return;
                }
                
                const startY = drone.mesh.position.y;
                const initialState = this.initialStates.get(drone.id || index);
                const targetY = initialState?.position.y || 0;
                
                if (startY <= targetY + 0.1) {
                    resolve(); // Déjà au sol
                    return;
                }
                
                console.log(`🛬 Atterrissage drone ${drone.id || index} depuis ${startY.toFixed(2)}m`);
                
                const startTime = Date.now();
                const duration = 4000; // 4 secondes pour un atterrissage doux
                
                const animateLanding = () => {
                    const elapsed = Date.now() - startTime;
                    const progress = Math.min(elapsed / duration, 1);
                    
                    // Courbe d'atterrissage douce (ease-out)
                    const easeProgress = 1 - Math.pow(1 - progress, 2);
                    
                    drone.mesh.position.y = startY + (targetY - startY) * easeProgress;
                    
                    // Réduction progressive de la puissance des hélices
                    if (drone.propellerAnimation) {
                        drone.propellerAnimation.power = Math.max(0.1, 1 - progress);
                    }
                    
                    // État de transition
                    drone.state = progress < 1 ? 'LANDING' : 'IDLE';
                    
                    if (progress < 1) {
                        requestAnimationFrame(animateLanding);
                    } else {
                        console.log(`✅ Drone ${drone.id || index} atterri`);
                        resolve();
                    }
                };
                
                animateLanding();
            });
        });
        
        await Promise.all(landingPromises);
        console.log('✅ Atterrissage terminé pour tous les drones');
    }
    
    /**
     * Remet tous les drones en position initiale
     */
    async restoreInitialPositions(drones) {
        const positionPromises = drones.map((drone, index) => {
            return new Promise((resolve) => {
                if (!drone.mesh) {
                    resolve();
                    return;
                }
                
                const initialState = this.initialStates.get(drone.id || index);
                if (!initialState) {
                    resolve();
                    return;
                }
                
                const startPos = {
                    x: drone.mesh.position.x,
                    y: drone.mesh.position.y,
                    z: drone.mesh.position.z
                };
                
                const targetPos = initialState.position;
                
                // Si déjà en position, pas besoin de bouger
                const distance = Math.sqrt(
                    Math.pow(startPos.x - targetPos.x, 2) +
                    Math.pow(startPos.z - targetPos.z, 2)
                );
                
                if (distance < 0.5) {
                    resolve();
                    return;
                }
                
                console.log(`📍 Repositionnement drone ${drone.id || index}`);
                
                const startTime = Date.now();
                const duration = 2000; // 2 secondes
                
                const animatePosition = () => {
                    const elapsed = Date.now() - startTime;
                    const progress = Math.min(elapsed / duration, 1);
                    
                    // Animation fluide
                    const easeProgress = progress < 0.5 
                        ? 2 * progress * progress
                        : 1 - Math.pow(-2 * progress + 2, 2) / 2;
                    
                    drone.mesh.position.x = startPos.x + (targetPos.x - startPos.x) * easeProgress;
                    drone.mesh.position.y = startPos.y + (targetPos.y - startPos.y) * easeProgress;
                    drone.mesh.position.z = startPos.z + (targetPos.z - startPos.z) * easeProgress;
                    
                    // Orientation
                    drone.mesh.rotation.x = initialState.rotation.x * easeProgress;
                    drone.mesh.rotation.y = initialState.rotation.y * easeProgress;
                    drone.mesh.rotation.z = initialState.rotation.z * easeProgress;
                    
                    if (progress < 1) {
                        requestAnimationFrame(animatePosition);
                    } else {
                        console.log(`✅ Drone ${drone.id || index} repositionné`);
                        resolve();
                    }
                };
                
                animatePosition();
            });
        });
        
        await Promise.all(positionPromises);
        console.log('✅ Repositionnement terminé pour tous les drones');
    }
    
    /**
     * Arrêt complet de tous les systèmes
     */
    stopAllSystems(drones) {
        drones.forEach((drone, index) => {
            // Réinitialiser l'état
            drone.state = 'IDLE';
            
            // Arrêter toutes les animations
            if (drone.propellerAnimationId) {
                cancelAnimationFrame(drone.propellerAnimationId);
                drone.propellerAnimationId = null;
            }
            
            // Arrêter les hélices
            if (drone.propellerAnimation) {
                drone.propellerAnimation.power = 0;
            }
            if (drone.stopRotors) {
                drone.stopRotors();
            }
            
            // Remettre à zéro les vitesses
            if (drone.velocity) {
                drone.velocity.x = 0;
                drone.velocity.y = 0;
                drone.velocity.z = 0;
            }
            
            // Remettre l'état initial complet
            const initialState = this.initialStates.get(drone.id || index);
            if (initialState) {
                Object.assign(drone, {
                    state: initialState.state,
                    isActive: initialState.isActive
                });
            }
            
            console.log(`⏹️ Systèmes arrêtés pour drone ${drone.id || index}`);
        });
    }
    
    /**
     * Affichage de l'état actuel de tous les drones
     */
    displayDroneStatus() {
        console.log('📊 === ÉTAT ACTUEL DES DRONES ===');
        
        if (!window.dronePanelController?.controller?.drones) {
            console.log('❌ Aucun drone détecté');
            return;
        }
        
        const drones = window.dronePanelController.controller.drones;
        
        drones.forEach((drone, index) => {
            const id = drone.id || `drone_${index}`;
            const pos = drone.mesh?.position;
            const state = drone.state || 'UNKNOWN';
            
            console.log(`🤖 ${id}:`);
            console.log(`   📍 Position: (${pos?.x.toFixed(2) || '?'}, ${pos?.y.toFixed(2) || '?'}, ${pos?.z.toFixed(2) || '?'})`);
            console.log(`   📊 État: ${state}`);
            console.log(`   🚁 Actif: ${drone.isActive ? 'Oui' : 'Non'}`);
            
            if (this.initialStates.has(drone.id || index)) {
                const initial = this.initialStates.get(drone.id || index);
                const distance = pos ? Math.sqrt(
                    Math.pow(pos.x - initial.position.x, 2) +
                    Math.pow(pos.y - initial.position.y, 2) +
                    Math.pow(pos.z - initial.position.z, 2)
                ).toFixed(2) : '?';
                console.log(`   📏 Distance de l'origine: ${distance}m`);
            }
        });
    }
    
    /**
     * Configuration des fonctions globales
     */
    setupStateManagement() {
        // Fonction globale pour revenir à l'état stationnaire
        window.RESTORE_TO_STATIONARY = () => {
            return this.restoreToStationaryState();
        };
        
        // Fonction globale pour afficher le statut
        window.SHOW_DRONE_STATUS = () => {
            return this.displayDroneStatus();
        };
        
        // Fonction globale de test rapide
        window.QUICK_LAND_AND_RESET = async () => {
            console.log('🚀 === TEST RAPIDE: ATTERRISSAGE + REMISE À ZÉRO ===');
            await this.restoreToStationaryState();
            this.displayDroneStatus();
        };
    }
}

// Initialiser le gestionnaire d'état
window.droneStateManager = new DroneStateManager();

console.log('🏠 === GESTIONNAIRE D\'ÉTAT STATIONNAIRE ACTIVÉ ===');
console.log('💡 Commandes disponibles:');
console.log('   - RESTORE_TO_STATIONARY() : Remettre tous les drones en état stationnaire');
console.log('   - SHOW_DRONE_STATUS() : Afficher l\'état de tous les drones');
console.log('   - QUICK_LAND_AND_RESET() : Test rapide atterrissage + remise à zéro');
