/**
 * Gestionnaire d'essaim de drones AuthenticCrazyflie
 * Utilise les vrais meshes et la physique authentique
 * Basé sur le prototype SMA.html qui marche
 */

// Mode silencieux global - utilise les fonctions globales
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

class AuthenticDroneSwarm {
    constructor(scene) {
        this.scene = scene;
        this.drones = [];
        this.isRunning = false;
        this.animationId = null;
        this.startTime = Date.now();
        this.lastUpdate = 0;
        
        log('🚁 AuthenticDroneSwarm initialisé');
    }

    // Initialiser les drones authentiques
    initializeDrones() {
        log('🔄 Initialisation des drones AuthenticCrazyflie...');
        
        // Chercher des drones existants dans la scène d'abord
        this.findExistingDrones();
        
        // Si pas assez de drones, en créer
        if (this.drones.length < 4) {
            this.createAuthenticDrones();
        }
        
        log(`✅ ${this.drones.length} drones AuthenticCrazyflie initialisés`);
        return this.drones.length;
    }

    // Chercher les drones AuthenticCrazyflie existants dans la scène
    findExistingDrones() {
        if (!window.diamantsSystem || !window.diamantsSystem.drones) {
            log('📋 Aucun système de drones existant trouvé');
            return;
        }

        // Utiliser les drones du système DIAMANTS s'ils existent
        window.diamantsSystem.drones.forEach(drone => {
            if (drone && typeof drone.startMission === 'function') {
                this.drones.push(drone);
                log(`🔄 Drone existant trouvé: ${drone.id}`);
            }
        });
    }

    // Créer de nouveaux drones AuthenticCrazyflie
    createAuthenticDrones() {
        log('🏗️ Création de nouveaux drones AuthenticCrazyflie...');
        
        const dronesNeeded = 4 - this.drones.length;
        let dronesCreatedSync = 0;
        
        // Créer tous les drones de manière synchrone
        for (let i = 0; i < dronesNeeded; i++) {
            try {
                // Position en formation circulaire
                const angle = (i / dronesNeeded) * Math.PI * 2;
                const radius = 5;
                const x = Math.cos(angle) * radius;
                const z = Math.sin(angle) * radius;
                const y = 12; // Commencer au-dessus de la plateforme (hauteur 8.5m) + marge sécurité
                
                // Essayer d'utiliser AuthenticCrazyflie global d'abord
                if (window.AuthenticCrazyflie) {
                    const drone = new window.AuthenticCrazyflie(
                        `authentic_${i}`,
                        x, y, z,
                        'SCOUT',
                        this.scene
                    );
                    
                    this.drones.push(drone);
                    dronesCreatedSync++;
                    log(`✅ Drone AuthenticCrazyflie ${drone.id} créé à [${x.toFixed(1)}, ${y}, ${z.toFixed(1)}]`);
                } else {
                    log(`⚠️ window.AuthenticCrazyflie non disponible pour drone ${i} - import dynamique`);
                    // Fallback: import dynamique (mais risque d'être asynchrone)
                    import('../drones/authentic-crazyflie.js').then(module => {
                        const AuthenticCrazyflie = module.default || module.AuthenticCrazyflie;
                        
                        const drone = new AuthenticCrazyflie(
                            `authentic_async_${i}`,
                            x, y, z,
                            'SCOUT',
                            this.scene
                        );
                        
                        this.drones.push(drone);
                        log(`✅ Drone AuthenticCrazyflie ${drone.id} créé à [${x.toFixed(1)}, ${y}, ${z.toFixed(1)}] (async)`);
                        
                        // Exposer les drones mis à jour
                        if (window.drones !== this.drones && this.drones.length > 0) {
                            window.drones = this.drones;
                            log(`🔄 window.drones mis à jour: ${window.drones.length} drones`);
                        }
                        
                    }).catch(error => {
                        error('❌ Erreur import AuthenticCrazyflie:', error);
                    });
                }
                
            } catch (error) {
                error(`❌ Erreur création drone ${i}:`, error);
            }
        }
        
        log(`🔧 Créés de manière synchrone: ${dronesCreatedSync}/${dronesNeeded} drones`);
        return dronesCreatedSync;
    }

    // Démarrer la mission d'exploration
    startMission(altitude = 15.0, mode = 'hover') { // 15m par défaut et mode hover stationnaire
        log(`🚀 Démarrage mission essaim: altitude=${altitude}m, mode=${mode}`);
        
        if (this.drones.length === 0) {
            error('❌ Aucun drone disponible pour la mission');
            return false;
        }

        // Démarrer la mission pour chaque drone
        let missionCount = 0;
        this.drones.forEach((drone, index) => {
            try {
                if (drone && typeof drone.startMission === 'function') {
                    // Altitude légèrement différente pour chaque drone - 5m minimum + espacement
                    const droneAltitude = altitude + (index * 0.3);
                    drone.startMission(droneAltitude, mode);
                    missionCount++;
                }
            } catch (error) {
                error(`❌ Erreur démarrage mission drone ${drone?.id || index}:`, error);
            }
        });

        if (missionCount === 0) {
            error('❌ Aucune mission n\'a pu être démarrée');
            return false;
        }

        // Démarrer la boucle d'animation
        this.isRunning = true;
        this.startTime = Date.now();
        this.animate();
        
        log(`✅ Mission démarrée pour ${missionCount}/${this.drones.length} drones`);
        return true;
    }

    // Boucle d'animation principale
    animate() {
        if (!this.isRunning) return;

        // GUARD: Ne pas animer si IntegratedController gère les drones
        // IntegratedController appelle déjà drone.update() dans updateDronesBehaviors()
        // Avoir 2 boucles RAF qui appellent drone.update() = physique à 2× la vitesse + flickering
        if (window.DIAMANTS?.missionSystem?.integratedController) {
            log('⚠️ AuthenticDroneSwarm: IntegratedController active — stopping independent RAF loop');
            this.isRunning = false;
            return;
        }

        const currentTime = Date.now();
        const deltaTime = Math.min((currentTime - this.lastUpdate) / 1000, 0.1); // Max 100ms
        this.lastUpdate = currentTime;

        // Mettre à jour tous les drones
        this.drones.forEach(drone => {
            try {
                if (drone && typeof drone.update === 'function') {
                    drone.update(deltaTime, this.drones);
                }
            } catch (error) {
                error(`❌ Erreur update drone ${drone?.id}:`, error);
            }
        });

        this.animationId = requestAnimationFrame(() => this.animate());
    }

    // Arrêter la mission
    stop() {
        log('🛑 Arrêt de la mission essaim');
        this.isRunning = false;
        
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
            this.animationId = null;
        }

        // Arrêter tous les drones
        this.drones.forEach(drone => {
            try {
                if (drone && typeof drone.stop === 'function') {
                    drone.stop();
                }
            } catch (error) {
                error(`❌ Erreur arrêt drone ${drone?.id}:`, error);
            }
        });
    }

    // Remettre les drones en position initiale
    reset() {
        log('🔄 Reset de l\'essaim');
        this.stop();
        
        // Reset chaque drone
        this.drones.forEach((drone, index) => {
            try {
                if (drone) {
                    // Position initiale en cercle
                    const angle = (index / this.drones.length) * Math.PI * 2;
                    const radius = 5;
                    const x = Math.cos(angle) * radius;
                    const z = Math.sin(angle) * radius;
                    const y = 1;
                    
                    drone.position.set(x, y, z);
                    drone.targetPosition.set(x, y, z);
                    drone.velocity.set(0, 0, 0);
                    drone.state = 'IDLE';
                    
                    if (drone.mesh) {
                        drone.mesh.position.copy(drone.position);
                        drone.mesh.rotation.set(0, 0, 0);
                    }
                }
            } catch (error) {
                error(`❌ Erreur reset drone ${drone?.id}:`, error);
            }
        });
        
        log(`✅ ${this.drones.length} drones remis en position initiale`);
    }
}

// Rendre disponible globalement
window.AuthenticDroneSwarm = AuthenticDroneSwarm;

log('✅ AuthenticDroneSwarm classe chargée');
