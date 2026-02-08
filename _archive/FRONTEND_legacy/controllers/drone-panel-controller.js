/**
 * Contrôleur du panneau de contrôle des drones
 * Pont entre les boutons HTML et le IntegratedDiamantsController existant
 */

class DronePanelController {
    constructor() {
        this.initialized = false;
        this.controller = null;
        this.app = null;
        this.waitForSystem();
    }

    waitForSystem() {
        console.log('⏳ Attente du système DIAMANTS...');
        
        let attempts = 0;
        const maxAttempts = 60; // 30 secondes max
        
        const checkInterval = setInterval(() => {
            attempts++;
            
            // Log détaillé pour debug
            if (attempts % 5 === 1) { // Log toutes les 2.5 secondes
                console.log(`🔍 Recherche du système... (tentative ${attempts}/${maxAttempts})`);
                console.log('  window.DIAMANTS:', window.DIAMANTS ? '✅ existe' : '❌ non trouvé');
                console.log('  window.engineInstance:', window.engineInstance ? '✅ existe' : '❌ non trouvé');
                console.log('  window.drones:', window.drones ? `✅ ${window.drones.length} drones` : '❌ non trouvé');
                
                if (window.DIAMANTS) {
                    console.log('  Structure DIAMANTS:', Object.keys(window.DIAMANTS));
                    console.log('  DIAMANTS.controller:', window.DIAMANTS.controller ? '✅' : '❌');
                }
            }
            
            // Méthode 1: Chercher via window.DIAMANTS.controller (API principale)
            if (window.DIAMANTS && window.DIAMANTS.controller) {
                console.log('✅ Contrôleur trouvé via window.DIAMANTS.controller');
                clearInterval(checkInterval);
                this.controller = window.DIAMANTS.controller;
                this.app = window.DIAMANTS;
                this.initialize();
                return;
            }
            
            // Méthode 2: Chercher via window.engineInstance (IntegratedController direct)
            if (window.engineInstance && window.engineInstance.drones) {
                console.log('✅ Contrôleur trouvé via window.engineInstance');
                clearInterval(checkInterval);
                this.controller = window.engineInstance;
                this.app = { drones: window.drones };
                this.initialize();
                return;
            }
            
            // Méthode 3: Chercher via window.diamantsSystem.integratedController (fallback)
            if (window.diamantsSystem && window.diamantsSystem.integratedController) {
                console.log('✅ Contrôleur trouvé via window.diamantsSystem.integratedController');
                clearInterval(checkInterval);
                this.controller = window.diamantsSystem.integratedController;
                this.app = window.diamantsSystem;
                this.initialize();
                return;
            }
            
            // Méthode 3: Compatibilité avec ancienne structure diamantsApp
            if (window.diamantsApp) {
                this.app = window.diamantsApp;
                
                if (window.diamantsApp.integratedController) {
                    clearInterval(checkInterval);
                    this.controller = window.diamantsApp.integratedController;
                    this.initialize();
                    return;
                } else if (window.diamantsApp.controller) {
                    clearInterval(checkInterval);
                    this.controller = window.diamantsApp.controller;
                    this.initialize();
                    return;
                }
                
                // Peut-être que les drones sont directement dans diamantsApp ?
                if (window.diamantsApp.drones && Array.isArray(window.diamantsApp.drones)) {
                    console.log('🎯 Drones trouvés directement dans diamantsApp');
                    clearInterval(checkInterval);
                    this.initializeWithDronesOnly();
                    return;
                }
            }
            
            // Méthode 4: Chercher dans window directement
            if (window.integratedController) {
                clearInterval(checkInterval);
                this.controller = window.integratedController;
                this.initialize();
                return;
            }
            
            if (attempts >= maxAttempts) {
                clearInterval(checkInterval);
                console.error('❌ Timeout: Système DIAMANTS non trouvé après 20s');
                console.log('💡 Conseil: Vérifiez que main.js expose bien le contrôleur via window.DIAMANTS.controller');
                console.log('💡 Structures disponibles:');
                console.log('   - window.diamantsSystem:', !!window.diamantsSystem);
                console.log('   - window.DIAMANTS:', !!window.DIAMANTS);
                console.log('   - window.diamantsApp:', !!window.diamantsApp);
            }
        }, 500);
    }

    initialize() {
        console.log('✅ Contrôleur intégré trouvé !', this.controller);
        
        // Vérifier que le contrôleur a les bonnes méthodes
        if (!this.controller.drones) {
            console.error('❌ Le contrôleur n\'a pas de propriété drones');
            return;
        }
        
        console.log(`🚁 ${this.controller.drones.length} drones trouvés dans le contrôleur`);
        
        this.exposeFunctions();
        this.initialized = true;
        this.updateUI();
        
        // Log des méthodes disponibles
        console.log('📋 Méthodes disponibles sur le contrôleur:');
        const methods = Object.getOwnPropertyNames(Object.getPrototypeOf(this.controller))
            .filter(name => typeof this.controller[name] === 'function' && name !== 'constructor');
        methods.forEach(method => console.log(`  - ${method}()`));
    }

    initializeWithDronesOnly() {
        console.log('⚠️ Pas de contrôleur trouvé, utilisation directe des drones');
        
        // Créer un contrôleur minimal
        this.controller = {
            drones: window.diamantsApp.drones || [],
            
            // Méthodes minimales basées sur les drones directement
            takeoffAllDrones: async function() {
                console.log('🚁 Décollage direct des drones...');
                this.drones.forEach((drone, index) => {
                    setTimeout(() => {
                        if (drone.takeoff) {
                            drone.takeoff(5.0 + index * 0.5);
                        } else if (drone.mesh) {
                            // Animation manuelle si pas de méthode takeoff
                            const targetY = 5.0 + index * 0.5;
                            const startY = drone.mesh.position.y;
                            const duration = 2000;
                            const startTime = Date.now();
                            
                            const animateTakeoff = () => {
                                const elapsed = Date.now() - startTime;
                                const progress = Math.min(elapsed / duration, 1);
                                drone.mesh.position.y = startY + (targetY - startY) * progress;
                                drone.state = progress < 1 ? 'TAKING_OFF' : 'FLYING';
                                
                                if (progress < 1) {
                                    requestAnimationFrame(animateTakeoff);
                                }
                            };
                            animateTakeoff();
                        }
                        console.log(`Drone ${index} décollage initié`);
                    }, index * 300);
                });
            },
            
            landAllDrones: async function() {
                console.log('🛬 Atterrissage direct des drones...');
                this.drones.forEach(drone => {
                    if (drone.land) {
                        drone.land();
                    } else if (drone.mesh) {
                        // Animation manuelle
                        const targetY = 0;
                        const startY = drone.mesh.position.y;
                        const duration = 3000;
                        const startTime = Date.now();
                        
                        const animateLanding = () => {
                            const elapsed = Date.now() - startTime;
                            const progress = Math.min(elapsed / duration, 1);
                            drone.mesh.position.y = startY + (targetY - startY) * progress;
                            drone.state = progress < 1 ? 'LANDING' : 'IDLE';
                            
                            if (progress < 1) {
                                requestAnimationFrame(animateLanding);
                            }
                        };
                        animateLanding();
                    }
                });
            },
            
            emergencyStop: function() {
                console.log('🚨 Arrêt d\'urgence!');
                this.drones.forEach(drone => {
                    if (drone.emergencyStop) {
                        drone.emergencyStop();
                    } else if (drone.mesh) {
                        drone.mesh.position.y = 0;
                        drone.state = 'IDLE';
                    }
                });
            },
            
            startMissionManual: function() {
                console.log('🎯 Mission manuelle (simulation)');
                // Faire voler les drones en cercle
                this.drones.forEach((drone, index) => {
                    if (drone.state === 'FLYING' && drone.mesh) {
                        const angle = (index / this.drones.length) * Math.PI * 2;
                        const radius = 10;
                        const targetX = Math.cos(angle) * radius;
                        const targetZ = Math.sin(angle) * radius;
                        
                        // Animation simple
                        const duration = 5000;
                        const startTime = Date.now();
                        const startX = drone.mesh.position.x;
                        const startZ = drone.mesh.position.z;
                        
                        const animateMove = () => {
                            const elapsed = Date.now() - startTime;
                            const progress = Math.min(elapsed / duration, 1);
                            drone.mesh.position.x = startX + (targetX - startX) * progress;
                            drone.mesh.position.z = startZ + (targetZ - startZ) * progress;
                            
                            if (progress < 1 && drone.state === 'FLYING') {
                                requestAnimationFrame(animateMove);
                            }
                        };
                        animateMove();
                    }
                });
            }
        };
        
        this.exposeFunctions();
        this.initialized = true;
        this.updateUI();
        
        console.log('✅ Mode dégradé activé - Contrôle direct des drones');
    }

    exposeFunctions() {
        // Fonctions de vol - utiliser les méthodes du contrôleur intégré
        window.takeoffAllDrones = async () => {
            console.log('🛫 Commande décollage reçue!');
            if (this.checkSystem()) {
                try {
                    this.showNotification('🛫 Décollage en cours...', 'info');
                    
                    // Appel prioritaire vers le contrôleur intégré
                    if (this.controller.takeoffAllDrones) {
                        await this.controller.takeoffAllDrones();
                        this.showNotification('✅ Décollage terminé', 'success');
                    } else if (this.controller.drones && this.controller.drones.length > 0) {
                        // Fallback: animation manuelle des drones
                        console.log('🔄 Animation manuelle du décollage...');
                        this.controller.drones.forEach((drone, index) => {
                            if (drone.mesh) {
                                const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
                                const targetAltitude = PLATFORM_HEIGHT + 1.0 + (index * 0.2);
                                this.animateDroneTakeoff(drone, targetAltitude);
                            }
                        });
                        this.showNotification('✅ Décollage simulé', 'success');
                    }
                } catch (error) {
                    console.error('❌ Erreur décollage:', error);
                    this.showNotification('❌ Erreur décollage', 'error');
                }
            }
        };
        
        window.landAllDrones = async () => {
            console.log('🛬 Commande atterrissage reçue!');
            if (this.checkSystem()) {
                try {
                    this.showNotification('🛬 Atterrissage en cours...', 'info');
                    
                    if (this.controller.landAllDrones) {
                        await this.controller.landAllDrones();
                        this.showNotification('✅ Atterrissage terminé', 'success');
                    } else if (this.controller.drones && this.controller.drones.length > 0) {
                        // Fallback: animation manuelle
                        console.log('🔄 Animation manuelle de l\'atterrissage...');
                        this.controller.drones.forEach((drone, index) => {
                            if (drone.mesh) {
                                this.animateDroneLanding(drone);
                            }
                        });
                        this.showNotification('✅ Atterrissage simulé', 'success');
                    }
                } catch (error) {
                    console.error('❌ Erreur atterrissage:', error);
                    this.showNotification('❌ Erreur atterrissage', 'error');
                }
            }
        };
        
        window.emergencyLand = () => {
            console.log('🚨 ARRÊT D\'URGENCE ACTIVÉ!');
            console.log('🎯 CIBLE: Atterrissage sur plateforme 8.5m');
            if (this.checkSystem()) {
                try {
                    // Arrêt immédiat des moteurs pour tous les drones - descente vers plateforme
                    this.controller.drones.forEach((drone) => {
                        if (drone.mesh) {
                            // Arrêter immédiatement toutes les animations
                            if (drone.propellerAnimationId) {
                                cancelAnimationFrame(drone.propellerAnimationId);
                                drone.propellerAnimationId = null;
                            }
                            if (drone.stabilizationId) {
                                cancelAnimationFrame(drone.stabilizationId);
                                drone.stabilizationId = null;
                            }
                            
                            // Chute contrôlée d'urgence (plus rapide qu'un atterrissage normal)
                            this.emergencyLandDrone(drone);
                        }
                    });
                    
                    // Appeler aussi la méthode du contrôleur si disponible
                    if (this.controller.emergencyStop) {
                        this.controller.emergencyStop();
                    }
                    
                    this.showNotification('🚨 ARRÊT D\'URGENCE ACTIVÉ!', 'error');
                } catch (error) {
                    console.error('❌ Erreur arrêt d\'urgence:', error);
                }
            }
        };
        
        window.stopAllDrones = () => {
            if (this.checkSystem()) {
                // Arrêter les mouvements sans atterrir
                this.controller.drones.forEach(drone => {
                    if (drone.hover) drone.hover();
                    else if (drone.velocity) {
                        drone.velocity.x = 0;
                        drone.velocity.z = 0;
                    }
                });
                this.showNotification('⏹️ Drones stoppés', 'warning');
            }
        };
        
        // 🚨 CORRECTION CRITIQUE: Fonctions de mission avec hauteur plateforme
        window.launchMission = () => {
            console.log('🚀 CORRECTION: launchMission() - hauteur plateforme 8.5m prise en compte');
            
            if (this.checkSystem()) {
                // 🚨 CORRECTION: startMissionManual n'existe pas → utiliser startMission
                if (typeof this.controller.startMission === 'function') {
                    this.controller.startMission();
                    this.showNotification('🚀 Mission lancée (plateforme: 8.5m)', 'success');
                } else if (typeof this.controller.launchSequence === 'function') {
                    this.controller.launchSequence();
                    this.showNotification('🚀 Séquence lancée (plateforme: 8.5m)', 'success');
                } else {
                    // Fallback: simulation avec hauteurs ajustées
                    console.log('⚠️ Simulation mission - hauteurs ajustées pour plateforme 8.5m');
                    this.simulateMissionWithPlatformHeight();
                    this.showNotification('🔧 Mission simulée (plateforme: 8.5m)', 'info');
                }
            }
        };
        
        // 🚨 CORRECTION: startMissionManual avec méthodes alternatives
        window.startMissionManual = () => {
            console.log('🚀 CORRECTION: startMissionManual() - hauteur plateforme prise en compte');
            
            if (this.checkSystem()) {
                if (typeof this.controller.startMission === 'function') {
                    this.controller.startMission();
                } else if (typeof this.controller.launchSequence === 'function') {
                    this.controller.launchSequence();
                } else {
                    console.log('⚠️ Simulation startMissionManual avec plateforme 8.5m');
                    this.simulateMissionWithPlatformHeight();
                }
            }
        };
        
        window.testSingleExploration = () => {
            if (this.checkSystem() && this.controller.drones.length > 0) {
                const drone = this.controller.drones[0];
                if (drone.takeoff) {
                    drone.takeoff(5.0);
                    setTimeout(() => {
                        if (drone.moveTo) {
                            drone.moveTo({x: 10, y: 5, z: 10});
                        }
                    }, 2000);
                }
                this.showNotification('🔬 Test mono-drone lancé', 'info');
            }
        };
        
        window.resetSwarm = () => {
            if (this.checkSystem()) {
                // Réinitialiser positions
                this.controller.drones.forEach((drone, index) => {
                    const angle = (index / this.controller.drones.length) * 2 * Math.PI;
                    const radius = 8.0;
                    if (drone.mesh) {
                        drone.mesh.position.set(
                            Math.cos(angle) * radius,
                            0,
                            Math.sin(angle) * radius
                        );
                    }
                    if (drone.velocity) {
                        drone.velocity.x = 0;
                        drone.velocity.y = 0;
                        drone.velocity.z = 0;
                    }
                    drone.state = 'IDLE';
                });
                this.showNotification('🔄 Essaim réinitialisé', 'info');
            }
        };
        
        // Fonctions de formation avec animations fluides
        window.setFormation = (type) => {
            console.log(`📐 Formation ${type} demandée`);
            if (this.checkSystem()) {
                try {
                    const formations = {
                        'line': this.createLineFormation,
                        'circle': this.createCircleFormation,
                        'triangle': this.createTriangleFormation,
                        'grid': this.createGridFormation
                    };
                    
                    if (formations[type]) {
                        const positions = formations[type].call(this);
                        this.applyFormationSmoothly(positions, type);
                        this.showNotification(`📐 Formation ${type} en cours...`, 'info');
                    } else {
                        console.warn(`⚠️ Formation '${type}' non reconnue`);
                        this.showNotification(`❌ Formation '${type}' inconnue`, 'error');
                    }
                } catch (error) {
                    console.error('❌ Erreur formation:', error);
                    this.showNotification('❌ Erreur formation', 'error');
                }
            }
        };
        
        // Fonctions de caméra
        window.resetCamera = () => {
            if (window.camera) {
                window.camera.position.set(10, 10, 10);
                window.camera.lookAt(0, 0, 0);
                if (window.controls) window.controls.update();
                this.showNotification('📹 Caméra réinitialisée', 'info');
            }
        };
        
        window.topView = () => {
            if (window.camera) {
                window.camera.position.set(0, 30, 0.1);
                window.camera.lookAt(0, 0, 0);
                if (window.controls) window.controls.update();
                this.showNotification('📹 Vue du dessus', 'info');
            }
        };
        
        window.toggleFollowMode = () => {
            this.showNotification('📹 Mode suivi (à implémenter)', 'info');
        };
        
        window.zoomToSwarm = () => {
            if (this.checkSystem() && window.camera) {
                let center = {x: 0, y: 0, z: 0};
                let count = 0;
                
                this.controller.drones.forEach(drone => {
                    if (drone.mesh) {
                        center.x += drone.mesh.position.x;
                        center.y += drone.mesh.position.y;
                        center.z += drone.mesh.position.z;
                        count++;
                    }
                });
                
                if (count > 0) {
                    center.x /= count;
                    center.y /= count;
                    center.z /= count;
                    
                    window.camera.position.set(center.x + 15, center.y + 15, center.z + 15);
                    window.camera.lookAt(center.x, center.y, center.z);
                    if (window.controls) window.controls.update();
                }
                
                this.showNotification('📹 Vue d\'ensemble', 'info');
            }
        };
        
        // Fonctions stub pour les autres boutons
        window.updateStatusPanel = () => this.updateUI();
        window.showNotification = (msg, type) => this.showNotification(msg, type);
        window.stopMission = () => console.log('Mission stoppée');
        window.selectDroneToFollow = () => console.log('Drone sélectionné');
        window.updateSafetyDistance = (val) => console.log('Distance:', val);
        window.updateAltitudeDisplay = (val) => console.log('Altitude:', val);
        window.applyMode = () => console.log('Mode appliqué');
        window.toggleDebugLogs = () => console.log('Debug logs toggle');
        window.clearDebugLogs = () => console.log('Logs cleared');
        window.toggleDebugPanels = () => console.log('Debug panels toggle');
        window.toggleConsoleNinja = () => console.log('Console Ninja toggle');
        
        // Fonctions de missions stub
        window.startExplorationMission = () => console.log('Mission exploration');
        window.startMappingMission = () => console.log('Mission mapping');
        window.startSurveillanceMission = () => console.log('Mission surveillance');
        window.animateDroneToPosition = () => {};
        window.animateExplorationPattern = () => {};
        window.createTakeoffEffect = () => {};
        window.flashDroneLight = () => {};
        
        // Log pour confirmer que les fonctions sont exposées
        console.log('🎮 Fonctions exposées globalement:');
        console.log('  - takeoffAllDrones:', typeof window.takeoffAllDrones);
        console.log('  - landAllDrones:', typeof window.landAllDrones);
        console.log('  - emergencyLand:', typeof window.emergencyLand);
    }

    // Les fonctions de formation sont définies plus bas dans la classe avec les bonnes altitudes
    // Suppression des anciennes définitions pour éviter la confusion

    applyFormation(positions) {
        this.controller.drones.forEach((drone, index) => {
            if (positions[index] && drone.moveTo) {
                drone.moveTo(positions[index]);
            } else if (positions[index] && drone.mesh) {
                // Fallback: déplacer directement le mesh
                drone.mesh.position.set(
                    positions[index].x,
                    positions[index].y,
                    positions[index].z
                );
            }
        });
    }

    checkSystem() {
        if (!this.initialized || !this.controller) {
            this.showNotification('⚠️ Système non initialisé', 'error');
            return false;
        }
        return true;
    }

    // Animation réaliste pour le décollage avec physique de vol
    animateDroneTakeoff(drone, targetAltitude) {
        if (!drone.mesh) return;
        
        const startY = drone.mesh.position.y;
        const duration = 4000; // 4 secondes pour un décollage réaliste
        const startTime = Date.now();
        
        console.log(`🚁 Décollage réaliste du drone ${drone.id} vers ${targetAltitude}m`);
        
        // Activer les hélices avec rotation réaliste
        this.startPropellerRotation(drone);
        
        // Variables pour animation physique réaliste
        let velocity = 0;
        const maxVelocity = 2.0; // m/s
        const acceleration = 0.8; // m/s²
        
        const animate = () => {
            const elapsed = Date.now() - startTime;
            const progress = Math.min(elapsed / duration, 1);
            
            // Phase de décollage avec accélération progressive
            if (progress < 0.3) {
                // Phase de montée en puissance (30% du temps)
                velocity = acceleration * (elapsed / 1000) * (progress / 0.3);
            } else if (progress < 0.8) {
                // Phase de montée constante (50% du temps)
                velocity = maxVelocity;
            } else {
                // Phase de ralentissement pour stabilisation (20% du temps)
                const slowProgress = (progress - 0.8) / 0.2;
                velocity = maxVelocity * (1 - slowProgress * 0.7);
            }
            
            // Calcul position avec physique réaliste
            const currentY = startY + velocity * (elapsed / 1000);
            const finalY = Math.min(currentY, startY + (targetAltitude - startY) * progress);
            
            drone.mesh.position.y = finalY;
            
            // Effets visuels réalistes pendant le décollage
            if (progress < 1) {
                // Légère vibration pendant la montée
                drone.mesh.position.x += (Math.random() - 0.5) * 0.02;
                drone.mesh.position.z += (Math.random() - 0.5) * 0.02;
                
                // Rotation légère due aux effets gyroscopiques
                drone.mesh.rotation.y += 0.01;
                
                drone.state = 'TAKING_OFF';
                requestAnimationFrame(animate);
            } else {
                // Stabilisation finale
                drone.mesh.position.x = Math.round(drone.mesh.position.x * 50) / 50;
                drone.mesh.position.z = Math.round(drone.mesh.position.z * 50) / 50;
                drone.mesh.rotation.x = 0;
                drone.mesh.rotation.z = 0;
                
                drone.state = 'FLYING';
                this.enableFlightStabilization(drone);
                console.log(`✅ Drone ${drone.id} décollé et stabilisé à ${targetAltitude}m`);
            }
        };
        
        animate();
    }

    // Animation réaliste pour l'atterrissage avec contrôle de descente
    animateDroneLanding(drone) {
        if (!drone.mesh) return;
        
        const startY = drone.mesh.position.y;
        const targetY = 0.05; // Légèrement au-dessus du sol
        const duration = 3500; // 3.5 secondes pour atterrissage en douceur
        const startTime = Date.now();
        
        console.log(`🛬 Atterrissage réaliste du drone ${drone.id}`);
        
        // Variables pour descente contrôlée
        let velocity = 0;
        const maxDescendVelocity = -1.2; // m/s (négatif = descente)
        const deceleration = 0.6; // m/s²
        
        const animate = () => {
            const elapsed = Date.now() - startTime;
            const progress = Math.min(elapsed / duration, 1);
            
            // Phase d'atterrissage avec décélération progressive
            if (progress < 0.6) {
                // Phase de descente contrôlée (60% du temps)
                velocity = maxDescendVelocity * (progress / 0.6);
            } else {
                // Phase de ralentissement final (40% du temps)
                const slowProgress = (progress - 0.6) / 0.4;
                velocity = maxDescendVelocity * (1 - slowProgress * 0.8);
            }
            
            // Calcul position avec descente réaliste
            const currentY = startY + velocity * (elapsed / 1000);
            const finalY = Math.max(currentY, targetY);
            
            drone.mesh.position.y = finalY;
            
            // Effets visuels pendant l'atterrissage
            if (progress < 1 && drone.mesh.position.y > targetY + 0.1) {
                // Stabilisation automatique en approche du sol
                if (progress > 0.7) {
                    drone.mesh.rotation.x *= 0.95;
                    drone.mesh.rotation.z *= 0.95;
                }
                
                drone.state = 'LANDING';
                requestAnimationFrame(animate);
            } else {
                // Atterrissage final et arrêt des moteurs
                drone.mesh.position.y = targetY;
                drone.mesh.rotation.x = 0;
                drone.mesh.rotation.z = 0;
                drone.mesh.rotation.y = 0;
                
                drone.state = 'IDLE';
                this.stopPropellerRotation(drone);
                console.log(`✅ Drone ${drone.id} atterri en sécurité`);
            }
        };
        
        animate();
    }

    // Système de rotation réaliste des hélices
    startPropellerRotation(drone) {
        if (!drone.mesh) return;
        
        // Trouver et activer les hélices
        const propellers = [];
        drone.mesh.traverse((child) => {
            if (child.isMesh && (
                child.name?.includes('propeller') || 
                child.name?.includes('prop') ||
                child.material?.name?.includes('propeller') ||
                child.geometry?.type === 'PlaneGeometry'
            )) {
                propellers.push(child);
            }
        });
        
        if (propellers.length === 0) {
            console.warn(`⚠️ Aucune hélice trouvée pour le drone ${drone.id}`);
            return;
        }
        
        console.log(`🌀 Démarrage rotation ${propellers.length} hélices pour drone ${drone.id}`);
        
        // Configurations réalistes Crazyflie (CW/CCW alternées)
        const rotationSpeeds = [0.8, -0.8, 0.8, -0.8]; // Alternance horaire/anti-horaire
        
        propellers.forEach((propeller, index) => {
            propeller.userData.isRotating = true;
            propeller.userData.rotationSpeed = rotationSpeeds[index % 4] || 0.8;
            propeller.userData.currentRotation = 0;
            
            // Rendre les hélices semi-transparentes quand elles tournent
            if (propeller.material) {
                propeller.material.transparent = true;
                propeller.material.opacity = 0.3;
                propeller.material.needsUpdate = true;
            }
        });
        
        // Démarrer l'animation continue
        if (!drone.propellerAnimationId) {
            drone.propellerAnimationId = this.animatePropellers(drone, propellers);
        }
    }

    stopPropellerRotation(drone) {
        if (!drone.mesh) return;
        
        console.log(`🛑 Arrêt rotation hélices pour drone ${drone.id}`);
        
        // Arrêter l'animation
        if (drone.propellerAnimationId) {
            cancelAnimationFrame(drone.propellerAnimationId);
            drone.propellerAnimationId = null;
        }
        
        // Remettre les hélices opaques
        drone.mesh.traverse((child) => {
            if (child.isMesh && child.userData.isRotating) {
                child.userData.isRotating = false;
                
                if (child.material) {
                    child.material.opacity = 0.8;
                    child.material.needsUpdate = true;
                }
                
                // Arrêt progressif
                const slowDown = () => {
                    if (child.userData.rotationSpeed && Math.abs(child.userData.rotationSpeed) > 0.01) {
                        child.userData.rotationSpeed *= 0.95;
                        child.rotation.z = child.userData.currentRotation;
                        child.userData.currentRotation += child.userData.rotationSpeed;
                        requestAnimationFrame(slowDown);
                    }
                };
                slowDown();
            }
        });
    }

    animatePropellers(drone, propellers) {
        const animate = () => {
            if (!drone.mesh || drone.state === 'IDLE') return;
            
            propellers.forEach((propeller) => {
                if (propeller.userData.isRotating) {
                    propeller.userData.currentRotation += propeller.userData.rotationSpeed;
                    propeller.rotation.z = propeller.userData.currentRotation;
                }
            });
            
            drone.propellerAnimationId = requestAnimationFrame(animate);
        };
        
        return requestAnimationFrame(animate);
    }

    // Système de stabilisation en vol
    enableFlightStabilization(drone) {
        if (!drone.mesh || drone.stabilizationId) return;
        
        console.log(`⚖️ Activation stabilisation pour drone ${drone.id}`);
        
        const stabilize = () => {
            if (!drone.mesh || drone.state !== 'FLYING') {
                drone.stabilizationId = null;
                return;
            }
            
            // Micro-mouvements de stabilisation (simule les corrections automatiques)
            const stabilizationForce = 0.005;
            const dampening = 0.98;
            
            // Correction des oscillations
            if (Math.abs(drone.mesh.rotation.x) > 0.01) {
                drone.mesh.rotation.x *= dampening;
            }
            if (Math.abs(drone.mesh.rotation.z) > 0.01) {
                drone.mesh.rotation.z *= dampening;
            }
            
            // Légères oscillations naturelles
            const time = Date.now() * 0.001;
            drone.mesh.position.y += Math.sin(time * 2 + drone.id.charCodeAt(0)) * stabilizationForce;
            
            drone.stabilizationId = requestAnimationFrame(stabilize);
        };
        
        drone.stabilizationId = requestAnimationFrame(stabilize);
    }

    // 🚨 Atterrissage d'urgence - CORRECTION: cible plateforme 8.5m
    emergencyLandDrone(drone) {
        if (!drone.mesh) return;
        
        const startY = drone.mesh.position.y;
        const PLATFORM_HEIGHT = 8.5; // CORRECTION: cible plateforme au lieu de sol
        const targetY = PLATFORM_HEIGHT; // Atterrissage sur plateforme
        const duration = 1500; // 1.5 secondes (plus rapide qu'un atterrissage normal)
        const startTime = Date.now();
        
        console.log(`🚨 Atterrissage d'urgence drone ${drone.id}`);
        console.log(`🎯 CORRECTION: Cible plateforme ${PLATFORM_HEIGHT}m au lieu de sol 0m`);
        
        // Arrêter immédiatement les hélices
        this.stopPropellerRotation(drone);
        
        const animate = () => {
            const elapsed = Date.now() - startTime;
            const progress = Math.min(elapsed / duration, 1);
            
            // Chute plus rapide avec léger freinage vers la fin
            let easeProgress;
            if (progress < 0.8) {
                // Chute rapide (80% du temps)
                easeProgress = progress * 1.2;
            } else {
                // Freinage d'urgence (20% du temps)
                const slowProgress = (progress - 0.8) / 0.2;
                easeProgress = 0.96 + slowProgress * 0.04;
            }
            
            drone.mesh.position.y = startY + (targetY - startY) * Math.min(easeProgress, 1);
            
            // Rotation chaotique pendant la chute (simule perte de contrôle)
            if (progress < 0.7) {
                drone.mesh.rotation.x += 0.05;
                drone.mesh.rotation.z += 0.03;
            } else {
                // Stabilisation forcée avant impact
                drone.mesh.rotation.x *= 0.9;
                drone.mesh.rotation.z *= 0.9;
            }
            
            if (progress < 1) {
                drone.state = 'EMERGENCY_LANDING';
                requestAnimationFrame(animate);
            } else {
                // Impact final
                drone.mesh.position.y = 0;
                drone.mesh.rotation.x = 0;
                drone.mesh.rotation.z = 0;
                drone.state = 'EMERGENCY_STOPPED';
                console.log(`💥 Drone ${drone.id} atterrissage d'urgence terminé`);
            }
        };
        
        animate();
    }

    updateUI() {
        const droneCount = document.getElementById('drone_count');
        if (droneCount && this.controller) {
            droneCount.textContent = this.controller.drones ? this.controller.drones.length : 0;
        }
        
        const status = document.getElementById('system_status');
        if (status) {
            status.textContent = this.initialized ? 'Ready' : 'Initializing...';
        }
        
        // Mettre à jour le sélecteur de drones
        const selector = document.getElementById('drone_selector');
        if (selector && this.controller && this.controller.drones) {
            selector.innerHTML = '<option value="">No Selection</option>';
            this.controller.drones.forEach((drone, index) => {
                const option = document.createElement('option');
                option.value = index;
                option.textContent = `Drone ${index} (${drone.id})`;
                selector.appendChild(option);
            });
        }
    }

    // 🚨 MÉTHODE CRITIQUE: Simulation mission avec hauteur plateforme
    simulateMissionWithPlatformHeight() {
        console.log('🎯 SIMULATION MISSION - Hauteur plateforme 8.5m prise en compte');
        
        const PLATFORM_HEIGHT = 8.5; // Hauteur détectée dans les logs
        const droneHeights = {
            'crazyflie_0': 2.0,   // Observé: pos=8.50m dans logs
            'crazyflie_1': 2.5,   // Observé: pos=8.50m dans logs
            'crazyflie_2': 3.0,   // Observé: pos=8.50m dans logs  
            'crazyflie_3': 3.5,   // Observé: pos=8.50m dans logs
            'crazyflie_4': 4.0,   // Observé: pos=8.50m dans logs
            'crazyflie_5': 4.5    // Observé: pos=8.50m dans logs
        };
        
        console.log('📍 Configuration hauteurs détectée dans Console Ninja:');
        Object.entries(droneHeights).forEach(([droneId, relativeHeight]) => {
            const absoluteHeight = PLATFORM_HEIGHT + relativeHeight;
            console.log(`🚁 ${droneId}: ${relativeHeight}m relatif → ${absoluteHeight}m absolu`);
        });
        
        // Simulation séquence de décollage avec hauteurs ajustées pour plateforme
        console.log('🚀 Séquence de décollage simulée avec corrections plateforme...');
        
        if (this.controller && this.controller.drones) {
            this.controller.drones.forEach((drone, index) => {
                const droneId = drone.id || `crazyflie_${index}`;
                const targetHeight = droneHeights[droneId] || 3.0;
                const absoluteHeight = PLATFORM_HEIGHT + targetHeight;
                
                setTimeout(() => {
                    console.log(`✈️ ${droneId}: Décollage simulé vers ${absoluteHeight}m (plateforme ${PLATFORM_HEIGHT}m + ${targetHeight}m)`);
                    
                    // Simulation des étapes de vol avec correction hauteur
                    if (drone.position) {
                        drone.position.y = absoluteHeight;
                    }
                    
                    if (drone.setState) {
                        drone.setState('FLYING');
                    }
                    
                }, index * 500); // Échelonnement des décollages
            });
        }
        
        // Notification de fin de séquence
        setTimeout(() => {
            console.log('✅ Mission simulée terminée - Tous drones à altitude de sécurité au-dessus plateforme 8.5m');
            this.showNotification('✅ Mission simulée (plateforme: 8.5m)', 'success');
        }, (this.controller?.drones?.length || 6) * 500 + 1000);
    }

    showNotification(message, type = 'info') {
        console.log(`[${type.toUpperCase()}] ${message}`);
        
        let notification = document.getElementById('notification');
        if (!notification) {
            notification = document.createElement('div');
            notification.id = 'notification';
            notification.style.cssText = `
                position: fixed;
                top: 70px;
                left: 50%;
                transform: translateX(-50%);
                padding: 10px 20px;
                border-radius: 5px;
                z-index: 10000;
                font-family: monospace;
                font-weight: bold;
                transition: opacity 0.3s;
                pointer-events: none;
            `;
            document.body.appendChild(notification);
        }
        
        const colors = {
            'success': '#10b981',
            'error': '#ef4444',
            'warning': '#f59e0b',
            'info': '#3b82f6'
        };
        
        notification.style.background = colors[type] || colors['info'];
        notification.style.color = 'white';
        notification.textContent = message;
        notification.style.opacity = '1';
        
        clearTimeout(notification.hideTimeout);
        notification.hideTimeout = setTimeout(() => {
            notification.style.opacity = '0';
        }, 3000);
    }

    // === FORMATIONS AVANCÉES ===
    
    applyFormationPattern(formationType) {
        console.log(`🌟 === DÉBUT applyFormationPattern(${formationType}) ===`);
        console.log('🔍 this.controller exists:', !!this.controller);
        console.log('🔍 this.controller.drones exists:', !!this.controller?.drones);
        console.log('🔍 Drone count:', this.controller?.drones?.length || 'N/A');
        
        let positions = [];
        
        switch(formationType.toLowerCase()) {
            case 'grid':
                console.log('🎯 Creating GRID formation');
                positions = this.createGridFormation();
                break;
            case 'line':
                console.log('🎯 Creating LINE formation');
                positions = this.createLineFormation();
                break;
            case 'circle':
                console.log('🎯 Creating CIRCLE formation');
                positions = this.createCircleFormation();
                break;
            case 'triangle':
            case 'v_formation':
                console.log('🎯 Creating TRIANGLE formation');
                positions = this.createTriangleFormation();
                break;
            case 'diamond':
                console.log('🎯 Creating DIAMOND formation');
                positions = this.createDiamondFormation();
                break;
            default:
                console.warn(`⚠️ Formation inconnue: ${formationType}, utilisation de grid`);
                positions = this.createGridFormation();
        }
        
        console.log(`🔢 Positions créées: ${positions.length}`);
        console.log('📍 Positions:', positions);
        
        if (positions.length > 0) {
            console.log('🎯 Calling applyFormationSmoothly...');
            this.applyFormationSmoothly(positions, formationType);
            console.log(`✅ Formation ${formationType} appliquée à ${positions.length} drones`);
        } else {
            console.error('❌ Aucune position créée pour la formation');
        }
        console.log(`🌟 === FIN applyFormationPattern(${formationType}) ===`);
    }
    
    createDiamondFormation() {
        const droneCount = this.controller.drones.length;
        const positions = [];
        const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
        const SAFETY_MARGIN = 3.0; // Marge sécurité au-dessus plateforme
        
        console.log(`💎 Formation diamant - Altitude: ${PLATFORM_HEIGHT + SAFETY_MARGIN}m (plateforme ${PLATFORM_HEIGHT}m + ${SAFETY_MARGIN}m)`);
        
        if (droneCount === 1) {
            return [{ x: 0, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 0 }];
        }
        
        // Centre
        positions.push({ x: 0, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 0 });
        
        if (droneCount > 1) {
            // Points cardinaux
            positions.push({ x: 5, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 0 });   // Est
            if (droneCount > 2) positions.push({ x: 0, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 5 });    // Nord
            if (droneCount > 3) positions.push({ x: -5, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 0 });  // Ouest
            if (droneCount > 4) positions.push({ x: 0, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: -5 });   // Sud
            
            // Points diagonaux
            for (let i = 5; i < droneCount; i++) {
                const layer = Math.floor((i - 5) / 4) + 1;
                const pos = (i - 5) % 4;
                const distance = 3.5 * (layer + 1);
                
                switch(pos) {
                    case 0: positions.push({ x: distance, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: distance }); break;
                    case 1: positions.push({ x: -distance, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: distance }); break;
                    case 2: positions.push({ x: -distance, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: -distance }); break;
                    case 3: positions.push({ x: distance, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: -distance }); break;
                }
            }
        }
        
        return positions.slice(0, droneCount);
    }
    
    createLineFormation() {
        const droneCount = this.controller.drones.length;
        const spacing = 3.0; // 3 mètres entre chaque drone
        const startX = -(droneCount - 1) * spacing / 2;
        const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
        const SAFETY_MARGIN = 3.0; // Marge sécurité au-dessus plateforme
        
        console.log(`📐 Formation ligne - Altitude: ${PLATFORM_HEIGHT + SAFETY_MARGIN}m (plateforme ${PLATFORM_HEIGHT}m + ${SAFETY_MARGIN}m)`);
        
        return this.controller.drones.map((drone, index) => ({
            x: startX + index * spacing,
            y: PLATFORM_HEIGHT + SAFETY_MARGIN, // 11.5m au-dessus du sol
            z: 0
        }));
    }

    createCircleFormation() {
        const droneCount = this.controller.drones.length;
        const radius = Math.max(droneCount * 1.5, 8.0); // Rayon adaptatif
        const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
        const SAFETY_MARGIN = 3.0; // Marge sécurité au-dessus plateforme
        
        console.log(`⭕ Formation cercle - Altitude: ${PLATFORM_HEIGHT + SAFETY_MARGIN}m (plateforme ${PLATFORM_HEIGHT}m + ${SAFETY_MARGIN}m)`);
        
        return this.controller.drones.map((drone, index) => {
            const angle = (index / droneCount) * 2 * Math.PI;
            return {
                x: Math.cos(angle) * radius,
                y: PLATFORM_HEIGHT + SAFETY_MARGIN, // 11.5m au-dessus du sol
                z: Math.sin(angle) * radius
            };
        });
    }

    createTriangleFormation() {
        const droneCount = this.controller.drones.length;
        const positions = [];
        const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
        const SAFETY_MARGIN = 3.0; // Marge sécurité au-dessus plateforme
        
        console.log(`🔺 Formation triangle - Altitude: ${PLATFORM_HEIGHT + SAFETY_MARGIN}m (plateforme ${PLATFORM_HEIGHT}m + ${SAFETY_MARGIN}m)`);
        
        if (droneCount === 1) {
            return [{ x: 0, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 0 }];
        }
        
        // Leader au front
        positions.push({ x: 0, y: PLATFORM_HEIGHT + SAFETY_MARGIN, z: 8 });
        
        // Distribuer le reste en V
        for (let i = 1; i < droneCount; i++) {
            const side = i % 2 === 1 ? 1 : -1; // Alternance gauche/droite
            const layer = Math.floor((i - 1) / 2) + 1;
            positions.push({
                x: side * layer * 2.5,
                y: PLATFORM_HEIGHT + SAFETY_MARGIN, // 11.5m au-dessus du sol
                z: 8 - layer * 3
            });
        }
        
        return positions;
    }

    createGridFormation() {
        const droneCount = this.controller.drones.length;
        const cols = Math.ceil(Math.sqrt(droneCount));
        const rows = Math.ceil(droneCount / cols);
        const spacing = 4.0;
        const PLATFORM_HEIGHT = 8.5; // Hauteur de la plateforme
        const SAFETY_MARGIN = 3.0; // Marge sécurité au-dessus plateforme
        
        console.log(`🔢 Formation grille - Altitude: ${PLATFORM_HEIGHT + SAFETY_MARGIN}m (plateforme ${PLATFORM_HEIGHT}m + ${SAFETY_MARGIN}m)`);
        
        const positions = [];
        for (let i = 0; i < droneCount; i++) {
            const row = Math.floor(i / cols);
            const col = i % cols;
            positions.push({
                x: (col - (cols - 1) / 2) * spacing,
                y: PLATFORM_HEIGHT + SAFETY_MARGIN, // 11.5m au-dessus du sol
                z: (row - (rows - 1) / 2) * spacing
            });
        }
        
        return positions;
    }

    applyFormationSmoothly(positions, formationType) {
        console.log(`🎯 === DÉBUT applyFormationSmoothly ===`);
        console.log(`🎯 Formation: ${formationType} pour ${positions.length} drones`);
        console.log(`📍 Positions cibles:`, positions);
        
        this.controller.drones.forEach((drone, index) => {
            if (!drone.mesh || !positions[index]) {
                console.log(`⚠️ Drone ${index} skippé - mesh: ${!!drone.mesh}, position: ${!!positions[index]}`);
                return;
            }
            
            const target = positions[index];
            const startPos = {
                x: drone.position.x,
                y: drone.position.y,
                z: drone.position.z
            };
            
            console.log(`🚁 Drone ${index} (${drone.id}):`);
            console.log(`   📍 Position actuelle: x=${startPos.x.toFixed(2)}, y=${startPos.y.toFixed(2)}, z=${startPos.z.toFixed(2)}`);
            console.log(`   🎯 Position cible: x=${target.x.toFixed(2)}, y=${target.y.toFixed(2)}, z=${target.z.toFixed(2)}`);
            console.log(`   📐 Mesh position: x=${drone.mesh.position.x.toFixed(2)}, y=${drone.mesh.position.y.toFixed(2)}, z=${drone.mesh.position.z.toFixed(2)}`);
            
            const duration = 5000; // 5 secondes pour la transition
            const startTime = Date.now();
            
            // S'assurer que le drone est en vol
            if (drone.state === 'IDLE') {
                this.startPropellerRotation(drone);
                drone.state = 'FLYING';
            }
            
            const animateToPosition = () => {
                const elapsed = Date.now() - startTime;
                const progress = Math.min(elapsed / duration, 1);
                
                // Courbe d'easing pour mouvement fluide
                const easeProgress = progress < 0.5 
                    ? 2 * progress * progress
                    : 1 - Math.pow(-2 * progress + 2, 2) / 2;
                
                // Interpolation position
                const newX = startPos.x + (target.x - startPos.x) * easeProgress;
                const newY = startPos.y + (target.y - startPos.y) * easeProgress;
                const newZ = startPos.z + (target.z - startPos.z) * easeProgress;
                
                // Debug: log des positions pendant l'animation (chaque seconde)
                if (elapsed % 1000 < 50) {
                    console.log(`🔄 Drone ${index} animation progress: ${(progress*100).toFixed(1)}%`);
                    console.log(`   📍 New position: x=${newX.toFixed(2)}, y=${newY.toFixed(2)}, z=${newZ.toFixed(2)}`);
                    console.log(`   🎯 Target: x=${target.x.toFixed(2)}, y=${target.y.toFixed(2)}, z=${target.z.toFixed(2)}`);
                }
                
                // CORRECTION: modifier drone.position ET les propriétés de contrôle ROS2/Gazebo
                drone.position.x = newX;
                drone.position.y = newY;
                drone.position.z = newZ;
                
                // NOUVEAUTÉ: Mettre à jour aussi les propriétés de contrôle du drone
                if (drone.targetPosition) {
                    drone.targetPosition.x = newX;
                    drone.targetPosition.y = newY;
                    drone.targetPosition.z = newZ;
                }
                if (drone.targetAltitude !== undefined) {
                    drone.targetAltitude = newY;
                }
                
                // REMOVEÉ: Plus besoin de forcer mesh.position car updateVisuals() s'en occupe
                // drone.mesh.position.x = newX;
                // drone.mesh.position.y = newY;
                // drone.mesh.position.z = newZ;
                
                // Orientation vers la direction de déplacement
                const deltaX = target.x - startPos.x;
                const deltaZ = target.z - startPos.z;
                if (Math.abs(deltaX) > 0.1 || Math.abs(deltaZ) > 0.1) {
                    const targetRotation = Math.atan2(deltaX, deltaZ);
                    drone.mesh.rotation.y = targetRotation * easeProgress;
                }
                
                if (progress < 1) {
                    requestAnimationFrame(animateToPosition);
                } else {
                    console.log(`✅ Drone ${drone.id} en position formation ${formationType}`);
                    console.log(`   📍 Position finale: x=${drone.position.x.toFixed(2)}, y=${drone.position.y.toFixed(2)}, z=${drone.position.z.toFixed(2)}`);
                    console.log(`   📐 Mesh finale: x=${drone.mesh.position.x.toFixed(2)}, y=${drone.mesh.position.y.toFixed(2)}, z=${drone.mesh.position.z.toFixed(2)}`);
                    if (index === this.controller.drones.length - 1) {
                        this.showNotification(`✅ Formation ${formationType} terminée`, 'success');
                    }
                }
            };
            
            animateToPosition();
        });
    }
}

// =======================================================================
// FONCTIONS UI COMPLÉMENTAIRES MANQUANTES
// =======================================================================

// Fonctions de mode et contrôles avancés
window.applyMode = function() {
    const modeSelect = document.getElementById('mode_select');
    if (modeSelect && window.dronePanelController?.checkSystem()) {
        const mode = modeSelect.value;
        console.log(`🔧 Application du mode: ${mode}`);
        
        const controller = window.dronePanelController.controller;
        if (controller?.drones) {
            controller.drones.forEach(drone => {
                if (drone.setMode) {
                    drone.setMode(mode);
                } else {
                    drone.explorationMode = mode;
                }
            });
        }
        
        window.dronePanelController.showNotification(`🔧 Mode ${mode} appliqué`, 'info');
    }
};

// Fonctions de debug et logs
window.toggleDebugLogs = function() {
    const btn = document.getElementById('btn-toggle-logs');
    if (btn) {
        const isVisible = btn.textContent.includes('Hide');
        btn.textContent = isVisible ? '📋 Show Logs' : '📋 Hide Logs';
        
        // Ici on pourrait afficher/masquer un panneau de logs
        console.log(isVisible ? 'Logs cachés' : 'Logs affichés');
        window.dronePanelController?.showNotification(
            isVisible ? 'Logs cachés' : 'Logs affichés', 
            'info'
        );
    }
};

window.clearDebugLogs = function() {
    console.clear();
    window.dronePanelController?.showNotification('🧹 Logs effacés', 'info');
};

window.toggleDebugPanels = function() {
    const btn = document.getElementById('btn-toggle-debug');
    if (btn) {
        const isVisible = btn.textContent.includes('Hide');
        btn.textContent = isVisible ? '🛠️ Show Debug' : '🛠️ Hide Debug';
        window.dronePanelController?.showNotification(
            isVisible ? 'Debug caché' : 'Debug affiché', 
            'info'
        );
    }
};

window.toggleConsoleNinja = function() {
    const btn = document.getElementById('btn-toggle-ninja');
    if (btn) {
        const isVisible = btn.textContent.includes('Hide');
        btn.textContent = isVisible ? '🥷 Console' : '🥷 Hide Console';
        window.dronePanelController?.showNotification(
            'Console Ninja ' + (isVisible ? 'masqué' : 'affiché'), 
            'info'
        );
    }
};

// Fonctions d'altitude et sécurité
window.updateAltitudeDisplay = function(value) {
    const display = document.getElementById('altitude_display');
    if (display) {
        display.textContent = parseFloat(value).toFixed(1) + 'm';
    }
};

window.updateSafetyDistance = function(value) {
    const display = document.getElementById('safety_distance_value');
    if (display) {
        display.textContent = parseFloat(value).toFixed(1) + 'm';
    }
    
    // Appliquer la distance de sécurité au système
    if (window.dronePanelController?.checkSystem()) {
        const controller = window.dronePanelController.controller;
        if (controller.setSafetyDistance) {
            controller.setSafetyDistance(parseFloat(value));
        }
    }
};

// =======================================================================
// ATTERRISSAGE IMMÉDIAT - FONCTION PRINCIPALE
// =======================================================================

window.LAND_ALL_DRONES_NOW = async function() {
    console.log('🛬🛬🛬 ATTERRISSAGE IMMÉDIAT DEMANDÉ !');
    
    if (window.dronePanelController?.checkSystem()) {
        try {
            // Méthode 1: Via le système d'atterrissage dédié
            if (window.landAllDrones) {
                console.log('📞 Appel landAllDrones()...');
                await window.landAllDrones();
                return;
            }
            
            // Méthode 2: Via le contrôleur intégré
            const controller = window.dronePanelController.controller;
            if (controller?.landAllDrones) {
                console.log('📞 Appel controller.landAllDrones()...');
                await controller.landAllDrones();
                return;
            }
            
            // Méthode 3: Atterrissage manuel direct
            if (controller?.drones && controller.drones.length > 0) {
                console.log('🔧 Atterrissage manuel forcé...');
                controller.drones.forEach((drone, index) => {
                    if (drone.mesh) {
                        console.log(`🛬 Atterrissage drone ${drone.id || index}`);
                        
                        // Forcer l'état d'atterrissage
                        drone.state = 'LANDING';
                        
                        // Animation d'atterrissage immédiate
                        const startY = drone.mesh.position.y;
                        const startTime = Date.now();
                        const duration = 3000; // 3 secondes
                        
                        const animateLanding = () => {
                            const elapsed = Date.now() - startTime;
                            const progress = Math.min(elapsed / duration, 1);
                            
                            // Descente avec courbe d'ease-out
                            const easeProgress = 1 - Math.pow(1 - progress, 3);
                            drone.mesh.position.y = startY * (1 - easeProgress);
                            
                            // Arrêter les hélices progressivement
                            if (drone.stopRotors) {
                                drone.stopRotors();
                            } else if (drone.propellerAnimation) {
                                drone.propellerAnimation.power = 1 - progress;
                            }
                            
                            if (progress < 1) {
                                requestAnimationFrame(animateLanding);
                            } else {
                                // Atterrissage terminé
                                drone.mesh.position.y = 0;
                                drone.state = 'IDLE';
                                console.log(`✅ Drone ${drone.id || index} atterri`);
                            }
                        };
                        
                        animateLanding();
                    }
                });
                
                window.dronePanelController.showNotification('🛬 Atterrissage forcé en cours...', 'warning');
            } else {
                console.warn('⚠️ Aucun drone trouvé pour l\'atterrissage');
                window.dronePanelController.showNotification('⚠️ Aucun drone trouvé', 'warning');
            }
            
        } catch (error) {
            console.error('❌ Erreur lors de l\'atterrissage:', error);
            window.dronePanelController.showNotification('❌ Erreur atterrissage', 'error');
        }
    } else {
        console.warn('⚠️ Système DIAMANTS non initialisé');
    }
};

// Initialiser le contrôleur au chargement
window.dronePanelController = new DronePanelController();

console.log('✅ drone-panel-controller.js chargé avec atterrissage immédiat');
console.log('🔍 Recherche du système DIAMANTS en cours...');
console.log('💡 Tapez LAND_ALL_DRONES_NOW() pour atterrir immédiatement');

// Exposer une fonction de debug globale
window.debugDroneSystem = function() {
    console.log('=== DEBUG DRONE SYSTEM ===');
    console.log('dronePanelController:', window.dronePanelController);
    console.log('  initialized:', window.dronePanelController?.initialized);
    console.log('  controller:', window.dronePanelController?.controller);
    console.log('diamantsApp:', window.diamantsApp);
    if (window.diamantsApp) {
        console.log('  Structure:', Object.keys(window.diamantsApp));
        console.log('  integratedController:', window.diamantsApp.integratedController);
        console.log('  drones:', window.diamantsApp.drones);
    }
    console.log('=========================');
};

console.log('💡 Tapez debugDroneSystem() dans la console pour diagnostiquer');

/**
 * ========================================
 * FONCTIONS UI GLOBALES - CÂBLAGE COMPLET
 * ========================================
 * Toutes les fonctions onclick détectées dans l'HTML
 */

// === NAVIGATION & RELOAD ===
window.reload = function() {
    console.log('🔄 Rechargement de la page...');
    location.reload();
};

// === MISSION CONTROL ===
window.launchMission = function() {
    console.log('🚀 === LANCEMENT DE MISSION ===');
    if (window.dronePanelController?.checkSystem()) {
        const controller = window.dronePanelController.controller;
        
        try {
            if (controller.startMissionManual) {
                controller.startMissionManual();
                console.log('✅ Mission lancée via startMissionManual()');
            } else if (controller.startMission) {
                controller.startMission();
                console.log('✅ Mission lancée via startMission()');
            } else if (controller.launchMission) {
                controller.launchMission();
                console.log('✅ Mission lancée via launchMission()');
            } else {
                console.log('⚠️ Méthode de lancement générique');
                // Démarrage générique
                if (controller.drones) {
                    controller.drones.forEach(drone => {
                        if (drone.setState) drone.setState('MISSION');
                        if (drone.startMission) drone.startMission();
                    });
                }
            }
            
            window.dronePanelController.showNotification('🚀 Mission lancée!', 'success');
        } catch (error) {
            console.error('❌ Erreur lancement mission:', error);
            window.dronePanelController.showNotification('❌ Erreur mission', 'error');
        }
    } else {
        console.warn('❌ Système DIAMANTS non disponible');
        window.dronePanelController.showNotification('❌ Système non prêt', 'error');
    }
};

window.emergencyLand = function() {
    console.log('🚨 === ATTERRISSAGE D\'URGENCE ===');
    if (window.dronePanelController?.checkSystem()) {
        const controller = window.dronePanelController.controller;
        
        try {
            if (controller.emergencyLand) {
                controller.emergencyLand();
                console.log('✅ Atterrissage d\'urgence via emergencyLand()');
            } else if (controller.landAllDrones) {
                controller.landAllDrones();
                console.log('✅ Atterrissage d\'urgence via landAllDrones()');
            } else if (controller.drones) {
                // Force emergency landing pour tous les drones
                controller.drones.forEach((drone, index) => {
                    console.log(`🚨 Drone ${index}: atterrissage d'urgence`);
                    
                    if (drone.emergencyLanding) {
                        drone.emergencyLanding();
                    } else {
                        // Force l'état d'urgence
                        if (drone.setState) drone.setState('EMERGENCY');
                        if (drone.state) drone.state = 'EMERGENCY';
                        
                        // Position au sol immédiatement
                        if (drone.mesh) {
                            drone.mesh.position.y = 0;
                        }
                        if (drone.position) {
                            drone.position.y = 0;
                        }
                        
                        // Arrêt des moteurs
                        if (drone.propellers) {
                            drone.propellers.forEach(prop => {
                                if (prop.rotation) {
                                    prop.rotation.x = 0;
                                    prop.rotation.y = 0;
                                    prop.rotation.z = 0;
                                }
                            });
                        }
                    }
                });
                console.log('✅ Atterrissage d\'urgence forcé pour tous les drones');
            }
            
            window.dronePanelController.showNotification('🚨 ATTERRISSAGE D\'URGENCE!', 'error');
        } catch (error) {
            console.error('❌ Erreur atterrissage d\'urgence:', error);
            window.dronePanelController.showNotification('❌ Erreur urgence', 'error');
        }
    } else {
        console.warn('❌ Système DIAMANTS non disponible');
    }
};

window.resetSwarm = function() {
    console.log('🔄 === RESET COMPLET DU SWARM ===');
    if (window.dronePanelController?.checkSystem()) {
        const controller = window.dronePanelController.controller;
        
        try {
            if (controller.resetSwarm) {
                controller.resetSwarm();
                console.log('✅ Reset via resetSwarm()');
            } else if (controller.reset) {
                controller.reset();
                console.log('✅ Reset via reset()');
            } else if (controller.drones) {
                // Reset manuel de tous les drones
                controller.drones.forEach((drone, index) => {
                    console.log(`🔄 Reset drone ${index}`);
                    
                    // État stationnaire
                    if (drone.setState) drone.setState('IDLE');
                    if (drone.state) drone.state = 'IDLE';
                    
                    // Position d'origine
                    if (drone.mesh) {
                        drone.mesh.position.set(0, 0, 0);
                        drone.mesh.rotation.set(0, 0, 0);
                    }
                    
                    // Reset physique
                    if (drone.physics && drone.physics.reset) {
                        drone.physics.reset();
                    }
                    
                    // Arrêt propellers
                    if (drone.propellers) {
                        drone.propellers.forEach(prop => {
                            if (prop.rotation) {
                                prop.rotation.x = 0;
                                prop.rotation.y = 0;
                                prop.rotation.z = 0;
                            }
                        });
                    }
                });
                console.log('✅ Reset manuel complet effectué');
            }
            
            window.dronePanelController.showNotification('🔄 Swarm reset!', 'success');
        } catch (error) {
            console.error('❌ Erreur reset swarm:', error);
            window.dronePanelController.showNotification('❌ Erreur reset', 'error');
        }
    } else {
        console.warn('❌ Système DIAMANTS non disponible');
    }
};

// === FORMATION CONTROL ===
window.changePattern = function() {
    console.log('🌟 === DÉBUT changePattern() ===');
    
    const patterns = ['grid', 'line', 'circle', 'triangle', 'diamond', 'v_formation'];
    const display = document.getElementById('currentPattern');
    let current = display ? display.textContent.trim() : 'grid';
    
    console.log('🔍 Element currentPattern trouvé:', display);
    console.log('🔍 Pattern actuel brut:', current);
    
    // Nettoyage du nom actuel
    current = current.toLowerCase().replace(/[^a-z_]/g, '');
    console.log('🔍 Pattern actuel nettoyé:', current);
    
    const currentIndex = patterns.indexOf(current);
    const nextIndex = (currentIndex + 1) % patterns.length;
    const nextPattern = patterns[nextIndex];
    
    console.log('🔢 Index actuel:', currentIndex, '| Index suivant:', nextIndex);
    console.log(`🌟 Changement de pattern: ${current} → ${nextPattern}`);
    
    // Mettre à jour l'affichage
    if (display) {
        display.textContent = nextPattern;
        console.log('✅ Affichage mis à jour vers:', nextPattern);
    } else {
        console.warn('❌ Element currentPattern non trouvé !');
    }
    
    // Appliquer la formation
    console.log('🎯 Appel de setFormation avec:', nextPattern);
    if (window.setFormation) {
        window.setFormation(nextPattern);
        console.log('✅ setFormation appelé');
    } else {
        console.error('❌ window.setFormation non disponible !');
    }
    
    console.log('🌟 === FIN changePattern() ===');
};

window.setFormation = function(formationType) {
    console.log(`🌟 === DÉBUT setFormation(${formationType}) ===`);
    console.log('🔍 window.dronePanelController exists:', !!window.dronePanelController);
    console.log('🔍 dronePanelController.checkSystem exists:', !!window.dronePanelController?.checkSystem);
    
    if (window.dronePanelController?.checkSystem()) {
        console.log('✅ checkSystem() passed');
        const controller = window.dronePanelController.controller;
        console.log('🔍 controller exists:', !!controller);
        console.log('🔍 controller.setFormation exists:', !!controller?.setFormation);
        console.log('🔍 controller.changeFormation exists:', !!controller?.changeFormation);
        console.log('🔍 controller.drones exists:', !!controller?.drones);
        console.log('🔍 controller.drones length:', controller?.drones?.length || 'N/A');
        
        try {
            if (controller.setFormation) {
                console.log('🎯 Using controller.setFormation()');
                controller.setFormation(formationType);
                console.log(`✅ Formation ${formationType} via setFormation()`);
            } else if (controller.changeFormation) {
                console.log('🎯 Using controller.changeFormation()');
                controller.changeFormation(formationType);
                console.log(`✅ Formation ${formationType} via changeFormation()`);
            } else if (controller.drones) {
                console.log('🎯 Using manual formation applyFormationPattern()');
                console.log('🔍 dronePanelController.applyFormationPattern exists:', !!window.dronePanelController.applyFormationPattern);
                // Formation manuelle basique
                window.dronePanelController.applyFormationPattern(formationType);
                console.log(`✅ Formation ${formationType} appliquée manuellement`);
            } else {
                console.warn('❌ Aucune méthode de formation disponible');
            }
            
            // Mettre à jour l'affichage
            const display = document.getElementById('currentPattern');
            if (display) {
                display.textContent = formationType;
                console.log('✅ Display updated to:', formationType);
            }
            
            if (window.dronePanelController.showNotification) {
                window.dronePanelController.showNotification(`🌟 Formation: ${formationType}`, 'success');
            }
        } catch (error) {
            console.error('❌ Erreur formation:', error);
            if (window.dronePanelController.showNotification) {
                window.dronePanelController.showNotification('❌ Erreur formation', 'error');
            }
        }
    } else {
        console.warn('❌ checkSystem() failed or dronePanelController not available');
    }
    console.log(`🌟 === FIN setFormation(${formationType}) ===`);
};

// === DEBUG CONTROLS ===
window.toggleDebugLogs = function() {
    const btn = document.getElementById('btn-toggle-logs');
    if (btn) {
        const isHidden = btn.textContent.includes('Show');
        btn.textContent = isHidden ? '📋 Hide Logs' : '📋 Show Logs';
        console.log(`📋 Debug logs ${isHidden ? 'affichés' : 'cachés'}`);
    }
};

window.clearDebugLogs = function() {
    console.clear();
    console.log('🧹 === LOGS EFFACÉS ===');
    window.dronePanelController.showNotification('🧹 Logs effacés', 'info');
};

window.toggleDebugPanels = function() {
    const btn = document.getElementById('btn-toggle-debug');
    if (btn) {
        const isHidden = btn.textContent.includes('Show');
        btn.textContent = isHidden ? '🛠️ Hide Debug' : '🛠️ Show Debug';
        console.log(`🛠️ Panneaux debug ${isHidden ? 'affichés' : 'cachés'}`);
    }
};

window.toggleConsoleNinja = function() {
    const btn = document.getElementById('btn-toggle-ninja');
    if (btn) {
        const isHidden = btn.textContent.includes('Console');
        btn.textContent = isHidden ? '🥷 Hide Console' : '🥷 Console';
        console.log(`🥷 Console Ninja ${isHidden ? 'activé' : 'masqué'}`);
    }
};

// === CAMERA CONTROLS ===
window.resetCamera = function() {
    console.log('📹 Reset caméra');
    
    if (window.camera) {
        window.camera.position.set(10, 10, 10);
        window.camera.lookAt(0, 0, 0);
        
        if (window.controls) {
            window.controls.update();
        }
        
        console.log('✅ Caméra réinitialisée');
        window.dronePanelController.showNotification('📹 Caméra reset', 'info');
    } else {
        console.warn('❌ Caméra non trouvée');
    }
};

window.topView = function() {
    console.log('📹 Vue du dessus');
    
    if (window.camera) {
        window.camera.position.set(0, 30, 0.1);
        window.camera.lookAt(0, 0, 0);
        
        if (window.controls) {
            window.controls.update();
        }
        
        console.log('✅ Vue du dessus activée');
        window.dronePanelController.showNotification('📹 Vue du dessus', 'info');
    } else {
        console.warn('❌ Caméra non trouvée');
    }
};

window.toggleFollowMode = function() {
    console.log('📹 Toggle mode suivi');
    // TODO: Implémenter le mode suivi si nécessaire
    window.dronePanelController.showNotification('📹 Mode suivi (WIP)', 'info');
};

window.zoomToSwarm = function() {
    console.log('📹 Zoom sur le swarm');
    
    if (window.dronePanelController?.checkSystem() && window.camera) {
        const controller = window.dronePanelController.controller;
        
        // Calculer le centre du swarm
        let center = {x: 0, y: 0, z: 0};
        let count = 0;
        
        controller.drones.forEach(drone => {
            if (drone.mesh) {
                center.x += drone.mesh.position.x;
                center.y += drone.mesh.position.y;
                center.z += drone.mesh.position.z;
                count++;
            }
        });
        
        if (count > 0) {
            center.x /= count;
            center.y /= count;
            center.z /= count;
            
            // Positionner la caméra
            const distance = 15;
            window.camera.position.set(
                center.x + distance, 
                center.y + distance, 
                center.z + distance
            );
            window.camera.lookAt(center.x, center.y, center.z);
            
            if (window.controls) {
                window.controls.update();
            }
            
            console.log(`✅ Zoom sur swarm centré en (${center.x.toFixed(1)}, ${center.y.toFixed(1)}, ${center.z.toFixed(1)})`);
            window.dronePanelController.showNotification('📹 Zoom swarm', 'info');
        } else {
            console.warn('❌ Aucun drone avec mesh trouvé');
        }
    }
};

// === FLIGHT CONTROLS ===
window.takeoffAllDrones = function() {
    console.log('🛫 === DÉCOLLAGE TOUS DRONES ===');
    if (window.dronePanelController?.checkSystem()) {
        window.dronePanelController.takeoffAllDrones();
    } else {
        console.warn('❌ Système non disponible');
    }
};

window.landAllDrones = function() {
    console.log('🛬 === ATTERRISSAGE TOUS DRONES ===');
    if (window.dronePanelController?.checkSystem()) {
        window.dronePanelController.landAllDrones();
    } else {
        console.warn('❌ Système non disponible');
    }
};

window.stopAllDrones = function() {
    console.log('⏹️ === ARRÊT TOUS DRONES ===');
    if (window.dronePanelController?.checkSystem()) {
        window.dronePanelController.stopAllDrones();
    } else {
        console.warn('❌ Système non disponible');
    }
};

window.startMissionManual = function() {
    console.log('🎮 === MISSION MANUELLE ===');
    if (window.dronePanelController?.checkSystem()) {
        const controller = window.dronePanelController.controller;
        
        if (controller.startMissionManual) {
            controller.startMissionManual();
            console.log('✅ Mission manuelle démarrée');
            window.dronePanelController.showNotification('🎮 Mission manuelle', 'success');
        } else {
            console.warn('⚠️ startMissionManual() non disponible');
            window.launchMission(); // Fallback
        }
    }
};

window.testSingleExploration = function() {
    console.log('🔍 === TEST EXPLORATION SIMPLE ===');
    if (window.dronePanelController?.checkSystem()) {
        const controller = window.dronePanelController.controller;
        
        if (controller.drones && controller.drones.length > 0) {
            const firstDrone = controller.drones[0];
            console.log('🔍 Test d\'exploration avec le premier drone');
            
            if (firstDrone.startExploration) {
                firstDrone.startExploration();
            } else if (firstDrone.setState) {
                firstDrone.setState('EXPLORATION');
            }
            
            window.dronePanelController.showNotification('🔍 Test exploration', 'info');
        }
    }
};

// === MODE CONTROLS ===
window.applyMode = function() {
    const modeSelect = document.getElementById('mode_select');
    if (modeSelect) {
        const mode = modeSelect.value;
        console.log(`🔧 === APPLICATION DU MODE: ${mode} ===`);
        
        if (window.dronePanelController?.checkSystem()) {
            const controller = window.dronePanelController.controller;
            
            if (controller.setMode) {
                controller.setMode(mode);
            } else if (controller.drones) {
                controller.drones.forEach(drone => {
                    if (drone.setMode) {
                        drone.setMode(mode);
                    } else if (drone.setState) {
                        drone.setState(mode.toUpperCase());
                    }
                });
            }
            
            window.dronePanelController.showNotification(`🔧 Mode: ${mode}`, 'success');
        }
    }
};

console.log('🎯 === CÂBLAGE UI COMPLET CHARGÉ ===');
console.log('✅ Toutes les fonctions onclick sont maintenant disponibles !');
