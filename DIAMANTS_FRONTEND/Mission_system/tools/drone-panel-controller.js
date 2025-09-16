/**
 * DIAMANTS - Drone Panel Controller
 * ================================
 * Contrôleur des panneaux UI pour la gestion des drones
 * Cache bust: 2025-09-13-14:58
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

// Mode silencieux global - utilisation de window pour éviter les conflits
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Utiliser les fonctions globales définies dans index.html

class DronePanelController {
    constructor() {
        this.initialized = false;
        this.controller = null;
        this.app = null;
        this.waitForSystem();
    }

    waitForSystem() {
        log('⏳ Attente du système DIAMANTS...');
        
        let attempts = 0;
        const maxAttempts = 60; // 30 secondes max
        
        const checkInterval = setInterval(() => {
            attempts++;
            
                        // Log détaillé pour debug
            if (attempts % 5 === 1) { // Log toutes les 2.5 secondes
                log(`🔍 Recherche du système... (tentative ${attempts}/${maxAttempts})`);
                log('  window.DIAMANTS:', window.DIAMANTS ? '✅ existe' : '❌ non trouvé');
                log('  window.engineInstance:', window.engineInstance ? '✅ existe' : '❌ non trouvé');
                log('  window.drones:', window.drones ? `✅ ${window.drones.length} drones` : '❌ non trouvé');
                
                if (window.DIAMANTS) {
                    log('  Structure DIAMANTS:', Object.keys(window.DIAMANTS));
                    log('  DIAMANTS.controller:', window.DIAMANTS.controller ? '✅' : '❌');
                }
            }
            
            // Méthode 1: Chercher via window.DIAMANTS.controller (API principale)
            if (window.DIAMANTS && window.DIAMANTS.controller) {
                log('✅ Contrôleur trouvé via window.DIAMANTS.controller');
                clearInterval(checkInterval);
                this.controller = window.DIAMANTS.controller;
                this.app = window.DIAMANTS;
                this.initialize();
                return;
            }
            
            // Méthode 2: Chercher via window.engineInstance (IntegratedController direct)
            if (window.engineInstance && window.engineInstance.drones) {
                log('✅ Contrôleur trouvé via window.engineInstance');
                clearInterval(checkInterval);
                this.controller = window.engineInstance;
                this.app = { drones: window.drones };
                this.initialize();
                return;
            }
            
            // Méthode 3: Chercher via window.diamantsSystem.integratedController (fallback)
            if (window.diamantsSystem && window.diamantsSystem.integratedController) {
                log('✅ Contrôleur trouvé via window.diamantsSystem.integratedController');
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
                    log('🎯 Drones trouvés directement dans diamantsApp');
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
                log('💡 Conseil: Vérifiez que main.js expose bien le contrôleur via window.DIAMANTS.controller');
                log('💡 Structures disponibles:');
                log('   - window.diamantsSystem:', !!window.diamantsSystem);
                log('   - window.DIAMANTS:', !!window.DIAMANTS);
                log('   - window.diamantsApp:', !!window.diamantsApp);
            }
        }, 500);
    }

    initialize() {
        log('✅ Contrôleur intégré trouvé !', this.controller);
        
        // Vérifier que le contrôleur a les bonnes méthodes
        if (!this.controller.drones) {
            console.error('❌ Le contrôleur n\'a pas de propriété drones');
            return;
        }
        
        log(`🚁 ${this.controller.drones.length} drones trouvés dans le contrôleur`);
        
        this.exposeFunctions();
        this.initialized = true;
        this.updateUI();
        
        // Log des méthodes disponibles
        log('📋 Méthodes disponibles sur le contrôleur:');
        const methods = Object.getOwnPropertyNames(Object.getPrototypeOf(this.controller))
            .filter(name => typeof this.controller[name] === 'function' && name !== 'constructor');
        methods.forEach(method => log(`  - ${method}()`));
    }

    initializeWithDronesOnly() {
        log('⚠️ Pas de contrôleur trouvé, utilisation directe des drones');
        
        // Créer un contrôleur minimal
        this.controller = {
            drones: window.diamantsApp.drones || [],
            
            // Méthodes minimales basées sur les drones directement
            takeoffAllDrones: async function() {
                log('🚁 Décollage direct des drones...');
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
                        log(`Drone ${index} décollage initié`);
                    }, index * 300);
                });
            },
            
            landAllDrones: async function() {
                log('🛬 Atterrissage direct des drones...');
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
                log('🚨 Arrêt d\'urgence!');
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
                log('🎯 Mission manuelle (simulation)');
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
        
        log('✅ Mode dégradé activé - Contrôle direct des drones');
    }

    exposeFunctions() {
        // Fonctions de vol - utiliser les méthodes du contrôleur intégré
        window.takeoffAllDrones = async () => {
            log('🛫 Commande décollage reçue!');
            if (this.checkSystem()) {
                try {
                    this.showNotification('🛫 Décollage en cours...', 'info');
                    
                    // Appel prioritaire vers le contrôleur intégré
                    if (this.controller.takeoffAllDrones) {
                        await this.controller.takeoffAllDrones();
                        this.showNotification('✅ Décollage terminé', 'success');
                    } else if (this.controller.drones && this.controller.drones.length > 0) {
                        // Fallback: animation manuelle des drones
                        log('🔄 Animation manuelle du décollage...');
                        this.controller.drones.forEach((drone, index) => {
                            if (drone.mesh) {
                                const targetAltitude = 5.0 + (index * 0.8);
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
            log('🛬 Commande atterrissage reçue!');
            if (this.checkSystem()) {
                try {
                    this.showNotification('🛬 Atterrissage en cours...', 'info');
                    
                    if (this.controller.landAllDrones) {
                        await this.controller.landAllDrones();
                        this.showNotification('✅ Atterrissage terminé', 'success');
                    } else if (this.controller.drones && this.controller.drones.length > 0) {
                        // Fallback: animation manuelle
                        log('🔄 Animation manuelle de l\'atterrissage...');
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
            log('🚨 ARRÊT D\'URGENCE ACTIVÉ!');
            if (this.checkSystem()) {
                try {
                    // Arrêt immédiat des moteurs pour tous les drones
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
                } catch (err) {
                    console.error('❌ Erreur arrêt d\'urgence:', err);
                    // Force cache refresh 13-09-2025 14:52
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
        
        // Fonctions de mission
        window.launchMission = () => {
            if (this.checkSystem()) {
                this.controller.startMissionManual();
                this.showNotification('🚀 Mission lancée', 'success');
            }
        };
        
        window.startMissionManual = () => {
            if (this.checkSystem()) {
                this.controller.startMissionManual();
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
            log(`📐 Formation ${type} demandée`);
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
                        warn(`⚠️ Formation '${type}' non reconnue`);
                        this.showNotification(`❌ Formation '${type}' inconnue`, 'error');
                    }
                } catch (error) {
                    console.error('❌ Erreur formation:', error);
                    this.showNotification('❌ Erreur formation', 'error');
                }
            }
        };
        
        // Fonctions globales pour test des boundary boxes
        window.testAllBoundaryBoxes = () => {
            console.log('🔍 TEST BOUNDARY BOXES - Début diagnostic');
            
            if (this.controller && this.controller.drones) {
                this.controller.drones.forEach(drone => {
                    if (drone.ensureBoundaryBoxes) {
                        console.log(`🔧 ${drone.id}: Test boundary boxes...`);
                        drone.ensureBoundaryBoxes();
                    } else {
                        console.log(`❌ ${drone.id}: Pas de méthode ensureBoundaryBoxes`);
                    }
                });
            } else {
                console.log('❌ Pas de drones trouvés dans le contrôleur');
            }
        };

        window.changePattern = () => {
            console.log('🔍 PATTERN BUTTON CLICKED - Test boundary boxes');
            const patterns = ['grid', 'line', 'circle', 'triangle'];
            const current = document.getElementById('currentPattern')?.textContent || 'grid';
            const index = patterns.indexOf(current);
            const next = patterns[(index + 1) % patterns.length];
            
            // FORCER LE DÉCOLLAGE À 10M AVANT LA FORMATION
            log(`🚁 PATTERN BUTTON: Forçage décollage à 10m pour tous les drones`);
            
            if (this.controller && this.controller.drones) {
                this.controller.drones.forEach(drone => {
                    // S'assurer que les boundary boxes sont créées
                    if (drone.ensureBoundaryBoxes) {
                        drone.ensureBoundaryBoxes();
                    }
                    
                    // Forcer l'état de décollage
                    drone.state = 'TAKEOFF';
                    // S'assurer qu'on a une position cible à 10m
                    if (!drone.targetPosition) {
                        drone.targetPosition = new THREE.Vector3();
                    }
                    drone.targetPosition.set(drone.position.x, 10.0, drone.position.z);
                    log(`🚁 ${drone.id}: DÉCOLLAGE FORCÉ vers 10m - État=${drone.state}`);
                });
            }
            
            // Appliquer la formation après un délai pour le décollage
            setTimeout(() => {
                window.setFormation(next);
            }, 1000); // 1 seconde de délai
            
            const display = document.getElementById('currentPattern');
            if (display) display.textContent = next;
        };
        
        // COPIE EXACTE DE changePattern pour launchMission
        window.launchMission = () => {
            console.log('🚀 LAUNCH MISSION CLICKED - Décollage stationnaire');
            
            // FORCER LE VOL STATIONNAIRE À 15M
            console.log(`🚁 LAUNCH MISSION: Vol stationnaire à 15m pour tous les drones`);
            
            // Essayer plusieurs sources de drones
            let drones = null;
            if (window.diamantsSystem?.integratedController?.drones) {
                drones = window.diamantsSystem.integratedController.drones;
                console.log(`✅ Drones trouvés via diamantsSystem: ${drones.length}`);
            } else if (window.drones) {
                drones = window.drones;
                console.log(`✅ Drones trouvés via window.drones: ${drones.length}`);
            } else if (this.controller && this.controller.drones) {
                drones = this.controller.drones;
                console.log(`✅ Drones trouvés via this.controller: ${drones.length}`);
            } else {
                console.log(`❌ Aucun drone trouvé dans aucune source`);
                console.log(`  - window.diamantsSystem:`, !!window.diamantsSystem);
                console.log(`  - window.drones:`, !!window.drones);
                console.log(`  - this.controller:`, !!this.controller);
                return;
            }
            
            if (drones && drones.length > 0) {
                console.log(`🔄 Traitement de ${drones.length} drones pour vol stationnaire à 15m`);
                drones.forEach((drone, index) => {
                    console.log(`🚁 Drone ${index + 1}/${drones.length} - ID: ${drone.id || 'sans-id'}`);
                    
                    // S'assurer que les boundary boxes sont créées
                    if (drone.ensureBoundaryBoxes) {
                        drone.ensureBoundaryBoxes();
                        console.log(`✅ Boundary boxes créées pour drone ${drone.id}`);
                    }
                    
                    // Récupérer la position actuelle pour rester stationnaire (comme Pattern)
                    const currentX = drone.position.x;
                    const currentY = drone.position.y;
                    const currentZ = drone.position.z;
                    console.log(`📍 Position actuelle drone ${drone.id}: X=${currentX.toFixed(2)}, Y=${currentY.toFixed(2)}, Z=${currentZ.toFixed(2)}`);
                    
                    // Forcer l'état de décollage comme Pattern button
                    drone.state = 'TAKEOFF';
                    // S'assurer qu'on a une position cible à 15m d'altitude (Y dans Three.js) MAIS à la position actuelle en X,Z
                    if (!drone.targetPosition) {
                        drone.targetPosition = new THREE.Vector3();
                    }
                    drone.targetPosition.set(currentX, 15.0, currentZ);
                    
                    console.log(`🚁 ${drone.id}: DÉCOLLAGE STATIONNAIRE vers [${currentX.toFixed(2)}, 15.0, ${currentZ.toFixed(2)}] - État=${drone.state}`);
                });
            }
            
            // PAS de formation automatique - juste vol stationnaire à 15m
            console.log(`✅ LAUNCH MISSION: Tous les drones en vol stationnaire à 15m`);
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
        window.stopMission = () => log('Mission stoppée');
        window.selectDroneToFollow = () => log('Drone sélectionné');
        window.updateSafetyDistance = (val) => log('Distance:', val);
        window.updateAltitudeDisplay = (val) => log('Altitude:', val);
        window.applyMode = () => log('Mode appliqué');
        window.toggleDebugLogs = () => log('Debug logs toggle');
        window.clearDebugLogs = () => log('Logs cleared');
        window.toggleDebugPanels = () => log('Debug panels toggle');
        window.toggleConsoleNinja = () => log('Console Ninja toggle');
        
        // Fonctions de missions stub
        window.startExplorationMission = () => log('Mission exploration');
        window.startMappingMission = () => log('Mission mapping');
        window.startSurveillanceMission = () => log('Mission surveillance');
        window.animateDroneToPosition = () => {};
        window.animateExplorationPattern = () => {};
        window.createTakeoffEffect = () => {};
        window.flashDroneLight = () => {};
        
        // Log pour confirmer que les fonctions sont exposées
        log('🎮 Fonctions exposées globalement:');
        log('  - takeoffAllDrones:', typeof window.takeoffAllDrones);
        log('  - landAllDrones:', typeof window.landAllDrones);
        log('  - emergencyLand:', typeof window.emergencyLand);
    }

    // Formations
    createLineFormation() {
        const positions = [];
        const count = this.controller.drones.length;
        const spacing = 3;
        const FLIGHT_ALTITUDE = 12; // 12m d'altitude comme les autres formations
        
        for (let i = 0; i < count; i++) {
            positions.push({
                x: (i - count/2) * spacing,
                y: FLIGHT_ALTITUDE,
                z: 0
            });
        }
        log(`📏 Formation ligne: ${count} drones, espacement=${spacing}m, altitude=${FLIGHT_ALTITUDE}m`);
        return positions;
    }

    createCircleFormation() {
        const positions = [];
        const count = this.controller.drones.length;
        const radius = Math.max(5, count);
        const FLIGHT_ALTITUDE = 12; // 12m d'altitude (4m au-dessus de la plateforme de 8m)
        
        for (let i = 0; i < count; i++) {
            const angle = (i / count) * Math.PI * 2;
            positions.push({
                x: Math.cos(angle) * radius,
                y: FLIGHT_ALTITUDE,
                z: Math.sin(angle) * radius
            });
        }
        log(`🔵 Formation circulaire: ${count} drones, rayon=${radius}m, altitude=${FLIGHT_ALTITUDE}m`);
        return positions;
    }

    createTriangleFormation() {
        const positions = [];
        const count = this.controller.drones.length;
        let row = 0, col = 0, maxCols = 1;
        const FLIGHT_ALTITUDE = 12; // 12m d'altitude
        
        for (let i = 0; i < count; i++) {
            positions.push({
                x: (col - row/2) * 3,
                y: FLIGHT_ALTITUDE,
                z: row * 3
            });
            
            col++;
            if (col >= maxCols) {
                row++;
                col = 0;
                maxCols++;
            }
        }
        log(`🔺 Formation triangle: ${count} drones, altitude=${FLIGHT_ALTITUDE}m`);
        return positions;
    }

    createGridFormation() {
        const positions = [];
        const count = this.controller.drones.length;
        const gridSize = Math.ceil(Math.sqrt(count));
        const spacing = 3;
        const FLIGHT_ALTITUDE = 12; // 12m d'altitude
        
        for (let i = 0; i < count; i++) {
            const x = (i % gridSize - gridSize/2) * spacing;
            const z = (Math.floor(i / gridSize) - gridSize/2) * spacing;
            positions.push({x, y: FLIGHT_ALTITUDE, z});
        }
        log(`⬜ Formation grille: ${count} drones, ${gridSize}x${gridSize}, altitude=${FLIGHT_ALTITUDE}m`);
        return positions;
    }

    createLineFormation() {
        const positions = [];
        const count = this.controller.drones.length;
        const spacing = 4;
        const FLIGHT_ALTITUDE = 12; // 12m d'altitude
        
        for (let i = 0; i < count; i++) {
            positions.push({
                x: (i - count/2) * spacing,
                y: FLIGHT_ALTITUDE,
                z: 0
            });
        }
        log(`📏 Formation ligne: ${count} drones, espacement=${spacing}m, altitude=${FLIGHT_ALTITUDE}m`);
        return positions;
    }

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
        
        log(`🚁 Décollage réaliste du drone ${drone.id} vers ${targetAltitude}m`);
        
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
                log(`✅ Drone ${drone.id} décollé et stabilisé à ${targetAltitude}m`);
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
        
        log(`🛬 Atterrissage réaliste du drone ${drone.id}`);
        
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
                log(`✅ Drone ${drone.id} atterri en sécurité`);
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
            warn(`⚠️ Aucune hélice trouvée pour le drone ${drone.id}`);
            return;
        }
        
        log(`🌀 Démarrage rotation ${propellers.length} hélices pour drone ${drone.id}`);
        
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
        
        log(`🛑 Arrêt rotation hélices pour drone ${drone.id}`);
        
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
        
        log(`⚖️ Activation stabilisation pour drone ${drone.id}`);
        
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

    // Atterrissage d'urgence (chute contrôlée)
    emergencyLandDrone(drone) {
        if (!drone.mesh) return;
        
        const startY = drone.mesh.position.y;
        const targetY = 0;
        const duration = 1500; // 1.5 secondes (plus rapide qu'un atterrissage normal)
        const startTime = Date.now();
        
        log(`🚨 Atterrissage d'urgence drone ${drone.id}`);
        
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
                log(`💥 Drone ${drone.id} atterrissage d'urgence terminé`);
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

    showNotification(message, type = 'info') {
        log(`[${type.toUpperCase()}] ${message}`);
        
        let notification = document.getElementById('notification');
        if (!notification) {
            notification = document.createElement('div');
            notification.id = 'notification';
            notification.style.cssText = `
                position: fixed;
                top: 120px;
                right: 20px;
                padding: 10px 20px;
                border-radius: 5px;
                z-index: 10000;
                font-family: monospace;
                font-weight: bold;
                transition: opacity 0.3s;
                pointer-events: none;
                max-width: 300px;
                word-wrap: break-word;
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
    
    createLineFormation() {
        const droneCount = this.controller.drones.length;
        const spacing = 3.0; // 3 mètres entre chaque drone
        const startX = -(droneCount - 1) * spacing / 2;
        
        return this.controller.drones.map((drone, index) => ({
            x: startX + index * spacing,
            y: 5.0, // Altitude de vol
            z: 0
        }));
    }

    createCircleFormation() {
        const droneCount = this.controller.drones.length;
        const radius = Math.max(droneCount * 1.5, 8.0); // Rayon adaptatif
        
        return this.controller.drones.map((drone, index) => {
            const angle = (index / droneCount) * 2 * Math.PI;
            return {
                x: Math.cos(angle) * radius,
                y: 5.0,
                z: Math.sin(angle) * radius
            };
        });
    }

    createTriangleFormation() {
        const droneCount = this.controller.drones.length;
        const positions = [];
        
        if (droneCount === 1) {
            return [{ x: 0, y: 5.0, z: 0 }];
        }
        
        // Leader au front
        positions.push({ x: 0, y: 5.0, z: 8 });
        
        // Distribuer le reste en V
        for (let i = 1; i < droneCount; i++) {
            const side = i % 2 === 1 ? 1 : -1; // Alternance gauche/droite
            const layer = Math.floor((i - 1) / 2) + 1;
            positions.push({
                x: side * layer * 2.5,
                y: 5.0,
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
        
        const positions = [];
        for (let i = 0; i < droneCount; i++) {
            const row = Math.floor(i / cols);
            const col = i % cols;
            positions.push({
                x: (col - (cols - 1) / 2) * spacing,
                y: 5.0,
                z: (row - (rows - 1) / 2) * spacing
            });
        }
        
        return positions;
    }

    applyFormationSmoothly(positions, formationType) {
        log(`🎯 Application formation ${formationType} pour ${positions.length} drones avec PID authentique`);
        
        this.controller.drones.forEach((drone, index) => {
            if (!positions[index]) return;
            
            const target = positions[index];
            
            // Utiliser le vrai système PID du drone
            if (drone.setTargetPosition) {
                // Définir l'altitude de vol au-dessus de la plateforme (minimum 10m)
                const FLIGHT_ALTITUDE = Math.max(10.5, target.y);
                
                drone.setTargetPosition(target.x, FLIGHT_ALTITUDE, target.z);
                
                log(`🚁 ${drone.id} → cible PID: [${target.x.toFixed(1)}, ${FLIGHT_ALTITUDE.toFixed(1)}, ${target.z.toFixed(1)}]`);
            } else {
                warn(`❌ Drone ${drone.id} n'a pas de méthode setTargetPosition - utilisation fallback`);
                
                // Fallback pour anciens drones
                if (drone.mesh) {
                    const startPos = {
                        x: drone.mesh.position.x,
                        y: drone.mesh.position.y,
                        z: drone.mesh.position.z
                    };
                    
                    const duration = 5000;
                    const startTime = Date.now();
                    
                    if (drone.state === 'IDLE') {
                        this.startPropellerRotation(drone);
                        drone.state = 'FLYING';
                    }
                    
                    const animateToPosition = () => {
                        const elapsed = Date.now() - startTime;
                        const progress = Math.min(elapsed / duration, 1);
                        
                        const easeProgress = progress < 0.5 
                            ? 2 * progress * progress
                            : 1 - Math.pow(-2 * progress + 2, 2) / 2;
                        
                        drone.mesh.position.x = startPos.x + (target.x - startPos.x) * easeProgress;
                        drone.mesh.position.y = startPos.y + (target.y - startPos.y) * easeProgress;
                        drone.mesh.position.z = startPos.z + (target.z - startPos.z) * easeProgress;
                        
                        const deltaX = target.x - startPos.x;
                        const deltaZ = target.z - startPos.z;
                        if (Math.abs(deltaX) > 0.1 || Math.abs(deltaZ) > 0.1) {
                            const targetRotation = Math.atan2(deltaX, deltaZ);
                            drone.mesh.rotation.y = targetRotation * easeProgress;
                        }
                        
                        if (progress < 1) {
                            requestAnimationFrame(animateToPosition);
                        }
                    };
                    
                    animateToPosition();
                }
            }
        });
        
        // Notification de formation appliquée
        setTimeout(() => {
            this.showNotification(`✅ Formation ${formationType} appliquée avec contrôle PID`, 'success');
        }, 1000);
    }
}

// Initialiser le contrôleur au chargement
window.dronePanelController = new DronePanelController();

// Exposer une fonction de debug globale
window.debugDroneSystem = function() {
    log('=== DEBUG DRONE SYSTEM ===');
    log('dronePanelController:', window.dronePanelController);
    log('  initialized:', window.dronePanelController?.initialized);
    log('  controller:', window.dronePanelController?.controller);
    log('diamantsApp:', window.diamantsApp);
    if (window.diamantsApp) {
        log('  Structure:', Object.keys(window.diamantsApp));
        if (window.diamantsApp.drones) {
            log('  Drones:', window.diamantsApp.drones.length);
            window.diamantsApp.drones.forEach((drone, i) => {
                log(`    Drone ${i}: id=${drone.id}, state=${drone.state}, scene=${!!drone.scene}`);
                log(`      position=[${drone.position.x.toFixed(1)}, ${drone.position.y.toFixed(1)}, ${drone.position.z.toFixed(1)}]`);
                log(`      collisionBox=${!!drone.collisionBox}, safetyZone=${!!drone.safetyZone}`);
            });
        }
    }
};

// FONCTION DEBUG POUR FORCER LES VISUELS
window.debugForceVisuals = function() {
    log('🔧 Forçage des éléments visuels...');
    if (window.diamantsApp && window.diamantsApp.drones) {
        window.diamantsApp.drones.forEach(drone => {
            if (drone.forceCreateVisuals) {
                drone.forceCreateVisuals();
            }
        });
    }
};

// FONCTION DEBUG POUR FORCER LE DÉCOLLAGE
window.debugForceTakeoff = function() {
    log('🚁 Forçage du décollage...');
    if (window.diamantsApp && window.diamantsApp.drones) {
        window.diamantsApp.drones.forEach(drone => {
            drone.state = 'TAKEOFF';
            log(`🚁 ${drone.id} : État forcé en TAKEOFF`);
        });
    }
};

log('✅ drone-panel-controller.js chargé');
log('🔍 Recherche du système DIAMANTS en cours...');

log('💡 Tapez debugDroneSystem() dans la console pour diagnostiquer');
log('💡 Tapez debugForceVisuals() pour forcer les bounding boxes');
log('💡 Tapez debugForceTakeoff() pour forcer le décollage');
