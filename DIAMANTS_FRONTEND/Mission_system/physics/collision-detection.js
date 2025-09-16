/**
 * DIAMANTS V3 - Système de Détection de Collisions
 * ================================================
 * Détection physique optimisée avec boundary boxes intelligents
 */

// Mode silencieux pour les logs de debug
const SILENT_MODE = true; // Régler à false pour debug
const debugLog = SILENT_MODE ? () => {} : (...args) => console.warn(...args);

// Gestion des importations et dépendances

import * as THREE from 'three';

export class CollisionDetection {
    constructor() {
        this.platformBounds = null;
        this.droneBounds = new Map();
        this.debugMode = false; // Démarrer avec le debug désactivé
        this.debugHelpers = [];
        this.scene = null;
        
        // Configuration de la plateforme
        this.PLATFORM_HEIGHT = 2.0;
        this.PLATFORM_SIZE = { width: 50, depth: 50 };
        this.PLATFORM_POSITION = { x: 0, y: 0, z: 0 };
        
        // Configuration des drones
        this.DRONE_SIZE = { width: 2.0, height: 1.0, depth: 2.0 };
        this.COLLISION_MARGIN = 0.5; // Marge de sécurité
        
        log('🔧 Système de collision initialisé');
    }

    /**
     * Initialise le système avec la scène THREE.js
     */
    initialize(scene) {
        this.scene = scene;
        this.createPlatformBounds();
        log('✅ Système de collision prêt');
    }

    /**
     * Crée les bounding box de la plateforme
     */
    createPlatformBounds() {
        const platformBox = new THREE.Box3();
        platformBox.setFromCenterAndSize(
            new THREE.Vector3(
                this.PLATFORM_POSITION.x,
                this.PLATFORM_POSITION.y + this.PLATFORM_HEIGHT / 2,
                this.PLATFORM_POSITION.z
            ),
            new THREE.Vector3(
                this.PLATFORM_SIZE.width,
                this.PLATFORM_HEIGHT,
                this.PLATFORM_SIZE.depth
            )
        );
        
        this.platformBounds = platformBox;
        
        if (this.debugMode && this.scene) {
            this.createDebugHelper(platformBox, 0xff0000, 'Platform');
        }
        
        log('🏗️ Bounding box plateforme créée:', platformBox.min, platformBox.max);
        log(`📏 Hauteur plateforme: ${this.PLATFORM_HEIGHT}m`);
    }

    /**
     * Met à jour la configuration de la plateforme
     */
    updatePlatformConfig(height, radius = null) {
        this.PLATFORM_HEIGHT = height;
        if (radius) {
            this.PLATFORM_SIZE.width = radius * 2;
            this.PLATFORM_SIZE.depth = radius * 2;
        }
        
        // Supprimer l'ancien helper de debug si il existe
        if (this.scene) {
            const oldHelper = this.scene.getObjectByName('CollisionHelper-Platform');
            if (oldHelper) {
                this.scene.remove(oldHelper);
                oldHelper.geometry.dispose();
                oldHelper.material.dispose();
            }
        }
        
        // Recréer les bounds avec la nouvelle configuration
        this.createPlatformBounds();
        log(`🔄 Configuration plateforme mise à jour: hauteur=${height}m, taille=${this.PLATFORM_SIZE.width}x${this.PLATFORM_SIZE.depth}`);
    }

    /**
     * Crée une bounding box pour un drone
     */
    createDroneBounds(droneId, position) {
        const droneBox = new THREE.Box3();
        droneBox.setFromCenterAndSize(
            position.clone(),
            new THREE.Vector3(
                this.DRONE_SIZE.width + this.COLLISION_MARGIN,
                this.DRONE_SIZE.height + this.COLLISION_MARGIN,
                this.DRONE_SIZE.depth + this.COLLISION_MARGIN
            )
        );
        
        this.droneBounds.set(droneId, droneBox);
        
        if (this.debugMode && this.scene) {
            this.createDebugHelper(droneBox, 0x00ff00, `Drone-${droneId}`);
        }
        
        return droneBox;
    }

    /**
     * Met à jour la position d'un drone et vérifie les collisions
     */
    updateDronePosition(droneId, newPosition) {
        // Créer ou mettre à jour la bounding box du drone
        const droneBox = this.droneBounds.get(droneId) || this.createDroneBounds(droneId, newPosition);
        
        // Mettre à jour la position de la bounding box
        droneBox.setFromCenterAndSize(
            newPosition.clone(),
            new THREE.Vector3(
                this.DRONE_SIZE.width + this.COLLISION_MARGIN,
                this.DRONE_SIZE.height + this.COLLISION_MARGIN,
                this.DRONE_SIZE.depth + this.COLLISION_MARGIN
            )
        );

        // Vérifier collision avec la plateforme
        const collisionResult = this.checkCollisions(droneId, newPosition);
        
        // Mettre à jour le debug helper si en mode debug
        if (this.debugMode) {
            this.updateDebugHelper(droneId, droneBox);
        }
        
        return collisionResult;
    }

    /**
     * Vérifie toutes les collisions pour un drone
     */
    checkCollisions(droneId, position) {
        const result = {
            hasCollision: false,
            withPlatform: false,
            withDrones: [],
            correctedPosition: position.clone(),
            collisionNormal: new THREE.Vector3()
        };

        const droneBox = this.droneBounds.get(droneId);
        if (!droneBox) return result;

        // 1. Vérifier collision avec la plateforme
        if (this.platformBounds && droneBox.intersectsBox(this.platformBounds)) {
            result.hasCollision = true;
            result.withPlatform = true;
            
            // Corriger la position pour éviter la traversée
            result.correctedPosition = this.correctPlatformCollision(position, droneBox);
            result.collisionNormal.set(0, 1, 0); // Normale vers le haut
            
            debugLog(`⚠️ Collision détectée: Drone ${droneId} vs Plateforme`);
        }

        // 2. Vérifier collisions avec autres drones
        for (const [otherDroneId, otherBox] of this.droneBounds) {
            if (otherDroneId !== droneId && droneBox.intersectsBox(otherBox)) {
                result.hasCollision = true;
                result.withDrones.push(otherDroneId);
                
                debugLog(`⚠️ Collision détectée: Drone ${droneId} vs Drone ${otherDroneId}`);
            }
        }

        return result;
    }

    /**
     * Corrige la position d'un drone en collision avec la plateforme
     */
    correctPlatformCollision(position, droneBox) {
        const correctedPos = position.clone();
        
        // Si le drone est en dessous de la plateforme, le remonter
        if (position.y < this.PLATFORM_POSITION.y + this.PLATFORM_HEIGHT + this.DRONE_SIZE.height / 2) {
            correctedPos.y = this.PLATFORM_POSITION.y + this.PLATFORM_HEIGHT + this.DRONE_SIZE.height / 2 + this.COLLISION_MARGIN;
            log(`🔧 Drone corrigé: altitude ${position.y.toFixed(2)}m -> ${correctedPos.y.toFixed(2)}m`);
        }
        
        return correctedPos;
    }

    /**
     * Vérifie si une position est valide (pas en collision)
     */
    isPositionValid(droneId, position) {
        const tempBox = new THREE.Box3();
        tempBox.setFromCenterAndSize(
            position,
            new THREE.Vector3(
                this.DRONE_SIZE.width + this.COLLISION_MARGIN,
                this.DRONE_SIZE.height + this.COLLISION_MARGIN,
                this.DRONE_SIZE.depth + this.COLLISION_MARGIN
            )
        );

        // Vérifier avec la plateforme
        if (this.platformBounds && tempBox.intersectsBox(this.platformBounds)) {
            return false;
        }

        // Vérifier avec autres drones
        for (const [otherDroneId, otherBox] of this.droneBounds) {
            if (otherDroneId !== droneId && tempBox.intersectsBox(otherBox)) {
                return false;
            }
        }

        return true;
    }

    /**
     * Trouve une position valide proche de la position désirée
     */
    findValidPosition(droneId, desiredPosition, maxAttempts = 10) {
        if (this.isPositionValid(droneId, desiredPosition)) {
            return desiredPosition.clone();
        }

        // Essayer des positions alternatives
        for (let attempt = 0; attempt < maxAttempts; attempt++) {
            const offset = new THREE.Vector3(
                (Math.random() - 0.5) * 4,
                Math.random() * 2 + 1,
                (Math.random() - 0.5) * 4
            );
            
            const testPosition = desiredPosition.clone().add(offset);
            
            if (this.isPositionValid(droneId, testPosition)) {
                log(`✅ Position valide trouvée pour drone ${droneId} après ${attempt + 1} tentatives`);
                return testPosition;
            }
        }

        // Fallback: position de sécurité au-dessus de la plateforme
        const safePosition = new THREE.Vector3(
            desiredPosition.x,
            this.PLATFORM_POSITION.y + this.PLATFORM_HEIGHT + 5,
            desiredPosition.z
        );
        
        debugLog(`⚠️ Position de sécurité utilisée pour drone ${droneId}`);
        return safePosition;
    }

    /**
     * Crée un helper visuel pour debug
     */
    createDebugHelper(box, color, name) {
        if (!this.scene) return;

        const size = new THREE.Vector3();
        const center = new THREE.Vector3();
        box.getSize(size);
        box.getCenter(center);

        const geometry = new THREE.BoxGeometry(size.x, size.y, size.z);
        const material = new THREE.MeshBasicMaterial({
            color: color,
            wireframe: true,
            transparent: true,
            opacity: 0.3
        });
        
        const helper = new THREE.Mesh(geometry, material);
        helper.position.copy(center);
        helper.name = `CollisionHelper-${name}`;
        
        this.scene.add(helper);
        this.debugHelpers.push(helper);
        
        log(`🔍 Debug helper créé pour ${name}`);
    }

    /**
     * Met à jour un helper de debug
     */
    updateDebugHelper(droneId, box) {
        if (!this.scene) return;

        const helperName = `CollisionHelper-Drone-${droneId}`;
        const helper = this.scene.getObjectByName(helperName);
        
        if (helper) {
            const center = new THREE.Vector3();
            const size = new THREE.Vector3();
            box.getCenter(center);
            box.getSize(size);
            
            // Mettre à jour la position
            helper.position.copy(center);
            
            // Mettre à jour la taille si nécessaire
            helper.scale.set(
                size.x / helper.geometry.parameters.width,
                size.y / helper.geometry.parameters.height,
                size.z / helper.geometry.parameters.depth
            );
        }
    }

    /**
     * Supprime un drone du système de collision
     */
    removeDrone(droneId) {
        this.droneBounds.delete(droneId);
        
        if (this.scene) {
            const helperName = `CollisionHelper-Drone-${droneId}`;
            const helper = this.scene.getObjectByName(helperName);
            if (helper) {
                this.scene.remove(helper);
                helper.geometry.dispose();
                helper.material.dispose();
            }
        }
        
        log(`🗑️ Drone ${droneId} supprimé du système de collision`);
    }

    /**
     * Active/désactive le mode debug
     */
    setDebugMode(enabled) {
        this.debugMode = enabled;
                debugLog(`🔧 setDebugMode appelé avec: ${enabled}`);
        
        // Fonction pour chercher et traiter les drones avec retry automatique
        const processDonesWithRetry = (attempt = 1, maxAttempts = 5) => {
            debugLog(`🔍 Tentative ${attempt}/${maxAttempts} de recherche des drones...`);
            
            // Chercher les drones dans tous les endroits possibles
            let dronesArray = [];
            if (window.drones && window.drones.length > 0) {
                dronesArray = window.drones;
                debugLog(`🔍 Drones trouvés dans window.drones: ${dronesArray.length}`);
            } else if (window.authenticSwarm && window.authenticSwarm.drones) {
                dronesArray = window.authenticSwarm.drones;
                debugLog(`🔍 Drones trouvés dans window.authenticSwarm.drones: ${dronesArray.length}`);
            } else {
                debugLog(`❌ Aucun drone trouvé (tentative ${attempt})`);
                debugLog(`  - window.drones:`, window.drones);
                debugLog(`  - window.authenticSwarm:`, window.authenticSwarm);
                
                // Retry automatique si pas de drones trouvés et on n'a pas atteint le max
                if (attempt < maxAttempts) {
                    debugLog(`🔄 Retry dans 1 seconde...`);
                    setTimeout(() => processDonesWithRetry(attempt + 1, maxAttempts), 1000);
                    return;
                }
            }
            
            if (!enabled) {
                // Supprimer tous les helpers
                this.debugHelpers.forEach(helper => {
                    if (this.scene) {
                        this.scene.remove(helper);
                        helper.geometry.dispose();
                        helper.material.dispose();
                    }
                });
                this.debugHelpers = [];
                
                // Supprimer les boundary boxes visuelles des drones
                if (dronesArray.length > 0) {
                    dronesArray.forEach(drone => {
                        debugLog(`🗑️ Suppression boundary box de ${drone.id || 'drone sans id'}`);
                        if (drone.collisionBox && drone.mesh) {
                            drone.mesh.remove(drone.collisionBox);
                            drone.collisionBox = null;
                        }
                        if (drone.safetyZone && this.scene) {
                            this.scene.remove(drone.safetyZone);
                            drone.safetyZone = null;
                        }
                    });
                }
            } else {
                // Créer les boundary boxes de la plateforme si elle existe
                if (this.platformBounds && this.scene) {
                    this.createDebugHelper(this.platformBounds, 0xff0000, 'Platform');
                    debugLog(`✅ Boundary box plateforme créée`);
                }
                
                // Créer les boundary boxes des drones
                if (dronesArray.length > 0) {
                    debugLog(`🔧 Création des boundary boxes pour ${dronesArray.length} drones`);
                    dronesArray.forEach(drone => {
                        debugLog(`🔧 Tentative création boundary box pour ${drone.id || 'drone sans id'}`);
                        debugLog(`  - createCollisionBox disponible:`, typeof drone.createCollisionBox);
                        debugLog(`  - drone mesh:`, !!drone.mesh);
                        debugLog(`  - drone scene:`, !!drone.scene);
                        if (drone.createCollisionBox) {
                            try {
                                drone.createCollisionBox();
                                debugLog(`✅ Boundary box créée pour ${drone.id}`);
                            } catch (error) {
                                debugLog(`❌ Erreur création boundary box pour ${drone.id}:`, error);
                            }
                        } else {
                            debugLog(`❌ Pas de méthode createCollisionBox pour ${drone.id}`);
                        }
                    });
                } else {
                    debugLog(`❌ Aucun drone à traiter après ${maxAttempts} tentatives`);
                }
            }
        };
        
        // Commencer le processus
        processDonesWithRetry();
        
        log(`🔧 Mode debug collision: ${enabled ? 'activé' : 'désactivé'}`);
    }

    /**
     * Obtient les statistiques de collision
     */
    getStats() {
        return {
            dronesTracked: this.droneBounds.size,
            platformBounds: !!this.platformBounds,
            debugMode: this.debugMode,
            debugHelpers: this.debugHelpers.length
        };
    }

    /**
     * Bascule l'état du mode debug
     * @returns {boolean} Le nouvel état du mode debug
     */
    toggleDebugMode() {
        this.debugMode = !this.debugMode;
        this.setDebugMode(this.debugMode);
        return this.debugMode;
    }

    /**
     * Obtient l'état actuel du mode debug
     * @returns {boolean} État du mode debug
     */
    isDebugMode() {
        return this.debugMode;
    }

    /**
     * Nettoie toutes les ressources
     */
    dispose() {
        this.droneBounds.clear();
        this.setDebugMode(false);
        this.platformBounds = null;
        this.scene = null;
        log('🧹 Système de collision nettoyé');
    }
}

// Instance globale
export const collisionDetection = new CollisionDetection();
