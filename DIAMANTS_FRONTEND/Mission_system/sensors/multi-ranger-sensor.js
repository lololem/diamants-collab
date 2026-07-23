/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Multi-Ranger Sensor Simulation
 * ==========================================
 * Simulation du deck Multi-Ranger du Crazyflie
 * 
 * Le Multi-Ranger utilise 5 capteurs ToF VL53L1X:
 * - Front (avant)
 * - Back (arrière)  
 * - Left (gauche)
 * - Right (droite)
 * - Up (haut)
 * 
 * Caractéristiques VL53L1X:
 * - Portée max: 4m (conditions idéales), ~3.5m pratique
 * - Angle FOV: ~15° (cône étroit)
 * - Fréquence: jusqu'à 50Hz
 * - Précision: ±3% à 4m
 * 
 * @see https://www.bitcraze.io/products/multi-ranger-deck/
 * @module sensors/multi-ranger-sensor
 */

import * as THREE from 'three';
import { logger } from '../core/logger.js';

// Scratch réutilisé par le pré-filtre spatial (évite d'allouer par frame)
const _msCenter = new THREE.Vector3();

// ============================================================================
// CONFIGURATION CAPTEUR
// ============================================================================

export const MULTI_RANGER_CONFIG = {
    // Portée du capteur ToF VL53L1X
    maxRange: 5.0,           // mètres (portée max étendue pour vitesse 4m/s)
    minRange: 0.05,          // mètres (distance minimum)
    
    // Angle du faisceau (Field of View)
    beamFOV: Math.PI / 10,   // ~18° en radians (wider cone for better canopy detection)
    
    // Nombre de rayons par direction pour simuler le cône
    raysPerDirection: 7,
    
    // Fréquence de mise à jour
    updateRate: 30,          // Hz (33ms entre scans — faster for high speed)
    
    // Bruit de mesure (réaliste)
    noiseStdDev: 0.02,       // écart-type en mètres (~2cm)
    
    // Seuils d'alerte
    warningThreshold: 1.0,   // mètres - alerte proche (was 0.5)
    criticalThreshold: 0.4,  // mètres - danger immédiat (was 0.25)
    
    // Directions des capteurs (orientées selon le corps du drone)
    directions: {
        FRONT: { name: 'front', vector: new THREE.Vector3(0, 0, -1), index: 0 },
        BACK:  { name: 'back',  vector: new THREE.Vector3(0, 0, 1),  index: 1 },
        LEFT:  { name: 'left',  vector: new THREE.Vector3(-1, 0, 0), index: 2 },
        RIGHT: { name: 'right', vector: new THREE.Vector3(1, 0, 0),  index: 3 },
        UP:    { name: 'up',    vector: new THREE.Vector3(0, 1, 0),  index: 4 }
    }
};

// ============================================================================
// MULTI-RANGER SENSOR CLASS
// ============================================================================

/**
 * Simulation complète du Multi-Ranger Deck
 */
export class MultiRangerSensor {
    /**
     * @param {THREE.Scene} scene - Scène Three.js pour raycasting
     * @param {Object} config - Configuration optionnelle
     */
    constructor(scene, config = {}) {
        this.scene = scene;
        this.config = { ...MULTI_RANGER_CONFIG, ...config };
        
        // Raycaster Three.js
        this.raycaster = new THREE.Raycaster();
        this.raycaster.far = this.config.maxRange;
        this.raycaster.near = this.config.minRange;
        
        // Mesures actuelles [front, back, left, right, up]
        this.ranges = new Float32Array(5).fill(this.config.maxRange);
        
        // Timestamps et stats
        this.lastUpdate = 0;
        this.updateInterval = 1000 / this.config.updateRate;
        
        // Cache des objets à tester (pour performance)
        this.obstacleObjects = [];
        this.cacheValid = false;
        this.cacheExpiry = 0;
        
        // Debug visualization
        this.debugMode = false;
        this.debugHelpers = [];
        
        // Stats
        this.stats = {
            totalScans: 0,
            avgScanTime: 0,
            obstaclesDetected: 0
        };
        
        logger.log('sensor', '📡 MultiRangerSensor initialisé');
    }
    
    // ========================================================================
    // SCAN PRINCIPAL
    // ========================================================================
    
    /**
     * Effectuer un scan complet (5 directions)
     * @param {THREE.Vector3} position - Position du drone
     * @param {number} heading - Cap du drone en radians (yaw)
     * @returns {Object} Résultat du scan
     */
    scan(position, heading = 0) {
        const now = performance.now();
        
        // Rate limiting
        if (now - this.lastUpdate < this.updateInterval) {
            return this.getLastResult();
        }
        
        const startTime = now;
        this.lastUpdate = now;
        
        // Mettre à jour cache d'obstacles si nécessaire
        this._updateObstacleCache();
        
        // Rotation pour orienter selon le heading du drone
        const rotation = new THREE.Quaternion();
        rotation.setFromEuler(new THREE.Euler(0, heading, 0));
        
        // Scanner chaque direction
        const directions = this.config.directions;
        const results = {
            front: this._scanDirection(position, directions.FRONT, rotation),
            back:  this._scanDirection(position, directions.BACK, rotation),
            left:  this._scanDirection(position, directions.LEFT, rotation),
            right: this._scanDirection(position, directions.RIGHT, rotation),
            up:    this._scanDirection(position, directions.UP, rotation)
        };
        
        // Mettre à jour les ranges
        this.ranges[0] = results.front.distance;
        this.ranges[1] = results.back.distance;
        this.ranges[2] = results.left.distance;
        this.ranges[3] = results.right.distance;
        this.ranges[4] = results.up.distance;
        
        // Stats
        this.stats.totalScans++;
        const scanTime = performance.now() - startTime;
        this.stats.avgScanTime = this.stats.avgScanTime * 0.9 + scanTime * 0.1;
        
        // Debug visualization
        if (this.debugMode) {
            this._updateDebugVisualization(position, heading, results);
        }
        
        return {
            ranges: [...this.ranges],
            results: results,
            timestamp: now,
            alerts: this._generateAlerts(results)
        };
    }
    
    /**
     * Scanner une direction spécifique
     */
    _scanDirection(origin, dirConfig, rotation) {
        // Orienter le vecteur selon le heading
        const direction = dirConfig.vector.clone().applyQuaternion(rotation);

        // ── PRÉ-FILTRE SPATIAL (correctif perf CRITIQUE) ──
        // Sans ça on raycaste contre la géométrie COMPLÈTE de toute la forêt
        // (~260 meshes, des milliers de triangles chacun) × 5 directions × N rayons
        // × N drones × chaque frame => le CPU s'écroule (c'était la cause des 5-11 FPS).
        // Le capteur ne porte qu'à maxRange : on ne garde que ce qui est réellement
        // à portée, via un test de sphère englobante (très bon marché).
        const maxR = this.config.maxRange;
        const nearby = [];
        for (let i = 0; i < this.obstacleObjects.length; i++) {
            const o = this.obstacleObjects[i];
            const g = o.geometry;
            if (!g) continue;
            if (!g.boundingSphere) g.computeBoundingSphere();
            const bs = g.boundingSphere;
            if (!bs) continue;
            _msCenter.copy(bs.center).applyMatrix4(o.matrixWorld);
            const reach = maxR + bs.radius * o.matrixWorld.getMaxScaleOnAxis();
            if (_msCenter.distanceToSquared(origin) <= reach * reach) nearby.push(o);
        }


        // Lancer plusieurs rayons pour simuler le FOV du capteur
        let minDistance = this.config.maxRange;
        let hitPoint = null;
        let hitObject = null;
        
        const numRays = this.config.raysPerDirection;
        const halfFOV = this.config.beamFOV / 2;
        
        for (let i = 0; i < numRays; i++) {
            // Varier légèrement la direction dans le cône FOV
            let rayDir = direction.clone();
            
            if (numRays > 1) {
                // Créer dispersion dans le cône
                const spreadX = (Math.random() - 0.5) * halfFOV;
                const spreadY = (Math.random() - 0.5) * halfFOV;
                
                // Rotation locale autour de l'axe perpendiculaire
                const perpX = new THREE.Vector3(1, 0, 0).cross(direction);
                const perpY = new THREE.Vector3(0, 1, 0);
                
                if (perpX.length() > 0.01) {
                    perpX.normalize();
                    rayDir.applyAxisAngle(perpX, spreadX);
                }
                rayDir.applyAxisAngle(perpY, spreadY);
            }
            
            // Raycast
            this.raycaster.set(origin, rayDir.normalize());
            // non-récursif : `nearby` ne contient que des meshes (déjà filtrés)
            const intersects = this.raycaster.intersectObjects(nearby, false);
            
            if (intersects.length > 0) {
                const hit = intersects[0];
                if (hit.distance < minDistance) {
                    minDistance = hit.distance;
                    hitPoint = hit.point;
                    hitObject = hit.object;
                    this.stats.obstaclesDetected++;
                }
            }
        }
        
        // Ajouter bruit réaliste
        if (minDistance < this.config.maxRange) {
            minDistance += (Math.random() - 0.5) * 2 * this.config.noiseStdDev;
            minDistance = Math.max(this.config.minRange, Math.min(minDistance, this.config.maxRange));
        }
        
        return {
            distance: minDistance,
            direction: dirConfig.name,
            hit: minDistance < this.config.maxRange,
            hitPoint: hitPoint,
            hitObject: hitObject ? hitObject.name : null
        };
    }
    
    // ========================================================================
    // GESTION DES OBSTACLES
    // ========================================================================
    
    /**
     * Mettre à jour le cache des objets obstacles
     */
    _updateObstacleCache() {
        const now = performance.now();
        
        // Cache valide pendant 1 seconde
        if (this.cacheValid && now < this.cacheExpiry) {
            return;
        }
        
        this.obstacleObjects = [];
        
        if (!this.scene) return;
        
        // Collecter tous les objets solides de la scène
        this.scene.traverse((obj) => {
            if (!obj.isMesh) return;
            
            // Ignorer objets non-collidables
            if (obj.userData.noCollision) return;
            if (obj.name.includes('sky') || obj.name.includes('Sky')) return;
            if (obj.name.includes('grass') || obj.name.includes('Grass')) return;
            if (obj.name.includes('particle') || obj.name.includes('Particle')) return;
            if (obj.name.includes('helper') || obj.name.includes('Helper')) return;
            if (obj.name.includes('drone') || obj.name.includes('Drone')) return;
            if (obj.name.includes('crazyflie')) return;
            
            // Inclure objets solides
            if (obj.name.includes('tree') || obj.name.includes('Tree') ||
                obj.name.includes('arbre') || obj.name.includes('Arbre') ||
                obj.name.includes('trunk') || obj.name.includes('Trunk') ||
                obj.name.includes('rock') || obj.name.includes('Rock') ||
                obj.name.includes('building') || obj.name.includes('Building') ||
                obj.name.includes('wall') || obj.name.includes('Wall') ||
                obj.name.includes('platform') || obj.name.includes('Platform') ||
                obj.name.includes('terrain') || obj.name.includes('Terrain') ||
                obj.name.includes('ground') || obj.name.includes('Ground') ||
                obj.name.includes('Sol') || obj.name.includes('sol') ||
                obj.name.includes('obstacle') ||
                obj.geometry?.boundingSphere?.radius > 0.5) {  // Objets significatifs
                
                this.obstacleObjects.push(obj);
            }
        });
        
        this.cacheValid = true;
        this.cacheExpiry = now + 1000; // Cache valide 1s
        
        logger.trace('sensor', `📡 Cache obstacles: ${this.obstacleObjects.length} objets`);
    }
    
    /**
     * Forcer la mise à jour du cache
     */
    invalidateCache() {
        this.cacheValid = false;
    }
    
    /**
     * Ajouter manuellement des obstacles (arbres, etc.)
     */
    addObstacles(objects) {
        if (Array.isArray(objects)) {
            this.obstacleObjects.push(...objects);
        } else {
            this.obstacleObjects.push(objects);
        }
    }
    
    // ========================================================================
    // ALERTES ET ÉVITEMENT
    // ========================================================================
    
    /**
     * Générer les alertes basées sur les distances
     */
    _generateAlerts(results) {
        const alerts = {
            warning: [],
            critical: [],
            obstacleVector: new THREE.Vector3()
        };
        
        const directions = ['front', 'back', 'left', 'right', 'up'];
        const vectors = [
            new THREE.Vector3(0, 0, -1),  // front
            new THREE.Vector3(0, 0, 1),   // back
            new THREE.Vector3(-1, 0, 0),  // left
            new THREE.Vector3(1, 0, 0),   // right
            new THREE.Vector3(0, 1, 0)    // up
        ];
        
        directions.forEach((dir, i) => {
            const dist = results[dir].distance;
            
            if (dist < this.config.criticalThreshold) {
                alerts.critical.push({ direction: dir, distance: dist });
                // Vecteur de répulsion depuis l'obstacle
                const repulsion = vectors[i].clone().multiplyScalar(
                    (this.config.criticalThreshold - dist) / this.config.criticalThreshold * 2
                );
                alerts.obstacleVector.add(repulsion);
            } else if (dist < this.config.warningThreshold) {
                alerts.warning.push({ direction: dir, distance: dist });
                const repulsion = vectors[i].clone().multiplyScalar(
                    (this.config.warningThreshold - dist) / this.config.warningThreshold
                );
                alerts.obstacleVector.add(repulsion);
            }
        });
        
        return alerts;
    }
    
    /**
     * Obtenir un vecteur d'évitement basé sur les capteurs
     * @returns {THREE.Vector3} Vecteur de répulsion normalisé
     */
    getAvoidanceVector() {
        const avoidance = new THREE.Vector3();
        const ranges = this.ranges;
        
        // Calculer force de répulsion pour chaque direction
        const addRepulsion = (dist, dir, threshold) => {
            if (dist < threshold && dist > this.config.minRange) {
                const strength = Math.pow((threshold - dist) / threshold, 2);
                avoidance.add(dir.clone().multiplyScalar(strength));
            }
        };
        
        addRepulsion(ranges[0], new THREE.Vector3(0, 0, 1), this.config.maxRange);   // front → push back
        addRepulsion(ranges[1], new THREE.Vector3(0, 0, -1), this.config.maxRange);  // back → push front
        addRepulsion(ranges[2], new THREE.Vector3(1, 0, 0), this.config.maxRange);   // left → push right
        addRepulsion(ranges[3], new THREE.Vector3(-1, 0, 0), this.config.maxRange);  // right → push left
        addRepulsion(ranges[4], new THREE.Vector3(0, -1, 0), this.config.maxRange);  // up → push down
        
        if (avoidance.length() > 0.01) {
            avoidance.normalize();
        }
        
        return avoidance;
    }
    
    /**
     * Vérifier si le chemin vers une cible est dégagé
     */
    isPathClear(targetDirection) {
        // Trouver quelle direction correspond le mieux à la cible
        const dirs = [
            { range: this.ranges[0], dot: targetDirection.dot(new THREE.Vector3(0, 0, -1)) },
            { range: this.ranges[1], dot: targetDirection.dot(new THREE.Vector3(0, 0, 1)) },
            { range: this.ranges[2], dot: targetDirection.dot(new THREE.Vector3(-1, 0, 0)) },
            { range: this.ranges[3], dot: targetDirection.dot(new THREE.Vector3(1, 0, 0)) }
        ];
        
        // Pondérer les distances par l'alignement avec la cible
        let weightedDist = 0;
        let totalWeight = 0;
        
        dirs.forEach(d => {
            if (d.dot > 0) {
                weightedDist += d.range * d.dot;
                totalWeight += d.dot;
            }
        });
        
        const effectiveRange = totalWeight > 0 ? weightedDist / totalWeight : this.config.maxRange;
        
        return effectiveRange > this.config.warningThreshold;
    }
    
    // ========================================================================
    // ACCESSEURS
    // ========================================================================
    
    /**
     * Obtenir les dernières mesures
     */
    getLastResult() {
        return {
            ranges: [...this.ranges],
            front: this.ranges[0],
            back: this.ranges[1],
            left: this.ranges[2],
            right: this.ranges[3],
            up: this.ranges[4],
            timestamp: this.lastUpdate
        };
    }
    
    /**
     * Distance minimale détectée (tous capteurs)
     */
    getMinRange() {
        return Math.min(...this.ranges);
    }
    
    /**
     * Distance minimale horizontale (sans up)
     */
    getMinHorizontalRange() {
        return Math.min(this.ranges[0], this.ranges[1], this.ranges[2], this.ranges[3]);
    }
    
    /**
     * Statistiques du capteur
     */
    getStats() {
        return {
            ...this.stats,
            cachedObjects: this.obstacleObjects.length,
            lastUpdate: this.lastUpdate
        };
    }
    
    // ========================================================================
    // DEBUG VISUALIZATION
    // ========================================================================
    
    /**
     * Activer/désactiver la visualisation debug
     */
    setDebugMode(enabled) {
        this.debugMode = enabled;
        
        if (!enabled) {
            this._clearDebugHelpers();
        }
    }
    
    /**
     * Mettre à jour les helpers visuels
     */
    _updateDebugVisualization(position, heading, results) {
        if (!this.scene) return;
        
        this._clearDebugHelpers();
        
        const rotation = new THREE.Quaternion();
        rotation.setFromEuler(new THREE.Euler(0, heading, 0));
        
        // Créer une ligne pour chaque direction
        const colors = {
            front: 0x00ff00,  // vert
            back:  0xff0000,  // rouge
            left:  0x0000ff,  // bleu
            right: 0xffff00,  // jaune
            up:    0xff00ff   // magenta
        };
        
        const directions = this.config.directions;
        
        Object.entries(results).forEach(([name, result]) => {
            const dirConfig = directions[name.toUpperCase()];
            if (!dirConfig) return;
            
            const dir = dirConfig.vector.clone().applyQuaternion(rotation);
            const endPoint = position.clone().add(dir.multiplyScalar(result.distance));
            
            // Ligne de détection
            const geometry = new THREE.BufferGeometry().setFromPoints([
                position.clone(),
                endPoint
            ]);
            
            const color = result.hit ? 0xff6600 : colors[name];
            const material = new THREE.LineBasicMaterial({ 
                color: color,
                opacity: 0.7,
                transparent: true
            });
            
            const line = new THREE.Line(geometry, material);
            line.name = `MultiRanger_${name}`;
            
            this.scene.add(line);
            this.debugHelpers.push(line);
            
            // Point de hit
            if (result.hit && result.hitPoint) {
                const sphereGeom = new THREE.SphereGeometry(0.1);
                const sphereMat = new THREE.MeshBasicMaterial({ color: 0xff0000 });
                const sphere = new THREE.Mesh(sphereGeom, sphereMat);
                sphere.position.copy(result.hitPoint);
                sphere.name = `MultiRanger_hit_${name}`;
                
                this.scene.add(sphere);
                this.debugHelpers.push(sphere);
            }
        });
    }
    
    /**
     * Nettoyer les helpers visuels
     */
    _clearDebugHelpers() {
        this.debugHelpers.forEach(helper => {
            if (helper.geometry) helper.geometry.dispose();
            if (helper.material) helper.material.dispose();
            if (this.scene) this.scene.remove(helper);
        });
        this.debugHelpers = [];
    }
    
    /**
     * Nettoyage
     */
    dispose() {
        this._clearDebugHelpers();
        this.obstacleObjects = [];
        this.cacheValid = false;
    }
}

// ============================================================================
// MULTI-RANGER MANAGER (pour plusieurs drones)
// ============================================================================

/**
 * Gestionnaire de capteurs Multi-Ranger pour essaim
 */
export class MultiRangerManager {
    constructor(scene) {
        this.scene = scene;
        this.sensors = new Map(); // droneId → MultiRangerSensor
    }
    
    /**
     * Créer un capteur pour un drone
     */
    createSensor(droneId) {
        const sensor = new MultiRangerSensor(this.scene);
        this.sensors.set(droneId, sensor);
        return sensor;
    }
    
    /**
     * Obtenir le capteur d'un drone
     */
    getSensor(droneId) {
        return this.sensors.get(droneId);
    }
    
    /**
     * Scanner tous les drones
     * @param {Map<string, {position, heading}>} droneStates
     */
    scanAll(droneStates) {
        const results = new Map();
        
        for (const [droneId, state] of droneStates) {
            let sensor = this.sensors.get(droneId);
            if (!sensor) {
                sensor = this.createSensor(droneId);
            }
            
            const position = new THREE.Vector3(
                state.position.x,
                state.position.y,
                state.position.z
            );
            
            results.set(droneId, sensor.scan(position, state.heading || 0));
        }
        
        return results;
    }
    
    /**
     * Obtenir les vecteurs d'évitement pour tous les drones
     */
    getAllAvoidanceVectors() {
        const vectors = new Map();
        
        for (const [droneId, sensor] of this.sensors) {
            vectors.set(droneId, sensor.getAvoidanceVector());
        }
        
        return vectors;
    }
    
    /**
     * Activer le debug pour tous les capteurs
     */
    setDebugMode(enabled) {
        for (const sensor of this.sensors.values()) {
            sensor.setDebugMode(enabled);
        }
    }
    
    /**
     * Nettoyage
     */
    dispose() {
        for (const sensor of this.sensors.values()) {
            sensor.dispose();
        }
        this.sensors.clear();
    }
}

// Exports par défaut
export default {
    MultiRangerSensor,
    MultiRangerManager,
    MULTI_RANGER_CONFIG
};
