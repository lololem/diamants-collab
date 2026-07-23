/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Drone Pathfinder — Dijkstra/SSSP pour navigation DIAMANTS
 * 
 * Intégration de l'algorithme de Dijkstra pour la navigation des drones
 * avec évitement d'obstacles et optimisation de trajectoire.
 * 
 * @module drone-pathfinder
 */

import { logger } from '../core/logger.js';
import { VoxelGrid, CellState } from './environment-voxelizer.js';

// ============================================================================
// CONFIGURATION
// ============================================================================

const DEFAULT_CONFIG = {
    gridResolution: 0.5,      // Résolution de la grille (m)
    safetyMargin: 0.3,        // Marge de sécurité autour obstacles (m)
    maxAltitude: 15,          // Altitude max (m)
    minAltitude: 0.5,         // Altitude min (m)
    droneRadius: 0.15,        // Rayon du drone (m)
    neighborConnectivity: 26, // 6 (faces), 18 (+ edges), 26 (+ corners)
    weightFactors: {
        distance: 1.0,
        altitude: 0.3,        // Pénalité montée/descente
        turn: 0.1             // Pénalité changement direction
    }
};

// ============================================================================
// NAVIGATION GRID (3D)
// ============================================================================

/**
 * Grille de navigation 3D pour pathfinding
 */
export class NavigationGrid {
    constructor(bounds, config = {}) {
        this.config = { ...DEFAULT_CONFIG, ...config };
        this.bounds = bounds; // { minX, maxX, minY, maxY, minZ, maxZ }
        
        // Dimensions de la grille
        const res = this.config.gridResolution;
        this.sizeX = Math.ceil((bounds.maxX - bounds.minX) / res);
        this.sizeY = Math.ceil((bounds.maxY - bounds.minY) / res);
        this.sizeZ = Math.ceil((bounds.maxZ - bounds.minZ) / res);
        
        // Grille d'occupation (true = obstacle)
        this.occupancy = new Uint8Array(this.sizeX * this.sizeY * this.sizeZ);
        
        // Cache des voisins pour performance
        this.neighborOffsets = this._computeNeighborOffsets();
        
        logger.log('navigation', `Grid: ${this.sizeX}x${this.sizeY}x${this.sizeZ} = ${this.totalCells} cells`);
    }
    
    get totalCells() {
        return this.sizeX * this.sizeY * this.sizeZ;
    }
    
    /**
     * Convertir position monde → index grille
     */
    worldToGrid(pos) {
        const res = this.config.gridResolution;
        return {
            x: Math.floor((pos.x - this.bounds.minX) / res),
            y: Math.floor((pos.y - this.bounds.minY) / res),
            z: Math.floor((pos.z - this.bounds.minZ) / res)
        };
    }
    
    /**
     * Convertir index grille → position monde (centre de cellule)
     */
    gridToWorld(gridPos) {
        const res = this.config.gridResolution;
        return {
            x: this.bounds.minX + (gridPos.x + 0.5) * res,
            y: this.bounds.minY + (gridPos.y + 0.5) * res,
            z: this.bounds.minZ + (gridPos.z + 0.5) * res
        };
    }
    
    /**
     * Index linéaire
     */
    toIndex(x, y, z) {
        return x + y * this.sizeX + z * this.sizeX * this.sizeY;
    }
    
    /**
     * Depuis index linéaire
     */
    fromIndex(idx) {
        const z = Math.floor(idx / (this.sizeX * this.sizeY));
        const rem = idx % (this.sizeX * this.sizeY);
        const y = Math.floor(rem / this.sizeX);
        const x = rem % this.sizeX;
        return { x, y, z };
    }
    
    /**
     * Vérifier si coordonnées valides
     */
    isValid(x, y, z) {
        return x >= 0 && x < this.sizeX &&
               y >= 0 && y < this.sizeY &&
               z >= 0 && z < this.sizeZ;
    }
    
    /**
     * Vérifier si cellule libre
     */
    isFree(x, y, z) {
        if (!this.isValid(x, y, z)) return false;
        return this.occupancy[this.toIndex(x, y, z)] === 0;
    }
    
    /**
     * Marquer obstacle sphérique
     */
    addSphereObstacle(center, radius) {
        const res = this.config.gridResolution;
        const margin = this.config.safetyMargin + this.config.droneRadius;
        const effectiveRadius = radius + margin;
        
        const gc = this.worldToGrid(center);
        const gridRadius = Math.ceil(effectiveRadius / res);
        
        for (let dz = -gridRadius; dz <= gridRadius; dz++) {
            for (let dy = -gridRadius; dy <= gridRadius; dy++) {
                for (let dx = -gridRadius; dx <= gridRadius; dx++) {
                    const gx = gc.x + dx;
                    const gy = gc.y + dy;
                    const gz = gc.z + dz;
                    
                    if (!this.isValid(gx, gy, gz)) continue;
                    
                    // Distance depuis centre
                    const dist = Math.sqrt(dx*dx + dy*dy + dz*dz) * res;
                    if (dist <= effectiveRadius) {
                        this.occupancy[this.toIndex(gx, gy, gz)] = 1;
                    }
                }
            }
        }
    }
    
    /**
     * Marquer obstacle cylindrique (vertical)
     */
    addCylinderObstacle(base, radius, height) {
        const res = this.config.gridResolution;
        const margin = this.config.safetyMargin + this.config.droneRadius;
        const effectiveRadius = radius + margin;
        
        const gb = this.worldToGrid(base);
        const gridRadius = Math.ceil(effectiveRadius / res);
        const gridHeight = Math.ceil(height / res);
        
        for (let dz = -gridRadius; dz <= gridRadius; dz++) {
            for (let dx = -gridRadius; dx <= gridRadius; dx++) {
                const distXZ = Math.sqrt(dx*dx + dz*dz) * res;
                if (distXZ > effectiveRadius) continue;
                
                for (let dy = 0; dy <= gridHeight; dy++) {
                    const gx = gb.x + dx;
                    const gy = gb.y + dy;
                    const gz = gb.z + dz;
                    
                    if (this.isValid(gx, gy, gz)) {
                        this.occupancy[this.toIndex(gx, gy, gz)] = 1;
                    }
                }
            }
        }
    }
    
    /**
     * Marquer autre drone comme obstacle mobile
     */
    addDroneObstacle(position, droneRadius = null) {
        const radius = droneRadius || this.config.droneRadius;
        // Utiliser rayon plus grand pour éviter collision
        this.addSphereObstacle(position, radius * 3);
    }
    
    /**
     * Effacer grille
     */
    clear() {
        this.occupancy.fill(0);
    }
    
    /**
     * Calculer offsets des voisins selon connectivité
     */
    _computeNeighborOffsets() {
        const offsets = [];
        const conn = this.config.neighborConnectivity;
        
        // 6-connectivité (faces)
        const faces = [
            [-1, 0, 0], [1, 0, 0],
            [0, -1, 0], [0, 1, 0],
            [0, 0, -1], [0, 0, 1]
        ];
        offsets.push(...faces.map(([dx, dy, dz]) => ({ dx, dy, dz, dist: 1 })));
        
        if (conn >= 18) {
            // 18-connectivité (+ arêtes)
            const edges = [
                [-1, -1, 0], [-1, 1, 0], [1, -1, 0], [1, 1, 0],
                [-1, 0, -1], [-1, 0, 1], [1, 0, -1], [1, 0, 1],
                [0, -1, -1], [0, -1, 1], [0, 1, -1], [0, 1, 1]
            ];
            offsets.push(...edges.map(([dx, dy, dz]) => ({ dx, dy, dz, dist: Math.SQRT2 })));
        }
        
        if (conn >= 26) {
            // 26-connectivité (+ coins)
            const corners = [
                [-1, -1, -1], [-1, -1, 1], [-1, 1, -1], [-1, 1, 1],
                [1, -1, -1], [1, -1, 1], [1, 1, -1], [1, 1, 1]
            ];
            offsets.push(...corners.map(([dx, dy, dz]) => ({ dx, dy, dz, dist: Math.sqrt(3) })));
        }
        
        return offsets;
    }
    
    /**
     * Obtenir voisins libres d'une cellule
     */
    getFreeNeighbors(x, y, z) {
        const neighbors = [];
        const res = this.config.gridResolution;
        
        for (const { dx, dy, dz, dist } of this.neighborOffsets) {
            const nx = x + dx;
            const ny = y + dy;
            const nz = z + dz;
            
            if (this.isFree(nx, ny, nz)) {
                // Coût = distance euclidienne * résolution + pénalités
                let cost = dist * res * this.config.weightFactors.distance;
                
                // Pénalité altitude (monter coûte plus)
                if (dy > 0) cost += this.config.weightFactors.altitude;
                
                neighbors.push({
                    x: nx, y: ny, z: nz,
                    cost
                });
            }
        }
        
        return neighbors;
    }
}

// ============================================================================
// PRIORITY QUEUE (Min-Heap)
// ============================================================================

class MinHeap {
    constructor() {
        this.heap = [];
        this.positions = new Map(); // index → position in heap
    }
    
    get size() { return this.heap.length; }
    isEmpty() { return this.heap.length === 0; }
    
    push(index, priority) {
        if (this.positions.has(index)) {
            // Decrease key
            const pos = this.positions.get(index);
            if (priority < this.heap[pos].priority) {
                this.heap[pos].priority = priority;
                this._bubbleUp(pos);
            }
        } else {
            this.heap.push({ index, priority });
            this.positions.set(index, this.heap.length - 1);
            this._bubbleUp(this.heap.length - 1);
        }
    }
    
    pop() {
        if (this.isEmpty()) return null;
        
        const min = this.heap[0];
        const last = this.heap.pop();
        this.positions.delete(min.index);
        
        if (this.heap.length > 0) {
            this.heap[0] = last;
            this.positions.set(last.index, 0);
            this._bubbleDown(0);
        }
        
        return min;
    }
    
    _bubbleUp(pos) {
        while (pos > 0) {
            const parent = Math.floor((pos - 1) / 2);
            if (this.heap[pos].priority < this.heap[parent].priority) {
                this._swap(pos, parent);
                pos = parent;
            } else break;
        }
    }
    
    _bubbleDown(pos) {
        while (true) {
            let smallest = pos;
            const left = 2 * pos + 1;
            const right = 2 * pos + 2;
            
            if (left < this.heap.length && 
                this.heap[left].priority < this.heap[smallest].priority) {
                smallest = left;
            }
            if (right < this.heap.length && 
                this.heap[right].priority < this.heap[smallest].priority) {
                smallest = right;
            }
            
            if (smallest !== pos) {
                this._swap(pos, smallest);
                pos = smallest;
            } else break;
        }
    }
    
    _swap(i, j) {
        [this.heap[i], this.heap[j]] = [this.heap[j], this.heap[i]];
        this.positions.set(this.heap[i].index, i);
        this.positions.set(this.heap[j].index, j);
    }
}

// ============================================================================
// DIJKSTRA PATHFINDER
// ============================================================================

/**
 * Pathfinder basé sur Dijkstra pour drones
 */
export class DronePathfinder {
    constructor(grid) {
        this.grid = grid;
        this.stats = {
            nodesExpanded: 0,
            pathLength: 0,
            computeTime: 0
        };
    }
    
    /**
     * Trouver chemin optimal entre deux positions (monde)
     * @param {Object} start - Position départ {x, y, z}
     * @param {Object} goal - Position arrivée {x, y, z}
     * @returns {Array|null} Liste de waypoints ou null si pas de chemin
     */
    findPath(start, goal) {
        const t0 = performance.now();
        this.stats = { nodesExpanded: 0, pathLength: 0, computeTime: 0 };
        
        // Convertir en coordonnées grille
        const gStart = this.grid.worldToGrid(start);
        const gGoal = this.grid.worldToGrid(goal);
        
        // Vérifier validité
        if (!this.grid.isFree(gStart.x, gStart.y, gStart.z)) {
            logger.warn('pathfinder', 'Start position is blocked');
            return null;
        }
        if (!this.grid.isFree(gGoal.x, gGoal.y, gGoal.z)) {
            logger.warn('pathfinder', 'Goal position is blocked');
            return null;
        }
        
        // Indices
        const startIdx = this.grid.toIndex(gStart.x, gStart.y, gStart.z);
        const goalIdx = this.grid.toIndex(gGoal.x, gGoal.y, gGoal.z);
        
        // Dijkstra
        const dist = new Float32Array(this.grid.totalCells).fill(Infinity);
        const prev = new Int32Array(this.grid.totalCells).fill(-1);
        const visited = new Uint8Array(this.grid.totalCells);
        
        dist[startIdx] = 0;
        const heap = new MinHeap();
        heap.push(startIdx, 0);
        
        while (!heap.isEmpty()) {
            const { index: current, priority: d } = heap.pop();
            
            if (visited[current]) continue;
            visited[current] = 1;
            this.stats.nodesExpanded++;
            
            // Trouvé !
            if (current === goalIdx) {
                break;
            }
            
            // Si distance actuelle > celle stockée, ignorer
            if (d > dist[current]) continue;
            
            // Explorer voisins
            const pos = this.grid.fromIndex(current);
            const neighbors = this.grid.getFreeNeighbors(pos.x, pos.y, pos.z);
            
            for (const n of neighbors) {
                const nIdx = this.grid.toIndex(n.x, n.y, n.z);
                const newDist = dist[current] + n.cost;
                
                if (newDist < dist[nIdx]) {
                    dist[nIdx] = newDist;
                    prev[nIdx] = current;
                    heap.push(nIdx, newDist);
                }
            }
        }
        
        // Pas de chemin
        if (dist[goalIdx] === Infinity) {
            this.stats.computeTime = performance.now() - t0;
            logger.warn('pathfinder', `No path found (expanded ${this.stats.nodesExpanded} nodes)`);
            return null;
        }
        
        // Reconstruire chemin
        const path = [];
        let current = goalIdx;
        while (current !== -1) {
            const pos = this.grid.fromIndex(current);
            path.unshift(this.grid.gridToWorld(pos));
            current = prev[current];
        }
        
        this.stats.pathLength = dist[goalIdx];
        this.stats.computeTime = performance.now() - t0;
        
        logger.log('pathfinder', 
            `Path found: ${path.length} waypoints, ${this.stats.pathLength.toFixed(2)}m, ` +
            `${this.stats.nodesExpanded} nodes in ${this.stats.computeTime.toFixed(1)}ms`
        );
        
        // Simplifier le chemin (enlever points colinéaires)
        return this._simplifyPath(path);
    }
    
    /**
     * Simplifier chemin en gardant uniquement les points de changement de direction
     */
    _simplifyPath(path) {
        if (path.length <= 2) return path;
        
        const simplified = [path[0]];
        
        for (let i = 1; i < path.length - 1; i++) {
            const prev = path[i - 1];
            const curr = path[i];
            const next = path[i + 1];
            
            // Direction précédente et suivante
            const d1 = {
                x: curr.x - prev.x,
                y: curr.y - prev.y,
                z: curr.z - prev.z
            };
            const d2 = {
                x: next.x - curr.x,
                y: next.y - curr.y,
                z: next.z - curr.z
            };
            
            // Normaliser
            const len1 = Math.sqrt(d1.x**2 + d1.y**2 + d1.z**2);
            const len2 = Math.sqrt(d2.x**2 + d2.y**2 + d2.z**2);
            
            if (len1 > 0 && len2 > 0) {
                const dot = (d1.x*d2.x + d1.y*d2.y + d1.z*d2.z) / (len1 * len2);
                
                // Si changement de direction significatif, garder le point
                if (dot < 0.98) {
                    simplified.push(curr);
                }
            }
        }
        
        simplified.push(path[path.length - 1]);
        
        logger.log('pathfinder', `Simplified: ${path.length} → ${simplified.length} waypoints`);
        return simplified;
    }
}

// ============================================================================
// SWARM PATHFINDER (Multi-Agent)
// ============================================================================

/**
 * Pathfinder pour essaim de drones (évitement collision inter-drones)
 */
export class SwarmPathfinder {
    constructor(bounds, config = {}) {
        this.config = { ...DEFAULT_CONFIG, ...config };
        this.bounds = bounds;
        
        // Grille partagée
        this.grid = new NavigationGrid(bounds, config);
        this.pathfinder = new DronePathfinder(this.grid);
        
        // Position des drones
        this.dronePositions = new Map();
        
        // Chemins réservés (pour éviter collision future)
        this.reservedPaths = new Map();
    }
    
    /**
     * Mettre à jour position d'un drone
     */
    updateDronePosition(droneId, position) {
        this.dronePositions.set(droneId, position);
    }
    
    /**
     * Ajouter obstacle statique
     */
    addObstacle(type, ...params) {
        if (type === 'sphere') {
            this.grid.addSphereObstacle(...params);
        } else if (type === 'cylinder') {
            this.grid.addCylinderObstacle(...params);
        }
    }
    
    /**
     * Calculer chemin pour un drone en évitant les autres
     */
    findPathForDrone(droneId, goal) {
        const start = this.dronePositions.get(droneId);
        if (!start) {
            logger.warn('swarm-pathfinder', `Drone ${droneId} position unknown`);
            return null;
        }
        
        // Sauvegarder état grille
        const gridBackup = new Uint8Array(this.grid.occupancy);
        
        // Ajouter autres drones comme obstacles temporaires
        for (const [otherId, pos] of this.dronePositions) {
            if (otherId !== droneId) {
                this.grid.addDroneObstacle(pos);
            }
        }
        
        // Trouver chemin
        const path = this.pathfinder.findPath(start, goal);
        
        // Restaurer grille
        this.grid.occupancy.set(gridBackup);
        
        if (path) {
            this.reservedPaths.set(droneId, path);
        }
        
        return path;
    }
    
    /**
     * Calculer chemins optimaux pour tous les drones vers leurs objectifs
     * (résout les conflits de chemin)
     */
    findPathsForAll(goals) {
        const paths = new Map();
        
        // Trier par distance (plus proche d'abord pour éviter blocage)
        const sortedDrones = [...goals.entries()].sort((a, b) => {
            const posA = this.dronePositions.get(a[0]);
            const posB = this.dronePositions.get(b[0]);
            if (!posA || !posB) return 0;
            
            const distA = Math.hypot(a[1].x - posA.x, a[1].y - posA.y, a[1].z - posA.z);
            const distB = Math.hypot(b[1].x - posB.x, b[1].y - posB.y, b[1].z - posB.z);
            return distA - distB;
        });
        
        for (const [droneId, goal] of sortedDrones) {
            const path = this.findPathForDrone(droneId, goal);
            if (path) {
                paths.set(droneId, path);
            }
        }
        
        return paths;
    }
    
    /**
     * Effacer tous les obstacles dynamiques
     */
    clearDynamicObstacles() {
        // Reconstruire grille avec uniquement obstacles statiques
        this.grid.clear();
    }
}

// ============================================================================
// INTEGRATION HELPER
// ============================================================================

/**
 * Créer pathfinder pour zone DIAMANTS standard
 */
export function createDiamantsPathfinder(zoneParams) {
    const bounds = {
        minX: -zoneParams.sizeX / 2,
        maxX: zoneParams.sizeX / 2,
        minY: 0,
        maxY: zoneParams.maxAltitude || 15,
        minZ: -zoneParams.sizeZ / 2,
        maxZ: zoneParams.sizeZ / 2
    };
    
    return new SwarmPathfinder(bounds, {
        gridResolution: zoneParams.resolution || 0.5
    });
}

// ============================================================================
// VOXELGRID INTEGRATION
// ============================================================================

/**
 * Adaptateur pour utiliser VoxelGrid avec DronePathfinder
 * Connecte environment-voxelizer.js avec drone-pathfinder.js
 */
export class VoxelGridAdapter {
    constructor(voxelGrid) {
        this.voxelGrid = voxelGrid;
        this.config = {
            gridResolution: voxelGrid.resolution,
            weightFactors: DEFAULT_CONFIG.weightFactors
        };
        
        // Exposer les propriétés nécessaires
        this.bounds = voxelGrid.bounds;
        this.sizeX = voxelGrid.sizeX;
        this.sizeY = voxelGrid.sizeY;
        this.sizeZ = voxelGrid.sizeZ;
        this.occupancy = voxelGrid.state; // Map state to occupancy
    }
    
    get totalCells() {
        return this.voxelGrid.totalCells;
    }
    
    worldToGrid(pos) {
        return this.voxelGrid.worldToGrid(pos);
    }
    
    gridToWorld(gridPos) {
        if (typeof gridPos === 'object') {
            return this.voxelGrid.gridToWorld(gridPos.x, gridPos.y, gridPos.z);
        }
        const pos = this.voxelGrid.fromIndex(gridPos);
        return this.voxelGrid.gridToWorld(pos.x, pos.y, pos.z);
    }
    
    toIndex(x, y, z) {
        return this.voxelGrid.toIndex(x, y, z);
    }
    
    fromIndex(idx) {
        return this.voxelGrid.fromIndex(idx);
    }
    
    isValid(x, y, z) {
        return this.voxelGrid.isValidGrid(x, y, z);
    }
    
    isFree(x, y, z) {
        return this.voxelGrid.isFree(x, y, z);
    }
    
    getFreeNeighbors(x, y, z) {
        return this.voxelGrid.getNeighbors(x, y, z, 26);
    }
    
    addSphereObstacle(center, radius) {
        this.voxelGrid.addSphere(center, radius, CellState.OBSTACLE);
    }
    
    addCylinderObstacle(base, radius, height) {
        this.voxelGrid.addCylinder(base, radius, height, CellState.OBSTACLE);
    }
    
    addDroneObstacle(position, droneRadius = 0.15) {
        this.voxelGrid.addSphere(position, droneRadius * 3, CellState.DRONE);
    }
    
    clear() {
        this.voxelGrid.reset();
    }
}

/**
 * Créer pathfinder depuis VoxelGrid existant
 * @param {VoxelGrid} voxelGrid - Grille voxel depuis environment-voxelizer
 * @returns {Object} - { pathfinder, grid }
 */
export function createPathfinderFromVoxelGrid(voxelGrid) {
    const adapter = new VoxelGridAdapter(voxelGrid);
    const pathfinder = new DronePathfinder(adapter);
    
    return {
        pathfinder,
        grid: adapter,
        voxelGrid,
        
        /**
         * Trouver chemin entre deux positions
         */
        findPath(start, goal) {
            return pathfinder.findPath(start, goal);
        },
        
        /**
         * Marquer chemin dans la grille voxel
         */
        findAndMarkPath(start, goal) {
            const path = pathfinder.findPath(start, goal);
            if (path) {
                voxelGrid.markPath(path);
            }
            return path;
        },
        
        /**
         * Statistiques du dernier calcul
         */
        getStats() {
            return pathfinder.stats;
        }
    };
}

/**
 * Exemple d'utilisation
 */
export function pathfinderDemo() {
    // Créer zone
    const pathfinder = createDiamantsPathfinder({
        sizeX: 50,
        sizeZ: 50,
        maxAltitude: 10,
        resolution: 0.5
    });
    
    // Ajouter obstacles
    pathfinder.addObstacle('cylinder', { x: 10, y: 0, z: 5 }, 2, 8);
    pathfinder.addObstacle('sphere', { x: -5, y: 5, z: -10 }, 3);
    
    // Positions drones
    pathfinder.updateDronePosition('drone_1', { x: -20, y: 2, z: -20 });
    pathfinder.updateDronePosition('drone_2', { x: -18, y: 2, z: -20 });
    
    // Calculer chemins
    const goals = new Map([
        ['drone_1', { x: 20, y: 2, z: 20 }],
        ['drone_2', { x: 18, y: 2, z: 20 }]
    ]);
    
    const paths = pathfinder.findPathsForAll(goals);
    
    console.log('=== DIAMANTS Pathfinder Demo ===');
    for (const [droneId, path] of paths) {
        console.log(`${droneId}: ${path.length} waypoints`);
    }
    
    return paths;
}

// Exports
export default {
    NavigationGrid,
    DronePathfinder,
    SwarmPathfinder,
    VoxelGridAdapter,
    createDiamantsPathfinder,
    createPathfinderFromVoxelGrid,
    pathfinderDemo
};
