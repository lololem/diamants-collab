/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Environment Voxelizer — Discrétisation 3D de l'environnement DIAMANTS
 * 
 * Pixelise/Voxelise l'espace de vol en cellules 3D pour:
 * - Pathfinding (Dijkstra/A*)
 * - Détection collision
 * - Cartographie de couverture
 * - Visualisation de l'état de l'environnement
 * 
 * @module environment-voxelizer
 */

import * as THREE from 'three';
import { logger } from '../core/logger.js';

// ============================================================================
// CONSTANTES
// ============================================================================

export const CellState = {
    FREE: 0,           // Libre, navigable
    OBSTACLE: 1,       // Obstacle statique
    DRONE: 2,          // Position d'un drone
    EXPLORED: 3,       // Exploré par un drone
    TARGET: 4,         // Cible/waypoint
    NO_FLY: 5,         // Zone interdite
    DANGER: 6,         // Zone dangereuse détectée
    PATH: 7            // Chemin planifié
};

export const CellColors = {
    [CellState.FREE]: 0x00ff00,      // Vert transparent
    [CellState.OBSTACLE]: 0xff0000,   // Rouge
    [CellState.DRONE]: 0x00ffff,      // Cyan
    [CellState.EXPLORED]: 0x0066ff,   // Bleu
    [CellState.TARGET]: 0xffff00,     // Jaune
    [CellState.NO_FLY]: 0xff00ff,     // Magenta
    [CellState.DANGER]: 0xff6600,     // Orange
    [CellState.PATH]: 0xffffff        // Blanc
};

// ============================================================================
// VOXEL GRID
// ============================================================================

/**
 * Grille voxel 3D de l'environnement
 */
export class VoxelGrid {
    /**
     * @param {Object} bounds - Limites de l'environnement
     * @param {number} resolution - Taille d'un voxel en mètres
     */
    constructor(bounds, resolution = 0.5) {
        this.bounds = {
            minX: bounds.minX ?? -25,
            maxX: bounds.maxX ?? 25,
            minY: bounds.minY ?? 0,
            maxY: bounds.maxY ?? 15,
            minZ: bounds.minZ ?? -25,
            maxZ: bounds.maxZ ?? 25
        };
        
        this.resolution = resolution;
        
        // Calculer dimensions
        this.sizeX = Math.ceil((this.bounds.maxX - this.bounds.minX) / resolution);
        this.sizeY = Math.ceil((this.bounds.maxY - this.bounds.minY) / resolution);
        this.sizeZ = Math.ceil((this.bounds.maxZ - this.bounds.minZ) / resolution);
        
        this.totalCells = this.sizeX * this.sizeY * this.sizeZ;
        
        // Grilles de données
        this.state = new Uint8Array(this.totalCells);          // État de chaque cellule
        this.costMap = new Float32Array(this.totalCells);       // Coût de traversée
        this.visitCount = new Uint16Array(this.totalCells);     // Nombre de visites
        this.lastVisit = new Float32Array(this.totalCells);     // Timestamp dernière visite
        this.dronePresence = new Uint8Array(this.totalCells);   // ID drone présent (0 = aucun)
        
        // Initialiser coûts par défaut
        this.costMap.fill(1.0);
        
        // Statistiques
        this.stats = {
            freeCells: this.totalCells,
            obstacleCells: 0,
            exploredCells: 0,
            coveragePercent: 0
        };
        
        logger.log('voxelizer', 
            `Grille: ${this.sizeX}×${this.sizeY}×${this.sizeZ} = ${this.totalCells} voxels ` +
            `(${resolution}m/voxel)`
        );
    }
    
    // ========================================================================
    // CONVERSION COORDONNÉES
    // ========================================================================
    
    /**
     * Position monde → indices grille
     */
    worldToGrid(pos) {
        return {
            x: Math.floor((pos.x - this.bounds.minX) / this.resolution),
            y: Math.floor((pos.y - this.bounds.minY) / this.resolution),
            z: Math.floor((pos.z - this.bounds.minZ) / this.resolution)
        };
    }
    
    /**
     * Indices grille → position monde (centre du voxel)
     */
    gridToWorld(gx, gy, gz) {
        return {
            x: this.bounds.minX + (gx + 0.5) * this.resolution,
            y: this.bounds.minY + (gy + 0.5) * this.resolution,
            z: this.bounds.minZ + (gz + 0.5) * this.resolution
        };
    }
    
    /**
     * Index linéaire
     */
    toIndex(x, y, z) {
        if (!this.isValidGrid(x, y, z)) return -1;
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
     * Vérifier coordonnées grille valides
     */
    isValidGrid(x, y, z) {
        return x >= 0 && x < this.sizeX &&
               y >= 0 && y < this.sizeY &&
               z >= 0 && z < this.sizeZ;
    }
    
    /**
     * Vérifier position monde dans les limites
     */
    isInBounds(pos) {
        return pos.x >= this.bounds.minX && pos.x <= this.bounds.maxX &&
               pos.y >= this.bounds.minY && pos.y <= this.bounds.maxY &&
               pos.z >= this.bounds.minZ && pos.z <= this.bounds.maxZ;
    }
    
    // ========================================================================
    // MANIPULATION ÉTATS
    // ========================================================================
    
    /**
     * Obtenir état d'une cellule
     */
    getState(x, y, z) {
        const idx = this.toIndex(x, y, z);
        return idx >= 0 ? this.state[idx] : CellState.OBSTACLE;
    }
    
    /**
     * Définir état d'une cellule
     */
    setState(x, y, z, state) {
        const idx = this.toIndex(x, y, z);
        if (idx >= 0) {
            const oldState = this.state[idx];
            this.state[idx] = state;
            this._updateStats(oldState, state);
        }
    }
    
    /**
     * Obtenir état à position monde
     */
    getStateAt(pos) {
        const g = this.worldToGrid(pos);
        return this.getState(g.x, g.y, g.z);
    }
    
    /**
     * Définir état à position monde
     */
    setStateAt(pos, state) {
        const g = this.worldToGrid(pos);
        this.setState(g.x, g.y, g.z, state);
    }
    
    /**
     * Cellule libre ?
     */
    isFree(x, y, z) {
        const state = this.getState(x, y, z);
        return state === CellState.FREE || state === CellState.EXPLORED;
    }
    
    /**
     * Cellule libre à position monde ?
     */
    isFreeAt(pos) {
        const g = this.worldToGrid(pos);
        return this.isFree(g.x, g.y, g.z);
    }
    
    /**
     * Obtenir coût de traversée
     */
    getCost(x, y, z) {
        const idx = this.toIndex(x, y, z);
        if (idx < 0) return Infinity;
        if (this.state[idx] === CellState.OBSTACLE) return Infinity;
        if (this.state[idx] === CellState.NO_FLY) return Infinity;
        return this.costMap[idx];
    }
    
    /**
     * Définir coût de traversée
     */
    setCost(x, y, z, cost) {
        const idx = this.toIndex(x, y, z);
        if (idx >= 0) {
            this.costMap[idx] = cost;
        }
    }
    
    // ========================================================================
    // AJOUT D'OBSTACLES
    // ========================================================================
    
    /**
     * Ajouter obstacle sphérique
     */
    addSphere(center, radius, state = CellState.OBSTACLE) {
        const gc = this.worldToGrid(center);
        const gridRadius = Math.ceil(radius / this.resolution) + 1;
        
        for (let dz = -gridRadius; dz <= gridRadius; dz++) {
            for (let dy = -gridRadius; dy <= gridRadius; dy++) {
                for (let dx = -gridRadius; dx <= gridRadius; dx++) {
                    const gx = gc.x + dx;
                    const gy = gc.y + dy;
                    const gz = gc.z + dz;
                    
                    if (!this.isValidGrid(gx, gy, gz)) continue;
                    
                    // Distance depuis centre en monde
                    const worldPos = this.gridToWorld(gx, gy, gz);
                    const dist = Math.sqrt(
                        (worldPos.x - center.x) ** 2 +
                        (worldPos.y - center.y) ** 2 +
                        (worldPos.z - center.z) ** 2
                    );
                    
                    if (dist <= radius) {
                        this.setState(gx, gy, gz, state);
                    }
                }
            }
        }
    }
    
    /**
     * Ajouter obstacle boîte (AABB)
     */
    addBox(min, max, state = CellState.OBSTACLE) {
        const gMin = this.worldToGrid(min);
        const gMax = this.worldToGrid(max);
        
        for (let gz = gMin.z; gz <= gMax.z; gz++) {
            for (let gy = gMin.y; gy <= gMax.y; gy++) {
                for (let gx = gMin.x; gx <= gMax.x; gx++) {
                    this.setState(gx, gy, gz, state);
                }
            }
        }
    }
    
    /**
     * Ajouter obstacle cylindrique (vertical)
     */
    addCylinder(base, radius, height, state = CellState.OBSTACLE) {
        const gb = this.worldToGrid(base);
        const gridRadius = Math.ceil(radius / this.resolution) + 1;
        const gridHeight = Math.ceil(height / this.resolution);
        
        for (let dy = 0; dy <= gridHeight; dy++) {
            for (let dz = -gridRadius; dz <= gridRadius; dz++) {
                for (let dx = -gridRadius; dx <= gridRadius; dx++) {
                    const gx = gb.x + dx;
                    const gy = gb.y + dy;
                    const gz = gb.z + dz;
                    
                    if (!this.isValidGrid(gx, gy, gz)) continue;
                    
                    // Distance XZ depuis axe
                    const worldPos = this.gridToWorld(gx, gy, gz);
                    const distXZ = Math.sqrt(
                        (worldPos.x - base.x) ** 2 +
                        (worldPos.z - base.z) ** 2
                    );
                    
                    if (distXZ <= radius) {
                        this.setState(gx, gy, gz, state);
                    }
                }
            }
        }
    }
    
    /**
     * Ajouter zone interdite (polygone 2D extrudé)
     */
    addNoFlyZone(polygon, minY, maxY) {
        const gMinY = Math.floor((minY - this.bounds.minY) / this.resolution);
        const gMaxY = Math.ceil((maxY - this.bounds.minY) / this.resolution);
        
        // Pour chaque cellule XZ
        for (let gz = 0; gz < this.sizeZ; gz++) {
            for (let gx = 0; gx < this.sizeX; gx++) {
                const worldPos = this.gridToWorld(gx, 0, gz);
                
                // Test point-in-polygon
                if (this._pointInPolygon({ x: worldPos.x, z: worldPos.z }, polygon)) {
                    for (let gy = gMinY; gy <= gMaxY; gy++) {
                        if (this.isValidGrid(gx, gy, gz)) {
                            this.setState(gx, gy, gz, CellState.NO_FLY);
                        }
                    }
                }
            }
        }
    }
    
    /**
     * Ray-casting point-in-polygon
     */
    _pointInPolygon(point, polygon) {
        let inside = false;
        for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
            const xi = polygon[i].x, zi = polygon[i].z;
            const xj = polygon[j].x, zj = polygon[j].z;
            
            if (((zi > point.z) !== (zj > point.z)) &&
                (point.x < (xj - xi) * (point.z - zi) / (zj - zi) + xi)) {
                inside = !inside;
            }
        }
        return inside;
    }
    
    // ========================================================================
    // MISE À JOUR DYNAMIQUE
    // ========================================================================
    
    /**
     * Mettre à jour position d'un drone
     */
    updateDrone(droneId, position, radius = 0.15) {
        // Effacer ancienne position
        const numericId = typeof droneId === 'string' 
            ? parseInt(droneId.replace(/\D/g, '')) || 1 
            : droneId;
            
        for (let i = 0; i < this.totalCells; i++) {
            if (this.dronePresence[i] === numericId) {
                this.dronePresence[i] = 0;
                if (this.state[i] === CellState.DRONE) {
                    this.state[i] = CellState.EXPLORED;
                }
            }
        }
        
        // Marquer nouvelle position
        const gc = this.worldToGrid(position);
        const gridRadius = Math.ceil(radius / this.resolution) + 1;
        
        for (let dz = -gridRadius; dz <= gridRadius; dz++) {
            for (let dy = -gridRadius; dy <= gridRadius; dy++) {
                for (let dx = -gridRadius; dx <= gridRadius; dx++) {
                    const gx = gc.x + dx;
                    const gy = gc.y + dy;
                    const gz = gc.z + dz;
                    
                    const idx = this.toIndex(gx, gy, gz);
                    if (idx >= 0) {
                        const dist = Math.sqrt(dx*dx + dy*dy + dz*dz) * this.resolution;
                        if (dist <= radius) {
                            this.dronePresence[idx] = numericId;
                            this.state[idx] = CellState.DRONE;
                        }
                    }
                }
            }
        }
        
        // Marquer zone explorée autour du drone (rayon de détection)
        const detectionRadius = 2.0; // mètres
        this.markExplored(position, detectionRadius);
    }
    
    /**
     * Marquer zone comme explorée
     */
    markExplored(center, radius) {
        const gc = this.worldToGrid(center);
        const gridRadius = Math.ceil(radius / this.resolution);
        const now = performance.now() / 1000;
        
        for (let dz = -gridRadius; dz <= gridRadius; dz++) {
            for (let dy = -gridRadius; dy <= gridRadius; dy++) {
                for (let dx = -gridRadius; dx <= gridRadius; dx++) {
                    const gx = gc.x + dx;
                    const gy = gc.y + dy;
                    const gz = gc.z + dz;
                    
                    const idx = this.toIndex(gx, gy, gz);
                    if (idx >= 0) {
                        const dist = Math.sqrt(dx*dx + dy*dy + dz*dz) * this.resolution;
                        if (dist <= radius && this.state[idx] === CellState.FREE) {
                            this.state[idx] = CellState.EXPLORED;
                            this.visitCount[idx]++;
                            this.lastVisit[idx] = now;
                            this.stats.exploredCells++;
                        }
                    }
                }
            }
        }
        
        this._updateCoverageStats();
    }
    
    /**
     * Marquer chemin planifié
     */
    markPath(waypoints, pathId = 1) {
        // Effacer ancien chemin
        for (let i = 0; i < this.totalCells; i++) {
            if (this.state[i] === CellState.PATH) {
                this.state[i] = CellState.FREE;
            }
        }
        
        // Marquer nouveau chemin
        for (const wp of waypoints) {
            const g = this.worldToGrid(wp);
            const idx = this.toIndex(g.x, g.y, g.z);
            if (idx >= 0 && this.state[idx] !== CellState.OBSTACLE) {
                this.state[idx] = CellState.PATH;
            }
        }
    }
    
    /**
     * Effacer toutes les données dynamiques
     */
    clearDynamic() {
        for (let i = 0; i < this.totalCells; i++) {
            if (this.state[i] === CellState.DRONE || 
                this.state[i] === CellState.PATH) {
                this.state[i] = CellState.FREE;
            }
            this.dronePresence[i] = 0;
        }
    }
    
    /**
     * Réinitialiser complètement
     */
    reset() {
        this.state.fill(CellState.FREE);
        this.costMap.fill(1.0);
        this.visitCount.fill(0);
        this.lastVisit.fill(0);
        this.dronePresence.fill(0);
        this._resetStats();
    }
    
    // ========================================================================
    // STATISTIQUES
    // ========================================================================
    
    _updateStats(oldState, newState) {
        if (oldState === newState) return;
        
        // Décompter ancien état
        if (oldState === CellState.FREE) this.stats.freeCells--;
        else if (oldState === CellState.OBSTACLE) this.stats.obstacleCells--;
        else if (oldState === CellState.EXPLORED) this.stats.exploredCells--;
        
        // Compter nouveau état
        if (newState === CellState.FREE) this.stats.freeCells++;
        else if (newState === CellState.OBSTACLE) this.stats.obstacleCells++;
        else if (newState === CellState.EXPLORED) this.stats.exploredCells++;
    }
    
    _updateCoverageStats() {
        const navigable = this.stats.freeCells + this.stats.exploredCells;
        this.stats.coveragePercent = navigable > 0 
            ? (this.stats.exploredCells / navigable) * 100 
            : 0;
    }
    
    _resetStats() {
        this.stats = {
            freeCells: this.totalCells,
            obstacleCells: 0,
            exploredCells: 0,
            coveragePercent: 0
        };
    }
    
    // ========================================================================
    // REQUÊTES SPATIALES
    // ========================================================================
    
    /**
     * Obtenir voisins navigables
     */
    getNeighbors(x, y, z, connectivity = 6) {
        const neighbors = [];
        
        // 6-connectivité (faces)
        const offsets6 = [
            [-1, 0, 0], [1, 0, 0],
            [0, -1, 0], [0, 1, 0],
            [0, 0, -1], [0, 0, 1]
        ];
        
        // 26-connectivité (tout)
        const offsets26 = [];
        for (let dz = -1; dz <= 1; dz++) {
            for (let dy = -1; dy <= 1; dy++) {
                for (let dx = -1; dx <= 1; dx++) {
                    if (dx !== 0 || dy !== 0 || dz !== 0) {
                        offsets26.push([dx, dy, dz]);
                    }
                }
            }
        }
        
        const offsets = connectivity >= 26 ? offsets26 : offsets6;
        
        for (const [dx, dy, dz] of offsets) {
            const nx = x + dx;
            const ny = y + dy;
            const nz = z + dz;
            
            if (this.isFree(nx, ny, nz)) {
                const dist = Math.sqrt(dx*dx + dy*dy + dz*dz) * this.resolution;
                const cost = this.getCost(nx, ny, nz) * dist;
                
                neighbors.push({ x: nx, y: ny, z: nz, cost });
            }
        }
        
        return neighbors;
    }
    
    /**
     * Trouver cellules non explorées les plus proches
     */
    findNearestUnexplored(fromPos, maxDistance = Infinity) {
        const gc = this.worldToGrid(fromPos);
        let nearest = null;
        let minDist = maxDistance;
        
        for (let i = 0; i < this.totalCells; i++) {
            if (this.state[i] === CellState.FREE) {
                const pos = this.fromIndex(i);
                const dist = Math.sqrt(
                    (pos.x - gc.x) ** 2 +
                    (pos.y - gc.y) ** 2 +
                    (pos.z - gc.z) ** 2
                ) * this.resolution;
                
                if (dist < minDist) {
                    minDist = dist;
                    nearest = this.gridToWorld(pos.x, pos.y, pos.z);
                }
            }
        }
        
        return nearest;
    }
    
    /**
     * Raycast pour vérifier ligne de vue
     */
    hasLineOfSight(from, to) {
        const gFrom = this.worldToGrid(from);
        const gTo = this.worldToGrid(to);
        
        // Bresenham 3D
        let x = gFrom.x, y = gFrom.y, z = gFrom.z;
        const dx = Math.abs(gTo.x - x), dy = Math.abs(gTo.y - y), dz = Math.abs(gTo.z - z);
        const sx = x < gTo.x ? 1 : -1, sy = y < gTo.y ? 1 : -1, sz = z < gTo.z ? 1 : -1;
        
        const dm = Math.max(dx, dy, dz);
        let i = dm;
        
        let x_inc = dx / dm, y_inc = dy / dm, z_inc = dz / dm;
        let x_err = 0, y_err = 0, z_err = 0;
        
        while (i-- > 0) {
            if (!this.isFree(Math.round(x), Math.round(y), Math.round(z))) {
                return false;
            }
            
            x_err += x_inc; y_err += y_inc; z_err += z_inc;
            if (x_err >= 0.5) { x += sx; x_err -= 1; }
            if (y_err >= 0.5) { y += sy; y_err -= 1; }
            if (z_err >= 0.5) { z += sz; z_err -= 1; }
        }
        
        return true;
    }
}

// ============================================================================
// VISUALISATION THREE.JS
// ============================================================================

/**
 * Visualiseur de grille voxel pour Three.js
 */
export class VoxelVisualizer {
    /**
     * @param {VoxelGrid} grid - Grille voxel à visualiser
     * @param {THREE.Scene} scene - Scène Three.js
     */
    constructor(grid, scene) {
        this.grid = grid;
        this.scene = scene;
        
        // Groupe pour tous les voxels
        this.group = new THREE.Group();
        this.group.name = 'VoxelGrid';
        scene.add(this.group);
        
        // Géométrie partagée
        this.voxelGeometry = new THREE.BoxGeometry(
            grid.resolution * 0.9,
            grid.resolution * 0.9,
            grid.resolution * 0.9
        );
        
        // Matériaux par état
        this.materials = {};
        for (const [state, color] of Object.entries(CellColors)) {
            this.materials[state] = new THREE.MeshBasicMaterial({
                color,
                transparent: true,
                opacity: state == CellState.FREE ? 0.05 : 0.6,
                wireframe: state == CellState.FREE
            });
        }
        
        // Meshes instanciées par état
        this.instancedMeshes = new Map();
        
        // Options de rendu
        this.options = {
            showFree: false,
            showObstacles: true,
            showExplored: true,
            showDrones: true,
            showPath: true,
            maxDisplayCells: 50000  // Limite pour performance
        };
    }
    
    /**
     * Mettre à jour la visualisation
     */
    update() {
        // Effacer anciens meshes
        while (this.group.children.length > 0) {
            const child = this.group.children[0];
            this.group.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
        
        // Compter cellules par état
        const cellsByState = new Map();
        for (let i = 0; i < this.grid.totalCells; i++) {
            const state = this.grid.state[i];
            if (!cellsByState.has(state)) cellsByState.set(state, []);
            cellsByState.get(state).push(i);
        }
        
        // Créer meshes instanciées
        for (const [state, indices] of cellsByState) {
            // Filtrer selon options
            if (state === CellState.FREE && !this.options.showFree) continue;
            if (state === CellState.OBSTACLE && !this.options.showObstacles) continue;
            if (state === CellState.EXPLORED && !this.options.showExplored) continue;
            if (state === CellState.DRONE && !this.options.showDrones) continue;
            if (state === CellState.PATH && !this.options.showPath) continue;
            
            // Limiter nombre
            const displayIndices = indices.slice(0, this.options.maxDisplayCells);
            if (displayIndices.length === 0) continue;
            
            // Créer InstancedMesh
            const mesh = new THREE.InstancedMesh(
                this.voxelGeometry,
                this.materials[state],
                displayIndices.length
            );
            
            const matrix = new THREE.Matrix4();
            for (let i = 0; i < displayIndices.length; i++) {
                const pos = this.grid.fromIndex(displayIndices[i]);
                const worldPos = this.grid.gridToWorld(pos.x, pos.y, pos.z);
                
                matrix.setPosition(worldPos.x, worldPos.y, worldPos.z);
                mesh.setMatrixAt(i, matrix);
            }
            
            mesh.instanceMatrix.needsUpdate = true;
            this.group.add(mesh);
        }
    }
    
    /**
     * Afficher/masquer un type de cellule
     */
    setVisible(stateType, visible) {
        switch (stateType) {
            case 'free': this.options.showFree = visible; break;
            case 'obstacles': this.options.showObstacles = visible; break;
            case 'explored': this.options.showExplored = visible; break;
            case 'drones': this.options.showDrones = visible; break;
            case 'path': this.options.showPath = visible; break;
        }
        this.update();
    }
    
    /**
     * Afficher grille en mode wireframe
     */
    showWireframe(show) {
        for (const [state, mat] of Object.entries(this.materials)) {
            mat.wireframe = show;
        }
    }
    
    /**
     * Nettoyer ressources
     */
    dispose() {
        while (this.group.children.length > 0) {
            const child = this.group.children[0];
            this.group.remove(child);
            if (child.geometry) child.geometry.dispose();
        }
        this.scene.remove(this.group);
        this.voxelGeometry.dispose();
        for (const mat of Object.values(this.materials)) {
            mat.dispose();
        }
    }
}

// ============================================================================
// FACTORY & INTEGRATION
// ============================================================================

/**
 * Créer grille voxel pour environnement DIAMANTS
 */
export function createDiamantsVoxelGrid(config = {}) {
    const bounds = {
        minX: config.minX ?? -25,
        maxX: config.maxX ?? 25,
        minY: config.minY ?? 0,
        maxY: config.maxY ?? 15,
        minZ: config.minZ ?? -25,
        maxZ: config.maxZ ?? 25
    };
    
    const resolution = config.resolution ?? 0.5;
    
    return new VoxelGrid(bounds, resolution);
}

/**
 * Classe intégrée pour DIAMANTS
 */
export class EnvironmentVoxelizer {
    constructor(scene, config = {}) {
        this.grid = createDiamantsVoxelGrid(config);
        this.visualizer = scene ? new VoxelVisualizer(this.grid, scene) : null;
        
        // Cache des obstacles
        this.obstacles = [];
        this.drones = new Map();
    }
    
    /**
     * Ajouter obstacle et mettre à jour grille
     */
    addObstacle(type, ...params) {
        this.obstacles.push({ type, params });
        
        switch (type) {
            case 'sphere':
                this.grid.addSphere(...params);
                break;
            case 'box':
                this.grid.addBox(...params);
                break;
            case 'cylinder':
                this.grid.addCylinder(...params);
                break;
        }
    }
    
    /**
     * NOUVEAU: Peupler le terrain comme obstacles (sol + relief)
     * @param {Function} getHeightAt - (x, z) => y hauteur du terrain
     * @param {number} [margin=0.3] - Marge au-dessus du terrain (en mètres)
     */
    populateTerrain(getHeightAt, margin = 0.3) {
        if (!getHeightAt) return;
        
        const res = this.grid.resolution;
        let terrainCells = 0;
        
        // Parcourir toute la grille XZ
        for (let gx = 0; gx < this.grid.sizeX; gx++) {
            for (let gz = 0; gz < this.grid.sizeZ; gz++) {
                const worldX = this.grid.bounds.minX + (gx + 0.5) * res;
                const worldZ = this.grid.bounds.minZ + (gz + 0.5) * res;
                
                const terrainY = getHeightAt(worldX, worldZ);
                const minFlyY = terrainY + margin;
                
                // Marquer toutes les cellules en-dessous de minFlyY comme obstacles
                const maxGyObstacle = Math.floor((minFlyY - this.grid.bounds.minY) / res);
                
                for (let gy = 0; gy <= maxGyObstacle && gy < this.grid.sizeY; gy++) {
                    const idx = this.grid.toIndex(gx, gy, gz);
                    if (idx >= 0 && this.grid.state[idx] === CellState.FREE) {
                        this.grid.state[idx] = CellState.OBSTACLE;
                        this.grid.costMap[idx] = Infinity;
                        terrainCells++;
                    }
                }
            }
        }
        
        // Mettre à jour stats
        this.grid.stats.obstacleCells += terrainCells;
        this.grid.stats.freeCells -= terrainCells;
        
        logger.log('voxelizer', `🏔️ Terrain: ${terrainCells} cellules marquées comme obstacles`);
    }
    
    /**
     * Mettre à jour position drone
     */
    updateDrone(droneId, position) {
        this.drones.set(droneId, position);
        this.grid.updateDrone(droneId, position);
    }
    
    /**
     * Rafraîchir visualisation
     */
    refresh() {
        if (this.visualizer) {
            this.visualizer.update();
        }
    }
    
    /**
     * Obtenir statistiques
     */
    getStats() {
        return {
            ...this.grid.stats,
            totalCells: this.grid.totalCells,
            resolution: this.grid.resolution,
            dimensions: {
                x: this.grid.sizeX,
                y: this.grid.sizeY,
                z: this.grid.sizeZ
            }
        };
    }
}

// Exports
export default {
    CellState,
    CellColors,
    VoxelGrid,
    VoxelVisualizer,
    EnvironmentVoxelizer,
    createDiamantsVoxelGrid
};
