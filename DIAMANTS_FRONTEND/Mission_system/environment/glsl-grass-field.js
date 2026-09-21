/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Système d'herbe GLSL Procédurale
 * ================================================
 * Basé sur : https://github.com/Nitash-Biswas/grass-shader-glsl
 * - 200k+ brins d'herbe avec instancing
 * - Animation de vent avec Perlin Noise
 * - Système LOD (Level of Detail)
 * - Billboarding vers la caméra
 */

import * as THREE from 'three';
import grassVertexShader from '../shaders/grass-vertex-ultra.js';
import grassFragmentShader from '../shaders/grass-fragment-ultra.js';

export class GLSLGrassField {
    constructor(config = {}) {
        this.config = {
            fieldSize: config.fieldSize || 220,
            grassCount: config.grassCount || 250000,  // ULTRA density
            grassScale: config.grassScale || 0.9,     // ULTRA scale
            tipColor: config.tipColor || '#8fbc8f',
            baseColor: config.baseColor || '#228b22',
            fogColor: config.fogColor || '#87ceeb',
            ...config
        };
        
        this.halfWidth = 0.025;  // ULTRA - légèrement plus épais
        this.height = 0.95;      // ULTRA - plus haut
        this.lodThreshold = 50;
        this.camera = null;
        this.material = null;
        this.grassData = [];
        this.instancedMesh = null;
        
        // SYSTÈME DE CHUNKS DYNAMIQUES - ULTRA
        this.chunkSize = 20;     // Taille d'un chunk en unités
        this.renderDistance = 80; // ULTRA distance de rendu
        this.grassPerChunk = 12000; // ULTRA densité par chunk
        this.activeChunks = new Map();
        this.lastPlayerPos = new THREE.Vector3();
        this.chunkUpdateThreshold = 5; // Mise à jour si déplacement > 5 unités
    }

    // NOUVEAU : Calculer la normale du terrain pour orienter l'herbe
    getTerrainNormal(x, z) {
        if (!this.getTerrainHeight) return new THREE.Vector3(0, 1, 0);
        
        const offset = 0.1; // Petite distance pour calculer la pente
        
        // Échantillonner les hauteurs autour du point
        const heightC = this.getTerrainHeight(x, z);
        const heightL = this.getTerrainHeight(x - offset, z);
        const heightR = this.getTerrainHeight(x + offset, z);
        const heightD = this.getTerrainHeight(x, z - offset);
        const heightU = this.getTerrainHeight(x, z + offset);
        
        // Calculer les vecteurs tangents
        const tangentX = new THREE.Vector3(2 * offset, heightR - heightL, 0);
        const tangentZ = new THREE.Vector3(0, heightU - heightD, 2 * offset);
        
        // Produit vectoriel pour obtenir la normale
        const normal = new THREE.Vector3().crossVectors(tangentX, tangentZ).normalize();
        
        return normal;
    }

    createGrassGeometry(segments) {
        const taper = 0.005;
    const positions = [];
    const uvs = [];

        // Création des segments rectangulaires
        for (let i = 0; i < segments - 1; i++) {
            const y0 = (i / segments) * this.height;
            const y1 = ((i + 1) / segments) * this.height;

            // Deux triangles par segment
            positions.push(
                -this.halfWidth + taper * i, y0, 0,
                this.halfWidth - taper * i, y0, 0,
                -this.halfWidth + taper * (i + 1), y1, 0,

                -this.halfWidth + taper * (i + 1), y1, 0,
                this.halfWidth - taper * i, y0, 0,
                this.halfWidth - taper * (i + 1), y1, 0
            );

            // UVs correspondants (u: 0 à gauche, 1 à droite; v selon hauteur)
            const v0 = y0 / this.height;
            const v1 = y1 / this.height;
            uvs.push(
                0, v0,
                1, v0,
                0, v1,

                0, v1,
                1, v0,
                1, v1
            );
        }

        // Triangle de pointe
        positions.push(
            -this.halfWidth + taper * (segments - 1), ((segments - 1) / segments) * this.height, 0,
            this.halfWidth - taper * (segments - 1), ((segments - 1) / segments) * this.height, 0,
            0, this.height, 0
        );
        const vTipBase = ((segments - 1) / segments);
        uvs.push(
            0, vTipBase,
            1, vTipBase,
            0.5, 1.0
        );

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
        geometry.setAttribute('uv', new THREE.BufferAttribute(new Float32Array(uvs), 2));
        geometry.computeVertexNormals();
        
        return geometry;
    }

    createShaderMaterial() {
        this.material = new THREE.ShaderMaterial({
            vertexShader: grassVertexShader,
            fragmentShader: grassFragmentShader,
            uniforms: {
                uFrequency: { value: new THREE.Vector2(5, 5) },  // Ajouté du repo référence
                uTime: { value: 0 },
                uSpeed: { value: 1.4 },  // ULTRA - vent plus dynamique
                uTipColor: { value: new THREE.Color(this.config.tipColor) },
                uBaseColor: { value: new THREE.Color(this.config.baseColor) },
                uFogColor: { value: new THREE.Color(this.config.fogColor) },
                uHalfWidth: { value: this.halfWidth },
                uBladeHeight: { value: this.height },  // Ajouté du repo référence
            },
            side: THREE.DoubleSide,
        });

        return this.material;
    }

    // SYSTÈME DE CHUNKS OPTIMISÉ BASÉ SUR LE REPO ORIGINAL
    generateGrassData() {
        // Génération simple initiale pour le premier affichage
        this.grassData = [];
        const initialCount = Math.min(this.config.grassCount, 200000); // Limite raisonnable
        
        for (let i = 0; i < initialCount; i++) {
            this.grassData.push({
                x: (Math.random() - 0.5) * this.config.fieldSize,
                z: (Math.random() - 0.5) * this.config.fieldSize,
                rotation: Math.random() * Math.PI * 2,
            });
        }
        
        console.log(`🌱 Données d'herbe générées: ${this.grassData.length} brins`);
    }

    getChunkKey(x, z) {
        return `${Math.floor(x / this.chunkSize)},${Math.floor(z / this.chunkSize)}`;
    }

    generateChunkGrass(chunkX, chunkZ) {
        const grassInChunk = [];
        const startX = chunkX * this.chunkSize;
        const startZ = chunkZ * this.chunkSize;
        
        for (let i = 0; i < this.grassPerChunk; i++) {
            const x = startX + Math.random() * this.chunkSize;
            const z = startZ + Math.random() * this.chunkSize;
            
            grassInChunk.push({
                x,
                z,
                rotation: Math.random() * Math.PI * 2,
            });
        }
        
        return grassInChunk;
    }
    
    updateGrassChunks(playerPosition) {
        // Vérifier si le joueur s'est assez déplacé pour justifier une mise à jour
        if (this.lastPlayerPos.distanceTo(playerPosition) < this.chunkUpdateThreshold) {
            return; // Pas besoin de mise à jour
        }
        
        this.lastPlayerPos.copy(playerPosition);
        
        const playerChunkX = Math.floor(playerPosition.x / this.chunkSize);
        const playerChunkZ = Math.floor(playerPosition.z / this.chunkSize);
        const chunksRadius = Math.ceil(this.renderDistance / this.chunkSize);
        
        const neededChunks = new Set();
        
        // Déterminer quels chunks sont nécessaires
        for (let x = playerChunkX - chunksRadius; x <= playerChunkX + chunksRadius; x++) {
            for (let z = playerChunkZ - chunksRadius; z <= playerChunkZ + chunksRadius; z++) {
                const distance = Math.sqrt((x - playerChunkX) ** 2 + (z - playerChunkZ) ** 2);
                
                if (distance <= chunksRadius) {
                    const key = `${x},${z}`;
                    neededChunks.add(key);
                }
            }
        }
        
        // Supprimer les chunks trop éloignés
        for (const key of this.activeChunks.keys()) {
            if (!neededChunks.has(key)) {
                this.activeChunks.delete(key);
            }
        }
        
        // Ajouter les nouveaux chunks
        for (const key of neededChunks) {
            if (!this.activeChunks.has(key)) {
                const [x, z] = key.split(',').map(Number);
                const chunkGrass = this.generateChunkGrass(x, z);
                this.activeChunks.set(key, chunkGrass);
            }
        }
        
        // Reconstruire la liste globale d'herbe seulement si nécessaire
        this.grassData = [];
        for (const chunkGrass of this.activeChunks.values()) {
            this.grassData.push(...chunkGrass);
        }
        
        console.log(`🌱 Chunks actifs: ${this.activeChunks.size}, Herbe totale: ${this.grassData.length}`);
        
        // Re-créer les instances si les données ont changé
        if (this.highDetailRef && this.lowDetailRef) {
            this.setupAllInstances();
        }
    }
    
    createInstances() {
        if (!this.scene || !this.highDetailGeo || !this.lowDetailGeo || !this.material) {
            console.error("❌ Composants manquants pour créer les instances d'herbe");
            return;
        }
        
        // Nettoyer les anciennes instances
        if (this.highDetailRef) {
            this.scene.remove(this.highDetailRef);
            this.highDetailRef.geometry.dispose();
        }
        if (this.lowDetailRef) {
            this.scene.remove(this.lowDetailRef);
            this.lowDetailRef.geometry.dispose();
        }
        
        // Utiliser un nombre fixe d'instances comme dans le repo original
        const maxInstances = Math.min(this.grassData.length, this.config.grassCount);
        
        console.log(`🌱 Création instances: ${maxInstances} brins d'herbe`);
        
        // Créer les meshes instanciés avec le même nombre d'instances
        this.highDetailRef = new THREE.InstancedMesh(this.highDetailGeo, this.material, maxInstances);
        this.lowDetailRef = new THREE.InstancedMesh(this.lowDetailGeo, this.material, maxInstances);
        
        // Configuration
        this.highDetailRef.frustumCulled = false;
        this.lowDetailRef.frustumCulled = false;
        this.highDetailRef.castShadow = true;
        this.lowDetailRef.castShadow = true;
        this.highDetailRef.receiveShadow = true;
        this.lowDetailRef.receiveShadow = true;
        
        // Positionner toutes les instances initiales
        this.setupAllInstances();
        
        // Ajouter à la scène
        this.scene.add(this.highDetailRef);
        this.scene.add(this.lowDetailRef);
        
        console.log(`✅ Instances créées et ajoutées à la scène`);
    }

    async createGrassSystem(scene, camera, getTerrainHeight) {
        console.log("🌱 Initialisation du système d'herbe GLSL...");
        
        // Stocker les références nécessaires
        this.scene = scene;
        this.camera = camera;
        this.getTerrainHeight = getTerrainHeight;
        this.clock = new THREE.Clock();
        
        // Génération des géométries haute et basse qualité (comme le repo original)
        this.highDetailGeo = this.createGrassGeometry(7);  // 7 segments haute qualité
        this.lowDetailGeo = this.createGrassGeometry(3);   // 3 segments basse qualité (pas 1)

        // Création du matériau shader
        this.material = this.createShaderMaterial();

        // Génération initiale des données d'herbe
        this.generateGrassData();

        // Création initiale des instances
        this.createInstances();

        // Configuration des ombres
        if (this.highDetailRef) {
            this.highDetailRef.castShadow = true;
            this.highDetailRef.receiveShadow = true;
        }
        if (this.lowDetailRef) {
            this.lowDetailRef.castShadow = true;
            this.lowDetailRef.receiveShadow = true;
        }

        console.log("🌿 Système d'herbe GLSL initialisé avec", this.grassData.length, "brins");
        return { highDetail: this.highDetailRef, lowDetail: this.lowDetailRef };
    }

    setupAllInstances() {
        // Positionner toutes les instances d'un coup (plus simple et efficace)
        const dummy = new THREE.Object3D();
        
        for (let i = 0; i < this.grassData.length; i++) {
            const { x, z, rotation } = this.grassData[i];
            const terrainHeight = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
            
            dummy.position.set(x, terrainHeight, z);
            dummy.scale.set(this.config.grassScale, this.config.grassScale, this.config.grassScale);
            dummy.rotation.set(0, rotation, 0);
            dummy.updateMatrix();
            
            // Configurer les deux meshes avec la même position
            // Le LOD sera géré dans updateLOD()
            this.highDetailRef.setMatrixAt(i, dummy.matrix);
            this.lowDetailRef.setMatrixAt(i, dummy.matrix);
        }
        
        // Initialement, afficher tous en haute qualité
        this.highDetailRef.count = this.grassData.length;
        this.lowDetailRef.count = 0; // Aucun en basse qualité au début
        
        this.highDetailRef.instanceMatrix.needsUpdate = true;
        this.lowDetailRef.instanceMatrix.needsUpdate = true;
        
        console.log(`🎯 ${this.grassData.length} instances positionnées`);
    }

    updateColors(tipColor, baseColor, fogColor) {
        if (!this.material) return;
        
        if (tipColor) this.material.uniforms.uTipColor.value.set(tipColor);
        if (baseColor) this.material.uniforms.uBaseColor.value.set(baseColor);
        if (fogColor) this.material.uniforms.uFogColor.value.set(fogColor);
    }

    update(deltaTime, camera = null) {
        if (!this.material) return;

        // Animation du vent (toujours active pour fluidité)
        this.material.uniforms.uTime.value = this.clock.getElapsedTime();

        // Mise à jour des chunks si une caméra est fournie
        if (camera) {
            this.updateGrassChunks(camera.position);
            this.updateLOD(camera);
        }
    }

    updateLOD(camera) {
        if (!camera || !this.highDetailRef || !this.lowDetailRef) return;
        
        // LOD SIMPLE ET EFFICACE comme dans le repo original
        const dummy = new THREE.Object3D();
        let highIndex = 0;
        let lowIndex = 0;
        const LODDistance = 35; // ULTRA - distance LOD étendue
        
        for (let i = 0; i < this.grassData.length; i++) {
            const { x, z, rotation } = this.grassData[i];
            const distance = new THREE.Vector3(x, 0, z).distanceTo(camera.position);
            const terrainHeight = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;

            dummy.position.set(x, terrainHeight, z);
            dummy.scale.set(this.config.grassScale, this.config.grassScale, this.config.grassScale);
            dummy.rotation.y = rotation;
            dummy.updateMatrix();

            // Distribution LOD basée sur la distance
            if (distance < LODDistance) {
                if (highIndex < this.grassData.length) {
                    this.highDetailRef.setMatrixAt(highIndex++, dummy.matrix);
                }
            } else {
                if (lowIndex < this.grassData.length) {
                    this.lowDetailRef.setMatrixAt(lowIndex++, dummy.matrix);
                }
            }
        }

        // Mettre à jour les counts
        this.highDetailRef.count = highIndex;
        this.lowDetailRef.count = lowIndex;
        this.highDetailRef.instanceMatrix.needsUpdate = true;
        this.lowDetailRef.instanceMatrix.needsUpdate = true;
        
        // Debug occasionnel
        if (Math.random() < 0.01) { // 1% du temps
            console.log(`🔄 LOD: ${highIndex} haute qualité, ${lowIndex} basse qualité`);
        }
    }

    updateColors(tipColor, baseColor, fogColor) {
        if (!this.material) return;
        
        if (tipColor) this.material.uniforms.uTipColor.value.set(tipColor);
        if (baseColor) this.material.uniforms.uBaseColor.value.set(baseColor);
        if (fogColor) this.material.uniforms.uFogColor.value.set(fogColor);
    }

    dispose() {
        if (this.highDetailRef) {
            this.scene.remove(this.highDetailRef);
            this.highDetailRef.geometry.dispose();
            this.highDetailRef = null;
        }
        
        if (this.lowDetailRef) {
            this.scene.remove(this.lowDetailRef);
            this.lowDetailRef.geometry.dispose();
            this.lowDetailRef = null;
        }
        
        if (this.material) {
            this.material.dispose();
            this.material = null;
        }
    }
}
