/**
 * DIAMANTS - Système d'herbe GLSL Dynamique avec Gestion de Qualité
 * ===================================================================
 * Génération dynamique d'herbe basée sur la position du joueur
 * - Chunks de 20x20 unités avec 5000 brins chacun
 * - Génération/destruction automatique selon la distance
 * - Performance optimisée avec LOD
 * - Système de qualité dynamique intégré
 */

import * as THREE from 'three';
import { shaderQualityManager, QUALITY_LEVELS } from '../shaders/shader-quality-manager.js';

// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class GLSLGrassFieldDynamic {
    constructor(config = {}) {
        // Configuration OPTIMISÉE pour couverture maximale
        this.config = {
            grassPerChunk: 6000,         // Densité équilibrée
            chunkSize: 18,               // Chunks équilibrés
            renderDistance: 100,         // Distance AUGMENTÉE pour voir l'herbe au fond
            maxChunks: 20,              // Plus de chunks pour couvrir plus loin
            // LOD distances pour couverture maximale
            lodDistance: 35,            // LOD plus loin
            lodFarDistance: 70,         // Distance max très étendue
            // Couleurs mixtes vert-marron pour plus de réalisme
            tipColor: '#c8be9c',        // Beige clair naturel (repo original)
            baseColor: '#404709',       // Vert très foncé (repo original)
            fogColor: '#e6ebef',        // Gris bleu très pâle (repo original)
            // Nouvelles couleurs vert-marron
            brownTipColor: '#8b7355',   // Brun clair pour pointes sèches
            brownBaseColor: '#3d2f1f',  // Brun foncé pour bases
            mixRatio: 0.3              // 30% d'herbes brunes mélangées
        };
        
        // Dimensions exactes du repo référence
        this.halfWidth = 0.06;        // halfWidth du repo original
        this.height = 1.0;           // height du repo original
        this.camera = null;
        this.scene = null;
        this.material = null;
        this.getTerrainHeight = null;
        
        // Système de chunks avec LOD
        this.activeChunks = new Map();
        this.lastPlayerPos = new THREE.Vector3();
        this.updateThreshold = 6; // Mise à jour ENCORE moins fréquente pour économiser CPU
        
        // Système LOD comme dans le repo référence
        this.highDetailGeo = null;
        this.lowDetailGeo = null;
        
        // Suivi de la qualité
        this.lastGrassPerChunk = this.config.grassPerChunk;
        
        // Pool d'objets pour optimiser les performances
        this.chunkPool = [];
        this.maxPoolSize = 12; // Pool encore plus réduit pour économiser mémoire
        
        // Performance monitoring (nouveau)
        this.frameCount = 0;
        this.skipFrames = 1; // Moins de skip pour animation plus fluide avec haute densité
        // FPS monitor for UI panel
        this._perf = {
            frames: 0,
            lastSec: (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000,
            fps: 0
        };
    }

    createGrassGeometry(segments = 7) {
        // Géométrie EXACTE du repo référence avec pointe triangulaire
        const positions = [];
        const halfWidth = this.halfWidth;
        const height = this.height;
        const taper = 0.005; // Taper exact du repo référence

        // Création des segments comme dans le repo référence
        for (let i = 0; i < segments; i++) {
            const y = (i / (segments - 1)) * height;
            const ratio = i / (segments - 1);
            
            // Largeur qui diminue vers le haut (taper)
            let width = halfWidth * (1 - ratio * taper * 10); // Réduction progressive
            
            // POINTE TRIANGULAIRE : la dernière section forme un triangle
            if (i === segments - 1) {
                // Pointe triangulaire EXACTE du repo référence
                positions.push(0, y, 0); // Point central de la pointe
                continue;
            }
            
            // Segments normaux avec deux points (gauche et droite)
            if (i < segments - 1) {
                const yNext = ((i + 1) / (segments - 1)) * height;
                const ratioNext = (i + 1) / (segments - 1);
                let widthNext = halfWidth * (1 - ratioNext * taper * 10);
                
                // TRIANGLE STRIP pour performance optimale
                if (i === segments - 2) {
                    // Avant-dernière section qui connecte à la pointe
                    positions.push(
                        -width, y, 0,      // Gauche actuel
                        width, y, 0,       // Droite actuel
                        0, yNext, 0        // POINTE triangulaire
                    );
                } else {
                    // Sections normales (rectangulaires)
                    positions.push(
                        -width, y, 0,         // Gauche actuel
                        width, y, 0,          // Droite actuel
                        -widthNext, yNext, 0, // Gauche suivant
                        
                        -widthNext, yNext, 0, // Gauche suivant
                        width, y, 0,          // Droite actuel
                        widthNext, yNext, 0   // Droite suivant
                    );
                }
            }
        }

        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute("position", new THREE.BufferAttribute(new Float32Array(positions), 3));
        geometry.computeVertexNormals();
        return geometry;
    }

    async createShaderMaterial() {
        // Obtenir les shaders selon la qualité actuelle
        const shaderConfig = await shaderQualityManager.getShaders();
        
        log(`🎨 Création matériau herbe - Qualité: ${shaderQualityManager.currentQuality}`);
        log(`   Features: ${JSON.stringify(shaderConfig.features)}`);
        
        // Ajuster la configuration selon la qualité
        if (shaderConfig.features.grassPerChunk) {
            this.config.grassPerChunk = shaderConfig.features.grassPerChunk;
        }
        
        // Create material with only valid THREE.js properties
        const materialConfig = {
            vertexShader: shaderConfig.vertex,
            fragmentShader: shaderConfig.fragment,
            uniforms: {
                uFrequency: { value: new THREE.Vector2(5, 5) },  // Ajouté pour mouvement plus naturel
                uTime: { value: 0 },
                uSpeed: { value: 800 }, // Base plus rapide
                uTipColor: { value: new THREE.Color(this.config.tipColor) },
                uBaseColor: { value: new THREE.Color(this.config.baseColor) },
                uFogColor: { value: new THREE.Color(this.config.fogColor) },
                uHalfWidth: { value: this.halfWidth },
                uBladeHeight: { value: this.height },
                // New: camera-aware brown visibility
                uCameraPos: { value: new THREE.Vector3(0,0,0) },
                uBrownNear: { value: 25 }, // fully visible within 25m
                uBrownFar: { value: 55 }   // disappears beyond 55m
            },
            side: THREE.DoubleSide,
            transparent: true,
        };

        this.material = new THREE.ShaderMaterial(materialConfig);
        
        // Appliquer accélération du vent selon la qualité
        if (shaderConfig.features.windMultiplier && this.material?.uniforms?.uSpeed) {
            this.material.uniforms.uSpeed.value *= shaderConfig.features.windMultiplier;
        }

        return this.material;
    }

        // SYSTÈME LOD COMME LE REPO RÉFÉRENCE : créer chunks avec haute/basse qualité
    generateChunk(chunkX, chunkZ) {
        if (!this.material) {
            error("❌ Matériau d'herbe manquant lors de la génération du chunk");
            return null;
        }
        
        // SYSTÈME OPTIMISÉ : Un seul mesh par chunk avec LOD dynamique
        if (!this.grassGeometry) {
            this.grassGeometry = this.createGrassGeometry(7); // 7 segments comme le repo référence
        }
        
        const mesh = new THREE.InstancedMesh(this.grassGeometry, this.material, this.config.grassPerChunk);
        
        const startX = chunkX * this.config.chunkSize;
        const startZ = chunkZ * this.config.chunkSize;
        
        // Population optimisée des instances
        this.populateGrassInstances(mesh, startX, startZ, this.config.grassPerChunk);
        
        // Configuration optimisée
        mesh.instanceMatrix.needsUpdate = true;
        mesh.frustumCulled = true; // Réactivé pour performance
        mesh.castShadow = false;   // Désactivé pour performance
        mesh.receiveShadow = true;
        
        return {
            mesh: mesh,
            chunkX,
            chunkZ,
            instanceCount: this.config.grassPerChunk
        };
    }

    // MÉTHODE OPTIMISÉE : Population d'instances comme le repo référence
    populateGrassInstances(mesh, startX, startZ, grassCount) {
        const dummy = new THREE.Object3D();
        
        for (let i = 0; i < grassCount; i++) {
            // Distribution uniforme optimisée
            const x = startX + (Math.random() - 0.5) * this.config.chunkSize;
            const z = startZ + (Math.random() - 0.5) * this.config.chunkSize;
            
            // Hauteur du terrain
            const y = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
            
            // Exclure l'herbe si hauteur spéciale (-999)
            if (y === -999) continue;
            
            dummy.position.set(x, y, z);
            
            // Rotation simple pour performance
            dummy.rotation.y = Math.random() * Math.PI * 2;
            
            // Inclinaisons subtiles réalistes
            dummy.rotation.x = (Math.random() - 0.5) * 0.2; // Plus subtil
            dummy.rotation.z = (Math.random() - 0.5) * 0.2;
            
            // Échelle uniforme pour pelouse homogène
            const scale = 0.9 + Math.random() * 0.2; // 0.9 à 1.1
            dummy.scale.set(scale, scale, scale);
            
            dummy.updateMatrix();
            mesh.setMatrixAt(i, dummy.matrix);
        }
        
        mesh.instanceMatrix.needsUpdate = true;
    }

    // Interface compatible avec AuthenticProvencalEnvironment
    async createGrassSystem(scene, getTerrainHeightFunc, camera) {
        this.scene = scene;
        this.getTerrainHeight = getTerrainHeightFunc;
        this.camera = camera;
        
        // Créer le matériau shader
        this.material = await this.createShaderMaterial();
        
        // Démarrer l'animation de vent immédiatement
        this.startWindAnimation();
        
        log("🌿 Système d'herbe dynamique initialisé avec animation");
        log(`📊 Config: ${this.config.grassPerChunk} brins/chunk, rayon ${this.config.renderDistance}m`);
        
        // Génération initiale autour de l'origine (0,0,0) même si pas de caméra
        const initialPosition = camera ? camera.position : new THREE.Vector3(0, 0, 0);
        this.updateChunks(initialPosition);
        
        return {
            update: (time, cameraPosition) => this.update(time, cameraPosition),
            dispose: () => this.dispose()
        };
    }
    
    // Méthode de mise à jour appelée par l'environnement (ULTRA-OPTIMISÉE)
    update(time, cameraPosition) {
        if (!this.material) return;
        
        // PERFORMANCE : Skip frames pour réduire CPU load
        this.frameCount++;
        // FPS accumulation
        try {
            const nowSec = (typeof performance !== 'undefined' ? performance.now() : Date.now()) / 1000;
            this._perf.frames += 1;
            const elapsed = nowSec - this._perf.lastSec;
            if (elapsed >= 1.0) {
                this._perf.fps = this._perf.frames / elapsed;
                this._perf.frames = 0;
                this._perf.lastSec = nowSec;
            }
        } catch (_) {}
        if (this.frameCount % this.skipFrames !== 0) {
            // Animation uniquement sur les frames importantes
            const currentTime = time || Date.now();
            this.material.uniforms.uTime.value = currentTime * 0.001;
            return;
        }
        
        // Assurer l'animation continue même sans temps fourni
        const currentTime = time || Date.now();
        this.material.uniforms.uTime.value = currentTime * 0.001;
        
        // Mise à jour des chunks et uniforms liés à la caméra si fournie
        if (cameraPosition) {
            // Update camera position uniform for brown culling in shader
            if (this.material.uniforms.uCameraPos) {
                this.material.uniforms.uCameraPos.value.copy(cameraPosition);
            }
            this.updateChunks(cameraPosition);
        }
        
        // Optimisation CPU : Vérification de visibilité par caméra frustum (réduite)
        this.optimizeChunkVisibility();
    }
    
    // OPTIMISATION SIMPLIFIÉE : Culling de base sans calculs complexes
    optimizeChunkVisibility() {
        if (!this.camera) return;
        
        // PERFORMANCE : Skip visibility check sur certains frames
        if (this.frameCount % 4 !== 0) return;
        
        const cameraPos = this.camera.position;
        
        // Optimisation simple par distance seulement
        this.activeChunks.forEach(chunk => {
            if (chunk.mesh) {
                const chunkWorldX = chunk.chunkX * this.config.chunkSize;
                const chunkWorldZ = chunk.chunkZ * this.config.chunkSize;
                
                // Distance simple sans sqrt pour performance
                const distSq = (cameraPos.x - chunkWorldX) ** 2 + (cameraPos.z - chunkWorldZ) ** 2;
                const maxDistSq = this.config.renderDistance ** 2;
                
                // Culling simple
                chunk.mesh.visible = distSq < maxDistSq;
            }
        });
    }
    
    // Démarrer l'animation de vent automatique
    startWindAnimation() {
        if (!this.material) return;
        
        const animate = () => {
            if (this.material && this.material.uniforms.uTime) {
                const time = Date.now();
                this.material.uniforms.uTime.value = time * 0.001;
                requestAnimationFrame(animate);
            }
        };
        animate();
        log('🌬️ Animation de vent démarrée - vitesse:', this.material.uniforms.uSpeed.value);
    }
    
    // Met à jour les chunks selon la position du joueur
    updateChunks(playerPosition) {
        const currentPos = playerPosition.clone();
        
        // Vérifier si le joueur s'est assez déplacé (sauf pour la première fois)
        if (this.activeChunks.size > 0 && this.lastPlayerPos.distanceTo(currentPos) < this.updateThreshold) {
            return;
        }
        
        this.lastPlayerPos.copy(currentPos);
        
        // Calculer les chunks nécessaires
        const centerChunkX = Math.floor(currentPos.x / this.config.chunkSize);
        const centerChunkZ = Math.floor(currentPos.z / this.config.chunkSize);
        const chunkRadius = Math.ceil(this.config.renderDistance / this.config.chunkSize);
        
        const neededChunks = new Set();
        
        // Déterminer quels chunks sont nécessaires (optimisé)
        for (let x = centerChunkX - chunkRadius; x <= centerChunkX + chunkRadius; x++) {
            for (let z = centerChunkZ - chunkRadius; z <= centerChunkZ + chunkRadius; z++) {
                const distance = Math.sqrt((x - centerChunkX) ** 2 + (z - centerChunkZ) ** 2);
                if (distance <= chunkRadius) {
                    neededChunks.add(`${x},${z}`);
                }
            }
        }
        
        // Supprimer les chunks trop éloignés
        for (const [key, chunk] of this.activeChunks) {
            if (!neededChunks.has(key)) {
                this.scene.remove(chunk.mesh);
                // Nettoyer les mesh dans le groupe (double couche)
                if (chunk.mesh.children) {
                    chunk.mesh.children.forEach(childMesh => {
                        if (childMesh.geometry) childMesh.geometry.dispose();
                    });
                }
                this.recycleChunk(chunk);
                this.activeChunks.delete(key);
            }
        }
        
        // Ajouter les nouveaux chunks
        let newChunksAdded = 0;
        for (const key of neededChunks) {
            if (!this.activeChunks.has(key)) {
                const [x, z] = key.split(',').map(Number);
                const chunk = this.generateChunk(x, z);
                
                if (chunk && chunk.mesh) {
                    this.activeChunks.set(key, chunk);
                    this.scene.add(chunk.mesh);
                    newChunksAdded++;
                }
            }
        }
        
        // Log optimisé (seulement si changements)
        if (newChunksAdded > 0 || this.activeChunks.size !== neededChunks.size) {
            const totalGrass = this.activeChunks.size * (this.config.grassPerChunk + Math.floor(this.config.grassPerChunk * 0.7) + Math.floor(this.config.grassPerChunk * 0.5));
            log(`🌱 PELOUSE OPTIMISÉE: ${this.activeChunks.size} chunks (${newChunksAdded} nouveaux), ~${totalGrass} brins - Performance équilibrée`);
        }
    }

    // Système de recyclage des chunks pour optimiser la mémoire
    recycleChunk(chunk) {
        if (this.chunkPool.length < this.maxPoolSize) {
            this.chunkPool.push(chunk);
        }
    }

    populateGrassLayer(mesh, startX, startZ, dummy, grassCount, scale = 1.0) {
        for (let i = 0; i < grassCount; i++) {
            // Distribution plus serrée pour vraie pelouse dense
            const x = startX + Math.random() * this.config.chunkSize;
            const z = startZ + Math.random() * this.config.chunkSize;
            
            // Ajouter quelques brins en grille pour assurer la couverture
            if (i < grassCount * 0.3) {
                const gridDensity = Math.sqrt(grassCount * 0.3);
                const gridX = i % gridDensity;
                const gridZ = Math.floor(i / gridDensity);
                const offsetX = (gridX / gridDensity) * this.config.chunkSize + (Math.random() - 0.5) * 0.5;
                const offsetZ = (gridZ / gridDensity) * this.config.chunkSize + (Math.random() - 0.5) * 0.5;
                const gridY = this.getTerrainHeight ? this.getTerrainHeight(startX + offsetX, startZ + offsetZ) : 0;
                
                // Exclure l'herbe si hauteur spéciale (-999)
                if (gridY === -999) continue;
                
                dummy.position.set(startX + offsetX, gridY, startZ + offsetZ);
            } else {
                // Position aléatoire normale
                const y = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
                
                // Exclure l'herbe si hauteur spéciale (-999)
                if (y === -999) continue;
                
                dummy.position.set(x, y, z);
            }
            
            // Rotation aléatoire
            dummy.rotation.y = Math.random() * Math.PI * 2;
            
            // Échelle avec variation pour brins plus naturels
            const scaleVariation = 0.7 + Math.random() * 0.5; // 0.7 à 1.2
            dummy.scale.set(
                scale * scaleVariation,
                scale * scaleVariation * (0.8 + Math.random() * 0.4), // Hauteur variable 0.8-1.2
                scale * scaleVariation
            );
            
            // Appliquer la transformation à l'instance
            dummy.updateMatrix();
            mesh.setMatrixAt(i, dummy.matrix);
        }
    }

    updateColors(tipColor, baseColor, fogColor) {
        if (!this.material) return;
        
        if (tipColor) this.material.uniforms.uTipColor.value.set(tipColor);
        if (baseColor) this.material.uniforms.uBaseColor.value.set(baseColor);
        if (fogColor) this.material.uniforms.uFogColor.value.set(fogColor);
    }
    
    // Nouvelle méthode pour ajuster le vent
    setWindSpeed(speed) {
        if (this.material && this.material.uniforms.uSpeed) {
            let s = speed;
            try {
                const features = (shaderQualityManager.currentShaders && shaderQualityManager.currentShaders.features) || {};
                if (features.windMultiplier) s *= features.windMultiplier;
            } catch (_) {}
            this.material.uniforms.uSpeed.value = s;
            log('🌬️ Vitesse du vent ajustée à:', speed);
        }
    }

    dispose() {
        // Nettoyer tous les chunks actifs
        for (const [key, chunk] of this.activeChunks) {
            this.scene.remove(chunk.mesh);
            chunk.mesh.geometry.dispose();
        }
        this.activeChunks.clear();
        
        // Nettoyer le pool
        for (const chunk of this.chunkPool) {
            if (chunk.mesh.geometry) {
                chunk.mesh.geometry.dispose();
            }
        }
        this.chunkPool.length = 0;
        
        if (this.material) {
            this.material.dispose();
            this.material = null;
        }
        
        log('🧹 Système d\'herbe dynamique nettoyé');
    }

    /**
     * Change la qualité dynamiquement
     */
    async setQuality(quality) {
        log(`🎨 Changement qualité herbe: ${quality}`);
        
        try {
            await shaderQualityManager.setQuality(quality);
            // Recréer le matériau avec la nouvelle qualité
            await this.recreateMaterial();
        } catch (error) {
            error('Erreur lors du changement de qualité:', error);
        }
    }

    /**
     * Active/désactive la qualité automatique
     */
    setAutoQuality(enabled) {
        shaderQualityManager.setAutoQuality(enabled);
    }

    /**
     * Recrée le matériau avec la nouvelle qualité
     */
    async recreateMaterial() {
        if (!this.material) return;
        
        // Sauvegarder les uniforms actuels
        const currentUniforms = this.material.uniforms;
        
        // Disposer l'ancien matériau
        this.material.dispose();
        
        // Créer le nouveau matériau
        this.material = await this.createShaderMaterial();
        
        // Restaurer les valeurs des uniforms
        Object.keys(currentUniforms).forEach(key => {
            if (this.material.uniforms[key]) {
                this.material.uniforms[key].value = currentUniforms[key].value;
            }
        });
        
        // Mettre à jour tous les chunks existants
        this.activeChunks.forEach((chunk) => {
            if (chunk.mesh) {
                chunk.mesh.material = this.material;
            }
        });
        
        // Régénérer les chunks avec la nouvelle densité si nécessaire
        const shaderConfig = await shaderQualityManager.getShaders();
        if (shaderConfig.features.grassPerChunk && 
            shaderConfig.features.grassPerChunk !== this.lastGrassPerChunk) {
            
            this.lastGrassPerChunk = shaderConfig.features.grassPerChunk;
            log(`🔄 Régénération chunks avec ${this.lastGrassPerChunk} brins`);
            
            // Forcer la régénération des chunks
            this.clearAllChunks();
            const fallbackPos = (this.camera && this.camera.position) || this.lastPlayerPos || new THREE.Vector3(0, 0, 0);
            this.updateChunks(fallbackPos);
        }
        
        log('✅ Matériau herbe recréé avec nouvelle qualité');
    }
    
    /**
     * Nettoie tous les chunks
     */
    clearAllChunks() {
        this.activeChunks.forEach((chunk) => {
            if (chunk.mesh) {
                this.scene.remove(chunk.mesh);
                if (chunk.mesh.geometry) {
                    chunk.mesh.geometry.dispose();
                }
            }
        });
        this.activeChunks.clear();
    }

    /**
     * Obtient les informations de qualité actuelles
     */
    getQualityInfo() {
        return {
            current: shaderQualityManager.currentQuality,
            features: (shaderQualityManager.currentShaders && shaderQualityManager.currentShaders.features) || {},
            availableLevels: shaderQualityManager.getQualityInfo(),
            activeChunks: this.activeChunks.size,
            totalGrassInstances: this.activeChunks.size * this.config.grassPerChunk,
            fps: this._perf?.fps || 0
        };
    }

    /**
     * Détection automatique de la qualité optimale
     */
    async detectOptimalQuality(renderer) {
        // Fallback vers un renderer global si non fourni
        if (!renderer && typeof window !== 'undefined' && window.renderer) {
            renderer = window.renderer;
        }
        const optimalQuality = await shaderQualityManager.detectOptimalQuality(renderer);
    // Attendre la fin du changement de qualité pour garantir la cohérence UI/scene
    await this.setQuality(optimalQuality);
        return optimalQuality;
    }
}
