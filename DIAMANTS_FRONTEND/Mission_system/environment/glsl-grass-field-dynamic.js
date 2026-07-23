/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
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
        // Configuration pour couverture maximale
        this.config = {
            grassPerChunk: 6000,
            // ⚠️ VALEURS PAR DÉFAUT, EN PRATIQUE ÉCRASÉES.
            // terrain-environment.js construit le champ avec chunkSize 20 et
            // renderDistance 90, et la config de l'appelant l'emporte à la fusion.
            // Les valeurs ci-dessous ne s'appliquent donc que si le champ est
            // instancié sans configuration — ce qu'aucun appelant ne fait
            // aujourd'hui. Les 20/90 effectifs donnent chunkRadius 5, soit une
            // soixantaine de chunks chargés, et non les 13 que suggéraient ces
            // constantes. À corriger en alignant les deux fichiers, pas en
            // retouchant ce commentaire.
            chunkSize: 44,
            renderDistance: 88,
            // Chunks générés par frame. Était implicitement à 1, alors que le
            // déchargement, lui, vide tous les chunks hors portée d'un seul coup :
            // l'asymétrie laissait des carrés sans herbe derrière la caméra.
            chunksPerFrame: 3,
            // LOD distances pour couverture maximale
            lodDistance: 35,
            lodFarDistance: 70,
            // Couleurs mixtes vert-marron pour plus de réalisme
            tipColor: '#9cbf6a',        // Pointe VERT clair (fini le beige pâle)
            baseColor: '#3e5720',       // Base vert profond
            fogColor: '#aebb92',        // Vert doux au loin (fini le gris-blanc délavé)
            // Nouvelles couleurs vert-marron
            brownTipColor: '#8b7355',   // Brun clair pour pointes sèches
            brownBaseColor: '#3d2f1f',  // Brun foncé pour bases
            mixRatio: 0.12             // 12% d'herbes brunes (le reste bien vert)
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
        this.updateThreshold = 14; // Seuil plus large => moins de load/unload de chunks => moins de popping visible
        
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
        
        // Ajuster la configuration selon la qualité — PLAFONNÉ pour tenir le FPS
        // (la qualité auto peut monter à 14000/chunk = essaim de brins qui tue le FPS).
        if (shaderConfig.features.grassPerChunk) {
            // Densité pilotée par la qualité, mais avec un PLANCHER : en qualité LOW
            // (2000) sur des chunks de 44 m on tombe à ~1 brin/m² => herbe invisible.
            // Le nombre de brins doit suivre la SURFACE du chunk, sinon agrandir les
            // chunks (pour couvrir plus loin) dilue mécaniquement l'herbe.
            const area = this.config.chunkSize * this.config.chunkSize;
            const minForDensity = Math.round(area * 2.6); // ~2.6 brins/m² minimum
            this.config.grassPerChunk = Math.min(
                Math.max(shaderConfig.features.grassPerChunk, minForDensity),
                9000
            );
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
                // Fondu de bord : les brins rapetissent sur le dernier tiers de la
                // portée => plus de frontière carrée herbe/sol nu.
                uFadeStart: { value: this.config.renderDistance * 0.62 },
                uFadeEnd: { value: this.config.renderDistance * 0.98 },
                uBladeHeight: { value: this.height },
                // New: camera-aware brown visibility
                uCameraPos: { value: new THREE.Vector3(0,0,0) },
                uBrownNear: { value: 25 }, // fully visible within 25m
                uBrownFar: { value: 55 }   // disappears beyond 55m
            },
            side: THREE.DoubleSide,
            transparent: false, // brins OPAQUES (alpha=1) => depth-write + early-z, fini l'overdraw
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
            this.grassGeometry = this.createGrassGeometry(4); // 4 segments (au lieu de 7) => moins de sommets, FPS++
        }
        
        const mesh = new THREE.InstancedMesh(this.grassGeometry, this.material, this.config.grassPerChunk);
        
        const startX = chunkX * this.config.chunkSize;
        const startZ = chunkZ * this.config.chunkSize;
        
        // Population optimisée des instances
        this.populateGrassInstances(mesh, startX, startZ, this.config.grassPerChunk);
        
        // Configuration optimisée
        mesh.instanceMatrix.needsUpdate = true;

        // ⚠️ CORRECTIF TROUS D'HERBE : les positions des brins vivent dans les matrices
        // d'instances, mais le mesh reste à l'origine du monde. Sans recalcul, Three.js
        // déduit la sphère englobante de la géométrie d'UN brin à l'origine (0,0,0) et
        // élimine le chunk entier dès que la caméra ne cadre plus l'origine — d'où des
        // pans d'herbe qui disparaissent. computeBoundingSphere() prend les vraies
        // bornes issues des instances : le culling redevient correct.
        mesh.computeBoundingSphere();

        // Culling RÉACTIVÉ. Il avait été coupé quand la sphère englobante était
        // fausse (elle ne couvrait qu'un brin à l'origine) : le culling supprimait
        // alors des pans d'herbe entiers. Avec le computeBoundingSphere() ci-dessus
        // la sphère est juste, et le gain n'est plus négligeable — un chunk pèse
        // 6000 brins passés dans un vertex shader lourd, et à peu près la moitié
        // des chunks est hors champ à tout instant.
        //
        // MARGE : le vertex shader déplace les brins (vent, courbure, billboard)
        // au-delà des bornes géométriques dont la sphère est déduite. On l'élargit
        // pour que les brins de bordure ne clignotent pas en limite de champ.
        if (mesh.boundingSphere) {
            mesh.boundingSphere.radius += 2.0;
        }
        mesh.frustumCulled = true;
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
        let validCount = 0;
        
        for (let i = 0; i < grassCount; i++) {
            // Distribution uniforme optimisée
            const x = startX + (Math.random() - 0.5) * this.config.chunkSize;
            const z = startZ + (Math.random() - 0.5) * this.config.chunkSize;
            
            // Hauteur du terrain
            const y = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
            
            // Exclure l'herbe si hauteur spéciale (-999) — zone héliport
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
            mesh.setMatrixAt(validCount, dummy.matrix);
            validCount++;
        }
        
        // Only render valid instances — hide the rest
        mesh.count = validCount;
        mesh.instanceMatrix.needsUpdate = true;
    }

    // Interface compatible avec TerrainEnvironment
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
    
    // Met à jour les chunks selon la position de la caméra.
    // STREAMING : construit AU PLUS 1 chunk (le plus proche) par frame — comme
    // updateTreeChunks — pour ne JAMAIS bloquer le rendu. generateChunk() alloue
    // un InstancedMesh de plusieurs milliers de brins ; tout ajouter d'un coup
    // gelait la frame lors des déplacements. Le déchargement reste immédiat (cheap).
    updateChunks(playerPosition) {
        const cs = this.config.chunkSize;
        const rd = this.config.renderDistance;
        const chunkRadius = Math.ceil(rd / cs);
        // Centre du chunk placé en x*cs (cf. populateGrassInstances) → round, pas floor
        const cx0 = Math.round(playerPosition.x / cs);
        const cz0 = Math.round(playerPosition.z / cs);

        const missing = [];
        for (let x = cx0 - chunkRadius; x <= cx0 + chunkRadius; x++) {
            for (let z = cz0 - chunkRadius; z <= cz0 + chunkRadius; z++) {
                const dx = x * cs - playerPosition.x;
                const dz = z * cs - playerPosition.z;
                const d2 = dx * dx + dz * dz;
                if (d2 <= rd * rd) {
                    const key = `${x},${z}`;
                    if (!this.activeChunks.has(key)) missing.push({ x, z, d2, key });
                }
            }
        }

        // ── Décharger les chunks hors portée ──
        // ⚠️ FUITE MÉMOIRE GPU CORRIGÉE : un InstancedMesh détient un buffer
        // `instanceMatrix` (grassPerChunk × 64 octets ≈ 512 Ko/chunk) que le garbage
        // collector JS ne libère PAS — Three.js impose un dispose() explicite. Sans lui,
        // chaque chunk déchargé fuyait son buffer : en se déplaçant, la VRAM se
        // remplissait jusqu'à écrouler le FPS.
        // dispose() sur l'InstancedMesh ne touche NI la géométrie NI le matériau
        // (partagés entre tous les chunks) — uniquement ses buffers d'instances.
        //
        // HYSTÉRÉSIS : on garde un chunk jusqu'à 1.3 × la portée de chargement.
        // Sans cette marge, une caméra qui oscille autour d'une frontière de chunk
        // déchargeait puis rechargeait le même chunk en boucle.
        const keepRange = rd * 1.3;
        const keepRange2 = keepRange * keepRange;
        for (const [key, chunk] of this.activeChunks) {
            const dx = chunk.chunkX * cs - playerPosition.x;
            const dz = chunk.chunkZ * cs - playerPosition.z;
            if (dx * dx + dz * dz > keepRange2) {
                this.scene.remove(chunk.mesh);
                if (typeof chunk.mesh.dispose === 'function') chunk.mesh.dispose();
                this.activeChunks.delete(key);
            }
        }

        // ── Charger les chunks manquants, les plus proches d'abord ──
        // ⚠️ CORRECTIF "CARRÉS SANS HERBE" : le déchargement ci-dessus est immédiat
        // et porte sur TOUS les chunks hors portée, alors que le chargement était
        // plafonné à UN chunk par frame. À 15 fps, recouvrir 13 chunks demandait
        // presque une seconde — d'où les carrés vides qui traînaient derrière la
        // caméra dès qu'on se déplaçait. Les deux côtés doivent avoir un débit
        // comparable.
        if (missing.length) {
            missing.sort((a, b) => a.d2 - b.d2);
            const budget = Math.min(this.config.chunksPerFrame, missing.length);
            for (let i = 0; i < budget; i++) {
                const m = missing[i];
                const chunk = this.generateChunk(m.x, m.z);
                if (chunk && chunk.mesh) {
                    this.activeChunks.set(m.key, chunk);
                    this.scene.add(chunk.mesh);
                }
            }
        }
        this.lastPlayerPos.copy(playerPosition);
    }

    // Système de recyclage des chunks pour optimiser la mémoire
    recycleChunk(chunk) {
        if (this.chunkPool.length < this.maxPoolSize) {
            this.chunkPool.push(chunk);
        }
    }

    populateGrassLayer(mesh, startX, startZ, dummy, grassCount, scale = 1.0) {
        for (let i = 0; i < grassCount; i++) {
            // Distribution centrée comme populateGrassInstances (±chunkSize/2)
            const x = startX + (Math.random() - 0.5) * this.config.chunkSize;
            const z = startZ + (Math.random() - 0.5) * this.config.chunkSize;
            
            // Ajouter quelques brins en grille pour assurer la couverture
            if (i < grassCount * 0.3) {
                const gridDensity = Math.sqrt(grassCount * 0.3);
                const gridX = i % gridDensity;
                const gridZ = Math.floor(i / gridDensity);
                // Centré sur startX/startZ (±chunkSize/2) pour cohérence avec la couche principale
                const offsetX = ((gridX / gridDensity) - 0.5) * this.config.chunkSize + (Math.random() - 0.5) * 0.5;
                const offsetZ = ((gridZ / gridDensity) - 0.5) * this.config.chunkSize + (Math.random() - 0.5) * 0.5;
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
        // ⚠️ La géométrie de brin est PARTAGÉE par tous les chunks : la disposer par
        // chunk (ancien code) la détruisait pour tous les autres. On libère ici les
        // buffers d'instances de chaque chunk, puis la géométrie partagée UNE seule fois.
        for (const [, chunk] of this.activeChunks) {
            this.scene.remove(chunk.mesh);
            if (typeof chunk.mesh.dispose === 'function') chunk.mesh.dispose();
        }
        this.activeChunks.clear();

        for (const chunk of this.chunkPool) {
            if (chunk?.mesh && typeof chunk.mesh.dispose === 'function') chunk.mesh.dispose();
        }
        this.chunkPool.length = 0;

        // Géométrie partagée : une seule libération
        if (this.grassGeometry) { this.grassGeometry.dispose(); this.grassGeometry = null; }
        
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
        // ⚠️ Ne PAS disposer chunk.mesh.geometry : elle est partagée par tous les chunks
        // (et par ceux recréés ensuite). On libère uniquement les buffers d'instances,
        // sinon changement de qualité = géométrie détruite + fuite des instanceMatrix.
        this.activeChunks.forEach((chunk) => {
            if (chunk.mesh) {
                this.scene.remove(chunk.mesh);
                if (typeof chunk.mesh.dispose === 'function') chunk.mesh.dispose();
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
