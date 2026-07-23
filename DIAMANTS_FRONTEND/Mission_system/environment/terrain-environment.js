/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Environnement Provençal Authentique avec EZ-Tree
 * =============================================================
 *
 * Utilise la vraie API EZ-Tree avec presets JSON et textures haute qualité
 * Heliport rayon 8m, exclusion herbe 10m, exclusion arbres 12m, pas de murs d'arène.
 */

import * as THREE from 'three';
// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Import depuis les sources EZ-Tree
import { Tree } from '../third-party/ez-tree/src/lib/tree.js';
import { TreePreset, loadPreset } from '../third-party/ez-tree/src/lib/presets/index.js';
import { SkyboxShaders, SkyboxDefaults } from './skybox.js';
import { GLSLGrassFieldDynamic } from './glsl-grass-field-dynamic.js';
import QualityControlPanel from './quality-control-panel.js';
import { QUALITY_LEVELS } from '../shaders/shader-quality-manager.js';
import { FallenLeavesSystem } from './fallen-leaves.js';
import { ForestWoodSystem } from './forest-wood.js';
import { UndergrowthSystem } from './undergrowth.js';
import { TreeTextureEnhancer } from './tree-texture-enhancer.js';
import { CollisionDetection } from '../physics/collision-detection.js';

export class TerrainEnvironment {
    constructor(scene, config = {}) {
        this.scene = scene;
        this.config = {
            terrainSize: config.terrainSize || { x: 500, y: 500 },
            forestDensity: config.forestDensity ?? 0.9, // Réduction densité forêt
            maxTrees: config.maxTrees || 200,
            minTreeSpacing: config.minTreeSpacing || 30.0,
            enableSkybox: config.enableSkybox !== false,
            enableTerrain: config.enableTerrain !== false,
            enableForest: config.enableForest !== false,
            enableGrass: config.enableGrass !== false,
            enableFallenLeaves: config.enableFallenLeaves !== false,
            enableForestWood: config.enableForestWood !== false,
            grassCount: config.grassCount || 150000,
            leavesCount: config.leavesCount || 3500,
            woodCount: config.woodCount || 2600,
            ...config
        };

        this.trees = [];
        this.terrain = null;
        this.skybox = null;
        this.materials = [];
        this.grassField = null;
        this.qualityPanel = null;
        this.fallenLeaves = null; // NOUVEAU
        this.forestWood = null; // NOUVEAU : Système de bois et branches
        this.undergrowth = null; // NOUVEAU : Sous-bois
        this.treeEnhancer = null; // NOUVEAU : Améliorateur de textures
        this.collisionDetection = new CollisionDetection(); // NOUVEAU : Système de collision

        // ── Fleet-scaled platform dimensions (computed once) ──
        const _droneCount = window.FLEET_CONFIG?.numDrones || config.droneCount || 8;
        this.droneCount = _droneCount;

        // Count X500 drones for mixed-fleet platform sizing
        const fleetDrones = window.FLEET_CONFIG?.drones || [];
        const x500Types = ['x500', 'heavy', 'leader', 'cognitive'];
        const x500Count = fleetDrones.filter(d => x500Types.includes(d.type)).length;

        if (x500Count > 0) {
            // Mixed fleet: X500 outer ring needs spacing proportional to visual size
            const x500Scale = 10; // Must match x500-v2.json visual.scale
            const x500Spacing = x500Scale * 0.6;
            const x500RingRadius = Math.max(15, (x500Count * x500Spacing) / (2 * Math.PI));
            this.platformRadius = x500RingRadius + 5;
        } else {
            this.platformRadius = _droneCount <= 8 ? 8.0 : 8.0 * Math.sqrt(_droneCount / 8);
        }
        this.flatZoneRadius = 70.0; // Flat within full exploration zone — hills only at map edges
        this.grassExclusionRadius = this.platformRadius + 2.0;
        this.treeExclusionRadius = this.platformRadius + 6.0;

        log(`🌲 Initialisation Environnement Provençal (${_droneCount} drones, plateforme ${this.platformRadius.toFixed(1)}m)`);
        // Expose a promise so other systems can await full initialization
        this.ready = this.initializeEnvironment();
    }

    async initializeEnvironment() {
        if (this.config.enableTerrain) {
            await this.createAuthenticTerrain();
        }
        
        // Créer la plateforme surélevée au centre
        await this.createCentralPlatform();
        
        if (this.config.enableSkybox) {
            await this.createSkybox();
        }
        
        if (this.config.enableForest) {
            await this.createForest();
        }

        if (this.config.enableGrass) {
            await this.createGLSLGrassField();
        }

        // Créer le panneau de contrôle qualité après le champ d'herbe
        if (this.config.enableGrass && this.grassField) {
            this.createQualityPanel();
        }

        if (this.config.enableFallenLeaves) {
            await this.createFallenLeaves();
        }

        if (this.config.enableForestWood) {
            await this.createForestWood();
        }

        if (this.config.enableUndergrowth) {
            await this.createUndergrowth();
        }

        if (this.config.enableTreeEnhancement) {
            await this.enhanceTreeTextures();
        }

        // Skip lighting setup in lightweight mode (lights already created by main.js)
        if (!this.config.lightweight) {
            this.setupLighting();
        }
        log('✅ Environnement Provençal Authentique initialisé');
    }

    async createAuthenticTerrain() {
        log('🏔️ Création terrain méditerranéen (sol unique, suit la caméra)...');

        // ── UN SEUL sol qui SUIT la caméra ──
        // Grille moyenne résolution ; le RELIEF est calculé en WORLD SPACE dans le
        // vertex shader (displacement GPU, 0 CPU au déplacement) avec EXACTEMENT la
        // même formule que getHeightAt(). Plus de mesh fixe 256² + sol plat séparé :
        // plus de couture, le relief est présent partout sous l'observation, et le
        // boot ne recalcule plus 66k sommets (colline + vertex-colors) côté CPU.
        this._terrainSize = 800;   // couverture monde (m)
        this._terrainSeg = 160;    // 5 m / quad
        this._terrainStep = this._terrainSize / this._terrainSeg;
        this._groundTile = 6;      // m/tuile (config ~36 FPS ; plus fin => filtrage coûteux en rasant)

        const geometry = new THREE.PlaneGeometry(this._terrainSize, this._terrainSize, this._terrainSeg, this._terrainSeg);

        // Texture d'herbe photo tuilée (~6 m/tuile) — sol VERT réaliste PARTOUT.
        // Fallback couleur verte si assets/terrain/grass.jpg manque (jamais de pâle).
        const _texLoader = new THREE.TextureLoader();
        const material = new THREE.MeshStandardMaterial({
            color: 0x6b8c5a,   // fallback vert si la texture est absente
            roughness: 1.0,
            metalness: 0.0,
        });
        _texLoader.load(
            'assets/terrain/grass.jpg',
            (tex) => {
                tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
                tex.colorSpace = THREE.SRGBColorSpace;
                tex.anisotropy = 4;
                tex.repeat.set(this._terrainSize / this._groundTile, this._terrainSize / this._groundTile);
                material.map = tex;
                material.color.set(0x88a458); // vert herbe franc => le sol lointain lit comme un champ (fini l'olive délavé)
                material.needsUpdate = true;
            },
            undefined,
            () => { /* texture absente → le fallback vert reste en place */ }
        );

        // Terrain PLAT texturé (pas de displacement GPU custom = zéro risque shader).
        // Aire de vol déjà plate (flatZoneRadius 70m) ; le relief lointain est cosmétique.
        this.terrain = new THREE.Mesh(geometry, material);
        this.terrain.rotation.x = -Math.PI / 2;
        this.terrain.receiveShadow = true;
        this.terrain.name = 'Terrain';
        this.scene.add(this.terrain);
        this.materials.push(material);

        // Terrain PLAT => hauteur 0 partout. Herbe et arbres se posent au niveau du sol
        // (fini la crête d'herbe flottant sur des collines fantômes).
        this.getHeightAt = (_x, _z) => 0;
        
        // Fonction spécialisée pour l'herbe — exclut la zone du héliport central
        this.getGrassHeightAt = (x, z) => {
            // No grass on the heliport platform
            const distFromCenter = Math.sqrt(x * x + z * z);
            if (distFromCenter < this.grassExclusionRadius) {
                return -999; // Signal to grass system: skip this position
            }
            return this.getHeightAt(x, z);
        };

        // Position initiale du sol sous l'origine ; la boucle de rendu le recentre
        // ensuite sur la caméra à chaque frame (updateTerrainFollow).
        this.updateTerrainFollow(new THREE.Vector3());

        log(`✅ Terrain unique suit-caméra (${this._terrainSize}m, ${this._terrainSeg}² quads, relief GPU world-space)`);
    }

    async createSkybox() {
        log('🌅 Création skybox méditerranéen...');
        
        const skyboxGeometry = new THREE.SphereGeometry(500, 64, 64);
        const skyboxMaterial = new THREE.ShaderMaterial({
            vertexShader: SkyboxShaders.vertex,
            fragmentShader: SkyboxShaders.fragment,
            uniforms: {
                uSunAzimuth: { value: SkyboxDefaults.sunAzimuth },
                uSunElevation: { value: SkyboxDefaults.sunElevation },
                uSunColor: { value: new THREE.Vector3(...SkyboxDefaults.sunColor) },
                uSkyColorLow: { value: new THREE.Vector3(...SkyboxDefaults.skyColorLow) },
                uSkyColorHigh: { value: new THREE.Vector3(...SkyboxDefaults.skyColorHigh) },
                uSunSize: { value: SkyboxDefaults.sunSize },
                uTime: { value: 0 },
                uHazeIntensity: { value: SkyboxDefaults.hazeIntensity },
                uHazeColor: { value: new THREE.Vector3(...SkyboxDefaults.hazeColor) }
            },
            side: THREE.BackSide
        });
        
        this.skybox = new THREE.Mesh(skyboxGeometry, skyboxMaterial);
        this.skybox.name = 'Skybox';
        this.scene.add(this.skybox);
        this.materials.push(skyboxMaterial);
        
        // Animation du skybox
        this.updateSkybox = (time) => {
            if (this.skybox && this.skybox.material.uniforms) {
                this.skybox.material.uniforms.uTime.value = time * 0.1;
            }
        };
        
        log('✅ Skybox méditerranéen créé');
    }

    async createForest() {
        log('🌳 Génération forêt provençale authentique avec EZ-Tree...');
        
        const { x, y } = this.config.terrainSize;
        const margin = 10; // Keep trees 10m inside terrain edges
        
        // Configuration espèces provençales avec presets EZ-Tree adaptés
        const forestSpecies = [
            {
                name: 'Chêne Vert',
                preset: 'Oak Medium',
                weight: 0.4,
                modifications: {
                    leafColor: 0x2F4F2F, // Vert sombre persistant
                    barkColor: 0x5D4037,
                    maxHeight: 12,
                    minHeight: 8
                }
            },
            {
                name: 'Pin Parasol',
                preset: 'Pine Large',
                weight: 0.25,
                modifications: {
                    leafColor: 0x228B22, // Vert pin
                    barkColor: 0x8B4513,
                    maxHeight: 18,
                    minHeight: 12
                }
            },
            {
                name: 'Cyprès',
                preset: 'Pine Medium',
                weight: 0.15,
                modifications: {
                    leafColor: 0x1B5E20, // Vert très sombre
                    barkColor: 0x6D4C41,
                    maxHeight: 20,
                    minHeight: 15,
                    columnar: true // Forme colonnaire
                }
            },
            {
                name: 'Olivier',
                preset: 'Oak Small',
                weight: 0.2,
                modifications: {
                    leafColor: 0x9CAF88, // Vert olive argenté
                    barkColor: 0x8B7D6B,
                    maxHeight: 8,
                    minHeight: 4,
                    gnarled: true // Forme tortueuse
                }
            }
        ];

        // ── POOL DE TEMPLATES ── un arbre par espèce, généré UNE fois (source de géométrie).
        this._treeTemplates = [];
        for (const species of forestSpecies) {
            const tmpl = await this.createAuthenticTree(species, 0, 0);
            if (tmpl) {
                this.trees.pop(); // template hors-scène, pas un arbre placé
                this._treeTemplates.push(tmpl);
            }
        }

        // ── FORÊT STATIQUE (clones placés UNE fois) ── rendu CORRECT (le shader de
        // feuilles EZ-tree ne supporte pas l'InstancedMesh), placée une seule fois donc
        // AUCUN coût par frame. Rayon large => arbres visibles au loin.
        const _forestRadius = this.config.treeForestRadius || 300;
        const _treeCount = this.config.treeCount || 130;
        const _forest = new THREE.Group();
        _forest.name = 'Forest';
        let _s = 0x9e3779b1 >>> 0;
        const _rnd = () => { _s = (Math.imul(_s, 1103515245) + 12345) >>> 0; return _s / 4294967296; };
        if (this._treeTemplates.length) {
            for (let i = 0, tries = 0; i < _treeCount && tries < _treeCount * 8; tries++) {
                const ang = _rnd() * Math.PI * 2;
                const r = this.treeExclusionRadius + Math.pow(_rnd(), 0.6) * (_forestRadius - this.treeExclusionRadius);
                const tmpl = this._treeTemplates[Math.floor(_rnd() * this._treeTemplates.length)];
                const tree = tmpl.clone(true); // clone (géométrie/matériaux partagés) => rendu correct
                tree.traverse(o => { o.castShadow = false; }); // pas d'ombre (perf)
                tree.position.set(Math.cos(ang) * r, 0, Math.sin(ang) * r);
                tree.rotation.y = _rnd() * Math.PI * 2;
                tree.scale.setScalar(0.8 + _rnd() * 0.6);
                _forest.add(tree);
                i++;
            }
        }
        this.scene.add(_forest);
        log(`✅ Forêt statique: ${_forest.children.length} arbres (rayon ${_forestRadius}m)`);
    }

    // Forêt statique => aucun travail par frame (no-op conservé pour la boucle de rendu).
    updateTreeChunks(_cameraPos) { /* forêt statique : rien à streamer */ }

    selectSpeciesByWeight(species) {
        const random = Math.random();
        let cumulative = 0;
        
        for (const s of species) {
            cumulative += s.weight;
            if (random <= cumulative) {
                return s;
            }
        }
        
        return species[0]; // Fallback
    }

    async createAuthenticTree(species, x, z) {
        try {
            // Créer l'arbre avec le preset EZ-Tree
            const tree = new Tree();
            tree.loadPreset(species.preset);
            // loadPreset already calls generate() internally.

            // ── RÉDUCTION DE COMPLEXITÉ GÉOMÉTRIQUE (le vrai levier FPS) ──
            // Par défaut EZ-tree fait levels:3 + children{7,7,5} => ~300 branches et
            // 10-20k sommets PAR ARBRE. Avec 130 arbres, la lisière plein cadre = des
            // millions de sommets => 5 FPS. En passant à levels:2 on supprime le niveau
            // de brindilles (quasi invisible) : ~5x moins de géométrie, même silhouette.
            // Géométrie d'arbre laissée à sa qualité NATIVE : le profilage montre que
            // l'environnement coûte <1 ms/frame — l'alléger n'apportait aucun FPS et
            // dégradait le rendu. (⚠️ ne jamais toucher `levels` : les FEUILLES poussent
            // sur le dernier niveau de branches, le réduire donne des arbres nus.)


            // Appliquer modifications spécifiques à l'espèce APRÈS generate()
            // tree.leavesMesh.material is the actual leaf material (not tree.leafMaterial)
            if (species.modifications) {
                this.applySpeciesModifications(tree, species.modifications);
            }
            
            // Positionner sur le terrain
            const y = this.getHeightAt ? this.getHeightAt(x, z) : 0;
            tree.position.set(x, y, z);
            
            // Rotation aléatoire
            tree.rotation.y = Math.random() * Math.PI * 2;
            
            tree.castShadow = true;
            tree.receiveShadow = true;
            
            // Nom et référence
            tree.name = `${species.name}_${this.trees.length}`;
            this.trees.push(tree);
            
            return tree;
            
        } catch (error) {
            warn(`⚠️ Erreur génération arbre ${species.name}:`, error);
            return null;
        }
    }

    applySpeciesModifications(tree, modifications) {
        // Apply to the actual leaf mesh material (tree.leavesMesh.material, NOT tree.leafMaterial)
        if (modifications.leafColor && tree.leavesMesh && tree.leavesMesh.material) {
            tree.leavesMesh.material.color.setHex(modifications.leafColor);
        }
        
        if (modifications.barkColor && tree.branchesMesh && tree.branchesMesh.material) {
            tree.branchesMesh.material.color.setHex(modifications.barkColor);
        }
        
        // Ajustements morphologiques
        if (modifications.columnar) {
            // Forme colonnaire pour cyprès
            tree.options.branchAngle *= 0.3;
            tree.options.trunkTaper = 0.95;
        }
        
        if (modifications.gnarled) {
            // Forme tortueuse pour oliviers
            tree.options.gnarliness = 0.8;
            tree.options.branchNoise = 0.6;
        }
    }

    setupLighting() {
        log('☀️ Configuration éclairage méditerranéen...');
        
        // Lumière ambiante dorée (complète l'hemisphere de main.js — assure l'éclairage des sous-bois)
        const ambientLight = new THREE.AmbientLight(0xFFF8DC, 0.65);
        this.scene.add(ambientLight);
        
        // Soleil méditerranéen (intensité réaliste, pas de surexposition car DAE lights supprimées)
        const sunLight = new THREE.DirectionalLight(0xFFE4B5, 1.0);
        sunLight.position.set(100, 80, 50);
        sunLight.castShadow = true;
        
        sunLight.shadow.mapSize.width = this.config.shadowMapSize || 4096;
        sunLight.shadow.mapSize.height = this.config.shadowMapSize || 4096;
        sunLight.shadow.camera.near = 0.5;
        sunLight.shadow.camera.far = 500;
        sunLight.shadow.camera.left = -150;
        sunLight.shadow.camera.right = 150;
        sunLight.shadow.camera.top = 150;
        sunLight.shadow.camera.bottom = -150;
        sunLight.shadow.bias = -0.0001;
        
        this.scene.add(sunLight);
        
        // Lumière d'appoint chaude (complète l'hemisphere de main.js)
        const fillLight = new THREE.HemisphereLight(0x87CEEB, 0xD2B48C, 0.7);
        this.scene.add(fillLight);
        
        // Brume désactivée (cosmétique, crée des halos)
        // this.scene.fog = new THREE.FogExp2(0xE6E6FA, 0.0008);
        
        log('✅ Éclairage méditerranéen configuré');
    }

    async createGLSLGrassField() {
        log('🌿 Creation du système d\'herbe dynamique avec zone réduite...');
        
        try {
            // Configuration de performance optimisée avec zone réduite
            this.grassField = new GLSLGrassFieldDynamic({
                fieldSize: 160,                    // Cover full exploration zone
                grassCount: 25000,                 // 25k instances max
                chunkSize: 20,                     // 20 unités
                renderDistance: 90,                // Cover exploration zone + margin
                // grassPerChunk géré par shader-quality-manager (réduit automatiquement)
                grassScale: 0.6,                   // Taille optimisée
                tipColor: '#4a7c3a',               // Vert naturel pour les pointes
                baseColor: '#2d4a1f',              // Vert foncé pour la base
                fogColor: '#6b8c5a'                // Vert brumeux
            });
            
            // Initialiser le système avec la scène et la fonction de terrain
            await this.grassField.createGrassSystem(
                this.scene,
                this.getGrassHeightAt.bind(this), // Utiliser une fonction spécialisée
                null // La caméra sera fournie dans update()
            );
            
            log('✅ Champ d\'herbe dynamique créé avec zone réduite pour performance');
            
            // Lier le panneau qualité au champ d'herbe si déjà créé
            if (this.qualityPanel) {
                this.qualityPanel.bindToGrassField(this.grassField);
                this.qualityPanel.forceUpdate();
                log('🔗 Panneau qualité connecté au champ d\'herbe');
            }
            
        } catch (error) {
            warn('⚠️ Impossible de créer l\'herbe dynamique:', error);
        }
    }

    /**
     * Crée le panneau de contrôle de qualité
     */
    createQualityPanel() {
        log('🎮 Création du panneau de contrôle qualité...');
        
        try {
            // Nettoyer le panneau existant s'il y en a un
            if (this.qualityPanel) {
                log('🗑️ Nettoyage du panneau existant...');
                this.qualityPanel.dispose();
                this.qualityPanel = null;
            }
            
            // Créer le panneau dans le container principal
            const container = document.body;
            this.qualityPanel = new QualityControlPanel(container);
            
            // Lier au champ d'herbe si disponible
            if (this.grassField) {
                this.qualityPanel.bindToGrassField(this.grassField);
                log('🔗 Panneau lié au champ d\'herbe');
            } else {
                log('⚠️ Champ d\'herbe non disponible lors de la création du panneau');
            }
            
            // Force la mise à jour immédiate
            this.qualityPanel.forceUpdate();
            
            // Callbacks pour les changements
            this.qualityPanel.setCallbacks({
                onQualityChange: (quality) => {
                    log(`🎨 Qualité changée vers: ${quality}`);
                    if (this.grassField) {
                        this.grassField.setQuality(quality);
                    }
                },
                onAutoToggle: (enabled) => {
                    log(`🤖 Qualité automatique: ${enabled ? 'activée' : 'désactivée'}`);
                    if (this.grassField) {
                        this.grassField.setAutoQuality(enabled);
                    }
                }
            });
            
            log('✅ Panneau de contrôle qualité créé');
            
        } catch (error) {
            error('❌ Erreur lors de la création du panneau qualité:', error);
        }
    }

    async createFallenLeaves() {
        log('🍂 Creation des feuilles tombees au sol...');
        
        try {
            this.fallenLeaves = new FallenLeavesSystem(this.scene, this.getHeightAt, {
                count: this.config.leavesCount,     // Nombre de feuilles
                fieldSize: 180,                     // Zone de dispersion
                leafScale: 0.4                      // Taille des feuilles
            });
            
            log('✅ Feuilles tombees creees avec', this.config.leavesCount, 'feuilles');
            
        } catch (error) {
            warn('⚠️ Impossible de creer feuilles tombees:', error);
        }
    }

    async createForestWood() {
        log('🪵 Creation du systeme de bois et branches...');
        
        try {
            this.forestWood = new ForestWoodSystem(this.scene, this.terrain, {
                branchCount: Math.floor(this.config.woodCount * 0.6),  // 60% branches
                logCount: Math.floor(this.config.woodCount * 0.15),    // 15% troncs
                stickCount: Math.floor(this.config.woodCount * 0.25),  // 25% brindilles
                terrainSize: 160
            });
            
            log('✅ Systeme de bois cree avec', this.config.woodCount, 'elements (branches, troncs, brindilles)');
            
        } catch (error) {
            warn('⚠️ Impossible de creer le systeme de bois:', error);
        }
    }

    async createUndergrowth() {
        log('🌿 Creation du sous-bois (fougeres, buissons, petites plantes)...');
        
        try {
            this.undergrowth = new UndergrowthSystem(this.scene, this.getHeightAt, {
                fernCount: Math.floor(this.config.undergrowthCount * 0.4),      // 40% fougères
                bushCount: Math.floor(this.config.undergrowthCount * 0.3),      // 30% buissons
                smallPlantCount: Math.floor(this.config.undergrowthCount * 0.3), // 30% petites plantes
                fieldSize: 160
            });
            
            log('✅ Sous-bois cree avec', this.config.undergrowthCount, 'elements (fougeres, buissons, plantes)');
            
        } catch (error) {
            warn('⚠️ Impossible de creer le sous-bois:', error);
        }
    }

    async enhanceTreeTextures() {
        log('🎨 Amelioration des textures d\'arbres...');
        
        try {
            this.treeEnhancer = new TreeTextureEnhancer();
            
            // Améliorer tous les arbres existants
            if (this.trees && this.trees.length > 0) {
                this.treeEnhancer.enhanceTreeGroup(this.trees, {
                    trunkVariation: 0.4,           // Variation des troncs
                    uniformSeason: 'summer'        // Saison d'été par défaut
                });
                
                log('✅ Textures ameliorees pour', this.trees.length, 'arbres');
            }
            
        } catch (error) {
            warn('⚠️ Impossible d\'ameliorer les textures d\'arbres:', error);
        }
    }

    async createCentralPlatform() {
        // ── Central heliport platform (NO arena walls — open space) ──
        const droneCount = this.droneCount;
        log(`🏗️ Création plateforme héliport (${droneCount} drones, espace ouvert)...`);

        const PLATFORM_HEIGHT = 0.15;   // Low platform flush with ground
        const PLATFORM_RADIUS = this.platformRadius;
        // Drone marker radius = 70% of platform radius
        const DRONE_MARKER_RADIUS = PLATFORM_RADIUS * 0.6875;

        const arenaGroup = new THREE.Group();
        arenaGroup.name = 'GazeboArena';

        // ── Central platform (heliport) ──
        const platformGeo = new THREE.CylinderGeometry(PLATFORM_RADIUS, PLATFORM_RADIUS, PLATFORM_HEIGHT, 48);
        const platformMat = new THREE.MeshStandardMaterial({
            color: 0x555555,
            roughness: 0.9,
            metalness: 0.1,
        });
        const platform = new THREE.Mesh(platformGeo, platformMat);
        platform.position.set(0, PLATFORM_HEIGHT / 2, 0);
        platform.receiveShadow = true;
        platform.name = 'CentralPlatform';
        arenaGroup.add(platform);

        // Heliport marking — outer ring (scales with platform)
        const outerRingInner = PLATFORM_RADIUS * 0.8125;
        const outerRingOuter = PLATFORM_RADIUS * 0.875;
        const markingGeo = new THREE.RingGeometry(outerRingInner, outerRingOuter, 48);
        const markingMat = new THREE.MeshBasicMaterial({
            color: 0xffcc00,
            side: THREE.DoubleSide,
        });
        const marking = new THREE.Mesh(markingGeo, markingMat);
        marking.rotation.x = -Math.PI / 2;
        marking.position.set(0, PLATFORM_HEIGHT + 0.01, 0);
        marking.name = 'HeliportMarking';
        arenaGroup.add(marking);

        // Inner "H" marking
        const hMarkGeo = new THREE.RingGeometry(PLATFORM_RADIUS * 0.3, PLATFORM_RADIUS * 0.375, 32);
        const hMark = new THREE.Mesh(hMarkGeo, markingMat.clone());
        hMark.rotation.x = -Math.PI / 2;
        hMark.position.set(0, PLATFORM_HEIGHT + 0.01, 0);
        arenaGroup.add(hMark);

        // NO WALLS — drones fly in open space, forest is the boundary

        // Grille de sol de debug RETIRÉE — faisait "fake" (quadrillage technique).
        // Le sol naturel + les anneaux du héliport suffisent au repère visuel.

        log(`✅ Plateforme: rayon=${PLATFORM_RADIUS.toFixed(1)}m, ${droneCount} spots`);

        this.scene.add(arenaGroup);

        this.centralPlatform = platform;

        // Initialize collision detection
        this.collisionDetection.initialize(this.scene);

        log(`✅ Plateforme héliport créée (${droneCount} drones, rayon ${PLATFORM_RADIUS.toFixed(1)}m)`);
    }

    /**
     * Distribute n drones across concentric rings on a platform
     * Returns array of {x, z} positions
     */
    _computeRingPositions(n, platformRadius) {
        const positions = [];
        if (n <= 8) {
            const r = platformRadius * 0.7;
            for (let i = 0; i < n; i++) {
                const angle = (i / n) * 2 * Math.PI;
                positions.push({ x: Math.cos(angle) * r, z: Math.sin(angle) * r });
            }
            return positions;
        }
        const numRings = n <= 16 ? 2 : n <= 32 ? 3 : Math.min(5, Math.ceil(n / 10));
        const minR = platformRadius * 0.22;
        const maxR = platformRadius * 0.75;
        const rings = [];
        let totalCirc = 0;
        for (let r = 0; r < numRings; r++) {
            const radius = numRings === 1 ? maxR : minR + (maxR - minR) * r / (numRings - 1);
            const circ = 2 * Math.PI * radius;
            rings.push({ radius, circ });
            totalCirc += circ;
        }
        let assigned = 0;
        for (let r = 0; r < numRings; r++) {
            const count = r === numRings - 1
                ? n - assigned
                : Math.max(3, Math.round(n * rings[r].circ / totalCirc));
            const angleOffset = r * 0.3;
            for (let i = 0; i < count; i++) {
                const angle = (i / count) * 2 * Math.PI + angleOffset;
                positions.push({
                    x: Math.cos(angle) * rings[r].radius,
                    z: Math.sin(angle) * rings[r].radius
                });
            }
            assigned += count;
        }
        return positions;
    }

    createPlatformPillars(platformGroup, radius, height) {
        // Piliers de support réalistes qui ne dépassent PAS de la plateforme
        const pillarGeometry = new THREE.CylinderGeometry(0.8, 1.2, height, 12);
        const pillarMaterial = new THREE.MeshLambertMaterial({
            color: 0x555555, // Béton gris standard
            roughness: 0.9
        });
        
        // 4 piliers SOUS la plateforme, bien répartis mais DANS le rayon
        const pillarCount = 4;
        const pillarRadius = radius * 0.35; // ENCORE PLUS près du centre pour éviter tout dépassement
        
        for (let i = 0; i < pillarCount; i++) {
            const angle = (i / pillarCount) * Math.PI * 2;
            const x = Math.cos(angle) * pillarRadius;
            const z = Math.sin(angle) * pillarRadius;
            
            // SIMPLIFIÉ: Tous les piliers partent du niveau 0 (sol) jusqu'à la hauteur de la plateforme
            const pillar = new THREE.Mesh(pillarGeometry, pillarMaterial);
            // Position Y = height/2 pour centrer le pilier (de 0 à height)
            pillar.position.set(x, height / 2, z);
            pillar.castShadow = true;
            pillar.receiveShadow = true;
            pillar.name = `PlatformPillar_${i}`;
            
            platformGroup.add(pillar);
        }
    }

    createPlatformSurface(platformGroup, radius, height) {
        // Surface principale - plateforme d'hélipont RÉALISTE sans couleurs criarde
        const platformGeometry = new THREE.CylinderGeometry(radius, radius, 0.4, 32);
        const platformMaterial = new THREE.MeshStandardMaterial({
            color: 0x606060, // Gris béton réaliste
            roughness: 0.8,
            metalness: 0.1,
            envMapIntensity: 0.3
        });
        
        const platform = new THREE.Mesh(platformGeometry, platformMaterial);
        platform.position.set(0, height + 0.2, 0); // Au-dessus des piliers
        platform.receiveShadow = true;
        platform.castShadow = true;
        platform.name = 'PlatformSurface';
        
        // Bordure de sécurité simple et réaliste
        const borderGeometry = new THREE.CylinderGeometry(radius + 0.2, radius + 0.1, 0.1, 32);
        const borderMaterial = new THREE.MeshStandardMaterial({
            color: 0x404040, // Gris plus foncé pour la bordure
            metalness: 0.2,
            roughness: 0.7
        });
        
        const border = new THREE.Mesh(borderGeometry, borderMaterial);
        border.position.set(0, height + 0.05, 0);
        border.receiveShadow = true;
        border.castShadow = true;
        border.name = 'PlatformBorder';
        
        // Marquage central discret (pas de jaune criard)
        const centerMarkGeometry = new THREE.CylinderGeometry(2, 2, 0.01, 16);
        const centerMarkMaterial = new THREE.MeshBasicMaterial({
            color: 0x808080, // Gris neutre
            transparent: true,
            opacity: 0.8
        });
        
        const centerMark = new THREE.Mesh(centerMarkGeometry, centerMarkMaterial);
        centerMark.position.set(0, height + 0.41, 0);
        centerMark.name = 'CenterMark';
        
        platformGroup.add(border);
        platformGroup.add(platform);
        platformGroup.add(centerMark);
    }

    createDroneMarkers(platformGroup, radius, height) {
        // Marqueurs de positionnement des drones - SIMPLES et réalistes
        const markerRadius = 0.8;
        const markerGeometry = new THREE.CylinderGeometry(markerRadius, markerRadius, 0.02, 16);
        const markerMaterial = new THREE.MeshBasicMaterial({
            color: 0xFF2222, // Rouge simple mais visible
            transparent: true,
            opacity: 0.9
        });
        
        // Créer 6 marqueurs simples en hexagone
        for (let i = 0; i < 6; i++) {
            const angle = (i / 6) * Math.PI * 2;
            const x = Math.cos(angle) * 8; // Rayon pour les drones
            const z = Math.sin(angle) * 8;
            
            const marker = new THREE.Mesh(markerGeometry, markerMaterial);
            marker.position.set(x, height + 0.41, z); // Juste au-dessus de la surface
            marker.name = `DroneMarker_${i}`;
            
            platformGroup.add(marker);
        }
    }

    // Sol unique : suit la caméra en XZ (snappé à la grille pour éviter le "swimming"
    // des sommets sur le relief), texture verrouillée au monde via offset. Le relief
    // est calculé côté GPU en world-space → le plan glisse, les collines restent fixes.
    updateTerrainFollow(cameraPos) {
        if (!this.terrain || !cameraPos) return;
        const step = this._terrainStep || 5;
        const sx = Math.round(cameraPos.x / step) * step;
        const sz = Math.round(cameraPos.z / step) * step;
        this.terrain.position.x = sx;
        this.terrain.position.z = sz;
        const tex = this.terrain.material.map;
        if (tex) {
            const t = this._groundTile;
            tex.offset.x = sx / t;
            tex.offset.y = -sz / t;
        }
    }

    update(time, camera = null) {
        // Mettre à jour les éléments animés
        if (this.updateSkybox) {
            this.updateSkybox(time);
        }
        if (camera) this.updateTerrainFollow(camera.position);

        // Mise à jour de l'herbe dynamique avec position du joueur
        if (this.grassField && camera) {
            this.grassField.update(time, camera.position);
        }

        // Animation subtile des feuilles tombées
        if (this.fallenLeaves) {
            this.fallenLeaves.animate(time);
        }

        // Animation subtile du bois (si nécessaire)
        if (this.forestWood) {
            this.forestWood.update(time, camera);
        }

        // Animation du sous-bois avec le vent
        if (this.undergrowth) {
            this.undergrowth.update(time);
        }
        
        // Animation vent sur les arbres (si supportée par EZ-Tree)
        this.trees.forEach(tree => {
            if (tree.update) {
                tree.update(time);
            }
        });
        
        // Mise à jour du panneau de qualité (comptage FPS)
        if (this.qualityPanel) {
            this.qualityPanel.updatePerformanceDisplay();
        }
    }

    dispose() {
        // Nettoyage ressources
        this.materials.forEach(material => {
            if (material.dispose) material.dispose();
        });
        
        this.trees.forEach(tree => {
            if (tree.dispose) tree.dispose();
        });

        // Nettoyage de l'herbe GLSL
        if (this.grassField) {
            this.grassField.dispose();
            this.grassField = null;
        }

        // Nettoyage du panneau de qualité
        if (this.qualityPanel) {
            this.qualityPanel.dispose();
            this.qualityPanel = null;
        }

        // Nettoyage des feuilles tombées
        if (this.fallenLeaves) {
            this.fallenLeaves.dispose();
            this.fallenLeaves = null;
        }

        // Nettoyage du système de bois
        if (this.forestWood) {
            this.forestWood.dispose();
            this.forestWood = null;
        }

        // Nettoyage du sous-bois
        if (this.undergrowth) {
            this.undergrowth.dispose();
            this.undergrowth = null;
        }

        // Nettoyage de l'améliorateur de textures
        if (this.treeEnhancer) {
            this.treeEnhancer.dispose();
            this.treeEnhancer = null;
        }

        // Nettoyage de la plateforme centrale
        if (this.centralPlatform) {
            this.centralPlatform = null;
        }
        
        // Nettoyage du panneau de qualité
        if (this.qualityPanel) {
            this.qualityPanel.dispose();
            this.qualityPanel = null;
        }
        
        // Nettoyage du système de collision
        if (this.collisionDetection) {
            this.collisionDetection.dispose();
            this.collisionDetection = null;
        }
        
        this.materials = [];
        this.trees = [];
        
        log('🧹 Environnement provençal nettoyé');
    }

    // ===== MÉTHODES PUBLIQUES POUR COLLISION =====
    
    /**
     * Returns all tree bounding boxes for external collision systems
     * Each entry: { center: {x, z}, radius: number, box: THREE.Box3 }
     */
    getTreeBoundingBoxes() {
        return this.trees
            .filter(t => t.userData && t.userData.collisionBox)
            .map(t => ({
                center: t.userData.collisionCenter,
                radius: t.userData.collisionRadius,
                box: t.userData.collisionBox,
                name: t.name,
            }));
    }

    /**
     * Check if a position collides with any tree trunk
     * Returns { collides: boolean, nearestTree: string|null, distance: number }
     */
    checkTreeCollision(x, z, droneRadius = 0.3) {
        let nearestDist = Infinity;
        let nearestTree = null;
        
        for (const tree of this.trees) {
            if (!tree.userData || !tree.userData.collisionCenter) continue;
            const tc = tree.userData.collisionCenter;
            const tr = tree.userData.collisionRadius || 1.5;
            const dx = x - tc.x;
            const dz = z - tc.z;
            const dist = Math.sqrt(dx * dx + dz * dz);
            const minDist = tr + droneRadius;
            
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestTree = tree.name;
            }
            
            if (dist < minDist) {
                return { collides: true, nearestTree: tree.name, distance: dist };
            }
        }
        
        return { collides: false, nearestTree, distance: nearestDist };
    }

    /**
     * Vérifie et corrige la position d'un drone pour éviter les collisions
     */
    checkDroneCollision(droneId, position) {
        if (!this.collisionDetection) return { hasCollision: false, correctedPosition: position };
        return this.collisionDetection.updateDronePosition(droneId, position);
    }

    /**
     * Vérifie si une position est valide pour un drone
     */
    isPositionValid(droneId, position) {
        if (!this.collisionDetection) return true;
        return this.collisionDetection.isPositionValid(droneId, position);
    }

    /**
     * Trouve une position valide proche de la position désirée
     */
    findValidPosition(droneId, desiredPosition) {
        if (!this.collisionDetection) return desiredPosition;
        return this.collisionDetection.findValidPosition(droneId, desiredPosition);
    }

    /**
     * Supprime un drone du système de collision
     */
    removeDroneFromCollision(droneId) {
        if (this.collisionDetection) {
            this.collisionDetection.removeDrone(droneId);
        }
    }

    /**
     * Active/désactive le mode debug des collisions
     */
    setCollisionDebugMode(enabled) {
        if (this.collisionDetection) {
            this.collisionDetection.setDebugMode(enabled);
        }
    }

    /**
     * Obtient les statistiques du système de collision
     */
    getCollisionStats() {
        return this.collisionDetection ? this.collisionDetection.getStats() : null;
    }
}
