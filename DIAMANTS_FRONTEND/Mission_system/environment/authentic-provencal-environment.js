/**
 * DIAMANTS - Environn            enableFallenLeaves: config.enableFallenLeaves !== false,
            enableForestWood: config.enableForestWood !== false, // NOUVEAU : Système de bois
            enableUndergrowth: config.enableUndergrowth !== false, // NOUVEAU : Sous-bois
            enableTreeEnhancement: config.enableTreeEnhancement !== false, // NOUVEAU : Amélioration textures arbres
            grassCount: config.grassCount || 120000, // AUGMENTÉ : Plus d'herbe comme demandé
            leavesCount: config.leavesCount || 4500, // AUGMENTÉ : Plus de feuilles colorées
            woodCount: config.woodCount || 2600, // NOUVEAU : 2.6k éléments de bois (branches, troncs, brindilles)
            undergrowthCount: config.undergrowthCount || 2600, // NOUVEAU : Sous-bois (fougères, buissons, petites plantes) Provençal Authentique avec EZ-Tree
 * =============================================================
 * Utilise la vraie API EZ-Tree avec presets JSON et textures haute qualité
 */

import * as THREE from 'three';
// Mode silencieux pour les logs
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Import depuis les sources EZ-Tree
import { Tree } from '../third-party/ez-tree/src/lib/tree.js';
import { TreePreset, loadPreset } from '../third-party/ez-tree/src/lib/presets/index.js';
import { ProvencalSkyboxShaders, ProvencalSkyboxDefaults } from './provencal-skybox.js';
import { GLSLGrassFieldDynamic } from './glsl-grass-field-dynamic.js';
import QualityControlPanel from './quality-control-panel.js';
import { QUALITY_LEVELS } from '../shaders/shader-quality-manager.js';
import { FallenLeavesSystem } from './fallen-leaves.js';
import { ForestWoodSystem } from './forest-wood.js';
import { UndergrowthSystem } from './undergrowth.js';
import { TreeTextureEnhancer } from './tree-texture-enhancer.js';
import { CollisionDetection } from '../physics/collision-detection.js';

export class AuthenticProvencalEnvironment {
    constructor(scene, config = {}) {
        this.scene = scene;
        this.config = {
            terrainSize: config.terrainSize || { x: 200, y: 200 },
            forestDensity: config.forestDensity ?? 0.9, // Réduction densité forêt
            maxTrees: config.maxTrees || 200, // Moins d'arbres pour performance
            minTreeSpacing: config.minTreeSpacing || 30.0, // Plus d'espacement
            enableSkybox: config.enableSkybox !== false,
            enableTerrain: config.enableTerrain !== false,
            enableForest: config.enableForest !== false,
            enableGrass: config.enableGrass !== false,
            enableFallenLeaves: config.enableFallenLeaves !== false,
            enableForestWood: config.enableForestWood !== false, // NOUVEAU : Système de bois
            grassCount: config.grassCount || 150000, // OPTIMISÉ : 150k pour équilibre performance/densité
            leavesCount: config.leavesCount || 3500, // AUGMENTÉ de 2k à 3.5k pour plus de feuilles
            woodCount: config.woodCount || 2600, // NOUVEAU : 2.6k éléments de bois (branches, troncs, brindilles)
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

        log('🌲 Initialisation Environnement Provençal Authentique avec EZ-Tree');
        this.initializeEnvironment();
    }

    async initializeEnvironment() {
        if (this.config.enableTerrain) {
            await this.createAuthenticTerrain();
        }
        
        // Créer la plateforme surélevée au centre
        await this.createCentralPlatform();
        
        if (this.config.enableSkybox) {
            await this.createProvencalSkybox();
        }
        
        if (this.config.enableForest) {
            await this.createAuthenticProvencalForest();
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

        this.setupLighting();
        log('✅ Environnement Provençal Authentique initialisé');
    }

    async createAuthenticTerrain() {
        log('🏔️ Création terrain méditerranéen...');
        
        const { x, y } = this.config.terrainSize;
        const geometry = new THREE.PlaneGeometry(x, y, 256, 256);
        
        // Relief provençal réaliste avec collines et vallées
        const vertices = geometry.attributes.position.array;
        for (let i = 0; i < vertices.length; i += 3) {
            const xPos = vertices[i];
            const yPos = vertices[i + 1];
            
            // Relief multi-fréquences pour collines provençales
            const elevation = 
                Math.sin(xPos * 0.02) * Math.cos(yPos * 0.015) * 8.0 +
                Math.sin(xPos * 0.05) * Math.cos(yPos * 0.04) * 4.0 +
                Math.sin(xPos * 0.1) * Math.cos(yPos * 0.08) * 2.0 +
                (Math.random() - 0.5) * 0.5; // Rugosité naturelle
            
            vertices[i + 2] = elevation;
        }
        
        geometry.attributes.position.needsUpdate = true;
        geometry.computeVertexNormals();
        
        // Matériau terrain méditerranéen amélioré avec nuances
        const material = new THREE.ShaderMaterial({
            vertexShader: `
                varying vec3 vPosition;
                varying vec3 vNormal;
                varying vec2 vUv;
                varying float vElevation;
                
                void main() {
                    vPosition = position;
                    vNormal = normal;
                    vUv = uv;
                    vElevation = position.y;
                    
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
                }
            `,
            fragmentShader: `
                uniform float uTime;
                varying vec3 vPosition;
                varying vec3 vNormal;
                varying vec2 vUv;
                varying float vElevation;
                
                // Sol de forêt ÉQUILIBRÉ - Mélange vert-brun naturel
                vec3 terreBrune = vec3(0.35, 0.25, 0.15);       // Terre brune modérée
                vec3 argileFoncee = vec3(0.28, 0.2, 0.12);      // Argile brun-vert
                vec3 terreHumide = vec3(0.25, 0.2, 0.1);        // Terre humide sombre
                vec3 mousseVerte = vec3(0.15, 0.35, 0.2);       // Mousse VERTE claire
                vec3 humusNoir = vec3(0.12, 0.15, 0.1);         // Humus vert-noir
                vec3 sableBeige = vec3(0.5, 0.45, 0.3);         // Sable beige visible
                
                // VERTS dominants pour équilibre
                vec3 vertOlive = vec3(0.3, 0.4, 0.2);           // Vert olive CLAIR
                vec3 brunVert = vec3(0.25, 0.35, 0.18);         // Brun-vert équilibré
                vec3 terreRouge = vec3(0.3, 0.2, 0.12);         // Terre rouge modérée
                vec3 ocreJaune = vec3(0.4, 0.35, 0.2);          // Ocre jaune visible
                vec3 vertMousse = vec3(0.2, 0.4, 0.25);         // Vert mousse CLAIR
                vec3 brunClair = vec3(0.4, 0.3, 0.18);          // Brun clair visible
                vec3 terreGrise = vec3(0.25, 0.28, 0.2);        // Terre gris-vert
                vec3 vertSombre = vec3(0.18, 0.3, 0.15);        // Vert sombre mais visible
                vec3 rouilleBrun = vec3(0.3, 0.22, 0.15);       // Rouille modérée
                vec3 sableVert = vec3(0.35, 0.45, 0.28);        // Sable VERT clair
                
                // Nouveaux VERTS CLAIRS ajoutés
                vec3 vertTendre = vec3(0.25, 0.45, 0.3);        // Vert tendre CLAIR
                vec3 vertPrairie = vec3(0.3, 0.5, 0.25);        // Vert prairie
                vec3 vertEmeraude = vec3(0.2, 0.4, 0.3);        // Vert émeraude
                
                // Couleurs manquantes équilibrées
                vec3 terreClaire = vec3(0.45, 0.35, 0.22);      // Terre claire
                vec3 terreFeuilles = vec3(0.22, 0.25, 0.15);    // Terre avec feuilles
                vec3 terreSeche = vec3(0.4, 0.3, 0.18);         // Terre sèche
                vec3 terreMousse = vec3(0.2, 0.35, 0.2);        // Terre moussue VERTE
                vec3 terreCailloux = vec3(0.3, 0.28, 0.22);     // Terre avec cailloux
                vec3 terreSombre = vec3(0.15, 0.18, 0.12);      // Terre sombre vert-brun
                vec3 terreArgile = vec3(0.28, 0.25, 0.15);      // Terre argile
                vec3 terreOmbre = vec3(0.12, 0.2, 0.15);        // Terre dans l'ombre VERTE
                
                // Noise simple
                float noise(vec2 p) {
                    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
                }
                
                void main() {
                    vec2 pos = vPosition.xz * 0.01;
                    
                    // Variations de terrain selon l'élévation et position
                    float elevationFactor = (vElevation + 10.0) / 20.0;
                    float noiseValue = noise(pos * 10.0) * 0.3 + noise(pos * 30.0) * 0.1;
                    float microNoise = noise(pos * 50.0) * 0.05;
                    float shadowNoise = noise(pos * 15.0);
                    float moistureNoise = noise(pos * 8.0);
                    
                    // Base VERTE dominante avec nuances
                    vec3 baseColor = mix(vertMousse, terreBrune, elevationFactor * 0.6 + 0.2);
                    
                    // FORCER les zones vertes - 60% de la surface
                    float vertMarronFactor = moistureNoise * 0.8 + shadowNoise * 0.2;
                    
                    // Zones TRÈS vertes (40% du terrain)
                    if (moistureNoise > 0.5) {
                        if (vElevation < 1.0) {
                            baseColor = mix(baseColor, vertTendre, 0.7);
                            baseColor = mix(baseColor, vertPrairie, moistureNoise * 0.5);
                        } else {
                            baseColor = mix(baseColor, vertEmeraude, 0.6);
                        }
                    }
                    
                    // Zones vert-brun (30% du terrain)
                    if (moistureNoise > 0.3 && moistureNoise <= 0.7) {
                        baseColor = mix(baseColor, mix(brunVert, vertOlive, 0.7), 0.5);
                        baseColor = mix(baseColor, sableVert, noiseValue * 0.3);
                    }
                    
                    // Zones moussues VERTES (priorité au vert)
                    if (vElevation < 0.0 || moistureNoise > 0.7) {
                        vec3 mousseRiche = mix(terreMousse, vertTendre, 0.8);
                        baseColor = mix(baseColor, mousseRiche, 0.6);
                        baseColor = mix(baseColor, vertEmeraude, moistureNoise * 0.4);
                    }
                    
                    // Zones ombragées : vert sombre mais VISIBLE
                    if (shadowNoise > 0.6) {
                        vec3 ombreVerte = mix(terreOmbre, vertSombre, 0.7);
                        baseColor = mix(baseColor, ombreVerte, 0.4);
                    }
                    
                    // Zones brunes LIMITÉES (seulement 20% du terrain)
                    if (noiseValue > 0.75 && elevationFactor > 0.7 && moistureNoise < 0.4) {
                        vec3 rougeNuancee = mix(terreRouge, rouilleBrun, shadowNoise);
                        baseColor = mix(baseColor, rougeNuancee, 0.3); // Réduit l'impact
                    }
                    
                    // Feuilles : mélange vert-brun équilibré
                    if (shadowNoise > 0.4 && moistureNoise > 0.4) {
                        vec3 feuillesVertes = mix(terreFeuilles, vertMousse, 0.8);
                        baseColor = mix(baseColor, feuillesVertes, 0.3);
                    }
                    
                    // Pentes : ocre-vert au lieu de juste brun
                    float slope = 1.0 - abs(dot(vNormal, vec3(0.0, 1.0, 0.0)));
                    if (slope > 0.3 && elevationFactor > 0.5) {
                        vec3 penteVerte = mix(ocreJaune, sableVert, 0.6);
                        baseColor = mix(baseColor, penteVerte, slope * 0.4);
                    }
                    
                    // Cailloux : gris-vert au lieu de gris-brun
                    if (noiseValue > 0.7 && elevationFactor > 0.4) {
                        vec3 caillouxVerts = mix(terreCailloux, sableVert, 0.6);
                        baseColor = mix(baseColor, caillouxVerts, 0.25);
                    }
                    
                    // Variation FORCÉE vers le vert
                    vec3 variationVerte = mix(vertSombre, vertTendre, vertMarronFactor);
                    baseColor = mix(baseColor, variationVerte, noiseValue * 0.2);
                    
                    // MICRO-VARIATIONS VERTES prioritaires
                    float microVertMarron = noise(pos * 80.0);
                    float microDetail = noise(pos * 150.0);
                    float microVert = noise(pos * 200.0);
                    
                    // 70% des micro-variations en VERT
                    if (microVertMarron > 0.7) {
                        baseColor = mix(baseColor, vertTendre, 0.25);
                    } else if (microVertMarron > 0.5) {
                        baseColor = mix(baseColor, vertPrairie, 0.2);
                    } else if (microVertMarron > 0.3) {
                        baseColor = mix(baseColor, vertEmeraude, 0.18);
                    } else if (microVertMarron < 0.2) {
                        baseColor = mix(baseColor, rouilleBrun, 0.15); // Peu de brun
                    }
                    
                    // Détails fins VERTS
                    if (microDetail > 0.75) {
                        baseColor = mix(baseColor, vertMousse, 0.15);
                    } else if (microDetail > 0.6) {
                        baseColor = mix(baseColor, sableVert, 0.12);
                    } else if (microDetail < 0.25) {
                        baseColor = mix(baseColor, humusNoir, 0.08);
                    }
                    
                    // Variations ultra-fines VERTES
                    if (microVert > 0.8) {
                        baseColor = mix(baseColor, vertOlive, 0.1);
                    } else if (microVert < 0.2) {
                        baseColor = mix(baseColor, terreMousse, 0.1);
                    }
                    
                    // Micro-variations réduits pour garder les couleurs
                    baseColor += microNoise * 0.15;
                    
                    // Éclairage doux pour préserver les nuances vertes
                    float lightFactor = dot(vNormal, normalize(vec3(1.0, 1.2, 0.6)));
                    lightFactor = max(0.6, lightFactor * 0.9); // Plus clair pour voir les verts
                    baseColor *= lightFactor;
                    
                    // Pattern forestier vert dominante
                    float forestPattern = sin(pos.x * 2.5) * sin(pos.y * 2.0) * 0.03;
                    baseColor *= 0.95 + forestPattern; // Base plus claire
                    
                    gl_FragColor = vec4(baseColor, 1.0);
                }
            `,
            uniforms: {
                uTime: { value: 0.0 }
            }
        });
        
        this.terrain = new THREE.Mesh(geometry, material);
        this.terrain.rotation.x = -Math.PI / 2;
        this.terrain.receiveShadow = true;
        this.terrain.name = 'ProvencalTerrain';
        
        this.scene.add(this.terrain);
        this.materials.push(material);
        
        // Helper pour positionner sur le terrain - EXACTEMENT les mêmes calculs que le terrain
        this.getHeightAt = (x, z) => {
            // Vérifier si on est sur la plateforme centrale
            if (this.centralPlatform) {
                const distanceFromCenter = Math.sqrt(x * x + z * z);
                if (distanceFromCenter <= this.centralPlatform.radius) {
                    return this.centralPlatform.height + 0.3; // Hauteur plateforme + épaisseur surface
                }
            }
            
            // IMPORTANT: Mêmes calculs que la génération du terrain pour correspondance parfaite
            return (
                Math.sin(x * 0.02) * Math.cos(z * 0.015) * 8.0 +
                Math.sin(x * 0.05) * Math.cos(z * 0.04) * 4.0 +
                Math.sin(x * 0.1) * Math.cos(z * 0.08) * 2.0
                // Note: On retire le random pour avoir une fonction déterministe
            );
        };
        
        // Fonction spécialisée pour l'herbe qui AUTORISE l'herbe sous la plateforme
        this.getGrassHeightAt = (x, z) => {
            // Important: on ne filtre plus la zone de la plateforme, pour permettre de l'herbe en dessous
            // Utiliser la hauteur normale du terrain (pas la hauteur de la plateforme)
            return (
                Math.sin(x * 0.02) * Math.cos(z * 0.015) * 8.0 +
                Math.sin(x * 0.05) * Math.cos(z * 0.04) * 4.0 +
                Math.sin(x * 0.1) * Math.cos(z * 0.08) * 2.0
            );
        };
        
        log('✅ Terrain méditerranéen créé');
    }

    async createProvencalSkybox() {
        log('🌅 Création skybox méditerranéen...');
        
        const skyboxGeometry = new THREE.SphereGeometry(1000, 32, 32);
        const skyboxMaterial = new THREE.ShaderMaterial({
            vertexShader: ProvencalSkyboxShaders.vertex,
            fragmentShader: ProvencalSkyboxShaders.fragment,
            uniforms: {
                uSunAzimuth: { value: ProvencalSkyboxDefaults.sunAzimuth },
                uSunElevation: { value: ProvencalSkyboxDefaults.sunElevation },
                uSunColor: { value: new THREE.Vector3(...ProvencalSkyboxDefaults.sunColor) },
                uSkyColorLow: { value: new THREE.Vector3(...ProvencalSkyboxDefaults.skyColorLow) },
                uSkyColorHigh: { value: new THREE.Vector3(...ProvencalSkyboxDefaults.skyColorHigh) },
                uSunSize: { value: ProvencalSkyboxDefaults.sunSize },
                uTime: { value: 0 },
                uHazeIntensity: { value: ProvencalSkyboxDefaults.hazeIntensity },
                uHazeColor: { value: new THREE.Vector3(...ProvencalSkyboxDefaults.hazeColor) }
            },
            side: THREE.BackSide
        });
        
        this.skybox = new THREE.Mesh(skyboxGeometry, skyboxMaterial);
        this.skybox.name = 'ProvencalSkybox';
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

    async createAuthenticProvencalForest() {
        log('🌳 Génération forêt provençale authentique avec EZ-Tree...');
        
        const { x, y } = this.config.terrainSize;
        const margin = 20;
        
        // Configuration espèces provençales avec presets EZ-Tree adaptés
        const provencalSpecies = [
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

        const forestGroup = new THREE.Group();
        forestGroup.name = 'ProvencalForest';
        
        let treeCount = 0;
        const maxTrees = this.config.maxTrees;
        const spacing = this.config.minTreeSpacing;

        // Génération avec placement intelligent
        const placedPositions = [];
        
        // Zone centrale exclue (clairière)
        const centerRadius = 40; // Rayon de la clairière centrale
        
        for (let attempt = 0; attempt < maxTrees * 3 && treeCount < maxTrees; attempt++) {
            const px = (Math.random() - 0.5) * (x - margin * 2);
            const pz = (Math.random() - 0.5) * (y - margin * 2);
            
            // Exclure le centre (créer une clairière)
            const distanceFromCenter = Math.sqrt(px * px + pz * pz);
            if (distanceFromCenter < centerRadius) continue;
            
            // Vérifier espacement minimum
            const tooClose = placedPositions.some(pos => {
                const dx = px - pos.x;
                const dz = pz - pos.z;
                return Math.sqrt(dx * dx + dz * dz) < spacing;
            });
            
            if (tooClose) continue;
            
            // Sélectionner espèce selon poids
            const species = this.selectSpeciesByWeight(provencalSpecies);
            const tree = await this.createAuthenticTree(species, px, pz);
            
            if (tree) {
                forestGroup.add(tree);
                placedPositions.push({ x: px, z: pz });
                treeCount++;
                
                // Log progression
                if (treeCount % 20 === 0) {
                    log(`🌲 ${treeCount}/${maxTrees} arbres générés...`);
                }
            }
        }
        
        this.scene.add(forestGroup);
        log(`✅ Forêt provençale authentique générée: ${treeCount} arbres`);
    }

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
            
            // Appliquer modifications spécifiques à l'espèce
            if (species.modifications) {
                this.applySpeciesModifications(tree, species.modifications);
            }
            
            // Générer la géométrie
            tree.generate();
            
            // Positionner sur le terrain
            const y = this.getHeightAt ? this.getHeightAt(x, z) : 0;
            tree.position.set(x, y, z);
            
            // Rotation aléatoire
            tree.rotation.y = Math.random() * Math.PI * 2;
            
            // Ombres
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
        // Appliquer les modifications spécifiques à l'espèce
        if (modifications.leafColor && tree.leafMaterial) {
            tree.leafMaterial.color.setHex(modifications.leafColor);
        }
        
        if (modifications.barkColor && tree.barkMaterial) {
            tree.barkMaterial.color.setHex(modifications.barkColor);
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
        
        // Lumière ambiante dorée
        const ambientLight = new THREE.AmbientLight(0xFFF8DC, 0.4);
        this.scene.add(ambientLight);
        
        // Soleil méditerranéen
        const sunLight = new THREE.DirectionalLight(0xFFE4B5, 1.2);
        sunLight.position.set(100, 80, 50);
        sunLight.castShadow = true;
        
        // Ombres haute qualité
        sunLight.shadow.mapSize.width = 4096;
        sunLight.shadow.mapSize.height = 4096;
        sunLight.shadow.camera.near = 0.5;
        sunLight.shadow.camera.far = 500;
        sunLight.shadow.camera.left = -150;
        sunLight.shadow.camera.right = 150;
        sunLight.shadow.camera.top = 150;
        sunLight.shadow.camera.bottom = -150;
        sunLight.shadow.bias = -0.0001;
        
        this.scene.add(sunLight);
        
        // Lumière d'appoint chaude
        const fillLight = new THREE.HemisphereLight(0x87CEEB, 0xD2B48C, 0.6);
        this.scene.add(fillLight);
        
        // Brume méditerranéenne
        this.scene.fog = new THREE.FogExp2(0xE6E6FA, 0.0008);
        
        log('✅ Éclairage méditerranéen configuré');
    }

    async createGLSLGrassField() {
        log('🌿 Creation du système d\'herbe dynamique avec zone réduite...');
        
        try {
            // Configuration de performance optimisée avec zone réduite
            this.grassField = new GLSLGrassFieldDynamic({
                fieldSize: 120,                    // RÉDUIT de 220 à 120 pour performance
                grassCount: 25000,                 // RÉDUIT de 50k à 25k instances max
                chunkSize: 20,                     // RÉDUIT de 25 à 20 unités
                renderDistance: 60,                // RÉDUIT de 100 à 60 unités - zone visible uniquement
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
        log('🏗️ Création de la plateforme d\'atterrissage pour drones...');
        
        const platformRadius = 12; // Réduite pour 6 drones
        const platformHeight = 8.5; // UNIFIÉ: Même hauteur que PLATFORM_HEIGHT système
        
        // Groupe principal de la plateforme
        const platformGroup = new THREE.Group();
        platformGroup.name = 'DronesPlatform';
        
        // === PILIERS DE SUPPORT ===
        this.createPlatformPillars(platformGroup, platformRadius, platformHeight);
        
        // === SURFACE DE LA PLATEFORME ===
        this.createPlatformSurface(platformGroup, platformRadius, platformHeight);
        
        // === MARQUAGES DRONES ===
        this.createDroneMarkers(platformGroup, platformRadius, platformHeight);
        
        // Ajouter à la scène
        this.scene.add(platformGroup);
        
        // Stocker la référence pour la fonction getHeightAt
        this.centralPlatform = {
            position: { x: 0, y: platformHeight, z: 0 },
            radius: platformRadius,
            height: platformHeight
        };
        
        // NOUVEAU: Initialiser le système de collision avec les dimensions réelles de la plateforme
        this.collisionDetection.initialize(this.scene);
        this.collisionDetection.updatePlatformConfig(platformHeight, platformRadius);
        
        log('✅ Plateforme de drones créée avec marquages et système de collision');
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

    update(time, camera = null) {
        // Mettre à jour les éléments animés
        if (this.updateSkybox) {
            this.updateSkybox(time);
        }
        
        // Mise à jour du shader terrain
        if (this.terrain && this.terrain.material && this.terrain.material.uniforms) {
            this.terrain.material.uniforms.uTime.value = time;
        }

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
