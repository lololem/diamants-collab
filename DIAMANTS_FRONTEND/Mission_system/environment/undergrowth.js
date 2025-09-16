/**
 * DIAMANTS - Système de sous-bois
 * ===================================
 * Fougères, petites plantes et végétation de sous-bois pour enrichir l'environnement forestier
 */

import * as THREE from 'three';

export class UndergrowthSystem {
    constructor(scene, terrainHeightFunction = null, config = {}) {
        this.scene = scene;
        this.getTerrainHeight = terrainHeightFunction;
        this.config = {
            fernCount: config.fernCount || 800,        // Nombre de fougères
            bushCount: config.bushCount || 600,        // Nombre de buissons
            smallPlantCount: config.smallPlantCount || 1200,  // Petites plantes
            fieldSize: config.fieldSize || 150,
            ...config
        };

        this.undergrowthObjects = [];
        this.init();
    }

    init() {
        this.createMaterials();
        this.createFerns();
        this.createSmallBushes();
        this.createSmallPlants();
    }

    createMaterials() {
        this.materials = {
            // Fougères - plusieurs nuances de vert
            fern_bright: new THREE.MeshLambertMaterial({ 
                color: '#228B22',  // Vert forêt vif
                transparent: true,
                opacity: 0.85
            }),
            fern_dark: new THREE.MeshLambertMaterial({ 
                color: '#006400',  // Vert foncé
                transparent: true,
                opacity: 0.85
            }),
            fern_young: new THREE.MeshLambertMaterial({ 
                color: '#32CD32',  // Vert citron pour jeunes pousses
                transparent: true,
                opacity: 0.8
            }),
            
            // Buissons
            bush_dense: new THREE.MeshLambertMaterial({ 
                color: '#2E7D32',  // Vert dense
                transparent: true,
                opacity: 0.9
            }),
            bush_light: new THREE.MeshLambertMaterial({ 
                color: '#4CAF50',  // Vert plus clair
                transparent: true,
                opacity: 0.85
            }),
            
            // Petites plantes
            plant_grass: new THREE.MeshLambertMaterial({ 
                color: '#7CB342',  // Vert herbe
                transparent: true,
                opacity: 0.8
            }),
            plant_moss: new THREE.MeshLambertMaterial({ 
                color: '#8BC34A',  // Vert mousse
                transparent: true,
                opacity: 0.75
            })
        };
    }

    createFerns() {
        // Géométries de fougères - formes caractéristiques
        const fernGeometries = [
            this.createFernGeometry(0.8, 1.2),  // Grande fougère
            this.createFernGeometry(0.5, 0.8),  // Fougère moyenne
            this.createFernGeometry(0.3, 0.5)   // Petite fougère
        ];

        const fernMaterials = [
            this.materials.fern_bright,
            this.materials.fern_dark,
            this.materials.fern_young
        ];

        for (let i = 0; i < this.config.fernCount; i++) {
            // Position avec clustering naturel
            let x, z;
            if (Math.random() < 0.7) {
                // 70% de chance d'être dans une zone humide (clustering)
                const clusterCenter = {
                    x: (Math.random() - 0.5) * this.config.fieldSize * 0.8,
                    z: (Math.random() - 0.5) * this.config.fieldSize * 0.8
                };
                x = clusterCenter.x + (Math.random() - 0.5) * 15;
                z = clusterCenter.z + (Math.random() - 0.5) * 15;
            } else {
                x = (Math.random() - 0.5) * this.config.fieldSize;
                z = (Math.random() - 0.5) * this.config.fieldSize;
            }

            const y = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
            
            // Sélection aléatoire de géométrie et matériau
            const geometry = fernGeometries[Math.floor(Math.random() * fernGeometries.length)];
            const material = fernMaterials[Math.floor(Math.random() * fernMaterials.length)];
            
            const fern = new THREE.Mesh(geometry, material);
            
            fern.position.set(x, y, z);
            fern.rotation.y = Math.random() * Math.PI * 2;
            fern.rotation.x = (Math.random() - 0.5) * 0.2;
            
            // Variation de taille
            const scale = 0.7 + Math.random() * 0.6;
            fern.scale.set(scale, scale, scale);
            
            fern.castShadow = true;
            fern.receiveShadow = true;
            
            this.scene.add(fern);
            this.undergrowthObjects.push(fern);
        }
    }

    createFernGeometry(width, height) {
        // Création d'une géométrie de fougère stylisée
        const group = new THREE.Group();
        
        // Tige principale
        const stemGeometry = new THREE.CylinderGeometry(0.02, 0.03, height, 6);
        const stemMaterial = new THREE.MeshLambertMaterial({ color: '#8B4513' });
        const stem = new THREE.Mesh(stemGeometry, stemMaterial);
        stem.position.y = height / 2;
        group.add(stem);
        
        // Frondes (feuilles de fougère) - disposition caractéristique
        const frondeCount = 8 + Math.floor(Math.random() * 6);
        for (let i = 0; i < frondeCount; i++) {
            const frondeHeight = height * (0.6 + i * 0.05);
            const frondeWidth = width * (0.3 + i * 0.08) * (1 - i / frondeCount);
            
            const frondeGeometry = new THREE.PlaneGeometry(frondeWidth, frondeHeight * 0.3);
            const fronde = new THREE.Mesh(frondeGeometry, this.materials.fern_bright);
            
            // Position en spirale autour de la tige
            const angle = (i / frondeCount) * Math.PI * 2 + Math.random() * 0.5;
            const radius = width * 0.3;
            
            fronde.position.x = Math.cos(angle) * radius;
            fronde.position.z = Math.sin(angle) * radius;
            fronde.position.y = frondeHeight;
            
            fronde.rotation.x = -Math.PI / 6;
            fronde.rotation.y = angle;
            fronde.rotation.z = (Math.random() - 0.5) * 0.3;
            
            group.add(fronde);
        }
        
        return group;
    }

    createSmallBushes() {
        for (let i = 0; i < this.config.bushCount; i++) {
            const x = (Math.random() - 0.5) * this.config.fieldSize;
            const z = (Math.random() - 0.5) * this.config.fieldSize;
            const y = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
            
            // Buisson sphérique avec variation
            const bushSize = 0.3 + Math.random() * 0.4;
            const bushGeometry = new THREE.SphereGeometry(bushSize, 8, 6);
            const bushMaterial = Math.random() > 0.5 ? this.materials.bush_dense : this.materials.bush_light;
            
            const bush = new THREE.Mesh(bushGeometry, bushMaterial);
            bush.position.set(x, y + bushSize * 0.5, z);
            
            // Déformation légère pour naturalisme
            bush.scale.set(
                0.8 + Math.random() * 0.4,
                0.6 + Math.random() * 0.8,
                0.8 + Math.random() * 0.4
            );
            
            bush.castShadow = true;
            bush.receiveShadow = true;
            
            this.scene.add(bush);
            this.undergrowthObjects.push(bush);
        }
    }

    createSmallPlants() {
        for (let i = 0; i < this.config.smallPlantCount; i++) {
            const x = (Math.random() - 0.5) * this.config.fieldSize;
            const z = (Math.random() - 0.5) * this.config.fieldSize;
            const y = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;
            
            // Petites touffes d'herbe et mousse
            const plantHeight = 0.1 + Math.random() * 0.3;
            const plantGeometry = new THREE.ConeGeometry(0.05, plantHeight, 4);
            const plantMaterial = Math.random() > 0.5 ? this.materials.plant_grass : this.materials.plant_moss;
            
            const plant = new THREE.Mesh(plantGeometry, plantMaterial);
            plant.position.set(x, y + plantHeight * 0.5, z);
            
            // Rotation et scale aléatoires
            plant.rotation.y = Math.random() * Math.PI * 2;
            const scale = 0.5 + Math.random() * 1.0;
            plant.scale.set(scale, scale, scale);
            
            this.scene.add(plant);
            this.undergrowthObjects.push(plant);
        }
    }

    // Animation subtile avec le vent
    update(time) {
        const windStrength = Math.sin(time * 0.001) * 0.02;
        
        this.undergrowthObjects.forEach((obj, index) => {
            if (obj.userData.isMovable !== false) {
                // Animation subtile seulement pour certains éléments
                if (index % 5 === 0) {
                    obj.rotation.z = windStrength * Math.sin(time * 0.002 + index * 0.1);
                }
            }
        });
    }

    // Nettoyage des ressources
    dispose() {
        this.undergrowthObjects.forEach(obj => {
            this.scene.remove(obj);
            if (obj.geometry) {
                if (obj.geometry.dispose) obj.geometry.dispose();
                // Pour les groupes, nettoyer récursivement
                if (obj.children) {
                    obj.children.forEach(child => {
                        if (child.geometry) child.geometry.dispose();
                        if (child.material) child.material.dispose();
                    });
                }
            }
            if (obj.material) obj.material.dispose();
        });
        
        Object.values(this.materials).forEach(material => {
            material.dispose();
        });
        
        this.undergrowthObjects = [];
    }
}
