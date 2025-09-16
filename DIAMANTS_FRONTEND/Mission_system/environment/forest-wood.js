// Forest Wood System - Système de bois et branches pour le sol forestier
import * as THREE from 'three';

export class ForestWoodSystem {
    constructor(scene, terrain, config = {}) {
        this.scene = scene;
        this.terrain = terrain;
        this.config = {
            branchCount: config.branchCount || 1500,  // Nombre de branches
            logCount: config.logCount || 300,         // Nombre de troncs tombés
            stickCount: config.stickCount || 800,     // Nombre de petites branches
            terrainSize: config.terrainSize || 160,
            ...config
        };
        
        this.woodObjects = [];
        this.init();
    }

    init() {
        this.createWoodMaterials();
        this.createBranches();
        this.createFallenLogs();
        this.createSticks();
    }

    createWoodMaterials() {
        // Matériaux pour différents types de bois - PLUS VISIBLES
        this.materials = {
            branch: new THREE.MeshLambertMaterial({ 
                color: '#8B4513',  // Brun selle plus visible
                transparent: false,
                opacity: 1.0
            }),
            log: new THREE.MeshLambertMaterial({ 
                color: '#A0522D',  // Brun sienna plus clair
                transparent: false,
                opacity: 1.0
            }),
            stick: new THREE.MeshLambertMaterial({ 
                color: '#654321',  // Brun chocolat noir visible
                transparent: false,
                opacity: 1.0
            })
        };
    }

    createBranches() {
        // Géométries pour les branches - PLUS GROSSES et VISIBLES
        const branchGeometries = [
            new THREE.CylinderGeometry(0.15, 0.1, 2.5, 8),  // Branche grosse plus visible
            new THREE.CylinderGeometry(0.12, 0.08, 1.8, 8), // Branche moyenne plus grosse
            new THREE.CylinderGeometry(0.2, 0.15, 3.0, 10), // Très grosse branche
            new THREE.CylinderGeometry(0.08, 0.05, 1.2, 6)  // Petite branche plus visible
        ];

        const branchMesh = new THREE.InstancedMesh(
            branchGeometries[0], 
            this.materials.branch, 
            this.config.branchCount
        );

        for (let i = 0; i < this.config.branchCount; i++) {
            const matrix = new THREE.Matrix4();
            
            // Position aléatoire
            const x = (Math.random() - 0.5) * this.config.terrainSize;
            const z = (Math.random() - 0.5) * this.config.terrainSize;
            const y = this.getTerrainHeight(x, z) + 0.1; // Plus haut au-dessus du sol
            
            // Rotation aléatoire pour un aspect naturel
            const rotationX = (Math.random() - 0.5) * 0.5; // Plus d'inclinaison
            const rotationY = Math.random() * Math.PI * 2;  // Rotation libre sur Y
            const rotationZ = (Math.random() - 0.5) * 0.6; // Plus d'inclinaison Z
            
            // Scale plus gros pour être visible
            const scale = 1.0 + Math.random() * 1.2;
            
            matrix.compose(
                new THREE.Vector3(x, y, z),
                new THREE.Euler(rotationX, rotationY, rotationZ),
                new THREE.Vector3(scale, scale, scale)
            );
            
            branchMesh.setMatrixAt(i, matrix);
        }

        branchMesh.instanceMatrix.needsUpdate = true;
        branchMesh.castShadow = true;
        branchMesh.receiveShadow = true;
        this.scene.add(branchMesh);
        this.woodObjects.push(branchMesh);
    }

    createFallenLogs() {
        // Troncs tombés - PLUS GROS et VISIBLES
        const logGeometry = new THREE.CylinderGeometry(0.4, 0.35, 4.5, 16);
        const logMesh = new THREE.InstancedMesh(
            logGeometry, 
            this.materials.log, 
            this.config.logCount
        );

        for (let i = 0; i < this.config.logCount; i++) {
            const matrix = new THREE.Matrix4();
            
            // Position avec clustering - troncs tombés en groupes
            let x, z;
            if (Math.random() < 0.6) {
                // 60% de chance d'être près d'un autre tronc (clustering)
                const clusterCenter = {
                    x: (Math.random() - 0.5) * this.config.terrainSize * 0.8,
                    z: (Math.random() - 0.5) * this.config.terrainSize * 0.8
                };
                x = clusterCenter.x + (Math.random() - 0.5) * 15;
                z = clusterCenter.z + (Math.random() - 0.5) * 15;
            } else {
                x = (Math.random() - 0.5) * this.config.terrainSize;
                z = (Math.random() - 0.5) * this.config.terrainSize;
            }
            
            const y = this.getTerrainHeight(x, z) + 0.2; // Plus haut pour être visible
            
            // Troncs couchés avec rotation sur X ou Z
            const rotationX = Math.random() < 0.5 ? Math.PI / 2 : 0;
            const rotationY = Math.random() * Math.PI * 2;
            const rotationZ = Math.random() < 0.5 ? Math.PI / 2 : 0;
            
            // Variation de taille plus importante
            const scale = 1.2 + Math.random() * 1.0;
            
            matrix.compose(
                new THREE.Vector3(x, y, z),
                new THREE.Euler(rotationX, rotationY, rotationZ),
                new THREE.Vector3(scale, scale, scale)
            );
            
            logMesh.setMatrixAt(i, matrix);
        }

        logMesh.instanceMatrix.needsUpdate = true;
        logMesh.castShadow = true;
        logMesh.receiveShadow = true;
        this.scene.add(logMesh);
        this.woodObjects.push(logMesh);
    }

    createSticks() {
        // Petites brindilles et bâtons - PLUS VISIBLES
        const stickGeometry = new THREE.CylinderGeometry(0.04, 0.03, 0.8, 6);
        const stickMesh = new THREE.InstancedMesh(
            stickGeometry, 
            this.materials.stick, 
            this.config.stickCount
        );

        for (let i = 0; i < this.config.stickCount; i++) {
            const matrix = new THREE.Matrix4();
            
            // Distribution dense autour des troncs et branches
            const x = (Math.random() - 0.5) * this.config.terrainSize;
            const z = (Math.random() - 0.5) * this.config.terrainSize;
            const y = this.getTerrainHeight(x, z) + 0.05; // Plus haut pour être visible
            
            // Rotation complètement aléatoire pour un aspect naturel
            const rotationX = Math.random() * Math.PI;
            const rotationY = Math.random() * Math.PI * 2;
            const rotationZ = Math.random() * Math.PI;
            
            // Scale plus gros pour être visible
            const scale = 1.0 + Math.random() * 1.5;
            
            matrix.compose(
                new THREE.Vector3(x, y, z),
                new THREE.Euler(rotationX, rotationY, rotationZ),
                new THREE.Vector3(scale, scale, scale)
            );
            
            stickMesh.setMatrixAt(i, matrix);
        }

        stickMesh.instanceMatrix.needsUpdate = true;
        stickMesh.castShadow = true;
        stickMesh.receiveShadow = true;
        this.scene.add(stickMesh);
        this.woodObjects.push(stickMesh);
    }

    // Fonction pour obtenir la hauteur du terrain (synchronisée avec le terrain principal)
    getTerrainHeight(x, z) {
        // Utilisation des mêmes fonctions que le terrain principal
        const scale1 = 0.02;
        const scale2 = 0.05;
        const scale3 = 0.1;
        
        const noise1 = Math.sin(x * scale1) * Math.cos(z * scale1) * 2;
        const noise2 = Math.sin(x * scale2) * Math.cos(z * scale2) * 1;
        const noise3 = Math.sin(x * scale3) * Math.cos(z * scale3) * 0.5;
        
        return (noise1 + noise2 + noise3) * 0.8;
    }

    // Fonction de mise à jour si nécessaire
    update(time, camera) {
        // Animation subtile des petites branches avec le vent
        if (this.woodObjects.length > 0) {
            const windEffect = Math.sin(time * 0.001) * 0.02;
            // Les bois sont statiques, pas d'animation nécessaire pour le réalisme
        }
    }

    // Nettoyage
    dispose() {
        this.woodObjects.forEach(obj => {
            this.scene.remove(obj);
            if (obj.geometry) obj.geometry.dispose();
            if (obj.material) obj.material.dispose();
        });
        
        Object.values(this.materials).forEach(material => {
            material.dispose();
        });
        
        this.woodObjects = [];
    }
}
