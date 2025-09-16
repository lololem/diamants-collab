/**
 * DIAMANTS V3 - Système de feuilles tombées au sol enrichi
 * ========================================================
 * Simulation de feuilles d'automne multicolores dispersées naturellement sur le terrain
 */

import * as THREE from 'three';

// Mode silencieux global
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class FallenLeavesSystem {
    constructor(scene, terrainHeightFunction = null, config = {}) {
        this.scene = scene;
        this.getTerrainHeight = terrainHeightFunction;
        this.config = {
            count: config.count || 5000,           // Nombre de feuilles
            fieldSize: config.fieldSize || 150,    // Zone de dispersion
            leafScale: config.leafScale || 0.3,    // Taille des feuilles
            ...config
        };

        this.leafInstances = [];
        this.leafTypes = [];
        this.createColorfulLeafTypes();
        this.generateFallenLeaves();
    }

    createColorfulLeafTypes() {
        // Créer différents types de feuilles avec géométries détaillées et textures réalistes
        
        // === FEUILLES DE CHÊNE avec forme lobée ===
        const oakGeometry = this.createOakLeafGeometry(0.4, 0.6);
        
        // Chêne brun classique avec normale mapping
        const oakBrown = new THREE.MeshPhongMaterial({
            color: '#8B4513',
            transparent: true,
            opacity: 0.9,
            side: THREE.DoubleSide,
            shininess: 5,
            normalMap: this.createLeafNormalTexture('#8B4513')
        });
        
        // Chêne orange automnal
        const oakOrange = new THREE.MeshPhongMaterial({
            color: '#FF8C00',
            transparent: true,
            opacity: 0.9,
            side: THREE.DoubleSide,
            shininess: 5,
            normalMap: this.createLeafNormalTexture('#FF8C00')
        });

        // === FEUILLES D'ÉRABLE avec forme palmée ===
        const mapleGeometry = this.createMapleLeafGeometry(0.5, 0.5);
        
        // Érable rouge vif avec texture détaillée
        const mapleRed = new THREE.MeshPhongMaterial({
            color: '#DC143C',
            transparent: true,
            opacity: 0.9,
            side: THREE.DoubleSide,
            shininess: 8,
            normalMap: this.createLeafNormalTexture('#DC143C')
        });
        
        // Érable jaune doré
        const mapleYellow = new THREE.MeshPhongMaterial({
            color: '#FFD700',
            transparent: true,
            opacity: 0.9,
            side: THREE.DoubleSide,
            shininess: 8,
            normalMap: this.createLeafNormalTexture('#FFD700')
        });

        // === FEUILLES DE BOULEAU avec forme ovale ===
        const birchGeometry = this.createBirchLeafGeometry(0.3, 0.4);
        
        // Bouleau jaune clair
        const birchYellow = new THREE.MeshPhongMaterial({
            color: '#F0E68C',
            transparent: true,
            opacity: 0.85,
            side: THREE.DoubleSide,
            shininess: 6,
            normalMap: this.createLeafNormalTexture('#F0E68C')
        });

        // === FEUILLES DE PLATANE ===
        const planeGeometry = new THREE.PlaneGeometry(0.6, 0.5);
        
        // Platane doré
        const planeGold = new THREE.MeshLambertMaterial({
            color: '#DAA520',
            transparent: true,
            opacity: 0.8,
            side: THREE.DoubleSide
        });
        
        // Platane brun
        const planeBrown = new THREE.MeshLambertMaterial({
            color: '#CD853F',
            transparent: true,
            opacity: 0.8,
            side: THREE.DoubleSide
        });

        // === FEUILLES DE CHÂTAIGNIER ===
        const chestnutGeometry = new THREE.PlaneGeometry(0.35, 0.8);
        
        const chestnutBrown = new THREE.MeshLambertMaterial({
            color: '#A0522D',
            transparent: true,
            opacity: 0.8,
            side: THREE.DoubleSide
        });

        // === FEUILLES PERSISTANTES ===
        const evergreenGeometry = new THREE.PlaneGeometry(0.15, 0.6);
        
        const evergreenDark = new THREE.MeshLambertMaterial({
            color: '#2F4F2F',
            transparent: true,
            opacity: 0.7,
            side: THREE.DoubleSide
        });

        // Distribution des types de feuilles
        this.leafTypes = [
            { geometry: oakGeometry, material: oakBrown, weight: 0.2, name: 'oak_brown' },
            { geometry: oakGeometry, material: oakOrange, weight: 0.15, name: 'oak_orange' },
            { geometry: mapleGeometry, material: mapleRed, weight: 0.15, name: 'maple_red' },
            { geometry: mapleGeometry, material: mapleYellow, weight: 0.15, name: 'maple_yellow' },
            { geometry: birchGeometry, material: birchYellow, weight: 0.1, name: 'birch_yellow' },
            { geometry: planeGeometry, material: planeGold, weight: 0.1, name: 'plane_gold' },
            { geometry: planeGeometry, material: planeBrown, weight: 0.08, name: 'plane_brown' },
            { geometry: chestnutGeometry, material: chestnutBrown, weight: 0.05, name: 'chestnut' },
            { geometry: evergreenGeometry, material: evergreenDark, weight: 0.02, name: 'evergreen' }
        ];
    }

    generateFallenLeaves() {
        log('🍂 Génération des feuilles tombées...');

        for (let i = 0; i < this.config.count; i++) {
            // Position aléatoire dans la zone
            const x = (Math.random() - 0.5) * this.config.fieldSize;
            const z = (Math.random() - 0.5) * this.config.fieldSize;
            const terrainHeight = this.getTerrainHeight ? this.getTerrainHeight(x, z) : 0;

            // Sélection du type de feuille selon probabilité
            const leafType = this.selectLeafType();
            
            // Création de la feuille
            const leaf = new THREE.Mesh(leafType.geometry, leafType.material);
            
            // Positionnement sur le terrain
            leaf.position.set(x, terrainHeight + 0.01, z); // Légèrement au-dessus du sol
            
            // Rotation aléatoire naturelle
            leaf.rotation.x = -Math.PI / 2 + (Math.random() - 0.5) * 0.3; // Principalement plate
            leaf.rotation.y = Math.random() * Math.PI * 2; // Rotation libre
            leaf.rotation.z = (Math.random() - 0.5) * 0.4; // Légère inclinaison
            
            // Échelle variable
            const scale = this.config.leafScale * (0.5 + Math.random() * 0.5);
            leaf.scale.set(scale, scale, scale);
            
            // Cluster naturel - certaines feuilles près d'autres
            if (Math.random() < 0.3 && this.leafInstances.length > 0) {
                const nearbyLeaf = this.leafInstances[Math.floor(Math.random() * this.leafInstances.length)];
                const offset = (Math.random() - 0.5) * 2;
                leaf.position.x = nearbyLeaf.position.x + offset;
                leaf.position.z = nearbyLeaf.position.z + offset;
                leaf.position.y = this.getTerrainHeight ? this.getTerrainHeight(leaf.position.x, leaf.position.z) + 0.01 : 0.01;
            }

            this.scene.add(leaf);
            this.leafInstances.push(leaf);
        }

        log(`✅ ${this.config.count} feuilles tombées générées`);
    }

    selectLeafType() {
        const random = Math.random();
        let accumulated = 0;
        
        for (const leafType of this.leafTypes) {
            accumulated += leafType.weight;
            if (random <= accumulated) {
                return leafType;
            }
        }
        
        return this.leafTypes[0]; // Fallback
    }

    // Animation légère des feuilles (optionnelle)
    animate(time) {
        if (!this.leafInstances.length) return;

        // Animation subtile de quelques feuilles
        for (let i = 0; i < this.leafInstances.length; i += 50) { // Seulement une feuille sur 50
            const leaf = this.leafInstances[i];
            if (leaf) {
                leaf.rotation.z += Math.sin(time * 0.0005 + i) * 0.001;
            }
        }
    }

    dispose() {
        // Nettoyage des ressources
        this.leafInstances.forEach(leaf => {
            this.scene.remove(leaf);
            leaf.geometry.dispose();
            leaf.material.dispose();
        });
        this.leafInstances = [];
        
        this.leafTypes.forEach(type => {
            type.geometry.dispose();
            type.material.dispose();
        });
        this.leafTypes = [];
    }

    // Méthodes pour créer des géométries de feuilles réalistes
    createOakLeafGeometry(width, height) {
        const shape = new THREE.Shape();
        const segments = 16;
        
        // Forme lobée caractéristique du chêne
        for (let i = 0; i <= segments; i++) {
            const angle = (i / segments) * Math.PI * 2;
            const radius = (width / 2) * (1 + 0.3 * Math.sin(angle * 4)); // 4 lobes
            const x = Math.cos(angle) * radius;
            const y = Math.sin(angle) * radius * (height / width);
            
            if (i === 0) {
                shape.moveTo(x, y);
            } else {
                shape.lineTo(x, y);
            }
        }
        
        const geometry = new THREE.ShapeGeometry(shape);
        return geometry;
    }

    createMapleLeafGeometry(width, height) {
        const shape = new THREE.Shape();
        const points = [
            [0, height/2],      // Pointe du haut
            [width/4, height/4], // Lobe droit haut
            [width/2, 0],       // Pointe droite
            [width/4, -height/4], // Lobe droit bas
            [0, -height/2],     // Pointe du bas
            [-width/4, -height/4], // Lobe gauche bas
            [-width/2, 0],      // Pointe gauche
            [-width/4, height/4] // Lobe gauche haut
        ];
        
        shape.moveTo(points[0][0], points[0][1]);
        for (let i = 1; i < points.length; i++) {
            shape.lineTo(points[i][0], points[i][1]);
        }
        shape.lineTo(points[0][0], points[0][1]);
        
        const geometry = new THREE.ShapeGeometry(shape);
        return geometry;
    }

    createBirchLeafGeometry(width, height) {
        const shape = new THREE.Shape();
        const segments = 20;
        
        // Forme ovale dentelée caractéristique du bouleau
        for (let i = 0; i <= segments; i++) {
            const angle = (i / segments) * Math.PI * 2;
            let radius = (width / 2) * (1 + 0.1 * Math.sin(angle * 8)); // Dentelures fines
            
            // Forme ovale
            const x = Math.cos(angle) * radius;
            const y = Math.sin(angle) * radius * (height / width) * 1.3; // Plus allongé
            
            if (i === 0) {
                shape.moveTo(x, y);
            } else {
                shape.lineTo(x, y);
            }
        }
        
        const geometry = new THREE.ShapeGeometry(shape);
        return geometry;
    }

    createLeafNormalTexture(baseColor) {
        const canvas = document.createElement('canvas');
        canvas.width = 64;
        canvas.height = 64;
        const ctx = canvas.getContext('2d');
        
        // Créer une texture de normale pour simuler les nervures
        const gradient = ctx.createLinearGradient(0, 0, canvas.width, canvas.height);
        gradient.addColorStop(0, '#8080FF'); // Bleu pour les normales
        gradient.addColorStop(0.5, '#8080C0');
        gradient.addColorStop(1, '#8080FF');
        
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Ajouter des nervures
        ctx.strokeStyle = '#6060A0';
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(canvas.width/2, 0);
        ctx.lineTo(canvas.width/2, canvas.height);
        for (let i = 1; i < 4; i++) {
            const y = (canvas.height / 4) * i;
            ctx.moveTo(canvas.width/2, y);
            ctx.lineTo(canvas.width/4, y + 10);
            ctx.moveTo(canvas.width/2, y);
            ctx.lineTo(canvas.width * 3/4, y + 10);
        }
        ctx.stroke();
        
        const texture = new THREE.CanvasTexture(canvas);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        return texture;
    }
}
