/**
 * DIAMANTS V3 - Améliorateur de textures d'arbres
 * ===============================================
 * Système pour améliorer visuellement les textures et matériaux des arbres EZ-Tree
 */

import * as THREE from 'three';

export class TreeTextureEnhancer {
    constructor() {
        this.enhancedMaterials = new Map();
        this.textureCache = new Map();
        this.init();
    }

    init() {
        this.createProceduralTextures();
        this.createEnhancedMaterials();
    }

    createProceduralTextures() {
        // Création de textures procédurales pour les troncs
        this.barkTexture = this.createBarkTexture();
        this.leafTexture = this.createLeafTexture();
        this.detailTexture = this.createDetailTexture();
    }

    createBarkTexture() {
        // Texture d'écorce procédurale
        const canvas = document.createElement('canvas');
        canvas.width = 512;
        canvas.height = 512;
        const ctx = canvas.getContext('2d');

        // Base brun foncé
        ctx.fillStyle = '#4A3C28';
        ctx.fillRect(0, 0, 512, 512);

        // Ajout de texture d'écorce
        for (let i = 0; i < 200; i++) {
            const x = Math.random() * 512;
            const y = Math.random() * 512;
            const size = Math.random() * 20 + 5;
            
            ctx.fillStyle = `rgba(${90 + Math.random() * 40}, ${60 + Math.random() * 30}, ${30 + Math.random() * 20}, ${0.3 + Math.random() * 0.4})`;
            ctx.fillRect(x, y, size, size * 3);
        }

        // Lignes verticales pour effet écorce
        for (let i = 0; i < 50; i++) {
            const x = Math.random() * 512;
            ctx.strokeStyle = `rgba(${60 + Math.random() * 30}, ${40 + Math.random() * 20}, ${20 + Math.random() * 15}, ${0.4 + Math.random() * 0.3})`;
            ctx.lineWidth = 1 + Math.random() * 3;
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x + (Math.random() - 0.5) * 20, 512);
            ctx.stroke();
        }

        const texture = new THREE.CanvasTexture(canvas);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        texture.repeat.set(2, 4);
        
        return texture;
    }

    createLeafTexture() {
        // Texture de feuillage améliorée
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 256;
        const ctx = canvas.getContext('2d');

        // Base verte
        const gradient = ctx.createRadialGradient(128, 128, 0, 128, 128, 128);
        gradient.addColorStop(0, '#4CAF50');
        gradient.addColorStop(0.7, '#2E7D32');
        gradient.addColorStop(1, '#1B5E20');
        
        ctx.fillStyle = gradient;
        ctx.fillRect(0, 0, 256, 256);

        // Ajout de variations de feuilles
        for (let i = 0; i < 100; i++) {
            const x = Math.random() * 256;
            const y = Math.random() * 256;
            const size = Math.random() * 8 + 2;
            
            ctx.fillStyle = `rgba(${30 + Math.random() * 60}, ${120 + Math.random() * 60}, ${30 + Math.random() * 40}, ${0.6 + Math.random() * 0.4})`;
            
            ctx.beginPath();
            ctx.ellipse(x, y, size, size * 1.5, Math.random() * Math.PI, 0, Math.PI * 2);
            ctx.fill();
        }

        const texture = new THREE.CanvasTexture(canvas);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        
        return texture;
    }

    createDetailTexture() {
        // Texture de détail pour bump mapping
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 256;
        const ctx = canvas.getContext('2d');

        // Base grise
        ctx.fillStyle = '#808080';
        ctx.fillRect(0, 0, 256, 256);

        // Ajout de noise pour le relief
        const imageData = ctx.getImageData(0, 0, 256, 256);
        const data = imageData.data;

        for (let i = 0; i < data.length; i += 4) {
            const noise = (Math.random() - 0.5) * 100;
            data[i] = Math.max(0, Math.min(255, 128 + noise));     // R
            data[i + 1] = Math.max(0, Math.min(255, 128 + noise)); // G
            data[i + 2] = Math.max(0, Math.min(255, 128 + noise)); // B
            data[i + 3] = 255; // A
        }

        ctx.putImageData(imageData, 0, 0);

        const texture = new THREE.CanvasTexture(canvas);
        texture.wrapS = THREE.RepeatWrapping;
        texture.wrapT = THREE.RepeatWrapping;
        
        return texture;
    }

    createEnhancedMaterials() {
        // Matériau de tronc amélioré
        this.enhancedTrunkMaterial = new THREE.MeshPhongMaterial({
            map: this.barkTexture,
            bumpMap: this.detailTexture,
            bumpScale: 0.1,
            color: '#5D4E37',
            shininess: 1,
            specular: '#2F2F2F',
            transparent: false
        });

        // Matériau de feuillage amélioré
        this.enhancedLeafMaterial = new THREE.MeshLambertMaterial({
            map: this.leafTexture,
            color: '#4CAF50',
            transparent: true,
            opacity: 0.95,
            side: THREE.DoubleSide,
            alphaTest: 0.1
        });

        // Matériaux saisonniers pour variété
        this.seasonalMaterials = {
            spring: new THREE.MeshLambertMaterial({
                color: '#7CB342',
                transparent: true,
                opacity: 0.9,
                side: THREE.DoubleSide
            }),
            summer: new THREE.MeshLambertMaterial({
                color: '#4CAF50',
                transparent: true,
                opacity: 0.95,
                side: THREE.DoubleSide
            }),
            autumn: new THREE.MeshLambertMaterial({
                color: '#FF8F00',
                transparent: true,
                opacity: 0.9,
                side: THREE.DoubleSide
            }),
            winter: new THREE.MeshLambertMaterial({
                color: '#6D4C41',
                transparent: true,
                opacity: 0.7,
                side: THREE.DoubleSide
            })
        };
    }

    // Méthode pour améliorer un arbre existant
    enhanceTree(tree, options = {}) {
        const season = options.season || 'summer';
        const trunkVariation = options.trunkVariation || 0;
        
        if (!tree) return;

        // Parcours récursif de l'arbre pour améliorer les matériaux
        tree.traverse((child) => {
            if (child.isMesh) {
                // Amélioration du tronc
                if (this.isTrunkMesh(child)) {
                    child.material = this.enhancedTrunkMaterial.clone();
                    
                    // Variation de couleur pour le tronc
                    if (trunkVariation > 0) {
                        const hue = 0.08 + (Math.random() - 0.5) * trunkVariation * 0.02;
                        const saturation = 0.3 + Math.random() * 0.2;
                        const lightness = 0.2 + Math.random() * 0.1;
                        child.material.color.setHSL(hue, saturation, lightness);
                    }
                    
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
                
                // Amélioration du feuillage
                if (this.isLeafMesh(child)) {
                    child.material = this.seasonalMaterials[season].clone();
                    
                    // Variation saisonnière
                    if (season === 'autumn') {
                        const autumnColors = ['#FF8F00', '#FF6F00', '#E65100', '#BF360C'];
                        const randomColor = autumnColors[Math.floor(Math.random() * autumnColors.length)];
                        child.material.color.setStyle(randomColor);
                    }
                    
                    child.castShadow = true;
                    child.receiveShadow = true;
                }
            }
        });
    }

    // Détection de type de mesh (basique)
    isTrunkMesh(mesh) {
        // Heuristique pour détecter un tronc
        const geometry = mesh.geometry;
        if (!geometry || !geometry.attributes || !geometry.attributes.position) return false;
        
        const positions = geometry.attributes.position.array;
        const boundingBox = new THREE.Box3().setFromBufferAttribute(geometry.attributes.position);
        const height = boundingBox.max.y - boundingBox.min.y;
        const width = Math.max(boundingBox.max.x - boundingBox.min.x, boundingBox.max.z - boundingBox.min.z);
        
        // Si plus haut que large, probablement un tronc
        return height > width * 1.5;
    }

    isLeafMesh(mesh) {
        // Heuristique pour détecter le feuillage
        const geometry = mesh.geometry;
        if (!geometry || !geometry.attributes || !geometry.attributes.position) return false;
        
        // Si pas un tronc et a beaucoup de vertices, probablement du feuillage
        return !this.isTrunkMesh(mesh) && geometry.attributes.position.count > 100;
    }

    // Amélioration de groupe d'arbres
    enhanceTreeGroup(trees, options = {}) {
        const seasons = ['spring', 'summer', 'autumn'];
        
        trees.forEach((tree, index) => {
            // Variation saisonnière pour réalisme
            const season = options.uniformSeason || seasons[index % seasons.length];
            const trunkVariation = options.trunkVariation || 0.3;
            
            this.enhanceTree(tree, {
                season: season,
                trunkVariation: trunkVariation
            });
        });
    }

    // Nettoyage des ressources
    dispose() {
        // Nettoyer les textures
        if (this.barkTexture) this.barkTexture.dispose();
        if (this.leafTexture) this.leafTexture.dispose();
        if (this.detailTexture) this.detailTexture.dispose();
        
        // Nettoyer les matériaux
        if (this.enhancedTrunkMaterial) this.enhancedTrunkMaterial.dispose();
        if (this.enhancedLeafMaterial) this.enhancedLeafMaterial.dispose();
        
        Object.values(this.seasonalMaterials).forEach(material => {
            material.dispose();
        });
        
        this.enhancedMaterials.clear();
        this.textureCache.clear();
    }
}
