/**
 * DIAMANTS V3 - Enhanced GLSL Grass Field with Quality Management
 * Version unifiée avec sélection dynamique de qualité
 */
import * as THREE from 'three';
import { shaderQualityManager, QUALITY_LEVELS } from '../shaders/shader-quality-manager.js';

/**
 * Champ d'herbe GLSL avec gestion de qualité dynamique
 */
export class EnhancedGLSLGrassField {
    constructor(options = {}) {
        this.options = {
            quality: options.quality || QUALITY_LEVELS.MEDIUM,
            autoQuality: options.autoQuality || false,
            density: options.density || 1.0,
            size: options.size || 100,
            windStrength: options.windStrength || 0.3,
            ...options
        };

        this.mesh = null;
        this.material = null;
        this.geometry = null;
        this.windUniforms = null;
        this.performanceMonitor = {
            lastFrameTime: 0,
            frameCount: 0,
            fps: 60
        };

        // Configuration initiale de la qualité
        shaderQualityManager.setQuality(this.options.quality);
        shaderQualityManager.setAutoQuality(this.options.autoQuality);
        
        this.init();
    }

    /**
     * Initialise le champ d'herbe
     */
    init() {
        this.createGeometry();
        this.createMaterial();
        this.createMesh();
        this.setupWindAnimation();
    }

    /**
     * Crée la géométrie basée sur la qualité
     */
    createGeometry() {
        const shaderConfig = shaderQualityManager.getShaders();
        const features = shaderConfig.features;
        
        // Ajuste la densité selon la qualité
        const density = this.options.density * features.density;
        const grassCount = Math.floor(density * 10000);
        
        console.log(`🌱 Création géométrie herbe - Qualité: ${shaderQualityManager.currentQuality}`);
        console.log(`   Brins d'herbe: ${grassCount.toLocaleString()}`);
        console.log(`   LOD: ${features.lod}`);

        // Géométrie instancée pour les performances
        const grassGeometry = new THREE.PlaneGeometry(0.1, 1, 1, features.lod);
        this.geometry = new THREE.InstancedBufferGeometry();
        this.geometry.copy(grassGeometry);

        // Positions aléatoires
        const positions = new Float32Array(grassCount * 3);
        const scales = new Float32Array(grassCount);
        const rotations = new Float32Array(grassCount);

        for (let i = 0; i < grassCount; i++) {
            const i3 = i * 3;
            
            // Position aléatoire dans l'aire
            positions[i3] = (Math.random() - 0.5) * this.options.size;
            positions[i3 + 1] = 0;
            positions[i3 + 2] = (Math.random() - 0.5) * this.options.size;
            
            // Échelle et rotation variées
            scales[i] = 0.8 + Math.random() * 0.4;
            rotations[i] = Math.random() * Math.PI * 2;
        }

        this.geometry.setAttribute('instancePosition', new THREE.InstancedBufferAttribute(positions, 3));
        this.geometry.setAttribute('instanceScale', new THREE.InstancedBufferAttribute(scales, 1));
        this.geometry.setAttribute('instanceRotation', new THREE.InstancedBufferAttribute(rotations, 1));
    }

    /**
     * Crée le matériau avec les shaders appropriés
     */
    createMaterial() {
        const shaderConfig = shaderQualityManager.getShaders();
        const features = shaderConfig.features;

        console.log(`🎨 Création matériau - Features: ${JSON.stringify(features)}`);

        // Uniforms de base
        this.windUniforms = {
            uTime: { value: 0 },
            uWindStrength: { value: this.options.windStrength },
            uWindDirection: { value: new THREE.Vector2(1, 0.5) },
            uWindFrequency: { value: 2.0 }
        };

        // Uniforms additionnels selon les features
        const uniforms = { ...this.windUniforms };
        
        if (features.shadows) {
            uniforms.uShadowIntensity = { value: 0.3 };
        }
        
        if (features.debug) {
            uniforms.uDebugMode = { value: 1.0 };
        }

        this.material = new THREE.ShaderMaterial({
            vertexShader: shaderConfig.vertex,
            fragmentShader: shaderConfig.fragment,
            uniforms: uniforms,
            side: THREE.DoubleSide,
            transparent: true,
            alphaTest: 0.5
        });

        // Configuration selon les features
        if (features.shadows) {
            this.material.shadowSide = THREE.DoubleSide;
        }
    }

    /**
     * Crée le mesh final
     */
    createMesh() {
        this.mesh = new THREE.Mesh(this.geometry, this.material);
        this.mesh.castShadow = shaderQualityManager.getShaders().features.shadows;
        this.mesh.receiveShadow = true;
        this.mesh.name = 'enhanced-grass-field';
        
        console.log(`✅ Champ d'herbe créé - Qualité: ${shaderQualityManager.currentQuality}`);
    }

    /**
     * Configure l'animation du vent
     */
    setupWindAnimation() {
        const features = shaderQualityManager.getShaders().features;
        
        if (features.wind) {
            this.windAnimation = (deltaTime) => {
                if (this.windUniforms) {
                    this.windUniforms.uTime.value += deltaTime;
                }
            };
        }
    }

    /**
     * Met à jour l'animation
     */
    update(deltaTime) {
        // Animation du vent
        if (this.windAnimation) {
            this.windAnimation(deltaTime);
        }

        // Monitoring des performances
        this.updatePerformanceMonitor(deltaTime);
    }

    /**
     * Surveille les performances et ajuste la qualité si nécessaire
     */
    updatePerformanceMonitor(deltaTime) {
        this.performanceMonitor.frameCount++;
        this.performanceMonitor.lastFrameTime += deltaTime;

        // Calcul FPS toutes les secondes
        if (this.performanceMonitor.lastFrameTime >= 1.0) {
            this.performanceMonitor.fps = this.performanceMonitor.frameCount;
            
            // Mise à jour des métriques dans le gestionnaire de qualité
            shaderQualityManager.updatePerformanceMetrics(
                this.performanceMonitor.fps,
                1, // drawCalls - à améliorer si nécessaire
                this.geometry.instanceCount || 0
            );

            // Reset des compteurs
            this.performanceMonitor.frameCount = 0;
            this.performanceMonitor.lastFrameTime = 0;
        }
    }

    /**
     * Change la qualité dynamiquement
     */
    setQuality(quality) {
        if (shaderQualityManager.setQuality(quality)) {
            // Recréer les composants avec la nouvelle qualité
            this.recreateWithQuality();
        }
    }

    /**
     * Recrée les composants avec la nouvelle qualité
     */
    recreateWithQuality() {
        console.log(`🔄 Recréation avec qualité: ${shaderQualityManager.currentQuality}`);
        
        // Sauvegarder la position
        const position = this.mesh ? this.mesh.position.clone() : new THREE.Vector3();
        const rotation = this.mesh ? this.mesh.rotation.clone() : new THREE.Euler();
        
        // Nettoyer l'ancien
        this.dispose();
        
        // Recréer
        this.init();
        
        // Restaurer la position
        if (this.mesh) {
            this.mesh.position.copy(position);
            this.mesh.rotation.copy(rotation);
        }
    }

    /**
     * Active/désactive la qualité automatique
     */
    setAutoQuality(enabled) {
        this.options.autoQuality = enabled;
        shaderQualityManager.setAutoQuality(enabled);
    }

    /**
     * Obtient les informations de qualité actuelles
     */
    getQualityInfo() {
        return {
            current: shaderQualityManager.currentQuality,
            autoEnabled: this.options.autoQuality,
            fps: this.performanceMonitor.fps,
            features: (shaderQualityManager.currentShaders && shaderQualityManager.currentShaders.features) || {},
            availableLevels: shaderQualityManager.getQualityInfo()
        };
    }

    /**
     * Détection automatique de la qualité optimale
     */
    async detectOptimalQuality(renderer) {
        if (!renderer && typeof window !== 'undefined' && window.renderer) {
            renderer = window.renderer;
        }
        const optimalQuality = await shaderQualityManager.detectOptimalQuality(renderer);
        this.setQuality(optimalQuality);
        return optimalQuality;
    }

    /**
     * Nettoie les ressources
     */
    dispose() {
        if (this.geometry) {
            this.geometry.dispose();
        }
        if (this.material) {
            this.material.dispose();
        }
    }

    /**
     * Ajoute à la scène
     */
    addToScene(scene) {
        if (this.mesh) {
            scene.add(this.mesh);
            console.log(`🌍 Champ d'herbe ajouté à la scène`);
        }
    }

    /**
     * Retire de la scène
     */
    removeFromScene(scene) {
        if (this.mesh) {
            scene.remove(this.mesh);
        }
    }
}

// Compatibilité avec les anciennes classes
export class GLSLGrassField extends EnhancedGLSLGrassField {
    constructor(options = {}) {
        super({ ...options, quality: QUALITY_LEVELS.MEDIUM });
    }
}

export class GLSLGrassFieldDynamic extends EnhancedGLSLGrassField {
    constructor(options = {}) {
        super({ ...options, autoQuality: true });
    }
}

// Export par défaut
export default EnhancedGLSLGrassField;
