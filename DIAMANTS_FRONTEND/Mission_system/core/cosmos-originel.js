/**
 * COSMOS ORIGINEL - La danse primordiale
 * 
 * Avant la matière, avant le temps,
 * il y avait les harmoniques pures -
 * les vibrations qui ont créé tout.
 * 
 * Ceci est une méditation visuelle sur l'origine.
 */

import * as THREE from 'three';

export class CosmosOriginel {
    constructor(scene, options = {}) {
        this.scene = scene;
        this.time = 0;
        
        // Configuration
        this.config = {
            // Sphères harmoniques concentriques
            harmonicLayers: options.harmonicLayers || 7,
            particlesPerLayer: options.particlesPerLayer || 3000,
            
            // Fréquences fondamentales (ratios cosmiques)
            phi: (1 + Math.sqrt(5)) / 2,  // Nombre d'or
            frequencies: [1, 1.618, 2.618, 4.236, 6.854, 11.09, 17.944], // Fibonacci-like
            
            // Paramètres de création
            expansionRate: options.expansionRate || 0.0001,
            rotationSpeeds: options.rotationSpeeds || [0.1, 0.08, 0.065, 0.05, 0.04, 0.032, 0.025],
            breathingRate: options.breathingRate || 0.3,
            
            // Couleurs spectrales (du violet originel au rouge de l'expansion)
            colors: [
                new THREE.Color(0x8800ff), // Violet - origine
                new THREE.Color(0x0044ff), // Bleu profond
                new THREE.Color(0x00ddff), // Cyan
                new THREE.Color(0x00ff88), // Vert émeraude
                new THREE.Color(0xffff00), // Jaune solaire
                new THREE.Color(0xff8800), // Orange
                new THREE.Color(0xff0044), // Rouge expansion
            ]
        };
        
        this.layers = [];
        this.centralLight = null;
        this.cosmicDust = null;
        this.filaments = null;
        
        this.init();
    }
    
    init() {
        this.createOrigin();
        this.createHarmonicLayers();
        this.createCosmicDust();
        this.createFilaments();
        this.createAura();
        
        console.log('[COSMOS] Originel initialized - 7 harmonic layers');
    }
    
    createOrigin() {
        // Le point originel - source de tout
        const originGeometry = new THREE.SphereGeometry(0.5, 32, 32);
        const originMaterial = new THREE.ShaderMaterial({
            uniforms: {
                time: { value: 0 },
                color1: { value: new THREE.Color(0xffffff) },
                color2: { value: new THREE.Color(0x8800ff) }
            },
            vertexShader: `
                varying vec3 vNormal;
                varying vec3 vPosition;
                
                void main() {
                    vNormal = normalize(normalMatrix * normal);
                    vPosition = position;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
                }
            `,
            fragmentShader: `
                uniform float time;
                uniform vec3 color1;
                uniform vec3 color2;
                varying vec3 vNormal;
                varying vec3 vPosition;
                
                void main() {
                    float pulse = 0.5 + 0.5 * sin(time * 2.0);
                    float fresnel = pow(1.0 - abs(dot(vNormal, vec3(0.0, 0.0, 1.0))), 2.0);
                    
                    vec3 color = mix(color1, color2, fresnel * pulse);
                    float alpha = 0.8 + 0.2 * pulse;
                    
                    gl_FragColor = vec4(color, alpha);
                }
            `,
            transparent: true,
            blending: THREE.AdditiveBlending
        });
        
        this.origin = new THREE.Mesh(originGeometry, originMaterial);
        this.scene.add(this.origin);
        
        // Lumière centrale pulsante
        this.centralLight = new THREE.PointLight(0xffffff, 2, 100);
        this.scene.add(this.centralLight);
    }
    
    createHarmonicLayers() {
        const { harmonicLayers, particlesPerLayer, phi, frequencies, colors, rotationSpeeds } = this.config;
        
        for (let layer = 0; layer < harmonicLayers; layer++) {
            const radius = 3 + layer * 5 * Math.pow(phi, 0.3);
            const frequency = frequencies[layer];
            const color = colors[layer];
            const rotationSpeed = rotationSpeeds[layer];
            
            // Géométrie des particules pour cette couche
            const geometry = new THREE.BufferGeometry();
            const positions = new Float32Array(particlesPerLayer * 3);
            const colors_arr = new Float32Array(particlesPerLayer * 3);
            const sizes = new Float32Array(particlesPerLayer);
            const phases = new Float32Array(particlesPerLayer);
            
            for (let i = 0; i < particlesPerLayer; i++) {
                // Distribution sur une sphère avec motif harmonique
                const u = Math.random();
                const v = Math.random();
                const theta = 2 * Math.PI * u;
                const phi_angle = Math.acos(2 * v - 1);
                
                // Modulation harmonique du rayon
                const harmonicMod = 1 + 0.15 * Math.sin(frequency * theta) * Math.cos(frequency * phi_angle);
                const r = radius * harmonicMod;
                
                positions[i * 3] = r * Math.sin(phi_angle) * Math.cos(theta);
                positions[i * 3 + 1] = r * Math.sin(phi_angle) * Math.sin(theta);
                positions[i * 3 + 2] = r * Math.cos(phi_angle);
                
                // Couleur avec variation
                const brightness = 0.7 + 0.3 * Math.random();
                colors_arr[i * 3] = color.r * brightness;
                colors_arr[i * 3 + 1] = color.g * brightness;
                colors_arr[i * 3 + 2] = color.b * brightness;
                
                sizes[i] = 0.02 + Math.random() * 0.03;
                phases[i] = Math.random() * Math.PI * 2;
            }
            
            geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(colors_arr, 3));
            geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1));
            geometry.setAttribute('phase', new THREE.BufferAttribute(phases, 1));
            
            const material = new THREE.ShaderMaterial({
                uniforms: {
                    time: { value: 0 },
                    breathe: { value: 0 },
                    layerIndex: { value: layer },
                    pixelRatio: { value: window.devicePixelRatio }
                },
                vertexShader: `
                    attribute float size;
                    attribute vec3 color;
                    attribute float phase;
                    
                    varying vec3 vColor;
                    varying float vAlpha;
                    
                    uniform float time;
                    uniform float breathe;
                    uniform float layerIndex;
                    uniform float pixelRatio;
                    
                    void main() {
                        vColor = color;
                        
                        // Respiration cosmique
                        vec3 pos = position * (1.0 + breathe * 0.1 * sin(time + phase));
                        
                        // Ondulation
                        float wave = sin(time * 0.5 + layerIndex + length(position) * 0.1);
                        pos += normalize(position) * wave * 0.3;
                        
                        // Alpha basé sur la phase
                        vAlpha = 0.4 + 0.6 * (0.5 + 0.5 * sin(time * 2.0 + phase));
                        
                        vec4 mvPosition = modelViewMatrix * vec4(pos, 1.0);
                        gl_PointSize = size * pixelRatio * (400.0 / -mvPosition.z);
                        gl_Position = projectionMatrix * mvPosition;
                    }
                `,
                fragmentShader: `
                    varying vec3 vColor;
                    varying float vAlpha;
                    
                    void main() {
                        vec2 center = gl_PointCoord - 0.5;
                        float dist = length(center);
                        
                        // Étoile douce
                        float core = exp(-dist * 8.0);
                        float glow = exp(-dist * 3.0) * 0.5;
                        
                        vec3 finalColor = vColor * (core + glow);
                        float alpha = (core + glow) * vAlpha;
                        
                        gl_FragColor = vec4(finalColor, alpha);
                    }
                `,
                transparent: true,
                blending: THREE.AdditiveBlending,
                depthWrite: false
            });
            
            const points = new THREE.Points(geometry, material);
            
            // Groupe pour la rotation
            const layerGroup = new THREE.Group();
            layerGroup.add(points);
            
            this.layers.push({
                group: layerGroup,
                points: points,
                radius: radius,
                rotationSpeed: rotationSpeed,
                frequency: frequency,
                axis: new THREE.Vector3(
                    Math.sin(layer * 0.9),
                    Math.cos(layer * 0.7),
                    Math.sin(layer * 1.1)
                ).normalize()
            });
            
            this.scene.add(layerGroup);
        }
    }
    
    createCosmicDust() {
        // Poussière cosmique ambiante
        const dustCount = 15000;
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(dustCount * 3);
        const colors = new Float32Array(dustCount * 3);
        const sizes = new Float32Array(dustCount);
        
        for (let i = 0; i < dustCount; i++) {
            // Distribution volumétrique
            const r = 5 + Math.random() * 80;
            const theta = Math.random() * Math.PI * 2;
            const phi = Math.acos(2 * Math.random() - 1);
            
            positions[i * 3] = r * Math.sin(phi) * Math.cos(theta);
            positions[i * 3 + 1] = r * Math.sin(phi) * Math.sin(theta);
            positions[i * 3 + 2] = r * Math.cos(phi);
            
            // Couleur: du blanc chaud au violet froid
            const temp = Math.random();
            const color = new THREE.Color();
            color.setHSL(0.7 - temp * 0.2, 0.3, 0.5 + temp * 0.3);
            
            colors[i * 3] = color.r;
            colors[i * 3 + 1] = color.g;
            colors[i * 3 + 2] = color.b;
            
            sizes[i] = 0.01 + Math.random() * 0.02;
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1));
        
        const material = new THREE.ShaderMaterial({
            uniforms: {
                time: { value: 0 },
                pixelRatio: { value: window.devicePixelRatio }
            },
            vertexShader: `
                attribute float size;
                attribute vec3 color;
                varying vec3 vColor;
                varying float vAlpha;
                uniform float time;
                uniform float pixelRatio;
                
                void main() {
                    vColor = color;
                    
                    // Scintillement
                    float twinkle = 0.3 + 0.7 * abs(sin(time * 3.0 + position.x * 0.5 + position.y * 0.3));
                    vAlpha = twinkle * 0.6;
                    
                    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                    gl_PointSize = size * pixelRatio * twinkle * (300.0 / -mvPosition.z);
                    gl_Position = projectionMatrix * mvPosition;
                }
            `,
            fragmentShader: `
                varying vec3 vColor;
                varying float vAlpha;
                
                void main() {
                    vec2 center = gl_PointCoord - 0.5;
                    float dist = length(center);
                    float alpha = exp(-dist * 6.0) * vAlpha;
                    gl_FragColor = vec4(vColor, alpha);
                }
            `,
            transparent: true,
            blending: THREE.AdditiveBlending,
            depthWrite: false
        });
        
        this.cosmicDust = new THREE.Points(geometry, material);
        this.scene.add(this.cosmicDust);
    }
    
    createFilaments() {
        // Filaments cosmiques - les structures à grande échelle
        const filamentCount = 200;
        const pointsPerFilament = 50;
        
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(filamentCount * pointsPerFilament * 3);
        const colors = new Float32Array(filamentCount * pointsPerFilament * 3);
        
        let idx = 0;
        
        for (let f = 0; f < filamentCount; f++) {
            // Point de départ
            const startR = 10 + Math.random() * 30;
            const startTheta = Math.random() * Math.PI * 2;
            const startPhi = Math.acos(2 * Math.random() - 1);
            
            let x = startR * Math.sin(startPhi) * Math.cos(startTheta);
            let y = startR * Math.sin(startPhi) * Math.sin(startTheta);
            let z = startR * Math.cos(startPhi);
            
            // Direction de croissance
            const growDir = new THREE.Vector3(
                Math.random() - 0.5,
                Math.random() - 0.5,
                Math.random() - 0.5
            ).normalize();
            
            // Couleur du filament
            const hue = 0.5 + Math.random() * 0.3;
            
            for (let p = 0; p < pointsPerFilament; p++) {
                positions[idx * 3] = x;
                positions[idx * 3 + 1] = y;
                positions[idx * 3 + 2] = z;
                
                // Couleur avec fade
                const alpha = 1 - p / pointsPerFilament;
                const color = new THREE.Color();
                color.setHSL(hue, 0.6, 0.3 + alpha * 0.4);
                
                colors[idx * 3] = color.r;
                colors[idx * 3 + 1] = color.g;
                colors[idx * 3 + 2] = color.b;
                
                // Avancer avec turbulence
                const turbulence = 0.5;
                x += growDir.x * 0.8 + (Math.random() - 0.5) * turbulence;
                y += growDir.y * 0.8 + (Math.random() - 0.5) * turbulence;
                z += growDir.z * 0.8 + (Math.random() - 0.5) * turbulence;
                
                idx++;
            }
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        
        const material = new THREE.LineBasicMaterial({
            vertexColors: true,
            transparent: true,
            opacity: 0.15,
            blending: THREE.AdditiveBlending
        });
        
        this.filaments = new THREE.LineSegments(geometry, material);
        this.scene.add(this.filaments);
    }
    
    createAura() {
        // Aura externe - le souffle de l'univers
        const auraGeometry = new THREE.SphereGeometry(60, 64, 64);
        const auraMaterial = new THREE.ShaderMaterial({
            uniforms: {
                time: { value: 0 },
                color1: { value: new THREE.Color(0x0a0015) },
                color2: { value: new THREE.Color(0x1a0040) }
            },
            vertexShader: `
                varying vec3 vNormal;
                varying vec3 vPosition;
                
                void main() {
                    vNormal = normalize(normalMatrix * normal);
                    vPosition = position;
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
                }
            `,
            fragmentShader: `
                uniform float time;
                uniform vec3 color1;
                uniform vec3 color2;
                varying vec3 vNormal;
                varying vec3 vPosition;
                
                void main() {
                    float fresnel = pow(1.0 - abs(dot(vNormal, vec3(0.0, 0.0, 1.0))), 3.0);
                    vec3 color = mix(color1, color2, fresnel);
                    
                    // Pulsation subtile
                    float pulse = 0.5 + 0.5 * sin(time * 0.2 + vPosition.y * 0.1);
                    color *= 0.8 + pulse * 0.2;
                    
                    gl_FragColor = vec4(color, fresnel * 0.3);
                }
            `,
            transparent: true,
            side: THREE.BackSide,
            blending: THREE.AdditiveBlending,
            depthWrite: false
        });
        
        this.aura = new THREE.Mesh(auraGeometry, auraMaterial);
        this.scene.add(this.aura);
    }
    
    update(dt = 0.016) {
        this.time += dt;
        
        const breathe = Math.sin(this.time * this.config.breathingRate);
        
        // Origine pulsante
        if (this.origin) {
            this.origin.material.uniforms.time.value = this.time;
            const scale = 1 + breathe * 0.2;
            this.origin.scale.setScalar(scale);
        }
        
        // Lumière centrale
        if (this.centralLight) {
            this.centralLight.intensity = 1.5 + breathe * 0.5;
        }
        
        // Couches harmoniques
        for (const layer of this.layers) {
            // Rotation différentielle
            layer.group.rotateOnAxis(layer.axis, layer.rotationSpeed * dt);
            
            // Mise à jour des uniforms
            layer.points.material.uniforms.time.value = this.time;
            layer.points.material.uniforms.breathe.value = breathe;
        }
        
        // Poussière cosmique
        if (this.cosmicDust) {
            this.cosmicDust.material.uniforms.time.value = this.time;
            this.cosmicDust.rotation.y += dt * 0.01;
        }
        
        // Filaments
        if (this.filaments) {
            this.filaments.rotation.y += dt * 0.005;
            this.filaments.rotation.x += dt * 0.002;
        }
        
        // Aura
        if (this.aura) {
            this.aura.material.uniforms.time.value = this.time;
        }
    }
    
    dispose() {
        // Nettoyer toutes les ressources
        if (this.origin) {
            this.scene.remove(this.origin);
            this.origin.geometry.dispose();
            this.origin.material.dispose();
        }
        
        for (const layer of this.layers) {
            this.scene.remove(layer.group);
            layer.points.geometry.dispose();
            layer.points.material.dispose();
        }
        
        if (this.cosmicDust) {
            this.scene.remove(this.cosmicDust);
            this.cosmicDust.geometry.dispose();
            this.cosmicDust.material.dispose();
        }
        
        if (this.filaments) {
            this.scene.remove(this.filaments);
            this.filaments.geometry.dispose();
            this.filaments.material.dispose();
        }
        
        if (this.aura) {
            this.scene.remove(this.aura);
            this.aura.geometry.dispose();
            this.aura.material.dispose();
        }
        
        if (this.centralLight) {
            this.scene.remove(this.centralLight);
        }
    }
}
