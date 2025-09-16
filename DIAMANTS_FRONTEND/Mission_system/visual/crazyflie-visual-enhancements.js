/**
 * DIAMANTS V3 - Améliorations Visuelles Crazyflie
 * ===============================================
 * Effets visuels et animations pour comportement authentique
 */

// Mode silencieux global
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

export class CrazyflieVisualEnhancements {
    constructor(scene) {
        this.scene = scene;
        this.particleSystems = new Map();
        this.soundEffects = new Map();
        this.visualEffects = new Map();
        
        this.initParticleSystem();
        this.initSoundSystem();
        this.initVisualEffects();
    }
    
    initParticleSystem() {
        if (!window.THREE) return;
        const THREE = window.THREE;
        
        // Système de particules pour les hélices
        const particleGeometry = new THREE.BufferGeometry();
        const particleCount = 100;
        
        const positions = new Float32Array(particleCount * 3);
        const velocities = new Float32Array(particleCount * 3);
        const colors = new Float32Array(particleCount * 3);
        
        for (let i = 0; i < particleCount; i++) {
            positions[i * 3] = 0;
            positions[i * 3 + 1] = 0;
            positions[i * 3 + 2] = 0;
            
            velocities[i * 3] = (Math.random() - 0.5) * 0.02;
            velocities[i * 3 + 1] = -Math.random() * 0.05;
            velocities[i * 3 + 2] = (Math.random() - 0.5) * 0.02;
            
            colors[i * 3] = 0.8;
            colors[i * 3 + 1] = 0.9;
            colors[i * 3 + 2] = 1.0;
        }
        
        particleGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        particleGeometry.setAttribute('velocity', new THREE.BufferAttribute(velocities, 3));
        particleGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        
        const particleMaterial = new THREE.PointsMaterial({
            size: 0.01,
            vertexColors: true,
            transparent: true,
            opacity: 0.6,
            blending: THREE.AdditiveBlending
        });
        
        this.baseParticleSystem = { geometry: particleGeometry, material: particleMaterial };
    }
    
    initSoundSystem() {
        // Simulation sons moteurs (oscillateurs Web Audio)
        try {
            this.audioContext = new (window.AudioContext || window.webkitAudioContext)();
            log('🔊 Système audio initialisé pour moteurs Crazyflie');
        } catch (e) {
            warn('Audio non disponible');
        }
    }
    
    initVisualEffects() {
        if (!window.THREE) return;
        const THREE = window.THREE;
        
        // Shader pour hélices en rotation rapide
        this.propellerShader = {
            vertexShader: `
                varying vec2 vUv;
                varying vec3 vPosition;
                uniform float time;
                uniform float rotationSpeed;
                
                void main() {
                    vUv = uv;
                    vPosition = position;
                    
                    // Rotation hélice
                    float angle = time * rotationSpeed;
                    float c = cos(angle);
                    float s = sin(angle);
                    
                    vec3 pos = position;
                    pos.x = position.x * c - position.y * s;
                    pos.y = position.x * s + position.y * c;
                    
                    gl_Position = projectionMatrix * modelViewMatrix * vec4(pos, 1.0);
                }
            `,
            fragmentShader: `
                varying vec2 vUv;
                uniform float opacity;
                uniform vec3 color;
                
                void main() {
                    float dist = distance(vUv, vec2(0.5));
                    float alpha = opacity * (1.0 - smoothstep(0.4, 0.5, dist));
                    
                    gl_FragColor = vec4(color, alpha);
                }
            `
        };
    }
    
    createPropellerParticles(dronePosition) {
        if (!window.THREE || !this.baseParticleSystem) return null;
        const THREE = window.THREE;
        
        const particles = new THREE.Points(
            this.baseParticleSystem.geometry.clone(),
            this.baseParticleSystem.material.clone()
        );
        
        particles.position.copy(dronePosition);
        this.scene.add(particles);
        
        return particles;
    }
    
    updatePropellerParticles(particles, isFlying) {
        if (!particles || !window.THREE) return;
        
        const positions = particles.geometry.attributes.position.array;
        const velocities = particles.geometry.attributes.velocity.array;
        
        for (let i = 0; i < positions.length; i += 3) {
            if (isFlying) {
                positions[i] += velocities[i];
                positions[i + 1] += velocities[i + 1];
                positions[i + 2] += velocities[i + 2];
                
                // Reset particules qui tombent trop bas
                if (positions[i + 1] < -0.2) {
                    positions[i] = (Math.random() - 0.5) * 0.1;
                    positions[i + 1] = 0.05;
                    positions[i + 2] = (Math.random() - 0.5) * 0.1;
                }
            }
        }
        
        particles.geometry.attributes.position.needsUpdate = true;
    }
    
    createMotorSound(droneId, isFlying) {
        if (!this.audioContext) return;
        
        if (isFlying && !this.soundEffects.has(droneId)) {
            // Oscillateur pour simulation moteur
            const oscillator = this.audioContext.createOscillator();
            const gainNode = this.audioContext.createGain();
            
            oscillator.type = 'sawtooth';
            oscillator.frequency.setValueAtTime(150 + Math.random() * 50, this.audioContext.currentTime);
            gainNode.gain.setValueAtTime(0.1, this.audioContext.currentTime);
            
            oscillator.connect(gainNode);
            gainNode.connect(this.audioContext.destination);
            
            oscillator.start();
            
            this.soundEffects.set(droneId, { oscillator, gainNode });
        } else if (!isFlying && this.soundEffects.has(droneId)) {
            const sound = this.soundEffects.get(droneId);
            sound.gainNode.gain.exponentialRampToValueAtTime(0.01, this.audioContext.currentTime + 0.5);
            sound.oscillator.stop(this.audioContext.currentTime + 0.5);
            this.soundEffects.delete(droneId);
        }
    }
    
    createSwarmVisualization(dronesState) {
        if (!window.THREE) return;
        const THREE = window.THREE;
        
        // Lignes de connexion entre drones
        const connectedDrones = Object.entries(dronesState).filter(([_, state]) => state.connected);
        
        if (connectedDrones.length < 2) return;
        
        // Supprimer anciennes connexions
        const oldConnections = this.scene.children.filter(child => child.userData.isSwarmConnection);
        oldConnections.forEach(connection => this.scene.remove(connection));
        
        // Créer nouvelles connexions
        for (let i = 0; i < connectedDrones.length; i++) {
            for (let j = i + 1; j < connectedDrones.length; j++) {
                const drone1 = connectedDrones[i][1];
                const drone2 = connectedDrones[j][1];
                
                const distance = Math.sqrt(
                    Math.pow(drone1.position.x - drone2.position.x, 2) +
                    Math.pow(drone1.position.y - drone2.position.y, 2) +
                    Math.pow(drone1.position.z - drone2.position.z, 2)
                );
                
                // Ne connecter que les drones proches (< 3m)
                if (distance < 3.0) {
                    const geometry = new THREE.BufferGeometry().setFromPoints([
                        new THREE.Vector3(drone1.position.x, drone1.position.z, drone1.position.y),
                        new THREE.Vector3(drone2.position.x, drone2.position.z, drone2.position.y)
                    ]);
                    
                    const material = new THREE.LineBasicMaterial({
                        color: 0x00ffff,
                        transparent: true,
                        opacity: 0.3
                    });
                    
                    const line = new THREE.Line(geometry, material);
                    line.userData.isSwarmConnection = true;
                    this.scene.add(line);
                }
            }
        }
    }
    
    createIntelligenceVisualization(intelligence) {
        if (!window.THREE || !this.scene) return;
        const THREE = window.THREE;
        
        // Sphère d'intelligence collective
        const radius = Math.max(0.5, Math.min(3.0, intelligence * 0.1));
        const geometry = new THREE.SphereGeometry(radius, 16, 16);
        const material = new THREE.MeshBasicMaterial({
            color: 0xff6b35,
            transparent: true,
            opacity: 0.1,
            wireframe: true
        });
        
        // Supprimer ancienne sphère
        const oldSphere = this.scene.children.find(child => child.userData.isIntelligenceSphere);
        if (oldSphere) this.scene.remove(oldSphere);
        
        const sphere = new THREE.Mesh(geometry, material);
        sphere.position.set(0, 2, 0);
        sphere.userData.isIntelligenceSphere = true;
        
        // Animation pulsation
        sphere.scale.setScalar(1 + Math.sin(Date.now() * 0.005) * 0.2);
        
        this.scene.add(sphere);
    }
    
    updateVisualEnhancements(droneId, droneState) {
        // Mise à jour particules hélices
        if (!this.particleSystems.has(droneId) && droneState.connected) {
            const particles = this.createPropellerParticles(
                new window.THREE.Vector3(
                    droneState.position.x,
                    droneState.position.z,
                    droneState.position.y
                )
            );
            this.particleSystems.set(droneId, particles);
        }
        
        if (this.particleSystems.has(droneId)) {
            this.updatePropellerParticles(
                this.particleSystems.get(droneId),
                droneState.connected
            );
        }
        
        // Mise à jour sons moteurs
        this.createMotorSound(droneId, droneState.connected);
    }
    
    dispose() {
        // Nettoyage ressources
        this.particleSystems.forEach(particles => {
            this.scene.remove(particles);
        });
        
        this.soundEffects.forEach(sound => {
            sound.oscillator.stop();
        });
        
        this.particleSystems.clear();
        this.soundEffects.clear();
    }
}
