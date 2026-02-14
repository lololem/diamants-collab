/**
 * STIGMERGY SWARM - Vraie intelligence collective
 * 
 * Les particules:
 * 1. Déposent des phéromones (traces) dans le champ σ
 * 2. Suivent les gradients des traces des autres
 * 3. Les traces s'évaporent et diffusent
 * 4. Émergence de patterns collectifs
 */

import * as THREE from 'three';

export class StigmergySwarm {
    constructor(scene, options = {}) {
        this.scene = scene;
        
        // Configuration
        this.config = {
            particleCount: options.particleCount || 2000,
            fieldSize: options.fieldSize || { x: 50, y: 50, z: 10 },
            resolution: options.resolution || 1.0,
            
            // Stigmergie
            depositRate: options.depositRate || 0.15,      // Taux de dépôt phéromone
            evaporationRate: options.evaporationRate || 0.985, // Évaporation par frame
            diffusionRate: options.diffusionRate || 0.08,  // Diffusion spatiale
            
            // Comportement
            senseDistance: options.senseDistance || 3.0,   // Distance de perception
            senseAngle: options.senseAngle || Math.PI / 4, // Angle de perception
            turnSpeed: options.turnSpeed || 0.15,          // Vitesse de rotation
            moveSpeed: options.moveSpeed || 0.08,          // Vitesse de déplacement
            randomness: options.randomness || 0.02,        // Bruit aléatoire
            
            // Visuel
            trailLength: options.trailLength || 12,
            particleSize: options.particleSize || 0.15,
        };
        
        // Grille de phéromones 3D
        this.gridDims = {
            nx: Math.ceil(this.config.fieldSize.x / this.config.resolution),
            ny: Math.ceil(this.config.fieldSize.y / this.config.resolution),
            nz: Math.ceil(this.config.fieldSize.z / this.config.resolution)
        };
        
        const totalCells = this.gridDims.nx * this.gridDims.ny * this.gridDims.nz;
        this.pheromoneField = new Float32Array(totalCells);
        this.pheromoneFieldNext = new Float32Array(totalCells);
        
        // Particules
        this.particles = [];
        this.initParticles();
        
        // Rendu
        this.initRendering();
        
        // Trails (historique des positions)
        this.trails = [];
        this.initTrails();
        
        console.log(`[STIGMERGY] Swarm initialized: ${this.config.particleCount} agents, ${totalCells} cells`);
    }
    
    initParticles() {
        const { particleCount, fieldSize } = this.config;
        
        for (let i = 0; i < particleCount; i++) {
            this.particles.push({
                // Position
                x: (Math.random() - 0.5) * fieldSize.x,
                y: (Math.random() - 0.5) * fieldSize.y,
                z: (Math.random() - 0.5) * fieldSize.z,
                
                // Direction (angles sphériques)
                theta: Math.random() * Math.PI * 2,  // Azimut
                phi: Math.random() * Math.PI - Math.PI / 2, // Élévation
                
                // État
                energy: 0.5 + Math.random() * 0.5,
                age: 0,
                
                // Couleur basée sur comportement
                hue: Math.random(),
            });
        }
    }
    
    initRendering() {
        const { particleCount, particleSize } = this.config;
        
        // Géométrie des particules
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(particleCount * 3);
        const colors = new Float32Array(particleCount * 3);
        const sizes = new Float32Array(particleCount);
        
        for (let i = 0; i < particleCount; i++) {
            const p = this.particles[i];
            positions[i * 3] = p.x;
            positions[i * 3 + 1] = p.y;
            positions[i * 3 + 2] = p.z;
            
            // Couleur initiale
            const color = new THREE.Color();
            color.setHSL(p.hue, 0.9, 0.6);
            colors[i * 3] = color.r;
            colors[i * 3 + 1] = color.g;
            colors[i * 3 + 2] = color.b;
            
            sizes[i] = particleSize * (0.5 + p.energy);
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1));
        
        // Shader material pour particules fluides
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
                    vAlpha = 0.7 + 0.3 * sin(time * 2.0 + position.x * 0.5);
                    
                    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                    gl_PointSize = size * pixelRatio * (300.0 / -mvPosition.z);
                    gl_Position = projectionMatrix * mvPosition;
                }
            `,
            fragmentShader: `
                varying vec3 vColor;
                varying float vAlpha;
                
                void main() {
                    vec2 center = gl_PointCoord - 0.5;
                    float dist = length(center);
                    
                    // Cercle doux avec glow
                    float alpha = smoothstep(0.5, 0.1, dist) * vAlpha;
                    float glow = exp(-dist * 4.0) * 0.5;
                    
                    vec3 finalColor = vColor + vec3(glow);
                    gl_FragColor = vec4(finalColor, alpha + glow * 0.3);
                }
            `,
            transparent: true,
            blending: THREE.AdditiveBlending,
            depthWrite: false
        });
        
        this.particleSystem = new THREE.Points(geometry, material);
        this.scene.add(this.particleSystem);
    }
    
    initTrails() {
        // Lignes pour les trails
        const trailGeometry = new THREE.BufferGeometry();
        const trailPositions = new Float32Array(this.config.particleCount * this.config.trailLength * 3);
        const trailColors = new Float32Array(this.config.particleCount * this.config.trailLength * 3);
        
        trailGeometry.setAttribute('position', new THREE.BufferAttribute(trailPositions, 3));
        trailGeometry.setAttribute('color', new THREE.BufferAttribute(trailColors, 3));
        
        const trailMaterial = new THREE.LineBasicMaterial({
            vertexColors: true,
            transparent: true,
            opacity: 0.4,
            blending: THREE.AdditiveBlending
        });
        
        // Initialiser les trails
        for (let i = 0; i < this.config.particleCount; i++) {
            const p = this.particles[i];
            this.trails.push({
                positions: Array(this.config.trailLength).fill(null).map(() => ({
                    x: p.x, y: p.y, z: p.z
                })),
                index: 0
            });
        }
        
        this.trailSystem = new THREE.LineSegments(trailGeometry, trailMaterial);
        this.scene.add(this.trailSystem);
    }
    
    // Conversion coordonnées → index grille
    coordToIndex(x, y, z) {
        const { nx, ny, nz } = this.gridDims;
        const { fieldSize, resolution } = this.config;
        
        const ix = Math.floor((x + fieldSize.x / 2) / resolution);
        const iy = Math.floor((y + fieldSize.y / 2) / resolution);
        const iz = Math.floor((z + fieldSize.z / 2) / resolution);
        
        if (ix < 0 || ix >= nx || iy < 0 || iy >= ny || iz < 0 || iz >= nz) {
            return -1;
        }
        
        return ix + iy * nx + iz * nx * ny;
    }
    
    // Échantillonner le champ de phéromones
    samplePheromone(x, y, z) {
        const idx = this.coordToIndex(x, y, z);
        return idx >= 0 ? this.pheromoneField[idx] : 0;
    }
    
    // Déposer une phéromone
    depositPheromone(x, y, z, amount) {
        const idx = this.coordToIndex(x, y, z);
        if (idx >= 0) {
            this.pheromoneField[idx] = Math.min(1.0, this.pheromoneField[idx] + amount);
        }
    }
    
    // Sens: percevoir dans une direction
    sense(particle, angleOffset, elevOffset) {
        const { senseDistance } = this.config;
        
        const theta = particle.theta + angleOffset;
        const phi = particle.phi + elevOffset;
        
        const dx = Math.cos(phi) * Math.cos(theta) * senseDistance;
        const dy = Math.cos(phi) * Math.sin(theta) * senseDistance;
        const dz = Math.sin(phi) * senseDistance;
        
        return this.samplePheromone(
            particle.x + dx,
            particle.y + dy,
            particle.z + dz
        );
    }
    
    // Mettre à jour une particule
    updateParticle(particle, dt) {
        const { senseAngle, turnSpeed, moveSpeed, randomness, fieldSize, depositRate } = this.config;
        
        // === PERCEPTION ===
        // Échantillonner dans plusieurs directions
        const ahead = this.sense(particle, 0, 0);
        const left = this.sense(particle, -senseAngle, 0);
        const right = this.sense(particle, senseAngle, 0);
        const up = this.sense(particle, 0, senseAngle * 0.5);
        const down = this.sense(particle, 0, -senseAngle * 0.5);
        
        // === DÉCISION ===
        // Tourner vers la plus forte concentration
        if (ahead > left && ahead > right) {
            // Continuer tout droit (légère variation)
            particle.theta += (Math.random() - 0.5) * randomness;
        } else if (left > right) {
            particle.theta -= turnSpeed * (1 + left - ahead);
        } else if (right > left) {
            particle.theta += turnSpeed * (1 + right - ahead);
        } else {
            // Exploration aléatoire
            particle.theta += (Math.random() - 0.5) * turnSpeed * 2;
        }
        
        // Élévation
        if (up > down) {
            particle.phi += turnSpeed * 0.5 * (up - ahead);
        } else if (down > up) {
            particle.phi -= turnSpeed * 0.5 * (down - ahead);
        }
        
        // Limiter l'élévation
        particle.phi = Math.max(-Math.PI / 3, Math.min(Math.PI / 3, particle.phi));
        
        // === MOUVEMENT ===
        const speed = moveSpeed * (0.8 + particle.energy * 0.4);
        particle.x += Math.cos(particle.phi) * Math.cos(particle.theta) * speed;
        particle.y += Math.cos(particle.phi) * Math.sin(particle.theta) * speed;
        particle.z += Math.sin(particle.phi) * speed;
        
        // === DÉPÔT ===
        // Déposer une phéromone
        this.depositPheromone(particle.x, particle.y, particle.z, depositRate * particle.energy);
        
        // === REBOND AUX BORDS ===
        const margin = 1.0;
        if (Math.abs(particle.x) > fieldSize.x / 2 - margin) {
            particle.theta = Math.PI - particle.theta;
            particle.x = Math.sign(particle.x) * (fieldSize.x / 2 - margin);
        }
        if (Math.abs(particle.y) > fieldSize.y / 2 - margin) {
            particle.theta = -particle.theta;
            particle.y = Math.sign(particle.y) * (fieldSize.y / 2 - margin);
        }
        if (Math.abs(particle.z) > fieldSize.z / 2 - margin) {
            particle.phi = -particle.phi;
            particle.z = Math.sign(particle.z) * (fieldSize.z / 2 - margin);
        }
        
        // === ÉVOLUTION ===
        particle.age += dt;
        
        // Couleur basée sur l'activité locale
        const localPheromone = this.samplePheromone(particle.x, particle.y, particle.z);
        particle.hue = 0.5 + localPheromone * 0.3; // Cyan → Magenta selon activité
    }
    
    // Diffusion et évaporation du champ
    updatePheromoneField() {
        const { nx, ny, nz } = this.gridDims;
        const { evaporationRate, diffusionRate } = this.config;
        
        // Copier vers buffer suivant avec diffusion
        for (let z = 0; z < nz; z++) {
            for (let y = 0; y < ny; y++) {
                for (let x = 0; x < nx; x++) {
                    const idx = x + y * nx + z * nx * ny;
                    
                    // Évaporation
                    let value = this.pheromoneField[idx] * evaporationRate;
                    
                    // Diffusion (moyenne des voisins)
                    let sum = 0;
                    let count = 0;
                    
                    for (let dz = -1; dz <= 1; dz++) {
                        for (let dy = -1; dy <= 1; dy++) {
                            for (let dx = -1; dx <= 1; dx++) {
                                if (dx === 0 && dy === 0 && dz === 0) continue;
                                
                                const nx2 = x + dx;
                                const ny2 = y + dy;
                                const nz2 = z + dz;
                                
                                if (nx2 >= 0 && nx2 < nx && ny2 >= 0 && ny2 < ny && nz2 >= 0 && nz2 < nz) {
                                    const nidx = nx2 + ny2 * nx + nz2 * nx * ny;
                                    sum += this.pheromoneField[nidx];
                                    count++;
                                }
                            }
                        }
                    }
                    
                    if (count > 0) {
                        const avgNeighbor = sum / count;
                        value = value * (1 - diffusionRate) + avgNeighbor * diffusionRate;
                    }
                    
                    this.pheromoneFieldNext[idx] = value;
                }
            }
        }
        
        // Swap buffers
        [this.pheromoneField, this.pheromoneFieldNext] = [this.pheromoneFieldNext, this.pheromoneField];
    }
    
    // Mettre à jour les trails
    updateTrails() {
        const trailPositions = this.trailSystem.geometry.attributes.position.array;
        const trailColors = this.trailSystem.geometry.attributes.color.array;
        const { trailLength } = this.config;
        
        for (let i = 0; i < this.particles.length; i++) {
            const p = this.particles[i];
            const trail = this.trails[i];
            
            // Ajouter position actuelle
            trail.positions[trail.index] = { x: p.x, y: p.y, z: p.z };
            trail.index = (trail.index + 1) % trailLength;
            
            // Mettre à jour les segments de ligne
            const baseIdx = i * trailLength * 3;
            const color = new THREE.Color();
            
            for (let j = 0; j < trailLength - 1; j++) {
                const curr = trail.positions[(trail.index + j) % trailLength];
                const next = trail.positions[(trail.index + j + 1) % trailLength];
                
                const segIdx = baseIdx + j * 6;
                
                if (curr && next) {
                    trailPositions[segIdx] = curr.x;
                    trailPositions[segIdx + 1] = curr.y;
                    trailPositions[segIdx + 2] = curr.z;
                    trailPositions[segIdx + 3] = next.x;
                    trailPositions[segIdx + 4] = next.y;
                    trailPositions[segIdx + 5] = next.z;
                    
                    // Couleur avec fade
                    const alpha = j / trailLength;
                    color.setHSL(p.hue, 0.8, 0.4 + alpha * 0.3);
                    
                    trailColors[segIdx] = color.r;
                    trailColors[segIdx + 1] = color.g;
                    trailColors[segIdx + 2] = color.b;
                    trailColors[segIdx + 3] = color.r;
                    trailColors[segIdx + 4] = color.g;
                    trailColors[segIdx + 5] = color.b;
                }
            }
        }
        
        this.trailSystem.geometry.attributes.position.needsUpdate = true;
        this.trailSystem.geometry.attributes.color.needsUpdate = true;
    }
    
    // Mettre à jour le rendu des particules
    updateRendering(time) {
        const positions = this.particleSystem.geometry.attributes.position.array;
        const colors = this.particleSystem.geometry.attributes.color.array;
        const sizes = this.particleSystem.geometry.attributes.size.array;
        
        const color = new THREE.Color();
        
        for (let i = 0; i < this.particles.length; i++) {
            const p = this.particles[i];
            
            positions[i * 3] = p.x;
            positions[i * 3 + 1] = p.y;
            positions[i * 3 + 2] = p.z;
            
            color.setHSL(p.hue, 0.9, 0.6);
            colors[i * 3] = color.r;
            colors[i * 3 + 1] = color.g;
            colors[i * 3 + 2] = color.b;
            
            sizes[i] = this.config.particleSize * (0.5 + p.energy);
        }
        
        this.particleSystem.geometry.attributes.position.needsUpdate = true;
        this.particleSystem.geometry.attributes.color.needsUpdate = true;
        this.particleSystem.geometry.attributes.size.needsUpdate = true;
        
        this.particleSystem.material.uniforms.time.value = time;
    }
    
    // Update principal
    update(dt = 0.016) {
        const time = performance.now() * 0.001;
        
        // 1. Mettre à jour le champ de phéromones
        this.updatePheromoneField();
        
        // 2. Mettre à jour chaque particule
        for (const particle of this.particles) {
            this.updateParticle(particle, dt);
        }
        
        // 3. Mettre à jour le rendu
        this.updateRendering(time);
        this.updateTrails();
    }
    
    // Statistiques
    getStats() {
        let totalPheromone = 0;
        let maxPheromone = 0;
        
        for (let i = 0; i < this.pheromoneField.length; i++) {
            totalPheromone += this.pheromoneField[i];
            maxPheromone = Math.max(maxPheromone, this.pheromoneField[i]);
        }
        
        return {
            particleCount: this.particles.length,
            totalPheromone: totalPheromone.toFixed(2),
            maxPheromone: maxPheromone.toFixed(3),
            avgPheromone: (totalPheromone / this.pheromoneField.length).toFixed(4)
        };
    }
    
    // Injecter un attracteur externe
    injectAttractor(x, y, z, strength = 0.5, radius = 5) {
        const { resolution } = this.config;
        const radiusCells = Math.ceil(radius / resolution);
        
        for (let dz = -radiusCells; dz <= radiusCells; dz++) {
            for (let dy = -radiusCells; dy <= radiusCells; dy++) {
                for (let dx = -radiusCells; dx <= radiusCells; dx++) {
                    const dist = Math.sqrt(dx * dx + dy * dy + dz * dz) * resolution;
                    if (dist < radius) {
                        const amount = strength * (1 - dist / radius);
                        this.depositPheromone(
                            x + dx * resolution,
                            y + dy * resolution,
                            z + dz * resolution,
                            amount
                        );
                    }
                }
            }
        }
    }
    
    dispose() {
        this.scene.remove(this.particleSystem);
        this.scene.remove(this.trailSystem);
        this.particleSystem.geometry.dispose();
        this.particleSystem.material.dispose();
        this.trailSystem.geometry.dispose();
        this.trailSystem.material.dispose();
    }
}
