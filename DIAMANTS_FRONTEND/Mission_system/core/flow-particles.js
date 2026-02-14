/**
 * DIAMANTS Flow Particles
 * =======================
 * Particules fluides qui suivent le gradient ∇ψ
 * Visualisation dynamique de la stigmergie en action
 */

export class FlowParticles {
    constructor(scene, diamantFormulas, options = {}) {
        this.scene = scene;
        this.formulas = diamantFormulas;
        
        this.options = {
            particleCount: options.particleCount || 2000,
            particleSize: options.particleSize || 0.15,
            speed: options.speed || 2.0,
            lifetime: options.lifetime || 8.0,  // seconds
            trailLength: options.trailLength || 5,
            colorStart: options.colorStart || 0x00ffaa,  // cyan-vert
            colorEnd: options.colorEnd || 0xff00ff,      // magenta
            domainSize: options.domainSize || { x: 50, y: 10, z: 50 },
            offset: options.offset || { x: -25, y: 0, z: -25 },
            ...options
        };
        
        this.particles = [];
        this.geometry = null;
        this.material = null;
        this.points = null;
        this.isVisible = true;
        this._time = 0;
        
        this.init();
    }
    
    init() {
        const THREE = window.THREE;
        if (!THREE) {
            console.error('THREE.js not loaded for FlowParticles');
            return;
        }
        
        const count = this.options.particleCount;
        
        // Create geometry with positions, colors, sizes
        this.geometry = new THREE.BufferGeometry();
        
        const positions = new Float32Array(count * 3);
        const colors = new Float32Array(count * 3);
        const sizes = new Float32Array(count);
        const alphas = new Float32Array(count);
        
        // Initialize particles
        for (let i = 0; i < count; i++) {
            this.particles.push({
                position: new THREE.Vector3(),
                velocity: new THREE.Vector3(),
                age: Math.random() * this.options.lifetime,
                lifetime: this.options.lifetime * (0.5 + Math.random() * 0.5)
            });
            
            // Random initial positions
            this._respawnParticle(this.particles[i]);
            
            const p = this.particles[i].position;
            positions[i * 3] = p.x;
            positions[i * 3 + 1] = p.y;
            positions[i * 3 + 2] = p.z;
            
            // Initial color
            colors[i * 3] = 0;
            colors[i * 3 + 1] = 1;
            colors[i * 3 + 2] = 0.7;
            
            sizes[i] = this.options.particleSize;
            alphas[i] = 1.0;
        }
        
        this.geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        this.geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        this.geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1));
        
        // Custom shader material for glowing particles
        this.material = new THREE.ShaderMaterial({
            uniforms: {
                uTime: { value: 0 },
                uPixelRatio: { value: Math.min(window.devicePixelRatio, 2) }
            },
            vertexShader: `
                attribute float size;
                attribute vec3 color;
                varying vec3 vColor;
                varying float vAlpha;
                uniform float uTime;
                uniform float uPixelRatio;
                
                void main() {
                    vColor = color;
                    
                    vec4 mvPosition = modelViewMatrix * vec4(position, 1.0);
                    gl_PointSize = size * uPixelRatio * (300.0 / -mvPosition.z);
                    gl_PointSize = clamp(gl_PointSize, 1.0, 50.0);
                    gl_Position = projectionMatrix * mvPosition;
                    
                    // Fade based on distance
                    vAlpha = smoothstep(100.0, 20.0, -mvPosition.z);
                }
            `,
            fragmentShader: `
                varying vec3 vColor;
                varying float vAlpha;
                
                void main() {
                    // Circular particle with soft edge
                    vec2 center = gl_PointCoord - 0.5;
                    float dist = length(center);
                    float alpha = 1.0 - smoothstep(0.3, 0.5, dist);
                    
                    // Glow effect
                    vec3 glow = vColor * (1.0 + 0.5 * (1.0 - dist * 2.0));
                    
                    gl_FragColor = vec4(glow, alpha * vAlpha * 0.8);
                }
            `,
            transparent: true,
            depthWrite: false,
            blending: THREE.AdditiveBlending
        });
        
        this.points = new THREE.Points(this.geometry, this.material);
        this.points.name = 'DIAMANTS_FlowParticles';
        this.points.frustumCulled = false;
        
        this.scene.add(this.points);
        
        console.log('✅ DIAMANTS FlowParticles initialized:', count, 'particles');
    }
    
    _respawnParticle(particle) {
        const { domainSize, offset } = this.options;
        
        // Spawn in random position within domain
        particle.position.set(
            Math.random() * domainSize.x + offset.x,
            Math.random() * domainSize.y + offset.y,
            Math.random() * domainSize.z + offset.z
        );
        
        particle.velocity.set(0, 0, 0);
        particle.age = 0;
        particle.lifetime = this.options.lifetime * (0.5 + Math.random() * 0.5);
    }
    
    _sampleGradient(x, y, z) {
        const { domainSize, offset } = this.options;
        const resolution = this.formulas?.config?.resolution || 2.0;
        
        // Convert world coords to grid indices
        const ix = Math.floor((x - offset.x) / resolution);
        const iy = Math.floor((z - offset.z) / resolution);  // Z in world = Y in grid
        const iz = Math.floor((y - offset.y) / resolution);  // Y in world = Z in grid
        
        const grad = this.formulas?.gradient_field;
        if (!grad || !grad.x) return { x: 0, y: 0, z: 0 };
        
        // Bounds check
        if (ix < 0 || ix >= (grad.x.length || 0)) return { x: 0, y: 0, z: 0 };
        if (!grad.x[ix] || iy < 0 || iy >= grad.x[ix].length) return { x: 0, y: 0, z: 0 };
        if (!grad.x[ix][iy] || iz < 0 || iz >= grad.x[ix][iy].length) return { x: 0, y: 0, z: 0 };
        
        return {
            x: grad.x[ix]?.[iy]?.[iz] || 0,
            y: grad.z[ix]?.[iy]?.[iz] || 0,  // Z gradient -> Y velocity
            z: grad.y[ix]?.[iy]?.[iz] || 0   // Y gradient -> Z velocity
        };
    }
    
    _sampleFieldValue(x, y, z) {
        const { offset } = this.options;
        const resolution = this.formulas?.config?.resolution || 2.0;
        
        const ix = Math.floor((x - offset.x) / resolution);
        const iy = Math.floor((z - offset.z) / resolution);
        const iz = Math.floor((y - offset.y) / resolution);
        
        const field = this.formulas?.psi_field;
        if (!field || !field[ix]?.[iy]?.[iz]) return 0;
        
        return field[ix][iy][iz];
    }
    
    update(deltaTime) {
        if (!this.isVisible || !this.points) return;
        
        this._time += deltaTime;
        this.material.uniforms.uTime.value = this._time;
        
        const positions = this.geometry.attributes.position.array;
        const colors = this.geometry.attributes.color.array;
        const { speed, lifetime, domainSize, offset } = this.options;
        
        // Color interpolation
        const startColor = new THREE.Color(this.options.colorStart);
        const endColor = new THREE.Color(this.options.colorEnd);
        
        for (let i = 0; i < this.particles.length; i++) {
            const particle = this.particles[i];
            
            // Age particle
            particle.age += deltaTime;
            
            // Respawn if too old or out of bounds
            if (particle.age > particle.lifetime ||
                particle.position.x < offset.x - 5 || particle.position.x > offset.x + domainSize.x + 5 ||
                particle.position.y < offset.y - 5 || particle.position.y > offset.y + domainSize.y + 5 ||
                particle.position.z < offset.z - 5 || particle.position.z > offset.z + domainSize.z + 5) {
                this._respawnParticle(particle);
            }
            
            // Sample gradient at particle position
            const grad = this._sampleGradient(
                particle.position.x,
                particle.position.y,
                particle.position.z
            );
            
            // Sample field value for color
            const fieldVal = this._sampleFieldValue(
                particle.position.x,
                particle.position.y,
                particle.position.z
            );
            
            // Update velocity (follow negative gradient = flow toward minima)
            // With some inertia for smooth motion
            const inertia = 0.9;
            const gradScale = speed * 0.5;
            particle.velocity.x = particle.velocity.x * inertia - grad.x * gradScale;
            particle.velocity.y = particle.velocity.y * inertia - grad.y * gradScale * 0.3;  // Less vertical
            particle.velocity.z = particle.velocity.z * inertia - grad.z * gradScale;
            
            // Add slight upward drift + turbulence
            particle.velocity.y += 0.02;
            particle.velocity.x += (Math.random() - 0.5) * 0.1;
            particle.velocity.z += (Math.random() - 0.5) * 0.1;
            
            // Clamp velocity
            const maxVel = speed * 2;
            const vel = Math.sqrt(
                particle.velocity.x ** 2 + 
                particle.velocity.y ** 2 + 
                particle.velocity.z ** 2
            );
            if (vel > maxVel) {
                const scale = maxVel / vel;
                particle.velocity.x *= scale;
                particle.velocity.y *= scale;
                particle.velocity.z *= scale;
            }
            
            // Update position
            particle.position.x += particle.velocity.x * deltaTime;
            particle.position.y += particle.velocity.y * deltaTime;
            particle.position.z += particle.velocity.z * deltaTime;
            
            // Update buffer
            positions[i * 3] = particle.position.x;
            positions[i * 3 + 1] = particle.position.y;
            positions[i * 3 + 2] = particle.position.z;
            
            // Color based on field value and age
            const t = Math.min(1, fieldVal / 3);  // Normalize field value
            const ageRatio = particle.age / particle.lifetime;
            const color = startColor.clone().lerp(endColor, t);
            
            // Fade out near end of life
            const fade = ageRatio < 0.8 ? 1.0 : (1.0 - ageRatio) / 0.2;
            
            colors[i * 3] = color.r * fade;
            colors[i * 3 + 1] = color.g * fade;
            colors[i * 3 + 2] = color.b * fade;
        }
        
        this.geometry.attributes.position.needsUpdate = true;
        this.geometry.attributes.color.needsUpdate = true;
    }
    
    setVisible(visible) {
        this.isVisible = visible;
        if (this.points) this.points.visible = visible;
    }
    
    setSpeed(speed) {
        this.options.speed = speed;
    }
    
    setParticleCount(count) {
        // Would need to recreate geometry - simplified for now
        console.log('Particle count change requires restart');
    }
    
    dispose() {
        if (this.points) {
            this.scene.remove(this.points);
            this.geometry.dispose();
            this.material.dispose();
        }
    }
}

export default FlowParticles;
