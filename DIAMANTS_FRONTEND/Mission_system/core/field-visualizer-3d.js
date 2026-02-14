/**
 * DIAMANTS 3D Field Visualizer
 * ============================
 * Visualisation volumétrique des champs ψ, ∇ψ et harmoniques
 * Utilise InstancedMesh pour performance optimale
 */

export class FieldVisualizer3D {
    constructor(scene, diamantFormulas, options = {}) {
        this.scene = scene;
        this.formulas = diamantFormulas;
        
        this.options = {
            // Visualisation
            particleSize: options.particleSize || 0.3,
            opacity: options.opacity || 0.6,
            colorScale: options.colorScale || 'viridis',
            threshold: options.threshold || 0.05,  // Min value to display
            maxParticles: options.maxParticles || 5000,
            
            // Update rate
            updateInterval: options.updateInterval || 100,  // ms
            
            // Field offset (center of domain)
            offset: options.offset || { x: -25, y: 0, z: -25 },
            
            // Display modes
            mode: options.mode || 'gradient',  // 'psi', 'gradient', 'phi', 'sigma'
            
            ...options
        };
        
        this.mesh = null;
        this.gradientArrows = null;
        this.isVisible = true;
        this._lastUpdate = 0;
        this._dummy = new THREE.Object3D();
        this._colorCache = new Map();
        
        this.init();
    }
    
    init() {
        const THREE = window.THREE;
        if (!THREE) {
            console.error('THREE.js not loaded');
            return;
        }
        
        // Create instanced mesh for field particles
        const geometry = new THREE.SphereGeometry(this.options.particleSize, 8, 6);
        const material = new THREE.MeshStandardMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: this.options.opacity,
            metalness: 0.2,
            roughness: 0.8
        });
        
        this.mesh = new THREE.InstancedMesh(geometry, material, this.options.maxParticles);
        this.mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
        this.mesh.frustumCulled = false;
        this.mesh.name = 'DIAMANTS_FieldVisualizer';
        
        // Create instanced color buffer
        this.colors = new Float32Array(this.options.maxParticles * 3);
        this.mesh.instanceColor = new THREE.InstancedBufferAttribute(this.colors, 3);
        
        // Add to scene
        this.scene.add(this.mesh);
        
        // Create gradient arrows group
        this.gradientArrows = new THREE.Group();
        this.gradientArrows.name = 'DIAMANTS_GradientArrows';
        this.scene.add(this.gradientArrows);
        
        // Initial arrow pool
        this.arrowPool = [];
        this._createArrowPool(200);
        
        console.log('✅ DIAMANTS Field Visualizer 3D initialized');
    }
    
    _createArrowPool(count) {
        const THREE = window.THREE;
        const arrowMaterial = new THREE.MeshBasicMaterial({ 
            color: 0x00ff88,
            transparent: true,
            opacity: 0.7
        });
        
        for (let i = 0; i < count; i++) {
            // Simple arrow geometry
            const coneGeom = new THREE.ConeGeometry(0.1, 0.4, 6);
            const shaft = new THREE.CylinderGeometry(0.03, 0.03, 0.6, 6);
            
            const cone = new THREE.Mesh(coneGeom, arrowMaterial);
            const shaftMesh = new THREE.Mesh(shaft, arrowMaterial);
            
            const arrow = new THREE.Group();
            cone.position.y = 0.5;
            shaftMesh.position.y = 0.1;
            arrow.add(cone);
            arrow.add(shaftMesh);
            arrow.visible = false;
            
            this.arrowPool.push(arrow);
            this.gradientArrows.add(arrow);
        }
    }
    
    // Viridis-like color mapping
    _valueToColor(value) {
        // Clamp to [0, 1]
        const t = Math.max(0, Math.min(1, value));
        
        // Viridis approximation
        const r = Math.max(0, Math.min(1, -0.027 + t * (1.25 + t * (-4.0 + t * 3.5))));
        const g = Math.max(0, Math.min(1, 0.014 + t * (0.66 + t * (1.2 - t * 1.4))));
        const b = Math.max(0, Math.min(1, 0.33 + t * (1.0 + t * (-2.4 + t * 1.2))));
        
        return { r, g, b };
    }
    
    // Plasma color mapping
    _valueToColorPlasma(value) {
        const t = Math.max(0, Math.min(1, value));
        
        const r = Math.max(0, Math.min(1, 0.05 + t * (1.5 - t * 0.6)));
        const g = Math.max(0, Math.min(1, t * t * 0.8));
        const b = Math.max(0, Math.min(1, 0.53 + t * (0.5 - t * 1.1)));
        
        return { r, g, b };
    }
    
    update(time) {
        if (!this.isVisible || !this.formulas || !this.mesh) return;
        
        // Throttle updates
        if (time - this._lastUpdate < this.options.updateInterval) return;
        this._lastUpdate = time;
        
        const field = this._getActiveField();
        if (!field || !field.length) {
            // Debug: log once if field is empty
            if (!this._debugLogged) {
                console.warn('⚠️ DIAMANTS Visualizer: field empty or undefined');
                this._debugLogged = true;
            }
            return;
        }
        
        const { resolution, domainSize } = this.formulas.config;
        const offset = this.options.offset;
        const threshold = this.options.threshold;
        
        let instanceIndex = 0;
        const maxInstances = this.options.maxParticles;
        
        // Find min/max for normalization (with safety checks)
        let minVal = Infinity, maxVal = -Infinity;
        let validCount = 0;
        for (let ix = 0; ix < field.length; ix++) {
            if (!field[ix]) continue;
            for (let iy = 0; iy < field[ix].length; iy++) {
                if (!field[ix][iy]) continue;
                for (let iz = 0; iz < field[ix][iy].length; iz++) {
                    const val = Math.abs(field[ix][iy][iz] || 0);
                    if (val > threshold) {
                        minVal = Math.min(minVal, val);
                        maxVal = Math.max(maxVal, val);
                        validCount++;
                    }
                }
            }
        }
        
        // Debug: log field stats once
        if (!this._statsLogged && validCount > 0) {
            console.log('✅ DIAMANTS Visualizer: field stats', {
                validCount, minVal: minVal.toFixed(3), maxVal: maxVal.toFixed(3),
                fieldSize: `${field.length}x${field[0]?.length || 0}x${field[0]?.[0]?.length || 0}`
            });
            this._statsLogged = true;
        }
        
        if (maxVal <= minVal) {
            maxVal = minVal + 1;
        }
        
        // Render field particles (with safety checks)
        for (let ix = 0; ix < field.length && instanceIndex < maxInstances; ix++) {
            if (!field[ix]) continue;
            for (let iy = 0; iy < field[ix].length && instanceIndex < maxInstances; iy++) {
                if (!field[ix][iy]) continue;
                for (let iz = 0; iz < field[ix][iy].length && instanceIndex < maxInstances; iz++) {
                    const val = Math.abs(field[ix][iy][iz] || 0);
                    
                    if (val < threshold) continue;
                    
                    // Position
                    const x = ix * resolution + offset.x;
                    const y = iz * resolution + offset.y;  // Z -> Y for Three.js
                    const z = iy * resolution + offset.z;
                    
                    // Normalized value for color
                    const normVal = (val - minVal) / (maxVal - minVal);
                    
                    // Size based on value
                    const scale = 0.5 + normVal * 1.5;
                    
                    // Set transform
                    this._dummy.position.set(x, y, z);
                    this._dummy.scale.setScalar(scale);
                    this._dummy.updateMatrix();
                    this.mesh.setMatrixAt(instanceIndex, this._dummy.matrix);
                    
                    // Set color
                    const color = this.options.colorScale === 'plasma' 
                        ? this._valueToColorPlasma(normVal)
                        : this._valueToColor(normVal);
                    
                    this.colors[instanceIndex * 3] = color.r;
                    this.colors[instanceIndex * 3 + 1] = color.g;
                    this.colors[instanceIndex * 3 + 2] = color.b;
                    
                    instanceIndex++;
                }
            }
        }
        
        // Hide unused instances
        for (let i = instanceIndex; i < maxInstances; i++) {
            this._dummy.position.set(0, -1000, 0);
            this._dummy.scale.setScalar(0.001);
            this._dummy.updateMatrix();
            this.mesh.setMatrixAt(i, this._dummy.matrix);
        }
        
        this.mesh.instanceMatrix.needsUpdate = true;
        this.mesh.instanceColor.needsUpdate = true;
        this.mesh.count = instanceIndex;
        
        // Update gradient arrows if in gradient mode
        if (this.options.mode === 'gradient') {
            this._updateGradientArrows();
        } else {
            this._hideAllArrows();
        }
    }
    
    _getActiveField() {
        switch (this.options.mode) {
            case 'psi':
                return this.formulas.psi_field;
            case 'phi':
                return this.formulas.phi_field;
            case 'sigma':
                return this.formulas.sigma_field;
            case 'gradient':
            default:
                // Use magnitude of gradient
                return this._computeGradientMagnitude();
        }
    }
    
    _computeGradientMagnitude() {
        const grad = this.formulas.gradient_field;
        if (!grad || !grad.x) return this.formulas.psi_field;
        
        const nx = grad.x.length;
        const ny = grad.x[0]?.length || 0;
        const nz = grad.x[0]?.[0]?.length || 0;
        
        const mag = Array(nx).fill().map(() =>
            Array(ny).fill().map(() =>
                Array(nz).fill(0)
            )
        );
        
        for (let ix = 0; ix < nx; ix++) {
            for (let iy = 0; iy < ny; iy++) {
                for (let iz = 0; iz < nz; iz++) {
                    const gx = grad.x[ix]?.[iy]?.[iz] || 0;
                    const gy = grad.y[ix]?.[iy]?.[iz] || 0;
                    const gz = grad.z[ix]?.[iy]?.[iz] || 0;
                    mag[ix][iy][iz] = Math.sqrt(gx*gx + gy*gy + gz*gz);
                }
            }
        }
        
        return mag;
    }
    
    _updateGradientArrows() {
        const grad = this.formulas.gradient_field;
        if (!grad || !grad.x) return;
        
        const { resolution, domainSize } = this.formulas.config;
        const offset = this.options.offset;
        
        // Sample arrows at lower resolution
        const skipFactor = 2;
        let arrowIndex = 0;
        
        const nx = grad.x.length;
        const ny = grad.x[0]?.length || 0;
        const nz = grad.x[0]?.[0]?.length || 0;
        
        for (let ix = 0; ix < nx && arrowIndex < this.arrowPool.length; ix += skipFactor) {
            for (let iy = 0; iy < ny && arrowIndex < this.arrowPool.length; iy += skipFactor) {
                for (let iz = 0; iz < nz && arrowIndex < this.arrowPool.length; iz += skipFactor) {
                    const gx = grad.x[ix]?.[iy]?.[iz] || 0;
                    const gy = grad.y[ix]?.[iy]?.[iz] || 0;
                    const gz = grad.z[ix]?.[iy]?.[iz] || 0;
                    
                    const mag = Math.sqrt(gx*gx + gy*gy + gz*gz);
                    if (mag < 0.1) continue;
                    
                    const arrow = this.arrowPool[arrowIndex];
                    
                    // Position
                    arrow.position.set(
                        ix * resolution + offset.x,
                        iz * resolution + offset.y,
                        iy * resolution + offset.z
                    );
                    
                    // Rotation to point in gradient direction
                    const dir = new THREE.Vector3(gx, gz, gy).normalize();
                    arrow.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir);
                    
                    // Scale by magnitude
                    const scale = Math.min(2, mag * 2);
                    arrow.scale.setScalar(scale);
                    
                    arrow.visible = true;
                    arrowIndex++;
                }
            }
        }
        
        // Hide unused arrows
        for (let i = arrowIndex; i < this.arrowPool.length; i++) {
            this.arrowPool[i].visible = false;
        }
    }
    
    _hideAllArrows() {
        this.arrowPool.forEach(arrow => arrow.visible = false);
    }
    
    setMode(mode) {
        this.options.mode = mode;
        console.log(`🎨 DIAMANTS visualization mode: ${mode}`);
    }
    
    setVisible(visible) {
        this.isVisible = visible;
        if (this.mesh) this.mesh.visible = visible;
        if (this.gradientArrows) this.gradientArrows.visible = visible;
    }
    
    setOpacity(opacity) {
        this.options.opacity = opacity;
        if (this.mesh?.material) {
            this.mesh.material.opacity = opacity;
        }
    }
    
    setThreshold(threshold) {
        this.options.threshold = threshold;
    }
    
    setColorScale(scale) {
        this.options.colorScale = scale;
    }
    
    dispose() {
        if (this.mesh) {
            this.scene.remove(this.mesh);
            this.mesh.geometry.dispose();
            this.mesh.material.dispose();
        }
        
        if (this.gradientArrows) {
            this.scene.remove(this.gradientArrows);
            this.arrowPool.forEach(arrow => {
                arrow.children.forEach(child => {
                    if (child.geometry) child.geometry.dispose();
                    if (child.material) child.material.dispose();
                });
            });
        }
    }
}

// Harmonics HUD display
export class HarmonicsHUD {
    constructor(formulas, container = document.body) {
        this.formulas = formulas;
        this.container = container;
        this.element = null;
        this._lastUpdate = 0;
        
        this.init();
    }
    
    init() {
        this.element = document.createElement('div');
        this.element.id = 'diamants-harmonics-hud';
        this.element.style.cssText = `
            position: fixed;
            top: 10px;
            right: 10px;
            background: rgba(0, 0, 0, 0.85);
            color: #00ff88;
            padding: 15px;
            border-radius: 8px;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            z-index: 10000;
            min-width: 200px;
            border: 1px solid #00ff88;
            box-shadow: 0 0 20px rgba(0, 255, 136, 0.2);
        `;
        
        this.container.appendChild(this.element);
        console.log('✅ DIAMANTS Harmonics HUD initialized');
    }
    
    update(time) {
        if (time - this._lastUpdate < 200) return;
        this._lastUpdate = time;
        
        const H = this.formulas.harmonics || [];
        const I = this.formulas.diamants_value || 0;
        const E = this.formulas.emergence_factor || 0;
        const C = this.formulas.coherence_level || 0;
        
        const formatVal = (v) => typeof v === 'number' ? v.toFixed(3) : '—';
        
        this.element.innerHTML = `
            <div style="text-align: center; margin-bottom: 10px; font-size: 14px; color: #fff;">
                <strong>◆ DIAMANTS ◆</strong>
            </div>
            <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 4px;">
                <span>I(t):</span><span style="text-align: right;">${formatVal(I)}</span>
                <span>Emergence:</span><span style="text-align: right;">${formatVal(E)}</span>
                <span>Cohérence:</span><span style="text-align: right;">${formatVal(C)}</span>
            </div>
            <div style="margin-top: 10px; border-top: 1px solid #00ff88; padding-top: 8px;">
                <div style="color: #888; margin-bottom: 4px;">Harmoniques:</div>
                <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 2px; font-size: 10px;">
                    ${H.slice(0, 15).map((h, i) => 
                        `<span>H${i+1}: ${formatVal(h)}</span>`
                    ).join('')}
                </div>
            </div>
        `;
    }
    
    setVisible(visible) {
        this.element.style.display = visible ? 'block' : 'none';
    }
    
    dispose() {
        if (this.element && this.element.parentNode) {
            this.element.parentNode.removeChild(this.element);
        }
    }
}

export default { FieldVisualizer3D, HarmonicsHUD };
