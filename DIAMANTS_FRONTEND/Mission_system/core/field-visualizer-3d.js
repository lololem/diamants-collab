/**
 * DIAMANTS 3D Field Visualizer
 * ============================
 * Visualisation volumétrique des champs de l'essaim
 * Utilise InstancedMesh pour performance optimale
 */

import { makeDraggable } from '../ui/panel-utils.js';

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

// ═══════════════════════════════════════════════════════════════════════════
// DIAMANTS Decision-Support HUD
// ═══════════════════════════════════════════════════════════════════════════

/**
 * DIAMANTS Operational HUD — Scientific Swarm Metrics
 *
 * Based on peer-reviewed algorithms:
 *   • Vicsek order parameter φ — collective alignment (Phys. Rev. Lett. 1995)
 *   • Reynolds Boids — separation, alignment, cohesion (SIGGRAPH 1987)
 *   • Digital pheromone stigmergy — σ-field coverage (Aznar, PLoS ONE 2018)
 *   • Emergence detection — Vicsek phase transition above 1/√N baseline
 *
 * PDE harmonics H1-H7, H15 remain available in the raw data panel.
 */

// Which harmonics are actually computed via real PDE integrals
const REAL_HARMONICS = [0, 1, 2, 3, 4, 5, 6, 14]; // H1-H7, H15 (0-indexed)
const STUB_HARMONICS = [7, 8, 9, 10, 11, 12, 13]; // H8-H14

/**
 * Compute honest I(t): only sum REAL harmonics, not stubs
 */
function computeHonestI(H, alpha) {
    return REAL_HARMONICS.reduce((sum, i) => sum + alpha[i] * H[i], 0);
}

/**
 * Trend arrow from rate of change
 */
function trendArrow(dI) {
    if (dI > 1)    return { arrow: '↑', color: '#00ff88', label: 'croissant' };
    if (dI > 0.01) return { arrow: '↗', color: '#88ddaa', label: 'lent +' };
    if (dI < -1)   return { arrow: '↓', color: '#ff6666', label: 'décroissant' };
    if (dI < -0.01) return { arrow: '↘', color: '#ffaa66', label: 'lent −' };
    return            { arrow: '→', color: '#999',    label: 'stable' };
}

/** Color for a [0,1] metric value */
function metricBarColor(v) {
    if (v < 0.2) return '#ff4444';
    if (v < 0.5) return '#ffaa44';
    if (v < 0.8) return '#88ddbb';
    return '#00ff88';
}

/** Format a [0,1] value as percentage string */
function pct(v) {
    if (typeof v !== 'number' || isNaN(v)) return '—';
    return (v * 100).toFixed(1) + '%';
}


export class HarmonicsHUD {
    constructor(formulas, container = document.body) {
        this.formulas = formulas;
        this.container = container;
        this.element = null;
        this._lastUpdate = 0;
        this._expanded = false;
        // History ring buffer for sparkline (last 60 samples ~ 15s at 250ms)
        this._history = [];
        this._historyMax = 60;
        
        this.init();
    }
    
    init() {
        this.element = document.createElement('div');
        this.element.id = 'diamants-harmonics-hud';
        this.element.style.cssText = `
            position: fixed;
            top: 10px;
            right: 10px;
            background: rgba(8, 12, 20, 0.94);
            color: #c0e8d8;
            padding: 0;
            border-radius: 10px;
            font-family: 'JetBrains Mono', 'Fira Code', 'Courier New', monospace;
            font-size: 11px;
            z-index: 5000;
            width: 290px;
            border: 1px solid rgba(0, 255, 136, 0.3);
            box-shadow: 0 0 24px rgba(0, 0, 0, 0.5);
            user-select: none;
        `;
        
        this.element.addEventListener('click', (e) => {
            if (e.target.closest('.hud-toggle')) {
                this._expanded = !this._expanded;
                this._forceRender();
            }
        });

        // ── Persistent drag header ──
        const dragHeader = document.createElement('div');
        dragHeader.style.cssText = 'padding: 3px 12px; cursor: grab; background: rgba(0,255,136,0.06); border-bottom: 1px solid rgba(0,255,136,0.15); font-size: 9px; color: #446; text-align: center; user-select: none; letter-spacing: 2px;';
        dragHeader.textContent = '⋮⋮⋮';
        this.element.appendChild(dragHeader);

        // Content div for dynamic HTML
        this._contentDiv = document.createElement('div');
        this.element.appendChild(this._contentDiv);
        
        this.container.appendChild(this.element);

        // ── Drag support ──
        makeDraggable(this.element, dragHeader);

        console.log('✅ DIAMANTS HUD initialized (scientific mode)');
    }

    _forceRender() {
        this._lastUpdate = 0;
        this.update(performance.now());
    }
    
    update(time) {
        if (time - this._lastUpdate < 250) return;
        this._lastUpdate = time;
        
        const H = this.formulas.harmonics || new Array(15).fill(0);
        const alpha = this.formulas.config?.alpha || Array(15).fill(1).map((_, i) => 1 / (i + 1));
        const sm = this.formulas.swarmMetrics || {};

        // Honest I(t) — only real harmonics
        const I = computeHonestI(H, alpha);

        // Record history
        this._history.push(I);
        if (this._history.length > this._historyMax) this._history.shift();

        // Trend from H7
        const trend = trendArrow(H[6]);

        // Sparkline SVG
        const spark = this._sparkline(this._history, 200, 24);

        const N = sm.activeAgents || 0;

        // ── Header ──
        let html = `
        <div style="padding: 10px 12px 6px; border-bottom: 1px solid rgba(255,255,255,0.08);">
            <div style="display: flex; justify-content: space-between; align-items: center;">
                <span style="color: #fff; font-size: 12px; font-weight: bold; letter-spacing: 1px;">◆ DIAMANTS</span>
                <span style="font-size: 9px; color: #555; border: 1px solid #333; padding: 1px 6px; border-radius: 3px;">${N} agent${N !== 1 ? 's' : ''} actif${N !== 1 ? 's' : ''}</span>
            </div>
        </div>`;

        // ── I(t) with sparkline ──
        html += `
        <div style="padding: 8px 12px;">
            <div style="display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 4px;">
                <span style="color: #aaa; font-size: 10px;">I(t) Intelligence collective</span>
                <span style="color: ${trend.color}; font-size: 11px;">${trend.arrow}</span>
            </div>
            <div style="display: flex; align-items: center; gap: 8px;">
                <span style="font-size: 20px; font-weight: bold; color: #fff; min-width: 70px;">${this._fmt(I)}</span>
                <div style="flex: 1;">${spark}</div>
            </div>
        </div>`;

        // ── Scientific Swarm Metrics ──
        html += `<div style="padding: 0 12px 8px;">`;

        // 1. ALIGNEMENT — Vicsek Order Parameter
        html += this._domainCard('◎ Alignement', 'Vicsek φ (1995)', [
            { label: 'Paramètre d\'ordre φ', value: sm.alignment, format: 'pct' },
            { label: 'Seuil aléatoire 1/√N', value: N >= 2 ? 1 / Math.sqrt(N) : 0, format: 'pct', dim: true },
        ]);

        // 2. FORMATION — Reynolds Boids
        html += this._domainCard('◈ Formation', 'Reynolds (1987)', [
            { label: 'Cohésion (CV⁻¹)', value: sm.cohesion, format: 'pct' },
            { label: 'Séparation (d_min/d₀)', value: sm.separation, format: 'pct' },
        ]);

        // 3. STIGMERGIE — Pheromone field
        html += this._domainCard('⚡ Stigmergie', 'champ σ phéromone', [
            { label: 'Couverture σ-field', value: sm.stigmergyCoverage, format: 'pct' },
            { label: 'Intensité moyenne', value: sm.stigmergyIntensity, format: 'num' },
        ]);

        // 4. ÉMERGENCE — Not yet implemented
        html += this._domainCard('✦ Émergence', 'pas encore implémentée', [
            { label: 'Métrique', value: null, format: 'na' },
            { label: 'Entropie Shannon', value: sm.entropy, format: 'bits' },
        ]);

        html += `</div>`;

        // ── Toggle for raw PDE data ──
        html += `
        <div style="padding: 0 12px 8px; text-align: center;">
            <span class="hud-toggle" style="cursor: pointer; color: #555; font-size: 9px; padding: 2px 10px; border: 1px solid #333; border-radius: 4px;">
                ${this._expanded ? '▲ masquer' : '▼ harmoniques PDE brutes'}
            </span>
        </div>`;

        // ── Expanded: raw harmonics table ──
        if (this._expanded) {
            html += `<div style="padding: 4px 12px 10px; border-top: 1px solid rgba(255,255,255,0.06);">`;
            html += `<table style="width: 100%; border-collapse: collapse; font-size: 10px;">`;
            html += `<tr style="color: #555;"><td style="padding: 2px 0;">ID</td><td>Valeur</td><td style="text-align:right;">Source</td></tr>`;
            for (let i = 0; i < 15; i++) {
                const isReal = REAL_HARMONICS.includes(i);
                const v = H[i];
                const rowColor = isReal ? '#bbb' : '#444';
                const badge = isReal
                    ? '<span style="color:#00aa66;font-size:8px;">PDE</span>'
                    : '<span style="color:#664400;font-size:8px;">stub</span>';
                html += `<tr style="color: ${rowColor}; border-top: 1px solid rgba(255,255,255,0.03);">
                    <td style="padding: 2px 0;">H${i + 1}</td>
                    <td style="font-variant-numeric: tabular-nums;">${this._fmt(v)}</td>
                    <td style="text-align:right;">${badge}</td>
                </tr>`;
            }
            html += `</table>`;
            html += `<div style="color: #444; font-size: 8px; margin-top: 4px; text-align: center;">
                Réf: Vicsek (1995) · Reynolds (1987) · Aznar (2018)
            </div>`;
            html += `</div>`;
        }

        this._contentDiv.innerHTML = html;
    }

    /**
     * Render a domain card with bar visualization
     */
    _domainCard(title, ref, metrics) {
        let html = `<div style="
            background: rgba(255,255,255,0.03);
            border-radius: 6px;
            padding: 6px 8px;
            margin-bottom: 4px;
            border-left: 3px solid rgba(0,255,136,0.3);
        ">`;
        html += `<div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 3px;">
            <span style="color: #889; font-size: 9px; text-transform: uppercase; letter-spacing: 1px;">${title}</span>
            <span style="color: #444; font-size: 8px; font-style: italic;">${ref}</span>
        </div>`;

        for (const m of metrics) {
            const v = m.value || 0;
            let display, barWidth, barColor;

            if (m.format === 'na') {
                display = '— non implémentée';
                barWidth = 0;
                barColor = '#555';
            } else if (m.format === 'pct') {
                display = pct(v);
                barWidth = Math.min(100, v * 100);
                barColor = m.color || metricBarColor(v);
            } else if (m.format === 'bits') {
                display = typeof v === 'number' && !isNaN(v) ? v.toFixed(2) + ' bits' : '—';
                barWidth = Math.min(100, (v / 5) * 100); // 5 bits = max
                barColor = '#88ddbb';
            } else {
                display = this._fmt(v);
                barWidth = Math.min(100, (v / 5) * 100);
                barColor = '#88ddbb';
            }

            const labelColor = m.dim ? '#555' : '#aab';

            html += `<div style="margin-bottom: 2px;">
                <div style="display: flex; justify-content: space-between; align-items: center; height: 16px;">
                    <span style="color: ${labelColor}; font-size: 10px;">${m.label}</span>
                    <span style="color: ${barColor}; font-size: 11px; font-weight: bold; font-variant-numeric: tabular-nums;">${display}</span>
                </div>
                ${m.dim ? '' : `<div style="height: 2px; background: rgba(255,255,255,0.06); border-radius: 1px; margin-top: 1px;">
                    <div style="height: 100%; width: ${barWidth}%; background: ${barColor}; border-radius: 1px; transition: width 0.3s;"></div>
                </div>`}
            </div>`;
        }

        html += `</div>`;
        return html;
    }

    /**
     * SVG sparkline from history array
     */
    _sparkline(data, width, height) {
        if (data.length < 2) return '';
        const min = Math.min(...data);
        const max = Math.max(...data);
        const range = max - min || 1;
        const step = width / (data.length - 1);

        const points = data.map((v, i) => {
            const x = (i * step).toFixed(1);
            const y = (height - 2 - ((v - min) / range) * (height - 4)).toFixed(1);
            return `${x},${y}`;
        }).join(' ');

        return `<svg width="${width}" height="${height}" style="display:block;">
            <polyline points="${points}" fill="none" stroke="rgba(0,255,136,0.5)" stroke-width="1.5" />
            <circle cx="${((data.length - 1) * step).toFixed(1)}" cy="${(height - 2 - ((data[data.length - 1] - min) / range) * (height - 4)).toFixed(1)}" r="2.5" fill="#00ff88" />
        </svg>`;
    }

    /**
     * Smart number formatting
     */
    _fmt(v) {
        if (typeof v !== 'number' || isNaN(v)) return '—';
        const abs = Math.abs(v);
        if (abs === 0) return '0';
        if (abs < 0.001) return v.toExponential(1);
        if (abs < 1) return v.toFixed(3);
        if (abs < 10) return v.toFixed(2);
        if (abs < 100) return v.toFixed(1);
        return v.toFixed(0);
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
