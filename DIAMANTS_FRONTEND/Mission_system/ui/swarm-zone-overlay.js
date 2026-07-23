/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Swarm Zone 3D Overlay
 * ===================================
 * Affiche les zones de coordination dans la scène 3D:
 *   - Anneau coloré au sol pour chaque coordinateur X500/S500
 *   - Lignes de liaison coordinateur → scouts assignés
 *   - Labels flottants avec ID + rôle
 *   - Mise à jour dynamique via events
 *
 * Les overlays sont ajoutés au scene Three.js et mis à jour chaque frame
 * via la méthode update() appelée dans la boucle animate() de main.js.
 *
 * Écoute:
 *   - diamants:swarm-coordination → zones + scouts
 *   - diamants:drone-positions    → positions temps réel
 */

const log = (...a) => console.log('[ZoneOverlay]', ...a);

export class SwarmZoneOverlay {
    /**
     * @param {THREE.Scene} scene
     */
    constructor(scene) {
        this.scene = scene;
        this._group = new THREE.Group();
        this._group.name = 'swarm-zone-overlay';
        this.scene.add(this._group);

        /** @type {Map<string, {ring: THREE.Mesh, label: THREE.Sprite, lines: THREE.Line[]}>} */
        this._zones = new Map();

        /** @type {Map<string, THREE.Vector3>} */
        this._dronePositions = new Map();

        /** @type {Map<string, {zone: {n:number,e:number}, role: string, scouts: string[], color: string}>} */
        this._coordinatorData = new Map();

        this._visible = false; // caché par défaut — Ctrl+B pour afficher
        this._colors = [0xa78bfa, 0x67e8f9, 0xf59e0b, 0xf472b6, 0x34d399, 0xfb923c];
        this._group.visible = false;

        this._hookEvents();
        log('✅ SwarmZoneOverlay initialized (hidden — Ctrl+B to show)');
    }

    // =========================================================================
    // Events
    // =========================================================================

    _hookEvents() {
        window.addEventListener('diamants:swarm-coordination', (evt) => {
            const d = evt.detail;
            if (d.coordinators) {
                for (const c of d.coordinators) {
                    this._coordinatorData.set(c.id, {
                        zone: c.zone || { n: 0, e: 0 },
                        role: c.role || 'coordinator',
                        scouts: c.scouts || [],
                        color: c.color || '#a78bfa',
                    });
                }
                this._rebuildZones();
            }
        });

        window.addEventListener('diamants:drone-positions', (evt) => {
            const positions = evt.detail;
            if (!positions || typeof positions !== 'object') return;

            for (const [id, data] of Object.entries(positions)) {
                const pos = data.position || data;
                const x = pos.x ?? pos.n ?? 0;
                const y = pos.y ?? 0.5;
                const z = pos.z ?? pos.e ?? 0;
                this._dronePositions.set(id, new THREE.Vector3(x, y, z));
            }
        });

        // Also derive from fleet config at startup
        window.addEventListener('diamants:engine-ready', () => {
            this._deriveFromFleetConfig();
        });

        // Keyboard: Ctrl+B toggles zone visibility
        document.addEventListener('keydown', (e) => {
            if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
            if (e.ctrlKey && e.code === 'KeyB') {
                e.preventDefault();
                this.toggleVisibility();
            }
        });
    }

    _deriveFromFleetConfig() {
        const fleet = window.FLEET_CONFIG;
        if (!fleet?.drones) return;

        const coordDrones = fleet.drones.filter(d => {
            const t = (d.type || '').toLowerCase();
            return t === 'x500' || t === 's500';
        });
        const scoutDrones = fleet.drones.filter(d => {
            const t = (d.type || '').toLowerCase();
            return t !== 'x500' && t !== 's500';
        });

        if (coordDrones.length === 0) return;

        // Partition scouts among coordinators
        // Spread zones across the simulation area (~80m radius terrain)
        const envRadius = 35; // place coordinator zone centers ~35m from origin
        const angleStep = (2 * Math.PI) / coordDrones.length;

        for (let i = 0; i < coordDrones.length; i++) {
            const cd = coordDrones[i];
            const angle = i * angleStep - Math.PI / 2; // start from north
            const cx = envRadius * Math.cos(angle);
            const cz = envRadius * Math.sin(angle);
            const role = (cd.type || '').toLowerCase() === 's500' ? 'patrol' : 'coordinator';

            // Assign scouts
            const assignedScouts = [];
            for (let j = 0; j < scoutDrones.length; j++) {
                if (j % coordDrones.length === i) {
                    assignedScouts.push(scoutDrones[j].id);
                }
            }

            this._coordinatorData.set(cd.id, {
                zone: { n: cx, e: cz },
                role,
                scouts: assignedScouts,
                color: '#' + this._colors[i % this._colors.length].toString(16).padStart(6, '0'),
            });
        }

        this._rebuildZones();
    }

    // =========================================================================
    // 3D Construction
    // =========================================================================

    _rebuildZones() {
        // Clear existing
        this._clearGroup();

        // Compute zone ring radius based on max separation to avoid overlap
        const coords = [...this._coordinatorData.values()];
        let minDist = Infinity;
        for (let i = 0; i < coords.length; i++) {
            for (let j = i + 1; j < coords.length; j++) {
                const dx = coords[i].zone.n - coords[j].zone.n;
                const dz = coords[i].zone.e - coords[j].zone.e;
                const d = Math.sqrt(dx * dx + dz * dz);
                if (d < minDist) minDist = d;
            }
        }
        // Ring radius = 40% of half-distance between closest coordinators (no overlap)
        const ringRadius = Math.min(Math.max(minDist * 0.4, 4), 18);
        const ringWidth = Math.max(ringRadius * 0.06, 0.3);

        let colorIdx = 0;
        for (const [id, data] of this._coordinatorData) {
            const color = this._colors[colorIdx % this._colors.length];
            colorIdx++;

            // Ring on ground — radius proportional to zone spacing
            const ringGeo = new THREE.RingGeometry(ringRadius - ringWidth, ringRadius, 48);
            const ringMat = new THREE.MeshBasicMaterial({
                color,
                transparent: true,
                opacity: 0.3,
                side: THREE.DoubleSide,
                depthWrite: false,
            });
            const ring = new THREE.Mesh(ringGeo, ringMat);
            ring.rotation.x = -Math.PI / 2;
            ring.position.set(data.zone.n, 0.15, data.zone.e);
            this._group.add(ring);

            // Inner fill (very transparent)
            const fillGeo = new THREE.CircleGeometry(ringRadius - ringWidth, 48);
            const fillMat = new THREE.MeshBasicMaterial({
                color,
                transparent: true,
                opacity: 0.04,
                side: THREE.DoubleSide,
                depthWrite: false,
            });
            const fill = new THREE.Mesh(fillGeo, fillMat);
            fill.rotation.x = -Math.PI / 2;
            fill.position.set(data.zone.n, 0.12, data.zone.e);
            this._group.add(fill);

            // Label sprite
            const label = this._makeLabel(
                `${data.role === 'patrol' ? '🛡' : '🎯'} ${id}`,
                color
            );
            label.position.set(data.zone.n, 12, data.zone.e);
            label.scale.set(4, 1.5, 1);
            this._group.add(label);

            // Store references
            this._zones.set(id, { ring, label, lines: [], fill });
        }
    }

    _makeLabel(text, color) {
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 64;
        const ctx = canvas.getContext('2d');

        // Background pill
        ctx.fillStyle = 'rgba(5, 12, 24, 0.85)';
        ctx.beginPath();
        ctx.roundRect(0, 0, 256, 64, 12);
        ctx.fill();

        // Border
        const hexColor = '#' + (color & 0xffffff).toString(16).padStart(6, '0');
        ctx.strokeStyle = hexColor;
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.roundRect(1, 1, 254, 62, 12);
        ctx.stroke();

        // Text
        ctx.fillStyle = '#ffffff';
        ctx.font = 'bold 24px monospace';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(text, 128, 32);

        const texture = new THREE.CanvasTexture(canvas);
        texture.needsUpdate = true;

        const mat = new THREE.SpriteMaterial({
            map: texture,
            transparent: true,
            depthTest: false,
            sizeAttenuation: true,
        });

        return new THREE.Sprite(mat);
    }

    _clearGroup() {
        while (this._group.children.length > 0) {
            const child = this._group.children[0];
            this._group.remove(child);
            if (child.geometry) child.geometry.dispose();
            if (child.material) {
                if (child.material.map) child.material.map.dispose();
                child.material.dispose();
            }
        }
        this._zones.clear();
    }

    // =========================================================================
    // Update (called each frame)
    // =========================================================================

    update() {
        if (!this._visible) return;

        // Update ring positions from live drone positions
        for (const [id, zoneData] of this._zones) {
            const dronePos = this._dronePositions.get(id);
            if (dronePos) {
                // Move label to follow drone (above it)
                zoneData.label.position.set(dronePos.x, dronePos.y + 3.5, dronePos.z);
            }
        }

        // Update scout connection lines
        this._updateScoutLines();
    }

    _updateScoutLines() {
        // Remove old lines
        for (const [, zd] of this._zones) {
            for (const line of zd.lines) {
                this._group.remove(line);
                if (line.geometry) line.geometry.dispose();
                if (line.material) line.material.dispose();
            }
            zd.lines = [];
        }

        // Draw new lines
        for (const [coordId, data] of this._coordinatorData) {
            const coordPos = this._dronePositions.get(coordId);
            if (!coordPos) continue;

            const zoneData = this._zones.get(coordId);
            if (!zoneData) continue;

            const colorIdx = [...this._coordinatorData.keys()].indexOf(coordId);
            const color = this._colors[colorIdx % this._colors.length];

            for (const scoutId of data.scouts) {
                const scoutPos = this._dronePositions.get(scoutId);
                if (!scoutPos) continue;

                const points = [
                    coordPos.clone(),
                    scoutPos.clone(),
                ];
                const geo = new THREE.BufferGeometry().setFromPoints(points);
                const mat = new THREE.LineBasicMaterial({
                    color,
                    transparent: true,
                    opacity: 0.2,
                    linewidth: 1,
                });
                const line = new THREE.Line(geo, mat);
                this._group.add(line);
                zoneData.lines.push(line);
            }
        }
    }

    // =========================================================================
    // Visibility
    // =========================================================================

    toggleVisibility() {
        this._visible = !this._visible;
        this._group.visible = this._visible;
        log(`Zone overlay ${this._visible ? 'visible' : 'hidden'}`);
    }

    show() {
        this._visible = true;
        this._group.visible = true;
    }

    hide() {
        this._visible = false;
        this._group.visible = false;
    }

    dispose() {
        this._clearGroup();
        this.scene.remove(this._group);
    }
}
