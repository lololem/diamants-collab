/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - SITAC Minimap (OAK-D Pro W Detection)
 * 
 * Radar tactique 360 avec detection OAK-D Pro W :
 *   - FOV 127 deg par drone (cone de perception reelle)
 *   - Zones de menace : 2.5m STOP / 5m REPLAN / 10m ALERT / 15m MONITOR
 *   - Obstacles PERSISTANTS : une fois detecte, reste sur la carte
 *   - Arbres = symboles d'arbre reels (tronc + canopee)
 *   - Drones NATO friendly symbols avec heading
 *   - Cercles concentriques de distance
 *
 * Donnees : StigmergyEngine._obstacles[] (tree hitboxes)
 * Refs OAK : oak_perception_v3.py, real_flight.py
 */

// NATO APP-6: BLUE = Friendly
const DRONE_COLORS = [
    '#4488FF', '#66AAFF', '#88BBFF', '#3377EE',
    '#5599FF', '#77AAFF', '#3388FF', '#6699FF',
];

function _droneIdx(id) {
    if (typeof id === 'number') return id;
    const m = String(id).match(/(\d+)/);
    return m ? parseInt(m[1], 10) - 1 : 0;
}

// OAK-D Pro W specs
const OAK_HFOV = 127 * Math.PI / 180;
const OAK_RANGE = 15;
// Threat zones - adjusted for realistic drone operations
const ZONE_STOP = 5.0;
const ZONE_REPLAN = 10.0;
const ZONE_ALERT = 20.0;
const ZONE_MONITOR = 30.0;

export class SitacMinimap {
    constructor(canvasId = 'sitac_canvas') {
        this.canvas = document.getElementById(canvasId);
        this.ctx = this.canvas?.getContext('2d');

        this.config = {
            zoneSize: 120,
            updateInterval: 80,
        };

        this._adaptiveZone = 40;

        /** @type {Map<number, {x:number, y:number, z:number, heading:number, waypoint:{x:number,z:number}|null, timestamp:number}>} */
        this.dronePositions = new Map();

        // === PERSISTENT OBSTACLE MAP ===
        // Key = "x,z" rounded to 0.5m. Once detected, stays forever.
        // Value = {wx, wz, radius, firstSeen, lastSeen, detectionCount, currentThreat}
        /** @type {Map<string, {wx:number, wz:number, radius:number, firstSeen:number, lastSeen:number, detectionCount:number, currentThreat:string, currentDist:number}>} */
        this._knownObstacles = new Map();

        // Current frame transient detection (for threat coloring)
        /** @type {Array<{key:string, wx:number, wz:number, radius:number, minDist:number, threat:string, inFOV:boolean}>} */
        this._currentDetections = [];

        this._snapInterval = 200;
        this._lastSnapTime = 0;
        this._lastRender = 0;

        this._init();
    }

    _init() {
        if (!this.canvas) { console.warn('Canvas SITAC non trouve'); return; }
        this._offscreen = document.createElement('canvas');
        this._offscreen.width = this.canvas.width;
        this._offscreen.height = this.canvas.height;
        this._offCtx = this._offscreen.getContext('2d');
        this.canvas._minimapInstance = this;
        this._startRenderLoop();
        window.DIAMANTS_SITAC = this;
        console.log('SITAC Minimap initialisee (OAK-D Pro W persistent detection)');
    }

    // -- PUBLIC API --
    updateDronePosition(droneId, position, heading = 0, waypoint = null) {
        if (!position) return;
        this.dronePositions.set(droneId, {
            x: position.x, y: position.y, z: position.z,
            heading, waypoint, timestamp: Date.now(),
        });
    }

    // -- DETECTION SYNC (with persistence) --
    _syncDetection() {
        const now = performance.now();
        if (now - this._lastSnapTime < this._snapInterval) return;
        this._lastSnapTime = now;

        const engine = window.DIAMANTS_STIGMERGY_INSTANCE;
        const obstacles = engine?._obstacles || [];
        if (obstacles.length === 0 || this.dronePositions.size === 0) {
            this._currentDetections = [];
            // Update existing known obstacles — mark them as "not currently detected"
            for (const obs of this._knownObstacles.values()) {
                obs.currentThreat = 'mapped';
                obs.currentDist = Infinity;
            }
            return;
        }

        const currentFrame = [];
        const timestamp = Date.now();

        for (const obs of obstacles) {
            let minDist = Infinity;
            let nearestId = 0;
            let inFOV = false;

            for (const [id, pos] of this.dronePositions.entries()) {
                const dx = obs.center.x - pos.x;
                const dz = obs.center.z - pos.z;
                const d = Math.sqrt(dx * dx + dz * dz);

                if (d < OAK_RANGE) {
                    const bearing = Math.atan2(dx, dz);
                    const headingDiff = Math.abs(((bearing - pos.heading + Math.PI * 3) % (Math.PI * 2)) - Math.PI);
                    if (headingDiff < OAK_HFOV / 2) {
                        inFOV = true;
                    }
                }

                if (d < minDist) {
                    minDist = d;
                    nearestId = id;
                }
            }

            // Only detect within extended range (monitor zone)
            if (minDist > ZONE_MONITOR) continue;

            let threat;
            if (minDist < ZONE_STOP) threat = 'critical';
            else if (minDist < ZONE_REPLAN) threat = 'high';
            else if (minDist < ZONE_ALERT) threat = 'medium';
            else threat = 'low';

            // Persistent key (round to 1m grid to avoid duplicates)
            const key = Math.round(obs.center.x) + ',' + Math.round(obs.center.z);

            // Update or create persistent record
            if (this._knownObstacles.has(key)) {
                const known = this._knownObstacles.get(key);
                known.lastSeen = timestamp;
                known.detectionCount++;
                known.currentThreat = threat;
                known.currentDist = minDist;
            } else {
                this._knownObstacles.set(key, {
                    wx: obs.center.x,
                    wz: obs.center.z,
                    radius: obs.radius || 1.5,
                    firstSeen: timestamp,
                    lastSeen: timestamp,
                    detectionCount: 1,
                    currentThreat: threat,
                    currentDist: minDist,
                });
            }

            currentFrame.push({ key, wx: obs.center.x, wz: obs.center.z, radius: obs.radius || 1.5, minDist, threat, inFOV });
        }

        // Update obstacles NOT detected this frame -> "mapped" (gray/dim)
        const currentKeys = new Set(currentFrame.map(d => d.key));
        for (const [key, obs] of this._knownObstacles.entries()) {
            if (!currentKeys.has(key)) {
                obs.currentThreat = 'mapped';
                obs.currentDist = Infinity;
            }
        }

        this._currentDetections = currentFrame;
    }

    // -- HELPERS --
    _getFleetCenter() {
        let sx = 0, sz = 0, n = 0;
        this.dronePositions.forEach(p => { sx += p.x; sz += p.z; n++; });
        return n > 0 ? { x: sx / n, z: sz / n } : { x: 0, z: 0 };
    }

    _getDroneColor(id, alpha) {
        const c = DRONE_COLORS[_droneIdx(id) % DRONE_COLORS.length];
        if (alpha === undefined || alpha >= 1) return c;
        const r = parseInt(c.slice(1, 3), 16);
        const g = parseInt(c.slice(3, 5), 16);
        const b = parseInt(c.slice(5, 7), 16);
        return 'rgba(' + r + ',' + g + ',' + b + ',' + alpha + ')';
    }

    /** Draw a tree icon at pixel position */
    _drawTreeIcon(ctx, px, py, size, threat, confidence) {
        // confidence = 0..1 based on detectionCount
        const alpha = 0.4 + confidence * 0.6;

        // Traffic-light coloring: RED = danger, ORANGE = caution, GREEN = safe
        let trunkColor, canopyColor, outlineColor;
        switch (threat) {
            case 'critical': // RED — immediate danger
                canopyColor = 'rgba(255, 30, 30, ' + alpha + ')';
                outlineColor = 'rgba(255, 255, 255, 0.95)';
                trunkColor = 'rgba(180, 30, 20, ' + alpha + ')';
                break;
            case 'high': // RED-ORANGE — close hazard
                canopyColor = 'rgba(230, 60, 20, ' + alpha + ')';
                outlineColor = 'rgba(255, 100, 50, 0.8)';
                trunkColor = 'rgba(160, 40, 15, ' + alpha + ')';
                break;
            case 'medium': // ORANGE — moderate distance
                canopyColor = 'rgba(240, 160, 30, ' + alpha + ')';
                outlineColor = 'rgba(220, 140, 30, 0.6)';
                trunkColor = 'rgba(150, 90, 20, ' + alpha + ')';
                break;
            case 'low': // GREEN — far, safe
                canopyColor = 'rgba(60, 190, 80, ' + alpha + ')';
                outlineColor = 'rgba(40, 150, 60, 0.5)';
                trunkColor = 'rgba(40, 90, 30, ' + alpha + ')';
                break;
            default: // 'mapped' - detected previously, not in range now — dim green
                canopyColor = 'rgba(50, 120, 60, ' + (alpha * 0.5) + ')';
                outlineColor = 'rgba(40, 80, 50, 0.3)';
                trunkColor = 'rgba(35, 60, 25, ' + (alpha * 0.4) + ')';
                break;
        }

        // Trunk (small rectangle)
        const trunkW = size * 0.25;
        const trunkH = size * 0.5;
        ctx.fillStyle = trunkColor;
        ctx.fillRect(px - trunkW / 2, py, trunkW, trunkH);

        // Canopy (circle/triangle for tree shape)
        ctx.beginPath();
        ctx.arc(px, py - size * 0.1, size * 0.6, 0, Math.PI * 2);
        ctx.fillStyle = canopyColor;
        ctx.fill();
        ctx.strokeStyle = outlineColor;
        ctx.lineWidth = 0.8;
        ctx.stroke();

        // Second smaller canopy layer on top (pine tree effect)
        ctx.beginPath();
        ctx.arc(px, py - size * 0.35, size * 0.35, 0, Math.PI * 2);
        ctx.fillStyle = canopyColor;
        ctx.fill();
    }

    // -- RENDER --
    _startRenderLoop() {
        const loop = () => {
            requestAnimationFrame(loop);
            const now = performance.now();
            if (now - this._lastRender < this.config.updateInterval) return;
            this._lastRender = now;
            this._syncDetection();
            this._render();
        };
        loop();
    }

    _render() {
        if (!this.ctx) return;
        const ctx = this._offCtx || this.ctx;
        const w = this.canvas.width;
        const h = this.canvas.height;
        const cx = w / 2;
        const cy = h / 2;
        const maxR = Math.min(cx, cy) - 14;

        if (window.DIAMANTS_DOCTRINE) {
            const dz = window.DIAMANTS_DOCTRINE.zoneParams;
            this.config.zoneSize = Math.max(dz.sizeX, dz.sizeZ);
        }

        // SITAC adaptive zoom: fit all drones + obstacles with margin
        // Zone boundary is a REFERENCE, not a clipping limit
        const center = this._getFleetCenter();
        const halfZone = this.config.zoneSize / 2;

        // Find max distance from center across all drones and known obstacles
        let maxDist = halfZone; // minimum = half zone always visible
        this.dronePositions.forEach(p => {
            const dx = p.x - center.x;
            const dz = p.z - center.z;
            const d = Math.sqrt(dx * dx + dz * dz);
            if (d > maxDist) maxDist = d;
        });
        for (const obs of this._knownObstacles.values()) {
            const dx = obs.wx - center.x;
            const dz = obs.wz - center.z;
            const d = Math.sqrt(dx * dx + dz * dz);
            if (d > maxDist) maxDist = d;
        }

        // Add 20% margin so elements at the edge are not on the border
        const range = maxDist * 1.2;
        const scale = maxR / range;

        const wToX = (wx) => cx + (wx - center.x) * scale;
        const wToY = (wz) => cy + (wz - center.z) * scale;

        // === BACKGROUND (NATO C2 dark blue-gray) ===
        ctx.fillStyle = '#0a0e18';
        ctx.fillRect(0, 0, w, h);

        // === ZONE PERIMETER (reference, not a clip boundary) ===
        const zoneR = halfZone * scale;
        ctx.beginPath();
        ctx.arc(cx, cy, zoneR, 0, Math.PI * 2);
        ctx.strokeStyle = 'rgba(100, 180, 255, 0.35)';
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        // Zone fill (subtle)
        ctx.beginPath();
        ctx.arc(cx, cy, zoneR, 0, Math.PI * 2);
        ctx.fillStyle = 'rgba(30, 60, 100, 0.08)';
        ctx.fill();

        // === RANGE RINGS ===
        const ringStep = range < 30 ? 5 : range < 60 ? 10 : 20;
        ctx.textAlign = 'center';
        ctx.font = '7px monospace';
        for (let d = ringStep; d <= range; d += ringStep) {
            const r = d * scale;
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.strokeStyle = 'rgba(80, 140, 200, 0.12)';
            ctx.lineWidth = 0.5;
            ctx.stroke();
            ctx.fillStyle = 'rgba(100, 160, 220, 0.35)';
            ctx.fillText(d + 'm', cx + r + 1, cy - 2);
        }

        // === CROSSHAIRS (NATO blue) ===
        ctx.strokeStyle = 'rgba(80, 140, 200, 0.15)';
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.moveTo(cx - maxR, cy); ctx.lineTo(cx + maxR, cy);
        ctx.moveTo(cx, cy - maxR); ctx.lineTo(cx, cy + maxR);
        ctx.stroke();

        // === OAK-D FOV CONES (127 deg) ===
        this.dronePositions.forEach((pos) => {
            const px = wToX(pos.x);
            const py = wToY(pos.z);
            const fovR = OAK_RANGE * scale;
            if (fovR < 3) return;

            const halfFov = OAK_HFOV / 2;
            const startAngle = -pos.heading - halfFov + Math.PI;
            const endAngle = -pos.heading + halfFov + Math.PI;

            // FOV fill
            ctx.beginPath();
            ctx.moveTo(px, py);
            ctx.arc(px, py, fovR, startAngle, endAngle);
            ctx.closePath();
            ctx.fillStyle = 'rgba(70, 130, 200, 0.04)';
            ctx.fill();

            // FOV edges (NATO blue)
            ctx.strokeStyle = 'rgba(80, 160, 255, 0.2)';
            ctx.lineWidth = 0.8;
            ctx.beginPath();
            ctx.moveTo(px, py);
            ctx.lineTo(px + Math.cos(startAngle) * fovR, py + Math.sin(startAngle) * fovR);
            ctx.stroke();
            ctx.beginPath();
            ctx.moveTo(px, py);
            ctx.lineTo(px + Math.cos(endAngle) * fovR, py + Math.sin(endAngle) * fovR);
            ctx.stroke();

            // Arc
            ctx.beginPath();
            ctx.arc(px, py, fovR, startAngle, endAngle);
            ctx.strokeStyle = 'rgba(80, 160, 255, 0.1)';
            ctx.lineWidth = 0.5;
            ctx.stroke();
        });

        // === SWEEP LINE ===
        const sweepAngle = (performance.now() / 3000) * Math.PI * 2;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + Math.cos(sweepAngle) * maxR, cy + Math.sin(sweepAngle) * maxR);
        ctx.strokeStyle = 'rgba(80, 160, 255, 0.06)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // === PERSISTENT OBSTACLES (tree icons) ===
        // First draw "mapped" (not currently detected) -> dim
        // Then draw currently detected -> bright, on top
        const mapped = [];
        const active = [];
        for (const obs of this._knownObstacles.values()) {
            const px = wToX(obs.wx);
            const py = wToY(obs.wz);
            // Skip only if completely off-canvas
            if (px < -20 || px > w + 20 || py < -20 || py > h + 20) continue;

            const confidence = Math.min(1, obs.detectionCount / 10);
            const size = Math.max(4, obs.radius * scale * 0.6);

            if (obs.currentThreat === 'mapped') {
                mapped.push({ px, py, size, threat: obs.currentThreat, confidence, obs });
            } else {
                active.push({ px, py, size, threat: obs.currentThreat, confidence, obs });
            }
        }

        // Draw mapped first (background)
        for (const item of mapped) {
            this._drawTreeIcon(ctx, item.px, item.py, item.size, item.threat, item.confidence);
        }

        // Draw active detections (foreground, with distance labels)
        for (const item of active) {
            this._drawTreeIcon(ctx, item.px, item.py, item.size, item.threat, item.confidence);

            // Distance label for close threats
            if (item.obs.currentDist < ZONE_REPLAN) {
                ctx.fillStyle = '#fff';
                ctx.font = '6px monospace';
                ctx.textAlign = 'center';
                ctx.fillText(item.obs.currentDist.toFixed(1) + 'm', item.px, item.py + item.size + 10);
            }
        }

        // === DRONE POSITIONS ===
        this.dronePositions.forEach((pos, id) => {
            const px = wToX(pos.x);
            const py = wToY(pos.z);
            const color = DRONE_COLORS[_droneIdx(id) % DRONE_COLORS.length];

            ctx.beginPath();
            ctx.arc(px, py, 4, 0, Math.PI * 2);
            ctx.fillStyle = color;
            ctx.fill();
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 0.8;
            ctx.stroke();

            if (pos.heading !== undefined) {
                const ax = px + Math.sin(pos.heading) * 10;
                const ay = py + Math.cos(pos.heading) * 10;
                ctx.beginPath();
                ctx.moveTo(px, py);
                ctx.lineTo(ax, ay);
                ctx.strokeStyle = color;
                ctx.lineWidth = 1.5;
                ctx.stroke();
            }

            ctx.fillStyle = '#ddd';
            ctx.font = 'bold 7px monospace';
            ctx.textAlign = 'center';
            ctx.fillText(id.toString(), px, py - 7);
        });

        // === CARDINALS ===
        ctx.font = 'bold 9px monospace';
        ctx.textAlign = 'center';
        ctx.fillStyle = '#ff5555';
        ctx.fillText('N', cx, 10);
        ctx.beginPath();
        ctx.moveTo(cx, 13); ctx.lineTo(cx - 3, 18); ctx.lineTo(cx + 3, 18);
        ctx.closePath(); ctx.fill();
        ctx.fillStyle = 'rgba(100, 160, 220, 0.6)';
        ctx.fillText('S', cx, h - 3);
        ctx.textAlign = 'right';
        ctx.fillText('E', w - 3, cy + 3);
        ctx.textAlign = 'left';
        ctx.fillText('W', 3, cy + 3);

        // === STATS ===
        const nKnown = this._knownObstacles.size;
        const nActive = this._currentDetections.length;
        const nCrit = this._currentDetections.filter(d => d.threat === 'critical').length;
        const nHigh = this._currentDetections.filter(d => d.threat === 'high').length;

        ctx.textAlign = 'right';
        ctx.font = 'bold 8px monospace';
        let statY = 10;
        // Total known objects
        ctx.fillStyle = 'rgba(140, 180, 220, 0.7)';
        ctx.fillText(nKnown + ' obj', w - 4, statY);
        statY += 10;
        if (nCrit > 0) {
            ctx.fillStyle = '#ff3322';
            ctx.fillText(nCrit + ' CRIT', w - 4, statY);
            statY += 10;
        }
        if (nHigh > 0) {
            ctx.fillStyle = '#ff8822';
            ctx.fillText(nHigh + ' CLOSE', w - 4, statY);
        }

        // Range + NATO SITAC label
        ctx.fillStyle = 'rgba(160, 200, 255, 0.5)';
        ctx.font = '7px monospace';
        ctx.textAlign = 'left';
        ctx.fillText('SITAC', 4, 10);
        ctx.textAlign = 'right';
        ctx.fillText('R:' + Math.round(range) + 'm | Zone:' + Math.round(halfZone) + 'm', w - 4, h - 4);

        // === LEGEND ===
        const ly = h - 10;
        ctx.font = '7px monospace';
        ctx.textAlign = 'left';

        // NATO legend: blue friendly, red hostile, yellow unknown, green neutral
        // Friendly (blue)
        ctx.fillStyle = '#4488FF';
        ctx.beginPath();
        ctx.arc(8, ly, 3, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#88BBFF';
        ctx.fillText('AMI', 14, ly + 3);

        // Hostile (red diamond) — obstacles/dangers
        ctx.fillStyle = '#DD3030';
        ctx.save();
        ctx.translate(46, ly);
        ctx.rotate(Math.PI / 4);
        ctx.fillRect(-3, -3, 6, 6);
        ctx.restore();
        ctx.fillStyle = '#FF6644';
        ctx.fillText('OBS', 53, ly + 3);

        // Unknown (yellow)
        ctx.fillStyle = '#E0C830';
        ctx.beginPath();
        ctx.arc(82, ly, 3, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = '#DDC840';
        ctx.fillText('INC', 88, ly + 3);

        // === BORDER (NATO blue canvas border) ===
        ctx.strokeStyle = '#4477CC';
        ctx.lineWidth = 1.5;
        ctx.strokeRect(1, 1, w - 2, h - 2);

        // Blit
        if (this._offCtx) {
            this.ctx.clearRect(0, 0, w, h);
            this.ctx.drawImage(this._offscreen, 0, 0);
        }
    }
}

export default SitacMinimap;
