/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Fractal Hypervision — orchestrateur L0→L5
 * Descente ordres / remontée état ; couche basse L4-L5 = ROS2/Gazebo.
 */

import { buildFractalSnapshot, buildElementaryFromDrones } from './fractal-layers.js';

export class FractalHypervision {
    constructor(system) {
        this.system = system;
        this.rosState = {
            connected: false,
            mission: {},
            swarm: {},
            system: {},
            elementary: { drones: {} },
        };
        this.snapshot = null;
        this.focusLayer = 'L0';
        this._accum = 0;

        this._onWsConnected = () => { this.rosState.connected = true; };
        this._onWsDisconnected = () => { this.rosState.connected = false; };
        this._onFractalState = (e) => this._mergeBackendFractal(e.detail);
        this._onMissionStatus = (e) => { this.rosState.mission = e.detail || {}; };
        this._onSwarmUpdate = (e) => {
            const d = e.detail || {};
            if (d.type === 'swarm_status') Object.assign(this.rosState.swarm, d);
            else if (d.score != null) this.rosState.swarm.intelligence_score = d.score;
            else if (d.area != null) this.rosState.swarm.coverage_area = d.area;
        };
        this._onSystemStatus = (e) => { this.rosState.system = e.detail || {}; };
    }

    mount() {
        window.addEventListener('diamants:ws-connected', this._onWsConnected);
        window.addEventListener('diamants:ws-disconnected', this._onWsDisconnected);
        window.addEventListener('diamants:fractal-state', this._onFractalState);
        window.addEventListener('diamants:mission-status', this._onMissionStatus);
        window.addEventListener('diamants:swarm-update', this._onSwarmUpdate);
        window.addEventListener('diamants:system-status', this._onSystemStatus);
        this.refresh();
    }

    destroy() {
        window.removeEventListener('diamants:ws-connected', this._onWsConnected);
        window.removeEventListener('diamants:ws-disconnected', this._onWsDisconnected);
        window.removeEventListener('diamants:fractal-state', this._onFractalState);
        window.removeEventListener('diamants:mission-status', this._onMissionStatus);
        window.removeEventListener('diamants:swarm-update', this._onSwarmUpdate);
        window.removeEventListener('diamants:system-status', this._onSystemStatus);
    }

    setFocusLayer(layerId) {
        if (this.snapshot?.layers?.some((l) => l.id === layerId)) {
            this.focusLayer = layerId;
        }
    }

    drillDown() {
        const ids = ['L0', 'L1', 'L2', 'L3', 'L4', 'L5'];
        const i = ids.indexOf(this.focusLayer);
        if (i < ids.length - 1) this.focusLayer = ids[i + 1];
    }

    drillUp() {
        const ids = ['L0', 'L1', 'L2', 'L3', 'L4', 'L5'];
        const i = ids.indexOf(this.focusLayer);
        if (i > 0) this.focusLayer = ids[i - 1];
    }

    _mergeBackendFractal(detail) {
        if (!detail) return;
        if (detail.mission) this.rosState.mission = { ...this.rosState.mission, ...detail.mission };
        if (detail.swarm) this.rosState.swarm = { ...this.rosState.swarm, ...detail.swarm };
        if (detail.elementary) this.rosState.elementary = { ...this.rosState.elementary, ...detail.elementary };
        if (detail.system) this.rosState.system = { ...this.rosState.system, ...detail.system };
    }

    refresh() {
        const drones = this.system?.drones || this.system?.integratedController?.drones || [];
        if (!this.rosState.elementary?.drones || Object.keys(this.rosState.elementary.drones).length === 0) {
            this.rosState.elementary = buildElementaryFromDrones(drones);
        } else {
            const local = buildElementaryFromDrones(drones);
            this.rosState.elementary = {
                ...this.rosState.elementary,
                drones: { ...local.drones, ...this.rosState.elementary.drones },
                gz_sim: local.gz_sim || this.rosState.elementary.gz_sim,
            };
        }
        this.rosState.connected = !!(this.system?.ros?.connected && this.system.ros.ws?.readyState === WebSocket.OPEN);
        this.snapshot = buildFractalSnapshot(this.system, this.rosState);
        window.DIAMANTS = window.DIAMANTS || {};
        window.DIAMANTS.fractal = this.snapshot;
        window.DIAMANTS.fractalFocus = this.focusLayer;
        try {
            window.dispatchEvent(new CustomEvent('diamants:fractal-updated', { detail: this.snapshot }));
        } catch (_) { /* safe */ }
        return this.snapshot;
    }

    tick(deltaSeconds) {
        this._accum += deltaSeconds;
        if (this._accum >= 0.25) {
            this._accum = 0;
            this.refresh();
        }
    }
}
