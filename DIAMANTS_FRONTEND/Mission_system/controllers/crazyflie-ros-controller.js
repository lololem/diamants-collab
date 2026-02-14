/**
 * DIAMANTS — Stub CrazyflieRosController
 * =========================================
 * Network bridge removed — simulation-only build (public vitrine).
 * This stub preserves the public API so other modules can still import it
 * without errors, but all network operations are no-ops.
 */

export class CrazyflieRosController {
    constructor(config = {}) {
        this.connected = false;
        this.isActive = false;
        this.standaloneMode = true;
        this.drones_state = {};
        this.swarm_intelligence = 0;
        this.callbacks = { onDroneUpdate: [], onMapUpdate: [], onConnectionChange: [] };
    }

    connect() { /* no-op */ }
    disconnect() { this.connected = false; }
    takeoff() { /* no-op */ }
    land() { /* no-op */ }
    move() { /* no-op */ }
    sendCommand() { /* no-op */ }
    onDroneUpdate(cb) { this.callbacks.onDroneUpdate.push(cb); }
    onMapUpdate(cb) { this.callbacks.onMapUpdate.push(cb); }
    onConnectionChange(cb) { this.callbacks.onConnectionChange.push(cb); }
    triggerCallbacks() { /* no-op */ }
    getDroneState() { return null; }
    getAllDronesState() { return {}; }
    getDroneCount() { return 0; }
    getSwarmIntelligence() { return 0; }
}
