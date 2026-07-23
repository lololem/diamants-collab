/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Contrôleur ROS 2 pour Crazyflie
 * ==============================================
 * Interface moderne entre frontend et backend ROS 2
 */

export class CrazyflieRosController {
    constructor(config = {}) {
        this.websocket = null;
        this.connected = false;
    this.wsUrl = config.wsUrl || 'ws://localhost:8765';
        this.drones_state = {};
        this.callbacks = {
            onDroneUpdate: [],
            onMapUpdate: [],
            onConnectionChange: []
        };
        this.swarm_intelligence = 0;

        // Options de contrôle du bruit / reconnexion
        this.isActive = config.isActive !== undefined ? config.isActive : true;
        this.maxReconnectDelayMs = config.maxReconnectDelayMs || 30000; // 30s max
        this._reconnectDelayMs = 2000; // backoff starting point
        this._lastWarnTs = 0;
        this._warnEveryMs = 15000; // log au plus toutes les 15s
        this.standaloneMode = false;

        if (this.isActive) {
            this.connect();
        } else {
            // Mode silencieux inactif
            this._quietWarn('ROS controller disabled; running in standalone mode.');
        }
    }
    
    connect() {
        if (!this.isActive) return;
    const wsUrl = this.wsUrl;
        this._quietLog(`🔗 Connexion WebSocket ROS Bridge: ${wsUrl}`);
        
        this.websocket = new WebSocket(wsUrl);
        
        this.websocket.onopen = () => {
            this.connected = true;
            this._reconnectDelayMs = 2000; // reset backoff
            this._quietLog('✅ WebSocket ROS Bridge connecté');
            this.triggerCallbacks('onConnectionChange', { connected: true });
        };
        
        this.websocket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this.handleMessage(data);
            } catch (e) {
                this._quietError('Erreur parsing message WebSocket:', e);
            }
        };
        
        this.websocket.onclose = () => {
            this.connected = false;
            this._quietWarn('❌ WebSocket ROS Bridge déconnecté');
            this.triggerCallbacks('onConnectionChange', { connected: false });
            
            // Reconnexion automatique avec backoff exponentiel
            if (this.isActive) {
                const delay = Math.min(this._reconnectDelayMs, this.maxReconnectDelayMs);
                setTimeout(() => this.connect(), delay);
                this._reconnectDelayMs = Math.min(this._reconnectDelayMs * 2, this.maxReconnectDelayMs);
            }
        };
        
        this.websocket.onerror = (error) => {
            this._quietWarn('⚠️ WebSocket ROS indisponible, mode standalone activé');
            this.connected = false;
            this.standaloneMode = true;
        };
    }
    
    handleMessage(data) {
        switch (data.type) {
            case 'drones_state':
                // Supporter deux formats: {data, swarm_intelligence} ou {drones, swarm_intelligence}
                if (data.data) {
                    this.drones_state = data.data;
                } else if (data.drones) {
                    this.drones_state = data.drones;
                } else {
                    this.drones_state = {};
                }
                if (typeof data.swarm_intelligence === 'number') {
                    this.swarm_intelligence = data.swarm_intelligence;
                }
                this.triggerCallbacks('onDroneUpdate', this.drones_state);
                break;
                
            case 'map_update':
                this.triggerCallbacks('onMapUpdate', data.data);
                break;
        }
    }

    disconnect() {
        // Désactive la reconnexion et ferme proprement la socket
        this.isActive = false;
        this.standaloneMode = true;
        try {
            if (this.websocket) {
                const ws = this.websocket;
                this.websocket = null;
                if (ws.readyState === 0 || ws.readyState === 1) {
                    // CONNECTING or OPEN
                    ws.close();
                }
            }
        } catch (_) { /* noop */ }
        this.connected = false;
        this._quietLog('🔌 ROS controller disconnected.');
    }
    
    // === COMMANDES DRONES ===
    
    takeoff(droneId) {
        this.sendCommand('takeoff', { drone_id: droneId });
        console.log(`🚁 Takeoff ${droneId}`);
    }
    
    land(droneId) {
        this.sendCommand('land', { drone_id: droneId });
        console.log(`🛬 Land ${droneId}`);
    }
    
    move(droneId, velocity) {
        this.sendCommand('move', { 
            drone_id: droneId, 
            velocity: velocity 
        });
    }
    
    sendCommand(command, params = {}) {
        if (!this.connected) {
            this._quietWarn(`WebSocket non connecté, commande ignorée: ${command}`);
            return;
        }
        
        const message = {
            command: command,
            ...params,
            timestamp: Date.now()
        };
        
        this.websocket.send(JSON.stringify(message));
    }
    
    // === CALLBACKS ===
    
    onDroneUpdate(callback) {
        this.callbacks.onDroneUpdate.push(callback);
    }
    
    onMapUpdate(callback) {
        this.callbacks.onMapUpdate.push(callback);
    }
    
    onConnectionChange(callback) {
        this.callbacks.onConnectionChange.push(callback);
    }
    
    triggerCallbacks(event, data) {
        this.callbacks[event].forEach(callback => callback(data));
    }
    
    // === ÉTAT DRONES ===
    
    getDroneState(droneId) {
        return this.drones_state[droneId] || null;
    }
    
    getAllDronesState() {
        return this.drones_state;
    }
    
    getDroneCount() {
        return Object.keys(this.drones_state).length;
    }
    
    getSwarmIntelligence() {
        // Calculé côté backend
        return this.swarm_intelligence || 0;
    }

    // === Helpers silencieux ===
    _quietLog(msg) {
        const now = Date.now();
        if (now - this._lastWarnTs > this._warnEveryMs) {
            console.log(msg);
            this._lastWarnTs = now;
        }
    }

    _quietWarn(msg) {
        const now = Date.now();
        if (now - this._lastWarnTs > this._warnEveryMs) {
            console.warn(msg);
            this._lastWarnTs = now;
        }
    }

    _quietError(msg, err) {
        const now = Date.now();
        if (now - this._lastWarnTs > this._warnEveryMs) {
            console.error(msg, err);
            this._lastWarnTs = now;
        }
    }
}
