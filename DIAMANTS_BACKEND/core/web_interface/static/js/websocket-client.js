/*
 * DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
 * 
 * Copyright (c) 2025 DIAMANTS Project Contributors
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * DIAMANTS V3 WebSocket Client
 * ===========================
 * Client WebSocket pour communication temps réel avec ROS2
 */

class DiamantWebSocketClient {
    constructor(host = null, port = null) {
        // Détection automatique hôte/port/protocole depuis la page actuelle
        const isBrowser = typeof window !== 'undefined' && window.location;
        const defaultHost = isBrowser ? window.location.hostname : 'localhost';
        const defaultPort = isBrowser
            ? (window.location.port || (window.location.protocol === 'https:' ? '443' : '80'))
            : '8080';

        this.host = host || defaultHost;
        this.port = String(port || defaultPort);
        this.ws = null;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 10;
        this.reconnectDelay = 1000;
        this.isConnected = false;

        // Event listeners
        this.messageHandlers = new Map();
        this.eventHandlers = {
            open: [],
            close: [],
            error: []
        };

        // Data cache
        this.lastData = {
            dronePositions: {},
            intelligenceScore: 0.0,
            coverageArea: 0.0,
            statusReport: {}
        };

        this.connect();
        this.setupHeartbeat();
    }

    /**
     * Établir connexion WebSocket
     */
    connect() {
        const isBrowser = typeof window !== 'undefined' && window.location;
        const protocol = isBrowser && window.location.protocol === 'https:' ? 'wss' : 'ws';
        const host = this.host;
        const port = this.port;
        const wsUrl = `${protocol}://${host}:${port}/ws`;

        try {
            this.ws = new WebSocket(wsUrl);
            this.setupEventHandlers();

            console.log(`🔗 Tentative connexion WebSocket: ${wsUrl}`);

        } catch (error) {
            console.error('❌ Erreur connexion WebSocket:', error);
            this.scheduleReconnect();
        }
    }

    /**
     * Configuration gestionnaires événements WebSocket
     */
    setupEventHandlers() {
        this.ws.onopen = (event) => {
            console.log('✅ WebSocket connecté');
            this.isConnected = true;
            this.reconnectAttempts = 0;

            // Mettre à jour indicateur statut
            this.updateConnectionStatus(true);

            // Demander état initial
            this.send({
                type: 'get_status'
            });

            // Déclencher callbacks onopen
            this.eventHandlers.open.forEach(handler => handler(event));
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this.handleMessage(data);

            } catch (error) {
                console.warn('⚠️ Message WebSocket invalide:', event.data);
            }
        };

        this.ws.onclose = (event) => {
            console.log('🔌 WebSocket fermé:', event.code, event.reason);
            this.isConnected = false;
            this.updateConnectionStatus(false);

            // Déclencher callbacks onclose
            this.eventHandlers.close.forEach(handler => handler(event));

            // Tentative reconnexion si non intentionnelle
            if (event.code !== 1000) {
                this.scheduleReconnect();
            }
        };

        this.ws.onerror = (error) => {
            console.error('❌ Erreur WebSocket:', error);

            // Déclencher callbacks onerror
            this.eventHandlers.error.forEach(handler => handler(error));
        };
    }

    /**
     * Traiter message reçu
     */
    handleMessage(data) {
        const { type, data: messageData } = data;

        // Utilitaire pour émettre des événements vers les handlers enregistrés
        const emit = (evtType, payload, raw = data) => {
            if (this.messageHandlers.has(evtType)) {
                this.messageHandlers.get(evtType).forEach(handler => {
                    try { handler(payload, raw); } catch (error) {
                        console.error(`Erreur handler ${evtType}:`, error);
                    }
                });
            }
        };

        // Mettre à jour cache selon type
        switch (type) {
            case 'initial_state':
                this.lastData = { ...this.lastData, ...messageData };
                break;

            case 'initial_data':
            case 'data_update': {
                // messageData correspond au data_cache complet côté serveur
                const cache = messageData || {};
                const positions = cache.drone_positions || {};

                // Transformer en format attendu par le front { id: {drone_id, position} }
                const normalized = {};
                Object.entries(positions).forEach(([id, pos]) => {
                    normalized[id] = { drone_id: id, position: { x: pos.x ?? 0, y: pos.y ?? 0, z: pos.z ?? 0 } };
                });

                this.lastData.dronePositions = normalized;
                this.lastData.intelligenceScore = cache.intelligence_score ?? this.lastData.intelligenceScore;
                this.lastData.coverageArea = cache.coverage_area ?? this.lastData.coverageArea;
                this.lastData.statusReport = { mission_status: cache.mission_status, gazebo_status: cache.gazebo_status, rviz_status: cache.rviz_status };

                // Émettre événements dérivés pour compat front existant
                emit('drone_positions', normalized);
                emit('intelligence_score', { score: this.lastData.intelligenceScore });
                emit('coverage_area', { area: this.lastData.coverageArea });
                emit('status_report', this.lastData.statusReport);
                break;
            }

            case 'drone_position':
                if (messageData?.drone_id) {
                    this.lastData.dronePositions[messageData.drone_id] = messageData;
                    emit('drone_position', messageData);
                }
                break;

            case 'intelligence_score':
                this.lastData.intelligenceScore = messageData.score;
                break;

            case 'coverage_area':
                this.lastData.coverageArea = messageData.area;
                break;

            case 'status_report':
                this.lastData.statusReport = messageData;
                break;

            case 'pong':
                // Heartbeat response
                return;
        }

        // Handler générique pour tous messages
        if (this.messageHandlers.has('*')) {
            this.messageHandlers.get('*').forEach(handler => {
                try { handler(data); } catch (error) {
                    console.error('Erreur handler générique:', error);
                }
            });
        }

        console.log(`📨 Message reçu [${type}]:`, messageData);
    }

    /**
     * Envoyer message
     */
    send(message) {
        if (!this.isConnected || this.ws.readyState !== WebSocket.OPEN) {
            console.warn('⚠️ WebSocket non connecté, message mis en attente');
            return false;
        }

        try {
            const messageStr = JSON.stringify(message);
            this.ws.send(messageStr);
            console.log(`📤 Message envoyé [${message.type}]:`, message);
            return true;

        } catch (error) {
            console.error('❌ Erreur envoi message:', error);
            return false;
        }
    }

    /**
     * Enregistrer handler pour type de message
     */
    onMessage(typeOrHandler, handler) {
        if (typeof typeOrHandler === 'function') {
            // Handler générique
            handler = typeOrHandler;
            typeOrHandler = '*';
        }

        if (!this.messageHandlers.has(typeOrHandler)) {
            this.messageHandlers.set(typeOrHandler, []);
        }

        this.messageHandlers.get(typeOrHandler).push(handler);

        return () => {
            // Fonction de désabonnement
            const handlers = this.messageHandlers.get(typeOrHandler);
            const index = handlers.indexOf(handler);
            if (index > -1) {
                handlers.splice(index, 1);
            }
        };
    }

    /**
     * Enregistrer handler pour événements connexion
     */
    onConnection(event, handler) {
        if (this.eventHandlers[event]) {
            this.eventHandlers[event].push(handler);
        }
    }

    /**
     * Commandes essaim
     */
    sendSwarmCommand(command, params = {}) {
        return this.send({
            type: 'swarm_command',
            data: {
                command,
                params,
                timestamp: Date.now()
            }
        });
    }

    /**
     * Commandes drone individuel
     */
    sendDroneCommand(droneId, commandType, data = {}) {
        return this.send({
            type: 'drone_command',
            data: {
                drone_id: droneId,
                type: commandType,
                ...data,
                timestamp: Date.now()
            }
        });
    }

    /**
     * Changement paramètres
     */
    setParameter(paramName, value) {
        return this.send({
            type: 'parameter_change',
            data: {
                parameter: paramName,
                value: value,
                timestamp: Date.now()
            }
        });
    }

    /**
     * Actions mission
     */
    startMission(missionType = 'exploration') {
        return this.sendSwarmCommand('start_mission', { mission_type: missionType });
    }

    pauseMission() {
        return this.sendSwarmCommand('pause_mission');
    }

    stopMission() {
        return this.sendSwarmCommand('stop_mission');
    }

    /**
     * Changement mode exploration
     */
    setExplorationMode(mode) {
        return this.setParameter('exploration_mode', mode);
    }

    /**
     * Vitesse essaim
     */
    setSwarmSpeed(speed) {
        return this.setParameter('swarm_speed', speed);
    }

    /**
     * Formation essaim
     */
    setFormation(formation) {
        return this.sendSwarmCommand('set_formation', { formation });
    }

    /**
     * Planification reconnexion
     */
    scheduleReconnect() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error('❌ Nombre maximum de tentatives de reconnexion atteint');
            return;
        }

        this.reconnectAttempts++;
        const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);

        console.log(`🔄 Reconnexion dans ${delay}ms (tentative ${this.reconnectAttempts}/${this.maxReconnectAttempts})`);

        setTimeout(() => {
            this.connect();
        }, delay);
    }

    /**
     * Mettre à jour indicateur statut connexion
     */
    updateConnectionStatus(connected) {
        const statusDot = document.getElementById('connection-status');
        const statusText = document.getElementById('connection-text');

        if (statusDot) {
            statusDot.className = `status-dot ${connected ? 'online' : 'offline'}`;
        }

        if (statusText) {
            statusText.textContent = connected ? 'Connecté' : 'Déconnecté';
        }
    }

    /**
     * Système heartbeat pour maintenir connexion
     */
    setupHeartbeat() {
        setInterval(() => {
            if (this.isConnected) {
                this.send({ type: 'ping' });
            }
        }, 30000); // Ping toutes les 30 secondes
    }

    /**
     * Fermer connexion proprement
     */
    disconnect() {
        if (this.ws) {
            this.ws.close(1000, 'Disconnection demandée');
            this.ws = null;
        }
        this.isConnected = false;
    }

    /**
     * Obtenir état actuel des données
     */
    getCurrentState() {
        return { ...this.lastData };
    }

    /**
     * Utilitaires pour interface utilisateur
     */

    // Obtenir nombre de drones actifs
    getActiveDroneCount() {
        return Object.keys(this.lastData.dronePositions).length;
    }

    // Obtenir dernière position d'un drone
    getDronePosition(droneId) {
        return this.lastData.dronePositions[droneId]?.position;
    }

    // Obtenir toutes les positions drones
    getAllDronePositions() {
        return Object.values(this.lastData.dronePositions);
    }

    // Obtenir score intelligence
    getIntelligenceScore() {
        return this.lastData.intelligenceScore;
    }

    // Obtenir couverture zone
    getCoverageArea() {
        return this.lastData.coverageArea;
    }

    // Obtenir rapport statut
    getStatusReport() {
        return this.lastData.statusReport;
    }
}

// Export pour utilisation
window.DiamantWebSocketClient = DiamantWebSocketClient;