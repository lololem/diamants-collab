/**
 * DIAMANTS API - Client WebSocket Unifié Frontend
 * ===============================================
 * 
 * Client WebSocket central pour éliminer duplications frontend
 * 
 * 🎯 Objectif: Un seul client WebSocket pour toute communication Frontend ↔ Backend
 * 🔄 Remplace: websocket-adaptive-manager.js + websocket-client.js + logique éparpillée
 * 
 * @version 1.0.0
 * @author DIAMANTS Team
 */

class DiamantWebSocketClient {
    constructor(options = {}) {
        // Configuration par défaut
        this.config = {
            url: options.url || 'ws://localhost:8765',
            reconnectInterval: options.reconnectInterval || 5000,
            maxReconnectAttempts: options.maxReconnectAttempts || 10,
            pingInterval: options.pingInterval || 30000,
            debug: options.debug || false,
            ...options
        };
        
        // État connexion
        this.ws = null;
        this.isConnected = false;
        this.isReconnecting = false;
        this.reconnectAttempts = 0;
        this.lastMessageTime = 0;
        
        // Gestionnaires événements
        this.eventHandlers = new Map();
        this.messageQueue = [];
        
        // Timers
        this.reconnectTimer = null;
        this.pingTimer = null;
        
        // État système reçu du backend
        this.systemState = {
            drones: {},
            swarm: {},
            simulation: {},
            lastUpdate: 0
        };
        
        this.log('🌐 DiamantWebSocketClient initialisé', this.config);
        
        // Auto-connect si demandé
        if (options.autoConnect !== false) {
            this.connect();
        }
    }
    
    // ============================================================================
    // CONNECTION MANAGEMENT
    // ============================================================================
    
    async connect() {
        if (this.isConnected || this.isReconnecting) {
            this.log('⚠️ Connexion déjà en cours ou active');
            return;
        }
        
        this.log(`🔌 Connexion à ${this.config.url}...`);
        this.isReconnecting = true;
        
        try {
            this.ws = new WebSocket(this.config.url);
            this.setupWebSocketHandlers();
            
            // Promise pour attendre connexion
            await new Promise((resolve, reject) => {
                const timeout = setTimeout(() => {
                    reject(new Error('Timeout connexion WebSocket'));
                }, 5000);
                
                this.ws.onopen = () => {
                    clearTimeout(timeout);
                    resolve();
                };
                
                this.ws.onerror = (error) => {
                    clearTimeout(timeout);
                    reject(error);
                };
            });
            
        } catch (error) {
            this.log('❌ Erreur connexion:', error);
            this.scheduleReconnect();
            throw error;
        }
    }
    
    setupWebSocketHandlers() {
        this.ws.onopen = () => {
            this.log('✅ WebSocket connecté');
            this.isConnected = true;
            this.isReconnecting = false;
            this.reconnectAttempts = 0;
            
            // Démarrer ping automatique
            this.startPingTimer();
            
            // Vider queue messages en attente
            this.flushMessageQueue();
            
            // Événement connexion
            this.emit('connected');
        };
        
        this.ws.onmessage = (event) => {
            this.handleMessage(event.data);
        };
        
        this.ws.onclose = (event) => {
            this.log(`🔌 WebSocket fermé (code: ${event.code})`);
            this.cleanup();
            
            if (!event.wasClean) {
                this.scheduleReconnect();
            }
            
            this.emit('disconnected', { code: event.code, reason: event.reason });
        };
        
        this.ws.onerror = (error) => {
            this.log('❌ Erreur WebSocket:', error);
            this.emit('error', error);
        };
    }
    
    disconnect() {
        this.log('🔌 Déconnexion WebSocket...');
        
        // Nettoyer timers
        this.cleanup();
        
        // Fermer connexion
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.close(1000, 'Déconnexion volontaire');
        }
        
        this.ws = null;
        this.isConnected = false;
        this.isReconnecting = false;
    }
    
    cleanup() {
        this.isConnected = false;
        
        if (this.pingTimer) {
            clearInterval(this.pingTimer);
            this.pingTimer = null;
        }
        
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }
    }
    
    scheduleReconnect() {
        if (this.reconnectAttempts >= this.config.maxReconnectAttempts) {
            this.log('❌ Nombre max tentatives reconnexion atteint');
            this.emit('maxReconnectAttemptsReached');
            return;
        }
        
        this.reconnectAttempts++;
        const delay = this.config.reconnectInterval * Math.min(this.reconnectAttempts, 5);
        
        this.log(`🔄 Reconnexion dans ${delay}ms (tentative ${this.reconnectAttempts})`);
        
        this.reconnectTimer = setTimeout(() => {
            this.connect().catch(error => {
                this.log('❌ Échec reconnexion:', error);
            });
        }, delay);
    }
    
    // ============================================================================
    // MESSAGE HANDLING
    // ============================================================================
    
    handleMessage(messageStr) {
        try {
            const message = JSON.parse(messageStr);
            this.lastMessageTime = Date.now();
            
            this.log('📨 Message reçu:', message.type);
            
            // Router selon type message
            switch (message.type) {
                case 'initial_state':
                    this.handleInitialState(message.data);
                    break;
                    
                case 'drone_positions':
                    this.handleDronePositions(message.data);
                    break;
                    
                case 'swarm_intelligence':
                case 'swarm_coverage':
                    this.handleSwarmUpdate(message);
                    break;
                    
                case 'system_status':
                    this.handleSystemStatus(message.data);
                    break;
                    
                case 'pong':
                    this.log('🏓 Pong reçu');
                    break;
                    
                default:
                    this.log('⚠️ Type message inconnu:', message.type);
            }
            
            // Émettre événement générique
            this.emit('message', message);
            
            // Émettre événement spécifique au type
            this.emit(message.type, message.data, message);
            
        } catch (error) {
            this.log('❌ Erreur parsing message:', error);
        }
    }
    
    handleInitialState(data) {
        this.log('📊 État initial reçu');
        this.systemState = { ...data };
        this.emit('initialState', data);
    }
    
    handleDronePositions(positions) {
        // Mettre à jour état local
        Object.entries(positions).forEach(([droneId, position]) => {
            if (!this.systemState.drones[droneId]) {
                this.systemState.drones[droneId] = {};
            }
            this.systemState.drones[droneId].position = position;
        });
        
        this.emit('dronePositions', positions);
    }
    
    handleSwarmUpdate(message) {
        const { type, data } = message;
        
        if (type === 'swarm_intelligence') {
            this.systemState.swarm.intelligence_score = data.score;
            this.emit('swarmIntelligence', data.score);
        } else if (type === 'swarm_coverage') {
            this.systemState.swarm.coverage_area = data.area;
            this.emit('swarmCoverage', data.area);
        }
    }
    
    handleSystemStatus(data) {
        this.systemState = { ...this.systemState, ...data };
        this.emit('systemStatus', data);
    }
    
    // ============================================================================
    // COMMAND SENDING
    // ============================================================================
    
    send(message) {
        if (!this.isConnected) {
            this.log('⚠️ Non connecté, message mis en queue');
            this.messageQueue.push(message);
            return false;
        }
        
        try {
            const messageStr = typeof message === 'string' ? message : JSON.stringify(message);
            this.ws.send(messageStr);
            this.log('📤 Message envoyé:', message.type || 'raw');
            return true;
        } catch (error) {
            this.log('❌ Erreur envoi message:', error);
            return false;
        }
    }
    
    // Commandes drones
    sendDroneCommand(droneId, action, params = {}) {
        return this.send({
            type: 'drone_command',
            data: {
                drone_id: droneId,
                action: action,
                params: params,
                timestamp: Date.now()
            }
        });
    }
    
    // Commandes essaim
    sendSwarmCommand(action, params = {}) {
        return this.send({
            type: 'swarm_command',
            data: {
                action: action,
                params: params,
                timestamp: Date.now()
            }
        });
    }
    
    // Commandes mission
    sendMissionCommand(action, params = {}) {
        return this.send({
            type: 'mission_command',
            data: {
                action: action,
                params: params,
                timestamp: Date.now()
            }
        });
    }
    
    // Abonnements (future feature)
    subscribe(topics = []) {
        return this.send({
            type: 'subscribe',
            data: { topics }
        });
    }
    
    // Demander état actuel
    requestStatus() {
        return this.send({ type: 'get_status' });
    }
    
    flushMessageQueue() {
        while (this.messageQueue.length > 0) {
            const message = this.messageQueue.shift();
            this.send(message);
        }
    }
    
    // ============================================================================
    // PING/KEEPALIVE
    // ============================================================================
    
    startPingTimer() {
        if (this.pingTimer) {
            clearInterval(this.pingTimer);
        }
        
        this.pingTimer = setInterval(() => {
            this.send({ type: 'ping', timestamp: Date.now() });
        }, this.config.pingInterval);
    }
    
    // ============================================================================
    // EVENT SYSTEM
    // ============================================================================
    
    on(event, handler) {
        if (!this.eventHandlers.has(event)) {
            this.eventHandlers.set(event, new Set());
        }
        this.eventHandlers.get(event).add(handler);
        
        this.log(`📡 Handler ajouté pour événement: ${event}`);
    }
    
    off(event, handler) {
        if (this.eventHandlers.has(event)) {
            this.eventHandlers.get(event).delete(handler);
        }
    }
    
    emit(event, ...args) {
        if (this.eventHandlers.has(event)) {
            this.eventHandlers.get(event).forEach(handler => {
                try {
                    handler(...args);
                } catch (error) {
                    this.log(`❌ Erreur handler ${event}:`, error);
                }
            });
        }
    }
    
    // ============================================================================
    // UTILITIES
    // ============================================================================
    
    getSystemState() {
        return { ...this.systemState };
    }
    
    isWebSocketConnected() {
        return this.isConnected && this.ws && this.ws.readyState === WebSocket.OPEN;
    }
    
    getConnectionInfo() {
        return {
            url: this.config.url,
            connected: this.isConnected,
            reconnecting: this.isReconnecting,
            attempts: this.reconnectAttempts,
            lastMessage: this.lastMessageTime
        };
    }
    
    log(...args) {
        if (this.config.debug) {
            console.log('[DiamantWS]', ...args);
        }
    }
}

// ============================================================================
// HELPER FUNCTIONS & FACTORY
// ============================================================================

/**
 * Factory pour créer client WebSocket avec configuration par défaut
 */
window.createDiamantWebSocket = (options = {}) => {
    return new DiamantWebSocketClient({
        debug: true,
        autoConnect: true,
        ...options
    });
};

/**
 * Instance globale par défaut (optionnel)
 */
window.diamantWS = null;

/**
 * Initialisation automatique si pas en mode test
 */
if (typeof window !== 'undefined' && !window.__DIAMANT_TEST_MODE__) {
    window.diamantWS = window.createDiamantWebSocket();
    
    // Ajouter gestionnaires par défaut pour debug
    window.diamantWS.on('connected', () => {
        console.log('🌟 DIAMANTS WebSocket connecté !');
    });
    
    window.diamantWS.on('dronePositions', (positions) => {
        console.log('🚁 Positions drones mises à jour:', Object.keys(positions));
    });
}

// Export pour modules ES6 si supporté
if (typeof module !== 'undefined' && module.exports) {
    module.exports = DiamantWebSocketClient;
}
