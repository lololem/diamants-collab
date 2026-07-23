/**
 * agent-communication.js — Learned communication protocol for multi-agent drones.
 *
 * Each agent decides WHAT and WHEN to communicate using its learned policy.
 * Communication flows through the existing CommLink (simulated radio).
 *
 * Message types:
 *   - MAP_SHARE:   Sparse occupancy grid data
 *   - FRONTIER:    Frontier location announcement
 *   - CLAIM:       Task/zone claim
 *   - YIELD:       Release a claimed zone
 *   - VOTE:        Raft consensus messages
 *   - HEARTBEAT:   Alive signal + position
 *
 * Communication budget: each agent can send at most 1 message per comm cycle
 * (configurable interval). This forces agents to LEARN what's worth sharing.
 *
 * @module agent-communication
 */

// ─── MESSAGE TYPES ───────────────────────────────────────────────────

export const CommMessageType = Object.freeze({
    HEARTBEAT:  'heartbeat',
    MAP_SHARE:  'map_share',
    FRONTIER:   'frontier',
    CLAIM:      'claim',
    YIELD:      'yield',
    VOTE:       'vote',
    ALERT:      'alert'
});


// ─── AGENT COMMUNICATOR ──────────────────────────────────────────────

/**
 * Per-agent communication manager.
 * Handles message formatting, sending budget, and received message processing.
 */
export class AgentCommunicator {
    /**
     * @param {string} droneId
     * @param {Object} config
     */
    constructor(droneId, config = {}) {
        this.droneId = droneId;

        // Communication budget
        this.commIntervalMs   = config.commIntervalMs   ?? 500;  // Min ms between sends
        this.lastSendTime     = 0;
        this.heartbeatInterval = config.heartbeatInterval ?? 1000; // ms
        this.lastHeartbeat    = 0;

        // Inbox (messages received from CommLink, processed each tick)
        /** @type {Array<CommMessage>} */
        this.inbox = [];

        // Outbox (messages to send via CommLink)
        /** @type {Array<CommMessage>} */
        this.outbox = [];

        // Known peers (from heartbeats)
        /** @type {Map<string, PeerInfo>} */
        this.peers = new Map();
        this.peerTimeout = config.peerTimeout ?? 5000; // ms before considering peer lost

        // Claimed zones (distributed task allocation)
        /** @type {Map<string, { claimedBy: string, timestamp: number }>} */
        this.zoneClaims = new Map();

        // Stats
        this.messagesSent = 0;
        this.messagesReceived = 0;
        this.mapSharesSent = 0;
        this.mapSharesReceived = 0;

        // Received map data (accumulated between getReceivedMaps calls)
        /** @type {Array<Object>} */
        this._receivedMaps = [];
    }

    /**
     * Process incoming messages from the CommLink.
     * @param {Array<Object>} rawMessages - Raw payloads from CommLink.receive()
     * @param {number} now - Current timestamp
     * @returns {{ mapUpdates: Array, frontierInfo: Array, claims: Array, votes: Array }}
     */
    processInbox(rawMessages, now) {
        const result = {
            mapUpdates: [],
            frontierInfo: [],
            claims: [],
            votes: [],
            alerts: []
        };

        for (const msg of rawMessages) {
            if (!msg || !msg.type) continue;
            this.messagesReceived++;

            switch (msg.type) {
                case CommMessageType.HEARTBEAT:
                    this._handleHeartbeat(msg, now);
                    break;

                case CommMessageType.MAP_SHARE:
                    result.mapUpdates.push(msg);
                    this.mapSharesReceived++;
                    if (msg.data) this._receivedMaps.push(msg.data);
                    break;

                case CommMessageType.FRONTIER:
                    result.frontierInfo.push(msg);
                    break;

                case CommMessageType.CLAIM:
                    this._handleClaim(msg, now);
                    result.claims.push(msg);
                    break;

                case CommMessageType.YIELD:
                    this._handleYield(msg);
                    break;

                case CommMessageType.VOTE:
                    result.votes.push(msg);
                    break;

                case CommMessageType.ALERT:
                    result.alerts.push(msg);
                    break;
            }
        }

        // Prune stale peers
        for (const [peerId, info] of this.peers) {
            if (now - info.lastSeen > this.peerTimeout) {
                this.peers.delete(peerId);
            }
        }

        return result;
    }

    /**
     * Queue a heartbeat message (always sent, not learned).
     * @param {Object} position - { x, y, z }
     * @param {number} battery
     * @param {string} phase - Current flight phase
     * @param {number} now
     */
    queueHeartbeat(position, battery, phase, now) {
        if (now - this.lastHeartbeat < this.heartbeatInterval) return;
        this.lastHeartbeat = now;

        this.outbox.push({
            type: CommMessageType.HEARTBEAT,
            from: this.droneId,
            timestamp: now,
            data: {
                position: { x: position.x, y: position.y, z: position.z },
                battery,
                phase
            }
        });
    }

    /**
     * Queue a map share message (agent decides to share).
     * @param {Object} sparseGrid - From LocalOccupancyGrid.exportSparse()
     * @param {number} now
     * @returns {boolean} True if queued (respects comm budget)
     */
    queueMapShare(sparseGrid, now) {
        if (now - this.lastSendTime < this.commIntervalMs) return false;
        if (!sparseGrid || sparseGrid.cells.length === 0) return false;

        this.lastSendTime = now;
        this.outbox.push({
            type: CommMessageType.MAP_SHARE,
            from: this.droneId,
            timestamp: now,
            data: sparseGrid
        });
        this.mapSharesSent++;
        return true;
    }

    /**
     * Queue a frontier announcement.
     * @param {Object} frontier - { x, z, dist }
     * @param {number} now
     */
    queueFrontierAnnouncement(frontier, now) {
        if (now - this.lastSendTime < this.commIntervalMs) return false;
        if (!frontier) return false;

        this.lastSendTime = now;
        this.outbox.push({
            type: CommMessageType.FRONTIER,
            from: this.droneId,
            timestamp: now,
            data: { x: frontier.x, z: frontier.z, dist: frontier.dist }
        });
        return true;
    }

    /**
     * Queue a zone claim.
     * @param {string} zoneKey - Zone identifier (e.g., grid sector)
     * @param {number} now
     */
    queueClaim(zoneKey, now) {
        this.outbox.push({
            type: CommMessageType.CLAIM,
            from: this.droneId,
            timestamp: now,
            data: { zoneKey }
        });

        this.zoneClaims.set(zoneKey, { claimedBy: this.droneId, timestamp: now });
    }

    /**
     * Queue a zone yield (release claim).
     * @param {string} zoneKey
     * @param {number} now
     */
    queueYield(zoneKey, now) {
        this.outbox.push({
            type: CommMessageType.YIELD,
            from: this.droneId,
            timestamp: now,
            data: { zoneKey }
        });
        this.zoneClaims.delete(zoneKey);
    }

    /**
     * Queue a consensus vote message.
     * @param {Object} voteMsg - Raft/Paxos message
     * @param {number} now
     */
    queueVote(voteMsg, now) {
        this.outbox.push({
            type: CommMessageType.VOTE,
            from: this.droneId,
            timestamp: now,
            data: voteMsg
        });
    }

    /**
     * Queue an alert (emergency, danger zone detected).
     * @param {string} alertType
     * @param {Object} data
     * @param {number} now
     */
    queueAlert(alertType, data, now) {
        this.outbox.push({
            type: CommMessageType.ALERT,
            from: this.droneId,
            timestamp: now,
            data: { alertType, ...data }
        });
    }

    /**
     * Drain the outbox (returns messages to send via CommLink).
     * @returns {Array<Object>}
     */
    drainOutbox() {
        const messages = this.outbox.splice(0);
        this.messagesSent += messages.length;
        return messages;
    }

    /**
     * Get known peer positions (for neighbor awareness).
     * @returns {Array<{id: string, position: Object, battery: number, phase: string}>}
     */
    getKnownPeers() {
        return Array.from(this.peers.values());
    }

    /**
     * Check if a zone is claimed by another drone.
     * @param {string} zoneKey
     * @returns {string|null} Claiming drone ID, or null
     */
    getZoneClaimer(zoneKey) {
        const claim = this.zoneClaims.get(zoneKey);
        if (!claim) return null;
        return claim.claimedBy;
    }

    /**
     * Get communication stats.
     */
    getStats() {
        return {
            sent: this.messagesSent,
            received: this.messagesReceived,
            mapSent: this.mapSharesSent,
            mapReceived: this.mapSharesReceived,
            activePeers: this.peers.size,
            activeClaims: this.zoneClaims.size
        };
    }

    // ── Internal handlers ──

    _handleHeartbeat(msg, now) {
        this.peers.set(msg.from, {
            id: msg.from,
            position: msg.data.position,
            battery: msg.data.battery,
            phase: msg.data.phase,
            lastSeen: now
        });
    }

    _handleClaim(msg, now) {
        const existing = this.zoneClaims.get(msg.data.zoneKey);
        // Conflict resolution: lowest drone ID wins
        if (!existing || msg.from < existing.claimedBy ||
            (msg.from === existing.claimedBy && msg.timestamp > existing.timestamp)) {
            this.zoneClaims.set(msg.data.zoneKey, {
                claimedBy: msg.from,
                timestamp: msg.timestamp
            });
        }
    }

    _handleYield(msg) {
        const existing = this.zoneClaims.get(msg.data.zoneKey);
        if (existing && existing.claimedBy === msg.from) {
            this.zoneClaims.delete(msg.data.zoneKey);
        }
    }

    /**
     * Get and clear accumulated received map data.
     * @returns {Array<Object>}
     */
    getReceivedMaps() {
        return this._receivedMaps.splice(0);
    }

    /**
     * Get the number of active peers.
     * @returns {number}
     */
    getActivePeerCount() {
        return this.peers.size;
    }

    /**
     * Reset all state for a new episode.
     */
    reset() {
        this.inbox.length = 0;
        this.outbox.length = 0;
        this.peers.clear();
        this.zoneClaims.clear();
        this._receivedMaps.length = 0;
        this.messagesSent = 0;
        this.messagesReceived = 0;
        this.mapSharesSent = 0;
        this.mapSharesReceived = 0;
        this.lastSendTime = 0;
        this.lastHeartbeat = 0;
    }
}
