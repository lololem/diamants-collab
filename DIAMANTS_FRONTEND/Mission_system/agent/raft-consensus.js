/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * raft-consensus.js — Raft consensus algorithm adapted for drone swarms.
 *
 * Each drone runs a Raft node. The elected leader coordinates:
 *   - Zone/task assignments (replicated log)
 *   - Parameter changes (autonomy level, mission mode)
 *   - Emergency decisions (RTL, land all)
 *
 * Adaptations for drones (vs server Raft):
 *   - Larger election timeouts (500-2000ms) for radio latency
 *   - Graceful partition handling (partitions run independent)
 *   - Heartbeat piggybacks on CommLink messages
 *   - Non-blocking: tick() is O(1) amortized, safe for 60fps
 *
 * @module raft-consensus
 */

// ─── RAFT MESSAGE TYPES ──────────────────────────────────────────────

export const RaftMsgType = Object.freeze({
    REQUEST_VOTE:          'raft_request_vote',
    REQUEST_VOTE_RESPONSE: 'raft_vote_response',
    APPEND_ENTRIES:        'raft_append_entries',
    APPEND_ENTRIES_RESPONSE: 'raft_append_response'
});

export const RaftRole = Object.freeze({
    FOLLOWER:  'follower',
    CANDIDATE: 'candidate',
    LEADER:    'leader'
});


// ─── LOG ENTRY ───────────────────────────────────────────────────────

/**
 * @typedef {Object} LogEntry
 * @property {number} term
 * @property {number} index
 * @property {Object} command - { type, droneId, payload, timestamp }
 */

// ─── RAFT COMMAND TYPES ──────────────────────────────────────────────

export const RaftCommand = Object.freeze({
    ZONE_ASSIGN:     'zone_assign',     // Assign exploration zone to a drone
    PARAM_UPDATE:    'param_update',    // Update shared parameter
    MISSION_ORDER:   'mission_order',   // Mission-level command
    EMERGENCY:       'emergency',       // Emergency action
    PEER_JOIN:       'peer_join',       // New drone joins swarm
    PEER_LEAVE:      'peer_leave'       // Drone leaves swarm
});


// ─── RAFT NODE ───────────────────────────────────────────────────────

/**
 * Raft consensus node for a single drone.
 * Designed for non-blocking operation within a 60fps game loop.
 */
export class RaftNode {
    /**
     * @param {string} id - Drone ID
     * @param {Object} config
     */
    constructor(id, config = {}) {
        this.id = id;

        // ── Persistent state ──
        this.currentTerm = 0;
        this.votedFor    = null;
        /** @type {LogEntry[]} */
        this.log         = [];

        // ── Volatile state ──
        this.commitIndex  = -1;
        this.lastApplied  = -1;
        this.role         = RaftRole.FOLLOWER;

        // ── Leader state ──
        /** @type {Map<string, number>} */
        this.nextIndex    = new Map();
        /** @type {Map<string, number>} */
        this.matchIndex   = new Map();

        // ── Election ──
        this.electionTimeout = 0;  // ms until election starts
        this.lastHeartbeat   = 0;  // last time we heard from leader
        /** @type {Set<string>} */
        this.votesReceived   = new Set();

        // ── Peers ──
        /** @type {Set<string>} */
        this.peers = new Set();

        // ── Config ──
        this.heartbeatIntervalMs    = config.heartbeatInterval    ?? 300;
        this.electionTimeoutMinMs   = config.electionTimeoutMin   ?? 800;
        this.electionTimeoutMaxMs   = config.electionTimeoutMax   ?? 2000;

        // ── Message queue (non-blocking outbox) ──
        /** @type {Array<Object>} */
        this.outbox = [];

        // ── Committed command callback ──
        /** @type {Function|null} */
        this.onCommit = null;

        // ── State ──
        this._nextHeartbeat = 0;
        this._resetElectionTimer();
    }

    // ────────────────────────────────────────────────────────────────
    // PUBLIC API
    // ────────────────────────────────────────────────────────────────

    /**
     * Add a peer to the known set.
     * @param {string} peerId
     */
    addPeer(peerId) {
        if (peerId !== this.id) {
            this.peers.add(peerId);
        }
    }

    /**
     * Remove a peer.
     * @param {string} peerId
     */
    removePeer(peerId) {
        this.peers.delete(peerId);
        this.nextIndex.delete(peerId);
        this.matchIndex.delete(peerId);
    }

    /**
     * Non-blocking tick. Call every frame or every N ms.
     * Returns outgoing messages to send via CommLink.
     *
     * @param {number} now - Current timestamp (ms)
     * @returns {Array<Object>} Messages to broadcast
     */
    tick(now) {
        // Apply committed entries
        this._applyCommitted();

        if (this.role === RaftRole.LEADER) {
            // Send heartbeats periodically
            if (now >= this._nextHeartbeat) {
                this._sendHeartbeats();
                this._nextHeartbeat = now + this.heartbeatIntervalMs;
            }
        } else {
            // Check election timeout
            if (now - this.lastHeartbeat > this.electionTimeout) {
                this._startElection(now);
            }
        }

        // Drain and return outbox
        return this.outbox.splice(0);
    }

    /**
     * Handle an incoming Raft message.
     * @param {Object} msg - Message with { type, from, data }
     * @param {number} now
     */
    handleMessage(msg, now) {
        if (!msg || !msg.data) return;
        const data = msg.data;

        // Any message with higher term → step down
        if (data.term > this.currentTerm) {
            this.currentTerm = data.term;
            this.role = RaftRole.FOLLOWER;
            this.votedFor = null;
        }

        switch (data.raftType) {
            case RaftMsgType.REQUEST_VOTE:
                this._handleRequestVote(data, msg.from, now);
                break;
            case RaftMsgType.REQUEST_VOTE_RESPONSE:
                this._handleVoteResponse(data, msg.from, now);
                break;
            case RaftMsgType.APPEND_ENTRIES:
                this._handleAppendEntries(data, msg.from, now);
                break;
            case RaftMsgType.APPEND_ENTRIES_RESPONSE:
                this._handleAppendEntriesResponse(data, msg.from);
                break;
        }
    }

    /**
     * Propose a command (only works if this node is the leader).
     * @param {Object} command - { type, droneId, payload }
     * @returns {boolean} True if accepted (we are leader)
     */
    propose(command) {
        if (this.role !== RaftRole.LEADER) return false;

        const entry = {
            term: this.currentTerm,
            index: this.log.length,
            command: {
                ...command,
                timestamp: Date.now()
            }
        };
        this.log.push(entry);

        // Immediately try to replicate
        for (const peer of this.peers) {
            this._sendAppendEntries(peer);
        }
        return true;
    }

    /**
     * Is this node the current leader?
     * @returns {boolean}
     */
    isLeader() {
        return this.role === RaftRole.LEADER;
    }

    /**
     * Get current leader ID (if known).
     * @returns {string|null}
     */
    getLeaderId() {
        if (this.role === RaftRole.LEADER) return this.id;
        return this._currentLeader || null;
    }

    /**
     * Get the current role as a string for the observation vector.
     * @returns {'leader'|'candidate'|'follower'}
     */
    getRoleString() {
        return this.role;
    }

    /**
     * Get Raft state for debugging.
     */
    getState() {
        return {
            id: this.id,
            role: this.role,
            term: this.currentTerm,
            logLength: this.log.length,
            commitIndex: this.commitIndex,
            peers: this.peers.size,
            votedFor: this.votedFor
        };
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Election
    // ────────────────────────────────────────────────────────────────

    _startElection(now) {
        this.role = RaftRole.CANDIDATE;
        this.currentTerm++;
        this.votedFor = this.id;
        this.votesReceived = new Set([this.id]);
        this.lastHeartbeat = now;
        this._resetElectionTimer();

        const lastIdx = this.log.length - 1;
        const lastTerm = lastIdx >= 0 ? this.log[lastIdx].term : 0;

        // Request votes from all peers
        for (const peer of this.peers) {
            this.outbox.push({
                to: peer,
                data: {
                    raftType: RaftMsgType.REQUEST_VOTE,
                    term: this.currentTerm,
                    candidateId: this.id,
                    lastLogIndex: lastIdx,
                    lastLogTerm: lastTerm
                }
            });
        }

        // If alone, become leader immediately
        if (this.peers.size === 0) {
            this._becomeLeader();
        }
    }

    _handleRequestVote(data, from, now) {
        let granted = false;

        if (data.term >= this.currentTerm) {
            const lastIdx = this.log.length - 1;
            const lastTerm = lastIdx >= 0 ? this.log[lastIdx].term : 0;

            // Log up-to-date check
            const logOk = data.lastLogTerm > lastTerm ||
                (data.lastLogTerm === lastTerm && data.lastLogIndex >= lastIdx);

            if ((this.votedFor === null || this.votedFor === data.candidateId) && logOk) {
                granted = true;
                this.votedFor = data.candidateId;
                this.lastHeartbeat = now; // Reset election timer
            }
        }

        this.outbox.push({
            to: from,
            data: {
                raftType: RaftMsgType.REQUEST_VOTE_RESPONSE,
                term: this.currentTerm,
                voteGranted: granted
            }
        });
    }

    _handleVoteResponse(data, from, now) {
        if (this.role !== RaftRole.CANDIDATE || data.term !== this.currentTerm) return;

        if (data.voteGranted) {
            this.votesReceived.add(from);
            const needed = Math.floor((this.peers.size + 1) / 2) + 1;
            if (this.votesReceived.size >= needed) {
                this._becomeLeader();
            }
        }
    }

    _becomeLeader() {
        this.role = RaftRole.LEADER;
        this._currentLeader = this.id;

        // Initialize leader state
        for (const peer of this.peers) {
            this.nextIndex.set(peer, this.log.length);
            this.matchIndex.set(peer, -1);
        }

        // Send immediate heartbeat
        this._sendHeartbeats();
        console.log(`[Raft:${this.id}] 👑 Became LEADER (term ${this.currentTerm})`);
    }

    // ────────────────────────────────────────────────────────────────
    // INTERNAL: Log Replication
    // ────────────────────────────────────────────────────────────────

    _sendHeartbeats() {
        for (const peer of this.peers) {
            this._sendAppendEntries(peer);
        }
    }

    _sendAppendEntries(peer) {
        const ni = this.nextIndex.get(peer) ?? this.log.length;
        const prevIdx = ni - 1;
        const prevTerm = prevIdx >= 0 && prevIdx < this.log.length ?
            this.log[prevIdx].term : 0;

        const entries = this.log.slice(ni);

        this.outbox.push({
            to: peer,
            data: {
                raftType: RaftMsgType.APPEND_ENTRIES,
                term: this.currentTerm,
                leaderId: this.id,
                prevLogIndex: prevIdx,
                prevLogTerm: prevTerm,
                entries: entries,
                leaderCommit: this.commitIndex
            }
        });
    }

    _handleAppendEntries(data, from, now) {
        if (data.term < this.currentTerm) {
            this.outbox.push({
                to: from,
                data: {
                    raftType: RaftMsgType.APPEND_ENTRIES_RESPONSE,
                    term: this.currentTerm,
                    success: false
                }
            });
            return;
        }

        // Valid leader
        this.role = RaftRole.FOLLOWER;
        this.lastHeartbeat = now;
        this._currentLeader = data.leaderId;

        // Log consistency check
        if (data.prevLogIndex >= 0) {
            if (data.prevLogIndex >= this.log.length ||
                this.log[data.prevLogIndex].term !== data.prevLogTerm) {
                this.outbox.push({
                    to: from,
                    data: {
                        raftType: RaftMsgType.APPEND_ENTRIES_RESPONSE,
                        term: this.currentTerm,
                        success: false,
                        conflictIndex: Math.min(data.prevLogIndex, this.log.length - 1)
                    }
                });
                return;
            }
        }

        // Append entries
        if (data.entries && data.entries.length > 0) {
            for (let i = 0; i < data.entries.length; i++) {
                const idx = data.prevLogIndex + 1 + i;
                if (idx < this.log.length) {
                    if (this.log[idx].term !== data.entries[i].term) {
                        this.log.length = idx;
                        this.log.push(data.entries[i]);
                    }
                } else {
                    this.log.push(data.entries[i]);
                }
            }
        }

        // Update commit index
        if (data.leaderCommit > this.commitIndex) {
            this.commitIndex = Math.min(data.leaderCommit, this.log.length - 1);
        }

        this.outbox.push({
            to: from,
            data: {
                raftType: RaftMsgType.APPEND_ENTRIES_RESPONSE,
                term: this.currentTerm,
                success: true,
                matchIndex: this.log.length - 1
            }
        });
    }

    _handleAppendEntriesResponse(data, from) {
        if (this.role !== RaftRole.LEADER) return;

        if (data.success) {
            if (data.matchIndex !== undefined) {
                this.nextIndex.set(from, data.matchIndex + 1);
                this.matchIndex.set(from, data.matchIndex);
            }
            // Check if we can advance commitIndex
            this._advanceCommitIndex();
        } else {
            // Log inconsistency — retry with earlier entries
            const ni = this.nextIndex.get(from) ?? 1;
            this.nextIndex.set(from, Math.max(0,
                data.conflictIndex !== undefined ? data.conflictIndex : ni - 1));
            this._sendAppendEntries(from);
        }
    }

    _advanceCommitIndex() {
        for (let n = this.commitIndex + 1; n < this.log.length; n++) {
            if (this.log[n].term !== this.currentTerm) continue;
            let replicatedCount = 1; // self
            for (const peer of this.peers) {
                if ((this.matchIndex.get(peer) ?? -1) >= n) {
                    replicatedCount++;
                }
            }
            if (replicatedCount > (this.peers.size + 1) / 2) {
                this.commitIndex = n;
            }
        }
    }

    _applyCommitted() {
        while (this.lastApplied < this.commitIndex) {
            this.lastApplied++;
            const entry = this.log[this.lastApplied];
            if (entry && this.onCommit) {
                this.onCommit(entry.command);
            }
        }
    }

    _resetElectionTimer() {
        this.electionTimeout = this.electionTimeoutMinMs +
            Math.random() * (this.electionTimeoutMaxMs - this.electionTimeoutMinMs);
    }
}
