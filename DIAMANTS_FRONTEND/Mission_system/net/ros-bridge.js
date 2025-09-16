// Lightweight ROS bridge for browser-based WebGL simulation (ROS topics over WebSocket)
// Prefers roslibjs; dynamically imports from CDN if not present.

export class RosWebBridge {
    constructor({ url = 'ws://localhost:9090', autoConnect = false } = {}) {
        // Vite env override if provided
        const envUrl = (typeof import.meta !== 'undefined' && import.meta.env && import.meta.env.VITE_ROSBRIDGE_URL) || null;
        this.url = envUrl || url;
        this.ros = null;
        this.ROSLIB = null;
        this.connected = false;
        this.topics = new Map();
        if (autoConnect) this.connect();
    }

    async ensureLib() {
        if (this.ROSLIB) return this.ROSLIB;
        // Try global first
        if (typeof window !== 'undefined' && window.ROSLIB) {
            this.ROSLIB = window.ROSLIB;
            return this.ROSLIB;
        }
        // Try multiple CDNs with ESM-friendly variants
        const cdnCandidates = [
            'https://esm.sh/roslib?bundle',
            'https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js',
            'https://unpkg.com/roslib/build/roslib.js'
        ];
        for (const url of cdnCandidates) {
            try {
                const mod = await import(/* @vite-ignore */ url);
                this.ROSLIB = mod?.default || mod?.ROSLIB || (typeof window !== 'undefined' ? window.ROSLIB : null) || mod;
                if (this.ROSLIB) return this.ROSLIB;
            } catch (_) { /* try next */ }
        }
        // As a last resort, install a no-op shim so the rest of the app can run without ROS
        console.warn('ROS lib not available; installing no-op shim. The app will run without ROS.');
        const ShimTopic = class {
            constructor() {}
            subscribe() {}
            unsubscribe() {}
            publish() {}
        };
        const ShimMessage = class { constructor(obj) { Object.assign(this, obj); } };
        const ShimRos = class {
            constructor() { setTimeout(() => this.onclose && this.onclose(), 0); }
            on(evt, cb) { if (evt === 'close') this.onclose = cb; }
        };
        this.ROSLIB = { Topic: ShimTopic, Message: ShimMessage, Ros: ShimRos };
        return this.ROSLIB;
    }

    async connect() {
        try {
            const ROSLIB = await this.ensureLib();
            // If shim, just resolve without a real connection
            if (!ROSLIB || !ROSLIB.Ros) {
                this.connected = false;
                console.warn('ROS shim active; skipping connection.');
                return;
            }
            this.ros = new ROSLIB.Ros({ url: this.url });
            return new Promise((resolve) => {
                this.ros.on('connection', () => {
                    this.connected = true;
                    console.log(`🔌 Connected to rosbridge at ${this.url}`);
                    resolve();
                });
                this.ros.on('error', (err) => {
                    console.warn('ROS connection error (non-fatal):', err);
                    this.connected = false;
                    resolve(); // resolve to avoid blocking app
                });
                this.ros.on('close', () => {
                    console.warn('ROS connection closed');
                    this.connected = false;
                });
            });
        } catch (e) {
            console.warn('ROS not available; continuing without ROS.', e?.message || e);
            this.connected = false;
        }
    }

    advertise(topicName, type) {
        if (!this.connected) return null;
        if (this.topics.has(topicName)) return this.topics.get(topicName);
        const topic = new this.ROSLIB.Topic({ ros: this.ros, name: topicName, messageType: type, queue_size: 10 });
        // roslib auto-advertises on publish; we just keep a handle
        this.topics.set(topicName, topic);
        return topic;
    }

    publish(topicName, type, message) {
        const topic = this.advertise(topicName, type);
        if (!topic) return;
        try {
            topic.publish(new this.ROSLIB.Message(message));
        } catch (e) {
            console.warn('Publish failed for', topicName, e);
        }
    }

    subscribe(topicName, type, cb) {
        if (!this.connected) return () => {};
        const topic = new this.ROSLIB.Topic({ ros: this.ros, name: topicName, messageType: type, queue_size: 10 });
        const handler = (msg) => {
            try { cb(msg); } catch (_) {}
        };
        topic.subscribe(handler);
        return () => {
            try { topic.unsubscribe(handler); } catch (_) {}
        };
    }
}

// Helpers for composing common messages
export function makeOdometry({ frame_id = 'map', child_frame_id = 'base_link', position, orientation, linear, angular }) {
    const now = Date.now();
    return {
        header: { stamp: { secs: Math.floor(now / 1000), nsecs: (now % 1000) * 1e6 }, frame_id },
        child_frame_id,
        pose: { pose: { position, orientation } },
        twist: { twist: { linear, angular } }
    };
}

export function yawToQuat(yaw) {
    const half = yaw * 0.5;
    return { x: 0, y: Math.sin(half), z: 0, w: Math.cos(half) };
}
