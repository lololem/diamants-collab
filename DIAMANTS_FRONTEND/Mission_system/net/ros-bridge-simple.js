// Simplified ROS bridge for DIAMANTS V3 - Fixed version

export class RosWebBridge {
    constructor({ url = 'ws://localhost:9090', autoConnect = false, silent = true } = {}) {
        this.url = url;
        this.ros = null;
        this.connected = false;
        this.topics = new Map();
        this.silent = silent; // Flag pour contrôler les logs
        
        if (autoConnect) {
            this.connect();
        }
    }

    async connect() {
        if (!this.silent) console.log('🔌 Attempting to connect to ROS bridge...');
        try {
            // Simulation mode for now
            this.connected = true;
            if (!this.silent) console.log('✅ ROS bridge connected (simulation mode)');
            return true;
        } catch (error) {
            if (!this.silent) console.warn('⚠️ ROS bridge connection failed:', error.message);
            return false;
        }
    }

    disconnect() {
        this.connected = false;
        if (!this.silent) console.log('🔌 ROS bridge disconnected');
    }

    publish(topic, message) {
        if (this.connected) {
            if (!this.silent) console.log(`📤 Publishing to ${topic}:`, message);
        } else {
            if (!this.silent) console.warn('⚠️ Cannot publish: ROS bridge not connected');
        }
    }

    subscribe(topic, callback) {
        if (!this.silent) console.log(`📥 Subscribed to ${topic}`);
        this.topics.set(topic, callback);
    }

    isConnected() {
        return this.connected;
    }
}

export function makeOdometry({ 
    position = { x: 0, y: 0, z: 0 }, 
    orientation = { x: 0, y: 0, z: 0, w: 1 }, 
    linear = { x: 0, y: 0, z: 0 }, 
    angular = { x: 0, y: 0, z: 0 },
    frame_id = 'odom',
    child_frame_id = 'base_link'
} = {}) {
    const now = Date.now();
    return {
        header: {
            stamp: {
                secs: Math.floor(now / 1000),
                nsecs: (now % 1000) * 1e6
            },
            frame_id
        },
        child_frame_id,
        pose: {
            pose: {
                position,
                orientation
            }
        },
        twist: {
            twist: {
                linear,
                angular
            }
        }
    };
}

export function yawToQuat(yaw) {
    const half = yaw * 0.5;
    return {
        x: 0,
        y: Math.sin(half),
        z: 0,
        w: Math.cos(half)
    };
}

// Mode silencieux par défaut - décommentez pour réactiver les logs
// console.log('✅ Simplified ROS bridge loaded successfully');
