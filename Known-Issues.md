# ⚠️ Known Issues and Solutions

This page documents identified problems in the DIAMANTS system, their causes, and recommended solutions.

## 🔴 Critical Issues Identified

### 1. **Frontend Module Loading Chaos**

**Status:** 🔴 **CRITICAL** - Prevents system initialization

**Description:**
The frontend uses 5 contradictory THREE.js import strategies simultaneously, causing module resolution conflicts.

**Affected Files:**
- `Mission_system/main.js` (ES6 imports)
- `Mission_system/index.html` (CDN global)
- `visual/three-visualizer.js` (Dynamic imports)
- `controllers/orbit-controls.js` (Assumed globals)
- `physics/physics-engine.js` (Imports avoided)

**Impact:**
- Application fails to start
- `THREE is not defined` errors
- Module resolution conflicts

**Recommended Solution:**
```javascript
// Unified strategy - Use ES6 modules exclusively
// In package.json
{
  "type": "module",
  "dependencies": {
    "three": "^0.160.0"
  }
}

// In all files
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
```

---

### 2. **Missing UI-Controller Bridge**

**Status:** 🔴 **CRITICAL** - Non-functional interface

**Description:**
The UI interface references `application.controller` and `window.missionController` objects that don't exist.

**Affected Files:**
- `ui/mission-interface.js` - Line 25
- `ui/drone-controls.js` - Line 42
- `controllers/mission-controller.js` - Not globally exposed

**Errors:**
```javascript
// Error in mission-interface.js
application.controller.startMission(); // application undefined

// Error in drone-controls.js  
window.missionController.selectDrone(); // missionController undefined
```

**Recommended Solution:**
```javascript
// Create global state manager
class ApplicationState {
    constructor() {
        this.missionController = new MissionController();
        this.droneManager = new DroneManager();
    }
}

// Expose via module pattern
export const appState = new ApplicationState();
window.DIAMANTS = appState; // For legacy compatibility
```

---

### 3. **Physics System Duplication**

**Status:** 🟠 **MAJOR** - Unpredictable behaviors

**Description:**
Multiple instances of identical physics classes create conflicts and redundant calculations.

**Affected Files:**
- `physics/physics-engine.js` - PhysicsEngine #1
- `physics/cannon-physics.js` - PhysicsEngine #2  
- `core/diamant-system.js` - PhysicsWorld #3

**Impact:**
- Inconsistent physics calculations
- Degraded performance
- Poorly detected collisions

**Recommended Solution:**
```javascript
// Singleton pattern for physics engine
class PhysicsEngine {
    static instance = null;
    
    static getInstance() {
        if (!PhysicsEngine.instance) {
            PhysicsEngine.instance = new PhysicsEngine();
        }
        return PhysicsEngine.instance;
    }
}
```

---

### 4. **Broken Mission Execution Bridge**

**Status:** 🟠 **MAJOR** - Simulated missions without effect

**Description:**
The mission manager simulates movements but doesn't affect real drone objects.

**Problematic Code:**
```javascript
// In mission-manager.js
executeWaypoint(waypoint) {
    // ❌ Local simulation only
    this.simulatedPosition = waypoint.position;
    console.log(`Drone moved to ${this.simulatedPosition}`);
    
    // ❌ Doesn't update real drone
    // drone.position.copy(waypoint.position); // MISSING
}
```

**Recommended Solution:**
```javascript
executeWaypoint(waypoint) {
    // ✅ Update real drone object
    const drone = this.droneManager.getDrone(waypoint.droneId);
    if (drone) {
        drone.position.copy(waypoint.position);
        drone.updatePhysics();
    }
    
    // ✅ Send ROS2 command
    this.rosInterface.sendMoveCommand(waypoint.droneId, waypoint.position);
}
```

---

### 5. **Limited Environment Integration**

**Status:** 🟡 **MINOR** - Reduced functionality

**Description:**
The rich 3D environment has no dynamic feedback with drones.

**Issues:**
- No collision detection with obstacles
- Weather not considered in flight
- Static terrain without interaction

**Recommended Solution:**
```javascript
// Environmental detection system
class EnvironmentManager {
    constructor() {
        this.obstacles = [];
        this.weatherSystem = new WeatherSystem();
    }
    
    checkDroneCollisions(drone) {
        this.obstacles.forEach(obstacle => {
            if (this.detectCollision(drone, obstacle)) {
                this.handleCollision(drone, obstacle);
            }
        });
    }
    
    applyWeatherEffects(drone) {
        const wind = this.weatherSystem.getWindAt(drone.position);
        drone.physics.addForce(wind);
    }
}
```

## 🔧 Immediate Solutions

### Emergency Fix for Module Loading

**Create:** `Mission_system/module-fix.js`
```javascript
// Temporary fix for THREE.js conflicts
if (typeof THREE === 'undefined') {
    // Load THREE.js via dynamic import
    const THREE = await import('three');
    window.THREE = THREE;
}

// Unify all imports
export { THREE };
```

### Integrity Check Script

**Create:** `scripts/check-integrity.js`
```javascript
#!/usr/bin/env node

const checks = [
    () => checkModuleConsistency(),
    () => checkUIControllerBridge(),
    () => checkPhysicsUniqueness(),
    () => checkMissionExecution(),
    () => checkEnvironmentIntegration()
];

checks.forEach((check, i) => {
    try {
        check();
        console.log(`✅ Check ${i+1} passed`);
    } catch (error) {
        console.error(`❌ Check ${i+1} failed:`, error.message);
    }
});
```

## 🛠 Systemic Repair Plan

### Phase 1: Stabilization (Priority 1) 🔴

1. **Unify THREE.js imports**
   - Remove CDN imports from `index.html`
   - Convert all files to ES6 modules
   - Test browser compatibility

2. **Create UI-Controller bridge**
   - Implement global `ApplicationState`
   - Connect all UI components
   - Add error handling

3. **Centralize physics system**
   - Implement singleton PhysicsEngine
   - Migrate all calculations to single instance
   - Clean redundant instances

### Phase 2: Functionality (Priority 2) 🟠

4. **Fix mission execution**
   - Connect simulation to real objects
   - Implement complete ROS2 bridge
   - Add command validation

5. **Integrate dynamic environment**
   - Add collision detection
   - Implement weather system
   - Create environmental feedback

### Phase 3: Optimization (Priority 3) 🟡

6. **Optimize performance**
   - Profile bottlenecks
   - Implement frustum culling
   - Optimize 3D rendering

7. **Improve robustness**
   - Add automated tests
   - Implement error boundaries
   - Create system health monitoring

## 🧪 Validation Tests

### Module Integrity Test

```javascript
// Automated import testing
describe('Module Loading', () => {
    it('should load THREE.js without conflicts', async () => {
        const THREE = await import('three');
        expect(THREE).toBeDefined();
        expect(window.THREE).toBeDefined();
        expect(THREE === window.THREE).toBe(true);
    });
});
```

### UI-Controller Connection Test

```javascript
// UI reference testing
describe('UI Controller Bridge', () => {
    it('should have mission controller accessible', () => {
        expect(window.DIAMANTS.missionController).toBeDefined();
        expect(typeof window.DIAMANTS.missionController.startMission).toBe('function');
    });
});
```

### Mission Execution Test

```javascript
// Real mission testing
describe('Mission Execution', () => {
    it('should move real drone objects', async () => {
        const drone = droneManager.getDrone('cf2x_01');
        const initialPosition = drone.position.clone();
        
        await missionManager.executeWaypoint({
            droneId: 'cf2x_01',
            position: {x: 1, y: 1, z: 1}
        });
        
        expect(drone.position).not.toEqual(initialPosition);
    });
});
```

## 📊 System Health Metrics

### Health Monitoring Dashboard

```javascript
class SystemHealthMonitor {
    constructor() {
        this.metrics = {
            moduleLoadErrors: 0,
            uiConnectionErrors: 0,
            physicsConflicts: 0,
            missionExecutionFailures: 0,
            environmentLags: 0
        };
    }
    
    generateHealthReport() {
        const totalIssues = Object.values(this.metrics).reduce((sum, val) => sum + val, 0);
        const healthScore = Math.max(0, 100 - totalIssues * 10);
        
        return {
            score: healthScore,
            status: healthScore > 80 ? 'Healthy' : healthScore > 50 ? 'Warning' : 'Critical',
            details: this.metrics
        };
    }
}
```

## 🚨 Emergency Procedures

### Complete System Failure

1. **Emergency stop**: `Ctrl+C` in all terminals
2. **Reset environment**: `./stop_diamants.sh && ./launch_diamants.sh`
3. **Check logs**: `tail -f DIAMANTS_API/logs/error.log`
4. **Contact support**: Create GitHub issue with logs

### Unresponsive Drones

1. **Emergency stop**: Red button in interface
2. **Forced landing**: `rosservice call /emergency_land`
3. **Check ROS2**: `ros2 node list`
4. **Reset connection**: Restart websocket bridge

## 🔍 Diagnostic Tools

### Log Analysis Commands

```bash
# Frontend errors
tail -f /var/log/nginx/error.log

# API errors
tail -f DIAMANTS_API/logs/error.log

# ROS2 diagnostics
ros2 run diagnostics diagnostic_viewer

# System resources
htop
iotop
nethogs
```

### Network Diagnostics

```bash
# Check connections
netstat -tulpn | grep -E "(3000|8080|7400)"

# Test WebSocket
wscat -c ws://localhost:8080/ws

# ROS2 communication
ros2 multicast receive
ros2 multicast send
```

### Performance Profiling

```bash
# Frontend performance
# Open DevTools -> Performance tab -> Record

# Python profiling
python -m cProfile -o profile.stats launcher.py
python -c "import pstats; p=pstats.Stats('profile.stats'); p.sort_stats('cumulative').print_stats(20)"

# ROS2 performance
ros2 topic hz /cf2x_01/pose
ros2 topic bw /cf2x_01/pose
```

Resolution of these critical issues is **MANDATORY** before any production use of the DIAMANTS system.