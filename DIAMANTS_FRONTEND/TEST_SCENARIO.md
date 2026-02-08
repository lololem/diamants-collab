# 🧪 Test Scenario — Multi-Drone Collaborative Scouting

## Objective
Validate the end-to-end multi-drone collaborative scouting system:
**Gazebo → ROS2 → WebSocket → 3D Frontend**

## Pre-requisites
| Component | Command | Expected |
|-----------|---------|----------|
| Gazebo Harmonic | `pgrep -f "gz sim"` | ≥1 process |
| ROS2 bridge | `pgrep -f parameter_bridge` | 1 process |
| Microservices | `pgrep -af diamants_microservices` | 4 nodes |
| API gateway | `curl -s localhost:8000` | HTTP 200 |
| WebSocket bridge | `ss -tlnp \| grep 8765` | listening |
| Frontend (Vite) | `curl -s localhost:3001` | HTTP 200 |

---

## Test 1 — Drone Count & IDs

**Criteria:** 8 drones visible with matching IDs across the entire pipeline.

```bash
# Check backend IDs
ros2 topic echo /diamants/drones/positions --once --full-length 2>&1 | \
  python3 -c "import sys,json;[print(k) for k in sorted(json.loads(next(l for l in sys.stdin if 'data:' in l).strip().split(\"'\")[1]).keys())]"
```

**Expected:** `crazyflie_01` through `crazyflie_08` (1-indexed, zero-padded)

**Frontend check:** Open browser console (F12), wait 10s, check:
```js
// Should print 8 matched drones every 5s
// Look for: "📡 Backend positions: N updates, 8/8 drones matched, 8 local drones"
```

| ✅ Pass | ❌ Fail |
|---------|---------|
| 8 IDs in backend, 8/8 matched in frontend | Missing drones or ID mismatch |

---

## Test 2 — Altitude Stability

**Criteria:** All drones at target altitude 0.5m ± 0.1m for >60s.

```bash
# Take 5 samples over 10 seconds
for i in $(seq 5); do
  ros2 topic echo /diamants/drones/positions --once --full-length 2>&1 | \
    python3 -c "
import sys,json
for l in sys.stdin:
  if 'data:' in l:
    d=json.loads(l.strip().split(\"'\")[1])
    alts=[v['y'] for v in d.values()]
    print(f'altitudes: min={min(alts):.3f} max={max(alts):.3f} avg={sum(alts)/len(alts):.3f}')
    break"
  sleep 2
done
```

| ✅ Pass | ❌ Fail |
|---------|---------|
| All altitudes in [0.40, 0.60] range | Any drone < 0.3 or > 0.8 |

---

## Test 3 — Coordinate Mapping (Gazebo ↔ Frontend)

**Criteria:** Frontend positions match Gazebo (no scale, correct axis swap).

**Mapping rules:**
- `frontend.x = gazebo.x` (forward)
- `frontend.y = gazebo.z` (altitude, must be positive)
- `frontend.z = -gazebo.y` (depth, negated)

```bash
# Compare raw Gazebo odom vs broadcast
echo "=== Gazebo raw (crazyflie = cf_01) ===" && \
  ros2 topic echo /crazyflie/odom --once 2>&1 | grep -A3 "position:" && \
echo "=== Broadcast (crazyflie_01) ===" && \
  ros2 topic echo /diamants/drones/positions --once --full-length 2>&1 | \
    python3 -c "import sys,json;[print(f'x={v[\"x\"]:.3f} y={v[\"y\"]:.3f} z={v[\"z\"]:.3f}') for k,v in json.loads(next(l for l in sys.stdin if 'data:' in l).strip().split(\"'\")[1]).items() if k=='crazyflie_01']"
```

| ✅ Pass | ❌ Fail |
|---------|---------|
| x matches, y = gz_z > 0, z ≈ -gz_y | Scale mismatch or axis wrong |

---

## Test 4 — Exploration Coverage

**Criteria:** Drones spread out and cover the zone progressively.

```bash
# Check spread after 60s of exploration
ros2 topic echo /diamants/drones/positions --once --full-length 2>&1 | \
  python3 -c "
import sys,json,math
for l in sys.stdin:
  if 'data:' in l:
    d=json.loads(l.strip().split(\"'\")[1])
    xs=[v['x'] for v in d.values()]
    zs=[v['z'] for v in d.values()]
    spread=max(max(xs)-min(xs), max(zs)-min(zs))
    print(f'Spread: {spread:.1f}m (x: {min(xs):.1f} to {max(xs):.1f}, z: {min(zs):.1f} to {max(zs):.1f})')
    pairs=[(i,j) for i in range(8) for j in range(i+1,8)]
    dists=[math.sqrt((xs[i]-xs[j])**2+(zs[i]-zs[j])**2) for i,j in pairs]
    print(f'Min inter-drone: {min(dists):.2f}m, Mean: {sum(dists)/len(dists):.2f}m')
    break"
```

| ✅ Pass (after 2 min) | ❌ Fail |
|---------|---------|
| Spread > 3m, min inter-drone > 0.3m | All drones clustered or colliding |

---

## Test 5 — Frontend 3D Visualization

**Criteria:** Drones visible and moving in 3D view.

Open `http://localhost:3001` in browser:

1. **Camera position:** Scene should show drones at ~0.5m altitude, camera close enough to see them clearly
2. **Drone count:** 8 distinct drone meshes visible
3. **Movement:** Drones visibly moving (watch for 10s)
4. **No tipping:** Drones remain level (not tilted > 30°)
5. **Console check (F12):**
   ```
   📡 Backend positions: N updates, 8/8 drones matched
   ```

| ✅ Pass | ❌ Fail |
|---------|---------|
| 8 drones flying, moving, visible in 3D | Missing meshes, frozen, or wrong scale |

---

## Test 6 — Full Scouting Mission (Optimal Time)

**Scenario:** 8 drones scouting a 5m × 5m zone, starting from center.

**Timeline expectations:**
| Time | Expected State |
|------|---------------|
| T+0s | Launch command sent |
| T+2-16s | Staggered takeoff (2s intervals) |
| T+20s | All drones at 0.5m altitude (cruise → explore) |
| T+25s | Exploration begins, drones spreading out |
| T+60s | Coverage should be growing, drones in different sectors |
| T+120s | Drones exploring outer areas (~2.5m from center) |
| T+300s | Significant zone coverage |

**Monitoring command:**
```bash
watch -n 2 'source /opt/ros/jazzy/setup.bash && source /tmp/diamants_build/install/setup.bash && ros2 topic echo /diamants/drones/positions --once --full-length 2>&1 | python3 -c "
import sys,json,math
for l in sys.stdin:
  if \"data:\" in l:
    d=json.loads(l.strip().split(chr(39))[1])
    xs=[v[\"x\"] for v in d.values()]; zs=[v[\"z\"] for v in d.values()]; ys=[v[\"y\"] for v in d.values()]
    statuses=set(v[\"status\"] for v in d.values())
    print(f\"Drones: {len(d)} | Alt: {sum(ys)/len(ys):.2f}m | Spread: {max(max(xs)-min(xs),max(zs)-min(zs)):.1f}m | Status: {statuses}\")
    break"'
```

---

## Quick Validation Script

```bash
#!/bin/bash
# validate_diamants.sh — Quick end-to-end check
echo "🔍 DIAMANTS End-to-End Validation"
echo "================================="

# 1. Backend processes
GZ=$(pgrep -c -f "gz sim" 2>/dev/null)
BR=$(pgrep -c -f "parameter_bridge" 2>/dev/null)
MS=$(pgrep -af "diamants_microservices" 2>/dev/null | wc -l)
echo "1. Gazebo: ${GZ:-0} | Bridge: ${BR:-0} | Microservices: ${MS:-0}/4"

# 2. API + WS
API=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:8000/ 2>/dev/null)
WS=$(ss -tlnp 2>/dev/null | grep -c 8765)
echo "2. API: HTTP ${API} | WebSocket: ${WS} listener(s)"

# 3. Frontend
FE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:3001/ 2>/dev/null)
echo "3. Frontend: HTTP ${FE}"

# 4. Position data
source /opt/ros/jazzy/setup.bash 2>/dev/null
source /tmp/diamants_build/install/setup.bash 2>/dev/null
POS=$(timeout 3 ros2 topic echo /diamants/drones/positions --once --full-length 2>&1)
NDRONES=$(echo "$POS" | python3 -c "import sys,json;print(len(json.loads(next(l for l in sys.stdin if 'data:' in l).strip().split(\"'\")[1])))" 2>/dev/null)
echo "4. Position data: ${NDRONES:-0}/8 drones broadcasting"

# Verdict
if [[ "$GZ" -ge 1 && "$BR" -ge 1 && "$MS" -ge 4 && "$API" == "200" && "$WS" -ge 1 && "$FE" == "200" && "$NDRONES" == "8" ]]; then
    echo "═══════════════════════════"
    echo "  ✅ ALL SYSTEMS NOMINAL"
    echo "═══════════════════════════"
else
    echo "═══════════════════════════"
    echo "  ❌ SOME SYSTEMS DOWN"
    echo "═══════════════════════════"
fi
```

---

## Architecture Reminder

```
Gazebo Harmonic (headless)
    ↓ /model/crazyflie*/odometry
ros_gz_bridge (parameter_bridge)
    ↓ /crazyflie*/odom (ROS2 nav_msgs/Odometry)
SwarmController → sends cmd_vel (PD altitude + sector exploration)
PositionBroadcaster → aggregates → /diamants/drones/positions (JSON)
    ↓ Coordinate mapping: x=gz_x, y=gz_z(alt), z=-gz_y
WebSocket Bridge (port 8765) → relays JSON
    ↓ CustomEvent: diamants:drone-positions
Frontend (Three.js, port 3001) → lerp to targetPosition → 3D render
```
