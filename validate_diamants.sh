#!/bin/bash
# validate_diamants.sh — Quick end-to-end system check
# Usage: bash validate_diamants.sh

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
POS=$(timeout 5 ros2 topic echo /diamants/drones/positions --once --full-length 2>&1)
NDRONES=$(echo "$POS" | python3 -c "
import sys, json
for l in sys.stdin:
    l = l.strip()
    if l.startswith(\"data: '\"):
        d = json.loads(l[7:-1])
        print(len(d))
        break
" 2>/dev/null)
echo "4. Position data: ${NDRONES:-0}/8 drones broadcasting"

# 5. Altitude check
if [[ -n "$NDRONES" && "$NDRONES" -gt 0 ]]; then
    echo "$POS" | python3 -c "
import sys, json
for l in sys.stdin:
    l = l.strip()
    if l.startswith(\"data: '\"):
        d = json.loads(l[7:-1])
        alts = [v['y'] for v in d.values()]
        statuses = set(v['status'] for v in d.values())
        xs = [v['x'] for v in d.values()]
        zs = [v['z'] for v in d.values()]
        spread = max(max(xs)-min(xs), max(zs)-min(zs))
        print(f'5. Altitudes: min={min(alts):.3f} max={max(alts):.3f} avg={sum(alts)/len(alts):.3f}')
        print(f'6. Spread: {spread:.1f}m | Statuses: {statuses}')
        break
" 2>/dev/null
fi

# Verdict
echo ""
if [[ "$GZ" -ge 1 && "$BR" -ge 1 && "$MS" -ge 4 && "$API" == "200" && "$WS" -ge 1 && "$FE" == "200" && "$NDRONES" == "8" ]]; then
    echo "═══════════════════════════"
    echo "  ✅ ALL SYSTEMS NOMINAL"
    echo "═══════════════════════════"
    exit 0
else
    echo "═══════════════════════════"
    echo "  ❌ SOME SYSTEMS DOWN"
    echo "═══════════════════════════"
    exit 1
fi
