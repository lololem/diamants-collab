#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════════════
# DIAMANTS Backend — Setup Script
# ═══════════════════════════════════════════════════════════════════════
# Installs dependencies, builds all ROS2 packages, and verifies the
# environment for multi-drone SITL with Gazebo Harmonic.
#
# Usage:
#   ./setup.sh          # Full setup (deps + build)
#   ./setup.sh --build  # Build only (skip apt install)
#   ./setup.sh --check  # Verify environment only
# ═══════════════════════════════════════════════════════════════════════
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/slam_collaboratif/ros2_ws"
MICROSERVICES_DIR="$SCRIPT_DIR/ros2_microservices"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log()   { echo -e "${CYAN}[DIAMANTS]${NC} $*"; }
ok()    { echo -e "${GREEN}[  OK  ]${NC} $*"; }
warn()  { echo -e "${YELLOW}[ WARN ]${NC} $*"; }
fail()  { echo -e "${RED}[FAIL  ]${NC} $*"; }

# ── Determine ROS2 distro ──
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"

check_env() {
    log "Checking environment..."
    local errors=0

    # ROS2
    if [ -f "$ROS_SETUP" ]; then
        ok "ROS2 $ROS_DISTRO found"
    else
        fail "ROS2 $ROS_DISTRO not found at $ROS_SETUP"
        ((errors++))
    fi

    # Gazebo
    if command -v gz &>/dev/null; then
        ok "Gazebo Harmonic: $(gz sim --version 2>/dev/null | head -1 || echo 'installed')"
    else
        fail "Gazebo (gz) not found"
        ((errors++))
    fi

    # ros_gz_bridge
    if ros2 pkg list 2>/dev/null | grep -q ros_gz_bridge; then
        ok "ros_gz_bridge available"
    elif dpkg -l 2>/dev/null | grep -q "ros-${ROS_DISTRO}-ros-gz-bridge"; then
        ok "ros_gz_bridge installed (source ROS2 first)"
    else
        warn "ros_gz_bridge not found — install with: sudo apt install ros-${ROS_DISTRO}-ros-gz-bridge"
        ((errors++))
    fi

    # ros_gz_sim
    if ros2 pkg list 2>/dev/null | grep -q ros_gz_sim; then
        ok "ros_gz_sim available"
    elif dpkg -l 2>/dev/null | grep -q "ros-${ROS_DISTRO}-ros-gz-sim"; then
        ok "ros_gz_sim installed (source ROS2 first)"
    else
        warn "ros_gz_sim not found — install with: sudo apt install ros-${ROS_DISTRO}-ros-gz-sim"
        ((errors++))
    fi

    # Python websockets
    if python3 -c "import websockets" 2>/dev/null; then
        ok "Python websockets module"
    else
        warn "Python websockets not installed — pip install websockets"
        ((errors++))
    fi

    # Workspace built?
    if [ -d "$WS_DIR/install" ]; then
        local pkg_count
        pkg_count=$(ls -1 "$WS_DIR/install/" 2>/dev/null | grep -v "setup\." | wc -l)
        ok "Crazyflie workspace: $pkg_count packages installed"
    else
        warn "Crazyflie workspace not built yet"
        ((errors++))
    fi

    if [ $errors -eq 0 ]; then
        echo ""
        ok "Environment ready for DIAMANTS SITL"
    else
        echo ""
        warn "$errors issue(s) found — run './setup.sh' to fix"
    fi
    return $errors
}

install_deps() {
    log "Installing system dependencies..."
    sudo apt update -qq

    local pkgs=(
        "ros-${ROS_DISTRO}-ros-gz-sim"
        "ros-${ROS_DISTRO}-ros-gz-bridge"
        "ros-${ROS_DISTRO}-ros-gz-interfaces"
        "python3-websockets"
    )

    for pkg in "${pkgs[@]}"; do
        if dpkg -l "$pkg" &>/dev/null; then
            ok "$pkg already installed"
        else
            log "Installing $pkg..."
            sudo apt install -y -qq "$pkg" && ok "$pkg installed" || warn "Failed to install $pkg"
        fi
    done
}

build_workspace() {
    log "Building Crazyflie ROS2 workspace..."
    source "$ROS_SETUP"

    # 1. Build slam_collaboratif workspace (Crazyflie models, bridge configs)
    if [ -d "$WS_DIR/src" ]; then
        cd "$WS_DIR"
        colcon build --symlink-install --parallel-workers "$(nproc)" 2>&1 | tail -5
        ok "Crazyflie workspace built"
        source "$WS_DIR/install/setup.bash"
    else
        warn "No workspace source at $WS_DIR/src — skipping"
    fi

    # 2. Build diamants_microservices (standalone)
    if [ -d "$MICROSERVICES_DIR" ]; then
        cd "$MICROSERVICES_DIR"
        # Build as an overlay on the crazyflie workspace
        colcon build --symlink-install 2>&1 | tail -5
        ok "diamants_microservices built"
    else
        warn "No microservices at $MICROSERVICES_DIR — skipping"
    fi

    cd "$SCRIPT_DIR"
    log "Build complete."
}

# ── Main ──
case "${1:-}" in
    --check)
        check_env
        ;;
    --build)
        source "$ROS_SETUP" 2>/dev/null || true
        build_workspace
        ;;
    *)
        echo ""
        echo "═══════════════════════════════════════════════"
        echo "  DIAMANTS Backend Setup"
        echo "  ROS2 $ROS_DISTRO + Gazebo Harmonic"
        echo "═══════════════════════════════════════════════"
        echo ""
        install_deps
        echo ""
        build_workspace
        echo ""
        check_env || true
        echo ""
        log "Setup complete. Launch with:"
        echo "  make launch              # Headless, forest world"
        echo "  make launch-gui          # With Gazebo GUI"
        echo "  make launch-indoor       # Indoor warehouse"
        echo "  make launch DRONES=2     # 2 drones only"
        ;;
esac
