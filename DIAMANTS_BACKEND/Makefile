# ═══════════════════════════════════════════════════════════════════════
# DIAMANTS Backend — Makefile
# ═══════════════════════════════════════════════════════════════════════
# Convenient targets for building and launching the multi-drone SITL.
#
# Usage:
#   make setup             # Install deps + build everything
#   make build             # Build ROS2 packages only
#   make launch            # Headless, forest, 8 drones
#   make launch-gui        # With Gazebo 3D GUI
#   make launch-indoor     # Indoor warehouse world
#   make launch DRONES=2   # 2 drones only
#   make check             # Verify environment
#   make kill              # Kill all ROS2/Gazebo processes
#   make bridge            # Start WebSocket bridge + API only
# ═══════════════════════════════════════════════════════════════════════

SHELL := /bin/bash
.DEFAULT_GOAL := help

# ── Configurable parameters ──
DRONES    ?= 8
WORLD     ?= forest
ALTITUDE  ?= 0.5
RADIUS    ?= 20.0
HEADLESS  ?= True

# ── Paths ──
ROOT        := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
ROS_DISTRO  ?= jazzy
ROS_SETUP   := /opt/ros/$(ROS_DISTRO)/setup.bash
WS_DIR      := $(ROOT)/slam_collaboratif/ros2_ws
MICRO_DIR   := $(ROOT)/ros2_microservices
API_DIR     := $(ROOT)/../DIAMANTS_API

# ── Source chain ──
SOURCE_CMD := source $(ROS_SETUP) && \
              [ -f $(WS_DIR)/install/setup.bash ] && source $(WS_DIR)/install/setup.bash || true && \
              [ -f $(MICRO_DIR)/install/setup.bash ] && source $(MICRO_DIR)/install/setup.bash || true

.PHONY: help setup build check launch launch-gui launch-indoor launch-open kill bridge clean

help: ## Show this help
	@echo ""
	@echo "  DIAMANTS Backend — Multi-Drone SITL"
	@echo "  ──────────────────────────────────────"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-18s\033[0m %s\n", $$1, $$2}'
	@echo ""
	@echo "  Variables:"
	@echo "    DRONES=$(DRONES)  WORLD=$(WORLD)  ALTITUDE=$(ALTITUDE)  RADIUS=$(RADIUS)"
	@echo ""

setup: ## Install dependencies + build everything
	@chmod +x $(ROOT)/setup.sh
	@$(ROOT)/setup.sh

build: ## Build ROS2 packages only
	@chmod +x $(ROOT)/setup.sh
	@$(ROOT)/setup.sh --build

check: ## Verify environment is ready
	@chmod +x $(ROOT)/setup.sh
	@$(ROOT)/setup.sh --check

launch: ## Launch SITL (headless by default)
	@echo ""
	@echo "  Launching DIAMANTS SITL"
	@echo "  World: $(WORLD) | Drones: $(DRONES) | Headless: $(HEADLESS)"
	@echo ""
	@bash -c '$(SOURCE_CMD) && \
		ros2 launch diamants_microservices diamants_full.launch.py \
			headless:=$(HEADLESS) \
			world:=$(WORLD) \
			num_drones:=$(DRONES) \
			target_altitude:=$(ALTITUDE) \
			exploration_radius:=$(RADIUS)'

launch-gui: ## Launch with Gazebo 3D GUI
	@$(MAKE) launch HEADLESS=False

launch-indoor: ## Launch indoor warehouse world
	@$(MAKE) launch WORLD=indoor

launch-open: ## Launch empty open world (original)
	@$(MAKE) launch WORLD=open

bridge: ## Start WebSocket bridge + REST API only (no Gazebo)
	@echo "  Starting DiamantsBridge (WS:8765) + API (REST:8000)..."
	@bash -c '$(SOURCE_CMD) && cd $(API_DIR) && python3 launcher.py'

kill: ## Kill all ROS2, Gazebo, and bridge processes
	@echo "  Stopping all DIAMANTS processes..."
	@-pkill -f "ros2" 2>/dev/null || true
	@-pkill -f "gz sim" 2>/dev/null || true
	@-pkill -f "ruby.*gz" 2>/dev/null || true
	@-pkill -f "parameter_bridge" 2>/dev/null || true
	@-pkill -f "launcher.py" 2>/dev/null || true
	@-pkill -f "websocket_bridge" 2>/dev/null || true
	@echo "  Done."

clean: ## Remove build artifacts
	@echo "  Cleaning build artifacts..."
	@rm -rf $(WS_DIR)/build $(WS_DIR)/install $(WS_DIR)/log
	@rm -rf $(MICRO_DIR)/build $(MICRO_DIR)/install $(MICRO_DIR)/log
	@echo "  Done."
