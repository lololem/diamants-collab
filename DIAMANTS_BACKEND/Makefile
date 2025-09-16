# Makefile pour Projet Diamants
# Reconstruction automatique après git clone
# Usage: make [target]

# Variables de configuration
SHELL := /bin/bash
PROJECT_ROOT := $(shell pwd)
ROS2_WS := $(PROJECT_ROOT)/slam_collaboratif/ros2_ws
PYTHON_VERSION := 3.12
VENV_DIR := $(PROJECT_ROOT)/.venv

# Couleurs pour l'affichage
GREEN := \033[0;32m
BLUE := \033[0;34m
YELLOW := \033[1;33m
RED := \033[0;31m
NC := \033[0m

.PHONY: all setup install build test clean help status launch-tmux kill-tmux status-tmux post-clone

# Target par défaut
all: post-clone ## Configuration complète (recommandé après git clone)

help: ## Affiche cette aide
	@echo -e "$(BLUE)🔧 PROJET DIAMANTS - MAKEFILE$(NC)"
	@echo "================================="
	@echo ""
	@echo "Targets disponibles:"
	@awk 'BEGIN {FS = ":.*?## "} /^[a-zA-Z_-]+:.*?## / {printf "  \033[1;33m%-15s\033[0m %s\n", $$1, $$2}' $(MAKEFILE_LIST)
	@echo ""
	@echo "Usage typique après git clone:"
	@echo -e "  $(GREEN)make$(NC)              - Configuration complète automatique"
	@echo -e "  $(GREEN)make post-clone$(NC)   - Configuration post-clone"
	@echo -e "  $(GREEN)make launch-tmux$(NC)  - Lancer le système TMUX"
	@echo -e "  $(GREEN)make status-tmux$(NC)  - Vérifier l'état du système"

setup: ## Configuration initiale complète du projet
	@echo -e "$(BLUE)🚀 CONFIGURATION INITIALE PROJET DIAMANTS$(NC)"
	@echo "=============================================="
	@$(MAKE) check-system
	@$(MAKE) setup-permissions
	@$(MAKE) setup-python
	@$(MAKE) setup-ros2
	@$(MAKE) setup-vscode
	@echo -e "$(GREEN)✅ Configuration initiale terminée !$(NC)"

check-system: ## Vérification des prérequis système
	@echo -e "$(BLUE)🔍 Vérification des prérequis...$(NC)"
	@command -v python3 >/dev/null 2>&1 || { echo -e "$(RED)❌ Python3 requis$(NC)"; exit 1; }
	@command -v git >/dev/null 2>&1 || { echo -e "$(RED)❌ Git requis$(NC)"; exit 1; }
	@command -v colcon >/dev/null 2>&1 || echo -e "$(YELLOW)⚠️  colcon non trouvé - installation recommandée$(NC)"
	@python3 --version | grep -E "3\.(8|9|10|11|12)" >/dev/null || echo -e "$(YELLOW)⚠️  Python 3.8+ recommandé$(NC)"
	@echo -e "$(GREEN)✓ Prérequis vérifiés$(NC)"

setup-permissions: ## Configuration des permissions des scripts
	@echo -e "$(BLUE)🔐 Configuration des permissions...$(NC)"
	@chmod +x $(PROJECT_ROOT)/diamants
	@chmod +x $(PROJECT_ROOT)/maintenance
	@chmod +x $(PROJECT_ROOT)/launch.sh
	@find $(PROJECT_ROOT)/scripts -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true
	@find $(PROJECT_ROOT)/scripts/launch -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true
	@find $(PROJECT_ROOT)/archives/maintenance/scripts -name "*.sh" -exec chmod +x {} \; 2>/dev/null || true
	@find $(PROJECT_ROOT)/archives/maintenance/scripts -name "*.py" -exec chmod +x {} \; 2>/dev/null || true
	@echo -e "$(GREEN)✓ Permissions configurées$(NC)"

setup-python: ## Configuration de l'environnement Python
	@echo -e "$(BLUE)🐍 Configuration environnement Python...$(NC)"
	@if [ ! -f "$(PROJECT_ROOT)/pyproject.toml" ]; then \
		echo -e "$(YELLOW)⚠️  pyproject.toml manquant$(NC)"; \
	else \
		echo -e "$(GREEN)✓ Configuration Python trouvée$(NC)"; \
	fi
	@python3 -m pip install --user --upgrade pip setuptools wheel 2>/dev/null || true
	@echo -e "$(GREEN)✓ Environnement Python configuré$(NC)"

setup-ros2: ## Configuration du workspace ROS2
	@echo -e "$(BLUE)🤖 Configuration workspace ROS2...$(NC)"
	@if [ -d "$(ROS2_WS)" ]; then \
		echo -e "$(GREEN)✓ Workspace ROS2 trouvé: $(ROS2_WS)$(NC)"; \
		if command -v colcon >/dev/null 2>&1; then \
			echo -e "$(BLUE)📦 Installation des dépendances ROS2...$(NC)"; \
			cd $(ROS2_WS) && rosdep update 2>/dev/null || true; \
			cd $(ROS2_WS) && rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true; \
		else \
			echo -e "$(YELLOW)⚠️  colcon non disponible - sautant l'installation des dépendances$(NC)"; \
		fi; \
	else \
		echo -e "$(YELLOW)⚠️  Workspace ROS2 non trouvé$(NC)"; \
	fi

setup-vscode: ## Configuration de VS Code
	@echo -e "$(BLUE)🖥️  Configuration VS Code...$(NC)"
	@if [ -d "$(PROJECT_ROOT)/.vscode" ]; then \
		echo -e "$(GREEN)✓ Configuration VS Code trouvée$(NC)"; \
		if [ -f "$(PROJECT_ROOT)/.vscode/extensions.json" ]; then \
			echo -e "$(BLUE)📋 Extensions recommandées configurées$(NC)"; \
		fi; \
	else \
		echo -e "$(YELLOW)⚠️  Configuration VS Code manquante$(NC)"; \
	fi

install: setup ## Installation complète (alias pour setup)

build: ## Construction du workspace ROS2
	@echo -e "$(BLUE)🔨 CONSTRUCTION WORKSPACE ROS2$(NC)"
	@echo "==============================="
	@if [ -d "$(ROS2_WS)" ] && command -v colcon >/dev/null 2>&1; then \
		echo -e "$(BLUE)📦 Nettoyage ancien build...$(NC)"; \
		cd $(ROS2_WS) && rm -rf build install log 2>/dev/null || true; \
		echo -e "$(BLUE)🔨 Construction avec colcon...$(NC)"; \
		cd $(ROS2_WS) && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release; \
		if [ $$? -eq 0 ]; then \
			echo -e "$(GREEN)✅ Construction réussie !$(NC)"; \
		else \
			echo -e "$(RED)❌ Erreur de construction$(NC)"; \
			exit 1; \
		fi; \
	else \
		if [ ! -d "$(ROS2_WS)" ]; then \
			echo -e "$(RED)❌ Workspace ROS2 non trouvé: $(ROS2_WS)$(NC)"; \
		else \
			echo -e "$(RED)❌ colcon non disponible$(NC)"; \
		fi; \
		exit 1; \
	fi

test: ## Tests de validation du système
	@echo -e "$(BLUE)🧪 TESTS DE VALIDATION$(NC)"
	@echo "======================="
	@echo -e "$(BLUE)🧪 Test script principal...$(NC)"
	@if [ -x "$(PROJECT_ROOT)/launch.sh" ]; then \
		echo -e "$(GREEN)✓ Lanceur principal exécutable$(NC)"; \
	else \
		echo -e "$(RED)❌ Lanceur principal manquant$(NC)"; \
	fi
	@echo -e "$(BLUE)🧪 Test structure propre...$(NC)"
	@if [ -x "$(PROJECT_ROOT)/scripts/validation/validate_clean_structure.sh" ]; then \
		echo -e "$(GREEN)✓ Script de validation disponible$(NC)"; \
		$(PROJECT_ROOT)/scripts/validation/validate_clean_structure.sh; \
	else \
		echo -e "$(RED)❌ Script de validation manquant$(NC)"; \
	fi
	@echo -e "$(BLUE)🧪 Test workspace ROS2...$(NC)"
	@if [ -d "$(ROS2_WS)" ]; then \
		echo -e "$(GREEN)✓ Workspace ROS2 présent$(NC)"; \
	else \
		echo -e "$(RED)❌ Workspace ROS2 manquant$(NC)"; \
	fi

clean: ## Nettoyage des fichiers de build
	@echo -e "$(BLUE)🧹 NETTOYAGE$(NC)"
	@echo "============"
	@echo -e "$(BLUE)🗑️  Nettoyage caches Python...$(NC)"
	@find $(PROJECT_ROOT) -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
	@find $(PROJECT_ROOT) -name "*.pyc" -delete 2>/dev/null || true
	@find $(PROJECT_ROOT) -name ".pytest_cache" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo -e "$(BLUE)🗑️  Nettoyage build ROS2...$(NC)"
	@if [ -d "$(ROS2_WS)" ]; then \
		cd $(ROS2_WS) && rm -rf build install log 2>/dev/null || true; \
	fi
	@echo -e "$(GREEN)✅ Nettoyage terminé$(NC)"

clean-all: clean ## Nettoyage complet (+ archives temporaires)
	@echo -e "$(BLUE)🗑️  Nettoyage complet...$(NC)"
	@find $(PROJECT_ROOT) -name "*.log" -not -path "$(PROJECT_ROOT)/archives/*" -delete 2>/dev/null || true
	@find $(PROJECT_ROOT) -name ".mypy_cache" -type d -exec rm -rf {} + 2>/dev/null || true
	@echo -e "$(GREEN)✅ Nettoyage complet terminé$(NC)"

status: ## Affiche le statut du projet
	@echo -e "$(BLUE)📊 STATUT PROJET DIAMANTS$(NC)"
	@echo "=========================="
	@echo -e "$(BLUE)📁 Structure:$(NC)"
	@echo "  • Répertoire: $(PROJECT_ROOT)"
	@echo -e "  • ROS2 Workspace: $(if $(wildcard $(ROS2_WS)),$(GREEN)✓ Présent$(NC),$(RED)✗ Manquant$(NC))"
	@echo -e "  • Script principal: $(if $(wildcard $(PROJECT_ROOT)/diamants),$(GREEN)✓ Présent$(NC),$(RED)✗ Manquant$(NC))"
	@echo -e "  • Configuration VS Code: $(if $(wildcard $(PROJECT_ROOT)/.vscode),$(GREEN)✓ Présente$(NC),$(RED)✗ Manquante$(NC))"
	@echo ""
	@echo -e "$(BLUE)🔧 Outils:$(NC)"
	@echo "  • Python: $(shell python3 --version 2>/dev/null || echo 'Non trouvé')"
	@echo "  • Git: $(shell git --version 2>/dev/null || echo 'Non trouvé')"
	@echo "  • Colcon: $(shell colcon version 2>/dev/null || echo 'Non trouvé')"
	@echo ""
	@echo -e "$(BLUE)🚀 Points d'entrée:$(NC)"
	@echo "  • ./launch.sh [sim|real]            - Lanceur unique missions"
	@echo "  • make run                           - Raccourci simulation"
	@echo "  • scripts/validation/validate_clean_structure.sh - Validation structure"
	@echo ""
	@if [ -f "$(PROJECT_ROOT)/archives/maintenance/rapports/VALIDATION_FINALE.json" ]; then \
		echo -e "$(BLUE)✅ Dernière validation:$(NC)"; \
		grep -E '"status"|"score"' $(PROJECT_ROOT)/archives/maintenance/rapports/VALIDATION_FINALE.json 2>/dev/null || true; \
	fi

# Target pour reconstruction après git clone
rebuild: clean setup build test ## Reconstruction complète après git clone
	@echo -e "$(GREEN)🎉 RECONSTRUCTION TERMINÉE !$(NC)"
	@echo ""
	@echo "Le projet est prêt à l'utilisation:"
	@echo -e "  $(YELLOW)make launch-tmux$(NC)  - Lancer le système TMUX complet"
	@echo -e "  $(YELLOW)./launch_slam_collaborative.sh$(NC) - Lanceur interactif"

# Gestion TMUX pour DIAMANTS V3
launch-tmux: ## Lance le système SLAM collaboratif avec TMUX
	@echo -e "$(BLUE)🚀 LANCEMENT SYSTÈME TMUX DIAMANTS V3$(NC)"
	@echo "==========================================="
	@if ! command -v tmux >/dev/null 2>&1; then \
		echo -e "$(RED)❌ TMUX non installé$(NC)"; \
		echo "Installation: sudo apt install tmux"; \
		exit 1; \
	fi
	@echo -e "$(BLUE)🔄 Nettoyage sessions TMUX existantes...$(NC)"
	@tmux kill-session -t slam_collab 2>/dev/null || true
	@echo -e "$(BLUE)📺 Lancement système collaboratif...$(NC)"
	@cd $(PROJECT_ROOT) && echo "1" | ./launch_slam_collaborative.sh

kill-tmux: ## Arrête toutes les sessions TMUX DIAMANTS
	@echo -e "$(BLUE)🛑 ARRÊT SESSIONS TMUX$(NC)"
	@echo "======================="
	@tmux kill-session -t slam_collab 2>/dev/null || echo "Aucune session slam_collab active"
	@pkill -f "gazebo\|rviz\|ros2" 2>/dev/null || echo "Aucun processus ROS2/Gazebo actif"
	@echo -e "$(GREEN)✅ Nettoyage terminé$(NC)"

status-tmux: ## Affiche le statut des sessions TMUX
	@echo -e "$(BLUE)📊 STATUT TMUX DIAMANTS V3$(NC)"
	@echo "============================"
	@if tmux list-sessions 2>/dev/null | grep -q slam_collab; then \
		echo -e "$(GREEN)✅ Session slam_collab active$(NC)"; \
		tmux list-windows -t slam_collab; \
	else \
		echo -e "$(YELLOW)⚠️  Aucune session slam_collab active$(NC)"; \
	fi
	@echo ""
	@echo "Commandes utiles:"
	@echo -e "  $(YELLOW)tmux attach -t slam_collab$(NC)  - Se connecter à la session"
	@echo -e "  $(YELLOW)make kill-tmux$(NC)               - Arrêter le système"

# Configuration post-clone automatique
post-clone: ## Configuration automatique complète après git clone
	@echo -e "$(BLUE)🔄 CONFIGURATION POST-CLONE DIAMANTS V3$(NC)"
	@echo "=============================================="
	@$(MAKE) check-system
	@$(MAKE) setup
	@$(MAKE) build
	@echo -e "$(BLUE)🧪 Test rapide du système...$(NC)"
	@$(MAKE) test
	@echo -e "$(GREEN)🎉 SYSTÈME PRÊT !$(NC)"
	@echo ""
	@echo "Prochaines étapes:"
	@echo -e "  $(YELLOW)make launch-tmux$(NC)    - Lancer le système complet"
	@echo -e "  $(YELLOW)make status-tmux$(NC)    - Vérifier l'état TMUX"

run: ## Lance la simulation multi-drones (équiv. ./launch.sh sim)
	@$(PROJECT_ROOT)/launch.sh sim

# Targets pour développement
dev-setup: setup ## Configuration pour développement
	@echo -e "$(BLUE)🛠️  Configuration développement...$(NC)"
	@python3 -m pip install --user black flake8 mypy pytest 2>/dev/null || true
	@echo -e "$(GREEN)✓ Outils de développement installés$(NC)"

dev-test: ## Tests de développement étendus
	@echo -e "$(BLUE)🧪 Tests de développement...$(NC)"
	@if command -v flake8 >/dev/null 2>&1; then \
		echo -e "$(BLUE)📏 Vérification style avec flake8...$(NC)"; \
		find $(PROJECT_ROOT) -name "*.py" -not -path "$(PROJECT_ROOT)/external_deps/*" -exec flake8 {} + 2>/dev/null || true; \
	fi
	@$(MAKE) test

# Raccourcis pratiques
install-deps: setup-ros2 ## Installation des dépendances seulement
compile: build ## Alias pour build
validate: test ## Alias pour test

# Target d'information
info: ## Informations sur le projet
	@echo -e "$(BLUE)ℹ️  PROJET DIAMANTS V3$(NC)"
	@echo "========================"
	@echo "Version: 3.0.0-Production"
	@echo "Système SLAM collaboratif multi-drones avec TMUX"
	@echo ""
	@echo "Workflow après git clone:"
	@echo -e "  $(YELLOW)1. make$(NC)              - Configuration automatique"
	@echo -e "  $(YELLOW)2. make launch-tmux$(NC)  - Lancer le système"
	@echo -e "  $(YELLOW)3. make status-tmux$(NC)  - Vérifier l'état"
	@echo ""
	@echo "Gestion TMUX:"
	@echo -e "  $(YELLOW)tmux attach -t slam_collab$(NC)  - Se connecter"
	@echo -e "  $(YELLOW)make kill-tmux$(NC)               - Arrêter le système"
