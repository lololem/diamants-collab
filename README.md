# 💎 DIAMANTS: Distributed Autonomous Multi-Agent Systems

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/) [![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/) [![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE) [![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)

**DIAMANTS is an open-source platform for the simulation and execution of distributed intelligence. Our philosophy: code once, and deploy everywhere—in simulation and in the real world.**

This project is a 'playground' for developers, creating a robust bridge between high-level intent (the swarm's strategy) and low-level execution (each drone's physical commands). It's a space to code distributed intelligence, test it in a credible simulation, and push it directly to real-world swarms.

## 🎯 Vision

DIAMANTS's objective is to tackle a major technical challenge: achieving emergent collective behaviors in open, modular, and interoperable code. We aim to demonstrate that distributed intelligence is not a theoretical concept, but a robust, documented, and reusable software artifact for the community.

## ✨ Key Features & Technical Challenges

### 🌱 Open Source by Design

Our entire ecosystem is built on recognized open standards. No proprietary lock-in.

**Main Stack:**
- **Backend & Simulation**: ROS 2, Gazebo, PX4/ArduPilot, MAVROS
- **Frontend & Visualization**: Vite, Three.js, Babylon.js, WebGL, Node.js

**Goal**: To foster an ecosystem where every developer can contribute, test, and enrich the platform.

### 🌐 Radical Interoperability

A defined mission must be understood by any agent, regardless of its hardware or middleware.
- **Universal Missions**: Mission definitions in YAML/JSON
- **Unified APIs**: Design of agnostic interfaces and mission translators

### 🧠 From Simple Rules to Collective Intelligence

The scientific core of the project is the study of emergence from simple interactions.
- **Explored Algorithms**: Stigmergy, consensus algorithms, social forces (Lennard-Jones type)
- **Challenge**: To code simple behavioral bricks and observe the emergence of a credible collective intelligence

### 🔄 Seamless Sim-to-Real Workflow

Simulation is a mirror of reality. Zero throwaway code.
- **High-Fidelity Simulation**: The high-level code produces command streams via ROS 2 that are translated identically for the simulator and for physical agents
- **Continuous Validation**: Every line of code is validated in simulation and then executed without any modification on the real hardware

## 🚀 Quick Start � DIAMANTS: Distributed Autonomous Multi## 🚀 Quick Start

### Prerequisites
- **Ubuntu 24.04** (recommended)
- **ROS2 Jazzy** ([installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Python 3.12+**
- **Node.js 16+**

### Installation

```bash
# Clone the collaborative repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# One-command launch (interactive menu)
./launch_diamants.sh
```

## 🔧 Component-Specific Launch

You can run each system autonomously using dedicated scripts:

### 📡 DIAMANTS_API - FastAPI Service & WebSocket Bridge
```bash
cd DIAMANTS_API
./setup.sh        # First-time setup
./start.sh        # Launch API service
./status.sh       # Check service status
./stop.sh         # Stop service
```

### 🤖 DIAMANTS_BACKEND - ROS2 SLAM System
```bash
cd DIAMANTS_BACKEND
./setup.sh                          # Install ROS2 dependencies
./launch_slam_collaborative.sh      # Start collaborative SLAM
# Monitor logs: tail -f logs/diamants_tmux_slam_collab_journal.log
```

### 🌐 DIAMANTS_FRONTEND - 3D Visualization Interface
```bash
cd DIAMANTS_FRONTEND/Mission_system
npm install       # Install dependencies
npm run dev       # Start development server (http://localhost:5173)
npm run build     # Production build
```

## 🤝 Collaborative Development

**DIAMANTS is actively seeking contributors!** 

We need help with:
- 🐛 **Bug Fixes**: System stability and edge cases
- 🚀 **Performance Optimization**: Multi-agent coordination efficiency
- 📚 **Documentation**: User guides and API documentation
- 🧪 **Testing**: Unit tests and integration testing
- 🎨 **UI/UX**: Frontend interface improvements
- 🤖 **Algorithm Development**: Advanced swarm intelligence behaviors

### 🛠️ Development Workflow
1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes and test thoroughly
4. Submit a pull request with detailed description

### 🆘 Report Issues
Found a bug or have a feature request? Please [create an issue](https://github.com/lololem/diamants-collab/issues) with:
- Detailed description of the problem
- Steps to reproduce
- System environment (Ubuntu version, ROS2 distribution, etc.)
- Expected vs actual behavior[ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/) [![Python](https://img.shields.io/badge/Python-3.12-blue.svg)](https://www.python.org/) [![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE) [![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-green.svg)](https://fastapi.tiangolo.com/)

**DIAMANTS is an open-source platform for the simulation and execution of distributed intelligence. Our philosophy: code once, and deploy everywhere—in simulation and in the real world.**

This project is a 'playground' for developers, creating a robust bridge between high-level intent (the swarm's strategy) and low-level execution (each drone's physical commands). It's a space to code distributed intelligence, test it in a credible simulation, and push it directly to real-world swarms.

## 🎯 Vision

DIAMANTS's objective is to tackle a major technical challenge: achieving emergent collective behaviors in open, modular, and interoperable code. We aim to demonstrate that distributed intelligence is not a theoretical concept, but a robust, documented, and reusable software artifact for the community.

## ✨ Key Features & Technical Challenges

### 🌱 Open Source by Design

Our entire ecosystem is built on recognized open standards. No proprietary lock-in.

**Main Stack:**
- **Backend & Simulation**: ROS 2, Gazebo, PX4/ArduPilot, MAVROS
- **Frontend & Visualization**: Vite, Three.js, Babylon.js, WebGL, Node.js

**Goal**: To foster an ecosystem where every developer can contribute, test, and enrich the platform.

### 🌐 Radical Interoperability

A defined mission must be understood by any agent, regardless of its hardware or middleware.
- **Universal Missions**: Mission definitions in YAML/JSON
- **Unified APIs**: Design of agnostic interfaces and mission translators

### 🧠 From Simple Rules to Collective Intelligence

The scientific core of the project is the study of emergence from simple interactions.
- **Explored Algorithms**: Stigmergy, consensus algorithms, social forces (Lennard-Jones type)
- **Challenge**: To code simple behavioral bricks and observe the emergence of a credible collective intelligence

### 🔄 Seamless Sim-to-Real Workflow

Simulation is a mirror of reality. Zero throwaway code.
- **High-Fidelity Simulation**: The high-level code produces command streams via ROS 2 that are translated identically for the simulator and for physical agents
- **Continuous Validation**: Every line of code is validated in simulation and then executed without any modification on the real hardware

## � Quick Start

### Prerequisites
- **Ubuntu 24.04** (recommended)
- **ROS2 Jazzy** ([installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- **Python 3.12+**
- **Node.js 16+**

### Installation

```bash
# Clone the collaborative repository
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab

# One-command launch (interactive menu)
./launch_diamants.sh
```

### Component Launch

```bash
# Complete system (recommended)
./launch_diamants.sh  # Select option 6: "Complete System"

# Individual components
./launch_diamants.sh  # Option 1: Backend + SLAM
./launch_diamants.sh  # Option 2: API Service  
./launch_diamants.sh  # Option 3: Frontend Interface
```

## 🏗️ Architecture Overview

### System Components

```
DIAMANTS System Architecture
├── 🔧 DIAMANTS_API/          # FastAPI + WebSocket Service
│   ├── api/                   # REST API endpoints
│   ├── services/             # WebSocket & ROS2 integration
│   └── launcher.py           # Unified entry point
├── 🤖 DIAMANTS_BACKEND/      # ROS2 SLAM System
│   ├── slam_collaboratif/    # Multi-agent SLAM workspace
│   ├── config/               # ROS2 configurations
│   └── launch_slam_collaborative.sh
├── 🎮 DIAMANTS_FRONTEND/     # 3D Visualization Interface
│   └── Mission_system/       # WebGL application
└── 📋 Root Scripts/          # System management
    ├── launch_diamants.sh    # Interactive launcher
    ├── stop_diamants.sh      # Clean shutdown
    └── check_ros_processes.sh # System diagnostics
```

## 🌟 Key Features

### 🧠 **Collaborative Intelligence**
- **Stigmergy-based coordination**: Bio-inspired pheromone communication
- **Real-time map fusion**: Dynamic merging of individual SLAM maps
- **Distributed decision making**: Autonomous task allocation and path planning

### � **Advanced Simulation**
- **Authentic physics**: Accurate Crazyflie 2.0 flight dynamics
- **Provençal environment**: Mediterranean forest with procedural generation
- **Real-time rendering**: 60fps WebGL visualization with advanced shaders

### 🔧 **Production Ready**
- **Automated orchestration**: TMUX-based multi-component management
- **Comprehensive testing**: pytest suite with ROS2 integration
- **Type safety**: Full Pylance support with conditional imports
- **Monitoring tools**: Real-time diagnostics and performance metrics

## 📡 Service Endpoints

| Component | URL | Description |
|-----------|-----|-------------|
| **API Documentation** | `http://localhost:8000/docs` | Interactive Swagger UI |
| **WebSocket Service** | `ws://localhost:8765` | Real-time drone communication |
| **3D Interface** | `http://localhost:5550` | Mission control interface |
| **RViz Visualization** | ROS2 launch | SLAM map visualization |

## �️ Development

### Environment Setup
```bash
# Configure Python environment
cd DIAMANTS_API
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Build ROS2 workspace
cd DIAMANTS_BACKEND/slam_collaboratif/ros2_ws
colcon build

# Install frontend dependencies
cd DIAMANTS_FRONTEND/Mission_system
npm install
```

### Testing
```bash
# API tests
cd DIAMANTS_API
python -m pytest tests/ -v

# System integration test
./check_ros_processes.sh

# Frontend development
cd DIAMANTS_FRONTEND/Mission_system
npm run dev
```

## 📚 Documentation

- **[API Reference](DIAMANTS_API/README.md)** - FastAPI service documentation
- **[Backend Guide](DIAMANTS_BACKEND/README.md)** - ROS2 SLAM system setup
- **[Frontend Manual](DIAMANTS_FRONTEND/Mission_system/README.md)** - 3D interface development
- **[Contributing Guidelines](CONTRIBUTING.md)** - Development workflow

## 🚦 System Status

### ✅ Architecture v2.0 - Production Ready

**Major Improvements:**
- ✅ **Unified API**: Single entry point with FastAPI + WebSocket Service
- ✅ **Centralized configuration**: Dataclass replaces Pydantic for reliability
- ✅ **Complete testing**: pytest suite with ROS2 support and mocks
- ✅ **Type checking**: Full Pylance support with conditional imports
- ✅ **Clean architecture**: Eliminated duplications, 6 essential root files
- ✅ **Optimized performance**: Single WebSocket Service instead of multiple bridges

| Before v2.0 | After v2.0 | Benefit |
|-------------|-------------|---------|
| ❌ Multiple WebSocket bridges | ✅ Unified Service | Performance + Simplicity |
| ❌ Pydantic BaseSettings | ✅ Dataclass config | Reliability + Type safety |
| ❌ Untyped imports | ✅ Complete type checking | Robust development |  
| ❌ Scattered scripts | ✅ 6 essential files | Simplified maintenance |
| ❌ Fragmented tests | ✅ Unified pytest suite | Quality assurance |

## 🆘 Troubleshooting

### Common Issues

#### 🔌 Service Status Check
```bash
./check_ros_processes.sh       # Complete system diagnostics
cd DIAMANTS_API/
./check.sh                           # Statut rapide unifié

# Diagnostic détaillé
./scripts/maintenance/diagnose.sh    # Analyse complète avec logs

# Redémarrage propre
./stop.sh && sleep 2 && ./start.sh   # Redémarrage intelligent
```y/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Garden-orange.svg)](https://gazebosim.org/)
[![Node.js](https://img.shields.io/badge/Node.js-16+-green.svg)](https://nodejs.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

## 🎯 Aperçu du Projet

DIAMANTS est un système avancé de navigation autonome multi-agents utilisant des algorithmes de SLAM collaboratif pour la coordination de drones Crazyflie. Le système intègre une simulation Gazebo haute-fidélité avec une interface web moderne pour le contrôle de mission en temps réel.

### ✨ Fonctionnalités Principales

- **🤖 SLAM Collaboratif** : 8 drones coordonnés avec cartographie partagée
- **🌐 Interface Web** : Contrôle de mission en temps réel avec visualisation 3D  
- **🎮 Simulation Gazebo** : Environnement physique réaliste avec physics engine
- **📡 Bridge ROS2** : Communication temps-réel entre simulation et contrôle
- **🔄 Auto-déploiement** : Installation et lancement automatiques
- **📊 Monitoring** : Surveillance en temps réel des performances système

## 🚀 Démarrage Rapide

### 📋 Prérequis Système

- **Ubuntu 22.04+ ou 24.04+**
- **ROS2 Jazzy/Humble/Iron** (détection automatique)
- **Gazebo Garden**
- **Node.js 16+** 
- **Python 3.8+**
- **Git**

### ⚡ Installation Express (Clone & Run)

```bash
# 1. Clone du repository
git clone https://github.com/lololem/diamants.git
cd diamants

# 2. Configuration et démarrage unifié
cd DIAMANTS_API
./scripts/setup/setup-dependencies.sh  # Installation complète
./start.sh                             # Démarrage système

# 3. Vérification
./check.sh                             # Statut de tous les services
```

**Services actifs :**
- 🌐 **API REST** : http://localhost:8000 (Documentation: /docs)
- 🌐 **WebSocket Service** : ws://localhost:8765 (Service unifié)
- 🤖 **ROS2 Nodes** : diamant_websocket_service
- ⚙️ **Processus** : launcher.py (intégration complète)

## 🚀 Lanceurs Master - Clone & Run Complet

### ⚡ Démarrage Express (Nouveau !)

**Script tout-en-un** pour démarrer l'écosystème DIAMANTS complet :

```bash
# Lancement rapide - Frontend + Backend + API
./run.sh                    # 🚀 Clone & Run Express (mode automatique)

# OU lancement interactif avec menu
./launch_diamants.sh        # 🎮 Menu interactif complet

# Arrêt complet du système  
./stop_diamants.sh          # 🛑 Arrêt de tous les composants
```

### 🎮 Options du Menu Interactif

Le script `./launch_diamants.sh` propose :

1. **🚀 Lancement complet** - API + Frontend + Backend ROS2 SLAM
2. **📡 API seulement** - WebSocket Service + FastAPI  
3. **🎮 Frontend seulement** - Interface Three.js (port 5173)
4. **🤖 Backend seulement** - SLAM collaboratif avec TMUX
5. **🔧 Setup/Installation** - Configuration automatique des dépendances
6. **📊 Status des services** - Monitoring temps réel
7. **🛑 Arrêter tout** - Nettoyage complet du système

### 🏃‍♂️ Mode Express vs Menu

| Mode | Commande | Usage | Temps |
|------|----------|-------|--------|
| **Express** | `./run.sh` | Démonstration rapide | ~30s |
| **Développement** | `./launch_diamants.sh` | Contrôle fin composants | Manuel |
| **Production** | `DIAMANTS_API/start.sh` | API service seulement | ~5s |

### 🔧 Auto-Setup Intégré

Les scripts master incluent :
- ✅ **Détection prérequis** (ROS2, Node.js, Python3, TMUX)
- ✅ **Installation automatique** des environnements Python et Node.js
- ✅ **Build ROS2** automatique si workspace non compilé
- ✅ **Gestion des ports** et nettoyage des processus
- ✅ **Monitoring temps réel** des services

## 🏗️ Architecture Système Unifiée

```
DIAMANTS/
├── 🔧 DIAMANTS_API/        # Hub de déploiement + API unifiée
├── 🚁 DIAMANTS_BACKEND/    # Core ROS2 + SLAM + Simulation  
├── 🌐 DIAMANTS_FRONTEND/   # Interface Web (Vite + Three.js)
└── 📚 Documentation/       # Guides et références
```

### 🔧 DIAMANTS_API - Architecture Simplifiée v2.0

**Nouveau système unifié** avec élimination des duplications :

```bash
./start.sh                # 🚀 Démarrage unifié (API + WebSocket + ROS2)
./stop.sh                 # ⏹️  Arrêt propre de tous les services
./check.sh                # 📊 Statut unifié (plus de confusion Bridge/Service)
./launcher.py             # ⭐ Point d'entrée principal avec WebSocket Service
```

**Améliorations majeures :**
- ✅ **WebSocket Service unifié** : Un seul service (port 8765) au lieu de multiples bridges
- ✅ **Configuration centralisée** : Dataclass remplace Pydantic pour plus de fiabilité
- ✅ **Type checking complet** : Support Pylance avec imports conditionnels ROS2
- ✅ **Tests intégrés** : Suite pytest complète avec mocks pour environnements sans ROS2
- ✅ **Architecture clean** : 6 fichiers essentiels à la racine, scripts organisés

## �️ Architecture Système

```
DIAMANTS/
├── 🔧 DIAMANTS_API/        # Système de déploiement automatique
├── 🚁 DIAMANTS_BACKEND/    # Core ROS2 + SLAM + Simulation
├── 🌐 DIAMANTS_FRONTEND/   # Interface Web (Vite + Three.js)
└── 📚 Documentation/       # Guides et références
```

### 🔧 DIAMANTS_API - Hub de Déploiement

Scripts de gestion automatisée :

```bash
./quick-setup.sh      # 🚀 Installation complète automatique
./start-all.sh        # ▶️  Démarrage système complet  
./stop-all.sh         # ⏹️  Arrêt propre de tous les services
./restart.sh          # 🔄 Redémarrage intelligent
./status.sh           # 📊 Monitoring état système
./diagnose.sh         # 🔍 Diagnostic détaillé
```

### 🚁 DIAMANTS_BACKEND - Core System

- **SLAM Collaboratif** : Cartographie multi-agents avec fusion de cartes
- **Simulation Gazebo** : Environnement 3D avec 8 drones Crazyflie
- **Bridge ROS2-Gazebo** : Communication bidirectionnelle temps-réel
- **Orchestration TMUX** : Gestion multi-processus optimisée

### 🌐 DIAMANTS_FRONTEND - Interface Moderne

- **Mission Control** : Interface de contrôle intuitive
- **Visualisation 3D** : Rendu temps-réel avec Three.js
- **Monitoring Live** : Métriques système en temps réel
- **Multi-plateforme** : Compatible desktop/mobile

## 📚 Utilisation Système Unifié

### 🎮 Contrôles Simplifiés

```bash
# Dans DIAMANTS_API/ - Commands essentiels
cd DIAMANTS_API/

./start.sh                    # 🚀 Démarrage complet (API + WebSocket + ROS2)
./stop.sh                     # ⏹️  Arrêt propre de tous les services
./check.sh                    # 📊 Statut unifié de tous les composants

# Diagnostic et maintenance
./scripts/maintenance/diagnose.sh    # 🔍 Diagnostic détaillé du système
./scripts/setup/setup-dependencies.sh # 🛠️ Réinstallation des dépendances
```

### 🌐 Accès Services

```bash
# API REST et documentation
curl http://localhost:8000/health     # Test santé API
open http://localhost:8000/docs       # Documentation interactive

# WebSocket Service (unifié)
wscat -c ws://localhost:8765          # Test connexion WebSocket

# ROS2 Integration
source /opt/ros/jazzy/setup.bash
ros2 node list | grep diamant         # Vérification nodes actifs
ros2 topic list | grep crazyflie      # Topics drone disponibles
```

### ⚙️ Configuration

```bash
# Variables principales dans DIAMANTS_API/api/config.py
API_HOST = "0.0.0.0"               # Host API REST (port 8000)
WEBSOCKET_HOST = "localhost"       # Host WebSocket Service (port 8765)  
ROS2_DOMAIN_ID = 0                 # Domain ROS2
ROS2_NAMESPACE = "/diamants"       # Namespace topics
```

## 🏗️ Technologies & Standards

### 🏗️ Stack Technique Unifié

| Composant | Technologie | Version | Rôle |
|-----------|-------------|---------|------|
| 🤖 **API Backend** | FastAPI + WebSockets | Latest | Interface unifiée REST + WebSocket |
| 🤖 **Middleware** | ROS2 | Jazzy | Communication robotique temps-réel |
| 🎮 **Simulation** | Gazebo | Garden | Physique et rendu 3D |
| 🌐 **Frontend** | Vite + Three.js | Latest | Interface utilisateur moderne |
| 🚁 **Drones** | Crazyflie 2.1 | Crazyswarm2 | Plateforme matérielle |
| 🗺️ **SLAM** | Custom + ROS2 | - | Localisation et cartographie |
| 🔗 **Orchestration** | Python + Asyncio | 3.12+ | Gestion événements asynchrones |

## ✅ Statut Architecture v2.0

## �️ Technologies & Standards

### 🏗️ Stack Technique

| Composant | Technologie | Version | Rôle |
|-----------|-------------|---------|------|
| 🤖 **Middleware** | ROS2 | Jazzy/Humble | Communication inter-processus |
| 🎮 **Simulation** | Gazebo | Garden | Physique et rendu 3D |
| 🌐 **Frontend** | Vite + Three.js | Latest | Interface utilisateur moderne |
| 🚁 **Drones** | Crazyflie 2.1 | Crazyswarm2 | Plateforme matérielle |
| 🗺️ **SLAM** | Custom + ROS2 | - | Localisation et cartographie |
| 🔗 **Orchestration** | TMUX + Bash | - | Gestion multi-processus |

## ✅ Statut de Déploiement

**✅ SYSTÈME OPÉRATIONNEL** - Déploiement complet testé et fonctionnel !

- ✅ **Frontend** : Déploiement automatique avec gestion ports (5550-5559)
- ✅ **Backend** : Orchestration TMUX complète avec support ROS2 Jazzy/Humble
- ✅ **Cross-platform** : Scripts shell portables avec chemins relatifs
- ✅ **Auto-setup** : Expérience clone-and-run en une commande
- ✅ **Monitoring** : Vérification statut temps-réel et diagnostics

## 🔍 Diagnostic & Résolution Problèmes

### 🆘 Problèmes Courants

#### 🔌 Conflit de Ports
```bash
# Libération ports automatique
./DIAMANTS_API/fix-ports.sh

# Vérification ports utilisés
netstat -tlnp | grep :5551
```

#### 🚁 Session TMUX Issues
```bash
# Vérification session active
tmux ls

# Attacher à la session backend
tmux attach -t slam_collab

# Nettoyage session corrompue
tmux kill-session -t slam_collab
./DIAMANTS_API/restart.sh
```

#### 🌐 Issues Frontend
```bash
# Mode développement avec hot-reload
cd DIAMANTS_FRONTEND/Mission_system
npm run dev -- --port 5552
```

## 📊 Monitoring en Temps Réel

```bash
# Statut système complet
./DIAMANTS_API/status.sh

# Logs backend en temps réel
tail -f DIAMANTS_BACKEND/logs/diamants_tmux_slam_collab_journal.log

# Monitoring ROS2 topics
ros2 topic list
ros2 node list

# Performance Gazebo
gz topic -l
gz model -l
```

## 🚀 Commencer Maintenant

```bash
# Installation express - 3 commandes seulement !
git clone https://github.com/lololem/diamants.git
cd diamants  
./DIAMANTS_API/quick-setup.sh && ./DIAMANTS_API/start-all.sh
```

**🎉 Votre système DIAMANTS sera opérationnel en moins de 5 minutes !**

> 💡 **Astuce** : Utilisez `./DIAMANTS_API/status.sh` pour monitorer l'état du système à tout moment.

---

<div align="center">

**🚁 DIAMANTS - Repoussons les Limites de l'Autonomie Multi-Agents 🚁**

[� Star ce Repo](https://github.com/lololem/diamants) • [🐛 Report Bug](https://github.com/lololem/diamants/issues) • [💡 Request Feature](https://github.com/lololem/diamants/issues)

</div>

### Common Issues

**Port conflicts?**
```bash
./DIAMANTS_API/fix-ports.sh
```

**Dependencies missing?**
```bash
./DIAMANTS_API/setup-dependencies.sh
```

**Services not starting?**
```bash
./DIAMANTS_API/diagnose.sh
```

**Permission issues?**
```bash
sudo chown -R $USER:$USER ./DIAMANTS_*
```

---

**🚁 Ready to explore autonomous drone swarms?**

Run `./DIAMANTS_API/quick-setup.sh` and start your journey into the future of robotics! ✨
