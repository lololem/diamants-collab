# 🚁 DIAMANTS - Distributed Intelligence for Autonomous Multi-Agent Navigation and Tactical Systems

[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.r## ✅ Statut Architecture v2.0

**✅ SYSTÈME ENTIÈREMENT REFACTORISÉ** - Architecture unifiée et simplifiée !

- ✅ **API unifiée** : Un seul point d'entrée (launcher.py) avec FastAPI + WebSocket Service
- ✅ **Configuration centralisée** : Dataclass remplace Pydantic pour fiabilité  
- ✅ **Tests complets** : Suite pytest avec support ROS2 et mocks
- ✅ **Type checking** : Support Pylance complet avec imports conditionnels
- ✅ **Architecture clean** : Élimination duplications, 6 fichiers essentiels à la racine
- ✅ **Performance optimisée** : Un seul WebSocket Service au lieu de multiples bridges

### 🔥 Améliorations Majeures

| Avant v2.0 | Après v2.0 | Bénéfice |
|-------------|-------------|----------|
| ❌ 2+ WebSocket bridges | ✅ 1 Service unifié | Performance + Simplicité |
| ❌ Pydantic BaseSettings | ✅ Dataclass config | Fiabilité + Type safety |
| ❌ Imports non typés | ✅ Type checking complet | Développement robuste |  
| ❌ Scripts éparpillés | ✅ 6 fichiers essentiels | Maintenance facilitée |
| ❌ Tests fragmentés | ✅ Suite pytest unifiée | Qualité assurée |

## 🔍 Diagnostic & Résolution Problèmes

### 🆘 Problèmes Courants

#### 🔌 Services inactifs
```bash
# Vérification complète système
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
