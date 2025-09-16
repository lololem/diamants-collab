# 🚁 DIAMANTS API

**Plateforme de simulation collaborative de drones avec intégration ROS2 Jazzy**

> ✅ **Architecture unifiée** avec WebSocket Service principal et API REST FastAPI

## 🚀 Démarrage Rapide

### Prérequis
- **Ubuntu 24.04** (recommandé)
- **ROS2 Jazzy** 
- **Python 3.12**
- **Node.js 16+**

### Installation & Démarrage

```bash
# 1. Installation des dépendances (première fois)
./scripts/setup/setup-dependencies.sh

# 2. Démarrage de DIAMANTS
./start.sh

# 3. Vérification du statut
./check.sh
```

### Arrêt

```bash
./stop.sh
```

## 📡 Services Disponibles

| Service | URL | Description | Statut |
|---------|-----|-------------|--------|
| **API REST** | `http://localhost:8000` | Interface principale FastAPI | ✅ Actif |
| **Documentation** | `http://localhost:8000/docs` | Documentation interactive Swagger | ✅ Actif |
| **WebSocket Service** | `ws://localhost:8765` | Service WebSocket unifié ROS2-Web | ✅ Actif |

## 🎯 Architecture Unifiée

```
DIAMANTS API (Port 8000)
├── 📡 REST API (FastAPI)
├── 🌐 WebSocket Service (Port 8765)
├── 🤖 ROS2 Integration (Jazzy)
└── ⚙️ Launcher Process
```

### ✨ Nouveautés v2.0
- ✅ **Architecture simplifiée** : Un seul WebSocket Service au lieu de multiples bridges
- ✅ **Performance optimisée** : Élimination des duplications de code
- ✅ **Configuration unifiée** : Gestion centralisée via dataclass
- ✅ **Type checking complet** : Support Pylance avec imports conditionnels
- ✅ **Tests intégrés** : Suite de tests complète avec pytest

## 🛠️ Scripts Principaux

### À la racine (Essentiels)
- `./start.sh` - Démarrage principal avec ROS2 Jazzy
- `./stop.sh` - Arrêt propre des services  
- `./check.sh` - Vérification rapide du statut (unifiée)
- `./launcher.py` - Point d'entrée principal avec WebSocket Service

### Setup & Maintenance
- `./scripts/setup/setup-dependencies.sh` - Installation complète ROS2 + Python
- `./scripts/setup/quick-setup.sh` - Configuration rapide environnement

### Maintenance
- `./scripts/maintenance/diagnose.sh` - Diagnostic complet
- `./scripts/maintenance/status-detail.sh` - Statut détaillé
- `./scripts/maintenance/stop-all.sh` - Arrêt complet
- `./scripts/maintenance/restart.sh` - Redémarrage
- `./scripts/maintenance/fix-ports.sh` - Réparation des ports

### Développement
- `./scripts/dev/run_tests.sh` - Tests unitaires
- `./scripts/dev/audit-duplications.sh` - Audit du code (éliminé duplications)
- `./scripts/dev/start-*.sh` - Démarrage par composant

## 🤖 Intégration ROS2 Jazzy

DIAMANTS utilise **ROS2 Jazzy** avec architecture unifiée :

```bash
# Topics principaux
/crazyflie/cmd_vel          # Commandes de vélocité
/crazyflie/pose             # Position du drone  
/crazyflie/takeoff          # Décollage
/crazyflie/land             # Atterrissage
/diamants/mission_status    # Statut des missions
/diamants/swarm_coordination # Coordination essaim

# Node ROS2 actif
diamant_websocket_service   # Service WebSocket unifié avec ROS2
```

### 🔧 Configuration ROS2

```python
# Configuration dans api/config.py
ROS2_DOMAIN_ID = 0
ROS2_NAMESPACE = "/diamants"

# Topics mappings automatiques
ROS2_TOPICS = {
    "cmd_vel": "/crazyflie/cmd_vel",
    "pose": "/crazyflie/pose",
    "battery": "/crazyflie/battery_state"
}
```

## 🧪 Tests & Validation

```bash
# Tests complets avec ROS2
cd /home/loic/Projects/AI_PROJECTS/DIAMANTS/DIAMANTS_API
source /opt/ros/jazzy/setup.bash
python -m pytest tests/ -v

# Diagnostic complet système
./scripts/maintenance/diagnose.sh

# Vérification santé API
curl http://localhost:8000/health

# Test WebSocket
wscat -c ws://localhost:8765
```

## 📁 Architecture Simplifiée

```
DIAMANTS_API/
├── start.sh              # ⭐ Démarrage principal (ROS2 + API + WebSocket)
├── stop.sh               # ⭐ Arrêt propre de tous les services
├── check.sh              # ⭐ Statut unifié (API + WebSocket + ROS2)
├── launcher.py           # ⭐ Point d'entrée avec WebSocket Service principal
├── requirements.txt      # ⭐ Dépendances (FastAPI + WebSockets + ROS2)
├── README.md             # ⭐ Documentation mise à jour
├── api/                  # FastAPI REST + configuration unifiée
│   ├── main.py          # Application FastAPI
│   ├── config.py        # Configuration dataclass (plus de Pydantic)
│   └── models.py        # Modèles Pydantic pour API
├── services/            # Services WebSocket unifiés
│   └── websocket_service.py  # Service principal (port 8765)
├── tests/               # Tests pytest complets
└── scripts/             # Scripts organisés par catégorie
    ├── setup/           # Installation et configuration
    ├── maintenance/     # Diagnostic et maintenance  
    └── dev/            # Outils de développement
├── clients/             # Clients WebSocket
├── docs/                # Documentation complète
└── scripts/             # Scripts organisés
    ├── setup/           # Installation & configuration
    ├── maintenance/     # Maintenance & diagnostic
    └── dev/            # Outils développement
```

## 🔧 Configuration

### Variables d'environnement
```bash
export ROS_DOMAIN_ID=0
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
```

### Ports utilisés
- `8000` - API REST
- `8765` - WebSocket Service
- `9001` - WebSocket Bridge ROS2
- `5550-5552` - Frontend (si activé)

## 🆘 Dépannage

### Problèmes courants

**API ne répond pas :**
```bash
./scripts/maintenance/fix-ports.sh
./stop.sh && ./start.sh
```

**ROS2 non configuré :**
```bash
./scripts/setup/setup-dependencies.sh
```

**Diagnostic complet :**
```bash
./scripts/maintenance/diagnose.sh
```

## 🤝 Contribution

1. **Tests** : `./scripts/dev/run_tests.sh`
2. **Audit** : `./scripts/dev/audit-duplications.sh`
3. **Statut** : `./check.sh`

## 📖 Documentation

- **API Interactive** : http://localhost:8000/docs
- **Architecture** : `./PROJECT_README.md`
- **TODO** : `./TODO_API_ARCHITECTURE.md`

---

**🎯 Usage Principal :**
```bash
./start.sh    # Démarrer DIAMANTS
./check.sh    # Vérifier le statut
./stop.sh     # Arrêter DIAMANTS
```
