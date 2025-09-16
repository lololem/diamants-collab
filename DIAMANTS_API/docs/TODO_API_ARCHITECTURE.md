# TODO: Architecture API/ServiceWEB/ServiceLayer Centralisée
## Objectif: Éliminer la duplication code WebSocket et centraliser la couche service

---

## 🔍 ANALYSE ACTUELLE - DUPLICATIONS IDENTIFIÉES

### WebSocket Bridges (DUPLICATIONS CRITIQUES)
```
❌ DUPLICATION DÉTECTÉE:
- DIAMANTS_BACKEND/core/web_interface/websocket_bridge.py (367 lignes)
- DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/.../websocket_bridge.py (367 lignes - IDENTIQUE!)
- DIAMANTS_FRONTEND/Mission_system/ros2_bridge/unified_websocket_bridge.py (différent)
```

### Serveurs Web Multiples
```
❌ DUPLICATION POTENTIELLE:
- DIAMANTS_BACKEND/core/web_interface/web_server.py (FastAPI)
- DIAMANTS_BACKEND/core/web_interface/web_server_robust.py
- DIAMANTS_FRONTEND/Mission_system/net/ (clients WebSocket)
```

### Clients WebSocket Frontend
```
❌ LOGIQUE ÉPARPILLÉE:
- DIAMANTS_FRONTEND/Mission_system/net/websocket-adaptive-manager.js
- DIAMANTS_FRONTEND/Mission_system/net/websocket-client.js  
- DIAMANTS_FRONTEND/Mission_system/controllers/crazyflie-ros-controller.js
```

---

## 🎯 ARCHITECTURE CIBLE - DIAMANTS_API Centralisé

### Structure Proposée
```
DIAMANTS_API/
├── services/
│   ├── websocket_service.py          # Service WebSocket unifié
│   ├── ros2_bridge_service.py        # Bridge ROS2 ↔ API
│   ├── drone_command_service.py      # Service commandes drones
│   └── swarm_intelligence_service.py # Service intelligence essaim
├── api/
│   ├── __init__.py
│   ├── main.py                       # FastAPI principal
│   ├── routes/
│   │   ├── drones.py                 # Routes /api/drones/*
│   │   ├── swarm.py                  # Routes /api/swarm/*
│   │   ├── simulation.py             # Routes /api/simulation/*
│   │   └── websocket.py              # Endpoints WebSocket
│   └── models/
│       ├── drone_models.py
│       ├── swarm_models.py
│       └── message_models.py
├── clients/
│   ├── websocket_client.js           # Client WebSocket unifié frontend
│   └── ros2_client.py               # Client ROS2 unifié backend
└── config/
    ├── api_config.py
    ├── websocket_config.py
    └── cors_config.py
```

---

## 📋 PLAN DE MIGRATION (Priorisation par impact)

### Phase 1: Centralisation WebSocket Service 🚨 URGENT
- [ ] **1.1** Créer `DIAMANTS_API/services/websocket_service.py`
  - Merger logique des 2 bridges identiques du BACKEND
  - Port unifié: 8765 pour WebSocket
  - Message format standardisé
  
- [ ] **1.2** Supprimer duplications BACKEND
  - ❌ Supprimer: `DIAMANTS_BACKEND/slam_collaboratif/.../websocket_bridge.py`
  - ✅ Conserver: `DIAMANTS_BACKEND/core/web_interface/websocket_bridge.py` → migrate vers API
  
- [ ] **1.3** Unifier clients frontend
  - Créer `DIAMANTS_API/clients/websocket_client.js`
  - Merger: `websocket-adaptive-manager.js` + `websocket-client.js`
  - ❌ Supprimer fichiers dupliqués frontend

### Phase 2: API REST Structurée
- [ ] **2.1** Setup FastAPI principal (`DIAMANTS_API/api/main.py`)
  - Port: 8080 pour API REST
  - CORS configuré pour frontend:5554
  
- [ ] **2.2** Routes modulaires
  ```python
  GET /api/drones              # Liste drones actifs
  POST /api/drones/{id}/command # Commande drone individuel
  GET /api/swarm/status        # État essaim
  POST /api/swarm/mission      # Nouvelle mission
  WS /ws/realtime             # WebSocket temps réel
  ```

### Phase 3: Service Layer Backend
- [ ] **3.1** ROS2 Bridge Service (`services/ros2_bridge_service.py`)
  - Interface unique ROS2 ↔ API
  - Topics standardisés:
    ```
    /diamants/drones/positions
    /diamants/drones/commands  
    /diamants/swarm/intelligence
    /diamants/swarm/status
    ```

- [ ] **3.2** Services métier
  - `drone_command_service.py`: takeoff, land, move, hover
  - `swarm_intelligence_service.py`: formation, pathfinding, stigmergy
  
### Phase 4: Migration Frontend
- [ ] **4.1** Adapter client WebSocket unifié
  - ✅ Un seul point de connexion: `ws://localhost:8765/ws/realtime`
  - ✅ Un seul gestionnaire d'état WebSocket
  
- [ ] **4.2** API REST integration
  - Remplacer appels directs WebSocket par API REST quand approprié
  - WebSocket reserved pour: positions temps réel, alertes, live metrics

---

## 🔧 CONFIGURATION RÉSEAU UNIFIÉE

### Ports Standardisés
```bash
# API Layer (DIAMANTS_API)
8080: API REST FastAPI
8765: WebSocket temps réel

# Frontend (DIAMANTS_FRONTEND)  
5554: Mission System Vite dev

# Backend (DIAMANTS_BACKEND)
11311: ROS Master (si ROS1)
Variable: ROS2 DDS ports
9090: ROSBridge (si nécessaire)
```

### Message Protocol Unifié
```javascript
// Format standard tous messages WebSocket
{
  "timestamp": "2025-09-16T09:30:00Z",
  "type": "drone_position|swarm_command|system_alert",
  "source": "frontend|backend|api",
  "data": {
    // Payload spécifique au type
  },
  "meta": {
    "version": "1.0",
    "correlation_id": "uuid"
  }
}
```

---

## 🎮 EXEMPLES CONCRETS POST-MIGRATION

### Commande Drone (Frontend → Backend)
```javascript
// Frontend: Un seul appel API
await fetch('http://localhost:8080/api/drones/crazyflie1/command', {
  method: 'POST',
  body: JSON.stringify({
    action: 'takeoff',
    altitude: 1.5,
    duration: 3.0
  })
});

// Plus de gestion WebSocket complexe pour les commandes!
```

### Position Temps Réel (Backend → Frontend)  
```javascript
// Frontend: Un seul WebSocket client
const ws = new DiamantWebSocketClient('ws://localhost:8765/ws/realtime');
ws.onPosition((droneId, position) => {
  // Mise à jour position 3D temps réel
  updateDronePosition(droneId, position);
});
```

### Déploiement Simplifié
```bash
# Un seul point d'entrée API
cd DIAMANTS_API && python -m api.main

# Plus de bridges multiples à gérer!
```

---

## ✅ CRITÈRES DE SUCCÈS

1. **Zero Duplication**: Aucun code WebSocket dupliqué
2. **Single Source of Truth**: API centralisée pour toute communication
3. **Simplified Deployment**: 3 services max (API, Frontend, Backend ROS2)
4. **Maintainability**: Code WebSocket dans 1 seul endroit
5. **Performance**: Latence ≤ 50ms Frontend ↔ Backend via API

---

## 🚨 ACTIONS IMMÉDIATES REQUISES

1. ✋ **STOP**: Ne plus modifier les bridges WebSocket actuels
2. 🔍 **AUDIT**: Identifier tous les points d'intégration WebSocket
3. 🏗️ **BUILD**: Commencer par `DIAMANTS_API/services/websocket_service.py`
4. 🧪 **TEST**: Setup environnement test API isolé
5. 📝 **DOC**: Documenter nouveau protocole API

---

**Prochaine étape**: Qui s'occupe de la Phase 1 ? 🚀
