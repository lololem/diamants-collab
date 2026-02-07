# 📋 DIAMANTS — Plan de Travail Global

> Généré le 2026-02-07 — Basé sur l'analyse de diamants-collab, diamants-private, LOTUSSIM, et les 14 issues GitHub.

---

## 🧠 Compréhension du Système

### Ce qu'on a :
- **diamants-collab** : Plateforme de simulation multi-drones (ROS2 Jazzy + Gazebo + FastAPI + Three.js). 14 issues ouvertes, dont 8 critiques. Le pipeline Frontend → API → Backend ne fonctionne pas de bout en bout (ports WebSocket incohérents, topics ROS2 inexistants, endpoints API hardcodés).
- **diamants-private** : Stack d'autonomie IA pour drones réels (MAVLink/PX4 + LLM Qwen fine-tuné + RL PPO/SAC). Pipeline Perception → Décision → Action fonctionnel. Bridge Gazebo fragile via subprocess. Aucune intégration ROS2.
- **LOTUSSIM** : Simulateur maritime Naval Group (Gazebo Harmonic + ROS2 + xDyn). Fonctionne en autonome, zéro intégration avec DIAMANTS. Modèle x500 (drone) disponible mais non connecté.

### L'objectif :
Faire le pont entre diamants-private (IA/autonomie) et diamants-collab (simulation/visualisation), pour qu'un drone piloté par l'IA de diamants-private puisse être simulé, visualisé, et contrôlé via la plateforme diamants-collab.

---

## 🔴 PHASE 1 — Fondations (Critique, bloquant)

> Résoudre les issues GitHub critiques qui empêchent diamants-collab de fonctionner.

### T1.1 — Consolider en UN SEUL WebSocket Bridge
**Issues : #13, #12, #1**
- [ ] Auditer les 6 implémentations WebSocket existantes
- [ ] Conserver `DIAMANTS_API/services/websocket_service.py` comme unique bridge
- [ ] Supprimer les 5 autres (api/websocket_bridge.py, api/websocket_bridge_simple.py, inline main.py, backend/core/web_interface/websocket_bridge.py, frontend/ros2_bridge/unified_websocket_bridge.py)
- [ ] Définir UN port unique (ex: 8765) dans un fichier de config centralisé
- [ ] Mettre à jour le frontend pour se connecter au bon port

### T1.2 — Créer un Registre de Topics ROS2 Partagé
**Issue : #11**
- [ ] Créer `config/ros2_topics.yaml` avec tous les noms de topics standardisés
- [ ] Aligner l'API, le backend, et le frontend sur ces noms
- [ ] Documenter chaque topic (type de message, direction, fréquence)

### T1.3 — Corriger l'Installation des Packages ROS2 Backend
**Issue : #14**
- [ ] Auditer les CMakeLists.txt et setup.py de chaque package ROS2
- [ ] Corriger les `entry_points` pour multi_agent_framework
- [ ] S'assurer que les executables référencés dans les launch files existent
- [ ] Tester un `colcon build` propre

### T1.4 — Connecter les Endpoints API aux Vraies Données
**Issue : #11**
- [ ] Remplacer les réponses hardcodées de main.py par des requêtes au WebSocket bridge
- [ ] Utiliser les modèles Pydantic existants dans models.py (actuellement dead code)
- [ ] Implémenter les publishers ROS2 manquants pour `/diamants/drones/positions`
- [ ] Remplir `drone_position_coordinator.py` (actuellement vide)

### T1.5 — Brancher le Vrai ROS Bridge dans le Frontend
- [ ] Remplacer l'import de `ros-bridge-simple.js` (stub) par `ros-bridge.js` (réel) dans main.js
- [ ] Configurer la connexion rosbridge sur le bon port
- [ ] Tester la réception de données de télémétrie en temps réel

---

## 🟠 PHASE 2 — Bridge diamants-private ↔ diamants-collab

> Créer le pont entre l'IA (MAVLink/PX4) et la simulation (ROS2/Gazebo).

### T2.1 — Créer un Node ROS2 Bridge MAVLink ↔ ROS2
- [ ] Créer un package ROS2 `diamants_mavlink_bridge` dans le backend
- [ ] Implémenter un node qui traduit :
  - MAVLink `GLOBAL_POSITION_INT` → ROS2 `geometry_msgs/PoseStamped`
  - MAVLink `ATTITUDE` → ROS2 `sensor_msgs/Imu`
  - MAVLink `SYS_STATUS` → topic batterie
  - ROS2 commandes → MAVLink `SET_POSITION_TARGET_LOCAL_NED`
- [ ] Supporter le mode SITL (UDP 14550) et matériel réel (serial)
- [ ] Publier sur les topics standardisés de T1.2

### T2.2 — Intégrer le World Gazebo de diamants-private
- [ ] Adapter `worlds/diamants_world.sdf` de diamants-private pour diamants-collab
- [ ] Ajouter le modèle X500 avec caméra OAK-D aux assets Gazebo de collab
- [ ] Créer un launch file qui charge ce monde dans le Gazebo de collab
- [ ] Remplacer le bridge `gz topic` par subprocess de diamants-private par un bridge gz-transport natif

### T2.3 — Connecter l'Inference RL/LLM au Pipeline ROS2
- [ ] Exposer `inference/decision_loop.py` comme service ROS2 (ou via l'API WebSocket)
- [ ] Créer un topic `/diamants/ai/decision` pour publier les décisions RL/LLM
- [ ] Faire remonter les données perception Gazebo vers le decision loop
- [ ] Permettre le mode hybride : FSM de sécurité (collab) + LLM adaptatif (private)

### T2.4 — Entraîner le Modèle RL Manquant
- [ ] Utiliser `training/train_rl.py` + `training/gym_env.py` (code prêt, aucun modèle entraîné)
- [ ] Lancer un entraînement PPO sur la DiamantsDroneEnv
- [ ] Exporter le modèle dans `models/rl_models/`
- [ ] Valider l'inférence < 1ms via `inference/rl_inference.py`

---

## 🟡 PHASE 3 — Qualité & Robustesse

> Corriger les issues majeures et améliorer la stabilité.

### T3.1 — Sécuriser l'API
**Issue : #4**
- [ ] Ajouter une authentification token sur les endpoints critiques (takeoff, land, arm)
- [ ] Restreindre CORS (`allow_origins=["*"]` → origines spécifiques)
- [ ] Ajouter validation d'entrée sur les commandes de mouvement
- [ ] Ajouter rate limiting
- [ ] Remplacer `"your-secret-key-here"` par une vraie clé via env var

### T3.2 — Découper le God-Class Frontend
**Issue : #5**
- [ ] Extraire `SceneManager` de main.js (rendu, caméra, lumières)
- [ ] Extraire `RosBridgeController` (communication WebSocket)
- [ ] Extraire `InputController` (gestion événements utilisateur)
- [ ] Extraire `DroneManager` (gestion des entités drones)
- [ ] Supprimer le monkey-patching de `console.log`

### T3.3 — Internationaliser le Code
**Issue : #3**
- [ ] Migrer tous les commentaires/logs français → anglais dans le code
- [ ] Conserver la documentation bilingue (README, wiki) mais pas le code
- [ ] Utiliser un système i18n pour les messages UI

### T3.4 — Fiabiliser les Scripts Shell
**Issue : #6**
- [ ] Remplacer `kill -9` par des shutdowns gracieux
- [ ] Ajouter des vérifications de dépendances avant lancement
- [ ] Standardiser les logs en anglais
- [ ] Ajouter gestion d'erreurs et codes de retour

### T3.5 — Corriger l'Initialisation Three.js
**Issue : #2**
- [ ] Ajouter détection WebGL robuste avec fallback
- [ ] Gérer les race conditions de chargement async
- [ ] Ajouter cleanup/dispose pour éviter les fuites mémoire
- [ ] Tester sur devices à faible capacité

---

## 🟢 PHASE 4 — S'inspirer de LOTUSSIM + Scénario Défense des Forêts

> LOTUSSIM n'est PAS intégré au système. On s'en inspire uniquement pour récupérer des patterns utiles.
> Le use case reste 100% défense des forêts (lutte incendies VAR).

### T4.1 — Récupérer les bons patterns de LOTUSSIM
- [ ] S'inspirer de l'architecture plugins Gazebo de LOTUSSIM (entity_manager, sensors)
- [ ] Étudier le système LOTUSim-UI (React+Leaflet+Three.js) pour améliorer le frontend DIAMANTS
- [ ] Reprendre le pattern de physics_engine_interface (abstraction pluggable) si pertinent
- [ ] Ne PAS intégrer les messages `lotusim_msgs` ni le bridge Unity — hors scope

### T4.2 — Scénario Défense des Forêts du VAR
- [ ] Créer un monde Gazebo "forêt VAR" avec végétation, relief, zones de feu
- [ ] Définir des missions YAML spécifiques : détection incendie, surveillance périmètre, cartographie zone brûlée
- [ ] Intégrer la détection de feu via le pipeline VLM (Moondream) de diamants-private
- [ ] Créer un scénario de démo complet : détection → alerte → coordination essaim → intervention

---

## 🔵 PHASE 5 — Polish & Documentation

### T5.1 — Réécrire Architecture.md
**Issue : #8**
- [ ] Supprimer les références à des fichiers inexistants
- [ ] Documenter l'architecture réelle (pas "React UI" quand c'est du vanilla JS)
- [ ] Ajouter le diagramme d'intégration diamants-private ↔ diamants-collab
- [ ] Documenter le pipeline IA (LLM + RL)

### T5.2 — Ajouter des Tests
- [ ] Tests unitaires frontend avec Vitest (formules, intelligence, engine)
- [ ] Remplacer les `assert True # Placeholder` dans les tests API
- [ ] Tests d'intégration WebSocket (frontend → API → ROS2)
- [ ] CI/CD basique avec GitHub Actions

### T5.3 — Standardiser le Code
**Issues : #7, #9, #10**
- [ ] Configurer ESLint + Prettier pour le frontend
- [ ] Fixer les versions des dépendances dans package.json
- [ ] Nettoyer le CSS inline → design system avec variables CSS
- [ ] Uniformiser les conventions de nommage (camelCase partout en JS)

---

## 📊 Résumé des Priorités

| Phase | Priorité | Effort estimé | Impact |
|-------|----------|---------------|--------|
| **Phase 1** | 🔴 Critique | ~3-5 jours | Le système fonctionne de bout en bout |
| **Phase 2** | 🟠 Haute | ~5-7 jours | Bridge IA ↔ Simulation opérationnel |
| **Phase 3** | 🟡 Moyenne | ~3-4 jours | Stabilité et sécurité |
| **Phase 4** | 🟢 Basse | ~3-5 jours | Patterns LOTUSSIM + scénario forêts VAR |
| **Phase 5** | 🔵 Basse | ~2-3 jours | Qualité projet open-source |

---

## 📎 Références

- **Issues GitHub** : https://github.com/lololem/diamants-collab/issues (#1 à #14, toutes ouvertes)
- **Wiki** : https://github.com/lololem/diamants-collab/wiki
- **diamants-private** : Stack IA locale (MAVLink + Qwen fine-tuné + RL PPO)
- **LOTUSSIM** : Simulateur Naval Group (Gazebo + xDyn) — source d'inspiration uniquement, pas d'intégration
