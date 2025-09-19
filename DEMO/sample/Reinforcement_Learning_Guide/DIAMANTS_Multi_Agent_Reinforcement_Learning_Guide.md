# 🧠 DIAMANTS - Guide du Reinforcement Learning Multi-Agent

## 📋 Vue d'ensemble

Ce guide présente l'implémentation du Reinforcement Learning (RL) collaboratif pour vos drones Crazyflie dans l'environnement DIAMANTS. Nous utilisons une approche Multi-Agent Reinforcement Learning (MARL) avec des techniques modernes.

## 🎯 Objectifs du RL Collaboratif

1. **Apprentissage coordonné** : Les drones apprennent à collaborer efficacement
2. **Optimisation des formations** : Formation dynamique selon les tâches
3. **Évitement de collisions** : Apprentissage adaptatif des trajectoires
4. **Exploration collective** : Stratégies de recherche optimisées
5. **Communication** : Partage d'informations entre agents

## 🏗️ Architecture Recommandée

### 1. Framework Multi-Agent
- **PettingZoo** : Standard pour environnements multi-agents
- **Stable-Baselines3** : Algorithmes RL robustes
- **Ray RLlib** : Apprentissage distribué (optionnel)

### 2. Algorithmes Sélectionnés

#### A. Multi-Agent PPO (MAPPO)
```
Avantages:
- Stabilité d'entraînement
- Bon pour coopération
- Facile à implémenter
```

#### B. Multi-Agent Deep Deterministic Policy Gradient (MADDPG)
```
Avantages:
- Actions continues (parfait pour drones)
- Apprentissage centralisé, exécution décentralisée
- Adaptation à environnements partiellement observables
```

#### C. Independent Q-Learning (IQL)
```
Avantages:
- Simple à implémenter
- Chaque agent apprend indépendamment
- Bon point de départ
```

## 🔧 Implémentation Technique

### 1. Structure de l'Environnement

```python
# Structure de base pour environnement multi-agent
class DiamantsDroneEnv:
    def __init__(self, num_drones=4):
        self.num_drones = num_drones
        self.agents = [f"drone_{i}" for i in range(num_drones)]
        
    def observation_space(self, agent):
        # Position, vitesse, voisins, targets
        return Box(low=-np.inf, high=np.inf, shape=(28,))
    
    def action_space(self, agent):
        # [vx, vy, vz, yaw_rate]
        return Box(low=-1.0, high=1.0, shape=(4,))
```

### 2. Fonction de Récompense Collaborative

```python
def compute_rewards(self, states, actions):
    rewards = {}
    
    for agent in self.agents:
        reward = 0
        
        # Récompense individuelle
        reward += self.target_proximity_reward(agent)
        reward += self.collision_avoidance_reward(agent)
        
        # Récompense collaborative
        reward += self.formation_maintenance_reward(agent)
        reward += self.exploration_coverage_reward()
        reward += self.communication_efficiency_reward(agent)
        
        rewards[agent] = reward
    
    return rewards
```

### 3. Observation Partagée

```python
def get_observation(self, agent):
    obs = {
        # État local
        'position': self.get_position(agent),
        'velocity': self.get_velocity(agent),
        'orientation': self.get_orientation(agent),
        
        # Informations sur les voisins
        'neighbors': self.get_neighbors_info(agent),
        
        # Objectifs de mission
        'targets': self.get_visible_targets(agent),
        'mission_progress': self.get_mission_progress(),
        
        # Communication
        'messages': self.get_received_messages(agent)
    }
    return obs
```

## 🚀 Guide d'Installation

### 1. Dépendances Python

```bash
# Environnements multi-agents
pip install pettingzoo[all]
pip install stable-baselines3[extra]
pip install supersuit

# RL avancé (optionnel)
pip install ray[rllib]
pip install wandb  # pour monitoring

# Simulation
pip install gymnasium
pip install numpy matplotlib torch
```

### 2. Structure de Fichiers

```
DIAMANTS_RL/
├── environments/
│   ├── drone_env.py
│   └── scenarios/
├── agents/
│   ├── mappo_agent.py
│   ├── maddpg_agent.py
│   └── communication/
├── training/
│   ├── train_mappo.py
│   ├── train_maddpg.py
│   └── configs/
├── evaluation/
│   ├── evaluate.py
│   └── metrics/
└── models/
    └── saved_models/
```

## 📊 Métriques d'Évaluation

### 1. Métriques Individuelles
- Précision de navigation
- Évitement de collisions
- Efficacité énergétique
- Temps de mission

### 2. Métriques Collaboratives
- Cohésion de l'essaim
- Couverture de zone
- Coordination temporelle
- Efficacité de communication

### 3. Métriques de Mission
- Taux de succès
- Nombre de targets trouvées
- Temps moyen de découverte
- Redondance d'exploration

## 🔬 Stratégies d'Entraînement

### 1. Curriculum Learning
```python
# Progression de la difficulté
stages = [
    {"num_drones": 2, "targets": 1, "obstacles": 0},
    {"num_drones": 3, "targets": 2, "obstacles": 1},
    {"num_drones": 4, "targets": 3, "obstacles": 2},
    {"num_drones": 6, "targets": 5, "obstacles": 3}
]
```

### 2. Self-Play
- Entraîner contre des versions antérieures
- Maintenir diversité des stratégies
- Éviter surspécialisation

### 3. Communication Emergente
- Laisser les agents développer leur propre protocole
- Récompenser coordination efficace
- Analyser patterns de communication

## 📈 Hyperparamètres Recommandés

### MAPPO
```python
config = {
    'learning_rate': 3e-4,
    'batch_size': 512,
    'n_epochs': 10,
    'gamma': 0.99,
    'gae_lambda': 0.95,
    'clip_range': 0.2,
    'value_loss_coef': 0.5,
    'entropy_coef': 0.01
}
```

### MADDPG
```python
config = {
    'actor_lr': 1e-4,
    'critic_lr': 1e-3,
    'tau': 0.01,
    'gamma': 0.95,
    'buffer_size': 1000000,
    'batch_size': 256,
    'noise_std': 0.2
}
```

## 🎮 Interface avec DIAMANTS

### 1. Intégration WebSocket
```javascript
// Dans votre simulation HTML
class RLInterface {
    constructor() {
        this.ws = new WebSocket('ws://localhost:8765');
        this.agents = {};
    }
    
    sendObservations(observations) {
        this.ws.send(JSON.stringify({
            type: 'observations',
            data: observations
        }));
    }
    
    receiveActions(callback) {
        this.ws.onmessage = (event) => {
            const data = JSON.parse(event.data);
            if (data.type === 'actions') {
                callback(data.actions);
            }
        };
    }
}
```

### 2. État de Simulation
```python
# Connecteur Python-JavaScript
class DiamantsBridge:
    def __init__(self):
        self.websocket_server = start_websocket_server()
        
    def step(self, actions):
        # Envoyer actions à la simulation
        self.send_to_simulation(actions)
        
        # Recevoir nouvel état
        observations = self.receive_from_simulation()
        rewards = self.compute_rewards(observations)
        dones = self.check_episode_end(observations)
        
        return observations, rewards, dones, {}
```

## 🧪 Scénarios d'Entraînement

### 1. Formation Flying
```python
def formation_scenario():
    """Maintenir formation en V pendant vol"""
    targets = generate_waypoints_path()
    formation_shape = "V_formation"
    obstacles = generate_dynamic_obstacles()
    return scenario_config
```

### 2. Search and Rescue
```python
def search_rescue_scenario():
    """Recherche de targets dans zone inconnue"""
    search_area = generate_complex_terrain()
    targets = place_hidden_targets()
    time_limit = 300  # secondes
    return scenario_config
```

### 3. Surveillance Collaborative
```python
def surveillance_scenario():
    """Couverture optimale d'une zone"""
    surveillance_area = define_area_of_interest()
    patrol_points = generate_patrol_pattern()
    intruders = add_moving_intruders()
    return scenario_config
```

## 📝 Exemple d'Entraînement

```python
# train_collaborative_drones.py
import torch
from stable_baselines3 import PPO
from pettingzoo.utils import parallel_to_aec

# 1. Créer environnement
env = DiamantsDroneEnv(num_drones=4)
env = parallel_to_aec(env)

# 2. Configurer agent
model = PPO(
    "MultiInputPolicy", 
    env, 
    verbose=1,
    tensorboard_log="./tensorboard_logs/"
)

# 3. Entraîner
model.learn(total_timesteps=1000000)

# 4. Sauvegarder
model.save("diamants_collaborative_model")
```

## 🎯 Prochaines Étapes

1. **Phase 1** : Implémentation de base avec IQL
2. **Phase 2** : Upgrade vers MAPPO
3. **Phase 3** : Ajout de communication
4. **Phase 4** : Déploiement dans simulation complète
5. **Phase 5** : Tests sur matériel réel

## 📚 Ressources Complémentaires

- [PettingZoo Documentation](https://pettingzoo.farama.org/)
- [Stable-Baselines3 Guide](https://stable-baselines3.readthedocs.io/)
- [Multi-Agent RL Papers](https://arxiv.org/search/?query=multi-agent+reinforcement+learning&searchtype=all)
- [Ray RLlib Examples](https://docs.ray.io/en/latest/rllib/index.html)

Ce guide fournit une base solide pour implémenter l'apprentissage collaboratif dans votre système DIAMANTS. L'approche modulaire permet une progression graduelle vers des comportements plus sophistiqués.
