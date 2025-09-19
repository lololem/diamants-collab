# 🧠 DIAMANTS - Multi-Agent Reinforcement Learning Guide

## 📋 Overview

This guide presents the implementation of collaborative Reinforcement Learning (RL) for your Crazyflie drones in the DIAMANTS environment. We use a Multi-Agent Reinforcement Learning (MARL) approach with modern techniques.

## 🎯 Collaborative RL Objectives

1. **Coordinated Learning**: Drones learn to collaborate effectively
2. **Formation Optimization**: Dynamic formation according to tasks
3. **Collision Avoidance**: Adaptive trajectory learning
4. **Collective Exploration**: Optimized search strategies
5. **Communication**: Information sharing between agents

## 🏗️ Recommended Architecture

### 1. Multi-Agent Framework
- **PettingZoo**: Standard for multi-agent environments
- **Stable-Baselines3**: Robust RL algorithms
- **Ray RLlib**: Distributed learning (optional)

### 2. Selected Algorithms

#### A. Multi-Agent PPO (MAPPO)
```
Advantages:
- Training stability
- Good for cooperation
- Easy to implement
```

#### B. Multi-Agent Deep Deterministic Policy Gradient (MADDPG)
```
Advantages:
- Continuous actions (perfect for drones)
- Centralized training, decentralized execution
- Adaptation to partially observable environments
```

#### C. Independent Q-Learning (IQL)
```
Advantages:
- Simple to implement
- Each agent learns independently
- Good starting point
```

## 🔧 Technical Implementation

### 1. Environment Structure

```python
# Basic structure for multi-agent environment
class DiamantsDroneEnv:
    def __init__(self, num_drones=4):
        self.num_drones = num_drones
        self.agents = [f"drone_{i}" for i in range(num_drones)]
        
    def observation_space(self, agent):
        # Position, velocity, neighbors, targets
        return Box(low=-np.inf, high=np.inf, shape=(28,))
    
    def action_space(self, agent):
        # [vx, vy, vz, yaw_rate]
        return Box(low=-1.0, high=1.0, shape=(4,))
```

### 2. Collaborative Reward Function

```python
def compute_rewards(self, states, actions):
    rewards = {}
    
    for agent in self.agents:
        reward = 0
        
        # Individual reward
        reward += self.target_proximity_reward(agent)
        reward += self.collision_avoidance_reward(agent)
        
        # Collaborative reward
        reward += self.formation_maintenance_reward(agent)
        reward += self.exploration_coverage_reward()
        reward += self.communication_efficiency_reward(agent)
        
        rewards[agent] = reward
    
    return rewards
```

### 3. Shared Observation

```python
def get_observation(self, agent):
    obs = {
        # Local state
        'position': self.get_position(agent),
        'velocity': self.get_velocity(agent),
        'orientation': self.get_orientation(agent),
        
        # Neighbor information
        'neighbors': self.get_neighbors_info(agent),
        
        # Mission objectives
        'targets': self.get_visible_targets(agent),
        'mission_progress': self.get_mission_progress(),
        
        # Communication
        'messages': self.get_received_messages(agent)
    }
    return obs
```

## 🚀 Installation Guide

### 1. Python Dependencies

```bash
# Multi-agent environments
pip install pettingzoo[all]
pip install stable-baselines3[extra]
pip install supersuit

# Advanced RL (optional)
pip install ray[rllib]
pip install wandb  # for monitoring

# Simulation
pip install gymnasium
pip install numpy matplotlib torch
```

### 2. File Structure

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

## 📊 Evaluation Metrics

### 1. Individual Metrics
- Navigation accuracy
- Collision avoidance
- Energy efficiency
- Mission time

### 2. Collaborative Metrics
- Swarm cohesion
- Area coverage
- Temporal coordination
- Communication efficiency

### 3. Mission Metrics
- Success rate
- Number of targets found
- Average discovery time
- Exploration redundancy

## 🔬 Training Strategies

### 1. Curriculum Learning
```python
# Difficulty progression
stages = [
    {"num_drones": 2, "targets": 1, "obstacles": 0},
    {"num_drones": 3, "targets": 2, "obstacles": 1},
    {"num_drones": 4, "targets": 3, "obstacles": 2},
    {"num_drones": 6, "targets": 5, "obstacles": 3}
]
```

### 2. Self-Play
- Train against previous versions
- Maintain strategy diversity
- Avoid over-specialization

### 3. Emergent Communication
- Let agents develop their own protocol
- Reward efficient coordination
- Analyze communication patterns

## 📈 Recommended Hyperparameters

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

## 🎮 DIAMANTS Integration

### 1. WebSocket Integration
```javascript
// In your HTML simulation
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

### 2. Simulation State
```python
# Python-JavaScript connector
class DiamantsBridge:
    def __init__(self):
        self.websocket_server = start_websocket_server()
        
    def step(self, actions):
        # Send actions to simulation
        self.send_to_simulation(actions)
        
        # Receive new state
        observations = self.receive_from_simulation()
        rewards = self.compute_rewards(observations)
        dones = self.check_episode_end(observations)
        
        return observations, rewards, dones, {}
```

## 🧪 Training Scenarios

### 1. Formation Flying
```python
def formation_scenario():
    """Maintain V formation during flight"""
    targets = generate_waypoints_path()
    formation_shape = "V_formation"
    obstacles = generate_dynamic_obstacles()
    return scenario_config
```

### 2. Search and Rescue
```python
def search_rescue_scenario():
    """Search for targets in unknown area"""
    search_area = generate_complex_terrain()
    targets = place_hidden_targets()
    time_limit = 300  # seconds
    return scenario_config
```

### 3. Collaborative Surveillance
```python
def surveillance_scenario():
    """Optimal coverage of an area"""
    surveillance_area = define_area_of_interest()
    patrol_points = generate_patrol_pattern()
    intruders = add_moving_intruders()
    return scenario_config
```

## 📝 Training Example

```python
# train_collaborative_drones.py
import torch
from stable_baselines3 import PPO
from pettingzoo.utils import parallel_to_aec

# 1. Create environment
env = DiamantsDroneEnv(num_drones=4)
env = parallel_to_aec(env)

# 2. Configure agent
model = PPO(
    "MultiInputPolicy", 
    env, 
    verbose=1,
    tensorboard_log="./tensorboard_logs/"
)

# 3. Train
model.learn(total_timesteps=1000000)

# 4. Save
model.save("diamants_collaborative_model")
```

## 🎯 Next Steps

1. **Phase 1**: Basic implementation with IQL
2. **Phase 2**: Upgrade to MAPPO
3. **Phase 3**: Add communication
4. **Phase 4**: Deploy in complete simulation
5. **Phase 5**: Real hardware testing

## 📚 Additional Resources

- [PettingZoo Documentation](https://pettingzoo.farama.org/)
- [Stable-Baselines3 Guide](https://stable-baselines3.readthedocs.io/)
- [Multi-Agent RL Papers](https://arxiv.org/search/?query=multi-agent+reinforcement+learning&searchtype=all)
- [Ray RLlib Examples](https://docs.ray.io/en/latest/rllib/index.html)

This guide provides a solid foundation for implementing collaborative learning in your DIAMANTS system. The modular approach allows gradual progression towards more sophisticated behaviors.
