# DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
# 
# Copyright (c) 2025 DIAMANTS Project Contributors
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/usr/bin/env python3
"""
Swarm Behavior - Intelligence Collective Adaptée pour V3
=======================================================
Version adaptée du swarm_behavior.py de DIAMANTS V4
Compatible avec l'architecture ROS2 simplifiée
"""

import math
import time
import random
from typing import Dict, List, Tuple, Optional
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32


class SwarmBehavior:
    """Gestionnaire des comportements sociaux et auto-organisation"""
    
    def __init__(self, drone_id: str, config: Optional[Dict] = None):
        self.drone_id = drone_id
        
        # Configuration par défaut
        self.config = config or self._default_config()
        
        # État social
        self.other_drones = {}
        self.communication_range = self.config.get('communication_range', 5.0)
        self.social_interactions = 0
        
        # Auto-organisation RENFORCÉE pour couverture rapide
        self.exploration_state = 'exploring'  # exploring, following, returning
        self.efficiency = 1.0
        self.recruitment_signal = 0.0
        self.exploration_bias = 0.5 + (hash(drone_id) % 100) / 200.0
        
        # Paramètres d'auto-organisation améliorés
        self.dispersion_force = self.config.get('dispersion_force', 1.5)
        self.coverage_memory = []
        self.territorial_radius = self.config.get('territorial_radius', 8.0)
        
        # Forces sociales
        self.social_force_x = 0.0
        self.social_force_y = 0.0
        self.recruitment_force_x = 0.0
        self.recruitment_force_y = 0.0
        
        # Mémoire spatiale
        self.spatial_memory = []
        self.last_success_time = time.time()
        
        # Métriques
        self.intelligence_score = 0.0
        self.coverage_contribution = 0.0
        
    def _default_config(self) -> Dict:
        """Configuration par défaut compatible V3"""
        return {
            'communication_range': 5.0,
            'dispersion_force': 1.5,
            'territorial_radius': 8.0,
            'safe_distance': 2.0,
            'max_speed': 0.5,
            'exploration_bonus': 2.5,
            'social_learning_rate': 0.01,
            'noise_factor': 0.05
        }
    
    def update_other_drone(self, drone_id: str, position: List[float], 
                          velocity: Optional[List[float]] = None):
        """Mettre à jour position d'un autre drone"""
        self.other_drones[drone_id] = {
            'position': position.copy(),
            'velocity': velocity.copy() if velocity else [0.0, 0.0, 0.0],
            'timestamp': time.time(),
            'last_interaction': time.time()
        }
        
        # Mise à jour interactions sociales
        distance = self._calculate_distance(position[:2], [0.0, 0.0])  # Distance du centre
        if distance < self.communication_range:
            self.social_interactions += 1
    
    def calculate_social_forces(self, current_position: List[float]) -> Tuple[float, float]:
        """Calcul des forces sociales pour auto-organisation"""
        social_x, social_y = 0.0, 0.0
        current_time = time.time()
        
        # Nettoyer drones inactifs
        active_drones = {
            drone_id: data for drone_id, data in self.other_drones.items()
            if current_time - data['timestamp'] < 3.0
        }
        self.other_drones = active_drones
        
        if not self.other_drones:
            return 0.0, 0.0
            
        my_x, my_y = current_position[0], current_position[1]
        
        for drone_id, data in self.other_drones.items():
            other_x, other_y = data['position'][0], data['position'][1]
            
            # Distance entre drones
            dx = other_x - my_x
            dy = other_y - my_y
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < 0.1:  # Éviter division par zéro
                continue
                
            # Force de répulsion (anti-collision)
            safe_distance = self.config.get('safe_distance', 2.0)
            if distance < safe_distance:
                repulsion_strength = self.dispersion_force * (safe_distance - distance) / distance
                social_x -= repulsion_strength * dx
                social_y -= repulsion_strength * dy
            
            # Force d'attraction modérée (cohésion essaim)
            elif distance < self.territorial_radius:
                attraction_strength = 0.1 / distance
                social_x += attraction_strength * dx
                social_y += attraction_strength * dy
        
        # Limitation des forces
        max_force = self.config.get('max_speed', 0.5)
        force_magnitude = math.sqrt(social_x*social_x + social_y*social_y)
        if force_magnitude > max_force:
            social_x = (social_x / force_magnitude) * max_force
            social_y = (social_y / force_magnitude) * max_force
        
        self.social_force_x = social_x
        self.social_force_y = social_y
        
        return social_x, social_y
    
    def calculate_exploration_incentive(self, current_position: List[float]) -> Tuple[float, float]:
        """Calcul incitation à l'exploration (éviter zones déjà couvertes)"""
        my_x, my_y = current_position[0], current_position[1]
        
        # Vérifier si cette zone a été explorée récemment
        exploration_x, exploration_y = 0.0, 0.0
        exploration_bonus = self.config.get('exploration_bonus', 2.5)
        
        # Analyser couverture par les autres drones
        coverage_map = {}
        for drone_id, data in self.other_drones.items():
            pos = data['position']
            grid_x, grid_y = int(pos[0]), int(pos[1])
            coverage_map[(grid_x, grid_y)] = coverage_map.get((grid_x, grid_y), 0) + 1
        
        # Trouver direction vers zone moins couverte
        best_direction = None
        min_coverage = float('inf')
        
        # Échantillonner directions possibles
        for angle in range(0, 360, 45):
            rad = math.radians(angle)
            test_x = my_x + 5.0 * math.cos(rad)
            test_y = my_y + 5.0 * math.sin(rad)
            
            grid_x, grid_y = int(test_x), int(test_y)
            coverage = coverage_map.get((grid_x, grid_y), 0)
            
            if coverage < min_coverage:
                min_coverage = coverage
                best_direction = (math.cos(rad), math.sin(rad))
        
        # Appliquer bonus exploration
        if best_direction and min_coverage < 2:  # Zone peu couverte
            exploration_x = exploration_bonus * best_direction[0] * self.exploration_bias
            exploration_y = exploration_bonus * best_direction[1] * self.exploration_bias
        
        return exploration_x, exploration_y
    
    def calculate_intelligence_score(self) -> float:
        """Calcul score d'intelligence collective du drone"""
        # Facteurs d'intelligence
        social_factor = min(self.social_interactions / 10.0, 1.0)
        coverage_factor = min(self.coverage_contribution / 5.0, 1.0)
        efficiency_factor = self.efficiency
        
        # Score combiné
        self.intelligence_score = (social_factor + coverage_factor + efficiency_factor) / 3.0
        return self.intelligence_score
    
    def update_exploration_state(self, current_position: List[float]) -> str:
        """Mise à jour état d'exploration basé sur context"""
        # Logique simple d'état
        if len(self.other_drones) < 2:
            self.exploration_state = 'exploring'
        elif self.social_interactions > 20:
            self.exploration_state = 'following'
        else:
            self.exploration_state = 'exploring'
            
        return self.exploration_state
    
    def get_swarm_command(self, current_position: List[float], 
                         current_velocity: List[float]) -> Twist:
        """Génère commande basée sur intelligence collective"""
        # Calcul forces
        social_x, social_y = self.calculate_social_forces(current_position)
        explore_x, explore_y = self.calculate_exploration_incentive(current_position)
        
        # Combinaison forces avec pondération
        social_weight = 0.6
        explore_weight = 0.4
        
        total_x = social_weight * social_x + explore_weight * explore_x
        total_y = social_weight * social_y + explore_weight * explore_y
        
        # Ajout bruit adaptatif
        noise = self.config.get('noise_factor', 0.05)
        total_x += random.uniform(-noise, noise)
        total_y += random.uniform(-noise, noise)
        
        # Créer commande Twist
        cmd = Twist()
        cmd.linear.x = total_x
        cmd.linear.y = total_y
        cmd.linear.z = 0.0  # Maintenir altitude
        
        # Mise à jour métriques
        self.calculate_intelligence_score()
        self.update_exploration_state(current_position)
        
        return cmd
    
    def _calculate_distance(self, pos1: List[float], pos2: List[float]) -> float:
        """Calcul distance euclidienne"""
        dx = pos1[0] - pos2[0]
        dy = pos1[1] - pos2[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def get_status_dict(self) -> Dict:
        """Retourne statut pour monitoring"""
        return {
            'drone_id': self.drone_id,
            'exploration_state': self.exploration_state,
            'intelligence_score': self.intelligence_score,
            'social_interactions': self.social_interactions,
            'active_neighbors': len(self.other_drones),
            'social_force': [self.social_force_x, self.social_force_y],
            'efficiency': self.efficiency
        }