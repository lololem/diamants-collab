# DIAMANTS V3 - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
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
Système d'évitement d'obstacles et de répulsion.
Ce module gère l'évitement des obstacles statiques (arbres) et dynamiques
(autres drones).
"""

import math
from typing import Dict, List, Tuple, Any

try:
    from .config import (
        REPULSION_DIST, REPULSION_GAIN,
        REPULSION_DRONE_DIST, REPULSION_DRONE_GAIN, MIN_DRONE_DISTANCE
    )
    from .advanced_collision_manager import CollisionController
except ImportError:
    from config import (  # type: ignore
        REPULSION_DIST, REPULSION_GAIN,
        REPULSION_DRONE_DIST, REPULSION_DRONE_GAIN, MIN_DRONE_DISTANCE
    )
    try:
        from advanced_collision_manager import CollisionController  # type: ignore
    except ImportError:
        CollisionController = None


class ObstacleAvoidance:
    """Gestionnaire d'évitement d'obstacles et de répulsion."""
    
    def __init__(self, drone_id: str = "default", use_advanced: bool = True) -> None:
        """
        Initialise le système d'évitement d'obstacles.
        
        Args:
            drone_id (str): Identifiant du drone
            use_advanced (bool): Utiliser le système de collision avancé
        """
        self.other_positions: Dict[str, Any] = {}
        self.use_advanced = use_advanced and CollisionController is not None
        
        if self.use_advanced:
            self.advanced_controller = CollisionController(drone_id)
        else:
            self.advanced_controller = None
    
    def update_other_positions(self, positions_dict: Dict[str, Any]) -> None:
        """
        Met à jour les positions des autres drones.
        
        Args:
            positions_dict (Dict[str, Any]): Dictionnaire des positions
                des autres drones
        """
        self.other_positions = positions_dict
        
        # Mettre à jour le système avancé si disponible
        if self.use_advanced and self.advanced_controller:
            self.advanced_controller.update_statistics(positions_dict)
    
    def compute_obstacle_repulsion(
            self, ranges: List[float]) -> Tuple[float, float, float]:
        """
        Calcule la répulsion due aux obstacles statiques (arbres).
        
        Args:
            ranges (list): Distances mesurées [back, right, front, left]
            
        Returns:
            tuple: (repulsion_x, repulsion_y, repulsion_z)
        """
        repulsion_x = 0.0
        repulsion_y = 0.0
        repulsion_z = 0.0
        
        if len(ranges) >= 4:
            back_range, right_range, front_range, left_range = ranges[:4]
            
            # Répulsion avant/arrière
            if front_range < REPULSION_DIST:
                repulsion_x -= REPULSION_GAIN * (REPULSION_DIST - front_range)
                # Si le drone est trop proche d'un obstacle frontal, le faire s'élever légèrement
                repulsion_z += 0.3 * (REPULSION_DIST - front_range)
            if back_range < REPULSION_DIST:
                repulsion_x += REPULSION_GAIN * (REPULSION_DIST - back_range)
            
            # Répulsion droite/gauche
            if right_range < REPULSION_DIST:
                repulsion_y += REPULSION_GAIN * (REPULSION_DIST - right_range)
            if left_range < REPULSION_DIST:
                repulsion_y -= REPULSION_GAIN * (REPULSION_DIST - left_range)
        
        return repulsion_x, repulsion_y, repulsion_z
    
    def compute_drone_repulsion(
            self, current_position: List[float]) -> Tuple[float, float]:
        """
        Calcule la répulsion due aux autres drones.
        
        Args:
            current_position (list): Position actuelle [x, y, z]
            
        Returns:
            tuple: (repulsion_x, repulsion_y)
        """
        repulsion_x = 0.0
        repulsion_y = 0.0
        
        for pos in self.other_positions.values():
            if pos and None not in pos:
                dx = current_position[0] - pos[0]
                dy = current_position[1] - pos[1]
                dist = math.hypot(dx, dy)
                
                if MIN_DRONE_DISTANCE < dist < REPULSION_DRONE_DIST:
                    strength = (REPULSION_DRONE_GAIN *
                                (REPULSION_DRONE_DIST - dist))
                    repulsion_x += strength * (dx / dist)
                    repulsion_y += strength * (dy / dist)
        
        return repulsion_x, repulsion_y
    
    def compute_total_repulsion(
            self, current_position: List[float],
            ranges: List[float]) -> Tuple[float, float, float]:
        """
        Calcule la répulsion totale (obstacles + drones).
        
        Args:
            current_position (list): Position actuelle [x, y, z]
            ranges (list): Distances mesurées [back, right, front, left]
            
        Returns:
            tuple: (total_repulsion_x, total_repulsion_y, total_repulsion_z)
        """
        # Répulsion obstacles
        obs_x, obs_y, obs_z = self.compute_obstacle_repulsion(ranges)
        
        # Répulsion drones
        drone_x, drone_y = self.compute_drone_repulsion(current_position)
        
        return obs_x + drone_x, obs_y + drone_y, obs_z
    
    def compute_advanced_avoidance(
            self, current_position: List[float],
            ranges: List[float]) -> Tuple[float, float, float]:
        """
        Calcule l'évitement avancé en utilisant le gestionnaire de collision.
        
        Args:
            current_position (list): Position actuelle [x, y, z]
            ranges (list): Distances mesurées [back, right, front, left]
            
        Returns:
            tuple: (avoidance_x, avoidance_y, avoidance_z)
        """
        if self.use_advanced and self.advanced_controller:
            # Utiliser le gestionnaire de collision avancé
            try:
                return self.advanced_controller.compute_avoidance(
                    current_position, ranges, self.other_positions)
            except (AttributeError, TypeError, ValueError):
                # Fallback sur la répulsion standard en cas d'erreur
                return self.compute_total_repulsion(current_position, ranges)
        else:
            # Utiliser la répulsion standard
            return self.compute_total_repulsion(current_position, ranges)
    
    def is_obstacle_detected(
            self, ranges: List[float],
            detection_distance: float) -> bool:
        """
        Vérifie si un obstacle est détecté à une distance donnée.
        
        Args:
            ranges (list): Distances mesurées
            detection_distance (float): Distance de détection
            
        Returns:
            bool: True si obstacle détecté
        """
        if len(ranges) >= 3:  # Au moins front range disponible
            front_range = ranges[2]
            return front_range < detection_distance
        return False
