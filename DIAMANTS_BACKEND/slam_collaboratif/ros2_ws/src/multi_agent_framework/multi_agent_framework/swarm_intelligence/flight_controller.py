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
Flight Controller V3 - Gestionnaire Vol Intelligent
==================================================
Version adaptée du flight_controller.py de DIAMANTS V4
Compatible avec architecture ROS2 simplifiée
"""

import time
import math
import numpy as np
from typing import List, Tuple, Dict, Optional
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32


class FlightController:
    """Contrôleur de vol intelligent et sécurisé"""
    
    def __init__(self, drone_id: str, config: Optional[Dict] = None):
        self.drone_id = drone_id
        self.config = config or self._default_config()
        
        # Paramètres vol
        self.target_altitude = self.config.get('target_altitude', 1.0)
        self.altitude_tolerance = self.config.get('altitude_tolerance', 0.1)
        self.max_speed = self.config.get('max_speed', 0.5)
        
        # Machine d'état
        self.phase = 'waiting'  # waiting, takeoff, cruise, navigate, landing, emergency
        self.takeoff_rate = self.config.get('takeoff_rate', 0.3)
        self.start_time = None
        self.phase_start_time = time.time()
        
        # Délais séquentiels
        self.startup_delay = 0.0
        self.takeoff_delay = 0.0
        
        # Position et état
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.is_active = False
        self.last_command = Twist()
        
        # Sécurité
        self.emergency_stop = False
        self.last_safe_position = [0.0, 0.0, 0.0]
        self.collision_risk = 0.0
        
        # Historique pour lissage
        self.position_history = []
        self.command_history = []
        
    def _default_config(self) -> Dict:
        """Configuration par défaut"""
        return {
            'target_altitude': 1.0,
            'altitude_tolerance': 0.1,
            'max_speed': 0.5,
            'takeoff_rate': 0.3,
            'landing_rate': 0.2,
            'emergency_descent_rate': 0.5,
            'position_smoothing': 0.8,
            'command_smoothing': 0.7,
            'safety_margin': 0.5
        }
    
    def set_startup_delay(self, delay: float):
        """Définir délai de démarrage (séquencement anti-collision)"""
        self.startup_delay = delay
    
    def set_takeoff_delay(self, delay: float):
        """Définir délai de décollage"""
        self.takeoff_delay = delay
    
    def update_position(self, position: List[float], velocity: Optional[List[float]] = None):
        """Mettre à jour position et vitesse"""
        # Lissage position
        if self.position_history:
            smoothing = self.config.get('position_smoothing', 0.8)
            self.position[0] = smoothing * self.position[0] + (1-smoothing) * position[0]
            self.position[1] = smoothing * self.position[1] + (1-smoothing) * position[1]
            self.position[2] = smoothing * self.position[2] + (1-smoothing) * position[2]
        else:
            self.position = position.copy()
        
        # Mettre à jour vitesse
        if velocity:
            self.velocity = velocity.copy()
        
        # Historique pour analyse
        self.position_history.append(self.position.copy())
        if len(self.position_history) > 10:
            self.position_history.pop(0)
        
        # Sauvegarder position sûre
        if not self.emergency_stop and self.collision_risk < 0.3:
            self.last_safe_position = self.position.copy()
    
    def update_phase(self) -> str:
        """Mise à jour automatique phase de vol"""
        current_time = time.time()
        current_z = self.position[2]
        phase_duration = current_time - self.phase_start_time
        
        # Machine d'état
        if self.phase == 'waiting':
            if self.start_time and current_time - self.start_time >= self.startup_delay:
                self._transition_to_phase('takeoff')
                
        elif self.phase == 'takeoff':
            if current_z >= self.target_altitude - self.altitude_tolerance:
                self._transition_to_phase('cruise')
            elif phase_duration > 30.0:  # Timeout décollage
                self._transition_to_phase('emergency')
                
        elif self.phase == 'cruise':
            # Stabilisation à l'altitude cible
            if abs(current_z - self.target_altitude) > self.altitude_tolerance * 2:
                self._transition_to_phase('takeoff')  # Re-ajustement
            elif phase_duration > 5.0:  # Stabilisé
                self._transition_to_phase('navigate')
                
        elif self.phase == 'navigate':
            # Phase normale - continuer navigation
            if abs(current_z - self.target_altitude) > self.altitude_tolerance * 3:
                self._transition_to_phase('cruise')  # Re-stabilisation
                
        elif self.phase == 'emergency':
            # Mode d'urgence - atterrissage
            if current_z <= 0.2:
                self._transition_to_phase('waiting')
                self.emergency_stop = False
        
        return self.phase
    
    def _transition_to_phase(self, new_phase: str):
        """Transition sécurisée entre phases"""
        old_phase = self.phase
        self.phase = new_phase
        self.phase_start_time = time.time()
        
        # Actions de transition
        if new_phase == 'takeoff':
            self.is_active = True
        elif new_phase == 'emergency':
            self.emergency_stop = True
            
    def get_altitude_command(self) -> float:
        """Commande d'altitude basée sur phase actuelle"""
        current_z = self.position[2]
        
        if self.phase == 'waiting':
            return 0.0
            
        elif self.phase == 'takeoff':
            # Décollage progressif
            altitude_error = self.target_altitude - current_z
            if altitude_error > 0:
                return min(self.takeoff_rate, altitude_error * 2.0)
            return 0.0
            
        elif self.phase == 'cruise' or self.phase == 'navigate':
            # Maintien altitude avec correction
            altitude_error = self.target_altitude - current_z
            correction = altitude_error * 0.5  # Gain proportionnel
            return max(-0.2, min(0.2, correction))  # Limitation
            
        elif self.phase == 'emergency':
            # Descente d'urgence contrôlée
            if current_z > 0.3:
                return -self.config.get('emergency_descent_rate', 0.5)
            return 0.0
            
        return 0.0
    
    def apply_safety_limits(self, command: Twist) -> Twist:
        """Application des limites de sécurité"""
        safe_cmd = Twist()
        
        # Mode d'urgence
        if self.emergency_stop:
            safe_cmd.linear.x = 0.0
            safe_cmd.linear.y = 0.0
            safe_cmd.linear.z = self.get_altitude_command()
            return safe_cmd
        
        # Limitation vitesse horizontale
        horizontal_speed = math.sqrt(command.linear.x**2 + command.linear.y**2)
        if horizontal_speed > self.max_speed:
            scale = self.max_speed / horizontal_speed
            safe_cmd.linear.x = command.linear.x * scale
            safe_cmd.linear.y = command.linear.y * scale
        else:
            safe_cmd.linear.x = command.linear.x
            safe_cmd.linear.y = command.linear.y
        
        # Commande altitude
        safe_cmd.linear.z = self.get_altitude_command()
        
        # Lissage commandes
        if self.command_history:
            smoothing = self.config.get('command_smoothing', 0.7)
            last_cmd = self.command_history[-1]
            safe_cmd.linear.x = smoothing * last_cmd.linear.x + (1-smoothing) * safe_cmd.linear.x
            safe_cmd.linear.y = smoothing * last_cmd.linear.y + (1-smoothing) * safe_cmd.linear.y
        
        # Historique
        self.command_history.append(safe_cmd)
        if len(self.command_history) > 5:
            self.command_history.pop(0)
        
        self.last_command = safe_cmd
        return safe_cmd
    
    def set_emergency_stop(self, emergency: bool = True):
        """Activer/désactiver arrêt d'urgence"""
        if emergency and not self.emergency_stop:
            self._transition_to_phase('emergency')
        self.emergency_stop = emergency
    
    def set_collision_risk(self, risk: float):
        """Mettre à jour niveau de risque collision"""
        self.collision_risk = max(0.0, min(1.0, risk))
        
        # Urgence si risque critique
        if self.collision_risk > 0.8:
            self.set_emergency_stop(True)
    
    def start_mission(self):
        """Démarrer mission (active la séquence)"""
        self.start_time = time.time()
        self.is_active = True
    
    def is_ready_for_navigation(self) -> bool:
        """Vérifier si prêt pour navigation"""
        return (self.phase == 'navigate' and 
                abs(self.position[2] - self.target_altitude) < self.altitude_tolerance and
                not self.emergency_stop)
    
    def get_status_dict(self) -> Dict:
        """État complet pour monitoring"""
        return {
            'drone_id': self.drone_id,
            'phase': self.phase,
            'position': self.position.copy(),
            'velocity': self.velocity.copy(),
            'target_altitude': self.target_altitude,
            'is_active': self.is_active,
            'emergency_stop': self.emergency_stop,
            'collision_risk': self.collision_risk,
            'ready_for_navigation': self.is_ready_for_navigation(),
            'phase_duration': time.time() - self.phase_start_time
        }