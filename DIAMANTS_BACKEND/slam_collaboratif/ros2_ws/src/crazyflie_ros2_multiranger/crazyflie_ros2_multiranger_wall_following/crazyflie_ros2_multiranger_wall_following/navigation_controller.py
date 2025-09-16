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
Contrôleur de navigation et de mouvement pour les drones Crazyflie.
Ce module gère les mouvements aléatoires, les rotations et les déplacements.
"""

import random
import math
from geometry_msgs.msg import Twist  # type: ignore

try:
    from .config import (  # type: ignore
        RANDOM_WALK_MIN_VX, RANDOM_WALK_MAX_VX_RANGE,
        RANDOM_WALK_MIN_VY, RANDOM_WALK_MAX_VY,
        RANDOM_WALK_MIN_YAW, RANDOM_WALK_MAX_YAW,
        RANDOM_WALK_MIN_INTERVAL, RANDOM_WALK_MAX_INTERVAL_RANGE,
        RANDOM_WALK_INITIAL_INTERVAL_BASE,
        START_BOOST_FORWARD_SPEED, START_BOOST_LATERAL_SPEED,
        TURN_DEGREES, TURN_DURATION
    )
except ImportError:
    from config import (  # type: ignore
        RANDOM_WALK_MIN_VX, RANDOM_WALK_MAX_VX_RANGE,
        RANDOM_WALK_MIN_VY, RANDOM_WALK_MAX_VY,
        RANDOM_WALK_MIN_YAW, RANDOM_WALK_MAX_YAW,
        RANDOM_WALK_MIN_INTERVAL, RANDOM_WALK_MAX_INTERVAL_RANGE,
        RANDOM_WALK_INITIAL_INTERVAL_BASE,
        START_BOOST_FORWARD_SPEED, START_BOOST_LATERAL_SPEED,
        TURN_DEGREES, TURN_DURATION
    )


class NavigationController:
    """Contrôleur de navigation et de mouvement."""
    
    def __init__(self, max_forward_speed: float, start_direction: str) -> None:
        """
        Initialise le contrôleur de navigation.
        
        Args:
            max_forward_speed (float): Vitesse avant maximale
            start_direction (str): Direction de départ ('right' ou 'left')
        """
        self.max_forward_speed: float = max_forward_speed
        self.start_direction: str = start_direction
        
        # Attributs de mouvement aléatoire (initialisés par reset_random_walk)
        self.random_walk_timer: float = 0.0
        self.random_walk_interval: float = 0.0
        self.random_walk_vx: float = 0.0
        self.random_walk_y: float = 0.0
        self.random_walk_yaw: float = 0.0
        
        # État du mouvement aléatoire
        self.reset_random_walk()
        
        # État de rotation
        self.turning: bool = False
        self.turn_ticks: int = 0
        
        # État du boost initial
        self.start_boost_ticks: int = 15
    
    def reset_random_walk(self) -> None:
        """Réinitialise les paramètres du mouvement aléatoire."""
        self.random_walk_timer = 0
        self.random_walk_interval = (RANDOM_WALK_INITIAL_INTERVAL_BASE +
                                     random.random())
        self.random_walk_vx = (RANDOM_WALK_MIN_VX +
                               RANDOM_WALK_MAX_VX_RANGE * random.random())
        self.random_walk_y = 0.0
        self.random_walk_yaw = random.uniform(RANDOM_WALK_MIN_YAW,
                                              RANDOM_WALK_MAX_YAW)
    
    def update_random_walk(self, current_time: float) -> None:
        """
        Met à jour les paramètres du mouvement aléatoire si nécessaire.
        
        Args:
            current_time (float): Temps actuel
        """
        if current_time - self.random_walk_timer > self.random_walk_interval:
            self.random_walk_vx = (RANDOM_WALK_MIN_VX +
                                   RANDOM_WALK_MAX_VX_RANGE * random.random())
            self.random_walk_y = random.uniform(RANDOM_WALK_MIN_VY,
                                                RANDOM_WALK_MAX_VY)
            self.random_walk_yaw = 0.0
            self.random_walk_timer = current_time
            self.random_walk_interval = (RANDOM_WALK_MIN_INTERVAL +
                                         RANDOM_WALK_MAX_INTERVAL_RANGE *
                                         random.random())
    
    def create_takeoff_command(self, takeoff_rate):
        """
        Crée une commande de décollage.
        
        Args:
            takeoff_rate (float): Vitesse de décollage
            
        Returns:
            Twist: Commande de mouvement
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = takeoff_rate
        msg.angular.z = 0.0
        return msg
    
    def create_boost_command(self, altitude_correction):
        """
        Crée une commande de boost initial.
        
        Args:
            altitude_correction (float): Correction d'altitude
            
        Returns:
            Twist: Commande de mouvement
        """
        msg = Twist()
        msg.linear.x = START_BOOST_FORWARD_SPEED
        lateral_speed = START_BOOST_LATERAL_SPEED
        msg.linear.y = (lateral_speed if self.start_direction == 'right'
                        else -lateral_speed)
        msg.linear.z = altitude_correction
        msg.angular.z = 0.0
        
        self.start_boost_ticks -= 1
        return msg
    
    def create_advance_command(self, altitude_correction):
        """
        Crée une commande d'avancement droit.
        
        Args:
            altitude_correction (float): Correction d'altitude
            
        Returns:
            Twist: Commande de mouvement
        """
        msg = Twist()
        msg.linear.x = self.max_forward_speed
        msg.linear.y = 0.0
        msg.linear.z = altitude_correction
        msg.angular.z = 0.0
        return msg
    
    def create_turn_command(self, altitude_correction):
        """
        Crée une commande de rotation.
        
        Args:
            altitude_correction (float): Correction d'altitude
            
        Returns:
            Twist: Commande de mouvement
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = altitude_correction
        
        angle_rad = math.radians(TURN_DEGREES)
        turn_direction = (angle_rad if self.start_direction == 'right'
                          else -angle_rad)
        msg.angular.z = turn_direction
        
        self.turn_ticks -= 1
        if self.turn_ticks <= 0:
            self.turning = False
        
        return msg
    
    def create_random_walk_command(self, altitude_correction, current_time):
        """
        Crée une commande de mouvement aléatoire.
        
        Args:
            altitude_correction (float): Correction d'altitude
            current_time (float): Temps actuel
            
        Returns:
            Twist: Commande de mouvement
        """
        self.update_random_walk(current_time)
        
        msg = Twist()
        msg.linear.x = self.random_walk_vx
        msg.linear.y = self.random_walk_y
        msg.linear.z = altitude_correction
        msg.angular.z = self.random_walk_yaw
        return msg
    
    def start_turn(self):
        """Démarre une séquence de rotation."""
        self.turning = True
        self.turn_ticks = TURN_DURATION
    
    def is_turning(self):
        """Retourne True si le drone est en train de tourner."""
        return self.turning
    
    def has_boost_ticks_remaining(self):
        """Retourne True s'il reste des ticks de boost."""
        return self.start_boost_ticks > 0
    
    def set_random_walk_timer(self, current_time):
        """Met à jour le timer du random walk."""
        self.random_walk_timer = current_time
