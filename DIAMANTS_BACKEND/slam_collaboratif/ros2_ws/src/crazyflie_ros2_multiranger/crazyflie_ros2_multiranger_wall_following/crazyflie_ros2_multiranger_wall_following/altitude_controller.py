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
Contrôleur d'altitude avec régulation PD et filtrage passe-bas.
Ce module gère la stabilisation en altitude des drones Crazyflie.
"""

import time
from typing import Dict, Any

try:
    from .config import K_Z, D_Z, ALPHA_Z, MAX_ALTITUDE_SPEED, DEADBAND_Z
except ImportError:
    # Import absolu si l'import relatif échoue
    from config import (K_Z, D_Z, ALPHA_Z, MAX_ALTITUDE_SPEED,  # type: ignore
                        DEADBAND_Z)  # type: ignore


class AltitudeController:
    """Contrôleur d'altitude PD avec filtre passe-bas."""
    
    def __init__(self, target_altitude: float) -> None:
        """
        Initialise le contrôleur d'altitude.
        
        Args:
            target_altitude (float): Altitude cible en mètres
        """
        self.target_z: float = target_altitude
        self.last_z: float = 0.0
        self.last_time: float = time.time()
        self.filtered_altitude_correction: float = 0.0
    
    def update_target(self, new_target: float) -> None:
        """Met à jour l'altitude cible."""
        self.target_z = new_target
    
    def compute_correction(self, current_z: float) -> float:
        """
        Calcule la correction d'altitude nécessaire.
        
        Args:
            current_z (float): Altitude actuelle en mètres
            
        Returns:
            float: Correction d'altitude (vitesse verticale)
        """
        current_time = time.time()
        dt = max(current_time - self.last_time, 1e-3)
        
        # Calcul de la dérivée de l'altitude
        dz = (current_z - self.last_z) / dt
        
        # Erreur d'altitude
        error_z = self.target_z - current_z
        
        # Correction PD brute
        raw_altitude_correction = K_Z * error_z - D_Z * dz
        
        # DEADZONE: si très proche, correction nulle
        if abs(error_z) < DEADBAND_Z:
            raw_altitude_correction = 0.0
        
        # Filtrage passe-bas
        self.filtered_altitude_correction = (
            ALPHA_Z * raw_altitude_correction +
            (1 - ALPHA_Z) * self.filtered_altitude_correction
        )
        
        # Clipping de la correction
        altitude_correction = max(
            min(self.filtered_altitude_correction, MAX_ALTITUDE_SPEED),
            -MAX_ALTITUDE_SPEED
        )
        
        # Mise à jour des valeurs précédentes
        self.last_z = current_z
        self.last_time = current_time
        
        return altitude_correction
    
    def get_debug_info(self, current_z: float) -> Dict[str, Any]:
        """
        Retourne les informations de debug du contrôleur.
        
        Args:
            current_z (float): Altitude actuelle
            
        Returns:
            dict: Informations de debug
        """
        current_time = time.time()
        dt = max(current_time - self.last_time, 1e-3)
        dz = (current_z - self.last_z) / dt
        error_z = self.target_z - current_z
        
        return {
            'current_z': current_z,
            'target_z': self.target_z,
            'error_z': error_z,
            'dz': dz,
            'filtered_correction': self.filtered_altitude_correction
        }
