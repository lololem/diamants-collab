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
Machine d'état pour la gestion des phases de vol des drones Crazyflie.
Ce module coordonne les différentes phases : attente, décollage, boost,
avancement, navigation.
"""

from enum import Enum
from typing import Optional, Any

try:
    from .config import TAKEOFF_ALTITUDE, ADVANCE_AFTER_TAKEOFF  # type: ignore
except ImportError:
    from config import TAKEOFF_ALTITUDE, ADVANCE_AFTER_TAKEOFF  # type: ignore


class FlightPhase(Enum):
    """Énumération des phases de vol."""
    WAITING = "waiting"
    TAKEOFF = "takeoff"
    BOOST = "boost"
    ADVANCE = "advance"
    NAVIGATION = "navigation"


class StateMachine:
    """Machine d'état pour la gestion des phases de vol."""
    
    def __init__(self, delay: float, start_clock: float) -> None:
        """
        Initialise la machine d'état.
        
        Args:
            delay (float): Délai avant le décollage
            start_clock (float): Temps de démarrage
        """
        self.delay: float = delay
        self.start_clock: float = start_clock
        self.current_phase: FlightPhase = FlightPhase.WAITING
        
        # États internes
        self.takeoff_done: bool = False
        self.advance_after_takeoff: int = 0
        self.wait_for_start: bool = True
        
        # Contrôleur de navigation (optionnel)
        self._navigation_controller: Optional[Any] = None
    
    def update_phase(self, current_time: float,
                     current_z: float) -> FlightPhase:
        """
        Met à jour la phase actuelle basée sur le temps et l'altitude.
        
        Args:
            current_time (float): Temps actuel
            current_z (float): Altitude actuelle
            
        Returns:
            FlightPhase: Phase actuelle
        """
        t_since_start = current_time - self.start_clock
        
        # Phase d'attente
        if self.wait_for_start:
            if t_since_start > self.delay:
                self.wait_for_start = False
                self.current_phase = FlightPhase.TAKEOFF
            else:
                self.current_phase = FlightPhase.WAITING
                return self.current_phase
        
        # Phase de décollage
        if not self.takeoff_done or current_z < TAKEOFF_ALTITUDE - 0.01:
            self.current_phase = FlightPhase.TAKEOFF
            if current_z >= TAKEOFF_ALTITUDE - 0.01:
                self.takeoff_done = True
                self.advance_after_takeoff = ADVANCE_AFTER_TAKEOFF
                self.current_phase = FlightPhase.BOOST
            return self.current_phase
        
        # Phase boost (après décollage)
        if (hasattr(self, '_navigation_controller') and
                self._navigation_controller and
                self._navigation_controller.has_boost_ticks_remaining()):
            self.current_phase = FlightPhase.BOOST
            return self.current_phase
        
        # Phase d'avancement
        if self.advance_after_takeoff > 0:
            self.current_phase = FlightPhase.ADVANCE
            self.advance_after_takeoff -= 1
            return self.current_phase
        
        # Phase de navigation
        self.current_phase = FlightPhase.NAVIGATION
        return self.current_phase
    
    def set_navigation_controller(self, nav_controller: Any) -> None:
        """
        Définit le contrôleur de navigation pour accéder aux ticks
        de boost.
        """
        self._navigation_controller = nav_controller
    
    def get_current_phase(self) -> FlightPhase:
        """Retourne la phase actuelle."""
        return self.current_phase
    
    def is_waiting(self) -> bool:
        """Retourne True si en phase d'attente."""
        return self.current_phase == FlightPhase.WAITING
    
    def is_taking_off(self) -> bool:
        """Retourne True si en phase de décollage."""
        return self.current_phase == FlightPhase.TAKEOFF
    
    def is_boosting(self) -> bool:
        """Retourne True si en phase de boost."""
        return self.current_phase == FlightPhase.BOOST
    
    def is_advancing(self) -> bool:
        """Retourne True si en phase d'avancement."""
        return self.current_phase == FlightPhase.ADVANCE
    
    def is_navigating(self) -> bool:
        """Retourne True si en phase de navigation."""
        return self.current_phase == FlightPhase.NAVIGATION
    
    def get_phase_name(self) -> str:
        """Retourne le nom de la phase actuelle."""
        return self.current_phase.value.upper()
