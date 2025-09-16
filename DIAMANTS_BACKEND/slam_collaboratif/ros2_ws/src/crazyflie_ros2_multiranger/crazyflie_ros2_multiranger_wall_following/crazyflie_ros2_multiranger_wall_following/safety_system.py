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
Système de sécurité avancé pour correction automatique des drones.
Ce module corrige automatiquement les situations dangereuses au lieu d'atterrir.
"""
# flake8: noqa
# pylint: disable=all
# type: ignore

import time
import math
from typing import Tuple, List, Optional
from dataclasses import dataclass
from enum import Enum

# Configuration de sécurité avec valeurs par défaut
try:
    from .config import TAKEOFF_ALTITUDE, K_Z, D_Z  # type: ignore
except ImportError:
    try:
        from config import TAKEOFF_ALTITUDE, K_Z, D_Z  # type: ignore
    except ImportError:
        # Valeurs par défaut de sécurité
        TAKEOFF_ALTITUDE = 0.8
        K_Z = 1.5
        D_Z = 0.8

# Constantes de sécurité avec valeurs par défaut robustes
AUTO_CORRECTION_ENABLED = True
TARGET_ALTITUDE_TOLERANCE = 0.05
AUTO_CORRECTION_GAIN = 1.2
MIN_CORRECTION_THRESHOLD = 0.02
MAX_CORRECTION_DURATION = 5.0
CRITICAL_ALTITUDE_THRESHOLD = 0.2
LOW_ALTITUDE_THRESHOLD = 0.4
HIGH_ALTITUDE_THRESHOLD = 1.5


class SafetyLevel(Enum):
    """Niveaux de sécurité du drone."""

    SAFE = 0
    CAUTION = 1
    WARNING = 2
    CRITICAL = 3
    CORRECTING = 4  # Niveau pour correction active

    def get_name(self) -> str:
        """Retourne le nom du niveau de sécurité."""
        names = {0: "safe", 1: "caution", 2: "warning", 3: "critical", 4: "correcting"}
        return names.get(self.value, "unknown")


@dataclass
class SafetyLimits:
    """Limites de sécurité pour les corrections automatiques"""

    min_altitude: float = CRITICAL_ALTITUDE_THRESHOLD
    max_altitude_correction: float = 1.0
    max_repulsion_force: float = 1.8
    oscillation_threshold: int = 5
    target_altitude: float = TAKEOFF_ALTITUDE


class DroneSecuritySystem:
    """
    Système de sécurité avec correction automatique au lieu d'atterrissage.
    """

    def __init__(self):
        self.limits = SafetyLimits()
        self.altitude_history = []
        self.correction_history = []
        self.safety_events = []

        # État de correction automatique
        self.auto_correction_active = False
        self.correction_start_time = None
        self.target_altitude = TAKEOFF_ALTITUDE

        # Filtres et historique
        self.altitude_filter_alpha = 0.8
        self.filtered_altitude = TAKEOFF_ALTITUDE
        self.oscillation_counter = 0
        self.last_update_time = time.time()

    def update_safety_status(
        self,
        current_altitude: float,
        altitude_correction: float,
        repulsion_force: Tuple[float, float],
        battery_level: Optional[float] = None,
    ) -> SafetyLevel:
        """
        Met à jour et retourne le niveau de sécurité actuel avec correction.
        """
        current_time = time.time()
        dt = current_time - self.last_update_time

        # Mise à jour des historiques
        self.altitude_history.append(current_altitude)
        self.correction_history.append(altitude_correction)

        # Limiter la taille des historiques
        if len(self.altitude_history) > 20:
            self.altitude_history = self.altitude_history[-20:]
        if len(self.correction_history) > 20:
            self.correction_history = self.correction_history[-20:]

        # Mettre à jour l'altitude filtrée
        self.filtered_altitude = (
            self.altitude_filter_alpha * self.filtered_altitude + (1 - self.altitude_filter_alpha) * current_altitude
        )

        # Déterminer le niveau de sécurité
        safety_level = SafetyLevel.SAFE

        # Liste des niveaux de sécurité détectés
        levels = [SafetyLevel.SAFE]

        # Vérification altitude critique avec auto-correction
        altitude_level = self._check_and_correct_altitude(current_altitude)
        levels.append(altitude_level)

        # Vérification de la force de répulsion
        repulsion_level = self._check_repulsion_safety(repulsion_force)
        levels.append(repulsion_level)

        # Vérification oscillations
        if self._check_oscillations():
            levels.append(SafetyLevel.WARNING)

        # Vérification batterie si disponible
        if battery_level is not None:
            battery_safety = self._check_battery_safety(battery_level)
            levels.append(battery_safety)

        # Prendre le niveau de sécurité le plus élevé
        safety_level = max(levels, key=lambda x: x.value)

        self.last_update_time = current_time
        return safety_level

    def _check_and_correct_altitude(self, altitude: float) -> SafetyLevel:
        """Vérifie l'altitude et déclenche la correction automatique."""
        # Altitude critique - nécessite correction immédiate
        if altitude < CRITICAL_ALTITUDE_THRESHOLD:
            if not self.auto_correction_active:
                self._trigger_auto_correction(altitude, "Altitude critique détectée")
            return SafetyLevel.CORRECTING

        # Altitude trop basse
        elif altitude < LOW_ALTITUDE_THRESHOLD:
            if not self.auto_correction_active:
                self._trigger_auto_correction(altitude, "Altitude trop basse")
            return SafetyLevel.WARNING

        # Altitude trop haute
        elif altitude > HIGH_ALTITUDE_THRESHOLD:
            if not self.auto_correction_active:
                self._trigger_auto_correction(altitude, "Altitude excessive")
            return SafetyLevel.WARNING

        # Altitude normale - vérifier si correction terminée
        else:
            # Si correction active et altitude OK, désactiver correction
            if self.auto_correction_active and abs(altitude - self.target_altitude) < TARGET_ALTITUDE_TOLERANCE:
                self._stop_auto_correction()
                return SafetyLevel.SAFE

        return SafetyLevel.SAFE

    def _trigger_auto_correction(self, current_altitude: float, reason: str) -> None:
        """Déclenche la correction automatique."""
        if not AUTO_CORRECTION_ENABLED:
            return

        self.auto_correction_active = True
        self.correction_start_time = time.time()

        # Enregistrer l'événement
        event = {
            'timestamp': time.time(),
            'type': 'auto_correction_triggered',
            'altitude': current_altitude,
            'target_altitude': self.target_altitude,
            'reason': reason,
        }
        self.safety_events.append(event)

    def _stop_auto_correction(self) -> None:
        """Désactive la correction automatique"""
        if self.auto_correction_active:
            self.auto_correction_active = False
            correction_duration = time.time() - self.correction_start_time

            event = {'timestamp': time.time(), 'type': 'auto_correction_stopped', 'duration': correction_duration}
            self.safety_events.append(event)

    def get_altitude_correction(self, current_altitude: float) -> float:
        """
        Calcule la correction d'altitude nécessaire.
        Retourne 0.0 si pas de correction active.
        """
        if not self.auto_correction_active:
            return 0.0

        # Vérifier timeout de correction
        if self.correction_start_time and time.time() - self.correction_start_time > MAX_CORRECTION_DURATION:
            self._stop_auto_correction()
            return 0.0

        # Calculer erreur d'altitude
        error = self.target_altitude - current_altitude

        # Correction proportionnelle avec limitation
        correction = error * AUTO_CORRECTION_GAIN

        # Appliquer seuil minimum et limitation
        if abs(correction) < MIN_CORRECTION_THRESHOLD:
            return 0.0

        return max(-0.5, min(0.5, correction))

    def _check_altitude_safety(self, altitude: float) -> SafetyLevel:
        """Vérifie la sécurité des corrections d'altitude"""
        if altitude < self.limits.min_altitude:
            return SafetyLevel.CRITICAL
        elif altitude < 0.1:
            return SafetyLevel.WARNING
        elif altitude > 1.0:
            return SafetyLevel.WARNING
        else:
            return SafetyLevel.SAFE

    def _check_repulsion_safety(self, repulsion_force: Tuple[float, float]) -> SafetyLevel:
        """Vérifie la sécurité de la force de répulsion."""
        force_magnitude = math.sqrt(repulsion_force[0] ** 2 + repulsion_force[1] ** 2)

        if force_magnitude > self.limits.max_repulsion_force * 1.5:
            return SafetyLevel.CRITICAL
        elif force_magnitude > self.limits.max_repulsion_force:
            return SafetyLevel.WARNING
        else:
            return SafetyLevel.SAFE

    def _check_oscillations(self) -> bool:
        """Détecte les oscillations dangereuses dans l'altitude."""
        if len(self.altitude_history) < 6:
            return False

        # Analyser les 6 dernières valeurs d'altitude
        recent_altitudes = self.altitude_history[-6:]
        variations = []
        for i in range(1, len(recent_altitudes)):
            variations.append(recent_altitudes[i] - recent_altitudes[i - 1])

        # Détecter les changements de direction fréquents
        direction_changes = 0
        for i in range(1, len(variations)):
            if (variations[i] * variations[i - 1]) < 0:
                direction_changes += 1

        return direction_changes >= self.limits.oscillation_threshold

    def _check_battery_safety(self, battery_level: float) -> SafetyLevel:
        """Évalue le niveau de sécurité de la batterie."""
        if battery_level < 0.15:  # 15%
            return SafetyLevel.CRITICAL
        elif battery_level < 0.25:  # 25%
            return SafetyLevel.WARNING
        elif battery_level < 0.35:  # 35%
            return SafetyLevel.CAUTION
        else:
            return SafetyLevel.SAFE

    def should_trigger_emergency_landing(self) -> bool:
        """
        Détermine si un atterrissage d'urgence doit être déclenché.
        Dans le nouveau système, retourne toujours False.
        """
        return False

    def get_safety_summary(self) -> dict:
        """Retourne un résumé de l'état de sécurité."""
        return {
            'auto_correction_active': self.auto_correction_active,
            'correction_start_time': self.correction_start_time,
            'target_altitude': self.target_altitude,
            'filtered_altitude': self.filtered_altitude,
            'oscillation_counter': self.oscillation_counter,
            'safety_events_count': len(self.safety_events),
            'altitude_history_size': len(self.altitude_history),
        }

    def clear_history(self) -> None:
        """Nettoie l'historique pour libérer la mémoire."""
        self.altitude_history.clear()
        self.correction_history.clear()
        # Garder seulement les 10 derniers événements de sécurité
        if len(self.safety_events) > 10:
            self.safety_events = self.safety_events[-10:]
