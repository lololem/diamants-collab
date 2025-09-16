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
Module avancé de gestion des collisions entre drones.
Ce module fournit des classes pour gérer la détection de collisions
entre drones et éviter les collisions, notamment lors d'opérations
en essaim et de navigation d'évitement d'obstacles.
"""

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Any

# Note: Config imports disponibles si nécessaires plus tard
# try:
#     from .config import (
#         REPULSION_DIST, REPULSION_GAIN,
#         REPULSION_DRONE_DIST, REPULSION_DRONE_GAIN
#     )
# except ImportError:
#     from config import (  # type: ignore
#         REPULSION_DIST, REPULSION_GAIN,
#         REPULSION_DRONE_DIST, REPULSION_DRONE_GAIN
#     )


class RiskLevel(Enum):
    """Niveau de risque de collision défini selon la distance."""

    SAFE = "safe"
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class Threat:
    """Représente une menace à un autre drone ou obstacle."""

    position: List[float]  # [x, y, z]
    velocity: List[float]  # [vx, vy, vz]
    timestamp: float
    drone_id: str


@dataclass
class FlightZone:
    """Informations sur une zone de vol d'un drone."""

    level: RiskLevel
    position: float
    next_position: List[float]  # [x, y, z]
    vector: List[float]  # [vx, vy, vz]
    zone_id: str


class CollisionManager:
    """Gestionnaire avancé des collisions entre drones et obstacles."""

    def __init__(self, drone_id: str) -> None:
        """
        Initialise le gestionnaire de collisions.

        Args:
            drone_id (str): Identifiant unique du drone
        """
        self.drone_id = drone_id
        self.threats: Dict[str, Threat] = {}
        self.history: List[FlightZone] = []
        self.last_update = time.time()

        # Paramètres de collision configurés
        self.collision_distance = 3.0  # mètres
        self.safe_distance = 0.8  # mètres
        self.warning_distance = 0.4  # mètres
        self.urgent_distance = 5.0  # mètres

        # Paramètres d'intensité d'évitement
        self.risk_factors = {
            'critical': 0.15,  # Distance critique d'évitement
            'danger': 0.25,  # Distance de danger
            'warning': 0.40,  # Distance d'alerte
            'caution': 0.60,  # Distance de précaution
        }

        # Paramètres d'intensité des actions
        self.action_intensity = {'critical': 2.5, 'danger': 1.8, 'warning': 1.2, 'caution': 0.6}

    def track_threat(
        self, drone_id: str, position: List[float], velocity: List[float], own_position: Optional[List[float]] = None
    ) -> None:
        """
        Met à jour les informations d'un autre drone.

        Args:
            drone_id (str): Identifiant du drone
            position (List[float]): Position actuelle [x, y, z]
            velocity (List[float]): Vitesse actuelle [vx, vy, vz]
            own_position (Optional[List[float]]): Position de ce drone pour
                calculer la distance
        """
        current_time = time.time()

        # Log des nouvelles menaces ou mises à jour importantes
        is_new_threat = drone_id not in self.threats
        if is_new_threat:
            print(f"[COLLISION_MANAGER] {self.drone_id}: " f"Nouvelle menace détectée - {drone_id} à {position}")

        # Vérifier si le drone est proche (distance < 2m) si on a notre
        # position
        if len(position) >= 3 and own_position and len(own_position) >= 3:
            distance = math.sqrt(
                (position[0] - own_position[0]) ** 2
                + (position[1] - own_position[1]) ** 2
                + (position[2] - own_position[2]) ** 2
            )
            print(f"[COLLISION_MANAGER] {self.drone_id}: " f"Tracking {drone_id} à distance {distance:.2f}m")
            if distance < 2.0:
                print(f"[COLLISION_MANAGER] {self.drone_id}: " f"ATTENTION! Drone {drone_id} proche à {distance:.2f}m")
        elif len(position) >= 3:
            # Fallback: distance depuis origine (pour compatibilité)
            distance = math.sqrt(position[0] ** 2 + position[1] ** 2 + position[2] ** 2)
            print(
                f"[COLLISION_MANAGER] {self.drone_id}: "
                f"Tracking {drone_id} à position {position} "
                f"(dist origine: {distance:.2f}m)"
            )

        self.threats[drone_id] = Threat(position=position, velocity=velocity, timestamp=current_time, drone_id=drone_id)
        self.last_update = current_time

        # Nettoyer les anciennes entrées
        self.cleanup_threats()

    def cleanup_threats(self) -> None:
        """Nettoie les anciennes entrées de menaces expirées."""
        current_time = time.time()
        expired_threats = [
            drone_id
            for drone_id, threat in self.threats.items()
            if current_time - threat.timestamp > self.urgent_distance
        ]
        for drone_id in expired_threats:
            del self.threats[drone_id]

    def detect_collision(
        self, other_threat: Threat, position: List[float], velocity: List[float]
    ) -> Optional[FlightZone]:
        """
        Détecte une collision potentielle avec un autre drone.

        Args:
            other_threat (Threat): Représente l'autre drone
            position (List[float]): Position actuelle
            velocity (List[float]): Vitesse actuelle

        Returns:
            Optional[FlightZone]: Informations de collision si détectée
        """
        print(f"[COLLISION_DETECT] {self.drone_id}: " f"Analyse collision avec {other_threat.drone_id}")
        print(f"[COLLISION_DETECT] {self.drone_id}: " f"Notre pos={position}, leur pos={other_threat.position}")
        print(f"[COLLISION_DETECT] {self.drone_id}: " f"Notre vel={velocity}, leur vel={other_threat.velocity}")

        # Calculer les distances et les vitesses relatives
        rel_pos = (
            other_threat.position[0] - position[0],
            other_threat.position[1] - position[1],
            other_threat.position[2] - position[2],
        )

        rel_vel = (
            other_threat.velocity[0] - velocity[0],
            other_threat.velocity[1] - velocity[1],
            other_threat.velocity[2] - velocity[2],
        )

        # Calculer la distance actuelle
        current_distance = math.sqrt(rel_pos[0] ** 2 + rel_pos[1] ** 2 + rel_pos[2] ** 2)

        # Log détaillé pour le debug des collisions
        print(
            f"[COLLISION_DETECT] {self.drone_id} vs "
            f"{other_threat.drone_id}: "
            f"distance={current_distance:.2f}m, rel_pos={rel_pos}, "
            f"rel_vel={rel_vel}"
        )

        # Si les drones s'éloignent, pas de collision
        dot_product = rel_pos[0] * rel_vel[0] + rel_pos[1] * rel_vel[1] + rel_pos[2] * rel_vel[2]

        print(f"[COLLISION_DETECT] {self.drone_id}: " f"dot_product={dot_product:.3f} (>0 = s'éloignent)")

        # Cas spécial : vitesses relatives nulles (drones statiques) ou qui
        # s'éloignent lentement
        if abs(dot_product) < 1e-6:  # Pratiquement zéro
            print(
                f"[COLLISION_DETECT] {self.drone_id}: "
                f"Vitesses relatives nulles, "
                f"distance={current_distance:.2f}m"
            )
            if current_distance < self.safe_distance:
                print(
                    f"[COLLISION_DETECT] {self.drone_id}: "
                    f"ALERTE! Drones statiques trop proches "
                    f"({current_distance:.2f}m < "
                    f"{self.safe_distance:.2f}m)"
                )
                # Créer une zone de collision immédiate
                return FlightZone(
                    level=RiskLevel.HIGH,
                    position=0.0,  # Collision immédiate
                    next_position=list(position),
                    vector=self.calculate_avoidance_vector(list(rel_pos), [0.0, 0.0, 0.0], RiskLevel.HIGH),
                    zone_id="statique_proche",
                )
            return None

        if dot_product > 0:  # S'éloignent
            if current_distance < 1.0:  # Mais toujours proches
                print(
                    f"[COLLISION_DETECT] {self.drone_id}: "
                    f"Drones s'éloignent mais proches "
                    f"({current_distance:.2f}m)"
                )
            return None

        # Calculer du temps de collision le plus proche
        rel_speed_sq = rel_vel[0] ** 2 + rel_vel[1] ** 2 + rel_vel[2] ** 2
        print(f"[COLLISION_DETECT] {self.drone_id}: " f"rel_speed_sq={rel_speed_sq:.6f}")

        if rel_speed_sq < 1e-6:  # Vitesses relatives trop faibles
            print(
                f"[COLLISION_DETECT] {self.drone_id}: "
                f"Vitesses relatives faibles, "
                f"distance={current_distance:.2f}m"
            )
            if current_distance < self.safe_distance:
                print(
                    f"[COLLISION_DETECT] {self.drone_id}: "
                    f"ALERTE! Drones statiques trop proches "
                    f"({current_distance:.2f}m < "
                    f"{self.safe_distance:.2f}m)"
                )
                # Créer une zone de collision immédiate
                return FlightZone(
                    level=RiskLevel.HIGH,
                    position=0.0,  # Collision immédiate
                    next_position=list(position),
                    vector=self.calculate_avoidance_vector(list(rel_pos), [0.0, 0.0, 0.0], RiskLevel.HIGH),
                    zone_id="statique_proche",
                )
            return None

        time_to_closest = -dot_product / rel_speed_sq
        print(f"[COLLISION_DETECT] {self.drone_id}: " f"time_to_closest={time_to_closest:.2f}s")

        if time_to_closest > self.collision_distance:
            print(
                f"[COLLISION_DETECT] {self.drone_id}: "
                f"Collision trop lointaine ({time_to_closest:.2f}s > "
                f"{self.collision_distance:.2f}s)"
            )
            return None

        # Position au point de collision la plus proche
        closest_pos = (
            rel_pos[0] + rel_vel[0] * time_to_closest,
            rel_pos[1] + rel_vel[1] * time_to_closest,
            rel_pos[2] + rel_vel[2] * time_to_closest,
        )

        closest_distance = math.sqrt(closest_pos[0] ** 2 + closest_pos[1] ** 2 + closest_pos[2] ** 2)

        # Évaluer du niveau de risque
        risk_level = self.assess_risk(closest_distance, time_to_closest)

        if risk_level == RiskLevel.SAFE:
            return None

        # Log détaillé des collisions détectées
        print(
            f"[COLLISION_DETECTED] {self.drone_id} vs "
            f"{other_threat.drone_id}: "
            f"Risk={risk_level.value}, "
            f"closest_dist={closest_distance:.2f}m, "
            f"time={time_to_closest:.2f}s, "
            f"current_dist={current_distance:.2f}m"
        )

        # Calculer du point de collision prévu
        collision_point = (
            position[0] + velocity[0] * time_to_closest,
            position[1] + velocity[1] * time_to_closest,
            position[2] + velocity[2] * time_to_closest,
        )

        # Calculer du vecteur d'évitement
        avoidance_vector = self.calculate_avoidance_vector(list(rel_pos), list(rel_vel), risk_level)

        print(
            f"[COLLISION_AVOIDANCE] {self.drone_id}: "
            f"Vecteur d'évitement {avoidance_vector} "
            f"pour éviter {other_threat.drone_id}"
        )

        return FlightZone(
            level=risk_level,
            position=time_to_closest,
            next_position=list(collision_point),
            vector=avoidance_vector,
            zone_id=self.get_zone_description(risk_level),
        )

    def assess_risk(self, distance: float, time_until: float) -> RiskLevel:
        """
        Évalue le niveau de risque d'une collision.

        Args:
            distance (float): Distance au point de collision
            time_until (float): Temps jusqu'à l'heure la plus proche

        Returns:
            RiskLevel: Niveau de risque
        """
        if distance > self.safe_distance:
            return RiskLevel.SAFE
        if distance > self.warning_distance:
            if time_until < 1.0:
                return RiskLevel.HIGH
            if time_until < 2.5:  # Élargi de 2.0 à 2.5
                return RiskLevel.MEDIUM
            else:
                return RiskLevel.LOW
        else:  # distance < warning_distance - Collision imminente
            if time_until < 0.8:  # Seuil critique
                return RiskLevel.CRITICAL
            if time_until < 1.5:  # Réduit de 2.0 à 1.5 pour HIGH
                return RiskLevel.HIGH
            else:
                return RiskLevel.MEDIUM

    def calculate_avoidance_vector(
        self, rel_pos: List[float], rel_vel: List[float], risk_level: RiskLevel
    ) -> List[float]:
        """
        Calcule le vecteur d'évitement optimal.

        Args:
            rel_pos (List): Position relative
            rel_vel (List): Vitesse relative
            risk_level (RiskLevel): Niveau de risque

        Returns:
            List[float]: Vecteur d'évitement [dx, dy, dz]
        """
        # Intensité d'évitement basée sur le niveau de risque
        intensity_factor = {
            RiskLevel.LOW: 0.5,
            RiskLevel.MEDIUM: 1.0,
            RiskLevel.HIGH: 2.0,
            RiskLevel.CRITICAL: 3.0,
        }.get(risk_level, 0.5)

        # Vecteur d'évitement perpendiculaire à la trajectoire de collision
        if abs(rel_vel[0]) > abs(rel_vel[1]):
            # Évitement latéral
            dx = 0.0
            dy = intensity_factor * (1.0 if rel_pos[1] > 0 else -1.0)
        else:
            # Évitement longitudinal
            dx = intensity_factor * (1.0 if rel_pos[0] > 0 else -1.0)
            dy = 0.0

        # Évitement vertical si les autres axes sont insuffisants
        dz = 0.0
        if risk_level == RiskLevel.CRITICAL:
            dz = intensity_factor * 0.3 * (1.0 if rel_pos[2] > 0 else -1.0)

        return [dx, dy, dz]

    def get_zone_description(self, risk_level: RiskLevel) -> str:
        """
        Retourne la description de la zone selon le niveau de risque.

        Args:
            risk_level (RiskLevel): Niveau de risque

        Returns:
            str: Description textuelle
        """
        descriptions = {
            RiskLevel.LOW: "précaution",
            RiskLevel.MEDIUM: "attention",
            RiskLevel.HIGH: "danger",
            RiskLevel.CRITICAL: "urgence",
        }
        return descriptions.get(risk_level, "inconnu")

    def calculate_multiranger_avoidance(self, ranges: List[float]) -> List[float]:
        """
        Calcule l'évitement basé sur les mesures du capteur multiranger.

        Args:
            ranges (List[float]): Capteurs [back, right, front, left]

        Returns:
            List[float]: Forces d'évitement [fx, fy, fz]
        """
        if len(ranges) < 4:
            return [0.0, 0.0, 0.0]

        back, right, front, left = ranges[:4]

        fx = 0.0
        fy = 0.0
        fz = 0.0

        # Éviter les obstacles devant/derrière sur l'axe X
        for risk_level_key, threshold in self.risk_factors.items():
            intensity = self.action_intensity[risk_level_key]

            # Évitement frontal / arrière
            if front < threshold:
                strength = (threshold - front) / threshold
                fx -= intensity * strength * strength  # Reculer

            if back < threshold:
                strength = (threshold - back) / threshold
                fx += intensity * strength * strength

            # Évitement droite / gauche
            if right < threshold:
                strength = (threshold - right) / threshold
                fy += intensity * strength * strength

            if left < threshold:
                strength = (threshold - left) / threshold
                fy -= intensity * strength * strength

        # Évitement vertical si nécessaire
        min_range = min(ranges[:4])
        if min_range < self.risk_factors['critical']:
            fz = 0.5  # Élévation d'urgence

        return [fx, fy, fz]

    def navigate_with_avoidance(self, position: List[float], velocity: List[float], ranges: List[float]) -> List[float]:
        """
        Calcule la commande de navigation avec évitement intégral.

        Args:
            position (List): Position actuelle
            velocity (List): Vitesse actuelle
            ranges (List[float]): Mesures des capteurs

        Returns:
            List[float]: Commande d'évitement [vx, vy, vz]
        """
        combined_avoidance = [0.0, 0.0, 0.0]

        # Évitement d'obstacles
        obstacle_avoidance = self.calculate_multiranger_avoidance(ranges)
        combined_avoidance[0] += obstacle_avoidance[0]
        combined_avoidance[1] += obstacle_avoidance[1]
        combined_avoidance[2] += obstacle_avoidance[2]

        # Évitement de drones des autres menaces
        closest_collision = None
        highest_risk = RiskLevel.SAFE

        for threat in self.threats.values():
            if threat.drone_id != self.drone_id:
                flight_zone = self.detect_collision(threat, position, velocity)

                collision_detected = flight_zone and flight_zone.level.value > highest_risk.value
                if collision_detected:
                    closest_collision = flight_zone
                    if flight_zone:
                        highest_risk = flight_zone.level

        if closest_collision:
            # Appliquer la commande d'évitement
            avoidance_cmd = closest_collision.vector
            combined_avoidance[0] += avoidance_cmd[0]
            combined_avoidance[1] += avoidance_cmd[1]
            combined_avoidance[2] += avoidance_cmd[2]

        # Limitation de la commande d'évitement
        max_intensity = 1.5
        magnitude = math.sqrt(sum(f * f for f in combined_avoidance))
        if magnitude > max_intensity:
            scale_factor = max_intensity / magnitude
            combined_avoidance = [f * scale_factor for f in combined_avoidance]

        return list(combined_avoidance)

    def get_status(self) -> Dict[str, Any]:
        """
        Retourne le statut actuel des collisions pour debug.

        Returns:
            Dict[str, Any]: Informations de statut
        """
        return {
            'threats': len(self.threats),
            'drone_ids': list(self.threats.keys()),
            'last_update': self.last_update,
            'history_count': len(self.history),
        }


# Gestionnaire coordonné du système d'évitement d'obstacles


class CollisionController:
    """Gestionnaire coordonné du système d'évitement d'obstacles."""

    def __init__(self, drone_id: str = "default") -> None:
        """Initialise le gestionnaire d'évitement coordonné."""
        self.collision_manager = CollisionManager(drone_id)
        self.statistics: Dict[str, Any] = {}

    def update_statistics(self, stats_dict: Dict[str, Any]) -> None:
        """Met à jour les statistiques des autres drones et obstacles."""
        self.statistics = stats_dict

        # Met à jour le gestionnaire de collisions
        for drone_id, drone_stats in stats_dict.items():
            if drone_stats and len(drone_stats) >= 3:
                # Assume la vitesse stationnaire pour l'évitement
                velocity = [0.0, 0.0, 0.0]  # Initialisé pour l'évitement
                self.collision_manager.track_threat(
                    drone_id, [drone_stats[0], drone_stats[1], drone_stats[2]], velocity
                )

    def compute_avoidance(
            self, position: List[float], ranges: List[float],
            other_positions: Dict[str, Any] = None) -> List[float]:
        """
        Calcule l'évitement avancé incluant la composante Z.
        
        Args:
            position (List[float]): Position actuelle [x, y, z]
            ranges (List[float]): Distances mesurées [back, right, front, left]
            other_positions (Dict[str, Any]): Positions des autres drones (optionnel)
            
        Returns:
            List[float]: [avoidance_x, avoidance_y, avoidance_z]
        """
        if len(position) < 3:
            return [0.0, 0.0, 0.0]

        # Utiliser le système coordonné d'évitement pour X et Y
        avoidance_command = self.collision_manager.navigate_with_avoidance(
            [position[0], position[1], position[2]], [0.0, 0.0, 0.0], ranges
        )
        
        # Ajouter une composante Z basée sur la distance frontale
        avoidance_z = 0.0
        if len(ranges) >= 3:  # Vérifier que front_range existe
            front_range = ranges[2]
            if front_range < 0.50:  # Seuil pour élévation
                avoidance_z = 0.3 * (0.50 - front_range)
        
        return [avoidance_command[0], avoidance_command[1], avoidance_z]

    def emergency_stop(self, ranges: List[float], threshold: float) -> bool:
        """
        Détermine si un arrêt d'urgence est nécessaire.

        Args:
            ranges (List[float]): Mesures des capteurs
            threshold (float): Distance de sécurité

        Returns:
            bool: True si l'arrêt est nécessaire
        """
        if len(ranges) >= 4:
            return any(r < threshold for r in ranges[:4])
        return False


__all__ = ['RiskLevel', 'Threat', 'FlightZone', 'CollisionManager', 'CollisionController']
