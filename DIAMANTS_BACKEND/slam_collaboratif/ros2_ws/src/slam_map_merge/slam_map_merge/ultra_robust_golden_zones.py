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
Module de gestion ultra-robuste des zones d'or pour la fusion de cartes multi-drones.
Version industrielle avec seuils stricts, timeouts longs et anti-faux positifs maximum.
"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import OccupancyGrid
import numpy as np
from typing import Dict, Set, Tuple, Optional, List
import time
from dataclasses import dataclass
from enum import Enum

class ZoneState(Enum):
    """États possibles d'une zone d'or"""
    CANDIDATE = "candidate"      # Zone candidate (détectée par peu de drones)
    CONSENSUS = "consensus"      # Zone en consensus (détectée par seuil moyen)
    VALIDATED = "validated"      # Zone validée (détectée par beaucoup de drones)
    SUPPRESSION = "suppression"  # Zone en cours de suppression

@dataclass
class ZoneInfo:
    """Informations détaillées sur une zone d'or"""
    x: int
    y: int
    state: ZoneState
    creation_time: float
    last_update: float
    detection_count: int
    confidence: float
    observers: Set[str]
    suppression_votes: int
    last_suppression_vote: float

class UltraRobustGoldenZoneManager:
    """
    Gestionnaire ultra-robuste des zones d'or avec seuils stricts.
    
    Paramètres ultra-stricts:
    - creation_threshold: 3+ drones minimum pour créer une zone
    - validation_threshold: 4+ drones pour validation  
    - disappear_threshold: 5+ drones pour supprimer
    - Timeouts longs: 30s/10min/20min
    - Confiance élevée: 80%/90%
    - Cooldown suppression: 60s
    """
    
    def __init__(self, 
                 creation_threshold: int = 3,
                 validation_threshold: int = 4,
                 disappear_threshold: int = 5,
                 creation_timeout: float = 30.0,
                 consensus_timeout: float = 600.0,
                 validated_timeout: float = 1200.0,
                 min_confidence: float = 0.8,
                 validation_confidence: float = 0.9,
                 suppression_cooldown: float = 60.0,
                 golden_cell_value: int = 75,
                 node: Optional[Node] = None):
        """
        Initialise le gestionnaire ultra-robuste.
        
        Args:
            creation_threshold: Nombre minimum de drones pour créer une zone
            validation_threshold: Nombre minimum de drones pour valider  
            disappear_threshold: Nombre minimum de drones pour supprimer
            creation_timeout: Timeout pour création de zone (secondes)
            consensus_timeout: Timeout pour zones consensus (secondes)
            validated_timeout: Timeout pour zones validées (secondes)
            min_confidence: Confiance minimum pour création
            validation_confidence: Confiance minimum pour validation
            suppression_cooldown: Cooldown après suppression (secondes)
            golden_cell_value: Valeur des cellules zones d'or dans la grille
            node: Node ROS2 pour logging
        """
        self.creation_threshold = creation_threshold
        self.validation_threshold = validation_threshold
        self.disappear_threshold = disappear_threshold
        self.creation_timeout = creation_timeout
        self.consensus_timeout = consensus_timeout
        self.validated_timeout = validated_timeout
        self.min_confidence = min_confidence
        self.validation_confidence = validation_confidence
        self.suppression_cooldown = suppression_cooldown
        self.golden_cell_value = golden_cell_value
        self.node = node
        
        # État interne
        self.zones: Dict[Tuple[int, int], ZoneInfo] = {}
        self.suppression_history: Dict[Tuple[int, int], float] = {}
        self.last_cleanup = time.time()
        
        if self.node:
            self.node.get_logger().info(
                f"[ZONES D'OR] ULTRA-ROBUSTES Activées - "
                f"Création: {creation_threshold}, Validation: {validation_threshold}, "
                f"Disparition: {disappear_threshold}"
            )
            self.node.get_logger().info(
                f"[ZONES D'OR] Timeouts: création={creation_timeout}s, "
                f"consensus={consensus_timeout}s, validée={validated_timeout}s"
            )
            self.node.get_logger().info(
                f"[ZONES D'OR] Confiance: min={min_confidence}, validation={validation_confidence}"
            )
    
    def add_observation(self, cell_index: int, robot_id: str, is_obstacle: bool) -> None:
        """
        Ajoute une observation d'un robot (compatibilité avec l'interface existante).
        Cette méthode stocke les observations pour utilisation dans update_from_consensus.
        
        Args:
            cell_index: Index de la cellule dans la grille
            robot_id: Identifiant du robot
            is_obstacle: True si obstacle détecté, False si libre
        """
        # Pour l'ultra-robuste, on stocke temporairement les observations
        # et on les traite dans update_from_consensus
        if not hasattr(self, '_temp_observations'):
            self._temp_observations = {}
        
        if cell_index not in self._temp_observations:
            self._temp_observations[cell_index] = {}
            
        self._temp_observations[cell_index][robot_id] = is_obstacle
    
    def update_from_consensus(self, consensus_obstacles: Set[Tuple[int, int]], 
                            robot_detections: Optional[Dict[str, Set[Tuple[int, int]]]] = None) -> None:
        """
        Met à jour les zones d'or à partir du consensus multi-robots.
        
        Args:
            consensus_obstacles: Ensemble des obstacles en consensus
            robot_detections: Détections par robot (optionnel, peut être None)
        """
        current_time = time.time()
        
        # Si pas de robot_detections fourni, utiliser les observations temporaires
        if robot_detections is None:
            robot_detections = self._build_robot_detections_from_temp()
        
        # Traiter chaque obstacle en consensus
        for obstacle in consensus_obstacles:
            self._process_obstacle_detection(obstacle, robot_detections, current_time)
        
        # Vérifier les suppressions
        self._check_suppressions(robot_detections, current_time)
        
        # Nettoyage périodique
        if current_time - self.last_cleanup > 60.0:  # Toutes les minutes
            self._cleanup_expired_zones(current_time)
            self.last_cleanup = current_time
    
    def _build_robot_detections_from_temp(self) -> Dict[str, Set[Tuple[int, int]]]:
        """Construit robot_detections à partir des observations temporaires"""
        if not hasattr(self, '_temp_observations'):
            return {}
            
        robot_detections = {}
        
        for cell_index, robot_obs in self._temp_observations.items():
            # Convertir cell_index vers coordonnées (approximation)
            # Pour une vraie conversion, il faudrait width, mais on peut approximer
            y, x = divmod(cell_index, 200)  # Assume width=200 par défaut
            
            for robot_id, is_obstacle in robot_obs.items():
                if robot_id not in robot_detections:
                    robot_detections[robot_id] = set()
                    
                if is_obstacle:
                    robot_detections[robot_id].add((x, y))
        
        return robot_detections
    
    def _process_obstacle_detection(self, obstacle: Tuple[int, int], 
                                  robot_detections: Dict[str, Set[Tuple[int, int]]],
                                  current_time: float) -> None:
        """Traite la détection d'un obstacle par les robots"""
        x, y = obstacle
        
        # Compter les détections de cet obstacle
        detecting_robots = set()
        for robot_id, detections in robot_detections.items():
            if obstacle in detections:
                detecting_robots.add(robot_id)
        
        detection_count = len(detecting_robots)
        confidence = min(1.0, detection_count / self.validation_threshold)
        
        # Vérifier cooldown suppression
        if (x, y) in self.suppression_history:
            if current_time - self.suppression_history[(x, y)] < self.suppression_cooldown:
                return  # Encore en cooldown
        
        if (x, y) not in self.zones:
            # Nouvelle zone candidate
            if detection_count >= self.creation_threshold and confidence >= self.min_confidence:
                self.zones[(x, y)] = ZoneInfo(
                    x=x, y=y,
                    state=ZoneState.CANDIDATE,
                    creation_time=current_time,
                    last_update=current_time,
                    detection_count=detection_count,
                    confidence=confidence,
                    observers=detecting_robots.copy(),
                    suppression_votes=0,
                    last_suppression_vote=0.0
                )
                
                if self.node:
                    self.node.get_logger().debug(
                        f"[ZONES D'OR] Nouvelle zone candidate ({x},{y}) - "
                        f"{detection_count} drones, confiance {confidence:.2f}"
                    )
        else:
            # Zone existante - mise à jour
            zone = self.zones[(x, y)]
            zone.last_update = current_time
            zone.detection_count = detection_count
            zone.confidence = confidence
            zone.observers = detecting_robots.copy()
            
            # Progression d'état
            self._update_zone_state(zone, current_time)
    
    def _update_zone_state(self, zone: ZoneInfo, current_time: float) -> None:
        """Met à jour l'état d'une zone selon les critères ultra-stricts"""
        age = current_time - zone.creation_time
        
        if zone.state == ZoneState.CANDIDATE:
            # Candidate → Consensus
            if (zone.detection_count >= self.validation_threshold and
                age >= self.creation_timeout and
                zone.confidence >= self.min_confidence):
                
                zone.state = ZoneState.CONSENSUS
                if self.node:
                    self.node.get_logger().info(
                        f"[ZONES D'OR] Zone ({zone.x},{zone.y}) → CONSENSUS "
                        f"({zone.detection_count} drones, {age:.1f}s)"
                    )
        
        elif zone.state == ZoneState.CONSENSUS:
            # Consensus → Validated
            if (zone.detection_count >= self.validation_threshold and
                zone.confidence >= self.validation_confidence):
                
                zone.state = ZoneState.VALIDATED
                if self.node:
                    self.node.get_logger().info(
                        f"[ZONES D'OR] Zone ({zone.x},{zone.y}) → VALIDÉE "
                        f"({zone.detection_count} drones, confiance {zone.confidence:.2f})"
                    )
    
    def _check_suppressions(self, robot_detections: Dict[str, Set[Tuple[int, int]]],
                           current_time: float) -> None:
        """Vérifie les zones à supprimer selon les critères ultra-stricts"""
        zones_to_remove = []
        
        for (x, y), zone in self.zones.items():
            # Compter les robots qui ne voient plus cette zone
            non_detecting_robots = 0
            total_robots = len(robot_detections)
            
            for robot_id, detections in robot_detections.items():
                if (x, y) not in detections:
                    non_detecting_robots += 1
            
            # Critère ultra-strict pour suppression
            if non_detecting_robots >= self.disappear_threshold:
                zone.suppression_votes = non_detecting_robots
                zone.last_suppression_vote = current_time
                
                # Suppression immédiate si critères ultra-stricts remplis
                if (zone.suppression_votes >= self.disappear_threshold and
                    zone.detection_count < self.creation_threshold):
                    
                    zones_to_remove.append((x, y))
                    self.suppression_history[(x, y)] = current_time
                    
                    if self.node:
                        self.node.get_logger().info(
                            f"[ZONES D'OR] Suppression zone ({x},{y}) - "
                            f"{zone.suppression_votes} votes suppression"
                        )
        
        # Supprimer les zones
        for zone_key in zones_to_remove:
            del self.zones[zone_key]
    
    def _cleanup_expired_zones(self, current_time: float) -> None:
        """Nettoie les zones expirées selon les timeouts"""
        zones_to_remove = []
        
        for (x, y), zone in self.zones.items():
            age = current_time - zone.last_update
            
            # Timeouts selon l'état
            if zone.state == ZoneState.CANDIDATE and age > self.creation_timeout * 2:
                zones_to_remove.append((x, y))
            elif zone.state == ZoneState.CONSENSUS and age > self.consensus_timeout:
                zones_to_remove.append((x, y))
            elif zone.state == ZoneState.VALIDATED and age > self.validated_timeout:
                zones_to_remove.append((x, y))
        
        # Supprimer les zones expirées
        for zone_key in zones_to_remove:
            zone = self.zones[zone_key]
            del self.zones[zone_key]
            
            if self.node:
                self.node.get_logger().info(
                    f"[ZONES D'OR] Zone expirée ({zone.x},{zone.y}) - "
                    f"état {zone.state.value}, âge {current_time - zone.last_update:.1f}s"
                )
    
    def cleanup_all_expired(self) -> None:
        """Force le nettoyage de toutes les zones expirées"""
        current_time = time.time()
        self._cleanup_expired_zones(current_time)
    
    def get_golden_zones_map(self, width: int, height: int, 
                           resolution: float, origin_x: float, origin_y: float) -> OccupancyGrid:
        """
        Génère une carte des zones d'or pour RViz.
        
        Args:
            width: Largeur de la carte
            height: Hauteur de la carte  
            resolution: Résolution de la carte
            origin_x: Origine X de la carte
            origin_y: Origine Y de la carte
            
        Returns:
            OccupancyGrid avec les zones d'or
        """
        # Créer la grille
        grid = np.full((height, width), -1, dtype=np.int8)  # -1 = inconnu
        
        # Ajouter les zones d'or selon leur état avec visibilité améliorée
        for (x, y), zone in self.zones.items():
            if 0 <= x < width and 0 <= y < height:
                # Valeurs plus contrastées pour meilleure visibilité dans RViz
                if zone.state == ZoneState.VALIDATED:
                    # Zone validée : Rouge très intense (100 = obstacle sûr)
                    grid[y, x] = 100
                    # Ajouter un effet de halo pour plus de visibilité
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx, ny = x + dx, y + dy
                            if (0 <= nx < width and 0 <= ny < height and 
                                grid[ny, nx] == -1):  # Seulement si vide
                                grid[ny, nx] = 90  # Halo intense
                elif zone.state == ZoneState.CONSENSUS:
                    # Zone consensus : Orange vif (85 = obstacle probable)
                    grid[y, x] = 85
                    # Petit halo
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx, ny = x + dx, y + dy
                            if (0 <= nx < width and 0 <= ny < height and 
                                grid[ny, nx] == -1):
                                grid[ny, nx] = 75
                elif zone.state == ZoneState.CANDIDATE:
                    # Zone candidate : Jaune visible (65 = obstacle possible)
                    grid[y, x] = 65
        
        # Créer le message
        msg = OccupancyGrid()
        msg.header.stamp = self.node.get_clock().now().to_msg() if self.node else Clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = origin_x
        msg.info.origin.position.y = origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        
        return msg
    
    def get_statistics(self) -> Dict[str, int]:
        """Retourne les statistiques des zones d'or"""
        stats = {
            'total': len(self.zones),
            'candidates': 0,
            'consensus': 0,
            'validated': 0
        }
        
        for zone in self.zones.values():
            if zone.state == ZoneState.CANDIDATE:
                stats['candidates'] += 1
            elif zone.state == ZoneState.CONSENSUS:
                stats['consensus'] += 1
            elif zone.state == ZoneState.VALIDATED:
                stats['validated'] += 1
        
        return stats
    
    def get_golden_zones_count(self) -> Tuple[int, int]:
        """Retourne le nombre de zones en consensus et validées (compatibilité)"""
        stats = self.get_statistics()
        return stats['consensus'], stats['validated']
