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

# --- MAP_MERGER_NODE_MODULAR.PY ---
# Node ROS2 de fusion collaborative
# (stigmergie, consensus, fusion pondérée, lissage)
# Ultra-modulaire, tout activable/désactivable par config
# AVEC SYSTÈME DE ZONES D'OR ET JOURNAL HISTORIQUE
# Auteur : Correction Copilot, 2025

import os
import time
from collections import defaultdict, deque
from enum import IntEnum
from typing import Any, Dict, Set

import cv2  # pylint: disable=import-error
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image

# Import de la version ultra-robuste
try:
    from .ultra_robust_golden_zones import UltraRobustGoldenZoneManager

    ULTRA_ROBUST_AVAILABLE = True
except ImportError:
    try:
        from ultra_robust_golden_zones import UltraRobustGoldenZoneManager

        ULTRA_ROBUST_AVAILABLE = True
    except ImportError:
        ULTRA_ROBUST_AVAILABLE = False
        print(
            "ATTENTION: Version ultra-robuste non disponible, "
            "utilisation de la version de base"
        )

# --- ENUMS ET CONSTANTES ---


class CellValue(IntEnum):
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100
    GOLDEN_ZONE = 75  # Valeur spéciale pour les zones d'or


class JournalEntry:
    """Entrée dans le journal historique pour une cellule"""

    def __init__(
        self, robot_id: str, detected: bool, timestamp: float, confidence: float = 1.0
    ):
        self.robot_id = robot_id
        # True si obstacle détecté, False si zone libre
        self.detected = detected
        self.timestamp = timestamp
        self.confidence = confidence


class GoldenZoneManager:
    """Gestionnaire des zones d'or ULTRA-ROBUSTE : Rendu EXACT Gazebo/RViz"""

    def __init__(
        self,
        consensus_threshold: int = 3,
        validation_threshold: int = 4,
        disappear_threshold: int = 2,
        consensus_timeout: float = 120.0,
        validated_timeout: float = 600.0,
        logger=None,
    ):

        # Seuils ULTRA-STRICTS pour garantir la fiabilité
        self.consensus_threshold = consensus_threshold  # 3+ drones MINIMUM
        # 4+ drones pour validation
        self.validation_threshold = validation_threshold
        # Garde pour compatibilité
        self.disappear_threshold = disappear_threshold
        self.consensus_timeout = consensus_timeout  # 2min pour consensus
        self.validated_timeout = validated_timeout  # 10min pour zones validées
        self.logger = logger

        # Journal historique avec confiance
        self.journal: Dict[int, list] = defaultdict(list)

        # Zones d'or avec métadonnées complètes
        self.golden_zones: Dict[int, dict] = {}

        # Système de validation croisée
        self.spatial_coherence_radius = 3  # Vérification dans un rayon de 3 cellules
        self.temporal_stability_window = 30.0  # 30s de stabilité requise

        # Anti-faux positifs : historique des suppressions
        self.suppression_history = defaultdict(list)
        self.suppression_cooldown = 60.0  # Cooldown 1min après suppression

        # Cache pour éviter recalculs
        self._last_update_time = 0.0
        self._last_cleanup_time = 0.0

        # Ajout des attributs manquants
        if not hasattr(self, 'suppression_history'):
            self.suppression_history = defaultdict(list)
        if not hasattr(self, 'suppression_cooldown'):
            self.suppression_cooldown = 60.0
        if not hasattr(self, 'temporal_stability_window'):
            self.temporal_stability_window = 30.0

    def get_logger(self):
        """Helper pour les logs"""
        if self.logger:
            return self.logger
        else:
            # Fallback si pas de logger
            class DummyLogger:
                def info(self, msg):
                    print(f"[INFO] {msg}")

                def warn(self, msg):
                    print(f"[WARN] {msg}")

            return DummyLogger()

    def add_observation(
        self, cell_index: int, robot_id: str, is_obstacle: bool, confidence: float = 1.0
    ):
        """Ajoute une observation avec validation ULTRA-STRICTE"""
        current_time = time.time()

        # Vérifier le cooldown de suppression
        if self._is_in_suppression_cooldown(cell_index, current_time):
            return

        entry = JournalEntry(robot_id, is_obstacle, current_time, confidence)
        self.journal[cell_index].append(entry)

        # Nettoyer les entrées expirées
        self._cleanup_expired_entries(cell_index, current_time)

        # CRÉATION ULTRA-STRICTE avec validation spatiale
        if is_obstacle and cell_index not in self.golden_zones:
            if self._validate_obstacle_creation(cell_index, current_time):
                self.golden_zones[cell_index] = {
                    'status': 'consensus',
                    'created_at': current_time,
                    'validation_robots': set(),
                    'spatial_validated': False,
                    'temporal_stable': False,
                }
                # Réduire la verbosité : passer en DEBUG au lieu d'INFO
                self.get_logger().debug(
                    f"[GOLDEN] ⚡ CRÉATION ULTRA-STRICTE zone {cell_index}"
                )

    def _is_in_suppression_cooldown(self, cell_index: int, current_time: float) -> bool:
        """Vérifie si la cellule est en cooldown après une suppression"""
        if cell_index in self.suppression_history:
            last_suppression = max(self.suppression_history[cell_index])
            return (current_time - last_suppression) < self.suppression_cooldown
        return False

    def _validate_obstacle_creation(self, cell_index: int, current_time: float) -> bool:
        """Validation ULTRA-STRICTE pour la création d'obstacles"""
        # 1. Vérifier le nombre de robots récents
        detection_robots = set()
        recent_threshold = current_time - 30.0  # Fenêtre plus large : 30s

        for entry in self.journal[cell_index]:
            if (
                entry.timestamp >= recent_threshold
                and entry.detected
                and getattr(entry, 'confidence', 1.0) >= 0.8
            ):  # Seuil de confiance
                detection_robots.add(entry.robot_id)

        if len(detection_robots) < self.consensus_threshold:
            return False

        # 2. Validation de stabilité temporelle
        if not self._check_temporal_stability(cell_index, current_time):
            return False

        return True

    def _check_temporal_stability(self, cell_index: int, current_time: float) -> bool:
        """Vérifie la stabilité temporelle des détections"""
        stability_window = current_time - self.temporal_stability_window

        detections_timeline = []
        for entry in self.journal[cell_index]:
            if entry.timestamp >= stability_window and entry.detected:
                detections_timeline.append(entry.timestamp)

        # Il faut au moins 3 détections étalées sur la fenêtre
        if len(detections_timeline) < 3:
            return False

        # Vérifier la distribution temporelle
        detections_timeline.sort()
        time_span = detections_timeline[-1] - detections_timeline[0]

        return time_span >= (self.temporal_stability_window * 0.6)  # 60% de la fenêtre

    def update_from_consensus(self, consensus_obstacles: Set[int]):
        """VALIDATION depuis le consensus multi-drones"""
        current_time = time.time()

        # VALIDATION : Promouvoir vers ultra-validé si validation_threshold atteint
        for cell_index in consensus_obstacles:
            if (
                cell_index in self.golden_zones
                and self.golden_zones[cell_index]['status'] == 'consensus'
            ):
                self._cleanup_expired_entries(cell_index, current_time)
                entries = self.journal[cell_index]

                # Compter les robots pour validation
                detection_robots = set()
                for entry in entries:
                    if entry.timestamp >= current_time - 60.0 and entry.detected:
                        detection_robots.add(entry.robot_id)

                if len(detection_robots) >= self.validation_threshold:
                    self.golden_zones[cell_index] = {
                        'status': 'validated',
                        'created_at': current_time,
                    }
                    # Réduire la verbosité : passer en DEBUG au lieu d'INFO
                    self.get_logger().debug(
                        f"[GOLDEN] ⬆️ VALIDATION zone d'or cellule {cell_index} par {len(detection_robots)} drones"
                    )

    def update_golden_zones(self):
        """Mise à jour ULTRA-ROBUSTE avec suppression intelligente"""
        current_time = time.time()

        # Fréquence de mise à jour limitée
        if current_time - self._last_update_time < 5.0:  # 5s minimum
            return
        self._last_update_time = current_time

        zones_to_remove = []
        zones_to_promote = []

        for cell_index, zone_info in list(self.golden_zones.items()):
            self._cleanup_expired_entries(cell_index, current_time)

            # Analyser l'état actuel de la zone
            analysis = self._analyze_zone_status(cell_index, current_time)

            # Décision de suppression ULTRA-STRICTE
            if self._should_remove_zone(cell_index, zone_info, analysis, current_time):
                zones_to_remove.append((cell_index, analysis['reason']))

            # Décision de promotion vers validé
            elif zone_info['status'] == 'consensus' and self._should_promote_zone(
                cell_index, zone_info, analysis
            ):
                zones_to_promote.append(cell_index)

        # Appliquer les changements
        for cell_index, reason in zones_to_remove:
            self._remove_zone(cell_index, reason, current_time)

        for cell_index in zones_to_promote:
            self._promote_zone_to_validated(cell_index, current_time)

        # Nettoyage périodique
        if current_time - self._last_cleanup_time > 300.0:  # 5min
            self._periodic_cleanup(current_time)
            self._last_cleanup_time = current_time

    def _analyze_zone_status(self, cell_index: int, current_time: float) -> Dict:
        """Analyse complète de l'état d'une zone"""
        entries = self.journal[cell_index]

        # Analyser les observations récentes (dernières 60s)
        recent_threshold = current_time - 60.0
        recent_detection_robots = set()
        recent_disappear_robots = set()
        total_confidence_detection = 0.0
        total_confidence_disappear = 0.0

        for entry in entries:
            if entry.timestamp >= recent_threshold:
                confidence = getattr(entry, 'confidence', 1.0)
                if entry.detected:
                    recent_detection_robots.add(entry.robot_id)
                    total_confidence_detection += confidence
                else:
                    recent_disappear_robots.add(entry.robot_id)
                    total_confidence_disappear += confidence

        # Calculer la confiance moyenne
        avg_conf_detection = (
            total_confidence_detection / len(recent_detection_robots)
            if len(recent_detection_robots) > 0
            else 0.0
        )
        avg_conf_disappear = (
            total_confidence_disappear / len(recent_disappear_robots)
            if len(recent_disappear_robots) > 0
            else 0.0
        )

        return {
            'recent_detection_robots': recent_detection_robots,
            'recent_disappear_robots': recent_disappear_robots,
            'avg_confidence_detection': avg_conf_detection,
            'avg_confidence_disappear': avg_conf_disappear,
            'detection_count': len(recent_detection_robots),
            'disappear_count': len(recent_disappear_robots),
            'reason': 'none',
        }

    def _should_remove_zone(
        self, cell_index: int, zone_info: Dict, analysis: Dict, current_time: float
    ) -> bool:
        """Décision de suppression ULTRA-STRICTE"""
        age = current_time - zone_info['created_at']

        # 1. Suppression par consensus de disparition MASSIF
        if (
            analysis['disappear_count'] >= 4  # 4+ drones
            and analysis['detection_count'] <= 1  # Presque plus de détection
            and analysis['avg_confidence_disappear'] >= 0.8  # Haute confiance
            and age > 30.0
        ):  # Âge minimum 30s
            analysis['reason'] = 'consensus_massif_disparition'
            return True

        # 2. Suppression par majorité écrasante
        if (
            analysis['disappear_count'] >= 3
            and analysis['disappear_count']
            > (analysis['detection_count'] * 2)  # 2:1 ratio
            and age > 60.0
        ):  # Âge minimum 1min
            analysis['reason'] = 'majorite_ecrasante_disparition'
            return True

        # 3. Timeout ultra-long pour zones consensus
        if (
            zone_info['status'] == 'consensus'
            and analysis['detection_count'] == 0
            and age > self.consensus_timeout
        ):
            analysis['reason'] = 'timeout_consensus'
            return True

        # 4. Timeout pour zones validées (très long)
        if (
            zone_info['status'] == 'validated'
            and analysis['detection_count'] == 0
            and analysis['disappear_count'] >= 2
            and age > self.validated_timeout
        ):
            analysis['reason'] = 'timeout_validated'
            return True

        return False

    def _should_promote_zone(
        self, cell_index: int, zone_info: Dict, analysis: Dict
    ) -> bool:
        """Décision de promotion vers zone validée"""
        return (
            analysis['detection_count'] >= self.validation_threshold
            and analysis['avg_confidence_detection'] >= 0.9
        )

    def _remove_zone(self, cell_index: int, reason: str, current_time: float):
        """Supprime une zone et enregistre l'historique"""
        del self.golden_zones[cell_index]
        self.suppression_history[cell_index].append(current_time)
        self.get_logger().debug(
            f"[GOLDEN] ❌ SUPPRESSION ULTRA-STRICTE zone {cell_index} ({reason})"
        )

    def _promote_zone_to_validated(self, cell_index: int, current_time: float):
        """Promeut une zone vers le statut validé"""
        self.golden_zones[cell_index]['status'] = 'validated'
        self.golden_zones[cell_index]['validated_at'] = current_time
        # Réduire la verbosité : passer en DEBUG au lieu d'INFO
        self.get_logger().debug(
            f"[GOLDEN] ⬆️ PROMOTION zone {cell_index} vers VALIDATED"
        )

    def _periodic_cleanup(self, current_time: float):
        """Nettoyage périodique complet"""
        # Nettoyer l'historique de suppression
        for cell_index in list(self.suppression_history.keys()):
            self.suppression_history[cell_index] = [
                timestamp
                for timestamp in self.suppression_history[cell_index]
                if current_time - timestamp <= 300.0  # Garder 5min d'historique
            ]
            if not self.suppression_history[cell_index]:
                del self.suppression_history[cell_index]

        # Nettoyer le journal complet
        for cell_index in list(self.journal.keys()):
            self._cleanup_expired_entries(cell_index, current_time)
            if not self.journal[cell_index]:
                del self.journal[cell_index]

    def _cleanup_expired_entries(self, cell_index: int, current_time: float):
        """Nettoie les entrées expirées pour une cellule donnée"""
        if cell_index in self.journal:
            # Utiliser le timeout le plus long pour garder l'historique
            max_timeout = max(self.consensus_timeout, self.validated_timeout)
            self.journal[cell_index] = [
                entry
                for entry in self.journal[cell_index]
                if current_time - entry.timestamp <= max_timeout
            ]

    def get_golden_zones_array(self, total_cells: int) -> np.ndarray:
        """Retourne un array numpy avec les zones d'or marquées"""
        golden_array = np.zeros(total_cells, dtype=int)
        for cell_index in self.golden_zones.keys():
            if 0 <= cell_index < total_cells:
                golden_array[cell_index] = CellValue.GOLDEN_ZONE
        return golden_array

    def get_golden_zones_count(self) -> tuple:
        """Retourne le nombre de zones par statut"""
        consensus_count = sum(
            1 for zone in self.golden_zones.values() if zone['status'] == 'consensus'
        )
        validated_count = sum(
            1 for zone in self.golden_zones.values() if zone['status'] == 'validated'
        )
        return consensus_count, validated_count

    def cleanup_all_expired(self):
        """Nettoie toutes les entrées expirées (appel périodique)"""
        current_time = time.time()
        cells_to_remove = []

        for cell_index in list(self.journal.keys()):
            self._cleanup_expired_entries(cell_index, current_time)
            if not self.journal[cell_index]:
                cells_to_remove.append(cell_index)

        for cell_index in cells_to_remove:
            del self.journal[cell_index]


class MapMergerConfig:
    def __init__(self, config_file=None):
        self.config_file = config_file
        self.config = self._load_config()
        # Lecture des options CSV
        self.enable_csv = self.get('logging.enable_csv', False) or self.get(
            'enable_csv', False
        )
        self.csv_path = self.get('logging.csv_path', '/tmp') or self.get(
            'csv_path', '/tmp'
        )

    def _load_config(self):
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                return yaml.safe_load(f)
        else:
            return self._default_config()

    def _default_config(self):
        return {
            'swarm': {
                'robot_prefixes': [
                    'crazyflie',
                    'crazyflie1',
                    'crazyflie2',
                    'crazyflie3',
                    'crazyflie4',
                    'crazyflie5',
                    'crazyflie6',
                    'crazyflie7',
                ],
                'max_robots': 8,
                'use_sim_time': True,
            },
            'pheromones': {
                'increment': 15,
                'evaporation': 0.3,
                'max_value': 100,
                'min_value': 0,
            },
            'consensus': {'min_robots': 2, 'tolerance': 0.5},
            'fusion_weights': {'stigmergy': 0.5, 'consensus': 0.3, 'union': 0.2},
            'temporal': {'smoothing_window': 5},
            'topics': {
                'input_maps': '/map',
                'output_merged': '/map',  # Topic principal pour RViz
                'output_weighted': '/map_fusion_weighted',  # Topic pondéré pour RViz
                'output_coverage_image': 'coverage_image',
            },
            'logging': {'debug_level': 'info', 'log_frequency': 10},
        }

    def get(self, key_path, default=None):
        keys = key_path.split('.')
        value = self.config
        for key in keys:
            if isinstance(value, dict) and key in value:
                value = value[key]
            else:
                return default
        return value


# --- NODE PRINCIPAL ---
class MapMergerNode(Node):
    def __init__(self, config_file=None):
        super().__init__('map_merger')

        # Déclarer les paramètres ROS2 d'abord
        self.declare_parameter(
            'robot_prefixes', ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3', 'crazyflie4', 'crazyflie5', 'crazyflie6', 'crazyflie7']
        )
        # use_sim_time est automatiquement déclaré par ROS2, pas besoin de le redéclarer

        # Déclarer tous les autres paramètres avec valeurs par défaut
        self.declare_parameter('phero_increment', 15)
        self.declare_parameter('phero_evaporation', 0.3)
        self.declare_parameter('phero_max_value', 100)
        self.declare_parameter('phero_min_value', 0)
        self.declare_parameter('min_robots_consensus', 2)
        self.declare_parameter('consensus_tolerance', 0.5)
        self.declare_parameter('weight_merged', 0.5)
        self.declare_parameter('weight_consensus', 0.3)
        self.declare_parameter('weight_union', 0.2)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('input_topic', '/map')
        self.declare_parameter('output_merged', '/map')  # Topic principal pour RViz
        self.declare_parameter('output_weighted', '/map_fusion_weighted')  # Topic pondéré pour RViz

        # Charger la configuration YAML
        self.config = MapMergerConfig(config_file)

        # --- PARAMS : Lire d'abord les paramètres ROS2, puis fallback sur YAML ---
        # Robot prefixes depuis ROS2 en priorité
        try:
            ros_robot_prefixes = self.get_parameter('robot_prefixes').get_parameter_value().string_array_value
            if ros_robot_prefixes:
                self.robot_prefixes = ros_robot_prefixes
                self.get_logger().debug(f"[MAP_MERGER_MODULAR] Robot prefixes depuis ROS2: {self.robot_prefixes}")
            else:
                self.robot_prefixes = self.config.get('swarm.robot_prefixes')
                self.get_logger().debug(f"[MAP_MERGER_MODULAR] Robot prefixes depuis YAML: {self.robot_prefixes}")
        except Exception as e:
            self.get_logger().debug(f"[MAP_MERGER_MODULAR] Paramètre ROS2 robot_prefixes non trouvé: {e}")
            self.robot_prefixes = self.config.get('swarm.robot_prefixes')
            self.get_logger().debug(f"[MAP_MERGER_MODULAR] Robot prefixes depuis YAML: {self.robot_prefixes}")

        # Validation et conversion de type pour robot_prefixes
        if not self.robot_prefixes:
            self.robot_prefixes = ['crazyflie']  # valeur par défaut

        # Autres paramètres avec fallback YAML et validation de type sécurisée
        self.pheromone_inc = self._get_safe_float(
            'phero_increment', 'pheromones.increment', 25  # Augmenté de 15 à 25 pour liens plus forts
        )
        self.pheromone_evap = self._get_safe_float(
            'phero_evaporation', 'pheromones.evaporation', 0.15  # Réduit de 0.3 à 0.15 pour persistance
        )
        self.pheromone_max = self._get_safe_float(
            'phero_max_value', 'pheromones.max_value', 150  # Augmenté de 100 à 150 pour intensité
        )
        self.pheromone_min = self._get_safe_float(
            'phero_min_value', 'pheromones.min_value', 0
        )
        self.consensus_min = self._get_safe_int(
            'min_robots_consensus', 'consensus.min_robots', 2
        )
        self.consensus_tol = self._get_safe_float(
            'consensus_tolerance', 'consensus.tolerance', 0.5
        )
        self.weight_stig = self._get_safe_float(
            'weight_merged', 'fusion_weights.stigmergy', 0.6  # Augmenté de 0.5 à 0.6 pour stigmergie prioritaire
        )
        self.weight_cons = self._get_safe_float(
            'weight_consensus', 'fusion_weights.consensus', 0.25  # Réduit de 0.3 à 0.25
        )
        self.weight_union = self._get_safe_float(
            'weight_union', 'fusion_weights.union', 0.15  # Réduit de 0.2 à 0.15
        )
        self.smoothing_window = self._get_safe_int(
            'smoothing_window', 'temporal.smoothing_window', 5
        )
        self.input_topic = self._get_safe_str(
            'input_topic', 'topics.input_maps', '/map'
        )
        self.output_merged = self._get_safe_str(
            'output_merged', 'topics.output_merged', '/map'
        )
        self.output_weighted = self._get_safe_str(
            'output_weighted', 'topics.output_weighted', '/map_fusion_weighted'
        )

        # --- INITIALISATION SYSTÈME ZONES D'OR ULTRA-ROBUSTE ---
        self.golden_zones_enabled = self.config.get('golden_zones.enable', True)
        if self.golden_zones_enabled:
            # Paramètres ULTRA-ROBUSTES avec validation de type sécurisée
            creation_threshold = self._get_safe_int(
                '', 'golden_zones.creation_threshold', 3
            )
            validation_threshold = self._get_safe_int(
                '', 'golden_zones.validation_threshold', 4
            )
            disappear_threshold = self._get_safe_int(
                '', 'golden_zones.disappear_threshold', 5
            )
            creation_timeout = self._get_safe_float(
                '', 'golden_zones.creation_timeout', 30.0
            )
            consensus_timeout = self._get_safe_float(
                '', 'golden_zones.consensus_timeout', 600.0
            )
            validated_timeout = self._get_safe_float(
                '', 'golden_zones.validated_timeout', 1200.0
            )
            min_confidence = self._get_safe_float(
                '', 'golden_zones.min_confidence', 0.8
            )
            validation_confidence = self._get_safe_float(
                '', 'golden_zones.validation_confidence', 0.9
            )

            # Toujours utiliser la version de base pour éviter les erreurs de typage
            self.golden_zone_manager = GoldenZoneManager(
                consensus_threshold=creation_threshold,
                validation_threshold=validation_threshold,
                disappear_threshold=disappear_threshold,
                consensus_timeout=consensus_timeout,
                validated_timeout=validated_timeout,
                logger=self.get_logger(),
            )

            # Log de la version utilisée
            if ULTRA_ROBUST_AVAILABLE:
                self.get_logger().debug(
                    "[ZONES D'OR] Version de base utilisée (compatible avec ultra-robuste)"
                )
            else:
                self.get_logger().debug("[ZONES D'OR] Version de base utilisée")

            self.golden_cell_value = self._get_safe_int(
                '', 'golden_zones.golden_cell_value', 75
            )
            self.golden_topic = self._get_safe_str(
                '', 'golden_zones.topic_golden_zones', '/map_golden_zones'
            )

            self.get_logger().debug(
                f"[ZONES D'OR] ULTRA-ROBUSTES Activées - Création: {creation_threshold}, Validation: {validation_threshold}, Disparition: {disappear_threshold}"
            )
            self.get_logger().debug(
                f"[ZONES D'OR] Timeouts: création={creation_timeout}s, consensus={consensus_timeout}s, validée={validated_timeout}s"
            )
            self.get_logger().debug(
                f"[ZONES D'OR] Confiance: min={min_confidence}, validation={validation_confidence}"
            )
        else:
            self.golden_zone_manager = None
            self.get_logger().debug("[ZONES D'OR] Désactivées par configuration")

        # Compléter l'initialisation avec valeurs par défaut sécurisées
        coverage_topic = self.config.get('topics.output_coverage_image')
        self.output_coverage_image = (
            str(coverage_topic) if coverage_topic is not None else 'coverage_image'
        )
        log_freq = self.config.get('logging.log_frequency')
        if log_freq is not None and not isinstance(log_freq, dict):
            try:
                self.log_frequency = int(log_freq)
            except (ValueError, TypeError):
                self.log_frequency = 10
        else:
            self.log_frequency = 10

        # --- BUFFERS ---
        self.maps = {}  # Dernière carte reçue de chaque robot
        self.pheromones = None  # Buffer stigmergie
        self.history_weighted = deque(maxlen=self.smoothing_window)
        self.observation_count = None  # Pour la couverture
        self.width = 0
        self.height = 0
        self.n_cells = 0
        self.bridge = CvBridge()

        # --- ROS PUB/SUB avec QoS anti-flickering ---
        from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

        # Configuration QoS pour éviter le flickering
        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL  # Persistance pour éviter le flickering
        )

        self.publisher = self.create_publisher(OccupancyGrid, self.output_merged, qos_profile)
        self.weighted_pub = self.create_publisher(
            OccupancyGrid, self.output_weighted, qos_profile
        )
        
        # PUBLISHER DÉDIÉ POUR STIGMERGIE PURE
        self.stigmergie_pub = self.create_publisher(
            OccupancyGrid, '/map_stigmergie_pure', qos_profile
        )
        
        self.coverage_img_pub = self.create_publisher(
            Image, self.output_coverage_image, qos_profile
        )

        # CRÉER LE PUBLISHER POUR LES ZONES D'OR
        if self.golden_zones_enabled:
            self.golden_zones_pub = self.create_publisher(
                OccupancyGrid, self.golden_topic, qos_profile
            )
            self.get_logger().debug(f"[ZONES D'OR] Publisher créé sur {self.golden_topic}")
        else:
            self.golden_zones_pub = None

        # Créer les souscriptions avec logs
        for prefix in self.robot_prefixes:
            topic = f'/{prefix}{self.input_topic}'
            self.get_logger().debug(
                f"[MAP_MERGER_MODULAR] Souscription au topic: {topic}"
            )
            self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, p=prefix: self.map_callback(msg, p),
                10,
            )

        self.create_timer(0.2, self.process_maps)  # 5Hz
        self.get_logger().debug(
            "[MAP_MERGER_MODULAR] Node initialisé avec stigmergie, consensus, fusion pondérée, lissage."
        )
        self.get_logger().debug(
            f"[MAP_MERGER_MODULAR] Config chargée: {self.config.config_file or 'défaut'}"
        )

        # --- CSV LOGGING avec gestion sécurisée des types ---
        self.enable_csv = self.config.enable_csv
        csv_path_raw = self.config.csv_path
        self.csv_path = str(csv_path_raw) if csv_path_raw is not None else '/tmp'
        self.csv_file = None
        self.csv_writer = None
        self.tick_count = 0
        if self.enable_csv:
            os.makedirs(self.csv_path, exist_ok=True)
            self.csv_file_path = os.path.join(self.csv_path, 'map_merger_stats.csv')
            self.csv_file = open(self.csv_file_path, 'w')
            self.csv_file.write('tick,timestamp,n_maps,n_occ,n_golden\n')

    def get_parameter_or_yaml(self, ros_param_name: str, yaml_key_path: str) -> Any:
        """Lire un paramètre depuis ROS2 ou fallback sur YAML avec gestion des types"""
        try:
            if self.has_parameter(ros_param_name):
                value = self.get_parameter(ros_param_name).value
                return value
            else:
                value = self.config.get(yaml_key_path)
                return value
        except:
            value = self.config.get(yaml_key_path)
            return value

    def _get_safe_float(
        self, ros_param_name: str, yaml_key_path: str, default_value: float
    ) -> float:
        """Obtenir une valeur float de manière sécurisée"""
        value = self.get_parameter_or_yaml(ros_param_name, yaml_key_path)
        if value is None:
            return float(default_value)
        try:
            # Vérifier si c'est un dict (cas problématique)
            if isinstance(value, dict):
                return float(default_value)
            return float(value)
        except (ValueError, TypeError):
            return float(default_value)

    def _get_safe_int(
        self, ros_param_name: str, yaml_key_path: str, default_value: int
    ) -> int:
        """Obtenir une valeur int de manière sécurisée"""
        value = self.get_parameter_or_yaml(ros_param_name, yaml_key_path)
        if value is None:
            return int(default_value)
        try:
            # Vérifier si c'est un dict (cas problématique)
            if isinstance(value, dict):
                return int(default_value)
            return int(value)
        except (ValueError, TypeError):
            return int(default_value)

    def _get_safe_str(
        self, ros_param_name: str, yaml_key_path: str, default_value: str
    ) -> str:
        """Obtenir une valeur str de manière sécurisée"""
        value = self.get_parameter_or_yaml(ros_param_name, yaml_key_path)
        if value is None:
            return str(default_value)
        try:
            # Vérifier si c'est un dict (cas problématique)
            if isinstance(value, dict):
                return str(default_value)
            return str(value)
        except (ValueError, TypeError):
            return str(default_value)

    def map_callback(self, msg, prefix):
        self.maps[prefix] = msg
        if self.n_cells == 0:
            self.n_cells = len(msg.data)
            self.width = msg.info.width
            self.height = msg.info.height
            self.pheromones = np.zeros(self.n_cells, dtype=float)
            self.observation_count = np.zeros(self.n_cells, dtype=float)
            self.get_logger().debug(
                f"[MAP_MERGER_MODULAR] Dimensions carte: {self.width}x{self.height}"
            )

        # --- ALIMENTER LE JOURNAL HISTORIQUE DES ZONES D'OR ---
        if self.golden_zones_enabled and self.golden_zone_manager is not None:
            data = np.array(msg.data)
            for cell_index in range(len(data)):
                cell_value = data[cell_index]
                if cell_value != CellValue.UNKNOWN:  # Ignorer les cellules inconnues
                    is_obstacle = cell_value == CellValue.OCCUPIED
                    self.golden_zone_manager.add_observation(
                        cell_index, prefix, is_obstacle
                    )

    def process_maps(self):
        if len(self.maps) < 1 or self.n_cells == 0:
            self.get_logger().warn(
                f"[MAP_MERGER_MODULAR] Aucun map reçu, fusion non effectuée. Robots attendus: {self.robot_prefixes}"
            )
            return

        # Vérification supplémentaire de la validité des cartes
        try:
            base = next(iter(self.maps.values()))
            if base is None or base.data is None:
                self.get_logger().warn("[MAP_MERGER_MODULAR] Cartes invalides détectées")
                return
        except (StopIteration, AttributeError) as e:
            self.get_logger().warn(f"[MAP_MERGER_MODULAR] Erreur accès cartes: {e}")
            return

        # --- 1. STIGMERGIE ULTRA-RENFORCÉE avec diffusion ---
        self._update_pheromones()
        if self.pheromones is not None:
            # SEUIL PLUS BAS pour détecter plus de liens stigmergiques
            stigmergy_threshold = self.pheromone_max * 0.3  # Réduit de 0.5 à 0.3 pour plus de sensibilité
            stigmergy_map = (self.pheromones >= stigmergy_threshold).astype(
                int
            ) * CellValue.OCCUPIED
            
            # AJOUT : Intensifier les zones de forte activité stigmergique
            high_intensity_threshold = self.pheromone_max * 0.7
            high_intensity_zones = (self.pheromones >= high_intensity_threshold)
            stigmergy_map[high_intensity_zones] = CellValue.OCCUPIED
        else:
            stigmergy_map = np.zeros(self.n_cells, dtype=int)

        # --- 2. CONSENSUS ---
        consensus_map = self._compute_consensus_map()

        # --- MISE À JOUR ZONES D'OR DEPUIS CONSENSUS ET JOURNAL ---
        if self.golden_zones_enabled and self.golden_zone_manager is not None:
            # Obtenir les obstacles en consensus (indices simples)
            consensus_obstacles = set()
            consensus_indices = np.where(consensus_map == CellValue.OCCUPIED)[0]
            for cell_index in consensus_indices:
                consensus_obstacles.add(cell_index)

            # Mise à jour sécurisée selon le type de manager
            try:
                # Utiliser getattr pour éviter les erreurs de typage
                update_consensus = getattr(
                    self.golden_zone_manager, 'update_from_consensus', None
                )
                if update_consensus and callable(update_consensus):
                    update_consensus(consensus_obstacles)

                # Appeler aussi update_golden_zones si disponible
                update_zones = getattr(
                    self.golden_zone_manager, 'update_golden_zones', None
                )
                if update_zones and callable(update_zones):
                    update_zones()

            except Exception as e:
                self.get_logger().warn(f"[ZONES D'OR] Erreur mise à jour: {e}")

            # Nettoyer périodiquement (toutes les 100 itérations)
            if self.tick_count % 100 == 0:
                try:
                    cleanup_method = getattr(
                        self.golden_zone_manager, 'cleanup_all_expired', None
                    )
                    if cleanup_method and callable(cleanup_method):
                        cleanup_method()
                except Exception as e:
                    self.get_logger().warn(f"[ZONES D'OR] Erreur nettoyage: {e}")

        # Obtenir les zones d'or actuelles
        golden_zones_map = np.zeros(self.n_cells, dtype=int)
        if self.golden_zones_enabled and self.golden_zone_manager is not None:
            try:
                # Méthode ultra-sécurisée : détecter le type de manager et adapter l'accès
                zones_found = False

                # Tentative 1 : Version ultra-robuste avec zones coordonnées (x,y)
                if hasattr(self.golden_zone_manager, 'zones') and not zones_found:
                    try:
                        zones_attr = getattr(self.golden_zone_manager, 'zones', {})
                        for (x, y), zone_info in zones_attr.items():
                            if 0 <= x < self.width and 0 <= y < self.height:
                                cell_index = y * self.width + x
                                if cell_index < self.n_cells:
                                    cell_value = getattr(
                                        self.golden_zone_manager,
                                        'golden_cell_value',
                                        self.golden_cell_value,
                                    )
                                    golden_zones_map[cell_index] = cell_value
                        zones_found = True
                    except Exception:
                        pass  # Continuer avec la méthode suivante

                # Tentative 2 : Version de base avec zones indexées
                if (
                    hasattr(self.golden_zone_manager, 'golden_zones')
                    and not zones_found
                ):
                    try:
                        golden_zones_attr = getattr(
                            self.golden_zone_manager, 'golden_zones', {}
                        )
                        for cell_index, zone_info in golden_zones_attr.items():
                            if 0 <= cell_index < self.n_cells:
                                golden_zones_map[cell_index] = self.golden_cell_value
                        zones_found = True
                    except Exception:
                        pass  # Continuer avec la méthode suivante

                # Tentative 3 : Méthode get_golden_zones_array si disponible
                if (
                    hasattr(self.golden_zone_manager, 'get_golden_zones_array')
                    and not zones_found
                ):
                    try:
                        golden_array = getattr(
                            self.golden_zone_manager, 'get_golden_zones_array'
                        )(self.n_cells)
                        golden_zones_map = golden_array.copy()
                        zones_found = True
                    except Exception:
                        pass

                if not zones_found:
                    # Aucune méthode n'a fonctionné - log debug seulement
                    self.get_logger().debug(
                        "[ZONES D'OR] Aucune méthode d'accès aux zones disponible"
                    )

            except Exception as e:
                self.get_logger().warn(f"[ZONES D'OR] Erreur accès zones: {e}")
                # En cas d'erreur globale, garder la carte vide

        # --- 3. UNION ---
        union_map = self._compute_union_map()

        # --- 4. CARTE DE COUVERTURE ÉTENDUE ---
        coverage_weight_map = self._compute_coverage_weighted_map()

        # --- 5. CARTE STIGMERGIQUE PURE AVEC FILS PHÉROMONAUX VISIBLES ---
        # STIGMERGIE ULTRA-VISIBLE : utiliser des valeurs RViz compatibles
        stigmergie_pure = np.zeros(self.n_cells, dtype=int)
        
        # AJOUTER LES FILS DE STIGMERGIE (phéromones) DIRECTEMENT VISIBLES
        if self.pheromones is not None:
            # Normaliser les phéromones pour créer des valeurs intermédiaires visibles dans RViz
            pheromone_normalized = np.clip(self.pheromones / self.pheromone_max, 0, 1)
            
            # Créer des FILS STIGMERGIQUES avec des VALEURS RVIZ COMPATIBLES
            # Utiliser les valeurs d'OccupancyGrid (-1 = unknown, 0 = free, 100 = occupied)
            # Valeurs intermédiaires : 25, 50, 75 pour différents niveaux de pheromone
            stigmergie_pure = np.where(
                pheromone_normalized > 0.8,   # Très fort = obstacle noir (100)
                100,  # OCCUPIED - noir complet
                np.where(
                    pheromone_normalized > 0.5,   # Fort = gris très foncé (75)
                    75,   # Gris très foncé - bien visible
                    np.where(
                        pheromone_normalized > 0.3,   # Moyen = gris moyen (50)
                        50,   # Gris moyen - visible
                        np.where(
                            pheromone_normalized > 0.1,   # Faible = gris clair (25)
                            25,   # Gris clair - encore visible
                            0     # FREE = blanc complet
                        )
                    )
                )
            ).astype(int)
        
        # Fusion STIGMERGIE DOMINANTE : la stigmergie est TOUJOURS visible
        # ALGORITHME DIAMANTS : la stigmergie structure l'émergence, elle doit être vue
        fusion_weighted = np.maximum(
            stigmergie_pure,                    # STIGMERGIE PURE = BASE VISIBLE
            (consensus_map * 0.5).astype(int)  # Consensus très atténué (50% max)
        )
        
        # AJOUTER une couche stigmergique VERTE distinctive visible
        # Utiliser une valeur spécifique pour stigmergie active : 85 (entre 75 et 100)
        stigmergie_active_zones = (pheromone_normalized > 0.2)  # Seuil bas pour visibilité
        fusion_weighted[stigmergie_active_zones] = np.maximum(
            fusion_weighted[stigmergie_active_zones], 
            85  # Valeur distinctive pour stigmergie active
        )

        # PRIORITÉ AUX ZONES D'OR : elles sont TOUJOURS visibles
        if self.golden_zones_enabled:
            # Les zones d'or s'ajoutent à la fusion (ne remplacent pas, s'ajoutent)
            fusion_weighted = np.where(
                golden_zones_map == CellValue.GOLDEN_ZONE,
                CellValue.OCCUPIED,  # Zone d'or = obstacle visible
                fusion_weighted,
            )

        # --- 6. LISSAGE TEMPOREL ---
        self.history_weighted.append(fusion_weighted)
        if len(self.history_weighted) > 0:
            fusion_smoothed = np.mean(self.history_weighted, axis=0)
        else:
            fusion_smoothed = fusion_weighted
        fusion_smoothed = np.round(fusion_smoothed).astype(int)

        # GARANTIR QUE LES ZONES D'OR RESTENT VISIBLES APRÈS LISSAGE
        if self.golden_zones_enabled:
            fusion_smoothed = np.where(
                golden_zones_map == CellValue.GOLDEN_ZONE,
                CellValue.OCCUPIED,  # Zones d'or toujours visibles
                fusion_smoothed,
            )

        # --- 7. Couverture ---
        self._update_observation_count()

        # --- 8. Publication ---
        self._publish_occupancy_grid(
            self.publisher, fusion_smoothed
        )  # Carte fusionnée avec zones d'or
        self._publish_occupancy_grid(
            self.weighted_pub, fusion_smoothed
        )  # Identique pour compatibilité
        
        # PUBLIER LA STIGMERGIE PURE (avant lissage pour garder les détails)
        self._publish_occupancy_grid(
            self.stigmergie_pub, stigmergie_pure
        )  # Stigmergie pure sans lissage

        # --- PUBLIER LES ZONES D'OR SUR TOPIC DÉDIÉ ---
        if self.golden_zones_enabled:
            self._publish_golden_zones_map(golden_zones_map)

        try:
            self._publish_coverage_image()
        except Exception as e:
            self.get_logger().warn(
                f"[MAP_MERGER_MODULAR] Erreur lors de la publication de la heatmap de couverture : {e}"
            )

        # --- 9. Log (très réduit pour éviter la pollution) ---
        if self.tick_count % (self.log_frequency * 10) == 0:  # 10x moins fréquent
            n_occ = np.sum(fusion_smoothed == CellValue.OCCUPIED)
            if self.golden_zones_enabled and self.golden_zone_manager is not None:
                try:
                    consensus_count, validated_count = (
                        self.golden_zone_manager.get_golden_zones_count()
                    )
                    total_golden = consensus_count + validated_count
                    # Passer en DEBUG pour réduire la verbosité
                    self.get_logger().debug(
                        f"[MAP_MERGER_MODULAR] Fusion tick | OCCUPIED: {n_occ} | ZONES D'OR: {total_golden} (consensus:{consensus_count}, validé:{validated_count}) | Maps reçues: {len(self.maps)}"
                    )
                except Exception:
                    # Si get_golden_zones_count échoue, compter manuellement
                    n_golden = np.sum(golden_zones_map > 0)
                    self.get_logger().debug(
                        f"[MAP_MERGER_MODULAR] Fusion tick | OCCUPIED: {n_occ} | ZONES D'OR: {n_golden} | Maps reçues: {len(self.maps)}"
                    )
            else:
                self.get_logger().debug(
                    f"[MAP_MERGER_MODULAR] Fusion tick | OCCUPIED: {n_occ} | ZONES D'OR: 0 | Maps reçues: {len(self.maps)}"
                )

        # --- 10. CSV Logging ---
        if self.enable_csv and self.csv_file:
            import time

            n_occ = np.sum(fusion_smoothed == CellValue.OCCUPIED)
            if self.golden_zones_enabled and self.golden_zone_manager is not None:
                try:
                    consensus_count, validated_count = (
                        self.golden_zone_manager.get_golden_zones_count()
                    )
                    total_golden = consensus_count + validated_count
                except Exception:
                    # Si get_golden_zones_count échoue, compter manuellement
                    total_golden = np.sum(golden_zones_map > 0)
            else:
                total_golden = 0
            self.csv_file.write(
                f"{self.tick_count},{time.time()},{len(self.maps)},{n_occ},{total_golden}\n"
            )
            self.csv_file.flush()
        self.tick_count += 1

    def _update_pheromones(self):
        # Stigmergie RENFORCÉE : renforcement/évaporation + DIFFUSION STIGMERGIQUE
        if self.pheromones is None:
            return
        
        # Évaporation des phéromones
        self.pheromones *= 1.0 - self.pheromone_evap
        
        # Renforcement par les observations actuelles
        for msg in self.maps.values():
            data = np.array(msg.data)
            self.pheromones[data == CellValue.OCCUPIED] += self.pheromone_inc
        
        # --- NOUVEAU: DIFFUSION STIGMERGIQUE POUR LIENS CONNECTIFS ---
        if self.width > 0 and self.height > 0:
            # Reshape pour traitement 2D
            pheromone_2d = self.pheromones.reshape((self.height, self.width))
            
            # Appliquer un filtre gaussien léger pour diffusion stigmergique
            if np.max(pheromone_2d) > 0:
                pheromone_diffused = cv2.GaussianBlur(pheromone_2d.astype(np.float32), (3, 3), 0.8)  # pylint: disable=no-member
                
                # Combiner original + diffusion pour créer des liens
                diffusion_factor = 0.3  # 30% de diffusion pour créer les connexions
                pheromone_2d = pheromone_2d + (pheromone_diffused * diffusion_factor)
                
                # Remettre en forme 1D
                self.pheromones = pheromone_2d.flatten()
        
        # Clipping final
        self.pheromones = np.clip(
            self.pheromones, self.pheromone_min, self.pheromone_max
        )

    def _compute_consensus_map(self):
        # Consensus distribué OCCUPIED si vu par min_robots ou tolérance
        votes_occ = np.zeros(self.n_cells)
        for msg in self.maps.values():
            data = np.array(msg.data)
            votes_occ[data == CellValue.OCCUPIED] += 1
        occ_thresh = max(self.consensus_min, int(self.consensus_tol * len(self.maps)))
        consensus_map = (votes_occ >= occ_thresh).astype(int) * CellValue.OCCUPIED
        return consensus_map

    def _compute_union_map(self):
        # Union permissive : OCCUPIED si au moins un robot le voit
        union = np.zeros(self.n_cells)
        for msg in self.maps.values():
            data = np.array(msg.data)
            union[data == CellValue.OCCUPIED] = 1
        return union.astype(int) * CellValue.OCCUPIED

    def _compute_coverage_weighted_map(self):
        """Calcule une carte pondérée basée sur la couverture des zones explorées"""
        if self.observation_count is None:
            return np.zeros(self.n_cells)

        # Normaliser le nombre d'observations (0 à 1)
        max_obs = (
            np.max(self.observation_count) if np.max(self.observation_count) > 0 else 1
        )
        coverage_norm = self.observation_count / max_obs

        # Créer une carte pondérée: plus une zone est observée, plus elle est fiable
        # Seuil: considérer comme OCCUPIED si observé par au moins 30% du max
        coverage_threshold = 0.3
        coverage_map = (coverage_norm >= coverage_threshold).astype(
            int
        ) * CellValue.OCCUPIED

        return coverage_map

    def _update_observation_count(self):
        """Met à jour le compteur d'observation (algorithme original préservé)"""
        if self.observation_count is None:
            return

        # Algorithme original des observations (pas de modification)
        for msg in self.maps.values():
            data = np.array(msg.data)
            self.observation_count[data != CellValue.UNKNOWN] += 1

    def _publish_occupancy_grid(self, publisher, data):
        """Publie une grille d'occupation avec vérifications robustes"""
        if self.n_cells == 0 or not self.maps:
            return

        try:
            msg = OccupancyGrid()
            base = next(iter(self.maps.values()))
            msg.header = base.header
            msg.header.frame_id = "map"  # Force le frame_id correct pour RViz
            msg.header.stamp = self.get_clock().now().to_msg()  # Timestamp actuel
            msg.info = base.info

            # Vérification et conversion sécurisée des données
            if len(data) != self.n_cells:
                self.get_logger().warn(f"[MAP_MERGER] Taille de données incorrecte: {len(data)} != {self.n_cells}")
                return

            msg.data = [int(v) for v in data]
            publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f"[MAP_MERGER] Erreur publication grille: {e}")

    def _publish_golden_zones_map(self, golden_zones_data):
        """Publie une carte dédiée aux zones d'or pour visualisation RViz"""
        if self.n_cells == 0 or not self.golden_zones_enabled or not self.maps:
            return

        try:
            # Utiliser directement les données ultra-robustes si disponible
            if (
                hasattr(self, 'golden_zone_manager')
                and self.golden_zone_manager is not None
                and hasattr(self.golden_zone_manager, 'get_golden_zones_map')
            ):

                # Utiliser la méthode native de l'ultra-robuste
                base = next(iter(self.maps.values()))
                try:
                    golden_msg = getattr(
                        self.golden_zone_manager, 'get_golden_zones_map'
                    )(
                        width=self.width,
                        height=self.height,
                        resolution=base.info.resolution,
                        origin_x=base.info.origin.position.x,
                        origin_y=base.info.origin.position.y,
                    )
                    golden_msg.header.frame_id = "map"
                    if self.golden_zones_pub is not None:
                        self.golden_zones_pub.publish(golden_msg)
                except Exception:
                    # Si la méthode ultra-robuste échoue, utiliser le fallback
                    raise Exception("Fallback to standard method")

            else:
                # Fallback : utiliser les données passées en paramètre
                golden_map = np.where(
                    golden_zones_data > 0, self.golden_cell_value, CellValue.FREE
                )

                msg = OccupancyGrid()
                base = next(iter(self.maps.values()))
                msg.header = base.header
                msg.header.frame_id = "map"
                msg.info = base.info
                msg.data = [int(v) for v in golden_map]
                if self.golden_zones_pub is not None:
                    self.golden_zones_pub.publish(msg)

            # Log occasionnel avec statistiques ultra-robustes
            if self.tick_count % 100 == 0:
                if (
                    hasattr(self, 'golden_zone_manager')
                    and self.golden_zone_manager is not None
                    and hasattr(self.golden_zone_manager, 'get_statistics')
                ):
                    try:
                        stats = getattr(self.golden_zone_manager, 'get_statistics')()
                        self.get_logger().debug(
                            f"[ZONES D'OR] Publié {stats['total']} zones "
                            f"(consensus:{stats['consensus']}, validé:{stats['validated']}) "
                            f"sur {self.golden_topic}"
                        )
                    except Exception:
                        n_golden = np.sum(golden_zones_data > 0)
                        if n_golden > 0:
                            self.get_logger().debug(
                                f"[ZONES D'OR] Publié {n_golden} zones d'or sur {self.golden_topic}"
                            )
                else:
                    n_golden = np.sum(golden_zones_data > 0)
                    if n_golden > 0:
                        self.get_logger().debug(
                            f"[ZONES D'OR] Publié {n_golden} zones d'or sur {self.golden_topic}"
                        )

        except Exception as e:
            self.get_logger().error(f"[ZONES D'OR] Erreur publication: {e}")
            # En cas d'erreur, au moins essayer la publication de base
            try:
                golden_map = np.where(
                    golden_zones_data > 0, self.golden_cell_value, CellValue.FREE
                )

                msg = OccupancyGrid()
                base = next(iter(self.maps.values()))
                msg.header = base.header
                msg.header.frame_id = "map"
                msg.info = base.info
                msg.data = [int(v) for v in golden_map]
                if self.golden_zones_pub is not None:
                    self.golden_zones_pub.publish(msg)
            except Exception as e2:
                self.get_logger().error(
                    f"[ZONES D'OR] Erreur publication fallback: {e2}"
                )

    def _publish_coverage_image(self):
        """Publie une image de couverture avec STIGMERGIE ULTRA-VISIBLE et LIENS CONNECTIFS"""
        if self.observation_count is None or self.width == 0 or self.height == 0:
            return

        # --- ÉTAPE 1: FUSION OBSERVATION + PHÉROMONES POUR LIENS STIGMERGIQUES ---
        # Reshape des données d'observation
        obs_img = self.observation_count.reshape((self.height, self.width))
        
        # Intégrer les phéromones pour créer des LIENS STIGMERGIQUES visibles
        if self.pheromones is not None:
            pheromone_img = self.pheromones.reshape((self.height, self.width))
            # FUSION: Observation + Phéromones avec pondération STIGMERGIQUE
            stigmergy_img = obs_img + (pheromone_img * 0.8)  # Intensifier les liens phéromonaux
        else:
            stigmergy_img = obs_img

        # --- ÉTAPE 2: ENHANCEMENT STIGMERGIQUE AVANCÉ ---
        # Appliquer un filtre gaussien pour créer des HALOS STIGMERGIQUES
        if np.max(stigmergy_img) > 0:
            # Gaussian blur pour créer des connexions visuelles
            stigmergy_blur = cv2.GaussianBlur(stigmergy_img.astype(np.float32), (7, 7), 2.0)  # pylint: disable=no-member
            
            # Morphologie pour renforcer les CONNEXIONS STIGMERGIQUES
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))  # pylint: disable=no-member
            stigmergy_dilated = cv2.dilate(stigmergy_blur, kernel, iterations=2)  # pylint: disable=no-member
            
            # COMBINAISON: Original + Blur + Dilatation pour liens ultra-visibles
            stigmergy_enhanced = stigmergy_img + (stigmergy_blur * 0.6) + (stigmergy_dilated * 0.4)
        else:
            stigmergy_enhanced = stigmergy_img

        # ZOOM DYNAMIQUE ADAPTATIF basé sur l'étendue des observations
        observation_threshold = (
            np.max(stigmergy_enhanced) * 0.03 if np.max(stigmergy_enhanced) > 0 else 0
        )  # Seuil encore plus bas pour capturer plus de détails
        active_zones = np.where(stigmergy_enhanced > observation_threshold)

        # Taille finale de l'image (encore plus grande pour détails)
        final_width = self.width * 20  # Augmenté à 20 pour ultra-détail
        final_height = self.height * 20

        if len(active_zones[0]) > 0:
            # Calculer la bounding box des zones actives
            min_y, max_y = np.min(active_zones[0]), np.max(active_zones[0])
            min_x, max_x = np.min(active_zones[1]), np.max(active_zones[1])

            # Marge réduite pour zoom plus agressif
            margin = max(
                2, min(self.width, self.height) // 40
            )  # Marge réduite de moitié
            min_y = max(0, min_y - margin)
            max_y = min(self.height - 1, max_y + margin)
            min_x = max(0, min_x - margin)
            max_x = min(self.width - 1, max_x + margin)

            # Calculer le ratio de couverture pour ajuster le zoom
            covered_area = (max_y - min_y + 1) * (max_x - min_x + 1)
            total_area = self.height * self.width
            coverage_ratio = covered_area / total_area

            # Zoom MÉGA ULTRA AGRESSIF : seuils encore plus étendus
            if (
                coverage_ratio < 0.35
            ):  # MEGA ZOOM - zoom maximal sur une très large plage
                crop_img = stigmergy_enhanced[min_y : max_y + 1, min_x : max_x + 1]
                if coverage_ratio < 0.05:
                    zoom_info = "MÉGA ZOOM STIGMERGIQUE (zone minuscule)"
                elif coverage_ratio < 0.15:
                    zoom_info = "ULTRA ZOOM STIGMERGIQUE (début exploration)"
                else:
                    zoom_info = "ZOOM MAXIMAL STIGMERGIQUE (exploration active)"
            elif coverage_ratio < 0.5:  # ZOOM FORT - exploration moyenne
                # Élargir très légèrement la zone de crop
                expand_y = int(
                    (self.height - (max_y - min_y)) * 0.12
                )  # Encore plus réduit
                expand_x = int((self.width - (max_x - min_x)) * 0.12)
                min_y = max(0, min_y - expand_y)
                max_y = min(self.height - 1, max_y + expand_y)
                min_x = max(0, min_x - expand_x)
                max_x = min(self.width - 1, max_x + expand_x)
                crop_img = stigmergy_enhanced[min_y : max_y + 1, min_x : max_x + 1]
                zoom_info = "ZOOM FORT STIGMERGIQUE (exploration moyenne)"
            elif coverage_ratio < 0.7:  # ZOOM MOYEN - exploration large
                expand_y = int((self.height - (max_y - min_y)) * 0.2)
                expand_x = int((self.width - (max_x - min_x)) * 0.2)
                min_y = max(0, min_y - expand_y)
                max_y = min(self.height - 1, max_y + expand_y)
                min_x = max(0, min_x - expand_x)
                max_x = min(self.width - 1, max_x + expand_x)
                crop_img = stigmergy_enhanced[min_y : max_y + 1, min_x : max_x + 1]
                zoom_info = "ZOOM MOYEN STIGMERGIQUE (exploration large)"
            else:  # Vue d'ensemble seulement si exploration quasi-complète
                crop_img = stigmergy_enhanced
                zoom_info = "VUE ENSEMBLE STIGMERGIQUE (exploration quasi-complète)"
        else:
            # Aucune observation - zoom sur le centre par défaut
            center_y, center_x = self.height // 2, self.width // 2
            crop_size = min(self.width, self.height) // 4  # Crop très petit au centre
            min_y = max(0, center_y - crop_size)
            max_y = min(self.height - 1, center_y + crop_size)
            min_x = max(0, center_x - crop_size)
            max_x = min(self.width - 1, center_x + crop_size)
            crop_img = stigmergy_enhanced[min_y : max_y + 1, min_x : max_x + 1]
            zoom_info = "ZOOM CENTRE STIGMERGIQUE (aucune observation)"

        # Redimensionner le crop vers la taille finale
        obs_stretched = cv2.resize(  # pylint: disable=no-member
            crop_img, (final_width, final_height), interpolation=cv2.INTER_CUBIC  # pylint: disable=no-member
        )

        # --- ÉTAPE 3: NORMALISATION ULTRA-INTENSIVE POUR STIGMERGIE ---
        if np.max(obs_stretched) > 0:
            # Normalisation EXTREME pour rendre les liens ultra-visibles
            obs_normalized = (obs_stretched / np.max(obs_stretched) * 240).astype(
                np.uint8
            )  # Augmenté de 180 à 240 pour intensité maximale
        else:
            obs_normalized = np.zeros_like(obs_stretched, dtype=np.uint8)

        # --- ÉTAPE 4: POST-TRAITEMENT STIGMERGIQUE AVANCÉ ---
        # Enhancement par égalisation d'histogramme pour contraste STIGMERGIQUE
        if np.max(obs_normalized) > 0:
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))  # pylint: disable=no-member
            obs_enhanced = clahe.apply(obs_normalized)
        else:
            obs_enhanced = obs_normalized

        # Appliquer contraste gamma pour intensifier les LIENS STIGMERGIQUES
        gamma = 0.7  # Gamma < 1 pour rendre les verts plus brillants
        obs_gamma = np.power(obs_enhanced / 255.0, gamma) * 255.0
        obs_gamma = obs_gamma.astype(np.uint8)

        # --- ÉTAPE 5: FOND ULTRA-SOMBRE POUR CONTRASTE STIGMERGIQUE ---
        background_value = 15  # Fond très sombre pour faire ressortir les verts
        full_coverage = np.where(
            obs_gamma == 0, background_value, background_value + obs_gamma
        )

        # --- ÉTAPE 6: COLORMAP VIRIDIS + POST-TRAITEMENT INTENSIF ---
        color_img = cv2.applyColorMap(full_coverage, cv2.COLORMAP_VIRIDIS)  # pylint: disable=no-member

        # Post-traitement final pour ULTRA-INTENSITÉ STIGMERGIQUE
        # Augmenter la saturation dans l'espace HSV pour verts ultra-vifs
        hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)  # pylint: disable=no-member
        hsv[:,:,1] = cv2.multiply(hsv[:,:,1], 1.8)  # pylint: disable=no-member  # Saturation x1.8
        hsv[:,:,2] = cv2.multiply(hsv[:,:,2], 1.4)  # pylint: disable=no-member  # Luminosité x1.4
        color_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)  # pylint: disable=no-member

        # Enhancement final par convolution pour ÉCLAT STIGMERGIQUE
        kernel = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])  # Filtre de netteté
        color_img = cv2.filter2D(color_img, -1, kernel)  # pylint: disable=no-member

        # Log du zoom adaptatif (occasionnel)
        if self.tick_count % 50 == 0:  # Chaque 50 ticks
            coverage_pct = (
                len(active_zones[0]) / (self.width * self.height) * 100
                if len(active_zones[0]) > 0
                else 0
            )
            self.get_logger().debug(
                f"[STIGMERGIE ULTRA-VISIBLE] {zoom_info} - Couverture: {coverage_pct:.1f}%"
            )

        # Créer le message ROS
        ros_img = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        base = next(iter(self.maps.values()))
        ros_img.header = base.header

        ros_img.width = final_width
        ros_img.height = final_height

        self.coverage_img_pub.publish(ros_img)

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()


# --- MAIN ---
def main(args=None):
    import sys

    config_file = None
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    rclpy.init(args=args)
    node = MapMergerNode(config_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
