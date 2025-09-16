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

# --- MAP_MERGER_NODE.PY ---
# Node ROS2 de fusion collaborative NON permissif (stigmergie stricte, consensus strict, morphologie, filtrage, lissage)
# Ce node applique tous les filtres et restrictions pour une fusion robuste et conservatrice.
# Auteur : Correction Copilot, 2025

from enum import IntEnum
from collections import deque
import os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from scipy.ndimage import label, binary_opening, binary_erosion
import yaml
from typing import Any

class CellValue(IntEnum):
    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

class MapMergerConfig:
    def __init__(self, config_file=None):
        self.config_file = config_file
        self.config = self._load_config()
    def _load_config(self):
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, 'r') as f:
                return yaml.safe_load(f)
        else:
            return self._default_config()
    def _default_config(self):
        return {
            'robot_prefixes': ['crazyflie', 'crazyflie1', 'crazyflie2', 'crazyflie3', 'crazyflie4', 'crazyflie5', 'crazyflie6', 'crazyflie7'],
            'pheromone_inc': 10,
            'pheromone_evap': 0.5,
            'pheromone_max': 100,
            'pheromone_min': 0,
            'consensus_min': 2,
            'consensus_tol': 0.7,
            'enable_morphology': True,
            'min_cluster_size': 5,
            'smoothing_window': 5,
            'input_topic': '/map',
            'output_merged': 'map_merged',
            'output_weighted': 'map_fusion_weighted',
            'output_coverage_image': 'coverage_image',
        }
    def get(self, key, default=None):
        return self.config.get(key, default)

class MapMergerNode(Node):
    def __init__(self, config_file=None):
        super().__init__('map_merger')
        self.config = MapMergerConfig(config_file)
        
        # --- DÉCLARATION DES PARAMÈTRES ROS2 ---
        # Les paramètres ROS2 ont la priorité sur le YAML
        self.declare_parameter('robot_prefixes', self.config.get('robot_prefixes'))
        self.declare_parameter('pheromone_inc', self.config.get('pheromone_inc'))
        self.declare_parameter('pheromone_evap', self.config.get('pheromone_evap'))
        self.declare_parameter('pheromone_max', self.config.get('pheromone_max'))
        self.declare_parameter('pheromone_min', self.config.get('pheromone_min'))
        self.declare_parameter('consensus_min', self.config.get('consensus_min'))
        self.declare_parameter('consensus_tol', self.config.get('consensus_tol'))
        self.declare_parameter('enable_morphology', self.config.get('enable_morphology'))
        self.declare_parameter('min_cluster_size', self.config.get('min_cluster_size'))
        self.declare_parameter('smoothing_window', self.config.get('smoothing_window'))
        self.declare_parameter('input_topic', self.config.get('input_topic'))
        self.declare_parameter('output_merged', self.config.get('output_merged'))
        self.declare_parameter('output_weighted', self.config.get('output_weighted'))
        self.declare_parameter('output_coverage_image', self.config.get('output_coverage_image'))
        
        # --- LECTURE DES PARAMÈTRES ROS2 (priorité) puis YAML (fallback) avec validation de type sécurisée ---
        robot_prefixes_param = self.get_parameter('robot_prefixes').value
        self.robot_prefixes = robot_prefixes_param if robot_prefixes_param else ['crazyflie']
        
        # Paramètres avec validation de type et valeurs par défaut sécurisées
        self.pheromone_inc = self._get_safe_float(self.get_parameter('pheromone_inc').value, 15)
        self.pheromone_evap = self._get_safe_float(self.get_parameter('pheromone_evap').value, 0.3)
        self.pheromone_max = self._get_safe_float(self.get_parameter('pheromone_max').value, 100)
        self.pheromone_min = self._get_safe_float(self.get_parameter('pheromone_min').value, 0)
        self.consensus_min = self._get_safe_int(self.get_parameter('consensus_min').value, 2)
        self.consensus_tol = self._get_safe_float(self.get_parameter('consensus_tol').value, 0.5)
        enable_morph_val = self.get_parameter('enable_morphology').value
        self.enable_morphology = bool(enable_morph_val) if enable_morph_val is not None else True
        self.min_cluster_size = self._get_safe_int(self.get_parameter('min_cluster_size').value, 4)
        self.smoothing_window = self._get_safe_int(self.get_parameter('smoothing_window').value, 5)
        self.input_topic = self._get_safe_str(self.get_parameter('input_topic').value, '/map')
        self.output_merged = self._get_safe_str(self.get_parameter('output_merged').value, 'map_merged')
        self.output_weighted = self._get_safe_str(self.get_parameter('output_weighted').value, 'map_fusion_weighted')
        self.output_coverage_image = self._get_safe_str(self.get_parameter('output_coverage_image').value, 'coverage_image')
        # Buffers
        self.maps = {}
        self.pheromones = None
        self.history_weighted = deque(maxlen=self.smoothing_window)
        self.width = 0
        self.height = 0
        self.n_cells = 0
        self.bridge = CvBridge()
        self.tick_count = 0  # Pour les logs du zoom adaptatif
        # ROS PUB/SUB
        self.publisher = self.create_publisher(OccupancyGrid, self.output_merged, 10)
        self.weighted_pub = self.create_publisher(OccupancyGrid, self.output_weighted, 10)
        self.coverage_img_pub = self.create_publisher(Image, self.output_coverage_image, 10)
        for prefix in self.robot_prefixes:
            topic = f'/{prefix}{self.input_topic}'
            self.create_subscription(OccupancyGrid, topic, lambda msg, p=prefix: self.map_callback(msg, p), 10)
        self.create_timer(0.5, self.process_maps)  # 2Hz
        
        # --- LOGS DE CONFIGURATION ---
        self.get_logger().info("[MAP_MERGER] Node initialisé (NON permissif, morphologie/filtrage activés)")
        self.get_logger().info(f"[MAP_MERGER] Config YAML: {self.config.config_file or 'défaut'}")
        self.get_logger().info(f"[MAP_MERGER] Drones configurés ({len(self.robot_prefixes)}): {self.robot_prefixes}")
        self.get_logger().info(f"[MAP_MERGER] Topics souscription: {[f'/{p}{self.input_topic}' for p in self.robot_prefixes]}")
        self.get_logger().info(f"[MAP_MERGER] Paramètres fusion: consensus_min={self.consensus_min}, consensus_tol={self.consensus_tol}")
        self.get_logger().info(f"[MAP_MERGER] Morphologie activée: {self.enable_morphology}, min_cluster_size={self.min_cluster_size}")

    # --- HELPERS SÉCURISÉS POUR LA VALIDATION DES PARAMÈTRES ---
    def _get_safe_float(self, value: Any, default: float) -> float:
        """Convertit une valeur en float de manière sécurisée"""
        if value is None:
            return default
        if isinstance(value, (int, float)):
            return float(value)
        if isinstance(value, str):
            try:
                return float(value)
            except ValueError:
                self.get_logger().warn(f"[PARAM] Impossible de convertir '{value}' en float, utilisation de {default}")
                return default
        self.get_logger().warn(f"[PARAM] Type invalide {type(value)} pour float, utilisation de {default}")
        return default

    def _get_safe_int(self, value: Any, default: int) -> int:
        """Convertit une valeur en int de manière sécurisée"""
        if value is None:
            return default
        if isinstance(value, int):
            return value
        if isinstance(value, float):
            return int(value)
        if isinstance(value, str):
            try:
                return int(float(value))  # Permet la conversion "2.0" -> 2
            except ValueError:
                self.get_logger().warn(f"[PARAM] Impossible de convertir '{value}' en int, utilisation de {default}")
                return default
        self.get_logger().warn(f"[PARAM] Type invalide {type(value)} pour int, utilisation de {default}")
        return default

    def _get_safe_str(self, value: Any, default: str) -> str:
        """Convertit une valeur en string de manière sécurisée"""
        if value is None:
            return default
        if isinstance(value, str):
            return value
        if isinstance(value, (int, float, bool)):
            return str(value)
        self.get_logger().warn(f"[PARAM] Type invalide {type(value)} pour string, utilisation de '{default}'")
        return default

    def map_callback(self, msg, prefix):
        self.maps[prefix] = msg
        if self.n_cells == 0:
            self.n_cells = len(msg.data)
            self.width = msg.info.width
            self.height = msg.info.height
            self.pheromones = np.zeros(self.n_cells, dtype=float)
            self.get_logger().info(f"[MAP_MERGER] Dimensions carte: {self.width}x{self.height}")

    def process_maps(self):
        if len(self.maps) < 1 or self.n_cells == 0:
            return
        # --- 1. STIGMERGIE stricte avec vérification None ---
        self._update_pheromones()
        if self.pheromones is not None:
            stigmergy_map = (self.pheromones >= (self.pheromone_max * 0.7)).astype(int) * CellValue.OCCUPIED
        else:
            stigmergy_map = np.zeros(self.n_cells, dtype=int)
        # --- 2. CONSENSUS strict ---
        consensus_map = self._compute_consensus_map(strict=True)
        # --- 3. UNION restrictive ---
        union_map = self._compute_union_map(strict=True)
        # --- 4. CARTE DE COUVERTURE ÉTENDUE ---
        coverage_weight_map = self._compute_coverage_weighted_map()
        
        # --- 5. FUSION PONDÉRÉE HYBRIDE (avec couverture) ---
        # Compromis: plus strict que ultra-permissif mais intègre la couverture
        fusion_weighted = (
            0.35 * stigmergy_map +          # Réduit légèrement pour intégrer couverture
            0.35 * consensus_map +          # Augmenté pour maintenir strictness
            0.2 * union_map +               # Maintenu
            0.1 * coverage_weight_map       # Nouveau: influence de la couverture
        )
        fusion_weighted = np.clip(fusion_weighted, CellValue.UNKNOWN, CellValue.OCCUPIED)
        # --- 6. Morphologie/filtrage ---
        if self.enable_morphology:
            fusion_weighted = self._apply_morphology(fusion_weighted)
        # --- 7. Lissage temporel ---
        self.history_weighted.append(fusion_weighted)
        if len(self.history_weighted) > 0:
            fusion_smoothed = np.mean(self.history_weighted, axis=0)
        else:
            fusion_smoothed = fusion_weighted
        fusion_smoothed = np.round(fusion_smoothed).astype(int)
        # --- 8. Publication ---
        self._publish_occupancy_grid(self.publisher, stigmergy_map)
        self._publish_occupancy_grid(self.weighted_pub, fusion_smoothed)
        self._publish_coverage_image()
        # --- 9. Log ---
        n_occ = np.sum(fusion_smoothed == CellValue.OCCUPIED)
        self.get_logger().info(f"[MAP_MERGER] Fusion tick | OCCUPIED: {n_occ}")
        self.tick_count += 1

    def _update_pheromones(self):
        """Met à jour les phéromones (algorithme original préservé)"""
        if self.pheromones is None:
            return
        
        # Algorithme original des phéromones (pas de modification)
        self.pheromones *= (1.0 - self.pheromone_evap)
        for msg in self.maps.values():
            data = np.array(msg.data)
            self.pheromones[data == CellValue.OCCUPIED] += self.pheromone_inc
        self.pheromones = np.clip(self.pheromones, self.pheromone_min, self.pheromone_max)

    def _compute_consensus_map(self, strict=True):
        votes_occ = np.zeros(self.n_cells)
        for msg in self.maps.values():
            data = np.array(msg.data)
            votes_occ[data == CellValue.OCCUPIED] += 1
        if strict:
            occ_thresh = max(self.consensus_min, int(self.consensus_tol * len(self.maps)))
        else:
            occ_thresh = 1
        consensus_map = (votes_occ >= occ_thresh).astype(int) * CellValue.OCCUPIED
        # Suppression des petits clusters OCCUPIED (anti-faux positifs)
        consensus_map = self._remove_small_clusters(consensus_map)
        return consensus_map

    def _compute_union_map(self, strict=True):
        union = np.zeros(self.n_cells)
        for msg in self.maps.values():
            data = np.array(msg.data)
            union[data == CellValue.OCCUPIED] = 1
        union_map = union.astype(int) * CellValue.OCCUPIED
        if strict:
            union_map = self._remove_small_clusters(union_map)
        return union_map

    def _compute_coverage_weighted_map(self):
        """Calcule une carte pondérée basée sur les phéromones (version restrictive)"""
        if self.pheromones is None:
            return np.zeros(self.n_cells)
        
        # Normaliser les phéromones (0 à 1)
        max_pheromone = np.max(self.pheromones) if np.max(self.pheromones) > 0 else 1
        pheromone_norm = self.pheromones / max_pheromone
        
        # Seuil plus strict pour la version originale: 50% du max (vs 30% modulaire)
        coverage_threshold = 0.5
        coverage_map = (pheromone_norm >= coverage_threshold).astype(int) * CellValue.OCCUPIED
        
        # Appliquer la suppression des petits clusters (cohérent avec l'approche restrictive)
        coverage_map = self._remove_small_clusters(coverage_map)
        
        return coverage_map

    def _remove_small_clusters(self, occ_array: np.ndarray) -> np.ndarray:
        img = occ_array.reshape((self.height, self.width))
        # Utiliser scipy.ndimage.label avec une annotation de type explicite
        binary_img = (img == CellValue.OCCUPIED).astype(np.uint8)
        
        # Appel à scipy.ndimage.label et gestion explicite du type de retour
        label_result = label(binary_img)
        
        # Gestion du type de retour de label (tuple ou array selon version scipy)
        labeled_array: np.ndarray
        num_features: int
        
        if isinstance(label_result, tuple):
            labeled_array, num_features = label_result
        else:
            labeled_array = np.asarray(label_result)  # Conversion explicite en array
            num_features = int(np.max(labeled_array))
        
        if num_features == 0:
            return img.ravel()
        
        # Calculer les tailles des régions connectées
        # labeled_array est maintenant garanti d'être un np.ndarray
        sizes = np.bincount(labeled_array.ravel())
        mask_clean = np.ones_like(img, dtype=bool)
        
        for region_label in range(1, num_features + 1):  # Ignorer le label 0 (background)
            if region_label < len(sizes) and sizes[region_label] < self.min_cluster_size:
                mask_clean[labeled_array == region_label] = False
        
        # Nettoyer l'image
        img[(img == CellValue.OCCUPIED) & (~mask_clean)] = CellValue.UNKNOWN
        return img.ravel()

    def _apply_morphology(self, occ_array):
        img = occ_array.reshape((self.height, self.width))
        occupied_mask = (img == CellValue.OCCUPIED)
        structure = np.ones((3, 3))
        cleaned_mask = binary_opening(occupied_mask, structure=structure, iterations=1)
        if np.sum(cleaned_mask) > 100:
            cleaned_mask = binary_erosion(cleaned_mask, structure=structure, iterations=1)
        result = img.copy()
        n_before = np.sum(occupied_mask)
        n_after = np.sum(cleaned_mask)
        n_removed = n_before - n_after
        if n_removed > 0:
            self.get_logger().info(f"[Morphology] Activé: {n_removed} pixels OCCUPIED supprimés ({n_before}→{n_after})")
        # Utiliser np.logical_not au lieu de ~ pour éviter les problèmes de type
        removed_pixels = occupied_mask & np.logical_not(cleaned_mask)
        result[removed_pixels] = CellValue.UNKNOWN
        return result.ravel()

    def _publish_occupancy_grid(self, publisher, data):
        if self.n_cells == 0 or len(self.maps) == 0:
            return
        msg = OccupancyGrid()
        base = next(iter(self.maps.values()))
        msg.header = base.header
        msg.info = base.info
        msg.data = [int(v) for v in data]
        publisher.publish(msg)

    def _publish_coverage_image(self):
        """Publie une image de couverture avec ZOOM DYNAMIQUE ADAPTATIF (phéromones)"""
        if self.pheromones is None or self.width == 0 or self.height == 0:
            return
        
        # Reshape de la carte de phéromones (ALGORITHME ORIGINAL INCHANGÉ)
        pheromone_img = self.pheromones.reshape((self.height, self.width))
        
        # ZOOM DYNAMIQUE ULTRA AGRESSIF basé sur l'étendue des phéromones
        pheromone_threshold = np.max(pheromone_img) * 0.03 if np.max(pheromone_img) > 0 else 0  # Seuil très bas
        active_zones = np.where(pheromone_img > pheromone_threshold)
        
        # Taille finale ULTRA AGRANDIE (plus grande que modulaire)
        final_width = self.width * 20  # Augmenté de 15 à 20
        final_height = self.height * 20
        
        if len(active_zones[0]) > 0:
            # Calculer la bounding box des zones avec phéromones
            min_y, max_y = np.min(active_zones[0]), np.max(active_zones[0])
            min_x, max_x = np.min(active_zones[1]), np.max(active_zones[1])
            
            # Marge ultra-réduite pour zoom ULTRA agressif
            margin = max(1, min(self.width, self.height) // 60)  # Marge minimale
            min_y = max(0, min_y - margin)
            max_y = min(self.height - 1, max_y + margin)
            min_x = max(0, min_x - margin)
            max_x = min(self.width - 1, max_x + margin)
            
            # Calculer le ratio de couverture pour ajuster le zoom
            covered_area = (max_y - min_y + 1) * (max_x - min_x + 1)
            total_area = self.height * self.width
            coverage_ratio = covered_area / total_area
            
            # Zoom MÉGA ULTRA AGRESSIF avec seuils encore plus étendus
            if coverage_ratio < 0.4:  # MÉGA ZOOM étendu - zoom maximal sur très large plage
                crop_img = pheromone_img[min_y:max_y+1, min_x:max_x+1]
                if coverage_ratio < 0.02:
                    zoom_info = "MÉGA ZOOM ABSOLU (zone microscopique)"
                elif coverage_ratio < 0.08:
                    zoom_info = "ULTRA ZOOM MAX (zone minuscule)"
                elif coverage_ratio < 0.2:
                    zoom_info = "ZOOM MAXIMAL (début stigmergie)"
                else:
                    zoom_info = "ZOOM TRÈS FORT (stigmergie active)"
            elif coverage_ratio < 0.55:  # ZOOM FORT - stigmergie moyenne
                # Élargir très légèrement
                expand_y = int((self.height - (max_y - min_y)) * 0.08)  # Expansion encore plus minimale
                expand_x = int((self.width - (max_x - min_x)) * 0.08)
                min_y = max(0, min_y - expand_y)
                max_y = min(self.height - 1, max_y + expand_y)
                min_x = max(0, min_x - expand_x)
                max_x = min(self.width - 1, max_x + expand_x)
                crop_img = pheromone_img[min_y:max_y+1, min_x:max_x+1]
                zoom_info = "ZOOM FORT (stigmergie moyenne)"
            elif coverage_ratio < 0.75:  # ZOOM MOYEN - stigmergie étendue
                expand_y = int((self.height - (max_y - min_y)) * 0.15)
                expand_x = int((self.width - (max_x - min_x)) * 0.15)
                min_y = max(0, min_y - expand_y)
                max_y = min(self.height - 1, max_y + expand_y)
                min_x = max(0, min_x - expand_x)
                max_x = min(self.width - 1, max_x + expand_x)
                crop_img = pheromone_img[min_y:max_y+1, min_x:max_x+1]
                zoom_info = "ZOOM MOYEN (stigmergie étendue)"
            else:  # Vue d'ensemble seulement si stigmergie quasi-complète
                crop_img = pheromone_img
                zoom_info = "VUE ENSEMBLE (stigmergie quasi-complète)"
        else:
            # Aucune phéromone - zoom sur le centre
            center_y, center_x = self.height // 2, self.width // 2
            crop_size = min(self.width, self.height) // 8  # Crop très petit au centre
            min_y = max(0, center_y - crop_size)
            max_y = min(self.height - 1, center_y + crop_size)
            min_x = max(0, center_x - crop_size)
            max_x = min(self.width - 1, center_x + crop_size)
            crop_img = pheromone_img[min_y:max_y+1, min_x:max_x+1]
            zoom_info = "ZOOM CENTRE (aucune phéromone)"
        
        # Redimensionner le crop vers la taille finale
        pheromone_stretched = cv2.resize(crop_img, (final_width, final_height), 
                                       interpolation=cv2.INTER_CUBIC)
        
        # VISUALISATION: Normaliser sur la grande image
        if np.max(pheromone_stretched) > 0:
            obs_normalized = (pheromone_stretched / np.max(pheromone_stretched) * 180).astype(np.uint8)
        else:
            obs_normalized = np.zeros_like(pheromone_stretched, dtype=np.uint8)
        
        # REMPLIR avec fond orange (évite le violet dans PLASMA)
        background_value = 180  # Orange dans PLASMA
        full_coverage = np.where(obs_normalized == 0, background_value, 
                                np.clip(background_value + (obs_normalized * 0.4), background_value, 255).astype(np.uint8))
        
        # Appliquer la colormap PLASMA
        color_img = cv2.applyColorMap(full_coverage, cv2.COLORMAP_PLASMA)
        
        # Post-traitement pour améliorer la saturation
        color_img = cv2.convertScaleAbs(color_img, alpha=1.2, beta=20)
        
        # Log du zoom adaptatif (occasionnel)
        if hasattr(self, 'tick_count') and self.tick_count % 50 == 0:
            coverage_pct = len(active_zones[0]) / (self.width * self.height) * 100 if len(active_zones[0]) > 0 else 0
            self.get_logger().info(f"[ZOOM ADAPTATIF] {zoom_info} - Phéromones: {coverage_pct:.1f}%")
        
        # Créer le message ROS
        ros_img = self.bridge.cv2_to_imgmsg(color_img, encoding='bgr8')
        
        # Vérification sécurisée que nous avons au moins une carte
        if len(self.maps) > 0:
            base = next(iter(self.maps.values()))
            ros_img.header = base.header
        else:
            # Header par défaut si aucune carte disponible
            ros_img.header.stamp = self.get_clock().now().to_msg()
            ros_img.header.frame_id = "map"
        
        ros_img.width = final_width
        ros_img.height = final_height
        
        self.coverage_img_pub.publish(ros_img)

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
