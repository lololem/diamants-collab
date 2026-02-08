#!/usr/bin/env python3
"""
DIAMANTS - SLAM Fusion Microservice
=====================================
Collaborative SLAM map merging from multi-drone lidar data.

Subscribes to /{cfN}/scan lidar scans and each drone's odometry,
builds per-drone occupancy grids, and merges them using a
stigmergy + consensus hybrid approach.

Publishes:
    /diamants/slam/map       - Merged OccupancyGrid (for RViz / advanced consumers)
    /diamants/slam/map_json  - JSON representation for the frontend
    /diamants/slam/coverage  - Coverage statistics

Based on slam_map_merge/map_merger_node.py (stigmergy + consensus + union)
"""

import json
import time
import math
import threading
from typing import Dict, List, Tuple, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose, Point, Quaternion


class DroneMap:
    """Per-drone local occupancy grid."""

    def __init__(self, drone_id: str, resolution: float, size: int):
        self.drone_id = drone_id
        self.resolution = resolution
        self.size = size
        self.origin_x = -size * resolution / 2.0
        self.origin_y = -size * resolution / 2.0

        # -1 = unknown, 0 = free, 100 = occupied
        self.grid = np.full((size, size), -1, dtype=np.int8)

        # Pheromone layer for stigmergy
        self.pheromone = np.zeros((size, size), dtype=np.float32)

        # Drone pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_update = 0.0

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        gx = int((wx - self.origin_x) / self.resolution)
        gy = int((wy - self.origin_y) / self.resolution)
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.size and 0 <= gy < self.size

    def update_from_scan(self, scan: LaserScan):
        """Ray-cast lidar scan into occupancy grid."""
        if self.last_update == 0.0:
            return

        gx0, gy0 = self.world_to_grid(self.x, self.y)
        if not self.in_bounds(gx0, gy0):
            return

        # Mark drone position as free
        self.grid[gy0, gx0] = 0
        self.pheromone[gy0, gx0] = min(100.0, self.pheromone[gy0, gx0] + 5.0)

        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment + self.yaw

            # Ray-cast: mark free cells along the ray
            steps = int(r / self.resolution)
            for step in range(steps):
                frac = step * self.resolution
                wx = self.x + frac * math.cos(angle)
                wy = self.y + frac * math.sin(angle)
                gx, gy = self.world_to_grid(wx, wy)
                if self.in_bounds(gx, gy):
                    self.grid[gy, gx] = 0  # Free

            # Mark endpoint as occupied
            hit_x = self.x + r * math.cos(angle)
            hit_y = self.y + r * math.sin(angle)
            hgx, hgy = self.world_to_grid(hit_x, hit_y)
            if self.in_bounds(hgx, hgy):
                self.grid[hgy, hgx] = 100  # Occupied


class SLAMFusion(Node):
    """
    Collaborative SLAM fusion node.

    Merges per-drone occupancy grids using a hybrid approach:
    - Stigmergy: pheromone accumulation at visited cells
    - Consensus: voting among drone maps
    - Union: simple union of all observations

    Weights: 0.35×stigmergy + 0.35×consensus + 0.2×union + 0.1×coverage
    """

    DRONE_NAMES = ["crazyflie", "crazyflie1", "crazyflie2",
                   "crazyflie3", "crazyflie4", "crazyflie5",
                   "crazyflie6", "crazyflie7"]

    def __init__(self):
        super().__init__("diamants_slam_fusion")

        # Parameters
        self.declare_parameter("map_resolution", 0.1)     # meters per cell
        self.declare_parameter("map_size", 200)            # cells (200×200 = 20m×20m)
        self.declare_parameter("fusion_rate_hz", 2.0)      # Merge rate
        self.declare_parameter("publish_rate_hz", 1.0)     # JSON publish rate
        self.declare_parameter("pheromone_evap", 0.98)     # Evaporation per cycle
        self.declare_parameter("num_drones", 8)

        self.resolution = self.get_parameter("map_resolution").value
        self.map_size = self.get_parameter("map_size").value
        self.fusion_rate = self.get_parameter("fusion_rate_hz").value
        self.publish_rate = self.get_parameter("publish_rate_hz").value
        self.pheromone_evap = self.get_parameter("pheromone_evap").value
        num_drones = self.get_parameter("num_drones").value

        # Per-drone maps
        self.drone_maps: Dict[str, DroneMap] = {}
        for name in self.DRONE_NAMES[:num_drones]:
            self.drone_maps[name] = DroneMap(name, self.resolution, self.map_size)

        # Merged map
        self.merged_grid = np.full((self.map_size, self.map_size), -1, dtype=np.int8)
        self.merged_pheromone = np.zeros((self.map_size, self.map_size), dtype=np.float32)

        # Lock for thread safety
        self._lock = threading.Lock()

        # --- ROS2 Subscribers ---
        for name in self.drone_maps:
            self.create_subscription(
                LaserScan, f"/{name}/scan",
                lambda msg, n=name: self._on_scan(n, msg), 10
            )
            self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda msg, n=name: self._on_odom(n, msg), 10
            )

        # --- ROS2 Publishers ---
        self._pub_map = self.create_publisher(
            OccupancyGrid, "/diamants/slam/map", 10
        )
        self._pub_map_json = self.create_publisher(
            String, "/diamants/slam/map_json", 10
        )
        self._pub_coverage = self.create_publisher(
            Float32, "/diamants/slam/coverage", 10
        )

        # --- Timers ---
        self.create_timer(1.0 / self.fusion_rate, self._fuse_maps)
        self.create_timer(1.0 / self.publish_rate, self._publish_map_json)

        self.get_logger().info(
            f"SLAMFusion started — {num_drones} drones, "
            f"grid={self.map_size}×{self.map_size} @ {self.resolution}m/cell, "
            f"fusion@{self.fusion_rate}Hz"
        )

    # =========================================================================
    # Callbacks
    # =========================================================================

    def _on_odom(self, drone_id: str, msg: Odometry):
        dm = self.drone_maps.get(drone_id)
        if dm:
            dm.x = msg.pose.pose.position.x
            dm.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            dm.yaw = math.atan2(siny, cosy)
            dm.last_update = time.time()

    def _on_scan(self, drone_id: str, msg: LaserScan):
        dm = self.drone_maps.get(drone_id)
        if dm:
            with self._lock:
                dm.update_from_scan(msg)

    # =========================================================================
    # Map Fusion
    # =========================================================================

    def _fuse_maps(self):
        """
        Hybrid fusion: stigmergy + consensus + union.
        Based on slam_map_merge/map_merger_node.py algorithm.
        """
        with self._lock:
            s = self.map_size
            merged = np.full((s, s), -1, dtype=np.int8)

            # --- Stigmergy layer: average pheromone ---
            total_pheromone = np.zeros((s, s), dtype=np.float32)
            active_maps = 0
            for dm in self.drone_maps.values():
                if dm.last_update > 0:
                    total_pheromone += dm.pheromone
                    dm.pheromone *= self.pheromone_evap  # Evaporation
                    active_maps += 1

            if active_maps == 0:
                return

            # Normalize pheromone
            max_pher = total_pheromone.max()
            if max_pher > 0:
                stigmergy = (total_pheromone / max_pher * 100).astype(np.int8)
            else:
                stigmergy = np.zeros((s, s), dtype=np.int8)

            # --- Consensus layer: voting ---
            votes_free = np.zeros((s, s), dtype=np.int32)
            votes_occ = np.zeros((s, s), dtype=np.int32)
            votes_total = np.zeros((s, s), dtype=np.int32)

            for dm in self.drone_maps.values():
                if dm.last_update == 0:
                    continue
                free_mask = dm.grid == 0
                occ_mask = dm.grid == 100
                known_mask = dm.grid >= 0

                votes_free += free_mask.astype(np.int32)
                votes_occ += occ_mask.astype(np.int32)
                votes_total += known_mask.astype(np.int32)

            consensus = np.full((s, s), -1, dtype=np.int8)
            # Need at least 1 vote
            known = votes_total > 0
            # Occupied if majority say occupied
            occ_consensus = (votes_occ > votes_free) & known
            free_consensus = ~occ_consensus & known
            consensus[free_consensus] = 0
            consensus[occ_consensus] = 100

            # --- Union layer: any observation wins ---
            union = np.full((s, s), -1, dtype=np.int8)
            for dm in self.drone_maps.values():
                if dm.last_update == 0:
                    continue
                known_mask = dm.grid >= 0
                # Occupied cells always win
                occ_mask = dm.grid == 100
                union[occ_mask] = 100
                # Free only if not already occupied
                free_mask = (dm.grid == 0) & (union != 100)
                union[free_mask] = 0

            # --- Coverage layer ---
            coverage = np.zeros((s, s), dtype=np.int8)
            coverage[votes_total > 0] = np.clip(
                votes_total[votes_total > 0] * 25, 0, 100
            ).astype(np.int8)

            # --- Hybrid merge: weighted combination ---
            # Only for known cells
            any_known = (stigmergy > 0) | (consensus >= 0) | (union >= 0)

            stig_f = np.where(stigmergy > 0, stigmergy.astype(np.float32), 0.0)
            cons_f = np.where(consensus >= 0, consensus.astype(np.float32), 0.0)
            uni_f = np.where(union >= 0, union.astype(np.float32), 0.0)
            cov_f = coverage.astype(np.float32)

            hybrid = 0.35 * stig_f + 0.35 * cons_f + 0.2 * uni_f + 0.1 * cov_f

            merged[any_known] = np.clip(hybrid[any_known], 0, 100).astype(np.int8)

            self.merged_grid = merged
            self.merged_pheromone = total_pheromone

        # Publish OccupancyGrid (for RViz)
        self._publish_occupancy_grid()

        # Publish coverage metric
        total_cells = self.map_size * self.map_size
        known_cells = int(np.sum(self.merged_grid >= 0))
        coverage_pct = known_cells / total_cells * 100.0

        cov_msg = Float32()
        cov_msg.data = coverage_pct
        self._pub_coverage.publish(cov_msg)

    def _publish_occupancy_grid(self):
        """Publish merged map as ROS2 OccupancyGrid."""
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info = MapMetaData()
        msg.info.resolution = self.resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin = Pose(
            position=Point(
                x=-self.map_size * self.resolution / 2.0,
                y=-self.map_size * self.resolution / 2.0,
                z=0.0,
            ),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )
        msg.data = self.merged_grid.flatten().tolist()
        self._pub_map.publish(msg)

    def _publish_map_json(self):
        """
        Publish a lightweight JSON summary for frontend consumption.
        Only includes occupied and free cell coordinates (sparse format).
        """
        occupied = []
        free_boundary = []

        # Downsample for frontend (every 5th cell)
        step = 5
        for gy in range(0, self.map_size, step):
            for gx in range(0, self.map_size, step):
                val = self.merged_grid[gy, gx]
                wx = (gx * self.resolution) + (-self.map_size * self.resolution / 2.0)
                wy = (gy * self.resolution) + (-self.map_size * self.resolution / 2.0)

                if val >= 70:  # Occupied
                    occupied.append([round(wx, 2), round(wy, 2)])
                elif val == 0:
                    # Only publish boundary free cells (adjacent to unknown)
                    is_boundary = False
                    for dy, dx in [(-step, 0), (step, 0), (0, -step), (0, step)]:
                        ny, nx = gy + dy, gx + dx
                        if 0 <= ny < self.map_size and 0 <= nx < self.map_size:
                            if self.merged_grid[ny, nx] < 0:
                                is_boundary = True
                                break
                    if is_boundary:
                        free_boundary.append([round(wx, 2), round(wy, 2)])

        # Drone positions on map
        drone_positions = {}
        for name, dm in self.drone_maps.items():
            if dm.last_update > 0:
                drone_positions[name] = {
                    "x": round(dm.x, 3),
                    "y": round(dm.y, 3),
                    "yaw": round(dm.yaw, 3),
                }

        map_json = {
            "resolution": self.resolution,
            "size": self.map_size,
            "occupied": occupied,
            "frontier": free_boundary,
            "drone_positions": drone_positions,
            "total_known_cells": int(np.sum(self.merged_grid >= 0)),
            "total_cells": self.map_size * self.map_size,
            "timestamp": time.time(),
        }

        msg = String()
        msg.data = json.dumps(map_json)
        self._pub_map_json.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SLAMFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("SLAMFusion shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
