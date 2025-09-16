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
Mission de test DIAMANTS V3 - Visualisation Web
==============================================
Crée des données de test pour valider l'interface web
"""

import rclpy
from rclpy.node import Node
import json
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MissionTestWeb(Node):
    """Mission de test pour l'interface web"""
    
    def __init__(self):
        super().__init__('mission_test_web')
        
        # Publishers
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        self.status_pub = self.create_publisher(String, '/swarm_status', 10)
        
        # Timer pour publier des données test
        self.timer = self.create_timer(1.0, self.publish_test_data)
        
        # État simulation
        self.time_start = time.time()
        self.drone_count = 3
        
        self.get_logger().info("🎯 Mission test web démarrée")
    
    def publish_test_data(self):
        """Publier données de test"""
        current_time = time.time() - self.time_start
        
        # 1. Publier transformations des drones
        self.publish_drone_transforms(current_time)
        
        # 2. Publier carte SLAM simulée
        self.publish_map_data()
        
        # 3. Publier status essaim
        self.publish_swarm_status(current_time)
    
    def publish_drone_transforms(self, t):
        """Publier positions des drones"""
        tf_msg = TFMessage()
        
        for i in range(self.drone_count):
            # Mouvement circulaire pour chaque drone
            radius = 2.0 + i * 0.5
            angle = t * 0.5 + i * (2 * math.pi / self.drone_count)
            
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = f"drone_{i+1}"
            
            # Position circulaire
            transform.transform.translation.x = radius * math.cos(angle)
            transform.transform.translation.y = radius * math.sin(angle)
            transform.transform.translation.z = 1.0 + 0.5 * math.sin(t + i)
            
            # Orientation
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(angle * 0.5)
            transform.transform.rotation.w = math.cos(angle * 0.5)
            
            tf_msg.transforms.append(transform)
        
        self.tf_pub.publish(tf_msg)
    
    def publish_map_data(self):
        """Publier carte SLAM simulée"""
        map_msg = OccupancyGrid()
        
        # Métadonnées carte
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        
        map_msg.info.resolution = 0.1  # 10cm par pixel
        map_msg.info.width = 100
        map_msg.info.height = 100
        map_msg.info.origin.position.x = -5.0
        map_msg.info.origin.position.y = -5.0
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Données carte (simple grille avec obstacles)
        data = np.zeros((100, 100), dtype=np.int8)
        
        # Ajouter quelques obstacles
        data[20:30, 20:30] = 100  # Obstacle
        data[70:80, 70:80] = 100  # Obstacle
        data[10:15, 50:90] = 100  # Mur
        
        # Zone explorée (gris)
        for i in range(100):
            for j in range(100):
                if data[i, j] == 0:
                    # Gradient d'exploration
                    dist_center = math.sqrt((i-50)**2 + (j-50)**2)
                    if dist_center < 40:
                        data[i, j] = -1  # Espace libre
        
        map_msg.data = data.flatten().tolist()
        self.map_pub.publish(map_msg)
    
    def publish_swarm_status(self, t):
        """Publier status de l'essaim"""
        status = {
            "intelligence_score": 0.8 + 0.1 * math.sin(t * 0.3),
            "coverage_area": min(0.9, 0.3 + t * 0.01),
            "mission_status": "exploration" if t < 30 else "mapping",
            "active_drones": self.drone_count,
            "battery_avg": 85 - min(15, t * 0.1),
            "timestamp": time.time()
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    node = MissionTestWeb()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Arrêt mission test")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()