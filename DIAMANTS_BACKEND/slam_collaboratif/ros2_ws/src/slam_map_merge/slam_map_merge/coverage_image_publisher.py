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

# Suppression complète du fichier coverage_image_publisher.py (plus de publication d'image de couverture)
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import numpy as np
import cv2

class CoverageImagePublisher(Node):
    def __init__(self):
        super().__init__('coverage_image_publisher')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map_coverage',
            self.coverage_callback,
            10)
        self.publisher = self.create_publisher(Image, '/map_coverage_image', 10)

    def coverage_callback(self, msg):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self.get_logger().info(f"/map_coverage: shape=({h},{w}), min={data.min()}, max={data.max()}, unique={np.unique(data, return_counts=True)}")
        # -1 = transparent, 0 = rien, 10...100 = dégradé de vert
        img = np.zeros((h, w, 4), dtype=np.uint8)
        mask = data >= 0  # tout ce qui est observé
        # Dégradé de vert selon la valeur (0 à 100 → 0 à 255)
        img[..., 1][mask] = np.clip(data[mask]*2.55, 0, 255).astype(np.uint8)  # vert
        img[..., 3][mask] = 200  # alpha
        img[..., 3][data == -1] = 0  # transparent
        self.get_logger().info(f"Image RGBA: min={img.min()}, max={img.max()}, shape={img.shape}")
        # Conversion en Image ROS
        image_msg = Image()
        image_msg.header = msg.header
        image_msg.height = h
        image_msg.width = w
        image_msg.encoding = 'rgba8'
        image_msg.is_bigendian = False
        image_msg.step = 4 * w
        image_msg.data = img.tobytes()
        self.publisher.publish(image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoverageImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
