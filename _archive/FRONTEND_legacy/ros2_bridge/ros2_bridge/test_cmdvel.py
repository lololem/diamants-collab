#!/usr/bin/env python3
"""
Script de test - Commandes cmd_vel pour DIAMANTS
===============================================
"""

import time
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelTester(Node):
    def __init__(self):
        super().__init__('cmdvel_tester')
        
        # Publisher cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/crazyflie/cmd_vel', 10)
        
        self.get_logger().info("🚁 Testeur cmd_vel démarré")
    
    def test_takeoff(self):
        """Test décollage"""
        self.get_logger().info("🛫 Test décollage...")
        
        cmd = Twist()
        cmd.linear.z = 0.5  # Montée
        
        for i in range(10):
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        # Arrêt
        cmd.linear.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def test_movement(self):
        """Test mouvement horizontal"""
        self.get_logger().info("🔄 Test mouvement...")
        
        cmd = Twist()
        
        # Avancer
        cmd.linear.x = 0.3
        for i in range(20):
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        # Tourner
        cmd.linear.x = 0.0
        cmd.angular.z = 0.5
        for i in range(20):
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
        
        # Arrêt
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
    
    def test_hover(self):
        """Test vol stationnaire avec variations"""
        self.get_logger().info("🌪️ Test vol stationnaire...")
        
        cmd = Twist()
        
        for cycle in range(5):
            # Petites variations
            cmd.linear.x = 0.1 * (1 if cycle % 2 == 0 else -1)
            cmd.linear.y = 0.1 * (1 if cycle % 3 == 0 else -1)
            cmd.angular.z = 0.2 * (1 if cycle % 2 == 0 else -1)
            
            for i in range(15):
                self.cmd_vel_pub.publish(cmd)
                time.sleep(0.1)
        
        # Arrêt complet
        cmd = Twist()  # Tout à zéro
        self.cmd_vel_pub.publish(cmd)


def main():
    rclpy.init()
    
    try:
        tester = CmdVelTester()
        
        # Séquence de tests
        tester.get_logger().info("🎮 Démarrage séquence de tests...")
        
        time.sleep(2)
        tester.test_takeoff()
        
        time.sleep(2) 
        tester.test_movement()
        
        time.sleep(2)
        tester.test_hover()
        
        tester.get_logger().info("✅ Tests terminés")
        
    except KeyboardInterrupt:
        print("\n🛑 Tests arrêtés")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
