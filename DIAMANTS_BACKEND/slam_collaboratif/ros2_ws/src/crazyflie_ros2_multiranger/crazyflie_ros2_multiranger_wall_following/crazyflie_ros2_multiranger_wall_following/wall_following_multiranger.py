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
# pylint: disable-all
# flake8: noqa
# type: ignore

"""
This simple mapper is loosely based on both the bitcraze cflib
point cloud example and the webots epuck simple mapper example.

Original sources:
- bitcraze/crazyflie-lib-python multiranger example
- webots_ros2 epuck mapper
- knmcguire/crazyflie_ros2_experimental
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from std_msgs.msg import String

import tf_transformations
import math
import time
from .wall_following.wall_following import WallFollowing
from .advanced_collision_manager import CollisionController

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1


class WallFollowingMultiranger(Node):
    def __init__(self):

        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = str(self.get_parameter('robot_prefix').value)  # type: ignore
        self.declare_parameter('delay', 5.0)
        self.delay = float(self.get_parameter('delay').value)  # type: ignore
        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = float(self.get_parameter('max_turn_rate').value)  # type: ignore
        self.declare_parameter('max_forward_speed', 0.5)
        max_forward_speed = float(self.get_parameter('max_forward_speed').value)  # type: ignore
        self.declare_parameter('wall_following_direction', 'right')
        self.wall_following_direction = str(self.get_parameter('wall_following_direction').value)  # type: ignore

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10
        )
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10
        )

        # add service to stop wall following and make the crazyflie land
        self.srv = self.create_service(Trigger, robot_prefix + '/stop_wall_following', self.stop_wall_following_cb)

        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        # Initialize ranges with safe default values (inf = no obstacles)
        self.ranges = [float('inf'), float('inf'), float('inf'), float('inf')]
        # Track if we've received real sensor data
        self.ranges_received = False

        self.position_update = False

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f"Wall following set for crazyflie {robot_prefix} "
            f"using the scan topic with a delay of {self.delay} seconds"
        )

        # Create a timer to run the wall following state machine
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Initialize wall following state machine
        self.wall_following = WallFollowing(
            max_turn_rate=max_turn_rate,
            max_forward_speed=max_forward_speed,
            init_state=WallFollowing.StateWallFollowing.FORWARD,
        )

        # Initialize advanced collision management system
        drone_id = robot_prefix.replace('/', '')
        self.collision_controller = CollisionController(drone_id=drone_id)
        self.get_logger().info(f"Advanced collision manager initialized for {robot_prefix}")

        # === COMMUNICATION INTER-DRONES POUR ÉVITEMENT DE COLLISIONS ===
        self.robot_prefix = robot_prefix.strip('/')
        # Dict pour stocker les positions des autres drones
        self.other_drones = {}
        self.velocity = [0.0, 0.0, 0.0]  # Notre vitesse actuelle

        # Publisher pour diffuser notre position aux autres drones
        self.position_publisher = self.create_publisher(String, '/swarm/drone_positions', 10)

        # Subscriber pour recevoir les positions des autres drones
        self.position_subscriber = self.create_subscription(
            String, '/swarm/drone_positions', self.other_drone_position_callback, 10
        )

        # Timer pour publier notre position régulièrement
        # 10Hz publication rate
        self.position_timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info(f"Inter-drone communication setup for {robot_prefix}")
        self.get_logger().info("Collision avoidance between drones is ACTIVE")

        # Give a take off command but wait for the delay to start
        # the wall following
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()
        msg.linear.z = 0.5
        self.twist_publisher.publish(msg)

    def stop_wall_following_cb(self, request, response):
        self.get_logger().info('Stopping wall following')
        self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = -0.2
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        response.success = True

        return response

    def publish_position(self):
        """Publie notre position actuelle pour les autres drones."""
        if not self.position_update:
            return

        # Format: drone_id:x:y:z:vx:vy:vz:timestamp
        pos_str = f"{self.position[0]:.3f}:{self.position[1]:.3f}:" f"{self.position[2]:.3f}"
        vel_str = f"{self.velocity[0]:.3f}:{self.velocity[1]:.3f}:" f"{self.velocity[2]:.3f}"
        message = f"{self.robot_prefix}:{pos_str}:{vel_str}:" f"{time.time():.3f}"

        msg = String()
        msg.data = message
        self.position_publisher.publish(msg)

    def other_drone_position_callback(self, msg):
        """Reçoit et traite les positions des autres drones."""
        try:
            # Parse le message: drone_id:x:y:z:vx:vy:vz:timestamp
            parts = msg.data.split(':')
            if len(parts) >= 8:
                drone_id = parts[0]

                # Ignorer notre propre message
                if drone_id == self.robot_prefix:
                    return

                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                vx, vy, vz = float(parts[4]), float(parts[5]), float(parts[6])
                timestamp = float(parts[7])

                # Stocker la position de l'autre drone
                self.other_drones[drone_id] = {'position': [x, y, z], 'velocity': [vx, vy, vz], 'timestamp': timestamp}

                # Mettre à jour le système de collision avec cette menace
                self.collision_controller.collision_manager.track_threat(
                    drone_id=drone_id, position=[x, y, z], velocity=[vx, vy, vz], own_position=self.position
                )

                # Log périodique pour debug (pas trop fréquent)
                current_time = time.time()
                if hasattr(self, '_last_debug_log'):
                    # Toutes les 2 secondes
                    if current_time - self._last_debug_log > 2.0:
                        log_msg = (
                            f"[INTER_DRONE] Tracking {drone_id} at "
                            f"pos=({x:.2f},{y:.2f},{z:.2f}), "
                            f"vel=({vx:.2f},{vy:.2f},{vz:.2f})"
                        )
                        self.get_logger().info(log_msg)
                        self._last_debug_log = current_time
                else:
                    self._last_debug_log = current_time
                    contact_msg = f"[INTER_DRONE] First contact with {drone_id}"
                    self.get_logger().info(contact_msg)

        except (ValueError, IndexError) as e:
            error_msg = f"[INTER_DRONE] EXCEPTION: Failed to parse drone position message: {msg.data}"
            self.get_logger().error(error_msg)
            self.get_logger().error(f"[INTER_DRONE] Exception details: {type(e).__name__}: {str(e)}")
            self.get_logger().error(f"[INTER_DRONE] Message parts received: {len(msg.data.split(':')) if msg.data else 0} parts")
        except Exception as e:
            fatal_msg = f"[INTER_DRONE] FATAL: Unexpected exception in position callback"
            self.get_logger().fatal(fatal_msg)
            self.get_logger().fatal(f"[INTER_DRONE] Fatal exception: {type(e).__name__}: {str(e)}")
            import traceback
            self.get_logger().fatal(f"[INTER_DRONE] Traceback: {traceback.format_exc()}")

    def timer_callback(self):
        try:
            # wait for the delay to pass and then start wall following
            if self.wait_for_start:
                current_time = self.get_clock().now().nanoseconds * 1e-9
                if current_time - self.start_clock > self.delay:
                    self.get_logger().info('Starting wall following')
                    self.wait_for_start = False
                else:
                    return

            # initialize variables
            velocity_x = 0.0
            velocity_y = 0.0
            yaw_rate = 0.0
            state_wf = WallFollowing.StateWallFollowing.HOVER

            # Get Yaw
            actual_yaw_rad = self.angles[2]

            # get front and side range in meters
            right_range = self.ranges[1]
            front_range = self.ranges[2]
            left_range = self.ranges[3]

            # Debug: ranges info
            # self.get_logger().info(f"Front: {front_range}, Right: {right_range},
            # Left: {left_range}")

            # choose here the direction that you want the wall following to turn to
            if self.wall_following_direction == 'right':
                wf_dir = WallFollowing.WallFollowingDirection.RIGHT
                side_range = left_range
            else:
                wf_dir = WallFollowing.WallFollowingDirection.LEFT
                side_range = right_range

            time_now = self.get_clock().now().nanoseconds * 1e-9

            # get velocity commands and current state from wall following
            # state machine
            if side_range > 0.1:
                result = self.wall_following.wall_follower(front_range, side_range, actual_yaw_rad, wf_dir, time_now)
                velocity_x, velocity_y, yaw_rate, state_wf = result

            # === CALCUL ET STOCKAGE DE LA VITESSE POUR COMMUNICATION ===
            # Calculer notre vitesse actuelle
            # (approximation par différences finies)
            current_time = time.time()
            if hasattr(self, '_last_position') and hasattr(self, '_last_time'):
                dt = current_time - self._last_time
                if dt > 0.001:  # Éviter division par zéro
                    dx = self.position[0] - self._last_position[0]
                    self.velocity[0] = dx / dt
                    dy = self.position[1] - self._last_position[1]
                    self.velocity[1] = dy / dt
                    dz = self.position[2] - self._last_position[2]
                    self.velocity[2] = dz / dt

            self._last_position = self.position.copy()
            self._last_time = current_time

            # Apply advanced collision avoidance
            # Check for emergency stop first (obstacles statiques)
            # Only apply emergency stop if we have received real sensor data
            emergency_threshold = 0.15  # 15cm emergency threshold
            emergency_stop = False
            if self.ranges_received:
                emergency_stop = self.collision_controller.emergency_stop(self.ranges, emergency_threshold)
            else:
                # Log once that we're waiting for sensor data
                if not hasattr(self, '_logged_no_sensors'):
                    self.get_logger().info("Waiting for sensor data... Emergency stop disabled.")
                    self._logged_no_sensors = True

            if emergency_stop:
                warn_msg = "[COLLISION] Emergency stop activated - " "static obstacle too close!"
                self.get_logger().warn(warn_msg)
                velocity_x = 0.0
                velocity_y = 0.0
                yaw_rate = 0.0
            else:
                # === 1. ÉVITEMENT D'OBSTACLES STATIQUES ===
                avoidance_command = self.collision_controller.compute_avoidance(self.position, self.ranges)

                # === 2. ÉVITEMENT ENTRE DRONES ===
                inter_drone_avoidance = [0.0, 0.0, 0.0]
                active_threats = []

                # Nettoyer les anciennes positions de drones (plus de 2 secondes)
                current_time = time.time()
                expired_drones = []
                for drone_id, drone_data in self.other_drones.items():
                    if current_time - drone_data['timestamp'] > 2.0:
                        expired_drones.append(drone_id)

                for drone_id in expired_drones:
                    del self.other_drones[drone_id]
                    debug_msg = f"[INTER_DRONE] Expired data for {drone_id}"
                    self.get_logger().debug(debug_msg)

                # Calculer l'évitement pour chaque drone actif
                for drone_id, drone_data in self.other_drones.items():
                    # Calculer la distance
                    dx = drone_data['position'][0] - self.position[0]
                    dy = drone_data['position'][1] - self.position[1]
                    dz = drone_data['position'][2] - self.position[2]
                    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

                # Détecter si collision imminente
                from .advanced_collision_manager import Threat

                threat = Threat(
                    position=drone_data['position'],
                    velocity=drone_data['velocity'],
                    timestamp=drone_data['timestamp'],
                    drone_id=drone_id,
                )

                collision_zone = self.collision_controller.collision_manager.detect_collision(
                    threat, self.position, self.velocity
                )

                if collision_zone:
                    active_threats.append(drone_id)
                    # Appliquer le vecteur d'évitement
                    inter_drone_avoidance[0] += collision_zone.vector[0]
                    inter_drone_avoidance[1] += collision_zone.vector[1]
                    inter_drone_avoidance[2] += collision_zone.vector[2]

                    self.get_logger().warn(
                        f"[COLLISION_DRONE] Avoiding {drone_id}: risk={collision_zone.level.value}, "
                        f"distance={distance:.2f}m, avoidance=({collision_zone.vector[0]:.2f}, {collision_zone.vector[1]:.2f})"
                    )
                elif distance < 3.0:  # Log pour drones proches mais sans collision
                    msg = f"[INTER_DRONE] {drone_id} nearby at " f"{distance:.2f}m (safe)"
                    self.get_logger().debug(msg)

            # === 3. FUSION DES COMMANDES D'ÉVITEMENT ===
            # Pondération entre wall-following,
            # évitement statique et évitement drones
            static_weight = 0.4  # Évitement obstacles statiques
            drone_weight = 0.8  # Évitement entre drones (priorité élevée)
            # Ce qui reste pour wall-following
            wall_weight = 1.0 - min(static_weight, drone_weight)

            # Ajuster les pondérations si évitement de drones actif
            if active_threats:
                wall_weight = 0.2  # Réduire fortement le wall-following
                static_weight = 0.3  # Réduire légèrement l'évitement statique
                drone_weight = 0.8  # Priorité maximale aux drones

                self.get_logger().warn(f"[COLLISION_ACTIVE] Avoiding {len(active_threats)} drone(s): {active_threats}")

            # Appliquer les corrections pondérées
            final_vx = (
                velocity_x * wall_weight
                + avoidance_command[0] * static_weight
                + inter_drone_avoidance[0] * drone_weight
            )
            final_vy = (
                velocity_y * wall_weight
                + avoidance_command[1] * static_weight
                + inter_drone_avoidance[1] * drone_weight
            )

            # Limiter les vitesses
            max_speed = 0.8
            final_vx = max(-max_speed, min(max_speed, final_vx))
            final_vy = max(-max_speed, min(max_speed, final_vy))

            velocity_x = final_vx
            velocity_y = final_vy

            # Log détaillé des corrections d'évitement
            if (
                abs(avoidance_command[0]) > 0.05
                or abs(avoidance_command[1]) > 0.05
                or abs(inter_drone_avoidance[0]) > 0.05
                or abs(inter_drone_avoidance[1]) > 0.05
            ):
                self.get_logger().info(
                    f"[COLLISION_CORRECTION] "
                    f"Static=({avoidance_command[0]:.2f},{avoidance_command[1]:.2f}), "
                    f"Drone=({inter_drone_avoidance[0]:.2f},{inter_drone_avoidance[1]:.2f}), "
                    f"Final=({velocity_x:.2f},{velocity_y:.2f})"
                )

            msg = Twist()
            msg.linear.x = velocity_x
            msg.linear.y = velocity_y
            msg.angular.z = yaw_rate
            self.twist_publisher.publish(msg)
            
        except Exception as e:
            error_msg = f"[TIMER_CALLBACK] EXCEPTION in timer_callback: {type(e).__name__}: {str(e)}"
            self.get_logger().error(error_msg)
            import traceback
            self.get_logger().fatal(f"[TIMER_CALLBACK] Traceback: {traceback.format_exc()}")
            # Publier commande d'arrêt d'urgence
            emergency_msg = Twist()
            emergency_msg.linear.x = 0.0
            emergency_msg.linear.y = 0.0
            emergency_msg.angular.z = 0.0
            self.twist_publisher.publish(emergency_msg)

    def odom_subscribe_callback(self, msg):
        try:
            self.position[0] = msg.pose.pose.position.x
            self.position[1] = msg.pose.pose.position.y
            self.position[2] = msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            self.angles[0] = euler[0]
            self.angles[1] = euler[1]
            self.angles[2] = euler[2]
            self.position_update = True
        except Exception as e:
            error_msg = f"[ODOM_CALLBACK] EXCEPTION in odom callback: {type(e).__name__}: {str(e)}"
            self.get_logger().error(error_msg)
            import traceback
            self.get_logger().error(f"[ODOM_CALLBACK] Traceback: {traceback.format_exc()}")

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges
        self.ranges_received = True  # Mark that we have real sensor data


def main(args=None):
    rclpy.init(args=args)
    wall_following_multiranger = WallFollowingMultiranger()
    rclpy.spin(wall_following_multiranger)
    wall_following_multiranger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
