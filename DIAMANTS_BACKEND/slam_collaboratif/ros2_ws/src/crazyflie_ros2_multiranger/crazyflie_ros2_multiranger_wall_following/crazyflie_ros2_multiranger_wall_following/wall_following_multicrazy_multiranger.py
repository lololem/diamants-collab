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

from typing import List, Dict, Any
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # type: ignore
from sensor_msgs.msg import LaserScan  # type: ignore
from geometry_msgs.msg import Twist, TransformStamped  # type: ignore
from std_srvs.srv import Trigger  # type: ignore
import tf_transformations  # type: ignore
from tf2_ros.static_transform_broadcaster import (  # type: ignore
    StaticTransformBroadcaster)  # type: ignore
from std_msgs.msg import String  # <-- nouvel import

# Import des modules locaux
from .config import (
    MODE_SCOOT, DEFAULT_DELAY, DEFAULT_ROBOT_PREFIX, DEFAULT_MAX_TURN_RATE,
    DEFAULT_MAX_FORWARD_SPEED, DEFAULT_QUEUE_SIZE, CONTROL_TIMER_PERIOD,
    TAKEOFF_ALTITUDE, TAKEOFF_RATE, WALL_DETECTION_DISTANCE,
    STOP_DESCENT_RATE
)
from .altitude_controller import AltitudeController
from .navigation_controller import NavigationController
from .obstacle_avoidance import ObstacleAvoidance
from .state_machine import StateMachine


class WallFollowingMultiranger(Node):
    def __init__(self):
        super().__init__('wall_following_multicrazy_multiranger')
        # Logs de debug optionnels
        # print("=== DEBUG: wall_following_multicrazy_multiranger.py ===")
        # self.get_logger().info("=== DEBUG: Constructor ===")
        
        # self.get_logger().info("[INIT] Démarrage du node...")

        # === PARAMS ===
        self.declare_parameter('robot_prefix', DEFAULT_ROBOT_PREFIX)
        self.declare_parameter('delay', DEFAULT_DELAY)
        self.declare_parameter('max_turn_rate', DEFAULT_MAX_TURN_RATE)
        self.declare_parameter('max_forward_speed', DEFAULT_MAX_FORWARD_SPEED)
        self.declare_parameter('wall_following_direction', 'right')
        self.declare_parameter('mode', MODE_SCOOT)
        self.declare_parameter('start_direction', 'right')
        
        # Instanciation explicite des attributs Python
        self.robot_prefix = self.get_parameter(
            'robot_prefix').get_parameter_value().string_value
        self.delay = self.get_parameter(
            'delay').get_parameter_value().double_value
        self.max_turn_rate = self.get_parameter(
            'max_turn_rate').get_parameter_value().double_value
        self.max_forward_speed = self.get_parameter(
            'max_forward_speed').get_parameter_value().double_value
        self.wall_following_direction = self.get_parameter(
            'wall_following_direction').get_parameter_value().string_value
        self.mode = self.get_parameter(
            'mode').get_parameter_value().string_value
        self.start_direction = self.get_parameter(
            'start_direction').get_parameter_value().string_value

        # self.get_logger().info(
        #     f"[PARAMS] robot_prefix = {self.robot_prefix}")

        # === INITIALISATION DES CONTRÔLEURS ===
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        
        # Contrôleur d'altitude
        self.altitude_controller = AltitudeController(TAKEOFF_ALTITUDE)
        
        # Contrôleur de navigation
        self.navigation_controller = NavigationController(
            self.max_forward_speed,
            self.start_direction
        )
        
        # Système d'évitement d'obstacles
        self.obstacle_avoidance = ObstacleAvoidance()
        
        # Machine d'état
        self.state_machine = StateMachine(self.delay, self.start_clock)
        self.state_machine.set_navigation_controller(
            self.navigation_controller)

        # === STATIC TF: odom → base_footprint ===
        try:
            # Tentative d'instanciation du StaticTransformBroadcaster
            self.static_broadcaster = StaticTransformBroadcaster(self)
            # self.get_logger().info(
            #     "[TF][INIT] StaticTransformBroadcaster OK")

            static_transform = TransformStamped()
            static_transform.header.stamp = self.get_clock().now().to_msg()
            static_transform.header.frame_id = f"{self.robot_prefix}/odom"
            static_transform.child_frame_id = (
                f"{self.robot_prefix}/base_footprint"
            )
            static_transform.transform.translation.x = 0.0
            static_transform.transform.translation.y = 0.0
            static_transform.transform.translation.z = 0.0
            static_transform.transform.rotation.x = 0.0
            static_transform.transform.rotation.y = 0.0
            static_transform.transform.rotation.z = 0.0
            static_transform.transform.rotation.w = 1.0

            # Debug publication du static transform
            # self.get_logger().info(
            #     f"[TF][INIT] Publication : "
            #     f"{static_transform.header.frame_id} --> "
            #     f"{static_transform.child_frame_id}"
            # )

            self.static_broadcaster.sendTransform(static_transform)
            # self.get_logger().info("[TF][INIT] StaticTransform envoyé !")

        except (ImportError, RuntimeError, AttributeError):
            # Capture les exceptions TF courantes
            # self.get_logger().error(
            #     "[TF][ERROR] Exception lors de l'init TF")
            pass

        # === ROS2 COMMUNICATIONS ===
        self.odom_subscriber = self.create_subscription(
            Odometry, f"/{self.robot_prefix}/odom",
            self.odom_callback, DEFAULT_QUEUE_SIZE)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, f"/{self.robot_prefix}/scan",
            self.scan_callback, DEFAULT_QUEUE_SIZE)
        self.srv = self.create_service(
            Trigger, f"/{self.robot_prefix}/stop_wall_following",
            self.stop_cb)
        self.twist_publisher = self.create_publisher(
            Twist, f"/{self.robot_prefix}/cmd_vel", DEFAULT_QUEUE_SIZE)
        # Subscribers initialisés

        # === Communication inter-drones ===
        # Publisher pour diffuser la position du drone
        self.swarm_pub = self.create_publisher(
            String, '/swarm/drone_positions', DEFAULT_QUEUE_SIZE)
        # Subscriber pour recevoir les positions des autres drones
        self.swarm_sub = self.create_subscription(
            String, '/swarm/drone_positions',
            self._swarm_callback, DEFAULT_QUEUE_SIZE)
        # Dictionnaire des positions des autres drones
        self._other_positions: Dict[str, List[float]] = {}
        # Timer pour publier régulièrement la position (toutes les 0,1 s)
        self.publish_timer = self.create_timer(
            0.1, self._publish_swarm_position
        )

        # === ÉTAT INTERNE ===
        self.position: List[float] = [0.0, 0.0, 0.0]
        self.angles: List[float] = [0.0, 0.0, 0.0]
        self.ranges: List[float] = [0.0, 0.0, 0.0, 0.0]
        self.position_update: bool = False
        self.random_walk_cooldown: float = 0.0  # Cooldown pour évitement

        # Timer pour le contrôle de vol
        self.timer = self.create_timer(
            CONTROL_TIMER_PERIOD, self.timer_callback)

    def stop_cb(self, request: Trigger.Request,
                response: Trigger.Response) -> Trigger.Response:
        # self.get_logger().info('Stopping wall following')
        self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = STOP_DESCENT_RATE
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)
        response.success = True
        return response

    def timer_callback(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        current_z = self.position[2]

        # Diagnostic: vérifie si on reçoit des données d'odométrie
        if not self.position_update:
            self.get_logger().warning(
                f"[DIAG] Pas de données d'odométrie reçues - "
                f"position: {self.position}")
            # On continue avec les dernières valeurs connues

        # Mise à jour de la phase de vol
        self.state_machine.update_phase(now, current_z)
        
        # Calcul de la correction d'altitude
        altitude_correction = self.altitude_controller.compute_correction(
            current_z)

        # ==== LOG DEBUG GLOBAL ====
        debug_info = self.altitude_controller.get_debug_info(current_z)
        self.get_logger().info(
            f"[ALTITUDE_LOOP] z={debug_info['current_z']:.3f} "
            f"target_z={debug_info['target_z']:.3f} | "
            f"err_z={debug_info['error_z']:.3f} "
            f"corr_filt={debug_info['filtered_correction']:.3f} "
            f"corr_clip={altitude_correction:.3f} | "
            f"phase={self.state_machine.get_phase_name()}")

        # Gestion des phases
        if self.state_machine.is_waiting():
            self.get_logger().debug('[PHASE] Waiting for start delay...')
            return

        elif self.state_machine.is_taking_off():
            msg = self.navigation_controller.create_takeoff_command(
                TAKEOFF_RATE)
            # Log activé pour diagnostic
            self.get_logger().warning(
                f"[PHASE] TAKEOFF | z={current_z:.3f} | "
                f"takeoff_z={TAKEOFF_RATE:.2f}")
            self.twist_publisher.publish(msg)
            return

        elif self.state_machine.is_boosting():
            msg = self.navigation_controller.create_boost_command(
                altitude_correction)
            # Log optionnel
            # self.get_logger().info(
            #     f"[PHASE] START BOOST | z={current_z:.3f} | "
            #     f"alt_corr={altitude_correction:.3f}")
            self.twist_publisher.publish(msg)
            return

        elif self.state_machine.is_advancing():
            msg = self.navigation_controller.create_advance_command(
                altitude_correction)
            # Log optionnel
            # self.get_logger().info(
            #     f"[PHASE] AVANCE | z={current_z:.3f} | "
            #     f"alt_corr={altitude_correction:.3f}")
            self.twist_publisher.publish(msg)
            return

        # Phase de navigation
        if self.state_machine.is_navigating():
            # Vérification de la longueur des ranges
            n_ranges = len(self.ranges)
            if n_ranges < 4:
                self.ranges += [float('inf')] * (4 - n_ranges)
            
            # Phase de rotation si nécessaire
            if self.navigation_controller.is_turning():
                msg = self.navigation_controller.create_turn_command(
                    altitude_correction)
                # Log optionnel
                # self.get_logger().warning(
                #     f"[PHASE] TURN | z={current_z:.3f} | "
                #     f"alt_corr={altitude_correction:.3f}")
                self.twist_publisher.publish(msg)
                return

            # Détection de mur
            if self.obstacle_avoidance.is_obstacle_detected(
                    self.ranges, WALL_DETECTION_DISTANCE):
                # Log optionnel
                # self.get_logger().error(
                #     f"[WALL DETECTED] | z={current_z:.3f} | "
                #     f"switching to TURN")
                self.navigation_controller.start_turn()
                self.navigation_controller.set_random_walk_timer(now)
                self.random_walk_cooldown = now + 0.5  # Cooldown de 0.5s
                return

            # Vérifier le cooldown après collision
            if now < self.random_walk_cooldown:
                return  # Attendre avant d'envoyer une nouvelle commande

            # Mouvement aléatoire avec évitement d'obstacles
            msg = self.navigation_controller.create_random_walk_command(
                altitude_correction, now)
            
            # Calcul de l'évitement avancé (prises en compte des obstacles et drones)
            avoid_x, avoid_y, avoid_z = (
                self.obstacle_avoidance.compute_advanced_avoidance(
                    self.position, self.ranges))
            msg.linear.x += avoid_x
            msg.linear.y += avoid_y
            msg.linear.z += avoid_z

            # Log de debug pour l'évitement
            if abs(avoid_x) > 0.01 or abs(avoid_y) > 0.01 or abs(avoid_z) > 0.01:
                self.get_logger().debug(
                    f"[AVOIDANCE] {self.robot_prefix} | "
                    f"ranges=[{self.ranges[0]:.2f},{self.ranges[1]:.2f},"
                    f"{self.ranges[2]:.2f},{self.ranges[3]:.2f}] | "
                    f"avoid=({avoid_x:.3f},{avoid_y:.3f},{avoid_z:.3f})")

            # Log optionnel
            # self.get_logger().info(
            #     f"[SCOOT] PHASE | x={msg.linear.x:.2f} "
            #     f"y={msg.linear.y:.2f} z={current_z:.3f} "
            #     f"alt_corr={altitude_correction:.3f}")
            self.twist_publisher.publish(msg)

    def update_other_drone_positions(
            self, positions_dict: Dict[str, Any]) -> None:
        """
        Met à jour les positions des autres drones pour l'évitement.
        
        Args:
            positions_dict (Dict[str, Any]): Dictionnaire des positions
                des autres drones
        """
        self.obstacle_avoidance.update_other_positions(positions_dict)

    def odom_callback(self, msg: Odometry) -> None:
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles = list(euler)
        self.position_update = True

    def scan_callback(self, msg: LaserScan) -> None:
        self.ranges = list(msg.ranges)

    def _publish_swarm_position(self) -> None:
        """
        Publie la position et une vitesse nulle du drone sur /swarm/drone_positions.
        Format du message : "drone_id:x:y:z:vx:vy:vz:timestamp".
        """
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        msg = String()
        msg.data = (
            f"{self.robot_prefix}:{self.position[0]}:{self.position[1]}:{self.position[2]}:"
            f"0.0:0.0:0.0:{timestamp}"
        )
        self.swarm_pub.publish(msg)

    def _swarm_callback(self, msg: String) -> None:
        """
        Callback appelé lorsqu'un message est reçu sur /swarm/drone_positions.
        Met à jour le dictionnaire des positions des autres drones et
        informe ObstacleAvoidance.
        """
        parts = msg.data.split(':')
        if len(parts) < 8:
            return
        drone_id = parts[0]
        # Ignorer la position du drone qui publie le message
        if drone_id == self.robot_prefix:
            return
        try:
            x = float(parts[1]); y = float(parts[2]); z = float(parts[3])
        except ValueError:
            return
        self._other_positions[drone_id] = [x, y, z]
        # Mise à jour pour la répulsion
        self.update_other_drone_positions(self._other_positions)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = WallFollowingMultiranger()
    # Le timer est déjà créé dans le constructeur
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
