# Type stubs pour ROS2 - à utiliser avec pylance
# Ces stubs aident l'éditeur à reconnaître les imports ROS2

import sys
from typing import Any, Optional, Dict, List

# Stub pour rclpy
class Node:
    def __init__(self, node_name: str) -> None: ...
    def create_publisher(self, msg_type: Any, topic: str, qos_profile: int) -> Any: ...
    def create_subscription(self, msg_type: Any, topic: str, callback: Any, qos_profile: int) -> Any: ...
    def create_service(self, srv_type: Any, srv_name: str, callback: Any) -> Any: ...
    def create_client(self, srv_type: Any, srv_name: str) -> Any: ...
    def get_logger(self) -> Any: ...

def init(args: Optional[List[str]] = None) -> None: ...
def shutdown() -> None: ...
def spin_once(node: Node, timeout_sec: Optional[float] = None) -> None: ...
def spin(node: Node) -> None: ...

# Stub pour geometry_msgs
class Twist:
    def __init__(self) -> None:
        self.linear: Any = None
        self.angular: Any = None

class Vector3:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0  
        self.z: float = 0.0

class Point:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0

class Quaternion:
    def __init__(self) -> None:
        self.x: float = 0.0
        self.y: float = 0.0
        self.z: float = 0.0
        self.w: float = 1.0

class Pose:
    def __init__(self) -> None:
        self.position: Point = Point()
        self.orientation: Quaternion = Quaternion()

class PoseStamped:
    def __init__(self) -> None:
        self.header: Any = None
        self.pose: Pose = Pose()

# Stub pour std_msgs
class Empty:
    def __init__(self) -> None: ...

class String:
    def __init__(self) -> None:
        self.data: str = ""

# Stub pour sensor_msgs
class BatteryState:
    def __init__(self) -> None:
        self.voltage: float = 0.0
        self.current: float = 0.0
        self.charge: float = 0.0
        self.capacity: float = 0.0
        self.percentage: float = 0.0

class LaserScan:
    def __init__(self) -> None:
        self.header: Any = None
        self.angle_min: float = 0.0
        self.angle_max: float = 0.0
        self.ranges: List[float] = []

# Ajouter les stubs aux modules sys pour l'importation
if 'rclpy' not in sys.modules:
    import types
    rclpy = types.ModuleType('rclpy')
    rclpy.init = init
    rclpy.shutdown = shutdown
    rclpy.spin_once = spin_once
    rclpy.spin = spin
    
    node_module = types.ModuleType('rclpy.node')
    node_module.Node = Node
    rclpy.node = node_module
    
    sys.modules['rclpy'] = rclpy
    sys.modules['rclpy.node'] = node_module

# Messages modules
if 'geometry_msgs' not in sys.modules:
    import types
    geom_msgs = types.ModuleType('geometry_msgs')
    geom_msg_module = types.ModuleType('geometry_msgs.msg')
    geom_msg_module.Twist = Twist
    geom_msg_module.Vector3 = Vector3
    geom_msg_module.Point = Point
    geom_msg_module.Quaternion = Quaternion
    geom_msg_module.Pose = Pose
    geom_msg_module.PoseStamped = PoseStamped
    geom_msgs.msg = geom_msg_module
    
    sys.modules['geometry_msgs'] = geom_msgs
    sys.modules['geometry_msgs.msg'] = geom_msg_module

if 'std_msgs' not in sys.modules:
    import types
    std_msgs = types.ModuleType('std_msgs')
    std_msg_module = types.ModuleType('std_msgs.msg')
    std_msg_module.Empty = Empty
    std_msg_module.String = String
    std_msgs.msg = std_msg_module
    
    sys.modules['std_msgs'] = std_msgs
    sys.modules['std_msgs.msg'] = std_msg_module

if 'sensor_msgs' not in sys.modules:
    import types
    sensor_msgs = types.ModuleType('sensor_msgs')
    sensor_msg_module = types.ModuleType('sensor_msgs.msg')
    sensor_msg_module.BatteryState = BatteryState
    sensor_msg_module.LaserScan = LaserScan
    sensor_msgs.msg = sensor_msg_module
    
    sys.modules['sensor_msgs'] = sensor_msgs
    sys.modules['sensor_msgs.msg'] = sensor_msg_module
