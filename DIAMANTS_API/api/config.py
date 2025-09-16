"""
DIAMANTS API Configuration
Configuration settings for the API server avec imports conditionnels
"""

import os
from typing import Dict, Any, Optional
from dataclasses import dataclass, field

@dataclass
class Settings:
    """Application settings"""
    
    # API Configuration
    api_title: str = "DIAMANTS API"
    api_version: str = "1.0.0"
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    debug: bool = False
    
    # WebSocket Configuration
    websocket_host: str = "localhost"
    websocket_port: int = 8765
    max_connections: int = 100
    
    # ROS2 Configuration
    ros2_domain_id: int = 0
    ros2_namespace: str = "/diamants"
    
    cors_origins: list = field(default_factory=lambda: ["*"])
    
    # Backend Configuration
    backend_host: str = "localhost"
    backend_port: int = 11311  # ROS Master port
    
    # Logging Configuration
    log_level: str = "INFO"
    log_file: str = "diamants_api.log"
    
    # Security (if needed)
    secret_key: str = os.environ.get("DIAMANTS_SECRET_KEY", "your-secret-key-here")
    
    def __post_init__(self):
        """Charge les variables d'environnement"""
        # Charge depuis les variables d'environnement si disponibles
        for field_name, field in self.__dataclass_fields__.items():
            env_var = f"DIAMANTS_{field_name.upper()}"
            if env_var in os.environ:
                env_value = os.environ[env_var]
                # Conversion de type basique
                if field.type == int:
                    setattr(self, field_name, int(env_value))
                elif field.type == bool:
                    setattr(self, field_name, env_value.lower() in ('true', '1', 'yes'))
                elif field.type == list:
                    setattr(self, field_name, env_value.split(','))
                else:
                    setattr(self, field_name, env_value)

# Import conditionnel pour ROS2
def check_ros2_available() -> bool:
    """Vérifie si ROS2 est disponible"""
    try:
        import rclpy  # type: ignore
        return True
    except ImportError:
        return False

# Import conditionnel pour WebSocket  
def check_websocket_available() -> bool:
    """Vérifie si websockets est disponible"""
    try:
        import websockets  # type: ignore
        return True
    except ImportError:
        return False

# Global settings instance
settings = Settings()

# Topic mappings for ROS2
ROS2_TOPICS: Dict[str, str] = {
    "cmd_vel": "/crazyflie/cmd_vel",
    "takeoff": "/crazyflie/takeoff", 
    "land": "/crazyflie/land",
    "pose": "/crazyflie/pose",
    "battery": "/crazyflie/battery_state",
    "scan": "/crazyflie/scan"
}

# Message type mappings
ROS2_MESSAGE_TYPES: Dict[str, str] = {
    "cmd_vel": "geometry_msgs/Twist",
    "takeoff": "std_msgs/Empty",
    "land": "std_msgs/Empty", 
    "pose": "geometry_msgs/PoseStamped",
    "battery": "sensor_msgs/BatteryState",
    "scan": "sensor_msgs/LaserScan"
}

# API Response templates
API_RESPONSES: Dict[str, Dict[str, Any]] = {
    "success": {
        "status": "success",
        "message": "Operation completed successfully"
    },
    "error": {
        "status": "error", 
        "message": "Operation failed"
    },
    "not_found": {
        "status": "error",
        "message": "Resource not found"
    },
    "validation_error": {
        "status": "error",
        "message": "Validation error"
    }
}

# Default drone configurations
DEFAULT_DRONE_CONFIG: Dict[str, Any] = {
    "type": "crazyflie",
    "max_speed": 2.0,
    "max_altitude": 3.0,
    "battery_threshold": 20.0,
    "connection_timeout": 30.0
}

# Mission configurations
MISSION_CONFIGS: Dict[str, Dict[str, Any]] = {
    "scouting": {
        "min_drones": 1,
        "max_drones": 5,
        "default_altitude": 1.5,
        "coverage_radius": 10.0
    },
    "slam": {
        "min_drones": 2,
        "max_drones": 4,
        "default_altitude": 1.0,
        "mapping_resolution": 0.05
    },
    "collaborative": {
        "min_drones": 3,
        "max_drones": 10,
        "communication_range": 15.0,
        "coordination_interval": 1.0
    }
}
