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
"""
Module __init__.py pour le package crazyflie_ros2_multiranger_wall_following.
Importe tous les modules nécessaires pour faciliter leur utilisation.
"""

from .config import (
    MODE_WALL,
    MODE_SCOOT,
    DEFAULT_DELAY,
    TAKEOFF_ALTITUDE,
    TAKEOFF_RATE,
    ADVANCE_AFTER_TAKEOFF,
    TURN_DEGREES,
    TURN_DURATION,
    K_Z,
    D_Z,
    ALPHA_Z,
    MAX_ALTITUDE_SPEED,
    DEADBAND_Z,
    DEFAULT_MAX_TURN_RATE,
    DEFAULT_MAX_FORWARD_SPEED,
    START_BOOST_TICKS,
    START_BOOST_FORWARD_SPEED,
    START_BOOST_LATERAL_SPEED,
    RANDOM_WALK_MIN_VX,
    RANDOM_WALK_MAX_VX_RANGE,
    RANDOM_WALK_MIN_VY,
    RANDOM_WALK_MAX_VY,
    RANDOM_WALK_MIN_YAW,
    RANDOM_WALK_MAX_YAW,
    RANDOM_WALK_MIN_INTERVAL,
    RANDOM_WALK_MAX_INTERVAL_RANGE,
    RANDOM_WALK_INITIAL_INTERVAL_BASE,
    WALL_DETECTION_DISTANCE,
    REPULSION_DIST,
    REPULSION_GAIN,
    REPULSION_DRONE_DIST,
    REPULSION_DRONE_GAIN,
    MIN_DRONE_DISTANCE,
    CONTROL_TIMER_PERIOD,
    DEFAULT_ROBOT_PREFIX,
    DEFAULT_QUEUE_SIZE,
    STOP_DESCENT_RATE,
)
from .altitude_controller import AltitudeController
from .navigation_controller import NavigationController
from .obstacle_avoidance import ObstacleAvoidance
from .state_machine import StateMachine, FlightPhase
from .advanced_collision_manager import RiskLevel, Threat, FlightZone, CollisionManager, CollisionController

# Module file_protector supprimé lors du nettoyage - imports supprimés

__all__ = [
    # Configuration constants
    'MODE_WALL',
    'MODE_SCOOT',
    'DEFAULT_DELAY',
    'TAKEOFF_ALTITUDE',
    'TAKEOFF_RATE',
    'ADVANCE_AFTER_TAKEOFF',
    'TURN_DEGREES',
    'TURN_DURATION',
    'K_Z',
    'D_Z',
    'ALPHA_Z',
    'MAX_ALTITUDE_SPEED',
    'DEADBAND_Z',
    'DEFAULT_MAX_TURN_RATE',
    'DEFAULT_MAX_FORWARD_SPEED',
    'START_BOOST_TICKS',
    'START_BOOST_FORWARD_SPEED',
    'START_BOOST_LATERAL_SPEED',
    'RANDOM_WALK_MIN_VX',
    'RANDOM_WALK_MAX_VX_RANGE',
    'RANDOM_WALK_MIN_VY',
    'RANDOM_WALK_MAX_VY',
    'RANDOM_WALK_MIN_YAW',
    'RANDOM_WALK_MAX_YAW',
    'RANDOM_WALK_MIN_INTERVAL',
    'RANDOM_WALK_MAX_INTERVAL_RANGE',
    'RANDOM_WALK_INITIAL_INTERVAL_BASE',
    'WALL_DETECTION_DISTANCE',
    'REPULSION_DIST',
    'REPULSION_GAIN',
    'REPULSION_DRONE_DIST',
    'REPULSION_DRONE_GAIN',
    'MIN_DRONE_DISTANCE',
    'CONTROL_TIMER_PERIOD',
    'DEFAULT_ROBOT_PREFIX',
    'DEFAULT_QUEUE_SIZE',
    'STOP_DESCENT_RATE',
    # Class modules
    'AltitudeController',
    'NavigationController',
    'ObstacleAvoidance',
    'StateMachine',
    'FlightPhase',
    # Advanced collision management
    'RiskLevel',
    'Threat',
    'FlightZone',
    'CollisionManager',
    'CollisionController',
]
