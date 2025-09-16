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
Configuration et constantes pour le système de navigation des drones Crazyflie.
Ce module centralise tous les paramètres du système pour faciliter
la maintenance.
"""

# === MODES DE FONCTIONNEMENT ===
MODE_WALL = "wall_following"
MODE_SCOOT = "scout"

# === PARAMÈTRES DE DÉCOLLAGE ===
DEFAULT_DELAY = 2.0
TAKEOFF_ALTITUDE = 0.20
TAKEOFF_RATE = 0.35
ADVANCE_AFTER_TAKEOFF = 20

# === PARAMÈTRES DE ROTATION ===
TURN_DEGREES = 180
TURN_DURATION = 30

# === CONTRÔLE D'ALTITUDE PD + FILTRE ===
K_Z = 1.05
D_Z = 0.85
ALPHA_Z = 0.55   # filtre passe-bas pour correction altitude
MAX_ALTITUDE_SPEED = 0.13  # clipping élargi (au lieu de 0.15)
DEADBAND_Z = 0.01  # On arrête la correction si proche

# === PARAMÈTRES DE MOUVEMENT ===
# Réduire les vitesses pour éviter les collisions avec les arbres
DEFAULT_MAX_TURN_RATE = 0.3    # au lieu de 0.5 - rotation plus lente
DEFAULT_MAX_FORWARD_SPEED = 0.3  # au lieu de 0.5 - vitesse avant plus lente

# === BOOST INITIAL ===
# Réduire également le boost initial pour plus de sécurité
START_BOOST_TICKS = 10  # Nombre de cycles pour le boost initial
START_BOOST_FORWARD_SPEED = 0.12  # au lieu de 0.18 - vitesse avant initiale plus douce
START_BOOST_LATERAL_SPEED = 0.08  # au lieu de 0.13 - vitesse latérale initiale plus douce

# === RANDOM WALK (mode scout) ===
# Paramètres pour le déplacement aléatoire
RANDOM_WALK_MIN_VX = 0.25  # Vitesse minimale en X
RANDOM_WALK_MAX_VX_RANGE = 0.35  # Amplitude de la vitesse en X
RANDOM_WALK_MIN_VY = -0.17  # Vitesse minimale en Y
RANDOM_WALK_MAX_VY = 0.17  # Vitesse maximale en Y
RANDOM_WALK_MIN_YAW = -1.1  # Rotation minimale
RANDOM_WALK_MAX_YAW = 1.1  # Rotation maximale
RANDOM_WALK_MIN_INTERVAL = 1.0  # Intervalle minimal entre changements
RANDOM_WALK_MAX_INTERVAL_RANGE = 1.8  # Amplitude de l'intervalle
RANDOM_WALK_INITIAL_INTERVAL_BASE = 1.7  # Base de l'intervalle initial

# === DÉTECTION D'OBSTACLES ===
# Augmenter encore plus la distance de détection pour mieux anticiper les arbres
WALL_DETECTION_DISTANCE = 0.80  # au lieu de 0.60 - détection plus précoce

# === RÉPULSION OBSTACLES (arbres) ===
# Augmenter davantage le seuil de répulsion et le gain pour éviter les collisions
REPULSION_DIST = 0.70          # au lieu de 0.50 - répulsion plus large 
REPULSION_GAIN = 1.8           # au lieu de 1.2 - force de répulsion plus forte

# === RÉPULSION INTER-DRONES ===
# Portée maximale de répulsion (en mètres)
REPULSION_DRONE_DIST = 0.60  # anciennement 0.40
# Gain de répulsion inter-drones (force appliquée lorsque la distance diminue)
REPULSION_DRONE_GAIN = 1.2   # anciennement 0.8
MIN_DRONE_DISTANCE = 0.01  # distance minimale pour éviter division par zéro

# === FRÉQUENCE DE CONTRÔLE ===
CONTROL_TIMER_PERIOD = 0.03  # 33 Hz

# === PARAMÈTRES ROS2 ===
DEFAULT_ROBOT_PREFIX = 'crazyflie1'
DEFAULT_QUEUE_SIZE = 10

# === PARAMÈTRES DE STOP ===
STOP_DESCENT_RATE = -0.2
