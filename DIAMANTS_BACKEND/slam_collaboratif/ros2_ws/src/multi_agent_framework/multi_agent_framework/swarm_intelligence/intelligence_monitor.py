#!/usr/bin/env python3
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
Intelligence Monitor V3 - Surveillance Intelligence Collective
============================================================
Version adaptée du monitor_auto_organisation.py de DIAMANTS V4
Compatible avec architecture ROS2 simplifiée
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
import time
import math
import json
from collections import defaultdict, deque
from typing import Dict, List, Tuple, Optional


class IntelligenceMonitor(Node):
    """Monitoring intelligence collective et auto-organisation"""
    
    def __init__(self, config: Optional[Dict] = None):
        super().__init__('intelligence_monitor')
        
        self.config = config or self._default_config()
        
        # État des drones
        self.drone_positions = {}
        self.drone_states = {}
        self.drone_intelligence = {}
        self.drone_last_seen = {}
        
        # Métriques historiques
        self.dispersion_history = deque(maxlen=100)
        self.coverage_history = deque(maxlen=100)
        self.intelligence_history = deque(maxlen=100)
        self.collision_risk_history = deque(maxlen=50)
        
        # Statistiques globales
        self.swarm_stats = {
            'total_interactions': 0,
            'coverage_area': 0.0,
            'average_intelligence': 0.0,
            'collision_events': 0,
            'mission_start_time': time.time()
        }
        
        # Abonnements ROS2
        self._setup_subscriptions()
        
        # Publishers pour métriques
        self.intelligence_pub = self.create_publisher(Float32, '/swarm/intelligence_score', 10)
        self.coverage_pub = self.create_publisher(Float32, '/swarm/coverage_area', 10)
        self.status_pub = self.create_publisher(String, '/swarm/status_report', 10)
        
        # Timer pour monitoring
        self.create_timer(2.0, self.monitor_callback)
        self.create_timer(10.0, self.publish_metrics)
        
        self.get_logger().info("🧠 Intelligence Monitor V3 démarré")
    
    def _default_config(self) -> Dict:
        """Configuration par défaut"""
        return {
            'update_frequency': 2.0,
            'cleanup_timeout': 5.0,
            'collision_threshold': 2.0,
            'intelligence_threshold': 0.5,
            'coverage_threshold': 50.0,
            'display_detailed': True
        }
    
    def _setup_subscriptions(self):
        """Configuration abonnements ROS2"""
        # Positions drones (topic standard)
        self.position_sub = self.create_subscription(
            String, '/multi_agent/drone_positions', 
            self.position_callback, 10
        )
        
        # États intelligence (nouveau topic)
        self.intelligence_sub = self.create_subscription(
            String, '/swarm/intelligence_states',
            self.intelligence_callback, 10
        )
        
        # États vol
        self.flight_sub = self.create_subscription(
            String, '/swarm/flight_states',
            self.flight_callback, 10
        )
    
    def position_callback(self, msg):
        """Réception positions drones"""
        try:
            # Format: "drone_id:x:y:z"
            parts = msg.data.split(':')
            if len(parts) >= 4:
                drone_id = parts[0]
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                
                self.drone_positions[drone_id] = [x, y, z]
                self.drone_last_seen[drone_id] = time.time()
                
        except Exception as e:
            self.get_logger().warn(f"Erreur parsing position: {e}")
    
    def intelligence_callback(self, msg):
        """Réception états intelligence"""
        try:
            # Format JSON pour états complexes
            state_data = json.loads(msg.data)
            drone_id = state_data.get('drone_id')
            
            if drone_id:
                self.drone_intelligence[drone_id] = {
                    'score': state_data.get('intelligence_score', 0.0),
                    'social_interactions': state_data.get('social_interactions', 0),
                    'exploration_state': state_data.get('exploration_state', 'unknown'),
                    'efficiency': state_data.get('efficiency', 0.0),
                    'timestamp': time.time()
                }
                
        except Exception as e:
            self.get_logger().debug(f"Erreur parsing intelligence: {e}")
    
    def flight_callback(self, msg):
        """Réception états de vol"""
        try:
            flight_data = json.loads(msg.data)
            drone_id = flight_data.get('drone_id')
            
            if drone_id:
                self.drone_states[drone_id] = {
                    'phase': flight_data.get('phase', 'unknown'),
                    'ready': flight_data.get('ready_for_navigation', False),
                    'emergency': flight_data.get('emergency_stop', False),
                    'collision_risk': flight_data.get('collision_risk', 0.0),
                    'timestamp': time.time()
                }
                
        except Exception as e:
            self.get_logger().debug(f"Erreur parsing flight state: {e}")
    
    def cleanup_inactive_drones(self):
        """Nettoyer drones inactifs"""
        current_time = time.time()
        timeout = self.config.get('cleanup_timeout', 5.0)
        
        # Positions
        active_positions = {
            drone_id: pos for drone_id, pos in self.drone_positions.items()
            if current_time - self.drone_last_seen.get(drone_id, 0) < timeout
        }
        self.drone_positions = active_positions
        
        # Intelligence
        active_intelligence = {
            drone_id: data for drone_id, data in self.drone_intelligence.items()
            if current_time - data['timestamp'] < timeout
        }
        self.drone_intelligence = active_intelligence
        
        # États vol
        active_states = {
            drone_id: data for drone_id, data in self.drone_states.items()
            if current_time - data['timestamp'] < timeout
        }
        self.drone_states = active_states
    
    def calculate_dispersion(self) -> float:
        """Calcul dispersion des drones (indicateur auto-organisation)"""
        if len(self.drone_positions) < 2:
            return 0.0
        
        positions = list(self.drone_positions.values())
        distances = []
        
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                dist = math.sqrt(
                    (positions[i][0] - positions[j][0])**2 + 
                    (positions[i][1] - positions[j][1])**2
                )
                distances.append(dist)
        
        return sum(distances) / len(distances) if distances else 0.0
    
    def calculate_coverage_area(self) -> float:
        """Estimation aire couverte par l'essaim"""
        if not self.drone_positions:
            return 0.0
        
        positions = list(self.drone_positions.values())
        
        # Bounding box
        min_x = min(pos[0] for pos in positions)
        max_x = max(pos[0] for pos in positions)
        min_y = min(pos[1] for pos in positions)
        max_y = max(pos[1] for pos in positions)
        
        area = abs(max_x - min_x) * abs(max_y - min_y)
        return min(area, 1000.0)  # Limite raisonnable
    
    def calculate_swarm_intelligence(self) -> float:
        """Calcul intelligence collective de l'essaim"""
        if not self.drone_intelligence:
            return 0.0
        
        total_score = sum(data['score'] for data in self.drone_intelligence.values())
        avg_score = total_score / len(self.drone_intelligence)
        
        # Bonus pour interactions sociales
        total_interactions = sum(data['social_interactions'] for data in self.drone_intelligence.values())
        interaction_bonus = min(total_interactions / 100.0, 0.3)
        
        # Score final
        swarm_intelligence = avg_score + interaction_bonus
        return min(swarm_intelligence, 1.0)
    
    def detect_collision_risks(self) -> List[Tuple[str, str, float]]:
        """Détection risques de collision"""
        risks = []
        threshold = self.config.get('collision_threshold', 2.0)
        
        positions = self.drone_positions
        drone_ids = list(positions.keys())
        
        for i in range(len(drone_ids)):
            for j in range(i + 1, len(drone_ids)):
                id1, id2 = drone_ids[i], drone_ids[j]
                pos1, pos2 = positions[id1], positions[id2]
                
                distance = math.sqrt(
                    (pos1[0] - pos2[0])**2 + 
                    (pos1[1] - pos2[1])**2 + 
                    (pos1[2] - pos2[2])**2
                )
                
                if distance < threshold:
                    risks.append((id1, id2, distance))
        
        return risks
    
    def monitor_callback(self):
        """Callback principal monitoring"""
        self.cleanup_inactive_drones()
        
        if not self.drone_positions:
            return
        
        # Calcul métriques
        dispersion = self.calculate_dispersion()
        coverage = self.calculate_coverage_area()
        intelligence = self.calculate_swarm_intelligence()
        collision_risks = self.detect_collision_risks()
        
        # Historiques
        self.dispersion_history.append(dispersion)
        self.coverage_history.append(coverage)
        self.intelligence_history.append(intelligence)
        self.collision_risk_history.append(len(collision_risks))
        
        # Mise à jour stats
        self.swarm_stats.update({
            'coverage_area': coverage,
            'average_intelligence': intelligence,
            'collision_events': len(collision_risks)
        })
        
        # Affichage si configuré
        if self.config.get('display_detailed', True):
            self._display_status(dispersion, coverage, intelligence, collision_risks)
    
    def _display_status(self, dispersion: float, coverage: float, 
                       intelligence: float, collision_risks: List):
        """Affichage statut détaillé"""
        current_time = time.strftime('%H:%M:%S')
        
        print("\n" + "="*70)
        print(f"🧠 INTELLIGENCE COLLECTIVE DIAMANTS - {current_time}")
        print("="*70)
        
        # Informations essaim
        print(f"🚁 Drones actifs: {len(self.drone_positions)}")
        print(f"📏 Dispersion: {dispersion:.1f}m")
        print(f"🗺️  Couverture: {coverage:.1f}m²")
        print(f"🧠 Intelligence: {intelligence:.3f}/1.0")
        
        # Risques collision
        if collision_risks:
            print(f"⚠️  Risques collision: {len(collision_risks)}")
            for id1, id2, dist in collision_risks[:3]:
                print(f"   {id1} ↔ {id2}: {dist:.1f}m")
        else:
            print("✅ Aucun risque collision")
        
        # États des drones
        if self.config.get('display_detailed', True):
            print("\n📍 États des drones:")
            for drone_id in sorted(self.drone_positions.keys()):
                pos = self.drone_positions[drone_id]
                
                # État intelligence
                intel_data = self.drone_intelligence.get(drone_id, {})
                intel_score = intel_data.get('score', 0.0)
                exploration = intel_data.get('exploration_state', 'unknown')
                
                # État vol
                flight_data = self.drone_states.get(drone_id, {})
                phase = flight_data.get('phase', 'unknown')
                ready = flight_data.get('ready', False)
                
                status = "🟢" if ready else "🟡" if phase == 'navigate' else "🔴"
                
                print(f"   {status} {drone_id}: ({pos[0]:+5.1f}, {pos[1]:+5.1f}, {pos[2]:4.1f}) "
                      f"| Phase: {phase} | Intel: {intel_score:.2f} | {exploration}")
        
        # Score global
        total_score = self._calculate_total_score(dispersion, coverage, intelligence, collision_risks)
        status_emoji = "🟢" if total_score > 80 else "🟡" if total_score > 50 else "🔴"
        print(f"\n{status_emoji} Score Global Essaim: {total_score}/100")
    
    def _calculate_total_score(self, dispersion: float, coverage: float, 
                              intelligence: float, collision_risks: List) -> int:
        """Calcul score global de performance"""
        score = 0
        
        # Dispersion (0-25 points)
        if dispersion > 3.0:
            score += 25
        elif dispersion > 1.5:
            score += 15
        else:
            score += 5
        
        # Couverture (0-30 points)
        coverage_score = min(coverage / 100.0 * 30, 30)
        score += int(coverage_score)
        
        # Intelligence (0-30 points)
        intelligence_score = intelligence * 30
        score += int(intelligence_score)
        
        # Sécurité (0-15 points)
        if len(collision_risks) == 0:
            score += 15
        elif len(collision_risks) < 3:
            score += 10
        else:
            score += 5
        
        return min(score, 100)
    
    def publish_metrics(self):
        """Publication métriques ROS2"""
        if self.drone_positions:
            # Intelligence
            intelligence = self.calculate_swarm_intelligence()
            intel_msg = Float32()
            intel_msg.data = intelligence
            self.intelligence_pub.publish(intel_msg)
            
            # Couverture
            coverage = self.calculate_coverage_area()
            coverage_msg = Float32()
            coverage_msg.data = coverage
            self.coverage_pub.publish(coverage_msg)
            
            # Rapport statut
            status_report = {
                'timestamp': time.time(),
                'active_drones': len(self.drone_positions),
                'intelligence_score': intelligence,
                'coverage_area': coverage,
                'dispersion': self.calculate_dispersion(),
                'collision_risks': len(self.detect_collision_risks())
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status_report)
            self.status_pub.publish(status_msg)
    
    def get_swarm_status(self) -> Dict:
        """Retour état complet essaim"""
        return {
            'active_drones': len(self.drone_positions),
            'positions': self.drone_positions.copy(),
            'intelligence_states': self.drone_intelligence.copy(),
            'flight_states': self.drone_states.copy(),
            'stats': self.swarm_stats.copy(),
            'recent_metrics': {
                'dispersion': list(self.dispersion_history)[-10:],
                'coverage': list(self.coverage_history)[-10:],
                'intelligence': list(self.intelligence_history)[-10:]
            }
        }


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    try:
        monitor = IntelligenceMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 Intelligence Monitor arrêté")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()