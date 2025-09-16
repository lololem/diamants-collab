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
DIAMANTS V3 - Launcher intégré avec visualisation
================================================
Lance Gazebo + RViz + Interface Web de façon coordonnée
"""

import os
import sys
import time
import subprocess
import signal
import threading
from pathlib import Path

class DiamantLauncher:
    def __init__(self):
        self.processes = []
        # Calcul du chemin relatif vers le répertoire de base
        current_file = Path(__file__).resolve()
        self.base_dir = current_file.parents[7]  # Remonte jusqu'à DIAMANTS_BACKEND
        self.ros2_ws = self.base_dir / "slam_collaboratif/ros2_ws"
        self.running = False
        
    def setup_environment(self):
        """Configuration environnement ROS2"""
        print("🔧 Configuration environnement ROS2...")
        
        # Source ROS2 Jazzy
        ros2_setup = "source /opt/ros/jazzy/setup.bash"
        
        # Source workspace overlay si existe
        overlay_setup = ""
        if (self.ros2_ws / "install/setup.bash").exists():
            overlay_setup = f" && source {self.ros2_ws}/install/setup.bash"
        
        self.ros_env = f"{ros2_setup}{overlay_setup}"
        print(f"✅ Environnement: {self.ros_env}")
        
    def launch_gazebo(self):
        """Lancer Gazebo avec monde DIAMANTS"""
        print("🌍 Lancement Gazebo...")
        
        # Commande Gazebo avec monde par défaut
        gz_cmd = f"""
        {self.ros_env} && 
        gz sim empty.sdf -v 4
        """
        
        try:
            proc = subprocess.Popen(
                gz_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(("Gazebo", proc))
            print("✅ Gazebo démarré")
            return True
            
        except Exception as e:
            print(f"❌ Erreur Gazebo: {e}")
            return False
    
    def launch_rviz(self):
        """Lancer RViz avec configuration SLAM"""
        print("📊 Lancement RViz...")
        
        # Configuration RViz
        rviz_config = self.base_dir / "config/rviz_config_slam_optimized.rviz"
        
        if rviz_config.exists():
            rviz_cmd = f"""
            {self.ros_env} && 
            rviz2 -d {rviz_config}
            """
        else:
            rviz_cmd = f"""
            {self.ros_env} && 
            rviz2
            """
        
        try:
            proc = subprocess.Popen(
                rviz_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(("RViz", proc))
            print("✅ RViz démarré")
            return True
            
        except Exception as e:
            print(f"❌ Erreur RViz: {e}")
            return False
    
    def launch_crazyflie_sim(self):
        """Lancer simulation Crazyflie"""
        print("🚁 Lancement simulation Crazyflie...")
        
        cf_cmd = f"""
        {self.ros_env} && 
        ros2 launch ros_gz_crazyflie_bringup crazyflie.launch.py
        """
        
        try:
            proc = subprocess.Popen(
                cf_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(("Crazyflie Sim", proc))
            print("✅ Simulation Crazyflie démarrée")
            return True
            
        except Exception as e:
            print(f"❌ Erreur Crazyflie: {e}")
            return False
    
    def launch_web_interface(self):
        """Lancer interface web avec bridge ROS2"""
        print("🌐 Lancement interface web...")
        
        web_cmd = f"""
        {self.ros_env} && 
        cd {self.ros2_ws}/src/multi_agent_framework/multi_agent_framework/web_interface &&
        python3 web_server.py
        """
        
        try:
            proc = subprocess.Popen(
                web_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(("Web Interface", proc))
            print("✅ Interface web démarrée sur http://localhost:8080")
            return True
            
        except Exception as e:
            print(f"❌ Erreur interface web: {e}")
            return False
    
    def launch_websocket_bridge(self):
        """Lancer bridge WebSocket ↔ ROS2"""
        print("📡 Lancement bridge WebSocket...")
        
        bridge_cmd = f"""
        {self.ros_env} && 
        cd {self.ros2_ws}/src/multi_agent_framework/multi_agent_framework/web_interface &&
        python3 websocket_bridge.py
        """
        
        try:
            proc = subprocess.Popen(
                bridge_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.processes.append(("WebSocket Bridge", proc))
            print("✅ Bridge WebSocket démarré sur ws://localhost:8765")
            return True
            
        except Exception as e:
            print(f"❌ Erreur bridge: {e}")
            return False
    
    def check_dependencies(self):
        """Vérifier dépendances"""
        print("🔍 Vérification dépendances...")
        
        # Vérifier ROS2
        result = subprocess.run(
            "source /opt/ros/jazzy/setup.bash && ros2 --version",
            shell=True,
            capture_output=True,
            text=True
        )
        
        if result.returncode != 0:
            print("❌ ROS2 Jazzy non trouvé")
            return False
        
        print(f"✅ {result.stdout.strip()}")
        
        # Vérifier Gazebo
        result = subprocess.run("gz --version", shell=True, capture_output=True)
        if result.returncode != 0:
            print("❌ Gazebo non trouvé")
            return False
        
        print("✅ Gazebo disponible")
        
        # Vérifier workspace ROS2
        if not self.ros2_ws.exists():
            print(f"❌ Workspace ROS2 non trouvé: {self.ros2_ws}")
            return False
        
        print(f"✅ Workspace: {self.ros2_ws}")
        return True
    
    def launch_all(self):
        """Lancer tous les composants"""
        print("🚀 DIAMANTS V3 - Lancement complet")
        print("=" * 50)
        
        if not self.check_dependencies():
            print("❌ Dépendances manquantes")
            return False
        
        self.setup_environment()
        
        # Séquence de lancement
        components = [
            ("Gazebo", self.launch_gazebo, 3),
            ("Crazyflie Sim", self.launch_crazyflie_sim, 2),
            ("RViz", self.launch_rviz, 2),
            ("WebSocket Bridge", self.launch_websocket_bridge, 1),
            ("Web Interface", self.launch_web_interface, 1)
        ]
        
        self.running = True
        
        for name, launcher, delay in components:
            if not self.running:
                break
                
            print(f"\n⏳ Lancement {name}...")
            if launcher():
                print(f"✅ {name} démarré")
                time.sleep(delay)
            else:
                print(f"❌ Échec {name}")
                
        if self.running:
            print("\n" + "=" * 50)
            print("🎉 DIAMANTS V3 démarré avec succès!")
            print("🌐 Interface web: http://localhost:8080")
            print("📊 RViz: Interface SLAM")
            print("🌍 Gazebo: Simulation monde")
            print("🚁 Crazyflie: Drones simulés")
            print("\nAppuyez Ctrl+C pour arrêter")
            
            # Attendre signal arrêt
            try:
                while self.running:
                    time.sleep(1)
            except KeyboardInterrupt:
                self.shutdown()
        
        return True
    
    def shutdown(self):
        """Arrêter tous les processus"""
        print("\n🛑 Arrêt DIAMANTS V3...")
        self.running = False
        
        for name, proc in reversed(self.processes):
            try:
                print(f"⏹️ Arrêt {name}...")
                proc.terminate()
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(f"🔪 Force arrêt {name}")
                proc.kill()
            except Exception as e:
                print(f"⚠️ Erreur arrêt {name}: {e}")
        
        print("✅ DIAMANTS V3 arrêté")
    
    def status(self):
        """Vérifier statut processus"""
        print("📊 Statut DIAMANTS V3:")
        print("-" * 30)
        
        for name, proc in self.processes:
            if proc.poll() is None:
                print(f"✅ {name}: Running (PID {proc.pid})")
            else:
                print(f"❌ {name}: Stopped")

def signal_handler(signum, frame):
    """Gestionnaire signal"""
    global launcher
    if launcher:
        launcher.shutdown()
    sys.exit(0)

if __name__ == "__main__":
    launcher = DiamantLauncher()
    
    # Gestionnaire arrêt propre
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "status":
            launcher.status()
        elif sys.argv[1] == "check":
            launcher.check_dependencies()
        else:
            print("Usage: python3 diamant_launcher.py [status|check]")
    else:
        launcher.launch_all()