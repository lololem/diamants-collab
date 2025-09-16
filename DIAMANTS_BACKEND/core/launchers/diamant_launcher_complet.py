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
DIAMANTS V3 - Launcher Intégré Complet
=====================================
Lance Gazebo + RViz + Interface Web de manière coordonnée
"""

import os
import sys
import time
import signal
import subprocess
import threading
from pathlib import Path
from typing import List, Dict, Optional

class DiamantLauncher:
    """Launcher intégré pour DIAMANTS V3"""
    
    def __init__(self):
    # Resolve repository root dynamically (…/ROS-GZ)
    self.base_dir = Path(__file__).resolve().parents[2]
        self.ros2_ws = self.base_dir / "slam_collaboratif/ros2_ws"
        self.processes: List[subprocess.Popen] = []
        self.running = False
        
        # Configuration environnement
        self.env = os.environ.copy()
        self.setup_environment()
        
        print("🚀 DIAMANTS V3 Launcher Intégré")
        print(f"📁 Workspace: {self.ros2_ws}")
    
    def setup_environment(self):
        """Configuration environnement ROS2"""
        self.env.update({
            'ROS_DOMAIN_ID': '42',
            'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
            'RCUTILS_LOGGING_USE_STDOUT': '1'
        })
        
        # Source ROS2 Jazzy
        jazzy_setup = "/opt/ros/jazzy/setup.bash"
        if os.path.exists(jazzy_setup):
            print("✅ ROS2 Jazzy trouvé")
        else:
            print("❌ ROS2 Jazzy non trouvé")
        
        # Source workspace overlay
        overlay_setup = self.ros2_ws / "install/setup.bash"
        if overlay_setup.exists():
            print("✅ Workspace overlay trouvé")
        else:
            print("⚠️ Workspace overlay manquant - construction requise")
    
    def run_command(self, cmd: List[str], name: str, cwd: Optional[Path] = None, wait_time: float = 2.0) -> subprocess.Popen:
        """Exécuter commande avec gestion d'erreurs"""
        print(f"🔄 Lancement: {name}")
        print(f"   Commande: {' '.join(cmd)}")
        
        if cwd:
            print(f"   Répertoire: {cwd}")
        
        try:
            # Commande bash pour sourcer l'environnement
            bash_cmd = [
                "bash", "-c", 
                f"source /opt/ros/jazzy/setup.bash && "
                f"{'source ' + str(self.ros2_ws / 'install/setup.bash') + ' && ' if (self.ros2_ws / 'install/setup.bash').exists() else ''}"
                f"{' '.join(cmd)}"
            ]
            
            process = subprocess.Popen(
                bash_cmd,
                cwd=cwd or self.ros2_ws,
                env=self.env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True
            )
            
            self.processes.append(process)
            
            # Attendre démarrage
            time.sleep(wait_time)
            
            # Vérifier si processus encore vivant
            if process.poll() is None:
                print(f"✅ {name} démarré (PID: {process.pid})")
                return process
            else:
                stdout, stderr = process.communicate()
                print(f"❌ {name} a échoué")
                if stdout:
                    print(f"   STDOUT: {stdout}")
                if stderr:
                    print(f"   STDERR: {stderr}")
                return None
                
        except Exception as e:
            print(f"❌ Erreur lancement {name}: {e}")
            return None
    
    def check_dependencies(self) -> bool:
        """Vérifier dépendances système"""
        print("\n🔍 Vérification dépendances...")
        
        deps = {
            "gz": "Gazebo",
            "ros2": "ROS2",
            "rviz2": "RViz2",
            "python3": "Python3"
        }
        
        missing = []
        for cmd, name in deps.items():
            try:
                result = subprocess.run(
                    ["which", cmd], 
                    capture_output=True, 
                    text=True
                )
                if result.returncode == 0:
                    print(f"✅ {name}: {result.stdout.strip()}")
                else:
                    print(f"❌ {name}: Non trouvé")
                    missing.append(name)
            except:
                print(f"❌ {name}: Erreur vérification")
                missing.append(name)
        
        if missing:
            print(f"\n⚠️ Dépendances manquantes: {', '.join(missing)}")
            return False
        
        print("\n✅ Toutes les dépendances sont disponibles")
        return True
    
    def build_workspace(self) -> bool:
        """Construire workspace ROS2 si nécessaire"""
        if not (self.ros2_ws / "install").exists():
            print("\n🔨 Construction du workspace ROS2...")
            
            build_cmd = [
                "colcon", "build", 
                "--cmake-args", "-DCMAKE_BUILD_TYPE=Release",
                "--packages-select", "multi_agent_framework", "slam_map_merge", "ros_gz_crazyflie_bringup"
            ]
            
            process = self.run_command(build_cmd, "Build ROS2", cwd=self.ros2_ws, wait_time=30.0)
            
            if process and process.poll() == 0:
                print("✅ Workspace construit avec succès")
                return True
            else:
                print("❌ Échec construction workspace")
                return False
        
        print("✅ Workspace déjà construit")
        return True
    
    def launch_gazebo(self) -> Optional[subprocess.Popen]:
        """Lancer Gazebo avec monde Crazyflie"""
        print("\n🌍 Lancement Gazebo...")
        
        # Vérifier si Gazebo déjà en cours
        try:
            result = subprocess.run(["pgrep", "-f", "gz sim"], capture_output=True)
            if result.returncode == 0:
                print("⚠️ Gazebo déjà en cours d'exécution")
                return None
        except:
            pass
        
        # Monde par défaut ou spécifique
        world_file = "empty.sdf"  # Monde simple pour commencer
        
        gazebo_cmd = [
            "gz", "sim", world_file, 
            "--verbose", "1"
        ]
        
        return self.run_command(gazebo_cmd, "Gazebo", wait_time=5.0)
    
    def launch_rviz(self) -> Optional[subprocess.Popen]:
        """Lancer RViz avec configuration SLAM"""
        print("\n📊 Lancement RViz...")
        
        # Configuration RViz
        rviz_config = self.base_dir / "config/rviz_config_slam_optimized.rviz"
        
        rviz_cmd = ["rviz2"]
        if rviz_config.exists():
            rviz_cmd.extend(["-d", str(rviz_config)])
            print(f"   Config: {rviz_config}")
        
        return self.run_command(rviz_cmd, "RViz2", wait_time=3.0)
    
    def launch_web_interface(self) -> Optional[subprocess.Popen]:
        """Lancer interface web"""
        print("\n🌐 Lancement Interface Web...")
        
        web_server = self.ros2_ws / "src/multi_agent_framework/multi_agent_framework/web_interface/web_server_robust.py"
        
        if not web_server.exists():
            print(f"❌ Serveur web non trouvé: {web_server}")
            return None
        
        web_cmd = ["python3", str(web_server)]
        
        return self.run_command(web_cmd, "Interface Web", wait_time=3.0)
    
    def launch_ros2_nodes(self) -> List[subprocess.Popen]:
        """Lancer nœuds ROS2 essentiels"""
        print("\n🤖 Lancement nœuds ROS2...")
        
        nodes = []
        
        # TF publisher pour transformations
        tf_cmd = ["ros2", "run", "tf2_ros", "static_transform_publisher", 
                  "0", "0", "0", "0", "0", "0", "map", "odom"]
        tf_process = self.run_command(tf_cmd, "TF Publisher", wait_time=1.0)
        if tf_process:
            nodes.append(tf_process)
        
        # Map server (optionnel)
        # map_cmd = ["ros2", "run", "nav2_map_server", "map_server", "--ros-args", "-p", "yaml_filename:=map.yaml"]
        # map_process = self.run_command(map_cmd, "Map Server", wait_time=2.0)
        
        return nodes
    
    def monitor_processes(self):
        """Monitorer processus en arrière-plan"""
        while self.running:
            time.sleep(5.0)
            
            dead_processes = []
            for i, process in enumerate(self.processes):
                if process and process.poll() is not None:
                    print(f"💀 Processus mort détecté (PID: {process.pid})")
                    dead_processes.append(i)
            
            # Nettoyer processus morts
            for i in reversed(dead_processes):
                self.processes.pop(i)
        
        print("🛑 Monitoring arrêté")
    
    def launch_full_system(self):
        """Lancer système complet"""
        print("\n" + "="*60)
        print("🚀 LANCEMENT SYSTÈME DIAMANTS V3 COMPLET")
        print("="*60)
        
        # 1. Vérifications préliminaires
        if not self.check_dependencies():
            return False
        
        # 2. Construction workspace
        if not self.build_workspace():
            return False
        
        # 3. Lancement Gazebo
        gazebo_process = self.launch_gazebo()
        if not gazebo_process:
            print("⚠️ Gazebo non lancé - continuons")
        
        # 4. Lancement RViz
        rviz_process = self.launch_rviz()
        if not rviz_process:
            print("⚠️ RViz non lancé - continuons")
        
        # 5. Lancement nœuds ROS2
        ros2_nodes = self.launch_ros2_nodes()
        
        # 6. Lancement interface web
        web_process = self.launch_web_interface()
        if not web_process:
            print("❌ Interface web non lancée")
            return False
        
        # 7. Démarrer monitoring
        self.running = True
        monitor_thread = threading.Thread(target=self.monitor_processes, daemon=True)
        monitor_thread.start()
        
        # 8. Informations finales
        print("\n" + "="*60)
        print("✅ SYSTÈME DIAMANTS V3 DÉMARRÉ")
        print("="*60)
        print(f"🌍 Gazebo: {'✅ Actif' if gazebo_process else '❌ Inactif'}")
        print(f"📊 RViz: {'✅ Actif' if rviz_process else '❌ Inactif'}")
        print(f"🤖 Nœuds ROS2: {len(ros2_nodes)} actifs")
        print(f"🌐 Interface Web: {'✅ Actif' if web_process else '❌ Inactif'}")
        print(f"🔗 Processus total: {len(self.processes)}")
        print("\n📱 Accès Interface Web: http://localhost:8080")
        print("⌨️  Ctrl+C pour arrêter le système")
        print("="*60)
        
        return True
    
    def shutdown(self):
        """Arrêt propre du système"""
        print("\n🛑 Arrêt système DIAMANTS V3...")
        
        self.running = False
        
        # Terminer tous les processus
        for i, process in enumerate(self.processes):
            if process and process.poll() is None:
                print(f"🔄 Arrêt processus {i+1}/{len(self.processes)} (PID: {process.pid})")
                try:
                    process.terminate()
                    time.sleep(2.0)
                    
                    if process.poll() is None:
                        print(f"💀 Forçage arrêt processus {process.pid}")
                        process.kill()
                    
                except Exception as e:
                    print(f"❌ Erreur arrêt processus: {e}")
        
        print("✅ Système arrêté")
    
    def run(self):
        """Point d'entrée principal"""
        try:
            success = self.launch_full_system()
            
            if success:
                # Attendre interruption utilisateur
                while self.running:
                    time.sleep(1.0)
            else:
                print("❌ Échec lancement système")
                
        except KeyboardInterrupt:
            print("\n\n⌨️ Interruption utilisateur détectée")
        except Exception as e:
            print(f"\n❌ Erreur système: {e}")
        finally:
            self.shutdown()


def main():
    """Point d'entrée"""
    launcher = DiamantLauncher()
    
    # Gestionnaire signal pour arrêt propre
    def signal_handler(sig, frame):
        launcher.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    launcher.run()


if __name__ == "__main__":
    main()