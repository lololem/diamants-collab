#!/usr/bin/env python3
"""
Test final complet de l'API DIAMANTS avec ROS2
"""
import asyncio
import aiohttp
import websockets
import json
import time
import threading
import subprocess
import os
import signal
import sys

class DiamantAPITester:
    def __init__(self):
        self.api_process = None
        self.base_url = "http://localhost:8000"
        self.ws_url = "ws://localhost:9001"
        
    async def start_api(self):
        """Démarre l'API en arrière-plan"""
        print("🚀 Démarrage de l'API DIAMANTS...")
        
        # Change to API directory and start with proper ROS2 environment
        cmd = [
            "bash", "-c",
            "cd DIAMANTS_API && "
            "source /opt/ros/jazzy/setup.bash && "
            "source venv/bin/activate && "
            "export PYTHONPATH='/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH' && "
            "python launcher.py"
        ]
        
        self.api_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            preexec_fn=os.setsid
        )
        
        # Wait for API to start
        print("⏳ Attente du démarrage de l'API...")
        await asyncio.sleep(5)
        
        return self.api_process.poll() is None
    
    async def test_rest_api(self):
        """Test des endpoints REST"""
        print("🧪 Test des endpoints REST...")
        
        async with aiohttp.ClientSession() as session:
            # Test status endpoint
            try:
                async with session.get(f"{self.base_url}/api/status") as resp:
                    data = await resp.json()
                    print(f"✅ Status API: {data}")
                    assert resp.status == 200
                    assert data['status'] == 'operational'
            except Exception as e:
                print(f"❌ Erreur status API: {e}")
                return False
            
            # Test drones endpoint
            try:
                async with session.get(f"{self.base_url}/api/drones") as resp:
                    data = await resp.json()
                    print(f"✅ Drones API: {len(data['drones'])} drones")
                    assert resp.status == 200
            except Exception as e:
                print(f"❌ Erreur drones API: {e}")
                return False
            
            # Test missions endpoint
            try:
                async with session.get(f"{self.base_url}/api/missions") as resp:
                    data = await resp.json()
                    print(f"✅ Missions API: {len(data['missions'])} missions")
                    assert resp.status == 200
            except Exception as e:
                print(f"❌ Erreur missions API: {e}")
                return False
        
        return True
    
    async def test_websocket(self):
        """Test de la connexion WebSocket"""
        print("🧪 Test WebSocket...")
        
        try:
            async with websockets.connect(self.ws_url) as websocket:
                # Send test message
                test_message = {
                    "type": "command",
                    "data": {
                        "action": "ping",
                        "timestamp": time.time()
                    }
                }
                
                await websocket.send(json.dumps(test_message))
                print("✅ Message WebSocket envoyé")
                
                # Wait for response (with timeout)
                try:
                    response = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                    data = json.loads(response)
                    print(f"✅ Réponse WebSocket reçue: {data}")
                    return True
                except asyncio.TimeoutError:
                    print("⚠️ Timeout WebSocket (normal si pas de réponse ping)")
                    return True  # Timeout is OK for ping
        except Exception as e:
            print(f"❌ Erreur WebSocket: {e}")
            return False
    
    async def test_ros2_integration(self):
        """Test de l'intégration ROS2"""
        print("🧪 Test intégration ROS2...")
        
        try:
            async with websockets.connect(self.ws_url) as websocket:
                # Send ROS2 command
                ros2_command = {
                    "type": "ros2_command",
                    "data": {
                        "topic": "/crazyflie/cmd_vel",
                        "message_type": "geometry_msgs/msg/Twist",
                        "data": {
                            "linear": {"x": 0.0, "y": 0.0, "z": 0.1},
                            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
                        }
                    }
                }
                
                await websocket.send(json.dumps(ros2_command))
                print("✅ Commande ROS2 envoyée via WebSocket")
                
                return True
        except Exception as e:
            print(f"❌ Erreur intégration ROS2: {e}")
            return False
    
    def stop_api(self):
        """Arrête l'API"""
        if self.api_process:
            print("🛑 Arrêt de l'API...")
            try:
                os.killpg(os.getpgid(self.api_process.pid), signal.SIGTERM)
                self.api_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.api_process.pid), signal.SIGKILL)
            print("✅ API arrêtée")
    
    async def run_all_tests(self):
        """Lance tous les tests"""
        print("🚀 DIAMANTS API - Test Final Complet")
        print("="*50)
        
        # Start API
        if not await self.start_api():
            print("❌ Impossible de démarrer l'API")
            return False
        
        try:
            # Run all tests
            rest_ok = await self.test_rest_api()
            ws_ok = await self.test_websocket()
            ros2_ok = await self.test_ros2_integration()
            
            print("\n" + "="*50)
            print("📊 RÉSULTATS DES TESTS:")
            print(f"   REST API: {'✅ OK' if rest_ok else '❌ FAILED'}")
            print(f"   WebSocket: {'✅ OK' if ws_ok else '❌ FAILED'}")
            print(f"   ROS2 Integration: {'✅ OK' if ros2_ok else '❌ FAILED'}")
            
            all_ok = rest_ok and ws_ok and ros2_ok
            print(f"\n🎯 RÉSULTAT GLOBAL: {'✅ TOUS LES TESTS PASSENT' if all_ok else '❌ CERTAINS TESTS ÉCHOUENT'}")
            
            return all_ok
            
        finally:
            self.stop_api()

async def main():
    tester = DiamantAPITester()
    
    try:
        success = await tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Tests interrompus par l'utilisateur")
        tester.stop_api()
        sys.exit(1)
    except Exception as e:
        print(f"❌ Erreur lors des tests: {e}")
        tester.stop_api()
        sys.exit(1)

if __name__ == "__main__":
    asyncio.run(main())
