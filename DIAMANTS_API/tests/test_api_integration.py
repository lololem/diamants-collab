#!/usr/bin/env python3
"""
Test d'intégration complète de l'API DIAMANTS avec ROS2

Ce test E2E lance le serveur API puis le teste.
Marqué skip par défaut — lancer avec:  pytest -m e2e --run-e2e
"""
import asyncio
import pytest
import requests
import time
import threading
from subprocess import Popen, PIPE
import signal
import os

# Skip all tests in this module unless --run-e2e is passed
pytestmark = pytest.mark.skipif(
    not os.environ.get("DIAMANTS_RUN_E2E"),
    reason="E2E test — set DIAMANTS_RUN_E2E=1 to run",
)

try:
    import rclpy
except ImportError:
    rclpy = None

class TestDiamantsAPI:
    """Tests d'intégration pour l'API DIAMANTS"""
    
    @classmethod
    def setup_class(cls):
        """Setup avant tous les tests"""
        # Initialiser ROS2 pour les tests
        if not rclpy.ok():
            rclpy.init()
        
        # Lancer l'API en arrière-plan
        cls.api_process = None
        cls.start_api()
        
        # Attendre que l'API soit prête
        cls.wait_for_api()
    
    @classmethod
    def teardown_class(cls):
        """Nettoyage après tous les tests"""
        if cls.api_process:
            cls.api_process.terminate()
            cls.api_process.wait()
        
        if rclpy.ok():
            rclpy.shutdown()
    
    @classmethod
    def start_api(cls):
        """Lance l'API dans un processus séparé"""
        env = os.environ.copy()
        env['PYTHONPATH'] = "/opt/ros/jazzy/lib/python3.12/site-packages:" + env.get('PYTHONPATH', '')
        
        cmd = [
            'bash', '-c',
            'source /opt/ros/jazzy/setup.bash && source venv/bin/activate && python launcher.py'
        ]
        
        # Get current working directory as API root
        api_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        
        cls.api_process = Popen(cmd, env=env, stdout=PIPE, stderr=PIPE, 
                               cwd=api_root)
    
    @classmethod
    def wait_for_api(cls):
        """Attend que l'API soit prête"""
        max_attempts = 30
        for i in range(max_attempts):
            try:
                response = requests.get('http://localhost:8000/api/status', timeout=1)
                if response.status_code == 200:
                    print(f"✅ API prête après {i+1} tentatives")
                    return
            except:
                pass
            time.sleep(1)
        
        raise Exception("API non disponible après 30 secondes")
    
    def test_api_status(self):
        """Test l'endpoint status"""
        response = requests.get('http://localhost:8000/api/status')
        assert response.status_code == 200
        
        data = response.json()
        assert 'status' in data
        assert data['status'] == 'healthy'
        assert 'ros2_available' in data
        print("✅ Test API Status: OK")
    
    def test_drones_list(self):
        """Test l'endpoint des drones"""
        response = requests.get('http://localhost:8000/api/drones')
        assert response.status_code == 200
        
        data = response.json()
        assert 'drones' in data
        assert isinstance(data['drones'], list)
        print("✅ Test Drones List: OK")
    
    def test_mission_create(self):
        """Test la création d'une mission"""
        mission_data = {
            "name": "Test Mission",
            "type": "exploration",
            "drones": ["crazyflie_1"],
            "area": {
                "x_min": -10, "x_max": 10,
                "y_min": -10, "y_max": 10,
                "z_min": 0, "z_max": 5
            }
        }
        
        response = requests.post('http://localhost:8000/api/missions', json=mission_data)
        assert response.status_code == 201
        
        data = response.json()
        assert 'mission_id' in data
        assert data['status'] == 'created'
        print("✅ Test Mission Create: OK")
    
    def test_websocket_available(self):
        """Test que les WebSockets sont disponibles"""
        # Test du WebSocket service (port 8765)
        import socket
        sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        result1 = sock1.connect_ex(('localhost', 8765))
        sock1.close()
        
        assert result1 == 0, "WebSocket Bridge non disponible sur port 8765"
        print("✅ Test WebSocket Availability: OK")

if __name__ == '__main__':
    # Lancer les tests avec pytest
    pytest.main([__file__, '-v', '-s'])
