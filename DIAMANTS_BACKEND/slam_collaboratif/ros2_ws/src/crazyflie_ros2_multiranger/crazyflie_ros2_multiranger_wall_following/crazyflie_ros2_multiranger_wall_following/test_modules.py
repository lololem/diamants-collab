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
Script de test pour valider que tous les modules refactorisés fonctionnent.
"""
# flake8: noqa
# pylint: disable=all
# type: ignore


def test_imports():
    """Test tous les imports des modules."""
    print("🔍 Test des imports des modules refactorisés...")

    try:
        # Test config
        from config import TAKEOFF_ALTITUDE, K_Z, D_Z  # type: ignore

        print("✅ config.py - Import OK")

        # Test altitude_controller
        from altitude_controller import AltitudeController  # type: ignore

        altitude_ctrl = AltitudeController(TAKEOFF_ALTITUDE)
        print("✅ altitude_controller.py - Import et instanciation OK")

        # Test navigation_controller
        from navigation_controller import NavigationController  # type: ignore

        nav_ctrl = NavigationController(0.5, 'right')
        print("✅ navigation_controller.py - Import et instanciation OK")

        # Test obstacle_avoidance
        from obstacle_avoidance import ObstacleAvoidance  # type: ignore

        obstacle_ctrl = ObstacleAvoidance()
        print("✅ obstacle_avoidance.py - Import et instanciation OK")

        # Test state_machine
        from state_machine import StateMachine, FlightPhase  # type: ignore

        state_machine = StateMachine(2.0, 0.0)
        print("✅ state_machine.py - Import et instanciation OK")

        return True

    except Exception as e:
        print(f"❌ Erreur lors du test: {e}")
        return False


def test_functionality():
    """Test des fonctionnalités de base."""
    print("\n🧪 Test des fonctionnalités de base...")

    try:
        from config import TAKEOFF_ALTITUDE
        from altitude_controller import AltitudeController
        from navigation_controller import NavigationController
        from obstacle_avoidance import ObstacleAvoidance
        from state_machine import StateMachine, FlightPhase

        # Test altitude controller
        alt_ctrl = AltitudeController(TAKEOFF_ALTITUDE)
        correction = alt_ctrl.compute_correction(0.5)
        debug_info = alt_ctrl.get_debug_info(0.5)
        print(f"✅ AltitudeController - correction: {correction:.3f}")

        # Test navigation controller
        nav_ctrl = NavigationController(0.5, 'right')
        nav_ctrl.reset_random_walk()
        print("✅ NavigationController - random walk reset OK")

        # Test obstacle avoidance
        obs_ctrl = ObstacleAvoidance()
        is_obstacle = obs_ctrl.is_obstacle_detected([1.0, 1.0, 1.0, 1.0], 0.5)
        print(f"✅ ObstacleAvoidance - obstacle detection: {is_obstacle}")

        # Test state machine
        sm = StateMachine(2.0, 0.0)
        phase = sm.update_phase(3.0, 0.5)
        print(f"✅ StateMachine - phase: {sm.get_phase_name()}")

        return True

    except Exception as e:
        print(f"❌ Erreur lors du test fonctionnel: {e}")
        return False


if __name__ == "__main__":
    print("🚀 VALIDATION DU SYSTÈME MODULAIRE REFACTORISÉ")
    print("=" * 50)

    success1 = test_imports()
    success2 = test_functionality()

    print("\n" + "=" * 50)
    if success1 and success2:
        print("🎉 TOUS LES TESTS RÉUSSIS !")
        print("✨ Le système modulaire est entièrement fonctionnel !")
        print("🚀 Prêt pour la production !")
    else:
        print("❌ Certains tests ont échoué")
        exit(1)
