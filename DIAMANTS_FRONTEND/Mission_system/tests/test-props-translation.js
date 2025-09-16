import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { AuthenticCrazyflie } from '../drones/authentic-crazyflie.js';

// Expose THREE for classes relying on window.THREE
try { window.THREE = THREE; } catch (_) {}

const appEl = document.getElementById('app');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.setClearColor(0x0b1220, 1);
appEl.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(6, 4, 6);
camera.lookAt(0, 0, 0);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 1.5, 0);
controls.update();

// Lights
scene.add(new THREE.AmbientLight(0x334466, 0.8));
const sun = new THREE.DirectionalLight(0xdbeafe, 0.9);
sun.position.set(10, 15, 8);
sun.castShadow = true;
scene.add(sun);

// Ground
const g = new THREE.PlaneGeometry(40, 40);
const m = new THREE.MeshStandardMaterial({ color: 0x19324a, roughness: 0.95 });
const ground = new THREE.Mesh(g, m);
ground.receiveShadow = true;
ground.rotation.x = -Math.PI / 2;
scene.add(ground);

// Create one drone — start near ground for visible lift
const drone = new AuthenticCrazyflie({ id: 'test_cf', position: { x: 0, y: 0.05, z: 0 }, scene });

// Minimal motion control
let running = false;
let rpmOffset = 0; // not used with state machine path

function setSpinning(on) {
  running = on;
  document.getElementById('btn-toggle').textContent = on ? '⏸️ Stop' : '▶️ Start';
}

// Simple et efficace: mouvement + rotation des hélices garantis
const clock = new THREE.Clock();
let t = 0;

function animate() {
  requestAnimationFrame(animate);
  const dt = clock.getDelta();
  t += dt;

  try {
    if (running) {
      // Start a simple forward translation by nudging the target X while flying
      if (drone.state === 'IDLE') {
        // Let state machine switch to TAKEOFF toward targetAltitude
        drone.targetAltitude = 1.5;
      }
      // Push target forward slowly to see translation once airborne
      drone.targetPosition.x += 0.3 * dt;
    } else {
      // Keep the drone hovering using the state machine
      drone.targetAltitude = 1.0;
    }

    // Always run the integrated update path (mission -> control -> physics -> visuals)
    drone.update(dt, [], [], null);
    
    // FALLBACK: Si les propellerGroups ne fonctionnent pas, rotation manuelle
    if (!drone.propellerGroups || drone.propellerGroups.length === 0) {
      console.warn('No propellerGroups, using manual rotation');
      if (drone?.mesh) {
        drone.mesh.children.forEach(child => {
          if (child.name && (child.name.includes('prop') || child.name.includes('propeller'))) {
            const speed = running ? 15.0 * dt : 5.0 * dt; // rotation de base
            child.rotation.z += speed;
          }
        });
      }
    }
    
  } catch (e) {
    console.error('Animation error:', e);
    // ULTIMATE FALLBACK: rotation manuelle de tous les enfants potentiels
    if (drone?.mesh) {
      drone.mesh.children.forEach(child => {
        if (child.name && child.name.includes('prop')) {
          child.rotation.z += 10.0 * dt;
        }
      });
    }
  }

  controls.update();
  renderer.render(scene, camera);
  
  // Update debug info
  updateInfo();
}

animate();

// Debug info function
function updateInfo() {
  const info = document.getElementById('info');
  if (info && drone) {
    const motorRPMs = drone.motors?.map(m => Math.round(m.rpm || 0)).join(', ') || 'N/A';
    const pos = `${drone.position.x.toFixed(2)}, ${drone.position.y.toFixed(2)}, ${drone.position.z.toFixed(2)}`;
    const cmds = `T:${(drone.commands.throttle || 0).toFixed(2)} P:${(drone.commands.pitch || 0).toFixed(2)}`;
    
    info.innerHTML = `
      <strong>État:</strong> ${running ? 'EN MARCHE' : 'ARRÊT'}<br>
      <strong>Position:</strong> ${pos}<br>
      <strong>Commandes:</strong> ${cmds}<br>
      <strong>RPM:</strong> ${motorRPMs}<br>
      <strong>Hélices chargées:</strong> ${drone.propellerGroups?.length || 0}
    `;
  }
}

// HUD + keyboard
const statusEl = document.getElementById('status');
statusEl.textContent = 'Prêt — Start pour spin + translation';

document.getElementById('btn-toggle').addEventListener('click', () => setSpinning(!running));
document.getElementById('btn-reset').addEventListener('click', () => {
  setSpinning(false);
  rpmOffset = 400;
  drone.position.set(0, 2, 0);
});

window.addEventListener('keydown', (e) => {
  if (e.code === 'Space') setSpinning(!running);
  // Arrow keys not used in state-machine-based test
  if (e.code === 'KeyR') {
    setSpinning(false);
    rpmOffset = 400;
    drone.position.set(0, 2, 0);
  }
});

// Resize
window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
