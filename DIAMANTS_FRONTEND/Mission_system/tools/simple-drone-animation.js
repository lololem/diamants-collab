/**
 * Simple THREE.js Drone Animation System
 * Solution simple et fonctionnelle pour animer des drones
 */

class SimpleDroneAnimator {
    constructor(scene) {
        this.scene = scene;
        this.drones = [];
        this.isAnimating = false;
        this.animationId = null;
        this.startTime = Date.now();
    }

    // Trouver ou créer les drones dans la scène
    initializeDrones() {
        console.log('Initializing drones...');
        
        // Chercher des drones existants
        this.findExistingDrones();
        
        // Si pas de drones, en créer quelques-uns pour le test
        if (this.drones.length === 0) {
            this.createTestDrones();
        }
        
        console.log(`Initialized ${this.drones.length} drones`);
        return this.drones.length;
    }

    findExistingDrones() {
        this.scene.traverse((object) => {
            if (this.isDroneObject(object)) {
                this.drones.push({
                    mesh: object,
                    originalPosition: object.position.clone(),
                    targetPosition: object.position.clone(),
                    velocity: new THREE.Vector3(),
                    id: this.drones.length
                });
            }
        });
    }

    isDroneObject(object) {
        if (!object.isMesh) return false;
        
        const name = object.name.toLowerCase();
        return (
            name.includes('drone') ||
            name.includes('crazyflie') ||
            name.includes('quad') ||
            object.userData?.type === 'drone' ||
            (object.geometry && object.material && name !== 'ground' && name !== 'floor')
        );
    }

    createTestDrones() {
        console.log('Creating test drones...');
        
        for (let i = 0; i < 4; i++) {
            // Géométrie simple pour les drones
            const geometry = new THREE.BoxGeometry(0.8, 0.2, 0.8);
            const material = new THREE.MeshBasicMaterial({ 
                color: new THREE.Color().setHSL(i / 4, 0.8, 0.5) 
            });
            
            const drone = new THREE.Mesh(geometry, material);
            drone.name = `SimpleDrone_${i}`;
            drone.position.set(i * 2 - 3, 2, 0);
            
            this.scene.add(drone);
            
            this.drones.push({
                mesh: drone,
                originalPosition: drone.position.clone(),
                targetPosition: drone.position.clone(),
                velocity: new THREE.Vector3(),
                id: i
            });
        }
    }

    // Animation en cercle simple
    startCircleAnimation() {
        console.log('Starting circle animation...');
        
        if (this.drones.length === 0) {
            console.warn('No drones to animate');
            return false;
        }

        this.isAnimating = true;
        this.startTime = Date.now();
        this.animate();
        
        return true;
    }

    animate() {
        if (!this.isAnimating) return;

        const elapsed = (Date.now() - this.startTime) / 1000;
        const radius = 6;
        const speed = 0.5;

        // Animer chaque drone en cercle
        this.drones.forEach((droneData, i) => {
            const angle = elapsed * speed + (i * 2 * Math.PI / this.drones.length);
            
            // Position en cercle
            const x = Math.cos(angle) * radius;
            const z = Math.sin(angle) * radius;
            const y = 3 + Math.sin(elapsed * 2 + i) * 0.5; // Oscillation verticale
            
            droneData.mesh.position.set(x, y, z);
            
            // Rotation pour pointer dans la direction du mouvement
            droneData.mesh.rotation.y = angle + Math.PI / 2;
            droneData.mesh.rotation.z = Math.sin(elapsed * 3 + i) * 0.1; // Léger roulis
        });

        this.animationId = requestAnimationFrame(() => this.animate());
    }

    stop() {
        console.log('Stopping animation...');
        this.isAnimating = false;
        if (this.animationId) {
            cancelAnimationFrame(this.animationId);
            this.animationId = null;
        }
    }

    reset() {
        this.stop();
        
        // Remettre les drones à leur position originale
        this.drones.forEach(droneData => {
            droneData.mesh.position.copy(droneData.originalPosition);
            droneData.mesh.rotation.set(0, 0, 0);
        });
        
        console.log('Drones reset to original positions');
    }
}

// Rendre disponible globalement
window.SimpleDroneAnimator = SimpleDroneAnimator;

// Auto-initialisation si la scène est déjà disponible
if (window.scene) {
    window.droneAnimator = new SimpleDroneAnimator(window.scene);
    console.log('SimpleDroneAnimator ready');
}
