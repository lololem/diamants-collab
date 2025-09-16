/*
 * DIAMANTS - Drone Intelligence for Advanced Mapping and Navigation Through Swarms
 * 
 * Copyright (c) 2025 DIAMANTS Project Contributors
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * DIAMANTS - 3D Visualization Module
 * =====================================
 * Module Three.js pour visualisation 3D temps réel
 */

class DiamantVisualization {
    constructor(containerId) {
        this.containerId = containerId;
        this.container = document.getElementById(containerId);

        if (!this.container) {
            console.error(`Container ${containerId} non trouvé`);
            return;
        }

        // Vérification Three.js
        if (typeof THREE === 'undefined') {
            console.warn('Three.js non chargé, utilisation placeholder');
            this.usePlaceholder = true;
            this.createPlaceholder();
            return;
        }

        this.usePlaceholder = false;
        this.drones = new Map();
        this.trails = new Map();
        this.showTrails = true;
        this.showGrid = true;

        this.init();
    }

    createPlaceholder() {
        /* Créer placeholder en attendant Three.js */
        this.container.innerHTML = `
            <div style="display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100%; background: linear-gradient(45deg, #0d1b2a, #1b263b); color: #90e0ef; text-align: center;">
                <div style="font-size: 3rem; margin-bottom: 1rem;">🚁</div>
                <h3>Visualisation 3D DIAMANTS</h3>
                <p style="margin: 1rem 0; opacity: 0.8;">Module de visualisation temps réel des positions drones</p>
                <div style="display: flex; gap: 1rem; margin-top: 2rem;">
                    <div style="padding: 0.5rem 1rem; background: rgba(0, 180, 216, 0.2); border-radius: 8px; border: 1px solid #00b4d8;">
                        <div style="font-size: 1.5rem; font-weight: bold;">3</div>
                        <div style="font-size: 0.8rem;">Drones Simulés</div>
                    </div>
                    <div style="padding: 0.5rem 1rem; background: rgba(6, 255, 165, 0.2); border-radius: 8px; border: 1px solid #06ffa5;">
                        <div style="font-size: 1.5rem; font-weight: bold;">85%</div>
                        <div style="font-size: 0.8rem;">Score IA</div>
                    </div>
                    <div style="padding: 0.5rem 1rem; background: rgba(255, 183, 0, 0.2); border-radius: 8px; border: 1px solid #ffb700;">
                        <div style="font-size: 1.5rem; font-weight: bold;">67%</div>
                        <div style="font-size: 0.8rem;">Couverture</div>
                    </div>
                </div>
                <p style="margin-top: 2rem; font-size: 0.9rem; opacity: 0.6;">
                    📦 Three.js sera chargé automatiquement pour la visualisation 3D complète
                </p>
            </div>
        `;

        // Simuler données pour demo
        this.simulateData();
    }

    simulateData() {
        /** Simuler données de drones pour demo */
        const droneData = [
            { drone_id: 'drone_1', position: { x: 1.5, y: 2.0, z: 1.2 } },
            { drone_id: 'drone_2', position: { x: -1.0, y: 1.5, z: 0.8 } },
            { drone_id: 'drone_3', position: { x: 0.5, y: -1.8, z: 1.5 } }
        ];

        // Mettre à jour positions de façon cyclique
        let index = 0;
        setInterval(() => {
            const drone = droneData[index % droneData.length];

            // Ajouter variation aléatoire
            const variation = 0.2;
            drone.position.x += (Math.random() - 0.5) * variation;
            drone.position.y += (Math.random() - 0.5) * variation;
            drone.position.z += (Math.random() - 0.5) * variation * 0.5;

            // Simuler mise à jour
            console.log(`🚁 Position ${drone.drone_id}:`, drone.position);

            index++;
        }, 2000);
    }

    init() {
        /** Initialiser scene Three.js */
        console.log('🌐 Initialisation visualisation 3D Three.js');

        // Scene, camera, renderer
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x061621);

        this.camera = new THREE.PerspectiveCamera(
            75,
            this.container.clientWidth / this.container.clientHeight,
            0.1,
            1000
        );

        this.renderer = new THREE.WebGLRenderer({ antialias: true });
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

        // Retirer loading overlay
        const loading = this.container.querySelector('.loading-overlay');
        if (loading) {
            loading.remove();
        }

        this.container.appendChild(this.renderer.domElement);

        // Setup scene
        this.setupLighting();
        this.setupGrid();
        this.setupCamera();

        // Controls
        if (typeof THREE.OrbitControls !== 'undefined') {
            this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
            this.controls.enableDamping = true;
            this.controls.dampingFactor = 0.05;
        }

        // Resize handler
        window.addEventListener('resize', () => this.onWindowResize());

        // Animation loop
        this.animate();

        console.log('✅ Visualisation 3D initialisée');
    }

    setupLighting() {
        /** Configuration éclairage */
        // Lumière ambiante
        const ambientLight = new THREE.AmbientLight(0x404040, 0.4);
        this.scene.add(ambientLight);

        // Lumière directionnelle
        const directionalLight = new THREE.DirectionalLight(0x00b4d8, 1);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        this.scene.add(directionalLight);

        // Lumière ponctuelle pour les drones
        const pointLight = new THREE.PointLight(0x90e0ef, 0.8, 10);
        pointLight.position.set(0, 5, 0);
        this.scene.add(pointLight);
    }

    setupGrid() {
        /** Créer grille de référence */
        const gridHelper = new THREE.GridHelper(20, 20, 0x415a77, 0x1b263b);
        this.gridHelper = gridHelper;
        this.scene.add(gridHelper);
    }

    setupCamera() {
        /** Position initiale caméra */
        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
    }

    updateDronePosition(droneData) {
        /** Mettre à jour position drone */
        if (this.usePlaceholder) return;

        const { drone_id, position } = droneData;

        if (!this.drones.has(drone_id)) {
            this.createDrone(drone_id);
        }

        const drone = this.drones.get(drone_id);
        if (drone) {
            drone.position.set(position.x, position.z, position.y);

            // Mise à jour traînée
            if (this.showTrails) {
                this.updateTrail(drone_id, position);
            }
        }
    }

    createDrone(droneId) {
        /** Créer représentation 3D drone */
        const droneGroup = new THREE.Group();

        // Corps principal
        const bodyGeometry = new THREE.BoxGeometry(0.2, 0.05, 0.2);
        const bodyMaterial = new THREE.MeshLambertMaterial({ color: 0x00b4d8 });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        body.castShadow = true;
        droneGroup.add(body);

        // Hélices
        for (let i = 0; i < 4; i++) {
            const propGeometry = new THREE.CylinderGeometry(0.05, 0.05, 0.01, 8);
            const propMaterial = new THREE.MeshLambertMaterial({ color: 0x90e0ef });
            const prop = new THREE.Mesh(propGeometry, propMaterial);

            const angle = (i / 4) * Math.PI * 2;
            prop.position.set(
                Math.cos(angle) * 0.12,
                0.03,
                Math.sin(angle) * 0.12
            );

            droneGroup.add(prop);
        }

        // Label
        this.addDroneLabel(droneGroup, droneId);

        this.drones.set(droneId, droneGroup);
        this.scene.add(droneGroup);

        console.log(`🚁 Drone ${droneId} créé`);
    }

    addDroneLabel(droneGroup, droneId) {
        /** Ajouter label texte au drone */
        // Simplified label - canvas texture would be better
        const labelGeometry = new THREE.PlaneGeometry(0.3, 0.1);
        const labelMaterial = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: 0.8
        });
        const label = new THREE.Mesh(labelGeometry, labelMaterial);
        label.position.y = 0.15;
        droneGroup.add(label);
    }

    updateTrail(droneId, position) {
        /** Mettre à jour traînée drone */
        if (!this.trails.has(droneId)) {
            this.createTrail(droneId);
        }

        const trail = this.trails.get(droneId);
        // Ajouter point à la traînée
        // Implementation simplifiée
    }

    createTrail(droneId) {
        /** Créer traînée pour drone */
        const trailMaterial = new THREE.LineBasicMaterial({
            color: 0x00b4d8,
            transparent: true,
            opacity: 0.6
        });

        const trailGeometry = new THREE.BufferGeometry();
        const trail = new THREE.Line(trailGeometry, trailMaterial);

        this.trails.set(droneId, trail);
        this.scene.add(trail);
    }

    animate() {
        /** Boucle animation */
        requestAnimationFrame(() => this.animate());

        if (this.controls) {
            this.controls.update();
        }

        // Animation hélices
        this.drones.forEach(drone => {
            drone.children.forEach((child, index) => {
                if (index > 0 && index < 5) { // Hélices
                    child.rotation.y += 0.5;
                }
            });
        });

        this.renderer.render(this.scene, this.camera);
    }

    resetCamera() {
        /** Reset position caméra */
        if (this.usePlaceholder) return;

        this.camera.position.set(5, 5, 5);
        this.camera.lookAt(0, 0, 0);
        if (this.controls) {
            this.controls.reset();
        }
    }

    toggleTrails() {
        /** Basculer affichage traînées */
        this.showTrails = !this.showTrails;

        this.trails.forEach(trail => {
            trail.visible = this.showTrails;
        });

        console.log(`✨ Traînées: ${this.showTrails ? 'ON' : 'OFF'}`);
    }

    toggleGrid() {
        /** Basculer affichage grille */
        if (this.usePlaceholder) return;

        this.showGrid = !this.showGrid;
        if (this.gridHelper) {
            this.gridHelper.visible = this.showGrid;
        }

        console.log(`⚏ Grille: ${this.showGrid ? 'ON' : 'OFF'}`);
    }

    onWindowResize() {
        /** Gestion redimensionnement */
        if (this.usePlaceholder) return;

        this.camera.aspect = this.container.clientWidth / this.container.clientHeight;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.container.clientWidth, this.container.clientHeight);
    }
}

// Export global
window.DiamantVisualization = DiamantVisualization;