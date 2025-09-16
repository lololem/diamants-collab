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
 * DIAMANTS - Enhanced 3D Visualization with Authentic Formulas
 * ==============================================================
 * Intégration des formules DIAMANTS authentiques dans l'interface web existante
 */

class DiamantEnhancedVisualization {
    constructor(containerId) {
        this.containerId = containerId;
        this.container = document.getElementById(containerId);
        
        if (!this.container) {
            console.error(`Container ${containerId} non trouvé`);
            return;
        }
        
        // Vérification Three.js
        if (typeof THREE === 'undefined') {
            console.warn('Three.js non chargé, chargement automatique...');
            this.loadThreeJS().then(() => this.init());
            return;
        }
        
        this.drones = new Map();
        this.trails = new Map();
        this.diamantFields = new Map();
        this.showTrails = true;
        this.showGrid = true;
        this.showDiamantFields = true;
        
        // Configuration des formules DIAMANTS
        this.diamantFormulas = new DiamantFormulasIntegration();
        
        this.init();
    }
    
    async loadThreeJS() {
        return new Promise((resolve, reject) => {
            const script = document.createElement('script');
            script.src = 'https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js';
            script.onload = () => {
                console.log('✅ Three.js chargé');
                resolve();
            };
            script.onerror = reject;
            document.head.appendChild(script);
        });
    }
    
    init() {
        console.log('🎨 Initialisation DIAMANTS Enhanced Visualization');
        
        // Configuration de base
        this.setupScene();
        this.setupRenderer();
        this.setupCamera();
        this.setupControls();
        this.setupLighting();
        this.createEnvironment();
        this.setupDiamantFields();
        
        // Démarrage de l'animation
        this.animate();
        
        // Gestion du redimensionnement
        window.addEventListener('resize', () => this.onWindowResize());
        
        console.log('✅ Visualisation 3D DIAMANTS initialisée');
    }
    
    setupScene() {
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x0a0f1c); // Bleu foncé spatial
        
        // Brouillard pour la profondeur
        this.scene.fog = new THREE.Fog(0x0a0f1c, 10, 50);
    }
    
    setupRenderer() {
        this.renderer = new THREE.WebGLRenderer({ 
            antialias: true,
            alpha: true,
            powerPreference: 'high-performance'
        });
        
        const rect = this.container.getBoundingClientRect();
        this.renderer.setSize(rect.width, rect.height);
        this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
        this.renderer.shadowMap.enabled = true;
        this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
        
        this.container.appendChild(this.renderer.domElement);
    }
    
    setupCamera() {
        const rect = this.container.getBoundingClientRect();
        this.camera = new THREE.PerspectiveCamera(75, rect.width / rect.height, 0.1, 1000);
        this.camera.position.set(12, 8, 12);
        this.camera.lookAt(0, 0, 0);
    }
    
    setupControls() {
        // Contrôles basiques sans dépendance externe
        this.controls = {
            enabled: true,
            rotateSpeed: 0.5,
            zoomSpeed: 1.0,
            panSpeed: 0.8,
            target: new THREE.Vector3(0, 0, 0)
        };
        
        this.setupMouseControls();
    }
    
    setupMouseControls() {
        let isMouseDown = false;
        let mouseX = 0;
        let mouseY = 0;
        
        this.renderer.domElement.addEventListener('mousedown', (event) => {
            isMouseDown = true;
            mouseX = event.clientX;
            mouseY = event.clientY;
        });
        
        this.renderer.domElement.addEventListener('mousemove', (event) => {
            if (!isMouseDown) return;
            
            const deltaX = event.clientX - mouseX;
            const deltaY = event.clientY - mouseY;
            
            // Rotation orbitale simple
            const spherical = new THREE.Spherical();
            spherical.setFromVector3(this.camera.position);
            
            spherical.theta -= deltaX * 0.01;
            spherical.phi += deltaY * 0.01;
            spherical.phi = Math.max(0.1, Math.min(Math.PI - 0.1, spherical.phi));
            
            this.camera.position.setFromSpherical(spherical);
            this.camera.lookAt(this.controls.target);
            
            mouseX = event.clientX;
            mouseY = event.clientY;
        });
        
        this.renderer.domElement.addEventListener('mouseup', () => {
            isMouseDown = false;
        });
        
        this.renderer.domElement.addEventListener('wheel', (event) => {
            const distance = this.camera.position.distanceTo(this.controls.target);
            const newDistance = distance + event.deltaY * 0.01;
            
            if (newDistance > 2 && newDistance < 50) {
                this.camera.position.normalize().multiplyScalar(newDistance);
                this.camera.lookAt(this.controls.target);
            }
        });
    }
    
    setupLighting() {
        // Lumière ambiante douce
        const ambientLight = new THREE.AmbientLight(0x404040, 0.3);
        this.scene.add(ambientLight);
        
        // Lumière directionnelle principale (soleil)
        const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
        directionalLight.position.set(10, 10, 5);
        directionalLight.castShadow = true;
        directionalLight.shadow.mapSize.width = 1024;
        directionalLight.shadow.mapSize.height = 1024;
        this.scene.add(directionalLight);
        
        // Lumière de remplissage bleue (espace)
        const fillLight = new THREE.DirectionalLight(0x87ceeb, 0.2);
        fillLight.position.set(-5, 3, -5);
        this.scene.add(fillLight);
    }
    
    createEnvironment() {
        // Sol avec grille
        if (this.showGrid) {
            const gridHelper = new THREE.GridHelper(20, 20, 0x00ffff, 0x00ffff);
            gridHelper.material.opacity = 0.2;
            gridHelper.material.transparent = true;
            this.scene.add(gridHelper);
        }
        
        // Repères d'axes
        const axesHelper = new THREE.AxesHelper(5);
        this.scene.add(axesHelper);
        
        // Zone de vol (cube transparent)
        const geometry = new THREE.BoxGeometry(15, 8, 15);
        const edges = new THREE.EdgesGeometry(geometry);
        const lineMaterial = new THREE.LineBasicMaterial({ 
            color: 0x00ff88, 
            opacity: 0.4, 
            transparent: true 
        });
        const wireframe = new THREE.LineSegments(edges, lineMaterial);
        wireframe.position.y = 4;
        this.scene.add(wireframe);
        
        this.createEnvironmentObjects();
    }
    
    createEnvironmentObjects() {
        // Obstacles pour navigation
        const obstacles = [
            { pos: [3, 2, 3], size: [1, 4, 1], color: 0xff6b6b },
            { pos: [-4, 1.5, -2], size: [1.5, 3, 1.5], color: 0x4ecdc4 },
            { pos: [0, 1, -5], size: [2, 2, 1], color: 0x45b7d1 }
        ];
        
        obstacles.forEach(obs => {
            const geometry = new THREE.BoxGeometry(...obs.size);
            const material = new THREE.MeshLambertMaterial({ 
                color: obs.color,
                opacity: 0.7,
                transparent: true
            });
            const mesh = new THREE.Mesh(geometry, material);
            mesh.position.set(...obs.pos);
            mesh.castShadow = true;
            mesh.receiveShadow = true;
            this.scene.add(mesh);
        });
    }
    
    setupDiamantFields() {
        // Visualisation des champs DIAMANTS
        this.diamantFieldMesh = this.createDiamantFieldVisualization();
        this.scene.add(this.diamantFieldMesh);
    }
    
    createDiamantFieldVisualization() {
        // Champ de particules pour représenter les forces DIAMANTS
        const particleCount = 1000;
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(particleCount * 3);
        const colors = new Float32Array(particleCount * 3);
        const sizes = new Float32Array(particleCount);
        
        for (let i = 0; i < particleCount; i++) {
            // Positions aléatoires dans l'espace
            positions[i * 3] = (Math.random() - 0.5) * 20;
            positions[i * 3 + 1] = (Math.random() - 0.5) * 10 + 3;
            positions[i * 3 + 2] = (Math.random() - 0.5) * 20;
            
            // Couleurs selon les harmoniques DIAMANTS
            const harmonicIndex = Math.floor(Math.random() * 15);
            const harmonicColor = this.getHarmonicColor(harmonicIndex);
            colors[i * 3] = harmonicColor.r;
            colors[i * 3 + 1] = harmonicColor.g;
            colors[i * 3 + 2] = harmonicColor.b;
            
            sizes[i] = Math.random() * 3 + 1;
        }
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        geometry.setAttribute('size', new THREE.BufferAttribute(sizes, 1));
        
        const material = new THREE.PointsMaterial({
            size: 0.1,
            vertexColors: true,
            transparent: true,
            opacity: 0.6,
            blending: THREE.AdditiveBlending
        });
        
        return new THREE.Points(geometry, material);
    }
    
    getHarmonicColor(harmonicIndex) {
        // Couleurs pour chaque harmonique DIAMANTS (H1-H15)
        const harmonicColors = [
            { r: 1.0, g: 0.2, b: 0.2 }, // H1 - Cohésion (Rouge)
            { r: 0.2, g: 1.0, b: 0.2 }, // H2 - Répulsion (Vert)
            { r: 0.2, g: 0.2, b: 1.0 }, // H3 - Adaptation (Bleu)
            { r: 1.0, g: 1.0, b: 0.2 }, // H4 - Navigation (Jaune)
            { r: 1.0, g: 0.2, b: 1.0 }, // H5 - Évitement (Magenta)
            { r: 0.2, g: 1.0, b: 1.0 }, // H6 - Exploration (Cyan)
            { r: 1.0, g: 0.5, b: 0.0 }, // H7 - Consensus (Orange)
            { r: 0.5, g: 0.0, b: 1.0 }, // H8 - Communication (Violet)
            { r: 0.0, g: 1.0, b: 0.5 }, // H9 - Apprentissage (Vert-bleu)
            { r: 1.0, g: 0.8, b: 0.0 }, // H10 - Synchronisation (Or)
            { r: 0.8, g: 0.0, b: 0.8 }, // H11 - Leadership (Pourpre)
            { r: 0.0, g: 0.8, b: 1.0 }, // H12 - Émergence (Bleu clair)
            { r: 1.0, g: 0.4, b: 0.6 }, // H13 - Optimisation (Rose)
            { r: 0.6, g: 1.0, b: 0.4 }, // H14 - Résilience (Vert clair)
            { r: 0.9, g: 0.9, b: 0.9 }  // H15 - Créativité (Blanc)
        ];
        
        return harmonicColors[harmonicIndex] || { r: 0.5, g: 0.5, b: 0.5 };
    }
    
    /**
     * Méthodes publiques pour interaction avec l'interface web
     */
    updateDroneData(droneData) {
        if (!droneData || !Array.isArray(droneData)) return;
        
        droneData.forEach(drone => {
            this.updateDrone(drone);
        });
        
        // Mise à jour des champs DIAMANTS
        if (this.showDiamantFields) {
            this.updateDiamantFields(droneData);
        }
    }
    
    updateDrone(droneInfo) {
        const { id, position, orientation, status, mission } = droneInfo;
        
        if (!this.drones.has(id)) {
            this.createDrone(id);
        }
        
        const drone = this.drones.get(id);
        
        // Mise à jour position
        if (position) {
            drone.position.set(position.x || 0, position.y || 2, position.z || 0);
            
            // Mise à jour de la traînée
            if (this.showTrails) {
                this.updateTrail(id, drone.position);
            }
        }
        
        // Mise à jour orientation
        if (orientation) {
            drone.rotation.set(
                orientation.pitch || 0,
                orientation.yaw || 0,
                orientation.roll || 0
            );
        }
        
        // Mise à jour couleur selon statut
        this.updateDroneStatus(drone, status, mission);
    }
    
    createDrone(id) {
        // Groupe pour le drone complet
        const droneGroup = new THREE.Group();
        
        // Corps principal (Crazyflie authentique)
        const bodyGeometry = new THREE.BoxGeometry(0.1, 0.03, 0.1);
        const bodyMaterial = new THREE.MeshLambertMaterial({ color: 0x333333 });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        droneGroup.add(body);
        
        // Hélices (4 moteurs)
        const propPositions = [
            { x: 0.031, z: 0.031 },   // M1
            { x: -0.031, z: -0.031 }, // M2
            { x: -0.031, z: 0.031 },  // M3
            { x: 0.031, z: -0.031 }   // M4
        ];
        
        propPositions.forEach((pos, index) => {
            const propGeometry = new THREE.CylinderGeometry(0.02, 0.02, 0.002);
            const propMaterial = new THREE.MeshLambertMaterial({ 
                color: index % 2 === 0 ? 0x00ff00 : 0xff0000 
            });
            const prop = new THREE.Mesh(propGeometry, propMaterial);
            prop.position.set(pos.x, 0.02, pos.z);
            prop.rotation.x = Math.PI / 2;
            droneGroup.add(prop);
        });
        
        // LED de statut
        const ledGeometry = new THREE.SphereGeometry(0.01);
        const ledMaterial = new THREE.MeshBasicMaterial({ 
            color: 0x00ff00,
            emissive: 0x004400
        });
        const led = new THREE.Mesh(ledGeometry, ledMaterial);
        led.position.y = 0.025;
        droneGroup.add(led);
        
        // Label avec ID
        this.createDroneLabel(droneGroup, id);
        
        // Ajout à la scène
        this.scene.add(droneGroup);
        this.drones.set(id, droneGroup);
        
        // Initialisation de la traînée
        if (this.showTrails) {
            this.initTrail(id);
        }
        
        console.log(`🚁 Drone ${id} créé`);
    }
    
    createDroneLabel(droneGroup, id) {
        // Canvas pour le texte
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        canvas.width = 128;
        canvas.height = 64;
        
        context.fillStyle = 'rgba(0, 0, 0, 0.7)';
        context.fillRect(0, 0, canvas.width, canvas.height);
        
        context.fillStyle = '#ffffff';
        context.font = '16px Arial';
        context.textAlign = 'center';
        context.fillText(id, canvas.width / 2, canvas.height / 2 + 6);
        
        const texture = new THREE.CanvasTexture(canvas);
        const material = new THREE.SpriteMaterial({ map: texture });
        const sprite = new THREE.Sprite(material);
        sprite.position.y = 0.1;
        sprite.scale.set(0.5, 0.25, 1);
        
        droneGroup.add(sprite);
    }
    
    updateDroneStatus(drone, status, mission) {
        // Mise à jour LED selon le statut
        const led = drone.children.find(child => 
            child.material && child.material.emissive
        );
        
        if (led) {
            switch (status) {
                case 'active':
                case 'flying':
                    led.material.color.setHex(0x00ff00);
                    led.material.emissive.setHex(0x004400);
                    break;
                case 'warning':
                    led.material.color.setHex(0xffff00);
                    led.material.emissive.setHex(0x444400);
                    break;
                case 'error':
                case 'emergency':
                    led.material.color.setHex(0xff0000);
                    led.material.emissive.setHex(0x440000);
                    break;
                default:
                    led.material.color.setHex(0x666666);
                    led.material.emissive.setHex(0x000000);
            }
        }
    }
    
    initTrail(droneId) {
        const maxTrailPoints = 50;
        const geometry = new THREE.BufferGeometry();
        const positions = new Float32Array(maxTrailPoints * 3);
        
        geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        
        const material = new THREE.LineBasicMaterial({
            color: 0x00ffff,
            opacity: 0.6,
            transparent: true
        });
        
        const trail = new THREE.Line(geometry, material);
        this.scene.add(trail);
        
        this.trails.set(droneId, {
            mesh: trail,
            points: [],
            maxPoints: maxTrailPoints
        });
    }
    
    updateTrail(droneId, position) {
        const trail = this.trails.get(droneId);
        if (!trail) return;
        
        // Ajout du nouveau point
        trail.points.push(position.clone());
        
        // Limitation du nombre de points
        if (trail.points.length > trail.maxPoints) {
            trail.points.shift();
        }
        
        // Mise à jour de la géométrie
        const positions = trail.mesh.geometry.attributes.position.array;
        for (let i = 0; i < trail.points.length; i++) {
            const point = trail.points[i];
            positions[i * 3] = point.x;
            positions[i * 3 + 1] = point.y;
            positions[i * 3 + 2] = point.z;
        }
        
        trail.mesh.geometry.attributes.position.needsUpdate = true;
        trail.mesh.geometry.setDrawRange(0, trail.points.length);
    }
    
    updateDiamantFields(droneData) {
        // Animation des particules selon les formules DIAMANTS
        const time = Date.now() * 0.001;
        const positions = this.diamantFieldMesh.geometry.attributes.position.array;
        
        for (let i = 0; i < positions.length; i += 3) {
            // Calcul des forces DIAMANTS simplifiées
            const x = positions[i];
            const z = positions[i + 2];
            
            // Effet d'ondulation basé sur les drones
            let influence = 0;
            droneData.forEach(drone => {
                if (drone.position) {
                    const dx = x - drone.position.x;
                    const dz = z - drone.position.z;
                    const distance = Math.sqrt(dx * dx + dz * dz);
                    influence += Math.exp(-distance * 0.5) * Math.sin(time + distance);
                }
            });
            
            positions[i + 1] += influence * 0.01;
        }
        
        this.diamantFieldMesh.geometry.attributes.position.needsUpdate = true;
    }
    
    /**
     * Contrôles de visualisation
     */
    toggleTrails() {
        this.showTrails = !this.showTrails;
        this.trails.forEach(trail => {
            trail.mesh.visible = this.showTrails;
        });
    }
    
    toggleDiamantFields() {
        this.showDiamantFields = !this.showDiamantFields;
        this.diamantFieldMesh.visible = this.showDiamantFields;
    }
    
    toggleGrid() {
        this.showGrid = !this.showGrid;
        // Rechercher et toggle la grille
        const gridHelper = this.scene.children.find(child => child.type === 'GridHelper');
        if (gridHelper) {
            gridHelper.visible = this.showGrid;
        }
    }
    
    /**
     * Animation principale
     */
    animate() {
        requestAnimationFrame(() => this.animate());
        
        // Animation des hélices
        this.drones.forEach(drone => {
            drone.children.forEach(child => {
                if (child.geometry && child.geometry.type === 'CylinderGeometry') {
                    child.rotation.y += 0.3; // Rotation des hélices
                }
            });
        });
        
        this.renderer.render(this.scene, this.camera);
    }
    
    onWindowResize() {
        const rect = this.container.getBoundingClientRect();
        this.camera.aspect = rect.width / rect.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(rect.width, rect.height);
    }
    
    /**
     * Nettoyage
     */
    destroy() {
        this.drones.clear();
        this.trails.clear();
        
        if (this.renderer) {
            this.renderer.dispose();
            if (this.container.contains(this.renderer.domElement)) {
                this.container.removeChild(this.renderer.domElement);
            }
        }
        
        window.removeEventListener('resize', this.onWindowResize);
    }
}

/**
 * Intégration simplifiée des formules DIAMANTS
 */
class DiamantFormulasIntegration {
    constructor() {
        this.harmonics = new Array(15).fill(0);
        this.lastUpdate = Date.now();
    }
    
    // Simulation simplifiée des 15 harmoniques DIAMANTS
    calculateHarmonics(droneData) {
        const time = Date.now() * 0.001;
        
        if (!droneData || droneData.length === 0) {
            return this.harmonics;
        }
        
        // H1 - Cohésion
        this.harmonics[0] = this.calculateCohesion(droneData);
        
        // H2 - Répulsion
        this.harmonics[1] = this.calculateRepulsion(droneData);
        
        // H3 - Adaptation
        this.harmonics[2] = Math.sin(time * 0.5) * 0.5 + 0.5;
        
        // H4-H15 - Autres harmoniques (simplifiées)
        for (let i = 3; i < 15; i++) {
            this.harmonics[i] = Math.sin(time * (0.1 + i * 0.05)) * 0.3 + 0.7;
        }
        
        return this.harmonics;
    }
    
    calculateCohesion(droneData) {
        if (droneData.length < 2) return 0;
        
        let totalDistance = 0;
        let pairCount = 0;
        
        for (let i = 0; i < droneData.length; i++) {
            for (let j = i + 1; j < droneData.length; j++) {
                const drone1 = droneData[i];
                const drone2 = droneData[j];
                
                if (drone1.position && drone2.position) {
                    const dx = drone1.position.x - drone2.position.x;
                    const dy = drone1.position.y - drone2.position.y;
                    const dz = drone1.position.z - drone2.position.z;
                    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    
                    totalDistance += distance;
                    pairCount++;
                }
            }
        }
        
        if (pairCount === 0) return 0;
        
        const avgDistance = totalDistance / pairCount;
        return Math.max(0, 1 - avgDistance / 10); // Cohésion inverse à la distance
    }
    
    calculateRepulsion(droneData) {
        let minDistance = Infinity;
        
        for (let i = 0; i < droneData.length; i++) {
            for (let j = i + 1; j < droneData.length; j++) {
                const drone1 = droneData[i];
                const drone2 = droneData[j];
                
                if (drone1.position && drone2.position) {
                    const dx = drone1.position.x - drone2.position.x;
                    const dy = drone1.position.y - drone2.position.y;
                    const dz = drone1.position.z - drone2.position.z;
                    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
                    
                    minDistance = Math.min(minDistance, distance);
                }
            }
        }
        
        if (minDistance === Infinity) return 0;
        
        return minDistance < 2 ? 1 - minDistance / 2 : 0; // Répulsion forte si trop proches
    }
}

// Export pour utilisation dans l'interface
window.DiamantEnhancedVisualization = DiamantEnhancedVisualization;
