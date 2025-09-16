/**
 * DIAMANTS V3 - Crazyflie Visuel (Frontend Only)
 * ==============================================
 * Version allégée pour rendu 3D uniquement avec améliorations visuelles
 */

export class CrazyflieVisual {
    constructor(id, scene, position = { x: 0, y: 3, z: 0 }, THREE = null) {
        this.id = id;
        this.scene = scene;
        this.position = position;
    this.THREE = THREE || (typeof window !== 'undefined' ? window.THREE : null);
        
        // Références Three.js
        this.mesh = null;
        this.propellers = [];
        this.trail = null;
        this.batteryIndicator = null;
        this.statusText = null;
        
        // État visuel
        this.isVisible = true;
        this.isFlying = false;
        this.battery = 4.0;
        this.lastPosition = { x: 0, y: 0, z: 0 };
        
        // Effets visuels
        this.propellerRotation = [0, 0, 0, 0];
        this.ledBlinkTimer = 0;
        
        this.createMesh();
        this.createTrail();
        this.createBatteryIndicator();
        this.createStatusDisplay();
    }
    
    createMesh() {
        if (!this.THREE) {
            console.error('THREE.js non disponible');
            return;
        }
        
        const THREE = this.THREE;
        
        // Groupe principal
        this.mesh = new THREE.Group();
        
        // Corps du drone (authentique Crazyflie)
        const bodyGeometry = new THREE.BoxGeometry(0.092, 0.092, 0.029);
        const bodyMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x2c3e50,
            transparent: true,
            opacity: 0.9
        });
        const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
        this.mesh.add(body);
        
        // 4 moteurs avec hélices
        this.createPropellers();
        
        // LED de statut
        this.createStatusLEDs();
        
        // Position initiale
        this.mesh.position.set(this.position.x, this.position.y, this.position.z);
        
        // Ajout à la scène
        if (this.scene) {
            this.scene.add(this.mesh);
        }
    }
    
    createPropellers() {
        if (!this.THREE) return;
        const THREE = this.THREE;
        
        // Positions moteurs Crazyflie (authentiques)
        const motorPositions = [
            { x: 0.031, y: 0, z: 0.031 },   // Front Right
            { x: -0.031, y: 0, z: 0.031 },  // Front Left  
            { x: -0.031, y: 0, z: -0.031 }, // Rear Left
            { x: 0.031, y: 0, z: -0.031 }   // Rear Right
        ];
        
        motorPositions.forEach((pos, index) => {
            // Moteur
            const motorGeometry = new THREE.CylinderGeometry(0.003, 0.003, 0.006, 8);
            const motorMaterial = new THREE.MeshPhongMaterial({ color: 0x34495e });
            const motor = new THREE.Mesh(motorGeometry, motorMaterial);
            motor.position.set(pos.x, pos.z, pos.y);
            this.mesh.add(motor);
            
            // Hélice
            const propGeometry = new THREE.PlaneGeometry(0.046, 0.006);
            const propMaterial = new THREE.MeshPhongMaterial({ 
                color: 0x95a5a6,
                transparent: true,
                opacity: this.isFlying ? 0.3 : 0.8
            });
            const propeller = new THREE.Mesh(propGeometry, propMaterial);
            propeller.position.set(pos.x, pos.z + 0.005, pos.y);
            propeller.rotation.x = Math.PI / 2;
            
            this.mesh.add(propeller);
            this.propellers.push(propeller);
        });
    }
    
    createStatusLEDs() {
        if (!this.THREE) return;
        const THREE = this.THREE;
        
        // LED avant (position authentique)
        const ledGeometry = new THREE.SphereGeometry(0.002, 8, 8);
        const ledMaterial = new THREE.MeshPhongMaterial({ 
            color: 0x00ff00,
            emissive: 0x004400
        });
        const led = new THREE.Mesh(ledGeometry, ledMaterial);
        led.position.set(0, 0.005, 0.040);
        this.mesh.add(led);
        
        this.statusLED = led;
    }
    
    createTrail() {
        if (!this.THREE) return;
        const THREE = this.THREE;
        
        // Trail de trajectoire
        const trailGeometry = new THREE.BufferGeometry();
        const trailMaterial = new THREE.LineBasicMaterial({ 
            color: 0x3498db,
            transparent: true,
            opacity: 0.6
        });
        
        this.trail = new THREE.Line(trailGeometry, trailMaterial);
        this.trailPoints = [];
        
        if (this.scene) {
            this.scene.add(this.trail);
        }
    }
    
    // === MISE À JOUR ÉTAT ===
    
    updateFromROS(rosState) {
        if (!rosState || !this.mesh) return;
        
        // Sauvegarder ancienne position pour calcul vitesse
        this.lastPosition = {
            x: this.mesh.position.x,
            y: this.mesh.position.y,
            z: this.mesh.position.z
        };
        
        // Position
        if (rosState.position) {
            this.mesh.position.set(
                rosState.position.x,
                rosState.position.z, // ROS Z → Three.js Y
                rosState.position.y
            );
            
            // Ajouter point au trail
            this.addTrailPoint(this.mesh.position);
        }
        
        // Orientation
        if (rosState.orientation) {
            this.mesh.quaternion.set(
                rosState.orientation.x,
                rosState.orientation.z,
                rosState.orientation.y,
                rosState.orientation.w
            );
        }
        
        // État vol
        const wasFlying = this.isFlying;
        this.isFlying = rosState.connected || false;
        this.battery = rosState.battery || 4.0;
        
        // Mise à jour statut si changement
        if (wasFlying !== this.isFlying) {
            this.updateStatusText(this.isFlying ? 'FLYING' : 'STANDBY');
        }
        
        // Calcul vitesse pour inclinaison
        this.updateFlightDynamics();
        
        // Mise à jour visuelle
        this.updateVisuals();
    }
    
    updateFlightDynamics() {
        if (!this.mesh) return;
        
        // Calcul vitesse
        const velocity = {
            x: this.mesh.position.x - this.lastPosition.x,
            y: this.mesh.position.y - this.lastPosition.y,
            z: this.mesh.position.z - this.lastPosition.z
        };
        
        const speed = Math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2);
        
        // Inclinaison selon direction (comportement authentique)
        if (this.isFlying && speed > 0.01) {
            const maxTilt = Math.PI / 8; // 22.5 degrés max
            
            // Inclinaison latérale selon vitesse X
            this.mesh.rotation.z = -velocity.x * maxTilt * 20;
            
            // Inclinaison avant/arrière selon vitesse Z
            this.mesh.rotation.x = velocity.z * maxTilt * 20;
            
            // Limiter inclinaisons
            this.mesh.rotation.x = Math.max(-maxTilt, Math.min(maxTilt, this.mesh.rotation.x));
            this.mesh.rotation.z = Math.max(-maxTilt, Math.min(maxTilt, this.mesh.rotation.z));
        } else {
            // Retour position horizontale
            this.mesh.rotation.x *= 0.9;
            this.mesh.rotation.z *= 0.9;
        }
    }
    
    updateVisuals() {
        if (!this.mesh) return;
        
        const deltaTime = 0.016; // ~60fps
        
        // Animation hélices avec rotation différentielle
        if (this.isFlying) {
            this.propellers.forEach((prop, index) => {
                // Vitesses différentes par moteur (CW/CCW authentique Crazyflie)
                const baseSpeed = 0.8;
                const speeds = [baseSpeed, -baseSpeed, baseSpeed, -baseSpeed]; // Rotation alternée
                
                this.propellerRotation[index] += speeds[index];
                prop.rotation.z = this.propellerRotation[index];
                
                // Effet flou pour rotation rapide
                prop.material.opacity = 0.2;
            });
        } else {
            this.propellers.forEach((prop, index) => {
                // Arrêt progressif
                this.propellerRotation[index] *= 0.95;
                prop.rotation.z = this.propellerRotation[index];
                prop.material.opacity = 0.8;
            });
        }
        
        // LED de statut avec effet de clignotement
        if (this.statusLED) {
            this.ledBlinkTimer += deltaTime;
            
            if (this.isFlying) {
                // Vert clignotant en vol
                const intensity = 0.7 + 0.3 * Math.sin(this.ledBlinkTimer * 8);
                this.statusLED.material.color.setHex(0x00ff00);
                this.statusLED.material.emissive.setHex(0x004400);
                this.statusLED.material.emissive.multiplyScalar(intensity);
            } else {
                // Rouge fixe au sol
                this.statusLED.material.color.setHex(0xff0000);
                this.statusLED.material.emissive.setHex(0x440000);
            }
        }
        
        // Mise à jour indicateur batterie
        this.updateBatteryIndicator();
        
        // Transparence selon batterie
        if (this.mesh.children[0]) { // Corps
            this.mesh.children[0].material.opacity = Math.max(0.3, this.battery / 4.0);
        }
        
        // Vibration légère en vol
        if (this.isFlying) {
            const vibration = 0.002;
            this.mesh.position.x += (Math.random() - 0.5) * vibration;
            this.mesh.position.y += (Math.random() - 0.5) * vibration;
            this.mesh.position.z += (Math.random() - 0.5) * vibration;
        }
    }
    
    updateBatteryIndicator() {
        if (!this.batteryIndicator) return;
        
        // Couleur selon niveau batterie
        const batteryLevel = this.battery / 4.0;
        
        if (batteryLevel > 0.6) {
            this.batteryIndicator.material.color.setHex(0x00ff00); // Vert
        } else if (batteryLevel > 0.3) {
            this.batteryIndicator.material.color.setHex(0xffaa00); // Orange
        } else {
            this.batteryIndicator.material.color.setHex(0xff0000); // Rouge
        }
        
        // Largeur selon niveau
        this.batteryIndicator.scale.x = batteryLevel;
    }
    
    addTrailPoint(position) {
        this.trailPoints.push(position.clone());
        
        // Limiter longueur trail
        if (this.trailPoints.length > 100) {
            this.trailPoints.shift();
        }
        
        // Mettre à jour géométrie trail
        if (this.trail && this.THREE) {
            const points = this.trailPoints.map(p => new this.THREE.Vector3(p.x, p.y, p.z));
            this.trail.geometry.setFromPoints(points);
        }
    }
    
    createBatteryIndicator() {
        if (!this.THREE) return;
        const THREE = this.THREE;
        
        // Barre de batterie au-dessus du drone
        const batteryGeometry = new THREE.PlaneGeometry(0.06, 0.01);
        const batteryMaterial = new THREE.MeshBasicMaterial({ 
            color: 0x00ff00,
            transparent: true,
            opacity: 0.8
        });
        
        this.batteryIndicator = new THREE.Mesh(batteryGeometry, batteryMaterial);
        this.batteryIndicator.position.set(0, 0.08, 0);
        this.batteryIndicator.rotation.x = -Math.PI / 2;
        
        if (this.mesh) {
            this.mesh.add(this.batteryIndicator);
        }
    }
    
    createStatusDisplay() {
        if (!this.THREE) return;
        const THREE = this.THREE;
        
        // Créer texture pour texte de statut
        const canvas = document.createElement('canvas');
        canvas.width = 128;
        canvas.height = 32;
        const context = canvas.getContext('2d');
        
        const texture = new THREE.CanvasTexture(canvas);
        const material = new THREE.SpriteMaterial({ map: texture, transparent: true });
        
        this.statusText = new THREE.Sprite(material);
        this.statusText.position.set(0, 0.12, 0);
        this.statusText.scale.set(0.1, 0.025, 1);
        
        if (this.mesh) {
            this.mesh.add(this.statusText);
        }
        
        this.updateStatusText('STANDBY');
    }
    
    updateStatusText(status) {
        if (!this.statusText) return;
        
        const canvas = this.statusText.material.map.image;
        const context = canvas.getContext('2d');
        
        // Effacer canvas
        context.clearRect(0, 0, canvas.width, canvas.height);
        
        // Style texte
        context.font = 'bold 12px Arial';
        context.fillStyle = this.isFlying ? '#00ff00' : '#ffaa00';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        
        // Dessiner texte
        context.fillText(`${this.id.toUpperCase()}: ${status}`, canvas.width / 2, canvas.height / 2);
        
        // Mettre à jour texture
        this.statusText.material.map.needsUpdate = true;
    }
    
    setVisible(visible) {
        this.isVisible = visible;
        if (this.mesh) {
            this.mesh.visible = visible;
        }
        if (this.trail) {
            this.trail.visible = visible;
        }
    }
    
    setHighlight(highlight) {
        if (!this.mesh || !this.mesh.children[0]) return;
        
        if (highlight) {
            this.mesh.children[0].material.emissive.setHex(0x444444);
        } else {
            this.mesh.children[0].material.emissive.setHex(0x000000);
        }
    }
    
    dispose() {
        if (this.mesh && this.scene) {
            this.scene.remove(this.mesh);
        }
        if (this.trail && this.scene) {
            this.scene.remove(this.trail);
        }
        
        // Nettoyage textures
        if (this.statusText && this.statusText.material.map) {
            this.statusText.material.map.dispose();
        }
    }
}
