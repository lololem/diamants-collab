// Test Quokka.js avec TypeScript
// Utilisez Ctrl+Shift+P puis "Quokka.js: Start on Current File"

interface DiamantConfig {
    phi: number;
    sigma: number;
    domainScale: number;
    agentCount: number;
}

interface Vector3D {
    x: number;
    y: number;
    z: number;
}

class DiamantSystemTest {
    private config: DiamantConfig;
    
    constructor(config: DiamantConfig) {
        this.config = config;
    }
    
    calculateGradient(position: Vector3D): Vector3D {
        const h = 0.01;
        
        // Simulation du gradient DIAMANTS ∇(φ+σ)
        const gradX = (this.samplePotential(position.x + h, position.y, position.z) - 
                      this.samplePotential(position.x - h, position.y, position.z)) / (2 * h);
        
        const gradY = (this.samplePotential(position.x, position.y + h, position.z) - 
                      this.samplePotential(position.x, position.y - h, position.z)) / (2 * h);
        
        const gradZ = (this.samplePotential(position.x, position.y, position.z + h) - 
                      this.samplePotential(position.x, position.y, position.z - h)) / (2 * h);
        
        return { x: gradX, y: gradY, z: gradZ };
    }
    
    private samplePotential(x: number, y: number, z: number): number {
        // Potentiel φ (attractif) + σ (répulsif)
        const phi = this.config.phi * Math.exp(-(x*x + y*y + z*z) / 4);
        const sigma = this.config.sigma * Math.exp(-(x*x + y*y + z*z) / 2);
        return phi + sigma;
    }
    
    calculateIntelligence(gradient: Vector3D): number {
        // I(t) = |∇(φ+σ)|
        return Math.sqrt(gradient.x * gradient.x + gradient.y * gradient.y + gradient.z * gradient.z);
    }
}

// Test en temps réel
const config: DiamantConfig = {
    phi: 1.5,
    sigma: 1.2, 
    domainScale: 3.0,
    agentCount: 100
}; // ?

const system = new DiamantSystemTest(config);

// Test avec différentes positions
const position1: Vector3D = { x: 1, y: 1, z: 0 };
const gradient1 = system.calculateGradient(position1); // ?
const intelligence1 = system.calculateIntelligence(gradient1); // ?

const position2: Vector3D = { x: 0, y: 0, z: 0 };
const gradient2 = system.calculateGradient(position2); // ?
const intelligence2 = system.calculateIntelligence(gradient2); // ?

console.log('🧠 Test DIAMANTS avec TypeScript:');
console.log(`Position (1,1,0) - Intelligence: ${intelligence1.toFixed(4)}`);
console.log(`Position (0,0,0) - Intelligence: ${intelligence2.toFixed(4)}`);

// Test de performance
const startTime = performance.now();
const testPositions: Vector3D[] = [];
for(let i = 0; i < 10; i++) {
    testPositions.push({
        x: Math.random() * 2 - 1,
        y: Math.random() * 2 - 1, 
        z: Math.random() * 2 - 1
    });
}

const intelligenceResults = testPositions.map(pos => {
    const grad = system.calculateGradient(pos);
    return system.calculateIntelligence(grad);
}); // ?

const endTime = performance.now();
const duration = endTime - startTime; // ?

console.log(`⚡ Calculé ${testPositions.length} intelligences en ${duration.toFixed(2)}ms`);
console.log('Résultats:', intelligenceResults.map(r => r.toFixed(3)));

console.log('✅ Test Quokka.js + TypeScript réussi !');
