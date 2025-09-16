// 🚀 DÉMONSTRATION AVANCÉE QUOKKA + DIAMANTS V3
// Ctrl+Shift+P → "Quokka.js: Start on Current File" pour voir la magie !

console.log('🎯 DIAMANTS V3 - Démonstration Quokka Interactive');

// 🔧 Configuration dynamique - modifiez ces valeurs et voyez le résultat instantané !
const CONFIG = {
    phi: 2.0,           // ← Changez cette valeur !
    sigma: 1.5,         // ← Changez cette valeur !
    timeStep: 0.1,
    spatialRes: 0.2,
    agents: 50
}; // ?

// 📐 Classe Agent DIAMANTS avec visualisation temps réel
class DiamantAgent {
    constructor(id, x = 0, y = 0, z = 0) {
        this.id = id;
        this.position = { x, y, z };
        this.velocity = { x: 0, y: 0, z: 0 };
        this.phi = 0;
        this.sigma = 0;
        this.gradient = { x: 0, y: 0, z: 0 };
        this.intelligence = 0;
        this.trajectory = [{ ...this.position, intelligence: 0 }];
    }
    
    // Calcul du potentiel total en temps réel
    samplePotential(x, y, z, t) {
        const r2 = x*x + y*y + z*z;
        
        // φ (attractif) avec oscillation temporelle
        const phi = CONFIG.phi * Math.exp(-r2 / 3) * (1 + 0.2 * Math.sin(t * 0.3));
        
        // σ (répulsif) avec oscillation décalée
        const sigma = CONFIG.sigma * Math.exp(-r2 / 1.5) * (1 + 0.3 * Math.cos(t * 0.5));
        
        return phi + sigma;
    }
    
    // Gradient ∇(φ+σ) par différentiation numérique
    calculateGradient(t) {
        const h = 0.01;
        const {x, y, z} = this.position;
        
        this.gradient.x = (this.samplePotential(x+h, y, z, t) - this.samplePotential(x-h, y, z, t)) / (2*h);
        this.gradient.y = (this.samplePotential(x, y+h, z, t) - this.samplePotential(x, y-h, z, t)) / (2*h);
        this.gradient.z = (this.samplePotential(x, y, z+h, t) - this.samplePotential(x, y, z-h, t)) / (2*h);
        
        return this.gradient;
    }
    
    // Intelligence I = |∇(φ+σ)|
    calculateIntelligence() {
        const {x, y, z} = this.gradient;
        this.intelligence = Math.sqrt(x*x + y*y + z*z);
        return this.intelligence;
    }
    
    // Mise à jour complète de l'agent
    update(t, dt = 0.1) {
        this.calculateGradient(t);
        this.calculateIntelligence();
        
        // Mouvement basé sur le gradient (force = -∇U)
        const force = 0.1;
        this.velocity.x += -this.gradient.x * force * dt;
        this.velocity.y += -this.gradient.y * force * dt;
        this.velocity.z += -this.gradient.z * force * dt;
        
        // Friction
        this.velocity.x *= 0.95;
        this.velocity.y *= 0.95;
        this.velocity.z *= 0.95;
        
        // Mise à jour position
        this.position.x += this.velocity.x * dt;
        this.position.y += this.velocity.y * dt;
        this.position.z += this.velocity.z * dt;
        
        // Enregistrer trajectoire
        this.trajectory.push({
            x: this.position.x,
            y: this.position.y, 
            z: this.position.z,
            intelligence: this.intelligence,
            time: t
        });
        
        // Limiter taille trajectoire
        if(this.trajectory.length > 20) {
            this.trajectory.shift();
        }
        
        return this;
    }
}

// 🌟 Création du système multi-agents
let agents = [];
for(let i = 0; i < CONFIG.agents; i++) {
    const angle = (i / CONFIG.agents) * 2 * Math.PI;
    const radius = 0.5 + Math.random() * 1.5;
    const x = radius * Math.cos(angle);
    const y = radius * Math.sin(angle);
    const z = (Math.random() - 0.5) * 0.5;
    
    agents.push(new DiamantAgent(i, x, y, z));
}

console.log(`🎮 ${agents.length} agents DIAMANTS créés`);

// 📊 Simulation temps réel - modifiez la boucle et voyez !
let simulationData = [];
let currentTime = 0;

for(let step = 0; step < 20; step++) {  // ← Changez le nombre d'étapes !
    currentTime += CONFIG.timeStep;
    
    // Mise à jour de tous les agents
    agents.forEach(agent => agent.update(currentTime, CONFIG.timeStep));
    
    // Calcul des statistiques globales
    const intelligences = agents.map(a => a.intelligence);
    const totalIntelligence = intelligences.reduce((sum, i) => sum + i, 0);
    const avgIntelligence = totalIntelligence / agents.length;
    const maxIntelligence = Math.max(...intelligences);
    const minIntelligence = Math.min(...intelligences);
    
    // Calcul du centre de masse
    const centerOfMass = {
        x: agents.reduce((sum, a) => sum + a.position.x, 0) / agents.length,
        y: agents.reduce((sum, a) => sum + a.position.y, 0) / agents.length,
        z: agents.reduce((sum, a) => sum + a.position.z, 0) / agents.length
    };
    
    simulationData.push({
        time: currentTime,
        totalIntelligence,
        avgIntelligence,
        maxIntelligence,
        minIntelligence,
        centerOfMass,
        agentCount: agents.length
    });
}

// 📈 Résultats de la simulation en temps réel
simulationData; // ? ← Voyez l'évolution complète !

// Statistiques finales
const finalStats = simulationData[simulationData.length - 1]; // ?
console.log(`📊 Statistiques finales (t=${finalStats.time.toFixed(1)}):`);
console.log(`   Intelligence totale: ${finalStats.totalIntelligence.toFixed(4)}`);
console.log(`   Intelligence moyenne: ${finalStats.avgIntelligence.toFixed(6)}`);
console.log(`   Intelligence max: ${finalStats.maxIntelligence.toFixed(6)}`);

// 🎯 Agent le plus intelligent
const smartestAgent = agents.reduce((best, agent) => 
    agent.intelligence > best.intelligence ? agent : best
); // ?

console.log(`🧠 Agent le plus intelligent: #${smartestAgent.id}`);
console.log(`   Position: (${smartestAgent.position.x.toFixed(3)}, ${smartestAgent.position.y.toFixed(3)}, ${smartestAgent.position.z.toFixed(3)})`);
console.log(`   Intelligence: ${smartestAgent.intelligence.toFixed(6)}`);

// 🔄 Analyse de trajectoire de l'agent le plus intelligent
smartestAgent.trajectory; // ? ← Voyez sa trajectoire complète !

// ⚡ Test de performance avec Quokka
const perfStart = performance.now();
const performanceTest = agents.map(agent => {
    agent.calculateGradient(currentTime);
    return agent.calculateIntelligence();
});
const perfEnd = performance.now();
const perfDuration = perfEnd - perfStart; // ?

console.log(`⚡ Performance: ${agents.length} agents calculés en ${perfDuration.toFixed(2)}ms`);

// 🎨 Visualisation ASCII simple
function createVisualization() {
    const size = 20;
    const scale = 2;
    let grid = Array(size).fill().map(() => Array(size).fill(' '));
    
    agents.forEach(agent => {
        const x = Math.floor((agent.position.x + scale) / (2*scale) * size);
        const y = Math.floor((agent.position.y + scale) / (2*scale) * size);
        
        if(x >= 0 && x < size && y >= 0 && y < size) {
            const intensity = Math.floor(agent.intelligence * 9);
            grid[y][x] = intensity.toString();
        }
    });
    
    return grid.map(row => row.join('')).join('\n');
}

const visualization = createVisualization(); // ?
console.log('🎨 Visualisation ASCII (intelligence des agents):');
console.log(visualization);

// 🏆 Conclusion
console.log('🎉 Démonstration Quokka + DIAMANTS terminée !');
console.log('💡 Modifiez les valeurs CONFIG et voyez les changements instantanés !');

// 📝 Note importante pour l'utilisateur
console.log('');
console.log('🔍 AVEC QUOKKA ACTIVÉ:');
console.log('   - Toutes les variables marquées // ? s\'affichent en temps réel');
console.log('   - Modifiez CONFIG.phi ou CONFIG.sigma et voyez l\'impact immédiat');
console.log('   - Les objets complexes sont explorables interactivement');
console.log('   - Performance monitoring en temps réel');
console.log('   ✨ C\'est la magie de Quokka.js ! ✨');
