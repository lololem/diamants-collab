// 🚀 QUOKKA.JS TEST - Vérification fonctionnement
// Ctrl+Shift+P → "Quokka.js: Start on Current File" pour voir la magie !

console.log('🎯 Test Quokka.js - DIAMANTS V3'); // ?

// Test simple de calcul en temps réel
const phi = 1.5; // ?
const sigma = 2.0; // ?
const intelligence = Math.sqrt(phi * phi + sigma * sigma); // ?

console.log(`💡 Intelligence calculée: ${intelligence}`); // ?

// Test tableau avec map
const agents = [1, 2, 3, 4, 5]; // ?
const intelligences = agents.map(id => ({ 
    id, 
    intelligence: Math.random() * 10 
})); // ?

console.log('🤖 Agents créés:', intelligences); // ?

// Test performance
const start = performance.now(); // ?
let sum = 0;
for (let i = 0; i < 100000; i++) {
    sum += Math.sqrt(i);
}
const duration = performance.now() - start; // ?

console.log(`⚡ Performance: ${sum.toFixed(2)} en ${duration.toFixed(2)}ms`); // ?

// Test objet complexe
const diamantSystem = {
    phi: 1.8,
    sigma: 1.2,
    agents: 50,
    calculateTotal() {
        return this.phi + this.sigma;
    },
    getStatus() {
        return `Système DIAMANTS: φ=${this.phi}, σ=${this.sigma}, agents=${this.agents}`;
    }
}; // ?

console.log('🔧 Système:', diamantSystem.getStatus()); // ?
console.log('📊 Total:', diamantSystem.calculateTotal()); // ?

// Test de formule DIAMANTS simplifiée
function calculateDiamantsFormula(x, y, z, t) {
    const phi = Math.sin(x + t) * Math.cos(y);
    const sigma = Math.cos(x - t) * Math.sin(z);
    return Math.sqrt(phi * phi + sigma * sigma);
}

const testPoints = [
    [0, 0, 0],
    [1, 1, 1],
    [Math.PI, Math.PI/2, Math.PI/4]
]; // ?

const results = testPoints.map(([x, y, z]) => ({
    point: [x, y, z],
    intelligence: calculateDiamantsFormula(x, y, z, 0)
})); // ?

console.log('🧮 Résultats DIAMANTS:', results); // ?

// Vérification que tout fonctionne
console.log('✅ Quokka.js fonctionne parfaitement !'); // ?
