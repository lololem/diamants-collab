// Test Quokka.js - Fichier de test pour vérifier le fonctionnement
// Utilisez Ctrl+Shift+P puis "Quokka.js: Start on Current File"

console.log('🚀 Quokka.js est actif !');

// Variables basiques
let message = 'Hello Quokka!';
console.log(message);

// Calculs en temps réel
let x = 10;
let y = 20;
let somme = x + y; // ?
console.log(`${x} + ${y} = ${somme}`);

// Fonction simple
function calculerPuissance(base, exposant) {
    return Math.pow(base, exposant);
}

let puissance = calculerPuissance(2, 8); // ?
console.log(`2^8 = ${puissance}`);

// Tableau et manipulation
let nombres = [1, 2, 3, 4, 5];
let doubles = nombres.map(n => n * 2); // ?
console.log('Nombres doublés:', doubles);

// Test avec DIAMANTS (simulation basique)
class DiamantAgent {
    constructor(x, y) {
        this.x = x;
        this.y = y;
        this.phi = 0;
        this.sigma = 0;
        this.intelligence = 0;
    }
    
    calculerIntelligence() {
        this.intelligence = Math.sqrt(this.phi * this.phi + this.sigma * this.sigma);
        return this.intelligence;
    }
}

let agent = new DiamantAgent(5, 10);
agent.phi = 3.5;
agent.sigma = 2.1;
let intelligence = agent.calculerIntelligence(); // ?
console.log(`Intelligence de l'agent: ${intelligence}`);

// Test avec Date
let maintenant = new Date(); // ?
console.log('Timestamp actuel:', maintenant.toISOString());

// Performance test
let debut = performance.now();
for(let i = 0; i < 1000; i++) {
    Math.sqrt(i);
}
let fin = performance.now();
let duree = fin - debut; // ?
console.log(`Calcul de 1000 racines carrées: ${duree.toFixed(2)}ms`);

console.log('✅ Test Quokka.js terminé avec succès !');
