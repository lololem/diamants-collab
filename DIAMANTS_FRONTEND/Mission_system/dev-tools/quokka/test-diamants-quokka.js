// Test DIAMANTS avec Quokka.js - Formule I(t) = ∬|∇(φ+σ)|dΩ
// Démarrez Quokka: Ctrl+Shift+P → "Quokka.js: Start on Current File"

console.log('🔬 Test DIAMANTS V3 avec Quokka.js');

// Configuration DIAMANTS
const DIAMANTS_CONFIG = {
    phi: 1.5,          // Potentiel attractif
    sigma: 1.2,        // Potentiel répulsif  
    domainScale: 3.0,  // Échelle du domaine
    agentCount: 100,   // Nombre d'agents
    h: 0.01           // Pas pour dérivation numérique
}; // ?

// Fonction potentiel φ (attractif)
function phiPotential(x, y, z, t) {
    const r2 = x*x + y*y + z*z;
    const temporal = 1 + 0.1 * Math.sin(t * 0.1);
    return DIAMANTS_CONFIG.phi * Math.exp(-r2 / 4) * temporal;
}

// Fonction potentiel σ (répulsif)
function sigmaPotential(x, y, z, t) {
    const r2 = x*x + y*y + z*z;
    const temporal = 1 + 0.1 * Math.cos(t * 0.15);
    return DIAMANTS_CONFIG.sigma * Math.exp(-r2 / 2) * temporal;
}

// Potentiel total φ+σ
function totalPotential(x, y, z, t) {
    return phiPotential(x, y, z, t) + sigmaPotential(x, y, z, t);
}

// Calcul du gradient ∇(φ+σ)
function calculateGradient(x, y, z, t) {
    const h = DIAMANTS_CONFIG.h;
    
    const gradX = (totalPotential(x + h, y, z, t) - totalPotential(x - h, y, z, t)) / (2 * h);
    const gradY = (totalPotential(x, y + h, z, t) - totalPotential(x, y - h, z, t)) / (2 * h);
    const gradZ = (totalPotential(x, y, z + h, t) - totalPotential(x, y, z - h, t)) / (2 * h);
    
    return { x: gradX, y: gradY, z: gradZ };
}

// Calcul de l'intelligence I(t) = |∇(φ+σ)|
function calculateIntelligence(gradient) {
    return Math.sqrt(gradient.x * gradient.x + gradient.y * gradient.y + gradient.z * gradient.z);
}

// Tests en temps réel avec Quokka
console.log('📊 Tests du système DIAMANTS:');

// Test 1: Point central
let position1 = { x: 0, y: 0, z: 0 };
let t1 = 0;
let grad1 = calculateGradient(position1.x, position1.y, position1.z, t1); // ?
let intelligence1 = calculateIntelligence(grad1); // ?
console.log(`Centre (0,0,0): I = ${intelligence1.toFixed(6)}`);

// Test 2: Point décalé
let position2 = { x: 1, y: 1, z: 0.5 };
let t2 = 10;
let grad2 = calculateGradient(position2.x, position2.y, position2.z, t2); // ?
let intelligence2 = calculateIntelligence(grad2); // ?
console.log(`Position (1,1,0.5): I = ${intelligence2.toFixed(6)}`);

// Test 3: Évolution temporelle
let timeEvolution = [];
for(let t = 0; t < 100; t += 10) {
    let grad = calculateGradient(0.5, 0.5, 0, t);
    let intel = calculateIntelligence(grad);
    timeEvolution.push({ time: t, intelligence: intel });
}
timeEvolution; // ?

// Test 4: Distribution spatiale
let spatialDistribution = [];
for(let x = -2; x <= 2; x += 0.5) {
    for(let y = -2; y <= 2; y += 0.5) {
        let grad = calculateGradient(x, y, 0, 50);
        let intel = calculateIntelligence(grad);
        spatialDistribution.push({ x, y, intelligence: intel });
    }
}

// Statistiques de la distribution
let maxIntel = Math.max(...spatialDistribution.map(p => p.intelligence)); // ?
let minIntel = Math.min(...spatialDistribution.map(p => p.intelligence)); // ?
let avgIntel = spatialDistribution.reduce((sum, p) => sum + p.intelligence, 0) / spatialDistribution.length; // ?

console.log(`📈 Statistiques spatiales:`);
console.log(`   Max Intelligence: ${maxIntel.toFixed(6)}`);
console.log(`   Min Intelligence: ${minIntel.toFixed(6)}`);
console.log(`   Moyenne: ${avgIntel.toFixed(6)}`);

// Test 5: Performance
let startTime = performance.now();
let performanceTest = [];
for(let i = 0; i < 1000; i++) {
    let x = (Math.random() - 0.5) * 4;
    let y = (Math.random() - 0.5) * 4;
    let z = (Math.random() - 0.5) * 4;
    let t = Math.random() * 100;
    
    let grad = calculateGradient(x, y, z, t);
    let intel = calculateIntelligence(grad);
    performanceTest.push(intel);
}
let endTime = performance.now();
let duration = endTime - startTime; // ?

console.log(`⚡ Performance: ${1000} calculs en ${duration.toFixed(2)}ms`);
console.log(`   Débit: ${(1000 / duration * 1000).toFixed(0)} calculs/seconde`);

// Formule DIAMANTS complète - Intégration approximative
function approximateIntegral(domain, resolution) {
    let sum = 0;
    let count = 0;
    
    for(let x = -domain; x <= domain; x += resolution) {
        for(let y = -domain; y <= domain; y += resolution) {
            for(let z = -domain; z <= domain; z += resolution) {
                let grad = calculateGradient(x, y, z, 0);
                let intel = calculateIntelligence(grad);
                sum += intel * Math.pow(resolution, 3); // dΩ = dx*dy*dz
                count++;
            }
        }
    }
    
    return { integral: sum, points: count };
}

// Test d'intégration I(t) = ∬|∇(φ+σ)|dΩ
let integralResult = approximateIntegral(1.5, 0.3); // ?
console.log(`🧮 Intégrale DIAMANTS: I(t) = ${integralResult.integral.toFixed(6)}`);
console.log(`   Calculée sur ${integralResult.points} points`);

console.log('✅ Test DIAMANTS avec Quokka terminé !');

// Note: Avec Quokka, vous devriez voir toutes les valeurs marquées // ? 
// s'afficher en temps réel à droite du code !
