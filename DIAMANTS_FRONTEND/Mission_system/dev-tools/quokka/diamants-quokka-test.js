// 🧪 Test Quokka.js - Version compatible DIAMANTS V3
// Ctrl+Shift+P → "Quokka.js: Start on Current File" pour voir la magie !

console.log('🧪 Tests DIAMANTS V3 - Version Quokka'); // ?

// Fonction DIAMANTS basique pour les tests
function calculateDiamantPotential(phi, sigma) {
    if (typeof phi !== 'number' || typeof sigma !== 'number') {
        throw new Error('phi et sigma doivent être des nombres');
    }
    return Math.sqrt(phi * phi + sigma * sigma);
}

function calculateGradient(x, y, z, potential) {
    const h = 0.01;
    return {
        x: (potential(x + h, y, z) - potential(x - h, y, z)) / (2 * h),
        y: (potential(x, y + h, z) - potential(x, y - h, z)) / (2 * h),
        z: (potential(x, y, z + h) - potential(x, y, z - h)) / (2 * h)
    };
}

// Tests Quokka (sans Jest) - Résultats visibles avec // ?
console.log('\n🔬 === TESTS DIAMANTS V3 ==='); // ?

// Test 1: Calcul basique
const result1 = calculateDiamantPotential(3, 4); // ?
console.log('✓ Test 1 - Calcul basique (3,4):', result1); // ?

// Test 2: Valeurs zéro
const result2 = calculateDiamantPotential(0, 0); // ?
console.log('✓ Test 2 - Zéros (0,0):', result2); // ?

// Test 3: Valeurs négatives
const result3 = calculateDiamantPotential(-3, 4); // ?
console.log('✓ Test 3 - Négatives (-3,4):', result3); // ?

// Test 4: Gradient
const testPotential = (x, y, z) => x * x + y * y + z * z;
const gradient = calculateGradient(1, 1, 1, testPotential); // ?
console.log('✓ Test 4 - Gradient en (1,1,1):', gradient); // ?

// Test 5: Très grandes valeurs
const result5 = calculateDiamantPotential(1e6, 1e6); // ?
console.log('✓ Test 5 - Grandes valeurs:', result5); // ?

// Test 6: Très petites valeurs
const result6 = calculateDiamantPotential(1e-10, 1e-10); // ?
console.log('✓ Test 6 - Petites valeurs:', result6); // ?

// Validation automatique des résultats
const tests = [
    { name: 'Calcul basique', result: result1, expected: 5, tolerance: 0.001 },
    { name: 'Zéros', result: result2, expected: 0, tolerance: 0.001 },
    { name: 'Négatives', result: result3, expected: 5, tolerance: 0.001 },
    { name: 'Gradient X', result: gradient.x, expected: 2, tolerance: 0.1 },
    { name: 'Gradient Y', result: gradient.y, expected: 2, tolerance: 0.1 },
    { name: 'Gradient Z', result: gradient.z, expected: 2, tolerance: 0.1 }
]; // ?

console.log('\n📊 Résultats automatiques:'); // ?
const results = tests.map(test => {
    const passed = Math.abs(test.result - test.expected) < test.tolerance;
    const status = passed ? '✅' : '❌';
    const message = `${status} ${test.name}: ${test.result.toFixed(4)} (attendu: ${test.expected})`;
    console.log(message);
    return { ...test, passed, status };
}); // ?

// Statistiques des tests
const passedCount = results.filter(r => r.passed).length; // ?
const totalCount = results.length; // ?
const successRate = (passedCount / totalCount * 100).toFixed(1); // ?

console.log(`\n🎯 Résumé: ${passedCount}/${totalCount} tests réussis (${successRate}%)`); // ?

// Test d'erreur
console.log('\n🚫 Test de gestion d\'erreurs:'); // ?
try {
    calculateDiamantPotential('abc', 4);
    console.log('❌ Test erreur échoué - devrait lancer une exception'); // ?
} catch (error) {
    console.log('✅ Test erreur réussi:', error.message); // ?
}

// Test interactif - modifiez ces valeurs pour voir les changements !
console.log('\n🎮 Test interactif - Modifiez les valeurs ci-dessous:'); // ?
const phi_interactive = 2.5; // ← Changez cette valeur !
const sigma_interactive = 1.8; // ← Changez cette valeur !
const intelligence_interactive = calculateDiamantPotential(phi_interactive, sigma_interactive); // ?

console.log(`💡 Intelligence DIAMANTS: φ=${phi_interactive}, σ=${sigma_interactive} → I=${intelligence_interactive.toFixed(4)}`); // ?

// Démonstration formule DIAMANTS avancée
console.log('\n🧮 Formule DIAMANTS avancée:'); // ?
function advancedDiamantsFormula(x, y, z, t) {
    const phi = Math.sin(x + t) * Math.cos(y) * Math.exp(-z*z/10);
    const sigma = Math.cos(x - t) * Math.sin(z) * Math.exp(-y*y/10);
    const gradient_magnitude = Math.sqrt(phi * phi + sigma * sigma);
    return { phi, sigma, intelligence: gradient_magnitude };
}

const t = 0; // ← Changez le temps !
const testPoints = [
    [0, 0, 0],
    [Math.PI/4, Math.PI/4, Math.PI/4],
    [1, 1, 1]
]; // ?

const diamantsResults = testPoints.map(([x, y, z]) => ({
    point: [x.toFixed(2), y.toFixed(2), z.toFixed(2)],
    ...advancedDiamantsFormula(x, y, z, t)
})); // ?

console.log('🔮 Résultats DIAMANTS avancés:', diamantsResults); // ?

// Performance test
console.log('\n⚡ Test de performance:'); // ?
const perfStart = performance.now(); // ?
let perfSum = 0;
for (let i = 0; i < 10000; i++) {
    perfSum += calculateDiamantPotential(Math.random(), Math.random());
}
const perfDuration = performance.now() - perfStart; // ?
const perfRate = (10000 / perfDuration * 1000).toFixed(0); // ?

console.log(`🚀 Performance: ${perfRate} calculs/seconde (${perfDuration.toFixed(2)}ms pour 10k calculs)`); // ?

// Export pour utilisation dans d'autres modules
module.exports = {
    calculateDiamantPotential,
    calculateGradient,
    advancedDiamantsFormula
};

console.log('\n✨ Quokka.js fonctionne parfaitement avec DIAMANTS V3 !'); // ?
