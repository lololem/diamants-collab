// Test Wallaby.js - Suite de tests unitaires
// Ce fichier teste les fonctions DIAMANTS V3

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

// Tests unitaires
describe('DIAMANTS V3 Core Functions', () => {
    test('calculateDiamantPotential avec valeurs valides', () => {
        const result = calculateDiamantPotential(3, 4);
        expect(result).toBe(5);
    });

    test('calculateDiamantPotential avec zéros', () => {
        const result = calculateDiamantPotential(0, 0);
        expect(result).toBe(0);
    });

    test('calculateDiamantPotential avec valeurs négatives', () => {
        const result = calculateDiamantPotential(-3, 4);
        expect(result).toBe(5);
    });

    test('calculateDiamantPotential lance erreur avec paramètres invalides', () => {
        expect(() => calculateDiamantPotential('abc', 4)).toThrow();
        expect(() => calculateDiamantPotential(3, null)).toThrow();
    });

    test('calculateGradient calcule les dérivées partielles', () => {
        const testPotential = (x, y, z) => x * x + y * y + z * z;
        const gradient = calculateGradient(1, 1, 1, testPotential);
        
        // Pour f(x,y,z) = x² + y² + z², le gradient en (1,1,1) est (2,2,2)
        expect(gradient.x).toBeCloseTo(2, 1);
        expect(gradient.y).toBeCloseTo(2, 1);
        expect(gradient.z).toBeCloseTo(2, 1);
    });
});

describe('DIAMANTS V3 Edge Cases', () => {
    test('Très grandes valeurs', () => {
        const result = calculateDiamantPotential(1e6, 1e6);
        expect(result).toBeCloseTo(Math.sqrt(2) * 1e6, -5);
    });

    test('Très petites valeurs', () => {
        const result = calculateDiamantPotential(1e-10, 1e-10);
        expect(result).toBeCloseTo(Math.sqrt(2) * 1e-10, 20);
    });
});

// Export pour utilisation dans d'autres modules
module.exports = {
    calculateDiamantPotential,
    calculateGradient
};
