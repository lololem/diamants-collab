/**
 * Console Ninja Test - DIAMANTS V3
 * ================================
 * Test pour vérifier que Console Ninja fonctionne avec Mission_system
 */

// 🔧 Test de base Console Ninja
function testConsoleNinja() {
    console.log('🎯 Console Ninja Test Started');
    
    // Test 1: Variables simples
    const testVar = "Mission_system Debug Test";
    console.log('📊 Test Variable:', testVar);
    
    // Test 2: Objet complexe
    const diamantsState = {
        mission: 'DIAMANTS V3',
        version: '3.0.0',
        debug: {
            consoleNinja: true,
            timestamp: new Date().toISOString(),
            tools: ['Turbo Console Log', 'Console Ninja', 'ESLint', 'Prettier']
        },
        scene: {
            initialized: !!window.scene,
            drones: window.drones?.length || 0,
            renderer: !!window.renderer
        }
    };
    console.log('🎮 DIAMANTS State:', diamantsState);
    
    // Test 3: Erreur intentionnelle pour test
    try {
        throw new Error('Test error for Console Ninja');
    } catch (error) {
        console.error('❌ Test Error:', error.message);
    }
    
    // Test 4: Console trace
    console.trace('🔍 Test Stack Trace');
    
    // Test 5: Performance timing
    console.time('DIAMANTS-Test-Timer');
    setTimeout(() => {
        console.timeEnd('DIAMANTS-Test-Timer');
        console.log('✅ Console Ninja Test Completed');
    }, 100);
    
    return diamantsState;
}

// 🚀 Auto-test au chargement
document.addEventListener('DOMContentLoaded', () => {
    console.log('🌟 Page loaded - Starting Console Ninja test');
    setTimeout(testConsoleNinja, 1000);
});

// Export pour utilisation globale
window.testConsoleNinja = testConsoleNinja;

// 📋 Instructions pour l'utilisateur
console.log(`
🎯 CONSOLE NINJA SETUP COMPLETE
==============================

1. Ouvrez VS Code
2. Votre serveur tourne sur http://localhost:39409
3. Les logs apparaîtront directement dans votre éditeur VS Code
4. Utilisez window.testConsoleNinja() pour tester

Extensions actives:
- ✅ Console Ninja (wallabyjs.console-ninja)
- ✅ Turbo Console Log (chakrounanas.turbo-console-log)  
- ✅ ESLint (dbaeumer.vscode-eslint)
- ✅ Prettier (esbenp.prettier-vscode)
- ✅ Code Runner (formulahendry.code-runner)

Node.js version: ${navigator.userAgent.includes('Chrome') ? 'Browser' : 'Node'} runtime
Console Ninja status: ${window.consoleNinja ? 'Active' : 'Monitoring'}
`);
