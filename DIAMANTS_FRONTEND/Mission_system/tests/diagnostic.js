/**
 * DIAMANTS - Script de Diagnostic et Correction Automatique
 * ========================================================
 */

console.log('🔍 DIAMANTS - Diagnostic en cours...');

// Test de tous les modules critiques
async function diagnosticComplet() {
    const resultats = {
        modules: {},
        erreurs: [],
        corrections: []
    };

    const modulesATest = [
        './drones/authentic-crazyflie.js',
        './missions/mission-manager.js',
        './environment/provencal-environment.js',
        './intelligence/collective-intelligence.js',
        './ui/diamant-ui.js',
        // Nouveaux modules migrés
        './behaviors/flight-behaviors.js',
        './behaviors/collaborative-scouting.js',
        './intelligence/advanced-collective-intelligence.js',
        './integrated-controller.js'
    ];

    console.log('📋 Test des imports...');

    for (const modulePath of modulesATest) {
        try {
            console.log(`  ⏳ Test: ${modulePath}`);
            const module = await import(modulePath);

            if (module && Object.keys(module).length > 0) {
                resultats.modules[modulePath] = '✅ OK';
                console.log(`  ✅ ${modulePath} : Chargé`);
            } else {
                resultats.modules[modulePath] = '⚠️ Vide';
                resultats.erreurs.push(`Module ${modulePath} ne contient pas d'exports`);
            }
        } catch (error) {
            resultats.modules[modulePath] = '❌ Erreur';
            resultats.erreurs.push(`${modulePath}: ${error.message}`);
            console.error(`  ❌ ${modulePath} : ${error.message}`);
        }
    }

    console.log('🧪 Test des méthodes critiques...');

    try {
        // Test MissionManager
        const { MissionManager } = await import('./missions/mission-manager.js');
        const mission = new MissionManager();

        if (typeof mission.checkBoundaries !== 'function') {
            resultats.erreurs.push('checkBoundaries manquante dans MissionManager');
        }

    } catch (error) {
        resultats.erreurs.push(`Erreur test méthodes: ${error.message}`);
    }

    console.log('📊 Résultats du diagnostic:');
    console.log('Modules:', resultats.modules);

    if (resultats.erreurs.length === 0) {
        console.log('✅ Aucune erreur détectée !');
        document.body.innerHTML = `
            <h1 style="color: green;">✅ DIAMANTS - Système Opérationnel</h1>
            <p>Tous les modules sont chargés correctement.</p>
            <ul>
                ${Object.entries(resultats.modules).map(([module, status]) =>
            `<li>${module}: ${status}</li>`
        ).join('')}
            </ul>
            <button onclick="location.href='./index.html'">Lancer DIAMANTS</button>
        `;
    } else {
        console.log('❌ Erreurs détectées:', resultats.erreurs);
        document.body.innerHTML = `
            <h1 style="color: red;">❌ DIAMANTS - Erreurs Détectées</h1>
            <h2>Modules:</h2>
            <ul>
                ${Object.entries(resultats.modules).map(([module, status]) =>
            `<li>${module}: ${status}</li>`
        ).join('')}
            </ul>
            <h2>Erreurs:</h2>
            <ul style="color: red;">
                ${resultats.erreurs.map(erreur => `<li>${erreur}</li>`).join('')}
            </ul>
        `;
    }
}

// Capture des erreurs globales
window.addEventListener('error', function (e) {
    console.error('Erreur globale capturée:', e.error);
});

window.addEventListener('unhandledrejection', function (e) {
    console.error('Promise rejetée:', e.reason);
});

// Démarrage du diagnostic
diagnosticComplet().catch(error => {
    console.error('Erreur fatale diagnostic:', error);
    document.body.innerHTML = `<h1 style="color: red;">Erreur fatale: ${error.message}</h1>`;
});
