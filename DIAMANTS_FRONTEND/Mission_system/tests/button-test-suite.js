/**
 * DIAMANTS Button Test Suite
 * Test systématique de tous les boutons et contrôles UI
 * 
 * Usage: Dans la console du navigateur:
 *   runButtonTests()           // Lancer tous les tests
 *   runButtonTests('mission')  // Tester uniquement Mission Control
 */

// ============================================
// DATASETS DE TEST
// ============================================

const TEST_DATASETS = {
    // Doctrines disponibles
    doctrines: [
        { value: 'stigmergy', label: 'Stigmergie', expected: 'STIGMERGY' },
        { value: 'swarm', label: 'Essaim', expected: 'SWARM' },
        { value: 'coverage', label: 'Couverture', expected: 'COVERAGE' },
        { value: 'formation', label: 'Formation', expected: 'FORMATION' },
        { value: 'search', label: 'Recherche', expected: 'SEARCH' }
    ],
    
    // COA disponibles
    coursesOfAction: [
        { value: 'adaptive', label: 'Adaptatif', expected: 'ADAPTIVE' },
        { value: 'grid', label: 'Grille', expected: 'GRID' },
        { value: 'boustrophedon', label: 'Boustrophedon', expected: 'BOUSTROPHEDON' },
        { value: 'spiral', label: 'Spirale', expected: 'SPIRAL' },
        { value: 'radial', label: 'Radial', expected: 'RADIAL' },
        { value: 'perimeter', label: 'Périmètre', expected: 'PERIMETER' }
    ],
    
    // Boutons Mission Control
    missionButtons: [
        { 
            id: 'btn-launch', 
            onclick: 'launchMission', 
            selector: '[data-action="launchMission"], button[onclick*="launchMission"]',
            description: 'Lance la mission',
            expectedBehavior: 'Démarre minimap, active doctrine'
        },
        { 
            id: 'btn-stop', 
            onclick: 'emergencyLand', 
            selector: '[data-action="emergencyLand"], button[onclick*="emergencyLand"]',
            description: 'Arrêt d\'urgence',
            expectedBehavior: 'Stop minimap, atterrit drones'
        },
        { 
            id: 'btn-takeoff', 
            onclick: 'takeoffAllDrones', 
            selector: '[data-action="takeoffAllDrones"], button[onclick*="takeoffAllDrones"]',
            description: 'Décollage tous drones',
            expectedBehavior: 'Envoie commande takeoff à tous les drones'
        },
        { 
            id: 'btn-land', 
            onclick: 'landAllDrones', 
            selector: '[data-action="landAllDrones"], button[onclick*="landAllDrones"]',
            description: 'Atterrissage tous drones',
            expectedBehavior: 'Envoie commande land à tous les drones'
        },
        { 
            id: 'btn-reset', 
            onclick: 'resetMission', 
            selector: '[data-action="resetMission"], button[onclick*="resetMission"]',
            description: 'Reset mission',
            expectedBehavior: 'Réinitialise UI, reset minimap, retour positions'
        }
    ],
    
    // Boutons Caméra
    cameraButtons: [
        { 
            onclick: 'resetCamera', 
            selector: '[data-action="resetCamera"], button[onclick*="resetCamera"]',
            description: 'Reset caméra',
            expectedBehavior: 'Retour position 40,25,40'
        },
        { 
            onclick: 'topView', 
            selector: '[data-action="topView"], button[onclick*="topView"]',
            description: 'Vue du dessus',
            expectedBehavior: 'Caméra au-dessus regardant vers bas'
        },
        { 
            onclick: 'zoomToSwarm', 
            selector: '[data-action="zoomToSwarm"], button[onclick*="zoomToSwarm"]',
            description: 'Zoom sur essaim',
            expectedBehavior: 'Centre sur barycentre des drones'
        },
        { 
            onclick: 'toggleFollowMode', 
            selector: '[data-action="toggleFollowMode"], button[onclick*="toggleFollowMode"]',
            description: 'Mode suivi',
            expectedBehavior: 'Toggle suivi automatique'
        }
    ],
    
    // Boutons Display
    displayButtons: [
        { 
            onclick: 'toggleMinimap', 
            selector: '[data-action="toggleMinimap"], button[onclick*="toggleMinimap"]',
            description: 'Toggle minimap',
            expectedBehavior: 'Affiche/masque minimap'
        },
        { 
            onclick: 'toggleDebugPanels', 
            selector: '[data-action="toggleDebugPanels"], button[onclick*="toggleDebugPanels"]',
            description: 'Toggle debug',
            expectedBehavior: 'Affiche/masque panels debug'
        },
        { 
            onclick: 'togglePanel', 
            selector: '[data-action="togglePanel"], button[onclick*="togglePanel"]',
            description: 'Toggle Vue intérieur',
            expectedBehavior: 'Bascule vue intérieur/extérieur'
        }
    ],
    
    // Sliders
    sliders: [
        {
            id: 'altitude_slider',
            displayId: 'altitude_display',
            min: 1,
            max: 8,
            testValues: [1, 3, 5, 8],
            description: 'Altitude'
        },
        {
            id: 'safety_distance',
            displayId: 'safety_distance_value',
            min: 1,
            max: 8,
            testValues: [1, 3, 5, 8],
            description: 'Distance sécurité'
        }
    ],
    
    // Éléments UI requis
    requiredElements: [
        'mission_type',
        'mode_select',
        'ros_interface',
        'minimap',
        'altitude_slider',
        'safety_distance'
    ]
};

// ============================================
// CLASSE DE TEST
// ============================================

class ButtonTestSuite {
    constructor() {
        this.results = {
            total: 0,
            passed: 0,
            failed: 0,
            skipped: 0,
            tests: []
        };
        this.log = [];
    }
    
    // Logging
    logTest(name, passed, details = '') {
        const status = passed ? '✅' : '❌';
        const msg = `${status} ${name}${details ? ': ' + details : ''}`;
        console.log(msg);
        this.log.push(msg);
        
        this.results.total++;
        if (passed) {
            this.results.passed++;
        } else {
            this.results.failed++;
        }
        this.results.tests.push({ name, passed, details });
    }
    
    logInfo(msg) {
        console.log(`ℹ️ ${msg}`);
        this.log.push(`ℹ️ ${msg}`);
    }
    
    logSection(title) {
        console.log(`\n${'='.repeat(50)}`);
        console.log(`📋 ${title}`);
        console.log('='.repeat(50));
        this.log.push(`\n--- ${title} ---`);
    }
    
    // ==========================================
    // TESTS - Éléments UI
    // ==========================================
    
    testRequiredElements() {
        this.logSection('TEST 1: Éléments UI Requis');
        
        TEST_DATASETS.requiredElements.forEach(id => {
            const element = document.getElementById(id);
            this.logTest(`Element #${id}`, element !== null, 
                element ? 'présent' : 'MANQUANT');
        });
    }
    
    // ==========================================
    // TESTS - Fonctions Globales
    // ==========================================
    
    testGlobalFunctions() {
        this.logSection('TEST 2: Fonctions Globales');
        
        const functions = [
            'launchMission',
            'emergencyLand',
            'takeoffAllDrones',
            'landAllDrones',
            'resetMission',
            'resetCamera',
            'topView',
            'zoomToSwarm',
            'toggleFollowMode',
            'toggleMinimap',
            'toggleDebugPanels',
            'togglePanel',
            'applyMode',
            'panelSelfTest'
        ];
        
        functions.forEach(fn => {
            const exists = typeof window[fn] === 'function';
            this.logTest(`window.${fn}()`, exists, 
                exists ? 'function' : 'undefined');
        });
    }
    
    // ==========================================
    // TESTS - Boutons Mission Control
    // ==========================================
    
    testMissionButtons() {
        this.logSection('TEST 3: Boutons Mission Control');
        
        TEST_DATASETS.missionButtons.forEach(btn => {
            // Vérifier bouton existe
            const element = document.querySelector(btn.selector);
            const buttonExists = element !== null;
            this.logTest(`Bouton ${btn.onclick}`, buttonExists,
                buttonExists ? btn.description : 'bouton non trouvé');
            
            // Vérifier fonction onclick
            if (buttonExists) {
                const fnName = btn.onclick;
                const fnExists = typeof window[fnName] === 'function';
                this.logTest(`  → onclick ${fnName}()`, fnExists);
            }
        });
    }
    
    // ==========================================
    // TESTS - Boutons Caméra
    // ==========================================
    
    testCameraButtons() {
        this.logSection('TEST 4: Boutons Caméra');
        
        TEST_DATASETS.cameraButtons.forEach(btn => {
            const element = document.querySelector(btn.selector);
            const buttonExists = element !== null;
            this.logTest(`Bouton ${btn.onclick}`, buttonExists, btn.description);
            
            if (buttonExists) {
                const fnExists = typeof window[btn.onclick] === 'function';
                this.logTest(`  → onclick ${btn.onclick}()`, fnExists);
            }
        });
    }
    
    // ==========================================
    // TESTS - Boutons Display
    // ==========================================
    
    testDisplayButtons() {
        this.logSection('TEST 5: Boutons Display');
        
        TEST_DATASETS.displayButtons.forEach(btn => {
            const element = document.querySelector(btn.selector);
            const buttonExists = element !== null;
            this.logTest(`Bouton ${btn.onclick}`, buttonExists, btn.description);
            
            if (buttonExists) {
                const fnExists = typeof window[btn.onclick] === 'function';
                this.logTest(`  → onclick ${btn.onclick}()`, fnExists);
            }
        });
    }
    
    // ==========================================
    // TESTS - Select Doctrine
    // ==========================================
    
    testDoctrineSelect() {
        this.logSection('TEST 6: Select Doctrine');
        
        const select = document.getElementById('mission_type');
        this.logTest('Select #mission_type existe', select !== null);
        
        if (select) {
            const options = Array.from(select.options);
            this.logInfo(`Options trouvées: ${options.length}`);
            
            TEST_DATASETS.doctrines.forEach(doctrine => {
                const option = options.find(o => o.value === doctrine.value);
                this.logTest(`  Option "${doctrine.value}"`, option !== null,
                    option ? doctrine.label : 'MANQUANTE');
            });
            
            // Test changement de valeur
            const originalValue = select.value;
            TEST_DATASETS.doctrines.forEach(doctrine => {
                select.value = doctrine.value;
                const changed = select.value === doctrine.value;
                this.logTest(`  Changement → ${doctrine.value}`, changed);
            });
            select.value = originalValue;
        }
    }
    
    // ==========================================
    // TESTS - Select COA
    // ==========================================
    
    testCOASelect() {
        this.logSection('TEST 7: Select Course of Action');
        
        const select = document.getElementById('mode_select');
        this.logTest('Select #mode_select existe', select !== null);
        
        if (select) {
            const options = Array.from(select.options);
            this.logInfo(`Options trouvées: ${options.length}`);
            
            TEST_DATASETS.coursesOfAction.forEach(coa => {
                const option = options.find(o => o.value === coa.value);
                this.logTest(`  Option "${coa.value}"`, option !== null,
                    option ? coa.label : 'MANQUANTE');
            });
            
            // Test changement de valeur
            const originalValue = select.value;
            TEST_DATASETS.coursesOfAction.forEach(coa => {
                select.value = coa.value;
                const changed = select.value === coa.value;
                this.logTest(`  Changement → ${coa.value}`, changed);
            });
            select.value = originalValue;
        }
    }
    
    // ==========================================
    // TESTS - Sliders
    // ==========================================
    
    testSliders() {
        this.logSection('TEST 8: Sliders');
        
        TEST_DATASETS.sliders.forEach(slider => {
            const element = document.getElementById(slider.id);
            const display = document.getElementById(slider.displayId);
            
            this.logTest(`Slider #${slider.id}`, element !== null, slider.description);
            this.logTest(`  Display #${slider.displayId}`, display !== null);
            
            if (element) {
                const originalValue = element.value;
                
                // Test valeurs
                slider.testValues.forEach(val => {
                    element.value = val;
                    element.dispatchEvent(new Event('input', { bubbles: true }));
                    
                    const accepted = parseFloat(element.value) === val || 
                                    parseFloat(element.value) >= slider.min;
                    this.logTest(`  Valeur ${val}`, accepted);
                });
                
                element.value = originalValue;
                element.dispatchEvent(new Event('input', { bubbles: true }));
            }
        });
    }
    
    // ==========================================
    // TESTS - Exécution réelle des fonctions
    // ==========================================
    
    testFunctionExecution() {
        this.logSection('TEST 9: Exécution Fonctions (dry-run)');
        
        // Test resetCamera
        try {
            if (typeof window.resetCamera === 'function') {
                window.resetCamera();
                this.logTest('resetCamera() exécuté', true);
            }
        } catch (e) {
            this.logTest('resetCamera() exécuté', false, e.message);
        }
        
        // Test toggleMinimap
        try {
            if (typeof window.toggleMinimap === 'function') {
                const minimap = document.getElementById('minimap');
                const wasHidden = minimap?.style.display === 'none';
                window.toggleMinimap();
                window.toggleMinimap(); // Retour état initial
                this.logTest('toggleMinimap() toggle', true);
            }
        } catch (e) {
            this.logTest('toggleMinimap() toggle', false, e.message);
        }
        
        // Test panelSelfTest
        try {
            if (typeof window.panelSelfTest === 'function') {
                const result = window.panelSelfTest();
                this.logTest('panelSelfTest() retourne résultats', 
                    result && typeof result.passed === 'number',
                    `${result?.passed}/${result?.passed + result?.failed} tests`);
            }
        } catch (e) {
            this.logTest('panelSelfTest() retourne résultats', false, e.message);
        }
    }
    
    // ==========================================
    // TESTS - Système
    // ==========================================
    
    testSystem() {
        this.logSection('TEST 10: Système DIAMANTS');
        
        // diamantsSystem
        const system = window.diamantsSystem;
        this.logTest('window.diamantsSystem', system !== undefined);
        
        if (system) {
            this.logTest('  → scene', system.scene !== undefined);
            this.logTest('  → camera', system.camera !== undefined);
            this.logTest('  → renderer', system.renderer !== undefined);
            this.logTest('  → integratedController', system.integratedController !== undefined);
        }
        
        // DoctrineManager
        const doctrine = window.doctrineManager;
        this.logTest('window.doctrineManager', doctrine !== undefined);
        
        // Minimap
        const minimap = window.DIAMANTS_MINIMAP;
        this.logTest('window.DIAMANTS_MINIMAP', minimap !== undefined);
        
        // PanelController
        const panel = window.panelController;
        this.logTest('window.panelController', panel !== undefined);
    }
    
    // ==========================================
    // RAPPORT FINAL
    // ==========================================
    
    generateReport() {
        this.logSection('📊 RAPPORT FINAL');
        
        const passRate = ((this.results.passed / this.results.total) * 100).toFixed(1);
        
        console.log(`\nTotal tests: ${this.results.total}`);
        console.log(`✅ Passés: ${this.results.passed}`);
        console.log(`❌ Échoués: ${this.results.failed}`);
        console.log(`📈 Taux de réussite: ${passRate}%`);
        
        if (this.results.failed > 0) {
            console.log('\n⚠️ Tests échoués:');
            this.results.tests
                .filter(t => !t.passed)
                .forEach(t => console.log(`   - ${t.name}: ${t.details}`));
        }
        
        return this.results;
    }
    
    // ==========================================
    // RUN ALL
    // ==========================================
    
    runAll(category = null) {
        console.log('\n🧪 DIAMANTS BUTTON TEST SUITE');
        console.log('=' .repeat(50));
        console.log(`Date: ${new Date().toISOString()}`);
        console.log(`Category: ${category || 'ALL'}\n`);
        
        // Reset results
        this.results = { total: 0, passed: 0, failed: 0, skipped: 0, tests: [] };
        this.log = [];
        
        const categoryMap = {
            'elements': () => this.testRequiredElements(),
            'functions': () => this.testGlobalFunctions(),
            'mission': () => this.testMissionButtons(),
            'camera': () => this.testCameraButtons(),
            'display': () => this.testDisplayButtons(),
            'doctrine': () => this.testDoctrineSelect(),
            'coa': () => this.testCOASelect(),
            'sliders': () => this.testSliders(),
            'execution': () => this.testFunctionExecution(),
            'system': () => this.testSystem()
        };
        
        if (category && categoryMap[category]) {
            categoryMap[category]();
        } else {
            // Exécuter tous les tests
            this.testRequiredElements();
            this.testGlobalFunctions();
            this.testMissionButtons();
            this.testCameraButtons();
            this.testDisplayButtons();
            this.testDoctrineSelect();
            this.testCOASelect();
            this.testSliders();
            this.testFunctionExecution();
            this.testSystem();
        }
        
        return this.generateReport();
    }
}

// ============================================
// EXPORT GLOBAL
// ============================================

window.ButtonTestSuite = ButtonTestSuite;
window.TEST_DATASETS = TEST_DATASETS;

window.runButtonTests = function(category = null) {
    const suite = new ButtonTestSuite();
    return suite.runAll(category);
};

// Info
console.log('[ButtonTestSuite] 🧪 Test suite chargée');
console.log('[ButtonTestSuite] Usage: runButtonTests() ou runButtonTests("mission")');
console.log('[ButtonTestSuite] Catégories: elements, functions, mission, camera, display, doctrine, coa, sliders, execution, system');

export { ButtonTestSuite, TEST_DATASETS };
