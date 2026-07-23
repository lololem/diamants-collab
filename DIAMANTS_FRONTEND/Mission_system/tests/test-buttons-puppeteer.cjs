/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS Button Test Runner
 * Test automatisé des boutons via Puppeteer
 * 
 * Usage: node test-buttons-puppeteer.js
 */

const puppeteer = require('puppeteer');

const URL = 'http://localhost:3005';

async function runTests() {
    console.log('🚀 DIAMANTS Button Test Runner');
    console.log('================================\n');
    
    const browser = await puppeteer.launch({ 
        headless: 'new',
        args: ['--no-sandbox', '--disable-setuid-sandbox']
    });
    
    const page = await browser.newPage();
    
    // Capturer les logs console
    const logs = [];
    const errors = [];
    
    page.on('console', msg => {
        const text = msg.text();
        logs.push({ type: msg.type(), text });
        console.log(`[${msg.type()}] ${text}`);
    });
    
    page.on('pageerror', err => {
        errors.push(err.message);
        console.error(`[ERROR] ${err.message}`);
    });
    
    try {
        console.log(`📡 Navigation vers ${URL}...`);
        await page.goto(URL, { waitUntil: 'domcontentloaded', timeout: 15000 });
        
        // Attendre que le système soit initialisé
        console.log('⏳ Attente initialisation...');
        await new Promise(r => setTimeout(r, 5000));
        
        // ================================
        // TEST 1: Vérifier les fonctions globales
        // ================================
        console.log('\n📋 TEST 1: Fonctions globales');
        console.log('─'.repeat(40));
        
        const globalFunctions = await page.evaluate(() => {
            const fns = [
                'launchMission', 'emergencyLand', 'takeoffAllDrones', 
                'landAllDrones', 'resetMission', 'resetCamera', 
                'topView', 'zoomToSwarm', 'toggleFollowMode',
                'toggleMinimap', 'toggleDebugPanels', 'togglePanel',
                'applyMode', 'panelSelfTest', 'runButtonTests'
            ];
            
            return fns.map(fn => ({
                name: fn,
                exists: typeof window[fn] === 'function'
            }));
        });
        
        let passed = 0;
        let failed = 0;
        
        globalFunctions.forEach(fn => {
            const status = fn.exists ? '✅' : '❌';
            console.log(`  ${status} window.${fn.name}`);
            fn.exists ? passed++ : failed++;
        });
        
        console.log(`\n  Résultat: ${passed}/${passed + failed} fonctions trouvées`);
        
        // ================================
        // TEST 2: Éléments UI
        // ================================
        console.log('\n📋 TEST 2: Éléments UI');
        console.log('─'.repeat(40));
        
        const uiElements = await page.evaluate(() => {
            const elements = [
                'mission_type', 'mode_select', 'minimap', 
                'altitude_slider', 'safety_distance', 'ros_interface'
            ];
            
            return elements.map(id => ({
                id,
                exists: document.getElementById(id) !== null
            }));
        });
        
        uiElements.forEach(el => {
            const status = el.exists ? '✅' : '❌';
            console.log(`  ${status} #${el.id}`);
            el.exists ? passed++ : failed++;
        });
        
        // ================================
        // TEST 3: Boutons Mission Control
        // ================================
        console.log('\n📋 TEST 3: Boutons Mission Control');
        console.log('─'.repeat(40));
        
        const buttons = await page.evaluate(() => {
            // Chercher les boutons par leur contenu texte
            const allButtons = Array.from(document.querySelectorAll('button.btn_ros'));
            
            const btnTests = [
                { name: 'Launch', text: 'Launch', fn: 'launchMission' },
                { name: 'Stop', text: 'Stop', fn: 'emergencyLand' },
                { name: 'Takeoff', text: 'Takeoff', fn: 'takeoffAllDrones' },
                { name: 'Land', text: 'Land', fn: 'landAllDrones' },
                { name: 'Reset', text: 'Reset', fn: 'resetMission' }
            ];
            
            return btnTests.map(btn => {
                const found = allButtons.find(b => 
                    b.textContent.includes(btn.text) || 
                    (b.getAttribute('onclick') && b.getAttribute('onclick').includes(btn.fn))
                );
                return {
                    name: btn.name,
                    exists: found !== undefined,
                    onclick: found?.getAttribute('onclick') || 'N/A'
                };
            });
        });
        
        buttons.forEach(btn => {
            const status = btn.exists ? '✅' : '❌';
            console.log(`  ${status} ${btn.name}`);
        });
        
        // ================================
        // TEST 4: Cliquer sur les boutons
        // ================================
        console.log('\n📋 TEST 4: Test des clics');
        console.log('─'.repeat(40));
        
        // Test resetCamera
        const resetCameraResult = await page.evaluate(() => {
            try {
                if (typeof window.resetCamera === 'function') {
                    window.resetCamera();
                    return { success: true, message: 'OK' };
                }
                return { success: false, message: 'Function not found' };
            } catch (e) {
                return { success: false, message: e.message };
            }
        });
        console.log(`  ${resetCameraResult.success ? '✅' : '❌'} resetCamera(): ${resetCameraResult.message}`);
        
        // Test toggleMinimap
        const toggleMinimapResult = await page.evaluate(() => {
            try {
                if (typeof window.toggleMinimap === 'function') {
                    window.toggleMinimap();
                    return { success: true, message: 'OK' };
                }
                return { success: false, message: 'Function not found' };
            } catch (e) {
                return { success: false, message: e.message };
            }
        });
        console.log(`  ${toggleMinimapResult.success ? '✅' : '❌'} toggleMinimap(): ${toggleMinimapResult.message}`);
        
        // Test select doctrine
        const doctrineTest = await page.evaluate(() => {
            try {
                const select = document.getElementById('mission_type');
                if (select) {
                    const original = select.value;
                    select.value = 'coverage';
                    select.dispatchEvent(new Event('change', { bubbles: true }));
                    const changed = select.value === 'coverage';
                    select.value = original;
                    return { success: changed, message: changed ? 'OK' : 'Value not changed' };
                }
                return { success: false, message: 'Select not found' };
            } catch (e) {
                return { success: false, message: e.message };
            }
        });
        console.log(`  ${doctrineTest.success ? '✅' : '❌'} mission_type change: ${doctrineTest.message}`);
        
        // Test select COA
        const coaTest = await page.evaluate(() => {
            try {
                const select = document.getElementById('mode_select');
                if (select) {
                    const original = select.value;
                    select.value = 'spiral';
                    select.dispatchEvent(new Event('change', { bubbles: true }));
                    const changed = select.value === 'spiral';
                    select.value = original;
                    return { success: changed, message: changed ? 'OK' : 'Value not changed' };
                }
                return { success: false, message: 'Select not found' };
            } catch (e) {
                return { success: false, message: e.message };
            }
        });
        console.log(`  ${coaTest.success ? '✅' : '❌'} mode_select change: ${coaTest.message}`);
        
        // Test slider
        const sliderTest = await page.evaluate(() => {
            try {
                const slider = document.getElementById('altitude_slider');
                if (slider) {
                    const original = slider.value;
                    slider.value = 5;
                    slider.dispatchEvent(new Event('input', { bubbles: true }));
                    const changed = parseFloat(slider.value) === 5;
                    slider.value = original;
                    return { success: changed, message: changed ? 'OK' : 'Value not changed' };
                }
                return { success: false, message: 'Slider not found' };
            } catch (e) {
                return { success: false, message: e.message };
            }
        });
        console.log(`  ${sliderTest.success ? '✅' : '❌'} altitude_slider: ${sliderTest.message}`);
        
        // ================================
        // TEST 5: Exécuter runButtonTests
        // ================================
        console.log('\n📋 TEST 5: Suite de tests intégrée');
        console.log('─'.repeat(40));
        
        const testSuiteResult = await page.evaluate(() => {
            try {
                if (typeof window.runButtonTests === 'function') {
                    const results = window.runButtonTests();
                    return {
                        success: true,
                        passed: results.passed,
                        failed: results.failed,
                        total: results.total
                    };
                }
                return { success: false, message: 'runButtonTests not found' };
            } catch (e) {
                return { success: false, message: e.message };
            }
        });
        
        if (testSuiteResult.success) {
            console.log(`  ✅ runButtonTests() exécuté`);
            console.log(`     Passés: ${testSuiteResult.passed}`);
            console.log(`     Échoués: ${testSuiteResult.failed}`);
            console.log(`     Total: ${testSuiteResult.total}`);
        } else {
            console.log(`  ❌ runButtonTests(): ${testSuiteResult.message}`);
        }
        
        // ================================
        // RAPPORT FINAL
        // ================================
        console.log('\n' + '═'.repeat(50));
        console.log('📊 RAPPORT FINAL');
        console.log('═'.repeat(50));
        
        const totalPassed = globalFunctions.filter(f => f.exists).length + 
                           uiElements.filter(e => e.exists).length +
                           buttons.filter(b => b.exists).length;
        
        const totalTests = globalFunctions.length + uiElements.length + buttons.length;
        
        console.log(`Tests basiques: ${totalPassed}/${totalTests}`);
        
        if (testSuiteResult.success) {
            console.log(`Suite complète: ${testSuiteResult.passed}/${testSuiteResult.total}`);
        }
        
        console.log(`Erreurs JS: ${errors.length}`);
        
        if (errors.length > 0) {
            console.log('\n⚠️ Erreurs détectées:');
            errors.forEach(e => console.log(`   - ${e}`));
        }
        
    } catch (e) {
        console.error('❌ Erreur fatale:', e.message);
    } finally {
        await browser.close();
        console.log('\n✅ Tests terminés');
    }
}

runTests().catch(console.error);
