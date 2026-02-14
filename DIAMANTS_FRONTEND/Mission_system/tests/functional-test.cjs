/**
 * TEST FONCTIONNEL - Clique chaque bouton et vérifie la chaîne
 */
const puppeteer = require('puppeteer');

const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

const BUTTONS_TO_TEST = [
    // Mission Control
    { name: 'Launch Mission', selector: '[data-action="launchMission"], button[onclick*="launchMission"]', expectLogs: ['mission', 'launch', 'start'] },
    { name: 'Reset Mission', selector: '[data-action="resetMission"], button[onclick*="resetMission"]', expectLogs: ['reset'] },
    { name: 'Emergency Land', selector: '[data-action="emergencyLand"], button[onclick*="emergencyLand"]', expectLogs: ['emergency', 'land', 'stop'] },
    { name: 'Takeoff All', selector: '[data-action="takeoffAllDrones"], button[onclick*="takeoffAllDrones"]', expectLogs: ['takeoff'] },
    { name: 'Land All', selector: '[data-action="landAllDrones"], button[onclick*="landAllDrones"]', expectLogs: ['land'] },
    
    // Camera
    { name: 'Reset Camera', selector: '[data-action="resetCamera"], button[onclick*="resetCamera"]', expectLogs: ['camera', 'reset'] },
    { name: 'Top View', selector: '[data-action="topView"], button[onclick*="topView"]', expectLogs: ['top', 'view'] },
    { name: 'Zoom Swarm', selector: '[data-action="zoomToSwarm"], button[onclick*="zoomToSwarm"]', expectLogs: ['zoom', 'swarm'] },
    { name: 'Toggle Follow', selector: '[data-action="toggleFollowMode"], button[onclick*="toggleFollowMode"]', expectLogs: ['follow'] },
    
    // Display
    { name: 'Toggle Minimap', selector: '[data-action="toggleMinimap"], button[onclick*="toggleMinimap"]', expectLogs: ['minimap'] },
    { name: 'Toggle Debug', selector: '[data-action="toggleDebugPanels"], button[onclick*="toggleDebugPanels"]', expectLogs: ['debug'] },
];

async function runFunctionalTests() {
    console.log('\n🧪 TEST FONCTIONNEL - Clic sur chaque bouton\n');
    console.log('='.repeat(60));
    
    const browser = await puppeteer.launch({ 
        headless: 'new',
        args: ['--no-sandbox']
    });
    
    const page = await browser.newPage();
    page.setDefaultNavigationTimeout(120000);
    page.setDefaultTimeout(60000);
    
    // Capturer tous les logs console
    const consoleLogs = [];
    page.on('console', msg => {
        consoleLogs.push({
            type: msg.type(),
            text: msg.text(),
            time: Date.now()
        });
    });
    
    // Capturer les erreurs
    const errors = [];
    page.on('pageerror', err => {
        errors.push(err.message);
    });
    
    await page.goto('http://localhost:3005', { waitUntil: 'domcontentloaded', timeout: 60000 });
    await delay(5000); // Attendre init Three.js
    
    // Rendre le panel ros_interface visible
    await page.evaluate(() => {
        const panel = document.getElementById('ros_interface');
        if (panel) {
            panel.style.display = 'block';
            panel.style.visibility = 'visible';
        }
    });
    await delay(500);
    console.log('📌 Panel ros_interface rendu visible');
    
    const results = [];
    
    for (const btn of BUTTONS_TO_TEST) {
        console.log(`\n▶️  Test: ${btn.name}`);
        console.log('-'.repeat(40));
        
        // Vider les logs avant le clic
        const logsBefore = consoleLogs.length;
        const errorsBefore = errors.length;
        
        try {
            // Trouver le bouton
            const element = await page.$(btn.selector);
            
            if (!element) {
                console.log(`   ❌ Bouton non trouvé: ${btn.selector}`);
                results.push({ name: btn.name, status: 'NOT_FOUND', logs: [], errors: [] });
                continue;
            }
            
            // Vérifier si visible
            const isVisible = await element.isIntersectingViewport();
            console.log(`   📍 Trouvé, visible: ${isVisible}`);
            
            // Cliquer via evaluate (fonctionne même si pas visible)
            await page.evaluate(el => el.click(), element);
            console.log(`   🖱️  Cliqué`);
            
            // Attendre effets
            await delay(500);
            
            // Analyser les nouveaux logs
            const newLogs = consoleLogs.slice(logsBefore);
            const newErrors = errors.slice(errorsBefore);
            
            console.log(`   📝 Logs générés: ${newLogs.length}`);
            newLogs.forEach(log => {
                const icon = log.type === 'error' ? '🔴' : log.type === 'warn' ? '🟡' : '📄';
                console.log(`      ${icon} [${log.type}] ${log.text.substring(0, 80)}`);
            });
            
            if (newErrors.length > 0) {
                console.log(`   ⚠️  Erreurs: ${newErrors.length}`);
                newErrors.forEach(e => console.log(`      🔴 ${e.substring(0, 80)}`));
            }
            
            // Vérifier si des logs attendus sont présents
            const logsText = newLogs.map(l => l.text.toLowerCase()).join(' ');
            const foundExpected = btn.expectLogs.some(exp => logsText.includes(exp.toLowerCase()));
            
            const status = newErrors.length > 0 ? 'ERROR' : 
                           newLogs.length === 0 ? 'NO_LOGS' :
                           foundExpected ? 'OK' : 'UNEXPECTED';
            
            console.log(`   ✅ Status: ${status}`);
            
            results.push({
                name: btn.name,
                status,
                logsCount: newLogs.length,
                errorsCount: newErrors.length,
                logs: newLogs.map(l => l.text.substring(0, 100)),
                errors: newErrors
            });
            
        } catch (err) {
            console.log(`   ❌ Erreur: ${err.message}`);
            results.push({ name: btn.name, status: 'EXCEPTION', error: err.message });
        }
    }
    
    await browser.close();
    
    // Résumé
    console.log('\n' + '='.repeat(60));
    console.log('📊 RÉSUMÉ\n');
    
    const ok = results.filter(r => r.status === 'OK').length;
    const notFound = results.filter(r => r.status === 'NOT_FOUND').length;
    const noLogs = results.filter(r => r.status === 'NO_LOGS').length;
    const errored = results.filter(r => r.status === 'ERROR' || r.status === 'EXCEPTION').length;
    
    console.log(`✅ OK:         ${ok}`);
    console.log(`❓ No logs:    ${noLogs}`);
    console.log(`🔍 Not found:  ${notFound}`);
    console.log(`❌ Erreurs:    ${errored}`);
    console.log(`📊 Total:      ${results.length}`);
    
    if (notFound > 0 || errored > 0) {
        console.log('\n⚠️  Problèmes détectés:');
        results.filter(r => r.status !== 'OK' && r.status !== 'NO_LOGS')
               .forEach(r => console.log(`   - ${r.name}: ${r.status}`));
    }
    
    return results;
}

runFunctionalTests().catch(console.error);
