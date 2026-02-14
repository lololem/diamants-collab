/**
 * TEST FONCTIONNEL IN-BROWSER
 * Coller ce code dans la console du navigateur pour tester tous les boutons
 */

const ButtonFunctionalTest = {
    results: [],
    logs: [],
    
    // Capturer les console.log
    startCapture() {
        this.originalLog = console.log;
        this.originalWarn = console.warn;
        this.originalError = console.error;
        
        console.log = (...args) => {
            this.logs.push({ type: 'log', text: args.join(' ') });
            this.originalLog.apply(console, args);
        };
        console.warn = (...args) => {
            this.logs.push({ type: 'warn', text: args.join(' ') });
            this.originalWarn.apply(console, args);
        };
        console.error = (...args) => {
            this.logs.push({ type: 'error', text: args.join(' ') });
            this.originalError.apply(console, args);
        };
    },
    
    stopCapture() {
        console.log = this.originalLog;
        console.warn = this.originalWarn;
        console.error = this.originalError;
    },
    
    async delay(ms) {
        return new Promise(r => setTimeout(r, ms));
    },
    
    async testButton(name, selector, expectLogs) {
        console.log(`\n▶️ Test: ${name}`);
        
        // Capture logs avant clic
        const logsBefore = this.logs.length;
        
        // Trouver element
        const el = document.querySelector(selector);
        if (!el) {
            console.log(`   ❌ Non trouvé: ${selector}`);
            this.results.push({ name, status: 'NOT_FOUND', error: 'Element non trouvé' });
            return;
        }
        
        console.log(`   📍 Trouvé: ${el.tagName} "${el.textContent.substring(0, 30)}"`);
        
        try {
            // Clic
            el.click();
            console.log(`   🖱️ Cliqué`);
            
            await this.delay(300);
            
            // Analyser logs générés
            const newLogs = this.logs.slice(logsBefore);
            console.log(`   📝 Logs générés: ${newLogs.length}`);
            newLogs.forEach(l => {
                const icon = l.type === 'error' ? '🔴' : l.type === 'warn' ? '🟡' : '📄';
                console.log(`      ${icon} ${l.text.substring(0, 60)}`);
            });
            
            // Vérifier logs attendus
            const logsText = newLogs.map(l => l.text.toLowerCase()).join(' ');
            const hasExpected = expectLogs.some(e => logsText.includes(e.toLowerCase()));
            const hasError = newLogs.some(l => l.type === 'error');
            
            const status = hasError ? 'ERROR' : newLogs.length === 0 ? 'NO_LOGS' : hasExpected ? 'OK' : 'UNEXPECTED';
            console.log(`   ✅ Status: ${status}`);
            
            this.results.push({ name, status, logsCount: newLogs.length, hasExpected });
            
        } catch (err) {
            console.log(`   ❌ Erreur: ${err.message}`);
            this.results.push({ name, status: 'EXCEPTION', error: err.message });
        }
    },
    
    async runAll() {
        console.clear();
        console.log('🧪 TEST FONCTIONNEL IN-BROWSER');
        console.log('='.repeat(50));
        
        this.results = [];
        this.logs = [];
        this.startCapture();
        
        // Afficher le panel si nécessaire
        const panel = document.getElementById('ros_interface');
        if (panel) {
            panel.style.display = 'block';
            console.log('📌 Panel ros_interface rendu visible');
        }
        
        const BUTTONS = [
            // Mission
            { name: 'Launch Mission', selector: '[data-action="launchMission"], button[onclick*="launchMission"]', expect: ['mission', 'launch', 'start', 'minimap'] },
            { name: 'Reset Mission', selector: '[data-action="resetMission"], button[onclick*="resetMission"]', expect: ['reset'] },
            { name: 'Emergency Land', selector: '[data-action="emergencyLand"], button[onclick*="emergencyLand"]', expect: ['emergency', 'land', 'stop'] },
            { name: 'Takeoff All', selector: '[data-action="takeoffAllDrones"], button[onclick*="takeoffAllDrones"]', expect: ['takeoff'] },
            { name: 'Land All', selector: '[data-action="landAllDrones"], button[onclick*="landAllDrones"]', expect: ['land'] },
            
            // Camera
            { name: 'Reset Camera', selector: '[data-action="resetCamera"], button[onclick*="resetCamera"]', expect: ['camera', 'reset'] },
            { name: 'Top View', selector: '[data-action="topView"], button[onclick*="topView"]', expect: ['top', 'view'] },
            { name: 'Zoom Swarm', selector: '[data-action="zoomToSwarm"], button[onclick*="zoomToSwarm"]', expect: ['zoom', 'swarm'] },
            { name: 'Toggle Follow', selector: '[data-action="toggleFollowMode"], button[onclick*="toggleFollowMode"]', expect: ['follow'] },
            
            // Display
            { name: 'Toggle Minimap', selector: '[data-action="toggleMinimap"], button[onclick*="toggleMinimap"]', expect: ['minimap'] },
            { name: 'Toggle Debug', selector: '[data-action="toggleDebugPanels"], button[onclick*="toggleDebugPanels"]', expect: ['debug'] },
        ];
        
        for (const btn of BUTTONS) {
            await this.testButton(btn.name, btn.selector, btn.expect);
            await this.delay(200);
        }
        
        this.stopCapture();
        this.printSummary();
    },
    
    printSummary() {
        console.log('\n' + '='.repeat(50));
        console.log('📊 RÉSUMÉ\n');
        
        const ok = this.results.filter(r => r.status === 'OK').length;
        const noLogs = this.results.filter(r => r.status === 'NO_LOGS').length;
        const notFound = this.results.filter(r => r.status === 'NOT_FOUND').length;
        const errors = this.results.filter(r => r.status === 'ERROR' || r.status === 'EXCEPTION').length;
        
        console.log(`✅ OK:         ${ok}`);
        console.log(`❓ No logs:    ${noLogs}`);
        console.log(`🔍 Not found:  ${notFound}`);
        console.log(`❌ Erreurs:    ${errors}`);
        console.log(`📊 Total:      ${this.results.length}`);
        
        if (notFound > 0 || errors > 0) {
            console.log('\n⚠️ Problèmes:');
            this.results
                .filter(r => r.status === 'NOT_FOUND' || r.status === 'ERROR' || r.status === 'EXCEPTION')
                .forEach(r => console.log(`   - ${r.name}: ${r.status} ${r.error || ''}`));
        }
        
        // Tableau détaillé
        console.table(this.results.map(r => ({
            Bouton: r.name,
            Status: r.status,
            Logs: r.logsCount || 0
        })));
    }
};

// Lancer automatiquement
ButtonFunctionalTest.runAll();

// Export pour utilisation manuelle
window.ButtonTest = ButtonFunctionalTest;
