/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Quick test to get detailed failure info
 */
const puppeteer = require('puppeteer');

(async () => {
    const browser = await puppeteer.launch({ 
        headless: 'new',
        args: ['--no-sandbox', '--disable-setuid-sandbox']
    });
    const page = await browser.newPage();
    
    try {
        await page.goto('http://localhost:3005', { waitUntil: 'domcontentloaded', timeout: 10000 });
        await new Promise(r => setTimeout(r, 7000));
        
        const results = await page.evaluate(() => {
            if (typeof window.runButtonTests === 'function') {
                const r = window.runButtonTests();
                return {
                    total: r.total,
                    passed: r.passed,
                    failed: r.failed,
                    failedTests: r.tests.filter(t => !t.passed).map(t => {
                        return { name: t.name, details: t.details || '' };
                    })
                };
            }
            return { error: 'runButtonTests not found' };
        });
        
        console.log('\n📊 RÉSULTATS DÉTAILLÉS:');
        console.log('Total:', results.total);
        console.log('Passés:', results.passed);
        console.log('Échoués:', results.failed);
        
        if (results.failedTests && results.failedTests.length > 0) {
            console.log('\n❌ TESTS ÉCHOUÉS:');
            results.failedTests.forEach(t => {
                console.log('  -', t.name, ':', t.details);
            });
        }
    } catch (e) {
        console.error('Error:', e.message);
    }
    
    await browser.close();
})();
