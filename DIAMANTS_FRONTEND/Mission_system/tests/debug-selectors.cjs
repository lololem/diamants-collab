/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Debug test to check selectors
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
        await new Promise(r => setTimeout(r, 5000));
        
        // Test direct des sélecteurs
        const selectorTests = await page.evaluate(() => {
            const selectors = [
                'button[onclick*="launchMission"]',
                'button[onclick*="emergencyLand"]',
                'button[onclick*="takeoffAllDrones"]',
                'button[onclick*="landAllDrones"]',
                'button[onclick*="resetMission"]',
                'button[onclick*="resetCamera"]',
                'button[onclick*="topView"]',
                'button[onclick*="toggleMinimap"]'
            ];
            
            return selectors.map(sel => {
                const el = document.querySelector(sel);
                return {
                    selector: sel,
                    found: el !== null,
                    text: el ? el.textContent.trim() : null,
                    onclick: el ? el.getAttribute('onclick') : null
                };
            });
        });
        
        console.log('\n📋 TEST SÉLECTEURS:');
        selectorTests.forEach(t => {
            console.log(`${t.found ? '✅' : '❌'} ${t.selector}`);
            if (t.found) {
                console.log(`   Text: "${t.text}" | onclick: "${t.onclick}"`);
            }
        });
        
        // Test des objets window
        const windowObjects = await page.evaluate(() => {
            return {
                doctrineManager: typeof window.doctrineManager !== 'undefined',
                panelController: typeof window.panelController !== 'undefined',
                DIAMANTS_MINIMAP: typeof window.DIAMANTS_MINIMAP !== 'undefined',
                diamantsSystem: typeof window.diamantsSystem !== 'undefined'
            };
        });
        
        console.log('\n📋 OBJETS WINDOW:');
        Object.entries(windowObjects).forEach(([k, v]) => {
            console.log(`${v ? '✅' : '❌'} window.${k}`);
        });
        
    } catch (e) {
        console.error('Error:', e.message);
    }
    
    await browser.close();
})();
