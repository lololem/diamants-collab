/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Debug DOM loading
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
        
        // Get raw HTML of ros_interface
        const rosInterfaceHTML = await page.evaluate(() => {
            const panel = document.getElementById('ros_interface');
            if (!panel) return 'Panel not found';
            return panel.innerHTML.substring(0, 3000);
        });
        
        console.log('📋 ROS_INTERFACE CONTENT (first 3000 chars):');
        console.log(rosInterfaceHTML);
        
        // Count all elements
        const counts = await page.evaluate(() => {
            return {
                allButtons: document.querySelectorAll('button').length,
                btnRos: document.querySelectorAll('.btn_ros').length,
                withOnclick: document.querySelectorAll('[onclick]').length,
                rosPanels: document.querySelectorAll('.ros_panel').length
            };
        });
        
        console.log('\n📋 ELEMENT COUNTS:');
        console.log(counts);
        
    } catch (e) {
        console.error('Error:', e.message);
    }
    
    await browser.close();
})();
