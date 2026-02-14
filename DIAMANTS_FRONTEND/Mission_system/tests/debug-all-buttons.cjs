/**
 * Debug all buttons in page
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
        
        // Get ALL buttons with onclick
        const allButtons = await page.evaluate(() => {
            const buttons = document.querySelectorAll('button[onclick]');
            return Array.from(buttons).map(b => ({
                text: b.textContent.trim().substring(0, 30),
                onclick: b.getAttribute('onclick'),
                visible: b.offsetParent !== null,
                classes: b.className
            }));
        });
        
        console.log('\n📋 TOUS LES BOUTONS AVEC ONCLICK:');
        allButtons.forEach((b, i) => {
            console.log(`${i+1}. [${b.visible ? 'VISIBLE' : 'HIDDEN'}] "${b.text}" → ${b.onclick}`);
        });
        
        // Count buttons
        console.log(`\nTotal: ${allButtons.length} boutons`);
        console.log(`Visibles: ${allButtons.filter(b => b.visible).length}`);
        console.log(`Cachés: ${allButtons.filter(b => !b.visible).length}`);
        
        // Check ros_interface panel visibility
        const panelState = await page.evaluate(() => {
            const panel = document.getElementById('ros_interface');
            if (!panel) return { exists: false };
            const style = window.getComputedStyle(panel);
            return {
                exists: true,
                display: style.display,
                visibility: style.visibility,
                left: style.left
            };
        });
        
        console.log('\n📋 PANEL ros_interface:');
        console.log(panelState);
        
    } catch (e) {
        console.error('Error:', e.message);
    }
    
    await browser.close();
})();
