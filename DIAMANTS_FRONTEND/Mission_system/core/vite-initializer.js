/**
 * Alternative initializer for DIAMANTS V3 with Vite compatibility
 */

console.log('🚀 === DIAMANTS V3 VITE INITIALIZER ===');

// Import Three.js as module
import * as THREE from 'three';

// Make THREE available globally for legacy code
window.THREE = THREE;
window.THREE_READY = true;

console.log('✅ Three.js loaded as module, version:', THREE.REVISION);

// Dispatch ready event immediately and after a small delay
window.dispatchEvent(new Event('threeReady'));

// Wait a moment then load original initializer
setTimeout(async () => {
    try {
        if (!window.SILENT_MODE) {
            console.log('📦 Loading original DIAMANTS initializer...');
        }
        await import('./diamants-initializer.js');
        console.log('✅ Original DIAMANTS initializer loaded successfully');
    } catch (error) {
        console.error('❌ Error loading original initializer:', error);
        
        // Fallback: try to start manually
        console.log('🔄 Trying manual initialization...');
        if (window.diamantsInitializer) {
            window.diamantsInitializer.initialize().catch(err => {
                console.error('💥 Manual initialization failed:', err);
            });
        }
    }
}, 100);
