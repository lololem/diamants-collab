/**
 * DIAMANTS - Fix Resize et Gestion Canvas
 * ==========================================
 * Correction des problèmes de redimensionnement et rendu canvas
 */

class CanvasResizeManager {
    constructor() {
        this.isResizing = false;
        this.resizeTimeout = null;
        this.observers = [];
        console.log('🖥️ CanvasResizeManager initialisé');
    }

    /**
     * Fix le problème de resize du canvas
     */
    fixCanvasResize(renderer, camera, container) {
        if (!renderer || !camera || !container) {
            console.warn('⚠️ FixCanvasResize: paramètres manquants', { renderer: !!renderer, camera: !!camera, container: !!container });
            return;
        }

        const resizeCanvas = () => {
            if (this.isResizing) return;
            this.isResizing = true;

            try {
                // Obtenir les vraies dimensions du container
                const rect = container.getBoundingClientRect();
                const width = Math.max(rect.width || container.clientWidth || 800, 400);
                const height = Math.max(rect.height || container.clientHeight || 600, 300);

                console.log('🔄 Resize canvas:', { width, height });

                // Mettre à jour camera
                camera.aspect = width / height;
                camera.updateProjectionMatrix();

                // Mettre à jour renderer
                renderer.setSize(width, height);
                renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));

                // Forcer le style du canvas
                const canvas = renderer.domElement;
                canvas.style.width = '100%';
                canvas.style.height = '100%';
                canvas.style.display = 'block';

                console.log('✅ Canvas redimensionné:', { width, height, aspect: camera.aspect });

            } catch (error) {
                console.error('❌ Erreur resize canvas:', error);
            } finally {
                this.isResizing = false;
            }
        };

        // Resize initial
        setTimeout(resizeCanvas, 100);

        // Observer les changements de taille
        if (window.ResizeObserver) {
            const resizeObserver = new ResizeObserver((entries) => {
                if (this.resizeTimeout) clearTimeout(this.resizeTimeout);
                this.resizeTimeout = setTimeout(resizeCanvas, 150);
            });
            resizeObserver.observe(container);
            this.observers.push(resizeObserver);
        }

        // Fallback window resize
        const windowResize = () => {
            if (this.resizeTimeout) clearTimeout(this.resizeTimeout);
            this.resizeTimeout = setTimeout(resizeCanvas, 150);
        };
        window.addEventListener('resize', windowResize);

        return { resizeCanvas, cleanup: () => {
            window.removeEventListener('resize', windowResize);
            this.observers.forEach(obs => obs.disconnect());
            this.observers = [];
        }};
    }

    /**
     * Assure que le container a des dimensions valides
     */
    ensureContainerDimensions(container) {
        if (!container) return false;

        const style = window.getComputedStyle(container);
        const hasWidth = parseInt(style.width) > 0;
        const hasHeight = parseInt(style.height) > 0;

        if (!hasWidth || !hasHeight) {
            console.warn('⚠️ Container sans dimensions valides - correction...');
            
            // Forcer des dimensions minimales
            if (!hasWidth) {
                container.style.width = '100%';
                container.style.minWidth = '800px';
            }
            if (!hasHeight) {
                container.style.height = '600px';
                container.style.minHeight = '600px';
            }
            
            container.style.position = 'relative';
            container.style.display = 'block';
            
            console.log('✅ Dimensions container forcées');
            return true;
        }

        return true;
    }
}

// Instance globale
window.canvasResizeManager = new CanvasResizeManager();

console.log('🖥️ Fix resize canvas chargé');
