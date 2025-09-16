/**
 * DIAMANTS V3 - Shader Quality Manager (Simplifié)
 * Gère la sélection dynamique des shaders selon la qualité demandée
 */

// Mode silencieux global
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

/**
 * Niveaux de qualité disponibles
 */
export const QUALITY_LEVELS = {
    LOW: 'low',
    MEDIUM: 'medium',
    HIGH: 'high',
    ULTRA: 'ultra',
    DEV: 'dev',
    EXPERIMENTAL: 'experimental'
};

/**
 * Configuration des shaders par niveau de qualité
 */
const SHADER_CONFIGS = {
    [QUALITY_LEVELS.LOW]: {
        vertexShader: 'grass-vertex-stable.js',
        fragmentShader: 'grass-fragment-stable.js',
        description: '🟢 Performance - Simple et rapide',
        features: {
            wind: false,
            shadows: false,
            lod: 1,
            density: 0.3,
            grassPerChunk: 2000
        }
    },
    
    [QUALITY_LEVELS.MEDIUM]: {
        vertexShader: 'grass-vertex-stable.js',
        fragmentShader: 'grass-fragment-stable.js',
        description: '🟡 Équilibré - Bon compromis',
        features: {
            wind: true,
            shadows: true,
            lod: 2,
            density: 0.6,
            grassPerChunk: 5000
        }
    },
    
    [QUALITY_LEVELS.HIGH]: {
        vertexShader: 'grass-vertex-stable.js',
        fragmentShader: 'grass-fragment-stable.js',
        description: '🟠 Haute qualité - Détails avancés',
        features: {
            wind: true,
            shadows: true,
            lod: 3,
            density: 0.8,
            grassPerChunk: 8000
        }
    },
    
    [QUALITY_LEVELS.ULTRA]: {
        vertexShader: 'grass-vertex-ultra.js',
    fragmentShader: 'grass-fragment-stable.js',
        description: '🔴 Ultra - Effets visuels maximum',
        features: {
            wind: true,
            shadows: true,
            lod: 4,
            density: 1.4,
            grassPerChunk: 14000,
            windMultiplier: 3.0,
            drynessBoost: 0.6
        }
    },
    
    [QUALITY_LEVELS.DEV]: {
        vertexShader: 'grass-vertex-stable.js',
        fragmentShader: 'grass-fragment-stable.js',
        description: '🔧 Développement - Tests et debug (stable)',
        features: {
            wind: true,
            shadows: true,
            lod: 3,
            density: 0.8,
            grassPerChunk: 6000,
            debug: true
        }
    },
    
    [QUALITY_LEVELS.EXPERIMENTAL]: {
        vertexShader: 'grass-vertex-stable.js',
        fragmentShader: 'grass-fragment-stable.js',
        description: '⚗️ Expérimental - Nouvelles features (stable base)',
        features: {
            wind: true,
            shadows: true,
            lod: 5,
            density: 1.5,
            grassPerChunk: 15000,
            debug: true,
            experimental: true
        }
    }
};

// ===== Static import map to make Vite happy (no dynamic import vars) =====
// Each entry returns a Promise resolving to the module.
const SHADER_IMPORTS = {
    // Vertex shaders
    'grass-vertex-stable.js': () => import('./grass-vertex-stable.js'),
    'grass-vertex-low.js': () => import('./grass-vertex-low.js'),
    'grass-vertex-medium.js': () => import('./grass-vertex-medium.js'),
    'grass-vertex-ultra.js': () => import('./grass-vertex-ultra.js'),
    'grass-vertex-fixed.js': () => import('./grass-vertex-fixed.js'),

    // Fragment shaders
    'grass-fragment-stable.js': () => import('./grass-fragment-stable.js'),
    'grass-fragment-debug.js': () => import('./grass-fragment-debug.js'),
    'grass-fragment-low.js': () => import('./grass-fragment-low.js'),
    'grass-fragment-medium.js': () => import('./grass-fragment-medium.js'),
    'grass-fragment-ultra.js': () => import('./grass-fragment-ultra.js'),
    'grass-fragment-fixed.js': () => import('./grass-fragment-fixed.js')
};

function importShaderByName(name, fallback) {
    const importer = SHADER_IMPORTS[name];
    if (importer) return importer();
    warn(`Shader module not found in map: ${name}. Falling back to: ${fallback}`);
    return (SHADER_IMPORTS[fallback] || (() => Promise.reject(new Error(`Missing fallback shader: ${fallback}`))))();
}

/**
 * Gestionnaire de qualité des shaders
 */
export class ShaderQualityManager {
    constructor() {
        this.currentQuality = QUALITY_LEVELS.MEDIUM;
        this.autoQuality = false;
        this.performanceMetrics = {
            fps: 60,
            drawCalls: 0,
            triangles: 0
        };
    }

    /**
     * Retourne le niveau de qualité courant (compat pour UI)
     */
    getCurrentLevel() {
        return this.currentQuality;
    }

    /**
     * Obtient les shaders pour le niveau de qualité spécifié
     */
    async getShaders(quality = this.currentQuality) {
        const config = SHADER_CONFIGS[quality];
        if (!config) {
            warn(`Qualité "${quality}" non trouvée, utilisation de MEDIUM`);
            return await this.getShaders(QUALITY_LEVELS.MEDIUM);
        }

        // Import dynamique des shaders
        try {
            const [vertexModule, fragmentModule] = await Promise.all([
                importShaderByName(config.vertexShader, 'grass-vertex-stable.js'),
                importShaderByName(config.fragmentShader, 'grass-fragment-stable.js')
            ]);

            log(`🔄 Chargement shaders ${quality}:`, {
                vertex: config.vertexShader,
                fragment: config.fragmentShader
            });

            return {
                vertex: vertexModule.default,
                fragment: fragmentModule.default,
                description: config.description,
                features: config.features
            };
        } catch (error) {
            error(`❌ Erreur lors du chargement des shaders ${quality}:`, error);
            // Fallback vers stable
            if (quality !== QUALITY_LEVELS.MEDIUM) {
                log(`🔄 Fallback vers MEDIUM depuis ${quality}`);
                return await this.getShaders(QUALITY_LEVELS.MEDIUM);
            }
            throw error;
        }
    }

    /**
     * Change le niveau de qualité
     */
    async setQuality(quality) {
        log(`🔄 TENTATIVE DE CHANGEMENT VERS: ${quality}`);
        
        // Validation
        if (!Object.values(QUALITY_LEVELS).includes(quality)) {
            error(`❌ Qualité invalide: ${quality}`);
            quality = QUALITY_LEVELS.MEDIUM;
        }
        
        try {
            this.isLoading = true;
            
            const shaders = await this.getShaders(quality);
            
            // Test spécial pour ULTRA (APRÈS récupération des shaders)
            if (quality === QUALITY_LEVELS.ULTRA) {
                log('🔴 MODE ULTRA DÉTECTÉ - Tests spéciaux');
                log('🔍 Vertex shader ULTRA length:', shaders.vertex?.length);
                log('🔍 Fragment shader ULTRA length:', shaders.fragment?.length);
                
                // Test de validation des shaders ULTRA
                if (!shaders.vertex || shaders.vertex.length < 100) {
                    error('❌ ULTRA vertex shader trop court ou manquant');
                }
                if (!shaders.fragment || shaders.fragment.length < 100) {
                    error('❌ ULTRA fragment shader trop court ou manquant');
                }
            }
            
            if (!shaders || !shaders.vertex || !shaders.fragment) {
                error(`❌ Shaders invalides pour ${quality}:`, shaders);
                throw new Error(`Shaders invalides pour ${quality}`);
            }
            
            log(`✅ Shaders chargés pour ${quality}:`, {
                vertexLength: shaders.vertex?.length,
                fragmentLength: shaders.fragment?.length,
                description: shaders.description
            });

            this.currentQuality = quality;
            this.currentShaders = shaders;
            
        } catch (error) {
            error(`❌ Erreur lors du changement de qualité vers ${quality}:`, error);
            
            // Fallback vers MEDIUM si pas déjà MEDIUM
            if (quality !== QUALITY_LEVELS.MEDIUM) {
                log(`🔄 Fallback vers MEDIUM depuis ${quality}`);
                await this.setQuality(QUALITY_LEVELS.MEDIUM);
                return;
            }
            
            throw error;
        } finally {
            this.isLoading = false;
        }
    }

    /**
     * Active/désactive la qualité automatique basée sur les performances
     */
    setAutoQuality(enabled) {
        this.autoQuality = enabled;
        if (enabled) {
            this.optimizeQuality();
        }
    }

    /**
     * Met à jour les métriques de performance
     */
    updatePerformanceMetrics(fps, drawCalls, triangles) {
        this.performanceMetrics = { fps, drawCalls, triangles };
        
        if (this.autoQuality) {
            this.optimizeQuality();
        }
    }

    /**
     * Optimise automatiquement la qualité selon les performances
     */
    optimizeQuality() {
        const { fps } = this.performanceMetrics;
        
        let targetQuality = this.currentQuality;
        
        if (fps < 30) {
            targetQuality = QUALITY_LEVELS.LOW;
        } else if (fps < 45) {
            targetQuality = QUALITY_LEVELS.MEDIUM;
        } else if (fps >= 60) {
            targetQuality = QUALITY_LEVELS.HIGH;
        }
        
        if (targetQuality !== this.currentQuality) {
            this.setQuality(targetQuality);
            return true;
        }
        
        return false;
    }

    /**
     * Obtient les informations sur tous les niveaux disponibles
     */
    getQualityInfo() {
        return Object.entries(SHADER_CONFIGS).map(([level, config]) => ({
            level,
            description: config.description,
            features: config.features,
            current: level === this.currentQuality
        }));
    }

    /**
     * Détection automatique de la qualité optimale selon le GPU
     */
    async detectOptimalQuality(renderer) {
        // Rendre l'argument renderer optionnel et robuste
        let gl = null;
        let debugInfo = 'unknown';

        try {
            if (renderer && typeof renderer.getContext === 'function') {
                gl = renderer.getContext();
            }
        } catch (_) {}

        // Tenter d'extraire le nom du GPU via l'extension standard
        try {
            if (gl) {
                const dbgExt = gl.getExtension('WEBGL_debug_renderer_info');
                if (dbgExt) {
                    debugInfo = gl.getParameter(dbgExt.UNMASKED_RENDERER_WEBGL) || 'unknown';
                } else {
                    debugInfo = gl.getParameter(gl.RENDERER) || 'unknown';
                }
            }
        } catch (_) {
            // ignore et garder 'unknown'
        }
        
        // Analyse du GPU
        const gpuTier = this.analyzeGPU(String(debugInfo || 'unknown'));
        
        // Test de performance rapide
        const perfTest = await this.quickPerformanceTest(renderer);
        
        // Détermination de la qualité optimale
        let optimalQuality = QUALITY_LEVELS.MEDIUM;
        // Affiner selon le score
        if (perfTest.score >= 0.95 && gpuTier === 'high') {
            optimalQuality = QUALITY_LEVELS.ULTRA;
        } else if (perfTest.score >= 0.8 && gpuTier !== 'low') {
            optimalQuality = QUALITY_LEVELS.HIGH;
        } else if (perfTest.score < 0.45 || gpuTier === 'low') {
            optimalQuality = QUALITY_LEVELS.LOW;
        }
        
        log(`🔍 GPU détecté: ${debugInfo}`);
        log(`📊 Score performance: ${perfTest.score.toFixed(2)}`);
        log(`🎯 Qualité optimale suggérée: ${optimalQuality}`);
        
        return optimalQuality;
    }

    /**
     * Analyse le GPU pour déterminer sa catégorie
     */
    analyzeGPU(renderer) {
        const gpu = String(renderer || '').toLowerCase();
        
        // GPU haut de gamme
        if (gpu.includes('rtx') || gpu.includes('rx 6') || gpu.includes('rx 7')) {
            return 'high';
        }
        
        // GPU entrée de gamme
        if (gpu.includes('intel') || gpu.includes('gt ') || gpu.includes('mx ')) {
            return 'low';
        }
        
        return 'medium';
    }

    /**
     * Test rapide de performance
     */
    async quickPerformanceTest(renderer) {
        return new Promise((resolve) => {
            const startTime = performance.now();
            let frames = 0;
            
            const testFrame = () => {
                frames++;
                const elapsed = performance.now() - startTime;
                
                if (elapsed > 1000) { // Test d'1 seconde
                    const fps = frames;
                    const score = Math.min(fps / 60, 1);
                    resolve({ fps, score });
                } else {
                    requestAnimationFrame(testFrame);
                }
            };
            
            testFrame();
        });
    }
}

// Instance globale
export const shaderQualityManager = new ShaderQualityManager();

// Export par défaut
export default shaderQualityManager;
