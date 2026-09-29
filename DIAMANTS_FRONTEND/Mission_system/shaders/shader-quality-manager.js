/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS - Shader Quality Manager (Simplifié)
 * Gère la sélection dynamique des shaders selon la qualité demandée
 */

// Mode silencieux global
if (typeof window.SILENT_MODE === 'undefined') window.SILENT_MODE = true;

// Fonctions de logging
const log = (...args) => { if (!window.SILENT_MODE) console.log(...args); };
const warn = (...args) => { if (!window.SILENT_MODE) console.warn(...args); };
const error = (...args) => console.error(...args);

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
        fragmentShader: 'grass-fragment-ultra.js',
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

/* A LOAD THAT FAILS ONCE IS NOT A LOAD THAT FAILS.
 *
 * In development each shader is a module served on its own. When the quality
 * changes while the page is still loading, several imports leave together and
 * one of them can come back in error — the dev server transforms and serves
 * modules on demand. The console then showed « ❌ Erreur lors du chargement des
 * shaders low » on an application that was otherwise fine: the caller falls
 * back to MEDIUM.
 *
 * The file is there — reloaded alone, it answers. So we try once more, after a
 * breath, before calling it a failure. The built bundle never meets this case:
 * the shaders are already inside it. */
async function importWithRetry(importer, what) {
    try {
        return await importer();
    } catch (e) {
        /* Three attempts, with room to breathe: on the very first load the dev
         * server is still transforming modules and 120 ms is not always
         * enough. The exact cause, read from the message: « Failed to fetch
         * dynamically imported module » — the request is ABORTED during the
         * module loading storm, not refused (no 4xx answer). The same failure
         * hits other late imports. So the last attempt waits for the storm to
         * pass. */
        for (const wait of [150, 500, 1500]) {
            await new Promise(r => setTimeout(r, wait));
            try { return await importer(); } catch (_) { /* try again */ }
        }
        warn(`Shader ${what} unavailable after four attempts: ${e && e.message || e}`);
        throw e;
    }
}

function importShaderByName(name, fallback) {
    const importer = SHADER_IMPORTS[name];
    if (importer) return importWithRetry(importer, name);
    warn(`Shader module not found in map: ${name}. Falling back to: ${fallback}`);
    const fb = SHADER_IMPORTS[fallback];
    if (!fb) return Promise.reject(new Error(`Missing fallback shader: ${fallback}`));
    return importWithRetry(fb, fallback);
}

/**
 * Gestionnaire de qualité des shaders
 */
export class ShaderQualityManager {
    constructor() {
        this.currentQuality = QUALITY_LEVELS.MEDIUM; // safe default − overridden by autoDetect
        this.autoQuality = false;
        this.performanceMetrics = {
            fps: 60,
            drawCalls: 0,
            triangles: 0
        };
    }

    /**
     * Auto-detect optimal quality on startup.
     * Should be called once after the WebGL renderer is available.
     * Falls back gracefully if detection fails.
     */
    async autoDetectOnInit(renderer) {
        try {
            const quality = await this.detectOptimalQuality(renderer);
            log(`🎯 Auto-détection qualité: ${quality}`);
            this.currentQuality = quality;
            return quality;
        } catch (e) {
            console.warn('⚠️ Auto-détection qualité échouée, MEDIUM par défaut', e);
            this.currentQuality = QUALITY_LEVELS.MEDIUM;
            return QUALITY_LEVELS.MEDIUM;
        }
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
        } catch (err) {
            /* A RED ERROR FOR A SITUATION THAT IS CAUGHT IS A FALSE SIGNAL.
             *
             * When a shader does not arrive — request aborted during the module
             * loading storm, in development only — we fall back to MEDIUM and
             * the grass shows. Writing « ❌ Erreur » suggests a breakdown in an
             * otherwise clean console. We say it for what it is: a fallback,
             * with its cause. If the fallback itself fails, that is a real
             * error, and it propagates. */
            if (quality !== QUALITY_LEVELS.MEDIUM) {
                warn(`Shaders « ${quality} » unavailable (${err && err.message || err}) — falling back to MEDIUM`);
                return await this.getShaders(QUALITY_LEVELS.MEDIUM);
            }
            console.error(`❌ MEDIUM shaders unavailable, no fallback left:`, err);
            throw err;
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
            
        } catch (err) {
            console.error(`❌ Erreur lors du changement de qualité vers ${quality}:`, err);
            
            // Fallback vers MEDIUM si pas déjà MEDIUM
            if (quality !== QUALITY_LEVELS.MEDIUM) {
                log(`🔄 Fallback vers MEDIUM depuis ${quality}`);
                await this.setQuality(QUALITY_LEVELS.MEDIUM);
                return;
            }
            
            throw err;
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
     * Détection automatique de la qualité optimale selon le GPU + device
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

        // Device heuristics
        const isMobile = /android|iphone|ipad|mobile/i.test(navigator.userAgent);
        const cores = navigator.hardwareConcurrency || 2;
        const memoryGB = (navigator.deviceMemory || 4);
        const screenPixels = window.screen.width * window.screen.height * (window.devicePixelRatio || 1);

        // Test de performance rapide
        const perfTest = await this.quickPerformanceTest(renderer);

        // Composite score (0‒1)
        let score = perfTest.score; // fps-based

        // Adjust by device characteristics
        if (isMobile) score *= 0.6;                          // mobile penalty
        if (cores <= 2) score *= 0.7;                         // weak CPU
        if (memoryGB <= 2) score *= 0.7;                      // low memory
        if (gpuTier === 'high') score = Math.max(score, 0.9); // high-end GPU overrides
        if (gpuTier === 'low')  score = Math.min(score, 0.5); // integrated caps at medium

        // Determine quality
        let optimalQuality;
        if (score >= 0.85) {
            optimalQuality = QUALITY_LEVELS.ULTRA;
        } else if (score >= 0.6) {
            optimalQuality = QUALITY_LEVELS.HIGH;
        } else if (score >= 0.35) {
            optimalQuality = QUALITY_LEVELS.MEDIUM;
        } else {
            optimalQuality = QUALITY_LEVELS.LOW;
        }

        log(`🔍 GPU: ${debugInfo} (${gpuTier})`);
        log(`📊 Device: mobile=${isMobile}, cores=${cores}, mem=${memoryGB}GB, px=${(screenPixels/1e6).toFixed(1)}Mpx`);
        log(`📊 Score: ${score.toFixed(2)} → ${optimalQuality}`);
        
        return optimalQuality;
    }

    /**
     * Analyse le GPU pour déterminer sa catégorie
     */
    analyzeGPU(renderer) {
        const gpu = String(renderer || '').toLowerCase();

        // Mobile GPU → low
        if (/adreno|mali|powervr|apple gpu|sgx/.test(gpu)) return 'low';

        // High-end discrete GPUs
        if (/rtx\s*(30|40|50)|rx\s*(6[89]|7)|radeon\s*(rx\s*)?7|geforce\s*gtx\s*1[0-9]{3}/.test(gpu)) return 'high';

        // Mid-range discrete
        if (/gtx|rx\s*(5[0-9]{2,3}|6[0-7])|radeon|geforce/.test(gpu)) return 'medium';

        // Intel integrated
        if (/intel|mesa|llvmpipe|swiftshader|swrast/.test(gpu)) return 'low';

        // Apple Silicon (M1/M2/M3) — medium-high
        if (/apple\s*m[1-4]/.test(gpu)) return 'medium';

        return 'medium';
    }

    /**
     * Test rapide de performance
     */
    async quickPerformanceTest(renderer) {
        return new Promise((resolve) => {
            const startTime = performance.now();
            let frames = 0;
            let settled = false;

            const done = (fps) => {
                if (settled) return;
                settled = true;
                clearTimeout(safety);
                resolve({ fps, score: Math.min(fps / 60, 1) });
            };

            // Safety net: rAF can be throttled/paused (background tab, headless),
            // which would otherwise hang init forever. Resolve with a medium-tier
            // estimate if the test hasn't completed via rAF within 2s wall-clock.
            const safety = setTimeout(() => {
                const elapsed = performance.now() - startTime;
                const fps = elapsed > 0 ? Math.round((frames / elapsed) * 1000) : 45;
                done(fps || 45);
            }, 2000);

            const testFrame = () => {
                frames++;
                const elapsed = performance.now() - startTime;
                if (elapsed > 1000) { // Test d'1 seconde
                    done(frames);
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
