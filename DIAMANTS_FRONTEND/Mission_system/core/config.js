/**
 * CONFIG - Configuration Système DIAMANTS
 * Extrait et optimisé depuis SMA.html
 * 
 * Configuration centralisée pour performance et comportement réaliste
 */

export const CONFIG = {
    // === PARAMÈTRES GÉNÉRAUX ===
    maxDrones: Infinity,        // Unlimited intelligent swarm
    zoneSize: 120,             // Zone de mission (mètres)
    explorationGrid: 20,        // Grille d'exploration (précision)
    
    // === COMPORTEMENT ESSAIM ===
    swarmCohesion: 1.5,        // Cohésion réduite pour indépendance
    swarmSeparation: 5.0,      // Séparation forte anti-redondance
    swarmAlignment: 0.8,       // Alignement minimal pour autonomie
    swarmRadius: 45,           // Rayon coordination globale
    
    // === PARAMÈTRES DIAMANTS ===
    magneticForce: 1.2,        // Force magnétique réduite
    autonomyPower: 4.5,        // Autonomie maximale exploration
    searchSpeed: 3.2,          // Vitesse optimisée couverture
    redundancyRadius: 15,      // Zone anti-redondance
    
    // === FEATURES AVANCÉES ===
    sectorAssignment: true,     // Assignment secteurs actif
    adaptivePhases: true,       // Phases adaptatives
    completionThreshold: 0.95,  // Seuil completion changement phase
    
    // === PERFORMANCE ===
    targetFPS: 60,
    maxAnimationTime: 16,      // ms - cible <16ms pour 60fps
    
    // === DEBUG ET LOGGING ===
    logLevel: 'INFO',
    enablePerformanceLogging: true,
    enableConsoleNinja: true,
    consoleNinjaPort: 5551,
    
    // === SCOUTING SAR ===
    sarSpacing: 18,            // Espacement SAR standard (mètres)
    scoutingAltitude: 2.0,     // Altitude scouting (mètres)
    targetDetectionRange: 15,   // Portée détection cibles
    
    // === COMMUNICATION ===
    communicationRange: 30,     // Portée communication inter-drones
    broadcastRadius: 50,       // Rayon diffusion messages
    messageTimeout: 10000,     // Timeout messages (ms)
    
    // === ENVIRONNEMENT ===
    forestDensity: 30,         // Nombre d'arbres
    scoutingTargets: 15,       // Nombre cibles scouting
    minimumTreeDistance: 15,   // Distance minimale entre arbres
    
    // === RENDU 3D ===
    enableShadows: true,
    shadowMapSize: 1024,
    fogNear: 80,
    fogFar: 200,
    
    // === PATHS ===
    meshPath: './assets/crazyflie/meshes/',
    texturePath: './assets/textures/',
    
    // === MÉTRIQUES UI ===
    uiUpdateFrequency: 100,    // ms entre mises à jour UI
    metricsHistorySize: 100    // Nombre points historique métriques
};

// Types de drones optimisés pour scouting collaboratif
export const DRONE_TYPES = {
    SCOUT: {
        name: "Scout Collaboratif",
        size: 0.6,
        speed: 8,              // Vitesse réduite pour scouting précis
        color: 0x00FF88,
        role: "exploration",
        sensorRange: 15,       // Portée adaptée niveau troncs
        agility: 2.0
    },
    HEAVY: {
        name: "Porteur Lourd",
        size: 1.0,
        speed: 6,              // Plus lent pour transport précis
        color: 0xFF6600,
        role: "transport",
        sensorRange: 12,
        agility: 0.8
    },
    STEALTH: {
        name: "Furtif",
        size: 0.4,
        speed: 10,             // Rapide mais contrôlé
        color: 0x4444FF,
        role: "reconnaissance",
        sensorRange: 20,       // Bonne portée reconnaissance
        agility: 2.2
    },
    LEADER: {
        name: "Coordinateur",
        size: 0.8,
        speed: 7,              // Vitesse modérée coordination
        color: 0xFF0088,
        role: "coordination",
        sensorRange: 25,       // Grande portée coordination
        agility: 1.5
    }
};

// Patterns de recherche disponibles
export const SEARCH_PATTERNS = [
    'grid', 
    'boustrophedon', 
    'spiral', 
    'coverage', 
    'follow_leader', 
    'random'
];

// Phases de mission
export const MISSION_PHASES = {
    INITIALIZATION: 'INITIALIZATION',
    DISPERSION: 'DISPERSION',
    EXPLORATION: 'EXPLORATION',
    CONSOLIDATION: 'CONSOLIDATION',
    COMPLETION: 'COMPLETION'
};

// Types de messages inter-drones
export const MESSAGE_TYPES = {
    DISCOVERY: 'DISCOVERY',
    STRATEGY_CHANGE: 'STRATEGY_CHANGE',
    EXPERTISE_OFFER: 'EXPERTISE_OFFER',
    COORDINATION_REQUEST: 'COORDINATION_REQUEST',
    EMERGENCY: 'EMERGENCY',
    STATUS_UPDATE: 'STATUS_UPDATE'
};

// Validateur de configuration
export function validateConfig() {
    console.log('🔧 Validation configuration DIAMANTS...');
    
    const errors = [];
    
    // Vérifications critiques
    if (CONFIG.zoneSize <= 0) errors.push('zoneSize doit être > 0');
    if (CONFIG.explorationGrid <= 0) errors.push('explorationGrid doit être > 0');
    if (CONFIG.targetFPS <= 0) errors.push('targetFPS doit être > 0');
    
    // Vérifications logiques
    if (CONFIG.swarmSeparation >= CONFIG.swarmRadius) {
        errors.push('swarmSeparation doit être < swarmRadius');
    }
    
    if (CONFIG.redundancyRadius >= CONFIG.communicationRange) {
        errors.push('redundancyRadius doit être < communicationRange');
    }
    
    if (errors.length > 0) {
        console.error('❌ Erreurs de configuration:', errors);
        return false;
    }
    
    console.log('✅ Configuration validée');
    return true;
}

/**
 * Load fleet configuration from the API.
 * Updates CONFIG, DRONE_TYPES, and exposes window.FLEET_CONFIG.
 * @returns {Promise<Object>} The fleet config
 */
export async function loadFleetConfig(apiUrl = 'http://localhost:8000') {
    try {
        const ctrl = new AbortController();
        const timer = setTimeout(() => ctrl.abort(), 2000);
        const resp = await fetch(`${apiUrl}/api/fleet`, { signal: ctrl.signal });
        clearTimeout(timer);
        if (!resp.ok) throw new Error(`Fleet API returned ${resp.status}`);
        const fleet = await resp.json();
        
        // Update CONFIG
        CONFIG.maxDrones = fleet.numDrones || CONFIG.maxDrones;
        
        // Merge drone types from backend
        if (fleet.droneTypes) {
            for (const [key, val] of Object.entries(fleet.droneTypes)) {
                const upperKey = key.toUpperCase();
                DRONE_TYPES[upperKey] = {
                    name: val.label || key,
                    size: val.size || 0.6,
                    speed: val.speed || 8,
                    color: parseInt((val.color || '#00FF88').replace('#', '0x')),
                    role: val.role || 'exploration',
                    sensorRange: val.sensorRange || 15,
                    agility: val.agility || 1.0,
                };
            }
        }
        
        // Expose globally
        window.FLEET_CONFIG = fleet;
        console.log(`✅ Fleet config loaded: ${fleet.numDrones} drones`);
        return fleet;
    } catch (e) {
        // Fleet config API unavailable is expected when no backend — silent fallback
        // Fallback: try loading from local file
        try {
            const ctrl2 = new AbortController();
            const timer2 = setTimeout(() => ctrl2.abort(), 2000);
            const localResp = await fetch('/fleet_config.json', { signal: ctrl2.signal });
            clearTimeout(timer2);
            if (localResp.ok) {
                const local = await localResp.json();
                window.FLEET_CONFIG = { numDrones: local.drones?.length || 8, drones: local.drones || [] };
                CONFIG.maxDrones = window.FLEET_CONFIG.numDrones;
                console.log(`✅ Fleet config loaded from local file: ${window.FLEET_CONFIG.numDrones} drones`);
                return window.FLEET_CONFIG;
            }
        } catch (_) { /* ignore */ }
        
        window.FLEET_CONFIG = { numDrones: 8, drones: [] };
        return window.FLEET_CONFIG;
    }
}

export default CONFIG;
