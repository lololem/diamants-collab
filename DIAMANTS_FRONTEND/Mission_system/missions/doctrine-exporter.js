/**
 * DIAMANTS — Doctrine Exporter
 * ==============================
 * Pont CoLab → Autonome (Gap #1 du ROADMAP_SMA)
 *
 * Convertit l'état du DoctrineManager (frontend collaboratif)
 * en JSON compatible avec le DoctrineLoader du système autonome.
 *
 * Usage:
 *   import { exportDoctrine, downloadDoctrineJSON } from './doctrine-exporter.js';
 *   const json = exportDoctrine(doctrineManager);
 *   downloadDoctrineJSON(doctrineManager, 'ma_mission.json');
 */

import { DOCTRINES, COURSES_OF_ACTION } from './mission-doctrine.js';

// ─────────────────────────────────────────────
//  FSM Templates — Modèles de machine à états
//  par combinaison doctrine × COA
// ─────────────────────────────────────────────

/**
 * Templates FSM de base par type de doctrine.
 * Chaque template définit les doctrine_states[] pour le moteur autonome.
 * Le COA influence le behavior de l'état d'exploration.
 */
const FSM_TEMPLATES = {
    // ── Exploration / Coverage / Stigmergy ──
    exploration: [
        {
            state: 'PREFLIGHT',
            behavior: 'run_preflight_checks',
            transitions: [
                { condition: 'preflight_pass', target: 'ARM' },
                { condition: 'preflight_fail', target: 'ABORT' }
            ]
        },
        {
            state: 'ARM',
            behavior: 'arm_and_takeoff',
            transitions: [
                { condition: 'altitude_reached', target: 'EXPLORE' },
                { condition: 'arm_failed', target: 'ABORT' }
            ]
        },
        {
            state: 'EXPLORE',
            behavior: 'grid_exploration_with_ai',  // overridden by COA
            transitions: [
                { condition: 'obstacle_detected', target: 'AVOID' },
                { condition: 'person_detected', target: 'CAUTION' },
                { condition: 'coverage_complete', target: 'RTL' },
                { condition: 'battery_low', target: 'RTL' },
                { condition: 'time_limit', target: 'RTL' },
                { condition: 'geofence_warning', target: 'RTL' }
            ]
        },
        {
            state: 'AVOID',
            behavior: 'ai_replan_with_depth',
            transitions: [
                { condition: 'path_clear', target: 'EXPLORE' },
                { condition: 'obstacle_critical', target: 'EMERGENCY_STOP' }
            ]
        },
        {
            state: 'CAUTION',
            behavior: 'hover_and_assess',
            transitions: [
                { condition: 'person_clear', target: 'EXPLORE' },
                { condition: 'person_close', target: 'EMERGENCY_STOP' }
            ]
        },
        {
            state: 'RTL',
            behavior: 'return_to_launch',
            transitions: [
                { condition: 'home_reached', target: 'LAND' }
            ]
        },
        {
            state: 'LAND',
            behavior: 'precision_land_and_disarm',
            transitions: []
        },
        {
            state: 'EMERGENCY_STOP',
            behavior: 'full_stop_hold_position',
            transitions: [
                { condition: 'rc_override', target: 'RTL' }
            ]
        },
        {
            state: 'ABORT',
            behavior: 'disarm_safe',
            transitions: []
        }
    ],

    // ── Recherche (Search & Rescue) ──
    search: [
        {
            state: 'PREFLIGHT',
            behavior: 'run_preflight_checks',
            transitions: [
                { condition: 'preflight_pass', target: 'ARM' },
                { condition: 'preflight_fail', target: 'ABORT' }
            ]
        },
        {
            state: 'ARM',
            behavior: 'arm_and_takeoff',
            transitions: [
                { condition: 'altitude_reached', target: 'SEARCH' },
                { condition: 'arm_failed', target: 'ABORT' }
            ]
        },
        {
            state: 'SEARCH',
            behavior: 'grid_exploration_with_ai',  // overridden by COA
            transitions: [
                { condition: 'person_detected', target: 'INVESTIGATE' },
                { condition: 'obstacle_detected', target: 'AVOID' },
                { condition: 'coverage_complete', target: 'RTL' },
                { condition: 'battery_low', target: 'RTL' },
                { condition: 'time_limit', target: 'RTL' }
            ]
        },
        {
            state: 'INVESTIGATE',
            behavior: 'hover_and_assess',
            transitions: [
                { condition: 'person_confirmed', target: 'ORBIT_TARGET' },
                { condition: 'false_positive', target: 'SEARCH' }
            ]
        },
        {
            state: 'ORBIT_TARGET',
            behavior: 'orbit_poi',
            transitions: [
                { condition: 'orbit_complete', target: 'SEARCH' },
                { condition: 'battery_low', target: 'RTL' }
            ]
        },
        {
            state: 'AVOID',
            behavior: 'ai_replan_with_depth',
            transitions: [
                { condition: 'path_clear', target: 'SEARCH' },
                { condition: 'obstacle_critical', target: 'EMERGENCY_STOP' }
            ]
        },
        {
            state: 'RTL',
            behavior: 'return_to_launch',
            transitions: [
                { condition: 'home_reached', target: 'LAND' }
            ]
        },
        {
            state: 'LAND',
            behavior: 'precision_land_and_disarm',
            transitions: []
        },
        {
            state: 'EMERGENCY_STOP',
            behavior: 'full_stop_hold_position',
            transitions: [
                { condition: 'rc_override', target: 'RTL' }
            ]
        },
        {
            state: 'ABORT',
            behavior: 'disarm_safe',
            transitions: []
        }
    ],

    // ── Formation (Patrol / Escort) ──
    formation: [
        {
            state: 'PREFLIGHT',
            behavior: 'run_preflight_checks',
            transitions: [
                { condition: 'preflight_pass', target: 'ARM' },
                { condition: 'preflight_fail', target: 'ABORT' }
            ]
        },
        {
            state: 'ARM',
            behavior: 'arm_and_takeoff',
            transitions: [
                { condition: 'altitude_reached', target: 'PATROL' },
                { condition: 'arm_failed', target: 'ABORT' }
            ]
        },
        {
            state: 'PATROL',
            behavior: 'perimeter_patrol',
            transitions: [
                { condition: 'obstacle_detected', target: 'AVOID' },
                { condition: 'battery_low', target: 'RTL' },
                { condition: 'time_limit', target: 'RTL' },
                { condition: 'coverage_complete', target: 'RTL' }
            ]
        },
        {
            state: 'AVOID',
            behavior: 'ai_replan',
            transitions: [
                { condition: 'path_clear', target: 'PATROL' },
                { condition: 'obstacle_critical', target: 'EMERGENCY_STOP' }
            ]
        },
        {
            state: 'RTL',
            behavior: 'return_to_launch',
            transitions: [
                { condition: 'home_reached', target: 'LAND' }
            ]
        },
        {
            state: 'LAND',
            behavior: 'precision_land_and_disarm',
            transitions: []
        },
        {
            state: 'EMERGENCY_STOP',
            behavior: 'full_stop_hold_position',
            transitions: [
                { condition: 'rc_override', target: 'RTL' }
            ]
        },
        {
            state: 'ABORT',
            behavior: 'disarm_safe',
            transitions: []
        }
    ]
};

/**
 * Mapping COA → behavior name pour l'état principal d'exploration
 */
const COA_BEHAVIOR_MAP = {
    'grid':           'grid_exploration',
    'boustrophedon':  'grid_exploration',
    'spiral':         'spiral_exploration',
    'radial':         'radial_exploration',
    'perimeter':      'perimeter_patrol',
    'gradient':       'grid_exploration_with_ai',   // adaptatif = AI-guided
    'adaptive':       'grid_exploration_with_ai'
};

/**
 * Mapping doctrine frontend → template FSM + mission type
 */
const DOCTRINE_MAP = {
    'stigmergy': { template: 'exploration', missionType: 'exploration' },
    'swarm':     { template: 'exploration', missionType: 'exploration' },
    'coverage':  { template: 'exploration', missionType: 'exploration' },
    'formation': { template: 'formation',   missionType: 'patrol' },
    'search':    { template: 'search',      missionType: 'search' }
};


// ─────────────────────────────────────────────
//  Fonctions d'export
// ─────────────────────────────────────────────

/**
 * Convertit la zone frontend (center + size) en bounding box NED.
 * Le frontend utilise Cartesian (centerX, centerZ, sizeX, sizeZ).
 * L'autonome utilise NED (n_min, n_max, e_min, e_max).
 *
 * Convention : X frontend → East autonome, Z frontend → North autonome.
 */
function zoneToNED(zoneParams) {
    const halfX = zoneParams.sizeX / 2;
    const halfZ = zoneParams.sizeZ / 2;
    return {
        n_min: zoneParams.centerZ - halfZ,
        n_max: zoneParams.centerZ + halfZ,
        e_min: zoneParams.centerX - halfX,
        e_max: zoneParams.centerX + halfX
    };
}

/**
 * Construit les doctrine_states[] à partir du template et du COA sélectionné.
 */
function buildDoctrineStates(doctrineId, coaId) {
    const mapping = DOCTRINE_MAP[doctrineId] || DOCTRINE_MAP['stigmergy'];
    const template = FSM_TEMPLATES[mapping.template];

    // Deep clone
    const states = JSON.parse(JSON.stringify(template));

    // Remplacer le behavior de l'état principal par celui du COA
    const exploreBehavior = COA_BEHAVIOR_MAP[coaId] || 'grid_exploration_with_ai';
    const mainExploreState = states.find(s =>
        s.state === 'EXPLORE' || s.state === 'SEARCH' || s.state === 'PATROL'
    );
    if (mainExploreState) {
        mainExploreState.behavior = exploreBehavior;
    }

    return states;
}

/**
 * Exporte l'état complet du DoctrineManager en JSON doctrine autonome.
 *
 * @param {DoctrineManager} manager - Le DoctrineManager du frontend
 * @param {Object} [options] - Options supplémentaires
 * @param {string} [options.name] - Nom de la doctrine (défaut: auto-généré)
 * @param {string} [options.mode] - 'SIM' ou 'REAL' (défaut: 'SIM')
 * @param {string} [options.platform] - 'PX4' | 'ArduPilot' | 'Crazyflie' (défaut: 'PX4')
 * @param {string} [options.connection] - MAVLink connection string
 * @param {Object} [options.ai] - Overrides pour la section AI
 * @param {Object} [options.safety] - Overrides pour la section safety
 * @returns {Object} JSON doctrine compatible DoctrineLoader
 */
export function exportDoctrine(manager, options = {}) {
    const state = manager.getState();
    const doctrine = state.doctrine;
    const coa = state.coa;
    const zone = state.zone;
    const formation = state.formation;

    const doctrineId = doctrine.id;
    const coaId = coa.waypointGenerator || coa.id;
    const mapping = DOCTRINE_MAP[doctrineId] || DOCTRINE_MAP['stigmergy'];

    const mode = options.mode || 'SIM';
    const platform = options.platform || 'PX4';

    // Auto-generate name
    const name = options.name ||
        `${doctrine.name} + ${coa.name} — Export CoLab`;

    // Build NED zone
    const nedZone = zoneToNED(zone);

    // Build FSM states
    const doctrineStates = buildDoctrineStates(doctrineId, coaId);

    // Assemble full doctrine JSON
    const output = {
        doctrine: {
            name: name,
            version: '1.0.0',
            description: `Doctrine exportée depuis DIAMANTS CoLab.\n` +
                `Stratégie: ${doctrine.name} (${doctrine.description})\n` +
                `Tactique: ${coa.name} (${coa.description})`,
            author: 'DIAMANTS CoLab Export',
            mode: mode,
            sma_pillar: mode === 'REAL' ? 'OpsHub' : 'CoLab',
            tags: [
                doctrineId, coaId, mode.toLowerCase(),
                ...(doctrine.params.usePheromones ? ['stigmergy'] : []),
                'colab-export'
            ]
        },

        agent: {
            platform: platform,
            agent_id: 'colab_agent_1',
            agent_role: mapping.missionType === 'search' ? 'searcher' : 'explorer',
            connection: {
                type: 'udp',
                address: options.connection || 'udpin:0.0.0.0:14550'
            },
            origin: options.origin || {
                lat: 43.6047,
                lon: 1.4442,
                alt: 150.0
            }
        },

        ai: {
            decision_mode: 'RL_LLM',
            llm: {
                enabled: true,
                provider: 'ollama',
                model: 'qwen_drone_v7:latest',
                fallback_model: 'qwen_drone_v3:latest',
                endpoint: 'http://localhost:11434',
                temperature: 0.1,
                max_tokens: 300,
                timeout_s: 3.0
            },
            rl: {
                enabled: false,
                algorithm: 'PPO',
                model_path: 'models/rl/best_model.zip'
            },
            rules: {
                enabled: true,
                rules_file: 'data/llm_rules.json',
                priority: 'rules_first'
            },
            learning: {
                enabled: false,
                collect_experiences: false
            },
            ...(options.ai || {})
        },

        digital_twin: {
            enabled: mode === 'SIM',
            engine: mode === 'SIM' ? 'gazebo' : 'none',
            sync_to_real: mode === 'REAL'
        },

        perception: {
            camera: mode === 'REAL' ? 'oakd_pro_w' : 'gazebo_depth',
            oakd: {
                connection: 'usb',
                nn_model: 'yolov6-nano',
                nn_enabled: mode === 'REAL',
                depth_enabled: true,
                resolution: '1080p',
                fps: 30
            },
            mjpeg_stream: {
                enabled: true,
                port: 8891
            }
        },

        safety: {
            geofence: {
                n_min: nedZone.n_min - 5,
                n_max: nedZone.n_max + 5,
                e_min: nedZone.e_min - 5,
                e_max: nedZone.e_max + 5,
                alt_min: 1.0,
                alt_max: Math.max(zone.altitude * 2, 10.0)
            },
            max_velocity_ms: mode === 'REAL' ? 2.0 : 5.0,
            max_climb_rate_ms: 1.0,
            emergency_stop_m: zone.safetyDistance || 2.5,
            replan_distance_m: (zone.safetyDistance || 2.5) * 2.4,
            battery_critical_pct: 20,
            battery_low_pct: 30,
            heartbeat_timeout_s: 3.0,
            waypoint_timeout_s: 45.0,
            max_flight_time_s: 600,
            rc_kill_channel: 7,
            ...(options.safety || {})
        },

        mission: {
            type: mapping.missionType,
            zone: nedZone,
            altitude_m: zone.altitude,
            coverage_target: 0.85,
            cell_size_m: coa.params?.cellSize || 5.0,
            doctrine_states: doctrineStates
        },

        // Méta-données de la source CoLab
        colab_source: {
            doctrine_id: doctrineId,
            doctrine_params: doctrine.params,
            coa_id: coa.id,
            coa_params: coa.params,
            formation_id: formation?.id || null,
            zone_params: { ...zone },
            exported_at: new Date().toISOString()
        },

        dashboards: {
            console: { enabled: true, port: 8888 },
            digital_twin_3d: { enabled: true, port: 8889 },
            training: { enabled: false }
        }
    };

    return output;
}


/**
 * Exporte et télécharge le fichier JSON.
 */
export function downloadDoctrineJSON(manager, filename, options = {}) {
    const json = exportDoctrine(manager, options);
    const blob = new Blob(
        [JSON.stringify(json, null, 2)],
        { type: 'application/json' }
    );

    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = filename || `doctrine_${json.colab_source.doctrine_id}_${json.colab_source.coa_id}.json`;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(a.href);

    console.log(`📤 Doctrine exportée : ${a.download}`);
    return json;
}

/**
 * Exporte la doctrine sous forme de string JSON (pour WebSocket / API).
 */
export function exportDoctrineString(manager, options = {}) {
    return JSON.stringify(exportDoctrine(manager, options), null, 2);
}

/**
 * Retourne la liste des templates FSM disponibles.
 */
export function getAvailableTemplates() {
    return Object.keys(FSM_TEMPLATES).map(key => ({
        id: key,
        states: FSM_TEMPLATES[key].map(s => s.state)
    }));
}

/**
 * Retourne le mapping doctrine → behaviors.
 */
export function getCoaBehaviorMap() {
    return { ...COA_BEHAVIOR_MAP };
}

export default exportDoctrine;
