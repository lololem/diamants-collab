/*
 * DIAMANTS — Simulation d'essaim de drones collaboratifs
 * Copyright (c) 2026 Loïc Lemasle
 *
 * Distribué sous PolyForm Noncommercial License 1.0.0.
 * Usage commercial interdit. Voir le fichier LICENSE à la racine.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * DIAMANTS — Metrics UI Connector
 * =================================
 * Câble les métriques calculées dans IntegratedController vers les éléments DOM.
 * 
 * Résout le problème des IDs incohérents entre index.html et le reste du code.
 */

export class MetricsUIConnector {
    constructor(diamantsSystem) {
        this.system = diamantsSystem;
        
        // Mapping: nom interne → ID DOM
        this.domMappings = {
            // Intelligence Metrics Panel
            'stigmergyValue': 'stigmergy_value',
            'totalIntelligence': 'total_intelligence_display',
            'emergenceLevel': 'emergence_level',
            'cohesionIndex': 'cohesion_index',
            'coordinationScore': 'coordination_score',
            'emergenceBar': 'emergence_bar',

            // Emergence sub-metrics
            'emM1': 'em_m1',
            'emM2': 'em_m2',
            'emM3': 'em_m3',

            // Mission Progress Panel
            'missionCoverage': 'mission_coverage',

            // Header status (IDs réels dans index.html)
            'droneCount': 'drone_count',
            'intelSummary': 'total_intelligence_display',
            'emergeSummary': 'emergence_display',
        };
        
        // Cache des éléments DOM
        this.elements = {};
        this.updateInterval = null;
        this.lastUpdate = 0;
        
        this.init();
    }

    /**
     * Initialisation - cache les références DOM
     */
    init() {
        Object.entries(this.domMappings).forEach(([key, domId]) => {
            const el = document.getElementById(domId);
            if (el) {
                this.elements[key] = el;
            }
            // Silently ignore missing elements — not all panels exist in every layout
        });
        
        console.log('📊 MetricsUIConnector initialisé', Object.keys(this.elements).length, 'éléments trouvés');
    }

    /**
     * Démarre la mise à jour périodique
     */
    start(intervalMs = 500) {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
        }
        
        this.updateInterval = setInterval(() => this.update(), intervalMs);
        this.update(); // Premier update immédiat
        
        console.log('📊 MetricsUI démarré (interval:', intervalMs, 'ms)');
    }

    /**
     * Arrête la mise à jour
     */
    stop() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    }

    /**
     * Mise à jour de tous les éléments UI
     */
    update() {
        const now = Date.now();
        if (now - this.lastUpdate < 100) return; // Debounce
        this.lastUpdate = now;

        const controller = this.system?.integratedController;
        const drones = this.system?.drones || [];
        
        if (!controller) return;

        // === Intelligence Metrics ===
        const metrics = controller.metrics || {};
        const diamantFormulas = controller.diamantFormulas;
        const sm = diamantFormulas?.swarmMetrics || {};
        
        // Count drones actually in flight (not IDLE on pad)
        const flyingDrones = drones.filter(d => d.state && d.state !== 'IDLE' && d.state !== 'LANDED');
        const anyFlying = flyingDrones.length > 0;
        
        // Stigmergie — couverture cellulaire du moteur (proxy visuel)
        const engine = controller.autonomousFlightEngine;
        if (engine?.visitedCells) {
            const visited = engine.visitedCells.size || 0;
            const bounds = engine.explorationBounds || 60;
            const cs = engine.cellSize || 3;
            const totalCells = Math.pow(2 * bounds / cs, 2) || 1;
            const stigPct = anyFlying ? Math.min(100, (visited / totalCells) * 100) : 0;
            this.setText('stigmergyValue', `${stigPct.toFixed(0)}%`);
        }

        // Total Intelligence (combinaison de plusieurs facteurs)
        const totalIntel = this.calculateTotalIntelligence(metrics, drones);
        const intelStr = anyFlying ? totalIntel.toFixed(0) : '0';
        this.setText('totalIntelligence', intelStr);
        this.setText('intelSummary', intelStr);
        
        // Emergence Level — phase transition detector (EI × significance-gated TE)
        const emergence = anyFlying ? (sm.emergence || 0) * 100 : 0;
        const detail = diamantFormulas?._emergenceDetail;
        const phase = detail?.phase || 'centralized';

        // Display numeric emergence value + phase badge
        const phaseLabels = {
            'centralized':   'CENTR',
            'transition':    'TRANS',
            'self-organized': 'AUTO',
        };
        const phaseLabel = phaseLabels[phase] || 'CENTR';
        const emergencePercent = anyFlying ? Math.round((detail?.smoothed ?? 0) * 100) : 0;
        this.setText('emergenceLevel', `${emergencePercent}% ${phaseLabel}`);

        // Color the emergence label based on phase
        const elEmergence = this.elements.emergenceLevel;
        if (elEmergence) {
            const phaseColors = {
                'centralized':   '#888888',
                'transition':    '#FFAA00',
                'self-organized': '#00FF88',
            };
            elEmergence.style.color = phaseColors[phase] || '#888888';
            elEmergence.style.fontSize = '11px';
        }

        // ── Emergence sub-metrics (M1/M2/M3) ──
        if (detail && anyFlying) {
            this.setText('emM1', `${detail.m1_coverage ?? 0}%`);
            this.setText('emM2', `${detail.m2_diversity ?? 0}%`);
            this.setText('emM3', `${detail.m3_infoFlow ?? 0}%`);
        } else {
            this.setText('emM1', '0%');
            this.setText('emM2', '0%');
            this.setText('emM3', '0%');
        }

        // Cohesion Index — scientific: Reynolds cohesion from swarmMetrics
        const cohesion = anyFlying ? (sm.cohesion || 0) * 100 : 0;
        this.setText('cohesionIndex', cohesion.toFixed(1));
        
        // Coordination Score — scientific: Vicsek alignment (= how coordinated)
        const coordination = anyFlying ? (sm.alignment || 0) * 100 : 0;
        this.setText('coordinationScore', coordination.toFixed(1));
        
        // Emergence bar (visual indicator — maps phase to bar width)
        if (this.elements.emergenceBar) {
            // Bar shows 0/50/100 for phase states (not the raw %)
            const barWidth = phase === 'centralized' ? 0
                           : phase === 'transition' ? 50
                           : 100;
            this.elements.emergenceBar.style.width = `${barWidth}%`;
            const barColors = {
                'centralized': '#444',
                'transition': '#FFAA00',
                'self-organized': '#00FF88',
            };
            this.elements.emergenceBar.style.background = barColors[phase] || '#444';
        }

        // === Mission Progress ===
        const coverage = metrics.coveragePercentage || 0;
        this.setText('missionCoverage', `${coverage.toFixed(1)}%`);
        
        // Targets (depuis mission manager ou scouting)
        const scouting = controller.collaborativeScouting;
        const targetsFound = scouting?.discoveredTargets?.length || 0;
        const targetsTotal = scouting?.totalTargets || '?';
        this.setText('targetsDiscovered', `${targetsFound}/${targetsTotal}`);
        
        // Efficiency (coverage / temps normalisé)
        const efficiency = this.calculateEfficiency(metrics, drones);
        this.setText('missionEfficiency', `${efficiency.toFixed(1)}%`);
        
        // Exploration speed (zones per minute)
        const speed = this.calculateExplorationSpeed(scouting);
        this.setText('explorationSpeed', `${speed.toFixed(2)} zones/min`);

        // === Header Status ===
        const flyingCount = drones.filter(d => d.state && d.state !== 'IDLE' && d.state !== 'LANDING').length;
        this.setText('droneCount', `${flyingCount}/${drones.length}`);
        
        // Header emergence numeric + phase icon
        const phaseIcons = { 'centralized': '⬜', 'transition': '🟡', 'self-organized': '🟢' };
        this.setText('emergeSummary', anyFlying
            ? `${emergencePercent}${phaseIcons[phase] || ''}`
            : '0');
    }

    /**
     * Calcul de l'intelligence collective totale I(t)
     * Uses scientific swarm metrics: coverage + alignment + emergence
     */
    calculateTotalIntelligence(metrics, drones) {
        const α = 0.35;  // Poids couverture
        const β = 0.35;  // Poids alignement (Vicsek)
        const γ = 0.30;  // Poids émergence
        
        const coverage = (metrics.coveragePercentage || 0) / 100;
        
        // Use scientific swarm metrics from DIAMANTS formulas
        const controller = this.system?.integratedController;
        const sm = controller?.diamantFormulas?.swarmMetrics || {};
        const alignment = sm.alignment || 0;
        const emergence = sm.emergence || 0;
        
        // Bonus pour nombre de drones actifs
        const activeRatio = drones.filter(d => d.state && d.state !== 'IDLE').length / Math.max(1, drones.length);
        
        const I = (α * coverage + β * alignment + γ * emergence) * 100 * activeRatio;
        return Math.min(100, I);
    }

    /**
     * Calcul de la cohésion de l'essaim (fallback)
     * Primary metric now comes from DiamantFormulas.computeReynoldsCohesion()
     * This method is kept for backward compatibility.
     * Ref: Reynolds C., SIGGRAPH 1987
     */
    calculateCohesion(drones) {
        if (drones.length < 2) return 0;
        
        // Centroïde
        let cx = 0, cy = 0, cz = 0, count = 0;
        drones.forEach(d => {
            if (d.position) {
                cx += d.position.x;
                cy += d.position.y;
                cz += d.position.z;
                count++;
            }
        });
        
        if (count === 0) return 0;
        cx /= count; cy /= count; cz /= count;
        
        // Écart-type des distances au centroïde
        let sumSqDist = 0;
        drones.forEach(d => {
            if (d.position) {
                const dist = Math.sqrt(
                    Math.pow(d.position.x - cx, 2) +
                    Math.pow(d.position.y - cy, 2) +
                    Math.pow(d.position.z - cz, 2)
                );
                sumSqDist += dist * dist;
            }
        });
        
        const stdDev = Math.sqrt(sumSqDist / count);
        
        // Normaliser: cohésion = 100 si stdDev ≈ 3m (formation idéale)
        // Diminue si trop dispersé ou trop compact
        const idealStdDev = 3.0;
        const cohesion = Math.max(0, 100 - Math.abs(stdDev - idealStdDev) * 15);
        
        return cohesion;
    }

    /**
     * Calcul de l'efficacité de mission
     */
    calculateEfficiency(metrics, drones) {
        const coverage = metrics.coveragePercentage || 0;
        const time = metrics.totalFlightTime || 1;
        const droneCount = drones.length || 1;
        
        // Efficacité = Coverage / (Temps × Drones)^0.5
        // Plus on couvre rapidement avec peu de drones, plus c'est efficace
        const efficiency = coverage / Math.sqrt(time * droneCount);
        
        // Normaliser à 0-100
        return Math.min(100, efficiency * 10);
    }

    /**
     * Vitesse d'exploration en zones/minute
     */
    calculateExplorationSpeed(scouting) {
        if (!scouting) return 0;
        
        const status = scouting.getMissionStatus?.() || {};
        const explored = status.zonesExplored || 0;
        const timeMin = (status.elapsedTime || 0) / 60;
        
        return timeMin > 0 ? explored / timeMin : 0;
    }

    /**
     * Helper pour mettre à jour le texte d'un élément
     */
    setText(key, value) {
        if (this.elements[key]) {
            this.elements[key].textContent = value;
        }
    }

    /**
     * Dispose
     */
    dispose() {
        this.stop();
        this.elements = {};
    }
}

// Export global
if (typeof window !== 'undefined') {
    window.MetricsUIConnector = MetricsUIConnector;
}
