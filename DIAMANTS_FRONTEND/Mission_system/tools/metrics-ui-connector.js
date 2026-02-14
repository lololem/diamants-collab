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
            'totalIntelligence': 'total_intelligence',
            'emergenceLevel': 'emergence_level',
            'cohesionIndex': 'cohesion_index',
            'coordinationScore': 'coordination_score',
            'emergenceBar': 'emergence_bar',
            
            // Mission Progress Panel
            'missionCoverage': 'mission_coverage',
            'targetsDiscovered': 'targets_discovered',
            'missionEfficiency': 'mission_efficiency',
            'explorationSpeed': 'exploration_speed',
            
            // Header status
            'droneCount': 'drone_count',
            'statusIndicator': 'status_indicator',
            'intelSummary': 'intel_summary',
            'emergeSummary': 'emerge_summary'
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
            } else {
                console.warn(`MetricsUI: Element #${domId} not found`);
            }
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
        
        // Total Intelligence (combinaison de plusieurs facteurs)
        const totalIntel = this.calculateTotalIntelligence(metrics, drones);
        this.setText('totalIntelligence', totalIntel.toFixed(1));
        
        // Emergence Level
        const emergence = (metrics.emergenceLevel || 0) * 100;
        this.setText('emergenceLevel', emergence.toFixed(1));
        
        // Cohesion Index (calculé depuis les positions des drones)
        const cohesion = this.calculateCohesion(drones);
        this.setText('cohesionIndex', cohesion.toFixed(1));
        
        // Coordination Score
        const coordination = (metrics.collaborationEfficiency || 0) * 100;
        this.setText('coordinationScore', coordination.toFixed(1));
        
        // Emergence bar (visual indicator)
        if (this.elements.emergenceBar) {
            this.elements.emergenceBar.style.width = `${emergence}%`;
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
        
        // Summary numbers for header
        this.setText('intelSummary', Math.round(totalIntel).toString());
        this.setText('emergeSummary', Math.round(emergence).toString());
    }

    /**
     * Calcul de l'intelligence collective totale I(t)
     * Formule: I(t) = α·Coverage + β·Coordination + γ·Emergence
     */
    calculateTotalIntelligence(metrics, drones) {
        const α = 0.4;  // Poids couverture
        const β = 0.3;  // Poids coordination
        const γ = 0.3;  // Poids émergence
        
        const coverage = (metrics.coveragePercentage || 0) / 100;
        const coordination = metrics.collaborationEfficiency || 0;
        const emergence = metrics.emergenceLevel || 0;
        
        // Bonus pour nombre de drones actifs
        const activeRatio = drones.filter(d => d.state && d.state !== 'IDLE').length / Math.max(1, drones.length);
        
        const I = (α * coverage + β * coordination + γ * emergence) * 100 * activeRatio;
        return Math.min(100, I);
    }

    /**
     * Calcul de la cohésion de l'essaim
     * Basé sur l'écart-type des distances au centroïde
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
