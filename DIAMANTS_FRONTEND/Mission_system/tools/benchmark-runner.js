/**
 * DIAMANTS — Benchmark Runner
 * =============================
 * Système de benchmark pour mesurer l'impact de la coopération multi-drone.
 * 
 * Génère des données pour tracer des courbes:
 *   - Axe X: Nombre de drones (1, 2, 4, 8)
 *   - Axe Y: Métriques de performance (Coverage, Emergence, Efficiency)
 * 
 * Usage:
 *   const benchmark = new BenchmarkRunner(diamantsSystem);
 *   const results = await benchmark.runScalingExperiment([1, 2, 4, 8]);
 *   benchmark.exportChartData(); // JSON pour graphiques
 */

export class BenchmarkRunner {
    constructor(diamantsSystem) {
        this.system = diamantsSystem;
        this.results = new Map();
        
        // Configuration des expériences
        this.config = {
            // Durée de chaque run en secondes
            runDuration: 60,
            // Nombre de répétitions par configuration
            repetitions: 3,
            // Intervalle d'échantillonnage métriques (ms)
            sampleInterval: 1000,
            // Métriques à collecter
            metricsToCollect: [
                'coveragePercentage',
                'emergenceLevel',
                'collaborationEfficiency',
                'coordinationIndex',
                'missionSuccess',
                'avgSpeed',
                'collisionCount'
            ]
        };
        
        this.isRunning = false;
        this.currentRun = null;
    }

    /**
     * Lance une série d'expériences avec différents nombres de drones
     * @param {number[]} droneCounts - Ex: [1, 2, 4, 8]
     * @returns {Promise<BenchmarkResults>}
     */
    async runScalingExperiment(droneCounts = [1, 2, 4, 8]) {
        console.log('🧪 DIAMANTS Benchmark — Démarrage expérience scaling');
        console.log(`   Configurations: ${droneCounts.join(', ')} drones`);
        console.log(`   Durée par run: ${this.config.runDuration}s`);
        console.log(`   Répétitions: ${this.config.repetitions}`);

        const experimentId = `exp_${Date.now()}`;
        const allResults = {
            experimentId,
            timestamp: new Date().toISOString(),
            config: { ...this.config, droneCounts },
            runs: []
        };

        for (const droneCount of droneCounts) {
            console.log(`\n📊 Testing with ${droneCount} drone(s)...`);
            
            const runResults = [];
            for (let rep = 0; rep < this.config.repetitions; rep++) {
                console.log(`   Répétition ${rep + 1}/${this.config.repetitions}`);
                
                try {
                    const result = await this.runSingleExperiment(droneCount, rep);
                    runResults.push(result);
                } catch (e) {
                    console.error(`   ❌ Erreur run ${rep}:`, e.message);
                }
            }

            // Agréger les résultats pour ce nombre de drones
            const aggregated = this.aggregateResults(runResults);
            allResults.runs.push({
                droneCount,
                repetitions: runResults,
                aggregated
            });
        }

        this.results.set(experimentId, allResults);
        console.log('\n✅ Expérience terminée!');
        
        return allResults;
    }

    /**
     * Un seul run de benchmark
     */
    async runSingleExperiment(droneCount, repetition) {
        this.isRunning = true;
        const samples = [];
        const startTime = Date.now();
        
        // Reconfigurer le nombre de drones
        await this.reconfigureSwarm(droneCount);
        
        // Reset mission
        if (this.system?.integratedController) {
            await this.system.integratedController.resetMission?.();
        }
        
        // Collecte périodique des métriques
        return new Promise((resolve) => {
            const collectMetrics = () => {
                const elapsed = (Date.now() - startTime) / 1000;
                const sample = {
                    timestamp: elapsed,
                    metrics: this.collectCurrentMetrics()
                };
                samples.push(sample);
                
                // Mise à jour UI
                this.updateBenchmarkUI(droneCount, repetition, elapsed);
            };

            // Démarrer la collecte
            const intervalId = setInterval(collectMetrics, this.config.sampleInterval);
            collectMetrics(); // Premier sample immédiat

            // Arrêter après la durée configurée
            setTimeout(() => {
                clearInterval(intervalId);
                this.isRunning = false;
                
                // Calcul des métriques finales
                const finalMetrics = this.calculateFinalMetrics(samples);
                
                resolve({
                    droneCount,
                    repetition,
                    duration: this.config.runDuration,
                    samples,
                    finalMetrics
                });
            }, this.config.runDuration * 1000);
        });
    }

    /**
     * Collecte les métriques actuelles du système
     */
    collectCurrentMetrics() {
        const metrics = {};
        const controller = this.system?.integratedController;
        
        if (controller?.metrics) {
            metrics.coveragePercentage = controller.metrics.coveragePercentage || 0;
            metrics.emergenceLevel = controller.metrics.emergenceLevel || 0;
            metrics.collaborationEfficiency = controller.metrics.collaborationEfficiency || 0;
            metrics.missionSuccess = controller.metrics.missionSuccess || 0;
        }

        // Métriques additionnelles
        const drones = this.system?.drones || [];
        metrics.activeDrones = drones.filter(d => d.state !== 'IDLE').length;
        
        // Vitesse moyenne
        let totalSpeed = 0;
        drones.forEach(d => {
            if (d.velocity) {
                const speed = Math.sqrt(d.velocity.x**2 + d.velocity.y**2 + d.velocity.z**2);
                totalSpeed += speed;
            }
        });
        metrics.avgSpeed = drones.length > 0 ? totalSpeed / drones.length : 0;

        // Coordination index (distance moyenne entre voisins proches)
        metrics.coordinationIndex = this.calculateCoordinationIndex(drones);

        // Compteur collisions
        metrics.collisionCount = controller?.collisionCount || 0;

        return metrics;
    }

    /**
     * Calcul de l'indice de coordination
     * Mesure la distribution spatiale de l'essaim
     */
    calculateCoordinationIndex(drones) {
        if (drones.length < 2) return 0;

        let totalNearestDist = 0;
        let count = 0;

        drones.forEach((d1, i) => {
            let nearestDist = Infinity;
            drones.forEach((d2, j) => {
                if (i !== j && d1.position && d2.position) {
                    const dx = d1.position.x - d2.position.x;
                    const dy = d1.position.y - d2.position.y;
                    const dz = d1.position.z - d2.position.z;
                    const dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
                    nearestDist = Math.min(nearestDist, dist);
                }
            });
            if (nearestDist !== Infinity) {
                totalNearestDist += nearestDist;
                count++;
            }
        });

        // Normaliser: distance idéale ≈ 3m, on normalise à 0-1
        const avgDist = count > 0 ? totalNearestDist / count : 0;
        const idealDist = 3.0;
        return Math.max(0, 1 - Math.abs(avgDist - idealDist) / idealDist);
    }

    /**
     * Calcul des métriques finales d'un run
     */
    calculateFinalMetrics(samples) {
        if (samples.length === 0) return {};

        const metrics = {};
        const keys = Object.keys(samples[0].metrics);

        keys.forEach(key => {
            const values = samples.map(s => s.metrics[key]).filter(v => v !== undefined);
            if (values.length > 0) {
                metrics[key] = {
                    final: values[values.length - 1],
                    max: Math.max(...values),
                    mean: values.reduce((a, b) => a + b, 0) / values.length,
                    min: Math.min(...values)
                };
            }
        });

        return metrics;
    }

    /**
     * Agrège les résultats de plusieurs répétitions
     */
    aggregateResults(runs) {
        if (runs.length === 0) return {};

        const aggregated = {};
        const metricKeys = Object.keys(runs[0].finalMetrics || {});

        metricKeys.forEach(key => {
            const values = runs.map(r => r.finalMetrics?.[key]?.final).filter(v => v !== undefined);
            if (values.length > 0) {
                aggregated[key] = {
                    mean: values.reduce((a, b) => a + b, 0) / values.length,
                    std: this.standardDeviation(values),
                    min: Math.min(...values),
                    max: Math.max(...values)
                };
            }
        });

        return aggregated;
    }

    standardDeviation(values) {
        const n = values.length;
        if (n === 0) return 0;
        const mean = values.reduce((a, b) => a + b, 0) / n;
        const variance = values.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / n;
        return Math.sqrt(variance);
    }

    /**
     * Reconfigure l'essaim avec un nouveau nombre de drones
     */
    async reconfigureSwarm(droneCount) {
        // Ce sera connecté au contrôleur pour spawner/despawner des drones
        // Pour l'instant, on ajuste côté frontend
        if (this.system?.integratedController) {
            console.log(`   🔄 Reconfiguration essaim: ${droneCount} drones`);
            // TODO: Implémenter la reconfiguration dynamique
            // Pour les tests, on peut modifier this.system.config.droneCount
        }
    }

    /**
     * Export des données pour graphiques
     * Format optimisé pour Chart.js / D3.js / Python matplotlib
     */
    exportChartData(experimentId) {
        const results = experimentId ? this.results.get(experimentId) : Array.from(this.results.values()).pop();
        
        if (!results) {
            console.warn('Aucun résultat à exporter');
            return null;
        }

        // Format pour courbes (x = nb drones, y = métrique)
        const chartData = {
            labels: results.runs.map(r => r.droneCount),
            datasets: {}
        };

        // Extraire chaque métrique
        const metricNames = ['coveragePercentage', 'emergenceLevel', 'collaborationEfficiency', 'coordinationIndex', 'avgSpeed'];
        
        metricNames.forEach(metric => {
            chartData.datasets[metric] = {
                values: results.runs.map(r => r.aggregated[metric]?.mean || 0),
                errors: results.runs.map(r => r.aggregated[metric]?.std || 0)
            };
        });

        return chartData;
    }

    /**
     * Export complet en JSON pour analyse externe
     */
    exportJSON() {
        return JSON.stringify(Array.from(this.results.entries()), null, 2);
    }

    /**
     * Export CSV pour Excel/Python
     */
    exportCSV() {
        let csv = 'experiment_id,drone_count,repetition,coverage,emergence,collaboration,coordination,speed\n';
        
        this.results.forEach((exp, expId) => {
            exp.runs.forEach(run => {
                run.repetitions.forEach(rep => {
                    const fm = rep.finalMetrics;
                    csv += `${expId},${run.droneCount},${rep.repetition},`;
                    csv += `${fm.coveragePercentage?.final || 0},`;
                    csv += `${fm.emergenceLevel?.final || 0},`;
                    csv += `${fm.collaborationEfficiency?.final || 0},`;
                    csv += `${fm.coordinationIndex?.final || 0},`;
                    csv += `${fm.avgSpeed?.final || 0}\n`;
                });
            });
        });

        return csv;
    }

    /**
     * Mise à jour UI pendant le benchmark
     */
    updateBenchmarkUI(droneCount, repetition, elapsed) {
        const progress = (elapsed / this.config.runDuration) * 100;
        
        // Afficher le statut dans le panel
        const statusEl = document.getElementById('benchmark_status');
        if (statusEl) {
            statusEl.innerHTML = `
                <div style="color: #4ade80;">🧪 Benchmark en cours</div>
                <div>${droneCount} drones — Rep ${repetition + 1}</div>
                <div>${elapsed.toFixed(0)}s / ${this.config.runDuration}s</div>
                <div class="benchmark_progress" style="width: ${progress}%"></div>
            `;
        }
    }
}

/**
 * Fonction utilitaire pour lancer un benchmark rapide
 */
export async function runQuickBenchmark(diamantsSystem, droneCounts = [1, 2, 4, 8], duration = 30) {
    const benchmark = new BenchmarkRunner(diamantsSystem);
    benchmark.config.runDuration = duration;
    benchmark.config.repetitions = 1; // Rapide

    const results = await benchmark.runScalingExperiment(droneCounts);
    
    console.log('\n📈 Résultats benchmark:');
    console.table(benchmark.exportChartData(results.experimentId));
    
    return benchmark;
}

// Exposer globalement pour debug
if (typeof window !== 'undefined') {
    window.BenchmarkRunner = BenchmarkRunner;
    window.runQuickBenchmark = runQuickBenchmark;
}
