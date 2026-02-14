/**
 * DIAMANTS — Benchmark Chart Generator
 * ======================================
 * Génère des graphiques SVG pour visualiser les résultats de benchmark.
 * 
 * Courbes principales:
 *   - Coverage vs Nombre de drones
 *   - Emergence vs Nombre de drones
 *   - Coordination vs Nombre de drones
 *   - Efficiency vs Nombre de drones
 */

export class BenchmarkChartGenerator {
    constructor(containerId = 'benchmark_charts') {
        this.containerId = containerId;
        this.colors = {
            coverage: '#4ade80',      // green
            emergence: '#f472b6',     // pink
            coordination: '#60a5fa',  // blue
            efficiency: '#fbbf24',    // yellow
            avgSpeed: '#a78bfa'       // purple
        };
        
        this.chartConfig = {
            width: 400,
            height: 250,
            padding: { top: 30, right: 20, bottom: 40, left: 50 }
        };
    }

    /**
     * Génère tous les graphiques à partir des données de benchmark
     */
    generateAllCharts(chartData) {
        const container = document.getElementById(this.containerId);
        if (!container) {
            console.error('Container not found:', this.containerId);
            return;
        }

        container.innerHTML = '';
        
        // Chart header
        const header = document.createElement('h3');
        header.textContent = '📈 Résultats Benchmark — Scaling Multi-Drone';
        header.style.cssText = 'color: #fff; margin: 10px 0; font-size: 14px;';
        container.appendChild(header);

        // Générer chaque graphique
        const metricsToPlot = [
            { key: 'coveragePercentage', label: 'Coverage (%)', color: this.colors.coverage },
            { key: 'emergenceLevel', label: 'Emergence', color: this.colors.emergence },
            { key: 'collaborationEfficiency', label: 'Coordination', color: this.colors.coordination },
            { key: 'avgSpeed', label: 'Vitesse (m/s)', color: this.colors.avgSpeed }
        ];

        metricsToPlot.forEach(metric => {
            const chartDiv = document.createElement('div');
            chartDiv.style.cssText = 'margin: 15px 0; background: #1a1a2e; border-radius: 8px; padding: 10px;';
            
            const svg = this.createLineChart(
                chartData.labels,
                chartData.datasets[metric.key]?.values || [],
                chartData.datasets[metric.key]?.errors || [],
                metric.label,
                metric.color
            );
            
            chartDiv.appendChild(svg);
            container.appendChild(chartDiv);
        });

        // Export buttons
        const exportDiv = document.createElement('div');
        exportDiv.style.cssText = 'margin-top: 15px; display: flex; gap: 10px;';
        exportDiv.innerHTML = `
            <button onclick="window.benchmarkExportCSV()" class="btn_ros" style="font-size: 11px;">📊 Export CSV</button>
            <button onclick="window.benchmarkExportJSON()" class="btn_ros" style="font-size: 11px;">📋 Export JSON</button>
        `;
        container.appendChild(exportDiv);
    }

    /**
     * Crée un graphique linéaire SVG
     */
    createLineChart(labels, values, errors, title, color) {
        const { width, height, padding } = this.chartConfig;
        const plotWidth = width - padding.left - padding.right;
        const plotHeight = height - padding.top - padding.bottom;

        // Créer SVG
        const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
        svg.setAttribute('width', width);
        svg.setAttribute('height', height);
        svg.style.cssText = 'display: block; margin: 0 auto;';

        // Background
        const bg = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        bg.setAttribute('width', width);
        bg.setAttribute('height', height);
        bg.setAttribute('fill', '#0d0d1a');
        bg.setAttribute('rx', '4');
        svg.appendChild(bg);

        // Title
        const titleEl = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        titleEl.setAttribute('x', width / 2);
        titleEl.setAttribute('y', 20);
        titleEl.setAttribute('text-anchor', 'middle');
        titleEl.setAttribute('fill', '#fff');
        titleEl.setAttribute('font-size', '12');
        titleEl.setAttribute('font-weight', 'bold');
        titleEl.textContent = title;
        svg.appendChild(titleEl);

        if (values.length === 0) {
            const noData = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            noData.setAttribute('x', width / 2);
            noData.setAttribute('y', height / 2);
            noData.setAttribute('text-anchor', 'middle');
            noData.setAttribute('fill', '#666');
            noData.setAttribute('font-size', '11');
            noData.textContent = 'No data';
            svg.appendChild(noData);
            return svg;
        }

        // Calcul des échelles
        const maxVal = Math.max(...values) * 1.2 || 100;
        const minVal = 0;
        
        const xScale = (i) => padding.left + (i / (labels.length - 1 || 1)) * plotWidth;
        const yScale = (v) => padding.top + plotHeight - ((v - minVal) / (maxVal - minVal)) * plotHeight;

        // Grille
        for (let i = 0; i <= 4; i++) {
            const y = padding.top + (i / 4) * plotHeight;
            const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
            line.setAttribute('x1', padding.left);
            line.setAttribute('y1', y);
            line.setAttribute('x2', width - padding.right);
            line.setAttribute('y2', y);
            line.setAttribute('stroke', '#333');
            line.setAttribute('stroke-dasharray', '2,2');
            svg.appendChild(line);

            // Y labels
            const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            label.setAttribute('x', padding.left - 5);
            label.setAttribute('y', y + 4);
            label.setAttribute('text-anchor', 'end');
            label.setAttribute('fill', '#888');
            label.setAttribute('font-size', '9');
            label.textContent = ((1 - i/4) * maxVal).toFixed(0);
            svg.appendChild(label);
        }

        // Axes
        const axisColor = '#555';
        
        // X axis
        const xAxis = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        xAxis.setAttribute('x1', padding.left);
        xAxis.setAttribute('y1', height - padding.bottom);
        xAxis.setAttribute('x2', width - padding.right);
        xAxis.setAttribute('y2', height - padding.bottom);
        xAxis.setAttribute('stroke', axisColor);
        svg.appendChild(xAxis);

        // Y axis
        const yAxis = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        yAxis.setAttribute('x1', padding.left);
        yAxis.setAttribute('y1', padding.top);
        yAxis.setAttribute('x2', padding.left);
        yAxis.setAttribute('y2', height - padding.bottom);
        yAxis.setAttribute('stroke', axisColor);
        svg.appendChild(yAxis);

        // X axis label
        const xAxisLabel = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        xAxisLabel.setAttribute('x', width / 2);
        xAxisLabel.setAttribute('y', height - 5);
        xAxisLabel.setAttribute('text-anchor', 'middle');
        xAxisLabel.setAttribute('fill', '#888');
        xAxisLabel.setAttribute('font-size', '10');
        xAxisLabel.textContent = 'Nombre de drones';
        svg.appendChild(xAxisLabel);

        // X labels (drone counts)
        labels.forEach((label, i) => {
            const x = xScale(i);
            const text = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            text.setAttribute('x', x);
            text.setAttribute('y', height - padding.bottom + 15);
            text.setAttribute('text-anchor', 'middle');
            text.setAttribute('fill', '#888');
            text.setAttribute('font-size', '10');
            text.textContent = label;
            svg.appendChild(text);
        });

        // Error bars (si disponibles)
        if (errors && errors.length === values.length) {
            values.forEach((val, i) => {
                const x = xScale(i);
                const yTop = yScale(val + (errors[i] || 0));
                const yBottom = yScale(val - (errors[i] || 0));
                
                const errorLine = document.createElementNS('http://www.w3.org/2000/svg', 'line');
                errorLine.setAttribute('x1', x);
                errorLine.setAttribute('y1', yTop);
                errorLine.setAttribute('x2', x);
                errorLine.setAttribute('y2', yBottom);
                errorLine.setAttribute('stroke', color);
                errorLine.setAttribute('stroke-opacity', '0.4');
                errorLine.setAttribute('stroke-width', '2');
                svg.appendChild(errorLine);
            });
        }

        // Line path
        const pathData = values.map((val, i) => {
            const x = xScale(i);
            const y = yScale(val);
            return `${i === 0 ? 'M' : 'L'} ${x} ${y}`;
        }).join(' ');

        const path = document.createElementNS('http://www.w3.org/2000/svg', 'path');
        path.setAttribute('d', pathData);
        path.setAttribute('fill', 'none');
        path.setAttribute('stroke', color);
        path.setAttribute('stroke-width', '2');
        svg.appendChild(path);

        // Data points
        values.forEach((val, i) => {
            const x = xScale(i);
            const y = yScale(val);
            
            const circle = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
            circle.setAttribute('cx', x);
            circle.setAttribute('cy', y);
            circle.setAttribute('r', '5');
            circle.setAttribute('fill', color);
            circle.setAttribute('stroke', '#fff');
            circle.setAttribute('stroke-width', '1');
            svg.appendChild(circle);

            // Value label
            const valLabel = document.createElementNS('http://www.w3.org/2000/svg', 'text');
            valLabel.setAttribute('x', x);
            valLabel.setAttribute('y', y - 10);
            valLabel.setAttribute('text-anchor', 'middle');
            valLabel.setAttribute('fill', '#fff');
            valLabel.setAttribute('font-size', '9');
            valLabel.textContent = val.toFixed(1);
            svg.appendChild(valLabel);
        });

        return svg;
    }

    /**
     * Génère un rapport HTML complet
     */
    generateReport(benchmarkResults) {
        const chartData = benchmarkResults.exportChartData?.() || benchmarkResults;
        
        let html = `
        <!DOCTYPE html>
        <html>
        <head>
            <title>DIAMANTS Benchmark Report</title>
            <style>
                body { 
                    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
                    background: #0d0d1a; color: #fff; padding: 20px;
                }
                .chart-container { display: flex; flex-wrap: wrap; gap: 20px; justify-content: center; }
                .chart { background: #1a1a2e; border-radius: 8px; padding: 15px; }
                h1 { text-align: center; color: #4ade80; }
                .summary { background: #1a1a2e; padding: 15px; border-radius: 8px; margin: 20px 0; }
                table { width: 100%; border-collapse: collapse; }
                th, td { padding: 8px; text-align: left; border-bottom: 1px solid #333; }
                th { color: #4ade80; }
            </style>
        </head>
        <body>
            <h1>🚁 DIAMANTS — Benchmark Report</h1>
            <p style="text-align: center;">Impact de la coopération multi-drone sur les métriques de performance</p>
            
            <div class="summary">
                <h3>📊 Résumé</h3>
                <p>Configurations testées: ${chartData.labels?.join(', ')} drones</p>
                <p>Date: ${new Date().toISOString()}</p>
            </div>
            
            <div class="chart-container" id="charts">
                <!-- Charts will be inserted here -->
            </div>
            
            <div class="summary">
                <h3>📋 Données brutes</h3>
                <table>
                    <tr>
                        <th>Drones</th>
                        <th>Coverage (%)</th>
                        <th>Emergence</th>
                        <th>Coordination</th>
                        <th>Speed</th>
                    </tr>
        `;

        chartData.labels?.forEach((label, i) => {
            html += `
                    <tr>
                        <td>${label}</td>
                        <td>${(chartData.datasets.coveragePercentage?.values[i] || 0).toFixed(1)}</td>
                        <td>${(chartData.datasets.emergenceLevel?.values[i] || 0).toFixed(2)}</td>
                        <td>${(chartData.datasets.collaborationEfficiency?.values[i] || 0).toFixed(2)}</td>
                        <td>${(chartData.datasets.avgSpeed?.values[i] || 0).toFixed(2)}</td>
                    </tr>
            `;
        });

        html += `
                </table>
            </div>
        </body>
        </html>
        `;

        return html;
    }
}

// Export global
if (typeof window !== 'undefined') {
    window.BenchmarkChartGenerator = BenchmarkChartGenerator;
}
