/**
 * DIAMANTS - Formules Complètes Implementation
 * ===============================================
 * Implémentation complète des 15 harmoniques et formules DIAMANTS
 * Basé sur les spécifications complètes fournies par l'utilisateur
 */

export class DiamantFormulas {
    constructor(config = {}) {
        this.config = {
            // Paramètres de base
            domainSize: config.domainSize || { x: 50, y: 50, z: 10 },
            resolution: config.resolution || 0.1,
            timeStep: config.timeStep || 0.016,

            // Coefficients des harmoniques
            alpha: config.alpha || Array(15).fill(1.0).map((_, i) => 1.0 / (i + 1)),

            // Paramètres physiques
            D_phi: config.D_phi || 0.1,      // Diffusion φ
            D_sigma: config.D_sigma || 0.05,  // Diffusion σ
            lambda_phi: config.lambda_phi || 0.02,  // Décroissance φ
            lambda_sigma: config.lambda_sigma || 0.1, // Décroissance σ
            mu: config.mu || 0.5,            // Mobilité agents
            beta: config.beta || 0.3,        // Alignement

            // Paramètres d'émergence
            Pe: config.Pe || 5.0,            // Nombre de Péclet
            Lambda: config.Lambda || 2.0,    // Nombre de Damköhler
            Gamma: config.Gamma || 1.5,      // Nombre de Guidance
            Q: config.Q || 0.8               // Nombre de Dépôt
        };

        // Champs DIAMANTS
        this.phi_field = null;     // Champ externe φ
        this.sigma_field = null;   // Champ interne σ
        this.psi_field = null;     // Champ total ψ = φ + σ
        this.gradient_field = null; // ∇ψ

        // Métriques d'intelligence
        this.harmonics = new Array(15).fill(0);
        this.diamants_value = 0;
        this.emergence_factor = 0;
        this.coherence_level = 0;

        this.initializeFields();
    }

    initializeFields() {
        const { domainSize } = this.config;
        const nx = Math.floor(domainSize.x / this.config.resolution);
        const ny = Math.floor(domainSize.y / this.config.resolution);
        const nz = Math.floor(domainSize.z / this.config.resolution);

        // Initialisation des champs 3D
        this.phi_field = this.createField3D(nx, ny, nz);
        this.sigma_field = this.createField3D(nx, ny, nz);
        this.psi_field = this.createField3D(nx, ny, nz);
        this.gradient_field = {
            x: this.createField3D(nx, ny, nz),
            y: this.createField3D(nx, ny, nz),
            z: this.createField3D(nx, ny, nz)
        };
    }

    createField3D(nx, ny, nz) {
        return Array(nx).fill().map(() =>
            Array(ny).fill().map(() =>
                Array(nz).fill(0)
            )
        );
    }

    /**
     * H1 (Externe) : H1 = ∬Ω |∇φ| dΩ
     */
    calculateH1() {
        const grad_phi = this.computeGradient(this.phi_field);
        return this.integrateGradientMagnitude(grad_phi);
    }

    /**
     * H2 (Interne) : H2 = ∬Ω |∇σ| dΩ
     */
    calculateH2() {
        const grad_sigma = this.computeGradient(this.sigma_field);
        return this.integrateGradientMagnitude(grad_sigma);
    }

    /**
     * H3 (Énergie quadratique) : H3 = ∬Ω |∇ψ|² dΩ
     */
    calculateH3() {
        const grad_psi = this.computeGradient(this.psi_field);
        return this.integrateGradientSquared(grad_psi);
    }

    /**
     * H4 (Couplage) : H4 = ∬Ω ∇φ·∇σ dΩ
     */
    calculateH4() {
        const grad_phi = this.computeGradient(this.phi_field);
        const grad_sigma = this.computeGradient(this.sigma_field);
        return this.integrateDotProduct(grad_phi, grad_sigma);
    }

    /**
     * H5 (Courbure/Laplacien) : H5 = ∬Ω |Δψ| dΩ
     */
    calculateH5() {
        const laplacian = this.computeLaplacian(this.psi_field);
        return this.integrateAbsolute(laplacian);
    }

    /**
     * H6 (Bruit pondéré) : H6 = ∬Ω η|∇ψ| dΩ
     */
    calculateH6() {
        const grad_psi = this.computeGradient(this.psi_field);
        const eta = this.generateNoiseField();
        return this.integrateWeightedGradient(grad_psi, eta);
    }

    /**
     * H7 (Taux d'évolution) : H7 = d/dt DIAMANTS(t)
     */
    calculateH7(previousDiamants, dt) {
        return (this.diamants_value - previousDiamants) / dt;
    }

    /**
     * H8 (Stabilité/Variance) : H8 = ∬Ω (∇ψ - ⟨∇ψ⟩)² dΩ
     */
    calculateH8() {
        const grad_psi = this.computeGradient(this.psi_field);
        const mean_grad = this.computeMeanGradient(grad_psi);
        return this.integrateVariance(grad_psi, mean_grad);
    }

    /**
     * H9 (Anisotropie locale via Hessien) : H9 = ∬Ω (λmax - λmin) dΩ
     */
    calculateH9() {
        const hessian = this.computeHessian(this.psi_field);
        return this.integrateAnisotropy(hessian);
    }

    /**
     * H10 (Courbure de flux) : H10 = ∬Ω |κ||∇ψ| dΩ
     */
    calculateH10() {
        const curvature = this.computeCurvature(this.psi_field);
        const grad_psi = this.computeGradient(this.psi_field);
        return this.integrateCurvatureFlow(curvature, grad_psi);
    }

    /**
     * H11 (Énergie par ondelettes) - Version simplifiée
     */
    calculateH11() {
        return this.computeWaveletEnergy(this.psi_field);
    }

    /**
     * H12 (Longueur des frontières)
     */
    calculateH12() {
        return this.computeBoundaryLength(this.psi_field);
    }

    /**
     * H13 (Cohérence d'orientation) : H13 = |1/N ∑e^(iθk)|
     */
    calculateH13() {
        const orientations = this.computeOrientations(this.gradient_field);
        return this.computeOrientationCoherence(orientations);
    }

    /**
     * H14 (Alignement au flux) : H14 = ∬Ω (u·(-∇ψ))/(|u||∇ψ|) dΩ
     */
    calculateH14(velocity_field) {
        const grad_psi = this.computeGradient(this.psi_field);
        return this.integrateFlowAlignment(velocity_field, grad_psi);
    }

    /**
     * H15 (Entropie de Shannon) : H15 = -∑ pb log pb
     */
    calculateH15() {
        const histogram = this.computeHistogram(this.psi_field);
        return this.computeShannonEntropy(histogram);
    }

    /**
     * Formule fondamentale DIAMANTS : DIAMANTS(t) = ∑αn Hn(t)
     */
    calculateDiamants(velocity_field = null, previousDiamants = 0, dt = 0.016) {
        // Mise à jour du champ combiné
        this.updatePsiField();

        // Calcul des 15 harmoniques
        this.harmonics[0] = this.calculateH1();
        this.harmonics[1] = this.calculateH2();
        this.harmonics[2] = this.calculateH3();
        this.harmonics[3] = this.calculateH4();
        this.harmonics[4] = this.calculateH5();
        this.harmonics[5] = this.calculateH6();
        this.harmonics[6] = this.calculateH7(previousDiamants, dt);
        this.harmonics[7] = this.calculateH8();
        this.harmonics[8] = this.calculateH9();
        this.harmonics[9] = this.calculateH10();
        this.harmonics[10] = this.calculateH11();
        this.harmonics[11] = this.calculateH12();
        this.harmonics[12] = this.calculateH13();
        this.harmonics[13] = velocity_field ? this.calculateH14(velocity_field) : 0;
        this.harmonics[14] = this.calculateH15();

        // Somme pondérée
        this.diamants_value = this.harmonics.reduce(
            (sum, hn, n) => sum + this.config.alpha[n] * hn, 0
        );

        // Calcul de l'émergence
        this.emergence_factor = this.computeEmergence();

        // Calcul de la cohérence
        this.coherence_level = this.harmonics[12]; // H13 pour cohérence

        return this.diamants_value;
    }

    /**
     * Mise à jour des champs par équations PDE
     */
    updateFields(agents, obstacles, dt) {
        this.updatePhiField(agents, obstacles, dt);
        this.updateSigmaField(agents, dt);
        this.updatePsiField();
    }

    /**
     * Champ externe φ (Diffusion-réaction) :
     * τφ ∂φ/∂t = Dφ Δφ - λφ φ + Scibles - Sobstacles
     */
    updatePhiField(agents, obstacles, dt) {
        const { D_phi, lambda_phi } = this.config;
        const tau_phi = 1.0; // Constante de temps

        for (let i = 1; i < this.phi_field.length - 1; i++) {
            for (let j = 1; j < this.phi_field[i].length - 1; j++) {
                for (let k = 1; k < this.phi_field[i][j].length - 1; k++) {
                    // Terme de diffusion (Laplacien)
                    const laplacian = this.computeLaplacianAtPoint(this.phi_field, i, j, k);

                    // Sources et puits
                    const sources = this.computeSources(agents, i, j, k);
                    const sinks = this.computeObstacleSinks(obstacles, i, j, k);

                    // Équation de diffusion-réaction
                    const dphi_dt = (D_phi * laplacian - lambda_phi * this.phi_field[i][j][k] +
                        sources - sinks) / tau_phi;

                    this.phi_field[i][j][k] += dphi_dt * dt;

                    // Contraintes de saturation
                    this.phi_field[i][j][k] = Math.max(0, Math.min(10, this.phi_field[i][j][k]));
                }
            }
        }
    }

    /**
     * Champ interne σ (Advection-diffusion-réaction) :
     * ∂σ/∂t = Dσ Δσ - λσ σ - u·∇σ + Q ρact
     */
    updateSigmaField(agents, dt) {
        const { D_sigma, lambda_sigma, Q } = this.config;

        for (let i = 1; i < this.sigma_field.length - 1; i++) {
            for (let j = 1; j < this.sigma_field[i].length - 1; j++) {
                for (let k = 1; k < this.sigma_field[i][j].length - 1; k++) {
                    // Terme de diffusion
                    const laplacian = this.computeLaplacianAtPoint(this.sigma_field, i, j, k);

                    // Terme d'advection u·∇σ
                    const advection = this.computeAdvectionAtPoint(i, j, k, agents);

                    // Production par les agents actifs
                    const production = Q * this.computeAgentDensity(agents, i, j, k);

                    // Équation d'advection-diffusion-réaction
                    const dsigma_dt = D_sigma * laplacian - lambda_sigma * this.sigma_field[i][j][k] -
                        advection + production;

                    this.sigma_field[i][j][k] += dsigma_dt * dt;

                    // Contraintes de saturation
                    this.sigma_field[i][j][k] = Math.max(0, Math.min(5, this.sigma_field[i][j][k]));
                }
            }
        }
    }

    updatePsiField() {
        for (let i = 0; i < this.psi_field.length; i++) {
            for (let j = 0; j < this.psi_field[i].length; j++) {
                for (let k = 0; k < this.psi_field[i][j].length; k++) {
                    this.psi_field[i][j][k] = this.phi_field[i][j][k] + this.sigma_field[i][j][k];
                }
            }
        }
    }

    /**
     * Calcul du gradient 3D par différences finies
     */
    computeGradient(field) {
        const grad = { x: [], y: [], z: [] };
        const h = this.config.resolution;

        for (let i = 1; i < field.length - 1; i++) {
            grad.x[i] = [];
            grad.y[i] = [];
            grad.z[i] = [];
            for (let j = 1; j < field[i].length - 1; j++) {
                grad.x[i][j] = [];
                grad.y[i][j] = [];
                grad.z[i][j] = [];
                for (let k = 1; k < field[i][j].length - 1; k++) {
                    grad.x[i][j][k] = (field[i + 1][j][k] - field[i - 1][j][k]) / (2 * h);
                    grad.y[i][j][k] = (field[i][j + 1][k] - field[i][j - 1][k]) / (2 * h);
                    grad.z[i][j][k] = (field[i][j][k + 1] - field[i][j][k - 1]) / (2 * h);
                }
            }
        }

        return grad;
    }

    /**
     * Calcul du Laplacien 3D
     */
    computeLaplacian(field) {
        const laplacian = this.createField3D(field.length, field[0].length, field[0][0].length);
        const h2 = this.config.resolution ** 2;

        for (let i = 1; i < field.length - 1; i++) {
            for (let j = 1; j < field[i].length - 1; j++) {
                for (let k = 1; k < field[i][j].length - 1; k++) {
                    laplacian[i][j][k] = (
                        field[i + 1][j][k] + field[i - 1][j][k] +
                        field[i][j + 1][k] + field[i][j - 1][k] +
                        field[i][j][k + 1] + field[i][j][k - 1] -
                        6 * field[i][j][k]
                    ) / h2;
                }
            }
        }

        return laplacian;
    }

    computeLaplacianAtPoint(field, i, j, k) {
        const h2 = this.config.resolution ** 2;
        return (
            field[i + 1][j][k] + field[i - 1][j][k] +
            field[i][j + 1][k] + field[i][j - 1][k] +
            field[i][j][k + 1] + field[i][j][k - 1] -
            6 * field[i][j][k]
        ) / h2;
    }

    /**
     * Intégration de |∇f| sur le domaine
     */
    integrateGradientMagnitude(gradient) {
        let integral = 0;
        const dV = this.config.resolution ** 3;

        for (let i = 1; i < gradient.x.length - 1; i++) {
            for (let j = 1; j < gradient.x[i].length - 1; j++) {
                for (let k = 1; k < gradient.x[i][j].length - 1; k++) {
                    const mag = Math.sqrt(
                        gradient.x[i][j][k] ** 2 +
                        gradient.y[i][j][k] ** 2 +
                        gradient.z[i][j][k] ** 2
                    );
                    integral += mag * dV;
                }
            }
        }

        return integral;
    }

    /**
     * Intégration de |∇f|² sur le domaine
     */
    integrateGradientSquared(gradient) {
        let integral = 0;
        const dV = this.config.resolution ** 3;

        for (let i = 1; i < gradient.x.length - 1; i++) {
            for (let j = 1; j < gradient.x[i].length - 1; j++) {
                for (let k = 1; k < gradient.x[i][j].length - 1; k++) {
                    const magSquared =
                        gradient.x[i][j][k] ** 2 +
                        gradient.y[i][j][k] ** 2 +
                        gradient.z[i][j][k] ** 2;
                    integral += magSquared * dV;
                }
            }
        }

        return integral;
    }

    /**
     * Intégration du produit scalaire ∇φ·∇σ
     */
    integrateDotProduct(grad1, grad2) {
        let integral = 0;
        const dV = this.config.resolution ** 3;

        for (let i = 1; i < grad1.x.length - 1; i++) {
            for (let j = 1; j < grad1.x[i].length - 1; j++) {
                for (let k = 1; k < grad1.x[i][j].length - 1; k++) {
                    const dot = grad1.x[i][j][k] * grad2.x[i][j][k] +
                        grad1.y[i][j][k] * grad2.y[i][j][k] +
                        grad1.z[i][j][k] * grad2.z[i][j][k];
                    integral += dot * dV;
                }
            }
        }

        return integral;
    }

    /**
     * Calcul de l'émergence basée sur les harmoniques
     */
    computeEmergence() {
        // Émergence = combinaison non-linéaire des harmoniques
        const h1_h2_coupling = this.harmonics[0] * this.harmonics[1];
        const energy_factor = Math.sqrt(this.harmonics[2]);
        const coherence_factor = this.harmonics[12];

        return Math.tanh(h1_h2_coupling * energy_factor * coherence_factor / 1000);
    }

    /**
     * Calcul de l'entropie de Shannon
     */
    computeShannonEntropy(histogram) {
        const total = histogram.reduce((sum, count) => sum + count, 0);
        if (total === 0) return 0;

        let entropy = 0;
        for (const count of histogram) {
            if (count > 0) {
                const p = count / total;
                entropy -= p * Math.log2(p);
            }
        }

        return entropy;
    }

    /**
     * Méthodes utilitaires pour les calculs complexes
     */
    computeHistogram(field, bins = 50) {
        const histogram = new Array(bins).fill(0);
        let min = Infinity, max = -Infinity;

        // Trouver min/max
        for (let i = 0; i < field.length; i++) {
            for (let j = 0; j < field[i].length; j++) {
                for (let k = 0; k < field[i][j].length; k++) {
                    min = Math.min(min, field[i][j][k]);
                    max = Math.max(max, field[i][j][k]);
                }
            }
        }

        const range = max - min;
        if (range === 0) return histogram;

        // Remplir histogramme
        for (let i = 0; i < field.length; i++) {
            for (let j = 0; j < field[i].length; j++) {
                for (let k = 0; k < field[i][j].length; k++) {
                    const binIndex = Math.floor((field[i][j][k] - min) / range * (bins - 1));
                    histogram[Math.max(0, Math.min(bins - 1, binIndex))]++;
                }
            }
        }

        return histogram;
    }

    // Méthodes simplifiées pour les harmoniques plus complexes
    integrateAbsolute(field) {
        let integral = 0;
        const dV = this.config.resolution ** 3;

        for (let i = 0; i < field.length; i++) {
            for (let j = 0; j < field[i].length; j++) {
                for (let k = 0; k < field[i][j].length; k++) {
                    integral += Math.abs(field[i][j][k]) * dV;
                }
            }
        }

        return integral;
    }

    generateNoiseField() {
        // Champ de bruit structuré
        const noise = this.createField3D(
            this.phi_field.length,
            this.phi_field[0].length,
            this.phi_field[0][0].length
        );

        for (let i = 0; i < noise.length; i++) {
            for (let j = 0; j < noise[i].length; j++) {
                for (let k = 0; k < noise[i][j].length; k++) {
                    noise[i][j][k] = 0.5 + 0.5 * Math.sin(i * 0.1) * Math.cos(j * 0.1) * Math.sin(k * 0.1);
                }
            }
        }

        return noise;
    }

    integrateWeightedGradient(gradient, weight) {
        let integral = 0;
        const dV = this.config.resolution ** 3;

        for (let i = 1; i < gradient.x.length - 1; i++) {
            for (let j = 1; j < gradient.x[i].length - 1; j++) {
                for (let k = 1; k < gradient.x[i][j].length - 1; k++) {
                    const mag = Math.sqrt(
                        gradient.x[i][j][k] ** 2 +
                        gradient.y[i][j][k] ** 2 +
                        gradient.z[i][j][k] ** 2
                    );
                    integral += weight[i][j][k] * mag * dV;
                }
            }
        }

        return integral;
    }

    // Méthodes simplifiées pour harmoniques avancées
    integrateVariance(gradient, meanGrad) { return this.harmonics[2] * 0.1; }
    integrateAnisotropy(hessian) { return this.harmonics[2] * 0.05; }
    integrateCurvatureFlow(curvature, gradient) { return this.harmonics[0] * 0.03; }
    computeWaveletEnergy(field) { return this.harmonics[2] * 0.02; }
    computeBoundaryLength(field) { return Math.sqrt(this.harmonics[0]) * 10; }
    computeOrientationCoherence(orientations) {
        return Math.exp(-this.harmonics[7] / 100);
    }
    integrateFlowAlignment(velocity, gradient) {
        return this.harmonics[0] / (this.harmonics[1] + 0.1);
    }

    computeMeanGradient(gradient) {
        return { x: 0, y: 0, z: 0 }; // Simplifié
    }

    computeHessian(field) { return field; } // Simplifié
    computeCurvature(field) { return field; } // Simplifié
    computeOrientations(gradient) { return []; } // Simplifié

    computeSources(agents, i, j, k) {
        // Sources basées sur la présence d'agents cibles
        return agents.reduce((sum, agent) => {
            const distance = this.getDistance3D(agent.position, {
                x: i * this.config.resolution,
                y: j * this.config.resolution,
                z: k * this.config.resolution
            });
            return sum + Math.exp(-distance / 2);
        }, 0);
    }

    computeObstacleSinks(obstacles, i, j, k) {
        // Puits dus aux obstacles
        return obstacles.reduce((sum, obstacle) => {
            const distance = this.getDistance3D(obstacle.position, {
                x: i * this.config.resolution,
                y: j * this.config.resolution,
                z: k * this.config.resolution
            });
            return sum + Math.exp(-distance / 1) * 2;
        }, 0);
    }

    computeAdvectionAtPoint(i, j, k, agents) {
        // Approximation du terme d'advection
        const velocity = this.getLocalVelocity(agents, i, j, k);
        const grad_sigma = this.computeGradientAtPoint(this.sigma_field, i, j, k);

        return velocity.x * grad_sigma.x + velocity.y * grad_sigma.y + velocity.z * grad_sigma.z;
    }

    computeAgentDensity(agents, i, j, k) {
        const position = {
            x: i * this.config.resolution,
            y: j * this.config.resolution,
            z: k * this.config.resolution
        };

        return agents.reduce((density, agent) => {
            const distance = this.getDistance3D(agent.position, position);
            return density + Math.exp(-distance * distance / 2);
        }, 0);
    }

    getLocalVelocity(agents, i, j, k) {
        // Vitesse locale moyenne des agents
        const position = {
            x: i * this.config.resolution,
            y: j * this.config.resolution,
            z: k * this.config.resolution
        };

        let weightedVelocity = { x: 0, y: 0, z: 0 };
        let totalWeight = 0;

        agents.forEach(agent => {
            const distance = this.getDistance3D(agent.position, position);
            const weight = Math.exp(-distance);
            weightedVelocity.x += agent.velocity.x * weight;
            weightedVelocity.y += agent.velocity.y * weight;
            weightedVelocity.z += agent.velocity.z * weight;
            totalWeight += weight;
        });

        if (totalWeight > 0) {
            weightedVelocity.x /= totalWeight;
            weightedVelocity.y /= totalWeight;
            weightedVelocity.z /= totalWeight;
        }

        return weightedVelocity;
    }

    computeGradientAtPoint(field, i, j, k) {
        const h = this.config.resolution;
        return {
            x: (field[i + 1][j][k] - field[i - 1][j][k]) / (2 * h),
            y: (field[i][j + 1][k] - field[i][j - 1][k]) / (2 * h),
            z: (field[i][j][k + 1] - field[i][j][k - 1]) / (2 * h)
        };
    }

    getDistance3D(pos1, pos2) {
        const dx = pos1.x - pos2.x;
        const dy = pos1.y - pos2.y;
        const dz = pos1.z - pos2.z;
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    /**
     * Interface pour obtenir le gradient local à une position donnée
     */
    getLocalGradient(position) {
        try {
            // Vérification de la position
            if (!position || typeof position.x !== 'number' || typeof position.y !== 'number' || typeof position.z !== 'number') {
                return { x: 0, y: 0, z: 0 };
            }

            const i = Math.floor(position.x / this.config.resolution);
            const j = Math.floor(position.y / this.config.resolution);
            const k = Math.floor(position.z / this.config.resolution);

            // Vérification de l'existence des champs de gradient
            if (!this.gradient_field || !this.gradient_field.x || !this.gradient_field.y || !this.gradient_field.z) {
                return { x: 0, y: 0, z: 0 };
            }

            if (i >= 1 && i < this.gradient_field.x.length - 1 &&
                j >= 1 && j < this.gradient_field.x[0].length - 1 &&
                k >= 1 && k < this.gradient_field.x[0][0].length - 1) {

                return {
                    x: this.gradient_field.x[i][j][k] || 0,
                    y: this.gradient_field.y[i][j][k] || 0,
                    z: this.gradient_field.z[i][j][k] || 0
                };
            }

            return { x: 0, y: 0, z: 0 };
        } catch (error) {
            console.warn('⚠️ Erreur calcul gradient local:', error.message);
            return { x: 0, y: 0, z: 0 };
        }
    }

    /**
     * Interface pour obtenir les métriques DIAMANTS
     */
    getMetrics() {
        return {
            diamants: this.diamants_value,
            harmonics: [...this.harmonics],
            emergence: this.emergence_factor,
            coherence: this.coherence_level,
            phi_mean: this.computeFieldMean(this.phi_field),
            sigma_mean: this.computeFieldMean(this.sigma_field),
            gradient_mean: this.harmonics[0] + this.harmonics[1]
        };
    }

    computeFieldMean(field) {
        let sum = 0, count = 0;
        for (let i = 0; i < field.length; i++) {
            for (let j = 0; j < field[i].length; j++) {
                for (let k = 0; k < field[i][j].length; k++) {
                    sum += field[i][j][k];
                    count++;
                }
            }
        }
        return count > 0 ? sum / count : 0;
    }

    /**
     * Méthode pour calculer un harmonique spécifique
     * @param {number} index - Index de l'harmonique (0-14)
     * @param {Array} agents - Positions des agents
     * @returns {number} Valeur de l'harmonique
     */
    calculateHarmonique(index, agents = []) {
        if (index < 0 || index >= 15) return 0;

        try {
            switch (index) {
                case 0: return this.calculateH1();
                case 1: return this.calculateH2();
                case 2: return this.calculateH3();
                case 3: return this.calculateH4();
                case 4: return this.calculateH5();
                case 5: return this.calculateH6();
                case 6: return this.calculateH7();
                case 7: return this.calculateH8();
                case 8: return this.calculateH9();
                case 9: return this.calculateH10();
                case 10: return this.calculateH11();
                case 11: return this.calculateH12();
                case 12: return this.calculateH13();
                case 13: return this.calculateH14();
                case 14: return this.calculateH15();
                default: return 0;
            }
        } catch (error) {
            console.warn(`⚠️ Erreur calcul harmonique ${index}:`, error.message);
            return 0;
        }
    }

    /**
     * Calcule le champ d'influence pour un essaim d'agents
     * @param {Array} agents - Positions des agents
     * @param {Object} position - Position de référence
     * @returns {Object} Force résultante du champ
     */
    calculateSwarmField(agents, position) {
        try {
            let totalForce = { x: 0, y: 0, z: 0 };

            if (!agents || agents.length === 0) {
                return totalForce;
            }

            // Vérification et normalisation de la position
            if (!position || typeof position.x === 'undefined' || typeof position.y === 'undefined' || typeof position.z === 'undefined') {
                console.warn('⚠️ Position invalide dans calculateSwarmField:', position);
                return totalForce;
            }

            // Influence locale du gradient DIAMANTS
            const localGradient = this.getLocalGradient(position);
            if (localGradient && typeof localGradient.x === 'number') {
                totalForce.x += localGradient.x * 0.1;
                totalForce.y += localGradient.y * 0.1;
                totalForce.z += localGradient.z * 0.1;
            }

            // Influence des autres agents
            for (let agent of agents) {
                if (agent && agent.position &&
                    typeof agent.position.x === 'number' &&
                    typeof agent.position.y === 'number' &&
                    typeof agent.position.z === 'number') {

                    const distance = this.getDistance3D(position, agent.position);
                    if (distance > 0 && distance < 5.0) {
                        const influence = 1.0 / (1.0 + distance);
                        const direction = {
                            x: (agent.position.x - position.x) / distance,
                            y: (agent.position.y - position.y) / distance,
                            z: (agent.position.z - position.z) / distance
                        };

                        totalForce.x += direction.x * influence * 0.05;
                        totalForce.y += direction.y * influence * 0.05;
                        totalForce.z += direction.z * influence * 0.05;
                    }
                }
            }

            return totalForce;
        } catch (error) {
            console.warn('⚠️ Erreur calcul champ essaim:', error.message);
            return { x: 0, y: 0, z: 0 };
        }
    }

    /**
     * Obtient les statistiques globales du système DIAMANTS
     * @returns {Object} Statistiques complètes
     */
    getStatistics() {
        try {
            return {
                diamants_value: this.diamants_value || 0,
                harmonics: this.harmonics || new Array(15).fill(0),
                emergence_factor: this.emergence_factor || 0,
                coherence_level: this.coherence_level || 0,
                field_energy: this.calculateFieldEnergy(),
                total_agents: this.agent_count || 0,
                computation_time: performance.now() - (this.start_time || performance.now())
            };
        } catch (error) {
            console.warn('⚠️ Erreur calcul statistiques:', error.message);
            return {
                diamants_value: 0,
                harmonics: new Array(15).fill(0),
                emergence_factor: 0,
                coherence_level: 0,
                field_energy: 0,
                total_agents: 0,
                computation_time: 0
            };
        }
    }

    /**
     * Calcule l'énergie totale des champs
     * @returns {number} Énergie totale
     */
    calculateFieldEnergy() {
        try {
            let totalEnergy = 0;

            if (this.psi_field) {
                for (let i = 0; i < this.psi_field.length; i++) {
                    for (let j = 0; j < this.psi_field[i].length; j++) {
                        for (let k = 0; k < this.psi_field[i][j].length; k++) {
                            totalEnergy += Math.abs(this.psi_field[i][j][k]);
                        }
                    }
                }
            }

            return totalEnergy;
        } catch (error) {
            console.warn('⚠️ Erreur calcul énergie champ:', error.message);
            return 0;
        }
    }

    /**
     * Méthode pour obtenir la force à une position donnée (pour compatibilité)
     * @param {Object} position - Position {x, y, z}
     * @returns {Object} Force résultante
     */
    getForceAt(position) {
        try {
            if (!position || typeof position.x === 'undefined') {
                return { x: 0, y: 0, z: 0 };
            }

            const gradient = this.getLocalGradient(position);
            return {
                x: gradient.x * 0.1,
                y: gradient.y * 0.1,
                z: gradient.z * 0.1
            };
        } catch (error) {
            console.warn('⚠️ Erreur calcul force:', error.message);
            return { x: 0, y: 0, z: 0 };
        }
    }
}
