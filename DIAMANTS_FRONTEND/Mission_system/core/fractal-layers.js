/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * Fractal Hypervision — définitions L0→L5 (BattleVerse / D2.2)
 * Même logique TACON/TACOM à chaque échelle ; L4/L5 = couche basse ROS.
 */

export const FRACTAL_LAYERS = [
    {
        id: 'L0',
        name: 'Théâtre',
        authority: 'OPCON / OPCOM',
        color: '#1F3864',
        source: 'local',
        collect(system, rosState) {
            const doctrine = window.DIAMANTS_DOCTRINE;
            const ic = system?.integratedController;
            const coverage = ic?.metrics?.coveragePercentage ?? 0;
            const phase = doctrine?.missionPhase || ic?.missionStarted ? 'active' : 'idle';
            return {
                status: phase === 'active' ? 'ok' : 'idle',
                summary: `Phase ${phase} · couverture ${coverage.toFixed(0)}%`,
                metrics: { phase, coverage },
            };
        },
    },
    {
        id: 'L1',
        name: 'Composante',
        authority: 'TACOM',
        color: '#2F5496',
        source: 'hybrid',
        collect(system, rosState) {
            const mission = rosState?.mission || {};
            const local = system?.integratedController?.missionStarted ? 'running' : 'idle';
            const status = mission.status || local;
            const fromRos = rosState?.connected && !!mission.status;
            return {
                status: status === 'active' || status === 'running' ? 'ok' : 'idle',
                summary: `Mission ${status}`,
                metrics: { status, waypoints: mission.waypoints?.length ?? 0 },
                ros: fromRos,
            };
        },
    },
    {
        id: 'L2',
        name: 'Cellule conduite',
        authority: 'TACON',
        color: '#4472C4',
        source: 'ros',
        collect(system, rosState) {
            const swarm = rosState?.swarm || {};
            const status = swarm.status || swarm.phase || 'idle';
            const score = swarm.intelligence_score ?? 0;
            const hasRos = rosState?.connected && Object.keys(rosState?.swarm || {}).length > 1;
            return {
                status: hasRos ? 'ok' : (system?.integratedController ? 'sim' : 'offline'),
                summary: hasRos
                    ? `Essaim ${status} · I=${score.toFixed(2)}`
                    : 'SwarmController ROS offline — moteur local',
                metrics: { status, intelligence_score: score, coverage_area: swarm.coverage_area ?? 0 },
                ros: hasRos,
            };
        },
    },
    {
        id: 'L3',
        name: 'Essaim',
        authority: 'Coordinateur',
        color: '#5B9BD5',
        source: 'hybrid',
        collect(system, rosState) {
            const drones = system?.drones || system?.integratedController?.drones || [];
            const flying = drones.filter((d) => d.state === 'FLYING' || d.state === 'EXPLORING').length;
            const stig = window.DIAMANTS_STIGMERGY_INSTANCE;
            const cells = system?.integratedController?.autonomousFlightEngine?.visitedCells?.size ?? 0;
            return {
                status: flying > 0 ? 'ok' : 'idle',
                summary: `${drones.length} plateformes · ${flying} actives · ${cells} cellules`,
                metrics: { droneCount: drones.length, flying, stigmergy: !!stig, visitedCells: cells },
            };
        },
    },
    {
        id: 'L4',
        name: 'Plateforme',
        authority: 'Calculateur embarqué',
        color: '#22c55e',
        source: 'hybrid',
        collect(system, rosState) {
            const drones = system?.drones || system?.integratedController?.drones || [];
            const now = Date.now();
            let backend = 0;
            let local = 0;
            for (const d of drones) {
                const rd = d?.rosData;
                const isBackend = rd?.position && rd.source !== 'engine'
                    && (now - (rd.lastUpdate || 0)) < 5000;
                if (isBackend) backend++;
                else if (d.state !== 'IDLE') local++;
            }
            const cas = window.DIAMANTS?.casLevel ?? 2;
            return {
                status: backend > 0 ? 'ros' : (local > 0 ? 'sim' : 'idle'),
                summary: backend > 0
                    ? `${backend} drones ROS · ${local} sim`
                    : `${local} drones moteur PID (CAS-${cas})`,
                metrics: { backendDriven: backend, engineDriven: local, total: drones.length },
                ros: backend > 0,
            };
        },
    },
    {
        id: 'L5',
        name: 'Fonction élémentaire',
        authority: 'Capteur / effecteur',
        color: '#f97316',
        source: 'ros',
        collect(system, rosState) {
            const elementary = rosState?.elementary || {};
            const drones = elementary.drones || {};
            const entries = Object.entries(drones);
            const fresh = entries.filter(([, v]) => v?.odom_fresh).length;
            const total = entries.length || (system?.drones?.length ?? 0);
            const wsOk = rosState?.connected;
            if (!wsOk && !fresh) {
                const engine = !!system?.integratedController?.autonomousFlightEngine;
                return {
                    status: engine ? 'sim' : 'offline',
                    summary: engine
                        ? 'Pas de ROS — capteurs simulés (MultiRanger local)'
                        : 'Couche ROS inactive',
                    metrics: { odom_topics: 0, fresh: 0, mode: 'simulated' },
                    ros: false,
                };
            }
            return {
                status: fresh === total && total > 0 ? 'ok' : fresh > 0 ? 'degraded' : 'offline',
                summary: `Odom Gazebo ${fresh}/${total} · scan/cmd_vel actifs`,
                metrics: {
                    odom_topics: total,
                    fresh,
                    stale: total - fresh,
                    gz_sim: elementary.gz_sim ?? false,
                },
                ros: fresh > 0,
            };
        },
    },
];

/** Fusionne l'état ROS backend dans un snapshot fractal complet. */
export function buildFractalSnapshot(system, rosState = {}) {
    const layers = FRACTAL_LAYERS.map((def) => ({
        id: def.id,
        name: def.name,
        authority: def.authority,
        color: def.color,
        source: def.source,
        ...def.collect(system, rosState),
    }));
    const rosDepth = layers.filter((l) => l.ros).length;
    return {
        timestamp: Date.now(),
        casLevel: window.DIAMANTS?.casLevel ?? 2,
        wsConnected: !!rosState.connected,
        rosDepth,
        layers,
    };
}

/** État élémentaire L5 dérivé des positions ROS reçues par le frontend. */
export function buildElementaryFromDrones(drones = []) {
    const now = Date.now();
    const out = {};
    for (const d of drones) {
        const id = d.id || d.name;
        if (!id) continue;
        const rd = d.rosData;
        const fresh = rd?.position && rd.source !== 'engine'
            && (now - (rd.lastUpdate || 0)) < 2000;
        out[id] = {
            odom_fresh: !!fresh,
            battery: rd?.battery ?? d.battery,
            status: rd?.status || d.state,
            last_update_ms: rd?.lastUpdate ? now - rd.lastUpdate : null,
        };
    }
    return { drones: out, gz_sim: Object.values(out).some((v) => v.odom_fresh) };
}
