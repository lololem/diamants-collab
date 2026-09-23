/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */
/**
 * FLEET & MODELS — add a drone and give it a model, without the command line.
 *
 * The file route stays: a JSON in physics/profiles/, an entry in
 * agent-models.json. This panel does the same thing while the simulation is
 * running, so a contributor can try an airframe and a model in a few seconds
 * and see the result in the air immediately.
 *
 *   1. Drone    — load a profile file (or paste it), then put one in the air.
 *   2. Model    — pick a provider and a model per profile; the list of models
 *                 is read from the Ollama server, not typed from memory.
 *   3. Rules    — unchanged: whatever you plug in goes through them.
 *
 * Nothing here decides anything: it wires your profile and your model into the
 * same paths the files use (DronePhysicsRegistry, NeuroSymbolicIntelligenceManager).
 */

import { DronePhysicsRegistry } from '../physics/drone-physics-registry.js';

const css = (el, s) => Object.assign(el.style, s);
const OLLAMA_DEFAUT = 'http://localhost:11434';

const CHAMPS = {
    ollama: ['model', 'baseUrl'],
    'openai-compatible': ['model', 'baseUrl'],
    'http-policy': ['url', 'name'],
};

export function initFleetModelsPanel(controllerInitial) {
    if (typeof document === 'undefined' || window.__fleetModelsPanel) return window.__fleetModelsPanel;

    /* Le panneau est monté pendant l'initialisation : le contrôleur peut ne pas
     * exister encore, et le capturer donnerait un null définitif. On le résout
     * à chaque usage. */
    const ctrl = () => controllerInitial
        || window.DIAMANTS?.controller
        || window.diamantsSystem?.integratedController
        || null;

    const bouton = document.createElement('button');
    bouton.id = 'fleet-models-fab';
    bouton.type = 'button';
    bouton.textContent = '🚁 Fleet & models';
    bouton.title = 'Add a drone profile and give it a model';
    css(bouton, {
        position: 'fixed', right: '18px', bottom: '150px', zIndex: '5200', cursor: 'pointer',
        background: 'linear-gradient(135deg,#123048,#0b1c2c)', color: '#cfe9ff', border: '1px solid #2f6f9e',
        borderRadius: '10px', padding: '9px 13px', font: '600 12px system-ui, -apple-system, Segoe UI, sans-serif',
        boxShadow: '0 6px 18px rgba(0,0,0,0.45)',
    });
    document.body.appendChild(bouton);

    const panneau = document.createElement('div');
    panneau.id = 'fleet-models-panel';
    css(panneau, {
        position: 'fixed', right: '18px', bottom: '196px', width: '392px', maxHeight: '72vh', overflowY: 'auto',
        zIndex: '5201', display: 'none', background: 'rgba(6,12,22,0.96)', border: '1px solid rgba(90,170,230,0.5)',
        borderRadius: '12px', padding: '14px 16px', color: '#dceaf8',
        font: '12.5px/1.5 system-ui, -apple-system, Segoe UI, sans-serif', boxShadow: '0 12px 34px rgba(0,0,0,0.6)',
    });
    document.body.appendChild(panneau);
    bouton.addEventListener('click', () => {
        const ouvert = panneau.style.display !== 'none';
        panneau.style.display = ouvert ? 'none' : 'block';
        if (!ouvert) rafraichir();
    });

    const h = (t, marge = '14px') => `<div style="font-weight:700;color:#8fd8ff;letter-spacing:.03em;margin:${marge} 0 6px">${t}</div>`;
    panneau.innerHTML = `
        <div style="font-weight:800;font-size:13.5px;letter-spacing:.04em">FLEET &amp; MODELS</div>
        <div style="opacity:.7;font-size:11.5px">Add an airframe, give it a model, watch it fly.</div>
        ${h('1 · Drone profile')}
        <input type="file" id="fm-fichier" accept="application/json,.json" style="width:100%;font-size:11.5px">
        <textarea id="fm-json" rows="4" placeholder='or paste a profile: { "id": "MY_DRONE", "label": "…", "physical": {…}, "performance": {…}, "pid": {…}, "visual": { "model": "x500" } }'
            style="width:100%;margin-top:6px;background:#02080f;color:#bfe0ff;border:1px solid #1d3550;border-radius:6px;padding:6px;font:11px ui-monospace,Menlo,monospace"></textarea>
        <div style="display:flex;gap:6px;margin-top:6px">
            <button id="fm-ajouter" style="flex:1">Add profile</button>
            <button id="fm-exemple" title="Fill the box with a valid example">Example</button>
        </div>
        <div id="fm-profils" style="margin-top:8px"></div>
        ${h('2 · Model for a profile')}
        <div style="display:flex;gap:6px;align-items:center">
            <select id="fm-profil-cible" style="flex:1"></select>
            <select id="fm-provider">
                <option value="ollama">ollama</option>
                <option value="openai-compatible">openai-compatible</option>
                <option value="http-policy">http-policy</option>
            </select>
        </div>
        <div id="fm-champs" style="margin-top:6px"></div>
        <div style="display:flex;gap:6px;margin-top:6px;align-items:center">
            <label style="opacity:.8">min confidence</label>
            <input id="fm-conf" type="number" min="0" max="1" step="0.05" value="0.6" style="width:70px">
            <button id="fm-appliquer" style="flex:1">Apply to fleet</button>
        </div>
        <div id="fm-etat" style="margin-top:8px;font-size:11.5px;opacity:.85"></div>
        ${h('3 · Rules')}
        <div style="opacity:.8;font-size:11.5px">Whatever you plug in goes through the rule layer:
            a critical rule vetoes before the model is asked, a forbidden action or a low confidence is
            dropped, and an accepted decision only nudges the next waypoint (8&nbsp;m max).</div>
        <div id="fm-stats" style="margin-top:8px;font-size:11.5px"></div>`;

    for (const b of panneau.querySelectorAll('button')) {
        css(b, { background: '#12314a', color: '#dceaf8', border: '1px solid #2f6f9e', borderRadius: '6px', padding: '5px 8px', cursor: 'pointer', font: '600 11.5px system-ui' });
    }
    for (const s of panneau.querySelectorAll('select, input[type=number]')) {
        css(s, { background: '#02080f', color: '#cfe9ff', border: '1px solid #1d3550', borderRadius: '6px', padding: '4px 6px', font: '11.5px system-ui' });
    }

    const $ = (id) => panneau.querySelector(id);
    const dire = (texte, bon = true) => { $('#fm-etat').innerHTML = `<span style="color:${bon ? '#5ef2a0' : '#ff6b7a'}">${texte}</span>`; };

    // ─── 1. profils ──────────────────────────────────────────────────
    const registre = () => DronePhysicsRegistry.getInstance();

    const listerProfils = () => {
        const r = registre();
        if (r?.listProfiles) return r.listProfiles();
        return (window.DIAMANTS?.listDroneProfiles?.() || []).map(p => p.id || p);
    };

    const ajouterProfil = (texte) => {
        let brut;
        try { brut = JSON.parse(texte); } catch (e) { return dire(`Not valid JSON: ${e.message}`, false); }
        for (const clef of ['id', 'label', 'physical', 'performance', 'pid']) {
            if (!brut[clef]) return dire(`Missing "${clef}" — see profiles/drone-profile.schema.json`, false);
        }
        const r = registre();
        if (!r?.registerCustomProfile) return dire('Profile registry unavailable', false);
        r.registerCustomProfile(brut);
        dire(`${brut.id} added — ${brut.visual?.model && brut.visual.model !== 'generic'
            ? `it will fly with the ${brut.visual.model} airframe`
            : 'tip: set visual.model to x500, s500 or crazyflie for a real airframe'}`);
        rafraichir();
    };

    $('#fm-fichier').addEventListener('change', (e) => {
        const f = e.target.files?.[0];
        if (!f) return;
        const lecteur = new FileReader();
        lecteur.onload = () => { $('#fm-json').value = String(lecteur.result); ajouterProfil(String(lecteur.result)); };
        lecteur.readAsText(f);
    });
    $('#fm-ajouter').addEventListener('click', () => ajouterProfil($('#fm-json').value));
    $('#fm-exemple').addEventListener('click', () => {
        $('#fm-json').value = JSON.stringify({
            id: 'SURVEY_X', label: 'Survey X (example)', manufacturer: 'You', category: 'medium',
            physical: { mass: 1.6, armLength: 0.28, boundingRadius: 0.55, propCount: 4 },
            performance: { maxSpeed: 8, maxClimb: 3, cruiseAlt: 6, maxAlt: 40, agility: 1.1, explorationRadius: 80, endurance_min: 22 },
            pid: { pos: { kp: 2.2, ki: 0.05, kd: 1.1 }, alt: { kp: 3.1, ki: 0.1, kd: 1.3 }, yaw: { kp: 1.9, ki: 0, kd: 0.3 } },
            visual: { scale: 11, color: '0xffaa22', model: 'x500' },
        }, null, 2);
    });

    let n = 0;
    const poser = async (profileId) => {
        const id = `${profileId.toLowerCase()}_${String(++n).padStart(2, '0')}`;
        const a = 0.9 + n * 1.3;
        const ok = await window.DIAMANTS?.spawnDrone?.(id, profileId, { x: Math.cos(a) * 6, y: 0.4, z: Math.sin(a) * 6 });
        if (!ok) return dire(`Could not spawn ${profileId}`, false);
        window.launchMission?.();
        window.takeoffAllDrones?.();
        /* Décoller ne suffit pas : sans ordre d'exploration l'appareil reste en
         * vol stationnaire — et un drone qui n'explore pas ne consulte jamais
         * son modèle. */
        await new Promise(r => setTimeout(r, 6000));
        ctrl()?.autonomousFlightEngine?.startExploration?.();
        const etat = ctrl()?.autonomousFlightEngine?.drones?.get(id);
        dire(`${id} in the air — profile ${etat?.profile?.id || '?'}, exploring`);
        rafraichir();
    };

    // ─── 2. modèles ──────────────────────────────────────────────────
    let modelesOllama = [];
    const lireModelesOllama = async (base) => {
        try {
            const r = await fetch(`${(base || OLLAMA_DEFAUT).replace(/\/$/, '')}/api/tags`, { cache: 'no-store' });
            if (!r.ok) throw new Error(`HTTP ${r.status}`);
            modelesOllama = ((await r.json()).models || []).map(m => m.name);
        } catch (_) { modelesOllama = []; }
        return modelesOllama;
    };

    const dessinerChamps = () => {
        const p = $('#fm-provider').value;
        const champs = CHAMPS[p];
        $('#fm-champs').innerHTML = champs.map(c => {
            if (c === 'model') {
                return `<div style="display:flex;gap:6px;align-items:center;margin-bottom:4px">
                    <label style="width:72px;opacity:.8">model</label>
                    <input id="fm-model" list="fm-modeles" placeholder="model name" style="flex:1">
                    <datalist id="fm-modeles">${modelesOllama.map(m => `<option value="${m}">`).join('')}</datalist></div>`;
            }
            const valeur = c === 'baseUrl' ? OLLAMA_DEFAUT : c === 'url' ? 'http://localhost:9000/decide' : '';
            return `<div style="display:flex;gap:6px;align-items:center;margin-bottom:4px">
                <label style="width:72px;opacity:.8">${c}</label>
                <input id="fm-${c}" value="${valeur}" style="flex:1"></div>`;
        }).join('');
        for (const i of $('#fm-champs').querySelectorAll('input')) {
            css(i, { background: '#02080f', color: '#cfe9ff', border: '1px solid #1d3550', borderRadius: '6px', padding: '4px 6px', font: '11.5px ui-monospace,Menlo,monospace' });
        }
    };
    $('#fm-provider').addEventListener('change', dessinerChamps);

    /** Le pont n'existe que si un agent-models.json était présent au démarrage :
     *  s'il manque, on le crée ici, avec le registre que l'on vient de remplir. */
    const pont = async (registreModeles) => {
        const c = ctrl(); if (!c) throw new Error('simulation not ready yet');
        const actuel = c.droneIntelligenceManager;
        if (actuel && actuel.constructor?.name === 'NeuroSymbolicIntelligenceManager') {
            Object.assign(actuel.registry, registreModeles);
            return actuel;
        }
        const { NeuroSymbolicIntelligenceManager } = await import('../intelligence/neurosymbolic-bridge.js');
        const m = new NeuroSymbolicIntelligenceManager(registreModeles);
        for (const [id, etat] of c.autonomousFlightEngine?.drones || []) m.registerDrone(id, etat.profile?.id);
        m.setGlobalEnabled(true);
        c.droneIntelligenceManager = m;
        c.autonomousFlightEngine?.setIntelligenceManager(m);
        return m;
    };

    $('#fm-appliquer').addEventListener('click', async () => {
        const profileId = $('#fm-profil-cible').value;
        const provider = $('#fm-provider').value;
        const entree = { provider, minConfidence: +$('#fm-conf').value || 0.6, timeoutMs: 20000 };
        for (const c of CHAMPS[provider]) {
            const v = panneau.querySelector(`#fm-${c}`)?.value?.trim();
            if (c === 'model' && !v) return dire('Pick a model first', false);
            if (v) entree[c] = v;
        }
        entree.allowedActions = ['EXPLORE', 'AVOID', 'HOVER', 'RTL', 'COORDINATE'];
        entree.systemPrompt = 'You are a survey drone. Answer with one JSON object: '
            + '{"action": "EXPLORE|AVOID|HOVER|RTL|COORDINATE", "direction": "N|S|E|W|NE|NW|SE|SW", '
            + '"confidence": 0.0-1.0, "reasoning": "40 characters max"}';
        try {
            const m = await pont({ [profileId]: entree, _settings: { maxConcurrent: 2 } });
            for (const [id, etat] of ctrl()?.autonomousFlightEngine?.drones || []) {
                if (etat.profile?.id === profileId && !m.brains.has(id)) m.registerDrone(id, profileId);
            }
            m.setGlobalEnabled(true);
            const n = [...m.brains.values()].filter(b => b.type === profileId).length;
            dire(`${provider} → ${entree.model || entree.url} · ${n} drone${n > 1 ? 's' : ''} of ${profileId} now decide with it`);
        } catch (e) {
            dire(`Could not plug the model: ${e.message}`, false);
        }
        rafraichir();
    });

    // ─── rafraîchissement ────────────────────────────────────────────
    const rafraichir = async () => {
        const profils = listerProfils();
        const cible = $('#fm-profil-cible');
        const choisi = cible.value;
        cible.innerHTML = profils.map(p => `<option value="${p}">${p}</option>`).join('');
        if (profils.includes(choisi)) cible.value = choisi;

        const enVol = new Map();
        for (const [, etat] of ctrl()?.autonomousFlightEngine?.drones || []) {
            const k = etat.profile?.id || '?';
            enVol.set(k, (enVol.get(k) || 0) + 1);
        }
        $('#fm-profils').innerHTML = profils.map(p => `
            <div style="display:flex;gap:6px;align-items:center;margin:3px 0">
                <span style="flex:1">${p} <span style="opacity:.6">· ${enVol.get(p) || 0} flying</span></span>
                <button data-poser="${p}" style="padding:3px 7px">Put one in the air</button>
            </div>`).join('');
        for (const b of $('#fm-profils').querySelectorAll('button')) {
            css(b, { background: '#12314a', color: '#dceaf8', border: '1px solid #2f6f9e', borderRadius: '6px', cursor: 'pointer', font: '600 11px system-ui' });
            b.addEventListener('click', () => poser(b.dataset.poser));
        }

        const base = panneau.querySelector('#fm-baseUrl')?.value || OLLAMA_DEFAUT;
        const modeles = await lireModelesOllama(base);
        const liste = panneau.querySelector('#fm-modeles');
        if (liste) liste.innerHTML = modeles.map(m => `<option value="${m}">`).join('');

        const m = ctrl()?.droneIntelligenceManager;
        const s = m?.getStats?.();
        $('#fm-stats').innerHTML = `
            <div>Ollama: ${modeles.length ? `<b style="color:#5ef2a0">${modeles.length} models</b>` : '<span style="color:#ff9f43">unreachable</span>'}</div>
            ${s ? `<div>decisions accepted <b>${s.accepted ?? 0}</b> · rejected <b>${s.rejected ?? 0}</b> · vetoed by rules <b>${s.vetoes ?? 0}</b> · failed <b>${s.failures ?? 0}</b></div>` : ''}`;
    };

    dessinerChamps();
    rafraichir();
    setInterval(() => { if (panneau.style.display !== 'none') rafraichir(); }, 4000);

    const api = { panneau, bouton, ouvrir: () => { panneau.style.display = 'block'; rafraichir(); }, fermer: () => { panneau.style.display = 'none'; }, rafraichir, ajouterProfil, poser };
    window.__fleetModelsPanel = api;
    return api;
}

export default initFleetModelsPanel;
