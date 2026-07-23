# DIAMANTS — Distributed Autonomous Multi-Agent Systems

Plateforme 3D de simulation et de visualisation d'essaims de drones
hétérogènes, exécutée dans le navigateur.

DIAMANTS est une **infrastructure** : vous décrivez votre drone dans un fichier
JSON, vous implémentez une interface JavaScript pour votre algorithme d'essaim,
et le système prend en charge le reste — rendu 3D, contrôle PID, évitement de
collisions, exploration autonome.

> **Licence : PolyForm Noncommercial 1.0.0.** Vous pouvez télécharger, étudier,
> modifier et redistribuer ce logiciel pour tout usage **non commercial**
> (recherche, enseignement, projets personnels, organisations à but non
> lucratif). Toute exploitation commerciale est interdite. Voir [LICENSE](LICENSE).

---

## Ce que fait le projet

- Simulation 3D d'essaims hétérogènes (Crazyflie, X500, S500, ou votre drone)
- Moteur de vol autonome à contrôle PID, dans le navigateur
- Système de greffons : profils de drones (JSON) et intelligence d'essaim (JS)
- Doctrines commutables en temps réel
- Minimap d'exploration avec suivi de couverture
- Destiné à la recherche, l'enseignement et le prototypage d'algorithmes

---

## Installation

### Prérequis

| Outil | Version | Note |
|-------|---------|------|
| Node.js | **20.x** | testé sur 20.20.0 ; `nvm` recommandé |
| npm | 10.x | fourni avec Node 20 |
| Navigateur | récent | WebGL 2 requis |

Une carte graphique n'est pas indispensable, mais le rendu logiciel est très
lent — comptez quelques images par seconde.

### Étapes

```bash
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab/DIAMANTS_FRONTEND/Mission_system

# Node 20 recommandé
nvm use 20        # ou : nvm install 20

npm install
npm run dev
```

Ouvrez ensuite <http://localhost:5550>. Les drones apparaissent sur l'héliport
et explorent de façon autonome.

### Autres commandes

```bash
npm run build      # build de production dans dist/
npm run preview    # sert le build de production
npm test           # tests unitaires (vitest)
npm run coverage   # tests + couverture
```

### En cas de problème

| Symptôme | Cause probable |
|----------|----------------|
| Page noire, « WebGL non disponible » | accélération matérielle désactivée dans le navigateur |
| Très basse fréquence d'images | rendu logiciel ; vérifiez que le GPU est utilisé |
| `npm install` échoue | version de Node ≠ 20.x |
| Port 5550 occupé | modifiez `port` dans `scripts/dev.js` |

---

## Ce que ce dépôt ne contient pas

Ce dépôt est la **partie publique** du projet. Plusieurs modules de recherche
restent privés et sont remplacés ici par des **stubs** : des fichiers qui
exposent la même API mais ne font rien. L'application démarre, les drones
volent, l'interface est complète — seuls les comportements ci-dessous sont
inertes.

| Module stubbé | Effet dans ce dépôt |
|---------------|---------------------|
| `intelligence/stigmergy-engine.js` | pas d'intelligence stigmergique ; le chargeur bascule sur un repli |
| `intelligence/distributed-swarm-engine.js` | pas d'agents distribués |
| `intelligence/swarm-comm-manager.js` | pas de radio simulée : les drones ne partagent pas leur carte |
| `intelligence/drone-intelligence.js` | pas d'appel aux modèles de langage locaux |
| `intelligence/scenario-engine.js` | catalogue de scénarios vide |
| `intelligence/optimized-search.js` | recherche hiérarchique non implémentée |
| `core/diamants-formulas.js` | métriques de champ inertes |

Chaque stub porte un en-tête expliquant ce qu'il remplace. Pour brancher votre
propre implémentation, remplacez simplement le fichier : les interfaces sont
documentées dans `intelligence/swarm-intelligence-interface.js` et
`intelligence/stigmergy-interface.js`.

**Note sur le panneau « Intelligence LLM ».** Il s'appuie sur un serveur
[Ollama](https://ollama.com) local, qui ne peut pas être fourni par un dépôt
Git. En son absence, le panneau affiche des décisions **simulées** à titre de
démonstration ; ce ne sont pas des sorties de modèle.

---

## Structure du projet

```
diamants-collab/
  DIAMANTS_FRONTEND/Mission_system/     # Application principale
    main.js                             # Point d'entrée, boucle de rendu
    index.html                          # Interface et panneaux
    physics/
      autonomous-flight-engine.js       # Moteur de vol PID (cœur)
      drone-physics-registry.js         # Chargement des profils
      pid-controller.js                 # Contrôleur PID
      profiles/                         # Déposez votre JSON de drone ici
    intelligence/                       # Interfaces d'essaim (+ stubs)
    behaviors/                          # Motifs de vol et reconnaissance
    missions/                           # Missions et doctrines
    environment/                        # Terrain, végétation, ciel
    shaders/                            # Shaders GLSL (herbe, ciel)
    drones/                             # Modèles et visuels de drones
    ui/                                 # Panneaux, minimaps, overlays
    core/                               # État applicatif, événements
    assets/                             # Maillages, textures
    third-party/ez-tree/                # Générateur d'arbres (licence propre)
  DIAMANTS_BACKEND/                     # Ponts ROS 2 / SLAM (optionnel)
  DIAMANTS_API/                         # Passerelle WebSocket (optionnel)
  DEMO/                                 # Vidéos et données d'exemple
```

Le frontend fonctionne **seul**. Le backend et l'API ne sont nécessaires que
pour piloter des drones réels ou une simulation Gazebo.

---

## Ajouter votre drone

Créez un fichier JSON dans `DIAMANTS_FRONTEND/Mission_system/physics/profiles/` :

```json
{
    "id": "MY_DRONE",
    "label": "My Custom Drone",
    "manufacturer": "Your Company",
    "category": "compact",
    "physical": {
        "mass": 0.5,
        "armLength": 0.15,
        "boundingRadius": 0.4,
        "propCount": 4
    },
    "performance": {
        "maxSpeed": 5.0,
        "maxClimb": 2.0,
        "cruiseAlt": 5.0,
        "maxAlt": 20.0,
        "agility": 1.2,
        "explorationRadius": 80,
        "endurance_min": 15
    },
    "pid": {
        "pos":  { "kp": 2.5, "ki": 0.05, "kd": 1.0 },
        "alt":  { "kp": 3.5, "ki": 0.1,  "kd": 1.2 },
        "yaw":  { "kp": 2.0, "ki": 0.0,  "kd": 0.3 }
    },
    "visual": {
        "scale": 20,
        "color": "0xFF6600",
        "model": "generic"
    }
}
```

Champs obligatoires : `id`, `label`, `physical`, `performance`, `pid`. Le schéma
complet est dans `profiles/drone-profile.schema.json`. Le moteur charge tous les
JSON du répertoire au démarrage.

---

## Ajouter votre intelligence d'essaim

Implémentez `SwarmIntelligenceInterface`
(`intelligence/swarm-intelligence-interface.js`) :

```javascript
import { SwarmIntelligenceInterface } from './swarm-intelligence-interface.js';

export class MySwarmAlgorithm extends SwarmIntelligenceInterface {
    constructor() {
        super();
        this.name = 'my-algorithm';
        this.version = '1.0.0';
    }

    initialize(config) {
        // Appelé une fois au démarrage : { droneCount, arena, profiles }
    }

    computeInfluences(droneStates, dt) {
        // Appelé à chaque frame.
        // droneStates : Map<id, {position, velocity, target, ...}>
        // Retour     : Map<id, {targetModifier, velocityBias, priorityOverride}>
        return new Map();
    }
}
```

Le moteur appelle `computeInfluences()` à chaque frame et fusionne vos sorties
avec le contrôleur PID. **Vous observez, vous suggérez — le PID décide.**

---

## Pile technique

| Composant | Technologie | Version |
|-----------|-------------|---------|
| Moteur 3D | Three.js | 0.167.x |
| Build | Vite | 4.5.3 |
| Tests | Vitest | 1.6.x |
| Exécution | Node.js | 20.x |
| Langage | modules ES6 | — |

---

## Vidéos

- [Démo frontend 3D](https://www.youtube.com/watch?v=fyEmYu4lbzo) — interface Three.js
- [Systèmes multi-agents](https://www.youtube.com/watch?v=1Av_o-9fzrE) — coordination distribuée
- [Navigation par gradient](https://www.youtube.com/watch?v=ElABxOde6ak) — planification
- [Coordination d'essaim](https://www.youtube.com/watch?v=L8V64LajM2w) — formations
- [Démo stigmergie](https://www.youtube.com/watch?v=SyqeRwcbDO4) — coordination bio-inspirée

D'autres démonstrations sont dans `DEMO/`.

---

## Contribuer

Les contributions sont bienvenues, dans le cadre non commercial de la licence :

- ajouter un profil de drone (JSON + modèle 3D optionnel)
- implémenter un algorithme d'essaim (étendre l'interface JS)
- améliorer la visualisation 3D
- corriger des bugs, écrire des tests, améliorer la documentation

Fork, branche, pull request. Voir [Contributing.md](Contributing.md).

En proposant une contribution, vous acceptez qu'elle soit distribuée sous la
même licence que le projet.

---

## Licence

**PolyForm Noncommercial License 1.0.0** — voir [LICENSE](LICENSE).

En résumé : usage libre pour la recherche, l'enseignement, l'étude personnelle
et les organisations à but non lucratif. **Toute exploitation commerciale est
interdite.**

Les composants tiers embarqués (EZ-Tree, Crazyswarm2, paquets npm) restent sous
leur propre licence ; la section « Composants tiers » du fichier LICENSE les
énumère.

Ce dépôt a été distribué sous licence MIT jusqu'au 23 juillet 2026 ; les copies
antérieures restent régies par ces termes.

---

## Contact

- Issues : [github.com/lololem/diamants-collab/issues](https://github.com/lololem/diamants-collab/issues)
