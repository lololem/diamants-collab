# DIAMANTS

**Faire voler un essaim de drones dans votre navigateur, et y brancher votre propre intelligence.**

DIAMANTS est un terrain de jeu pour la recherche en essaims. Vous ouvrez une page
web, une flotte de drones décolle d'un héliport au milieu d'une forêt et part
explorer. Tout tourne en local, sans serveur, sans compte à créer.

L'idée derrière : la partie pénible d'un projet d'essaim — le rendu 3D, la
physique de vol, les collisions, les modèles de drones — est déjà faite. Vous
arrivez avec votre algorithme de coordination, vous l'écrivez dans une interface
JavaScript, et vous le regardez piloter la flotte. Vous voulez un autre drone ?
C'est un fichier JSON.

> **Licence : PolyForm Noncommercial 1.0.0.** Téléchargez, étudiez, modifiez,
> redistribuez — pour la recherche, l'enseignement, vos projets personnels ou une
> organisation à but non lucratif. En revanche, pas d'exploitation commerciale.
> Le texte complet est dans [LICENSE](LICENSE).

---

## Démarrer

Il vous faut **Node.js 20 ou plus récent** et un navigateur avec WebGL 2
(n'importe quel Chrome, Firefox ou Edge des dernières années).

```bash
git clone https://github.com/lololem/diamants-collab.git
cd diamants-collab/DIAMANTS_FRONTEND/Mission_system
npm install
npm run dev
```

L'installation télécharge un peu moins de 800 paquets et prend une demi-minute.
Vous verrez passer des avertissements `npm warn deprecated` : ils viennent de
dépendances indirectes et sont sans conséquence.

Vite vous affiche ensuite l'adresse à ouvrir, en général
**http://localhost:5550**. Si ce port est déjà pris, il en choisit un autre tout
seul et vous le dit — il n'y a rien à configurer.

Les drones apparaissent sur l'héliport et commencent à explorer.

**Si l'écran reste noir**, c'est presque toujours l'accélération matérielle
désactivée dans le navigateur. Si tout est très lent (quelques images par
seconde), c'est que le rendu se fait sur le processeur au lieu de la carte
graphique.

Quelques autres commandes utiles :

```bash
npm run build      # construit la version de production dans dist/
npm run preview    # sert cette version
npm test           # lance la suite de tests
```

---

## Ce que ce dépôt ne contient pas

Autant le dire tout de suite pour vous éviter de chercher.

DIAMANTS est développé dans un dépôt privé, et une partie du travail de
recherche n'est pas publiée. Concrètement, **sept modules sont remplacés par des
coquilles vides** : des fichiers qui ont la bonne forme, que le reste du code
peut appeler sans planter, mais qui ne font rien.

L'application démarre normalement, les drones volent, explorent et évitent les
obstacles. Ce qui manque, c'est leur **intelligence collective** :

| Ce qui est remplacé | Ce que vous perdez |
|---|---|
| `stigmergy-engine.js` | les drones ne déposent pas de phéromones et ne s'en servent pas pour choisir où aller |
| `distributed-swarm-engine.js` | pas d'agents autonomes coordonnés |
| `swarm-comm-manager.js` | les drones ne se parlent pas et ne partagent pas leur carte |
| `drone-intelligence.js` | pas d'appel aux modèles de langage locaux |
| `scenario-engine.js` | la liste de scénarios est vide |
| `optimized-search.js` | la recherche hiérarchique n'est pas implémentée |
| `core/diamants-formulas.js` | les métriques de champ restent à zéro |

Chacune de ces coquilles porte un en-tête qui explique ce qu'elle remplace.
**Rien ne vous empêche d'écrire la vôtre** : c'est même l'usage prévu. Les
contrats à respecter sont documentés dans
`intelligence/swarm-intelligence-interface.js` et
`intelligence/stigmergy-interface.js`.

**Le backend ROS 2 / SLAM et la passerelle WebSocket ne sont pas publiés non
plus.** Ils servent à piloter de vrais drones ou une simulation Gazebo, et
restent dans le dépôt privé. Ça ne vous gêne pas pour autant : le frontend
fonctionne seul, c'est même son mode normal.

**À propos du panneau « Intelligence LLM ».** Il est prévu pour dialoguer avec un
serveur [Ollama](https://ollama.com) qui tourne sur votre machine. Un dépôt Git
ne peut évidemment pas en fournir un. Sans lui, le panneau affiche des décisions
**simulées**, uniquement pour la démonstration — ce ne sont pas des sorties de
modèle.

---

## Comment c'est organisé

```
diamants-collab/
  DIAMANTS_FRONTEND/Mission_system/     ← toute l'application est ici
    main.js                             point d'entrée, boucle de rendu
    index.html                          interface et panneaux de contrôle
    physics/                            moteur de vol PID, profils de drones
    intelligence/                       interfaces d'essaim (et les coquilles)
    environment/                        terrain, végétation, ciel
    shaders/                            herbe et ciel en GLSL
    drones/                             modèles 3D et visuels
    ui/                                 panneaux, minimaps, superpositions
    core/                               état de l'application, événements
    assets/                             maillages et textures
    third-party/ez-tree/                générateur d'arbres (licence propre)
  DEMO/                                 vidéos et données d'exemple
```

---

## Ajouter votre drone

Déposez un fichier JSON dans `physics/profiles/`. Le moteur charge tout ce qu'il
y trouve au démarrage.

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

Seuls `id`, `label`, `physical`, `performance` et `pid` sont obligatoires ; le
reste a des valeurs par défaut. Le schéma complet est dans
`profiles/drone-profile.schema.json`.

---

## Brancher votre algorithme d'essaim

Vous étendez une classe, vous implémentez deux méthodes, c'est tout.

```javascript
import { SwarmIntelligenceInterface } from './swarm-intelligence-interface.js';

export class MySwarmAlgorithm extends SwarmIntelligenceInterface {
    constructor() {
        super();
        this.name = 'my-algorithm';
        this.version = '1.0.0';
    }

    initialize(config) {
        // Appelé une fois au démarrage.
        // config contient { droneCount, arena, profiles }
    }

    computeInfluences(droneStates, dt) {
        // Appelé à chaque image.
        // droneStates : Map<id, {position, velocity, target, ...}>
        // Vous renvoyez : Map<id, {targetModifier, velocityBias, priorityOverride}>
        // Une Map vide = aucune influence, les drones continuent comme avant.
        return new Map();
    }
}
```

Le principe est volontairement simple : **vous observez et vous suggérez, le
contrôleur PID décide.** Vos sorties sont fusionnées avec la commande de vol, pas
substituées à elle. Vous ne pouvez donc pas faire s'écraser un drone par erreur,
et vous n'avez pas à vous occuper de la stabilité.

---

## Sous le capot

Three.js 0.167 pour le rendu, Vite 4.5 pour le build et le serveur de
développement, Vitest pour les tests, et des modules ES sans transpilation. Node
20 ou plus récent.

---

## Voir avant d'installer

- [Démo frontend 3D](https://www.youtube.com/watch?v=fyEmYu4lbzo)
- [Systèmes multi-agents](https://www.youtube.com/watch?v=1Av_o-9fzrE)
- [Navigation par gradient](https://www.youtube.com/watch?v=ElABxOde6ak)
- [Coordination d'essaim](https://www.youtube.com/watch?v=L8V64LajM2w)
- [Démo stigmergie](https://www.youtube.com/watch?v=SyqeRwcbDO4)

D'autres vidéos et des données d'exemple sont dans `DEMO/`.

---

## Contribuer

Les contributions sont les bienvenues, dans le cadre non commercial de la
licence. Ce qui aide le plus :

- un profil de drone (et son modèle 3D si vous en avez un)
- une implémentation d'algorithme d'essaim
- des améliorations du rendu
- des corrections de bugs, des tests, de la documentation

Fork, branche, pull request. Les détails sont dans
[Contributing.md](Contributing.md). En proposant une contribution, vous acceptez
qu'elle soit distribuée sous la même licence que le projet.

---

## Licence

**PolyForm Noncommercial License 1.0.0** — [LICENSE](LICENSE).

En clair : faites-en ce que vous voulez tant que vous n'en tirez pas de revenus.
Recherche, enseignement, apprentissage, association : oui. Produit commercial,
service payant, intégration dans une offre : non.

Les composants tiers embarqués (EZ-Tree, Crazyswarm2, paquets npm) gardent leur
propre licence ; ils sont listés à la fin du fichier LICENSE.

---

## Contact

Une question, un bug, une idée :
[ouvrez une issue](https://github.com/lololem/diamants-collab/issues).
