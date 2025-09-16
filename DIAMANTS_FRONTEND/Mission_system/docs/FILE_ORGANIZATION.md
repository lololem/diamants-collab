# 📁 **ORGANISATION DES FICHIERS - DIAMANTS Mission_system**

## 🏗️ **STRUCTURE RECLASSÉE**

```
Mission_system/
├── 📄 index.html              # Point d'entrée web principal
├── 📄 main.js                 # Point d'entrée JavaScript principal  
├── 📄 package.json            # Configuration npm et dépendances
├── 📄 vite.config.js          # Configuration build Vite
│
├── 📂 docs/                   # 📚 Documentation
│   └── CONTROL_PANEL_BUTTONS_GUIDE.md
│
├── 📂 data/                   # 📊 Données et rapports
│   └── code-usage-analysis.json
│
├── 📂 scripts/                # ⚙️ Scripts et utilitaires
│   ├── dev.js                 # Script de développement principal
│   ├── debug-server.js        # Serveur de debug
│   ├── debug-test.js          # Tests de debug
│   ├── diagnostic-repair.js   # Auto-réparation système
│   ├── analyze-code-usage.js  # Analyse du code
│   ├── start-console-ninja.sh # Console Ninja launcher
│   ├── quick-start_vite.sh    # Démarrage rapide
│   ├── quick-start-niinja-vite.sh
│   ├── test-realistic-flight.sh
│   └── (autres scripts .sh)
│
├── 📂 tools/                  # 🔧 Outils et utilitaires système
│   ├── three-bootstrap.js     # Bootstrap THREE.js
│   ├── diamants-initializer.js # Gestionnaire d'initialisation
│   ├── drone-panel-controller.js # Contrôleur panneau drones
│   ├── simple-drone-animation.js # Animation simple drones
│   ├── authentic-drone-swarm.js # Gestionnaire essaim
│   ├── integrated-controller.js # Contrôleur intégré principal
│   └── (autres outils)
│
├── 📂 tests/                  # 🧪 Tests et diagnostics
│   ├── webgl-test.html        # Test WebGL
│   ├── debug.html
│   ├── diagnostic.html
│   └── (autres tests)
│
├── 📂 core/                   # 🧠 Moteur central
├── 📂 drones/                 # 🚁 Système de drones
├── 📂 environment/           # 🌍 Environnement 3D
├── 📂 intelligence/          # 🤖 Intelligence collective
├── 📂 controllers/           # 🎮 Contrôleurs
├── 📂 physics/               # ⚡ Physique
├── 📂 net/                   # 🔗 Communication ROS2
├── 📂 shaders/               # 🎨 Shaders GLSL
├── 📂 ui/                    # 🖼️ Interface utilisateur
├── 📂 missions/              # 🎯 Gestion missions
├── 📂 behaviors/             # 🧬 Comportements
├── 📂 visual/                # 👁️ Améliorations visuelles
├── 📂 sample/                # 📋 Exemples
├── 📂 dev-tools/             # 🛠️ Outils développement
├── 📂 third-party/           # 📦 Bibliothèques tierces
├── 📂 assets/                # 🎭 Ressources
├── 📂 ros2_bridge/           # 🌉 Bridge ROS2
└── 📂 archives/              # 🗃️ Archives et backups
```

## 🔄 **CHANGEMENTS EFFECTUÉS**

### **✅ Fichiers déplacés**
- `CONTROL_PANEL_BUTTONS_GUIDE.md` → `docs/`
- `code-usage-analysis.json` → `data/`
- Tous les scripts `*.sh` → `scripts/`
- Scripts de debug → `scripts/`
- Outils système → `tools/`
- `webgl-test.html` → `tests/`

### **✅ Références mises à jour**
- `index.html` : Chemins des scripts corrigés
- `package.json` : Scripts npm mis à jour
- `authentic-drone-swarm.js` : Import relatif corrigé

### **✅ À la racine** (fichiers essentiels uniquement)
- `index.html` - Point d'entrée web
- `main.js` - Point d'entrée JavaScript  
- `package.json` - Configuration npm
- `vite.config.js` - Configuration build

## 🎯 **AVANTAGES DE CETTE ORGANISATION**

1. **🧹 Racine propre** : Seulement 4 fichiers essentiels
2. **📂 Séparation claire** : Chaque type de fichier dans son dossier
3. **🔍 Facilité de navigation** : Structure logique et intuitive
4. **🔧 Maintenance simplifiée** : Scripts groupés, documentation centralisée
5. **⚡ Performance** : Pas d'impact sur le build ou l'exécution

## 🚀 **UTILISATION**

Tous les scripts de démarrage restent fonctionnels :
```bash
npm run dev                    # Développement standard
npm run debug                  # Mode debug
npm run console-ninja          # Debug avancé
./scripts/quick-start_vite.sh  # Démarrage rapide
```

## 📋 **NOTES**

- Aucune régression fonctionnelle
- Tous les imports et liens mis à jour
- Structure compatible avec les outils existants
- Archives conservées pour historique
