# 📁 DIAMANTS Mission_system - Structure du Projet

## 🎯 **RACINE DU PROJET** (Fichiers essentiels uniquement)

```
Mission_system/
├── 📄 index.html         # Point d'entrée web principal
├── ⚡ main.js             # Point d'entrée JavaScript principal  
├── 📦 package.json       # Configuration npm et scripts
├── 🔒 package-lock.json  # Dépendances verrouillées
└── ⚙️ vite.config.js     # Configuration Vite
```

## 📂 **ORGANISATION MODULAIRE**

### 🛠️ `/tools/` - Outils et utilitaires
```
tools/
├── three-bootstrap.js           # Bootstrap THREE.js avec exposition globale
├── diamants-initializer.js      # Gestionnaire d'initialisation système
├── drone-panel-controller.js    # Contrôleur interface utilisateur
├── simple-drone-animation.js    # Système d'animation simple
├── authentic-drone-swarm.js     # Gestionnaire d'essaim authentique
└── integrated-controller.js     # Contrôleur principal intégré
```

### 📝 `/scripts/` - Scripts de développement et debug
```
scripts/
├── dev.js                      # Script de développement Vite
├── debug-server.js             # Serveur de debug avec Console Ninja
├── debug-test.js               # Tests de debug temporaires
├── diagnostic-repair.js        # Auto-diagnostic et réparation
├── analyze-code-usage.js       # Analyseur d'utilisation du code
├── start-console-ninja.sh      # Démarrage Console Ninja
├── quick-start_vite.sh         # Démarrage rapide
├── quick-start-niinja-vite.sh  # Démarrage hybride
├── test-realistic-flight.sh    # Tests de vol réaliste
└── *.sh                        # Autres scripts shell
```

### 📊 `/data/` - Données et rapports
```
data/
└── code-usage-analysis.json    # Rapport d'analyse du code
```

### 📚 `/docs/` - Documentation
```
docs/
├── CONTROL_PANEL_BUTTONS_GUIDE.md  # Guide des boutons interface
└── PROJECT_STRUCTURE.md            # Ce fichier (structure projet)
```

### 🧪 `/tests/` - Tests et diagnostics
```
tests/
├── webgl-test.html             # Test et diagnostic WebGL
├── debug.html                  # Tests de debug
├── diagnostic.html             # Interface de diagnostic
└── *.test.js                   # Tests unitaires
```

## 🔄 **MODIFICATIONS APPORTÉES**

### ✅ Fichiers déplacés
- **Documentation** : `CONTROL_PANEL_BUTTONS_GUIDE.md` → `docs/`
- **Rapports** : `code-usage-analysis.json` → `data/`
- **Scripts shell** : `*.sh` → `scripts/`
- **Scripts debug** : `debug-*.js`, `diagnostic-*.js` → `scripts/`
- **Outils** : `three-bootstrap.js`, `diamants-initializer.js`, etc. → `tools/`
- **Tests** : `webgl-test.html` → `tests/`

### 🔧 Références mises à jour
- **index.html** : Tous les chemins `src` mis à jour vers les nouveaux dossiers
- **package.json** : Scripts npm mis à jour pour pointer vers `scripts/`

### 🏆 Résultat
- **Racine propre** : 5 fichiers essentiels seulement
- **Organisation logique** : Fichiers regroupés par fonction
- **Aucune régression** : Tous les liens et imports préservés
- **Maintenance facilitée** : Structure claire et modulaire

## 🚀 **UTILISATION**

Les commandes restent identiques :
```bash
npm run dev              # Développement
npm run debug            # Debug avec Console Ninja
npm run build            # Build production
```

La structure est maintenant **claire, organisée et maintenable** ! ✨
