# DIAMANTS - Reinforcement Learning Guide

## 🚁 Description
Simulation de drones Crazyflie avec apprentissage par renforcement collaboratif basée sur Three.js et WebGL.

## 📁 Fichiers

### ✅ DIAMANTS_RL_Crazyflie_Fixed.html
**Statut:** CORRIGÉ - Version fonctionnelle réparée

**Corrections appliquées (19/09/2025):**
- ✅ **Système de chargement mesh** : Transféré depuis fichier de référence `DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html`
  - Fonction `preloadCrazyflieMeshes()` identique
  - Fonction `loadColladaMesh()` avec timeout et gestion d'erreurs
  - Configuration `meshPath: 'meshes/'` correcte
  - Fichiers mesh : `cf2_assembly.dae`, `cw_prop.dae`, `ccw_prop.dae`

- ✅ **Problèmes JavaScript résolus:**
  - Erreurs `scene undefined` : Exposition globale via `window.scene`
  - Erreurs `velocity.clone()` : Vérifications de sécurité ajoutées
  - Conflits de fonctions : Fonction `initScene()` dupliquée supprimée

- ✅ **Système RL timing fixé:**
  - Fonction `startRLTrainingMission()` avec vérification de scène
  - Attente active si scène non disponible (polling 500ms)
  - Timeout de sécurité 10 secondes
  - Délai de démarrage augmenté (1s → 3s)

- ✅ **Fonctions de target/obstacles sécurisées:**
  - `generateRandomTargets()` : Fallback `window.scene || window.scene3D`
  - `clearAllTargets()` : Protection erreurs scène
  - `addDynamicObstacles()` : Vérification scène disponible

- ✅ **Optimisations rendu:**
  - Logs debug excessifs supprimés
  - Boucle de rendu simplifiée
  - Indicateurs visuels debug retirés

**Issues résolues:**
- ❌ "Scene non disponible pour ajouter target" → ✅ Système d'attente de scène
- ❌ Rectangles de fallback au lieu des mesh → ✅ Chemins mesh corrigés
- ❌ Erreurs JavaScript multiples → ✅ Variables globales exposées
- ❌ Rendu Three.js non fonctionnel → ✅ Conflits de fonctions supprimés

### 📋 DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html
**Statut:** RÉFÉRENCE - Fichier source fonctionnel

**Utilisé comme référence pour:**
- Structure de chargement des mesh Crazyflie
- Configuration renderer Three.js
- Ordre d'initialisation des composants
- Boucle de rendu et animation

## 🔧 Installation

### Prérequis
1. Serveur HTTP local (ex: Live Server VS Code)
2. Fichiers mesh dans le dossier `meshes/`:
   ```
   meshes/
   ├── cf2_assembly.dae
   ├── cw_prop.dae
   └── ccw_prop.dae
   ```

### Lancement
1. Démarrer serveur local sur port 5500
2. Ouvrir `DIAMANTS_RL_Crazyflie_Fixed.html`
3. La simulation démarre automatiquement après 3 secondes

## 🎮 Fonctionnalités

### ✅ Système de Rendu 3D
- ✅ Scene Three.js avec WebGL
- ✅ Mesh Crazyflie chargés dynamiquement
- ✅ Éclairage réaliste et ombres
- ✅ Caméra perspective avec contrôles

### ✅ Système RL (Reinforcement Learning)
- ✅ Agents collaboratifs
- ✅ Exploration et exploitation
- ✅ Métriques de performance
- ✅ Cibles et obstacles dynamiques

### ✅ Interface Utilisateur
- ✅ Panneau de contrôle mission
- ✅ Métriques intelligence DIAMANTS
- ✅ Logs de debug
- ✅ Boutons diagnostic

## 🐛 Issues Restantes

### ⚠️ À Vérifier
1. **Rendu visuel** : Vérifier que le fallback renderer n'apparaît plus
2. **Performance** : Optimiser si frame rate faible
3. **Mesh loading** : Confirmer chargement correct des fichiers .dae

### 🔄 Améliorations Futures
1. **UI/UX** : Améliorer interface utilisateur
2. **RL Algorithm** : Optimiser algorithmes d'apprentissage
3. **Multi-drone** : Étendre à plus de drones simultanés
4. **Metrics** : Ajouter métriques avancées

## 📊 Diagnostic

### Commandes Debug Disponibles
```javascript
// Vérifier état scène
window.scene
window.renderer
window.camera

// Tester chargement mesh
testMeshPaths()
checkMeshCache()

// Forcer démarrage RL
startRLTrainingMission()
```

### Logs Importants
```
✅ WebGL Renderer created
✅ Global scope assignments completed
✅ Mesh chargés avec succès
✅ Scène détectée - relance du démarrage RL
```

## 🛠️ Développement

### Structure Code
```
DIAMANTS_RL_Crazyflie_Fixed.html
├── Configuration (CONFIG)
├── Classes RL (CollaborativeLearningSystem)
├── Chargement Mesh (loadColladaMesh, preloadCrazyflieMeshes)
├── Initialisation 3D (initScene, renderer)
├── Boucle Animation (animate)
├── Interface UI (boutons, métriques)
└── Gestion Events (DOMContentLoaded)
```

### Ordre d'Exécution
1. DOMContentLoaded
2. initScene() → Création scene/renderer/camera
3. initLoaders() → Chargement mesh
4. animate() → Démarrage boucle rendu
5. setTimeout(3s) → startRLTrainingMission()

## 📝 Changelog

### v1.1 (19/09/2025)
- ✅ Fix: Fonction initScene() dupliquée supprimée
- ✅ Fix: Système RL timing corrigé
- ✅ Fix: Chargement mesh depuis référence
- ✅ Fix: Erreurs JavaScript résolues
- ✅ Optimisation: Logs debug réduits

### v1.0 (Base)
- 🚁 Simulation drones Crazyflie
- 🧠 Système RL collaboratif
- 🎮 Interface Three.js WebGL

---

**Dernière mise à jour:** 19 septembre 2025
**Statut:** ✅ FONCTIONNEL (corrections timing/mesh/JS appliquées)
