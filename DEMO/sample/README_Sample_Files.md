# 🎮 DIAMANTS Sample Files Documentation

## 📁 HTML Demo Files

Cette documentation décrit le contenu des fichiers de démonstration HTML du projet DIAMANTS dans le répertoire `/DEMO/sample/`.

---

### 🚁 **Simulation & Search and Rescue**

#### `DIAMANTS_Crazyflie_Search_Rescue_Gazebo_Simulation.html` (378 KB)
**Fonctionnalité :** Simulation complète de drones Crazyflie avec ROS2 Gazebo pour missions de recherche et sauvetage

**🎯 Caractéristiques Principales :**
- ✅ **Simulation Gazebo Réaliste** - Mesh détaillés Crazyflie avec physique réaliste
- ✅ **Interface ROS2-like** - Panels de contrôle style ROS2 avec topics monitor
- ✅ **Mission Search & Rescue** - Algorithmes de recherche collaborative et sauvetage
- ✅ **Intelligence Collective** - Système DIAMANTS avec stigmergie et consensus
- ✅ **Anti-Collision Avancé** - Évitement d'obstacles et gestion de formation
- ✅ **Minimap Tactique** - Vue d'ensemble avec contrôles zoom et visualisation stigmergie
- ✅ **Métriques Temps Réel** - Monitoring intelligence, progression mission, coordination

**🔧 Technologies Utilisées :**
- Three.js + WebGL pour rendu 3D haute performance
- ROS2-like communication system avec WebSockets
- Algorithmes DIAMANTS (cohésion, exploration, émergence)
- Physique de vol réaliste avec compensation gravitationnelle
- Interface utilisateur style Gazebo/RViz

**🎮 Fonctionnalités Interactives :**
- Contrôles de mission (Start/Stop, Emergency Land, Reset)
- Ajout/suppression dynamique de drones
- Sélection de patterns de recherche (grid, spiral, boustrophedon)
- Contrôles caméra (top view, follow mode, zoom swarm)
- Configuration formation et paramètres de sécurité

---

### 🌟 **Frontend Complet & Intelligence d'Essaim**

#### `DIAMANTS_Complete_Frontend_Swarm_Intelligence_Demo.html` (265 KB)
**Fonctionnalité :** Démonstration frontend complète avec effet Wahoo et intelligence collective avancée

**🧠 Caractéristiques Principales :**
- ✅ **Effet Wahoo** - Visualisation spectaculaire des interactions d'essaim
- ✅ **Intelligence Collective** - Calculs DIAMANTS φ+σ avec gradients
- ✅ **Métriques Avancées** - Monitoring en temps réel de l'émergence
- ✅ **Auras Énergétiques** - Visualisation des champs de force entre drones
- ✅ **Formules Mathématiques** - Affichage des équations DIAMANTS en temps réel
- ✅ **Interface Immersive** - Design futuriste avec effets visuels avancés

**🎨 Effets Visuels :**
- Particules énergétiques entre drones
- Gradients de couleur représentant les potentiels
- Trails de mouvement avec effet de persistance
- Éclairage dynamique selon l'activité d'essaim
- Interface transparente avec blur effects

**📊 Métriques Visualisées :**
- **φ (Phi)** - Potentiel attractif DIAMANTS (cohésion)
- **σ (Sigma)** - Potentiel répulsif DIAMANTS (exploration)
- **∇(φ+σ)** - Gradient DIAMANTS pour navigation
- **Intelligence Collective** - Indice I(t) global d'émergence
- **Coordination** - Niveaux de synchronisation d'essaim

**🎯 Cas d'Usage :**
- Démonstrations spectaculaires pour présentations
- Recherche sur l'émergence d'intelligence collective
- Validation visuelle des algorithmes DIAMANTS
- Formation et éducation sur les systèmes multi-agents

---

## 🔄 **Comparaison des Fichiers**

| Aspect | Search & Rescue Simulation | Complete Frontend Demo |
|--------|---------------------------|------------------------|
| **Focus** | Mission opérationnelle | Recherche & visualisation |
| **Interface** | Style ROS2/Gazebo | Design futuriste immersif |
| **Complexité** | Production-ready | Expérimental/démo |
| **Physique** | Réaliste détaillée | Stylisée pour effet |
| **Interaction** | Contrôles complets | Observation passive |
| **Taille** | 378 KB | 265 KB |

---

## 🚀 **Guide d'Utilisation**

### Pour Développeurs :
1. **Commencer par** `Search_Rescue_Gazebo_Simulation.html` pour comprendre l'architecture
2. **Analyser** le code DIAMANTS pour l'intelligence collective
3. **Adapter** les algorithmes selon vos besoins spécifiques

### Pour Présentations :
1. **Utiliser** `Complete_Frontend_Swarm_Intelligence_Demo.html` pour l'impact visuel
2. **Expliquer** les métriques DIAMANTS en temps réel
3. **Démontrer** l'émergence d'intelligence collective

### Pour Recherche :
1. **Étudier** les deux fichiers pour comprendre les différentes approches
2. **Modifier** les paramètres DIAMANTS dans le code
3. **Observer** les comportements émergents résultants

---

## 📋 **Notes Techniques**

### Dépendances :
- Three.js (CDN) pour rendu 3D
- Navigateur moderne avec support WebGL
- Résolution minimale 1280x720 recommandée

### Performance :
- Optimisé pour 20-50 drones simultanés
- Rendu 60 FPS avec WebGL
- Calculs DIAMANTS en temps réel

### Compatibilité :
- Chrome/Chromium (recommandé)
- Firefox avec WebGL activé
- Safari (support partiel)

---

**Last Updated :** September 17, 2025  
**Project :** DIAMANTS - Distributed Autonomous Multi-agents Systems  
**Maintainer :** lololem  
**Repository :** [diamants-collab](https://github.com/lololem/diamants-collab)
