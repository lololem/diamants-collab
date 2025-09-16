# 🎛️ DIAMANTS V3 - Guide Complet des Boutons Interface

## **📁 LOCALISATION**
Le panneau de contrôle est intégré dans `index.html` (lignes 540-800+)

---

## **🎮 BOUTON PRINCIPAL**

### **► (Toggle Panel)**
- **Fonction** : `togglePanel()`
- **Position** : Flottant à gauche de l'écran
- **Utilité** : Afficher/masquer tout le panneau de contrôle ROS2

---

## **🚁 DIAMANTS V3 STATUS PANEL**

### **Affichage Métrique** (pas de boutons)
- **Status** : État du système (Active/Inactive)
- **Drones** : Nombre de drones connectés
- **Intel** : Intelligence totale I(t)
- **Emerge** : Niveau d'émergence

---

## **🚁 MISSION CONTROL PANEL**

### **Launch Mission**
- **Fonction** : `launchMission()`
- **Utilité** : Démarrer une mission automatique selon le type sélectionné

### **Emergency Land**
- **Fonction** : `emergencyLand()`
- **Utilité** : Atterrissage d'urgence de tous les drones

### **Reset Swarm**
- **Fonction** : `resetSwarm()`
- **Utilité** : Réinitialiser complètement l'essaim de drones

### **🌟 Pattern**
- **Fonction** : `changePattern()`
- **Utilité** : Changer le motif de formation de l'essaim

### **📋 Show Logs**
- **Fonction** : `toggleDebugLogs()`
- **ID** : `btn-toggle-logs`
- **Utilité** : Afficher/masquer les logs de debug

### **🧹 Clear Logs**
- **Fonction** : `clearDebugLogs()`
- **Utilité** : Vider les logs de debug

### **🛠️ Show Debug**
- **Fonction** : `toggleDebugPanels()`
- **ID** : `btn-toggle-debug`
- **Utilité** : Afficher/masquer les panneaux de debug

### **🥷 Console**
- **Fonction** : `toggleConsoleNinja()`
- **ID** : `btn-toggle-ninja`
- **Utilité** : Activer/désactiver Console Ninja

### **🖼️ PLEIN ÉCRAN**
- **Fonction** : `togglePanel()`
- **Utilité** : Mode plein écran (masque l'interface)

---

## **📹 CAMERA CONTROLS**

### **Reset View**
- **Fonction** : `resetCamera()`
- **Utilité** : Remettre la caméra en position par défaut

### **Top View**
- **Fonction** : `topView()`
- **Utilité** : Vue du dessus de la scène

### **Follow Mode**
- **Fonction** : `toggleFollowMode()`
- **Utilité** : Mode suivi d'un drone sélectionné

### **View All**
- **Fonction** : `zoomToSwarm()`
- **Utilité** : Zoomer pour voir tout l'essaim

---

## **🔢 FORMATION CONTROL**

### **Line**
- **Fonction** : `setFormation('line')`
- **Utilité** : Formation en ligne droite

### **Circle**
- **Fonction** : `setFormation('circle')`
- **Utilité** : Formation en cercle

### **Triangle**
- **Fonction** : `setFormation('triangle')`
- **Utilité** : Formation triangulaire

### **Grid**
- **Fonction** : `setFormation('grid')`
- **Utilité** : Formation en grille (défaut)

---

## **🚁 FLIGHT CONTROL** ⭐ (PANEL PRINCIPAL)

### **🛫 Takeoff All** ⚠️ PROBLÉMATIQUE
- **Fonction** : `takeoffAllDrones()`
- **Ligne** : 723
- **Utilité** : Décollage de tous les drones
- **⚠️ BUGS** : C'est ce bouton qui pose problème selon votre demande initiale

### **🎯 Start Mission**
- **Fonction** : `startMissionManual()`
- **Ligne** : 724
- **Utilité** : Démarrer une mission manuelle

### **🛬 Land All**
- **Fonction** : `landAllDrones()`
- **Ligne** : 725
- **Utilité** : Atterrissage de tous les drones

### **⏹️ Stop All**
- **Fonction** : `stopAllDrones()`
- **Ligne** : 726
- **Utilité** : Arrêt d'urgence de tous les drones

### **🎯 Test Explorer**
- **Fonction** : `testSingleExploration()`
- **Ligne** : 727
- **Utilité** : Test d'exploration avec un seul drone

---

## **🌟 DRONE MODES**

### **Apply Mode**
- **Fonction** : `applyMode()`
- **Utilité** : Appliquer le mode sélectionné dans le dropdown

**Modes disponibles :**
- Grid (Grille)
- Boustrophedon (Balayage)
- Spiral (Spirale)
- Coverage (Couverture)
- Follow Leader (Suiveur)
- Random (Aléatoire)

---

## **🔄 BOUTON WEBGL (ERREUR)**

### **🔄 Réessayer**
- **Fonction** : `location.reload()`
- **Ligne** : 44
- **Utilité** : Recharger la page en cas d'erreur WebGL

---

## **📊 CONTRÔLES NON-BOUTONS**

### **Sliders et Inputs**
- **Flight Altitude** : Curseur 0.5-5.0m
- **Safety Distance** : Distance de sécurité anti-collision
- **Exploration Bounds X/Y** : Limites de la zone d'exploration

### **Dropdowns**
- **Mission Type** : Type de mission (exploration, mapping, etc.)
- **Drone Selector** : Sélection du drone à suivre
- **Mode Select** : Mode de vol à appliquer

---

## **⚠️ BOUTON PROBLÉMATIQUE IDENTIFIÉ**

Le bouton **"🛫 Takeoff All"** (ligne 723) qui appelle `takeoffAllDrones()` est celui qui génère les "nombreux bugs" mentionnés dans votre demande initiale.

**Fonction associée** : lignes 1430+ dans le JavaScript
**Action** : Décollage simultané de tous les drones connectés

---

## **🎯 RÉSUMÉ RAPIDE**

**Total des boutons actifs** : ~25 boutons
**Bouton problématique** : 🛫 Takeoff All
**Panels principaux** : 
- Mission Control (7 boutons)
- Flight Control (5 boutons) ⭐ 
- Formation Control (4 boutons)
- Camera Controls (4 boutons)
- Drone Modes (1 bouton + dropdown)

L'interface est organisée en style ROS2/Gazebo avec des panneaux spécialisés pour chaque aspect du contrôle de l'essaim de drones.
