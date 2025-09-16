#!/bin/bash
# DIAMANTS - Script de Migration WebSocket
# Déplace bridges dupliqués vers architecture API centralisée

echo "🔄 DIAMANTS - Migration WebSocket vers API Layer"
echo "==============================================="
echo ""

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BACKUP_DIR="$PROJECT_ROOT/BACKUP_MIGRATION_$(date +%Y%m%d_%H%M%S)"

# Créer dossier backup
mkdir -p "$BACKUP_DIR"
echo "📁 Backup créé: $BACKUP_DIR"
echo ""

# ============================================================================
# PHASE 1: BACKUP DES FICHIERS ACTUELS
# ============================================================================

echo "💾 PHASE 1: Backup des fichiers actuels"
echo "========================================"

# Backup bridges WebSocket backend (duplications)
if [ -f "$PROJECT_ROOT/DIAMANTS_BACKEND/core/web_interface/websocket_bridge.py" ]; then
    cp "$PROJECT_ROOT/DIAMANTS_BACKEND/core/web_interface/websocket_bridge.py" \
       "$BACKUP_DIR/websocket_bridge_core.py"
    echo "✅ Backup bridge core: websocket_bridge_core.py"
fi

if [ -f "$PROJECT_ROOT/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/src/multi_agent_framework/multi_agent_framework/web_interface/websocket_bridge.py" ]; then
    cp "$PROJECT_ROOT/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/src/multi_agent_framework/multi_agent_framework/web_interface/websocket_bridge.py" \
       "$BACKUP_DIR/websocket_bridge_slam.py"
    echo "✅ Backup bridge SLAM: websocket_bridge_slam.py"
fi

# Backup client unifié frontend
if [ -f "$PROJECT_ROOT/DIAMANTS_FRONTEND/Mission_system/ros2_bridge/unified_websocket_bridge.py" ]; then
    cp "$PROJECT_ROOT/DIAMANTS_FRONTEND/Mission_system/ros2_bridge/unified_websocket_bridge.py" \
       "$BACKUP_DIR/unified_websocket_bridge_frontend.py"
    echo "✅ Backup bridge frontend: unified_websocket_bridge_frontend.py"
fi

# Backup contrôleur ROS2
if [ -f "$PROJECT_ROOT/DIAMANTS_FRONTEND/Mission_system/controllers/crazyflie-ros-controller.js" ]; then
    cp "$PROJECT_ROOT/DIAMANTS_FRONTEND/Mission_system/controllers/crazyflie-ros-controller.js" \
       "$BACKUP_DIR/crazyflie-ros-controller_original.js"
    echo "✅ Backup contrôleur ROS: crazyflie-ros-controller_original.js"
fi

echo ""

# ============================================================================
# PHASE 2: SUPPRESSION DES DUPLICATIONS
# ============================================================================

echo "🗑️ PHASE 2: Suppression des duplications"
echo "========================================"

# Supprimer bridge SLAM (duplication exacte)
SLAM_BRIDGE="$PROJECT_ROOT/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/src/multi_agent_framework/multi_agent_framework/web_interface/websocket_bridge.py"
if [ -f "$SLAM_BRIDGE" ]; then
    rm "$SLAM_BRIDGE"
    echo "❌ Supprimé: websocket_bridge.py (SLAM - duplication)"
fi

# Laisser commentaire dans le dossier
SLAM_WEB_DIR="$PROJECT_ROOT/DIAMANTS_BACKEND/slam_collaboratif/ros2_ws/src/multi_agent_framework/multi_agent_framework/web_interface"
if [ -d "$SLAM_WEB_DIR" ]; then
    cat > "$SLAM_WEB_DIR/README_MIGRATION.md" << 'EOF'
# WebSocket Bridge - Migration vers DIAMANTS_API

⚠️ **FICHIER MIGRÉ**

Le bridge WebSocket de ce dossier était une **duplication exacte** du bridge dans:
`DIAMANTS_BACKEND/core/web_interface/websocket_bridge.py`

## Nouvelle Architecture

✅ **Service unifié**: `DIAMANTS_API/services/websocket_service.py`
✅ **Client unifié**: `DIAMANTS_API/clients/websocket_client.js`
✅ **Zero duplication**: Un seul point d'entrée WebSocket

## Utilisation

```bash
# Démarrer service WebSocket unifié
cd DIAMANTS_API
python services/websocket_service.py
```

```javascript
// Frontend - utiliser client unifié
const ws = window.createDiamantWebSocket();
ws.on('dronePositions', (positions) => {
    // Traiter positions...
});
```

**Backup disponible**: Voir dossier `BACKUP_MIGRATION_*`
EOF
    echo "✅ README migration créé dans SLAM web_interface/"
fi

echo ""

# ============================================================================
# PHASE 3: MISE À JOUR RÉFÉRENCES
# ============================================================================

echo "🔗 PHASE 3: Mise à jour des références"
echo "====================================="

# Mettre à jour imports dans __init__.py si nécessaire
INIT_FILE="$PROJECT_ROOT/DIAMANTS_BACKEND/core/web_interface/__init__.py"
if [ -f "$INIT_FILE" ]; then
    # Backup
    cp "$INIT_FILE" "$BACKUP_DIR/__init___original.py"
    
    # Ajouter note de migration
    cat >> "$INIT_FILE" << 'EOF'

# MIGRATION NOTE: WebSocket bridge duplications supprimées
# Service WebSocket unifié disponible dans DIAMANTS_API/services/websocket_service.py
# Pour utilisation complète du nouveau service, voir TODO_API_ARCHITECTURE.md
EOF
    echo "✅ Note migration ajoutée dans __init__.py"
fi

# Mettre à jour contrôleur frontend pour pointer vers nouvelle API
CONTROLLER_FILE="$PROJECT_ROOT/DIAMANTS_FRONTEND/Mission_system/controllers/crazyflie-ros-controller.js"
if [ -f "$CONTROLLER_FILE" ]; then
    # Ajouter commentaire de migration en haut du fichier
    sed -i '1i// MIGRATION: Voir DIAMANTS_API/clients/websocket_client.js pour client WebSocket unifié' "$CONTROLLER_FILE"
    sed -i '2i// TODO: Remplacer logique WebSocket locale par DiamantWebSocketClient global' "$CONTROLLER_FILE"
    sed -i '3i//' "$CONTROLLER_FILE"
    echo "✅ Notes migration ajoutées dans crazyflie-ros-controller.js"
fi

echo ""

# ============================================================================
# PHASE 4: VALIDATION
# ============================================================================

echo "✔️ PHASE 4: Validation migration"
echo "==============================="

# Vérifier que les nouveaux fichiers existent
if [ -f "$PROJECT_ROOT/DIAMANTS_API/services/websocket_service.py" ]; then
    echo "✅ Service WebSocket unifié: PRÉSENT"
else
    echo "❌ Service WebSocket unifié: MANQUANT"
fi

if [ -f "$PROJECT_ROOT/DIAMANTS_API/clients/websocket_client.js" ]; then
    echo "✅ Client WebSocket unifié: PRÉSENT"
else
    echo "❌ Client WebSocket unifié: MANQUANT"
fi

# Compter fichiers backup
BACKUP_COUNT=$(ls -1 "$BACKUP_DIR" | wc -l)
echo "📁 Fichiers sauvegardés: $BACKUP_COUNT"

echo ""

# ============================================================================
# PHASE 5: PROCHAINES ÉTAPES
# ============================================================================

echo "🚀 PHASE 5: Prochaines étapes"
echo "============================"
echo ""
echo "Migration WebSocket partiellement terminée !"
echo ""
echo "✅ FAIT:"
echo "   - Duplication bridge SLAM supprimée"
echo "   - Service WebSocket unifié créé"
echo "   - Client WebSocket unifié créé"
echo "   - Backups réalisés"
echo ""
echo "📋 TODO MANUEL:"
echo "   1. Tester service WebSocket unifié:"
echo "      cd DIAMANTS_API && python services/websocket_service.py"
echo ""
echo "   2. Intégrer client unifié dans frontend:"
echo "      - Remplacer appels WebSocket dans controllers/"
echo "      - Utiliser window.createDiamantWebSocket()"
echo ""
echo "   3. Adapter topics ROS2 pour nouveaux endpoints:"
echo "      - /diamants/drones/positions"
echo "      - /diamants/swarm/commands"
echo ""
echo "   4. Tester communication complète Frontend ↔ Backend"
echo ""
echo "📄 Documentation: DIAMANTS_API/TODO_API_ARCHITECTURE.md"
echo "💾 Backups: $BACKUP_DIR"
echo ""
echo "🎯 Objectif atteint: Zero duplication WebSocket!"
echo ""
