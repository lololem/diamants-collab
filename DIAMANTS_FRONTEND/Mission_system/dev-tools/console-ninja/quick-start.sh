#!/bin/bash

# Script de démarrage rapide Console Ninja - DIAMANTS
# Usage: ./quick-start.sh

echo "🥷 DIAMANTS - Console Ninja Quick Start"
echo "======================================="

# Aller dans le bon répertoire
cd /path/to/DIAMANTS/DIAMANTS_FRONTEND/Mission_system

# Vérification rapide de la version Node.js
CURRENT_NODE=$(node --version 2>/dev/null)
if [[ "$CURRENT_NODE" != "v16.15.0" ]]; then
    echo "⚠️  Node.js $CURRENT_NODE détecté, basculement vers v16.15.0..."
    source ~/.nvm/nvm.sh && nvm use 16.15.0 > /dev/null 2>&1
fi

# Vérifier la version finale
echo "🔧 Node.js: $(node --version)"
echo "📦 NPM: $(npm --version)"

# Nettoyer les processus existants (silencieux)
pkill -f "vite" > /dev/null 2>&1 || true
pkill -f "node.*dev.js" > /dev/null 2>&1 || true

# Lancer avec Console Ninja
echo "🚀 Démarrage du serveur..."
console-ninja npm run dev
