#!/bin/bash

# Script ultra-rapide - DIAMANTS Console Ninja
echo "🚀 Démarrage ultra-rapide..."

# Aller au bon endroit
cd /home/loic/Projects/AI_PROJECTS/DIAMANTS/DIAMANTS_FRONTEND/Mission_system

# Vérifier rapidement Node.js
if ! command -v node &> /dev/null || [[ "$(node --version)" != "v16.15.0" ]]; then
    echo "🔄 Activation Node.js v16.15.0..."
    export NVM_DIR="$HOME/.nvm"
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    nvm use 16.15.0 > /dev/null 2>&1
fi

echo "✅ Node.js: $(node --version)"

# Nettoyer rapidement
pkill -f "vite\|node.*dev" 2>/dev/null || true

# Démarrer directement
echo "🎯 Lancement direct..."
console-ninja npm run dev
