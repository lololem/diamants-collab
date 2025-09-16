#!/bin/bash

# Script de démarrage ultra-rapide Console Ninja - DIAMANTS
# Charge automatiquement la bonne version de Node.js

set -e  # Sortir en cas d'erreur

echo "⚡ DIAMANTS - Console Ninja Fast Start"
echo "======================================"

# Aller dans le bon répertoire
cd /home/loic/Projects/AI_PROJECTS/DIAMANTS/DIAMANTS_FRONTEND/Mission_system

# Charger NVM si nécessaire
if [[ -s ~/.nvm/nvm.sh ]]; then
    source ~/.nvm/nvm.sh
fi

# Vérification et basculement automatique
NODE_VERSION=$(node --version 2>/dev/null || echo "none")
if [[ "$NODE_VERSION" != "v16.15.0" ]]; then
    echo "🔄 Basculement: $NODE_VERSION → v16.15.0"
    nvm use 16.15.0 > /dev/null 2>&1
fi

echo "✅ Node.js: $(node --version)"

# Nettoyer rapidement
pkill -f "vite" > /dev/null 2>&1 || true
pkill -f "node.*dev.js" > /dev/null 2>&1 || true

# Démarrage immédiat
echo "🚀 Démarrage immédiat..."
exec console-ninja npm run dev
