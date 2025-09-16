#!/bin/bash

# Script direct sans Console Ninja - DIAMANTS
echo "⚡ Démarrage direct Vite..."

cd /home/loic/Projects/AI_PROJECTS/DIAMANTS/DIAMANTS_FRONTEND/Mission_system

# Node.js v16.15.0 direct
if [[ "$(node --version)" != "v16.15.0" ]]; then
    export NVM_DIR="$HOME/.nvm"
    [ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"
    nvm use 16.15.0 > /dev/null
fi

# Démarrer Vite directement
echo "🎯 npm run dev..."
npm run dev
