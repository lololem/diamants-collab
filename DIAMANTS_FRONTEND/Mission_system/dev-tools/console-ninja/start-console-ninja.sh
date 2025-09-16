#!/bin/bash

# DIAMANTS Console Ninja Wrapper Script
# Démarre Vite avec Console Ninja intégré

echo "🚀 Démarrage Console Ninja pour DIAMANTS..."

# Charge NVM si disponible
if [ -s "$HOME/.nvm/nvm.sh" ]; then
    source "$HOME/.nvm/nvm.sh"
    echo "📦 Basculement vers Node.js 16.15.0..."
    nvm use 16.15.0
    
    # Vérification que nous sommes bien sur la bonne version
    if [[ "$(node --version)" != "v16.15.0" ]]; then
        echo "❌ Erreur: Impossible de basculer vers Node.js 16.15.0"
        echo "Version actuelle: $(node --version)"
        exit 1
    fi
else
    echo "⚠️ NVM non trouvé, utilisation de Node.js système..."
    if [[ "$(node --version)" != "v16.15.0" ]]; then
        echo "❌ Console Ninja nécessite Node.js v16.15.0, version actuelle: $(node --version)"
        exit 1
    fi
fi

# Affiche la version Node.js utilisée
echo "🔧 Node.js: $(node --version)"

# Tue les processus Vite existants
echo "🧹 Nettoyage des processus Vite existants..."
pkill -f "vite" 2>/dev/null || true
pkill -f "node.*dev.js" 2>/dev/null || true

# Installe les dépendances si nécessaire
if [ ! -d "node_modules" ]; then
    echo "📦 Installation des dépendances..."
    npm install
fi

# Lance le serveur de développement avec Console Ninja
echo "🌟 Lancement de Vite avec Console Ninja..."
cd ../../  # Remonte vers Mission_system
console-ninja npm run dev
