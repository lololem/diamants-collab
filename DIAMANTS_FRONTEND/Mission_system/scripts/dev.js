#!/usr/bin/env node

/**
 * DIAMANTS Mission System - Development Server
 * Lancement du serveur de développement Vite avec configuration optimisée
 */

import { createServer } from 'vite'
import { resolve } from 'path'

async function startDevServer() {
  try {
    console.log('🚁 DIAMANTS - Mission System Development Server');
    console.log('================================================');
    
    const server = await createServer({
      // Configuration Vite pour DIAMANTS
      root: process.cwd(),
      server: {
        port: 5550,
        host: 'localhost',
        open: false,
        cors: true,
        watch: {
          followSymlinks: false,
          ignored: [
            '**/node_modules/**', '**/dist/**', '**/.cache/**',
            '**/third-party/**', '**/assets/**', '**/public/**',
            '**/DIAMANTS_BACKEND/**', '**/log/**', '**/.git/**'
          ]
        }
      },
      resolve: {
        alias: {
          '@': resolve(process.cwd(), './'),
          '@assets': resolve(process.cwd(), './assets'),
          '@core': resolve(process.cwd(), './core'),
          '@behaviors': resolve(process.cwd(), './behaviors'),
          '@controllers': resolve(process.cwd(), './controllers'),
          '@ui': resolve(process.cwd(), './ui'),
          '@visual': resolve(process.cwd(), './visual'),
          '@physics': resolve(process.cwd(), './physics'),
          '@intelligence': resolve(process.cwd(), './intelligence'),
          '@missions': resolve(process.cwd(), './missions'),
          '@environment': resolve(process.cwd(), './environment'),
          '@drones': resolve(process.cwd(), './drones'),
          '@net': resolve(process.cwd(), './net'),
          '@ros2_bridge': resolve(process.cwd(), './ros2_bridge')
        }
      },
      optimizeDeps: {
        include: ['three']
      },
      build: {
        target: 'es2020',
        rollupOptions: {
          input: {
            main: resolve(process.cwd(), 'index.html')
          }
        }
      }
    });

    await server.listen();
    server.printUrls();
    
    console.log('\n✅ Serveur de développement démarré !');
    console.log('🌐 Interface 3D: http://localhost:5550');
    console.log('🚁 Système multi-drones prêt pour la simulation');
    
  } catch (error) {
    console.error('❌ Erreur lors du démarrage du serveur:', error);
    process.exit(1);
  }
}

// Démarrage du serveur
startDevServer();
