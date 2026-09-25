#!/usr/bin/env node
/*
 * DIAMANTS — Collaborative drone swarm simulation
 * Copyright (c) 2026 Loic Lemasle
 *
 * Licensed under the PolyForm Noncommercial License 1.0.0.
 * Commercial use is not permitted. See LICENSE at the repository root.
 * https://polyformproject.org/licenses/noncommercial/1.0.0/
 */

/**
 * DIAMANTS Mission System - Development Server
 * Lancement du serveur de développement Vite avec configuration optimisée
 */

import { existsSync } from 'fs'
import { createServer } from 'vite'
import { resolve } from 'path'

async function startDevServer() {
  try {
    console.log('🚁 DIAMANTS - Mission System Development Server');
    console.log('================================================');
    
    /* A MISSING MODEL REGISTRY IS NOT AN ERROR.
     *
     * intelligence/model-providers/agent-models.json is yours to create — it
     * is git-ignored on purpose, since it can hold an API key. Until you do,
     * every page load asked for it and the browser printed a red 404. The dev
     * server now answers an empty registry instead: same behaviour, clean
     * console. Your own file, once written, is served normally. */
    const registreVide = () => ({
      name: 'diamants-registre-modeles',
      configureServer(srv) {
        srv.middlewares.use((req, res, next) => {
          if (!req.url || !req.url.startsWith('/intelligence/model-providers/agent-models.json')) return next();
          const chemin = resolve(process.cwd(), 'intelligence/model-providers/agent-models.json');
          if (existsSync(chemin)) return next();
          res.setHeader('Content-Type', 'application/json');
          res.end('{}');
        });
      },
    });

    const server = await createServer({
      // Vite configuration for DIAMANTS
      root: process.cwd(),
      plugins: [registreVide()],
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
            '**/log/**', '**/.git/**'
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
          '@net': resolve(process.cwd(), './net')
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
    
    console.log('\n✅ Development server started!');
    console.log('🌐 3D interface: http://localhost:5550');
    console.log('🚁 Multi-drone system ready for simulation');
    
  } catch (error) {
    console.error('❌ Erreur lors du démarrage du serveur:', error);
    process.exit(1);
  }
}

// Démarrage du serveur
startDevServer();
