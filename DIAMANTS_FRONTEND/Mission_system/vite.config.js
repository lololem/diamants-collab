import { defineConfig } from 'vite';
import path from 'path';

// Use a user-writable cache directory under HOME to avoid EACCES
const home = process.env.HOME || process.env.USERPROFILE || process.cwd();
const userCacheDir = path.resolve(home, '.cache/vite/diamants-mission-v1');

export default defineConfig({
  root: '.',
  cacheDir: userCacheDir,
  publicDir: 'public',
  resolve: {
    alias: {
  // Use the prebuilt ES module bundle to avoid JSON import issues
  '@ez-tree': path.resolve(__dirname, 'third-party/ez-tree/build/ez-tree.es.js'),
  '@dgreenheck/ez-tree': path.resolve(__dirname, 'third-party/ez-tree/build/ez-tree.es.js'),
  '@ez-tree-app': path.resolve(__dirname, 'third-party/ez-tree/src/app'),
      '@': path.resolve(__dirname, './'),
    },
  },
  server: {
  port: 5550,
    host: true,
    hmr: true,
  open: '/index.html',
  // Laisser Vite choisir un autre port si le 5550 est occupé
  strictPort: false,
  fs: {
    // Permettre l'accès aux fichiers en dehors du projet (symlinks vers diamants-private)
    allow: [
      path.resolve(__dirname, '../../../'),
      '/home/loic/diamants-private'
    ]
  }
  },
  build: {
    outDir: 'dist',
    sourcemap: true,
    rollupOptions: {
      input: {
  main: path.resolve(__dirname, 'index.html')
  // Sample files have been moved to DEMO/sample/ directory
  // and are no longer part of the Mission_system build process
      }
    }
  },
  assetsInclude: ['**/*.frag', '**/*.vert', '**/*.glb', '**/*.gltf', '**/*.jpg', '**/*.png']
});
