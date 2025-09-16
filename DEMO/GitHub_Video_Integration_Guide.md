# 📹 Guide d'Intégration Vidéo Native GitHub

## ❌ Problème Actuel
Les fichiers vidéo stockés dans le repository (`./DEMO/video/Backend.mp4`) ne peuvent **pas** être lus nativement dans le README. GitHub ne supporte la lecture native que pour les vidéos uploadées directement dans son interface.

## ✅ Solution pour Intégration Native

### Étape 1: Accéder à l'éditeur GitHub
1. Aller sur https://github.com/lololem/diamants-collab
2. Cliquer sur "Edit README.md" (icône crayon)

### Étape 2: Upload des Vidéos
1. Dans l'éditeur, glisser-déposer directement vos fichiers vidéo
2. GitHub génère automatiquement des URLs comme:
   ```
   https://user-images.githubusercontent.com/12345678/video-hash.mp4
   ```

### Étape 3: Intégration Native
```html
<video width="100%" controls>
  <source src="https://user-images.githubusercontent.com/12345678/video-hash.mp4" type="video/mp4">
  Votre navigateur ne supporte pas la balise vidéo.
</video>
```

## 📁 Fichiers à Uploader
- `Backend.mp4` (25MB)
- `Frontend1.mp4` (164MB) 
- `Fontend2.mp4` (84MB)
- `Other1.mp4` à `Other6.mp4`

## 🎯 Résultat Attendu
Avec cette méthode, les vidéos se lisent **directement** dans le README sans téléchargement, avec contrôles de lecture natifs.

## 📝 Notes Importantes
- Les fichiers repository ne supportent que les liens de téléchargement
- Seules les URLs `user-images.githubusercontent.com` supportent la lecture native
- Limite : 100MB par vidéo (votre plan GitHub)
