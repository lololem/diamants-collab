/**
 * StigmergyEngine — Stub public
 * ===============================
 * L'implémentation réelle (grille de phéromones 2D, évaporation, diffusion,
 * génération de waypoints par descente de gradient, journal d'exploration
 * partagé) vit dans le dépôt privé.
 *
 * Ce fichier n'exporte VOLONTAIREMENT aucune classe `StigmergyEngine` :
 * stigmergy-loader.js teste sa présence et, ne la trouvant pas, bascule sur
 * son comportement de repli. Sa seule raison d'être est de permettre au
 * bundler de résoudre l'import dynamique — sans lui, la compilation échoue.
 *
 * Conséquence : les drones volent et explorent, mais sans intelligence
 * stigmergique. Pour brancher le moteur réel, remplacer ce fichier.
 */

// Aucun export nommé StigmergyEngine => le loader utilise son repli.
export default null;
