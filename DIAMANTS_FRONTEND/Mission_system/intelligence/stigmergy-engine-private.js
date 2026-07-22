/**
 * Stigmergy Engine — Stub public
 * ================================
 * Ce fichier satisfait l'import dynamique de `stigmergy-loader.js`. Sans lui,
 * le bundler ne résout pas `./stigmergy-engine-private.js` et la compilation
 * échoue — le dépôt public devient inutilisable pour un contributeur.
 *
 * Il n'exporte volontairement AUCUN moteur : le loader constate l'absence de
 * `StigmergyEngine` et bascule sur son comportement de repli. La simulation
 * tourne et l'essaim vole ; seule l'intelligence stigmergique propriétaire
 * est absente.
 *
 * Ce fichier était auparavant un lien symbolique vers un chemin absolu du
 * dépôt privé. Outre qu'il était cassé, un lien de ce type publié sur un
 * dépôt public expose l'arborescence privée et risque, s'il est un jour
 * résolu en fichier par un outil, d'y déverser le moteur propriétaire.
 *
 * Pour brancher l'implémentation privée en local : remplacer ce fichier, ou
 * injecter la classe via `window.DIAMANTS_STIGMERGY_ENGINE`.
 */

// Aucun StigmergyEngine exporté => le loader utilise le repli.
export default null;
