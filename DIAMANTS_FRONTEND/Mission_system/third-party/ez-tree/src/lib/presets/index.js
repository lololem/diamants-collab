// Temporary minimal imports to fix JSON parsing errors
import TreeOptions from '../options';

export const TreePreset = {
  // Presets temporarily disabled until JSON parsing issue is resolved
};

/**
 * @param {string} name The name of the preset to load
 * @returns {TreeOptions}
 */
export function loadPreset(name) {
  const preset = TreePreset[name];
  return preset ? structuredClone(preset) : new TreeOptions();
}