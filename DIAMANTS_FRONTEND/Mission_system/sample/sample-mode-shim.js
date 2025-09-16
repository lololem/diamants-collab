// Optional shim: if SAMPLE_MODE is defined by a wrapper page, expose it in a safe place.
// Apps can import this file if they need to branch behavior.
export const SAMPLE_MODE = typeof window !== 'undefined' ? window.SAMPLE_MODE : undefined;
if (SAMPLE_MODE) {
  // Non-invasive log for visibility during migration.
  console.log(`[Sample] Running in SAMPLE_MODE='${SAMPLE_MODE}'`);
}