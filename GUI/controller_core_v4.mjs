// Compatibility exports for module consumers. The browser GUI loads the classic script.
import './controller_core_v4.js';
export const { proportionalTargets, benchmarkTargets, rampSettings } = globalThis.TigerBallDriveCore;
