export type Vec3 = [number, number, number];
export type Quaternion = [number, number, number, number];

// Canvas Y increases downward. Firmware positive sin(alpha) accelerates the
// ball toward +X, so the right-hand end must be lower on screen.
export function tubeScreenAngle(sinAlpha: number): number {
  return Math.asin(Math.max(-1, Math.min(1, sinAlpha)));
}

export function tubeSideViewEndpoints(centerX: number, centerY: number, length: number, sinAlpha: number): [[number, number], [number, number]] {
  const angle = tubeScreenAngle(sinAlpha);
  const half = length / 2;
  const dx = half * Math.cos(angle);
  const dy = half * Math.sin(angle);
  return [[centerX - dx, centerY - dy], [centerX + dx, centerY + dy]];
}

// IMU gravity is R(q)^T [0, 0, g]. Rotating the body by R places gravity on
// world +Z, which our viewer intentionally maps downward.
export function rotateBodyToWorld(point: Vec3, quaternion: Quaternion): Vec3 {
  const n = Math.hypot(...quaternion);
  if (!Number.isFinite(n) || n < 1e-8) return [...point];
  const [w, x, y, z] = quaternion.map(v => v / n);
  const [px, py, pz] = point;
  return [
    (1 - 2 * (y*y + z*z))*px + 2*(x*y - w*z)*py + 2*(x*z + w*y)*pz,
    2*(x*y + w*z)*px + (1 - 2*(x*x + z*z))*py + 2*(y*z - w*x)*pz,
    2*(x*z - w*y)*px + 2*(y*z + w*x)*py + (1 - 2*(x*x + y*y))*pz,
  ];
}

export function projectWorld(point: Vec3): [number, number] {
  const [x, y, z] = point;
  return [x + .52*y, z + .30*y];
}
