export type Mode = "LIGHTSABER" | "TEXTURED" | "SLIDING" | "MARBLES";

export interface MotionSample {
  sinAlpha: number;
  xMm: number;
  velocityMps: number;
  at: number;
}

export interface MarbleSample {
  gravity: [number, number, number];
  impact: number;
  quaternion: [number, number, number, number];
  marbles: Array<[number, number, number, number]>;
  at: number;
}

export interface DeviceMetrics {
  mode?: Mode;
  cavityMm?: number;
  rebound?: number;
  hapticGain?: number;
  physicsHz?: number;
  physicsTargetHz?: number;
  imuHz?: number;
  audioDrops?: number;
  bounceE?: number;
  gameRound?: number;
  gameRevealed?: boolean;
  gameCount?: number;
  [key: string]: string | number | boolean | undefined;
}

export type GameActionKind = "newRound" | "reveal";

export type TelemetryMessage =
  | { kind: "motion"; value: MotionSample }
  | { kind: "metrics"; value: DeviceMetrics }
  | { kind: "marbles"; value: MarbleSample };

const finite = (value: string | undefined): number | undefined => {
  if (value === undefined || value.trim() === "") return undefined;
  const parsed = Number(value);
  return Number.isFinite(parsed) ? parsed : undefined;
};

function values(line: string): number[] | undefined {
  const parsed = line.split(",").map((part) => finite(part.trim()));
  if (parsed.some((part) => part === undefined)) return undefined;
  return parsed as number[];
}

export function parseTelemetryLine(line: string, at = Date.now()): TelemetryMessage | undefined {
  const clean = line.trim();
  if (!clean) return undefined;

  if (clean.startsWith("$")) {
    const parsed = values(clean.slice(1));
    if (!parsed || parsed.length !== 3) return undefined;
    const [sinAlpha, xMm, velocityMps] = parsed;
    return { kind: "motion", value: { sinAlpha, xMm, velocityMps, at } };
  }

  if (clean.startsWith("#")) {
    const metrics: DeviceMetrics = {};
    const names: Record<string, string> = {
      cavity_mm: "cavityMm", haptic_gain: "hapticGain", phys_hz: "physicsHz",
      phys_target_hz: "physicsTargetHz", imu_hz: "imuHz", audio_drops: "audioDrops", bounce_e: "bounceE",
      game_round: "gameRound", game_count: "gameCount",
    };
    for (const match of clean.slice(1).matchAll(/([A-Za-z][A-Za-z0-9_]*)=([^\s,]+)/g)) {
      const [, key, raw] = match;
      if (key === "mode") {
        const mode = raw.toUpperCase();
        if (["LIGHTSABER", "TEXTURED", "SLIDING", "MARBLES"].includes(mode)) metrics.mode = mode as Mode;
      } else if (key === "game_revealed") {
        if (raw === "0" || raw === "1") metrics.gameRevealed = raw === "1";
      } else {
        const parsed = finite(raw);
        if (parsed === undefined) continue;
        if (key === "game_round" && (!Number.isInteger(parsed) || parsed < 0)) continue;
        if (key === "game_count" && (!Number.isInteger(parsed) || parsed < -1 || parsed > 5)) continue;
        metrics[names[key] ?? key] = parsed;
      }
    }
    return Object.keys(metrics).length ? { kind: "metrics", value: metrics } : undefined;
  }

  if (clean.startsWith("@")) {
    const parsed = values(clean.slice(1));
    // gravity xyz, impact, quaternion wxyz, then zero to five x/y/z/r tuples
    if (!parsed || parsed.length < 8 || parsed.length > 28 || (parsed.length - 8) % 4 !== 0) return undefined;
    const [gx, gy, gz, impact, qw, qx, qy, qz, ...tail] = parsed;
    const marbles: MarbleSample["marbles"] = [];
    for (let offset = 0; offset + 3 < tail.length; offset += 4) {
      marbles.push([tail[offset], tail[offset + 1], tail[offset + 2], tail[offset + 3]]);
    }
    return {
      kind: "marbles",
      value: { gravity: [gx, gy, gz], impact, quaternion: [qw, qx, qy, qz], marbles, at },
    };
  }
  return undefined;
}

export function mergeMetrics(current: DeviceMetrics, next: DeviceMetrics): DeviceMetrics {
  return { ...current, ...next };
}

export function revealedGameCount(metrics: DeviceMetrics, fresh: boolean): number | undefined {
  if (!fresh || metrics.gameRevealed !== true) return undefined;
  const count = metrics.gameCount;
  return Number.isInteger(count) && count! >= 0 && count! <= 5 ? count : undefined;
}

export function gameActionConfirmed(action: GameActionKind, baselineRound: number | undefined, metrics: DeviceMetrics): boolean {
  if (metrics.mode !== "MARBLES") return false;
  if (action === "newRound") {
    return metrics.gameRound !== undefined
      && metrics.gameRound > (baselineRound ?? -1)
      && metrics.gameRevealed === false
      && metrics.gameCount === -1;
  }
  return baselineRound !== undefined
    && metrics.gameRound === baselineRound
    && metrics.gameRevealed === true
    && metrics.gameCount !== undefined
    && Number.isInteger(metrics.gameCount)
    && metrics.gameCount >= 0
    && metrics.gameCount <= 5;
}

export function historyPush<T>(history: T[], value: T, limit = 180): T[] {
  const next = [...history, value];
  return next.length > limit ? next.slice(next.length - limit) : next;
}
