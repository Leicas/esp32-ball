import assert from "node:assert/strict";
import test from "node:test";
import { gameActionConfirmed, parseTelemetryLine, revealedGameCount } from "../src/telemetry.ts";

test("parses firmware motion packets and preserves the measurement timestamp", () => {
  const sample = parseTelemetryLine("$0.125,431.5,-0.24", 1234);
  assert.deepEqual(sample, {
    kind: "motion",
    value: { sinAlpha: 0.125, xMm: 431.5, velocityMps: -0.24, at: 1234 },
  });
});

test("maps firmware metric names to the UI model", () => {
  const packet = parseTelemetryLine("# mode=TEXTURED cavity_mm=1000 rebound=1 haptic_gain=0.7 phys_hz=58 phys_target_hz=58 imu_hz=200 audio_drops=0 bounce_e=0.30");
  assert.deepEqual(packet, {
    kind: "metrics",
    value: {
      mode: "TEXTURED", cavityMm: 1000, rebound: 1, hapticGain: 0.7,
      physicsHz: 58, physicsTargetHz: 58, imuHz: 200, audioDrops: 0, bounceE: 0.3,
    },
  });
});

test("accepts exactly three motion values and the full three-marble packet", () => {
  assert.equal(parseTelemetryLine("$0.1,80,0.2,999"), undefined);
  assert.equal(parseTelemetryLine("$0.1,80"), undefined);
  const marble = parseTelemetryLine("@0,0,-9.8,0.3,1,0,0,0,10,12,8,5,25,12,9,4,40,15,13,3", 99);
  assert.equal(marble?.kind, "marbles");
  if (marble?.kind === "marbles") {
    assert.deepEqual(marble.value.marbles, [[10, 12, 8, 5], [25, 12, 9, 4], [40, 15, 13, 3]]);
    assert.equal(marble.value.at, 99);
  }
  assert.equal(parseTelemetryLine("@0,0,-9.8,0.3,1,0,0,0,10,12,8,5,25,12,9,4,40,15,13,3,999"), undefined);
});

test("ignores invalid mode names and malformed packets", () => {
  assert.deepEqual(parseTelemetryLine("# mode=UNKNOWN"), undefined);
  assert.equal(parseTelemetryLine("hello"), undefined);
  assert.equal(parseTelemetryLine("$NaN,0,1"), undefined);
});

test("parses blind-game state while keeping hidden counts private to display logic", () => {
  const hidden = parseTelemetryLine("# mode=MARBLES game_round=7 game_revealed=0 game_count=-1");
  assert.deepEqual(hidden, { kind: "metrics", value: { mode: "MARBLES", gameRound: 7, gameRevealed: false, gameCount: -1 } });
  assert.equal(revealedGameCount(hidden.value, true), undefined);
  assert.equal(revealedGameCount({ gameRevealed: false, gameCount: 4 }, true), undefined);
  assert.equal(revealedGameCount({ gameRevealed: true, gameCount: 5 }, false), undefined);
  assert.equal(revealedGameCount({ gameRevealed: true, gameCount: 0 }, true), 0);
  assert.equal(revealedGameCount({ gameRevealed: true, gameCount: 5 }, true), 5);
  assert.equal(revealedGameCount({ gameRevealed: true, gameCount: -1 }, true), undefined);
  assert.equal(revealedGameCount({ gameRevealed: true, gameCount: 6 }, true), undefined);
});

test("accepts hidden and revealed variable-length marble packets from zero to five balls", () => {
  const hidden = parseTelemetryLine("@0,0,1,0,1,0,0,0");
  assert.equal(hidden?.kind, "marbles");
  if (hidden?.kind === "marbles") assert.deepEqual(hidden.value.marbles, []);

  const five = parseTelemetryLine("@0,0,1,0,1,0,0,0,5,5,5,5,10,5,5,5,15,5,5,5,20,5,5,5,25,5,5,5");
  assert.equal(five?.kind, "marbles");
  if (five?.kind === "marbles") assert.equal(five.value.marbles.length, 5);

  assert.equal(parseTelemetryLine("@0,0,1,0,1,0,0"), undefined);
  assert.equal(parseTelemetryLine("@0,0,1,0,1,0,0,0,5"), undefined);
  assert.equal(parseTelemetryLine(`@${Array.from({ length: 29 }, () => "0").join(",")}`), undefined);
  assert.deepEqual(parseTelemetryLine("# mode=MARBLES game_revealed=2 game_count=6 game_round=-1"), {
    kind: "metrics", value: { mode: "MARBLES" },
  });
});

test("confirms game actions only from matching fresh firmware state fields", () => {
  const hiddenRound = { mode: "MARBLES", gameRound: 8, gameRevealed: false, gameCount: -1 };
  assert.equal(gameActionConfirmed("newRound", 7, hiddenRound), true);
  assert.equal(gameActionConfirmed("newRound", 8, hiddenRound), false);
  assert.equal(gameActionConfirmed("newRound", 7, { ...hiddenRound, gameRevealed: true, gameCount: 2 }), false);
  assert.equal(gameActionConfirmed("newRound", 7, { ...hiddenRound, mode: "TEXTURED" }), false);

  const revealed = { mode: "MARBLES", gameRound: 8, gameRevealed: true, gameCount: 0 };
  assert.equal(gameActionConfirmed("reveal", 8, revealed), true);
  assert.equal(gameActionConfirmed("reveal", 7, revealed), false);
  assert.equal(gameActionConfirmed("reveal", 8, { ...revealed, gameCount: 6 }), false);
  assert.equal(gameActionConfirmed("reveal", 8, { ...revealed, mode: "ROLLING" }), false);

  let merged = { mode: "MARBLES", gameRound: 7, gameRevealed: true, gameCount: 2 };
  merged = { ...merged, gameRound: 8 };
  assert.equal(gameActionConfirmed("newRound", 7, merged), false, "old reveal fields cannot confirm a new round");
  merged = { ...merged, gameRevealed: false, gameCount: -1 };
  assert.equal(gameActionConfirmed("newRound", 7, merged), true, "a later status line can complete the merged transition");

  let revealState = { mode: "MARBLES", gameRound: 8, gameRevealed: false, gameCount: -1 };
  revealState = { ...revealState, gameRevealed: true };
  assert.equal(gameActionConfirmed("reveal", 8, revealState), false);
  revealState = { ...revealState, gameCount: 5 };
  assert.equal(gameActionConfirmed("reveal", 8, revealState), true);
});

 test("recognizes lightsaber mode", () => { assert.equal(parseTelemetryLine("# mode=LIGHTSABER").value.mode, "LIGHTSABER"); });
