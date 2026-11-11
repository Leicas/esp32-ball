import { invoke } from "@tauri-apps/api/core";
import { listen } from "@tauri-apps/api/event";
import { tubeSideViewEndpoints, tubeScreenAngle } from "./geometry";
import {
  historyPush,
  gameActionConfirmed,
  mergeMetrics,
  parseTelemetryLine,
  revealedGameCount,
  type DeviceMetrics,
  type GameActionKind,
  type Mode,
  type MotionSample,
} from "./telemetry";
import "./styles.css";

type Port = { name: string; description: string };
type ConnectionEvent = { state: string; port: string; detail: string };
type PendingAction = { command: string; baseline: DeviceMetrics; targetMode?: Mode; gameAction?: GameActionKind; timeout: number };

const isTauri = "__TAURI_INTERNALS__" in window || "__TAURI__" in window;
const app = document.querySelector<HTMLDivElement>("#app")!;

app.innerHTML = `
  <main class="console-shell">
    <header class="topbar">
      <a class="brand" href="#" aria-label="Rolling Stone home">
        <span class="brand-mark" aria-hidden="true"><span></span><span></span><span></span></span>
        <span class="brand-copy"><strong>Rolling Stone</strong><small>motion console</small></span>
      </a>
      <div class="device-dock">
        <span class="link-indicator" id="link-indicator" aria-hidden="true"></span>
        <label class="visually-hidden" for="port-select">Serial port</label>
        <select id="port-select" aria-label="Serial port"><option value="">Choose a port</option></select>
        <button class="icon-button refresh-button" id="refresh-ports" type="button" aria-label="Refresh serial ports" title="Refresh ports">
          <svg viewBox="0 0 20 20" aria-hidden="true"><path d="M16.5 8A6.7 6.7 0 0 0 4.9 5.7L3.5 7.2M3.5 7.2V3.8M3.5 7.2h3.4M3.5 12a6.7 6.7 0 0 0 11.6 2.3l1.4-1.5m0 0v3.4m0-3.4h-3.4"/></svg>
        </button>
        <button class="connect-button" id="connect-button" type="button">Connect</button>
      </div>
      <div class="connection-copy" aria-live="polite">
        <span id="connection-state">Looking for a board</span>
        <small id="connection-detail">Connect an ESP32 to begin</small>
      </div>
    </header>

    <section class="workspace" aria-label="Device workspace">
      <div class="instrument-column">
        <div class="section-heading">
          <div>
            <p class="eyebrow">Live instrument</p>
            <h1>Motion field</h1>
          </div>
          <div class="stream-stamp"><span class="pulse-dot"></span><span id="stream-status">No signal</span></div>
        </div>

        <section class="field-panel" aria-label="Live ball motion visualization">
          <canvas id="motion-canvas" aria-label="Live test tube motion or blind shake box"></canvas>
          <div class="field-caption"><span class="field-state" id="field-state">Waiting for telemetry</span><span id="field-coordinate">Position unavailable</span></div>
          <span class="view-kind" id="view-kind">Side view</span>
          <div class="field-key" id="field-key"><span><i class="key-ball"></i> measured position</span><span id="field-key-path"><i class="key-path"></i> recent path</span></div>
        </section>

        <section class="trace-section" aria-label="Recent telemetry or game controls">
          <div class="trace-heading"><h2 id="trace-title">Recent motion</h2><span id="trace-window">Last 3 seconds</span></div>
          <div class="trace-grid" id="trace-grid">
            <div class="trace-row"><div class="trace-label"><span id="angle-label">Incline</span><strong id="angle-value">—<small>sin α</small></strong></div><canvas class="sparkline" id="angle-spark" aria-label="Recent incline history"></canvas><span class="trace-unit" id="angle-unit">input</span></div>
            <div class="trace-row"><div class="trace-label"><span id="velocity-label">Velocity</span><strong id="velocity-value">—<small>m/s</small></strong></div><canvas class="sparkline" id="velocity-spark" aria-label="Recent velocity history"></canvas><span class="trace-unit" id="velocity-unit">along tube</span></div>
          </div>
          <div class="game-controls" id="game-controls" hidden>
            <div class="game-prompt"><span>How many balls?</span><strong id="game-round">Waiting for round</strong></div>
            <div class="guess-buttons" role="group" aria-label="Choose your guess from zero to five balls">
              ${Array.from({ length: 6 }, (_, count) => `<button class="guess-button" type="button" data-guess="${count}" aria-pressed="false">${count}</button>`).join("")}
            </div>
            <div class="game-actions">
              <button class="game-action" id="game-new-round" type="button" disabled><span id="game-new-round-label">New round</span> <kbd>N</kbd></button>
              <button class="game-action game-reveal" id="game-reveal" type="button" disabled><span>Reveal</span> <kbd>V</kbd></button>
            </div>
            <p class="game-result" id="game-result" role="status" aria-live="polite">The count stays hidden until you reveal.</p>
          </div>
        </section>
      </div>

      <aside class="readout-column" aria-label="Device readouts">
        <section class="primary-readout">
          <div class="readout-title"><span class="readout-kicker" id="position-label">Ball position</span></div>
          <div class="big-value" id="position-value">—<span>mm</span></div>
          <div class="readout-foot"><span id="position-foot">From tube inlet</span><span id="cavity-value">Cavity — mm</span></div>
          <div class="meter-track"><span id="position-meter"></span></div>
        </section>

        <section class="mode-readout">
          <div class="readout-title"><span class="readout-kicker">Surface mode</span><span class="mode-led" id="mode-led"></span></div>
          <p class="mode-name" id="mode-name">—</p>
          <p class="mode-explainer" id="mode-description">A mode appears when the board reports its current surface.</p>
        </section>

        <section class="health-readout">
          <div class="readout-title"><span class="readout-kicker">Device health</span><span class="health-state" id="health-state">Offline</span></div>
          <div class="health-row"><span>Physics loop</span><strong id="physics-value">— <small>Hz</small></strong><span class="health-target" id="physics-target">target —</span></div>
          <div class="health-row"><span>IMU stream</span><strong id="imu-value">— <small>Hz</small></strong><span class="health-target">sensor</span></div>
          <div class="health-row"><span>Audio drops</span><strong id="audio-value">—</strong><span class="health-target" id="bounce-value">e —</span></div>
        </section>
        <div class="feedback-line" id="feedback-line" role="status" aria-live="polite">Controls unlock with a fresh device signal.</div>
      </aside>
    </section>

    <section class="control-deck" aria-label="Device controls">
      <div class="deck-intro"><span class="deck-icon" aria-hidden="true">↗</span><div><strong>Tune the surface</strong><small>Changes are confirmed by the board</small></div></div>
      <div class="mode-controls" role="group" aria-label="Choose surface mode">
        <button class="mode-button" data-mode="LIGHTSABER" data-command="l" type="button" disabled><span class="mode-swatch swatch-roll"></span><span>Lightsaber</span><kbd>L</kbd></button>
        <button class="mode-button" data-mode="TEXTURED" data-command="t" type="button" disabled><span class="mode-swatch swatch-texture"></span><span>Textured</span><kbd>T</kbd></button>
        <button class="mode-button" data-mode="SLIDING" data-command="s" type="button" disabled><span class="mode-swatch swatch-slide"></span><span>Sliding</span><kbd>S</kbd></button>
        <button class="mode-button" data-mode="MARBLES" data-command="m" type="button" disabled><span class="mode-swatch swatch-marble"></span><span>Marbles</span><kbd>M</kbd></button>
      </div>
      <div class="tuning-controls" role="group" aria-label="Adjust device parameters">
        <button class="utility-button" id="gain-button" type="button" disabled><span class="utility-icon">◒</span><span>Haptic gain</span><kbd>G</kbd><i id="gain-value">—</i></button>
        <button class="utility-button" id="bounce-button" type="button" disabled><span class="utility-icon bounce-icon">⌁</span><span>Bounce</span><kbd>B</kbd><i id="bounce-control-value">—</i></button>
        <div class="cavity-control" id="cavity-control" title="Adjusts tube cavity length. MARBLES uses a fixed 50 × 25 × 25 mm enclosure."><span class="cavity-label" id="cavity-label">Cavity</span><button id="cavity-minus" type="button" aria-label="Decrease cavity by 50 millimeters" disabled>−</button><strong id="cavity-control-value">—</strong><button id="cavity-plus" type="button" aria-label="Increase cavity by 50 millimeters" disabled>+</button><kbd>+/−</kbd></div>
      </div>
    </section>

    <footer class="statusbar"><span><i class="status-glyph"></i> Firmware stream <strong id="firmware-state">idle</strong></span><span id="last-packet">No packets received</span><span class="keyboard-hint">Keyboard shortcuts active when connected</span><button id="demo-button" class="demo-button" type="button" hidden>Start demo</button></footer>
  </main>`;

const $ = <T extends HTMLElement>(selector: string): T => document.querySelector<T>(selector)!;
const portSelect = $("#port-select") as HTMLSelectElement;
const connectButton = $("#connect-button") as HTMLButtonElement;
const refreshButton = $("#refresh-ports") as HTMLButtonElement;
const feedbackLine = $("#feedback-line");
const modeButtons = [...document.querySelectorAll<HTMLButtonElement>(".mode-button")];
const demoButton = $("#demo-button") as HTMLButtonElement;
const angleCanvas = $("#angle-spark") as HTMLCanvasElement;
const velocityCanvas = $("#velocity-spark") as HTMLCanvasElement;

const modeColors: Record<Mode, string> = { LIGHTSABER: "#62d6bd", TEXTURED: "#e2ad6d", SLIDING: "#7aa9ee", MARBLES: "#d893d3" };
const modeDescriptions: Record<Mode, string> = {
  LIGHTSABER: "A steady hum that rises with swings and reacts to sharp motion.",
  TEXTURED: "Surface detail with the board’s native haptic texture.",
  SLIDING: "A smooth slide with reduced rolling response.",
  MARBLES: "Shake the sealed box, guess the count, then reveal.",
};
let visualDirty = true;
let previousFresh = false;

const state: {
  ports: Port[];
  connected: boolean;
  port: string;
  connectionDetail: string;
  connectionState: string;
  metrics: DeviceMetrics;
  history: MotionSample[];
  latestMotion?: MotionSample;
  lastPacketAt?: number;
  lastSignalAt?: number;
  lastLine?: string;
  pending?: PendingAction;
  demo: boolean;
  demoStartedAt: number;
  demoTimer?: number;
  userGuess?: number;
  demoGameAnswer?: number;
  demoGameRound: number;
  autoConnected: boolean;
} = {
  ports: [], connected: false, port: "", connectionDetail: "Connect an ESP32 to begin", connectionState: "Looking for a board",
  metrics: {}, history: [], demo: false, demoStartedAt: 0, demoGameRound: 0, autoConnected: false,
};

function setConnection(stateText: string, port = state.port, detail = "") {
  state.connectionState = stateText;
  state.port = port;
  state.connectionDetail = detail || (port ? port : "Connect an ESP32 to begin");
  const normalized = stateText.trim().toLowerCase();
  state.connected = ["connected", "connecting", "reconnecting", "stale"].includes(normalized);
  $("#connection-state").textContent = state.connectionState;
  $("#connection-detail").textContent = state.connectionDetail;
  $("#link-indicator").classList.toggle("is-connected", state.connected);
  connectButton.textContent = state.connected ? "Disconnect" : "Connect";
  connectButton.classList.toggle("is-connected", state.connected);
  connectButton.disabled = state.demo || (!state.connected && !portSelect.value);
  syncControls();
}

function isFresh(): boolean {
  return state.demo || (state.connectionState.trim() === "Connected" && state.lastSignalAt !== undefined && Date.now() - state.lastSignalAt < 500);
}

function syncControls() {
  const enabled = isFresh() && !state.pending;
  const cavityEnabled = enabled && state.metrics.mode !== "MARBLES";
  const bounceEnabled = enabled && state.metrics.mode !== "MARBLES";
  const gameEnabled = enabled && state.metrics.mode === "MARBLES";
  const guessEnabled = gameEnabled && state.metrics.gameRevealed === false;
  for (const button of [...modeButtons, $("#gain-button") as HTMLButtonElement]) {
    button.disabled = !enabled;
  }
  ($( "#bounce-button") as HTMLButtonElement).disabled = !bounceEnabled;
  for (const button of document.querySelectorAll<HTMLButtonElement>(".guess-button")) button.disabled = !guessEnabled;
  (document.querySelector<HTMLButtonElement>("#game-new-round")!).disabled = !gameEnabled;
  (document.querySelector<HTMLButtonElement>("#game-reveal")!).disabled = !guessEnabled;
  const cavityLength = state.metrics.cavityMm ?? 1000;
  ($("#cavity-minus") as HTMLButtonElement).disabled = !cavityEnabled || cavityLength <= 100;
  ($("#cavity-plus") as HTMLButtonElement).disabled = !cavityEnabled || cavityLength >= 2000;
  connectButton.disabled = state.demo || (!state.connected && !portSelect.value);
  refreshButton.disabled = state.demo;
}

function setFeedback(message: string, tone: "plain" | "pending" | "confirmed" | "error" = "plain") {
  feedbackLine.textContent = message;
  feedbackLine.dataset.tone = tone;
}

function setPorts(ports: Port[]) {
  state.ports = ports;
  const oldSelection = portSelect.value;
  portSelect.replaceChildren(new Option("Choose a port", ""));
  for (const port of ports) {
    const option = new Option(`${port.name}  ·  ${port.description || "Serial device"}`, port.name);
    portSelect.add(option);
  }
  const selected = ports.find((port) => port.name === oldSelection);
  const recognized = ports.filter(isEspressifPort);
  const preferred = selected ?? (recognized.length === 1 ? recognized[0] : undefined);
  if (preferred) portSelect.value = preferred.name;
  if (recognized.length === 1 && !state.autoConnected && !state.connected) {
    state.autoConnected = true;
    window.setTimeout(() => void connectDevice(recognized[0].name, true), 180);
  }
  if (ports.length === 0) setConnection("Board not found", "", "Plug in a board and refresh ports");
  else if (!state.connected) setConnection("Ready to connect", preferred?.name ?? "", `${ports.length} serial ${ports.length === 1 ? "port" : "ports"} available`);
  connectButton.disabled = state.demo || (!state.connected && !portSelect.value);
}

function isEspressifPort(port: Port): boolean {
  const label = `${port.name} ${port.description}`.toLowerCase();
  return /espressif|usb jtag|cp210[0-9x]|silicon labs|ch340|wch usb|esp32/.test(label);
}

async function refreshPorts() {
  if (!isTauri) {
    setPorts([]);
    return;
  }
  refreshButton.classList.add("is-spinning");
  try {
    const ports = await invoke<Port[]>("list_ports");
    setPorts(ports);
  } catch (error) {
    setConnection("Port scan failed", "", String(error));
  } finally {
    refreshButton.classList.remove("is-spinning");
  }
}

async function connectDevice(port = portSelect.value, automatic = false) {
  if (!port || !isTauri) return;
  connectButton.disabled = true;
  setConnection("Connecting", port, automatic ? "Connecting to the detected ESP32" : "Opening serial link");
  setFeedback("Waiting for the board to report motion…", "pending");
  try {
    await invoke("connect", { port });
    // The event stream owns connection state; the command only starts the worker.
  } catch (error) {
    setConnection("Connection failed", port, String(error));
    setFeedback(`Could not connect: ${String(error)}`, "error");
  }
}

async function disconnectDevice() {
  const hadPendingAction = Boolean(state.pending);
  connectButton.disabled = true;
  try {
    await invoke("disconnect");
    setConnection("Disconnected", state.port, "Serial link closed");
    if (hadPendingAction) clearPending("Disconnected before confirmation");
    else setFeedback("Disconnected");
  } catch (error) {
    setFeedback(`Could not disconnect: ${String(error)}`, "error");
  }
}

function isControlFresh(): boolean {
  return isFresh() && !state.pending;
}

function requestCommand(command: string, targetMode?: Mode) {
  if (command === "b" && state.metrics.mode === "MARBLES") return;
  if ((command === "n" || command === "v") && state.metrics.mode !== "MARBLES") return;
  if (command === "v" && state.metrics.gameRevealed !== false) return;
  if (state.demo) {
    applyDemoCommand(command, targetMode);
    return;
  }
  if (!isTauri || !isControlFresh()) return;
  const gameAction: GameActionKind | undefined = command === "n" || (command === "m" && targetMode === "MARBLES")
    ? "newRound"
    : command === "v" ? "reveal" : undefined;
  state.pending = {
    command,
    baseline: { ...state.metrics },
    targetMode,
    gameAction,
    timeout: window.setTimeout(() => {
      if (!state.pending || state.pending.command !== command) return;
      state.pending = undefined;
      syncControls();
      render();
      setFeedback(`Sent “${command}” · no matching device update yet`, "plain");
    }, 1800),
  };
  syncControls();
  render();
  const actionNames: Record<string, string> = { l: "Lightsaber", t: "Textured", s: "Sliding", m: "Marbles · new round", b: "Bounce", g: "Haptic gain", "+": "Cavity +50 mm", "-": "Cavity −50 mm", n: "New round", v: "Reveal" };
  setFeedback(`Sending ${actionNames[command] ?? command}…`, "pending");
  void invoke("send_command", { command }).catch((error) => {
    if (state.pending?.command === command) clearPending(`Command failed: ${String(error)}`, "error");
  });
}

function chooseGameGuess(count: number) {
  if (!isControlFresh() || state.metrics.mode !== "MARBLES" || state.metrics.gameRevealed !== false) return;
  state.userGuess = count;
  render();
  if (state.demo) setFeedback("Demo guess recorded · answer remains hidden", "confirmed");
}

function confirmPending(metrics: DeviceMetrics) {
  const action = state.pending;
  if (!action) return;
  const baseline = action.baseline;
  let confirmed = false;
  if (action.gameAction === "newRound") {
    confirmed = gameActionConfirmed("newRound", baseline.gameRound, metrics)
      && (action.targetMode === undefined || metrics.mode === action.targetMode);
  } else if (action.gameAction === "reveal") {
    confirmed = gameActionConfirmed("reveal", baseline.gameRound, metrics);
  } else if (action.targetMode && metrics.mode === action.targetMode) confirmed = true;
  else if (action.command === "b") confirmed = changed("rebound");
  else if (action.command === "g") confirmed = changed("hapticGain");
  else if (action.command === "+" || action.command === "-") confirmed = changed("cavityMm");
  if (!confirmed) return;
  window.clearTimeout(action.timeout);
  state.pending = undefined;
  syncControls();
  setFeedback(`${action.gameAction === "newRound" ? "New round" : action.gameAction === "reveal" ? "Reveal" : action.targetMode ?? (action.command === "b" ? "Bounce" : action.command === "g" ? "Haptic gain" : "Cavity")} confirmed by device`, "confirmed");

  function changed(key: string) {
    return metrics[key] !== undefined && baseline[key] !== metrics[key];
  }
}

function clearPending(message: string, tone: "plain" | "error" = "plain") {
  if (state.pending) window.clearTimeout(state.pending.timeout);
  state.pending = undefined;
  syncControls();
  setFeedback(message, tone);
}

function onTelemetry(line: string) {
  const parsed = parseTelemetryLine(line);
  if (!parsed) return;
  state.lastPacketAt = Date.now();
  state.lastLine = line;
  if (parsed.kind === "motion") {
    state.lastSignalAt = Date.now();
    state.latestMotion = parsed.value;
    state.history = historyPush(state.history, parsed.value);
  } else if (parsed.kind === "metrics") {
    const previousMode = state.metrics.mode;
    const previousRound = state.metrics.gameRound;
    state.metrics = mergeMetrics(state.metrics, parsed.value);
    if (parsed.value.mode && parsed.value.mode !== previousMode) {
      state.latestMotion = undefined;
      state.history = [];
      state.userGuess = undefined;
    }
    if (parsed.value.gameRound !== undefined && parsed.value.gameRound !== previousRound) {
      state.userGuess = undefined;
    }
    confirmPending(state.metrics);
  } else {
    state.lastSignalAt = Date.now();
  }
  if (state.connected && state.connectionState !== "Connected") setConnection("Connected", state.port, "Receiving device telemetry");
  if (isFresh() && !state.pending && (feedbackLine.textContent?.startsWith("Waiting for the board") || feedbackLine.textContent?.startsWith("Disconnected before confirmation"))) {
    setFeedback("Live telemetry received");
  }
  render();
}

function applyDemoCommand(command: string, targetMode?: Mode) {
  if (targetMode && targetMode !== state.metrics.mode) {
    state.metrics.mode = targetMode;
    state.latestMotion = undefined;
    state.history = [];
  }
  if (command === "g") {
    const levels = [0.4, 0.7, 1];
    const current = levels.indexOf(Number(state.metrics.hapticGain ?? 0.4));
    state.metrics.hapticGain = levels[(current + 1 + levels.length) % levels.length];
  }
  if (command === "b") state.metrics.rebound = Number(state.metrics.rebound ?? 0) > 0 ? 0 : 1;
  if (command === "+") state.metrics.cavityMm = Math.min(2000, Number(state.metrics.cavityMm ?? 1000) + 50);
  if (command === "-") state.metrics.cavityMm = Math.max(100, Number(state.metrics.cavityMm ?? 1000) - 50);
  if (command === "n" || (command === "m" && targetMode === "MARBLES")) startDemoGameRound();
  if (command === "v" && state.metrics.mode === "MARBLES" && state.metrics.gameRevealed === false) {
    state.metrics.gameRevealed = true;
    state.metrics.gameCount = state.demoGameAnswer ?? 0;
  }
  render();
  setFeedback("Demo control updated · no device command was sent", "confirmed");
}

function toggleDemo() {
  if (state.demo) {
    window.clearInterval(state.demoTimer);
    state.demo = false;
    state.history = [];
    state.latestMotion = undefined;
    state.metrics = {};
    state.lastPacketAt = undefined;
    state.lastSignalAt = undefined;
    demoButton.textContent = "Start demo";
    setConnection("Browser preview", "", "No board connected");
    $("#firmware-state").textContent = "idle";
    setFeedback("Demo stopped · connect a board for live telemetry");
    render();
    return;
  }
  state.demo = true;
  state.demoStartedAt = performance.now();
  state.connectionState = "Demo preview";
  state.connectionDetail = "Sample telemetry · no hardware linked";
  state.metrics = { mode: "TEXTURED", cavityMm: 1000, rebound: 1, bounceE: 0.3, hapticGain: 0.4, physicsHz: 58, physicsTargetHz: 58, imuHz: 200, audioDrops: 0 };
  state.demoGameRound = 0;
  state.demoGameAnswer = undefined;
  state.userGuess = undefined;
  demoButton.textContent = "Stop demo";
  const step = () => {
    const t = (performance.now() - state.demoStartedAt) / 1000;
    const xMm = 500 + 380 * Math.sin(t * 0.68) * Math.cos(t * 0.13);
    const velocityMps = 0.26 * Math.cos(t * 0.68);
    const sinAlpha = Math.sin(t * 0.5) * 0.33;
    const at = Date.now();
    const motion: MotionSample = { sinAlpha, xMm, velocityMps, at };
    state.latestMotion = motion;
    state.history = historyPush(state.history, motion);
    state.lastPacketAt = at;
    state.lastSignalAt = at;
    render();
  };
  step();
  state.demoTimer = window.setInterval(step, 1000 / 24);
  setFeedback("Demo is active · sample motion only", "confirmed");
  render();
}

function startDemoGameRound() {
  state.demoGameRound += 1;
  state.demoGameAnswer = (state.demoGameRound * 3 + 2) % 6;
  state.userGuess = undefined;
  state.metrics.gameRound = state.demoGameRound;
  state.metrics.gameRevealed = false;
  state.metrics.gameCount = -1;
}

function fmt(value: number | undefined, digits = 2): string {
  return value === undefined || !Number.isFinite(value) ? "—" : value.toFixed(digits);
}

function render() {
  visualDirty = true;
  const motion = state.latestMotion;
  const metrics = state.metrics;
  const fresh = isFresh();
  const mode = metrics.mode as Mode | undefined;
  const isMarbles = mode === "MARBLES";
  const revealedCount = isMarbles ? revealedGameCount(metrics, fresh) : undefined;
  $("#stream-status").textContent = state.demo ? "Demo signal" : fresh ? "Receiving" : state.connected ? "Signal stale" : "No signal";
  $("#stream-status").parentElement?.classList.toggle("is-live", fresh);
  $("#firmware-state").textContent = state.demo ? "demo" : fresh ? "streaming" : state.connected ? "waiting" : "idle";
  $("#last-packet").textContent = state.lastPacketAt ? `Last packet ${Math.max(0, (Date.now() - state.lastPacketAt) / 1000).toFixed(1)} s ago` : "No packets received";
  const roundLabel = Number.isInteger(metrics.gameRound) ? `Round ${metrics.gameRound}` : "Round pending";
  const gameIsHidden = metrics.gameRevealed === false && metrics.gameCount === -1;
  $("#field-state").textContent = isMarbles
    ? fresh ? `${state.demo ? "Demo · " : "Live · "}${roundLabel} · ${revealedCount === undefined ? "count hidden" : "revealed"}` : "Waiting for fresh game status"
    : fresh ? (state.demo ? "Demo · sample motion" : "Live device position") : "Waiting for telemetry";
  $("#field-coordinate").textContent = isMarbles
    ? revealedCount === undefined ? "Sealed shake box" : "Reveal confirmed"
    : motion ? `x ${fmt(motion.xMm, 1)} mm` : "Position unavailable";
  $("#field-key-path").toggleAttribute("hidden", isMarbles);
  $("#field-key").toggleAttribute("hidden", isMarbles);
  $("#position-label").textContent = isMarbles ? revealedCount === undefined ? "Your guess" : "Actual count" : "Ball position";
  $("#position-foot").textContent = isMarbles ? revealedCount === undefined ? "Guess stays on this device" : state.userGuess === undefined ? "No guess entered" : state.userGuess === revealedCount ? "Correct guess" : `You guessed ${state.userGuess}` : "From tube inlet";
  $("#cavity-value").textContent = isMarbles ? roundLabel : `Cavity ${metrics.cavityMm === undefined ? "—" : `${fmt(metrics.cavityMm, 0)} mm`}`;
  $("#cavity-label").textContent = isMarbles ? "Round" : "Cavity";
  $("#cavity-control").title = isMarbles
    ? "Tube cavity length cannot change the fixed MARBLES enclosure (50 × 25 × 25 mm)."
    : "Adjust tube cavity length in 50 mm steps.";
  $("#cavity-control-value").textContent = isMarbles ? "fixed" : metrics.cavityMm === undefined ? "—" : fmt(metrics.cavityMm, 0);
  $("#trace-title").textContent = isMarbles ? "Blind count game" : "Recent motion";
  $("#trace-window").textContent = isMarbles ? "Guess first · reveal second" : "Last 3 seconds";
  $("#trace-grid").toggleAttribute("hidden", isMarbles);
  $("#game-controls").toggleAttribute("hidden", !isMarbles);
  $("#angle-label").textContent = "Incline";
  $("#velocity-label").textContent = "Velocity";
  $("#angle-unit").textContent = "input";
  $("#velocity-unit").textContent = "along tube";
  $("#angle-value").innerHTML = isMarbles ? "—" : `${motion ? fmt(motion.sinAlpha, 3) : "—"}<small>sin α</small>`;
  $("#velocity-value").innerHTML = isMarbles ? "—" : `${motion ? fmt(motion.velocityMps, 3) : "—"}<small>m/s</small>`;
  $("#position-value").innerHTML = isMarbles
    ? `<span class="game-value">${revealedCount ?? state.userGuess ?? "—"}</span><span>balls</span>`
    : `${motion ? fmt(motion.xMm, 1) : "—"}<span>mm</span>`;
  const meter = $("#position-meter") as HTMLSpanElement;
  meter.parentElement?.toggleAttribute("hidden", isMarbles);
  const positionRange = Math.max(100, Math.min(2000, metrics.cavityMm ?? 1000));
  meter.style.width = motion ? `${Math.max(0, Math.min(100, motion.xMm / positionRange * 100))}%` : "0%";
  $("#game-round").textContent = fresh && Number.isInteger(metrics.gameRound) ? `${state.demo ? "DEMO · " : ""}${roundLabel}${revealedCount === undefined ? " · SEALED" : " · REVEALED"}` : "Waiting for fresh round status";
  $("#game-result").textContent = !fresh
    ? "Reconnect for a fresh round state."
    : !Number.isInteger(metrics.gameRound)
      ? "Waiting for the firmware to report a round."
      : revealedCount !== undefined
        ? state.userGuess === undefined ? `The answer is ${revealedCount}.` : state.userGuess === revealedCount ? `Correct · the box held ${revealedCount}.` : `Not quite · the box held ${revealedCount}.`
        : gameIsHidden ? "Choose your guess, then reveal the count." : "Waiting for the board to confirm the hidden count.";
  for (const button of document.querySelectorAll<HTMLButtonElement>(".guess-button")) {
    const selected = state.userGuess === Number(button.dataset.guess);
    button.classList.toggle("is-selected", selected);
    button.setAttribute("aria-pressed", String(selected));
  }
  $("#game-new-round-label").textContent = revealedCount === undefined ? "New round" : "Play again";
  $("#mode-name").textContent = mode ?? "—";
  $("#mode-description").textContent = mode ? modeDescriptions[mode] : "A mode appears when the board reports its current surface.";
  const led = $("#mode-led");
  led.style.backgroundColor = mode ? modeColors[mode] : "transparent";
  led.style.color = mode ? modeColors[mode] : "transparent";
  led.classList.toggle("is-lit", Boolean(mode));
  for (const button of modeButtons) {
    button.classList.toggle("is-active", button.dataset.mode === mode);
    button.classList.toggle("is-pending", state.pending?.targetMode === button.dataset.mode);
  }
  $("#physics-value").innerHTML = `${metrics.physicsHz === undefined ? "—" : fmt(metrics.physicsHz, 0)} <small>Hz</small>`;
  $("#physics-target").textContent = `target ${metrics.physicsTargetHz === undefined ? "—" : fmt(metrics.physicsTargetHz, 0)}`;
  $("#imu-value").innerHTML = `${metrics.imuHz === undefined ? "—" : fmt(metrics.imuHz, 0)} <small>Hz</small>`;
  $("#audio-value").textContent = metrics.audioDrops === undefined ? "—" : fmt(metrics.audioDrops, 0);
  $("#bounce-value").textContent = `e ${fmt(metrics.bounceE ?? metrics.rebound, 2)}`;
  $("#bounce-value").toggleAttribute("hidden", isMarbles);
  $("#bounce-control-value").textContent = isMarbles ? "fixed" : metrics.rebound === undefined ? "—" : metrics.rebound > 0 ? "ON" : "OFF";
  $("#bounce-button").toggleAttribute("hidden", isMarbles);
  $("#cavity-control").toggleAttribute("hidden", isMarbles);
  $("#gain-value").textContent = fmt(metrics.hapticGain, 2);
  $("#health-state").textContent = fresh ? "Healthy" : state.connected ? "Waiting" : "Offline";
  $("#health-state").className = `health-state${fresh ? " is-healthy" : ""}`;
  document.body.dataset.mode = mode ?? "none";
  syncControls();
}

function paintFrame() {
  if (visualDirty) {
    drawSparklines();
    drawMotionField();
    visualDirty = false;
  }
  window.requestAnimationFrame(paintFrame);
}

function fitCanvas(canvas: HTMLCanvasElement): CanvasRenderingContext2D | undefined {
  const bounds = canvas.getBoundingClientRect();
  if (bounds.width < 1 || bounds.height < 1) return undefined;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.round(bounds.width * dpr);
  const height = Math.round(bounds.height * dpr);
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
  }
  const context = canvas.getContext("2d");
  context?.setTransform(dpr, 0, 0, dpr, 0, 0);
  return context ?? undefined;
}

function drawMotionField() {
  const canvas = $("#motion-canvas") as HTMLCanvasElement;
  const context = fitCanvas(canvas);
  if (!context) return;
  const { width, height } = canvas.getBoundingClientRect();
  context.clearRect(0, 0, width, height);
  context.fillStyle = "#122021";
  context.fillRect(0, 0, width, height);

  if (state.metrics.mode === "MARBLES") {
    $("#view-kind").textContent = "SEALED SHAKE BOX";
    drawShakeBox(context, width, height, revealedGameCount(state.metrics, isFresh()) !== undefined);
    return;
  }

  const padX = Math.max(42, width * 0.075);
  const centerX = width / 2;
  const centerY = height * 0.5;
  const horizontalRoom = width - padX * 2;
  // A side-on tube rotates only in the screen plane. Fit one fixed physical
  // length for the steepest view so the tube never appears to telescope.
  const verticalHalfRoom = Math.max(1, height / 2 - 10);
  const tubeRadius = Math.max(9, Math.min(21, verticalHalfRoom / 10));
  const ballRadius = tubeRadius * 0.85;
  const tubeWidth = Math.min(horizontalRoom, tubeRadius * 18);
  const usable = Math.max(1, tubeWidth - ballRadius * 2);
  const live = isFresh() && Boolean(state.latestMotion);
  const motion = state.latestMotion;
  const cavityMm = Math.max(100, Math.min(2000, Number(state.metrics.cavityMm ?? 1000)));
  const sine = Math.max(-1, Math.min(1, live ? (motion?.sinAlpha ?? 0) : 0));
  const angle = tubeScreenAngle(sine);
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const [leftEnd, rightEnd] = tubeSideViewEndpoints(centerX, centerY, tubeWidth, sine);
  const project = (along: number, across = 0): [number, number] => {
    const u = (along + tubeWidth / 2) / tubeWidth;
    return [
      leftEnd[0] + (rightEnd[0] - leftEnd[0]) * u - across * sin,
      leftEnd[1] + (rightEnd[1] - leftEnd[1]) * u + across * cos,
    ];
  };
  const ballU = motion ? Math.max(0, Math.min(1, motion.xMm / cavityMm)) : .5;
  const ballAlong = (ballU - .5) * usable;
  const [ballX, ballY] = project(ballAlong);

  // Measuring grid and vertical centerline
  context.save();
  context.strokeStyle = "rgba(172, 207, 194, .08)";
  context.lineWidth = 1;
  for (let i = 0; i <= 8; i++) {
    const x = padX + tubeWidth * i / 8;
    context.beginPath(); context.moveTo(x, 28); context.lineTo(x, height - 32); context.stroke();
  }
  context.restore();

  $("#view-kind").textContent = "Side view";

  // Fixed world-level reference. The track changes angle in the screen plane only.
  context.save();
  context.setLineDash([3, 7]);
  context.strokeStyle = "rgba(193,219,204,.16)";
  context.lineWidth = 1;
  context.beginPath(); context.moveTo(padX, centerY); context.lineTo(width - padX, centerY); context.stroke();
  context.setLineDash([]);
  context.fillStyle = "rgba(152,180,164,.56)";
  context.font = "10px Consolas, monospace"; context.textAlign = "left";
  context.fillText("LEVEL", padX, centerY - 8);
  context.restore();

  // Tube body, glass rim and end caps
  const gradient = context.createLinearGradient(0, -tubeRadius * 1.5, 0, tubeRadius * 1.5);
  gradient.addColorStop(0, "rgba(166, 217, 201, .19)");
  gradient.addColorStop(.22, "rgba(99, 155, 143, .075)");
  gradient.addColorStop(.65, "rgba(71, 118, 112, .12)");
  gradient.addColorStop(1, "rgba(177, 205, 174, .18)");
  context.save();
  context.translate(centerX, centerY);
  context.rotate(angle);
  const track = new Path2D();
  track.roundRect(-tubeWidth / 2, -tubeRadius, tubeWidth, tubeRadius * 2, tubeRadius);
  context.fillStyle = gradient;
  context.fill(track);
  context.strokeStyle = "rgba(177, 211, 195, .42)";
  context.lineWidth = 1.3;
  context.stroke(track);
  context.strokeStyle = "rgba(218, 236, 215, .14)";
  context.beginPath(); context.moveTo(-tubeWidth / 2 + tubeRadius, -tubeRadius * .42); context.lineTo(tubeWidth / 2 - tubeRadius, -tubeRadius * .42); context.stroke();
  context.fillStyle = "rgba(215, 190, 144, .72)";
  context.fillRect(-tubeWidth / 2 + 14, tubeRadius + 8, 1.5, 6);
  context.fillRect(tubeWidth / 2 - 14, tubeRadius + 8, 1.5, 6);
  context.restore();

  const start = project(-usable / 2, tubeRadius + 25);
  const middle = project(0, -(tubeRadius + 25));
  const end = project(usable / 2, tubeRadius + 25);
  context.fillStyle = "rgba(202, 217, 204, .54)";
  context.font = "11px 'Bahnschrift', 'Segoe UI', sans-serif";
  const labelY = (y: number) => Math.max(24, Math.min(height - 14, y));
  context.textAlign = "left";
  context.fillText("0 mm", start[0], labelY(start[1]));
  context.textAlign = "right";
  context.fillText(`${Math.round(cavityMm)} mm`, end[0], labelY(end[1]));
  context.textAlign = "center";
  context.fillStyle = "rgba(202, 217, 204, .34)";
  context.fillText(`${Math.round(cavityMm / 2)} mm`, middle[0], labelY(middle[1]));

  // Screen-down gravity is a fixed side-view reference, never a camera rotation.
  const gravityX = width - 32;
  const gravityY = 60;
  context.strokeStyle = "rgba(226,173,109,.86)";
  context.fillStyle = "rgba(226,173,109,.9)";
  context.lineWidth = 1.5;
  context.beginPath(); context.moveTo(gravityX, gravityY); context.lineTo(gravityX, gravityY + 27); context.stroke();
  context.beginPath(); context.moveTo(gravityX - 5, gravityY + 20); context.lineTo(gravityX, gravityY + 27); context.lineTo(gravityX + 5, gravityY + 20); context.stroke();
  context.font = "10px Consolas, monospace"; context.textAlign = "center";
  context.fillText("g ↓", gravityX, gravityY - 8);

  if (!live || !motion) {
    return;
  }

  // A fading trail maps only measured x positions from the recent device stream.
  const path = state.history.slice(-32);
  for (let i = 1; i < path.length; i++) {
    const previous = path[i - 1]; const current = path[i];
    const px = project((Math.max(0, Math.min(1, previous.xMm / cavityMm)) - .5) * usable, tubeRadius * .4);
    const cx = project((Math.max(0, Math.min(1, current.xMm / cavityMm)) - .5) * usable, tubeRadius * .4);
    const age = i / path.length;
    context.strokeStyle = `rgba(86, 215, 189, ${age * .25})`;
    context.lineWidth = 1 + age * 4;
    context.beginPath(); context.moveTo(px[0], px[1]); context.lineTo(cx[0], cx[1]); context.stroke();
  }

  const shadow = context.createRadialGradient(ballX, ballY + ballRadius * .7, 2, ballX, ballY + ballRadius * .72, ballRadius * 1.12);
  shadow.addColorStop(0, "rgba(0,0,0,.38)"); shadow.addColorStop(1, "rgba(0,0,0,0)");
  context.fillStyle = shadow; context.beginPath(); context.ellipse(ballX, ballY + ballRadius * .75, ballRadius * 1.15, ballRadius * .39, 0, 0, Math.PI * 2); context.fill();
  const ball = context.createRadialGradient(ballX - ballRadius * .42, ballY - ballRadius * .48, ballRadius * .06, ballX, ballY, ballRadius);
  ball.addColorStop(0, "#f4f1dc"); ball.addColorStop(.13, "#cbd9c7"); ball.addColorStop(.37, "#6f8b82"); ball.addColorStop(.7, "#344946"); ball.addColorStop(1, "#172827");
  context.fillStyle = ball; context.beginPath(); context.arc(ballX, ballY, ballRadius, 0, Math.PI * 2); context.fill();
  context.strokeStyle = "rgba(231, 240, 222, .7)"; context.lineWidth = 1; context.stroke();
  const glint = context.createRadialGradient(ballX - ballRadius * .38, ballY - ballRadius * .46, 1, ballX - ballRadius * .38, ballY - ballRadius * .46, ballRadius * .47);
  glint.addColorStop(0, "rgba(255,255,231,.62)"); glint.addColorStop(1, "rgba(255,255,231,0)");
  context.fillStyle = glint; context.beginPath(); context.arc(ballX, ballY, ballRadius, 0, Math.PI * 2); context.fill();
  context.fillStyle = "rgba(34, 49, 45, .72)"; context.beginPath(); context.arc(ballX + ballRadius * .14, ballY + ballRadius * .08, 1.5, 0, Math.PI * 2); context.fill();

}

function drawShakeBox(context: CanvasRenderingContext2D, width: number, height: number, revealed: boolean) {
  const centerX = width * .5;
  const centerY = height * .53;
  const boxWidth = Math.min(width * .46, 420);
  const boxHeight = Math.min(height * .34, 152);
  const depth = Math.min(58, boxWidth * .14);
  const left = centerX - boxWidth / 2;
  const right = centerX + boxWidth / 2;
  const top = centerY - boxHeight / 2;
  const bottom = centerY + boxHeight / 2;
  const polygon = (points: Array<[number, number]>, fill: string | CanvasGradient, stroke: string) => {
    context.beginPath();
    points.forEach(([x, y], i) => i ? context.lineTo(x, y) : context.moveTo(x, y));
    context.closePath();
    context.fillStyle = fill;
    context.fill();
    context.strokeStyle = stroke;
    context.lineWidth = 1.2;
    context.stroke();
  };

  // A solid, closed shell keeps every ball hidden until the board confirms reveal.
  polygon([[left, top], [right, top], [right + depth, top - depth * .55], [left + depth, top - depth * .55]], "#273330", "rgba(209,225,211,.34)");
  polygon([[right, top], [right + depth, top - depth * .55], [right + depth, bottom - depth * .55], [right, bottom]], "#182321", "rgba(209,225,211,.28)");
  const front = context.createLinearGradient(left, top, right, bottom);
  front.addColorStop(0, "#3a4540");
  front.addColorStop(.48, "#26312e");
  front.addColorStop(1, "#1b2624");
  polygon([[left, top], [right, top], [right, bottom], [left, bottom]], front, "rgba(209,225,211,.48)");

  context.strokeStyle = "rgba(230,237,218,.14)";
  context.lineWidth = 1;
  context.beginPath(); context.moveTo(left + 14, top + 12); context.lineTo(right - 14, top + 12); context.stroke();
  context.beginPath(); context.moveTo(left + 14, bottom - 12); context.lineTo(right - 14, bottom - 12); context.stroke();
  const latchWidth = Math.min(94, boxWidth * .22);
  context.fillStyle = revealed ? "rgba(98,214,189,.18)" : "rgba(0,0,0,.22)";
  context.strokeStyle = revealed ? "rgba(98,214,189,.68)" : "rgba(226,173,109,.52)";
  context.beginPath(); context.roundRect(centerX - latchWidth / 2, centerY - 11, latchWidth, 22, 4); context.fill(); context.stroke();
  context.fillStyle = revealed ? "#80deca" : "#d7ae7b";
  context.beginPath(); context.arc(centerX, centerY, 3, 0, Math.PI * 2); context.fill();

  for (const x of [left + 13, right - 13]) for (const y of [top + 13, bottom - 13]) {
    context.fillStyle = "rgba(213,225,214,.46)";
    context.beginPath(); context.arc(x, y, 2, 0, Math.PI * 2); context.fill();
  }
  context.fillStyle = "rgba(183,199,188,.7)";
  context.font = "10px Consolas, monospace";
  context.textAlign = "center";
  context.fillText(revealed ? "ROUND COMPLETE" : "COUNT SEALED", centerX, bottom + 29);
}
function drawSparklines() {
  if (state.metrics.mode === "MARBLES") return;
  drawSparkline(angleCanvas, (sample) => sample.sinAlpha, "#d9aa70", -1, 1);
  drawSparkline(velocityCanvas, (sample) => sample.velocityMps, "#62d6bd");
}

function drawSparkline(canvas: HTMLCanvasElement, select: (sample: MotionSample) => number, color: string, fixedMin?: number, fixedMax?: number) {
  const history = state.history.filter((sample) => Date.now() - sample.at < 3000);
  const values = history.map(select);
  const min = fixedMin ?? Math.min(...values, -.04);
  const max = fixedMax ?? Math.max(...values, .04);
  drawSparklineValues(canvas, values, color, min, max);
}

function drawSparklineValues(canvas: HTMLCanvasElement, values: number[], color: string, min: number, max: number) {
  const context = fitCanvas(canvas);
  if (!context) return;
  const { width, height } = canvas.getBoundingClientRect();
  context.clearRect(0, 0, width, height);
  context.strokeStyle = "rgba(176, 202, 190, .11)";
  context.lineWidth = 1;
  context.beginPath(); context.moveTo(0, height / 2); context.lineTo(width, height / 2); context.stroke();
  if (values.length < 2) return;
  context.beginPath();
  values.forEach((value, index) => {
    const x = index / (values.length - 1) * width;
    const normalized = (value - min) / Math.max(.0001, max - min);
    const y = height - 3 - normalized * (height - 6);
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.strokeStyle = color; context.lineWidth = 1.7; context.lineJoin = "round"; context.lineCap = "round"; context.stroke();
  const last = values[values.length - 1];
  const y = height - 3 - ((last - min) / Math.max(.0001, max - min)) * (height - 6);
  context.fillStyle = color; context.beginPath(); context.arc(width - 1, y, 2.5, 0, Math.PI * 2); context.fill();
}

for (const button of modeButtons) {
  button.addEventListener("click", () => requestCommand(button.dataset.command ?? "", button.dataset.mode as Mode));
}
$("#gain-button").addEventListener("click", () => requestCommand("g"));
$("#bounce-button").addEventListener("click", () => requestCommand("b"));
$("#cavity-plus").addEventListener("click", () => requestCommand("+"));
$("#cavity-minus").addEventListener("click", () => requestCommand("-"));
for (const button of document.querySelectorAll<HTMLButtonElement>(".guess-button")) {
  button.addEventListener("click", () => chooseGameGuess(Number(button.dataset.guess)));
}
$("#game-new-round").addEventListener("click", () => requestCommand("n"));
$("#game-reveal").addEventListener("click", () => requestCommand("v"));
portSelect.addEventListener("change", () => {
  if (!state.connected) setConnection("Ready to connect", portSelect.value, portSelect.value ? "Port selected" : "Connect an ESP32 to begin");
});
connectButton.addEventListener("click", () => state.connected ? void disconnectDevice() : void connectDevice());
refreshButton.addEventListener("click", () => void refreshPorts());
demoButton.addEventListener("click", toggleDemo);

window.addEventListener("keydown", (event) => {
  if (event.metaKey || event.ctrlKey || event.altKey || event.repeat) return;
  const target = event.target as HTMLElement | null;
  if (target?.matches("input, select, textarea, [contenteditable='true']")) return;
  const key = event.key.toLowerCase();
  const modeKeys: Record<string, Mode> = { l: "LIGHTSABER", t: "TEXTURED", s: "SLIDING", m: "MARBLES" };
  if (modeKeys[key]) requestCommand(key, modeKeys[key]);
  else if (key === "n" || key === "v") requestCommand(key);
  else if (key === "g" || key === "b") requestCommand(key);
  else if (key === "+" || key === "=") requestCommand("+");
  else if (key === "-" || key === "_") requestCommand("-");
});

window.addEventListener("resize", () => render());
window.setInterval(() => {
  const fresh = isFresh();
  document.body.classList.toggle("is-stale", state.connected && !fresh);
  if (!fresh && !state.demo) {
    $("#stream-status").textContent = state.connected ? "Signal stale" : "No signal";
    $("#firmware-state").textContent = state.connected ? "waiting" : "idle";
  }
  if (fresh !== previousFresh) {
    previousFresh = fresh;
    render();
  } else {
    syncControls();
  }
}, 250);

if (isTauri) {
  void (async () => {
    await Promise.all([
      listen<string>("device-line", (event) => onTelemetry(event.payload)),
      listen<ConnectionEvent>("connection", (event) => {
        const { state: connectionState, port, detail } = event.payload;
        setConnection(connectionState, port || state.port, detail);
        if (connectionState.trim().toLowerCase() !== "connected") {
          state.lastPacketAt = undefined;
          state.lastSignalAt = undefined;
          state.metrics = { ...state.metrics, gameRound: undefined, gameRevealed: undefined, gameCount: undefined };
          state.userGuess = undefined;
          state.history = [];
          state.latestMotion = undefined;
          if (state.pending) clearPending("Disconnected before confirmation");
          else if (connectionState.trim().toLowerCase() === "disconnected") setFeedback("Disconnected");
        }
        render();
      }),
    ]);
    await refreshPorts();
  })();
} else {
  demoButton.hidden = false;
  setConnection("Browser preview", "", "No board connected");
  setFeedback("Connect a board in the desktop app, or start the labeled sample demo.");
  render();
}

window.requestAnimationFrame(paintFrame);
