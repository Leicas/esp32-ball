#pragma once
#include <stdint.h>
#include "rolling_response.h"

// ─────────────────────────────────────────────────────────────────────────────
// Physics constants  (shared between all targets)
// ─────────────────────────────────────────────────────────────────────────────

// Length of the real handle the virtual tube lives in. A7 enclosure: 220 mm
// overall before the bottom was shortened 10.5 mm (hardware/.../mechanical).
// The ball cavity and the rainstick are both sized to this, so what is felt
// matches what is held. '+' / '-' still resize the ball cavity at run time.
static constexpr float TUBE_LENGTH_M = 0.20f;
static constexpr float G_FACTOR = 7.0f;       // g/1.4 — solid sphere [m/s²]
static constexpr float FRICTION_MU = 0.20f;   // Coulomb coefficient (sliding)
// Rolling resistance. Feel settings, much larger than steel on smooth plastic
// (~0.005): an ideal ball crosses the 0.20 m tube in 0.57 s at 10°, which reads
// as too abrupt. With these it takes about 0.75 s and still arrives at 0.43 m/s,
// fast enough for a solid hit (stronger settings made the end stops vanish).
static constexpr float ROLLING_RESISTANCE = 0.01f; // constant part, fraction of the normal load
static constexpr float ROLLING_DRAG = 2.0f;        // speed-proportional part [1/s]
static constexpr int WTABLE_SIZE = 30;        // rolling texture spatial period [mm]
// Perceptual tuning for a metal ball in a plastic tube: a shorter return to
// the end stop. This is a feel setting, not a measured material coefficient.
static constexpr float RESTITUTION_E = 0.30f; // outgoing / incoming wall speed

#if defined(A7_BOARD)
static constexpr int PHYS_HZ = 2000; // microsecond-timer driven, independent of RTOS tick
#else
static constexpr int PHYS_HZ = 1000;
#endif
static constexpr float PHYS_H = 1.0f / PHYS_HZ; // time step [s]

// ─────────────────────────────────────────────────────────────────────────────
// Haptic parameters
// ─────────────────────────────────────────────────────────────────────────────

static constexpr float HAPTIC_MAX_AMP = 0.80f;   // cap rolling amplitude [0–1]
static constexpr float HAPTIC_MIN_SPD = ROLLING_ONSET_MPS;   // silence below this speed [m/s]
static constexpr uint32_t IMPACT_MS = 9;         // impact pulse max duration [ms]
static constexpr uint32_t IMPACT_MIN_MS = 2;     // impact pulse min duration [ms]
// In a 0.20 m tube the ball rarely exceeds 1 m/s, so full strength is reached
// at 0.8 m/s (was 2.0, sized for a metre-long cavity).
static constexpr float HAPTIC_IMPACT_REF = 0.8f; // speed [m/s] → full impact amplitude

// ─────────────────────────────────────────────────────────────────────────────
// Marble box simulation (3D)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr int MARBLE_COUNT = 5;          // storage capacity; active count is set at runtime
static constexpr int MARBLE_DEFAULT_COUNT = 3;
static constexpr float MARBLE_BOX_W = 0.050f;      // box width  [m] (X)
static constexpr float MARBLE_BOX_H = 0.025f;      // box height [m] (Y)
static constexpr float MARBLE_BOX_D = 0.025f;      // box depth  [m] (Z)
static constexpr float MARBLE_RADIUS = 0.005f;    // 10 mm identical steel marbles [m]
static constexpr float MARBLE_MASS = 1.0f;        // relative units; identical marbles
static constexpr float MARBLE_WALL_RESTITUTION = 0.35f; // damped matchbox-like rattle
static constexpr float MARBLE_PAIR_RESTITUTION = 0.18f; // inter-marble hits are duller
static constexpr float MARBLE_PAIR_HAPTIC_GAIN = 0.30f; // lighter cue than a box-wall hit
static constexpr float MARBLE_WALL_FRICTION = 0.20f; // dissipates tangential motion under support
static constexpr float MARBLE_REST_SPEED = 0.015f; // settle tiny contact chatter [m/s]
static constexpr float MARBLE_MIN_IMPACT_SPEED = 0.025f; // suppress microimpact cues [m/s]
static constexpr float MARBLE_IMPACT_REF = 0.50f;  // summed relative-mass impulse → full haptic amplitude

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry periods
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint32_t TELEM_PERIOD_MS = 17;   // serial stream  ~58 Hz
static constexpr uint32_t BLE_PERIOD_MS = 100;    // BLE notify     10 Hz
static constexpr uint32_t DEBUG_PERIOD_MS = 1000; // debug print     1 Hz

// ─────────────────────────────────────────────────────────────────────────────
// Board pin assignments
// ─────────────────────────────────────────────────────────────────────────────

#if defined(A7_BOARD)
// ── Rolling Stone A7 custom PCB (see hardware/firmware-interface.md) ─────────
//   LSM6DS3TR-C : SPI SCK=IO14 MOSI=IO15 MISO=IO18 CS=IO19 (raw accel+gyro, fused on-chip here)
//   PCA9632 RGB : I2C SDA=IO6 SCL=IO7 @0x62
//   4× MAX98357A: TDM BCLK=IO21 FS=IO22 DATA=IO23, SD_MODE=HAPTIC_EN IO5; slot 0 LFi(J3) / 2 HF(J5)
//   Buttons     : MODE=IO0, GAIN=IO1 (active low); USB_500 = IO17

static constexpr int PIN_SDA = 6;
static constexpr int PIN_SCL = 7;
static constexpr int PIN_IMU_SCK = 14;
static constexpr int PIN_IMU_MOSI = 15;
static constexpr int PIN_IMU_MISO = 18;
static constexpr int PIN_IMU_CS = 19;
static constexpr int PIN_TDM_BCLK = 21;
static constexpr int PIN_TDM_FS = 22;
static constexpr int PIN_TDM_DATA = 23;
static constexpr int PIN_HAPTIC_EN = 5;
static constexpr int PIN_USB_500 = 17;
static constexpr int PIN_BTN_MODE = 0;
static constexpr int PIN_BTN_GAIN = 1;

static constexpr uint32_t TDM_RATE = 48000; // 24 frames per 0.5 ms physics tick

// IMU axis along the virtual tube (0 = X, 1 = Y, 2 = Z) and its sign — Y runs along the A7 board
static constexpr int A7_TUBE_AXIS = 1;
static constexpr float A7_TUBE_SIGN = 1.0f;

// Output levels, fraction of full scale (MAX98357A 12 dB: ~5.7 V peak at FS).
// TacHammer Drake 8 Ω: +V drives the hammer into its impact end ("tick", 8.6 ms recommended),
// −V into the repelling magnet. So impacts are positive pulses, rolling rumble stays negative.
// 0.40 FS square ticks (~0.3 A step) browned the board out on USB power; 0.25 FS with 1 ms
// raised-cosine edges (A7_IMPACT_EDGE_MS) is the known-safe level.
// 0.40 FS browns out on USB even with soft edges (7 resets in 90 s, 2026-10-08); fine on battery?
#if defined(A7_THREEWAY)
static constexpr float A7_IMPACT_FS = 1.0f; // full-scale asymmetric impacts, user-selected
#else
#if defined(A7_THREEWAY)
static constexpr float A7_IMPACT_FS = 1.0f; // full-scale positive impacts
#else
static constexpr float A7_IMPACT_FS = 0.25f;  // ~1.4 V tick, ≤ 9 ms
#endif
#endif
static constexpr int A7_IMPACT_EDGE_MS = 1;
static constexpr float A7_ROLL_FS = 0.25f;    // rolling rumble on LFi (negative half only)
static constexpr float A7_HF_ROLL_FS = 0.20f; // rolling texture on HF

#elif defined(HBRIDGE)
// ── H-bridge target (CodeCell ESP32-C6 Drive) ────────────────────────────────
//   BNO085  : I2C SDA=IO8, SCL=IO9  (power via IO11)
//   H-bridge: IN1=IO22 (plus/impact), IN2=IO21 (minus/rumble)

static constexpr int PIN_SDA = 8;
static constexpr int PIN_SCL = 9;
static constexpr int PIN_SENS_ON = 18; // sensor LDO enable  (CodeCell SENS_ON_PIN)
static constexpr int PIN_LED_ON = 20;  // LED enable         (CodeCell LED_ON_PIN)

// Rumble (rolling): only the minus pin (IN2) — unipolar negative drive
// Impact          : only the plus  pin (IN1) — short positive pulse
static constexpr int PIN_HB_PLUS = 22;  // IN1 — impact  (positive)
static constexpr int PIN_HB_MINUS = 21; // IN2 — rumble  (negative)

// LEDC PWM — 20 kHz carrier, 8-bit duty resolution
static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t PWM_BITS = 8;
static constexpr uint32_t PWM_MAX = (1U << PWM_BITS) - 1; // 255

#else
// ── I2S audio target (Adafruit Feather ESP32-C6) ─────────────────────────────
//   BNO085 : STEMMA QT — SDA=IO19, SCL=IO18, power enable=IO20
//   I2S amp: BCLK=IO21 (SCK header), LRCLK=IO22 (MOSI header), DOUT=IO23 (MISO header)
//   NeoPixel: IO9

static constexpr int PIN_SDA = 19;
static constexpr int PIN_SCL = 18;
static constexpr int PIN_SENS_ON = 20; // STEMMA QT power enable
static constexpr int PIN_NEOPIXEL = 9;

static constexpr int PIN_I2S_BCLK = 21;  // SCK  header
static constexpr int PIN_I2S_LRCLK = 22; // MOSI header
static constexpr int PIN_I2S_DOUT = 23;  // MISO header

static constexpr uint32_t AUDIO_RATE = 22050; // [Hz]

#endif // board
