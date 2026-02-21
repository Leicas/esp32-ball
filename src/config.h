#pragma once
#include <stdint.h>

// ─────────────────────────────────────────────────────────────────────────────
// Physics constants  (shared between all targets)
// ─────────────────────────────────────────────────────────────────────────────

static constexpr float    G_FACTOR       = 7.0f;   // g/1.4 — solid sphere [m/s²]
static constexpr float    FRICTION_MU    = 0.20f;  // Coulomb coefficient (sliding)
static constexpr int      WTABLE_SIZE    = 30;     // rolling texture spatial period [mm]
static constexpr float    RESTITUTION_E  = 0.35f;  // bounce coefficient

static constexpr int      PHYS_HZ        = 1000;   // physics loop rate [Hz]
static constexpr float    PHYS_H         = 1.0f / PHYS_HZ; // time step [s]

// ─────────────────────────────────────────────────────────────────────────────
// Haptic parameters
// ─────────────────────────────────────────────────────────────────────────────

static constexpr float    HAPTIC_MAX_AMP      = 0.80f; // cap rolling amplitude [0–1]
static constexpr float    HAPTIC_MIN_SPD      = 0.05f; // silence below this speed [m/s]
static constexpr uint32_t IMPACT_MS           = 9;     // impact pulse duration [ms]
static constexpr float    HAPTIC_IMPACT_REF   = 3.0f;  // speed [m/s] → full impact amplitude

// ─────────────────────────────────────────────────────────────────────────────
// Telemetry periods
// ─────────────────────────────────────────────────────────────────────────────

static constexpr uint32_t TELEM_PERIOD_MS = 17;    // serial stream  ~58 Hz
static constexpr uint32_t BLE_PERIOD_MS   = 100;   // BLE notify     10 Hz
static constexpr uint32_t DEBUG_PERIOD_MS = 1000;  // debug print     1 Hz

// ─────────────────────────────────────────────────────────────────────────────
// Board pin assignments
// ─────────────────────────────────────────────────────────────────────────────

#ifdef HBRIDGE
// ── H-bridge target (CodeCell ESP32-C6 Drive) ────────────────────────────────
//   BNO085  : I2C SDA=IO8, SCL=IO9  (power via IO11)
//   H-bridge: IN1=IO22 (plus/impact), IN2=IO21 (minus/rumble)

static constexpr int PIN_SDA      =  8;
static constexpr int PIN_SCL      =  9;
static constexpr int PIN_SENS_ON  = 18;   // sensor LDO enable  (CodeCell SENS_ON_PIN)
static constexpr int PIN_LED_ON   = 20;   // LED enable         (CodeCell LED_ON_PIN)

// Rumble (rolling): only the minus pin (IN2) — unipolar negative drive
// Impact          : only the plus  pin (IN1) — short positive pulse
static constexpr int PIN_HB_PLUS  = 22;   // IN1 — impact  (positive)
static constexpr int PIN_HB_MINUS = 21;   // IN2 — rumble  (negative)

// LEDC PWM — 20 kHz carrier, 8-bit duty resolution
static constexpr uint32_t PWM_FREQ = 20000;
static constexpr uint8_t  PWM_BITS = 8;
static constexpr uint32_t PWM_MAX  = (1U << PWM_BITS) - 1; // 255

#else
// ── I2S audio target (Adafruit Feather ESP32-C6) ─────────────────────────────
//   BNO085 : STEMMA QT — SDA=IO19, SCL=IO18, power enable=IO20
//   I2S amp: BCLK=IO21 (SCK header), LRCLK=IO22 (MOSI header), DOUT=IO23 (MISO header)
//   NeoPixel: IO9

static constexpr int PIN_SDA       = 19;
static constexpr int PIN_SCL       = 18;
static constexpr int PIN_SENS_ON   = 20;  // STEMMA QT power enable
static constexpr int PIN_NEOPIXEL  =  9;

static constexpr int PIN_I2S_BCLK  = 21;  // SCK  header
static constexpr int PIN_I2S_LRCLK = 22;  // MOSI header
static constexpr int PIN_I2S_DOUT  = 23;  // MISO header

static constexpr uint32_t AUDIO_RATE = 22050; // [Hz]

#endif // HBRIDGE
