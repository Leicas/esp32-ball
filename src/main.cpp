/*
 * Virtual Rolling Stone — Feather ESP32-C6
 *
 * Reproduces the apparatus from:
 *   Yao & Hayward, "An Experiment on Length Perception with a Virtual Rolling Stone"
 *   Eurohaptics 2006, pp. 325-330.
 *
 * Hardware:
 *   BNO085 via STEMMA QT  →  measures tube inclination (replaces ADXL210)
 *   I2S amp + actuator    →  haptic output (replaces custom DAC + coil actuator)
 *
 * Physics (from paper):
 *   Rolling : ẍ = (g/1.4)·sin(α) ≈ 7.0·sin(α)          [Eq. 2]
 *   Sliding : ẍ = g·sin(α) − g·µ·sgn(sin(α))·cos(α)     [Eq. 3]
 *   sin(α) extracted from quaternion: 2·(w·y − x·z)
 *   (gravity component along sensor body-X axis = tube long axis)
 *
 * Rolling noise synthesis (paper §5.1):
 *   Source: positive arch of sine wave repeated every WTABLE_SIZE mm of travel.
 *   Index : i = pos_mm mod WTABLE_SIZE  (pitch rises naturally with velocity).
 *   Amplitude: proportional to |velocity|.
 *
 * Impact synthesis:
 *   One-sample pulse, amplitude ∝ |velocity at wall|.
 *
 * I2S pins — adjust PIN_I2S_* to match your amp wiring.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>
#include <math.h>

// ── Hardware pins ─────────────────────────────────────────────────────────────

#define BNO08X_RESET -1

static constexpr int PIN_SDA = 19;
static constexpr int PIN_SCL = 18;
static constexpr int PIN_STEMMA_POWER = 20;
static constexpr int PIN_NEOPIXEL = 9;
static constexpr int NEO_COUNT = 1;

// I2S output — using SPI header pins (GPIO 4-7 are JTAG on ESP32-C6)
// Board labels: SCK=IO21  MOSI=IO22  MISO=IO23
static constexpr gpio_num_t PIN_I2S_BCLK = (gpio_num_t)21;  // SCK  header pin
static constexpr gpio_num_t PIN_I2S_LRCLK = (gpio_num_t)22; // MOSI header pin
static constexpr gpio_num_t PIN_I2S_DOUT = (gpio_num_t)23;  // MISO header pin

// ── Physics & audio parameters ────────────────────────────────────────────────

static constexpr uint32_t SAMPLE_RATE = 8000;  // audio/physics rate [Hz]
static constexpr float H = 1.0f / SAMPLE_RATE; // time step [s]
static constexpr float G_FACTOR = 7.0f;        // g/1.4 — solid sphere [m/s²]
static constexpr float D_CAVITY = 1.00f;       // virtual tube length [m]
static constexpr float FRICTION_MU = 0.20f;    // Coulomb coeff (sliding only)
static constexpr int WTABLE_SIZE = 30;         // wavetable size [mm per repeat]
static constexpr int AUDIO_CHUNK = 64;         // frames per I2S write (~8 ms)
static constexpr float GAIN = 1.00f;           // output amplitude 0–1

// Switch between ROLLING (rolling noise + impact) and SLIDING (impact only).
// Matches the two experimental conditions in the paper.
enum class SimMode : uint8_t
{
    ROLLING,
    SLIDING
};
static constexpr SimMode SIM_MODE = SimMode::ROLLING;

// ── Objects ───────────────────────────────────────────────────────────────────

Adafruit_NeoPixel pixel(NEO_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

static i2s_chan_handle_t tx_chan = nullptr;

// ── Physics state ─────────────────────────────────────────────────────────────

static float x_pos = D_CAVITY / 2.0f; // ball position  [m], starts at centre
static float x_vel = 0.0f;            // ball velocity  [m/s]
static float accel_k = 0.0f;          // accel at step k (trapezoidal carry)
static float sin_alpha = 0.0f;        // gravity component along tube axis

// Rolling noise wavetable — positive arch of sine
static float wtable[WTABLE_SIZE];

// Drake LFi impact state — sustains the 8.6 ms tick pulse across chunks
// Datasheet §3.1.1: single tick = +V rectangular pulse, 8.6 ms duration.
// At 8 kHz that is 69 samples.
static constexpr int IMPACT_SAMPLES = (int)(0.0086f * SAMPLE_RATE + 0.5f); // 69
static int impact_left = 0;                                                // samples remaining in current pulse
static float impact_amp = 0.0f;

// BNO085 state (updated from sensor events)
static float qw = 1.0f, qx = 0.0f, qy = 0.0f, qz = 0.0f;
static float heading_acc = NAN;

// Mono audio buffer
static int16_t audio_buf[AUDIO_CHUNK];

// Housekeeping
static uint8_t neo_hue = 0;
static uint32_t pkt_cnt = 0;
static uint32_t last_hz_ms = 0;

// ── NeoPixel helpers ──────────────────────────────────────────────────────────

static void neoSet(uint8_t r, uint8_t g, uint8_t b)
{
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

static void neoBlink(uint8_t r, uint8_t g, uint8_t b, int n, uint32_t ms = 120)
{
    for (int i = 0; i < n; i++)
    {
        neoSet(r, g, b);
        delay(ms);
        neoSet(0, 0, 0);
        delay(ms);
    }
    neoSet(r, g, b);
}

// ── I2C scan ──────────────────────────────────────────────────────────────────

static void i2cScan()
{
    Serial.println("Scanning I2C...");
    int found = 0;
    for (uint8_t a = 1; a < 127; a++)
    {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf("  0x%02X\n", a);
            found++;
        }
    }
    if (!found)
        Serial.println("  None found.");
}

// ── BNO085 ────────────────────────────────────────────────────────────────────

static void setReports()
{
    // 2500 µs = 400 Hz — full 9-DOF fusion, no yaw drift
    if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 2500))
    {
        Serial.println("Could not enable ARVR stabilized RV");
    }
}

// sin(α) = gravity projected onto body X (tube long axis).
// Derived from rotation matrix element R[2][0] = 2(xz−wy):
//   g_body_x = −R[2][0] = 2(wy − xz)
static inline void updateSinAlpha()
{
    sin_alpha = 2.0f * (qw * qy - qx * qz);
}

// ── Rolling noise wavetable ───────────────────────────────────────────────────
// Positive arch of sine: sin(π·i/(N−1)) for i ∈ [0, N).
// Indexed by ball position in mm, so pitch ∝ velocity — exactly like a real ball.

static void initWavetable()
{
    // Negative arch: drives hammer into repelling field → blunt "pulse" feel.
    // Keeps rolling sensation soft and distinct from the sharp +V impact tick.
    // (Datasheet §3.1.1: +V = sharp tick, -V = blunt pulse.)
    for (int i = 0; i < WTABLE_SIZE; i++)
    {
        wtable[i] = -sinf((float)M_PI * i / (WTABLE_SIZE - 1));
    }
}

// ── I2S initialisation ────────────────────────────────────────────────────────

static void initI2S()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws = PIN_I2S_LRCLK,
            .dout = PIN_I2S_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    Serial.printf("I2S : BCLK=IO%d  LRCLK=IO%d  DOUT=IO%d  %u Hz  OK\n",
                  (int)PIN_I2S_BCLK, (int)PIN_I2S_LRCLK, (int)PIN_I2S_DOUT,
                  SAMPLE_RATE);
}

// ── Physics + audio synthesis ─────────────────────────────────────────────────
// Fills audio_buf with AUDIO_CHUNK mono frames.
// Each sample advances the physics simulation by one step H.
// Trapezoidal integration matches the paper's finite-difference scheme.

static void fillAudioBuf()
{
    for (int i = 0; i < AUDIO_CHUNK; i++)
    {
        // ── Acceleration at current step ──────────────────────────────────────
        float accel_new;

        if (SIM_MODE == SimMode::ROLLING)
        {
            // Eq. 2: ẍ = (g/1.4)·sin(α)
            accel_new = G_FACTOR * sin_alpha;
        }
        else
        {
            // Eq. 3: sliding with Coulomb friction.
            // Use sgn(sin α) in place of sgn(ẋ) to avoid spurious chattering
            // (as noted in the paper).
            float s = sin_alpha;
            float s2 = s * s;
            float c = sqrtf(fmaxf(1.0f - s2, 0.0f)); // cos(α) ≥ 0

            // Static friction threshold: tan(α) > µ  ↔  sin²α > µ²·cos²α
            if (s2 <= FRICTION_MU * FRICTION_MU * (1.0f - s2))
                accel_new = 0.0f;
            else
                accel_new = 9.8f * s - 9.8f * FRICTION_MU * copysignf(c, s);
        }

        // ── Trapezoidal integration (paper §5.1) ──────────────────────────────
        float v_new = x_vel + H * (accel_k + accel_new) * 0.5f;
        float x_new = x_pos + H * (x_vel + v_new) * 0.5f;
        accel_k = accel_new;

        // ── Cue synthesis ─────────────────────────────────────────────────────
        float sample_f = 0.0f;

        if (x_new <= 0.0f || x_new >= D_CAVITY)
        {
            // Impact: start Drake LFi tick pulse (§3.1.1 — 8.6 ms +V pulse).
            impact_amp  = 1.0f;
            impact_left = IMPACT_SAMPLES; // 69 samples @ 8 kHz
            v_new = 0.0f;
            x_new = (x_new <= 0.0f) ? 0.0f : D_CAVITY;
        }

        if (impact_left > 0)
        {
            // Sustain the rectangular tick pulse for its full 8.6 ms duration.
            sample_f = impact_amp;
            impact_left--;
        }
        else if (SIM_MODE == SimMode::ROLLING)
        {
            // Rolling noise: wavetable[pos_mm mod 30], amplitude ∝ speed.
            // Index advances by (v·1000/SAMPLE_RATE) mm per step, so frequency
            // rises linearly with velocity — exactly like a real rolling object.
            int idx = abs((int)(x_new * 1000.0f)) % WTABLE_SIZE;
            float speed = fminf(fabsf(v_new) / 2.0f, 1.0f);
            sample_f = wtable[idx] * speed;
        }
        // (SLIDING without impact → silence between walls, just like the paper)

        x_vel = v_new;
        x_pos = x_new;

        audio_buf[i] = (int16_t)(sample_f * GAIN * 32767.0f);
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────

void setup()
{
    pinMode(PIN_STEMMA_POWER, OUTPUT);
    digitalWrite(PIN_STEMMA_POWER, HIGH); // power STEMMA QT + NeoPixel

    pixel.begin();
    pixel.setBrightness(40);
    neoBlink(255, 255, 255, 3); // WHITE: firmware alive

    Serial.begin(115200);
    delay(1000);

    Serial.println("\n================================");
    Serial.println("Virtual Rolling Stone  —  Feather ESP32-C6");
    Serial.println("================================");

    initWavetable();

    initI2S();

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    delay(200);

    neoSet(255, 180, 0); // YELLOW: scanning I2C
    i2cScan();

    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire))
    {
        Serial.println("BNO085 not found — check wiring!");
        neoSet(255, 0, 0); // RED: fatal
        while (1)
            delay(10);
    }
    neoBlink(0, 255, 0, 3); // GREEN: sensor found
    Serial.println("BNO085 found!");

    setReports();

    Serial.printf("Physics: cavity=%.0f cm  mode=%s  G=%.1f  mu=%.2f\n",
                  D_CAVITY * 100.0f,
                  SIM_MODE == SimMode::ROLLING ? "rolling" : "sliding",
                  G_FACTOR, FRICTION_MU);
    delay(100);
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop()
{
    // 1. Read BNO085 — drain up to 8 pending events, keep latest quaternion
    if (bno08x.wasReset())
    {
        Serial.println("# sensor reset — re-enabling");
        neoBlink(255, 80, 0, 2, 80); // ORANGE
        setReports();
    }
    for (int n = 0; n < 8 && bno08x.getSensorEvent(&sensorValue); n++)
    {
        if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV)
        {
            qw = sensorValue.un.arvrStabilizedRV.real;
            qx = sensorValue.un.arvrStabilizedRV.i;
            qy = sensorValue.un.arvrStabilizedRV.j;
            qz = sensorValue.un.arvrStabilizedRV.k;
            heading_acc = sensorValue.un.arvrStabilizedRV.accuracy;
            updateSinAlpha();
            pkt_cnt++;
        }
    }

    // 2. NeoPixel: magnetometer calibration quality
    if (isnan(heading_acc) || heading_acc > 1.0f)
        neoSet(200, 0, 0); // RED   — poor  (>57°)
    else if (heading_acc > 0.35f)
        neoSet(255, 100, 0); // ORANGE — medium (20–57°)
    else
    {
        uint32_t c = pixel.ColorHSV((uint16_t)(neo_hue++) * 256, 255, 180);
        pixel.setPixelColor(0, c);
        pixel.show(); // BLUE cycling — good (<20°)
    }

    // 3. Run physics simulation + generate audio samples
    fillAudioBuf();

    // 4. Push to I2S DMA (blocks ~8 ms in steady state — natural rate limiter)
    size_t written;
    i2s_channel_write(tx_chan, audio_buf, sizeof(audio_buf),
                      &written, pdMS_TO_TICKS(100));

    // 5. Debug output — once per second
    uint32_t now = millis();
    if (now - last_hz_ms >= 1000)
    {
        float sensor_hz = pkt_cnt * 1000.0f / (float)(now - last_hz_ms);
        Serial.printf("# sin_a=%+.3f  x=%5.1f mm  v=%+6.3f m/s"
                      "  cal=%.0f°  imu=%.0f Hz\n",
                      sin_alpha,
                      x_pos * 1000.0f,
                      x_vel,
                      isnan(heading_acc) ? 999.f : heading_acc * 57.296f,
                      sensor_hz);
        pkt_cnt = 0;
        last_hz_ms = now;
    }
}
