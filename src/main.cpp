/*
 * Virtual Rolling Stone — Feather ESP32-C6 / CodeCell ESP32-C6 Drive
 *
 * Reproduces the apparatus from:
 *   Yao & Hayward, "An Experiment on Length Perception with a Virtual Rolling Stone"
 *   Eurohaptics 2006, pp. 325-330.
 *
 * Two build profiles (select via PlatformIO environment):
 *
 *   adafruit_feather_esp32c6  (default)
 *     BNO085 via STEMMA QT  →  measures tube inclination (I2C SDA=IO19 SCL=IO18)
 *     I2S amp + actuator    →  haptic output  (BCLK=IO21 LRCLK=IO22 DOUT=IO23)
 *
 *   codecell_drive  (-DCODECELL_DRIVE)
 *     CodeCell ESP32-C6 Drive board
 *     Onboard BNO085 (I2C SDA=IO8 SCL=IO9, managed by CodeCell library)
 *     Dual H-bridge via DriveCell (members of CodeCell object):
 *       Drive1: IN1=IO22, IN2=IO21   ← primary haptic actuator
 *       Drive2: IN1=IO2,  IN2=IO3    ← spare / second actuator
 *     Rolling haptic : Drive1.Run(smooth, speed%, flip_ms)
 *                      flip_ms = 15/|v| ms  →  pitch rises with velocity
 *     Impact haptic  : Drive1.Pulse(direction, 9 ms)
 *     NOTE: axis orientation depends on board mounting; swap axes in
 *           Motion_GravityRead()/Motion_LinearAccRead() if the ball rolls the wrong way.
 *
 * Physics (from paper):
 *   Rolling : ẍ = (g/1.4)·sin(α) ≈ 7.0·sin(α)          [Eq. 2]
 *   Sliding : ẍ = g·sin(α) − g·µ·sgn(sin(α))·cos(α)     [Eq. 3]
 *   sin(α) = accel_x / g  (gravity + inertia projected on tube axis)
 *
 * Rolling noise synthesis (paper §5.1):
 *   Feather : wavetable (negative sine arch, 30-sample period) sampled at
 *             22 050 Hz; pitch rises naturally with velocity.
 *   CodeCell: DriveCell.Run() with flip_ms = 15/|v| ms reproduces the same
 *             pitch-velocity law via the H-bridge driver.
 *
 * Impact synthesis:
 *   Feather : positive rectangular pulse, 8.6 ms, via I2S.
 *   CodeCell: DriveCell.Pulse() for 9 ms.
 *
 * Companion app serial protocol:
 *   Firmware → host (both profiles):
 *     "$sin_a,x_mm,v_mps\n"  at ~100 Hz (STREAM_INTERVAL_US)
 *     "# key=val ...\n"       at  1 Hz, and immediately after command replies
 *   Host → firmware (single ASCII char):
 *     'r'  ROLLING mode
 *     's'  SLIDING mode
 *     '+'  cavity +5 cm  (clamps 10 cm – 200 cm)
 *     '-'  cavity −5 cm
 *     '?'  print current parameters
 */

#include <Arduino.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#ifdef CODECELL_DRIVE
#include <CodeCell.h>
// Wire, BNO085, NeoPixel, and power management are handled inside CodeCell.
#else
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_NeoPixel.h>
#include <driver/i2s_std.h>
#endif

// ── Hardware pins (Feather only) ──────────────────────────────────────────────

#ifndef CODECELL_DRIVE
#define BNO08X_RESET -1

static constexpr int PIN_SDA = 19;
static constexpr int PIN_SCL = 18;
static constexpr int PIN_STEMMA_POWER = 20;
static constexpr int PIN_NEOPIXEL = 9;
static constexpr int NEO_COUNT = 1;

// I2S output — using SPI header pins (GPIO 4-7 are JTAG on ESP32-C6)
// Board labels: SCK=IO21  MOSI=IO22  MISO=IO23
static constexpr gpio_num_t PIN_I2S_BCLK = (gpio_num_t)21;
static constexpr gpio_num_t PIN_I2S_LRCLK = (gpio_num_t)22;
static constexpr gpio_num_t PIN_I2S_DOUT = (gpio_num_t)23;
#endif

// ── Physics parameters (shared, fixed) ────────────────────────────────────────

static constexpr float G_FACTOR = 7.0f;     // g/1.4 — solid sphere [m/s²]
static constexpr float FRICTION_MU = 0.20f; // Coulomb coeff (sliding only)
static constexpr int WTABLE_SIZE = 30;      // spatial period [mm per repeat]
static constexpr float RESTITUTION_E = 0.35f; // bounce coefficient (0=no bounce)
static constexpr float IMPACT_VREF = 1.0f;    // impact amplitude at 1 m/s

// Switch between ROLLING (rolling noise + impact) and SLIDING (impact only).
enum class SimMode : uint8_t
{
    ROLLING,
    SLIDING
};

// ── Runtime-adjustable simulation parameters ──────────────────────────────────
// Changed by single-char commands from the companion app (r/s/+/-/?).

static float g_cavity = 1.00f;                // virtual tube length [m]
static SimMode g_sim_mode = SimMode::ROLLING; // ROLLING or SLIDING

// ── Profile-specific synthesis parameters ─────────────────────────────────────

#ifdef CODECELL_DRIVE
static constexpr int PHYS_HZ = 100;          // physics update rate [Hz] (CodeCell.Run() caps at 100 Hz)
static constexpr float H = 1.0f / PHYS_HZ;   // time step [s] = 10 ms
static constexpr float CC_MIN_SPEED = 0.05f; // m/s — stop actuator below this
static constexpr int CC_IMPACT_MS = 1;       // impact pulse duration [steps at 100 Hz = 10 ms] → 1 step ≈ 10 ms
#else
static constexpr uint32_t SAMPLE_RATE = 22050;
static constexpr float H = 1.0f / SAMPLE_RATE;
static constexpr int AUDIO_CHUNK = 128; // frames per I2S write (~5.8 ms)
static constexpr float GAIN = 1.00f;
static constexpr int IMPACT_SAMPLES = (int)(0.0086f * SAMPLE_RATE + 0.5f); // 190
#endif

// ── Companion streaming ────────────────────────────────────────────────────────

static constexpr uint32_t STREAM_INTERVAL_US = 10000; // 100 Hz telemetry to companion app

// ── Objects ───────────────────────────────────────────────────────────────────

#ifdef CODECELL_DRIVE
CodeCell myCodeCell;
// myCodeCell.Drive1 (IO22/IO21) and myCodeCell.Drive2 (IO2/IO3) are
// DriveCell members instantiated by the CodeCell library.
#else
Adafruit_NeoPixel pixel(NEO_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
static i2s_chan_handle_t tx_chan = nullptr;
#endif

// ── Physics state ─────────────────────────────────────────────────────────────

static float x_pos = 0.50f;      // ball position [m] — half of default 1.0 m cavity
static float x_vel = 0.0f;       // ball velocity [m/s]
static float accel_k = 0.0f;     // accel at step k (trapezoidal carry)
static float sin_alpha = 0.0f;   // sin of tube tilt angle (from gravity vector)
static float lin_accel_x = 0.0f; // tube linear accel along axis [m/s^2]

static bool at_left_wall = false;
static bool at_right_wall = false;

// Profile-specific impact / synthesis state

#ifdef CODECELL_DRIVE
static int cc_impact_left = 0; // cooldown steps remaining (don't call Run/Drive during Pulse)
#else
static int impact_left = 0;
static float impact_amp = 0.0f;
static float wtable[WTABLE_SIZE];
static int16_t audio_buf[AUDIO_CHUNK];
#endif

// Housekeeping

#ifndef CODECELL_DRIVE
static uint8_t neo_hue = 0;
#endif
static uint32_t pkt_cnt = 0;
static uint32_t last_hz_ms = 0;
static uint32_t last_stream_us = 0;

// ── BLE GATT server ───────────────────────────────────────────────────────────

static const char *BLE_DEV_NAME = "RollingStone";
static const char *BLE_SVC_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
static const char *BLE_TELEM_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
static const char *BLE_CMD_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214";
static const char *BLE_STAT_UUID = "19b10003-e8f2-537e-4f6c-d104768a1214";

static BLEServer *ble_server = nullptr;
static BLECharacteristic *ble_telem_chr = nullptr;
static BLECharacteristic *ble_stat_chr = nullptr;

static void handleCommand(char c); // forward declaration

class BleServerCB : public BLEServerCallbacks
{
    void onDisconnect(BLEServer *pServer) override
    {
        BLEDevice::startAdvertising();
    }
};

class BleCmdCB : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *chr) override
    {
        String val = chr->getValue();
        if (val.length() > 0)
            handleCommand(val.charAt(0));
    }
};

static void bleSendStatus()
{
    if (!ble_stat_chr)
        return;
    char buf[32];
    snprintf(buf, sizeof(buf), "%s,%.0f",
             g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
             g_cavity * 1000.0f);
    ble_stat_chr->setValue(buf);
    ble_stat_chr->notify();
}

static void initBLE()
{
    BLEDevice::init(BLE_DEV_NAME);
    ble_server = BLEDevice::createServer();
    ble_server->setCallbacks(new BleServerCB());

    BLEService *svc = ble_server->createService(BLE_SVC_UUID);

    ble_telem_chr = svc->createCharacteristic(
        BLE_TELEM_UUID,
        BLECharacteristic::PROPERTY_NOTIFY);

    BLECharacteristic *cmd_chr = svc->createCharacteristic(
        BLE_CMD_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    cmd_chr->setCallbacks(new BleCmdCB());

    ble_stat_chr = svc->createCharacteristic(
        BLE_STAT_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    svc->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.printf("BLE : advertising as \"%s\"\n", BLE_DEV_NAME);
}

// ── Serial command handler ─────────────────────────────────────────────────────

static void handleCommand(char c)
{
    switch (c)
    {
    case 'r':
        g_sim_mode = SimMode::ROLLING;
        Serial.println("# mode=ROLLING");
        bleSendStatus();
        break;
    case 's':
        g_sim_mode = SimMode::SLIDING;
        Serial.println("# mode=SLIDING");
        bleSendStatus();
        break;
    case '+':
        g_cavity = fminf(g_cavity + 0.05f, 2.00f);
        if (x_pos > g_cavity)
        {
            x_pos = g_cavity;
            x_vel = 0.0f;
            accel_k = 0.0f;
        }
        Serial.printf("# cavity_mm=%.0f\n", g_cavity * 1000.0f);
        bleSendStatus();
        break;
    case '-':
        g_cavity = fmaxf(g_cavity - 0.05f, 0.10f);
        if (x_pos > g_cavity)
        {
            x_pos = g_cavity;
            x_vel = 0.0f;
            accel_k = 0.0f;
        }
        Serial.printf("# cavity_mm=%.0f\n", g_cavity * 1000.0f);
        bleSendStatus();
        break;
    case '?':
        Serial.printf("# mode=%s  cavity_mm=%.0f  G=%.1f  mu=%.2f\n",
                      g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                      g_cavity * 1000.0f, G_FACTOR, FRICTION_MU);
        break;
    }
}

struct ImpactInfo
{
    bool hit;
    bool left;
    float speed;
};

static ImpactInfo physicsStep(float dt)
{
    ImpactInfo impact = {false, false, 0.0f};

    // ── Acceleration at current step ──────────────────────────────────────
    float accel_new;
    if (g_sim_mode == SimMode::ROLLING)
    {
        // Eq. 2: ẍ = (g/1.4)·sin(α)
        // Tube acceleration acts opposite in the tube frame.
        accel_new = G_FACTOR * sin_alpha - (G_FACTOR / 9.8f) * lin_accel_x;
    }
    else
    {
        // Eq. 3: sliding with Coulomb friction.
        float s = sin_alpha;
        float c = sqrtf(fmaxf(1.0f - s * s, 0.0f));
        if (s * s <= FRICTION_MU * FRICTION_MU * (1.0f - s * s))
            accel_new = 0.0f;
        else
            accel_new = 9.8f * s - 9.8f * FRICTION_MU * copysignf(c, s);
        accel_new -= lin_accel_x;
    }

    // ── Trapezoidal integration (paper §5.1) ──────────────────────────────
    float v_new = x_vel + dt * (accel_k + accel_new) * 0.5f;
    float x_new = x_pos + dt * (x_vel + v_new) * 0.5f;
    accel_k = accel_new;

    // ── Wall constraint + rebound ─────────────────────────────────────────
    if (x_new <= 0.0f)
    {
        if (v_new < 0.0f)
        {
            if (!at_left_wall)
            {
                impact.hit = true;
                impact.left = true;
                impact.speed = fabsf(v_new);
                v_new = -RESTITUTION_E * v_new;
            }
            else
            {
                v_new = 0.0f;
            }
        }
        x_new = 0.0f;
        at_left_wall = true;
        at_right_wall = false;
    }
    else if (x_new >= g_cavity)
    {
        if (v_new > 0.0f)
        {
            if (!at_right_wall)
            {
                impact.hit = true;
                impact.left = false;
                impact.speed = fabsf(v_new);
                v_new = -RESTITUTION_E * v_new;
            }
            else
            {
                v_new = 0.0f;
            }
        }
        x_new = g_cavity;
        at_right_wall = true;
        at_left_wall = false;
    }
    else
    {
        at_left_wall = false;
        at_right_wall = false;
    }

    x_vel = v_new;
    x_pos = x_new;

    return impact;
}

// ══════════════════════════════════════════════════════════════════════════════
// FEATHER-ONLY helpers
// ══════════════════════════════════════════════════════════════════════════════

#ifndef CODECELL_DRIVE

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

static void setReports()
{
    // Gravity + linear acceleration reports at 1000 Hz.
    if (!bno08x.enableReport(SH2_GRAVITY, 1000))
        Serial.println("Could not enable gravity vector");
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 1000))
        Serial.println("Could not enable linear acceleration");
}

// Rolling noise wavetable — negative arch of sine.
// Indexed by ball position in mm, so pitch ∝ velocity — exactly like a real ball.
static void initWavetable()
{
    // Negative arch: drives hammer into repelling field → blunt "pulse" feel.
    // Keeps rolling sensation soft and distinct from the sharp +V impact tick.
    for (int i = 0; i < WTABLE_SIZE; i++)
        wtable[i] = -sinf((float)M_PI * i / (WTABLE_SIZE - 1));
}

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
                  (int)PIN_I2S_BCLK, (int)PIN_I2S_LRCLK, (int)PIN_I2S_DOUT, SAMPLE_RATE);
}

// Fills audio_buf with AUDIO_CHUNK mono frames.
// Each sample advances the physics simulation by one step H.
static void fillAudioBuf()
{
    for (int i = 0; i < AUDIO_CHUNK; i++)
    {
        ImpactInfo impact = physicsStep(H);

        if (impact.hit)
        {
            impact_amp = fminf(impact.speed / IMPACT_VREF, 1.0f);
            impact_left = IMPACT_SAMPLES;
        }

        float sample_f = 0.0f;
        if (impact_left > 0)
        {
            sample_f = impact_amp;
            impact_left--;
        }
        else if (g_sim_mode == SimMode::ROLLING)
        {
            // Wavetable index = pos_mm mod 30; amplitude ∝ speed.
            int idx = abs((int)(x_pos * 1000.0f)) % WTABLE_SIZE;
            float speed = fminf(fabsf(x_vel) / 2.0f, 1.0f);
            sample_f = wtable[idx] * speed;
        }
        // (SLIDING without impact → silence between walls, just like the paper)

        audio_buf[i] = (int16_t)(sample_f * GAIN * 32767.0f);
    }
}

#endif // !CODECELL_DRIVE

// ══════════════════════════════════════════════════════════════════════════════
// CODECELL-ONLY: one physics + haptic step, called at PHYS_HZ (1000 Hz)
// ══════════════════════════════════════════════════════════════════════════════

#ifdef CODECELL_DRIVE

static void ccPhysicsAndHaptic()
{
    ImpactInfo impact = physicsStep(H);

    if (impact.hit)
    {
        myCodeCell.Drive1.Pulse(!impact.left, CC_IMPACT_MS);
        cc_impact_left = CC_IMPACT_MS;
    }

    // ── Haptic output ─────────────────────────────────────────────────────────
    if (cc_impact_left > 0)
    {
        // Pulse() is running its timer — do not call Drive/Run until it finishes.
        cc_impact_left--;
    }
    else if (g_sim_mode == SimMode::ROLLING)
    {
        float speed = fabsf(x_vel);
        if (speed >= CC_MIN_SPEED)
        {
            // Same pitch-velocity law as the wavetable:
            //   spatial period = WTABLE_SIZE mm = 0.030 m
            //   half-period time = 0.015 m / v → flip_ms = 15 / v  [ms]
            uint8_t pct = (uint8_t)fminf(speed / 2.0f * 100.0f, 100.0f);
            uint16_t flip_ms = (uint16_t)fmaxf(fminf(15.0f / speed, 500.0f), 5.0f);
            myCodeCell.Drive1.Run(true, pct, flip_ms);
        }
        else
        {
            myCodeCell.Drive1.Drive(false, 0); // ball nearly stopped → quiet
        }
    }
    else
    {
        // SLIDING: no rolling noise between wall contacts (matches paper)
        myCodeCell.Drive1.Drive(false, 0);
    }
}

#endif // CODECELL_DRIVE

// ── setup ─────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);

#ifdef CODECELL_DRIVE
    // CodeCell.Init() starts Wire(IO8, IO9), configures power management,
    // initialises the onboard BNO085, and sets up the status LED.
    myCodeCell.Init(MOTION_GRAVITY | MOTION_LINEAR_ACC);
    myCodeCell.Drive1.Init();
    // Drive2 (IO2/IO3) is available for a second actuator if needed.

    delay(500); // let serial settle after USB-CDC init

    Serial.println("\n================================");
    Serial.println("Virtual Rolling Stone  —  CodeCell ESP32-C6 Drive");
    Serial.println("================================");
    Serial.printf("# mode=%s  cavity_mm=%.0f  G=%.1f  mu=%.2f\n",
                  g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                  g_cavity * 1000.0f, G_FACTOR, FRICTION_MU);
    Serial.printf("# Output: H-bridge Drive1 (IO22/IO21)  phys=%d Hz  stream=%lu Hz\n",
                  PHYS_HZ, 1000000UL / STREAM_INTERVAL_US);

#else
    pinMode(PIN_STEMMA_POWER, OUTPUT);
    digitalWrite(PIN_STEMMA_POWER, HIGH); // power STEMMA QT + NeoPixel

    pixel.begin();
    pixel.setBrightness(40);
    neoBlink(255, 255, 255, 3); // WHITE: firmware alive

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

    Serial.printf("# mode=%s  cavity_mm=%.0f  G=%.1f  mu=%.2f\n",
                  g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                  g_cavity * 1000.0f, G_FACTOR, FRICTION_MU);
    Serial.printf("# Output: I2S %u Hz  stream=%lu Hz\n",
                  SAMPLE_RATE, 1000000UL / STREAM_INTERVAL_US);
    delay(100);
#endif

    initBLE();

    last_hz_ms = millis();
    last_stream_us = micros();
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop()
{
    // ── Serial commands from companion app ────────────────────────────────────
    while (Serial.available())
        handleCommand((char)Serial.read());

#ifdef CODECELL_DRIVE
    // myCodeCell.Run(Hz) returns true at the requested rate and handles
    // sensor polling, power management, and LED updates internally.
    if (myCodeCell.Run((uint8_t)PHYS_HZ))
    {
        // Gravity vector gives tilt; linear accel gives tube shake along axis.
        float gx, gy, gz;
        float lax, lay, laz;
        myCodeCell.Motion_GravityRead(gx, gy, gz);
        myCodeCell.Motion_LinearAccRead(lax, lay, laz);
        sin_alpha = fmaxf(-1.0f, fminf(1.0f, gx / 9.8f));
        lin_accel_x = lax;
        pkt_cnt++;

        ccPhysicsAndHaptic();

        uint32_t now = millis();

        // Stream telemetry to companion app at 100 Hz
        uint32_t now_us = micros();
        if (now_us - last_stream_us >= STREAM_INTERVAL_US)
        {
            Serial.printf("$%+.4f,%.1f,%+.4f\n",
                          sin_alpha, x_pos * 1000.0f, x_vel);
            if (ble_server->getConnectedCount() > 0)
            {
                float pkt[3] = {sin_alpha, x_pos * 1000.0f, x_vel};
                ble_telem_chr->setValue((uint8_t *)pkt, sizeof(pkt));
                ble_telem_chr->notify();
            }
            last_stream_us = now_us;
        }

        // 1 Hz debug line
        if (now - last_hz_ms >= 1000)
        {
            float phys_hz = pkt_cnt * 1000.0f / (float)(now - last_hz_ms);
            Serial.printf("# sin_a=%+.3f  x_mm=%.1f  v_mps=%+.4f  hz=%.0f"
                          "  mode=%s  cavity_mm=%.0f\n",
                          sin_alpha, x_pos * 1000.0f, x_vel, phys_hz,
                          g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                          g_cavity * 1000.0f);
            pkt_cnt = 0;
            last_hz_ms = now;
        }
    }

#else
    // 1. Read BNO085 — drain up to 8 pending events
    if (bno08x.wasReset())
    {
        Serial.println("# sensor reset — re-enabling");
        neoBlink(255, 80, 0, 2, 80); // ORANGE
        setReports();
    }
    for (int n = 0; n < 8 && bno08x.getSensorEvent(&sensorValue); n++)
    {
        if (sensorValue.sensorId == SH2_GRAVITY)
        {
            float gx = sensorValue.un.gravity.x;
            sin_alpha = fmaxf(-1.0f, fminf(1.0f, gx / 9.8f));
            pkt_cnt++;
        }
        else if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION)
        {
            lin_accel_x = sensorValue.un.linearAcceleration.x;
        }
    }

    // 2. NeoPixel: cycle hue while running (indicates activity)
    {
        uint32_t c = pixel.ColorHSV((uint16_t)(neo_hue++) * 256, 255, 180);
        pixel.setPixelColor(0, c);
        pixel.show();
    }

    // 3. Run physics simulation + generate audio samples
    fillAudioBuf();

    // 4. Stream telemetry to companion app at 100 Hz
    {
        uint32_t now = millis();
        uint32_t now_us = micros();
        if (now_us - last_stream_us >= STREAM_INTERVAL_US)
        {
            Serial.printf("$%+.4f,%.1f,%+.4f\n",
                          sin_alpha, x_pos * 1000.0f, x_vel);
            if (ble_server->getConnectedCount() > 0)
            {
                float pkt[3] = {sin_alpha, x_pos * 1000.0f, x_vel};
                ble_telem_chr->setValue((uint8_t *)pkt, sizeof(pkt));
                ble_telem_chr->notify();
            }
            last_stream_us = now_us;
        }
    }

    // 5. Push to I2S DMA (blocks ~8 ms in steady state — natural rate limiter)
    size_t written;
    i2s_channel_write(tx_chan, audio_buf, sizeof(audio_buf),
                      &written, pdMS_TO_TICKS(100));

    // 6. 1 Hz debug line (includes mode + cavity for companion app)
    uint32_t now = millis();
    if (now - last_hz_ms >= 1000)
    {
        float sensor_hz = pkt_cnt * 1000.0f / (float)(now - last_hz_ms);
        Serial.printf("# sin_a=%+.3f  x_mm=%.1f  v_mps=%+.4f  hz=%.0f"
                      "  mode=%s  cavity_mm=%.0f\n",
                      sin_alpha, x_pos * 1000.0f, x_vel, sensor_hz,
                      g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                      g_cavity * 1000.0f);
        pkt_cnt = 0;
        last_hz_ms = now;
    }
#endif
}
