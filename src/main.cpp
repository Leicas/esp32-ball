/*
 * Virtual Rolling Stone — CodeCell ESP32-C6 Drive (Unified Firmware)
 *
 * Reproduces the apparatus from:
 *   Yao & Hayward, "An Experiment on Length Perception with a Virtual Rolling Stone"
 *   Eurohaptics 2006, pp. 325-330.
 *
 * Unified firmware (no CodeCell/DriveCell libs) — direct I2C + PWM on CodeCell board:
 *   Onboard BNO085 (I2C SDA=IO8 SCL=IO9)
 *   H-bridge via ledc PWM (IN1=IO22, IN2=IO21)
 *   Rolling haptic: unipolar PWM with pitch ∝ velocity
 *   Impact haptic: unipolar (+V) pulse
 *   Strict 1 kHz physics loop with rate limiting
 *
 * Physics (from paper):
 *   Rolling : ẍ = (g/1.4)·sin(α) ≈ 7.0·sin(α)          [Eq. 2]
 *   Sliding : ẍ = g·sin(α) − g·µ·sgn(sin(α))·cos(α)     [Eq. 3]
 *   sin(α) = accel_x / g  (gravity + inertia projected on tube axis)
 *
 * Companion app serial protocol:
 *   Firmware → host:
 *     "$sin_a,x_mm,v_mps\n"              at 60 Hz (serial only)
 *     "# key=val ...\n"                  at  1 Hz debug line
 *   Host → firmware (single ASCII char):
 *     'r'  ROLLING mode
 *     's'  SLIDING mode
 *     'b'  toggle REBOUND (bounce)
 *     '+'  cavity +5 cm  (clamps 10 cm – 200 cm)
 *     '-'  cavity −5 cm
 *     '?'  print current parameters
 */

#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ── CodeCell board pins ───────────────────────────────────────────────────────

#define BNO08X_RESET -1
static constexpr int PIN_SDA = 8;
static constexpr int PIN_SCL = 9;
static constexpr int PIN_HB_IN1 = 22;  // H-bridge control IN1 (negative drive)
static constexpr int PIN_HB_IN2 = 21;  // H-bridge control IN2 (positive drive)
static constexpr int PIN_LED_ON = 10;  // status LED enable (hold high)
static constexpr int PIN_SENS_ON = 11; // sensor power enable (hold high)

// ── PWM configuration for H-bridge ────────────────────────────────────────────

static constexpr uint32_t CC_PWM_FREQ = 20000;  // 20 kHz PWM frequency
static constexpr uint8_t CC_PWM_BITS = 8;       // 8-bit resolution (0-255)
static constexpr uint32_t CC_PWM_MAX = (1U << CC_PWM_BITS) - 1;  // 255
static constexpr float CC_MAX_PCT = 80.0f;      // cap rolling amplitude to 80%
static constexpr float CC_MIN_SPEED = 0.05f;    // m/s — stop actuator below this
static constexpr uint32_t IMPACT_MS = 9;        // impact pulse duration [ms]

// ── Physics parameters (shared, fixed) ────────────────────────────────────────

static constexpr float G_FACTOR = 7.0f;        // g/1.4 — solid sphere [m/s²]
static constexpr float FRICTION_MU = 0.20f;    // Coulomb coeff (sliding only)
static constexpr int WTABLE_SIZE = 30;         // spatial period [mm per repeat]
static constexpr float RESTITUTION_E = 0.35f;  // bounce coefficient (0=no bounce)
static constexpr float IMPACT_VREF = 0.5f;     // reference speed for impact (m/s) — softer scaling

// Physics loop timing
static constexpr int PHYS_HZ = 1000;             // 1 kHz physics loop
static constexpr float H = 1.0f / PHYS_HZ;       // time step [s] = 1 ms
static constexpr uint32_t LOOP_PERIOD_US = 1000; // 1 ms period

// Streaming
static constexpr uint32_t STREAM_INTERVAL_US = 16667;  // ~60 Hz serial telemetry
static constexpr uint32_t BLE_INTERVAL_US = 33333;     // ~30 Hz BLE telemetry

// Switch between ROLLING (rolling noise + impact) and SLIDING (impact only).
enum class SimMode : uint8_t
{
    ROLLING,
    SLIDING
};

// ── Runtime-adjustable simulation parameters ──────────────────────────────────

static float g_cavity = 1.00f;                // virtual tube length [m]
static SimMode g_sim_mode = SimMode::ROLLING; // ROLLING or SLIDING
static bool g_rebound = true;                 // enable/disable bounce
static bool g_cavity_mode_large = true;       // true = 1000mm, false = 200mm (for double-tap toggle)

// ── Double-tap detection parameters ───────────────────────────────────────────
static constexpr float TAP_ACCEL_THRESHOLD = 15.0f;  // m/s² threshold for tap detection
static constexpr uint32_t TAP_WINDOW_MS = 300;       // max time between taps [ms]
static uint32_t last_tap_time = 0;                   // timestamp of last tap
static bool tap_pending = false;                     // awaiting second tap

// ── Objects ───────────────────────────────────────────────────────────────────

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// ── Physics state ─────────────────────────────────────────────────────────────

static float x_pos = 0.50f;      // ball position [m] — half of default 1.0 m cavity
static float x_vel = 0.0f;       // ball velocity [m/s]
static float accel_k = 0.0f;     // accel at step k (trapezoidal carry)
static float sin_alpha = 0.0f;   // sin of tube tilt angle (from gravity vector)
static float lin_accel_x = 0.0f; // tube linear accel along axis [m/s^2]

static bool at_left_wall = false;
static bool at_right_wall = false;

// ── Impact state ──────────────────────────────────────────────────────────────

static uint32_t impact_end_ms = 0;   // when impact pulse ends [ms]
static bool at_left_wall_contact = false;  // was at left wall during last step (prevent re-trigger)
static bool at_right_wall_contact = false; // was at right wall during last step

// ── Wavetable for rolling noise synthesis ─────────────────────────────────────

static float wtable[WTABLE_SIZE];

// ── Housekeeping / debug ──────────────────────────────────────────────────────

static uint32_t pkt_cnt = 0;
static uint32_t loop_cnt = 0;
static uint32_t busy_sum_us = 0;
static uint32_t overrun_cnt = 0;
static uint32_t last_hz_ms = 0;
static uint32_t last_stream_us = 0;
static uint32_t last_ble_us = 0;
static uint32_t last_loop_check_ms = 0;

// Running averages for debug output
static float sensor_hz = 0.0f;
static float loop_hz = 0.0f;
static float busy_pct = 0.0f;

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
    char buf[64];
    snprintf(buf, sizeof(buf), "%s,%.0f,rebound=%d",
             g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
             g_cavity * 1000.0f, g_rebound ? 1 : 0);
    ble_stat_chr->setValue(buf);
    ble_stat_chr->notify();
}

static void toggleCavitySize()
{
    // Toggle cavity between 200mm and 1000mm
    g_cavity_mode_large = !g_cavity_mode_large;
    g_cavity = g_cavity_mode_large ? 1.00f : 0.20f;
    
    if (x_pos > g_cavity)
    {
        x_pos = g_cavity;
        x_vel = 0.0f;
        accel_k = 0.0f;
    }
    
    Serial.printf("# DOUBLE-TAP: cavity_mm=%.0f\n", g_cavity * 1000.0f);
    bleSendStatus();
}

static void detectDoubleTap(float accel_magnitude)
{
    // Detect double-tap by monitoring acceleration peaks
    uint32_t now = millis();
    
    if (accel_magnitude > TAP_ACCEL_THRESHOLD)
    {
        if (tap_pending && (now - last_tap_time) <= TAP_WINDOW_MS)
        {
            // Double-tap detected!
            tap_pending = false;
            toggleCavitySize();
        }
        else
        {
            // First tap (or tap window expired—restart)
            tap_pending = true;
            last_tap_time = now;
        }
    }
    else if (tap_pending && (now - last_tap_time) > TAP_WINDOW_MS)
    {
        // Tap window expired without second tap
        tap_pending = false;
    }
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
    case 'b':
        g_rebound = !g_rebound;
        Serial.printf("# rebound=%d\n", g_rebound ? 1 : 0);
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
        Serial.printf("# mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f\n",
                      g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                      g_cavity * 1000.0f, g_rebound ? 1 : 0, G_FACTOR, FRICTION_MU);
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
            if (!at_left_wall_contact)
            {
                if (g_rebound)
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
            if (!at_right_wall_contact)
            {
                if (g_rebound)
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
    
    // Update wall contact state for next frame (edge detection)
    at_left_wall_contact = at_left_wall;
    at_right_wall_contact = at_right_wall;

    return impact;
}

// ──────────────────────────────────────────────────────────────────────────────
// Wavetable initialization
// ──────────────────────────────────────────────────────────────────────────────

static void initWavetable()
{
    // Negative arch: drives hammer into repelling field → blunt "pulse" feel.
    for (int i = 0; i < WTABLE_SIZE; i++)
        wtable[i] = -sinf((float)M_PI * i / (WTABLE_SIZE - 1));
}

// ──────────────────────────────────────────────────────────────────────────────
// PWM H-bridge control — unipolar haptic synthesis
// ──────────────────────────────────────────────────────────────────────────────

static void ccInitPwm()
{
    ledcAttach(PIN_HB_IN1, CC_PWM_FREQ, CC_PWM_BITS);
    ledcAttach(PIN_HB_IN2, CC_PWM_FREQ, CC_PWM_BITS);
    ledcWrite(PIN_HB_IN1, 0);
    ledcWrite(PIN_HB_IN2, 0);
}

static void ccUpdateHaptic()
{
    uint32_t now_ms = millis();

    // Impact pulse takes priority
    if (impact_end_ms > 0 && now_ms < impact_end_ms)
    {
        // Positive pulse for impact
        ledcWrite(PIN_HB_IN1, CC_PWM_MAX);
        ledcWrite(PIN_HB_IN2, 0);
        return;
    }
    impact_end_ms = 0;

    // Rolling texture: simple PWM modulation
    if (g_sim_mode == SimMode::ROLLING && fabsf(x_vel) > CC_MIN_SPEED)
    {
        // Frequency proportional to velocity (like original flip_ms = 15/|v|)
        // At 20 kHz PWM, use velocity to modulate duty cycle with spatial position
        float speed = fminf(fabsf(x_vel) / 2.0f, 1.0f);
        int pos_idx = abs((int)(x_pos * 1000.0f)) % WTABLE_SIZE;
        float wave = wtable[pos_idx]; // -1 to 0

        // Negative drive (IN2) with amplitude cap
        float amp = -wave * speed * (CC_MAX_PCT / 100.0f);
        uint32_t duty = (uint32_t)(amp * CC_PWM_MAX);
        ledcWrite(PIN_HB_IN1, 0);
        ledcWrite(PIN_HB_IN2, duty);
    }
    else
    {
        ledcWrite(PIN_HB_IN1, 0);
        ledcWrite(PIN_HB_IN2, 0);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// I2C and sensor setup
// ──────────────────────────────────────────────────────────────────────────────

static void setReports()
{
    // Gravity + linear acceleration reports at 1000 Hz.
    if (!bno08x.enableReport(SH2_GRAVITY, 1000))
        Serial.println("Could not enable gravity vector");
    if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION, 1000))
        Serial.println("Could not enable linear acceleration");
}

// ──────────────────────────────────────────────────────────────────────────────
// setup
// ──────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0 < 3000))
    {
        delay(10);
    }

    pinMode(PIN_LED_ON, OUTPUT);
    digitalWrite(PIN_LED_ON, HIGH);

    pinMode(PIN_SENS_ON, OUTPUT);
    gpio_hold_dis((gpio_num_t)PIN_SENS_ON);
    digitalWrite(PIN_SENS_ON, HIGH);

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    delay(200);

    initWavetable();
    ccInitPwm();

    Serial.println("# imu: init BNO085");
    if (!bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire))
    {
        Serial.println("BNO085 not found — check wiring!");
        while (1)
            delay(10);
    }
    Serial.println("# imu: ok");
    setReports();

    delay(500);

    Serial.println("\n================================");
    Serial.println("Virtual Rolling Stone  —  CodeCell ESP32-C6 Drive (Unified)");
    Serial.println("================================");
    Serial.printf("# mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f\n",
                  g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                  g_cavity * 1000.0f, g_rebound ? 1 : 0, G_FACTOR, FRICTION_MU);
    Serial.printf("# Output: H-bridge + PWM  1000 Hz physics  stream=%lu Hz\n",
                  1000000UL / STREAM_INTERVAL_US);

    initBLE();

    last_hz_ms = millis();
    last_stream_us = micros();
    last_ble_us = micros();
    last_loop_check_ms = millis();
}

// ──────────────────────────────────────────────────────────────────────────────
// main loop — 1 kHz synchronized
// ──────────────────────────────────────────────────────────────────────────────

void loop()
{
    // ── Serial commands from companion app ────────────────────────────────────
    while (Serial.available())
        handleCommand((char)Serial.read());

    // Fixed 1 kHz loop with rate limiting
    static uint32_t next_loop_us = 0;
    uint32_t loop_start_us = micros();

    if (next_loop_us == 0)
    {
        next_loop_us = loop_start_us;
    }

    // 1. Read BNO085 — drain up to 2 pending events (minimize I2C blocking)
    if (bno08x.wasReset())
    {
        Serial.println("# sensor reset — re-enabling");
        setReports();
    }
    for (int n = 0; n < 2 && bno08x.getSensorEvent(&sensorValue); n++)
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

    // 2. Physics step at 1 kHz
    ImpactInfo impact = physicsStep(H);

    // Trigger impact pulse on wall contact
    if (impact.hit)
    {
        impact_end_ms = millis() + IMPACT_MS;
    }

    // Update haptic output
    ccUpdateHaptic();

    uint32_t now_us = micros();
    uint32_t now = millis();

    // 3. Stream telemetry to companion app at 60 Hz (serial only)
    if (now_us - last_stream_us >= STREAM_INTERVAL_US)
    {
        Serial.printf("$%+.4f,%.1f,%+.4f\n",
                      sin_alpha, x_pos * 1000.0f, x_vel);
        last_stream_us = now_us;
    }

    // BLE telemetry at 30 Hz
    if (ble_server->getConnectedCount() > 0 && now_us - last_ble_us >= BLE_INTERVAL_US)
    {
        float pkt[3] = {sin_alpha, x_pos * 1000.0f, x_vel};
        ble_telem_chr->setValue((uint8_t *)pkt, sizeof(pkt));
        ble_telem_chr->notify();
        last_ble_us = now_us;
    }

    // 4. 1 Hz debug line with loop rate check
    loop_cnt++;
    if (now - last_hz_ms >= 1000)
    {
        sensor_hz = pkt_cnt * 1000.0f / (float)(now - last_hz_ms);
        loop_hz = loop_cnt * 1000.0f / (float)(now - last_loop_check_ms);
        busy_pct = loop_cnt > 0
                       ? 100.0f * (float)busy_sum_us / (float)(LOOP_PERIOD_US * loop_cnt)
                       : 0.0f;
        Serial.printf("# sin_a=%+.3f  x_mm=%.1f  v_mps=%+.4f  hz=%.0f  loop_hz=%.0f"
                      "  busy=%.1f  mode=%s  cavity_mm=%.0f  rebound=%d",
                      sin_alpha, x_pos * 1000.0f, x_vel, sensor_hz, loop_hz, busy_pct,
                      g_sim_mode == SimMode::ROLLING ? "ROLLING" : "SLIDING",
                      g_cavity * 1000.0f, g_rebound ? 1 : 0);
        if (overrun_cnt > 0)
        {
            Serial.printf("  *** %lu loops exceeded 1ms (%.1f%%) ***",
                          (unsigned long)overrun_cnt, 100.0f * overrun_cnt / loop_cnt);
        }
        Serial.println();
        bleSendStatus();
        pkt_cnt = 0;
        loop_cnt = 0;
        overrun_cnt = 0;
        busy_sum_us = 0;
        last_hz_ms = now;
        last_loop_check_ms = now;
    }

    // 5. Rate limiting: busy-wait until next loop period or resync if overrun
    next_loop_us += LOOP_PERIOD_US;
    uint32_t work_end_us = micros();
    uint32_t busy_us = work_end_us - loop_start_us;
    if (busy_us > LOOP_PERIOD_US)
        busy_us = LOOP_PERIOD_US;
    busy_sum_us += busy_us;

    int32_t wait_us = (int32_t)(next_loop_us - work_end_us);
    if (wait_us > 0)
    {
        while ((int32_t)(micros() - next_loop_us) < 0)
        {
            // do nothing - spin until next period
        }
    }
    else if (wait_us < -100) // more than 100 us overrun
    {
        overrun_cnt++;
        // Resync to prevent drift accumulation
        next_loop_us = work_end_us;
    }
}
