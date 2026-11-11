#include "rain_tube.h"
static RainTube s_rain;
static volatile int s_rain_variant=1;
/*
 * Virtual Rolling Stone — ESP32-C6 Haptic Firmware
 *
 * Reproduces Yao & Hayward, "An Experiment on Length Perception with a
 * Virtual Rolling Stone", Eurohaptics 2006, pp. 325-330.
 *
 * Three build targets (platformio.ini):
 *   feather   — STEMMA BNO085 + I2S audio amp + NeoPixel
 *   hbridge   — onboard BNO085 + H-bridge PWM  (-DHBRIDGE)
 *   a7_ball   — Rolling Stone A7 PCB: LSM6DS3TR-C + TDM to TacHammer LFi/HF (-DA7_BOARD)
 *
 * Architecture (ESP32-C6 = single core):
 *   taskPhysics (priority 5, 1 kHz) — IMU → physics → haptic → telemetry snapshot
 *   taskComms   (priority 1, low)   — serial/BLE commands + ~58 Hz telemetry stream
 *
 * Serial protocol (companion app):
 *   Firmware → host: "$sin_a,x_mm,v_mps\n"  at ~58 Hz
 *                    "# key=val …\n"          at  1 Hz and on command replies
 *   Host → firmware: 'l' LIGHTSABER | 't' TEXTURED ('r' legacy alias) | 's' SLIDING | 'm' MARBLES | 'b' rebound
 *                    '+' cavity+5cm | '-' cavity-5cm | '?' dump params
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "config.h"
#include "imu.h"
#include "physics.h"
#include "haptic.h"
#include "comms.h"
#include "marbles.h"
#include "game.h"
#include "battery_runtime.h"
#include "battery_wifi.h"

std::atomic<uint32_t> game_round{0};
std::atomic<bool> game_revealed{false};
std::atomic<int> game_count{0};
static std::atomic<uint32_t> game_seed_request{0};
static std::atomic<bool> game_reveal_request{false};
void gameNewRound() {
    game_revealed.store(false);
    game_reveal_request.store(false);
    game_seed_request.store(esp_random() | 1u);
    g_sim_mode = SimMode::MARBLES;
}
void gameReveal() {
    if (g_sim_mode == SimMode::MARBLES)
        game_reveal_request.store(true);
}

#ifdef HBRIDGE
#include "driver/gpio.h" // gpio_hold_dis
#endif

#if defined(A7_BOARD)
// ── A7: PCA9632 handle RGB stands in for the NeoPixel; MODE / GAIN buttons ───
#include <Preferences.h>
static constexpr uint8_t PCA9632_ADDR = 0x62;
uint32_t g_a7_brownouts = 0; // persistent count, printed in the 1 Hz status line
static bool s_pca_ok = false;
static volatile float s_saber_motion = 0.0f;
static volatile uint32_t s_impact_flash_ms = 0; // set by the physics task on wall / marble impacts
static int s_gain_idx = 2;
static const float kHapticGains[3] = {0.4f, 0.7f, 1.0f};
static uint32_t s_mode_ack_start = 0, s_gain_ack_start = 0;
static bool s_mode_ack_active = false, s_gain_ack_pending = false, s_gain_ack_active = false;
static int s_ack_blinks = 1; // flashes shown by the count acknowledgement (gain level or rain character)

static void pcaWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(PCA9632_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}
static void neoSet(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_pca_ok)
        return;
    Wire.beginTransmission(PCA9632_ADDR);
    Wire.write(0x80 | 0x02); // auto-increment from PWM0
    Wire.write(b);           // LED0 blue
    Wire.write(r);           // LED1 red
    Wire.write(g);           // LED2 green
    Wire.endTransmission();
}

// Shared by the physical GAIN button and the 'g' serial/BLE command. This is
// called on the comms task so UI state has a single owner.
void a7CycleGain()
{
    s_gain_idx = (s_gain_idx + 1) % 3;
    g_haptic_gain = kHapticGains[s_gain_idx];
    s_ack_blinks = s_gain_idx + 1;
    const uint32_t now = millis();
    if (s_mode_ack_active)
        s_gain_ack_pending = true;
    else
    {
        s_gain_ack_start = now;
        s_gain_ack_active = true;
    }
}

static void pcaInit()
{
    Wire.beginTransmission(PCA9632_ADDR);
    s_pca_ok = Wire.endTransmission() == 0;
    if (!s_pca_ok)
        return;
    pcaWrite(0x01, 0x00); // MODE2: open-drain, non-inverted
    pcaWrite(0x00, 0x01); // MODE1: awake
    delayMicroseconds(600);
    pcaWrite(0x08, 0x2A); // LEDOUT: LED0..2 individual PWM
}

struct A7DebouncedButton
{
    bool initialized = false;
    int candidate = HIGH;
    int stable = HIGH;
    uint32_t candidate_since = 0;
};

static bool a7ButtonPressed(A7DebouncedButton &button, int pin, uint32_t now)
{
    const int raw = digitalRead(pin);
    if (!button.initialized)
    {
        button.initialized = true;
        button.candidate = button.stable = raw;
        button.candidate_since = now;
        return false;
    }

    if (raw != button.candidate)
    {
        button.candidate = raw;
        button.candidate_since = now;
    }
    else if (button.candidate != button.stable && now - button.candidate_since >= 40)
    {
        button.stable = button.candidate;
        return button.stable == LOW;
    }
    return false;
}

static void a7ModeColor(uint8_t &r, uint8_t &g, uint8_t &b)
{
    // One colour per mode, so the handle always says which effect is running.
    switch (g_sim_mode)
    {
    case SimMode::ROLLING:    r = 0;   g = 255; b = 0;   break; // green
    case SimMode::SLIDING:    r = 0;   g = 0;   b = 255; break; // blue
    case SimMode::TEXTURED:   r = 255; g = 40;  b = 0;   break; // red-orange
    case SimMode::MARBLES:    r = 255; g = 0;   b = 255; break; // magenta
    case SimMode::LIGHTSABER: r = 0;   g = 200; b = 255; break; // cyan
    case SimMode::RAIN:       r = 255; g = 180; b = 0;   break; // yellow
    }
}

// Called from the comms task (~1 kHz): buttons at 50 Hz, LED at ~33 Hz.
static void a7UiTick()
{
    static uint32_t t_btn = 0, t_led = 0;
    static A7DebouncedButton mode_button, gain_button;
    static bool mode_seen = false;
    static SimMode observed_mode;
    uint32_t now = millis();
    if (now - t_btn >= 20)
    {
        t_btn = now;
        if (a7ButtonPressed(mode_button, PIN_BTN_MODE, now))
        {
            if(g_sim_mode==SimMode::RAIN){
                // MODE picks the rainstick character: 1 flash sand, 2 rice, 3 beans, 4 moo box.
                s_rain_variant=(s_rain_variant+1)%4;
                s_ack_blinks=s_rain_variant+1;
                s_gain_ack_start=now;s_gain_ack_active=true;
            }
            else if(g_sim_mode==SimMode::LIGHTSABER)hapticReplayStartup();
            else {
                g_sim_mode=g_sim_mode==SimMode::ROLLING?SimMode::SLIDING:g_sim_mode==SimMode::SLIDING?SimMode::TEXTURED:g_sim_mode==SimMode::TEXTURED?SimMode::MARBLES:SimMode::ROLLING;
                if(g_sim_mode==SimMode::MARBLES)gameNewRound();
            }
            commsSendStatus();
        }
        if (a7ButtonPressed(gain_button, PIN_BTN_GAIN, now))
        {
            static SimMode last_ball=SimMode::ROLLING;
            if(g_sim_mode==SimMode::LIGHTSABER)g_sim_mode=SimMode::RAIN;
            else if(g_sim_mode==SimMode::RAIN)g_sim_mode=last_ball;
            else {last_ball=g_sim_mode;g_sim_mode=SimMode::LIGHTSABER;}
            commsSendStatus();
        }
    }
    if (!mode_seen)
    {
        observed_mode = g_sim_mode;
        mode_seen = true;
    }
    else if (g_sim_mode != observed_mode)
    {
        observed_mode = g_sim_mode;
        s_mode_ack_start = now;
        s_mode_ack_active = true;
        s_gain_ack_pending = s_gain_ack_pending || s_gain_ack_active;
        s_gain_ack_active = false;
    }
    if (now - t_led >= 30)
    {
        t_led = now;
        uint8_t r, gr, b;
        a7ModeColor(r, gr, b);
        const uint32_t mode_ack_age = now - s_mode_ack_start;
        if (s_mode_ack_active && mode_ack_age >= 360)
        {
            s_mode_ack_active = false;
            if (s_gain_ack_pending)
            {
                s_gain_ack_pending = false;
                s_gain_ack_active = true;
                s_gain_ack_start = now;
            }
        }

        if (s_mode_ack_active)
        {
            // Steady display of the selected mode colour, distinct from gain-count flashes.
            neoSet(r, gr, b);
        }
        else if (s_gain_ack_active && now - s_gain_ack_start < (uint32_t)s_ack_blinks * 220)
        {
            // One, two, or three 110 ms flashes: gain 0.4 / 0.7 / 1.0, or rain sand / rice / beans.
            if ((now - s_gain_ack_start) % 220 < 110)
                neoSet(r, gr, b);
            else
                neoSet(0, 0, 0);
        }
        else
        {
            s_gain_ack_active = false;
            // Mode colour, brighter with ball speed; white flash on impacts.
            float lv = g_sim_mode == SimMode::LIGHTSABER ? .15f + .65f*s_saber_motion : g_sim_mode == SimMode::MARBLES ? 0.15f : g_sim_mode == SimMode::RAIN ? 0.30f : 0.20f + fminf(fabsf(phys_x_vel) / 1.5f, 1.0f) * 0.6f;
            if ((int32_t)(now - s_impact_flash_ms) < 60)
                neoSet(160, 160, 160);
            else
                neoSet((uint8_t)(r * lv), (uint8_t)(gr * lv), (uint8_t)(b * lv));
        }
    }
}
#elif !defined(HBRIDGE)
#include <Adafruit_NeoPixel.h>
static Adafruit_NeoPixel s_neo(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static void neoSet(uint8_t r, uint8_t g, uint8_t b)
{
    s_neo.setPixelColor(0, s_neo.Color(r, g, b));
    s_neo.show();
}
#endif

// ─────────────────────────────────────────────────────────────────────────────
// IMU task  —  priority 3, reads BNO085 as fast as possible (I2C is slow)
// ─────────────────────────────────────────────────────────────────────────────

static void taskIMU(void * /*param*/)
{
    for (;;)
    {
        imuUpdate();   // updates volatile globals
        vTaskDelay(1); // yield every 1ms, actual rate depends on I2C speed
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Physics task — priority 5; A7 2 kHz timer, other profiles 1 kHz RTOS tick
// ─────────────────────────────────────────────────────────────────────────────

static void taskPhysics(void * /*param*/)
{
#if defined(A7_BOARD)
    const TaskHandle_t self = xTaskGetCurrentTaskHandle();
    esp_timer_handle_t timer;
    esp_timer_create_args_t timer_args = {};
    timer_args.callback = [](void *arg) { xTaskNotifyGive((TaskHandle_t)arg); };
    timer_args.arg = self;
    timer_args.name = "physics";
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 1000000 / PHYS_HZ));
#else
    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t kPeriod = pdMS_TO_TICKS(1); // 1 tick = 1 ms (1000 Hz)
#endif

    uint32_t step_count = 0;
    uint32_t hz_last_us = micros(); // use microseconds for precision
    uint32_t step_max_window_us = 0;
    uint32_t step_sum_us = 0, imu_sum_us = 0; // per-second profile

    float marble_impact_telemetry = 0.0f;
    SimMode last_mode = g_sim_mode;
    uint32_t saber_catchup=0;

    for (;;)
    {
#if defined(A7_BOARD)
        // Consume every timer tick; SPI sampling immediately precedes simulation.
        const uint32_t pending_ticks = saber_catchup ? (--saber_catchup,1u) : ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pending_ticks > 1)
        {
            telem_step_overruns += pending_ticks - 1;
            if(g_sim_mode==SimMode::LIGHTSABER || g_sim_mode==SimMode::TEXTURED || g_sim_mode==SimMode::ROLLING || g_sim_mode==SimMode::RAIN) {
                // Refill up to 1.5 ms of audio after a brief scheduling stall.
                // Dropping every delayed block produces periodic zero-filled gaps.
                saber_catchup=std::min<uint32_t>(pending_ticks-1,3);
            } else {
            // Do not spin through stale timer notifications at priority 5. One
            // RTOS tick gives serial/BLE and LED work time to run; next sample
            // is fresh, and the dropped time remains visible in diagnostics.
            vTaskDelay(1);
            telem_step_overruns += ulTaskNotifyTake(pdTRUE, 0);
            }
        }
#endif
        const uint32_t step_start_us = micros();
#if defined(A7_BOARD)
        imuUpdate();
        imu_sum_us += micros() - step_start_us;
#endif

        if(g_sim_mode != last_mode) {hapticResetTransient();if(g_sim_mode==SimMode::RAIN)s_rain.reset();}
#if defined(A7_BOARD)
        if(g_sim_mode==SimMode::RAIN){
            const int variant=s_rain_variant;
            static int last_variant=-1;
            if(variant!=last_variant){hapticResetTransient();s_rain.reset();last_variant=variant;}
            if(variant==3){
                // Moo box: a bellows piston instead of grains.
                const float piston=hapticMooBox(imu_sin_alpha);
                last_mode=SimMode::RAIN;telem_sin_alpha=imu_sin_alpha;telem_x_mm=piston*TUBE_LENGTH_M*1000;telem_x_vel=0;
            } else {
            auto rain=s_rain.step(imu_sin_alpha,imu_tube_accel/9.80665f,variant);
            hapticRainstick(rain.density,rain.energy,rain.knock,variant);
            last_mode=SimMode::RAIN;telem_sin_alpha=imu_sin_alpha;telem_x_mm=rain.position*TUBE_LENGTH_M*1000;telem_x_vel=rain.speed;
            }
        } else
        if(g_sim_mode == SimMode::LIGHTSABER) {
            float motion=hapticLightsaber(imu_swing_speed,imu_lin_accel_x,imu_lin_accel_y,imu_lin_accel_z);
            last_mode=SimMode::LIGHTSABER;
            s_saber_motion=motion;
            telem_sin_alpha=imu_sin_alpha;
            telem_x_mm=0;
            telem_x_vel=motion;
        }
        else
#endif
        if (g_sim_mode == SimMode::MARBLES)
        {
            if (last_mode != SimMode::MARBLES)
            {
                hapticResetTransient();
                marble_impact_telemetry = 0.0f;
            }
            const uint32_t round_seed = game_seed_request.exchange(0);
            if (round_seed) {
                // Use an independent RNG draw for count so the placement seed
                // does not constrain the available empty/even-numbered rounds.
                const int count = (int)(esp_random() % 6u);
                marblesInit(count, round_seed);
                game_count.store(count);
                game_revealed.store(false);
                marble_impact_telemetry = 0.0f;
                hapticResetTransient();
                game_round.fetch_add(1);
            }
            if (game_reveal_request.exchange(false)) game_revealed.store(true);
            last_mode = SimMode::MARBLES;

            // 2. Marble box physics (box acceleration + gravity)
            float impact = marblesStep(PHYS_H,
                                       imu_grav_x, imu_grav_y, imu_grav_z,
                                       imu_lin_accel_x, // box linear acceleration X [m/s²]
                                       imu_lin_accel_y, // box linear acceleration Y [m/s²]
                                       imu_lin_accel_z);// box linear acceleration Z [m/s²]

            // Shake-box: discrete collision pulses and HF ticks, no rolling bed.
            hapticUpdate(0.0f, 0.0f, false,
                         impact * (HAPTIC_IMPACT_REF / MARBLE_IMPACT_REF), true, true);
#if defined(A7_BOARD)
            if (impact > 0.0f && game_revealed.load())
                s_impact_flash_ms = millis();
#endif

            // 4. Telemetry snapshot for marble stream
            telem_grav_x = imu_grav_x / 9.80665f;
            telem_grav_y = imu_grav_y / 9.80665f;
            telem_grav_z = imu_grav_z / 9.80665f;
            // Telemetry streams at ~58 Hz, so retain contact peaks for 25 ms
            // for the visualizer. This envelope never feeds back into haptics.
            marble_impact_telemetry = marblesImpactEnvelopeStep(
                marble_impact_telemetry, impact, PHYS_H);
            telem_impact_energy = marble_impact_telemetry;
            telem_qw = imu_qw;
            telem_qx = imu_qx;
            telem_qy = imu_qy;
            telem_qz = imu_qz;
            commsPublishMarbles();
        }
        else
        {
            last_mode = g_sim_mode;
            // 2. Tube physics step
            ImpactInfo impact = physicsStep(PHYS_H, imu_sin_alpha, imu_tube_accel);

            // 3. Haptic — amplitude scales with impact energy
            hapticUpdate(phys_x_pos, phys_x_vel,
                         isRollingMode(g_sim_mode),
                         impact.hit ? impact.speed : 0.0f,
                         g_sim_mode == SimMode::TEXTURED);
#if defined(A7_BOARD)
            if (impact.hit)
                s_impact_flash_ms = millis();
#endif

            // 4. Publish telemetry snapshot for comms task
            telem_sin_alpha = imu_sin_alpha;
            telem_x_mm = phys_x_pos * 1000.0f;
            telem_x_vel = phys_x_vel;
        }

        // 5. Measure actual physics rate (updated every second using microseconds)
        step_count++;
        uint32_t now_us = micros();
        const uint32_t step_runtime_us = now_us - step_start_us;
        step_sum_us += step_runtime_us;
        if (step_runtime_us > step_max_window_us)
            step_max_window_us = step_runtime_us;
        if (step_runtime_us > (1000000UL / PHYS_HZ))
            ++telem_step_overruns;
        uint32_t elapsed_us = now_us - hz_last_us;
        if (elapsed_us >= 1000000) // 1 second in microseconds
        {
            // Calculate actual Hz with fractional precision
            telem_phys_hz = (step_count * 1000000UL) / elapsed_us;
            telem_step_us_max = step_max_window_us;
            telem_step_us_avg = step_count ? step_sum_us / step_count : 0;
            telem_imu_us_avg = step_count ? imu_sum_us / step_count : 0;
            step_sum_us = imu_sum_us = 0;
            step_max_window_us = 0;
            step_count = 0;
            hz_last_us = now_us;
        }

#if !defined(A7_BOARD)
        vTaskDelayUntil(&xLastWake, kPeriod);
#endif
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Comms task  —  priority 1
// ─────────────────────────────────────────────────────────────────────────────

static void taskComms(void * /*param*/)
{
#if defined(A7_BOARD)
    batteryWifiInit();
#endif
    uint32_t last_telem_ms = 0;
    uint32_t last_debug_ms = 0;

    for (;;)
    {
        uint32_t now = millis();

        commsHandleSerial();
#if defined(A7_BOARD)
        a7UiTick();
        batteryRuntimeTick();
#endif

        if (now - last_telem_ms >= TELEM_PERIOD_MS)
        {
            if (g_sim_mode == SimMode::MARBLES)
                commsStreamMarbles();
            else
                commsStreamTelemetry();
            last_telem_ms = now;
        }

        if (now - last_debug_ms >= DEBUG_PERIOD_MS)
        {
            commsSendStatus();
            last_debug_ms = now;
        }

        vTaskDelay(1);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup
// ─────────────────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);
#if defined(A7_BOARD)
    // A disconnected host must never stall logging. The timeout must be 1, not
    // 0: HWCDC::write() counts its retries down from this value, and from 0 the
    // counter wraps, so a full buffer with USB plugged in but no program reading
    // the port blocked forever (boot stuck on the white LED, later dead buttons).
    Serial.setTxTimeoutMs(1);
#endif
    {
        uint32_t t0 = millis();
        while (!Serial && millis() - t0 < 3000)
            delay(10);
    }

#if defined(A7_BOARD)
    // USB500: the 100 mA default browns out with the amps running (firmware-interface.md allows it
    // once USB allocation is granted; bench use here)
    pinMode(PIN_USB_500, OUTPUT);
    digitalWrite(PIN_USB_500, HIGH);
    pinMode(PIN_BTN_MODE, INPUT); // external pull-ups
    pinMode(PIN_BTN_GAIN, INPUT);
    batteryRuntimeInit();
    Wire.begin(PIN_SDA, PIN_SCL); // PCA9632 handle RGB
    Wire.setClock(400000);
    pcaInit();
    neoSet(16, 16, 16); // dim white = booting
#elif !defined(HBRIDGE)
    s_neo.begin();
    neoSet(16, 16, 16); // dim white = booting
#endif

    // ── Power on sensor / LED ────────────────────────────────────────────────
#if defined(A7_BOARD)
    // LSM6DS3TR-C is on SPI and always powered
#elif defined(HBRIDGE)
    pinMode(PIN_LED_ON, OUTPUT);
    digitalWrite(PIN_LED_ON, HIGH);

    gpio_hold_dis((gpio_num_t)PIN_SENS_ON);
    pinMode(PIN_SENS_ON, OUTPUT);
    digitalWrite(PIN_SENS_ON, HIGH);
#else
    pinMode(PIN_SENS_ON, OUTPUT);
    digitalWrite(PIN_SENS_ON, HIGH);
#endif

    // ── I2C ──────────────────────────────────────────────────────────────────
    // Release I2C pins from any deep-sleep hold before configuring Wire
#ifdef HBRIDGE
    gpio_hold_dis((gpio_num_t)PIN_SDA);
    gpio_hold_dis((gpio_num_t)PIN_SCL);
#endif
#if !defined(A7_BOARD)
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    delay(500); // BNO085 needs up to ~800 ms after power-on; imuInit retries the rest
#endif

    // ── IMU ──────────────────────────────────────────────────────────────────
#if defined(A7_BOARD)
    Serial.println("# imu: init LSM6DS3TR-C (keep the board still: gyro calibration) …");
#else
    Serial.println("# imu: init BNO085 …");
#endif
    if (!imuInit(Wire))
    {
        Serial.println("# imu: FAILED — check wiring!");
#ifndef HBRIDGE
        neoSet(64, 0, 0); // red = fatal
#endif
        while (1)
            delay(10);
    }
    Serial.println("# imu: ok");

    // ── Init modules ─────────────────────────────────────────────────────────
    physicsInit();
    marblesInit();
    hapticInit(); // configures LEDC (HBRIDGE), I2S channel (feather) or TDM (A7)
    commsInit();  // serial + BLE

    // ── Banner ───────────────────────────────────────────────────────────────
    Serial.println("\n================================");
    Serial.printf("Virtual Rolling Stone — %s\n",
#if defined(A7_BOARD)
                  "Rolling Stone A7 (TDM, TacHammer)"
#elif defined(HBRIDGE)
                  "H-bridge PWM"
#else
                  "Feather I2S audio"
#endif
    );
    Serial.println("================================");
    {
        esp_reset_reason_t rr = esp_reset_reason();
#if defined(A7_BOARD)
        // persistent counters so brownouts are visible without watching the serial port
        Preferences p;
        p.begin("a7ball", false);
        uint32_t boots = p.getUInt("boots", 0) + 1, brown = p.getUInt("brown", 0) + (rr == ESP_RST_BROWNOUT);
        p.putUInt("boots", boots);
        p.putUInt("brown", brown);
        p.end();
        g_a7_brownouts = brown;
        Serial.printf("# reset_reason=%d%s  boots=%lu brownouts=%lu\n", (int)rr, rr == ESP_RST_BROWNOUT ? " BROWNOUT" : "",
                      (unsigned long)boots, (unsigned long)brown);
        if (rr == ESP_RST_BROWNOUT)
            for (int i = 0; i < 3; i++) // red blinks = the last reset was a brownout
            {
                neoSet(120, 0, 0);
                delay(150);
                neoSet(0, 0, 0);
                delay(150);
            }
#else
        Serial.printf("# reset_reason=%d%s\n", (int)rr,
                      rr == ESP_RST_BROWNOUT ? " BROWNOUT" : rr == ESP_RST_PANIC ? " PANIC" : rr == ESP_RST_POWERON ? " power-on" : "");
#endif
    }
    Serial.printf("# mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f\n",
                  simModeName(g_sim_mode),
                  g_cavity * 1000.0f, g_rebound ? 1 : 0, G_FACTOR, FRICTION_MU);
    Serial.printf("# physics=%d Hz  telem=%lu Hz\n",
                  PHYS_HZ, 1000UL / TELEM_PERIOD_MS);

#ifndef HBRIDGE
    neoSet(0, 24, 0); // green = running
#endif

    // ── Launch FreeRTOS tasks ────────────────────────────────────────────────
    // ESP32-C6 is single-core; remove loopTask from TWDT so portMAX_DELAY in
    // loop() doesn't trigger the 10-second watchdog reset.
    esp_task_wdt_delete(NULL);

#if !defined(A7_BOARD)
    xTaskCreate(taskIMU, "imu", 4096, nullptr, 3, nullptr);
#endif
    xTaskCreate(taskPhysics, "physics", 8192, nullptr, 5, nullptr);
    xTaskCreate(taskComms, "comms", 8192, nullptr, 1, nullptr);
}

void loop() { vTaskDelay(portMAX_DELAY); }
