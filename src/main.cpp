/*
 * Virtual Rolling Stone — ESP32-C6 Haptic Firmware
 *
 * Reproduces Yao & Hayward, "An Experiment on Length Perception with a
 * Virtual Rolling Stone", Eurohaptics 2006, pp. 325-330.
 *
 * Two build targets (platformio.ini):
 *   feather   — STEMMA BNO085 + I2S audio amp + NeoPixel
 *   hbridge   — onboard BNO085 + H-bridge PWM  (-DHBRIDGE)
 *
 * Architecture (ESP32-C6 = single core):
 *   taskPhysics (priority 5, 1 kHz) — IMU → physics → haptic → telemetry snapshot
 *   taskComms   (priority 1, low)   — serial/BLE commands + ~58 Hz telemetry stream
 *
 * Serial protocol (companion app):
 *   Firmware → host: "$sin_a,x_mm,v_mps\n"  at ~58 Hz
 *                    "# key=val …\n"          at  1 Hz and on command replies
 *   Host → firmware: 'r' ROLLING | 's' SLIDING | 'b' toggle rebound
 *                    '+' cavity+5cm | '-' cavity-5cm | '?' dump params
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

#include "config.h"
#include "imu.h"
#include "physics.h"
#include "haptic.h"
#include "comms.h"
#include "marbles.h"

#ifdef HBRIDGE
#include "driver/gpio.h" // gpio_hold_dis
#endif

#ifndef HBRIDGE
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
// Physics task  —  priority 5, 1 kHz via vTaskDelayUntil
// ─────────────────────────────────────────────────────────────────────────────

static void taskPhysics(void * /*param*/)
{
    TickType_t xLastWake = xTaskGetTickCount();
    const TickType_t kPeriod = pdMS_TO_TICKS(1); // 1 tick = 1 ms (1000 Hz)

    uint32_t step_count = 0;
    uint32_t hz_last_us = micros(); // use microseconds for precision

    for (;;)
    {
        // 1. IMU data is read by separate task, just use latest values

        if (g_sim_mode == SimMode::MARBLES)
        {
            // 2. Marble box physics (box acceleration + gravity)
            float impact = marblesStep(PHYS_H,
                                       imu_lin_accel_x, // box acceleration X [m/s²]
                                       imu_lin_accel_y, // box acceleration Y [m/s²]
                                       imu_lin_accel_z, // box acceleration Z [m/s²]
                                       imu_grav_z);     // gravity Z [m/s²]

            // 3. Haptic — scale marble impact speed to tube haptic reference
            hapticUpdate(0.0f, 0.0f, false,
                         impact * (HAPTIC_IMPACT_REF / MARBLE_IMPACT_REF));

            // 4. Telemetry snapshot for marble stream
            telem_grav_x = imu_lin_accel_x / 9.8f; // normalize for display
            telem_grav_y = imu_lin_accel_y / 9.8f;
            telem_grav_z = (imu_lin_accel_z + imu_grav_z) / 9.8f;
            telem_impact_energy = fminf(impact / MARBLE_IMPACT_REF, 1.0f);
            telem_qw = imu_qw;
            telem_qx = imu_qx;
            telem_qy = imu_qy;
            telem_qz = imu_qz;
        }
        else
        {
            // 2. Tube physics step
            ImpactInfo impact = physicsStep(PHYS_H, imu_sin_alpha, imu_lin_accel_x);

            // 3. Haptic — amplitude scales with impact energy
            hapticUpdate(phys_x_pos, phys_x_vel,
                         g_sim_mode == SimMode::ROLLING,
                         impact.hit ? impact.speed : 0.0f);

            // 4. Publish telemetry snapshot for comms task
            telem_sin_alpha = imu_sin_alpha;
            telem_x_mm = phys_x_pos * 1000.0f;
            telem_x_vel = phys_x_vel;
        }

        // 5. Measure actual physics rate (updated every second using microseconds)
        step_count++;
        uint32_t now_us = micros();
        uint32_t elapsed_us = now_us - hz_last_us;
        if (elapsed_us >= 1000000) // 1 second in microseconds
        {
            // Calculate actual Hz with fractional precision
            telem_phys_hz = (step_count * 1000000UL) / elapsed_us;
            step_count = 0;
            hz_last_us = now_us;
        }

        vTaskDelayUntil(&xLastWake, kPeriod);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Comms task  —  priority 1
// ─────────────────────────────────────────────────────────────────────────────

static void taskComms(void * /*param*/)
{
    uint32_t last_telem_ms = 0;
    uint32_t last_debug_ms = 0;

    for (;;)
    {
        uint32_t now = millis();

        commsHandleSerial();

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
    {
        uint32_t t0 = millis();
        while (!Serial && millis() - t0 < 3000)
            delay(10);
    }

#ifndef HBRIDGE
    s_neo.begin();
    neoSet(16, 16, 16); // dim white = booting
#endif

    // ── Power on sensor / LED ────────────────────────────────────────────────
#ifdef HBRIDGE
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
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    delay(500); // BNO085 needs up to ~800 ms after power-on; imuInit retries the rest

    // ── IMU ──────────────────────────────────────────────────────────────────
    Serial.println("# imu: init BNO085 …");
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
    hapticInit(); // configures LEDC (HBRIDGE) or I2S channel (feather)
    commsInit();  // serial + BLE

    // ── Banner ───────────────────────────────────────────────────────────────
    Serial.println("\n================================");
    Serial.printf("Virtual Rolling Stone — %s\n",
#ifdef HBRIDGE
                  "H-bridge PWM"
#else
                  "Feather I2S audio"
#endif
    );
    Serial.println("================================");
    Serial.printf("# mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f\n",
                  g_sim_mode == SimMode::ROLLING ? "ROLLING" : g_sim_mode == SimMode::SLIDING ? "SLIDING"
                                                                                              : "MARBLES",
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

    xTaskCreate(taskIMU, "imu", 4096, nullptr, 3, nullptr);
    xTaskCreate(taskPhysics, "physics", 8192, nullptr, 5, nullptr);
    xTaskCreate(taskComms, "comms", 8192, nullptr, 1, nullptr);
}

void loop() { vTaskDelay(portMAX_DELAY); }
