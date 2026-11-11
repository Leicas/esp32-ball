/*
 * Rolling Stone A7 PCB — hardware bring-up / self-test firmware
 *
 * Target: custom A7 board (esp32-ball-hardware), ESP32-C6-MINI-1-H4.
 * Build:  pio run -e a7_bringup --target upload --upload-port COMxx
 *
 * Verifies, over USB serial (native USB-Serial/JTAG):
 *   - PCA9632 handle RGB LED driver  (I2C 0x62, SDA=IO6 SCL=IO7)
 *   - LSM6DS3TR-C IMU                (SPI SCK=IO14 MOSI=IO15 MISO=IO18 CS=IO19, INT1=IO2)
 *   - VSYS ADC                       (IO4, divide-by-2)
 *   - USB_GOOD_N charger input       (IO3)
 *   - MODE / GAIN / BOOT buttons     (IO0 / IO1 / IO9, active low)
 *   - MAX98357A x4 TDM haptic amps   (BCLK=IO21 FS=IO22 DATA=IO23, SD=IO5)  — on command only
 *   - Fitted actuators (LFi, HF)     ('t': frequency sweep per channel, vibration
 *                                     measured by the on-board IMU at 1.66 kHz)
 *   - Addressable blade data         (IO20 via 5 V buffer)                   — on command only
 *
 * Pin map source: esp32-ball-hardware/firmware-interface.md, cross-checked
 * against kicad/circuit.json (U5 module pins → nets).
 *
 * Safety policy from firmware-interface.md:
 *   - USB_500 (IO17) HIGH by default → USB500 input limit (USB100 browns out at
 *     0.25 FS haptics without a battery); 'u' toggles back to USB100
 *   - HAPTIC_EN (IO5) LOW at boot; only raised after TDM clocks run with zeros
 *   - haptic amplitude capped at AMP_MAX full-scale; auto-off after HAPTIC_MAX_ON_MS
 */

#include <Arduino.h>
#include "../threeway_filter.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/i2s_tdm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ─────────────────────────────────────────────────────────────────────────────
// Pin map (ESP32-C6 GPIO numbers)
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int PIN_BTN_MODE = 0;
static constexpr int PIN_BTN_GAIN = 1;
static constexpr int PIN_IMU_INT = 2;
static constexpr int PIN_USB_GOOD_N = 3;
static constexpr int PIN_VSYS_ADC = 4;
static constexpr int PIN_HAPTIC_EN = 5;
static constexpr int PIN_SDA = 6;
static constexpr int PIN_SCL = 7;
static constexpr int PIN_BTN_BOOT = 9;
static constexpr int PIN_IMU_SCK = 14;
static constexpr int PIN_IMU_MOSI = 15;
static constexpr int PIN_USB_500 = 17;
static constexpr int PIN_IMU_MISO = 18;
static constexpr int PIN_IMU_CS = 19;
static constexpr int PIN_RGB_DATA = 20;
static constexpr int PIN_TDM_BCLK = 21;
static constexpr int PIN_TDM_FS = 22;
static constexpr int PIN_TDM_DATA = 23;

// ─────────────────────────────────────────────────────────────────────────────
// Tunables
// ─────────────────────────────────────────────────────────────────────────────
static constexpr uint8_t PCA9632_ADDR = 0x62;
static constexpr uint8_t LSM6_WHO_AM_I = 0x6A;     // LSM6DS3TR-C (and LSM6DSL); plain LSM6DS3 = 0x69
static constexpr uint8_t LSM6_WHO_AM_I_ALT = 0x69;
static constexpr uint32_t SPI_HZ = 4000000;

static constexpr int BLADE_PIXELS = 8; // pixels lit by the 'p' test

static constexpr uint32_t TDM_RATE = 48000;
static constexpr int TDM_SLOTS = 8;
static constexpr int TDM_BLOCK_FRAMES = 256;
static constexpr float AMP_DEFAULT = 0.10f; // fraction of full scale (~0.4 Vrms @ 12 dB)
static constexpr float AMP_MAX = 1.0f;     // full-scale test limit (8-ohm Drake, 3.3 V rail) — raise only after bench measurement
static constexpr float AMP_STEP = 0.05f;
static constexpr float RAMP_PER_SAMPLE = 1.0f / (0.020f * TDM_RATE); // 20 ms ramp
static constexpr uint32_t HAPTIC_MAX_ON_MS = 3000;
static const float CH_DEFAULT_HZ[4] = {80.0f, 150.0f, 250.0f, 60.0f}; // LFi, MF, HF, LF
static const char *CH_NAME[4] = {"LFi(J3,U7)", "MF(J4,U8)", "HF(J5,U10)", "LF(J6,U11)"};
static const bool CH_FITTED[4] = {true, true, true, true}; // fitted: LFi, MF, HF, LF

// Actuator test ('t'): sweep each fitted channel, compare IMU vibration RMS to a silent baseline
static const float ACT_TEST_HZ[] = {50.0f, 80.0f, 125.0f, 175.0f, 250.0f, 320.0f};
static constexpr int ACT_TEST_NF = sizeof(ACT_TEST_HZ) / sizeof(ACT_TEST_HZ[0]);
static constexpr uint32_t ACT_MEAS_MS = 250;
static constexpr float ACT_PASS_RATIO = 3.0f;  // tone RMS / baseline RMS
static constexpr float ACT_PASS_MIN_MG = 2.0f; // and an absolute floor

// Full sweep ('W'): log-spaced lock-in measurement, streamed as $sw CSV for companion/actuator_sweep.py
static constexpr float SWEEP_F0 = 20.0f, SWEEP_F1 = 500.0f;
static constexpr int SWEEP_NF = 40;
static constexpr float SWEEP_MEAS_S = 0.20f; // rounded up to whole cycles
static constexpr float IMU_NYQ_HZ = 800.0f;  // 1.66 kHz ODR
static constexpr uint8_t SWEEP_CTRL1_XL = 0x84;  // 1.66 kHz, ±16 g for full-drive measurements
static constexpr double SWEEP_MG_PER_LSB = 0.488;
static volatile bool g_signature_sweep=false;
static uint32_t g_signature_t[1024];
static int16_t g_signature_xyz[1024][3];
static int g_signature_n=0;

// Continuous chirp stream ('C'): exponential chirp per mode, raw accel streamed from the IMU FIFO
// for companion/chirp_view.py. Frequency is updated once per TDM block (5.3 ms).
static constexpr float CHIRP_F0 = 20.0f, CHIRP_F1 = 500.0f, CHIRP_S = 6.0f;
static constexpr uint32_t CHIRP_GAP_MS = 400;
static constexpr uint32_t CHIRP_NBLK = (uint32_t)(CHIRP_S * TDM_RATE / TDM_BLOCK_FRAMES);

// ─────────────────────────────────────────────────────────────────────────────
// State
// ─────────────────────────────────────────────────────────────────────────────
static bool g_pca_ok = false, g_imu_ok = false;
static Adafruit_NeoPixel g_blade(BLADE_PIXELS, PIN_RGB_DATA, NEO_GRB + NEO_KHZ800);
static volatile uint32_t g_imu_irq_count = 0;
static bool g_led_auto = true; // breathing status colour vs manual colour
static uint8_t g_led_r = 0, g_led_g = 0, g_led_b = 0;
static bool g_accel_stream = false;

// haptic
static i2s_chan_handle_t g_tx = nullptr;
static bool g_tdm_ok = false;
static esp_err_t g_tdm_enable_err = ESP_OK;
static volatile uint32_t g_aud_blocks = 0;
static volatile esp_err_t g_aud_err = ESP_OK;
static volatile uint32_t g_loop_iters = 0;
static volatile bool g_tdm_running = false;
static volatile float g_amp_target[4] = {0, 0, 0, 0};
static volatile bool g_ch_invert[4] = {false, false, false, false};
static portMUX_TYPE g_amp_mux = portMUX_INITIALIZER_UNLOCKED; // targets change atomically across channels
static bool g_usb500 = true;
static volatile bool g_slot_mirror = false;
static volatile bool g_chirp_on = false;    // audio task sweeps g_freq[] while set
static volatile uint32_t g_chirp_blk = 0;   // blocks since chirp start (audio task increments)
static float g_freq[4] = {CH_DEFAULT_HZ[0], CH_DEFAULT_HZ[1], CH_DEFAULT_HZ[2], CH_DEFAULT_HZ[3]};
static float g_amp_level = AMP_DEFAULT;
static uint32_t g_haptic_on_since = 0;
static int16_t g_tdm_buf[TDM_BLOCK_FRAMES * TDM_SLOTS];

struct ActResult
{
    bool run;
    bool pass;
    float base_mg, best_mg, best_hz;
};
static ActResult g_act[4] = {};

static void IRAM_ATTR imuIsr() { g_imu_irq_count++; }

// ─────────────────────────────────────────────────────────────────────────────
// PCA9632 handle RGB (common anode on 3V3, open-drain sink)
//   LED0 = blue, LED1 = red, LED2 = green  (per firmware-interface.md)
// ─────────────────────────────────────────────────────────────────────────────
static bool pcaWrite(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(PCA9632_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}
static int pcaRead(uint8_t reg)
{
    Wire.beginTransmission(PCA9632_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return -1;
    if (Wire.requestFrom((int)PCA9632_ADDR, 1) != 1)
        return -1;
    return Wire.read();
}
static bool pcaInit()
{
    if (!pcaWrite(0x01, 0x00)) // MODE2: open-drain, non-inverted
        return false;
    pcaWrite(0x02, 0); // PWM0..2 = 0
    pcaWrite(0x03, 0);
    pcaWrite(0x04, 0);
    pcaWrite(0x05, 0);
    pcaWrite(0x08, 0x00); // LEDOUT: all off
    pcaWrite(0x00, 0x01); // MODE1: SLEEP=0, ALLCALL=1
    delayMicroseconds(600); // oscillator start-up (datasheet: 500 µs)
    pcaWrite(0x08, 0x2A);   // LEDOUT: LED0..2 individual PWM, LED3 off
    int m1 = pcaRead(0x00);
    return m1 >= 0 && (m1 & 0x10) == 0;
}
static void ledSet(uint8_t r, uint8_t g, uint8_t b)
{
    g_led_r = r; g_led_g = g; g_led_b = b;
    if (!g_pca_ok)
        return;
    Wire.beginTransmission(PCA9632_ADDR);
    Wire.write(0x80 | 0x02); // auto-increment from PWM0
    Wire.write(b);           // LED0 blue
    Wire.write(r);           // LED1 red
    Wire.write(g);           // LED2 green
    Wire.endTransmission();
}

// ─────────────────────────────────────────────────────────────────────────────
// LSM6DS3TR-C over SPI (mode 3)
// ─────────────────────────────────────────────────────────────────────────────
static const SPISettings g_spi_set(SPI_HZ, MSBFIRST, SPI_MODE3);

static uint8_t imuRead8(uint8_t reg)
{
    SPI.beginTransaction(g_spi_set);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
    return v;
}
static void imuWrite8(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(g_spi_set);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}
static void imuReadBurst(uint8_t reg, uint8_t *dst, size_t n)
{
    SPI.beginTransaction(g_spi_set);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg | 0x80);
    for (size_t i = 0; i < n; i++)
        dst[i] = SPI.transfer(0x00);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}
// Probe WHO_AM_I under several SPI modes/speeds and dump raw bytes.
// 0x69 = LSM6DS3TR-C; 0x00 = MISO stuck low / no response; 0xFF = MISO floating.
static void imuProbe()
{
    const uint8_t modes[2] = {SPI_MODE3, SPI_MODE0};
    const uint32_t speeds[2] = {1000000, 4000000};
    for (int m = 0; m < 2; m++)
        for (int sp = 0; sp < 2; sp++)
        {
            SPISettings st(speeds[sp], MSBFIRST, modes[m]);
            uint8_t rx[4];
            SPI.beginTransaction(st);
            digitalWrite(PIN_IMU_CS, LOW);
            delayMicroseconds(5);
            SPI.transfer(0x0F | 0x80); // WHO_AM_I read
            for (int i = 0; i < 4; i++)
                rx[i] = SPI.transfer(0x00); // WHO_AM_I, then 0x10.. with IF_INC? (no) → repeats/garbage
            digitalWrite(PIN_IMU_CS, HIGH);
            SPI.endTransaction();
            Serial.printf("# imu probe: mode%d %lu Hz  WHO_AM_I=0x%02X  next=%02X %02X %02X\n",
                          modes[m] == SPI_MODE3 ? 3 : 0, (unsigned long)speeds[sp], rx[0], rx[1], rx[2], rx[3]);
        }
    Serial.printf("# imu probe: CTRL3_C=0x%02X (default 0x04)  INT1(IO2)=%d\n",
                  imuRead8(0x12), digitalRead(PIN_IMU_INT));
}

static bool imuInit()
{
    uint8_t who = imuRead8(0x0F);
    Serial.printf("# imu: WHO_AM_I=0x%02X (expect 0x%02X LSM6DS3TR-C, or 0x%02X LSM6DS3)\n",
                  who, LSM6_WHO_AM_I, LSM6_WHO_AM_I_ALT);
    if (who != LSM6_WHO_AM_I && who != LSM6_WHO_AM_I_ALT)
        return false;
    imuWrite8(0x12, 0x01); // CTRL3_C: SW_RESET
    delay(20);
    // CTRL3_C: BDU | H_LACTIVE | PP_OD | IF_INC  → INT1 active-low open-drain
    imuWrite8(0x12, 0x40 | 0x20 | 0x10 | 0x04);
    imuWrite8(0x10, 0x40); // CTRL1_XL: 104 Hz, ±2 g
    imuWrite8(0x11, 0x40); // CTRL2_G : 104 Hz, 245 dps
    imuWrite8(0x0D, 0x01); // INT1_CTRL: accel data-ready on INT1
    return true;
}
static void imuSample(float acc_g[3], float gyr_dps[3])
{
    uint8_t raw[12];
    imuReadBurst(0x22, raw, 12); // OUTX_L_G .. OUTZ_H_XL
    for (int i = 0; i < 3; i++)
    {
        int16_t gv = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
        int16_t av = (int16_t)(raw[6 + 2 * i] | (raw[7 + 2 * i] << 8));
        gyr_dps[i] = gv * 0.00875f;  // 8.75 mdps/LSB @ 245 dps
        acc_g[i] = av * 0.000061f;   // 0.061 mg/LSB @ ±2 g
    }
}

// Vibration RMS (mg, summed over axes, DC removed) with the accel at 1.66 kHz.
// Caller must have set CTRL1_XL to 1.66 kHz first.
static float imuVibRms(uint32_t ms, int *n_out = nullptr, double mg_per_lsb = 0.061)
{
    int64_t sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
    int n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < ms)
    {
        if (!(imuRead8(0x1E) & 0x01)) // STATUS_REG.XLDA
            continue;
        uint8_t raw[6];
        imuReadBurst(0x28, raw, 6); // OUTX_L_XL .. OUTZ_H_XL
        for (int i = 0; i < 3; i++)
        {
            int32_t v = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
            sum[i] += v;
            sq[i] += (int64_t)v * v;
        }
        n++;
    }
    if (n_out)
        *n_out = n;
    if (n < 2)
        return -1.0f;
    double var = 0;
    for (int i = 0; i < 3; i++)
    {
        double m = (double)sum[i] / n;
        var += (double)sq[i] / n - m * m;
    }
    return (float)(sqrt(var > 0 ? var : 0) * mg_per_lsb); // default 0.061 mg/LSB @ ±2 g
}

// ─────────────────────────────────────────────────────────────────────────────
// Power / inputs
// ─────────────────────────────────────────────────────────────────────────────
static float readVsys()
{
    uint32_t acc = 0;
    for (int i = 0; i < 8; i++)
        acc += analogReadMilliVolts(PIN_VSYS_ADC);
    return (acc / 8) * 2.0f / 1000.0f; // R20/R21 = 100k/100k
}

// ─────────────────────────────────────────────────────────────────────────────
// TDM haptics — 48 kHz, 8 × 16-bit slots, 128 BCLK/frame (6.144 MHz)
// MAX98357A TDM = one-BCLK-wide FS pulse, MSB one BCLK later → PCM-short preset
// ─────────────────────────────────────────────────────────────────────────────
static bool tdmInit()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true; // underflow → zeros, never stale samples
    if (i2s_new_channel(&chan_cfg, &g_tx, nullptr) != ESP_OK)
        return false;

    i2s_tdm_config_t cfg = {
        .clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(TDM_RATE),
        .slot_cfg = I2S_TDM_PCM_SHORT_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO,
            (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3 |
                                  I2S_TDM_SLOT4 | I2S_TDM_SLOT5 | I2S_TDM_SLOT6 | I2S_TDM_SLOT7)),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)PIN_TDM_BCLK,
            .ws = (gpio_num_t)PIN_TDM_FS,
            .dout = (gpio_num_t)PIN_TDM_DATA,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {.mclk_inv = 0, .bclk_inv = 0, .ws_inv = 0},
        },
    };
    cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_512; // MCLK 24.576 MHz, BCLK = MCLK/4
    cfg.slot_cfg.total_slot = TDM_SLOTS;
    cfg.slot_cfg.ws_width = 1;
    esp_err_t e = i2s_channel_init_tdm_mode(g_tx, &cfg);
    if (e != ESP_OK)
    {
        Serial.printf("# tdm: init_tdm_mode failed: %s\n", esp_err_to_name(e));
        return false;
    }
    return true;
}

static void hapticStart()
{
    if (g_tdm_running)
        return;
    memset(g_tdm_buf, 0, sizeof(g_tdm_buf));
    size_t n = 0;
    i2s_channel_preload_data(g_tx, g_tdm_buf, sizeof(g_tdm_buf), &n);
    g_tdm_enable_err = i2s_channel_enable(g_tx);
    g_tdm_running = true;
    delay(10);                          // valid clocks + zeros before un-muting the amps
    digitalWrite(PIN_HAPTIC_EN, HIGH);  // SD_MODE high → amps on (gain fixed 12 dB in TDM)
    delay(10);
    Serial.println("# haptic: TDM clocks running, HAPTIC_EN=1");
}
static void hapticStop()
{
    for (int i = 0; i < 4; i++)
        g_amp_target[i] = 0;
    if (!g_tdm_running)
        return;
    delay(40); // let the audio task ramp to zero
    digitalWrite(PIN_HAPTIC_EN, LOW);
    delay(10);
    g_tdm_running = false;
    delay(20); // audio task sees the flag and stops writing
    i2s_channel_disable(g_tx);
    Serial.println("# haptic: stopped, HAPTIC_EN=0");
}
static void hapticPlay(int ch)
{
    hapticStart();
    for (int i = 0; i < 4; i++)
        g_amp_target[i] = (i == ch) ? g_amp_level : 0.0f;
    g_haptic_on_since = millis();
    Serial.printf("# haptic: ch%d %s  %.0f Hz  amp=%.2f FS  (auto-off in %lu ms)\n",
                  ch + 1, CH_NAME[ch], g_freq[ch], g_amp_level, HAPTIC_MAX_ON_MS);
}

// Sweep each fitted actuator and check the IMU feels it. Blocks ~5 s.
static void actuatorTest(bool all_channels = false)
{
    if (!g_tdm_ok)
    {
        Serial.println("# act: TDM not initialised — skipped");
        return;
    }
    Serial.printf("# --- actuator test (amp=%.2f FS, IMU %s) ---------------------\n",
                  g_amp_level, g_imu_ok ? "1.66 kHz" : "UNAVAILABLE — feel check only");
    if (g_imu_ok)
    {
        imuWrite8(0x10, 0x80); // CTRL1_XL: 1.66 kHz, ±2 g
        delay(20);
    }
    for (int i = 0; i < 4; i++)
        g_amp_target[i] = 0;
    hapticStart(); // amps on, silent → baseline includes idle amp/clock coupling
    delay(50);

    for (int ch = 0; ch < 4; ch++)
    {
        if (!CH_FITTED[ch] && !all_channels)
            continue;
        ActResult &r = g_act[ch];
        r = {};
        r.run = true;
        float saved_hz = g_freq[ch];
        int n = 0;
        delay(300); // let the previous channel ring out so it doesn't inflate this baseline
        r.base_mg = g_imu_ok ? imuVibRms(ACT_MEAS_MS, &n) : 0;
        Serial.printf("# act: ch%d %-11s baseline %.2f mg (%d samples)\n", ch + 1, CH_NAME[ch], r.base_mg, n);
        for (int k = 0; k < ACT_TEST_NF; k++)
        {
            g_freq[ch] = ACT_TEST_HZ[k];
            g_amp_target[ch] = g_amp_level;
            delay(80); // 20 ms ramp + mechanical settle
            float mg = g_imu_ok ? imuVibRms(ACT_MEAS_MS) : 0;
            if (!g_imu_ok)
                delay(ACT_MEAS_MS);
            g_amp_target[ch] = 0;
            float ratio = r.base_mg > 0 ? mg / r.base_mg : 0;
            Serial.printf("# act:   %3.0f Hz  %6.2f mg  x%.1f\n", ACT_TEST_HZ[k], mg, ratio);
            if (mg > r.best_mg)
            {
                r.best_mg = mg;
                r.best_hz = ACT_TEST_HZ[k];
            }
            delay(60); // ramp down + ring-out before next step
        }
        g_freq[ch] = saved_hz;
        r.pass = g_imu_ok && r.best_mg >= ACT_PASS_MIN_MG && r.best_mg >= ACT_PASS_RATIO * r.base_mg;
        Serial.printf("# act: ch%d %-11s %s  peak %.2f mg @ %.0f Hz\n", ch + 1, CH_NAME[ch],
                      g_imu_ok ? (r.pass ? "PASS" : "FAIL") : "NOT MEASURED", r.best_mg, r.best_hz);
    }

    hapticStop();
    if (g_imu_ok)
        imuWrite8(0x10, 0x40); // CTRL1_XL back to 104 Hz, ±2 g
    Serial.println("# ----------------------------------------------------------");
}

static void usb500Set(bool on)
{
    g_usb500 = on;
    digitalWrite(PIN_USB_500, on ? HIGH : LOW);
    Serial.printf("# usb input limit: %s\n", on ? "USB500 (IO17=1)" : "USB100 (IO17=0)");
}

// Set several channel targets in one step so the audio task starts them on the same sample
// (phase accumulators restart at 0 from silence → channels driven together are phase-locked).
static void ampTargets(float a0, float a1, float a2, float a3)
{
    portENTER_CRITICAL(&g_amp_mux);
    g_amp_target[0] = a0; g_amp_target[1] = a1; g_amp_target[2] = a2; g_amp_target[3] = a3;
    portEXIT_CRITICAL(&g_amp_mux);
}

// Lock-in measurement at f, 2f, 3f on all three accel axes (accel must be at SWEEP_CTRL1_XL).
// DFT uses each sample's own timestamp, so ODR tolerance and polling jitter don't bias the bin.
struct ToneResp
{
    float h_mg[3]; // peak amplitude of the vector response at f, 2f, 3f (-1 = above Nyquist)
    float rms_mg;  // broadband AC RMS
    int n;
};
static ToneResp imuToneResp(float f, float min_s)
{
    g_signature_n=0;
    float c[3][3] = {}, s[3][3] = {}, sc[3] = {}, ss[3] = {}; // [harmonic][axis]
    int64_t sum[3] = {0, 0, 0}, sq[3] = {0, 0, 0};
    int n = 0;
    const int64_t dur_us = (int64_t)(ceilf(min_s * f) / f * 1e6f);
    const int64_t t0 = esp_timer_get_time();
    for (;;)
    {
        if (!(imuRead8(0x1E) & 0x01))
            continue;
        int64_t t = esp_timer_get_time() - t0;
        if (t >= dur_us)
            break;
        uint8_t raw[6];
        imuReadBurst(0x28, raw, 6);
        if(g_signature_sweep && g_signature_n<1024) {
            g_signature_t[g_signature_n]=(uint32_t)t;
            for(int axis=0;axis<3;++axis)
                g_signature_xyz[g_signature_n][axis]=(int16_t)(raw[2*axis]|(raw[2*axis+1]<<8));
            ++g_signature_n;
        }
        double cyc = (double)f * (double)t * 1e-6;
        float ph = (float)(2.0 * M_PI * (cyc - floor(cyc)));
        float c1 = cosf(ph), s1 = sinf(ph);
        float ch[3] = {c1, c1 * c1 - s1 * s1, 0}, sh[3] = {s1, 2 * s1 * c1, 0};
        ch[2] = ch[1] * c1 - sh[1] * s1;
        sh[2] = sh[1] * c1 + ch[1] * s1;
        for (int h = 0; h < 3; h++)
        {
            sc[h] += ch[h];
            ss[h] += sh[h];
        }
        for (int i = 0; i < 3; i++)
        {
            int32_t v = (int16_t)(raw[2 * i] | (raw[2 * i + 1] << 8));
            sum[i] += v;
            sq[i] += (int64_t)v * v;
            for (int h = 0; h < 3; h++)
            {
                c[h][i] += v * ch[h];
                s[h][i] += v * sh[h];
            }
        }
        n++;
    }
    ToneResp r = {};
    r.n = n;
    if (n < 8)
    {
        r.h_mg[0] = r.h_mg[1] = r.h_mg[2] = r.rms_mg = -1;
        return r;
    }
    double var = 0;
    for (int h = 0; h < 3; h++)
    {
        if ((h + 1) * f > IMU_NYQ_HZ * 0.9f)
        {
            r.h_mg[h] = -1;
            continue;
        }
        double p = 0;
        for (int i = 0; i < 3; i++)
        {
            double m = (double)sum[i] / n; // remove DC leakage into the bin
            double cr = c[h][i] - m * sc[h], si = s[h][i] - m * ss[h];
            p += cr * cr + si * si;
        }
        r.h_mg[h] = (float)(2.0 * sqrt(p) / n * SWEEP_MG_PER_LSB);
    }
    for (int i = 0; i < 3; i++)
    {
        double m = (double)sum[i] / n;
        var += (double)sq[i] / n - m * m;
    }
    r.rms_mg = (float)(sqrt(var > 0 ? var : 0) * SWEEP_MG_PER_LSB);
    return r;
}

// Host-driven characterization: Q mask hz amp seconds invertMask.
// Binary records are timestamp-us + XYZ int16 (10 bytes), sent only after silence.
static uint32_t g_char_t[14000];
static int16_t g_char_xyz[14000][3];
static volatile bool g_characterize=false;
static volatile int g_char_shape=0;
static void characterize(int mask,float hz,float amp,float seconds,int invertMask) {
 if(!g_tdm_ok||!g_imu_ok||mask<0||mask>15||hz<0||hz>20000||seconds<.08f||seconds>4.f){Serial.println("$err,parameters");return;}
 amp=fminf(1.f,fmaxf(0.f,amp));
 ampTargets(0,0,0,0);hapticStart();delay(40);
 imuWrite8(0x10,0x94); // 3.33kHz +/-16g, analog BW1.5kHz
 imuWrite8(0x15,0x00);imuWrite8(0x17,0x00); // high performance, default LPF1
 g_char_shape=invertMask&48;g_characterize=true;
 for(int ch=0;ch<4;++ch){g_freq[ch]=hz;g_ch_invert[ch]=(invertMask&(1<<ch))!=0;}
 float a[4];for(int ch=0;ch<4;++ch)a[ch]=(mask&(1<<ch))?amp:0;
 ampTargets(a[0],a[1],a[2],a[3]);
 delay((uint32_t)(1000*fminf(.5f,fmaxf(.08f,hz>0?2.f/hz:0))));
 Serial.printf("$begin,%d,%.3f,%.3f,%.3f,%d\n",mask,hz,amp,seconds,invertMask);Serial.flush();
 int n=0;int64_t t0=esp_timer_get_time();
 while(esp_timer_get_time()-t0<(int64_t)(seconds*1e6f)) {
  if(!(imuRead8(0x1E)&1))continue;
  uint8_t raw[6];imuReadBurst(0x28,raw,6);
  if(n<14000){g_char_t[n]=(uint32_t)(esp_timer_get_time()-t0);for(int k=0;k<3;++k)g_char_xyz[n][k]=(int16_t)(raw[k*2]|(raw[k*2+1]<<8));++n;}
 }
 ampTargets(0,0,0,0);delay(35);
 Serial.printf("$data,%d,%.3f\n",n,readVsys());
 for(int i=0;i<n;++i){Serial.write((uint8_t*)&g_char_t[i],4);Serial.write((uint8_t*)g_char_xyz[i],6);}
 Serial.println("$end");
 g_characterize=false;hapticStop();imuWrite8(0x10,0x40);
 for(int ch=0;ch<4;++ch)g_ch_invert[ch]=false;
}

// Full response sweep of the fitted channels: each solo, then the first two fitted together
// in phase and in anti-phase. Streams "$sw,mode,hz,amp_fs,h1_mg,h2_mg,h3_mg,rms_mg,vsys,n".
static volatile bool g_filter_test = false;
static volatile bool g_filter_reset = false;
static volatile float g_filter_hz = 50.0f;

static void filteredSweep()
{
    if (!g_tdm_ok || !g_imu_ok) { Serial.println("# filtered sweep skipped"); return; }
    ampTargets(0,0,0,0);
    imuWrite8(0x10, SWEEP_CTRL1_XL);
    delay(20);
    hapticStart();
    Serial.println("# sweep begin filtered FIR 2kHz, input=1.0, slots=0/2/3");
    for(int k=0;k<SWEEP_NF;++k) {
        float f=SWEEP_F0*powf(SWEEP_F1/SWEEP_F0,(float)k/(SWEEP_NF-1));
        g_filter_hz=f; g_filter_reset=true; g_filter_test=true;
        delay(160);
        ToneResp r=imuToneResp(f,SWEEP_MEAS_S);
        float vs=readVsys();
        g_filter_test=false;
        Serial.printf("$sw,E,%.2f,1.00,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",f,r.h_mg[0],r.h_mg[1],r.h_mg[2],r.rms_mg,vs,r.n);
        delay(160);
        g_haptic_on_since=millis();
        if(Serial.available() && Serial.peek()=='x') { Serial.read(); break; }
    }
    g_filter_test=false; hapticStop(); imuWrite8(0x10,0x40);
    Serial.println("# sweep end");
}

static void sweepRun()
{
    if (!g_tdm_ok || !g_imu_ok)
    {
        Serial.println("# sweep: needs TDM and IMU — skipped");
        return;
    }
    int fit[4], nfit = 0;
    for (int ch = 0; ch < 4; ch++)
        if (CH_FITTED[ch])
            fit[nfit++] = ch;
    Serial.printf("# sweep begin amp=%.2f usb500=%d f0=%.0f f1=%.0f nf=%d\n", g_amp_level, (int)g_usb500,
                  SWEEP_F0, SWEEP_F1, SWEEP_NF);
    imuWrite8(0x10, SWEEP_CTRL1_XL);
    delay(20);
    ampTargets(0, 0, 0, 0);
    hapticStart();
    delay(100);

    const int nmodes = g_signature_sweep?nfit:nfit + nfit * (nfit - 1);
    for (int m = 0; m < nmodes; m++)
    {
        float a[4] = {0, 0, 0, 0};
        char label[8];
        for (int i = 0; i < 4; i++)
            g_ch_invert[i] = false;
        if (m < nfit)
        {
            a[fit[m]] = g_amp_level;
            snprintf(label, sizeof(label), "%d", fit[m] + 1);
        }
        else
        {
            bool anti = ((m - nfit) % 2) != 0;
            int pair = (m - nfit) / 2, pi = 0, pj = 1;
            for (int i = 0, k = 0; i < nfit; ++i)
                for (int j = i + 1; j < nfit; ++j, ++k)
                    if (k == pair) { pi = i; pj = j; }
            a[fit[pi]] = a[fit[pj]] = g_amp_level;
            g_ch_invert[fit[pj]] = anti;
            snprintf(label, sizeof(label), "%d%c%d", fit[pi] + 1, anti ? '-' : '+', fit[pj] + 1);
        }
        delay(300); // ring-out from the previous mode
        ToneResp b = imuToneResp(100.0f, SWEEP_MEAS_S);
        Serial.printf("$sw,%s,0,0,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n", label, b.h_mg[0], b.h_mg[1], b.h_mg[2],
                      b.rms_mg, readVsys(), b.n); // hz=0 row = silent baseline
        if(g_signature_sweep) for(int j=0;j<g_signature_n;++j)
            Serial.printf("$raw,%s,0,%lu,%d,%d,%d\n",label,(unsigned long)g_signature_t[j],g_signature_xyz[j][0],g_signature_xyz[j][1],g_signature_xyz[j][2]);
        for (int k = 0; k < SWEEP_NF; k++)
        {
            float f = SWEEP_F0 * powf(SWEEP_F1 / SWEEP_F0, (float)k / (SWEEP_NF - 1));
            for (int i = 0; i < 4; i++)
                g_freq[i] = f;
            ampTargets(a[0], a[1], a[2], a[3]);
            delay(70); // 20 ms ramp + settle
            ToneResp r = imuToneResp(f, SWEEP_MEAS_S);
            float vs = readVsys();
            ampTargets(0, 0, 0, 0);
            Serial.printf("$sw,%s,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n", label, f, g_amp_level, r.h_mg[0],
                          r.h_mg[1], r.h_mg[2], r.rms_mg, vs, r.n);
            delay(40); // ramp down (phase accumulators reset at zero amplitude)
            if(g_signature_sweep) for(int j=0;j<g_signature_n;++j)
                Serial.printf("$raw,%s,%.2f,%lu,%d,%d,%d\n",label,f,(unsigned long)g_signature_t[j],g_signature_xyz[j][0],g_signature_xyz[j][1],g_signature_xyz[j][2]);
            g_haptic_on_since = millis();
        }
    }
    for (int i = 0; i < 4; i++)
    {
        g_ch_invert[i] = false;
        g_freq[i] = CH_DEFAULT_HZ[i];
    }
    hapticStop();
    imuWrite8(0x10, 0x40); // CTRL1_XL back to 104 Hz, ±2 g
    Serial.println("# sweep end");
}

// ── Continuous chirp stream ──────────────────────────────────────────────────
// Output (all on serial):
//   $ci,odr_hz,mg_per_lsb,f0,f1,chirp_ms   stream parameters, once
//   %x,y,z                                 raw accel sample (LSB), every FIFO sample
//   $cs,mode,sample_idx  /  $ce,mode,sample_idx   chirp start / end at that sample index
//   $ts,sample_count,esp_timer_us          every 500 ms, for ODR estimation
static bool g_cs_on = false, g_cs_in_chirp = false;
static int g_cs_mode = 0;
static uint32_t g_cs_t = 0, g_cs_ts = 0, g_xl_count = 0;

static int imuFifoUnreadWords()
{
    uint8_t st[2];
    imuReadBurst(0x3A, st, 2); // FIFO_STATUS1/2
    return st[0] | ((st[1] & 0x07) << 8);
}
static int16_t imuFifoWord()
{
    uint8_t w[2];
    imuReadBurst(0x3E, w, 2); // FIFO_DATA_OUT_L/H
    return (int16_t)(w[0] | (w[1] << 8));
}
static void imuFifoDrain()
{
    static char buf[1024];
    int len = 0;
    int words = imuFifoUnreadWords();
    // realign to an X word if a partial sample was left behind
    uint8_t pat[2];
    imuReadBurst(0x3C, pat, 2); // FIFO_STATUS3/4: next word's position in the pattern
    int p = pat[0] | ((pat[1] & 0x03) << 8);
    while (p != 0 && words > 0)
    {
        imuFifoWord();
        words--;
        p = (p + 1) % 3;
    }
    for (int s = 0; s < words / 3 && s < 96; s++)
    {
        int16_t x = imuFifoWord(), y = imuFifoWord(), z = imuFifoWord();
        len += snprintf(buf + len, sizeof(buf) - len, "%%%d,%d,%d\n", x, y, z);
        g_xl_count++;
        if (len > (int)sizeof(buf) - 32)
        {
            Serial.write((const uint8_t *)buf, len);
            len = 0;
        }
    }
    if (len)
        Serial.write((const uint8_t *)buf, len);
}

static int chirpModes(int fit[4], int &nfit)
{
    nfit = 0;
    for (int ch = 0; ch < 4; ch++)
        if (CH_FITTED[ch])
            fit[nfit++] = ch;
    return nfit + (nfit >= 2 ? 1 : 0); // each solo, then the first two together in phase
}

static void chirpStreamStop()
{
    if (!g_cs_on)
        return;
    g_cs_on = false;
    g_chirp_on = false;
    hapticStop();
    imuWrite8(0x0A, 0x00); // FIFO bypass
    imuWrite8(0x10, 0x40); // CTRL1_XL back to 104 Hz, ±2 g
    for (int i = 0; i < 4; i++)
        g_freq[i] = CH_DEFAULT_HZ[i];
    Serial.println("# chirp stream off");
}

static void chirpStreamStart()
{
    if (!g_tdm_ok || !g_imu_ok)
    {
        Serial.println("# chirp: needs TDM and IMU — skipped");
        return;
    }
    imuWrite8(0x10, SWEEP_CTRL1_XL);
    imuWrite8(0x0A, 0x00); // FIFO bypass (clears it)
    imuWrite8(0x08, 0x01); // FIFO_CTRL3: accel in FIFO, no decimation; gyro out
    imuWrite8(0x09, 0x00);
    imuWrite8(0x0A, 0x46); // FIFO_CTRL5: FIFO ODR 1.66 kHz, continuous mode
    ampTargets(0, 0, 0, 0);
    hapticStart();
    g_cs_on = true;
    g_cs_in_chirp = false;
    g_cs_mode = 0;
    g_cs_t = g_cs_ts = millis();
    g_xl_count = 0;
    Serial.printf("# chirp stream on (amp=%.2f FS) — 'C' or 'x' to stop\n", g_amp_level);
    Serial.printf("$ci,1666,%.3f,%.0f,%.0f,%lu\n", SWEEP_MG_PER_LSB, CHIRP_F0, CHIRP_F1,
                  (unsigned long)(CHIRP_NBLK * TDM_BLOCK_FRAMES * 1000UL / TDM_RATE));
}

static void chirpStreamTick()
{
    imuFifoDrain();
    int fit[4], nfit;
    int nmodes = chirpModes(fit, nfit);
    char label[8];
    if (g_cs_mode < nfit)
        snprintf(label, sizeof(label), "%d", fit[g_cs_mode] + 1);
    else
        snprintf(label, sizeof(label), "%d+%d", fit[0] + 1, fit[1] + 1);
    uint32_t now = millis();
    if (!g_cs_in_chirp)
    {
        if (now - g_cs_t >= CHIRP_GAP_MS)
        {
            float a[4] = {0, 0, 0, 0};
            if (g_cs_mode < nfit)
                a[fit[g_cs_mode]] = g_amp_level;
            else
                a[fit[0]] = a[fit[1]] = g_amp_level;
            for (int i = 0; i < 4; i++)
                g_freq[i] = CHIRP_F0;
            g_chirp_blk = 0;
            g_chirp_on = true;
            ampTargets(a[0], a[1], a[2], a[3]);
            g_cs_in_chirp = true;
            Serial.printf("$cs,%s,%lu\n", label, (unsigned long)(g_xl_count + imuFifoUnreadWords() / 3));
        }
    }
    else if (g_chirp_blk >= CHIRP_NBLK)
    {
        ampTargets(0, 0, 0, 0);
        g_chirp_on = false;
        g_cs_in_chirp = false;
        g_cs_t = now;
        Serial.printf("$ce,%s,%lu\n", label, (unsigned long)(g_xl_count + imuFifoUnreadWords() / 3));
        g_cs_mode = (g_cs_mode + 1) % nmodes;
    }
    if (now - g_cs_ts >= 500)
    {
        g_cs_ts = now;
        Serial.printf("$ts,%lu,%lld\n", (unsigned long)(g_xl_count + imuFifoUnreadWords() / 3),
                      (long long)esp_timer_get_time());
    }
    g_haptic_on_since = now; // the stream manages its own on/off
}

// ── Constant vibration ('v', for companion/vibe_gui.py) ──────────────────────
// "v<ch> <hz> <amp>\n": ch 1-4 holds a continuous tone, ch 0 stops. No 3 s auto-off, but the
// host must repeat the command at least every HOLD_WATCHDOG_MS or the haptics are cut.
// While holding, "$lv,<rms_mg>,<vsys>" is printed every 200 ms (IMU at 1.66 kHz, ±8 g).
static constexpr uint32_t HOLD_WATCHDOG_MS = 1500;
static int g_hold_ch = -1;
static float g_hold_hz = 0, g_hold_amp = 0;
static uint32_t g_hold_kick = 0;

static void holdSet(int ch, float hz, float amp)
{
    if (ch < 0 || ch > 3)
    {
        if (g_hold_ch >= 0)
        {
            g_hold_ch = -1;
            hapticStop();
            if (g_imu_ok)
                imuWrite8(0x10, 0x40); // CTRL1_XL back to 104 Hz, ±2 g
            for (int i = 0; i < 4; i++)
                g_freq[i] = CH_DEFAULT_HZ[i];
            Serial.println("$hold,off");
        }
        return;
    }
    hz = fminf(fmaxf(hz, 10.0f), 1000.0f);
    amp = fminf(fmaxf(amp, 0.0f), AMP_MAX);
    g_hold_kick = millis();
    if (ch == g_hold_ch && hz == g_hold_hz && amp == g_hold_amp)
        return; // keepalive only
    if (g_hold_ch < 0 && g_imu_ok)
        imuWrite8(0x10, SWEEP_CTRL1_XL);
    g_freq[ch] = hz;
    hapticStart();
    float a[4] = {0, 0, 0, 0};
    a[ch] = amp;
    ampTargets(a[0], a[1], a[2], a[3]);
    g_hold_ch = ch;
    g_hold_hz = hz;
    g_hold_amp = amp;
    Serial.printf("$hold,%d,%.1f,%.3f\n", ch + 1, hz, amp);
}

static void holdTick(uint32_t now)
{
    static uint32_t t_lv = 0;
    if (g_hold_ch < 0)
        return;
    if (now - g_hold_kick > HOLD_WATCHDOG_MS)
    {
        Serial.println("# hold: no keepalive from host — haptics off");
        holdSet(-1, 0, 0);
        return;
    }
    g_haptic_on_since = now; // holding replaces the 3 s auto-off
    if (g_imu_ok && now - t_lv >= 200)
    {
        t_lv = now;
        float mg = imuVibRms(80, nullptr, SWEEP_MG_PER_LSB);
        Serial.printf("$lv,%.1f,%.2f\n", mg, readVsys());
    }
}

// Integer-only synthesis: the ESP32-C6 has no FPU, so per-sample sinf() for
// 4 channels at 48 kHz starves every other task. 1024-entry table + Q32 phase
// accumulator + Q15 amplitude with a linear ramp.
static int16_t g_sine[1024];
static uint32_t g_phase_acc[4] = {0, 0, 0, 0};
static int32_t g_amp_q15[4] = {0, 0, 0, 0};
static constexpr int32_t AMP_RAMP_STEP = (int32_t)(32767.0f * RAMP_PER_SAMPLE) + 1;

static void taskAudio(void *)
{
    for (int i = 0; i < 1024; i++)
        g_sine[i] = (int16_t)lrintf(32767.0f * sinf(2.0f * (float)M_PI * i / 1024.0f));
    for (;;)
    {
        if (!g_tdm_running)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (g_chirp_on)
        {
            static const float k = logf(CHIRP_F1 / CHIRP_F0) / CHIRP_NBLK;
            uint32_t b = g_chirp_blk;
            float f = b >= CHIRP_NBLK ? CHIRP_F1 : CHIRP_F0 * expf(k * b);
            for (int ch = 0; ch < 4; ch++)
                g_freq[ch] = f;
            g_chirp_blk = b + 1;
        }
        uint32_t inc[4];
        int32_t tgt[4];
        bool inv[4];
        float amp_snap[4];
        portENTER_CRITICAL(&g_amp_mux);
        for (int ch = 0; ch < 4; ch++)
            amp_snap[ch] = g_amp_target[ch];
        portEXIT_CRITICAL(&g_amp_mux);
        for (int ch = 0; ch < 4; ch++)
        {
            inc[ch] = (uint32_t)(g_freq[ch] * 4294967296.0f / TDM_RATE); // one float op per block
            tgt[ch] = (int32_t)(fminf(amp_snap[ch], AMP_MAX) * 32767.0f);
            inv[ch] = g_ch_invert[ch];
        }
        static ThreewayFilter filter;
        static uint32_t filter_phase=0;
        static int filter_sub=0;
        static int16_t filter_prev[3]={},filter_next[3]={};
        if(g_filter_reset) {
            filter.reset(); filter_phase=0; filter_sub=0;
            memset(filter_prev,0,sizeof(filter_prev)); memset(filter_next,0,sizeof(filter_next));
            g_filter_reset=false;
        }
        const bool filter_on=g_filter_test;
        const uint32_t filter_inc=(uint32_t)(g_filter_hz*4294967296.0f/2000.0f);
        for (int f = 0; f < TDM_BLOCK_FRAMES; f++)
        {
            int16_t *frame = &g_tdm_buf[f * TDM_SLOTS];
            for (int ch = 0; ch < 4; ch++)
            {
                int32_t a = g_amp_q15[ch];
                if (a < tgt[ch]) { a += AMP_RAMP_STEP; if (a > tgt[ch]) a = tgt[ch]; }
                else if (a > tgt[ch]) { a -= AMP_RAMP_STEP; if (a < tgt[ch]) a = tgt[ch]; }
                g_amp_q15[ch] = a;
                if (a > 0)
                {
                    int16_t smp = (int16_t)(((int32_t)g_sine[g_phase_acc[ch] >> 22] * a) >> 15);
                    if(g_signature_sweep && ch==0 && smp<0)smp=0;
                    if(g_characterize && ch==0 && g_char_shape!=16) {
                        // Positive half-sine impacts, capped at 8ms per pulse.
                        float phase=(float)g_phase_acc[ch]/4294967296.f;
                        float width=fminf(.5f,g_freq[ch]*.008f);
                        smp=(width>0&&phase<width)?(int16_t)(((int32_t)g_sine[(int)(phase/width*512.f)]*a)>>15):0;
                        if(g_char_shape==32) {
                            // Smooth offset sine, range [0,A], zero slope at return.
                            int idx=((g_phase_acc[ch]>>22)+768)&1023;
                            smp=(int16_t)(((int32_t)(32767+g_sine[idx])*a)>>16);
                        }
                    }
                    frame[ch] = inv[ch] ? (int16_t)-smp : smp;
                    g_phase_acc[ch] += inc[ch];
                }
                else
                {
                    frame[ch] = 0;
                    g_phase_acc[ch] = 0;
                }
            }
            if(filter_on) {
                memset(frame,0,TDM_SLOTS*sizeof(int16_t));
                if(filter_sub==0) {
                    memcpy(filter_prev,filter_next,sizeof(filter_next));
                    filter.process(g_sine[filter_phase>>22],filter_next);
                    filter_phase+=filter_inc;
                }
                for(int ch=0;ch<3;++ch)
                    frame[ThreewayCoefficients::kSlots[ch]]=filter_prev[ch]+((int32_t)(filter_next[ch]-filter_prev[ch])*(filter_sub+1))/24;
                filter_sub=(filter_sub+1)%24;
            }
            if (g_slot_mirror && !filter_on) // diagnostic: HF signal also on slots 4..7 (catches amp slot mis-detect)
                frame[4] = frame[5] = frame[6] = frame[7] = frame[2];
            else
                frame[4] = frame[5] = frame[6] = frame[7] = 0;
        }
        size_t written = 0;
        g_aud_err = i2s_channel_write(g_tx, g_tdm_buf, sizeof(g_tdm_buf), &written, pdMS_TO_TICKS(100));
        g_aud_blocks++;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Blade pixels (only meaningful with Pololu boost + strip on J7/J8)
// ─────────────────────────────────────────────────────────────────────────────
static void bladeBlack()
{
    g_blade.clear();
    g_blade.show();
    Serial.println("# blade: black frame sent on IO20");
}
static void bladeRainbow()
{
    for (int i = 0; i < BLADE_PIXELS; i++)
        g_blade.setPixelColor(i, g_blade.gamma32(g_blade.ColorHSV((uint16_t)(i * 65536L / BLADE_PIXELS), 255, 40)));
    g_blade.show();
    Serial.printf("# blade: dim rainbow on %d pixels (needs 5 V at J7.1)\n", BLADE_PIXELS);
}

// ─────────────────────────────────────────────────────────────────────────────
// I2C scan
// ─────────────────────────────────────────────────────────────────────────────
static void i2cScan()
{
    Serial.print("# i2c scan:");
    int n = 0;
    for (uint8_t a = 0x08; a < 0x78; a++)
    {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0)
        {
            Serial.printf(" 0x%02X", a);
            n++;
        }
    }
    Serial.printf("  (%d found; expect 0x62 PCA9632, 0x70 all-call)\n", n);
}

// ─────────────────────────────────────────────────────────────────────────────
// Status / help
// ─────────────────────────────────────────────────────────────────────────────
static void printHelp()
{
    Serial.println("# --- A7 bring-up commands ----------------------------------");
    Serial.println("#  ?        help          s  self-test summary      i  I2C scan   m  IMU probe/retry");
    Serial.println("#  r g b w  LED colour    0  LED off               c  LED auto (status breathing)");
    Serial.println("#  a        toggle 20 Hz accel/gyro stream");
    Serial.println("#  1 2 3 4  play test tone on LFi / MF / HF / LF   x  stop haptics");
    Serial.println("#  t        actuator test: sweep fitted channels (LFi, HF), IMU pass/fail");
    Serial.println("#  T        same, all four channels (finds actuators on the wrong connector)");
    Serial.println("#  E        validate threeway FIR (input full scale, coefficient-limited outputs)");
    Serial.println("#  W        full sweep 20-500 Hz, solo + paired in/anti-phase ($sw CSV, ~1 min)");
    Serial.println("#  C        continuous chirp loop 20-500 Hz, raw accel stream (companion/chirp_view.py)");
    Serial.println("#  y        diag: mirror HF onto TDM slots 4-7");
    Serial.println("#  v<ch> <hz> <amp>  constant tone (repeat <1.5 s as keepalive), v0 = off (companion/vibe_gui.py)");
    Serial.println("#  u        toggle USB input limit USB500 / USB100 (IO17)");
    Serial.println("#  + -      haptic amplitude ±0.05 FS (cap 1.00)   f  cycle tone freq 40→320 Hz");
    Serial.println("#  p        blade rainbow  k  blade black frame   (needs boost + strip)");
    Serial.println("# ----------------------------------------------------------");
}

static void selfTest()
{
    Serial.println("# --- self-test ---------------------------------------------");
    Serial.printf("#  reset reason      : %d\n", (int)esp_reset_reason());
    Serial.printf("#  PCA9632 @0x62     : %s\n", g_pca_ok ? "OK" : "FAIL");
    Serial.printf("#  LSM6DS3TR-C (SPI) : %s\n", g_imu_ok ? "OK" : "FAIL");
    float v = readVsys();
    Serial.printf("#  VSYS              : %.2f V %s\n", v, (v > 3.0f && v < 4.5f) ? "OK" : "SUSPECT");
    Serial.printf("#  USB_GOOD_N (IO3)  : %d  (%s)\n", digitalRead(PIN_USB_GOOD_N),
                  digitalRead(PIN_USB_GOOD_N) ? "no charger power-good" : "charger power-good");
    Serial.printf("#  buttons MODE/GAIN/BOOT : %d/%d/%d  (1 = released)\n",
                  digitalRead(PIN_BTN_MODE), digitalRead(PIN_BTN_GAIN), digitalRead(PIN_BTN_BOOT));
    Serial.printf("#  IMU INT1 (IO2)    : %d  irq count=%lu\n", digitalRead(PIN_IMU_INT), (unsigned long)g_imu_irq_count);
    Serial.printf("#  USB_500 (IO17)    : %d (%s)\n", (int)g_usb500, g_usb500 ? "USB500" : "USB100");
    Serial.printf("#  HAPTIC_EN (IO5)   : %d\n", digitalRead(PIN_HAPTIC_EN));
    Serial.printf("#  TDM channel       : %s  enable=%s  write=%s\n", g_tdm_ok ? "OK" : "FAIL",
                  esp_err_to_name(g_tdm_enable_err), esp_err_to_name(g_aud_err));
    for (int ch = 0; ch < 4; ch++)
    {
        const ActResult &r = g_act[ch];
        if (!CH_FITTED[ch] && !r.run)
            Serial.printf("#  actuator %-11s: not fitted\n", CH_NAME[ch]);
        else if (!r.run)
            Serial.printf("#  actuator %-11s: not run (press t)\n", CH_NAME[ch]);
        else
            Serial.printf("#  actuator %-11s: %s  peak %.2f mg @ %.0f Hz, baseline %.2f mg\n", CH_NAME[ch],
                          g_imu_ok ? (r.pass ? "OK" : "FAIL") : "not measured", r.best_mg, r.best_hz, r.base_mg);
    }
    Serial.println("# ----------------------------------------------------------");
}

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    // Safe defaults first
    pinMode(PIN_HAPTIC_EN, OUTPUT);
    digitalWrite(PIN_HAPTIC_EN, LOW);
    pinMode(PIN_USB_500, OUTPUT);
    digitalWrite(PIN_USB_500, g_usb500 ? HIGH : LOW); // USB500 so full-amplitude haptics don't brown out

    pinMode(PIN_BTN_MODE, INPUT);  // external 100k pull-ups
    pinMode(PIN_BTN_GAIN, INPUT);
    pinMode(PIN_BTN_BOOT, INPUT);  // external 10k pull-up
    pinMode(PIN_USB_GOOD_N, INPUT_PULLUP); // BQ24074 PGOOD is open-drain
    pinMode(PIN_IMU_INT, INPUT);   // external 10k pull-up (R29)
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);

    Serial.begin(115200);
    Serial.setTimeout(50); // 'v' line commands
    {
        uint32_t t0 = millis();
        while (!Serial && millis() - t0 < 3000)
            delay(10);
    }
    Serial.println("\n# ===== Rolling Stone A7 bring-up =====");

    // I2C + handle LED
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    i2cScan();
    g_pca_ok = pcaInit();
    Serial.printf("# pca9632: %s\n", g_pca_ok ? "ok" : "FAILED");
    if (g_pca_ok)
    {
        // colour sweep so each die is visually verified
        const uint8_t seq[4][3] = {{40, 0, 0}, {0, 40, 0}, {0, 0, 40}, {40, 40, 40}};
        const char *nm[4] = {"red", "green", "blue", "white"};
        for (int i = 0; i < 4; i++)
        {
            ledSet(seq[i][0], seq[i][1], seq[i][2]);
            Serial.printf("# led: %s\n", nm[i]);
            delay(400);
        }
        ledSet(0, 0, 0);
    }

    // IMU
    SPI.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, -1);
    imuProbe();
    g_imu_ok = imuInit();
    Serial.printf("# imu: %s\n", g_imu_ok ? "ok" : "FAILED");
    attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuIsr, FALLING);

    // ADC
    analogReadResolution(12);

    // TDM (clocks stay stopped until a tone is requested)
    g_tdm_ok = tdmInit();
    Serial.printf("# tdm: %s\n", g_tdm_ok ? "ok (48 kHz, 8x16-bit slots)" : "init FAILED");
    xTaskCreate(taskAudio, "audio", 4096, nullptr, 5, nullptr);

    // Blade: nothing sent at boot; U12 is unpowered unless the boost is wired
    g_blade.begin();

    selfTest();
    printHelp();
}

static void handleCommand(char c)
{
    switch (c)
    {
    case '?': printHelp(); break;
    case 's': selfTest(); break;
    case 'i': i2cScan(); break;
    case 'm':
        imuProbe();
        g_imu_ok = imuInit();
        Serial.printf("# imu: %s\n", g_imu_ok ? "ok" : "FAILED");
        break;
    case 'r': g_led_auto = false; ledSet(60, 0, 0); break;
    case 'g': g_led_auto = false; ledSet(0, 60, 0); break;
    case 'b': g_led_auto = false; ledSet(0, 0, 60); break;
    case 'w': g_led_auto = false; ledSet(60, 60, 60); break;
    case '0': g_led_auto = false; ledSet(0, 0, 0); break;
    case 'c': g_led_auto = true; break;
    case 'a': g_accel_stream = !g_accel_stream; Serial.printf("# accel stream %s\n", g_accel_stream ? "on" : "off"); break;
    case '1': case '2': case '3': case '4': hapticPlay(c - '1'); break;
    case 'x': holdSet(-1, 0, 0); chirpStreamStop(); hapticStop(); break;
    case 't': holdSet(-1, 0, 0); chirpStreamStop(); actuatorTest(); break;
    case 'T': holdSet(-1, 0, 0); chirpStreamStop(); actuatorTest(true); break;
    case 'E': holdSet(-1, 0, 0); chirpStreamStop(); filteredSweep(); break;
    case 'W': holdSet(-1, 0, 0); chirpStreamStop(); sweepRun(); break;
    case 'Q': {
        String q=Serial.readStringUntil('\n');int mask=0,inv=0;float hz=0,amp=0,dur=0;
        if(sscanf(q.c_str(),"%d %f %f %f %d",&mask,&hz,&amp,&dur,&inv)==5){holdSet(-1,0,0);chirpStreamStop();characterize(mask,hz,amp,dur,inv);}
        else Serial.println("$err,parse");
        break;
    }
    case 'B': holdSet(-1, 0, 0); chirpStreamStop(); g_signature_sweep=true; sweepRun(); g_signature_sweep=false; break;
    case 'v':
    {
        String s = Serial.readStringUntil('\n');
        int ch = 0;
        float hz = 0, amp = 0;
        if (sscanf(s.c_str(), "%d %f %f", &ch, &hz, &amp) >= 1)
        {
            chirpStreamStop();
            holdSet(ch - 1, hz, amp);
        }
        break;
    }
    case 'C':
        holdSet(-1, 0, 0);
        if (g_cs_on) chirpStreamStop();
        else chirpStreamStart();
        break;
    case 'u': usb500Set(!g_usb500); break;
    case 'y':
        g_slot_mirror = !g_slot_mirror;
        Serial.printf("# slot mirror (HF also on slots 4-7): %s\n", g_slot_mirror ? "on" : "off");
        break;
    case '+': case '=':
        g_amp_level = fminf(AMP_MAX, g_amp_level + AMP_STEP);
        Serial.printf("# amp=%.2f FS\n", g_amp_level);
        for (int i = 0; i < 4; i++) if (g_amp_target[i] > 0) g_amp_target[i] = g_amp_level;
        break;
    case '-':
        g_amp_level = fmaxf(0.0f, g_amp_level - AMP_STEP);
        Serial.printf("# amp=%.2f FS\n", g_amp_level);
        for (int i = 0; i < 4; i++) if (g_amp_target[i] > 0) g_amp_target[i] = g_amp_level;
        break;
    case 'f':
        for (int i = 0; i < 4; i++)
        {
            g_freq[i] *= 2.0f;
            if (g_freq[i] > 320.0f) g_freq[i] = 40.0f;
        }
        Serial.printf("# freq: LFi=%.0f MF=%.0f HF=%.0f LF=%.0f Hz\n", g_freq[0], g_freq[1], g_freq[2], g_freq[3]);
        break;
    case 'p': bladeRainbow(); break;
    case 'k': bladeBlack(); break;
    case '\n': case '\r': break;
    default: Serial.printf("# unknown '%c' — press ? for help\n", c); break;
    }
}

void loop()
{
    static uint32_t t_status = 0, t_stream = 0, t_led = 0, t_btn = 0;
    static int prev_btn[3] = {1, 1, 1};
    static uint32_t last_irq = 0, irq_hz = 0, last_blocks = 0, last_iters = 0;
    static uint8_t colour_idx = 0;
    uint32_t now = millis();
    g_loop_iters++;

    while (Serial.available())
        handleCommand((char)Serial.read());

    if (g_cs_on)
    {
        chirpStreamTick();
        delay(1);
        return; // stream owns the serial port and the haptics
    }

    holdTick(millis());

    // Haptic auto-off
    // re-read millis(): g_haptic_on_since may have been set by handleCommand() above
    if (g_tdm_running && (int32_t)(millis() - g_haptic_on_since) > (int32_t)HAPTIC_MAX_ON_MS)
    {
        bool any = false;
        for (int i = 0; i < 4; i++) any |= g_amp_target[i] > 0;
        if (any) { Serial.println("# haptic: auto-off"); hapticStop(); }
    }

    // Buttons at 100 Hz with edge detection
    if (now - t_btn >= 10)
    {
        t_btn = now;
        const int pins[3] = {PIN_BTN_MODE, PIN_BTN_GAIN, PIN_BTN_BOOT};
        const char *names[3] = {"MODE(SW4)", "GAIN(SW5)", "BOOT(SW2)"};
        for (int i = 0; i < 3; i++)
        {
            int v = digitalRead(pins[i]);
            if (v != prev_btn[i])
            {
                prev_btn[i] = v;
                Serial.printf("# btn %s %s\n", names[i], v ? "released" : "PRESSED");
                if (!v && i == 0)
                { // MODE: step through colours
                    const uint8_t pal[6][3] = {{60,0,0},{0,60,0},{0,0,60},{60,30,0},{0,40,40},{40,0,40}};
                    colour_idx = (colour_idx + 1) % 6;
                    g_led_auto = false;
                    ledSet(pal[colour_idx][0], pal[colour_idx][1], pal[colour_idx][2]);
                }
                if (!v && i == 1)
                    g_led_auto = true; // GAIN: back to status breathing
            }
        }
    }

    // Status-breathing LED at ~50 Hz: green = all OK, orange = something failed
    if (g_led_auto && now - t_led >= 20)
    {
        t_led = now;
        float ph = (now % 2000) / 2000.0f;
        float br = 0.5f - 0.5f * cosf(2.0f * (float)M_PI * ph); // 0..1
        uint8_t v = (uint8_t)(4 + br * 56);
        if (g_pca_ok && g_imu_ok) ledSet(0, v, 0);
        else ledSet(v, v / 3, 0);
    }

    // Accel stream (20 Hz)
    if (g_accel_stream && g_imu_ok && now - t_stream >= 50)
    {
        t_stream = now;
        float a[3], g[3];
        imuSample(a, g);
        Serial.printf("$%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n", a[0], a[1], a[2], g[0], g[1], g[2]);
    }

    // 1 Hz status line
    if (now - t_status >= 1000)
    {
        t_status = now;
        uint32_t ic = g_imu_irq_count;
        irq_hz = ic - last_irq;
        last_irq = ic;
        float a[3] = {0, 0, 0}, g[3] = {0, 0, 0};
        if (g_imu_ok) imuSample(a, g);
        uint32_t blk = g_aud_blocks, it = g_loop_iters;
        Serial.printf("# up=%lu us=%lld vsys=%.2fV usb_good=%d btn=%d%d%d acc=[%+.2f %+.2f %+.2f]g gyr=[%+.0f %+.0f %+.0f]dps imu_irq=%luHz haptic=%d aud_blk/s=%lu aud_err=%s loop/s=%lu heap=%lu\n",
                      (unsigned long)now, (long long)(esp_timer_get_time() / 1000), readVsys(), digitalRead(PIN_USB_GOOD_N) == 0,
                      !digitalRead(PIN_BTN_MODE), !digitalRead(PIN_BTN_GAIN), !digitalRead(PIN_BTN_BOOT),
                      a[0], a[1], a[2], g[0], g[1], g[2], (unsigned long)irq_hz,
                      (int)g_tdm_running, (unsigned long)(blk - last_blocks), esp_err_to_name(g_aud_err),
                      (unsigned long)(it - last_iters), (unsigned long)ESP.getFreeHeap());
        last_blocks = blk; last_iters = it;
    }

    delay(2);
}
