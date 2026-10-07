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
 *   - Addressable blade data         (IO20 via 5 V buffer)                   — on command only
 *
 * Pin map source: esp32-ball-hardware/firmware-interface.md, cross-checked
 * against kicad/circuit.json (U5 module pins → nets).
 *
 * Safety policy from firmware-interface.md:
 *   - USB_500 (IO17) held LOW  → USB100 charging only
 *   - HAPTIC_EN (IO5) LOW at boot; only raised after TDM clocks run with zeros
 *   - haptic amplitude capped at AMP_MAX full-scale; auto-off after HAPTIC_MAX_ON_MS
 */

#include <Arduino.h>
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
static constexpr float AMP_MAX = 0.25f;     // hard cap (~1 Vrms) — raise only after bench measurement
static constexpr float AMP_STEP = 0.05f;
static constexpr float RAMP_PER_SAMPLE = 1.0f / (0.020f * TDM_RATE); // 20 ms ramp
static constexpr uint32_t HAPTIC_MAX_ON_MS = 3000;
static const float CH_DEFAULT_HZ[4] = {80.0f, 150.0f, 250.0f, 60.0f}; // LFi, MF, HF, LF
static const char *CH_NAME[4] = {"LFi(J3,U7)", "MF(J4,U8)", "HF(J5,U10)", "LF(J6,U11)"};

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
static float g_freq[4] = {CH_DEFAULT_HZ[0], CH_DEFAULT_HZ[1], CH_DEFAULT_HZ[2], CH_DEFAULT_HZ[3]};
static float g_amp_level = AMP_DEFAULT;
static uint32_t g_haptic_on_since = 0;
static int16_t g_tdm_buf[TDM_BLOCK_FRAMES * TDM_SLOTS];

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
        uint32_t inc[4];
        int32_t tgt[4];
        for (int ch = 0; ch < 4; ch++)
        {
            inc[ch] = (uint32_t)(g_freq[ch] * 4294967296.0f / TDM_RATE); // one float op per block
            tgt[ch] = (int32_t)(fminf(g_amp_target[ch], AMP_MAX) * 32767.0f);
        }
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
                    frame[ch] = (int16_t)(((int32_t)g_sine[g_phase_acc[ch] >> 22] * a) >> 15);
                    g_phase_acc[ch] += inc[ch];
                }
                else
                {
                    frame[ch] = 0;
                    g_phase_acc[ch] = 0;
                }
            }
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
    Serial.println("#  + -      haptic amplitude ±0.05 FS (cap 0.25)   f  cycle tone freq 40→320 Hz");
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
    Serial.printf("#  USB_500 (IO17)    : held LOW (USB100)\n");
    Serial.printf("#  HAPTIC_EN (IO5)   : %d\n", digitalRead(PIN_HAPTIC_EN));
    Serial.printf("#  TDM channel       : %s  enable=%s  write=%s\n", g_tdm_ok ? "OK" : "FAIL",
                  esp_err_to_name(g_tdm_enable_err), esp_err_to_name(g_aud_err));
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
    digitalWrite(PIN_USB_500, LOW);

    pinMode(PIN_BTN_MODE, INPUT);  // external 100k pull-ups
    pinMode(PIN_BTN_GAIN, INPUT);
    pinMode(PIN_BTN_BOOT, INPUT);  // external 10k pull-up
    pinMode(PIN_USB_GOOD_N, INPUT_PULLUP); // BQ24074 PGOOD is open-drain
    pinMode(PIN_IMU_INT, INPUT);   // external 10k pull-up (R29)
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);

    Serial.begin(115200);
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
    case 'x': hapticStop(); break;
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
