#include "haptic.h"
#include "config.h"
#include <Arduino.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// Shared wavetable — negative arch: values in [-1, 0]
// Position-indexed so pitch rises naturally with ball speed.
// Identical to the wavetable used in the companion app audio synthesis.
// ─────────────────────────────────────────────────────────────────────────────

static float s_wtable[WTABLE_SIZE];

static void initWavetable()
{
    for (int i = 0; i < WTABLE_SIZE; i++)
        s_wtable[i] = -sinf((float)M_PI * i / (WTABLE_SIZE - 1));
}

// ─────────────────────────────────────────────────────────────────────────────
#ifdef HBRIDGE
// ── H-bridge output  (CodeCell ESP32-C6 Drive) ───────────────────────────────
//
// Signal routing:
//   Rolling rumble → minus pin (IN2 = PIN_HB_MINUS) — negative arch values
//   Impact pulse   → plus  pin (IN1 = PIN_HB_PLUS ) — short positive burst
//
// hapticUpdate() is called from the 1 kHz physics task and writes LEDC duty
// directly — no separate synthesis timer needed.  The position-indexed wavetable
// naturally produces pitch ∝ velocity at any update rate.
// ─────────────────────────────────────────────────────────────────────────────

#include "driver/ledc.h"
#include "driver/gpio.h"

static constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_channel_t kChanPlus = LEDC_CHANNEL_0;  // impact  (IN1)
static constexpr ledc_channel_t kChanMinus = LEDC_CHANNEL_1; // rumble  (IN2)
static constexpr ledc_timer_t kTimer = LEDC_TIMER_0;

static uint32_t s_impact_end_ms = 0;
static float s_impact_amp = 0.0f; // amplitude captured at moment of impact

static inline void setDuty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(kMode, ch, duty);
    ledc_update_duty(kMode, ch);
}

void hapticInit()
{
    initWavetable();

    // Release any GPIO hold that may have persisted from a previous power cycle
    gpio_hold_dis((gpio_num_t)PIN_HB_PLUS);
    gpio_hold_dis((gpio_num_t)PIN_HB_MINUS);

    // Configure LEDC timer
    ledc_timer_config_t tc = {};
    tc.speed_mode = kMode;
    tc.duty_resolution = (ledc_timer_bit_t)PWM_BITS;
    tc.timer_num = kTimer;
    tc.freq_hz = PWM_FREQ;
    tc.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&tc);

    // Plus pin — impact (IN1)
    ledc_channel_config_t cc = {};
    cc.speed_mode = kMode;
    cc.timer_sel = kTimer;
    cc.duty = 0;
    cc.hpoint = 0;
    cc.intr_type = LEDC_INTR_DISABLE;
    cc.gpio_num = PIN_HB_PLUS;
    cc.channel = kChanPlus;
    ledc_channel_config(&cc);

    // Minus pin — rumble (IN2)
    cc.gpio_num = PIN_HB_MINUS;
    cc.channel = kChanMinus;
    ledc_channel_config(&cc);
}

void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed)
{
    uint32_t now = millis();
    if (impact_speed > 0.0f)
    {
        s_impact_amp = fminf(impact_speed / HAPTIC_IMPACT_REF, 1.0f);
        // Scale duration: low energy = short pulse, high energy = long pulse
        uint32_t duration_ms = IMPACT_MIN_MS + (uint32_t)((IMPACT_MS - IMPACT_MIN_MS) * s_impact_amp);
        s_impact_end_ms = now + duration_ms;
    }

    float sig;
    if (s_impact_end_ms > 0 && now < s_impact_end_ms)
    {
        sig = s_impact_amp; // burst amplitude ∝ impact speed
    }
    else
    {
        s_impact_end_ms = 0;
        if (rolling)
        {
            float speed = fabsf(x_vel_mps);
            if (speed >= HAPTIC_MIN_SPD)
            {
                float amp = fminf(speed / 2.0f, 1.0f) * HAPTIC_MAX_AMP;
                int idx = abs((int)(x_pos_m * 1000.0f)) % WTABLE_SIZE;
                sig = s_wtable[idx] * amp; // negative arch → [-amp, 0]
            }
            else
            {
                sig = 0.0f;
            }
        }
        else
        {
            sig = 0.0f;
        }
    }

    // Route by sign: positive → plus pin (impact), negative → minus pin (rumble)
    if (sig > 0.001f)
    {
        setDuty(kChanPlus, (uint32_t)(sig * PWM_MAX));
        setDuty(kChanMinus, 0);
    }
    else if (sig < -0.001f)
    {
        setDuty(kChanPlus, 0);
        setDuty(kChanMinus, (uint32_t)((-sig) * PWM_MAX));
    }
    else
    {
        setDuty(kChanPlus, 0);
        setDuty(kChanMinus, 0);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
#else
// ── I2S audio output  (Adafruit Feather ESP32-C6) ────────────────────────────
//
// Each 1 ms physics tick writes ~22 stereo int16 samples to the I2S DMA.
// A fractional accumulator keeps the long-run output at exactly AUDIO_RATE Hz:
//   every 20 ticks emit 23 samples instead of 22 →  950×22 + 50×23 = 22050/s
// ─────────────────────────────────────────────────────────────────────────────

#include "driver/i2s_std.h"

static i2s_chan_handle_t s_tx = nullptr;
static uint32_t s_impact_end_ms = 0;
static float s_impact_amp = 0.0f;

static float s_frac_acc = 0.0f;
static constexpr float FRAC_PER_TICK = (float)AUDIO_RATE / PHYS_HZ - 22.0f; // 0.05

// Stereo 16-bit buffer — large enough for 23 samples
static int16_t s_buf[23 * 2];

static float rollingSignal(float x_pos_m, float x_vel_mps)
{
    float speed = fabsf(x_vel_mps);
    if (speed < HAPTIC_MIN_SPD)
        return 0.0f;
    speed = fminf(speed / 2.0f, 1.0f);
    int idx = abs((int)(x_pos_m * 1000.0f)) % WTABLE_SIZE;
    return s_wtable[idx] * speed * HAPTIC_MAX_AMP;
}

static void i2sInit()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 128;
    i2s_new_channel(&chan_cfg, &s_tx, nullptr);

    i2s_std_config_t std_cfg = {};
    std_cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE);
    std_cfg.slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                       I2S_SLOT_MODE_STEREO);
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
    std_cfg.gpio_cfg.ws = (gpio_num_t)PIN_I2S_LRCLK;
    std_cfg.gpio_cfg.dout = (gpio_num_t)PIN_I2S_DOUT;
    std_cfg.gpio_cfg.din = I2S_GPIO_UNUSED;

    i2s_channel_init_std_mode(s_tx, &std_cfg);
    i2s_channel_enable(s_tx);
}

void hapticInit()
{
    initWavetable();
    i2sInit();
}

void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed)
{
    uint32_t now = millis();
    if (impact_speed > 0.0f)
    {
        s_impact_amp = fminf(impact_speed / HAPTIC_IMPACT_REF, 1.0f);
        // Scale duration: low energy = short pulse, high energy = long pulse
        uint32_t duration_ms = IMPACT_MIN_MS + (uint32_t)((IMPACT_MS - IMPACT_MIN_MS) * s_impact_amp);
        s_impact_end_ms = now + duration_ms;
    }

    float sig;
    if (s_impact_end_ms > 0 && now < s_impact_end_ms)
    {
        sig = s_impact_amp;
    }
    else
    {
        s_impact_end_ms = 0;
        sig = rolling ? rollingSignal(x_pos_m, x_vel_mps) : 0.0f;
    }

    int16_t sample = (int16_t)(sig * 32767.0f);

    int n = 22;
    s_frac_acc += FRAC_PER_TICK;
    if (s_frac_acc >= 1.0f)
    {
        n = 23;
        s_frac_acc -= 1.0f;
    }

    for (int i = 0; i < n; i++)
    {
        s_buf[i * 2] = sample;
        s_buf[i * 2 + 1] = sample;
    }

    size_t written = 0;
    i2s_channel_write(s_tx, s_buf, (size_t)n * 4, &written, 0);
}

#endif // HBRIDGE
