#include "rolling_response.h"
#include "haptic.h"
#include "config.h"
#include "rolling_texture.h"
#include "rolling_detail.h"
#include "rattle_pulse.h"
#include <Arduino.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// Shared wavetable — negative arch: values in [-1, 0]
// Position-indexed so pitch rises naturally with ball speed.
// Identical to the wavetable used in the companion app audio synthesis.
// ─────────────────────────────────────────────────────────────────────────────

static float s_wtable[WTABLE_SIZE];
static RollingTexture s_texture;
static bool s_texture_active = false;
static RattlePulse s_rattle;

static void initWavetable()
{
    s_texture.init((float)PHYS_HZ);
    for (int i = 0; i < WTABLE_SIZE; i++)
        s_wtable[i] = -sinf((float)M_PI * i / (WTABLE_SIZE - 1));
}

static RollingTexture::Output textureFrame(float x, float v, float impact, bool active)
{
    if (active != s_texture_active)
    {
        s_texture.reset();
        s_texture_active = active;
    }
    return active ? s_texture.step(x, v, impact / HAPTIC_IMPACT_REF)
                  : RollingTexture::Output{0.0f, 0.0f};
}

// ─────────────────────────────────────────────────────────────────────────────
#if defined(A7_BOARD)
// ── A7 output: TDM to 4× MAX98357A, TacHammer Drake actuators ────────────────
//
// Same signal plan as the H-bridge profile, which matches the TacHammer datasheet:
//   LFi (slot 0): impact = positive pulse (+V drives the hammer into its impact end → "tick"),
//                 rolling rumble = negative arch (−V side, into the magnet: no end-stop rattle)
//   HF  (slot 2): rolling texture (same arch, full sign) for the higher-pitched part of the roll
// Each 0.5 ms tick writes 24 frames, ramped from the previous value to avoid control-rate steps.
// ─────────────────────────────────────────────────────────────────────────────

#include "driver/i2s_tdm.h"
#include "lightsaber.h"
#include "lightsaber_audio.h"
#include "lightsaber_reference.h"
#include "lightsaber_mix.h"
#include "rainstick_audio.h"
#include "contact_voices.h"
#include "rolling_ball.h"
#include "moo_box.h"
static MooBox s_moo;
static RainContact s_rain_contact;
static RollingBall s_ball;
static RainstickAudio s_rain_audio;
#include "imu.h"
static LightsaberMix s_saber_mix;
static int32_t s_saber_prev[4]={};
static Lightsaber s_saber;
static LightsaberAudio s_saber_audio;
static LightsaberReference s_saber_reference;

static constexpr int kSlots = 8, kSlotLfi = 0, kSlotHf = 2;
static constexpr int kFrames = TDM_RATE / PHYS_HZ; // 24
static i2s_chan_handle_t s_tx = nullptr;
static int16_t s_buf[kFrames * kSlots];
static uint32_t s_impact_end_us = 0, s_impact_start_us = 0;
static float s_impact_amp = 0.0f;
static int32_t s_prev_lfi_q15 = 0, s_prev_hf_q15 = 0;
static RollingDetail s_detail;
static bool s_detail_active = false;
volatile float g_haptic_gain = 1.0f; // GAIN button: 0.4 / 0.7 / 1.0
volatile uint32_t g_audio_short_writes = 0;

static void tdmInit()
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 3; // 3 × 1 ms queue, down from 8 ms
    chan_cfg.dma_frame_num = 48;
    chan_cfg.auto_clear = true; // underflow → zeros, never stale samples
    i2s_new_channel(&chan_cfg, &s_tx, nullptr);
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
    cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_512;
    cfg.slot_cfg.total_slot = kSlots;
    cfg.slot_cfg.ws_width = 1; // MAX98357A TDM: one-BCLK frame sync
    i2s_channel_init_tdm_mode(s_tx, &cfg);
    memset(s_buf, 0, sizeof(s_buf));
    size_t n;
    i2s_channel_preload_data(s_tx, s_buf, sizeof(s_buf), &n);
    i2s_channel_enable(s_tx);
}

void hapticInit()
{
    initWavetable();
    s_detail.init((float)TDM_RATE);
    s_ball.init((float)TDM_RATE);
    s_saber.init();
    s_saber_audio.init();
    s_saber_reference.reset();
    s_saber_mix.reset();memset(s_saber_prev,0,sizeof(s_saber_prev));
    pinMode(PIN_HAPTIC_EN, OUTPUT);
    digitalWrite(PIN_HAPTIC_EN, LOW); // amps muted until valid clocks run with zeros
    tdmInit();
    delay(10);
    digitalWrite(PIN_HAPTIC_EN, HIGH);
}

static volatile int32_t s_live_target[4]={32768,19660,32768,32768};
static int32_t s_live_current[4]={32768,19660,32768,32768};
static volatile bool s_restart_voice=false;
void hapticReplayStartup(){s_restart_voice=true;}
void hapticLiveMix(int a,int b,int c,int d,bool restart){
 int v[4]={a,b,c,d};for(int i=0;i<4;i++)s_live_target[i]=v[i]*32768/100;
 if(restart)s_restart_voice=true;
}

// Shared output stage for every A7 voice: limiter + intensity + live channel
// trim in one pass, then a non-blocking TDM write. `mixed` is LFi/MF/HF/LF.
static void mixAndWrite(const int32_t mixed[kFrames][4], float intensity)
{
    const int32_t master=(int32_t)(fminf(1.f,fmaxf(0.f,intensity))*32768.f);
    int32_t gain[4];
    for(int c=0;c<4;c++){
        // Live trim moves at the old 64-per-sample rate, stepped once per tick.
        const int32_t delta=s_live_target[c]-s_live_current[c], limit=64*kFrames;
        s_live_current[c]+=delta>limit?limit:delta < -limit?-limit:delta;
        gain[c]=(int32_t)((int64_t)master*s_live_current[c]/32768);
    }
    memset(s_buf,0,sizeof(s_buf));
    s_saber_mix.process(mixed,s_buf,kSlots,gain);
    size_t written=0;
    i2s_channel_write(s_tx,s_buf,sizeof(s_buf),&written,0); // never block the physics loop
    if(written!=sizeof(s_buf))++g_audio_short_writes;
}
static volatile bool s_diagnostic_reset=false;
void hapticDiagnosticStart(){s_diagnostic_reset=true;}
void hapticDiagnosticReport(){
 Serial.print("$mix_stats");for(int c=0;c<4;c++)Serial.printf(",%ld,%ld",(long)s_saber_mix.maxOutput[c],(long)s_saber_mix.minGain[c]);Serial.println();
}
float hapticLightsaber(float angular, float ax, float ay, float az)
{
    if(s_diagnostic_reset){s_saber.reset();s_saber_audio.reset();s_saber_reference.diagnosticReset();s_saber_mix.reset();s_saber_mix.clearStats();memset(s_saber_prev,0,sizeof(s_saber_prev));s_diagnostic_reset=false;}
    if(s_restart_voice){s_saber_reference.reset();s_restart_voice=false;}
    angular=imuCaptureAngular(angular);
    if(imuCaptureScripted())ax=ay=az=0;
    auto v=s_saber.step(angular,sqrtf(ax*ax+ay*ay+az*az),(float)PHYS_HZ);
    // Reference recordings supply ignition, idle and swings. Keep the
    // synthetic metallic accent only for an actual clash.
    s_saber_audio.control(0,v.impact>0,(float)PHYS_HZ);
    // Audio vibrations contaminate acceleration magnitude. Gyro-based swing
    // triggers avoid the recording repeatedly triggering itself at rest.
    s_saber_reference.control(fminf(1.0f,fabsf(angular)/6.0f),(float)PHYS_HZ);
    // One common hum/reference mix feeds every continuous actuator filter.
    int32_t target[4]={(int32_t)(v.lfi*32767),0,0,(int32_t)(v.hum*32767)};
    static int recovery=0;
    if(v.impact>0)recovery=100;else if(recovery>0)--recovery;
    int32_t mixed[kFrames][4];
    const bool boost=imuCaptureBoost();
    const int mask=imuCaptureMask();
    for(int i=0;i<kFrames;++i) {
        int32_t audio[3];
        int32_t bed=s_saber_prev[3]+(target[3]-s_saber_prev[3])*(i+1)/kFrames;
        s_saber_mix.voice((int32_t)s_saber_reference.next()+s_saber_audio.next()+bed,audio,boost);
        for(int ch=0;ch<4;ch++) {
            int32_t tactile=s_saber_prev[ch]+(target[ch]-s_saber_prev[ch])*(i+1)/kFrames;
            mixed[i][ch]=(mask&(1<<ch))?(ch?audio[ch-1]:(v.impact>0?tactile:(int32_t)((int64_t)s_saber_mix.negativeBass*(100-recovery)/100))):0;
        }
    }
    memcpy(s_saber_prev,target,sizeof(target));
    mixAndWrite(mixed,imuCaptureGain(g_haptic_gain));
    return v.motion;
}

void hapticRainstick(float density, float energy, float knock, int variant) {
    // Contact flow drives a stochastic grain cloud (MF/HF) and a tactile patter
    // bed (LF); distinct knocks go to LFi/LF. See rainstick_audio.h.
    static int character = -1;
    if (variant != character) { s_rain_audio.setCharacter(variant); character = variant; }
    s_rain_contact.excite(knock);
    s_rain_audio.flow(density, energy);
    int32_t mixed[kFrames][4];
    for(int i=0;i<kFrames;i++){
        int32_t bands[3],contact[2];s_rain_contact.next(contact);
        s_saber_mix.voice(s_rain_audio.next(),bands);
        mixed[i][0]=contact[0];
        mixed[i][1]=bands[0];
        mixed[i][2]=bands[1];
        mixed[i][3]=bands[2]+s_rain_audio.bed()+contact[1];
    }
    mixAndWrite(mixed,g_haptic_gain);
}

float hapticMooBox(float tilt) {
    // The moo sits between the actuators' bands, so it goes to all of them
    // directly instead of through the crossover. Landing clunk on LFi/LF.
    s_rain_contact.excite(s_moo.control(tilt, kFrames));
    int32_t mixed[kFrames][4];
    for(int i=0;i<kFrames;i++){
        int32_t contact[2];s_rain_contact.next(contact);
        const int32_t moo=s_moo.next();
        mixed[i][0]=contact[0];
        mixed[i][1]=moo;
        mixed[i][2]=moo;
        mixed[i][3]=moo*3/4+contact[1];
    }
    mixAndWrite(mixed,g_haptic_gain);
    return s_moo.position();
}

void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed, bool textured, bool rattle)
{
    // Rolling (three-way build) is the metal-ball synthesis in rolling_ball.h;
    // the procedural Textured Slide remains available as its own mode.
    const auto texture = textureFrame(x_pos_m, x_vel_mps, impact_speed, textured && !rattle);
    const float rattle_level = rattle ? s_rattle.step(impact_speed / HAPTIC_IMPACT_REF, PHYS_HZ) : 0.0f;
    if (textured != s_detail_active)
    {
        s_detail.reset();
        s_detail_active = textured;
    }
    if (textured)
        s_detail.begin(x_pos_m, x_vel_mps, impact_speed / HAPTIC_IMPACT_REF, (float)PHYS_HZ);

    uint32_t now = micros();
    if (!rattle && impact_speed > 0.0f)
    {
        // Square-root law: a gentle arrival is still a clear knock, a fast one
        // is not much louder. Linear made every hit at a mild tilt disappear.
        s_impact_amp = sqrtf(fminf(impact_speed / HAPTIC_IMPACT_REF, 1.0f));
        uint32_t duration_ms = IMPACT_MIN_MS + (uint32_t)((IMPACT_MS - IMPACT_MIN_MS) * s_impact_amp);
        s_impact_start_us = now;
        s_impact_end_us = now + duration_ms * 1000;
    }

    float lfi, hf = 0.0f;
    if (rattle) { s_impact_end_us = 0; lfi = rattle_level * A7_IMPACT_FS; }
    else if (s_impact_end_us > 0 && (int32_t)(s_impact_end_us - now) > 0)
    {
        // positive tick with raised-cosine edges: the supply sees a current ramp, not a step
        uint32_t t_in = now - s_impact_start_us, t_out = s_impact_end_us - now;
        float e = fminf(1.0f, fminf((t_in + 1000000.0f / PHYS_HZ), (float)t_out) / (A7_IMPACT_EDGE_MS * 1000.0f));
        float w = 0.5f - 0.5f * cosf((float)M_PI * e);
        lfi = s_impact_amp * A7_IMPACT_FS * w;
    }
    else
    {
        s_impact_end_us = 0;
        float arch = 0.0f, amp = 0.0f; // arch in [-amp, 0]
#if !defined(A7_THREEWAY) // the three-way build renders rolling in rolling_ball.h
        if (rolling && !textured)
        {
            float speed = fabsf(x_vel_mps);
            if (speed >= HAPTIC_MIN_SPD)
            {
                amp = rollingClassicEnvelope(speed) * HAPTIC_MAX_AMP;
                const float phase = fmodf(fabsf(x_pos_m * 1000.0f), (float)WTABLE_SIZE);
                const int idx = (int)phase;
                const float frac = phase - idx;
                arch = (s_wtable[idx] + frac * (s_wtable[(idx + 1) % WTABLE_SIZE] - s_wtable[idx])) * amp;
            }
        }
#endif
        lfi = arch * A7_ROLL_FS;                            // negative only: away from the end stop
        hf = (arch + 0.6366f * amp) * A7_HF_ROLL_FS;        // same texture, DC removed (mean of arch = -2/π·amp)
    }
    if (textured)
    {
        if (!rattle && !s_impact_end_us) lfi = texture.low * A7_ROLL_FS;
        hf = texture.high * A7_HF_ROLL_FS * 0.4f;
    }
#if defined(A7_THREEWAY)
    {
        const bool impacting=s_impact_end_us>0;
        int32_t mono=(int32_t)((textured?(texture.low+texture.high):(impacting?0.f:lfi/A7_ROLL_FS+hf/A7_HF_ROLL_FS))*49150.f);
        int32_t hit=(int32_t)(fmaxf(0.f,impacting?lfi:0.f)*32767.f);
        if(rolling&&!textured)s_ball.begin(x_pos_m,x_vel_mps,impact_speed>0.0f?sqrtf(fminf(impact_speed/HAPTIC_IMPACT_REF,1.0f)):0.0f,kFrames);
        int32_t mixed[kFrames][4];
        for(int i=0;i<kFrames;i++){
            const int32_t pulse=s_saber_prev[0]+(hit-s_saber_prev[0])*(i+1)/kFrames;
            if(textured){
                int32_t bands[3];
                int32_t signal=s_saber_prev[3]+(mono-s_saber_prev[3])*(i+1)/kFrames;
                signal+=(int32_t)s_detail.next()*21/32;
                s_saber_mix.voice(signal,bands);
                mixed[i][0]=impacting?pulse:s_saber_mix.negativeBass;
                for(int c=1;c<4;c++)mixed[i][c]=bands[c-1];
            } else if(rolling){
                // LF carries the roll and the plastic thud, HF only the impact
                // tick. LFi adds the hammer pulse and the roll's negative-side
                // sub-bass (its positive side is the end stop).
                const int32_t roll=s_ball.next();
                s_saber_mix.subBass(roll);
                mixed[i][0]=impacting?pulse:pulse+s_saber_mix.negativeBass;
                mixed[i][1]=0;
                mixed[i][2]=s_ball.tick();
                mixed[i][3]=roll+s_ball.thud();
            } else {
                // Sliding: silent in motion, the end-stop knock only.
                mixed[i][0]=pulse;
                mixed[i][1]=mixed[i][2]=mixed[i][3]=0;
            }
        }
        s_saber_prev[0]=hit;s_saber_prev[3]=mono;
        mixAndWrite(mixed,g_haptic_gain);
        return;
    }

#endif
    lfi *= g_haptic_gain;
    hf *= g_haptic_gain;
    const int32_t detail_gain = (int32_t)(A7_HF_ROLL_FS * 0.65f * g_haptic_gain * 32768.0f);
    const int32_t hf_limit = (int32_t)(A7_HF_ROLL_FS * g_haptic_gain * 32767.0f);
    const int32_t lfi_target_q15 = (int32_t)(lfi * 32767.0f);
    const int32_t hf_target_q15 = (int32_t)(hf * 32767.0f);

    // Ramp across the audio frames of this physics tick with bounded Q15 math.
    // The old float interpolation did 48 software-float multiplies per tick on
    // the C6; integer interpolation differs by at most one output LSB.
    for (int i = 0; i < kFrames; i++)
    {
        int16_t *fr = &s_buf[i * kSlots];
        memset(fr, 0, kSlots * sizeof(int16_t));
        const int32_t frame = i + 1;
        fr[kSlotLfi] = (int16_t)(s_prev_lfi_q15 +
                                  ((lfi_target_q15 - s_prev_lfi_q15) * frame) / kFrames);
        int32_t high = s_prev_hf_q15 +
                       ((hf_target_q15 - s_prev_hf_q15) * frame) / kFrames;
        if (textured) high += (int32_t)s_detail.next() * detail_gain / 32768;
        high = high < -hf_limit ? -hf_limit : high > hf_limit ? hf_limit : high;
        fr[kSlotHf] = (int16_t)high;
    }
    s_prev_lfi_q15 = lfi_target_q15;
    s_prev_hf_q15 = hf_target_q15;
    size_t written = 0;
    i2s_channel_write(s_tx, s_buf, sizeof(s_buf), &written, 0); // never block the physics loop
    if (written != sizeof(s_buf)) ++g_audio_short_writes;
}

// ─────────────────────────────────────────────────────────────────────────────
#elif defined(HBRIDGE)
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

void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed, bool textured, bool rattle)
{
    const auto texture = textureFrame(x_pos_m, x_vel_mps, impact_speed, textured && !rattle);
    const float rattle_level = rattle ? s_rattle.step(impact_speed / HAPTIC_IMPACT_REF, PHYS_HZ) : 0.0f;
    uint32_t now = millis();
    if (!rattle && impact_speed > 0.0f)
    {
        s_impact_amp = fminf(impact_speed / HAPTIC_IMPACT_REF, 1.0f);
        // Scale duration: low energy = short pulse, high energy = long pulse
        uint32_t duration_ms = IMPACT_MIN_MS + (uint32_t)((IMPACT_MS - IMPACT_MIN_MS) * s_impact_amp);
        s_impact_end_ms = now + duration_ms;
    }

    float sig;
    if (rattle) { s_impact_end_ms = 0; sig = rattle_level * HAPTIC_MAX_AMP; }
    else if (s_impact_end_ms > 0 && now < s_impact_end_ms)
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
                float amp = rollingClassicEnvelope(speed) * HAPTIC_MAX_AMP;
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

    if (textured && !rattle && !s_impact_end_ms) sig = texture.low;

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
    speed = rollingClassicEnvelope(speed);
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
    // IDF 5.3's default macro has out-of-order C++ designators on ESP32-C6.
    std_cfg.clk_cfg.sample_rate_hz = AUDIO_RATE;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_DEFAULT;
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
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

void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed, bool textured, bool rattle)
{
    const auto texture = textureFrame(x_pos_m, x_vel_mps, impact_speed, textured && !rattle);
    const float rattle_level = rattle ? s_rattle.step(impact_speed / HAPTIC_IMPACT_REF, PHYS_HZ) : 0.0f;
    uint32_t now = millis();
    if (!rattle && impact_speed > 0.0f)
    {
        s_impact_amp = fminf(impact_speed / HAPTIC_IMPACT_REF, 1.0f);
        // Scale duration: low energy = short pulse, high energy = long pulse
        uint32_t duration_ms = IMPACT_MIN_MS + (uint32_t)((IMPACT_MS - IMPACT_MIN_MS) * s_impact_amp);
        s_impact_end_ms = now + duration_ms;
    }

    float sig;
    if (rattle) { s_impact_end_ms = 0; sig = rattle_level * HAPTIC_MAX_AMP; }
    else if (s_impact_end_ms > 0 && now < s_impact_end_ms)
    {
        sig = s_impact_amp;
    }
    else
    {
        s_impact_end_ms = 0;
        sig = rolling ? rollingSignal(x_pos_m, x_vel_mps) : 0.0f;
    }

    if (textured && !rattle && !s_impact_end_ms)
        sig = fmaxf(-HAPTIC_MAX_AMP, fminf(0.0f, texture.low + 0.15f * texture.high));

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

#endif // board

void hapticResetTransient() {
    s_rattle.reset();
    s_texture.reset();
#if defined(A7_BOARD)
    s_rain_audio.reset();
    s_rain_contact.reset();s_ball.reset();s_moo.reset();
#endif
#if defined(A7_BOARD)
    s_impact_end_us = 0;
    s_saber.reset();
    s_saber_audio.reset();
    s_saber_reference.reset();
    s_saber_mix.reset();memset(s_saber_prev,0,sizeof(s_saber_prev));
    s_prev_lfi_q15 = s_prev_hf_q15 = 0;
    s_detail.reset();
#else
    s_impact_end_ms = 0;
#endif
}
