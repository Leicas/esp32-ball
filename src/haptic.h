#pragma once
#include <stdbool.h>
#include <stdint.h>

// Initialise output hardware (PWM pins or I2S channel).
void hapticInit();

// Called once per physics tick (2 kHz on A7; 1 kHz on other profiles).
//   x_pos_m      : ball position [m]
//   x_vel_mps    : ball velocity [m/s]
//   rolling      : true = rolling mode (rumble active); false = sliding (impact only)
//   impact_speed : ball speed at wall contact [m/s]; 0 = no impact this tick
//
// Signal routing:
//   Rolling rumble → minus pin (IN2) only — negative arch wavetable
//   Impact pulse   → plus  pin (IN1) only — burst scaled to impact energy
//   (For I2S: rolling = negative audio, impact = positive burst)
// textured selects procedural contact/resonance synthesis, with identical ball physics.
void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed, bool textured = false, bool rattle = false);
void hapticResetTransient();
float hapticLightsaber(float angular_speed, float ax, float ay, float az);

#if defined(A7_BOARD)
// Output intensity multiplier (A7 GAIN button cycles 0.4 / 0.7 / 1.0)
extern volatile float g_haptic_gain;
extern volatile uint32_t g_audio_short_writes;
#endif

#if defined(A7_BOARD)
void hapticDiagnosticStart();
void hapticDiagnosticReport();
#endif

#if defined(A7_BOARD)
void hapticLiveMix(int a,int b,int c,int d,bool restart);
#endif

#if defined(A7_BOARD)
void hapticReplayStartup();
// density/energy/knock come from RainTube::step(); variant 0 sand, 1 rice, 2 beans.
void hapticRainstick(float density, float energy, float knock, int variant);
// Rainstick character 3: the moo box. Returns the piston position, 0..1.
float hapticMooBox(float tilt);
#endif
