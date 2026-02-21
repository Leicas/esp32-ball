#pragma once
#include <stdbool.h>

// Initialise output hardware (PWM pins or I2S channel).
void hapticInit();

// Called once per physics tick (1 kHz).
//   x_pos_m      : ball position [m]
//   x_vel_mps    : ball velocity [m/s]
//   rolling      : true = rolling mode (rumble active); false = sliding (impact only)
//   impact_speed : ball speed at wall contact [m/s]; 0 = no impact this tick
//
// Signal routing:
//   Rolling rumble → minus pin (IN2) only — negative arch wavetable
//   Impact pulse   → plus  pin (IN1) only — burst scaled to impact energy
//   (For I2S: rolling = negative audio, impact = positive burst)
void hapticUpdate(float x_pos_m, float x_vel_mps, bool rolling, float impact_speed);
