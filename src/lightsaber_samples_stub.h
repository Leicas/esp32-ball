#pragma once
#include <stdint.h>
// Silent placeholder for lightsaber_samples.h.
//
// The real file holds 16 kHz PCM cut from local reference recordings and is
// not distributed with this repository. This stub declares the same arrays,
// with the same lengths, filled with silence, so the firmware builds and the
// lightsaber's timing logic is unchanged; it just has no recorded voice.
// To get sound, generate your own lightsaber_samples.h next to this file with
// companion/prepare_lightsaber_samples.py; it is picked up automatically.
namespace LightsaberSamples {
constexpr int kRate=16000;
inline constexpr int16_t ignition[20000]={};
inline constexpr int16_t idle[60800]={};
inline constexpr int16_t swing0[9440]={};
inline constexpr int16_t swing1[9440]={};
struct Clip { const int16_t *data; uint32_t count; };
inline constexpr Clip swings[]={{swing0,sizeof(swing0)/sizeof(int16_t)},{swing1,sizeof(swing1)/sizeof(int16_t)}};
}
