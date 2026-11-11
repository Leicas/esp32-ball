#pragma once
#include <atomic>
#include <stdint.h>
extern std::atomic<uint32_t> game_round;
extern std::atomic<bool> game_revealed;
extern std::atomic<int> game_count;
// Requests are applied by the physics task, never by the comms task.
void gameNewRound();
void gameReveal();
