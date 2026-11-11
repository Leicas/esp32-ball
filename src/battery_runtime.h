#pragma once
#if defined(A7_BOARD)
void batteryRuntimeInit();
void batteryRuntimeTick();
bool batteryRuntimeCommand(char c);
#endif
