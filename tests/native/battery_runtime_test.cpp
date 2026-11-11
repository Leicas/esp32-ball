#include <cassert>
#include "../../src/battery_estimate.h"
#include "../../src/battery_runtime.cpp"
volatile float g_haptic_gain=1.f;
volatile SimMode g_sim_mode=SimMode::LIGHTSABER;
void step(uint32_t ms){fake_ms+=ms;batteryRuntimeTick();}
void reboot(){rec=Record{};started_us=0;last_tick=last_save=0;fake_ms=0;batteryRuntimeInit();}
int main(){
 assert(estimateBattery(3300).percent==0);assert(estimateBattery(4500).percent==100);
 int lastPct=-1,lastTime=-1;for(int mv=2500;mv<=4500;++mv){auto e=estimateBattery(mv);assert(e.percent>=lastPct&&e.remaining_s>=lastTime&&e.percent<=100);lastPct=e.percent;lastTime=e.remaining_s;}
 batteryRuntimeInit();assert(rec.state==EMPTY);
 assert(batteryRuntimeCommand('D'));assert(rec.state==ARMED);
 step(60000);assert(rec.elapsed_s==0&&rec.state==ARMED);
 reboot();assert(rec.state==ARMED&&g_sim_mode==SimMode::LIGHTSABER);
 fake_usb=false;step(250);step(1000);assert(rec.state==ARMED);
 step(1000);assert(rec.state==RUNNING);
 step(30000);assert(rec.elapsed_s==30&&rec.checkpoints==1);
 step(10000);assert(rec.elapsed_s==40);
 reboot();assert(rec.state==INTERRUPTED&&rec.elapsed_s==30);
 step(60000);assert(rec.elapsed_s==30); // no automatic overwrite after reboot
 fake_usb=true;assert(batteryRuntimeCommand('D'));step(250);step(2000);
 fake_usb=false;step(250);step(2000);assert(rec.state==RUNNING);
 step(5000);fake_usb=true;step(250);step(2000);assert(rec.state==USB_RETURNED);
 auto elapsed=rec.elapsed_s;reboot();assert(rec.elapsed_s==elapsed&&rec.state==USB_RETURNED);
 fail_save=true;batteryRuntimeCommand('D');assert(rec.state==EMPTY&&!storage_ok);
 puts("Battery runtime: USB gating, debounce, checkpoints, reboot preservation, stop, write failure passed");
}
