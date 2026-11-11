#include "config.h"
#if defined(A7_BOARD)
#include "battery_runtime.h"
#include <Arduino.h>
#include <Preferences.h>
#include "esp_timer.h"
#include "esp_system.h"
#include "haptic.h"
#include "physics.h"

namespace {
enum State:uint32_t { EMPTY,ARMED,RUNNING,USB_RETURNED,INTERRUPTED,CANCELLED };
struct Record {
 uint32_t magic=0x42525431,version=1,state=EMPTY,elapsed_s=0;
 uint32_t start_mv=0,last_mv=0,min_mv=0,reset_reason=0;
 uint32_t gain_milli=0,mode=0,settings_changed=0,checkpoints=0;
};
Record rec;
uint64_t started_us=0;
uint32_t last_tick=0,last_save=0,usb_changed=0;
bool usb_candidate=true,usb_stable=true,storage_ok=true;
const char* names[]={"empty","armed","running","usb_returned","interrupted","cancelled"};
uint32_t voltage() {
 uint32_t sum=0;for(int i=0;i<8;++i)sum+=analogReadMilliVolts(4);
 return (sum+2)/4;
}
bool save() {
 Preferences p;
 if(!p.begin("batt-runtime",false)){storage_ok=false;return false;}
 bool ok=p.putBytes("record",&rec,sizeof(rec))==sizeof(rec);p.end();
 storage_ok=ok;return ok;
}
void report() {
 if(!Serial)return;
 Serial.printf("# battery_test=%s elapsed_s=%lu saved_interval_s=30 start_mv=%lu last_mv=%lu min_mv=%lu reset_reason=%lu gain_milli=%lu settings_changed=%lu storage_ok=%d checkpoints=%lu\n",
 names[rec.state],(unsigned long)rec.elapsed_s,(unsigned long)rec.start_mv,(unsigned long)rec.last_mv,(unsigned long)rec.min_mv,(unsigned long)rec.reset_reason,(unsigned long)rec.gain_milli,(unsigned long)rec.settings_changed,storage_ok,(unsigned long)rec.checkpoints);
}
void updateElapsed() { rec.elapsed_s=(uint32_t)((esp_timer_get_time()-started_us)/1000000); }
}
void batteryRuntimeInit() {
 pinMode(3,INPUT); // USB_GOOD_N: low only while external USB power is present
 Preferences p;
 if(p.begin("batt-runtime",true)) {
  Record saved;
  if(p.getBytesLength("record")==sizeof(saved) && p.getBytes("record",&saved,sizeof(saved))==sizeof(saved)
     && saved.magic==rec.magic && saved.version==1 && saved.state<=CANCELLED)rec=saved;
  p.end();
 }
 usb_candidate=usb_stable=digitalRead(3)==LOW;usb_changed=millis();
 if(rec.state==RUNNING) {
  // A restart is not proof of an empty battery; preserve the last checkpoint
  // and its reset reason, never silently begin another run or overwrite it.
  rec.state=INTERRUPTED;rec.reset_reason=(uint32_t)esp_reset_reason();save();
 }
 if(rec.state==ARMED && rec.mode>=1 && rec.mode<=4) {
  g_sim_mode=(SimMode)rec.mode;g_haptic_gain=rec.gain_milli/1000.f;
 }
 report();
}
bool batteryRuntimeCommand(char c) {
 if(c=='Q'){report();return true;}
 if(c=='D') {
  if(rec.state==ARMED||rec.state==RUNNING){report();return true;}
  rec=Record{};rec.state=ARMED;
  rec.gain_milli=(uint32_t)(g_haptic_gain*1000);rec.mode=(uint32_t)g_sim_mode;
  if(!save())rec.state=EMPTY;
  report();return true;
 }
 if(c=='X') {
  if(rec.state==RUNNING)updateElapsed();
  if(rec.state==RUNNING||rec.state==ARMED){rec.state=CANCELLED;save();}
  report();return true;
 }
 return false;
}
void batteryRuntimeTick() {
 const uint32_t now=millis();if(now-last_tick<250)return;last_tick=now;
 bool usb=digitalRead(3)==LOW;
 if(usb!=usb_candidate){usb_candidate=usb;usb_changed=now;}
 if(now-usb_changed>=2000)usb_stable=usb_candidate;
 if(rec.state==ARMED && !usb_stable && now-usb_changed>=2000) {
  rec.state=RUNNING;started_us=esp_timer_get_time();rec.elapsed_s=0;
  rec.start_mv=rec.last_mv=rec.min_mv=voltage();
  rec.gain_milli=(uint32_t)(g_haptic_gain*1000);rec.mode=(uint32_t)g_sim_mode;
  last_save=now;if(!save())rec.state=ARMED;report();
 }
 if(rec.state!=RUNNING)return;
 updateElapsed();
 if(rec.mode!=(uint32_t)g_sim_mode || rec.gain_milli!=(uint32_t)(g_haptic_gain*1000))rec.settings_changed=1;
 if(usb_stable || now-last_save>=30000) {
  rec.last_mv=voltage();if(rec.last_mv<rec.min_mv)rec.min_mv=rec.last_mv;
  if(usb_stable)rec.state=USB_RETURNED;
  ++rec.checkpoints;
  if(save())last_save=now;
  report();
 }
}
#endif
