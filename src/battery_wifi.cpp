#include "config.h"
#if defined(A7_BOARD)
#include "battery_wifi.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include "esp_system.h"
#include "haptic.h"
#include "physics.h"
#include "comms.h"
#include "battery_estimate.h"
static bool active=false;
static String ssid,pass;
static void worker(void*) {
 WiFi.mode(WIFI_STA);WiFi.setAutoReconnect(true);WiFi.setSleep(true);WiFi.begin(ssid.c_str(),pass.c_str());
 WiFiUDP udp;uint32_t seq=0,lastReconnect=millis();char boot[20];snprintf(boot,sizeof(boot),"%08lx%08lx",(unsigned long)esp_random(),(unsigned long)esp_random());
 float filtered=0;int settled=0;
 for(;;) {
  if(WiFi.status()==WL_CONNECTED) {
   uint32_t mv=0;for(int i=0;i<8;++i)mv+=analogReadMilliVolts(4);mv=(mv+2)/4;
   bool usb=digitalRead(3)==LOW;
   if(usb){filtered=0;settled=0;}else{filtered=settled?filtered+.25f*(mv-filtered):mv;++settled;}
   BatteryEstimate estimate=(!usb&&settled>=3)?estimateBattery(filtered):BatteryEstimate{-1,-1};
   char packet[768];String mac=WiFi.macAddress();
   int n=snprintf(packet,sizeof(packet),"{\"v\":1,\"kind\":\"a7_battery\",\"device\":\"%s\",\"boot\":\"%s\",\"seq\":%lu,\"uptime_s\":%lu,\"vsys_mv\":%lu,\"usb\":%d,\"rssi\":%d,\"reset_reason\":%d,\"mode\":\"%s\",\"gain\":%.2f,\"audio_drops\":%lu}",mac.c_str(),boot,(unsigned long)seq++,(unsigned long)(millis()/1000),(unsigned long)mv,digitalRead(3)==LOW?1:0,WiFi.RSSI(),(int)esp_reset_reason(),simModeName(g_sim_mode),(float)g_haptic_gain,(unsigned long)g_audio_short_writes);
   if(n>0&&n<(int)sizeof(packet)-150){--n;n+=snprintf(packet+n,sizeof(packet)-n,",\"battery_pct\":%d,\"remaining_s\":%d,\"estimate_model\":\"measured-3v3-v1\"}",estimate.percent,estimate.remaining_s);}
   // Directed UDP avoids depending on broadcast forwarding through Docker/VLANs.
   if(n>0&&n<(int)sizeof(packet)&&udp.beginPacket(IPAddress(192,168,1,158),9876)){udp.write((uint8_t*)packet,n);udp.endPacket();}
  } else if(millis()-lastReconnect>30000){WiFi.reconnect();lastReconnect=millis();}
  vTaskDelay(pdMS_TO_TICKS(15000));
 }
}
void batteryWifiInit() {
 if(active)return;Preferences p;if(!p.begin("batt-wifi",true))return;
 ssid=p.getString("ssid","");pass=p.getString("pass","");p.end();
 if(ssid.isEmpty())return;
 active=xTaskCreate(worker,"battery-wifi",6144,nullptr,1,nullptr)==pdPASS;
}
bool batteryWifiCommand(char c) {
 if(c=='I'){Serial.printf("# battery_wifi connected=%d ip=%s\n",WiFi.status()==WL_CONNECTED,WiFi.localIP().toString().c_str());return true;}
 if(c!='N')return false;
 // Provision only via local USB serial: N<ssid>\t<password>\n. Never echo secrets.
 String line=Serial.readStringUntil('\n');int split=line.indexOf('\t');
 if(split<1||split>32||line.length()-split-1>63){Serial.println("# wifi_config invalid");return true;}
 Preferences p;bool ok=p.begin("batt-wifi",false);
 if(ok){ok=p.putString("ssid",line.substring(0,split))>0&&p.putString("pass",line.substring(split+1))>0;p.end();}
 Serial.println(ok?"# wifi_config saved":"# wifi_config failed");
 if(ok)batteryWifiInit();return true;
}
#endif
