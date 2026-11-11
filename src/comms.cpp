#include "comms.h"
#include "config.h"
#include "physics.h"
#include "marbles.h"
#include "game.h"
#include "imu.h"
#include "haptic.h"
#include "battery_runtime.h"
#include "battery_wifi.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// ── Telemetry snapshot (defined here, extern'd in comms.h) ───────────────────
volatile float telem_sin_alpha = 0.0f;
volatile float telem_x_mm = 0.0f;
volatile float telem_x_vel = 0.0f;
volatile uint32_t telem_phys_hz = PHYS_HZ; // initialize to expected value
volatile uint32_t telem_step_us_max = 0;
volatile uint32_t telem_step_overruns = 0;
volatile uint32_t telem_step_us_avg = 0;
volatile uint32_t telem_imu_us_avg = 0;
volatile float telem_grav_x = 0.0f;
volatile float telem_grav_y = 0.0f;
volatile float telem_grav_z = 0.0f;
volatile float telem_impact_energy = 0.0f;
volatile float telem_qw = 1.0f;
volatile float telem_qx = 0.0f;
volatile float telem_qy = 0.0f;
volatile float telem_qz = 0.0f;

static const char *modeStr()
{
    return simModeName(g_sim_mode);
}

// ── BLE UUIDs ─────────────────────────────────────────────────────────────────
static const char *BLE_DEV_NAME = "RollingStone";
static const char *BLE_SVC_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
static const char *BLE_TELEM_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214";
static const char *BLE_CMD_UUID = "19b10002-e8f2-537e-4f6c-d104768a1214";
static const char *BLE_STAT_UUID = "19b10003-e8f2-537e-4f6c-d104768a1214";

static BLEServer *s_ble_server = nullptr;
static BLECharacteristic *s_ble_telem = nullptr;
static BLECharacteristic *s_ble_stat = nullptr;
static QueueHandle_t s_ble_commands = nullptr;

// ─────────────────────────────────────────────────────────────────────────────

static void bleSendStatus()
{
    if (!s_ble_stat)
        return;
    float gain = 1.0f;
#if defined(A7_BOARD)
    gain = g_haptic_gain;
#endif
    char buf[192];
    snprintf(buf, sizeof(buf), "%s,%.0f,rebound=%d,phys_hz=%lu,haptic_gain=%.1f,phys_target_hz=%d,game_round=%lu,game_revealed=%d,game_count=%d",
             modeStr(), g_cavity * 1000.0f, g_rebound ? 1 : 0,
             (unsigned long)telem_phys_hz, gain, PHYS_HZ,
             (unsigned long)game_round.load(),game_revealed.load()?1:0,
             game_revealed.load()?game_count.load():-1);
#if defined(A7_BOARD)
    // GPIO4 senses the system rail through a 100k/100k divider.
    uint32_t mv = 0;
    for (int i = 0; i < 8; ++i)
        mv += analogReadMilliVolts(4);
    mv = (mv + 2) / 4;
    const int usbGood = digitalRead(3) == LOW;
    const size_t used = strlen(buf);
    snprintf(buf + used, sizeof(buf) - used, ",vsys_mv=%lu,usb_good=%d", (unsigned long)mv, usbGood);
    Serial.printf("# vsys_mv=%lu usb_good=%d\n", (unsigned long)mv, usbGood);
#endif
    s_ble_stat->setValue(buf);
    s_ble_stat->notify();
}

static void clampPosition()
{
    if (phys_x_pos > g_cavity)
    {
        phys_x_pos = g_cavity;
        phys_x_vel = 0.0f;
    }
}

static void handleCommand(char c)
{
#if defined(A7_BOARD)
    if(batteryRuntimeCommand(c))return;
    if(c=='I' && batteryWifiCommand(c))return;
    if(c=='J'||c=='K'||c=='L'||c=='P'||c=='U'||c=='V'||c=='W'||c=='A'||c=='B'||c=='C'){imuCaptureStart(c);return;}
#endif
    switch (c)
    {
    case 'l':
        #if defined(A7_BOARD)
        g_sim_mode = SimMode::LIGHTSABER;
#else
        g_sim_mode = SimMode::TEXTURED;
#endif
        commsSendStatus();
        break;
    case 'y':
        g_sim_mode = SimMode::RAIN;commsSendStatus();break;
    case 'r':
        g_sim_mode = SimMode::ROLLING;
        commsSendStatus();
        break;
    case 't':
        g_sim_mode = SimMode::TEXTURED;
        commsSendStatus();
        break;
    case 's':
        g_sim_mode = SimMode::SLIDING;
        commsSendStatus();
        break;
    case 'm':
    case 'n':
        gameNewRound();
        commsSendStatus();
        break;
    case 'v':
        gameReveal();
        commsSendStatus();
        break;
    case 'g':
#if defined(A7_BOARD)
        a7CycleGain();
#endif
        commsSendStatus();
        break;
    case 'b':
        g_rebound = !g_rebound;
        commsSendStatus();
        break;
    case '+':
        g_cavity = fminf(g_cavity + 0.05f, 2.00f);
        commsSendStatus();
        break;
    case '-':
        g_cavity = fmaxf(g_cavity - 0.05f, 0.10f);
        clampPosition();
        commsSendStatus();
        break;
    case '?':
        commsSendStatus();
        break;
    }
}

// ── BLE callbacks ─────────────────────────────────────────────────────────────

class BleServerCB : public BLEServerCallbacks
{
    void onDisconnect(BLEServer *) override { BLEDevice::startAdvertising(); }
};

class BleCmdCB : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *chr) override
    {
        String v = chr->getValue();
        if (v.length() > 0 && s_ble_commands)
        {
            const char command = v.charAt(0);
            xQueueSend(s_ble_commands, &command, 0);
        }
    }
};

// ─────────────────────────────────────────────────────────────────────────────

void commsInit()
{
#if defined(A7_BOARD)
    pinMode(3, INPUT_PULLUP); // BQ24074 active-low power good
#endif
    s_ble_commands = xQueueCreate(16, sizeof(char));
    BLEDevice::init(BLE_DEV_NAME);
    // Marble notifications carry 84 bytes; allow a central to negotiate enough
    // ATT payload for them and the additive status fields.
    BLEDevice::setMTU(185);
    s_ble_server = BLEDevice::createServer();
    s_ble_server->setCallbacks(new BleServerCB());

    BLEService *svc = s_ble_server->createService(BLE_SVC_UUID);

    s_ble_telem = svc->createCharacteristic(
        BLE_TELEM_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    s_ble_telem->addDescriptor(new BLE2902());

    BLECharacteristic *cmd = svc->createCharacteristic(
        BLE_CMD_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    cmd->setCallbacks(new BleCmdCB());

    s_ble_stat = svc->createCharacteristic(
        BLE_STAT_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    s_ble_stat->addDescriptor(new BLE2902());

    svc->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.printf("# BLE: advertising as \"%s\"\n", BLE_DEV_NAME);
}

void commsHandleSerial()
{
#if defined(A7_BOARD)
    imuCaptureService();
#endif
    while (Serial.available())
    {
        char c=(char)Serial.read();
#if defined(A7_BOARD)
        static bool mixing=false;static char mixline[80];static int mixlen=0;
        if(c=='Z'&&!mixing){mixing=true;mixlen=0;continue;}
        if(mixing){
            if(c=='\n'){
                mixline[mixlen]=0;int a,b,d,e,r;
                if(sscanf(mixline,"%d %d %d %d %d",&a,&b,&d,&e,&r)==5&&a>=0&&a<=100&&b>=0&&b<=100&&d>=0&&d<=100&&e>=0&&e<=100&&(r==0||r==1)){
                    hapticLiveMix(a,b,d,e,r!=0);g_sim_mode=SimMode::LIGHTSABER;
                    Serial.printf("# mixer=%d,%d,%d,%d startup=%d\n",a,b,d,e,r);
                }else Serial.println("# mixer_error=invalid");
                mixing=false;
            }else if(mixlen<79)mixline[mixlen++]=c;
            else {mixing=false;Serial.println("# mixer_error=length");}
            continue;
        }
        if(c=='N'){batteryWifiCommand(c);continue;}
#endif
        handleCommand(c);
    }

    char command;
    while (s_ble_commands && xQueueReceive(s_ble_commands, &command, 0) == pdTRUE)
        handleCommand(command);
    static uint32_t last_game_state = 0;
    const uint32_t game_state = game_round.load() * 2u + (game_revealed.load() ? 1u : 0u);
    if (game_state != last_game_state) {
        last_game_state = game_state;
        commsSendStatus();
    }
}

void commsStreamTelemetry()
{
    // Serial: "$sin_a,x_mm,v_mps\n" — companion app expects this exact format at ~58 Hz
    Serial.printf("$%+.4f,%.1f,%+.4f\n",
                  (float)telem_sin_alpha,
                  (float)telem_x_mm,
                  (float)telem_x_vel);

    // BLE: float32[4] packed little-endian (added phys_hz)
    if (s_ble_server && s_ble_server->getConnectedCount() > 0 && s_ble_telem)
    {
        float pkt[4] = {telem_sin_alpha, telem_x_mm, telem_x_vel, (float)telem_phys_hz};
        s_ble_telem->setValue((uint8_t *)pkt, sizeof(pkt));
        s_ble_telem->notify();
    }
}

void commsSendStatus()
{
    Serial.printf("# game_round=%lu game_revealed=%d game_count=%d\n",
                  (unsigned long)game_round.load(), game_revealed.load() ? 1 : 0,
                  game_revealed.load() ? game_count.load() : -1);

    if (g_sim_mode == SimMode::MARBLES)
    {
        Serial.printf("# grav_x=%+.3f  grav_y=%+.3f  grav_z=%+.3f  phys_hz=%lu  phys_target_hz=%d  mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f  bounce_e=%.2f\n",
                      (float)telem_grav_x, (float)telem_grav_y, (float)telem_grav_z,
                      (uint32_t)telem_phys_hz, PHYS_HZ, modeStr(), g_cavity * 1000.0f,
                      g_rebound ? 1 : 0, G_FACTOR, FRICTION_MU, RESTITUTION_E);
    }
    else
    {
        Serial.printf("# sin_a=%+.3f  x_mm=%.1f  v_mps=%+.4f  phys_hz=%lu  phys_target_hz=%d  mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f  bounce_e=%.2f\n",
                      (float)telem_sin_alpha, (float)telem_x_mm, (float)telem_x_vel,
                      (uint32_t)telem_phys_hz, PHYS_HZ, modeStr(),
                      g_cavity * 1000.0f, g_rebound ? 1 : 0, G_FACTOR, FRICTION_MU, RESTITUTION_E);
    }
#if defined(A7_BOARD)
    extern uint32_t g_a7_brownouts;
    Serial.printf("# up_s=%lu  brownouts=%lu  imu_hz=%lu  haptic_gain=%.1f  audio_drops=%lu  step_us_avg=%lu  imu_us_avg=%lu  step_us_max=%lu  overruns=%lu\n",
                  (unsigned long)(millis() / 1000), (unsigned long)g_a7_brownouts,
                  (unsigned long)imu_sample_hz, (float)g_haptic_gain,
                  (unsigned long)g_audio_short_writes,
                  (unsigned long)telem_step_us_avg, (unsigned long)telem_imu_us_avg,
                  (unsigned long)telem_step_us_max,
                  (unsigned long)telem_step_overruns);
#endif
    bleSendStatus();
}

namespace {
struct MarbleSnapshot {
    float header[8] = {};
    float balls[MARBLE_COUNT][4] = {};
    int count = 0;
    uint32_t round = 0;
    bool revealed = false;
};
MarbleSnapshot s_marble_snapshot;
portMUX_TYPE s_marble_mux = portMUX_INITIALIZER_UNLOCKED;
}
void commsPublishMarbles() {
    MarbleSnapshot next;
    next.header[0]=telem_grav_x; next.header[1]=telem_grav_y; next.header[2]=telem_grav_z;
    next.header[3]=telem_impact_energy;
    next.header[4]=telem_qw; next.header[5]=telem_qx; next.header[6]=telem_qy; next.header[7]=telem_qz;
    next.count=marblesCount(); next.round=game_round.load(); next.revealed=game_revealed.load();
    for (int i=0;i<next.count;++i) {
        next.balls[i][0]=g_marbles[i].x*1000.0f;
        next.balls[i][1]=g_marbles[i].y*1000.0f;
        next.balls[i][2]=g_marbles[i].z*1000.0f;
        next.balls[i][3]=g_marbles[i].r*1000.0f;
    }
    portENTER_CRITICAL(&s_marble_mux);
    s_marble_snapshot=next;
    portEXIT_CRITICAL(&s_marble_mux);
}
void commsStreamMarbles()
{
    MarbleSnapshot frame;
    portENTER_CRITICAL(&s_marble_mux);
    frame=s_marble_snapshot;
    portEXIT_CRITICAL(&s_marble_mux);
    // Immediately seal the old snapshot when a new round has been requested.
    const int visible_count = frame.revealed && game_revealed.load() && frame.round==game_round.load()
                              ? frame.count : 0;
    char buf[320];
    int n = snprintf(buf, sizeof(buf), "@%+.3f,%+.3f,%+.3f,%.3f,%.5f,%.5f,%.5f,%.5f",
                     frame.header[0],frame.header[1],frame.header[2],frame.header[3],
                     frame.header[4],frame.header[5],frame.header[6],frame.header[7]);
    for (int i=0;i<visible_count;++i)
        n += snprintf(buf+n,sizeof(buf)-n,",%.1f,%.1f,%.1f,%.2f",
                      frame.balls[i][0],frame.balls[i][1],frame.balls[i][2],frame.balls[i][3]);
    snprintf(buf+n,sizeof(buf)-n,"\n");
    Serial.print(buf);
    if (s_ble_server && s_ble_server->getConnectedCount()>0 && s_ble_telem) {
        float pkt[9+MARBLE_COUNT*4];
        for (int i=0;i<4;++i) pkt[i]=frame.header[i];
        pkt[4]=(float)telem_phys_hz;
        for (int i=4;i<8;++i) pkt[i+1]=frame.header[i];
        for (int i=0;i<visible_count;++i)
            for(int j=0;j<4;++j) pkt[9+i*4+j]=frame.balls[i][j];
        s_ble_telem->setValue((uint8_t*)pkt,(9+visible_count*4)*sizeof(float));
        s_ble_telem->notify();
    }
}

void commsCheckDoubleTap(float /*accel_mag_mps2*/)
{
    // Double-tap gesture removed — use serial/BLE commands to change mode.
}
