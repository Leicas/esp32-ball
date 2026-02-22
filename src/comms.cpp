#include "comms.h"
#include "config.h"
#include "physics.h"
#include "marbles.h"
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ── Telemetry snapshot (defined here, extern'd in comms.h) ───────────────────
volatile float telem_sin_alpha = 0.0f;
volatile float telem_x_mm = 0.0f;
volatile float telem_x_vel = 0.0f;
volatile uint32_t telem_phys_hz = PHYS_HZ; // initialize to expected value
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
    if (g_sim_mode == SimMode::ROLLING)
        return "ROLLING";
    if (g_sim_mode == SimMode::SLIDING)
        return "SLIDING";
    return "MARBLES";
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

// ─────────────────────────────────────────────────────────────────────────────

static void bleSendStatus()
{
    if (!s_ble_stat)
        return;
    char buf[80];
    snprintf(buf, sizeof(buf), "%s,%.0f,rebound=%d,phys_hz=%lu",
             modeStr(), g_cavity * 1000.0f, g_rebound ? 1 : 0,
             (unsigned long)telem_phys_hz);
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
    switch (c)
    {
    case 'r':
        g_sim_mode = SimMode::ROLLING;
        Serial.println("# mode=ROLLING");
        bleSendStatus();
        break;
    case 's':
        g_sim_mode = SimMode::SLIDING;
        Serial.println("# mode=SLIDING");
        bleSendStatus();
        break;
    case 'm':
        marblesInit();
        g_sim_mode = SimMode::MARBLES;
        Serial.println("# mode=MARBLES");
        bleSendStatus();
        break;
    case 'b':
        g_rebound = !g_rebound;
        Serial.printf("# rebound=%d\n", g_rebound ? 1 : 0);
        bleSendStatus();
        break;
    case '+':
        g_cavity = fminf(g_cavity + 0.05f, 2.00f);
        Serial.printf("# cavity_mm=%.0f\n", g_cavity * 1000.0f);
        bleSendStatus();
        break;
    case '-':
        g_cavity = fmaxf(g_cavity - 0.05f, 0.10f);
        clampPosition();
        Serial.printf("# cavity_mm=%.0f\n", g_cavity * 1000.0f);
        bleSendStatus();
        break;
    case '?':
        Serial.printf("# mode=%s  cavity_mm=%.0f  rebound=%d  G=%.1f  mu=%.2f\n",
                      modeStr(), g_cavity * 1000.0f, g_rebound ? 1 : 0,
                      G_FACTOR, FRICTION_MU);
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
        if (v.length() > 0)
            handleCommand(v.charAt(0));
    }
};

// ─────────────────────────────────────────────────────────────────────────────

void commsInit()
{
    BLEDevice::init(BLE_DEV_NAME);
    s_ble_server = BLEDevice::createServer();
    s_ble_server->setCallbacks(new BleServerCB());

    BLEService *svc = s_ble_server->createService(BLE_SVC_UUID);

    s_ble_telem = svc->createCharacteristic(
        BLE_TELEM_UUID, BLECharacteristic::PROPERTY_NOTIFY);

    BLECharacteristic *cmd = svc->createCharacteristic(
        BLE_CMD_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    cmd->setCallbacks(new BleCmdCB());

    s_ble_stat = svc->createCharacteristic(
        BLE_STAT_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

    svc->start();

    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    Serial.printf("# BLE: advertising as \"%s\"\n", BLE_DEV_NAME);
}

void commsHandleSerial()
{
    while (Serial.available())
        handleCommand((char)Serial.read());
}

void commsStreamTelemetry()
{
    // Serial: "$sin_a,x_mm,v_mps\n" — companion app expects this exact format at 20 Hz
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
    if (g_sim_mode == SimMode::MARBLES)
    {
        Serial.printf("# grav_x=%+.3f  grav_y=%+.3f  grav_z=%+.3f  phys_hz=%lu  mode=%s\n",
                      (float)telem_grav_x, (float)telem_grav_y, (float)telem_grav_z,
                      (uint32_t)telem_phys_hz, modeStr());
    }
    else
    {
        Serial.printf("# sin_a=%+.3f  x_mm=%.1f  v_mps=%+.4f  phys_hz=%lu  mode=%s  cavity_mm=%.0f  rebound=%d\n",
                      (float)telem_sin_alpha, (float)telem_x_mm, (float)telem_x_vel,
                      (uint32_t)telem_phys_hz, modeStr(),
                      g_cavity * 1000.0f, g_rebound ? 1 : 0);
    }
    bleSendStatus();
}

void commsStreamMarbles()
{
    char buf[256];
    int n = snprintf(buf, sizeof(buf), "@%+.3f,%+.3f,%+.3f,%.3f,%.5f,%.5f,%.5f,%.5f",
                     (float)telem_grav_x,
                     (float)telem_grav_y,
                     (float)telem_grav_z,
                     (float)telem_impact_energy,
                     (float)telem_qw,
                     (float)telem_qx,
                     (float)telem_qy,
                     (float)telem_qz);
    for (int i = 0; i < MARBLE_COUNT; i++)
    {
        n += snprintf(buf + n, (int)sizeof(buf) - n,
                      ",%.1f,%.1f,%.1f",
                      g_marbles[i].x * 1000.0f,
                      g_marbles[i].y * 1000.0f,
                      g_marbles[i].z * 1000.0f);
    }
    snprintf(buf + n, (int)sizeof(buf) - n, "\n");
    Serial.print(buf);

    // BLE: send marble data (9 header floats + MARBLE_COUNT × 3 position floats)
    if (s_ble_server && s_ble_server->getConnectedCount() > 0 && s_ble_telem)
    {
        const int float_count = 9 + MARBLE_COUNT * 3;
        float pkt[float_count];
        pkt[0] = telem_grav_x;
        pkt[1] = telem_grav_y;
        pkt[2] = telem_grav_z;
        pkt[3] = telem_impact_energy;
        pkt[4] = (float)telem_phys_hz;
        pkt[5] = telem_qw;
        pkt[6] = telem_qx;
        pkt[7] = telem_qy;
        pkt[8] = telem_qz;
        for (int i = 0; i < MARBLE_COUNT; i++)
        {
            pkt[9 + i * 3 + 0] = g_marbles[i].x * 1000.0f;
            pkt[9 + i * 3 + 1] = g_marbles[i].y * 1000.0f;
            pkt[9 + i * 3 + 2] = g_marbles[i].z * 1000.0f;
        }
        s_ble_telem->setValue((uint8_t *)pkt, sizeof(pkt));
        s_ble_telem->notify();
    }
}

void commsCheckDoubleTap(float /*accel_mag_mps2*/)
{
    // Double-tap gesture removed — use serial/BLE commands to change mode.
}
