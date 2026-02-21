#include "imu.h"
#include <Adafruit_BNO08x.h>
#include <Arduino.h>

#define BNO08X_RESET -1

static Adafruit_BNO08x s_bno(BNO08X_RESET);
static sh2_SensorValue_t s_sv;

volatile float imu_sin_alpha   = 0.0f;
volatile float imu_lin_accel_x = 0.0f;
volatile float imu_lin_accel_y = 0.0f;
volatile float imu_lin_accel_z = 0.0f;
volatile float imu_grav_y      = 0.0f;
volatile float imu_grav_z      = 0.0f;

static void enableReports()
{
    if (!s_bno.enableReport(SH2_GRAVITY, 1000))
        Serial.println("# imu: gravity report failed");
    if (!s_bno.enableReport(SH2_LINEAR_ACCELERATION, 1000))
        Serial.println("# imu: linear-accel report failed");
}

bool imuInit(TwoWire &wire)
{
    // Retry up to 10 times — BNO085 needs up to ~800 ms after power-on
    for (int attempt = 0; attempt < 10; attempt++) {
        if (s_bno.begin_I2C(BNO08x_I2CADDR_DEFAULT, &wire)) {
            enableReports();
            return true;
        }
        // Also try secondary address (SA0 pin high = 0x4B)
        if (s_bno.begin_I2C(0x4B, &wire)) { // secondary address (SA0 pin high)
            enableReports();
            return true;
        }
        Serial.printf("# imu: no ACK (attempt %d/10) — waiting 100ms\n", attempt + 1);
        delay(100);
    }
    return false;
}

void imuUpdate()
{
    if (s_bno.wasReset()) {
        Serial.println("# imu: reset — re-enabling reports");
        enableReports();
    }
    // Drain up to 4 queued events per tick (I2C is fast at 400 kHz)
    for (int n = 0; n < 4 && s_bno.getSensorEvent(&s_sv); n++) {
        switch (s_sv.sensorId) {
        case SH2_GRAVITY:
            imu_sin_alpha = fmaxf(-1.0f, fminf(1.0f,
                                  s_sv.un.gravity.x / 9.8f));
            imu_grav_y    = s_sv.un.gravity.y;  // raw m/s²
            imu_grav_z    = s_sv.un.gravity.z;  // raw m/s²
            break;
        case SH2_LINEAR_ACCELERATION:
            imu_lin_accel_x = s_sv.un.linearAcceleration.x;
            imu_lin_accel_y = s_sv.un.linearAcceleration.y;
            imu_lin_accel_z = s_sv.un.linearAcceleration.z;
            break;
        }
    }
}
