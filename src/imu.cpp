#include "imu.h"
#include <Arduino.h>

volatile float imu_angular_speed = 0.0f;
volatile float imu_swing_speed = 0.0f;
volatile float imu_sin_alpha = 0.0f;
volatile float imu_tube_accel = 0.0f;
volatile uint32_t imu_sample_hz = 0;
volatile float imu_lin_accel_x = 0.0f;
volatile float imu_lin_accel_y = 0.0f;
volatile float imu_lin_accel_z = 0.0f;
volatile float imu_grav_x = 0.0f;
volatile float imu_grav_y = 0.0f;
volatile float imu_grav_z = 0.0f;
volatile float imu_qw = 1.0f;
volatile float imu_qx = 0.0f;
volatile float imu_qy = 0.0f;
volatile float imu_qz = 0.0f;

#if defined(A7_BOARD)
// ─────────────────────────────────────────────────────────────────────────────
// A7: LSM6DS3TR-C over SPI (raw accel + gyro) + Mahony filter → same outputs as the BNO085
// fusion reports: gravity vector, linear acceleration, game rotation quaternion.
// ─────────────────────────────────────────────────────────────────────────────
#include <SPI.h>
#include "config.h"
#include "haptic.h"

// Short diagnostic capture: no serial work in the real-time sampling path.
static uint32_t captureT[6000];
static int16_t captureA[6000][3];
static volatile uint32_t captureStart=0;
static volatile int captureN=0;
static volatile bool captureOn=false,captureReady=false;
static char captureMode='J';
void imuCaptureStart(char mode) {
 if(captureOn||captureReady)return;
 hapticDiagnosticStart();
 captureN=0;captureMode=mode;captureStart=micros();captureOn=true;
 Serial.printf("$capture_begin,%c\n",mode);
}
float imuCaptureAngular(float live) {
 if(!captureOn||captureMode=='J')return live;
 uint32_t elapsed=micros()-captureStart;
 return elapsed<1000000?2.f:elapsed<2000000?6.f:0.f;
}
float imuCaptureGain(float live) {return captureOn&&(captureMode=='A'||captureMode=='B'||captureMode=='C')?.5f:captureOn&&captureMode=='L'?.7f:live;}
int imuCaptureMask(){if(!captureOn)return 15;return captureMode=='U'||captureMode=='A'?8:captureMode=='V'||captureMode=='B'?2:captureMode=='W'||captureMode=='C'?4:15;}
bool imuCaptureScripted(){return captureOn&&captureMode!='J';}
bool imuCaptureBoost(){return captureOn&&captureMode=='P';}
void imuCaptureService() {
 if(!captureReady)return;
 int n=captureN;Serial.printf("$capture_data,%d,0.122\n",n);
 for(int i=0;i<n;i++){Serial.write((uint8_t*)&captureT[i],4);Serial.write((uint8_t*)captureA[i],6);}
 Serial.println("$capture_end");hapticDiagnosticReport();captureReady=false;
}
static const SPISettings kSpi(4000000, MSBFIRST, SPI_MODE3);
static constexpr float kAccLsb = 0.000122f * 9.80665f;          // ±4 g → m/s² per LSB
static constexpr float kGyrLsb = 0.0175f * (float)M_PI / 180.0f; // ±500 dps → rad/s per LSB
static constexpr float kDt = 1.0f / 1666.0f;                     // nominal ODR; runtime uses elapsed time
static constexpr float kKp = 2.0f, kKi = 0.005f;                 // Mahony gains

static float s_q[4] = {1, 0, 0, 0};
static float s_ibias[3] = {0, 0, 0};
static float s_gbias[3] = {0, 0, 0}; // gyro offset from the boot calibration [rad/s]

static uint8_t rd8(uint8_t reg)
{
    SPI.beginTransaction(kSpi);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg | 0x80);
    uint8_t v = SPI.transfer(0);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
    return v;
}
static void wr8(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(kSpi);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
}
// gyro[3] then accel[3], raw LSB
static void readRaw(int16_t g[3], int16_t a[3])
{
    uint8_t b[12];
    SPI.beginTransaction(kSpi);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transfer(0x22 | 0x80); // OUTX_L_G .. OUTZ_H_XL (IF_INC)
    for (int i = 0; i < 12; i++)
        b[i] = SPI.transfer(0);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
    for (int i = 0; i < 3; i++)
    {
        g[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        a[i] = (int16_t)(b[6 + 2 * i] | (b[7 + 2 * i] << 8));
    }
}

// Status and both sensors in one 17-byte burst from STATUS_REG (0x1E, IF_INC):
// status, reserved, temperature x2, gyro x6, accel x6. One transaction and one
// driver call per tick; the byte-by-byte version was a quarter of the tick.
static bool readSample(int16_t g[3], int16_t a[3])
{
    uint8_t tx[17] = {0x1E | 0x80}, rx[17];
    SPI.beginTransaction(kSpi);
    digitalWrite(PIN_IMU_CS, LOW);
    SPI.transferBytes(tx, rx, sizeof(tx));
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.endTransaction();
    if ((rx[1] & 0x03) != 0x03) // XLDA | GDA
        return false;
    const uint8_t *b = rx + 5;
    for (int i = 0; i < 3; i++)
    {
        g[i] = (int16_t)(b[2 * i] | (b[2 * i + 1] << 8));
        a[i] = (int16_t)(b[6 + 2 * i] | (b[7 + 2 * i] << 8));
    }
    return true;
}

bool imuInit(TwoWire & /*unused on A7*/)
{
    pinMode(PIN_IMU_CS, OUTPUT);
    digitalWrite(PIN_IMU_CS, HIGH);
    SPI.begin(PIN_IMU_SCK, PIN_IMU_MISO, PIN_IMU_MOSI, -1);
    uint8_t who = rd8(0x0F);
    Serial.printf("# imu: LSM6DS3TR-C WHO_AM_I=0x%02X\n", who);
    if (who != 0x6A && who != 0x69)
        return false;
    wr8(0x12, 0x01); // CTRL3_C: SW_RESET
    delay(20);
    wr8(0x12, 0x44); // BDU | IF_INC
    wr8(0x10, 0x88); // CTRL1_XL: 1.66 kHz, ±4 g
    wr8(0x11, 0x84); // CTRL2_G : 1.66 kHz, 500 dps
    delay(100);

    // gyro bias + initial attitude from a 0.25 s window in which the board is actually still
    // (a reset while it is being handled would otherwise bake hundreds of dps into the bias)
    double gs[3] = {}, gq[3] = {}, as[3] = {};
    int n = 0, windows = 0;
    bool still = false;
    uint32_t t0 = millis();
    while (millis() - t0 < 3000)
    {
        if ((rd8(0x1E) & 0x03) != 0x03) // XLDA | GDA
            continue;
        int16_t g[3], a[3];
        readRaw(g, a);
        for (int i = 0; i < 3; i++)
        {
            double gv = g[i] * kGyrLsb;
            gs[i] += gv;
            gq[i] += gv * gv;
            as[i] += a[i] * kAccLsb;
        }
        if (++n < 400)
            continue;
        double sd = 0;
        for (int i = 0; i < 3; i++)
            sd = fmax(sd, sqrt(fmax(0.0, gq[i] / n - (gs[i] / n) * (gs[i] / n))));
        windows++;
        if (sd < 1.5 * M_PI / 180.0) // < 1.5 dps spread: still
        {
            still = true;
            break;
        }
        for (int i = 0; i < 3; i++) // moving: start a new window
            gs[i] = gq[i] = as[i] = 0;
        n = 0;
    }
    if (still)
        for (int i = 0; i < 3; i++)
            s_gbias[i] = (float)(gs[i] / n);
    else
    {
        Serial.println("# imu: board moving during calibration — gyro bias 0, filter will converge");
        if (n == 0)
            return true; // attitude converges from the accelerometer within ~1 s
    }
    // initial quaternion aligning body "up" with the measured gravity (yaw arbitrary)
    float ax = as[0] / n, ay = as[1] / n, az = as[2] / n, an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 1e-3f)
    {
        ax /= an; ay /= an; az /= an;
        // shortest rotation with v(q) = a, where v is the body-frame gravity used in mahony()
        // (checked: a=(1,0,0) → q=(.707,0,-.707,0) → v=(1,0,0); a=(0,1,0) → q=(.707,.707,0,0))
        float w = 1.0f + az, x = ay, y = -ax, qn = sqrtf(w * w + x * x + y * y);
        if (qn > 1e-4f)
        {
            s_q[0] = w / qn; s_q[1] = x / qn; s_q[2] = y / qn; s_q[3] = 0.0f;
        }
        else
        {
            s_q[0] = 0.0f; s_q[1] = 1.0f; s_q[2] = s_q[3] = 0.0f;
        }
    }
    Serial.printf("# imu: gyro bias %.2f %.2f %.2f dps, |a| %.2f m/s², a/g = %+.3f %+.3f %+.3f (%d samples)\n",
                  s_gbias[0] * 57.2958f, s_gbias[1] * 57.2958f, s_gbias[2] * 57.2958f, an, ax, ay, az, n);
    return true;
}

static void mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
    float &qw = s_q[0], &qx = s_q[1], &qy = s_q[2], &qz = s_q[3];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    // trust the accelerometer as a gravity reference only near 1 g (shakes are not gravity)
    if (an > 6.0f && an < 13.6f)
    {
        ax /= an; ay /= an; az /= an;
        float vx = 2.0f * (qx * qz - qw * qy), vy = 2.0f * (qw * qx + qy * qz), vz = qw * qw - qx * qx - qy * qy + qz * qz;
        float ex = ay * vz - az * vy, ey = az * vx - ax * vz, ez = ax * vy - ay * vx;
        s_ibias[0] += kKi * ex * dt;
        s_ibias[1] += kKi * ey * dt;
        s_ibias[2] += kKi * ez * dt;
        gx += kKp * ex + s_ibias[0];
        gy += kKp * ey + s_ibias[1];
        gz += kKp * ez + s_ibias[2];
    }
    float hx = 0.5f * dt;
    float dw = -qx * gx - qy * gy - qz * gz, dx = qw * gx + qy * gz - qz * gy;
    float dy = qw * gy - qx * gz + qz * gx, dz = qw * gz + qx * gy - qy * gx;
    qw += dw * hx; qx += dx * hx; qy += dy * hx; qz += dz * hx;
    float qn = 1.0f / sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);
    qw *= qn; qx *= qn; qy *= qn; qz *= qn;
}

void imuUpdate()
{
    // Output registers hold only the latest sample, not a FIFO. Integrate elapsed
    // time so a missed sample does not slow rotation tracking down.
    static uint32_t last_us = 0, rate_us = 0, count = 0;
    for (int k = 0; k < 1; k++)
    {
        int16_t g[3], a[3];
        if (!readSample(g, a))
            break;
        float gr[3], ac[3];
        for (int i = 0; i < 3; i++)
        {
            gr[i] = g[i] * kGyrLsb - s_gbias[i];
            ac[i] = a[i] * kAccLsb;
        }
        const uint32_t now_us = micros();
        if(captureOn) {
            uint32_t elapsed=now_us-captureStart;
            int n=captureN;
            if(elapsed<3000000 && n<6000){captureT[n]=elapsed;for(int j=0;j<3;j++)captureA[n][j]=a[j];captureN=n+1;}
            else {captureOn=false;captureReady=true;}
        }

        const float dt = last_us ? fminf((now_us - last_us) * 1e-6f, 0.01f) : kDt;
        last_us = now_us;
        if (!rate_us) rate_us = now_us;
        ++count;
        if (now_us - rate_us >= 1000000)
        {
            imu_sample_hz = (uint32_t)((uint64_t)count * 1000000 / (now_us - rate_us));
            rate_us = now_us;
            count = 0;
        }
        imu_angular_speed = sqrtf(gr[0]*gr[0]+gr[1]*gr[1]+gr[2]*gr[2]);
        // Only rotation perpendicular to the handle sweeps the blade.
        // Ignore axial roll (Y on A7), even when combined with a swing.
        const int cross1=(A7_TUBE_AXIS+1)%3,cross2=(A7_TUBE_AXIS+2)%3;
        imu_swing_speed = sqrtf(gr[cross1]*gr[cross1]+gr[cross2]*gr[cross2]);
        mahony(gr[0], gr[1], gr[2], ac[0], ac[1], ac[2], dt);
        const float qw = s_q[0], qx = s_q[1], qy = s_q[2], qz = s_q[3];
        // gravity as an accelerometer at rest would read it (same convention as BNO085 SH2_GRAVITY)
        float grav[3] = {2.0f * (qx * qz - qw * qy) * 9.80665f, 2.0f * (qw * qx + qy * qz) * 9.80665f,
                         (qw * qw - qx * qx - qy * qy + qz * qz) * 9.80665f};
        float tube = A7_TUBE_SIGN * grav[A7_TUBE_AXIS];
        imu_sin_alpha = fmaxf(-1.0f, fminf(1.0f, tube / 9.80665f));
        imu_grav_x = grav[0];
        imu_grav_y = grav[1];
        imu_grav_z = grav[2];
        imu_tube_accel = A7_TUBE_SIGN * (ac[A7_TUBE_AXIS] - grav[A7_TUBE_AXIS]);
        imu_lin_accel_x = ac[0] - grav[0];
        imu_lin_accel_y = ac[1] - grav[1];
        imu_lin_accel_z = ac[2] - grav[2];
        imu_qw = qw;
        imu_qx = qx;
        imu_qy = qy;
        imu_qz = qz;
    }
}

#else
// ─────────────────────────────────────────────────────────────────────────────
// Feather / H-bridge: BNO085 on-chip fusion
// ─────────────────────────────────────────────────────────────────────────────
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

static Adafruit_BNO08x s_bno(BNO08X_RESET);
static sh2_SensorValue_t s_sv;

static void enableReports()
{
    if (!s_bno.enableReport(SH2_GRAVITY, 1000))
        Serial.println("# imu: gravity report failed");
    if (!s_bno.enableReport(SH2_LINEAR_ACCELERATION, 1000))
        Serial.println("# imu: linear-accel report failed");
    if (!s_bno.enableReport(SH2_GAME_ROTATION_VECTOR, 1000))
        Serial.println("# imu: game-rotation report failed");
}

bool imuInit(TwoWire &wire)
{
    // Retry up to 10 times — BNO085 needs up to ~800 ms after power-on
    for (int attempt = 0; attempt < 10; attempt++)
    {
        if (s_bno.begin_I2C(BNO08x_I2CADDR_DEFAULT, &wire))
        {
            enableReports();
            return true;
        }
        // Also try secondary address (SA0 pin high = 0x4B)
        if (s_bno.begin_I2C(0x4B, &wire))
        { // secondary address (SA0 pin high)
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
    if (s_bno.wasReset())
    {
        Serial.println("# imu: reset — re-enabling reports");
        enableReports();
    }
    // Drain up to 4 queued events per tick (I2C is fast at 400 kHz)
    for (int n = 0; n < 4 && s_bno.getSensorEvent(&s_sv); n++)
    {
        switch (s_sv.sensorId)
        {
        case SH2_GRAVITY:
            imu_sin_alpha = fmaxf(-1.0f, fminf(1.0f,
                                               s_sv.un.gravity.x / 9.8f));
            imu_grav_x = s_sv.un.gravity.x;
            imu_grav_y = s_sv.un.gravity.y; // raw m/s²
            imu_grav_z = s_sv.un.gravity.z; // raw m/s²
            break;
        case SH2_LINEAR_ACCELERATION:
            imu_lin_accel_x = s_sv.un.linearAcceleration.x;
            imu_tube_accel = s_sv.un.linearAcceleration.x;
            imu_lin_accel_y = s_sv.un.linearAcceleration.y;
            imu_lin_accel_z = s_sv.un.linearAcceleration.z;
            break;
        case SH2_GAME_ROTATION_VECTOR:
            imu_qw = s_sv.un.gameRotationVector.real;
            imu_qx = s_sv.un.gameRotationVector.i;
            imu_qy = s_sv.un.gameRotationVector.j;
            imu_qz = s_sv.un.gameRotationVector.k;
            break;
        }
    }
}

#endif // A7_BOARD
