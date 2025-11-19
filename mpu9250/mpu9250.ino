/*
  MPU-9250 (MPU9250) full fused orientation for ESP32-S3
  - accel + gyro (MPU9250) with your original calibration flow
  - AK8963 magnetometer read (via I2C bypass)
  - tilt-compensated magnetometer yaw + complementary filter for yaw
  - ESP32-S3: Wire.begin(SDA_PIN, SCL_PIN, 400000);

  Notes:
  - This uses direct AK8963 reads by enabling INT_PIN_CFG I2C_BYPASS_EN on the MPU.
  - You still should perform a figure-8 magnetometer calibration in real usage to fill mag_bias & mag_scale.
  - Factory sensitivity adjustment (ASAX/ASAY/ASAZ) is read from AK8963 fuse ROM.
*/

#include <Wire.h>
#include <math.h>

// ----------------- I2C / Device addrs -----------------
#define MPU9250_ADDR 0x68 // AD0 low = 0x68; AD0 high = 0x69
#define AK8963_ADDR 0x0C

// MPU registers
#define WHO_AM_I 0x75
#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define INT_PIN_CFG 0x37

// AK8963 registers
#define AK8963_WHO_AM_I 0x00
#define AK8963_ST1 0x02
#define AK8963_XOUT_L 0x03
#define AK8963_CNTL1 0x0A
#define AK8963_ASAX 0x10
#define AK8963_ST2 0x09

// ----------------- Config -----------------
const uint8_t ACCEL_FS_4G = (1 << 3); // ±4g
const uint8_t GYRO_FS_500 = (1 << 3); // ±500 dps
const uint8_t DLPF_CFG = 3;           // ~44 Hz accel / 42 Hz gyro
const float ACC_LSB_PER_G = 8192.0f;  // ±4g
const float GYRO_LSB_PER_DPS = 65.5f; // ±500 dps
const float AK8963_LSB_uT = 0.15f;    // 0.15 uT/LSB for 16-bit continuous

const int CAL_DISCARD = 500;
const int CAL_SAMPLES = 8000;

const int SAMPLE_PERIOD_MS = 5; // ~200 Hz
const float ALPHA_RP = 0.98f;   // roll/pitch complementary
const float ALPHA_YAW = 0.98f;  // yaw complementary

// I2C pins for ESP32-S3 - change if needed
const int I2C_SDA_PIN = 8; // example, set to your board wiring
const int I2C_SCL_PIN = 9; // example, set to your board wiring

// ----------------- Calibration / state -----------------
float ax_bias = 0, ay_bias = 0, az_bias = 0;
float gx_bias = 0, gy_bias = 0, gz_bias = 0;

float roll_deg = 0.0f;
float pitch_deg = 0.0f;
float yaw_deg = 0.0f;

// Magnetometer adjustments & calibration placeholders
float mag_adj[3] = {1.0f, 1.0f, 1.0f};   // factory sensitivity adjustments (ASAX/ASAY/ASAZ)
float mag_bias[3] = {0.0f, 0.0f, 0.0f};  // hard-iron (LSB) - fill after calibration
float mag_scale[3] = {1.0f, 1.0f, 1.0f}; // soft-iron scale - fill after calibration

// ----------------- I2C helpers (addr param) -----------------
void i2cWrite8To(uint8_t devaddr, uint8_t reg, uint8_t val)
{
    Wire.beginTransmission((int)devaddr);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission(true);
}

bool i2cReadNFrom(uint8_t devaddr, uint8_t reg, uint8_t n, uint8_t *buf)
{
    Wire.beginTransmission((int)devaddr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    int got = Wire.requestFrom((int)devaddr, (int)n, (int)true);
    if (got != n)
        return false;
    for (int i = 0; i < n; i++)
        buf[i] = Wire.read();
    return true;
}

// ----------------- Raw sensor reads -----------------
bool readRawMPU(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz)
{
    uint8_t data[14];
    if (!i2cReadNFrom(MPU9250_ADDR, ACCEL_XOUT_H, 14, data))
        return false;
    ax = (int16_t)((data[0] << 8) | data[1]);
    ay = (int16_t)((data[2] << 8) | data[3]);
    az = (int16_t)((data[4] << 8) | data[5]);
    // temp ignored
    gx = (int16_t)((data[8] << 8) | data[9]);
    gy = (int16_t)((data[10] << 8) | data[11]);
    gz = (int16_t)((data[12] << 8) | data[13]);
    return true;
}

bool readMagRaw(int16_t &mx, int16_t &my, int16_t &mz)
{
    // Check data ready
    uint8_t st1 = 0;
    if (!i2cReadNFrom(AK8963_ADDR, AK8963_ST1, 1, &st1))
        return false;
    if (!(st1 & 0x01))
        return false; // not ready

    uint8_t buf[7];
    if (!i2cReadNFrom(AK8963_ADDR, AK8963_XOUT_L, 7, buf))
        return false;

    // Little-endian: X L, X H, Y L, Y H, Z L, Z H
    mx = (int16_t)((buf[1] << 8) | buf[0]);
    my = (int16_t)((buf[3] << 8) | buf[2]);
    mz = (int16_t)((buf[5] << 8) | buf[4]);

    uint8_t st2 = buf[6];
    if (st2 & 0x08)
        return false; // overflow
    return true;
}

// ----------------- Device init -----------------
bool mpu_init()
{
    // Wake device
    i2cWrite8To(MPU9250_ADDR, PWR_MGMT_1, 0x00);
    delay(100);

    uint8_t who = 0;
    if (!i2cReadNFrom(MPU9250_ADDR, WHO_AM_I, 1, &who))
        return false;
    // MPU9250 WHO_AM_I typically 0x71
    if (who != 0x71)
    {
        Serial.printf("MPU WHO_AM_I != 0x71 (got 0x%02X)\n", who);
        // continue attempt but return false to indicate mismatch
        return false;
    }

    i2cWrite8To(MPU9250_ADDR, CONFIG_REG, DLPF_CFG & 0x07);
    i2cWrite8To(MPU9250_ADDR, SMPLRT_DIV, 4); // 200 Hz nominal
    i2cWrite8To(MPU9250_ADDR, GYRO_CONFIG, GYRO_FS_500);
    i2cWrite8To(MPU9250_ADDR, ACCEL_CONFIG, ACCEL_FS_4G);

    // Enable I2C bypass so AK8963 is directly accessible on main bus (sets BYPASS_EN bit)
    // INT_PIN_CFG register, set bit1 (0x02)
    uint8_t int_pin_cfg = 0x02;
    i2cWrite8To(MPU9250_ADDR, INT_PIN_CFG, int_pin_cfg);
    delay(10);

    return true;
}

bool ak8963_init()
{
    // Power down
    i2cWrite8To(AK8963_ADDR, AK8963_CNTL1, 0x00);
    delay(10);

    // Enter fuse ROM access mode to read ASAX/ASAY/ASAZ
    i2cWrite8To(AK8963_ADDR, AK8963_CNTL1, 0x0F);
    delay(10);

    uint8_t asax[3];
    if (!i2cReadNFrom(AK8963_ADDR, AK8963_ASAX, 3, asax))
        return false;

    // sensitivity adjustment: (ASAX - 128)/256 + 1
    mag_adj[0] = ((int)asax[0] - 128) / 256.0f + 1.0f;
    mag_adj[1] = ((int)asax[1] - 128) / 256.0f + 1.0f;
    mag_adj[2] = ((int)asax[2] - 128) / 256.0f + 1.0f;

    // Power down again
    i2cWrite8To(AK8963_ADDR, AK8963_CNTL1, 0x00);
    delay(10);

    // Continuous measurement mode 2 (100Hz), 16-bit output -> CNTL1 = 0x16
    i2cWrite8To(AK8963_ADDR, AK8963_CNTL1, 0x16);
    delay(10);

    // Optional: check AK8963 who_am_i (not all chips respond to 0x00 read)
    return true;
}

// ----------------- Calibration (accel+gyro) -----------------
bool calibrateMPU()
{
    const int DISCARD = CAL_DISCARD;
    const int N = CAL_SAMPLES;
    const int READ_DELAY_MS = 2;

    long long ax_sum = 0, ay_sum = 0, az_sum = 0;
    long long gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int good = 0;

    Serial.println("Calibration: discarding samples - keep device still...");
    for (int i = 0; i < DISCARD; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        if (!readRawMPU(ax, ay, az, gx, gy, gz))
            return false;
        delay(READ_DELAY_MS);
    }

    Serial.println("Calibration: collecting samples...");
    for (int i = 0; i < N; i++)
    {
        int16_t ax, ay, az, gx, gy, gz;
        if (!readRawMPU(ax, ay, az, gx, gy, gz))
            return false;
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        good++;
        delay(READ_DELAY_MS);
    }
    if (good < N)
        return false;

    float invN = 1.0f / (float)good;
    float ax_mean = (float)ax_sum * invN;
    float ay_mean = (float)ay_sum * invN;
    float az_mean = (float)az_sum * invN;
    float gx_mean = (float)gx_sum * invN;
    float gy_mean = (float)gy_sum * invN;
    float gz_mean = (float)gz_sum * invN;

    gx_bias = gx_mean;
    gy_bias = gy_mean;
    gz_bias = gz_mean;

    // detect which axis is aligned with gravity
    float ax_abs = fabsf(ax_mean), ay_abs = fabsf(ay_mean), az_abs = fabsf(az_mean);
    int up_idx = 0;
    float up_val = ax_abs;
    if (ay_abs > up_val)
    {
        up_idx = 1;
        up_val = ay_abs;
    }
    if (az_abs > up_val)
    {
        up_idx = 2;
        up_val = az_abs;
    }

    float up_g = up_val / ACC_LSB_PER_G;
    if (up_g < 0.7f || up_g > 1.3f)
    {
        Serial.println("Calibration fail: sensor not flat/still enough.");
        return false;
    }

    ax_bias = ax_mean;
    ay_bias = ay_mean;
    az_bias = az_mean;

    float g_target_raw = ACC_LSB_PER_G;
    if ((up_idx == 0 && ax_mean < 0) || (up_idx == 1 && ay_mean < 0) || (up_idx == 2 && az_mean < 0))
    {
        g_target_raw = -ACC_LSB_PER_G;
    }
    if (up_idx == 0)
        ax_bias = ax_mean - g_target_raw;
    else if (up_idx == 1)
        ay_bias = ay_mean - g_target_raw;
    else
        az_bias = az_mean - g_target_raw;

    Serial.println("Calibration done.");
    return true;
}

void print_offsets()
{
    Serial.println("CALIBRATION_OFFSETS_BEGIN");
    Serial.printf("GX_BIAS_LSB,%.3f\n", gx_bias);
    Serial.printf("GY_BIAS_LSB,%.3f\n", gy_bias);
    Serial.printf("GZ_BIAS_LSB,%.3f\n", gz_bias);
    Serial.printf("AX_BIAS_LSB,%.3f\n", ax_bias);
    Serial.printf("AY_BIAS_LSB,%.3f\n", ay_bias);
    Serial.printf("AZ_BIAS_LSB,%.3f\n", az_bias);
    Serial.printf("MAG_ADJ_X,%.6f\n", mag_adj[0]);
    Serial.printf("MAG_ADJ_Y,%.6f\n", mag_adj[1]);
    Serial.printf("MAG_ADJ_Z,%.6f\n", mag_adj[2]);
    Serial.printf("MAG_BIAS_X_LSB,%.3f\n", mag_bias[0]);
    Serial.printf("MAG_BIAS_Y_LSB,%.3f\n", mag_bias[1]);
    Serial.printf("MAG_BIAS_Z_LSB,%.3f\n", mag_bias[2]);
    Serial.println("CALIBRATION_OFFSETS_END");
}

// ----------------- Math helpers -----------------
static inline float rad2deg(float r) { return r * (180.0f / (float)M_PI); }
static inline float deg2rad(float d) { return d * ((float)M_PI / 180.0f); }

// ----------------- Setup / Loop -----------------
void setup()
{
    Serial.begin(115200);
    delay(200);

    // I2C init for ESP32-S3 (set your SDA/SCL pins)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // SDA, SCL, freq
    delay(20);

    Serial.println("INIT: MPU9250 on ESP32-S3 I2C...");
    if (!mpu_init())
    {
        Serial.println("INIT_FAIL (MPU)");
        // continue - we may still be able to talk to AK8963 if MPU init partially worked
        // return;
    }
    else
    {
        Serial.println("MPU INIT_OK");
    }

    if (!ak8963_init())
    {
        Serial.println("AK8963_INIT_FAIL");
        // continue - magnetometer may be unavailable
    }
    else
    {
        Serial.println("AK8963 INIT_OK");
    }

    Serial.println("CALIBRATION_START: keep the sensor flat and still...");
    delay(2000);
    if (!calibrateMPU())
    {
        Serial.println("CALIBRATION_FAIL");
        print_offsets();
        // continue anyway
    }
    else
    {
        Serial.println("CALIBRATION_OK");
        print_offsets();
    }

    // Initialize roll/pitch from accelerometer (yaw kept from mag later)
    int16_t ax, ay, az, gx, gy, gz;
    if (readRawMPU(ax, ay, az, gx, gy, gz))
    {
        float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
        float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
        float az_g = (az - az_bias) / ACC_LSB_PER_G;
        roll_deg = rad2deg(atan2f(ay_g, az_g));
        pitch_deg = rad2deg(atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)));
        yaw_deg = 0.0f;
    }

    Serial.println("ORIENTATION_READY");
    Serial.println("Header: roll_deg,pitch_deg,yaw_deg");
}

void loop()
{
    static uint32_t last_ms = millis();
    uint32_t now = millis();
    uint32_t dt_ms = now - last_ms;
    if (dt_ms < (uint32_t)SAMPLE_PERIOD_MS)
        return;
    last_ms = now;
    float dt = dt_ms * 1e-3f;

    int16_t ax, ay, az, gx, gy, gz;
    if (!readRawMPU(ax, ay, az, gx, gy, gz))
        return;

    // Bias-correct and scale
    float ax_g = (ax - ax_bias) / ACC_LSB_PER_G;
    float ay_g = (ay - ay_bias) / ACC_LSB_PER_G;
    float az_g = (az - az_bias) / ACC_LSB_PER_G;
    float gx_dps = (gx - gx_bias) / GYRO_LSB_PER_DPS;
    float gy_dps = (gy - gy_bias) / GYRO_LSB_PER_DPS;
    float gz_dps = (gz - gz_bias) / GYRO_LSB_PER_DPS;

    // 1) Integrate gyro
    roll_deg += gx_dps * dt;
    pitch_deg += gy_dps * dt;
    yaw_deg += gz_dps * dt; // relative yaw (will be corrected by mag)

    // 2) Compute accel-only tilt (gravity-based)
    float norm = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);
    if (norm > 0.5f && norm < 2.0f)
    {
        float roll_acc = rad2deg(atan2f(ay_g, az_g));
        float pitch_acc = rad2deg(atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)));

        // 3) Complementary blend for roll/pitch
        roll_deg = ALPHA_RP * roll_deg + (1.0f - ALPHA_RP) * roll_acc;
        pitch_deg = ALPHA_RP * pitch_deg + (1.0f - ALPHA_RP) * pitch_acc;
    }

    // 4) Read magnetometer and compute tilt-compensated heading (if available)
    int16_t mx_raw, my_raw, mz_raw;
    if (readMagRaw(mx_raw, my_raw, mz_raw))
    {
        // Convert to microTesla with factory adjustment and calibration
        float mx = (float)mx_raw * mag_adj[0];
        float my = (float)my_raw * mag_adj[1];
        float mz = (float)mz_raw * mag_adj[2];

        // Apply hard-iron bias (LSB) then scale -> convert to microtesla
        // (mag_bias and mag_scale are placeholders; perform external calibration to fill them)
        float mx_corr = (mx - mag_bias[0]) * mag_scale[0] * AK8963_LSB_uT;
        float my_corr = (my - mag_bias[1]) * mag_scale[1] * AK8963_LSB_uT;
        float mz_corr = (mz - mag_bias[2]) * mag_scale[2] * AK8963_LSB_uT;

        // Tilt compensation
        float roll_rad = deg2rad(roll_deg);
        float pitch_rad = deg2rad(pitch_deg);

        float sinR = sinf(roll_rad), cosR = cosf(roll_rad);
        float sinP = sinf(pitch_rad), cosP = cosf(pitch_rad);

        float mx_comp = mx_corr * cosP + mz_corr * sinP;
        float my_comp = mx_corr * sinR * sinP + my_corr * cosR - mz_corr * sinR * cosP;

        float yaw_mag = rad2deg(atan2f(-my_comp, mx_comp)); // note sign to match right-hand coords

        // Normalize yaw_mag to [-180,180]
        if (yaw_mag > 180.0f)
            yaw_mag -= 360.0f;
        else if (yaw_mag < -180.0f)
            yaw_mag += 360.0f;

        // Fuse yaw
        // small-angle discontinuity handling: if difference > 180, wrap to nearest equivalent
        float diff = yaw_mag - yaw_deg;
        if (diff > 180.0f)
            diff -= 360.0f;
        else if (diff < -180.0f)
            diff += 360.0f;
        float yaw_fused = yaw_deg + diff; // bring yaw_mag into yaw_deg frame
        yaw_deg = ALPHA_YAW * yaw_deg + (1.0f - ALPHA_YAW) * yaw_fused;

        // wrap yaw
        if (yaw_deg > 180.0f)
            yaw_deg -= 360.0f;
        else if (yaw_deg < -180.0f)
            yaw_deg += 360.0f;
    }

    // Output CSV: roll_deg,pitch_deg,yaw_deg
    Serial.printf("%.2f,%.2f,%.2f\n", roll_deg, pitch_deg, yaw_deg);
}
