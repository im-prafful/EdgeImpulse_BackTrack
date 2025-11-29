#include "imu_driver.h"
#include <Wire.h>

// If you have a library (SparkFun/MPU9250) you can swap these low-level helpers.
// Here we use direct I2C reads for portability.

#define MPU9250_ADDR 0x68
#define AK8963_ADDR 0x0C

// MPU regs (subset)
#define ACCEL_XOUT_H 0x3B
#define PWR_MGMT_1 0x6B
#define INT_PIN_CFG 0x37

static inline uint8_t i2c_read8(uint8_t dev, uint8_t reg)
{
    Wire.beginTransmission((int)dev);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((int)dev, 1, true);
    return Wire.read();
}

static inline bool i2c_readN(uint8_t dev, uint8_t reg, uint8_t n, uint8_t *buf)
{
    Wire.beginTransmission((int)dev);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    int got = Wire.requestFrom((int)dev, (int)n, (int)true);
    if (got != n)
        return false;
    for (int i = 0; i < n; ++i)
        buf[i] = Wire.read();
    return true;
}

IMUDriver::IMUDriver() : ax_bias(0), ay_bias(0), az_bias(0),
                         gx_bias(0), gy_bias(0), gz_bias(0),
                         _addr(MPU9250_ADDR), _initialized(false) {}

bool IMUDriver::begin(uint8_t addr)
{
    _addr = addr;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
    delay(10);
    // Wakeup
    Wire.beginTransmission((int)_addr);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(100);
    // Enable I2C bypass to talk to AK8963
    Wire.beginTransmission((int)_addr);
    Wire.write(INT_PIN_CFG);
    Wire.write(0x02);
    Wire.endTransmission(true);
    delay(10);
    _initialized = true;
    return true;
}

bool IMUDriver::readRaw(int16_t &ax, int16_t &ay, int16_t &az,
                        int16_t &gx, int16_t &gy, int16_t &gz,
                        int16_t &mx, int16_t &my, int16_t &mz)
{
    uint8_t buf[14];
    if (!i2c_readN(_addr, ACCEL_XOUT_H, 14, buf))
        return false;
    ax = (int16_t)((buf[0] << 8) | buf[1]);
    ay = (int16_t)((buf[2] << 8) | buf[3]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gx = (int16_t)((buf[8] << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Try read magnetometer via AK8963
    uint8_t magbuf[7];
    if (!i2c_readN(AK8963_ADDR, 0x03, 7, magbuf))
    {
        mx = my = mz = 0;
    }
    else
    {
        mx = (int16_t)((magbuf[1] << 8) | magbuf[0]);
        my = (int16_t)((magbuf[3] << 8) | magbuf[2]);
        mz = (int16_t)((magbuf[5] << 8) | magbuf[4]);
    }
    return true;
}

bool IMUDriver::readSample(IMUSample &s)
{
    if (!_initialized)
        return false;
    int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
    if (!readRaw(ax, ay, az, gx, gy, gz, mx, my, mz))
        return false;

    // Convert LSB -> units (approx) and apply biases
    // Using FSR ±4g: 8192 LSB/g ; gyro ±500 dps: 65.5 LSB/dps ; magnetometer LSB->uT conversion handled later
    s.ax = ((float)ax - ax_bias) / 8192.0f;
    s.ay = ((float)ay - ay_bias) / 8192.0f;
    s.az = ((float)az - az_bias) / 8192.0f;
    s.gx = ((float)gx - gx_bias) / 65.5f;
    s.gy = ((float)gy - gy_bias) / 65.5f;
    s.gz = ((float)gz - gz_bias) / 65.5f;
    s.mx = (float)mx;
    s.my = (float)my;
    s.mz = (float)mz;
    return true;
}

bool IMUDriver::calibrate(int discard, int samples)
{
    if (!_initialized)
        return false;
    long long ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0, gy_sum = 0, gz_sum = 0;
    int got = 0;
    for (int i = 0; i < discard; i++)
    {
        int16_t t;
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        if (!readRaw(ax, ay, az, gx, gy, gz, mx, my, mz))
            return false;
        delay(2);
    }
    for (int i = 0; i < samples; i++)
    {
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        if (!readRaw(ax, ay, az, gx, gy, gz, mx, my, mz))
            return false;
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        got++;
        delay(2);
    }
    if (!got)
        return false;
    float inv = 1.0f / (float)got;
    ax_bias = (float)ax_sum * inv;
    ay_bias = (float)ay_sum * inv;
    az_bias = (float)az_sum * inv;
    gx_bias = (float)gx_sum * inv;
    gy_bias = (float)gy_sum * inv;
    gz_bias = (float)gz_sum * inv;
    return true;
}

void IMUDriver::getAccelBias(float &bx, float &by, float &bz) const
{
    bx = ax_bias;
    by = ay_bias;
    bz = az_bias;
}
void IMUDriver::getGyroBias(float &bx, float &by, float &bz) const
{
    bx = gx_bias;
    by = gy_bias;
    bz = gz_bias;
}
