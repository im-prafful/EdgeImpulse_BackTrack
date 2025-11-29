#include "calibration_utils.h"
#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;
float neutralAngle = 0;

void initIMU()
{
    mpu.setup(0x68);
    delay(1000);
}

IMUData readIMU()
{
    mpu.update();

    IMUData d;
    d.ax = mpu.getAccX();
    d.ay = mpu.getAccY();
    d.az = mpu.getAccZ();
    d.gx = mpu.getGyroX();
    d.gy = mpu.getGyroY();
    d.gz = mpu.getGyroZ();
    return d;
}

void performCalibration()
{
    const int samples = 100;
    float total = 0;

    for (int i = 0; i < samples; i++)
    {
        mpu.update();
        float angle = atan2(mpu.getAccX(), mpu.getAccZ()) * 57.3;
        total += angle;
        delay(20);
    }

    neutralAngle = total / samples;
}

float getNeutralAngle()
{
    return neutralAngle;
}
