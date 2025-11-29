#pragma once
#include <Arduino.h>

struct IMUData
{
    float ax, ay, az;
    float gx, gy, gz;
};

void initIMU();
IMUData readIMU();

void performCalibration();
float getNeutralAngle();
