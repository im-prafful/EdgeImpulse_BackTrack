#include "posture_detection.h"
#include <Arduino.h>

float computeTilt(const IMUData &imu)
{
    // Basic tilt angle using accelerometer only
    float angle = atan2(imu.ax, imu.az) * 57.3;
    return angle;
}

PostureState classifyPosture(float currentAngle)
{
    float neutral = getNeutralAngle();
    float diff = currentAngle - neutral;

    if (abs(diff) < 5)
        return GOOD_POSTURE;

    if (abs(diff) < 12)
        return SLIGHT_SLOUCH;

    return BAD_SLOUCH;
}
