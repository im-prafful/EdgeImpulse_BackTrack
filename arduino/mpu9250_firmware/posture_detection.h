#pragma once
#include "calibration_utils.h"

enum PostureState
{
    GOOD_POSTURE,
    SLIGHT_SLOUCH,
    BAD_SLOUCH
};

float computeTilt(const IMUData &imu);
PostureState classifyPosture(float currentAngle);
