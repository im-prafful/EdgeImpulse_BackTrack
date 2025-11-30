#pragma once
#include <Arduino.h>

enum PostureState
{
    POSTURE_GOOD = 0,
    POSTURE_SLIGHT = 1,
    POSTURE_BAD = 2
};

class PostureFeedback
{
public:
    PostureFeedback(int motorPin = MOTOR_PIN);
    void begin();
    void handle(PostureState state);
    void setVibrationStrength(uint8_t strength);

private:
    int _motorPin;
    uint8_t _strength;
};
