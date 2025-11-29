#include "vibration_feedback.h"
#include <Arduino.h>

#define MOTOR_PIN 13

void initVibrationMotor()
{
    pinMode(MOTOR_PIN, OUTPUT);
    digitalWrite(MOTOR_PIN, LOW);
}

void handleFeedback(PostureState state)
{
    if (state == BAD_SLOUCH)
    {
        digitalWrite(MOTOR_PIN, HIGH);
        delay(100);
        digitalWrite(MOTOR_PIN, LOW);
    }
}
