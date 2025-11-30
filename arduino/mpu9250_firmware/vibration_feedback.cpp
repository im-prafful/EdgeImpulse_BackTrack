#include "posture_feedback.h"
#include "config.h"
#include <Arduino.h>

PostureFeedback::PostureFeedback(int motorPin)
    : _motorPin(motorPin), _strength(200) {}

void PostureFeedback::begin()
{
    pinMode(_motorPin, OUTPUT);
    digitalWrite(_motorPin, LOW);
}

void PostureFeedback::setVibrationStrength(uint8_t s)
{
    _strength = s;
}

void PostureFeedback::handle(PostureState state)
{
    if (state == POSTURE_BAD)
    {
        analogWrite(_motorPin, _strength);
        delay(120);
        analogWrite(_motorPin, 0);
    }
    else if (state == POSTURE_SLIGHT)
    {
        analogWrite(_motorPin, _strength / 3);
        delay(60);
        analogWrite(_motorPin, 0);
    }
    else
    {
        analogWrite(_motorPin, 0);
    }
}
