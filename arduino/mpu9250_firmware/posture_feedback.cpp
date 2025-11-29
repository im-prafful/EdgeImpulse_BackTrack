#include "posture_feedback.h"
#include "config.h"
#include <Arduino.h>

PostureFeedback::PostureFeedback(int motorPin) : _motorPin(motorPin), _strength(200) {}

void PostureFeedback::begin()
{
    pinMode(_motorPin, OUTPUT);
    digitalWrite(_motorPin, LOW);
    pinMode(LED_STATUS_PIN, OUTPUT);
}

void PostureFeedback::setVibrationStrength(uint8_t s) { _strength = s; }

void PostureFeedback::handle(PostureState state)
{
    if (state == POSTURE_BAD)
    {
        // short vibration pulse
        analogWrite(_motorPin, _strength); // if using MOSFET driver; else digitalWrite
        delay(120);
        analogWrite(_motorPin, 0);
        digitalWrite(LED_STATUS_PIN, HIGH);
        delay(40);
        digitalWrite(LED_STATUS_PIN, LOW);
    }
    else if (state == POSTURE_SLIGHT)
    {
        // light reminder
        analogWrite(_motorPin, _strength / 3);
        delay(60);
        analogWrite(_motorPin, 0);
    }
    else
    {
        // nothing
        analogWrite(_motorPin, 0);
    }
}
