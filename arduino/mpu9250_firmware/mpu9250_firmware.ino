#include <Arduino.h>
#include "config.h"
#include "imu_driver.h"
#include "feature_extractor.h"
#include "tflite_inference.h"
#include "posture_feedback.h"

// Globals
IMUDriver imu;
PostureFeedback feedback;
float feature_buffer[FEATURE_VECTOR_SIZE];
float sample9[9];

unsigned long last_sample_ms = 0;

void setup()
{
    Serial.begin(SERIAL_BAUD);
    delay(200);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    if (!imu.begin())
    {
        Serial.println("IMU init failed");
    }
    else
    {
        Serial.println("IMU init ok");
    }

    featureExtractorBegin(SAMPLE_RATE_HZ, WINDOW_SAMPLES);
    tfliteBegin();
    feedback.begin();

    Serial.println("Postura+ firmware ready");
    // Optionally calibrate automatically briefly
    imu.calibrate(200, 500); // quick calibration; increase for accuracy
}

void loop()
{
    unsigned long now = millis();
    if (now - last_sample_ms < SAMPLE_PERIOD_MS)
    {
        // Non-blocking wait
        delay(1);
        return;
    }
    last_sample_ms = now;

    IMUSample s;
    if (!imu.readSample(s))
        return;

    // pack sample
    sample9[0] = s.ax;
    sample9[1] = s.ay;
    sample9[2] = s.az;
    sample9[3] = s.gx;
    sample9[4] = s.gy;
    sample9[5] = s.gz;
    sample9[6] = s.mx;
    sample9[7] = s.my;
    sample9[8] = s.mz;

    // push into extractor
    featureExtractorPushSample(sample9);

    // If we have enough samples (window filled), extract features and classify
    static int cycles = 0;
    cycles++;
    if (cycles >= WINDOW_SAMPLES)
    {
        bool ok = featureExtractorGetFeatures(feature_buffer, FEATURE_VECTOR_SIZE);
        if (ok)
        {
            int cls = tfliteClassify(feature_buffer, FEATURE_VECTOR_SIZE);
            PostureState state = POSTURE_GOOD;
            if (cls == 0)
                state = POSTURE_GOOD;
            else if (cls == 1)
                state = POSTURE_SLIGHT;
            else if (cls == 2)
                state = POSTURE_BAD;
            feedback.handle(state);

            if (DEBUG_SERIAL)
            {
                Serial.print("Class:");
                Serial.print(cls);
                Serial.print(", conf=");
                Serial.println(tfliteGetClassConfidence(cls), 4);
            }
        }
        cycles = 0;
    }

    // Optional: handle button to re-calibrate
    if (digitalRead(BUTTON_PIN) == LOW)
    {
        Serial.println("Button pressed - re-calibrating...");
        imu.calibrate(200, 1000);
        featureExtractorReset();
    }
}
