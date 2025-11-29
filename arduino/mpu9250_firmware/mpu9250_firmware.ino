/**
 * BackTrack - ESP32-S3 Firmware
 * MPU-9250 Posture Detection (MVP Version)
 * -----------------------------------------
 * Features:
 *  - Reads IMU (accel + gyro)
 *  - Calibration for neutral posture
 *  - Threshold-based posture detection
 *  - Vibration motor feedback
 *  - Blynk integration
 */

#include <Wire.h>
#include "calibration_utils.h"
#include "posture_detection.h"
#include "vibration_feedback.h"
#include "blynk_integration.h"

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    initIMU();
    initVibrationMotor();

    delay(1500);
    Serial.println("Starting Calibration...");
    performCalibration(); // User sits straight

#ifdef USE_BLYNK
    connectToBlynk();
#endif

    Serial.println("Setup complete.");
}

void loop()
{
    IMUData imu = readIMU();            // Read accel + gyro
    float tiltAngle = computeTilt(imu); // IMU fusion (simplified)
    PostureState state = classifyPosture(tiltAngle);

    handleFeedback(state); // Vibrate if slouching

    Serial.print("Angle:");
    Serial.print(tiltAngle);
    Serial.print(", State:");
    Serial.println(state);

#ifdef USE_BLYNK
    sendToBlynk(state, tiltAngle);
#endif

    delay(100);
}
