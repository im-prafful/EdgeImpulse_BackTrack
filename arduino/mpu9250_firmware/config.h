#pragma once

// ---------- Hardware pins ----------
#define MOTOR_PIN 13     // vibration motor control (use transistor)
#define LED_STATUS_PIN 2 // optional status LED
#define BUTTON_PIN 0     // optional calibration trigger

// ---------- IMU / sampling ----------
#define I2C_SDA_PIN 8 // change to your board wiring
#define I2C_SCL_PIN 9
#define SAMPLE_PERIOD_MS 5 // ~200 Hz sampling
#define WINDOW_SIZE_MS 500 // sliding window length (ms)
#define SAMPLE_RATE_HZ (1000 / SAMPLE_PERIOD_MS)
#define WINDOW_SAMPLES ((WINDOW_SIZE_MS) / SAMPLE_PERIOD_MS)

// ---------- Feature sizes (placeholder) ----------
#define FEATURE_VECTOR_SIZE 64 // must match model input size (update after training)

// ---------- TFLite model name (C array) ----------
#define TFLITE_MODEL_NAME posture_model_tflite
extern const unsigned char TFLITE_MODEL_NAME[]; // provided by generated tflite_model.h

// ---------- Misc ----------
#define SERIAL_BAUD 115200
#define DEBUG_SERIAL true
