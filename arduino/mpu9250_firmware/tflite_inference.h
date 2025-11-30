#pragma once
#include <Arduino.h>

bool tfliteBegin();
int tfliteClassify(const float *feature_vector, size_t feature_len);
// returns index of highest score (0..N-1) or -1 on error
float tfliteGetClassConfidence(int idx);
