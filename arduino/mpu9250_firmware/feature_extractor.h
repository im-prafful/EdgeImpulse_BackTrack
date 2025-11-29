#pragma once
#include <Arduino.h>

// EI-compatible DSP wrapper interface (template)
// You can replace dsp_block.cpp/h with a real Edge Impulse export later.

// Initialize extractor with sample rate and channel count
bool featureExtractorBegin(int sample_rate_hz, int window_size_samples);

// Feed one sample (9-axis) into sliding window buffer
void featureExtractorPushSample(const float *sample9); // sample9 = [ax,ay,az,gx,gy,gz,mx,my,mz]

// When ready, extract features into provided buffer (size FEATURE_VECTOR_SIZE)
// Returns true if feature vector produced
bool featureExtractorGetFeatures(float *out_features, size_t out_len);

// Reset internal state
void featureExtractorReset();
