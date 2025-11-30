#include "feature_extractor.h"
#include "config.h"
#include <math.h>

// --- Simple, EI-compatible placeholder implementation ---
// Produces FEATURE_VECTOR_SIZE floats by computing per-channel stats (mean, std, min, max)
// and some simple spectral-ish energy features. This is a template to match EI layout.
// Replace with EI-generated dsp_block.cpp for exact match.

static int g_sample_rate = SAMPLE_RATE_HZ;
static int g_window_samples = WINDOW_SAMPLES;
static float *g_window = nullptr;
static int g_head = 0;
static int g_count = 0;

bool featureExtractorBegin(int sample_rate_hz, int window_size_samples)
{
    g_sample_rate = sample_rate_hz;
    g_window_samples = window_size_samples;
    if (g_window)
        free(g_window);
    // window contains 9 channels per sample: size = window_samples * 9
    g_window = (float *)malloc(sizeof(float) * g_window_samples * 9);
    if (!g_window)
        return false;
    memset(g_window, 0, sizeof(float) * g_window_samples * 9);
    g_head = 0;
    g_count = 0;
    return true;
}

void featureExtractorReset()
{
    if (g_window)
        memset(g_window, 0, sizeof(float) * g_window_samples * 9);
    g_head = 0;
    g_count = 0;
}

void featureExtractorPushSample(const float *sample9)
{
    if (!g_window)
        return;
    int idx = (g_head % g_window_samples) * 9;
    for (int i = 0; i < 9; i++)
        g_window[idx + i] = sample9[i];
    g_head = (g_head + 1) % g_window_samples;
    if (g_count < g_window_samples)
        g_count++;
}

static void compute_stats_for_channel(int ch, float *out_mean, float *out_std, float *out_min, float *out_max)
{
    if (g_count == 0)
    {
        *out_mean = *out_std = *out_min = *out_max = 0;
        return;
    }
    float sum = 0;
    float sumsq = 0;
    float mn = 1e9f;
    float mx = -1e9f;
    for (int i = 0; i < g_count; i++)
    {
        float v = g_window[i * 9 + ch];
        sum += v;
        sumsq += v * v;
        if (v < mn)
            mn = v;
        if (v > mx)
            mx = v;
    }
    float mean = sum / (float)g_count;
    float var = sumsq / (float)g_count - mean * mean;
    if (var < 0)
        var = 0;
    *out_mean = mean;
    *out_std = sqrtf(var);
    *out_min = mn;
    *out_max = mx;
}

bool featureExtractorGetFeatures(float *out_features, size_t out_len)
{
    if (!g_window)
        return false;
    // Our simple layout: for each of 9 channels compute mean,std,min,max -> 9*4 =36
    // Then compute cross-channel energy sums (9 values) -> total 45
    // Expand to FEATURE_VECTOR_SIZE by padding with zeros.
    size_t needed = FEATURE_VECTOR_SIZE;
    if (out_len < needed)
        return false;
    float features[FEATURE_VECTOR_SIZE];
    memset(features, 0, sizeof(features));

    int k = 0;
    for (int ch = 0; ch < 9 && k + 4 <= FEATURE_VECTOR_SIZE; ch++)
    {
        float mean, stdv, mn, mx;
        compute_stats_for_channel(ch, &mean, &stdv, &mn, &mx);
        features[k++] = mean;
        features[k++] = stdv;
        features[k++] = mn;
        features[k++] = mx;
    }
    // energy per channel (sum squares)
    for (int ch = 0; ch < 9 && k < FEATURE_VECTOR_SIZE; ch++)
    {
        float energy = 0;
        for (int i = 0; i < g_count; i++)
        {
            float v = g_window[i * 9 + ch];
            energy += v * v;
        }
        features[k++] = energy;
    }
    // fill remaining with zeros
    for (; k < (int)FEATURE_VECTOR_SIZE; k++)
        features[k] = 0.0f;

    // copy out
    for (size_t i = 0; i < needed; i++)
        out_features[i] = features[i];
    return true;
}
