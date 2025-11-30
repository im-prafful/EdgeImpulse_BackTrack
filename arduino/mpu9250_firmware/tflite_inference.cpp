#include "tflite_inference.h"
#include "config.h"
#include "tflite_model.h"

// TinyML includes
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// Interpreter globals
namespace
{
    const tflite::Model *model = nullptr;
    tflite::MicroInterpreter *interpreter = nullptr;
    tflite::MicroErrorReporter micro_error_reporter;
    tflite::AllOpsResolver resolver;
    // Arena size estimate — tune if your model is big. 64KB default; increase if needed.
    const int kTensorArenaSize = 128 * 1024;
    static uint8_t tensor_arena[kTensorArenaSize];
    TfLiteTensor *inputTensor = nullptr;
    TfLiteTensor *outputTensor = nullptr;
    int outputLen = 0;
}

bool tfliteBegin()
{
    model = tflite::GetModel(TFLITE_MODEL_NAME);
    if (model->version() != TFLITE_SCHEMA_VERSION)
    {
        Serial.println("TFLITE: Schema version mismatch!");
        return false;
    }
    interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, kTensorArenaSize, &micro_error_reporter);
    TfLiteStatus alloc = interpreter->AllocateTensors();
    if (alloc != kTfLiteOk)
    {
        Serial.println("TFLITE: AllocateTensors failed");
        return false;
    }
    inputTensor = interpreter->input(0);
    outputTensor = interpreter->output(0);
    outputLen = 1;
    for (int i = 0; i < outputTensor->dims->size; i++)
    {
        if (i == 0)
            continue;
    }
    return true;
}

int tfliteClassify(const float *feature_vector, size_t feature_len)
{
    if (!interpreter)
        return -1;
    // Input check: assume input is float32 (or quantized). Handle both.
    if (inputTensor->type == kTfLiteFloat32)
    {
        float *in = inputTensor->data.f;
        size_t n = feature_len;
        for (size_t i = 0; i < n; i++)
            in[i] = feature_vector[i];
    }
    else if (inputTensor->type == kTfLiteInt8)
    {
        // quantize
        float scale = inputTensor->params.scale;
        int zero = inputTensor->params.zero_point;
        for (size_t i = 0; i < feature_len; i++)
        {
            int8_t q = (int8_t)round(feature_vector[i] / scale) + (int8_t)zero;
            inputTensor->data.int8[i] = q;
        }
    }
    else
    {
        // unsupported
        return -1;
    }
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk)
        return -1;

    int best = -1;
    float best_score = -1e9;
    if (outputTensor->type == kTfLiteFloat32)
    {
        float *out = outputTensor->data.f;
        int n = outputTensor->bytes / sizeof(float);
        for (int i = 0; i < n; i++)
        {
            if (out[i] > best_score)
            {
                best_score = out[i];
                best = i;
            }
        }
    }
    else if (outputTensor->type == kTfLiteInt8)
    {
        int8_t *out = outputTensor->data.int8;
        int n = outputTensor->bytes / sizeof(int8_t);
        for (int i = 0; i < n; i++)
        {
            float deq = (out[i] - outputTensor->params.zero_point) * outputTensor->params.scale;
            if (deq > best_score)
            {
                best_score = deq;
                best = i;
            }
        }
    }
    return best;
}

float tfliteGetClassConfidence(int idx)
{
    if (!outputTensor)
        return 0.0f;
    if (outputTensor->type == kTfLiteFloat32)
    {
        return outputTensor->data.f[idx];
    }
    else if (outputTensor->type == kTfLiteInt8)
    {
        int8_t v = outputTensor->data.int8[idx];
        return (v - outputTensor->params.zero_point) * outputTensor->params.scale;
    }
    return 0.0f;
}
