// Tiny MLP policy runner. Weights come from policy_weights.h (tools/onnx_to_header.py).
#pragma once
#include <stdint.h>
namespace Policy {
int  inputSize();
int  outputSize();
/** obs[inputSize()] -> act[outputSize()], applying the exported normalizer and output head. */
void forward(const float* obs, float* act);
/** Average microseconds per forward over `iters` runs on a zero observation. */
uint32_t benchmarkUs(int iters);
const char* describe();
}
// upper bound for the observation buffer in main.cpp (101 for the ODM v2 contract)
#ifndef POLICY_OBS_MAX
#define POLICY_OBS_MAX 128
#endif
