#include "policy.h"
#include <Arduino.h>
#include <math.h>
#include "policy_weights.h"

namespace {
float bufA[POLICY_MAX_WIDTH];
float bufB[POLICY_MAX_WIDTH];

inline float activate(float x) {
#if POLICY_ACT == 0      // ELU
    return x > 0 ? x : (expf(x) - 1.0f);
#elif POLICY_ACT == 1    // swish / SiLU  (x * sigmoid(x))
    return x / (1.0f + expf(-x));
#elif POLICY_ACT == 2    // ReLU
    return x > 0 ? x : 0;
#else                    // tanh
    return tanhf(x);
#endif
}
}

namespace Policy {
int inputSize()  { return POLICY_IN; }
int outputSize() { return POLICY_OUT; }
const char* describe() { return POLICY_DESCRIPTION; }

void forward(const float* obs, float* act) {
    float* x = bufA; float* y = bufB;
    for (int i = 0; i < POLICY_IN; i++) x[i] = (obs[i] - POLICY_NORM_MEAN[i]) * POLICY_NORM_SCALE[i];
    for (int l = 0; l < POLICY_NLAYERS; l++) {
        const PolicyLayer& L = POLICY_LAYERS[l];
        const bool last = (l == POLICY_NLAYERS - 1);
        for (int o = 0; o < L.out; o++) {
            const float* w = L.W + (size_t)o * L.in;
            float acc = L.b[o];
            for (int i = 0; i < L.in; i++) acc += w[i] * x[i];
            y[o] = last ? acc : activate(acc);
        }
        float* t = x; x = y; y = t;
    }
    // output head: optional split (mean | log_std) and tanh squash
    for (int o = 0; o < POLICY_OUT; o++) {
        float v = x[o];
#if POLICY_OUT_TANH
        v = tanhf(v);
#endif
        act[o] = v;
    }
}

uint32_t benchmarkUs(int iters) {
    static float obs[POLICY_IN]; static float act[POLICY_OUT];
    for (int i = 0; i < POLICY_IN; i++) obs[i] = 0.0f;
    uint32_t t0 = micros();
    for (int k = 0; k < iters; k++) forward(obs, act);
    return (micros() - t0) / (uint32_t)(iters > 0 ? iters : 1);
}
}
