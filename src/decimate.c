/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/*
 * Power-of-two decimation for the rates below the hardware's 10 MSps.
 *
 * Each stage is one half-band FIR that halves the rate. The last stage sets
 * the passband, the inner 80 percent of the output band with 0.0004 dB ripple,
 * and rejects 92.8 dB, below the 14-bit ADC's floor. An earlier stage only has
 * to keep aliases out of that final passband, so its transition is wider and a
 * much shorter filter rejects more than 96 dB. Designed with
 * scipy.signal.remez(n, [0, fp, 0.5 - fp, 0.5], [1, 0]), fp 0.2 for the last
 * stage and 0.4 / 2^(k + 1) for one with k stages after it; the even offsets,
 * zero by symmetry, are exactly zero and the center is exactly 0.5. Stages run
 * in float, whose 24-bit mantissa holds the 14-bit samples and what decimation
 * gains, and the output keeps two extra bits: full scale 32768, not 8192.
 */
#include "internal.h"
#include <math.h>
#include <string.h>

/* Taps at offsets 1, 3, 5, ... from the center, the same on both sides. A
 * filter with side such taps has 4 * side - 1 taps in all.
 */
static const float last[14] = { /* 55 taps, 92.8 dB */
    3.166975187e-01f, -1.013523991e-01f, 5.602288840e-02f, -3.533317814e-02f,
    2.321414363e-02f, -1.530794360e-02f, 9.923265126e-03f, -6.229764820e-03f,
    3.737238043e-03f, -2.110598397e-03f, 1.100672188e-03f, -5.143106130e-04f,
    2.040145673e-04f, -6.206497305e-05f,
};
static const float one_after[5] = { /* 19 taps, 109.6 dB */
    3.060336229e-01f, -7.405318430e-02f, 2.263745261e-02f, -5.277493643e-03f, 6.612020220e-04f,
};
static const float two_after[3] = { /* 11 taps, 105.7 dB */
    2.943997076e-01f, -5.102741817e-02f, 6.630083775e-03f,
};
static const float three_after[2] = { /* 7 taps, 96.5 dB */
    2.818225965e-01f, -3.182924556e-02f,
};
/* The one stage after the chip's channel filter, which already keeps everything
 * past 0.3 of the input rate 40 dB or more down: the same band edges as last,
 * 51.2 dB, 0.05 dB ripple, at half the work. remez(27, [0, 0.2, 0.3, 0.5]). */
static const float after_channel[7] = { /* 27 taps, 51.2 dB */
    3.153677459e-01f, -9.755306284e-02f, 5.020501980e-02f, -2.821058868e-02f,
    1.555949263e-02f, -7.910530616e-03f, 3.864509926e-03f,
};

void bs_decimator_reset(struct bs_decimator *decimator, unsigned stages, bool after_channel)
{
    memset(decimator, 0, sizeof(*decimator));
    decimator->stages = stages;
    decimator->after_channel = after_channel;
}

/* One component of a stage: y[j] from the even samples around the odd center
 * 2j + 2 * side - 1 and that center itself. Each output keeps its sum in a
 * register through all the taps, and the outputs are computed four at a time.
 * Inlined with side a constant, so the taps unroll and the outputs vectorize.
 */
static inline __attribute__((always_inline)) void fir(
    float *restrict y, const float *restrict even, const float *restrict odd, size_t outputs,
    const float *taps, unsigned side)
{
    for (size_t j = 0; j < outputs; ++j) {
        float sum = 0.5f * odd[j + side - 1];
        for (unsigned t = 0; t < side; ++t)
            sum += taps[t] * (even[j + side - 1 - t] + even[j + side + t]);
        y[j] = sum;
    }
}

/* The filter for a stage with remaining stages after it. */
static void fir_stage(float *y, const float *even, const float *odd, size_t outputs,
                      unsigned remaining, bool after)
{
    switch (remaining) {
    case 0:
        if (after)
            fir(y, even, odd, outputs, after_channel, 7);
        else
            fir(y, even, odd, outputs, last, 14);
        break;
    case 1: fir(y, even, odd, outputs, one_after, 5); break;
    case 2: fir(y, even, odd, outputs, two_after, 3); break;
    default: fir(y, even, odd, outputs, three_after, 2); break;
    }
}

/* Halve n complex samples, which follow the stage's history in work, into out,
 * and keep the last samples as the next call's history. Every side tap falls on
 * an even sample and the center on an odd one: split them, I and Q apart, and
 * each output is a plain symmetric FIR over contiguous floats.
 */
static size_t halve(struct bs_decimator *d, unsigned stage, size_t n, float *out)
{
    const unsigned remaining = d->stages - 1 - stage;
    const size_t side = remaining == 0 ? (d->after_channel ? 7 : 14) :
                        remaining == 1 ? 5 : remaining == 2 ? 3 : 2;
    const size_t kept = 4 * side - 2, half = (kept + n) / 2, outputs = n / 2;
    float *base = d->work + 2 * (BS_HALFBAND_HISTORY - kept);
    float *ei = d->planar[0], *eq = d->planar[1], *oi = d->planar[2], *oq = d->planar[3];
    float *yi = d->planar[4], *yq = d->planar[5];
    memcpy(base, d->history[stage], sizeof(float) * 2 * kept);
    for (size_t m = 0; m < half; ++m) {
        ei[m] = base[4 * m];
        eq[m] = base[4 * m + 1];
        oi[m] = base[4 * m + 2];
        oq[m] = base[4 * m + 3];
    }
    fir_stage(yi, ei, oi, outputs, remaining, d->after_channel);
    fir_stage(yq, eq, oq, outputs, remaining, d->after_channel);
    for (size_t j = 0; j < outputs; ++j) {
        out[2 * j] = yi[j];
        out[2 * j + 1] = yq[j];
    }
    memcpy(d->history[stage], base + 2 * n, sizeof(float) * 2 * kept);
    return outputs;
}

/* Scale by the two extra bits, round half away from zero, then clip. Integer
 * clipping vectorizes without libm; the values stay far inside int32.
 */
static void to_samples(int16_t *restrict out, const float *restrict in, size_t values)
{
    for (size_t k = 0; k < values; ++k) {
        const float v = in[k] * BS_DECIMATED_GAIN;
        const int32_t rounded = (int32_t)(v + copysignf(0.5f, v));
        out[k] = (int16_t)(rounded > 32767 ? 32767 : rounded < -32768 ? -32768 : rounded);
    }
}

size_t bs_decimate(struct bs_decimator *decimator, const int16_t *in, size_t samples,
                   int16_t *out)
{
    float *input = decimator->work + 2 * BS_HALFBAND_HISTORY;
    size_t n = samples;
    for (size_t k = 0; k < 2 * n; ++k)
        input[k] = in[k];
    for (unsigned s = 0; s < decimator->stages; ++s) {
        n = halve(decimator, s, n, decimator->stage);
        memcpy(input, decimator->stage, sizeof(float) * 2 * n);
    }
    to_samples(out, decimator->stage, 2 * n);
    return n;
}
