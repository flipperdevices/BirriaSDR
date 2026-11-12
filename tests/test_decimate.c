/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
#include "internal.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); exit(1); } } while (0)

enum { PACKET = 1024, PACKETS = 64 };

/* A complex tone at frequency (a fraction of the input rate) and amplitude. */
static void tone(int16_t *iq, size_t samples, double frequency, double amplitude)
{
    for (size_t n = 0; n < samples; ++n) {
        iq[2 * n] = (int16_t)lround(amplitude * cos(2 * M_PI * frequency * n));
        iq[2 * n + 1] = (int16_t)lround(amplitude * sin(2 * M_PI * frequency * n));
    }
}

/* Decimate packet by packet and return the output count. */
static bool after_channel; /* the one-stage filter behind the chip's channel filter */

static size_t run(struct bs_decimator *d, unsigned stages, const int16_t *in, size_t samples,
                  size_t chunk, int16_t *out)
{
    size_t produced = 0;
    bs_decimator_reset(d, stages, after_channel);
    for (size_t at = 0; at < samples; at += chunk)
        produced += bs_decimate(d, in + 2 * at, chunk, out + 2 * produced);
    return produced;
}

/* Largest and mean magnitude after the filters have settled. */
static void magnitude(const int16_t *iq, size_t samples, double *peak, double *mean)
{
    *peak = 0;
    *mean = 0;
    for (size_t n = samples / 4; n < samples; ++n) {
        const double m = hypot(iq[2 * n], iq[2 * n + 1]);
        *peak = m > *peak ? m : *peak;
        *mean += m / (samples - samples / 4);
    }
}

int main(void)
{
    static int16_t in[2 * PACKET * PACKETS], out[2 * PACKET * PACKETS], again[2 * PACKET * PACKETS];
    static struct bs_decimator d;
    /* The stage behind the chip's channel filter: flat to 0.2 of the input rate,
     * 51 dB down from 0.3, the same edges as the last stage at half the taps. */
    after_channel = true;
    {
        double peak, mean;
        tone(in, PACKET * PACKETS, 0.19, 2000);
        size_t n = run(&d, 1, in, PACKET * PACKETS, PACKET, out);
        CHECK(n == (size_t)PACKET * PACKETS / 2);
        magnitude(out, n, &peak, &mean);
        CHECK(fabs(mean - 4 * 2000) < 4 * 2000 * 0.01);
        tone(in, PACKET * PACKETS, 0.31, 8000);
        n = run(&d, 1, in, PACKET * PACKETS, PACKET, out);
        magnitude(out, n, &peak, &mean);
        CHECK(peak <= 4 * 8000 * 0.0028); /* -51 dB */
        tone(in, PACKET * PACKETS, -0.33, 8000);
        n = run(&d, 1, in, PACKET * PACKETS, PACKET, out);
        magnitude(out, n, &peak, &mean);
        CHECK(peak <= 4 * 8000 * 0.0028);
    }
    after_channel = false;
    for (unsigned stages = 1; stages <= BS_DECIMATION_MAX; ++stages) {
        const double rate = 1.0 / (1u << stages); /* output rate, as a fraction of the input */
        double peak, mean;
        /* In the passband: the amplitude, four times larger for the two extra bits. */
        tone(in, PACKET * PACKETS, 0.3 * rate, 2000);
        size_t n = run(&d, stages, in, PACKET * PACKETS, PACKET, out);
        CHECK(n == (size_t)PACKET * PACKETS >> stages);
        magnitude(out, n, &peak, &mean);
        CHECK(fabs(mean - 4 * 2000) < 4 * 2000 * 0.001);
        /* Just past the passband edge of the last stage: rejected below the rounding. */
        tone(in, PACKET * PACKETS, 0.65 * rate, 8000);
        n = run(&d, stages, in, PACKET * PACKETS, PACKET, out);
        magnitude(out, n, &peak, &mean);
        CHECK(peak <= 2.0);
        /* Where the first stage would fold a tone into the final passband: the
         * earlier, shorter stages still reject it below the rounding. */
        tone(in, PACKET * PACKETS, 0.5 - 0.3 * rate, 8000);
        n = run(&d, stages, in, PACKET * PACKETS, PACKET, out);
        magnitude(out, n, &peak, &mean);
        CHECK(peak <= 2.0);
        /* Any packet split gives exactly the same samples. */
        tone(in, PACKET * PACKETS, 0.13 * rate, 6000);
        n = run(&d, stages, in, PACKET * PACKETS, PACKET, out);
        CHECK(run(&d, stages, in, PACKET * PACKETS, 1u << stages, again) == n);
        CHECK(!memcmp(out, again, 4 * n));
        /* A reset forgets the history: silence in, silence out. */
        memset(in, 0, sizeof(in));
        n = run(&d, stages, in, PACKET, PACKET, out);
        for (size_t k = 0; k < 2 * n; ++k)
            CHECK(out[k] == 0);
    }
    puts("Decimator tests passed");
    return 0;
}
