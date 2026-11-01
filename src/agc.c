/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */

/*
 * Host automatic gain
 *
 * In automatic mode with host AGC on, the codes stay forced and this loop moves
 * them along the band's measured table, the way receiver AGCs do: a fast attack,
 * a hold, then a slow decay. Every tick it compares the RMS of recent samples
 * with a window. Above it, or on a sample near full scale, the gain drops
 * BS_AGC_STEP at once. A clip also holds the gain at or below the total it
 * dropped to for BS_AGC_HOLD_TICKS, counted again from each clip, so a bursty
 * signal whose mean dips between bursts does not pull it straight back into
 * clipping. Below the window it climbs BS_AGC_RISE every BS_AGC_RISE_TICKS; more
 * than 12 dB below it BS_AGC_STEP every tick, so a read that starts quiet or a
 * signal that went away gets its gain back quickly, but not within
 * BS_AGC_LOCK_TICKS of a clip, so the gaps between bursts do not set it off. The loop keeps its own
 * target total, since a small step may pick the same codes,
 * and fills it LNA first.
 *
 * Delivery measures only the head of each buffer, with atomics, so the sample
 * path takes no lock; the loop applies changes through the same locked path as
 * the public gain calls, and skips the samples that arrive while a change
 * settles.
 */
#include "internal.h"
#include <time.h>

#define BS_AGC_TICK_MS 20
#define BS_AGC_SETTLE_MS 10
#define BS_AGC_HEAD 4096u

static void sleep_ms(long ms)
{
    struct timespec t = {ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&t, NULL);
}

/* The next target total for a tick's mean square per I or Q sample and peak,
 * given the total in force. Pure apart from its state, for the tests.
 */
int bs_agc_next(struct bs_agc_state *state, int total, int top, uint64_t mean_square, int peak)
{
    if (state->target < 0)
        state->target = total;
    if (peak >= BS_AGC_CLIP) {
        state->target = total - BS_AGC_STEP;
        state->ceiling = state->target;
        state->hold = BS_AGC_HOLD_TICKS;
        state->lock = BS_AGC_LOCK_TICKS;
        state->wait = 0;
    } else {
        if (mean_square > (uint64_t)BS_AGC_HIGH * BS_AGC_HIGH) {
            state->target = (state->target < total ? state->target : total) - BS_AGC_STEP;
        } else if (mean_square < (uint64_t)BS_AGC_LOW * BS_AGC_LOW / 16 && !state->lock) {
            state->target += BS_AGC_STEP;
            state->wait = 0;
        } else if (mean_square < (uint64_t)BS_AGC_LOW * BS_AGC_LOW) {
            if (state->wait) {
                --state->wait;
            } else {
                state->target += BS_AGC_RISE;
                state->wait = BS_AGC_RISE_TICKS - 1;
            }
            if (state->hold && state->target > state->ceiling)
                state->target = state->ceiling;
        }
        if (state->hold)
            --state->hold;
        if (state->lock)
            --state->lock;
    }
    if (state->target < 0)
        state->target = 0;
    if (state->target > top)
        state->target = top;
    return state->target;
}

/* Called by delivery for every buffer it hands over. */
void bs_agc_measure(birriasdr_dev_t *dev, const int16_t *iq, uint32_t count)
{
    uint64_t power = 0;
    int peak = 0;
    if (!dev->agc.running)
        return;
    if (count > BS_AGC_HEAD)
        count = BS_AGC_HEAD;
    for (uint32_t n = 0; n < 2 * count; ++n) {
        int v = iq[n] < 0 ? -iq[n] : iq[n];
        power += (uint64_t)(v * v);
        if (v > peak)
            peak = v;
    }
    atomic_fetch_add(&dev->agc.power, power);
    atomic_fetch_add(&dev->agc.count, count);
    for (int seen = atomic_load(&dev->agc.peak);
         peak > seen && !atomic_compare_exchange_weak(&dev->agc.peak, &seen, peak);)
        ;
}

static void reset_measurement(birriasdr_dev_t *dev)
{
    atomic_store(&dev->agc.power, 0);
    atomic_store(&dev->agc.count, 0);
    atomic_store(&dev->agc.peak, 0);
}

/* One decision with the control lock held; true when the codes changed. */
static bool agc_tick(birriasdr_dev_t *dev, uint64_t power, uint64_t count, int peak)
{
    const struct birriasdr_gain_table *table;
    struct birriasdr_gain next;
    int total, top;
    if (!dev->control_ready || atomic_load(&dev->run_state) != BS_RUNNING || dev->manual ||
        !dev->host_agc || !(table = bs_band_gain(dev)))
        return false;
    total = bs_total_gain(table, &dev->gain);
    top = table->lna[table->lna_count - 1] + table->lpf[table->lpf_count - 1] +
          table->digital[table->digital_count - 1];
    next = bs_pick_codes(table, bs_agc_next(&dev->agc.state, total, top, power / (2 * count), peak));
    if (next.lna == dev->gain.lna && next.lpf == dev->gain.lpf && next.digital == dev->gain.digital)
        return false;
    return bs_set_gain_locked(dev, false, &next) == 0;
}

static void *agc_loop(void *arg)
{
    birriasdr_dev_t *dev = arg;
    while (!atomic_load(&dev->agc.stop)) {
        uint64_t power, count;
        int peak;
        bool changed;
        sleep_ms(BS_AGC_TICK_MS);
        power = atomic_exchange(&dev->agc.power, 0);
        count = atomic_exchange(&dev->agc.count, 0);
        peak = atomic_exchange(&dev->agc.peak, 0);
        if (!count)
            continue;
        pthread_mutex_lock(&dev->control_mutex);
        changed = agc_tick(dev, power, count, peak);
        pthread_mutex_unlock(&dev->control_mutex);
        if (changed) {
            sleep_ms(BS_AGC_SETTLE_MS);
            reset_measurement(dev);
        }
    }
    return NULL;
}

/* Run the loop for one read; it idles while the mode leaves gain to others. */
void bs_agc_start(birriasdr_dev_t *dev)
{
    reset_measurement(dev);
    dev->agc.state = (struct bs_agc_state){-1, 0, 0, 0, 0};
    atomic_store(&dev->agc.stop, false);
    dev->agc.running = pthread_create(&dev->agc.thread, NULL, agc_loop, dev) == 0;
}

void bs_agc_stop(birriasdr_dev_t *dev)
{
    if (!dev->agc.running)
        return;
    atomic_store(&dev->agc.stop, true);
    pthread_join(dev->agc.thread, NULL);
    dev->agc.running = false;
}
