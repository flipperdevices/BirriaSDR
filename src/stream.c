/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "usb_stream.h"
#include <stdlib.h>
#include <string.h>

/* macOS can delay USB completions long enough to drain 64 queued transfers. */
#ifdef __APPLE__
#define BS_DEFAULT_DEPTH 256u
#else
#define BS_DEFAULT_DEPTH 64u
#endif

struct delivery {
    birriasdr_dev_t *dev;
    birriasdr_read_async_cb_t callback;
    birriasdr_read_multi_cb_t multi; /* two-chain reads: chain 0 in iq, chain 1 in iq1 */
    void *context;
    int16_t *iq, *iq1;
    uint32_t capacity, used;
    uint32_t restarts;
    uint64_t delivered, segment_start;
    int staged_result;
    bool holdback, staged;
    /* With half-band stages (below 10 MSps, and 20 MSps in the two-chain format):
     * payloads decode into native, a chain each, and are halved into iq once
     * confirmed. The AGC measures them before that, so it still sees signals the
     * filters remove, which can clip the ADC all the same. */
    struct bs_decimator *decimator, *decimator1;
    int16_t *native, *native1;
    unsigned packets; /* packets halved; the AGC measures one in four, as at 10 MSps */
    bool probing;
    /* The channel-filtered path with the 20 MHz filter tunes high by its offset;
     * turn the native samples back by offset/rate cycles a sample: a whole step of
     * a 32-point table (1 at 40 MSps), its phase kept across packets.
     */
    unsigned turn_step, turn_phase;
};

/* 32767 cos and sin of 2 pi k / 32. */
static const int16_t turn_cos[32] = {
    32767, 32137, 30273, 27245, 23170, 18204, 12539, 6393,
    0, -6393, -12539, -18204, -23170, -27245, -30273, -32137,
    -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393,
    0, 6393, 12539, 18204, 23170, 27245, 30273, 32137
};
static const int16_t turn_sin[32] = {
    0, 6393, 12539, 18204, 23170, 27245, 30273, 32137,
    32767, 32137, 30273, 27245, 23170, 18204, 12539, 6393,
    0, -6393, -12539, -18204, -23170, -27245, -30273, -32137,
    -32767, -32137, -30273, -27245, -23170, -18204, -12539, -6393
};

static void setup_turn(struct delivery *delivery)
{
    const birriasdr_dev_t *dev = delivery->dev;
    const uint32_t offset = bs_filter_offset(dev);
    delivery->turn_step = offset ? (unsigned)((uint64_t)offset * 32 / bs_native_rate(dev)) : 0;
    delivery->turn_phase = 0;
}

/* Multiply by exp(+j 2 pi offset n / rate): the signal at the requested center,
 * which arrives offset below the tuned one, returns to 0 Hz. Magnitude is kept,
 * so I or Q alone can exceed full scale by up to the square root of 2. The phase
 * pattern repeats every 32 samples, so the table is laid out once per call from
 * the current phase and each block of 32 runs over contiguous values, which the
 * compiler vectorizes; packets are whole blocks.
 */
static void turn_samples(struct delivery *delivery, int16_t *iq, uint32_t samples)
{
    int32_t c[32], s[32];
    for (unsigned k = 0; k < 32; ++k) {
        const unsigned at = (delivery->turn_phase + k * delivery->turn_step) & 31;
        c[k] = turn_cos[at];
        s[k] = turn_sin[at];
    }
    for (uint32_t n = 0; n < samples; n += 32) {
        int16_t *x = iq + 2 * n;
        const uint32_t block = samples - n < 32 ? samples - n : 32;
        for (uint32_t k = 0; k < block; ++k) {
            const int32_t i = x[2 * k], q = x[2 * k + 1];
            x[2 * k] = (int16_t)((i * c[k] - q * s[k] + 16384) >> 15);
            x[2 * k + 1] = (int16_t)((i * s[k] + q * c[k] + 16384) >> 15);
        }
    }
    delivery->turn_phase = (delivery->turn_phase + samples * delivery->turn_step) & 31;
}

static void reset_decimators(struct delivery *delivery)
{
    const unsigned stages = bs_host_halvings(delivery->dev);
    const bool after = bs_filtered(delivery->dev); /* the chip's channel filter came first */
    if (delivery->decimator)
        bs_decimator_reset(delivery->decimator, stages, after);
    if (delivery->decimator1)
        bs_decimator_reset(delivery->decimator1, stages, after);
}

/* Deliver only complete validated samples. A short final buffer is allowed;
 * sequence offsets are complex-sample counts, independent of callback sizing.
 */
static void flush_samples(struct delivery *delivery)
{
    if (!delivery->used)
        return;
    if (delivery->multi) {
        const int16_t *chains[2] = {delivery->iq, delivery->iq1};
        if (!delivery->decimator) {
            bs_agc_measure(delivery->dev, delivery->iq, delivery->used);
            bs_agc_measure(delivery->dev, delivery->iq1, delivery->used);
        }
        delivery->multi(chains, 2, delivery->used, delivery->delivered, delivery->context);
    } else {
        /* The automatic antenna's first comparison withholds the read's first samples. */
        bool probing = delivery->decimator ? delivery->probing :
                       bs_agc_measure(delivery->dev, delivery->iq, delivery->used);
        if (probing && !delivery->delivered) {
            delivery->used = 0;
            return;
        }
        delivery->callback(delivery->iq, delivery->used, delivery->delivered, delivery->context);
    }
    delivery->delivered += delivery->used;
    delivery->dev->stats.samples = delivery->delivered - delivery->segment_start;
    delivery->used = 0;
}

/* Reorder one complete raw Q,I payload into public I,Q pairs just past the
 * delivered ones (or into native, before half-band stages), without counting
 * them. A packet holds 2048 hardware samples, 1024 with the packer at 10 and
 * 20 MSps and 1024 a chain in the two-chain format; callback capacity is a
 * whole number of 2048-sample packets, so every case divides it evenly.
 */
static int decode_payload(struct delivery *delivery, const uint8_t *raw)
{
    birriasdr_dev_t *dev = delivery->dev;
    const uint32_t samples = bs_packet_samples(dev);
    int16_t *iq = delivery->decimator ? delivery->native : delivery->iq + 2u * delivery->used;
    int result;
    if (delivery->multi) {
        int16_t *iq1 = delivery->decimator1 ? delivery->native1 : delivery->iq1 + 2u * delivery->used;
        const unsigned phase = delivery->turn_phase;
        result = bs_decode_iq_dual(raw, iq, iq1, BS_PACKET_SAMPLES);
        if (!result && delivery->turn_step) {
            turn_samples(delivery, iq, samples);
            delivery->turn_phase = phase; /* both chains turn by the same phases */
            turn_samples(delivery, iq1, samples);
        }
        return result;
    }
    if (bs_lanes(dev))
        result = bs_decode_iq_lane(raw, iq, BS_PACKET_SAMPLES, atomic_load(&dev->lane));
    else if (samples < BS_PACKET_SAMPLES)
        result = bs_decode_iq_halved(raw, iq, BS_PACKET_SAMPLES);
    else
        result = bs_decode_iq(raw, iq, BS_PACKET_SAMPLES);
    if (!result && delivery->turn_step)
        turn_samples(delivery, iq, samples);
    return result;
}

/* Count the decoded payload, or measure and halve it, and deliver once the
 * buffer is full. */
static void commit_payload(struct delivery *delivery)
{
    const uint32_t samples = bs_packet_samples(delivery->dev);
    if (delivery->decimator) {
        if (delivery->packets++ % 4 == 0) {
            delivery->probing = bs_agc_measure(delivery->dev, delivery->native, samples);
            if (delivery->multi)
                bs_agc_measure(delivery->dev, delivery->native1, samples);
        }
        const uint32_t out = (uint32_t)bs_decimate(delivery->decimator, delivery->native, samples,
                                                   delivery->iq + 2u * delivery->used);
        if (delivery->multi)
            bs_decimate(delivery->decimator1, delivery->native1, samples,
                        delivery->iq1 + 2u * delivery->used);
        delivery->used += out;
    } else {
        delivery->used += samples;
    }
    if (delivery->used == delivery->capacity)
        flush_samples(delivery);
}

/* Count a staged payload now that it is confirmed, or fail with its decode error. */
static int commit_staged(struct delivery *delivery)
{
    if (!delivery->staged)
        return 0;
    delivery->staged = false;
    BS_TRY(delivery->staged_result);
    commit_payload(delivery);
    return 0;
}

/* With local recovery enabled, the next data header confirms that the prior
 * DMA packet survived the source checks. A gap marker instead invalidates it.
 * Stage it decoded but uncounted until then, and keep its decode verdict, so
 * even overwritten sample bits cannot leak into callbacks or cause a spurious
 * decoder failure before the marker arrives.
 */
static int receive_payload(void *context, const uint8_t *raw)
{
    struct delivery *delivery = context;
    if (!delivery->holdback) {
        BS_TRY(decode_payload(delivery, raw));
        commit_payload(delivery);
        return 0;
    }
    BS_TRY(commit_staged(delivery));
    delivery->staged_result = decode_payload(delivery, raw);
    delivery->staged = true;
    return 0;
}

/* One notification counter spans local gaps and full acquisition restarts.
 * Saturate at the largest public counter value rather than wrapping to zero.
 */
static uint32_t next_attempt(struct delivery *delivery)
{
    if (delivery->restarts != UINT32_MAX)
        ++delivery->restarts;
    return delivery->restarts;
}

/* The ordered device marker discards the single possibly overwritten packet.
 * Deliver the preceding validated prefix before announcing the gap. Receiver
 * settings, the USB reader and the boot nonce remain live, so no host reset or
 * artificial backoff is needed. Missing sample counts are still unknown.
 */
static int receive_gap(void *context, uint32_t state, uint64_t packets)
{
    struct delivery *delivery = context;
    birriasdr_dev_t *dev = delivery->dev;
    delivery->staged = false;
    flush_samples(delivery);
    reset_decimators(delivery); /* the samples on either side are not continuous */
    struct birriasdr_recovery_event event = {
        .next_sample = delivery->delivered, .attempt = next_attempt(delivery),
        .error = BIRRIASDR_ERROR_OVERRUN, .previous = dev->stats
    };
    event.previous.firmware_state = state;
    event.previous.packets = packets;
    dev->recovery_callback(&event, dev->recovery_context);
    return 0;
}

/* Validate terminal firmware state before waiting for its advertised packets.
 * A fault can strand submitted descriptors that will never arrive over USB.
 * Preserve that known fault instead of waiting for them and reporting timeout.
 */
static int validate_terminal_status(birriasdr_dev_t *dev, const uint32_t words[16], uint32_t limit)
{
    dev->stats.firmware_state = words[7];
    dev->stats.stage = BIRRIASDR_STAGE_VERIFY;
    dev->stats.dma_errors = words[13];
    if (words[7] == 8 || words[7] == 9)
        return BIRRIASDR_ERROR_OVERRUN;
    if (words[7] == 4 || words[7] == 5 || words[7] == 10)
        return BIRRIASDR_ERROR_TIMEOUT;
    if (words[7] != 2 && words[7] != 3)
        return BIRRIASDR_ERROR_STATE;
    if (words[13] || words[10] != words[11])
        return BIRRIASDR_ERROR_PROTOCOL;
    if (words[7] == 2 && (!limit || words[10] != limit))
        return BIRRIASDR_ERROR_PROTOCOL;
    unsigned state = atomic_load(&dev->run_state);
    if (words[7] == 3 && state != BS_CANCELLING && state != BS_CONTROL_FAILED)
        return BIRRIASDR_ERROR_STATE;
    return 0;
}

/* Serialize complete control procedures with live setters. The USB event pump
 * does not acquire this lock, so sample transfers continue during RF updates.
 * Closing the live gate before STOP prevents a late update on a parked MCU.
 */
static int stop_capture(birriasdr_dev_t *dev)
{
    int result;
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    result = dev->backend->stop(dev);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* A terminal status also closes the live-control gate. A finite capture can have
 * already ended while buffered samples are still being delivered to the host.
 */
static int capture_status(birriasdr_dev_t *dev, uint32_t words[16])
{
    int result;
    pthread_mutex_lock(&dev->control_mutex);
    result = dev->backend->status(dev, words);
    if (result < 0 || words[7] != 1)
        dev->control_ready = false;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Publish live readiness only after initialization, queued reads and GO. An
 * application control thread cannot interleave RF operations with startup.
 */
static int prepare_capture(birriasdr_dev_t *dev, struct bs_reader *reader, uint32_t packets)
{
    int result;
    pthread_mutex_lock(&dev->control_mutex);
    result = dev->backend->prepare(dev, packets);
    if (!result)
        result = bs_reader_start(reader);
    if (!result)
        result = dev->backend->start(dev);
    dev->control_ready = result == 0 && atomic_load(&dev->run_state) == BS_RUNNING;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* The bytes of a completion that belong to events. An aggregate whose length is
 * a whole number of 1,024-byte SuperSpeed packets gains four zero bytes, so the
 * transfer still ends with a short packet. Events are 40, 128 or 8,320 bytes, so
 * no completion of events alone is four bytes past such a multiple.
 */
static int event_bytes(const uint8_t *bytes, size_t *length)
{
    if (*length % 1024 != 4)
        return 0;
    if (bs_get_le32(bytes + *length - 4))
        return BIRRIASDR_ERROR_PROTOCOL;
    *length -= 4;
    return 0;
}

/* Consume independently of the USB event pump. EP0 status reads are deferred
 * until a finite run finishes, cancellation is requested, or progress pauses;
 * hammering EP0 during capture would itself compete with the data endpoint.
 */
static int receive_loop(birriasdr_dev_t *dev, struct bs_reader *reader,
                        struct bs_framer *framer, uint32_t limit)
{
    uint64_t last_progress = bs_now_ns(), next_poll = 0, expected_packets = 0;
    uint32_t words[16] = {0};
    bool stop_sent = false, terminal = false;
    for (;;) {
        const uint8_t *bytes;
        size_t length;
        uint64_t now;
        int result;
        unsigned state = atomic_load(&dev->run_state);
        if ((state == BS_CANCELLING || state == BS_CONTROL_FAILED) && !stop_sent) {
            BS_TRY(stop_capture(dev));
            stop_sent = true;
        }
        result = bs_reader_take(reader, &bytes, &length, 20);
        if (result != BIRRIASDR_ERROR_TIMEOUT) {
            BS_TRY(result);
            dev->stats.usb_bytes += length;
            BS_TRY(event_bytes(bytes, &length));
            BS_TRY(bs_framer_feed(framer, bytes, length));
            dev->stats.packets = framer->packets;
            if (length)
                last_progress = bs_now_ns();
        }
        now = bs_now_ns();
        if (!terminal && now >= next_poll && (stop_sent ||
            (limit && framer->packets >= limit) || now - last_progress > 250000000)) {
            BS_TRY(capture_status(dev, words));
            terminal = words[7] != 1;
            if (terminal) {
                uint32_t remaining;
                BS_TRY(validate_terminal_status(dev, words, limit));
                /* Wire counters wrap at 32 bits. The bounded transport cannot
                 * retain half that many packets; a backwards count is invalid. */
                remaining = words[10] - (uint32_t)framer->packets;
                if (remaining > UINT32_MAX / 2)
                    return BIRRIASDR_ERROR_PROTOCOL;
                expected_packets = framer->packets + remaining;
            }
            next_poll = now + 50000000;
        }
        if (terminal && framer->packets >= expected_packets) {
            /* Drain already queued/trailing events: duplicates must fail too. */
            do {
                result = bs_reader_take(reader, &bytes, &length, 20);
                if (result == BIRRIASDR_ERROR_TIMEOUT)
                    break;
                BS_TRY(result);
                dev->stats.usb_bytes += length;
                BS_TRY(event_bytes(bytes, &length));
                BS_TRY(bs_framer_feed(framer, bytes, length));
            } while (length);
            dev->stats.packets = framer->packets;
            return bs_framer_finish(framer, expected_packets);
        }
        if (now - last_progress > UINT64_C(3000000000))
            return BIRRIASDR_ERROR_TIMEOUT;
    }
}

/* Decide whether another complete acquisition can plausibly recover. A failed
 * control transaction may return STATE after rollback, while corrupted packets,
 * missing devices and retained vendor patches require application intervention.
 */
static bool recoverable(int error, bool control_error)
{
    return error == BIRRIASDR_ERROR_OVERRUN || error == BIRRIASDR_ERROR_IO ||
           error == BIRRIASDR_ERROR_TIMEOUT ||
           (control_error && error == BIRRIASDR_ERROR_STATE);
}

/* An overflow loses sample continuity, not necessarily the device connection.
 * Only exempt it from the fault budget after successful teardown with no DMA
 * or live-control failure. A failed reset must still spend a bounded retry.
 */
static bool clean_overrun(const birriasdr_dev_t *dev, int error)
{
    return error == BIRRIASDR_ERROR_OVERRUN && !dev->stats.reset_error &&
           !dev->stats.dma_errors && !dev->control_error && !dev->usb_lost;
}

/* Execute one acquisition without releasing ownership of the public handle.
 * Destroy the reader (including cancellation callbacks) BEFORE reset. Keep the
 * first live-control fault as the primary error if STOP subsequently fails, but
 * never mask device removal or framing corruption. The outer loop decides
 * whether to report a discontinuity and try the committed settings again.
 */
static int capture_once(struct delivery *delivery, uint32_t depth, uint32_t packets,
                        bool *can_retry)
{
    birriasdr_dev_t *dev = delivery->dev;
    struct bs_reader *reader = NULL;
    struct bs_framer framer = {0};
    int result = 0;
    *can_retry = false;
    memset(&dev->stats, 0, sizeof(dev->stats));
    delivery->segment_start = delivery->delivered;
    delivery->holdback = !packets && dev->recovery_limit && dev->recovery_callback;
    delivery->staged = false;
    reset_decimators(delivery);
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    dev->control_error = 0;
    dev->control_error_stage = BIRRIASDR_STAGE_IDLE;
    pthread_mutex_unlock(&dev->control_mutex);
    if (dev->usb_lost) {
        result = BIRRIASDR_ERROR_NO_DEVICE;
        goto cleanup;
    }
    if (atomic_load(&dev->run_state) == BS_CANCELLING)
        goto cleanup;
    result = bs_reader_create(dev, depth, &reader);
    if (result < 0)
        goto cleanup;
    result = prepare_capture(dev, reader, packets);
    if (result < 0)
        goto cleanup;
    framer.nonce = dev->nonce;
    framer.payload = receive_payload;
    framer.gap = delivery->holdback ? receive_gap : NULL;
    framer.context = delivery;
    dev->stats.stage = BIRRIASDR_STAGE_STREAM;
    result = receive_loop(dev, reader, &framer, packets);
    dev->stats.packets = framer.packets;
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    pthread_mutex_unlock(&dev->control_mutex);
    /* Only a verified clean terminal state releases the final held packet. */
    if (!result)
        result = commit_staged(delivery);
    delivery->staged = false;
    flush_samples(delivery);
cleanup:
    pthread_mutex_lock(&dev->control_mutex);
    dev->control_ready = false;
    if (dev->control_error && (!result || result == BIRRIASDR_ERROR_IO ||
        result == BIRRIASDR_ERROR_TIMEOUT || result == BIRRIASDR_ERROR_STATE)) {
        result = dev->control_error;
        dev->stats.stage = dev->control_error_stage;
    }
    pthread_mutex_unlock(&dev->control_mutex);
    bs_reader_destroy(reader);
    pthread_mutex_lock(&dev->control_mutex);
    if (result == BIRRIASDR_ERROR_NO_DEVICE)
        dev->usb_lost = true;
    if (dev->usb_lost)
        dev->touched = false;
    if (dev->touched) {
        dev->stats.reset_error = dev->backend->reset(dev);
        dev->touched = dev->stats.reset_error != 0;
        if (dev->stats.reset_error == BIRRIASDR_ERROR_NO_DEVICE) {
            dev->usb_lost = true;
            dev->touched = false;
        }
        if (!result)
            result = dev->stats.reset_error;
    }
    *can_retry = recoverable(result, dev->control_error != 0) &&
        (!dev->stats.reset_error || recoverable(dev->stats.reset_error, true));
    /* A drained source overflow needs a new acquisition, but a successful
     * subsystem reset need not reset USB enumeration as well. Startup still
     * escalates to USB recovery if the ordinary prepare fails. Host queue
     * overflows retain transport recovery because queued transfers were lost.
     */
    bool drained_overrun = clean_overrun(dev, result) &&
        (dev->stats.firmware_state == 8 || dev->stats.firmware_state == 9);
    if ((*can_retry && !drained_overrun) || result == BIRRIASDR_ERROR_PROTOCOL)
        dev->needs_usb_reset = true;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Publish a gap after teardown and before any new sample callback. Compare/exchange
 * changes only our internal control-failure STOP back to RUNNING; an application
 * cancellation is sticky across every recovery phase and cannot be cleared here.
 */
static bool announce_restart(struct delivery *delivery, uint32_t attempt, int error)
{
    birriasdr_dev_t *dev = delivery->dev;
    struct birriasdr_recovery_event event = {
        .next_sample = delivery->delivered, .attempt = attempt,
        .error = error, .previous = dev->stats
    };
    unsigned state = BS_CONTROL_FAILED;
    atomic_compare_exchange_strong(&dev->run_state, &state, BS_RUNNING);
    if (atomic_load(&dev->run_state) == BS_CANCELLING)
        return false;
    dev->recovery_callback(&event, dev->recovery_context);
    /* Avoid spinning on a persistent transport failure; cancellation can still
     * prevent startup immediately after this short, bounded delay. */
    if (atomic_load(&dev->run_state) != BS_CANCELLING)
        bs_sleep_us(50000);
    return atomic_load(&dev->run_state) != BS_CANCELLING;
}

/* One owner spans local source gaps and full acquisition restarts. Legacy
 * callers remain strict, and finite reads never join discontinuous segments.
 * Device markers preserve firmware, nonce and packet counters. Faults that
 * require full recovery instead boot with a fresh nonce and committed settings.
 * Delivered offsets remain cumulative across both kinds of discontinuity.
 */
static int stream_pinned(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
                         birriasdr_read_multi_cb_t multi, void *context, uint32_t depth,
                         uint32_t buffer_samples, uint32_t packets)
{
    struct delivery delivery = {0};
    uint64_t total_packets = 0, total_bytes = 0;
    uint32_t faults = 0;
    unsigned idle = BS_IDLE;
    int result = BIRRIASDR_ERROR_NO_MEMORY;
    if (!dev || !callback == !multi || !multi != !dev->dual)
        return BIRRIASDR_ERROR_INVALID;
    if (!depth) depth = BS_DEFAULT_DEPTH;
    if (!buffer_samples) buffer_samples = 16384;
    if (depth < 4 || depth > 256 || buffer_samples > 262144 || buffer_samples % BS_PACKET_SAMPLES)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    if (!atomic_compare_exchange_strong(&dev->run_state, &idle, BS_RUNNING)) {
        pthread_mutex_unlock(&dev->control_mutex);
        return BIRRIASDR_ERROR_BUSY;
    }
    dev->control_ready = false;
    dev->stream_continuous = packets == 0;
    pthread_mutex_unlock(&dev->control_mutex);
    memset(&dev->stats, 0, sizeof(dev->stats));
    delivery = (struct delivery){.dev = dev, .callback = callback, .multi = multi,
                                .context = context, .capacity = buffer_samples};
    setup_turn(&delivery);
    delivery.iq = calloc((size_t)buffer_samples * 2, sizeof(*delivery.iq));
    bool buffers = delivery.iq != NULL;
    if (multi) {
        delivery.iq1 = calloc((size_t)buffer_samples * 2, sizeof(*delivery.iq1));
        buffers = buffers && delivery.iq1;
    }
    if (bs_host_halvings(dev)) {
        delivery.decimator = malloc(sizeof(*delivery.decimator));
        delivery.native = malloc(2 * BS_PACKET_SAMPLES * sizeof(*delivery.native));
        buffers = buffers && delivery.decimator && delivery.native;
        if (multi) {
            delivery.decimator1 = malloc(sizeof(*delivery.decimator1));
            delivery.native1 = malloc(2 * BS_PACKET_SAMPLES * sizeof(*delivery.native1));
            buffers = buffers && delivery.decimator1 && delivery.native1;
        }
    }
    bs_agc_start(dev);
    if (buffers) {
        for (;;) {
            bool can_retry;
            result = capture_once(&delivery, depth, packets, &can_retry);
            total_packets += dev->stats.packets;
            total_bytes += dev->stats.usb_bytes;
            bool overflow = clean_overrun(dev, result);
            if (!can_retry || packets || !dev->recovery_callback ||
                !dev->recovery_limit || (!overflow && faults == dev->recovery_limit))
                break;
            if (!overflow)
                ++faults;
            if (!announce_restart(&delivery, next_attempt(&delivery), result))
                break;
        }
    }
    /* In recovering mode an application may stop at an overflow boundary.
     * No further samples are joined, and the retained statistics still show
     * that last gap. Preserve real transport/control failures and strict reads.
     */
    if (!packets && dev->recovery_limit && dev->recovery_callback &&
        atomic_load(&dev->run_state) == BS_CANCELLING && clean_overrun(dev, result))
        result = 0;
    bs_agc_stop(dev);
    dev->stats.samples = delivery.delivered;
    dev->stats.packets = total_packets;
    dev->stats.usb_bytes = total_bytes;
    free(delivery.iq);
    free(delivery.iq1);
    free(delivery.decimator);
    free(delivery.native);
    free(delivery.decimator1);
    free(delivery.native1);
    pthread_mutex_lock(&dev->control_mutex);
    dev->stream_continuous = false;
    atomic_store(&dev->run_state, BS_IDLE);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* The caller's thread runs the stream, so it is pinned for the stream only. */
int bs_stream(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
               birriasdr_read_multi_cb_t multi, void *context, uint32_t depth,
               uint32_t buffer_samples, uint32_t packets)
{
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    void *saved = bs_pin_caller(dev);
    const int result = stream_pinned(dev, callback, multi, context, depth, buffer_samples, packets);
    bs_restore_caller(saved);
    return result;
}
