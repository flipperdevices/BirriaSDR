/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "internal.h"
#include <stdlib.h>
#include <string.h>

/* Controls without a live implementation remain idle-only. Gain and tuning
 * have serialized paths; cancellation uses the atomic acquisition state.
 */
static int require_idle(birriasdr_dev_t *dev)
{
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    return atomic_load(&dev->run_state) != BS_IDLE ? BIRRIASDR_ERROR_BUSY : 0;
}

/* Keep the ABI version independent of individual firmware image revisions. */
const char *birriasdr_get_version(void)
{
    return "0.1.0";
}

/* Errors are library-owned values, not ambiguous errno/libusb mixtures. */
const char *birriasdr_strerror(int error)
{
    switch (error) {
    case 0: return "success";
    case BIRRIASDR_ERROR_INVALID: return "invalid argument";
    case BIRRIASDR_ERROR_NO_DEVICE: return "device absent or disconnected";
    case BIRRIASDR_ERROR_ACCESS: return "USB or device-lock permission denied";
    case BIRRIASDR_ERROR_BUSY: return "device, driver, or stream is busy";
    case BIRRIASDR_ERROR_IO: return "USB/register I/O failed";
    case BIRRIASDR_ERROR_TIMEOUT: return "hardware handshake timed out";
    case BIRRIASDR_ERROR_NO_MEMORY: return "allocation failed";
    case BIRRIASDR_ERROR_UNSUPPORTED: return "chipset, board, or control not implemented";
    case BIRRIASDR_ERROR_COLD_BOOT: return "retained vendor patch: cold power-on required";
    case BIRRIASDR_ERROR_PROTOCOL: return "invalid, missing, or stale stream data";
    case BIRRIASDR_ERROR_OVERRUN: return "capture continuity lost to device or host queue overrun";
    case BIRRIASDR_ERROR_STATE: return "unexpected hardware or receiver state";
    default: return "unknown library error";
    }
}

/* Discovery uses descriptors only, so unsupported backends remain visible. */
int birriasdr_get_device_count(void)
{
    return bs_usb_discover(0, NULL, NULL);
}

/* Copy descriptive information; do not hold libusb descriptor pointers. */
int birriasdr_get_device_info(uint32_t index, struct birriasdr_device_info *info)
{
    if (!info)
        return BIRRIASDR_ERROR_INVALID;
    memset(info, 0, sizeof(*info));
    return bs_usb_discover(index, info, NULL);
}

/* Allocate one independent context, then claim a qualified interface. Booting
 * is delayed until read so setters configure the next acquisition atomically.
 */
int birriasdr_open(birriasdr_dev_t **out, uint32_t index)
{
    birriasdr_dev_t *dev;
    int result;
    if (!out)
        return BIRRIASDR_ERROR_INVALID;
    *out = NULL;
    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return BIRRIASDR_ERROR_NO_MEMORY;
    dev->lock_fd = -1;
    dev->interface_number = -1;
    atomic_init(&dev->run_state, BS_IDLE);
    atomic_init(&dev->frequency, 0);
    if (pthread_mutex_init(&dev->control_mutex, NULL)) {
        free(dev);
        return BIRRIASDR_ERROR_NO_MEMORY;
    }
    dev->gain = (struct birriasdr_gain){1, 4, 0};
    result = bs_usb_discover(index, NULL, dev);
    if (result < 0) {
        bs_usb_close(dev);
        pthread_mutex_destroy(&dev->control_mutex);
        free(dev);
        return result;
    }
    dev->host_agc = (dev->backend->capabilities & BIRRIASDR_CAP_AGC) != 0;
    *out = dev;
    return 0;
}

/* A read owns every outstanding transfer. Requiring it to return before close
 * prevents transfer callbacks from dereferencing a freed device/context.
 */
int birriasdr_close(birriasdr_dev_t *dev)
{
    int result = 0;
    BS_TRY(require_idle(dev));
    if (dev->touched && !dev->usb_lost)
        result = dev->backend->reset(dev);
    bs_usb_close(dev);
    pthread_mutex_destroy(&dev->control_mutex);
    free(dev);
    return result;
}

/* Capabilities describe implemented operations, never silicon marketing limits. */
uint32_t birriasdr_get_capabilities(const birriasdr_dev_t *dev)
{
    return dev ? dev->backend->capabilities : 0;
}

/* With the control mutex held, turn a failed hardware update into an orderly
 * stop. Preserve its stage independently of later status/reset procedures.
 */
static void control_failed(birriasdr_dev_t *dev, int error, enum birriasdr_stage stage)
{
    unsigned expected = BS_RUNNING;
    dev->control_ready = false;
    dev->control_error = error;
    dev->control_error_stage = stage;
    /* Keep an application cancellation distinct from our internal STOP. */
    atomic_compare_exchange_strong(&dev->run_state, &expected, BS_CONTROL_FAILED);
}

/* Report backend command limits rather than inferring them from a Wi-Fi channel
 * table, narrowed by any measured board bounds. The bounds describe programming
 * support, not measured sensitivity.
 */
int birriasdr_get_tuning_range(const birriasdr_dev_t *dev, uint64_t *minimum, uint64_t *maximum)
{
    if (!dev || !minimum || !maximum)
        return BIRRIASDR_ERROR_INVALID;
    *minimum = dev->min_frequency ? dev->min_frequency : dev->backend->min_frequency;
    *maximum = dev->max_frequency ? dev->max_frequency : dev->backend->max_frequency;
    return *minimum && *maximum >= *minimum ? 0 : BIRRIASDR_ERROR_UNSUPPORTED;
}

/* Report one band narrowed by board bounds. Callers iterate from index 0 until
 * INVALID; a band the board cannot reach returns UNSUPPORTED and is skipped.
 */
int birriasdr_get_tuning_band(const birriasdr_dev_t *dev, uint32_t index,
                              uint64_t *minimum, uint64_t *maximum)
{
    struct bs_band band;
    if (!dev || !minimum || !maximum)
        return BIRRIASDR_ERROR_INVALID;
    if (dev->backend->bands) {
        if (index >= dev->backend->band_count)
            return BIRRIASDR_ERROR_INVALID;
        band = dev->backend->bands[index];
    } else {
        if (index)
            return BIRRIASDR_ERROR_INVALID;
        band = (struct bs_band){dev->backend->min_frequency, dev->backend->max_frequency, NULL};
    }
    *minimum = dev->min_frequency > band.min_frequency ? dev->min_frequency : band.min_frequency;
    *maximum = dev->max_frequency && dev->max_frequency < band.max_frequency ? dev->max_frequency : band.max_frequency;
    return *minimum && *maximum >= *minimum ? 0 : BIRRIASDR_ERROR_UNSUPPORTED;
}

/* Stage a center while idle, or synchronously retune under the same mutex as
 * gain/start/stop/reset. The backend calibrates its synthesizer; it never rounds
 * to a Wi-Fi channel or silently translates this into host-side mixing.
 */
int birriasdr_set_center_freq(birriasdr_dev_t *dev, uint64_t frequency)
{
    unsigned state;
    int result = 0;
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready)) {
        result = BIRRIASDR_ERROR_BUSY;
        goto done;
    }
    result = dev->backend->validate(frequency, dev->rate, &dev->gain);
    if (result < 0)
        goto done;
    if ((dev->min_frequency && frequency < dev->min_frequency) ||
        (dev->max_frequency && frequency > dev->max_frequency)) {
        result = BIRRIASDR_ERROR_UNSUPPORTED;
        goto done;
    }
    if (state == BS_RUNNING) {
        if (!(dev->backend->capabilities & BIRRIASDR_CAP_LIVE_TUNING) ||
            !dev->backend->change_frequency) {
            result = BIRRIASDR_ERROR_UNSUPPORTED;
            goto done;
        }
        result = dev->backend->change_frequency(dev, frequency);
        if (result < 0) {
            control_failed(dev, result, BIRRIASDR_STAGE_RF);
            goto done;
        }
    }
    atomic_store(&dev->frequency, frequency);
done:
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Read committed metadata atomically, including from sample callbacks while
 * another thread retunes. It does not label the frequency of queued samples.
 */
uint64_t birriasdr_get_center_freq(const birriasdr_dev_t *dev)
{
    return dev ? atomic_load(&dev->frequency) : 0;
}

/* The backend validates the rate and programs the matching capture tap and
 * filter profile at preparation. No host decimator is hidden here.
 */
int birriasdr_set_sample_rate(birriasdr_dev_t *dev, uint32_t rate)
{
    BS_TRY(require_idle(dev));
    BS_TRY(dev->backend->validate(dev->frequency, rate, &dev->gain));
    dev->rate = rate;
    return 0;
}

/* The nominal complex rate is configuration metadata, not measured timing. */
uint32_t birriasdr_get_sample_rate(const birriasdr_dev_t *dev)
{
    return dev ? dev->rate : 0;
}

/* Programmed when a read starts, like the rate. */
int birriasdr_set_usb_burst(birriasdr_dev_t *dev, unsigned burst)
{
    BS_TRY(require_idle(dev));
    if (burst > 15)
        return BIRRIASDR_ERROR_INVALID;
    dev->usb_burst = burst;
    return 0;
}

unsigned birriasdr_get_usb_burst(const birriasdr_dev_t *dev)
{
    const char *env = getenv("BIRRIA_BURST");
    if (!dev)
        return 0;
    return dev->usb_burst ? dev->usb_burst : env ? (unsigned)strtoul(env, NULL, 0) & 0xf : 0;
}

/* Applied when the library's threads start, at the next read. */
int birriasdr_set_cpus(birriasdr_dev_t *dev, const char *list)
{
    BS_TRY(require_idle(dev));
    if (!list) {
        dev->cpus_set = false;
        return 0;
    }
    if (strlen(list) >= sizeof(dev->cpus) || bs_parse_cpus(list, NULL))
        return BIRRIASDR_ERROR_INVALID;
    strcpy(dev->cpus, list);
    dev->cpus_set = true;
    return 0;
}

const char *birriasdr_get_cpus(const birriasdr_dev_t *dev)
{
    const char *env = getenv("BIRRIA_CPUS");
    if (!dev)
        return "";
    return dev->cpus_set ? dev->cpus : env ? env : "";
}

/* The caller holds control_mutex across validation, the complete hardware
 * transaction and the settings commit. Startup/stop/status/reset use the same
 * mutex, so even an RF-SPI read/modify/write cannot straddle a device reset.
 * A failed live update stops this acquisition. Its error reaches either the
 * recovery callback or, if no restart succeeds, the read caller.
 */
int bs_set_gain_locked(birriasdr_dev_t *dev, bool manual, const struct birriasdr_gain *gain)
{
    unsigned state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready))
        return BIRRIASDR_ERROR_BUSY;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_RAW_GAIN))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    BS_TRY(dev->backend->validate(dev->frequency, dev->rate, gain));
    if (state == BS_RUNNING) {
        int result;
        if (!(dev->backend->capabilities & BIRRIASDR_CAP_LIVE_GAIN) ||
            !dev->backend->change_gain)
            return BIRRIASDR_ERROR_UNSUPPORTED;
        result = dev->backend->change_gain(dev, manual || dev->host_agc, gain);
        if (result < 0) {
            control_failed(dev, result, BIRRIASDR_STAGE_GAIN);
            return result;
        }
    }
    if (gain->digital != dev->gain.digital)
        atomic_store(&dev->agc.dc_restart, true);
    dev->manual = manual;
    dev->gain = *gain;
    return 0;
}

/* Stage mode while idle; during capture, synchronously apply the known force
 * fields. Released mode is not a claim about all internal AGC loops. A gain
 * acknowledgment means readback passed, not that every queued sample is newer.
 */
int birriasdr_set_tuner_gain_mode(birriasdr_dev_t *dev, int manual)
{
    int result;
    if (!dev || (manual != 0 && manual != 1))
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    result = bs_set_gain_locked(dev, manual != 0, &dev->gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Reject invalid indices before hardware writes. A live request shares the
 * capture's control lock and retains the original boot baseline for cleanup.
 */
int birriasdr_set_gain_codes(birriasdr_dev_t *dev, const struct birriasdr_gain *gain)
{
    int result;
    if (!dev || !gain)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    if (atomic_load(&dev->run_state) != BS_IDLE &&
        (atomic_load(&dev->run_state) != BS_RUNNING || !dev->control_ready))
        result = BIRRIASDR_ERROR_BUSY;
    else if (!dev->manual)
        result = BIRRIASDR_ERROR_STATE;
    else
        result = bs_set_gain_locked(dev, true, gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Read settings under the same lock used for live commits. This is not an
 * instantaneous AGC measurement, and performs no RF-SPI transaction.
 */
int birriasdr_get_gain_codes(birriasdr_dev_t *dev, struct birriasdr_gain *gain, int *manual)
{
    if (!dev || !gain || !manual)
        return BIRRIASDR_ERROR_INVALID;
    pthread_mutex_lock(&dev->control_mutex);
    *gain = dev->gain;
    *manual = dev->manual;
    pthread_mutex_unlock(&dev->control_mutex);
    return 0;
}

/* Missing research must be observable to callers, not a successful no-op. */
static int unsupported_control(birriasdr_dev_t *dev)
{
    BS_TRY(require_idle(dev));
    return BIRRIASDR_ERROR_UNSUPPORTED;
}

/* The measured table of the band holding the center, or NULL where none exists. */
const struct birriasdr_gain_table *bs_band_gain(const birriasdr_dev_t *dev)
{
    uint64_t frequency = atomic_load(&dev->frequency);
    for (unsigned i = 0; dev->backend->bands && i < dev->backend->band_count; ++i) {
        const struct bs_band *band = &dev->backend->bands[i];
        if (frequency >= band->min_frequency && frequency <= band->max_frequency)
            return band->gain;
    }
    return NULL;
}

int bs_total_gain(const struct birriasdr_gain_table *table, const struct birriasdr_gain *gain)
{
    return table->lna[gain->lna] + table->lpf[gain->lpf] + table->digital[gain->digital];
}

/* Fill the request stage by stage: the highest LNA code that fits, then the
 * filter code that fits best, then the nearest digital code for the rest, of
 * those digital_codes allows (one bit per code; 0 allows all). Of filter codes
 * with equal gain only the lowest is used.
 */
struct birriasdr_gain bs_pick_codes(const struct birriasdr_gain_table *table, int tenths_db,
                                    uint16_t digital_codes)
{
    struct birriasdr_gain gain = {0, 0, 0};
    int rest;
    for (int l = 1; l < table->lna_count; ++l)
        if (table->lna[l] <= tenths_db)
            gain.lna = (uint8_t)l;
    rest = tenths_db - table->lna[gain.lna];
    for (int f = 1; f < table->lpf_count; ++f)
        if (table->lpf[f] != table->lpf[f - 1] && table->lpf[f] <= rest &&
            table->lpf[f] > table->lpf[gain.lpf])
            gain.lpf = (uint8_t)f;
    rest -= table->lpf[gain.lpf];
    for (int d = 1; d < table->digital_count; ++d)
        if ((!digital_codes || digital_codes & 1u << d) &&
            abs(table->digital[d] - rest) < abs(table->digital[gain.digital] - rest))
            gain.digital = (uint8_t)d;
    return gain;
}

int birriasdr_get_gain_table(const birriasdr_dev_t *dev, struct birriasdr_gain_table *table)
{
    const struct birriasdr_gain_table *measured;
    if (!dev || !table)
        return BIRRIASDR_ERROR_INVALID;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_GAIN_DB) || !(measured = bs_band_gain(dev)))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    *table = *measured;
    return 0;
}

int birriasdr_tuner_gain_codes(const birriasdr_dev_t *dev, int tenths_db,
                               struct birriasdr_gain *gain)
{
    const struct birriasdr_gain_table *table;
    if (!dev || !gain)
        return BIRRIASDR_ERROR_INVALID;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_GAIN_DB) || !(table = bs_band_gain(dev)))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    *gain = bs_pick_codes(table, tenths_db, 0);
    return 0;
}

/* Same mode and live-window rules as set_gain_codes, with codes from the table. */
int birriasdr_set_tuner_gain(birriasdr_dev_t *dev, int tenths_db)
{
    struct birriasdr_gain gain;
    int result = birriasdr_tuner_gain_codes(dev, tenths_db, &gain);
    if (result < 0)
        return result;
    pthread_mutex_lock(&dev->control_mutex);
    if (atomic_load(&dev->run_state) != BS_IDLE &&
        (atomic_load(&dev->run_state) != BS_RUNNING || !dev->control_ready))
        result = BIRRIASDR_ERROR_BUSY;
    else if (!dev->manual)
        result = BIRRIASDR_ERROR_STATE;
    else
        result = bs_set_gain_locked(dev, true, &gain);
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* Walk the requests in whole dB and keep each new total that the picker reaches. */
int birriasdr_get_tuner_gains(birriasdr_dev_t *dev, int *gains, size_t capacity)
{
    const struct birriasdr_gain_table *table;
    int count = 0, last = -1, top;
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_GAIN_DB) || !(table = bs_band_gain(dev)))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    top = table->lna[table->lna_count - 1] + table->lpf[table->lpf_count - 1] +
          table->digital[table->digital_count - 1];
    for (int request = 0; request <= top + 9; request += 10) {
        struct birriasdr_gain gain = bs_pick_codes(table, request, 0);
        int total = bs_total_gain(table, &gain);
        if (total <= last)
            continue;
        if (gains && (size_t)count < capacity)
            gains[count] = total;
        ++count;
        last = total;
    }
    return count;
}

/* Choose what automatic mode means, and reapply the current mode under it. */
int birriasdr_set_agc_mode(birriasdr_dev_t *dev, int enabled)
{
    bool before;
    int result;
    if (!dev || (enabled != 0 && enabled != 1))
        return BIRRIASDR_ERROR_INVALID;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_AGC))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    pthread_mutex_lock(&dev->control_mutex);
    before = dev->host_agc;
    dev->host_agc = enabled != 0;
    result = bs_set_gain_locked(dev, dev->manual, &dev->gain);
    if (result < 0)
        dev->host_agc = before;
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

/* TODO: independently selectable filter widths and calibrated RF response.
 * The current backend applies the profile selected by the sample rate during
 * preparation.
 */
int birriasdr_set_tuner_bandwidth(birriasdr_dev_t *dev, uint32_t bandwidth)
{
    (void)bandwidth;
    return unsupported_control(dev);
}

/* The frequency to program for a request when the reference is ppm off, rounded. */
uint64_t bs_corrected_frequency(uint64_t frequency, int32_t ppm)
{
    uint64_t scale = (uint64_t)(1000000 + ppm);
    return (frequency * 1000000 + scale / 2) / scale;
}

/* Same live-window rules as set_center_freq; a failed retune restores the old value. */
int birriasdr_set_freq_correction(birriasdr_dev_t *dev, int ppm)
{
    unsigned state;
    int32_t before;
    int result = 0;
    if (!dev || ppm < -1000 || ppm > 1000)
        return BIRRIASDR_ERROR_INVALID;
    if (!(dev->backend->capabilities & BIRRIASDR_CAP_FREQ_CORRECTION))
        return BIRRIASDR_ERROR_UNSUPPORTED;
    pthread_mutex_lock(&dev->control_mutex);
    state = atomic_load(&dev->run_state);
    if (state != BS_IDLE && (state != BS_RUNNING || !dev->control_ready)) {
        result = BIRRIASDR_ERROR_BUSY;
    } else {
        before = dev->ppm;
        dev->ppm = ppm;
        if (state == BS_RUNNING) {
            result = dev->backend->change_frequency(dev, atomic_load(&dev->frequency));
            if (result < 0) {
                dev->ppm = before;
                control_failed(dev, result, BIRRIASDR_STAGE_RF);
            }
        }
    }
    pthread_mutex_unlock(&dev->control_mutex);
    return result;
}

int birriasdr_get_freq_correction(const birriasdr_dev_t *dev)
{
    return dev ? dev->ppm : 0;
}

/* TODO: decode board routing; the ICAP chain selector is a different control. */
int birriasdr_set_antenna(birriasdr_dev_t *dev, unsigned antenna)
{
    (void)antenna;
    return unsupported_control(dev);
}

/* TODO: identify writable coefficient fields and their ordering/units. This API
 * reserves volatile application only; it must never become an eFuse writer.
 */
int birriasdr_set_calibration(birriasdr_dev_t *dev, const void *data, size_t length)
{
    (void)data;
    (void)length;
    return unsupported_control(dev);
}

/* There is no retained host queue between reads. Clear diagnostics only. */
int birriasdr_reset_buffer(birriasdr_dev_t *dev)
{
    BS_TRY(require_idle(dev));
    memset(&dev->stats, 0, sizeof(dev->stats));
    return 0;
}

/* Recovery requires a gap consumer and a finite per-read budget. Configuration
 * is idle-only; a read owns its callback/context for its entire lifetime.
 */
int birriasdr_set_recovery(birriasdr_dev_t *dev, uint32_t limit,
                          birriasdr_recovery_cb_t callback, void *context)
{
    BS_TRY(require_idle(dev));
    if (limit && !callback)
        return BIRRIASDR_ERROR_INVALID;
    dev->recovery_limit = limit;
    dev->recovery_callback = limit ? callback : NULL;
    dev->recovery_context = limit ? context : NULL;
    return 0;
}

/* Cancellation and running/idle share one atomic state, so startup cannot erase
 * a concurrent cancellation and idle cancellation cannot poison the next read.
 * The read thread owns STOP/drain; other live controls share its register lock.
 */
int birriasdr_cancel_async(birriasdr_dev_t *dev)
{
    unsigned expected;
    if (!dev)
        return BIRRIASDR_ERROR_INVALID;
    expected = atomic_load(&dev->run_state);
    while (expected == BS_RUNNING || expected == BS_CONTROL_FAILED)
        if (atomic_compare_exchange_weak(&dev->run_state, &expected, BS_CANCELLING))
            return 0;
    return expected == BS_CANCELLING ? 0 : BIRRIASDR_ERROR_STATE;
}

/* Statistics are stable once the read thread has returned. */
int birriasdr_get_stream_stats(birriasdr_dev_t *dev, struct birriasdr_stream_stats *stats)
{
    BS_TRY(require_idle(dev));
    if (!stats)
        return BIRRIASDR_ERROR_INVALID;
    *stats = dev->stats;
    return 0;
}

/* The shared read engine owns startup, asynchronous transfers and cleanup. */
int birriasdr_read_async(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
                        void *context, uint32_t depth, uint32_t buffer_samples)
{
    return bs_stream(dev, callback, context, depth, buffer_samples, 0);
}

struct sync_buffer { int16_t *iq; uint32_t count; uint32_t received; };

/* The finite firmware packet count bounds this copy; keep a defensive bound
 * so malformed backend output cannot overflow the caller's allocation.
 */
static void copy_sync(const int16_t *iq, uint32_t count, uint64_t first, void *context)
{
    struct sync_buffer *buffer = context;
    (void)first;
    if (count > buffer->count - buffer->received)
        count = buffer->count - buffer->received;
    memcpy(buffer->iq + 2u * (size_t)buffer->received, iq, 4u * (size_t)count);
    buffer->received += count;
}

/* Finite reads use the same queued USB engine and packet validation as async.
 * Requiring whole payloads avoids discarding unseen samples at the last packet.
 */
int birriasdr_read_sync(birriasdr_dev_t *dev, int16_t *iq, uint32_t count, uint32_t *n_read)
{
    struct sync_buffer buffer = {iq, count, 0};
    int result;
    if (n_read)
        *n_read = 0;
    if (!dev || !iq || !n_read || !count || count % BS_PACKET_SAMPLES)
        return BIRRIASDR_ERROR_INVALID;
    result = bs_stream(dev, copy_sync, &buffer, 64, 16384, count / bs_packet_samples(dev));
    *n_read = buffer.received;
    if (!result && buffer.received != count)
        return BIRRIASDR_ERROR_PROTOCOL;
    return result;
}
