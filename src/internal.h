/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#ifndef BIRRIASDR_INTERNAL_H
#define BIRRIASDR_INTERNAL_H

#include "birriasdr.h"
#include <libusb.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>

#define BS_TRY(expr) do { int bs_result_ = (expr); if (bs_result_ < 0) return bs_result_; } while (0)
#define BS_PAYLOAD_BYTES 8192u
#define BS_PACKET_SAMPLES 2048u
/* The event before the payload: 44 header bytes, zeros to 124 and the four zeros
 * USB takes past the RXD0 length. 128 keeps the payload's 128-byte lines aligned
 * in USB aggregates. A gap marker is a header alone. */
#define BS_HEADER_BYTES 128u
#define BS_FRAME_BYTES (BS_HEADER_BYTES + BS_PAYLOAD_BYTES)

enum bs_space { BS_MMIO, BS_UHW, BS_BRIDGE };
enum bs_run_state { BS_IDLE, BS_RUNNING, BS_CANCELLING, BS_CONTROL_FAILED };

/* Register I/O is injectable for ordered hardware-sequence and failure tests.
 * It keeps RF code independent of libusb packet details without hiding errors.
 */
struct bs_io {
    void *context;
    int (*read32)(void *, enum bs_space, uint32_t, uint32_t *);
    int (*write32)(void *, enum bs_space, uint32_t, uint32_t);
};

/* The AGC's own state: the target total (negative until the first tick), the
 * ceiling a clip set and the ticks it still holds, the ticks left before the
 * next slow rise, and those before a fast climb is allowed again after a clip.
 */
struct bs_agc_state {
    int target, ceiling;
    unsigned hold, wait, lock;
};

/* A receive filter width, in Hz, measured clean at one sample rate. */
struct bs_filter {
    uint32_t rate, width;
};

/* One contiguous tuning interval; backends list them in ascending order. A band
 * without a measured gain table leaves calibrated gain UNSUPPORTED there.
 */
struct bs_band {
    uint64_t min_frequency, max_frequency;
    const struct birriasdr_gain_table *gain;
};

struct bs_backend {
    enum birriasdr_chipset chipset;
    const char *name;
    uint32_t capabilities;
    uint64_t default_frequency;
    uint64_t min_frequency, max_frequency;
    const struct bs_band *bands; /* NULL means one band, min to max. */
    unsigned band_count;
    /* Digital gain codes the AGC may use, one bit per code; 0 allows all. */
    uint16_t agc_digital;
    /* Receive filters each rate can use, by rate then width ascending; NULL means
     * the rate's span only.
     */
    const struct bs_filter *filters;
    unsigned filter_count;
    /* Receive chains a read can capture; 0 or 1 means only the first. */
    unsigned antenna_count;
    /* With CAP_CHANNEL_FILTER: how far below the tuned center the chip's 20 MHz
     * channel filter sits, in Hz. The channel-filtered path tunes this much high
     * and turns the samples back on the host.
     */
    uint32_t filter_offset;
    /* Switch the captured chain during a read, for the automatic antenna. */
    int (*select_chain)(birriasdr_dev_t *, unsigned chain);
    uint32_t default_rate;
    int (*validate)(uint64_t frequency, uint32_t rate, const struct birriasdr_gain *gain);
    int (*identify)(birriasdr_dev_t *);
    int (*prepare)(birriasdr_dev_t *, uint32_t packets);
    int (*start)(birriasdr_dev_t *);
    int (*stop)(birriasdr_dev_t *);
    int (*status)(birriasdr_dev_t *, uint32_t words[16]);
    int (*reset)(birriasdr_dev_t *);
    int (*change_gain)(birriasdr_dev_t *, bool manual, const struct birriasdr_gain *);
    int (*change_frequency)(birriasdr_dev_t *, uint64_t frequency);
};

struct bs_usb_id {
    uint16_t vid, pid;
    const char *name;
    const struct bs_backend *backend;
    bool qualified;
    /* Board tuning bounds; zero keeps the backend's. */
    uint64_t min_frequency, max_frequency;
};

struct birriasdr_dev {
    libusb_context *usb;
    libusb_device_handle *handle;
    int interface_number;
    int lock_fd;
    uint8_t ep_in, ep_command, ep_download;
    const struct bs_backend *backend;
    uint64_t min_frequency, max_frequency;
    struct bs_io io;
    atomic_uint run_state;
    /* Serializes whole register procedures, not individual USB transfers.
     * Application callbacks and the USB event pump never hold this mutex.
     */
    pthread_mutex_t control_mutex;
    bool control_ready;
    /* Immutable while a reader owns the handle; enables lossless USB backpressure. */
    bool stream_continuous;
    int control_error;
    enum birriasdr_stage control_error_stage;
    birriasdr_recovery_cb_t recovery_callback;
    void *recovery_context;
    uint32_t recovery_limit;
    atomic_uint_fast64_t frequency;
    int32_t ppm;
    uint32_t rate;       /* the hardware's rate; delivered samples are rate >> decimation */
    unsigned decimation; /* half-band stages after decoding, 0 at the native rates */
    unsigned usb_burst;  /* endpoint burst to program; 0 for BIRRIA_BURST or the ROM's */
    char cpus[64];       /* CPU list for the library's threads, when cpus_set */
    bool cpus_set;       /* else BIRRIA_CPUS applies */
    uint32_t bandwidth; /* requested filter width in Hz; 0 matches the rate */
    uint8_t antenna;    /* receive chain captured, the choice when automatic */
    bool channel_filter; /* use the chip's channel-filtered taps where the rate allows */
    bool dual;          /* capture both chains, birriasdr_read_async_multi() */
    uint8_t host_burst; /* bMaxBurst the host enumerated for the data endpoint */
    atomic_uint lane;   /* channel-filtered 40 MSps: the beat word pair of the captured chain */
    bool antenna_auto;  /* choose the stronger chain at start and after retunes */
    bool manual;
    /* Automatic mode runs the host AGC, keeping the codes forced, instead of
     * releasing them to the chip.
     */
    bool host_agc;
    struct birriasdr_gain gain;
    struct {
        atomic_uint_fast64_t power, count;
        atomic_int peak;
        atomic_bool stop;
        pthread_t thread;
        bool running;
        /* DC estimate: written by delivery only; a digital gain change asks it to restart. */
        atomic_int_fast64_t dc_i, dc_q, dc_count;
        atomic_bool dc_restart;
        uint64_t dc_skip_until; /* bs_now_ns() before which DC is not measured */
        /* Automatic antenna: compare the chains at the next chance; withhold
         * samples until the first comparison of a read is done.
         */
        atomic_bool probe_request, probing;
        struct bs_agc_state state; /* used by the AGC thread only */
    } agc;
    uint32_t nonce;
    uint8_t command_sequence;
    bool touched;
    bool needs_usb_reset;
    bool usb_lost;
    struct {
        uint32_t gain_baseline[2][4];
        unsigned gain_chains; /* bit per receive chain forced */
        bool gain_saved;
        uint32_t filter_baseline[12];
        uint32_t filter_fields;
        bool filter_saved;
        bool high_band;
        bool band_valid;
    } mt7921;
    struct birriasdr_stream_stats stats;
};

typedef int (*bs_payload_fn)(void *context, const uint8_t *payload);
typedef int (*bs_gap_fn)(void *context, uint32_t state, uint64_t packets);
struct bs_framer {
    uint8_t pending[BS_FRAME_BYTES];
    size_t used;
    uint32_t nonce;
    uint32_t gaps;
    uint64_t packets;
    unsigned acknowledgements;
    bs_payload_fn payload;
    bs_gap_fn gap;
    void *context;
};

/* Host AGC window as RMS of I or Q (-30 and -12 dB of full scale), step in tenths
 * of a dB, and the sample level that cuts gain at once; fpv-scanner's values.
 */
#define BS_AGC_LOW 250
#define BS_AGC_HIGH 2000
#define BS_AGC_STEP 60
#define BS_AGC_CLIP 7500
/* Fast attack, hold, slow decay, in 20 ms ticks and tenths of a dB: after a clip
 * the gain stays at or below where the attack put it for 500 ms, counted again
 * from each clip; below the window it climbs 1 dB every 100 ms, and more than
 * 12 dB below it 6 dB a tick, but not within 2 s of a clip, so the gaps between
 * bursts do not set off a fast climb.
 */
#define BS_AGC_HOLD_TICKS 25
#define BS_AGC_LOCK_TICKS 100
#define BS_AGC_RISE 10
#define BS_AGC_RISE_TICKS 5


uint64_t bs_corrected_frequency(uint64_t frequency, int32_t ppm);
const struct birriasdr_gain_table *bs_band_gain(const birriasdr_dev_t *dev);
int bs_total_gain(const struct birriasdr_gain_table *table, const struct birriasdr_gain *gain);
struct birriasdr_gain bs_pick_codes(const struct birriasdr_gain_table *table, int tenths_db,
                                    uint16_t digital_codes);
int bs_set_gain_locked(birriasdr_dev_t *dev, bool manual, const struct birriasdr_gain *gain);
int bs_agc_next(struct bs_agc_state *state, int total, int top, uint64_t mean_square, int peak);
bool bs_agc_measure(birriasdr_dev_t *dev, const int16_t *iq, uint32_t count);
void bs_agc_start(birriasdr_dev_t *dev);
void bs_agc_stop(birriasdr_dev_t *dev);

/* The filter a read uses: of those listed for the rate, the smallest at least
 * the request, or the widest for no request or one wider than any. Without a
 * list, or none for the rate, the rate's span.
 */
static inline uint32_t bs_filter_width(const birriasdr_dev_t *dev)
{
    uint32_t widest = 0;
    if (!dev->backend || !dev->backend->filters)
        return dev->rate;
    for (unsigned i = 0; i < dev->backend->filter_count; ++i) {
        const struct bs_filter *f = &dev->backend->filters[i];
        if (f->rate != dev->rate)
            continue;
        if (dev->bandwidth && f->width >= dev->bandwidth)
            return f->width;
        widest = f->width;
    }
    return widest ? widest : dev->rate;
}

extern const struct bs_backend bs_mt7921_backend;
extern const struct bs_backend bs_mt7925_backend;
const struct bs_usb_id *bs_lookup_id(uint16_t vid, uint16_t pid);
int bs_usb_discover(uint32_t index, struct birriasdr_device_info *info,
                    birriasdr_dev_t *open_device);
void bs_usb_close(birriasdr_dev_t *dev);
int bs_usb_error(int error);
int bs_control(birriasdr_dev_t *dev, uint8_t type, uint8_t request,
               uint16_t value, uint16_t index, uint8_t *data, uint16_t length);
int bs_bulk_out(birriasdr_dev_t *dev, uint8_t endpoint, const uint8_t *data, int length);
int bs_usb_prepare_endpoints(birriasdr_dev_t *dev);
int bs_usb_recover(birriasdr_dev_t *dev);
int bs_read32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t *value);
int bs_write32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t value);
int bs_update32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
                uint32_t clear, uint32_t set);
int bs_poll32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
              uint32_t mask, uint32_t expected, unsigned timeout_ms);
uint64_t bs_now_ns(void);
void bs_sleep_us(unsigned microseconds);
int bs_parse_cpus(const char *list, void *cpu_set);
void bs_pin_thread(const birriasdr_dev_t *dev);
void *bs_pin_caller(const birriasdr_dev_t *dev);
void bs_restore_caller(void *saved);
uint32_t bs_get_le32(const uint8_t *bytes);
void bs_put_le32(uint8_t *bytes, uint32_t value);
void bs_put_le16(uint8_t *bytes, uint16_t value);
int bs_framer_feed(struct bs_framer *framer, const uint8_t *data, size_t length);
int bs_framer_finish(const struct bs_framer *framer, uint64_t expected);
int bs_decode_iq(const uint8_t *raw, int16_t *iq, size_t samples);
int bs_decode_iq_halved(const uint8_t *raw, int16_t *iq, size_t words);
int bs_decode_iq_lane(const uint8_t *raw, int16_t *iq, size_t words, unsigned lane);
int bs_decode_iq_dual(const uint8_t *raw, int16_t *iq0, int16_t *iq1, size_t words);

/* Whether a read uses the chip's channel-filtered taps: by default at 20 and
 * 40 MSps, and for two chains. They arrive in the two-chain format at 40 MSps a
 * chain, 320 MB/s, so only when the host takes bursts of 3 or more; otherwise,
 * or with the setting off, the stock taps.
 */
static inline bool bs_filtered(const birriasdr_dev_t *dev)
{
    if (!dev->channel_filter || !dev->backend ||
        !(dev->backend->capabilities & BIRRIASDR_CAP_CHANNEL_FILTER) || dev->decimation)
        return false;
    return dev->dual || ((dev->rate == 20000000 || dev->rate == 40000000) && dev->host_burst >= 3);
}

/* Both chains in each 16-byte record, [chain 0 t, chain 1 t, chain 0 t+1,
 * chain 1 t+1], 40 MSps a chain: every channel-filtered read, which keeps one
 * lane, and every two-chain read, which keeps both.
 */
static inline bool bs_lanes(const birriasdr_dev_t *dev)
{
    return dev->dual || bs_filtered(dev);
}

/* The rate the hardware delivers to the host, a chain. */
static inline uint32_t bs_native_rate(const birriasdr_dev_t *dev)
{
    return bs_lanes(dev) ? 40000000u : dev->decimation ? 10000000u : dev->rate;
}

/* Half-band stages on the host: below 10 MSps the decimation, and one for the
 * two-chain format at 20 MSps a chain. */
static inline unsigned bs_host_halvings(const birriasdr_dev_t *dev)
{
    if (dev->decimation)
        return dev->decimation;
    return bs_lanes(dev) && dev->rate == 20000000 ? 1u : 0u;
}

/* The frequency shift a read applies: the chip's 20 MHz channel filter sits this
 * far below the tuned center, so the channel-filtered path with it tunes this
 * much high and turns the samples back on the host before any half-band stage.
 * 0 for every other path.
 */
static inline uint32_t bs_filter_offset(const birriasdr_dev_t *dev)
{
    return bs_filtered(dev) && bs_filter_width(dev) <= 20000000 ? dev->backend->filter_offset : 0;
}

/* Hardware samples a packet carries, a chain: the packer at 10 and 20 MSps pads
 * each 16-byte record with two zero words, and the two-chain format gives each
 * chain half the words.
 */
static inline uint32_t bs_packet_samples(const birriasdr_dev_t *dev)
{
    return bs_lanes(dev) || dev->rate <= 20000000 ? BS_PACKET_SAMPLES / 2 : BS_PACKET_SAMPLES;
}
/* Power-of-two decimation below the 10 MSps hardware rate, in decimate.c. */
#define BS_DECIMATION_BASE 10000000u
#define BS_DECIMATION_MAX 4u
#define BS_DECIMATED_GAIN 4.0f /* two more bits: full scale 32768 */
#define BS_HALFBAND_SIDE 14u
#define BS_HALFBAND_HISTORY (4u * BS_HALFBAND_SIDE - 2u) /* 55 taps less one */
struct bs_decimator {
    unsigned stages;
    bool after_channel; /* one stage behind the chip's channel filter: a shorter filter */
    float history[BS_DECIMATION_MAX][2 * BS_HALFBAND_HISTORY];
    float work[2 * (BS_HALFBAND_HISTORY + BS_PACKET_SAMPLES)];
    float planar[6][(BS_HALFBAND_HISTORY + BS_PACKET_SAMPLES) / 2]; /* even, odd and out I, Q */
    float stage[BS_PACKET_SAMPLES];
};
void bs_decimator_reset(struct bs_decimator *decimator, unsigned stages, bool after_channel);
/* samples is a multiple of 2^stages, at most BS_PACKET_SAMPLES; returns the output count. */
size_t bs_decimate(struct bs_decimator *decimator, const int16_t *in, size_t samples,
                   int16_t *out);

int bs_stream(birriasdr_dev_t *dev, birriasdr_read_async_cb_t callback,
               birriasdr_read_multi_cb_t multi, void *context, uint32_t depth,
               uint32_t buffer_samples, uint32_t packets);

#endif
