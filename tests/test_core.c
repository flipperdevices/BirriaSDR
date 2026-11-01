/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "internal.h"
#include "chips/mt7921.h"
#include "chips/mt7921_image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); exit(1); } } while (0)

/* Count parser deliveries without depending on DSP or external USB hardware. */
static int payload(void *context, const uint8_t *raw)
{
    unsigned *count = context;
    int16_t iq[BS_PACKET_SAMPLES * 2];
    CHECK(bs_decode_iq(raw, iq, BS_PACKET_SAMPLES) == 0);
    ++*count;
    return 0;
}

/* Construct a protocol fixture independently of any USB transfer boundaries. */
static void make_frame(uint8_t frame[BS_FRAME_BYTES], uint32_t sequence)
{
    memset(frame, 0, BS_FRAME_BYTES);
    bs_put_le32(frame, 0x38000000u | (BS_FRAME_BYTES - 4));
    bs_put_le32(frame + 24, 0x38000014u + BS_PAYLOAD_BYTES);
    frame[28] = 0xed;
    frame[32] = 0xf5;
    bs_put_le32(frame + 36, sequence);
    bs_put_le32(frame + 40, 0x12345678);
    bs_put_le32(frame + BS_HEADER_BYTES - 4, 0xffffffff); /* The bytes past RXD0's length are not I/Q. */
}

/* Every split point must produce exactly one packet, including bytewise headers.
 * Then verify gaps, stale nonces, duplicates, truncation and malformed lengths.
 */
static void test_framing(void)
{
    uint8_t frame[BS_FRAME_BYTES];
    make_frame(frame, 0);
    for (size_t split = 0; split <= sizeof(frame); ++split) {
        unsigned count = 0;
        struct bs_framer f = {.nonce = 0x12345678, .payload = payload, .context = &count};
        CHECK(bs_framer_feed(&f, frame, split) == 0);
        CHECK(bs_framer_feed(&f, frame + split, sizeof(frame) - split) == 0);
        CHECK(bs_framer_finish(&f, 1) == 0 && count == 1);
        CHECK(bs_framer_feed(&f, frame, sizeof(frame)) == BIRRIASDR_ERROR_PROTOCOL);
    }
    for (unsigned kind = 0; kind < 4; ++kind) {
        unsigned count = 0;
        struct bs_framer f = {.nonce = 0x12345678, .payload = payload, .context = &count};
        make_frame(frame, 0);
        if (kind == 0) frame[36] = 1;
        if (kind == 1) frame[40] ^= 1;
        if (kind == 2) frame[0] = 0;
        if (kind == 3) frame[32] = 0xf4;
        CHECK(bs_framer_feed(&f, frame, sizeof(frame)) == BIRRIASDR_ERROR_PROTOCOL);
        CHECK(count == 0);
    }
    {
        unsigned count = 0;
        struct bs_framer f = {.nonce = 0x12345678, .payload = payload, .context = &count};
        make_frame(frame, 0);
        CHECK(bs_framer_feed(&f, frame, sizeof(frame) - 1) == 0);
        CHECK(bs_framer_finish(&f, 1) == BIRRIASDR_ERROR_PROTOCOL);
    }
}

struct gap_observation { unsigned count; uint32_t state; uint64_t packets; };

/* Markers carry no samples and must precede the next packet at this sequence. */
static int observe_gap(void *context, uint32_t state, uint64_t packets)
{
    struct gap_observation *o = context;
    ++o->count;
    o->state = state;
    o->packets = packets;
    return 0;
}

/* Exercise every marker split, strict readers, duplicates, missing markers,
 * invalid reasons and both wrapping counters. A marker never advances the
 * data packet count or bypasses the acquisition nonce check.
 */
static void test_gap_events(void)
{
    uint8_t marker[48] = {0};
    bs_put_le32(marker, 0x3800002c);
    bs_put_le32(marker + 8, 9);
    bs_put_le32(marker + 12, 1);
    bs_put_le32(marker + 24, 0x38000014);
    marker[28] = 0xed;
    marker[32] = 0xf6;
    bs_put_le32(marker + 36, 3);
    bs_put_le32(marker + 40, 0x12345678);
    for (size_t split = 0; split <= sizeof(marker); ++split) {
        struct gap_observation o = {0};
        struct bs_framer f = {.nonce = 0x12345678, .packets = 3,
                              .gap = observe_gap, .context = &o};
        CHECK(bs_framer_feed(&f, marker, split) == 0);
        CHECK(bs_framer_feed(&f, marker + split, sizeof(marker) - split) == 0);
        CHECK(o.count == 1 && o.state == 9 && o.packets == 3);
        CHECK(f.packets == 3 && f.gaps == 1 && bs_framer_finish(&f, 3) == 0);
        CHECK(bs_framer_feed(&f, marker, sizeof(marker)) == BIRRIASDR_ERROR_PROTOCOL);
    }
    const unsigned corrupt[] = {8, 12, 24, 28, 32, 36, 40};
    for (unsigned n = 0; n < sizeof(corrupt) / sizeof(corrupt[0]); ++n) {
        struct gap_observation o = {0};
        struct bs_framer f = {.nonce = 0x12345678, .packets = 3,
                              .gap = observe_gap, .context = &o};
        marker[corrupt[n]] ^= 2;
        CHECK(bs_framer_feed(&f, marker, sizeof(marker)) == BIRRIASDR_ERROR_PROTOCOL);
        CHECK(!o.count);
        marker[corrupt[n]] ^= 2;
    }
    struct bs_framer strict = {.nonce = 0x12345678, .packets = 3};
    CHECK(bs_framer_feed(&strict, marker, sizeof(marker)) == BIRRIASDR_ERROR_PROTOCOL);
    struct gap_observation o = {0};
    struct bs_framer f = {.nonce = 0x12345678, .packets = UINT64_C(1) << 32,
                          .gaps = UINT32_MAX, .gap = observe_gap, .context = &o};
    bs_put_le32(marker + 8, 8);
    bs_put_le32(marker + 12, 0);
    bs_put_le32(marker + 36, 0);
    CHECK(bs_framer_feed(&f, marker, sizeof(marker)) == 0);
    CHECK(o.count == 1 && o.state == 8 && o.packets == (UINT64_C(1) << 32));
    CHECK(f.gaps == 0);
}

/* Known signed extrema catch I/Q reversal, sign-extension and accidental scaling. */
static void test_samples(void)
{
    uint8_t raw[8] = {0xff, 0x1f, 0x00, 0x20, 0xff, 0x3f, 0x01, 0x00};
    int16_t iq[4];
    CHECK(bs_decode_iq(raw, iq, 2) == 0);
    CHECK(iq[0] == -8192 && iq[1] == 8191 && iq[2] == 1 && iq[3] == -1);
    raw[7] = 0x40;
    CHECK(bs_decode_iq(raw, iq, 2) == BIRRIASDR_ERROR_PROTOCOL);
}

/* The 20 MSps packer keeps two samples and two zero words per 16-byte record. */
static void test_halved_samples(void)
{
    uint8_t raw[32] = {0};
    int16_t iq[8];
    raw[0] = 0xff; raw[1] = 0x1f; raw[2] = 0x00; raw[3] = 0x20;  /* I -8192, Q 8191 */
    raw[4] = 0xff; raw[5] = 0x3f; raw[6] = 0x01; raw[7] = 0x00;  /* I 1, Q -1 */
    raw[16] = 0x05; raw[22] = 0x07;                              /* Q 5, then I 7 */
    CHECK(bs_decode_iq_halved(raw, iq, 8) == 0);
    CHECK(iq[0] == -8192 && iq[1] == 8191 && iq[2] == 1 && iq[3] == -1);
    CHECK(iq[4] == 0 && iq[5] == 5 && iq[6] == 7 && iq[7] == 0);
    CHECK(bs_decode_iq_halved(raw, iq, 6) == BIRRIASDR_ERROR_INVALID);
    raw[9] = 0x01;                                               /* non-zero padding */
    CHECK(bs_decode_iq_halved(raw, iq, 8) == BIRRIASDR_ERROR_PROTOCOL);
    raw[9] = 0;
    raw[7] = 0x40;                                               /* invalid sample bits */
    CHECK(bs_decode_iq_halved(raw, iq, 8) == BIRRIASDR_ERROR_PROTOCOL);
}

/* A start acknowledgment may arrive fragmented beside sample events. Reject
 * duplicate/nonzero ACKs and carry packet sequence through its wire rollover.
 */
static void test_events(void)
{
    uint8_t ack[40] = {0}, frame[BS_FRAME_BYTES];
    unsigned count = 0;
    struct bs_framer framer = {.nonce = 0x12345678, .payload = payload, .context = &count};
    bs_put_le32(ack, 0x38000024);
    ack[28] = 1;
    CHECK(bs_framer_feed(&framer, ack, 17) == 0);
    CHECK(bs_framer_feed(&framer, ack + 17, 23) == 0);
    make_frame(frame, 0);
    CHECK(bs_framer_feed(&framer, frame, sizeof(frame)) == 0);
    CHECK(framer.acknowledgements == 1 && count == 1);
    CHECK(bs_framer_feed(&framer, ack, sizeof(ack)) == BIRRIASDR_ERROR_PROTOCOL);
    framer = (struct bs_framer){.nonce = 0x12345678, .payload = payload, .context = &count};
    ack[32] = 1;
    CHECK(bs_framer_feed(&framer, ack, sizeof(ack)) == BIRRIASDR_ERROR_PROTOCOL);
    framer = (struct bs_framer){.nonce = 0x12345678, .payload = payload, .context = &count,
                              .packets = UINT32_MAX};
    make_frame(frame, UINT32_MAX);
    CHECK(bs_framer_feed(&framer, frame, sizeof(frame)) == 0);
    make_frame(frame, 0);
    CHECK(bs_framer_feed(&framer, frame, sizeof(frame)) == 0);
    CHECK(bs_framer_finish(&framer, UINT64_C(4294967297)) == 0);
}

/* Every signed 14-bit component must survive decode, including both extrema.
 * Complement Q so reversing the two components cannot accidentally pass.
 */
static void test_all_sample_values(void)
{
    uint8_t raw[4];
    int16_t iq[2];
    for (uint32_t value = 0; value < 16384; ++value) {
        uint32_t q = 16383 - value;
        int expected_i = value < 8192 ? (int)value : (int)value - 16384;
        int expected_q = q < 8192 ? (int)q : (int)q - 16384;
        bs_put_le32(raw, value << 16 | q);
        CHECK(bs_decode_iq(raw, iq, 1) == 0);
        CHECK(iq[0] == expected_i && iq[1] == expected_q);
    }
}

/* Verify the embedded original image's PDA trailer, independently of USB ROM. */
static void test_image(void)
{
    uint32_t state = 0;
    CHECK(sizeof(mt7921_image) == 1632);
    for (size_t i = 0; i < sizeof(mt7921_image); i += 4) {
        uint32_t bits = state & 0x10921111, parity = 0;
        while (bits) { parity ^= bits & 1; bits >>= 1; }
        state = (state >> 1) ^ parity << 31 ^ bs_get_le32(mt7921_image + i);
    }
    CHECK(state == 0);
}

/* Unsupported chip/control paths must fail without USB access or pretend values. */
static void test_api(void)
{
    birriasdr_dev_t dev = {0};
    const struct bs_usb_id *id = bs_lookup_id(0x0e8d, 0x7925);
    struct birriasdr_gain gain = {2, 8, 0};
    uint64_t minimum, maximum;
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    dev.backend = &bs_mt7921_backend;
    dev.frequency = dev.backend->default_frequency;
    dev.rate = dev.backend->default_rate;
    CHECK(id && id->backend->chipset == BIRRIASDR_CHIP_MT7925 && !id->qualified);
    CHECK(id->backend->prepare(&dev, 0) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_tuner_gain(&dev, 100) == BIRRIASDR_ERROR_STATE);
    CHECK(birriasdr_set_agc_mode(&dev, 2) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_set_agc_mode(&dev, 1) == 0 && dev.host_agc);
    CHECK(birriasdr_set_agc_mode(&dev, 0) == 0 && !dev.host_agc);
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 40000000) == 0);
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 0) == 0);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(3000000000)) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(4499999999)) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(6200000001)) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(5180000000)) == 0);
    CHECK(birriasdr_get_center_freq(&dev) == UINT64_C(5180000000));
    CHECK(birriasdr_get_tuning_range(&dev, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2156000000) && maximum == UINT64_C(6200000000));
    CHECK(birriasdr_get_tuning_band(&dev, 1, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(4500000000) && maximum == UINT64_C(6200000000));
    CHECK(birriasdr_set_center_freq(&dev, minimum) == 0);
    CHECK(birriasdr_set_center_freq(&dev, maximum) == 0);
    CHECK(birriasdr_get_tuning_band(&dev, 2, &minimum, &maximum) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_get_tuning_band(&dev, 0, &minimum, &maximum) == 0);
    CHECK(birriasdr_get_capabilities(&dev) & BIRRIASDR_CAP_TUNING);
    CHECK(minimum == UINT64_C(2156000000) && maximum == UINT64_C(2610000000));
    CHECK(birriasdr_set_center_freq(&dev, minimum) == 0);
    CHECK(birriasdr_set_center_freq(&dev, maximum) == 0);
    CHECK(birriasdr_set_center_freq(&dev, minimum - 1) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_center_freq(&dev, maximum + 1) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_get_center_freq(&dev) == maximum);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(2437123456)) == 0);
    CHECK(birriasdr_get_center_freq(&dev) == UINT64_C(2437123456));
    id = bs_lookup_id(0x0e8d, 0x7961);
    CHECK(id && id->qualified && id->min_frequency == UINT64_C(2190000000) && !id->max_frequency);
    dev.min_frequency = id->min_frequency;
    CHECK(birriasdr_get_tuning_band(&dev, 0, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2190000000) && maximum == UINT64_C(2610000000));
    CHECK(birriasdr_get_tuning_band(&dev, 1, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(4500000000) && maximum == UINT64_C(6200000000));
    dev.max_frequency = UINT64_C(2610000000);
    CHECK(birriasdr_get_tuning_band(&dev, 1, &minimum, &maximum) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(5180000000)) == BIRRIASDR_ERROR_UNSUPPORTED);
    dev.max_frequency = 0;
    CHECK(birriasdr_get_tuning_range(&dev, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2190000000) && maximum == UINT64_C(6200000000));
    CHECK(birriasdr_set_center_freq(&dev, minimum - 1) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_get_center_freq(&dev) == UINT64_C(2437123456));
    CHECK(birriasdr_set_center_freq(&dev, minimum) == 0);
    dev.min_frequency = 0;
    CHECK(birriasdr_get_tuning_range(&dev, &minimum, &maximum) == 0);
    CHECK(minimum == UINT64_C(2156000000));
    CHECK(birriasdr_set_sample_rate(&dev, 0) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_sample_rate(&dev, 5000000) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_sample_rate(&dev, 10000000) == 0);
    /* Burst and CPU settings: explicit values win, the environment fills in. */
    unsetenv("BIRRIA_BURST");
    unsetenv("BIRRIA_CPUS");
    CHECK(birriasdr_get_usb_burst(&dev) == 0 && !strcmp(birriasdr_get_cpus(&dev), ""));
    setenv("BIRRIA_BURST", "3", 1);
    setenv("BIRRIA_CPUS", "4-7", 1);
    CHECK(birriasdr_get_usb_burst(&dev) == 3 && !strcmp(birriasdr_get_cpus(&dev), "4-7"));
    CHECK(birriasdr_set_usb_burst(&dev, 16) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_set_usb_burst(&dev, 2) == 0 && birriasdr_get_usb_burst(&dev) == 2);
    CHECK(birriasdr_set_usb_burst(&dev, 0) == 0 && birriasdr_get_usb_burst(&dev) == 3);
    for (const char *bad = "x\0" "4-\0" "7-4\0" "4,,5\0" "1024\0"; *bad; bad += strlen(bad) + 1)
        CHECK(birriasdr_set_cpus(&dev, bad) == BIRRIASDR_ERROR_INVALID);
    CHECK(!strcmp(birriasdr_get_cpus(&dev), "4-7"));
    CHECK(birriasdr_set_cpus(&dev, "0,2-3") == 0 && !strcmp(birriasdr_get_cpus(&dev), "0,2-3"));
    CHECK(birriasdr_set_cpus(&dev, "") == 0 && !strcmp(birriasdr_get_cpus(&dev), ""));
    CHECK(birriasdr_set_cpus(&dev, NULL) == 0 && !strcmp(birriasdr_get_cpus(&dev), "4-7"));
    unsetenv("BIRRIA_BURST");
    unsetenv("BIRRIA_CPUS");
    CHECK(birriasdr_set_sample_rate(&dev, 60000000) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_sample_rate(&dev, 20000000) == 0);
    CHECK(birriasdr_get_sample_rate(&dev) == 20000000);
    CHECK(birriasdr_set_sample_rate(&dev, 80000000) == 0);
    CHECK(birriasdr_get_sample_rate(&dev) == 80000000);
    CHECK(birriasdr_set_sample_rate(&dev, 40000000) == 0);
    CHECK(birriasdr_get_sample_rate(&dev) == 40000000);
    CHECK(birriasdr_set_gain_codes(&dev, &gain) == BIRRIASDR_ERROR_STATE);
    CHECK(birriasdr_set_tuner_gain_mode(&dev, 1) == 0);
    CHECK(birriasdr_set_gain_codes(&dev, &gain) == 0);
    gain.lna = 5;
    CHECK(birriasdr_set_gain_codes(&dev, &gain) == 0);
    gain.lna = 6;
    CHECK(birriasdr_set_gain_codes(&dev, &gain) == BIRRIASDR_ERROR_INVALID);
    gain.lna = 9;
    CHECK(birriasdr_set_gain_codes(&dev, &gain) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_cancel_async(&dev) == BIRRIASDR_ERROR_STATE);
    CHECK(atomic_load(&dev.run_state) == BS_IDLE);
    atomic_store(&dev.run_state, BS_RUNNING);
    CHECK(birriasdr_cancel_async(&dev) == 0);
    CHECK(atomic_load(&dev.run_state) == BS_CANCELLING);
    CHECK(birriasdr_cancel_async(&dev) == 0);
    CHECK(birriasdr_set_tuner_gain_mode(&dev, 0) == BIRRIASDR_ERROR_BUSY);
    CHECK(birriasdr_set_center_freq(&dev, minimum) == BIRRIASDR_ERROR_BUSY);
    CHECK(birriasdr_close(&dev) == BIRRIASDR_ERROR_BUSY);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Calibrated gain uses the table of the band holding the center and fills the
 * LNA first; a backend without measurements refuses.
 */
static void test_tuner_gain(void)
{
    birriasdr_dev_t dev = {0};
    struct birriasdr_gain_table table;
    struct birriasdr_gain gain;
    int gains[64], manual, count;
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    dev.backend = &bs_mt7921_backend;
    dev.frequency = UINT64_C(2472000000);
    dev.rate = dev.backend->default_rate;
    CHECK(birriasdr_get_capabilities(&dev) & BIRRIASDR_CAP_GAIN_DB);
    CHECK(birriasdr_get_gain_table(&dev, &table) == 0);
    CHECK(table.lna_count == 6 && table.lna[5] == 300 && table.lpf[8] == 40);
    CHECK(table.digital_count == 13 && table.digital[12] == 120);
    count = birriasdr_get_tuner_gains(&dev, NULL, 0);
    CHECK(count > 30 && count <= 64);
    CHECK(birriasdr_get_tuner_gains(&dev, gains, 64) == count);
    CHECK(gains[0] == 0 && gains[count - 1] == 300 + 40 + 120);
    CHECK(birriasdr_set_tuner_gain_mode(&dev, 1) == 0 && birriasdr_set_tuner_gain(&dev, 400) == 0);
    CHECK(birriasdr_get_gain_codes(&dev, &gain, &manual) == 0);
    CHECK(gain.lna == 5 && gain.lpf == 8 && gain.digital == 6);
    CHECK(birriasdr_set_tuner_gain_mode(&dev, 0) == 0);
    for (int i = 1; i < count; ++i)
        CHECK(gains[i] == gains[i - 1] + 10);
    CHECK(birriasdr_get_tuner_gains(&dev, gains, 3) == count);
    gain = bs_pick_codes(&table, 400, 1u << 0 | 1u << 6);
    CHECK(gain.lna == 5 && gain.lpf == 8 && gain.digital == 6);
    gain = bs_pick_codes(&table, 350, 1u << 0 | 1u << 6);
    CHECK(gain.lna == 5 && gain.lpf == 8 && gain.digital == 0);
    CHECK(birriasdr_tuner_gain_codes(&dev, 210, &gain) == 0);
    CHECK(gain.lna == 3 && gain.lpf == 0 && gain.digital == 0);
    CHECK(birriasdr_set_tuner_gain(&dev, 210) == BIRRIASDR_ERROR_STATE);
    CHECK(birriasdr_set_tuner_gain_mode(&dev, 1) == 0);
    CHECK(birriasdr_set_tuner_gain(&dev, 210) == 0);
    CHECK(birriasdr_get_gain_codes(&dev, &gain, &manual) == 0 && manual);
    CHECK(gain.lna == 3 && gain.lpf == 0 && gain.digital == 0);
    CHECK(birriasdr_set_tuner_gain(&dev, 250) == 0);
    CHECK(birriasdr_get_gain_codes(&dev, &gain, &manual) == 0);
    CHECK(gain.lna == 4 && gain.lpf == 0 && gain.digital == 1);
    CHECK(birriasdr_set_tuner_gain(&dev, -50) == 0);
    CHECK(birriasdr_get_gain_codes(&dev, &gain, &manual) == 0);
    CHECK(gain.lna == 0 && gain.lpf == 0 && gain.digital == 0);
    CHECK(birriasdr_set_tuner_gain(&dev, 1000) == 0);
    CHECK(birriasdr_get_gain_codes(&dev, &gain, &manual) == 0);
    CHECK(gain.lna == 5 && gain.lpf == 8 && gain.digital == 12);
    CHECK(birriasdr_set_center_freq(&dev, UINT64_C(5785000000)) == 0);
    CHECK(birriasdr_get_gain_table(&dev, &table) == 0 && table.lna[5] == 240);
    CHECK(birriasdr_set_tuner_gain(&dev, 240) == 0);
    CHECK(birriasdr_get_gain_codes(&dev, &gain, &manual) == 0);
    CHECK(gain.lna == 5 && gain.lpf == 0 && gain.digital == 0);
    dev.backend = bs_lookup_id(0x0e8d, 0x7925)->backend;
    CHECK(birriasdr_get_gain_table(&dev, &table) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_get_tuner_gains(&dev, NULL, 0) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_tuner_gain(&dev, 100) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_tuner_gain_codes(&dev, 100, &gain) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Fast attack, hold, slow decay: 6 dB down at once above the window or on a
 * clip, which also caps the target for 25 ticks from the last clip; 1 dB up
 * every 5 ticks below the window and 6 dB a tick far below it, but not within
 * 100 ticks of a clip; nothing inside it; always within the table.
 */
static void test_agc_step(void)
{
    struct bs_agc_state s = {-1, 0, 0, 0, 0};
    const uint64_t quiet = 249 * 249, inside = 1000 * 1000, loud = 2001 * 2001;
    CHECK(bs_agc_next(&s, 200, 464, inside, 3000) == 200);
    CHECK(bs_agc_next(&s, 200, 464, 250 * 250, 1000) == 200);
    CHECK(bs_agc_next(&s, 200, 464, 2000 * 2000, 6000) == 200);
    CHECK(bs_agc_next(&s, 200, 464, loud, 6000) == 140);
    CHECK(bs_agc_next(&s, 140, 464, quiet, 1000) == 150);
    for (int i = 0; i < 4; ++i)
        CHECK(bs_agc_next(&s, 140, 464, quiet, 1000) == 150);
    CHECK(bs_agc_next(&s, 140, 464, quiet, 1000) == 160);
    CHECK(bs_agc_next(&s, 300, 464, inside, BS_AGC_CLIP) == 240);
    CHECK(s.ceiling == 240 && s.hold == BS_AGC_HOLD_TICKS);
    for (int i = 0; i < BS_AGC_HOLD_TICKS; ++i)
        CHECK(bs_agc_next(&s, 240, 464, quiet, 1000) == 240);
    CHECK(s.hold == 0);
    for (int i = 0; i < BS_AGC_RISE_TICKS; ++i)
        bs_agc_next(&s, 240, 464, quiet, 1000);
    CHECK(s.target == 250);
    CHECK(bs_agc_next(&s, 250, 464, quiet, BS_AGC_CLIP) == 190 && s.hold == BS_AGC_HOLD_TICKS);
    CHECK(bs_agc_next(&s, 190, 464, loud, 1000) == 130);
    struct bs_agc_state low = {-1, 0, 0, 0, 0}, high = {-1, 0, 0, 0, 0}, far = {-1, 0, 0, 0, 0};
    CHECK(bs_agc_next(&low, 30, 464, 3000 * 3000, 8191) == 0);
    CHECK(bs_agc_next(&high, 460, 464, 10, 100) == 464);
    CHECK(bs_agc_next(&far, 60, 464, 50 * 50, 100) == 120);
    CHECK(bs_agc_next(&far, 120, 464, 50 * 50, 100) == 180);
    CHECK(bs_agc_next(&far, 180, 464, 50 * 50, BS_AGC_CLIP) == 120);
    CHECK(bs_agc_next(&far, 120, 464, 50 * 50, 100) == 120);
    for (int i = 0; i < BS_AGC_LOCK_TICKS - 2; ++i)
        CHECK(bs_agc_next(&far, far.target, 464, 50 * 50, 100) <= 120 + 10 * (i / 5 + 1));
    CHECK(far.lock == 1 && far.hold == 0);
    int slow = far.target;
    bs_agc_next(&far, slow, 464, 50 * 50, 100);
    CHECK(far.lock == 0 && bs_agc_next(&far, far.target, 464, 50 * 50, 100) == far.target);
    CHECK(far.target >= slow + 60);
}

/* A ppm correction programs center / (1 + ppm / 1e6), rounded, keeps the
 * reported center, stays within +-1000 and needs the capability.
 */
static void test_freq_correction(void)
{
    birriasdr_dev_t dev = {0};
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    dev.backend = &bs_mt7921_backend;
    dev.frequency = UINT64_C(2437000000);
    CHECK(bs_corrected_frequency(UINT64_C(2437000000), 0) == UINT64_C(2437000000));
    CHECK(bs_corrected_frequency(UINT64_C(2437000000), 10) == UINT64_C(2436975630));
    CHECK(bs_corrected_frequency(UINT64_C(5785000000), -20) == UINT64_C(5785115702));
    CHECK(birriasdr_get_capabilities(&dev) & BIRRIASDR_CAP_FREQ_CORRECTION);
    CHECK(birriasdr_set_freq_correction(&dev, 10) == 0 && birriasdr_get_freq_correction(&dev) == 10);
    CHECK(birriasdr_get_center_freq(&dev) == UINT64_C(2437000000));
    CHECK(birriasdr_set_freq_correction(&dev, 1001) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_set_freq_correction(&dev, -1001) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_get_freq_correction(&dev) == 10);
    dev.backend = bs_lookup_id(0x0e8d, 0x7925)->backend;
    CHECK(birriasdr_set_freq_correction(&dev, 5) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* The filter is the smallest of the rate's widths at least the request, the
 * widest for 0 or a request too wide; set only while idle.
 */
static void test_bandwidth(void)
{
    birriasdr_dev_t dev = {0};
    uint32_t widths[8];
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    dev.backend = &bs_mt7921_backend;
    dev.rate = 80000000;
    CHECK(birriasdr_get_capabilities(&dev) & BIRRIASDR_CAP_FILTER);
    CHECK(birriasdr_get_tuner_bandwidth(&dev) == 80000000);
    CHECK(birriasdr_get_tuner_bandwidths(&dev, widths, 8) == 2);
    CHECK(widths[0] == 40000000 && widths[1] == 80000000);
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 15000000) == 0);
    CHECK(birriasdr_get_tuner_bandwidth(&dev) == 40000000);
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 50000000) == 0);
    CHECK(birriasdr_get_tuner_bandwidth(&dev) == 80000000);
    dev.rate = 40000000;
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 1) == 0);
    CHECK(birriasdr_get_tuner_bandwidth(&dev) == 20000000);
    dev.rate = 20000000;
    CHECK(birriasdr_get_tuner_bandwidths(&dev, NULL, 0) == 1);
    CHECK(birriasdr_get_tuner_bandwidth(&dev) == 20000000);
    dev.rate = 10000000;
    CHECK(birriasdr_get_tuner_bandwidth(&dev) == 10000000);
    atomic_store(&dev.run_state, BS_RUNNING);
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 0) == BIRRIASDR_ERROR_BUSY);
    atomic_store(&dev.run_state, BS_IDLE);
    dev.backend = bs_lookup_id(0x0e8d, 0x7925)->backend;
    CHECK(birriasdr_set_tuner_bandwidth(&dev, 0) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* The chain is chosen while idle, within the backend's count, and needs the
 * capability.
 */
static void test_antenna(void)
{
    birriasdr_dev_t dev = {0};
    atomic_init(&dev.run_state, BS_IDLE);
    CHECK(pthread_mutex_init(&dev.control_mutex, NULL) == 0);
    dev.backend = &bs_mt7921_backend;
    CHECK(birriasdr_get_capabilities(&dev) & BIRRIASDR_CAP_ANTENNA);
    CHECK(birriasdr_get_antenna_count(&dev) == 2 && birriasdr_get_antenna(&dev) == 0);
    CHECK(birriasdr_set_antenna(&dev, 1) == 0 && birriasdr_get_antenna(&dev) == 1);
    CHECK(birriasdr_set_antenna(&dev, 2) == BIRRIASDR_ERROR_INVALID);
    CHECK(birriasdr_get_antenna(&dev) == 1);
    atomic_store(&dev.run_state, BS_RUNNING);
    CHECK(birriasdr_set_antenna(&dev, 0) == BIRRIASDR_ERROR_BUSY);
    atomic_store(&dev.run_state, BS_IDLE);
    CHECK(birriasdr_set_antenna(&dev, 0) == 0);
    CHECK(birriasdr_set_antenna(&dev, BIRRIASDR_ANTENNA_AUTO) == 0);
    CHECK(birriasdr_get_antenna_auto(&dev) && birriasdr_get_antenna(&dev) == 0);
    CHECK(birriasdr_set_antenna(&dev, 1) == 0);
    CHECK(!birriasdr_get_antenna_auto(&dev) && birriasdr_get_antenna(&dev) == 1);
    dev.backend = bs_lookup_id(0x0e8d, 0x7925)->backend;
    CHECK(birriasdr_set_antenna(&dev, 0) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_set_antenna(&dev, BIRRIASDR_ANTENNA_AUTO) == BIRRIASDR_ERROR_UNSUPPORTED);
    CHECK(birriasdr_get_antenna_count(&dev) == 1);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* Tests deliberately run without a dongle, root privileges or vendor firmware. */
int main(void)
{
    test_framing();
    test_gap_events();
    test_samples();
    test_halved_samples();
    test_events();
    test_all_sample_values();
    test_image();
    test_api();
    test_tuner_gain();
    test_agc_step();
    test_freq_correction();
    test_bandwidth();
    test_antenna();
    puts("core tests passed");
    return 0;
}
