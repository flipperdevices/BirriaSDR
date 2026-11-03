/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */

/* Exercise terminal accounting at a simulated multi-day packet count. Include
 * the private reader implementation and the existing fake transport to seed
 * its framer directly, without adding a test-only entry point to the library.
 * No real USB transport is linked or called.
 */
#define main stream_lifecycle_test_main
#include "test_stream.c"
#undef main
#include "stream.c"

/* A terminal count is modulo 2^32; the parser already holds a 64-bit count.
 * An empty queue must finish at that extended count, rejecting a backwards
 * terminal count instead of treating it as billions of outstanding packets.
 */
static void terminal_count(uint64_t received, uint32_t submitted, int expected)
{
    struct model model = {.generated = submitted, .terminal = 3, .stopped = true};
    birriasdr_dev_t dev = {.backend = &backend, .control_mutex = PTHREAD_MUTEX_INITIALIZER};
    struct bs_reader reader = {.dev = &dev};
    struct bs_framer framer = {.packets = received};
    atomic_init(&dev.run_state, BS_CANCELLING);
    dev.io.context = &model;
    CHECK(receive_loop(&dev, &reader, &framer, 0) == expected);
    CHECK(pthread_mutex_destroy(&dev.control_mutex) == 0);
}

/* An aggregate of whole 1,024-byte packets arrives with four zero bytes, which
 * are not events; any other length, or a nonzero pad, passes or fails as is.
 */
static void packet_pad(void)
{
    static uint8_t bytes[57 * 1024 + 4];
    size_t length = sizeof(bytes);
    CHECK(event_bytes(bytes, &length) == 0 && length == 57 * 1024);
    length = 7 * 8320;
    CHECK(event_bytes(bytes, &length) == 0 && length == 7 * 8320);
    bytes[sizeof(bytes) - 1] = 1;
    length = sizeof(bytes);
    CHECK(event_bytes(bytes, &length) == BIRRIASDR_ERROR_PROTOCOL);
}

int main(void)
{
    packet_pad();
    terminal_count(UINT32_MAX, UINT32_MAX, 0);
    terminal_count(UINT64_C(1) << 32, 0, 0);
    terminal_count((UINT64_C(1) << 32) + 1, 1, 0);
    terminal_count((UINT64_C(2) << 32) + 17, 17, 0);
    terminal_count((UINT64_C(1) << 32) + 1, 0, BIRRIASDR_ERROR_PROTOCOL);
    puts("Terminal packet rollover tests passed");
    return 0;
}
