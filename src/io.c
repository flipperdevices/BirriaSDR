/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#define _GNU_SOURCE
#include "internal.h"
#include <errno.h>
#include <stdlib.h>
#include <time.h>
#ifdef __linux__
#include <sched.h>
#endif

/* Parse a CPU list such as "4-7", "4,6" or a mix into cpu_set, a cpu_set_t on
 * Linux, or only check it when cpu_set is NULL. Empty is valid and selects none.
 */
int bs_parse_cpus(const char *list, void *cpu_set)
{
#ifdef __linux__
    cpu_set_t *set = cpu_set;
    if (set)
        CPU_ZERO(set);
#else
    (void)cpu_set;
#endif
    while (*list) {
        char *end;
        long first = strtol(list, &end, 10), last;
        if (end == list || first < 0 || first >= 1024)
            return BIRRIASDR_ERROR_INVALID;
        last = first;
        if (*end == '-') {
            list = end + 1;
            last = strtol(list, &end, 10);
            if (end == list || last < first || last >= 1024)
                return BIRRIASDR_ERROR_INVALID;
        }
#ifdef __linux__
        for (long cpu = first; set && cpu <= last && cpu < CPU_SETSIZE; ++cpu)
            CPU_SET((int)cpu, set);
#endif
        if (*end != ',' && *end)
            return BIRRIASDR_ERROR_INVALID;
        list = *end ? end + 1 : end;
    }
    return 0;
}

/* Pin the calling thread to the device's CPU list, else BIRRIA_CPUS's, if any.
 * The USB event thread calls it, and a stream's caller through bs_pin_caller,
 * so the host application keeps its other threads wherever its scheduler puts them.
 */
void bs_pin_thread(const birriasdr_dev_t *dev)
{
#ifdef __linux__
    const char *list = dev->cpus_set ? dev->cpus : getenv("BIRRIA_CPUS");
    cpu_set_t set;
    if (!list || !*list || bs_parse_cpus(list, &set) || !CPU_COUNT(&set))
        return;
    sched_setaffinity(0, sizeof(set), &set);
#else
    (void)dev;
#endif
}

/* Pin the calling thread for a stream and return its previous CPU set, for
 * bs_restore_caller, or NULL when there is nothing to restore.
 */
void *bs_pin_caller(const birriasdr_dev_t *dev)
{
#ifdef __linux__
    cpu_set_t *saved = malloc(sizeof(*saved));
    if (saved && sched_getaffinity(0, sizeof(*saved), saved)) {
        free(saved);
        saved = NULL;
    }
    bs_pin_thread(dev);
    return saved;
#else
    (void)dev;
    return NULL;
#endif
}

void bs_restore_caller(void *saved)
{
#ifdef __linux__
    if (saved)
        sched_setaffinity(0, sizeof(cpu_set_t), saved);
#endif
    free(saved);
}

/* Explicit byte assembly keeps protocol access safe on unaligned/big-endian hosts. */
uint32_t bs_get_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

/* USB register and mailbox words are always little-endian. */
void bs_put_le32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i)
        p[i] = (uint8_t)(value >> (i * 8));
}

/* MCU command lengths use 16-bit little-endian fields. */
void bs_put_le16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

/* Hardware deadlines use a monotonic clock so wall-clock adjustments are harmless. */
uint64_t bs_now_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (uint64_t)t.tv_sec * UINT64_C(1000000000) + (uint64_t)t.tv_nsec;
}

/* Preserve settling delays when a process receives a signal. */
void bs_sleep_us(unsigned us)
{
    struct timespec t = {(time_t)(us / 1000000u), (long)(us % 1000000u) * 1000};
    while (nanosleep(&t, &t) < 0 && errno == EINTR) {}
}

/* Dispatch one register read through the selected transport/test harness. */
int bs_read32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t *value)
{
    return dev->io.read32(dev->io.context, space, address, value);
}

/* Never retry a failed write automatically: some registers trigger actions. */
int bs_write32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address, uint32_t value)
{
    return dev->io.write32(dev->io.context, space, address, value);
}

/* Preserve unrelated register bits. set may include bits outside clear, matching
 * the decoded Python driver semantics; this is also used for pure set operations.
 */
int bs_update32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
                uint32_t clear, uint32_t set)
{
    uint32_t value;
    BS_TRY(bs_read32(dev, space, address, &value));
    return bs_write32(dev, space, address, (value & ~clear) | set);
}

/* Every busy/done handshake is bounded. Returning a timeout aborts the outer
 * setup and leads to reset instead of continuing with a half-configured radio.
 */
int bs_poll32(birriasdr_dev_t *dev, enum bs_space space, uint32_t address,
              uint32_t mask, uint32_t expected, unsigned timeout_ms)
{
    uint64_t deadline = bs_now_ns() + (uint64_t)timeout_ms * 1000000u;
    do {
        uint32_t value;
        BS_TRY(bs_read32(dev, space, address, &value));
        if ((value & mask) == expected)
            return 0;
        bs_sleep_us(100);
    } while (bs_now_ns() < deadline);
    return BIRRIASDR_ERROR_TIMEOUT;
}
