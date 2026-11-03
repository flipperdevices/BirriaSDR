/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 * Fake libusb completions test ownership; this executable never opens USB.
 */
#include "usb_stream.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); abort(); } } while (0)
struct request { struct libusb_transfer *transfer; bool cancelled; };
static struct {
    pthread_mutex_t mutex;
    struct request pending[256];
    unsigned count, allocated, freed, submitted, fail_submit, completions, limit;
    unsigned allocations, fail_allocation;
    enum libusb_transfer_status status;
} mock = {.mutex = PTHREAD_MUTEX_INITIALIZER};

#ifdef TEST_WRAP_ALLOCATIONS
/* GNU ld wrappers inject each reader allocation failure without adding hooks
 * to production code. Only arm them during single-threaded reader creation.
 */
static unsigned allocation_call, allocation_failure;
void *__real_malloc(size_t size);
void *__real_calloc(size_t count, size_t size);
void *__wrap_malloc(size_t size)
{
    if (allocation_failure && ++allocation_call == allocation_failure) return NULL;
    return __real_malloc(size);
}
void *__wrap_calloc(size_t count, size_t size)
{
    if (allocation_failure && ++allocation_call == allocation_failure) return NULL;
    return __real_calloc(count, size);
}
#endif

/* Allocate the same public transfer layout used by libusb_fill_bulk_transfer. */
struct libusb_transfer * LIBUSB_CALL libusb_alloc_transfer(int iso_packets)
{
    struct libusb_transfer *transfer;
    CHECK(iso_packets == 0);
    if (++mock.allocations == mock.fail_allocation)
        return NULL;
    transfer = calloc(1, sizeof(*transfer));
    if (transfer) ++mock.allocated;
    return transfer;
}

/* Freeing a transfer still owned by the event loop is a test failure. */
void LIBUSB_CALL libusb_free_transfer(struct libusb_transfer *transfer)
{
    pthread_mutex_lock(&mock.mutex);
    for (unsigned i = 0; i < mock.count; ++i)
        CHECK(mock.pending[i].transfer != transfer);
    ++mock.freed;
    pthread_mutex_unlock(&mock.mutex);
    free(transfer);
}

/* Inject either a partial-start failure or a failure during callback resubmission. */
int LIBUSB_CALL libusb_submit_transfer(struct libusb_transfer *transfer)
{
    pthread_mutex_lock(&mock.mutex);
    if (++mock.submitted == mock.fail_submit) {
        pthread_mutex_unlock(&mock.mutex);
        return LIBUSB_ERROR_IO;
    }
    CHECK(mock.count < 256);
    mock.pending[mock.count++] = (struct request){transfer, false};
    pthread_mutex_unlock(&mock.mutex);
    return 0;
}

/* Cancellation marks a request; it deliberately does not complete it yet. */
int LIBUSB_CALL libusb_cancel_transfer(struct libusb_transfer *transfer)
{
    pthread_mutex_lock(&mock.mutex);
    for (unsigned i = 0; i < mock.count; ++i) {
        if (mock.pending[i].transfer == transfer) {
            mock.pending[i].cancelled = true;
            pthread_mutex_unlock(&mock.mutex);
            return 0;
        }
    }
    pthread_mutex_unlock(&mock.mutex);
    return LIBUSB_ERROR_NOT_FOUND;
}

/* Deliver data or deferred cancellation on the actual reader thread, exercising
 * callback/main-thread locking. No completion may outlive reader destruction.
 */
int LIBUSB_CALL libusb_handle_events_timeout_completed(libusb_context *context,
                                                       struct timeval *timeout, int *completed)
{
    struct libusb_transfer *transfer = NULL;
    (void)context;
    (void)timeout;
    (void)completed;
    pthread_mutex_lock(&mock.mutex);
    for (unsigned i = 0; i < mock.count; ++i) {
        struct request request = mock.pending[i];
        if (!request.cancelled && mock.completions >= mock.limit)
            continue;
        transfer = request.transfer;
        memmove(&mock.pending[i], &mock.pending[i + 1], (mock.count - i - 1) * sizeof(request));
        --mock.count;
        transfer->status = request.cancelled ? LIBUSB_TRANSFER_CANCELLED : mock.status;
        transfer->actual_length = request.cancelled ? 0 : 16;
        if (!request.cancelled)
            memset(transfer->buffer, (int)++mock.completions, 16);
        break;
    }
    pthread_mutex_unlock(&mock.mutex);
    if (transfer)
        transfer->callback(transfer);
    else
        bs_sleep_us(1000);
    return 0;
}

/* Reset only after all previous transfers have completed and been freed. */
static void reset_mock(unsigned limit, unsigned fail_submit)
{
    CHECK(mock.count == 0 && mock.allocated == mock.freed);
    mock.allocated = mock.freed = mock.submitted = mock.completions = 0;
    mock.allocations = mock.fail_allocation = 0;
    mock.status = LIBUSB_TRANSFER_COMPLETED;
    mock.limit = limit;
    mock.fail_submit = fail_submit;
}

/* Wait for the fake device to fill all 32 queue slots and park completions. */
static void wait_completions(unsigned count)
{
    uint64_t deadline = bs_now_ns() + UINT64_C(1000000000);
    for (;;) {
        pthread_mutex_lock(&mock.mutex);
        unsigned completed = mock.completions;
        pthread_mutex_unlock(&mock.mutex);
        if (completed >= count) return;
        CHECK(bs_now_ns() < deadline);
        bs_sleep_us(1000);
    }
}

/* The USB reader only checks the opt-in. Gap parsing belongs to stream.c. */
static void unused_gap(const struct birriasdr_recovery_event *event, void *context)
{
    (void)event; (void)context;
    abort();
}

/* Retaining completed requests must preserve every byte in order while using
 * bounded storage. Also destroy fully/partly parked readers and inject failure
 * when the consumer first resumes one. No libusb-owned buffer may be freed.
 */
static void test_backpressure(void)
{
    birriasdr_dev_t dev = {.stream_continuous = true, .recovery_limit = 1,
                          .recovery_callback = unused_gap};
    struct bs_reader *reader;
    const uint8_t *bytes;
    size_t length;
    reset_mock(160, 0);
    CHECK(bs_reader_create(&dev, 4, &reader) == 0);
    CHECK(bs_reader_start(reader) == 0);
    wait_completions(36);
    bs_sleep_us(5000);
    pthread_mutex_lock(&mock.mutex);
    CHECK(mock.completions == 36 && mock.count == 0);
    pthread_mutex_unlock(&mock.mutex);
    for (unsigned i = 1; i <= 160; ++i) {
        CHECK(bs_reader_take(reader, &bytes, &length, 1000) == 0);
        CHECK(length == 16 && bytes[0] == i && bytes[15] == i);
        if (i % 20 == 0) bs_sleep_us(1000);
    }
    CHECK(bs_reader_take(reader, &bytes, &length, 5) == BIRRIASDR_ERROR_TIMEOUT);
    bs_reader_destroy(reader);
    for (unsigned count = 34; count <= 36; ++count) {
        reset_mock(count, 0);
        CHECK(bs_reader_create(&dev, 4, &reader) == 0);
        CHECK(bs_reader_start(reader) == 0);
        wait_completions(count);
        bs_reader_destroy(reader);
    }
    reset_mock(100, 37);
    CHECK(bs_reader_create(&dev, 4, &reader) == 0);
    CHECK(bs_reader_start(reader) == 0);
    wait_completions(36);
    CHECK(bs_reader_take(reader, &bytes, &length, 1000) == 0);
    CHECK(bytes[0] == 1 && length == 16);
    CHECK(bs_reader_take(reader, &bytes, &length, 1000) == BIRRIASDR_ERROR_IO);
    bs_reader_destroy(reader);
    reset_mock(0, 0);
}

/* Clean shutdown, consumer timeout, partial startup and resubmit failure must
 * all leave zero in-flight transfers and exactly one free per allocation.
 */
int main(void)
{
    birriasdr_dev_t dev = {0};
    struct bs_reader *reader;
    const uint8_t *bytes;
    size_t length;
#ifdef TEST_WRAP_ALLOCATIONS
    for (unsigned i = 1; i <= 12; ++i) {
        reset_mock(0, 0);
        allocation_call = 0;
        allocation_failure = i;
        CHECK(bs_reader_create(&dev, 4, &reader) == BIRRIASDR_ERROR_NO_MEMORY);
        allocation_failure = 0;
        CHECK(reader == NULL && mock.allocated == mock.freed);
    }
#endif
    for (unsigned i = 1; i <= 4; ++i) {
        reset_mock(0, 0);
        mock.fail_allocation = i;
        CHECK(bs_reader_create(&dev, 4, &reader) == BIRRIASDR_ERROR_NO_MEMORY);
        CHECK(reader == NULL && mock.allocated == mock.freed);
    }
    reset_mock(8, 0);
    CHECK(bs_reader_create(&dev, 4, &reader) == 0);
    CHECK(bs_reader_start(reader) == 0);
    for (unsigned i = 1; i <= 8; ++i) {
        CHECK(bs_reader_take(reader, &bytes, &length, 1000) == 0);
        CHECK(length == 16 && bytes[0] == i && bytes[15] == i);
    }
    CHECK(bs_reader_take(reader, &bytes, &length, 5) == BIRRIASDR_ERROR_TIMEOUT);
    bs_reader_destroy(reader);
    for (unsigned i = 1; i <= 4; ++i) {
        reset_mock(0, i);
        CHECK(bs_reader_create(&dev, 4, &reader) == 0);
        CHECK(bs_reader_start(reader) == BIRRIASDR_ERROR_IO);
        bs_reader_destroy(reader);
    }
    reset_mock(8, 5);
    CHECK(bs_reader_create(&dev, 4, &reader) == 0);
    CHECK(bs_reader_start(reader) == 0);
    CHECK(bs_reader_take(reader, &bytes, &length, 1000) == BIRRIASDR_ERROR_IO);
    bs_reader_destroy(reader);
    for (unsigned i = 0; i < 2; ++i) {
        reset_mock(1, 0);
        mock.status = i ? LIBUSB_TRANSFER_ERROR : LIBUSB_TRANSFER_NO_DEVICE;
        CHECK(bs_reader_create(&dev, 4, &reader) == 0);
        CHECK(bs_reader_start(reader) == 0);
        CHECK(bs_reader_take(reader, &bytes, &length, 1000) ==
              (i ? BIRRIASDR_ERROR_IO : BIRRIASDR_ERROR_NO_DEVICE));
        bs_reader_destroy(reader);
    }
    reset_mock(1000, 0);
    CHECK(bs_reader_create(&dev, 4, &reader) == 0);
    CHECK(bs_reader_start(reader) == 0);
    uint64_t deadline = bs_now_ns() + UINT64_C(1000000000);
    for (;;) {
        unsigned completed;
        pthread_mutex_lock(&mock.mutex);
        completed = mock.completions;
        pthread_mutex_unlock(&mock.mutex);
        if (completed > 32) break;
        CHECK(bs_now_ns() < deadline);
        bs_sleep_us(1000);
    }
    CHECK(bs_reader_take(reader, &bytes, &length, 1000) == BIRRIASDR_ERROR_OVERRUN);
    bs_reader_destroy(reader);
    reset_mock(0, 0);
    test_backpressure();
    puts("USB transfer ownership and failure tests passed");
    return 0;
}
