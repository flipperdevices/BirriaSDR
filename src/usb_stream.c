/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#include "usb_stream.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* Completions queued before the consumer is woken. Each wakeup costs about as
 * much as parsing a completion, and four are under 7 ms of samples at 10 MSps;
 * a shorter run is taken when the consumer's wait times out.
 */
#define BS_WAKE_COUNT 4u

#define BS_QUEUE_SLOTS ((32u << 20) / BS_USB_BYTES)

struct transfer_slot {
    struct bs_reader *reader;
    struct libusb_transfer *transfer;
    bool pending, dev_mem;
    struct transfer_slot *next;
};

struct bs_reader {
    birriasdr_dev_t *dev;
    pthread_mutex_t mutex;
    pthread_cond_t ready;
    pthread_t thread;
    bool started, stopping, backpressure, lent;
    int error;
    unsigned depth, inflight, capacity, head, tail, count;
    struct transfer_slot *slots;
    struct transfer_slot *held_head, *held_tail;
    uint8_t *queue;
    unsigned *lengths;
};

/* Cancel once with the queue lock held. Transfer memory remains alive until
 * cancellation callbacks have retired every submitted request.
 */
static void cancel_transfers(struct bs_reader *reader)
{
    if (reader->stopping)
        return;
    reader->stopping = true;
    for (unsigned i = 0; i < reader->depth; ++i)
        if (reader->slots[i].pending)
            libusb_cancel_transfer(reader->slots[i].transfer);
}

/* Preserve a completed transfer before reusing its buffer. The queue lock
 * protects both byte order and ownership, including parked completions.
 */
static void queue_transfer(struct bs_reader *reader, struct libusb_transfer *transfer)
{
    size_t length = (size_t)transfer->actual_length;
    memcpy(reader->queue + (size_t)reader->tail * BS_USB_BYTES, transfer->buffer, length);
    reader->lengths[reader->tail] = (unsigned)length;
    reader->tail = (reader->tail + 1) % reader->capacity;
    ++reader->count;
}

/* Submit only after completed bytes have a stable queue slot. Submission
 * failure leaves ownership with us and is still a real transport error.
 */
static void submit_slot(struct transfer_slot *slot)
{
    struct bs_reader *reader = slot->reader;
    int result = libusb_submit_transfer(slot->transfer);
    if (result < 0)
        reader->error = bs_usb_error(result);
    else {
        slot->pending = true;
        ++reader->inflight;
    }
}

/* Completion callbacks only copy USB bytes into a bounded queue and re-submit.
 * RF control and application callbacks run outside libusb callbacks instead,
 * avoiding libusb event-lock recursion and preventing Python-style scheduling
 * gaps between individual USB reads. Strict reads fail on a full queue. With
 * local recovery enabled, park completed transfers in arrival order instead.
 * Their buffers remain owned and unchanged until the consumer frees capacity.
 * Stopping resubmission propagates backpressure to the device, whose ordered
 * gap marker then describes lost source samples without losing framing here.
 */
static void LIBUSB_CALL receive_transfer(struct libusb_transfer *transfer)
{
    struct transfer_slot *slot = transfer->user_data;
    struct bs_reader *reader = slot->reader;
    pthread_mutex_lock(&reader->mutex);
    slot->pending = false;
    --reader->inflight;
    bool held = false, wake = true;
    if (!reader->stopping && !reader->error) {
        if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
            reader->error = transfer->status == LIBUSB_TRANSFER_NO_DEVICE ?
                BIRRIASDR_ERROR_NO_DEVICE : BIRRIASDR_ERROR_IO;
        } else if (transfer->actual_length > 0) {
            if (reader->held_head || reader->count == reader->capacity) {
                if (reader->backpressure) {
                    slot->next = NULL;
                    if (reader->held_tail)
                        reader->held_tail->next = slot;
                    else
                        reader->held_head = slot;
                    reader->held_tail = slot;
                    held = true;
                } else {
                    reader->error = BIRRIASDR_ERROR_OVERRUN;
                }
            } else {
                queue_transfer(reader, transfer);
                wake = reader->count >= BS_WAKE_COUNT;
            }
        }
        if (!reader->error && !held)
            submit_slot(slot);
    }
    if (wake || reader->error)
        pthread_cond_broadcast(&reader->ready);
    pthread_mutex_unlock(&reader->mutex);
}

/* Resume oldest completions first, without allocating or discarding bytes.
 * The total bound stays queue capacity plus the already allocated transfers.
 * Call with the queue lock held, from the event pump or the consuming thread.
 */
static void resume_transfers(struct bs_reader *reader)
{
    while (!reader->stopping && !reader->error && reader->held_head &&
           reader->count < reader->capacity) {
        struct transfer_slot *slot = reader->held_head;
        reader->held_head = slot->next;
        if (!reader->held_head)
            reader->held_tail = NULL;
        slot->next = NULL;
        queue_transfer(reader, slot->transfer);
        submit_slot(slot);
    }
}

/* One event thread belongs to one handle/context. It keeps processing events
 * during shutdown until cancellation callbacks finish; freeing early is unsafe.
 */
static void *pump_events(void *context)
{
    struct bs_reader *reader = context;
    bs_pin_thread(reader->dev);
    for (;;) {
        bool done;
        int result;
        struct timeval timeout = {0, 10000};
        pthread_mutex_lock(&reader->mutex);
        resume_transfers(reader);
        if (reader->error)
            cancel_transfers(reader);
        done = reader->stopping && reader->inflight == 0;
        pthread_mutex_unlock(&reader->mutex);
        if (done)
            return NULL;
        result = libusb_handle_events_timeout_completed(reader->dev->usb, &timeout, NULL);
        if (result < 0 && result != LIBUSB_ERROR_INTERRUPTED) {
            pthread_mutex_lock(&reader->mutex);
            if (!reader->error)
                reader->error = bs_usb_error(result);
            pthread_cond_broadcast(&reader->ready);
            pthread_mutex_unlock(&reader->mutex);
        }
    }
}

/* Allocate and touch every queue page before starting RF. The queue holds
 * eight completions per transfer, at most 32 MiB; its bound prevents unbounded
 * memory growth behind a slow callback.
 */
int bs_reader_create(birriasdr_dev_t *dev, unsigned depth, struct bs_reader **out)
{
    struct bs_reader *reader = calloc(1, sizeof(*reader));
    *out = NULL;
    if (!reader)
        return BIRRIASDR_ERROR_NO_MEMORY;
    reader->dev = dev;
    reader->backpressure = dev->stream_continuous && dev->recovery_limit &&
                           dev->recovery_callback;
    reader->depth = depth;
    reader->capacity = depth * 8 < BS_QUEUE_SLOTS ? depth * 8 : BS_QUEUE_SLOTS;
    if (pthread_mutex_init(&reader->mutex, NULL)) {
        free(reader);
        return BIRRIASDR_ERROR_IO;
    }
    if (pthread_cond_init(&reader->ready, NULL)) {
        pthread_mutex_destroy(&reader->mutex);
        free(reader);
        return BIRRIASDR_ERROR_IO;
    }
    reader->slots = calloc(depth, sizeof(*reader->slots));
    reader->queue = malloc((size_t)reader->capacity * BS_USB_BYTES);
    reader->lengths = calloc(reader->capacity, sizeof(*reader->lengths));
    if (!reader->slots || !reader->queue || !reader->lengths)
        goto allocation_failed;
    memset(reader->queue, 0, (size_t)reader->capacity * BS_USB_BYTES);
    for (unsigned i = 0; i < depth; ++i) {
        struct transfer_slot *slot = &reader->slots[i];
        uint8_t *buffer;
        slot->reader = reader;
        slot->transfer = libusb_alloc_transfer(0);
        if (!slot->transfer)
            goto allocation_failed;
        /* Memory usbfs maps for DMA spares the kernel a bounce buffer and a copy
         * per transfer. The heap serves where it is unavailable. */
        buffer = dev->handle ? libusb_dev_mem_alloc(dev->handle, BS_USB_BYTES) : NULL;
        slot->dev_mem = buffer != NULL;
        if (!buffer)
            buffer = malloc(BS_USB_BYTES);
        if (!buffer)
            goto allocation_failed;
        memset(buffer, 0, BS_USB_BYTES);
        libusb_fill_bulk_transfer(slot->transfer, dev->handle, dev->ep_in, buffer,
                                  BS_USB_BYTES, receive_transfer, slot, 0);
    }
    *out = reader;
    return 0;
allocation_failed:
    bs_reader_destroy(reader);
    return BIRRIASDR_ERROR_NO_MEMORY;
}

/* Queue all transfers before the MCU GO write. On partial submission failure,
 * still run/drain the event pump so libusb retains no pointers into freed memory.
 */
int bs_reader_start(struct bs_reader *reader)
{
    int result = 0;
    for (unsigned i = 0; i < reader->depth; ++i) {
        result = libusb_submit_transfer(reader->slots[i].transfer);
        if (result < 0)
            break;
        reader->slots[i].pending = true;
        ++reader->inflight;
    }
    if (result < 0)
        reader->error = bs_usb_error(result);
    if (pthread_create(&reader->thread, NULL, pump_events, reader)) {
        reader->error = BIRRIASDR_ERROR_IO;
        cancel_transfers(reader);
        pump_events(reader);
        return reader->error;
    }
    reader->started = true;
    return result < 0 ? bs_usb_error(result) : 0;
}

/* Lend the oldest completed transfer in place. Its queue slot stays occupied,
 * so USB callbacks cannot overwrite it, until the next take or destroy.
 * CLOCK_REALTIME matches POSIX's default condition clock, including on macOS.
 */
int bs_reader_take(struct bs_reader *reader, const uint8_t **data, size_t *length,
                   unsigned timeout_ms)
{
    struct timespec deadline;
    int result = 0;
    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_nsec += (long)timeout_ms * 1000000;
    deadline.tv_sec += deadline.tv_nsec / 1000000000;
    deadline.tv_nsec %= 1000000000;
    *data = NULL;
    *length = 0;
    pthread_mutex_lock(&reader->mutex);
    if (reader->lent) {
        reader->lent = false;
        reader->head = (reader->head + 1) % reader->capacity;
        --reader->count;
        resume_transfers(reader);
    }
    while (!reader->count && !reader->error) {
        int wait = pthread_cond_timedwait(&reader->ready, &reader->mutex, &deadline);
        if (wait == ETIMEDOUT) {
            if (!reader->count)
                result = BIRRIASDR_ERROR_TIMEOUT;
            break;
        }
        if (wait) {
            result = BIRRIASDR_ERROR_IO;
            break;
        }
    }
    if (reader->error)
        result = reader->error;
    if (!result && reader->count) {
        *data = reader->queue + (size_t)reader->head * BS_USB_BYTES;
        *length = reader->lengths[reader->head];
        reader->lent = true;
    }
    pthread_mutex_unlock(&reader->mutex);
    return result;
}

/* Drain cancellations, join, then free. Also handles pre-start allocation errors. */
void bs_reader_destroy(struct bs_reader *reader)
{
    if (!reader)
        return;
    if (reader->started) {
        pthread_mutex_lock(&reader->mutex);
        cancel_transfers(reader);
        pthread_mutex_unlock(&reader->mutex);
        pthread_join(reader->thread, NULL);
    }
    if (reader->slots) {
        for (unsigned i = 0; i < reader->depth; ++i) {
            struct libusb_transfer *transfer = reader->slots[i].transfer;
            if (transfer) {
                if (reader->slots[i].dev_mem)
                    libusb_dev_mem_free(reader->dev->handle, transfer->buffer, BS_USB_BYTES);
                else
                    free(transfer->buffer);
                libusb_free_transfer(transfer);
            }
        }
    }
    free(reader->slots);
    free(reader->queue);
    free(reader->lengths);
    pthread_cond_destroy(&reader->ready);
    pthread_mutex_destroy(&reader->mutex);
    free(reader);
}
