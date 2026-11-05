/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Luigi C.
 */
#ifndef BIRRIASDR_USB_STREAM_H
#define BIRRIASDR_USB_STREAM_H
#include "internal.h"
/* A whole 128 KiB aggregate, 133,120 bytes, and the four bytes it gains as a
 * whole number of 1,024-byte packets, rounded up to a packet. */
#define BS_USB_BYTES 139264u
struct bs_reader;
unsigned bs_usb_depth_limit(unsigned memory_mb);
unsigned bs_usbfs_memory_mb(void);
int bs_reader_create(birriasdr_dev_t *dev, unsigned depth, struct bs_reader **out);
int bs_reader_start(struct bs_reader *reader);
int bs_reader_take(struct bs_reader *reader, const uint8_t **data, size_t *length,
                   unsigned timeout_ms);
void bs_reader_destroy(struct bs_reader *reader);
#endif
