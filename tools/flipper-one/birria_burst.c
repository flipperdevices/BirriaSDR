/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 * Make the Flipper One's onboard MT7921 advertise SuperSpeed burst 3 on endpoint 0x84.
 * The ROM hard-codes burst 0 but serves the configuration descriptor from RAM, so patch
 * the companion's bMaxBurst there and reset the port; the host re-reads the descriptor.
 * Volatile: a power cycle undoes it. The ROM programs endpoint 0x84 for burst 3 from the
 * patched descriptor as the device re-enumerates, so no library setting is needed.
 */
#include <libusb.h>
#include <stdint.h>
#include <stdio.h>

#define DESCRIPTOR_BURST 0x7c0531c8u

int main(void)
{
    libusb_context *ctx;
    libusb_device_handle *handle;
    const uint8_t value[4] = {3, 0, 0, 0};
    int result;

    if (libusb_init(&ctx))
        return 1;
    handle = libusb_open_device_with_vid_pid(ctx, 0x0e8d, 0x7961);
    if (!handle) {
        fprintf(stderr, "birria_burst: no 0e8d:7961\n");
        libusb_exit(ctx);
        return 1;
    }
    result = libusb_control_transfer(handle, 0x5f, 0x66, DESCRIPTOR_BURST >> 16,
                                     DESCRIPTOR_BURST & 0xffff, (uint8_t *)value, 4, 500);
    if (result != 4) {
        fprintf(stderr, "birria_burst: descriptor write: %s\n", libusb_error_name(result));
    } else {
        /* The device re-enumerates with the new descriptor, so this handle goes stale. */
        result = libusb_reset_device(handle);
        if (result && result != LIBUSB_ERROR_NOT_FOUND)
            fprintf(stderr, "birria_burst: reset: %s\n", libusb_error_name(result));
        else
            result = 0;
    }
    libusb_close(handle);
    libusb_exit(ctx);
    return result ? 1 : 0;
}
