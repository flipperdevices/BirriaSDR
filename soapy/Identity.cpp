/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
#include "SoapyBirria.hpp"
#include <cstdio>
#include <fstream>
#include <iterator>
#include <sstream>

static std::string readFile(const std::string &path)
{
    std::ifstream file(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

/* U-Boot's CRC32 without the final complement, continuing from crc. */
static uint32_t crc32NoComp(uint32_t crc, const std::string &bytes)
{
    for (unsigned char b : bytes) {
        crc ^= b;
        for (int i = 0; i < 8; ++i)
            crc = crc & 1 ? (crc >> 1) ^ 0xedb88320u : crc >> 1;
    }
    return crc;
}

/* The RK3576 serial U-Boot reports as serial#: CRCs of the odd and then the even
 * bytes of the 16-byte CPU ID at OTP offset 0x0a, printed high word first.
 */
static std::string otpSerial(const std::string &root)
{
    const std::string otp = readFile(root + "/sys/bus/nvmem/devices/rockchip-otp0/nvmem");
    if (otp.size() < 0x0a + 16)
        return "";
    std::string high, low;
    for (size_t i = 0; i < 16; ++i)
        (i % 2 ? low : high) += otp[0x0a + i];
    const uint32_t lo = crc32NoComp(0, low);
    const uint32_t hi = crc32NoComp(lo, high);
    char text[17];
    snprintf(text, sizeof(text), "%08x%08x", hi, lo);
    return text;
}

/* The name the Flipper's hostname is set to at first boot: the board's device
 * name without its revision, a dash, and the first six characters of the serial
 * from the device tree, else from the OTP, else the machine ID. Empty without a
 * device tree, so on other machines there is no serial.
 */
std::string birriaFlipperSerial(const std::string &root)
{
    std::istringstream compatible(readFile(root + "/sys/firmware/devicetree/base/compatible"));
    if (!compatible.rdbuf()->in_avail())
        return "";
    std::string entry, device = "rk3576";
    while (std::getline(compatible, entry, '\0')) {
        const size_t comma = entry.find(',');
        const size_t start = comma == std::string::npos ? entry.size() : comma + 1;
        if (entry.substr(start, entry.find(',', start) - start) == "rk3576")
            continue;
        device = entry.substr(0, entry.find("-rev-"));
        const size_t first = device.find(',');
        if (first != std::string::npos)
            device.erase(first, 1);
        break;
    }
    std::string serial = readFile(root + "/sys/firmware/devicetree/base/serial-number");
    serial = serial.substr(0, serial.find('\0'));
    if (serial.empty())
        serial = otpSerial(root);
    if (serial.empty()) {
        serial = readFile(root + "/etc/machine-id");
        serial = serial.substr(0, serial.find('\n'));
    }
    return serial.empty() ? "" : device + "-" + serial.substr(0, 6);
}

bool birriaIsFlipperOne(const std::string &root)
{
    std::istringstream compatible(readFile(root + "/sys/firmware/devicetree/base/compatible"));
    std::string entry;
    while (std::getline(compatible, entry, '\0'))
        if (entry == "flipper,one" || entry.rfind("flipper,one-", 0) == 0)
            return true;
    return false;
}
