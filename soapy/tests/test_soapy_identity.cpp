/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
/* The serial follows the hostname script: device tree serial, else the OTP CPU
 * ID through U-Boot's CRC, else the machine ID, in fake filesystem trees. */
#include "../SoapyBirria.hpp"
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>

#define CHECK(test) do { if (!(test)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #test); exit(1); } } while (0)

static void put(const std::filesystem::path &path, const std::string &bytes)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << bytes;
}

int main(int argc, char **argv)
{
    CHECK(argc == 2);
    const std::filesystem::path root = argv[1];
    std::filesystem::remove_all(root);
    CHECK(birriaFlipperSerial(root.string()).empty());
    CHECK(!birriaIsFlipperOne(root.string()));
    put(root / "sys/firmware/devicetree/base/compatible",
        std::string("flipper,one-rev-f0b1c2\0rockchip,rk3576\0", 39));
    put(root / "etc/machine-id", "9b32d73fcfeb4d5aa2f87e3651d16222\n");
    CHECK(birriaFlipperSerial(root.string()) == "flipperone-9b32d7");
    CHECK(birriaIsFlipperOne(root.string()));
    /* A Flipper One's CPU ID, whose U-Boot serial is 45d2488f32838acd. */
    std::string otp(256, '\0');
    const unsigned char cpuid[16] = {0x4e, 0x59, 0x31, 0x37, 0x48, 0, 0, 0,
                                     0, 0, 0, 0, 0, 0x16, 0x14, 0x0c};
    otp.replace(0x0a, 16, reinterpret_cast<const char *>(cpuid), 16);
    put(root / "sys/bus/nvmem/devices/rockchip-otp0/nvmem", otp);
    CHECK(birriaFlipperSerial(root.string()) == "flipperone-45d248");
    put(root / "sys/firmware/devicetree/base/serial-number", std::string("c0ffee1234\0", 11));
    CHECK(birriaFlipperSerial(root.string()) == "flipperone-c0ffee");
    put(root / "sys/firmware/devicetree/base/compatible", std::string("rockchip,rk3576\0", 16));
    CHECK(birriaFlipperSerial(root.string()) == "rk3576-c0ffee");
    CHECK(!birriaIsFlipperOne(root.string()));
    std::filesystem::remove_all(root);
    puts("Flipper serial tests passed");
    return 0;
}
