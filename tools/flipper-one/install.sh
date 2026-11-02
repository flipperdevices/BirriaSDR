#!/bin/sh
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 The BirriaSDR Authors
# Install the prebuilt release archive this script came in: the runtime packages it needs,
# then everything under /usr/local.
set -e
[ "$(id -u)" = 0 ] || { echo "run with sudo" >&2; exit 1; }
[ "$(uname -m)" = aarch64 ] || { echo "this build is for arm64" >&2; exit 1; }
cd "$(dirname "$(readlink -f "$0")")"
[ -d usr/local ] || { echo "run it from the unpacked archive" >&2; exit 1; }
apt-get update
apt-get install -y --no-install-recommends libusb-1.0-0 libsoapysdr0.8 soapysdr-tools acl
# Replace rather than overwrite, so a running program keeps its old copy of the library.
cp -R --remove-destination usr/local/. /usr/local/
ldconfig
echo "installed: take the chip with sudo mt7921-sdr-hold"
