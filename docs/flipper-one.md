# Flipper One

The Flipper One's onboard Wi-Fi and Bluetooth chip is an MT7921 (`0e8d:7961`),
and it works as the receiver. It tunes 2190 to 2610 MHz and 4500 to 6200 MHz at
10 to 80 MSps. While the SDR holds the chip, Wi-Fi and Bluetooth are off.

## Install

Download `flipperone-wifi-sdr-arm64.tar.gz` from the
[Releases](https://github.com/flipperdevices/BirriaSDR/releases) page onto
the Flipper, then:

```sh
tar -xzf flipperone-wifi-sdr-arm64.tar.gz
sudo flipperone-wifi-sdr-arm64/install.sh
```

It installs the runtime packages it needs and puts everything under `/usr/local`.
To build from source instead, see [Build and try](../README.md#build-and-try).

## Take the chip and give it back

```sh
sudo mt7921-sdr-hold
sudo mt7921-sdr-release
```

`mt7921-sdr-hold` keeps the Wi-Fi and Bluetooth drivers off the chip, power
cycles it, patches it for 80 MSps and gives the user who ran `sudo` access to it.
Airplane mode must be off. Nothing is permanent: a reboot gives the chip back to
Wi-Fi, so after one, run `mt7921-sdr-hold` again.

`mt7921-sdr-release` refuses while an SDR application is running. Close it first.

## Receive

Pin the library's threads to the fast A72 cores:

```sh
export BIRRIA_CPUS=4-7
birria_rx capture.cs16 40000000
SoapySDRUtil --find="driver=birria"
```

In SDR++ (`sudo apt install sdrpp`), add a `soapy_source` instance in the Module
Manager, then choose the device in the source menu. The
[SoapySDR module guide](soapy.md) lists its settings.
