# SoapySDR module

The `soapy/` directory holds a SoapySDR module (`driver=birria`) so any
SoapySDR application can use a BirriaSDR receiver. The module wraps the
library's RX path. It offers only the rates the hardware produces, and it
converts samples to the requested format without resampling.

## What the module provides

| Item | Value |
| --- | --- |
| Driver key | `driver=birria`. A dongle is selected by enumeration order with `index=N` |
| Serial | On a Flipper One, the onboard chip's `serial` is the Flipper's own name, as its hostname is set at first boot, such as `flipperone-45d248`: from the device tree's serial number, else the SoC's, else the machine ID. Elsewhere, and with more than one dongle, there is none |
| Sample rates | 10, 20, 40 and 80 MSps, the native rates, and 5, 2.5, 1.25 and 0.625 MSps, which the library decimates from 10 MSps with half-band filters: passband the inner 80 percent of the span, stopband 92.8 dB down. The module does not resample to other rates |
| Antennas | `RX0` and `RX1`, receive chains 0 and 1, each with its own antenna connector, and `AUTO`, which captures the stronger chain, compared at the start of a stream and after each retune. Changing it restarts a running stream, as for a rate change |
| Bandwidth | The receive filters the current rate can use: 40 or 80 MHz at 80 MSps, 20 or 40 MHz at 40 MSps, and the span's own at 10 and 20 MSps. Below 10 MSps it is the 10 MSps filter's, and the decimation filters set the passband. A request gets the smallest at least as wide; a running stream restarts, as for a rate change |
| Frequency correction | In ppm, rounded to whole ppm, through `birriasdr_set_freq_correction()`. It retunes at once and does not correct the sample clock |
| Stream formats | `CS16` (native, raw), `CF32`, and `CS8` and `CU8` for tools that expect 8 bits, raw like `CS16` and louder: rounded from a quarter of the full scale, 3 dB more for each halving below 10 MSps, and clipped, so the noise floor the AGC leaves spans a few steps at every rate instead of rounding to zero; `CU8` is offset by 128 as RTL-SDR tools expect. `CS16` full scale is 8192 at the native rates and 32768 below 10 MSps, where decimation keeps two more bits; `getNativeStreamFormat()` reports it and `CF32` is scaled to match |
| Time | Each read carries `SOAPY_SDR_HAS_TIME` and the time of its first sample, on the steady clock that `getHardwareTime()` reads. Within a stream the times follow the sample count exactly. After a start or a device gap, which loses an unknown number of samples, they are anchored again to the arrival time, so a gap shows as a jump |
| Settings | `usb_burst`, the bulk endpoint burst, 1 to 15, or 0 for `BIRRIA_BURST`, else the device's; above 1 it needs the patched descriptor that `mt7921-sdr-hold` sets up. `cpus`, the CPUs the library's stream threads run on, such as `4-7`; empty leaves them unpinned and `default` follows `BIRRIA_CPUS`. A running stream restarts to apply either |
| Stream arguments | `buffer_samples`, the samples per buffer handed to the application, a multiple of 2048 up to 262144, default 16384, and `transfers`, the USB transfers kept queued, 4 to 256, default the library's 64. On Linux the library lowers it to what the kernel's `usbfs_memory_mb` allows, about 119 at its default 16 MiB. Larger buffers cost less CPU and add latency. An invalid value fails `setupStream()` |
| Stream MTU | `buffer_samples`. Applications such as CubicSDR and SoapyRemote read at most the MTU per call |
| DC offset mode | Automatic (default) or off. Automatic takes the library's DC estimate (`birriasdr_get_dc_offset()`), the digital gain stage's truncation, out of `CF32`. `CS16` is always raw |
| Frequency range | One range per band, from `birriasdr_get_tuning_band()`, so front ends do not offer the gap between bands. The bands match the frequency tuning row of the README [specifications](../README.md#specifications) |
| Gains | `LNA`, `LPF` and `DIG` in dB from the library's measured table for the current band; a value snaps to the nearest step of its stage. The overall gain fills the LNA first, then `LPF`, then `DIG`, as `birriasdr_set_tuner_gain()` does. In a band without a table the elements are raw codes: `LNA` 0 to 5, `LPF` 0 to 8 and `DIG` 0 to 12 |
| Gain mode | Automatic (the library default) or manual. Automatic mode runs the library's AGC, and the gain elements then show its current choice. Gains set in automatic mode are stored and sent once manual mode is selected. In manual mode a refused change keeps the previous code |

Each frequency range is a command interval from the library, not measured RF
coverage. A band that the board cannot reach is left out. The sample rate can
change while streaming: the module stops the read, changes the rate and starts
it again. A read in progress returns the samples it already has, and a read that
starts during the change waits for it, up to its timeout, so no read mixes two
rates. The next read returns `SOAPY_SDR_OVERFLOW`, and samples at the old rate
are dropped. If the read cannot restart, later reads return
`SOAPY_SDR_STREAM_ERROR`. A request within 1 Hz of a listed rate is set to
exactly that rate, and any other rate is refused. The module has no sensors.

Errors from tuning, sample-rate changes and gain changes are logged, not
thrown, because some front ends do not catch exceptions from these calls and
would abort. A center outside the tuning bands leaves the receiver where it
was, and reads give zeros until a center inside them is set, because front ends
such as SDR++ do not limit tuning to the reported ranges and would show the old
center's samples under the new label.

A retune takes tens of milliseconds. Samples still queued from the old center
are dropped when it completes, so an application that hops, such as a scanner,
reads the new center straight after it. Only the few milliseconds the device and
USB still hold remain from before the retune. The drop is not reported as a gap,
since the application asked for the retune.

The module enables the library's automatic recovery, so a stream keeps running
across overruns. Each gap reaches the application as `SOAPY_SDR_OVERFLOW` from
the read call, and the samples that follow are not continuous with the ones
before it. The number of missing samples is unknown. A read returns the samples
from before a gap first, and the next read reports the gap.

Clean overruns recover without limit. An overrun with a failed reset or DMA
error, a USB I/O error, a timeout or a failed live control update gets up to
three restarts per stream. Other errors, such as a lost device or corrupted
packets, end the stream at once. When the stream ends, reads return
`SOAPY_SDR_STREAM_ERROR` after the remaining samples and the module logs the
cause. Before activation and after deactivation, reads wait for the timeout and
return `SOAPY_SDR_TIMEOUT`. One stream can be open at a time. Activation and
deactivation accept no flags, so timed starts and stops, triggers and finite
bursts return `SOAPY_SDR_NOT_SUPPORTED`.

## Build

The module builds as part of the normal BirriaSDR build whenever SoapySDR is
installed. No extra flag is needed:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The build links the module against the in-tree `libbirriasdr`, so no separate
library install is required. The module is written to `build/soapy/`. If
SoapySDR is not installed, CMake prints a note and skips the module. Pass
`-DBIRRIASDR_SOAPY=OFF` to skip it on purpose.

Check that SoapySDR finds the dongle:

```sh
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy SoapySDRUtil --find=driver=birria
```

Every command below sets `SOAPY_SDR_PLUGIN_PATH`. To avoid that, install the
module into the system module directory:

```sh
sudo cmake --install build
```

## USB access without root

Install the udev rule as the [build steps](../README.md#build-and-try)
describe, so the `plugdev` group can open the dongle without `sudo`.

The `mt7921u` kernel driver must not be bound to the dongle. A run under
`sudo` can leave root-owned lock files in `/tmp`, and a later unprivileged run
cannot open the device until those are removed. Only one program can open the
dongle at a time, so stop one client before you start another.

## Using the module from an application

A SoapySDR application loads the module when it starts. Set
`SOAPY_SDR_PLUGIN_PATH` when you launch it, or install the module as the build
steps describe. The application must use a SoapySDR library with the same ABI
version as the one the module builds against, such as `0.8`, because SoapySDR
loads only modules with a matching ABI. Patch releases keep the ABI.

```sh
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy <application>
```

Select the `driver=birria` device in the application's device list. Turn
automatic gain off to use the overall gain or the `LNA`, `LPF` and `DIG`
controls, all in dB. Automatic gain is a reasonable start. The sample
rate sets the span, from 10 MHz at 10 MSps to 80 MHz at 80 MSps. Enter
frequencies in Hz, for example `2437000000` for Wi-Fi channel 6. General-purpose
SDR applications cannot demodulate Wi-Fi, so use their spectrum, waterfall or
raw I/Q views.

An application that serves the stream over a network needs a link that carries
the stream. Each complex sample takes 4 bytes in `CS16` and 8 bytes in
`CF32`. In `CS16`, 10 MSps needs 40 MB/s and 20 MSps needs 80 MB/s, which fit a
one gigabit link. The 40 and 80 MSps rates need 160 and 320 MB/s, so decimate
before sending, or use a faster link.

## Manual stream test

The `soapy/tests/test_soapy_stream.cpp` program captures through the module and
reports the averaged spectrum peak, to check the frequency axis and the
sample format.
It builds with the module when testing is enabled, but CTest does not run it
because it needs a dongle:

```sh
SOAPY_SDR_PLUGIN_PATH=$PWD/build/soapy build/soapy/test_soapy_stream <freq_hz> <rate> <seconds> [out.cf32]
```

To reproduce an application's problem, replay its call sequence against the
module in the same order: gain mode, gains, frequency, sample rate, then the
stream.
