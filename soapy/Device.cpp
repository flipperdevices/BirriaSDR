/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 The BirriaSDR Authors
 */
#include "SoapyBirria.hpp"
#include <SoapySDR/Logger.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <shared_mutex>
#include <stdexcept>

/* Construction has no front end to report to, so a failed open throws. */
static void check(int result, const char *what)
{
    if (result != 0)
        throw std::runtime_error(std::string("birriasdr: ") + what + " failed (" +
                                 std::to_string(result) + ")");
}

/* Test builds can pause a read between taking a chunk and copying it, and make
 * a worker fail to start, to exercise the rate-change boundary. Other builds
 * compile these to nothing.
 */
#ifdef SOAPY_BIRRIA_TEST_HOOKS
void soapy_birria_read_hook();
bool soapy_birria_fail_worker_start();
#else
static inline void soapy_birria_read_hook() {}
static inline bool soapy_birria_fail_worker_start() { return false; }
#endif

/* Log an error as best effort. Logging can allocate, and it runs on failure
 * paths where an exception would terminate the process, so it never throws.
 */
static void logError(const char *message) noexcept
{
    try {
        SoapySDR::log(SOAPY_SDR_ERROR, message);
    } catch (...) {
    }
}

/* Setters called while an application runs log a refusal instead of throwing:
 * some front ends do not catch exceptions from these calls and abort.
 */
static bool report(int result, const char *what) noexcept
{
    if (result != 0) {
        try {
            SoapySDR::logf(SOAPY_SDR_ERROR, "birria: %s failed: %s (%d)", what,
                           birriasdr_strerror(result), result);
        } catch (...) {
        }
    }
    return result == 0;
}

/* Open the dongle at its enumeration index. Device arguments are not used.
 * Recovery is configured here because the library accepts it only while idle.
 * It keeps a stream running across overruns and reports each gap.
 */
SoapyBirria::SoapyBirria(uint32_t index, const SoapySDR::Kwargs &)
{
    check(birriasdr_open(&dev_, index), "open");
    const int rc = birriasdr_set_recovery(dev_, kMaxRestarts, &SoapyBirria::recoveryCallback, this);
    if (rc != 0) {
        birriasdr_close(dev_);
        check(rc, "set_recovery");
    }
    rate_ = birriasdr_get_sample_rate(dev_);
    SoapySDR::logf(SOAPY_SDR_INFO, "birria: opened device %u", index);
}

/* Stop any stream first: the library requires the reader to be joined before
 * close, and destroying a joinable thread terminates the process.
 */
SoapyBirria::~SoapyBirria()
{
    deactivateStream(nullptr);
    if (dev_)
        birriasdr_close(dev_);
}

/* Warn front ends that the gains are raw codes. */
SoapySDR::Kwargs SoapyBirria::getHardwareInfo() const
{
    SoapySDR::Kwargs info;
    info["note"] = "experimental; raw gain codes, not dB";
    return info;
}

/* Settings the library reads when a read starts. */
SoapySDR::ArgInfoList SoapyBirria::getSettingInfo() const
{
    SoapySDR::ArgInfo burst;
    burst.key = "usb_burst";
    burst.value = "0";
    burst.name = "USB burst";
    burst.description = "Bulk endpoint burst, 1 to 15, or 0 for BIRRIA_BURST, else the "
                        "device's. Above 1 it needs the patched descriptor that "
                        "mt7921-sdr-hold sets up on a Flipper One.";
    burst.type = SoapySDR::ArgInfo::INT;
    burst.range = SoapySDR::Range(0, 15, 1);
    SoapySDR::ArgInfo cpus;
    cpus.key = "cpus";
    cpus.value = "default";
    cpus.name = "Stream CPUs";
    cpus.description = "CPUs the library's stream threads run on, such as 4-7. Empty "
                       "leaves them unpinned; default follows BIRRIA_CPUS.";
    cpus.type = SoapySDR::ArgInfo::STRING;
    SoapySDR::ArgInfo filter;
    filter.key = "channel_filter";
    filter.value = "true";
    filter.name = "Channel filter";
    filter.description = "At 20 and 40 Msps, and for two channels, take the samples after the "
                         "chip's channel filter, which keeps signals outside the span 40 dB or "
                         "more down instead of 4. Needs USB burst 3 or more (the patched "
                         "descriptor); without it 20 and 40 Msps use the stock path.";
    filter.type = SoapySDR::ArgInfo::BOOL;
    return {burst, cpus, filter};
}

/* A running stream restarts to apply a setting, as for a rate change. */
void SoapyBirria::writeSetting(const std::string &key, const std::string &value)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    if (key == "usb_burst") {
        size_t used = 0;
        unsigned long burst = 16;
        try {
            burst = std::stoul(value, &used);
        } catch (const std::exception &) {
        }
        if (used != value.size() || burst > 15) {
            logError("birria: usb_burst must be 0 to 15");
            return;
        }
        whileIdle([this, burst] {
            report(birriasdr_set_usb_burst(dev_, static_cast<unsigned>(burst)), "set_usb_burst");
        }, "birria: cannot restart the stream after a setting change");
    } else if (key == "channel_filter") {
        const bool on = value == "true" || value == "1";
        if (!on && value != "false" && value != "0") {
            logError("birria: channel_filter must be true or false");
            return;
        }
        whileIdle([this, on] {
            report(birriasdr_set_channel_filter(dev_, on ? 1 : 0), "set_channel_filter");
        }, "birria: cannot restart the stream after a setting change");
    } else if (key == "cpus") {
        whileIdle([this, value] {
            report(birriasdr_set_cpus(dev_, value == "default" ? nullptr : value.c_str()),
                   "set_cpus");
        }, "birria: cannot restart the stream after a setting change");
    } else {
        logError(("birria: unknown setting " + key).c_str());
    }
}

std::string SoapyBirria::readSetting(const std::string &key) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    if (key == "usb_burst")
        return std::to_string(birriasdr_get_usb_burst(dev_));
    if (key == "cpus")
        return birriasdr_get_cpus(dev_);
    if (key == "channel_filter")
        return birriasdr_get_channel_filter(dev_) ? "true" : "false";
    return "";
}

/* One RX channel per receive chain, and no TX path. Channel 0 alone follows the
 * antenna setting; channel 1 alone is chain 1; both stream the two chains,
 * sampled together from one LO.
 */
size_t SoapyBirria::getNumChannels(const int direction) const
{
    if (direction != SOAPY_SDR_RX)
        return 0;
    return birriasdr_get_capabilities(dev_) & BIRRIASDR_CAP_CHANNEL_FILTER ?
           birriasdr_get_antenna_count(dev_) : 1;
}

/* One antenna per receive chain, named RX0 and RX1 after the chain, and AUTO,
 * which captures the stronger chain, compared at the start of a read and after
 * each retune. The chain is set up when a read starts, so a running stream
 * restarts as for a rate change.
 */
std::vector<std::string> SoapyBirria::listAntennas(const int, const size_t) const
{
    std::vector<std::string> names;
    const unsigned count = birriasdr_get_antenna_count(dev_);
    for (unsigned i = 0; i < count; ++i)
        names.push_back("RX" + std::to_string(i));
    if (count > 1)
        names.push_back("AUTO");
    return names;
}

void SoapyBirria::setAntenna(const int, const size_t, const std::string &name)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    unsigned antenna;
    if (name == "AUTO") {
        antenna = BIRRIASDR_ANTENNA_AUTO;
        if (birriasdr_get_antenna_auto(dev_))
            return;
    } else if (name.size() == 3 && name.compare(0, 2, "RX") == 0 && name[2] >= '0' && name[2] <= '9') {
        antenna = static_cast<unsigned>(name[2] - '0');
        if (!birriasdr_get_antenna_auto(dev_) && antenna == birriasdr_get_antenna(dev_))
            return;
    } else {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: unknown antenna " + name);
        return;
    }
    whileIdle([this, antenna] {
        report(birriasdr_set_antenna(dev_, antenna), "set_antenna");
    }, "birria: cannot restart the stream after an antenna change");
}

std::string SoapyBirria::getAntenna(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    if (birriasdr_get_antenna_auto(dev_))
        return "AUTO";
    return "RX" + std::to_string(birriasdr_get_antenna(dev_));
}

/* The measured stage table of the current band, when the library has one. */
static bool gainTable(birriasdr_dev_t *dev, birriasdr_gain_table &table)
{
    return birriasdr_get_gain_table(dev, &table) == 0;
}

/* One stage's dB per code, in tenths, and how many codes it has. */
static const int16_t *stageGains(const birriasdr_gain_table &table, const std::string &name,
                                 size_t &count)
{
    if (name == "LNA") {
        count = table.lna_count;
        return table.lna;
    }
    if (name == "LPF") {
        count = table.lpf_count;
        return table.lpf;
    }
    if (name == "DIG") {
        count = table.digital_count;
        return table.digital;
    }
    throw std::runtime_error("birria: unknown gain element " + name);
}

/* The three stages, LNA first, which is also the order the total fills them. */
std::vector<std::string> SoapyBirria::listGains(const int, const size_t) const
{
    return {"LNA", "LPF", "DIG"};
}

/* In dB from the measured table of the current band, as continuous ranges that
 * setGain snaps to the nearest step. Without a table, raw codes in steps of one:
 * LNA 0 to 5, LPF 0 to 13, DIG 0 to 12.
 */
SoapySDR::Range SoapyBirria::getGainRange(const int, const size_t, const std::string &name) const
{
    birriasdr_gain_table table;
    size_t count;
    if (gainTable(dev_, table)) {
        const int16_t *gains = stageGains(table, name, count);
        return SoapySDR::Range(0, gains[count - 1] / 10.0);
    }
    if (name == "LNA")
        return SoapySDR::Range(0, 5, 1);
    if (name == "LPF")
        return SoapySDR::Range(0, 8, 1);
    if (name == "DIG")
        return SoapySDR::Range(0, 12, 1);
    throw std::runtime_error("birria: unknown gain element " + name);
}

/* The total in dB when the band has a table; otherwise the sum of raw ranges. */
SoapySDR::Range SoapyBirria::getGainRange(const int direction, const size_t channel) const
{
    birriasdr_gain_table table;
    if (!gainTable(dev_, table))
        return SoapySDR::Device::getGainRange(direction, channel);
    return SoapySDR::Range(0, (table.lna[table.lna_count - 1] + table.lpf[table.lpf_count - 1] +
                               table.digital[table.digital_count - 1]) / 10.0);
}

/* The library starts in automatic mode and accepts codes only in manual mode,
 * so codes set during AGC are kept and applied when AGC is turned off.
 */
bool SoapyBirria::hasGainMode(const int, const size_t) const
{
    return true;
}

/* Switch modes, then send the stored codes when entering manual mode. The
 * gain lock keeps the codes consistent with concurrent gain calls.
 */
void SoapyBirria::setGainMode(const int, const size_t, const bool automatic)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    std::lock_guard<std::mutex> lock(gainMutex_);
    if (report(birriasdr_set_tuner_gain_mode(dev_, automatic ? 0 : 1), "set_tuner_gain_mode") &&
        !automatic)
        report(birriasdr_set_gain_codes(dev_, &gain_), "set_gain_codes");
}

/* Automatic DC correction, on by default, applies to CF32: it subtracts the
 * library's estimate of the digital gain stage's offset. CS16 is always raw.
 */
bool SoapyBirria::hasDCOffsetMode(const int direction, const size_t) const
{
    return direction == SOAPY_SDR_RX;
}

void SoapyBirria::setDCOffsetMode(const int, const size_t, const bool automatic)
{
    dcAuto_.store(automatic);
}

bool SoapyBirria::getDCOffsetMode(const int, const size_t) const
{
    return dcAuto_.load();
}

/* Report automatic mode unless the library confirms manual mode. */
bool SoapyBirria::getGainMode(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    birriasdr_gain gain;
    int manual = 0;
    return birriasdr_get_gain_codes(dev_, &gain, &manual) != 0 || !manual;
}

/* With gainMutex_ held: in manual mode send the codes and keep them only if the
 * library accepts them, so a refused change does not look applied; in automatic
 * mode store them for when manual mode starts.
 */
void SoapyBirria::commitGainLocked(const birriasdr_gain &next, const bool manual)
{
    if (!manual || report(birriasdr_set_gain_codes(dev_, &next), "set_gain_codes"))
        gain_ = next;
}

/* Snap the request to one stage's code: the nearest measured dB, the lowest of
 * codes with equal gain; or, without a table, the rounded and clamped code. In
 * manual mode it replaces only its own stage of the library's committed codes,
 * so a stale stored code for another stage is never sent.
 */
void SoapyBirria::setGain(const int, const size_t, const std::string &name, const double value)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    birriasdr_gain_table table;
    uint8_t code = 0;
    if (gainTable(dev_, table)) {
        size_t count;
        const int16_t *gains = stageGains(table, name, count);
        const double tenths = value * 10.0;
        for (size_t i = 1; i < count; ++i)
            if (std::fabs(gains[i] - tenths) < std::fabs(gains[code] - tenths))
                code = static_cast<uint8_t>(i);
    } else {
        const auto range = getGainRange(SOAPY_SDR_RX, 0, name);
        code = static_cast<uint8_t>(
            std::lround(std::clamp(value, range.minimum(), range.maximum())));
    }
    std::lock_guard<std::mutex> lock(gainMutex_);
    birriasdr_gain committed;
    int manual = 0;
    const bool known = birriasdr_get_gain_codes(dev_, &committed, &manual) == 0;
    birriasdr_gain next = known && manual ? committed : gain_;
    if (name == "LNA")
        next.lna = code;
    else if (name == "LPF")
        next.lpf = code;
    else
        next.digital = code;
    commitGainLocked(next, known && manual);
}

/* A total in dB fills the LNA first, then the filter stage, then digital gain,
 * as the library's set_tuner_gain does. Without a table, Soapy's default spread.
 */
void SoapyBirria::setGain(const int direction, const size_t channel, const double value)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    birriasdr_gain next;
    if (birriasdr_tuner_gain_codes(dev_, static_cast<int>(std::lround(value * 10.0)), &next) != 0) {
        control.unlock();
        SoapySDR::Device::setGain(direction, channel, value);
        return;
    }
    std::lock_guard<std::mutex> lock(gainMutex_);
    birriasdr_gain committed;
    int manual = 0;
    const bool known = birriasdr_get_gain_codes(dev_, &committed, &manual) == 0;
    commitGainLocked(next, known && manual);
}

/* Return the stage's dB, or its raw code without a table: from the codes the
 * library committed, which in automatic mode are the AGC's current choice, or
 * the stored codes when the library cannot report. Soapy's total is the sum.
 */
double SoapyBirria::getGain(const int, const size_t, const std::string &name) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    std::lock_guard<std::mutex> lock(gainMutex_);
    birriasdr_gain gain = gain_;
    birriasdr_gain committed;
    birriasdr_gain_table table;
    int manual = 0;
    size_t count;
    if (birriasdr_get_gain_codes(dev_, &committed, &manual) == 0)
        gain = committed;
    const uint8_t code = name == "LNA" ? gain.lna : name == "LPF" ? gain.lpf : gain.digital;
    if (gainTable(dev_, table)) {
        const int16_t *gains = stageGains(table, name, count);
        return code < count ? gains[code] / 10.0 : 0.0;
    }
    if (name == "LNA" || name == "LPF" || name == "DIG")
        return code;
    throw std::runtime_error("birria: unknown gain element " + name);
}

/* Request a center in Hz. The library rejects one outside the tuning bands
 * without retuning. Applications such as SDR++ do not limit tuning to
 * getFrequencyRange() and would label the old center's samples with the new
 * one, so reads give zeros until a tunable center is set.
 */
void SoapyBirria::setFrequency(const int, const size_t, const std::string &, const double frequency,
                               const SoapySDR::Kwargs &)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    const int result = birriasdr_set_center_freq(dev_, static_cast<uint64_t>(std::llround(frequency)));
    if (result == BIRRIASDR_ERROR_UNSUPPORTED) {
        if (!offBand_.exchange(true)) {
            try {
                SoapySDR::logf(SOAPY_SDR_WARNING,
                               "birria: %.0f Hz is outside the tuning bands; zeros until a center inside them",
                               frequency);
            } catch (...) {
            }
        }
        return;
    }
    if (report(result, "set_center_freq")) {
        offBand_ = false;
        discardQueued();
    }
}

/* A retune takes tens of milliseconds, and an application that retunes from
 * its read thread, as GNU Radio does, reads nothing meanwhile. Samples queued
 * until now were taken at the old center, so drop them: later reads come from
 * the new center, apart from the chunk being read and the few milliseconds the
 * device and USB still hold. No gap is reported, since the application asked
 * for the discontinuity, and GNU Radio would print each OVERFLOW.
 */
void SoapyBirria::discardQueued()
{
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    queued_ = 0;
}

/* Return the committed center, not the frequency of samples already queued. */
double SoapyBirria::getFrequency(const int, const size_t, const std::string &) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    return static_cast<double>(birriasdr_get_center_freq(dev_));
}

/* The only tunable element is the RF center. */
std::vector<std::string> SoapyBirria::listFrequencies(const int, const size_t) const
{
    return {"RF"};
}

/* The reference crystal's error in ppm, positive when it runs fast, rounded to
 * whole ppm; the library retunes at once and keeps reporting the requested
 * center. The sample clock keeps its error.
 */
bool SoapyBirria::hasFrequencyCorrection(const int direction, const size_t) const
{
    return direction == SOAPY_SDR_RX;
}

void SoapyBirria::setFrequencyCorrection(const int, const size_t, const double value)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    report(birriasdr_set_freq_correction(dev_, static_cast<int>(std::lround(value))),
           "set_freq_correction");
}

double SoapyBirria::getFrequencyCorrection(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    return birriasdr_get_freq_correction(dev_);
}

/* Report one range per contiguous band, so front ends cannot offer the gaps
 * between them. A band this board cannot reach returns UNSUPPORTED and is
 * skipped. INVALID marks the end of the list.
 */
SoapySDR::RangeList SoapyBirria::getFrequencyRange(const int, const size_t,
                                                   const std::string &) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    SoapySDR::RangeList ranges;
    for (uint32_t index = 0;; ++index) {
        uint64_t minimum = 0, maximum = 0;
        const int rc = birriasdr_get_tuning_band(dev_, index, &minimum, &maximum);
        if (rc == BIRRIASDR_ERROR_INVALID)
            break;
        if (rc == BIRRIASDR_ERROR_UNSUPPORTED)
            continue;
        if (rc != 0) {
            report(rc, "get_tuning_band");
            break;
        }
        ranges.push_back(
            SoapySDR::Range(static_cast<double>(minimum), static_cast<double>(maximum)));
    }
    return ranges;
}

/* The library's rates: the hardware's native ones, and those below 10 MSps by a
 * power of two, which it decimates. The library cannot list its rates, so add
 * one here when the backend gains it.
 */
static const double kRates[] = {625e3, 1.25e6, 2.5e6, 5e6, 10e6, 20e6, 40e6, 80e6};

/* Accept only a listed rate and pass it to the library. The library requires
 * this setter to run alone on the handle, so it holds the control lock
 * exclusively, which also keeps a stream from starting meanwhile.
 */
void SoapyBirria::setSampleRate(const int, const size_t, const double rate)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    /* Match the request within 1 Hz of a listed rate and forward that exact
     * rate, so neither a huge value nor a fractional one reaches the library.
     */
    const double *native = std::find_if(std::begin(kRates), std::end(kRates),
                                        [rate](double n) { return std::fabs(rate - n) < 1.0; });
    if (native == std::end(kRates)) {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: the sample rate must be a listed rate");
        return;
    }
    if (static_cast<uint32_t>(*native) == static_cast<uint32_t>(rate_))
        return;
    whileIdle([this, native] {
        if (report(birriasdr_set_sample_rate(dev_, static_cast<uint32_t>(*native)),
                   "set_sample_rate"))
            rate_ = birriasdr_get_sample_rate(dev_);
    }, "birria: cannot restart the stream after a rate change");
}

/* With controlMutex_ held exclusively, make a change the library accepts only
 * while idle. A running stream stops, changes and restarts: wake a waiting read
 * so it returns, then take the reader lock, so the change falls between two
 * reads. Samples from before are dropped, and the next read returns OVERFLOW.
 * If the read cannot restart, the stream stays active with no worker, so reads
 * return STREAM_ERROR after the overflow. An active stream whose read already
 * ended restarts too, so queued old samples never pass for new ones.
 */
void SoapyBirria::whileIdle(const std::function<void()> &change, const char *restartFailed)
{
    const bool restart = active_;
    std::unique_lock<std::mutex> reading(readerMutex_, std::defer_lock);
    if (restart) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            restarting_ = true;
        }
        cond_.notify_all();
        reading.lock();
        stopWorker();
    }
    change();
    if (!restart)
        return;
    current_.iq.clear();
    current_.iq1.clear();
    offset_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        queued_ = 0;
        markGap();
    }
    if (startWorker() != 0)
        logError(restartFailed);
    {
        std::lock_guard<std::mutex> lock(mutex_);
        restarting_ = false;
    }
    cond_.notify_all();
}

/* The receive filter: the library's smallest width for this rate at least the
 * request. The filter is set up when a read starts, so a running stream
 * restarts as for a rate change.
 */
void SoapyBirria::setBandwidth(const int, const size_t, const double bandwidth)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    const uint32_t request = bandwidth <= 0 ? 0 : bandwidth >= 4e9 ? UINT32_MAX
                                                 : static_cast<uint32_t>(std::lround(bandwidth));
    whileIdle([this, request] {
        report(birriasdr_set_tuner_bandwidth(dev_, request), "set_tuner_bandwidth");
    }, "birria: cannot restart the stream after a bandwidth change");
}

double SoapyBirria::getBandwidth(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    return birriasdr_get_tuner_bandwidth(dev_);
}

/* The filter widths the current rate can use, from narrowest to the span. */
std::vector<double> SoapyBirria::listBandwidths(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    uint32_t widths[8];
    const int count = birriasdr_get_tuner_bandwidths(dev_, widths, 8);
    std::vector<double> list;
    for (int i = 0; i < count && i < 8; ++i)
        list.push_back(widths[i]);
    return list;
}

/* Return the rate last committed to the library, cached atomically so this
 * needs no library call.
 */
double SoapyBirria::getSampleRate(const int, const size_t) const
{
    return rate_;
}

/* List the rates from lowest to highest. */
std::vector<double> SoapyBirria::listSampleRates(const int, const size_t) const
{
    return std::vector<double>(std::begin(kRates), std::end(kRates));
}

/* The library delivers CS16. CF32 is a scaled copy, with the library's DC
 * estimate, the digital gain stage's truncation, taken out while automatic DC
 * correction is on. CS8 and CU8, CS8 offset by 128 as RTL-SDR tools expect,
 * are raw like CS16 and louder: 8 bits from a quarter of the full scale, plus
 * 3 dB for each halving below 10 MSps, clipped. From the full scale, the AGC's
 * noise floor would round to zero.
 */
std::vector<std::string> SoapyBirria::getStreamFormats(const int, const size_t) const
{
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16, SOAPY_SDR_CS8, SOAPY_SDR_CU8};
}

/* Report the native format and its full-scale value. */
std::string SoapyBirria::getNativeStreamFormat(const int, const size_t, double &fullScale) const
{
    /* Signed 14-bit samples carried in int16, or 16-bit below 10 MSps, where
     * the library's decimation keeps two more bits. */
    fullScale = birriasdr_get_full_scale(dev_);
    return SOAPY_SDR_CS16;
}

/* The library's read sizing, as stream arguments. */
SoapySDR::ArgInfoList SoapyBirria::getStreamArgsInfo(const int, const size_t) const
{
    SoapySDR::ArgInfo transfers;
    transfers.key = "transfers";
    transfers.value = "0";
    transfers.name = "USB transfers";
    transfers.description = "USB transfers kept queued, 4 to 256, lowered on Linux to what "
                            "usbfs_memory_mb allows. 0 is the library default, 64, or 256 on macOS.";
    transfers.type = SoapySDR::ArgInfo::INT;
    SoapySDR::ArgInfo samples;
    samples.key = "buffer_samples";
    samples.value = "16384";
    samples.name = "Buffer samples";
    samples.description = "Samples per buffer handed to the application, a multiple of 2048 "
                          "up to 262144. It is also the stream MTU.";
    samples.type = SoapySDR::ArgInfo::INT;
    return {transfers, samples};
}

/* One stream argument as an integer within its range, or the default when absent. */
static uint32_t streamArg(const SoapySDR::Kwargs &args, const char *key, uint32_t fallback,
                          uint32_t low, uint32_t high, uint32_t step)
{
    const auto found = args.find(key);
    if (found == args.end())
        return fallback;
    size_t used = 0;
    unsigned long value = 0;
    try {
        value = std::stoul(found->second, &used);
    } catch (const std::exception &) {
        used = 0;
    }
    if (!used || used != found->second.size() || value < low || value > high || value % step)
        throw std::runtime_error(std::string("birria: invalid stream argument ") + key + "=" +
                                 found->second);
    return static_cast<uint32_t>(value);
}

/* Accept one RX stream on channel 0 in a listed format. The device is the
 * stream handle, so a second stream would change the format under the first.
 */
SoapySDR::Stream *SoapyBirria::setupStream(const int direction, const std::string &format,
                                           const std::vector<size_t> &channels,
                                           const SoapySDR::Kwargs &args)
{
    static const std::pair<const char *, Format> formats[] = {
        {SOAPY_SDR_CF32, Format::CF32}, {SOAPY_SDR_CS16, Format::CS16},
        {SOAPY_SDR_CS8, Format::CS8}, {SOAPY_SDR_CU8, Format::CU8}};
    const auto chosen = std::find_if(std::begin(formats), std::end(formats),
                                     [&format](const auto &f) { return format == f.first; });
    if (direction != SOAPY_SDR_RX || chosen == std::end(formats))
        throw std::runtime_error("birria: only RX CF32, CS16, CS8 and CU8 streams are supported");
    const size_t count = getNumChannels(SOAPY_SDR_RX);
    const bool dual = channels.size() == 2 && channels[0] == 0 && channels[1] == 1 && count == 2;
    const bool second = channels.size() == 1 && channels[0] == 1 && count == 2;
    if (!dual && !second && (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0)))
        throw std::runtime_error("birria: channels {0}, {1} or {0, 1} only");
    if (streamOpen_)
        throw std::runtime_error("birria: close the open stream before setting up another");
    if (dual) {
        const double rate = getSampleRate(SOAPY_SDR_RX, 0);
        if (rate != 20e6 && rate != 40e6)
            throw std::runtime_error("birria: two channels stream at 20 or 40 Msps");
    }
    {
        /* Channel 0 alone keeps the antenna setting, AUTO included. */
        std::unique_lock<std::shared_mutex> control(controlMutex_);
        if (dual)
            report(birriasdr_set_channels(dev_, 3), "set_channels");
        else if (second)
            report(birriasdr_set_channels(dev_, 2), "set_channels");
        else if (birriasdr_get_channels(dev_) == 3) {
            const bool automatic = birriasdr_get_antenna_auto(dev_);
            report(birriasdr_set_channels(dev_, 1u << birriasdr_get_antenna(dev_)), "set_channels");
            if (automatic)
                report(birriasdr_set_antenna(dev_, BIRRIASDR_ANTENNA_AUTO), "set_antenna");
        }
    }
    dual_ = dual;
    const uint32_t transfers = streamArg(args, "transfers", 0, 0, 256, 1);
    if (transfers && transfers < 4)
        throw std::runtime_error("birria: invalid stream argument transfers=" +
                                 args.at("transfers"));
    bufferSamples_ = streamArg(args, "buffer_samples", 16384, 2048, 262144, 2048);
    transfers_ = transfers;
    format_ = chosen->second;
    streamOpen_ = true;
    return reinterpret_cast<SoapySDR::Stream *>(this);
}

/* Applications such as CubicSDR and SoapyRemote read at most this much per call. */
size_t SoapyBirria::getStreamMTU(SoapySDR::Stream *) const
{
    return bufferSamples_;
}

/* Stop the stream if it is running, then allow a new one. */
void SoapyBirria::closeStream(SoapySDR::Stream *)
{
    deactivateStream(nullptr);
    streamOpen_ = false;
}

/* Forward the library callback to the device instance. No exception may cross
 * into the C library, so a failure here, such as an allocation, stops the read.
 */
void SoapyBirria::asyncCallback(const int16_t *iq, uint32_t count, uint64_t first, void *ctx)
{
    auto *self = static_cast<SoapyBirria *>(ctx);
    try {
        self->onSamples(iq, nullptr, count, first);
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* The two-channel read's callback, chain 0 and chain 1 for the same instants. */
void SoapyBirria::asyncMultiCallback(const int16_t *const *iq, unsigned, uint32_t count,
                                     uint64_t first, void *ctx)
{
    auto *self = static_cast<SoapyBirria *>(ctx);
    try {
        self->onSamples(iq[0], iq[1], count, first);
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* Forward the library's gap notification to the device instance, with the same
 * failure handling as the sample callback.
 */
void SoapyBirria::recoveryCallback(const birriasdr_recovery_event *, void *ctx)
{
    auto *self = static_cast<SoapyBirria *>(ctx);
    try {
        self->onGap();
    } catch (...) {
        self->onCallbackFailure();
    }
}

/* Record the failure and cancel the read, which callbacks may do. The read then
 * returns normally, so the library still runs its cleanup. Callbacks that
 * arrive while it drains are dropped, so no samples follow the lost ones
 * without a gap.
 */
void SoapyBirria::onCallbackFailure()
{
    callbackFailed_ = true;
    birriasdr_cancel_async(dev_);
}

static long long steadyNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/* Nanoseconds that samples take at rate, without overflow over long runs. */
static long long samplesNs(uint64_t samples, uint32_t rate)
{
    return static_cast<long long>(samples / rate * 1000000000ull +
                                  samples % rate * 1000000000ull / rate);
}

/* Steady-clock time, the clock sample times are given in. */
long long SoapyBirria::getHardwareTime(const std::string &) const
{
    return steadyNs();
}

/* Runs on the read thread: copy and return. Never block here or the device SRAM overruns. */
void SoapyBirria::onSamples(const int16_t *iq, const int16_t *iq1, uint32_t count, uint64_t first)
{
    if (count == 0 || callbackFailed_)
        return; /* an empty chunk would read as a gap, and a failure ends the stream */
    const uint32_t rate = static_cast<uint32_t>(rate_.load());
    if (reanchor_ && rate) {
        /* The chunk has just arrived, so its first sample is a chunk's time old. */
        anchorSample_ = first;
        anchorNs_ = steadyNs() - samplesNs(count, rate);
        reanchor_ = false;
    }
    const long long ns = rate ? anchorNs_ + samplesNs(first - anchorSample_, rate) : 0;
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_ + count > kMaxQueuedSamples) {
        droppedSamples_ += count;
        markGap(); /* the application is not keeping up, so drop these samples */
        return;
    }
    const size_t values = 2 * static_cast<size_t>(count);
    queue_.push_back({std::vector<int16_t>(iq, iq + values),
                      iq1 ? std::vector<int16_t>(iq1, iq1 + values) : std::vector<int16_t>(), ns});
    queued_ += count;
    if (queued_ >= wanted_)
        cond_.notify_one();
}

/* Runs on the read thread after the library has delivered every sample from
 * before the gap, so a marker queued here sits at the gap's position.
 */
void SoapyBirria::onGap()
{
    if (callbackFailed_)
        return;
    ++deviceGaps_;
    reanchor_ = true; /* the lost samples are not counted */
    std::lock_guard<std::mutex> lock(mutex_);
    markGap();
}

/* Gaps are otherwise silent: a consumer sees only OVERFLOW. Say which side lost
 * the samples, the device or this queue, so a slow application is visible.
 */
void SoapyBirria::reportLosses()
{
    const auto now = std::chrono::steady_clock::now();
    if (now - lastReport_ < std::chrono::seconds(5))
        return;
    lastReport_ = now;
    const uint32_t gaps = deviceGaps_.exchange(0);
    const uint64_t dropped = droppedSamples_.exchange(0);
    if (gaps || dropped)
        SoapySDR::logf(SOAPY_SDR_WARNING,
                       "birria: %u device gaps, %llu samples dropped because the application "
                       "read too slowly",
                       gaps, (unsigned long long)dropped);
}

/* Queue a gap marker unless one is already last, so markers stay bounded by
 * the data chunks between them. The caller holds mutex_.
 */
void SoapyBirria::markGap()
{
    if (queue_.empty() || !queue_.back().iq.empty())
        queue_.emplace_back();
    cond_.notify_one();
}

/* Reset the queue, then run the blocking library read on a worker. A worker
 * whose read already ended on its own is joined before replacing it. Timed
 * starts, triggers and finite bursts are not supported, so any flag is refused.
 */
int SoapyBirria::activateStream(SoapySDR::Stream *, const int flags, const long long,
                                const size_t numElems)
{
    if (flags != 0 || numElems != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    /* Exclusive, so an idle-only setter cannot overlap the start of the read. */
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    if (running_)
        return 0;
    std::lock_guard<std::mutex> reading(readerMutex_);
    current_.iq.clear();
    current_.iq1.clear();
    offset_ = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        queued_ = 0;
    }
    active_ = true;
    if (startWorker() != 0) {
        active_ = false;
        return SOAPY_SDR_STREAM_ERROR;
    }
    return 0;
}

/* Run the blocking library read on a worker. A worker whose read already ended
 * on its own is joined first. The caller holds the control lock exclusively.
 */
int SoapyBirria::startWorker()
{
    if (worker_.joinable())
        worker_.join();
    callbackFailed_ = false;
    reanchor_ = true;
    running_ = true;
    try {
        if (soapy_birria_fail_worker_start())
            throw std::runtime_error("injected worker start failure");
        worker_ = std::thread([this] {
            if (dual_)
                report(birriasdr_read_async_multi(dev_, &SoapyBirria::asyncMultiCallback, this,
                                                  transfers_, bufferSamples_),
                       "read_async_multi");
            else
                report(birriasdr_read_async(dev_, &SoapyBirria::asyncCallback, this, transfers_,
                                            bufferSamples_),
                       "read_async");
            if (callbackFailed_)
                logError("birria: stopped the stream after a callback failure");
            {
                /* Publish under the lock so a reader cannot miss the wakeup. */
                std::lock_guard<std::mutex> lock(mutex_);
                running_ = false;
            }
            cond_.notify_all();
        });
    } catch (const std::exception &) {
        running_ = false;
        logError("birria: cannot start the stream thread");
        return SOAPY_SDR_STREAM_ERROR;
    }
    return 0;
}

/* Cancel the library read and join its worker. Right after a start the worker
 * may not have started the read yet, and cancel returns STATE, so keep retrying
 * until the read accepts it or the worker has finished. The caller holds the
 * control lock exclusively.
 */
void SoapyBirria::stopWorker()
{
    if (worker_.joinable()) {
        while (birriasdr_cancel_async(dev_) == BIRRIASDR_ERROR_STATE && running_)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        worker_.join();
    }
    running_ = false;
}

/* Stop the read and discard samples still queued. Timed and triggered stops
 * are not supported.
 */
int SoapyBirria::deactivateStream(SoapySDR::Stream *, const int flags, const long long)
{
    if (flags != 0)
        return SOAPY_SDR_NOT_SUPPORTED;
    /* Exclusive, so a stop cannot overlap a rate-change restart. */
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    active_ = false;
    stopWorker();
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
    queued_ = 0;
    return 0;
}

/* Copy what is available, up to numElems, converting to the stream format.
 * Wait only when nothing is ready. A gap ends the read early and returns
 * OVERFLOW on the next call, so no buffer spans it. An inactive stream waits
 * for the timeout and returns TIMEOUT, as SoapySDR requires. An active stream
 * whose read has ended returns STREAM_ERROR once its samples are drained. A
 * read during a rate-change restart waits for it, up to its timeout.
 */
int SoapyBirria::readStream(SoapySDR::Stream *, void *const *buffs, const size_t numElems,
                            int &flags, long long &timeNs, const long timeoutUs)
{
    flags = 0;
    timeNs = 0;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::microseconds(timeoutUs);
    if (!active_) {
        std::this_thread::sleep_until(deadline);
        return SOAPY_SDR_TIMEOUT;
    }
    for (;;) {
        /* Wait out a rate-change restart without holding the reader lock, so
         * the restart can take it. The deadline still applies.
         */
        {
            std::unique_lock<std::mutex> lock(mutex_);
            if (!cond_.wait_until(lock, deadline, [this] { return !restarting_; }))
                return SOAPY_SDR_TIMEOUT;
        }
        std::lock_guard<std::mutex> reading(readerMutex_);
        if (restarting_)
            continue;
        reportLosses();
        bool interrupted = false;
        const int result = readLocked(buffs, numElems, deadline, interrupted, timeNs, flags);
        if (!interrupted)
            return result;
    }
}

/* Convert n samples into buff at sample index at, in the stream format. CF32 takes
 * out the library's DC estimate when dc is set; the estimate is single-chain, so a
 * two-channel stream leaves DC in.
 */
void SoapyBirria::convert(const int16_t *in, void *buff, size_t at, size_t n, uint32_t rate,
                          bool dc) const
{
    const size_t width = format_ == Format::CF32 ? 8 : format_ == Format::CS16 ? 4 : 2;
    if (offBand_.load()) {
        std::memset(static_cast<char *>(buff) + width * at, format_ == Format::CU8 ? 0x80 : 0,
                    width * n);
    } else if (format_ == Format::CS16) {
        std::memcpy(static_cast<int16_t *>(buff) + 2 * at, in, 4 * n);
    } else if (format_ != Format::CF32) {
        /* Round a quarter of the full scale to 128, and clip. Each halving
         * below 10 MSps narrows the band and takes 3 dB of noise with it,
         * so it gets those 3 dB back and the noise floor stays put. */
        const float scale = 512.0f / static_cast<float>(birriasdr_get_full_scale(dev_)) *
                            (rate && rate < 10000000 ? std::sqrt(10e6f / static_cast<float>(rate)) : 1.0f);
        const int offset = format_ == Format::CU8 ? 128 : 0;
        uint8_t *out = static_cast<uint8_t *>(buff) + 2 * at;
        for (size_t k = 0; k < 2 * n; ++k) {
            const float v = static_cast<float>(in[k]) * scale;
            const int rounded = static_cast<int>(v + std::copysign(0.5f, v));
            out[k] = static_cast<uint8_t>(std::clamp(rounded, -128, 127) + offset);
        }
    } else {
        const float kScale = 1.0f / static_cast<float>(birriasdr_get_full_scale(dev_));
        float *out = static_cast<float *>(buff) + 2 * at;
        float dc_i = 0, dc_q = 0;
        if (dc && dcAuto_.load())
            birriasdr_get_dc_offset(dev_, &dc_i, &dc_q);
        for (size_t k = 0; k < n; ++k) {
            out[2 * k] = (in[2 * k] - dc_i) * kScale;
            out[2 * k + 1] = (in[2 * k + 1] - dc_q) * kScale;
        }
    }
}

/* Copy what is available, up to numElems, with the reader lock held. A restart
 * that begins before anything is copied sets interrupted, so the caller waits
 * for it and reads again. One that begins later ends the read with what was
 * copied.
 */
int SoapyBirria::readLocked(void *const *buffs, const size_t numElems,
                            const std::chrono::steady_clock::time_point deadline,
                            bool &interrupted, long long &timeNs, int &flags)
{
    size_t count = 0;
    while (count < numElems) {
        if (restarting_) {
            interrupted = count == 0;
            break;
        }
        if (offset_ == current_.iq.size() / 2) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.empty()) {
                if (count > 0)
                    break;
                if (running_) {
                    const size_t rateCap = static_cast<size_t>(rate_.load()) / kWakeRateDivisor;
                    wanted_ = std::max<size_t>(1, std::min({numElems, kWakeSamples, rateCap}));
                    const bool ready = cond_.wait_until(lock, deadline, [this] {
                        return queued_ >= wanted_ || (!queue_.empty() && queue_.back().iq.empty()) ||
                               !running_ || restarting_;
                    });
                    wanted_ = 1;
                    if (!ready && queue_.empty())
                        return SOAPY_SDR_TIMEOUT;
                }
                if (restarting_ && queue_.empty()) {
                    interrupted = true;
                    return 0;
                }
                /* The read has ended. Report it after the timeout, so a caller
                 * that retries at once does not spin on the error. */
                if (queue_.empty()) {
                    cond_.wait_until(lock, deadline, [this] { return restarting_ || !active_; });
                    return active_ ? SOAPY_SDR_STREAM_ERROR : SOAPY_SDR_TIMEOUT;
                }
            }
            if (queue_.front().iq.empty()) {
                if (count > 0)
                    break;
                queue_.pop_front();
                return SOAPY_SDR_OVERFLOW;
            }
            current_ = std::move(queue_.front());
            queue_.pop_front();
            queued_ -= current_.iq.size() / 2;
            offset_ = 0;
        }
        soapy_birria_read_hook();
        const size_t n = std::min(current_.iq.size() / 2 - offset_, numElems - count);
        const int16_t *in = &current_.iq[2 * offset_];
        const uint32_t rate = static_cast<uint32_t>(rate_.load());
        if (count == 0 && current_.ns && rate) {
            timeNs = current_.ns + samplesNs(offset_, rate);
            flags |= SOAPY_SDR_HAS_TIME;
        }
        convert(in, buffs[0], count, n, rate, !dual_);
        if (dual_)
            convert(&current_.iq1[2 * offset_], buffs[1], count, n, rate, false);
        count += n;
        offset_ += n;
    }
    return static_cast<int>(count);
}
