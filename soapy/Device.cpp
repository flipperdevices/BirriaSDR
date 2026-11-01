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

/* The receiver has one RX channel and no TX path. */
size_t SoapyBirria::getNumChannels(const int direction) const
{
    return direction == SOAPY_SDR_RX ? 1 : 0;
}

/* One antenna per receive chain, named RX0 and RX1 after the chain. The chain
 * is set up when a read starts, so a running stream restarts as for a rate
 * change.
 */
std::vector<std::string> SoapyBirria::listAntennas(const int, const size_t) const
{
    std::vector<std::string> names;
    for (unsigned i = 0; i < birriasdr_get_antenna_count(dev_); ++i)
        names.push_back("RX" + std::to_string(i));
    return names;
}

void SoapyBirria::setAntenna(const int, const size_t, const std::string &name)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    if (name.size() != 3 || name.compare(0, 2, "RX") != 0 || name[2] < '0' || name[2] > '9') {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: unknown antenna " + name);
        return;
    }
    const unsigned antenna = static_cast<unsigned>(name[2] - '0');
    if (antenna == birriasdr_get_antenna(dev_))
        return;
    whileIdle([this, antenna] {
        report(birriasdr_set_antenna(dev_, antenna), "set_antenna");
    }, "birria: cannot restart the stream after an antenna change");
}

std::string SoapyBirria::getAntenna(const int, const size_t) const
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
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
 * LNA 0 to 5, LPF 0 to 8, DIG 0 to 12.
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

/* Request a center in Hz. The library rejects it without retuning if invalid. */
void SoapyBirria::setFrequency(const int, const size_t, const std::string &, const double frequency,
                               const SoapySDR::Kwargs &)
{
    std::shared_lock<std::shared_mutex> control(controlMutex_);
    report(birriasdr_set_center_freq(dev_, static_cast<uint64_t>(std::llround(frequency))),
           "set_center_freq");
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

/* Rates the hardware produces without resampling. The library cannot list its
 * rates, so add one here when the backend gains it.
 */
static const double kNativeRates[] = {10e6, 20e6, 40e6, 80e6};

/* Accept only a native rate and pass it to the library. The library requires
 * this setter to run alone on the handle, so it holds the control lock
 * exclusively, which also keeps a stream from starting meanwhile.
 */
void SoapyBirria::setSampleRate(const int, const size_t, const double rate)
{
    std::unique_lock<std::shared_mutex> control(controlMutex_);
    /* Match the request within 1 Hz of a native rate and forward that exact
     * rate, so neither a huge value nor a fractional one reaches the library.
     */
    const double *native = std::find_if(std::begin(kNativeRates), std::end(kNativeRates),
                                        [rate](double n) { return std::fabs(rate - n) < 1.0; });
    if (native == std::end(kNativeRates)) {
        SoapySDR::log(SOAPY_SDR_ERROR, "birria: the sample rate must be a native rate");
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
    current_.clear();
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

/* List the native rates from lowest to highest. */
std::vector<double> SoapyBirria::listSampleRates(const int, const size_t) const
{
    return std::vector<double>(std::begin(kNativeRates), std::end(kNativeRates));
}

/* The library delivers CS16. CF32 is a scaled copy, with the library's DC
 * estimate, the digital gain stage's truncation, taken out while automatic DC
 * correction is on.
 */
std::vector<std::string> SoapyBirria::getStreamFormats(const int, const size_t) const
{
    return {SOAPY_SDR_CF32, SOAPY_SDR_CS16};
}

/* Report the native format and its full-scale value. */
std::string SoapyBirria::getNativeStreamFormat(const int, const size_t, double &fullScale) const
{
    fullScale = 8192.0; /* signed 14-bit samples carried in int16 */
    return SOAPY_SDR_CS16;
}

/* Accept one RX stream on channel 0 in CS16 or CF32. The device is the stream
 * handle, so a second stream would change the format under the first.
 */
SoapySDR::Stream *SoapyBirria::setupStream(const int direction, const std::string &format,
                                           const std::vector<size_t> &channels,
                                           const SoapySDR::Kwargs &)
{
    if (direction != SOAPY_SDR_RX || (format != SOAPY_SDR_CS16 && format != SOAPY_SDR_CF32))
        throw std::runtime_error("birria: only RX CF32 and CS16 streams are supported");
    if (channels.size() > 1 || (channels.size() == 1 && channels[0] != 0))
        throw std::runtime_error("birria: only channel 0 is supported");
    if (streamOpen_)
        throw std::runtime_error("birria: close the open stream before setting up another");
    cs16_ = format == SOAPY_SDR_CS16;
    streamOpen_ = true;
    return reinterpret_cast<SoapySDR::Stream *>(this);
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
void SoapyBirria::asyncCallback(const int16_t *iq, uint32_t count, uint64_t, void *ctx)
{
    auto *self = static_cast<SoapyBirria *>(ctx);
    try {
        self->onSamples(iq, count);
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

/* Runs on the read thread: copy and return. Never block here or the device SRAM overruns. */
void SoapyBirria::onSamples(const int16_t *iq, uint32_t count)
{
    if (count == 0 || callbackFailed_)
        return; /* an empty chunk would read as a gap, and a failure ends the stream */
    std::lock_guard<std::mutex> lock(mutex_);
    if (queued_ + count > kMaxQueuedSamples) {
        droppedSamples_ += count;
        markGap(); /* the application is not keeping up, so drop these samples */
        return;
    }
    queue_.emplace_back(iq, iq + 2 * static_cast<size_t>(count));
    queued_ += count;
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
    if (queue_.empty() || !queue_.back().empty())
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
    current_.clear();
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
    running_ = true;
    try {
        if (soapy_birria_fail_worker_start())
            throw std::runtime_error("injected worker start failure");
        worker_ = std::thread([this] {
            report(birriasdr_read_async(dev_, &SoapyBirria::asyncCallback, this, 0, 0),
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
        const int result = readLocked(buffs, numElems, deadline, interrupted);
        if (!interrupted)
            return result;
    }
}

/* Copy what is available, up to numElems, with the reader lock held. A restart
 * that begins before anything is copied sets interrupted, so the caller waits
 * for it and reads again. One that begins later ends the read with what was
 * copied.
 */
int SoapyBirria::readLocked(void *const *buffs, const size_t numElems,
                            const std::chrono::steady_clock::time_point deadline,
                            bool &interrupted)
{
    size_t count = 0;
    while (count < numElems) {
        if (restarting_) {
            interrupted = count == 0;
            break;
        }
        if (offset_ == current_.size() / 2) {
            std::unique_lock<std::mutex> lock(mutex_);
            if (queue_.empty()) {
                if (count > 0)
                    break;
                if (running_ && !cond_.wait_until(lock, deadline, [this] {
                        return !queue_.empty() || !running_ || restarting_;
                    }))
                    return SOAPY_SDR_TIMEOUT;
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
            if (queue_.front().empty()) {
                if (count > 0)
                    break;
                queue_.pop_front();
                return SOAPY_SDR_OVERFLOW;
            }
            current_ = std::move(queue_.front());
            queue_.pop_front();
            queued_ -= current_.size() / 2;
            offset_ = 0;
        }
        soapy_birria_read_hook();
        const size_t n = std::min(current_.size() / 2 - offset_, numElems - count);
        const int16_t *in = &current_[2 * offset_];
        if (cs16_) {
            std::memcpy(static_cast<int16_t *>(buffs[0]) + 2 * count, in, 4 * n);
        } else {
            constexpr float kScale = 1.0f / 8192.0f;
            float *out = static_cast<float *>(buffs[0]) + 2 * count;
            float dc_i = 0, dc_q = 0;
            if (dcAuto_.load())
                birriasdr_get_dc_offset(dev_, &dc_i, &dc_q);
            for (size_t k = 0; k < n; ++k) {
                out[2 * k] = (in[2 * k] - dc_i) * kScale;
                out[2 * k + 1] = (in[2 * k + 1] - dc_q) * kScale;
            }
        }
        count += n;
        offset_ += n;
    }
    return static_cast<int>(count);
}
