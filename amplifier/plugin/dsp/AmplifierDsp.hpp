// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// AmplifierDsp.hpp
//
// Framework-independent core shared by both plugins in this project --
// no DPF/DGL dependency. Originally deliberately tiny (a static-gain
// amplifier's whole per-sample job is a couple of scalar multiplies) --
// grew one more piece with the Listen/Normalize feature (see ListenMeter
// below): still no envelope/curve to speak of, unlike the sibling
// ../compressor project's own plugin/dsp/CompressorDsp.hpp this file is
// styled after. Header-only for the same reason that one is (nothing here
// is large enough to justify splitting declaration from definition).

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ssamp {

inline double dbToLinear(double db)
{
    return std::pow(10.0, db / 20.0);
}

// Classic mixing-console "Balance" law (distinct from panning a mono
// source across a stereo field): centered (balance == 0) leaves both
// channels at unity: moving toward one side only ever attenuates the
// *other* channel, linearly, down to silence at the extreme -- it never
// boosts the side you're moving toward past unity. `balance` is clamped
// to [-1, +1] (-1 = full left, +1 = full right) before use, so a stray
// out-of-range value (a host sending something odd) can't invert the
// law's own direction or push either gain negative.
inline void balanceGains(double balance, double& gainL, double& gainR) noexcept
{
    balance = std::max(-1.0, std::min(1.0, balance));
    gainL = balance <= 0.0 ? 1.0 : (1.0 - balance);
    gainR = balance >= 0.0 ? 1.0 : (1.0 + balance);
}

// Which reading Listen normalizes against -- see ListenMeter's own
// comment for what each actually measures.
enum class ListenMode { Peak = 0, Rms = 1 };

inline ListenMode clampListenMode(int raw)
{
    return raw == 1 ? ListenMode::Rms : ListenMode::Peak;
}

// The "check the volume" side of the Listen/Normalize feature: while
// Listen is active, run()'s own audio path is a raw passthrough of the
// input (see PluginAmplifierMono/Stereo::run()'s own comment for why --
// you're auditioning the *source*, not whatever Gain happens to be
// currently dialed in) and every sample also passes through here, purely
// to measure it. Deliberately NOT a real-time-ballistics meter (no decay,
// no hold, no time constant) -- this measures the ENTIRE listened section
// as one whole, exactly like a normalize function in an audio editor
// would: Peak is the single loudest absolute sample seen since reset(),
// RMS is the true root-mean-square energy of every sample seen since
// reset(), not a moving average. Both readings are available continuously
// via peakDb()/rmsDb() (Peak can only ever rise; RMS converges as more
// audio streams through) so the GUI can show a live, updating number
// while Listen is still running, not just a final value once stopped.
class ListenMeter
{
public:
    void reset() noexcept { peakLin_ = 0.0; sumSq_ = 0.0; count_ = 0; }

    // One sample. For a stereo pair, call once per channel per frame (see
    // the stereo plugin's own run()) -- RMS then reads as the combined
    // energy across both channels, Peak as whichever single sample (of
    // either channel) was loudest.
    inline void process(double x) noexcept
    {
        const double a = std::fabs(x);
        if (a > peakLin_) peakLin_ = a;
        sumSq_ += x * x;
        ++count_;
    }

    double peakDb() const noexcept { return 20.0 * std::log10(std::max(peakLin_, 1e-9)); }
    double rmsDb() const noexcept
    {
        const double meanSq = count_ > 0 ? sumSq_ / double(count_) : 0.0;
        return 20.0 * std::log10(std::max(std::sqrt(meanSq), 1e-9));
    }
    double readDb(ListenMode mode) const noexcept { return mode == ListenMode::Rms ? rmsDb() : peakDb(); }
    bool hasAnySamples() const noexcept { return count_ > 0; }

private:
    double peakLin_ = 0.0;
    double sumSq_ = 0.0;
    uint64_t count_ = 0;
};

// A continuous, always-live "is the signal leaving this plugin right now
// too hot" indicator -- deliberately separate from ListenMeter above,
// whose own job (a one-shot Peak/RMS measurement for Normalize, frozen
// the instant Listen stops) is the wrong shape for this: reported
// directly ("If the signal is to strong he is not showing the
// clipping"), and true regardless of Listen -- outside a Listen session
// the meter box shows a static last-audition snapshot, and even during
// one, neither reading was ever compared against 0dBFS or flagged in any
// way. This checks the *actual samples the plugin just wrote to its own
// output* (whichever of run()'s branches -- Gain, Bypass, or Listen's own
// passthrough -- produced them; any of the three can carry an
// already-too-hot signal straight through unchanged) against plain
// digital full scale (|x| > 1.0, 0dBFS) once per processed block, and
// latches "clipping" for a short hold window afterward rather than
// flickering on/off at the exact instant a sample stops exceeding the
// threshold -- the same "hold the LED lit a moment after the fact" idea
// a hardware VU's clip light or a DAW meter's own clip indicator already
// uses, so a single hot transient is actually visible on screen instead
// of vanishing again before the next GUI repaint.
class ClipHold
{
public:
    void reset() noexcept { secondsSinceClip_ = kHoldSeconds; } // starts "not clipping"

    // Scans one channel's own buffer for anything at/beyond 0dBFS. For
    // more than one channel, OR the per-channel results together and pass
    // the combined bool to update() *once* per block -- calling update()
    // once per channel would double-advance the hold timer when neither
    // channel is clipping, shrinking the hold window by however many
    // channels there are.
    static bool bufferClips(const float* buf, uint32_t frames) noexcept
    {
        for (uint32_t i = 0; i < frames; ++i)
            if (std::fabs(buf[i]) > 1.0f) return true;
        return false;
    }

    // `blockSeconds` converts the block's own length into wall-clock time
    // so the hold window means the same thing regardless of host block
    // size or sample rate.
    void update(bool clippedThisBlock, double blockSeconds) noexcept
    {
        secondsSinceClip_ = clippedThisBlock ? 0.0 : secondsSinceClip_ + blockSeconds;
    }

    bool isClipping() const noexcept { return secondsSinceClip_ < kHoldSeconds; }

private:
    static constexpr double kHoldSeconds = 1.5;
    double secondsSinceClip_ = kHoldSeconds;
};

} // namespace ssamp
