// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// LoudnessDsp.hpp
//
// Framework-independent core of the Loudness Meter plugin -- no DPF/DGL
// dependency, same "small enough to stay header-only" reasoning
// PhaseScopeDsp.hpp's own header comment gives. A standards-based (ITU-R
// BS.1770-4 / EBU R128 / EBU Tech 3342) stereo loudness measurement,
// modeled on Youlean Loudness Meter 2's own set of readings and their
// units -- Momentary/Short-term/Integrated Loudness in LUFS, Loudness
// Range (LRA) in LU, True Peak in dBTP, and the two peak-to-loudness
// ratios (PLR, PSR) those combine into, all in LU.
//
// Six pieces, all fed straight from the input signal in
// PluginLoudnessMeter::run() (this plugin, like phase_scope, never
// touches the audio -- passes L/R straight through unmodified):
//
//   - KWeightingFilter: the ITU-R BS.1770-4 "K-weighting" pre-filter (a
//     high-frequency shelf cascaded with a high-pass), one per channel.
//     Every loudness reading below is computed from *this* filtered
//     signal, not the raw one -- K-weighting is what makes an LUFS/LKFS
//     reading correlate with perceived loudness at all, rather than just
//     being an RMS meter with extra steps.
//   - SlidingMeanSquare: an exact rectangular-window mean square (not an
//     exponential-decay ballistic like PhaseScopeDsp.hpp's own
//     LoudnessMeter) -- Momentary (400ms) and Short-term (3s) loudness
//     are both defined by the standard as literal time windows, not a
//     decaying average, so this is a real ring-buffer sum, not a one-pole
//     filter standing in for one.
//   - LoudnessHistogram: an energy-weighted histogram of gating-block
//     loudness values, the same technique reference implementations
//     (e.g. libebur128) use to compute Integrated Loudness's two-stage
//     absolute+relative gating and Loudness Range's percentile
//     statistics without storing every block ever measured.
//   - TruePeakDetector: a 4x-oversampling interpolator catching
//     inter-sample peaks a plain sample-peak reading misses -- see its
//     own comment for why this is a general-purpose design, not a
//     bit-exact reproduction of the ITU's own reference filter.
//   - DistributionHistogram: a plain tally of how many Short-term
//     readings landed in each of a coarse set of loudness bins, backing
//     the "statistical view" ("Youlean shows the example") -- separate
//     from LoudnessHistogram above, which stores power (not counts)
//     specifically to support gated averaging, not a distribution shape.
//   - LufsMeter: ties all of the above together per the plugin's own
//     stereo input, at the fixed 100ms "gating block" cadence ITU-R
//     BS.1770-4 specifies.

#pragma once

#include "PhaseScopeDsp.hpp" // reuses linearToDb/dbToLinear

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ssmeter {

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ---- K-weighting filter (ITU-R BS.1770-4) ----------------------------------
//
// Two cascaded biquads -- a high-frequency shelf ("pre-filter") and a
// high-pass ("RLB weighting") -- both defined in the standard by an
// analog-prototype center frequency, Q, and (for the shelf) a gain, then
// turned into digital coefficients via the standard RBJ Audio EQ
// Cookbook bilinear-transform formulas. Computing coefficients from
// these parameters at the *actual* sample rate (rather than hardcoding a
// 48kHz-only coefficient table, which is how the standard's own
// informative annex tabulates them) is what makes this correct at
// 44.1kHz, 96kHz, or any other rate a host happens to run at.
class KWeightingFilter
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        designHighShelf(1681.9744509555319, 3.99984385397, 0.7071752369554193, stage1_);
        designHighPass(38.13547087602444, 0.5003270373238773, stage2_);
        reset();
    }

    inline double process(double x)
    {
        return stage2_.process(stage1_.process(x));
    }

    void reset() { stage1_.reset(); stage2_.reset(); }

private:
    struct Biquad
    {
        double b0 = 1.0, b1 = 0.0, b2 = 0.0, a1 = 0.0, a2 = 0.0;
        double z1 = 0.0, z2 = 0.0;

        // Transposed Direct Form II -- a1/a2 here are already normalized
        // by a0 (see the design*() functions below), so no further
        // division happens per sample.
        inline double process(double x)
        {
            const double y = b0 * x + z1;
            z1 = b1 * x - a1 * y + z2;
            z2 = b2 * x - a2 * y;
            return y;
        }

        void reset() { z1 = z2 = 0.0; }
    };

    void designHighShelf(double f0, double gainDb, double Q, Biquad& bq) const
    {
        const double A = std::pow(10.0, gainDb / 40.0);
        const double w0 = 2.0 * M_PI * f0 / sampleRate_;
        const double cosw0 = std::cos(w0), sinw0 = std::sin(w0);
        const double alpha = sinw0 / (2.0 * Q);
        const double sqrtA = std::sqrt(A);

        const double b0 = A * ((A + 1.0) + (A - 1.0) * cosw0 + 2.0 * sqrtA * alpha);
        const double b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cosw0);
        const double b2 = A * ((A + 1.0) + (A - 1.0) * cosw0 - 2.0 * sqrtA * alpha);
        const double a0 = (A + 1.0) - (A - 1.0) * cosw0 + 2.0 * sqrtA * alpha;
        const double a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cosw0);
        const double a2 = (A + 1.0) - (A - 1.0) * cosw0 - 2.0 * sqrtA * alpha;

        bq.b0 = b0 / a0; bq.b1 = b1 / a0; bq.b2 = b2 / a0;
        bq.a1 = a1 / a0; bq.a2 = a2 / a0;
    }

    void designHighPass(double f0, double Q, Biquad& bq) const
    {
        const double w0 = 2.0 * M_PI * f0 / sampleRate_;
        const double cosw0 = std::cos(w0), sinw0 = std::sin(w0);
        const double alpha = sinw0 / (2.0 * Q);

        const double b0 = (1.0 + cosw0) / 2.0;
        const double b1 = -(1.0 + cosw0);
        const double b2 = (1.0 + cosw0) / 2.0;
        const double a0 = 1.0 + alpha;
        const double a1 = -2.0 * cosw0;
        const double a2 = 1.0 - alpha;

        bq.b0 = b0 / a0; bq.b1 = b1 / a0; bq.b2 = b2 / a0;
        bq.a1 = a1 / a0; bq.a2 = a2 / a0;
    }

    double sampleRate_ = 48000.0;
    Biquad stage1_, stage2_;
};

// ---- exact rectangular-window mean square ----------------------------------
//
// Momentary (400ms) and Short-term (3s) loudness are literal time
// windows in the standard, not a decaying ballistic -- a true ring-buffer
// running sum, unlike PhaseScopeDsp.hpp's own one-pole LoudnessMeter.
class SlidingMeanSquare
{
public:
    void prepare(double sampleRate, double windowSeconds)
    {
        size_ = std::max(1, int(std::lround(sampleRate * windowSeconds)));
        buffer_.assign(size_t(size_), 0.0);
        reset();
    }

    // weightedSquareSum: the channel-weighted sum of squares for one
    // sample (e.g. L^2 + R^2 for an equal-weighted stereo pair) --
    // pushed once per sample.
    inline void push(double weightedSquareSum)
    {
        sum_ -= buffer_[size_t(writeIndex_)];
        buffer_[size_t(writeIndex_)] = weightedSquareSum;
        sum_ += weightedSquareSum;
        writeIndex_ = (writeIndex_ + 1) % size_;
        if (filled_ < size_) ++filled_;
    }

    // Mean over however much of the window has actually filled -- matters
    // only in the first fraction of a second of a stream, before a full
    // window's worth of audio has actually played.
    double mean() const { return filled_ > 0 ? sum_ / double(filled_) : 0.0; }

    void reset()
    {
        std::fill(buffer_.begin(), buffer_.end(), 0.0);
        writeIndex_ = 0;
        filled_ = 0;
        sum_ = 0.0;
    }

private:
    std::vector<double> buffer_;
    int size_ = 1;
    int writeIndex_ = 0;
    int filled_ = 0;
    double sum_ = 0.0;
};

// ---- energy-weighted gating histogram --------------------------------------
//
// Both Integrated Loudness (ITU-R BS.1770-4's two-stage absolute+relative
// gate) and Loudness Range (EBU Tech 3342's percentile statistics over a
// relative+absolute-gated distribution) reduce to "the mean power of
// every gating block whose own loudness clears some threshold" or "the
// Nth percentile position within that gated set" -- computed here from a
// histogram of *summed power* (not summed dB, which cannot be correctly
// averaged) keyed by 0.1 LU loudness bins, so an unboundedly long
// "Integrated" measurement costs a fixed, small amount of memory
// regardless of how long the transport has been rolling.
class LoudnessHistogram
{
public:
    static constexpr double kMinLufs = -70.0; // ITU-R BS.1770-4's own absolute gate
    static constexpr double kMaxLufs = 5.0;
    static constexpr double kBinWidth = 0.1;
    static constexpr int kBinCount = int((kMaxLufs - kMinLufs) / kBinWidth + 0.5) + 1; // 751

    void reset()
    {
        for (auto& b : bins_) { b.sumZ = 0.0; b.count = 0.0; }
    }

    // z: one gating block's own channel-weighted mean-square power.
    // Blocks quieter than kMinLufs are the absolute gate -- never even
    // entered, since no relative-gate stage could include them anyway.
    void addBlock(double z)
    {
        const double l = loudnessFromZ(z);
        if (l < kMinLufs) return;
        int bin = int(std::floor((l - kMinLufs) / kBinWidth));
        bin = std::max(0, std::min(kBinCount - 1, bin));
        bins_[size_t(bin)].sumZ += z;
        bins_[size_t(bin)].count += 1.0;
    }

    // The standard's two-stage gate: relativeOffsetLu is -10 LU for
    // Integrated Loudness (ITU-R BS.1770-4), -20 LU for the short-term
    // distribution LRA is computed from (EBU Tech 3342) -- both share the
    // same absolute gate and the same "relative to the ungated mean"
    // shape, just a different offset, so one function serves both.
    double gatedLoudness(double relativeOffsetLu) const
    {
        const double zUngated = meanZFrom(0);
        if (zUngated <= 0.0) return kMinLufs;
        const int fromBin = relativeGateBin(zUngated, relativeOffsetLu);
        const double zGated = meanZFrom(fromBin);
        return zGated > 0.0 ? loudnessFromZ(zGated) : kMinLufs;
    }

    // The Nth percentile (fraction in [0,1]) of the block-count
    // distribution, within the same relative+absolute gate
    // gatedLoudness() uses -- EBU Tech 3342's LRA is percentile(0.95, -20)
    // - percentile(0.10, -20).
    double percentile(double fraction, double relativeOffsetLu) const
    {
        const double zUngated = meanZFrom(0);
        if (zUngated <= 0.0) return kMinLufs;
        const int fromBin = relativeGateBin(zUngated, relativeOffsetLu);

        double totalCount = 0.0;
        for (int i = fromBin; i < kBinCount; ++i) totalCount += bins_[size_t(i)].count;
        if (totalCount <= 0.0) return kMinLufs;

        const double target = fraction * totalCount;
        double cumulative = 0.0;
        for (int i = fromBin; i < kBinCount; ++i)
        {
            cumulative += bins_[size_t(i)].count;
            if (cumulative >= target)
                return kMinLufs + (double(i) + 0.5) * kBinWidth; // bin center
        }
        return kMaxLufs;
    }

    static double loudnessFromZ(double z) { return -0.691 + 10.0 * std::log10(std::max(1.0e-12, z)); }

private:
    struct Bin { double sumZ = 0.0; double count = 0.0; };

    double meanZFrom(int fromBin) const
    {
        double sumZ = 0.0, count = 0.0;
        for (int i = std::max(0, fromBin); i < kBinCount; ++i)
        {
            sumZ += bins_[size_t(i)].sumZ;
            count += bins_[size_t(i)].count;
        }
        return count > 0.0 ? sumZ / count : 0.0;
    }

    int relativeGateBin(double zUngated, double relativeOffsetLu) const
    {
        const double thresholdLufs = loudnessFromZ(zUngated) + relativeOffsetLu;
        return int(std::ceil((thresholdLufs - kMinLufs) / kBinWidth));
    }

    Bin bins_[kBinCount] = {};
};

// ---- true peak: 4x-oversampling interpolating peak detector ---------------
//
// A plain sample-peak reading can miss a real inter-sample overshoot on a
// hot digital mix -- the entire point of a "True Peak" (dBTP) reading is
// reconstructing what the analog waveform between samples actually looks
// like and measuring *that* peak instead. This is a general-purpose
// windowed-sinc 4x oversampling interpolator tuned for that job, not a
// bit-exact reproduction of ITU-R BS.1770 Annex 2's own reference filter
// -- a real certification tool would need to match that filter exactly;
// a monitoring meter just needs to reliably catch and report genuine
// inter-sample energy, which this does. Has kHalfTaps samples of
// reporting latency (inherent to any symmetric-support interpolation
// filter) -- harmless here since nothing in this plugin's own audio path
// depends on it (see PluginLoudnessMeter.cpp's own run() comment: input
// is always copied straight to output, unaffected by any meter's
// internal latency).
class TruePeakDetector
{
public:
    static constexpr int kOversample = 4;
    static constexpr int kHalfTaps = 8;
    static constexpr int kHistSize = 2 * kHalfTaps + 1;

    TruePeakDetector() { designFilter(); reset(); }

    void reset()
    {
        history_.assign(kHistSize, 0.0);
        runningPeak_ = 0.0;
    }

    inline void process(double x)
    {
        for (int i = 0; i < kHistSize - 1; ++i) history_[size_t(i)] = history_[size_t(i + 1)];
        history_[size_t(kHistSize - 1)] = x;

        // Phase 0 is the exact center sample -- see designFilter()'s own
        // comment for why that tap set reduces to a pure impulse.
        double blockPeak = std::fabs(history_[size_t(kHalfTaps)]);
        for (int phase = 1; phase < kOversample; ++phase)
        {
            double acc = 0.0;
            const auto& taps = taps_[size_t(phase)];
            for (int k = -kHalfTaps; k <= kHalfTaps; ++k)
                acc += taps[size_t(k + kHalfTaps)] * history_[size_t(k + kHalfTaps)];
            blockPeak = std::max(blockPeak, std::fabs(acc));
        }
        runningPeak_ = std::max(runningPeak_, blockPeak);
    }

    double peakLinear() const { return runningPeak_; }
    double peakDb() const { return linearToDb(runningPeak_); }
    void resetPeak() { runningPeak_ = 0.0; }

private:
    void designFilter()
    {
        for (int phase = 0; phase < kOversample; ++phase)
        {
            auto& taps = taps_[size_t(phase)];
            taps.assign(kHistSize, 0.0);
            const double frac = double(phase) / double(kOversample);
            for (int k = -kHalfTaps; k <= kHalfTaps; ++k)
            {
                const double t = double(k) - frac;
                const double sinc = (std::fabs(t) < 1.0e-9) ? 1.0 : std::sin(M_PI * t) / (M_PI * t);
                // Blackman window over the whole symmetric support --
                // exactly 1.0 at the center by construction, which is
                // what makes phase 0's tap set a pure impulse below.
                const double u = double(k + kHalfTaps) / double(kHistSize - 1);
                const double w = 0.42 - 0.5 * std::cos(2.0 * M_PI * u) + 0.08 * std::cos(4.0 * M_PI * u);
                taps[size_t(k + kHalfTaps)] = sinc * w;
            }
            // Normalize to exactly unity gain at DC -- without this, a
            // constant (DC) input would read a true peak slightly off
            // from its own sample value purely from windowing error, not
            // from any genuine inter-sample content.
            double tapSum = 0.0;
            for (double v : taps) tapSum += v;
            if (std::fabs(tapSum) > 1.0e-9)
                for (double& v : taps) v /= tapSum;
        }
    }

    std::vector<double> history_;
    std::array<std::vector<double>, kOversample> taps_;
    double runningPeak_ = 0.0;
};

// ---- distribution histogram: a plain tally for the "statistical view" -----
//
// Deliberately separate from LoudnessHistogram above: that one stores
// summed *power* per bin specifically to support gated averaging
// (Integrated Loudness, LRA); this one just counts how many Short-term
// readings landed in each of a much coarser set of bins, which is exactly
// what a "how has this track's loudness been distributed over time"
// chart needs -- no gating, no power-domain averaging, just a plain
// tally, the same technique Youlean Loudness Meter 2's own Distribution
// view is based on ("Youlean shows the example").
class DistributionHistogram
{
public:
    static constexpr double kMinLufs = -60.0;
    static constexpr double kMaxLufs = 0.0;
    static constexpr double kBinWidth = 1.0;
    static constexpr int kBinCount = int((kMaxLufs - kMinLufs) / kBinWidth + 0.5); // 60

    void reset() { for (double& c : counts_) c = 0.0; }

    void addSample(double lufs)
    {
        int bin = int(std::floor((lufs - kMinLufs) / kBinWidth));
        bin = std::max(0, std::min(kBinCount - 1, bin));
        counts_[size_t(bin)] += 1.0;
    }

    double count(int bin) const { return (bin >= 0 && bin < kBinCount) ? counts_[size_t(bin)] : 0.0; }

private:
    double counts_[kBinCount] = {};
};

// ---- top-level stereo loudness meter ---------------------------------------
//
// Named LufsMeter, not LoudnessMeter, specifically to avoid colliding
// with PhaseScopeDsp.hpp's own (differently-shaped, exponential-decay)
// ssmeter::LoudnessMeter class -- both headers live in the same
// namespace and dsp_regression.cpp includes both.
//
// Ties the pieces above together at the fixed 100ms "gating block"
// cadence ITU-R BS.1770-4 specifies: SlidingMeanSquare's own continuously
// updated Momentary/Short-term windows, sampled once every 100ms, *are*
// the sequence of gating blocks the standard describes -- no separate
// block-accumulation mechanism is needed, since a 100ms-stepped sample of
// an exact 400ms/3s sliding window is exactly what "400ms blocks with 75%
// overlap" / "3s blocks with 100ms hop" already describe.
class LufsMeter
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        kWeightL_.prepare(sampleRate_);
        kWeightR_.prepare(sampleRate_);
        momentary_.prepare(sampleRate_, 0.4);
        shortTerm_.prepare(sampleRate_, 3.0);
        truePeakL_.reset();
        truePeakR_.reset();
        blockPeriodSamples_ = std::max(1, int(std::lround(sampleRate_ * 0.1)));
        reset();
    }

    void reset()
    {
        kWeightL_.reset();
        kWeightR_.reset();
        momentary_.reset();
        shortTerm_.reset();
        integratedHistogram_.reset();
        shortTermHistogram_.reset();
        distributionShortTerm_.reset();
        distributionMomentary_.reset();
        truePeakL_.reset();
        truePeakR_.reset();
        samplesSinceBlock_ = 0;
        momentaryLufs_ = kSilenceLufs;
        shortTermLufs_ = kSilenceLufs;
    }

    inline void process(double l, double r)
    {
        const double kl = kWeightL_.process(l);
        const double kr = kWeightR_.process(r);
        // Equal-weighted stereo (G_i = 1.0 for L and R, per ITU-R
        // BS.1770-4's own channel weighting table) -- surrounds, which
        // would carry the standard's own +1.5dB weighting, don't apply to
        // a plain stereo input.
        const double z = kl * kl + kr * kr;
        momentary_.push(z);
        shortTerm_.push(z);

        truePeakL_.process(l);
        truePeakR_.process(r);

        if (++samplesSinceBlock_ >= blockPeriodSamples_)
        {
            samplesSinceBlock_ = 0;
            const double zMomentary = momentary_.mean();
            const double zShortTerm = shortTerm_.mean();
            momentaryLufs_ = LoudnessHistogram::loudnessFromZ(zMomentary);
            shortTermLufs_ = LoudnessHistogram::loudnessFromZ(zShortTerm);
            integratedHistogram_.addBlock(zMomentary); // Integrated Loudness gates on 400ms blocks
            shortTermHistogram_.addBlock(zShortTerm);  // LRA gates on 3s (short-term) blocks
            // Two separate plain tallies -- "one with the short term and
            // one with momentary values" -- rather than one shared
            // histogram, since the two readings have genuinely different
            // distributions (Momentary is far more volatile) and the UI
            // now draws them as two distinct density plots.
            //
            // Skip a block that's at or below the plot's own visible floor
            // (kMinLufs, -60), so pausing the transport or starting
            // playback before the music begins doesn't pile silent blocks
            // into the bottom bin. Unlike integratedHistogram_/
            // shortTermHistogram_ just above, DistributionHistogram is
            // deliberately a plain, ungated tally (see its own class
            // comment) whose own addSample() *always* clamps an
            // out-of-range value into the nearest end bin rather than
            // dropping it (by design -- dsp_regression.cpp's own [18]
            // test relies on exactly that clamping behavior for
            // DistributionHistogram used standalone), so the gate has to
            // live here, in the caller, instead.
            //
            // A first version of this gated on `z > 0.0` instead -- wrong:
            // exact digital zero is the
            // easy case, but a K-weighting filter's own decay tail after
            // real audio stops (its internal state doesn't snap to zero
            // the instant the input does), plus ordinary denormals/dither/
            // noise-floor residue, all produce a genuinely nonzero but
            // still musically-silent z -- comfortably passing `z > 0.0`
            // while resolving to some very negative LUFS number, which
            // addSample() then clamps straight into the bottom bin
            // regardless of how far below range it actually is. Gating on
            // the *already-clamped-range* check instead -- would this
            // block's own LUFS reading even be visible on the plot at all
            // -- catches every one of those cases the same way, without
            // having to reason about how close to exactly zero "close
            // enough" needs to be.
            if (shortTermLufs_ >= DistributionHistogram::kMinLufs) distributionShortTerm_.addSample(shortTermLufs_);
            if (momentaryLufs_ >= DistributionHistogram::kMinLufs) distributionMomentary_.addSample(momentaryLufs_);
        }
    }

    double momentaryLufs() const { return momentaryLufs_; }
    double shortTermLufs() const { return shortTermLufs_; }
    double integratedLufs() const { return integratedHistogram_.gatedLoudness(-10.0); }
    double loudnessRangeLu() const
    {
        return shortTermHistogram_.percentile(0.95, -20.0) - shortTermHistogram_.percentile(0.10, -20.0);
    }

    double truePeakLDb() const { return truePeakL_.peakDb(); }
    double truePeakRDb() const { return truePeakR_.peakDb(); }
    double truePeakMaxDb() const { return std::max(truePeakL_.peakDb(), truePeakR_.peakDb()); }

    // Peak-to-Loudness Ratio / Peak-to-Short-term Ratio, both in LU --
    // "the difference between the peak level and integrated loudness" /
    // the same but against the live Short-term reading, per Youlean's own
    // definitions this plugin follows the units and naming of.
    double plrLu() const { return truePeakMaxDb() - integratedLufs(); }
    double psrLu() const { return truePeakMaxDb() - shortTermLufs(); }

    static constexpr int kDistributionBinCount = DistributionHistogram::kBinCount;
    double distributionShortTermCount(int bin) const { return distributionShortTerm_.count(bin); }
    double distributionMomentaryCount(int bin) const { return distributionMomentary_.count(bin); }

    // Clears the accumulated Integrated/LRA/Distribution/True-Peak-max
    // statistics (backs the UI's "Reset" button) without disturbing the
    // live Momentary/Short-term readings' own filter/window state --
    // matches Youlean's own "Reset" convention of clearing session
    // statistics, not restarting live metering.
    void resetStatistics()
    {
        integratedHistogram_.reset();
        shortTermHistogram_.reset();
        distributionShortTerm_.reset();
        distributionMomentary_.reset();
        truePeakL_.resetPeak();
        truePeakR_.resetPeak();
    }

    static constexpr double kSilenceLufs = -120.0;

private:
    double sampleRate_ = 48000.0;
    int blockPeriodSamples_ = 4800;
    int samplesSinceBlock_ = 0;

    KWeightingFilter kWeightL_, kWeightR_;
    SlidingMeanSquare momentary_, shortTerm_;
    LoudnessHistogram integratedHistogram_, shortTermHistogram_;
    DistributionHistogram distributionShortTerm_, distributionMomentary_;
    TruePeakDetector truePeakL_, truePeakR_;

    double momentaryLufs_ = kSilenceLufs;
    double shortTermLufs_ = kSilenceLufs;
};

} // namespace ssmeter
