// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// TruePeakLimiterDsp.hpp
//
// A second, genuinely different gain-computer for the Limiter plugin only
// (not shared by the Mono/Stereo compressor plugins, unlike CompressorDsp.hpp
// -- this is Limiter-specific, so it lives in its own file rather than
// growing that shared one). Built after directly, empirically comparing
// this project's existing Limiter (a one-pole attack/release envelope
// follower driving a fixed-ratio soft-knee curve -- CompressorVoice, in
// CompressorDsp.hpp) against LSP's own installed Limiter on identical
// input: raising the envelope-follower version's internal ratio from 20:1
// to 200:1 barely moved the numbers, which ruled out "the ratio isn't
// steep enough" and pointed at something structural -- LSP's algorithm
// holds the output peak almost exactly at its threshold even on a 2ms
// transient spike, ours does not, regardless of ratio. The reason: an
// envelope follower's output is a *smoothed lag* of the true level, so
// even a very fast attack still only approximates "the exact gain needed
// right now" -- it can't guarantee a hard ceiling by construction. This
// file is that different construction: a genuine sample-accurate
// lookahead peak-hold limiter, the standard architecture most dedicated
// brick-wall limiters (LSP's included) actually use.
//
// The idea, in one paragraph: reuse the SAME lookahead delay line the
// existing envelope-follower Limiter already has (PluginLimiter.hpp's
// delayBufL_/delayBufR_/delaySamples_) -- the audio path is held back by
// `lookaheadSamples` before reaching the output, exactly as before. The
// difference is what computes the gain applied to the sample being read
// out of that delay line each step: instead of smoothing the *level* and
// then reading a curve off the smoothed value, compute the linear gain
// each incoming sample would need on its own (ceilingLin / that sample's
// own level, clamped to <= 1.0), and apply to the delayed sample the
// MINIMUM of that value over the whole window from where it currently
// sits in the buffer up through the newest sample just pushed in -- i.e.
// "look at every sample this one will share the lookahead window with
// before it leaves, and never apply more gain than the worst (loudest) of
// them allows". That minimum is computed with a classic monotonic-deque
// sliding-window-minimum (see TruePeakGainComputer::process() below) --
// an ordinary, well-known O(1)-amortized technique, not anything
// experimental -- so it costs one array push/pop per sample, no
// allocation, safe for the audio thread.
//
// Why this actually guarantees the peak (a proof, not just an empirical
// observation): let g_target[n] be the minimum required-gain over the
// window [n-lookahead, n] once sample n has been pushed -- by
// construction, g_target[n] <= ceilingLin / |x[n-lookahead]| (n-lookahead
// is IN that window). The gain actually applied, appliedGain[n], is never
// allowed to exceed g_target[n] (see process()'s own release-smoothing
// comment: recovery toward 1.0 is smoothed, but a *drop* is always
// immediate and a *rise* is always capped at g_target[n], never above
// it). So appliedGain[n] * |x[n-lookahead]| <= g_target[n] *
// |x[n-lookahead]| <= ceilingLin, for every n. The output sample the
// caller actually writes is exactly appliedGain[n] applied to
// x[n-lookahead] (the same delay-line read the existing envelope-follower
// path already does) -- so it can never exceed ceilingLin, full stop, not
// "usually" or "unless the ratio is too shallow". Everything upstream of
// that guarantee (Attack, Knee) has no role to play here and is simply
// unused in this mode -- see PluginLimiter.cpp's own comment on why.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace sscomp {

// Identical formula to (private, so not reusable from here as-is)
// EnvelopeFollower::coeffFromMs() in CompressorDsp.hpp -- duplicated
// rather than exposed from that shared file, to avoid touching a file the
// Mono/Stereo compressor plugins also depend on for a change only this
// Limiter-only file needs.
inline double truePeakReleaseCoeffFromMs(double ms, double sampleRate)
{
    ms = std::max(0.01, ms);
    return std::exp(-1.0 / (0.001 * ms * sampleRate));
}

class TruePeakGainComputer
{
public:
    // `maxWindowSamples` should match the caller's own delay line's
    // maximum lookahead length (PluginLimiter's kMaxLookaheadMs' worth of
    // samples) -- this class's internal ring buffer is sized once here and
    // never reallocated on the audio thread afterward.
    void prepare(size_t maxWindowSamples)
    {
        capacity_ = maxWindowSamples + 4; // headroom: the deque can hold at most (window length + 1) entries, see file comment
        idxBuf_.assign(capacity_, int64_t(0));
        valBuf_.assign(capacity_, 1.0);
        reset();
    }

    void reset() noexcept
    {
        head_ = 0;
        count_ = 0;
        sampleCounter_ = 0;
        appliedGain_ = 1.0;
    }

    // One-pole coefficient, 0..1, governing how fast appliedGain_ is
    // allowed to *rise* back toward 1.0 once the window's own minimum
    // relaxes -- a *drop* is always applied immediately and in full (see
    // process()'s own comment for why that's required for the guarantee,
    // not just a stylistic choice). Same "coeffFromMs()" shape as
    // EnvelopeFollower's own release coefficient -- pass the identical
    // formula's result in.
    void setReleaseCoeff(double coeff) noexcept { releaseCoeff_ = coeff; }

    // `requiredGain` is this NEW sample's own linear gain need (already
    // computed by the caller: ceilingLin / max(levelLin, tiny), clamped to
    // <= 1.0). `windowSamples` is the caller's current lookahead length in
    // samples (may change live -- read fresh every call, no stale
    // assumption baked in). Returns the gain to apply to the sample
    // `windowSamples` positions behind the one just pushed -- i.e. exactly
    // the sample the caller is about to read out of its own delay line
    // this same step.
    inline double process(double requiredGain, int windowSamples) noexcept
    {
        // Standard monotonic-increasing-deque sliding-window minimum:
        // drop every trailing candidate this new, smaller-or-equal value
        // makes irrelevant (they can never be the window minimum again
        // while this one is still in range), then push it.
        while (count_ > 0 && valBuf_[backIndex()] >= requiredGain) --count_;
        pushBack(sampleCounter_, requiredGain);

        // Evict anything that has aged out of the window ending at the
        // sample just pushed.
        const int64_t windowStart = sampleCounter_ - int64_t(std::max(0, windowSamples));
        while (count_ > 0 && idxBuf_[head_] < windowStart)
        {
            head_ = next(head_);
            --count_;
        }

        const double target = (count_ > 0) ? valBuf_[head_] : 1.0;

        // The guarantee (see file comment): a rise is smoothed (avoids an
        // audible "gain snapping back up" click once a loud passage's
        // window clears), a drop is instant and uncapped -- appliedGain_
        // must never exceed `target`, in either branch, for the per-
        // sample bound the file comment proves to hold. Same retain-
        // coefficient convention as EnvelopeFollower's own one-pole
        // (coeff close to 1 == slow: `env_ = coeff*env_ + (1-coeff)*in`),
        // NOT a step-fraction multiplying the gap directly -- releaseCoeff_
        // comes straight from the identical coeffFromMs() formula, so it
        // must be used the identical way, or a "slow" release value would
        // instead act nearly instant -- exactly the failure mode
        // dsp_regression test [11] below checks against directly.
        if (target > appliedGain_) appliedGain_ = releaseCoeff_ * appliedGain_ + (1.0 - releaseCoeff_) * target;
        else appliedGain_ = target;

        ++sampleCounter_;
        return appliedGain_;
    }

private:
    size_t next(size_t i) const noexcept { return (i + 1 == capacity_) ? 0 : i + 1; }
    size_t backIndex() const noexcept { return (head_ + count_ - 1) % capacity_; }

    void pushBack(int64_t idx, double val) noexcept
    {
        const size_t pos = (head_ + count_) % capacity_;
        idxBuf_[pos] = idx;
        valBuf_[pos] = val;
        ++count_;
    }

    std::vector<int64_t> idxBuf_;
    std::vector<double> valBuf_;
    size_t capacity_ = 0;
    size_t head_ = 0;
    size_t count_ = 0;
    int64_t sampleCounter_ = 0;
    double releaseCoeff_ = 0.0;
    double appliedGain_ = 1.0;
};

} // namespace sscomp
