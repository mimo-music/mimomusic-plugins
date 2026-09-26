// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// PreDelayLine.hpp
//
// A pure delay applied to the WET signal's own input, before it ever
// reaches whichever tank engine sits behind it, shared by every reverb
// model. Already named as a
// planned-but-unwired parameter in docs/REVERB_CONCEPT.md's own
// parameter table since this project's first pass ("Pre-Delay (ms) --
// pure delay before the early-reflection tap list and the tank's own
// input (not yet wired, see 'Phase 2')"); this is that wiring.
//
// Deliberately model-independent and reused via #include across both
// plugin_*/ targets, NOT duplicated per model the way each
// ReverbEngine*.hpp's own class body is (see e.g.
// ReverbEngineDense.hpp's own header comment for why THOSE are
// deliberate copies). There is nothing here that differs per model: a
// pre-delay is the same "hold the wet signal back N samples before the
// tank ever sees it" operation regardless of which tank it feeds, so
// sharing it costs nothing and duplicating it would only be needless
// risk of the two copies drifting apart.
//
// Applied ONLY to a copy of the signal that feeds the tank, never to the
// dry path -- the entire point of pre-delay is that the dry sound
// arrives immediately and the reverb "blooms" after a gap; see each
// PluginReverb*::run()'s own comment at the call site for how the two
// paths are kept separate.
//
// std::vector-backed (not a fixed std::array like the engines' own
// tank buffers) because its size depends on sample rate x a user-facing
// GUI range (0-250ms), and resizing only ever happens from
// sampleRateChanged() (message-thread/setup-time, never the audio
// thread) -- the same "no allocation where the audio thread could ever
// see it" discipline every fixed-array buffer elsewhere in this project
// follows, just satisfied by "resize only off the audio thread" rather
// than "never resize at all", since nothing on the audio thread itself
// can ever trigger a pre-delay resize (only sampleRateChanged() can).
//
// The delay amount itself IS safe to change from any calling context
// live, including mid-block: it glides (5ms one-pole smoothing, the same
// time constant OnePoleDamping's own coefficient smoothing and
// ReverbEngine's own duck timing both use) toward its target and reads
// via linear interpolation at the smoothed (generally fractional)
// position -- without this, an abrupt delay-time change would read a
// suddenly-different buffer position and click/zipper, the well-known
// failure mode of any live-changeable delay line. An exact identity
// (bit-for-bit passthrough) at 0ms, both at rest and immediately after
// prepare() -- verified directly in tests/predelay_regression.cpp, not
// just argued here.

#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace ssfilter {
namespace reverb {

class PreDelayLine {
public:
    void prepare(double sampleRate, double maxDelayMs) {
        sampleRate_ = sampleRate;
        smoothCoeff_ = 1.0 - std::exp(-1.0 / (kSmoothTimeConstantSec * sampleRate));
        const int maxSamples = std::max(2, (int)std::lround(maxDelayMs * 0.001 * sampleRate) + 2);
        buf_.assign((size_t)maxSamples, 0.0f);
        writeIdx_ = 0;
        setDelayMs(delayMsLast_); // re-clamp the current setting to the new buffer size
        delaySamplesSmoothed_ = delaySamplesTarget_; // instant on prepare() -- nothing to click yet, buffer is silent
    }

    // Clears the buffered audio only -- the delay time setting itself
    // (target and smoothed) is untouched, matching every sibling Reset
    // handler's own "reset state, not settings" convention (e.g.
    // ReverbEngine::prepare()'s own preservation of lastT60Dc_/Ny_ across
    // a room-size reset).
    void reset() { std::fill(buf_.begin(), buf_.end(), 0.0f); }

    // The current, smoothed (generally fractional) delay-in-samples --
    // exposed for verification only (tests/predelay_regression.cpp's own
    // [D3]), the same "return internal state for direct measurement"
    // convention ReverbEngine::currentT60DcSec() already follows.
    double currentDelaySamples() const { return delaySamplesSmoothed_; }

    void setDelayMs(double ms) {
        delayMsLast_ = std::max(0.0, ms);
        const double maxSamples = buf_.empty() ? 0.0 : (double)(buf_.size() - 2);
        delaySamplesTarget_ = std::clamp(delayMsLast_ * 0.001 * sampleRate_, 0.0, std::max(0.0, maxSamples));
    }

    // In-place, one channel at a time (call once per channel -- each
    // channel needs its own PreDelayLine instance, same as every other
    // per-channel processing stage in this project).
    void process(float *chan, int numFrames) {
        if (buf_.empty()) return;
        const int n = (int)buf_.size();
        for (int i = 0; i < numFrames; ++i) {
            delaySamplesSmoothed_ += smoothCoeff_ * (delaySamplesTarget_ - delaySamplesSmoothed_);

            const float x = chan[i];
            buf_[writeIdx_] = x;

            double readPos = (double)writeIdx_ - delaySamplesSmoothed_;
            while (readPos < 0.0) readPos += (double)n;
            const int i0 = (int)readPos % n;
            const int i1 = (i0 + 1) % n;
            const double frac = readPos - std::floor(readPos);
            chan[i] = (float)(buf_[i0] * (1.0 - frac) + buf_[i1] * frac);

            writeIdx_ = (writeIdx_ + 1 >= n) ? 0 : writeIdx_ + 1;
        }
    }

private:
    static constexpr double kSmoothTimeConstantSec = 0.005;

    std::vector<float> buf_;
    int writeIdx_ = 0;
    double sampleRate_ = 48000.0;
    double smoothCoeff_ = 0.0;
    double delayMsLast_ = 0.0;
    double delaySamplesTarget_ = 0.0;
    double delaySamplesSmoothed_ = 0.0;
};

} // namespace reverb
} // namespace ssfilter
