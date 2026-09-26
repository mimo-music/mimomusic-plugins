// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ReferenceEngine.hpp
//
// Renders a dry excitation signal through the higher-order image method
// (ImageSourceMethod.hpp) to produce a stereo "reference" wet signal --
// the offline, expensive, physically-grounded ground truth
// tools/reference/generate_reference_samples.cpp writes out as training
// material.
//
// Deliberately does NOT impose a synthetic decay envelope or a "Decay
// Time"-style parameter on top of the render -- the entire point of this
// engine is to be a genuine physical ground truth, and fabricating a
// tail shape that doesn't emerge from the actual geometry/absorption
// would quietly turn it back into just another parametrized approximation,
// the exact thing it exists to be independent of. What you get is
// whatever the requested room + wall absorption + image order actually
// produce: at low/moderate order, a physically exact but SPARSE cloud of
// discrete early reflections (still useful on its own, e.g. as an
// early-reflections-only training target); only at high order does the
// image count become dense enough to blur into something that resembles
// a full, smoothly-decaying tail (image-source theory: this converges
// toward the statistical/Sabine diffuse-field prediction as order -> infinity)
// -- which is exactly the "hours of calculation" case the user explicitly
// opted into, not a shortcut around it.
//
// Stereo comes from two distinct receiver points (an "ear spacing" apart
// along the room's own width axis), each hearing the same source via
// genuinely different path geometry -- the physically correct way to get
// stereo from a room, and a different mechanism from the mid/side trick
// reverb/reverb_dense/reverb_plate all use on their own single internal
// signal.

#pragma once

#include "ImageSourceMethod.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace ssfilter {
namespace reverb {
namespace reference {

struct ReferenceRenderResult {
    std::vector<float> wetL, wetR;
    size_t tapCountL = 0, tapCountR = 0;
    double longestTapDelaySec = 0.0;
};

// Average human inter-ear spacing, used only to place the two receiver
// points -- not a claim of real binaural/HRTF modeling, just two
// distinct physical listening points in the same room.
constexpr double kEarSpacingM = 0.17;

inline ReferenceRenderResult renderReferenceStereo(
    const std::vector<float> &dry, double sampleRate,
    const RoomDims &room, double wallAbsorption01, int maxOrder,
    bool verboseProgress = true) {
    ReferenceRenderResult result;

    const auto tapsL = higherOrderImageSources(room, wallAbsorption01, maxOrder, -kEarSpacingM * 0.5);
    const auto tapsR = higherOrderImageSources(room, wallAbsorption01, maxOrder, +kEarSpacingM * 0.5);
    result.tapCountL = tapsL.size();
    result.tapCountR = tapsR.size();

    double longestDelay = 0.0;
    for (const auto &t : tapsL) longestDelay = std::max(longestDelay, t.delaySec);
    for (const auto &t : tapsR) longestDelay = std::max(longestDelay, t.delaySec);
    result.longestTapDelaySec = longestDelay;

    const size_t outLen = dry.size() + (size_t)std::ceil(longestDelay * sampleRate) + 16;
    result.wetL.assign(outLen, 0.0f);
    result.wetR.assign(outLen, 0.0f);

    if (verboseProgress) {
        std::printf("    image taps: L=%zu R=%zu (order<=%d), longest delay=%.1fms, render length=%.2fs\n",
                    result.tapCountL, result.tapCountR, maxOrder, longestDelay * 1000.0, (double)outLen / sampleRate);
    }

    auto accumulate = [&](const std::vector<EarlyReflection> &taps, std::vector<float> &dst) {
        size_t done = 0;
        for (const auto &t : taps) {
            const size_t delaySamples = (size_t)std::lround(t.delaySec * sampleRate);
            const float gain = (float)t.gain;
            for (size_t i = 0; i < dry.size(); ++i) dst[i + delaySamples] += dry[i] * gain;
            if (verboseProgress && taps.size() > 5000) {
                ++done;
                if (done % 200000 == 0)
                    std::printf("    ...%zu/%zu taps accumulated\n", done, taps.size());
            }
        }
    };
    accumulate(tapsL, result.wetL);
    accumulate(tapsR, result.wetR);

    return result;
}

} // namespace reference
} // namespace reverb
} // namespace ssfilter
