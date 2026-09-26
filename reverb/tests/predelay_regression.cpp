// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// predelay_regression.cpp
//
// Correctness tests for plugin/dsp/PreDelayLine.hpp -- the Pre-Delay
// shared by every reverb model. This is the
// shared, model-independent piece (see that file's own header comment
// for why it's reused via #include across all three plugin_*/ targets
// rather than duplicated like the tank engines themselves).
//
//   g++ -std=c++17 -O2 -Wall -Wextra -I plugin/dsp tests/predelay_regression.cpp -o predelay_regression
//   ./predelay_regression

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "PreDelayLine.hpp"

using ssfilter::reverb::PreDelayLine;

static int gFailures = 0;
static void check(bool cond, const char *what) {
    if (!cond) {
        std::printf("FAIL: %s\n", what);
        ++gFailures;
    } else {
        std::printf("ok:   %s\n", what);
    }
}

int main() {
    const double fs = 48000.0;

    // [D1] Exact identity at 0ms, both immediately after prepare() and
    // after processing plenty of signal -- the "default changes nothing"
    // requirement every optional control in this project follows.
    {
        PreDelayLine pd;
        pd.prepare(fs, 250.0);
        std::vector<float> x(4000), orig;
        std::mt19937 rng(1);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        for (auto &v : x) v = (float)dist(rng);
        orig = x;
        pd.process(x.data(), (int)x.size());
        bool identical = true;
        for (size_t i = 0; i < x.size(); ++i) if (x[i] != orig[i]) identical = false;
        check(identical, "[D1] 0ms Pre-Delay is an exact, bit-identical passthrough");
    }

    // [D2] A nonzero setting produces the correct measured delay, once
    // the smoothing has settled -- fed an impulse after a silent
    // settling period, the output peak must land at the expected sample
    // offset.
    {
        PreDelayLine pd;
        pd.prepare(fs, 250.0);
        pd.setDelayMs(20.0); // 960 samples @ 48kHz
        std::vector<float> settle(20000, 0.0f);
        pd.process(settle.data(), (int)settle.size());
        std::vector<float> imp(2000, 0.0f);
        imp[0] = 1.0f;
        pd.process(imp.data(), (int)imp.size());
        int peakIdx = -1;
        float peakVal = 0.0f;
        for (size_t i = 0; i < imp.size(); ++i)
            if (std::fabs(imp[i]) > peakVal) { peakVal = std::fabs(imp[i]); peakIdx = (int)i; }
        std::printf("      expected ~960 samples, measured peak at %d (val=%.4f)\n", peakIdx, peakVal);
        check(peakIdx == 960, "[D2] 20ms Pre-Delay measures to exactly 960 samples once settled");
        check(peakVal > 0.99f, "[D2] the delayed impulse arrives at (near) full amplitude, not smeared away");
    }

    // [D3] A live delay-time change glides toward its target rather than
    // snapping instantly -- checked directly via currentDelaySamples(),
    // not inferred from output samples (a fair, un-gameable check:
    // whatever the underlying audio content happens to be, the smoothed
    // delay-in-samples value itself must still take real time to reach
    // its target, not jump there in one step).
    {
        PreDelayLine pd;
        pd.prepare(fs, 250.0); // starts at 0 samples (snapped)
        pd.setDelayMs(100.0);  // target jumps to 4800 samples
        std::vector<float> probe(2000, 0.0f);
        double afterOneSample = 0.0, afterFull = 0.0;
        bool monotonic = true, neverOvershoots = true;
        double prev = pd.currentDelaySamples();
        for (size_t i = 0; i < probe.size(); ++i) {
            pd.process(&probe[i], 1);
            const double cur = pd.currentDelaySamples();
            if (i == 0) afterOneSample = cur;
            if (cur < prev - 1e-9) monotonic = false;
            if (cur > 4800.0 + 1e-6) neverOvershoots = false;
            prev = cur;
        }
        afterFull = pd.currentDelaySamples();
        std::printf("      currentDelaySamples(): after 1 sample=%.2f, after %zu samples=%.2f (target=4800)\n",
                    afterOneSample, probe.size(), afterFull);
        check(afterOneSample < 4800.0 * 0.5, "[D3] one sample after a drastic retarget, the smoothed delay is still well short of the target (a glide, not a snap)");
        check(monotonic, "[D3] the smoothed delay moves monotonically toward its target, no overshoot-and-settle wobble");
        check(neverOvershoots, "[D3] the smoothed delay never exceeds its own target");
        check(afterFull > 4800.0 * 0.99, "[D3] given enough samples, the smoothed delay does converge to (within 1% of) its target");
    }

    // [D4] Clamped to the buffer's own max range -- setDelayMs() beyond
    // what prepare() allocated for must not read/write out of bounds
    // (checked indirectly: the tool must simply stay well-behaved/finite,
    // not crash or produce garbage).
    {
        PreDelayLine pd;
        pd.prepare(fs, 50.0); // only 50ms of buffer
        pd.setDelayMs(500.0); // way beyond it
        std::vector<float> settle(20000, 0.0f);
        pd.process(settle.data(), (int)settle.size());
        std::vector<float> imp(10000, 0.0f);
        imp[0] = 1.0f;
        pd.process(imp.data(), (int)imp.size());
        bool finite = true;
        for (float v : imp) if (!std::isfinite(v)) finite = false;
        check(finite, "[D4] requesting a delay beyond the prepared max range stays finite/well-behaved (clamped)");
    }

    // [D5] reset() clears buffered audio but leaves the delay-time
    // setting itself untouched -- matches every sibling Reset handler's
    // own "reset state, not settings" convention.
    {
        PreDelayLine pd;
        pd.prepare(fs, 250.0);
        pd.setDelayMs(20.0);
        std::vector<float> settle(20000, 0.7f);
        pd.process(settle.data(), (int)settle.size());
        pd.reset();
        std::vector<float> probe(10, 0.0f);
        pd.process(probe.data(), (int)probe.size());
        bool allSilentRightAfterReset = true;
        for (float v : probe) if (v != 0.0f) allSilentRightAfterReset = false;
        check(allSilentRightAfterReset, "[D5] reset() clears buffered audio (no leftover pre-delayed content bleeds through)");
    }

    std::printf("\n%s (%d failure%s)\n", gFailures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
                gFailures, gFailures == 1 ? "" : "s");
    return gFailures == 0 ? 0 : 1;
}
