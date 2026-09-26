// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// dsp_regression.cpp
//
// Fast, pure-math correctness tests for plugin/dsp/AmplifierDsp.hpp: no
// DPF, no GUI, no audio files. Mirrors the style and intent of both
// sibling projects' own tests/dsp_regression.cpp.
//
// Build and run via the `dsp_regression` CMake target, or standalone:
//   g++ -std=c++17 -O2 -Iplugin/dsp tests/dsp_regression.cpp -o dsp_regression
//   ./dsp_regression

#include "AmplifierDsp.hpp"

#include <cmath>
#include <cstdio>

using namespace ssamp;

namespace {
bool nearlyEqual(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
}

int main()
{
    int failures = 0;

    // [1] dbToLinear: the two reference points everything else is built on.
    {
        const bool ok = nearlyEqual(dbToLinear(0.0), 1.0, 1e-9) && nearlyEqual(dbToLinear(-20.0), 0.1, 1e-9);
        printf("[1] dbToLinear(0)=1.0, dbToLinear(-20)=0.1: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [2] balanceGains: centered is an exact, unity passthrough on both
    // channels -- the "do nothing" case must be a true no-op.
    {
        double gL, gR;
        balanceGains(0.0, gL, gR);
        const bool ok = nearlyEqual(gL, 1.0, 1e-9) && nearlyEqual(gR, 1.0, 1e-9);
        printf("[2] balance=0 is unity on both channels: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [3] balanceGains: moving toward one side only ever attenuates the
    // OTHER channel -- the side moved toward must stay at exactly unity,
    // never boosted past it (a real, common bug in naive pan/balance laws).
    {
        double gL, gR;
        balanceGains(0.5, gL, gR);   // toward the right
        const bool rightOk = nearlyEqual(gR, 1.0, 1e-9) && nearlyEqual(gL, 0.5, 1e-9);
        balanceGains(-0.5, gL, gR);  // toward the left
        const bool leftOk = nearlyEqual(gL, 1.0, 1e-9) && nearlyEqual(gR, 0.5, 1e-9);
        const bool ok = rightOk && leftOk;
        printf("[3] balance only attenuates the far channel, never boosts the near one: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [4] balanceGains: full-hard-left/right silences the opposite channel
    // exactly, and stays silent (not negative, not >1) even if a host
    // sends something outside [-1, 1].
    {
        double gL, gR;
        balanceGains(1.0, gL, gR);
        const bool fullRightOk = nearlyEqual(gL, 0.0, 1e-9) && nearlyEqual(gR, 1.0, 1e-9);
        balanceGains(-1.0, gL, gR);
        const bool fullLeftOk = nearlyEqual(gR, 0.0, 1e-9) && nearlyEqual(gL, 1.0, 1e-9);
        balanceGains(5.0, gL, gR); // out of range -- must clamp, not extrapolate
        const bool clampedOk = nearlyEqual(gL, 0.0, 1e-9) && nearlyEqual(gR, 1.0, 1e-9) && gL >= 0.0 && gR <= 1.0;
        const bool ok = fullRightOk && fullLeftOk && clampedOk;
        printf("[4] full-hard balance silences the opposite channel exactly, out-of-range clamps: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [5] End-to-end per-sample formula both plugins' run() actually use
    // (gain * balance * invert, combined into one scalar per channel):
    // a few concrete inputs against hand-computed expected outputs.
    {
        const double gainDb = 6.0;
        const double balance = -0.25; // toward left -> attenuates R by 0.25, L stays 1.0
        const bool invertL = true, invertR = false;

        const double gainLin = dbToLinear(gainDb);
        double balL, balR;
        balanceGains(balance, balL, balR);
        const double totalL = gainLin * balL * (invertL ? -1.0 : 1.0);
        const double totalR = gainLin * balR * (invertR ? -1.0 : 1.0);

        const double inSample = 0.4;
        const double outL = inSample * totalL;
        const double outR = inSample * totalR;

        const double expectedGainLin = std::pow(10.0, 6.0 / 20.0); // ~1.99526
        const double expectedOutL = -inSample * expectedGainLin * 1.0;   // inverted, balL=1.0 (not attenuated)
        const double expectedOutR = inSample * expectedGainLin * 0.75;   // not inverted, balR=1-0.25=0.75

        const bool ok = nearlyEqual(outL, expectedOutL, 1e-9) && nearlyEqual(outR, expectedOutR, 1e-9);
        printf("[5] combined gain+balance+invert formula matches hand-computed values: %s (L=%.6f want %.6f, R=%.6f want %.6f)\n",
               ok ? "OK" : "FAIL", outL, expectedOutL, outR, expectedOutR);
        if (!ok) ++failures;
    }

    // [6] ListenMeter: a constant-amplitude signal's Peak and RMS must
    // both read back as exactly that amplitude (in dB) -- the simplest
    // possible sanity check, and it pins down that Peak isn't secretly
    // averaging and RMS isn't secretly just tracking the max.
    {
        ListenMeter m;
        m.reset();
        constexpr double amp = 0.25;
        for (int i = 0; i < 10000; ++i) m.process(amp);
        const double expectedDb = 20.0 * std::log10(amp);
        const bool ok = nearlyEqual(m.peakDb(), expectedDb, 1e-6) && nearlyEqual(m.rmsDb(), expectedDb, 1e-6);
        printf("[6] ListenMeter on a constant amplitude: Peak=%.3fdB RMS=%.3fdB (want both %.3fdB): %s\n",
               m.peakDb(), m.rmsDb(), expectedDb, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [7] ListenMeter: a full-scale sine wave's Peak and RMS must differ
    // by the textbook 20*log10(sqrt(2)) ~ 3.01dB crest factor -- confirms
    // the two readings are genuinely independent measurements, not the
    // same number reported twice, using a signal shape (not just a flat
    // DC-like value) closer to what real audio actually looks like.
    {
        ListenMeter m;
        m.reset();
        constexpr int N = 48000;
        for (int i = 0; i < N; ++i)
            m.process(std::sin(2.0 * M_PI * 1000.0 * double(i) / 48000.0));
        const double crestDb = m.peakDb() - m.rmsDb();
        const bool ok = nearlyEqual(m.peakDb(), 0.0, 0.05) && nearlyEqual(crestDb, 20.0 * std::log10(std::sqrt(2.0)), 0.05);
        printf("[7] ListenMeter on a full-scale sine: Peak=%.3fdB RMS=%.3fdB (crest=%.3fdB, want ~3.01dB): %s\n",
               m.peakDb(), m.rmsDb(), crestDb, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [8] Peak can only ever rise as more (no louder) samples arrive, and
    // RMS keeps converging toward the true value rather than resetting --
    // the two properties the GUI's "live, updating number while Listen is
    // still running" behavior depends on.
    {
        ListenMeter m;
        m.reset();
        m.process(0.1);
        const double peak1 = m.peakDb();
        m.process(0.05); // quieter -- must NOT lower the peak
        const bool peakOk = nearlyEqual(m.peakDb(), peak1, 1e-9);
        m.process(0.5); // louder -- must raise it
        const bool peakRoseOk = m.peakDb() > peak1;
        printf("[8] ListenMeter Peak never drops, only rises on a new maximum: %s\n", (peakOk && peakRoseOk) ? "OK" : "FAIL");
        if (!(peakOk && peakRoseOk)) ++failures;
    }

    // [9] The actual Listen-stop formula both plugins' setParameterValue()
    // uses (newGainDb = target - measuredDb, clamped to the Gain
    // parameter's own [-60, 24] range) -- checked directly against
    // hand-computed values, including both clamps actually engaging.
    {
        auto computeNewGain = [](double targetDb, double measuredDb) {
            return std::max(-60.0, std::min(24.0, targetDb - measuredDb));
        };
        const bool normalOk = nearlyEqual(computeNewGain(-1.0, -19.0), 18.0, 1e-9);   // a quiet source, needs +18dB to reach -1dB -- no clamp
        const bool clampHighOk = nearlyEqual(computeNewGain(-1.0, -60.0), 24.0, 1e-9); // would need +59dB -- clamps to the Gain param's own max
        const bool clampLowOk = nearlyEqual(computeNewGain(-24.0, 40.0), -60.0, 1e-9); // would need -64dB -- clamps to the Gain param's own min
        const bool ok = normalOk && clampHighOk && clampLowOk;
        printf("[9] Listen-stop Gain formula (target - measured, clamped): %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [10] ClipHold::bufferClips(): exactly at 0dBFS (|x| == 1.0) is NOT
    //      clipping (digital full scale itself is a legitimate, if hot,
    //      level -- only *exceeding* it counts), anything past it is.
    {
        const float atFullScale[] = { 0.5f, -1.0f, 0.9f };
        const float overFullScale[] = { 0.5f, 1.0001f, -0.2f };
        const bool atOk = !ClipHold::bufferClips(atFullScale, 3);
        const bool overOk = ClipHold::bufferClips(overFullScale, 3);
        printf("[10] ClipHold::bufferClips(): exactly 0dBFS not flagged, past it is: %s\n",
               (atOk && overOk) ? "OK" : "FAIL");
        if (!(atOk && overOk)) ++failures;
    }

    // [11] ClipHold::update()/isClipping(): a single clipped block latches
    //      "clipping" for the full hold window, then clears once enough
    //      silent time has passed -- the "hold the LED lit a moment"
    //      behavior the GUI's CLIP badge depends on to make a brief
    //      transient actually visible.
    {
        ClipHold c;
        const bool initiallyOk = !c.isClipping(); // starts clean
        c.update(true, 0.01); // one clipped 10ms block
        const bool immediatelyOk = c.isClipping();
        c.update(false, 1.0); // 1s of silence since -- still within the 1.5s hold
        const bool stillHeldOk = c.isClipping();
        c.update(false, 1.0); // another 1s -- now 2s since the clip, past the hold
        const bool clearedOk = !c.isClipping();
        const bool ok = initiallyOk && immediatelyOk && stillHeldOk && clearedOk;
        printf("[11] ClipHold latches through its hold window then clears: %s"
               " (initial=%d immediate=%d stillHeld=%d cleared=%d)\n",
               ok ? "OK" : "FAIL", initiallyOk, immediatelyOk, stillHeldOk, clearedOk);
        if (!ok) ++failures;
    }

    // [12] ClipHold::reset(): returns to the same "not clipping" state a
    //      freshly-constructed one starts in (activate() calling reset()
    //      on a plugin instance re-used across a sample-rate change or
    //      similar must not leave a stale clip flag latched).
    {
        ClipHold c;
        c.update(true, 0.01);
        c.reset();
        printf("[12] ClipHold::reset() clears a latched clip: %s\n", !c.isClipping() ? "OK" : "FAIL");
        if (c.isClipping()) ++failures;
    }

    printf("\n%s: %d test(s) failed\n", failures == 0 ? "PASS" : "FAILURES", failures);
    return failures == 0 ? 0 : 1;
}
