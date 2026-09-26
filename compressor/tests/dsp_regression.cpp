// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// dsp_regression.cpp
//
// Fast, pure-math correctness tests for plugin/dsp/CompressorDsp.hpp: no
// DPF, no GUI, no audio files -- just the gain-computer curve, envelope
// follower, loudness/compensation math and the Mid/Side algebra exercised
// directly and checked against known-correct values. Runs in well under a
// second. Mirrors the style and intent of
// ../statespace/tests/dsp_regression.cpp.
//
// Build and run via the `dsp_regression` CMake target, or standalone:
//   g++ -std=c++17 -O2 -Iplugin/dsp tests/dsp_regression.cpp -o dsp_regression
//   ./dsp_regression

#include "CompressorDsp.hpp"
#include "CurveDsp.hpp"
#include "TruePeakLimiterDsp.hpp"

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace sscomp;

namespace {
bool nearlyEqual(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
}

int main()
{
    int failures = 0;

    // [1] Below the knee: untouched (0 dB reduction), exactly -- with
    // ratioUp=1.0 (the historical default, no upward compression).
    {
        const double gr = computeGainReductionDb(-40.0, -18.0, 4.0, 1.0, 6.0); // well under thr-knee/2 = -21
        const bool ok = nearlyEqual(gr, 0.0, 1e-9);
        printf("[1] below-knee gain reduction is exactly 0dB: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [2] Above the knee: matches the textbook straight ratio-line formula
    // y = thr + (x-thr)/ratio  =>  reduction = y - x.
    {
        const double thr = -18.0, ratio = 4.0, knee = 6.0;
        const double x = 0.0; // well above thr+knee/2 = -15
        const double expectedOutDb = thr + (x - thr) / ratio;
        const double gr = computeGainReductionDb(x, thr, ratio, 1.0, knee);
        const bool ok = nearlyEqual(gr, expectedOutDb - x, 1e-9);
        printf("[2] above-knee gain reduction matches straight ratio line: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [3] Knee continuity: the quadratic knee segment must meet both
    // straight lines exactly at its own edges (thr-knee/2 and thr+knee/2)
    // -- a discontinuous knee would be an audible, visible kink.
    {
        const double thr = -18.0, ratio = 4.0, knee = 6.0;
        const double lo = thr - knee * 0.5, hi = thr + knee * 0.5;
        const double grAtLoFromBelow = computeGainReductionDb(lo - 0.0001, thr, ratio, 1.0, knee); // ~0
        const double grAtLoFromKnee = computeGainReductionDb(lo + 0.0001, thr, ratio, 1.0, knee);
        const double grAtHiFromKnee = computeGainReductionDb(hi - 0.0001, thr, ratio, 1.0, knee);
        const double grAtHiFromAbove = computeGainReductionDb(hi + 0.0001, thr, ratio, 1.0, knee);
        const bool ok = nearlyEqual(grAtLoFromBelow, grAtLoFromKnee, 0.01) &&
                         nearlyEqual(grAtHiFromKnee, grAtHiFromAbove, 0.01);
        printf("[3] knee segment meets both straight lines continuously: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [4] Ratio = 1:1 (both down and up) means no compression at any
    // level, full stop.
    {
        bool ok = true;
        for (double x = -60.0; x <= 6.0; x += 3.0)
            if (!nearlyEqual(computeGainReductionDb(x, -18.0, 1.0, 1.0, 6.0), 0.0, 1e-9)) ok = false;
        printf("[4] ratio 1:1 never reduces gain: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [4b] Upward compression: below the threshold, a quiet level should
    // be *boosted* (gain > 0) by an amount matching the mirrored straight
    // line y = thr - (thr-x)/ratioUp, symmetric to test [2]'s downward
    // case; above the threshold with ratioDown=1 it must stay untouched
    // (isolating the upward side, the same way test [1] isolates the
    // downward side against ratioUp=1). Also checks the knee is
    // continuous on this side too, mirroring test [3].
    {
        const double thr = -18.0, ratioUp = 4.0, knee = 6.0;
        const double x = -40.0; // well below thr-knee/2 = -21
        const double expectedOutDb = thr - (thr - x) / ratioUp;
        const double gr = computeGainReductionDb(x, thr, 1.0, ratioUp, knee);
        const bool boostedCorrectly = nearlyEqual(gr, expectedOutDb - x, 1e-9) && gr > 0.01;
        const bool aboveUntouched = nearlyEqual(computeGainReductionDb(0.0, thr, 1.0, ratioUp, knee), 0.0, 1e-9);
        const double lo = thr - knee * 0.5, hi = thr + knee * 0.5;
        const bool kneeContinuous =
            nearlyEqual(computeGainReductionDb(lo - 0.0001, thr, 1.0, ratioUp, knee),
                        computeGainReductionDb(lo + 0.0001, thr, 1.0, ratioUp, knee), 0.01) &&
            nearlyEqual(computeGainReductionDb(hi - 0.0001, thr, 1.0, ratioUp, knee),
                        computeGainReductionDb(hi + 0.0001, thr, 1.0, ratioUp, knee), 0.01);
        const bool ok = boostedCorrectly && aboveUntouched && kneeContinuous;
        printf("[4b] upward compression boosts below threshold, symmetric to downward, knee continuous: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [5] EnvelopeFollower: a linear-domain one-pole should converge
    // exponentially towards a suddenly-applied constant input, reaching
    // 1-e^-1 (~63.2%) of the way there after exactly one time constant's
    // worth of samples, matching the closed-form one-pole step response.
    {
        EnvelopeFollower env;
        constexpr double sr = 48000.0;
        constexpr double attackMs = 10.0;
        env.prepare(sr);
        env.setTimesMs(attackMs, 300.0);
        const int nSamples = int(0.001 * attackMs * sr); // one time constant, in samples
        for (int i = 0; i < nSamples; ++i) env.process(1.0);
        const double expected = 1.0 - std::exp(-1.0);
        const bool ok = nearlyEqual(env.value(), expected, 0.01);
        printf("[5] envelope follower reaches 1-e^-1 after one attack time constant: %s (got %.4f, want %.4f)\n",
               ok ? "OK" : "FAIL", env.value(), expected);
        if (!ok) ++failures;
    }

    // [6] LoudnessMeter: a constant (DC) input's RMS is just its own
    // magnitude -- after enough samples to settle, rmsDb() should read
    // back linearToDb(amplitude).
    {
        LoudnessMeter meter;
        constexpr double sr = 48000.0;
        meter.prepare(sr);
        constexpr double amplitude = 0.25;
        for (int i = 0; i < int(sr * 2.0); ++i) meter.process(amplitude); // 2s, several integration-time-constants
        const double expectedDb = linearToDb(amplitude);
        const bool ok = nearlyEqual(meter.rmsDb(), expectedDb, 0.05);
        printf("[6] loudness meter settles to the correct dB for a constant input: %s (got %.3f, want %.3f)\n",
               ok ? "OK" : "FAIL", meter.rmsDb(), expectedDb);
        if (!ok) ++failures;
    }

    // [7] Mid/Side is a lossless, exact algebraic transform when nothing
    // in between changes the signal (gain reduction = unity): reconstructing
    // L/R from Mid/Side must recover the *exact* original samples. This is
    // the identity PluginCompressorStereo's MidSide mode relies on --
    // a sign or scale slip here would leak crosstalk between channels even
    // with the compressor doing nothing.
    {
        bool ok = true;
        for (double L = -1.0; L <= 1.0; L += 0.137)
            for (double R = -1.0; R <= 1.0; R += 0.211)
            {
                const double mid = (L + R) * 0.5;
                const double side = (L - R) * 0.5;
                const double Lr = mid + side;
                const double Rr = mid - side;
                if (!nearlyEqual(Lr, L, 1e-9) || !nearlyEqual(Rr, R, 1e-9)) ok = false;
            }
        printf("[7] Mid/Side reconstruction is lossless: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [8] CompressorVoice end-to-end self-consistency: feeding a constant
    // level well above threshold until the envelope settles, the reported
    // gainReductionDb() must equal computeGainReductionDb() evaluated at
    // the voice's own reported (settled) detector level -- i.e. the class
    // wrapper doesn't do anything beyond "envelope, then the same curve".
    {
        CompressorVoice voice;
        constexpr double sr = 48000.0;
        voice.prepare(sr);
        voice.setParams(5.0, 100.0, -18.0, 4.0, 1.0, 6.0);
        constexpr double level = 0.5; // well above threshold
        double gr = 0.0;
        for (int i = 0; i < int(sr * 0.5); ++i) gr = voice.process(level); // 0.5s, settles well within a 5ms attack
        const double expectedGr = computeGainReductionDb(voice.detectorLevelDb(), -18.0, 4.0, 1.0, 6.0);
        const bool ok = nearlyEqual(voice.gainReductionDb(), expectedGr, 1e-9) &&
                        nearlyEqual(dbToLinear(voice.gainReductionDb()), gr, 1e-9) &&
                        voice.gainReductionDb() < -0.01; // must actually be compressing
        printf("[8] CompressorVoice matches the static curve at its own settled level: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9] TruePeakGainComputer's whole reason to exist: the hard
    // guarantee TruePeakLimiterDsp.hpp's file comment proves on paper --
    // output = appliedGain[n] * x[n-window] must never exceed thresholdLin
    // -- checked here directly against real (pseudo-random, spiky) data,
    // not just asserted in a comment. Mirrors PluginLimiter.cpp's own
    // integration (a same-length delay line, read/write every sample)
    // closely enough to be a faithful stand-in for the real call site.
    {
        constexpr double sr = 48000.0;
        constexpr int window = int(0.005 * sr); // 5ms lookahead, PluginLimiter's own default
        constexpr double thresholdDb = -6.0;
        const double thresholdLin = dbToLinear(thresholdDb);

        TruePeakGainComputer comp;
        comp.prepare(size_t(window) + 4);
        comp.setReleaseCoeff(truePeakReleaseCoeffFromMs(100.0, sr));

        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> quiet(-0.05, 0.05);
        std::uniform_real_distribution<double> spikeChance(0.0, 1.0);
        std::uniform_real_distribution<double> spikeAmp(0.5, 4.0); // well over threshold (~0.5 linear)

        constexpr int N = 200000;
        std::vector<double> signal(N);
        for (int i = 0; i < N; ++i)
            signal[i] = (spikeChance(rng) < 0.002 ? spikeAmp(rng) : 1.0) * quiet(rng) * (spikeChance(rng) < 0.5 ? 1.0 : -1.0);
        // Force a few guaranteed, deliberately extreme spikes so the test
        // doesn't depend on the RNG happening to produce one.
        signal[1000] = 9.0; signal[50000] = -12.0; signal[150000] = 20.0;

        std::vector<double> delayBuf(size_t(window) + 1, 0.0);
        size_t pos = 0;
        double worstOvershoot = 0.0;
        bool everActuallyLimited = false;

        for (int n = 0; n < N; ++n)
        {
            const double level = std::fabs(signal[n]);
            const double requiredGain = std::min(1.0, thresholdLin / std::max(level, 1e-9));
            const double gain = comp.process(requiredGain, window);
            if (gain < 0.999) everActuallyLimited = true;

            const double bufLen = double(delayBuf.size());
            delayBuf[pos] = signal[n];
            const size_t readPos = (pos + delayBuf.size() - size_t(window)) % delayBuf.size();
            const double delayedSample = delayBuf[readPos];
            pos = (pos + 1) % delayBuf.size();
            (void)bufLen;

            const double out = std::fabs(delayedSample * gain);
            worstOvershoot = std::max(worstOvershoot, out - thresholdLin);
        }

        const bool ok = worstOvershoot <= 1e-9 && everActuallyLimited;
        printf("[9] TruePeakGainComputer never exceeds threshold (worst overshoot=%.3e, want <=0): %s\n",
               worstOvershoot, ok ? "OK" : "FAIL");
        if (!ok) { printf("FAIL: this is the entire point of this algorithm -- see TruePeakLimiterDsp.hpp's proof\n"); ++failures; }
    }

    // [10] The release side is genuinely smoothed, not an instant snap
    // back to unity the moment a loud spike leaves the lookahead window --
    // otherwise "Release" would be a dead control in this mode. Feed one
    // isolated loud spike, then silence, and confirm the gain's recovery
    // trajectory afterward matches the same closed-form one-pole step
    // response test [5] already checks for EnvelopeFollower (1-e^-1 after
    // one time constant), not a step function.
    {
        constexpr double sr = 48000.0;
        constexpr int window = 0; // isolates the release side: gain can react on the very same sample, so this is purely a release-recovery test after the spike
        constexpr double thresholdDb = -6.0;
        constexpr double releaseMs = 50.0;
        const double thresholdLin = dbToLinear(thresholdDb);

        TruePeakGainComputer comp;
        comp.prepare(4);
        comp.setReleaseCoeff(truePeakReleaseCoeffFromMs(releaseMs, sr));

        // One sample at 10x threshold, forcing gain down to exactly 0.1 (window=0 -> instant, uncapped drop), then silence.
        double gain = comp.process(std::min(1.0, thresholdLin / (thresholdLin * 10.0)), window);
        const bool droppedExactly = nearlyEqual(gain, 0.1, 1e-9);

        const int oneTimeConstantSamples = int(0.001 * releaseMs * sr);
        for (int i = 0; i < oneTimeConstantSamples; ++i)
            gain = comp.process(1.0, window); // silence: requiredGain is always 1.0 (no new peak to hold it down)

        // Closed-form one-pole step response toward the ceiling (1.0),
        // starting from 0.1: gain(t) = 1.0 - (1.0-0.1)*e^-(t/tau).
        const double expected = 1.0 - 0.9 * std::exp(-1.0);
        const bool recoveredGradually = nearlyEqual(gain, expected, 0.01) && gain < 0.999; // not already snapped back to 1.0
        const bool ok = droppedExactly && recoveredGradually;
        printf("[10] release recovery follows the same one-pole curve as EnvelopeFollower's own: %s (got %.4f, want %.4f)\n",
               ok ? "OK" : "FAIL", gain, expected);
        if (!ok) ++failures;
    }

    // ---- Contour (plugin_contour): TransferCurve, computeCurveGainDb(),
    //      CurveVoice -- see plugin/dsp/CurveDsp.hpp's own header comment
    //      for the full design.

    // [11] Default curve is an exact identity (2 points, both endpoints
    //      at Y=X) -- "no change at all" is the safe starting shape.
    //      Endpoints' own X never moves, even when asked to.
    {
        TransferCurve curve;
        const bool defaultsOk = curve.pointCount() == 2 &&
                                 nearlyEqual(curve.point(0).inputDb, TransferCurve::kMinDb, 1e-9) &&
                                 nearlyEqual(curve.point(0).outputDb, TransferCurve::kMinDb, 1e-9) &&
                                 nearlyEqual(curve.point(1).inputDb, TransferCurve::kMaxDb, 1e-9) &&
                                 nearlyEqual(curve.point(1).outputDb, TransferCurve::kMaxDb, 1e-9);
        const bool identityOk = nearlyEqual(curve.evaluate(-30.0), -30.0, 1e-6) &&
                                 nearlyEqual(curve.evaluate(TransferCurve::kMinDb), TransferCurve::kMinDb, 1e-9) &&
                                 nearlyEqual(curve.evaluate(TransferCurve::kMaxDb), TransferCurve::kMaxDb, 1e-9);

        // Try to move the left endpoint's own X -- must be ignored.
        curve.setPoint(0, -10.0, -50.0);
        const bool endpointXFixed = nearlyEqual(curve.point(0).inputDb, TransferCurve::kMinDb, 1e-9) &&
                                     nearlyEqual(curve.point(0).outputDb, -50.0, 1e-9);

        const bool ok = defaultsOk && identityOk && endpointXFixed;
        printf("[11] TransferCurve default is an exact identity; endpoint X can't be moved: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [12] addPoint()/removePoint(): sorted insertion, refuses a point
    //      too close to a neighbor, refuses removing either fixed
    //      endpoint, accepts removing an interior point.
    {
        TransferCurve curve;
        const int idx1 = curve.addPoint(-30.0, -20.0); // between the two endpoints
        const bool insertedSorted = idx1 == 1 && curve.pointCount() == 3 &&
                                     nearlyEqual(curve.point(0).inputDb, TransferCurve::kMinDb, 1e-9) &&
                                     nearlyEqual(curve.point(1).inputDb, -30.0, 1e-9) &&
                                     nearlyEqual(curve.point(2).inputDb, TransferCurve::kMaxDb, 1e-9);

        const int idx2 = curve.addPoint(-30.2, 0.0); // 0.2dB from the point just added -- under kMinSeparationDb
        const bool tooCloseRefused = idx2 == -1 && curve.pointCount() == 3;

        curve.removePoint(0); // the left endpoint -- must refuse
        curve.removePoint(curve.pointCount() - 1); // the right endpoint -- must refuse
        const bool endpointsSurvived = curve.pointCount() == 3;

        curve.removePoint(1); // the interior point -- must succeed
        const bool interiorRemoved = curve.pointCount() == 2 &&
                                      nearlyEqual(curve.point(0).inputDb, TransferCurve::kMinDb, 1e-9) &&
                                      nearlyEqual(curve.point(1).inputDb, TransferCurve::kMaxDb, 1e-9);

        const bool ok = insertedSorted && tooCloseRefused && endpointsSurvived && interiorRemoved;
        printf("[12] TransferCurve addPoint()/removePoint(): sorted insert, too-close refused, "
               "endpoints unremovable, interior point removable: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [13] A corner-only curve (every point's handle left at its default
    //      (0,0)) is an exact lerp between the two bracketing points, and
    //      flat-extrapolates beyond either end -- the old curve-wide
    //      "Linear" mode is now just what every point already defaults to,
    //      not a separate setting (see plugin/dsp/CurveDsp.hpp's own
    //      header comment on "per-point interpolation").
    {
        TransferCurve curve;
        curve.setPoint(0, 0.0, -60.0); // left endpoint: output -60
        curve.setPoint(1, 0.0, 0.0);   // right endpoint: output 0 (X pinned to kMaxDb regardless)
        // Now a straight line from (-60,-60) to (6,0).
        const double midDb = 0.5 * (TransferCurve::kMinDb + TransferCurve::kMaxDb);
        const double expectedMid = 0.5 * (-60.0 + 0.0);
        const bool midOk = nearlyEqual(curve.evaluate(midDb), expectedMid, 1e-6);
        const bool belowOk = nearlyEqual(curve.evaluate(TransferCurve::kMinDb - 20.0), -60.0, 1e-9); // flat below
        const bool aboveOk = nearlyEqual(curve.evaluate(TransferCurve::kMaxDb + 20.0), 0.0, 1e-9);   // flat above
        const bool ok = midOk && belowOk && aboveOk;
        printf("[13] TransferCurve corner-only curve: exact lerp mid-segment, flat extrapolation beyond both ends: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [14] A point given a tangent handle (setPointHandle()) still passes
    //      through its own exact value, but the resulting Bezier segment
    //      CAN now overshoot past a neighboring point's own value on
    //      purpose -- the deliberate opposite of the old monotone-Hermite
    //      spline's own guarantee (see TransferCurve::setPointHandle()'s
    //      own comment on why: a hand-placed handle overshooting is the
    //      entire reason to use one). Also checks the handle's own X
    //      offset is clamped so the segment can never double back on X
    //      (the one constraint that *is* still enforced, needed for
    //      evaluate() to remain a valid function of X).
    {
        TransferCurve curve;
        const int mid = curve.addPoint(-20.0, -10.0);
        curve.setPointHandle(mid, 5.0, 20.0); // a big Y handle, deliberately steep
        const bool passesThroughPoint = nearlyEqual(curve.evaluate(-20.0), -10.0, 1e-6);

        double maxY = -1000.0;
        for (double x = -20.0; x <= -19.0; x += 0.05) maxY = std::max(maxY, curve.evaluate(x));
        const bool overshoots = maxY > -9.5; // bulges well past the point's own -10 value, on purpose

        TransferCurve clampTest;
        const int c = clampTest.addPoint(0.0, 0.0);
        clampTest.setPointHandle(c, 1000.0, 1000.0); // absurd request
        const double gapToRight = TransferCurve::kMaxDb - 0.0, gapToLeft = 0.0 - TransferCurve::kMinDb;
        const bool clamped = clampTest.point(c).handleInputDb <= std::min(gapToRight, gapToLeft) + 1e-9;

        const bool ok = passesThroughPoint && overshoots && clamped;
        printf("[14] TransferCurve tangent handle: passes exactly through its own point, "
               "overshoots a neighbor's value on purpose, X offset clamped to the neighbor gap: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [15] computeCurveGainDb(): Above condition gives exactly 0 below
    //      threshold and the curve's own gap above it; Below condition is
    //      the mirror image -- confirms the trigger, not the curve
    //      itself, is what decides which side is ever actually audible.
    {
        TransferCurve curve;
        curve.setPoint(0, 0.0, -60.0); // heavy attenuation at the very bottom -- a gate-like shape
        curve.setPoint(1, 0.0, 0.0);   // unchanged at the very top

        const double thresholdDb = -20.0;
        const bool aboveOk =
            nearlyEqual(computeCurveGainDb(-30.0, thresholdDb, TriggerCondition::Above, curve), 0.0, 1e-9) && // below threshold: untouched
            !nearlyEqual(computeCurveGainDb(-10.0, thresholdDb, TriggerCondition::Above, curve), 0.0, 1e-3);  // above threshold: curve applies
        const bool belowOk =
            !nearlyEqual(computeCurveGainDb(-30.0, thresholdDb, TriggerCondition::Below, curve), 0.0, 1e-3) && // below threshold: curve applies (a gate closing)
            nearlyEqual(computeCurveGainDb(-10.0, thresholdDb, TriggerCondition::Below, curve), 0.0, 1e-9);    // above threshold: untouched (gate open)

        const bool ok = aboveOk && belowOk;
        printf("[15] computeCurveGainDb(): Above/Below conditions gate which side of threshold "
               "the curve actually reaches: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [16] CurveVoice end to end: a sustained tone well past the trigger
    //      settles at exactly the curve's own gap (same "run it long
    //      enough to settle, then check against the closed form"
    //      methodology test [8]/[10] already use), for both an Above
    //      (downward-compression-shaped curve) and a Below (gate-shaped
    //      curve) scenario -- proving the full envelope+trigger+curve
    //      pipeline agrees with the pure computeCurveGainDb() formula,
    //      not just the formula in isolation.
    {
        constexpr double sr = 48000.0;
        auto settle = [&](TriggerCondition cond, TransferCurve& curve, double thresholdDb, double toneDb) {
            CurveVoice voice;
            voice.prepare(sr);
            voice.setParams(3.0, 30.0, thresholdDb, cond);
            const double amp = std::pow(10.0, toneDb / 20.0);
            double g = 1.0;
            for (int i = 0; i < int(sr * 0.5); ++i) g = voice.process(amp, curve); // well past both attack and release times
            return linearToDb(g);
        };

        TransferCurve downCurve;
        downCurve.setPoint(0, 0.0, TransferCurve::kMinDb);
        downCurve.setPoint(1, 0.0, -12.0); // at the very top (0dBFS-ish end), output caps at -12 -- a limiting-like shape
        const double aboveGainDb = settle(TriggerCondition::Above, downCurve, -18.0, -3.0); // loud, well above threshold
        const double aboveExpected = downCurve.evaluate(-3.0) - (-3.0);
        const bool aboveVoiceOk = nearlyEqual(aboveGainDb, aboveExpected, 0.5);

        TransferCurve gateCurve;
        gateCurve.setPoint(0, 0.0, -50.0); // deep attenuation at the quiet end
        gateCurve.setPoint(1, 0.0, 0.0);
        const double belowGainDb = settle(TriggerCondition::Below, gateCurve, -20.0, -40.0); // quiet, well below threshold
        const double belowExpected = gateCurve.evaluate(-40.0) - (-40.0);
        const bool belowVoiceOk = nearlyEqual(belowGainDb, belowExpected, 0.5);

        const bool ok = aboveVoiceOk && belowVoiceOk;
        printf("[16] CurveVoice settles at the curve's own gap: Above/limiting=%.2fdB (want ~%.2f), "
               "Below/gate=%.2fdB (want ~%.2f): %s\n",
               aboveGainDb, aboveExpected, belowGainDb, belowExpected, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [17] loadPoints(): accepts a well-formed point list wholesale
    //      (session/preset restore), rejects a malformed one (too few
    //      points, points too close together) and leaves the curve
    //      completely untouched on rejection -- defends against a hand-
    //      edited or foreign/future-version saved file the same way
    //      EqModel::fromJson() clamps rather than trusts its input in the
    //      sibling ../statespace project.
    {
        TransferCurve curve;
        curve.addPoint(-30.0, -10.0);

        const CurvePoint good[4] = { { TransferCurve::kMinDb, TransferCurve::kMinDb }, { -20.0, -30.0 }, { -5.0, -2.0 }, { 6.0, 6.0 } };
        const bool acceptedGood = curve.loadPoints(good, 4) && curve.pointCount() == 4 &&
                                   nearlyEqual(curve.point(1).inputDb, -20.0, 1e-9);

        const CurvePoint tooClose[3] = { { TransferCurve::kMinDb, TransferCurve::kMinDb }, { TransferCurve::kMinDb + 0.5, 0.0 }, { 6.0, 6.0 } }; // 0.5dB apart, under kMinSeparationDb
        const bool rejectedTooClose = !curve.loadPoints(tooClose, 3) && curve.pointCount() == 4; // unchanged from the accepted-good load above

        TransferCurve untouched;
        untouched.addPoint(-30.0, -10.0);
        const CurvePoint singlePoint[1] = { { 0.0, 0.0 } };
        const bool rejectedTooFew = !untouched.loadPoints(singlePoint, 1) && untouched.pointCount() == 3;

        const bool ok = acceptedGood && rejectedTooClose && rejectedTooFew;
        printf("[17] TransferCurve::loadPoints(): accepts a well-formed list, rejects a malformed "
               "one (too few / too-close points) leaving the curve untouched: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [18] curveToString()/curveFromString(): a real curve (interior
    //      points, one of them given a tangent handle) round-trips
    //      exactly, and malformed input (empty string, garbage, a valid-
    //      looking but too-close point pair) is rejected leaving the
    //      destination curve untouched.
    {
        TransferCurve curve;
        curve.addPoint(-40.0, -35.0);
        const int handled = curve.addPoint(-15.0, -3.0);
        curve.setPointHandle(handled, 2.0, -6.0);
        curve.setPoint(0, 0.0, -55.0);
        curve.setPoint(curve.pointCount() - 1, 0.0, 4.0);

        const std::string encoded = curveToString(curve);
        TransferCurve decoded;
        const bool decodedOk = curveFromString(encoded, decoded) &&
                                decoded.pointCount() == curve.pointCount() &&
                                nearlyEqual(decoded.point(handled).handleInputDb, 2.0, 1e-4) &&
                                nearlyEqual(decoded.point(handled).handleOutputDb, -6.0, 1e-4);
        bool pointsMatch = decodedOk;
        if (decodedOk)
            for (int i = 0; i < curve.pointCount(); ++i)
            {
                const CurvePoint a = curve.point(i), b = decoded.point(i);
                if (!nearlyEqual(a.inputDb, b.inputDb, 1e-4) || !nearlyEqual(a.outputDb, b.outputDb, 1e-4))
                    pointsMatch = false;
            }

        TransferCurve untouched;
        untouched.addPoint(-25.0, -25.0);
        const bool rejectsEmpty = !curveFromString("", untouched);
        const bool rejectsGarbage = !curveFromString("L;not,a,number;also-garbage", untouched);
        const bool rejectsTooClose = !curveFromString("L;-60,-60;-59.5,0;6,6", untouched); // 0.5dB apart
        const bool untouchedOk = untouched.pointCount() == 3; // unaffected by any of the three rejected parses above

        const bool ok = decodedOk && pointsMatch && rejectsEmpty && rejectsGarbage && rejectsTooClose && untouchedOk;
        printf("[18] curveToString()/curveFromString(): a real curve round-trips exactly, "
               "malformed input rejected without disturbing the destination: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [19] TransferCurve step/gate pair: setPoint() snaps an interior
    //      point flush to a neighbor once within kMinSeparationDb of it
    //      (rather than clamping just short), evaluate() then resolves the
    //      shared X with a ">=" convention (the pair's later/upper-index
    //      point governs at and above the shared X, the earlier one
    //      governs strictly below), a third point can never stack onto an
    //      already-formed pair, and the step survives a full
    //      curveToString()/curveFromString() round trip intact.
    {
        TransferCurve curve;
        const int idxA = curve.addPoint(-20.0, -50.0); // will become the step's lower/left half
        // Drag it to within kMinSeparationDb of the right endpoint (X=6) --
        // must snap flush to X=6 exactly, not stop 1dB short.
        curve.setPoint(idxA, 5.7, -50.0);
        const bool snapped = nearlyEqual(curve.point(idxA).inputDb, TransferCurve::kMaxDb, 1e-9);

        // curve is now: [ (kMinDb,kMinDb), (kMaxDb,-50), (kMaxDb,kMaxDb) ] --
        // a hard step right at the top of the range.
        const bool stepCountOk = curve.pointCount() == 3;
        const bool belowStepOk = nearlyEqual(curve.evaluate(TransferCurve::kMaxDb - 0.01), -50.0, 0.5); // approaches the left half from below
        const bool atStepOk = nearlyEqual(curve.evaluate(TransferCurve::kMaxDb), TransferCurve::kMaxDb, 1e-9); // ">=" -- the right half wins exactly at X

        // A third point can't stack onto the same X as an existing pair --
        // must refuse the snap and fall back to the ordinary clamp.
        const int idxB = curve.addPoint(4.0, 0.0);
        curve.setPoint(idxB, 5.99, 0.0); // trying to also snap to kMaxDb
        const bool noTripleStack = !nearlyEqual(curve.point(idxB).inputDb, TransferCurve::kMaxDb, 1e-9);

        const std::string encoded = curveToString(curve);
        TransferCurve decoded;
        const bool roundTripOk = curveFromString(encoded, decoded) && decoded.pointCount() == curve.pointCount() &&
                                  nearlyEqual(decoded.evaluate(TransferCurve::kMaxDb), TransferCurve::kMaxDb, 1e-6) &&
                                  nearlyEqual(decoded.evaluate(TransferCurve::kMaxDb - 0.01), -50.0, 0.5);

        const bool ok = snapped && stepCountOk && belowStepOk && atStepOk && noTripleStack && roundTripOk;
        printf("[19] TransferCurve step/gate pair: setPoint() snaps flush to a neighbor, evaluate() "
               "resolves it with a >= convention, no 3-way stack, round-trips intact: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [20] Hysteresis direction tag: an untagged curve evaluates
    //      identically in evaluateDirectional(x,true)/(x,false) and plain
    //      evaluate() (the default-parameter computeCurveGainDb() call
    //      every pre-hysteresis call site still makes is therefore a true
    //      no-op); a RisingOnly-tagged interior point is only seen by the
    //      rising view (the falling view interpolates straight past it,
    //      between its own neighbors instead); the tag survives a
    //      curveToString()/curveFromString() round trip, and an older,
    //      pre-hysteresis saved string (no third per-point field) still
    //      parses, defaulting every point to Both.
    {
        TransferCurve untagged;
        const int untaggedMid = untagged.addPoint(-20.0, 3.0);
        untagged.setPointHandle(untaggedMid, 4.0, 10.0); // give it a real handle too -- must still match on both views
        bool identicalWhenUntagged = true;
        for (double x = -55.0; x < 5.0; x += 3.3)
        {
            const double a = untagged.evaluate(x);
            const double r = untagged.evaluateDirectional(x, true);
            const double f = untagged.evaluateDirectional(x, false);
            if (!nearlyEqual(a, r, 1e-9) || !nearlyEqual(a, f, 1e-9)) identicalWhenUntagged = false;
        }

        TransferCurve tagged;
        const int mid = tagged.addPoint(-20.0, 3.0); // well off the straight identity line between endpoints
        tagged.setPointDirection(mid, PointDirection::RisingOnly);
        const bool risingSeesIt = nearlyEqual(tagged.evaluateDirectional(-20.0, true), 3.0, 1e-9);
        const bool fallingSkipsIt = nearlyEqual(tagged.evaluateDirectional(-20.0, false), -20.0, 1e-9); // identity line without it

        const std::string encoded = curveToString(tagged);
        TransferCurve decoded;
        const bool roundTripOk = curveFromString(encoded, decoded) && decoded.pointDirection(mid) == PointDirection::RisingOnly;

        TransferCurve legacy;
        const bool legacyParses = curveFromString("L;-80,-80;-10,5;6,6", legacy) && legacy.pointCount() == 3 &&
                                   legacy.pointDirection(1) == PointDirection::Both;

        const bool ok = identicalWhenUntagged && risingSeesIt && fallingSkipsIt && roundTripOk && legacyParses;
        printf("[20] Hysteresis direction tag: untagged curve unaffected, a RisingOnly point is seen "
               "only by the rising view, round-trips through curveToString()/curveFromString(), "
               "pre-hysteresis saved strings still parse: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [21] CurveVoice tracks its own detector's rising/falling direction
    //      (judged on the enveloped level it already evaluates against,
    //      not the raw per-sample waveform) and feeds it through to
    //      computeCurveGainDb(): sweeping the level UP through a
    //      RisingOnly-tagged point's own X reaches that point's value,
    //      sweeping back DOWN through the same X does not (falls back to
    //      the untagged identity line instead).
    {
        TransferCurve curve;
        const int mid = curve.addPoint(-20.0, 3.0);
        curve.setPointDirection(mid, PointDirection::RisingOnly);

        CurveVoice voice;
        voice.prepare(48000.0);
        voice.setParams(0.0, 0.0, TransferCurve::kMinDb + 1.0, TriggerCondition::Above); // always triggered

        double gLin = 0.0;
        for (double lvl = -50.0; lvl <= -20.0; lvl += 0.5)
            gLin = voice.process(std::pow(10.0, lvl / 20.0), curve);
        const double risingOutDb = 20.0 * std::log10(std::fabs(std::pow(10.0, -20.0 / 20.0) * gLin));
        const bool risingReachesTag = nearlyEqual(risingOutDb, 3.0, 0.05);

        for (double lvl = -20.0; lvl >= -50.0; lvl -= 0.5)
            gLin = voice.process(std::pow(10.0, lvl / 20.0), curve);
        const double fallingOutDb = 20.0 * std::log10(std::fabs(std::pow(10.0, -50.0 / 20.0) * gLin));
        const bool fallingSkipsTag = nearlyEqual(fallingOutDb, -50.0, 0.05); // identity line, tag not in the falling view

        const bool ok = risingReachesTag && fallingSkipsTag;
        printf("[21] CurveVoice hysteresis: rising sweep reaches a RisingOnly-tagged point's own value, "
               "falling sweep through the same X does not: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [22] Support-point (handle) X limits are direction-aware: a FallingOnly
    //      node's handle is limited by neighbors it can conflict with (Both /
    //      Falling) and NOT by an adjacent RisingOnly node (and vice versa);
    //      a Both node still stops at any adjacent neighbor; changing a
    //      direction re-clamps; a genuine conflict is still limited.
    {
        TransferCurve c;
        const int r = c.addPoint(-40.0, -40.0);  // will be RisingOnly, close to F
        const int f = c.addPoint(-38.0, -30.0);  // will be FallingOnly, 2 dB right of R
        c.setPointDirection(r, PointDirection::RisingOnly);
        c.setPointDirection(f, PointDirection::FallingOnly);
        // f's nearest sharing neighbors: r is Rising (no), so left = endpoint at -60 (22 dB away),
        // right = endpoint at +6 (44 dB away) -> limit 22.
        c.setPointHandle(f, 15.0, 3.0);
        const bool fallingNotLimitedByRising = nearlyEqual(c.point(f).handleInputDb, 15.0, 1e-9);
        // ...and r likewise isn't limited by f: left endpoint 20 dB away.
        c.setPointHandle(r, 15.0, 3.0);
        const bool risingNotLimitedByFalling = nearlyEqual(c.point(r).handleInputDb, 15.0, 1e-9);
        // Same tag next to each other still conflict: make f Rising -> r and f are 2 dB apart.
        c.setPointDirection(f, PointDirection::RisingOnly);
        const bool retaggedReclamps = nearlyEqual(c.point(f).handleInputDb, 2.0, 1e-9) && nearlyEqual(c.point(r).handleInputDb, 2.0, 1e-9);
        c.setPointHandle(f, 15.0, 0.0);
        const bool sameTagStillLimited = nearlyEqual(c.point(f).handleInputDb, 2.0, 1e-9);
        // A Both node stops at any adjacent neighbor.
        TransferCurve d;
        const int a = d.addPoint(-30.0, -30.0), b = d.addPoint(-28.0, -20.0);
        d.setPointDirection(b, PointDirection::FallingOnly);
        d.setPointHandle(a, 15.0, 0.0); // Both, neighbor b (Falling) shares the falling view
        const bool bothLimited = nearlyEqual(d.point(a).handleInputDb, 2.0, 1e-9);
        const bool ok = fallingNotLimitedByRising && risingNotLimitedByFalling && retaggedReclamps && sameTagStillLimited && bothLimited;
        printf("[22] handle X limits only consider neighbors that share a rising/falling view (%d %d %d %d %d): %s\n",
               fallingNotLimitedByRising, risingNotLimitedByFalling, retaggedReclamps, sameTagStillLimited, bothLimited, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [23] Link groups: singleton groups dissolve (normalizeLinks / removePoint),
    //      groups survive add/remove index shifts, round-trip through
    //      curveToString()/curveFromString() and the Parameter-mirror packing
    //      (direction + 3*link), and an old string without the field loads unlinked.
    {
        TransferCurve c;
        const int p1 = c.addPoint(-40.0, -40.0);
        const int p2 = c.addPoint(-20.0, -10.0);
        const int p3 = c.addPoint(-5.0, 0.0);
        c.setPointLink(p1, 1); c.setPointLink(p2, 1); c.setPointLink(p3, 2);
        c.normalizeLinks(); // p3 alone in group 2 -> dissolved
        const bool singletonDissolved = c.pointLink(p1) == 1 && c.pointLink(p2) == 1 && c.pointLink(p3) == 0;
        c.addPoint(-50.0, -50.0); // inserts before everything: indices shift, links must follow their nodes
        const bool shiftFollows = c.pointLink(2) == 1 && c.pointLink(3) == 1 && c.pointLink(1) == 0;
        c.removePoint(3); // removes one member of group 1 -> the other dissolves
        const bool removeDissolves = c.pointLink(2) == 0;
        c.setPointLink(1, 3); c.setPointLink(2, 3);
        c.setPointDirection(1, PointDirection::FallingOnly);
        const std::string str = curveToString(c);
        TransferCurve back;
        const bool parsed = curveFromString(str, back);
        const bool roundTrip = parsed && back.pointCount() == c.pointCount() && back.pointLink(1) == 3 && back.pointLink(2) == 3 &&
                               back.pointDirection(1) == PointDirection::FallingOnly;
        const float packed = packDirectionAndLink(PointDirection::FallingOnly, 7);
        const bool packing = unpackDirection(packed) == PointDirection::FallingOnly && unpackLink(packed) == 7 &&
                             unpackDirection(2.0f) == PointDirection::FallingOnly && unpackLink(2.0f) == 0; // old session value
        TransferCurve old;
        const bool oldLoads = curveFromString("L;-80,-80,0,0,0;-20,-10,1,0,0;6,6,0,0,0", old) && old.pointLink(1) == 0 &&
                              old.pointDirection(1) == PointDirection::RisingOnly;
        const bool ok = singletonDissolved && shiftFollows && removeDissolves && roundTrip && packing && oldLoads;
        printf("[23] link groups: singleton dissolves, follows index shifts, dissolves on removal, round-trips "
               "(string + Parameter packing), old data loads unlinked (%d %d %d %d %d %d): %s\n",
               singletonDissolved, shiftFollows, removeDissolves, roundTrip, packing, oldLoads, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [24] Below the curve's own left end the gain HOLDS its end value (unity
    //      slope) instead of the old flat extrapolation, which boosted every
    //      near-zero sample of an always-triggered instant (0ms/0ms) voice up
    //      to the left end's output level -- a crunch even on an untouched
    //      identity curve. An identity curve must now be transparent, sample
    //      for sample, at any level including far below -60dB.
    {
        TransferCurve identity;
        const bool zeroGainFarBelow = nearlyEqual(computeCurveGainDb(-95.0, 6.0, TriggerCondition::Below, identity), 0.0, 1e-9) &&
                                      nearlyEqual(computeCurveGainDb(-81.0, 6.0, TriggerCondition::Below, identity), 0.0, 1e-9);
        // A curve whose left end sits 20dB above its input: below the end the gain holds +20dB (no runaway).
        TransferCurve lifted;
        lifted.setPoint(0, TransferCurve::kMinDb, TransferCurve::kMinDb + 20.0);
        const bool holdsEndGain = nearlyEqual(computeCurveGainDb(-80.0, 6.0, TriggerCondition::Below, lifted), 20.0, 1e-9) &&
                                  nearlyEqual(computeCurveGainDb(-81.0, 6.0, TriggerCondition::Below, lifted), 20.0, 1e-9) &&
                                  nearlyEqual(computeCurveGainDb(-110.0, 6.0, TriggerCondition::Below, lifted), 20.0, 1e-9);
        // Whole-voice check at 0/0: transparent identity within float noise on a real signal with many sub -60dB samples.
        CurveVoice v; v.prepare(48000.0); v.setParams(0.0, 0.0, 6.0, TriggerCondition::Below);
        double err = 0.0, sig = 0.0; int quiet = 0;
        for (int i = 0; i < 48000; ++i)
        {
            const double x = 0.05 * std::sin(2.0 * M_PI * 220.0 * i / 48000.0) + 0.02 * std::sin(2.0 * M_PI * 3130.0 * i / 48000.0);
            const double y = x * v.process(std::fabs(x), identity);
            err += (y - x) * (y - x); sig += x * x;
            if (std::fabs(x) < 0.001) ++quiet;
        }
        const bool transparent = quiet > 100 && err / sig < 1e-18;
        // Above the right end stays flat (a ceiling): unchanged behaviour.
        const bool aboveFlat = nearlyEqual(identity.evaluate(TransferCurve::kMaxDb + 20.0), TransferCurve::kMaxDb, 1e-9);
        const bool ok = zeroGainFarBelow && holdsEndGain && transparent && aboveFlat;
        printf("[24] below the curve's left end the gain holds (identity transparent even far below -60dB at 0/0; %d quiet samples): "
               "(%d %d %d %d): %s\n", quiet, zeroGainFarBelow, holdsEndGain, transparent, aboveFlat, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [25] End nodes' support points are part of the calculation: giving the
    //      left/right end node a handle changes the curve near that end, is
    //      limited only by its nearest neighbour, and survives serialization.
    {
        TransferCurve flat;
        flat.addPoint(-30.0, -30.0);
        const double before = flat.evaluate(-50.0);
        flat.setPointHandle(0, 8.0, 12.0);   // left end: leave the end with a steep slope
        const double afterLeft = flat.evaluate(-50.0);
        const int last = flat.pointCount() - 1;
        const double beforeR = flat.evaluate(0.0);
        flat.setPointHandle(last, 8.0, 12.0);
        const double afterRight = flat.evaluate(0.0);
        const bool changes = std::fabs(afterLeft - before) > 0.5 && std::fabs(afterRight - beforeR) > 0.5;
        flat.setPointHandle(0, 500.0, 0.0); // absurd reach clamps to the gap to the next node (50 dB: -80 .. -30)
        const bool clamped = nearlyEqual(flat.point(0).handleInputDb, 50.0, 1e-9);
        TransferCurve back;
        const bool round = curveFromString(curveToString(flat), back) && nearlyEqual(back.point(last).handleOutputDb, 12.0, 1e-4);
        const bool ok = changes && clamped && round;
        printf("[25] end-node support points change the curve near that end, clamp to the neighbour gap, round-trip (%d %d %d): %s\n",
               changes, clamped, round, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [26] The threshold can be linked (X only, no CurvePoint of its own): it
    //      counts as a group member, a group left with only the threshold
    //      dissolves, and its group id round-trips through the curve string
    //      (marker byte) and the trigger-condition Parameter packing.
    {
        TransferCurve c;
        const int a = c.addPoint(-30.0, -30.0);
        c.setPointLink(a, 2); c.setThresholdLink(2);
        c.normalizeLinks();
        const bool pair = c.pointLink(a) == 2 && c.thresholdLink() == 2;
        TransferCurve back;
        const bool round = curveFromString(curveToString(c), back) && back.thresholdLink() == 2 && back.pointLink(1) == 2;
        c.removePoint(a); // the node leaves: the threshold alone can't be a group
        const bool dissolved = c.thresholdLink() == 0;
        const float packed = packConditionAndThresholdLink(1, 5);
        const bool packing = unpackConditionInt(packed) == 1 && unpackThresholdLink(packed) == 5 &&
                             unpackConditionInt(1.0f) == 1 && unpackThresholdLink(1.0f) == 0; // old session value
        TransferCurve old;
        const bool oldLoads = curveFromString("L;-60,-60,0,0,0;6,6,0,0,0", old) && old.thresholdLink() == 0;
        c.setThresholdLink(4); c.resetToDefault();
        const bool resetClears = c.thresholdLink() == 0;
        const bool ok = pair && round && dissolved && packing && oldLoads && resetClears;
        printf("[26] threshold link: counts as a member, dissolves when alone, string marker + Parameter packing round-trip, "
               "old data unlinked (%d %d %d %d %d %d): %s\n", pair, round, dissolved, packing, oldLoads, resetClears, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [27] The trail's (level, output = level+gain) pair packs into one float EXACTLY (24-bit integer) and unpacks
    //      to within the quantization step across the whole range -- through an actual float round trip. The output
    //      position is what is kept exact: a low right node at a high detector level needs a gain far beyond the old
    //      +-72dB pack range, which used to clamp and make the dot's y climb up the right edge.
    {
        bool ok = true; double worstL = 0.0, worstOut = 0.0;
        for (double lv = -125.0; lv <= 135.0; lv += 0.37)
            for (double g = -200.0; g <= 80.0; g += 1.13)
            {
                const float packed = packTrailPair(lv, g);
                const float viaFloat = static_cast<float>(static_cast<double>(packed)); // any float transport
                double l2, g2; unpackTrailPair(viaFloat, l2, g2);
                const double lc = std::max(kTrailLevelMinDb, std::min(kTrailLevelMaxDb, lv));
                const double oc = std::max(kTrailOutMinDb, std::min(kTrailOutMaxDb, lv + g));
                worstL = std::max(worstL, std::fabs(l2 - lc)); worstOut = std::max(worstOut, std::fabs((l2 + g2) - oc));
                if (packed != viaFloat || packed < 0.0f || packed > 16777215.0f) ok = false;
            }
        ok = ok && worstL < 0.035 && worstOut < 0.06;
        // The reported case, explicitly: right node at -50dB, detector at +34dB / +54dB -> the dot must sit at y = -50.
        double l1, g1, l2, g2;
        unpackTrailPair(packTrailPair(34.0, -84.0), l1, g1);
        unpackTrailPair(packTrailPair(54.0, -104.0), l2, g2);
        const bool staysPut = std::fabs((l1 + g1) - (-50.0)) < 0.06 && std::fabs((l2 + g2) - (-50.0)) < 0.06;
        ok = ok && staysPut;
        printf("[27] trail (level, output) pair packs into one exactly-representable float, worst error %.4f dB / %.4f dB; a dot at "
               "y=-50 stays at -50 for detector levels of +34 and +54dB (%d): %s\n", worstL, worstOut, int(staysPut), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [28] A fractional value in a direction/link slot (what sessions saved while the mirror banks
    //      overlapped can hold: a support-point dB value) decodes as "Both, unlinked", never as noise.
    {
        const bool ok = unpackDirection(5.8f) == PointDirection::Both && unpackLink(5.8f) == 0 &&
                        unpackDirection(-3.25f) == PointDirection::Both && unpackLink(-3.25f) == 0 &&
                        unpackDirection(14.0f) == PointDirection::FallingOnly && unpackLink(14.0f) == 4;
        printf("[28] direction/link decode ignores fractional (legacy/garbage) values, exact integers still decode: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [29] moveLinked(): editing one node of a link group numerically moves the whole group (and the
    //      threshold, X only) by the same delta; an unlinked node moves alone; the right-most member
    //      goes first when moving right, so close members (a and b are 2 dB apart) are not clamped
    //      against a neighbour that has not moved yet.
    {
        TransferCurve c;
        const int a = c.addPoint(-40.0, -40.0), b = c.addPoint(-38.0, -30.0), d = c.addPoint(-10.0, -12.0), e = c.addPoint(4.0, 4.0);
        c.setPointLink(a, 1); c.setPointLink(b, 1); c.setPointLink(d, 1); c.setThresholdLink(1);
        c.normalizeLinks();
        double thr = -30.0;
        c.moveLinked(a, 5.0, 3.0, thr);
        const bool ins = nearlyEqual(c.point(a).inputDb, -35.0, 1e-9) && nearlyEqual(c.point(b).inputDb, -33.0, 1e-9) &&
                         nearlyEqual(c.point(d).inputDb, -5.0, 1e-9);
        const bool outs = nearlyEqual(c.point(a).outputDb, -37.0, 1e-9) && nearlyEqual(c.point(b).outputDb, -27.0, 1e-9);
        const bool thrMoved = nearlyEqual(thr, -25.0, 1e-9);
        const bool loneStays = nearlyEqual(c.point(e).inputDb, 4.0, 1e-9);
        double thr2 = -30.0;
        c.moveLinked(e, -1.0, 0.0, thr2); // unlinked node: only itself, threshold and the group untouched
        const bool alone = nearlyEqual(c.point(e).inputDb, 3.0, 1e-9) && nearlyEqual(thr2, -30.0, 1e-9) && nearlyEqual(c.point(a).inputDb, -35.0, 1e-9);
        const bool ok = ins && outs && thrMoved && loneStays && alone;
        printf("[29] moveLinked moves the whole group (+threshold on X), unlinked nodes alone, direction-ordered (%d %d %d %d %d): %s\n",
               ins, outs, thrMoved, loneStays, alone, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [30] The Bezier solve is exact even for extreme handles: evaluate() against an independent
    //      bisection solve of the same segment, on random curves with huge/overlapping handles.
    //      (Plain Newton capped at 8 iterations used to miss by up to ~1.5dB there.)
    {
        auto bez = [](double a, double b, double c, double d, double t) { const double u = 1.0 - t; return u * u * u * a + 3 * u * u * t * b + 3 * u * t * t * c + t * t * t * d; };
        auto ref = [&](const TransferCurve& c, double x) {
            const int n = c.pointCount();
            if (x <= c.point(0).inputDb) return c.point(0).outputDb;
            if (x >= c.point(n - 1).inputDb) return c.point(n - 1).outputDb;
            int i = 0; while (i < n - 2 && x >= c.point(i + 1).inputDb) ++i;
            const CurvePoint a = c.point(i), b = c.point(i + 1);
            if (a.handleInputDb == 0 && a.handleOutputDb == 0 && b.handleInputDb == 0 && b.handleOutputDb == 0)
                return a.outputDb + (x - a.inputDb) / std::max(1e-9, b.inputDb - a.inputDb) * (b.outputDb - a.outputDb);
            const double p1x = std::max(a.inputDb, std::min(b.inputDb, a.inputDb + a.handleInputDb)), p1y = a.outputDb + a.handleOutputDb;
            const double p2x = std::max(a.inputDb, std::min(b.inputDb, b.inputDb - b.handleInputDb)), p2y = b.outputDb - b.handleOutputDb;
            double lo = 0.0, hi = 1.0;
            for (int k = 0; k < 80; ++k) { const double m = 0.5 * (lo + hi); if (bez(a.inputDb, p1x, p2x, b.inputDb, m) < x) lo = m; else hi = m; }
            return bez(a.outputDb, p1y, p2y, b.outputDb, 0.5 * (lo + hi));
        };
        unsigned seed = 12345; auto rnd = [&]() { seed = seed * 1664525u + 1013904223u; return double(seed >> 8) / double(1 << 24); };
        double worst = 0.0;
        for (int trial = 0; trial < 2000; ++trial)
        {
            TransferCurve r; const int np = 2 + int(rnd() * 5);
            for (int i = 0; i < np; ++i) r.addPoint(-58.0 + rnd() * 62.0, -60.0 + rnd() * 66.0);
            for (int i = 0; i < r.pointCount(); ++i) r.setPointHandle(i, rnd() * 40.0, (rnd() - 0.5) * 60.0);
            for (int k = 0; k < 300; ++k) { const double x = -60.0 + 66.0 * k / 299.0; worst = std::max(worst, std::fabs(r.evaluate(x) - ref(r, x))); }
        }
        const bool ok = worst < 1e-4;
        printf("[30] evaluate() matches an exact Bezier solve on random extreme-handle curves, worst error %.6f dB: %s\n", worst, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [31] The model the GUI edits equals what the DSP loads: after moving a node close to a neighbour, the
    //      curve (GUI copy) and a copy rebuilt via loadPoints() (what the DSP does with the mirrored points)
    //      evaluate identically, and the stored handles are already within their limits. (Used to differ
    //      by up to 1.7dB -- the trail dots then sat visibly off the drawn line after such an edit.)
    {
        TransferCurve gui;
        curveFromString("L;-60,-60,0,0,0,0;-42.379,-33.457,0,7.845,5.229,0;-14.858,-23.395,0,8.717,2.060,0;6,6,0,19.55,1.90,0", gui);
        double worstDiff = 0.0; bool within = true;
        const double moves[][2] = { { -37.0, -25.0 }, { -44.0, -30.0 }, { -20.0, -40.0 }, { -3.0, -10.0 } };
        for (const auto& mv : moves)
        {
            gui.setPoint(2, mv[0], mv[1]);
            CurvePoint pts[TransferCurve::kMaxPoints];
            for (int i = 0; i < gui.pointCount(); ++i) pts[i] = gui.point(i);
            TransferCurve dsp; dsp.loadPoints(pts, gui.pointCount(), gui.thresholdLink());
            for (int k = 0; k <= 6600; ++k) { const double x = -60.0 + k * 0.01; worstDiff = std::max(worstDiff, std::fabs(gui.evaluate(x) - dsp.evaluate(x))); }
            for (int i = 0; i < gui.pointCount(); ++i) if (gui.point(i).handleInputDb > gui.maxHandleDx(i) + 1e-12) within = false;
        }
        const bool ok = worstDiff < 1e-9 && within;
        printf("[31] GUI curve == DSP-loaded curve after edits that squeeze neighbours (worst diff %.9f dB, handles within limits: %d): %s\n",
               worstDiff, int(within), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [32] A support point handle d and -d are the same tangent line: the curve must be identical, so dx may be
    //      NEGATIVE (the bright dot on the other side of its node). Also survives serialization, and |dx| is
    //      still limited by the neighbour gap with its sign kept.
    {
        auto build = [](double dx, double dy) {
            TransferCurve c; c.addPoint(-40.0, -35.0); c.addPoint(-15.0, -20.0);
            c.setPointHandle(1, dx, dy); c.setPointHandle(c.pointCount() - 1, dx * 0.5, dy * 0.5);
            return c;
        };
        TransferCurve pos = build(8.0, 5.0), neg = build(-8.0, -5.0);
        double worst = 0.0;
        for (int k = 0; k <= 6600; ++k) { const double x = -60.0 + k * 0.01; worst = std::max(worst, std::fabs(pos.evaluate(x) - neg.evaluate(x))); }
        const bool same = worst < 1e-12;
        TransferCurve limited; limited.addPoint(-40.0, -35.0);
        limited.setPointHandle(1, -500.0, 3.0);
        const bool signKept = limited.point(1).handleInputDb < 0.0 && nearlyEqual(std::fabs(limited.point(1).handleInputDb), 40.0, 1e-9); // gap to the left end (-80)
        TransferCurve back;
        const bool round = curveFromString(curveToString(neg), back) && nearlyEqual(back.point(1).handleInputDb, -8.0, 1e-4) && nearlyEqual(back.point(1).handleOutputDb, -5.0, 1e-4);
        const bool ok = same && signKept && round;
        printf("[32] handle d and -d give the identical curve (negative dx allowed), |dx| limited with sign kept, round-trips (worst diff %.2e; %d %d %d): %s\n",
               worst, same, signKept, round, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [33] Link axis: X-only / Y-only groups propagate only that axis (the edited node itself moves freely), the
    //      threshold follows X-carrying groups only and can't be in a Y-only group, and the axis round-trips through
    //      the curve string and the Parameter packing (old axis-0 values unchanged).
    {
        TransferCurve c;
        const int a = c.addPoint(-40.0, -40.0), b = c.addPoint(-30.0, -30.0), d = c.addPoint(-20.0, -20.0), e = c.addPoint(-10.0, -10.0);
        c.setPointLink(a, 1, kLinkAxisX); c.setPointLink(b, 1, kLinkAxisX);
        c.setPointLink(d, 2, kLinkAxisY); c.setPointLink(e, 2, kLinkAxisY);
        c.normalizeLinks();
        double thr = -50.0;
        c.moveLinked(a, 3.0, 4.0, thr);   // X group: a moves freely (3,4); b only gets dx=3
        const bool xGroup = nearlyEqual(c.point(a).inputDb, -37.0, 1e-9) && nearlyEqual(c.point(a).outputDb, -36.0, 1e-9) &&
                            nearlyEqual(c.point(b).inputDb, -27.0, 1e-9) && nearlyEqual(c.point(b).outputDb, -30.0, 1e-9);
        c.moveLinked(d, 2.0, -5.0, thr);  // Y group: d moves freely; e only gets dy=-5
        const bool yGroup = nearlyEqual(c.point(d).inputDb, -18.0, 1e-9) && nearlyEqual(c.point(d).outputDb, -25.0, 1e-9) &&
                            nearlyEqual(c.point(e).inputDb, -10.0, 1e-9) && nearlyEqual(c.point(e).outputDb, -15.0, 1e-9);
        TransferCurve t;
        const int p1 = t.addPoint(-30.0, -30.0), p2 = t.addPoint(-10.0, -10.0);
        t.setPointLink(p1, 3, kLinkAxisX); t.setPointLink(p2, 3, kLinkAxisX); t.setThresholdLink(3); t.normalizeLinks();
        double thr2 = -20.0; t.moveLinked(p1, 4.0, 9.0, thr2);
        const bool thrFollowsX = nearlyEqual(thr2, -16.0, 1e-9) && t.thresholdLink() == 3;
        t.setPointLink(p1, 3, kLinkAxisY); t.setPointLink(p2, 3, kLinkAxisY); t.normalizeLinks();
        const bool noThrInY = t.thresholdLink() == 0;
        TransferCurve r2;
        const bool round = curveFromString(curveToString(c), r2) && r2.pointLinkAxis(a) == kLinkAxisX && r2.pointLinkAxis(d) == kLinkAxisY && r2.pointLink(d) == 2;
        const float packed = packDirectionAndLink(PointDirection::FallingOnly, 7, kLinkAxisY);
        const bool packing = unpackDirection(packed) == PointDirection::FallingOnly && unpackLink(packed) == 7 && unpackLinkAxis(packed) == kLinkAxisY &&
                             unpackLink(14.0f) == 4 && unpackLinkAxis(14.0f) == 0; // an old axis-less value
        const bool ok = xGroup && yGroup && thrFollowsX && noThrInY && round && packing;
        printf("[33] link axes: X-only/Y-only propagate one axis, threshold follows X groups only, round-trips (string + packing) (%d %d %d %d %d %d): %s\n",
               xGroup, yGroup, thrFollowsX, noThrInY, round, packing, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [34] softLimitListen(): Listen output is bounded whatever the (now up to +60dB amplified) trigger signal is.
    //      Passes small signals untouched, is continuous and monotonic through the knee, odd-symmetric, and never
    //      exceeds 1.0 -- and the trigger gain range really reaches +60dB.
    {
        bool passes = true, bounded = true, monotonic = true, odd = true, continuous = true;
        double prev = softLimitListen(-1.0e6), worstJump = 0.0;
        for (double x = -1.0e6; x <= 1.0e6; x += (std::fabs(x) < 4.0 ? 0.001 : 997.0))
        {
            const double y = softLimitListen(x);
            if (std::fabs(x) <= 0.7 && y != x) passes = false;
            if (std::fabs(y) > 1.0) bounded = false;
            if (y < prev - 1e-12) monotonic = false;
            if (std::fabs(x) < 4.0) worstJump = std::max(worstJump, std::fabs(y - prev));
            if (std::fabs(softLimitListen(-x) + y) > 1e-12) odd = false;
            prev = y;
        }
        continuous = worstJump < 0.0011;
        const bool range = kSidechainGainMaxDb >= 60.0 && kSidechainGainMinDb <= -24.0 && nearlyEqual(dbToLinear(kSidechainGainMaxDb), 1000.0, 1e-6);
        const bool ok = passes && bounded && monotonic && odd && continuous && range;
        printf("[34] Listen soft limiter: transparent up to 0.7, bounded by 1.0 for any input, monotonic, odd, continuous; gain range reaches +60dB (%d %d %d %d %d %d): %s\n",
               passes, bounded, monotonic, odd, continuous, range, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [35] Above the curve's right end (detector level over +6dB) the OUTPUT is held at the right node's value: no
    //      "<" vs "<=" seam at exactly +6dB (the curve is continuous there, also with a handle on the end node), and
    //      out = level + gain stays constant for any level above it -- however far -- and below the left end the gain
    //      holds. (Dots climbing up the right edge were the trail's display packing, not the curve -- see test [27].)
    {
        TransferCurve c;
        c.addPoint(-30.0, -40.0);
        c.setPointHandle(c.pointCount() - 1, -9.0, -4.0);   // a handle on the right end node (negative dx: white dot to its left)
        c.setPoint(c.pointCount() - 1, TransferCurve::kMaxDb, -50.0);
        const double yEnd = c.point(c.pointCount() - 1).outputDb;
        double seam = 0.0, worstAbove = 0.0;
        // (offsets small enough that the curve's own slope cannot account for a difference: a jump would show as ~dB, not ~1e-6)
        for (double e : { 1e-6, 1e-9, 0.0 }) seam = std::max(seam, std::fabs(c.evaluate(TransferCurve::kMaxDb - e) - yEnd));
        for (double lvl : { 6.0, 6.0000001, 6.5, 12.0, 34.0, 60.0, 120.0, 400.0 })
        {
            const double gain = computeCurveGainDb(lvl, -60.0, TriggerCondition::Above, c);
            worstAbove = std::max(worstAbove, std::fabs((lvl + gain) - yEnd));
        }
        const bool ok = seam < 1e-4 && worstAbove < 1e-9;
        printf("[35] curve continuous at the right end (worst seam %.2e dB); output held at the end value for detector levels 6..400dB (worst %.2e dB): %s\n",
               seam, worstAbove, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [36] The trail pair's sign bit carries a "triggered" flag without changing the level/output magnitude, and the
    //      old two-argument unpack still works.
    {
        bool ok = true;
        for (double lv = -100.0; lv <= 100.0; lv += 3.7)
            for (double g = -60.0; g <= 30.0; g += 2.9)
                for (int t = 0; t < 2; ++t)
                {
                    const float p = packTrailPair(lv, g, t != 0);
                    double l1, g1, l2, g2; bool tr = false;
                    unpackTrailPair(p, l1, g1, tr);
                    unpackTrailPair(p, l2, g2);
                    if (tr != (t != 0) || l1 != l2 || g1 != g2) ok = false;
                    const float q = packTrailPair(lv, g, false);
                    if (std::fabs(double(p)) != std::fabs(double(q))) ok = false; // same magnitude either way
                }
        // the all-zero magnitude with the flag set must still read back as triggered (-0.0f)
        double l0, g0; bool t0 = false;
        unpackTrailPair(packTrailPair(-120.0, 120.0 - 100.0 + 0.0 - 20.0, true), l0, g0, t0); // level -120, out clamps: magnitude may be 0
        ok = ok && t0;
        printf("[36] trail pair: triggered flag in the sign bit, magnitude identical, legacy unpack intact: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [37] Trigger action: Diagonal inverts the region the curve rules in; pack/unpack of condition+link+action is
    //      lossless and old values decode as ApplyCurve; in Diagonal mode Attack is the reaction to the trigger.
    {
        TransferCurve c; // flat -60 dB gate curve
        const CurvePoint pts[2] = { {-60.0, -60.0}, {6.0, -60.0} };
        c.loadPoints(pts, 2);
        bool ok = true;
        for (double lv = -50.0; lv <= 0.0; lv += 5.0)
        {
            const double g1 = computeCurveGainDb(lv, -30.0, TriggerCondition::Above, c, true, TriggerAction::ApplyCurve);
            const double g2 = computeCurveGainDb(lv, -30.0, TriggerCondition::Above, c, true, TriggerAction::ReleaseToDiagonal);
            const bool met = lv > -30.0;
            if (met) ok = ok && g1 < -20.0 && g2 == 0.0;
            else     ok = ok && g1 == 0.0 && g2 < -9.0; // level-dependent: y0 - x = -60 - lv (-10 dB at -50, -30 dB at the threshold)
        }
        for (int cond = 0; cond < 2; ++cond)
            for (int link = 0; link <= kMaxLinkGroups; ++link)
                for (int act = 0; act < 2; ++act)
                {
                    const float v = packConditionAndThresholdLink(cond, link, act);
                    ok = ok && unpackConditionInt(v) == cond && unpackThresholdLink(v) == link && unpackTriggerActionInt(v) == act;
                }
        ok = ok && unpackTriggerActionInt(packConditionAndThresholdLink(1, 4)) == 0 && unpackThresholdLink(7.0f) == 3 && unpackConditionInt(7.0f) == 1;

        // Timing: gate curve (-60 dB when the trigger is met -> for Diagonal: -60 dB when NOT met).
        // Attack 5 ms / release 200 ms, sample-accurate steps of the detector level.
        auto settle = [&](TriggerAction a, double startLevelDb, double endLevelDb, double ms) {
            CurveVoice v; v.prepare(48000.0); v.setParams(5.0, 200.0, -30.0, TriggerCondition::Above, a);
            for (int i = 0; i < 48000; ++i) v.process(std::pow(10.0, startLevelDb / 20.0), c);
            double g = 0.0;
            for (int i = 0; i < int(ms * 48.0); ++i) g = linearToDb(v.process(std::pow(10.0, endLevelDb / 20.0), c));
            return g;
        };
        // Diagonal mode, quiet -> loud (trigger fires, opening): fast attack => after 30 ms nearly open (gain near 0).
        const double opened = settle(TriggerAction::ReleaseToDiagonal, -50.0, -10.0, 30.0);
        // ...and loud -> quiet (closing): slow release => after 30 ms still nearly open.
        const double closing = settle(TriggerAction::ReleaseToDiagonal, -10.0, -50.0, 30.0);
        // Curve mode, quiet -> loud (the curve engages = attack): after 30 ms fully shut.
        const double engaged = settle(TriggerAction::ApplyCurve, -50.0, -10.0, 30.0);
        ok = ok && opened > -3.0 && closing > -20.0 && engaged < -40.0;
        printf("[37] trigger action: region inversion, pack/unpack, attack=opening/release=closing in Diagonal mode (%.1f / %.1f / %.1f): %s\n", opened, closing, engaged, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [38] Lookahead: the owner delays the AUDIO by L and feeds the voice the un-delayed trigger. The gain seen by an audio
    //      sample is the voice's gain at the moment that sample is fed: the switch is already at the curve when the burst's
    //      first audio sample arrives (the trigger began L samples earlier), and it is released only after the burst's last
    //      audio sample -- never early.
    {
        constexpr double sr = 48000.0;
        TransferCurve c;
        const CurvePoint pts[2] = { {-60.0, -60.0}, {6.0, -60.0} }; // flat -60 dB when the curve rules
        c.loadPoints(pts, 2);
        const int L = 240; // 5 ms
        const int t0 = 4000, t1 = 8000, n = 12000;
        auto run = [&](int lookahead) {
            CurveVoice v; v.prepare(sr); v.setParams(1.0, 0.0, -30.0, TriggerCondition::Above);
            v.setLookaheadSamples(lookahead);
            std::vector<double> g;
            for (int i = 0; i < n; ++i)
            {
                const double trig = (i >= t0 && i < t1) ? 0.3 : 1e-5;
                const int ai = i - lookahead; // the audio, delayed
                const double aud = (ai >= t0 && ai < t1) ? 0.3 : 1e-5;
                v.process(trig, aud, c);
                g.push_back(v.gainDb());
            }
            return g;
        };
        const auto g0 = run(0), gL = run(L);
        const double atOnsetNoLa = g0[t0 + 5];          // 0.1 ms into the burst, no lookahead: the switch has barely moved
        const double atOnsetLa = gL[t0 + L + 5];        // the same audio sample with lookahead: the switch has long been at the curve
        const double atEndLa = gL[t1 + L - 1];          // the last audio sample of the burst: still at the curve
        const double afterEndLa = gL[t1 + L + 240];     // well after it: released (Release 0 = instant)
        const bool ok = atOnsetNoLa > -10.0 && atOnsetLa < -25.0 && atEndLa < -40.0 && afterEndLa > -3.0;
        printf("[38] lookahead: onset gain no-lookahead %.1f dB -> with %.1f dB; last audio sample still %.1f dB; released after: %.1f dB: %s\n",
               atOnsetNoLa, atOnsetLa, atEndLa, afterEndLa, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [39] SidechainBandFilter: each corner knob is where the cascade is -3 dB, and the slope beyond it is ~24 dB/oct.
    //      (The high-pass stage used to be designed at corner/warp instead of corner*warp: HP 53 Hz behaved like ~280 Hz.)
    {
        auto respDb = [](double hp, double lp, double f) {
            SidechainBandFilter b; b.prepare(48000.0); b.setFrequencies(hp, lp);
            double pk = 0.0; const int n = 96000;
            for (int i = 0; i < n; ++i) { const double y = b.process(std::sin(2.0 * M_PI * f * double(i) / 48000.0)); if (i > n / 2) pk = std::max(pk, std::fabs(y)); }
            return 20.0 * std::log10(pk + 1e-12);
        };
        bool ok = true;
        for (double hz : { 60.0, 100.0, 500.0 })
        {
            const double hpAt = respDb(hz, 20000.0, hz);                 // high-pass only: -3 dB at its own corner
            const double lpAt = respDb(20.0, hz, hz);                    // low-pass only: -3 dB at its own corner
            // slope well away from the corner approaches 24 dB/oct (4 one-pole stages)
            const double hpFar = respDb(hz, 20000.0, hz * 0.0625) - respDb(hz, 20000.0, hz * 0.03125);
            const double lpFar = respDb(20.0, hz, hz * 8.0) - respDb(20.0, hz, hz * 16.0);
            if (std::fabs(hpAt + 3.0) > 0.6 || std::fabs(lpAt + 3.0) > 0.6 || std::fabs(hpFar - 24.0) > 3.0 || std::fabs(lpFar - 24.0) > 3.0) ok = false;
        }
        // a narrow low band (HP 53 / LP 79): its peak must sit between the two corners, not far above the low-pass
        double bestF = 0.0, bestDb = -1e9;
        for (double f = 20.0; f <= 400.0; f *= 1.03) { const double r = respDb(53.0, 79.0, f); if (r > bestDb) { bestDb = r; bestF = f; } }
        ok = ok && bestF > 50.0 && bestF < 85.0;
        printf("[39] trigger band filter: corners are the -3 dB points, ~24 dB/oct slopes, HP53/LP79 peaks at %.0f Hz: %s\n", bestF, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [41] The model: the curve maps the AUDIO's level; the trigger only switches between the curve and the diagonal,
    //      smoothed by Attack (trigger begins) / Release (trigger ends). The trigger's own level (after Trigger Gain, the
    //      filter, a sidechain...) never moves the audio along the curve.
    {
        TransferCurve c; const CurvePoint pts[2] = { {-60.0, -60.0}, {6.0, -60.0} }; c.loadPoints(pts, 2); // flat -60 line
        auto settled = [&](TriggerAction a, TriggerCondition cond, double trigDb, double audioDb, double thrDb) {
            CurveVoice v; v.prepare(48000.0); v.setParams(0.0, 0.0, thrDb, cond, a);
            v.reset();
            const double t = std::pow(10.0, trigDb / 20.0), au = std::pow(10.0, audioDb / 20.0);
            for (int i = 0; i < 4800; ++i) v.process(t, au, c);
            return v.gainDb();
        };
        bool ok = true;
        // trigger never met (level -80, threshold -30, Above): Curve -> diagonal (0 dB); Diagonal -> the curve at the AUDIO's level (-60 - -20 = -40)
        ok = ok && std::fabs(settled(TriggerAction::ApplyCurve, TriggerCondition::Above, -80, -20, -30)) < 1e-6;
        ok = ok && std::fabs(settled(TriggerAction::ReleaseToDiagonal, TriggerCondition::Above, -80, -20, -30) - (-40.0)) < 0.01;
        // trigger always met (level -10 > -30): the other way round
        ok = ok && std::fabs(settled(TriggerAction::ApplyCurve, TriggerCondition::Above, -10, -20, -30) - (-40.0)) < 0.01;
        ok = ok && std::fabs(settled(TriggerAction::ReleaseToDiagonal, TriggerCondition::Above, -10, -20, -30)) < 1e-6;
        // where the audio sits on the curve does not depend on how loud the trigger is (same audio, trigger 40 dB apart)
        ok = ok && std::fabs(settled(TriggerAction::ApplyCurve, TriggerCondition::Above, -25, -20, -30) - settled(TriggerAction::ApplyCurve, TriggerCondition::Above, +5, -20, -30)) < 1e-6;
        // a trigger level below the curve's left end (-60) is just "not met" -- the audio still goes to the curve when the action says so
        ok = ok && std::fabs(settled(TriggerAction::ReleaseToDiagonal, TriggerCondition::Above, -95, -12, -30) - (-48.0)) < 0.01;
        // Below condition mirrors it
        ok = ok && std::fabs(settled(TriggerAction::ApplyCurve, TriggerCondition::Below, -80, -20, -30) - (-40.0)) < 0.01;
        // smoothing: Attack governs the switch when the trigger BEGINS, Release when it ENDS (both actions)
        for (int act = 0; act < 2; ++act)
        {
            const TriggerAction a = act ? TriggerAction::ReleaseToDiagonal : TriggerAction::ApplyCurve;
            CurveVoice v; v.prepare(48000.0); v.setParams(2.0, 200.0, -30.0, TriggerCondition::Above, a); v.reset();
            const double au = std::pow(10.0, -20.0 / 20.0);
            for (int i = 0; i < 48000; ++i) v.process(1e-5, au, c);                 // no trigger, settled
            for (int i = 0; i < 480; ++i) v.process(0.3, au, c);                    // trigger begins: 10 ms = 5 attacks
            const double after10msOfTrigger = v.switchValue();
            for (int i = 0; i < 48000; ++i) v.process(0.3, au, c);                  // settled with the trigger present
            for (int i = 0; i < 480; ++i) v.process(1e-5, au, c);                   // trigger ended: 10 ms = 0.05 releases (plus the 8 ms detector release)
            const double after10msOfRelease = v.switchValue();
            const double present = (act ? 0.0 : 1.0), absent = (act ? 1.0 : 0.0);   // switch value with / without the trigger
            ok = ok && std::fabs(after10msOfTrigger - present) < 0.02 && std::fabs(after10msOfRelease - present) < 0.1 && std::fabs(absent - present) > 0.5;
        }
        printf("[41] audio-mapping model: curve read at the audio level, trigger only switches curve/diagonal (Attack = trigger begins, Release = ends): %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [42] The window is [-80, +6]; a curve saved with the old [-60, +6] window loads with EXACTLY the behaviour it had:
    //      below its old left end (-60) the gain used to hold (out - in) of that end, so a new left end at -80 carries the same
    //      gain and the old end stays as a node; a plain straight line stays 2 points; strings, Parameter-style point lists and a
    //      full curve (interior nodes, handles) all migrate; a curve already in the new window is untouched.
    {
        bool ok = TransferCurve::kMinDb == -80.0;
        // (a) default identity line saved in the old window -> the 2-point identity of the new window
        TransferCurve a; ok = ok && curveFromString("L;-60,-60,0,0,0,0;6,6,0,0,0,0", a) && a.pointCount() == 2 &&
                            a.point(0).inputDb == -80.0 && nearlyEqual(a.point(0).outputDb, -80.0, 1e-9);
        // (b) a "gate" saved as a flat -60 line: identical gains everywhere it used to be defined AND below its old end
        TransferCurve gateOld; ok = ok && curveFromString("L;-60,-60,0,0,0,0;6,-60,0,0,0,0", gateOld) && gateOld.pointCount() == 3;
        for (double lv : { -75.0, -61.0, -59.0, -40.0, -10.0, 5.0 })
        {
            const double expected = lv < -60.0 ? 0.0 : -60.0 - lv;                   // what the old window gave (hold below its left end)
            ok = ok && nearlyEqual(curveGainDb(lv, gateOld), expected, 1e-9);
        }
        // (c) a lifted left end (+15 dB above its input) keeps holding +15 dB below the old end
        TransferCurve lifted; ok = ok && curveFromString("L;-60,-45,0,0,0,0;-20,-20,0,0,0,0;6,6,0,0,0,0", lifted) &&
                             nearlyEqual(curveGainDb(-70.0, lifted), 15.0, 1e-9) && nearlyEqual(curveGainDb(-60.0, lifted), 15.0, 1e-9) &&
                             nearlyEqual(curveGainDb(-20.0, lifted), 0.0, 1e-9);
        // (d) Parameter-style loadPoints() migrates too, and a curve already in the new window is left exactly alone
        const CurvePoint viaParams[3] = { { -60.0, -50.0 }, { -20.0, -30.0 }, { 6.0, 6.0 } };
        TransferCurve b; ok = ok && b.loadPoints(viaParams, 3) && b.pointCount() == 4 && b.point(0).inputDb == -80.0 && b.point(1).inputDb == -60.0 && nearlyEqual(b.point(0).outputDb, -70.0, 1e-9);
        const CurvePoint fresh[3] = { { -80.0, -80.0 }, { -20.0, -30.0 }, { 6.0, 6.0 } };
        TransferCurve c; ok = ok && c.loadPoints(fresh, 3) && c.pointCount() == 3;
        // (e) round trip of a migrated curve does not migrate again
        TransferCurve d; ok = ok && curveFromString(curveToString(gateOld), d) && d.pointCount() == gateOld.pointCount();
        printf("[42] curve window -80..+6: old -60-window curves migrate with identical gain (identity stays 2 points), new ones untouched: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [43] Attack = Release = 0 and the trigger equal to the audio: the switch happens exactly at the threshold, sample by sample --
    //      every sample above it is on the curve, every sample below it on the diagonal (Curve action; the other way round for
    //      Diagonal), with no leftover "trigger still met" from the previous peak of the waveform.
    {
        TransferCurve c; const CurvePoint pts[2] = { {-80.0, -60.0}, {6.0, -60.0} }; c.loadPoints(pts, 2); // flat -60 line
        bool ok = true; int above = 0, below = 0;
        for (int act = 0; act < 2 && ok; ++act)
        {
            CurveVoice v; v.prepare(48000.0);
            v.setParams(0.0, 0.0, -20.0, TriggerCondition::Above, act ? TriggerAction::ReleaseToDiagonal : TriggerAction::ApplyCurve); v.reset();
            for (int i = 0; i < 48000 && ok; ++i)
            {
                const double x = 0.5 * std::sin(2.0 * M_PI * 220.0 * double(i) / 48000.0); // -6 dB peak: crosses -20 dB twice per half cycle
                const double a = std::fabs(x);
                v.process(a, a, c);
                const double lv = 20.0 * std::log10(std::max(a, 1e-12));
                if (lv < -79.0) continue;
                const bool isAbove = lv > -20.0;
                (isAbove ? above : below)++;
                const bool onCurve = (act == 0) ? isAbove : !isAbove;
                const double want = onCurve ? -60.0 - lv : 0.0;
                if (std::fabs(v.gainDb() - want) > 1e-6 || v.triggered() != isAbove) ok = false;
            }
        }
        ok = ok && above > 1000 && below > 1000;
        printf("[43] Attack/Release 0, trigger = audio: every sample switches exactly at the threshold (%d above / %d below): %s\n", above, below, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [44] A smooth curve must not "crush": with Attack/Release > 0 a smooth compressor-like curve keeps its distortion (the residual
    //      after removing the best-fit fundamental of the output) far below what per-sample waveshaping gives. This pins down a real
    //      regression of the audio-mapping redesign: the curve gain followed the level envelope's ripple directly (no gain smoothing),
    //      making the distortion 20-50 dB worse than before (measured: 100 Hz, 5/20 ms: -38 dB then, -57 dB now; 1 kHz: -56 vs -95).
    {
        TransferCurve c; c.addPoint(-30.0, -30.0); c.setPointHandle(1, 12.0, 12.0 * 0.6); c.setPoint(c.pointCount() - 1, 6.0, -10.0);
        auto residualDb = [&](double f, double attackMs, double releaseMs) {
            constexpr double sr = 48000.0; const int n = 48000;
            CurveVoice v; v.prepare(sr); v.setParams(attackMs, releaseMs, -40.0, TriggerCondition::Above); v.reset();
            std::vector<double> y(size_t(n), 0.0);
            for (int i = 0; i < n + 24000; ++i)
            {
                const double x = 0.25 * std::sin(2.0 * M_PI * f * double(i) / sr);
                const double g = v.process(std::fabs(x), std::fabs(x), c);
                if (i >= 24000) y[size_t(i - 24000)] = x * g;
            }
            double sc = 0.0, ss = 0.0;
            for (int i = 0; i < n; ++i) { sc += y[size_t(i)] * std::cos(2.0 * M_PI * f * i / sr); ss += y[size_t(i)] * std::sin(2.0 * M_PI * f * i / sr); }
            sc *= 2.0 / n; ss *= 2.0 / n;
            double res = 0.0, fund = 0.0;
            for (int i = 0; i < n; ++i) { const double fit = sc * std::cos(2.0 * M_PI * f * i / sr) + ss * std::sin(2.0 * M_PI * f * i / sr); res += (y[size_t(i)] - fit) * (y[size_t(i)] - fit); fund += fit * fit; }
            return 10.0 * std::log10(res / fund + 1e-30);
        };
        const double a = residualDb(100.0, 5.0, 20.0), b = residualDb(1000.0, 5.0, 20.0), d = residualDb(100.0, 10.0, 100.0), e = residualDb(1000.0, 0.0, 50.0);
        const bool ok = a < -50.0 && b < -85.0 && d < -75.0 && e < -100.0;
        printf("[44] smooth curve, no crush: residual 100 Hz 5/20 = %.1f dB, 1 kHz 5/20 = %.1f, 100 Hz 10/100 = %.1f, 1 kHz 0/50 = %.1f: %s\n", a, b, d, e, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [45] "Snap to Diagonal" (right-click menu, CurvePoint::diagonal): a
    //      snapped point's Y is forced to its own X on EVERY write, from any
    //      source, and the flag survives every persistence path a link
    //      already does (Parameter packing, plain-text serialization) --
    //      old data without the flag loads as not-snapped.
    {
        bool ok = true;
        // setPointDiagonal(true) snaps an interior point onto the line immediately.
        {
            TransferCurve c;
            c.addPoint(-20.0, 3.0);
            c.setPointDiagonal(1, true);
            const CurvePoint p = c.point(1);
            ok = ok && nearlyEqual(p.outputDb, p.inputDb, 1e-9) && p.diagonal;
        }
        // Once snapped, an ordinary setPoint() (a live drag) can move it in X
        // but never off the line, however far off-diagonal the requested Y is.
        {
            TransferCurve c;
            c.addPoint(-20.0, 3.0);
            c.setPointDiagonal(1, true);
            c.setPoint(1, -10.0, 55.0); // absurd Y on purpose
            const CurvePoint p = c.point(1);
            ok = ok && nearlyEqual(p.inputDb, -10.0, 1e-9) && nearlyEqual(p.outputDb, -10.0, 1e-9);
        }
        // An endpoint's X never moves (by design, see setPoint()'s own
        // comment) -- snapping it pins it to its own fixed corner.
        {
            TransferCurve c;
            c.setPointDiagonal(0, true);
            c.setPointDiagonal(c.pointCount() - 1, true);
            c.setPoint(0, 999.0, -40.0);
            c.setPoint(c.pointCount() - 1, -999.0, -40.0);
            const CurvePoint p0 = c.point(0), p1 = c.point(c.pointCount() - 1);
            ok = ok && nearlyEqual(p0.inputDb, TransferCurve::kMinDb, 1e-9) && nearlyEqual(p0.outputDb, TransferCurve::kMinDb, 1e-9) &&
                       nearlyEqual(p1.inputDb, TransferCurve::kMaxDb, 1e-9) && nearlyEqual(p1.outputDb, TransferCurve::kMaxDb, 1e-9);
        }
        // Still fully linkable: an X-only-linked pair, one of them snapped --
        // moveLinked() (numeric node-list edit) drags them together, the
        // snapped member's Y still tracks its own X, not the other member's.
        {
            TransferCurve c;
            c.addPoint(-40.0, -10.0);
            c.addPoint(-20.0, 5.0);
            c.setPointDiagonal(1, true); // the interior point at -40
            c.setPointLink(1, 1, kLinkAxisX);
            c.setPointLink(2, 1, kLinkAxisX);
            c.normalizeLinks();
            double thr = -18.0;
            c.moveLinked(2, 5.0, 0.5, thr); // drag the OTHER member; both should shift +5 in X
            const CurvePoint p1 = c.point(1), p2 = c.point(2);
            ok = ok && nearlyEqual(p1.inputDb, -35.0, 1e-9) && nearlyEqual(p1.outputDb, -35.0, 1e-9) && p1.link == 1 &&
                       nearlyEqual(p2.inputDb, -15.0, 1e-9) && nearlyEqual(p2.outputDb, 5.5, 1e-9);
        }
        // Parameter-mirror packing: direction + link + axis + diagonal all
        // round-trip together; a pre-existing (non-diagonal) packed value
        // decodes exactly as it always did (no diagonal flag).
        {
            const float packed = packDirectionAndLink(PointDirection::RisingOnly, 4, kLinkAxisY, true);
            ok = ok && unpackDirection(packed) == PointDirection::RisingOnly && unpackLink(packed) == 4 &&
                       unpackLinkAxis(packed) == kLinkAxisY && unpackPointDiagonal(packed);
            const float legacy = packDirectionAndLink(PointDirection::FallingOnly, 2, kLinkAxisX, false);
            ok = ok && unpackLinkAxis(legacy) == kLinkAxisX && !unpackPointDiagonal(legacy);
            // An older build's packing (no diagonal parameter at all) must
            // still decode as "not snapped".
            ok = ok && !unpackPointDiagonal(float(int(PointDirection::Both) + 3 * (5 + 10 * kLinkAxisBoth)));
        }
        // Plain-text serialization round-trips the flag; a string saved
        // before this field existed (5-6 comma fields) loads as unsnapped.
        {
            TransferCurve c;
            c.addPoint(-20.0, 3.0);
            c.setPointDiagonal(1, true);
            const std::string s = curveToString(c);
            TransferCurve d;
            ok = ok && curveFromString(s, d) && d.point(1).diagonal && nearlyEqual(d.point(1).outputDb, d.point(1).inputDb, 1e-9);
            TransferCurve e;
            ok = ok && curveFromString("L;-80,-80,0,0,0;6,6,0,0,0", e) && !e.point(0).diagonal && !e.point(1).diagonal;
        }
        printf("[45] Snap to Diagonal: immediate snap, live drag stays pinned, endpoints pin to their own corner, still linkable "
               "(link carries it, its own Y stays on its own X), Parameter-packing and string round-trips, old data loads unsnapped: %s\n",
               ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    printf("\n%s: %d test(s) failed\n", failures == 0 ? "PASS" : "FAILURES", failures);
    return failures == 0 ? 0 : 1;
}
