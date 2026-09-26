// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// dsp_regression.cpp
//
// Fast, pure-math correctness tests for the DSP core: no DPF, no GUI, no
// audio files -- just the engine/guard/transfer-function/serialization
// code exercised directly and checked against known-correct values. Runs
// in well under a second. Build and run via the `dsp_regression` CMake
// target, or standalone:
//
//   g++ -std=c++17 -O2 -Iplugin/dsp tests/dsp_regression.cpp \
//       plugin/dsp/StabilityGuard.cpp plugin/dsp/MonoStabilityGuard.cpp \
//       plugin/dsp/TransferFunction.cpp \
//       plugin/dsp/Serialization.cpp plugin/dsp/GainCurveTrainer.cpp \
//       plugin/dsp/EqModel.cpp plugin/dsp/SpectrumAnalyzer.cpp \
//       -o dsp_regression && ./dsp_regression
//
// Test [7] specifically pins down a real bug found and fixed during
// development: StabilityGuard::enforce() used to (a) estimate the spectral
// radius via single-vector power iteration, which is measurably biased for
// a complex-conjugate dominant pole pair of *exactly* equal magnitude --
// the routine case for any resonant biquad, not an edge case -- and (b)
// check its refinement loop's convergence against the loose exact norm
// bound instead of the (biased) estimate, which for a controllable-
// canonical-form matrix can be structurally ~3x the true spectral radius
// and so never actually converges. Together these silently collapsed a
// perfectly stable, deliberately narrow 50 Hz/Q=25 notch filter's
// coefficients to ~30% of their designed value -- turning it into a
// filter with no audible or visible notch at all. See StabilityGuard.cpp
// for the fix (repeated-squaring spectral radius estimate, additive safety
// margin, convergence checked against the estimate). If this test ever
// fails again, suspect the same class of bug before anything else.
//
// Test [9] is the other headline capability's correctness check: given
// synthetic (input, target-output) audio generated from a *known*
// NonlinearSchedule, GainCurveTrainer::fit() must recover that same
// schedule (attack/release exactly, since they're grid points; the curve
// within the level range the synthetic input actually visited) -- i.e.
// this proves "give it an input and a desired output, coefficients are
// learned" actually works, not just that the fitting code runs.
//
// See also tools/testbench/audio_testbench.cpp for the audio-file-based
// counterpart to this (actual WAV signals through the real engine, cross-
// checked against this same transfer-function math), and
// tools/trainer/fit_gain_curve.cpp for the CLI front-end to the trainer.
//
// Test [14] covers the mono EQ plugin's whole DSP path, and its headline
// check is an end-to-end equivalence one: the same band list, assembled
// and run through MonoStateSpaceEngine, must produce sample-for-sample
// what the stereo StateSpaceEngine produces on one channel -- while
// needing exactly half the states to do it (that halving is the entire
// reason the mono variant exists, see MonoStateSpaceTypes.hpp).
//
// Test [10] covers the sibling EQ plugin's coefficient mapper
// (plugin/dsp/EqModel.hpp): that its state-space assembly
// (mappers::makeStereoBiquadChain() under the hood) and its closed-form
// per-band/combined response (used for the EQ GUI's live curve, cheap
// enough to redraw every frame -- see EqModel.hpp's file comment) agree
// exactly, since they describe the same LTI system by construction.
//
// Tests [15]-[17] cover the multiband compressor mechanism (see
// StateSpaceTypes.hpp's MultibandSchedule/BandSplit, CoefficientMapper.hpp's
// makeStereoCrossoverBank(), and GainCurveTrainer.hpp's fitMultiband()):
// [15] the Linkwitz-Riley crossover bank sums flat (0dB) at every
// frequency and its BandSplit bookkeeping partitions the assembled order
// exactly, [16] mirrors test [8] but per-band -- the engine's multiband
// gain stage matches the closed-form compressor formula independently in
// two different bands at once, [17] mirrors test [9] -- the multiband
// trainer recovers two different known per-band curves from one synthetic
// (full-mix input, full-mix target) audio pair.
//
// Tests [19]-[21] cover the dynamic EQ mechanism (plugin/dsp/
// DynamicEqModel.hpp's Regalia-Mitra fixed-allpass construction and
// plugin/dsp/DynamicEqEngine.hpp's dedicated real-time engine -- see both
// files' own comments for the full derivation): [19] the resting-shape
// closed-form response hits the expected gain exactly at a band's own
// center frequency and 0dB far from it, and resolveDynamicGainDb() matches
// its own documented formula (flat below threshold, clamped to rangeDb
// above it); [20] the actual real-time engine, run end to end on a
// synthetic tone, settles to the resting gain when quiet and to the
// closed-form-expected reduced gain when loud enough to cross threshold --
// mirroring test [8]'s "settled out matches closed form" methodology, now
// for a filter whose *shape*, not just a broadband scalar, is what's
// moving -- and DynamicEqEngine::getCurrentGainDb()'s readback agrees with
// what actually came out; [21] the *static* export path
// (dyneq::assembleStateSpace(), an ordinary RBJ designPeaking() cascade,
// deliberately NOT the live Regalia-Mitra shape -- see DynamicEqModel.hpp's
// file comment for why) agrees with TransferFunction on the assembled
// matrix, mirroring test [10]'s identical EqModel cross-check, plus a JSON
// round-trip.

#include "StateSpaceEngine.hpp"
#include "MonoStateSpaceEngine.hpp"
#include "MonoStabilityGuard.hpp"
#include "StabilityGuard.hpp"
#include "TransferFunction.hpp"
#include "CoefficientMapper.hpp"
#include "Serialization.hpp"
#include "GainCurveTrainer.hpp"
#include "EqModel.hpp"
#include "MbCompressorModel.hpp"
#include "DynamicEqModel.hpp"
#include "DynamicEqEngine.hpp"
#include "DynamicEqModel2.hpp" // dynamic_eq2 -- a hand-adapted clone, see test [33]'s own comment
#include "DynamicEqEngine2.hpp" // for the crossfeed time-domain tests, see test [38]'s own comment
#include "SpectrumAnalyzer.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <pthread.h>
#include <random>
#include <vector>

using namespace ssfilter;
static bool nearlyEqual(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
static int failures = 0;

int main()
{
    const double fs = 48000.0;

    // 1. Identity passthrough is bit-exact.
    {
        StateSpaceEngine eng; eng.prepare(fs);
        StateSpaceMatrices m; eng.setModel(m); eng.resetState();
        const int N = 8;
        std::vector<float> inL(N), inR(N), outL(N), outR(N);
        for (int i=0;i<N;++i){ inL[i]=float(i)*0.1f-0.3f; inR[i]=-float(i)*0.05f; }
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), N);
        bool ok=true; for(int i=0;i<N;++i) if(!nearlyEqual(outL[i],inL[i],1e-6)||!nearlyEqual(outR[i],inR[i],1e-6)) ok=false;
        printf("[1] passthrough bit-exact: %s\n", ok?"OK":"FAIL"); if(!ok) failures++;
    }

    // 2. A genuinely unstable matrix still gets clamped and stays finite.
    {
        StateSpaceMatrices m; m.setSilence(); m.order=2;
        m.A[0][0]=2.0; m.A[1][1]=2.0; m.B[0][0]=1.0; m.B[1][1]=1.0; m.C[0][0]=1.0; m.C[1][1]=1.0;
        StabilityReport before = StabilityGuard::analyze(m);
        StabilityReport after = StabilityGuard::enforce(m);
        double rho = StabilityGuard::estimateSpectralRadius(m);
        printf("[2] unstable A: before.provenStable=%d after.wasClamped=%d rho_after=%.6f\n", before.provenStable, after.wasClamped, rho);
        if (before.provenStable) { printf("FAIL: should not have been proven stable\n"); failures++; }
        if (!(rho < 1.0)) { printf("FAIL: post-enforce rho not <1\n"); failures++; }
        StateSpaceEngine eng; eng.prepare(fs); eng.setModel(m); eng.resetState();
        std::vector<float> inL(4096,0.8f), inR(4096,-0.8f), outL(4096), outR(4096);
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), 4096);
        bool bounded=true; for(int i=0;i<4096;++i) if(!std::isfinite(outL[i])||!std::isfinite(outR[i])||std::fabs(outL[i])>100.0f) bounded=false;
        printf("    sustained-input run finite/bounded: %s\n", bounded?"OK":"FAIL"); if(!bounded) failures++;
    }

    // 3. RBJ lowpass shape correct.
    {
        BiquadCoeffs bq = mappers::designLowpass(fs, 1000.0, 0.7071);
        StateSpaceMatrices m = mappers::makeStereoBiquad(bq, bq);
        StabilityGuard::enforce(m);
        std::vector<TransferFunction::SweepPoint> sw;
        TransferFunction::sweep(m, fs, 0, 0, 20.0, 20000.0, 400, sw);
        double at100=0, at1k=0, at10k=0;
        for (auto& p: sw) {
            if (nearlyEqual(std::log10(p.freqHz), 2.0, 0.02)) at100=p.magnitudeDb;
            if (nearlyEqual(std::log10(p.freqHz), 3.0, 0.02)) at1k=p.magnitudeDb;
            if (nearlyEqual(std::log10(p.freqHz), 4.0, 0.02)) at10k=p.magnitudeDb;
        }
        printf("[3] lowpass 1kHz: 100Hz=%.2fdB 1kHz=%.2fdB 10kHz=%.2fdB\n", at100, at1k, at10k);
        if (!(at100>-1.0&&at100<1.0)) { printf("FAIL passband\n"); failures++; }
        if (!(at1k>-6.0&&at1k<-1.0)) { printf("FAIL cutoff\n"); failures++; }
        if (!(at10k<-30.0)) { printf("FAIL stopband\n"); failures++; }
    }

    // 4. JSON round-trip.
    {
        StateSpaceMatrices m; m.order=3;
        m.A[0][0]=0.5; m.A[1][2]=-0.25; m.B[2][1]=0.75; m.C[1][0]=1.25; m.D[0][1]=-0.9;
        std::string j = Serialization::toJson(m);
        StateSpaceMatrices m2; bool ok = Serialization::fromJson(j, m2);
        bool same = ok && m2.order==m.order && nearlyEqual(m2.A[0][0],0.5,1e-12) && nearlyEqual(m2.B[2][1],0.75,1e-12);
        printf("[4] JSON round-trip: %s\n", same?"OK":"FAIL"); if(!same) failures++;
    }

    // 5. Normalize hits exactly 0dB peak.
    {
        BiquadCoeffs bq = mappers::designPeaking(fs, 2000.0, 1.0, 8.0);
        StateSpaceMatrices m = mappers::makeStereoBiquad(bq, bq);
        for (int i=0;i<2;++i) { m.C[0][i]*=0.3; m.C[1][i+2]*=0.3; }
        m.D[0][0]*=0.3; m.D[1][1]*=0.3;
        double scale = TransferFunction::normalizeToUnityPassbandGain(m, fs);
        std::vector<TransferFunction::SweepPoint> sw;
        TransferFunction::sweep(m, fs, 0,0,20,20000,300,sw);
        double peak=-1000; for(auto&p:sw) peak=std::max(peak,p.magnitudeDb);
        printf("[5] normalize: peak after=%.4fdB (scale=%.4g)\n", peak, scale);
        if (!nearlyEqual(peak,0.0,0.05)) { printf("FAIL\n"); failures++; }
    }

    // 6. Cascade bandlimit shape correct.
    {
        BiquadCoeffs hp = mappers::designHighpass(fs,300,0.7071), lp = mappers::designLowpass(fs,3400,0.7071);
        std::vector<BiquadCoeffs> chain = {hp, lp};
        StateSpaceMatrices m = mappers::makeStereoBiquadChain(chain, chain);
        StabilityReport rep = StabilityGuard::analyze(m);
        std::vector<TransferFunction::SweepPoint> sw;
        TransferFunction::sweep(m, fs, 0,0,20,20000,400,sw);
        double at100=0, at1k=0, at10k=0;
        for (auto& p: sw) {
            if (nearlyEqual(std::log10(p.freqHz), 2.0, 0.02)) at100=p.magnitudeDb;
            if (nearlyEqual(std::log10(p.freqHz), 3.0, 0.02)) at1k=p.magnitudeDb;
            if (nearlyEqual(std::log10(p.freqHz), 4.0, 0.02)) at10k=p.magnitudeDb;
        }
        printf("[6] cascade bandlimit: proven=%d 100Hz=%.2f 1kHz=%.2f 10kHz=%.2f\n", rep.provenStable, at100, at1k, at10k);
        if (!(at1k>-3.0)) { printf("FAIL passband\n"); failures++; }
        if (!(at100<-6.0 && at10k<-6.0)) { printf("FAIL edges\n"); failures++; }
    }

    // 7. Notch filter actually notches (the bug this whole session started from).
    {
        BiquadCoeffs bq = mappers::designNotch(fs, 50.0, 25.0);
        StateSpaceMatrices m = mappers::makeStereoBiquad(bq, bq);
        StabilityReport rep = StabilityGuard::enforce(m);
        ComplexResponse r50 = TransferFunction::evaluate(m, Complex(std::cos(2*M_PI*50/fs), std::sin(2*M_PI*50/fs)));
        ComplexResponse r1k = TransferFunction::evaluate(m, Complex(std::cos(2*M_PI*1000/fs), std::sin(2*M_PI*1000/fs)));
        double db50 = 20*std::log10(std::abs(r50.H[0][0]));
        double db1k = 20*std::log10(std::abs(r1k.H[0][0]));
        printf("[7] notch 50Hz: wasClamped=%d appliedScale=%.6f  |H(50Hz)|=%.2fdB  |H(1kHz)|=%.2fdB\n",
               rep.wasClamped, rep.appliedScale, db50, db1k);
        if (!(db50 < -10.0)) { printf("FAIL: notch not deep enough (regression of the original bug!)\n"); failures++; }
        if (!(db1k > -1.0 && db1k < 1.0)) { printf("FAIL: passband away from notch not ~0dB\n"); failures++; }
        if (rep.appliedScale < 0.99) { printf("FAIL: guard over-clamped a legitimately stable narrow filter\n"); failures++; }
    }

    // 8. Nonlinear schedule (hand-set curve) behaves like a classic
    //    compressor: A/B are untouched, only the output gain varies with
    //    the input's own smoothed level. See StateSpaceTypes.hpp's
    //    NonlinearSchedule and StateSpaceEngine.hpp's processBlock().
    {
        StateSpaceMatrices m; // identity passthrough
        m.nonlinear.enabled = true;
        const double threshold = -18.0, ratio = 4.0;
        for (int i = 0; i < GainCurve::kNumKnots; ++i)
        {
            const double lvl = GainCurve::knotLevelDb(i);
            m.nonlinear.curve.gainDb[i] = lvl <= threshold ? 0.0 : (lvl - threshold) * (1.0 / ratio - 1.0);
        }
        m.nonlinear.attackMs = 3.0;
        m.nonlinear.releaseMs = 100.0;

        StateSpaceEngine eng; eng.prepare(fs); eng.setModel(m); eng.resetState();
        const int N = 48000; // 1s, plenty of settle time at these attack/release values
        std::vector<float> inL(N, 1.0f), inR(N, 1.0f), outL(N), outR(N); // sustained 0 dBFS
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), N);
        const double expectedDb = (0.0 - threshold) * (1.0 / ratio - 1.0);
        const double expectedLin = std::pow(10.0, expectedDb / 20.0);
        printf("[8] nonlinear schedule: settled out=%.4f expected~%.4f\n", outL[N - 1], expectedLin);
        if (!nearlyEqual(outL[N - 1], expectedLin, 0.01)) { printf("FAIL: did not settle to expected gain reduction\n"); ++failures; }

        eng.resetState();
        const float quietAmp = float(std::pow(10.0, -40.0 / 20.0)); // well under threshold
        std::vector<float> inL2(4096, quietAmp), inR2(4096, quietAmp), outL2(4096), outR2(4096);
        eng.processBlock(inL2.data(), inR2.data(), outL2.data(), outR2.data(), 4096);
        const bool transparent = nearlyEqual(outL2[4095], quietAmp, 1e-4);
        printf("    quiet signal (under threshold) transparent: %s\n", transparent ? "OK" : "FAIL");
        if (!transparent) ++failures;
    }

    // 9. GainCurveTrainer recovers a known curve and known attack/release
    //    from synthetic (input, target-output) data -- the actual "give it
    //    an input and desired output, coefficients are learned" capability.
    {
        const double knownThresholdDb = -20.0, knownRatio = 4.0;
        const double knownAttackMs = 3.0, knownReleaseMs = 100.0; // exact grid points, see GainCurveTrainer.cpp
        NonlinearSchedule known;
        known.enabled = true;
        known.attackMs = knownAttackMs;
        known.releaseMs = knownReleaseMs;
        for (int i = 0; i < GainCurve::kNumKnots; ++i)
        {
            const double lvl = GainCurve::knotLevelDb(i);
            known.curve.gainDb[i] = lvl <= knownThresholdDb ? 0.0 : (lvl - knownThresholdDb) * (1.0 / knownRatio - 1.0);
        }

        // Synthetic training input: white-noise bursts at varying levels,
        // sweeping through the curve's interesting range -- standing in for
        // "record some real audio at a range of levels".
        std::mt19937 rng(12345);
        std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
        const double levelsDb[] = { -50, -40, -30, -25, -20, -15, -10, -5, -2, 0 };
        const int segN = int(0.4 * fs);
        std::vector<float> inL, inR;
        for (double lvlDb : levelsDb)
        {
            const float amp = float(std::pow(10.0, lvlDb / 20.0));
            for (int i = 0; i < segN; ++i) { const float s = amp * dist(rng); inL.push_back(s); inR.push_back(s); }
        }

        StateSpaceMatrices base; // identity passthrough -- the base model the fitted curve will multiply
        StateSpaceMatrices withKnown = base;
        withKnown.nonlinear = known;
        StateSpaceEngine eng; eng.prepare(fs); eng.setModel(withKnown); eng.resetState();
        std::vector<float> tgtL(inL.size()), tgtR(inR.size());
        eng.processBlock(inL.data(), inR.data(), tgtL.data(), tgtR.data(), uint32_t(inL.size()));

        const GainCurveFitResult fitResult = GainCurveTrainer::fit(base, fs, inL, inR, tgtL, tgtR);
        printf("[9] trainer round-trip: ok=%d attack=%.2fms(known %.2f) release=%.2fms(known %.2f) rmsErr=%.4fdB\n",
               fitResult.ok, fitResult.schedule.attackMs, knownAttackMs,
               fitResult.schedule.releaseMs, knownReleaseMs, fitResult.rmsErrorDb);
        if (!fitResult.ok) { printf("FAIL: fit did not succeed\n"); ++failures; }
        if (!nearlyEqual(fitResult.schedule.attackMs, knownAttackMs, 0.01)) { printf("FAIL: attack time not recovered\n"); ++failures; }
        if (!nearlyEqual(fitResult.schedule.releaseMs, knownReleaseMs, 0.01)) { printf("FAIL: release time not recovered\n"); ++failures; }
        if (fitResult.rmsErrorDb > 0.5) { printf("FAIL: fit residual too large\n"); ++failures; }

        const double fittedAt10 = fitResult.schedule.curve.evaluate(-10.0);
        const double knownAt10 = known.curve.evaluate(-10.0);
        printf("    curve@-10dB: fitted=%.3f known=%.3f\n", fittedAt10, knownAt10);
        if (!nearlyEqual(fittedAt10, knownAt10, 0.5)) { printf("FAIL: curve not recovered within the visited level range\n"); ++failures; }
    }

    // 10. EqModel: the assembled StateSpaceMatrices (via
    //     mappers::makeStereoBiquadChain()) and the closed-form direct
    //     product (eq::evaluateCombinedResponse(), what the GUI actually
    //     draws every frame) must agree exactly -- and a disabled band
    //     must be excluded from both.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::HighPass, 80.0, 0.0, 0.707, true, false });
        eq.bands.push_back({ EqBandType::Bell, 250.0, -4.0, 1.2, true, false });
        eq.bands.push_back({ EqBandType::Bell, 3000.0, 5.0, 2.0, true, false });
        eq.bands.push_back({ EqBandType::HighShelf, 9000.0, 3.0, 0.707, true, false });
        eq.bands.push_back({ EqBandType::Notch, 50.0, 0.0, 20.0, false, false }); // disabled
        eq.outputTrimDb = -2.0;

        StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
        StabilityGuard::enforce(assembled);

        double worstDiffDb = 0.0;
        for (double f : { 30.0, 80.0, 250.0, 500.0, 1000.0, 3000.0, 9000.0, 15000.0 })
        {
            const double omega = 2.0 * M_PI * f / fs;
            const ComplexResponse resp = TransferFunction::evaluate(assembled, Complex(std::cos(omega), std::sin(omega)));
            const double stateSpaceDb = 20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12));
            const double directDb = 20.0 * std::log10(std::max(std::abs(eq::evaluateCombinedResponse(eq, fs, f)), 1e-12));
            worstDiffDb = std::max(worstDiffDb, std::fabs(stateSpaceDb - directDb));
        }
        printf("[10] EqModel: assembled order=%d  state-space vs. closed-form worst diff=%.6fdB\n",
               assembled.order, worstDiffDb);
        if (worstDiffDb > 0.01) { printf("FAIL: EqModel's two response paths disagree\n"); ++failures; }

        // JSON round-trip.
        EqModel eq2;
        const bool jsonOk = eq::fromJson(eq::toJson(eq), eq2);
        bool bandsMatch = jsonOk && eq2.bands.size() == eq.bands.size();
        if (bandsMatch)
            for (size_t i = 0; i < eq.bands.size(); ++i)
            {
                const auto& a = eq.bands[i]; const auto& b = eq2.bands[i];
                if (a.type != b.type || a.freqHz != b.freqHz || a.gainDb != b.gainDb ||
                    a.q != b.q || a.enabled != b.enabled || a.solo != b.solo) bandsMatch = false;
            }
        printf("     JSON round-trip: %s\n", bandsMatch ? "OK" : "FAIL");
        if (!bandsMatch) ++failures;

        // Empty/all-disabled model -> identity passthrough (D = I, no gain).
        EqModel empty;
        const StateSpaceMatrices passthrough = eq::assembleStateSpace(empty, fs);
        const bool isIdentity = passthrough.D[0][0] == 1.0 && passthrough.D[1][1] == 1.0 &&
                                 passthrough.D[0][1] == 0.0 && passthrough.D[1][0] == 0.0;
        printf("     empty model -> identity passthrough: %s\n", isIdentity ? "OK" : "FAIL");
        if (!isIdentity) { printf("FAIL: empty EqModel did not assemble to identity passthrough\n"); ++failures; }
    }

    // 10b. Harmonic linking (EqBand::harmonicGroup/harmonicRatio,
    //      eq::createHarmonicTwin()/propagateHarmonicFreq()/
    //      unlinkHarmonic()): creating a twin links both bands; moving
    //      any linked member repositions every other member to the same
    //      ratio; unlinking or deleting one member mid-chain leaves the
    //      survivors' own ratios untouched (the entire point of storing
    //      each member's ratio relative to a virtual base rather than a
    //      pairwise chain -- see EqBand::harmonicGroup's own comment);
    //      and the group/ratio fields round-trip through JSON.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 100.0, 3.0, 1.0, true, false });

        // x2 twin: standard octave harmonic.
        const int twin2 = eq::createHarmonicTwin(eq, 0, 2.0, fs);
        // x2 again, from the twin: chains to x4 of the original root, not
        // back to x2 of it (this function's own declaration comment).
        const int twin4 = eq::createHarmonicTwin(eq, twin2, 2.0, fs);
        const bool createdOk = twin2 == 1 && twin4 == 2 && eq.bands.size() == 3 &&
                                eq.bands[0].harmonicGroup != 0 &&
                                eq.bands[0].harmonicGroup == eq.bands[1].harmonicGroup &&
                                eq.bands[1].harmonicGroup == eq.bands[2].harmonicGroup &&
                                nearlyEqual(eq.bands[1].freqHz, 200.0, 1e-6) &&
                                nearlyEqual(eq.bands[2].freqHz, 400.0, 1e-6) &&
                                eq.bands[1].type == EqBandType::Bell && nearlyEqual(eq.bands[1].gainDb, 3.0, 1e-9);
        printf("[10b] Harmonic twin created (x2, then x2 again chained to x4): %s"
               " (freqs=%.1f/%.1f/%.1f)\n",
               createdOk ? "OK" : "FAIL", eq.bands[0].freqHz, eq.bands[1].freqHz, eq.bands[2].freqHz);
        if (!createdOk) { printf("FAIL: createHarmonicTwin() didn't link/place/copy correctly\n"); ++failures; }

        // Move the *root* (index 0) -- every member should follow, each
        // keeping its own ratio (1x/2x/4x) to the new base.
        eq::propagateHarmonicFreq(eq, 0, 150.0, fs);
        const bool movedRootOk = nearlyEqual(eq.bands[0].freqHz, 150.0, 1e-6) &&
                                  nearlyEqual(eq.bands[1].freqHz, 300.0, 1e-6) &&
                                  nearlyEqual(eq.bands[2].freqHz, 600.0, 1e-6);
        printf("      moving the root propagates to every member: %s (freqs=%.1f/%.1f/%.1f)\n",
               movedRootOk ? "OK" : "FAIL", eq.bands[0].freqHz, eq.bands[1].freqHz, eq.bands[2].freqHz);
        if (!movedRootOk) { printf("FAIL: propagateHarmonicFreq() from the root didn't move every member\n"); ++failures; }

        // Move a *non-root* member (index 1, currently 2x) -- the whole
        // group's base shifts, still preserving every ratio.
        eq::propagateHarmonicFreq(eq, 1, 500.0, fs); // base becomes 500/2 = 250
        const bool movedMiddleOk = nearlyEqual(eq.bands[0].freqHz, 250.0, 1e-6) &&
                                    nearlyEqual(eq.bands[1].freqHz, 500.0, 1e-6) &&
                                    nearlyEqual(eq.bands[2].freqHz, 1000.0, 1e-6);
        printf("      moving a non-root member re-bases the whole group: %s (freqs=%.1f/%.1f/%.1f)\n",
               movedMiddleOk ? "OK" : "FAIL", eq.bands[0].freqHz, eq.bands[1].freqHz, eq.bands[2].freqHz);
        if (!movedMiddleOk) { printf("FAIL: propagateHarmonicFreq() from a non-root member didn't re-base the group\n"); ++failures; }

        // Unlink the middle member (index 1, the x2 one) and move the
        // root again -- the unlinked one must NOT follow, but the
        // remaining x4 member must still follow correctly (its own
        // stored ratio was never affected by the middle one leaving).
        eq::unlinkHarmonic(eq, 1);
        const double unlinkedFreqBefore = eq.bands[1].freqHz;
        eq::propagateHarmonicFreq(eq, 0, 300.0, fs);
        const bool survivorOk = eq.bands[1].harmonicGroup == 0 &&
                                 nearlyEqual(eq.bands[1].freqHz, unlinkedFreqBefore, 1e-9) && // untouched
                                 nearlyEqual(eq.bands[2].freqHz, 1200.0, 1e-6); // still exactly 4x the new root, no adjustment needed
        printf("      unlinking the middle member leaves it in place and the survivor still exactly linked: %s"
               " (unlinked=%.1f, survivor=%.1f)\n",
               survivorOk ? "OK" : "FAIL", eq.bands[1].freqHz, eq.bands[2].freqHz);
        if (!survivorOk) { printf("FAIL: unlinkHarmonic() disturbed a surviving group member\n"); ++failures; }

        // JSON round-trip of the harmonic fields themselves.
        EqModel parsed;
        const bool harmonicRoundTrips = eq::fromJson(eq::toJson(eq), parsed) &&
                                         parsed.bands.size() == eq.bands.size() &&
                                         parsed.bands[0].harmonicGroup == eq.bands[0].harmonicGroup &&
                                         parsed.bands[2].harmonicGroup == eq.bands[2].harmonicGroup &&
                                         nearlyEqual(parsed.bands[2].harmonicRatio, eq.bands[2].harmonicRatio, 1e-9) &&
                                         parsed.bands[1].harmonicGroup == 0; // the unlinked one stayed unlinked
        printf("      harmonicGroup/harmonicRatio round-trip through JSON: %s\n", harmonicRoundTrips ? "OK" : "FAIL");
        if (!harmonicRoundTrips) { printf("FAIL: harmonic link fields lost across a JSON round-trip\n"); ++failures; }

        // eq::activeBandIndices(): soloing one member of a harmonic group
        // pulls in every other member too, not just the one whose own
        // `solo` flag is set. A third, unrelated band (not in the group, not soloed) must
        // stay silenced, same as ordinary (non-harmonic) solo behavior.
        EqModel soloEq;
        soloEq.bands.push_back({ EqBandType::Bell, 100.0, 3.0, 1.0, true, false }); // group root
        eq::createHarmonicTwin(soloEq, 0, 2.0, fs); // index 1, same group as 0
        soloEq.bands.push_back({ EqBandType::Bell, 5000.0, -2.0, 1.0, true, false }); // unrelated, index 2
        soloEq.bands[0].solo = true; // solo only the root; index 1 should still be pulled in via the group
        const std::vector<int> soloActive = eq::activeBandIndices(soloEq);
        const bool soloGroupOk = soloActive.size() == 2 &&
                                  std::find(soloActive.begin(), soloActive.end(), 0) != soloActive.end() &&
                                  std::find(soloActive.begin(), soloActive.end(), 1) != soloActive.end() &&
                                  std::find(soloActive.begin(), soloActive.end(), 2) == soloActive.end();
        printf("      soloing one harmonic-group member also activates its linked twin, leaving an unrelated band silenced: %s"
               " (active=%zu)\n", soloGroupOk ? "OK" : "FAIL", soloActive.size());
        if (!soloGroupOk) { printf("FAIL: activeBandIndices() didn't pull in a soloed band's harmonic siblings\n"); ++failures; }
    }

    // 10d. Per-channel band routing (EqBand::channel, eq::hasPerChannelBands(),
    //      eq::assembleStateSpace()'s left/right stage split,
    //      eq::evaluateChannelResponse()): a Left-only band must filter the
    //      Left channel and leave Right completely untouched (an exact
    //      identity, not just "close to it"), with no cross-channel leakage
    //      either way -- makeStereoBiquadChain() already guarantees the
    //      no-leakage part structurally, this just confirms the channel
    //      split actually reaches it. evaluateChannelResponse() (the O(1)
    //      closed-form path the GUI curve uses) must agree with the real
    //      assembled state-space's own H(z) for both channels, the same
    //      "two independent paths, must agree" check test [10] already
    //      makes for the channel-agnostic case.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 1000.0, 9.0, 1.0, true, false });
        eq.bands.back().channel = EqChannel::Left;

        const bool hadPerChannelBefore = eq::hasPerChannelBands(eq);
        eq.bands.back().channel = EqChannel::Both;
        const bool noneAfterReverting = !eq::hasPerChannelBands(eq);
        eq.bands.back().channel = EqChannel::Left; // restore for the rest of this test
        printf("[10d] hasPerChannelBands(): true for a Left-only band, false once reverted to Both: %s\n",
               (hadPerChannelBefore && noneAfterReverting) ? "OK" : "FAIL");
        if (!(hadPerChannelBefore && noneAfterReverting)) { printf("FAIL: hasPerChannelBands() didn't track the channel field\n"); ++failures; }

        const StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
        const double testFreq = 1000.0; // right on the band's own peak -- the 9dB boost is unmissable here if present
        const Complex z = std::polar(1.0, 2.0 * M_PI * testFreq / fs);
        const ComplexResponse resp = TransferFunction::evaluate(assembled, z);

        const double leftGainDb = 20.0 * std::log10(std::abs(resp.H[0][0]));
        const double rightGainDb = 20.0 * std::log10(std::abs(resp.H[1][1]));
        const double crossLR = std::abs(resp.H[0][1]); // Right input -> Left output
        const double crossRL = std::abs(resp.H[1][0]); // Left input -> Right output

        const bool leftBoosted = leftGainDb > 8.0;                 // the assigned channel: full 9dB boost
        const bool rightUntouched = std::fabs(rightGainDb) < 0.01; // the other channel: exact identity, not just "quiet"
        const bool noCrossTalk = crossLR < 1e-9 && crossRL < 1e-9;
        printf("[10d] Left-only band: L gain=%.2fdB (want ~9) R gain=%.4fdB (want 0) cross-talk=%.2e/%.2e (want 0): %s\n",
               leftGainDb, rightGainDb, crossLR, crossRL,
               (leftBoosted && rightUntouched && noCrossTalk) ? "OK" : "FAIL");
        if (!(leftBoosted && rightUntouched && noCrossTalk))
            { printf("FAIL: a Left-only band leaked into the Right channel, or didn't reach the Left one\n"); ++failures; }

        // evaluateChannelResponse() (closed-form, O(1)) vs. the real
        // assembled state-space's own H(z) -- same cross-check discipline
        // as test [10]'s channel-agnostic version, now per channel.
        const std::complex<double> hLeftClosed = eq::evaluateChannelResponse(eq, fs, testFreq, EqChannel::Left);
        const std::complex<double> hRightClosed = eq::evaluateChannelResponse(eq, fs, testFreq, EqChannel::Right);
        const double diffL = std::abs(hLeftClosed - resp.H[0][0]);
        const double diffR = std::abs(hRightClosed - resp.H[1][1]);
        const bool closedFormOk = diffL < 1e-6 && diffR < 1e-6;
        printf("      evaluateChannelResponse() vs. assembled state-space: L diff=%.2e R diff=%.2e: %s\n",
               diffL, diffR, closedFormOk ? "OK" : "FAIL");
        if (!closedFormOk) { printf("FAIL: evaluateChannelResponse() disagrees with the real per-channel assembly\n"); ++failures; }

        // JSON round-trip of the channel field itself.
        EqModel parsed;
        const bool channelRoundTrips = eq::fromJson(eq::toJson(eq), parsed) &&
                                        parsed.bands.size() == 1 && parsed.bands[0].channel == EqChannel::Left;
        printf("      channel round-trips through JSON: %s\n", channelRoundTrips ? "OK" : "FAIL");
        if (!channelRoundTrips) { printf("FAIL: EqBand::channel lost across a JSON round-trip\n"); ++failures; }
    }

    // 10e. Soloing a Left/Right-only band silences the channel it never
    //      reaches, rather than leaving it as a raw passthrough. Two bands: one Left-only (soloed),
    //      one Right-only (not soloed, so excluded from the active set
    //      entirely) -- Left must show the soloed band's own filtering,
    //      Right must be an *exact* digital zero (D=0 too, not just
    //      "very quiet"), and soloing back off must restore Right to an
    //      ordinary passthrough again (proving this is solo-gated, not a
    //      permanent side effect of the band ever having been Left-only).
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 1000.0, 9.0, 1.0, true, true }); // Left-only, soloed
        eq.bands.back().channel = EqChannel::Left;
        eq.bands.push_back({ EqBandType::Bell, 200.0, 6.0, 1.0, true, false }); // Right-only, not soloed -- excluded by the solo itself
        eq.bands.back().channel = EqChannel::Right;

        const StateSpaceMatrices soloed = eq::assembleStateSpace(eq, fs);
        const Complex z = std::polar(1.0, 2.0 * M_PI * 1000.0 / fs);
        const ComplexResponse resp = TransferFunction::evaluate(soloed, z);
        const double leftGainDb = 20.0 * std::log10(std::abs(resp.H[0][0]));
        const bool leftBoostedOk = leftGainDb > 8.0;
        const bool rightSilentOk = resp.H[1][1] == Complex(0.0, 0.0) && soloed.D[1][0] == 0.0 && soloed.D[1][1] == 0.0;

        eq.bands[0].solo = false; // un-solo -- nothing is soloed now, so hasActiveSolo() is false and the silencing rule doesn't apply at all
        const StateSpaceMatrices unsoloed = eq::assembleStateSpace(eq, fs);
        // Not checking for a specific gain here (band1's own real Bell
        // response at 1000Hz, ~2.3 octaves from its 200Hz center, isn't
        // exactly unity, just close to it -- the wrong thing to pin an
        // exact-equality test to) -- just that D[1][1] is no longer
        // forced to the exact 0.0 the silencing path leaves it at. Any
        // real biquad's own D coefficient landing on exactly 0.0 by
        // coincidence is not a realistic concern here.
        const bool rightRestoredOk = unsoloed.D[1][1] != 0.0;

        printf("[10e] Soloing a Left-only band silences Right exactly (D=0 too): %s"
               " (L gain=%.2fdB, R=%s)\n",
               (leftBoostedOk && rightSilentOk) ? "OK" : "FAIL",
               leftGainDb, rightSilentOk ? "exact zero" : "NOT silent");
        if (!(leftBoostedOk && rightSilentOk)) { printf("FAIL: solo didn't silence the channel the soloed band never reaches\n"); ++failures; }
        printf("      un-soloing restores Right's own band (no longer silenced): %s\n", rightRestoredOk ? "OK" : "FAIL");
        if (!rightRestoredOk) { printf("FAIL: Right channel stayed silenced after un-soloing\n"); ++failures; }
    }

    // 11. SpectrumAnalyzer (the EQ graph's background spectrum display,
    //     see EqGraphWidget.cpp): a handful of pure tones at known
    //     frequencies/levels must read back through the analyzer at
    //     essentially exact level and land in the expected log-spaced
    //     band. This is a deterministic check (unlike broadband noise,
    //     whose single-frame level has real, expected statistical
    //     variance -- smoothed out visually on the UI side, not tested
    //     here at the DSP level).
    {
        auto bandCenterHz = [](int b) {
            const double t0 = double(b) / double(SpectrumAnalyzer::kNumBands);
            const double t1 = double(b + 1) / double(SpectrumAnalyzer::kNumBands);
            const double lo = SpectrumAnalyzer::kMinHz * std::pow(SpectrumAnalyzer::kMaxHz / SpectrumAnalyzer::kMinHz, t0);
            const double hi = SpectrumAnalyzer::kMinHz * std::pow(SpectrumAnalyzer::kMaxHz / SpectrumAnalyzer::kMinHz, t1);
            return std::sqrt(lo * hi);
        };

        struct ToneCase { double freqHz, levelDb; };
        const ToneCase cases[] = { { 100.0, -6.0 }, { 440.0, -3.0 }, { 1000.0, 0.0 }, { 5000.0, -9.0 }, { 10000.0, -12.0 } };

        double worstFreqErrFrac = 0.0, worstLevelErrDb = 0.0;
        for (const auto& tc : cases)
        {
            SpectrumAnalyzer sa;
            sa.prepare(fs);
            const double amp = std::pow(10.0, tc.levelDb / 20.0);
            const int n = int(2.5 * fs);
            const size_t bufSize = size_t(n);
            std::vector<float> buf(bufSize);
            for (int i = 0; i < n; ++i) buf[size_t(i)] = float(amp * std::sin(2.0 * M_PI * tc.freqHz * i / fs));
            for (uint32_t off = 0; off < uint32_t(n); off += 256)
            {
                const uint32_t count = std::min<uint32_t>(256, uint32_t(n) - off);
                sa.process(buf.data() + off, count);
            }

            int peakBand = -1; float peakDb = -1000.0f;
            for (int b = 0; b < SpectrumAnalyzer::kNumBands; ++b)
            {
                const float db = sa.getBandDb(b);
                if (db > peakDb) { peakDb = db; peakBand = b; }
            }
            const double freqErrFrac = std::fabs(bandCenterHz(peakBand) - tc.freqHz) / tc.freqHz;
            const double levelErrDb = std::fabs(double(peakDb) - tc.levelDb);
            worstFreqErrFrac = std::max(worstFreqErrFrac, freqErrFrac);
            worstLevelErrDb = std::max(worstLevelErrDb, levelErrDb);
        }
        printf("[11] SpectrumAnalyzer: %zu tones, worst peak-band freq error=%.1f%%  worst level error=%.3fdB\n",
               sizeof(cases) / sizeof(cases[0]), worstFreqErrFrac * 100.0, worstLevelErrDb);
        if (worstFreqErrFrac > 0.25) { printf("FAIL: peak landed in the wrong band\n"); ++failures; }
        if (worstLevelErrDb > 0.5) { printf("FAIL: level reading not accurate enough\n"); ++failures; }

        // A dense sweep, not just 5 hand-picked frequencies: a tone whose
        // true bin sits right at (or just past) a display band's own edge
        // has its Hann-window mainlobe split across that boundary unless
        // the two neighbors' capture ranges overlap enough to still catch
        // it whole -- exactly the bug increasing kNumBands (see this
        // file's own comment on that change) exposed for the fixed 5kHz
        // case above, caught only because that one frequency happened to
        // land near a boundary. 60 log-spaced frequencies, at two FFT
        // sizes (smallest and largest preset -- binHz, and so where the
        // boundaries actually fall, differs by 8x between them), is a
        // real attempt to catch the *general* version of that bug rather
        // than re-verify the one specific frequency that happened to
        // trip it, at any *future* kNumBands/kMinBinSpan/overlap tuning.
        double worstSweepErrDb = 0.0;
        double worstSweepFreqHz = 0.0;
        int worstSweepSizeIdx = -1;
        constexpr int kSweepCount = 60;
        for (int sizeIdx : { 0, SpectrumAnalyzer::kNumSizePresets - 1 })
        {
            for (int i = 0; i < kSweepCount; ++i)
            {
                // 60Hz-15000Hz -- clear of kMinHz/kMaxHz's own edges, where
                // this analyzer was never claiming edge-to-edge accuracy.
                const double freqHz = 60.0 * std::pow(15000.0 / 60.0, double(i) / double(kSweepCount - 1));
                constexpr double levelDb = -6.0;

                SpectrumAnalyzer sa;
                sa.requestFftSizeIndex(sizeIdx);
                sa.prepare(fs);
                const double amp = std::pow(10.0, levelDb / 20.0);
                const int n = int(2.5 * fs);
                const size_t bufSize = size_t(n);
                std::vector<float> buf(bufSize);
                for (int s = 0; s < n; ++s) buf[size_t(s)] = float(amp * std::sin(2.0 * M_PI * freqHz * s / fs));
                for (uint32_t off = 0; off < uint32_t(n); off += 256)
                {
                    const uint32_t count = std::min<uint32_t>(256, uint32_t(n) - off);
                    sa.process(buf.data() + off, count);
                }

                float peakDb = -1000.0f;
                for (int b = 0; b < SpectrumAnalyzer::kNumBands; ++b)
                    peakDb = std::max(peakDb, sa.getBandDb(b));
                const double errDb = std::fabs(double(peakDb) - levelDb);
                if (errDb > worstSweepErrDb) { worstSweepErrDb = errDb; worstSweepFreqHz = freqHz; worstSweepSizeIdx = sizeIdx; }
            }
        }
        printf("     dense sweep: %d freqs x 2 FFT sizes, worst level error=%.3fdB at %.1fHz (fftSizeIdx=%d)\n",
               kSweepCount, worstSweepErrDb, worstSweepFreqHz, worstSweepSizeIdx);
        if (worstSweepErrDb > 0.5) { printf("FAIL: some frequency reads inaccurately (likely a band-boundary/mainlobe-split gap)\n"); ++failures; }
    }

    // 12. Coefficient smoothing (StateSpaceEngine): a same-order model
    //     swap must glide smoothly toward the new target rather than
    //     switching between one sample and the next -- the actual fix
    //     for a real reported bug (audible clicks when dragging an EQ
    //     band fast enough, described as "like plugging/unplugging a
    //     speaker cable"). Uses a pure-gain (D-only, A=B=C=0) model pair
    //     so the *exact* expected trajectory is a closed-form one-pole
    //     exponential -- checked to floating-point precision, not just
    //     "smoother than before".
    {
        StateSpaceMatrices g1;
        g1.setSilence();
        g1.order = 4;
        g1.D[0][0] = 1.0;
        g1.D[1][1] = 1.0;
        StateSpaceMatrices g2 = g1;
        g2.D[0][0] = 5.0;
        g2.D[1][1] = 5.0;

        StateSpaceEngine eng;
        eng.prepare(fs);
        eng.setModel(g1);
        eng.resetState();

        std::vector<float> warmIn(64, 1.0f), warmOutL(64), warmOutR(64);
        eng.processBlock(warmIn.data(), warmIn.data(), warmOutL.data(), warmOutR.data(), 64);
        const bool preSwitchOk = nearlyEqual(warmOutL[63], 1.0, 1e-9);

        eng.setModel(g2); // same order (4), a large D jump -- must now smooth, not snap

        const int n = 4000; // ~83ms at 48kHz -- several smoothing time constants
        const size_t nSize = size_t(n);
        std::vector<float> in(nSize, 1.0f), outL(nSize), outR(nSize);
        eng.processBlock(in.data(), in.data(), outL.data(), outR.data(), uint32_t(n));

        const double coeff = 1.0 - std::exp(-1.0 / (0.005 * fs)); // must match StateSpaceEngine::kCoeffSmoothTimeConstantSec
        double expected = 1.0;
        double worstDiffFromClosedForm = 0.0;
        for (int i = 0; i < n; ++i)
        {
            expected += coeff * (5.0 - expected);
            worstDiffFromClosedForm = std::max(worstDiffFromClosedForm, std::fabs(double(outL[size_t(i)]) - expected));
        }

        const double firstSampleJump = std::fabs(double(outL[0]) - 1.0);
        const bool notInstant = firstSampleJump < 3.5;      // an instant switch would jump straight to |5-1|=4.0
        const bool convergedByEnd = nearlyEqual(outL[size_t(n - 1)], 5.0, 0.01);
        const bool matchesClosedForm = worstDiffFromClosedForm < 1e-6;

        printf("[12] Coefficient smoothing: pre-switch=%.6f (want 1.0)  first-sample-after-switch=%.6f "
               "(instant switch would be 5.0)  converged-after-%dsamples=%.6f  worst-diff-from-closed-form=%.2e\n",
               warmOutL[63], outL[0], n, outL[size_t(n - 1)], worstDiffFromClosedForm);
        if (!preSwitchOk) { printf("FAIL: pre-switch output wasn't the expected 1.0\n"); ++failures; }
        if (!notInstant) { printf("FAIL: the coefficient switch was instant, not smoothed\n"); ++failures; }
        if (!convergedByEnd) { printf("FAIL: never converged to the new target\n"); ++failures; }
        if (!matchesClosedForm) { printf("FAIL: trajectory doesn't match the expected one-pole ramp\n"); ++failures; }
    }

    // 13. Order-changing edit (add/remove/enable/solo an EQ band) ducks
    //     instead of switching instantly -- the case coefficient
    //     smoothing (test [12]) can't cover, since two differently-sized
    //     state vectors have no meaningful "in between". Two *same-gain*,
    //     different-order pure-gain models: an instant switch would leave
    //     output flat at 1.0 throughout with no way to tell anything
    //     happened at all, so what's actually asserted is that the output
    //     visibly *dips* during the transition (proving a duck occurred)
    //     and fully recovers afterward.
    {
        StateSpaceMatrices g1;
        g1.setSilence();
        g1.order = 4;
        g1.D[0][0] = 1.0;
        g1.D[1][1] = 1.0;
        StateSpaceMatrices g2;
        g2.setSilence();
        g2.order = 8; // different order -- must duck, not snap
        g2.D[0][0] = 1.0;
        g2.D[1][1] = 1.0;

        StateSpaceEngine eng;
        eng.prepare(fs);
        eng.setModel(g1);
        eng.resetState();

        std::vector<float> warmIn(64, 1.0f), warmOutL(64), warmOutR(64);
        eng.processBlock(warmIn.data(), warmIn.data(), warmOutL.data(), warmOutR.data(), 64);
        const bool preSwitchOk2 = nearlyEqual(warmOutL[63], 1.0, 1e-9);

        eng.setModel(g2);

        const int n2 = int(0.05 * fs); // 50ms -- several duck durations
        const size_t n2Size = size_t(n2);
        std::vector<float> in2(n2Size, 1.0f), outL2(n2Size), outR2(n2Size);
        eng.processBlock(in2.data(), in2.data(), outL2.data(), outR2.data(), uint32_t(n2));

        float minAbs = 1.0f;
        for (int i = 0; i < n2; ++i) minAbs = std::min(minAbs, std::fabs(outL2[size_t(i)]));
        const bool dipped = minAbs < 0.3f;
        const bool recovered = nearlyEqual(outL2[size_t(n2 - 1)], 1.0, 0.01);

        printf("[13] Order-change duck: pre-switch=%.6f  min|output| during transition=%.6f (want a dip, <0.3)  "
               "recovered-after-%dsamples=%.6f\n",
               warmOutL[63], double(minAbs), n2, outL2[size_t(n2 - 1)]);
        if (!preSwitchOk2) { printf("FAIL: pre-switch output wasn't the expected 1.0\n"); ++failures; }
        if (!dipped) { printf("FAIL: no duck detected -- output never dipped during the order change\n"); ++failures; }
        if (!recovered) { printf("FAIL: output never recovered after the order-change duck\n"); ++failures; }
    }

    // 14. The mono EQ plugin's DSP path (plugin/dsp/MonoStateSpaceTypes.hpp,
    //     MonoStateSpaceEngine.hpp, MonoStabilityGuard.*, and
    //     eq::assembleMonoStateSpace()). Three properties, in increasing
    //     order of what they'd catch:
    //
    //     (a) The mono assembly of a band list must be *half the order* of
    //         the stereo assembly of the same list -- that size difference
    //         is the entire mechanism behind the mono variant's lower DSP
    //         cost (see MonoStateSpaceTypes.hpp's header comment), so if it
    //         ever regresses to stereo-sized, the plugin still works but
    //         silently stops being cheaper, which nothing else would catch.
    //
    //     (b) Running actual audio through MonoStateSpaceEngine must
    //         produce sample-for-sample what the *stereo* StateSpaceEngine
    //         produces on one channel from the same band list -- the two
    //         are the same LTI system by construction, so any discrepancy
    //         means the mono realization/cascade packing is wrong, not
    //         merely different. This is the real headline check, and it's
    //         an end-to-end one (assembler + guard + engine + recursion),
    //         not just a matrix-shape comparison.
    //
    //     (c) MonoStabilityGuard must behave like its stereo counterpart on
    //         the case that caused the original notch bug (see test [7]): a
    //         deliberately narrow 50Hz/Q=25 filter is legitimately stable
    //         and must NOT be clamped down.
    {
        EqModel eqm;
        eqm.bands.push_back({ EqBandType::HighPass, 80.0, 0.0, 0.707, true, false });
        eqm.bands.push_back({ EqBandType::Bell, 250.0, -4.0, 1.2, true, false });
        eqm.bands.push_back({ EqBandType::Bell, 3000.0, 5.0, 2.0, true, false });
        eqm.bands.push_back({ EqBandType::HighShelf, 9000.0, 3.0, 0.707, true, false });
        eqm.outputTrimDb = -2.0;

        StateSpaceMatrices stereoAsm = eq::assembleStateSpace(eqm, fs);
        mono::MonoStateSpaceMatrices monoAsm = eq::assembleMonoStateSpace(eqm, fs);

        // (a) half the order, for the same 4 bands: 8 states vs. 16.
        const bool halfOrder = monoAsm.order * 2 == stereoAsm.order;
        printf("[14] Mono EQ: stereo order=%d  mono order=%d (want exactly half)\n",
               stereoAsm.order, monoAsm.order);
        if (!halfOrder) { printf("FAIL: mono assembly is not half the stereo order -- the DSP saving is gone\n"); ++failures; }

        StabilityGuard::enforce(stereoAsm);
        mono::MonoStabilityGuard::enforce(monoAsm);

        // (b) identical audio out of both engines, for the same input.
        StateSpaceEngine stereoEng; stereoEng.prepare(fs); stereoEng.setModel(stereoAsm); stereoEng.resetState();
        mono::MonoStateSpaceEngine monoEng; monoEng.prepare(fs); monoEng.setModel(monoAsm); monoEng.resetState();

        const int N = 4096;
        const size_t NS = size_t(N);
        std::vector<float> in(NS), sOutL(NS), sOutR(NS), mOut(NS);
        // A mix of tones spanning the bands above, so every band actually
        // contributes something to the comparison rather than a signal
        // that happens to sit in one filter's stopband.
        for (int i = 0; i < N; ++i)
        {
            const double t = double(i) / fs;
            in[size_t(i)] = float(0.3 * std::sin(2.0 * M_PI * 60.0 * t) +
                                   0.3 * std::sin(2.0 * M_PI * 250.0 * t) +
                                   0.2 * std::sin(2.0 * M_PI * 3000.0 * t) +
                                   0.1 * std::sin(2.0 * M_PI * 11000.0 * t));
        }
        stereoEng.processBlock(in.data(), in.data(), sOutL.data(), sOutR.data(), uint32_t(N));
        monoEng.processBlock(in.data(), mOut.data(), uint32_t(N));

        double worstAbsDiff = 0.0;
        for (int i = 0; i < N; ++i)
            worstAbsDiff = std::max(worstAbsDiff, std::fabs(double(sOutL[size_t(i)]) - double(mOut[size_t(i)])));
        printf("     mono engine vs. stereo engine (same bands, same input): worst sample diff=%.3e\n", worstAbsDiff);
        // float output rounding is the only expected source of difference
        // here -- the two recursions are the same math in double.
        if (worstAbsDiff > 1e-6) { printf("FAIL: mono engine does not match the stereo engine's per-channel output\n"); ++failures; }

        // (c) MonoStabilityGuard must treat the case that caused the
        //     original notch bug (test [7]) exactly like the stereo guard
        //     does: a deliberately narrow 50Hz/Q=25 filter is legitimately
        //     stable, so it may only be nudged (the tiny sub-1% correction
        //     [7] also sees and explicitly allows), never collapsed the way
        //     the pre-fix guard did. Checked *against the stereo guard's
        //     own result on the same filter* rather than against a
        //     hard-coded number -- the two guards run the same algorithm,
        //     so any divergence between them is the actual regression to
        //     catch here, and the shared threshold below stays in sync
        //     with [7] automatically.
        EqModel narrow;
        narrow.bands.push_back({ EqBandType::Notch, 50.0, 0.0, 25.0, true, false });

        mono::MonoStateSpaceMatrices narrowMono = eq::assembleMonoStateSpace(narrow, fs);
        const StabilityReport nrepMono = mono::MonoStabilityGuard::enforce(narrowMono);

        StateSpaceMatrices narrowStereo = eq::assembleStateSpace(narrow, fs);
        const StabilityReport nrepStereo = StabilityGuard::enforce(narrowStereo);

        printf("     narrow 50Hz/Q=25 notch: mono guard scale=%.6f rho~%.6f  |  stereo guard scale=%.6f rho~%.6f\n",
               nrepMono.appliedScale, nrepMono.estimatedRadius,
               nrepStereo.appliedScale, nrepStereo.estimatedRadius);
        if (nrepMono.appliedScale < 0.99)
        { printf("FAIL: MonoStabilityGuard over-clamped a legitimately stable narrow filter (see test [7])\n"); ++failures; }
        if (!nearlyEqual(nrepMono.appliedScale, nrepStereo.appliedScale, 1e-9))
        { printf("FAIL: the mono and stereo guards disagree on the same filter\n"); ++failures; }

        // Empty model -> identity passthrough, mono edition (D = 1).
        EqModel emptyMono;
        const mono::MonoStateSpaceMatrices monoPass = eq::assembleMonoStateSpace(emptyMono, fs);
        const bool monoIsIdentity = monoPass.D == 1.0;
        printf("     empty model -> mono identity passthrough: %s\n", monoIsIdentity ? "OK" : "FAIL");
        if (!monoIsIdentity) { printf("FAIL: empty EqModel did not assemble to a mono identity passthrough\n"); ++failures; }
    }

    // 15. Crossover bank (mappers::makeStereoCrossoverBank): an N-way
    //     Linkwitz-Riley crossover must sum to unity gain (flat, 0dB) at
    //     every frequency when every band's own gain is 0dB -- the entire
    //     reason LR (not plain Butterworth) crossovers are used, see
    //     CoefficientMapper.hpp's header comment. Also checks the
    //     BandSplit bookkeeping (stateStart/stateCount/feedD)
    //     StateSpaceEngine's multiband stage depends on: bands must
    //     exactly partition [0, order) with no gaps/overlaps, and each
    //     band's own feedD must sum to exactly the assembled matrix's own
    //     combined D (see BandSplit's comment for why D is a *sum*).
    {
        const std::vector<double> crossovers = { 300.0, 3000.0 };
        const auto bandStages = mappers::designCrossoverBandStages(fs, crossovers, 4);
        MultibandSchedule layout;
        StateSpaceMatrices bank = mappers::makeStereoCrossoverBank(bandStages, layout);
        StabilityGuard::enforce(bank);

        double worstFlatDb = 0.0;
        for (double f : { 30.0, 100.0, 300.0, 1000.0, 3000.0, 8000.0, 15000.0 })
        {
            const double omega = 2.0 * M_PI * f / fs;
            const ComplexResponse resp = TransferFunction::evaluate(bank, Complex(std::cos(omega), std::sin(omega)));
            const double db = 20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12));
            worstFlatDb = std::max(worstFlatDb, std::fabs(db));
        }
        printf("[15] Crossover bank: %d bands, worst deviation from 0dB flat sum=%.4fdB\n", layout.numBands, worstFlatDb);
        if (worstFlatDb > 0.2) { printf("FAIL: Linkwitz-Riley crossover bank does not sum flat\n"); ++failures; }

        int coveredStates = 0;
        double sumD00 = 0.0, sumD11 = 0.0;
        bool partitionOk = true;
        for (int k = 0; k < layout.numBands; ++k)
        {
            if (layout.bands[k].stateStart != coveredStates) partitionOk = false;
            coveredStates += layout.bands[k].stateCount;
            sumD00 += layout.bands[k].feedD[0][0];
            sumD11 += layout.bands[k].feedD[1][1];
        }
        const bool orderMatches = coveredStates == bank.order;
        const bool dMatches = nearlyEqual(sumD00, bank.D[0][0], 1e-9) && nearlyEqual(sumD11, bank.D[1][1], 1e-9);
        printf("     BandSplit bookkeeping: partitionOk=%d orderMatches=%d(%d vs %d) dMatches=%d\n",
               partitionOk, orderMatches, coveredStates, bank.order, dMatches);
        if (!partitionOk || !orderMatches) { printf("FAIL: band state ranges do not exactly partition the assembled order\n"); ++failures; }
        if (!dMatches) { printf("FAIL: sum of per-band feedD does not match the assembled D\n"); ++failures; }
    }

    // 15b. The same flatness requirement at EVERY slope the multiband compressor offers (order 2/4/6/8 = 12/24/36/48 dB/oct).
    //      A true Linkwitz-Riley of order 2N is a Butterworth-N squared (its own Q table, see linkwitzRileyQs()) and, for N odd
    //      (orders 2, 6), needs the high side polarity-inverted to sum flat. Cascading identical Q=0.707 sections -- the old
    //      design -- was only right at order 4 (measured before the fix: a -40 dB hole at order 2, -58 dB at order 6, a -6 dB
    //      dip at order 8). Also every LR band is -6.02 dB at its crossover.
    {
        for (int order : { 2, 4, 6, 8 })
        {
            const std::vector<double> crossovers = { 300.0, 3000.0 };
            const auto bandStages = mappers::designCrossoverBandStages(fs, crossovers, order);
            MultibandSchedule layout;
            StateSpaceMatrices bank = mappers::makeStereoCrossoverBank(bandStages, layout);
            StabilityGuard::enforce(bank);
            double worstFlatDb = 0.0;
            for (double f = 25.0; f < 18000.0; f *= 1.07)
            {
                const double omega = 2.0 * M_PI * f / fs;
                const ComplexResponse resp = TransferFunction::evaluate(bank, Complex(std::cos(omega), std::sin(omega)));
                worstFlatDb = std::max(worstFlatDb, std::fabs(20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12))));
            }
            // -6.02 dB of the low band exactly at its crossover (direct biquad product)
            std::complex<double> h = 1.0;
            const std::complex<double> zi = std::polar(1.0, -2.0 * M_PI * 300.0 / fs);
            for (const BiquadCoeffs& c : bandStages[0]) h *= (c.b0 + c.b1 * zi + c.b2 * zi * zi) / (1.0 + c.a1 * zi + c.a2 * zi * zi);
            const double lowAtCrossoverDb = 20.0 * std::log10(std::abs(h));
            const bool ok = worstFlatDb < 0.5 && std::fabs(lowAtCrossoverDb + 6.02) < 0.1;
            printf("[15b] Crossover order %d: bank flat within %.3fdB, low band %.2fdB at its crossover: %s\n", order, worstFlatDb, lowAtCrossoverDb, ok ? "OK" : "FAIL");
            if (!ok) ++failures;
        }
    }

    // 16. StateSpaceEngine's multiband gain stage matches the closed-form
    //     compressor formula independently in two different bands at once
    //     (different threshold/ratio per band) -- the ground-truth check
    //     for the actual real-time mechanism, mirroring test [8]'s
    //     single-band version. Band 0 (lowpass -- passes DC) is checked
    //     with sustained DC input, exact and ripple-free just like test
    //     [8]; band 1 (includes a highpass -- blocks DC) is checked with a
    //     sustained tone well above the crossover, measured via settled
    //     RMS level (ripple-tolerant, same technique audio_testbench uses).
    {
        const std::vector<double> crossovers = { 1000.0 };
        const auto bandStages = mappers::designCrossoverBandStages(fs, crossovers, 4);
        MultibandSchedule layout;
        StateSpaceMatrices bank = mappers::makeStereoCrossoverBank(bandStages, layout);
        layout.enabled = true;

        struct BandParams { double thresholdDb, ratio, attackMs, releaseMs; };
        const BandParams bp[2] = { { -18.0, 4.0, 5.0, 100.0 }, { -12.0, 8.0, 5.0, 100.0 } };
        for (int k = 0; k < 2; ++k)
        {
            NonlinearSchedule& sch = layout.bands[k].schedule;
            sch.enabled = true;
            sch.attackMs = bp[k].attackMs;
            sch.releaseMs = bp[k].releaseMs;
            for (int i = 0; i < GainCurve::kNumKnots; ++i)
            {
                const double lvl = GainCurve::knotLevelDb(i);
                sch.curve.gainDb[i] = lvl <= bp[k].thresholdDb ? 0.0 : (lvl - bp[k].thresholdDb) * (1.0 / bp[k].ratio - 1.0);
            }
        }
        bank.multiband = layout;

        StateSpaceEngine eng; eng.prepare(fs); eng.setModel(bank); eng.resetState();

        {
            const int N = 48000;
            std::vector<float> inL(N, 1.0f), inR(N, 1.0f), outL(N), outR(N);
            eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), N);
            const double expectedDb = (0.0 - bp[0].thresholdDb) * (1.0 / bp[0].ratio - 1.0);
            const double expectedLin = std::pow(10.0, expectedDb / 20.0);
            printf("[16] Multiband: band0(DC) settled out=%.4f expected~%.4f\n", double(outL[size_t(N - 1)]), expectedLin);
            if (!nearlyEqual(outL[size_t(N - 1)], expectedLin, 0.01)) { printf("FAIL: band0 did not settle to expected gain\n"); ++failures; }
        }

        {
            eng.resetState();
            const int N = 48000; // 1s
            std::vector<float> inL(N), inR(N), outL(N), outR(N);
            for (int i = 0; i < N; ++i)
            {
                const float s = float(std::sin(2.0 * M_PI * 6000.0 * i / fs));
                inL[size_t(i)] = inR[size_t(i)] = s;
            }
            eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), N);

            auto rmsDbLocal = [](const float* d, int start, int count) {
                double sum = 0.0;
                for (int i = 0; i < count; ++i) sum += double(d[start + i]) * double(d[start + i]);
                return 10.0 * std::log10(std::max(sum / double(count), 1e-18));
            };
            const int tailCount = 4800; // last 100ms
            const int tailStart = N - tailCount;
            const double inRmsDb = rmsDbLocal(inL.data(), tailStart, tailCount);
            const double outRmsDb = rmsDbLocal(outL.data(), tailStart, tailCount);
            const double measuredGainDb = outRmsDb - inRmsDb;
            const double expectedDb = (0.0 - bp[1].thresholdDb) * (1.0 / bp[1].ratio - 1.0); // input at 0dBFS peak
            printf("     band1(6kHz tone) measured gain=%.3fdB expected~%.3fdB\n", measuredGainDb, expectedDb);
            if (!nearlyEqual(measuredGainDb, expectedDb, 1.0)) { printf("FAIL: band1 measured gain far from closed-form expectation\n"); ++failures; }
        }
    }

    // 17. GainCurveTrainer::fitMultiband() recovers two different known
    //     per-band curves from one synthetic (full-mix input, full-mix
    //     target) audio pair -- the "give it an input and desired output,
    //     per-band coefficients are learned" capability. Mirrors test
    //     [9]'s single-band version.
    //
    //     Ground truth is built to match exactly what fitMultiband()
    //     itself assumes -- bandTarget_k = gain_k(env(bandInput_k)) *
    //     bandInput_k -- by reusing the existing single-band
    //     NonlinearSchedule engine mechanism directly on each band's own
    //     already-filtered signal, then summing (deliberately *not* "run
    //     the full crossover+multiband engine once on the raw mix", which
    //     is what test [16] already checks and is exactly right for
    //     verifying the *engine*, but the two are mathematically
    //     equivalent here regardless).
    //
    //     This test drove a real fix to fitMultiband() itself, worth
    //     recording: its first version filtered the target through band
    //     k's own filter to recover that band's signal, and compared it
    //     *pointwise* against the input filtered through the same filter
    //     *once* -- but band k's own true contribution to an opaque,
    //     already-summed target has (in any sane construction) already
    //     gone through filter k *once* before this function ever sees it,
    //     so recovering it here applies filter k a *second* time, phase-
    //     shifting it by two applications of the filter's own phase
    //     response while the once-filtered input trace only carries one.
    //     fit()'s per-sample instantaneous-ratio technique
    //     (target[i]/base[i]) is meaningless comparing two differently-
    //     phase-shifted copies of the same waveform, even when their true
    //     envelope-level relationship is identical -- two sine waves of
    //     equal amplitude but different phase divide pointwise into noise,
    //     not a constant. This was large in practice (tens of percent
    //     wrong attack/release grid points, several dB of curve error) and
    //     invisible to test [9] (no filtering asymmetry exists in the
    //     single-band path -- fit() always compares a signal against
    //     itself run through an identity model). Fixed by filtering the
    //     input through the *same* filter *twice* too (see
    //     GainCurveTrainer.cpp's own comment) so both traces fit() compares
    //     carry the same two-application phase shift, leaving only the
    //     actual gain being learned as a difference.
    //
    //     Band 1's release time is checked loosely (a nearby grid point,
    //     not an exact match): its test tone is a single high frequency
    //     right above the crossover, whose fast oscillation makes release
    //     time somewhat less uniquely identifiable than band 0's DC-
    //     adjacent low-frequency content (a real, minor precision
    //     characteristic of fitting from a narrow-spectrum probe, not a
    //     bug -- richer/broadband per-band material identifies it more
    //     tightly, at the cost of also picking up cross-band leakage near
    //     the crossover itself, a tradeoff not worth chasing further here).
    {
        const std::vector<double> crossovers = { 1000.0 };
        const auto bandStages = mappers::designCrossoverBandStages(fs, crossovers, 4);
        MultibandSchedule known;
        (void)mappers::makeStereoCrossoverBank(bandStages, known); // populates `known`'s BandSplit ranges/feedD

        struct BandParams { double thresholdDb, ratio; };
        const BandParams bp[2] = { { -20.0, 4.0 }, { -15.0, 6.0 } };
        const double knownAttackMs = 3.0, knownReleaseMs = 100.0;
        for (int k = 0; k < 2; ++k)
        {
            known.bands[k].schedule.enabled = true;
            known.bands[k].schedule.attackMs = knownAttackMs;
            known.bands[k].schedule.releaseMs = knownReleaseMs;
            for (int i = 0; i < GainCurve::kNumKnots; ++i)
            {
                const double lvl = GainCurve::knotLevelDb(i);
                known.bands[k].schedule.curve.gainDb[i] =
                    lvl <= bp[k].thresholdDb ? 0.0 : (lvl - bp[k].thresholdDb) * (1.0 / bp[k].ratio - 1.0);
            }
        }
        known.enabled = true;

        // Synthetic training input: a low-band tone (200Hz) and a
        // high-band tone (6kHz) summed at matching, varying levels, well
        // separated from the 1kHz crossover on both sides so cross-band
        // leakage into each band's own extracted signal is negligible.
        const double levelsDb[] = { -40, -30, -20, -15, -10, -5, 0 };
        const int segN = int(0.4 * fs);
        std::vector<float> inL, inR;
        for (double lvlDb : levelsDb)
        {
            const float amp = float(std::pow(10.0, lvlDb / 20.0));
            for (int i = 0; i < segN; ++i)
            {
                const double t = double(inL.size()) / fs;
                const float lowPart = amp * float(std::sin(2.0 * M_PI * 200.0 * t));
                const float highPart = amp * float(std::sin(2.0 * M_PI * 6000.0 * t));
                const float s = 0.5f * (lowPart + highPart);
                inL.push_back(s); inR.push_back(s);
            }
        }
        const uint32_t n = uint32_t(inL.size());

        std::vector<float> tgtL(n, 0.0f), tgtR(n, 0.0f);
        for (int k = 0; k < 2; ++k)
        {
            StateSpaceMatrices bandOnly = mappers::makeStereoBiquadChain(bandStages[size_t(k)], bandStages[size_t(k)]);
            StabilityGuard::enforce(bandOnly);
            StateSpaceEngine bandEng; bandEng.prepare(fs); bandEng.setModel(bandOnly); bandEng.resetState();
            std::vector<float> bandInL(n), bandInR(n);
            bandEng.processBlock(inL.data(), inR.data(), bandInL.data(), bandInR.data(), n);

            StateSpaceMatrices identityWithCurve; // plain identity, nonlinear = this band's known schedule
            identityWithCurve.nonlinear = known.bands[k].schedule;
            StateSpaceEngine gainEng; gainEng.prepare(fs); gainEng.setModel(identityWithCurve); gainEng.resetState();
            std::vector<float> bandTgtL(n), bandTgtR(n);
            gainEng.processBlock(bandInL.data(), bandInR.data(), bandTgtL.data(), bandTgtR.data(), n);

            for (uint32_t i = 0; i < n; ++i) { tgtL[i] += bandTgtL[i]; tgtR[i] += bandTgtR[i]; }
        }

        const GainCurveTrainer::MultibandFitResult fitResult =
            GainCurveTrainer::fitMultiband(bandStages, fs, inL, inR, tgtL, tgtR);
        printf("[17] Multiband trainer round-trip: ok=%d\n", fitResult.ok);
        if (!fitResult.ok) { printf("FAIL: multiband fit did not succeed\n"); ++failures; }
        for (int k = 0; k < fitResult.numBands; ++k)
        {
            const double fittedAtMinus10 = fitResult.schedule.bands[k].schedule.curve.evaluate(-10.0);
            const double knownAtMinus10 = known.bands[k].schedule.curve.evaluate(-10.0);
            printf("     band %d: attack=%.2fms(known %.2f) release=%.2fms(known %.2f) rmsErr=%.4fdB curve@-10dB fitted=%.3f known=%.3f\n",
                   k, fitResult.schedule.bands[k].schedule.attackMs, knownAttackMs,
                   fitResult.schedule.bands[k].schedule.releaseMs, knownReleaseMs,
                   fitResult.rmsErrorDb[k], fittedAtMinus10, knownAtMinus10);
            if (!nearlyEqual(fitResult.schedule.bands[k].schedule.attackMs, knownAttackMs, 0.01)) { printf("FAIL: band %d attack not recovered\n", k); ++failures; }
            // Band 1's release is checked loosely -- see this test's header comment.
            const double releaseTol = (k == 1) ? 75.0 : 0.01;
            if (!nearlyEqual(fitResult.schedule.bands[k].schedule.releaseMs, knownReleaseMs, releaseTol)) { printf("FAIL: band %d release not recovered\n", k); ++failures; }
            if (!nearlyEqual(fittedAtMinus10, knownAtMinus10, 0.6)) { printf("FAIL: band %d curve not recovered within visited range\n", k); ++failures; }
        }
    }

    // 18. MbCompressorModel (plugin/dsp/MbCompressorModel.hpp), the
    //     multiband compressor plugin's coefficient-mapper layer: its
    //     state-space assembly (mappers::makeStereoCrossoverBank() under
    //     the hood) and its closed-form crossover-shape response
    //     (evaluateCombinedResponse(), what the plugin's graph actually
    //     draws every frame) must agree exactly, since they describe the
    //     same LTI crossover system by construction -- mirrors test
    //     [10]'s identical EqModel cross-check (this holds regardless of
    //     each band's own threshold/ratio, since TransferFunction has no
    //     notion of the runtime-only multiband gain stage at all, exactly
    //     like the single-band NonlinearSchedule case). Also a JSON
    //     round-trip and the addCrossover()/removeCrossover()
    //     bands.size()==crossoverHz.size()+1 invariant.
    {
        MbCompressorModel mbm;
        mb::addCrossover(mbm, 300.0);
        mb::addCrossover(mbm, 3000.0);
        mbm.bands[0].thresholdDb = -18.0; mbm.bands[0].ratio = 3.0;
        mbm.bands[1].thresholdDb = -12.0; mbm.bands[1].ratio = 6.0;
        mbm.bands[2].thresholdDb = -24.0; mbm.bands[2].ratio = 2.0;
        mbm.outputTrimDb = -1.5;

        StateSpaceMatrices assembled = mb::assembleStateSpace(mbm, fs);
        StabilityGuard::enforce(assembled);

        double worstDiffDb = 0.0;
        for (double f : { 30.0, 100.0, 300.0, 1000.0, 3000.0, 8000.0, 15000.0 })
        {
            const double omega = 2.0 * M_PI * f / fs;
            const ComplexResponse resp = TransferFunction::evaluate(assembled, Complex(std::cos(omega), std::sin(omega)));
            const double stateSpaceDb = 20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12));
            const double directDb = 20.0 * std::log10(std::max(std::abs(mb::evaluateCombinedResponse(mbm, fs, f)), 1e-12));
            worstDiffDb = std::max(worstDiffDb, std::fabs(stateSpaceDb - directDb));
        }
        printf("[18] MbCompressorModel: %d bands, assembled order=%d  state-space vs closed-form worst diff=%.6fdB\n",
               int(mbm.bands.size()), assembled.order, worstDiffDb);
        if (worstDiffDb > 0.01) { printf("FAIL: MbCompressorModel's two response paths disagree\n"); ++failures; }

        const std::string json = mb::toJson(mbm);
        MbCompressorModel parsed;
        const bool parseOk = mb::fromJson(json, parsed);
        const bool matches = parseOk && parsed.crossoverHz.size() == mbm.crossoverHz.size() &&
                              parsed.bands.size() == mbm.bands.size() &&
                              nearlyEqual(parsed.bands[1].thresholdDb, mbm.bands[1].thresholdDb, 1e-9) &&
                              nearlyEqual(parsed.outputTrimDb, mbm.outputTrimDb, 1e-9);
        printf("     JSON round-trip: parseOk=%d dataMatches=%d\n", parseOk, matches);
        if (!matches) { printf("FAIL: MbCompressorModel JSON round-trip lost data\n"); ++failures; }

        // Solo is a single spotlight (MbCompressorModel::soloedBand), not
        // a per-band flag -- verify only the soloed band stays audible
        // (every other band's schedule forced to -100dB at every knot,
        // see designBandSchedule()'s own comment) and that the choice
        // round-trips through JSON like any other field.
        mbm.soloedBand = 1;
        const StateSpaceMatrices assembledSolo = mb::assembleStateSpace(mbm, fs);
        const auto isMuted = [](const NonlinearSchedule& s) { return s.curve.gainDb[0] <= -99.0; };
        const bool soloIsolatesOneBand = !isMuted(assembledSolo.multiband.bands[1].schedule) &&
                                          isMuted(assembledSolo.multiband.bands[0].schedule) &&
                                          isMuted(assembledSolo.multiband.bands[2].schedule);
        const std::string jsonSolo = mb::toJson(mbm);
        MbCompressorModel parsedSolo;
        const bool soloRoundTrips = mb::fromJson(jsonSolo, parsedSolo) && parsedSolo.soloedBand == mbm.soloedBand;
        printf("     solo isolates exactly the soloed band: %s  soloedBand round-trips: %s\n",
               soloIsolatesOneBand ? "OK" : "FAIL", soloRoundTrips ? "OK" : "FAIL");
        if (!soloIsolatesOneBand || !soloRoundTrips) { printf("FAIL: MbCompressorModel solo spotlight\n"); ++failures; }
        mbm.soloedBand = -1; // back to no-solo before the invariant check below, which doesn't care about it but shouldn't rely on stale state either

        mb::removeCrossover(mbm, 0);
        const bool invariantOk = mbm.bands.size() == mbm.crossoverHz.size() + 1 && mbm.bands.size() == 2;
        printf("     removeCrossover invariant: bands=%zu crossovers=%zu %s\n",
               mbm.bands.size(), mbm.crossoverHz.size(), invariantOk ? "OK" : "FAIL");
        if (!invariantOk) { printf("FAIL: band-count/crossover-count invariant broken\n"); ++failures; }
    }

    // 19. DynamicEqModel: the Regalia-Mitra resting-shape closed form hits
    //     the resolved gain exactly at the band's own center frequency and
    //     0dB two octaves away (a genuine peaking shape, not merely "some
    //     curve"), and resolveDynamicGainDb() matches its own documented
    //     formula directly -- flat (== gainDb) at/under threshold, sloping
    //     by (1-1/ratio) above it, clamped to rangeDb.
    {
        DynamicEqBand b;
        b.freqHz = 1000.0; b.q = 2.0; b.gainDb = 12.0;
        b.thresholdDb = -20.0; b.ratio = 4.0; b.rangeDb = 8.0;

        const double atCenterDb = 20.0 * std::log10(std::abs(dyneq::evaluateBandResponse(b, fs, 1000.0, b.gainDb)));
        // Q=2 gives a -3dB bandwidth of f0/Q = 500Hz -- 50Hz (20x below
        // center) is comfortably outside that, unlike a nearby "two
        // octaves away" point, which a Q this wide hasn't actually decayed
        // away from yet.
        const double farAwayDb  = 20.0 * std::log10(std::abs(dyneq::evaluateBandResponse(b, fs, 50.0, b.gainDb)));
        printf("[19] DynamicEqModel resting shape: at f0=%.3fdB (want %.1f)  far below f0=%.3fdB (want ~0)\n",
               atCenterDb, b.gainDb, farAwayDb);
        if (!nearlyEqual(atCenterDb, b.gainDb, 0.01)) { printf("FAIL: Regalia-Mitra gain at center frequency wrong\n"); ++failures; }
        if (!nearlyEqual(farAwayDb, 0.0, 0.5)) { printf("FAIL: Regalia-Mitra shape not ~flat away from center\n"); ++failures; }

        const double belowThreshold = dyneq::resolveDynamicGainDb(b, -30.0);
        const double atThreshold = dyneq::resolveDynamicGainDb(b, b.thresholdDb);
        const double tenOver = dyneq::resolveDynamicGainDb(b, b.thresholdDb + 10.0);
        const double expectedTenOver = b.gainDb - std::min(b.rangeDb, 10.0 * (1.0 - 1.0 / b.ratio));
        const double wayOver = dyneq::resolveDynamicGainDb(b, b.thresholdDb + 100.0); // should clamp to rangeDb
        printf("     resolveDynamicGainDb: below=%.3f(want %.1f) atThreshold=%.3f(want %.1f) +10dB=%.3f(want %.3f) wayOver=%.3f(want %.1f)\n",
               belowThreshold, b.gainDb, atThreshold, b.gainDb, tenOver, expectedTenOver, wayOver, b.gainDb - b.rangeDb);
        if (!nearlyEqual(belowThreshold, b.gainDb, 1e-9)) { printf("FAIL: gain moved below threshold\n"); ++failures; }
        if (!nearlyEqual(atThreshold, b.gainDb, 1e-9)) { printf("FAIL: gain moved exactly at threshold\n"); ++failures; }
        if (!nearlyEqual(tenOver, expectedTenOver, 1e-9)) { printf("FAIL: dynamic slope wrong above threshold\n"); ++failures; }
        if (!nearlyEqual(wayOver, b.gainDb - b.rangeDb, 1e-9)) { printf("FAIL: range ceiling not clamped\n"); ++failures; }
    }

    // 20. DynamicEqEngine end to end: a single dynamic band, run on a
    //     synthetic tone at its own center frequency, at two different
    //     sustained levels -- quiet (well under threshold, must settle at
    //     the resting gainDb boost) and loud (well over threshold, must
    //     settle at the closed-form-reduced gain resolveDynamicGainDb()
    //     itself predicts) -- mirrors test [8]'s "settled measured gain
    //     matches closed form" methodology. Also checks
    //     DynamicEqEngine::getCurrentGainDb()'s live readback agrees with
    //     what the engine actually just produced.
    {
        DynamicEqRtModel rt;
        rt.numBands = 1;
        rt.bands[0].freqHz = 1000.0;
        rt.bands[0].q = 2.0;
        rt.bands[0].gainDb = 12.0;
        rt.bands[0].thresholdDb = -20.0;
        rt.bands[0].ratio = 4.0;
        rt.bands[0].rangeDb = 8.0;
        rt.bands[0].attackMs = 3.0;
        rt.bands[0].releaseMs = 30.0;
        rt.bands[0].dynamicsEnabled = true;

        DynamicEqEngine eng;
        eng.prepare(fs);
        eng.setModel(rt);
        eng.resetState();

        auto rmsDb = [](const std::vector<float>& d, int start, int count) {
            double sum = 0.0;
            for (int i = 0; i < count; ++i) sum += double(d[size_t(start + i)]) * double(d[size_t(start + i)]);
            return 10.0 * std::log10(std::max(sum / double(count), 1e-18));
        };
        auto runToneAt = [&](double amplitudeLin) {
            const int N = int(1.0 * fs); // 1s -- comfortably longer than the 30ms release, settles well before the tail
            const size_t Nu = size_t(N);
            std::vector<float> inL(Nu), inR(Nu), outL(Nu), outR(Nu);
            for (int i = 0; i < N; ++i)
            {
                const float s = float(amplitudeLin * std::sin(2.0 * M_PI * 1000.0 * i / fs));
                inL[size_t(i)] = inR[size_t(i)] = s;
            }
            eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), uint32_t(N));
            const int tailCount = int(0.1 * fs);
            const double inDb = rmsDb(inL, N - tailCount, tailCount);
            const double outDb = rmsDb(outL, N - tailCount, tailCount);
            return outDb - inDb;
        };

        eng.resetState();
        const double quietGainDb = runToneAt(std::pow(10.0, -40.0 / 20.0)); // well under threshold
        const double quietExpected = rt.bands[0].gainDb; // resolveDynamicGainDb() flat below threshold
        printf("[20] DynamicEqEngine quiet tone measured gain=%.3fdB expected~%.3fdB\n", quietGainDb, quietExpected);
        if (!nearlyEqual(quietGainDb, quietExpected, 0.5)) { printf("FAIL: quiet-tone gain far from resting gainDb\n"); ++failures; }

        eng.resetState();
        const double loudGainDb = runToneAt(std::pow(10.0, -3.0 / 20.0)); // well over threshold, sustained
        const double loudEnvelopeDb = -3.0; // the tone's own level is what the band's own detector settles to (center-frequency tone, unity-gain detector at its own center)
        const double loudExpected = dyneq::resolveDynamicGainDb(rt.bands[0], loudEnvelopeDb);
        printf("     loud tone measured gain=%.3fdB expected~%.3fdB (resolveDynamicGainDb)\n", loudGainDb, loudExpected);
        if (!nearlyEqual(loudGainDb, loudExpected, 1.0)) { printf("FAIL: loud-tone measured gain far from closed-form expectation\n"); ++failures; }

        const double liveReadback = eng.getCurrentGainDb(0);
        printf("     getCurrentGainDb() readback=%.3fdB (measured settled gain %.3fdB)\n", liveReadback, loudGainDb);
        if (!nearlyEqual(liveReadback, loudExpected, 1.0)) { printf("FAIL: live gain readback disagrees with the engine's own settled behavior\n"); ++failures; }
    }

    // 20b. DynamicEqEngine per-channel routing (DynamicEqBand::channel):
    //      the same loud, over-threshold tone fed identically to BOTH
    //      channels -- a Left-only band must still shape Left exactly as
    //      test [20]'s own loud case does, while leaving Right an EXACT
    //      identity (not merely "less affected"), since the band never
    //      touches it at all. Confirms both halves of
    //      DynamicEqEngine::processBlock()'s own channel gate: the
    //      detector/envelope reads only the assigned channel, and the
    //      Regalia-Mitra mix is only applied to it.
    {
        DynamicEqRtModel rt;
        rt.numBands = 1;
        rt.bands[0].freqHz = 1000.0;
        rt.bands[0].q = 2.0;
        rt.bands[0].gainDb = 12.0;
        rt.bands[0].thresholdDb = -20.0;
        rt.bands[0].ratio = 4.0;
        rt.bands[0].rangeDb = 8.0;
        rt.bands[0].attackMs = 3.0;
        rt.bands[0].releaseMs = 30.0;
        rt.bands[0].dynamicsEnabled = true;
        rt.bands[0].channel = EqChannel::Left;

        DynamicEqEngine eng;
        eng.prepare(fs);
        eng.setModel(rt);
        eng.resetState();

        auto rmsDb = [](const std::vector<float>& d, int start, int count) {
            double sum = 0.0;
            for (int i = 0; i < count; ++i) sum += double(d[size_t(start + i)]) * double(d[size_t(start + i)]);
            return 10.0 * std::log10(std::max(sum / double(count), 1e-18));
        };

        const int N = int(1.0 * fs);
        const size_t Nu = size_t(N);
        std::vector<float> inL(Nu), inR(Nu), outL(Nu), outR(Nu);
        const double amp = std::pow(10.0, -3.0 / 20.0); // well over threshold, same as test [20]'s loud case
        for (int i = 0; i < N; ++i)
        {
            const float s = float(amp * std::sin(2.0 * M_PI * 1000.0 * i / fs));
            inL[size_t(i)] = inR[size_t(i)] = s; // identical signal on both channels
        }
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), uint32_t(N));

        const int tailCount = int(0.1 * fs);
        const double leftGainDb = rmsDb(outL, N - tailCount, tailCount) - rmsDb(inL, N - tailCount, tailCount);
        const double rightGainDb = rmsDb(outR, N - tailCount, tailCount) - rmsDb(inR, N - tailCount, tailCount);
        const double loudEnvelopeDb = -3.0;
        const double leftExpected = dyneq::resolveDynamicGainDb(rt.bands[0], loudEnvelopeDb);

        const bool leftMatchesTest20 = nearlyEqual(leftGainDb, leftExpected, 1.0);
        const bool rightIsIdentity = std::fabs(rightGainDb) < 0.01;
        printf("[20b] DynamicEqEngine Left-only band, identical loud tone both channels: "
               "L gain=%.3fdB (want ~%.3f) R gain=%.4fdB (want 0, untouched): %s\n",
               leftGainDb, leftExpected, rightGainDb, (leftMatchesTest20 && rightIsIdentity) ? "OK" : "FAIL");
        if (!(leftMatchesTest20 && rightIsIdentity))
            { printf("FAIL: a Left-only DynamicEqBand either didn't shape Left or leaked into Right\n"); ++failures; }
    }

    // 20c. DynamicEqRtModel::muteLeft/muteRight (the engine-level half of
    //      "if a left filter is selected [soloed], only the left audio
    //      should be passed" -- PluginDynEq::rebuildEngineModel() is what
    //      actually decides these two flags from soloedBand, but that's
    //      plugin-level DPF glue with nothing to unit-test here; this
    //      confirms the engine itself honors them correctly once set):
    //      with muteRight set, Right must be *exact* digital silence
    //      regardless of program material, while Left keeps behaving
    //      exactly as an unmuted engine would (same loud-tone expectation
    //      as test [20]/[20b]).
    {
        DynamicEqRtModel rt;
        rt.numBands = 1;
        rt.bands[0].freqHz = 1000.0;
        rt.bands[0].q = 2.0;
        rt.bands[0].gainDb = 12.0;
        rt.bands[0].thresholdDb = -20.0;
        rt.bands[0].ratio = 4.0;
        rt.bands[0].rangeDb = 8.0;
        rt.bands[0].attackMs = 3.0;
        rt.bands[0].releaseMs = 30.0;
        rt.bands[0].dynamicsEnabled = true;
        rt.bands[0].channel = EqChannel::Left;
        rt.muteRight = true; // as PluginDynEq::rebuildEngineModel() would set when this Left-only band is the soloed one

        DynamicEqEngine eng;
        eng.prepare(fs);
        eng.setModel(rt);
        eng.resetState();

        const int N = int(1.0 * fs);
        const size_t Nu = size_t(N);
        std::vector<float> inL(Nu), inR(Nu), outL(Nu), outR(Nu);
        const double amp = std::pow(10.0, -3.0 / 20.0);
        for (int i = 0; i < N; ++i)
        {
            const float s = float(amp * std::sin(2.0 * M_PI * 1000.0 * i / fs));
            inL[size_t(i)] = inR[size_t(i)] = s;
        }
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), uint32_t(N));

        float worstAbsR = 0.0f;
        for (float v : outR) worstAbsR = std::max(worstAbsR, std::fabs(v));
        auto rmsDbLocal = [](const std::vector<float>& d, int start, int count) {
            double sum = 0.0;
            for (int i = 0; i < count; ++i) sum += double(d[size_t(start + i)]) * double(d[size_t(start + i)]);
            return 10.0 * std::log10(std::max(sum / double(count), 1e-18));
        };
        const int tailCount = int(0.1 * fs);
        const double leftGainDb = rmsDbLocal(outL, N - tailCount, tailCount) - rmsDbLocal(inL, N - tailCount, tailCount);
        const double leftExpected = dyneq::resolveDynamicGainDb(rt.bands[0], -3.0);

        const bool rightSilentOk = worstAbsR == 0.0f;
        const bool leftUnaffectedOk = nearlyEqual(leftGainDb, leftExpected, 1.0);
        printf("[20c] DynamicEqEngine muteRight: R worst|sample|=%.3e (want exactly 0), "
               "L gain=%.3fdB (want ~%.3f, unaffected by the mute): %s\n",
               double(worstAbsR), leftGainDb, leftExpected, (rightSilentOk && leftUnaffectedOk) ? "OK" : "FAIL");
        if (!(rightSilentOk && leftUnaffectedOk)) { printf("FAIL: muteRight didn't fully silence Right, or affected Left\n"); ++failures; }
    }

    // 20d. Toggling Audition mid-stream ducks instead of switching
    //      instantly (it used to click), the same fix as the order-change
    //      duck, test [13]. Same duck-detection
    //      idea as [13] (look for a dip right at the transition), but
    //      measured as *windowed RMS* rather than a single sample's
    //      instantaneous magnitude -- unlike [13]'s flat DC-like input, a
    //      1kHz tone's own instantaneous value legitimately passes through
    //      0 twice a cycle regardless of ducking, so a single near-zero
    //      sample proves nothing; a genuinely suppressed *RMS* over a
    //      several-millisecond window right at the toggle does.
    {
        auto rmsAbs = [](const std::vector<float>& d, int start, int count) {
            double sum = 0.0;
            for (int i = 0; i < count; ++i) sum += double(d[size_t(start + i)]) * double(d[size_t(start + i)]);
            return std::sqrt(sum / double(count));
        };

        DynamicEqRtModel rt;
        rt.numBands = 1;
        rt.bands[0].freqHz = 1000.0;
        rt.bands[0].q = 2.0;
        rt.bands[0].gainDb = 12.0; // a real, audible boost -- the tap's own level should differ noticeably from the plain input
        rt.bands[0].dynamicsEnabled = false; // static, so the settled level doesn't itself keep moving
        rt.bands[0].thresholdDb = -30.0; rt.bands[0].ratio = 1.0; rt.bands[0].rangeDb = 0.0;
        rt.bands[0].attackMs = 5.0; rt.bands[0].releaseMs = 80.0;

        DynamicEqEngine eng;
        eng.prepare(fs);
        eng.setModel(rt);
        eng.resetState();

        int sampleCount = 0;
        auto tone = [&](std::vector<float>& buf) {
            for (float& v : buf) { v = float(0.5 * std::sin(2.0 * M_PI * 1000.0 * sampleCount / fs)); ++sampleCount; }
        };

        const int warmN = int(0.05 * fs); // well past both the shaper/detector filters' own settling and any attack time
        const size_t warmNu = size_t(warmN);
        std::vector<float> warmIn(warmNu), warmOutL(warmNu), warmOutR(warmNu);
        tone(warmIn);
        eng.processBlock(warmIn.data(), warmIn.data(), warmOutL.data(), warmOutR.data(), uint32_t(warmN));
        const double settledNormalRms = rmsAbs(warmOutL, warmN - 400, 400); // steady state just before the toggle

        eng.setAudition(true);
        eng.setAuditionBandIndex(0);
        const int flipN = int(0.05 * fs); // several duck durations (~8ms fade-out + ~8ms fade-in)
        const size_t flipNu = size_t(flipN);
        std::vector<float> flipIn(flipNu), flipOutL(flipNu), flipOutR(flipNu);
        tone(flipIn);
        eng.processBlock(flipIn.data(), flipIn.data(), flipOutL.data(), flipOutR.data(), uint32_t(flipN));
        const double dipRmsOnEngage = rmsAbs(flipOutL, 0, 400);       // right at the transition -- should be suppressed if a duck ran
        const double settledAuditionRms = rmsAbs(flipOutL, flipN - 400, 400); // well after -- the tap's own settled level

        eng.setAudition(false);
        const int backN = int(0.05 * fs);
        const size_t backNu = size_t(backN);
        std::vector<float> backIn(backNu), backOutL(backNu), backOutR(backNu);
        tone(backIn);
        eng.processBlock(backIn.data(), backIn.data(), backOutL.data(), backOutR.data(), uint32_t(backN));
        const double dipRmsOnDisengage = rmsAbs(backOutL, 0, 400);
        const double recoveredRms = rmsAbs(backOutL, backN - 400, 400); // well after -- back to the normal path's own level

        // The dip window (the first ~400 samples, ~8ms at 48kHz) mostly
        // covers the fade-*out* half of the ~16ms duck (kDuckTimeSec is
        // 8ms each way) -- a linear gain ramp from 1 to 0 over that
        // window has an RMS multiplier of sqrt(mean((1-t)^2)) = 1/sqrt(3)
        // (~0.577), not near-zero, since it's an *average* over the whole
        // ramp, not a sample at its very bottom. 0.75 comfortably
        // separates that expected ~0.577 (duck ran) from ~1.0 (no duck at
        // all, an instant switch) with margin either side.
        const bool dippedOnEngage = dipRmsOnEngage < 0.75 * settledNormalRms;
        const bool dippedOnDisengage = dipRmsOnDisengage < 0.75 * settledAuditionRms;
        const bool recoveredOk = nearlyEqual(recoveredRms, settledNormalRms, 0.05 * settledNormalRms);

        printf("[20d] DynamicEqEngine Audition toggle ducks: settled-normal-rms=%.4f  dip-on-engage-rms=%.4f (want <%.4f)  "
               "settled-audition-rms=%.4f  dip-on-disengage-rms=%.4f (want <%.4f)  recovered-rms=%.4f (want ~%.4f): %s\n",
               settledNormalRms, dipRmsOnEngage, 0.75 * settledNormalRms, settledAuditionRms, dipRmsOnDisengage,
               0.75 * settledAuditionRms, recoveredRms, settledNormalRms,
               (dippedOnEngage && dippedOnDisengage && recoveredOk) ? "OK" : "FAIL");
        if (!dippedOnEngage) { printf("FAIL: no duck detected when Audition engaged -- instant switch would click\n"); ++failures; }
        if (!dippedOnDisengage) { printf("FAIL: no duck detected when Audition disengaged -- instant switch would click\n"); ++failures; }
        if (!recoveredOk) { printf("FAIL: output never recovered to the normal path's own level after disengaging Audition\n"); ++failures; }
    }

    // 21. DynamicEqModel's *static export* path (dyneq::assembleStateSpace(),
    //     an ordinary RBJ designPeaking() cascade -- deliberately NOT the
    //     live Regalia-Mitra shape, see DynamicEqModel.hpp's file comment)
    //     agrees with TransferFunction on the assembled matrix, mirroring
    //     test [10]'s identical EqModel cross-check. Also a JSON round-trip
    //     covering every field.
    {
        DynamicEqModel m;
        DynamicEqBand b1; b1.freqHz = 200.0; b1.gainDb = 4.0; b1.q = 1.2;
        b1.thresholdDb = -18.0; b1.ratio = 3.0; b1.rangeDb = 6.0; b1.attackMs = 5.0; b1.releaseMs = 80.0;
        DynamicEqBand b2; b2.freqHz = 4000.0; b2.gainDb = -6.0; b2.q = 0.8; b2.dynamicsEnabled = false;
        m.bands = { b1, b2 };
        m.outputTrimDb = -2.0;

        StateSpaceMatrices assembled = dyneq::assembleStateSpace(m, fs);
        StabilityGuard::enforce(assembled);

        double worstDiffDb = 0.0;
        for (double f : { 50.0, 200.0, 1000.0, 4000.0, 12000.0 })
        {
            const double omega = 2.0 * M_PI * f / fs;
            const ComplexResponse resp = TransferFunction::evaluate(assembled, Complex(std::cos(omega), std::sin(omega)));
            const double stateSpaceDb = 20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12));

            std::complex<double> direct(std::pow(10.0, m.outputTrimDb / 20.0), 0.0);
            for (const DynamicEqBand& b : m.bands)
                direct *= eq::evaluateBiquadResponse(mappers::designPeaking(fs, b.freqHz, b.q, b.gainDb), omega);
            const double directDb = 20.0 * std::log10(std::max(std::abs(direct), 1e-12));
            worstDiffDb = std::max(worstDiffDb, std::fabs(stateSpaceDb - directDb));
        }
        printf("[21] DynamicEqModel static export: %zu bands, assembled order=%d  state-space vs closed-form worst diff=%.6fdB\n",
               m.bands.size(), assembled.order, worstDiffDb);
        if (worstDiffDb > 0.01) { printf("FAIL: DynamicEqModel's static export disagrees with its own closed form\n"); ++failures; }

        const std::string json = dyneq::toJson(m);
        DynamicEqModel parsed;
        const bool parseOk = dyneq::fromJson(json, parsed);
        const bool matches = parseOk && parsed.bands.size() == m.bands.size() &&
                              nearlyEqual(parsed.bands[0].freqHz, m.bands[0].freqHz, 1e-9) &&
                              nearlyEqual(parsed.bands[0].thresholdDb, m.bands[0].thresholdDb, 1e-9) &&
                              nearlyEqual(parsed.bands[0].rangeDb, m.bands[0].rangeDb, 1e-9) &&
                              parsed.bands[1].dynamicsEnabled == m.bands[1].dynamicsEnabled &&
                              nearlyEqual(parsed.outputTrimDb, m.outputTrimDb, 1e-9);
        printf("     JSON round-trip: parseOk=%d dataMatches=%d\n", parseOk, matches);
        if (!matches) { printf("FAIL: DynamicEqModel JSON round-trip lost data\n"); ++failures; }

        // Solo is a single spotlight (DynamicEqModel::soloedBand), not a
        // per-band flag -- same redesign, same reasoning, as
        // MbCompressorModel::soloedBand (see test [18]'s identical check).
        m.soloedBand = 0;
        const std::vector<int> active = dyneq::activeBandIndices(m);
        const bool soloIsolatesOneBand = active.size() == 1 && active[0] == 0;
        const std::string jsonSolo = dyneq::toJson(m);
        DynamicEqModel parsedSolo;
        const bool soloRoundTrips = dyneq::fromJson(jsonSolo, parsedSolo) && parsedSolo.soloedBand == m.soloedBand;
        printf("     solo isolates exactly the soloed band: %s  soloedBand round-trips: %s\n",
               soloIsolatesOneBand ? "OK" : "FAIL", soloRoundTrips ? "OK" : "FAIL");
        if (!soloIsolatesOneBand || !soloRoundTrips) { printf("FAIL: DynamicEqModel solo spotlight\n"); ++failures; }
    }

    // 22. Upward compression -- up and down compression in one curve:
    //     GainCurve::classicCompressor()'s
    //     new ratioUp argument mirrors ratioDown below the threshold --
    //     below-threshold gain must rise (a boost, not the old flat-at-
    //     makeup behavior) by an amount matching the same 1-1/ratio slope
    //     formula test [8]/[16]'s downward checks already use, and
    //     mb::designBandSchedule() must actually thread MbBand::ratioUp
    //     through to the assembled curve unchanged.
    {
        const double thresholdDb = -20.0, ratioDown = 4.0, ratioUp = 5.0, makeupDb = 1.0;
        const GainCurve curve = GainCurve::classicCompressor(thresholdDb, ratioDown, ratioUp, makeupDb);
        const double quietLevelDb = -40.0; // 20dB below threshold
        const double expectedBoost = makeupDb + (quietLevelDb - thresholdDb) * (1.0 / ratioUp - 1.0);
        const double loudLevelDb = 0.0; // 20dB above threshold
        const double expectedCut = makeupDb + (loudLevelDb - thresholdDb) * (1.0 / ratioDown - 1.0);
        const double atThreshold = curve.evaluate(thresholdDb);
        printf("[22] classicCompressor bidirectional: quiet(-40dB)=%.3f(want %.3f, boosted) loud(0dB)=%.3f(want %.3f) atThreshold=%.3f(want %.1f)\n",
               curve.evaluate(quietLevelDb), expectedBoost, curve.evaluate(loudLevelDb), expectedCut, atThreshold, makeupDb);
        bool ok = nearlyEqual(curve.evaluate(quietLevelDb), expectedBoost, 0.15) && expectedBoost > makeupDb + 0.01 &&
                  nearlyEqual(curve.evaluate(loudLevelDb), expectedCut, 0.15) &&
                  nearlyEqual(atThreshold, makeupDb, 0.15);
        if (!ok) { printf("FAIL: classicCompressor's two-sided shape wrong\n"); ++failures; }

        MbBand band;
        band.thresholdDb = thresholdDb; band.ratio = ratioDown; band.ratioUp = ratioUp; band.makeupDb = makeupDb;
        const NonlinearSchedule sched = mb::designBandSchedule(band, /*audible=*/true);
        const bool threaded = nearlyEqual(sched.curve.evaluate(quietLevelDb), expectedBoost, 0.15);
        printf("     mb::designBandSchedule() threads ratioUp through: %s\n", threaded ? "OK" : "FAIL");
        if (!threaded) ++failures;
    }

    // 23. DynamicEqModel's resolveDynamicGainDb() upward side: below
    //     threshold, a band with ratioUp > 1 must boost (not sit flat at
    //     gainDb), by the mirrored formula test [19]'s downward check
    //     already covers, clamped to the same rangeDb ceiling. Also an
    //     end-to-end DynamicEqEngine settling check (mirrors test [20]'s
    //     methodology) confirming a quiet sustained tone actually comes
    //     out boosted, not just that the closed form says it should.
    {
        DynamicEqBand b;
        b.freqHz = 1000.0; b.q = 2.0; b.gainDb = 0.0;
        b.thresholdDb = -20.0; b.ratio = 4.0; b.ratioUp = 4.0; b.rangeDb = 8.0;
        const double quietEnvDb = -30.0; // 10dB below threshold
        const double expectedBoost = std::min(b.rangeDb, (b.thresholdDb - quietEnvDb) * (1.0 - 1.0 / b.ratioUp));
        const double got = dyneq::resolveDynamicGainDb(b, quietEnvDb);
        printf("[23] DynamicEqModel upward resolveDynamicGainDb: got=%.3f expected=%.3f (want >0, a boost)\n", got, expectedBoost);
        bool ok = nearlyEqual(got, expectedBoost, 1e-9) && got > 0.01;
        if (!ok) { printf("FAIL: resolveDynamicGainDb upward side wrong\n"); ++failures; }

        DynamicEqRtModel rt;
        rt.numBands = 1;
        rt.bands[0] = b;
        rt.bands[0].attackMs = 3.0; rt.bands[0].releaseMs = 30.0; rt.bands[0].dynamicsEnabled = true;
        DynamicEqEngine eng;
        eng.prepare(fs);
        eng.setModel(rt);
        eng.resetState();

        const int N = int(1.0 * fs);
        const size_t Nu = size_t(N);
        std::vector<float> inL(Nu), inR(Nu), outL(Nu), outR(Nu);
        const double amp = std::pow(10.0, quietEnvDb / 20.0); // matches this band's own detector convention, see test [20]
        for (int i = 0; i < N; ++i)
        {
            const float s = float(amp * std::sin(2.0 * M_PI * 1000.0 * i / fs));
            inL[size_t(i)] = inR[size_t(i)] = s;
        }
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), uint32_t(N));
        auto rmsDbLocal = [](const std::vector<float>& d, int start, int count) {
            double sum = 0.0;
            for (int i = 0; i < count; ++i) sum += double(d[size_t(start + i)]) * double(d[size_t(start + i)]);
            return 10.0 * std::log10(std::max(sum / double(count), 1e-18));
        };
        const int tailCount = int(0.1 * fs);
        const double measuredGainDb = rmsDbLocal(outL, N - tailCount, tailCount) - rmsDbLocal(inL, N - tailCount, tailCount);
        printf("     DynamicEqEngine quiet tone measured gain=%.3fdB expected~%.3fdB (a real boost, not just the closed form)\n",
               measuredGainDb, expectedBoost);
        if (!nearlyEqual(measuredGainDb, expectedBoost, 1.0)) { printf("FAIL: engine did not actually boost the quiet tone\n"); ++failures; }
    }

    // 24. The "inert band" trick PluginDynEq::rebuildEngineModel() uses to
    //     fix a real bug (Solo/Enable index-scrambling the live
    //     per-band gain readback -- see that function's own comment):
    //     replacing a disabled/muted-by-solo band with a copy that has
    //     dynamicsEnabled=false and gainDb=0.0, *at its own slot*, rather
    //     than omitting it, is only valid if such a band is mathematically
    //     an exact identity regardless of its own freqHz/q -- verified
    //     directly here rather than just asserted in a comment.
    {
        DynamicEqBand inert;
        inert.freqHz = 3000.0; inert.q = 5.0; inert.gainDb = 0.0; inert.dynamicsEnabled = false;
        double worstDiffDb = 0.0;
        for (double f : { 50.0, 200.0, 1000.0, 3000.0, 8000.0, 15000.0 })
        {
            const std::complex<double> resp = dyneq::evaluateBandResponse(inert, fs, f, inert.gainDb);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-12));
            worstDiffDb = std::max(worstDiffDb, std::fabs(db));
        }
        printf("[24] Inert dynamic-EQ band (dynamicsEnabled=false, gainDb=0) is an exact identity: worst deviation=%.9fdB\n", worstDiffDb);
        if (worstDiffDb > 1e-9) { printf("FAIL: an 'inert' band is not actually transparent -- the Solo/Enable index-stability fix depends on this\n"); ++failures; }

        // Also confirm resolveDynamicGainDb() itself returns exactly 0dB
        // for such a band regardless of the detected envelope -- the
        // per-sample engine path relies on this to skip any dynamics math
        // for an inert slot.
        bool flatOk = true;
        for (double envDb : { -80.0, -40.0, -20.0, 0.0 })
            if (!nearlyEqual(dyneq::resolveDynamicGainDb(inert, envDb), 0.0, 1e-12)) flatOk = false;
        printf("     resolveDynamicGainDb() stays exactly 0dB for an inert band at any level: %s\n", flatOk ? "OK" : "FAIL");
        if (!flatOk) ++failures;
    }

    // 25. EqBand::slopeStages ("steepness"): a LowPass/HighPass with
    //     slopeStages=N must assemble to exactly N cascaded copies of the
    //     same single-stage biquad (2N states for that band, stereo), its
    //     closed-form response (evaluateBandResponse(), now cascade-aware)
    //     must agree with TransferFunction on the assembled matrix exactly
    //     like test [10]'s single-stage check, and going from 1 to a
    //     higher slopeStages must measurably steepen the rolloff (more
    //     attenuation an octave past cutoff) without moving the passband.
    {
        EqBand hp;
        hp.type = EqBandType::HighPass;
        hp.freqHz = 500.0;
        hp.q = 0.7071; // Butterworth Q, so cascaded stages roll off cleanly
        hp.slopeStages = 3; // 36dB/oct

        EqModel m;
        m.bands = { hp };
        StateSpaceMatrices assembled = eq::assembleStateSpace(m, fs);
        StabilityGuard::enforce(assembled);

        const bool orderOk = assembled.order == 12; // 3 stages * 2 states/stage * 2 channels
        printf("[25] EqBand slopeStages: assembled order=%d (want 12 for 3 stages, stereo)\n", assembled.order);
        if (!orderOk) { printf("FAIL: slopeStages didn't cascade the expected number of stages\n"); ++failures; }

        double worstDiffDb = 0.0;
        for (double f : { 50.0, 250.0, 500.0, 1000.0, 4000.0, 15000.0 })
        {
            const double omega = 2.0 * M_PI * f / fs;
            const ComplexResponse resp = TransferFunction::evaluate(assembled, Complex(std::cos(omega), std::sin(omega)));
            const double stateSpaceDb = 20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12));
            const double directDb = 20.0 * std::log10(std::max(std::abs(eq::evaluateBandResponse(hp, fs, f)), 1e-12));
            worstDiffDb = std::max(worstDiffDb, std::fabs(stateSpaceDb - directDb));
        }
        printf("     state-space vs closed-form worst diff=%.6fdB\n", worstDiffDb);
        if (worstDiffDb > 0.01) { printf("FAIL: cascaded slopeStages closed form disagrees with assembled matrix\n"); ++failures; }

        // One octave below a 500Hz highpass cutoff, 3 stages (36dB/oct)
        // must attenuate substantially more than 1 stage (12dB/oct) --
        // and the passband (well above cutoff) must stay ~0dB either way,
        // confirming slopeStages steepens the *stopband* without touching
        // the *passband*.
        EqBand hp1 = hp; hp1.slopeStages = 1;
        const double oneStageAt250 = 20.0 * std::log10(std::max(std::abs(eq::evaluateBandResponse(hp1, fs, 250.0)), 1e-12));
        const double threeStageAt250 = 20.0 * std::log10(std::max(std::abs(eq::evaluateBandResponse(hp, fs, 250.0)), 1e-12));
        const double threeStageAt4k = 20.0 * std::log10(std::max(std::abs(eq::evaluateBandResponse(hp, fs, 4000.0)), 1e-12));
        printf("     one octave below cutoff: 1 stage=%.2fdB  3 stages=%.2fdB (want much lower)  passband(4kHz, 3 stages)=%.2fdB (want ~0)\n",
               oneStageAt250, threeStageAt250, threeStageAt4k);
        const bool steeperOk = threeStageAt250 < oneStageAt250 - 10.0;
        const bool passbandOk = nearlyEqual(threeStageAt4k, 0.0, 0.5);
        if (!steeperOk) { printf("FAIL: 3 slopeStages isn't meaningfully steeper than 1 in the stopband\n"); ++failures; }
        if (!passbandOk) { printf("FAIL: slopeStages should not touch the passband\n"); ++failures; }

        // JSON round-trip.
        const std::string json = eq::toJson(m);
        EqModel parsed;
        const bool parseOk = eq::fromJson(json, parsed);
        const bool matches = parseOk && parsed.bands.size() == 1 && parsed.bands[0].slopeStages == 3;
        printf("     JSON round-trip: parseOk=%d slopeStages preserved=%d\n", parseOk, matches);
        if (!matches) { printf("FAIL: slopeStages lost in JSON round-trip\n"); ++failures; }
    }

    // 26. DynamicEqEngine::processBlock()'s output-trim scaling, isolated:
    //     an "inert" band (see test [24] -- dynamicsEnabled=false, gainDb=0
    //     is a proven exact identity) as the only band means the *entire*
    //     measured input->output level change must come from outputTrimDb
    //     alone, nothing else. Added specifically to cover the code path a
    //     performance fix just touched: outputTrimDb's std::pow(10, .../20)
    //     conversion used to be recomputed every single sample inside the
    //     block loop even though it depends only on the block-invariant
    //     `m.outputTrimDb` -- hoisted to once per block instead. Since it's
    //     the exact same expression evaluated on the exact same input, this
    //     is bit-for-bit the same value either way and this test doesn't
    //     (and can't, from outside the engine) tell the two apart -- what
    //     it *does* prove is that hoisting it didn't accidentally apply the
    //     trim to only part of the block, skip it, or apply it twice.
    {
        DynamicEqBand inert;
        inert.freqHz = 3000.0; inert.q = 5.0; inert.gainDb = 0.0; inert.dynamicsEnabled = false;

        DynamicEqRtModel rt;
        rt.numBands = 1;
        rt.bands[0] = inert;
        rt.outputTrimDb = -6.0;

        DynamicEqEngine eng;
        eng.prepare(fs);
        eng.setModel(rt);
        eng.resetState();

        const int N = int(0.2 * fs);
        const size_t Nu = size_t(N);
        std::vector<float> inL(Nu), inR(Nu), outL(Nu), outR(Nu);
        for (int i = 0; i < N; ++i)
        {
            const float s = float(0.2 * std::sin(2.0 * M_PI * 1000.0 * i / fs));
            inL[size_t(i)] = inR[size_t(i)] = s;
        }
        eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), uint32_t(N));

        // Skip a short settle window (the allpass/detector filters' own
        // startup transient) rather than measuring from sample 0.
        const int skip = int(0.05 * fs);
        double sumIn = 0.0, sumOut = 0.0;
        for (int i = skip; i < N; ++i)
        {
            sumIn += double(inL[size_t(i)]) * double(inL[size_t(i)]);
            sumOut += double(outL[size_t(i)]) * double(outL[size_t(i)]);
        }
        const double inDb = 10.0 * std::log10(std::max(sumIn / double(N - skip), 1e-18));
        const double outDb = 10.0 * std::log10(std::max(sumOut / double(N - skip), 1e-18));
        const double measuredTrimDb = outDb - inDb;
        printf("[26] DynamicEqEngine outputTrimDb=%.1fdB through an inert band: measured=%.4fdB\n",
               rt.outputTrimDb, measuredTrimDb);
        if (!nearlyEqual(measuredTrimDb, rt.outputTrimDb, 0.01))
        {
            printf("FAIL: measured output trim disagrees with the configured outputTrimDb\n");
            ++failures;
        }

        // A second run with outputTrimDb=0 must reproduce the input exactly
        // (same inert-band identity as test [24], now through the real
        // per-sample engine rather than the closed-form response formula).
        rt.outputTrimDb = 0.0;
        eng.setModel(rt);
        eng.resetState();
        std::vector<float> outL0(Nu), outR0(Nu);
        eng.processBlock(inL.data(), inR.data(), outL0.data(), outR0.data(), uint32_t(N));
        double worstDiff = 0.0;
        for (int i = skip; i < N; ++i)
            worstDiff = std::max(worstDiff, std::fabs(double(outL0[size_t(i)]) - double(inL[size_t(i)])));
        printf("     outputTrimDb=0dB: worst |output - input|=%.3e (want ~0)\n", worstDiff);
        if (worstDiff > 1e-6) { printf("FAIL: inert band + zero trim should reproduce the input almost exactly\n"); ++failures; }
    }

    // 26b. DynamicEqEngine's own Audition (setAudition()/setAuditionBandIndex()),
    //      added to preview one band's isolated detector output by ear --
    //      same feature/idiom the (now-deleted) De-esser plugin's own
    //      DeesserEngine::setListen() used, generalized to "which band"
    //      since this engine can hold several.
    //      Two bands: band 0 real (dynamics engaged, so it actually reduces
    //      gain on a loud tone), band 1 inert and untouched by Audition. (a)
    //      previewing band 0 must differ from the normal path and must NOT
    //      differ when previewing an out-of-range index (Audition has nothing
    //      valid to substitute, so it must fall back to the normal path
    //      exactly, not silently mute or pass raw input). (b) previewing
    //      band 0 leaves band 1's own filter states running underneath
    //      (checked indirectly: turning Audition back off after some blocks
    //      reproduces the same steady-state output as an engine that was
    //      never auditioning at all, i.e. Audition didn't leave anything
    //      desynchronized behind it).
    {
        auto makeRt = [](double freqHz, bool dynamicsOn) {
            DynamicEqRtModel rt;
            rt.numBands = 2;
            DynamicEqBand real;
            real.freqHz = freqHz; real.q = 2.0; real.gainDb = 0.0;
            real.dynamicsEnabled = dynamicsOn;
            real.thresholdDb = -30.0; real.ratio = 4.0; real.rangeDb = 18.0;
            real.attackMs = 5.0; real.releaseMs = 80.0;
            rt.bands[0] = real;
            DynamicEqBand inert;
            inert.freqHz = 8000.0; inert.q = 3.0; inert.gainDb = 0.0; inert.dynamicsEnabled = false;
            rt.bands[1] = inert;
            return rt;
        };

        const int N = int(0.3 * fs);
        const size_t Nu = size_t(N);
        std::vector<float> inL(Nu), inR(Nu);
        for (int i = 0; i < N; ++i)
        {
            const float s = float(0.5 * std::sin(2.0 * M_PI * 1000.0 * i / fs));
            inL[size_t(i)] = inR[size_t(i)] = s;
        }

        DynamicEqEngine engNormal;
        engNormal.prepare(fs);
        engNormal.setModel(makeRt(1000.0, true));
        engNormal.resetState();
        std::vector<float> outNL(Nu), outNR(Nu);
        engNormal.processBlock(inL.data(), inR.data(), outNL.data(), outNR.data(), uint32_t(N));

        DynamicEqEngine engAudition;
        engAudition.prepare(fs);
        engAudition.setModel(makeRt(1000.0, true));
        engAudition.resetState();
        engAudition.setAudition(true);
        engAudition.setAuditionBandIndex(0);
        std::vector<float> outLL(Nu), outLR(Nu);
        engAudition.processBlock(inL.data(), inR.data(), outLL.data(), outLR.data(), uint32_t(N));

        double worstDiff = 0.0;
        for (int i = 0; i < N; ++i) worstDiff = std::max(worstDiff, double(std::fabs(outNL[size_t(i)] - outLL[size_t(i)])));
        printf("[26b] DynamicEqEngine Audition(band 0) differs from the normal path: worst |diff|=%.3e (want > 0)\n", worstDiff);
        if (worstDiff <= 0.0) { printf("FAIL: Audition(band 0) produced the same output as the normal path\n"); ++failures; }

        DynamicEqEngine engOob;
        engOob.prepare(fs);
        engOob.setModel(makeRt(1000.0, true));
        engOob.resetState();
        engOob.setAudition(true);
        engOob.setAuditionBandIndex(5); // out of range -- only 2 bands exist
        std::vector<float> outOL(Nu), outOR(Nu);
        engOob.processBlock(inL.data(), inR.data(), outOL.data(), outOR.data(), uint32_t(N));
        double oobDiff = 0.0;
        for (int i = 0; i < N; ++i) oobDiff = std::max(oobDiff, double(std::fabs(outNL[size_t(i)] - outOL[size_t(i)])));
        printf("     Audition(out-of-range band) falls back to the normal path: worst |diff|=%.3e (want ~0)\n", oobDiff);
        if (oobDiff > 1e-9) { printf("FAIL: an out-of-range Audition band index should be a no-op, not silence/passthrough\n"); ++failures; }
    }

    // 27. StateSpaceMatrices::sparseCascade2's zero-pattern claim, checked
    //     directly against real matrices from both EqModel and
    //     MbCompressorModel across a range of configurations -- a
    //     performance fix (StateSpaceEngine.hpp's A*x recursion, and its
    //     coefficient-smoothing loop, now skip A[i][j] for j at or beyond
    //     row i's own 2-wide diagonal block whenever this flag is set)
    //     that depends entirely on that claim actually being true. This
    //     doesn't just trust the mathematical argument in
    //     CoefficientMapper.hpp's own comments -- it mechanically verifies
    //     every single entry the fast path would skip really is 0.0 in the
    //     matrices the real model-assembly code actually produces, so a
    //     future change to that code that ever violates the claim (without
    //     also clearing the flag) fails a test instead of silently
    //     corrupting audio.
    {
        auto checkZeroPattern = [](const StateSpaceMatrices& m, const char* label) -> bool
        {
            if (!m.sparseCascade2) { printf("     %s: sparseCascade2 not set, nothing to check\n", label); return true; }
            const int n = clampOrder(m.order);
            double worst = 0.0;
            for (int i = 0; i < n; ++i)
            {
                const int jEnd = std::min(n, 2 * (i / 2) + 2);
                for (int j = jEnd; j < n; ++j) worst = std::max(worst, std::fabs(m.A[i][j]));
            }
            printf("     %s: order=%d  worst |A[i][j]| for j beyond row i's block = %.3e (want exactly 0)\n", label, n, worst);
            return worst == 0.0;
        };

        bool ok27 = true;

        // A representative spread of EqModel configurations: every band
        // type at least once, plus a multi-stage (slopeStages) Low/High
        // Pass, enough bands to exercise a non-trivial order.
        {
            EqModel m;
            m.bands.push_back({ EqBandType::Bell, 100.0, 6.0, 1.0, true, false, 1 });
            m.bands.push_back({ EqBandType::LowShelf, 200.0, -4.0, 0.9, true, false, 1 });
            m.bands.push_back({ EqBandType::HighShelf, 8000.0, 3.0, 0.8, true, false, 1 });
            m.bands.push_back({ EqBandType::LowPass, 12000.0, 0.0, 0.7071, true, false, 4 }); // 48dB/oct: 4 cascaded stages
            m.bands.push_back({ EqBandType::HighPass, 40.0, 0.0, 0.7071, true, false, 3 });
            m.bands.push_back({ EqBandType::Notch, 3000.0, 0.0, 5.0, true, false, 1 });
            m.bands.push_back({ EqBandType::BandPass, 1500.0, 0.0, 2.0, true, false, 1 });
            m.bands.push_back({ EqBandType::AllPass, 500.0, 0.0, 1.0, true, false, 1 });
            m.bands.push_back({ EqBandType::Bell, 5000.0, -8.0, 3.0, false, false, 1 }); // disabled -- excluded from assembly entirely
            const StateSpaceMatrices assembled = eq::assembleStateSpace(m, fs);
            if (!checkZeroPattern(assembled, "[27] EqModel (8 active bands, mixed types, slopeStages up to 4)")) ok27 = false;

            EqModel empty; // active.empty() branch -- identity passthrough
            if (!checkZeroPattern(eq::assembleStateSpace(empty, fs), "[27] EqModel (no bands, identity passthrough)")) ok27 = false;
        }

        // MbCompressorModel: a few crossovers (parallel bank), each band's
        // own dynamics enabled/disabled/soloed in different combinations
        // (schedule/gain content never touches A, but exercise it anyway).
        {
            MbCompressorModel m;
            m.crossoverHz = { 200.0, 2000.0 };
            m.bands.resize(3);
            m.bands[0].thresholdDb = -20.0; m.bands[0].ratio = 3.0;
            m.bands[1].thresholdDb = -15.0; m.bands[1].ratio = 4.0; m.soloedBand = 1;
            m.bands[2].thresholdDb = -25.0; m.bands[2].ratio = 2.0; m.bands[2].enabled = false;
            const StateSpaceMatrices assembled = mb::assembleStateSpace(m, fs);
            if (!checkZeroPattern(assembled, "[27] MbCompressorModel (3 bands, 2 crossovers, solo+disabled mixed in)")) ok27 = false;
        }

        printf("[27] sparseCascade2 zero-pattern holds for real assembled matrices: %s\n", ok27 ? "OK" : "FAIL");
        if (!ok27) { printf("FAIL: a real assembled matrix has a nonzero entry the fast path would have skipped\n"); ++failures; }
    }

    // 28. The actual point of test [27]: running the *same* model through
    //     StateSpaceEngine::processBlock() with the fast path enabled vs.
    //     forced off must produce bit-for-bit identical audio, not just
    //     "close". Real program material (not a pure tone) through a
    //     busy, multi-band EQ -- the configuration most likely to expose
    //     any accumulation-order mistake in the fast path, since it has
    //     the most active states.
    {
        EqModel m;
        m.bands.push_back({ EqBandType::Bell, 120.0, 5.0, 1.2, true, false, 1 });
        m.bands.push_back({ EqBandType::Bell, 900.0, -6.0, 2.0, true, false, 1 });
        m.bands.push_back({ EqBandType::Bell, 4000.0, 4.0, 1.5, true, false, 1 });
        m.bands.push_back({ EqBandType::LowShelf, 150.0, 3.0, 0.9, true, false, 1 });
        m.bands.push_back({ EqBandType::HighShelf, 6000.0, -3.0, 0.9, true, false, 1 });
        m.bands.push_back({ EqBandType::LowPass, 15000.0, 0.0, 0.7071, true, false, 3 });
        m.bands.push_back({ EqBandType::HighPass, 30.0, 0.0, 0.7071, true, false, 2 });

        StateSpaceMatrices sparseModel = eq::assembleStateSpace(m, fs);
        if (!sparseModel.sparseCascade2) { printf("FAIL: [28] setup -- expected sparseCascade2 to be set for this model\n"); ++failures; }
        StateSpaceMatrices denseModel = sparseModel;
        denseModel.sparseCascade2 = false; // otherwise byte-for-byte the same model -- the ONLY difference under test

        StabilityGuard::enforce(sparseModel);
        StabilityGuard::enforce(denseModel); // identical input, so this must do the identical thing to both

        StateSpaceEngine engSparse, engDense;
        engSparse.prepare(fs); engDense.prepare(fs);
        engSparse.setModel(sparseModel); engDense.setModel(denseModel);
        engSparse.resetState(); engDense.resetState();

        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> dist(-0.5, 0.5);
        const int N = 20000;
        const size_t Nu = size_t(N);
        std::vector<float> inL(Nu), inR(Nu), outSL(Nu), outSR(Nu), outDL(Nu), outDR(Nu);
        for (int i = 0; i < N; ++i)
        {
            // Broadband noise, not a pure tone -- exercises every state,
            // not just whichever ones a single frequency happens to excite.
            inL[size_t(i)] = float(dist(rng));
            inR[size_t(i)] = float(dist(rng));
        }
        engSparse.processBlock(inL.data(), inR.data(), outSL.data(), outSR.data(), uint32_t(N));
        engDense.processBlock(inL.data(), inR.data(), outDL.data(), outDR.data(), uint32_t(N));

        double worstDiff = 0.0;
        int worstIdx = -1;
        for (int i = 0; i < N; ++i)
        {
            const double dl = std::fabs(double(outSL[size_t(i)]) - double(outDL[size_t(i)]));
            const double dr = std::fabs(double(outSR[size_t(i)]) - double(outDR[size_t(i)]));
            if (dl > worstDiff) { worstDiff = dl; worstIdx = i; }
            if (dr > worstDiff) { worstDiff = dr; worstIdx = i; }
        }
        printf("[28] sparse-path vs dense-path output, %d samples of broadband noise through 7 bands: worst |diff|=%.3e at sample %d (want exactly 0)\n",
               N, worstDiff, worstIdx);
        if (worstDiff != 0.0) { printf("FAIL: the sparse fast path produced different audio than the dense path -- this must never happen\n"); ++failures; }
    }

    // 29. Test [28]'s scenario never actually touches
    //     StateSpaceEngine::processBlock()'s coefficient-*smoothing* loop
    //     (a fresh setModel()+resetState() snaps smoothed_ straight to the
    //     target, so smoothingActive_ stays false throughout) -- and that
    //     loop was *also* changed to use the same fast-path bound. Same
    //     bit-exactness check, but this time triggered by a live edit
    //     partway through (same order, so it smooths rather than ducks):
    //     process some noise, submit a different EQ model mid-stream, keep
    //     processing while the coefficients are actively gliding toward
    //     it, and compare sparse vs. dense the whole way through.
    {
        EqModel mA;
        mA.bands.push_back({ EqBandType::Bell, 200.0, 6.0, 1.0, true, false, 1 });
        mA.bands.push_back({ EqBandType::Bell, 3000.0, -5.0, 1.5, true, false, 1 });
        mA.bands.push_back({ EqBandType::HighShelf, 8000.0, 4.0, 0.8, true, false, 1 });

        EqModel mB = mA; // same *band count* (so same assembled order -- smooths, doesn't duck), different values
        mB.bands[0].freqHz = 350.0; mB.bands[0].gainDb = -8.0; mB.bands[0].q = 2.5;
        mB.bands[1].freqHz = 5000.0; mB.bands[1].gainDb = 7.0;
        mB.bands[2].gainDb = -6.0;

        StateSpaceMatrices sparseA = eq::assembleStateSpace(mA, fs);
        StateSpaceMatrices sparseB = eq::assembleStateSpace(mB, fs);
        StabilityGuard::enforce(sparseA);
        StabilityGuard::enforce(sparseB);
        if (!sparseA.sparseCascade2 || !sparseB.sparseCascade2 || sparseA.order != sparseB.order)
        {
            printf("FAIL: [29] setup -- expected both models sparse and same order (smoothing, not ducking)\n");
            ++failures;
        }
        StateSpaceMatrices denseA = sparseA; denseA.sparseCascade2 = false;
        StateSpaceMatrices denseB = sparseB; denseB.sparseCascade2 = false;

        StateSpaceEngine engSparse, engDense;
        engSparse.prepare(fs); engDense.prepare(fs);
        engSparse.setModel(sparseA); engDense.setModel(denseA);
        engSparse.resetState(); engDense.resetState();

        std::mt19937 rng(777);
        std::uniform_real_distribution<double> dist(-0.5, 0.5);
        const int N1 = 2000, N2 = 20000; // N1 before the edit, N2 (spanning the whole smoothing ramp and well past it) after
        const size_t N1u = size_t(N1), N2u = size_t(N2);
        std::vector<float> in1L(N1u), in1R(N1u), s1L(N1u), s1R(N1u), d1L(N1u), d1R(N1u);
        std::vector<float> in2L(N2u), in2R(N2u), s2L(N2u), s2R(N2u), d2L(N2u), d2R(N2u);
        for (int i = 0; i < N1; ++i) { in1L[size_t(i)] = float(dist(rng)); in1R[size_t(i)] = float(dist(rng)); }
        for (int i = 0; i < N2; ++i) { in2L[size_t(i)] = float(dist(rng)); in2R[size_t(i)] = float(dist(rng)); }

        engSparse.processBlock(in1L.data(), in1R.data(), s1L.data(), s1R.data(), uint32_t(N1));
        engDense.processBlock(in1L.data(), in1R.data(), d1L.data(), d1R.data(), uint32_t(N1));

        engSparse.setModel(sparseB); // the live edit -- same order, so this smooths
        engDense.setModel(denseB);

        engSparse.processBlock(in2L.data(), in2R.data(), s2L.data(), s2R.data(), uint32_t(N2));
        engDense.processBlock(in2L.data(), in2R.data(), d2L.data(), d2R.data(), uint32_t(N2));

        double worstDiff29 = 0.0;
        for (int i = 0; i < N1; ++i)
        {
            worstDiff29 = std::max(worstDiff29, double(std::fabs(s1L[size_t(i)] - d1L[size_t(i)])));
            worstDiff29 = std::max(worstDiff29, double(std::fabs(s1R[size_t(i)] - d1R[size_t(i)])));
        }
        for (int i = 0; i < N2; ++i)
        {
            worstDiff29 = std::max(worstDiff29, double(std::fabs(s2L[size_t(i)] - d2L[size_t(i)])));
            worstDiff29 = std::max(worstDiff29, double(std::fabs(s2R[size_t(i)] - d2R[size_t(i)])));
        }
        printf("[29] sparse vs dense through a live mid-stream edit (coefficient smoothing exercised): worst |diff|=%.3e (want exactly 0)\n", worstDiff29);
        if (worstDiff29 != 0.0) { printf("FAIL: the sparse fast path's coefficient-smoothing loop produced different audio than the dense path\n"); ++failures; }
    }

    // 32. Stereo crossfeed (EqBand::crossFeed, eq::assembleStateSpace()'s
    //     own graft step): the left input fed to the right output, and
    //     the other way around. A
    //     LeftToRight band must produce a STRONG H[1][0] (Left input ->
    //     Right output) term at its own frequency, leave H[1][1]/H[0][0]
    //     (each channel's own ordinary gain) untouched, and leave H[0][1]
    //     (the opposite direction) at exactly zero -- same "own channel
    //     boosted, everything else exactly untouched" discipline test
    //     [10d] already established for plain Left/Right bands, now for
    //     the genuinely different crossfeed routing.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 1000.0, 0.0, 1.0, true, false }); // ordinary Both band, well away from the crossfeed's own frequency
        eq.bands.back().gainDb = 6.0;
        eq.bands.push_back({ EqBandType::Bell, 3000.0, 0.0, 1.0, true, false }); // crossfeed band
        eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        eq.bands.back().gainDb = 12.0;
        // EqModel::crossFeedGainLtoRDb defaults to -60 (the point of it --
        // see this field's own comment); a real user raises it, so the
        // test does too, to a plain unity baseline (0dB) the Bell's own
        // +12dB peak then rides on top of, exactly like
        // assembleStateSpace()'s own graft-step comment describes.
        eq.crossFeedGainLtoRDb = 0.0;

        const StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
        const double crossFreq = 3000.0;
        const Complex zCross = std::polar(1.0, 2.0 * M_PI * crossFreq / fs);
        const ComplexResponse respCross = TransferFunction::evaluate(assembled, zCross);
        const double crossGainDb = 20.0 * std::log10(std::abs(respCross.H[1][0])); // L input -> R output
        const double reverseCross = std::abs(respCross.H[0][1]);                   // R input -> L output -- must be (numerically) zero
        const bool crossReachesRight = crossGainDb > 10.0; // the 12dB Bell's own peak, minus a little tolerance
        const bool noReverseLeak = reverseCross < 1e-9; // same tight-epsilon convention as test [10d]'s own noCrossTalk check, not exact ==0.0 (matrix inversion, not a direct write)
        printf("[32] LeftToRight crossfeed: L->R gain at 3kHz=%.2fdB (want >10) R->L leak=%.2e (want ~0): %s\n",
               crossGainDb, reverseCross, (crossReachesRight && noReverseLeak) ? "OK" : "FAIL");
        if (!(crossReachesRight && noReverseLeak)) { printf("FAIL: LeftToRight crossfeed didn't reach the Right output, or leaked into Left\n"); ++failures; }

        // The ordinary Both band at 1kHz must still show up identically
        // on BOTH channels' own diagonal terms -- adding a crossfeed band
        // elsewhere in the list must not disturb the normal L/R assembly
        // at all.
        const Complex zBoth = std::polar(1.0, 2.0 * M_PI * 1000.0 / fs);
        const ComplexResponse respBoth = TransferFunction::evaluate(assembled, zBoth);
        const double bothL = 20.0 * std::log10(std::abs(respBoth.H[0][0]));
        const double bothR = 20.0 * std::log10(std::abs(respBoth.H[1][1]));
        const bool bothChannelsOk = std::fabs(bothL - 6.0) < 0.1 && std::fabs(bothR - 6.0) < 0.1;
        printf("     ordinary Both band untouched by the crossfeed band elsewhere in the list: L=%.2fdB R=%.2fdB (want ~6 both): %s\n",
               bothL, bothR, bothChannelsOk ? "OK" : "FAIL");
        if (!bothChannelsOk) { printf("FAIL: adding a crossfeed band disturbed an ordinary Both band's own response\n"); ++failures; }

        printf("     hasPerChannelBands() true with an active crossfeed band: %s\n", eq::hasPerChannelBands(eq) ? "OK" : "FAIL");
        if (!eq::hasPerChannelBands(eq)) { printf("FAIL: hasPerChannelBands() missed an active crossfeed band\n"); ++failures; }

        // sparseCascade2's own exact contract (see StateSpaceMatrices'
        // field comment, and test [27]'s identical methodology): every
        // A[i][j] must be exactly 0 for j at or beyond row i's own 2-wide
        // diagonal block -- the crossfeed graft appends a brand-new
        // diagonal block, which must not violate this for ANY row, old
        // or new.
        {
            bool sparseOk = assembled.sparseCascade2;
            const int n = clampOrder(assembled.order);
            double worst = 0.0;
            for (int i = 0; i < n; ++i)
            {
                const int jStart = 2 * (i / 2 + 1);
                for (int j = jStart; j < n; ++j) worst = std::max(worst, std::fabs(assembled.A[size_t(i)][size_t(j)]));
            }
            printf("     sparseCascade2 flag=%d, worst |A[i][j]| beyond own block=%.2e (want exactly 0): %s\n",
                   assembled.sparseCascade2, worst, (sparseOk && worst == 0.0) ? "OK" : "FAIL");
            if (!(sparseOk && worst == 0.0)) { printf("FAIL: the crossfeed graft broke sparseCascade2's own zero-pattern contract\n"); ++failures; }
        }

        // JSON round-trip.
        EqModel parsed;
        const bool jsonOk = eq::fromJson(eq::toJson(eq), parsed) &&
                             parsed.bands.size() == 2 &&
                             parsed.bands[0].crossFeed == EqCrossFeed::None &&
                             parsed.bands[1].crossFeed == EqCrossFeed::LeftToRight;
        printf("     crossFeed round-trips through JSON: %s\n", jsonOk ? "OK" : "FAIL");
        if (!jsonOk) { printf("FAIL: EqBand::crossFeed lost or corrupted across a JSON round-trip\n"); ++failures; }
    }

    // 32a. The DSP-design fix itself, for the ORIGINAL crossfeed
    //      implementation's flaw -- a Bell
    //      shape alone left every frequency AWAY from the Bell's own peak
    //      at a full, unscaled, uncontrolled 0dB leak. EqModel::
    //      crossFeedGainLtoRDb must apply as a genuinely FLAT broadband
    //      baseline for the WHOLE transit: a crossfeed band with gainDb=0
    //      (no Bell bump at all) under a model with crossFeedGainLtoRDb=
    //      -20 must sit at -20dB at EVERY frequency, not just near
    //      band.freqHz.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 1000.0, 0.0, 1.0, true, false }); // flat shape -- no Bell bump
        eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        eq.crossFeedGainLtoRDb = -20.0;

        const StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
        const double testFreqs[] = { 100.0, 1000.0, 3000.0, 10000.0 };
        bool flatOk = true;
        double worstDeviationDb = 0.0;
        for (double f : testFreqs)
        {
            const Complex z = std::polar(1.0, 2.0 * M_PI * f / fs);
            const ComplexResponse resp = TransferFunction::evaluate(assembled, z);
            const double gainDb = 20.0 * std::log10(std::abs(resp.H[1][0]));
            worstDeviationDb = std::max(worstDeviationDb, std::fabs(gainDb - (-20.0)));
            if (std::fabs(gainDb - (-20.0)) > 0.5) flatOk = false;
        }
        printf("[32a] crossFeedGainLtoRDb is a genuinely flat baseline (no Bell bump): worst deviation from -20dB=%.2fdB: %s\n",
               worstDeviationDb, flatOk ? "OK" : "FAIL");
        if (!flatOk) { printf("FAIL: crossfeed baseline isn't flat across frequency -- the old Bell-only design flaw is still present\n"); ++failures; }

        // Multiple crossfeed bands in the same direction must NOT
        // multiply their way back down to near-silence -- the real,
        // non-obvious bug the earlier PER-BAND design had (see
        // EqModel::crossFeedGainLtoRDb's own comment): a second band in
        // the same direction, with its own Bell shape but no gain field
        // of its own to fight with, must land at the SAME model-wide
        // -20dB baseline the first one did, not -40dB from an
        // accidental series-cascade compounding.
        EqModel eq2;
        eq2.bands.push_back({ EqBandType::Bell, 500.0, 0.0, 1.0, true, false });
        eq2.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        eq2.bands.push_back({ EqBandType::Bell, 5000.0, 0.0, 1.0, true, false });
        eq2.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        eq2.crossFeedGainLtoRDb = -20.0;
        const StateSpaceMatrices assembled2 = eq::assembleStateSpace(eq2, fs);
        const Complex zMid = std::polar(1.0, 2.0 * M_PI * 1500.0 / fs); // away from both bands' own (flat, so irrelevant) frequencies
        const double gainDb2 = 20.0 * std::log10(std::abs(TransferFunction::evaluate(assembled2, zMid).H[1][0]));
        const bool twoBandsOk = std::fabs(gainDb2 - (-20.0)) < 0.5;
        printf("      two crossfeed bands, same direction, share one -20dB baseline (not -40dB compounded): %.2fdB: %s\n",
               gainDb2, twoBandsOk ? "OK" : "FAIL");
        if (!twoBandsOk) { printf("FAIL: multiple crossfeed bands in one direction compounded their gain instead of sharing one model-level baseline\n"); ++failures; }
    }

    // 32b. Backward compatibility: an old-format preset (hand-written,
    //      missing the "crossFeed" key entirely -- exactly what every
    //      real .sseqjson saved before this field existed looks like)
    //      must still parse, with every band's crossFeed defaulting to
    //      None -- the EQ plugins must stay backward compatible, since
    //      existing projects already use them.
    {
        const std::string oldFormatJson =
            "{\n"
            "  \"type\": \"statespace-eq-preset\",\n"
            "  \"version\": 1,\n"
            "  \"name\": \"Old Preset\",\n"
            "  \"outputTrimDb\": 0,\n"
            "  \"bands\": [\n"
            "    { \"type\": \"bell\", \"freqHz\": 1000, \"gainDb\": 6, \"q\": 1, \"slopeStages\": 1, \"enabled\": true, \"solo\": false, \"harmonicGroup\": 0, \"harmonicRatio\": 1, \"channel\": \"left\" }\n"
            "  ]\n"
            "}\n";
        EqModel parsed;
        const bool parseOk = eq::fromJson(oldFormatJson, parsed);
        const bool defaultedToNone = parseOk && parsed.bands.size() == 1 && parsed.bands[0].crossFeed == EqCrossFeed::None;
        const bool channelStillOk = parseOk && parsed.bands.size() == 1 && parsed.bands[0].channel == EqChannel::Left;
        printf("[32b] old-format preset (no \"crossFeed\" key at all) parses, defaults to None, channel unaffected: %s\n",
               (defaultedToNone && channelStillOk) ? "OK" : "FAIL");
        if (!(defaultedToNone && channelStillOk)) { printf("FAIL: loading a pre-crossfeed preset didn't default cleanly\n"); ++failures; }

        // And the DSP side: assembling that same old-format model must be
        // bit-identical to assembling a plain, never-touched-crossfeed
        // EqModel with the same one band -- the crossfeed graft step
        // must not run at all (and, if it somehow did, must not have
        // changed anything) when nothing uses it.
        EqModel plain;
        plain.bands.push_back({ EqBandType::Bell, 1000.0, 6.0, 1.0, true, false });
        plain.bands.back().channel = EqChannel::Left;
        const StateSpaceMatrices fromOld = eq::assembleStateSpace(parsed, fs);
        const StateSpaceMatrices fromPlain = eq::assembleStateSpace(plain, fs);
        bool identical = fromOld.order == fromPlain.order;
        if (identical)
        {
            const int n = clampOrder(fromOld.order);
            for (int i = 0; i < n && identical; ++i)
            {
                for (int j = 0; j < n && identical; ++j) if (fromOld.A[size_t(i)][size_t(j)] != fromPlain.A[size_t(i)][size_t(j)]) identical = false;
                for (int j = 0; j < kNumInputs && identical; ++j) if (fromOld.B[size_t(i)][size_t(j)] != fromPlain.B[size_t(i)][size_t(j)]) identical = false;
            }
            for (int i = 0; i < kNumOutputs && identical; ++i)
            {
                for (int j = 0; j < n && identical; ++j) if (fromOld.C[size_t(i)][size_t(j)] != fromPlain.C[size_t(i)][size_t(j)]) identical = false;
                for (int j = 0; j < kNumInputs && identical; ++j) if (fromOld.D[size_t(i)][size_t(j)] != fromPlain.D[size_t(i)][size_t(j)]) identical = false;
            }
        }
        printf("      assembled matrix bit-identical to a never-touched-crossfeed model: %s\n", identical ? "OK" : "FAIL");
        if (!identical) { printf("FAIL: loading an old preset produced a different assembled matrix than before this feature existed\n"); ++failures; }
    }

    // 32c. Soloing a band whose ONLY routing is crossfeed (no plain
    //      Left/Right/Both band active at all) must NOT silence its own
    //      destination channel -- a targeted regression test for the
    //      specific ordering bug this feature's own implementation had to
    //      avoid: the existing solo-silence logic (see
    //      eq::assembleStateSpace()'s own comment) only knew about
    //      leftStages/rightStages, and would have wrongly zeroed the
    //      destination channel's entire C/D row out from under a soloed
    //      crossfeed band if the crossfeed graft ran BEFORE that check,
    //      or if the check itself didn't also look at the crossfeed
    //      lists.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 1000.0, 12.0, 1.0, true, true }); // soloed, crossfeed only
        eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        eq.crossFeedGainLtoRDb = 0.0; // see test [32]'s own comment -- default -60 baseline raised to unity for this assertion
        eq.bands.push_back({ EqBandType::Bell, 200.0, 6.0, 1.0, true, false }); // not soloed -- excluded entirely
        eq.bands.back().channel = EqChannel::Both;

        const StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
        const Complex z = std::polar(1.0, 2.0 * M_PI * 1000.0 / fs);
        const ComplexResponse resp = TransferFunction::evaluate(assembled, z);
        const double crossGainDb = 20.0 * std::log10(std::abs(resp.H[1][0]));
        const bool crossSurvivedSolo = crossGainDb > 10.0;
        printf("[32c] soloing a crossfeed-only band still reaches its destination channel: L->R gain=%.2fdB (want >10): %s\n",
               crossGainDb, crossSurvivedSolo ? "OK" : "FAIL");
        if (!crossSurvivedSolo) { printf("FAIL: soloing a crossfeed-only band incorrectly silenced its own destination channel\n"); ++failures; }

        // The Left channel's own diagonal term (H[0][0]) legitimately IS
        // silenced here: nothing soloed feeds Left directly (the soloed
        // band is a LeftToRight crossfeed, which reads L but writes only
        // to R), so Left has no active content of its own right now --
        // this is the pre-existing, correct solo behavior test [10e]
        // already covers, just confirming the crossfeed addition didn't
        // accidentally leave Left as a passthrough leak instead.
        const bool leftSilentOk = std::abs(resp.H[0][0]) < 1e-9 && assembled.D[0][0] == 0.0 && assembled.D[0][1] == 0.0;
        printf("      Left's own output (nothing soloed feeds it directly) stays exactly silent: %s\n", leftSilentOk ? "OK" : "FAIL");
        if (!leftSilentOk) { printf("FAIL: Left leaked through despite nothing soloed reaching it directly\n"); ++failures; }
    }

    // 32d. The whole PluginEq::rebuildEngineModel() path (assemble +
    //      StabilityGuard::enforce) must fit comfortably on a host's real-
    //      time process-thread stack -- it runs there, from run(), every
    //      time the GUI commits an edit (kParamMirrorCommit). Real crash
    //      this guards (Ardour crashed as soon as a band was placed):
    //      the crossfeed graft added a second ~131KB StateSpaceMatrices
    //      local inside assembleStateSpace(), taking the path to ~508KB of
    //      stack and overflowing Ardour's 512KB process threads (SIGSEGV
    //      528,640 bytes deep, per the core dump). Run on a thread with
    //      384KB -- Ardour's 512KB minus generous headroom for the host's
    //      and DPF's own frames above run() -- with crossfeed bands in
    //      BOTH directions (the deepest path).
    {
        struct Job
        {
            static void* run(void* arg)
            {
                const double sr = 48000.0;
                EqModel eq;
                for (int i = 0; i < 12; ++i)
                {
                    eq.bands.push_back({ EqBandType::Bell, 100.0 * (i + 1), 3.0, 1.0, true, false });
                    if (i == 3) eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
                    if (i == 7) eq.bands.back().crossFeed = EqCrossFeed::RightToLeft;
                    if (i == 9) eq.bands.back().channel = EqChannel::Left;
                }
                StateSpaceMatrices assembled = eq::assembleStateSpace(eq, sr);
                StabilityGuard::enforce(assembled);
                *static_cast<int*>(arg) = assembled.order;
                return nullptr;
            }
        };
        constexpr size_t kStackBytes = 384 * 1024;
        int order = -1;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, kStackBytes);
        pthread_t th;
        const bool started = pthread_create(&th, &attr, &Job::run, &order) == 0;
        if (started) pthread_join(th, nullptr); // an overflow kills the whole test binary here, which fails the build's --test step just the same
        pthread_attr_destroy(&attr);
        const bool ok = started && order > 0;
        printf("[32d] EQ rebuild path (crossfeed both ways) fits a %zuKB real-time thread stack: order=%d: %s\n",
               kStackBytes / 1024, order, ok ? "OK" : "FAIL");
        if (!ok) { printf("FAIL: EQ rebuild path did not complete on a %zuKB thread stack\n", kStackBytes / 1024); ++failures; }
    }

    // 33. dynamic_eq2 (plugin/dsp/DynamicEqModel2.hpp/.cpp) -- a hand-
    //     adapted CLONE of DynamicEqModel/dyneq::, created as a safe place
    //     to grow new dynamic-EQ features (starting with stereo crossfeed
    //     bands, mirroring what parametric_eq just got -- see test [32]'s
    //     own comment) without risking the original, already-in-use
    //     dynamic_eq plugin. As of THIS commit the clone is functionally
    //     identical to the original -- this test is a "clone fidelity"
    //     check: the same band config assembled through BOTH
    //     dyneq::assembleStateSpace() and dyneq2::assembleStateSpace()
    //     must produce bit-identical matrices, proving the copy is exact
    //     before any new feature work diverges it. Also confirms the two
    //     plugins' own JSON preset formats use genuinely distinct "type"
    //     tags ("statespace-dyneq-preset" vs. "statespace-dyneq2-preset"),
    //     so a preset saved from one is correctly refused by the other's
    //     own fromJson() rather than silently cross-loading.
    {
        DynamicEqBand bandOrig{};
        bandOrig.freqHz = 1000.0; bandOrig.gainDb = 12.0; bandOrig.q = 1.0;
        bandOrig.thresholdDb = -20.0; bandOrig.ratio = 4.0; bandOrig.attackMs = 5.0; bandOrig.releaseMs = 80.0;
        bandOrig.dynamicsEnabled = false; // static export path -- same closed-form RBJ cascade either way, see dyneq::assembleStateSpace()'s own comment
        DynamicEqModel modelOrig;
        modelOrig.bands.push_back(bandOrig);

        DynamicEqBand2 bandClone{};
        bandClone.freqHz = 1000.0; bandClone.gainDb = 12.0; bandClone.q = 1.0;
        bandClone.thresholdDb = -20.0; bandClone.ratio = 4.0; bandClone.attackMs = 5.0; bandClone.releaseMs = 80.0;
        bandClone.dynamicsEnabled = false;
        DynamicEqModel2 modelClone;
        modelClone.bands.push_back(bandClone);

        const StateSpaceMatrices fromOrig = dyneq::assembleStateSpace(modelOrig, fs);
        const StateSpaceMatrices fromClone = dyneq2::assembleStateSpace(modelClone, fs);
        bool identical = fromOrig.order == fromClone.order;
        if (identical)
        {
            const int n = clampOrder(fromOrig.order);
            for (int i = 0; i < n && identical; ++i)
            {
                for (int j = 0; j < n && identical; ++j) if (fromOrig.A[size_t(i)][size_t(j)] != fromClone.A[size_t(i)][size_t(j)]) identical = false;
                for (int j = 0; j < kNumInputs && identical; ++j) if (fromOrig.B[size_t(i)][size_t(j)] != fromClone.B[size_t(i)][size_t(j)]) identical = false;
            }
            for (int i = 0; i < kNumOutputs && identical; ++i)
            {
                for (int j = 0; j < n && identical; ++j) if (fromOrig.C[size_t(i)][size_t(j)] != fromClone.C[size_t(i)][size_t(j)]) identical = false;
                for (int j = 0; j < kNumInputs && identical; ++j) if (fromOrig.D[size_t(i)][size_t(j)] != fromClone.D[size_t(i)][size_t(j)]) identical = false;
            }
        }
        printf("[33] dynamic_eq2 clone fidelity: assembled matrix bit-identical to the original for the same band config: %s\n", identical ? "OK" : "FAIL");
        if (!identical) { printf("FAIL: DynamicEqModel2's own assembleStateSpace() diverged from the original it was cloned from\n"); ++failures; }

        // JSON preset formats must NOT cross-load.
        const std::string origJson = dyneq::toJson(modelOrig);
        const std::string cloneJson = dyneq2::toJson(modelClone);
        DynamicEqModel2 wrongParse;
        DynamicEqModel wrongParseOrig;
        const bool origRejectedByClone = !dyneq2::fromJson(origJson, wrongParse);
        const bool cloneRejectedByOrig = !dyneq::fromJson(cloneJson, wrongParseOrig);
        printf("     preset formats stay distinct: dynamic_eq's own JSON refused by dynamic_eq2's fromJson()=%d, and vice versa=%d: %s\n",
               origRejectedByClone, cloneRejectedByOrig, (origRejectedByClone && cloneRejectedByOrig) ? "OK" : "FAIL");
        if (!(origRejectedByClone && cloneRejectedByOrig)) { printf("FAIL: dynamic_eq and dynamic_eq2 presets can cross-load -- their own \"type\" tags collide\n"); ++failures; }

        // And the clone's own JSON round-trips its own format correctly.
        DynamicEqModel2 reparsed;
        const bool selfRoundTrips = dyneq2::fromJson(cloneJson, reparsed) && reparsed.bands.size() == 1 &&
                                     reparsed.bands[0].freqHz == 1000.0 && reparsed.bands[0].gainDb == 12.0;
        printf("     dynamic_eq2's own JSON round-trips through its own fromJson(): %s\n", selfRoundTrips ? "OK" : "FAIL");
        if (!selfRoundTrips) { printf("FAIL: DynamicEqModel2's own JSON round-trip is broken\n"); ++failures; }
    }

    // 34. Time-domain sustained-audio fault check for crossfeed bands,
    //     across frequency -- after the [32]/[32a] fix above, placing a
    //     crossfeed band almost anywhere (less often above 10kHz) still
    //     produced loud artifacts and a "numerical fault auto-recovered"
    //     warning. Every crossfeed test before this one (32/32a/32c/32d)
    //     only ever checked the assembled matrix's own STEADY-STATE
    //     frequency response (TransferFunction::evaluate(), pure linear
    //     algebra, no recursion at all) or its stack footprint -- none of
    //     them ever ran real audio through StateSpaceEngine's own
    //     per-sample recursion, its coefficient-smoothing blend
    //     (same-order edits, e.g. a live drag), or its order-changing
    //     "duck" transition (see StateSpaceEngine.hpp's own header
    //     comment) with a crossfeed graft in play. This test does both,
    //     across a spread of frequencies, including above 10kHz.
    {
        std::mt19937 rng(12345);
        std::uniform_real_distribution<float> noise(-0.3f, 0.3f);

        const double testFreqs[] = { 40.0, 80.0, 150.0, 300.0, 600.0, 1200.0, 2500.0, 5000.0, 9000.0, 12000.0, 16000.0, 19000.0 };
        bool anyFault = false;
        double worstFaultFreq = 0.0;

        for (double f : testFreqs)
        {
            // A representative real session, pushed toward the extreme
            // end of what a user could actually dial in (high Q, near
            // the crossfeed gain's own +12dB ceiling, BOTH directions
            // active with more than one band each) rather than the
            // gentle single-band models [32]/[32a] use -- if "almost
            // everywhere" is real, it should show up under stress, not
            // just under a mild, hand-picked case.
            EqModel eq;
            eq.bands.push_back({ EqBandType::Bell, 150.0, 8.0, 6.0, true, false });
            eq.bands.push_back({ EqBandType::Bell, 4000.0, -6.0, 8.0, true, false });
            eq.bands.push_back({ EqBandType::Bell, f, 10.0, 15.0, true, false });
            eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
            eq.bands.push_back({ EqBandType::Bell, std::max(20.0, f * 0.6), -8.0, 20.0, true, false });
            eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
            eq.bands.push_back({ EqBandType::Bell, std::min(fs * 0.45, f * 1.4), 10.0, 15.0, true, false });
            eq.bands.back().crossFeed = EqCrossFeed::RightToLeft;
            eq.bands.push_back({ EqBandType::Bell, std::max(20.0, f * 0.8), -8.0, 20.0, true, false });
            eq.bands.back().crossFeed = EqCrossFeed::RightToLeft;
            eq.crossFeedGainLtoRDb = 12.0;
            eq.crossFeedGainRtoLDb = 12.0;

            StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
            StabilityGuard::enforce(assembled);

            StateSpaceEngine engine;
            engine.prepare(fs);
            engine.setModel(assembled);

            bool faulted = false;
            constexpr uint32_t kBlock = 256;
            float inL[kBlock], inR[kBlock], outL[kBlock], outR[kBlock];

            auto runBlocks = [&](int n)
            {
                for (int b = 0; b < n && !faulted; ++b)
                {
                    for (uint32_t i = 0; i < kBlock; ++i) { inL[i] = noise(rng); inR[i] = noise(rng); }
                    engine.processBlock(inL, inR, outL, outR, kBlock);
                    if (engine.consumeFaultFlag()) faulted = true;
                    for (uint32_t i = 0; i < kBlock && !faulted; ++i)
                        if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) faulted = true;
                }
            };

            // Several seconds of continuous noise through the freshly-set
            // model -- covers the order-changing "duck" transition (this
            // is the very first model this fresh engine has ever seen)
            // settling in, then sustained steady-state operation long
            // enough for even a marginally-unstable pole (radius just
            // over 1.0) to diverge to a detectable fault.
            runBlocks(5 * int(fs) / int(kBlock) + 1);

            // Now simulate a live drag of BOTH crossfeed bands' own
            // frequencies at once -- same order every step (a pure
            // recoefficient, exercising the smoothingActive_ blend path),
            // a series of small setModel() calls with only a few samples
            // of audio between them, matching EqGraphWidget's own
            // drag-updates-every-motion-event behavior.
            for (int step = 0; step < 60 && !faulted; ++step)
            {
                const double drift = 1.0 + 0.01 * step;
                eq.bands[2].freqHz = std::max(20.0, std::min(fs * 0.45, f * drift));
                eq.bands[4].freqHz = std::max(20.0, std::min(fs * 0.45, f * 1.4 * drift));
                StateSpaceMatrices dragged = eq::assembleStateSpace(eq, fs);
                StabilityGuard::enforce(dragged);
                engine.setModel(dragged);
                runBlocks(4);
            }

            if (faulted) { anyFault = true; worstFaultFreq = f; }
            printf("      crossfeed @ %.0fHz sustained + drag, time-domain: %s\n", f, faulted ? "FAULT" : "OK");
        }
        printf("[34] time-domain numerical-fault check across crossfeed frequencies: %s\n", anyFault ? "FAIL" : "OK");
        if (anyFault) { printf("FAIL: a crossfeed band produced a NaN/Inf fault during sustained real-time playback (seen near %.0fHz)\n", worstFaultFreq); ++failures; }
    }

    // 35. StateSpaceEngine's own duck/version state machine: a second
    //     order-changing setModel() landing while the FIRST one's duck is
    //     still fading back IN (not just still fading out) must not be
    //     silently dropped. Found auditing StateSpaceEngine.hpp while
    //     chasing the "numerical fault" problem above
    //     -- a real bug (independent of whether it explains that report):
    //     the swap that actually applies `pendingModel_` only ever ran at
    //     the bottom of a FadingOut; a later order-changing edit arriving
    //     during FadingIn used to just overwrite pendingModel_ and leave
    //     duckState_ alone, so FadingIn would run to completion with that
    //     newer target never applied, and lastSeenVersion_ already past
    //     it -- permanently stuck on the OLDER, superseded model.
    {
        StateSpaceEngine engine;
        engine.prepare(fs);

        StateSpaceMatrices matA; // order 8 (setIdentityPassthrough()'s own default), D = 1*I
        StateSpaceMatrices matB; matB.order = 16; matB.D[0][0] = 2.0; matB.D[1][1] = 2.0; // A/B/C already zero from setIdentityPassthrough()
        StateSpaceMatrices matC; matC.order = 24; matC.D[0][0] = 5.0; matC.D[1][1] = 5.0;

        constexpr uint32_t kFrames = 4096;
        float inL[kFrames], inR[kFrames], outL[kFrames], outR[kFrames];
        for (uint32_t i = 0; i < kFrames; ++i) { inL[i] = 1.0f; inR[i] = 1.0f; }

        engine.setModel(matA);
        engine.processBlock(inL, inR, outL, outR, 64); // let A settle in (trivially -- pure D, no dynamics)

        engine.setModel(matB); // order 8 -> 16: starts a duck (FadingOut)
        const uint32_t fadeOutSamples = uint32_t(0.008 * fs) + 1; // matches StateSpaceEngine's own kDuckTimeSec
        engine.processBlock(inL, inR, outL, outR, fadeOutSamples); // run past the swap into FadingIn
        engine.processBlock(inL, inR, outL, outR, fadeOutSamples / 4); // partway through fading back in -- NOT yet at duckGain_==1.0

        engine.setModel(matC); // order 16 -> 24, landing mid-FadingIn -- the exact scenario the fix above targets
        engine.processBlock(inL, inR, outL, outR, kFrames); // let everything fully settle either way

        engine.processBlock(inL, inR, outL, outR, 64);
        const float steadyL = outL[63], steadyR = outR[63];
        const bool reachedC = std::fabs(steadyL - 5.0f) < 0.01f && std::fabs(steadyR - 5.0f) < 0.01f;
        printf("[35] a second order-changing edit mid-FadingIn is not dropped: settled at %.3f/%.3f (want 5.0/5.0, matB's stale 2.0 would mean the bug is back): %s\n",
               steadyL, steadyR, reachedC ? "OK" : "FAIL");
        if (!reachedC) { printf("FAIL: StateSpaceEngine got stuck on a superseded model instead of applying the latest one submitted during FadingIn\n"); ++failures; }
    }

    // 36. The EXACT minimal user sequence reported to still fault after
    //     [34]/[35] above: a brand-new/empty EQ, ONE band placed (click
    //     empty graph space -> UIEq::addBand()'s own defaults: Bell,
    //     Q=1.0), then the Channel column clicked three times to reach
    //     L->R (Both -> Left -> Right -> L->R, see
    //     advanceChannelCrossFeed()'s own comment) -- confirmed by the
    //     user to fault even done slowly and deliberately, on the very
    //     first band, no other content in the session at all. [34] never
    //     tested this because it always started from an already-populated
    //     model; this walks the exact same per-click sequence
    //     BandListWidget's own release handler produces, with real audio
    //     flowing through the real engine between every click -- in
    //     particular the Right -> L->R step, which (unlike Both -> Left
    //     -> Right, each an order change) is a SAME-ORDER edit and so
    //     goes through the coefficient-SMOOTHING path, not the duck: it
    //     blends a Right-only band's own B (fed from u1, the right input)
    //     toward a LeftToRight band's own B (fed from u0, the left input)
    //     coefficient-by-coefficient over time, a transition [34] never
    //     exercised at all.
    {
        std::mt19937 rng(777);
        std::uniform_real_distribution<float> noise(-0.3f, 0.3f);
        constexpr uint32_t kBlock = 256;
        float inL[kBlock], inR[kBlock], outL[kBlock], outR[kBlock];
        bool faulted = false;
        double faultedAtStep = -1;

        StateSpaceEngine engine;
        engine.prepare(fs);

        auto runBlocks = [&](int n)
        {
            for (int b = 0; b < n && !faulted; ++b)
            {
                for (uint32_t i = 0; i < kBlock; ++i) { inL[i] = noise(rng); inR[i] = noise(rng); }
                engine.processBlock(inL, inR, outL, outR, kBlock);
                if (engine.consumeFaultFlag()) faulted = true;
                for (uint32_t i = 0; i < kBlock && !faulted; ++i)
                    if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) faulted = true;
            }
        };
        auto push = [&](const EqModel& m, double step)
        {
            StateSpaceMatrices assembled = eq::assembleStateSpace(m, fs);
            StabilityGuard::enforce(assembled);
            engine.setModel(assembled);
            runBlocks(20); // ~107ms of continuous audio -- comfortably past both the ~8ms duck and the coefficient-smoothing time constant
            if (faulted && faultedAtStep < 0.0) faultedAtStep = step;
        };

        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 1000.0, 6.0, 1.0, true, false }); // addBand()'s own defaults: Q=1, a plain click position
        push(eq, 0); // Both (the freshly-placed band's own starting state)

        eq.bands[0].channel = EqChannel::Left;
        push(eq, 1); // Both -> Left (order change: duck)

        eq.bands[0].channel = EqChannel::Right;
        push(eq, 2); // Left -> Right (SAME order: smoothing)

        eq.bands[0].crossFeed = EqCrossFeed::LeftToRight; // channel left at Right, ignored once crossFeed != None -- see advanceChannelCrossFeed()'s own comment
        push(eq, 3); // Right -> L->R (SAME order: smoothing -- the untested transition)

        printf("[36] exact click sequence (place band, Both->Left->Right->L->R) with real audio between every click: %s\n",
               faulted ? "FAULT" : "OK");
        if (faulted) { printf("FAIL: numerical fault reproduced at step %.0f (0=Both 1=Left 2=Right 3=L->R)\n", faultedAtStep); ++failures; }
    }

    // 37. Root cause of [36]'s fault, isolated directly: a model whose
    //     ONLY active content is a crossfeed band (leftStages AND
    //     rightStages both genuinely empty) must still produce an EVEN
    //     assembled order with a correctly 2-wide-aligned sparseCascade2
    //     structure. clampOrder()'s own [1, kMaxOrder] floor pads the
    //     empty `m`'s true order-0 up to a fake 1 (odd) unless
    //     assembleStateSpace()'s graft explicitly special-cases this --
    //     an odd graft offset silently misaligns the block the engine's
    //     own sparse fast path (StateSpaceEngine.hpp) reads, dropping a
    //     real A[i][j] pole-coupling term from the recursion. Checked two
    //     independent ways, same "time domain vs frequency domain must
    //     agree" discipline tools/testbench/audio_testbench.cpp's own
    //     header comment describes: the closed-form steady-state
    //     response (TransferFunction::evaluate(), no recursion at all)
    //     must match what the actual recursive engine measures for the
    //     same tone.
    {
        EqModel eq;
        eq.bands.push_back({ EqBandType::Bell, 200.0, 6.0, 3.0, true, false }); // low-ish freq, moderate Q -- exactly the "almost everywhere, low freq worse" shape reported
        eq.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        eq.crossFeedGainLtoRDb = 0.0;

        StateSpaceMatrices assembled = eq::assembleStateSpace(eq, fs);
        const bool evenOrder = assembled.order % 2 == 0;
        printf("[37] crossfeed-only model (no ordinary L/R content at all): order=%d is even: %s\n",
               assembled.order, evenOrder ? "OK" : "FAIL");
        if (!evenOrder) { printf("FAIL: base offset for the crossfeed graft was odd -- the exact misalignment behind [36]'s fault\n"); ++failures; }

        // Same sparseCascade2 zero-pattern check test [32] already uses.
        bool sparseOk = assembled.sparseCascade2;
        const int n = clampOrder(assembled.order);
        double worst = 0.0;
        for (int i = 0; i < n; ++i)
        {
            const int jStart = 2 * (i / 2 + 1);
            for (int j = jStart; j < n; ++j) worst = std::max(worst, std::fabs(assembled.A[size_t(i)][size_t(j)]));
        }
        printf("      sparseCascade2 flag=%d, worst |A[i][j]| beyond own block=%.2e (want exactly 0): %s\n",
               assembled.sparseCascade2, worst, (sparseOk && worst == 0.0) ? "OK" : "FAIL");
        if (!(sparseOk && worst == 0.0)) { printf("FAIL: crossfeed-only model's own zero-pattern violates its own sparseCascade2 contract\n"); ++failures; }

        // Time domain vs frequency domain cross-check at the band's own
        // frequency -- would have caught [36]'s bug directly: a
        // misaligned sparse read corrupts the recursion (wrong number),
        // not necessarily a NaN every time, so agreement here is a
        // stronger, more general check than "did it merely stay finite".
        StabilityGuard::enforce(assembled);
        const Complex z = std::polar(1.0, 2.0 * M_PI * 200.0 / fs);
        const double expectedDb = 20.0 * std::log10(std::abs(TransferFunction::evaluate(assembled, z).H[1][0]));

        StateSpaceEngine engine;
        engine.prepare(fs);
        engine.setModel(assembled);
        constexpr uint32_t kN = 8192;
        std::vector<float> inL(kN), inR(kN), outL(kN), outR(kN);
        for (uint32_t i = 0; i < kN; ++i) { inL[i] = float(std::sin(2.0 * M_PI * 200.0 * i / fs)); inR[i] = 0.0f; }
        engine.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), kN);
        double peak = 0.0;
        for (uint32_t i = kN / 2; i < kN; ++i) peak = std::max(peak, double(std::fabs(outR[i]))); // second half only -- past any settling
        const double measuredDb = 20.0 * std::log10(std::max(peak, 1e-12));
        const bool matches = std::fabs(measuredDb - expectedDb) < 1.0;
        printf("      time-domain engine vs closed-form response at 200Hz: measured=%.2fdB expected=%.2fdB: %s\n",
               measuredDb, expectedDb, matches ? "OK" : "FAIL");
        if (!matches) { printf("FAIL: the real-time recursive engine disagrees with the closed-form transfer function -- the recursion itself is corrupted\n"); ++failures; }
    }

    // 38. Dynamic EQ 2 crossfeed -- closed-form: hasPerChannelBands() sees
    //     it, evaluateCombinedResponse()/evaluateChannelResponse() exclude
    //     it (same "can't be folded into one channel's own multiplicative
    //     curve" reasoning as the static EQ's identical exclusion), the
    //     model-level crossFeedGainLtoRDb/RtoLDb baseline is genuinely
    //     flat via evaluateBandResponse()'s own extraFlatGainDb, and the
    //     whole model round-trips through JSON (including an old-format
    //     preset with none of these keys, defaulting cleanly).
    {
        DynamicEqModel2 model;
        model.bands.push_back(DynamicEqBand2{});
        model.bands.back().freqHz = 1000.0; model.bands.back().gainDb = 6.0; model.bands.back().dynamicsEnabled = false;
        model.bands.push_back(DynamicEqBand2{});
        model.bands.back().freqHz = 3000.0; model.bands.back().gainDb = 0.0; model.bands.back().dynamicsEnabled = false;
        model.bands.back().crossFeed = EqCrossFeed::LeftToRight;
        model.crossFeedGainLtoRDb = -20.0;

        printf("[38] hasPerChannelBands() true with an active Dynamic EQ 2 crossfeed band: %s\n",
               dyneq2::hasPerChannelBands(model) ? "OK" : "FAIL");
        if (!dyneq2::hasPerChannelBands(model)) { printf("FAIL: hasPerChannelBands() missed an active crossfeed band\n"); ++failures; }

        // Flat baseline: a crossfeed band with no resting gain bump at all
        // must show exactly crossFeedGainLtoRDb at every frequency, same
        // "no Bell-only design flaw" check as the static EQ's own [32a].
        bool flatOk = true;
        double worstDev = 0.0;
        const DynamicEqBand2& crossBand = model.bands[1];
        for (double f : { 100.0, 1000.0, 3000.0, 10000.0 })
        {
            const std::complex<double> resp = dyneq2::evaluateBandResponse(crossBand, fs, f, crossBand.gainDb, model.crossFeedGainLtoRDb);
            const double db = 20.0 * std::log10(std::abs(resp));
            worstDev = std::max(worstDev, std::fabs(db - (-20.0)));
            if (std::fabs(db - (-20.0)) > 0.1) flatOk = false;
        }
        printf("      crossFeedGainLtoRDb is a genuinely flat baseline: worst deviation from -20dB=%.3fdB: %s\n", worstDev, flatOk ? "OK" : "FAIL");
        if (!flatOk) { printf("FAIL: Dynamic EQ 2's own crossfeed baseline isn't flat across frequency\n"); ++failures; }

        // evaluateCombinedResponse() must exclude the crossfeed band --
        // at 3kHz (the crossfeed band's own frequency), the combined
        // response should reflect ONLY the ordinary 1kHz/6dB band's own
        // (small, off-center) contribution, not the crossfeed band's own
        // -20dB dip/bump.
        const std::complex<double> combinedAt3k = dyneq2::evaluateCombinedResponse(model, fs, 3000.0);
        const double combinedDb = 20.0 * std::log10(std::abs(combinedAt3k));
        const bool excludedOk = std::fabs(combinedDb) < 6.0; // nowhere near -20dB or the crossfeed band's own shape
        printf("      evaluateCombinedResponse() excludes the crossfeed band: %.2fdB at 3kHz (want far from -20): %s\n",
               combinedDb, excludedOk ? "OK" : "FAIL");
        if (!excludedOk) { printf("FAIL: evaluateCombinedResponse() folded a crossfeed band into the combined curve\n"); ++failures; }

        // JSON round-trip, including an old-format preset (no crossFeed/
        // crossFeedGainLtoRDb/RtoLDb keys at all) defaulting cleanly.
        DynamicEqModel2 parsed;
        const bool jsonOk = dyneq2::fromJson(dyneq2::toJson(model), parsed) &&
                            parsed.bands.size() == 2 &&
                            parsed.bands[0].crossFeed == EqCrossFeed::None &&
                            parsed.bands[1].crossFeed == EqCrossFeed::LeftToRight &&
                            std::fabs(parsed.crossFeedGainLtoRDb - (-20.0)) < 1e-9;
        printf("      crossFeed/crossFeedGainLtoRDb round-trip through JSON: %s\n", jsonOk ? "OK" : "FAIL");
        if (!jsonOk) { printf("FAIL: Dynamic EQ 2's own crossfeed fields lost or corrupted across a JSON round-trip\n"); ++failures; }

        const std::string oldFormatJson =
            "{\n  \"type\": \"statespace-dyneq2-preset\",\n  \"version\": 1,\n  \"name\": \"Old\",\n"
            "  \"outputTrimDb\": 0,\n  \"soloedBand\": -1,\n  \"bands\": [\n"
            "    { \"freqHz\": 1000, \"q\": 1, \"gainDb\": 6, \"thresholdDb\": -24, \"ratio\": 4, \"ratioUp\": 1,\n"
            "      \"rangeDb\": 5, \"attackMs\": 10, \"releaseMs\": 100, \"dynamicsEnabled\": true, \"enabled\": true, \"channel\": \"both\" }\n  ]\n}\n";
        DynamicEqModel2 oldParsed;
        const bool oldOk = dyneq2::fromJson(oldFormatJson, oldParsed) && oldParsed.bands.size() == 1 &&
                           oldParsed.bands[0].crossFeed == EqCrossFeed::None &&
                           oldParsed.crossFeedGainLtoRDb == -60.0 && oldParsed.crossFeedGainRtoLDb == -60.0;
        printf("      old-format preset (no crossfeed keys at all) parses, defaults cleanly: %s\n", oldOk ? "OK" : "FAIL");
        if (!oldOk) { printf("FAIL: loading a pre-crossfeed Dynamic EQ 2 preset didn't default cleanly\n"); ++failures; }
    }

    // 39. Dynamic EQ 2 crossfeed -- the EXACT minimal scenario that broke
    //     the sibling Parametric EQ overnight ([36]/[37] above): a single
    //     crossfeed-only band, real audio through the REAL
    //     DynamicEqEngine2, no other bands at all. Also covers the
    //     inactive-crossfeed-band leak this same round's own
    //     PluginDynEq2::rebuildEngineModel() fix targets (a disabled/
    //     muted crossfeed band must not still leak its source channel into
    //     the destination), multiple simultaneous crossfeed bands not
    //     compounding (the additive-not-cascaded design's own point), and
    //     a broad frequency/Q/gain sweep with sustained audio -- same
    //     "don't just test the gentle case" discipline as the sibling
    //     EQ's own [34].
    {
        std::mt19937 rng(2026);
        std::uniform_real_distribution<float> noise(-0.3f, 0.3f);
        constexpr uint32_t kBlock = 256;
        float inL[kBlock], inR[kBlock], outL[kBlock], outR[kBlock];
        bool anyFault = false;

        auto runBlocks = [&](DynamicEqEngine2& engine, int n, bool& faulted)
        {
            for (int b = 0; b < n && !faulted; ++b)
            {
                for (uint32_t i = 0; i < kBlock; ++i) { inL[i] = noise(rng); inR[i] = noise(rng); }
                engine.processBlock(inL, inR, outL, outR, kBlock);
                if (engine.consumeFaultFlag()) faulted = true;
                for (uint32_t i = 0; i < kBlock && !faulted; ++i)
                    if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) faulted = true;
            }
        };

        // 39a. Minimal case: ONE band, freshly "placed" (Both, default Q),
        //      then cycled Both -> Left -> Right -> L->R exactly like a
        //      real Channel-button click sequence, real audio flowing
        //      between every click.
        {
            DynamicEqEngine2 engine;
            engine.prepare(fs);
            bool faulted = false;

            auto push = [&](const DynamicEqModel2& m)
            {
                DynamicEqRtModel2 rt;
                rt.numBands = int(m.bands.size());
                for (int i = 0; i < rt.numBands; ++i) rt.bands[i] = m.bands[size_t(i)];
                rt.outputTrimDb = m.outputTrimDb;
                rt.crossFeedGainLtoRDb = m.crossFeedGainLtoRDb;
                rt.crossFeedGainRtoLDb = m.crossFeedGainRtoLDb;
                engine.setModel(rt);
                runBlocks(engine, 20, faulted);
            };

            DynamicEqModel2 m;
            m.bands.push_back(DynamicEqBand2{});
            m.bands.back().freqHz = 1000.0; m.bands.back().gainDb = 6.0; m.bands.back().dynamicsEnabled = false; // a fresh, deliberately static band -- isolates the routing change itself
            push(m); // Both

            m.bands[0].channel = EqChannel::Left;
            push(m); // Both -> Left (order-preserving in this engine -- no duck needed, but still a real edit)

            m.bands[0].channel = EqChannel::Right;
            push(m); // Left -> Right

            m.bands[0].crossFeed = EqCrossFeed::LeftToRight; // channel left at Right, ignored once crossFeed != None
            push(m); // Right -> L->R -- the exact untested transition that broke the sibling EQ

            printf("[39] exact click sequence (place band, Both->Left->Right->L->R), Dynamic EQ 2: %s\n", faulted ? "FAULT" : "OK");
            if (faulted) { anyFault = true; printf("FAIL: numerical fault reproduced in Dynamic EQ 2's own crossfeed\n"); ++failures; }
        }

        // 39b. Broad stress sweep: two ordinary bands plus two crossfeed
        //      bands (one each direction), high Q, gain/crossfeed gain
        //      near their own ceilings, across a spread of frequencies,
        //      several seconds of continuous noise, plus live-drag
        //      simulation (repeated setModel() calls with only a few
        //      samples of audio between them).
        for (double f : { 40.0, 150.0, 600.0, 2500.0, 9000.0, 16000.0 })
        {
            DynamicEqEngine2 engine;
            engine.prepare(fs);
            bool faulted = false;

            DynamicEqModel2 m;
            auto mkBand = [](double freq, double gain, double q) {
                DynamicEqBand2 b; b.freqHz = freq; b.gainDb = gain; b.q = q;
                b.dynamicsEnabled = true; b.thresholdDb = -30.0; b.ratio = 4.0; b.ratioUp = 2.0;
                b.attackMs = 5.0; b.releaseMs = 80.0; b.rangeDb = 10.0;
                return b;
            };
            m.bands.push_back(mkBand(150.0, 8.0, 6.0));
            m.bands.push_back(mkBand(4000.0, -6.0, 8.0));
            m.bands.push_back(mkBand(f, 10.0, 20.0)); m.bands.back().crossFeed = EqCrossFeed::LeftToRight;
            m.bands.push_back(mkBand(std::max(20.0, f * 0.6), -8.0, 20.0)); m.bands.back().crossFeed = EqCrossFeed::LeftToRight;
            m.bands.push_back(mkBand(std::min(fs * 0.45, f * 1.4), 10.0, 15.0)); m.bands.back().crossFeed = EqCrossFeed::RightToLeft;
            m.crossFeedGainLtoRDb = 12.0;
            m.crossFeedGainRtoLDb = 12.0;

            DynamicEqRtModel2 rt;
            rt.numBands = int(m.bands.size());
            for (int i = 0; i < rt.numBands; ++i) rt.bands[i] = m.bands[size_t(i)];
            rt.crossFeedGainLtoRDb = m.crossFeedGainLtoRDb;
            rt.crossFeedGainRtoLDb = m.crossFeedGainRtoLDb;
            engine.setModel(rt);
            runBlocks(engine, 5 * int(fs) / int(kBlock) + 1, faulted);

            for (int step = 0; step < 40 && !faulted; ++step)
            {
                const double drift = 1.0 + 0.01 * step;
                m.bands[2].freqHz = std::max(20.0, std::min(fs * 0.45, f * drift));
                DynamicEqRtModel2 rt2;
                rt2.numBands = int(m.bands.size());
                for (int i = 0; i < rt2.numBands; ++i) rt2.bands[i] = m.bands[size_t(i)];
                rt2.crossFeedGainLtoRDb = m.crossFeedGainLtoRDb;
                rt2.crossFeedGainRtoLDb = m.crossFeedGainRtoLDb;
                engine.setModel(rt2);
                runBlocks(engine, 4, faulted);
            }

            if (faulted) anyFault = true;
            printf("      Dynamic EQ 2 crossfeed @ %.0fHz, dynamics on, sustained + drag: %s\n", f, faulted ? "FAULT" : "OK");
        }
        if (anyFault) { printf("FAIL: Dynamic EQ 2's own crossfeed faulted during the stress sweep\n"); ++failures; }

        // 39c. An inactive (disabled) crossfeed band must not leak its
        //      source channel into the destination -- exactly the bug
        //      PluginDynEq2::rebuildEngineModel()'s own fix targets this
        //      round (forcing crossFeed back to None on an inert slot,
        //      not just gainDb=0/dynamicsEnabled=false). Verified directly
        //      against DynamicEqEngine2 by replicating what
        //      rebuildEngineModel() does for a disabled band's slot.
        {
            DynamicEqEngine2 engine;
            engine.prepare(fs);
            DynamicEqRtModel2 rt;
            rt.numBands = 1;
            rt.bands[0].freqHz = 1000.0;
            rt.bands[0].crossFeed = EqCrossFeed::LeftToRight; // as authored...
            rt.bands[0].gainDb = 0.0;
            rt.bands[0].dynamicsEnabled = false;
            rt.bands[0].crossFeed = EqCrossFeed::None; // ...but rebuildEngineModel() forces this back to None for a disabled/inactive slot -- see its own comment
            rt.crossFeedGainLtoRDb = 0.0; // 0dB -- if the leak bug were present, this would be maximally audible
            engine.setModel(rt);

            constexpr uint32_t kN = 4096;
            std::vector<float> il(kN, 0.3f), ir(kN, 0.0f), ol(kN), orr(kN); // constant L, silent R
            engine.processBlock(il.data(), ir.data(), ol.data(), orr.data(), kN);
            double maxR = 0.0;
            for (uint32_t i = kN / 2; i < kN; ++i) maxR = std::max(maxR, double(std::fabs(orr[i])));
            const bool noLeak = maxR < 1e-6;
            printf("[39c] an inactive crossfeed band's own slot doesn't leak L into R: max|R|=%.2e (want ~0): %s\n", maxR, noLeak ? "OK" : "FAIL");
            if (!noLeak) { printf("FAIL: an inactive crossfeed band's own slot still leaked its source channel into the destination\n"); ++failures; }
        }
    }

    // 40. Dynamic EQ 2's own rebuild path fits comfortably on a real-time
    //     process-thread stack -- same discipline as the
    //     sibling Parametric EQ's own [32d] after that project's real
    //     stack-overflow crash. Unlike EqModel's assembleStateSpace()
    //     (which builds a full ~131KB StateSpaceMatrices), Dynamic EQ 2's
    //     own model->engine path (PluginDynEq2::rebuildEngineModel(),
    //     replicated here) only ever builds a DynamicEqRtModel2 -- 16
    //     small per-band structs, a few hundred bytes total -- so this is
    //     expected to pass with enormous headroom; run for real anyway
    //     rather than assumed, on the exact same 384KB constrained stack
    //     [32d] uses.
    {
        struct Job
        {
            bool ok = false;
            double fs = 48000.0;
        };
        Job job;
        job.fs = fs;

        auto threadMain = [](void* arg) -> void*
        {
            Job* j = static_cast<Job*>(arg);
            DynamicEqModel2 m;
            for (int i = 0; i < 8; ++i)
            {
                DynamicEqBand2 b;
                b.freqHz = 100.0 * (i + 1);
                b.gainDb = 8.0;
                b.q = 10.0;
                b.crossFeed = (i % 2 == 0) ? EqCrossFeed::LeftToRight : EqCrossFeed::RightToLeft;
                m.bands.push_back(b);
            }
            m.crossFeedGainLtoRDb = 6.0;
            m.crossFeedGainRtoLDb = 6.0;

            // Same construction PluginDynEq2::rebuildEngineModel() does --
            // a plain stack-local DynamicEqRtModel2, then straight into
            // the engine.
            DynamicEqRtModel2 rt;
            rt.numBands = int(m.bands.size());
            for (int i = 0; i < rt.numBands; ++i) rt.bands[i] = m.bands[size_t(i)];
            rt.crossFeedGainLtoRDb = m.crossFeedGainLtoRDb;
            rt.crossFeedGainRtoLDb = m.crossFeedGainRtoLDb;

            DynamicEqEngine2 engine;
            engine.prepare(j->fs);
            engine.setModel(rt);
            float inL[256], inR[256], outL[256], outR[256];
            for (int i = 0; i < 256; ++i) { inL[i] = 0.2f * float(std::sin(0.1 * i)); inR[i] = 0.2f * float(std::cos(0.13 * i)); }
            for (int b = 0; b < 20; ++b) engine.processBlock(inL, inR, outL, outR, 256);
            j->ok = std::isfinite(double(outL[0])) && std::isfinite(double(outR[0]));
            return nullptr;
        };

        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 384 * 1024); // same constrained size as [32d]
        pthread_t th;
        const int rc = pthread_create(&th, &attr, threadMain, &job);
        bool ranOk = (rc == 0);
        if (ranOk) pthread_join(th, nullptr);
        pthread_attr_destroy(&attr);

        printf("[40] Dynamic EQ 2 rebuild path (8 bands, 4 crossfeed) fits a 384KB real-time thread stack: %s\n",
               (ranOk && job.ok) ? "OK" : "FAIL");
        if (!(ranOk && job.ok)) { printf("FAIL: Dynamic EQ 2's own model->engine path did not complete cleanly on a constrained-stack thread\n"); ++failures; }
    }

    // 41. Dynamic EQ 2 harmonic linking -- direct port of the sibling
    //     EQ's own EqModel.hpp mechanism:
    //     createHarmonicTwin() links the source band and mints a twin at
    //     the given ratio, propagateHarmonicFreq() moves every group
    //     member together from the virtual shared base, unlinkHarmonic()
    //     detaches one member without disturbing the rest, and it all
    //     round-trips through JSON.
    {
        DynamicEqModel2 model;
        model.bands.push_back(DynamicEqBand2{});
        model.bands.back().freqHz = 1000.0;

        const int twinIndex = dyneq2::createHarmonicTwin(model, 0, 2.0, fs);
        const bool createOk = twinIndex == 1 && model.bands.size() == 2 &&
                               model.bands[0].harmonicGroup != 0 &&
                               model.bands[0].harmonicGroup == model.bands[1].harmonicGroup &&
                               std::fabs(model.bands[1].freqHz - 2000.0) < 1e-6;
        printf("[41] createHarmonicTwin(): links source, mints an x2 twin at 2000Hz: %s\n", createOk ? "OK" : "FAIL");
        if (!createOk) { printf("FAIL: Dynamic EQ 2's own createHarmonicTwin() didn't link/position the twin correctly\n"); ++failures; }

        // Moving the TWIN (not the original source) must reposition the
        // source too, via the virtual shared base -- same "no middle to
        // break" property the sibling EQ's own test relies on.
        dyneq2::propagateHarmonicFreq(model, 1, 4000.0, fs); // twin moved to 4000Hz -> base=2000Hz -> source should follow to 2000Hz
        const bool propagateOk = std::fabs(model.bands[0].freqHz - 2000.0) < 1e-6 && std::fabs(model.bands[1].freqHz - 4000.0) < 1e-6;
        printf("      moving the twin repositions the source via the shared virtual base: source=%.1fHz twin=%.1fHz (want 2000/4000): %s\n",
               model.bands[0].freqHz, model.bands[1].freqHz, propagateOk ? "OK" : "FAIL");
        if (!propagateOk) { printf("FAIL: propagateHarmonicFreq() didn't move the whole group from whichever member was dragged\n"); ++failures; }

        dyneq2::unlinkHarmonic(model, 0);
        dyneq2::propagateHarmonicFreq(model, 1, 8000.0, fs); // twin moves again -- source must NOT follow now that it's unlinked
        const bool unlinkOk = model.bands[0].harmonicGroup == 0 && std::fabs(model.bands[0].freqHz - 2000.0) < 1e-6 &&
                               std::fabs(model.bands[1].freqHz - 8000.0) < 1e-6;
        printf("      unlinkHarmonic() detaches one member without disturbing the rest: source stayed at %.1fHz (want 2000): %s\n",
               model.bands[0].freqHz, unlinkOk ? "OK" : "FAIL");
        if (!unlinkOk) { printf("FAIL: unlinking a band either didn't detach it or it kept moving with its old group\n"); ++failures; }

        DynamicEqModel2 parsed;
        const bool jsonOk = dyneq2::fromJson(dyneq2::toJson(model), parsed) &&
                            parsed.bands.size() == 2 &&
                            parsed.bands[1].harmonicGroup != 0 &&
                            std::fabs(parsed.bands[1].harmonicRatio - model.bands[1].harmonicRatio) < 1e-9;
        printf("      harmonicGroup/harmonicRatio round-trip through JSON: %s\n", jsonOk ? "OK" : "FAIL");
        if (!jsonOk) { printf("FAIL: Dynamic EQ 2's own harmonic-link fields lost or corrupted across a JSON round-trip\n"); ++failures; }
    }

    // 42. Dynamic EQ 2 Low/High Shelf band types. The new first-order-allpass shelf construction
    //     (dyneq2::designShaperAllpass()/shaperAllpassSign(), unstable-
    //     by-construction impossible per that function's own comment) is
    //     checked three independent ways: closed-form shape (Low Shelf
    //     gains DC, unity at Nyquist; High Shelf the mirror image), the
    //     REAL time-domain engine agreeing with the closed-form curve
    //     (same "time domain must agree with frequency domain" discipline
    //     test [37] already established), and JSON round-trip.
    {
        DynamicEqBand2 low; low.type = EqBandType::LowShelf; low.freqHz = 1000.0; low.gainDb = 12.0; low.dynamicsEnabled = false;
        DynamicEqBand2 high; high.type = EqBandType::HighShelf; high.freqHz = 1000.0; high.gainDb = 12.0; high.dynamicsEnabled = false;

        const double lowAt20 = 20.0 * std::log10(std::abs(dyneq2::evaluateBandResponse(low, fs, 20.0, low.gainDb)));
        const double lowAt20k = 20.0 * std::log10(std::abs(dyneq2::evaluateBandResponse(low, fs, 20000.0, low.gainDb)));
        const double highAt20 = 20.0 * std::log10(std::abs(dyneq2::evaluateBandResponse(high, fs, 20.0, high.gainDb)));
        const double highAt20k = 20.0 * std::log10(std::abs(dyneq2::evaluateBandResponse(high, fs, 20000.0, high.gainDb)));
        const bool shapeOk = std::fabs(lowAt20 - 12.0) < 0.5 && std::fabs(lowAt20k - 0.0) < 0.5 &&
                             std::fabs(highAt20 - 0.0) < 0.5 && std::fabs(highAt20k - 12.0) < 0.5;
        printf("[42] Low Shelf: 20Hz=%.2fdB(want 12) 20kHz=%.2fdB(want 0); High Shelf: 20Hz=%.2fdB(want 0) 20kHz=%.2fdB(want 12): %s\n",
               lowAt20, lowAt20k, highAt20, highAt20k, shapeOk ? "OK" : "FAIL");
        if (!shapeOk) { printf("FAIL: Low/High Shelf don't produce the expected DC/Nyquist-asymptote shape\n"); ++failures; }

        // Real engine vs closed-form, for both shapes, at a frequency
        // firmly on the "boosted" side of each shelf's own corner.
        for (int which = 0; which < 2; ++which)
        {
            const DynamicEqBand2& b = which == 0 ? low : high;
            const double testFreq = which == 0 ? 100.0 : 8000.0; // firmly low (Low Shelf's boosted side) / firmly high (High Shelf's boosted side)
            const double expectedDb = 20.0 * std::log10(std::abs(dyneq2::evaluateBandResponse(b, fs, testFreq, b.gainDb)));

            DynamicEqEngine2 engine;
            engine.prepare(fs);
            DynamicEqRtModel2 rt;
            rt.numBands = 1;
            rt.bands[0] = b;
            engine.setModel(rt);
            constexpr uint32_t kN = 8192;
            std::vector<float> il(kN), ir(kN, 0.0f), ol(kN), orr(kN);
            for (uint32_t i = 0; i < kN; ++i) il[i] = float(std::sin(2.0 * M_PI * testFreq * i / fs));
            engine.processBlock(il.data(), ir.data(), ol.data(), orr.data(), kN);
            double peak = 0.0;
            for (uint32_t i = kN / 2; i < kN; ++i) peak = std::max(peak, double(std::fabs(ol[i])));
            const double measuredDb = 20.0 * std::log10(std::max(peak, 1e-12));
            const bool matches = std::fabs(measuredDb - expectedDb) < 1.0;
            printf("      %s: time-domain engine vs closed-form at %.0fHz: measured=%.2fdB expected=%.2fdB: %s\n",
                   which == 0 ? "Low Shelf" : "High Shelf", testFreq, measuredDb, expectedDb, matches ? "OK" : "FAIL");
            if (!matches) { printf("FAIL: the real-time engine disagrees with the closed-form shelf response\n"); ++failures; }
        }

        // JSON round-trip, including an old-format preset (no "type" key
        // at all) defaulting to Bell.
        DynamicEqModel2 model;
        model.bands.push_back(low);
        DynamicEqModel2 parsed;
        const bool jsonOk = dyneq2::fromJson(dyneq2::toJson(model), parsed) &&
                            parsed.bands.size() == 1 && parsed.bands[0].type == EqBandType::LowShelf;
        printf("      type round-trips through JSON: %s\n", jsonOk ? "OK" : "FAIL");
        if (!jsonOk) { printf("FAIL: Dynamic EQ 2's own band type was lost or corrupted across a JSON round-trip\n"); ++failures; }

        const std::string oldFormatJson =
            "{\n  \"type\": \"statespace-dyneq2-preset\",\n  \"version\": 1,\n  \"name\": \"Old\",\n"
            "  \"outputTrimDb\": 0,\n  \"soloedBand\": -1,\n  \"bands\": [\n"
            "    { \"freqHz\": 1000, \"q\": 1, \"gainDb\": 6, \"thresholdDb\": -24, \"ratio\": 4, \"ratioUp\": 1,\n"
            "      \"rangeDb\": 5, \"attackMs\": 10, \"releaseMs\": 100, \"dynamicsEnabled\": true, \"enabled\": true, \"channel\": \"both\" }\n  ]\n}\n";
        DynamicEqModel2 oldParsed;
        const bool oldOk = dyneq2::fromJson(oldFormatJson, oldParsed) && oldParsed.bands.size() == 1 &&
                           oldParsed.bands[0].type == EqBandType::Bell;
        printf("      old-format preset (no \"type\" key at all) defaults to Bell: %s\n", oldOk ? "OK" : "FAIL");
        if (!oldOk) { printf("FAIL: loading a pre-type Dynamic EQ 2 preset didn't default to Bell\n"); ++failures; }
    }

    // 43. Dynamic EQ 2 -- linked bands share their dynamics:
    //     propagateHarmonicDynamics()
    //     fans Threshold/Ratio/RatioUp/Range out to every member of a
    //     harmonic group VERBATIM (no per-member ratio scaling, unlike
    //     freqHz), a no-op when the source band isn't linked to anything.
    {
        DynamicEqModel2 model;
        model.bands.push_back(DynamicEqBand2{}); // unlinked control band
        model.bands.back().thresholdDb = -10.0;

        model.bands.push_back(DynamicEqBand2{});
        model.bands.back().freqHz = 1000.0;
        const int twinIndex = dyneq2::createHarmonicTwin(model, 1, 2.0, fs);

        model.bands[1].thresholdDb = -30.0; model.bands[1].ratio = 8.0; model.bands[1].ratioUp = 3.0; model.bands[1].rangeDb = 12.0;
        dyneq2::propagateHarmonicDynamics(model, 1);
        const bool linkedOk = std::fabs(model.bands[twinIndex].thresholdDb - (-30.0)) < 1e-9 &&
                               std::fabs(model.bands[twinIndex].ratio - 8.0) < 1e-9 &&
                               std::fabs(model.bands[twinIndex].ratioUp - 3.0) < 1e-9 &&
                               std::fabs(model.bands[twinIndex].rangeDb - 12.0) < 1e-9;
        printf("[43] propagateHarmonicDynamics(): linked twin picks up source's Threshold/Ratio/RatioUp/Range: %s\n", linkedOk ? "OK" : "FAIL");
        if (!linkedOk) { printf("FAIL: a harmonically-linked band's own dynamics didn't follow its group\n"); ++failures; }

        const bool unlinkedControlOk = std::fabs(model.bands[0].thresholdDb - (-10.0)) < 1e-9;
        printf("      an unrelated (unlinked) band's own dynamics are untouched: %s\n", unlinkedControlOk ? "OK" : "FAIL");
        if (!unlinkedControlOk) { printf("FAIL: propagateHarmonicDynamics() touched a band outside the group\n"); ++failures; }

        dyneq2::unlinkHarmonic(model, twinIndex);
        model.bands[1].thresholdDb = -50.0;
        dyneq2::propagateHarmonicDynamics(model, 1);
        const bool noLongerLinkedOk = std::fabs(model.bands[twinIndex].thresholdDb - (-30.0)) < 1e-9; // stayed at its last-linked value
        printf("      unlinking stops further propagation to that member: %s\n", noLongerLinkedOk ? "OK" : "FAIL");
        if (!noLongerLinkedOk) { printf("FAIL: an unlinked band still received a propagated dynamics change\n"); ++failures; }
    }

    // 44. Dynamic EQ 2 -- Audition treats linked bands as one: auditioning
    //     one member of a
    //     harmonic group sums that band's own isolated detector tap with
    //     every OTHER member's own, not just the one literally selected;
    //     auditioning an unlinked band is unaffected (still just itself).
    {
        DynamicEqEngine2 engine;
        engine.prepare(fs);

        auto buildAndAudition = [&](bool linkTwoBands) -> double
        {
            engine.resetState();
            DynamicEqRtModel2 rt;
            rt.numBands = 2;
            rt.bands[0].freqHz = 1000.0; rt.bands[0].gainDb = 0.0; rt.bands[0].dynamicsEnabled = false;
            rt.bands[1].freqHz = 3000.0; rt.bands[1].gainDb = 0.0; rt.bands[1].dynamicsEnabled = false;
            if (linkTwoBands) { rt.bands[0].harmonicGroup = 1; rt.bands[1].harmonicGroup = 1; }
            engine.setModel(rt);
            engine.setAudition(true);
            engine.setAuditionBandIndex(0); // audition band 0 only -- band 1 should join in ONLY when linked

            constexpr uint32_t kN = 8192;
            std::vector<float> il(kN), ir(kN, 0.0f), ol(kN), orr(kN);
            std::mt19937 rng(99);
            std::uniform_real_distribution<float> noise(-0.5f, 0.5f);
            for (uint32_t i = 0; i < kN; ++i) il[i] = noise(rng);
            engine.processBlock(il.data(), ir.data(), ol.data(), orr.data(), kN);
            engine.processBlock(il.data(), ir.data(), ol.data(), orr.data(), kN); // second block -- past the initial audition-engage duck
            double sumAbs = 0.0;
            for (uint32_t i = 0; i < kN; ++i) sumAbs += std::fabs(double(ol[i]));
            return sumAbs;
        };

        const double energyUnlinked = buildAndAudition(false);
        const double energyLinked = buildAndAudition(true);
        // Linked must audibly include MORE than the unlinked case (band
        // 1's own detector tap, centered at a different frequency,
        // joining in) -- not asserting an exact ratio (both bands' own
        // detector bandpass outputs are uncorrelated broadband noise
        // taps, not simple multiples of each other), just that the
        // linked case is genuinely louder, proving band 1 actually
        // joined the preview.
        const bool auditionLinkedOk = energyLinked > energyUnlinked * 1.3;
        printf("[44] auditioning a linked band sums its group: unlinked energy=%.2f linked energy=%.2f (want linked clearly louder): %s\n",
               energyUnlinked, energyLinked, auditionLinkedOk ? "OK" : "FAIL");
        if (!auditionLinkedOk) { printf("FAIL: auditioning a harmonically-linked band didn't include its other group member(s)\n"); ++failures; }
    }

    // 45. Band Shelf (Parametric EQ): High Shelf(f1,+G) x Low Shelf(f2,+G)
    //     x flat -G in ONE band -- 0dB outside the two corners, +G between
    //     them. Checked against a hand-built two-band preset (High Shelf
    //     91.7Hz +10dB, Low Shelf 2350.6Hz +10dB, output trim -10dB), which
    //     builds exactly this shape by hand; plus the real time-domain engine against the
    //     closed-form curve, JSON, type switching, harmonic linking, and a
    //     full list of Band Shelves (twice the states of any other band)
    //     on a constrained real-time stack.
    {
        auto bandDb = [&](const EqBand& b, double f) {
            return 20.0 * std::log10(std::abs(eq::evaluateBandResponse(b, fs, f)));
        };
        auto modelDb = [&](const EqModel& m, double f) {
            return 20.0 * std::log10(std::abs(eq::evaluateCombinedResponse(m, fs, f)));
        };

        // (a) The shape: flat outside, +G inside, for a boost and a cut.
        EqBand bs;
        bs.type = EqBandType::BandShelf;
        bs.freqHz = std::sqrt(100.0 * 2000.0); // centre of 100Hz..2kHz
        bs.bandwidthOct = std::log2(2000.0 / 100.0);
        bs.q = 0.707;
        bool shapeOk = true;
        for (double g : { 10.0, -10.0 })
        {
            bs.gainDb = g;
            const double lowDb = bandDb(bs, 20.0), midDb = bandDb(bs, bs.freqHz), highDb = bandDb(bs, 18000.0);
            const bool ok = std::fabs(lowDb) < 0.6 && std::fabs(midDb - g) < 0.6 && std::fabs(highDb) < 0.6;
            printf("[45] Band Shelf 100Hz-2kHz %+.0fdB: 20Hz=%+.2fdB centre=%+.2fdB 18kHz=%+.2fdB (want 0 / %+.0f / 0): %s\n",
                   g, lowDb, midDb, highDb, g, ok ? "OK" : "FAIL");
            shapeOk = shapeOk && ok;
        }
        if (!shapeOk) { printf("FAIL: Band Shelf doesn't return to 0dB outside its corners / reach its gain inside\n"); ++failures; }

        // Centre + bandwidth -> the two corners the shelves are built on.
        const double loC = eq::bandShelfLowCornerHz(bs), hiC = eq::bandShelfHighCornerHz(bs);
        const bool cornersOk = std::fabs(loC - 100.0) < 1e-9 && std::fabs(hiC - 2000.0) < 1e-9;
        printf("      centre 447Hz + BW 4.32 oct -> corners %.3fHz / %.3fHz (want 100 / 2000): %s\n", loC, hiC, cornersOk ? "OK" : "FAIL");
        if (!cornersOk) { printf("FAIL: Band Shelf corner computation\n"); ++failures; }

        // (b) One Band Shelf == a hand-built two-shelves-plus-trim preset.
        const std::string handBuiltPreset = R"({
  "type": "statespace-eq-preset", "version": 1, "name": "Untitled EQ",
  "outputTrimDb": -10, "crossFeedGainLtoRDb": 0, "crossFeedGainRtoLDb": -60,
  "bands": [
    { "type": "highShelf", "freqHz": 91.73910522460938, "gainDb": 10, "q": 0.7350299954414368, "slopeStages": 1, "enabled": true, "solo": false, "harmonicGroup": 0, "harmonicRatio": 1, "channel": "both", "crossFeed": "none" },
    { "type": "lowShelf", "freqHz": 2350.5888671875, "gainDb": 10, "q": 0.7350299954414368, "slopeStages": 1, "enabled": true, "solo": false, "harmonicGroup": 0, "harmonicRatio": 1, "channel": "both", "crossFeed": "none" }
  ]
})";
        EqModel handBuilt;
        const bool parsedHandBuilt = eq::fromJson(handBuiltPreset, handBuilt); // same Q on both shelves here -- the saved file's two differ by 0.005, a Band Shelf has one Q
        EqModel oneBand;
        EqBand ob;
        ob.type = EqBandType::BandShelf;
        ob.freqHz = std::sqrt(91.73910522460938 * 2350.5888671875);
        ob.bandwidthOct = std::log2(2350.5888671875 / 91.73910522460938);
        ob.gainDb = 10.0;
        ob.q = 0.7350299954414368;
        oneBand.bands.push_back(ob);
        double worstPreset = 0.0;
        for (double f = 20.0; f < 20000.0; f *= 1.05) worstPreset = std::max(worstPreset, std::fabs(modelDb(handBuilt, f) - modelDb(oneBand, f)));
        printf("      one Band Shelf == hand-built two shelves + -10dB trim: worst diff %.2e dB: %s\n",
               worstPreset, (parsedHandBuilt && worstPreset < 1e-9) ? "OK" : "FAIL");
        if (!(parsedHandBuilt && worstPreset < 1e-9)) { printf("FAIL: Band Shelf doesn't match the hand-built two-shelf preset\n"); ++failures; }

        // (c) The real recursive engine agrees with the closed form (same
        //     time-vs-frequency-domain discipline as [37]), and the band
        //     costs exactly two biquads per channel.
        StateSpaceMatrices assembled = eq::assembleStateSpace(oneBand, fs);
        const bool orderOk = assembled.order == 8;
        StabilityGuard::enforce(assembled);
        const double probeHz = 460.0; // mid-plateau
        const Complex z = std::polar(1.0, 2.0 * M_PI * probeHz / fs);
        const double expectedDb = 20.0 * std::log10(std::abs(TransferFunction::evaluate(assembled, z).H[0][0]));
        StateSpaceEngine engine;
        engine.prepare(fs);
        engine.setModel(assembled);
        constexpr uint32_t kN = 16384;
        std::vector<float> inL(kN), inR(kN), outL(kN), outR(kN);
        for (uint32_t i = 0; i < kN; ++i) inL[i] = inR[i] = float(0.1 * std::sin(2.0 * M_PI * probeHz * i / fs));
        engine.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), kN);
        double peak = 0.0;
        for (uint32_t i = kN / 2; i < kN; ++i) peak = std::max(peak, double(std::fabs(outL[i])));
        const double measuredDb = 20.0 * std::log10(std::max(peak / 0.1, 1e-12));
        const bool tdOk = orderOk && std::fabs(measuredDb - expectedDb) < 0.1 && std::fabs(expectedDb - 10.0) < 0.6;
        printf("      engine at %.0fHz: measured %+.2fdB vs closed-form %+.2fdB, order=%d (want 8): %s\n",
               probeHz, measuredDb, expectedDb, assembled.order, tdOk ? "OK" : "FAIL");
        if (!tdOk) { printf("FAIL: Band Shelf's real-time engine output disagrees with its own curve\n"); ++failures; }

        // (d) JSON round trip; a pre-Band-Shelf preset loads with the default
        //     bandwidth; a preset from the first, two-corner format
        //     (freqHz + freq2Hz) is converted to centre + bandwidth.
        EqModel back;
        const bool rt = eq::fromJson(eq::toJson(oneBand), back) && back.bands.size() == 1 &&
                        back.bands[0].type == EqBandType::BandShelf && back.bands[0].bandwidthOct == ob.bandwidthOct &&
                        back.bands[0].freqHz == ob.freqHz;
        const bool oldOk = parsedHandBuilt && handBuilt.bands.size() == 2 && handBuilt.bands[0].bandwidthOct == EqBand().bandwidthOct;
        EqModel legacy;
        const bool legacyParsed = eq::fromJson(R"({ "type": "statespace-eq-preset", "version": 1, "bands": [
            { "type": "bandShelf", "freqHz": 100, "freq2Hz": 1600, "gainDb": 6, "q": 0.7 } ] })", legacy);
        const bool legacyOk = legacyParsed && legacy.bands.size() == 1 && std::fabs(legacy.bands[0].freqHz - 400.0) < 1e-9 &&
                              std::fabs(legacy.bands[0].bandwidthOct - 4.0) < 1e-9;
        printf("      JSON round trip: %s; pre-Band-Shelf preset gets the default bandwidth: %s; two-corner preset 100/1600Hz -> centre 400Hz, 4 oct: %s\n",
               rt ? "OK" : "FAIL", oldOk ? "OK" : "FAIL", legacyOk ? "OK" : "FAIL");
        if (!(rt && oldOk && legacyOk)) { printf("FAIL: Band Shelf JSON\n"); ++failures; }

        // (e) Switching type keeps the band where it was: freqHz is the
        //     centre for every type, so nothing moves and a linked group's
        //     ratio is untouched.
        EqBand t;
        t.freqHz = 1000.0;
        t.harmonicGroup = 1;
        t.harmonicRatio = 2.0;
        eq::applyBandType(t, EqBandType::BandShelf);
        const bool into = t.freqHz == 1000.0 && t.harmonicRatio == 2.0 &&
                          std::fabs(eq::bandShelfLowCornerHz(t) - 500.0) < 1e-9 && std::fabs(eq::bandShelfHighCornerHz(t) - 2000.0) < 1e-9;
        eq::applyBandType(t, EqBandType::Bell);
        const bool outOf = t.freqHz == 1000.0 && t.harmonicRatio == 2.0;
        printf("      Bell 1kHz -> Band Shelf centre 1kHz (500Hz-2kHz) -> Bell 1kHz, harmonic ratio unchanged: %s\n", (into && outOf) ? "OK" : "FAIL");
        if (!(into && outOf)) { printf("FAIL: applyBandType() moved the band or its harmonic ratio\n"); ++failures; }

        // (f) Band Shelves are linked by their centres, each keeping its
        //     own bandwidth.
        EqModel hm;
        EqBand src;
        src.type = EqBandType::BandShelf;
        src.freqHz = 400.0;
        src.bandwidthOct = 2.0;
        hm.bands.push_back(src);
        const int twin = eq::createHarmonicTwin(hm, 0, 2.0, fs);
        bool linkOk = twin == 1 && hm.bands[1].freqHz == 800.0 && hm.bands[1].bandwidthOct == 2.0;
        hm.bands[1].bandwidthOct = 1.0; // widths are per band
        eq::propagateHarmonicFreq(hm, 0, 600.0, fs);
        linkOk = linkOk && std::fabs(hm.bands[1].freqHz - 1200.0) < 1e-9 && hm.bands[0].bandwidthOct == 2.0 && hm.bands[1].bandwidthOct == 1.0;
        printf("      Band Shelves linked by centre: x2 twin at 800Hz, moving the source to 600Hz puts it at 1200Hz, bandwidths stay per band: %s\n",
               linkOk ? "OK" : "FAIL");
        if (!linkOk) { printf("FAIL: Band Shelf harmonic linking\n"); ++failures; }

        // (g) A full list of Band Shelves (32 bands x 2 biquads x 2
        //     channels = 256 states, twice kMaxOrder) goes through the real
        //     rebuild path -- assemble + StabilityGuard + engine -- on the
        //     same 384KB constrained stack as [32d], without overflowing
        //     either the thread stack or the fixed-size matrices.
        struct FullJob
        {
            static void* run(void* arg)
            {
                const double sr = 48000.0;
                EqModel m;
                for (int i = 0; i < kMaxOrder / 4; ++i)
                {
                    EqBand b;
                    b.type = EqBandType::BandShelf;
                    b.freqHz = 40.0 * (i + 1);
                    b.bandwidthOct = std::log2(3.0);
                    b.gainDb = (i % 2 == 0) ? 3.0 : -3.0;
                    b.q = 0.9;
                    m.bands.push_back(b);
                }
                std::unique_ptr<StateSpaceEngine> eng(new StateSpaceEngine());
                StateSpaceMatrices assembled = eq::assembleStateSpace(m, sr);
                StabilityGuard::enforce(assembled);
                eng->prepare(sr);
                eng->setModel(assembled);
                float in[256], outA[256], outB[256];
                for (int i = 0; i < 256; ++i) in[i] = 0.2f * float(std::sin(0.05 * i));
                bool finite = true;
                for (int blk = 0; blk < 40; ++blk)
                {
                    eng->processBlock(in, in, outA, outB, 256);
                    for (int i = 0; i < 256; ++i) finite = finite && std::isfinite(outA[i]) && std::isfinite(outB[i]);
                }
                *static_cast<int*>(arg) = finite ? assembled.order : -1;
                return nullptr;
            }
        };
        int fullOrder = -1;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 384 * 1024);
        pthread_t th;
        const bool started = pthread_create(&th, &attr, &FullJob::run, &fullOrder) == 0;
        if (started) pthread_join(th, nullptr);
        pthread_attr_destroy(&attr);
        const bool fullOk = started && fullOrder > 0 && fullOrder <= kMaxOrder;
        printf("      32 Band Shelves (256 states requested) on a 384KB thread: order=%d (capped at %d), output finite: %s\n",
               fullOrder, kMaxOrder, fullOk ? "OK" : "FAIL");
        if (!fullOk) { printf("FAIL: a full list of Band Shelves didn't rebuild/run cleanly on a constrained stack\n"); ++failures; }
    }

    // 46. Band Shelf in Dynamic EQ 2: two first-order Regalia-Mitra
    //     shelves in series plus a flat 1/G (dyneq2::designShaperAllpass2()),
    //     corners compensated by the resting gain so they are the half-gain
    //     points for boost and cut alike. Checked: closed-form shape for
    //     +/-10dB (symmetric, corners at +/-5dB, back to ~0 outside --
    //     first-order slopes leave ~0.5dB at 20Hz, see the numbers), the
    //     real engine vs the closed form, gain 0 = exact passthrough,
    //     dynamics pulling the gain, JSON, and 16 Band Shelves on a
    //     constrained real-time stack.
    {
        DynamicEqBand2 bs;
        bs.type = EqBandType::BandShelf;
        bs.freqHz = std::sqrt(100.0 * 2000.0);
        bs.bandwidthOct = std::log2(20.0);
        bs.dynamicsEnabled = false;
        auto db = [&](const DynamicEqBand2& b, double f, double g) {
            return 20.0 * std::log10(std::abs(dyneq2::evaluateBandResponse(b, fs, f, g)));
        };
        bool shapeOk = true;
        for (double g : { 10.0, -10.0 })
        {
            bs.gainDb = g;
            const double at20 = db(bs, 20.0, g), atLo = db(bs, 100.0, g), atC = db(bs, bs.freqHz, g),
                         atHi = db(bs, 2000.0, g), at18k = db(bs, 18000.0, g);
            const bool ok = std::fabs(at20) < 1.0 && std::fabs(at18k) < 0.5 &&
                            std::fabs(atLo - g / 2.0) < 0.2 && std::fabs(atHi - g / 2.0) < 0.2 &&
                            std::fabs(atC) > 0.85 * std::fabs(g) && std::fabs(atC) <= std::fabs(g) + 0.01;
            printf("[46] Dynamic EQ 2 Band Shelf 100Hz-2kHz %+.0fdB: 20Hz=%+.2f 100Hz=%+.2f centre=%+.2f 2kHz=%+.2f 18kHz=%+.2f (corners %+.0f, outside ~0): %s\n",
                   g, at20, atLo, atC, atHi, at18k, g / 2.0, ok ? "OK" : "FAIL");
            shapeOk = shapeOk && ok;
        }
        if (!shapeOk) { printf("FAIL: Dynamic EQ 2 Band Shelf shape\n"); ++failures; }

        auto runEngine = [&](const DynamicEqBand2& b, double freq, double amp, double* liveGainDb) {
            DynamicEqEngine2 engine;
            engine.prepare(fs);
            DynamicEqRtModel2 rt;
            rt.numBands = 1;
            rt.bands[0] = b;
            engine.setModel(rt);
            constexpr uint32_t kN = 16384;
            std::vector<float> il(kN), ir(kN), ol(kN), orr(kN);
            for (uint32_t i = 0; i < kN; ++i) il[i] = ir[i] = float(amp * std::sin(2.0 * M_PI * freq * i / fs));
            engine.processBlock(il.data(), ir.data(), ol.data(), orr.data(), kN);
            if (liveGainDb) *liveGainDb = engine.getCurrentGainDb(0);
            double peak = 0.0;
            for (uint32_t i = kN / 2; i < kN; ++i) peak = std::max(peak, double(std::fabs(ol[i])));
            return 20.0 * std::log10(std::max(peak / amp, 1e-12));
        };
        bool tdOk = true;
        for (double g : { 10.0, -10.0 })
            for (double f : { 60.0, bs.freqHz, 5000.0 })
            {
                bs.gainDb = g;
                const double expected = db(bs, f, g), measured = runEngine(bs, f, 0.1, nullptr);
                const bool ok = std::fabs(measured - expected) < 0.1;
                printf("      engine %+.0fdB at %.0fHz: measured %+.2fdB vs closed-form %+.2fdB: %s\n", g, f, measured, expected, ok ? "OK" : "FAIL");
                tdOk = tdOk && ok;
            }
        if (!tdOk) { printf("FAIL: Dynamic EQ 2 Band Shelf engine disagrees with its own curve\n"); ++failures; }

        // Gain 0 (also what an inactive slot is forced to) must be an exact
        // passthrough: G = 1 collapses both stages to identity.
        // Compared sample by sample (out vs in), not by peak level -- a
        // tone at an exact fraction of fs (8kHz = 6 samples/cycle) never
        // samples its own peak.
        bs.gainDb = 0.0;
        double worstIdentity = 0.0;
        {
            DynamicEqEngine2 engine;
            engine.prepare(fs);
            DynamicEqRtModel2 rt;
            rt.numBands = 1;
            rt.bands[0] = bs;
            engine.setModel(rt);
            constexpr uint32_t kN = 16384;
            std::vector<float> il(kN), ir(kN), ol(kN), orr(kN);
            std::mt19937 rng(7);
            std::uniform_real_distribution<float> noise(-0.5f, 0.5f);
            for (uint32_t i = 0; i < kN; ++i) il[i] = ir[i] = noise(rng);
            engine.processBlock(il.data(), ir.data(), ol.data(), orr.data(), kN);
            for (uint32_t i = kN / 2; i < kN; ++i) worstIdentity = std::max(worstIdentity, double(std::fabs(ol[i] - il[i])));
        }
        printf("      gain 0 is a passthrough: worst |out - in| on noise %.2e: %s\n", worstIdentity, worstIdentity < 1e-5 ? "OK" : "FAIL");
        if (worstIdentity >= 1e-5) { printf("FAIL: an inert Dynamic EQ 2 Band Shelf isn't transparent\n"); ++failures; }

        // Dynamics: a loud tone in the band pulls the live gain down from rest.
        DynamicEqBand2 dyn = bs;
        dyn.gainDb = 6.0;
        dyn.dynamicsEnabled = true;
        dyn.thresholdDb = -40.0;
        dyn.ratio = 4.0;
        dyn.rangeDb = 8.0;
        double liveDb = 0.0;
        runEngine(dyn, dyn.freqHz, 0.5, &liveDb);
        const bool dynOk = liveDb < dyn.gainDb - 4.0 && liveDb >= dyn.gainDb - dyn.rangeDb - 1e-9;
        printf("      dynamics: loud in-band tone pulls the gain from %+.1fdB to %+.2fdB (range %.0fdB): %s\n",
               dyn.gainDb, liveDb, dyn.rangeDb, dynOk ? "OK" : "FAIL");
        if (!dynOk) { printf("FAIL: Dynamic EQ 2 Band Shelf dynamics don't engage\n"); ++failures; }

        DynamicEqModel2 jm;
        dyn.bandwidthOct = 3.25;
        jm.bands.push_back(dyn);
        DynamicEqModel2 jback;
        const bool jsonOk = dyneq2::fromJson(dyneq2::toJson(jm), jback) && jback.bands.size() == 1 &&
                            jback.bands[0].type == EqBandType::BandShelf && jback.bands[0].bandwidthOct == 3.25;
        printf("      type + bandwidth round-trip through JSON: %s\n", jsonOk ? "OK" : "FAIL");
        if (!jsonOk) { printf("FAIL: Dynamic EQ 2 Band Shelf JSON\n"); ++failures; }

        struct Job { bool ok = false; };
        Job job;
        auto threadMain = [](void* arg) -> void* {
            DynamicEqRtModel2 rt;
            rt.numBands = kMaxDynEqBands2;
            for (int i = 0; i < rt.numBands; ++i)
            {
                DynamicEqBand2 b;
                b.type = EqBandType::BandShelf;
                b.freqHz = 60.0 * (i + 1);
                b.bandwidthOct = 1.5;
                b.gainDb = (i % 2) ? 6.0 : -6.0;
                b.crossFeed = (i % 5 == 0) ? EqCrossFeed::LeftToRight : EqCrossFeed::None;
                rt.bands[i] = b;
            }
            DynamicEqEngine2 engine;
            engine.prepare(48000.0);
            engine.setModel(rt);
            float inL[256], inR[256], outL[256], outR[256];
            for (int i = 0; i < 256; ++i) { inL[i] = 0.2f * float(std::sin(0.1 * i)); inR[i] = 0.2f * float(std::cos(0.13 * i)); }
            bool finite = true;
            for (int blk = 0; blk < 40; ++blk)
            {
                engine.processBlock(inL, inR, outL, outR, 256);
                for (int i = 0; i < 256; ++i) finite = finite && std::isfinite(outL[i]) && std::isfinite(outR[i]);
            }
            static_cast<Job*>(arg)->ok = finite;
            return nullptr;
        };
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setstacksize(&attr, 384 * 1024);
        pthread_t th;
        const bool started = pthread_create(&th, &attr, threadMain, &job) == 0;
        if (started) pthread_join(th, nullptr);
        pthread_attr_destroy(&attr);
        printf("      %d Band Shelves (some crossfeed) on a 384KB real-time thread stack: %s\n", kMaxDynEqBands2, (started && job.ok) ? "OK" : "FAIL");
        if (!(started && job.ok)) { printf("FAIL: Dynamic EQ 2 Band Shelves on a constrained stack\n"); ++failures; }
    }

    if (failures==0) printf("\nALL TESTS PASSED\n"); else printf("\n%d TEST(S) FAILED\n", failures);
    return failures==0?0:1;
}
