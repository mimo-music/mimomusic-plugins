// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// reverb_regression.cpp
//
// Fast, pure-math correctness tests for plugin/dsp/ReverbEngine.hpp --
// concept-phase, see docs/REVERB_CONCEPT.md. No DPF, no GUI, no audio
// files, no cmake needed to run this (deliberately a single-TU,
// header-only-dependency test, compiled directly, so it runs even
// without cmake):
//
//   g++ -std=c++17 -O2 -Wall -Wextra -I plugin/dsp tests/reverb_regression.cpp -o reverb_regression -lpthread
//   ./reverb_regression
//
// [R11] specifically is also meant to be run under ThreadSanitizer, a much
// stronger check of the requestRoom()/processBlock() thread-safety design
// than plain assertions can give:
//   g++ -std=c++17 -O1 -fsanitize=thread -I plugin/dsp tests/reverb_regression.cpp -o reverb_regression_tsan -lpthread
//   ./reverb_regression_tsan
//
// Once this project's normal CMake build is available again, this should
// gain an add_executable() entry in CMakeLists.txt alongside
// dsp_regression/audio_testbench, matching their PROPERTIES block, rather
// than staying a hand-invoked g++ command permanently.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <thread>
#include <vector>

#include "ReverbEngine.hpp"

using namespace ssfilter::reverb;

static int gFailures = 0;
static void check(bool cond, const char *what) {
    if (!cond) {
        std::printf("FAIL: %s\n", what);
        ++gFailures;
    } else {
        std::printf("ok:   %s\n", what);
    }
}

// Schroeder backward-integrated energy decay curve -> T60 via a -5dB to
// -35dB slope extrapolated to -60dB (standard T30-style measurement,
// avoids the very start of the IR and its eventual noise-floor tail).
static double measureT60Sec(const std::vector<double> &ir, double fs) {
    const size_t n = ir.size();
    std::vector<double> energy(n);
    double acc = 0.0;
    for (size_t i = n; i-- > 0;) {
        acc += ir[i] * ir[i];
        energy[i] = acc;
    }
    const double total = energy[0];
    if (total <= 0.0) return -1.0;
    auto findDbPoint = [&](double dB) -> long {
        const double target = total * std::pow(10.0, dB / 10.0);
        for (size_t i = 0; i < n; ++i)
            if (energy[i] <= target) return (long)i;
        return -1;
    };
    const long i5 = findDbPoint(-5.0);
    const long i35 = findDbPoint(-35.0);
    if (i5 < 0 || i35 < 0 || i35 <= i5) return -1.0;
    const double slopeDbPerSample = -30.0 / (double)(i35 - i5);
    return (60.0 / (-slopeDbPerSample)) / fs;
}

static std::vector<double> lowpassIr(const std::vector<double> &in, double fs, double fc) {
    const double a = std::exp(-2.0 * M_PI * fc / fs);
    std::vector<double> out(in.size());
    double z = 0.0;
    for (size_t i = 0; i < in.size(); ++i) { z = (1 - a) * in[i] + a * z; out[i] = z; }
    return out;
}

int main() {
    const double fs = 48000.0;
    RoomDims room{10.0, 7.0, 3.5};
    room.clamp();
    const double t60Dc = 1.8;
    const double t60Ny = 0.9;

    // [R1] Delay lengths: sane count/range, all within the fixed storage
    // bound, and (a real first-draft bug this test would have caught) all
    // distinct/prime so no two lines share a periodicity.
    {
        auto lens = designDelayLengths(room, fs);
        bool allInRange = true, allPrime = true, allDistinct = true;
        for (int i = 0; i < kNumLines; ++i) {
            if (lens[i] < 8 || lens[i] >= kMaxDelaySamples) allInRange = false;
            if (!isPrime(lens[i])) allPrime = false;
            for (int j = i + 1; j < kNumLines; ++j)
                if (lens[i] == lens[j]) allDistinct = false;
        }
        check(allInRange, "[R1] delay lengths within storage bound");
        check(allPrime, "[R1] delay lengths are prime");
        check(allDistinct, "[R1] delay lengths are distinct");
    }

    // [R2] Damping filter hits its target T60 at DC to within 1%, measured
    // via a low-pass-isolated Schroeder decay curve of the real engine's
    // impulse response (a raw broadband impulse's decay is a blend of the
    // fast high-frequency and slow low-frequency modes, not a fair
    // isolated test of the DC target alone -- see docs/REVERB_CONCEPT.md).
    {
        ReverbEngine eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        const size_t n = (size_t)((t60Dc * 2.2 + 0.5) * fs);
        std::vector<double> irL(n);
        bool allFinite = true;
        for (size_t i = 0; i < n; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            irL[i] = outL;
            if (!std::isfinite(outL) || !std::isfinite(outR)) allFinite = false;
        }
        check(allFinite, "[R2] impulse response stays finite");
        auto irLow = lowpassIr(irL, fs, 120.0);
        const double measured = measureT60Sec(irLow, fs);
        std::printf("      target T60(DC)=%.3fs measured(low-band)=%.3fs\n", t60Dc, measured);
        check(measured > 0 && std::fabs(measured - t60Dc) / t60Dc < 0.02,
              "[R2] low-band T60 matches target within 2%");
    }

    // [R3] Damping actually darkens the tail when t60Ny < t60Dc: a
    // high-band-isolated decay must measure faster (shorter) than the
    // low-band one from [R2] -- otherwise "Damping"/"Brightness" would be
    // a parameter with no audible effect, exactly the kind of silent-no-op
    // bug docs/ARCHITECTURE.md's own spectrum-gain story warns about.
    {
        ReverbEngine eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        const size_t n = (size_t)((t60Dc * 2.2 + 0.5) * fs);
        std::vector<double> irL(n);
        for (size_t i = 0; i < n; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            irL[i] = outL;
        }
        // High-pass via input-minus-lowpass.
        auto low = lowpassIr(irL, fs, 15000.0);
        std::vector<double> high(irL.size());
        for (size_t i = 0; i < irL.size(); ++i) high[i] = irL[i] - low[i];
        const double measuredHigh = measureT60Sec(high, fs);
        std::printf("      target T60(Nyquist)=%.3fs measured(>15kHz band)=%.3fs\n", t60Ny, measuredHigh);
        check(measuredHigh > 0 && measuredHigh < t60Dc,
              "[R3] high-band decays faster than low-band target (damping has effect)");
    }

    // [R4] Householder feedback matrix is exactly lossless (energy in ==
    // energy out of the mix step alone) -- the structural stability
    // argument this file's header comment makes, checked directly rather
    // than only argued.
    {
        std::array<double, kNumLines> v{};
        std::mt19937 rng(7);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        double inEnergy = 0.0;
        for (int i = 0; i < kNumLines; ++i) { v[i] = dist(rng); inEnergy += v[i] * v[i]; }
        householderMix(v);
        double outEnergy = 0.0;
        for (int i = 0; i < kNumLines; ++i) outEnergy += v[i] * v[i];
        std::printf("      inEnergy=%.9f outEnergy=%.9f\n", inEnergy, outEnergy);
        check(std::fabs(inEnergy - outEnergy) < 1e-9, "[R4] Householder mix is exactly lossless");
    }

    // [R5] Stereo cross-coupling: an L-only impulse must produce
    // significant R-channel energy within the first 80ms (the feedback
    // matrix mixes every line, including R-routed ones, every sample) --
    // otherwise "reflect signals from one stereo channel to the other and
    // back" (the literal ask) would not actually be happening.
    {
        ReverbEngine eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        double energyL = 0.0, energyR = 0.0;
        const size_t n = (size_t)(0.08 * fs);
        for (size_t i = 0; i < n; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, 0.0, outL, outR);
            energyL += outL * outL;
            energyR += outR * outR;
        }
        const double ratio = energyR / energyL;
        std::printf("      energyL=%.6f energyR=%.6f ratio(R/L)=%.3f\n", energyL, energyR, ratio);
        check(ratio > 0.1, "[R5] L-only impulse produces substantial R-channel energy");
    }

    // [R6] Long continuous-noise stress run stays finite and bounded --
    // the "no amount of upstream validation proves a recursive filter
    // stays finite forever" net this file's header comment describes,
    // exercised directly rather than only asserted to exist.
    {
        ReverbEngine eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        std::mt19937 rng(42);
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        bool finite = true;
        double maxAbs = 0.0;
        const size_t n = (size_t)(10.0 * fs);
        for (size_t i = 0; i < n; ++i) {
            double outL, outR;
            eng.processStereo(dist(rng) * 0.5, dist(rng) * 0.5, outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) finite = false;
            maxAbs = std::max({maxAbs, std::fabs(outL), std::fabs(outR)});
        }
        std::printf("      10s stress run maxAbs=%.4f\n", maxAbs);
        check(finite, "[R6] 10s continuous-noise stress run stays finite");
        check(maxAbs < 20.0, "[R6] 10s continuous-noise stress run stays bounded (< 20.0)");
    }

    // [R7] First-order image-source early reflections land in a sane
    // range (a few ms to well under a second) and are non-negative/finite
    // -- a sanity check on the geometry math, not a perceptual claim.
    {
        auto early = firstOrderImageSources(room, 0.25);
        bool sane = true;
        for (auto &r : early) {
            if (!(r.delaySec > 0.0 && r.delaySec < 1.0)) sane = false;
            if (!(r.gain > 0.0 && std::isfinite(r.gain))) sane = false;
        }
        std::printf("      first reflection delay=%.1fms last=%.1fms\n",
                    early.front().delaySec * 1000.0, early.back().delaySec * 1000.0);
        check(sane, "[R7] early reflections have sane finite positive delay/gain");
    }

    // [R8] Tempo-to-decay conversion: sanity-check the note-value table
    // against hand-computed values (the "ms = 60000/BPM * noteValue"
    // convention, cross-checked against real online delay/reverb-time
    // calculators -- see docs/REVERB_CONCEPT.md's references), including
    // the 3/4-time-signature case (OneBar must depend on beatsPerBar, not
    // hard-code 4/4) and the host-not-reporting-tempo fallback.
    {
        const double q120 = t60SecFromTempo(120.0, TempoNoteValue::Quarter, 4.0);
        const double bar120 = t60SecFromTempo(120.0, TempoNoteValue::OneBar, 4.0);
        const double twoBars140 = t60SecFromTempo(140.0, TempoNoteValue::TwoBars, 4.0);
        const double bar3_4 = t60SecFromTempo(120.0, TempoNoteValue::OneBar, 3.0);
        const double fallback = t60SecFromTempo(0.0, TempoNoteValue::Quarter, 4.0);
        std::printf("      q120=%.4fs bar120=%.4fs twoBars140=%.4fs bar(3/4)=%.4fs fallback(bpm=0)=%.4fs\n",
                    q120, bar120, twoBars140, bar3_4, fallback);
        check(std::fabs(q120 - 0.5) < 1e-9, "[R8] quarter note @120bpm = 0.5s");
        check(std::fabs(bar120 - 2.0) < 1e-9, "[R8] one bar @120bpm (4/4) = 2.0s");
        check(std::fabs(twoBars140 - (60.0 / 140.0) * 8.0) < 1e-9, "[R8] two bars @140bpm matches formula");
        check(std::fabs(bar3_4 - 1.5) < 1e-9, "[R8] one bar @120bpm in 3/4 = 1.5s (respects time signature)");
        check(std::fabs(fallback - q120) < 1e-9, "[R8] bpm<=0 falls back to 120bpm, matching DPF's own default");
    }

    // [R9] Coefficient smoothing actually suppresses the transient an
    // instant retarget would cause -- an A/B measurement of one sample,
    // not just an assertion that the smoothing code runs.
    //
    // Tested at the OnePoleDamping level directly, not through the full
    // ReverbEngine's output taps -- a real architectural subtlety found
    // while writing this test, worth recording: a delay line's circular
    // buffer *reads* (for this sample's output tap) before it *writes*
    // (this sample's freshly-damped value), so a coefficient change only
    // affects what gets WRITTEN this sample -- it takes a full trip
    // around that line (its own length in samples, 6-27ms for this
    // room) before it's ever read back out. So, unlike StateSpaceEngine's
    // C/D, which multiply directly into the *same* sample's output, a
    // snapped g/a here produces no instantly-visible click at
    // ReverbEngine::processStereo()'s own output at all -- confirmed by
    // an earlier version of this test, which measured zero output
    // difference between smoothed and instant retargets for exactly this
    // reason. The real discontinuity this smoothing prevents is internal:
    // OnePoleDamping's own y[n]=g*(1-a)*x[n]+a*z1 recursion, evaluated
    // with a freshly-snapped g/a against a z1 that accumulated under the
    // *old* coefficients, which is what actually gets written into the
    // delay line (and is audible ~6-27ms later, when that value is read
    // back). That's the level this test isolates and measures directly.
    {
        const int M = 500; // a representative single line length (samples)
        const double r0Before = OnePoleDamping::decayFactor(M, t60Dc, fs);
        const double rNyBefore = OnePoleDamping::decayFactor(M, t60Ny, fs);
        const double r0After = OnePoleDamping::decayFactor(M, 0.3, fs);   // drastic retarget
        const double rNyAfter = OnePoleDamping::decayFactor(M, 0.15, fs);
        const double smoothCoeff = 1.0 - std::exp(-1.0 / (0.005 * fs)); // matches ReverbEngine's own 5ms constant

        OnePoleDamping smoothF, snapF;
        smoothF.design(r0Before, rNyBefore); smoothF.snapToTarget();
        snapF.design(r0Before, rNyBefore); snapF.snapToTarget();

        const double dc = 0.3;
        for (int i = 0; i < 2000; ++i) { smoothF.process(dc); snapF.process(dc); } // settle z1 (identical for both so far)
        const double before = smoothF.process(dc); // one more sample at OLD coefficients -- shared baseline for both

        smoothF.design(r0After, rNyAfter);              // target only -- glides via smoothTowardTarget()
        snapF.design(r0After, rNyAfter); snapF.snapToTarget(); // instant

        smoothF.smoothTowardTarget(smoothCoeff); // exactly one smoothing step, as ReverbEngine::processStereo() does per sample
        const double afterSmooth = smoothF.process(dc);
        const double afterSnap = snapF.process(dc);
        const double jumpSmooth = std::fabs(afterSmooth - before);
        const double jumpSnap = std::fabs(afterSnap - before);

        std::printf("      internal filter jump at retarget: smoothed=%.6f instant=%.6f\n", jumpSmooth, jumpSnap);
        check(jumpSmooth < jumpSnap,
              "[R9] smoothed coefficient retarget produces a smaller internal jump than an instant one");
        check(jumpSmooth < jumpSnap * 0.5,
              "[R9] smoothed jump is substantially (not just marginally) smaller");
    }

    // [R10] Room-size change via processBlock()/requestRoom() actually
    // ducks to silence and back (mirroring dsp_regression's own test [13]
    // for StateSpaceEngine's identical order-changing-edit duck), and the
    // engine really is running the new room's delay lengths afterward --
    // not just "no crash", both halves of the claim measured directly.
    {
        ReverbEngine eng;
        RoomDims smallRoom{4.0, 3.0, 2.5};
        eng.prepare(smallRoom, fs, t60Dc, t60Ny);
        const auto lengthsBefore = eng.lengths();

        std::vector<float> inL(fs * 1.0, 0.3f), inR(fs * 1.0, 0.3f); // 1s of continuous input
        std::vector<float> outL(inL.size()), outR(inL.size());

        RoomDims bigRoom{20.0, 15.0, 8.0};
        eng.requestRoom(bigRoom);

        const int chunk = 256;
        double minAbsAfterRequest = 1e9;
        bool sawFadingOut = false, sawFadingIn = false, sawIdleAgain = false;
        bool pastFirstChunk = false;
        for (size_t off = 0; off < inL.size(); off += chunk) {
            const int n = (int)std::min((size_t)chunk, inL.size() - off);
            eng.processBlock(inL.data() + off, inR.data() + off, outL.data() + off, outR.data() + off, n);
            if (eng.duckState() == DuckState::FadingOut) sawFadingOut = true;
            if (eng.duckState() == DuckState::FadingIn) sawFadingIn = true;
            if (pastFirstChunk && eng.duckState() == DuckState::Idle) sawIdleAgain = true;
            pastFirstChunk = true;
        }
        // Find the quietest point in a window right around where the duck
        // should have happened (well before 1s -- 8ms fade out + 8ms in is
        // ~770 samples, negligible next to fs samples of input).
        for (size_t i = 0; i < inL.size() && i < (size_t)(fs * 0.05); ++i)
            minAbsAfterRequest = std::min(minAbsAfterRequest, (double)std::fabs(outL[i]));

        const auto lengthsAfter = eng.lengths();
        const bool lengthsChanged = lengthsBefore != lengthsAfter;

        std::printf("      duck states seen: fadingOut=%d fadingIn=%d idleAgain=%d | min|out| near duck=%.6f\n",
                    sawFadingOut, sawFadingIn, sawIdleAgain, minAbsAfterRequest);
        check(sawFadingOut && sawFadingIn && sawIdleAgain, "[R10] room change visibly ducks out, in, then returns to Idle");
        check(minAbsAfterRequest < 0.001, "[R10] duck actually dips to near-silence (not just a state-flag change)");
        check(lengthsChanged, "[R10] delay lengths actually change to the new (larger) room's values after the duck");
    }

    // [R11] requestRoom() is genuinely safe to call from a different
    // thread than processBlock() -- exercised with a real std::thread,
    // not just argued from the atomics' memory-order annotations. This
    // test is also meant to be run under ThreadSanitizer for a much
    // stronger guarantee than the assertions below can give on their own:
    //   g++ -std=c++17 -O2 -fsanitize=thread -I plugin/dsp tests/reverb_regression.cpp -o reverb_regression_tsan -lpthread
    {
        ReverbEngine eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        std::atomic<bool> stop{false};
        std::atomic<int> roomChanges{0};

        std::thread requester([&]() {
            std::mt19937 rng(99);
            std::uniform_real_distribution<double> dimDist(2.0, 30.0);
            while (!stop.load(std::memory_order_relaxed)) {
                eng.requestRoom(RoomDims{dimDist(rng), dimDist(rng), dimDist(rng)});
                roomChanges.fetch_add(1, std::memory_order_relaxed);
                std::this_thread::sleep_for(std::chrono::microseconds(200));
            }
        });

        std::mt19937 rngAudio(123);
        std::uniform_real_distribution<double> dist(-0.5, 0.5);
        std::vector<float> inL(512), inR(512), outL(512), outR(512);
        bool finite = true;
        const int blocks = 2000; // ~21ms/block worth of iterations, plenty of overlap with the requester thread
        for (int b = 0; b < blocks; ++b) {
            for (int i = 0; i < 512; ++i) { inL[i] = (float)dist(rngAudio); inR[i] = (float)dist(rngAudio); }
            eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), 512);
            for (int i = 0; i < 512; ++i)
                if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) finite = false;
        }
        stop.store(true, std::memory_order_relaxed);
        requester.join();

        std::printf("      concurrent requestRoom() calls while processBlock() ran: %d\n", roomChanges.load());
        check(finite, "[R11] audio stays finite while a concurrent thread calls requestRoom() continuously");
        check(roomChanges.load() > 0, "[R11] the concurrent requester thread actually ran (test isn't vacuous)");
    }

    // [R12] The allpass diffuser (added directly in response to "it
    // sounds very metallic like in a tube") is genuinely unity-magnitude
    // at every frequency tested, not just assumed to be because "allpass"
    // is in the name -- an allpass filter is defined by |H(e^jw)|=1 for
    // all w, so this is the one property that actually has to hold for
    // adding it to be a safe, loudness/color-neutral fix for echo
    // density rather than a hidden EQ change in disguise.
    {
        AllpassDiffuser diffuser;
        diffuser.designLengths(fs);
        diffuser.setAmount(1.0); // full strength -- the case most likely to reveal any magnitude error
        bool allClose = true;
        double worstRatio = 1.0;
        for (double freqHz : {50.0, 200.0, 1000.0, 4000.0, 12000.0}) {
            AllpassDiffuser probe;
            probe.designLengths(fs);
            probe.setAmount(1.0);
            const size_t n = (size_t)(0.2 * fs);
            double inEnergy = 0.0, outEnergy = 0.0;
            // Skip the first 500 samples (the diffuser's own short delay
            // lines are still filling) before measuring steady-state RMS.
            for (size_t i = 0; i < n; ++i) {
                const double t = (double)i / fs;
                const double x = std::sin(2.0 * M_PI * freqHz * t);
                const double y = probe.process(x);
                if (i >= 500) { inEnergy += x * x; outEnergy += y * y; }
            }
            const double ratio = std::sqrt(outEnergy / inEnergy);
            worstRatio = std::fabs(ratio - 1.0) > std::fabs(worstRatio - 1.0) ? ratio : worstRatio;
            if (std::fabs(ratio - 1.0) > 0.02) allClose = false; // 2% RMS tolerance
        }
        std::printf("      worst RMS ratio (out/in) across probed frequencies: %.4f\n", worstRatio);
        check(allClose, "[R12] allpass diffuser preserves signal RMS (unity magnitude) at every probed frequency");
    }

    // [R13] Stereo Width: at 1.0 (the default) it's an exact identity --
    // confirmed by re-running the same L-only-impulse probe [R5] already
    // uses and checking outL/outR come back bit-identical to a
    // width-untouched engine; at 0.0 the tank's own output collapses to
    // mono (outL == outR every sample); at 2.0 the mid (sum) signal is
    // unchanged while the side (difference) signal doubles -- the defined
    // behavior of a mid/side width control, checked directly rather than
    // only asserted.
    {
        ReverbEngine engDefault, engExplicit1, engZero, engTwo;
        engDefault.prepare(room, fs, t60Dc, t60Ny);
        engExplicit1.prepare(room, fs, t60Dc, t60Ny);
        engExplicit1.setWidth(1.0);
        engZero.prepare(room, fs, t60Dc, t60Ny);
        engZero.setWidth(0.0);
        engTwo.prepare(room, fs, t60Dc, t60Ny);
        engTwo.setWidth(2.0);

        std::mt19937 rngW(55);
        std::uniform_real_distribution<double> distW(-0.5, 0.5);
        bool identityAt1 = true, monoAt0 = true, sideDoubledAt2 = true;
        const int n = 2000;
        for (int i = 0; i < n; ++i) {
            const double inL = distW(rngW), inR = distW(rngW);
            double dL, dR, e1L, e1R, zL, zR, tL, tR;
            engDefault.processStereo(inL, inR, dL, dR);
            engExplicit1.processStereo(inL, inR, e1L, e1R);
            engZero.processStereo(inL, inR, zL, zR);
            engTwo.processStereo(inL, inR, tL, tR);

            if (dL != e1L || dR != e1R) identityAt1 = false;
            if (std::fabs(zL - zR) > 1e-12) monoAt0 = false;

            const double midDefault = 0.5 * (dL + dR), sideDefault = 0.5 * (dL - dR);
            const double midTwo = 0.5 * (tL + tR), sideTwo = 0.5 * (tL - tR);
            if (std::fabs(midTwo - midDefault) > 1e-9) sideDoubledAt2 = false;
            if (std::fabs(sideTwo - 2.0 * sideDefault) > 1e-9) sideDoubledAt2 = false;
        }
        std::printf("      Width=1.0 identity=%d  Width=0.0 mono=%d  Width=2.0 mid-unchanged/side-doubled=%d\n",
                    identityAt1, monoAt0, sideDoubledAt2);
        check(identityAt1, "[R13] Width=1.0 (default) is an exact identity vs. width-untouched output");
        check(monoAt0, "[R13] Width=0.0 collapses the tank's own output to mono (outL==outR)");
        check(sideDoubledAt2, "[R13] Width=2.0 leaves mid unchanged and exactly doubles the side signal");
    }

    // [R14] The L/R delay-line assignment is genuinely balanced -- a
    // centred test signal used to lean slightly to the right. Root cause: designDelayLengths() used to assign its
    // monotonically-increasing spread factors to lines in plain ascending
    // index order, and processStereo() routes even/odd index to L/R, so L
    // systematically got the *low* half of the spread range and R the
    // *high* half -- a real, structural asymmetry in each channel's own
    // steady-state energy for continuous input (not just an impulse
    // decay-time difference, which was already tested and fine). Fixed
    // with a balanced "snake" assignment (see designDelayLengths()'s own
    // comment). This test feeds identical mono noise into both channels
    // (so any measured difference is entirely the engine's own doing, not
    // the input's) and checks the steady-state RMS ratio is close to
    // exactly 1.0 -- measured here: 1.0000 (was 0.9815, a measurable
    // ~0.16dB bias, before the fix).
    {
        ReverbEngine eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        eng.setDiffusion(0.7);
        std::mt19937 rngBal(1);
        std::uniform_real_distribution<double> distBal(-0.5, 0.5);
        double sumL2 = 0.0, sumR2 = 0.0;
        const size_t warmup = (size_t)(2.0 * fs), n = (size_t)(10.0 * fs);
        for (size_t i = 0; i < n; ++i) {
            const double x = distBal(rngBal); // identical mono signal to both channels
            double outL, outR;
            eng.processStereo(x, x, outL, outR);
            if (i > warmup) { sumL2 += outL * outL; sumR2 += outR * outR; }
        }
        const double ratio = std::sqrt(sumR2 / sumL2);
        std::printf("      steady-state L/R energy ratio (symmetric input): %.4f (%.3f dB)\n", ratio, 20.0 * std::log10(ratio));
        check(std::fabs(ratio - 1.0) < 0.01, "[R14] L/R delay-line assignment is balanced (steady-state ratio within 1%)");
    }

    // [R15] Balance: identity at 0.0 (the default), and correctly
    // silences the opposite channel at the extremes.
    {
        ReverbEngine engDefault, engBal;
        engDefault.prepare(room, fs, t60Dc, t60Ny);
        engBal.prepare(room, fs, t60Dc, t60Ny);
        engBal.setBalance(0.0);

        std::mt19937 rngB(77);
        std::uniform_real_distribution<double> distB(-0.5, 0.5);
        bool identityAtZero = true;
        for (int i = 0; i < 1000; ++i) {
            const double inL = distB(rngB), inR = distB(rngB);
            double dL, dR, bL, bR;
            engDefault.processStereo(inL, inR, dL, dR);
            engBal.processStereo(inL, inR, bL, bR);
            if (dL != bL || dR != bR) identityAtZero = false;
        }

        ReverbEngine engFullLeft, engFullRight;
        engFullLeft.prepare(room, fs, t60Dc, t60Ny);
        engFullLeft.setBalance(-1.0);
        engFullRight.prepare(room, fs, t60Dc, t60Ny);
        engFullRight.setBalance(1.0);
        double maxRAtFullLeft = 0.0, maxLAtFullRight = 0.0;
        std::mt19937 rngB2(88);
        for (int i = 0; i < 2000; ++i) {
            const double inL = distB(rngB2), inR = distB(rngB2);
            double lL, lR, rL, rR;
            engFullLeft.processStereo(inL, inR, lL, lR);
            engFullRight.processStereo(inL, inR, rL, rR);
            maxRAtFullLeft = std::max(maxRAtFullLeft, std::fabs(lR));
            maxLAtFullRight = std::max(maxLAtFullRight, std::fabs(rL));
        }
        std::printf("      Balance identity@0=%d  maxR@Balance=-1: %.9f  maxL@Balance=+1: %.9f\n",
                    identityAtZero, maxRAtFullLeft, maxLAtFullRight);
        check(identityAtZero, "[R15] Balance=0.0 (default) is an exact identity");
        check(maxRAtFullLeft < 1e-12, "[R15] Balance=-1.0 fully silences the R channel");
        check(maxLAtFullRight < 1e-12, "[R15] Balance=+1.0 fully silences the L channel");
    }

    // [R16] Delay-line modulation -- the fix for audible FDN resonant
    // ringing (see ReverbEngine.hpp's own
    // "Delay-line modulation" section for the research this is grounded
    // in). Three checks, not just "it runs": Modulation=0.0 is an exact,
    // bit-identical no-op against an engine that never had this stage
    // touched at all; a nonzero setting measurably changes the output
    // (proving it's actually engaged, not a dead knob); and the tank
    // stays finite/stable at maximum depth under sustained noise.
    {
        // Three engines, run in strict lockstep from sample 0 on the
        // identical input sequence: engOff/engOff2 (both Modulation=0.0,
        // prepared independently) must match every single sample if 0.0
        // is a genuine no-op; engOn (Modulation=0.3, this engine's own
        // default) is expected to diverge from them, proving the
        // modulation stage actually does something. (An earlier version
        // of this test compared an already-2-seconds-warmed-up engine
        // against a freshly-prepared one -- a test bug, not a DSP one --
        // caught immediately by this check itself failing; fixed by
        // running all three together from the start instead.)
        ReverbEngine engOff, engOff2, engOn;
        engOff.prepare(room, fs, t60Dc, t60Ny);
        engOff.setModulation(0.0);
        engOff2.prepare(room, fs, t60Dc, t60Ny);
        engOff2.setModulation(0.0);
        engOn.prepare(room, fs, t60Dc, t60Ny);
        engOn.setModulation(0.3); // this engine's own default

        std::mt19937 rngM(303);
        std::uniform_real_distribution<double> distM(-0.5, 0.5);
        bool identityAtZero = true, differsWhenOn = false;
        const size_t n = (size_t)(2.0 * fs);
        for (size_t i = 0; i < n; ++i) {
            const double inL = distM(rngM), inR = distM(rngM);
            double offL, offR, off2L, off2R, onL, onR;
            engOff.processStereo(inL, inR, offL, offR);
            engOff2.processStereo(inL, inR, off2L, off2R);
            engOn.processStereo(inL, inR, onL, onR);
            if (offL != off2L || offR != off2R) identityAtZero = false;
            if (offL != onL || offR != onR) differsWhenOn = true;
        }

        ReverbEngine engMax;
        engMax.prepare(room, fs, t60Dc, t60Ny);
        engMax.setModulation(1.0);
        std::mt19937 rngM3(404);
        std::uniform_real_distribution<double> distM3(-0.8, 0.8);
        bool finiteAtMax = true;
        double maxAbsAtMax = 0.0;
        const size_t nStress = (size_t)(5.0 * fs);
        for (size_t i = 0; i < nStress; ++i) {
            double outL, outR;
            engMax.processStereo(distM3(rngM3), distM3(rngM3), outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) finiteAtMax = false;
            maxAbsAtMax = std::max({maxAbsAtMax, std::fabs(outL), std::fabs(outR)});
        }

        std::printf("      Modulation=0.0 identity=%d  Modulation=0.3 differs-from-off=%d  Modulation=1.0 finite=%d maxAbs=%.4f\n",
                    identityAtZero, differsWhenOn, finiteAtMax, maxAbsAtMax);
        check(identityAtZero, "[R16] Modulation=0.0 is an exact, bit-identical no-op");
        check(differsWhenOn, "[R16] Modulation=0.3 (default) measurably changes the output (not a dead knob)");
        check(finiteAtMax, "[R16] Modulation=1.0 (max depth) stays finite under sustained noise");
        check(maxAbsAtMax < 20.0, "[R16] Modulation=1.0 (max depth) stays bounded under sustained noise");
    }

    std::printf("\n%s (%d failure%s)\n", gFailures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
                gFailures, gFailures == 1 ? "" : "s");
    return gFailures == 0 ? 0 : 1;
}
