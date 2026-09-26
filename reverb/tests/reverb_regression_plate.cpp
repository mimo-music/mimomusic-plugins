// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// reverb_regression_plate.cpp
//
// Correctness tests for plugin/dsp/ReverbEnginePlate.hpp -- a genuinely
// different topology from reverb_regression.cpp/reverb_regression_dense.cpp's
// own N-line Householder FDN (see ReverbEnginePlate.hpp's own header
// comment for why), so most checks here are new, not just [R#]-renamed:
// there is no Householder matrix to prove lossless, no per-line array to
// sanity-check, and a genuinely new concern this topology introduced and
// had to be measured, not assumed -- steady-state loudness parity with
// the other two models (see [P4]).
//
//   g++ -std=c++17 -O2 -Wall -Wextra -I plugin/dsp tests/reverb_regression_plate.cpp -o reverb_regression_plate -lpthread
//   ./reverb_regression_plate

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <thread>
#include <vector>

#include "ReverbEngine.hpp"       // for a same-settings loudness-parity reference in [P4]
#include "ReverbEnginePlate.hpp"

using namespace ssfilter::reverb;
using namespace ssfilter::reverb::plate;

static int gFailures = 0;
static void check(bool cond, const char *what) {
    if (!cond) {
        std::printf("FAIL: %s\n", what);
        ++gFailures;
    } else {
        std::printf("ok:   %s\n", what);
    }
}

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

    // [P1] The 12 designed segment lengths (4 mono input-diffusion
    // stages, then tank A's mod-allpass/delay1/decay-diffusion2-allpass/
    // delay2, then tank B's) are all within the fixed storage bound.
    // NOT required to be prime here, unlike the FDN models' own lines or
    // this engine's own first draft: these lengths are Dattorro's own
    // published, proven values scaled by room/sample-rate (see
    // ReverbEnginePlate.hpp's own header comment) -- rounding them to
    // the nearest prime would perturb his own exact ratios between
    // segments and the output tap offsets (also derived proportionally
    // from his own numbers) for no benefit, since anti-periodicity here
    // comes from his own topology (two structurally different tank
    // lengths, modulation, multi-tap output), not from prime lengths.
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        auto lens = eng.lengths();
        bool allInRange = true;
        for (int l : lens) {
            if (l < 8 || l >= kMaxDelaySamples) allInRange = false;
        }
        std::printf("      lengths (inDiff): %d %d %d %d | (tankA): %d %d %d %d | (tankB): %d %d %d %d\n",
                    lens[0], lens[1], lens[2], lens[3], lens[4], lens[5], lens[6], lens[7], lens[8], lens[9], lens[10], lens[11]);
        check(allInRange, "[P1] segment lengths within storage bound");
    }

    // [P2] Damping filter hits its target T60 at DC to within 2% --
    // measured the same way [R2] measures the FDN's own (low-pass-
    // isolated Schroeder decay curve of a real impulse response).
    {
        ReverbEnginePlate eng;
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
        check(allFinite, "[P2] impulse response stays finite");
        auto irLow = lowpassIr(irL, fs, 120.0);
        const double measured = measureT60Sec(irLow, fs);
        std::printf("      target T60(DC)=%.3fs measured(low-band)=%.3fs\n", t60Dc, measured);
        check(measured > 0 && std::fabs(measured - t60Dc) / t60Dc < 0.02,
              "[P2] low-band T60 matches target within 2% (same bar as [R2] -- kGroupDelayCorrection in ReverbEnginePlate.hpp is what gets this topology's own allpass-heavy loop to hit it)");
    }

    // [P3] Damping actually darkens the tail when t60Ny < t60Dc.
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        const size_t n = (size_t)((t60Dc * 2.2 + 0.5) * fs);
        std::vector<double> irL(n);
        for (size_t i = 0; i < n; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            irL[i] = outL;
        }
        auto low = lowpassIr(irL, fs, 15000.0);
        std::vector<double> high(irL.size());
        for (size_t i = 0; i < irL.size(); ++i) high[i] = irL[i] - low[i];
        const double measuredHigh = measureT60Sec(high, fs);
        std::printf("      target T60(Nyquist)=%.3fs measured(>15kHz band)=%.3fs\n", t60Ny, measuredHigh);
        check(measuredHigh > 0 && measuredHigh < t60Dc,
              "[P3] high-band decays faster than low-band target (damping has effect)");
    }

    // [P4] Loudness parity with `reverb` (the 8-line FDN) at the same
    // Decay Time/Damping -- direct occasion for this check: an earlier
    // draft of this engine measured ~4.3x (~12.7dB) QUIETER steady-state
    // RMS than reverb's own for identical settings (a structural
    // consequence of this topology's own much-longer, much-less-
    // frequent loop turnover -- see ReverbEnginePlate.hpp's own
    // kOutputMakeupGain comment), which would have made switching
    // between the two shared-GUI plugins silently change the effective
    // wet loudness at every setting. Fixed with a measured makeup gain
    // constant; this test is what keeps that fix from silently
    // regressing.
    {
        ReverbEngine reference;
        reference.prepare(room, fs, t60Dc, t60Ny);
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);

        std::mt19937 rngRef(42), rngPlate(42);
        std::uniform_real_distribution<double> dist(-0.5, 0.5);
        double sumSqRef = 0.0, sumSqPlate = 0.0;
        const int warmup = 96000, n = 48000 * 6;
        for (int i = 0; i < n; ++i) {
            const double xRef = dist(rngRef), xPlate = dist(rngPlate);
            double refL, refR, plL, plR;
            reference.processStereo(xRef, xRef, refL, refR);
            eng.processStereo(xPlate, xPlate, plL, plR);
            if (i > warmup) {
                sumSqRef += refL * refL + refR * refR;
                sumSqPlate += plL * plL + plR * plR;
            }
        }
        const double rmsRef = std::sqrt(sumSqRef / (2.0 * (n - warmup)));
        const double rmsPlate = std::sqrt(sumSqPlate / (2.0 * (n - warmup)));
        const double ratioDb = 20.0 * std::log10(rmsPlate / rmsRef);
        std::printf("      steady-state RMS: reverb(FDN-8)=%.5f  reverb_plate=%.5f  (%.2f dB)\n", rmsRef, rmsPlate, ratioDb);
        check(std::fabs(ratioDb) < 3.0, "[P4] reverb_plate's steady-state loudness is within 3dB of reverb's own at the same settings");
    }

    // [P5] Stereo cross-coupling: an L-only impulse must produce
    // significant R-channel energy -- here via the figure-8 tank
    // coupling and the multi-tap stereo extraction, not an N-line mix,
    // but the same requirement applies (signal must genuinely cross
    // channels, not just decay independently per side).
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        double energyL = 0.0, energyR = 0.0;
        const size_t n = (size_t)(0.5 * fs); // this topology's own loop is much longer than the FDN's lines, give it more time to cross over
        for (size_t i = 0; i < n; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, 0.0, outL, outR);
            energyL += outL * outL;
            energyR += outR * outR;
        }
        const double ratio = energyR / energyL;
        std::printf("      energyL=%.6f energyR=%.6f ratio(R/L)=%.3f\n", energyL, energyR, ratio);
        check(ratio > 0.1, "[P5] L-only impulse produces substantial R-channel energy");
    }

    // [P6] Long continuous-noise stress run stays finite and bounded.
    {
        ReverbEnginePlate eng;
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
        check(finite, "[P6] 10s continuous-noise stress run stays finite");
        check(maxAbs < 20.0, "[P6] 10s continuous-noise stress run stays bounded (< 20.0)");
    }

    // [P7] A cascade of PlateAllpass stages (the same structure the
    // engine's own mono input diffusion chain uses -- 4 stages, Dattorro's
    // own published lengths/gains at the reference room, see
    // ReverbEnginePlate.hpp's own header comment) is genuinely unity-
    // magnitude at every frequency tested -- same requirement and same
    // method as [R12] checks for AllpassDiffuser. Built directly from
    // PlateAllpass here (not via ReverbEnginePlate's own private
    // members) since input diffusion is now an internal implementation
    // detail of the engine, not a separately reusable public class.
    {
        auto makeCascade = [](std::array<PlateAllpass, 4> &stages) {
            static constexpr int kLens[4] = {142, 107, 379, 277};
            static constexpr double kGains[4] = {0.750, 0.750, 0.625, 0.625};
            for (int i = 0; i < 4; ++i) {
                stages[i].designLength(kLens[i]);
                stages[i].setGain(kGains[i]);
            }
        };
        bool allClose = true;
        double worstRatio = 1.0;
        for (double freqHz : {50.0, 200.0, 1000.0, 4000.0, 12000.0}) {
            std::array<PlateAllpass, 4> stages;
            makeCascade(stages);
            const size_t n = (size_t)(0.5 * fs);
            double inEnergy = 0.0, outEnergy = 0.0;
            for (size_t i = 0; i < n; ++i) {
                const double t = (double)i / fs;
                const double x = std::sin(2.0 * M_PI * freqHz * t);
                double y = x;
                for (auto &s : stages) y = s.process(y);
                if (i >= 2000) { inEnergy += x * x; outEnergy += y * y; } // longer settle time than [R12]'s -- this diffuser's own stages are longer
            }
            const double ratio = std::sqrt(outEnergy / inEnergy);
            worstRatio = std::fabs(ratio - 1.0) > std::fabs(worstRatio - 1.0) ? ratio : worstRatio;
            if (std::fabs(ratio - 1.0) > 0.02) allClose = false;
        }
        std::printf("      worst RMS ratio (out/in) across probed frequencies: %.4f\n", worstRatio);
        check(allClose, "[P7] input diffusion cascade preserves signal RMS (unity magnitude) at every probed frequency");
    }

    // [P8] Stereo Width identity/mono/double-side checks -- same
    // formula/behavior as ReverbEngine's own, checked the same way.
    {
        auto engDefault = std::make_unique<ReverbEnginePlate>();
        auto engExplicit1 = std::make_unique<ReverbEnginePlate>();
        auto engZero = std::make_unique<ReverbEnginePlate>();
        auto engTwo = std::make_unique<ReverbEnginePlate>();
        engDefault->prepare(room, fs, t60Dc, t60Ny);
        engExplicit1->prepare(room, fs, t60Dc, t60Ny);
        engExplicit1->setWidth(1.0);
        engZero->prepare(room, fs, t60Dc, t60Ny);
        engZero->setWidth(0.0);
        engTwo->prepare(room, fs, t60Dc, t60Ny);
        engTwo->setWidth(2.0);

        std::mt19937 rngW(55);
        std::uniform_real_distribution<double> distW(-0.5, 0.5);
        bool identityAt1 = true, monoAt0 = true, sideDoubledAt2 = true;
        const int n = 2000;
        for (int i = 0; i < n; ++i) {
            const double inL = distW(rngW), inR = distW(rngW);
            double dL, dR, e1L, e1R, zL, zR, tL, tR;
            engDefault->processStereo(inL, inR, dL, dR);
            engExplicit1->processStereo(inL, inR, e1L, e1R);
            engZero->processStereo(inL, inR, zL, zR);
            engTwo->processStereo(inL, inR, tL, tR);

            if (dL != e1L || dR != e1R) identityAt1 = false;
            if (std::fabs(zL - zR) > 1e-12) monoAt0 = false;

            const double midDefault = 0.5 * (dL + dR), sideDefault = 0.5 * (dL - dR);
            const double midTwo = 0.5 * (tL + tR), sideTwo = 0.5 * (tL - tR);
            if (std::fabs(midTwo - midDefault) > 1e-9) sideDoubledAt2 = false;
            if (std::fabs(sideTwo - 2.0 * sideDefault) > 1e-9) sideDoubledAt2 = false;
        }
        std::printf("      Width=1.0 identity=%d  Width=0.0 mono=%d  Width=2.0 mid-unchanged/side-doubled=%d\n",
                    identityAt1, monoAt0, sideDoubledAt2);
        check(identityAt1, "[P8] Width=1.0 (default) is an exact identity vs. width-untouched output");
        check(monoAt0, "[P8] Width=0.0 collapses the output to mono (outL==outR)");
        check(sideDoubledAt2, "[P8] Width=2.0 leaves mid unchanged and exactly doubles the side signal");
    }

    // [P9] Balance identity/extremes.
    {
        auto engDefault = std::make_unique<ReverbEnginePlate>();
        auto engBal = std::make_unique<ReverbEnginePlate>();
        engDefault->prepare(room, fs, t60Dc, t60Ny);
        engBal->prepare(room, fs, t60Dc, t60Ny);
        engBal->setBalance(0.0);

        std::mt19937 rngB(77);
        std::uniform_real_distribution<double> distB(-0.5, 0.5);
        bool identityAtZero = true;
        for (int i = 0; i < 1000; ++i) {
            const double inL = distB(rngB), inR = distB(rngB);
            double dL, dR, bL, bR;
            engDefault->processStereo(inL, inR, dL, dR);
            engBal->processStereo(inL, inR, bL, bR);
            if (dL != bL || dR != bR) identityAtZero = false;
        }

        auto engFullLeft = std::make_unique<ReverbEnginePlate>();
        auto engFullRight = std::make_unique<ReverbEnginePlate>();
        engFullLeft->prepare(room, fs, t60Dc, t60Ny);
        engFullLeft->setBalance(-1.0);
        engFullRight->prepare(room, fs, t60Dc, t60Ny);
        engFullRight->setBalance(1.0);
        double maxRAtFullLeft = 0.0, maxLAtFullRight = 0.0;
        std::mt19937 rngB2(88);
        for (int i = 0; i < 2000; ++i) {
            const double inL = distB(rngB2), inR = distB(rngB2);
            double lL, lR, rL, rR;
            engFullLeft->processStereo(inL, inR, lL, lR);
            engFullRight->processStereo(inL, inR, rL, rR);
            maxRAtFullLeft = std::max(maxRAtFullLeft, std::fabs(lR));
            maxLAtFullRight = std::max(maxLAtFullRight, std::fabs(rL));
        }
        std::printf("      Balance identity@0=%d  maxR@Balance=-1: %.9f  maxL@Balance=+1: %.9f\n",
                    identityAtZero, maxRAtFullLeft, maxLAtFullRight);
        check(identityAtZero, "[P9] Balance=0.0 (default) is an exact identity");
        check(maxRAtFullLeft < 1e-12, "[P9] Balance=-1.0 fully silences the R channel");
        check(maxLAtFullRight < 1e-12, "[P9] Balance=+1.0 fully silences the L channel");
    }

    // [P10] Delay-line modulation identity/engagement/stability -- this
    // engine's own two ModulatedPlateAllpass instances (one per tank),
    // checked the same three ways [R16] checks ReverbEngine's own N.
    {
        auto engOff = std::make_unique<ReverbEnginePlate>();
        auto engOff2 = std::make_unique<ReverbEnginePlate>();
        auto engOn = std::make_unique<ReverbEnginePlate>();
        engOff->prepare(room, fs, t60Dc, t60Ny);
        engOff->setModulation(0.0);
        engOff2->prepare(room, fs, t60Dc, t60Ny);
        engOff2->setModulation(0.0);
        engOn->prepare(room, fs, t60Dc, t60Ny);
        engOn->setModulation(0.3);

        std::mt19937 rngM(303);
        std::uniform_real_distribution<double> distM(-0.5, 0.5);
        bool identityAtZero = true, differsWhenOn = false;
        const size_t n = (size_t)(2.0 * fs);
        for (size_t i = 0; i < n; ++i) {
            const double inL = distM(rngM), inR = distM(rngM);
            double offL, offR, off2L, off2R, onL, onR;
            engOff->processStereo(inL, inR, offL, offR);
            engOff2->processStereo(inL, inR, off2L, off2R);
            engOn->processStereo(inL, inR, onL, onR);
            if (offL != off2L || offR != off2R) identityAtZero = false;
            if (offL != onL || offR != onR) differsWhenOn = true;
        }

        auto engMax = std::make_unique<ReverbEnginePlate>();
        engMax->prepare(room, fs, t60Dc, t60Ny);
        engMax->setModulation(1.0);
        std::mt19937 rngM3(404);
        std::uniform_real_distribution<double> distM3(-0.8, 0.8);
        bool finiteAtMax = true;
        double maxAbsAtMax = 0.0;
        const size_t nStress = (size_t)(5.0 * fs);
        for (size_t i = 0; i < nStress; ++i) {
            double outL, outR;
            engMax->processStereo(distM3(rngM3), distM3(rngM3), outL, outR);
            if (!std::isfinite(outL) || !std::isfinite(outR)) finiteAtMax = false;
            maxAbsAtMax = std::max({maxAbsAtMax, std::fabs(outL), std::fabs(outR)});
        }

        std::printf("      Modulation=0.0 identity=%d  Modulation=0.3 differs-from-off=%d  Modulation=1.0 finite=%d maxAbs=%.4f\n",
                    identityAtZero, differsWhenOn, finiteAtMax, maxAbsAtMax);
        check(identityAtZero, "[P10] Modulation=0.0 is an exact, bit-identical no-op");
        check(differsWhenOn, "[P10] Modulation=0.3 (default) measurably changes the output (not a dead knob)");
        check(finiteAtMax, "[P10] Modulation=1.0 (max depth) stays finite under sustained noise");
        check(maxAbsAtMax < 20.0, "[P10] Modulation=1.0 (max depth) stays bounded under sustained noise");
    }

    // [P11] Room-size change via processBlock()/requestRoom() ducks to
    // silence and back, and the engine really runs the new room's
    // lengths afterward.
    {
        ReverbEnginePlate eng;
        RoomDims smallRoom{4.0, 3.0, 2.5};
        eng.prepare(smallRoom, fs, t60Dc, t60Ny);
        const auto lengthsBefore = eng.lengths();

        std::vector<float> inL(fs * 1.0, 0.3f), inR(fs * 1.0, 0.3f);
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
        for (size_t i = 0; i < inL.size() && i < (size_t)(fs * 0.05); ++i)
            minAbsAfterRequest = std::min(minAbsAfterRequest, (double)std::fabs(outL[i]));

        const auto lengthsAfter = eng.lengths();
        const bool lengthsChanged = lengthsBefore != lengthsAfter;

        std::printf("      duck states seen: fadingOut=%d fadingIn=%d idleAgain=%d | min|out| near duck=%.6f\n",
                    sawFadingOut, sawFadingIn, sawIdleAgain, minAbsAfterRequest);
        check(sawFadingOut && sawFadingIn && sawIdleAgain, "[P11] room change visibly ducks out, in, then returns to Idle");
        check(minAbsAfterRequest < 0.001, "[P11] duck actually dips to near-silence (not just a state-flag change)");
        check(lengthsChanged, "[P11] segment lengths actually change to the new (larger) room's values after the duck");
    }

    // [P12] requestRoom() is genuinely safe to call from a different
    // thread than processBlock().
    {
        ReverbEnginePlate eng;
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
        const int blocks = 2000;
        for (int b = 0; b < blocks; ++b) {
            for (int i = 0; i < 512; ++i) { inL[i] = (float)dist(rngAudio); inR[i] = (float)dist(rngAudio); }
            eng.processBlock(inL.data(), inR.data(), outL.data(), outR.data(), 512);
            for (int i = 0; i < 512; ++i)
                if (!std::isfinite(outL[i]) || !std::isfinite(outR[i])) finite = false;
        }
        stop.store(true, std::memory_order_relaxed);
        requester.join();

        std::printf("      concurrent requestRoom() calls while processBlock() ran: %d\n", roomChanges.load());
        check(finite, "[P12] audio stays finite while a concurrent thread calls requestRoom() continuously");
        check(roomChanges.load() > 0, "[P12] the concurrent requester thread actually ran (test isn't vacuous)");
    }

    // [P13] Early-reflection onset -- a pre-delay was audible even with
    // Pre-Delay set to 0. Root-caused (not guessed): before
    // the early-reflection taps existed, the ONLY output taps sat past
    // nearly a full tank traversal, producing ~124ms of true silence
    // regardless of any Pre-Delay setting (Pre-Delay itself, applied
    // upstream of this engine entirely, was never the cause). Checked
    // here directly against the same order of magnitude as the FDN
    // models' own onset (measured separately, ~6ms) -- not just "greater
    // than zero", which the old ~124ms-latent version would also have
    // passed.
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        int firstAudible = -1;
        const double threshold = 1e-4;
        for (int i = 0; i < 20000 && firstAudible < 0; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            if (std::fabs(outL) > threshold || std::fabs(outR) > threshold) firstAudible = i;
        }
        std::printf("      first audible wet sample: %d (%.1fms)\n", firstAudible, firstAudible * 1000.0 / fs);
        check(firstAudible >= 0 && firstAudible < 1000,
              "[P13] wet signal starts within ~20ms of an impulse (matches reverb/reverb_dense's own onset order of magnitude, not the ~124ms a missing early tap used to produce)");
    }

    // [P14] No audible gap between the early-reflection taps and the
    // late tail's own first full loop trip -- the audible delay was still
    // there AFTER [P13]'s own fix already shipped (and only on the plate).
    // [P13] alone could not have
    // caught this: a single early tap (the first fix) genuinely did
    // produce audible output within ~6ms, satisfying [P13], but that tap
    // only echoes the original excitation's own one-time pass through
    // the tank's own front -- nothing refreshes it again until the
    // recirculating feedback completes a full loop trip, so the envelope
    // actually decayed ~30dB toward silence and then jumped back up once
    // the late tail arrived (measured directly: a 5ms-window RMS
    // envelope over the first 300ms, not inferred from a single
    // first-audible-sample check). Fixed with several taps spread across
    // delayA1_/delayB1_ and allpassA2_/allpassB2_'s own lengths (see
    // processStereo()'s own comment) -- this test measures the SAME
    // envelope shape directly and checks the dip stays shallow (a real
    // reverb's own envelope is never perfectly monotonic either; the bug
    // was a ~30dB canyon, not "not perfectly flat").
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        const int winSamples = (int)(0.005 * fs); // 5ms windows, matching the harness this was diagnosed with
        const int numWindows = 30; // covers 0-150ms, the region the original bug's own gap sat in
        std::vector<double> windowRmsDb(numWindows, -999.0);
        double winSumSq = 0.0;
        int winCount = 0, winIdx = 0;
        for (int i = 0; i < winSamples * numWindows; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            winSumSq += outL * outL + outR * outR;
            if (++winCount >= winSamples) {
                const double rms = std::sqrt(winSumSq / (2.0 * winCount));
                windowRmsDb[winIdx++] = rms > 1e-12 ? 20.0 * std::log10(rms) : -999.0;
                winSumSq = 0.0;
                winCount = 0;
            }
        }
        // The dip: peak dB in the first half of the window (the early
        // taps' own region) minus the lowest dB anywhere after that peak
        // but still before the tail has clearly arrived (the trough
        // right before the late tail's own rise). Measured directly
        // rather than assuming which window index the trough falls in.
        double peakDb = -999.0;
        int peakIdx = 0;
        for (int i = 0; i < numWindows / 2; ++i)
            if (windowRmsDb[i] > peakDb) { peakDb = windowRmsDb[i]; peakIdx = i; }
        double troughDb = peakDb;
        for (int i = peakIdx; i < numWindows; ++i)
            troughDb = std::min(troughDb, windowRmsDb[i]);
        const double dipDb = peakDb - troughDb;
        std::printf("      envelope: early peak=%.1fdB, worst trough after it=%.1fdB, dip=%.1fdB\n",
                    peakDb, troughDb, dipDb);
        check(dipDb < 20.0,
              "[P14] no large gap between the early-reflection taps and the late tail (dip stays under 20dB, not the ~30dB canyon the original single-tap fix left behind)");
    }

    // [P15] No audible SECOND HUMP once the late tail's own first full
    // loop trip arrives -- audible echo/delay problems remained AFTER
    // [P14]'s own fix already shipped. [P14]
    // could not have caught this: it only checks how much QUIETER things
    // get after the early peak, never whether something ELSE gets
    // LOUDER again later. Root-caused by rendering the actual waveform
    // as an image and looking at it directly (three different scalar
    // probes -- tail autocorrelation, threshold-based peak-density, and
    // cross-correlation against a synthetic clap -- were all tried first
    // and came back inconclusive): the late tail's own first full loop
    // trip (tankAOut/tankBOut/tapA_mid/tapB_mid, arriving all at once
    // since there are only two recirculating tanks here, unlike the FDN
    // models' own 8-16 independently-timed lines) was a single,
    // undispersed burst comparable in loudness to the early-reflection
    // cluster's own onset peak -- a visible "second hit" in the waveform,
    // absent from reverb_dense's own (rendered and inspected the same
    // way). Fixed with lateTailFadeInSamples_ (see processStereo()'s own
    // "Late-tail fade-in" comment) -- this test measures the same
    // early-peak-vs-later-peak relationship [P14] doesn't, and checks
    // the late region never gets more than a few dB louder than the
    // genuine early onset.
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        const int winSamples = (int)(0.005 * fs); // 5ms windows, same harness as [P14]
        const int numWindows = 80; // covers 0-400ms -- comfortably past where the second hump used to sit (~150-220ms)
        std::vector<double> windowRmsDb(numWindows, -999.0);
        double winSumSq = 0.0;
        int winCount = 0, winIdx = 0;
        for (int i = 0; i < winSamples * numWindows; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            winSumSq += outL * outL + outR * outR;
            if (++winCount >= winSamples) {
                const double rms = std::sqrt(winSumSq / (2.0 * winCount));
                windowRmsDb[winIdx++] = rms > 1e-12 ? 20.0 * std::log10(rms) : -999.0;
                winSumSq = 0.0;
                winCount = 0;
            }
        }
        // Early peak: the genuine onset, in the first 100ms (the early-
        // reflection taps' own region -- see [P13]/[P14]'s own comments).
        // Late peak: the loudest anything gets from 100ms onward. A
        // properly blended tail's own late peak should never exceed the
        // genuine onset by more than a few dB.
        const int splitWindow = 20; // 100ms / 5ms
        double earlyPeakDb = -999.0;
        for (int i = 0; i < splitWindow; ++i) earlyPeakDb = std::max(earlyPeakDb, windowRmsDb[i]);
        double latePeakDb = -999.0;
        int latePeakIdx = splitWindow;
        for (int i = splitWindow; i < numWindows; ++i)
            if (windowRmsDb[i] > latePeakDb) { latePeakDb = windowRmsDb[i]; latePeakIdx = i; }
        const double humpDb = latePeakDb - earlyPeakDb;
        std::printf("      envelope: early peak(0-100ms)=%.1fdB, late peak=%.1fdB at t=%.0fms, hump=%.1fdB\n",
                    earlyPeakDb, latePeakDb, latePeakIdx * 5.0 + 2.5, humpDb);
        check(humpDb < 2.0,
              "[P15] the late tail's own first full loop trip never gets more than 2dB louder than the genuine early onset (checked to actually discriminate: an earlier draft of this threshold, 6dB, still passed even with the fade-in effectively disabled -- lateTailRampSamples_ forced to 1 measured a 3.7dB hump and slipped under it -- so 6dB wasn't tight enough to guard the fix at all; 2dB is tight enough that the same sabotage fails while the real fix's own -2.4dB measurement still passes)");
    }

    // [P16] No REPEATING bumps in the LATE tail (>=200ms), separate from
    // a one-time onset transient -- a clear echo was still audible AFTER
    // [P14]/[P15] both already shipped. Root-caused
    // (in the engine's own first, from-scratch-designed topology, since
    // replaced -- see ReverbEnginePlate.hpp's own header comment) by
    // rendering the FULL ~1s decay as a dB envelope curve: a clearly
    // REPEATING bump roughly every ~200ms, continuing through the WHOLE
    // tail, because that topology's own two tanks recirculated at
    // nearly the same rate and reinforced each other's own periodicity
    // every single pass.
    //
    // After the rewrite to Dattorro's own real topology (matched-length
    // tanks, per his own published design -- 672+4453=908+4217=5125
    // samples in his own numbers, NOT spread apart, contrary to what
    // this file's earlier draft needed for its own different topology),
    // the same full-decay render shows something categorically
    // different: ONE localized transient at the tank's own first loop
    // return (measured directly: excess above a 100ms trailing trend
    // peaks at ~4.8dB around t=105-135ms, then drops to exactly 0.0dB
    // excess for the entire rest of the 1.2s render -- a single event,
    // not a repeating pattern). Rendering this as a waveform image
    // confirms it visually: a smooth, monotonically decaying envelope
    // with a small blend-in ripple near onset, nothing resembling the
    // old repeating-bump shape at any point after ~150ms. This is the
    // ordinary, physically-expected behavior of a 2-tank plate design
    // (the first full loop return is a real, one-time event even in
    // Dattorro's own proven design; his own multi-tap output structure
    // is what keeps it from being audible as a discrete "echo," not an
    // absence of any onset transient at all) -- so this test now checks
    // the LATE tail (>=200ms, well past that one-time transient) stays
    // free of the ORIGINAL repeating-bump defect, plus a separate,
    // looser bound on the onset region itself so a real regression
    // there (e.g. the transient growing much larger, or a second one
    // appearing) still gets caught.
    {
        ReverbEnginePlate eng;
        eng.prepare(room, fs, t60Dc, t60Ny);
        const int winSamples = (int)(0.01 * fs); // 10ms windows -- coarser than [P14]/[P15]'s own 5ms, appropriate for a much longer analysis window
        const int numWindows = 120; // covers 0-1200ms
        std::vector<double> windowRmsDb(numWindows, -999.0);
        double winSumSq = 0.0;
        int winCount = 0, winIdx = 0;
        for (int i = 0; i < winSamples * numWindows; ++i) {
            double outL, outR;
            eng.processStereo(i == 0 ? 1.0 : 0.0, i == 0 ? 1.0 : 0.0, outL, outR);
            winSumSq += outL * outL + outR * outR;
            if (++winCount >= winSamples) {
                const double rms = std::sqrt(winSumSq / (2.0 * winCount));
                windowRmsDb[winIdx++] = rms > 1e-12 ? 20.0 * std::log10(rms) : -999.0;
                winSumSq = 0.0;
                winCount = 0;
            }
        }
        const int trendWindows = 10; // 100ms trailing average
        const int onsetEndWindow = 20; // 200ms -- past the tank's own one-time first loop return
        double worstOnsetExcessDb = 0.0, worstLateExcessDb = 0.0;
        int worstOnsetIdx = trendWindows, worstLateIdx = onsetEndWindow;
        for (int i = trendWindows; i < numWindows; ++i) {
            double trend = 0.0;
            for (int j = i - trendWindows; j < i; ++j) trend += windowRmsDb[j];
            trend /= trendWindows;
            const double excess = windowRmsDb[i] - trend;
            if (i < onsetEndWindow) {
                if (excess > worstOnsetExcessDb) { worstOnsetExcessDb = excess; worstOnsetIdx = i; }
            } else {
                if (excess > worstLateExcessDb) { worstLateExcessDb = excess; worstLateIdx = i; }
            }
        }
        std::printf("      onset-region (<200ms) worst excess = %.1fdB at t=%.0fms | late-tail (>=200ms) worst excess = %.1fdB at t=%.0fms\n",
                    worstOnsetExcessDb, worstOnsetIdx * 10.0 + 5.0, worstLateExcessDb, worstLateIdx * 10.0 + 5.0);
        check(worstLateExcessDb < 2.0,
              "[P16] no REPEATING bump anywhere in the late tail (>=200ms, past the tank's own one-time first loop return): no point rises more than 2dB above its own local trend (this is what actually distinguished the old, since-replaced topology's defect -- a bump recurring every ~200ms through the WHOLE tail -- from the current design's own single onset transient)");
        check(worstOnsetExcessDb < 8.0,
              "[P16b] the onset region's own one-time transient (the tank's first loop return blending with the diffuse onset) stays under a generous 8dB bound -- loose on purpose (this is expected, present-even-in-Dattorro's-own-design behavior, not the defect being guarded against), just tight enough to catch a real regression (e.g. the transient growing much larger or a second one appearing)");
    }

    std::printf("\n%s (%d failure%s)\n", gFailures == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
                gFailures, gFailures == 1 ? "" : "s");
    return gFailures == 0 ? 0 : 1;
}
