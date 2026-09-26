// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ReverbEnginePlate.hpp
//
// The third reverb model -- added after the FDN-based "Smooth" model
// still sounded similar in character (still like material pitched far
// down). Both `ReverbEngine` (8 lines) and `ReverbEngineDense`
// (16 lines) are the SAME technology -- an N-line Householder Feedback
// Delay Network -- just at different N, which only ever changes mode
// DENSITY, not the underlying mechanism that produces the reported
// "pitched down" artifact (a small number of parallel resonant lines
// whose modes aren't dense enough to blend into noise-like decay). This
// file is a genuinely different topology: a Dattorro-style PLATE reverb
// -- a short cascade of input-diffusion allpasses feeding a *single*
// figure-8 coupled pair of long delay loops (not N parallel lines mixed
// by one matrix), with a modulated allpass and a one-pole damping filter
// once per loop traversal, and a stereo image built by tapping multiple
// points along both loops rather than by mixing N independent lines.
//
// REWRITE NOTE: the first version of this file used the
// same STRUCTURAL idea (input diffusion -> figure-8 coupled tanks ->
// multi-tap output) but its own exact numbers and internal wiring were
// designed from scratch, not verified against Jon Dattorro's own
// published design -- his 1997 paper ("Effect Design, Part 1:
// Reverberator and Other Filters," J. Audio Eng. Soc. Vol 45 No 9) was
// not available to check against a primary source at the time. Four
// separate rounds of tuning that from-scratch design (onset latency,
// envelope hump, an ongoing periodic beat through the tail, a second
// diffusion stage) each measured real, verified improvement -- yet the
// audible echo problem never fully cleared (better each time, but still
// there, and present at Pre-Delay 0ms too, so Pre-Delay was not the
// cause).
//
// Reading Dattorro's own paper directly (primary source, fetched and
// read page-by-page including Figure 1's topology diagram and Table
// 1/Table 2's own exact parameter/tap tables) surfaced two STRUCTURAL
// differences from what this file had built, either one enough to
// explain the persistent problem no amount of retuning the WRONG
// structure could fix:
//
//   1. The recirculating loop is SHORT: modulated allpass -> long delay
//      -> damping, then straight across to the other tank. The "decay
//      diffusion 2" allpass + its own second long delay is a SEPARATE,
//      NON-recirculating branch fed from that same crossover point --
//      it exists ONLY to feed extra output taps, and never rejoins the
//      loop. The first version of this file put that second allpass+
//      delay pair INSIDE the serial recirculating path itself, making
//      its own loop period longer and its own recirculation dynamics
//      structurally different from the proven design.
//
//   2. The stereo output is built from SEVEN weighted, alternating-sign
//      taps per channel (Dattorro's own Table 2), reading multiple
//      offsets from BOTH tanks' delay lines -- not two taps per channel
//      (one tank's own fresh output + a single midpoint tap of the
//      other). That much tap density is what turns two coupled mono
//      loops into a smooth, decorrelated stereo tail without a discrete
//      "echo" character; two taps per channel was not enough density
//      REGARDLESS of how the individual delay lengths were tuned.
//
// This version reproduces Dattorro's own published lengths, gains, and
// output-tap structure (Tables 1 and 2) as faithfully as this project's
// own conventions allow, rather than re-deriving proportions from
// scratch a second time -- his numbers are scaled by (this room's own
// mean-free-path relative to a reference room) x (this sample rate
// relative to his own reference 29761 Hz), preserving his own exact
// RATIOS between every segment while still letting Room Length/Width/
// Height stay meaningful (consistent with every other engine in this
// project). At the reference room (10m x 7m x 3.5m, this project's own
// established default), the room-relative factor is exactly 1.0, so at
// that room this engine's own lengths are Dattorro's own absolute
// numbers scaled ONLY by sample rate -- directly checkable against his
// own published table.
//
// Two deliberate departures from a byte-exact reproduction, both
// carried over from this project's own established conventions rather
// than introducing new machinery:
//   - Input is summed to mono before diffusion (Dattorro's own design),
//     but this engine's own Decay Time + Damping controls still drive
//     TWO independent per-tank OnePoleDamping filters (his own paper
//     uses a single simpler one-pole "damping" lowpass PLUS a separate
//     scalar "decay" gain per loop pass) -- OnePoleDamping's own
//     two-point DC/Nyquist fit already subsumes both roles in one
//     filter and is what every other engine in this project already
//     uses, so reusing it here avoids inventing a second, parallel
//     decay-parameter mapping for one engine only.
//   - "Decay diffusion 1" (the modulated allpass) and "decay diffusion
//     2" (the tap-only branch's own allpass) use Dattorro's own DEFAULT
//     gains (0.70 and 0.50) as fixed constants, not values derived from
//     his own "decay diffusion 2 = decay + 0.15, clamped" relationship
//     -- that relationship is itself described in his own paper as
//     empirically tuned "by ear," not a strict requirement, and at his
//     own default decay (0.50) it saturates at the clamp ceiling (0.50)
//     anyway, i.e. IS the fixed constant used here.
//
// Reuses every genuinely context-independent piece from ReverbEngine.hpp
// via #include (RoomDims, meanFreePathM, isPrime/nearestPrimeAtLeast,
// OnePoleDamping, TempoNoteValue/t60SecFromTempo, firstOrderImageSources,
// DuckState, kMaxDelaySamples, kSpeedOfSoundMPerSec) -- see that file's
// own pieces for what each does.
//
// RT-safety: every buffer below is a FIXED std::array<double,
// kMaxDelaySamples>, exactly like ReverbEngine.hpp's own buf_ -- NOT a
// std::vector sized at prepare() time. A room-size change still runs
// prepare() from inside processBlock()'s own duck window (see
// ReverbEngine.hpp's "Room-size changes" section for the full argument,
// reused unchanged below) -- allocating there would defeat the entire
// point of that mechanism, so every "resize" here is really just
// picking a new `length` into an already-allocated array and
// `.fill(0.0)`-ing it, never a real (re)allocation.

#pragma once

#include "ReverbEngine.hpp" // RoomDims, meanFreePathM, isPrime/nearestPrimeAtLeast, OnePoleDamping, TempoNoteValue/t60SecFromTempo, firstOrderImageSources, DuckState, kMaxDelaySamples, kSpeedOfSoundMPerSec

namespace ssfilter {
namespace reverb {
namespace plate {

// Fixed-delay allpass, y[n]=-g*x[n]+x[n-M]+g*y[n-M] -- identical
// recursion to ReverbEngine.hpp's own AllpassStage, just without that
// struct's 128-sample cap (a plate's own input-diffusion/decay-diffusion
// stages need hundreds to low-thousands of samples, not tens). Used for
// input diffusion and each tank's own fixed (non-modulated) "decay
// diffusion 2" allpass.
struct PlateAllpass {
    std::array<double, kMaxDelaySamples> buf{};
    int writeIdx = 0;
    int length = 1;
    double gain = 0.0;

    void designLength(int lengthSamples) {
        length = std::clamp(lengthSamples, 1, kMaxDelaySamples - 1);
        buf.fill(0.0);
        writeIdx = 0;
    }
    void setGain(double g) { gain = std::clamp(g, -0.95, 0.95); }

    double process(double x) {
        const double delayed = buf[writeIdx];
        const double y = -gain * x + delayed;
        buf[writeIdx] = x + gain * y;
        writeIdx = (writeIdx + 1 >= length) ? 0 : writeIdx + 1;
        return y;
    }

    // Non-destructive historical read of this allpass's own internal
    // state (x + gain*y, not a "clean" delayed copy of its input) --
    // used for the "decay diffusion 2" taps in Dattorro's own Table 2
    // (his own node31_33/node55_59), which read partway into this
    // stage's own delay register rather than only its full-length
    // output.
    double readTap(int k) const {
        k = std::clamp(k, 1, length);
        int idx = writeIdx - k;
        while (idx < 0) idx += length;
        return buf[idx % length];
    }
};

// Plain long delay line, read-before-write like every other delay line
// in this project -- plus readTap(), a non-destructive read at any
// historical offset up to `length`, used for Dattorro's own multi-tap
// stereo extraction (his own Table 2 reads several points along the
// same long delay rather than mixing N independent lines -- what turns
// two coupled mono loops into a decorrelated stereo image).
struct PlateDelay {
    std::array<double, kMaxDelaySamples> buf{};
    int writeIdx = 0;
    int length = 1;

    void designLength(int lengthSamples) {
        length = std::clamp(lengthSamples, 1, kMaxDelaySamples - 1);
        buf.fill(0.0);
        writeIdx = 0;
    }

    double process(double x) {
        const double y = buf[writeIdx];
        buf[writeIdx] = x;
        writeIdx = (writeIdx + 1 >= length) ? 0 : writeIdx + 1;
        return y;
    }

    // Sample that is `k` samples old, 1 <= k <= length (k == length is
    // the same value process()'s own read would return this sample).
    double readTap(int k) const {
        k = std::clamp(k, 1, length);
        int idx = writeIdx - k;
        while (idx < 0) idx += length;
        return buf[idx % length];
    }
};

// A modulated allpass -- Dattorro's own "decay diffusion 1" stage, one
// per tank, its own read tap swept by a slow LFO (his own paper: ~1Hz
// rate, +-8 sample excursion at his reference rate/room) so the loop
// never settles into a fixed, audible resonance. Buffer sized
// `baseLength + 2*maxDepthSamplesInt + margin` so the modulated read
// position always has real (already-written) history on both sides of
// its sweep to interpolate between, at any point in the LFO cycle.
struct ModulatedPlateAllpass {
    std::array<double, kMaxDelaySamples> buf{};
    int bufSize = 1;
    int writeIdx = 0;
    int baseLength = 1;
    double gain = 0.0;
    double phase = 0.0;      // 0..1, wraps
    double rate = 0.2;       // Hz
    double depthSamples = 0.0;

    void designLength(int lengthSamples, int maxDepthSamplesInt) {
        const int margin = 2 * maxDepthSamplesInt + 4;
        baseLength = std::clamp(lengthSamples, 1, kMaxDelaySamples - margin - 1);
        bufSize = std::clamp(baseLength + margin, baseLength + 1, kMaxDelaySamples);
        buf.fill(0.0);
        writeIdx = 0;
    }
    void setGain(double g) { gain = std::clamp(g, -0.95, 0.95); }

    double process(double x, double sampleRate) {
        phase += rate / sampleRate;
        if (phase >= 1.0) phase -= std::floor(phase);
        const double mod = depthSamples * std::sin(2.0 * M_PI * phase); // -depth..+depth
        double readPos = double(writeIdx) - double(baseLength) - mod;
        while (readPos < 0.0) readPos += double(bufSize);
        const int i0 = int(readPos) % bufSize;
        const int i1 = (i0 + 1) % bufSize;
        const double frac = readPos - std::floor(readPos);
        const double delayed = buf[i0] * (1.0 - frac) + buf[i1] * frac;

        const double y = -gain * x + delayed;
        buf[writeIdx] = x + gain * y;
        writeIdx = (writeIdx + 1 >= bufSize) ? 0 : writeIdx + 1;
        return y;
    }
};

// The plate engine itself. Public surface intentionally mirrors
// ReverbEngine/ReverbEngineDense exactly (prepare/setDecayTargets/
// setModulation/setDiffusion/setWidth/setBalance/snapDecayTargets/
// requestRoom/processBlock/duckState/currentT60Dc-NySec/consumeFaultFlag/
// processStereo/lengths) so PluginReverbPlate can be a drop-in structural
// copy of PluginReverb, and so the shared GUI (gui/UIReverb.*) needs zero
// changes to drive this third model -- see plugin/ReverbParams.hpp's own
// comment for why that sharing works at all.
class ReverbEnginePlate {
public:
    void prepare(const RoomDims &room, double sampleRate, double t60DcSec, double t60NySec) {
        sampleRate_ = sampleRate;
        smoothCoeff_ = 1.0 - std::exp(-1.0 / (kCoeffSmoothTimeConstantSec * sampleRate));
        duckStepPerSample_ = 1.0 / (kDuckFadeTimeSec * sampleRate);

        // Dattorro's own design is specified at a fixed Fs=29761Hz on a
        // fixed (unstated) room. Scaling every one of his own absolute
        // sample counts by (this room's own mean-free-path relative to
        // a reference room) x (this sample rate relative to his own
        // 29761Hz) preserves every one of his own RATIOS between
        // segments exactly, while still letting this project's own
        // Room Length/Width/Height controls do something audible here
        // too (consistent with every other engine in this project). At
        // the reference room, the room factor is exactly 1.0, so this
        // engine's own lengths at that room are directly checkable
        // against his own published table (scaled only by sample rate).
        static const double kReferenceMfpM = meanFreePathM(RoomDims{10.0, 7.0, 3.5});
        const double roomFactor = meanFreePathM(room) / kReferenceMfpM;
        const double srFactor = sampleRate / 29761.0;
        const double scale = roomFactor * srFactor;

        auto scaledLen = [&](int dattorroSamples) -> int {
            return std::clamp((int)std::lround(dattorroSamples * scale), 1, kMaxDelaySamples - 1);
        };

        // Input diffusion -- Dattorro's own Table 1: 4 cascaded
        // allpasses, lengths 142/107/379/277, the first two using
        // "input diffusion 1" gain (0.750) and the last two "input
        // diffusion 2" gain (0.625) -- NOT one shared gain for all 4,
        // unlike this file's own first version. Operates on the
        // INPUT SUMMED TO MONO (see processStereo()) -- his own design,
        // not this project's own earlier per-channel diffusion; the
        // stereo image is rebuilt entirely by the output tap structure
        // below instead.
        lenInDiff1_ = scaledLen(142);
        lenInDiff2_ = scaledLen(107);
        lenInDiff3_ = scaledLen(379);
        lenInDiff4_ = scaledLen(277);
        inDiff1_.designLength(lenInDiff1_);
        inDiff2_.designLength(lenInDiff2_);
        inDiff3_.designLength(lenInDiff3_);
        inDiff4_.designLength(lenInDiff4_);
        applyDiffusionGains();

        // Tank A (Dattorro's own "left" tank): decay diffusion 1
        // (modulated allpass, 672) -> long delay (4453) -> damping ->
        // [crossover to tank B] and, from that SAME crossover point, a
        // SEPARATE non-recirculating branch: decay diffusion 2
        // (allpass, 1800) -> second long delay (3720), existing only to
        // feed extra output taps (see processStereo()). This is the
        // key structural fix over this file's own first version, which
        // wrongly put the decay-diffusion-2+delay2 pair INSIDE the
        // recirculating loop itself.
        lenModA_ = scaledLen(672);
        lenDelayA1_ = scaledLen(4453);
        lenAllpassA2_ = scaledLen(1800);
        lenDelayA2_ = scaledLen(3720);

        // Tank B (Dattorro's own "right" tank): same shape, his own
        // different lengths (908/4217/2656/3163) -- deliberately NOT
        // the same as tank A's, so the two loops recirculate at
        // different, non-commensurate periods.
        lenModB_ = scaledLen(908);
        lenDelayB1_ = scaledLen(4217);
        lenAllpassB2_ = scaledLen(2656);
        lenDelayB2_ = scaledLen(3163);

        // Modulation excursion -- Dattorro's own default EXCURSION=16,
        // i.e. +-8 samples at his own reference rate/room, scaled the
        // same way as every length above. This is much smaller than
        // this file's own first version converged on (80 samples) after
        // four rounds of tuning the WRONG topology -- strong evidence
        // that the earlier need for such deep modulation was working
        // around the structural bug fixed by this rewrite, not a
        // genuine requirement of this class of design.
        modDepthBase_ = 8.0 * scale;
        const int maxModDepthSamplesInt = (int)std::ceil(modDepthBase_) + 8;
        modAllpassA_.designLength(lenModA_, maxModDepthSamplesInt);
        modAllpassA_.setGain(kDecayDiffusion1Gain);
        modAllpassA_.rate = 1.00; // Hz, Dattorro's own ~1Hz recommendation
        modAllpassA_.phase = 0.0;
        modAllpassA_.depthSamples = modulationAmount_ * modDepthBase_;

        modAllpassB_.designLength(lenModB_, maxModDepthSamplesInt);
        modAllpassB_.setGain(kDecayDiffusion1Gain);
        modAllpassB_.rate = 1.07;
        modAllpassB_.phase = 0.37;
        modAllpassB_.depthSamples = modulationAmount_ * modDepthBase_;

        delayA1_.designLength(lenDelayA1_);
        allpassA2_.designLength(lenAllpassA2_);
        allpassA2_.setGain(kDecayDiffusion2Gain);
        delayA2_.designLength(lenDelayA2_);

        delayB1_.designLength(lenDelayB1_);
        allpassB2_.designLength(lenAllpassB2_);
        allpassB2_.setGain(kDecayDiffusion2Gain);
        delayB2_.designLength(lenDelayB2_);

        // Only the SHORT recirculating loop (mod-allpass + delay1)
        // feeds the damping design -- decay diffusion 2 + delay2 are
        // no longer part of the loop, unlike this file's own first
        // version.
        loopLenA_ = lenModA_ + lenDelayA1_;
        loopLenB_ = lenModB_ + lenDelayB1_;

        lastT60Dc_ = t60DcSec;
        lastT60Ny_ = t60NySec;
        const double r0A = OnePoleDamping::decayFactor(dampDesignLenA(), t60DcSec, sampleRate);
        const double rNyA = OnePoleDamping::decayFactor(dampDesignLenA(), t60NySec, sampleRate);
        dampA_.design(r0A, rNyA);
        dampA_.snapToTarget();
        dampA_.reset();
        const double r0B = OnePoleDamping::decayFactor(dampDesignLenB(), t60DcSec, sampleRate);
        const double rNyB = OnePoleDamping::decayFactor(dampDesignLenB(), t60NySec, sampleRate);
        dampB_.design(r0B, rNyB);
        dampB_.snapToTarget();
        dampB_.reset();

        tankAFeedback_ = 0.0;
        tankBFeedback_ = 0.0;
    }

    void setDecayTargets(double t60DcSec, double t60NySec) {
        lastT60Dc_ = t60DcSec;
        lastT60Ny_ = t60NySec;
        const double r0A = OnePoleDamping::decayFactor(dampDesignLenA(), t60DcSec, sampleRate_);
        const double rNyA = OnePoleDamping::decayFactor(dampDesignLenA(), t60NySec, sampleRate_);
        dampA_.design(r0A, rNyA);
        const double r0B = OnePoleDamping::decayFactor(dampDesignLenB(), t60DcSec, sampleRate_);
        const double rNyB = OnePoleDamping::decayFactor(dampDesignLenB(), t60NySec, sampleRate_);
        dampB_.design(r0B, rNyB);
    }

    void setModulation(double amount01) {
        modulationAmount_ = std::clamp(amount01, 0.0, 1.0);
        modAllpassA_.depthSamples = modulationAmount_ * modDepthBase_;
        modAllpassB_.depthSamples = modulationAmount_ * modDepthBase_;
    }
    double modulation() const { return modulationAmount_; }

    void setDiffusion(double amount01) {
        diffusionAmount_ = std::clamp(amount01, 0.0, 1.0);
        applyDiffusionGains();
    }
    double diffusion() const { return diffusionAmount_; }

    void setWidth(double width) { widthAmount_ = std::clamp(width, 0.0, 2.0); }
    double width() const { return widthAmount_; }

    void setBalance(double balance) { balanceAmount_ = std::clamp(balance, -1.0, 1.0); }
    double balance() const { return balanceAmount_; }

    void snapDecayTargets(double t60DcSec, double t60NySec) {
        setDecayTargets(t60DcSec, t60NySec);
        dampA_.snapToTarget();
        dampB_.snapToTarget();
    }

    void requestRoom(const RoomDims &room) {
        pendingLengthM_.store(room.lengthM, std::memory_order_relaxed);
        pendingWidthM_.store(room.widthM, std::memory_order_relaxed);
        pendingHeightM_.store(room.heightM, std::memory_order_relaxed);
        roomChangePending_.store(true, std::memory_order_release);
    }

    void processBlock(const float *inL, const float *inR, float *outL, float *outR, int numFrames) {
        for (int i = 0; i < numFrames; ++i) {
            if (duckState_ == DuckState::Idle && roomChangePending_.load(std::memory_order_acquire)) {
                duckState_ = DuckState::FadingOut;
                duckGain_ = 1.0;
            }

            double dOutL, dOutR;
            processStereo((double)inL[i], (double)inR[i], dOutL, dOutR);

            if (duckState_ == DuckState::FadingOut) {
                duckGain_ -= duckStepPerSample_;
                if (duckGain_ <= 0.0) {
                    duckGain_ = 0.0;
                    applyPendingRoom();
                    duckState_ = DuckState::FadingIn;
                }
            } else if (duckState_ == DuckState::FadingIn) {
                duckGain_ += duckStepPerSample_;
                if (duckGain_ >= 1.0) { duckGain_ = 1.0; duckState_ = DuckState::Idle; }
            }

            outL[i] = (float)(dOutL * duckGain_);
            outR[i] = (float)(dOutR * duckGain_);
        }
    }

    DuckState duckState() const { return duckState_; }
    double currentT60DcSec() const { return lastT60Dc_; }
    double currentT60NySec() const { return lastT60Ny_; }
    bool consumeFaultFlag() { return faultFlag_.exchange(false, std::memory_order_relaxed); }

    void processStereo(double inL, double inR, double &outL, double &outR) {
        // Input diffusion operates on the signal summed to mono --
        // Dattorro's own design (Figure 1: L+R -> *0.5 -> diffusers),
        // not this file's own first version's separate per-channel
        // diffusion. The stereo image is rebuilt entirely by the
        // alternating-sign multi-tap output below, not by keeping L/R
        // distinct through the diffusers.
        const double monoIn = 0.5 * (inL + inR);
        double d = inDiff1_.process(monoIn);
        d = inDiff2_.process(d);
        d = inDiff3_.process(d);
        d = inDiff4_.process(d);

        // Figure-8 coupling: each tank's injection is the diffused mono
        // input PLUS the OTHER tank's own crossover output from the
        // previous sample (tankAFeedback_/tankBFeedback_ are read here,
        // before being overwritten below -- a one-sample loop delay
        // across the coupling point, the same "read before write"
        // discipline every delay line in this project already follows,
        // here applied at the loop level instead of the sample-buffer
        // level).
        const double injA = d + tankBFeedback_;
        const double injB = d + tankAFeedback_;

        // Tank A's own SHORT recirculating loop -- decay diffusion 1
        // (modulated) -> long delay 1 -> damping. This is the entire
        // loop; nothing else feeds back into modAllpassA_ next sample.
        double xA = modAllpassA_.process(injA, sampleRate_);
        xA = delayA1_.process(xA);
        dampA_.smoothTowardTarget(smoothCoeff_);
        const double crossA = dampA_.process(xA);

        double xB = modAllpassB_.process(injB, sampleRate_);
        xB = delayB1_.process(xB);
        dampB_.smoothTowardTarget(smoothCoeff_);
        const double crossB = dampB_.process(xB);

        // NaN/Inf safety net -- see ReverbEngine.hpp's own header comment
        // on why this exists even for a structurally lossy-by-
        // construction design: each tank's own OnePoleDamping is
        // designed from a physical T60 > 0 (gain always in (0,1)) and
        // the coupling between tanks carries no gain of its own (unity),
        // so the same "energy-preserving/lossless allpass stages times a
        // strictly lossy filter per loop, every pass" argument applies
        // here too -- a defensive net regardless, not a substitute for
        // that argument.
        double crossAsafe = crossA, crossBsafe = crossB;
        if (!std::isfinite(crossAsafe)) { crossAsafe = 0.0; faultFlag_.store(true, std::memory_order_relaxed); }
        if (!std::isfinite(crossBsafe)) { crossBsafe = 0.0; faultFlag_.store(true, std::memory_order_relaxed); }

        tankAFeedback_ = crossAsafe;
        tankBFeedback_ = crossBsafe;

        // Decay diffusion 2 branch -- SEPARATE, NON-recirculating, fed
        // from the SAME crossover point as the figure-8 coupling above.
        // Its own output is never fed back into the loop; it exists
        // only to supply extra output taps below (Dattorro's own Table
        // 2 reads several points inside this branch too, not just the
        // main loop's own delay1). This is the branch this file's own
        // first version incorrectly folded into the recirculating path.
        const double diffusion2A = allpassA2_.process(crossAsafe);
        delayA2_.process(diffusion2A); // return value unused -- only readTap() below reads this branch
        const double diffusion2B = allpassB2_.process(crossBsafe);
        delayB2_.process(diffusion2B);

        // Output taps -- Dattorro's own Table 2: seven alternating-sign
        // taps per channel, reading from BOTH tanks' own delay lines at
        // specific offsets (scaled the same way every length above is).
        // This is what actually builds a decorrelated stereo image from
        // two coupled mono loops -- replacing this file's own first
        // version's 2-taps-per-channel approach (one tank's own fresh
        // output + a single midpoint tap of the other), which was not
        // dense enough to avoid a discrete "echo" character regardless
        // of how the individual delay lengths were tuned.
        auto tap = [](const PlateDelay &d, double fracOfLength, double sign) {
            const int k = std::max(1, (int)std::lround(d.length * fracOfLength));
            return sign * d.readTap(k);
        };
        auto tapAP = [](const PlateAllpass &a, double fracOfLength, double sign) {
            const int k = std::max(1, (int)std::lround(a.length * fracOfLength));
            return sign * a.readTap(k);
        };

        // Fractions below are Dattorro's own Table 2 offsets divided by
        // his own corresponding segment's length (preserving his own
        // exact tap POSITIONS proportionally, since this engine's own
        // segment lengths are themselves a scaled -- not identical --
        // copy of his).
        double yL = 0.0;
        yL += tap(delayB1_, 266.0 / 4217.0, +1.0);
        yL += tap(delayB1_, 2974.0 / 4217.0, +1.0);
        yL += tapAP(allpassB2_, 1913.0 / 2656.0, -1.0);
        yL += tap(delayB2_, 1996.0 / 3163.0, +1.0);
        yL += tap(delayA1_, 1990.0 / 4453.0, -1.0);
        yL += tapAP(allpassA2_, 187.0 / 1800.0, -1.0);
        yL += tap(delayA2_, 1066.0 / 3720.0, -1.0);

        double yR = 0.0;
        yR += tap(delayA1_, 353.0 / 4453.0, +1.0);
        yR += tap(delayA1_, 3627.0 / 4453.0, +1.0);
        yR += tapAP(allpassA2_, 1228.0 / 1800.0, -1.0);
        yR += tap(delayA2_, 2673.0 / 3720.0, +1.0);
        yR += tap(delayB1_, 2111.0 / 4217.0, -1.0);
        yR += tapAP(allpassB2_, 335.0 / 2656.0, -1.0);
        yR += tap(delayB2_, 121.0 / 3163.0, -1.0);

        // kTapGain is Dattorro's own per-tap weight (0.6 in his own
        // paper); kOutputMakeupGain is a separate, purely loudness-
        // matching constant against reverb/reverb_dense's own
        // steady-state RMS (his own 0.6 was calibrated for his own
        // system's own gain staging, not this plugin's) -- see
        // tests/reverb_regression_plate.cpp's own [P4].
        outL = kTapGain * kOutputMakeupGain * yL;
        outR = kTapGain * kOutputMakeupGain * yR;

        // Width/Balance -- identical formulas to ReverbEngine's own (see
        // its setWidth()/setBalance() comments), applied to this
        // engine's own final L/R taps instead of an N-line sum.
        const double mid = 0.5 * (outL + outR);
        const double side = 0.5 * (outL - outR) * widthAmount_;
        outL = mid + side;
        outR = mid - side;

        outL *= 1.0 - std::max(0.0, balanceAmount_);
        outR *= 1.0 - std::max(0.0, -balanceAmount_);
    }

    // The 12 designed segment lengths (4 mono input-diffusion stages,
    // then tank A's mod-allpass/delay1/decay-diffusion2-allpass/delay2,
    // then tank B's) -- the plate's own analogue of ReverbEngine's own
    // lengths(), used the same way in tests/verification (sanity-check
    // bounds, confirm a room-size change actually changes them).
    std::array<int, 12> lengths() const {
        return {lenInDiff1_, lenInDiff2_, lenInDiff3_, lenInDiff4_,
                lenModA_, lenDelayA1_, lenAllpassA2_, lenDelayA2_,
                lenModB_, lenDelayB1_, lenAllpassB2_, lenDelayB2_};
    }

private:
    void applyDiffusionGains() {
        const double amt = std::clamp(diffusionAmount_, 0.0, 1.0);
        inDiff1_.setGain(amt * kInputDiffusion1Gain);
        inDiff2_.setGain(amt * kInputDiffusion1Gain);
        inDiff3_.setGain(amt * kInputDiffusion2Gain);
        inDiff4_.setGain(amt * kInputDiffusion2Gain);
    }

    // The "M" OnePoleDamping::decayFactor() is designed against needs to
    // be the TRUE number of samples elapsed between successive
    // applications of the damping filter, not just the nominal sum of
    // this tank's own delay-line lengths -- modAllpassA_/B_ is an
    // ALLPASS filter, whose own group delay -- especially near DC --
    // runs well past its nominal buffer length M (an allpass's
    // low-frequency group delay is M*(1+g)/(1-g), e.g. ~5.7x its
    // nominal M at g=0.7): summing the nominal loop length alone
    // undercounts the true elapsed time per damping application.
    // kGroupDelayCorrection is a measured ratio (see
    // tests/reverb_regression_plate.cpp's own [P2], T60(DC) accuracy),
    // applied only to the LENGTH FED INTO THE DAMPING DESIGN -- the
    // actual buffer lengths (lenModA_ etc., and everything lengths()
    // reports) are untouched, since those still have to match the real
    // allocated storage.
    static constexpr double kGroupDelayCorrection = 1.15;
    int dampDesignLenA() const { return std::max(1, (int)std::lround(loopLenA_ * kGroupDelayCorrection)); }
    int dampDesignLenB() const { return std::max(1, (int)std::lround(loopLenB_ * kGroupDelayCorrection)); }

    void applyPendingRoom() {
        RoomDims room;
        room.lengthM = pendingLengthM_.load(std::memory_order_relaxed);
        room.widthM = pendingWidthM_.load(std::memory_order_relaxed);
        room.heightM = pendingHeightM_.load(std::memory_order_relaxed);
        room.clamp();
        prepare(room, sampleRate_, lastT60Dc_, lastT60Ny_);
        roomChangePending_.store(false, std::memory_order_relaxed);
    }

    static constexpr double kCoeffSmoothTimeConstantSec = 0.005;
    static constexpr double kDuckFadeTimeSec = 0.008;

    // Dattorro's own Table 1 defaults -- see this file's own header
    // comment on why these stay fixed constants rather than being
    // derived from the Decay/Diffusion knobs.
    static constexpr double kInputDiffusion1Gain = 0.750;
    static constexpr double kInputDiffusion2Gain = 0.625;
    static constexpr double kDecayDiffusion1Gain = 0.70;
    static constexpr double kDecayDiffusion2Gain = 0.50;
    // Dattorro's own Table 2 per-tap weight and a separate, purely
    // loudness-matching makeup gain -- see processStereo()'s own
    // comment.
    static constexpr double kTapGain = 0.6;
    static constexpr double kOutputMakeupGain = 0.85;

    double sampleRate_ = 48000.0;
    double smoothCoeff_ = 0.0;
    double lastT60Dc_ = 1.8, lastT60Ny_ = 0.9;

    PlateAllpass inDiff1_, inDiff2_, inDiff3_, inDiff4_;
    double diffusionAmount_ = 0.7;
    double modulationAmount_ = 0.3;
    double modDepthBase_ = 8.0; // recomputed every prepare() from room/sample-rate scale
    double widthAmount_ = 1.0;
    double balanceAmount_ = 0.0;

    ModulatedPlateAllpass modAllpassA_, modAllpassB_;
    PlateDelay delayA1_, delayA2_, delayB1_, delayB2_;
    PlateAllpass allpassA2_, allpassB2_; // "decay diffusion 2" -- non-recirculating, tap-only
    OnePoleDamping dampA_, dampB_;
    double tankAFeedback_ = 0.0, tankBFeedback_ = 0.0;

    int lenInDiff1_ = 1, lenInDiff2_ = 1, lenInDiff3_ = 1, lenInDiff4_ = 1;
    int lenModA_ = 1, lenDelayA1_ = 1, lenAllpassA2_ = 1, lenDelayA2_ = 1;
    int lenModB_ = 1, lenDelayB1_ = 1, lenAllpassB2_ = 1, lenDelayB2_ = 1;
    int loopLenA_ = 1, loopLenB_ = 1;

    std::atomic<bool> roomChangePending_{false};
    std::atomic<double> pendingLengthM_{10.0};
    std::atomic<double> pendingWidthM_{7.0};
    std::atomic<double> pendingHeightM_{3.5};
    DuckState duckState_ = DuckState::Idle;
    double duckGain_ = 1.0;
    double duckStepPerSample_ = 1.0 / (kDuckFadeTimeSec * 48000.0);
    std::atomic<bool> faultFlag_{false};
};

} // namespace plate
} // namespace reverb
} // namespace ssfilter
