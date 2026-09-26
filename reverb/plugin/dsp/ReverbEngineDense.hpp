// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ReverbEngineDense.hpp
//
// This plugin's own official name is "Reverb Dense" (`reverb_dense`,
// renamed from its original "reverb_smooth"). The engine itself: a 16-line FDN, architecturally
// identical to plugin/dsp/ReverbEngine.hpp's own original 8-line design
// (that engine's own plugin, an unnamed-in-the-GUI "Reverb," was removed
// once this plugin and reverb_plate both sounded clearly better --
// ReverbEngine.hpp itself stays, as the shared foundation both
// remaining engines still #include): same Householder-mix/per-line-
// damping/allpass-pre-diffuser/delay-line-modulation design, same
// room-dimension-driven length design, same thread-safe room-change/duck
// handling, same Diffusion/Width/Balance/Modulation controls, same
// tempo-sync math -- only the line count differs, so a user can choose
// between models by sound and by CPU cost.
//
// Why a real 16-line engine class, not the 8-line one wearing a
// different room-size or a runtime N: mode density (how smoothly the
// tank's own resonances blend into noise-like decay instead of staying
// audible as discrete ringing tones -- see ReverbEngine.hpp's own
// "Delay-line modulation" section, and the research it's grounded in)
// scales with the *number of lines*, not their length. Doubling the line
// count roughly doubles the per-sample CPU cost (still O(N), just a
// bigger N) for a real, audible increase in tail smoothness/density --
// a sound-vs-CPU tradeoff: this is the
// plugin for someone who wants denser/smoother and is willing to pay
// roughly 2x the original 8-line design's own DSP cost for it.
//
// Why a genuine, separate, hand-adapted copy of ReverbEngine's own class
// body, not a C++ template parameterized on line count: this project's
// own established precedent for exactly this situation
// (plugin/dsp/MonoStabilityGuard.hpp/.cpp, in the sibling ../statespace
// project) is "a deliberate, straightforward copy... rather than a
// templating of it -- the [original] is the single most bug-prone and
// most carefully-fixed file in this project... and making it generic
// would have meant editing it." ReverbEngine.hpp carries an equivalent
// amount of hard-won, already-tested correctness (the thread-safe room-
// change duck logic, the balanced L/R channel assignment fix, the
// tempo-sync/coefficient-smoothing interplay) -- retrofitting genericity
// onto it risks all of that for a second model's sake. Every genuinely
// N-independent piece (OnePoleDamping, AllpassStage/AllpassDiffuser,
// ModulatedDelay, RoomDims, TempoNoteValue/t60SecFromTempo,
// firstOrderImageSources, DuckState) is reused unchanged via #include,
// not duplicated -- only the line-count-dependent pieces
// (designDelayLengths16(), householderMix16(), the engine class itself)
// are a genuine, from-scratch-adapted copy, verified by its own full
// tests/reverb_regression_dense.cpp suite (mirroring the original's own
// [R1]-[R16], not just assumed to still hold for a different N).

#pragma once

#include "ReverbEngine.hpp" // OnePoleDamping, AllpassStage, AllpassDiffuser, ModulatedDelay, RoomDims, TempoNoteValue, t60SecFromTempo, firstOrderImageSources, DuckState, kMaxDelaySamples, kSpeedOfSoundMPerSec, isPrime, nearestPrimeAtLeast -- all N-independent, reused as-is

namespace ssfilter {
namespace reverb {
namespace dense {

constexpr int kNumLines16 = 16;

// Same balanced "snake" assignment as ReverbEngine.hpp's own
// designDelayLengths() (see that function's own comment for the full
// derivation) -- generalized to N=16 rather than hand-picked: blocks of
// 4 consecutive ascending-sorted spread values, pattern L,R,R,L within
// each block. Verified directly (a scratch calculation, both channel
// sums equal to 6 significant figures), the identical verification
// method the N=8 table itself was checked with -- not assumed to
// generalize just because the block-symmetry argument sounds right.
inline std::array<int, kNumLines16> designDelayLengths16(const RoomDims &room, double sampleRate) {
    const double mfpSec = meanFreePathM(room) / kSpeedOfSoundMPerSec;
    const double baseSamples = mfpSec * sampleRate;
    static constexpr int kItemForArrayIndex[kNumLines16] = {0, 1, 3, 2, 4, 5, 7, 6, 8, 9, 11, 10, 12, 13, 15, 14};

    std::array<int, kNumLines16> lens{};
    for (int i = 0; i < kNumLines16; ++i) {
        const int k = kItemForArrayIndex[i];
        const double spread = 0.55 + 1.85 * (double)k / (double)(kNumLines16 - 1);
        int raw = (int)std::lround(baseSamples * spread);
        raw = std::clamp(raw, 8, kMaxDelaySamples - 1);
        lens[i] = std::min(nearestPrimeAtLeast(raw), kMaxDelaySamples - 1);
    }
    return lens;
}

inline void householderMix16(std::array<double, kNumLines16> &v) {
    double sum = 0.0;
    for (int i = 0; i < kNumLines16; ++i) sum += v[i];
    const double c = 2.0 * sum / (double)kNumLines16;
    for (int i = 0; i < kNumLines16; ++i) v[i] -= c;
}

// The late-tail engine itself -- see this file's own header comment for
// the full design rationale (identical to ReverbEngine's own, just N=16
// throughout instead of N=8).
class ReverbEngineDense {
public:
    void prepare(const RoomDims &room, double sampleRate, double t60DcSec, double t60NySec) {
        sampleRate_ = sampleRate;
        smoothCoeff_ = 1.0 - std::exp(-1.0 / (kCoeffSmoothTimeConstantSec * sampleRate));
        duckStepPerSample_ = 1.0 / (kDuckFadeTimeSec * sampleRate);
        diffuserL_.designLengths(sampleRate);
        diffuserR_.designLengths(sampleRate);
        diffuserL_.setAmount(diffusionAmount_);
        diffuserR_.setAmount(diffusionAmount_);
        lengths_ = designDelayLengths16(room, sampleRate);
        lastT60Dc_ = t60DcSec;
        lastT60Ny_ = t60NySec;
        for (int i = 0; i < kNumLines16; ++i) {
            buf_[i].fill(0.0);
            writeIdx_[i] = 0;
            const double r0 = OnePoleDamping::decayFactor(lengths_[i], t60DcSec, sampleRate);
            const double rNy = OnePoleDamping::decayFactor(lengths_[i], t60NySec, sampleRate);
            damp_[i].design(r0, rNy);
            damp_[i].snapToTarget();
            damp_[i].reset();

            static constexpr double kModRateHz[kNumLines16] = {
                0.09, 0.13, 0.11, 0.19, 0.15, 0.23, 0.12, 0.21,
                0.17, 0.29, 0.14, 0.25, 0.18, 0.31, 0.16, 0.27};
            mod_[i].reset();
            mod_[i].rate = kModRateHz[i];
            mod_[i].phase = double(i) / double(kNumLines16);
            mod_[i].depthSamples = modulationAmount_ * kMaxModDepthSamples;
        }
    }

    void setDecayTargets(double t60DcSec, double t60NySec) {
        lastT60Dc_ = t60DcSec;
        lastT60Ny_ = t60NySec;
        for (int i = 0; i < kNumLines16; ++i) {
            const double r0 = OnePoleDamping::decayFactor(lengths_[i], t60DcSec, sampleRate_);
            const double rNy = OnePoleDamping::decayFactor(lengths_[i], t60NySec, sampleRate_);
            damp_[i].design(r0, rNy);
        }
    }

    void setModulation(double amount01) {
        modulationAmount_ = std::clamp(amount01, 0.0, 1.0);
        for (auto &m : mod_) m.depthSamples = modulationAmount_ * kMaxModDepthSamples;
    }
    double modulation() const { return modulationAmount_; }

    void setDiffusion(double amount01) {
        diffusionAmount_ = std::clamp(amount01, 0.0, 1.0);
        diffuserL_.setAmount(diffusionAmount_);
        diffuserR_.setAmount(diffusionAmount_);
    }
    double diffusion() const { return diffusionAmount_; }

    void setWidth(double width) { widthAmount_ = std::clamp(width, 0.0, 2.0); }
    double width() const { return widthAmount_; }

    void setBalance(double balance) { balanceAmount_ = std::clamp(balance, -1.0, 1.0); }
    double balance() const { return balanceAmount_; }

    void snapDecayTargets(double t60DcSec, double t60NySec) {
        setDecayTargets(t60DcSec, t60NySec);
        for (int i = 0; i < kNumLines16; ++i) damp_[i].snapToTarget();
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
        const double diffusedL = diffuserL_.process(inL);
        const double diffusedR = diffuserR_.process(inR);

        std::array<double, kNumLines16> read{};
        for (int i = 0; i < kNumLines16; ++i) read[i] = buf_[i][writeIdx_[i]];

        std::array<double, kNumLines16> mixed = read;
        householderMix16(mixed);

        outL = 0.0;
        outR = 0.0;
        for (int i = 0; i < kNumLines16; ++i) {
            double toLine = mixed[i] + ((i % 2 == 0) ? diffusedL : diffusedR);
            damp_[i].smoothTowardTarget(smoothCoeff_);
            double damped = damp_[i].process(toLine);
            damped = mod_[i].process(damped, sampleRate_);
            if (!std::isfinite(damped)) { damped = 0.0; faultFlag_.store(true, std::memory_order_relaxed); }
            buf_[i][writeIdx_[i]] = damped;
            writeIdx_[i] = (writeIdx_[i] + 1 >= lengths_[i]) ? 0 : writeIdx_[i] + 1;
            if (i % 2 == 0) outL += read[i]; else outR += read[i];
        }
        const double norm = 2.0 / (double)kNumLines16;
        outL *= norm;
        outR *= norm;

        const double mid = 0.5 * (outL + outR);
        const double side = 0.5 * (outL - outR) * widthAmount_;
        outL = mid + side;
        outR = mid - side;

        outL *= 1.0 - std::max(0.0, balanceAmount_);
        outR *= 1.0 - std::max(0.0, -balanceAmount_);
    }

    const std::array<int, kNumLines16> &lengths() const { return lengths_; }

private:
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

    double sampleRate_ = 48000.0;
    double smoothCoeff_ = 0.0;
    double lastT60Dc_ = 1.8, lastT60Ny_ = 0.9;
    std::array<int, kNumLines16> lengths_{};
    std::array<std::array<double, kMaxDelaySamples>, kNumLines16> buf_{};
    std::array<int, kNumLines16> writeIdx_{};
    std::array<OnePoleDamping, kNumLines16> damp_;
    std::array<ModulatedDelay, kNumLines16> mod_;

    std::atomic<bool> roomChangePending_{false};
    std::atomic<double> pendingLengthM_{10.0};
    std::atomic<double> pendingWidthM_{7.0};
    std::atomic<double> pendingHeightM_{3.5};
    DuckState duckState_ = DuckState::Idle;
    double duckGain_ = 1.0;
    double duckStepPerSample_ = 1.0 / (kDuckFadeTimeSec * 48000.0);
    std::atomic<bool> faultFlag_{false};

    AllpassDiffuser diffuserL_, diffuserR_;
    double diffusionAmount_ = 0.7;
    double modulationAmount_ = 0.3;
    static constexpr double kMaxModDepthSamples = 6.0;
    double widthAmount_ = 1.0;
    double balanceAmount_ = 0.0;
};

} // namespace dense
} // namespace reverb
} // namespace ssfilter
