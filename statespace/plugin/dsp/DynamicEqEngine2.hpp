// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynamicEqEngine2.hpp
//
// The real-time audio engine for the dynamic EQ plugin -- a dedicated,
// purpose-built engine (like MonoStateSpaceEngine.hpp is for the mono EQ),
// NOT a mapper feeding the generic StateSpaceEngine, for the reason
// DynamicEqModel2.hpp's own file comment derives in full: a series cascade
// of Regalia-Mitra peaking sections whose gain moves every sample has no
// single fixed StateSpaceMatrices representation to hand that engine at
// all. Written to the same real-time rules as StateSpaceEngine.hpp: no
// locks, no allocation, no syscalls, no unbounded loops in the audio
// callback path.
//
// Per active band k (0 <= k < model.numBands), every sample:
//
//   1. Run the band's own fixed allpass A(z) (DynamicEqModel2.hpp's
//      designShaperAllpass(), depends only on freqHz/q) on this band's
//      *incoming* signal (i.e. after every earlier band in the cascade has
//      already been applied -- a real signal-chain series connection, not
//      an independent per-band tap the way MultibandSchedule's crossover
//      bands are).
//   2. Separately, run a genuine (magnitude-shaped) detector bandpass
//      (designDetectorBandpass()) on that *same* incoming signal, purely
//      to measure "how loud is this frequency region right now" --
//      max(|L|,|R|), one-pole attack/release smoothed in the linear
//      domain, then converted to dB -- the identical envelope-follower
//      shape StateSpaceEngine.hpp's own NonlinearSchedule/MultibandSchedule
//      handling already uses (see that file's own comment for why linear-
//      domain smoothing, not dB-domain, is the right one).
//   3. Resolve that envelope to a gain (dB) via
//      DynamicEqModel2.hpp's resolveDynamicGainDb() (threshold/ratio/range,
//      relative to the band's own resting gainDb), convert to linear G,
//      and combine: out = 0.5*(1+G)*in + 0.5*(1-G)*allpassOut -- the
//      Regalia-Mitra formula, see DynamicEqModel2.hpp's file comment for the
//      derivation. This becomes the next band's incoming signal.
//
// A band's own threshold/ratio/range/attack/release/gainDb are read from
// the *live* target model and switch instantly, exactly like
// NonlinearSchedule's own parameters do in StateSpaceEngine.hpp ("its
// output is already a smoothed envelope-driven scalar gain, not a
// recursive state, so there is no equivalent discontinuity risk"). The
// shaper/detector *filter coefficients* (freqHz/q-derived) are different:
// those feed a real IIR recursion, so -- mirroring StateSpaceEngine.hpp's
// own A/B/C/D coefficient-smoothing precisely, for the identical "a fast
// drag would otherwise apply a suddenly-different filter to whatever's
// already in that filter's own delay line" reason -- they're one-pole-
// smoothed toward their target every sample, using the same time constant.
//
// A *band-count* change (add/remove/enable/disable/solo -- anything that
// changes which/how-many bands are active) has no meaningful "in between"
// two different band lists could smoothly interpolate through, for
// exactly the same reason StateSpaceEngine.hpp gives for an EQ order
// change -- so it gets the identical fix: duck to silence, swap the whole
// per-band state array and snap every band's coefficients to its target
// while nothing is audible, fade back in.

#pragma once

#include "DynamicEqModel2.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace ssfilter {

// Biquad1State/processDF1() -- the same Direct-Form-I section every band
// here uses (shaper allpass + detector bandpass) -- now live in
// CoefficientMapper.hpp, next to BiquadCoeffs itself, so PluginEq's own
// Audition tap can reuse them too without depending on this whole engine.

// The engine's own input model: already resolved to an ordered, purely
// audible band list (see dyneq2::activeBandIndices() and
// PluginDynEq2::rebuildEngineModel()) -- the engine itself has no notion of
// enabled/solo, exactly like StateSpaceEngine has no notion of EqModel's
// band list, only the already-assembled result.
struct DynamicEqRtModel2
{
    int numBands = 0;
    DynamicEqBand2 bands[kMaxDynEqBands2]{}; // only [0, numBands) are meaningful
    double outputTrimDb = 0.0;

    // Crossfeed transit gains -- see DynamicEqModel2::crossFeedGainLtoRDb's
    // own comment. Copied straight through from that model-level field by
    // whoever assembles this struct (PluginDynEq2::rebuildEngineModel()),
    // same as outputTrimDb above.
    double crossFeedGainLtoRDb = -60.0;
    double crossFeedGainRtoLDb = -60.0;

    // Set by PluginDynEq2::rebuildEngineModel() specifically while a solo
    // is active and the soloed band is Left- or Right-only, so only that
    // channel's audio passes. Without this, soloing a channel-restricted band
    // correctly isolates its own filtering, but the channel it never
    // touches would otherwise just be a raw, unfiltered copy of the
    // input rather than silence -- this engine has no notion of "solo"
    // itself (see this file's own header comment on what it does and
    // doesn't know about the model), so the decision is made once, up
    // front, by whoever assembles this struct, not re-derived here every
    // block. False (both channels stay live) whenever nothing is soloed,
    // or the soloed band is Both -- identical to today's behavior.
    bool muteLeft = false;
    bool muteRight = false;
};

class DynamicEqEngine2
{
public:
    DynamicEqEngine2() = default;

    void prepare(double sampleRate) noexcept
    {
        sampleRate_ = sampleRate;
        resetState();
    }

    // Non-RT: install a new model. Same wait-free double-buffer + mutex +
    // version-counter mechanism as StateSpaceEngine::setModel() -- see that
    // file's own comment for the full reasoning.
    void setModel(const DynamicEqRtModel2& m)
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        const int live = activeSlot_.load(std::memory_order_relaxed);
        const int next = 1 - live;
        slots_[next] = m;
        activeSlot_.store(next, std::memory_order_release);
        version_.fetch_add(1, std::memory_order_release);
    }

    DynamicEqRtModel2 getModel() const
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        return slots_[activeSlot_.load(std::memory_order_acquire)];
    }

    // RT-safe: zero every band's filter/envelope state and snap smoothed
    // coefficients to the currently-active model -- same "explicit start
    // clean" contract as StateSpaceEngine::resetState().
    void resetState() noexcept
    {
        for (auto& st : shaperL_) st = Biquad1State{};
        for (auto& st : shaperR_) st = Biquad1State{};
        for (auto& st : shaper2L_) st = Biquad1State{};
        for (auto& st : shaper2R_) st = Biquad1State{};
        for (auto& st : detectorL_) st = Biquad1State{};
        for (auto& st : detectorR_) st = Biquad1State{};
        for (double& v : envLin_) v = 1e-9;
        for (auto& v : currentGainDb_) v.store(0.0, std::memory_order_relaxed);
        faultFlag_.store(false, std::memory_order_relaxed);

        const int slot = activeSlot_.load(std::memory_order_acquire);
        smoothedModel_ = slots_[slot];
        smoothedNumBands_ = smoothedModel_.numBands;
        for (int k = 0; k < smoothedNumBands_; ++k)
        {
            shaperCoeffs_[k] = dyneq2::designShaperAllpass(smoothedModel_.bands[k], sampleRate_);
            shaper2Coeffs_[k] = dyneq2::designShaperAllpass2(smoothedModel_.bands[k], sampleRate_);
            detectorCoeffs_[k] = dyneq2::designDetectorBandpass(smoothedModel_.bands[k], sampleRate_);
        }
        lastSeenVersion_ = version_.load(std::memory_order_acquire);
        duckState_ = DuckState::None;
        duckGain_ = 1.0;
        modelChangePending_ = false;
        // Snapped to whatever's live right now rather than forced to
        // "off" -- if Audition happens to already be engaged across this
        // reset (a host deactivate/reactivate mid-session, say), this
        // avoids an immediate, spurious duck being detected on the very
        // next block purely from the reset itself.
        committedAuditionValid_ = audition_;
        committedAuditionBandIndex_ = auditionBandIndex_;
    }

    bool consumeFaultFlag() noexcept { return faultFlag_.exchange(false, std::memory_order_relaxed); }

    // Preview the isolated detector output of one band (WITH its own live
    // dynamic gain reduction applied), for tuning Frequency/Gain/Threshold/
    // Range/Attack/Release by ear -- same idea, same "detector's own
    // magnitude-shaped bandpass output, scaled by G" formula, as the
    // (now-deleted) De-esser plugin's own DeesserEngine::setListen() used
    // (that engine's single fixed band is this one generalized to "which
    // band" being separately selectable, since this engine can have
    // several) -- renamed here (and on the sibling Parametric EQ plugin,
    // which later copied this feature) from "Listen" to "Audition":
    // "Listen" invited the wrong expectation, hearing what the band's own
    // EQ curve does to the mix, when this always actually previews the
    // raw frequency region the detector is centered on. `index`: which
    // slot in the cascade to preview, or < 0 for none. The rest of the
    // cascade keeps running exactly as normal underneath (every band's
    // filter state stays warm, downstream bands keep reacting to the
    // real signal) -- only the final output sample is swapped for the
    // preview while auditioning, so turning Audition back off is never a
    // cold-start jolt.
    void setAudition(bool on) noexcept { audition_ = on; }
    void setAuditionBandIndex(int index) noexcept { auditionBandIndex_ = index; }

    // RT-safe readback for the GUI's live/moving curve -- the gain (dB)
    // band `index` was actually running at as of the most recently
    // processed sample. Relaxed, single-writer(audio thread)/single-reader
    // (UI thread) atomics -- exactly consumeFaultFlag()'s own concurrency
    // contract, minus the exchange (this is read-many, not read-once).
    double getCurrentGainDb(int index) const noexcept
    {
        if (index < 0 || index >= kMaxDynEqBands2) return 0.0;
        return currentGainDb_[size_t(index)].load(std::memory_order_relaxed);
    }

    void processBlock(const float* inL, const float* inR,
                       float* outL, float* outR, uint32_t numFrames) noexcept
    {
        const int slot = activeSlot_.load(std::memory_order_acquire);
        const DynamicEqRtModel2& m = slots_[slot];
        const int targetN = std::max(0, std::min(kMaxDynEqBands2, m.numBands));

        // Same detect-a-new-target-since-last-block dance as
        // StateSpaceEngine::processBlock() -- see that file's own comment.
        const uint32_t ver = version_.load(std::memory_order_acquire);
        if (ver != lastSeenVersion_)
        {
            lastSeenVersion_ = ver;
            if (targetN == smoothedNumBands_)
            {
                smoothingActive_ = true;
            }
            else
            {
                pendingModel_ = m;
                pendingNumBands_ = targetN;
                smoothingActive_ = false;
                modelChangePending_ = true;
                if (duckState_ == DuckState::None)
                {
                    duckState_ = DuckState::FadingOut;
                    duckStep_ = 1.0 / (kDuckTimeSec * sampleRate_);
                }
            }
        }

        // Audition on/off (or which band it's previewing) is a second
        // kind of structural discontinuity, same reasoning as the
        // order-change case just above: swapping the final output from
        // "the real filtered signal" to "an isolated detector tap of one
        // frequency region" (or back) jumps between two essentially
        // unrelated waveforms -- that clicks just as audibly as an order
        // change does regardless of how continuous the underlying filter
        // states stay (the audition tap's own state is always running
        // live either way, see setAudition()'s own comment; that was
        // never what was clicking). Fixed the same way as the order-change
        // duck above: the
        // *live* audition_/auditionBandIndex_ requested by the UI thread
        // only actually takes effect (committedAuditionValid_/
        // committedAuditionBandIndex_, read by the per-sample body below
        // instead of the raw members) once a duck's fade-out reaches
        // silence, exactly mirroring smoothedModel_/smoothedNumBands_'s
        // own lag behind the live target model. Checked once per block,
        // same cadence as the model-version check above, not per sample --
        // audition_/auditionBandIndex_ are UI-thread-set controls with no
        // need for finer resolution than that.
        //
        // Compared by *effective* validity (range-checked against the
        // band count each side actually has), not the raw audition_ flag
        // -- an out-of-range auditionBandIndex_ was always a documented
        // no-op (falls straight back to the normal path, see
        // auditionValid below), and comparing raw flags alone would duck
        // even then: audition_ going false->true with nothing valid to
        // preview never actually changes the audible output at all, so
        // there is nothing there to hide behind a fade.
        const bool requestedAuditionValid = audition_ && auditionBandIndex_ >= 0 && auditionBandIndex_ < targetN;
        const bool effectiveAuditionValid = committedAuditionValid_ && committedAuditionBandIndex_ >= 0 && committedAuditionBandIndex_ < smoothedNumBands_;
        if (requestedAuditionValid != effectiveAuditionValid ||
            (requestedAuditionValid && auditionBandIndex_ != committedAuditionBandIndex_))
        {
            if (duckState_ == DuckState::None)
            {
                duckState_ = DuckState::FadingOut;
                duckStep_ = 1.0 / (kDuckTimeSec * sampleRate_);
            }
        }

        const double smoothCoeff = smoothingActive_
                                        ? (1.0 - std::exp(-1.0 / (kCoeffSmoothTimeConstantSec * sampleRate_)))
                                        : 0.0;

        // Attack/release coefficients and each band's target shaper/
        // detector coefficients only depend on this block's model, so
        // computed once per block, not per sample -- same reasoning as
        // StateSpaceEngine's own per-block nlAttackCoeff/nlReleaseCoeff.
        double attackCoeff[kMaxDynEqBands2] = {}, releaseCoeff[kMaxDynEqBands2] = {};
        BiquadCoeffs targetShaper[kMaxDynEqBands2], targetShaper2[kMaxDynEqBands2], targetDetector[kMaxDynEqBands2];
        for (int k = 0; k < targetN; ++k)
        {
            const DynamicEqBand2& b = m.bands[k];
            const double attackMs  = b.attackMs  > 0.05 ? b.attackMs  : 0.05;
            const double releaseMs = b.releaseMs > 1.0  ? b.releaseMs : 1.0;
            attackCoeff[k]  = std::exp(-1.0 / (0.001 * attackMs  * sampleRate_));
            releaseCoeff[k] = std::exp(-1.0 / (0.001 * releaseMs * sampleRate_));
            targetShaper[k] = dyneq2::designShaperAllpass(b, sampleRate_);
            targetShaper2[k] = dyneq2::designShaperAllpass2(b, sampleRate_);
            targetDetector[k] = dyneq2::designDetectorBandpass(b, sampleRate_);
        }

        // Output trim is a single fixed value for the whole block (it's
        // read straight from `m`, which this function never reassigns
        // mid-block) -- computed once here rather than, as before, calling
        // std::pow() on the exact same input every single sample for no
        // reason. Producing the identical value from the identical
        // std::pow(10.0, m.outputTrimDb / 20.0) call once instead of
        // numFrames times is bit-for-bit the same result, just without
        // the redundant work.
        const double outTrim = std::pow(10.0, m.outputTrimDb / 20.0);
        // Crossfeed transit gains -- ONE flat scalar per direction, read
        // straight from the model (never per-band, see
        // DynamicEqModel2::crossFeedGainLtoRDb's own comment) and applied
        // directly at each crossfeed band's own point of adding into the
        // destination channel below.
        const double crossGainLtoRLin = std::pow(10.0, m.crossFeedGainLtoRDb / 20.0);
        const double crossGainRtoLLin = std::pow(10.0, m.crossFeedGainRtoLDb / 20.0);

        for (uint32_t f = 0; f < numFrames; ++f)
        {
            // Duck state machine -- identical structure to
            // StateSpaceEngine::processBlock()'s own; see that file's
            // header comment for the full reasoning.
            double duckMul = 1.0;
            if (duckState_ == DuckState::FadingOut)
            {
                duckGain_ -= duckStep_;
                if (duckGain_ <= 0.0)
                {
                    duckGain_ = 0.0;
                    // Only actually swap the band list/reset filter state
                    // when an order-changing edit is what's pending --
                    // this same duck bottom is also reached for an
                    // audition-only change (see the trigger's own comment
                    // above), which has no state of its own to swap or
                    // reset; doing so anyway would cold-restart every
                    // band's filter memory on every Audition toggle for
                    // no reason, itself a fresh (if smaller) discontinuity
                    // right as the fade-in begins.
                    if (modelChangePending_)
                    {
                        smoothedModel_ = pendingModel_;
                        smoothedNumBands_ = pendingNumBands_;
                        for (auto& st : shaperL_) st = Biquad1State{};
                        for (auto& st : shaperR_) st = Biquad1State{};
                        for (auto& st : shaper2L_) st = Biquad1State{};
                        for (auto& st : shaper2R_) st = Biquad1State{};
                        for (auto& st : detectorL_) st = Biquad1State{};
                        for (auto& st : detectorR_) st = Biquad1State{};
                        for (double& v : envLin_) v = 1e-9;
                        for (int k = 0; k < smoothedNumBands_; ++k)
                        {
                            shaperCoeffs_[k] = dyneq2::designShaperAllpass(smoothedModel_.bands[k], sampleRate_);
                            shaper2Coeffs_[k] = dyneq2::designShaperAllpass2(smoothedModel_.bands[k], sampleRate_);
                            detectorCoeffs_[k] = dyneq2::designDetectorBandpass(smoothedModel_.bands[k], sampleRate_);
                        }
                        modelChangePending_ = false;
                    }
                    // Commits whatever's currently live, regardless of
                    // which of the two triggers actually started this
                    // duck -- a no-op reassignment if audition_/
                    // auditionBandIndex_ didn't change, and correctly
                    // picks up the latest request if the user toggled it
                    // more than once while this fade was already running.
                    committedAuditionValid_ = audition_;
                    committedAuditionBandIndex_ = auditionBandIndex_;
                    duckState_ = DuckState::FadingIn;
                }
                duckMul = duckGain_;
            }
            else if (duckState_ == DuckState::FadingIn)
            {
                duckGain_ += duckStep_;
                if (duckGain_ >= 1.0) { duckGain_ = 1.0; duckState_ = DuckState::None; }
                duckMul = duckGain_;
            }

            const int n = smoothedNumBands_; // the band count actually driving this sample's cascade, always matching smoothedModel_

            if (smoothingActive_)
            {
                double maxDiff = 0.0;
                auto smoothCoeffsOf = [&](BiquadCoeffs& s, const BiquadCoeffs& t) {
                    s.b0 += smoothCoeff * (t.b0 - s.b0); maxDiff = std::max(maxDiff, std::fabs(t.b0 - s.b0));
                    s.b1 += smoothCoeff * (t.b1 - s.b1); maxDiff = std::max(maxDiff, std::fabs(t.b1 - s.b1));
                    s.b2 += smoothCoeff * (t.b2 - s.b2); maxDiff = std::max(maxDiff, std::fabs(t.b2 - s.b2));
                    s.a1 += smoothCoeff * (t.a1 - s.a1); maxDiff = std::max(maxDiff, std::fabs(t.a1 - s.a1));
                    s.a2 += smoothCoeff * (t.a2 - s.a2); maxDiff = std::max(maxDiff, std::fabs(t.a2 - s.a2));
                };
                for (int k = 0; k < n; ++k)
                {
                    smoothCoeffsOf(shaperCoeffs_[k], targetShaper[k]);
                    smoothCoeffsOf(shaper2Coeffs_[k], targetShaper2[k]);
                    smoothCoeffsOf(detectorCoeffs_[k], targetDetector[k]);
                }
                if (maxDiff < 1e-9) smoothingActive_ = false;
            }

            double sigL = double(inL[f]);
            double sigR = double(inR[f]);
            double auditionL = 0.0, auditionR = 0.0; // captured from auditionBandIndex_'s own band (and any harmonic-linked bands, see below), if listening

            // Audition treats linked bands as one: if the auditioned band
            // belongs to a
            // harmonic group, every OTHER member of that same group gets
            // summed into the preview too, not just the one literally
            // selected -- computed fresh each sample (not once per block)
            // since committedAuditionBandIndex_ can itself change mid-block,
            // exactly at the sample a pending audition-change duck bottoms
            // out (see this file's own comment on that trigger, above).
            // Read from `m` (the LIVE target), not smoothedModel_, when
            // in range of it -- same defensive resolution `liveBand`
            // itself uses a few lines below, and for the same reason:
            // harmonicGroup is a discrete grouping flag, not a filter
            // coefficient, so it has no business lagging behind a
            // same-order edit the way shaper/detector coefficients
            // legitimately do during smoothingActive_ (a *duck* is what
            // updates smoothedModel_, and toggling a band's own link
            // membership never changes the band count, so it never
            // ducks at all -- smoothedModel_ would otherwise silently
            // never see the new value).
            const int auditionGroup = (committedAuditionValid_ && committedAuditionBandIndex_ >= 0 && committedAuditionBandIndex_ < n)
                                           ? ((committedAuditionBandIndex_ < targetN) ? m.bands[committedAuditionBandIndex_].harmonicGroup
                                                                                       : smoothedModel_.bands[committedAuditionBandIndex_].harmonicGroup)
                                           : 0;

            for (int k = 0; k < n; ++k)
            {
                // During a duck's fade-out, m/target arrays above are sized
                // for targetN, which may differ from n (=smoothedNumBands_)
                // -- read this band's own live parameters defensively, not
                // by indexing m.bands[k] when k could exceed targetN.
                const DynamicEqBand2& liveBand = (k < targetN) ? m.bands[k] : smoothedModel_.bands[k];

                // Always run both channels' own shaper/detector filters,
                // regardless of which one(s) this band's channel actually
                // uses below -- rather than skipping the unused side's
                // processDF1() calls entirely -- so that side's own filter
                // memory stays continuously warm against the live signal.
                // That means flipping a band's channel assignment live
                // (Both/Left/Right, see EqChannel's own comment) never
                // resumes the newly-affected channel from stale, silently-
                // aged state -- only the *use* of each result below is
                // gated by channel, never the computation itself.
                const double apOutL = processDF1(shaperL_[k], shaperCoeffs_[k], sigL);
                const double apOutR = processDF1(shaperR_[k], shaperCoeffs_[k], sigR);
                const double detOutL = processDF1(detectorL_[k], detectorCoeffs_[k], sigL);
                const double detOutR = processDF1(detectorR_[k], detectorCoeffs_[k], sigR);

                // Both (the default): the original combined-channel
                // behavior, unchanged -- one shared level (the louder of
                // the two channels) drives one shared gain, applied to
                // both. Left/Right: this band reads and reacts to only
                // that one channel's own level, completely independent of
                // whatever the other channel happens to be doing.
                // Crossfeed (see DynamicEqBand2::crossFeed's own comment):
                // `channel` is ignored, same convention as the static EQ's
                // own crossfeed -- level detection instead reads the
                // SOURCE channel only (L for LeftToRight, R for
                // RightToLeft), since that's the signal actually being
                // shaped and sent across.
                const EqChannel ch = liveBand.channel;
                const EqCrossFeed cf = liveBand.crossFeed;
                const bool isCrossFeed = cf != EqCrossFeed::None;
                const bool usesL = isCrossFeed ? (cf == EqCrossFeed::LeftToRight) : (ch != EqChannel::Right);
                const bool usesR = isCrossFeed ? (cf == EqCrossFeed::RightToLeft) : (ch != EqChannel::Left);
                const double levelLin = isCrossFeed
                                       ? (usesL ? std::fabs(detOutL) : std::fabs(detOutR))
                                       : (ch == EqChannel::Left)  ? std::fabs(detOutL)
                                       : (ch == EqChannel::Right) ? std::fabs(detOutR)
                                       : std::max(std::fabs(detOutL), std::fabs(detOutR));
                const double coeff = (k < targetN)
                                          ? ((levelLin > envLin_[k]) ? attackCoeff[k] : releaseCoeff[k])
                                          : ((levelLin > envLin_[k]) ? 0.99 : 0.999); // defensive fallback during the rare duck-transition read above; inaudible (duckMul is ~0 here)
                envLin_[k] = coeff * envLin_[k] + (1.0 - coeff) * levelLin;
                const double envDb = 20.0 * std::log10(std::max(envLin_[k], 1e-9));

                const double gainDb = dyneq2::resolveDynamicGainDb(liveBand, envDb);
                currentGainDb_[size_t(k)].store(gainDb, std::memory_order_relaxed);
                const double G = std::pow(10.0, gainDb / 20.0);
                // +1 for Bell/High Shelf, -1 for Low Shelf -- see
                // dyneq2::shaperAllpassSign()'s own comment for why the
                // SAME combination formula below produces either shape
                // purely from this sign.
                const double sign = dyneq2::shaperAllpassSign(liveBand.type);

                // Band Shelf: two shelves in series plus a flat 1/G -- see
                // dyneq2::designShaperAllpass2()'s comment for the formula.
                // Stage 1 (s1) and the second allpass run for both channels,
                // like apOutL/R above; shape() then gives the band's output
                // for one channel, for every type.
                const bool isBandShelf = liveBand.type == EqBandType::BandShelf;
                double s1L = 0.0, s1R = 0.0, ap2L = 0.0, ap2R = 0.0;
                if (isBandShelf)
                {
                    s1L = 0.5 * (1.0 + G) * sigL + 0.5 * (1.0 - G) * apOutL;
                    s1R = 0.5 * (1.0 + G) * sigR + 0.5 * (1.0 - G) * apOutR;
                    ap2L = processDF1(shaper2L_[k], shaper2Coeffs_[k], s1L);
                    ap2R = processDF1(shaper2R_[k], shaper2Coeffs_[k], s1R);
                }
                const double invG = 1.0 / G; // G = 10^(dB/20) > 0 always
                auto shape = [&](double x, double apOut, double s1, double ap2) {
                    if (isBandShelf) return (0.5 * (1.0 + G) * s1 - 0.5 * (1.0 - G) * ap2) * invG;
                    return 0.5 * (1.0 + G) * x + sign * 0.5 * (1.0 - G) * apOut;
                };

                if (committedAuditionValid_ &&
                    (k == committedAuditionBandIndex_ || (auditionGroup != 0 && liveBand.harmonicGroup == auditionGroup)))
                {
                    auditionL += detOutL * G;
                    auditionR += detOutR * G;
                }

                if (isCrossFeed)
                {
                    // This band's own Regalia-Mitra shape (the SAME
                    // formula every ordinary band uses to filter its own
                    // channel -- a dry+bump peaking combination, unity at
                    // rest when gainDb=0), applied to the SOURCE channel's
                    // own current (already-cascaded) signal, scaled by the
                    // model's own crossfeed transit gain and ADDED into
                    // the DESTINATION channel -- on top of whatever that
                    // channel already has, never replacing it. Exactly the
                    // additive "own channel's content, added elsewhere"
                    // story eq::assembleStateSpace()'s state-space graft
                    // tells (see EqBand::crossFeed's own comment there),
                    // just as a per-sample addition instead of a matrix
                    // graft. The band's own source channel is left
                    // completely unmodified by it, same as a state-space
                    // crossfeed band never alters its own source's C row.
                    if (cf == EqCrossFeed::LeftToRight)
                    {
                        sigR += crossGainLtoRLin * shape(sigL, apOutL, s1L, ap2L);
                    }
                    else // RightToLeft
                    {
                        sigL += crossGainRtoLLin * shape(sigR, apOutR, s1R, ap2R);
                    }
                }
                else
                {
                    // A channel this band doesn't use passes straight
                    // through completely untouched by it (no allpass phase
                    // shift, no gain) -- not "shaped with G=1", which the
                    // Regalia-Mitra formula would already reduce to at G=1
                    // but only exactly at that one gain value; leaving the
                    // assignment out entirely is both cheaper and correct
                    // for every G.
                    if (usesL) sigL = shape(sigL, apOutL, s1L, ap2L);
                    if (usesR) sigR = shape(sigR, apOutR, s1R, ap2R);
                }
            }

            // Deliberately skips outTrim while auditioning, same as the
            // (now-deleted) De-esser plugin's DeesserEngine did -- this is
            // a monitoring aid, not the plugin's actual output stage.
            const bool auditionValid = committedAuditionValid_ && committedAuditionBandIndex_ >= 0 && committedAuditionBandIndex_ < n;
            double y0 = auditionValid ? auditionL : sigL * outTrim;
            double y1 = auditionValid ? auditionR : sigR * outTrim;

            if (!(std::isfinite(y0) && std::isfinite(y1)))
            {
                faultFlag_.store(true, std::memory_order_relaxed);
                for (auto& st : shaperL_) st = Biquad1State{};
                for (auto& st : shaperR_) st = Biquad1State{};
                for (auto& st : shaper2L_) st = Biquad1State{};
                for (auto& st : shaper2R_) st = Biquad1State{};
                for (auto& st : detectorL_) st = Biquad1State{};
                for (auto& st : detectorR_) st = Biquad1State{};
                for (double& v : envLin_) v = 1e-9;
                y0 = 0.0; y1 = 0.0;
                duckState_ = DuckState::None;
                duckGain_ = 1.0;
            }
            else
            {
                constexpr double kCeiling = 64.0; // last-resort ceiling, same convention/value as StateSpaceEngine's own
                if (y0 > kCeiling) y0 = kCeiling; else if (y0 < -kCeiling) y0 = -kCeiling;
                if (y1 > kCeiling) y1 = kCeiling; else if (y1 < -kCeiling) y1 = -kCeiling;
            }

            // Solo-with-channel-routing silence -- see DynamicEqRtModel2::
            // muteLeft/muteRight's own comment. An unconditional override
            // applied last, after everything else (audition, the ceiling
            // clamp, the duck fade) -- exactly "this channel is silent
            // right now", not merely attenuated by whatever the rest of
            // the cascade happened to be doing.
            if (m.muteLeft) y0 = 0.0;
            if (m.muteRight) y1 = 0.0;

            outL[f] = float(y0 * duckMul);
            outR[f] = float(y1 * duckMul);
        }
    }

private:
    static constexpr double kCoeffSmoothTimeConstantSec = 0.005; // same value/reasoning as StateSpaceEngine's own
    static constexpr double kDuckTimeSec = 0.008;                // same value/reasoning as StateSpaceEngine's own

    std::array<DynamicEqRtModel2, 2> slots_{};
    std::atomic<int> activeSlot_{0};
    std::atomic<uint32_t> version_{0};
    mutable std::mutex writeMutex_;

    double sampleRate_ = 44100.0;
    std::atomic<bool> faultFlag_{false};

    // Set from the UI thread via setAudition()/setAuditionBandIndex() only --
    // read from the audio thread every sample, same relaxed single-writer/
    // single-reader convention every plain (non-std::atomic) cross-thread
    // scalar in this project family uses. Not behind writeMutex_: unlike
    // slots_, tearing either of these mid-read only ever costs one sample
    // of a stale preview target, never a structural inconsistency.
    bool audition_ = false;
    int auditionBandIndex_ = -1;

    // The audition state actually driving the per-sample body below --
    // lags behind the live audition_/auditionBandIndex_ above until a
    // duck's fade-out reaches silence, exactly mirroring smoothedModel_/
    // smoothedNumBands_'s own lag behind the live target model. See the
    // audition-change duck trigger's own comment (processBlock()) for
    // why this exists: a raw instant switch clicked.
    bool committedAuditionValid_ = false;
    int committedAuditionBandIndex_ = -1;

    Biquad1State shaperL_[kMaxDynEqBands2]{}, shaperR_[kMaxDynEqBands2]{};
    Biquad1State shaper2L_[kMaxDynEqBands2]{}, shaper2R_[kMaxDynEqBands2]{}; // Band Shelf's second allpass, see designShaperAllpass2()
    Biquad1State detectorL_[kMaxDynEqBands2]{}, detectorR_[kMaxDynEqBands2]{};
    BiquadCoeffs shaperCoeffs_[kMaxDynEqBands2]{}, shaper2Coeffs_[kMaxDynEqBands2]{}, detectorCoeffs_[kMaxDynEqBands2]{};
    double envLin_[kMaxDynEqBands2] = {};
    std::array<std::atomic<double>, kMaxDynEqBands2> currentGainDb_{};

    DynamicEqRtModel2 smoothedModel_;
    int smoothedNumBands_ = 0;
    bool smoothingActive_ = false;
    uint32_t lastSeenVersion_ = 0;

    enum class DuckState : uint8_t { None, FadingOut, FadingIn };
    DuckState duckState_ = DuckState::None;
    double duckGain_ = 1.0;
    double duckStep_ = 0.0;
    DynamicEqRtModel2 pendingModel_;
    int pendingNumBands_ = 0;
    // True from the moment an order-changing model edit is detected until
    // the current duck's fade-out bottoms out and actually applies it --
    // guards the state-swap-and-reset block at that bottom so an
    // audition-only duck (see above) doesn't also cold-restart every
    // band's filter memory for no reason.
    bool modelChangePending_ = false;
};

} // namespace ssfilter
