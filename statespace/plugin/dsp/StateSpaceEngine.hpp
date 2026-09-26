// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// StateSpaceEngine.hpp
//
// The real-time audio engine. Everything here is written to the standard
// real-time-audio rules: no locks, no allocation, no syscalls, no unbounded
// loops in the audio callback path (processBlock's cost is a fixed
// O(order^2) per sample, order <= kMaxOrder).
//
// Coefficient updates arrive from a non-real-time thread (GUI / state
// loader) via setModel(), which is expected to have already been passed
// through StabilityGuard::enforce(). The engine itself does not re-run the
// stability guard (that would be wasted work per update and, worse, would
// silently diverge from whatever the GUI is displaying as "the model") --
// it only defends the audio thread against genuine floating-point faults
// (NaN/Inf) with a cheap per-sample check, because no amount of upstream
// validation can prove that in *all* cases (e.g. extreme sustained input,
// accumulated float error over long runs) a recursive filter stays finite
// forever. If a fault is ever detected, the state vector is reset and
// silence is substituted for that block's remaining output, which is far
// preferable to propagating NaN/Inf into the host.
//
// Coefficient hand-off uses a wait-free double buffer: two preallocated
// model slots, an atomic index saying which one is "live". The writer
// (non-RT) fills the *other* slot, then flips the index with release
// ordering; the audio thread loads the index once per block with acquire
// ordering and uses that slot for the whole block. No locks are ever taken
// on the audio thread.
//
// A model's optional NonlinearSchedule (StateSpaceTypes.hpp) is applied
// here too: a state-dependent gain multiplies the already-computed y[n]
// every sample, driven by an envelope of the input level. `A`/`B` (and
// the state recursion's stability) are never touched by it.
//
// A model's optional MultibandSchedule (StateSpaceTypes.hpp) is the same
// idea generalized to N independent bands: each band computes its own
// pre-gain contribution from its own slice of the state vector (populated
// by a crossover mapper, e.g. mappers::makeStereoCrossoverBank() in
// CoefficientMapper.hpp), tracks its own envelope of its own filtered
// signal, and applies its own gain curve, before all bands' contributions
// sum into y[n]. Again, `A`/`B` are never touched -- only which
// already-computed state contributions get scaled before summing.
//
// A/B/C/D are *smoothed* toward each newly-submitted model, not switched
// to instantly, whenever the new model's order matches the one currently
// running -- i.e. every ordinary "drag a coefficient/EQ band" edit, which
// never changes state-vector size. Without this, a fast edit (a quick EQ
// drag covering a wide frequency/gain range, say) submits a target whose
// coefficients differ substantially from what the engine was just
// running; StabilityGuard has already made sure that target is stable in
// isolation, but *switching to it between one sample and the next* still
// applies a suddenly-different C/D to the state `x[n]` the *previous*
// coefficients had been accumulating, and a suddenly-different A to its
// ongoing recursion -- a real discontinuity, audible as a click or pop
// (dragging fast enough sounded like plugging or unplugging a speaker
// cable). A short exponential ramp
// (kCoeffSmoothTimeConstantSec, both direction-agnostic and content-
// agnostic -- it doesn't matter *what* changed, only that it changes
// gradually) removes the discontinuity while staying fast enough to feel
// immediate. Only A/B/C/D are smoothed -- the nonlinear stage's own
// parameters switch instantly (its output is already a smoothed
// envelope-driven scalar gain, not a recursive state, so there is no
// equivalent discontinuity risk there to begin with).
//
// An order change (adding/removing an EQ band, or toggling a band's
// enabled/solo flag -- which changes how many bands are *active* in the
// cascade, hence the assembled order, exactly as much as adding/removing
// one outright) can't be smoothed the same way: two different-sized
// state vectors have no meaningful "in between" to interpolate through.
// This clicked too, once the coefficient-smoothing above eliminated it
// for ordinary dragging. The fix here is different in
// kind: rather than trying to interpolate an undefined "in between",
// **duck the output to silence, make the (still-instant) switch while
// nothing is audible, then fade back up** (`DuckState`, `kDuckTimeSec` --
// ~8ms down, ~8ms up). This is a standard, robust technique for exactly
// this situation (switching between genuinely different filter
// structures) precisely because it doesn't depend on the two structures
// having anything in common -- unlike a parallel dual-state crossfade
// (the other standard technique), which would need the *new* filter to
// already be well-settled by the time it's audible, not guaranteed for a
// narrow/high-Q filter within a fade window short enough to feel
// immediate. A ~16ms total dip on a rare, deliberate action (add/remove/
// solo/enable a band) is a solidly better trade than either an audible
// click or a much longer, still-imperfect crossfade.

#pragma once

#include "StateSpaceTypes.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>

namespace ssfilter {

class StateSpaceEngine
{
public:
    StateSpaceEngine() = default;

    void prepare(double sampleRate) noexcept
    {
        sampleRate_ = sampleRate;
        resetState();
    }

    // Non-RT: install a new model. Safe to call from the UI/message thread
    // (takes a mutex to serialize against other non-RT callers, e.g. a
    // preset load racing a GUI drag) but never blocks the audio thread.
    void setModel(const StateSpaceMatrices& m)
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        const int live = activeSlot_.load(std::memory_order_relaxed);
        const int next = 1 - live;
        slots_[next] = m;
        activeSlot_.store(next, std::memory_order_release);
        // A separate monotonic counter, not just the slot index, so the
        // audio thread can reliably detect "at least one new model
        // arrived since last block" even in the (rare, but not
        // impossible) case of two setModel() calls landing between two
        // processBlock() calls -- comparing only the 2-valued slot index
        // would silently miss that (0 -> 1 -> 0 looks unchanged from the
        // outside), always still using the *latest* target correctly,
        // but skipping the "this needs a fresh smoothing ramp" detection
        // for the one that got skipped over.
        version_.fetch_add(1, std::memory_order_release);
    }

    // Non-RT: snapshot of whatever was last submitted via setModel().
    StateSpaceMatrices getModel() const
    {
        std::lock_guard<std::mutex> lock(writeMutex_);
        return slots_[activeSlot_.load(std::memory_order_acquire)];
    }

    // RT-safe: zero the internal state vector (e.g. on activate(), or after
    // a user-requested "Reset" so a previous, possibly large, state doesn't
    // linger audibly under the new coefficients).
    void resetState() noexcept
    {
        for (double& v : x_) v = 0.0;
        nlEnvLin_ = 1e-9;
        for (double& v : mbEnvLin_) v = 1e-9;
        for (auto& v : mbCurrentGainDb_) v.store(0.0, std::memory_order_relaxed);
        faultFlag_.store(false, std::memory_order_relaxed);
        // Snap the smoothed coefficients to whatever the currently-active
        // model is -- a hard reset (host activate(), user-requested
        // "Reset State") is an explicit "start clean" request, not a
        // moment to keep gliding from wherever a ramp happened to be.
        const int slot = activeSlot_.load(std::memory_order_acquire);
        smoothed_ = slots_[slot];
        smoothedOrder_ = clampOrder(smoothed_.order);
        smoothingActive_ = false;
        lastSeenVersion_ = version_.load(std::memory_order_acquire);
        // An explicit "start clean" request also cancels any in-progress
        // duck (see processBlock()'s DuckState) -- there's no continuity
        // left to protect once the state's already been zeroed here.
        duckState_ = DuckState::None;
        duckGain_ = 1.0;
    }

    // RT-safe: has a NaN/Inf fault been auto-recovered since the last call?
    // Clears the flag on read so the GUI can show a one-shot warning.
    bool consumeFaultFlag() noexcept
    {
        return faultFlag_.exchange(false, std::memory_order_relaxed);
    }

    // RT-safe readback of one MultibandSchedule band's own currently-
    // applied gain (dB), for a GUI meter -- the multiband compressor's
    // own "show gain reduction (and, at rest, makeup gain) graphically
    // and numerically" ask, in the same spirit as
    // DynamicEqEngine::getCurrentGainDb(). 0dB when the multiband stage
    // isn't enabled, or `index` is out of range -- a harmless default for
    // every plugin *except* the multiband compressor, which is the only
    // one that ever sets multiband.enabled at all. Same wait-free single-
    // writer(audio thread)/single-reader(UI thread) atomic contract as
    // consumeFaultFlag(), minus the exchange (this is read-many, not
    // read-once).
    double getBandGainDb(int index) const noexcept
    {
        if (index < 0 || index >= kMaxBands) return 0.0;
        return mbCurrentGainDb_[size_t(index)].load(std::memory_order_relaxed);
    }

    // RT-safe main processing entry point.
    void processBlock(const float* inL, const float* inR,
                       float* outL, float* outR, uint32_t numFrames) noexcept
    {
        const int slot = activeSlot_.load(std::memory_order_acquire);
        const StateSpaceMatrices& m = slots_[slot];
        const int targetN = clampOrder(m.order);

        // Detect a new target since the last block (see the version_
        // comment in setModel()) and decide how to react to it: same
        // order as what's currently running -> keep/start smoothing
        // A/B/C/D toward it from wherever they currently sit; different
        // order -> no meaningful "in between" exists, start a duck (see
        // this file's header comment) instead of switching instantly.
        const uint32_t ver = version_.load(std::memory_order_acquire);
        if (ver != lastSeenVersion_)
        {
            lastSeenVersion_ = ver;
            if (targetN == smoothedOrder_)
            {
                smoothingActive_ = true;
            }
            else
            {
                pendingModel_ = m;
                pendingOrder_ = targetN;
                smoothingActive_ = false;
                if (duckState_ == DuckState::None || duckState_ == DuckState::FadingIn)
                {
                    // FadingOut: already heading toward silence -- the
                    // in-progress fade-out just continues, and whichever
                    // model is newest in pendingModel_ is the one applied
                    // at the bottom of it (unchanged from before).
                    //
                    // None or FadingIn: a bug fixed here -- the swap that
                    // actually applies pendingModel_ only ever happens at
                    // the bottom of a FadingOut (see below). A second
                    // order-changing edit landing during FadingIn (volume
                    // already climbing back up from a PREVIOUS switch)
                    // used to just overwrite pendingModel_/pendingOrder_
                    // and leave duckState_ alone -- FadingIn would then
                    // run to completion (duckState_ = None) with the
                    // newer pendingModel_ never applied at all, and
                    // lastSeenVersion_ already advanced past it, so
                    // nothing would ever revisit it: the engine got
                    // permanently stuck on the OLDER, superseded model
                    // until some later, unrelated edit happened to arrive.
                    // Pivoting straight back into a fresh FadingOut here
                    // (continuing smoothly downward from wherever duckGain_
                    // currently sits, not jumping back to 1.0) makes the
                    // swap-at-the-bottom logic run again for this newest
                    // target instead.
                    duckState_ = DuckState::FadingOut;
                    duckStep_ = 1.0 / (kDuckTimeSec * sampleRate_);
                }
            }
        }

        const double smoothCoeff = smoothingActive_
                                        ? (1.0 - std::exp(-1.0 / (kCoeffSmoothTimeConstantSec * sampleRate_)))
                                        : 0.0;

        // Fast-path gate for the O(order^2) recursion below -- see
        // StateSpaceMatrices::sparseCascade2's own comment for the exact
        // guarantee this trusts (A[i][j] == 0 exactly for j beyond row i's
        // own 2-wide diagonal block) and why it's only ever true for a
        // cascade-built EQ/Multiband model, never for statespace_mimo's
        // arbitrary hand-edited one. Required from *both* the live target
        // (`m`) and the currently-running smoothed coefficients
        // (`smoothed_`) -- in ordinary operation these always agree
        // (a single plugin instance only ever feeds one "kind" of model to
        // this engine for its whole lifetime), but requiring both rather
        // than trusting `m` alone costs nothing and means a future mistake
        // that ever mismatches them fails safe into the always-correct
        // dense path instead of silently corrupting output.
        const bool sparse = m.sparseCascade2 && smoothed_.sparseCascade2;

        // Optional state-dependent gain stage (see StateSpaceTypes.hpp's
        // NonlinearSchedule): attack/release time constants only depend on
        // the sample rate and this block's ms settings, so computed once
        // per block, not per sample.
        const bool nlEnabled = m.nonlinear.enabled;
        double nlAttackCoeff = 0.0, nlReleaseCoeff = 0.0;
        if (nlEnabled)
        {
            const double attackMs  = m.nonlinear.attackMs  > 0.05 ? m.nonlinear.attackMs  : 0.05;
            const double releaseMs = m.nonlinear.releaseMs > 1.0  ? m.nonlinear.releaseMs : 1.0;
            nlAttackCoeff  = std::exp(-1.0 / (0.001 * attackMs  * sampleRate_));
            nlReleaseCoeff = std::exp(-1.0 / (0.001 * releaseMs * sampleRate_));
        }

        // Optional *multiband* state-dependent gain stage (see
        // StateSpaceTypes.hpp's MultibandSchedule): same idea as the
        // single-band stage above, but each band gets its own attack/
        // release coefficients (precomputed once per block, same reasoning)
        // and its own envelope state (mbEnvLin_[k], persisted across
        // blocks). Read from `m` (the live target model), not `smoothed_`
        // -- exactly like the single-band nonlinear stage's own parameters
        // above, which already switch instantly rather than being smoothed
        // (see this file's header comment: "nonlinear stage's own
        // parameters switch instantly, since its output is already a
        // smoothed envelope-driven scalar gain, not a recursive state").
        // Unlike that stage, a band's stateStart/stateCount *does*
        // reference state indices, which only unambiguously mean "this
        // band" for a model whose order matches what's currently running
        // -- during the brief order-changing duck window this can read a
        // stale/mismatched range, but the duck's own fade already mutes
        // output through exactly that window, so the transient has no
        // audible consequence (the backing arrays are always fully
        // allocated to kMaxOrder regardless of the active order, so this
        // is never an out-of-bounds read either -- see StateSpaceTypes.hpp).
        const bool mbEnabled = m.multiband.enabled && m.multiband.numBands > 0;
        double mbAttackCoeff[kMaxBands] = {}, mbReleaseCoeff[kMaxBands] = {};
        if (mbEnabled)
        {
            for (int k = 0; k < m.multiband.numBands; ++k)
            {
                const NonlinearSchedule& sch = m.multiband.bands[k].schedule;
                const double attackMs  = sch.attackMs  > 0.05 ? sch.attackMs  : 0.05;
                const double releaseMs = sch.releaseMs > 1.0  ? sch.releaseMs : 1.0;
                mbAttackCoeff[k]  = std::exp(-1.0 / (0.001 * attackMs  * sampleRate_));
                mbReleaseCoeff[k] = std::exp(-1.0 / (0.001 * releaseMs * sampleRate_));
            }
        }

        for (uint32_t f = 0; f < numFrames; ++f)
        {
            // Duck state machine: at most one order-changing switch
            // happens per block (right here, at the bottom of a fade-out,
            // never mid-recursion), so `n` below must be (re-)read *after*
            // this rather than cached once for the whole block.
            double duckMul = 1.0;
            if (duckState_ == DuckState::FadingOut)
            {
                duckGain_ -= duckStep_;
                if (duckGain_ <= 0.0)
                {
                    duckGain_ = 0.0;
                    // The switch itself, made while nothing is audible:
                    // a full state reset is correct here (not just
                    // "leave the extra/missing entries alone") since a
                    // different-order model can mean entirely different
                    // coefficients even at state indices that existed
                    // before (e.g. removing a middle EQ band reindexes
                    // every band after it in the series cascade, not
                    // just truncating the end).
                    smoothed_ = pendingModel_;
                    smoothedOrder_ = pendingOrder_;
                    for (double& v : x_) v = 0.0;
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

            const int n = smoothedOrder_; // the order actually driving x_/xNext_ this sample, always matching smoothed_'s contents

            if (smoothingActive_)
            {
                // One-pole-smooth every coefficient actually in play this
                // block toward the target, over just the active n x n /
                // n x 2 / 2 x n / 2 x 2 sub-block (not the full
                // kMaxOrder-sized backing store) so this costs no more
                // than the recursion below already does. Converges (and
                // stops paying this cost) once every element is within
                // 1e-9 of its target -- exponential smoothing never
                // reaches *exactly* zero difference in theory, so this
                // snaps the remainder once it's inaudibly small rather
                // than running forever.
                double maxDiff = 0.0;
                for (int i = 0; i < n; ++i)
                {
                    // Same fast-path bound as the recursion below -- when
                    // `sparse` holds, both t and s are exactly 0 for
                    // j >= jEnd (see StateSpaceMatrices::sparseCascade2's
                    // comment), so t - s is exactly 0 there too and skipping
                    // it changes maxDiff by exactly nothing.
                    const int jEnd = sparse ? std::min(n, 2 * (i / 2) + 2) : n;
                    for (int j = 0; j < jEnd; ++j)
                    {
                        double& s = smoothed_.A[i][j];
                        const double t = m.A[i][j];
                        s += smoothCoeff * (t - s);
                        maxDiff = std::max(maxDiff, std::fabs(t - s));
                    }
                }
                for (int i = 0; i < n; ++i)
                    for (int j = 0; j < kNumInputs; ++j)
                    {
                        double& s = smoothed_.B[i][j];
                        const double t = m.B[i][j];
                        s += smoothCoeff * (t - s);
                        maxDiff = std::max(maxDiff, std::fabs(t - s));
                    }
                for (int i = 0; i < kNumOutputs; ++i)
                    for (int j = 0; j < n; ++j)
                    {
                        double& s = smoothed_.C[i][j];
                        const double t = m.C[i][j];
                        s += smoothCoeff * (t - s);
                        maxDiff = std::max(maxDiff, std::fabs(t - s));
                    }
                for (int i = 0; i < kNumOutputs; ++i)
                    for (int j = 0; j < kNumInputs; ++j)
                    {
                        double& s = smoothed_.D[i][j];
                        const double t = m.D[i][j];
                        s += smoothCoeff * (t - s);
                        maxDiff = std::max(maxDiff, std::fabs(t - s));
                    }
                if (maxDiff < 1e-9) smoothingActive_ = false;
            }

            const double u0 = double(inL[f]);
            const double u1 = double(inR[f]);

            double y0, y1;
            if (mbEnabled)
            {
                // Each band computes its own pre-gain contribution from its
                // own contiguous slice of the state vector (plus its own
                // feed-through D -- see BandSplit's comment for why that
                // can't just be read off the model's combined D field),
                // tracks its own envelope of *that* signal (not the
                // broadband input -- unlike the single-band NonlinearSchedule
                // below, a per-band compressor must react to what's actually
                // in its own band), looks up its own gain curve, and the
                // gain-adjusted per-band contributions sum into the final
                // output.
                y0 = 0.0; y1 = 0.0;
                for (int k = 0; k < m.multiband.numBands; ++k)
                {
                    const BandSplit& band = m.multiband.bands[k];
                    double by0 = band.feedD[0][0] * u0 + band.feedD[0][1] * u1;
                    double by1 = band.feedD[1][0] * u0 + band.feedD[1][1] * u1;
                    const int lo = band.stateStart;
                    const int hi = std::min(lo + band.stateCount, kMaxOrder);
                    for (int j = lo; j < hi; ++j)
                    {
                        const double xj = x_[j];
                        by0 += smoothed_.C[0][j] * xj;
                        by1 += smoothed_.C[1][j] * xj;
                    }

                    double& envLin = mbEnvLin_[k];
                    const double levelLin = std::max(std::fabs(by0), std::fabs(by1));
                    const double coeff = (levelLin > envLin) ? mbAttackCoeff[k] : mbReleaseCoeff[k];
                    envLin = coeff * envLin + (1.0 - coeff) * levelLin;
                    const double envDb = 20.0 * std::log10(std::max(envLin, 1e-9));
                    const double gainDb = band.schedule.curve.evaluate(envDb);
                    const double gainLin = std::pow(10.0, gainDb / 20.0);
                    mbCurrentGainDb_[size_t(k)].store(gainDb, std::memory_order_relaxed);

                    y0 += by0 * gainLin;
                    y1 += by1 * gainLin;
                }
            }
            else
            {
                y0 = smoothed_.D[0][0] * u0 + smoothed_.D[0][1] * u1;
                y1 = smoothed_.D[1][0] * u0 + smoothed_.D[1][1] * u1;
                for (int j = 0; j < n; ++j)
                {
                    const double xj = x_[j];
                    y0 += smoothed_.C[0][j] * xj;
                    y1 += smoothed_.C[1][j] * xj;
                }
            }

            // The single most expensive line in this whole engine for a
            // heavily-loaded EQ/Multiband instance: O(order^2) every
            // sample. When `sparse` holds (see its own comment above),
            // A[i][j] is guaranteed exactly 0.0 for every j at or beyond
            // row i's own 2-wide diagonal block, so the terms this skips
            // would have added exactly 0.0 to `s` regardless -- same
            // floating-point accumulation, same order, for every term that
            // actually executes; this is bit-for-bit the same `s` either
            // way, just without multiplying by (and adding) known zeros.
            // Falls through to the full dense sweep whenever `sparse` is
            // false, i.e. always for statespace_mimo's arbitrary matrices
            // and for anything else that hasn't opted in -- see
            // StateSpaceMatrices::sparseCascade2's own comment.
            for (int i = 0; i < n; ++i)
            {
                double s = smoothed_.B[i][0] * u0 + smoothed_.B[i][1] * u1;
                const int jEnd = sparse ? std::min(n, 2 * (i / 2) + 2) : n;
                for (int j = 0; j < jEnd; ++j)
                    s += smoothed_.A[i][j] * x_[j];
                xNext_[i] = s;
            }
            for (int i = 0; i < n; ++i)
            {
                double v = xNext_[i];
                if (std::fabs(v) < 1e-15) v = 0.0; // flush near-zero residue (avoids subnormal buildup in feedback loops)
                x_[i] = v;
            }

            if (!(std::isfinite(y0) && std::isfinite(y1)))
            {
                // Numerical fault: something produced NaN/Inf despite the
                // stability guard (e.g. pathological sustained input driving
                // accumulated float error). Hard-reset and mute rather than
                // ever emit non-finite samples to the host.
                faultFlag_.store(true, std::memory_order_relaxed);
                for (double& v : x_) v = 0.0;
                y0 = 0.0;
                y1 = 0.0;
                // A hard fault reset already breaks continuity outright --
                // no point protecting a duck (or a coefficient-smoothing
                // ramp, though that has no separate state to clean up)
                // that no longer has anything meaningful to bridge.
                duckState_ = DuckState::None;
                duckGain_ = 1.0;
            }
            else
            {
                if (nlEnabled)
                {
                    // p[n]: feed-forward envelope of the raw input level.
                    // This is the "state dependence" in state-dependent-
                    // coefficients: C(p) and D(p) are computed fresh from p
                    // every sample as gain*C0 / gain*D0 (applying the gain
                    // to the already-computed y is exactly that,
                    // arithmetically) -- A and B above are completely
                    // untouched, so this can never affect the recursion's
                    // stability.
                    //
                    // The envelope itself is smoothed in the *linear*
                    // amplitude domain, not dB -- this matters more than it
                    // might look: a real (sine, or any AC) signal's
                    // instantaneous |sample| touches 0 (dB: -infinity) at
                    // every zero crossing, twice per cycle. Smoothing that
                    // directly in dB, as an earlier version of this engine
                    // did, makes the attack/release behavior depend on the
                    // signal's *frequency* (a fast attack partially chases
                    // each zero crossing at high frequencies more than low
                    // ones) -- the same threshold/ratio/attack/release
                    // settings then compress a bass note and a treble note
                    // differently, which no real compressor does and
                    // nothing here is supposed to either. Linear-domain
                    // smoothing (converting to dB only afterward, for the
                    // curve lookup) is the standard envelope-follower
                    // architecture for exactly this reason.
                    const double levelLin = std::max(std::fabs(u0), std::fabs(u1));
                    const double coeff = (levelLin > nlEnvLin_) ? nlAttackCoeff : nlReleaseCoeff;
                    nlEnvLin_ = coeff * nlEnvLin_ + (1.0 - coeff) * levelLin;

                    const double envDb = 20.0 * std::log10(std::max(nlEnvLin_, 1e-9));
                    const double gainDb = m.nonlinear.curve.evaluate(envDb);
                    const double gainLin = std::pow(10.0, gainDb / 20.0);
                    y0 *= gainLin;
                    y1 *= gainLin;
                }

                // Last-resort ceiling: this is not a mastering limiter and
                // should never engage for any sane configuration, it exists
                // only to bound truly runaway-but-still-finite output before
                // it reaches the host.
                constexpr double kCeiling = 64.0;
                if (y0 > kCeiling) y0 = kCeiling; else if (y0 < -kCeiling) y0 = -kCeiling;
                if (y1 > kCeiling) y1 = kCeiling; else if (y1 < -kCeiling) y1 = -kCeiling;
            }

            // Applied unconditionally (a no-op, duckMul == 1.0, outside a
            // duck) rather than branching -- see the duck state machine
            // above for when/why this is anything other than 1.0.
            outL[f] = float(y0 * duckMul);
            outR[f] = float(y1 * duckMul);
        }
    }

private:
    // Time constant for the A/B/C/D coefficient smoothing described in
    // this file's header comment -- short enough to feel immediate
    // (about 5ms to close ~63% of any jump, fully converged well under
    // 50ms), long enough that even a large jump lands as a fast, smooth
    // glide rather than a between-samples discontinuity.
    static constexpr double kCoeffSmoothTimeConstantSec = 0.005;

    // Duration of *each half* of the order-changing duck (fade-out, then
    // fade-in) described in this file's header comment -- ~16ms total,
    // brief enough to be a barely-there dip on a rare, deliberate action,
    // long enough that the linear ramp itself introduces no audible
    // artifact of its own.
    static constexpr double kDuckTimeSec = 0.008;

    std::array<StateSpaceMatrices, 2> slots_{};
    std::atomic<int> activeSlot_{0};
    std::atomic<uint32_t> version_{0}; // see setModel()'s comment
    mutable std::mutex writeMutex_;

    double x_[kMaxOrder] = {};
    double xNext_[kMaxOrder] = {};
    double sampleRate_ = 44100.0;
    double nlEnvLin_ = 1e-9; // smoothed input-level envelope for NonlinearSchedule, linear amplitude
    double mbEnvLin_[kMaxBands] = {}; // per-band smoothed envelope for MultibandSchedule, linear amplitude (see resetState())
    std::array<std::atomic<double>, kMaxBands> mbCurrentGainDb_{}; // per-band live gain readback, see getBandGainDb()
    std::atomic<bool> faultFlag_{false};

    // A/B/C/D actually used by the recursion, smoothly chasing whichever
    // model is currently active -- see this file's header comment.
    StateSpaceMatrices smoothed_;
    int smoothedOrder_ = 0;
    bool smoothingActive_ = false;
    uint32_t lastSeenVersion_ = 0;

    // Order-changing-edit duck -- see this file's header comment and
    // processBlock()'s duck state machine.
    enum class DuckState : uint8_t { None, FadingOut, FadingIn };
    DuckState duckState_ = DuckState::None;
    double duckGain_ = 1.0;
    double duckStep_ = 0.0; // per-sample linear increment/decrement, recomputed whenever a duck starts
    StateSpaceMatrices pendingModel_; // the order-changing target, applied at the bottom of the fade-out
    int pendingOrder_ = 0;
};

} // namespace ssfilter
