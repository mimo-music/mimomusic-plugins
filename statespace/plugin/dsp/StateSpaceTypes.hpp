// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// StateSpaceTypes.hpp
//
// Core data model for the generic MIMO state-space engine.
//
// Discrete-time state-space form:
//
//     x[n+1] = A x[n] + B u[n]
//     y[n]   = C x[n] + D u[n]
//
// with u[n] the stereo input vector (2x1), y[n] the stereo output vector
// (2x1), and x[n] an internal state vector of configurable order N
// (1 .. MAX_ORDER). Every coefficient of A, B, C and D is independently
// addressable, so this single structure can represent essentially any
// causal, linear, time-invariant (LTI) 2-in/2-out system that fits in
// MAX_ORDER states: EQ bands, crossover filters, resonators, cross-channel
// coupling (mid/side-style matrices), all-pass phase shaping, etc. -- any
// number of them cascaded or combined, at any order. Higher-level
// "coefficient mappers" (see CoefficientMapper.hpp) are meant to be built
// on top of this to translate musical parameters (e.g. "3 kHz bell, +4 dB,
// Q=1.2") into concrete A/B/C/D entries -- this struct itself stays
// agnostic of any of that and simply stores/plays back numbers.
//
// A fixed-coefficient LTI system -- which is exactly what A/B/C/D above
// are, at any order -- always obeys superposition (input `k*u` produces
// output exactly `k*y`), so nothing whose gain depends on signal *level*
// (compression, limiting, ...) can be built from *fixed* A/B/C/D alone, no
// matter the order. That does NOT mean it can't be represented here at
// all, though: see NonlinearSchedule below, which lets C/D become
// functions of an observed signal level, recomputed every sample -- a
// state-dependent-coefficient (SDC) / linear-parameter-varying (LPV)
// system, the standard control-theory construction for exactly this. `A`
// and `B` stay fixed and stability-guarded as always, so the recursion
// itself can never be destabilized by however C(p)/D(p) vary.
//
// All storage is fixed-size (no heap allocation) so the structure is safe
// to use, copy and swap from the real-time audio thread.

#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ssfilter {

// Maximum supported state order. Chosen generously -- an order-128 dense
// MIMO state update is still O(order^2) = 16384 multiply-adds per sample,
// trivially cheap on any modern CPU even at high sample rates -- while
// keeping the coefficient set (128*128 + 128*2*2 + 2*2 doubles ~= 131 KiB)
// small enough to copy/double-buffer without concern. Raised from an
// original 64 specifically to give the sibling EQ plugin
// (plugin/dsp/EqModel.hpp, docs/ARCHITECTURE.md's "EqModel / EQ plugin"
// section) headroom for up to 32 cascaded bands (4 states/band: two
// independent biquads, one per stereo channel) -- comfortably past
// FabFilter Pro-Q's advertised 24-band maximum -- without this shared
// core needing to change again. The two things that *do* scale worse
// than linearly with order -- StabilityGuard's repeated-squaring
// fallback and TransferFunction's dense Gaussian-elimination sweep, both
// O(order^3) -- were benchmarked at a worst-case 32-band, all-high-Q
// order-128 model (~15ms and ~47ms/curve respectively) before settling
// on this ceiling; see EqModel.hpp's comment for how the EQ plugin's GUI
// avoids paying the second cost on every frame.
constexpr int kMaxOrder = 128;

// Fixed stereo I/O. The engine is deliberately MIMO-general internally
// (all matrix code is written in terms of these constants) so widening to
// more channels later is a matter of changing these two numbers and the
// plugin's port count -- nothing in the math assumes 2.
constexpr int kNumInputs  = 2;
constexpr int kNumOutputs = 2;

// Default spectral-radius ceiling used by StabilityGuard. Strictly below 1
// so the system has a small stability margin rather than sitting exactly on
// the boundary, but deliberately close to 1: a narrow, high-Q filter (a
// tight notch, a resonant peak) routinely has a true pole radius of
// 0.999-0.99995 by design -- that's what "narrow" *means* for a filter
// running at audio sample rates -- and double-precision arithmetic makes
// that an entirely safe margin from a numerical-accumulation standpoint
// (StateSpaceEngine's own NaN/Inf safety net exists as a second,
// independent backstop regardless). A tighter ceiling here doesn't buy
// meaningful extra safety; it only needlessly flattens legitimate narrow
// filters (see StabilityGuard.cpp's class comment for a concrete case).
constexpr double kDefaultMaxPoleRadius = 0.99995;

// A piecewise-linear input-level(dB) -> gain(dB) curve on a fixed,
// evenly-spaced grid ("the gain computer" of a compressor/expander/
// limiter, in the classic sense, generalized to any shape). Flat
// extrapolation beyond the ends. All-zero (the default) means 0 dB gain
// at every level, i.e. completely transparent -- enabling
// NonlinearSchedule with a fresh curve changes nothing until the curve is
// actually shaped (by hand or via the trainer, see GainCurveTrainer.hpp).
struct GainCurve
{
    static constexpr double kMinDb = -72.0;
    static constexpr double kMaxDb = 24.0;
    static constexpr int kNumKnots = 25; // 4 dB spacing
    static constexpr double kStepDb = (kMaxDb - kMinDb) / double(kNumKnots - 1);

    double gainDb[kNumKnots] = {};

    static double knotLevelDb(int i) { return kMinDb + double(i) * kStepDb; }

    double evaluate(double levelDb) const noexcept
    {
        const double t = (levelDb - kMinDb) / kStepDb;
        if (t <= 0.0) return gainDb[0];
        if (t >= double(kNumKnots - 1)) return gainDb[kNumKnots - 1];
        const int i0 = int(t);
        const double frac = t - double(i0);
        return gainDb[i0] + frac * (gainDb[i0 + 1] - gainDb[i0]);
    }

    bool operator==(const GainCurve& o) const
    {
        for (int i = 0; i < kNumKnots; ++i) if (gainDb[i] != o.gainDb[i]) return false;
        return true;
    }
    bool operator!=(const GainCurve& o) const { return !(*this == o); }

    // The classic feed-forward compressor gain-computer shape: flat (0dB,
    // plus makeup) below threshold, sloping down above it by 1/ratioDown
    // instead of 1/1. This is the exact formula
    // UIStateSpace::applyCompressorCurve() bakes into a hand-edited curve's
    // 25 knots on "Apply Curve", and the one dsp_regression's test [8]
    // checks the engine's settled output against -- pulled out here as a
    // shared, reusable helper so a multiband compressor's per-band curves
    // (see plugin/dsp/MbCompressorModel.hpp) don't need their own copy of
    // it, rather than duplicating the same seven lines a third time.
    //
    // `ratioUp` generalizes this to downward and upward compression in one call:
    // it shapes the mirrored line *below* the threshold the same way
    // ratioDown shapes the one above it -- makeupDb + (levelDb-thresholdDb)
    // * (1/ratioUp - 1), which is a *positive* (boosting) contribution for
    // levelDb < thresholdDb since both factors are negative there. Passing
    // 1.0 (the default every existing caller keeps using) makes that term
    // exactly 0, i.e. flat at makeupDb below threshold -- the original,
    // downward-only shape, unchanged bit for bit.
    static GainCurve classicCompressor(double thresholdDb, double ratioDown, double ratioUp, double makeupDb)
    {
        GainCurve c;
        for (int i = 0; i < kNumKnots; ++i)
        {
            const double levelDb = knotLevelDb(i);
            const double ratio = levelDb <= thresholdDb ? ratioUp : ratioDown;
            c.gainDb[i] = makeupDb + (levelDb - thresholdDb) * (1.0 / ratio - 1.0);
        }
        return c;
    }
};

// Optional state-dependent-coefficient (SDC/LPV) gain stage: when enabled,
// the engine tracks an attack/release-smoothed envelope of the input
// level and looks up a gain (dB) from `curve` at that level, every sample
// -- i.e. C(p)=gain*C0, D(p)=gain*D0, with A/B (and hence the recursion's
// stability) completely unaffected. See StateSpaceEngine::processBlock()
// and docs/EXTENDING.md for the full picture, and GainCurveTrainer.hpp for
// fitting `curve` (and attackMs/releaseMs) to example input/target-output
// audio instead of hand-setting it.
struct NonlinearSchedule
{
    bool enabled = false;
    double attackMs = 10.0;
    double releaseMs = 100.0;
    GainCurve curve;

    bool operator==(const NonlinearSchedule& o) const
    {
        return enabled == o.enabled && attackMs == o.attackMs && releaseMs == o.releaseMs && curve == o.curve;
    }
    bool operator!=(const NonlinearSchedule& o) const { return !(*this == o); }
};

// Maximum number of independent bands a MultibandSchedule (below) can
// carry. Chosen generously above the 3-4 bands a commercial multiband
// compressor typically offers, while staying small enough that the whole
// schedule (a handful of BandSplit entries, each barely bigger than one
// NonlinearSchedule) is trivially cheap to copy/double-buffer like
// everything else in this struct.
constexpr int kMaxBands = 6;

// One frequency band's own slice of a MultibandSchedule (below): which
// contiguous states in the model's A/B/C belong to this band (a crossover
// mapper -- see mappers::makeStereoCrossoverBank() in
// CoefficientMapper.hpp -- lays bands out as adjacent parallel blocks, so a
// single contiguous [stateStart, stateStart+stateCount) range always
// identifies one band completely), this band's own feed-through `D`
// contribution, and this band's own envelope/gain-curve schedule (reusing
// NonlinearSchedule -- a band's compressor is architecturally identical to
// the single-band one, just scoped to one band's own signal instead of the
// whole output).
//
// `feedD` has to be stored per band separately from the model's own
// combined `D` field: for a parallel connection of N independent systems
// (which is what a crossover bank is -- see the mapper's own comment),
// the *combined* system's D is the *sum* of every band's own D, so the
// individual per-band values can't be recovered from it afterward. This is
// exactly the bookkeeping StateSpaceEngine::processBlock() needs to
// compute each band's own pre-gain contribution (`feedD*u + C-slice*x`)
// separately, before applying that band's own gain and summing.
struct BandSplit
{
    int stateStart = 0;
    int stateCount = 0;
    double feedD[kNumOutputs][kNumInputs] = {};
    NonlinearSchedule schedule;

    bool operator==(const BandSplit& o) const
    {
        if (stateStart != o.stateStart || stateCount != o.stateCount || schedule != o.schedule) return false;
        for (int i = 0; i < kNumOutputs; ++i)
            for (int j = 0; j < kNumInputs; ++j)
                if (feedD[i][j] != o.feedD[i][j]) return false;
        return true;
    }
    bool operator!=(const BandSplit& o) const { return !(*this == o); }
};

// A true *multiband* state-dependent gain stage: up to kMaxBands
// independent bands (see BandSplit above), each tracking its own envelope
// of its own (post-crossover-split) filtered signal and applying its own
// gain curve -- as opposed to NonlinearSchedule above, which is a single
// global envelope/curve applied to the whole, already-summed output. Both
// can be enabled on the same model at once (multiband sums first, the
// single global stage -- if also enabled -- then applies on top of that
// sum, e.g. as a broadband safety limiter over an otherwise per-band
// compressor); they're independent fields for exactly that reason. See
// docs/EXTENDING.md's "Multiband compression" section and
// StateSpaceEngine.hpp's processBlock() for the per-sample mechanics, and
// mappers::makeStereoCrossoverBank() (CoefficientMapper.hpp) for how a
// crossover bank populates the state ranges/feedD this depends on. `A` and
// `B` -- and therefore the recursion's stability -- are, as always,
// completely unaffected; only ever C(p)/D(p) per band.
struct MultibandSchedule
{
    bool enabled = false;
    int numBands = 0;
    BandSplit bands[kMaxBands];

    bool operator==(const MultibandSchedule& o) const
    {
        if (enabled != o.enabled || numBands != o.numBands) return false;
        for (int i = 0; i < numBands; ++i)
            if (bands[i] != o.bands[i]) return false;
        return true;
    }
    bool operator!=(const MultibandSchedule& o) const { return !(*this == o); }
};

// A complete, self-contained state-space model. `order` is the *active*
// sub-block size; entries at index >= order are ignored by the engine but
// are preserved in memory (so shrinking and re-growing the order does not
// silently discard coefficients the user already entered).
struct StateSpaceMatrices
{
    int order = 8;

    // Row-major dense storage, always MAX_ORDER-sized regardless of the
    // active order so pointers/strides never change and no reallocation is
    // ever needed when the order changes.
    std::array<std::array<double, kMaxOrder>, kMaxOrder> A{};
    std::array<std::array<double, kNumInputs>, kMaxOrder> B{};
    std::array<std::array<double, kMaxOrder>, kNumOutputs> C{};
    std::array<std::array<double, kNumInputs>, kNumOutputs> D{};

    // Optional state-dependent gain stage, see NonlinearSchedule above.
    // Disabled by default -- a plain LTI filter preset never has to think
    // about this.
    NonlinearSchedule nonlinear;

    // Optional *multiband* state-dependent gain stage, see
    // MultibandSchedule above. Disabled by default, same reasoning.
    MultibandSchedule multiband;

    // Human-readable label, purely informational (shown in the GUI/preset
    // browser), not used by the DSP.
    char name[64] = "Untitled";

    // Performance hint, OPT-IN ONLY, and false unless a caller that has
    // *proven* it's safe explicitly sets it -- see StateSpaceEngine.hpp's
    // own comment on how it's used, but in short: `true` is a promise that
    // A[i][j] == 0.0 exactly for every j >= 2*(i/2 + 1) (i.e. beyond the
    // 2-wide diagonal block row i belongs to), letting the recursion skip
    // guaranteed-zero multiply-adds instead of doing the full O(order^2)
    // dense sweep. This is true, by construction (not by coincidence, not
    // "usually"), for a series cascade or parallel bank of biquads --
    // mappers::makeStereoBiquadChain()/makeStereoCrossoverBank() (the only
    // two places this is ever set true) build A one biquad at a time and
    // never write to the block above a stage's own diagonal, so the zero
    // pattern is structural, not numerical. It is **not** true for an
    // arbitrary hand-edited matrix -- statespace_mimo's MatrixGridWidget
    // lets a user set any A[i][j] to anything, so `setIdentityPassthrough()`/
    // `setSilence()` below always reset this to `false`, and nothing in
    // that plugin's own code path ever sets it back to `true`. A future
    // plugin/feature that wants the fast path just needs to build its
    // matrix the same disciplined way and set this explicitly once, right
    // before handing the result to the engine; anything that doesn't set
    // it (including every existing caller except the two mappers above)
    // gets today's exact, unconditionally-correct dense behavior with zero
    // code changes required on its part.
    bool sparseCascade2 = false;

    StateSpaceMatrices() { setIdentityPassthrough(); }

    // Zero A/B/C, D = identity: a clean, always-stable, audibly-transparent
    // starting point. This is the safe default used whenever a new/invalid
    // model would otherwise be loaded.
    void setIdentityPassthrough()
    {
        for (auto& row : A) row.fill(0.0);
        for (auto& row : B) row.fill(0.0);
        for (auto& row : C) row.fill(0.0);
        for (auto& row : D) row.fill(0.0);
        D[0][0] = 1.0;
        D[1][1] = 1.0;
        order = 8;
        nonlinear = NonlinearSchedule();
        multiband = MultibandSchedule();
        sparseCascade2 = false; // conservative default -- see the field's own comment; a cascade-building caller re-enables it explicitly afterward
        std::snprintf(name, sizeof(name), "%s", "Passthrough");
    }

    // Fully silent system (all matrices zero, including D).
    void setSilence()
    {
        for (auto& row : A) row.fill(0.0);
        for (auto& row : B) row.fill(0.0);
        for (auto& row : C) row.fill(0.0);
        for (auto& row : D) row.fill(0.0);
        order = 8;
        nonlinear = NonlinearSchedule();
        multiband = MultibandSchedule();
        sparseCascade2 = false; // see setIdentityPassthrough()'s identical comment
        std::snprintf(name, sizeof(name), "%s", "Silence");
    }

    bool operator==(const StateSpaceMatrices& o) const
    {
        return order == o.order && A == o.A && B == o.B && C == o.C && D == o.D &&
               nonlinear == o.nonlinear && multiband == o.multiband && sparseCascade2 == o.sparseCascade2;
    }
    bool operator!=(const StateSpaceMatrices& o) const { return !(*this == o); }
};

// Clamp a requested order into the legal [1, kMaxOrder] range.
inline int clampOrder(int requested)
{
    if (requested < 1) return 1;
    if (requested > kMaxOrder) return kMaxOrder;
    return requested;
}

} // namespace ssfilter
