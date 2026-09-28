// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// GainCurveTrainer.hpp
//
// Fits a NonlinearSchedule (envelope attack/release + gain curve, see
// StateSpaceTypes.hpp) to example (input, target-output) audio -- the
// coefficients are learned from an input and a desired output -- for the
// state-dependent gain stage StateSpaceEngine applies on top of a
// fixed, stability-guarded A/B/C/D.
//
// Why this is a well-posed, cheap fitting problem rather than a general
// (slow, local-minima-prone) machine-learning one: the two things being
// fit play very different roles.
//
//   - The gain curve's knot *values* enter the predicted output only
//     through piecewise-linear interpolation at a given envelope level,
//     which is a LINEAR function of the knot values for any fixed
//     envelope trace. So for a fixed (attackMs, releaseMs), finding the
//     knot values that best match the target is an ordinary linear
//     least-squares problem -- solved once, exactly (via the normal
//     equations and a Cholesky solve), no iteration, no local minima.
//   - Only (attackMs, releaseMs) enter the envelope nonlinearly. There
//     are exactly two of them, so they're found by a small grid search,
//     re-solving the (cheap) linear fit at each grid point and keeping
//     whichever has the lowest residual. This is a standard "variable
//     projection" / separable-least-squares strategy: exploit whatever
//     part of the problem *is* linear, brute-force only the part that
//     isn't, and only where the search space is small enough that
//     brute force is actually cheap (it is, here: a few dozen grid
//     points x a handful of milliseconds of work each).
//
// Ridge regularization (pulling underdetermined knots -- levels never
// seen in the training audio -- toward 0 dB/transparent) keeps the fit
// well-posed even when the training material doesn't cover the whole
// level range, which is the normal case.

#pragma once

#include "CoefficientMapper.hpp"
#include "StateSpaceTypes.hpp"

#include <cstdint>
#include <vector>

namespace ssfilter {

struct GainCurveFitResult
{
    NonlinearSchedule schedule; // .enabled is set true iff the fit succeeded
    double rmsErrorDb = 0.0;    // residual: RMS difference between predicted and target gain, in dB
    uint32_t samplesUsed = 0;   // how many samples had a usable (non-silent) base signal to learn from
    bool ok = false;
};

class GainCurveTrainer
{
public:
    // `baseModel` supplies the fixed A/B/C/D the curve will multiply (its
    // own `nonlinear` field, if any, is ignored -- fitting always starts
    // from the plain LTI response). `inputL/R` and `targetL/R` must be the
    // same length (the shorter length is used if they differ) and at the
    // same sample rate as `sampleRate`.
    static GainCurveFitResult fit(const StateSpaceMatrices& baseModel, double sampleRate,
                                   const std::vector<float>& inputL, const std::vector<float>& inputR,
                                   const std::vector<float>& targetL, const std::vector<float>& targetR);

    // Multiband counterpart: fits one independent NonlinearSchedule per
    // band of `bandStages` (typically from
    // mappers::designCrossoverBandStages(), CoefficientMapper.hpp) against
    // one (full-mix input, full-mix target) audio pair, and returns them
    // assembled into a ready-to-use MultibandSchedule.
    //
    // This adds no new fitting math at all -- it's pure orchestration
    // around the exact same fit() above, called once per band:
    //   1. For band k, filter *both* the input and target audio through
    //      that band's own standalone chain (mappers::makeStereoBiquadChain()
    //      run through a plain StateSpaceEngine -- exactly the same
    //      machinery an ordinary EQ band uses, just applied to training
    //      audio rather than live audio).
    //   2. Call fit() with an *identity* base model against that pair --
    //      since the band-filtered input already *is* "the signal this
    //      band's compressor should react to and start from", fit()'s own
    //      envelope tracking (over its `inputL/R` arguments) and its own
    //      "base" computation (an identity model reproduces its input
    //      unchanged) both do exactly the right thing with no changes to
    //      fit() itself.
    // The result: band k's envelope follows *that band's own* filtered
    // signal level, and its curve predicts *that band's own* filtered
    // target level -- precisely what a per-band compressor needs, learned
    // from an ordinary full-mix (input, desired-output) pair the same way
    // the single-band trainer learns from one.
    struct MultibandFitResult
    {
        MultibandSchedule schedule; // .enabled is set true iff every band fit successfully
        double rmsErrorDb[kMaxBands] = {};
        uint32_t samplesUsed[kMaxBands] = {};
        int numBands = 0;
        bool ok = false;
    };

    static MultibandFitResult fitMultiband(const std::vector<std::vector<BiquadCoeffs>>& bandStages,
                                            double sampleRate,
                                            const std::vector<float>& inputL, const std::vector<float>& inputR,
                                            const std::vector<float>& targetL, const std::vector<float>& targetR);
};

} // namespace ssfilter
