// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// StabilityGuard.hpp
//
// The one job of this file: make it impossible for the GUI/preset loader to
// hand the real-time engine an A matrix whose state recursion x[n+1]=Ax[n]+...
// diverges. A discrete-time LTI system is stable iff every eigenvalue of A
// lies strictly inside the unit circle, i.e. the spectral radius rho(A) < 1.
//
// Rather than computing an exact eigendecomposition (a full real-Schur/QR
// solver is a lot of delicate code to get bug-free for a first draft, and
// we don't actually need individual eigenvalues -- only their maximum
// magnitude), this uses:
//
//   1. A cheap, *exact* upper bound: rho(A) <= min(||A||_1, ||A||_inf) for
//      any square matrix (induced 1-norm = max abs column sum, induced
//      inf-norm = max abs row sum). If that bound already clears the
//      target radius, A is provably stable and nothing is touched. For a
//      resonant filter's controllable-canonical-form matrix, this bound is
//      typically 2-3x the true spectral radius (it does *not* mean the
//      matrix is unstable), so most nontrivial filters fall through to:
//
//   2. A near-exact estimate of rho(A), via repeated squaring of A: for any
//      submultiplicative matrix norm, rho(A) = lim_{N->inf} ||A^N||^(1/N),
//      unconditionally -- no assumption about eigenvector genericity or a
//      gap between the dominant eigenvalue and the rest, unlike a
//      single-vector power iteration (which this guard used to do, and
//      which turned out to be measurably biased for exactly the matrices
//      this plugin deals with constantly: any resonant biquad section has
//      a complex-conjugate pole *pair* of *exactly* equal magnitude, and a
//      narrow/high-Q one rotates so slowly per iteration that a few
//      hundred vector-iteration steps sample only a sliver of one
//      rotation -- see the .cpp for the concrete case this caused). 40
//      squarings reach an effective N of 2^40, cheaply, for any order this
//      plugin supports, which is far more than enough to swamp that bias.
//      This estimate, padded with a small additive margin, is used to
//      compute a single uniform scale factor s such that A' = s*A. Because
//      rho(s*A) = s*rho(A) *exactly* (scaling is linear), the only
//      remaining source of error is in the estimate itself, and success is
//      re-checked against that same (accurate) estimate each round, not
//      against the loose bound from step 1 -- checking against the loose
//      bound instead was the second bug this guard used to have: that
//      bound can be structurally ~3x the true radius for a resonant
//      filter, so it would never fall below the target no matter how much
//      the matrix was scaled, forcing every such filter through the
//      maximally-conservative fallback below regardless of how comfortably
//      stable its real pole radius already was.
//
// Uniform scaling of A preserves the relative pole layout (angles /
// resonant frequencies), it only pulls everything radially toward the
// origin, so a too-hot filter gets tamed rather than distorted in shape --
// and because the estimate above is accurate rather than a loose bound,
// "too hot" now means "actually inside 1.0 by too little margin", not
// "the loose bound doesn't happen to prove it in one step".

#pragma once

#include "StateSpaceTypes.hpp"

namespace ssfilter {

struct StabilityReport
{
    double normBound      = 0.0;   // min(||A||_1, ||A||_inf), an exact upper bound on rho(A)
    double estimatedRadius = 0.0;  // near-exact estimate of rho(A) (see estimateSpectralRadius())
    bool   provenStable    = false; // true if normBound alone already proves rho(A) < target
    bool   wasClamped      = false; // true if enforce() had to scale A down
    double appliedScale    = 1.0;   // cumulative scale factor applied to A by enforce() (1.0 = untouched)
};

class StabilityGuard
{
public:
    // Exact, cheap upper bound on the spectral radius of the active
    // order x order block of A. Always a valid bound; never an
    // underestimate. O(order^2). Often loose (see class comment above).
    static double normRadiusBound(const StateSpaceMatrices& m);

    // Near-exact estimate of the spectral radius of the active block of A,
    // via repeated squaring (see class comment above). Not RT-safe
    // (allocates temporaries, does a fixed but small number of O(order^3)
    // squarings) -- call only from the UI/message thread when coefficients
    // change, never from the audio callback.
    static double estimateSpectralRadius(const StateSpaceMatrices& m);

    // Analyze without modifying anything.
    static StabilityReport analyze(const StateSpaceMatrices& m, double maxRadius = kDefaultMaxPoleRadius);

    // In-place: if the system is not provably stable, uniformly scale the
    // active A block down until it is (per the strategy above). Returns a
    // report describing what was found/done. Also sanitizes non-finite
    // entries (NaN/Inf) in A, B, C, D by zeroing them, since those can never
    // be part of a valid configuration.
    static StabilityReport enforce(StateSpaceMatrices& m, double maxRadius = kDefaultMaxPoleRadius);

private:
    static void sanitizeFinite(StateSpaceMatrices& m);
};

} // namespace ssfilter
