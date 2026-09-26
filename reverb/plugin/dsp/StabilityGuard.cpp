// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "StabilityGuard.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ssfilter {

namespace {

// Hard clamp for individual coefficients. Nothing musically meaningful ever
// needs a raw state-space coefficient this large; this exists purely to
// keep pathological hand-entered/loaded values from producing overflow in
// downstream math (norms, the spectral-radius estimate) before the
// stability pass even gets a chance to run.
constexpr double kAbsCoeffClamp = 1.0e6;

double clampFinite(double v)
{
    if (!std::isfinite(v)) return 0.0;
    if (v > kAbsCoeffClamp) return kAbsCoeffClamp;
    if (v < -kAbsCoeffClamp) return -kAbsCoeffClamp;
    return v;
}

} // namespace

double StabilityGuard::normRadiusBound(const StateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    double norm1 = 0.0;   // max abs column sum
    double normInf = 0.0; // max abs row sum

    for (int i = 0; i < n; ++i)
    {
        double rowSum = 0.0;
        for (int j = 0; j < n; ++j) rowSum += std::fabs(m.A[i][j]);
        normInf = std::max(normInf, rowSum);
    }
    for (int j = 0; j < n; ++j)
    {
        double colSum = 0.0;
        for (int i = 0; i < n; ++i) colSum += std::fabs(m.A[i][j]);
        norm1 = std::max(norm1, colSum);
    }
    return std::min(norm1, normInf);
}

// Spectral radius via repeated squaring: rho(A) = lim_{N->inf} ||A^N||^(1/N)
// for *any* submultiplicative matrix norm (here, the induced infinity-norm:
// max absolute row sum) -- this holds unconditionally, with no genericity
// assumption on a starting vector, unlike a single-vector power iteration.
// That distinction matters a lot for exactly the matrices this plugin deals
// with constantly: every 2nd-order biquad section with any real resonance
// (peaking/notch/bandpass at any reasonable Q) has a complex-conjugate pole
// pair -- i.e. *two* eigenvalues of exactly equal magnitude. Single-vector
// power iteration's usual convergence argument assumes a uniquely dominant
// eigenvalue with a gap to the rest; here there is no gap at all (both
// dominant eigenvalues tie), and for a narrow high-Q filter the pole angle
// is tiny, so the iterate's direction rotates extremely slowly -- a few
// hundred per-vector iterations sample only a small arc of that rotation
// and can be measurably (here: ~0.3%) *biased*, which the old
// implementation compounded into a wildly over-aggressive clamp for a
// perfectly stable filter (see the git history / conversation for the
// concrete case: a 50 Hz, Q=25 notch's coefficients were scaled down to
// ~30% of their design value despite the filter's true pole radius being a
// safely-stable 0.99987).
//
// Repeated squaring sidesteps this cleanly: it tracks the *matrix* norm's
// growth (which accounts for every direction at once, so no starting
// vector or rotation-phase sensitivity exists at all) and reaches an
// effective N of 2^40 in only 40 matrix multiplications -- utterly
// swamping any residual bias from the oscillatory case above, at a cost
// (O(order^3) per squaring) that's still trivial even at the maximum
// order this plugin supports.
double StabilityGuard::estimateSpectralRadius(const StateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    if (n <= 1)
        return std::fabs(m.A[0][0]);

    std::vector<std::vector<double>> M(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            M[i][j] = m.A[i][j];

    constexpr int kSquarings = 40; // effective N = 2^40
    double logNorm = 0.0;          // running log(||A^(2^s)||), renormalized each step

    std::vector<std::vector<double>> sq(n, std::vector<double>(n));
    for (int s = 0; s < kSquarings; ++s)
    {
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                sq[i][j] = 0.0;

        for (int i = 0; i < n; ++i)
        {
            for (int k = 0; k < n; ++k)
            {
                const double mik = M[i][k];
                if (mik == 0.0) continue;
                for (int j = 0; j < n; ++j)
                    sq[i][j] += mik * M[k][j];
            }
        }

        double norm = 0.0;
        for (int i = 0; i < n; ++i)
        {
            double rowSum = 0.0;
            for (int j = 0; j < n; ++j) rowSum += std::fabs(sq[i][j]);
            norm = std::max(norm, rowSum);
        }

        if (!(norm > 0.0) || !std::isfinite(norm))
            return 0.0; // matrix has (numerically) collapsed to zero

        logNorm = 2.0 * logNorm + std::log(norm);
        const double inv = 1.0 / norm;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                M[i][j] = sq[i][j] * inv;
    }

    const double effectiveN = std::pow(2.0, double(kSquarings));
    return std::exp(logNorm / effectiveN);
}

void StabilityGuard::sanitizeFinite(StateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            m.A[i][j] = clampFinite(m.A[i][j]);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < kNumInputs; ++j)
            m.B[i][j] = clampFinite(m.B[i][j]);
    for (int i = 0; i < kNumOutputs; ++i)
        for (int j = 0; j < n; ++j)
            m.C[i][j] = clampFinite(m.C[i][j]);
    for (int i = 0; i < kNumOutputs; ++i)
        for (int j = 0; j < kNumInputs; ++j)
            m.D[i][j] = clampFinite(m.D[i][j]);
}

StabilityReport StabilityGuard::analyze(const StateSpaceMatrices& m, double maxRadius)
{
    StabilityReport r;
    r.normBound = normRadiusBound(m);
    if (r.normBound <= maxRadius)
    {
        r.provenStable = true;
        r.estimatedRadius = r.normBound; // exact bound is already tight enough
        return r;
    }
    r.estimatedRadius = estimateSpectralRadius(m);
    r.provenStable = r.estimatedRadius <= maxRadius;
    return r;
}

StabilityReport StabilityGuard::enforce(StateSpaceMatrices& m, double maxRadius)
{
    sanitizeFinite(m);
    m.order = clampOrder(m.order);

    StabilityReport r;
    r.normBound = normRadiusBound(m);
    if (r.normBound <= maxRadius)
    {
        r.provenStable = true;
        r.estimatedRadius = r.normBound;
        return r; // already provably stable, nothing touched
    }

    // Below this point the cheap exact bound (normRadiusBound) was not
    // conclusive -- true of *every* biquad-style controllable-canonical-
    // form matrix with a resonance, since that bound is structurally much
    // larger than the actual spectral radius for that shape (see the class
    // comment on estimateSpectralRadius()). That does not mean the matrix
    // is unstable: the loop below decides that from the accurate estimate,
    // not from the loose bound -- using the bound here again (as an
    // earlier version of this guard did) would force every such filter
    // through the harsh fallback at the end, regardless of its actual,
    // perfectly safe pole radius.
    const int n = m.order;

    // The estimator above is essentially exact (residual error many orders
    // of magnitude below anything relevant here), so only a small additive
    // pad is needed -- and it must be additive, not multiplicative: a
    // narrow, high-Q filter's true pole radius is routinely 0.999+,
    // *deliberately* close to (but strictly below) the actual instability
    // boundary of 1.0, so even a tiny multiplicative margin (e.g. 1.001x)
    // would push a perfectly stable 0.9998 right back over any threshold
    // below 1.0 and defeat the point of having an accurate estimator.
    constexpr double kEstimationMargin = 0.0001;
    constexpr int kMaxRefineRounds = 8;

    double cumulativeScale = 1.0;
    bool converged = false;

    for (int round = 0; round < kMaxRefineRounds; ++round)
    {
        const double estRaw = estimateSpectralRadius(m);
        if (estRaw <= 1e-12)
            break; // A is effectively zero; nothing left to scale

        if (estRaw + kEstimationMargin <= maxRadius) { converged = true; break; }

        double scale = (maxRadius - kEstimationMargin) / estRaw;
        if (scale > 1.0) scale = 1.0;   // never amplify
        if (scale < 1e-9) scale = 1e-9; // avoid degenerating to a literal zero matrix in one step

        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                m.A[i][j] *= scale;
        cumulativeScale *= scale;
    }

    if (!converged)
    {
        // Ironclad last resort for a genuinely pathological matrix that
        // didn't converge above (should be exceedingly rare -- e.g. a
        // severely defective matrix): fall back to the exact norm bound.
        // rho(A) <= normRadiusBound(A) always holds, so this step is
        // guaranteed (not just likely) to bring the spectral radius under
        // maxRadius, at the cost of being far more conservative than
        // necessary -- which is why it is a last resort, not the primary
        // strategy.
        const double bound = normRadiusBound(m);
        if (bound > maxRadius)
        {
            const double scale = (maxRadius * 0.9) / bound;
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    m.A[i][j] *= scale;
            cumulativeScale *= scale;
        }
    }

    r.wasClamped      = cumulativeScale < 1.0;
    r.appliedScale    = cumulativeScale;
    r.normBound       = normRadiusBound(m);
    r.estimatedRadius = estimateSpectralRadius(m);
    r.provenStable    = r.normBound <= maxRadius || r.estimatedRadius <= maxRadius;
    return r;
}

} // namespace ssfilter
