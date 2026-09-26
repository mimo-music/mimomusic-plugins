// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "MonoStabilityGuard.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace ssfilter {
namespace mono {

namespace {
constexpr double kAbsCoeffClamp = 1.0e6; // same rationale as StabilityGuard.cpp's own copy
double clampFinite(double v)
{
    if (!std::isfinite(v)) return 0.0;
    if (v > kAbsCoeffClamp) return kAbsCoeffClamp;
    if (v < -kAbsCoeffClamp) return -kAbsCoeffClamp;
    return v;
}
} // namespace

double MonoStabilityGuard::normRadiusBound(const MonoStateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    double norm1 = 0.0, normInf = 0.0;
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

// See StabilityGuard.cpp's own copy of this algorithm for the full
// derivation/rationale -- identical here, just against a MonoStateSpaceMatrices.
double MonoStabilityGuard::estimateSpectralRadius(const MonoStateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    if (n <= 1) return std::fabs(m.A[0][0]);

    std::vector<std::vector<double>> M(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            M[i][j] = m.A[i][j];

    constexpr int kSquarings = 40;
    double logNorm = 0.0;

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
            return 0.0;

        logNorm = 2.0 * logNorm + std::log(norm);
        const double inv = 1.0 / norm;
        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                M[i][j] = sq[i][j] * inv;
    }

    const double effectiveN = std::pow(2.0, double(kSquarings));
    return std::exp(logNorm / effectiveN);
}

void MonoStabilityGuard::sanitizeFinite(MonoStateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            m.A[i][j] = clampFinite(m.A[i][j]);
    for (int i = 0; i < n; ++i)
        m.B[i] = clampFinite(m.B[i]);
    for (int i = 0; i < n; ++i)
        m.C[i] = clampFinite(m.C[i]);
    m.D = clampFinite(m.D);
}

StabilityReport MonoStabilityGuard::analyze(const MonoStateSpaceMatrices& m, double maxRadius)
{
    StabilityReport r;
    r.normBound = normRadiusBound(m);
    if (r.normBound <= maxRadius)
    {
        r.provenStable = true;
        r.estimatedRadius = r.normBound;
        return r;
    }
    r.estimatedRadius = estimateSpectralRadius(m);
    r.provenStable = r.estimatedRadius <= maxRadius;
    return r;
}

StabilityReport MonoStabilityGuard::enforce(MonoStateSpaceMatrices& m, double maxRadius)
{
    sanitizeFinite(m);
    m.order = clampOrder(m.order);

    StabilityReport r;
    r.normBound = normRadiusBound(m);
    if (r.normBound <= maxRadius)
    {
        r.provenStable = true;
        r.estimatedRadius = r.normBound;
        return r;
    }

    const int n = m.order;
    constexpr double kEstimationMargin = 0.0001;
    constexpr int kMaxRefineRounds = 8;

    double cumulativeScale = 1.0;
    bool converged = false;

    for (int round = 0; round < kMaxRefineRounds; ++round)
    {
        const double estRaw = estimateSpectralRadius(m);
        if (estRaw <= 1e-12) break;
        if (estRaw + kEstimationMargin <= maxRadius) { converged = true; break; }

        double scale = (maxRadius - kEstimationMargin) / estRaw;
        if (scale > 1.0) scale = 1.0;
        if (scale < 1e-9) scale = 1e-9;

        for (int i = 0; i < n; ++i)
            for (int j = 0; j < n; ++j)
                m.A[i][j] *= scale;
        cumulativeScale *= scale;
    }

    if (!converged)
    {
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

} // namespace mono
} // namespace ssfilter
