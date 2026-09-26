// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MonoStabilityGuard.hpp
//
// Single-channel counterpart to StabilityGuard.hpp, operating on
// mono::MonoStateSpaceMatrices instead of the stereo StateSpaceMatrices.
// The algorithm is identical (see StabilityGuard.hpp's class comment for
// the full reasoning: exact norm bound first, repeated-squaring spectral-
// radius estimate as the near-exact fallback, uniform scaling of A only)
// -- only sanitizeFinite()'s B/C/D loops are simpler, since B/C are single
// columns/rows and D is a scalar here rather than 2-wide/2x2. Kept as a
// separate small class (rather than templating StabilityGuard itself on a
// matrix type) so the well-tested stereo guard's code can't be disturbed
// by this addition -- see MonoStateSpaceTypes.hpp's header comment for the
// same reasoning applied to the matrix type itself.

#pragma once

#include "MonoStateSpaceTypes.hpp"
#include "StabilityGuard.hpp" // for StabilityReport, reused as-is (channel-count-agnostic)

namespace ssfilter {
namespace mono {

class MonoStabilityGuard
{
public:
    static double normRadiusBound(const MonoStateSpaceMatrices& m);
    static double estimateSpectralRadius(const MonoStateSpaceMatrices& m);
    static StabilityReport analyze(const MonoStateSpaceMatrices& m, double maxRadius = kDefaultMaxPoleRadius);
    static StabilityReport enforce(MonoStateSpaceMatrices& m, double maxRadius = kDefaultMaxPoleRadius);

private:
    static void sanitizeFinite(MonoStateSpaceMatrices& m);
};

} // namespace mono
} // namespace ssfilter
