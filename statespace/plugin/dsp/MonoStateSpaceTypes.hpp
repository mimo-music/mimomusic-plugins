// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MonoStateSpaceTypes.hpp
//
// Single-channel counterpart to StateSpaceTypes.hpp's StateSpaceMatrices,
// for the mono EQ plugin (plugin_eq_mono/PluginEqMono.cpp,
// gui/eq/UIEqMono.cpp). This is a genuinely separate, 1-in/1-out matrix
// shape -- not the stereo StateSpaceMatrices with one channel's B/C/D left
// at zero -- because the point of a mono variant is to *save* DSP load,
// and StateSpaceEngine::processBlock()'s dominant per-sample cost is the
// O(order^2) A-matrix recursion: for the same band count, a mono biquad
// chain needs half the states of the stereo one (2 states/band, one SISO
// chain, instead of 4 states/band, two independent L/R chains), so its A
// matrix is half the size *per side*, making the recursion's cost a
// quarter of the stereo plugin's for an equal number of bands -- a real
// saving, not just "the same matrix run once instead of twice". Leaving a
// channel's worth of always-zero rows/columns inside the existing 2x2
// StateSpaceMatrices would still pay the full stereo-sized O(order^2) cost
// for those zero entries, so it would not actually save anything.
//
// `A` still shares kMaxOrder with the stereo core (StateSpaceTypes.hpp) --
// nothing about the spectral-radius/duck/smoothing machinery needs a
// smaller ceiling, and reusing the same constant keeps MonoStabilityGuard
// a straightforward, easy-to-audit copy of StabilityGuard's algorithm. In
// practice the mono EQ's own band count is capped well under kMaxOrder
// anyway by mappers::detail::kMaxChainStates (the SISO cascade-builder's
// own fixed buffer, see CoefficientMapper.hpp), the same 32-band ceiling
// the stereo EQ has -- see EqModel.hpp's assembleMonoStateSpace() comment.

#pragma once

#include "StateSpaceTypes.hpp"

namespace ssfilter {
namespace mono {

// A complete, self-contained single-channel state-space model. Mirrors
// StateSpaceTypes.hpp's StateSpaceMatrices field-for-field, just with B/C
// as single columns/rows and D as a scalar instead of 2x2 -- see this
// file's header comment for why this is a distinct type rather than the
// stereo one with a channel zeroed out.
struct MonoStateSpaceMatrices
{
    int order = 4;

    std::array<std::array<double, kMaxOrder>, kMaxOrder> A{};
    std::array<double, kMaxOrder> B{};
    std::array<double, kMaxOrder> C{};
    double D = 1.0;

    // Reused as-is from the shared core -- neither GainCurve nor
    // NonlinearSchedule reference a channel count anywhere, so there is
    // nothing mono-specific to write here; see StateSpaceTypes.hpp.
    NonlinearSchedule nonlinear;

    char name[64] = "Untitled";

    MonoStateSpaceMatrices() { setIdentityPassthrough(); }

    // Zero A/B/C, D = 1: a clean, always-stable, audibly-transparent
    // starting point -- same convention as the stereo type's own
    // setIdentityPassthrough().
    void setIdentityPassthrough()
    {
        for (auto& row : A) row.fill(0.0);
        B.fill(0.0);
        C.fill(0.0);
        D = 1.0;
        order = 4;
        nonlinear = NonlinearSchedule();
        std::snprintf(name, sizeof(name), "%s", "Passthrough");
    }

    // Fully silent system (all matrices/scalars zero, including D).
    void setSilence()
    {
        for (auto& row : A) row.fill(0.0);
        B.fill(0.0);
        C.fill(0.0);
        D = 0.0;
        order = 4;
        nonlinear = NonlinearSchedule();
        std::snprintf(name, sizeof(name), "%s", "Silence");
    }

    bool operator==(const MonoStateSpaceMatrices& o) const
    {
        return order == o.order && A == o.A && B == o.B && C == o.C && D == o.D &&
               nonlinear == o.nonlinear;
    }
    bool operator!=(const MonoStateSpaceMatrices& o) const { return !(*this == o); }
};

} // namespace mono
} // namespace ssfilter
