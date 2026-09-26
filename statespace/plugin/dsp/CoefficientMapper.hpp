// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// CoefficientMapper.hpp
//
// This is the extension point the whole plugin is built around: a
// "coefficient mapper" is anything that turns musically-meaningful
// parameters (a cutoff frequency, a gain in dB, a Q...) into concrete
// A/B/C/D entries for the generic StateSpaceEngine. The engine and GUI
// never need to know a mapper exists -- they only ever see the resulting
// StateSpaceMatrices -- so new *linear, time-invariant* processing types
// (parametric EQ, crossovers, resonators, mid/side matrices, ...) can be
// added later as self-contained mapper functions/classes without touching
// the DSP core or the matrix-editing GUI at all.
//
// A mapper is just: (params) -> StateSpaceMatrices. There is deliberately
// no virtual base class forcing a common parameter shape, so each mapper
// below is a plain function/struct following that convention. A caller
// should always run the result through StabilityGuard::enforce() before
// handing it to StateSpaceEngine::setModel() -- mapper output is *usually*
// already stable by construction (e.g. valid biquads with |a1|,|a2| in the
// normal range are stable poles) but the guard is cheap insurance against
// extreme/edited parameter values.
//
// One worked example is included: a per-channel biquad, realized as a
// block-diagonal state-space system, plus RBJ "Audio EQ Cookbook" formulas
// for the common filter shapes. This both demonstrates the pattern and
// gives the plugin an immediately useful EQ mode to build a GUI parameter
// page on top of later.
//
// Out of scope for a mapper, on mathematical grounds, not just as a
// convention: anything whose gain depends on signal level (compression,
// limiting, saturation, gating). See the note in StateSpaceTypes.hpp and
// docs/EXTENDING.md for why a fixed-coefficient state-space system, at any
// order, cannot represent that.

#pragma once

#include "MonoStateSpaceTypes.hpp"
#include "StateSpaceTypes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

namespace ssfilter {

// Coefficients of a single second-order IIR section in the common
// "b0,b1,b2 / 1,a1,a2" normalized form:
//
//   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
struct BiquadCoeffs
{
    double b0 = 1.0, b1 = 0.0, b2 = 0.0;
    double a1 = 0.0, a2 = 0.0;
};

// One second-order section's Direct-Form-I delay line, and the per-sample
// step function to run it -- for a filter processed as its own small,
// independent recursion rather than folded into one big state-space
// matrix (e.g. DynamicEqEngine.hpp's per-band shaper/detector, or
// PluginEq's Audition tap). Deliberately the simplest correct realization,
// not the transposed form the state-space matrix path effectively uses --
// there's no shared A/B/C/D structure to fold into here, just one filter
// on its own.
struct Biquad1State
{
    double x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
};

inline double processDF1(Biquad1State& s, const BiquadCoeffs& c, double x0) noexcept
{
    const double y0 = c.b0 * x0 + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
    s.x2 = s.x1; s.x1 = x0;
    s.y2 = s.y1; s.y1 = y0;
    return y0;
}

namespace mappers {

// Controllable-canonical-form state-space realization of one biquad
// (2 states). This is the standard textbook realization:
//
//   A = [ -a1  -a2 ]   B = [ 1 ]   C = [ b1 - a1*b0,  b2 - a2*b0 ]   D = [ b0 ]
//       [  1    0  ]       [ 0 ]
inline void realizeBiquadInto(const BiquadCoeffs& c,
                               StateSpaceMatrices& out, int stateOffset, int channel)
{
    out.A[stateOffset][stateOffset]     = -c.a1;
    out.A[stateOffset][stateOffset + 1] = -c.a2;
    out.A[stateOffset + 1][stateOffset] = 1.0;
    out.A[stateOffset + 1][stateOffset + 1] = 0.0;

    out.B[stateOffset][channel]     = 1.0;
    out.B[stateOffset + 1][channel] = 0.0;

    out.C[channel][stateOffset]     = c.b1 - c.a1 * c.b0;
    out.C[channel][stateOffset + 1] = c.b2 - c.a2 * c.b0;

    out.D[channel][channel] = c.b0;
}

// Two independent biquads (no cross-channel coupling), one per stereo
// channel, packed into a single order-4 block-diagonal state-space model.
// This is the pattern a stereo parametric EQ band would use.
inline StateSpaceMatrices makeStereoBiquad(const BiquadCoeffs& left, const BiquadCoeffs& right)
{
    StateSpaceMatrices m;
    m.setSilence();
    m.order = 4;
    realizeBiquadInto(left,  m, 0, 0); // states 0,1 -> left channel
    realizeBiquadInto(right, m, 2, 1); // states 2,3 -> right channel
    std::snprintf(m.name, sizeof(m.name), "%s", "Stereo Biquad");
    return m;
}

// ---------------------------------------------------------------------
// RBJ "Audio EQ Cookbook" biquad design formulas (Robert Bristow-Johnson),
// the standard reference formulas used by countless open-source EQs.
// fs = sample rate (Hz), f0 = center/cutoff (Hz), Q = quality factor,
// gainDb = shelf/peak gain in dB (ignored by filters that don't use it).
// ---------------------------------------------------------------------

inline BiquadCoeffs designLowpass(double fs, double f0, double q)
{
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = ((1.0 - cw) / 2.0) / a0;
    c.b1 = (1.0 - cw) / a0;
    c.b2 = c.b0;
    c.a1 = (-2.0 * cw) / a0;
    c.a2 = (1.0 - alpha) / a0;
    return c;
}

inline BiquadCoeffs designHighpass(double fs, double f0, double q)
{
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = ((1.0 + cw) / 2.0) / a0;
    c.b1 = -(1.0 + cw) / a0;
    c.b2 = c.b0;
    c.a1 = (-2.0 * cw) / a0;
    c.a2 = (1.0 - alpha) / a0;
    return c;
}

inline BiquadCoeffs designPeaking(double fs, double f0, double q, double gainDb)
{
    const double A = std::pow(10.0, gainDb / 40.0);
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double a0 = 1.0 + alpha / A;
    BiquadCoeffs c;
    c.b0 = (1.0 + alpha * A) / a0;
    c.b1 = (-2.0 * cw) / a0;
    c.b2 = (1.0 - alpha * A) / a0;
    c.a1 = (-2.0 * cw) / a0;
    c.a2 = (1.0 - alpha / A) / a0;
    return c;
}

inline BiquadCoeffs designLowShelf(double fs, double f0, double q, double gainDb)
{
    const double A = std::pow(10.0, gainDb / 40.0);
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double sqrtA = std::sqrt(A);
    const double a0 = (A + 1.0) + (A - 1.0) * cw + 2.0 * sqrtA * alpha;
    BiquadCoeffs c;
    c.b0 = (A * ((A + 1.0) - (A - 1.0) * cw + 2.0 * sqrtA * alpha)) / a0;
    c.b1 = (2.0 * A * ((A - 1.0) - (A + 1.0) * cw)) / a0;
    c.b2 = (A * ((A + 1.0) - (A - 1.0) * cw - 2.0 * sqrtA * alpha)) / a0;
    c.a1 = (-2.0 * ((A - 1.0) + (A + 1.0) * cw)) / a0;
    c.a2 = ((A + 1.0) + (A - 1.0) * cw - 2.0 * sqrtA * alpha) / a0;
    return c;
}

inline BiquadCoeffs designHighShelf(double fs, double f0, double q, double gainDb)
{
    const double A = std::pow(10.0, gainDb / 40.0);
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double sqrtA = std::sqrt(A);
    const double a0 = (A + 1.0) - (A - 1.0) * cw + 2.0 * sqrtA * alpha;
    BiquadCoeffs c;
    c.b0 = (A * ((A + 1.0) + (A - 1.0) * cw + 2.0 * sqrtA * alpha)) / a0;
    c.b1 = (-2.0 * A * ((A - 1.0) + (A + 1.0) * cw)) / a0;
    c.b2 = (A * ((A + 1.0) + (A - 1.0) * cw - 2.0 * sqrtA * alpha)) / a0;
    c.a1 = (2.0 * ((A - 1.0) - (A + 1.0) * cw)) / a0;
    c.a2 = ((A + 1.0) - (A - 1.0) * cw - 2.0 * sqrtA * alpha) / a0;
    return c;
}

inline BiquadCoeffs designNotch(double fs, double f0, double q)
{
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = 1.0 / a0;
    c.b1 = (-2.0 * cw) / a0;
    c.b2 = c.b0;
    c.a1 = (-2.0 * cw) / a0;
    c.a2 = (1.0 - alpha) / a0;
    return c;
}

// Constant 0 dB peak gain bandpass (unity gain at the center frequency).
inline BiquadCoeffs designBandpass(double fs, double f0, double q)
{
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = alpha / a0;
    c.b1 = 0.0;
    c.b2 = -alpha / a0;
    c.a1 = (-2.0 * cw) / a0;
    c.a2 = (1.0 - alpha) / a0;
    return c;
}

inline BiquadCoeffs designAllpass(double fs, double f0, double q)
{
    const double w0 = 2.0 * M_PI * f0 / fs;
    const double alpha = std::sin(w0) / (2.0 * q);
    const double cw = std::cos(w0);
    const double a0 = 1.0 + alpha;
    BiquadCoeffs c;
    c.b0 = (1.0 - alpha) / a0;
    c.b1 = (-2.0 * cw) / a0;
    c.b2 = 1.0;
    c.a1 = (-2.0 * cw) / a0;
    c.a2 = (1.0 - alpha) / a0;
    return c;
}

// ---------------------------------------------------------------------
// Cascading multiple biquads in series on one channel (e.g. a band-limit
// made of a highpass + a lowpass, or a multi-band EQ chain). This is a
// second, more general worked example of the mapper pattern: instead of
// independent parallel blocks (makeStereoBiquad's block-diagonal A), a
// series cascade needs the standard state-space series-connection formula
// so stage N's output feeds stage N+1's input:
//
//   given stage1 = (A1,B1,C1,D1), stage2 = (A2,B2,C2,D2), "stage1 then
//   stage2" (single-input/single-output, since each stage is one channel):
//
//     A = [ A1        0  ]   B = [   B1  ]   C = [ D2*C1  C2 ]   D = D2*D1
//         [ B2*C1     A2 ]       [ B2*D1 ]
//
// Applied iteratively this combines any number of stages into one SISO
// state-space system, which is then packed into a stereo model exactly
// like a single biquad is (one independent chain per channel). Stability
// of the combined system follows automatically from each stage being
// individually stable (the combined poles are just the union of the
// per-stage poles), but the result is still worth running through
// StabilityGuard::enforce() like any other mapper output.
// ---------------------------------------------------------------------

namespace detail {

constexpr int kMaxChainStates = 64; // supports up to 32 cascaded biquads per channel (see kMaxOrder)

struct SisoSystem
{
    int order = 0;
    double A[kMaxChainStates][kMaxChainStates] = {};
    double B[kMaxChainStates] = {};
    double C[kMaxChainStates] = {};
    double D = 1.0;
};

inline SisoSystem biquadToSiso(const BiquadCoeffs& c)
{
    SisoSystem s;
    s.order = 2;
    s.A[0][0] = -c.a1; s.A[0][1] = -c.a2;
    s.A[1][0] = 1.0;   s.A[1][1] = 0.0;
    s.B[0] = 1.0; s.B[1] = 0.0;
    s.C[0] = c.b1 - c.a1 * c.b0;
    s.C[1] = c.b2 - c.a2 * c.b0;
    s.D = c.b0;
    return s;
}

inline SisoSystem cascadeSiso(const SisoSystem& s1, const SisoSystem& s2)
{
    // The arrays are fixed at kMaxChainStates; a combined order past that
    // would write out of bounds. cascadeChain() never asks for that (it
    // caps the stage count), this is the backstop: drop s2 rather than
    // corrupt memory.
    if (s1.order + s2.order > kMaxChainStates) return s1;
    SisoSystem out;
    out.order = s1.order + s2.order;
    for (int i = 0; i < s1.order; ++i)
        for (int j = 0; j < s1.order; ++j)
            out.A[i][j] = s1.A[i][j];
    for (int i = 0; i < s2.order; ++i)
        for (int j = 0; j < s1.order; ++j)
            out.A[s1.order + i][j] = s2.B[i] * s1.C[j];
    for (int i = 0; i < s2.order; ++i)
        for (int j = 0; j < s2.order; ++j)
            out.A[s1.order + i][s1.order + j] = s2.A[i][j];

    for (int i = 0; i < s1.order; ++i) out.B[i] = s1.B[i];
    for (int i = 0; i < s2.order; ++i) out.B[s1.order + i] = s2.B[i] * s1.D;

    for (int j = 0; j < s1.order; ++j) out.C[j] = s2.D * s1.C[j];
    for (int j = 0; j < s2.order; ++j) out.C[s1.order + j] = s2.C[j];

    out.D = s2.D * s1.D;
    return out;
}

inline SisoSystem cascadeChain(const std::vector<BiquadCoeffs>& stages)
{
    // An empty stage list is a legitimate input (e.g. a multiband
    // crossover's outermost band, which is "everything above/below the
    // last split" with no filtering of its own left to apply -- see
    // makeStereoCrossoverBank() below) -- identity (order 0, D=1) rather
    // than undefined behavior indexing stages[0].
    if (stages.empty()) { SisoSystem s; s.order = 0; s.D = 1.0; return s; }
    // At most kMaxChainStates/2 biquads fit; any further stages are
    // dropped. Before this cap, a longer list (reachable in the Parametric
    // EQ through steep Low/High Pass slopes or Band Shelves -- a band can
    // be more than one biquad, and UIEq::addBand()'s cap only counts
    // bands) made cascadeSiso() write past its fixed arrays, corrupting the
    // real-time thread's stack. Capping each channel here also keeps a
    // stereo chain's L+R total within kMaxOrder.
    const size_t n = std::min(stages.size(), size_t(kMaxChainStates / 2));
    SisoSystem combined = biquadToSiso(stages[0]);
    for (size_t i = 1; i < n; ++i)
        combined = cascadeSiso(combined, biquadToSiso(stages[i]));
    return combined;
}

} // namespace detail

// One independent biquad-chain per stereo channel (no cross-channel
// coupling), each chain applied in series on its own channel. `leftStages`
// and `rightStages` are typically the same chain (a normal stereo EQ) but
// don't have to be.
inline StateSpaceMatrices makeStereoBiquadChain(const std::vector<BiquadCoeffs>& leftStages,
                                                 const std::vector<BiquadCoeffs>& rightStages)
{
    const detail::SisoSystem L = detail::cascadeChain(leftStages);
    const detail::SisoSystem R = detail::cascadeChain(rightStages);

    StateSpaceMatrices m;
    m.setSilence();
    m.order = clampOrder(L.order + R.order);

    for (int i = 0; i < L.order; ++i)
    {
        for (int j = 0; j < L.order; ++j) m.A[i][j] = L.A[i][j];
        m.B[i][0] = L.B[i];
        m.C[0][i] = L.C[i];
    }
    m.D[0][0] = L.D;

    const int off = L.order;
    for (int i = 0; i < R.order; ++i)
    {
        for (int j = 0; j < R.order; ++j) m.A[off + i][off + j] = R.A[i][j];
        m.B[off + i][1] = R.B[i];
        m.C[1][off + i] = R.C[i];
    }
    m.D[1][1] = R.D;

    // Safe by construction, not by coincidence -- see
    // StateSpaceMatrices::sparseCascade2's own comment. L's and R's blocks
    // are each built by cascadeChain()/cascadeSiso(), which never writes
    // above a stage's own 2-wide diagonal block (proven directly in that
    // function's own comment), and L.order is always even (every biquad
    // contributes exactly 2 states), so the L/R boundary itself always
    // lands exactly on a block boundary too -- there's no row anywhere in
    // this matrix that straddles two blocks in a way this flag's contract
    // doesn't already cover.
    m.sparseCascade2 = true;

    std::snprintf(m.name, sizeof(m.name), "%s", "Stereo Biquad Chain");
    return m;
}

// A single biquad chain (one channel, no L/R duplication at all) --
// what the mono EQ plugin (plugin_eq_mono/PluginEqMono.cpp) uses instead
// of makeStereoBiquadChain(). This reuses the exact same SISO cascade
// (detail::cascadeChain) makeStereoBiquadChain() builds per channel; the
// difference is entirely in how the result is packed afterward:
// makeStereoBiquadChain places two independent copies of a same-sized
// chain side by side (order = 2 * chain order), while this packs just the
// one chain at its own natural size (order = chain order) into a
// mono::MonoStateSpaceMatrices -- see MonoStateSpaceTypes.hpp's header
// comment for why that size difference is the actual source of the mono
// plugin's DSP savings, not merely "one channel's math instead of two".
inline mono::MonoStateSpaceMatrices makeMonoBiquadChain(const std::vector<BiquadCoeffs>& stages)
{
    const detail::SisoSystem s = detail::cascadeChain(stages);

    mono::MonoStateSpaceMatrices m;
    m.setSilence();
    m.order = clampOrder(s.order);

    for (int i = 0; i < s.order; ++i)
    {
        for (int j = 0; j < s.order; ++j) m.A[i][j] = s.A[i][j];
        m.B[i] = s.B[i];
        m.C[i] = s.C[i];
    }
    m.D = s.D;

    std::snprintf(m.name, sizeof(m.name), "%s", "Mono Biquad Chain");
    return m;
}

// ---------------------------------------------------------------------
// Linkwitz-Riley crossovers, and a *parallel* connection of independent
// biquad chains (as opposed to the *series* connection cascadeChain()
// above builds) -- the standard state-space "parallel connection" formula,
// used to build an N-way frequency-splitting crossover bank for the
// multiband nonlinear stage (see StateSpaceTypes.hpp's MultibandSchedule
// and docs/EXTENDING.md's "Multiband compression" section).
//
// A Linkwitz-Riley crossover of order 2*N is a Butterworth filter of order N applied TWICE in series (LR4 = Butterworth-2
// squared = two Q=1/sqrt(2) sections; LR8 = Butterworth-4 squared = the Butterworth-4 Q pair 0.5412 / 1.3066, twice; ...).
// Each Butterworth-N contributes floor(N/2) second-order sections with
//     Q_k = 1 / (2 sin((2k - 1) * pi / (2N))),  k = 1 .. floor(N/2)
// plus, for odd N, one real pole (a first-order section) -- and two coincident real poles are exactly one biquad with
// Q = 0.5, which is what the squared odd-N filter needs. (An earlier version cascaded k identical Q=1/sqrt(2) sections for
// every order; that is a true LR only at order 4 -- at orders 2, 6 and 8 the band sum had deep holes at the crossover.)
// The defining property of an LR pair (LP and HP at the same cutoff) is that the two are power-complementary and sum to
// unity magnitude with no phase error -- unlike a plain Butterworth pair, whose sum has a bump at the crossover point --
// PROVIDED the polarity is right: for orders 4, 8, ... (N even) LP + HP is flat as it is; for orders 2, 6, ... (N odd) the
// two are 180 degrees apart at the crossover and LP - HP is the flat one. designCrossoverBandStages() below therefore
// inverts the polarity of every high-pass split for N odd (see lrNeedsHighpassInversion()). That property is what lets an
// N-way bank stay inaudible (flat) when every band's gain is 0dB: build it as a *tree* of 2-way LR splits, not N independent
// bandpass filters (which would not sum flat in general) -- designCrossoverBandStages() does exactly that.
// ---------------------------------------------------------------------

constexpr double kButterworthQ = 0.70710678118654752; // 1/sqrt(2)

// Q of each biquad of one LR lowpass/highpass of the given (even, >= 2) order, in cascade order.
inline std::vector<double> linkwitzRileyQs(int order)
{
    const int n = std::max(1, order / 2); // Butterworth order N, applied twice
    std::vector<double> once;
    for (int k = 1; k <= n / 2; ++k) once.push_back(1.0 / (2.0 * std::sin(double(2 * k - 1) * M_PI / double(2 * n))));
    std::vector<double> qs = once;
    qs.insert(qs.end(), once.begin(), once.end());   // squared
    if (n % 2 == 1) qs.push_back(0.5);               // the odd-N real pole, squared = two coincident poles = one Q=0.5 biquad
    return qs;
}

// True when LP and HP of this LR order are 180 degrees apart at the crossover, so the high side must be inverted for the
// two to sum flat (orders 2, 6, 10, ...).
inline bool lrNeedsHighpassInversion(int order) { return (std::max(1, order / 2)) % 2 == 1; }

inline std::vector<BiquadCoeffs> designLinkwitzRileyLowpass(double fs, double f0, int order = 4)
{
    std::vector<BiquadCoeffs> stages;
    for (double q : linkwitzRileyQs(order)) stages.push_back(designLowpass(fs, f0, q));
    return stages;
}

inline std::vector<BiquadCoeffs> designLinkwitzRileyHighpass(double fs, double f0, int order = 4)
{
    std::vector<BiquadCoeffs> stages;
    for (double q : linkwitzRileyQs(order)) stages.push_back(designHighpass(fs, f0, q));
    return stages;
}

// Builds the per-channel biquad-stage list for each of `crossoverHz.size()
// + 1` bands of an N-way Linkwitz-Riley crossover *tree* (see the header
// comment above): band 0 is everything below crossoverHz[0]; band i
// (0 < i < crossoverHz.size()) is between crossoverHz[i-1] and
// crossoverHz[i]; the last band is everything above crossoverHz.back().
// Each split peels a lowpass band off the "everything above the splits
// done so far" highpass remainder and cascades that remainder's own
// highpass stages in *series* in front of every band split off afterward
// -- exactly what a crossover tree means physically (the mid band, say,
// has already been highpassed once by the low/mid split before the
// mid/high split's own lowpass is applied to it).
//
// `crossoverHz` must already be sorted ascending -- callers (the CLI
// trainer, and eventually a GUI) are expected to enforce that, same as
// EqModel expects sane parameter ranges before StabilityGuard::enforce()
// catches anything that still slipped through. An empty `crossoverHz`
// yields exactly one band with no stages of its own (a transparent
// passthrough band) -- see cascadeChain()'s empty-input handling above.
inline std::vector<std::vector<BiquadCoeffs>> designCrossoverBandStages(
    double fs, const std::vector<double>& crossoverHz, int lrOrder = 4)
{
    std::vector<std::vector<BiquadCoeffs>> bands;
    if (crossoverHz.empty()) { bands.push_back({}); return bands; }

    std::vector<BiquadCoeffs> remainderHp; // "everything above the splits made so far"
    for (double f0 : crossoverHz)
    {
        std::vector<BiquadCoeffs> thisBand = remainderHp;
        const std::vector<BiquadCoeffs> low = designLinkwitzRileyLowpass(fs, f0, lrOrder);
        thisBand.insert(thisBand.end(), low.begin(), low.end());
        bands.push_back(thisBand);

        std::vector<BiquadCoeffs> high = designLinkwitzRileyHighpass(fs, f0, lrOrder);
        if (lrNeedsHighpassInversion(lrOrder) && !high.empty())
        {
            // Polarity inversion of this split's high side (see the LR comment above): negate one section's numerator.
            // Every band above the split carries this stage, so band k ends up with sign (-1)^k -- exactly the
            // "low - high" recursion of the tree.
            high[0].b0 = -high[0].b0; high[0].b1 = -high[0].b1; high[0].b2 = -high[0].b2;
        }
        remainderHp.insert(remainderHp.end(), high.begin(), high.end());
    }
    bands.push_back(remainderHp);
    return bands;
}

// Assembles a stereo N-band crossover (each band's own per-channel biquad
// chain, typically from designCrossoverBandStages() above) as a *parallel*
// state-space system -- every band's chain reads the same raw input, and
// every band's own output sums into the final y[n] -- via the standard
// state-space parallel-connection formula: block-diagonal A (each band's
// chain gets its own independent diagonal block, so bands never influence
// each other's recursion, only share the same input), C horizontally
// concatenated (each band's own C columns sit only in its own block's
// columns -- every other column is left at 0), D summed (every band's own
// D contributes directly to the same output). This is simpler than
// cascadeChain()'s *series* formula above precisely because bands don't
// feed each other's input the way cascaded EQ stages do.
//
// Also fills `scheduleOut` with each band's own state range and feed-
// through D (BandSplit, StateSpaceTypes.hpp) -- everything
// StateSpaceEngine::processBlock() needs to compute each band's own
// pre-gain contribution separately for its own envelope/gain stage, since
// the assembled matrix's own combined `D` field is the *sum* of every
// band's D and can't be un-summed later (see BandSplit's own comment).
// `scheduleOut.bands[k].schedule` (the actual gain curve/attack/release)
// is left at its default (flat, disabled) -- the caller fills that in by
// hand or via GainCurveTrainer::fitMultiband(), same as the single-band
// nonlinear stage already works via GainCurveTrainer::fit().
//
// Bands beyond kMaxBands, or states beyond kMaxOrder, are silently
// dropped rather than corrupting memory -- callers are expected to keep
// band count and per-band order within those ceilings themselves (the CLI
// trainer does; see its own usage message), same as EqModel's own band
// ceiling (see its header comment) is the caller's responsibility to
// respect, not something this low-level assembler enforces by refusing.
inline StateSpaceMatrices makeStereoCrossoverBank(const std::vector<std::vector<BiquadCoeffs>>& bandStages,
                                                   MultibandSchedule& scheduleOut)
{
    StateSpaceMatrices m;
    m.setSilence();
    scheduleOut = MultibandSchedule();

    const int numBands = std::min(int(bandStages.size()), kMaxBands);
    scheduleOut.numBands = numBands;

    int offset = 0;
    for (int k = 0; k < numBands; ++k)
    {
        BandSplit& band = scheduleOut.bands[k];
        band.stateStart = offset;
        if (offset >= kMaxOrder) { band.stateCount = 0; continue; }

        const detail::SisoSystem L = detail::cascadeChain(bandStages[size_t(k)]);
        const detail::SisoSystem R = detail::cascadeChain(bandStages[size_t(k)]); // same chain, both channels

        for (int i = 0; i < L.order && offset + i < kMaxOrder; ++i)
        {
            for (int j = 0; j < L.order && offset + j < kMaxOrder; ++j)
                m.A[offset + i][offset + j] = L.A[i][j];
            m.B[offset + i][0] = L.B[i];
            m.C[0][offset + i] = L.C[i];
        }
        m.D[0][0] += L.D;
        band.feedD[0][0] = L.D;

        const int roff = offset + L.order;
        for (int i = 0; i < R.order && roff + i < kMaxOrder; ++i)
        {
            for (int j = 0; j < R.order && roff + j < kMaxOrder; ++j)
                m.A[roff + i][roff + j] = R.A[i][j];
            m.B[roff + i][1] = R.B[i];
            m.C[1][roff + i] = R.C[i];
        }
        m.D[1][1] += R.D;
        band.feedD[1][1] = R.D;

        band.stateCount = L.order + R.order;
        offset += band.stateCount;
    }

    m.order = clampOrder(offset);

    // Safe by construction -- see makeStereoBiquadChain()'s identical
    // comment and StateSpaceMatrices::sparseCascade2's own. Each band's
    // L/R blocks are independently built by the same cascadeChain() used
    // there (so each is internally block-lower-triangular), no band's
    // block is ever coupled to another's (this is a *parallel*, not
    // series, connection -- see this function's own header comment), and
    // a kMaxOrder truncation mid-band can only leave *more* entries at
    // their already-zero setSilence() default, never fewer -- it can't
    // turn a should-be-zero entry into a nonzero one.
    m.sparseCascade2 = true;

    std::snprintf(m.name, sizeof(m.name), "%s", "Stereo Crossover Bank");
    return m;
}

} // namespace mappers
} // namespace ssfilter
