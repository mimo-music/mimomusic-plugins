// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// TransferFunction.hpp
//
// Frequency-domain analysis of the state-space model, independent of the
// stability guard. Used to draw the Bode (magnitude/phase) plots in the
// GUI. Given the model is discrete-time, the transfer function is
//
//     H(z) = D + C (zI - A)^-1 B
//
// which is a kNumOutputs x kNumInputs complex matrix for any z. Evaluating
// it on the unit circle (z = e^{j*omega}, omega = 2*pi*f/fs) gives the
// steady-state frequency response for every input/output channel pair,
// including cross-channel terms (e.g. left-in -> right-out), which is
// exactly the information a MIMO 2x2 system needs to be visualized fully.
//
// This is pure math with no real-time constraints (called from the GUI/
// message thread only, never the audio callback): it allocates, and is
// O(order^3) per evaluated frequency point (dense complex Gaussian
// elimination), which is intentionally simple and robust rather than
// optimized -- for a first draft, correctness and clarity win over speed.

#pragma once

#include "StateSpaceTypes.hpp"

#include <complex>
#include <vector>

namespace ssfilter {

using Complex = std::complex<double>;

struct ComplexResponse
{
    Complex H[kNumOutputs][kNumInputs];
};

class TransferFunction
{
public:
    // H(z) = D + C (zI - A)^-1 B, evaluated for the active order block.
    static ComplexResponse evaluate(const StateSpaceMatrices& m, Complex z);

    struct SweepPoint
    {
        double freqHz;
        double magnitudeDb;
        double phaseRad; // unwrapped, radians
    };

    // Log-spaced frequency sweep of H_{outIdx,inIdx}(e^{j*omega}) from fMin
    // to fMax Hz (0 < fMin < fMax <= sampleRate/2). Phase is unwrapped so
    // the plotted curve doesn't show artificial +/-pi jump discontinuities.
    static void sweep(const StateSpaceMatrices& m, double sampleRate,
                       int outIdx, int inIdx,
                       double fMin, double fMax, int numPoints,
                       std::vector<SweepPoint>& out,
                       bool logSpacing = true);

    // Scales C and D (never A/B, so stability is untouched) so that the
    // *peak* magnitude of the L-in->L-out response across the audible
    // range becomes exactly 0 dB. This is a generic stand-in for "the
    // passband": for a lowpass/highpass/peaking/shelf filter (or any
    // cascade of them) the passband is, by construction, wherever the
    // response is loudest, so normalizing the peak to unity gain makes
    // "where it's passing" sit at 0 dB regardless of the filter shape.
    // Applied uniformly to both channels so stereo balance/cross-terms are
    // preserved. Returns the linear scale factor that was applied (1.0 if
    // the model was already silent and nothing could be measured).
    static double normalizeToUnityPassbandGain(StateSpaceMatrices& m, double sampleRate);
};

} // namespace ssfilter
