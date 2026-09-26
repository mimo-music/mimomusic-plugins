// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "TransferFunction.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ssfilter {

ComplexResponse TransferFunction::evaluate(const StateSpaceMatrices& m, Complex z)
{
    ComplexResponse resp;
    const int n = clampOrder(m.order);

    // Augmented matrix [ (zI - A) | B ], solved once for all kNumInputs
    // right-hand sides simultaneously.
    std::vector<std::vector<Complex>> M(n, std::vector<Complex>(n + kNumInputs));
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
            M[i][j] = (i == j ? z : Complex(0.0, 0.0)) - m.A[i][j];
        for (int k = 0; k < kNumInputs; ++k)
            M[i][n + k] = m.B[i][k];
    }

    // Gaussian elimination with partial pivoting (by magnitude).
    for (int col = 0; col < n; ++col)
    {
        int pivot = col;
        double best = std::abs(M[col][col]);
        for (int i = col + 1; i < n; ++i)
        {
            const double mag = std::abs(M[i][col]);
            if (mag > best) { best = mag; pivot = i; }
        }
        if (pivot != col) std::swap(M[col], M[pivot]);

        Complex diag = M[col][col];
        if (std::abs(diag) < 1e-300)
            diag = Complex(1e-300, 0.0); // degenerate (zI - A) singular at this z; avoid a hard divide-by-zero

        for (int i = col + 1; i < n; ++i)
        {
            const Complex factor = M[i][col] / diag;
            if (factor == Complex(0.0, 0.0)) continue;
            for (int j = col; j < n + kNumInputs; ++j)
                M[i][j] -= factor * M[col][j];
        }
    }

    // Back substitution -> X (n x kNumInputs).
    std::vector<std::array<Complex, kNumInputs>> X(n);
    for (int row = n - 1; row >= 0; --row)
    {
        for (int k = 0; k < kNumInputs; ++k)
        {
            Complex sum = M[row][n + k];
            for (int j = row + 1; j < n; ++j)
                sum -= M[row][j] * X[j][k];
            Complex diag = M[row][row];
            if (std::abs(diag) < 1e-300) diag = Complex(1e-300, 0.0);
            X[row][k] = sum / diag;
        }
    }

    for (int o = 0; o < kNumOutputs; ++o)
    {
        for (int i = 0; i < kNumInputs; ++i)
        {
            Complex sum(m.D[o][i], 0.0);
            for (int j = 0; j < n; ++j)
                sum += m.C[o][j] * X[j][i];
            resp.H[o][i] = sum;
        }
    }
    return resp;
}

void TransferFunction::sweep(const StateSpaceMatrices& m, double sampleRate,
                              int outIdx, int inIdx,
                              double fMin, double fMax, int numPoints,
                              std::vector<SweepPoint>& out,
                              bool logSpacing)
{
    out.clear();
    if (numPoints < 2 || sampleRate <= 0.0) return;
    fMin = std::max(fMin, 1.0);
    fMax = std::min(fMax, sampleRate * 0.5 - 1.0);
    if (fMax <= fMin) fMax = fMin + 1.0;

    out.reserve(numPoints);
    const double logMin = std::log10(fMin);
    const double logMax = std::log10(fMax);

    double prevPhase = 0.0;
    double unwrapOffset = 0.0;
    bool first = true;

    for (int i = 0; i < numPoints; ++i)
    {
        const double t = (numPoints == 1) ? 0.0 : double(i) / double(numPoints - 1);
        const double freq = logSpacing ? std::pow(10.0, logMin + t * (logMax - logMin))
                                        : fMin + t * (fMax - fMin);
        const double omega = 2.0 * M_PI * freq / sampleRate;
        const Complex z(std::cos(omega), std::sin(omega));

        const ComplexResponse resp = evaluate(m, z);
        const Complex H = resp.H[outIdx][inIdx];
        const double mag = std::abs(H);
        const double magDb = 20.0 * std::log10(std::max(mag, 1e-12));
        double phase = std::atan2(H.imag(), H.real());

        if (first) { prevPhase = phase; first = false; }
        else
        {
            double delta = phase - prevPhase;
            while (delta > M_PI)  { unwrapOffset -= 2.0 * M_PI; delta -= 2.0 * M_PI; }
            while (delta < -M_PI) { unwrapOffset += 2.0 * M_PI; delta += 2.0 * M_PI; }
            prevPhase = phase;
        }

        out.push_back({ freq, magDb, phase + unwrapOffset });
    }
}

double TransferFunction::normalizeToUnityPassbandGain(StateSpaceMatrices& m, double sampleRate)
{
    if (sampleRate <= 0.0) sampleRate = 44100.0;
    const double fMax = std::min(20000.0, sampleRate * 0.5 - 1.0);

    std::vector<SweepPoint> pts;
    sweep(m, sampleRate, 0, 0, 20.0, fMax, 300, pts, true);

    double peakDb = -1000.0;
    for (const auto& p : pts) peakDb = std::max(peakDb, p.magnitudeDb);

    // Effectively silent (e.g. a fully zeroed model) -- nothing sane to
    // normalize against, leave it untouched.
    if (peakDb <= -180.0) return 1.0;

    const double scale = std::pow(10.0, -peakDb / 20.0);
    const int n = clampOrder(m.order);
    for (int o = 0; o < kNumOutputs; ++o)
    {
        for (int j = 0; j < n; ++j) m.C[o][j] *= scale;
        for (int i = 0; i < kNumInputs; ++i) m.D[o][i] *= scale;
    }
    return scale;
}

} // namespace ssfilter
