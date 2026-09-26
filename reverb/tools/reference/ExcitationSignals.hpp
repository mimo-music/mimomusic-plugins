// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ExcitationSignals.hpp
//
// Dry test/probe signals for the offline reference generator
// (tools/reference/generate_reference_samples.cpp) -- clapping hands is
// the common real-world way to hear a room's reflections -- plus other
// material to render through the expensive reference engine and keep as
// training targets.
//
// Every signal here is SYNTHESIZED, not sourced -- this environment has
// no way to fetch or license real dry recordings, and fabricating fake
// "dry music" would be low-value filler, not a real test asset (the user
// separately offered to supply their own dry recordings for exactly that
// role; see generate_reference_samples.cpp's own usage comment for where
// those go). What's here are the standard ACOUSTIC MEASUREMENT signals
// real-world room testing actually uses -- broadband transients and
// sweeps whose entire point is exciting a room evenly, not being
// pleasant to listen to dry.

#pragma once

#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace ssfilter {
namespace reverb {
namespace reference {

// A single unit impulse -- the simplest, idealized probe (infinite
// bandwidth in theory; in practice band-limited by nothing here at all,
// letting the reference engine's own physics do 100% of the shaping).
// Useful as a baseline/sanity check signal, not a real-world technique.
inline std::vector<float> impulseSignal(double sampleRate, double silenceTailSec) {
    std::vector<float> out(1 + (size_t)(silenceTailSec * sampleRate), 0.0f);
    out[0] = 1.0f;
    return out;
}

// A synthesized hand-clap-like transient. A real clap is not a
// clean impulse: it's a short (roughly 5-15ms) broadband noise burst
// with a near-instant attack, a fast decay, and a mild spectral emphasis
// around 2-4kHz from the cupped-hands cavity resonance. Approximated
// here as filtered noise under a fast-attack/fast-decay envelope, honest
// about being an approximation (a real recorded clap would be a better
// reference than this synthesis, which is why real recordings dropped
// into user_dry/ are picked up too -- this is what to reach for
// when a real recording isn't available, not a claim of being
// equivalent to one).
inline std::vector<float> clapSignal(double sampleRate, double silenceTailSec, uint32_t seed = 1) {
    const double burstSec = 0.012;      // ~12ms burst, in the typical real-clap range
    const double attackSec = 0.0006;    // near-instant attack
    const size_t burstN = (size_t)(burstSec * sampleRate);
    const size_t attackN = std::max((size_t)1, (size_t)(attackSec * sampleRate));
    const size_t tailN = (size_t)(silenceTailSec * sampleRate);

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-1.0, 1.0);

    std::vector<float> out(burstN + tailN, 0.0f);

    // Broadband noise burst under a fast-attack/exponential-decay envelope.
    std::vector<double> raw(burstN);
    for (size_t i = 0; i < burstN; ++i) {
        double env;
        if (i < attackN) env = (double)i / (double)attackN;
        else env = std::exp(-6.0 * (double)(i - attackN) / (double)burstN);
        raw[i] = dist(rng) * env;
    }

    // A mild resonant emphasis around 2.5-3kHz (a simple one-pole
    // bandpass-ish shaping: high-pass then low-pass in series) --
    // approximates the cupped-hands cavity resonance without pulling in
    // a full biquad implementation for a one-off synthesis helper.
    const double fcLow = 2500.0, fcHigh = 4000.0;
    const double aLow = std::exp(-2.0 * M_PI * fcLow / sampleRate);
    const double aHigh = std::exp(-2.0 * M_PI * fcHigh / sampleRate);
    double lpZ = 0.0, hpZ = 0.0;
    double peak = 0.0;
    std::vector<double> shaped(burstN);
    for (size_t i = 0; i < burstN; ++i) {
        // high-pass (input minus its own low-pass) to remove sub-bass rumble
        const double hpLp = (1 - aHigh) * raw[i] + aHigh * hpZ;
        hpZ = hpLp;
        const double hp = raw[i] - hpLp;
        // low-pass to tame anything above the resonance band
        lpZ = (1 - aLow) * hp + aLow * lpZ;
        shaped[i] = lpZ;
        peak = std::max(peak, std::fabs(shaped[i]));
    }
    const double norm = peak > 1e-9 ? 0.9 / peak : 1.0;
    for (size_t i = 0; i < burstN; ++i) out[i] = (float)(shaped[i] * norm);

    return out;
}

// A short white-noise burst -- useful for the same steady-state/RMS-
// style probing this project's own regression tests already rely on,
// just rendered through the (expensive) physical reference instead of a
// real-time engine.
inline std::vector<float> noiseBurstSignal(double sampleRate, double burstSec, double silenceTailSec, uint32_t seed = 2) {
    const size_t burstN = (size_t)(burstSec * sampleRate);
    const size_t tailN = (size_t)(silenceTailSec * sampleRate);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);
    std::vector<float> out(burstN + tailN, 0.0f);
    for (size_t i = 0; i < burstN; ++i) out[i] = (float)dist(rng);
    return out;
}

// An exponential ("Farina") sine sweep, 20Hz-20kHz -- the actual
// professional-standard room/IR measurement technique (higher SNR and
// more robust to nonlinearity/noise than a literal clap or impulse in
// real-world measurement, though the clap is generated too -- both, so
// later comparisons/fits have the choice). Included as the complementary,
// higher-fidelity-in-practice option alongside the more intuitive clap.
inline std::vector<float> sineSweepSignal(double sampleRate, double sweepSec, double silenceTailSec) {
    const double f1 = 20.0, f2 = 20000.0;
    const size_t n = (size_t)(sweepSec * sampleRate);
    const size_t tailN = (size_t)(silenceTailSec * sampleRate);
    std::vector<float> out(n + tailN, 0.0f);
    const double K = sweepSec * f1 / std::log(f2 / f1);
    const double L = sweepSec / std::log(f2 / f1);
    for (size_t i = 0; i < n; ++i) {
        const double t = (double)i / sampleRate;
        const double phase = 2.0 * M_PI * K * (std::exp(t / L) - 1.0);
        // A short fade-in/out (5ms) so this doesn't click at either end.
        const double fadeSec = 0.005;
        double env = 1.0;
        if (t < fadeSec) env = t / fadeSec;
        else if (t > sweepSec - fadeSec) env = (sweepSec - t) / fadeSec;
        out[i] = (float)(0.9 * env * std::sin(phase));
    }
    return out;
}

} // namespace reference
} // namespace reverb
} // namespace ssfilter
