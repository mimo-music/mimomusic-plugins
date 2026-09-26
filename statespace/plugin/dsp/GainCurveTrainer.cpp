// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "GainCurveTrainer.hpp"
#include "StabilityGuard.hpp"
#include "StateSpaceEngine.hpp"

#include <algorithm>
#include <cmath>

namespace ssfilter {

namespace {

constexpr int kK = GainCurve::kNumKnots;

// Solves (A + lambda*I) x = b for a symmetric positive-(semi)definite A
// via Cholesky decomposition. Returns false if A+lambda*I isn't SPD
// (shouldn't happen with any lambda > 0, but this is offline fitting code,
// not RT, so there's no harm double-checking).
bool solveSPD(const double A[kK][kK], const double b[kK], double xOut[kK])
{
    double L[kK][kK] = {};
    for (int i = 0; i < kK; ++i)
    {
        for (int j = 0; j <= i; ++j)
        {
            double sum = A[i][j];
            for (int p = 0; p < j; ++p) sum -= L[i][p] * L[j][p];
            if (i == j)
            {
                if (sum <= 0.0) return false;
                L[i][j] = std::sqrt(sum);
            }
            else
            {
                L[i][j] = sum / L[j][j];
            }
        }
    }

    double y[kK];
    for (int i = 0; i < kK; ++i)
    {
        double sum = b[i];
        for (int p = 0; p < i; ++p) sum -= L[i][p] * y[p];
        y[i] = sum / L[i][i];
    }
    for (int i = kK - 1; i >= 0; --i)
    {
        double sum = y[i];
        for (int p = i + 1; p < kK; ++p) sum -= L[p][i] * xOut[p];
        xOut[i] = sum / L[i][i];
    }
    return true;
}

// Interpolation weights for one envelope-dB reading against the fixed
// GainCurve knot grid: returns the lower knot index and the fractional
// blend toward the next one (clamped to the valid knot range).
void curveWeights(double levelDb, int& i0, double& frac)
{
    double t = (levelDb - GainCurve::kMinDb) / GainCurve::kStepDb;
    t = std::max(0.0, std::min(double(kK - 1) - 1e-9, t));
    i0 = int(t);
    frac = t - double(i0);
}

} // namespace

GainCurveFitResult GainCurveTrainer::fit(const StateSpaceMatrices& baseModel, double sampleRate,
                                          const std::vector<float>& inputL, const std::vector<float>& inputR,
                                          const std::vector<float>& targetL, const std::vector<float>& targetR)
{
    GainCurveFitResult result;
    const uint32_t n = uint32_t(std::min({ inputL.size(), inputR.size(), targetL.size(), targetR.size() }));
    if (n == 0 || sampleRate <= 0.0) return result;

    // 1. Run the plain LTI model (nonlinear disabled) to get the "base"
    //    (pre-gain) signal the curve is going to multiply.
    StateSpaceMatrices base = baseModel;
    base.nonlinear = NonlinearSchedule();
    StateSpaceEngine eng;
    eng.prepare(sampleRate);
    eng.setModel(base);
    eng.resetState();
    std::vector<float> baseL(n), baseR(n);
    eng.processBlock(inputL.data(), inputR.data(), baseL.data(), baseR.data(), n);

    // 2. Precompute the raw input level -- in *linear* amplitude, matching
    //    StateSpaceEngine::processBlock() exactly (smoothing the envelope
    //    in dB rather than linear makes attack/release behavior depend on
    //    the signal's frequency, since |sample| touches 0 at every zero
    //    crossing -- see the comment there) -- and the "instantaneous
    //    ideal gain" implied by target-vs-base at each sample.
    std::vector<double> rawLevelLin(n);
    std::vector<double> targetGainDb(n);
    std::vector<char> valid(n, 0);
    constexpr double kFloorLin = 1e-4; // ~ -80 dBFS: below this, target/base is measuring noise, not signal
    uint32_t validCount = 0;
    for (uint32_t i = 0; i < n; ++i)
    {
        rawLevelLin[i] = std::max(std::fabs(double(inputL[i])), std::fabs(double(inputR[i])));

        const double baseLevel = std::max(std::fabs(double(baseL[i])), std::fabs(double(baseR[i])));
        const double tgtLevel = std::max(std::fabs(double(targetL[i])), std::fabs(double(targetR[i])));
        if (baseLevel > kFloorLin)
        {
            targetGainDb[i] = 20.0 * std::log10(std::max(tgtLevel, 1e-9) / baseLevel);
            valid[i] = 1;
            ++validCount;
        }
    }
    if (validCount < uint32_t(kK))
        return result; // not enough non-silent material to fit anything meaningful

    // 3. Grid search over (attackMs, releaseMs); for each, solve the
    //    ridge-regularized linear least squares for the curve knot values
    //    and keep whichever grid point has the lowest residual. See the
    //    header comment for why this "solve the linear part exactly,
    //    brute-force only the 2 nonlinear parameters" split is well-posed
    //    rather than a general (slower, less reliable) joint optimization.
    static constexpr double kAttackGridMs[]  = { 0.2, 1.0, 3.0, 10.0, 30.0, 100.0 };
    static constexpr double kReleaseGridMs[] = { 10.0, 30.0, 100.0, 300.0, 1000.0, 3000.0 };
    constexpr double kRidgeLambda = 4.0; // dB^2 -- pulls knots with little/no data toward 0 dB (transparent)

    double bestRms = 1e300;

    for (double attackMs : kAttackGridMs)
    {
        for (double releaseMs : kReleaseGridMs)
        {
            const double attackCoeff  = std::exp(-1.0 / (0.001 * attackMs  * sampleRate));
            const double releaseCoeff = std::exp(-1.0 / (0.001 * releaseMs * sampleRate));

            double AtA[kK][kK] = {};
            double Atb[kK] = {};
            double envLin = 1e-9;

            for (uint32_t i = 0; i < n; ++i)
            {
                const double coeff = (rawLevelLin[i] > envLin) ? attackCoeff : releaseCoeff;
                envLin = coeff * envLin + (1.0 - coeff) * rawLevelLin[i];
                if (!valid[i]) continue;

                int i0; double frac;
                curveWeights(20.0 * std::log10(envLin), i0, frac);
                const double w0 = 1.0 - frac, w1 = frac;

                AtA[i0][i0]         += w0 * w0;
                AtA[i0][i0 + 1]     += w0 * w1;
                AtA[i0 + 1][i0]     += w1 * w0;
                AtA[i0 + 1][i0 + 1] += w1 * w1;
                Atb[i0]     += w0 * targetGainDb[i];
                Atb[i0 + 1] += w1 * targetGainDb[i];
            }

            for (int k = 0; k < kK; ++k) AtA[k][k] += kRidgeLambda;

            double g[kK];
            if (!solveSPD(AtA, Atb, g)) continue;

            // Residual: re-walk the envelope (same trace as above) and
            // compare the now-fitted curve's prediction against target.
            envLin = 1e-9;
            double sumSq = 0.0;
            uint32_t count = 0;
            for (uint32_t i = 0; i < n; ++i)
            {
                const double coeff = (rawLevelLin[i] > envLin) ? attackCoeff : releaseCoeff;
                envLin = coeff * envLin + (1.0 - coeff) * rawLevelLin[i];
                if (!valid[i]) continue;

                int i0; double frac;
                curveWeights(20.0 * std::log10(envLin), i0, frac);
                const double predicted = g[i0] + frac * (g[i0 + 1] - g[i0]);
                const double err = predicted - targetGainDb[i];
                sumSq += err * err;
                ++count;
            }
            const double rms = count > 0 ? std::sqrt(sumSq / double(count)) : 1e300;

            if (rms < bestRms)
            {
                bestRms = rms;
                result.schedule.enabled = true;
                result.schedule.attackMs = attackMs;
                result.schedule.releaseMs = releaseMs;
                for (int k = 0; k < kK; ++k) result.schedule.curve.gainDb[k] = g[k];
                result.samplesUsed = validCount;
            }
        }
    }

    result.rmsErrorDb = bestRms;
    result.ok = result.schedule.enabled;
    return result;
}

GainCurveTrainer::MultibandFitResult GainCurveTrainer::fitMultiband(
    const std::vector<std::vector<BiquadCoeffs>>& bandStages, double sampleRate,
    const std::vector<float>& inputL, const std::vector<float>& inputR,
    const std::vector<float>& targetL, const std::vector<float>& targetR)
{
    MultibandFitResult result;
    const int numBands = std::min(int(bandStages.size()), kMaxBands);
    result.numBands = numBands;
    if (numBands <= 0 || sampleRate <= 0.0) return result;

    // Assemble the combined crossover once, purely to get each band's own
    // state range/feedD (the BandSplit bookkeeping) -- the per-band
    // *audio* filtering below uses each band's own standalone chain
    // instead (see this function's header comment), since a standalone
    // single-band system is exactly what's needed to hear that band alone.
    MultibandSchedule layout;
    mappers::makeStereoCrossoverBank(bandStages, layout);
    result.schedule = layout;
    result.schedule.enabled = true;

    const uint32_t n = uint32_t(std::min({ inputL.size(), inputR.size(), targetL.size(), targetR.size() }));

    bool allOk = true;
    for (int k = 0; k < numBands; ++k)
    {
        StateSpaceMatrices bandOnly = mappers::makeStereoBiquadChain(bandStages[size_t(k)], bandStages[size_t(k)]);
        StabilityGuard::enforce(bandOnly);

        StateSpaceEngine eng;
        eng.prepare(sampleRate);
        eng.setModel(bandOnly);
        eng.resetState();

        // Phase-matching, and why it's essential: any *sane* multiband
        // compressor's own true target audio was built as
        // "sum_k(gain_k(t) * filter_k(rawInput))" -- band k's own true
        // contribution has already gone through filter_k *once* before we
        // ever see the (opaque, already-summed) target. Recovering band
        // k's own signal from that target means filtering it through
        // filter_k a *second* time here -- so band k's true contribution
        // ends up phase-shifted by *two* applications of filter_k's own
        // phase response, while `inputL/R` filtered the ordinary way
        // (once) is only phase-shifted by *one*. fit()'s per-sample
        // instantaneous-ratio technique (targetGainDb[i] =
        // 20*log10(target[i]/base[i])) is meaningless when comparing two
        // differently-phase-shifted copies of the same waveform, even
        // when their true *envelope-level* relationship (what's actually
        // being learned) is identical -- two sine waves of the same
        // amplitude but different phase divide pointwise into noise, not
        // a constant. Filtering the input through the *same* filter
        // *twice* too (discarding the once-filtered intermediate) matches
        // that same two-application phase shift, so the only remaining
        // difference between the two traces fit() compares is the actual
        // gain being learned.
        std::vector<float> bandInOnceL(n), bandInOnceR(n);
        eng.processBlock(inputL.data(), inputR.data(), bandInOnceL.data(), bandInOnceR.data(), n);
        eng.resetState();
        std::vector<float> bandInL(n), bandInR(n);
        eng.processBlock(bandInOnceL.data(), bandInOnceR.data(), bandInL.data(), bandInR.data(), n);

        // Fresh state for the target pass -- same filter, independent
        // signal, so the passes above must not share any recursion history
        // with this one.
        eng.resetState();
        std::vector<float> bandTgtL(n), bandTgtR(n);
        eng.processBlock(targetL.data(), targetR.data(), bandTgtL.data(), bandTgtR.data(), n);

        // Identity base model: the (now phase-matched) bandInL/R already
        // *is* "the signal this band's envelope should follow and this
        // band's curve should start from" (see this function's header
        // comment) -- fit()'s own envelope tracking (over the inputL/R
        // args below) and its own "base" computation (identity reproduces
        // its input unchanged) both do the right thing unmodified.
        const GainCurveFitResult bandFit = fit(StateSpaceMatrices(), sampleRate, bandInL, bandInR, bandTgtL, bandTgtR);
        result.rmsErrorDb[k] = bandFit.rmsErrorDb;
        result.samplesUsed[k] = bandFit.samplesUsed;
        result.schedule.bands[k].schedule = bandFit.schedule;
        if (!bandFit.ok) allOk = false;
    }

    result.ok = allOk;
    return result;
}

} // namespace ssfilter
