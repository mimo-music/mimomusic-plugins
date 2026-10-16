// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// LimiterDsp.hpp
//
// The Limiter's gain staging, clip stage, ratio choices and oversampled
// peak detector, around its two gain computers (CompressorDsp.hpp's
// CompressorVoice for Classic, TruePeakLimiterDsp.hpp's
// TruePeakGainComputer for True Peak). Framework-independent and header-only, shared by
// the plugin (PluginLimiter.cpp) and its graph (LimiterGraphWidget.cpp)
// so the drawn curve is computed by exactly the code the audio runs.
//
//   - computeLimiterStaging(): where the limiter limits and how much gain
//     follows it. Output Gain and Boost come AFTER the gain computer, so
//     uncapped they would lift the limited peaks over the Ceiling and the
//     clip stage would remove the excess by clipping, not limiting. The
//     gain after the limiter is therefore capped so the limited peaks land
//     on the Ceiling (Hard) or just below the soft-clip band (Soft), and
//     the clip stage only has to catch overshoot.
//   - softClip(): the Soft clip mode -- untouched up to the knee start,
//     then a tanh curve that bends towards the Ceiling and never crosses it.
//   - TruePeakDetector: 4x oversampled peak detection, so peaks between
//     samples (inter-sample peaks) are seen by the gain computer too.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace sscomp {

// ---- Classic's ratio ------------------------------------------------------
//
// Classic's gain computer is a compressor curve, so it has a ratio: 20:1
// (the default -- a signal 12 dB over the Threshold comes out
// 0.6 dB over it), 100:1, or infinity:1 (flat above the Threshold; slope
// 1/inf = 0 in computeGainReductionDb()). A ratio only shapes sustained
// levels: the envelope follower's lag still lets fast transients overshoot.
// True Peak is a brick wall by construction -- always infinity:1.
constexpr int kLimiterRatioCount = 3;

inline double limiterRatioValue(int index)
{
    switch (index)
    {
    case 1:  return 100.0;
    case 2:  return std::numeric_limits<double>::infinity();
    default: return 20.0;
    }
}

inline const char* limiterRatioLabel(int index) // UTF-8; U+221E is in DPF's DejaVu Sans
{
    switch (index)
    {
    case 1:  return "100:1";
    case 2:  return "\xE2\x88\x9E:1";
    default: return "20:1";
    }
}

// ---- gain staging ---------------------------------------------------------

struct LimiterStaging
{
    double effectiveThresholdDb; // what the gain computer actually limits to
    double postGainDb;           // gain applied after the limiter (Output Gain + Boost, after the cap)
    double requestedPostGainDb;  // Output Gain + Boost as set
    double targetTopDb;          // where limited peaks may sit: Ceiling (Hard) or Ceiling - Clip Knee (Soft)
};

// Output Gain + Boost is capped so that Threshold + post gain <= targetTop.
// The cap only ever takes away positive gain: if even 0 dB after the
// limiter would put the Threshold above targetTop (Ceiling set below
// Threshold), the gain computer limits lower instead -- turning the whole
// signal down would make the Ceiling act like a volume knob. A negative
// Output Gain is never changed.
inline LimiterStaging computeLimiterStaging(double thresholdDb, double ceilingDb, double outputGainDb,
                                            bool boost, bool softClip, double clipKneeDb)
{
    LimiterStaging s;
    s.targetTopDb = softClip ? ceilingDb - std::max(0.0, clipKneeDb) : ceilingDb;
    s.requestedPostGainDb = outputGainDb + (boost ? -thresholdDb : 0.0);
    const double cap = s.targetTopDb - thresholdDb;
    s.postGainDb = std::min(s.requestedPostGainDb, std::max(cap, std::min(s.requestedPostGainDb, 0.0)));
    s.effectiveThresholdDb = std::min(thresholdDb, s.targetTopDb - s.postGainDb);
    return s;
}

// ---- output clipping ------------------------------------------------------

inline double hardClip(double x, double ceilingLin)
{
    return std::max(-ceilingLin, std::min(ceilingLin, x));
}

// Same curve as softLimitListen() (CompressorDsp.hpp), with the knee start
// and the ceiling as arguments: |x| <= kneeLin passes untouched, above it
// kneeLin + room*tanh((|x|-kneeLin)/room) -- continuous with slope 1 at the
// knee start, monotonic, odd, and below ceilingLin for every input.
inline double softClip(double x, double kneeLin, double ceilingLin)
{
    const double a = std::fabs(x);
    if (a <= kneeLin) return x;
    const double room = ceilingLin - kneeLin;
    if (room <= 0.0) return hardClip(x, ceilingLin);
    const double y = kneeLin + room * std::tanh((a - kneeLin) / room);
    return x < 0.0 ? -y : y;
}

// ---- 4x oversampled peak detection ------------------------------------------
//
// Estimates the true (inter-sample) peak like an ITU-R BS.1770 true-peak
// meter: 3 extra points between each pair of samples (4x) from a polyphase
// FIR, a Kaiser-windowed sinc with 24 taps per phase, computed once in the
// constructor. A plain 4x grid still misses the top of a high tone that
// falls between two grid points (cos(pi*f/(4*fs)): -0.3 dB at 16 kHz,
// -0.47 dB at 20 kHz, 48 kHz rate), so each local maximum on the grid is
// refined with a parabola through it and its two neighbours -- measured
// in dsp_regression to within a few hundredths of a dB up to 20 kHz.
//
// The interpolation between samples c and c+1 needs samples up to c+12, so
// the detector reports the peak around sample c = (newest - kDelay): the
// caller delays the audio by kDelay more samples to keep both aligned.
// Per sample and channel: 3 points x 24 taps, plus the parabola fits (the
// interval to the left of c was done one sample earlier and is reused).
class TruePeakDetector
{
public:
    static constexpr int kTaps = 24;        // per phase
    static constexpr int kDelay = kTaps / 2; // samples, see above
    static constexpr int kPoints = 3;       // interpolated points between two samples (4x)

    TruePeakDetector()
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kBeta = 6.0;
        constexpr double kHalfSpan = double(kTaps) * 0.5;
        auto besselI0 = [](double x) {
            double sum = 1.0, term = 1.0;
            for (int k = 1; k < 30; ++k) { term *= (x / (2.0 * k)) * (x / (2.0 * k)); sum += term; }
            return sum;
        };
        for (int p = 0; p < kPoints; ++p)
        {
            const double t = double(p + 1) / double(kPoints + 1); // 0.25, 0.5, 0.75
            double sum = 0.0;
            for (int j = 0; j < kTaps; ++j)
            {
                // Tap j multiplies sample c + m, m = j - (kDelay - 1).
                const double u = t - double(j - (kDelay - 1));
                const double sinc = std::fabs(u) < 1e-12 ? 1.0 : std::sin(kPi * u) / (kPi * u);
                const double r = u / kHalfSpan;
                const double win = std::fabs(r) >= 1.0 ? 0.0 : besselI0(kBeta * std::sqrt(1.0 - r * r)) / besselI0(kBeta);
                coef_[p][j] = sinc * win;
                sum += coef_[p][j];
            }
            for (int j = 0; j < kTaps; ++j) coef_[p][j] /= sum; // unity gain at DC
        }
        reset();
    }

    void reset() noexcept
    {
        std::fill(std::begin(hist_), std::end(hist_), 0.0);
        pos_ = 0;
        prevIntervalPeak_ = 0.0;
        prevLastPoint_ = 0.0;
    }

    // Pushes the newest sample; returns the estimated peak magnitude around
    // the sample kDelay samples older (between its two neighbours).
    inline double process(double x) noexcept
    {
        hist_[pos_] = x;
        pos_ = (pos_ + 1) & kMask;
        // The kTaps newest samples, oldest first, are c-(kDelay-1) .. c+kDelay.
        double w[kTaps];
        for (int j = 0; j < kTaps; ++j) w[j] = hist_[(pos_ - kTaps + j) & kMask];

        // q[0] = c - 1/4 (from the previous call), q[1] = c, q[2..4] = the
        // interpolated points, q[5] = c + 1.
        double q[kPoints + 3];
        q[0] = prevLastPoint_;
        q[1] = w[kDelay - 1];
        for (int p = 0; p < kPoints; ++p)
        {
            double acc = 0.0;
            for (int j = 0; j < kTaps; ++j) acc += coef_[p][j] * w[j];
            q[p + 2] = acc;
        }
        q[kPoints + 2] = w[kDelay];

        // Grid points c .. c+3/4 (c+1 is refined on the next call, as its c).
        double peak = 0.0;
        for (int i = 1; i <= kPoints + 1; ++i)
            peak = std::max(peak, refinedPeak(q[i - 1], q[i], q[i + 1]));
        peak = std::max(peak, std::fabs(q[kPoints + 2]));

        const double level = std::max(peak, prevIntervalPeak_);
        prevIntervalPeak_ = peak;
        prevLastPoint_ = q[kPoints + 1];
        return level;
    }

private:
    // |b|, or the vertex of the parabola through (a, b, c) when b is a
    // local extreme of the three (equally spaced) points.
    static inline double refinedPeak(double a, double b, double c) noexcept
    {
        const double s = b < 0.0 ? -1.0 : 1.0;
        const double sa = s * a, sb = s * b, sc = s * c;
        const double curv = sa - 2.0 * sb + sc;
        if (sb >= sa && sb >= sc && curv < 0.0)
            return sb - (sc - sa) * (sc - sa) / (8.0 * curv);
        return sb;
    }

    static constexpr int kHist = 32; // power of two >= kTaps
    static constexpr int kMask = kHist - 1;
    double coef_[kPoints][kTaps] = {};
    double hist_[kHist] = {};
    int pos_ = 0;
    double prevIntervalPeak_ = 0.0;
    double prevLastPoint_ = 0.0;
};

} // namespace sscomp
