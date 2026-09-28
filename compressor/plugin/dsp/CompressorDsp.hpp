// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// CompressorDsp.hpp
//
// Framework-independent core of every compressor voice in this project --
// no DPF/DGL dependency, so it's shared verbatim by the Mono and Stereo
// plugins' DSP sides. Entirely header-only (unlike the sibling
// statespace project, which needs .cpp files for its
// stability guard / transfer function / JSON because those are genuinely
// large; a compressor's per-sample math is small enough that splitting
// declaration from definition would only add bookkeeping without a benefit).
//
// Design, in one paragraph: a feed-forward compressor with a *linear-domain*
// peak envelope follower (asymmetric attack/release one-pole), converted to
// dB only at the point the static soft-knee gain-computer curve is
// evaluated. Smoothing the envelope in the linear domain rather than the dB
// domain is a deliberate, previously-learned lesson (from the sibling
// statespace project's own nonlinear stage): a dB-domain envelope touches
// -infinity at every zero crossing of an AC signal, which makes the *same*
// threshold/ratio/attack/release numbers compress bass and treble
// differently -- purely an artifact of where the smoothing happens, not a
// real property of compression. Smoothing happens once, in linear
// amplitude, before any log is taken, so a fixed attack/release time reads
// the same regardless of the input frequency. This whole design is exactly
// the standard "level detector -> static curve" feed-forward architecture
// described in Giannoulis/Massberg/Reiss, "Digital Dynamic Range Compressor
// Design -- A Tutorial and Analysis" (2012) -- one multiply-add per sample
// for the envelope, one log10/pow pair for the dB round-trip, no
// transcendentals beyond that: fast and allocation-free, safe to call from
// the audio thread.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace sscomp {

// ---- dB <-> linear amplitude ----------------------------------------------

inline double linearToDb(double linear)
{
    return 20.0 * std::log10(std::max(1.0e-12, linear));
}

inline double dbToLinear(double db)
{
    return std::pow(10.0, db * 0.05);
}

// Range of the sidechain / trigger amplification. It used to stop at +24dB; a NARROW
// band-pass leaves very little energy in the trigger signal, so it needs far more
// -- up to +60dB (1000x).
// The amplification only ever feeds the detector (and, on request, Listen -- see
// softLimitListen()), never the audio path.
constexpr double kSidechainGainMinDb = -24.0;
constexpr double kSidechainGainMaxDb = 60.0;

// Listen plays the trigger signal, amplification included, in place of the output. With up
// to +60dB of trigger gain that could be a 1000x blast, so Listen (and only Listen) goes
// through this: samples up to +-0.7 pass untouched, above that they bend smoothly towards a
// hard ceiling of +-1.0 (tanh knee) -- continuous, monotonic, bounded whatever the input.
inline double softLimitListen(double x)
{
    constexpr double kKnee = 0.7, kRoom = 1.0 - kKnee;
    const double a = std::fabs(x);
    if (a <= kKnee) return x;
    const double y = kKnee + kRoom * std::tanh((a - kKnee) / kRoom);
    return x < 0.0 ? -y : y;
}

// ---- the static soft-knee gain-computer curve -----------------------------
//
// Returns the gain *change* in dB for a detected level `levelDb` given
// threshold/knee and two independent ratios: `ratioDown` shapes the
// classic downward-compression line above the threshold (gain <= 0,
// reducing loud material), `ratioUp` shapes the mirrored upward-
// compression line below it (gain >= 0, lifting quiet material) --
// downward and upward compression in one control pair rather than a mode
// switch, since the two lines are independent and can be dialed in (or
// left at 1.0, meaning "no effect on this side") simultaneously. Both
// default to the historical single-`ratio` downward-only behavior when
// `ratioUp` is 1.0 -- see this function's own derivation below for why
// that's exact, not approximate.
//
// The shape is the standard piecewise-quadratic soft-knee construction
// (Giannoulis et al., eq. 4), generalized from "identity below, ratio
// line above" to "one ratio line on each side of the threshold, meeting
// at the threshold with matching value by construction, blended through
// the knee with matching value *and* slope at both edges". Concretely,
// picture the *output* (not the gain change) as a function of `levelDb`,
// expressed via `overshoot = levelDb - thresholdDb`:
//
//   outDb = thresholdDb + slope * overshoot,  slope = 1/ratioUp for overshoot < 0, 1/ratioDown for overshoot > 0
//
// -- two lines that already agree in value (both equal thresholdDb) at
// overshoot = 0, so the only discontinuity to smooth over is their
// *slope*. For any two slopes s0 (left of a kink), s1 (right of it)
// meeting at a point, quadratic-blending them across a window of half-
// width `kneeHalf` centered on the kink is a standard, closed-form
// construction:
//
//   outDb = [left line](x) + (s1 - s0) * (x - kinkStart)^2 / (4*kneeHalf)
//
// which matches both value and slope at the *both* ends of the window
// (verified directly by substitution -- at x=kinkStart the correction and
// its derivative are both 0, so it equals the left line and its slope
// exactly; at the far edge the correction evaluates to (s1-s0)*kneeHalf,
// exactly the gap between the two lines' values there, with derivative
// (s1-s0), exactly closing the slope gap too). With s0=1 (ratioUp=1, i.e.
// "identity below") this reduces exactly to the original single-ratio
// formula this project shipped with first -- confirmed by direct
// substitution: (s1-s0)/(4*kneeHalf) = (1/ratioDown - 1)/(2*kneeDb),
// exactly the original `(1/ratio - 1) * t*t / (2*kneeDb)` coefficient.
//
// Pure function, no state -- used both by the real-time engine (per
// sample) and directly by the GUI (to draw the static input/output
// transfer curve, which needs exactly this function evaluated at a few
// hundred x positions, not the run-time envelope).
inline double computeGainReductionDb(double levelDb, double thresholdDb, double ratioDown, double ratioUp, double kneeDb)
{
    ratioDown = std::max(1.0, ratioDown);
    ratioUp = std::max(1.0, ratioUp);
    kneeDb = std::max(0.0, kneeDb);
    const double kneeHalf = kneeDb * 0.5;
    const double overshoot = levelDb - thresholdDb;
    const double slopeBelow = 1.0 / ratioUp;   // ratioUp == 1 -> slope 1 -> identity below, the original "untouched" behavior
    const double slopeAbove = 1.0 / ratioDown; // ratioDown == 1 -> slope 1 -> identity above, i.e. downward compression off

    double outDb;
    if (overshoot < -kneeHalf)
    {
        outDb = thresholdDb + slopeBelow * overshoot;
    }
    else if (overshoot > kneeHalf)
    {
        outDb = thresholdDb + slopeAbove * overshoot;
    }
    else
    {
        // Inside the knee: quadratic blend from slopeBelow to slopeAbove,
        // see this function's own derivation above.
        const double t = overshoot + kneeHalf; // 0 at the knee's low edge, kneeDb at its high edge
        const double leftLine = thresholdDb + slopeBelow * overshoot;
        outDb = leftLine + (slopeAbove - slopeBelow) * (t * t) / (2.0 * std::max(1.0e-9, kneeDb));
    }
    return outDb - levelDb;
}

// ---- envelope follower: linear-domain, asymmetric attack/release ---------

class EnvelopeFollower
{
public:
    void prepare(double sampleRate) { sampleRate_ = std::max(1.0, sampleRate); updateCoeffs(); }

    void setTimesMs(double attackMs, double releaseMs)
    {
        attackMs_ = attackMs;
        releaseMs_ = releaseMs;
        updateCoeffs();
    }

    // Feed one rectified (non-negative) sample, get back the current
    // envelope value (also non-negative, linear amplitude).
    inline double process(double absIn)
    {
        const double coeff = (absIn > env_) ? attackCoeff_ : releaseCoeff_;
        env_ = coeff * env_ + (1.0 - coeff) * absIn;
        return env_;
    }

    void reset(double value = 0.0) { env_ = value; }
    double value() const { return env_; }

private:
    static double coeffFromMs(double ms, double sampleRate)
    {
        // ms <= 0 means "instant, no smoothing at all" -- a real, exact
        // zero-latency mode (see CurveDsp.hpp's own CurveVoice, whose
        // Contour plugin exposes this all the way to the user: at 0, the
        // audio follows the curve exactly), not a fast approximation
        // of one. Deliberately not just clamping to a tiny-but-positive
        // floor (e.g. 0.01ms) the way this used to: even 0.01ms is a real,
        // if fast, one-pole lag, which is a different (if similar-looking)
        // thing from the literal, sample-accurate tracking a "0" on the
        // control actually promises. coeff=0 makes process()'s own
        // `env_ = coeff*env_ + (1-coeff)*absIn` reduce to `env_ = absIn`
        // every sample -- exact, not asymptotic.
        if (ms <= 0.0) return 0.0;
        return std::exp(-1.0 / (0.001 * ms * sampleRate));
    }

    void updateCoeffs()
    {
        attackCoeff_ = coeffFromMs(attackMs_, sampleRate_);
        releaseCoeff_ = coeffFromMs(releaseMs_, sampleRate_);
    }

    double sampleRate_ = 48000.0;
    double attackMs_ = 10.0, releaseMs_ = 150.0;
    double attackCoeff_ = 0.0, releaseCoeff_ = 0.0;
    double env_ = 0.0;
};

// ---- loudness meter: one-pole mean-square, fixed integration time --------
//
// Deliberately decoupled from the compressor's own attack/release -- this
// is a steady, ~VU-like loudness estimate used for (a) the bargraph's RMS
// fill and (b) the volume-compensation feature below, neither of which
// should itself lurch every time the user drags Attack/Release.
class LoudnessMeter
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        coeff_ = std::exp(-1.0 / (0.001 * kIntegrationMs * sampleRate_));
    }

    inline void process(double x) { meanSq_ = coeff_ * meanSq_ + (1.0 - coeff_) * (x * x); }

    double rmsDb() const { return linearToDb(std::sqrt(std::max(1.0e-12, meanSq_))); }
    void reset() { meanSq_ = 0.0; }

private:
    static constexpr double kIntegrationMs = 300.0;
    double sampleRate_ = 48000.0;
    double coeff_ = 0.0;
    double meanSq_ = 0.0;
};

// ---- peak meter: instant peak + slow decay, for the bargraph's peak line -
class PeakMeter
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        // ~20 dB/s fall, a conventional peak-meter ballistic.
        decayPerSample_ = std::pow(10.0, (-20.0 / 20.0) / sampleRate_);
    }

    inline void process(double absX)
    {
        if (absX > peak_) peak_ = absX;
        else peak_ *= decayPerSample_;
    }

    double peakDb() const { return linearToDb(peak_); }
    double peakLinear() const { return peak_; }
    void reset() { peak_ = 0.0; }

private:
    double sampleRate_ = 48000.0;
    double decayPerSample_ = 0.9999;
    double peak_ = 0.0;
};

// ---- sidechain band filter: isolate a frequency range to trigger from ---
//
// Sidechain amplification plus a band filter, so the compressor can be
// triggered by sound in one specific part of the spectrum. Two simple
// one-pole
// (6dB/oct) filters in series -- a high-pass then a low-pass -- so the
// detector only responds to energy between the two corner frequencies
// (e.g. isolate a kick drum's low end, or a hi-hat/sibilance's high end).
// One-pole, not a steeper multi-pole design: this is a *trigger* aid, not
// a mix-shaping EQ, and a gentle slope is both cheaper and, in practice,
// perfectly adequate for "mostly this range" isolation.
class SidechainBandFilter
{
public:
    // Steeper than the original single one-pole-per-corner design (6dB/oct,
    // too gentle to cleanly isolate a band for triggering). kStages
    // identical one-pole sections cascaded per corner now (24dB/oct at
    // the default of 4) -- each stage a genuine independent 1st-order
    // section (the highpass side re-derives its own local highpass every
    // stage, `x - lowpass(x)`, not one subtraction wrapped around an
    // already-cascaded lowpass, which would only ever give a 1st-order
    // highpass slope no matter how many lowpass stages sit behind it).
    // See updateCoeffs()'s own comment for why each stage is designed at
    // a *different* frequency than the knob's own value.
    static constexpr int kStages = 4;

    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        updateCoeffs();
    }

    void reset()
    {
        for (int i = 0; i < kStages; ++i) { hpState_[i] = 0.0; lpState_[i] = 0.0; }
    }

    void setFrequencies(double highpassHz, double lowpassHz)
    {
        highpassHz_ = highpassHz;
        lowpassHz_ = lowpassHz;
        updateCoeffs();
    }

    // High-pass first (removes everything below the low corner), then
    // low-pass (removes everything above the high corner) -- order doesn't
    // matter mathematically for two independent one-pole cascades, but
    // matches the natural "low cut, then high cut" reading of the two
    // controls.
    inline double process(double x)
    {
        double y = x;
        for (int i = 0; i < kStages; ++i)
        {
            hpState_[i] = hpCoeff_ * hpState_[i] + (1.0 - hpCoeff_) * y;
            y = y - hpState_[i];
        }
        for (int i = 0; i < kStages; ++i)
        {
            lpState_[i] = lpCoeff_ * lpState_[i] + (1.0 - lpCoeff_) * y;
            y = lpState_[i];
        }
        return y;
    }

private:
    void updateCoeffs()
    {
        const double hpHz = std::max(1.0, std::min(sampleRate_ * 0.49, highpassHz_));
        const double lpHz = std::max(1.0, std::min(sampleRate_ * 0.49, lowpassHz_));
        // Cascading kStages identical one-pole sections, each individually
        // built for corner f0, moves the *cascade's* own -3dB point to
        // f0 * sqrt(2^(1/kStages) - 1) (a standard result -- each extra
        // stage's own gain multiplies in, reaching -3dB sooner than any
        // single stage would alone). Design each stage at the knob's
        // target frequency divided by that same factor instead, so the
        // cascade's overall corner still lands where Highpass/Lowpass
        // says -- only the slope beyond it gets steeper, not the corner
        // itself. kStages==1 reduces the factor to exactly 1.0, i.e. the
        // previous uncorrected single-stage behavior.
        const double warp = std::sqrt(std::pow(2.0, 1.0 / double(kStages)) - 1.0);
        // Low-pass: n identical one-pole sections have their cascade -3 dB point at stageHz * warp, so each stage is
        // designed at corner / warp. High-pass is the mirror image -- the cascade's -3 dB point is at stageHz / warp --
        // so each stage is designed at corner * warp. (It used to be corner / warp here too, which put the real
        // high-pass corner at corner / warp^2 ~ 5.3x too high: HP 53 Hz behaved like ~280 Hz.)
        const double hpStageHz = std::min(sampleRate_ * 0.49, hpHz * warp);
        const double lpStageHz = std::min(sampleRate_ * 0.49, lpHz / warp);
        hpCoeff_ = std::exp(-2.0 * M_PI * hpStageHz / sampleRate_);
        lpCoeff_ = std::exp(-2.0 * M_PI * lpStageHz / sampleRate_);
    }

    double sampleRate_ = 48000.0;
    double highpassHz_ = 100.0, lowpassHz_ = 8000.0;
    double hpCoeff_ = 0.0, lpCoeff_ = 0.0;
    double hpState_[kStages] = {};
    double lpState_[kStages] = {};
};

// ---- one compression "voice": one envelope + the static curve -----------
//
// One instance detects and computes gain reduction for one detection
// signal. A stereo-linked mode uses exactly one voice fed max(|L|,|R|),
// applying its output gain to both channels (preserves the stereo image).
// Dual-mono and Mid/Side modes use two independent voices instead.
class CompressorVoice
{
public:
    void prepare(double sampleRate) { env_.prepare(sampleRate); }

    // `ratioUp` defaults to 1.0 (no upward effect) for any caller that
    // hasn't been updated to expose it -- see computeGainReductionDb()'s
    // own comment for why that's exactly, not approximately, the original
    // downward-only behavior.
    void setParams(double attackMs, double releaseMs, double thresholdDb, double ratioDown, double ratioUp, double kneeDb)
    {
        env_.setTimesMs(attackMs, releaseMs);
        thresholdDb_ = thresholdDb;
        ratioDown_ = ratioDown;
        ratioUp_ = ratioUp;
        kneeDb_ = kneeDb;
    }

    // Feed one rectified detector sample; returns the linear gain multiplier
    // to apply to the audio sample(s) this voice controls -- >1 (a boost)
    // when upward compression is lifting a quiet passage, <1 (a cut) when
    // downward compression is reducing a loud one, 1 in between.
    inline double process(double detectorAbs)
    {
        const double envLevel = env_.process(detectorAbs);
        detectorLevelDb_ = linearToDb(envLevel);
        gainReductionDb_ = computeGainReductionDb(detectorLevelDb_, thresholdDb_, ratioDown_, ratioUp_, kneeDb_);
        return dbToLinear(gainReductionDb_);
    }

    double gainReductionDb() const { return gainReductionDb_; }
    double detectorLevelDb() const { return detectorLevelDb_; }
    void reset() { env_.reset(); gainReductionDb_ = 0.0; detectorLevelDb_ = -120.0; }

private:
    EnvelopeFollower env_;
    double thresholdDb_ = -18.0, ratioDown_ = 4.0, ratioUp_ = 1.0, kneeDb_ = 6.0;
    double gainReductionDb_ = 0.0;
    double detectorLevelDb_ = -120.0;
};

// Stereo processing mode -- see docs/ARCHITECTURE.md.
enum class StereoMode
{
    Linked = 0,   // one shared detector (max(|L|,|R|)); identical gain to both channels, stereo image untouched
    DualMono = 1, // two independent detectors, one per channel
    MidSide = 2,  // convert to Mid/Side, two independent detectors (Mid, Side), convert back
};

inline StereoMode clampStereoMode(int raw)
{
    if (raw < 0 || raw > 2) return StereoMode::Linked;
    return StereoMode(raw);
}

} // namespace sscomp
