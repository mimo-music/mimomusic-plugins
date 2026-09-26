// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// PhaseScopeDsp.hpp
//
// Framework-independent core of the Phase Scope plugin -- no DPF/DGL
// dependency, entirely header-only (the same "small enough that splitting
// declaration from definition would only add bookkeeping" reasoning
// ../compressor's plugin/dsp/CompressorDsp.hpp gives for its own DSP core).
// This is the first plugin in this project; "measurement" plugins are
// their own sibling project because, unlike everything in ../compressor
// and ../statespace, they never change the audio.
//
// Four independent, purely-observational pieces, all fed straight from
// the input signal in PluginPhaseScope::run() and never touching the
// audio itself (this plugin passes L/R straight through, unmodified --
// see PluginPhaseScope.cpp's own comment):
//
//   - PeakMeter / LoudnessMeter: the same conventional peak/RMS ballistics
//     ../compressor's own CompressorDsp.hpp uses (see that file's header
//     comment) -- ~20dB/s peak decay, a ~300ms RMS integration -- copied
//     as a design (not literally shared code across the two independent
//     projects), not reinvented.
//   - StereoImageMeter: correlation + stereo width, both derived from one
//     shared set of leaky-integrated L/R statistics -- see its own comment
//     below for why one set of sums drives both readings.
//   - GoniometerBuffer: a decimated, packed ring buffer of Mid/Side scope
//     points for the vectorscope display, written at a fixed rate with a
//     UI-adjustable "Decay" controlling how much of it is actually shown
//     -- see its own comment for both the windowing design and the
//     packing scheme (the same 24-bit-into-one-float trick
//     ../statespace's own plugin/dsp/SpectrumAnalyzer.hpp uses to
//     transport bulk per-frame data to the UI side through DPF's one
//     guaranteed-portable channel, ordinary output Parameters).
//   - HeatmapGrid: a long-history (1-30s) density map of the same points
//     GoniometerBuffer's own trail draws live, accumulated into a small
//     fixed-size grid rather than transported as individual points -- see
//     its own comment for why, and for the packing scheme (the same
//     24-bit budget, just 4 lower-precision cells per Parameter instead
//     of GoniometerBuffer's 2 higher-precision coordinates).

#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace ssmeter {

// ---- dB <-> linear amplitude ----------------------------------------------

inline double linearToDb(double linear)
{
    return 20.0 * std::log10(std::max(1.0e-12, linear));
}

// Fraction of the goniometer plot's own unity-circle radius "Auto Gain"
// aims its own smoothed level reading at -- shared between the DSP
// side's own SmoothedLevelMeter-driven readout and the UI side's own
// Gain computation from it (see UIPhaseScope's own Auto Gain button
// comment). Was `0.9` initially, halved to `0.45` for a better-filled
// plot; once the level reading switched from an instant peak to a
// smoothed average (see kAutoGainTimeConstantSeconds below), that same
// `0.45` started aiming the *average* at the old peak-tuned target,
// landing about 8dB hotter than before on real material. An 8dB reduction in this
// constant produces exactly an 8dB reduction in the resulting Gain for
// any signal (`gainDb = targetDb - levelDb`, so shifting `targetDb` by a
// fixed amount shifts `gainDb` by that same fixed amount, independent of
// the signal), which is the whole reason this is tuned as one linear
// constant rather than, say, a dB value some other formula derives from.
constexpr double kAutoGainTargetPeak = 0.18;

// How long SmoothedLevelMeter's own one-pole average takes to settle,
// for the specific instance driving Auto Gain. Raised from an original
// `1.0` (itself replacing an instant peak reading) for smoother tracking
// -- if it still reacts too quickly for a given piece of material,
// this is the constant to raise further still.
constexpr double kAutoGainTimeConstantSeconds = 2.5;

inline double dbToLinear(double db)
{
    return std::pow(10.0, db * 0.05);
}

// ---- RMS meter: fixed ~300ms one-pole mean-square estimate -----------------
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

// ---- peak meter: instant peak + slow decay ---------------------------------
class PeakMeter
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        // ~20dB/s fall, a conventional peak-meter ballistic.
        decayPerSample_ = std::pow(10.0, (-20.0 / 20.0) / sampleRate_);
    }

    inline void process(double absX)
    {
        if (absX > peak_) peak_ = absX;
        else peak_ *= decayPerSample_;
    }

    double peakDb() const { return linearToDb(peak_); }
    void reset() { peak_ = 0.0; }

private:
    double sampleRate_ = 48000.0;
    double decayPerSample_ = 0.9999;
    double peak_ = 0.0;
};

// ---- smoothed level follower: a slow "average" reading for Auto Gain -----
//
// Auto Gain (see UIPhaseScope.cpp's own comment) continuously re-applies
// Gain from whatever this reads, on every update -- reusing PeakMeter's
// own instant attack for that made every continuous correction visibly
// twitchy/nervous, snapping straight to each new transient and then
// sliding back down as it decayed. A one-pole low-pass average of the plain magnitude (not
// its square/RMS -- Auto Gain cares about level, not power), the same
// "average over time" idea LoudnessMeter's own RMS integration already
// uses elsewhere in this file, just on the plain signal rather than its
// square, and with a much longer, caller-supplied time constant instead
// of LoudnessMeter's fixed 300ms.
class SmoothedLevelMeter
{
public:
    void prepare(double sampleRate, double timeConstantSeconds)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        coeff_ = std::exp(-1.0 / (std::max(0.001, timeConstantSeconds) * sampleRate_));
    }

    inline void process(double absX) { average_ = coeff_ * average_ + (1.0 - coeff_) * absX; }

    double levelDb() const { return linearToDb(average_); }
    void reset() { average_ = 0.0; }

private:
    double sampleRate_ = 48000.0;
    double coeff_ = 0.0;
    double average_ = 0.0;
};


// ---- stereo image: correlation + width from one shared set of sums --------
//
// The standard phase-correlation-meter technique -- the same one a
// hardware correlation meter's analog multiplier-plus-smoothing-capacitor
// implements, and the one every software correlation meter built since
// converged on -- is a leaky-integrated (one-pole smoothed) Pearson
// correlation coefficient: accumulate E[L*R], E[L^2], E[R^2] with the same
// one-pole smoothing LoudnessMeter already uses (~300ms, a conventional
// correlation-meter integration time -- fast enough to react to a section
// of the mix, slow enough not to flicker every sample), then
// correlation = E[L*R] / sqrt(E[L^2] * E[R^2]), which is exactly [-1, +1]
// by the Cauchy-Schwarz inequality: +1 for identical (mono-compatible)
// signals, 0 for uncorrelated content, -1 for exactly out-of-phase content
// that will cancel to silence in mono.
//
// Stereo width reuses the *same three sums* rather than tracking Mid/Side
// power separately with its own pair of meters: Mid = (L+R)/2 and
// Side = (L-R)/2, so (by expanding the square) E[Mid^2] =
// (E[L^2] + 2E[LR] + E[R^2])/4 and E[Side^2] = (E[L^2] - 2E[LR] + E[R^2])/4
// -- both already fully determined by the same E[LR]/E[LL]/E[RR] this class
// tracks for correlation anyway. widthPercent() reports
// 200 * sqrt(E[Side^2]) / (sqrt(E[Mid^2]) + sqrt(E[Side^2])): 0% for a
// perfectly mono signal (no Side energy at all), 100% for an ordinary
// uncorrelated stereo signal with equal Mid/Side energy, up to 200% for a
// signal that is *entirely* Side (e.g. L and R exactly out of phase) --
// the same 0-200%, 100%-is-normal scale most dedicated stereo-width meters
// use.
class StereoImageMeter
{
public:
    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        coeff_ = std::exp(-1.0 / (0.001 * kIntegrationMs * sampleRate_));
    }

    inline void process(double l, double r)
    {
        sumLL_ = coeff_ * sumLL_ + (1.0 - coeff_) * (l * l);
        sumRR_ = coeff_ * sumRR_ + (1.0 - coeff_) * (r * r);
        sumLR_ = coeff_ * sumLR_ + (1.0 - coeff_) * (l * r);

        const double corr = correlation();
        // A slowly-recovering hold of the *worst* (most negative)
        // correlation seen recently -- a transient phase problem is
        // exactly the kind of thing a plain smoothed reading can hide
        // (it comes and goes faster than ~300ms), so this mirrors
        // LevelMeterWidget's own peak-hold-with-decay idea, applied to
        // "how bad has it been lately" instead of "how loud".
        //
        // A genuine *plateau* before recovery starts, not just a slow
        // recovery rate: without one, the worst-value marker just follows
        // the end of the live correlation bar -- ordinary program material's correlation reading wobbles
        // continuously even after the ~300ms smoothing above (it's still a
        // real, moving signal, not a static test tone), so without a
        // plateau, *any* one-sample dip below the still-recovering hold
        // immediately snaps the hold right back down onto that dip,
        // reproducing the live value on every such wobble -- which for
        // real audio is often, defeating the entire point of a separate
        // "how bad has it recently been" marker. Every new worst value
        // now restarts a fixed hold time (kHoldPlateauSeconds, matching
        // LevelMeterWidget's own 1.5s clip-hold convention) during which
        // the marker doesn't move at all even if the signal keeps
        // improving; only once that plateau fully elapses without an even
        // worse dip does it start climbing back at the same rate as
        // before. This is the conventional two-stage "peak hold, then
        // decay" shape real peak meters use -- the previous single-stage
        // version was really just "instant peak, continuous slow decay",
        // never actually holding anything flat long enough to be seen as
        // separate from the live reading.
        if (corr < minHold_)
        {
            minHold_ = corr;
            holdTimer_ = kHoldPlateauSeconds;
        }
        else if (holdTimer_ > 0.0)
        {
            holdTimer_ -= 1.0 / sampleRate_;
        }
        else
        {
            minHold_ = std::min(1.0, minHold_ + kHoldRecoverPerSample_ / sampleRate_);
        }
    }

    // Cauchy-Schwarz guarantees this is in [-1, +1] except for the
    // near-silence denominator floor below, which only ever pulls a
    // genuinely tiny (inaudible) signal's reading toward 0 -- a
    // deliberate, honest "nothing meaningful to correlate" answer rather
    // than an undefined 0/0 or a noise-amplified value.
    double correlation() const
    {
        const double denom = std::sqrt(std::max(1.0e-12, sumLL_ * sumRR_));
        return std::max(-1.0, std::min(1.0, sumLR_ / denom));
    }

    double minHoldCorrelation() const { return minHold_; }

    double widthPercent() const
    {
        const double midPow = (sumLL_ + 2.0 * sumLR_ + sumRR_) * 0.25;
        const double sidePow = (sumLL_ - 2.0 * sumLR_ + sumRR_) * 0.25;
        const double midMag = std::sqrt(std::max(0.0, midPow));
        const double sideMag = std::sqrt(std::max(0.0, sidePow));
        const double denom = midMag + sideMag;
        if (denom < 1.0e-9) return 0.0; // silence: report mono rather than an undefined ratio
        return 200.0 * sideMag / denom;
    }

    void reset()
    {
        sumLL_ = sumRR_ = sumLR_ = 0.0;
        minHold_ = 1.0;
        holdTimer_ = 0.0;
    }

private:
    static constexpr double kIntegrationMs = 300.0;
    static constexpr double kHoldRecoverPerSample_ = 0.5; // correlation-units/second the min-hold climbs back per second, once its plateau has elapsed
    static constexpr double kHoldPlateauSeconds = 1.5; // how long a new worst value sits flat before recovery starts -- matches LevelMeterWidget's own clip-hold time
    double sampleRate_ = 48000.0;
    double coeff_ = 0.0;
    double sumLL_ = 0.0, sumRR_ = 0.0, sumLR_ = 0.0;
    double minHold_ = 1.0;
    double holdTimer_ = 0.0; // seconds remaining in the current plateau, 0 once expired
};

// ---- goniometer scope buffer: a decimated, packed ring of Mid/Side points --
//
// A real vectorscope's display is a continuous phosphor trail, not a
// discrete "one frame per screen refresh" snapshot -- the classic
// goniometer look this reproduces comes from a rotated Mid/Side plot
// (Mid straight up so a mono signal draws a vertical line, hard-left/
// hard-right draw the two bottom diagonals) with points fading out over
// roughly the persistence time, not vanishing instantly.
//
// **Write rate is fixed, independent of the "Decay" setting.** An
// earlier version of this class instead kept the buffer's own *point
// count* fixed and stretched/shrank decimationStride_ to make that same
// count of points span a shorter or longer requested duration -- which
// technically worked, but meant a longer Decay setting made *exactly the
// same number* of points cover more time, i.e. a sparser trail, not a
// longer *and* equally dense one. Now decimationStride_ is derived once, in prepare(), purely from
// the real sample rate and a fixed target density (kScopeBufferSize
// points spread over kMaxDecaySeconds) -- process() always writes at that
// same rate no matter what Decay is currently set to. The ring buffer is
// sized to hold a full kMaxDecaySeconds' worth of points at that density;
// "Decay" (visibleCountForDecay(), below) purely controls how many of the
// *most recent* buffered points the UI actually draws, from 1 up to the
// full kScopeBufferSize -- a shorter Decay reveals a shorter prefix of an
// otherwise-identical, constantly-refilling trail, a longer one reveals
// more of it, and the trail is exactly as dense (points per second)
// regardless of which is chosen.
//
// Bulk transport to the UI reuses the exact packing idea
// ../statespace's own SpectrumAnalyzer.hpp uses for its FFT bands: DPF's
// only guaranteed cross-process channel is a plain float Parameter (the
// DSP and UI sides can be different processes for some plugin formats/
// hosts), so kScopeBufferSize points would otherwise cost kScopeBufferSize
// separate output Parameters. Instead, one (x, y) point is quantized to
// kBitsPerCoord bits per axis and packed into a single float via plain
// integer arithmetic (not a bit-reinterpretation cast) -- 2 * 12 = 24
// bits, deliberately capped there because an IEEE-754 float's mantissa
// represents every integer up to 2^24 exactly, so the packed value
// round-trips with zero error beyond the quantization step itself and
// stays an ordinary, host-safe float value throughout.
class GoniometerBuffer
{
public:
    // kScopeBufferSize is sized so that, at kMaxDecaySeconds worth of
    // Decay, the trail is still just as dense (points/second, see
    // recomputeStride()) as it is at any shorter Decay -- there is no
    // separate "how many points exist" knob, only "how much of the fixed,
    // constantly-written trail is currently shown" (visibleCountForDecay()).
    // 4096 (was 2048, doubled for a denser trail) gives ~1365 points/second at kMaxDecaySeconds=3s
    // (a ~0.7ms gap between points, was ~1.5ms). This is the DSP-side
    // *capacity*, not what actually gets drawn every frame -- see
    // GoniometerWidget's own setResolutionPercent()/kFieldResolution for
    // the independent, UI-only knob that lets a machine with less GPU/CPU
    // headroom draw fewer of these points per frame without touching this
    // constant or the buffer's own real density at all.
    static constexpr int kScopeBufferSize = 4096;
    static constexpr double kDefaultDecaySeconds = 0.6; // raised from an original 0.35 -- a better default persistence
    static constexpr double kMinDecaySeconds = 0.1;
    static constexpr double kMaxDecaySeconds = 3.0;

    static constexpr int kBitsPerCoord = 12;
    static constexpr int kQuantLevels = (1 << kBitsPerCoord) - 1; // 4095
    static constexpr float kCoordMax = 1.2f; // a little past +-1 so a hot signal doesn't clip off the edge of the plot

    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        // Fixed target density: kScopeBufferSize points per
        // kMaxDecaySeconds, in real samples at this sample rate. Never
        // recomputed except on a sample-rate change -- see this class's
        // own file comment for why Decay itself no longer touches this.
        decimationStride_ = std::max(1, int(std::lround(sampleRate_ * kMaxDecaySeconds / double(kScopeBufferSize))));
        sinceLastWrite_ = 0;
    }

    // How many of the most recently written points (out of the fixed
    // kScopeBufferSize the buffer always holds) should actually be drawn
    // for a given requested Decay, in milliseconds -- a pure function of
    // the compile-time density, not of anything this instance is
    // currently doing. UI-side only (GoniometerWidget calls this), but
    // lives here since it's the exact inverse of the density prepare()
    // establishes.
    static int visibleCountForDecay(double decayMs) noexcept
    {
        const double seconds = std::max(kMinDecaySeconds, std::min(kMaxDecaySeconds, decayMs * 0.001));
        const int count = int(std::lround(seconds / kMaxDecaySeconds * double(kScopeBufferSize)));
        return std::max(1, std::min(kScopeBufferSize, count));
    }

    // RT-safe: called once per sample from run(). x/y are whatever the
    // caller wants plotted -- PluginPhaseScope feeds rotated Mid/Side
    // (Side on x, Mid on y) so a mono signal draws straight up. Returns
    // whether this call actually wrote a new point (i.e. the decimation
    // gate let it through) -- PluginPhaseScope uses this to feed
    // HeatmapGrid exactly the same decimated points the visible trail
    // itself gets, at the identical fixed rate, rather than duplicating
    // this class's own decimation logic a second time.
    inline bool process(float x, float y) noexcept
    {
        if (++sinceLastWrite_ < decimationStride_) return false;
        sinceLastWrite_ = 0;

        const int idx = writeIndex_.load(std::memory_order_relaxed) % kScopeBufferSize;
        points_[idx].store(packPoint(x, y), std::memory_order_relaxed);
        writeIndex_.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    // Message-thread-only, mirroring SpectrumAnalyzer::getPackedTriple()'s
    // own calling convention: read via PluginPhaseScope::getParameterValue().
    float getPackedPoint(int slot) const noexcept
    {
        if (slot < 0 || slot >= kScopeBufferSize) return 0.0f;
        return points_[size_t(slot)].load(std::memory_order_relaxed);
    }

    // How many points have ever been written, mod nothing -- the UI reads
    // this (via its own output Parameter) to know which ring-buffer slot
    // is "newest" and derive each point's fade-age from there.
    uint32_t getWriteIndex() const noexcept { return writeIndex_.load(std::memory_order_relaxed); }

    static float packPoint(float x, float y) noexcept
    {
        const auto quantize = [](float v) -> uint32_t {
            const float clamped = std::max(-kCoordMax, std::min(kCoordMax, v));
            const float t = (clamped + kCoordMax) / (2.0f * kCoordMax);
            return uint32_t(t * float(kQuantLevels) + 0.5f);
        };
        const uint32_t qx = quantize(x);
        const uint32_t qy = quantize(y);
        const uint32_t combined = qx | (qy << kBitsPerCoord);
        return float(combined);
    }

    // The exact inverse of packPoint() -- called on the UI side once per
    // received packed Parameter. Rounds via std::lround rather than the
    // more obvious "+0.5f then truncate": packPoint()'s combined value can
    // land anywhere in [0, 2^24), and for values at or above 2^23 a
    // float's representable integers are already exactly 1 apart (its
    // 24-bit mantissa is entirely spent on the integer part at that
    // magnitude) -- adding a further +0.5 there can itself round to the
    // *next* integer (round-half-to-even) before the truncation ever
    // happens, silently corrupting the packed value by 1 at exactly the
    // worst possible bit position (e.g. 0x800FFF -> 0x801000, flipping
    // every bit of both 12-bit fields at once). std::lround rounds the
    // float's actual value correctly regardless of magnitude, with no
    // such intermediate-representability trap.
    static void unpackPoint(float packed, float& outX, float& outY) noexcept
    {
        const uint32_t combined = uint32_t(std::lround(packed));
        const uint32_t qx = combined & uint32_t(kQuantLevels);
        const uint32_t qy = (combined >> kBitsPerCoord) & uint32_t(kQuantLevels);
        outX = (float(qx) / float(kQuantLevels)) * (2.0f * kCoordMax) - kCoordMax;
        outY = (float(qy) / float(kQuantLevels)) * (2.0f * kCoordMax) - kCoordMax;
    }

    void reset()
    {
        for (auto& p : points_) p.store(0.0f, std::memory_order_relaxed);
        writeIndex_.store(0, std::memory_order_relaxed);
        sinceLastWrite_ = 0;
    }

private:
    double sampleRate_ = 48000.0;
    int decimationStride_ = 70; // recomputed in prepare(); the 48kHz value of the formula above
    int sinceLastWrite_ = 0;

    std::atomic<float> points_[kScopeBufferSize] = {};
    std::atomic<uint32_t> writeIndex_{0};
};

// ---- heatmap: a long-history density map of the same points the trail
// draws live -----------------------------------------------------------
//
// A slow-fading "glow" of where the trace has been, over 1 to 30 seconds.
// GoniometerBuffer's own ring can't do this by itself: it holds a
// fixed kScopeBufferSize points at a fixed density (~1.4kHz), so even its
// full kMaxDecaySeconds (3s) is nowhere near the 30s of history wanted
// here, and transporting 30s of *individual* points at that density
// (~41,000 of them) as separate Parameters the way GoniometerBuffer
// itself does would be a wildly impractical Parameter count. A heatmap
// doesn't need individual points anyway -- it needs a compact *density*
// summary, so this accumulates hits into a small, fixed-size 2D grid
// instead, each cell independently leaky-integrating (the same one-pole
// "add on a hit, decay every tick" shape `StereoImageMeter`'s own sums
// use, just per spatial cell instead of per statistic) toward a
// requested Decay of 1-30 seconds.
//
// Fed exactly the same decimated points GoniometerBuffer's own ring
// writes (see GoniometerBuffer::process()'s own bool return and
// PluginPhaseScope::run()) -- not independently re-decimated -- so the
// heatmap is honestly "the same live trail, remembered longer", not a
// separately-sampled approximation of it. Because that feed rate is
// itself fixed (kScopeBufferSize points per kMaxDecaySeconds, see
// GoniometerBuffer's own file comment) and does not depend on the sample
// rate, this class's own per-point decay coefficient doesn't either --
// no prepare(sampleRate) needed, just prepare() once to pick up the
// initially-requested Decay.
//
// Bulk transport reuses the same packing idea as GoniometerBuffer's own
// points, just with more values per Parameter: a cell only needs
// visual-brightness precision, not the vectorscope's own positional
// precision, so 4 cells at 6 bits each (0-63 levels) fit the same 24-bit
// exact-integer-mantissa budget instead of GoniometerBuffer's 2 values at
// 12 bits each -- kGridSize*kGridSize/4 Parameters (256 at the current
// 32x32 sizing) rather than one per cell.
class HeatmapGrid
{
public:
    static constexpr int kGridSize = 32;
    static constexpr int kCellCount = kGridSize * kGridSize; // 1024

    static constexpr int kBitsPerCell = 6;
    static constexpr int kCellQuantLevels = (1 << kBitsPerCell) - 1; // 63
    static constexpr int kCellsPerPacket = 4; // 4 * 6 = 24 bits
    static constexpr int kNumPackets = kCellCount / kCellsPerPacket; // 256 -- kCellCount divides evenly

    static constexpr double kMinDecaySeconds = 1.0;
    static constexpr double kMaxDecaySeconds = 30.0;
    static constexpr double kDefaultDecaySeconds = 5.0;

    // The fixed rate addPoint() is actually called at -- see this class's
    // own file comment: identical to GoniometerBuffer's own write rate,
    // and just as independent of the real sample rate.
    static constexpr double kPointRateHz = double(GoniometerBuffer::kScopeBufferSize) / GoniometerBuffer::kMaxDecaySeconds;

    void prepare()
    {
        activeDecaySeconds_ = requestedDecaySeconds_.load(std::memory_order_relaxed);
        recomputeDecayPerPoint();
    }

    // Clamped to [kMinDecaySeconds, kMaxDecaySeconds]. Safe to call from
    // any thread; applied from inside addPoint() at the next call, same
    // "check where a mutation was about to happen anyway" convention
    // GoniometerBuffer's own (now-removed) decay-request handling used.
    void requestDecaySeconds(double seconds) noexcept
    {
        requestedDecaySeconds_.store(std::max(kMinDecaySeconds, std::min(kMaxDecaySeconds, seconds)), std::memory_order_relaxed);
    }

    // x, y: the same rotated Mid/Side coordinates GoniometerBuffer itself
    // plots, in [-GoniometerBuffer::kCoordMax, +kCoordMax]. Call exactly
    // once per point GoniometerBuffer::process() itself just wrote (see
    // that function's own bool return) -- never independently decimated.
    inline void addPoint(float x, float y) noexcept
    {
        const double requested = requestedDecaySeconds_.load(std::memory_order_relaxed);
        if (requested != activeDecaySeconds_)
        {
            activeDecaySeconds_ = requested;
            recomputeDecayPerPoint();
        }

        for (float& c : cells_) c *= decayPerPoint_;

        const int cx = cellIndex(x);
        const int cy = cellIndex(y);
        float& cell = cells_[size_t(cy) * size_t(kGridSize) + size_t(cx)];
        cell = std::min(1.0f, cell + kAddPerHit);
    }

    // Message-thread-only, mirroring GoniometerBuffer::getPackedPoint()'s
    // own calling convention: read via PluginPhaseScope::getParameterValue().
    float getPackedQuad(int packetIndex) const noexcept
    {
        if (packetIndex < 0 || packetIndex >= kNumPackets) return 0.0f;
        uint32_t combined = 0;
        for (int k = 0; k < kCellsPerPacket; ++k)
            combined |= quantize(cells_[size_t(packetIndex) * size_t(kCellsPerPacket) + size_t(k)]) << (kBitsPerCell * k);
        return float(combined);
    }

    // The exact inverse of getPackedQuad() -- called on the UI side once
    // per received packed Parameter. std::lround, not "+0.5f then
    // truncate" -- see GoniometerBuffer::unpackPoint()'s own comment for
    // the rounding trap that convention avoids.
    static void unpackQuad(float packed, float (&outCells)[kCellsPerPacket]) noexcept
    {
        const uint32_t combined = uint32_t(std::lround(packed));
        for (int k = 0; k < kCellsPerPacket; ++k)
        {
            const uint32_t q = (combined >> (kBitsPerCell * k)) & uint32_t(kCellQuantLevels);
            outCells[k] = float(q) / float(kCellQuantLevels);
        }
    }

    void reset()
    {
        for (float& c : cells_) c = 0.0f;
    }

    // Remaps every existing cell's own accumulated intensity to where it
    // belongs after the *plotted* coordinate space itself has been
    // rescaled by `ratio` (newGainLinear / oldGainLinear) -- called
    // whenever Gain changes, since both GoniometerBuffer's live trail and
    // this grid are addressed in that same Gain-scaled space (see
    // PluginPhaseScope.cpp's own kParamDisplayGain case). A plain
    // `reset()` on every Gain change was the original way of keeping the
    // heatmap aligned with the plot, but with Auto Gain now continuously
    // nudging Gain, that meant the heatmap could never actually accumulate
    // any visible history at all (spots vanished instead of fading out).
    // This resamples instead: each cell's
    // own center world coordinate is scaled by `ratio` and re-deposited
    // into whichever cell it now falls into (summed, since more than one
    // old cell can land in the same new one when zooming out, and
    // dropped entirely if scaled outside the plot when zooming in) --
    // history is preserved and repositioned instead of discarded, and
    // still fades out at its own requested Decay rate exactly as before.
    // Nearest-cell, not interpolated -- a big, sudden Gain jump can look
    // slightly blockier for a moment, but the alternative (an unbounded
    // reset on every continuous Auto Gain nudge) is visibly worse.
    void rescale(float ratio) noexcept
    {
        if (std::fabs(ratio - 1.0f) < 1.0e-6f) return;

        constexpr float kWorldExtent = GoniometerBuffer::kCoordMax;
        constexpr float kCellWorldSize = (2.0f * kWorldExtent) / float(kGridSize);

        float remapped[kCellCount] = {};
        for (int gy = 0; gy < kGridSize; ++gy)
        {
            for (int gx = 0; gx < kGridSize; ++gx)
            {
                const float v = cells_[size_t(gy) * size_t(kGridSize) + size_t(gx)];
                if (v < 0.003f) continue; // skip empty cells -- not worth remapping

                const float worldX = -kWorldExtent + (float(gx) + 0.5f) * kCellWorldSize;
                const float worldY = -kWorldExtent + (float(gy) + 0.5f) * kCellWorldSize;
                const float newX = worldX * ratio;
                const float newY = worldY * ratio;
                if (std::fabs(newX) > kWorldExtent || std::fabs(newY) > kWorldExtent)
                    continue; // scaled outside the plot entirely -- drop it, rather than pile it up at the edge

                const int ncx = cellIndex(newX);
                const int ncy = cellIndex(newY);
                float& cell = remapped[size_t(ncy) * size_t(kGridSize) + size_t(ncx)];
                cell = std::min(1.0f, cell + v);
            }
        }
        for (int i = 0; i < kCellCount; ++i) cells_[size_t(i)] = remapped[size_t(i)];
    }

private:
    static int cellIndex(float v) noexcept
    {
        const float clamped = std::max(-GoniometerBuffer::kCoordMax, std::min(GoniometerBuffer::kCoordMax, v));
        const float t = (clamped + GoniometerBuffer::kCoordMax) / (2.0f * GoniometerBuffer::kCoordMax);
        return std::max(0, std::min(kGridSize - 1, int(t * float(kGridSize))));
    }

    void recomputeDecayPerPoint() noexcept
    {
        // One-pole decay per *point* (not per sample -- addPoint() is
        // already called at the fixed kPointRateHz, see this class's own
        // file comment), so a cell that stops being hit fades to
        // (1/e) of its value after activeDecaySeconds_ of wall-clock time
        // regardless of sample rate.
        decayPerPoint_ = float(std::exp(-1.0 / (activeDecaySeconds_ * kPointRateHz)));
    }

    static uint32_t quantize(float v) noexcept
    {
        const float clamped = std::max(0.0f, std::min(1.0f, v));
        return uint32_t(clamped * float(kCellQuantLevels) + 0.5f);
    }

    static constexpr float kAddPerHit = 0.35f; // how much a single visit brightens a cell before decay -- tuned so a handful of nearby hits saturate, not just one passing point

    double activeDecaySeconds_ = kDefaultDecaySeconds;
    std::atomic<double> requestedDecaySeconds_{kDefaultDecaySeconds};
    float decayPerPoint_ = 0.0f;
    float cells_[kCellCount] = {};
};

} // namespace ssmeter
