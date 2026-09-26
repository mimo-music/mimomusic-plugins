// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// SpectrumAnalyzer.hpp
//
// A real-time-safe magnitude-spectrum estimator, used to draw a live
// spectrum "in the back" of the EQ graph (gui/eq/EqGraphWidget.cpp) --
// purely a visual reference layer, no effect on the audio path at all.
//
// Runs entirely inside the audio callback (process(), called from
// PluginEq::run()): accumulates incoming samples into a fixed-size ring
// buffer, and every hop boundary runs a windowed FFT and republishes
// kNumBands log-spaced magnitude readings (in dB) via lock-free
// std::atomic<float> stores -- no locks, no allocation, bounded work per
// call, same RT-safety rules as everywhere else in this project (see
// StateSpaceEngine.hpp's own header comment). The UI side reads
// getBandDb() indirectly: PluginEq exposes each band as a DPF *output*
// Parameter (the same portable mechanism `kParamFaultFlag` already uses,
// and the pattern DPF's own Meters example plugin uses for exactly this
// purpose) -- there is no direct shared-memory link between the DSP and
// UI sides in general (they can be different processes for some plugin
// formats/hosts), so parameters are the one guaranteed-portable channel.
//
// This deliberately is *not* metering-grade: the log-bin mapping is
// tuned for "a smooth, responsive background shape", not for precise
// level readings. Further, cheap exponential smoothing ("how fast/
// smooth") is applied on the *UI* side instead (see UIEq::uiIdle()), not
// here -- that keeps the smoothing time constant a pure UI preference,
// adjustable without any DSP-side parameter/state round-trip.
//
// **FFT size is a runtime-selectable preset** (kFftSizePresets), not a
// fixed constant: a bigger FFT gives finer low-frequency resolution
// (bin width = sampleRate/fftSize -- the reason a handful of low bands
// need the kMinBinSpan widening in the .cpp at *any* size, just less of
// one at a bigger size) at the cost of more buffering latency (fftSize
// samples' worth -- ~341ms at 16384/48kHz) before a change in the signal
// shows up at all. The actual CPU cost barely moves either way (an
// FFT's O(n log n) cost divided by a hop that scales with n nets out to
// O(log n) per unit time -- 16384 vs 2048 is about 3x the log2, i.e.
// negligible in absolute terms on any modern CPU), so "resolution vs.
// CPU" is a bit of a misnomer for what this control actually trades off,
// but it's presented as a single "Spectrum Resolution" choice either way
// since latency is the trade a user actually perceives (a bigger,
// finer-grained display that visibly lags the audio a bit more).
//
// **The resolution improvement is inherently concentrated at low
// frequencies** -- this is expected physics, not a bug some future fix
// should spread more evenly. An FFT's bin spacing is *linear*
// (sampleRate/fftSize Hz per bin, the same everywhere in the spectrum),
// while the kNumBands displayed bands are *log*-spaced (roughly equal
// spacing per octave). A high display band (say, 10-20kHz) already spans
// hundreds or thousands of Hz -- many FFT bins fall inside it even at
// the smallest preset, so there's little room left to improve. A low
// display band (say, 20-40Hz) spans only a couple of Hz -- at the
// smallest preset it usually contains *zero* native bins (needing the
// kMinBinSpan widening below just to have anything to read at all), so
// that's exactly where a bigger FFT's finer bin spacing has real,
// visible room to help. Increasing kNumBands (more log-spaced display
// points) makes the *plotted curve* denser/smoother everywhere, which is
// a separate, worthwhile improvement on its own -- but it doesn't change
// *where* the FFT-size preset's own effect concentrates, for the same
// reason: no number of denser sample points changes the underlying
// bins' physical spacing.
//
// Changing the active size is requested via requestFftSizeIndex() --
// callable from any thread (it's just an atomic store) -- but actually
// *applied* only from inside process(), i.e. on the audio thread itself,
// the next time it runs. Reconfiguring the ring buffer/window/band
// tables in place from another thread while the audio thread might be
// mid-process() would be a data race otherwise; this keeps every actual
// mutation single-threaded (the audio thread's own), matching this
// project's other wait-free hand-off patterns (e.g.
// StateSpaceEngine::setModel()) rather than introducing a lock.

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ssfilter {

class SpectrumAnalyzer
{
public:
    static constexpr int kMaxFftSize = 16384; // largest preset; every buffer below is sized to this
    static constexpr int kHopDivisor = 4;     // hop = active fftSize / kHopDivisor -- 75% overlap at every size

    static constexpr int kNumSizePresets = 4;
    static constexpr int kFftSizePresets[kNumSizePresets] = { 2048, 4096, 8192, 16384 };
    static constexpr int kDefaultSizeIndex = 2; // 8192 -- see file comment for the resolution/latency trade-off

    // Log-spaced 20Hz-20kHz. 384 bands over ~10 octaves is ~38/octave --
    // doubled again from a previous 192 (~19/octave, itself doubled from
    // an original 64) for a denser, smoother-looking plotted curve. This
    // is genuinely more real display resolution, not a finer interpolation
    // of the same underlying data: runFft() below sums *actual* FFT bin
    // power within each band's own edges (widened to kMinBinSpan bins
    // where a band would otherwise be narrower than one bin), never
    // interpolates between bins, so more bands means each one captures a
    // narrower (and, at the low end where kMinBinSpan already dominates,
    // eventually the same) slice of *real* spectral data, not a smoother
    // curve drawn through the same 192 numbers. At the largest FFT preset
    // (16384 @ 48kHz, ~2.9Hz/bin) there are on the order of 6800 real bins
    // between 20Hz-20kHz, comfortably more than 384 -- see this file's
    // header comment for why the resulting improvement still concentrates
    // at low frequencies regardless of how many display points there are.
    // Each band is one more DPF output Parameter (see PluginEq.hpp) --
    // a real increase in per-plugin parameter count (768 across the two
    // input/output banks, up from 384), still comfortably within what
    // every format this project targets (LV2/VST3) supports.
    static constexpr int kNumBands = 384;
    static constexpr double kMinHz = 20.0, kMaxHz = 20000.0;
    static constexpr float kFloorDb = -100.0f;

    SpectrumAnalyzer() { prepare(48000.0); }

    // Non-RT: (re)computes the Hann window and band-edge tables for a new
    // sample rate at the currently-active FFT size, and clears all state.
    // Safe to call from the audio thread too (e.g. from activate()) since
    // it does no allocation, but is not itself lock-free against a
    // concurrent process() call -- same convention as
    // StateSpaceEngine::prepare(), call it before audio starts flowing,
    // not concurrently with it.
    void prepare(double sampleRate) noexcept;

    // Request a different FFT size preset (clamped to
    // [0, kNumSizePresets)). See file comment -- safe to call from any
    // thread, but only takes effect from inside the next process() call.
    void requestFftSizeIndex(int idx) noexcept;

    // The preset actually in effect right now (may lag a just-requested
    // change by up to one process() call).
    int getActiveFftSizeIndex() const noexcept { return activeSizeIndex_.load(std::memory_order_relaxed); }

    // RT-safe: feed `numFrames` new (already mono-summed) samples. Runs
    // an FFT and republishes the bands every hop-size samples; the rest
    // of the time this is just an O(numFrames) ring-buffer copy.
    void process(const float* mono, uint32_t numFrames) noexcept;

    // RT-safe (relaxed atomic load) -- called from the *message* thread
    // via PluginEq::getParameterValue(), never the audio thread.
    float getBandDb(int band) const noexcept
    {
        return (band >= 0 && band < kNumBands) ? bands_[size_t(band)].load(std::memory_order_relaxed) : kFloorDb;
    }

    // ---- Packed transport: kBandsPerPacket bands per Parameter ---------
    //
    // Drawing the spectrum needs no saved state beyond the axis settings
    // and resolution -- these are output Parameters, never written to a
    // saved session at all, and the actual user-facing settings (FFT
    // size/axis range/autoscale) are separate, tiny States elsewhere.
    // What this *does* still cost is sheer Parameter *count*: one plain
    // output Parameter per band (kNumBands = 384) per analyzer, four
    // analyzer banks per plugin (PluginEq.hpp's own comment), came to
    // 1536 of a given plugin's total declared Parameters -- suspected
    // (not yet proven beyond doubt) of
    // interacting badly with how at least one host restores an unusually
    // large plugin's other Parameters during a route/channel-strip
    // import specifically. This packs kBandsPerPacket consecutive bands'
    // worth of dB readings into a single Parameter instead of one each,
    // cutting the per-analyzer Parameter count from kNumBands to
    // kNumPackedBands (384 -> 128, a straight 3x reduction, 1536 -> 512
    // across all four banks) -- with zero change to what actually reaches
    // the screen: unpackTriple() (below) reconstructs the same
    // kBandsPerPacket dB values on the UI side, which then feed the
    // exact same per-band array/graph-drawing code as before.
    //
    // Packing scheme: each band is quantized to kBitsPerBand bits (256
    // levels across [kFloorDb, kPackedMaxDb], ~0.47dB steps) -- far finer
    // than anything visually distinguishable on this deliberately-
    // approximate background layer (see this file's own header comment:
    // "not metering-grade... a smooth, responsive background shape").
    // kBandsPerPacket * kBitsPerBand = 24 bits, packed via plain integer
    // arithmetic (not a bit-reinterpretation cast) into one float, and
    // deliberately capped at exactly 24 bits: an IEEE-754 float's mantissa
    // represents every integer up to 2^24 (16,777,216) exactly, so the
    // packed value round-trips with zero additional error beyond the
    // quantization step itself, and -- just as importantly -- stays a
    // perfectly ordinary, in-range float value a host can clamp/round
    // (e.g. for its own kParameterIsInteger handling) without ever
    // corrupting it, unlike a raw bit-pattern reinterpretation would risk.
    static constexpr float kPackedMaxDb = 24.0f; // matches the un-packed per-band Parameter's own historical ceiling
    static constexpr int kBitsPerBand = 8;
    static constexpr int kQuantLevels = (1 << kBitsPerBand) - 1; // 255
    static constexpr int kBandsPerPacket = 3;
    static constexpr int kNumPackedBands = kNumBands / kBandsPerPacket; // 128 -- kNumBands divides evenly
    static constexpr float kMaxPackedValue = float((1 << (kBitsPerBand * kBandsPerPacket)) - 1); // 16,777,215 -- the declared Parameter ceiling

    // Message-thread-only (same caller as getBandDb()): reads
    // kBandsPerPacket consecutive real bands starting at
    // packetIndex*kBandsPerPacket and returns them packed into one float.
    float getPackedTriple(int packetIndex) const noexcept
    {
        if (packetIndex < 0 || packetIndex >= kNumPackedBands) return 0.0f;
        uint32_t combined = 0;
        for (int k = 0; k < kBandsPerPacket; ++k)
            combined |= uint32_t(quantizeDb(getBandDb(packetIndex * kBandsPerPacket + k))) << (kBitsPerBand * k);
        return float(combined);
    }

    // The exact inverse of getPackedTriple() -- called on the UI side,
    // once per received packed Parameter, to recover kBandsPerPacket
    // ordinary dB values for the existing per-band display array.
    static void unpackTriple(float packed, float (&outDb)[kBandsPerPacket]) noexcept
    {
        const uint32_t combined = uint32_t(packed + 0.5f); // packed is always an exact integer; +0.5 only guards float noise from host-side transport
        for (int k = 0; k < kBandsPerPacket; ++k)
        {
            const uint32_t q = (combined >> (kBitsPerBand * k)) & uint32_t(kQuantLevels);
            outDb[k] = kFloorDb + (float(q) / float(kQuantLevels)) * (kPackedMaxDb - kFloorDb);
        }
    }

private:
    static int quantizeDb(float db) noexcept
    {
        const float clamped = db < kFloorDb ? kFloorDb : (db > kPackedMaxDb ? kPackedMaxDb : db);
        const float t = (clamped - kFloorDb) / (kPackedMaxDb - kFloorDb);
        return int(t * float(kQuantLevels) + 0.5f);
    }

    void applySizeIndex(int idx) noexcept; // audio-thread-only: recomputes window/band tables for a preset, clears the ring buffer
    void runFft() noexcept;

    double sampleRate_ = 48000.0;
    int fftSize_ = kFftSizePresets[kDefaultSizeIndex];
    int hopSize_ = kFftSizePresets[kDefaultSizeIndex] / kHopDivisor;

    std::atomic<int> requestedSizeIndex_{kDefaultSizeIndex};
    std::atomic<int> activeSizeIndex_{kDefaultSizeIndex};

    float ring_[kMaxFftSize] = {};
    int writePos_ = 0;
    int samplesSinceHop_ = 0;

    double window_[kMaxFftSize] = {};
    double windowPowerSum_ = 1.0; // sum(window[i]^2) over the *active* fftSize_, for magnitude normalization -- see .cpp

    // FFT work buffers -- preallocated members, never (re)allocated in
    // process()/runFft(), so this stays RT-safe regardless of how often
    // a hop boundary is hit or how often the size preset changes.
    double fftRe_[kMaxFftSize] = {};
    double fftIm_[kMaxFftSize] = {};

    // Precomputed once per applySizeIndex(): for each output band, either
    // the [lowBin,highBin) range of FFT bins to sum (power-summed), or,
    // when a band is narrower than one FFT bin's frequency resolution
    // (the routine case at the low end of the spectrum, less so at bigger
    // FFT sizes), the single nearest bin to sample directly instead of
    // averaging over an empty range.
    int bandLowBin_[kNumBands] = {};
    int bandHighBin_[kNumBands] = {}; // exclusive
    int bandNearestBin_[kNumBands] = {};

    std::atomic<float> bands_[kNumBands];
};

// Which analyzer(s) actually run and get displayed for the EQ graph's
// background spectrum: pre-filter (Input), post-filter (Output), Both, or
// neither (Off). Shared between the stereo and mono EQ plugins' DSP and
// UI sides (PluginEq/PluginEqMono, UIEq/UIEqMono, EqGraphWidget) so all
// of them agree on the same four states in the same cycling order.
//
// Unlike most of this plugin's other spectrum-display preferences (smoothing
// time constant, gain-axis range -- see PluginEq.hpp's StateIndex comment),
// this one is genuinely DSP-relevant: each plugin runs two independent
// SpectrumAnalyzer instances (one fed the pre-EQ signal, one the post-EQ
// signal), and only the one(s) actually selected here are fed at all in
// run() -- switching a side off genuinely saves DSP: an unfed analyzer's
// process() (the FFT/windowing cost) simply never
// runs, rather than running and having its result silently ignored.
enum class SpectrumMode : int { Off = 0, Input = 1, Output = 2, Both = 3 };

// Output-only, matching the old (pre-this-feature) default of always
// showing the post-EQ spectrum -- see PluginEq.hpp's initState() comment.
constexpr SpectrumMode kDefaultSpectrumMode = SpectrumMode::Output;

// Cycling order for the toolbar's "Spectrum Source" control: Off -> Input
// -> Output -> Both -> Off. Both DSP setState() (validating a value
// restored from session state) and UI click handlers go through this pair
// so there is exactly one place that defines "what the four states are"
// and "what comes after what".
inline SpectrumMode clampSpectrumMode(int v)
{
    return (v >= 0 && v <= 3) ? SpectrumMode(v) : kDefaultSpectrumMode;
}

inline SpectrumMode nextSpectrumMode(SpectrumMode m)
{
    switch (m)
    {
    case SpectrumMode::Off:    return SpectrumMode::Input;
    case SpectrumMode::Input:  return SpectrumMode::Output;
    case SpectrumMode::Output: return SpectrumMode::Both;
    case SpectrumMode::Both:   return SpectrumMode::Off;
    }
    return SpectrumMode::Off;
}

inline const char* spectrumModeLabel(SpectrumMode m)
{
    switch (m)
    {
    case SpectrumMode::Off:    return "Off";
    case SpectrumMode::Input:  return "In";
    case SpectrumMode::Output: return "Out";
    case SpectrumMode::Both:   return "Both";
    }
    return "?";
}

inline bool spectrumModeWantsInput(SpectrumMode m)  { return m == SpectrumMode::Input  || m == SpectrumMode::Both; }
inline bool spectrumModeWantsOutput(SpectrumMode m) { return m == SpectrumMode::Output || m == SpectrumMode::Both; }

} // namespace ssfilter
