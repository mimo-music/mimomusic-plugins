// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "SpectrumAnalyzer.hpp"

#include <algorithm>
#include <cmath>

namespace ssfilter {

namespace {

// Standard iterative in-place radix-2 decimation-in-time FFT (Cooley-
// Tukey), `n` a power of two. Textbook bit-reversal permutation followed
// by log2(n) butterfly passes -- no allocation, no recursion, safe to
// call from the audio thread as long as `re`/`im` are pre-sized caller-
// owned buffers (they are: SpectrumAnalyzer::fftRe_/fftIm_ members).
void fft(double* re, double* im, int n) noexcept
{
    for (int i = 1, j = 0; i < n; ++i)
    {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { std::swap(re[i], re[j]); std::swap(im[i], im[j]); }
    }

    for (int len = 2; len <= n; len <<= 1)
    {
        const double ang = -2.0 * M_PI / double(len);
        const double wRe = std::cos(ang), wIm = std::sin(ang);
        for (int i = 0; i < n; i += len)
        {
            double curRe = 1.0, curIm = 0.0;
            for (int k = 0; k < len / 2; ++k)
            {
                const double uRe = re[i + k], uIm = im[i + k];
                const double vRe = re[i + k + len / 2] * curRe - im[i + k + len / 2] * curIm;
                const double vIm = re[i + k + len / 2] * curIm + im[i + k + len / 2] * curRe;
                re[i + k] = uRe + vRe;
                im[i + k] = uIm + vIm;
                re[i + k + len / 2] = uRe - vRe;
                im[i + k + len / 2] = uIm - vIm;
                const double nextRe = curRe * wRe - curIm * wIm;
                const double nextIm = curRe * wIm + curIm * wRe;
                curRe = nextRe;
                curIm = nextIm;
            }
        }
    }
}

} // namespace

void SpectrumAnalyzer::requestFftSizeIndex(int idx) noexcept
{
    if (idx < 0) idx = 0;
    else if (idx >= kNumSizePresets) idx = kNumSizePresets - 1;
    requestedSizeIndex_.store(idx, std::memory_order_release);
}

void SpectrumAnalyzer::prepare(double sampleRate) noexcept
{
    sampleRate_ = sampleRate > 0.0 ? sampleRate : 48000.0;
    // Apply whichever size is *requested*, not just whatever was already
    // active -- prepare() re-deriving sampleRate_-dependent tables (the
    // window, the band-edge bins) at the previous size would otherwise
    // silently discard a requestFftSizeIndex() call made shortly before
    // prepare() runs (e.g. a resolution change racing a host sample-rate
    // change), leaving the two callers permanently disagreeing about
    // which size is active until the next explicit request.
    const int idx = requestedSizeIndex_.load(std::memory_order_acquire);
    applySizeIndex(idx);
    activeSizeIndex_.store(idx, std::memory_order_relaxed);
    for (auto& a : bands_) a.store(kFloorDb, std::memory_order_relaxed);
}

void SpectrumAnalyzer::applySizeIndex(int idx) noexcept
{
    idx = std::max(0, std::min(kNumSizePresets - 1, idx));
    fftSize_ = kFftSizePresets[idx];
    hopSize_ = fftSize_ / kHopDivisor;

    for (int i = 0; i < fftSize_; ++i) ring_[i] = 0.0f;
    writePos_ = 0;
    samplesSinceHop_ = 0;

    // Hann window (over the *active* fftSize_ only), and the sum of its
    // *squares* -- Parseval's theorem relates total FFT bin power to
    // sum(windowed_signal^2), which for a sinusoid of amplitude A
    // averages to A^2/2 * sum(window^2), not sum(window) (an earlier
    // version of this used sum(window), which under-read a summed-
    // multi-bin band's level by several dB -- see runFft()'s
    // normalization comment for the corrected derivation).
    windowPowerSum_ = 0.0;
    for (int i = 0; i < fftSize_; ++i)
    {
        const double w = 0.5 - 0.5 * std::cos(2.0 * M_PI * double(i) / double(fftSize_ - 1));
        window_[i] = w;
        windowPowerSum_ += w * w;
    }
    if (windowPowerSum_ < 1e-9) windowPowerSum_ = 1.0;

    // Band edges: kNumBands log-spaced bands from kMinHz to kMaxHz, each
    // mapped to the FFT bins that fall inside it. At the low end an FFT's
    // sampleRate_/fftSize_-per-bin resolution can be coarser than the
    // band itself (routine at the smallest preset, rarer -- but not
    // eliminated -- at the largest), so the band's *own* edges can span
    // zero or one bin -- but a Hann window's mainlobe is intrinsically
    // ~4 bins wide regardless, so a tone sitting near (but not exactly
    // on) such a narrow band's single captured bin still leaks real
    // energy into that bin's immediate neighbors, energy a too-narrow
    // capture range would simply miss and under-read by several dB.
    // Widening every band's capture range to at least kMinBinSpan bins
    // (centered on the band, not just clamped to its own log-spaced
    // edges) fixes that without materially blurring the low end any
    // further than the FFT's own resolution already does. Trade-off
    // accepted knowingly: this makes a discrete low-frequency tone read
    // back essentially exactly (verified directly, see the analyzer's
    // own test harness used during development), at the cost of
    // slightly over-reading broadband/noise-like content in the same
    // narrow bands, whose true energy really is confined to a narrower
    // range than kMinBinSpan bins now captures. Tonal accuracy was
    // prioritized since spotting a peak/notch under the EQ curve -- this
    // analyzer's whole purpose -- matters far more than absolute
    // noise-floor calibration for a background visual reference, not a
    // metering tool.
    constexpr int kMinBinSpan = 5;
    for (int b = 0; b < kNumBands; ++b)
    {
        const double t0 = double(b) / double(kNumBands);
        const double t1 = double(b + 1) / double(kNumBands);
        const double lowHz = kMinHz * std::pow(kMaxHz / kMinHz, t0);
        const double highHz = kMinHz * std::pow(kMaxHz / kMinHz, t1);
        const double centerHz = std::sqrt(lowHz * highHz);

        const double binHz = sampleRate_ / double(fftSize_);
        const int maxBin = fftSize_ / 2;
        int lowBin = std::max(1, int(std::ceil(lowHz / binHz)));
        int highBin = std::min(maxBin, int(std::floor(highHz / binHz)));
        if (highBin < lowBin) highBin = lowBin;

        if (highBin - lowBin < kMinBinSpan)
        {
            const int centerBin = std::max(1, std::min(maxBin, int(std::lround(centerHz / binHz))));
            lowBin = std::max(1, centerBin - kMinBinSpan / 2);
            highBin = std::min(maxBin, lowBin + kMinBinSpan);
            lowBin = std::max(1, highBin - kMinBinSpan); // re-clamp if highBin hit maxBin
        }

        // A small overlap margin on *every* band (not just the narrow
        // ones kMinBinSpan already widens), added after the above: with
        // kNumBands increased and each band's own log-spaced edges hard-
        // partitioning the FFT's bins with zero overlap, a tone whose true
        // bin sits right at (or just past) a band's edge had its ~4-bin
        // Hann mainlobe split across that boundary -- neither neighbor's
        // exact-edges capture range held the whole thing, so the reported
        // peak under-read by as much as half the energy (~3dB). More
        // bands means more boundaries means this was more likely to bite
        // *some* real frequency, not a one-off -- caught directly by
        // dsp_regression test [11]'s 5kHz case regressing from ~0dB to
        // 2dB of error the moment kNumBands doubled. Padding every band's
        // capture by the mainlobe's own half-width on each side means
        // immediate neighbors now overlap by a full mainlobe width, so a
        // boundary-adjacent tone is captured close to completely by
        // *whichever* band is nearer it, not partially by both -- the
        // same "accept slightly more spread/over-read in exchange for not
        // losing real tonal energy" trade-off this file's own header
        // comment already made once for kMinBinSpan, applied consistently
        // rather than only where a band happened to already be narrow.
        constexpr int kBandOverlapBins = 2;
        lowBin = std::max(1, lowBin - kBandOverlapBins);
        highBin = std::min(maxBin, highBin + kBandOverlapBins);

        bandLowBin_[b] = lowBin;
        bandHighBin_[b] = highBin;
        bandNearestBin_[b] = std::max(1, std::min(maxBin, int(std::lround(centerHz / binHz))));
    }
}

void SpectrumAnalyzer::process(const float* mono, uint32_t numFrames) noexcept
{
    const int req = requestedSizeIndex_.load(std::memory_order_acquire);
    if (req != activeSizeIndex_.load(std::memory_order_relaxed))
    {
        // Reconfiguration happens here, on the audio thread, and only
        // here -- see the file comment for why that matters.
        applySizeIndex(req);
        activeSizeIndex_.store(req, std::memory_order_relaxed);
    }

    for (uint32_t i = 0; i < numFrames; ++i)
    {
        ring_[writePos_] = mono[i];
        writePos_ = (writePos_ + 1) % fftSize_;

        if (++samplesSinceHop_ >= hopSize_)
        {
            samplesSinceHop_ = 0;
            runFft();
        }
    }
}

void SpectrumAnalyzer::runFft() noexcept
{
    // Window the ring buffer's current content into the FFT work
    // buffers, oldest-sample-first (writePos_ is where the *next* write
    // will land, i.e. the oldest surviving sample).
    for (int i = 0; i < fftSize_; ++i)
    {
        const int idx = (writePos_ + i) % fftSize_;
        fftRe_[i] = double(ring_[idx]) * window_[i];
        fftIm_[i] = 0.0;
    }

    fft(fftRe_, fftIm_, fftSize_);

    // Amplitude normalization, derived from Parseval's theorem: for a
    // sinusoid of amplitude A, the *sum* of |X[k]|^2 across every FFT bin
    // that captures its (window-spread) energy approaches
    // A^2 * fftSize_ * windowPowerSum_ / 4 (the /4 rather than /2 because
    // half of a real signal's total bin power sits in the mirrored
    // negative-frequency bins this analyzer never reads). So summing
    // |X[k]|^2 -- not averaging -- across a band's bins and applying this
    // one global factor gives a calibrated combined-band reading
    // regardless of how many (or how few) bins that band spans, which
    // matters here since a band can be anywhere from one bin (the low
    // end, coarser than fftSize_'s resolution) to dozens (the high end).
    // Averaging instead of summing was tried first and under-read a
    // multi-bin band's level by several dB whenever a narrowband tone's
    // energy didn't fill the whole band -- exactly the routine case.
    const double normFactor = 2.0 / std::sqrt(double(fftSize_) * windowPowerSum_);

    for (int b = 0; b < kNumBands; ++b)
    {
        double powerSum;
        if (bandHighBin_[b] > bandLowBin_[b])
        {
            powerSum = 0.0;
            for (int k = bandLowBin_[b]; k < bandHighBin_[b]; ++k)
                powerSum += fftRe_[k] * fftRe_[k] + fftIm_[k] * fftIm_[k];
        }
        else
        {
            const int k = bandNearestBin_[b];
            powerSum = fftRe_[k] * fftRe_[k] + fftIm_[k] * fftIm_[k];
        }

        const double magLin = std::sqrt(powerSum) * normFactor;
        const float db = float(20.0 * std::log10(std::max(magLin, 1e-6)));
        bands_[size_t(b)].store(std::max(kFloorDb, db), std::memory_order_relaxed);
    }
}

} // namespace ssfilter
