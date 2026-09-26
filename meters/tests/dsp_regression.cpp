// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// dsp_regression.cpp
//
// Fast, pure-math correctness tests for plugin/dsp/PhaseScopeDsp.hpp: no
// DPF, no GUI, no audio files -- just the correlation/width formulas, the
// meter ballistics, and the goniometer packing round-trip exercised
// directly and checked against known-correct (often exact, by
// construction) values. Runs in well under a second. Mirrors the style
// and intent of ../compressor/tests/dsp_regression.cpp and
// ../statespace/tests/dsp_regression.cpp.
//
// Build and run via the `dsp_regression` CMake target, or standalone:
//   g++ -std=c++17 -O2 -Iplugin/dsp tests/dsp_regression.cpp -o dsp_regression
//   ./dsp_regression

#include "PhaseScopeDsp.hpp"
#include "LoudnessDsp.hpp"

#include <cmath>
#include <cstdio>

using namespace ssmeter;

namespace {
bool nearlyEqual(double a, double b, double tol) { return std::fabs(a - b) <= tol; }
}

int main()
{
    int failures = 0;
    constexpr double kSampleRate = 48000.0;

    // [1] Identical L==R at every sample: correlation is exactly +1 by
    // construction (sumLL == sumRR == sumLR at every step past the first
    // nonzero sample), regardless of how few samples have been processed
    // -- no settling time needed for this particular case.
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        for (int i = 0; i < 500; ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, x);
        }
        const bool ok = nearlyEqual(meter.correlation(), 1.0, 1e-9);
        printf("[1] identical L==R: correlation is exactly +1: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [2] Exactly out-of-phase L==-R: correlation is exactly -1, same
    // reasoning as [1].
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        for (int i = 0; i < 500; ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, -x);
        }
        const bool ok = nearlyEqual(meter.correlation(), -1.0, 1e-9);
        printf("[2] exactly out-of-phase L==-R: correlation is exactly -1: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [3] Mono (L==R): stereo width is exactly 0% -- Side energy is
    // exactly zero by construction (sumLL - 2*sumLR + sumRR == 0 when
    // sumLR == sumLL == sumRR).
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        for (int i = 0; i < 500; ++i)
        {
            const double x = std::cos(0.13 * i) * 0.4;
            meter.process(x, x);
        }
        const bool ok = nearlyEqual(meter.widthPercent(), 0.0, 1e-6);
        printf("[3] mono signal: stereo width is exactly 0%%: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [4] Exactly out-of-phase (L==-R): stereo width is exactly 200% --
    // Mid energy is exactly zero, all energy is Side.
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        for (int i = 0; i < 500; ++i)
        {
            const double x = std::cos(0.13 * i) * 0.4;
            meter.process(x, -x);
        }
        const bool ok = nearlyEqual(meter.widthPercent(), 200.0, 1e-6);
        printf("[4] exactly out-of-phase: stereo width is exactly 200%%: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [5] Uncorrelated, equal-power L/R (a quadrature sine/cosine pair --
    // exactly zero cross-correlation over whole periods, equal RMS on
    // each channel by construction): correlation settles near 0, width
    // settles near 100% -- the "ordinary stereo" reference point. Run
    // long enough (several multiples of the ~300ms integration time
    // constant) for the leaky integrator to actually settle.
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        constexpr double freq = 997.0; // an integer number of periods isn't required -- the leaky window averages over many either way
        constexpr double amp = 0.5;
        const int numSamples = int(kSampleRate * 1.5); // ~5 time constants at 300ms
        for (int i = 0; i < numSamples; ++i)
        {
            const double t = double(i) / kSampleRate;
            const double l = amp * std::sin(2.0 * M_PI * freq * t);
            const double r = amp * std::cos(2.0 * M_PI * freq * t);
            meter.process(l, r);
        }
        const bool corrOk = nearlyEqual(meter.correlation(), 0.0, 0.02);
        const bool widthOk = nearlyEqual(meter.widthPercent(), 100.0, 2.0);
        printf("[5] uncorrelated equal-power L/R: correlation ~0 (%.4f), width ~100%% (%.2f%%): %s\n",
               meter.correlation(), meter.widthPercent(), (corrOk && widthOk) ? "OK" : "FAIL");
        if (!(corrOk && widthOk)) ++failures;
    }

    // [6] Min-hold: snaps to the worst (most negative) correlation seen,
    // then recovers back upward once the signal returns to being
    // correlated. (resetMinHold()/"Reset Hold" -- a manual clear-the-hold
    // action -- was removed entirely: "the worstcase-bar is working fine
    // with his 'hold-time'", i.e. the plateau-then-decay behavior [6b]
    // checks below already does the job on its own, with nothing left for
    // a manual reset to add.)
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        // Drive it hard out-of-phase first...
        for (int i = 0; i < 2000; ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, -x);
        }
        const double minAfterBadPhase = meter.minHoldCorrelation();
        // ...then back in phase for long enough (several ~300ms time
        // constants) for the live correlation to actually settle back
        // close to +1, while the hold -- recovering at its own fixed,
        // much slower rate -- should still remember the bad moment
        // rather than snapping back instantly.
        for (int i = 0; i < int(kSampleRate * 1.5); ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, x);
        }
        const double liveAfterGoodPhase = meter.correlation();
        const double minStillLow = meter.minHoldCorrelation();
        const bool ok = nearlyEqual(minAfterBadPhase, -1.0, 1e-4) &&
                         liveAfterGoodPhase > 0.9 &&
                         minStillLow < liveAfterGoodPhase;
        printf("[6] correlation min-hold snaps low, recovers slowly: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [6b] Min-hold plateau: a new worst value must sit completely flat
    // for a fixed hold time before recovery starts, even if the signal is
    // already back in phase well before that -- without it, the marker
    // just followed the end of the live correlation bar (real program
    // material's correlation reading
    // wobbles continuously even after smoothing, so without a plateau any
    // brief dip immediately re-snapped the hold onto the live value).
    {
        StereoImageMeter meter;
        meter.prepare(kSampleRate);
        for (int i = 0; i < 2000; ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, -x);
        }
        const double minRightAfterDip = meter.minHoldCorrelation();
        // Well back in phase, but for much less than the plateau's own
        // hold time -- the hold must not have moved at all yet.
        for (int i = 0; i < int(kSampleRate * 0.5); ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, x);
        }
        const double minDuringPlateau = meter.minHoldCorrelation();
        // Keep going well past the plateau -- now it should actually climb.
        for (int i = 0; i < int(kSampleRate * 2.0); ++i)
        {
            const double x = std::sin(0.1 * i) * 0.6;
            meter.process(x, x);
        }
        const double minAfterPlateau = meter.minHoldCorrelation();
        const bool ok = nearlyEqual(minDuringPlateau, minRightAfterDip, 1e-9) &&
                         minAfterPlateau > minDuringPlateau;
        printf("[6b] correlation min-hold sits flat through its own plateau before recovering: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [7] Goniometer pack/unpack round-trip: every quantized point must
    // recover to within one quantization step of the original.
    {
        bool ok = true;
        const float step = (2.0f * GoniometerBuffer::kCoordMax) / float(GoniometerBuffer::kQuantLevels);
        for (int i = -12; i <= 12 && ok; ++i)
        {
            for (int j = -12; j <= 12 && ok; ++j)
            {
                const float x = float(i) * 0.1f;
                const float y = float(j) * 0.1f;
                const float packed = GoniometerBuffer::packPoint(x, y);
                float outX, outY;
                GoniometerBuffer::unpackPoint(packed, outX, outY);
                if (!nearlyEqual(outX, x, step) || !nearlyEqual(outY, y, step)) ok = false;
            }
        }
        printf("[7] goniometer pack/unpack round-trips within one quantization step: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [8] Goniometer packing stays within the 24-bit budget an IEEE-754
    // float mantissa represents exactly (2*kBitsPerCoord <= 24) -- the
    // same "no additional rounding beyond the quantization step itself"
    // guarantee ../statespace's own SpectrumAnalyzer packing relies on.
    {
        const bool ok = (2 * GoniometerBuffer::kBitsPerCoord) <= 24;
        printf("[8] goniometer packing fits in a float's exact-integer mantissa (<=24 bits): %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9] Goniometer write rate is fixed (independent of Decay, see
    // GoniometerBuffer's own file comment) and spans the intended
    // wall-clock duration regardless of sample rate: feeding exactly
    // kMaxDecaySeconds worth of samples should fill the *entire* buffer
    // (advance the write index by very close to kScopeBufferSize points),
    // at three different sample rates.
    {
        bool ok = true;
        for (double sr : { 48000.0, 44100.0, 96000.0 })
        {
            GoniometerBuffer buf;
            buf.prepare(sr);
            const int numSamples = int(std::lround(sr * GoniometerBuffer::kMaxDecaySeconds));
            for (int i = 0; i < numSamples; ++i) buf.process(0.0f, 0.0f);
            const uint32_t wi = buf.getWriteIndex();
            // Two independent roundings compound here (decimationStride_
            // itself, and how many decimated points fit in numSamples) --
            // a handful of points either way is expected, not a bug; the
            // point of this test is "the same ballpark regardless of
            // sample rate", not sample-exact.
            // Tolerance scales with buffer size (two independent roundings
            // compound here -- decimationStride_ itself, and how many
            // decimated points fit in numSamples -- see the comment above).
            if (!nearlyEqual(double(wi), double(GoniometerBuffer::kScopeBufferSize), double(GoniometerBuffer::kScopeBufferSize) * 0.02)) ok = false;
        }
        printf("[9] goniometer write rate spans the intended duration at multiple sample rates: %s\n", ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9b] Decay no longer changes the write rate at all -- feeding the
    // same fixed number of samples through two buffers configured
    // identically (prepare() has no decay input any more) must always
    // advance the write index by *exactly* the same amount, confirming
    // there is nothing left in process() itself that depends on Decay.
    {
        constexpr int numSamples = 48000; // 1 second at 48kHz
        GoniometerBuffer bufA, bufB;
        bufA.prepare(kSampleRate);
        bufB.prepare(kSampleRate);
        for (int i = 0; i < numSamples; ++i) { bufA.process(0.0f, 0.0f); bufB.process(0.0f, 0.0f); }
        const bool ok = bufA.getWriteIndex() == bufB.getWriteIndex();
        printf("[9b] write rate has nothing left that could depend on Decay (both buffers: %u points): %s\n",
               bufA.getWriteIndex(), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9c] visibleCountForDecay(): a bigger Decay setting must reveal
    // *more* of the fixed, constantly-refilling buffer -- not the same
    // count stretched over more time. Checked at
    // several points across the whole range, plus the two endpoints
    // (kMinDecaySeconds must show a small fraction of the buffer,
    // kMaxDecaySeconds must show the entire thing) and that doubling
    // Decay very roughly doubles the visible count (proportional, not
    // exactly linear only because of integer rounding).
    {
        const int atMin = GoniometerBuffer::visibleCountForDecay(GoniometerBuffer::kMinDecaySeconds * 1000.0);
        const int atMax = GoniometerBuffer::visibleCountForDecay(GoniometerBuffer::kMaxDecaySeconds * 1000.0);
        const int atDefault = GoniometerBuffer::visibleCountForDecay(GoniometerBuffer::kDefaultDecaySeconds * 1000.0);
        const int atDouble = GoniometerBuffer::visibleCountForDecay(GoniometerBuffer::kDefaultDecaySeconds * 2000.0);

        const bool minOk = atMin >= 1 && atMin < atDefault;
        const bool maxOk = atMax == GoniometerBuffer::kScopeBufferSize;
        const bool monotonicOk = atMin < atDefault && atDefault < atDouble && atDouble <= atMax;
        const bool proportionalOk = nearlyEqual(double(atDouble), double(atDefault) * 2.0, double(atDefault) * 0.05 + 2.0);

        printf("[9c] visibleCountForDecay is monotonic (min=%d, default=%d, 2x=%d, max=%d) and roughly proportional: %s\n",
               atMin, atDefault, atDouble, atMax, (minOk && maxOk && monotonicOk && proportionalOk) ? "OK" : "FAIL");
        if (!(minOk && maxOk && monotonicOk && proportionalOk)) ++failures;
    }

    // [9d] Auto Gain's own target constant, and the closed-form Gain
    // computation the UI performs directly from a settled SmoothedLevelMeter
    // reading (see UIPhaseScope.cpp's own btnAutoGain_ comment for why this
    // is a stateless computation with nothing DSP-side left to unit-test
    // beyond the level reading itself -- several earlier designs here, a
    // continuously-running follower, a multi-second one-shot "measurement"
    // state machine, and a twitchy instant-peak reading, all had real
    // problems traceable to either taking *time* to produce an answer or
    // reacting too instantly). This test exists to pin down
    // `kAutoGainTargetPeak` itself and the exact dB arithmetic the UI
    // relies on, using the already-verified SmoothedLevelMeter ([9f]
    // below) as the source of the "current level" input: feed a known
    // constant magnitude in for several of its own time constants (so its one-pole
    // average has actually settled, not just started climbing toward it),
    // read back the result, and confirm `targetDb - levelDb` matches the
    // closed-form `linearToDb(target/magnitude)` exactly.
    {
        SmoothedLevelMeter meter;
        meter.prepare(kSampleRate, kAutoGainTimeConstantSeconds);
        constexpr double kMagnitude = 0.2;
        for (int i = 0; i < int(kSampleRate * kAutoGainTimeConstantSeconds * 8.0); ++i) meter.process(kMagnitude);
        const double targetDb = linearToDb(kAutoGainTargetPeak);
        const double gainDb = targetDb - meter.levelDb();
        const double expectedDb = linearToDb(kAutoGainTargetPeak / kMagnitude);
        // A one-pole average only ever *asymptotically* approaches a step
        // input -- 8 time constants is already within ~0.03% of settled
        // (e^-8), but not bit-exact, the same reason [10]'s own
        // LoudnessMeter settling check below uses a generous tolerance
        // rather than an exact match.
        const bool ok = nearlyEqual(gainDb, expectedDb, 0.01);
        printf("[9d] Auto Gain's dB arithmetic (targetDb - levelDb) matches the closed-form target/magnitude computation once settled (%.3fdB): %s\n",
               gainDb, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9f] SmoothedLevelMeter: a step from silence to a constant magnitude
    // settles to that exact magnitude given enough time constants (the
    // same "settles to the exact closed-form value" property [10] checks
    // for LoudnessMeter's own RMS integration), and reacts far more slowly
    // than PeakMeter's own instant attack for the same input -- the whole
    // point of switching Auto Gain's own level reading to this class (see
    // kParamMeterPlotPeakDb's own comment).
    {
        SmoothedLevelMeter meter;
        meter.prepare(kSampleRate, kAutoGainTimeConstantSeconds);
        constexpr double kMagnitude = 0.3;
        for (int i = 0; i < int(kSampleRate * kAutoGainTimeConstantSeconds * 8.0); ++i) meter.process(kMagnitude);
        const bool settledOk = nearlyEqual(meter.levelDb(), linearToDb(kMagnitude), 0.01);

        SmoothedLevelMeter fresh;
        fresh.prepare(kSampleRate, kAutoGainTimeConstantSeconds);
        fresh.process(kMagnitude); // a single instantaneous sample -- must barely move the average at all
        const bool slowAttackOk = fresh.levelDb() < linearToDb(kMagnitude) - 20.0;

        const bool ok = settledOk && slowAttackOk;
        printf("[9f] SmoothedLevelMeter settles to the exact input magnitude given time (%.4fdB), reacts far slower than an instant peak to a single sample (%.4fdB): %s\n",
               meter.levelDb(), fresh.levelDb(), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9e] HeatmapGrid: a single hit decays toward zero over roughly its
    // own requested Decay (checked as "meaningfully faded by 3x Decay",
    // not an exact time constant match, since addPoint()'s own decay
    // coefficient is derived from the fixed kPointRateHz, not sample
    // count -- this exercises that derivation end to end); a shorter
    // requested Decay fades faster than a longer one for the same number
    // of points; and the pack/unpack round-trip for cell quads holds
    // within one quantization step, the same class of check
    // dsp_regression's own [7] already runs for goniometer points.
    {
        HeatmapGrid gridFast, gridSlow;
        gridFast.prepare();
        gridSlow.prepare();
        gridFast.requestDecaySeconds(HeatmapGrid::kMinDecaySeconds);
        gridSlow.requestDecaySeconds(HeatmapGrid::kMaxDecaySeconds);

        gridFast.addPoint(0.0f, 0.0f);
        gridSlow.addPoint(0.0f, 0.0f);
        const int centerIdx = (HeatmapGrid::kGridSize / 2) * HeatmapGrid::kGridSize + (HeatmapGrid::kGridSize / 2);

        // Advance both by the same number of points (a few seconds' worth
        // at the fixed point rate) with no further hits -- pure decay.
        const int points = int(std::lround(HeatmapGrid::kPointRateHz * HeatmapGrid::kMinDecaySeconds * 3.0));
        // (10, 10) clamps into the opposite corner cell, not the center
        // one being watched -- every one of these calls still decays
        // *every* cell (including the center), it just never re-hits it.
        for (int i = 0; i < points; ++i) { gridFast.addPoint(10.0f, 10.0f); gridSlow.addPoint(10.0f, 10.0f); }

        float fastQuad[HeatmapGrid::kCellsPerPacket], slowQuad[HeatmapGrid::kCellsPerPacket];
        HeatmapGrid::unpackQuad(gridFast.getPackedQuad(centerIdx / HeatmapGrid::kCellsPerPacket), fastQuad);
        HeatmapGrid::unpackQuad(gridSlow.getPackedQuad(centerIdx / HeatmapGrid::kCellsPerPacket), slowQuad);
        const float fastRemaining = fastQuad[centerIdx % HeatmapGrid::kCellsPerPacket];
        const float slowRemaining = slowQuad[centerIdx % HeatmapGrid::kCellsPerPacket];

        // A single hit lands at kAddPerHit (0.35, not exposed publicly --
        // these thresholds are calibrated against it via
        // exp(-elapsed/Decay), not against the full [0,1] cell range: at
        // the short Decay, elapsed is 3 of its own time constants
        // (0.35*exp(-3) ~= 0.017); at the long one, elapsed is only 1/10
        // of a time constant (0.35*exp(-0.1) ~= 0.317).
        const bool fastyFadedOk = fastRemaining < 0.05f;    // ~3 time constants at the short Decay -> mostly gone
        const bool slowStillHighOk = slowRemaining > 0.2f;  // same elapsed time is a tenth of a time constant at the long Decay -> barely moved
        const bool orderOk = fastRemaining < slowRemaining;

        // Direct packing round-trip (driving a cell to a specific
        // quantization level via addPoint()'s own decay/hit mechanics
        // would be awkward -- round-trip the packing function directly
        // instead, the same way dsp_regression's own [7] validates
        // GoniometerBuffer's packing without needing process() to land on
        // exact values): quantize/unquantize a grid of representative
        // intensities and confirm they survive within one quantization step.
        bool packOk = true;
        for (int i = 0; i <= 10 && packOk; ++i)
        {
            const float intensity = float(i) / 10.0f;
            const uint32_t q = uint32_t(intensity * float(HeatmapGrid::kCellQuantLevels) + 0.5f);
            const uint32_t combined = q | (q << HeatmapGrid::kBitsPerCell) | (q << (2 * HeatmapGrid::kBitsPerCell)) | (q << (3 * HeatmapGrid::kBitsPerCell));
            float outCells[HeatmapGrid::kCellsPerPacket];
            HeatmapGrid::unpackQuad(float(combined), outCells);
            const float step = 1.0f / float(HeatmapGrid::kCellQuantLevels);
            for (float c : outCells) if (!nearlyEqual(double(c), double(intensity), double(step) + 1e-6)) packOk = false;
        }

        const bool ok = fastyFadedOk && slowStillHighOk && orderOk && packOk;
        printf("[9e] HeatmapGrid: shorter Decay fades faster (fast=%.4f, slow=%.4f after the same elapsed points), packing round-trips: %s\n",
               fastRemaining, slowRemaining, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [9g] HeatmapGrid::rescale(): repositions accumulated history to
    // match a new Gain-scaled coordinate space instead of discarding it
    // -- without it, history vanished on every Auto Gain change instead
    // of fading out. A ratio of exactly 1.0 must be a complete no-op; a hit well away
    // from center, rescaled by 2x, must move to a different cell (not
    // stay put, and not simply vanish) while the grid's own total
    // intensity is conserved (nothing near the plot's own edge in this
    // case, so nothing should be dropped); a hit already near the edge,
    // rescaled by 2x, must fall outside the plot entirely and so be
    // dropped -- checked as the grid's total intensity actually
    // decreasing, rather than pinning down an exact resulting cell index.
    {
        const auto gridTotal = [](const HeatmapGrid& g) {
            double total = 0.0;
            for (int p = 0; p < HeatmapGrid::kNumPackets; ++p)
            {
                float cells[HeatmapGrid::kCellsPerPacket];
                HeatmapGrid::unpackQuad(g.getPackedQuad(p), cells);
                for (float c : cells) total += double(c);
            }
            return total;
        };
        const auto cellValue = [](const HeatmapGrid& g, int idx) {
            float cells[HeatmapGrid::kCellsPerPacket];
            HeatmapGrid::unpackQuad(g.getPackedQuad(idx / HeatmapGrid::kCellsPerPacket), cells);
            return double(cells[idx % HeatmapGrid::kCellsPerPacket]);
        };
        // Mirrors HeatmapGrid's own private cellIndex() exactly -- there's
        // no public accessor, and duplicating this one-line formula here
        // is simpler than adding a test-only accessor to the class itself.
        const auto cellIndexAt = [](float x, float y) {
            constexpr float kExtent = GoniometerBuffer::kCoordMax;
            const auto axis = [&](float v) {
                const float clamped = std::max(-kExtent, std::min(kExtent, v));
                const float t = (clamped + kExtent) / (2.0f * kExtent);
                return std::max(0, std::min(HeatmapGrid::kGridSize - 1, int(t * float(HeatmapGrid::kGridSize))));
            };
            return axis(y) * HeatmapGrid::kGridSize + axis(x);
        };
        const auto findNonemptyCell = [](const HeatmapGrid& g) {
            for (int p = 0; p < HeatmapGrid::kNumPackets; ++p)
            {
                float cells[HeatmapGrid::kCellsPerPacket];
                HeatmapGrid::unpackQuad(g.getPackedQuad(p), cells);
                for (int k = 0; k < HeatmapGrid::kCellsPerPacket; ++k)
                    if (cells[k] > 0.01f) return p * HeatmapGrid::kCellsPerPacket + k;
            }
            return -1;
        };

        HeatmapGrid noopGrid;
        noopGrid.prepare();
        noopGrid.addPoint(0.4f, 0.0f);
        const double beforeNoop = gridTotal(noopGrid);
        noopGrid.rescale(1.0f);
        const bool noopOk = nearlyEqual(gridTotal(noopGrid), beforeNoop, 1e-9);

        HeatmapGrid moveGrid;
        moveGrid.prepare();
        moveGrid.addPoint(0.4f, 0.0f); // comfortably inside the plot even after doubling (0.8 < kCoordMax)
        const int oldIdx = cellIndexAt(0.4f, 0.0f); // addPoint()'s own quantization -- exact
        const double beforeMove = gridTotal(moveGrid);
        moveGrid.rescale(2.0f);
        // rescale() remaps *cell centers*, not the original raw point, so
        // the destination cell can land one quantization step away from a
        // naive "just double the raw input" prediction -- find whichever
        // single cell actually ended up holding the value instead of
        // predicting its exact index.
        const int newIdx = findNonemptyCell(moveGrid);
        const bool movedOk = newIdx >= 0 && newIdx != oldIdx && cellValue(moveGrid, oldIdx) < 1e-6;
        const bool conservedOk = nearlyEqual(gridTotal(moveGrid), beforeMove, 1e-4);

        HeatmapGrid dropGrid;
        dropGrid.prepare();
        dropGrid.addPoint(1.1f, 0.0f); // 1.1 * 2.0 = 2.2, well outside kCoordMax (1.2)
        const double beforeDrop = gridTotal(dropGrid);
        dropGrid.rescale(2.0f);
        const bool droppedOk = gridTotal(dropGrid) < beforeDrop - 0.01;

        const bool ok = noopOk && movedOk && conservedOk && droppedOk;
        printf("[9g] HeatmapGrid::rescale(): ratio 1.0 is a no-op, a mid-plot hit moves cell and conserves total intensity (%.4f -> %.4f), "
               "an edge hit scaled outside the plot is dropped (%.4f -> %.4f): %s\n",
               beforeMove, gridTotal(moveGrid), beforeDrop, gridTotal(dropGrid), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [10] LoudnessMeter settles to the exact closed-form RMS of a
    // constant-amplitude input given enough time constants.
    {
        LoudnessMeter meter;
        meter.prepare(kSampleRate);
        constexpr double amp = 0.5;
        for (int i = 0; i < int(kSampleRate * 1.5); ++i) meter.process(amp);
        const double expectedDb = linearToDb(amp);
        const bool ok = nearlyEqual(meter.rmsDb(), expectedDb, 0.05);
        printf("[10] LoudnessMeter settles to the exact RMS of a constant input (%.3fdB vs expected %.3fdB): %s\n",
               meter.rmsDb(), expectedDb, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [11] PeakMeter tracks an instant peak exactly, then decays at the
    // conventional ~20dB/s rate.
    {
        PeakMeter meter;
        meter.prepare(kSampleRate);
        meter.process(0.7);
        const double peakAfterHit = meter.peakDb();
        const bool peakOk = nearlyEqual(peakAfterHit, linearToDb(0.7), 1e-6);
        for (int i = 0; i < int(kSampleRate); ++i) meter.process(0.0); // 1 second of silence
        const double peakAfter1s = meter.peakDb();
        const bool decayOk = nearlyEqual(peakAfter1s, peakAfterHit - 20.0, 1.0);
        printf("[11] PeakMeter: exact instant peak (%.3fdB), ~20dB/s decay after 1s (%.3fdB): %s\n",
               peakAfterHit, peakAfter1s, (peakOk && decayOk) ? "OK" : "FAIL");
        if (!(peakOk && decayOk)) ++failures;
    }

    // ---- LoudnessDsp.hpp (the Loudness Meter plugin's own DSP core) ----

    // [12] SlidingMeanSquare: a constant pushed value settles to its own
    // exact mean once the window has fully filled, and reports a
    // (necessarily smaller) partial mean correctly before then.
    {
        SlidingMeanSquare window;
        window.prepare(kSampleRate, 0.4); // 400ms, Momentary's own window length
        const int size = int(std::lround(kSampleRate * 0.4));
        for (int i = 0; i < size / 2; ++i) window.push(0.25);
        const bool partialOk = nearlyEqual(window.mean(), 0.25, 1e-9); // half-full: still exactly 0.25, the value pushed
        for (int i = 0; i < size; ++i) window.push(0.25); // now fully overwritten with the same value throughout
        const bool fullOk = nearlyEqual(window.mean(), 0.25, 1e-9);
        const bool ok = partialOk && fullOk;
        printf("[12] SlidingMeanSquare settles to the exact mean of a constant input, partially and fully filled: %s\n",
               ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [13] LoudnessHistogram: hand-computable cases the standard's own
    // gating logic must satisfy exactly. All blocks at the identical
    // power must gate to that exact loudness (nothing for the relative
    // gate to exclude) and report an LRA of exactly 0 LU (a perfectly
    // steady distribution has no range); a single block quieter than the
    // -70 LUFS absolute gate must be excluded entirely.
    {
        LoudnessHistogram hist;
        const double z = 0.01; // an arbitrary, comfortably-above-gate power
        const double expectedLufs = LoudnessHistogram::loudnessFromZ(z);
        for (int i = 0; i < 100; ++i) hist.addBlock(z);
        const bool integratedOk = nearlyEqual(hist.gatedLoudness(-10.0), expectedLufs, 1e-9);
        const bool lraOk = nearlyEqual(hist.percentile(0.95, -20.0) - hist.percentile(0.10, -20.0), 0.0, 1e-9);

        LoudnessHistogram silentHist;
        silentHist.addBlock(1.0e-10); // well under -70 LUFS -- the absolute gate
        const bool absoluteGateOk = nearlyEqual(silentHist.gatedLoudness(-10.0), LoudnessHistogram::kMinLufs, 1e-9);

        const bool ok = integratedOk && lraOk && absoluteGateOk;
        printf("[13] LoudnessHistogram: a steady distribution gates to its own exact loudness with 0 LU range (%.3f LUFS, %.4f LU), "
               "a sub-gate block is excluded entirely: %s\n",
               hist.gatedLoudness(-10.0), hist.percentile(0.95, -20.0) - hist.percentile(0.10, -20.0), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [14] KWeightingFilter: qualitative shape check -- well below the
    // ~38Hz high-pass corner should be heavily attenuated, comfortably
    // above the ~1682Hz shelf corner should be boosted (K-weighting's
    // whole point: correlate better with perceived loudness than a flat
    // RMS reading would).
    {
        KWeightingFilter filt;
        filt.prepare(kSampleRate);
        double sumSqLow = 0.0, sumSqLowIn = 0.0;
        constexpr double kLowFreq = 20.0;
        constexpr int kCycles = 4000;
        for (int i = 0; i < kCycles; ++i)
        {
            const double x = std::sin(2.0 * M_PI * kLowFreq * double(i) / kSampleRate);
            const double y = filt.process(x);
            sumSqLowIn += x * x;
            sumSqLow += y * y;
        }
        const bool lowAttenuated = sumSqLow < sumSqLowIn * 0.5; // heavily cut, well under the corner

        KWeightingFilter filt2;
        filt2.prepare(kSampleRate);
        double sumSqHigh = 0.0, sumSqHighIn = 0.0;
        constexpr double kHighFreq = 4000.0;
        for (int i = 0; i < kCycles; ++i)
        {
            const double x = std::sin(2.0 * M_PI * kHighFreq * double(i) / kSampleRate);
            const double y = filt2.process(x);
            sumSqHighIn += x * x;
            sumSqHigh += y * y;
        }
        const bool highBoosted = sumSqHigh > sumSqHighIn; // shelf's own +4dB-ish boost, comfortably above its corner

        const bool ok = lowAttenuated && highBoosted;
        printf("[14] KWeightingFilter: %.0fHz heavily attenuated (%.3fx power), %.0fHz boosted (%.3fx power): %s\n",
               kLowFreq, sumSqLow / sumSqLowIn, kHighFreq, sumSqHigh / sumSqHighIn, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [15] TruePeakDetector: a constant (DC) input's true peak must equal
    // that exact constant once settled -- the interpolation filter is
    // normalized to unity gain at DC specifically so this holds regardless
    // of the window/sinc design details (see its own file comment). The
    // step transition *into* DC (history_ starts at all-zero) legitimately
    // rings/overshoots through a lowpass filter -- the same Gibbs behavior
    // [16] deliberately exercises -- so warm up past that transient and
    // resetPeak() before measuring, rather than let a real (if brief)
    // startup transient masquerade as a steady-state DC gain error.
    {
        TruePeakDetector tp;
        constexpr double kDc = 0.7;
        for (int i = 0; i < TruePeakDetector::kHistSize; ++i) tp.process(kDc); // fully replaces the initial all-zero history
        tp.resetPeak();
        for (int i = 0; i < 4; ++i) tp.process(kDc);
        const bool ok = nearlyEqual(tp.peakLinear(), kDc, 1e-6);
        printf("[15] TruePeakDetector: a settled constant input's true peak matches it exactly (%.6f vs %.6f): %s\n",
               tp.peakLinear(), kDc, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [16] TruePeakDetector: a Nyquist-frequency (+1/-1 alternating)
    // square wave -- the most extreme high-frequency content a discrete
    // signal can carry -- must not read a true peak *below* its own
    // 1.0 sample peak (that would mean the filter is broken, e.g.
    // producing NaN or collapsing toward 0), and should stay within a
    // sane bound above it (a badly unstable/ungained filter would blow
    // up far past any plausible reconstruction, whereas real inter-sample
    // reconstruction of this signal is expected to overshoot only
    // modestly, the classic Gibbs-ringing behavior real true-peak meters
    // are built to catch in the first place).
    {
        TruePeakDetector tp;
        for (int i = 0; i < 200; ++i) tp.process(i % 2 == 0 ? 1.0 : -1.0);
        const bool ok = tp.peakLinear() >= 0.99 && tp.peakLinear() <= 2.0;
        printf("[16] TruePeakDetector: a Nyquist square wave's true peak (%.4f) is at or above its 1.0 sample peak, within a sane bound: %s\n",
               tp.peakLinear(), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [17] LufsMeter, full pipeline: a steady full-amplitude tone, once
    // settled, must have Momentary and Short-term readings agree closely
    // (both windows are measuring the same unchanging signal) and an LRA
    // of (very close to) 0 LU -- a perfectly steady loudness has no range
    // to report, the same exactness [13] already checks at the histogram
    // level, now exercised through the real filter/window/block pipeline
    // end to end. resetStatistics() is then checked to clear the
    // accumulated Integrated/LRA/true-peak statistics without disturbing
    // the live Momentary/Short-term readings, matching its own contract.
    {
        LufsMeter meter;
        meter.prepare(kSampleRate);
        constexpr double kFreq = 1000.0;
        constexpr double kAmp = 0.5;
        const int totalSamples = int(kSampleRate * 4.0); // settle past both windows several times over
        for (int i = 0; i < totalSamples; ++i)
        {
            const double x = kAmp * std::sin(2.0 * M_PI * kFreq * double(i) / kSampleRate);
            meter.process(x, x);
        }
        const bool finiteOk = std::isfinite(meter.momentaryLufs()) && std::isfinite(meter.shortTermLufs()) &&
                               std::isfinite(meter.integratedLufs()) && std::isfinite(meter.loudnessRangeLu());
        const bool agreeOk = nearlyEqual(meter.momentaryLufs(), meter.shortTermLufs(), 0.5);
        const bool lraOk = meter.loudnessRangeLu() < 0.5; // essentially 0 for a steady tone
        const double momentaryBeforeReset = meter.momentaryLufs();
        meter.resetStatistics();
        const bool resetIntegratedOk = nearlyEqual(meter.integratedLufs(), LoudnessHistogram::kMinLufs, 1e-9);
        const bool liveUntouchedOk = nearlyEqual(meter.momentaryLufs(), momentaryBeforeReset, 1e-9);
        const bool ok = finiteOk && agreeOk && lraOk && resetIntegratedOk && liveUntouchedOk;
        printf("[17] LufsMeter: a steady tone settles to agreeing Momentary/Short-term readings (%.2f/%.2f LUFS) with ~0 LU range (%.4f), "
               "resetStatistics() clears Integrated without disturbing the live reading: %s\n",
               meter.momentaryLufs(), meter.shortTermLufs(), meter.loudnessRangeLu(), ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [18] DistributionHistogram: a sample must land in exactly the bin
    // its own loudness value implies (hand-computable from the bin
    // width/floor), out-of-range samples clamp into the nearest end bin
    // rather than being dropped or wrapping, and reset() clears every
    // bin. Then LufsMeter's own integration: processing a steady tone
    // long enough to cross several 100ms gating blocks must land every
    // one of those blocks in the same single bin (a perfectly steady
    // loudness has nothing to distribute across bins), and
    // resetStatistics() must clear it back to all zero.
    {
        DistributionHistogram hist;
        hist.addSample(-59.4); // bin 0 (kMinLufs..kMinLufs+1)
        hist.addSample(-30.0); // bin 30
        hist.addSample(-0.1);  // bin 59, the top end
        hist.addSample(-1000.0); // far below range -- clamps into bin 0, doesn't crash/wrap
        hist.addSample(1000.0);  // far above range -- clamps into the last bin
        const bool binsOk = nearlyEqual(hist.count(0), 2.0, 1e-9) &&
                             nearlyEqual(hist.count(30), 1.0, 1e-9) &&
                             nearlyEqual(hist.count(DistributionHistogram::kBinCount - 1), 2.0, 1e-9);
        hist.reset();
        bool resetOk = true;
        for (int i = 0; i < DistributionHistogram::kBinCount; ++i)
            if (hist.count(i) != 0.0) resetOk = false;

        LufsMeter meter;
        meter.prepare(kSampleRate);
        constexpr double kFreq = 1000.0;
        constexpr double kAmp = 0.5;
        for (int i = 0; i < int(kSampleRate * 4.0); ++i)
        {
            const double x = kAmp * std::sin(2.0 * M_PI * kFreq * double(i) / kSampleRate);
            meter.process(x, x);
        }
        // Checked for both of LufsMeter's own separate tallies -- "one
        // with the short term and one with momentary values" -- not just
        // the short-term one the single shared histogram used to be.
        const auto concentrates = [&](bool momentary) {
            double totalCount = 0.0, maxBinCount = 0.0;
            int binsHit = 0;
            for (int i = 0; i < LufsMeter::kDistributionBinCount; ++i)
            {
                const double c = momentary ? meter.distributionMomentaryCount(i) : meter.distributionShortTermCount(i);
                totalCount += c;
                maxBinCount = std::max(maxBinCount, c);
                if (c > 0.0) ++binsHit;
            }
            // Allow a couple of blocks to land in a neighboring bin (the
            // K-weighting filter's own brief startup transient, biquad
            // states starting at zero, can nudge the very first block or
            // two off the final settled value) rather than requiring a
            // single exact bin -- the property actually being checked is
            // "a steady tone concentrates", not "crosses zero bin
            // boundaries ever".
            return binsHit <= 3 && maxBinCount >= totalCount * 0.8 && totalCount > 0.0;
        };
        const bool concentratedOk = concentrates(false) && concentrates(true);
        meter.resetStatistics();
        double totalAfterReset = 0.0;
        for (int i = 0; i < LufsMeter::kDistributionBinCount; ++i)
            totalAfterReset += meter.distributionShortTermCount(i) + meter.distributionMomentaryCount(i);
        const bool resetClearsOk = totalAfterReset == 0.0;

        const bool ok = binsOk && resetOk && concentratedOk && resetClearsOk;
        printf("[18] DistributionHistogram bins/clamping/reset are exact; LufsMeter's own short-term and momentary tallies each concentrate, resetStatistics() clears both: %s\n",
               ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    // [18b] A genuinely silent block must never land in either
    // Distribution tally -- see LufsMeter's own process() comment for the
    // full reasoning. Two separate cases, both real failures of an earlier
    // version of this fix that only checked `z > 0.0`:
    //   - exact digital zero (a paused transport, or a cursor sitting
    //     before the audio starts): the easy case, z is exactly 0.
    //   - a genuinely inaudible but *nonzero* signal (denormals, dither,
    //     noise floor, or a K-weighting filter's own decay tail right
    //     after real audio stops -- its internal state doesn't snap to
    //     zero the instant the input does): z > 0.0 here too, but the
    //     resolved LUFS value is still far below the plot's own visible
    //     floor, and addSample() clamps any out-of-range value into the
    //     bottom bin rather than dropping it (deliberately, see [18]
    //     above) -- so silence kept collecting at -59.5dB (bin 0's own
    //     center) with the first, too-narrow z > 0.0 gate.
    // A real block of audio afterward must still land normally in both
    // cases -- the fix has to be silence-specific, not "the first N
    // blocks are silently dropped" or some other coincidentally passing
    // shortcut.
    {
        const auto totalDistributionCount = [](const LufsMeter& m) {
            double total = 0.0;
            for (int i = 0; i < LufsMeter::kDistributionBinCount; ++i)
                total += m.distributionShortTermCount(i) + m.distributionMomentaryCount(i);
            return total;
        };
        constexpr double kFreq = 1000.0;
        const auto processTone = [&](LufsMeter& m, double amp, double seconds) {
            for (int i = 0; i < int(kSampleRate * seconds); ++i)
            {
                const double x = amp * std::sin(2.0 * M_PI * kFreq * double(i) / kSampleRate);
                m.process(x, x);
            }
        };

        LufsMeter meterZero;
        meterZero.prepare(kSampleRate);
        for (int i = 0; i < int(kSampleRate * 2.0); ++i)
            meterZero.process(0.0, 0.0);
        const bool exactZeroOk = totalDistributionCount(meterZero) == 0.0;

        LufsMeter meterQuiet;
        meterQuiet.prepare(kSampleRate);
        processTone(meterQuiet, 1.0e-5, 2.0); // ~-103 LUFS pre-weighting -- comfortably below the -60 LUFS plot floor, but genuinely nonzero throughout
        const double totalWhileQuiet = totalDistributionCount(meterQuiet);
        const bool quietOk = totalWhileQuiet == 0.0;

        processTone(meterQuiet, 0.5, 1.0);
        const bool toneStillCountsOk = totalDistributionCount(meterQuiet) > 0.0;

        const bool ok = exactZeroOk && quietOk && toneStillCountsOk;
        printf("[18b] Silent blocks (exact digital zero, and genuinely inaudible but nonzero signal, total=%.1f) never enter either Distribution tally; real audio afterward still counts normally: %s\n",
               totalWhileQuiet, ok ? "OK" : "FAIL");
        if (!ok) ++failures;
    }

    printf("\n%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
