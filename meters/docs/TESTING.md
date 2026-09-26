# Testing

One verification tool, no DPF/GUI dependency -- builds and runs in well
under a second.

```sh
./build/bin/dsp_regression
```

## `dsp_regression` (`tests/dsp_regression.cpp`)

Pure closed-form math against both plugins' own DSP cores
(`plugin/dsp/PhaseScopeDsp.hpp` and `plugin/dsp/LoudnessDsp.hpp`), no
audio files, no synthetic-signal testbench (unlike `../compressor`'s
two-tier `dsp_regression` + `audio_testbench` split) -- small enough, and
enough of the correctness properties involved are exact by algebraic
construction (or, for the Loudness Meter, by deliberately hand-computable
synthetic cases -- see items 9-11 below) rather than only approximately
true, that a single fast pure-math suite already gives real coverage
without needing a second tool. 25 checks across twelve areas:

1. **Correlation, exact cases** (`[1]`/`[2]`): identical `L==R` gives
   exactly `+1`, exactly out-of-phase `L==-R` gives exactly `-1` --
   `sumLL == sumRR == sumLR` (or its negation) at every step by
   construction, so these don't even need the ~300ms integrator to
   settle.
2. **Stereo width, exact cases** (`[3]`/`[4]`): mono (`L==R`) gives
   exactly 0% (Side energy is exactly zero by the shared-sums algebra),
   exactly out-of-phase gives exactly 200% (Mid energy is exactly zero).
   Checking these exactly, independently of the correlation cases above,
   is what actually validates `widthPercent()`'s "reuse the same three
   sums" derivation (`docs/ARCHITECTURE.md`) rather than trusting it by
   analogy.
3. **Settled, non-exact cases** (`[5]`): a quadrature sine/cosine pair
   (uncorrelated, equal-power by construction) settles near 0 correlation
   and 100% width after several ~300ms time constants -- the "ordinary
   stereo" reference point neither of the exact cases above can check.
4. **Correlation min-hold** (`[6]`/`[6b]`): `[6]` snaps to exactly the
   worst correlation seen, and recovers only partially (not instantly)
   after returning to a correlated signal for a comparable amount of
   time. (A manual "Reset Hold" clear was removed entirely, so there's
   nothing left for this check to test there.)
   `[6b]` checks the hold's own
   plateau specifically -- added after the marker was found to just
   follow the end of the live correlation bar: the hold must sit at *exactly* the same value after
   less than its own fixed hold time of good-phase signal, then actually
   have moved once that time is exceeded, confirming a brief recovery
   doesn't get short-circuited by ordinary signal wobble the way the
   pre-plateau design's single-stage "snap, then immediately recover"
   design did.
5. **Goniometer packing, write rate, and Decay windowing**
   (`[7]`/`[8]`/`[9]`/`[9b]`/`[9c]`): a grid of (x, y) points round-trips
   through `packPoint()`/`unpackPoint()` within one quantization step --
   this is the test that caught a real rounding bug (see
   `docs/ARCHITECTURE.md`); the 24-bit total stays within a float's
   exact-integer mantissa range; the buffer's write rate fills the entire
   `kScopeBufferSize` (`4096`, doubled from an original `2048`) in exactly
   `kMaxDecaySeconds` at three different sample rates (48kHz, 44.1kHz,
   96kHz), confirming it's duration-correct rather than sample-count-fixed
   (`[9]`'s own tolerance scales as `2%` of `kScopeBufferSize` rather than
   a fixed number, specifically so it keeps working if this constant
   changes again); `[9b]` confirms two identically-configured buffers
   advance at *exactly* the same rate (there is nothing left in
   `process()` that could depend on Decay, unlike an earlier version);
   and `[9c]` checks `visibleCountForDecay()` directly
   -- monotonically increasing across the whole Decay range, reaching the
   full buffer at `kMaxDecaySeconds`, and roughly proportional to the
   requested duration (doubling Decay roughly doubles the visible point
   count) -- now guaranteed by construction. Resolution
   and Smooth (the two rendering-cost/curve-shape controls added
   alongside the buffer-size increase) live entirely in `GoniometerWidget`
   (GUI/DGL-dependent, no DSP-side counterpart at all) and so have no
   `dsp_regression` case of their own, consistent with this suite only
   ever covering the framework-independent DSP core.
6. **Auto Gain's arithmetic and level reading** (`[9d]`/`[9f]`): Auto Gain
   is a plain `targetDb - levelDb` computation (the earlier one-shot
   `GainMeasurement` state machine was removed entirely -- it could get
   permanently stuck in "Measuring..." on hosts
   that stop calling `run()` when transport is stopped) on a
   `SmoothedLevelMeter` reading (a one-pole average with an adjustable,
   ~1s-or-longer time constant -- replacing an even earlier `PeakMeter`
   reading that made continuous tracking visibly twitchy). `[9d]` confirms the arithmetic matches the exact
   closed-form `target/magnitude` computation once the average has
   settled (a generous tolerance, since a one-pole average only ever
   asymptotically approaches a step input, never bit-exactly). `[9f]`
   checks `SmoothedLevelMeter` itself directly: a settled constant input
   reads back as that exact magnitude, and a single instantaneous sample
   barely moves the average at all (>20dB below what an instant-attack
   peak meter would read from the same one sample) -- the whole point of
   the switch.
7. **`HeatmapGrid`** (`[9e]`/`[9g]`): `[9e]` checks a single hit's decay is
   measurably faster at a short requested Decay than a long one after the
   same elapsed number of (fixed-rate) points, confirming the per-point
   decay coefficient is genuinely time-scaled and not just a constant, and
   that its 4-cells-per-Parameter packing round-trips within one
   quantization step, the same class of check `[7]` already runs for
   `GoniometerBuffer`'s own points. `[9g]` checks `rescale()` (which
   resamples the heatmap when Gain changes, see `docs/ARCHITECTURE.md`):
   a ratio of `1.0` is a complete no-op; a hit well away
   from the plot's own center, rescaled by 2x, moves to a different cell
   while the grid's total intensity is conserved (nothing near the edge in
   that case, so nothing should be dropped); and a hit already near the
   edge, scaled outside the plot entirely, is dropped -- checked as the
   grid's total intensity actually decreasing, rather than pinning down an
   exact resulting cell index (`rescale()` remaps *cell centers*, which
   can land one quantization step away from a naive "just scale the raw
   input" prediction).
8. **Meter ballistics** (`[10]`/`[11]`): `LoudnessMeter` settles to the
   exact closed-form RMS of a constant-amplitude input; `PeakMeter` tracks
   an instant peak exactly and decays at the conventional ~20dB/s rate.

The remaining items are the Loudness Meter plugin's own DSP core
(`plugin/dsp/LoudnessDsp.hpp`), see `docs/ARCHITECTURE.md`'s own "Loudness
Meter" section for the full design each of these is checking:

9. **`SlidingMeanSquare` and `LoudnessHistogram`** (`[12]`/`[13]`): a
   constant value pushed into the sliding window settles to its own exact
   mean, both partially and fully filled; the gating histogram used for
   Integrated Loudness and Loudness Range gates a perfectly steady
   distribution to its own exact loudness with an LRA of exactly 0 LU
   (nothing for the relative gate to exclude, no range to report in a
   distribution with only one value), and excludes a single block below
   the -70 LUFS absolute gate entirely -- all hand-computable exactly,
   independent of the real K-weighting filter or audio pipeline.
10. **`KWeightingFilter` and `TruePeakDetector`** (`[14]`/`[15]`/`[16]`):
    the K-weighting filter's own qualitative shape (well below its ~38Hz
    high-pass corner is heavily attenuated, comfortably above its
    ~1682Hz shelf corner is boosted); a settled constant (DC) input's
    true peak matches it exactly (the interpolator is normalized to unity
    gain at DC specifically so this holds); and a Nyquist-frequency
    (+1/-1 alternating) square wave's true peak sits at or above its own
    1.0 sample peak within a sane bound -- the classic Gibbs-ringing case
    a true-peak detector exists to catch, without asserting an exact
    predicted overshoot value this custom (non-ITU-reference) filter
    isn't meant to reproduce bit-exactly.
11. **`LufsMeter`, full pipeline** (`[17]`): a steady full-amplitude tone
    settles to closely agreeing Momentary and Short-term readings with an
    LRA of essentially 0 LU, exercising item 9's own exactness through the
    real filter/window/block pipeline end to end rather than via
    hand-fed histogram values; `resetStatistics()` then clears Integrated
    back to "nothing measured yet" while leaving the live Momentary
    reading completely undisturbed, matching its own documented contract.
12. **`DistributionHistogram`** (`[18]`): a sample lands in exactly the
    bin its own loudness value implies, an out-of-range sample clamps
    into the nearest end bin rather than being dropped or wrapping, and
    `reset()` clears every bin -- all hand-computable exactly. Then
    `LufsMeter`'s own integration: a steady tone processed for several
    seconds must concentrate almost entirely in a single bin (a small
    allowance for the K-weighting filter's own brief startup transient,
    biquad states starting at zero, rather than requiring literally every
    block in exactly one bin), and `resetStatistics()` clears it back to
    all zero.

No audio-file testbench exists yet for this project -- if a future plugin in this family has DSP
complex enough that pure closed-form math stops being sufficient coverage
(a spectrum analyzer's FFT windowing, say), add one then, following
`../compressor/tools/testbench/audio_testbench.cpp`'s pattern.
