# Architecture

Both plugins copy their input straight to the output and only measure.
The DSP headers (`plugin/dsp/`) are header-only and have no DPF/DGL
dependency.

# Phase Scope

## DSP core (`plugin/dsp/PhaseScopeDsp.hpp`)

```
input L/R --[unmodified copy]--> output L/R
         |
         +--> PeakMeter / LoudnessMeter (L, R) --> bargraphs
         +--> StereoImageMeter (shared L/R statistics) --> Correlation, Width, worst-case hold
         +--> SmoothedLevelMeter (raw Mid/Side magnitude) --> kParamMeterPlotPeakDb (Auto Gain)
         +--> rotated Mid/Side --[x Display Gain]--> GoniometerBuffer (fixed write rate) --> vectorscope
                                                    |
                                                    +--> HeatmapGrid (1-30 s density map) --> glow layer
```

Only the plotted Mid/Side values are scaled by Display Gain; every meter
reads the real signal, so Gain changes how large the goniometer draws,
never what the numbers say.

- **`PeakMeter` / `LoudnessMeter`**: instant peak with about 20 dB/s decay,
  and about 300 ms RMS integration.
- **`StereoImageMeter`**: three one-pole smoothed (about 300 ms) sums,
  `E[L*R]`, `E[L^2]`, `E[R^2]`.
  - `correlation() = E[LR] / sqrt(E[LL] * E[RR])` -- Pearson correlation,
    in [-1, +1].
  - `widthPercent()` from the same sums (`E[Mid^2]` and `E[Side^2]` follow
    from them): `200 * sqrt(E[S^2]) / (sqrt(E[M^2]) + sqrt(E[S^2]))` --
    0 % mono, 100 % uncorrelated equal-power stereo, 200 % pure Side.
  - `minHoldCorrelation()`: drops instantly to a new minimum, stays flat
    for `kHoldPlateauSeconds` (1.5 s; every new minimum restarts it), then
    recovers slowly. The plateau is needed because real program
    material's correlation keeps moving even after smoothing; without it
    the marker would sit on the live reading.
- **`SmoothedLevelMeter`**: a one-pole average (`kAutoGainTimeConstantSeconds`
  = 2.5 s) of the raw Mid/Side magnitude, exposed as
  `kParamMeterPlotPeakDb`. The UI computes Auto Gain from it:
  `gain = clamp(dB(kAutoGainTargetPeak) - plotLevelDb, -24, +24)` with
  `kAutoGainTargetPeak = 0.18`. A longer time constant makes Auto Gain
  calmer but, since an average sits below the peaks, also raises the gain
  it settles on.
- **`GoniometerBuffer`**: a ring of `kScopeBufferSize` (4096) packed (x, y)
  points written at a fixed rate -- the decimation stride is derived in
  `prepare()` from the sample rate and `kScopeBufferSize /
  kMaxDecaySeconds` (3 s), so points are about 0.73 ms apart at any
  sample rate. `visibleCountForDecay(decayMs)` says how many of the newest
  points a given Decay shows: a longer Decay shows more points at the same
  density.
- **`HeatmapGrid`**: a 32 x 32 grid of leaky cells fed exactly the points
  the goniometer buffer writes. Each point decays every cell once and
  brightens its own cell (`kAddPerHit` 0.35, capped at 1). Since the feed
  rate is fixed, the decay needs no sample rate. Cells live in the plotted
  (Gain-scaled) coordinates; when Gain changes, `rescale(ratio)` moves
  every cell's content to where it now belongs (summing, and dropping what
  leaves the plot), so the history survives continuous Auto Gain changes.

## Bulk data transport: packed output Parameters

A float Parameter is the only channel between DSP and UI that DPF
guarantees across processes. `GoniometerBuffer::packPoint()` quantizes a
point to 12 bits per axis and packs both into one float (24 bits, all
exactly representable); `unpackPoint()` rounds with `std::lround()` --
adding 0.5 before truncating can round to the neighbouring integer near
2^24 and corrupt both fields. Heatmap cells need less precision: 4 cells
of 6 bits per float, 256 Parameters for 1024 cells.
`kParamScopeWriteIndex` carries the newest slot modulo the buffer size, so
it stays a small exact integer however long a session runs.

## Plugin side (`plugin_phasescope/`)

No Parameter shapes the audio. Parameters are output readings (levels,
correlation, hold, width, plot level, the packed scope and heatmap
banks, the write index) or display controls (`kParamDisplayGain`, and
`kParamHeatmapDecaySec`, which changes what `HeatmapGrid` computes).

Per sample, `run()` feeds the level and stereo meters from the real
signal, computes Mid/Side (`side = (L-R)/2`, `mid = (L+R)/2`), feeds the
raw magnitude to `SmoothedLevelMeter`, scales Mid/Side by Display Gain
into `GoniometerBuffer`, and passes every point it writes on to
`HeatmapGrid`.

Rendering preferences are `State`s the DSP never reads: `drawLines`,
`decayMs`, `resolutionPercent`, `smoothLines`, `heatmapEnabled`, and
`autoGainActive`.

## GUI (`gui/`)

- **`SimpleButton`**, **`LevelMeterWidget`**, **`ParamStripWidget`** (the
  shared MimoMusic widgets, with tooltips), **`StereoMeterWidget`**
  (Correlation and Width bars).
- **`GoniometerWidget`**: `collectVisiblePoints()` runs once per repaint:
  it walks back `visibleCount` points from the newest slot (Decay), keeps
  every `round(100 / resolutionPercent)`-th (Res -- a pure drawing-cost
  setting) and maps them to the screen. Age and fade are normalized to
  the visible count, so the trail fades over the same proportion at any
  Decay. Dots draw a fading cloud; Lines connect the kept points, with
  Smooth using a Catmull-Rom spline (the Mid/Side-to-screen mapping is
  affine, so the spline can be built in screen coordinates). Colour
  follows the correlation. The heatmap is drawn first, as soft
  overlapping circles through four dark colour stops (blue, green,
  yellow, red, all channels below 100, alpha up to 0.7), so grid, labels
  and the live trail stay readable on top. Labels: M up, L and R at the
  upper ends of their diagonals (a hard-left signal traces the upper-left
  diagonal).
- **`TooltipOverlay`**: a full-window, click-through widget drawn last;
  the UI polls the buttons' and fields' hover state once per idle tick and
  shows the matching text.
- **`UIPhaseScope`**: collects the scope and heatmap packets in arrays and
  hands each batch to the widget once per idle tick. **Freeze** is local
  UI state that stops redrawing (settings changes still apply). **Auto
  Gain** is a toggle: while on, every new plot-level reading recomputes
  the gain and writes `kParamDisplayGain` inside one open host edit
  gesture; off leaves Gain where it is. Its button is never disabled.
  Decay and Res are fields backed by States rather than Parameters. Smooth
  is greyed out in Dots mode.

## Known limitations

- Correlation and width are wideband (no per-band breakdown).
- Lines mode connects the buffer's points, which are about 0.73 ms apart;
  Smooth interpolates between them but can't add detail the buffer never
  captured. Only a larger `kScopeBufferSize` (more Parameters) would.

# Loudness Meter

## DSP core (`plugin/dsp/LoudnessDsp.hpp`)

```
input L/R --[unmodified copy]--> output L/R
         |
         +--> KWeightingFilter (L, R) --> SlidingMeanSquare (400 ms, 3 s) --> Momentary / Short-term LUFS
         |                                          |
         |                                          +--(every 100 ms)--> LoudnessHistogram (x2) --> Integrated, LRA
         |                                                           +--> DistributionHistogram (x2) --> distribution view
         +--> TruePeakDetector (L, R) --> True Peak dBTP, PLR, PSR
```

- **`KWeightingFilter`**: the standard's high-frequency shelf followed by a
  high-pass, computed from the analog prototype parameters with the RBJ
  bilinear formulas at the actual sample rate (not the 48 kHz table).
- **`SlidingMeanSquare`**: an exact rectangular window (ring buffer with a
  running sum) -- Momentary (400 ms) and Short-term (3 s) are literal time
  windows in the standard, not decaying averages.
- **`LoudnessHistogram`**: 0.1 LU bins storing summed *power*, so gated
  means are exact without storing every block (dB values can't be
  averaged directly). One instance for Integrated (400 ms blocks,
  absolute -70 LUFS and relative -10 LU gate), one for LRA (3 s blocks,
  relative -20 LU gate, 95th minus 10th percentile).
- **`DistributionHistogram`**: a plain count per 1 LU bin from -60 to
  0 LUFS (out-of-range values clamp into the end bins), fed every 100 ms;
  one for Short-term, one for Momentary.
- **`TruePeakDetector`**: 4x oversampling with a Blackman-windowed sinc
  (8 samples of support per side), each phase normalized to unity at DC.
  A general-purpose design, not the ITU reference filter; it adds 8
  samples of reporting latency, which doesn't affect the audio.
- **`LufsMeter`**: ties everything together. The 400 ms and 3 s windows
  sampled every 100 ms *are* the standard's overlapping gating blocks.
  `resetStatistics()` clears Integrated, LRA, the distributions and the
  true-peak maxima, leaving the live windows running.

## Plugin side (`plugin_loudness/`)

Parameters are output readings only: the loudness values, plain
per-channel dBFS peak/RMS for the L/R bars (same `PeakMeter` /
`LoudnessMeter` as Phase Scope), and two banks of 60 distribution bin
counts (range 0–5000 each, a realistic ceiling -- some hosts judge output
changes relative to the declared range).

**Reset** is a `State` (`resetRequest`, only for the DSP) pushed with an
incrementing counter, not a trigger Parameter: in LV2, DPF passes a
control port on only when its value changes, and relies on the host to
reset trigger ports -- a host that doesn't would swallow every click after
the first. A State push reaches `Plugin::setState()` every time.

`targetLufs` (`"off"` or a number) and `showDistribution` are UI-only
States.

## GUI (`gui/`)

- **`LevelMeterWidget`** for the plain L/R dBFS bars.
- **`LoudnessMeterWidget`**:
  - **Bar meter**: Momentary as a filled bar, Short-term as a pointer, and
    Short-term and Integrated as horizontal bars on the same scale.
  - **History graph**: Momentary and Short-term as Catmull-Rom splines
    with translucent fills (Short-term behind, Momentary in front with the
    lower opacity). Samples are pushed every 100 ms of wall-clock time,
    independent of the host's UI rate, into a ring of 6 hours. Scroll to
    zoom (2 s to 6 h, default 60 s), drag to pan back in time (this detaches
    the view from "now"), double-click to return to the live view.
  - **Distribution view**: both distributions in one box on a shared count
    scale, Short-term behind Momentary, each drawn as a Fritsch-Carlson
    monotone cubic spline -- it never overshoots its data, so the curve's
    peak is exactly the most frequent bin. Each peak is marked with its
    LUFS value; the curves are also clipped to the box. Scroll zooms the
    LUFS axis around the mouse (Alt for finer steps), double-click resets.
  - **Target** line on all views, from `kTargetPresets` in the UI.
  - **Readout panel** with the numbers; it flashes briefly after Reset
    (`setResetFlash()`), since on steady material the cleared values can
    return to similar numbers within a second.
- **`UILoudnessMeter`**: mirrors every reading into the widget, cycles
  Target and View via States, pushes Reset, and uses `TooltipOverlay`.

## Known limitations

- Stereo only (no mono or surround channel weighting).
- The true-peak filter is not certified against the ITU conformance
  vectors -- fine for monitoring, not for certification.
