# Architecture

## The model

`ssfilter::StateSpaceMatrices` (`plugin/dsp/StateSpaceTypes.hpp`) is the
central data structure: an `order` (1..128) and fixed-size `A` (order x
order), `B` (order x 2), `C` (2 x order), `D` (2 x 2), always stored at the
maximum size (`kMaxOrder = 128`) so an order change never allocates. It
also carries:

- `nonlinear` -- an optional `NonlinearSchedule` (state-dependent gain);
- `multiband` -- an optional `MultibandSchedule` (per-band gain);
- `sparseCascade2` -- set by the cascade builders when `A` is known to be
  block lower-triangular with 2-wide diagonal blocks, which lets the
  engine skip the structural zeros.

Everything that isn't the engine treats the struct as an LTI model and
passes those fields through. A `.ssjson` preset is its JSON serialization.

## Real-time engine (`StateSpaceEngine`)

Per sample:

```
y[n]   = C x[n] + D u[n]
x[n+1] = A x[n] + B u[n]
```

- **Model hand-off**: `setModel()` is called from the UI/message thread.
  Two preallocated model slots and an atomic index form a wait-free double
  buffer; the audio thread picks up the current slot once per block. A
  monotonic version counter tells it reliably that a new model arrived,
  even if several arrived between two blocks. No locks or allocation on
  the audio thread.
- **Coefficient smoothing**: when the new model has the same order as the
  running one (every ordinary edit, e.g. dragging a band), `A/B/C/D` glide
  towards it with a 5 ms one-pole ramp. A stable target says nothing about
  the jump from the old coefficients applied to the current state, so an
  instant switch would click.
- **Duck on order changes**: adding, removing, enabling or soloing a band
  changes the order, and two differently sized state vectors have no
  meaningful in-between. The output fades out (~8 ms), the model is
  switched with a full state reset, and it fades back in (~8 ms). A
  crossfade would need the new filter to settle within the fade, which
  narrow filters don't.
- **Sparse cascade path**: for `sparseCascade2` models the recursion only
  visits the nonzero block structure; results are identical to the dense
  path.
- **Gain stages**: the `NonlinearSchedule` and `MultibandSchedule` stages
  below run after `y[n]`. Per-band gains of the multiband stage are
  published through an atomic array (`getBandGainDb()`).
- **Safety nets**, in addition to the stability guard: a non-finite
  output zeroes the state, outputs silence for that sample and raises a
  one-shot fault flag (`kParamFaultFlag`); a hard output ceiling of ±64
  (+36 dBFS) is a last resort that never engages in normal use.

## Stability guard (`StabilityGuard`)

The recursion is stable iff the spectral radius `rho(A) < 1`. Only the
largest eigenvalue magnitude is needed, so there is no full
eigendecomposition:

1. **Cheap exact bound**: `rho(A) <= min(||A||_1, ||A||_inf)`. If that is
   already below the target radius (0.99995), `A` is provably stable and
   untouched. For resonant filters the bound is typically 2–3x the true
   radius, so most filters continue to step 2.
2. **Near-exact estimate** by repeated squaring,
   `rho(A) = lim ||A^N||^(1/N)` for any submultiplicative norm -- this holds
   unconditionally, including for complex-conjugate pole pairs of equal
   magnitude (every resonant biquad), where single-vector power iteration
   is biased. 40 squarings give N = 2^40. If the estimate plus a small
   margin exceeds the target, `A` is scaled uniformly (`rho(sA) =
   s*rho(A)` exactly) and re-checked against the estimate. A fallback
   scales by the exact norm bound.

Uniform scaling keeps the pole angles (resonant frequencies) and pulls all
poles radially inwards: a too-hot filter is tamed, not reshaped. The
target 0.99995 leaves room for legitimately narrow filters, whose poles
sit very close to the unit circle; double precision handles that margin
safely. Non-finite entries in any matrix are zeroed. The guard runs on
every edit, preset load and state restore -- never in the audio callback.

## Transfer function (`TransferFunction`)

`H(z) = D + C (zI - A)^-1 B` on the unit circle, by dense complex Gaussian
elimination with partial pivoting: the full 2x2 complex response,
including cross-channel paths. O(order^3) per frequency, so it only runs
on the UI thread, debounced over a few idle ticks. `sweep()` returns
unwrapped phase; the Bode plot wraps it to ±180°.

## NonlinearSchedule (state-dependent gain)

A fixed `A/B/C/D` model can't compress (see `docs/EXTENDING.md`).
`NonlinearSchedule` scales the output by a level-dependent gain -- the same
as `C(p) = gain(p)*C0`, `D(p) = gain(p)*D0` -- leaving `A` and `B`, and so
the stability, untouched. Per sample:

1. **Envelope**: `level = max(|u0|, |u1|)`, smoothed by a one-pole with
   separate attack and release coefficients -- in the **linear** domain.
   `|sample|` touches zero twice per cycle, so smoothing in dB would make
   the result depend on frequency.
2. **Curve**: `envDb` is looked up in `GainCurve`, 25 piecewise-linear
   points from -72 to +24 dB (4 dB apart), flat beyond the ends.
3. **Apply**: the output is multiplied by the resulting gain.

`GainCurve::classicCompressor()` builds the curve of a classic
compressor (threshold, ratio, up ratio, makeup); the Multiband Compressor
and the Dynamic EQs use the same two-sided formula.

### Learning a curve (`GainCurveTrainer`)

Variable projection: attack and release are searched on a 6 x 6 grid
(0.2–100 ms, 10–3000 ms); for each grid point the 25 curve values are the
exact solution of a ridge-regularized (λ = 4 dB², pulling unsupported
points towards 0 dB) linear least-squares problem, solved by Cholesky --
for a fixed envelope the predicted gain is linear in the curve values. The
best residual wins. The trainer walks the envelope exactly like the engine
does. `tools/trainer/fit_gain_curve.cpp` is the command-line front end.

### Editor panel (`NonlinearPanelWidget`)

At the bottom of Matrix Filter's editor: Enabled, Attack and Release
(model fields), Threshold / Ratio / Makeup plus **Apply Curve** (which
bakes a classic curve into the 25 points; the three values themselves are
not saved), and the curve graph (-72..+24 dB level, -48..+24 dB gain):
click or drag to set the nearest point, double-click to reset it, scroll
to nudge by 0.5 dB.

## MultibandSchedule (multiband compression)

Up to `kMaxBands` (6) bands, each with its own envelope (of its own
filtered signal) and gain curve. `mappers::makeStereoCrossoverBank()`
builds the Linkwitz-Riley crossover as parallel per-band chains
(block-diagonal `A`, `C` concatenated, `D` summed). Each `BandSplit`
records the band's state range and its own feed-through `feedD` -- stored
separately because the combined `D` is a sum that can't be split again.
The engine computes each band's contribution (`feedD*u + C_band*x_band`),
applies that band's gain and sums. With all gains at 0 dB the bank sums
to exactly unity. Training: see `docs/EXTENDING.md`.

## Matrix Filter editor (`gui/`)

`UIStateSpace` owns the model, four `MatrixGridWidget`s (A/B/C/D, laid out
as `[[A,B],[C,D]]`), the `BodePlotWidget`, the `NonlinearPanelWidget` and
the toolbar. The grids talk to the UI through `ssgui::IModelEditorHost`.

Cell editing: click to edit (pre-filled), double-click for an empty
buffer, Enter/Tab commits, Escape cancels, arrow keys commit and move;
clicking elsewhere also commits, and starting an edit takes keyboard focus.
Every commit runs the stability guard and sends the model to the DSP as
the `"model"` State; `stateChanged()` brings host restores back into the
editor. Bode curve visibility and the last preset folder are States too.

Limitations: no click-and-drag on cells; at large orders the cells get too
small for text (labels are hidden, no zoom yet).

## Shared mechanisms of the EQ-type plugins

### Band list as a Parameter mirror

The band lists of the EQs and Dynamic EQs travel as ordinary, hidden
Parameters -- one bank per band field (`kParamBandTypeBase`,
`kParamBandFreqBase`, ... `kParamBandSoloBase`), plus band count, output
trim and crossfeed levels -- not as a State. Ardour stores States in an
external file next to the session, which its channel-strip import doesn't
copy, while Parameters are stored inline and survive.

- **Publishing** can only come from the UI: a host persists only values it
  set itself. After every edit the UI writes the complete bank and then
  `kParamMirrorCommit`, which is always last. Hosts deliver changed
  Parameters in index order before `run()`, so when the commit arrives the
  bank is complete, and the DSP rebuilds its model from it in one step.
- **Restoring** is plain DSP bookkeeping: the host sets the Parameters and
  the DSP adopts them.
- New fields are appended before `kParamMirrorCommit`; existing indices
  never move.
- Hosts pass a Parameter on only when its value changes, so every field
  that matters must be in the mirror, and values the UI "starts at" must
  match the host's default (e.g. the audition band starts at 0).

### Editor heartbeat

The editor increments `kParamUiHeartbeat` every idle tick. `run()` feeds
the spectrum analysers only if it changed within the last ~2 s -- a closed
editor costs no FFT time. A counter, not a toggle, because two toggles in
one block would cancel out.

### SpectrumAnalyzer (`plugin/dsp/SpectrumAnalyzer.*`)

Two analysers per plugin, before and after the processing (after = the
real output, so while bypassed both traces coincide). **Spectrum Source**
Off / In / Out / Both chooses which run at all; with per-channel bands the
output trace splits into left and right analysers. Each mono-mixes its
input into a ring buffer and every `fftSize/4` samples applies a Hann
window and an iterative radix-2 FFT (no allocation), then publishes 384
log-spaced bands (20 Hz–20 kHz) in dB.

- **Resolution**: FFT size 2048–16384 (default 8192; 16384 in the Dynamic
  EQs). Bigger means finer low frequencies and more latency; CPU cost per
  second barely changes. A size change is requested atomically and applied
  on the audio thread at the next `process()`.
- **Calibration**: band power is the *sum* of `|X[k]|^2` over the band's
  bins with one global normalization (Parseval), so a pure tone reads its
  true level however many bins the band spans. Low bands capture at least
  5 bins (the Hann main lobe is ~4 bins wide) -- tones read correctly,
  broadband noise reads slightly high there.
- **Transport**: bands are packed 3 per output Parameter (8 bits each, 128
  Parameters per analyser), since a Parameter is the only DSP-to-UI
  channel that works across processes.
- **Display** (UI only): one-pole smoothing with the Smoothing time; a
  fixed range of -66..-6 dB by default, or Autoscale, which drifts (600 ms)
  towards the range of the visible traces instead of jumping. Its scale is
  labelled on the right edge, separate from the EQ gain axis on the left.

### Export

**Export to State-Space...** writes
`Serialization::toJson(StabilityGuard::enforce(assembleStateSpace(model, sr)))`
-- an ordinary `.ssjson` that Matrix Filter opens directly. There is no
separate interchange format.

## Parametric EQ (`plugin_eq/`, `gui/eq/`, `plugin/dsp/EqModel.*`)

A coefficient mapper on the shared core, as its own plugin: Matrix Filter
edits coefficients directly, the EQ edits musical parameters that
overwrite them -- one plugin with both would clobber hand edits.

### EqModel

A vector of `EqBand`s plus output trim and the two crossfeed levels. Each
band: type (Bell, Low/High Shelf, Low/High Pass, Notch, Band Pass, All
Pass, Band Shelf), frequency, gain, Q, bandwidth (Band Shelf), enabled,
solo, slope stages (1–4 for Low/High Pass: 12–48 dB/octave), channel
(Both/Left/Right), crossfeed (None/L→R/R→L), harmonic group and ratio.

- **`assembleStateSpace()`** designs every active band (enabled, or only
  the soloed ones if any; a soloed harmonic twin brings its group) with
  the RBJ formulas and cascades them per channel
  (`makeStereoBiquadChain()`, block lower-triangular, `sparseCascade2`).
  A **Band Shelf** is a High Shelf at `centre / 2^(bw/2)`, a Low Shelf at
  `centre * 2^(bw/2)` and a flat `-G` folded into the second stage, so it
  returns to 0 dB outside the corners. Left/Right-only bands go into that
  channel's chain only; when a one-channel band is soloed, the other
  channel is silenced. **Crossfeed** bands form separate chains fed by
  the source channel and are grafted onto the model afterwards, adding into
  the destination channel's output, scaled by the per-direction
  crossfeed level. Chains are capped at 32 biquads per channel.
- **`evaluateCombinedResponse()`**: the same response computed as the
  product of each band's closed-form biquad response -- O(bands) per
  frequency instead of O(order^3), so the graph redraws on every mouse
  move. `dsp_regression` checks both paths agree.
- **Harmonic groups**: members share a group id and store their ratio to a
  virtual base frequency; moving any member repositions all
  (`propagateHarmonicFreq()`), and removing or unlinking one never
  disturbs the others.

### PluginEq

`StateSpaceEngine` unchanged, plus the band-list mirror, two or three
spectrum analysers, and **Audition**: a band-pass tap at the selected
band's frequency and Q (between the corners for a Band Shelf), scaled by
its gain, summed over its harmonic group, replaces the output. The EQ
model is kept so `sampleRateChanged()` can re-derive the coefficients.

### Editor (`UIEq`, `EqGraphWidget`, `BandListWidget`)

Both widgets work through `ssgui::IEqEditorHost`. `EqGraphWidget`: click
to add a band, drag nodes (gain-less types move only horizontally), scroll
for Q (BW for a Band Shelf, whose corner handles also drag the
bandwidth), double-click or Delete to remove, right-click menu (delete,
duplicate, type, slope, harmonic twin, unlink), right-button drag to
audition while held, a value readout on hover and drag, the gain range
label (click to cycle, drag for 1–40 dB), channel rings and per-channel
curves, and the crossfeed level controls. `BandListWidget`: one row per
band with type, EN, SOLO, channel, FREQ, BW, GAIN, Q and delete; values
can be typed, dragged or scrolled. `UIEq::removeBand()` fixes the
selection in both widgets. The model update to the DSP is debounced; the
graph redraws every frame from the closed-form response.

## Parametric EQ Mono (`plugin_eq_mono/`, `plugin/dsp/Mono*`)

Feeding a mono signal through the stereo engine would save nothing: the
cost is the O(order²) recursion over the whole order. The mono plugin
assembles one SISO chain (`assembleMonoStateSpace()`, 2 states per band
instead of 4) into `MonoStateSpaceMatrices` (one input column, one output
row, scalar D), run by `MonoStateSpaceEngine` and checked by
`MonoStabilityGuard`, a deliberate copy of the stereo guard's algorithm
(the test suite keeps them in agreement). Measured 2.8–4.4x faster than
the stereo EQ on the same bands (measured before the stereo engine got
its sparse path, so the current gap is smaller); the output equals one
channel of the stereo EQ exactly.

Shared unchanged: `EqModel`, the `.sseqjson` format, `EqGraphWidget` and
`BandListWidget` (no channel column in mono). Export writes a stereo
`.ssjson` with identical channels, since Matrix Filter is stereo. The
band limit is the same 32, set by the cascade builder's 64-state buffer.

## Multiband Compressor (`plugin_mb_compressor/`, `gui/mb/`, `plugin/dsp/MbCompressorModel.*`)

`MbCompressorModel`: ascending crossover frequencies, one `MbBand` per
band (threshold, ratio, up ratio, attack, release, makeup, enabled, solo)
-- always one more band than crossovers, maintained only by
`addCrossover()` / `removeCrossover()` -- plus the crossover slope and
output trim. `assembleStateSpace()` builds the crossover bank and turns
each band into a `NonlinearSchedule` with `classicCompressor()`. Output
trim scales each band's `feedD` as well as `C`/`D`, because the multiband
branch reads `feedD`. A disabled band gets a flat 0 dB curve (audio
passes, no dynamics); a band muted by another's solo gets -100 dB.
`evaluateCombinedResponse()` sums the bands' closed-form responses to
draw the crossover shape.

Editor: `MultibandGraphWidget` (shaded band regions, crossover handles,
split/merge by double-click, scroll for Ratio, the selected band's bipolar
gain/makeup meter from the `kParamBandGainBase` output Parameters) and a
`ParamStripWidget` retargeted at the selected band. Solo follows the
selection. Presets `.ssmbjson`.

## Dynamic EQ (`plugin_dyn_eq/`, `gui/dyneq/`, `plugin/dsp/DynamicEq*`)

Not a state-space mapper: see `docs/EXTENDING.md` for why time-varying
bells need the Regalia-Mitra construction and a dedicated engine.

- **`DynamicEqModel`**: bands with frequency, Q, resting gain, threshold,
  ratio, up ratio, range, attack, release, dynamics on/off, enabled,
  channel. `resolveDynamicGainDb()` is the gain law; the closed-form
  response functions evaluate a band at its resting or its live gain.
  `assembleStateSpace()` (export only) freezes the resting gains as an RBJ
  cascade.
- **`DynamicEqEngine`**: per band and sample, the fixed all-pass and a
  detector band-pass on the cascade signal; the detector envelope
  (linear-domain one-pole, `max(|L|,|R|)`, or one channel for a
  one-channel band) gives the gain; the Regalia-Mitra combination feeds
  the next band. Filter coefficients glide when frequency or Q change;
  band-count changes duck like `StateSpaceEngine`. Live gains are
  published atomically and reach the editor as output Parameters
  (`kParamCurrentGainBase`). Audition outputs the selected band's detector
  signal scaled by its current gain change, switching through a duck.
- **Editor**: `DynEqGraphWidget` (nodes, dynamic-range shading, the live
  combined curve) and one `ParamStripWidget` retargeted at the selected
  band; Solo follows the selection. Presets `.ssdynjson`.

## Dynamic EQ 2 (`plugin_dyn_eq2/`, `gui/dyneq2/`, `plugin/dsp/DynamicEq*2*`)

An independent copy of Dynamic EQ with its own model, engine, editor and
plugin identity, extended with: Low/High Shelf (first-order all-pass) and
Band Shelf (two first-order shelves and a flat gain, see
`docs/EXTENDING.md`); harmonic groups with shared frequency movement and
shared threshold/ratios/range (`propagateHarmonicDynamics()`), auditioned
together; crossfeed bands with per-direction levels. `dsp_regression`
checks that it matches Dynamic EQ exactly where their features overlap.

Its editor follows the Parametric EQ's: `DynEqGraphWidget2` has the same
gestures (click to add, right-click menu, right-button drag to audition
through `beginBandAudition()`/`endBandAudition()`), and
`DynEqBandListWidget2` -- an adapted copy of `BandListWidget` -- shows
every band's settings in one row, the dynamics included. `UIDynEq2`
owns one text-edit state (`EditTarget`) for the list fields, the inline
Output Trim / Smoothing fields and the harmonic custom-factor input.
Solo is one band at a time (`DynamicEqModel2::soloedBand`).

## Known limitations

- No pole/zero display (only a spectral-radius estimate; see
  `docs/EXTENDING.md`).
- No linear-phase mode: every band is a minimum-phase IIR section.
- No per-band transfer curves in the Multiband Compressor, and no mono
  versions of the Multiband Compressor and the Dynamic EQs.
- Training from audio is command-line only.
- One FFT size and smoothing time for both spectrum traces.
