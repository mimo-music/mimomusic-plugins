# Architecture

This document describes the *current* system.

## DSP core (`plugin/dsp/PhaseScopeDsp.hpp`)

Entirely header-only, no DPF/DGL dependency -- the same "small enough that
splitting declaration from definition would add bookkeeping without a
benefit" reasoning `../compressor`'s own `CompressorDsp.hpp` gives for
itself.

```
input L/R --[unmodified copy]--> output L/R
         |
         +--> PeakMeter/LoudnessMeter (L, R independently) --> bargraphs
         +--> StereoImageMeter (shared L/R statistics) --> Correlation, Width
         +--> PeakMeter (raw Mid/Side magnitude) --> kParamMeterPlotPeakDb --> UI computes Gain instantly on click
         +--> rotated Mid/Side --[x manual Gain]--> GoniometerBuffer (fixed write rate) --> vectorscope
                                                  |
                                                  +--> HeatmapGrid (1-30s density map) --> glow layer
```

Six independent, purely-observational pieces, all fed straight from the
input signal in `PluginPhaseScope::run()`. Only the *plotted* Mid/Side
values feeding `GoniometerBuffer`/`HeatmapGrid` are ever scaled (by manual
`kParamDisplayGain` -- see "Auto Gain" below for why nothing else ever
applies a second, separate scale on top) -- every meter reading
(`PeakMeter`/`LoudnessMeter`/`StereoImageMeter`) reads the real, unscaled
signal throughout, so Gain changes nothing about what the
correlation/width/level numbers report, only how big the goniometer draws:

- **`PeakMeter`/`LoudnessMeter`**: the same conventional ballistics
  (~20dB/s peak decay, ~300ms RMS integration) `../compressor`'s own
  `CompressorDsp.hpp` uses -- copied as a design, not literally shared
  code across the two independent repos.
- **`StereoImageMeter`**: tracks three leaky-integrated (one-pole
  smoothed, ~300ms) statistics -- `E[L*R]`, `E[L^2]`, `E[R^2]` -- and
  derives *both* readouts from them:
  - `correlation() = E[L*R] / sqrt(E[L^2] * E[R^2])`, the standard
    Pearson correlation coefficient a hardware correlation meter's analog
    multiplier-plus-smoothing-capacitor implements, exactly in [-1, +1] by
    the Cauchy-Schwarz inequality.
  - `widthPercent()` reuses the *same three sums* rather than a second,
    separate Mid/Side measurement: expanding `Mid=(L+R)/2` and
    `Side=(L-R)/2` shows `E[Mid^2]` and `E[Side^2]` are both already fully
    determined by `E[LR]`/`E[LL]`/`E[RR]`. Reported as
    `200 * sqrt(E[Side^2]) / (sqrt(E[Mid^2]) + sqrt(E[Side^2]))` -- 0% for
    a mono signal (no Side energy at all), 100% for an ordinary
    uncorrelated equal-power stereo signal, up to 200% for a signal that
    is entirely Side (e.g. exactly out-of-phase L/R).
  - A **worst-case hold with a real plateau**: `minHoldCorrelation()` snaps
    down instantly to the lowest correlation seen recently, then sits
    completely flat for a fixed hold time (`kHoldPlateauSeconds`, 1.5s,
    matching `../compressor`'s own `LevelMeterWidget` clip-hold time)
    before recovering back upward at a fixed, much slower rate
    (`kHoldRecoverPerSample_`, correlation-units/second) -- the
    conventional two-stage "peak snaps, holds, then decays" shape a real
    peak meter uses. The plateau isn't cosmetic: without it, the
    worst-value marker just followed the end of the live correlation bar
    -- ordinary program
    material's correlation reading keeps wobbling even after the ~300ms
    smoothing above (it's a real, continuously-moving signal, not a
    static test tone), so a single-stage "snap down, immediately start
    recovering" hold gets re-snapped onto the live value by *any* one-tick
    dip below wherever it's currently recovered to, which for real audio
    happens often enough that the hold and the live reading were
    practically never visibly different. Every new worst value restarts
    the plateau timer (`dsp_regression.cpp` `[6b]` checks this directly:
    the hold must sit exactly unchanged through less than one plateau's
    worth of good-phase signal, then actually move once that time is
    exceeded), so a brief phase problem now stays visibly pinned in place
    long enough to actually be seen as separate from the live reading
    before it starts fading back. There is no manual reset for this hold
    any more -- an earlier "Reset Hold" button/Parameter (a trigger
    calling a now-removed `resetMinHold()`) became unnecessary once the
    plateau itself worked correctly.
- **Auto Gain**: not its own DSP class at all -- just `SmoothedLevelMeter`
  (`plotLevelMeter_`), a one-pole low-pass average of the raw, unscaled
  rotated Mid/Side magnitude with an adjustable time constant
  (`kAutoGainTimeConstantSeconds`, currently `2.5`, raised from an
  original `1.0` for a smoother result), continuously
  exposed as the output Parameter `kParamMeterPlotPeakDb`. Was a plain
  `PeakMeter` (instant attack, ~20dB/s decay -- the same ballistic the
  L/R bargraphs use) at first; that made continuous tracking visibly
  twitchy, snapping Gain up on every individual transient and letting it
  slide back down as the peak decayed. `SmoothedLevelMeter` fixes this by
  construction -- see its own file comment -- but changing what "the
  current level" means semantically (a slow average rather than an
  instant peak) also raises the Gain a steady-state target settles on for
  any signal with real transient dynamics (the average sits below the
  peak, so more gain is needed to bring the *average* up to the same
  target fraction) -- observed in practice as about 8dB too much Gain
  (17dB where about 9dB looked right). `kAutoGainTargetPeak` was retuned
  in response -- see its own comment below for why an 8dB reduction
  there produces exactly an 8dB reduction in Gain for *any* signal, which
  made that a precise, one-line fix rather than a guess. The same tradeoff still applies
  post-retune: raising `kAutoGainTimeConstantSeconds` further trades even
  more smoothness for the average sitting even further below instantaneous
  peak, i.e. still more Gain for the same target -- worth knowing before
  reaching for that constant again.
  An on/off *toggle* (`UIPhaseScope::
  autoGainActive_`, mirrored to the host as State `"autoGainActive"`),
  not a one-shot click: on, Gain is adjusted continuously; off, Gain
  stays at whatever it last settled on. Turning it on calls
  `applyAutoGain()` once immediately (so the effect is visible right
  away) and opens one `editParameter(kParamDisplayGain, true)` touch
  bracket for the whole session; from then on, every fresh
  `kParamMeterPlotPeakDb` update (`UIPhaseScope::parameterChanged()`)
  calls `applyAutoGain()` again while the toggle is on, continuously
  recomputing `gainDb = clamp(linearToDb(kAutoGainTargetPeak) -
  meterPlotPeakDb_, -24, +24)` and writing it to `kParamDisplayGain`.
  Turning it off just closes that same bracket
  (`editParameter(kParamDisplayGain, false)`) and stops the recomputation
  -- Gain is left exactly where it last settled, not reset.
  `kAutoGainTargetPeak` has moved twice: `0.9` initially, halved to
  `0.45` for a better-filled plot, then reduced again to `0.18` -- an 8dB
  drop -- once the switch to a smoothed average (above) made that same
  `0.45` land noticeably hotter than before on real material.

  An earlier version computed and applied this once per click instead of
  continuously, and looked broken even though the arithmetic itself was
  correct throughout (verified in `dsp_regression.cpp` `[9d]`, unchanged
  by this): a one-shot click landing on an
  already-correct Gain value (e.g. on steady program material) looks
  identical to a click doing nothing at all, since there's no way to tell
  "computed the same answer again" apart from "didn't run". The toggle
  removes that ambiguity: an active Auto Gain visibly keeps tracking the
  signal, and the button's own label/color (`refreshAutoGainLabel()`, the
  same on/off convention `btnHeatmap_` already uses) makes the current
  state unambiguous at a glance.

  Two earlier, now-removed designs are still worth knowing about if this
  class of bug resurfaces: a continuously-running AGC-style follower
  (measuring the *Gain-scaled* signal, composing its own live factor on
  top of Gain) drifted upward over repeated clicks (0 -> 10dB, then
  12.6dB, then 14dB) because a one-pole follower only asymptotically
  approaches its target; and a one-shot `GainMeasurement` state machine
  (`Idle -> Measuring -> Held`, requiring `run()` to keep being called
  for a full ~1s window) could get permanently stuck in "Measuring" on a
  host that stops calling `run()` when transport is stopped. Today's design has neither problem: it's a plain, instant
  computation from an always-current `PeakMeter` reading, repeated on
  every update while toggled on -- no follower to drift, no window to get
  stuck in.
- **`HeatmapGrid`**: a long-history (1-30s, `kParamHeatmapDecaySec`)
  density map of the exact same points `GoniometerBuffer`'s own trail
  draws live, over up to 30 seconds. A 32x32 grid of independently
  leaky-integrating cells (`kCellCount = 1024`): `addPoint()` decays every
  cell once, then brightens the one cell the point landed in by a fixed
  `kAddPerHit` (`0.35`, clamped to `1.0`) -- the same "add on a hit, decay
  every tick" one-pole shape `StereoImageMeter`'s own sums use, just per
  spatial cell instead of per statistic. Fed exactly the points
  `GoniometerBuffer::process()` itself just wrote (that function now
  returns whether it did, see its own comment) rather than being
  independently re-decimated, so the heatmap is honestly "the same live
  trail, remembered longer". Because that feed rate is itself fixed
  (`kScopeBufferSize` points per `kMaxDecaySeconds`, sample-rate-
  independent by construction) `HeatmapGrid`'s own per-point decay
  coefficient doesn't depend on sample rate either -- no `prepare
  (sampleRate)` needed, `dsp_regression.cpp` `[9e]` checks the decay
  itself is correctly time-scaled (a shorter requested Decay fades a
  fixed number of points' worth of elapsed time faster than a longer one).
  Packed the same way as `GoniometerBuffer`'s own points but coarser -- a
  cell only needs brightness precision, not the vectorscope's own
  positional precision -- so 4 cells at 6 bits each fit the identical
  24-bit budget instead of 2 values at 12 bits each, `kCellCount / 4 =
  256` Parameters rather than one per cell.

  Cells are addressed in the *plotted* (Gain-scaled) coordinate space,
  the same one `GoniometerBuffer` itself plots in -- which means a
  history built up at one Gain no longer lines up with where the live
  trail draws once Gain changes: the same physical point in the signal
  now maps to a different cell. This has gone through two fixes. First,
  a plain `heatmap_.reset()` on every Gain change -- correct for a
  one-shot manual Gain edit, but once Auto Gain (see above) could nudge
  Gain *continuously*, that meant the heatmap could never actually
  accumulate any history, or fade out on its own Decay's own terms,
  while Auto Gain was active (its spots vanished almost immediately
  instead of fading). `HeatmapGrid::
  rescale(ratio)` (`ratio = dbToLinear(newGainDb - oldGainDb)`) is that
  resampling: every existing cell's own center world coordinate is scaled
  by `ratio` and re-deposited into whichever cell it now belongs in
  (summed, since more than one old cell can land in the same new one when
  zooming out; dropped if scaled outside the plot entirely when zooming
  in), so history survives a Gain change and keeps decaying normally
  instead of being wiped by one. Nearest-cell, not interpolated, so a
  large, sudden Gain jump can look briefly blockier -- `GoniometerWidget::
  drawHeatmap()`'s own draw radius was also enlarged (`0.8` -> `1.1` of a
  cell) to help mask that, alongside the resampling fix, not instead of it. `dsp_regression.cpp` `[9g]`
  checks `rescale()` directly: a ratio of `1.0` is a no-op, a mid-plot hit
  moves cell while the grid's total intensity is conserved, and a
  near-edge hit scaled outside the plot is dropped.
- **`GoniometerBuffer`**: a fixed-size (`kScopeBufferSize = 4096`, doubled
  from an original `2048` for a denser trail) continuously-overwritten ring of packed (x, y) points,
  written at a
  **fixed rate** -- `decimationStride_` is derived once in `prepare()`
  from the real sample rate and the compile-time ratio
  `kScopeBufferSize / kMaxDecaySeconds`, and never changes afterward for
  any other reason than a sample-rate change. An earlier version instead
  kept a *fixed point count* and stretched/shrank the write rate to make
  Decay changes fit that same count into a shorter or longer span --
  which technically worked, but meant a longer Decay setting spread
  exactly the same number of points over more time, i.e. a *sparser*
  trail rather than a longer *and* equally dense one.
  `visibleCountForDecay(decayMs)` is the fix: a pure function (no
  instance state) mapping a requested Decay to how many of the buffer's
  most recently written points should actually be drawn, from 1 up to the
  full `kScopeBufferSize` at `kMaxDecaySeconds` -- since the write rate
  never changes, a bigger Decay genuinely reveals more real points at the
  same density, not the same points spread thinner. `dsp_regression.cpp`
  `[9]`/`[9b]` confirm the write rate itself is fixed and duration-correct
  across sample rates; `[9c]` confirms `visibleCountForDecay()` is
  monotonic and roughly proportional across the whole range.

## Bulk data transport: packed output Parameters

DPF's only guaranteed cross-process channel is a plain float `Parameter`
(the DSP and UI sides can be different processes for some plugin formats/
hosts) -- the identical constraint, and identical solution,
`../statespace`'s own `SpectrumAnalyzer.hpp` documents for its FFT bands.
`GoniometerBuffer::packPoint()`/`unpackPoint()` quantize one (x, y) point
to 12 bits per axis and pack both into a single float via plain integer
arithmetic (24 bits total) -- deliberately capped there because a
`float`'s 24-bit mantissa represents every integer up to 2^24 exactly, so
the packed value round-trips with zero error beyond the quantization step
itself.

**A real bug was found and fixed here** (by the pack/unpack test):
`unpackPoint()` rounds
via `std::lround(packed)`, not the more obvious "`uint32_t(packed +
0.5f)`" -- at magnitudes near 2^23-2^24 a float's representable integers
are already exactly 1 apart, so adding a further `+0.5` can itself round
to the *next* integer (round-half-to-even) before truncation, corrupting
the packed bit pattern by 1 at exactly the worst possible bit position (a
carry across the two 12-bit fields, flipping both at once).
`dsp_regression.cpp` `[7]` round-trips a grid of points specifically to
catch this class of bug.

`kParamScopeWriteIndex` carries the ring buffer's current write slot
*modulo `kScopeBufferSize`* (0..4095), not the raw ever-growing sample
counter -- the UI only ever needs "which slot is newest" to derive each
point's fade-age, and reducing mod the buffer size keeps this an exact,
host-safe small integer indefinitely rather than eventually exceeding a
float's exact-integer range during a long session.

## Plugin side (`plugin_phasescope/`)

An ordinary DPF `Plugin` subclass with **no controls that shape the
audio** -- every `Parameter` is either an output meter reading (including
`kParamMeterPlotPeakDb`, the raw Mid/Side peak Auto Gain reads) or a
*display*-only control (`kParamDisplayGain`, `kParamHeatmapDecaySec`),
because this plugin never touches the signal itself (see
`PluginPhaseScope.cpp`'s `run()`: input is copied straight to output
before any measurement runs). There is no trigger Parameter at all any
more -- the one that existed (`kParamResetHold`) was removed along with
the "Reset Hold" button it backed (see `StereoImageMeter`'s own entry
above for why).
`run()`'s per-sample body: feed `PeakMeter`/`LoudnessMeter` (L, R
independently) and `StereoImageMeter` from the *real, unscaled* signal;
compute rotated Mid/Side (`side=(L-R)/2, mid=(L+R)/2` -- the same
transform `../compressor`'s own Mid/Side stereo mode uses, algebraically
lossless); feed that same raw magnitude to `plotLevelMeter_` (backing
`kParamMeterPlotPeakDb`); scale Mid/Side by manual Gain and feed that to
`GoniometerBuffer`; whenever that call reports it actually wrote a new
point, feed the same scaled coordinates to `HeatmapGrid::addPoint()` too.

Five `State`s -- `kStateDrawLines` ("drawLines"), `kStateDecayMs`
("decayMs"), `kStateResolutionPercent` ("resolutionPercent"),
`kStateSmoothLines` ("smoothLines") and `kStateHeatmapEnabled`
("heatmapEnabled") -- all pure rendering preferences (see the GUI section
below) the DSP side never reads at all (`setState()` no-ops for all
five), the same "nothing for the DSP side to do" convention
`../compressor`'s own "presetDir" State uses. Decay moved here from being
a Parameter (an earlier version of `GoniometerBuffer` let it change the
write rate, making it genuinely DSP-relevant) once the write-rate
redesign above made it a pure UI-side windowing choice with zero effect
on the DSP side at all; Resolution, Smooth and Heatmap-the-toggle never
had any DSP-side effect to begin with (all three are purely how
`GoniometerWidget` chooses to render whatever the DSP side already
handed it, `HeatmapGrid` itself always accumulating regardless of whether
its own display toggle is on) and so landed directly as States rather
than passing through a Parameter phase first -- the same "continuous and
DSP-relevant -> Parameter, else -> State" reasoning this project's
controls already follow. Heatmap's own *Decay*, by contrast, genuinely
changes what `HeatmapGrid` computes, so it's a real Parameter
(`kParamHeatmapDecaySec`), the same reasoning the goniometer's own Decay
used to follow before its redesign.

## GUI (`gui/`)

- **`SimpleButton`**, **`LevelMeterWidget`**, **`ParamStripWidget`**:
  originally copied verbatim from `../compressor/gui/`; `SimpleButton` and
  `ParamStripWidget` have since each gained a small, additive
  `setTooltip()`/`tooltip` field (see `TooltipOverlay`'s own entry below)
  -- their click/hover/drag mechanics are otherwise still identical.
  `ParamStripWidget` backs the Gain/Decay/Resolution/Heatmap-Decay fields
  (click-to-edit,
  click-and-drag, scroll-to-nudge, Alt+scroll for 10x finer -- the full
  existing convention, see that file's own header comment), the first
  fields this project actually needed once Gain/Decay became real,
  continuous controls. Decay and Resolution's own fields use
  `UIPhaseScope::kFieldDecayMs`/`kFieldResolution`, sentinel indices that
  are *not* real DPF Parameters -- `ParamStripWidget` itself doesn't care
  what a field's index actually means, it just forwards to whatever
  getter/setter callback the owner supplied, so `UIPhaseScope`'s own
  callbacks special-case those two sentinels to read/write
  `decayMs_`/`resolutionPercent_` and call `setState()` instead of
  `setParameterValue()`; Gain and Heatmap Decay need no such sentinel,
  since both are real Parameters and go through the ordinary
  `paramCache_`/`setParameterValue()` path every ordinary field uses.
  `ParamStripWidget`'s own header comment says keyboard events reach the
  top-level `UI`, not the strip subwidget itself -- `UIPhaseScope` didn't
  actually implement that forwarding when the strip was first added (no
  `onCharacterInput()`/`onKeyboard()` overrides at all), so a click-to-edit
  field could be entered but never typed into. Fixed by adding both overrides, mirroring
  `UILimiter`'s own identical pair exactly (forward to
  `strip_->appendChar()`/`commitEdit()`/`cancelEdit()`/`backspace()` while
  `strip_->isEditing()`).
- **`GoniometerWidget`**: `collectVisiblePoints()` runs once per repaint,
  shared by both draw modes, and does three things in one pass: walks
  back from the newest ring-buffer slot for `visibleCount_` points (Decay's
  own window, via `GoniometerBuffer::visibleCountForDecay()`), thins that
  walk to every `round(100/resolutionPercent_)`-th point (Resolution --
  `setResolutionPercent()` -- a pure draw-call-count throttle with no
  effect on `visibleCount_`, `GoniometerBuffer`, or anything else DSP-side
  at all), and maps each kept point straight to screen coordinates,
  stashing it alongside its original age (for the fade) in scratch member
  arrays (`ptSx_`/`ptSy_`/`ptAge_`). Both age and the alpha fade-out are
  normalized against `visibleCount_`, not the buffer's fixed
  `kBufferSize`, so the trail fades over the same *proportion* of itself
  regardless of how much of it Decay is currently showing -- normalizing
  against the full buffer instead would make a short Decay's trail barely
  fade at all within its own (much shorter) visible window.
  `setDrawLines(bool)` switches between `drawTrailAsDots()` (the fading
  dot cloud, unaffected by Smooth -- there's no "curve" between discrete
  dots) and `drawTrailAsLines()` (segments between consecutive *kept*
  points -- not necessarily adjacent ring-buffer slots any more once
  Resolution has thinned them -- a continuous vector trace, the other
  common goniometer rendering style, closer to what a real analog X-Y
  oscilloscope's continuously-moving beam draws); both share the same
  per-point age/alpha and correlation-tinted color logic. `setSmoothLines
  (bool)` (Draw Lines mode only) switches `drawTrailAsLines()`'s own
  segments from a plain `lineTo()` to a Catmull-Rom-to-Bezier `bezierTo()`
  construction through each point's clamped neighbors -- the standard way
  to turn a sampled point sequence into one smooth interpolating curve
  (the same *kind* of technique, though a different, unconstrained
  variant of it, `../compressor`'s own Contour plugin already uses for its
  monotone cubic curve-editing spline), built directly in the
  already-screen-mapped coordinates `collectVisiblePoints()` produced
  (valid because the side/mid -> screen mapping is affine, so the
  construction commutes with it). The M/L/R axis labels sit just outside
  the circle's own top half now (M straight up, L/R at the upper ends of
  their own diagonals) rather than L/R at the bottom -- with
  `side=(L-R)/2` on x and `mid=(L+R)/2` on y, a hard-Left-only signal
  traces the upper-left <-> lower-right diagonal and a hard-Right-only one
  traces the upper-right <-> lower-left diagonal (derived directly in the
  file's own comment), so the bottom corners were never even the right
  *corner* for L/R to begin with, an actual mislabeling an earlier version
  had, not only a matter of where on the circle to put the text.
  `drawHeatmap()` draws `HeatmapGrid`'s own cells as soft, slightly
  overlapping circles (not hard-edged squares, which is what actually
  produces the "glow" look), lerping through four *dark* stops as
  intensity rises -- dark blue, dark green, dark yellow, dark red -- with
  every channel value capped under 100 so even the hottest cell stays a
  dark tint, never a bright color (nudged up from an initial under-60 cap
  that looked a little too dark -- still a dark tint throughout, just a
  lighter one, and
  `kMaxAlpha` raised alongside it, `0.65` -> `0.7`, since a darker color
  needs more opacity to actually read as that color against the plot's
  own near-black background). Two earlier gradients both failed the same
  way despite fixing the previous problem each time: navy -> cyan ->
  pale warm white made the live green trail nearly disappear (cyan and
  pale white sit close enough to green in perceived hue/brightness to
  compete with it); the fix, black -> dark red -> bright orange, still
  made the trail hard to see, because it climbed to a fully
  saturated (255,140,20) at full intensity -- just as bright as the live
  trail itself, only a different hue. Capping *every* stop's brightness,
  not just picking a hue that avoids green, is what actually fixes this.
  "Fading to
  transparent" at low intensity is handled by alpha (`intensity *
  kMaxAlpha`), not by the color ramp approaching black on its own -- that
  keeps the hue distinctions between stops legible right down to the low
  end, where the RGB values alone would otherwise wash out first. Drawn
  first, right after the plain background fill and before the reference
  grid/axis lines/labels/live trail all draw on top of it, so all of
  those stay fully legible regardless of how bright the heatmap gets.
- **`StereoMeterWidget`**: two stacked bargraphs (Correlation, Width), no
  DSP dependency beyond the plain float readings `UIPhaseScope` hands it.
- **`TooltipOverlay`**: one full-window, click-through `NanoSubWidget`
  added last (so it draws on top of every other widget), showing a short
  hover description near whatever control the pointer is currently over.
  It has no hover logic of its own: `UIPhaseScope::
  updateTooltip()` (called once per `uiIdle()` tick, alongside the
  existing scope/heatmap batching) linearly checks each `SimpleButton`'s
  own `isHovered()`/`tooltip()` and `ParamStripWidget::hoveredTooltipField
  ()`/`tooltipFor()` (suppressed while that field is actively being
  edited or dragged) and calls this widget's `show(text, anchorRect)`/
  `hide()` accordingly -- a plain poll rather than a hover-changed event,
  cheap enough at six buttons and four fields that it needs no more
  machinery than that. `onMouse()`/`onMotion()`/`onScroll()` all
  unconditionally return `false` despite this widget covering the entire
  window, otherwise it would swallow every click meant for whatever's
  underneath it.
- **`UIPhaseScope`**: batches both the scope-packet and the heatmap-cell-
  packet `parameterChanged()` pushes into local arrays (`scopeDirty_`/
  `heatmapDirty_` flags) and hands each whole batch to
  `GoniometerWidget::setScope()`/`setHeatmap()` once per `uiIdle()` tick,
  rather than triggering hundreds of separate repaints per host
  UI-refresh cycle. **Freeze** is pure UI-local state -- an early return
  in `parameterChanged()` while frozen -- needing no Parameter or State
  round trip at all, the same "click the meter to clear its clip LED"
  convention `LevelMeterWidget` already established for widget-local,
  non-automatable interactions. That early return is placed *after* the
  Gain/Heatmap-Decay/plot-peak settings-mirror cases, not before them --
  Freeze is meant to pause only the meter/scope/heatmap redraw, not make
  an automated Gain change appear to silently not take effect. **Auto
  Gain**'s own `kParamMeterPlotPeakDb` case in `parameterChanged()` does
  double duty: it always mirrors `meterPlotPeakDb_`, and while
  `autoGainActive_` is true it also calls `applyAutoGain()` right there,
  which is what makes Auto Gain track the signal continuously rather than
  only on click (see "Auto Gain"'s own entry above for the full toggle
  design). **Draw Lines**, **Decay**, **Resolution**, **Smooth**,
  **Heatmap**, and **Auto Gain**'s own on/off flag, unlike Freeze,
  round-trip through their own States (`setState()`/`stateChanged()`)
  since all six
  are lasting preferences worth a host remembering across a session, not
  a momentary "pause to look at this" action. Smooth's own button is
  grayed out (`SimpleButton::setEnabled(false)`) rather than hidden while
  Draw: Dots is selected, the same "gray out what doesn't currently
  apply, don't make the setting disappear" convention `../compressor`'s
  own True Peak mode uses for its now-inert Attack/Knee fields.
  **Auto Gain's own button is deliberately never disabled** -- an earlier
  version grayed it out while `Measuring`, re-enabling it only once the
  `Measuring -> Held` notification arrived back from the host; if that one
  notification is ever missed, delayed, or the host simply doesn't poll an
  output Parameter that only changes twice per measurement promptly
  enough, the button was left permanently stuck. Leaving it always clickable means a click always at
  least restarts a fresh measurement regardless of what the label
  currently shows, so this feature can no longer wedge itself into a dead
  end the way disabling it could.

## Known limitations

`StereoImageMeter`'s correlation/width are wideband (the full signal, no
frequency breakdown) -- a per-band correlation reading is a plausible
future refinement if a single number turns out to hide frequency-
dependent phase issues in practice.

**Draw: Lines mode connects *decimated* points**, not the full-bandwidth
signal a real analog oscilloscope's continuously-moving beam would trace
-- consecutive buffered points are always `kMaxDecaySeconds /
kScopeBufferSize` apart in time (~0.73ms at the current `4096`-point
sizing, at any sample rate, and *independent of the current Decay
setting* -- see `GoniometerBuffer`'s own comment on why the write rate is
fixed). **Smooth** (`GoniometerWidget::setSmoothLines()`) draws a
Catmull-Rom spline through whatever points are actually drawn instead of
straight segments, which reads as visibly smoother, but it is still an
interpolation *through the same underlying samples* -- it cannot recover
detail the buffer's own fixed write rate never captured in the first
place, only avoid the faceted look straight segments would otherwise show
between them. Increasing `kScopeBufferSize` further (at the cost of
proportionally more output Parameters, see "Bulk data transport" above --
already doubled once, from `2048` to `4096`) is the only
way to close the underlying sampling gap itself.

**Resolution** (`GoniometerWidget::setResolutionPercent()`) is a pure
rendering-cost throttle, not a data-quality one in the other direction:
it never *adds* points beyond what the buffer already holds, only skips
some of them at draw time, so raising it back to 100% always recovers the
full trail with nothing lost -- unlike `kScopeBufferSize`, which
determines the *ceiling* Resolution can ever reveal.

# Loudness Meter

The project's second plugin, modeled on Youlean Loudness Meter 2's own
set of readings, units and layout. Same overall shape as Phase Scope: purely
observational, no controls that touch the audio, one shared `gui/`
directory.

## DSP core (`plugin/dsp/LoudnessDsp.hpp`)

Standards-based (ITU-R BS.1770-4 / EBU R128 / EBU Tech 3342) loudness
measurement, entirely header-only like `PhaseScopeDsp.hpp`, five pieces:

```
input L/R --[unmodified copy]--> output L/R
         |
         +--> KWeightingFilter (L, R) --> SlidingMeanSquare (400ms, 3s) --> Momentary/Short-term LUFS
         |                                          |
         |                                          +--(sampled every 100ms)--> LoudnessHistogram (x2) --> Integrated LUFS, LRA
         +--> TruePeakDetector (L, R) --> True Peak dBTP, PLR, PSR
```

- **`KWeightingFilter`**: two cascaded biquads (a high-frequency shelf,
  then a high-pass) implementing the standard's own "K-weighting"
  pre-filter -- every loudness number below is computed from *this*
  filtered signal, not the raw one, which is what makes an LUFS reading
  track perceived loudness at all rather than being a plain RMS meter
  with extra steps. Coefficients are computed from the standard's own
  analog-prototype parameters (center frequency, Q, and for the shelf, a
  gain) via the standard RBJ Audio EQ Cookbook bilinear-transform
  formulas, at whatever the *actual* sample rate is -- not read from a
  48kHz-only coefficient table the way the standard's own informative
  annex tabulates them, so this stays correct at 44.1kHz, 96kHz, or
  anything else a host runs at. `dsp_regression.cpp` `[14]` checks the
  filter's own shape qualitatively (well below the ~38Hz high-pass corner
  is heavily attenuated, comfortably above the ~1682Hz shelf corner is
  boosted).
- **`SlidingMeanSquare`**: an exact rectangular time window (a real
  ring-buffer running sum, add-new/subtract-oldest per sample), not an
  exponential-decay ballistic like `PhaseScopeDsp.hpp`'s own
  `LoudnessMeter` -- Momentary (400ms) and Short-term (3s) are both
  literal time windows in the standard, not a decaying average, so
  reusing that class (and its different name, hence `LufsMeter` rather
  than a second `LoudnessMeter` in the same namespace) would have been
  the wrong shape entirely, not just an inconvenient naming clash.
  `dsp_regression.cpp` `[12]` checks a constant pushed value settles to
  its own exact mean, both partially and fully filled.
- **`LoudnessHistogram`**: both Integrated Loudness (the standard's own
  two-stage absolute + relative gate) and Loudness Range (EBU Tech
  3342's percentile statistics over a relative+absolute-gated
  distribution) reduce to "the mean power of every gating block whose own
  loudness clears some threshold" or "the Nth percentile position within
  that gated set". Critically, this histogram stores *summed power*
  (`sumZ`) per 0.1 LU bin, not summed dB -- loudness in dB cannot be
  correctly averaged by taking the arithmetic mean of dB values (power
  must be averaged in the linear domain, then converted back), so a
  histogram keyed by bin but storing power is what makes the running mean
  exact regardless of how many blocks have ever been measured, without
  storing every block a long "Integrated" pass has ever seen. One
  instance per statistic: `integratedHistogram_` is fed 400ms blocks with
  a -10 LU relative gate (ITU-R BS.1770-4's own Integrated Loudness);
  `shortTermHistogram_` is fed 3s (short-term) blocks with a -20 LU
  relative gate (EBU Tech 3342's own LRA), `percentile(0.95, -20) -
  percentile(0.10, -20)`. `dsp_regression.cpp` `[13]` checks hand-
  computable cases directly: a perfectly steady distribution must gate to
  its own exact loudness with an LRA of exactly 0 LU (nothing for a
  relative gate to exclude, no range in a distribution with only one
  value), and a single block below the -70 LUFS absolute gate must be
  excluded entirely.
- **`DistributionHistogram`**: a plain tally (not energy-weighted like
  `LoudnessHistogram` above) of how many Short-term readings landed in
  each of 60 1-LU-wide bins spanning -60..0 LUFS, backing the UI's
  "statistical view" (as in Youlean). Fed once per 100ms
  gating block, the same cadence everything else here uses; `reset()`
  clears every bin. `dsp_regression.cpp` `[18]` checks a sample lands in
  exactly the bin its own value implies, an out-of-range sample clamps
  into the nearest end bin (never dropped or wrapped), and a steady tone
  processed through the full `LufsMeter` pipeline concentrates almost
  entirely in one bin (a small allowance for the K-weighting filter's own
  brief startup transient) rather than spreading across many.
- **`TruePeakDetector`**: a 4x-oversampling windowed-sinc interpolator
  (Blackman window, `kHalfTaps = 8` samples of symmetric support per
  phase) catching inter-sample peaks a plain sample-peak reading can
  miss. Deliberately a general-purpose design, **not** a bit-exact
  reproduction of ITU-R BS.1770 Annex 2's own reference filter -- a real
  certification tool would need to match that filter exactly; a
  monitoring meter just needs to reliably catch and report genuine
  inter-sample energy, which a correctly-designed oversampling
  interpolator does regardless of its exact tap count/window choice.
  Normalized to exactly unity gain at DC per phase (`designFilter()`'s
  own tap-sum division) so a constant input reads back as *exactly*
  itself (`dsp_regression.cpp` `[15]`, warmed up past the initial
  step-transient before measuring -- see that test's own comment for why
  a step *into* DC legitimately rings first); `[16]` checks a
  Nyquist-frequency (+1/-1 alternating) square wave's true peak is at or
  above its own 1.0 sample peak within a sane bound, the classic
  Gibbs-ringing case real true-peak meters exist to catch. Has
  `kHalfTaps` samples of reporting latency, inherent to any
  symmetric-support interpolation filter -- harmless here since nothing
  in this plugin's own audio path depends on it (input is always copied
  straight to output, see `PluginLoudnessMeter.cpp`'s own `run()`).
- **`LufsMeter`**: ties the above together at the fixed 100ms "gating
  block" cadence the standard specifies. `SlidingMeanSquare`'s own
  continuously-updated Momentary/Short-term windows, sampled once every
  100ms, *are* the sequence of gating blocks the standard describes -- no
  separate block-accumulation mechanism exists, since a 100ms-stepped
  sample of an exact 400ms/3s sliding window is exactly what "400ms
  blocks with 75% overlap" / "3s blocks with 100ms hop" already mean.
  `resetStatistics()` clears the Integrated/LRA/Distribution histograms
  and the True-Peak-max readings without touching the live filter/window
  state, backing the UI's "Reset" button -- the same "clear accumulated
  stats,
  not live readings" contract `PhaseScopeDsp.hpp`'s own
  `resetMinHold()` used to have. `dsp_regression.cpp` `[17]` exercises the full pipeline
  end to end: a steady full-amplitude tone must settle to closely
  agreeing Momentary/Short-term readings with an LRA of essentially 0 LU
  (a perfectly steady loudness has no range to report -- the same
  exactness `[13]` already checks at the histogram level alone), and
  `resetStatistics()` must reset Integrated back to "nothing measured"
  while leaving the live Momentary reading exactly where it was.

## Plugin side (`plugin_loudness/`)

An ordinary DPF `Plugin` subclass, same "no controls that shape the
audio" shape as `plugin_phasescope/` -- every `Parameter` is an output
loudness reading (`kParamMomentaryLufs` through `kParamPsrLu`), a plain
per-channel dBFS reading (`kParamInputRmsL` through `kParamInputPeakR`,
backing the L/R bars -- see the GUI section below), or one of two banks
of 60 unpacked Distribution bin counts: `kParamDistBinBase + [0..60)` for
Short-term, `kParamDistBinMomentaryBase + [0..60)` for Momentary --
`ssmeter::LufsMeter` now keeps two independent
`DistributionHistogram` tallies (see its own comment) rather than one
shared one, "do me two different distribution plots. one with the short
term and one with momentary values". Still unpacked (unlike
`plugin_phasescope`'s own goniometer/heatmap banks), since 120 total
Parameters needs no bit-packing scheme either. "Reset" is **not** a
Parameter at all any more -- see `kStateResetRequest`'s own comment in
`PluginLoudnessMeter.hpp` and the "Reset: the real fix" paragraph below
for why it moved to a State push. Each Distribution bin's own declared range was originally
`0..1e9` ("a plain, unbounded-in-practice count"), which is bad practice
regardless of whether it explains any specific reported symptom: some
hosts scale an output Parameter's own change-significance handling
against its declared range, and a `+1` count against a billion-wide range
is about as insignificant a relative change as a float can express.
Lowered to `0..5000` -- a real ceiling (~8 minutes of 100ms gating blocks
concentrated in one bin, already an extreme case for real material), not
a token "unbounded" one. On "The Reset button is somehow not working. I
see no affect on any numbers": this range fix is the one concrete,
verifiable correctness issue found on inspection, and may well be behind
it if the Distribution view happened to be what was being watched at the
time; a second, independent explanation was reasoned through but not
verified -- Reset deliberately leaves Momentary/Short-term alone (see
`ssmeter::LufsMeter::resetStatistics()`'s own comment), and the
statistics it *does* clear (Integrated, Range, True Peak Max) are
recomputed live from ongoing audio every block, so on steady material
they can resettle to a value very close to what they read before within
a second or two -- which without close attention can look identical to
the button doing nothing. `setResetFlash()` (see the GUI section) is a
direct, unconditional response to that second possibility regardless of
which explanation (or both) turns out to be the real one: an
unmistakable visual confirmation that the click itself registered, that
doesn't depend on how much the underlying numbers actually end up
moving.

**Reset: the real fix.** Even with the flash above proving every click
registers at the UI, the numbers still never moved at all -- not even the momentary dip `resetStatistics()` clearing the
Integrated/LRA/True-Peak-max histograms to empty should produce before
they climb back up. That rules out "resettles too fast to notice" and
points at the trigger never reaching the DSP side at all. The root
cause: `kParamReset` was a `kParameterIsTrigger` Parameter, and for a
format like LV2, DPF backs every Parameter with a plain control-port
float the host owns -- `DistrhoPluginLV2.cpp`'s own `lv2_run()` only
calls `Plugin::setParameterValue()` when that port's value actually
*differs* from the last one DPF itself observed
(`d_isNotEqual(fLastControlValues[i], curValue)`), and relies entirely on
the host to snap a trigger port back to its declared default afterward
(DPF's own comment: "NOTE: host is responsible for auto-updating control
port buffers"). A host that doesn't do this leaves the port pinned at
`1.0` forever after the first successful click -- every later click
writes that same `1.0` again, DPF's own equality check sees no change,
and `Plugin::setParameterValue()` is never called again. This plugin's
own `plugin_phasescope` sibling hit the same class of bug with its old
Auto Gain "Measure" trigger and ultimately routed around it by removing
the trigger Parameter entirely in favor of a continuous State-backed
toggle rather than ever confirming
the host-side mechanism -- the same "don't guess at unverifiable host
internals, make it correct by construction" approach is used here.
`kParamReset` is gone; `kStateResetRequest` (State key `"resetRequest"`,
`kStateIsOnlyForDSP`) takes its place, pushed via `UI::setState()` from
`btnReset_`'s callback with an incrementing counter string. Checked
directly against DPF's own source for every backend
(`dpf/distrho/src/DistrhoUI.cpp`, `DistrhoPluginLV2.cpp`,
`DistrhoPluginInternal.hpp`) rather than assumed: a State push has no
equality gate anywhere in that dispatch chain -- every call reaches
`Plugin::setState()` as its own discrete event regardless of whether the
value repeats, which is exactly the "always fires" guarantee this button
needs and a trigger Parameter could never actually provide on a host that
doesn't cooperate. `run()`'s body: copy input straight to output, then per sample
feed `LufsMeter::process(l, r)` (the K-weighted/gated pipeline) *and* a
plain `ssmeter::PeakMeter`/`LoudnessMeter` pair per channel (the same
ballistic classes `plugin_phasescope`'s own L/R bargraphs use, reused
here verbatim for a plain, non-K-weighted dBFS reading), mirroring all of
it into plain `float` members `getParameterValue()` returns. "Target"
(the reference line) and "showDistribution" (history graph vs.
Distribution view) are both States (`kStateTargetLufs`, string-valued:
`"off"` or a LUFS number like `"-23.0"`; `kStateShowDistribution`,
`"0"`/`"1"`) rather than Parameters, the same "pure UI preference with a
lasting value worth a host remembering" reasoning
`PluginPhaseScope.hpp`'s own `kStateHeatmapEnabled` uses -- the DSP side
never reads either one.

## GUI (`gui/`, Loudness Meter side)

- **`LevelMeterWidget`**: reused verbatim from `../plugin_phasescope`'s
  own `gui/` -- a plain per-channel dBFS peak/RMS bargraph, driven by the
  new plain (non-K-weighted) `kParamInputRms{L,R}`/`kParamInputPeak{L,R}`
  Parameters. A separate sibling widget in `UILoudnessMeter`, not part of
  `LoudnessMeterWidget` -- "do a vertical bar for l/r separate" is a
  literally different reading (plain dBFS, not LUFS) from anything
  `LoudnessMeterWidget` itself computes, so reusing the existing,
  already-correct widget for it was more direct than teaching
  `LoudnessMeterWidget` a second unrelated unit system.
- **`LoudnessMeterWidget`**: the LUFS-specific display, still one widget
  the same "one widget owns the whole visualization" shape
  `GoniometerWidget` uses for Phase Scope, now with a fourth region and
  richer content in two of the original three:
  - The **bar meter**: Momentary as a green-to-red filled bar (the same
    gradient convention `LevelMeterWidget`'s own RMS bar uses); Short-term
    as a triangular pointer on the bar's own right edge; and -- new --
    Short-term and Integrated *again*, each as a short horizontal bar at
    their own height on the exact same LUFS scale, extending further
    right than the triangle ("you can plot the shortterm, integrated
    loudness also in horizontal bar mapped to the scale of the other
    loudness-y-axis") -- a more direct, more comparable representation
    than the triangle pointer alone, not a replacement for it.
  - The **center panel** is now either the history graph or the
    Distribution view, toggled by `setShowDistribution()`/the UI's own
    "View" button. The **history graph** itself gained two things: a
    Catmull-Rom-to-Bezier interpolating spline through the plotted points
    instead of straight segments (`strokeSpline()`, the exact same
    construction `GoniometerWidget::drawTrailAsLines()` uses for Phase
    Scope's own "Smooth" option -- "Do a spline interpolation on the
    points not linear"), and a translucent fill of the area under each
    curve down to the panel's own bottom edge ("fill the space under the
    lines with color. They can be transparent"). Short-term draws first
    (background, `fillAlpha = 0.22`), Momentary second (foreground,
    `fillAlpha = 0.16`) -- deliberately the *lower* of the two alphas,
    specifically so Short-term's own curve/fill stays visible underneath
    it rather than being covered ("at least the one in the foreground
    need to be [transparent]").

    The **Distribution view** went through three iterations before
    settling on its current shape. First, **two** stacked plots, one per
    curve (Short-term and Momentary) -- reasoned as necessary because
    overlaying them the way the history graph overlays its own two curves
    would bury one under the other: Momentary's own distribution is far
    more volatile (a much wider spread of instantaneous readings) than
    Short-term's smoother one. Then, still two stacked boxes but sharing
    one count scale between them, computed once (`sharedMaxCount`, the tallest bin across
    *both* curves) rather than each box normalizing against its own
    tallest bin alone, so a genuinely taller peak in one plot read as
    taller than the other's. Finally, since that still read as "two axes"
    regardless, one plot with both curves overlapping -- which settled the
    actual design question:
    the two-panels reasoning above wasn't wrong about Momentary being more
    volatile, but the fix for that turned out to be the *shared scale*
    (already done), not separate boxes; one shared box, both curves drawn
    into it, layered the exact same way `drawHistoryGraph()` already
    layers its own overlaid Momentary/Short-term curves (Short-term first
    as the background at a lower `fillAlpha`, Momentary second in the
    foreground, so the background curve/fill stays visible underneath).
    `drawDistribution(x, y, w, h)` now takes no per-curve arguments at all
    -- it reads both of the widget's own `distributionCountsShortTerm_`/
    `distributionCountsMomentary_` arrays directly and draws both, via a
    local `drawCurve` lambda shared by both calls. Each still plots
    `ssmeter::DistributionHistogram`'s own bin counts as a smooth, filled
    curve through each bin's own center -- `strokeSpline()` again, the
    same construction the history graph's own curves use -- rather than a
    plain bar chart (a first version *was* a bar chart; a density plot was
    wanted, and a bar chart and a density plot are visually different
    things even though both can show the same underlying distribution).
    One **grid** behind
    both curves now -- vertical lines every 10 LUFS across the fixed
    -60..0 span, labeled along the bottom, the same "fixed steps" idea
    `drawHistoryGraph()`'s own horizontal gridlines use, just rotated
    since this plot's x-axis is LUFS rather than time.

    A real rendering bug surfaced while chasing the shared-scale change,
    not a tuning nitpick: normalizing a curve so its tallest bin maps to
    exactly `1.0` puts that point at the very top pixel of the plot box,
    so a bin that concentrates enough samples (any sustained steady
    passage) visually collides with the box's own top border. First
    fixed with reserved headroom (`titleGap` below the legend text at the
    top, `peakGap` above that) that the curve's own normalization maps its
    0..1 range into -- bounding where the tallest *data point* sits. That
    wasn't the whole fix, though: it could still overlap, because a fixed
    headroom number only bounds where a
    *point* is placed, not where the *curve* `strokeSpline()` draws
    through it can actually reach -- a Catmull-Rom spline can overshoot
    past a point's own value on its way to/from a neighbor, and a sharp,
    narrow peak (steady material concentrated in one bin surrounded by
    much lower neighbors) is exactly the shape that triggers it, by an
    amount no fixed headroom constant can reliably bound for arbitrary
    real distribution shapes. Fixed properly with a hard `scissor()` clip
    to the plot's own interior around each curve's `strokeSpline()` call
    -- the same "unpredictable fill, clip it to the box" convention
    `drawBarMeter()` already uses for its own gradient fill -- so the
    curve is now geometrically incapable of crossing the border regardless
    of how far a given segment's spline overshoots. The headroom itself
    stays (clipping the curve is orthogonal to, not a replacement for,
    keeping the peak marker/label reliably clear of the border and the
    legend).

    **The actual root cause, addressed directly.** The scissor clip
    stops overshoot from becoming visible *at the border specifically*,
    but doesn't stop the overshoot itself -- reported next as "runge
    effects" elsewhere in the curve (the standard name for an
    unconstrained high-order interpolant oscillating past its own data
    around a sharp change) and, separately, the rendered peak sometimes
    not lining up with the actual peak bin -- both the same underlying
    cause the border-clipping only ever treated one symptom of. Fixed at
    the source: `strokeMonotoneSpline()` (`LoudnessMeterWidget.cpp`), a
    Fritsch-Carlson monotone cubic Hermite spline, replaces
    `strokeSpline()`'s Catmull-Rom construction for this curve
    specifically (the history graph's own curves keep using
    `strokeSpline()` -- Catmull-Rom is the right tool for a naturally
    smooth, continuously-varying trace; the Distribution histogram's own
    bin counts are the wrong kind of data for it, blocky and prone to
    sharp bin-to-bin changes whenever material concentrates in one or two
    bins). Fritsch-Carlson forces each interior tangent to exactly zero
    whenever its two neighboring secant slopes disagree in sign or either
    is zero (precisely the condition at a local extremum -- a bin taller
    than both neighbors has a rising slope on one side, falling on the
    other), then rescales tangents per-interval so the curve is
    *guaranteed*, not just typically, to stay within `[min(y_i,
    y_{i+1}), max(y_i, y_{i+1})]` on every single interval -- verified
    directly with a standalone check (evaluating the resulting Bezier
    segments at 200 points each against both a sharp isolated spike and a
    jagged/noisy dataset; the interpolated curve's own global max/min
    matched the raw data's max/min exactly in both cases, not merely
    approximately) before trusting the algebra by inspection alone. A
    curve that can never rise above its own tallest data point can also
    never have its true maximum sit anywhere but *at* that data point,
    which is "clamp the peak" as an actual mathematical guarantee rather
    than a pixel-level patch -- the border-overlap symptom the earlier
    entries chased is now also just a specific case of this same
    property, not a separately-patched one (the scissor clip stays anyway,
    since it costs nothing and is still a second, independent line of
    defense against a headroom miscalculation elsewhere).

    `drawCurve` also tracks each curve's own peak bin (the mode -- whichever loudness
    value was hit most often) and marks it directly: a small filled dot at
    the peak point plus its own bin-center LUFS value as a text label --
    which is exactly the headroom the scaling fix above exists to make room for.
    Momentary's own label sits above its dot, Short-term's below, since
    both curves' peaks now share one box and often land close together in
    LUFS; putting the two labels on opposite sides of their own dots keeps
    them from typically overlapping each other. With no per-curve title
    left to lean on now that both share one box, a small two-color text
    legend ("Momentary" / "Short-term", each in its own curve's color) sits
    in the same top corners the old per-panel titles used to occupy.

    The streaming-platform reference lines (Spotify -14 LUFS, Apple Music
    -16, YouTube -14, a SoundCloud figure at -9 flagged `*`) used to be
    drawn here as permanent lines regardless of the selected Target --
    unwanted clutter. Removed from the widget entirely and folded into
    `UILoudnessMeter`'s own `kTargetPresets` array instead, so each is
    visible only when selected -- cycling Target to one of them now
    draws exactly one line, the same selectable Target line every view
    already draws, rather than four always-on lines specific to this one
    view. SoundCloud does not publish an official loudness-normalization
    target the way the other three do (it does not normalize playback
    loudness at all as of this writing); its own entry is a commonly-cited
    mastering reference from online discussion, not an official standard,
    which is exactly why it alone carries the `*` (now in its own
    `kTargetPresets` label, "Target: -9 (SoundCloud*)").
  - The **readout panel**'s own numeric values are drawn larger than the
    first version (`fontSize` `15.0` -> `21.0`; the row height and label
    size grew slightly alongside it), for better visibility. It also now
    flashes briefly
    right after "Reset" is clicked (`setResetFlash()`, a plain intensity
    in `[0,1]` the UI decays over ~0.6s and feeds into a background-color
    lerp) -- see the Plugin side section below for why this exists.
- **`UILoudnessMeter`**: much smaller than `UIPhaseScope` since there's no
  packed-point transport to batch here at all -- every Parameter is a
  single scalar, mirrored straight into a member and handed to
  `LoudnessMeterWidget::setReadings()` on every `parameterChanged()`
  push (the Short-term Distribution bin bank goes straight to
  `setDistributionShortTerm()`, the Momentary bank to
  `setDistributionMomentary()`; the L/R input readings go straight to
  `levelMeter_->setChannel()`).
  The history graph's own timing is **wall-clock-gated**, not tied to
  `parameterChanged()` pushes or a fixed `uiIdle()` call count: a
  `std::chrono::steady_clock` timestamp in `uiIdle()` triggers
  `pushHistorySample()` once every ~100ms regardless of the host's own UI
  refresh rate or whether a given reading actually changed value that
  tick, so the graph's own time axis stays evenly spaced. **Target**
  cycles through a small, fixed `kTargetPresets` array (Off, EBU R128,
  ATSC A/85, Spotify, Apple Music, YouTube, SoundCloud*) via
  `setState("targetLufs", ...)`;
  **View** toggles `showDistribution_` via `setState("showDistribution",
  ...)`, and **Reset** itself now pushes `setState("resetRequest",
  ...)` with an incrementing counter string -- all three the same State
  round-trip `UIPhaseScope`'s own Draw Lines/Smooth/Heatmap buttons use
  (see the Plugin side section above for why Reset moved off the
  `editParameter()`/Parameter-trigger convention `UIPhaseScope`'s own
  buttons still use for their genuine continuous Parameters).
  `TooltipOverlay` is reused verbatim from Phase Scope's own `gui/` --
  every button here has a hover description too.

## Known limitations (Loudness Meter)

**Only stereo** -- no mono/5.1/7.1 channel-weighting support the way
Youlean's own reference implementation offers; `LufsMeter::process()`
takes exactly `(l, r)` with equal `G_i = 1.0` weighting for both, the
same "stereo only for now" scoping `PhaseScopeDsp.hpp` itself uses.

**Distribution view added, but not Youlean's other alternate view
modes** -- a loudness Distribution histogram (a statistical view like
Youlean's) was added, toggled in place of the history graph; the bar meter and
numeric panel are still always shown regardless of that toggle. Youlean's
own remaining view modes (its "Mini View", full custom-preset system,
surround/Atmos channel layouts) are still out of scope -- the same
"standards-compliant numbers and the most useful views first" reasoning
as before, not a claim of full feature parity with a mature,
long-developed product.

**`TruePeakDetector` is not a certified reference filter** -- see its own
entry above. Good enough to reliably catch and report genuine
inter-sample energy for monitoring purposes; not validated against the
ITU's own conformance test vectors.
