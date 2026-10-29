# Architecture

## Overview

```
plugin/dsp/CompressorDsp.hpp       shared DSP core (header-only)
plugin/dsp/TruePeakLimiterDsp.hpp  the Limiter's True Peak gain computer
plugin/dsp/CurveDsp.hpp            Contour's curve and voice
plugin/dsp/SpectrumAnalyzer.*      FFT analyser for the sidechain/trigger scopes
plugin_mono/, plugin_stereo/,      DPF Plugin subclasses (parameters, state, run())
plugin_limiter/, plugin_contour/
plugin_contour_mono/               only DistrhoPluginInfo.h (CONTOUR_MONO)
gui/                               DPF UIs and NanoVG widgets
```

The DSP files have no DPF/DGL dependency, so the tests and the audio
testbench build against them directly.

## DSP core (`CompressorDsp.hpp`)

```
input -> x inputGain -> xg ---------------------------------------------+
                                                                         |
trigger (xg, or the sidechain input) -> x sidechainGain -> [band filter] |
   -> |.| -> EnvelopeFollower -> dB -> gain computer -> dB->linear = gr  |
                                                                         v
                                        y = xg * gr * makeup, then wet/dry, output gain
```

- **`computeGainReductionDb(levelDb, thresholdDb, ratioDown, ratioUp,
  kneeDb)`** -- the static soft-knee curve (Giannoulis/Massberg/Reiss).
  `ratioDown` acts above the threshold, `ratioUp` below it (upward
  compression); the plugins set one of them to 1 depending on Ratio
  Down/Up. The graph widget draws the curve with this same function.
- **`EnvelopeFollower`** -- linear-domain one-pole with separate attack
  and release coefficients (`exp(-1/(sr*t))`, 0 ms = instant). Smoothing
  happens in linear amplitude and is converted to dB only where the curve
  is evaluated; a dB-domain envelope would hit -infinity at every zero
  crossing.
- **`CompressorVoice`** -- one `EnvelopeFollower` plus the gain computer:
  feed a rectified detector sample, get a linear gain.
- **`SidechainBandFilter`** -- high-pass and low-pass corner in series for
  the trigger signal; skipped entirely while the filter is off.
- **`softLimitListen()`** -- soft limiter for Sidechain/Trigger Listen,
  since up to +60 dB of trigger gain can make that signal very loud.
- **`LoudnessMeter`** (about 300 ms mean square) and **`PeakMeter`**
  (instant peak, about 20 dB/s decay) for the bargraphs and clip LEDs.
- **`StereoMode`** -- `Linked`, `DualMono`, `MidSide`.

## Compressor Mono / Stereo (`plugin_mono/`, `plugin_stereo/`)

Per sample (Stereo):

1. Gain staging for the stereo mode: Input Gain (and in Dual Mono /
   Mid/Side the Global Input Gain before the split; in Mid/Side the
   per-bank Input Gain applies after the M/S decode).
2. The trigger signal: the external sidechain input with Sidechain on,
   otherwise the gain-staged main signal. Sidechain Gain, then the
   optional band filter.
3. Detection per mode: `max(|L|,|R|)` into one voice (Linked), one voice
   per channel (Dual Mono), Mid and Side of the filtered trigger into one
   voice each (Mid/Side).
4. Apply gain, makeup and wet/dry per bank; Mid/Side re-encodes.
5. Overrides: M/S Listen (Mid or Side alone), Sidechain Listen (the
   filtered trigger, soft-limited).
6. Output Gain, then the output meters (M/S in Mid/Side mode).

Bank B (Threshold B, Ratio B, ...) is only used while the two channels are
independent (Dual Mono with Link off, or Mid/Side); otherwise it mirrors
bank A. Parameters whose control is hidden in the current mode are also
inactive in the DSP, so stray automation can't change the sound through a
control that isn't shown.

Meters, gain reduction and the scope data are output Parameters read by
the UI. The gain-reduction readout is `sscomp::GrPeakMeter`: the most
extreme gain of the block, held 50 ms, so a short reduction between two
editor reads still arrives (the host only passes on the latest value).
The editors put their Fast/Slow ballistics on top (`gui/GrMeterBallistics.hpp`,
attack/release 5 ms/0.4 s or 0.3 s/1.2 s, for bar and number together;
peak hold and history use the unsmoothed readout). The choice is the
UI-only State `grMeterSpeed`, like `presetDir`. The two spectrum analysers only run while the editor's scope
panel is open: the UI increments `kParamUiHeartbeat` every idle tick, and
`run()` feeds the analysers only if it changed within the last ~2 s (a
counter, not a toggle, because the LV2 wrapper only delivers changed
values).

Discrete choices that are not meant to be automated -- stereo mode, Ratio
Down/Up, Link -- are `State`s. Parameters are only ever appended, never
reordered, so saved sessions and automation keep their meaning; unused
entries stay as placeholders marked "(unused)".

Bypass copies input to output but keeps feeding the meters and the scope.

## Limiter (`plugin_limiter/`, `LimiterDsp.hpp`)

`LimiterDsp.hpp` holds the math shared by `run()` and the graph, so the
drawn curve is computed by the code the audio runs.

- A lookahead delay line holds the audio back by Lookahead; the gain is
  computed from the undelayed signal. The delay is reported to the host as
  latency. Bypass runs through the delay line too, so the reported latency
  stays true and the delay line stays current for switching back.
- **Classic**: a `CompressorVoice`, stereo-linked, at 20:1, 100:1 or ∞:1
  (`limiterRatioValue()`; the slope above the knee is 1/ratio, so ∞:1 is
  flat).
- **True Peak** (`TruePeakGainComputer`): for each incoming sample the gain
  it alone would need (threshold / level, at most 1); the gain applied to
  the delayed sample is the minimum of those values over the whole
  lookahead window (a monotonic-deque sliding minimum, O(1) amortized, no
  allocation). A drop is applied immediately, recovery is smoothed with
  Release, so the output provably never exceeds the threshold. Attack,
  Knee and Ratio don't apply here; the graph draws it as ∞:1.
- **Gain staging** (`computeLimiterStaging()`): the gain after the limiter
  (Output Gain + Boost, Boost = `-threshold`) is capped at
  `targetTop - threshold`, where targetTop is the Ceiling (Hard) or
  `Ceiling - Clip Knee` (Soft). Only positive gain is removed; if even 0 dB
  would put the Threshold above targetTop, the gain computer limits at
  targetTop instead. So the limited peaks land on targetTop and the clip
  stage only sees overshoot.
- **Clip stage**: `hardClip()` (clamp) or `softClip()` (identity up to the
  knee start, then a tanh curve that approaches the Ceiling and never
  crosses it).
- **Oversampling** (`TruePeakDetector`): detection runs on a 4x estimate of
  the peak around each sample -- a 24-tap-per-phase polyphase Kaiser sinc
  for 3 points between samples, each local grid maximum refined with a
  parabola (a plain 4x grid misses up to 0.3 dB at 16 kHz). Its window
  needs 12 samples ahead, so with Oversampling the audio is delayed 12
  samples more and the latency grows accordingly.
- `run()` dispatches to `processBlock<oversampling, softClip>()`, so a
  disabled feature is compiled out of the loop.
- Switching the algorithm resets both gain computers.

## Contour (`plugin_contour/`, `CurveDsp.hpp`)

### `TransferCurve`

A fixed-size, allocation-free list of (input dB, output dB) points from
-80 to +6 dB: two end points with fixed X, up to 16 interior points. Each
point carries:

- an optional tangent handle -- zero means a corner (straight segments),
  non-zero makes the adjacent segments cubic Beziers (not clamped against
  overshoot);
- a direction tag (both / rising only / falling only) -- the rising and
  falling curves are separate views of the point list, so the level can
  take different paths up and down;
- a link group and axis (X+Y, X only, Y only) and an optional snap to the
  diagonal -- editing aids evaluated in the UI, not in the DSP.

Two points may share one X (a vertical step, e.g. for a gate); the later
point governs at and above that X. `curveToString()` / `curveFromString()`
serialize the curve as a short text string, used both as the DPF State
and in `.mccp` presets.

### `CurveVoice`

Two signals go in: the trigger (after Trigger Gain and filter) and the
audio's own level.

1. **Trigger**: a fast peak detector (instant attack, release = Release
   capped at 8 ms) compared with the threshold (Above or Below) -- *met* or
   not. With lookahead, the end of a trigger is held for the lookahead
   time so the delayed audio it belongs to is fully covered.
2. **Switch**: a value from 0 (diagonal) to 1 (curve). Its target follows
   *met* and the On Trigger setting (Curve or Diagonal); it moves with
   Attack when the trigger starts and Release when it ends.
3. **Curve gain**: the audio's level (peak follower, instant attack,
   Release to fall) is looked up on the curve -- on the rising or falling
   view depending on the level's direction. Deeper cuts are applied at
   once, shallower ones relax with Release, so a low tone's ripple doesn't
   modulate the gain.
4. Output gain = switch x curve gain (in dB).

The trigger decides only *when* the curve applies, never *where* on the
curve the audio sits.

### Plugin

As Compressor Stereo (stereo modes, bank B, wet/dry, heartbeat), plus a
lookahead ring buffer for the audio and a bank of (level, gain) pairs per
block for the graph's trail of dots.

**Contour Mono** is built from the same sources: its
`DistrhoPluginInfo.h` defines `CONTOUR_MONO`, and the few mono-specific
places check `kContourMono`. Its CMake target includes
`plugin_contour_mono/` instead of `plugin_contour/`, which is what selects
the mono info header.

## GUI (`gui/`)

- **`SimpleButton`**, **`TooltipOverlay`** (hover descriptions),
  **`LevelMeterWidget`** (bargraphs with peak line and clip LED).
- **`ParamStripWidget`** -- a row of value fields from a list of
  `ParamFieldSpec`s: click or double-click to type, drag vertically
  (linear, or logarithmic for Attack/Release), scroll to nudge, Alt for
  finer steps. Keyboard input goes to the owning UI, which forwards it
  while a field is being edited; starting an edit takes keyboard focus.
- **`CompressorGraphWidget`** -- transfer curve, knee shading, live
  marker, Threshold and Ratio handles, scroll for Knee, bipolar
  gain-reduction column with peak hold, its Fast | Slow switch
  (`GrSpeedSwitch`), and history strip. Drawing and hit-testing share one
  coordinate mapping.
- **`LimiterGraphWidget`** -- the Limiter's curve with Threshold handle and
  Ceiling line.
- **`ContourGraphWidget`** -- curve editing (tools, selection, handles,
  links, context menu), threshold line, trigger-side shading and the
  trail of dots; **`NodeListWidget`** -- the numeric node list.
- **`SidechainScopeWidget`** -- the trigger's level history and its
  spectrum before and after the filter, with draggable filter corners.
- **`UICompressorMono` / `UICompressorStereo` / `UILimiter` /
  `UIContour`** -- keep a cache of all parameter values, wire the widgets
  together, and implement A/B, undo/redo and presets (`PresetIO.hpp`, a
  dependency-free key=value text format).
