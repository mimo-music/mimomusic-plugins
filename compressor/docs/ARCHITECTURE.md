# Architecture

This document describes the *current* system.

## DSP core (`plugin/dsp/CompressorDsp.hpp`)

Entirely header-only -- no `.cpp` files to keep in sync between the DSP and
UI sides, unlike `../statespace`'s stability guard/
transfer-function math, because everything here is small enough that
splitting declaration from definition would add bookkeeping without a
real benefit.

```
input --[x inputGain]--> xg --[abs]--> envelope follower --[dB]--> gain computer --[dB->linear]--> gr
                            |                                                                        |
                            +-------------------------------- xg * gr --------------------------------+ = y (post-compression)
y --[x compensationGain, if enabled]--[x outputGain]--> output
```

- **`computeGainReductionDb(levelDb, thresholdDb, ratio, kneeDb)`**: pure
  function, the soft-knee static curve (Giannoulis/Massberg/Reiss). Used
  both by the real-time engine (per sample) and directly by
  `CompressorGraphWidget` to draw the static transfer curve -- the same
  math, not a GUI approximation of it.
- **`EnvelopeFollower`**: linear-domain one-pole, asymmetric attack/release
  coefficients (`exp(-1/(sr*t))`). Deliberately smooths in linear
  amplitude, converting to dB only once at the point the curve is
  evaluated -- which avoids by construction a frequency-dependence bug
  (a dB-domain envelope touches -infinity at every zero crossing).
- **`LoudnessMeter`**: fixed ~300ms one-pole mean-square estimate,
  independent of the compressor's own attack/release. Drives both the
  bargraph's RMS fill and the volume-compensation comparison.
- **`PeakMeter`**: instant peak with a ~20dB/s conventional decay, drives
  the bargraph's peak-hold line and (via a >=0dBFS check) the clip LED.
- **`CompensationGain`**: measures once from real audio, then holds --
  not a perpetual auto-leveler. `beginMeasurement()` starts a cycle: a
  short (100ms) warm-up is skipped (so the compressor's own attack
  transient doesn't bias the result), then ~0.6s of *actual sound*
  (silence doesn't advance the countdown) is averaged into a fixed dB
  value, which `update()` then returns unchanged (`State::Held`)
  regardless of what plays afterwards -- until something calls
  `beginMeasurement()` again. The two plugins do that automatically
  whenever a compression-shape parameter (Threshold/Ratio/Attack/Release/
  Knee) changes while enabled, or on request via the "Measure" button's
  trigger Parameter. `state()` (Idle/Measuring/Held) and `value()` both
  feed output Parameters so the UI can show exactly what's happening. (A
  real measurement bias was found and fixed by adding the warm-up skip.)
  Note: Volume Compensation has since been removed from all three plugins
  (its Parameters remain only as unused placeholders); this class is
  what's left of it.
- **`CompressorVoice`**: bundles one `EnvelopeFollower` + the gain-computer
  call into "feed a detector sample, get back a linear gain multiplier" --
  the actual per-detector unit both plugins use.
- **`StereoMode`**: `Linked` / `DualMono` / `MidSide` -- see the Stereo
  plugin's own `run()` for exactly how each wires two `CompressorVoice`s
  to the L/R (or Mid/Side) signal path.

## Plugin sides (`plugin_mono/`, `plugin_stereo/`)

Ordinary DPF `Plugin` subclasses: a fixed bank of automatable `Parameter`s
for the seven knobs + Bypass + Compensation, and a second bank of *output*
Parameters for the meters (level/peak per channel, detector level, gain
reduction, compensation gain) -- read once per `run()` call by the UI side
via `parameterChanged()`, the same cross-process-safe mechanism the
sibling project's spectrum-analyzer output bands use. The Stereo plugin
additionally has one `State` (`stereoMode`) for the Linked/Dual-Mono/
Mid-Side choice -- a discrete topology pick, not something meaningful to
automate continuously, matching that project's own `spectrumMode`
convention.

`run()`'s per-sample body (Stereo, abbreviated -- Mono is the same minus
the per-mode switch):

1. Apply input gain.
2. Detect (per the active `StereoMode`) and compute gain reduction via the
   relevant `CompressorVoice`(s).
3. Apply the resulting gain to the signal.
4. Feed pre/post `LoudnessMeter`s (a mono-downmixed bus level, so the
   comparison means the same thing regardless of stereo mode) and call
   `CompensationGain::update()` -- **but its return value is not applied
   to the signal here**. It only measures/reports (via output Parameters)
   -- see "Where Volume Compensation actually applies" below for why.
5. Apply output gain.
6. Feed `PeakMeter`s for the bargraph (raw input pre-anything; final
   output post-everything).

### Where Volume Compensation actually applies

Not in `run()` at all -- an earlier draft did exactly that (a per-sample
`y *= dbToLinear(compDb)`, gated on `compensationEnabled_`), and it
worked, but it meant the compensation amount was an invisible multiplier
stacked *behind* Output Gain, with no way to see or adjust the number
directly, and no way to nudge it without either fighting the auto value
or turning the whole feature off. Moved to the UI side instead:
`UICompressorMono`/`Stereo::parameterChanged()` watches
`kParamMeterCompState` for the `Measuring -> Held` edge and, exactly
once per edge, calls `applyCompensationToOutputGain()`, which writes the
held value straight into the real `kParamOutputGain` Parameter (via
`setParameterValue()`, the same call a manual fader drag makes). From
that instant, Output Gain is the *only* gain-shaping value in the whole
signal path past the compressor -- one visible, freely user-editable
number that can then be adjusted slightly by hand.

Two details make repeated re-measurement and disabling behave correctly
rather than compounding or losing a manual tweak:
- `lastAutoAppliedCompDb_` tracks how much of the *current* Output Gain
  value this feature itself contributed last time. A fresh measurement
  computes `manualBaseline = currentOutputGain - lastAutoAppliedCompDb_`
  (undoing its own old contribution) before adding the new one --
  preserving whatever the user tweaked on top of (or instead of) the
  previous auto-applied value, rather than stacking indefinitely across
  repeated Threshold/Ratio/etc. changes.
- Disabling Compensation calls the mirror-image
  `removeCompensationFromOutputGain()`, subtracting out exactly its own
  last contribution and leaving any further manual adjustment alone.

Because `run()` itself never multiplies the signal by anything
compensation-related, a manual Output Gain edit -- at any time, mid-
measurement or long after Held -- is now literally impossible for
Compensation to "fight": there is nothing left in the DSP that would.

Bypass shortcuts this entirely (a plain copy) but still feeds the meters
with the pass-through signal, so the bargraphs never freeze on a stale
reading while bypassed.

## GUI (`gui/`)

- **`SimpleButton`**: copied verbatim from the sibling project.
- **`ParamStripWidget`**: a caller-supplied list of `ParamFieldSpec`
  (label/unit/range/default/nudge-step/decimals/paramIndex) drawn as one
  row of click-to-edit fields, generalizing that project's
  `NonlinearPanelWidget` field-row interaction into one reusable class.
  Also supports click-and-drag: press and drag vertically to fader-adjust
  the value (linear range mapping, or log-domain for `nudgeMultiplicative`
  fields like Attack/Release), gated behind a small dead zone so a plain
  click-and-release still falls through to the click/double-click-to-edit
  logic unchanged (safe despite the sibling project leaving the
  equivalent feature out of its own matrix cells: DGL keeps delivering
  motion events to the widget that took the press). Keyboard events reach the owning
  `UICompressorMono`/`Stereo` (not this subwidget), matching that
  project's "keyboard events reach the top-level UI" convention -- the UI
  checks `isEditing()` and forwards. `beginEdit()` calls `getWindow().
  focus()` (an embedded plugin window isn't guaranteed to already hold
  keyboard focus just because a widget inside it was clicked -- the exact
  fix, for the exact same reason, as the sibling project's own
  `beginCellTextEdit()`, missed here originally); an active edit shows a
  trailing caret and pure-black-on-amber text for maximum contrast, and a
  press anywhere (not just Enter) commits it first before doing anything
  else, so no press is ever left unclaimed -- both fixes for a real
  stuck-edit-mode bug.
- **`LevelMeterWidget`**: 1 or 2 vertical bars, RMS gradient fill + peak
  line + a latching clip LED.
- **`CompressorGraphWidget`**: the transfer curve + knee shading + live
  marker(s) + GR meter column(s) + scrolling GR history strip, all in one
  widget (combined rather than split further, since they share one
  coordinate mapping and redraw together). Two handles sit directly on
  the curve -- Threshold (horizontal drag) and Ratio (vertical drag,
  further up the slope) -- using an absolute pointer-position -> value
  mapping, plus scroll-anywhere-over-the-curve to adjust Knee. Both
  handles and the drawn curve share one set of `dbToPxX/Y()`/
  `pxToDbX/Y()` coordinate-mapping functions so the hit regions can never
  drift from what's actually drawn.
- **`UICompressorMono`/`UICompressorStereo`**: own a small `paramCache_`
  array (populated by `parameterChanged()`, since DPF's `UI` base class
  only pushes values, it doesn't expose a getter) and wire the widgets
  above together. The Stereo UI additionally owns the Stereo Mode toggle
  button and relabels the meters/graph (L/R vs Mid/Side) when the mode
  changes. Both own Load.../Save As... buttons (`gui/PresetIO.hpp`'s
  dependency-free key=value text format, `.mcmp`/`.mcsp`) and a Measure
  button plus the Volume Compensation toggle, whose own label doubles as
  the compensation readout ("measuring..." / "+4.2 dB") (since removed,
  see `CompensationGain` above).

## Known limitations

The stereo compensation gain is measured from the combined mono-downmixed
bus rather than per-detector (Dual-Mono/Mid-Side), which is the right
choice for "keep the overall loudness constant" but won't independently
track just one side's own level changing differently from the other.

