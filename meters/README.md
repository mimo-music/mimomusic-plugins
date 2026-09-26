# MimoMusic Meters (Phase Scope, Loudness Meter)

> This project is the measurement sibling of
> [`../compressor`](../compressor) and [`../statespace`](../statespace) --
> same GUI toolkit, colors, and build tooling, deliberately kept
> consistent across all mimomusic projects -- but purely observational:
> nothing here ever changes the audio.

**Phase Scope** (2-in/2-out) -- `plugin_phasescope/`, `gui/UIPhaseScope.*`
-- the first plugin in this measurement family. Inspired by Logic Pro's
MultiMeter's correlation/goniometer view, narrowed to stereo
image/spread and phase correlation.

**Loudness Meter** (2-in/2-out) -- `plugin_loudness/`,
`gui/UILoudnessMeter.*` -- the second, a standards-based (ITU-R BS.1770-4
/ EBU R128 / EBU Tech 3342) loudness meter modeled on Youlean Loudness
Meter 2's own set of readings, units and general layout (bar meter +
history graph + numeric readout panel).

### Build and install

```sh
./build_and_install.sh            # build, install LV2 into ~/.lv2/
./build_and_install.sh --clean    # wipe build/ first (full rebuild)
./build_and_install.sh --test     # also run the regression suite
./build_and_install.sh --no-install
```

#### macOS

Not reliably cross-compilable from this Linux checkout -- see the sibling
`../compressor` project's README's identical note for why.
`./build_and_install_macos.sh`, run **on a Mac**, builds a universal
(Apple Silicon + Intel) LV2/VST3/AU set and installs into
`~/Library/Audio/Plug-Ins/{LV2,VST3,Components}`; written carefully but not
tested end to end (no Mac was available while writing it).

## Phase Scope

A purely observational stereo measurement plugin -- the audio signal
passes straight through unmodified, copied from input to output before any
metering runs. Four readouts:

- **Goniometer** (`gui/GoniometerWidget`): a rotated Mid/Side vectorscope,
  labeled M (mono, straight up), L and R (the upper ends of the two
  diagonals -- a hard-Left-only signal traces the upper-left diagonal, a
  hard-Right-only one the upper-right). Drawn as a fading dot trail
  (350ms by default, adjustable, see Decay below) or a connected-line
  vector trace rather than a single-frame snapshot, the classic goniometer
  look. The trail's own color shifts green -> amber -> red as phase
  correlation drops from +1 to -1, so a phase problem is visible in the
  shape *and* the color at a glance.
- **Correlation meter**: a -1..+1 bargraph (the standard leaky-integrated
  Pearson correlation coefficient, ~300ms ballistics), plus a **worst-case
  hold** marker -- a brief phase problem that a smoothed reading alone
  would hide instead sits flat for 1.5s (matching the L/R meters' own
  clip-hold time), then slowly recovers. That 1.5s plateau matters: ordinary program material's
  correlation reading keeps wobbling even after smoothing, so without it
  the marker would just re-snap onto every little dip and never visibly
  separate from the live reading.
- **Stereo width meter**: 0-200%, 100% marked as the "ordinary
  uncorrelated stereo" reference point -- 0% is mono (no Side energy at
  all), 200% is entirely Side (e.g. exactly out-of-phase content).
  Derived from the *same* correlation statistics rather than a second,
  separate measurement -- see `docs/ARCHITECTURE.md`.
- **L/R peak/RMS bargraphs** (`gui/LevelMeterWidget`, copied verbatim from
  `../compressor/gui/`): the same clip-LED/peak-hold meter every mimomusic
  plugin with a level readout already uses.

Display controls (all purely cosmetic -- the audio itself is never
touched, see `docs/ARCHITECTURE.md`):

- **Gain** (dB, -24..+24): a manual trim on the goniometer's own plot
  scale, for zooming in on a quiet signal or pulling back from a hot one.
- **Auto Gain**: an on/off toggle, not a one-shot action. Click it on, and
  Gain is continuously adjusted from a smoothed (multi-second-averaged,
  not instantaneous) reading of the signal's level so the loudest recent
  material keeps filling the plot without the correction visibly jumping
  around on every transient; click it off again, and Gain simply stops
  moving and stays at whatever it last settled on. No waiting, no
  "Measuring..." state to get stuck in either way. Both the smoothing
  time and the target level are single constants in
  `plugin/dsp/PhaseScopeDsp.hpp` (`kAutoGainTimeConstantSeconds`,
  `kAutoGainTargetPeak`) if it needs to react more slowly or land less
  hot on your own material.
- **Decay** (ms, 100-3000, default 600): how much of the goniometer's
  trail is currently shown. The trail is always written at the same fixed
  density regardless of this setting, so a bigger Decay genuinely reveals
  *more* points (a longer trail at the same density), not the same point
  count stretched thinner over more time.
- **Draw: Dots / Lines** (**Lines by default**): toggles the goniometer
  between a fading dot cloud and a connected-line vector trace -- the two
  common vectorscope rendering styles. Persists across sessions (a
  State), unlike Freeze.
- **Smooth** (Draw: Lines mode only, grayed out otherwise, **on by
  default**): draws a Catmull-Rom spline through the drawn points instead
  of straight segments between them -- a smooth, continuous curve that
  still passes through every point exactly, so a low Resolution setting
  doesn't have to look visibly faceted.
- **Res** (%, 10-100, default 100): what fraction of the currently-visible
  points actually get drawn each frame -- purely a rendering-cost knob for
  a machine with less GPU/CPU headroom to spend on this widget. Lower it
  if the goniometer feels heavy; the buffer itself is still written and
  transported at full density regardless, so raising it back to 100% any
  time shows the full trail again with nothing lost.
- **Heatmap**: an optional long-history (1.0-30.0s, its own **Heat**
  decay field in seconds) density glow drawn underneath the grid, axis
  labels and the live trail -- "where has this signal actually been
  spending its time" over many seconds, not just the last fraction of
  one. Colored a dark blue -> dark green -> dark yellow -> dark red
  gradient, every stop kept dark (two earlier, brighter gradients both
  made the live green trail hard to see), and capped
  well under fully opaque so the grid and the live trail both stay
  clearly visible on top of it. It accumulates in the background
  continuously regardless of whether this toggle is on, so switching it
  on shows the full multi-second history immediately rather than starting
  from blank -- and repositions its own history to match whenever Gain
  changes, since it's recorded in the same Gain-scaled coordinates the
  live trail plots in: rather than discarding it, existing "hot" spots
  move to where they now belong at the new scale, so the heatmap keeps
  accumulating and fading out normally even while Auto Gain is
  continuously nudging Gain, instead of being wiped every time it moves.
- **Freeze** pauses the display (purely a UI-local pause, not a Parameter)
  to study a moment without the trail scrolling away.
- **Hover any button or strip field** for a short description of what it
  does -- every control here has one.

## Loudness Meter

Another purely observational stereo plugin -- same "input copied straight
to output" convention as Phase Scope. A standards-based loudness
measurement (ITU-R BS.1770-4 / EBU R128 / EBU Tech 3342), the same set of
readings, units and general layout Youlean Loudness Meter 2 uses (bar
meter + history graph + numeric readout panel), without cloning every one
of its view modes -- see `docs/ARCHITECTURE.md` for the DSP details and
what was deliberately left out.

- **Momentary / Short-term / Integrated** (LUFS): the standard's own three
  time windows -- 400ms, 3s, and the whole measurement session
  respectively. Integrated uses the standard's two-stage absolute
  (-70 LUFS) + relative (-10 LU below the ungated mean) gating, so quiet
  passages and silence don't drag the "average loudness of the song"
  number down.
- **Loudness Range** (LRA, in LU): how much the Short-term reading moves
  around over the whole session (the 95th minus the 10th percentile of
  its own gated distribution, per EBU Tech 3342) -- a small number means
  consistently-leveled material, a large one means a lot of dynamic
  variation.
- **True Peak** (dBTP), per channel and the louder of the two: a
  4x-oversampling interpolator catching inter-sample peaks a plain
  sample-peak reading can miss on a hot digital mix. A general-purpose
  design, not a bit-exact reproduction of the ITU's own reference filter
  -- see `docs/ARCHITECTURE.md`.
- **PLR / PSR** (LU): Peak-to-Loudness Ratio (True Peak Max minus
  Integrated) and Peak-to-Short-term Ratio (True Peak Max minus the live
  Short-term reading) -- the same two ratios and units Youlean uses.
- **L/R bars**: a plain, non-K-weighted per-channel dBFS peak/RMS bargraph
  (the same `gui/LevelMeterWidget` `../plugin_phasescope` already uses),
  sitting to the left of the LUFS-specific display.
- **Bar meter**: Momentary as a filled, green-to-red bar; Short-term as a
  small triangular pointer riding its own right edge, *and* Short-term
  and Integrated both again as short horizontal bars mapped to the exact
  same LUFS scale the vertical bar itself uses, for a more direct
  side-by-side comparison -- the same overall convention Youlean uses,
  extended with the horizontal bars.
- **History graph**: a scrolling ~60s view of the same Momentary/
  Short-term readings over time, a dashed Target reference line, and
  (unlike the first version) a Catmull-Rom spline through the points
  instead of straight segments, with the area under each curve filled in
  a translucent version of its own color -- Short-term drawn first
  (background), Momentary second (foreground, at a lower fill opacity
  specifically so Short-term's own curve/fill stays visible underneath).
- **Distribution view**: click "View" to switch the center panel from the
  history graph to a loudness density plot -- a smooth, filled curve (the
  same spline construction the history graph's own curves use, not a bar
  chart) showing how many Short-term readings, over the whole session,
  fell at each loudness level -- the same "statistical view" Youlean
  offers -- with reference lines for a few commonly-cited platform
  loudness targets (Spotify, Apple Music, YouTube, and a SoundCloud
  figure flagged as an approximate, unofficial mastering reference rather
  than a published standard -- SoundCloud does not publish or enforce a
  loudness-normalization target the way the other three do) alongside the
  same selectable Target line.
- **Target**: cycles a reference line (drawn on the bar meter, the history
  graph, and the Distribution view) through EBU R128 (-23 LUFS), ATSC
  A/85 (-24 LUFS), and two common streaming-platform levels, or off. A
  pure display preference -- picking one never changes any measurement.
- **Reset** clears the accumulated Integrated/Range/Distribution/
  True-Peak-max statistics for a fresh measurement pass, without touching
  the live Momentary/Short-term readings -- a brief flash on the readout
  panel confirms the click itself registered, since on steady material
  those cleared numbers can resettle to a very similar value within a
  second or two, which without the flash can look identical to nothing
  having happened.
- **Numeric readouts** are drawn larger than in the first version.
- **Hover any button** for a short description, same convention as Phase
  Scope.

## Project layout

```
plugin/dsp/PhaseScopeDsp.hpp   framework-independent DSP core (header-only):
                                dB<->linear, PeakMeter/LoudnessMeter,
                                StereoImageMeter (correlation + width from
                                one shared set of leaky sums),
                                GoniometerBuffer (decimated, packed scope
                                ring buffer), HeatmapGrid (1-30s density map)
plugin/dsp/LoudnessDsp.hpp     framework-independent DSP core (header-only):
                                KWeightingFilter (ITU-R BS.1770-4), SlidingMeanSquare
                                (exact rectangular-window mean square), LoudnessHistogram
                                (energy-weighted gating for Integrated/LRA),
                                DistributionHistogram (plain tally for the
                                Distribution view), TruePeakDetector
                                (4x-oversampling interpolator), LufsMeter
                                (ties them all together)
plugin_phasescope/             Phase Scope's DPF plugin (DSP) side
plugin_loudness/                Loudness Meter's DPF plugin (DSP) side
gui/                            shared DPF UI (GUI) side, NanoVG-drawn widgets:
                                SimpleButton, LevelMeterWidget (both copied
                                verbatim from ../compressor, LevelMeterWidget
                                reused by both plugins here), GoniometerWidget,
                                StereoMeterWidget, LoudnessMeterWidget,
                                TooltipOverlay (hover descriptions for every
                                button/field), UIPhaseScope, UILoudnessMeter
build_and_install.sh           builds everything, installs the LV2 bundles to ~/.lv2/
tests/dsp_regression.cpp       fast pure-math correctness tests, no DPF/GUI dependency
docs/ARCHITECTURE.md           how the DSP core and GUI actually work
docs/TESTING.md                 the verification tool, in detail
dpf/                            DISTRHO Plugin Framework (git submodule)
```

## Testing

See [docs/TESTING.md](docs/TESTING.md) for the full picture.

```sh
./build/bin/dsp_regression    # fast, pure-math checks for both plugins --
                               # correlation/width formulas (several exact by
                               # construction), the min-hold ballistics,
                               # goniometer pack/unpack round-trip, decimation
                               # timing, the peak/RMS meter ballistics, and the
                               # Loudness Meter's own K-weighting/gating/
                               # true-peak/LUFS pipeline
```

No DPF/GUI dependency -- builds automatically alongside the plugin.

## License

Copyright (C) 2026 Mettronic GmbH. MimoMusic is a brand of Mettronic GmbH.

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your
option) any later version. It is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See [LICENSE](LICENSE)
for the full text.

Other licensing terms (for example for commercial or embedded use) are
available from Mettronic GmbH.

Contributions are welcome under the [Contributor License Agreement](CLA.md)
-- see [CONTRIBUTING.md](CONTRIBUTING.md) for why and how.

Third-party code: `dpf/` is the DISTRHO Plugin Framework, (C) Filipe Coelho
and contributors, ISC license -- see `dpf/LICENSE`; the libraries bundled
inside it keep their own licenses.

Before any real distribution: `DISTRHO_PLUGIN_URI` in each plugin's own
`DistrhoPluginInfo.h` must be a URI you actually control (LV2 identity
requirement), and the brand/unique IDs should be yours.
