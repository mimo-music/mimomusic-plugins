# MimoMusic State-Space (Matrix Filter, EQs, Multiband Compressor, Dynamic EQs)

Six plugins sharing one state-space DSP core:

- **Matrix Filter** (2 in / 2 out) -- a directly editable discrete-time
  state-space filter.
- **Parametric EQ** (2 in / 2 out) -- an EQ with any number of bands, drawn
  on a large interactive graph.
- **Parametric EQ Mono** (1 in / 1 out) -- the same EQ for mono signals, at
  a fraction of the CPU cost.
- **Multiband Compressor** (2 in / 2 out) -- up to 6 bands with
  Linkwitz-Riley crossovers.
- **Dynamic EQ** (2 in / 2 out) -- bell bands whose gain reacts to the
  level in their own frequency region.
- **Dynamic EQ 2** (2 in / 2 out) -- Dynamic EQ with shelf types, harmonic
  band linking and stereo crossfeed.

## Common to all editors

- **Load... / Save As...** (preset formats below), **A / B / Copy**,
  **Undo / Redo**, **Bypass**, **Reset State** (clears the filter's
  internal state).
- **Export to State-Space...** (EQs, Multiband Compressor, Dynamic EQs):
  writes the current result as a Matrix Filter preset (`.ssjson`), where
  every coefficient can be inspected and edited.
- **Value fields**: drag up/down, double-click to type (Enter/Tab
  confirms, Escape cancels), scroll to nudge -- hold **Alt** for 10x finer
  steps. The same applies to scrolling over graph nodes.
- **Spectrum analyzer** behind the graph (EQs, Multiband, Dynamic EQs):
  **Spectrum Source** Off / In / Out / Both, **Smoothing**, **Resolution**
  (FFT size 2048–16384), and **Fixed Range / Autoscale**. Only the
  selected analyzers run.
- Nothing can be driven unstable: every assembled filter passes a
  stability guard, backed by a real-time safety net.
- Coefficient changes are smoothed, and changes of the filter's size
  (adding, removing, enabling or soloing a band) fade out and back in
  within about 16 ms, so editing doesn't click.

Preset formats: `.ssjson` (Matrix Filter), `.sseqjson` (Parametric EQ /
Mono), `.ssmbjson` (Multiband Compressor), `.ssdynjson` (Dynamic EQ and
Dynamic EQ 2).

## Matrix Filter

A generic state-space filter:

```
x[n+1] = A x[n] + B u[n]
y[n]   = C x[n] + D u[n]
```

`u` is the stereo input, `y` the stereo output and `x` an internal state
vector of 1 to 128 states. Every entry of A, B, C and D is editable, so it
can represent any linear, time-invariant stereo filter of that size: EQs,
crossovers, resonators, cross-channel (mid/side) matrices, all-passes. See
[docs/EXTENDING.md](docs/EXTENDING.md) for building higher-level
processors on top of it.

- **Matrix editor** for A/B/C/D: click a cell to edit, arrow keys move to
  the neighbouring cell, scroll to nudge. Cells are coloured by value
  (warm positive, cool negative). **+ Order / - Order** resize the model;
  **Passthrough** and **Silence** load a neutral or silent model.
- **Bode plot** of the full 2x2 transfer function, updated as you edit.
  Click a legend entry to show or hide a path (L→L, R→R, and the
  cross-channel L→R, R→L); LOG/LIN switches the frequency axis.
- **Normalize 0dB** scales the output (C/D only) so the peak passband gain
  is exactly 0 dB.
- **Gain curve (state-dependent gain)**: an optional panel at the bottom
  scales C/D by a gain that depends on a smoothed input level -- a
  compressor-like stage on top of the fixed linear filter. It has
  attack/release fields, an **Apply Curve** helper (threshold, ratio,
  makeup) and a 25-point gain-vs-level curve you can draw directly
  (double-click a point to reset it). The `fit_gain_curve` tool can learn
  this curve from an example input and output recording.
- `presets/factory/` contains 15 presets, from single filters to
  multi-band chains.

## Parametric EQ / Parametric EQ Mono

- **Bands**: click the graph to add a band, drag it (frequency and gain),
  scroll over it for Q (or BW for a Band Shelf). Hovering or dragging
  shows its values. Up to 32 bands (the filter's 128-state limit).
- **Types**: Bell, Low Shelf, High Shelf, Low Pass, High Pass, Notch,
  Band Pass, All Pass, and **Band Shelf**. Low/High Pass have a slope of
  12, 24, 36 or 48 dB/octave (in the right-click menu, or by clicking the
  Gain field in the band list).
- **Band Shelf**: a flat boost or cut between two corners, set by its
  centre frequency and a bandwidth in octaves (BW), with gain and Q; it
  returns to exactly 0 dB outside the corners. Drag the corner handles to
  change the bandwidth.
- **Right-click a node** for Delete, Duplicate, every filter type, and
  **Create Harmonic Twin** (x2, x3, x4, ÷2 or a custom factor): a copy at a
  harmonic frequency that stays linked in frequency -- move either and
  the other follows at the same ratio (Band Shelves link by their
  centre). **Unlink** detaches a band again. Delete a band also by
  double-clicking it or with Delete/Backspace.
- **Band list** below the graph: type, **EN**able, **SOLO**, channel,
  **FREQ**, **BW**, **GAIN**, **Q** and delete per band, all editable as
  value fields. **+ Band** adds one (and selects it).
- **Selected band**: between graph and list, the band selected in either
  of them gets large Freq / Gain / Q / Bandwidth fields (values its type
  ignores are marked "unused").
- **Channel** (stereo version): **Both**, **Left only**, **Right only**,
  **L→R** or **R→L**. A Left/Right-only band filters only that channel
  and leaves the other untouched; the graph then shows a separate blue
  (left) and red (right) curve and spectrum. An **L→R / R→L crossfeed**
  band filters one channel and adds the result into the other -- e.g. to
  move a sound from the left over to the right; the overall crossfeed
  level per direction is set on the graph. Soloing a one-channel band
  silences the other channel.
- **Audition**: hear only the selected band's frequency region (a band-pass
  at its frequency and Q, or between a Band Shelf's corners) instead of
  the output. Press and drag a node with the right mouse button to
  audition it while held. Linked harmonic twins are auditioned -- and
  soloed -- together.
- **Output Trim**, and a **gain range** label on the graph: click to cycle
  3/6/12/30 dB, or drag it for any range from 1 to 40 dB.

**Parametric EQ Mono** has the same editor and features, without the
channel column. It builds a genuinely single-channel filter with half
the states per band, which makes it 2.8x to 4.4x faster than the stereo
EQ on the same bands; its output is identical sample for sample to one
channel of the stereo EQ. Its export writes a stereo preset with both
channels equal.

## Multiband Compressor

- **Bands**: **+ Band** splits the widest band, double-clicking inside a
  band splits it there; double-click a crossover handle, or **Delete
  Band**, to merge. Up to 6 bands. Drag a crossover directly on the
  graph; **Crossover Slope** 12/24/36/48 dB/octave applies to all
  crossovers (flat when no band compresses).
- **Per band**: **Threshold, Ratio, Up Ratio, Attack, Release, Makeup** --
  Up Ratio lifts material below the threshold (upward compression; 1.0 =
  off). Select a band by clicking its region or with **< / >**; scroll
  over a region to change its Ratio. **Enabled** bypasses a band's
  dynamics; **Solo** plays only the selected band.
- **Graph**: each band's region is shaded and labelled with its threshold
  and ratio; the selected band shows a live gain-reduction/makeup meter.
- **Output Gain** for both channels after all bands.
- The `fit_multiband_compressor` tool learns all bands' gain curves from
  an example input and a compressed target recording and writes a
  Matrix Filter preset.

## Dynamic EQ

Bell bands that also carry **Threshold, Ratio, Up Ratio, Range, Attack
and Release**: a band's boost or cut moves when its own frequency region
gets loud (Ratio) or quiet (Up Ratio), by at most Range.

- **+ Band** adds a band; drag its node to set frequency and resting gain,
  scroll for Q, double-click to remove. **Delete Band**, and **< / >** to
  select. A new band starts with Threshold and Range at 0 dB, i.e. as a
  plain static bell.
- The selected band's values are edited in the field strip above the
  graph. **Enabled**, **Solo** (plays only the selected band) and
  **Dynamics** (switches the band between moving and static).
- **Graph**: a shaded area shows how far each band can move, and the
  bright curve shows the actual, current gain moving with the music. The
  node readout shows the live gain of bands with dynamics.
- **Ch**: Both, Left only or Right only per band; a one-channel band also
  detects only its own channel's level.
- **Audition**: hear the selected band's detector signal (the frequency
  region it measures), scaled by its current gain reduction -- for tuning
  a band by ear. Switching Audition fades through a brief silence.
- Export writes the resting-gain EQ (a moving filter has no fixed
  coefficients).

## Dynamic EQ 2

The dynamics of Dynamic EQ (threshold, ratio, up ratio, range, attack,
release per band, the shaded range and the live curve) with the
Parametric EQ's editor and more band types:

- **Graph** as in the Parametric EQ: click to add a band, drag it, scroll
  for Q (BW on a Band Shelf), double-click to remove; **right-click** for
  Delete, Duplicate, **Create Harmonic Twin**, Unlink and the type;
  **right-button drag** auditions the band while held. **+ Band**,
  **Audition**, the spectrum fields and **Output Trim** sit above the
  graph.
- **Band list** below the graph, one row per band with every setting:
  TYPE, EN (enable), SOLO, DYN (dynamics on/off), CH, FREQ, BW, GAIN, Q,
  THRESH, RATIO, UP (up ratio), RANGE, ATTACK, RELEASE and delete. Values
  can be typed, dragged or scrolled; fields that don't apply to a band's
  type show "--", and the dynamics values are dimmed while DYN is off.
  Solo works on one band at a time.
- **Selected band**: between graph and list, large fields for the selected
  band's Freq, Gain, Q (Bandwidth for a Band Shelf), Threshold, Ratio, Up
  Ratio, Range, Attack and Release (marked "unused" where the type or a
  switched-off DYN ignores them).
- **Types**: Bell, Low Shelf, High Shelf and Band Shelf. The shelves have
  a fixed slope, so Q only applies to a Bell; a Band Shelf uses BW.
- **Harmonic linking**: linked bands move together in frequency, share
  threshold, ratio and range, and are auditioned together.
- **CH**: Both, Left only, Right only, **L→R** and **R→L** crossfeed, as
  in the Parametric EQ.

## How it works

- `plugin/dsp/` holds the framework-independent core: matrices, the
  stability guard, transfer functions, JSON serialization, the
  coefficient mappers (RBJ cookbook biquads as state-space blocks),
  `EqModel` (band list → state-space), `MbCompressorModel` (crossover bank
  + per-band gain curves), and the gain-curve trainers.
- The Dynamic EQs run their own engines (`DynamicEqEngine.hpp`,
  `DynamicEqEngine2.hpp`): each band is a Regalia-Mitra construction
  around a fixed all-pass, so the gain can change every sample without
  moving any pole.
- The mono EQ runs a separate single-channel engine and stability guard.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) and
[docs/EXTENDING.md](docs/EXTENDING.md).

## Building

From the repository root, `./build_and_install.sh` builds all projects.
To build only this one:

```sh
./build_and_install.sh               # build and install the LV2 bundles into ~/.lv2/
./build_and_install.sh --test        # also run the tests
./build_and_install.sh --clean       # start from an empty build directory
./build_and_install.sh --no-install  # build only
```

The build produces, for `matrix_filter`, `parametric_eq`,
`parametric_eq_mono`, `multiband_compressor`, `dynamic_eq` and
`dynamic_eq2`, an LV2 bundle, a VST3 bundle and a standalone JACK
program. On macOS use `./build_and_install_macos.sh` (run on a Mac; needs
the Xcode Command Line Tools), which builds universal LV2/VST3/AU and
installs them into `~/Library/Audio/Plug-Ins/`.

### DPF patches

`dpf/` is the official DPF, unchanged. `dpf-patches/` holds two small
patches the build scripts apply before building and revert afterwards
(`dpf-patches/dpf_patches.sh`): LV2 editors ask the host to resize
(`ui:resize`) when they resize themselves, and automatic scaling can be
relative to a design size, so editors can be made smaller than it, and
enlarged in half zoom steps (2x, 2.5x, ... -- fractional zooms such as
1.37x look soft; the extra space goes to the layout). A plain
`cmake` build without the scripts needs them applied first:
`dpf-patches/dpf_patches.sh apply dpf <some-state-dir>`.

## Testing

```sh
./build/bin/dsp_regression    # fast math checks (runs with --test)
./build/bin/audio_testbench   # real audio through the real engine, all factory presets
./build/bin/fit_gain_curve <base.ssjson> <input.wav> <target.wav> <out.ssjson>
./build/bin/fit_multiband_compressor "<crossover_hz_csv>" <input.wav> <target.wav> <out.ssjson>
```

See [docs/TESTING.md](docs/TESTING.md).

## Project layout

```
plugin/dsp/             DSP core shared by all six plugins
plugin/                 Matrix Filter plugin
plugin_eq/              Parametric EQ plugin
plugin_eq_mono/         Parametric EQ Mono plugin
plugin_mb_compressor/   Multiband Compressor plugin
plugin_dyn_eq/          Dynamic EQ plugin
plugin_dyn_eq2/         Dynamic EQ 2 plugin
gui/                    Matrix Filter editor and shared widgets
gui/eq/, gui/mb/,       editors of the EQs, the Multiband Compressor and
gui/dyneq/, gui/dyneq2/ the Dynamic EQs
presets/factory/        factory presets
tools/                  audio testbench and the gain-curve trainers
tests/                  dsp_regression, dyneq2_list_sim
docs/                   ARCHITECTURE.md, EXTENDING.md, TESTING.md
dpf/                    DISTRHO Plugin Framework (git submodule)
```

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
