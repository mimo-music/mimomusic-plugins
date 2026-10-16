# MimoMusic Compressor (Compressor, Limiter, Contour)

Dynamics plugins sharing one feed-forward DSP core:

- **Compressor Mono** (1 in / 1 out, plus a mono sidechain input)
- **Compressor Stereo** (2 in / 2 out, plus a stereo sidechain input)
- **Limiter** (2 in / 2 out)
- **Contour** (2 in / 2 out, plus a stereo sidechain input) -- dynamics
  from a freely drawn input/output curve
- **Contour Mono** (1 in / 1 out, plus a mono sidechain input)

## Common to all editors

- **Load... / Save As...** store the settings as a preset file (`.mcmp`
  Compressor Mono, `.mcsp` Compressor Stereo, `.mlim` Limiter, `.mccp`
  Contour -- Contour and Contour Mono share the format).
- **A / B / Copy** compare two settings; **Undo / Redo**; **Bypass**.
- **Value fields**: drag up/down to change, double-click to type (Enter or
  Tab confirms, Escape cancels), scroll to nudge -- hold **Alt** for 10x
  finer steps.
- **Level meters** with peak hold and a clip LED (click a meter to clear
  it).
- Hover over any control for a short description.

## Compressor Mono / Compressor Stereo

A classic compressor: **Input Gain, Threshold, Ratio, Attack, Release,
Knee, Makeup Gain**.

- **Ratio: Down / Up** -- Down compresses material above the threshold
  (ordinary compression), Up compresses the material below it (lifting
  quiet parts).
- **Graph**: the transfer curve with the knee shaded and a live marker of
  the current input level. Drag the **Threshold** handle horizontally and
  the **Ratio** handle vertically; scroll over the graph to change the
  Knee. Next to it a gain-reduction meter (bipolar -- Ratio Up can boost)
  and a scrolling gain-reduction history.
- **Sidechain: Internal / External** -- External takes the detector
  signal from the sidechain input; the compressed audio is always the main
  input. **Sidechain Gain** trims the detector signal.

### Compressor Stereo only

- **Stereo mode** (button): **Linked** (one detector for both channels,
  the stereo image never shifts), **Dual Mono** (independent L and R) or
  **Mid/Side** (independent Mid and Side). In Dual Mono and Mid/Side a
  second graph and control bank appear, plus a **Global Input Gain**
  applied before the split; in Dual Mono **Link** keeps both banks equal,
  in Mid/Side **Listen** plays Mid alone, then Side alone.
- **Wet/Dry** per compressor (parallel compression) and a global
  **Output Gain**.
- **Sidechain Options...** opens a panel for the trigger signal -- the
  external sidechain input when Sidechain is External, otherwise the main
  signal itself:
  - **Sidechain Gain** and a **Highpass/Lowpass** band filter (**Filter:
    On/Off**), so the compressor reacts only to one frequency range --
    e.g. a de-esser, or compression that ignores the bass;
  - per-channel corners in Dual Mono / Mid/Side (**L/R Link**, **Editing:
    L/R**);
  - a scope showing the trigger signal's level over time and its spectrum
    before and after the filter;
  - **Listen** to hear the trigger signal instead of the output.

  Gain and filter only change what drives the compressor, never the audio
  you hear (except while Listen is on).

## Limiter

**Input Gain, Threshold, Attack, Release, Lookahead, Knee, Ceiling, Clip
Knee, Output Gain**; top row **Bypass**; row 2 above the graph **Algorithm,
Ratio, Clip, Oversampling, Boost**. Detection is always stereo-linked.

- **Lookahead** (default 5 ms): the audio is delayed so the limiter can
  react before a peak arrives. The delay is reported to the host as
  latency, and Bypass keeps it, so bypassing doesn't shift the audio.
- **Algorithm: Classic / True Peak** -- Classic is an envelope-follower
  limiter with **Ratio 20:1 / 100:1 / ∞:1**; fast transients can still
  overshoot the Threshold briefly. True Peak is a sample-accurate
  lookahead limiter that never lets a sample over the Threshold (always
  ∞:1); Attack and Knee have no effect there and are marked "(unused)".
- **Ceiling** (default 0 dB): the output never goes above it. **Boost**
  (adds the Threshold's reduction back, like the "unity gain" switch of
  other limiters) and **Output Gain** come after the limiter and are
  capped so the limited peaks land on the Ceiling instead of being clipped
  by it; if the Ceiling is below the Threshold, the limiter limits at the
  Ceiling. The graph says when a gain is capped.
- **Clip: Hard / Soft** -- what happens to overshoot that still reaches
  the Ceiling: Hard cuts it flat, Soft rounds it off over the **Clip
  Knee** below the Ceiling (the shaded band in the graph, its lower edge
  draggable). With Soft the limited peaks sit at the start of that band.
  Switching the Algorithm sets the matching default -- Soft for Classic,
  Hard for True Peak -- which can be changed afterwards.
- **Oversampling: Off / 4x** -- 4x oversampled peak detection, so peaks
  between samples (inter-sample peaks, which can go over the Ceiling after
  D/A conversion or lossy encoding) are limited too. Adds 0.25 ms of
  latency; off costs nothing extra.
- **Graph**: the whole chain from input to output. Drag the **Threshold**
  handle horizontally and the dashed **Ceiling** line vertically. With
  Boost or Output Gain, a faint line shows the limiter alone and an arrow
  the gain added after it. A **CLIP** indicator above the gain-reduction
  meter shows when the clip stage acts; gain-reduction meter and history
  as in the Compressor.

## Contour

A dynamics processor where the level mapping is a curve you draw instead
of a ratio/knee formula -- one engine for downward and upward
compression, limiting, gates and expanders.

### How it works

1. **The curve maps the sound's level (x) to an output level (y).** A
   straight diagonal changes nothing.
2. **The trigger decides whether the sound follows the curve or the
   diagonal.** The trigger signal (the main input, or the external
   sidechain) can be amplified (**Trigger Gain**) and filtered (**Trigger
   Filter**); it is *met* while its level is above or below
   (**Trigger: Above / Below**) the **Threshold**.
   - **On Trigger: Curve** -- while the trigger is met the sound follows
     the curve, otherwise the diagonal.
   - **On Trigger: Diagonal** -- the other way round (a gate that opens).
3. **Attack and Release** set how fast the sound moves to the new state
   when the trigger starts or stops being met (0 = instant).
   **Lookahead** (0–20 ms) delays the audio so the switch is already in
   place when the triggering sound arrives; the delay is reported to the
   host as latency.

The trigger only decides *when* the switch happens, never where on the
curve the sound sits. The fading dots in the graph show each moment's
input level (x) against its output level (y); green dots mean the trigger
is met.

### Drawing the curve

The curve runs from -80 to +6 dB on both axes, with up to 16 nodes
between two end nodes (squares; their height can change, their position
not).

- **Tool** (button above the graph):
  - **Draw Node** -- click to place a corner node; click and drag to
    place a smooth node and pull out its Bezier handles. Drag an existing
    node to move it, double-click it to remove it.
  - **Draw Rising Node / Draw Falling Node** -- the node only shapes the
    curve while the level is rising / falling (hysteresis: different paths
    up and down).
  - **Edit / Move** -- drag nodes and handles, drag a window or
    Shift+click to select several nodes, drag the selection as a group.
- **Right-click a node** for a menu: Normal / Rising only / Falling only,
  **Link** selected nodes (X + Y, X only or Y only -- linked nodes move
  together; the threshold can join X or X + Y links), **Unlink**, **Snap to
  Diagonal** (the node then only slides along the diagonal) and
  **Delete**.
- **Double-click a handle** to make the node a corner again. **Delete** or
  Backspace removes the selected nodes.
- **Node List** shows every node's tag, In/Out level and handle as
  editable values (type, drag or scroll); **+ Add node** inserts one on the
  current curve. **Reset Curve** returns to the straight diagonal.
- The **Threshold** is a separate vertical line; the background shades the
  side where the trigger is met.

### Trigger Input...

The trigger's source is chosen with **Trigger: Main Input / Sidechain
In** in the toolbar. The **Trigger Input...** panel holds **Trigger
Gain** (-24 to +60 dB -- a narrow filter band needs a lot),
**Lookahead**, the Highpass/Lowpass **Trigger Filter** (with per-channel
corners in Dual Mono / Mid/Side), a scope of the trigger signal and
**Listen** (soft-limited). For example: a gate that only opens on one
frequency band of the material passing through it.

### Stereo modes

Like Compressor Stereo: **Linked**, **Dual Mono** or **Mid/Side**, with a
second curve and control bank in the latter two (**Link** in Dual Mono,
**Listen** in Mid/Side), per-curve **Wet/Dry**, **Global Input Gain** and
**Output Gain**. The curve, threshold and trigger settings take part in
A/B, Undo/Redo and presets like every other control.

### Contour Mono

The same plugin built for one channel: one main input, one mono sidechain
input, one output, one curve (no stereo modes). It is built from the same
source files as Contour, so every Contour change applies to both; presets
are interchangeable (the mono version ignores a stereo preset's second
curve).

## How it works

- `plugin/dsp/CompressorDsp.hpp` -- the shared core: a linear-domain peak
  envelope follower (asymmetric attack/release) driving a soft-knee gain
  computer (Giannoulis/Massberg/Reiss), `CompressorVoice`, meters,
  the sidechain band filter and the stereo modes. Mid/Side is a lossless
  decode/re-encode.
- `plugin/dsp/TruePeakLimiterDsp.hpp` -- the Limiter's True Peak algorithm
  (sliding-window minimum over the lookahead, a hard guarantee).
- `plugin/dsp/LimiterDsp.hpp` -- the Limiter's gain staging, clip stage,
  ratio choices and 4x oversampled peak detector.
- `plugin/dsp/CurveDsp.hpp` -- Contour's curve (`TransferCurve`),
  `CurveVoice` and the curve's text serialization.

See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for details.

## Building

From the repository root, `./build_and_install.sh` builds all projects.
To build only this one:

```sh
./build_and_install.sh               # build and install the LV2 bundles into ~/.lv2/
./build_and_install.sh --test        # also run the tests
./build_and_install.sh --clean       # start from an empty build directory
./build_and_install.sh --no-install  # build only
```

The build produces, for `compressor_mono`, `compressor_stereo`,
`limiter`, `contour` and `contour_mono`, an LV2 bundle, a VST3 bundle and
a standalone JACK program. On macOS use `./build_and_install_macos.sh`
(run on a Mac; needs the Xcode Command Line Tools), which builds
universal LV2/VST3/AU and installs them into `~/Library/Audio/Plug-Ins/`.

### DPF patches

`dpf/` is the official DPF, unchanged. `dpf-patches/` holds two small
patches the build scripts apply before building and revert afterwards
(`dpf-patches/dpf_patches.sh`): LV2 editors ask the host to resize
(`ui:resize`) when they resize themselves, and automatic scaling can be
relative to a design size, so editors can be made smaller than it. A plain
`cmake` build without the scripts needs them applied first:
`dpf-patches/dpf_patches.sh apply dpf <some-state-dir>`.

## Testing

- `dsp_regression` (runs with `--test`): gain computer, envelope
  follower, meters, Mid/Side algebra, voice consistency, the True Peak
  guarantee, the Limiter's gain staging, clip stage, ratio and peak detector, and
  Contour's curve math.
- `audio_testbench`: real synthetic audio through the DSP chain (accuracy,
  attack/release timing, stereo modes); writes WAV files to
  `tools/testbench/output/`.
- Plugin-level tests, built by hand (command in each file's header):
  `tests/stereo_trigger_test.cpp`, `tests/stereo_heartbeat_test.cpp`,
  `tests/contour_trigger_test.cpp`, `tests/contour_mono_test.cpp`,
  `tests/contour_widget_sim.cpp`, `tests/param_strip_sim.cpp`,
  `tests/limiter_test.cpp`, `tests/limiter_bypass_test.cpp`,
  `tests/limiter_widget_sim.cpp`.

See [docs/TESTING.md](docs/TESTING.md).

## Project layout

```
plugin/dsp/                  DSP core: CompressorDsp.hpp, TruePeakLimiterDsp.hpp, LimiterDsp.hpp, CurveDsp.hpp, SpectrumAnalyzer
plugin_mono/                 Compressor Mono plugin
plugin_stereo/               Compressor Stereo plugin
plugin_limiter/              Limiter plugin
plugin_contour/              Contour plugin (shared with Contour Mono)
plugin_contour_mono/         Contour Mono's DistrhoPluginInfo.h
gui/                         editors (UICompressorMono/Stereo, UILimiter, UIContour) and their widgets
tests/                       tests (see above)
tools/testbench/             audio_testbench, contour_preset_probe
docs/                        ARCHITECTURE.md, TESTING.md
dpf/                         DISTRHO Plugin Framework (git submodule)
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
