# MimoMusic Meters (Phase Scope, Loudness Meter)

Two stereo measurement plugins. Both are purely observational: the audio
passes through unchanged.

- **Phase Scope** (2 in / 2 out) -- goniometer, phase correlation, stereo
  width and level meters.
- **Loudness Meter** (2 in / 2 out) -- loudness measurement to ITU-R
  BS.1770-4 / EBU R128 / EBU Tech 3342.

Hover over any button or field in either editor for a short description.

## Phase Scope

### Displays

- **Goniometer**: a rotated Mid/Side vectorscope, M pointing up, L and R
  on the upper diagonals. The trail fades out over the Decay time, and
  its colour shifts from green to amber to red as the phase correlation
  drops from +1 to -1.
- **Correlation meter**: -1 to +1 (about 300 ms response), with a
  worst-case marker that holds a new minimum for 1.5 s and then slowly
  recovers, so short phase problems stay visible.
- **Stereo width**: 0 to 200 %. 0 % is mono, 100 % ordinary uncorrelated
  stereo, 200 % pure Side (out-of-phase) content.
- **L/R level meters**: peak and RMS with clip indicators.

### Controls (display only -- they never touch the audio)

| Control | Range / default | What it does |
|---|---|---|
| **Gain** | -24 to +24 dB | Zooms the goniometer in or out. |
| **Auto Gain** | on / off | Keeps adjusting Gain from a smoothed (about 2.5 s) level reading so the signal fills the plot; switching it off leaves Gain where it is. |
| **Decay** | 100–3000 ms, default 600 | Length of the visible trail. A longer Decay shows more points, not the same points spread thinner. |
| **Draw: Dots / Lines** | default Lines | Dot cloud or connected vector trace. |
| **Smooth** | default on (Lines only) | Draws a spline through the points instead of straight segments. |
| **Res** | 10–100 %, default 100 | Fraction of the points drawn -- lower it on a slow machine; nothing is lost, raising it shows the full trail again. |
| **Heatmap** / **Heat** | off; 1–30 s, default 5 | A long-term density glow under the trail, showing where the signal has spent its time. It keeps recording while hidden and follows Gain changes. |
| **Freeze** | | Pauses the display. |
| **Compact** / **Full view** | off | A small window (260 x 300 at 1x scale) with only the scope and the correlation bar, to keep open next to other windows. Its Freeze and Full view buttons appear while the mouse moves over it and hide 2 s after it stops. Remembered with the session. |

## Loudness Meter

### Readings

- **Momentary / Short-term / Integrated** (LUFS): 400 ms, 3 s and the
  whole measurement. Integrated uses the standard gating (absolute
  -70 LUFS, relative -10 LU), so silence and quiet passages don't pull it
  down.
- **Loudness Range** (LRA, LU): 95th minus 10th percentile of the gated
  Short-term values (EBU Tech 3342).
- **True Peak** (dBTP) per channel and maximum, from 4x oversampling (a
  general-purpose interpolator, not the ITU reference filter bit for bit).
- **PLR / PSR** (LU): True Peak maximum minus Integrated, and minus the
  current Short-term value.
- **L/R bars**: plain per-channel peak/RMS in dBFS.

### Display

- **Bar meter**: Momentary as a filled bar, Short-term as a pointer on
  its edge, plus Short-term and Integrated as horizontal markers on the
  same scale.
- **History graph**: Momentary and Short-term over time as filled
  curves. Scroll to zoom (2 s up to 6 h, default 60 s), drag to go back
  in time, double-click to return to the live view.
- **View: Graph / Distribution**: switches the centre panel to a
  distribution plot -- how often each loudness level occurred during the
  measurement -- with reference lines for common streaming targets.
  Scroll to zoom around the mouse, double-click to reset.
- **Target**: a reference line on all views: Off, -23 (EBU R128),
  -24 (ATSC A/85), -14 (Spotify), -16 (Apple Music), -14 (YouTube) or
  -9 (SoundCloud -- an unofficial mastering reference; SoundCloud
  publishes no normalization target). Display only.
- **Reset**: clears Integrated, LRA, the distribution and the True Peak
  maximum for a new measurement (Momentary and Short-term keep running).
  The readout flashes briefly to confirm.

## How it works

`plugin/dsp/PhaseScopeDsp.hpp` holds the Phase Scope DSP: level meters,
correlation and width from one shared set of leaky sums, the packed
goniometer ring buffer and the heatmap grid. `plugin/dsp/LoudnessDsp.hpp`
holds the loudness chain: K-weighting filter, exact sliding mean square,
the gating histograms for Integrated and LRA, the distribution tally and
the true-peak detector. See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
for details.

## Building

From the repository root, `./build_and_install.sh` builds all projects.
To build only this one:

```sh
./build_and_install.sh               # build and install the LV2 bundles into ~/.lv2/
./build_and_install.sh --test        # also run the tests
./build_and_install.sh --clean       # start from an empty build directory
./build_and_install.sh --no-install  # build only
```

The build produces, for `phase_scope` and `loudness_meter`, an LV2
bundle, a VST3 bundle and a standalone JACK program. On macOS use
`./build_and_install_macos.sh` (run on a Mac), which builds universal
LV2/VST3/AU and installs them into `~/Library/Audio/Plug-Ins/`.

### DPF patches

`dpf/` is the official DPF, unchanged. `dpf-patches/` holds two small
patches the build scripts apply before building and revert afterwards
(`dpf-patches/dpf_patches.sh`): LV2 editors ask the host to resize
(`ui:resize`) when they resize themselves, and automatic scaling can be
relative to a design size, so editors can be made smaller than it. A plain
`cmake` build without the scripts needs them applied first:
`dpf-patches/dpf_patches.sh apply dpf <some-state-dir>`.

## Testing

`dsp_regression` (built with the plugins, no GUI needed) checks the
correlation and width formulas, the worst-case hold, the goniometer
buffer, the level meter ballistics and the loudness pipeline (K-weighting,
gating, true peak, LUFS). See [docs/TESTING.md](docs/TESTING.md).

## Project layout

```
plugin/dsp/PhaseScopeDsp.hpp   Phase Scope DSP (meters, correlation/width, goniometer buffer, heatmap)
plugin/dsp/LoudnessDsp.hpp     Loudness Meter DSP (K-weighting, gating, LRA, true peak)
plugin_phasescope/             Phase Scope plugin
plugin_loudness/               Loudness Meter plugin
gui/                           editors (UIPhaseScope, UILoudnessMeter) and their widgets
tests/dsp_regression.cpp       tests
docs/                          ARCHITECTURE.md, TESTING.md
dpf/                           DISTRHO Plugin Framework (git submodule)
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
