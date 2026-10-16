# MimoMusic Reverb (Reverb Dense, Reverb Plate)

Two stereo reverbs with the same controls and editor, and different
reverb engines:

- **Reverb Dense** (2 in / 2 out) -- a 16-line feedback delay network
  (FDN) with densely spaced modes, for a smooth, grain-free tail.
- **Reverb Plate** (2 in / 2 out) -- a Dattorro-style plate: a figure-8
  network of two coupled delay loops with all-passes -- a different
  topology with a different character.

Pick whichever character suits the material.

## Controls

| Control | Range / default | What it does |
|---|---|---|
| **Length / Width / Height** | 1–80 m, 1–80 m, 1–40 m; default 10 × 7 × 3.5 m | Room dimensions; they set the delay lengths of the reverb. |
| **Input Gain** | -24 to +24 dB | Applied to the input before it splits into dry and reverb. |
| **Pre-Delay** | 0–250 ms | Delays the reverb signal before it enters the reverb. |
| **Decay Time** | 0.1–20 s, default 1.8 | Reverb time (T60). |
| **Damping** | 0–1, default 0.5 | How much faster the high frequencies die away than the lows. |
| **Diffusion** | 0–1, default 0.7 | Strength of the input diffuser; higher is smoother and less metallic. |
| **Modulation** | 0–1, default 0.3 | Gentle delay modulation against metallic ringing. |
| **Spread** | 0–2, default 1 | Stereo width of the reverb: 0 mono, 1 natural, 2 wider. |
| **Balance** | -1 to +1 | Left/right balance of the reverb signal. |
| **Dry/Wet** | 0–100 %, default 35 | Mix of original and reverb. |
| **Output Trim** | -24 to +24 dB | Level of the reverb signal before the mix. |
| **Bypass**, **Reset** | | Pass the input through; clear the reverb tail. |

### Tempo sync

- **Tempo Sync: On** sets the decay time from the host's tempo and the
  **Note** value (1/16, 1/8, 1/4, 1/2, 1 bar, 2 bars, 4 bars). The live
  decay time is shown instead of the Decay Time field.
- **Follow Host Tempo**: Off takes the tempo once when Tempo Sync is
  switched on; On follows every tempo change of the host.
- The editor shows the tempo the host reports (120 BPM is used if the host
  reports none).

### EQ on the reverb signal

Below the controls is a parametric EQ that shapes only the reverb signal
(not the dry signal), with the same graph as the MimoMusic Parametric EQ:
click to add a band, drag to shape it, scroll for Q, right-click for the
band type. A spectrum of the reverb signal before and after the EQ is
drawn behind the curve (**Fixed Range / Autoscale**). The band list
beside the graph edits every band precisely; below it, the selected band
gets large Freq / Gain / Q fields.

## How it works

Both engines share the same parameters, pre-delay, diffuser and EQ
(`plugin/dsp/`); they differ in the reverb network itself. Decay and
damping are set per delay line from the target T60 (with a separate
high-frequency decay), so every line dies away at the same rate. A
room-size change is handed to the audio thread safely and rebuilds the
delay lines there. See
[docs/REVERB_CONCEPT.md](docs/REVERB_CONCEPT.md) for the full design.

`tools/reference/` holds a reference-sample generator (image-source room
simulation) used to compare the engines against physically modelled
rooms.

## Building

From the repository root, `./build_and_install.sh` builds all projects.
To build only this one:

```sh
./build_and_install.sh               # build and install the LV2 bundles into ~/.lv2/
./build_and_install.sh --test        # also run the tests
./build_and_install.sh --clean       # start from an empty build directory
./build_and_install.sh --no-install  # build only
```

The build produces, for `reverb_dense` and `reverb_plate`, an LV2 bundle,
a VST3 bundle and a standalone JACK program. On macOS use
`./build_and_install_macos.sh` (run on a Mac; needs the Xcode Command
Line Tools), which builds universal LV2/VST3/AU and installs them into
`~/Library/Audio/Plug-Ins/`.

### DPF patches

`dpf/` is the official DPF, unchanged. `dpf-patches/` holds two small
patches the build scripts apply before building and revert afterwards
(`dpf-patches/dpf_patches.sh`): LV2 editors ask the host to resize
(`ui:resize`) when they resize themselves, and automatic scaling can be
relative to a design size, so editors can be made smaller than it. A plain
`cmake` build without the scripts needs them applied first:
`dpf-patches/dpf_patches.sh apply dpf <some-state-dir>`.

## Testing

`--test` runs `reverb_regression`, `reverb_regression_dense`,
`reverb_regression_plate` and `predelay_regression`: decay time accuracy
per frequency band, stability, energy preservation of the diffusers,
thread safety of room-size changes, pre-delay timing and more.

## Project layout

```
plugin/ReverbParams.hpp      parameters and states shared by both plugins
plugin/dsp/                  reverb engines, pre-delay, EQ and spectrum DSP
plugin_dense/                Reverb Dense plugin
plugin_plate/                Reverb Plate plugin
gui/                         the shared editor (UIReverb) and its widgets, gui/eq/ for the EQ
tests/                       regression tests
tools/reference/             reference-sample generator
docs/REVERB_CONCEPT.md       design
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
