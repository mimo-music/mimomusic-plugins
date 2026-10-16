# MimoMusic Amplifier (Mono / Stereo)

Two gain plugins with a built-in normalize function and a clipping
indicator.

- **Amplifier Mono** (1 in / 1 out)
- **Amplifier Stereo** (2 in / 2 out)

## Controls

| Control | Mono | Stereo | What it does |
|---|---|---|---|
| **Gain** | ✓ | ✓ | -60 to +24 dB. In the stereo version it applies to both channels. |
| **Invert L / Invert R** | | ✓ | Flips the polarity of each channel independently. |
| **Balance** | | ✓ | -1 to +1. Turning towards one side attenuates the *other* channel (down to silence at the extreme); the side you turn towards is never boosted. |
| **Bypass** | ✓ | ✓ | Passes the input through unchanged. |
| **Listen / Peak–RMS / Normalize To** | ✓ | ✓ | See below. |
| **Undo / Redo** | ✓ | ✓ | For the editor's changes. |

### Listen / Normalize

1. Click **Listen**. You hear the raw input (Gain, Balance, Invert and
   Bypass are skipped while listening), and the plugin measures it --
   **Peak** or **RMS**, selected with the button next to Listen. The
   reading updates live for as long as you listen.
2. Click **Listen** again to stop. **Gain is then set** so that the
   measured level lands exactly on **Normalize To** (default -1 dB),
   within Gain's range.

Gain stays an ordinary control afterwards; adjust it by hand as you like.
The measurement covers the whole listened section, like the normalize
function of an audio editor: Peak is the loudest sample, RMS the energy
of everything heard. In the stereo version both channels feed one
combined measurement.

### Clipping indicator

A red **CLIP** badge lights up whenever the plugin's output exceeds
0 dBFS, and stays on for 1.5 s after the last clipping sample so short
peaks are visible. It works in every state (processing, Bypass, Listen).
The plugin never limits the signal itself -- the badge is a warning, not
a limiter.

## How it works

The DSP is in `plugin/dsp/AmplifierDsp.hpp`: per block, Gain, Balance
and polarity are combined into one factor per channel,

```
gainLin = 10^(Gain/20)
outL = inL * gainLin * balanceL * (Invert L ? -1 : 1)
outR = inR * gainLin * balanceR * (Invert R ? -1 : 1)
```

`ListenMeter` measures the listened section (peak and true RMS over the
whole section), `ClipHold` drives the clipping badge. The new Gain after a
Listen pass is computed by the editor and sent to the host like any other
control change.

## Building

From the repository root, `./build_and_install.sh` builds all projects.
To build only this one:

```sh
./build_and_install.sh          # build and install the LV2 bundles into ~/.lv2/
./build_and_install.sh --test   # also run the tests
```

or by hand:

```sh
git submodule update --init --recursive   # first time only (DPF)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The build produces, for `amplifier_mono` and `amplifier_stereo`, an LV2
bundle (`<name>.lv2`), a VST3 bundle (`<name>.vst3`) and a standalone
JACK program. On macOS use `./build_and_install_macos.sh`, which also
builds AU components.

### DPF patches

`dpf/` is the official DPF, unchanged. `dpf-patches/` holds two small
patches the build scripts apply before building and revert afterwards
(`dpf-patches/dpf_patches.sh`): LV2 editors ask the host to resize
(`ui:resize`) when they resize themselves, and automatic scaling can be
relative to a design size, so editors can be made smaller than it. A plain
`cmake` build without the scripts needs them applied first:
`dpf-patches/dpf_patches.sh apply dpf <some-state-dir>`.

## Testing

`dsp_regression` (built with the plugins, no GUI needed) checks the gain
and balance law, the combined per-sample formula, the Listen measurement
(Peak/RMS, sine crest factor), the normalize Gain calculation and the
clipping indicator's threshold and hold time.

## Project layout

```
plugin/dsp/AmplifierDsp.hpp   DSP core: dbToLinear(), balanceGains(), ListenMeter, ClipHold
plugin_mono/                  Amplifier Mono plugin
plugin_stereo/                Amplifier Stereo plugin
gui/                          editors (UIAmplifierMono/Stereo) and their widgets
tests/dsp_regression.cpp      tests
dpf/                          DISTRHO Plugin Framework (git submodule)
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
