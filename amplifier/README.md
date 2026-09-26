# MimoMusic Amplifier (Mono + Stereo)

> New here? This is the third sibling project alongside
> `../statespace` (the state-space filter/EQ family)
> and `../compressor` (dynamics: Mono/Stereo Compressor, Limiter) -- same
> toolkit (DPF), same build conventions, own repo because a plain static-
> gain amplifier doesn't fit either sibling's own branding/theme.

Two sibling plugins sharing one tiny DSP core:

- **Amplifier Mono** (1-in/1-out) -- **Gain** (dB), **Bypass**,
  a **Listen/Normalize** feature, and a live **Clipping** indicator.
- **Amplifier Stereo** (2-in/2-out) -- **Gain** (dB, shared by
  both channels), independent **Invert L**/**Invert R** polarity switches
  (each channel on its own, not one combined flip), a **Balance** control,
  **Bypass**, the same **Listen/Normalize** feature, and the same live
  **Clipping** indicator.

Both plugins started with **no custom GUI at all**
(`DISTRHO_PLUGIN_HAS_UI 0` in both `DistrhoPluginInfo.h`) -- every LV2/VST3
host falls back to its own generic parameter-list editor, which was
originally all a handful of plain numeric/boolean controls needed. That
changed once Listen/Normalize needed a live level readout and a start/
stop control no host's own generic editor can give it -- see "Listen/
Normalize" below, and `DistrhoPluginInfo.h`'s own comment for the "no GUI,
then actually yes" history. Both plugins' GUIs (`gui/UIAmplifier*.*`)
reuse `SimpleButton`/`ParamStripWidget` ported verbatim from
`../compressor`, the same way that project's own widgets were ported from
`../statespace` in turn.

### Listen/Normalize

Click **Listen** to audition the raw, unprocessed input (Gain/Balance/
Invert/Bypass are all bypassed while it's on) and start measuring it --
**Peak** or **RMS**, your choice (a two-way swap button next to Listen).
The live reading updates continuously in the meter box while Listen stays
on, for as long as you want to listen. Click **Listen** again to stop --
that's the moment it actually does something: **Gain is set automatically**
so the level you just measured lands exactly on **Normalize To** (default
-1dB), clamped to Gain's own [-60, +24]dB range -- Listen's whole job is
landing on the right Gain, nothing more: it's a completely ordinary,
still-adjustable Gain value afterward, ready to nudge further by hand if
you want, not a separate hidden stage layered on top of it.

The actual "compute the new Gain" step happens in the GUI, not the DSP --
a plugin has no standard way to tell an LV2/VST3 host "one of your own
input parameters changed internally" (a host only ever pushes values *to*
a plugin for an input port, never polls one back), so the DSP side only
ever measures and reports; `gui/UIAmplifier{Mono,Stereo}.cpp`'s own
`applyListenResult()` is what turns that report into an actual new Gain,
via the UI's own `setParameterValue()` -- the same real, reliable
round trip to the host an ordinary knob turn in this GUI already uses.

### Clipping indicator

A red **CLIP** badge lights up on the meter box (its border also turns
red) whenever the signal actually leaving the plugin -- through Gain, or
straight through while Bypassed or Listening -- exceeds 0dBFS (`|sample|
> 1.0`), and stays lit for a short 1.5s hold after the last such sample so
a single hot transient is actually visible rather than flickering on and
off between GUI repaints. This is deliberately separate from the Listen/
Normalize reading right below it in the same box: that one is a frozen
snapshot of the *last completed audition*, exactly the wrong shape for
"is the current Gain setting too hot right now". The indicator is live in every mode (Gain-processing, Bypass,
and Listen alike), independent of whether a Listen session happens to be
running at the same time.

Note this plugin never itself hard-clips (limits) the signal to ±1.0 --
it's a plain gain stage, so a hot setting really can produce
past-full-scale float samples handed on to whatever's next in the chain.
The badge is a warning that this is happening, not a limiter; pull Gain
back (or lower Normalize To before a fresh Listen pass) if you don't want
that.

### Build and install

```sh
./build_and_install.sh    # does everything below, plus installs into ~/.lv2/
```

or by hand:

```sh
git submodule update --init --recursive   # first time only (pulls DPF + its own pugl submodule)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Outputs land in `build/bin/`, for each of `amplifier_mono` and
`amplifier_stereo`:
- `<name>.lv2/` -- LV2 bundle (copy to `~/.lv2/`)
- `<name>.vst3/` -- VST3 bundle (copy to `~/.vst3/`)
- `<name>` -- standalone JACK app, useful for quick manual testing without
  a full DAW (skipped on macOS -- see `CMakeLists.txt`'s own comment)

On macOS, use `./build_and_install_macos.sh` instead (universal binary,
adds an AU/Component build too) -- see that script's own header comment.

## The DSP (`plugin/dsp/AmplifierDsp.hpp`)

Deliberately tiny -- a static-gain amplifier's whole per-sample job is a
couple of scalar multiplies, computed once per audio block (not
recomputed per sample) since none of Gain/Invert/Balance change mid-block:

```
gainLin = 10^(GainDb/20)
(balL, balR) = balanceGains(Balance)     -- see below
totalL = gainLin * balL * (InvertL ? -1 : 1)
totalR = gainLin * balR * (InvertR ? -1 : 1)
outL[i] = inL[i] * totalL
outR[i] = inR[i] * totalR
```

**Balance** uses the classic mixing-console law, not a full pan: centered
(`0`) leaves both channels at unity gain; moving toward one side
*attenuates the other channel only*, linearly down to silence at the
extreme (`±1`) -- the side you move toward is never boosted past unity.
That's deliberately different from panning a mono source across a stereo
field (which typically *does* push one side above unity, or uses an
equal-power law) -- Balance here only ever corrects an already-stereo
signal's relative loudness between its two sides.

**`ListenMeter`** is the whole DSP side of Listen/Normalize: not a real-
time-ballistics meter (no decay, no hold, no time constant) -- it measures
the *entire* listened section as one whole, exactly like a normalize
function in an audio editor would. Peak is the single loudest absolute
sample seen since the last `reset()`; RMS is the true root-mean-square
energy of every sample seen since then, not a moving average. Both are
available continuously (`peakDb()`/`rmsDb()`) so the GUI's number can keep
updating live while Listen is still running. For the stereo plugin, every
sample of *both* channels feeds the same meter -- its running max is
exactly the Linked `max(|L|,|R|)` convention every dynamics plugin in the
sibling projects already uses (a global max over two channels' own maxima
is the max of their per-frame `max(|L|,|R|)` sequence), and its running
average is a genuine combined-channel RMS.

**`ClipHold`** is the whole DSP side of the Clipping indicator, and is the
opposite shape from `ListenMeter` on purpose: instead of accumulating over
an entire listened section, it's checked once per processed block against
whatever `run()` just wrote to its own output (`ClipHold::bufferClips()`,
`|sample| > 1.0`) and combines that into a single `secondsSinceClip_`
counter (`update()`) that only ever resets to zero on a fresh clip and
otherwise counts up in wall-clock time -- `isClipping()` is just "is that
counter still under the 1.5s hold window". Cheap enough to run
unconditionally on every block regardless of Gain/Bypass/Listen state.

## Testing

```sh
./build/bin/dsp_regression   # fast, pure-math checks -- dbToLinear, the
                              # balance law (centered = unity, attenuates
                              # only the far channel, clamps out-of-range
                              # input), the combined per-sample formula,
                              # ListenMeter (constant-signal Peak/RMS, the
                              # ~3.01dB sine crest factor, Peak-never-drops),
                              # the Listen-stop Gain formula's own clamps,
                              # and ClipHold (the 0dBFS threshold itself,
                              # its hold-then-clear timing, and reset())
```

No DPF/GUI dependency -- builds automatically alongside the plugins.

## Project layout

```
plugin/dsp/AmplifierDsp.hpp    framework-independent DSP core (header-only):
                                 dbToLinear(), balanceGains(), ListenMeter,
                                 ClipHold
plugin_mono/                    Amplifier Mono's DPF plugin (DSP) side
plugin_stereo/                  Amplifier Stereo's DPF plugin (DSP) side --
                                 own DistrhoPluginInfo.h, same include-
                                 ambiguity reason plugin_mono/ has its own
gui/                             DPF UI (GUI) side, NanoVG-drawn widgets:
                                 SimpleButton, ParamStripWidget (both ported
                                 verbatim from ../compressor), plus
                                 UIAmplifierMono/UIAmplifierStereo
build_and_install.sh            builds everything, installs the LV2 bundles to ~/.lv2/
build_and_install_macos.sh      macOS counterpart -- universal binary, adds AU
tests/dsp_regression.cpp        fast pure-math correctness tests, no DPF/GUI dependency
dpf/                             DISTRHO Plugin Framework (git submodule)
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
