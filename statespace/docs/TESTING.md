# Testing

Four tools, built by the normal CMake build into `build/bin/`, using only
the framework-independent code in `plugin/dsp/` (no DPF/DGL):

| Tool | Purpose |
|---|---|
| `dsp_regression` | fast pass/fail checks (run by `./build_and_install.sh --test`) |
| `audio_testbench` | real audio through the real engine, for every factory preset |
| `fit_gain_curve` | learns a gain curve from example audio |
| `fit_multiband_compressor` | the same for N crossover bands |

## `dsp_regression`

```sh
./build/bin/dsp_regression
```

Runs in about a second and exits non-zero if anything fails.

- **Core** ([1]–[7]): identity passthrough is bit-exact; an unstable
  matrix is clamped and stays finite under full-scale input; an RBJ
  low-pass has the expected shape; JSON presets round-trip exactly;
  `normalizeToUnityPassbandGain()` hits 0 dB; a cascaded band limit has
  the right shape; a narrow 50 Hz notch (poles very close to the unit
  circle) really notches after the stability guard.
- **Gain curve** ([8], [9]): a hand-set `NonlinearSchedule` settles to the
  closed-form compressor gain and is transparent below threshold;
  `GainCurveTrainer::fit()` recovers a known schedule (attack/release
  exactly, curve within a fraction of a dB).
- **Parametric EQ** ([10]–[10e], [14], [25], [45]): the assembled
  state-space matrix and the closed-form curve the GUI draws agree;
  harmonic twins; per-channel bands, and soloing a Left-only band silences
  the right channel; the mono EQ produces exactly the stereo EQ's output
  on one channel with exactly half the states, and its guard agrees with
  the stereo guard; Low/High Pass slope stages; Band Shelf (flat
  plateau, exact 0 dB outside, symmetric cut).
- **Spectrum and engine** ([11]–[13], [27]–[29], [35]): the analyser reads
  pure tones back to the right level and band; same-size model changes
  glide on the expected one-pole path; size changes duck to silence and
  recover; the sparse cascade path matches the dense path exactly, also
  through a live edit; a second size change during a fade-in isn't lost.
- **Crossfeed** ([32]–[32d], [34], [36]–[39c]): L→R bands reach the other
  channel, the crossfeed gain is a flat baseline, older presets load, solo
  and rebuild paths, no numerical faults across frequencies, recorded
  click sequences, inactive crossfeed slots don't leak.
- **Multiband** ([15]–[18]): the Linkwitz-Riley bank sums flat at every
  slope (12/24/36/48 dB/oct) with -6.02 dB at each crossover and correct
  state bookkeeping; two bands settle to the closed-form gains;
  `fitMultiband()` recovers two known curves from one mix; the
  multiband model's matrix and closed-form curve agree, JSON round-trips,
  and adding/removing crossovers keeps the band count consistent.
- **Dynamic EQ** ([19]–[24], [26], [26b]): the resting shape, measured gain
  of a quiet tone, one-channel bands, muted channels, Audition's duck and
  its output, the static export, the bidirectional (Ratio/Up Ratio) gain
  law, inert bands staying transparent, output trim.
- **Dynamic EQ 2** ([33], [40]–[44], [46]): it matches Dynamic EQ exactly
  where their features overlap; rebuild path; harmonic twins; Low/High
  Shelf; linked bands share their dynamics and are auditioned together;
  Band Shelf.

## `tests/dyneq2_list_sim.cpp` (built by hand)

Drives Dynamic EQ 2's real band list (`DynEqBandListWidget2`) headlessly
against `tests/gui_stub/NanoVG.hpp` with a fake host: column layout, the
type/enable/solo/dynamics/channel controls, click-to-type and
double-click, "--" fields per type, drag and scroll mappings, delete and
tooltips. The build command is in the file's header.

## `audio_testbench`

```sh
./build/bin/audio_testbench [presets_dir output_dir]
```

Defaults to `presets/factory/` and `tools/testbench/output/` (absolute
paths compiled in), so it works from any directory. For every `.ssjson`
it generates a 6-second log sweep (20 Hz–20 kHz) and a set of steady probe
tones (about 25 frequencies, each held long enough for that model to
settle), runs them through the real `StateSpaceEngine` and:

- compares each tone's measured gain with `TransferFunction::evaluate()`
  (an independent frequency-domain solve) within 0.5 dB;
- checks a small per-preset table of expected gains (`spotCheckTable()`);
- for presets with the gain-curve stage enabled, skips the linear
  comparison and instead requires all equal-level probe tones to be
  compressed by the same amount (spread below 0.3 dB) -- the envelope is
  smoothed in the linear domain, so compression must not depend on
  frequency;
- checks the sweep output stays finite and bounded;
- writes all input and output signals as WAV files plus `report.txt` to
  the (git-ignored) output directory.

## `fit_gain_curve`

```sh
./build/bin/fit_gain_curve <base_preset.ssjson> <input.wav> <target_output.wav> <output_preset.ssjson>
```

Fits a `NonlinearSchedule` (attack, release and the 25-point curve) with
`GainCurveTrainer::fit()` so the base preset turns the input into the
target, prints attack/release, the residual error and the curve, and
writes a new, stability-checked preset. Against a simulated reference
compressor it recovers attack/release exactly and the curve within about
0.005 dB RMS. `presets/factory/trained_compressor_demo.ssjson` was
produced this way.

## `fit_multiband_compressor`

```sh
./build/bin/fit_multiband_compressor "<crossover_hz_csv>" <input.wav> <target_output.wav> <output_preset.ssjson> [lr_order=4]
```

Builds the Linkwitz-Riley crossover bank for the given ascending
crossover frequencies (at most 6 bands) and fits one `NonlinearSchedule`
per band with `GainCurveTrainer::fitMultiband()`. The result is an
ordinary `.ssjson`, loadable in Matrix Filter.
