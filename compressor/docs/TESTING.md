# Testing

Two dependency-free tools, both built as normal CMake targets alongside
the plugins and landing in `build/bin/` -- neither has any DPF/GUI
dependency, both build and run in well under a second.

## `dsp_regression` (`tests/dsp_regression.cpp`)

Fast, pure-math checks against `plugin/dsp/CompressorDsp.hpp`'s functions
in isolation: the soft-knee gain-computer curve (below/above/at the knee
edges, and that ratio 1:1 never compresses), the envelope follower's
exponential step response against its closed-form expectation, the
loudness meter's settled reading against a known constant amplitude, the
compensation gain converging to (and clamping beyond) a known measured
gap, the Mid/Side transform's lossless algebra, and an end-to-end
`CompressorVoice` self-consistency check, plus (added when
`CompensationGain` was redesigned to measure-then-hold rather than track
continuously) that a Held value doesn't drift when different audio plays
through it and that `beginMeasurement()` picks up a fresh gap on request.
12/12 checks.

```sh
./build/bin/dsp_regression
```

## `audio_testbench` (`tools/testbench/audio_testbench.cpp`)

Runs actual synthetic audio through the *real* per-sample chain --
`processMono()`/`processStereo()` in that file are wired exactly the way
`PluginCompressorMono::run()`/`PluginCompressorStereo::run()` are (a
comment in each says so; if you change one, change the other) -- rather
than checking formulas in isolation the way `dsp_regression` does. Writes
WAV files to `tools/testbench/output/` (git-ignored, regenerable) so the
actual sound can be listened to, not just measured. Run it from the
project root (its output path is relative, unlike the sibling project's
`audio_testbench`, which has CMake inject an absolute source-dir define --
not needed here since this tool has no factory presets to locate).

```sh
./build/bin/audio_testbench
```

16 checks across four areas:

- **Test A -- steady-state gain reduction accuracy.** A 1kHz sine tone at
  several levels, processed and measured two independent ways: the
  engine's own reported `gainReductionDb()`, and a plain RMS ratio of the
  actual output/input samples (never touches `gainReductionDb()` at all)
  -- both must agree with each other and with
  `computeGainReductionDb()` evaluated at the tone's true peak level, but
  *only once the envelope's attack time constant is short relative to the
  tone's own quarter-period* (Part 1, a synthetic `0.05ms` attack). A
  second case (Part 2) deliberately uses a musical `5ms` attack against
  the same `1kHz` tone and documents the real, expected consequence: the
  peak follower can only partially rise within each cycle, so both
  measurements read *less* reduction than the naive full-peak estimate --
  and are still required to match *each other* exactly, proving the
  real audio-rate chain is self-consistent even where it deliberately
  doesn't track a single cycle's instantaneous peak. This is normal,
  correct compressor behavior (a fast musical attack time responds to a
  signal's amplitude *envelope* over tens of milliseconds, not to
  individual audio-rate cycles) -- pinned down explicitly here rather than
  either silently assumed away or mistaken for a bug.
- **Test B -- attack/release timing on a real sine burst.** A quiet lead-in,
  a loud sustained burst, a quiet tail; measures the time for gain
  reduction to cross 63% of the way to its settled value on both the
  attack and release edges, checked against the configured times with a
  generous (real-audio, not closed-form-DC) tolerance.
- **Test C -- volume compensation measures a value you can apply as output
  gain, and stays put.** Compensation no longer touches the audio path at
  all (see `docs/ARCHITECTURE.md`'s "Where Volume Compensation actually
  applies") -- so this tests the real two-step workflow, not a single
  all-in-one call: a "measure" pass (compensation enabled, Output Gain at
  0) captures the settled `compGainDb`, then an "applied" pass feeds that
  value in as Output Gain (compensation off, exactly what
  `UICompressorMono`/`Stereo` do automatically the instant a measurement
  lands). Output RMS loudness must land within 1dB of the input's, across
  three very different thresholds. A sanity check confirms the same
  threshold sweep *does* visibly change loudness without that second step
  -- otherwise this test would pass vacuously. A further check feeds a
  loud tone (letting compensation measure and hold) followed immediately
  by a much quieter one in the same run, and confirms the *reported*
  compensation-gain trace is bit-for-bit identical through the quiet
  section -- it doesn't drift once Held, matching `CompensationGain`'s own
  `dsp_regression` guarantee end to end.
- **Test D -- the three stereo modes are actually distinct.** A hard-panned
  signal confirms Linked keeps a silent channel exactly silent (shared
  gain) and Dual-Mono's independent detector on that same silent channel
  reports exactly zero reduction throughout. A second signal (loud "Mid"
  content plus much quieter "Side" content, both proportional multiples of
  one waveform) confirms Linked preserves the exact L/R ratio (a single
  shared gain can't do otherwise), while Mid/Side leaves the quiet,
  below-the-knee Side content untouched but substantially compresses the
  loud Mid content -- the concrete, audible thing "Mid/Side narrows or
  widens the stereo image" means, and the actual way this mode differs
  from the other two.

## What real WAV files exist for

`tools/testbench/output/*.wav` (regenerated by every `audio_testbench`
run): a mono attack/release burst (dry + wet), a hard-panned stereo signal
processed through all three modes, and a "wide" (loud-center/quiet-side)
stereo signal through Linked and Mid/Side -- listen to these directly to
sanity-check that the numeric checks above correspond to what you'd
actually expect to hear, not just what the assertions say.

There's no reference/target audio file in this repo to compare against
(unlike the sibling project's `fit_gain_curve`, which learns a curve from
example audio) -- `Wav.hpp` includes a WAV *reader* as well as a writer for
exactly that purpose if it's ever useful later (e.g. feeding a real
reference recording through the same harness), but nothing currently calls
it.
