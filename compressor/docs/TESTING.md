# Testing

## `dsp_regression` (`tests/dsp_regression.cpp`)

Pure-math checks of the DSP headers, built with CMake into `build/bin/`
and run by `./build_and_install.sh --test`. No DPF/GUI dependency; runs
in well under a second. 49 numbered checks:

- **Compressor core** ([1]–[10]): the soft-knee curve (zero below the
  knee, the ratio line above it, continuous through the knee, 1:1 never
  compresses, upward compression symmetric to downward), the envelope
  follower's step response, the loudness meter's settled value, lossless
  Mid/Side reconstruction, `CompressorVoice` against the static curve, the
  True Peak limiter never exceeding its threshold, and its release curve.
- **Contour's curve** ([11]–[35], [42], [45]): identity default and fixed
  end points, sorted insert/remove, linear segments and extrapolation,
  Bezier handles (through the point, exact against a direct Bezier solve,
  sign symmetry, neighbour limits), Above/Below gating, step/gate pairs,
  rising/falling nodes and hysteresis, link groups and link axes, threshold
  links, snap to diagonal, serialization round trips, the packed trail and
  node Parameters, continuity at the ends, and migration of curves from
  the older -60 dB window.
- **Contour's voice** ([16], [21], [36]–[38], [41], [43], [44]): settling
  to the curve, the trigger action (Curve/Diagonal) with Attack and
  Release, lookahead, the audio-level mapping (the trigger only switches),
  exact switching at the threshold with zero times, and low distortion on
  smooth curves.
- **Trigger filter and Listen** ([34], [39]): the band filter's -3 dB
  corners and slopes, and the Listen soft limiter.
- **Limiter** ([46]–[49]): the gain staging (Output Gain/Boost capped at
  the Ceiling or the soft clip's knee start, Ceiling below Threshold, a
  full parameter sweep never above the target), the soft clip curve
  (identity to the knee, slope 1 there, monotonic, odd, bounded) and the
  4x + parabola peak detector against sine peaks up to 22.5 kHz, and the
  Classic ratio choices (∞:1 exactly flat above the knee).

## `audio_testbench` (`tools/testbench/audio_testbench.cpp`)

Runs synthetic audio through a per-sample chain wired exactly like the
Compressor plugins' `run()` (if you change one, change the other) and
writes WAV files to `tools/testbench/output/` for listening. Run it from
the project root:

```sh
./build/bin/audio_testbench
```

- **Test A -- steady-state accuracy**: 1 kHz tones at several levels. The
  engine's reported gain reduction and the actual output/input RMS ratio
  must agree with each other and with the static curve (fast attack). With
  a musical 5 ms attack both read less reduction than the naive peak
  estimate -- expected, since the follower tracks the envelope, not single
  cycles -- and must still agree with each other.
- **Test B -- timing**: on a sine burst, attack and release reach 63 % of
  their target within a tolerance of the configured times.
- **Test C -- stereo modes**: with a hard-panned signal, Linked keeps the
  silent channel silent, Dual Mono's second detector reports no reduction;
  with loud Mid and quiet Side content, Linked preserves the L/R ratio
  while Mid/Side leaves the quiet Side untouched and compresses the Mid.

## Plugin-level tests (built by hand)

These compile the real plugin class (with DPF's `Plugin` base and
`-fno-access-control`) or the real widget code against a drawing stub;
the exact build command is in each file's header.

| Test | Checks |
|---|---|
| `tests/stereo_trigger_test.cpp` | Compressor Stereo: sidechain gain, filter and scope act on the internal signal with Sidechain Internal, on the key with External; `dump` prints output checksums to compare two builds bit for bit. |
| `tests/stereo_heartbeat_test.cpp` | Compressor Stereo: the spectrum analysers run only while the editor's heartbeat arrives. |
| `tests/contour_trigger_test.cpp` | Contour: Trigger Gain and Trigger Filter act on the internal signal. |
| `tests/contour_mono_test.cpp` | Contour Mono: 2 in / 1 out, sidechain, bypass, lookahead; same behaviour as Contour's Linked mode. |
| `tests/contour_widget_sim.cpp` | Contour's graph: mouse handling (selection windows, handles, links) with synthetic events. |
| `tests/limiter_bypass_test.cpp` | Limiter: an impulse comes out exactly the reported latency later, Bypass off and on, separate and in-place buffers. |
| `tests/limiter_test.cpp` | Limiter: output never over the Ceiling with Boost/Output Gain, clip stage idle in True Peak mode, Soft vs Hard on Classic's overshoot, inter-sample peaks with Oversampling off/on against a 16x reference, reported latency == real delay (Bypass too), Classic's Ratio on sustained material, timing. |
| `tests/limiter_widget_sim.cpp` | Limiter's graph: the Clip Knee line, the Ceiling line and the Threshold handle each grab their own drags; True Peak drawn flat; the "limiter only" line. |
| `tests/param_strip_sim.cpp` | Value fields: how far a value moves per pixel of drag, including from 0. |
| `tests/check_ui_ctor.py` | Static check that every widget pointer of a UI class is created exactly once at the top level of its constructor. |

`tools/testbench/contour_preset_probe.cpp` loads a Contour preset, runs
sine signals through the real `run()` and reports distortion, gain jumps
and the step at the threshold; `plot_contour_probe.py` plots the results.
