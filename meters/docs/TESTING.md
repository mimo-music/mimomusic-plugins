# Testing

```sh
./build/bin/dsp_regression
```

## `dsp_regression` (`tests/dsp_regression.cpp`)

Pure-math checks of both DSP cores (`plugin/dsp/PhaseScopeDsp.hpp`,
`plugin/dsp/LoudnessDsp.hpp`), no DPF/GUI dependency, built with CMake and
run by `./build_and_install.sh --test`. Many properties are exact by
construction, so one fast suite covers them without an audio testbench.
26 checks:

### Phase Scope

- **Correlation, exact** ([1], [2]): L = R gives exactly +1, L = -R
  exactly -1.
- **Width, exact** ([3], [4]): mono gives exactly 0 %, out-of-phase
  exactly 200 % -- this validates deriving width from the same three sums
  as the correlation.
- **Settled case** ([5]): an uncorrelated equal-power pair settles near
  0 correlation and 100 % width.
- **Worst-case hold** ([6], [6b]): the hold drops to the worst value and
  recovers slowly; it stays exactly flat for less than one plateau of
  good-phase signal and moves once the plateau has passed.
- **Goniometer buffer** ([7], [8], [9], [9b], [9c]): packed points
  round-trip within one quantization step and fit a float's exact-integer
  range; the buffer fills in exactly `kMaxDecaySeconds` at 44.1, 48 and
  96 kHz; the write rate doesn't depend on Decay; `visibleCountForDecay()`
  is monotonic, roughly proportional and reaches the full buffer at the
  maximum Decay.
- **Auto Gain** ([9d], [9f]): the gain arithmetic matches the closed-form
  target; `SmoothedLevelMeter` settles to the exact input level and a
  single sample barely moves it.
- **Heatmap** ([9e], [9g]): a shorter Decay fades faster, the cell packing
  round-trips; `rescale()` with ratio 1 changes nothing, moves a mid-plot
  hit while conserving the total, and drops a hit scaled off the plot.
- **Meter ballistics** ([10], [11]): the RMS meter settles to the exact
  RMS, the peak meter holds the peak and decays at about 20 dB/s.

Resolution and Smooth exist only in the GUI widget and aren't covered.

### Loudness Meter

- **Windows and gating** ([12], [13]): the sliding window settles to the
  exact mean, partially and fully filled; a steady distribution gates to
  its exact loudness with an LRA of 0 LU, and a block below -70 LUFS is
  excluded.
- **Filters** ([14], [15], [16]): the K-weighting shape (strong cut well
  below ~38 Hz, boost above ~1.7 kHz); DC reads back exactly as its true
  peak; a Nyquist-rate square wave's true peak is at or above its sample
  peak within a sane bound.
- **Full pipeline** ([17]): a steady tone gives agreeing Momentary and
  Short-term values and an LRA of about 0 LU; `resetStatistics()` clears
  Integrated but leaves Momentary untouched.
- **Distribution** ([18], [18b]): values land in the right bin, out-of-range
  values clamp to the end bins, reset clears everything; a steady tone
  concentrates in one bin; silent blocks (digital zero or inaudibly quiet)
  never enter the distributions.
