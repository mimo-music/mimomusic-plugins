# Extending: building processors on top of the state-space core

New linear processors are added as **coefficient mappers** -- plain code
that turns musical parameters into `A`/`B`/`C`/`D` entries -- without
touching the real-time engine, the stability guard, serialization or the
matrix editor. See `plugin/dsp/CoefficientMapper.hpp` for the convention
and worked examples.

This covers **linear, time-invariant (LTI)** processing. Level-dependent
processing needs the mechanisms described further down.

## Building an LTI processor -- EQ, crossovers, resonators, phase shaping

1. Design the filter in its natural form (biquad coefficients, pole/zero
   pairs, ...).
2. Realize it as a state-space block. `mappers::realizeBiquadInto()` is the
   template (controllable canonical form of one second-order section);
   `mappers::makeStereoBiquad()` packs one realization per channel into a
   block-diagonal order-4 model.
3. Combine blocks: cascade them in series (`makeStereoBiquadChain()`), or
   place them in parallel and sum their outputs in `C`/`D`.
4. Run the result through `StabilityGuard::enforce()` before handing it to
   `StateSpaceEngine::setModel()`. A well-designed filter is already
   stable, so this is normally a no-op, but it protects against extreme
   values (a Q of 500, say).

`plugin/dsp/EqModel.*` and the Parametric EQ (`plugin_eq/`, `gui/eq/`)
are this pattern in full: a band struct (type, frequency, gain, Q,
bandwidth, channel, crossfeed, link group), `eq::assembleStateSpace()`
doing steps 1–4, and an editor that works on the musical parameters. The
EQ's **Export to State-Space...** hands the assembled matrices to Matrix
Filter for inspection. Reading `EqModel.cpp` alongside
`CoefficientMapper.hpp` is the quickest way into writing a mapper.

## Why compression needs state-dependent coefficients

A state-space model with **constant** `A/B/C/D` cannot compress, at any
order. It is linear: if input `u` gives output `y`, input `k*u` gives
exactly `k*y`, for every `k`. A compressor exists to break that
proportionality. More states only buy more elaborate *frequency*-dependent
behaviour, never *level*-dependent behaviour.

The standard construction that adds level dependence without giving up the
model is **state-dependent coefficients** (a linear-parameter-varying
system): `A` and `B`, which determine the recursion and its stability,
stay fixed exactly as the stability guard validated them. Only `C` and `D`
vary, as `C(p[n])`, `D(p[n])`, with a scheduling variable `p[n]` measured
from the signal (here a smoothed input level).

The implementation uses the simplest case: `C(p) = gain(p) * C0`,
`D(p) = gain(p) * D0`, which equals multiplying the computed output by a
scalar `gain(p[n])`. See `NonlinearSchedule` / `GainCurve` in
`plugin/dsp/StateSpaceTypes.hpp` and the "NonlinearSchedule" section of
`docs/ARCHITECTURE.md`.

Before reaching for dynamics, check whether a use case is really a fixed
frequency-dependent gain -- that's an ordinary LTI mapper.

## Multiband compression

The same mechanism, once per frequency band:

1. **The crossover is an LTI mapper.** `mappers::makeStereoCrossoverBank()`
   splits the signal into up to `kMaxBands` (6) bands with a tree of
   Linkwitz-Riley two-way splits (`designCrossoverBandStages()`), at
   12/24/36/48 dB/octave. Each order uses its own Butterworth Q table, and
   the odd Butterworth orders invert the high-pass side, so the bands sum
   to exactly unity at every frequency -- with every band at 0 dB the
   compressor is a true passthrough. The bands are independent chains
   connected in parallel (block-diagonal `A`, `C` concatenated, `D`
   summed).
2. **Each band has its own `NonlinearSchedule`.** `MultibandSchedule` /
   `BandSplit` record each band's state range and feed-through. With
   `multiband.enabled`, `StateSpaceEngine::processBlock()` computes each
   band's contribution from its own states, tracks that band's own
   envelope, looks up its curve and sums the scaled contributions. `A` and
   `B` never change.
3. **Training reuses the single-band fit.** `GainCurveTrainer::fitMultiband()`
   runs the existing `fit()` once per band. One subtlety: the target mix
   already contains each band's signal filtered once, so recovering a band
   from the target filters it a second time. The input is therefore also
   filtered twice, so both signals `fit()` compares carry the same phase
   shift -- the fit compares samples pointwise, and a phase difference
   would otherwise look like a gain error. `fit_multiband_compressor` is
   the command-line front end.

The **Multiband Compressor** plugin (`plugin_mb_compressor/`, `gui/mb/`)
puts an editor on this: `MbCompressorModel` turns crossover frequencies
and per-band threshold/ratio/up ratio/attack/release/makeup into the
crossover bank and its schedules.

## Dynamic EQ

A dynamic EQ band is a bell whose gain follows the level in its own
region. Two tempting approaches don't work:

- Scaling a band-pass tap by a level-dependent gain (as the multiband
  compressor does with crossover slices) is wrong at rest: an EQ band
  overlaps its neighbours, so even at "0 dB" it would add a second copy of
  the band-passed signal.
- Varying an RBJ peaking filter's gain every sample moves its poles (the
  gain appears in the denominator), which the stability guard can't
  re-check per sample.

**Regalia-Mitra peaking.** For a fixed second-order all-pass `A(z)`
(`mappers::designAllpass()`, set by centre frequency and Q only),

```
H(z) = 0.5*(1+G) + 0.5*(1-G)*A(z)
```

is exactly a peaking filter with linear centre gain `G` and unity at DC
and Nyquist (`A = -1` at the centre, `+1` far from it). `G` enters only in
the output combination, so it can change every sample without touching a
pole. It is a constant-bandwidth shape, slightly different from RBJ's
constant-Q peaking. See `plugin/dsp/DynamicEqModel.hpp` for the
derivation.

**A dedicated engine.** Time-varying bands in series have no fixed
combined state-space realization, so `DynamicEqEngine.hpp` runs its own
per-sample recipe: per band, the fixed all-pass and a separate detector
band-pass on the same signal; the detector's envelope is resolved to a
gain by `resolveDynamicGainDb()` (Ratio above the threshold, Up Ratio
below it, limited to Range); the formula above combines them. Changing
the number of active bands fades through a short silence, as in
`StateSpaceEngine`. The live gain of each band reaches the editor as an
output Parameter (`kParamCurrentGainBase + band`).

The export freezes every band at its resting gain as an ordinary RBJ
cascade -- a moving filter has no fixed matrix.

### Dynamic EQ 2

`DynamicEqModel2` / `DynamicEqEngine2` extend this:

- **Low / High Shelf** use a *first-order* all-pass in the same
  construction; a first-order shelf's slope is fixed, so Q is unused.
- **Band Shelf** combines two first-order shelves and a flat gain:

  ```
  s1  = 0.5*(1+G)*x  + 0.5*(1-G)*A_lo(x)
  s2  = 0.5*(1+G)*s1 - 0.5*(1-G)*A_hi(s1)
  out = s2 / G
  ```

  The corner frequencies are compensated with the resting gain (low
  corner x sqrt(G), high corner / sqrt(G)) so that the corners are the
  half-gain points; outside the corners the response returns to 0 dB.
- **Harmonic linking**: linked bands move together in frequency (each keeps
  its ratio to the group's virtual base frequency) and share threshold,
  ratios and range (`propagateHarmonicDynamics()`); auditioning one sums
  the whole group.
- **Crossfeed** bands (L→R, R→L) filter one channel and add the result into
  the other, with one overall level per direction.

## Where things live when adding a mapper

- `plugin/dsp/CoefficientMapper.hpp` -- design formulas and realization
  functions.
- `StateSpaceEngine.hpp`, `StabilityGuard.*`, `TransferFunction.*` and
  `Serialization.*` should not need changes -- they only ever see a
  `StateSpaceMatrices`.
- A new editor is a new widget set under `gui/` plus its own plugin
  folder, following `plugin_eq/` + `gui/eq/`.
- `plugin/dsp/GainCurveTrainer.*` and `tools/trainer/` -- fitting from
  audio. Matrix Filter's gain-curve panel (`gui/NonlinearPanelWidget.*`)
  edits a trained curve like a hand-drawn one.

## Possible next steps

- An exact eigenvalue solver (Hessenberg reduction + implicit double-shift
  QR) to show real pole/zero locations instead of a spectral-radius
  estimate.
- Click-and-drag on Matrix Filter's matrix cells and gain-curve points
  (currently click to type and scroll to nudge), and typed values for
  individual curve points.
- Training from audio inside the plugin (today `fit_gain_curve` /
  `fit_multiband_compressor` on the command line).
- Mono versions of the Multiband Compressor (following the mono EQ's
  `MonoStateSpaceTypes` / `MonoStateSpaceEngine` / `MonoStabilityGuard`)
  and of the Dynamic EQs (which would need their own single-channel
  engine).
- Host automation of the musical parameters of the mappers.
