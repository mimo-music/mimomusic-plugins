# Extending: building EQ / other processors on top

The whole point of the generic state-space core is that new processing
types are added as **coefficient mappers** -- plain code that turns
musically-meaningful parameters into `A`/`B`/`C`/`D` entries -- without
touching the real-time engine, the stability guard, the serialization
format, or the matrix-editing GUI. See `plugin/dsp/CoefficientMapper.hpp`
for the convention and a worked example.

This only works for **linear, time-invariant (LTI)** processing -- see
"Why compression needs state-dependent coefficients" below for the hard
boundary on what a *fixed*-coefficient state-space model can and can't
represent, at any order, and how this plugin actually does support dynamics
without crossing it.

## Building an LTI processor -- EQ, crossovers, resonators, phase shaping

1. Design your filter in whatever representation is natural (a biquad's
   `b0,b1,b2,a1,a2`, a set of pole/zero pairs, a ladder filter's
   coefficients, ...).
2. Write a function that *realizes* it as a state-space block. The included
   `mappers::realizeBiquadInto()` is the template: it's the standard
   controllable-canonical-form realization of one second-order section,
   `mappers::makeStereoBiquad()` shows how to pack two independent (one per
   channel) realizations into a block-diagonal order-4 model.
3. For a multi-band EQ, realize each band as its own diagonal block (like
   the biquad example does for L/R) and sum contributions in `C`/`D`, or
   simply cascade bands by concatenating their blocks along the diagonal --
   either way, `StabilityGuard::enforce()` on the combined result is your
   safety net regardless of how you assembled it.
4. Always run the result through `StabilityGuard::enforce()` before handing
   it to `StateSpaceEngine::setModel()`. A validly-designed filter (e.g. an
   RBJ biquad with reasonable Q) is already stable by construction, so this
   is normally a no-op -- but it's free insurance against extreme/edited
   parameter values (a user typing `Q = 500` into an EQ GUI, say).

This isn't hypothetical: `plugin/dsp/EqModel.hpp` and the sibling
`statespace_eq` plugin (`plugin_eq/`, `gui/eq/`) are exactly this pattern,
fully worked through -- a per-band parameter struct
(type/frequency/gain/Q), `eq::assembleStateSpace()` doing steps 1-4 above
via `mappers::makeStereoBiquadChain()`, and a GUI (`EqGraphWidget`,
`BandListWidget`) that sits *above* the musical parameters rather than a
raw matrix grid -- the base plugin's matrix grids remain available as the
"see and hand-edit every actual coefficient" view, reached via the EQ
plugin's own "Export to State-Space..." button rather than a mode switch
inside one plugin (see `docs/ARCHITECTURE.md`'s "EqModel / EQ plugin"
section for why two plugins, not one with a mode toggle). If you're
building your own mapper, reading `EqModel.cpp` alongside
`CoefficientMapper.hpp` is probably more useful than this file's prose.

## Why compression needs state-dependent coefficients

A natural question is whether this plugin can grow a compressor. The
honest starting point is that a fixed-coefficient state-space model --
`x[n+1]=Ax[n]+Bu[n]`, `y[n]=Cx[n]+Du[n]` with **constant** `A/B/C/D` --
**cannot represent dynamic range compression, at any order.** This isn't a
missing feature or a limitation of this implementation; it's a direct
consequence of what "LTI" (linear, time-invariant) means:

> **Superposition.** For *any* state-space system with constant matrices,
> if input `u` produces output `y`, then input `k*u` produces output
> exactly `k*y` -- for every `k`, always, regardless of the order or what
> `A/B/C/D` contain.

A compressor's entire job is to violate that: a signal played twice as
loud should come out *less* than twice as loud (above threshold). A system
that always scales its output in exact proportion to its input can never
do that -- adding more states (higher order) only ever buys you more
elaborate *frequency-dependent* behavior (sharper filters, more bands, more
complex phase response), never *level-dependent* behavior with *fixed*
coefficients. Two earlier drafts of this plugin explored escape hatches
that weren't actually state-space models at all (a bolted-on post-
processing effect, then a signal-envelope-driven gain applied outside the
matrix recursion) and both were correctly rejected.

The mathematically legitimate way to get level-dependent behavior *out of*
a state-space model, without lying about what "state-space" means, is a
standard control-theory construction: **state-dependent coefficients**
(SDC), also called a linear-parameter-varying (LPV) system. `A` and `B` --
the part of the model that determines the *recursion's* stability -- stay
completely fixed, exactly as `StabilityGuard::enforce()` validated them.
Only `C` and `D` are allowed to vary, recomputed every sample as
`C(p[n])`, `D(p[n])` from an observable *scheduling variable* `p[n]`
(here: a smoothed envelope of the input level). Since `y[n] = C(p[n]) x[n]
+ D(p[n]) u[n]` is still evaluated exactly as before at each individual
sample, and the only thing driving the nonlinearity is a measurement of
the signal itself (not an arbitrary external time-varying command), this
is a real, well-understood member of the LPV family (gain-scheduled
control, SDRE) rather than an ad hoc bypass of the model.

This plugin implements the simplest useful case of that: `C(p)` and `D(p)`
are never literally recomputed as matrices -- they're `gain(p) * C0` /
`gain(p) * D0` for a fixed base `C0/D0`, which is arithmetically identical
to multiplying the already-computed `y[n]` by a scalar `gain(p[n])`. See
`plugin/dsp/StateSpaceTypes.hpp`'s `NonlinearSchedule`/`GainCurve` and the
"NonlinearSchedule" section of `docs/ARCHITECTURE.md` for exactly how
`p[n]` is computed and how `gain(p)` is represented and learned from data.

What's fully representable *without* the nonlinear stage, at arbitrarily
high order: any number of cascaded/combined linear filters, resonators,
all-pass sections, crossovers, and cross-channel (mid/side-style) coupling
-- see "Building an LTI processor" above and `mappers::makeStereoBiquadChain()`
for a worked multi-band example. If a use case looks like it needs
"dynamics", check first whether it's actually a fixed EQ curve, a
frequency-dependent (but level-independent) gain shape, or similar --
those are buildable as a plain LTI mapper and don't need the nonlinear
stage at all; genuine level-dependent gain reduction is what the
`NonlinearSchedule` mechanism is for.

## Multiband compression

Everything above (`NonlinearSchedule`) is *one* envelope tracking the
*whole* (already-summed) output. A real multiband compressor is the same
mechanism, N times, each scoped to its own frequency band -- and it's
built from pieces that already existed for other reasons:

1. **The crossover is a plain LTI mapper**, exactly like an EQ.
   `mappers::makeStereoCrossoverBank()` (`CoefficientMapper.hpp`) splits
   the signal into up to `kMaxBands` (6) bands via a tree of Linkwitz-
   Riley 2-way splits (`mappers::designCrossoverBandStages()`) -- the
   standard reason LR crossovers exist rather than plain Butterworth
   pairs: they sum to *exactly* unity gain at every frequency, so a
   multiband compressor with every band's gain at 0dB is indistinguishable
   from a straight passthrough, not a subtly-colored one. This assembles
   into an ordinary `StateSpaceMatrices` (a *parallel*, not series,
   connection of independent per-band chains -- block-diagonal `A`, `C`
   horizontally concatenated, `D` summed), so `StabilityGuard::enforce()`
   is exactly as much a no-op-in-practice safety net here as for any other
   mapper.
2. **Each band gets its own `NonlinearSchedule`.**
   `StateSpaceTypes.hpp`'s `MultibandSchedule`/`BandSplit` record which
   contiguous state range (and feed-through `D`) belongs to each band,
   plus that band's own schedule. `StateSpaceEngine::processBlock()`, when
   a model's `multiband.enabled` is set, computes each band's own pre-gain
   contribution from its own state slice, tracks *that band's own filtered
   signal's* envelope (not the broadband input -- the one real behavioral
   difference from the single-band stage), looks up that band's own curve,
   and sums the gain-adjusted contributions into the final output. `A`/`B`
   are, as always, never touched.
3. **Training generalizes by orchestration, not new math.**
   `GainCurveTrainer::fitMultiband()` filters the (input, target) audio
   through each band's own standalone chain and calls the *existing*,
   unmodified `fit()` once per band with an identity base model -- the
   least-squares fitting logic is 100% reused; the only new code is
   "which signal does each band see." `tools/trainer/
   fit_multiband_compressor.cpp` is the CLI front-end:
   `fit_multiband_compressor "<crossover_hz_csv>" input.wav target.wav
   out.ssjson` -- e.g. `"150,2000"` for a 3-band split. The output's
   A/B/C/D *is* the crossover bank itself (with every band transparent, it
   really is just a crossover filter), plus the fitted `multiband` block.

### A real bug found building the trainer: phase mismatch, not amplitude

`fitMultiband()`'s first version filtered the (opaque, already-summed)
target through band k's own filter to recover that band's signal, then
compared it *pointwise* against the input filtered through the same
filter *once*. This looked reasonable but was wrong, and not for an
amplitude reason (a band's own passband gain deep from the crossover is
within a thousandth of a dB of unity, even applied twice -- verified
directly, not just assumed): any *sane* multiband compressor's true target
audio is `sum_k(gain_k(t) * filter_k(rawInput))`, so band k's own true
contribution has already gone through `filter_k` *once* before this
function ever sees the summed target. Recovering it means filtering
*again* -- so band k's true contribution ends up phase-shifted by *two*
applications of the filter's own phase response, while the once-filtered
input side only carries one. `GainCurveTrainer::fit()`'s per-sample
instantaneous-ratio technique (`target[i]/base[i]`) is meaningless
comparing two differently-phase-shifted copies of the same waveform, even
when their true *envelope-level* relationship (the only thing actually
being learned) is identical -- two sine waves of equal amplitude but
different phase divide pointwise into noise, not a constant. This was
large in practice (wrong attack/release grid points, several dB of curve
error) and completely invisible to the single-band trainer's own test
(`dsp_regression` test `[9]`), since that path never has a filtering
asymmetry between its two traces -- `fit()` always compares a signal
against itself run through an identity model. Fixed by filtering the
input through the *same* filter *twice* too (see `GainCurveTrainer.cpp`'s
own comment on this), so both traces `fit()` compares carry the same
two-application phase shift, leaving only the actual gain as a difference.
`dsp_regression` tests `[15]`-`[17]` cover the crossover's flat-sum
property, the engine's per-band gain mechanism against closed-form ground
truth, and this trainer round-trip, respectively.

### Multiband compressor GUI (`statespace_mb_compressor`)

Built in a follow-up session on top of the DSP core above, matching the
order the single-band compressor was built in (core + CLI trainer first,
GUI later): a fourth plugin, `plugin_mb_compressor/` + `gui/mb/`, following
the EQ plugins' own established pattern almost exactly -- see
`docs/ARCHITECTURE.md`'s "Multiband compressor plugin" section for the
full writeup (the coefficient-mapper layer `MbCompressorModel`, the
`MultibandGraphWidget` graph, the `ParamStripWidget` numeric fields ported
from the sibling `../compressor` project, and Export/Import mirroring
`UIEq`'s own).

What that session's own "Known limitations" list still leaves open: no
live gain-reduction meter or per-band transfer-curve display (needs a new
metering path out of `StateSpaceEngine`, deliberately not touched yet --
see `docs/ARCHITECTURE.md` for why), no per-band-row numeric list the way
the EQ's `BandListWidget` shows every band at once (one field strip
re-targeted at whichever band is selected instead), no in-GUI "train from
audio" workflow (`fit_multiband_compressor` stays CLI-only, same gap the
single-band compressor has), and no mono variant yet (would follow the
mono EQ's own `MonoStateSpaceTypes`/`MonoStateSpaceEngine`/
`MonoStabilityGuard` pattern, not a new architecture).

## Dynamic EQ

The idea, as in FabFilter Pro-Q's dynamic bands: EQ bands that can
dynamically limit their own peaks -- a multiband compressor on top of the
filters. Multiband compression above already tracks a level
and moves a gain per band -- the missing piece is that its bands are
non-overlapping *crossover slices*, and an EQ-style dynamic band is a
freely-placed, overlapping *bell* instead. That difference turns out to
matter for more than just the GUI:

**Why `NonlinearSchedule`/`MultibandSchedule` can't just be reused
unchanged.** Both existing mechanisms rely on the same trick: keep a
band's own *filter* completely fixed (`A`/`B` never move) and only ever
scale that filter's *entire, already-computed* output by a level-dependent
scalar. That's exactly right for a crossover slice, because crossover
bands are non-overlapping and *sum* to the full spectrum -- "0dB, do
nothing" on one of them genuinely means "this slice passes through
untouched". A bell/peaking EQ band sits in the *middle* of the spectrum,
overlapping its neighbors on both sides; scaling an isolated bandpass tap
by "0dB, do nothing" would still add a whole extra copy of that bandpassed
signal on top of everything else, which is audibly wrong at rest. And the
other obvious option -- literally vary a standard RBJ peaking filter's own
`gainDb` input every sample -- doesn't work either: that formula's
denominator (`a1`, `a2`) depends on gain (the `alpha/A` split), so a
smoothly-moving gain means smoothly-moving *poles*, which is precisely the
"state-dependent coefficients can only ever be `C(p)`/`D(p)`, never `A`/
`B`" line this project draws everywhere else, for good reason (nothing
cheaply re-verifies a moving `A`'s stability every sample).

**The fix: Regalia-Mitra fixed-allpass peaking.** A classical (Regalia &
Mitra, 1987; also in Zolzer's *DAFX*) construction sidesteps this
entirely: for a fixed 2nd-order allpass `A(z)` (the existing RBJ cookbook
allpass, `mappers::designAllpass()`, parameterized by center frequency and
Q/bandwidth *only*, never gain),

```
H(z) = 0.5*(1+G) + 0.5*(1-G)*A(z)
```

is *exactly* a peaking filter with linear center-frequency gain `G` and
unity gain at DC/Nyquist -- `A(z) = -1` at its own center frequency gives
`H = G` there, `A(z) = +1` far from it gives `H = 1`. `G` enters purely
*affinely*: `A(z)`'s own state recursion never changes, only the linear
combination of its already-computed output with the dry signal -- so `G`
can be swept every sample from a level detector exactly as safely as
`NonlinearSchedule`'s existing `C(p)=gain*C0` mechanism, without ever
touching a pole. See `plugin/dsp/DynamicEqModel.hpp`'s file comment for
the full derivation and the one real caveat: this is a genuine, different
2nd-order EQ shape from RBJ's `designPeaking()` -- *constant bandwidth*
(fixed Hz width regardless of gain) rather than *constant Q* (RBJ's
gain-dependent `alpha/A` narrows the peak slightly as gain increases).

**The engine is dedicated, not a mapper.** Because each band's gain moves
continuously and bands are cascaded in *series* (like an ordinary EQ, not
summed in parallel like a crossover), there is no single fixed
`StateSpaceMatrices` this could ever assemble into -- cascading several
*time-varying* stages has no fixed combined state-space realization, only
a fixed per-sample recipe for combining them. `plugin/dsp/
DynamicEqEngine.hpp` is that recipe as a standalone real-time engine (like
`MonoStateSpaceEngine.hpp` is its own dedicated engine, not a mapper feeding
the generic one): per band, per sample, run the band's own fixed allpass
*and* a separate genuine (magnitude-shaped) detector bandpass on the same
incoming signal, resolve the detector's envelope to a gain via
`dyneq::resolveDynamicGainDb()` (flat below threshold, sloping by
`1-1/ratio` above it, clamped to a `rangeDb` ceiling -- the "how far can
this band's gain move" knob a per-band peak limiter needs but a broadband
compressor's curve doesn't), and combine via the formula above. A band-
*count* change (add/remove/enable/solo) gets the exact same duck-to-
silence-then-swap technique `StateSpaceEngine` uses for an EQ order change,
for the identical reason (no meaningful "in between" two different band
counts could interpolate through).

`dsp_regression` tests `[19]`-`[21]` cover: the resting-shape closed form
hits the resolved gain exactly at center frequency and flat away from it;
the real engine, run end to end on a synthetic tone, settles to the
resting gain when quiet and to the closed-form-predicted reduced gain when
loud, with `getCurrentGainDb()`'s live readback agreeing; and the
*static* export path (an ordinary RBJ `designPeaking()` cascade, freezing
every band at its resting gain -- deliberately not the Regalia-Mitra shape,
since a fixed matrix can't represent a moving one at all) matches
`TransferFunction` the same way `EqModel`'s own export does.

### Dynamic EQ GUI (`statespace_dyn_eq`)

A fifth plugin, `plugin_dyn_eq/` + `gui/dyneq/`, reusing `EqGraphWidget`'s
own node-drag interaction (drag a node to move frequency+gain, scroll to
adjust Q, double-click to remove, click empty space to add) since a
dynamic EQ band is still fundamentally a bell node on a frequency/gain
graph -- with no band-type context menu (every band is the same Regalia-
Mitra bell, the same simplification `MultibandGraphWidget` already made
for its own, differently-shaped bands). On top of that: a shaded "dynamic
range" region per band (the area between that band's response curve at
its resting gain and at `gainDb - rangeDb`, both via the same closed-form
evaluator with a different resolved gain plugged in) and a second,
brighter combined curve using each band's *live* `getCurrentGainDb()`
instead of its resting one -- a curve that visibly moves with the
program material. The live gain has to reach the UI as a DPF output `Parameter`
(`kParamCurrentGainBase`, one per possible band slot) since the DSP and UI
can be in separate processes -- the same reason spectrum data already
crosses that boundary as parameters rather than a direct getter call.

All eight of a band's numeric fields (Freq/Gain/Q/Threshold/Ratio/Range/
Attack/Release) live in one `ParamStripWidget` retargeted at whichever
band is selected, the same "select then edit" simplification the
multiband compressor's own GUI argues for (there, 5 fields against an
EQ's 3; here, 8) -- Freq/Gain/Q are also graph-draggable/scrollable, the
same intentional redundancy `EqGraphWidget`/`BandListWidget` already have.

Possible next steps: a live gain-reduction meter (same deferral reasoning
as the multiband compressor's own -- see below), a mono variant following
the established `MonoStateSpaceTypes`/`MonoStateSpaceEngine` pattern (this
plugin doesn't use that pattern at all today, since its engine isn't a
mapper -- a mono version would need its own dedicated single-channel
`DynamicEqEngine`, not a reuse of the existing mono machinery), and a
band-type choice beyond Bell (a Regalia-Mitra-style fixed-shelf
construction exists in the literature too, just not implemented here).

## Where things live if you're adding a mapper

- `plugin/dsp/CoefficientMapper.hpp` -- add your design formulas and
  realization functions here.
- Nothing in `plugin/dsp/StateSpaceEngine.hpp`, `StabilityGuard.*`,
  `TransferFunction.*` or `Serialization.*` should need to change -- they
  only ever see a `StateSpaceMatrices`.
- A new GUI "mode" (e.g. an EQ band editor) would be a new widget file in
  `gui/`, wired into `UIStateSpace` alongside (or, eventually, replacing as
  the default view of) the raw matrix grids. The matrix grids themselves
  stay useful as the underlying "show me what's actually happening" view
  for any mode.
- `plugin/dsp/GainCurveTrainer.*` and `tools/trainer/fit_gain_curve.cpp` --
  add your data-driven fitting logic here if you extend what the nonlinear
  stage can learn (see `docs/ARCHITECTURE.md`'s "NonlinearSchedule" section
  for the current scope: a single envelope-driven gain curve, fit by
  variable-projection least squares). Training itself is still CLI/
  JSON-file only (no "learn from audio" button in the plugin) -- see
  `gui/NonlinearPanelWidget.*` for the in-plugin hand-editing GUI (enable
  toggle, attack/release fields, a threshold/ratio/makeup "Apply Curve"
  shortcut, and a click/drag/scroll curve graph), which works equally well
  on a hand-drawn curve or one `fit_gain_curve` produced.

## Possible next steps beyond mappers

- An exact eigenvalue solver (real-Schur via Hessenberg reduction + implicit
  double-shift QR) would let the GUI show actual pole/zero locations, not
  just a spectral-radius estimate -- see the "Known limitations" note in
  `docs/ARCHITECTURE.md`.
- Click-drag value adjustment on matrix cells (this draft only supports
  click-to-type and scroll-to-nudge; see `docs/ARCHITECTURE.md`). No
  longer a purely open question: `gui/ParamStripWidget` (ported into this
  project for the multiband compressor plugin, see "Multiband compressor
  GUI" below) already implements exactly this click-and-drag-vertical-
  fader interaction and confirms DGL hands a widget every motion event for
  the rest of a drag it started regardless of where the pointer strays,
  as long as it stays inside the window -- the concrete uncertainty this
  bullet used to describe. Bringing the same interaction to the matrix
  grids/gain-curve knots is now a known-safe, not-yet-done port, not an
  open feasibility question.
- A modulation/automation layer so mapper parameters (not just raw matrix
  cells) are host-automatable DPF `Parameter`s.
- A "learn from audio" workflow inside the plugin itself (dropping in an
  input/target WAV pair and running `GainCurveTrainer` from the GUI)
  rather than needing the external `fit_gain_curve` CLI tool -- the curve
  editor and threshold/ratio convenience controls already exist
  (`gui/NonlinearPanelWidget.*`), just not a way to trigger the fit itself
  from inside the plugin.
- Per-knot numeric text entry on the gain curve (currently click/drag/
  scroll only, deliberately, for the same click-drag-across-widget-bounds
  reason the matrix grids don't support it either -- see "Known
  limitations" in `docs/ARCHITECTURE.md`) for surgically precise curve
  values without needing the JSON file directly.
- A live gain-reduction meter / per-band transfer-curve display for the
  multiband compressor plugin (see "Multiband compressor GUI" below and
  `docs/ARCHITECTURE.md`'s own "Known limitations" for that plugin) --
  needs a new metering path out of `StateSpaceEngine`, deliberately not
  added yet since that file is this project's most carefully-verified
  real-time code.
- A mono variant of the multiband compressor plugin, following the mono
  EQ's own established `MonoStateSpaceTypes`/`MonoStateSpaceEngine`/
  `MonoStabilityGuard` pattern -- `MbCompressorModel` and
  `mappers::makeStereoCrossoverBank()` are already channel-count-agnostic
  in the same way `EqModel`'s own design/assembly split is, so this is a
  known pattern to repeat, not a new architecture.
- A per-band-row numeric list for the multiband compressor plugin, the
  way the EQ's `BandListWidget` shows every band's fields at once, instead
  of one field strip re-targeted at whichever band is currently selected.
- A live gain-reduction meter for the dynamic EQ plugin too (see its own
  "Possible next steps" above), a mono variant (its own dedicated
  single-channel engine, not a reuse of `MonoStateSpaceEngine`, since
  `DynamicEqEngine` isn't a mapper feeding the generic core to begin
  with), and band types beyond Bell (a Regalia-Mitra-style fixed-shelf
  construction exists in the literature, just not implemented here).
