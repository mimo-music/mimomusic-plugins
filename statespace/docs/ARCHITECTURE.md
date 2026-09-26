# Architecture

## The model

Everything the plugin does revolves around one struct,
`ssfilter::StateSpaceMatrices` (`plugin/dsp/StateSpaceTypes.hpp`): an
`order` (1..128) plus fixed-size `A` (order x order), `B` (order x 2), `C`
(2 x order) and `D` (2 x 2) matrices, stored at their maximum size
(`kMaxOrder = 128`) regardless of the active order so nothing ever needs
to reallocate when the order changes. It also carries one optional field,
`nonlinear` (a `NonlinearSchedule`), for the state-dependent gain stage
described below -- everything else in the plugin treats a
`StateSpaceMatrices` as pure LTI and simply passes that field through.
This struct is the single source of truth: the GUI edits a copy of it, the
audio engine runs a copy of it, and a `.ssjson` preset file is just its
JSON serialization.

## Real-time engine (`StateSpaceEngine`)

`run()` on the DSP side is exactly the two matrix equations, evaluated once
per sample:

```
y[n] = C x[n] + D u[n]
x[n+1] = A x[n] + B u[n]
```

Coefficient updates come from the UI/message thread (`setModel()`), never
the audio thread. The hand-off is a **wait-free double buffer**: two
preallocated `StateSpaceMatrices` slots and an atomic index. The writer
fills the *other* slot then flips the index (release); the audio thread
reads the index once per block (acquire) and uses that slot throughout the
block. No lock, no allocation, no unbounded work on the audio thread ever.

**A/B/C/D are smoothed toward each new model, not switched to instantly,
whenever its order matches the one currently running** -- i.e. every
ordinary coefficient edit (dragging an EQ band, nudging a matrix cell),
which never changes state-vector size. This was a real bug, not a
preemptive nicety: dragging an EQ band fast enough produced audible
clicks/pops -- like plugging or unplugging a speaker cable -- because
`StabilityGuard` only guarantees a target model
is stable *in isolation*; it says nothing about what happens applying a
suddenly-different `C`/`D` to a state `x[n]` the *previous* coefficients
had been accumulating, or a suddenly-different `A` to the ongoing
recursion, between one sample and the next. A short exponential ramp
(`kCoeffSmoothTimeConstantSec`, 5ms -- about 63% closed per time constant,
fully converged well under 50ms) removes that discontinuity while staying
fast enough to feel immediate -- react fast, but without steps.
Implementation: a monotonic version counter (not just
the 2-valued slot index, which two rapid `setModel()` calls landing
between the same two `processBlock()` calls could alias) lets the audio
thread detect "a new target arrived" reliably; if the new target's order
differs from what's currently running, there's no meaningful "in between"
between two different-sized state vectors, so that case still switches
instantly -- but that's an order-*changing* edit (adding/removing an EQ
band, or toggling a band's enabled/solo flag, which changes how many
bands are *active* in the assembled cascade exactly as much as adding/
removing one outright), a rare, deliberate action, never the "drag an
existing point fast" case. Only
`A`/`B`/`C`/`D` are smoothed; the nonlinear stage's own parameters
(attack/release/curve) switch instantly, since its output is already a
smoothed envelope-driven scalar gain, not a recursive state, so there's
no equivalent discontinuity risk there. `dsp_regression` test `[12]` pins
down the exact expected one-pole trajectory (a pure-gain model pair,
closed-form checked to within 1e-6) as a permanent regression guard.

Order-changing edits *were* also reported clicking, once coefficient
smoothing eliminated it for ordinary dragging. Since two differently-
sized state vectors have no meaningful "in between" to interpolate
through, the fix here is a **duck** instead: the output fades to silence
(`DuckState::FadingOut`, ~8ms), the switch is made -- still instantly,
including a full state reset, since a different-order model can mean
entirely different coefficients even at state indices that existed
before (removing a *middle* EQ band reindexes every band after it in the
series cascade, not just truncating the end) -- while nothing is
audible, then the output fades back in (`FadingIn`, ~8ms). This is
deliberately not a parallel dual-state crossfade (the other standard
technique for this situation): a crossfade would need the *new* filter
to already be well-settled by the time it's audible, which isn't
guaranteed for a narrow/high-Q filter within a fade window short enough
to feel immediate, whereas a duck's correctness doesn't depend on the
two filter structures having anything in common. A monotonic version
counter (see `setModel()`'s comment) also drives this detection, so it
shares the same reliable "a new target arrived" signal coefficient
smoothing uses. `dsp_regression` test `[13]` confirms an order-changing
switch between two *same-gain* models (so an un-ducked instant switch
would look identical to a duck at every sample except during the actual
transition) visibly dips to silence and recovers, proving the duck runs
rather than an instant snap.

Two safety nets exist *in addition to* the stability guard below, because no
amount of upstream validation can prove a recursive filter stays finite
forever under arbitrary sustained input and accumulated floating-point
error:

- **NaN/Inf detection** per sample: if the output (equivalently, the state)
  ever goes non-finite, the state vector is zeroed and silence is
  substituted for that sample, with a one-shot fault flag exposed to the
  host/GUI (`kParamFaultFlag`, an output parameter).
- **A hard output ceiling** (+-64, i.e. +36 dBFS) as an absolute last
  resort -- this is not a mastering limiter and should never engage for any
  sane configuration.

## Stability guard (`StabilityGuard`)

A discrete-time LTI system `x[n+1] = A x[n] + ...` is stable iff every
eigenvalue of `A` has magnitude < 1 (spectral radius `rho(A) < 1`). Rather
than implement a full eigendecomposition (a real-Schur/implicit-QR solver is
a lot of delicate code to get right, and only the *maximum* eigenvalue
magnitude is actually needed here), the guard combines two things:

1. **An exact, cheap upper bound**: `rho(A) <= min(||A||_1, ||A||_inf)`
   (max abs column sum, max abs row sum) for any square matrix. If this
   bound already clears the target radius (0.99995 by default -- see
   below for why it's this close to 1), `A` is *provably* stable and
   nothing is touched. This bound is typically 2-3x the true spectral
   radius for a resonant filter's controllable-canonical-form matrix, so
   most nontrivial filters fall through to step 2 rather than taking this
   fast path -- that's expected, not a sign of near-instability.
2. Otherwise, a **near-exact estimate** of `rho(A)` via repeated squaring:
   `rho(A) = lim_{N->inf} ||A^N||^(1/N)` for any submultiplicative matrix
   norm, unconditionally (no dominant-eigenvalue-gap assumption needed,
   unlike a single-vector power iteration -- see the "why" note below).
   40 squarings reach an effective N of 2^40 in ~40 matrix multiplications.
   This estimate, padded with a small *additive* margin, drives a single
   **uniform scale factor** `A' = s*A`. Because scaling is linear,
   `rho(s*A) = s*rho(A)` *exactly* -- the only remaining source of error is
   in the estimate itself, and convergence is checked against that same
   estimate each round, not the loose bound from step 1. A final fallback
   (scaling directly by the exact norm bound, which is mathematically
   guaranteed to work) exists for the exceedingly rare case that doesn't
   converge, at the cost of being far more conservative than necessary.

Uniform scaling was chosen (over, say, clamping individual coefficients)
because it preserves the *shape* of `A` -- relative pole angles/resonant
frequencies are unchanged, everything is just pulled radially toward the
origin. A too-hot filter gets tamed, not distorted.

`B`, `C`, `D` don't affect stability (only `A`'s recursion can diverge) but
non-finite entries anywhere are sanitized to zero defensively.

This runs whenever a model is about to be handed to the engine (every GUI
edit, every preset load, every host state restore) -- never in the audio
callback.

**Why repeated squaring, not power iteration, and why the ceiling is
0.99995 not 0.999**: an earlier version used single-vector power iteration
for step 2, checked convergence against the loose bound from step 1, and
capped the ceiling at 0.999. All three turned out to be real bugs, found
via a very concrete symptom -- a 50 Hz/Q=25 notch preset produced no
audible or visible notch at all. Power iteration is measurably biased for
a complex-conjugate dominant pole pair of *exactly* equal magnitude, which
is the routine case for any resonant biquad, not an edge case; checking
against the loose bound meant the refinement loop could never actually
converge for that matrix shape; and 0.999 is simply too tight a ceiling
for a legitimately narrow filter, whose true pole radius is *supposed* to
sit at 0.999-0.99995 by design (double precision makes that an entirely
safe margin; `StateSpaceEngine`'s own NaN/Inf net is a second, independent
backstop regardless). Together these three silently collapsed the notch's
coefficients to ~30% of their designed value. See
`plugin/dsp/StabilityGuard.cpp`'s comments and `docs/TESTING.md` for the
full account, and `dsp_regression`'s test `[7]` / `audio_testbench`'s
`notch_50hz_hum` case for the regression tests that now pin this down.

A full real-Schur/eigenvalue decomposition would additionally let the GUI
show actual pole locations (a pole-zero plot) instead of just a scalar
radius estimate -- a natural next step, see `docs/EXTENDING.md`.

## Transfer function / Bode plot (`TransferFunction`)

Independent of stability: `H(z) = D + C (zI - A)^-1 B`, evaluated on the
unit circle (`z = e^{j*omega}`) via dense complex Gaussian elimination with
partial pivoting. This gives the full 2x2 complex response matrix at any
frequency, including cross-channel terms. The GUI sweeps ~220 log-spaced
points from 20 Hz to Nyquist (capped at 20 kHz) and plots magnitude (all
four channel pairs) and phase (the two direct paths). This is O(order^3)
per point and is **only ever run from the UI/message thread**, debounced
(a few `uiIdle()` ticks of hysteresis) so a burst of edits doesn't trigger a
recompute per keystroke.

## NonlinearSchedule (state-dependent gain stage)

`A`/`B`/`C`/`D` alone can only ever be LTI (see `docs/EXTENDING.md`'s "Why
compression needs state-dependent coefficients" for why that rules out
compression at any order with *fixed* coefficients). `StateSpaceMatrices`
carries one optional extra field, `nonlinear` (`StateSpaceTypes.hpp`'s
`NonlinearSchedule`), which lets `C`/`D` vary *per sample* as `gain(p[n]) *
C0` / `gain(p[n]) * D0` for a scheduling variable `p[n]` derived from the
input -- arithmetically identical to, and implemented as, multiplying the
already-computed `y[n]` by a scalar gain. `A`/`B` (and therefore the
recursion's stability, already locked in by `StabilityGuard`) are never
touched by this stage.

Per sample, in `StateSpaceEngine::processBlock()`, after `y[n]` is computed
from the ordinary state-space equations:

1. **Envelope follower**: `levelLin = max(|u0|, |u1|)`; a one-pole
   attack/release smoother tracks it: `env = coeff*env + (1-coeff)*level`,
   with `coeff` chosen per-sample from `nlAttackCoeff` (rising) or
   `nlReleaseCoeff` (falling), both precomputed once per block from
   `attackMs`/`releaseMs` and the sample rate. **This smoothing happens in
   the *linear* amplitude domain, not dB** -- converting to dB only
   afterward, for the curve lookup. This is deliberate and was the subject
   of a real bug (see below): a signal's instantaneous `|sample|` touches
   0 (dB: -infinity) at every zero crossing, twice per cycle, so smoothing
   directly in the dB domain makes the attack/release behavior depend on
   the signal's *frequency*, which no real compressor does.
2. **Curve lookup**: `envDb = 20*log10(env)` is looked up in `GainCurve`, a
   25-knot piecewise-linear table (`kMinDb=-72` to `kMaxDb=24`, 4 dB
   spacing) mapping envelope level to gain, both in dB, with flat
   extrapolation beyond the table's ends.
3. **Apply**: `y0 *= gainLin; y1 *= gainLin;` where `gainLin =
   10^(gainDb/20)`.

A hand-set curve (threshold/ratio/makeup baked into the 25 knot values) is
exactly a classic feed-forward compressor; `dsp_regression` test `[8]`
checks one against the closed-form settled-gain formula
`10^((levelDb-threshold)*(1/ratio-1)/20)` to within 0.01 (exact match in
practice).

### Learning a curve from example audio (`GainCurveTrainer`)

`plugin/dsp/GainCurveTrainer.{hpp,cpp}` fits a `NonlinearSchedule` to
(input, desired-output) audio pairs -- e.g. a reference compressor's
input/output, or manually-automated fader moves -- via **variable
projection** (separable least squares):

- The 2 truly nonlinear parameters, `attackMs`/`releaseMs`, are found by
  brute-force grid search (6 x 6 = 36 combinations spanning 0.2-100 ms
  attack, 10-3000 ms release).
- For *each* grid point, the 25 curve knot values are not searched at all
  -- they're the exact solution to a ridge-regularized (`lambda = 4 dB^2`,
  pulling knots with little/no supporting data toward 0 dB/transparent)
  linear least-squares problem, solved directly via Cholesky decomposition
  (`solveSPD()`). This is well-posed because, for a *fixed* envelope trace,
  the predicted gain is linear in the knot values (piecewise-linear
  interpolation is a linear function of its table).
- Whichever grid point has the lowest residual RMS (in dB) wins.

The residual envelope walk (for both fitting and scoring) uses the exact
same linear-domain-smoothing/dB-lookup procedure as the real-time engine
above -- they must match exactly, or the fitted curve would be fit against
one envelope shape and then played back against a different one.
`dsp_regression` test `[9]` round-trips a known synthetic schedule (fit
against audio generated by that same schedule) and recovers attack/release
exactly (they land on grid points) and the curve to within ~0.03 dB.

`tools/trainer/fit_gain_curve.cpp` is the CLI front-end: `fit_gain_curve
<base_preset.ssjson> <input.wav> <target_output.wav> <output_preset.ssjson>`
loads a base (LTI) preset, reads the two WAV files, runs the trainer, and
writes a new preset with the learned `nonlinear` block (after another pass
through `StabilityGuard::enforce()`, defensively). Learning itself is an
offline/CLI workflow, not something the plugin does live -- but its output
is a normal preset, so it loads and is then just as editable in
`NonlinearPanelWidget` (below) as a hand-drawn curve.

### GUI (`NonlinearPanelWidget`)

A full-width strip along the bottom of the window (`gui/NonlinearPanelWidget.*`),
wired into `UIStateSpace` the same way the four matrix grids are (through
`ssgui::IModelEditorHost`, extended with a small pair of accessors for the
25 curve knots -- see `ModelEditorHost.hpp`'s `MatrixKind::Nonlinear`
comment for why attack/release/threshold/ratio/makeup reuse the *existing*
click-to-edit machinery instead of needing new plumbing). Contents:

- an **Enabled/Disabled** toggle;
- **Attack (ms)** and **Release (ms)** fields -- real, persisted model
  fields, edited exactly like a matrix cell (click to edit pre-filled,
  double-click to clear, Enter/Tab commits, scroll nudges);
- **Threshold (dB)**, **Ratio (:1)**, **Makeup (dB)** fields plus an
  **Apply Curve** button -- these three are *not* part of the saved model
  (`StateSpaceMatrices` only ever remembers the resulting 25 knot values,
  not "how" they were authored -- exactly like a realized biquad doesn't
  remember its design Q). Apply Curve bakes them into all 25 knots at once
  via the same closed-form formula `dsp_regression` test `[8]` checks
  against, as a fast way to get a classic-compressor-shaped curve before
  fine-tuning it by hand;
- the **curve graph** itself: level (dB, x-axis, -72 to +24) vs. gain (dB,
  y-axis, a fixed -48 to +24 editing range) with the 25 knots drawn as a
  connected line. Click (and drag) anywhere in the graph to reshape it --
  the nearest knot by x position snaps to the pointer's y position --
  double-click a knot to reset it to 0dB (flat/transparent) instead of
  dragging it, the same double-click-resets convention every numeric
  field in this project follows -- and scroll to nudge whichever knot is
  nearest the cursor by 0.5 dB per notch. This is a first-draft
  simplification (freehand click/drag rather than per-knot numeric
  entry, following the same reasoning as the matrix grids' own "no
  click-drag fader" note in "Known limitations" below) -- a drag is
  clamped to the graph's own bounds rather than relying on
  pointer capture across widget boundaries, which this environment
  doesn't verify.

Nothing here touches `A`/`B` or the stability guard -- the panel only ever
writes `model_.nonlinear.*`, so every edit is exactly as safe as any other
model change and flows through `applyModelToEngine()` the same way a
matrix-cell edit does.

### The envelope-domain bug (linear vs. dB smoothing)

An earlier version of the engine smoothed the envelope directly in the dB
domain, applied to each sample's instantaneous level. `dsp_regression`'s
DC-level tests didn't catch this (a constant signal has no zero
crossings), but a realistic sine-tone probe did: measured compression gain
for the *same* input level varied by up to ~1.2 dB between a 100 Hz and a
3 kHz tone, purely because the higher-frequency tone crosses zero more
often per unit time, repeatedly yanking a dB-domain envelope toward
-infinity. Switching both the engine and the trainer to linear-domain
smoothing (dB conversion only at the point of curve lookup) reduced that
spread to ~0.03-0.1 dB. `audio_testbench` now has a permanent regression
check for this on every nonlinear-enabled preset (spread across all probed
frequencies, at a fixed input level, must stay under 0.3 dB) -- see
`docs/TESTING.md`.

## MultibandSchedule (multiband compression)

`NonlinearSchedule` above is *one* envelope tracking the whole,
already-summed output. `StateSpaceMatrices` carries a second optional
field, `multiband` (`StateSpaceTypes.hpp`'s `MultibandSchedule`), for a
*true* multiband compressor: up to `kMaxBands` (6) independent bands, each
with its own envelope (of *that band's own filtered signal*, not the
broadband input) and its own gain curve. `A`/`B` are, as always,
completely unaffected -- see `docs/EXTENDING.md`'s "Multiband compression"
section for the full mechanism (the crossover mapper, the per-band engine
branch, the trainer, and a real phase-alignment bug found building it), and
this file's own "Multiband compressor plugin" section below for the GUI
built on top of it.

Structurally, a `BandSplit` (one array entry per band) records a
contiguous `[stateStart, stateStart+stateCount)` state range and a
feed-through `feedD` -- both populated by
`mappers::makeStereoCrossoverBank()` (`CoefficientMapper.hpp`), which
assembles the crossover as a *parallel* connection of independent per-band
biquad chains (block-diagonal `A`, `C` concatenated by band, `D` summed --
the standard state-space parallel-connection formula, simpler than the
*series* connection `mappers::makeStereoBiquadChain()` uses for an EQ,
since bands don't feed each other's input). `feedD` has to be stored
separately per band because the assembled matrix's own combined `D` is
the *sum* of every band's `D`, and summed values can't be un-summed later
-- exactly the bookkeeping `StateSpaceEngine::processBlock()`'s multiband
branch needs to compute each band's own pre-gain contribution
(`feedD*u + C-slice*x`) independently, before applying that band's own
gain and summing into the final output.

The crossover itself is built from Linkwitz-Riley (not plain Butterworth)
splits specifically because an LR pair sums to *exactly* unity gain at
every frequency -- the reason a multiband compressor with every band's
gain at 0dB is indistinguishable from a plain passthrough rather than a
subtly EQ'd one. `dsp_regression` test `[15]` checks this flat-sum
property directly (worst deviation under 0.2dB across a 20Hz-15kHz sweep)
along with the `BandSplit` bookkeeping's own self-consistency (bands
exactly partition the assembled order; summed `feedD` matches the
assembled `D`).

### Training: orchestration, not new fitting math

`GainCurveTrainer::fitMultiband()` filters the (input, target) audio
through each band's own *standalone* chain (`mappers::
makeStereoBiquadChain()`, run through an ordinary `StateSpaceEngine`) and
calls the *existing*, completely unmodified `GainCurveTrainer::fit()` once
per band with an identity base model -- the ridge-regularized least-
squares fit itself is 100% reused; the only new code is deciding which
signal each band sees. `tools/trainer/fit_multiband_compressor.cpp` is the
CLI front-end, taking a comma-separated ascending crossover-frequency list
instead of a base preset (there's no separate "base filter" concept here
-- the crossover bank itself *is* the assembled model's A/B/C/D).

**A real bug found building this, worth recording in full**: the first
version of `fitMultiband()` filtered the (opaque, already-summed) target
through band k's own filter to recover that band's own signal, then
compared it *pointwise* against the input filtered through the same
filter *once*. This is wrong for a *phase*, not amplitude, reason (a
band's own passband gain deep from the crossover is within a thousandth
of a dB of unity even applied twice, verified directly -- amplitude was
never the problem). Any *sane* multiband compressor's true target audio is
`sum_k(gain_k(t) * filter_k(rawInput))`, so band k's own true contribution
has already gone through `filter_k` *once* before this function ever sees
the summed target; recovering it applies `filter_k` a *second* time --
leaving band k's true contribution phase-shifted by two applications of
the filter's own phase response, while the once-filtered input side only
carries one. `fit()`'s per-sample instantaneous-ratio technique
(`target[i]/base[i]`) is meaningless comparing two differently-phase-
shifted copies of the same waveform, even when their true envelope-level
relationship (the only thing actually being learned) is identical -- two
equal-amplitude, different-phase sine waves divide pointwise into noise,
not a constant. This produced large errors in practice (wrong
attack/release grid points, several dB of curve error) and was completely
invisible to the single-band trainer's own regression test (`dsp_regression`
test `[9]`), since that path never has a filtering asymmetry between its
two traces -- `fit()` there always compares a signal against itself run
through a plain identity model. Fixed by filtering the input through the
*same* filter *twice* too, so both traces `fit()` compares carry the same
two-application phase shift, leaving only the actual gain being learned as
a difference. `dsp_regression` tests `[16]` (the engine's per-band gain
stage against closed-form ground truth, mirroring test `[8]`) and `[17]`
(the trainer round-trip, mirroring test `[9]`) cover this.

## GUI (`gui/`)

Built on DPF's NanoVG-backed widget toolkit (immediate-mode-style drawing
over a retained widget tree). `UIStateSpace` owns the model, four
`MatrixGridWidget`s (one per A/B/C/D, laid out to mirror the textbook
`[[A,B],[C,D]]` block-matrix shape), a `BodePlotWidget`, and a toolbar of
`SimpleButton`s. `ssgui::IModelEditorHost` decouples the grid widgets from
the DPF-specific UI class (they only read/write cell values and request a
text edit through that interface).

Editing flow: click a cell to edit it pre-filled with its current value, or
double-click for an empty buffer (so typing replaces the value outright
instead of appending to the existing digits) -> type digits/`.`/`-`/`e` ->
Enter or Tab commits (Escape cancels, arrow keys commit-and-jump to the
neighboring cell within the same matrix). Clicking anywhere else while
editing also commits first, so finishing an edit doesn't depend solely on
the Enter key reaching the plugin window (not a safe assumption for a UI
embedded in a host -- see `beginCellTextEdit()`'s `getWindow().focus()`
call). Each commit re-runs the stability guard on the whole model, pushes
the result to the DSP via `setState("model", <json>)`, and marks the Bode
plot dirty. State flows DSP -> GUI via `stateChanged()` (covers host-driven
restores, e.g. loading a different session/program) and GUI -> DSP via
`setState()` (covers local edits and preset loads).

**Known limitations** (first-draft scope, not fundamental):
- No click-drag fader-style value adjustment -- only click-to-type and
  scroll-to-nudge. DPF's pointer-capture behavior across widget bounds
  wasn't verified in this environment, so a click-drag interaction was left
  out rather than shipped half-working.
- At high matrix orders (dozens of states) grid cells become too small to
  label; the widget hides text below a size threshold rather than render
  unreadable clutter, but there's no zoom/scroll/pagination yet.
- Phase is displayed wrapped to +-180 degrees (the conventional Bode
  convention); `TransferFunction::sweep()` itself returns an *unwrapped*
  phase, so a future group-delay display can use that directly.
- Bode plot curve visibility (which of the 4 channel pairs are shown) and
  the Load/Save file dialogs' last-used folder are persisted through the
  host's session state (see `PluginStateSpace.hpp`'s `StateIndex` enum),
  so both survive closing/reopening the plugin.

## EqModel / EQ plugin (`plugin_eq/`, `gui/eq/`, `plugin/dsp/EqModel.*`)

A second, separate plugin (`statespace_eq`, own LV2/VST3/JACK binaries,
own `DistrhoPluginInfo.h`) built entirely as a **coefficient mapper** on
top of the same DSP core -- see `docs/EXTENDING.md`'s "Building an LTI
processor" for the general pattern this follows, and its own header
comment for why it's a worked, fully-realized example of exactly that
rather than a mode of the base plugin. It's a parametric EQ in the
FabFilter-Pro-Q-like style: click on the graph to add a band, drag to
shape it, unbounded band count (in practice capped only by `kMaxOrder`,
see below).

**Why a second plugin, not a mode of the first**: the base plugin's whole
premise is "every coefficient is directly, permanently editable" -- a
raw matrix grid. An EQ's natural editing surface is the opposite: musical
parameters (type/frequency/gain/Q) driving the matrix, never touched
directly. Cramming both into one plugin would mean either exposing raw
matrix cells the EQ mode overwrites on every edit (confusing: whatever
you'd typed by hand would be silently clobbered the next time you moved a
band) or hiding the matrix editor behind a mode switch most sessions never
use. Two plugins, one shared core, is the cleaner boundary -- and it's
exactly what "export to the state-space plugin to hand-edit coefficients"
asks for architecturally: a one-way conversion from the musical
representation to the raw one, not a shared live model.

### EqModel (`plugin/dsp/EqModel.hpp/.cpp`)

`EqModel` is a `std::vector<EqBand>` (unlike `StateSpaceMatrices`, *not*
fixed-size -- there is no engine-side representation of an EqModel at
all, only of what it assembles into) plus an output trim. Each `EqBand`:
type (Bell/Low Shelf/High Shelf/Low Pass/High Pass/Notch/Band Pass/All
Pass -- the full RBJ cookbook set already in `CoefficientMapper.hpp`),
frequency, gain, Q, enabled, solo.

- **`assembleStateSpace(model, sampleRate)`**: designs each *active* band
  (every enabled band, or only the enabled-and-soloed ones if any band is
  soloed -- see `activeBandIndices()`) via the matching
  `mappers::designXxx()` formula, then series-cascades them with
  `mappers::makeStereoBiquadChain()` -- exactly what a multi-band EQ *is*,
  each band filtering the previous band's output -- and applies the
  output trim to `C`/`D`. An empty active-band list assembles to an
  identity passthrough. The result is an ordinary `StateSpaceMatrices`;
  the caller (`PluginEq::rebuildEngineModel()`, `UIEq::pushModelNow()`)
  runs it through `StabilityGuard::enforce()` and
  `StateSpaceEngine::setModel()` exactly like any other model in this
  project -- the EQ plugin reuses `StateSpaceEngine` completely unchanged.
- **`evaluateBandResponse()` / `evaluateCombinedResponse()`**: the *same*
  frequency response, computed a completely different way -- directly
  from each band's closed-form discrete biquad transfer function
  `H(e^{jw}) = (b0+b1 z^-1+b2 z^-2)/(1+a1 z^-1+a2 z^-2)` and multiplied
  together (series cascade = product of transfer functions, a basic LTI
  identity), rather than solving the assembled dense matrix via
  `TransferFunction::evaluate()`. `dsp_regression` test `[10]` confirms
  the two agree to numerical precision, as they must for the same LTI
  system. The *reason* for the second path is performance: benchmarked at
  a worst-case 32-band, all-high-Q order-128 model, `TransferFunction`'s
  dense sweep costs ~47ms per curve (see `StateSpaceTypes.hpp`'s
  `kMaxOrder` comment) -- fine for the base plugin's Bode plot, which only
  recomputes on a multi-frame debounce, but far too slow to redraw on
  every pixel of an interactive drag. The closed-form product is
  O(numBands) per frequency point instead of O(order^3), so
  `EqGraphWidget` redraws the true combined curve on every single mouse-
  motion event at no perceptible cost, while the (audio-only-visible, so
  less latency-sensitive) assemble+enforce+engine-push is debounced the
  same way the base plugin's Bode plot is -- see `UIEq::uiIdle()`.
- **Band count ceiling**: each band is 4 states (one biquad per stereo
  channel), so `kMaxOrder / 4` bands fit -- with `kMaxOrder = 128`, that's
  32 bands, comfortably past FabFilter Pro-Q's advertised 24-band maximum.
  `UIEq::addBand()` refuses silently past that rather than let
  `clampOrder()` silently truncate the chain to something that wouldn't
  match what the GUI shows.

### PluginEq (`plugin_eq/PluginEq.hpp/.cpp`)

Structurally identical to `PluginStateSpace` (same small Bypass/Reset/
Fault parameter set, `StateSpaceEngine` reused as-is) with one difference
worth calling out: it keeps the last-set `EqModel` around
(`rebuildEngineModel()`), because unlike the base plugin's raw
coefficients (which *are* the model, sample-rate-independent once
chosen), an `EqModel`'s biquad coefficients are derived from Hz values
and must be *re-derived* whenever the sample rate changes -- so
`sampleRateChanged()` reassembles from the stored `EqModel`, not just
`prepare()`s the engine.

### GUI (`gui/eq/UIEq.cpp`, `EqGraphWidget`, `BandListWidget`)

Two widgets share one `ssgui::IEqEditorHost` (in `EqEditorHost.hpp`,
the `EqModel` analogue of `ModelEditorHost.hpp`'s `IModelEditorHost`),
implemented by `UIEq` exactly the way `UIStateSpace` implements
`IModelEditorHost` for the matrix grids:

- **`EqGraphWidget`**: the primary Pro-Q-style interactive surface --
  log-frequency (x) vs. gain (y, an adjustable +-range, see below), every
  band drawn as a colored, draggable node plus its own dim individual-
  response curve, the bright combined curve on top. Click empty space to
  add a Bell band right there (immediately draggable without releasing
  first); drag a node to move it (a gain-less type -- high/low pass,
  notch, band pass, all pass, see `eqBandTypeHasGain()` -- only moves
  horizontally, its node pinned to the 0dB line rather than showing a
  gain effect that wouldn't do anything); scroll over a node to adjust
  its Q multiplicatively (proportionally the same nudge whether Q is
  currently 0.3 or 20). Three ways to remove a band, all equivalent:
  double-click its node, select it (click, or via the band list) and
  press Delete/Backspace (handled in `UIEq::onKeyboard()`, since keyboard
  events reach the top-level UI rather than subwidgets -- see
  `UIStateSpace`'s identical centralization), or right-click it for a
  small context menu (Delete, plus every band type as a direct pick --
  redundant with `BandListWidget`'s own type-cycle button, offered here
  too since reaching for the list for one quick change is friction).
  Whichever widget didn't initiate a deletion has no way to learn its own
  stored "selected band" index needs shifting (bands after the deleted
  one move down one slot) -- `UIEq::removeBand()` (the one place that
  actually performs the mutation, regardless of which widget asked)
  corrects both widgets' selection state itself rather than trusting the
  caller to have done it. Middle-click and hold a node to solo it (hear
  that band alone) for as long as the button stays down, restoring
  whichever solo state it actually had before on release (not just
  assuming it was off) -- a momentary "audition this band" gesture,
  distinct from `BandListWidget`'s persistent solo toggle, and drawn with
  its own gold ring so it doesn't read as "selected". Also draggable
  while held (reuses the same `dragBand_` machinery a left-click-drag
  does, which doesn't care which button started it), so a band can be
  repositioned while auditioning it alone.
- **Freq/gain/Q readout while dragging**: a small pill showing e.g.
  "1.20kHz  +3.5dB  Q 1.10" (frequency-only label, no dB, for a gain-less
  band type) tracks whichever node is currently being dragged, drawn just
  above it (or below, if that would clip above the graph's own top edge)
  -- covers the moment a band is first placed, since a click-to-add starts
  a drag immediately, as well as any later reposition, so there's always
  an exact numeric readout for a purely visual gesture.
- **`BandListWidget`**: the precise-numeric-entry and type-selection
  counterpart -- a scrollable row-per-band list (color swatch, a
  type-cycle button, Enabled/Solo toggles, click-to-edit Freq/Gain/Q
  fields with the same conventions as every other numeric field in this
  project, a delete button). There's no room on a small draggable node
  for a discoverable type selector or exact digit entry, so this list is
  where those live; selecting a row highlights the corresponding node on
  the graph and vice versa.
- **Gain-axis range**: a "±Ndb" label in the graph's top-right corner
  cycles through preset +-ranges (3/6/12/30 dB) -- a tight range for
  careful, sensitive-to-drag moves, a wide one for aggressive swings that
  still need to stay visible/reachable. Persisted through the host's
  session state (`PluginEq.hpp`'s `kStateYRangeDb`) like the base
  plugin's Bode curve visibility.
- **Background spectrum, pre- and/or post-EQ**: a live spectrum drawn
  behind the curves -- see "SpectrumAnalyzer" below for where the data
  comes from and how "how fast/smooth" is controlled, and "Two independent
  analyzers: pre-EQ and post-EQ" just below for how before/after
  comparison works. Cycled by the toolbar's "Spectrum Source" field
  (Off/In/Out/Both -- `ssfilter::SpectrumMode`), an inline click-to-cycle
  control (same convention as "Spectrum Resolution" right next to it, not
  a `SimpleButton` -- four short mode names read better as a labeled value
  box than crammed into a button that also has to say "Spectrum"),
  color-coded per mode to roughly match the graph's own trace colors. The
  mode and the smoothing time constant are both persisted through session
  state.
- **Spectrum's own autoscaled vertical range**: the spectrum is plotted
  through `EqGraphWidget::spectrumToY()`, a range *refit every single
  draw* to the current frame's own min/max -- the *union* of whichever
  trace(s) `spectrumMode_` currently shows, so "Both" gets one shared axis
  for a legitimate before/after comparison rather than two independently-
  scaled axes that would make the visual gap between the curves meaningless
  (headroom added, rounded to a clean 6dB step, clamped to a sane outer
  envelope) -- the same autoscaling principle `BodePlotWidget` already
  uses for its own magnitude axis -- deliberately independent of the EQ
  curve's own zoomable `gainRangeDb_`. Three iterations to get here, each a real
  first-draft bug rather than a design choice, worth recording in full:
  (1) originally shared the *same* axis/clamp as the EQ curve; since the
  spectrum shows *absolute* signal level (routinely -30 to -90dB per
  band once energy is split across the display bands) while the EQ curve's axis
  is a much smaller *relative*-gain zoom (as tight as +-3dB), a normal-
  level signal's spectrum sat pinned invisibly below the visible range;
  (2) fixing that with a display-gain *boost* into the *same shared*
  range didn't actually fix it either -- the boosted value still hit the
  same clamp, so most of a spectrum's real dynamic range (especially the
  quiet low end, exactly where a bigger FFT's resolution improvement is
  concentrated) still clamped to the same bottom pixel regardless of its
  true value, making the "Spectrum Resolution" control look like it did
  nothing even though the underlying numbers genuinely differed by
  30-40dB (confirmed directly, see `SpectrumAnalyzer`'s own comment); (3)
  giving the spectrum a *fixed but generous* range of its own (instead of
  the EQ curve's) stopped the clipping, but spreading a real 20-40dB
  difference across such a wide fixed span still looked nearly flat --
  technically correct, unconvincing in practice. Autoscaling every frame
  is what actually fixes it: computed and verified directly (a real
  before/after comparison across two FFT presets on the same signal), it
  turns what used to be an imperceptible pixel shift into up to ~220
  pixels of visible difference on a 300px-tall graph. It's also *why*
  there's no manual "how loud should this look" control any more (the
  removed "Spectrum Gain (dB)" field): autoscaling refits to whatever's
  there every frame, so a constant additive boost before it would shift
  the *range* by the same amount and produce an identical picture --
  offering a control with no visible effect would be worse than not
  offering one.
- **Spectrum's own scale, labeled on the graph's right edge**: since the
  autoscaled range above changes every frame, its numbers are drawn
  directly on the graph -- `+N`/`-N` dB labels at `graphX_+graphW_+3.0f`,
  gridded every 6dB (or 12dB once the autoscaled span exceeds 24dB, to
  avoid crowding), in a color distinct from the EQ curve's own left-edge
  gain-axis labels (they measure different things entirely: the EQ
  curve's fixed, zoomable *relative* gain vs. the spectrum's *absolute*,
  frame-to-frame-autoscaled level, and conflating them by drawing both on
  the same edge or in the same color would read as if they shared an
  axis). The graph's right margin was widened (8px to 30px) to make room.
- **Autoscale toggle, off by default, and slow instead of snapping when
  on**: refitting the range to the *current frame's* min/max every single
  draw (the previous entry above) turned out to have its own real cost
  once actually lived with -- program material's own min/max moves
  constantly even after the trace itself is smoothed
  (`spectrumSmoothMs_`), so the *axis itself* (gridlines, and the curve's
  apparent position relative to them) kept resettling on every redraw --
  visually disturbing even though each individual frame's fit was
  technically correct. Two changes, not a reversal of the
  autoscaling decision above: (1) `setSpectrumAutoscale()` (an on-graph
  "Autoscale"/"Fixed Range" toggle, persisted through session state --
  `EqGraphWidget`, `MultibandGraphWidget` and `DynEqGraphWidget` all carry
  the identical control) lets it be turned off entirely, in favor of a
  fixed range (`kFixedSpectrumRangeLoDb`/`HiDb`, -66..-6dB -- deliberately
  the same numbers the autoscale path already fell back to for a
  degenerate/silent frame, so "off" is "usually about where autoscale
  would settle anyway, just held still"), and **off is the default**;
  (2) when autoscale *is* turned on, the target range is still
  recomputed fresh every frame exactly as before, but the range actually
  drawn is a slow (600ms time constant) one-pole toward that target rather
  than an instant snap, so the axis visibly drifts instead of jumping even
  under real, constantly-shifting material.

**Display-band count** (`kNumBands`, log-spaced 20Hz-20kHz): raised from
64 to 192 (~19/octave instead of ~6.4/octave) purely to make the plotted
curve itself denser/smoother. This is a separate axis from the FFT-size preset above and doesn't change
where that preset's own resolution improvement is visible: FFT bins are
*linearly* spaced (`sampleRate/fftSize` Hz per bin, uniform across the
spectrum) while the display bands are *log*-spaced, so a high band
already spans many native bins at any preset (little room to improve)
while a low band spans very few or none (all the room to improve, and
exactly why the low-frequency bin-widening below exists at all). More
display points make the curve denser everywhere; they don't spread the
FFT-size effect more evenly, because no number of sample points changes
the underlying bins' physical spacing. See `SpectrumAnalyzer.hpp`'s own
header comment for the full explanation.

Both widgets read/write `UIEq`'s single `EqModel` through the host
interface; `UIEq` debounces the expensive assemble+`StabilityGuard::enforce`+
`setState()` push the same way `uiIdle()` debounces the base plugin's
Bode plot recompute, while the graph and list redraw every frame
regardless (cheap, see EqModel.hpp's closed-form math above), so the
curve and node positions always look instantaneous even though the
audio-affecting engine update lags by a few frames.

### SpectrumAnalyzer (`plugin/dsp/SpectrumAnalyzer.hpp/.cpp`)

A real-time-safe magnitude-spectrum estimator, feeding `EqGraphWidget`'s
background spectrum -- purely a visual reference layer with zero effect
on the audio path. Runs entirely inside `PluginEq::run()`: mono-mixes a
signal, accumulates it into a fixed-size ring buffer, and every hop
boundary (fftSize/4 samples) windows (Hann) and FFTs (a standard
iterative radix-2 Cooley-Tukey, no allocation/recursion) the last
`fftSize` samples, then republishes `kNumBands` (192) log-spaced
20Hz-20kHz band magnitudes (dB) via lock-free `std::atomic<float>` stores.

#### Two independent analyzers: pre-EQ and post-EQ

`PluginEq` (and `PluginEqMono`) each own **two** `SpectrumAnalyzer`
instances -- `inputSpectrumAnalyzer_` fed the raw pre-EQ signal,
`outputSpectrumAnalyzer_` fed whatever's actually in the output buffer
(the real post-EQ signal, or a copy of the input while bypassed, which
correctly makes the two traces coincide -- "no filtering is happening" is
exactly what that should look like, not a case to special-case around).
`ssfilter::SpectrumMode` (`SpectrumAnalyzer.hpp`) is the shared four-value
enum -- `Off`/`Input`/`Output`/`Both` -- both DSP and UI sides cycle
through via `nextSpectrumMode()`/`clampSpectrumMode()`, so there's exactly
one place that defines what the states are and what follows what.

`run()` only calls `process()` on the analyzer(s) `spectrumModeWantsInput()`/
`spectrumModeWantsOutput()` actually selects -- an unselected analyzer's
FFT/windowing cost genuinely isn't paid at all, not merely hidden from the
GUI -- switching a side off genuinely saves DSP. Verified with a standalone harness:
an analyzer that's never fed sits frozen at `kFloorDb` forever, and running
a 500Hz lowpass on a 5kHz test tone reads back ~40dB of attenuation
between the two analyzers' readings at the same band -- a real, large,
independently-measurable difference, not just two copies of the same
number.

`spectrumMode_` is DSP-relevant (unlike the smoothing time constant or
gain-axis range below, which stay UI-only preferences), so it travels as
a `State` (`"spectrumMode"`, `PluginEq.hpp`'s `kStateSpectrumMode`) rather
than something the DSP side never reads. Doubling the analyzer count also
doubles the output-parameter bank: `kParamSpectrumOutputBase` and
`kParamSpectrumInputBase` (each `+[0..kNumBands)`) replace the single
`kParamSpectrumBase` bank from before this feature -- 384 output
parameters total instead of 192, still the same portable per-band
`Parameter` mechanism described below, just two banks of it. Both
analyzers share one `"spectrumFftSize"` preset (there is only one
"Spectrum Resolution" control in the UI) -- no reason to let them drift to
different resolutions.

**FFT size is a runtime-selectable preset** (2048/4096/8192/16384,
default 8192), not a fixed constant -- the "Spectrum Resolution" cycle
control in `UIEq`'s toolbar. A bigger FFT gives finer low-frequency
resolution (bin width = sampleRate/fftSize) at the cost of more buffering
latency (fftSize samples' worth -- ~341ms at 16384/48kHz) before a change
in the signal shows up at all; the *CPU* cost barely moves either way (an
FFT's O(n log n) work divided by a hop that scales with n nets out to
O(log n) per unit time, so 16384 vs. 2048 is only about 3x the log2 term
-- negligible on any modern CPU), even though the control was originally
requested as a way to trade resolution for CPU headroom. Since the actual
buffers (`ring_`/`window_`/`fftRe_`/`fftIm_`) are shared with the audio
thread, changing the active preset can't safely happen from the UI/
message thread directly -- `requestFftSizeIndex()` is just an atomic
store; the actual reconfiguration (`applySizeIndex()`: recomputes the
window and band-edge tables, clears the ring buffer) happens from inside
`process()` itself, i.e. on the audio thread, the next time it's called,
mirroring this project's other wait-free hand-off patterns (e.g.
`StateSpaceEngine::setModel()`) rather than introducing a lock. The
DPF-side plumbing is a `State` (`PluginEq`'s `"spectrumFftSize"`, an
index, not a continuous `Parameter`), since it's a discrete resolution/
latency choice a host has no reason to automate.

Since there is no direct shared-memory link between the DSP and UI sides
in general (they can be different processes for some plugin formats/
hosts), each band of each analyzer is exposed as an ordinary DPF *output*
Parameter (`kParamSpectrumOutputBase + [0..kNumBands)` and
`kParamSpectrumInputBase + [0..kNumBands)`) -- the exact same portable
mechanism `kParamFaultFlag` already uses, and the pattern DPF's own
Meters example plugin uses for this exact purpose. `UIEq::parameterChanged()`
routes each update by which bank the index falls into, into
`spectrumOutRaw_[]`/`spectrumInRaw_[]`; `uiIdle()` applies the same
one-pole exponential smoothing filter to both (unconditionally, regardless
of `spectrumMode_` -- smoothing 192 floats twice is trivial, so there's no
reason to special-case "don't bother with the hidden one"; only the *DSP*
side's `run()` actually skips real work for an unselected analyzer) toward
`spectrumOutSmoothed_[]`/`spectrumInSmoothed_[]`, with a time constant
from the "Spectrum Smoothing (ms)" field -- this is deliberately a
*UI-side-only* preference (no DSP-side parameter/state round-trip) so
"how fast/smooth" is instant to change and has nothing to do with the
audio path.

Two calibration properties worth recording (both verified directly,
`dsp_regression` test `[11]`):
- **Amplitude**: derived from Parseval's theorem -- summing (not
  averaging) `|X[k]|^2` across a band's captured bins and applying one
  global normalization factor (`2/sqrt(fftSize * sum(window^2))`) reads
  a pure tone back at its true dB level regardless of how many bins that
  band spans, which varies enormously across a log-spaced axis (from a
  handful at 10kHz to less than one at 20Hz). *Averaging* instead of
  summing was tried first and under-read a multi-bin band by several dB
  whenever a tone's leaked energy didn't fill the whole band -- exactly
  the routine case.
- **Low-frequency bin widening**: at any FFT size, the resolution
  (sampleRate/fftSize -- ~11.7Hz-per-bin at the 8192 default, at 48kHz)
  is coarser than the lowest log-spaced bands themselves, but a Hann
  window's mainlobe is intrinsically ~4 bins wide regardless -- so every
  band's capture range is widened to at least 5 bins (centered on the
  band, not clamped to its own narrow edges) to reliably catch a tone's
  full leaked energy. This is a knowing trade-off (see
  `SpectrumAnalyzer.cpp`'s own comment): it makes discrete low tones read
  back essentially exactly, at the cost of slightly over-reading
  broadband/noise-like content in those same narrow bands -- accepted
  since spotting a peak/notch under the EQ curve is this analyzer's whole
  purpose, not absolute noise-floor calibration. A bigger FFT preset
  narrows the range this widening actually matters for, since more bins
  naturally fall in each band to begin with.

### Exporting to the base plugin

`UIEq`'s "Export to State-Space..." button does nothing but
`Serialization::toJson(StabilityGuard::enforce(eq::assembleStateSpace(eqModel_, sr)))`
-- an ordinary `.ssjson`, byte-for-byte the same format
`PluginStateSpace`/`UIStateSpace` read and write, because it *is* one.
There is no bespoke interchange format between the two plugins: the base
plugin's "Load..." button opens an exported EQ directly, showing every
resulting `A`/`B`/`C`/`D` entry (a block-lower-triangular structure from
the series cascade -- confirmed by inspection, not just by the response
curves matching) ready for hand-editing.

**Known limitations** (first-draft scope, same spirit as the base
plugin's own list above):
- No per-channel/mid-side EQ curves -- both stereo channels always share
  identical band coefficients (`makeStereoBiquadChain(stages, stages)`,
  same `stages` on both sides).
- No "dynamic EQ" (a band whose gain reacts to its own input level) --
  architecturally this is exactly what `NonlinearSchedule` (the base
  plugin's state-dependent gain stage, see the section above) already
  solves for a *single* global envelope; combining a per-band envelope
  with `EqModel` is a natural next step, not attempted here.
- No linear-phase mode -- every band is a minimum-phase IIR biquad by
  construction (the same recursive `A/B/C/D` state-space form the whole
  project is built on); a linear-phase EQ needs an FFT/FIR-convolution
  architecture that shares essentially nothing with this engine.
- Learning a curve from audio (`fit_gain_curve`, for the nonlinear gain
  stage) has no EQ equivalent -- there's no "auto-EQ from a reference
  track" workflow here, only hand-placed/dragged bands.
- No per-band spectrum resolution/smoothing override -- "Spectrum
  Resolution" and "Spectrum Smoothing (ms)" apply to both the pre- and
  post-EQ traces uniformly (one FFT size preset, one smoothing time
  constant), not independently per trace.

## Mono EQ plugin (`plugin_eq_mono/`, `gui/eq/UIEqMono.*`, `plugin/dsp/Mono*`)

A third plugin, 1-in/1-out, for projects where only mono is used. The
user-facing story is "the same EQ, cheaper"; the implementation story is
about *where* that cheapness comes from, which is the only part worth
documenting.

### Why a separate matrix type, engine and guard

The obvious implementation -- keep the existing stereo `StateSpaceEngine`
and just feed the mono signal into both channels, or leave one channel's
`B`/`C`/`D` at zero -- **saves nothing**. `StateSpaceEngine`'s dominant
per-sample cost is the `O(order²)` `A`-matrix recursion, and that cost is
paid over the *whole* active order regardless of how many of those entries
happen to be zero. A stereo EQ assembles two independent biquad chains
(one per channel, 4 states/band); zeroing one of them still leaves an
order-`2N` matrix being multiplied out every sample.

*(Since the sparse-cascade fast path was added, that last sentence is no
longer quite true for `StateSpaceEngine` itself: a cascade-
built matrix's zero entries are now provably structural
(`StateSpaceMatrices::sparseCascade2`), and the recursion skips them
rather than multiplying them out. That's still strictly worse than this
section's actual point, though -- a real `O(N²)` matrix at half the order
beats a sparsity-optimized `O((2N)²)` one at full order regardless, since
the sparse path's own cost is still bounded below by the size of each
2-wide diagonal block times the number of blocks, not by the *number* of
nonzero entries a smaller matrix would need in the first place. The
measured multipliers in the table below predate the fast path and were
not re-measured against it -- the stereo side of that comparison is now
faster than what's shown, so the *actual* current gap is smaller than the
table states, not larger; treat the numbers below as "at least this much
faster", not exact. `MonoStateSpaceEngine` itself was not given the same
fast path in that pass -- a reasonable next step, using the identical
technique, if this comparison is ever revisited with fresh measurements.)*

So the mono variant assembles a genuinely different, smaller system:

| | Stereo EQ | Mono EQ |
|---|---|---|
| Realization | two SISO chains (`makeStereoBiquadChain`) | one SISO chain (`makeMonoBiquadChain`) |
| States per band | 4 | 2 |
| Order at 32 bands | 128 | 64 |
| Recursion cost | `O((2N)²)` | `O(N²)` -- a quarter |

Measured (10s of audio per engine, identical band lists): **3.1× / 2.8× /
4.2× / 3.9× / 4.4×** faster at 4 / 8 / 16 / 24 / 32 bands respectively --
converging on the predicted 4× as the band count rises and the fixed
per-sample overhead stops dominating.

That size difference is why there is a `mono::MonoStateSpaceMatrices`
(single-column `B`, single-row `C`, scalar `D`) rather than a reused
`StateSpaceMatrices`, and hence a `MonoStateSpaceEngine` and
`MonoStabilityGuard` to operate on it. `MonoStabilityGuard` is a
deliberate, straightforward copy of `StabilityGuard`'s algorithm (same
norm bound, same repeated-squaring spectral-radius estimate, same additive
margin and refinement loop) rather than a templating of it -- the stereo
guard is the single most bug-prone and most carefully-fixed file in this
project (see the notch bug test `[7]`), and making it
generic would have meant editing it. `dsp_regression` test `[14]` pins the
two guards to agreeing on the same filter, so the copy can't silently
drift.

### What is *not* duplicated

Everything above the matrix layer is shared, unchanged:

- **`ssfilter::EqModel`** -- the band list itself has no channel count. The
  same type, the same `.sseqjson` format, the same `designBand()` RBJ math.
  Only the final assembly step differs (`assembleMonoStateSpace()` vs.
  `assembleStateSpace()`), and both call the identical per-band designer.
- **`EqGraphWidget` and `BandListWidget`** -- reused verbatim, not copied.
  Nothing about mono vs. stereo is visible to them; they talk to
  `IEqEditorHost` (`gui/eq/EqEditorHost.hpp`), which `UIEqMono` implements
  exactly as `UIEq` does. This is why the two plugins' GUIs cannot drift
  apart: there is only one graph widget and one band list in the codebase.
- **`SpectrumAnalyzer`** -- already mono-only internally (the stereo plugin
  mixes L/R down before feeding it), so the mono plugin simply skips that
  mix and feeds its single channel directly.

`UIEqMono` is a near-copy of `UIEq` (the toolbar, the inline field editing,
the debounce, the file dialogs). The one substantive difference besides the
assembler is `exportStateSpaceFile()`, which deliberately still uses the
**stereo** `assembleStateSpace()`: StateSpace MIMO is the only plugin able
to open a `.ssjson` and it is stereo-only, so a mono-shaped export would be
unopenable. The exported file is a stereo pair with both channels
identical -- an inspection/hand-editing artifact; the mono audio engine
itself only ever runs the mono-assembled system.

**Known limitations**, in addition to inheriting the stereo EQ's list
above:
- The 32-band ceiling is the same as the stereo plugin's, despite mono
  needing half the states per band. It's set by
  `mappers::detail::kMaxChainStates` (the SISO cascade builder's own fixed
  buffer, 64 states = 32 biquads), not by `kMaxOrder` -- so mono gets the
  same maximum band count at a quarter of the per-band cost, rather than a
  doubled maximum. Raising it would mean raising `kMaxChainStates`.
- No mono/stereo switch inside one plugin. Channel count is fixed at
  instantiation in every host format this targets, so this is a plugin
  identity, not a toggle -- the same reasoning that made the EQ a separate
  plugin from StateSpace MIMO rather than a mode of it (see the section
  above).

## Multiband compressor plugin (`plugin_mb_compressor/`, `gui/mb/`, `plugin/dsp/MbCompressorModel.*`)

A fourth plugin (`statespace_mb_compressor`, own LV2/VST3/JACK binaries,
own `DistrhoPluginInfo.h` for the same include-ambiguity reason
`plugin_eq/`/`plugin_eq_mono/` have their own), structurally identical to
`PluginEq`/`UIEq` -- same small Bypass/Reset/Fault parameter set, same
spectrum output-parameter mechanism, same debounced assemble+enforce+push
pattern -- built entirely as a coefficient mapper (`MbCompressorModel`)
plus a GUI on top of the `MultibandSchedule` mechanism described above,
following the exact "musical parameters -> coefficient mapper ->
StateSpaceMatrices" pattern `EqModel` already is (see `docs/EXTENDING.md`'s
"Building an LTI processor" section for the general shape, and its
"Multiband compression" section for the crossover/per-band-engine
mechanism this plugin is a GUI on top of).

### MbCompressorModel (`plugin/dsp/MbCompressorModel.hpp/.cpp`)

`MbCompressorModel` is a `std::vector<double> crossoverHz` (ascending) plus
a `std::vector<MbBand> bands` (threshold/ratio/ratioUp/attack/release/
makeup/enabled/solo -- `ratioUp` mirrors `ratio` *below* the threshold
instead of above it, genuine upward compression alongside the original
downward one, `1.0` meaning no upward effect; see `GainCurve::
classicCompressor()`'s own comment for the shared two-sided formula both
this and `DynamicEqModel` use), always `crossoverHz.size() + 1` entries -- an invariant only
`mb::addCrossover()`/`mb::removeCrossover()` are allowed to change, so it
can never drift out of sync. `assembleStateSpace()` calls
`mappers::makeStereoCrossoverBank()` (unchanged) for the crossover, then
turns each band's musical parameters into a `NonlinearSchedule` via
`GainCurve::classicCompressor()` (`StateSpaceTypes.hpp` -- the same
closed-form formula `UIStateSpace::applyCompressorCurve()`'s "Apply Curve"
button bakes into a hand-drawn curve, pulled out as a shared helper
specifically so this plugin didn't need a third copy of it) and drops the
result into the assembled `MultibandSchedule`. No new DSP math at all --
this file is pure bookkeeping over already-tested primitives.

One correctness detail worth recording, found while writing this: output
trim (`outputTrimDb`) has to scale each band's `BandSplit.feedD` *in
addition to* the assembled matrix's own `C`/`D` -- because this plugin's
model always sets `multiband.enabled = true`, the engine's multiband
branch (which reads each band's own `feedD`, never the combined `D`
field) is the *only* code path that ever runs for it. Scaling `C`/`D` and
forgetting `feedD` would have silently made Output Trim do nothing at
all, not merely something slightly wrong -- caught by inspection while
implementing, not by a symptom, which is exactly the kind of easy-to-miss
"but does it actually reach the audio path" question this project's own
past bugs (the click/pop fix, the aliasing fix) keep coming back to.

Solo/mute reuses the gain-curve mechanism itself rather than a structural
"this band doesn't exist" special case: a band muted by another band's
solo gets its curve forced to -100dB at every level (`designBandSchedule()`),
and a *disabled* (but audible) band gets a flat, transparent 0dB curve --
its own audio still plays, only the dynamics are bypassed. Both states are
just different curves, so nothing else in the engine or the graph needs to
know about them.

`evaluateCombinedResponse()` is the EQ's `evaluateCombinedResponse()`
pattern adapted to a *parallel* (not series) connection: the *sum* of each
band's own closed-form biquad-chain product (reusing `eq::
evaluateBiquadResponse()` unchanged), O(numBands) per frequency point
instead of the generic O(order^3) `TransferFunction` sweep -- this draws
the crossover's shape on the graph, not each band's own gain curve (a
per-level, not per-frequency, quantity). `dsp_regression` test `[18]`
confirms it agrees with `TransferFunction::evaluate()` on the assembled
matrix, as it must for the same LTI system -- this holds regardless of
each band's own threshold/ratio, since `TransferFunction` has no notion of
the runtime-only multiband gain stage at all, exactly like the single-band
`NonlinearSchedule` case.

### GUI (`gui/mb/UIMbCompressor.cpp`, `MultibandGraphWidget`, `gui/ParamStripWidget`)

**`MultibandGraphWidget`** is `EqGraphWidget`'s own log-frequency-axis,
band-color-palette, and (verbatim-reused mechanism) autoscaled background-
spectrum machinery, adapted to a narrower interaction set appropriate for
a crossover rather than a per-band-gain EQ curve: click a band's shaded
region to select it; drag a crossover split handle horizontally (absolute
position->frequency, clamped so splits can never cross); double-click
inside a band's region to split it there (adds a crossover); double-click
a handle to remove it (merges the two bands back into one); scroll over a
band's region to adjust its Ratio multiplicatively (mirroring the sibling
`../compressor` project's own "scroll the curve to adjust Knee"
convention, since there's no natural on-graph handle for Ratio the way a
Q-bearing EQ node has one). Deliberately simpler than `EqGraphWidget` in
two ways: no right-click context menu (there's no band *type* to pick
here, every band is the same kind of compressor) and no live gain-
reduction meter on the graph itself -- see "Known limitations" below for
why that second one was deferred.

**`gui/ParamStripWidget`** (shared, not `gui/mb`-specific) is ported, to
reuse its interaction model, from the sibling `../compressor` project's
widget of the same name -- a row of
click-to-edit numeric fields that also supports **click-and-drag vertical-
fader adjustment** (press and drag up/down; a small dead zone tells a drag
apart from a click-to-type), the one numeric-field interaction this
project's own matrix cells and gain-curve knots deliberately left out ("DPF's
pointer-capture behavior across widget bounds during a drag wasn't
verified in this environment" -- see `docs/ARCHITECTURE.md`'s "Known
limitations" for the base plugin's own note on this). That concern is
about a drag straying *outside the whole plugin window*, not outside the
widget's own bounds within it -- DGL hands a widget every motion event for
the rest of a gesture it started, regardless of where the pointer strays,
as long as it stays inside the window, which is all a fixed-position
field's vertical fader drag needs; the `../compressor` project had already
established this works, so this project's own "left out, unverified"
caveat no longer has to block it here. `UIMbCompressor` uses two instances:
one for the always-present Output Trim / Spectrum Smoothing fields, one
re-pointed at whichever band is currently selected for its Threshold/
Ratio/Attack/Release/Makeup.

**Selecting bands**: a single `ParamStripWidget` re-targeted at
`selectedBand_`, rather than a per-band-row scrollable list the way the
EQ's `BandListWidget` shows every band's fields at once -- a deliberate
simplification (a compressor band has 5 numeric fields against an EQ
band's 3, and "select the bands and set up attack/release" was the literal
ask, not "show me every band's numbers simultaneously"). Selection syncs
both ways between the graph (click a region) and the toolbar's Prev/Next
buttons; Enable/Solo toggle buttons and a "Band X of N" readout sit
alongside.

**Export/Import** follows `UIEq`'s pattern exactly: Load/Save use this
plugin's own `.ssmbjson` format (`mb::toJson()`/`fromJson()`, the
`MbCompressorModel` analogue of `.sseqjson`); "Export to State-Space..."
does nothing but
`Serialization::toJson(StabilityGuard::enforce(mb::assembleStateSpace(model, sr)))`
-- the exact same code path/format the base plugin's own "Load..." reads,
so an exported multiband compressor's crossover bank (and every band's
baked-in gain curve) is inspectable and hand-editable there, with zero new
interchange format needed. A preset produced by the `fit_multiband_compressor`
CLI trainer is already a `.ssjson` (the assembled result, not the musical
band-list form), so it loads directly in either this plugin's "Load..."
being irrelevant to it or in StateSpace MIMO -- there is no in-GUI "train
from audio" workflow, matching the single-band compressor's own equivalent
gap (see `docs/EXTENDING.md`'s "Possible next steps").

**Live gain-reduction/makeup meter** (added in a follow-up session, no
longer a "Known limitation" below): `StateSpaceEngine::getBandGainDb()`
exposes each multiband band's own per-sample gain (a wait-free atomic
array, the same contract `DynamicEqEngine::getCurrentGainDb()` already
established) via a new `kParamBandGainBase` output-Parameter bank
(`PluginMbCompressor.hpp`); `MultibandGraphWidget::drawBandGainMeter()`
draws it as a compact bipolar meter (0dB-referenced fill up or down --
bipolar because Up Ratio can now make a band's gain a boost, not just a
cut) for whichever band is selected, with a separate always-visible tick
at that band's configured Makeup value. Deliberately shown for the
*selected* band only, not all of them at once, to stay legible with up to
`kMaxBands` bands on one graph.

**Known limitations** (first-draft scope):
- No per-band transfer-curve display (each band's own individual response
  curve overlaid on the crossover shape, the way `EqGraphWidget` shows
  each EQ band's own dim curve alongside the combined one) -- the shaded
  region + gain meter above are the substitute for now.
- No per-band-row numeric list the way the EQ's `BandListWidget` offers --
  one field strip re-targeted at the selected band instead (see above).
- No in-GUI "train from audio" workflow -- `fit_multiband_compressor`
  stays CLI-only, same gap the single-band compressor has.
- No mono variant yet (unlike the EQ, which got one in a follow-up
  session) -- `MbCompressorModel`/`mappers::makeStereoCrossoverBank()` are
  already channel-count-agnostic in the same way `EqModel`'s design/
  assembly split is, so a mono variant would follow the exact same
  `MonoStateSpaceTypes`/`MonoStateSpaceEngine`/`MonoStabilityGuard` pattern
  the mono EQ already established, not a new architecture.

## Dynamic EQ plugin (`plugin_dyn_eq/`, `gui/dyneq/`, `plugin/dsp/DynamicEqModel.*`, `plugin/dsp/DynamicEqEngine.hpp`)

A fifth plugin (`statespace_dyn_eq`, own LV2/VST3/JACK binaries, own
`DistrhoPluginInfo.h`) -- a dynamic EQ in the spirit of FabFilter Pro-Q's,
"a multiband compressor on top of the filters" -- but, unlike every plugin
above, its live audio path is **not**
a `StateSpaceMatrices`/`StateSpaceEngine` instance at all. See
`docs/EXTENDING.md`'s "Dynamic EQ" section for the full derivation of why
neither `NonlinearSchedule` nor `MultibandSchedule` fit an *overlapping*
bell band the way they fit a broadband stage or a non-overlapping
crossover slice, and why a continuously gain-varying series cascade has no
fixed state-space realization to assemble into in the first place.

### DynamicEqModel (`plugin/dsp/DynamicEqModel.hpp/.cpp`)

`DynamicEqModel` is a `std::vector<DynamicEqBand>` -- each band a
frequency/Q/resting-gain bell plus its own threshold/ratio/rangeDb/attack/
release, the same musical-parameter shape `EqModel`'s bands and
`MbCompressorModel`'s bands both are. The mechanism that makes a
continuously-variable-gain bell safe is the classical Regalia-Mitra
fixed-allpass peaking construction: `H(z) = 0.5*(1+G) + 0.5*(1-G)*A(z)`,
where `A(z)` is the *existing* RBJ cookbook allpass
(`mappers::designAllpass()`, already in `CoefficientMapper.hpp`,
unchanged) shaped only by frequency/Q, never gain -- so `G` (the linear
gain) enters purely affinely and can be swept every sample from a level
detector without ever touching a pole, exactly the same "only ever
`C(p)`/`D(p)`" discipline `NonlinearSchedule` established, generalized to
an affine (not purely multiplicative) combination. `resolveDynamicGainDb()`
is the gain-computer: flat at the resting `gainDb` at/under `thresholdDb`,
sloping by `1 - 1/ratio` above it, clamped to a `rangeDb` ceiling (the "how
far can this band move" cap a per-band peak limiter needs that a broadband
compressor's uncapped curve doesn't). `evaluateBandResponse()`/
`evaluateCombinedResponse()` are the closed-form response functions the
GUI graph uses -- cheap enough (like `EqModel`'s own) to redraw on every
drag event or idle tick, and able to evaluate at either a band's resting
gain or an externally-supplied *live* one (see DynamicEqEngine below), for
the graph's static-shape-vs-moving-curve distinction.

`assembleStateSpace()` -- used only by "Export to State-Space..." -- is a
deliberate exception: it freezes every band at its resting `gainDb` via
the *ordinary* RBJ `designPeaking()` cascade (`EqModel`'s own mechanism),
not the Regalia-Mitra one, because a fixed matrix fundamentally cannot
represent a moving filter at all -- the exported preset is meant to be an
honest, hand-editable static EQ snapshot, not an opaque frozen allpass
combination.

### DynamicEqEngine (`plugin/dsp/DynamicEqEngine.hpp`)

A dedicated real-time engine, not a mapper feeding `StateSpaceEngine` --
the project's second one (after `MonoStateSpaceEngine`) built as its own
self-contained class rather than the generic MIMO core, because a series
cascade of independently time-varying stages has no single fixed `A`/`B`/
`C`/`D` a *mapper* could ever hand that core. Per active band, per sample:
run the band's own fixed Regalia-Mitra allpass and a separate genuine
(magnitude-shaped) detector bandpass on the same incoming (cascade-
position) signal; track the detector's envelope (linear-domain one-pole
attack/release smoothing, `max(|L|,|R|)` -- the identical convention
`StateSpaceEngine`'s own `NonlinearSchedule`/`MultibandSchedule` handling
already uses); resolve to a gain via `resolveDynamicGainDb()`; combine via
the Regalia-Mitra formula; the result feeds the next band. Threshold/
ratio/range/attack/release/gainDb switch instantly (the same "already a
smoothed envelope-driven scalar, not a recursive state" reasoning
`StateSpaceEngine` gives for `NonlinearSchedule`'s own parameters); the
shaper/detector filter *coefficients* (frequency/Q-derived) are one-pole
coefficient-smoothed toward their target every sample, mirroring
`StateSpaceEngine`'s own A/B/C/D coefficient smoothing for exactly the
same "a fast drag would otherwise apply a suddenly-different filter to
whatever's already in its delay line" reason. A band-*count* change gets
the identical duck-to-silence-then-swap fix `StateSpaceEngine` uses for an
EQ order change, for the identical "no meaningful in-between" reason.

Live per-band gain is exposed via `getCurrentGainDb()`, a wait-free atomic
readback (the same single-writer/single-reader contract as
`consumeFaultFlag()`) -- `PluginDynEq` carries it across to the UI as a DPF
output `Parameter` per possible band slot (`kParamCurrentGainBase`), since
the DSP and UI sides can be in separate processes with no direct memory
link, the same reason spectrum data already crosses that boundary as
parameters rather than a getter call.

### GUI (`gui/dyneq/UIDynEq.cpp`, `DynEqGraphWidget`, `gui/ParamStripWidget`)

**`DynEqGraphWidget`** reuses `EqGraphWidget`'s own node-drag interaction
(drag a node for frequency+gain, scroll for Q, double-click to remove,
click empty space to add) -- a dynamic EQ band is still fundamentally a
bell node on a frequency/gain graph -- with no band-type context menu
(every band is the same Regalia-Mitra bell, the same simplification
`MultibandGraphWidget` already made for its own bands), and the identical
freq/gain/Q drag readout `EqGraphWidget` has (see that section above).
Adds two things a
static EQ graph doesn't need: a shaded "dynamic range" region per band
(the area between that band's response curve at its resting gain and at
`gainDb - rangeDb`, via the same closed-form evaluator with a different
resolved gain plugged in each time) and a second, brighter combined curve
using every band's *live* `getCurrentGainDb()` instead of its resting
gain -- redrawn every `uiIdle()` tick regardless of whether anything was
dragged, so it visibly moves in response to program material, matching
the FabFilter screenshots' moving yellow trace that prompted this feature.

**Field strip**: all eight of a band's numeric fields (Freq/Gain/Q/
Threshold/Ratio/Range/Attack/Release) live in one `ParamStripWidget`
re-targeted at whichever band is selected -- the same "select then edit"
simplification `UIMbCompressor`'s own file comment argues for (there, 5
fields against an EQ's 3; here, 8), with Freq/Gain/Q *also*
graph-draggable/scrollable, the same intentional redundancy
`EqGraphWidget`/`BandListWidget` already have for an EQ band.

**Export/Import** follows the same pattern as every sibling plugin: Load/
Save use this plugin's own `.ssdynjson` format; "Export to State-Space..."
is `Serialization::toJson(StabilityGuard::enforce(dyneq::assembleStateSpace(model, sr)))`
-- the frozen, resting-gain RBJ snapshot described above, inspectable and
hand-editable in the base plugin's own "Load...".

**Known limitations** (first-draft scope), mirroring the multiband
compressor's own equivalent list for the identical reasons: no live
gain-reduction meter on the graph (would need a metering path out of
`DynamicEqEngine`, not yet added), no mono variant (would need its own
dedicated single-channel engine, since `DynamicEqEngine` isn't a mapper
feeding the generic core to begin with -- not a reuse of
`MonoStateSpaceEngine`), and no band type beyond Bell (a Regalia-Mitra-
style fixed-shelf construction exists in the literature, just not
implemented here).
