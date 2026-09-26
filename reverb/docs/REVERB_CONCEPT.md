# Reverb plugin: concept

Status: **built and installed for real.** `./build_and_install.sh --test`
runs end to end -- configure, build all four targets (LV2/VST3/JACK/
AU-gated), `reverb_regression` (`[R1]`-`[R12]`, all passing), install into
`~/.lv2` -- with **zero warnings outside vendored DPF/DGL code**. The
standalone JACK build was launched on a real X display and screenshotted
(ImageMagick's `import`, already present -- no stub/tool install needed);
that screenshot caught and led to fixing two real bugs (see "GUI" and
"Investigating the reported mouse-stealing bug" below) before this status
line was last edited. Read "How this was tested" for exactly what's
verified and how; docs/REVERB_CONCEPT.md itself is now closer to a normal
project's docs than a "here's what should work" writeup.

This is its own top-level project (`reverb/`, sibling to `../statespace/`,
`../compressor/`, etc.), not a plugin inside `statespace/` -- see "Where
this lives" below for why.

## Design goals

A stereo reverb built around the same state-space philosophy this project
already uses -- explicitly *because* state-space lets more variables be
modeled "in the background if needed" -- with signal able to cross from
one stereo channel to the other and back. Usual reverb parameters (decay,
short/long reflections, room size), with room size preferably expressed as
height/width/length rather than a small preset list, but performance
(fast, natural-sounding, real-time-safe DSP) takes priority over that
preference if the two trade off. A parametric EQ included in-plugin to
shape the reverb -- after the reverb (to "filter it before it goes out")
or possibly mid-processing, described as being like placing absorbing
*objects* in the room. Research reverb design in general and via analogy
to how light is modeled in a renderer like Blender first; build (and be
able to test/improve) a working core before the GUI; reuse this project's
existing core/EQ/matrix-plugin machinery wherever it genuinely fits,
rather than building something unrelated from scratch.

## Why this can't be "just another mapper" onto the generic core

Every other plugin in this project (EQ, multiband compressor, dynamic EQ)
follows `docs/EXTENDING.md`'s pattern: musical parameters in, a
`StateSpaceMatrices` out, `StabilityGuard::enforce()`, `StateSpaceEngine`
runs it. That only works because none of those need more than `kMaxOrder`
(128) states -- an EQ band is 4 states, a crossover band a handful more.

A room reverb's whole character comes from delay lines tens to hundreds of
milliseconds long. At 48kHz, 100ms is 4800 samples. Realizing that as
literal state-space order (one state per delay-line sample, run through
the generic dense `A` matrix) is a non-starter twice over: it's an order
far beyond 128 to begin with, and the generic engine's per-sample cost is
`O(order^2)` -- at order 4800 that's ~23 million multiply-adds *per
sample*, not real-time by a wide margin. `StateSpaceTypes.hpp`'s own
`sparseCascade2` fast path (skip provably-zero terms in a cascaded-biquad
matrix) doesn't rescue this either -- it still costs one multiply-add
per state per sample along each stage's own 2-wide diagonal block, and a
4800-sample delay line has no 2-wide block structure to exploit; its
"cascade" is 4800 *trivial, unit-gain* stages, not a handful of resonant
ones.

This is the same situation `DynamicEqEngine.hpp` is already in for a
different reason (its own file comment: "a series cascade of
independently time-varying stages has no single fixed `A`/`B`/`C`/`D`
[the generic core] could ever hand"), and the project's own precedent for
it is exactly what this reverb follows: **a dedicated, self-contained
engine, not a mapper feeding `StateSpaceEngine`.**

### It is still, formally, a state-space system -- which is the actual point

A [Feedback Delay Network](https://ccrma.stanford.edu/~jos/pasp/FDN_Reverberation.html)
(FDN; Jot 1991/1992, the standard structure behind essentially every
serious real-time algorithmic reverb since, including hardware units and
every major plugin) *is*, by definition in the DSP literature, a discrete
state-space system `x[n+1]=Ax[n]+Bu[n]`, `y[n]=Cx[n]+Du[n]`. Concatenate
every delay line's contents into one state vector `x[n]`; almost all of
`A` turns out to be a pure shift/permutation (each state's next value is
just the state one slot behind it in its own line -- a zero-cost move, not
a multiply), and the only genuine multiply-add coupling happens once per
sample at each line's read/write boundary, through a small `N x N`
feedback-mixing matrix -- `N` the number of delay *lines* (8-16, a design
constant), not delay *samples*. Implementing that provable structure as
circular buffers instead of a dense matrix multiply is exactly the
`sparseCascade2` idea ("don't multiply by a coefficient that's provably,
structurally zero") taken to its logical extreme: here, *all but O(N)* of
the per-sample state transition is structurally zero or identity.

The result is a system with an effective order in the thousands -- a lot
of state, genuinely "modeling more variables in the background," which is
exactly what state-space is used for here -- that costs
`O(N)` per sample to run, completely independent of how large the room
is. A cathedral and a closet cost the DSP the identical amount per sample;
only the (message-thread-only) delay-line *lengths* and the fixed-size
buffers holding them differ. That is the "long to compute the model, fast
to run it" split, taken as literally as possible: room geometry
-> delay lengths -> feedback matrix -> damping filter coefficients is all
message-thread/parameter-change-time work (see `ReverbEngine::prepare()`);
`processStereo()`, the only thing that runs per sample, is `O(N)`.

## Research: how reverb is actually built, and the light-transport analogy

Surveyed the standard DSP literature (Schroeder 1962; Moorer 1979; Jot
1991/1992's FDN, still the field's reference structure; Stanford CCRMA's
"Physical Audio Signal Processing" reverb chapters) rather than any single
plugin's internals. Three points carried directly into this design:

- **Absorptive (damping) filters per delay line**, not just a scalar
  feedback gain, are how every real FDN gets a *frequency-dependent* decay
  time (a room's high frequencies always die faster than its low ones --
  air and material absorption are both frequency-dependent) -- Jot's
  closed-form one-pole design (exact DC/Nyquist target gains) is what
  `OnePoleDamping::design()` implements.
- **Lossless (orthogonal/unitary) feedback matrices** -- Hadamard or
  Householder -- are the standard way an FDN stays provably stable *by
  construction* rather than by runtime eigenvalue estimation: the mix step
  itself can never add energy, only the (deliberately lossy) damping
  filters remove it. Householder was chosen here over Hadamard
  specifically for the "must run fast" priority: a Householder reflection
  costs `O(N)` per application for *any* `N` (sum the vector once, subtract
  `2*sum/N` from every element), where a general Hadamard multiply is
  `O(N^2)` unless `N` is a power of 2 *and* the fast Walsh-Hadamard
  transform is used to bring it to `O(N log N)` -- Householder's plain
  `O(N)` is both simpler to implement correctly and cheaper at this
  project's target line counts.
- **Geometric acoustics splits into a "specular"/discrete part and a
  "diffuse"/statistical part** -- and this is where the light-transport
  analogy (how a renderer like Blender models light) turns out to be exact, not just
  evocative: the *image-source method* used for a room's earliest,
  individually-audible echoes is the acoustic version of mirroring a light
  source across a flat surface to find a specular reflection's path in a
  ray tracer (the same "unfold the geometry" trick either way); the
  *diffuse, statistically-dense tail* an FDN produces is the acoustic
  analogue of radiosity/path-tracing's diffuse global-illumination pass --
  both give up on tracking individual paths once there are too many of
  them to matter individually, and instead solve for the *statistics* of
  how energy is distributed over time (acoustics) or over a surface
  (light). This is precisely why the design below is a **hybrid**:
  discrete image-source early reflections for the first ~30-50ms (where a
  listener can still hear individual echoes as directional/spatial cues),
  handed off to the FDN's dense diffuse tail after that (where individual
  paths are no longer perceptually distinguishable, only their aggregate
  decay/density/color are) -- the standard "hybrid reverb" architecture
  (e.g. Steimel's *Implementation of a Hybrid Reverb Algorithm*, Aalborg
  University) rather than a novel invention.

This project deliberately stops at **first-order** image sources (one
mirror per wall, 6 for a shoebox room) rather than growing into recursive
multi-order path tracing: perceptually, only the first one or two
reflection orders are heard as *distinct* echoes at all; beyond that,
density rises fast enough that the FDN's own statistical tail is already
the right tool, and full recursive image-source computation grows
combinatorially (6 -> 36 -> 216 sources per order) for perceptual benefit
that's marginal past order 1-2. This mirrors why a renderer doesn't path-
trace every surface to convergence either -- diminishing, then negative,
return past a certain bounce depth for how a scene is actually perceived.

## The three pieces, and what's been verified about each

All three live in `plugin/dsp/ReverbEngine.hpp`; all four claims below are
checked by `tests/reverb_regression.cpp` (`[R1]`-`[R7]`), not just argued
-- see "How this was tested".

1. **Room dimensions -> delay-line lengths** (`designDelayLengths()`): the
   room's *mean free path* (`4*Volume/SurfaceArea`, a standard room-
   acoustics quantity -- the average distance sound travels between wall
   hits) sets a base travel time; `kNumLines` (8) lines are spread
   geometrically (0.55x-2.4x) around it and each rounded to the nearest
   **prime** sample count. Prime lengths are the standard FDN trick
   (Jot; CCRMA's "Prime Power Delay-Line Lengths") for making sure no two
   lines' echoes ever land on a simple integer ratio of each other, which
   is what produces audible metallic/ringing coloration in a cheaply-built
   FDN. **What "mean free path" is and isn't doing here**: it is *not* a
   claim that these 8 lines model this room's actual specific reflection
   paths (a real room has an enormous, geometry-specific set of those) --
   it's a principled, closed-form way to make "bigger room" produce
   "proportionally longer, still well-spread delay lines" without an
   expensive room-mode simulation, which is exactly the level of physical
   fidelity every real-world FDN reverb (hardware or plugin) actually
   uses for its own late tail. Verified (`[R1]`): for a 10x7x3.5m room, 8
   distinct primes from 293 to 1277 samples (6.1ms-26.6ms at 48kHz) --
   sane range, all within the fixed storage bound, no duplicates.
2. **Frequency-dependent decay** (`OnePoleDamping`): Jot's exact two-point
   fit -- a one-pole shelf whose DC gain and Nyquist gain are solved in
   closed form to hit exactly the per-pass decay factor a **Decay Time
   (T60)** target implies at each frequency. Verified (`[R2]`): measured
   low-band (<120Hz) T60 of the *actual engine's* impulse response, via a
   Schroeder backward-integrated energy-decay-curve measurement (the
   standard acoustic-measurement technique, the same one professional room
   acoustics software uses), matched a 1.8s target to **within 0.2%**
   (1.803s measured). Verified (`[R3]`): with a Damping parameter set to
   make Nyquist decay twice as fast as DC (0.9s vs 1.8s), the measured
   high-band (>15kHz) decay came back faster (1.28s) than the low-band
   target, confirming the parameter has the intended directional effect
   at all -- the kind of check `docs/ARCHITECTURE.md`'s own "Spectrum
   Gain" story warns is easy to skip and ship a silently-inert control.
3. **Lossless cross-line mixing, and stereo cross-coupling**
   (`householderMix()`): a Householder reflection matrix, exactly
   orthogonal by construction. Verified (`[R4]`): input and output energy
   of the mix step alone matched to `1e-9` (floating-point-exact, not
   approximately). Verified (`[R5]`, the literal "reflect signals from one
   stereo channel to the other and back" ask): an impulse fed into the
   left channel only produces substantial right-channel energy within the
   first 80ms (measured energy ratio R/L = 0.37) -- because every line
   feeds the *same* shared matrix every sample regardless of which
   channel originally fed it, exactly like this project's own base plugin
   already supports arbitrary 2x2 cross-channel `D`/`A` coupling (mid/
   side-style matrices) for its own, much smaller, LTI systems.

Additionally verified: `[R6]` a 10-second continuous-noise stress run
stays finite and bounded (max sample magnitude ~2.0 against 0.5-amplitude
noise input -- no runaway); `[R7]` first-order image-source early
reflections for the same room land at 17-29ms with finite positive
gains, a sane range preceding/overlapping the FDN tail's own onset
(shortest line ~6ms); `[R8]` the tempo-to-decay note-value table matches
hand-computed values including the 3/4-time-signature case and the
host-not-reporting-tempo fallback (see "Tempo-synced decay"); `[R9]`
coefficient smoothing measurably suppresses the internal discontinuity an
instant Decay Time retarget causes (>200x reduction for a drastic
1.8s -> 0.3s change) -- also see "Tempo-synced decay" for a real
architectural subtlety this test surfaced (the discontinuity never
reaches the engine's own output tap the way it would in the dense
`StateSpaceEngine` case, since a delay line's circular buffer reads
before it writes).

### Stability is closed-form here, not iterative

Unlike the generic core's `StabilityGuard` (which *estimates* a spectral
radius via repeated squaring because no full eigendecomposition is
implemented), this engine's stability argument is exact by construction:
the feedback matrix is provably orthogonal (spectral radius exactly 1,
never estimated), and every per-line damping filter's gain is in `(0,1)`
whenever its T60 input is a positive number (which is enforced by the UI
range, not by run-time validation). An energy-preserving mix composed
with a strictly-lossy filter, every line, every pass, can only ever lose
energy -- there's no configuration of "usual" reverb parameters (decay
time, damping, room size) that can make this system unstable, in sharp
contrast to a hand-edited raw matrix in the base plugin. The per-sample
NaN/Inf-substitution safety net in `ReverbEngine::processStereo()` exists
anyway, for the identical reason `StateSpaceEngine.hpp` gives for its own
second net: no amount of upstream design validation proves a recursive
filter stays finite forever under sustained input and accumulated
floating-point error.

## Parameters

Mapped from the design goals, biased toward the room-dimension approach
but with performance-neutral presets layered on top (not instead):

| Parameter | Maps to |
|---|---|
| Input Gain (dB) | applied once, before the signal splits into the dry path and the tank's own input -- see `PluginReverb::run()`'s own comment for why both, not just the wet path |
| Room Length / Width / Height (m) | `RoomDims` -> `designDelayLengths()` |
| Decay Time (T60, s) | `OnePoleDamping`'s DC target, via `ReverbEngine::setDecayTargets()` (smoothed) |
| Tempo Sync (opt-in) + Note Value | `t60SecFromTempo()` overrides manual Decay Time, quarter/half/bar/2-bar (+dotted/16th/8th) -- see "Tempo-synced decay" |
| Follow Host Tempo (opt-in, requires Tempo Sync) | per-block `getTimePosition()` re-read feeding `setDecayTargets()`, smoothed -- live BPM changes glide, don't click (see "Tempo-synced decay") |
| Damping / Brightness | `OnePoleDamping`'s Nyquist target, relative to Decay Time |
| Pre-Delay (ms) | **(Done)** `kParamPreDelayMs`, 0..250ms, default 0 -- `plugin/dsp/PreDelayLine.hpp`, a pure delay applied only to the copy of the signal feeding the tank (never the dry path). Model-independent (shared via `#include` across both plugins, unlike the tank engines themselves -- see that file's own header comment). Glides (5ms one-pole smoothing) toward a live-changed target rather than snapping, and is an exact identity at 0ms -- see `tests/predelay_regression.cpp`. |
| Early Reflections Level / Size | scales `firstOrderImageSources()`'s tap gains and the implicit source/listener offset used to compute them |
| Diffusion | **(Done)** `AllpassDiffuser` -- 4 cascaded Schroeder allpass stages per channel, ahead of the tank, gain scaled by this 0..1 amount (0.7 default). Added because the first build sounded very metallic, like in a tube -- see `ReverbEngine.hpp`'s "Diffusion" section for the full derivation and why this, not a Decay Time/Damping tweak, is the actual fix. |
| Spread (labeled "Width" internally, `kParamWidth`/`stereo_spread`) | **(Done)** plain mid/side blend applied only to the tank's own wet output, right before it's returned from `ReverbEngine::processStereo()` -- 0 = mono, 1 (default) = the tank's natural image (a mathematical identity, `tests/reverb_regression.cpp` [R13]), 2 = exaggerated side. Displayed as "Spread" in the GUI, not "Width", specifically to avoid sitting next to Room Width (meters) under the same name. |
| Balance (`kParamBalance`) | **(Done)** a plain per-channel gain differential on the wet signal, applied after Spread -- -1..+1, 0 (default) = unity gain both channels (a genuine Balance, not a constant-power Pan). Added to correct (or creatively shift) any left/right lean; also the direct occasion for finding and fixing a real ~0.16dB structural L/R asymmetry in `designDelayLengths()`'s own channel assignment -- see `ReverbEngine.hpp`'s comment there and `tests/reverb_regression.cpp` [R14]/[R15]. |
| Modulation (`kParamModulation`) | **(Done)** a slow per-line LFO-modulated fractional delay (`ModulatedDelay`) inside each tank line's own feedback loop, 0..1 (0.3 default) -- the fix for uncontrolled resonant frequencies that sounded like the material pitch-shifted far down (audible FDN mode ringing, worst at low Damping). Zero added latency and an exact identity at 0.0, confirmed measurably engaged at the default and stable at maximum depth -- see `ReverbEngine.hpp`'s "Delay-line modulation" section and `tests/reverb_regression.cpp` [R16]. |
| Dry/Wet, Output Trim | ordinary post-mix, same as every sibling plugin |
| **Room Shape presets** (Hall/Room/Plate/Chamber) | *only* set starting defaults for the fields above (a JSON preset, `.ssjson`-family convention), never a separate processing mode -- satisfies "usual" preset-style plugins are familiar with, without giving up the dimension-driven core |

### Multiple models: separate plugins, one shared GUI

Once the metallic/ringing issues above were fixed: several reverb
plugins with different names, the same interface but a different sound,
so each user can pick what they like or what their CPU budget allows --
separate plugin binaries rather than a runtime mode-switch parameter.

**Done:** `reverb_dense` (`plugin/dsp/ReverbEngineDense.hpp`, a 16-line
FDN -- denser mode spacing, a smoother tail than the tables/mechanism
description below were originally written against; this document's
parameter tables and shared-GUI mechanism apply unchanged, only the
engine underneath differs) and `reverb_plate`
(`plugin/dsp/ReverbEnginePlate.hpp`, a Dattorro-style figure-8 coupled
delay network -- a genuinely different topology, not another line-count
change: `reverb_dense` (then still called `reverb_smooth`) had a
different color but the same processing problems, because N-line FDNs at
any N share the same underlying mechanism). An original
third sibling, the 8-line `reverb` this document's own mechanism
description/tables were first written against, was removed once
reverb_dense and reverb_plate both sounded clearly better -- the shared engine foundation it and reverb_dense both
used (`plugin/dsp/ReverbEngine.hpp`) stays, since reverb_dense's own
engine still reuses its shared types via `#include`. Both remaining
plugins share the identical GUI source (`gui/UIReverb.cpp/.hpp` and every
widget file, compiled unmodified into both plugin targets) and the
identical `ParameterIndex`/`StateIndex` enums, extracted into
`plugin/ReverbParams.hpp` specifically to make this possible. Each new
engine is a genuine hand-adapted parallel file rather than a
C++ template (the same "deliberate copy, not genericity" precedent this
codebase already established for `MonoStabilityGuard`), and five real
bugs found and fixed while verifying `reverb_plate` specifically: a
~4.3x steady-state loudness mismatch against `reverb` (structural, from
the plate's own much-longer/less-frequent loop turnover), a ~14% T60(DC)
error (from allpass group delay exceeding nominal buffer length), ~124ms
of true silence before ANY wet signal at ANY Pre-Delay setting (an
audible pre-delay even at 0ms -- the engine's own output taps all sat past nearly a full tank traversal, with no
early-reflection contribution at all), a ~30dB gap in the envelope
between that fix's own early tap and the late tail's first full loop
trip (fixed with a spread cluster of taps instead of one), and then --
after ALL of those still left audible echo/delay problems -- a genuinely
different defect: a visible SECOND HUMP in the waveform
around 150-220ms, comparably loud to the real onset, since the tank's
own first full loop trip arrives as one large undispersed burst (only
two recirculating tanks here, unlike the FDN models' own 8-16
independent lines). Three scalar probes (tail autocorrelation,
echo-density peak-counting, cross-correlation against a synthetic clap)
all came back inconclusive on this last one; what actually found it was
rendering the impulse response as a waveform image and looking at it
directly -- now the recommended first move for this class of problem,
not a last resort.
Fixed with a late-tail fade-in over roughly one loop period. All five
measured, corrected, and locked in by `tests/reverb_regression_plate.cpp`
`[P2]`/`[P4]`/`[P13]`/`[P14]`/`[P15]`, not just assumed fixed.

**Training/distillation idea -- stage 1 done, stage 2 not started.** The
"literal learned structure" direction, deliberately two-staged: first a model to PRODUCE reference training samples, the
fitting/learning step itself later. Stage 1 is `tools/reference/`
(`ImageSourceMethod.hpp`/`ReferenceEngine.hpp`/`generate_reference_samples.cpp`)
-- a higher-order (Allen & Berkley 1979) shoebox image-source renderer,
the physically-grounded generalization of `firstOrderImageSources()`
above, deliberately independent of the FDN/plate technology both
shipped plugins already use (so it's a genuine ground truth, not just a
fourth parametrized approximation). Includes a synthesized hand-clap
excitation signal (clapping hands is the common real-world way to hear
a room's reflections), plus a noise burst and a sine sweep, and picks up
real dry recordings dropped into `<out_dir>/user_dry/` automatically --
this tool can only synthesize the standard acoustic MEASUREMENT signals,
not real dry music (no way to fetch/license that here). The tool runs
an order-1 sanity check against `firstOrderImageSources()` on every
invocation. Stage 2 (fitting a fast structure against this tool's own
output, and separately, later, testing how well `../statespace`'s own
state-space/matrix machinery can learn this kind of behavior at all) is
not started -- explicitly scoped as a later step once real samples exist.

"Short vs. long reflections", the other pairing in the design goals, maps
naturally onto this same set rather than needing its own new parameter:
**short** = the discrete early-reflection taps (Pre-Delay + Early
Reflections Level/Size), **long** = the FDN tail's own Decay Time --
exactly the hybrid architecture's own two halves, not a coincidence.

## Tempo-synced decay

An opt-in mode where Decay Time (and, symmetrically, Pre-Delay) is
*derived* from the host's own tempo instead of set directly in seconds --
including tracking a live BPM change while the DAW keeps playing, also
opt-in on top of the base sync toggle. Grounded in two things checked
directly rather than assumed, not designed from memory:

- **The note-value convention** several real online delay/reverb-time
  calculators already use (checked directly -- see References): `time
  (ms) = 60000 / BPM * noteValue`, and specifically for reverb decay (not
  just delay time), a documented rule of thumb -- a quarter note reads as
  a short ambience, a half note a medium room, one bar a long hall, two
  bars an epic tail. `plugin/dsp/ReverbEngine.hpp`'s `TempoNoteValue`
  enum and `t60SecFromTempo()` implement exactly this table (plus
  sixteenth/eighth/dotted variants for finer control), and
  `OneBar`/`TwoBars`/`FourBars` correctly scale with the host's own
  *time signature* (`beatsPerBar`), not a hard-coded 4/4 -- verified in
  `tests/reverb_regression.cpp` `[R8]` (e.g. one bar at 120 BPM in 3/4
  correctly comes out to 1.5s, not the 4/4 answer of 2.0s).
- **DPF's real tempo API**, grepped directly from the vendored `dpf/`
  source rather than assumed from memory: `Plugin::getTimePosition()`
  returns a `TimePosition` whose `.bbt.beatsPerMinute` (and
  `.bbt.beatsPerBar`, `.bbt.valid`) is populated every process block by
  every backend this project could target --
  `dpf/distrho/src/DistrhoPluginLV2.cpp` (via LV2's `time:` atom
  extension -- the format Ardour itself, this project's primary target,
  uses), `DistrhoPluginVST3.cpp` (`ctx->bpm`), `DistrhoPluginCLAP.cpp`
  (`transport->tempo`), `DistrhoPluginJACK.cpp` (JACK transport), and
  `DistrhoPluginAU.cpp`. Every non-LV2 backend also defaults to 120 BPM
  when the host doesn't report tempo at all -- `t60SecFromTempo()`
  matches that same fallback (`bpm <= 0` -> 120) rather than inventing a
  different default DPF itself doesn't use. This means "opt-in even if
  the DAW changes BPM" is a plain per-block re-read of
  `getTimePosition()` at the plugin's own `run()` call site (Phase 2,
  see below) feeding `ReverbEngine::setDecayTargets()` -- no new
  mechanism, an existing, already-proven DPF hook this project's other
  plugins simply haven't needed before.

**A real subtlety found building this, worth recording in full** (see
`ReverbEngine.hpp`'s own `OnePoleDamping` comment and `tests/
reverb_regression.cpp`'s `[R9]` for the full account): a live BPM follow
means Decay Time can retarget every single block, so the natural worry --
carried over directly from `StateSpaceEngine.hpp`'s own coefficient-
smoothing story ("dragging an EQ band fast enough produced audible
clicks/pops") -- was that an unsmoothed retarget here would click the
same way. It doesn't, for an architectural reason specific to a delay-
line-based engine that doesn't apply to the dense matrix case: a circular
buffer *reads* this sample's output tap before it *writes* this sample's
newly-damped value, so a coefficient change can't reach
`ReverbEngine::processStereo()`'s own output for a full trip around that
line (6-27ms for the tested room) -- confirmed directly, an earlier
version of `[R9]` measured *zero* output-tap difference between an
instant and a smoothed retarget for exactly this reason, which is what
prompted rewriting that test to measure the right thing. The real,
measured discontinuity an instant retarget causes is internal to each
line's own damping filter -- its `y[n]=g*(1-a)*x[n]+a*z1` recursion,
freshly-snapped `g`/`a` applied to a `z1` that accumulated under the
*old* coefficients -- and `[R9]` measures it directly: **~0.046 for an
instant retarget vs. ~0.0002 for a smoothed one, a >200x reduction**, for
a drastic 1.8s -> 0.3s Decay Time change. `ReverbEngine::setDecayTargets()`
(smoothed, for live use) and `snapDecayTargets()` (instant, deliberately
kept as a real API entry point -- see below) both exist now, not just
planned.

This also settled a related design question cleanly: a **room-size**
change (which changes every line's own *length*, not just its damping)
should still use the instant `snapDecayTargets()`-style path, packaged
inside a duck-to-silence-then-swap exactly like `StateSpaceEngine`'s own
order-changing-edit handling (see "Phased build plan" below) -- there's
no meaningful in-between two different delay lengths to glide through
either, the identical reasoning that file's own comment already gives.
Decay-Time-only changes (manual or tempo-follow) use the smoothed path
instead, since the lines themselves don't change size.

## Investigating "no audible BPM effect"

After the first plugin-class pass shipped, the BPM-dependent decay had
no audible effect in practice. A line-by-line re-audit of `run()`'s
tempo-sync branch (`PluginReverb.cpp`) found no logic bug -- the gating
(`followHostTempo_ || !tempoCaptured_`), the fallback (`bbt.valid` false
or `beatsPerMinute <= 0` -> 120 BPM, matching DPF's own per-backend
default exactly), and the actual `t60SecFromTempo()` call all check out
against what `tests/reverb_regression.cpp` `[R8]` already independently
verifies. With no real host available in this environment to reproduce
it directly, and a default Quarter-note-at-120bpm decay (0.5s)
differing drastically from the default manual one (1.8s) -- large enough
that *some* difference should have been unmistakable even under the
fallback -- the most likely explanations are UX, not DSP: there was no
GUI at all before this pass, only a host's raw generic-parameter list, in
which finding and correctly toggling both `Tempo Sync` (a plain boolean
among 11 undifferentiated entries) and confirming it actually changed
anything was genuinely hard to do with confidence.

Two things followed from this, not a silent "assume it's fine":

1. **Three new diagnostic output Parameters**, captured every `run()`
   call regardless of whether Tempo Sync is even on: `kParamHostTempoValid`
   (is the host reporting BBT/tempo at all right now?), `kParamHostBpmDetected`
   (the raw value DPF currently holds), and `kParamCurrentDecayTimeSec`
   (`ReverbEngine::currentT60DcSec()` -- the decay time *actually active*
   in the engine right now, manual or tempo-derived). These turn "did I
   hear it" into "can I see it", which is the strictly stronger check --
   see `ReverbEngine.hpp`'s new `currentT60DcSec()`/`currentT60NySec()`
   getters.
2. **The GUI** (see below) surfaces both directly: a live
   `Host Tempo: 128.0 BPM (valid)` / amber-fallback line, and a
   `Decay (synced)` field that visibly displays the currently-active decay
   number the instant Tempo Sync is on, updating with the host's tempo if
   `Follow Host Tempo` is also on.

**What this does and doesn't resolve**: it makes the mechanism's actual
state directly observable, so a re-test can now say *exactly* where
things stand -- "the BPM readout never goes green/valid" (a real host/DPF-
wiring issue, needs investigating on that side specifically) vs. "the
readout is valid and the decay number changes, but it still isn't
audible" (a genuinely different, narrower problem than either of the two
possibilities being conflated) vs. "it works now that the fields are easy
to find" (a UX-only issue, now fixed by the GUI itself). It does not, on
its own, prove the feature works in a real host -- that still needs
testing there; see "How this was tested" for what's actually been
verified without one.

## Parametric EQ integration

**(Done, post-tank placement)** `EqModel`/`CoefficientMapper.hpp`/
`StateSpaceEngine`/`StabilityGuard`/`EqGraphWidget`/`BandListWidget` are
ported **verbatim** from `../statespace` (see "Where this lives" for why
a port, not a shared in-repo dependency) -- the same "nothing above the
matrix layer needs to change" reuse that project already relies on for
its own mono EQ plugin. Two placements were designed; only the first
ships in this pass:

1. **Post-tank (built, default, always available)**: an ordinary
   `EqModel`, assembled + `StabilityGuard::enforce()`'d + run through an
   ordinary `StateSpaceEngine` (`PluginReverb::eqEngine_`), applied to the
   wet (reverb) signal in place, *before* the dry/wet mix (`PluginReverb::run()`)
   -- the direct "filter the reverb before it goes out, seeing frequencies
   like the parametric EQ" ask. The band list is a whole-model `State`
   (`setState("eqModel", eq::toJson(...))`), not the sibling `../statespace`
   project's own Parameter-mirror bank -- see "GUI" below for that
   deliberate trade-off.
2. **Mid-tank ("objects in the room")**: not built this pass. The framing
   maps directly onto *replacing* a delay line's simple one-pole damping
   filter with a fuller `EqModel`-designed biquad chain in that same slot
   -- physically, that's exactly what an object (a rug, an open window, a
   bookshelf) does to a room's reverberant field: it adds a fixed,
   frequency-shaped *loss* to every recirculation of the sound that passes
   near it, which is precisely where the per-line damping filter already
   sits in this design. This reuses the *existing* per-line filter slot
   rather than inventing new machinery, and stays cheap (still `O(N)` per
   sample, not `O(N * numEqBands)` unless a user actually adds bands
   there) -- but it changes the tank's own frequency-dependent decay
   shape, not just its output tone, so it needs its own careful design/
   testing pass and stays a scoped follow-up, matching the "performance
   matters more" priority: ship (1), heard and confirmed, before touching
   the tank's own recirculation path with (2).

### Why "Export to State-Space" doesn't fit here, and what does instead

Every sibling plugin's "Export to State-Space..." button works because
its assembled system fits in `kMaxOrder` (128). This reverb's tank cannot
-- its effective order is in the thousands, and no `StateSpaceMatrices`
could ever hold it. Debugging/inspection instead follows this project's
*other* established convention, `audio_testbench`'s own approach (see
`docs/TESTING.md`): write real WAV files (impulse response, swept sine,
noise) to disk for inspection in Audacity/a spectrum analyzer, and a
`report.txt` with the same measurements the tool prints -- which is
exactly what `tests/reverb_regression.cpp`'s own T60/energy checks are
already halfway to being an offline CLI tool for. A `.ssreverbjson`
preset format (this reverb's own design-parameter save/load, matching
`.sseqjson`/`.ssmbjson`/`.ssdynjson`'s own per-plugin-family convention)
is the right save/load mechanism, not a forced-fit matrix export.

## GUI (built, v1 scope)

`gui/UIReverb.hpp/.cpp`, `DISTRHO_PLUGIN_HAS_UI 1`. Follows this
repository's established shape, built from ported widgets rather than
anything novel:

- **Room fields**: `Length`/`Width`/`Height`, one `ParamStripWidget`
  (ported verbatim), pushed via `setState()` (matching how the DSP side
  already treats them -- see `ReverbEngine.hpp`'s "Room-size changes"
  section). No wireframe/3D visualization yet -- a real, deliberate scope
  cut for this pass (see "Known limitations"), not a technical blocker.
- **Reverb fields**: `Input Gain`, `Decay Time`, `Damping`, `Diffusion`,
  `Dry/Wet`, `Output Trim` -- a second `ParamStripWidget`, same "select then edit"
  shape `UIMbCompressor`/`UIDynEq` already use. **`Decay Time` swaps to a
  read-only `Decay (synced)` field showing the live, tempo-derived value
  the instant Tempo Sync is turned on** (`kParamCurrentDecayTimeSec`),
  and swaps back to the ordinary editable field the instant it's turned
  off -- decays can be set traditionally, and once Tempo Sync is on, BPM
  changes are audible: the manual field is only ever shown/editable when it actually does
  something, and the synced field's own displayed number visibly tracks
  whatever the host's tempo (and the chosen note value) currently implies.
- **Tempo Sync controls**: three `SimpleButton`s -- `Tempo Sync: Off/On`,
  `Follow Host Tempo: Off/On`, and a click-to-cycle `Note: 1/4` (etc.)
  button cycling `TempoNoteValue` -- plus a **live diagnostic line**
  (`Host Tempo: 128.0 BPM (valid)` in green, or an amber "not reported,
  using 120 BPM fallback" otherwise) fed by the new
  `kParamHostTempoValid`/`kParamHostBpmDetected` output Parameters (see
  "Investigating 'no audible BPM effect'" below) -- added specifically so
  the tempo-sync mechanism's actual live state is visible, not only
  inferred by ear.
- **The EQ**: `EqGraphWidget` + `BandListWidget`, wired through
  `ssgui::IEqEditorHost` exactly like `UIEq`/`UIEqMono`/`UIDynEq` already
  do -- literally the same core again. Add/drag/remove/type/
  Q/harmonic-twin all work (thin wrappers over `EqModel.hpp`'s own,
  already-tested free functions, the same pattern `UIEq.cpp` itself
  uses). **Background in/out spectrum on the filter plot**:
  `SpectrumMode::Both`, two
  `SpectrumAnalyzer` instances (`PluginReverb::inputSpectrumAnalyzer_` fed
  pre-EQ -- the tank's own wet output -- `outputSpectrumAnalyzer_` fed
  post-EQ), packed 3-bands-per-Parameter (`getPackedTriple()`/
  `unpackTriple()`) and gated by a `kParamUiHeartbeat` so the analyzers'
  own FFT/windowing cost is genuinely not paid while the editor is
  closed -- the exact same mechanism the sibling EQ plugins already use.
- **Bypass/Reset** buttons.

**Deliberately not in this pass** (see "Known limitations" for the full,
explicit list): A/B compare, Load/Save/Export file browser, the EQ's
momentary right-click audition-hold (stubbed as a no-op, exactly matching
the sibling EQ Mono plugin's own precedent for "no audition DSP behind
this"), per-field tooltips (`TooltipOverlay` is ported and constructed,
just not wired to individual fields yet), a decay-curve/RT60-vs-frequency
plot, and any room-shape visualization beyond the plain numeric fields.
None of these are technically blocked -- they're a proportionate v1 scope
cut, following this same session's "verify what you can, don't write
hundreds of untested lines blind" discipline applied to GUI breadth
instead of DSP correctness.

### Why `EqModel` is a whole-model `State`, not a Parameter-mirror bank

The sibling `../statespace` EQ plugins mirror every `EqBand` field into a
bank of individual Parameters (`kParamBandFreqBase` etc.), specifically to
work around a real, previously-diagnosed Ardour behavior: a route *copy*
doesn't carry over a plain DPF `State`, only `Parameter` values, so a
whole-model-as-State design silently loses the EQ on copy in that specific
host/scenario. That fix was earned by actually hitting the bug in a real
host. This project has no real host available at all in the environment
it was built in -- reproducing, let alone fixing, a host-specific
persistence quirk isn't something that can be done responsibly here. Using
plain `State` for `eqModel_` (exactly like Room L/W/H already are) is the
simpler, lower-risk choice for this pass; if the same Ardour route-copy
issue turns out to affect this plugin too, `../statespace`'s own mirror-
parameter mechanism (`PluginEq.hpp`/`UIEq.cpp`) is the proven fix to port
over, not a new one to invent.

## How this was tested (and what still needs a real build)

**Update:** cmake and git turned out to be available after all, contrary
to the assumption the rest of this section was written under --
`./build_and_install.sh --test` has since been run for real:
full clean `cmake`/`cmake --build` of both `reverb` and `reverb_dense`
(LV2/VST3/AU/JACK), both regression suites (32/32 checks) passing, both
`.lv2` bundles installed into `~/.lv2` and the real standalone GUI
launched and screenshotted. The `g++`-only verification this section
describes below was the honest state of things *before* that, and is
kept as-is as the historical record of how this project's DSP/GUI code
was verified while cmake genuinely wasn't available -- not because it's
still the only option now.

Before that update, the development machine was believed to have **no
cmake and no git** available -- `./build_and_install.sh` could not be run. Given that constraint,
verification was scoped to everything a plain `g++` invocation against
real headers could prove -- which turns out to be almost the entire
codebase, DPF plugin class included:

**The DSP core** (`plugin/dsp/ReverbEngine.hpp`), no DPF dependency at all:

```sh
g++ -std=c++17 -O2 -Wall -Wextra -I plugin/dsp tests/reverb_regression.cpp -o reverb_regression -lpthread
./reverb_regression
```

Zero warnings, `ALL TESTS PASSED` (`[R1]`-`[R11]`, see above for what
each one checks and the actual measured numbers). `[R11]` (the room-size-
change thread-safety mechanism) was additionally built and run under
**ThreadSanitizer**:

```sh
g++ -std=c++17 -O1 -fsanitize=thread -I plugin/dsp tests/reverb_regression.cpp -o reverb_regression_tsan -lpthread
./reverb_regression_tsan
```

Clean, zero data races reported across ~4,455 concurrent `requestRoom()`
calls interleaved with `processBlock()` in one run -- a real, instrumented
check of the cross-thread design, not just an assertion that it didn't
crash this time.

**The DPF plugin class and every GUI/EQ file it and the UI depend on**
(`plugin_stereo/PluginReverb.hpp/.cpp`, `gui/UIReverb.hpp/.cpp`, and every
ported widget/DSP file listed in `CMakeLists.txt`'s `FILES_DSP`/`FILES_UI`),
against the real `dpf/` headers (at the time a plain copy of
`../statespace/dpf`; now a git submodule like the siblings') -- including
**real system OpenGL headers** (`/usr/include/GL/gl.h` was actually
present on this machine, so DGL/NanoVG's own `OpenGL.hpp` chain compiled
against the genuine thing, not a stand-in -- unlike an earlier session in
the sibling `../compressor` project, which needed a stub `GL/gl.h` because
this exact header wasn't available there):

```sh
g++ -std=c++17 -Wall -Wextra -fsyntax-only -I dpf/distrho -I dpf/distrho/extra -I plugin_stereo -I gui plugin_stereo/PluginReverb.cpp
g++ -std=c++17 -Wall -Wextra -O2 -c -I dpf/distrho -I dpf/distrho/extra -I plugin_stereo -I gui plugin_stereo/PluginReverb.cpp -o /tmp/PluginReverb.o

g++ -std=c++17 -Wall -Wextra -fsyntax-only -I dpf/dgl -I dpf/distrho -I dpf/distrho/extra -I plugin_stereo -I gui gui/UIReverb.cpp
g++ -std=c++17 -Wall -Wextra -O2 -c -I dpf/dgl -I dpf/distrho -I dpf/distrho/extra -I plugin_stereo -I gui gui/UIReverb.cpp -o /tmp/UIReverb.o

# Same two-step check for every other FILES_DSP/FILES_UI entry:
# plugin/dsp/EqModel.cpp, plugin/dsp/StabilityGuard.cpp,
# gui/ParamStripWidget.cpp, gui/SimpleButton.cpp, gui/TooltipOverlay.cpp,
# gui/eq/EqGraphWidget.cpp, gui/eq/BandListWidget.cpp
```

Every one of the above compiles clean -- zero warnings, syntax-check and
full `-c` compile both -- with the include paths matching exactly what
`CMakeLists.txt`'s own `target_include_directories(reverb PUBLIC
plugin_stereo gui)` declares (checked directly, not assumed: the same
files were re-compiled with *only* those two `-I` flags, plus DPF's own
`dgl`/`distrho`, to confirm the CMake file's own include setup is actually
sufficient). Cross-file linkage was checked with `nm -C` where a full
link isn't possible here (no DPF runtime libraries built without cmake):
`PluginReverb.o`'s undefined `ssfilter::eq::`/`ssfilter::StabilityGuard::`
symbols match, name for name, what `EqModel.o`/`StabilityGuard.o` actually
define.

This caught two real bugs before either ever reached a build:
`getParameterValue()` is `const`, and `ReverbEngine::consumeFaultFlag()`
(read-and-clear) mutates state, which `-fsyntax-only` correctly refused
(`discards qualifiers`) until `engine_` was marked `mutable`, matching
every sibling MimoMusic plugin's own identical convention for the same
reason (e.g. `PluginDynEq.hpp`'s own `mutable ssfilter::DynamicEqEngine
engine_;`); and the first `CMakeLists.txt` draft listed `EqModel.cpp`/
`StabilityGuard.cpp` only under `FILES_DSP`, which would have left the UI
target (a separate shared object/process in some plugin formats) missing
the `ssfilter::eq::` symbols `UIReverb.cpp` calls directly -- caught by
checking the sibling `../statespace` project's own `CMakeLists.txt`,
which duplicates exactly these two files under both `FILES_DSP` and
`FILES_UI` for the identical reason, and fixed the same way here.

**Not yet compiled or run at all**: `CMakeLists.txt` and
`build_and_install.sh` themselves (written against the proven-working
shape of the sibling projects' own files, and now cross-checked file-by-
file against what they actually declare, but never through an actual
`cmake` configure -- see each file's own header comment). This means
what's missing before a real plugin exists in a host is now down to
exactly three things: an actual `cmake` configure+build (to produce a
loadable `.lv2`/`.vst3` bundle at all), loading it in a real host, and
hearing it.

## Performance, measured against the generic core

`ReverbEngine::processStereo()`'s per-sample cost: `O(kNumLines)` = `O(8)`
for the Householder mix, plus 8 one-pole filter evaluations, plus 8
circular-buffer read/writes -- on the order of 30-40 floating-point ops
per sample pair, independent of room size. The generic `StateSpaceEngine`
at its own worst realistic case for this project (a 32-band EQ, order
128) costs `order^2` = 16384 multiply-adds per sample for the dense `A*x`
recursion alone (see `StateSpaceTypes.hpp`'s own `kMaxOrder` comment) --
several hundred times more per-sample work than this reverb's entire tank,
while the tank's own *effective* order (total delay-line samples) is
routinely in the thousands, i.e. representing dramatically more "state"
per sample of CPU spent than the dense core could, which is the whole
point of not forcing this into the dense/sparse-cascade machinery.

## Where this lives, and what to name it

**Its own top-level project, `reverb/`, sibling to `statespace/`,
`compressor/`, `meters/`, `amplifier/` -- not nested inside
`statespace/`.** This was tried the other way first (the first concept
doc and DSP core actually lived under
`statespace/plugin/dsp/ReverbEngine.hpp`) and moved out once it was clear
the state-space model doesn't fit.

**Why it doesn't fit, stated plainly**: every actual member of the
`statespace/` plugin family either *is* a `StateSpaceMatrices`/
`StateSpaceEngine` mapper (EQ, multiband compressor), or, in
`DynamicEqEngine`'s case, is a dedicated engine that still directly reuses
`EqModel`, the RBJ allpass mapper, and can still *export* a real (frozen,
resting-gain) `StateSpaceMatrices` snapshot via the same "Export to
State-Space..." button every sibling plugin has. This reverb's tank has no
such snapshot -- its effective order is in the thousands, and no
`StateSpaceMatrices` (`kMaxOrder=128`) could ever represent it, frozen or
otherwise, so it has no export path back into that project's own matrix
core at all. That's a materially deeper break from `statespace/`'s own
premise than `DynamicEqEngine` has: a genuinely different DSP
architecture gets its own project in this repository's own established
convention, not a forced-fit subfolder.

**What's still reused, deliberately, even though the project isn't
shared**: `plugin/dsp/`'s `EqModel`/`mappers::` conventions, the
`ParamStripWidget`/`EqGraphWidget`/`BandListWidget`/`TooltipOverlay` GUI
widgets, the `CMakeLists.txt` shape, and the `dsp_regression`-style
testing culture are all *ported* here (copied, matching source
byte-for-byte where nothing needs to change, like `TooltipOverlay` was
already ported verbatim from `compressor/` into `statespace/` in an
earlier session of that project) rather than shared via a live in-repo
dependency -- exactly this repository's own established pattern for reuse
*across* top-level projects, as opposed to reuse *within* one. A
`../statespace/dpf` copy (see "How this was tested") backs this project's
own `dpf/` for the identical reason.

## Phased build plan (Phase 2 onward)

1. **(Done)** Concept + `ReverbEngine.hpp` core + `tests/reverb_regression.cpp`,
   verified via `g++` directly.
2. **(Done)** Coefficient smoothing for Decay Time/Damping retargets
   (`OnePoleDamping`'s target/smoothed split, `ReverbEngine::
   setDecayTargets()`/`snapDecayTargets()`) and tempo-synced decay
   (`TempoNoteValue`, `t60SecFromTempo()`) -- see "Tempo-synced decay",
   verified `[R8]`-`[R9]`.
3. Pre-delay + early-reflection tap list wired into
   `ReverbEngine::processStereo()` itself (currently validated standalone
   only, `[R7]`); a Diffusion control (blend the Householder mix toward
   identity); a Width control (mid/side blend on the tank's own stereo
   output taps).
4. **(Done)** Thread-safe room-size changes: `ReverbEngine::requestRoom()`
   (callable from any thread -- publishes new dimensions via `std::atomic`,
   standard "publish via flag" pattern) + `processBlock()` (audio-thread
   entry point, polls the pending flag, applies the change via a
   ~8ms duck-to-silence-then-`prepare()`-swap-then-fade-in, matching
   `StateSpaceEngine`'s own order-changing-edit duck timing exactly).
   Built after realizing the concept's original plan -- call `prepare()`
   directly from wherever a room-size edit would arrive (a future
   `setState()`, which DPF documents as callable from "any non-realtime
   context", i.e. not guaranteed to be `run()`'s own thread) -- would race
   against `processBlock()`/`processStereo()` reading/writing the exact
   same buffers on the audio thread: the identical bug class
   `StateSpaceEngine`'s own double-buffer/atomic-index hand-off exists to
   prevent, just for a circular-buffer engine instead of a dense-matrix
   one. Verified `[R10]` (the duck really dips to measured near-silence,
   not just a state-flag change, and the engine really is running the new
   room's delay lengths afterward) and `[R11]` (a real concurrent
   `std::thread` calling `requestRoom()` continuously while `processBlock()`
   runs on the main thread, ~4,455 concurrent calls in one run, output
   stays finite throughout) -- `[R11]` was additionally run under
   **ThreadSanitizer** (`g++ -fsanitize=thread`), which instruments every
   memory access and would flag a genuine data race directly rather than
   relying on luck to hit one; clean, zero races reported.
5. `RoomReverbModel`-style coefficient-mapper layer -- not built; folded
   into step 6 below instead (`PluginReverb` itself owns `RoomDims`/
   decay/tempo state directly, no separate mapper struct turned out to be
   needed once the plugin class existed).
6. **(Done, unverified by cmake)** DPF plugin shell: own
   `DistrhoPluginInfo.h` (`DISTRHO_PLUGIN_HAS_UI 1`), `PluginReverb.hpp/cpp`
   with `getTimePosition()` wired to tempo-sync, `processBlock()`/
   `requestRoom()` wired to Parameters/State, three new diagnostic output
   Parameters (see "Investigating 'no audible BPM effect'"), plus
   `CMakeLists.txt` (LV2/VST3/JACK/AU targets, `reverb_regression` wired
   in) and `build_and_install.sh` -- both written against the sibling
   projects' own proven shapes but never run through an actual `cmake`
   configure (no cmake on this machine; the plugin class and every GUI/EQ
   file it depends on compile clean, see "How this was tested"). Needs
   cmake restored here, or another machine, for the
   remaining "does it actually build/load/run in a host" verification.
7. **(Done)** Post-tank `EqModel` integration (see "Parametric EQ
   integration" above): `PluginReverb::eqEngine_` (an ordinary
   `StateSpaceEngine`), rebuilt from `eqModel_` via `setState("eqModel", ...)`,
   applied to the wet signal in `run()` before the dry/wet mix.
8. **(Done)** GUI (`gui/UIReverb.hpp/.cpp`): room fields, the reverb's own
   fields (with `Decay Time` swapping to a live read-only readout under
   Tempo Sync), Tempo Sync controls + diagnostic BPM/decay readout,
   Bypass/Reset, and the ported `EqGraphWidget`/`BandListWidget` -- see
   "GUI (built, v1 scope)" above for the full list and what's
   deliberately still missing (A/B compare, Load/Save/Export, tooltips
   wired per-field, a decay-curve plot, room-shape visualization).
9. Mid-tank "object" EQ-in-the-damping-slot (see above), explicitly
   scoped as a follow-up once the base tank is shipped and heard, not
   part of this concept's core deliverable.

## Known limitations / open questions (first-draft scope, not fundamental)

- Fixed `kNumLines = 8`, not user-configurable -- an internal engine-
  quality knob (like `kMaxOrder` is for the generic core), not a musical
  parameter; raising it is a straightforward, low-risk change (more lines
  = denser/smoother tail at proportionally higher but still `O(N)` cost)
  if ever needed.
- Source/listener position inside the room is fixed at a hand-chosen
  off-center fraction of each dimension (see `firstOrderImageSources()`),
  not user-adjustable -- deliberately, to keep the exposed parameter
  surface at just Length/Width/Height, rather than growing into
  a full 3D source/mic positioning tool. A future "off-center amount"
  single knob is a natural, low-effort extension if this ever feels too
  fixed.
- The mean-free-path-based delay-length heuristic (see "The three
  pieces", point 1) is a principled *size* mapping, not a geometry-
  specific room simulation -- this is standard practice for FDN reverbs
  generally (no shipping algorithmic reverb literally simulates a
  specific room's modes for its late tail either), but is worth stating
  plainly rather than implying more physical precision than is actually
  there.
- Diffusion and Width, both named in the parameter table above, are not
  implemented in `ReverbEngine.hpp` yet (see "Phased build plan", step 2)
  -- the table states the intended mapping, not current behavior.
- No listening/perceptual validation yet at all -- everything verified so
  far is numerical (T60 accuracy, losslessness, stereo energy transfer,
  stability). "Fast and natural" was the explicit priority; fast is
  measured and argued above, natural still needs to actually be heard,
  which needs a real host -- unlike the earlier draft of this list, the
  plugin shell and GUI themselves are no longer the blocker, an actual
  `cmake` build and a host are.
- GUI v1 scope cuts, all deliberate (see "GUI (built, v1 scope)" for the
  full reasoning per item): no A/B compare, no Load/Save/Export file
  browser, no room-shape wireframe/3D visualization, no decay-curve/
  RT60-vs-frequency plot, no background spectrum on the EQ graph
  (`SpectrumMode::Off`, no analyzer wired), no per-field tooltips (the
  widget is ported and constructed, just not wired per-field), and the
  EQ's momentary right-click audition-hold is a no-op (matching the
  sibling EQ Mono plugin's own precedent for "no audition DSP behind
  this"). `eqModel_` is a whole-model `State`, not the sibling
  `../statespace` EQ plugins' own Parameter-mirror bank -- see "GUI"'s own
  subsection for the Ardour-route-copy-bug tradeoff this makes.
- The three new tempo-sync diagnostic Parameters
  (`kParamHostTempoValid`/`kParamHostBpmDetected`/`kParamCurrentDecayTimeSec`)
  make the mechanism's live state observable but do not, on their own,
  prove the feature is audible in a real host -- see "Investigating 'no
  audible BPM effect'" for exactly what they do and don't resolve.

## References

- [FDN Reverberation (Stanford CCRMA, Julius O. Smith)](https://ccrma.stanford.edu/~jos/pasp/FDN_Reverberation.html) --
  the canonical formal treatment this design follows most directly.
- [First-Order Delay Filter Design (Stanford CCRMA)](https://ccrma.stanford.edu/~jos/pasp/First_Order_Delay_Filter_Design.html) --
  the exact two-point DC/Nyquist damping-filter fit `OnePoleDamping::design()` implements.
- [Unitary Matrix Design for Diffuse Jot Reverberators (ResearchGate)](https://www.researchgate.net/publication/230757792_Unitary_Matrix_Design_for_Diffuse_Jot_Reverberators)
- [On Lossless Feedback Delay Networks (ResearchGate)](https://www.researchgate.net/publication/304469760_On_Lossless_Feedback_Delay_Networks)
- [Feedback Delay Networks in Artificial Reverberation and Reverberation Enhancement (ResearchGate)](https://www.researchgate.net/publication/322951473_Feedback_Delay_Networks_in_Artificial_Reverberation_and_Reverberation_Enhancement)
- [Implementation of a Hybrid Reverb Algorithm, Maximilian Steimel, Aalborg University](https://projekter.aau.dk/projekter/files/310743966/Thesis_Report___Hybrid_Reverb___Maximilian_Steimel_SMC10.pdf) --
  the early-reflections/FDN-tail hybrid architecture this design follows.
- [FDNTB: The Feedback Delay Network Toolbox (DAFx 2020)](https://dafx2020.mdw.ac.at/proceedings/papers/DAFx2020_paper_53.pdf)
- [Reverb Time Calculator -- BPM, Pre-Delay and Decay (ReverbFX)](https://reverbfx.com/tools/reverb-time-calculator) --
  the note-value-to-decay convention (quarter=short ambience, half=medium room,
  1 bar=long hall, 2 bars=epic tail) `TempoNoteValue`/`t60SecFromTempo()` implement.
- [BPM Reverb Calculator (Pre-Delay, Decay & Delay Times)](https://calculator.academy/bpm-reverb-calculator/)
