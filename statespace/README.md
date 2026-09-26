# Matrix Filter (+ Parametric EQ, EQ Mono, Multiband Compressor, Dynamic EQ)

Six sibling audio plugins sharing one DSP core:

- **Matrix Filter** (2-in/2-out) -- a fully generic, directly editable
  **discrete-time state-space filter**, described below.
- **Parametric EQ** (2-in/2-out) -- an unbounded-band parametric EQ (click
  the graph to add a band, drag to shape it), built as a coefficient mapper
  on the *same* core -- see "Parametric EQ" below. Its "Export to
  State-Space..." button writes an ordinary `.ssjson` the base plugin opens
  directly, every resulting `A`/`B`/`C`/`D` coefficient ready for
  hand-editing.
- **Parametric EQ Mono** (1-in/1-out) -- the same EQ, same GUI, natively
  single-channel for mono tracks/busses. Not the stereo plugin with a
  channel disabled: it assembles into a genuinely mono state-space system
  needing **half the states per band**, so its `O(order²)` per-sample
  recursion measures **2.8×-4.4× faster** than the stereo plugin for the
  same band list. See "Parametric EQ Mono" below.
- **Multiband Compressor** (2-in/2-out) -- a user-configurable
  Linkwitz-Riley crossover bank with a threshold/ratio/attack/release/
  makeup compressor per band, drag the crossover splits directly on the
  graph -- see "Multiband Compressor" below.
- **Dynamic EQ** (2-in/2-out) -- FabFilter Pro-Q-style bell
  bands whose own boost/cut pulls back automatically when that frequency
  region gets loud, then relaxes back -- a multiband compressor's per-band
  dynamics, targeted at an EQ bell shape instead of a crossover slice. Its
  live audio path is a dedicated engine, not a raw state-space matrix, and
  an Audition button previews whichever band is selected by ear -- see
  "Dynamic EQ" below. (The De-esser plugin that used to live alongside
  this one has been retired -- it was one fixed band of this exact same
  construction, and Dynamic EQ's own Audition now covers the one feature
  it had that this plugin didn't. Called "Listen" until this project's
  own rename -- see "Parametric EQ"'s own Audition bullet below for why.)

### Build and install

```
./build_and_install.sh            # build everything, install LV2 into ~/.lv2/
./build_and_install.sh --clean    # wipe build/ first (full rebuild)
./build_and_install.sh --test     # also run the regression suite
./build_and_install.sh --no-install
```

#### macOS

This project isn't reliably cross-compilable for macOS from a Linux
checkout -- DPF's macOS backends (Cocoa-based GUI code, plus AU/Audio
Unit) need the real macOS SDK and toolchain to compile and, just as
importantly, an actual Mac to verify the result on (cross-compiling via
osxcross is possible for anyone who wants to pursue it, but isn't
supported here). `./build_and_install_macos.sh`, run **on a Mac**,
builds a universal (Apple Silicon + Intel) LV2/VST3/AU set and installs
into `~/Library/Audio/Plug-Ins/{LV2,VST3,Components}`:

```
./build_and_install_macos.sh                 # build + install (universal binary)
./build_and_install_macos.sh --arch arm64    # single-arch, faster while iterating
./build_and_install_macos.sh --test
```

Needs Xcode Command Line Tools (`xcode-select --install`); see the
script's own header for the full flag list. Written carefully but not
tested end to end (no Mac was available while writing it) -- if something
doesn't work for your Xcode/macOS version, the CMake/DPF build underneath
is the same, already-verified one the Linux script uses, so the fix is
almost certainly in the macOS-specific flags this script layers on top.

### Conventions shared across every plugin here

- **Hover a numeric field/graph node and scroll to nudge its value** --
  every such control in every plugin in this project (this file's own
  "Enable/Solo per band" bullet below, every graph's node-scroll-to-
  adjust-Q, the Multiband Compressor's Ratio-scroll, Matrix Filter's own
  cell/knot scroll) shares one rule: **hold Alt while scrolling for 10x
  finer control** -- the same wheel motion moves the value a tenth as
  far, for the exact spot you're trying to land on precisely. Applied to
  every scroll-to-nudge gesture project-wide.

## Matrix Filter

A stereo (2-in/2-out) audio plugin that exposes a fully generic, directly
editable **discrete-time state-space filter**:

```
x[n+1] = A x[n] + B u[n]
y[n]   = C x[n] + D u[n]
```

`u[n]` is the stereo input, `y[n]` the stereo output, and `x[n]` an internal
state vector whose size ("order") you choose. Because every entry of `A`,
`B`, `C` and `D` is independently editable, this one structure can represent
essentially any causal, **linear, time-invariant** stereo process that fits
in the chosen order: EQ bands, crossovers, resonators, cross-channel
coupling (mid/side-style matrices), all-pass sections, etc. It's meant as a
**building block** -- see [docs/EXTENDING.md](docs/EXTENDING.md) for how to
build parametric EQ and other higher-level linear processors on top of it
without touching the DSP core. (A fixed-coefficient state-space model can
never represent level-dependent gain like compression -- that's a
mathematical fact of linear time-invariant systems, not a missing feature.
The model also carries an optional, genuinely state-dependent gain stage
for exactly this case -- see "Trainable dynamics" below and
`docs/EXTENDING.md`'s "Why compression needs state-dependent coefficients".)

Built with the [DISTRHO Plugin Framework](https://github.com/DISTRHO/DPF)
(vendored as a submodule), producing **LV2** and **VST3** plugin bundles plus
a standalone JACK app from one codebase.

### Status

Working draft, actively used in Ardour. The DSP core, stability guard,
transfer-function math and preset serialization are covered by two
verification tools (see `Testing` below): a fast pure-math regression
suite, and an audio testbench that runs every factory preset through the
real engine against synthetic WAV signals, cross-checked against the
frequency-domain transfer function. Both currently pass on all 15 factory
presets.

### Features

- **Generic MIMO state-space core**, order configurable from 1 to 128
  states, every `A`/`B`/`C`/`D` coefficient directly addressable.
- **Cannot be driven unstable**: every edit is passed through a stability
  guard that proves (or, failing that, forces) the state matrix's spectral
  radius stays below 1, so the recursive filter cannot diverge. A real-time
  NaN/Inf safety net and output ceiling back this up in case of unforeseen
  numerical edge cases. See `docs/ARCHITECTURE.md` for exactly how.
- **No clicks/pops from live coefficient edits**: a same-order edit (any
  ordinary coefficient tweak or EQ-band drag) is smoothed toward the new
  values over a few milliseconds rather than switched to instantly, so
  even a fast edit lands as a quick, smooth glide instead of an audible
  discontinuity -- fixed after exactly that was reported ("like plugging/
  unplugging a speaker cable" when dragging fast). An order-changing edit
  (adding/removing/enabling/soloing an EQ band) can't be smoothed the same
  way -- there's no meaningful "in between" for two differently-sized
  state vectors -- so that briefly ducks the output to silence, switches,
  and fades back in (~16ms total) instead, since that *was* also reported
  clicking once ordinary dragging was fixed. See `docs/ARCHITECTURE.md`'s
  "Real-time engine" section.
- **GUI matrix editor** for A/B/C/D -- click a cell to edit it (pre-filled
  with its current value), double-click to clear it first, Enter/Tab
  commits, Escape cancels, arrow keys commit-and-jump to the neighboring
  cell, scroll wheel nudges a value -- with live order (matrix size)
  resizing. Cells are colored as a diverging heatmap (warm = positive, cool
  = negative, scaled to the matrix's own peak magnitude) so structure in
  bigger matrices is visible at a glance instead of a wall of numbers.
- **Bode plot** of the full 2x2 transfer function, recomputed as you edit.
  Click a legend swatch to show/hide that channel pair's curve (the two
  direct paths L->L/R->R, shown by default, and the two cross-channel paths
  L->R/R->L, hidden by default since most models have no cross-coupling);
  the magnitude (y) axis autoscales to whichever curves are currently
  visible, and the visibility choice is remembered (persisted through the
  host's session state) across closing/reopening the plugin. A LOG/LIN
  toggle (top-right of the plot) switches the frequency axis.
- **"Normalize 0dB" button**: rescales the output (C/D only, so stability
  is untouched) so the model's peak passband gain is exactly unity/0 dB,
  regardless of filter shape.
- **Save/Load** presets as plain JSON (`.ssjson`) via native file dialogs
  that remember the last folder used (persisted through the host's session
  state, so it survives closing/reopening the plugin), plus automatic
  session/project recall of the whole model through the host (LV2/VST3
  state).
- **Coefficient-mapper pattern + worked examples**: RBJ "Audio EQ Cookbook"
  biquad formulas (lowpass/highpass/peaking/shelves/notch/bandpass/allpass)
  realized as state-space blocks, plus a series-cascade helper for
  multi-band chains (`makeStereoBiquadChain()`), demonstrating how to build
  a musical parameter layer (e.g. a parametric EQ UI) on top of the raw
  matrix engine. 15 factory presets in `presets/factory/` exercise all of
  this, from single filters to multi-band chains.
- **Trainable dynamics (state-dependent gain stage)**: `A`/`B` stay fixed
  (and stability-guarded) forever, but an optional per-model
  `NonlinearSchedule` lets `C`/`D` scale every sample by a gain that's a
  function of a smoothed input-level envelope -- a mathematically
  legitimate, standard control-theory construction (state-dependent
  coefficients / gain scheduling), not a bolted-on post-processor. A GUI
  panel (bottom of the window) has an Enable toggle, editable
  attack/release fields, a threshold/ratio/makeup "Apply Curve" button for
  classic-compressor-style one-click shaping, and a click/drag/scroll
  graph for freehand-sculpting the 25-knot gain-vs-level curve directly
  (double-click a knot to reset it to 0dB/flat).
  Or **learn it from audio** instead: the `fit_gain_curve` CLI tool takes
  an example input WAV and a desired output WAV and fits attack/release
  plus the whole curve via least squares (its output loads normally, and
  is then just as editable in the GUI panel as a hand-drawn curve). See
  `docs/ARCHITECTURE.md`'s "NonlinearSchedule" section and
  `docs/EXTENDING.md`'s "Why compression needs state-dependent
  coefficients" for the full story (including a real frequency-dependence
  bug found and fixed along the way).
- **Multiband compression (DSP core + CLI trainer; no GUI/plugin yet)**:
  the same state-dependent-gain mechanism above, generalized to N
  independent bands. A Linkwitz-Riley crossover (`mappers::
  makeStereoCrossoverBank()`, up to `kMaxBands=6` bands, itself an
  ordinary LTI parallel filter bank -- flat/inaudible whenever every
  band's own gain is 0dB) splits the signal into bands; each band gets its
  *own* envelope follower (tracking that band's own filtered signal, not
  the broadband input) and its own `NonlinearSchedule`/gain curve, summed
  back into the output. `A`/`B` are, as always, completely unaffected.
  `fit_multiband_compressor` learns all N curves at once from one (input,
  reference-compressed-target) audio pair: `fit_multiband_compressor
  "<crossover_hz_csv>" input.wav target.wav out.ssjson` -- e.g.
  `fit_multiband_compressor "150,2000" mix.wav mix_compressed.wav
  out.ssjson"` for a 3-band split. The output is an ordinary `.ssjson`
  (its A/B/C/D *is* the crossover bank itself), loadable and hand-editable
  in this same matrix grid exactly like any other preset. See
  `docs/EXTENDING.md`'s "Multiband compression" section for the mechanism
  and a real phase-alignment bug found while building the trainer.

## Parametric EQ

A parametric EQ, built as a worked example of the coefficient-mapper
pattern above (`plugin/dsp/EqModel.hpp`) rather than a mode of the base
plugin -- see `docs/ARCHITECTURE.md`'s "EqModel / EQ plugin" section for
why that split. Inspired by (not a clone of) FabFilter Pro-Q's
interaction model: a big interactive graph is the primary surface, not a
bank of knobs.

### Features

- **Unbounded band count** -- click anywhere on the graph to add a Bell
  band right there; drag it to reshape (frequency + gain), scroll to
  adjust Q, with a small readout ("1.20kHz  +3.5dB  Q 1.10") tracking the
  node above your finger the whole time. In practice capped only by
  `kMaxOrder` (128 states / 4 per band = 32 bands, comfortably past
  FabFilter Pro-Q's advertised 24), not by any smaller, arbitrary limit --
  "the more complex it gets, the more coefficients it costs" is the
  actual, honest tradeoff, not a marketing-driven band cap.
- **Three equivalent ways to delete a band**: double-click its node,
  select it and press Delete/Backspace, or right-click it for a small
  context menu (Delete, **Duplicate** -- an exact copy inserted right next
  to it and selected, same type/freq/gain/Q/slope, ready to drag apart --
  plus every filter type as a direct pick).
- **Create Harmonic Twin** (same right-click menu): adds another band at
  a harmonic frequency of the one you clicked -- x2 (Octave), x3, x4,
  ÷2, or a typed **Custom Factor...** -- copying its type/gain/Q, and
  links the two together *in frequency only*: drag, scroll-nudge, or
  type a new frequency on **either** one and the other follows,
  preserving the ratio. **Unlink** (same menu, shown once a band is
  linked) detaches it again without moving it. Chain further twins onto
  an already-linked band to build a whole harmonic series (root, x2, x3,
  x4, ...) -- each member's own ratio is tracked independently, so
  deleting or unlinking one out of the middle never disturbs how the
  others move relative to each other. No "relink" is offered once
  unlinked -- deliberately.
- **Right-click a node**: a plain click (press and release without
  dragging) opens the usual context menu (Delete/Duplicate/type picks/
  Create Harmonic Twin/Unlink). Press and drag instead (past a small dead
  zone, same click-vs-drag split every drag gesture in this project uses)
  and the menu never opens at all -- the band becomes draggable and
  momentarily Auditions for as long as the button stays down, restoring
  whatever Audition was actually doing before on release. See the
  **Audition** bullet below for what that preview actually does and why
  it replaced an earlier middle-click-hold-to-solo gesture.
- **Band Shelf**: a flat-topped boost or cut between two corners, set as
  a centre frequency (FREQ) and a bandwidth in octaves (BW), with one Q
  and one gain -- built as a High Shelf at the low corner, a Low Shelf at
  the high corner and a flat -G over the whole band, so it returns to
  exactly 0dB outside the corners. On the graph its node sits at the
  centre; the handles at each corner drag the bandwidth, and scrolling
  over it changes the bandwidth instead of Q.
- **All 8 RBJ filter types per band**, plus Band Shelf: Bell, Low Shelf,
  High Shelf, Low Pass, High Pass, Notch, Band Pass, All Pass -- selected from the band
  list below the graph or the graph's own right-click menu (a type has
  no discoverable, precise place on a small draggable node otherwise).
  A Low/High Pass band also gets a **steepness** choice (12/24/36/48
  dB/octave -- 1 to 4 cascaded copies of the same biquad at your own Q,
  not a fixed Butterworth override), in that same right-click menu *and*
  in the band list's own Gain column: Low/High Pass bands have no
  meaningful gain (see `eqBandTypeHasGain()`), so that field slot shows
  "24dB/o" etc. instead and cycles 12->24->36->48->12 on a plain click,
  the same "fixed set of options, click to cycle" convention the Type
  button next to it already uses.
- **Live combined + per-band curves**, redrawn on every single drag event
  at no perceptible cost regardless of band count -- computed directly
  from each band's closed-form biquad response (a basic LTI identity:
  series cascade = product of transfer functions) rather than the
  generic dense-matrix solve the base plugin's Bode plot uses, which
  would be far too slow to redraw per-frame at high band counts. See
  `docs/ARCHITECTURE.md` for the actual benchmarked numbers behind that
  choice.
- **Adjustable gain-axis range**: a "±Ndb" label on the graph cycles
  through 3/6/12/30 dB on a click, or **press and drag it vertically for
  continuous, precise control** anywhere from 1 to 40dB -- tight for
  careful, sensitive-to-drag moves, wide for aggressive swings that still
  need to stay reachable.
- **Hovering a node shows its exact freq/gain/Q**, the same small readout
  that appears while dragging one -- discoverable without needing to grab
  it first.
- **Live pre-EQ and/or post-EQ spectrum** (real FFT, 384 log-spaced bands
  for a dense, smooth-looking curve each -- every band still sums *actual*
  FFT bin power within its own edges, never interpolates between bins, so
  more bands is genuinely more real display resolution, not a smoother
  curve fitted through the same underlying numbers), drawn behind the curves, with
  its own scale labeled on the graph's right edge (in a neutral color
  distinct from the EQ curve's left-edge gain axis, since the two mean
  different things). Its vertical range is a **fixed range by default**
  (a click-to-toggle "Fixed Range"/"Autoscale" label, top-right of the
  graph); switching to Autoscale refits the range to whichever trace(s)
  are showing (the *union*, so "Both" gets one shared, meaningful axis),
  smoothly drifting toward the new fit rather than snapping to it every
  frame -- autoscaling was the original always-on behavior, but a
  per-frame-snapping axis under real program material read as visually
  disturbing, so it's now opt-in and gentler when on. A
  "Spectrum Source" field cycles Off -> In -> Out -> Both, **defaulting to
  Both**: two entirely
  independent analyzers run under the hood (one fed the signal before the
  EQ, one after), and only the one(s) actually selected are fed at all --
  switching to a single side, or Off, means the other's FFT cost genuinely
  isn't paid, not just hidden from view. In "Both", the input trace (warm
  orange) draws as a thin outline so it never muddies the output trace
  (cool blue, filled, the more important of the two) where they overlap,
  with a small color-key legend confirming which is which. An adjustable
  smoothing time constant ("Spectrum Smoothing (ms)") controls how
  fast/twitchy vs. slow/buttery it looks, and a "Spectrum Resolution"
  control cycles the FFT size (2048/4096/8192/16384) -- bigger trades a
  bit more buffering latency for finer low-frequency detail (an
  improvement that's inherently most
  visible at the low end, since FFT bins are linearly spaced while the
  display bands are log-spaced).
- **Enable/Solo per band**, a precise numeric Freq/Gain/Q field per band
  (click-to-edit, click-and-drag up/down, hover-and-scroll-to-nudge --
  Freq at a gentle 2% per notch, Q at 8%, Gain in 0.5dB steps, and
  **Alt+scroll for 10x finer control on any of them**, same
  convention every scroll-to-nudge gesture in this whole project family
  now shares -- see "Conventions shared across every plugin here" above --
  and double-click-to-clear-
  then-type, same as the graph's own nodes) in a scrollable band list,
  each column now labeled by a small title row above it (TYPE/EN/SOLO/
  FREQ/BW/GAIN/Q), and an output trim control (click to type, drag or
  scroll to change).
- **Audition**: preview the selected band's own isolated frequency
  region (a genuine bandpass tap at that band's freq/Q, scaled by its
  own gain, independent of its actual Type/shape) instead of the real
  output, for finding or tuning a problem frequency by ear -- same idea,
  same button, as the sibling Dynamic EQ plugin's own Audition. Also
  reachable as a momentary preview: press-and-drag (not a plain click,
  see the right-click bullet above) a node to audition it while the
  button stays down, release to stop, restoring whatever Audition was
  doing before. Previously a middle-click-and-hold gesture that toggled
  per-band Solo instead (middle-click was uncomfortable, and Audition is
  the more useful thing to preview with a quick hold in the first place).
  A Band Shelf previews the region between its two corners. Named
  "Audition", not the originally-shipped "Listen": "Listen" invited the
  expectation of hearing what the band's own EQ curve does to the mix
  (gain-dependent, like a solo/bypass A/B), when what it's always
  actually done is isolate the raw frequency region the band's freq/Q
  names. If the previewed band is part of a **Harmonic Twin** group, every
  other member of that group previews *with* it (summed together), not
  just the one band clicked -- previewing only the clicked half of a
  linked pair would be misleading when the whole point of linking them is
  that they're meant to be heard, and thought about, together. Ordinary
  per-band **Solo** (the band list's "S" toggle) has the identical rule:
  soloing one member of a harmonic group also unmutes every other member
  of that same group.
- **Per-band channel routing (Both/Left Only/Right Only)**: each band's
  own "Ch" toggle in the band list (click to cycle, same idiom as the
  Type button) restricts it to filtering just one stereo channel instead
  of both. A Left/Right-only band leaves the *other* channel completely untouched
  by it (no phase shift, no gain -- not merely "unheard"), while every
  other Both band still applies to both channels normally; the two
  channels only ever end up genuinely, independently filtered once at
  least one band asks for that. Visible three ways at once: the band
  list's own "L"/"R" toggle color (blue/red), a matching **ring drawn
  around that band's node** on the graph (always visible, not just on
  hover/selection), and the graph's own transfer
  curve, which splits into two colored curves (same blue/red) the instant
  any band is Left/Right-only -- a single curve would otherwise silently
  misrepresent whichever channel it doesn't reach. The background
  spectrum does the same: the moment channel routing actually differs,
  the single combined "Post-EQ" trace is replaced by two separate ones,
  **Left in blue, Right in red**, each fed from that
  channel's own real output signal; everything reverts to the ordinary
  single-trace/single-curve view the instant every band goes back to
  Both, with no separate toggle needed for any of it. **Soloing a Left/
  Right-only band silences the channel it never reaches**, rather than
  leaving it as a raw, unfiltered passthrough of the input: without this,
  soloing a channel-restricted band correctly
  isolated its own filtering, but the untouched channel would just play
  the source unprocessed, defeating the point of soloing it in the first
  place. Only while a solo is actually active -- an ordinary Left/Right-
  only band with nothing soloed at all still legitimately leaves the
  other channel as passthrough, since that's the routing feature working
  as intended.
- **Export to State-Space...**: writes the assembled, stability-enforced
  result as an ordinary `.ssjson` -- open it in Matrix Filter's own
  "Load..." to see and hand-edit every resulting coefficient. No bespoke
  interchange format: it's the exact same file format and
  `Serialization` code both plugins share.
- **Save/Load** its own band-list presets (`.sseqjson`) the same way the
  base plugin saves/loads `.ssjson` -- native file dialogs, remembered
  last folder, automatic session/project recall through the host.
- Cannot be driven unstable, for the same reason the base plugin can't:
  every assembled result passes through the same `StabilityGuard`.

## Parametric EQ Mono

The 1-in/1-out variant of Parametric EQ, for projects where only mono is
used. **The GUI and every EQ feature listed above are identical** -- the
graph, band list, spectrum, presets and all interactions are literally the
same widget code, because a band list (`ssfilter::EqModel`) has no channel
count of its own. Only what it assembles into differs. The one exception
is per-band channel routing: with only one channel to begin with, "Left
Only"/"Right Only" have nothing to mean here, so the band list's own "Ch"
column simply doesn't appear on this plugin -- every band is implicitly
Both, same as always.

The point is DSP load. This is *not* the stereo plugin with one channel
switched off: the stereo EQ realizes each band as two independent biquads
(one per channel, 4 states/band), while this one realizes a single SISO
chain (2 states/band). Since the engine's dominant per-sample cost is the
`O(order²)` `A`-matrix recursion, halving the order quarters that cost.
Measured end-to-end, mono vs. stereo engine on identical band lists:

| Bands | Stereo order | Mono order | Speedup |
|-------|--------------|------------|---------|
| 4     | 16           | 8          | 3.1×    |
| 8     | 32           | 16         | 2.8×    |
| 16    | 64           | 32         | 4.2×    |
| 24    | 96           | 48         | 3.9×    |
| 32    | 128          | 64         | 4.4×    |

(Approaching the theoretical 4× as the band count grows and the fixed
per-sample overhead stops dominating.)

Correctness is pinned by `dsp_regression` test `[14]`, which checks that
the mono engine's output is **sample-for-sample identical** to the stereo
engine's on one channel for the same band list -- they're the same LTI
system, so anything else would mean the mono realization is wrong -- while
using exactly half the states to produce it.

Its "Export to State-Space..." still writes a *stereo* `.ssjson` (both
channels identical), since Matrix Filter -- the only plugin that can
import one -- is stereo-only. That's an inspection/hand-editing path; the
mono audio engine itself always runs the mono-assembled system.

## Multiband Compressor

A GUI on top of the multiband compression mechanism described above
(`plugin/dsp/MbCompressorModel.hpp`, `MultibandSchedule`) -- select a band,
shape its threshold/ratio/attack/release/makeup, drag its crossover split
directly on the frequency graph. Reuses the sibling EQ plugin's own
Load/Save/Export/spectrum conventions, and ports the click-to-edit/drag-
fader/scroll-nudge numeric fields from the sibling `../compressor`
project. Its own Spectrum Source defaults to **Both** (moved off the
shared `kDefaultSpectrumMode` default -- now leaving EQ Mono as the only one of the four plugins with
this feature still on it -- same "own default" precedent the Parametric/
Dynamic EQ plugins' own identical fields already set).

### Features

- **User-configurable band count**: click "+ Band" (splits the currently
  widest band at its own geometric-mean frequency) or double-click inside
  a band's own region on the graph to split it there; double-click a
  crossover handle, or select a band and press Delete/Backspace, to merge
  it back into a neighbor. Capped at `kMaxBands` (6) -- refuses rather
  than silently dropping a band past that.
- **Crossover Slope**: 12/24/36/48 dB/oct (`MbCompressorModel::lrOrder`),
  click-to-cycle in the global fields row -- the same four choices as the
  EQ plugin's own Low/High Pass "Slope" control, applied uniformly to
  every crossover split in the band-splitting network (Linkwitz-Riley
  crossovers, flat/inaudible at 0dB regardless of the chosen order).
- **Drag crossover splits directly on the graph** -- an absolute
  position->frequency mapping (like dragging an EQ node), clamped so
  adjacent splits can never cross or collapse to zero band width, with a
  live "1.20kHz" readout near the split while dragging (or just hovering)
  it. Each band's own region is shaded in its own color (the same
  8-color palette the EQ plugin cycles band colors from) and labeled with
  its threshold/ratio at a glance.
- **Live gain-reduction/makeup meter for the selected band**: a bipolar
  bar (0dB in the middle, filling down for a cut, up for a boost now that
  Up Ratio can lift a quiet passage) shown right on the graph, with a dB
  scale down its side and the live number and the band's own configured
  Makeup value both printed large and labeled ("GAIN"/"MAKEUP") rather
  than as small corner text. The live number is also smoothed in time
  (not just redrawn less often) before it's ever printed or used to
  position the fill -- the raw reading can change several dB within a
  handful of milliseconds during an attack/release, which reads as
  flickering, unreadable digits otherwise.
- **A global Output Gain**, applied uniformly to both channels after
  every band's own compression -- the field strip's "Output Gain" (labeled
  "Output Trim" internally/in JSON presets, unchanged for compatibility).
- **Click a band's region to select it**, then shape it with a
  click-to-edit / click-and-drag-vertical-fader / scroll-to-nudge field
  strip (Threshold/Ratio/Up Ratio/Attack/Release/Makeup) -- **Up Ratio**
  is genuine upward compression, mirroring Ratio below the threshold
  instead of above it (lift quiet material at the same time loud material
  is pulled down, from one shared threshold); `1.0` means no upward effect
  (the default). The click-drag fader
  interaction is ported from the sibling `../compressor` project's
  `ParamStripWidget`, this project's first widget with that interaction
  (the base plugin's matrix cells and gain-curve knots deliberately left
  it out; see that widget's own header comment for why it's safe here).
  Scroll over a band's region (no click needed) adjusts its Ratio
  multiplicatively, the same "scroll the curve to shape it" convention the
  `../compressor` project's own graph uses for Knee.
- **Enable per band**: a disabled band's own audio still plays, just
  uncompressed (dynamics bypassed).
- **Solo**: a single spotlight, not a per-band flag (unlike the EQ
  plugin's own multi-band solo, which suits its all-bands-visible list) --
  since only one band's controls are shown/edited at a time here, Solo
  always follows whichever band is currently selected. Selecting a
  different band while Solo is on moves the spotlight with it, so it's
  never possible for Solo to be pointed at a band you're not looking at.
  The soloed band's own region is labeled "(solo)" directly on the graph.
- **Live pre-/post-compression spectrum**, the exact same mechanism
  (fixed-by-default/toggleable-autoscale range, dual-trace visual
  hierarchy, Off/In/Out/Both source control, adjustable smoothing and
  FFT-resolution) as the EQ plugin's own -- see that section above for the
  full description.
- **Export to State-Space...**: writes the assembled crossover bank (with
  every band's gain curve baked in) as an ordinary `.ssjson` -- open it in
  Matrix Filter's own "Load..." to see and hand-edit every resulting
  coefficient, exactly like the EQ plugin's identical button.
- **Save/Load** its own preset format (`.ssmbjson`) -- crossover
  frequencies + per-band settings, the same convention as the EQ plugin's
  `.sseqjson`. A preset produced by the `fit_multiband_compressor` CLI
  trainer (see above) is a `.ssjson`, not a `.ssmbjson` -- it's already the
  fully-assembled result, loadable directly in Matrix Filter or hand-
  edited there; there is no in-GUI "train from audio" workflow yet (see
  `docs/EXTENDING.md`'s "Multiband compressor GUI" section for what's
  deferred and why).
- Cannot be driven unstable, for the same reason every plugin in this
  project can't: the assembled crossover bank passes through the same
  `StabilityGuard` before ever reaching the engine.

## Dynamic EQ

FabFilter Pro-Q-style bell bands that also carry their own threshold/
ratio/range/attack/release -- a multiband compressor's per-band dynamics,
targeted at an EQ bell shape instead of a crossover slice, which was the
literal ask this plugin was built to answer. Unlike every plugin above,
its live audio path is not a raw state-space matrix at all -- see
`docs/EXTENDING.md`'s "Dynamic EQ" section for why an overlapping,
continuously gain-varying bell needed a genuinely new mechanism (a
classical Regalia-Mitra fixed-allpass construction) and its own dedicated
real-time engine, `plugin/dsp/DynamicEqEngine.hpp`.

### Features

- **Drag a node to place a band** (frequency + resting gain, exactly like
  the parametric EQ's own graph, with the identical live "1.20kHz +3.5dB
  Q 1.10" readout above the node while dragging *or just hovering* it),
  scroll to adjust its bandwidth (Q), double-click to remove -- every band
  is the same bell shape, so there's no type menu to reach for. A freshly
  placed band starts with Threshold at 0dB and Range at 0dB -- inert,
  behaving exactly like a plain static bell until you actually dial
  dynamics in, rather than already reacting at some default
  threshold/range you haven't touched yet. A band
  with dynamics on always shows a second "Live +N.NdB" line below that
  readout -- the band's real, currently-running gain (the same reading the
  moving curve below is drawn from), permanently visible rather than only
  appearing once it happens to differ from the resting gain, so the box
  doesn't change shape as that gap opens and closes. A band with dynamics
  *off* has no such line -- it can never move, so the resting gain shown
  above is already the whole story.
- **Threshold/Ratio/Up Ratio/Range/Attack/Release per band**, edited via a
  click-to-edit / click-and-drag-fader / scroll-to-nudge field strip
  re-targeted at whichever band is selected (the same `ParamStripWidget`
  interaction the multiband compressor uses) -- `Ratio` pulls gain down
  above the threshold (peak limiting, the original purpose), `Up Ratio`
  mirrors it below the threshold (lifting quiet material instead), and
  both share the same `Range` ceiling on how far gain is allowed to move
  in whichever direction is active, so a band can never be pulled further
  than intended even on a very loud (or very quiet) passage.
- **A shaded "dynamic range" region and a live, moving combined curve**
  on the graph: the shaded area shows the frequency-shaped envelope a
  band's gain can move within (its response curve at the extremes its own
  Ratio/Up Ratio can actually reach -- `gainDb - rangeDb` and/or
  `gainDb + rangeDb`); the brighter curve on top is drawn from each band's
  *actual, currently-running* gain and visibly moves/wiggles as program
  material crosses each band's threshold.
- **Enable/Solo/Dynamics per band**: Enable works exactly like the EQ
  plugin's own; Solo is a single spotlight that follows the selected band
  (same convention as the Multiband Compressor's own -- see that
  section's own bullet for the full reasoning), not a per-band flag;
  Dynamics toggles a band between "moving" and a plain static bell at its
  resting gain, without needing to zero out Range to get the same effect.
- **Audition**, per selected band: previews that band's isolated detector
  output (a genuine, magnitude-shaped bandpass tap -- the same one the
  band's own dynamics are actually measuring level from), scaled by
  whatever gain reduction it's currently applying -- for tuning
  Frequency/Q/Threshold/Range/Attack/Release entirely by ear, the same
  idea (and formula) the now-retired De-esser plugin's own "Listen" used
  (renamed here -- see the Parametric EQ's own Audition bullet for why),
  generalized here to "preview whichever band is selected" since this
  plugin can hold several. Grayed out with nothing selected; switching
  bands while auditioning follows the selection automatically. Turning
  Audition on or off (or switching which band it previews) **ducks
  through a brief silence instead of switching instantly** -- the exact
  same technique this project's own order-change edits already use (see
  `docs/ARCHITECTURE.md`): swapping the output between "the real filtered
  signal" and "an isolated detector tap" is a jump between two unrelated
  waveforms no amount of internal filter-state continuity can smooth over,
  so it clicked just as audibly as an order change did, and is fixed the
  same way.
- **Per-band channel routing (Both/Left Only/Right Only)**: a "Ch:
  Both"/"Ch: Left"/"Ch: Right" toolbar button, click-to-cycle, re-targeted
  at whichever band is selected -- same feature and same visible-three-
  ways treatment as the Parametric EQ's own identical bullet above (band
  ring on the graph, the transfer curve splitting into blue/red, the
  spectrum's single Post-EQ trace splitting into separate Left/Right
  ones), just reached through this plugin's own toolbar-button-per-band-
  property convention (see this section's own Enable/Solo/Dynamics
  bullet) rather than a band-list column, since this plugin has no
  BandListWidget of its own. A Left/Right-only band's detector reads
  *only* that one channel's own level too, not the combined max(L,R) a
  Both band still uses -- so its dynamics react solely to what's actually
  happening on the channel it's assigned to. **Soloing a Left/Right-only
  band silences the channel it never reaches** -- same behavior, same
  reasoning, as the Parametric EQ's own identical bullet above (see that
  one for the full "why").
- **Live pre-/post-EQ spectrum**, the identical mechanism as the EQ
  plugin's own -- both default to Both for the spectrum source itself
  (unlike EQ Mono, which still defaults to Output-only; the Multiband
  Compressor now also defaults to Both -- see that section's own note);
  this plugin's own default is additionally the
  largest FFT preset, 16384 (not the shared 8192).
- **Export to State-Space...**: writes a *frozen, resting-gain* snapshot
  (an ordinary RBJ bell cascade, the same math the parametric EQ plugin
  uses) as an `.ssjson` -- a deliberate simplification, since a moving
  filter has no fixed-coefficient representation to freeze into at all;
  see `docs/ARCHITECTURE.md` for the full reasoning.
- **Save/Load** its own preset format (`.ssdynjson`), the same convention
  as every sibling plugin's own extension.
- The real-time engine's own filter shapes are fixed by construction
  (only ever a level-dependent scalar combination on top of them, never a
  moving pole) -- there is no arbitrary-matrix editing path here to
  destabilize in the first place, unlike the base plugin.

## Building

```sh
./build_and_install.sh    # does everything below, plus installs into ~/.lv2/
```

or by hand:

```sh
git submodule update --init --recursive   # first time only (pulls DPF + its own pugl submodule)
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Outputs land in `build/bin/`, for *each* of `matrix_filter`,
`parametric_eq`, `parametric_eq_mono`, `multiband_compressor`,
`dynamic_eq` and `dynamic_eq2`:
- `<name>.lv2/` -- LV2 bundle (copy to `~/.lv2/`)
- `<name>.vst3/` -- VST3 bundle (copy to `~/.vst3/`)
- `<name>` -- standalone JACK app, useful for quick manual testing without
  a full DAW

## Testing

See [docs/TESTING.md](docs/TESTING.md) for the full picture, including the
real bug (a stability-guard issue that silently neutered a notch filter)
that motivated building both of these:

```sh
./build/bin/dsp_regression   # fast, pure-math checks
./build/bin/audio_testbench  # real audio through the real engine (works from any directory)
./build/bin/fit_gain_curve <base_preset.ssjson> <input.wav> <target_output.wav> <output_preset.ssjson>
./build/bin/fit_multiband_compressor "<crossover_hz_csv>" <input.wav> <target_output.wav> <output_preset.ssjson>
```

All four are built automatically alongside the plugin and have no DPF/GUI
dependency.

## Project layout

```
plugin/               Matrix Filter's DPF plugin (DSP) side
  dsp/                 framework-independent core, shared by ALL SIX plugins:
                        matrices, stability guard, transfer function, JSON,
                        coefficient mappers, nonlinear gain-curve trainer(s),
                        EqModel (the EQ's band-list -> matrix assembler),
                        MbCompressorModel (the multiband compressor's own,
                        crossover-bank-based assembler), DynamicEqModel +
                        DynamicEqEngine (the dynamic EQ's Regalia-Mitra
                        band-list model and its own dedicated real-time
                        engine -- not a mapper feeding StateSpaceEngine, see
                        DynamicEqModel.hpp's file comment), and the Mono*
                        trio (MonoStateSpaceTypes/Engine, MonoStabilityGuard)
                        the mono EQ runs on
  PluginStateSpace.*    DPF Plugin subclass (parameters, state, run())
  DistrhoPluginInfo.h   plugin identity / build-time capability flags
plugin_eq/             Parametric EQ's DPF plugin (DSP) side -- own identity,
                        reuses plugin/dsp/ entirely, adds nothing of its own
                        except PluginEq.* (parameters, state, run())
plugin_eq_mono/        Parametric EQ Mono's DPF plugin (DSP) side -- same
                        again, 1-in/1-out, running the mono engine
plugin_mb_compressor/  Multiband Compressor's DPF plugin (DSP)
                        side -- own identity, same shape as plugin_eq/,
                        assembles via MbCompressorModel instead of EqModel
plugin_dyn_eq/         Dynamic EQ's DPF plugin (DSP) side -- own
                        identity, same shape again, drives DynamicEqEngine
                        directly instead of StateSpaceEngine
gui/                   Matrix Filter's DPF UI (GUI) side, NanoVG-drawn
                        widgets, plus ParamStripWidget.* (ported from the
                        sibling ../compressor project) shared with gui/mb/
                        and gui/dyneq/
gui/eq/                Shared EQ GUI: EqGraphWidget + BandListWidget (used
                        unchanged by BOTH EQ plugins), plus UIEq.* (stereo)
                        and UIEqMono.* (mono) hosting them
gui/mb/                Multiband compressor GUI: MultibandGraphWidget +
                        UIMbCompressor.*, MbEditorHost.hpp (the EqEditorHost
                        analogue)
gui/dyneq/             Dynamic EQ GUI: DynEqGraphWidget (EqGraphWidget's own
                        node-drag interaction, plus a shaded dynamic-range
                        region and a live moving combined curve) +
                        UIDynEq.*, DynEqEditorHost.hpp
build_and_install.sh   builds everything, installs the LV2 bundles to ~/.lv2/
tests/                 dsp_regression.cpp -- fast pure-math correctness tests
tools/testbench/       audio_testbench.cpp -- real audio through the real engine
tools/trainer/         fit_gain_curve.cpp -- learns a gain curve from example audio
                        fit_multiband_compressor.cpp -- same idea, N independent
                        per-band curves behind a Linkwitz-Riley crossover
docs/
  ARCHITECTURE.md       how the stability guard, engine, nonlinear stage, EQ
                        plugin and GUIs actually work
  EXTENDING.md          how to build EQ/etc. on top (the EQ plugin is a real,
                        worked example of this), and the state-dependent-
                        coefficient mechanism dynamics uses
  TESTING.md            the verification tools, and the bugs that motivated them
presets/factory/       15 example .ssjson presets (built with the biquad mapper, plus one trained compressor)
dpf/                   DISTRHO Plugin Framework (git submodule)
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

Before any real distribution: `DISTRHO_PLUGIN_URI` in *each* plugin's
`DistrhoPluginInfo.h` must be a URI you actually control (LV2 identity
requirement), and the brand/unique IDs should be yours.
