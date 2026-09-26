# MimoMusic Compressor (Mono / Stereo / Contour)

> This project is the dynamics-processing sibling of
> [`../statespace`](../statespace)
> -- same GUI toolkit, colors, click/double-click/scroll conventions, and
> build tooling, deliberately kept consistent across both -- but an
> entirely separate, real feed-forward compressor rather than anything
> built on that project's linear state-space core (which, by mathematical
> necessity, cannot represent level-dependent gain on its own -- see that
> project's `docs/EXTENDING.md`).

Four sibling plugins sharing one DSP core:

- **Compressor Mono** (1-in/1-out) -- `plugin_mono/`, `gui/UICompressorMono.*`
- **Compressor Stereo** (2-in/2-out) -- `plugin_stereo/`, `gui/UICompressorStereo.*`,
  with a **Linked / Dual-Mono / Mid-Side** stereo-processing mode switch.
- **Limiter** (2-in/2-out) -- `plugin_limiter/`, `gui/UILimiter.*` -- the same
  DSP core, cut down to a small, dedicated control set (Threshold, Release,
  Ceiling, Input/Output Gain) rather than the full compressor surface. See
  "Limiter" below.
- **Contour** (2-in/2-out) -- `plugin_contour/`, `gui/UIContour.*` -- a
  generic dynamics processor with a freely-drawn input/output curve
  (`plugin/dsp/CurveDsp.hpp`) instead of Ratio/Knee, capable of building
  ordinary compression, upward compression, limiting, or a gate/expander
  from the same drawn shape. See "Contour" below.

### Build and install

```sh
./build_and_install.sh            # build everything, install LV2 into ~/.lv2/
./build_and_install.sh --clean    # wipe build/ first (full rebuild)
./build_and_install.sh --test     # also run the regression suite
./build_and_install.sh --no-install
```

#### macOS

Not reliably cross-compilable from this Linux checkout -- see the
sibling `../statespace` project's README's identical
note for why. `./build_and_install_macos.sh`, run **on a Mac**, builds a
universal (Apple Silicon + Intel) LV2/VST3/AU set and installs into
`~/Library/Audio/Plug-Ins/{LV2,VST3,Components}` -- same flags as that
project's own macOS script (`--arch`, `--min-macos`, `--test`, ...), see
its own `--help`/header comment. Needs Xcode Command Line Tools
(`xcode-select --install`); written carefully but not tested end to end
(no Mac was available while writing it).

## What it is

A standard feed-forward dynamics compressor: **input gain, threshold,
ratio, attack, release, soft knee, output gain**, an optional **external
sidechain/key input** (Mono and Stereo plugins only -- see "Sidechain"
below) -- and one Ratio that can compress
either side of the threshold: a **Ratio: Down / Ratio: Up** swap button
picks which, mirroring the same downward-pulls-above-threshold /
upward-lifts-below-threshold shape either way rather than exposing two
independently-adjustable ratios that could both be dialed in *at the same
time* (an earlier version did exactly that). Built with the
[DISTRHO Plugin Framework](https://github.com/DISTRHO/DPF) (vendored as a
submodule), producing **LV2** and **VST3** plugin bundles plus a standalone
JACK app, from one codebase -- same as the sibling project.

### DSP design (see `plugin/dsp/CompressorDsp.hpp` for the full story)

- A **linear-domain** peak envelope follower (asymmetric attack/release
  one-pole) feeds a standard **soft-knee gain-computer curve**
  (Giannoulis/Massberg/Reiss, "Digital Dynamic Range Compressor Design").
  Smoothing happens in linear amplitude, converting to dB only once, at
  the point the curve is evaluated -- deliberately, because smoothing a
  dB-domain envelope touches -infinity at every zero crossing of an AC
  signal, which was a real, previously-found-and-fixed bug in the sibling
  project's own nonlinear gain stage. One multiply-add per sample for the
  envelope, one log10/pow pair for the dB round-trip -- fast, allocation-
  free, real-time safe.
- **Stereo modes** (Stereo plugin only): **Linked** (one shared detector,
  `max(|L|,|R|)`, identical gain to both channels -- the stereo image is
  never disturbed), **Dual-Mono** (two fully independent detectors/gains,
  one per channel), **Mid/Side** (decode to Mid/Side, compress each
  independently, re-encode -- classic "compress the center differently
  from the width" processing). A pure-algebra regression test
  (`tests/dsp_regression.cpp` test `[8]`) pins down that the Mid/Side
  transform itself is lossless.
- **Save As.../Load...**: exports/imports the current settings (all seven
  knobs, the Ratio Direction swap, Bypass, and -- for the
  stereo plugin -- Stereo Mode) as a plain text preset file (`.mcmp` mono /
  `.mcsp` stereo), independent of the host's own session/automation state.
- **Bargraph level meters** (Input/Output, L/R or Mid/Side for the stereo
  plugin) -- a fast RMS-ish fill plus a peak-hold line, with a clip LED
  that latches briefly whenever that channel's true peak reaches 0dBFS.
  Click a meter to clear its clip LED early.
- **The animated compressor graphic** (`gui/CompressorGraphWidget`):
  the input->output transfer curve (computed directly from the exact
  same `computeGainReductionDb()` the real-time engine uses), the knee
  region shaded, threshold marked, a live marker tracking the
  instantaneous input level's position on that curve, a vertical
  gain-reduction meter, and a scrolling gain-reduction history strip so
  an attack/release shape is actually visible over time. Two draggable
  handles sit on the curve -- Threshold, and a single Ratio handle that
  sits above the threshold while compressing Down or below it (in the
  quiet region) while compressing Up, following the swap button rather
  than two independently-draggable handles that could both be live at once.
- **The gain-reduction meter is bipolar and self-labeled**: since
  compressing Up can make it read a *boost*, not just a cut, a 0dB
  reference line splits the column (amber/red filling down for a cut,
  blue/green filling up for a boost), with its own dB scale printed
  alongside it and
  a peak-hold marker that snaps to the most extreme recent excursion in
  *either* direction and slowly recovers back toward 0dB. The live
  numeric value sits directly on top of the column it describes (moved
  there from the graph's own corner) so the number and the bar it belongs
  to are never in different places.

### Sidechain (Mono and Stereo compressor plugins)

An extra **external key input** -- `DISTRHO_PLUGIN_NUM_INPUTS` is 2 for
Compressor Mono (main + a mono key) and 4 for Compressor Stereo (main L/R +
a stereo key L/R), the added port(s) flagged `kAudioPortIsSidechain` so
Ardour (and any other LV2/VST3/CLAP host) shows them as a separate,
independently-routable auxiliary input rather than folding them into the
main audio bus. A **Sidechain: Internal / External** button and a
**Sidechain Gain** trim (its own independent dB control, separate from
Input Gain, which only affects the signal actually being compressed --
default 0dB, and only shown in the control strip while Sidechain is
actually on) are the only new controls -- flipping it swaps the level *detector*'s source
from the main signal to the key input; the signal being compressed is
always the main input, exactly like a hardware compressor's external-key
switch. On the Stereo plugin the key input is derived into a detector the
same way the main signal already is per Stereo Mode (`max(|L|,|R|)` in
Linked, per-channel in Dual-Mono, Mid/Side-decoded in Mid/Side), so
switching modes doesn't change how the sidechain itself behaves. The Limiter plugin doesn't have
this -- see its own section below for why it keeps a smaller surface.

### GUI conventions (shared with `../statespace`)

Dark theme, `NANOVG_DEJAVU_SANS_TTF` font, the same button look
(`gui/SimpleButton`, copied verbatim) and the same click-to-edit /
double-click-to-clear-first / Enter-or-Tab-commits / Escape-cancels /
scroll-wheel-nudges (hold **Alt** while scrolling for 10x finer control --
the same "1/10th the step" convention every scroll-to-nudge gesture in
the sibling `../statespace` project shares too) numeric-field convention (`gui/ParamStripWidget`,
generalized from that project's `NonlinearPanelWidget` field row so both
plugins here share one implementation instead of duplicating the state
machine per plugin). Fields also support click-and-drag: press and drag
up/down to increase/decrease the value (linear for most fields, log-scale
for the wide-range Attack/Release), with a small dead zone so a plain
click-and-release still falls through to double-click-to-edit. The
compressor graph itself has two draggable handles right on the curve --
one at the threshold (drag horizontally), and one Ratio handle that sits
above the slope (drag vertically) while compressing Down or below it, in
the quiet region, while compressing Up -- plus scroll-to-adjust-Knee
anywhere over it.

## Limiter

`plugin_limiter/`, `gui/UILimiter.*` -- reuses `plugin/dsp/CompressorDsp.hpp`
(the same `sscomp::CompressorVoice` every compressor plugin here uses) but
has its own, smaller GUI (`gui/LimiterGraphWidget.*`, not
`CompressorGraphWidget`). Parameter set: **Input Gain**, **Threshold**,
**Attack**, **Release**, **Lookahead**, **Knee**, **Ceiling**, **Output
Gain**, and Bypass. Ratio is the one thing still fixed internally (20:1,
`PluginLimiter.cpp`'s `kFixedRatio`) rather than exposed -- a limiter is
inherently near-brick-wall, and no dedicated limiter plugin compared
against (LSP's Limiter Stereo/Mono among them) exposes Ratio either, since
Threshold/Attack/Release/Lookahead/Knee already fully shape the curve.
Detection is always stereo-**Linked** (one shared detector for both
channels, so limiting never shifts the stereo image).
- **Lookahead** (ms, default 5) is the genuinely new piece: a small delay
  buffer lets the gain computer react to a transient *before* it reaches
  the output rather than only after, so a fast Attack (kept at or below
  Lookahead for this to actually work) has time to finish ramping down
  before the delayed audio catches up to it -- most of what makes a
  dedicated limiter sound cleaner than a plain compressor with a hard clip
  bolted on. Adds real output latency, correctly reported to the host
  (`DISTRHO_PLUGIN_WANT_LATENCY`) so it doesn't silently shift the timeline.
- **Ceiling** (dB, default 0dB) is still the hard safety net on top of all
  of that, applied to the true output sample *after* everything else
  (input gain, gain reduction, output gain): even with Lookahead giving
  the gain computer a head start, an extreme or fast-enough transient can
  still momentarily slip past the intended reduction, so Ceiling
  guarantees the output never crosses it regardless, at the cost of a
  possible instant of hard clipping on such a transient rather than a
  perfect zero-overshoot catch.

**Algorithm** (Classic / True Peak, a button next to Bypass, backed by a
State not a Parameter -- see `PluginLimiter.hpp`'s own top comment): an
empirical A/B against LSP's own installed Limiter (same settings, same
input, real audio through both) found Classic (the envelope-follower
engine above) can't guarantee a hard peak ceiling on a fast transient no
matter how steep its ratio -- a 2ms spike 9dB over 0dBFS measured 5.5dB
past LSP's own -6dB peak, and raising the internal ratio 20:1 to 200:1
barely moved that number, ruling out "the ratio isn't steep enough" and
pointing at something structural. **True Peak** (`plugin/dsp/
TruePeakLimiterDsp.hpp`) is a second, genuinely different gain computer
built specifically to close that gap: a real sample-accurate lookahead
peak-hold limiter (a classic monotonic-deque sliding-window minimum, the
same architecture most dedicated brick-wall limiters, LSP's included,
actually use) that computes, for each sample, the worst (i.e. minimum)
gain needed across its whole upcoming lookahead window before that sample
ever reaches the output -- a hard, provable guarantee (see that file's own
comment for the proof) rather than an approximation, unlike an envelope
follower's inherent lag. Re-running the same LSP comparison after building
it: peak values now match LSP's to within rounding across every test
window, including the exact spike that motivated it (both settle at
precisely -6.00dB). Attack and Knee play no role in this mode (the
reaction is "as fast as the lookahead window allows", by construction, and
the guarantee only holds for a hard cut, not a soft-knee blend) -- their
strip fields grow a "(unused)" label suffix while True Peak is selected,
and the graph curve draws with a knee of exactly 0 to match what's
actually running. Switching Algorithm resets both gain computers' own
internal state (a plain instant switch, not a duck-to-silence crossfade --
see `PluginLimiter.cpp`'s `setState()` comment for why that's an
acceptable trade-off for a rare, explicit action). Classic stays the
unchanged default; this is an additive second path, not a replacement.

The Limiter's own graph has two draggable things: a **Threshold** handle
on the curve (drag horizontally, same gesture as the compressor graphs'
own Threshold handle) and a **Ceiling** handle drawn as a horizontal
dashed line across the curve (drag vertically) -- Ceiling is an output-
axis-only value, not a point on the input/output curve, so a full-width
line reads better than a point. The curve itself reflects the real,
current Knee value (not a fixed constant). The gain-reduction meter
column/scale/peak-hold/history strip is the same look and ballistic as the
compressor graphs' own, just one-directional (a limiter only ever cuts)
and single-channel (no Dual-Mono/Mid-Side second column, since detection
is always Linked).

## Contour

`plugin_contour/`, `gui/UIContour.*` -- a generic compressor where the
input/output mapping is an arbitrary drawn curve instead of a knee
formula, so it can build special behaviours: upward and downward
compression, but also gates. Shares Compressor Stereo's own Linked/Dual-Mono/Mid-Side stereo mode,
external sidechain input, and Load/Save/A-B/Undo-Redo machinery
unchanged -- the one real difference is what replaces
Ratio and Knee: an arbitrary, user-drawn transfer curve
(`plugin/dsp/CurveDsp.hpp`'s `sscomp::TransferCurve`) plus an independent
Threshold and an Above/Below trigger condition, instead of a fixed
ratio/knee formula.

### How Contour works (the whole idea)

1. **The curve is an input -> output mapping for your sound.** Draw it once; the sound's own level (x) is looked up on the curve and comes out
   at the curve's height (y). A straight diagonal changes nothing.
2. **The trigger decides whether the sound takes the curve or stays on the diagonal.** The trigger is a signal (the main input, or the
   external sidechain) that can be amplified (*Trigger Gain*) and filtered (*Trigger Filter*); it is "met" while its level is above /
   below (*Trigger: Above / Below*) the *Threshold*. **On Trigger: Curve** = while the trigger is met the sound goes on the curve,
   otherwise on the diagonal; **On Trigger: Diagonal** = the other way round (a gate that opens).
3. **Attack and Release make the switch smooth.** Attack = how fast the sound moves to its new state when the trigger starts being met,
   Release = how fast it moves back when the trigger ends (0 = instant). *Lookahead* (an option) lets the switch happen before the sound
   that triggered it arrives.

The trigger's gain, filter and source only change WHEN the switch flips -- never where on the curve the sound sits. So the dots in the graph
(each dot = the sound's level on x, its level after the effect on y) are on the drawn curve while the trigger says "curve", on the diagonal
while it says "diagonal", and travel between the two while Attack/Release switch. (Earlier versions read the curve at the trigger's
level, which made a lowered Trigger Gain or a narrow filter slide the sound along the curve; that is gone. A side effect: a ducking
compressor whose reduction should depend on the sidechain's level is now built by drawing the curve, not by the key's loudness.)

- **The curve**: an ordered list of (input dB, output dB) "pillar
  points", up to 16 interior ones plus two fixed endpoints. The two
  endpoints' own X position never moves (drawn as squares, not circles,
  so that's visible at a glance) -- only their height (Y) is ever
  adjustable.
  The window is **-80 .. +6 dB** on both axes, so a gate can close all the way down (a flat line at -80 takes any sound to -80). Curves saved
  with the earlier -60 .. +6 window are migrated on load with identical behaviour (below its old left end a curve always held that end's gain: the
  new left end at -80 carries the same gain, and the old end stays as an ordinary node; a plain straight line stays 2 points).
- **Tool button** (top of the graph, click to cycle) decides what a drag on
  the graph does: **Draw Node** -- a click on empty space places a node (a
  plain click = corner node with straight lines to its neighbours; click-
  and-drag = smooth node, the drag pulls out its two Bezier *support
  points*, Inkscape-style), while a click-and-drag ON an existing node
  moves just that node and a double-click on it removes it; **Draw Rising Node** / **Draw Falling Node** -- the same,
  but the node is tagged so it only shapes the curve while the input level
  is rising / falling (hysteresis: two different paths up and down);
  **Edit / Move** -- nothing is created, and this is where multi-node work
  lives: drag a node to move it, drag a
  node's support point to reshape the Bezier, drag on empty space to
  select several nodes (rubber-band), Shift+click to extend the selection,
  drag any selected node to move the whole group, double-click a node to
  remove it, **right-click a node for a menu**: *Normal / Rising only /
  Falling only* (current tag ticked), *Link N nodes* (shown when 2+ nodes
  are selected), *Unlink* (shown when a selected node is linked) and *Delete*
  -- all applied to the whole selection. **Linked** nodes (drawn with a
  colored ring and a faint connecting line) always move together, as if they
  were permanently selected together: selecting one selects the group, and
  dragging one moves the group, in either tool family. The **threshold can
  be linked too** (right-click it, or include it in the selection -- a
  drag-window only takes it when the round handle at the bottom of its line is
  inside the window, not merely when the line is crossed): it then follows its
  group left/right only. A linked node can still be removed (double-click
  removes just that node and it drops out of the group; a group left with
  fewer than two members dissolves). The menu acts on the right-clicked node
  alone when the selection is just its own group, so **Unlink** on a linked
  node unlinks only that node (*Unlink whole group* is offered next to it);
  with an explicit multi-selection it acts on all selected. **Link options**: *Link (X + Y)*,
  *Link X only* (partners follow the dragged node's horizontal moves only) and *Link Y only*
  (vertical only); the node you drag or edit always moves freely, the axis limits what the
  others get (shown as arrowheads on the link ring). The threshold can join X and X+Y links but
  not Y-only ones. The menu also offers ***Snap to Diagonal*** (any node,
  including an endpoint): pins it onto the unity (no-change) line, and from
  then on it only slides along that line -- one degree of freedom, exactly
  like the threshold (which has no Y of its own either) -- shown with a
  short tick through the node at the line's own angle. *Release from
  Diagonal* undoes it. A snapped node is otherwise a completely ordinary
  node: it can still join or lead its own link group like any other (a
  snapped node in an X-only group just carries the rest of the group along
  by its own diagonal position). Editing a linked node's
  In/Out in the **node list** moves its whole group (and a linked threshold, X
  only) by the same amount. The Delete key follows the same targeting: a plain
  click on a linked node then Delete removes that node only; a window /
  Shift-click selection is deleted as a whole. A selection window can start
  anywhere in the graph widget, margins included. **The support point you hold is always the white
  one**: the pair is mirror-symmetric (a tangent), so grabbing the dim dot just relabels it, and dx
  is signed -- a node at the right edge can have its white dot to the left of it. **Double-click a
  support point** to reset that node's handle to 0/0 (a corner again; for an
  end node, no slope). In either tool, pressing a
  **support point** grabs it in preference to placing a new node. **Delete**
  (or Backspace) removes every selected node in either tool family (the two
  end nodes are never removed). Hovering a node, support point or the
  threshold shows a tooltip with its values and what the gestures do. While
  drawing, a ghost node with its dB coordinates follows the cursor (red if
  the curve would refuse it).
- **Heat trail** (the fading dots): the sound's level on x, its level after the effect on y (see *How Contour works*); green dots = the
  trigger is met. The old *Dots: In -> Out / Detector* switch is gone -- the dots always show the real sound.
- **Node list** (right of the graph, toggle with *Node List*): one row per
  node with its Rising/Falling tag, In dB, Out dB and the two components
  of its support point (a node's support point is only limited by the
  nearest neighbours it can actually conflict with -- a Falling-only node
  is not held back by an adjacent Rising-only one) -- click a value to type it, drag it up/down like a
  fader (0.25 dB per pixel) or scroll over it to nudge (Alt = 10x finer), so "lower this point by exactly 1 dB" is one keystroke
  instead of a precisely vertical mouse drag. Selecting a row highlights
  the node on the graph and vice versa; *+ Add node* inserts one in the
  widest gap, exactly on the current curve. In Linked mode there's one list;
  in Dual-Mono/Mid-Side there are two, stacked vertically in the same
  column (A's/left's on top, B's/right's below) rather than side by side,
  which would squeeze the two graphs further -- *Node List* shows or hides
  both together.
- **Per-node interpolation, not a curve-wide mode**: a node with a zero-
  length support point is a corner (straight lines); a node with a support
  point is a cubic Bezier through that point. (The old curve-wide
  Linear/Spline button is gone.)
- **Threshold is deliberately independent of the curve's own points** --
  a separate, freely-draggable vertical line with no handle sitting on the
  curve itself (unlike the sibling Compressor's threshold, which always
  sits exactly on its own continuous knee construction). A colored
  background split behind the curve shows which side is actually "live"
  at a glance: warm amber on the side the trigger condition currently
  engages on, dim/neutral on the other.
- **Trigger condition** (a toolbar button, Above/Below): **Above** is the
  ordinary compressor/limiter shape -- the curve engages once the level
  *exceeds* Threshold, identity below it. **Below** is a gate/expander
  shape instead -- the curve engages once the level *drops under*
  Threshold, identity above it. Whichever side is engaged is where the
  curve's own shape actually reaches the audio; the other side is always a
  flat, untouched identity regardless of what's drawn there -- opening or
  closing depends only on the drawn curve: one mechanism, no separate
  up/down mode switch needed at all. A
  curve that lifts quiet material with Above is upward compression; the
  same mechanism with a curve that cuts loud material is ordinary downward
  compression; a curve that dives toward heavy attenuation as level drops,
  with Below, is a gate closing -- all three (plus limiting: a curve that
  caps output near the top with Above) from the identical engine.
- **On Trigger: Curve / Diagonal** (toolbar button; in Dual Mono / Mid-Side one per curve): which of the two states the sound is in while the trigger is
  met (see *How Contour works*). **Curve**: trigger met -> the drawn curve, otherwise the diagonal. **Diagonal**: trigger met -> the diagonal (a gate
  that opens), otherwise the drawn curve. **Attack** = how fast the sound moves to its new state when the trigger starts being met, **Release** = how
  fast it moves back when the trigger ends -- the same for both settings. Saved with presets/A-B/undo; older presets load as Curve.
- **Reset Curve** snaps back to the safe, do-nothing 2-point identity
  line without needing to manually
  drag both endpoints and remove every interior point by hand.
- **Trigger Input** (button *Trigger Input...*, plus the *Trigger: Main
  Input / Sidechain In* source switch): the signal that drives the curve
  can be the plugin's own input or the external Sidechain input, and
  **Trigger Gain** (-24..+60 dB: a narrow band-pass leaves little energy, so it needs a lot), **Lookahead** (0..20 ms: the audio is delayed by
  this much -- reported to the host as latency -- while the trigger is read un-delayed, so the curve or the diagonal is already in place when
  the sound that triggered it arrives, even with Attack 0; the release direction is held for the same time so the tail is not cut; about
  twice the Attack or more), the Highpass/Lowpass **Trigger Filter** and **Listen** (soft-limited, so a big Trigger Gain can't blast the output)
  work on either -- e.g. a gate that only opens on a chosen frequency band
  of the material passing through it. The filter/gain only shape what the
  detector sees, never the audio you hear (except while Listen is on).
  (Formerly "Sidechain Options", which only worked on the external input;
  Parameter symbols are unchanged, only their display names.)
- The curve (and Threshold, Trigger Condition) fully participate
  in **A/B compare, Undo/Redo, and Save As.../Load...** (`.mccp` preset
  files) exactly like every other control -- dragging a single curve
  point is as undoable as nudging any knob. The curve's own point list is
  a State (`sscomp::curveToString()`'s small hand-rolled text encoding,
  not JSON -- matching this project's established dependency-free
  preference), not a Parameter, since it's a variable-length list rather
  than one scalar.
- The GR meter column(s)/history are the same
  look and ballistic as the other compressor plugins' own graphs, just
  spanning a wider bipolar range (a curve can, by construction, produce a
  much larger gain swing than a ratio/knee shape ever could).

See `plugin/dsp/CurveDsp.hpp`'s own header comment for the full DSP design
(including why gain-change smoothing needed a second one-pole stage this
plugin's `CurveVoice` has and the other plugins' `CompressorVoice` never
needed), and `tests/dsp_regression.cpp`'s tests `[12]`-`[19]` for the
curve math's own correctness coverage (monotonicity, endpoint pinning,
trigger gating, serialization round-trip).

### Contour Mono

`contour_mono` (LV2/VST3/AU/JACK, like the others) is the mono version of Contour: one main input plus a mono sidechain
input, one output, always the single-curve ("Linked") mode -- no Stereo Mode / Link / Listen buttons, one meter channel,
no L/R filter link. It is **not a copy**: it is the *same source files* built a second time.

- `plugin_contour_mono/DistrhoPluginInfo.h` is the only mono-only file: it defines `CONTOUR_MONO 1`, the plugin identity
  (name, URI `.../contour_mono`, id `CCtM`) and the port count (2 in, 1 out).
- The shared code (`plugin_contour/PluginContour.*`, `gui/UIContour.*`, all widgets, `plugin/dsp/*`) contains the few
  mono differences as `if (kContourMono)` (constant declared at the top of `PluginContour.hpp`). Everything else --
  curve engine, gate/open-close, lookahead, trigger input, node list, presets -- is literally the same code, so **a change
  to Contour reaches Contour Mono with the next build**, and only a change to the mono spots (`grep kContourMono`) or to
  the mono info header needs a thought for both.
- Presets (`.mccp`) are interchangeable; a stereo preset's stereo mode / second curve are simply ignored by the mono plugin.
- In `CMakeLists.txt` its include path is `plugin_contour_mono` first and `plugin_contour` not at all -- that order is what
  makes the compiler pick the mono info header. Tests: `tests/contour_mono_test.cpp` (manual build, see its header).

## Project layout

```
plugin/dsp/CompressorDsp.hpp   framework-independent DSP core (header-only),
                                shared by both plugins: dB<->linear, the
                                soft-knee gain computer, envelope follower,
                                loudness/peak meters, volume-compensation,
                                CompressorVoice, StereoMode
plugin_mono/                   Mono plugin's DPF plugin (DSP) side
plugin_stereo/                 Stereo plugin's DPF plugin (DSP) side
plugin/dsp/TruePeakLimiterDsp.hpp  the Limiter's second gain computer (True
                                Peak algorithm) -- Limiter-only, not shared
                                by the Mono/Stereo compressor plugins, so it
                                lives in its own file rather than growing
                                CompressorDsp.hpp
plugin_limiter/                 Limiter plugin's DPF plugin (DSP) side -- same
                                DSP core as plugin_stereo/ (sscomp::CompressorVoice),
                                a much smaller parameter set, plus a hard
                                output-Ceiling clamp, plus (as of the
                                True Peak addition) a second, switchable
                                gain-computer algorithm
plugin/dsp/CurveDsp.hpp        the Contour plugin's own DSP core (built on
                                CompressorDsp.hpp's EnvelopeFollower/LoudnessMeter/
                                PeakMeter/CompensationGain/StereoMode): TransferCurve
                                (the drawn curve itself, linear or monotone-spline),
                                computeCurveGainDb(), CurveVoice, plus
                                curveToString()/curveFromString() serialization
plugin_contour/                 Contour plugin's DPF plugin (DSP) side
gui/                            shared DPF UI (GUI) side, NanoVG-drawn widgets:
                                SimpleButton, ParamStripWidget, LevelMeterWidget,
                                CompressorGraphWidget (Mono/Stereo), LimiterGraphWidget
                                (Limiter's own, smaller graph), ContourGraphWidget
                                (Contour's own curve-editing graph), NodeListWidget
                                (Contour's numeric node list), plus
                                UICompressorMono/Stereo, UILimiter and UIContour
build_and_install.sh           builds everything, installs the LV2 bundles to ~/.lv2/
tests/dsp_regression.cpp       fast pure-math correctness tests, no DPF/GUI dependency
tests/param_strip_sim.cpp       headless drive of ParamStripWidget: value per pixel of drag (incl. from exactly 0)
tests/contour_widget_sim.cpp    headless simulation of ContourGraphWidget's mouse handling (real widget
                                logic + tests/gui_stub/NanoVG.hpp, drawing = no-ops) on real curves
tools/testbench/contour_preset_probe.cpp
                                loads a Contour .mccp preset and runs sines through the
                                real PluginContour::run(); plot_contour_probe.py draws
                                waveform / gain / spectrum PNGs (manual build, see header)
tests/contour_trigger_test.cpp  drives the real PluginContour::run() to check Trigger Gain/
                                Filter on the internal signal (manual build, see its header)
tests/contour_mono_test.cpp     the same real run() built as Contour Mono (2 in / 1 out, sidechain, bypass, lookahead)
tests/stereo_heartbeat_test.cpp Compressor Stereo: spectrum analysers only run while the UI heartbeat arrives
plugin_contour_mono/            DistrhoPluginInfo.h of Contour Mono (CONTOUR_MONO) -- the rest is shared with plugin_contour/
tools/testbench/                audio_testbench.cpp -- real synthetic audio through
                                the real per-sample DSP chain, writes WAV files
docs/ARCHITECTURE.md           how the DSP core and GUI actually work
docs/TESTING.md                 the two verification tools, in detail
dpf/                            DISTRHO Plugin Framework (git submodule)
```

## Testing

See [docs/TESTING.md](docs/TESTING.md) for the full picture.

```sh
./build/bin/dsp_regression    # fast, pure-math checks -- gain computer, envelope
                               # follower, loudness meter, compensation gain,
                               # Mid/Side algebra, end-to-end voice consistency,
                               # and the True Peak gain computer's own hard
                               # guarantee (output never exceeds threshold)
./build/bin/audio_testbench   # real synthetic audio through the real per-sample
                               # chain -- steady-state accuracy, attack/release
                               # timing, volume compensation, stereo modes; also
                               # writes WAV files to tools/testbench/output/ to
                               # listen to (run from the project root)
```

Neither has any DPF/GUI dependency -- both build automatically alongside
the plugins.

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
