# Reverb: design

Two stereo reverbs -- **Reverb Dense** (16-line feedback delay network)
and **Reverb Plate** (Dattorro plate) -- sharing parameters, pre-delay,
the EQ on the reverb signal and one editor.

## Design goals

- A stereo reverb whose signal crosses from one channel to the other and
  back.
- The usual controls (decay, damping, room size), with the room given as
  length, width and height rather than a preset list.
- Fast, natural-sounding, real-time-safe DSP first.
- A parametric EQ to shape the reverb.
- Reuse the MimoMusic EQ, spectrum and widget code where it fits.

## Why a dedicated engine, not a state-space mapper

The `statespace` plugins turn musical parameters into a
`StateSpaceMatrices` of at most 128 states. A room's character comes from
delay lines tens to hundreds of milliseconds long -- 100 ms at 48 kHz is
4800 samples. As literal state-space order that is far beyond 128, and
the generic engine's O(order²) recursion would cost millions of
operations per sample.

A feedback delay network (FDN) *is* a state-space system nonetheless:
concatenate all delay-line contents into one state vector, and almost all
of `A` is a pure shift (each sample moves one slot along its line). The
only real coupling is an N x N mixing matrix at the line ends, N being the
number of lines (16), not samples. Implemented as circular buffers, the
system has an effective order in the thousands but costs O(N) per sample,
independent of room size. All geometry work (room → delay lengths →
damping coefficients) happens when parameters change, never per sample.

## Background: hybrid reverb and the light-transport analogy

The design follows the standard literature (Schroeder 1962, Moorer 1979,
Jot's FDN 1991/92, Stanford CCRMA):

- **Per-line damping filters**, not a single feedback gain, give a
  frequency-dependent decay (high frequencies die faster, as in real
  rooms).
- **Lossless (orthogonal) feedback matrices** make the network stable by
  construction: the mix can't add energy, only the damping removes it.
  Householder is used: O(N) for any N (sum once, subtract `2*sum/N`).
- **Early reflections vs. diffuse tail** mirror how renderers treat light:
  the image-source method for the first distinct echoes is the acoustic
  version of mirroring a light source for a specular reflection; the dense
  diffuse tail corresponds to diffuse global illumination, where only the
  statistics of energy matter. First-order image sources (one per wall)
  are implemented (`firstOrderImageSources()`); higher orders grow
  combinatorially for little perceptual gain.

## Reverb Dense (`plugin/dsp/ReverbEngineDense.hpp`)

Built on the shared pieces in `plugin/dsp/ReverbEngine.hpp`.

1. **Room → delay lengths** (`designDelayLengths16()`): the room's mean
   free path (`4 * volume / surface`) sets a base travel time; the 16
   lines are spread geometrically around it and rounded to **prime**
   sample counts, so no two lines' echoes fall on simple ratios (which
   would ring metallically). Lines are assigned to left/right in a
   balanced pattern (blocks of four sorted lengths as L, R, R, L), so both
   channels have equal total delay and the image doesn't lean. This is a
   principled size mapping, not a simulation of a specific room's modes --
   the usual level of physical fidelity for an FDN tail.
2. **Frequency-dependent decay** (`OnePoleDamping`): per line, a one-pole
   shelf whose DC and Nyquist gains are solved in closed form (Jot) for the
   per-pass decay implied by the Decay Time (T60) at DC and by Damping at
   Nyquist. Retargets glide (`setDecayTargets()`); `snapDecayTargets()`
   switches instantly.
3. **Mixing** (`householderMix16()`): exactly orthogonal, so the mix step
   preserves energy to floating-point precision. Every line feeds the
   shared matrix, so an impulse in the left channel reaches the right
   within milliseconds.
4. **Diffusion** (`AllpassDiffuser`): 4 Schroeder all-pass stages per
   channel ahead of the network, their gain scaled by Diffusion -- against the
   metallic sound a small-line-count network otherwise has.
5. **Modulation** (`ModulatedDelay`): a slow LFO-modulated fractional delay
   in each line's feedback loop, depth set by Modulation. It breaks up
   resonant modes that otherwise sound like the material pitched far down.
   No added latency; exactly transparent at 0.
6. **Spread** (mid/side blend of the reverb output: 0 mono, 1 natural,
   2 wider) and **Balance** (per-channel gain of the reverb output).

## Reverb Plate (`plugin/dsp/ReverbEnginePlate.hpp`)

A different topology, following Jon Dattorro, "Effect Design, Part 1:
Reverberator and Other Filters" (JAES 45(9), 1997), with his published
delay lengths, gains and taps (Tables 1 and 2), scaled to the sample rate
and room size:

- a cascade of input-diffusion all-passes;
- two tanks in a figure-8: each a modulated all-pass, a long delay and a
  damping filter, feeding the other tank. The "decay diffusion 2"
  all-pass and its second delay form a branch that only feeds output taps
  and never re-enters the loop;
- the stereo output built from seven weighted, alternating-sign taps per
  channel read from both tanks -- this tap density is what gives a smooth,
  decorrelated tail without discrete echoes.

The loop gain is set from the Decay Time, correcting for the group delay
of the all-passes inside the loop so the measured T60 matches the target.

## Shared by both

- **Pre-Delay** (`PreDelayLine.hpp`): 0–250 ms on the signal feeding the
  reverb only, gliding (5 ms) to new values, exactly transparent at 0.
- **Input Gain** before the dry/reverb split; **Dry/Wet** and **Output
  Trim** (reverb signal) at the end.
- **Room-size changes**: a new room changes every delay length, which has
  no in-between. `requestRoom()` publishes the new size atomically from
  any thread; the audio thread picks it up, fades out (~8 ms), rebuilds
  the lines and fades back in.
- **Stability** is exact by construction: an orthogonal mix composed with
  damping gains strictly below 1 can only lose energy, for any setting. A
  per-sample NaN/Inf check remains as a safety net.

## Parameters

| Parameter | Range / default | Maps to |
|---|---|---|
| Length / Width / Height | 1–80, 1–80, 1–40 m; 10 x 7 x 3.5 | delay lengths (a State, since it rebuilds the lines) |
| Input Gain | -24..+24 dB | before the dry/reverb split |
| Pre-Delay | 0–250 ms | `PreDelayLine` |
| Decay Time | 0.1–20 s, 1.8 | damping DC target (T60) |
| Tempo Sync, Note, Follow Host Tempo | off; 1/16 … 4 bars | decay from the host tempo, see below |
| Damping | 0–1, 0.5 | damping Nyquist target |
| Diffusion | 0–1, 0.7 | input all-pass gain |
| Modulation | 0–1, 0.3 | delay modulation depth |
| Spread | 0–2, 1 | mid/side width of the reverb output |
| Balance | -1..+1, 0 | L/R gain of the reverb output |
| Dry/Wet | 0–100 %, 35 | mix |
| Output Trim | -24..+24 dB | reverb level before the mix |

Output Parameters for the editor: host tempo valid, detected BPM, and the
decay time actually active in the engine.

## Tempo-synced decay

With Tempo Sync on, Decay Time is derived from the host tempo:
`t60 = 60 / BPM * beats(note value)`, following the common rule of thumb
(a quarter note is a short ambience, a half note a medium room, one bar a
long hall, two bars a very long tail). Bar values use the host's time
signature (one bar at 120 BPM in 3/4 is 1.5 s). The tempo comes from
DPF's `getTimePosition()` (`bbt.beatsPerMinute`, `bbt.beatsPerBar`); if
the host reports none, 120 BPM is used, like DPF's own backends.

- **Follow Host Tempo off**: the tempo is captured once when sync is
  switched on (or the note value changes).
- **Follow Host Tempo on**: re-read every block; changes glide through the
  smoothed damping targets. In a delay-line engine a coefficient change
  reaches the output only after a trip round the line, but the damping
  filters' internal state would still jump, so retargets are smoothed.

The editor shows the reported tempo and swaps the Decay Time field for
the live, read-only synced decay.

## EQ on the reverb signal

An ordinary `EqModel` assembled into a `StateSpaceEngine` (the EQ code is
ported from `statespace`), applied to the reverb signal before the
dry/wet mix. Two spectrum analysers (before and after the EQ, packed 3
bands per output Parameter) are drawn behind the curve; they only run
while the editor is open (`kParamUiHeartbeat`).

The band list is stored as one State (`eqModel`, JSON), not as the
Parameter mirror the `statespace` EQs use. Ardour's channel-strip import
does not carry plugin State files, so an imported reverb strip loses its
EQ settings; the `statespace` Parameter mirror is the known fix if this
matters.

Matrix export ("Export to State-Space") doesn't apply: the reverb network
has no representation of 128 states or fewer.

## Editor (`gui/UIReverb.*`)

Shared by both plugins (`plugin/ReverbParams.hpp` defines the common
parameter and state enums): room fields, reverb fields (Decay Time swaps
to the synced readout under Tempo Sync), Tempo Sync / Note / Follow Host
Tempo buttons with the host tempo line, Bypass and Reset, and
`EqGraphWidget` + `BandListWidget` for the EQ, with **Fixed Range /
Autoscale** for the spectrum.

## Reference samples (`tools/reference/`)

`generate_reference_samples` renders rooms with a higher-order
(Allen & Berkley) shoebox image-source model -- independent of the FDN and
plate technology, as ground truth -- for synthesized excitations (hand
clap, noise burst, sine sweep) and for dry recordings placed in
`<out_dir>/user_dry/`. Each run checks its first order against
`firstOrderImageSources()`. Intended as training data for fitting a fast
structure to measured rooms; the fitting itself is not built.

## Performance

Reverb Dense costs O(16) per sample pair (mix, damping, modulated reads,
diffusers), independent of room size. The generic state-space engine at a
32-band EQ (order 128) needs about 16 000 multiply-adds per sample for the
recursion alone -- several hundred times more than this reverb's whole
network, which holds thousands of samples of state.

## Known limitations

- Early reflections (image sources) are computed and tested but not mixed
  into the output of the FDN engine.
- The source/listener position in the room is fixed.
- The delay-length mapping models room size, not a specific room's modes.
- No EQ inside the network (per-line "objects in the room" absorption).
- The editor has no preset load/save, A/B, undo or tooltips yet, and no
  room or decay-curve visualization.
- The EQ band list is a State (see above).

## References

- [FDN Reverberation (Stanford CCRMA, J. O. Smith)](https://ccrma.stanford.edu/~jos/pasp/FDN_Reverberation.html)
- [First-Order Delay Filter Design (Stanford CCRMA)](https://ccrma.stanford.edu/~jos/pasp/First_Order_Delay_Filter_Design.html) -- the DC/Nyquist damping fit
- [Unitary Matrix Design for Diffuse Jot Reverberators](https://www.researchgate.net/publication/230757792_Unitary_Matrix_Design_for_Diffuse_Jot_Reverberators)
- [On Lossless Feedback Delay Networks](https://www.researchgate.net/publication/304469760_On_Lossless_Feedback_Delay_Networks)
- [Feedback Delay Networks in Artificial Reverberation and Reverberation Enhancement](https://www.researchgate.net/publication/322951473_Feedback_Delay_Networks_in_Artificial_Reverberation_and_Reverberation_Enhancement)
- [Implementation of a Hybrid Reverb Algorithm, M. Steimel, Aalborg University](https://projekter.aau.dk/projekter/files/310743966/Thesis_Report___Hybrid_Reverb___Maximilian_Steimel_SMC10.pdf)
- [FDNTB: The Feedback Delay Network Toolbox (DAFx 2020)](https://dafx2020.mdw.ac.at/proceedings/papers/DAFx2020_paper_53.pdf)
- J. Dattorro, "Effect Design, Part 1: Reverberator and Other Filters", J. Audio Eng. Soc. 45(9), 1997
- J. B. Allen, D. A. Berkley, "Image method for efficiently simulating small-room acoustics", JASA 65(4), 1979
- [Reverb Time Calculator -- BPM, Pre-Delay and Decay (ReverbFX)](https://reverbfx.com/tools/reverb-time-calculator)
