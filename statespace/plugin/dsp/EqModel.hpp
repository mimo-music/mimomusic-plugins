// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// EqModel.hpp
//
// The "musical" parameter layer for the sibling parametric EQ plugin
// (plugin/PluginEq.cpp, gui/UIEq.cpp) -- an ordered list of bands (type,
// frequency, gain, Q, enabled/solo), unbounded in principle, capped in
// practice only by kMaxOrder (StateSpaceTypes.hpp) once assembled: each
// band is one biquad per stereo channel (4 states), so up to
// kMaxOrder/4 = 32 bands fit, comfortably past FabFilter Pro-Q's
// advertised 24-band maximum.
//
// This is a worked coefficient-mapper (see CoefficientMapper.hpp's own
// header comment for the pattern) rather than a new engine or file format
// for the *audio* side: assembleStateSpace() below turns an EqModel into
// an ordinary StateSpaceMatrices via mappers::makeStereoBiquadChain() (a
// series cascade -- exactly what a multi-band EQ is, each band filtering
// the previous band's output), which is then run through
// StabilityGuard::enforce() and StateSpaceEngine::setModel() exactly like
// any other model in this project. Consequently "export to the state-space
// plugin" needs no new file format at all: the assembled StateSpaceMatrices
// is serialized with the *existing* Serialization::toJson(), producing a
// perfectly ordinary .ssjson the base plugin's own "Load..." button opens
// directly, coefficients and all, ready for hand-editing.
//
// The EQ plugin's live GUI curve deliberately does *not* go through
// assembleStateSpace() + TransferFunction::evaluate() (the generic,
// dense-matrix path the base plugin's Bode plot uses) -- benchmarked at a
// worst-case 32-band order-128 model, that sweep costs ~47ms per curve
// (O(order^3) dense complex Gaussian elimination; see StateSpaceTypes.hpp's
// kMaxOrder comment), which is fine for the base plugin's debounced Bode
// plot but far too slow to redraw on every pixel of an interactive node
// drag. Since an EQ's bands are *known* to be a series cascade of
// biquads, the combined response is exactly the *product* of each band's
// own closed-form H(z) -- evaluateCombinedResponse() below computes that
// directly, O(numBands) per frequency point instead of O(order^3),
// letting the GUI redraw the true combined curve in real time on every
// drag event while the (comparatively expensive, but audio-only-visible)
// assemble+StabilityGuard+engine-push is debounced instead -- see
// UIEq.cpp's uiIdle().

#pragma once

#include "CoefficientMapper.hpp"
#include "StateSpaceTypes.hpp"

#include <complex>
#include <string>
#include <vector>

namespace ssfilter {

// BandShelf is appended last, not grouped with the other shelves: the
// Parameter mirror stores a band's type as its integer value
// (kParamBandTypeBase), so inserting it anywhere else would renumber every
// type after it in already-saved sessions.
enum class EqBandType { Bell, LowShelf, HighShelf, LowPass, HighPass, Notch, BandPass, AllPass, BandShelf };

inline const char* eqBandTypeName(EqBandType t)
{
    switch (t)
    {
    case EqBandType::Bell:      return "Bell";
    case EqBandType::LowShelf:  return "Low Shelf";
    case EqBandType::HighShelf: return "High Shelf";
    case EqBandType::LowPass:   return "Low Pass";
    case EqBandType::HighPass:  return "High Pass";
    case EqBandType::Notch:     return "Notch";
    case EqBandType::BandPass:  return "Band Pass";
    case EqBandType::AllPass:   return "All Pass";
    case EqBandType::BandShelf: return "Band Shelf";
    }
    return "?";
}

// Low Pass/High Pass/Notch/Band Pass/All Pass have no meaningful gain
// parameter (the RBJ formulas for them don't take one) -- the GUI pins
// their node to the 0dB line and only lets frequency (and Q, via scroll)
// be dragged, rather than showing a gain control that would silently do
// nothing.
inline bool eqBandTypeHasGain(EqBandType t)
{
    return t == EqBandType::Bell || t == EqBandType::LowShelf || t == EqBandType::HighShelf || t == EqBandType::BandShelf;
}

// Band Shelf: a flat-topped boost (or, with negative gain, a flat-bottomed
// cut) of `gainDb`, `bandwidthOct` octaves wide and centred (geometrically)
// on `freqHz`, returning to exactly 0dB outside. Built from its two corners
// lo = freqHz / 2^(bw/2), hi = freqHz * 2^(bw/2) as High Shelf(lo, +G) x
// Low Shelf(hi, +G) x a flat -G on the whole band: below both corners only
// the Low Shelf is up (+G - G = 0dB), between them both are (+2G - G = +G),
// above both only the High Shelf is (+G - G = 0dB). Two shelves alone can't
// do this -- they have one gain knob each for three regions -- the flat -G
// is the third one. Both shelves share the band's one Q. Each corner is the
// half-gain (G/2 dB) point of its own RBJ shelf, so on the graph the edges
// sit where the curve is halfway up. See eq::designBandStages().
inline bool eqBandTypeHasBandwidth(EqBandType t)
{
    return t == EqBandType::BandShelf;
}

// EqBand::bandwidthOct's range: 0.05 octaves (corners ~3.5% apart) to 10
// octaves (about the whole audible range).
constexpr double kMinBandwidthOct = 0.05;
constexpr double kMaxBandwidthOct = 10.0;

// Which stereo channel(s) a band actually filters -- Both (the only
// option before this existed, and still the default: every existing
// preset/session loads with every band reading Both, identical to before)
// applies the band's own biquad identically to L and R, same as always;
// Left/Right restrict it to just that one channel, leaving the other
// channel's signal completely untouched *by this band* (it still passes
// through every other Both/matching-channel band in the cascade
// normally). See eq::assembleStateSpace()'s own comment for how this actually
// produces two independently-filtered channels via
// mappers::makeStereoBiquadChain(), and DynamicEqEngine.hpp's identical
// field on DynamicEqBand for the dynamic EQ's own per-sample version.
enum class EqChannel { Both = 0, Left = 1, Right = 2 };

inline const char* eqChannelName(EqChannel c)
{
    switch (c)
    {
    case EqChannel::Both:  return "Both";
    case EqChannel::Left:  return "Left Only";
    case EqChannel::Right: return "Right Only";
    }
    return "Both";
}

// Both -> Left -> Right -> Both -- the single click-to-cycle order every
// Channel control in the GUI uses (the band list's own toggle, and the
// dynamic EQ's Channel button), so there's exactly one place that order
// is decided.
inline EqChannel nextEqChannel(EqChannel c)
{
    switch (c)
    {
    case EqChannel::Both:  return EqChannel::Left;
    case EqChannel::Left:  return EqChannel::Right;
    case EqChannel::Right: return EqChannel::Both;
    }
    return EqChannel::Both;
}

// Stereo crossfeed routing for a single band -- see EqBand::crossFeed's
// own comment for the full reasoning (a deliberately separate concept
// from EqChannel above, not two more values on that shared enum).
enum class EqCrossFeed { None = 0, LeftToRight = 1, RightToLeft = 2 };

inline const char* eqCrossFeedName(EqCrossFeed cf)
{
    switch (cf)
    {
    case EqCrossFeed::None:        return "Off";
    case EqCrossFeed::LeftToRight: return "L -> R";
    case EqCrossFeed::RightToLeft: return "R -> L";
    }
    return "Off";
}

// None -> L->R -> R->L -> None -- the click-to-cycle order
// BandListWidget's own crossfeed column uses, mirroring nextEqChannel()'s
// identical role for the Channel column right next to it.
inline EqCrossFeed nextEqCrossFeed(EqCrossFeed cf)
{
    switch (cf)
    {
    case EqCrossFeed::None:        return EqCrossFeed::LeftToRight;
    case EqCrossFeed::LeftToRight: return EqCrossFeed::RightToLeft;
    case EqCrossFeed::RightToLeft: return EqCrossFeed::None;
    }
    return EqCrossFeed::None;
}

// Both -> Left -> Right -> L->R -> R->L -> Both -- the single unified
// click-to-cycle order the Parametric EQ's own Channel column uses
// (presenting `channel`/`crossFeed` as ONE control, even though they stay
// two genuinely separate fields underneath -- see EqCrossFeed's own
// comment for why they aren't merged into one enum). Shared here (moved
// out of gui/eq/BandListWidget.cpp, where it first shipped) specifically
// so the Dynamic EQ 2 plugin's own single Channel button
// (gui/dyneq2/UIDynEq2.cpp) can reuse the EXACT same cycle order and
// transition logic, so both plugins behave identically -- rather than a
// second,
// hand-copied implementation that could quietly drift out of sync with
// this one.
//
// Moving OUT of a crossfeed state resets crossFeed to None (obviously)
// but leaves `outCh` at Both rather than trying to guess which plain
// channel the user might want next -- Both is what every band already
// defaults to, and is the correct value should a caller ever read
// `channel` while `crossFeed` is still (briefly, mid-click) non-None.
// Moving INTO a crossfeed state doesn't bother resetting `channel` at
// all (it's ignored once crossFeed != None, see EqCrossFeed's own
// comment), so `outCh` there is simply whatever `ch` already was -- one
// fewer caller-side write needed per click.
inline void advanceChannelCrossFeed(EqChannel ch, EqCrossFeed cf, EqChannel& outCh, EqCrossFeed& outCf)
{
    if (cf == EqCrossFeed::LeftToRight) { outCh = ch; outCf = EqCrossFeed::RightToLeft; return; }
    if (cf == EqCrossFeed::RightToLeft) { outCh = EqChannel::Both; outCf = EqCrossFeed::None; return; }
    if (ch == EqChannel::Both)  { outCh = EqChannel::Left;  outCf = EqCrossFeed::None; return; }
    if (ch == EqChannel::Left)  { outCh = EqChannel::Right; outCf = EqCrossFeed::None; return; }
    /* ch == Right */            outCh = ch;               outCf = EqCrossFeed::LeftToRight;
}

struct EqBand
{
    EqBandType type = EqBandType::Bell;
    double freqHz = 1000.0;
    double gainDb = 0.0;
    double q = 1.0;
    bool enabled = true;
    bool solo = false; // see eq::activeBandIndices()

    // Steepness, in cascaded 2nd-order (12dB/octave) sections -- 1 (the
    // default, and the *only* value that ever meant anything before this
    // field existed) is a single ordinary RBJ Low/High Pass biquad at this
    // band's own Q; 2/3/4 cascade that same biquad 2/3/4 times running
    // (24/36/48dB/octave), preserving the user's own Q rather than forcing
    // a fixed Butterworth one -- see eq::designBandStages()'s own comment
    // for why identical repeated stages, not per-stage-tuned Butterworth
    // sections, is the deliberate design here. Only meaningful for
    // LowPass/HighPass (every other type ignores it, same convention as
    // eqBandTypeHasGain() gating Gain for types that don't use it).
    int slopeStages = 1;

    // Harmonic link -- "create a twin of this band at a harmonic
    // frequency, and keep them moving together afterward." 0 = not
    // linked to anything. A positive value groups this band together
    // with every other band in `bands` sharing the same value;
    // `harmonicRatio` is this band's own *fixed* frequency ratio
    // relative to the group's common, virtual (never separately stored)
    // base frequency. Moving any one member's freqHz (drag, scroll-
    // nudge, or typed edit -- see eq::propagateHarmonicFreq(), the one
    // function every frequency-changing gesture in the UI calls) derives
    // that base as `movedFreq / itsOwnRatio` and repositions every other
    // member to `base * itsOwnRatio`. Deliberately *not* a chain of
    // pairwise ratios (this band relative to whichever other band it was
    // created from) -- storing each member's ratio relative to the
    // shared, virtual base instead means deleting or unlinking any one
    // member (including the very one a later member was originally
    // created from) never requires recomputing anyone else's own ratio:
    // there is no "middle" to break, every member already stands on its
    // own. A lone band left in a group after every other member is
    // removed/unlinked is harmless (nothing else shares its group id, so
    // moving it moves nothing else) rather than needing special-cased
    // cleanup back to harmonicGroup=0.
    int harmonicGroup = 0;
    double harmonicRatio = 1.0;

    // Which stereo channel(s) this band filters -- see EqChannel's own
    // comment. Both by default, matching every band's behavior before
    // this field existed.
    EqChannel channel = EqChannel::Both;

    // Stereo crossfeed -- a genuinely different routing from `channel`
    // above (which only ever restricts a band to filtering ONE channel
    // IN PLACE, never touching the other): None (the default, and the
    // only value that ever meant anything before this field existed)
    // leaves `channel` in full control, exactly as before. LeftToRight/
    // RightToLeft instead take that band's own filtered SOURCE channel's
    // raw input and ADD it into the OTHER (destination) channel's own
    // output, on top of whatever that destination channel's own bands
    // already produce -- e.g. to move sounds that sit only on the left
    // over to the right, or the other way around, to shape a track's
    // stereo field. `channel` is ignored on a
    // band with `crossFeed != None` (there is no "both directions at
    // once" -- pick two bands, one each way, for that). Deliberately a
    // SEPARATE enum from EqChannel, not two new EqChannel values: EqChannel
    // is reused as-is by the sibling Dynamic EQ (DynamicEqModel.hpp,
    // DynamicEqEngine.hpp's own per-sample envelope-detector/gain-reducer
    // logic, built assuming only Both/Left/Right ever appear) -- adding
    // crossfeed there is a much bigger, separately-scoped change (a
    // static filter's output can just be added to another channel's
    // output; a *dynamic* band's own detector/gain-reduction loop has no
    // equivalent well-defined "which channel reacts, which channel hears
    // the result" story yet), so this stays a plain-EQ-only concept
    // rather than risk quietly changing what an existing EqChannel value
    // means for Dynamic EQ's own bands. See eq::assembleStateSpace()'s
    // own comment for exactly how this gets built into the assembled
    // matrix, and IEqEditorHost::supportsCrossFeed()/setBandCrossFeed()
    // for why the GUI side needs zero changes for any existing host
    // (UIEqMono, ../../reverb's own UIReverb) that doesn't opt in.
    EqCrossFeed crossFeed = EqCrossFeed::None;

    // Band Shelf only (see eqBandTypeHasBandwidth()): the width in octaves
    // between its two corners, centred on `freqHz` -- so freqHz means "the
    // centre" for a Band Shelf just like for a Bell, and positioning,
    // dragging and harmonic linking need no special case. Every other type
    // ignores this. Kept last in the struct so existing positional
    // initializers (`{ type, freq, gain, q, enabled, solo }`) keep meaning
    // what they did. Default 2: one octave either side of the centre.
    double bandwidthOct = 2.0;
};

struct EqModel
{
    std::vector<EqBand> bands;
    double outputTrimDb = 0.0; // applied uniformly to C/D after assembly, like normalizeToUnityPassbandGain
    char name[64] = "Untitled EQ";

    // The crossfeed transit's own OVERALL, flat/broadband send level --
    // ONE knob per direction, for the WHOLE L->R (or R->L) signal path,
    // not per band. Separate from any crossfeed band's own `gainDb`
    // (still exactly what it always meant: that band's own Bell/Shelf/
    // etc. shape's own peak boost/cut at its own frequency, riding on
    // top of this baseline). Applied once, after every active crossfeed
    // band in that direction has already been cascaded together (see
    // eq::assembleStateSpace()'s own graft-step comment) -- scaling the
    // combined C/D output rows, exactly like `outputTrimDb` above scales
    // the whole model's own output, so it means "the whole transit's
    // level" regardless of how many bands happen to be routed that way.
    //
    // Originally tried as a PER-BAND field instead (each crossfeed band
    // carrying its own crossFeedGainDb, scaled into that band's own
    // biquad numerator) -- reverted: the send level belongs to the whole
    // transit, starting at -60dB there, not to each band. That design
    // also had a real, non-obvious bug: multiple crossfeed bands
    // in the same direction cascade in SERIES (same as the main L/R
    // chains always have), so their individual gain knobs multiplied
    // together (in dB, added) rather than acting as independent,
    // interchangeable controls -- raising one band's own knob while an
    // earlier band in the same chain was still at its -60dB default left
    // the whole transit stuck near -60dB regardless, deeply
    // unintuitive. A single post-cascade scalar has no such interaction:
    // it means exactly what it says, once, no matter how many bands
    // route that way. Defaults to -60dB -- quiet but genuinely
    // controllable, rather than a crossfeed that jumps straight in at full
    // strength the moment a band is switched to L->R/R->L.
    double crossFeedGainLtoRDb = -60.0;
    double crossFeedGainRtoLDb = -60.0;
};

namespace eq {

// Q is clamped to a sane range and frequency to just under Nyquist so a
// wild/edited value can't send the RBJ formulas somewhere undefined --
// StabilityGuard::enforce() is still run on the assembled result
// regardless, this is just cheap insurance one level earlier.
BiquadCoeffs designBand(const EqBand& band, double sampleRate);

// The actual, possibly multi-stage, cascade a band contributes to the
// series chain: for every type except LowPass/HighPass, or either of
// those at slopeStages==1, this is just {designBand(band, sampleRate)} --
// a single element, unchanged from before slopeStages existed. For a
// LowPass/HighPass with slopeStages > 1, it's that *same* biquad repeated
// slopeStages times -- deliberately identical repeated sections (not a
// per-stage-tuned proper Butterworth design the way
// mappers::designLinkwitzRileyLowpass()/Highpass() build a crossover's own
// stages) so the user's own Q control keeps meaning exactly what it
// already means at any steepness, rather than being overridden by a fixed
// Butterworth Q the moment steepness is turned up. This is the same
// "cascade N identical sections" pattern that project already uses for a
// different reason (Linkwitz-Riley crossover order); the steepness this
// produces is close to, but not bit-identical to, a proper multi-order
// Butterworth (whose per-stage Q values are tuned so the *combined*
// response stays maximally flat -- identical repeated sections roll off
// correctly but the knee is very slightly softer), a documented, honest
// simplification rather than an attempt at bit-exact Butterworth response.
std::vector<BiquadCoeffs> designBandStages(const EqBand& band, double sampleRate);

// Indices into model.bands that should actually be audible: every
// enabled band, *unless* one or more bands are both enabled and soloed,
// in which case only those are -- the usual "solo mutes everything else"
// convention, without requiring a separate "was anything soloed" flag on
// the model itself.
std::vector<int> activeBandIndices(const EqModel& model);

// True if any enabled band is also soloed -- the same condition
// activeBandIndices() computes internally to decide whether it's
// filtering at all, exposed so assembleStateSpace() can use it too (to
// decide whether a channel with no *soloed* content reaching it should be
// silenced rather than left as a passthrough -- see that function's own
// comment).
bool hasActiveSolo(const EqModel& model);

// Harmonic linking -- see EqBand::harmonicGroup's own comment for the
// full data-model reasoning. These three are the only supported ways to
// change a band's link state or a linked band's frequency; callers
// should never write harmonicGroup/harmonicRatio/a linked band's freqHz
// directly, the same "only through these functions" discipline
// mb::addCrossover()/removeCrossover()/setCrossoverHz() already enforce
// for MbCompressorModel's own crossover list.

// A fresh, currently-unused harmonic group id (one past whatever the
// highest id already in use is, or 1 if none) -- used when linking a
// previously-unlinked band for the first time.
int nextHarmonicGroupId(const EqModel& model);

// Sets band `index`'s frequency (clamped to [1, sampleRate*0.499], same
// bound every other per-band frequency setter in this project uses), and
// -- if it's harmonically linked -- repositions every other member of
// its group to keep every member's own ratio intact. This is the one
// function every frequency-changing UI gesture (drag, scroll-nudge, or a
// typed edit) should call instead of writing `model.bands[index].freqHz`
// directly, so linking behaves identically no matter which gesture moved
// the band. For a Band Shelf freqHz is its centre, so linked Band Shelves
// move as a whole, each keeping its own bandwidth.
void propagateHarmonicFreq(EqModel& model, int index, double freqHz, double sampleRate);

// Creates a new band at `sourceIndex`'s own frequency times `factor`
// (clamped to the same Nyquist bound propagateHarmonicFreq() uses),
// copying its type/gain/Q/slopeStages/bandwidth/enabled, and harmonically links it
// to `sourceIndex` -- minting a fresh group id via nextHarmonicGroupId()
// if `sourceIndex` wasn't already linked to anything (in which case it
// becomes the group's ratio-1.0 reference point), or reusing the
// existing group id and multiplying `sourceIndex`'s own current ratio by
// `factor` otherwise (so repeatedly creating a twin from an
// already-linked band chains further harmonics, e.g. x2 then x2 again
// lands a third member at x4 of the original root, not x2 of it again).
// Inserted immediately after `sourceIndex`, mirroring
// UIEq::duplicateBand()'s own placement convention. `factor` must be
// finite and > 0; refuses (returns -1, no change made) past kMaxOrder/4
// bands (see UIEq::addBand()'s identical cap) or for an invalid
// `sourceIndex`/`factor`. Returns the new band's index on success.
int createHarmonicTwin(EqModel& model, int sourceIndex, double factor, double sampleRate);

// Removes band `index` from its harmonic group (harmonicGroup=0) --
// leaves its current frequency, and every other member of its (now
// possibly smaller, possibly empty) former group, exactly where they
// are; see EqBand::harmonicGroup's own comment for why no other member
// ever needs adjusting.
void unlinkHarmonic(EqModel& model, int index);

// A Band Shelf's two corner frequencies (unclamped): freqHz / 2^(bw/2) and
// freqHz * 2^(bw/2). Used for the DSP design, the graph's corner handles
// and Audition.
double bandShelfLowCornerHz(const EqBand& band);
double bandShelfHighCornerHz(const EqBand& band);

// Changes a band's type; types without gain get gainDb=0 (a stale, unused
// value otherwise lingers). freqHz is the centre for every type, so
// switching into or out of Band Shelf leaves the band where it was.
void applyBandType(EqBand& band, EqBandType type);

// Series-cascades every active band's biquad into one StateSpaceMatrices
// via mappers::makeStereoBiquadChain() -- one independent chain per
// stereo channel, built from two *separately filtered* stage lists: a
// Both band's stages go into both, a Left/Right band's stages go into
// only its own -- so the two channels only ever end up with identical
// coefficients when every active band actually is Both, and otherwise are
// genuinely, independently filtered (this is exactly what
// makeStereoBiquadChain() already supports for any pair of stage lists;
// see EqChannel's own comment for the feature this enables). Then applies
// outputTrimDb to C/D. An empty active-band list returns an identity
// passthrough (trimmed). Caller is still expected to run
// StabilityGuard::enforce() before StateSpaceEngine::setModel(), same as
// any other mapper output.
//
// Crossfeed bands (EqBand::crossFeed != None) are handled as a SEPARATE
// step after the above, entirely additive to it: their own stages are
// cascaded into a small extra system (reusing makeStereoBiquadChain()
// again, just for the two crossfeed directions instead of L/R) and
// GRAFTED onto the end of the main model -- new diagonal A states (so
// StateSpaceMatrices::sparseCascade2's own "never above a stage's own
// 2-wide block" guarantee still holds exactly, since a fresh diagonal
// block trivially satisfies it), but with B/C wired with channels
// SWAPPED relative to a normal band: a LeftToRight band's own states are
// fed from input column 0 (u_L, same as any Left-reaching band) but
// their OUTPUT is added into output row 1 (y_R) instead of row 0 -- the
// one deliberate asymmetry that turns "an independently-filtered copy of
// L" into "L's own content, crossfed into R's own output". When no band
// has crossFeed set (every model that predates this field, and any model
// that simply doesn't use it), this step is skipped entirely -- `order`
// never grows, and the result is bit-identical to what this function
// always produced.
StateSpaceMatrices assembleStateSpace(const EqModel& model, double sampleRate);

// True if any active (enabled, and audible under solo) band is Left- or
// Right-only, OR has crossfeed enabled -- i.e. the two channels are
// genuinely, audibly different right now, not just nominally "stereo".
// Used by the GUI/plugin to decide whether a per-channel (as opposed to
// combined) spectrum display is actually meaningful; see PluginEq.hpp's
// own comment on its L/R output spectrum analyzers.
bool hasPerChannelBands(const EqModel& model);

// Mono counterpart of assembleStateSpace(), used by the sibling mono EQ
// plugin (plugin_eq_mono/PluginEqMono.cpp) -- same band list, same
// per-band RBJ design via designBand(), but cascaded as a single SISO
// chain (mappers::makeMonoBiquadChain()) instead of two independent L/R
// chains, so the assembled order is half of what assembleStateSpace()
// would produce for the same bands (see MonoStateSpaceTypes.hpp's header
// comment for why that's the actual mechanism behind the mono plugin's
// DSP saving). Same empty/all-disabled -> identity-passthrough behavior.
mono::MonoStateSpaceMatrices assembleMonoStateSpace(const EqModel& model, double sampleRate);

// Closed-form discrete biquad response H(e^{j*omega}) = (b0 + b1 z^-1 +
// b2 z^-2) / (1 + a1 z^-1 + a2 z^-2), z^-1 = e^{-j*omega}. O(1) -- this is
// what makes the live curve cheap; see file comment.
std::complex<double> evaluateBiquadResponse(const BiquadCoeffs& c, double omega);

// One band's own response in isolation (ignores enabled/solo -- used to
// draw each band's individual dashed curve regardless of whether it's
// currently contributing to the combined curve). `extraFlatGainDb`
// (default 0, meaning "none") is an additional flat/broadband multiplier
// applied on top of the band's own shape -- used to show a crossfeed
// band's own curve riding on the model's own crossFeedGainLtoRDb/
// crossFeedGainRtoLDb baseline (see EqModel::crossFeedGainLtoRDb's own
// comment for why that lives on the model, not the band), the same
// externally-resolved-gain shape DynamicEqModel.hpp's own
// evaluateBandResponse() already uses for its own, differently-sourced,
// per-frame gain.
std::complex<double> evaluateBandResponse(const EqBand& band, double sampleRate, double freqHz, double extraFlatGainDb = 0.0);

// The product of every *active* band's response (see activeBandIndices())
// times the linear output trim -- this is exactly what the assembled
// StateSpaceMatrices' H(z) would evaluate to, computed directly instead.
// Deliberately channel-agnostic -- includes every active band regardless
// of its own EqChannel, i.e. "if this were all applied to one channel,
// what would it look like" -- used wherever the caller genuinely doesn't
// care about a per-channel split (Export to State-Space's own math, and
// the graph's single white curve when hasPerChannelBands() is false, see
// evaluateChannelResponse() below for the split-aware counterpart).
//
// A crossfeed band (EqBand::crossFeed != None) is deliberately EXCLUDED
// from this product: it doesn't reshape a channel's own signal the way a
// single multiplicative H(z) term can represent, it INJECTS a filtered
// copy of the OTHER channel's own signal on top of it -- a genuinely
// two-input effect a single per-channel curve can't honestly show. Its
// own designed filter shape is still visible via evaluateBandResponse()
// (its own individual dashed curve, and the graph's node itself), just
// not folded into a channel's combined response line. (Export to
// State-Space is unaffected by this exclusion -- that path serializes
// eq::assembleStateSpace()'s own actual matrix directly, which DOES
// include crossfeed content; only this closed-form GUI-curve shortcut
// skips it.)
std::complex<double> evaluateCombinedResponse(const EqModel& model, double sampleRate, double freqHz);

// Same idea, but only includes bands that actually reach `listenerChannel`
// (Both bands, plus whichever of Left/Right matches) -- i.e. "what does
// THIS channel alone actually hear". `listenerChannel` must be Left or
// Right (Both isn't a meaningful *listener*, only a band's own routing);
// passing Both here behaves exactly like evaluateCombinedResponse(),
// included only so this function stays total. Used by the graph to draw
// two genuinely different curves (blue/red) once hasPerChannelBands() is
// true, instead of one curve that would otherwise silently misrepresent
// whichever channel a Left/Right-only band doesn't reach. Crossfeed bands
// are excluded here too, for the same reason as evaluateCombinedResponse()
// above -- meaning a model with ONLY a crossfeed band active (no plain
// Left/Right/Both bands) will show two identical-looking curves even
// though the two channels' actual audio genuinely differs; the crossfeed
// itself is visible on the graph via its own node's ring, not the curve.
std::complex<double> evaluateChannelResponse(const EqModel& model, double sampleRate, double freqHz, EqChannel listenerChannel);

// ".sseqjson" -- the EQ's own band-list preset format (distinct from the
// base plugin's .ssjson, which stores raw matrices; an EqModel has no
// matrices of its own to store, only the musical parameters they're
// derived from). See UIEq's "Export to State-Space..." button for writing
// an actual .ssjson instead, via Serialization::toJson(assembleStateSpace(...)).
std::string toJson(const EqModel& model);
bool fromJson(const std::string& text, EqModel& out);

} // namespace eq
} // namespace ssfilter
