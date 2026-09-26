// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynamicEqModel2.hpp
//
// The "musical" parameter layer for the dynamic EQ plugin
// (plugin_dyn_eq2/PluginDynEq2.cpp, gui/dyneq2/UIDynEq2.cpp) -- FabFilter Pro-Q-
// style bands (frequency, bandwidth/Q, gain) that ALSO carry their own
// threshold/ratio/range/attack/release, so each band's own boost/cut moves
// toward less (or, symmetrically, more negative) gain whenever the signal
// *in that band's own frequency region* gets loud, then relaxes back --
// a multiband compressor's per-band dynamics, targeted at an EQ-style
// bell shape instead of a crossover slice. See docs/EXTENDING.md's "Dynamic EQ 2" section for the full
// derivation this file is a bookkeeping layer on top of.
//
// Why this needed a genuinely new mechanism, not just EqModel.hpp +
// StateSpaceTypes.hpp's existing MultibandSchedule:
//
//   - EqModel's bands are a *series* cascade of ordinary RBJ biquads
//     (mappers::designPeaking() et al.) -- correct for a *static* EQ, but
//     the RBJ peaking formula's denominator (a1, a2) depends on gain (the
//     `alpha/A` term), so smoothly varying a peaking band's gain every
//     sample means smoothly varying its *poles*, not just its C/D readout
//     -- exactly the "state-dependent coefficients can only ever be C(p)/
//     D(p), never A/B" line StateSpaceTypes.hpp's own file comment and
//     CoefficientMapper.hpp's own file comment draw, for good reason
//     (A's stability can't be cheaply/safely re-checked every sample).
//   - MultibandSchedule solves the analogous problem for the multiband
//     compressor by keeping each band's *filter* fixed (a crossover split)
//     and only ever scaling that already-fixed filter's *entire* output by
//     a level-dependent scalar -- but that only works because crossover
//     bands are non-overlapping and *sum* to the full spectrum; a "0dB
//     scalar" on one of them means "this slice passes through untouched".
//     A bell/shelf EQ band's node sits in the *middle* of the spectrum,
//     overlapping its neighbors, so the same trick doesn't apply: scaling
//     an isolated bandpass tap by a "0dB, do nothing" gain would still add
//     a whole extra copy of that bandpassed signal on top of everything
//     else, which is audibly wrong at rest.
//
// The fix used here is the classical Regalia-Mitra / DAFX "shelving via a
// fixed allpass" construction (A. Zolzer, *Digital Audio Signal
// Processing*, and P. Regalia & S. Mitra, 1987): for a fixed 2nd-order
// allpass A(z) (RBJ cookbook allpass -- exactly mappers::designAllpass(),
// already in this codebase, parameterized by center frequency and Q/
// bandwidth *only*, never by gain),
//
//     H(z) = 0.5*(1+G) + 0.5*(1-G)*A(z)
//
// is EXACTLY a peaking filter with center-frequency gain G (linear) and
// unity gain at DC/Nyquist (verified directly: A(z) = -1 at the allpass's
// own center frequency, giving H = G there; A(z) = +1 far from it, giving
// H = 1). Crucially, G enters *purely affinely* -- A(z)'s own state
// recursion (its "A"/"B", in state-space terms) never changes, only the
// linear combination of A(z)'s already-computed output with the dry
// signal -- so G can be swept every single sample from a level detector's
// output exactly as safely as NonlinearSchedule's existing C(p)=gain*C0
// mechanism, without ever touching a pole. This is a genuine, if less
// familiar, textbook 2nd-order EQ design in its own right (a *constant-
// bandwidth* peaking filter -- its -3dB width in Hz stays fixed regardless
// of gain, unlike RBJ's designPeaking(), whose alpha/A split makes it a
// *constant-Q* design whose bandwidth narrows slightly as gain increases);
// see this file's own designShaperAllpass()/evaluateBandResponse() for the
// concrete formulas.
//
// Consequently the *live, moving* audio path for this plugin is NOT built
// as one big StateSpaceMatrices the way EqModel/MbCompressorModel are --
// there is no single fixed A/B/C/D that a continuously-time-varying
// Regalia-Mitra gain per band, cascaded in series, could be expressed as
// (cascading several *time-varying* stages has no fixed combined state-
// space realization at all, only a fixed *recipe* for combining them
// sample by sample). See plugin/dsp/DynamicEqEngine2.hpp for that dedicated
// real-time engine -- this file is purely the musical-parameter layer plus
// the closed-form response math the GUI graph needs (both the band's
// resting/static shape and, given the engine's own live per-band gain
// readback, the currently-moving one that follows the program material).
//
// "Export to State-Space..." (assembleStateSpace() below) is necessarily a
// *snapshot*: it freezes every band at its resting gainDb (no dynamics --
// a fixed StateSpaceMatrices cannot represent a moving one at all, exactly
// the same fundamental limitation the project's very first mapper-pattern
// comment calls out) using the ordinary RBJ designPeaking()/EqModel-style
// cascade, not the Regalia-Mitra one -- a deliberate difference from the
// live engine, so the exported preset is an honest, ordinary, hand-
// editable parametric EQ a user opening it in the raw matrix editor would
// recognize, not a frozen allpass combination that would look opaque there.

#pragma once

#include "CoefficientMapper.hpp"
#include "EqModel.hpp" // for EqChannel/eqChannelName()/nextEqChannel() -- see DynamicEqBand2::channel's own comment for why this reuses the sibling EQ's own type rather than a second copy of it
#include "StateSpaceTypes.hpp"

#include <complex>
#include <string>
#include <vector>

namespace ssfilter {

// Ceiling on how many bands DynamicEqEngine2.hpp's real-time path can carry
// at once (a fixed-size array there, like StateSpaceTypes.hpp's kMaxBands
// for MultibandSchedule) -- chosen comfortably above what a dynamic EQ is
// normally used for (a handful of problem-frequency bands, not a full
// multi-band-crossover-style split), while each band here costs 4 small
// biquad states (shaper + detector, stereo) rather than kMaxOrder's states,
// so this ceiling is independent of, and much smaller than, EqModel's own
// 32-band (kMaxOrder-derived) one. The model itself (DynamicEqModel2::bands,
// a plain std::vector) is not bounded by this -- only what actually reaches
// the audio engine is, exactly like EqModel's own "unbounded in the GUI,
// capped only once assembled" convention (see PluginDynEq2::rebuildEngineModel()).
constexpr int kMaxDynEqBands2 = 16;

struct DynamicEqBand2
{
    // Band shape. Reuses the sibling (non-dynamic) EQ's own EqBandType
    // rather than a second enum, but only Bell/LowShelf/HighShelf/BandShelf
    // are ever reachable from this plugin's own UI (btnType_'s cycle, see
    // UIDynEq2.hpp, and the graph's right-click menu) --
    // LowPass/HighPass/Notch/BandPass/AllPass aren't
    // representable by the Regalia-Mitra "fixed allpass + gain blend"
    // construction this whole engine depends on for dynamic gain to be
    // safe to sweep every sample (see this file's own header comment):
    // those shapes aren't "unity far away, G at the corner/center", the
    // one property the construction requires. Bell keeps the original
    // 2nd-order allpass (q meaningful, a genuine bandwidth); Low/High
    // Shelf use a first-order allpass instead (see
    // dyneq2::designShaperAllpass()'s own comment) -- a first-order
    // shelf's own transition slope is fixed by construction, so `q` is
    // unused for these two, same "(unused)" convention the sibling
    // Limiter's own True-Peak-mode Attack/Knee fields already established
    // for a parameter that doesn't apply to the current mode.
    ssfilter::EqBandType type = ssfilter::EqBandType::Bell;
    double freqHz = 1000.0;
    double q = 1.0;              // bandwidth of both the shaping filter and its own level detector (see file comment) -- unused for Low/High Shelf, see `type`'s own comment
    double gainDb = 0.0;         // resting/static gain -- what the band sits at below threshold, or always if dynamicsEnabled is false
    double thresholdDb = -24.0;  // level (dB) of this band's OWN filtered signal that dynamics engage around
    double ratio = 4.0;          // downward-compression ratio (cuts gain when louder than threshold), same convention as MbBand::ratio
    double ratioUp = 1.0;        // upward-compression ratio (boosts gain when quieter than threshold); 1.0 = no upward effect (the historical default)
    double rangeDb = 5.0;        // ceiling on how far dynamics may pull gain away from gainDb, in *either* direction; 0 = static-only
    double attackMs = 10.0;
    double releaseMs = 100.0;
    bool dynamicsEnabled = true; // false: a plain static bell, no envelope tracking/movement at all
    bool enabled = true;         // false: this band is skipped entirely (not even a flat passthrough tap)

    // Which stereo channel(s) this band actually shapes/detects -- reuses
    // the sibling (non-dynamic) EQ's own EqChannel rather than a second,
    // identical enum (see EqModel.hpp's own comment for the feature this
    // is). Both (the default, and the only behavior before this field
    // existed) keeps today's shared-detector-level, both-channels-shaped
    // behavior; Left/Right restrict this band to reading and shaping only
    // that one channel, leaving the other completely untouched by it --
    // see DynamicEqEngine2::processBlock()'s own comment for exactly how.
    ssfilter::EqChannel channel = ssfilter::EqChannel::Both;

    // Stereo crossfeed -- reuses the sibling EQ's own EqCrossFeed (see
    // EqModel.hpp's own comment on why it's a separate enum from
    // EqChannel, and why it was deliberately NOT ported to the ORIGINAL
    // (non-"2") Dynamic EQ when the static EQ first got this feature: "a
    // *dynamic* band's own detector/gain-reduction loop has no equivalent
    // well-defined 'which channel reacts, which channel hears the result'
    // story yet". `dynamic_eq2` existing specifically as a safe place to
    // work that story out is exactly why it lands HERE first -- see
    // DynamicEqEngine2::processBlock()'s own comment for the answer this
    // clone settled on: a crossfeed band's OWN dynamics (threshold/ratio/
    // attack/release, all unchanged, no special-casing needed) react to
    // its SOURCE channel's level, and its shaped Regalia-Mitra output gets
    // ADDED into the DESTINATION channel's own signal, on top of whatever
    // is already there -- the exact same "own channel's content, added
    // elsewhere" story eq::assembleStateSpace()'s state-space graft tells,
    // just expressed as a per-sample addition instead of a matrix graft
    // (this engine has no matrix to graft onto in the first place, see
    // this file's own header comment). `channel` above is ignored once
    // this is non-None, same convention as the static EQ's own
    // EqBand::crossFeed.
    ssfilter::EqCrossFeed crossFeed = ssfilter::EqCrossFeed::None;

    // Harmonic link, as in the Parametric EQ. Same exact mechanism as
    // the sibling EqBand::harmonicGroup/harmonicRatio (see that field's
    // own comment for the full "virtual shared base, not a pairwise
    // chain" reasoning) -- a purely musical-model-layer concept (which
    // frequencies move together), with no DSP engine involvement at all,
    // so it ports across unchanged in shape even though this plugin's
    // own filtering math (Regalia-Mitra, not RBJ biquads) is completely
    // different.
    int harmonicGroup = 0;
    double harmonicRatio = 1.0;

    // Band Shelf only: width in octaves around freqHz, same field and
    // meaning as the Parametric EQ's EqBand::bandwidthOct (see
    // eqBandTypeHasBandwidth()). Here the shape is built from two
    // FIRST-order allpass shelves (see designShaperAllpass()/
    // designShaperAllpass2()), so like Low/High Shelf its slopes are fixed
    // and `q` is unused. Last in the struct so existing initializers keep
    // their meaning.
    double bandwidthOct = 2.0;
};

struct DynamicEqModel2
{
    std::vector<DynamicEqBand2> bands;
    double outputTrimDb = 0.0; // applied uniformly to C/D of the *exported* snapshot; the live engine applies it as a plain output multiply
    // Which band (if any) is currently soloed -- a single "spotlight"
    // index into `bands`, not a per-band flag: unlike the sibling EQ
    // plugin's BandListWidget (which shows every band at once and
    // genuinely benefits from soloing more than one simultaneously), this
    // UI shows only the *selected* band's own controls at a time, so a
    // per-band solo flag here just went stale the moment you selected a
    // different band (the sibling Multiband Compressor plugin, with the
    // identical UI shape, works the same way). UIDynEq2::selectBand() moves this
    // index along with the selection whenever solo is engaged. -1 = no
    // solo active. See dyneq2::activeBandIndices().
    int soloedBand = -1;
    char name[64] = "Untitled Dynamic EQ 2";

    // The crossfeed transit's own overall/static send level -- ONE knob
    // per direction, for the WHOLE L->R (or R->L) signal path, not per
    // band -- exactly EqModel::crossFeedGainLtoRDb/RtoLDb's own design
    // (see that field's own comment for the full "why model-level, not
    // per-band" reasoning, including the real bug a per-band version had
    // in its own first iteration). Here it's applied per-sample, directly
    // at each crossfeed band's own point of adding into the destination
    // channel (DynamicEqEngine2::processBlock()) -- since that's already
    // a plain additive `sigR += ...`/`sigL += ...`, not a series cascade,
    // multiple crossfeed bands in the same direction can never compound
    // this the way the state-space design's first attempt did (each
    // band's own contribution is scaled and added independently; scalar
    // multiplication distributing over addition means using the same
    // model-level gain at each one is exactly equivalent to "sum first,
    // scale once"). Defaults to -60dB, same safety-default reasoning.
    double crossFeedGainLtoRDb = -60.0;
    double crossFeedGainRtoLDb = -60.0;
};

namespace dyneq2 {

// Indices into model.bands that should actually be audible. Unlike the
// sibling EQ plugin's eq::activeBandIndices() (which can have more than
// one soloed band active at once, since BandListWidget shows every band's
// own solo toggle simultaneously), `soloedBand` is a single spotlight, so
// at most one extra index is ever excluded here on top of the ordinary
// per-band `enabled` filtering -- see DynamicEqModel2::soloedBand's own
// comment for why. (Unlike MbCompressorModel's crossover bands, which
// must all stay present to conserve total energy) an inactive band here
// is simply omitted from the cascade entirely, since overlapping bell
// bands don't need to sum to anything in particular.
std::vector<int> activeBandIndices(const DynamicEqModel2& model);

// Harmonic linking -- direct ports of EqModel.hpp's identical
// nextHarmonicGroupId()/propagateHarmonicFreq()/createHarmonicTwin()/
// unlinkHarmonic(), see those functions' own comments (EqModel.cpp) for
// the full reasoning; only the max-band-count cap differs
// (kMaxDynEqBands2, not EqModel's own kMaxOrder/4).
int nextHarmonicGroupId(const DynamicEqModel2& model);
void propagateHarmonicFreq(DynamicEqModel2& model, int index, double freqHz, double sampleRate);
int createHarmonicTwin(DynamicEqModel2& model, int sourceIndex, double factor, double sampleRate);
void unlinkHarmonic(DynamicEqModel2& model, int index);

// "linked bands should have linked threhold rations and ranges" (direct
// ask). Unlike freqHz (propagateHarmonicFreq(), which scales by each
// member's own harmonicRatio -- these bands sit at genuinely different
// frequencies, "linked" only in the sense of moving together),
// Threshold/Ratio/RatioUp/Range are level-domain settings with no
// per-member ratio to scale by at all -- every member of a group is
// meant to react identically, so this copies all four fields VERBATIM
// from `index` to every other member of its own group (a no-op when
// `index` isn't linked to anything). Call this AFTER writing whichever
// one of the four fields actually just changed on `model.bands[index]`
// -- same "the setter writes its own field, then this call fans it out"
// shape as every UIDynEq2::setBandThreshold()/setBandRatio()/
// setBandRatioUp()/setBandRange() call site uses. Re-copying the three
// fields that didn't just change is harmless (they're already identical
// across the group) and keeps this a single, simple function rather than
// four near-identical ones.
void propagateHarmonicDynamics(DynamicEqModel2& model, int index);

// Same purpose as EqModel.hpp's eq::hasPerChannelBands(): true if any
// active band is Left- or Right-only right now. Used to decide whether a
// per-channel spectrum display is actually meaningful; see
// PluginDynEq2.hpp's own comment.
bool hasPerChannelBands(const DynamicEqModel2& model);

// The fixed Regalia-Mitra allpass A(z) for one band -- depends only on
// freqHz (and, for Bell only, q -- clamped the same way eq::designBand()
// clamps them), never on gain. This is the "A/B" that never has to
// change on the audio thread; see file comment. Bell uses the original
// 2nd-order RBJ allpass (mappers::designAllpass()); Low/High Shelf use a
// first-order allpass instead (A(z) = (a+z^-1)/(1+a*z^-1), a standard,
// unconditionally-stable-for-any-corner-frequency construction -- see
// this function's own .cpp comment for the exact derivation and why the
// SAME combination formula becomes a low or high shelf purely from a
// sign flip, see shaperAllpassSign() below).
BiquadCoeffs designShaperAllpass(const DynamicEqBand2& band, double sampleRate);

// Band Shelf only: its SECOND allpass, first-order at the high corner.
// A Band Shelf is two Regalia-Mitra shelves in series plus a flat 1/G:
//   s1  = 0.5*(1+G)*x  + 0.5*(1-G)*A_lo(x)    (High Shelf at the low corner)
//   s2  = 0.5*(1+G)*s1 - 0.5*(1-G)*A_hi(s1)   (Low Shelf at the high corner)
//   out = s2 / G
// A(DC)=+1 and A(Nyquist)=-1 for any first-order allpass, so out = x at DC
// and at Nyquist, and out = G*x between the corners (A_lo ~ -1, A_hi ~ +1)
// -- the same flat-topped shape as the Parametric EQ's Band Shelf, with
// the same "poles never move with G" safety as every other band here.
// designShaperAllpass() supplies A_lo for a Band Shelf. For any other type
// this second stage is unused (returns a harmless first-order allpass at
// freqHz so its state has well-defined coefficients either way).
BiquadCoeffs designShaperAllpass2(const DynamicEqBand2& band, double sampleRate);

// A Band Shelf's corners: freqHz / 2^(bw/2) and freqHz * 2^(bw/2), with the
// bandwidth clamped to [kMinBandwidthOct, kMaxBandwidthOct].
double bandShelfLowCornerHz(const DynamicEqBand2& band);
double bandShelfHighCornerHz(const DynamicEqBand2& band);

// The sign the Regalia-Mitra combination (H = 0.5*(1+G) + sign*0.5*(1-G)*A(z))
// applies to the allpass term -- +1 for Bell and High Shelf (unity at DC,
// G at the corner/Nyquist), -1 for Low Shelf (G at DC/corner, unity at
// Nyquist). Bell's own +1 is unchanged from before this field existed
// (a symmetric 2nd-order allpass crosses -1 exactly at its own center
// frequency either way the sign goes... this file picks +1 by
// convention, matching every case this engine already shipped). Shared
// by both DynamicEqEngine2::processBlock() (the live per-sample
// combination) and evaluateBandResponse() (the closed-form one the GUI
// graph uses) so they can never disagree about which shape a given type
// produces.
double shaperAllpassSign(ssfilter::EqBandType type);

// A genuine (magnitude-shaped, 0dB-peak) bandpass, used purely to extract
// "how loud is this band's own frequency region right now" for the level
// detector -- the allpass above has flat magnitude everywhere by
// definition and can't serve this role. Centered at freqHz regardless of
// `type` -- even for a Shelf band (which shapes a whole half of the
// spectrum, not just a narrow region around freqHz), this measures
// loudness right at the shelf's own corner/transition, a reasonable,
// simple proxy rather than a wholesale redesign of what "how loud is
// this band's own region" means per shape.
BiquadCoeffs designDetectorBandpass(const DynamicEqBand2& band, double sampleRate);

// One band's own H(z) at a specific *resolved* gain (dB) -- either the
// resting gainDb (the static shape) or a live value read back from
// DynamicEqEngine2 (the moving one), via the Regalia-Mitra combination
// above. O(1), like eq::evaluateBandResponse() -- cheap enough to redraw
// every frame/drag event. `extraFlatGainDb` (default 0): the model's own
// crossFeedGainLtoRDb/RtoLDb transit baseline, for a crossfeed band's own
// individual preview curve -- same purpose, same externally-resolved-gain
// shape as eq::evaluateBandResponse()'s identically-named parameter (see
// that function's own comment).
std::complex<double> evaluateBandResponse(const DynamicEqBand2& band, double sampleRate, double freqHz, double resolvedGainDb, double extraFlatGainDb = 0.0);

// The product of every *active* band's response (series cascade, like
// eq::evaluateCombinedResponse()) times the linear output trim.
// `currentGainDb`, if non-null, must point to model.bands.size() entries
// (indexed by *band* index, not active-list index) and overrides each
// band's resting gainDb with a live one (DynamicEqEngine2::getCurrentGainDb())
// for drawing the moving curve; nullptr draws the resting/static shape.
std::complex<double> evaluateCombinedResponse(const DynamicEqModel2& model, double sampleRate, double freqHz,
                                               const double* currentGainDb = nullptr);

// Same idea as eq::evaluateChannelResponse(): only includes bands that
// actually reach `listenerChannel` (Both bands, plus whichever of Left/
// Right matches) -- "what does THIS channel alone actually hear". Used by
// the graph to draw two genuinely different curves (blue/red) once
// hasPerChannelBands() is true, instead of one curve that would otherwise
// silently misrepresent whichever channel a Left/Right-only band doesn't
// reach.
std::complex<double> evaluateChannelResponse(const DynamicEqModel2& model, double sampleRate, double freqHz,
                                              EqChannel listenerChannel, const double* currentGainDb = nullptr);

// The gain (dB) a band settles at for a given detected level -- "up and
// down compressing" in one function: above threshold, gain slopes down by
// 1-1/ratio (the classic downward case); below it, gain slopes *up* by
// 1-1/ratioUp (mirrored); both directions independently clamped to
// `rangeDb` (the "how far can this band's gain move" ceiling a per-band
// peak limiter/booster needs but GainCurve::classicCompressor()'s uncapped
// curve does not), and always relative to band.gainDb rather than a
// separate makeup value, so gainDb *is* both "the static shape" and "the
// center dynamics pulls away from" in either direction. ratioUp = 1.0 (the
// default) makes the whole quiet side flat, i.e. the original downward-
// only behavior, unchanged bit for bit.
double resolveDynamicGainDb(const DynamicEqBand2& band, double envelopeDb);

// Assembles model into an *ordinary* (static, no dynamics) StateSpaceMatrices
// via mappers::makeStereoBiquadChain() + mappers::designPeaking() at each
// active band's resting gainDb -- exactly EqModel::assembleStateSpace()'s
// own mechanism, reused for "Export to State-Space...". See file comment
// for why this deliberately does NOT use the Regalia-Mitra construction.
StateSpaceMatrices assembleStateSpace(const DynamicEqModel2& model, double sampleRate);

// ".ssdynjson" -- this plugin's own band-list preset format, same
// convention as EqModel's ".sseqjson" / MbCompressorModel's ".ssmbjson".
std::string toJson(const DynamicEqModel2& model);
bool fromJson(const std::string& text, DynamicEqModel2& out);

} // namespace dyneq2
} // namespace ssfilter
