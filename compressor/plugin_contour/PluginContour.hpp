// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include <vector>

#include "../plugin/dsp/CurveDsp.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"

START_NAMESPACE_DISTRHO

// One source, two plugins: "Contour" (stereo) and "Contour Mono" are the SAME code, built twice. The mono build
// is selected by `#define CONTOUR_MONO 1` in plugin_contour_mono/DistrhoPluginInfo.h (main input + sidechain
// input -> one output, always the Linked single-curve mode, no stereo-mode/link/listen controls). Everything
// mono-specific is a `if (kContourMono)` / `#if CONTOUR_MONO` inside the shared files (this header,
// PluginContour.cpp, gui/UIContour.*), so any change made there reaches both plugins.
#if defined(CONTOUR_MONO) && CONTOUR_MONO
constexpr bool kContourMono = true;
#else
constexpr bool kContourMono = false;
#endif

// Same automatable-Parameter/State split as PluginCompressorStereo, minus
// Ratio/Knee/Ratio-Direction (this plugin has neither a ratio nor a knee
// -- the curve itself replaces both) and plus StereoMode, which -- a
// small discrete choice -- is a State (see plugin/dsp/CurveDsp.hpp's own
// header comment for the full design this plugin wraps).
//
// kParamCurvePointCount/kParamCurveTriggerCondition/kParamCurvePointInBase/
// kParamCurvePointOutBase/kParamMirrorCommit at the end of this enum are
// the Parameter mirror of curve_/triggerCondition_ -- see
// ../statespace/plugin_eq/PluginEq.hpp's ParameterIndex comment for the
// full root cause and design (Ardour used to persist a DPF State to an
// external file a route/channel-strip import never copies, but embeds an
// ordinary Parameter inline; those States -- TriggerCondition, and the
// curve's own point list -- are gone now, this mirror is the only
// representation left). kParamCurveInterp/kParamCurveInterpB are
// vestigial (interpolation moved per-point, a tangent handle mirrored
// via kParamCurvePointHandleInBase below, replacing the old curve-wide
// Linear/Spline choice this pair used to carry -- see initParameter()'s
// own comment, PluginContour.cpp). Threshold is already a real Parameter
// (kParamThreshold above) and needs no mirror. The publish direction
// lives in UIContour::pushCurveParamMirror(), not here -- same "only a
// UI-initiated setParameterValue() call gets a host to persist it"
// reasoning as every other port of this fix.
constexpr uint32_t kNumMirroredPoints = uint32_t(sscomp::TransferCurve::kMaxPoints);
// How many (detector level, gain) pairs per voice the plugin publishes per audio
// block for the GUI's heat trail (see kParamMeterTrailBankA). The GUI can only
// see ONE block per redraw whatever we do (output ports hold the latest value),
// so this is how many points that one block is worth: 48 instead of the single
// last sample it used to be.
constexpr uint32_t kTrailBankSize = 48;

enum ParameterIndex
{
    kParamInputGain = 0,
    kParamThreshold,     // dB -- deliberately independent of any curve point, see CurveDsp.hpp's header comment
    kParamAttack,
    kParamRelease,
    kParamOutputGain,
    kParamCompensation,
    kParamCompensationMeasure, // trigger: force a fresh compensation measurement now
    kParamBypass,

    kParamMeterInputRmsL,
    kParamMeterInputPeakL,
    kParamMeterInputRmsR,
    kParamMeterInputPeakR,
    kParamMeterOutputRmsL,
    kParamMeterOutputPeakL,
    kParamMeterOutputRmsR,
    kParamMeterOutputPeakR,
    kParamMeterDetectorA,       // Linked/DualMono: L; MidSide: Mid
    kParamMeterDetectorB,       // Linked: mirrors A; DualMono: R; MidSide: Side
    kParamMeterGainReductionA,
    kParamMeterGainReductionB,
    // Volume Compensation removed. This pair (plus kParamCompensation/
    // kParamCompensationMeasure above) stay declared at their exact
    // existing positions -- see PluginCompressorStereo.cpp's identical,
    // empirically-verified comment on its own kParamCompensation for why.
    // Always 0 now.
    kParamMeterCompGainDb,
    kParamMeterCompState,

    // Same external key input as PluginCompressorStereo's own -- see that
    // plugin's kParamSidechainEnable comment for the full design.
    kParamSidechainEnable,
    kParamSidechainGain,

    // The curve/trigger-condition Parameter mirror -- see this enum's own
    // header comment.
    kParamCurvePointCount,             // 2..kNumMirroredPoints (a curve always has at least its 2 fixed endpoints)
    kParamCurveInterp,                 // sscomp::CurveInterp, as an integer
    kParamCurveTriggerCondition,       // sscomp::TriggerCondition, as an integer
    kParamCurvePointInBase,            // + [0 .. kNumMirroredPoints)
    kParamCurvePointOutBase = kParamCurvePointInBase + kNumMirroredPoints,
    // See ../statespace/plugin_eq/PluginEq.hpp's identical
    // kParamMirrorCommit comment -- the GUI fires this immediately after
    // writing every field of the mirror bank above for one edit,
    // guaranteed to arrive last within that burst. NOT a
    // kParameterIsTrigger -- see that comment for the real bug
    // (Ardour doesn't implement the LV2 trigger-port convention, so a
    // constant resent value only ever reaches the DSP once) and why this
    // has to be a plain automatable value the GUI toggles every push
    // instead. Shared by *both* curve banks (A's above, B's own further
    // below) -- applyMirrorToModel() applies whichever of A/B just
    // changed unconditionally either way, so one shared commit signal is
    // enough; the GUI always keeps both mirrors current in its own
    // paramCache_ regardless of which bank the user is actually editing
    // (see UIContour::pushCurveParamMirror()'s own comment).
    kParamMirrorCommit = kParamCurvePointOutBase + kNumMirroredPoints,

    // Everything below this line was ported from the sibling
    // PluginCompressorStereo.hpp, so windows, sidechain, tooltips and
    // input/output gain controls behave the same. Appended as one block after the
    // pre-existing bank above (including the curve mirror), same "append,
    // never insert" index-stability rule as every other addition in this
    // project -- see PluginCompressorStereo.hpp's own identical comments
    // on each of these for the full per-feature reasoning; only the
    // curve-specific differences get their own comments here.

    // In Mid/Side the two can be set completely differently -- see
    // kParamListenMode's own comment
    // (PluginCompressorStereo.hpp).
    kParamListenMode,

    // A second full dynamics bank -- Contour's own version of
    // kParamThresholdB/kParamRatioB/.../kParamMakeupGainB has no Ratio/
    // Knee (this plugin has neither), but does need a **second whole
    // curve** -- the freely drawn curve applies per bank too, once B is
    // independent, not just to A.
    // See kParamThresholdB's own comment (PluginCompressorStereo.hpp) for
    // the "start linked, split on edit" fallback every field here follows
    // identically -- B mirrors A's own curve_/triggerCondition_ (not just
    // its scalars) whenever it isn't independent, exactly like
    // isChannelBIndependent() already governs every other B-bank field.
    kParamThresholdB,
    kParamAttackB,
    kParamReleaseB,
    kParamMakeupGainB,
    kParamGlobalOutputGain,

    // Sidechain frequency-band filter + monitor -- see
    // kParamScFilterEnable/kParamMeterScSpectrumBase/kParamScListenEnable's
    // own comments (PluginCompressorStereo.hpp) for the full design. No
    // legacy mono kParamMeterScLevelDb slot here (unlike the compressor's
    // own, which had to keep one for session compatibility with an
    // already-shipped index) -- Contour never had this feature before, so
    // it goes straight to the per-channel L/R pair.
    kParamScFilterEnable,
    kParamScFilterHighpassHz,
    kParamScFilterLowpassHz,
    kParamMeterScSpectrumBase, // sscomp::SpectrumAnalyzer::kNumPackedBands (128) packed Parameters -- post-filter/current ("Out")
    kParamScListenEnable = kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands,
    kParamScFilterLinkLR,
    kParamScFilterHighpassHzR,
    kParamScFilterLowpassHzR,
    kParamMeterScLevelDbL,
    kParamMeterScLevelDbR,
    kParamMeterScSpectrumInBase, // another kNumPackedBands packed Parameters -- pre-filter ("In")

    // Per-channel input amplification in Dual-Mono/Mid-Side -- see
    // kParamInputGainB's own comment (PluginCompressorStereo.hpp).
    kParamInputGainB = kParamMeterScSpectrumInBase + sscomp::SpectrumAnalyzer::kNumPackedBands,

    // A Wet/Dry control per compressor -- see
    // kParamWetDry/kParamWetDryB's own comment (PluginCompressorStereo.hpp).
    // Blends the dry, gain-staged-but-not-yet-curve-shaped signal against
    // the curve-processed+Makeup-Gain signal (the curve-based counterpart
    // of "compressed+made-up" there).
    kParamWetDry,
    kParamWetDryB,

    // A global gain on the stereo signal before splitting or converting to
    // M/S -- see kParamGlobalInputGain's own
    // comment (PluginCompressorStereo.hpp) for the full design (Dual-Mono/
    // Mid-Side only, DSP-inert in Linked too, reset to 0dB by the GUI
    // whenever stereoMode_ leaves Linked).
    kParamGlobalInputGain,

    // B's own curve mirror -- the one piece of this whole port with no
    // direct PluginCompressorStereo counterpart (that plugin has no
    // per-voice "curve"), built as an exact second copy of
    // kParamCurvePointCount/kParamCurveInterp/kParamCurveTriggerCondition/
    // kParamCurvePointInBase/kParamCurvePointOutBase above, same shape,
    // same kNumMirroredPoints sizing, appended here (not next to A's own)
    // for the same "append, never insert" reason everything else in this
    // block is appended rather than interleaved.
    kParamCurvePointCountB,
    kParamCurveInterpB,
    kParamCurveTriggerConditionB,
    kParamCurvePointInBaseB,
    kParamCurvePointOutBaseB = kParamCurvePointInBaseB + kNumMirroredPoints,

    // Per-point hysteresis direction tag mirror (two different paths up
    // and down). One integer (sscomp::PointDirection: 0=Both/1=RisingOnly/
    // 2=FallingOnly) per point, both banks, same indexing convention as
    // kParamCurvePointInBase/OutBase above. Appended here -- after both
    // banks' own position/count/interp/condition mirrors but still
    // strictly BEFORE kParamMirrorCommitB below -- so it's covered by
    // that trigger's own "always arrives last within one block" guarantee
    // (see kParamMirrorCommitB's own comment); putting new mirror data
    // anywhere after that trigger would silently reintroduce the exact
    // one-edit-stale bug it exists to prevent.
    //
    // LAYOUT NOTE (a real bug, fixed 2026-09-21): every bank below is
    // kNumMirroredPoints wide, and each base MUST be spelled out as "previous
    // base + kNumMirroredPoints". These used to be plain enumerators
    // (kParamCurvePointDirBase, kParamCurvePointHandleInBase), which are only
    // "previous enumerator + 1" -- so the banks overlapped by 17 slots each
    // (OutBaseB/DirBase and DirBaseB/HandleInBase). setParameterValue()'s
    // first-match chain then sent "handle X of node n" to the direction-B
    // branch and "direction of node n" to the output-B branch: the DSP never
    // received any X support-point component, curve A's Rising/Falling tags
    // never arrived, and curve B's outputs were overwritten by direction
    // values. The static_asserts after this enum keep it from happening again.
    kParamCurvePointDirBase = kParamCurvePointOutBaseB + kNumMirroredPoints,
    kParamCurvePointDirBaseB = kParamCurvePointDirBase + kNumMirroredPoints,

    // Per-point Inkscape-style tangent handle mirror (replacing the old
    // curve-wide Linear/Spline toggle -- see
    // plugin/dsp/CurveDsp.hpp's own header comment for the full design).
    // Two floats per point (sscomp::CurvePoint::handleInputDb/
    // handleOutputDb), both banks -- same indexing convention as every
    // other per-point mirror bank above, and same ordering requirement:
    // appended here, still strictly BEFORE kParamMirrorCommitB below.
    kParamCurvePointHandleInBase = kParamCurvePointDirBaseB + kNumMirroredPoints,
    kParamCurvePointHandleOutBase = kParamCurvePointHandleInBase + kNumMirroredPoints,
    kParamCurvePointHandleInBaseB = kParamCurvePointHandleOutBase + kNumMirroredPoints,
    kParamCurvePointHandleOutBaseB = kParamCurvePointHandleInBaseB + kNumMirroredPoints,

    // A second commit trigger, for B's own curve bank specifically --
    // checking Contour's processing surfaced a real bug here: kParamMirrorCommit above sits well
    // *before* B's curve bank in this enum's own index order, but the
    // host delivers one block's worth of dirty Parameters to
    // setParameterValue() in ascending index order (confirmed by reading
    // DistrhoPluginLV2.cpp's own delivery loop -- see kParamMirrorCommit's
    // own comment, "guaranteed to arrive last within that burst," which is
    // only actually true for A's own bank). Editing curveB_ therefore had
    // the old kParamMirrorCommit fire and adopt mirrorPointsB_ *before*
    // this same block's freshly-written B mirror fields (higher indices)
    // had arrived -- one edit permanently stale, since nothing re-fires
    // the commit until the *next* edit. kParamMirrorCommitB is appended
    // here, strictly after B's own bank (the true last Parameter overall),
    // so it always arrives after every mirror field from both banks within
    // one block; UIContour::pushMirrorCommit() now fires both triggers on
    // every edit (harmless for the un-affected one -- applyMirrorToModel()
    // is a plain idempotent copy either way), and the correct result always
    // wins by the time run() executes for that block.
    kParamMirrorCommitB = kParamCurvePointHandleOutBaseB + kNumMirroredPoints,

    // The live trail's (detector level, gain) pair as ONE output value, per
    // voice -- see sscomp::packTrailPair()'s own comment for why two separate
    // meter Parameters can't be trusted to pair up (they tore across audio
    // blocks, scattering the dots off the curve at 0ms attack/release).
    // Outputs are never delivered to setParameterValue(), so appending them
    // after kParamMirrorCommitB does not touch that trigger's "arrives last"
    // guarantee, which only concerns the input mirror fields.
    kParamMeterTrailA,
    kParamMeterTrailB,

    // Bank of kTrailBankSize atomic (level, gain) pairs per voice, sampled at
    // evenly spaced instants through the block just processed -- many more
    // points, fading out, show where the sound is moving. Each slot is its own
    // packed pair (see sscomp::packTrailPair()), so slots can't tear against each
    // other's halves; kParamMeterTrailA/B above stay as the "latest sample" used
    // for the marker and the GR column. Outputs only, appended at the end.
    kParamMeterTrailBankA,
    kParamMeterTrailBankB = kParamMeterTrailBankA + kTrailBankSize,

    // OBSOLETE (kept so old sessions and the index layout stay valid; ignored): the dots always show the real audio -- its own
    // level on x, level + applied gain on y -- because the curve now maps the audio itself. It used to choose what the trail dots' x
    // position meant: 0 = the REAL audio -- input level on x,
    // output level (input + the curve's gain) on y; 1 = the DETECTOR level (what the
    // curve is evaluated at: after Trigger Gain, the Trigger Filter, the sidechain
    // choice and the attack/release envelope).
    // An input Parameter (so the choice is saved with the session), appended last.
    kParamTrailSource = kParamMeterTrailBankB + kTrailBankSize,

    // Lookahead in ms (0 = off): the audio is delayed by this much (reported to the host as latency) while
    // the trigger is evaluated un-delayed, so the curve/diagonal is already in place when a trigger's audio
    // arrives ("is it possible to make on attack 0 a lookahead window? so when the trigger comes, its on the
    // line or on curve for sure"). Global, appended last.
    kParamLookahead = kParamTrailSource + 1,

    // "The Trigger Options panel is showing" heartbeat, same idea as the statespace EQs' kParamUiHeartbeat: the
    // editor bumps this counter every idle tick WHILE the trigger scope is visible (so nothing is sent with the
    // window closed or the panel hidden), and run() feeds the two spectrum analysers -- about 70 % of this
    // plugin's CPU -- only while it has changed within the last ~2 s. An increasing counter, not the EQs' 0/1
    // toggle: DPF's LV2 wrapper only delivers a port whose value differs from the last block's, so with a
    // toggle an even number of ticks inside one (long) block cancels out.
    kParamUiHeartbeat = kParamLookahead + 1,

    kParamCount = kParamUiHeartbeat + 1
};

// Every per-point mirror bank is kNumMirroredPoints wide and must not share an
// index with any other -- see the LAYOUT NOTE inside the enum above.
static_assert(kParamCurvePointOutBase >= kParamCurvePointInBase + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointOutBaseB >= kParamCurvePointInBaseB + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointDirBase >= kParamCurvePointOutBaseB + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointDirBaseB >= kParamCurvePointDirBase + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointHandleInBase >= kParamCurvePointDirBaseB + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointHandleOutBase >= kParamCurvePointHandleInBase + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointHandleInBaseB >= kParamCurvePointHandleOutBase + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamCurvePointHandleOutBaseB >= kParamCurvePointHandleInBaseB + kNumMirroredPoints, "mirror banks overlap");
static_assert(kParamMirrorCommitB >= kParamCurvePointHandleOutBaseB + kNumMirroredPoints, "commit trigger must come after every mirror bank");
static_assert(kParamMeterTrailBankA > kParamMeterTrailB, "trail bank must come after the single-pair trail outputs");
static_assert(kParamMeterTrailBankB >= kParamMeterTrailBankA + kTrailBankSize, "trail banks overlap");

enum StateIndex
{
    kStateStereoMode = 0, // sscomp::StereoMode (0=Linked/1=DualMono/2=MidSide) -- same as PluginCompressorStereo's own
    // last folder used in the Save As.../Load... dialogs (UI convenience
    // only). (Three former entries used to sit here -- "triggerCondition",
    // "interp", "curve" -- all gone now: the curve's own shape, interp
    // mode, and trigger condition moved entirely to the Parameter mirror
    // above. Renumbering below is safe, states are addressed by key
    // string for session persistence, never by this enum's numeric value.)
    kStatePresetDir,
    // A/B compare, made to survive a session close/reopen -- see the
    // sibling ../statespace/plugin_eq/PluginEq.hpp's identical
    // StateIndex comment for the full reasoning: a plain State, not the
    // import-safe Parameter mirror the curve itself just got, since A/B
    // is a workflow convenience rather than the core audio-shaping data.
    // Each slot serializes a whole ContourSnapshot (see UIContour.hpp's
    // own comment on that type, and UIContour::serializeSnapshot()) as one
    // opaque string -- the DSP side never parses it, purely UI<->host
    // round trip, same as presetDir.
    kStateAbSlotA,
    kStateAbSlotB,
    kStateAbHasA,
    kStateAbHasB,
    kStateAbActive,
    // Link L/R parameters (Dual-Mono) -- see
    // kStateLinkAB's own comment (PluginCompressorStereo.hpp). A discrete
    // two-way choice, same State (not Parameter) convention as
    // kStateStereoMode.
    kStateLinkAB,
    kStateCount
};

class PluginContour : public Plugin
{
public:
    PluginContour();

protected:
    uint32_t getVersion() const override { return d_version(1, 1, 0); } // 1.1: adapted every ../plugin_stereo feature this session (sidechain filter/scope, dual-mono/mid-side B-bank incl. its own curve, Global/per-channel Input Gain, Wet/Dry) -- existing indices unchanged

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    // See ../statespace/plugin_eq/PluginEq.hpp's PluginEq::activate() own
    // comment -- called after both State and Parameter restore are
    // complete, exactly where adoptMirrorIfCurveNotReceived() needs to
    // run.
    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    // Whether B's own dynamics bank (kParamThresholdB etc., including
    // curveB_ itself) is actually live right now -- see
    // PluginCompressorStereo::isChannelBIndependent()'s own comment, this
    // is a verbatim port.
    bool isChannelBIndependent() const
    {
        return stereoMode_ == sscomp::StereoMode::MidSide ||
               (stereoMode_ == sscomp::StereoMode::DualMono && !linkAB_);
    }

    // Re-applies scFilterHighpassHz_/scFilterLowpassHz_ to scFilterL_
    // always, and to scFilterR_ too unless scFilterLinkLR_ is false -- see
    // PluginCompressorStereo::updateScFilterFrequencies()'s own comment,
    // verbatim port.
    void updateScFilterFrequencies()
    {
        scFilterL_.setFrequencies(scFilterHighpassHz_, scFilterLowpassHz_);
        scFilterR_.setFrequencies(scFilterLinkLR_ ? scFilterHighpassHz_ : scFilterHighpassHzR_,
                                   scFilterLinkLR_ ? scFilterLowpassHz_ : scFilterLowpassHzR_);
    }

    // Copies curve_/triggerCondition_'s current shape into the Parameter
    // mirror's own backing store below -- called whenever either changes
    // via setState(). See ../statespace/plugin_eq/PluginEq.hpp's identical
    // pair for the full reasoning, including why the "publish to the
    // host" direction deliberately does NOT live here. Syncs *both*
    // curve banks (A and B) in one call -- see mirrorEverSet_'s own
    // comment.
    void syncCurveMirrorFromModel();
    // Only takes effect once mirrorReceivedFromHost_ is true and this
    // hasn't already run once -- see those flags' own comments. Unlike
    // EqModel/DynamicEqModel, TransferCurve's own default constructor
    // already creates 2 points (the fixed endpoints, "a safe, does-
    // nothing starting shape"), so curve_/curveB_ can never be "empty" the
    // way those two models' own default (zero bands) naturally are -- the
    // exact same reasoning the sibling ../statespace project's own
    // Multiband Compressor port of this fix already ran into.
    void adoptMirrorIfCurveNotReceived();
    // The actual mirror -> curve_/curveB_/triggerCondition_/
    // triggerConditionB_ copy, unconditional (no gating) -- shared by
    // adoptMirrorIfCurveNotReceived() (restore path, gated below) and
    // setParameterValue()'s kParamMirrorCommit case (live edit path,
    // always applies). Applies *both* banks every time, not just whichever
    // one actually changed -- harmless (the other bank's own mirror
    // fields are always kept current too, see kParamMirrorCommit's own
    // comment), and keeps this function, like every other piece of this
    // mirror, a single unconditional copy rather than needing its own
    // "which one changed" tracking.
    void applyMirrorToModel();
    // True once any mirror Parameter has actually been written (by a host
    // restoring it, or by this plugin's own constructor priming it to
    // match curve_'s own real default -- see the constructor) -- guards
    // adoptMirrorIfCurveNotReceived() against ever adopting
    // mirrorPoints_'s own raw, un-synced (all-zero) initial contents,
    // since -- unlike mirrorBandCount_ elsewhere, where 0 is itself a
    // meaningful "nothing here" sentinel -- a curve's point count always
    // starts at 2 by construction, so it alone can't distinguish "really
    // has 2 real points" from "never touched".
    bool mirrorEverSet_ = false;
    // mirrorEverSet_ alone can't tell "the constructor's own self-primed
    // default" apart from "genuine data, import-restored or otherwise" --
    // it's set true unconditionally in the constructor (priming the
    // mirror to curve_'s own default) as well as by a real host restore.
    // mirrorReceivedFromHost_ is the real signal, set only from
    // setParameterValue()'s own handling of a mirror Parameter (never by
    // the constructor's self-priming sync). mirrorAdopted_ stops
    // adoptMirrorIfCurveNotReceived() from re-adopting on every single
    // run() call for the lifetime of an instance, once it's already run
    // once.
    bool mirrorReceivedFromHost_ = false;
    bool mirrorAdopted_ = false;

    sscomp::CurveVoice voiceA_, voiceB_;
    sscomp::LoudnessMeter loudnessInL_, loudnessInR_, loudnessOutL_, loudnessOutR_;
    sscomp::PeakMeter peakInL_, peakInR_, peakOutL_, peakOutR_;
    sscomp::TransferCurve curve_;
    // B's own independent curve -- see kParamCurvePointCountB's own
    // comment. Defaults identical to curve_'s own default (a plain 2-point
    // identity line), so a freshly-instantiated plugin behaves identically
    // regardless of stereo mode until something actually diverges them,
    // same "defaults match A's own" precedent every other B-bank field in
    // this project already follows.
    sscomp::TransferCurve curveB_;

    // The Parameter mirror's own backing store -- see ParameterIndex's own
    // comment. A's own; B's own pair (mirrorPointsB_ etc.) sits right
    // after, same shape.
    // Each CurvePoint here already carries its own handleInputDb/
    // handleOutputDb (see plugin/dsp/CurveDsp.hpp) alongside inputDb/
    // outputDb/direction -- no separate handle storage needed, the
    // Parameter mirror's kParamCurvePointHandleInBase/HandleOutBase cases
    // (setParameterValue()/getParameterValue()) read/write straight into
    // these same points, same as every other per-point field already
    // does.
    sscomp::CurvePoint mirrorPoints_[kNumMirroredPoints];
    int mirrorPointCount_ = 2;
    sscomp::TriggerCondition mirrorTriggerCondition_ = sscomp::TriggerCondition::Above;
    sscomp::TriggerAction mirrorTriggerAction_ = sscomp::TriggerAction::ApplyCurve; // packed into the trigger-condition Parameter, see packConditionAndThresholdLink()
    int mirrorThresholdLink_ = 0;  // carried inside the trigger-condition Parameter, see packConditionAndThresholdLink()
    sscomp::CurvePoint mirrorPointsB_[kNumMirroredPoints];
    int mirrorPointCountB_ = 2;
    sscomp::TriggerCondition mirrorTriggerConditionB_ = sscomp::TriggerCondition::Above;
    sscomp::TriggerAction mirrorTriggerActionB_ = sscomp::TriggerAction::ApplyCurve;
    int mirrorThresholdLinkB_ = 0;

    double inputGainDb_ = 0.0;
    double thresholdDb_ = -18.0;
    double attackMs_ = 10.0;
    double releaseMs_ = 150.0;
    double outputGainDb_ = 0.0;
    // Vestigial -- round-trips kParamCompensation's own stored value, but
    // nothing reads it any more. See that Parameter's own comment
    // (initParameter(), PluginContour.cpp).
    bool compensationEnabled_ = false;
    bool bypass_ = false;
    bool sidechainEnabled_ = false;
    double sidechainGainDb_ = 0.0;
    sscomp::StereoMode stereoMode_ = sscomp::StereoMode::Linked;
    sscomp::TriggerCondition triggerCondition_ = sscomp::TriggerCondition::Above;
    sscomp::TriggerAction triggerAction_ = sscomp::TriggerAction::ApplyCurve;   // what a met trigger does, curve A
    sscomp::TriggerAction triggerActionB_ = sscomp::TriggerAction::ApplyCurve;  // ...curve B
    int listenMode_ = 0; // 0=Off/1=Mid/2=Side -- see kParamListenMode's own comment (PluginCompressorStereo.hpp)

    // B's own independent dynamics bank -- see kParamThresholdB's own
    // comment above.
    double thresholdDbB_ = -18.0;
    double attackMsB_ = 10.0;
    double releaseMsB_ = 150.0;
    double makeupGainDbB_ = 0.0;
    sscomp::TriggerCondition triggerConditionB_ = sscomp::TriggerCondition::Above;
    double inputGainDbB_ = 0.0;
    double wetDry_ = 1.0;  // 0..1, not 0..100, converted once in setParameterValue() -- see kParamWetDry's own comment
    double wetDryB_ = 1.0;
    double globalInputGainDb_ = 0.0;
    bool linkAB_ = true; // see kStateLinkAB's own comment

    double globalOutputGainDb_ = 0.0;

    // See kParamScFilterEnable's own comment. One filter instance per
    // channel (L/R) -- verbatim port of PluginCompressorStereo's own.
    bool scFilterEnabled_ = false;
    double scFilterHighpassHz_ = 100.0;
    double scFilterLowpassHz_ = 8000.0;
    bool scFilterLinkLR_ = true;
    double scFilterHighpassHzR_ = 100.0;
    double scFilterLowpassHzR_ = 8000.0;
    sscomp::SidechainBandFilter scFilterL_, scFilterR_;

    sscomp::PeakMeter scMonitorPeakL_, scMonitorPeakR_;
    sscomp::SpectrumAnalyzer scMonitorSpectrumIn_, scMonitorSpectrumOut_;
    static constexpr uint32_t kScMonitorScratchFrames = 8192;
    float scMonitorScratchIn_[kScMonitorScratchFrames];
    float scMonitorScratchOut_[kScMonitorScratchFrames];

    bool scListenEnabled_ = false;

    float meterInputRmsL_ = -120.0f, meterInputPeakL_ = -120.0f;
    float meterInputRmsR_ = -120.0f, meterInputPeakR_ = -120.0f;
    float meterOutputRmsL_ = -120.0f, meterOutputPeakL_ = -120.0f;
    float meterOutputRmsR_ = -120.0f, meterOutputPeakR_ = -120.0f;
    float meterDetectorA_ = -120.0f, meterDetectorB_ = -120.0f;
    float meterGainReductionA_ = 0.0f, meterGainReductionB_ = 0.0f;
    // Safety net for a lost commit: any mirror DATA Parameter marks the mirror
    // dirty; a commit applies it and clears the flag; if data arrived but no
    // commit followed within kMirrorSettleBlocks blocks, run() applies it anyway,
    // so the DSP curve can never stay stale (see PluginContour::run()).
    bool mirrorDirty_ = false;
    int mirrorIdleBlocks_ = 0;
    static constexpr int kMirrorSettleBlocks = 3;
    float meterTrailA_ = 0.0f, meterTrailB_ = 0.0f; // packed pair, see kParamMeterTrailA
    float meterTrailBankA_[kTrailBankSize] = {}, meterTrailBankB_[kTrailBankSize] = {}; // see kParamMeterTrailBankA
    // Lookahead: the gain-staged audio (L, R, Mid, Side) is delayed by lookaheadSamples_ through these rings.
    static constexpr uint32_t kLookaheadRing = 4096; // power of two; 20 ms at up to 192 kHz = 3840
    static constexpr double kLookaheadMaxMs = 20.0;
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX; // samples since the last heartbeat delivery; UINT64_MAX = none yet
    double lookaheadMs_ = 0.0;
    uint32_t lookaheadSamples_ = 0;
    uint32_t reportedLatency_ = 0;
    uint32_t lookaheadPos_ = 0;
    std::vector<double> lookaheadRing_ = std::vector<double>(4 * kLookaheadRing, 0.0);
    bool trailDetectorMode_ = false; // kParamTrailSource: false = real input/output, true = detector level

    DISTRHO_DECLARE_NON_COPYABLE(PluginContour)
};

END_NAMESPACE_DISTRHO
