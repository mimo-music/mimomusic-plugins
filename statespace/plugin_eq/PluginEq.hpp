// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/EqModel.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../plugin/dsp/StateSpaceEngine.hpp"

#include <cstdint>
#include <vector>

START_NAMESPACE_DISTRHO

// kParamSpectrumOutputBase/
// kParamSpectrumInputBase are banks of *output* parameters (one per
// SpectrumAnalyzer band, one bank per analyzer -- see spectrumMode_'s own
// comment below for why there are two analyzers at all) -- the same
// portable, guaranteed-cross-process mechanism kParamFaultFlag already
// uses (and the pattern DPF's own Meters example plugin uses for exactly
// this), since there is no direct shared-memory link between the DSP and
// UI sides in general.
//
// kParamBandCount/kParamOutputTrimDb/kParamBand*Base (right at the end of
// this enum) are the EQ's band list, carried entirely as an ordinary
// Parameter mirror rather than as a DPF State -- added originally to fix
// a real, root-caused data loss: importing a channel strip into another
// Ardour project kept the plugin but lost every band. A DPF State (this plugin used to keep one,
// "eqmodel") is persisted by Ardour to an *external* file next to the
// session (`<session>/plugins/<id>/stateN/state.ttl`, via the LV2 State
// extension), referenced from the saved session XML only by a
// `state-dir="stateN"` pointer -- but an ordinary Parameter gets embedded
// directly, inline, as a `<Port value="..."/>` XML element (confirmed by
// reading Ardour's own LV2Plugin::add_state()). Ardour's route/channel-
// strip import (Session::import_route_state() -> new_route_from_template())
// only replays that XML tree; it never copies the external plugins/<id>/
// directory from the source session into the destination one, so a State-
// only value used to be silently lost on import while an ordinary
// Parameter survives. Once the mirror was proven correct (every project
// using this plugin re-saved under a build that populated it), the
// "eqmodel" State was removed outright -- this bank is now the *only*
// representation of the band list a host ever sees or persists.
//
// The two directions are NOT symmetric, and that asymmetry is
// deliberate, not an oversight: restoring (host -> this bank ->
// eqModel_, via setParameterValue()/adoptMirrorIfModelEmpty() below) is
// plain DSP-side bookkeeping, since a host restoring saved Parameter
// values is exactly what it always does regardless of who asked. But
// *publishing* a change (eqModel_ -> this bank -> the host's own saved
// copy) can only happen from the GUI side, in UIEq::pushBandParamMirror()
// -- confirmed directly in Ardour's own LV2Plugin::add_state(), a host
// only ever persists whatever value *it* itself last explicitly set on
// an input Parameter (its own cached shadow copy), never re-reading the
// plugin's current value on its own initiative. A DSP-internal update
// with no corresponding UI-initiated "change request" would be
// completely invisible to the host and would never actually reach the
// saved session file -- syncMirrorFromModel() below still runs on every
// DSP-side model change (keeps getParameterValue() itself honest, and is
// what the GUI's own push ultimately reads from indirectly through
// eqModel_), but on its own it is NOT what makes any of this
// import-safe. That happens only once UIEq.cpp also calls
// setParameterValue() the ordinary "UI requests a Parameter change" way,
// same as every other UI-driven Parameter in this project, finishing with
// kParamMirrorCommit (see its own comment) to tell the DSP the full bank
// now reflects one complete, consistent edit.

// The mirror's own fixed slot count -- matches EqModel.hpp's own
// documented ceiling (kMaxOrder/4 = 32 bands, "comfortably past FabFilter
// Pro-Q's advertised 24-band maximum") exactly, so nothing achievable
// today ever needs to be truncated to fit.
constexpr uint32_t kNumMirroredBands = uint32_t(ssfilter::kMaxOrder / 4);

enum ParameterIndex
{
    kParamBypass = 0,
    kParamResetTrigger = 1,
    kParamFaultFlag = 2,
    kParamSpectrumOutputBase = 3, // + [0 .. kNumPackedBands) -- post-EQ spectrum, output, dB, kBandsPerPacket bands packed per Parameter (see SpectrumAnalyzer.hpp's own comment)
    kParamSpectrumInputBase = kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands, // + [0 .. kNumPackedBands) -- pre-EQ spectrum, output, dB
    // Appended after the spectrum banks, not slotted in earlier -- same
    // index-stability reasoning as every other "appended, not inserted"
    // parameter elsewhere in this project. UIEq's uiIdle() writes to this
    // once per idle tick purely as a "the editor is currently open"
    // heartbeat -- the actual value is meaningless, only the *fact* and
    // *recency* of a write matter, see kUiHeartbeatTimeoutSamples in
    // PluginEq.cpp. Lets run() skip feeding the spectrum analyzer/analyzers
    // entirely while nobody has a window open to look at the result.
    kParamUiHeartbeat = kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    // Audition -- same idea, same parameter shape, as the sibling Dynamic
    // EQ plugin's own kParamAudition/kParamAuditionBand: preview the
    // selected band's own isolated frequency region (a genuine bandpass
    // tap at that band's freq/Q, not derived from the band's own Type-
    // specific shape, but *with* its own gainDb applied -- see
    // updateAuditionCoeffs()'s own comment) instead of the real output,
    // for finding/tuning by ear. Named "Audition" rather than the
    // originally-shipped "Listen" -- renamed directly: "Listen" invited
    // exactly the wrong expectation ("hear what this band's own EQ
    // curve is doing to the mix" -- gain-dependent, like a solo/bypass
    // A/B), when what it actually always was is "hear the raw frequency
    // region this band's freq/Q names" -- a spot-check/audition tool
    // gain applies to (once fixed, see updateAuditionCoeffs()) but that
    // was never about A/B'ing the curve's own effect on the mix. Appended
    // at the very end rather than inserted among the existing indices
    // above, so no already-assigned parameter shifts.
    kParamAudition = kParamUiHeartbeat + 1,
    kParamAuditionBand = kParamAudition + 1, // which band index to preview; UI keeps this synced to whichever band is selected
    // Two more spectrum banks, same shape as kParamSpectrumOutputBase
    // above -- fed only while at least one active band is Left/Right-only
    // (ssfilter::eq::hasPerChannelBands(), cached per rebuildEngineModel()
    // call as asymmetricChannels_ rather than recomputed every block), so
    // the ordinary "every band is Both" case pays no extra FFT cost at
    // all, same "not fed = no cost" convention kParamSpectrumOutputBase's
    // own spectrumMode_ gating already established. Post-EQ only -- a
    // per-channel *input* split would need the host to actually be
    // feeding genuinely different L/R content in the first place, which
    // this plugin has no way to know or show separately; the meaningful
    // "these two channels now differ" story is entirely about what this
    // plugin's own per-band routing just did to the *output*. See
    // EqGraphWidget's own comment on drawing these two traces instead of
    // (not in addition to) the single combined Output trace.
    kParamSpectrumOutputLBase = kParamAuditionBand + 1,
    kParamSpectrumOutputRBase = kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    // A fixed-size mirror of eqModel_ (see kNumMirroredBands's own comment
    // for the size and PluginEq.cpp's syncMirrorFromModel()/
    // adoptMirrorIfModelEmpty()/applyMirrorToModel() for how it's kept in
    // sync) -- appended last, same "appended, never inserted"
    // index-stability rule as every bank above. Host-facing: a host embeds
    // ordinary Parameter values directly inline in its own saved session
    // (confirmed: Ardour's LV2Plugin::add_state() writes every declared
    // Parameter as a <Port value="..."/> XML element), which is what makes
    // this survive a route/channel-strip import (see this file's own
    // top-of-file comment for the full history). The GUI both reads these
    // (to reconstruct its own band list when an editor opens or reopens --
    // see UIEq::parameterChanged()) and writes them (every edit, via
    // UIEq::pushBandParamMirror()).
    kParamBandCount = kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    kParamOutputTrimDb = kParamBandCount + 1,
    kParamBandTypeBase = kParamOutputTrimDb + 1,          // + [0 .. kNumMirroredBands) -- ssfilter::EqBandType, as an integer
    kParamBandFreqBase = kParamBandTypeBase + kNumMirroredBands,
    kParamBandGainBase = kParamBandFreqBase + kNumMirroredBands,
    kParamBandQBase = kParamBandGainBase + kNumMirroredBands,
    kParamBandEnabledBase = kParamBandQBase + kNumMirroredBands,
    kParamBandSlopeBase = kParamBandEnabledBase + kNumMirroredBands,       // slopeStages, 1..4
    kParamBandChannelBase = kParamBandSlopeBase + kNumMirroredBands,       // ssfilter::EqChannel, as an integer
    kParamBandHarmonicGroupBase = kParamBandChannelBase + kNumMirroredBands,
    kParamBandHarmonicRatioBase = kParamBandHarmonicGroupBase + kNumMirroredBands,
    // ssfilter::EqCrossFeed, as an integer -- added after the mirror bank
    // above already existed (see EqBand::crossFeed's own comment,
    // EqModel.hpp), appended at the end same as every bank before it
    // (this enum's own "appended, never inserted" index-stability rule).
    kParamBandCrossFeedBase = kParamBandHarmonicRatioBase + kNumMirroredBands,
    // EqModel::crossFeedGainLtoRDb/RtoLDb -- the crossfeed transit's own
    // overall/static send level. NOT part of the per-band mirror bank
    // above (there is no "+ kNumMirroredBands" on these two): this is a
    // single value per DIRECTION, covering the whole L->R (or R->L)
    // transit regardless of how many bands route that way, the same
    // "one scalar for the whole model" shape kParamOutputTrimDb already
    // has. See that field's own comment (EqModel.hpp) for why this isn't
    // a per-band value (it briefly was, in an earlier iteration of this
    // feature).
    kParamCrossFeedGainLtoRDb = kParamBandCrossFeedBase + kNumMirroredBands,
    kParamCrossFeedGainRtoLDb = kParamCrossFeedGainLtoRDb + 1,
    // EqBand::bandwidthOct -- a Band Shelf's width in octaves (ignored by
    // every other type). Added after everything above, and before
    // kParamMirrorCommit, which has to stay last.
    kParamBandBandwidthBase = kParamCrossFeedGainRtoLDb + 1,
    // EqBand::solo. Missing from the mirror until 2026-09-25: every host
    // echo of the mirror back into the UI rebuilt the band list with solo
    // false (a click on S turned itself straight back off), and solo never
    // reached the DSP at all. Before kParamMirrorCommit, which stays last.
    kParamBandSoloBase = kParamBandBandwidthBase + kNumMirroredBands,
    // A commit signal, appended last (see this enum's own "appended,
    // never inserted" rule): the GUI fires this immediately after it has
    // finished writing every field of the mirror bank above for one edit
    // (UIEq::pushBandParamMirror(), always called in full -- never a
    // partial/incremental update, see its own comment), so this is always
    // the *last* Parameter of the whole burst to become dirty. DPF/the LV2
    // host deliver all dirty input Parameters to setParameterValue() in
    // ascending index order once per block before run() (confirmed by
    // reading dpf/distrho/src/DistrhoPluginLV2.cpp's "Check for updated
    // parameters" loop), so by the time this fires the mirror bank above
    // is guaranteed complete and self-consistent -- see
    // setParameterValue()'s own case for why that matters: rebuilding the
    // engine from a partially-written mirror mid-burst (up to ~290 fields)
    // would risk audible glitches from a transiently-inconsistent band
    // list.
    //
    // NOT a kParameterIsTrigger -- that was the first version of this and
    // it was broken: DPF only declares the LV2
    // port-props#trigger property in the .ttl and otherwise leaves
    // "reset it to default after each cycle" entirely up to the host
    // (dpf/distrho/src/DistrhoPluginLV2.cpp's own
    // updateParameterOutputsAndTriggers(): "NOTE: host is responsible for
    // auto-updating control port buffers"), and Ardour's own LV2 host
    // (checked directly in its `lv2_plugin.cc`) does not implement that
    // property at all -- it treats the port as an ordinary persistent
    // control. Since UIEq always pushed the same constant 1.0f, and the
    // whole delivery chain (Ardour's own cached control value *and* DPF's
    // per-parameter `fLastControlValues[]` on the plugin side) only
    // redelivers a Parameter whose value actually *changed*, every commit
    // after the very first one in the port's lifetime was silently
    // swallowed as "no change" -- worse, a session save+reload bakes the
    // stuck 1.0 into the saved XML too, so it stays broken for the rest
    // of that instance's life after a reload. Symptom: a session survives close/reopen correctly (the ordinary,
    // genuinely-varying mirror *fields* were never affected by this), but
    // every live edit after that point updated the GUI/graph (driven
    // directly by the UI's own eqModel_) and even kept pushing correct
    // values into the mirror Parameters (feeding the *next* save
    // correctly), while the real-time engine silently never rebuilt again
    // -- visible on the spectrum analyzer as a removed/added band with no
    // audible effect. Fixed the same way kParamUiHeartbeat's identical
    // problem already was (see that Parameter's own comment): the GUI
    // toggles the value it sends every push (UIEq::mirrorCommitToggle_)
    // instead of resending a constant, guaranteeing a real, host-visible
    // change -- and every delivery just gets applied regardless of the
    // actual number, see setParameterValue()'s own case.
    kParamMirrorCommit = kParamBandSoloBase + kNumMirroredBands,
    kParamCount = kParamMirrorCommit + 1
};

enum StateIndex
{
    kStatePresetDir = 0,        // last folder used in Load/Save/Export dialogs (UI convenience only)
    kStateSpectrumMode = 1,     // ssfilter::SpectrumMode (0=Off/1=Input/2=Output/3=Both) -- DSP-relevant, see setState()
    kStateSpectrumSmoothMs = 2, // spectrum display smoothing time constant, ms (UI convenience only)
    kStateYRangeDb = 3,         // graph's +-gain-axis range, dB (UI convenience only)
    // (Two former entries here, "spectrumGainDb" and "eqmodel", are gone --
    // the first once autoscaling made a manual gain control redundant, the
    // second once the band list moved entirely to the Parameter mirror
    // (kParamBandCount et al. in ParameterIndex -- see this file's own
    // top-of-file comment). Renumbering the remaining indices below is
    // safe -- states are addressed by key string for session persistence,
    // never by this enum's numeric value.)
    kStateSpectrumFftSize = 4,  // FFT size preset index (0..kNumSizePresets-1) -- DSP-relevant, see setState()
    kStateSpectrumAutoscale = 5, // spectrum axis autoscale on/off, "0"/"1" (UI convenience only; off/"0" is the default)
    // A/B compare, made to survive a session close/reopen. Pure UI
    // convenience, same
    // "DPF's base Plugin class remembers whatever string was last passed
    // to setState() and hands it back for session persistence, DSP side
    // never reads it" convention as presetDir/spectrumSmoothMs/yRangeDb
    // above -- a plain State, not the same import-safe Parameter-mirror
    // treatment the main band list has, since A/B is a workflow
    // convenience rather than the core audio-shaping data (a deliberate
    // choice, weighed against roughly doubling this plugin's already-large
    // mirrored-Parameter count). See
    // UIEq::pushAbState()/stateChanged() for the read/write side.
    kStateAbSlotA = 6,   // ssfilter::eq::toJson() of whatever model was last active in slot A
    kStateAbSlotB = 7,   // same, slot B
    kStateAbHasA = 8,    // "0"/"1" -- has slot A ever been populated (vs. still empty/never used)
    kStateAbHasB = 9,    // same, slot B
    kStateAbActive = 10, // "0"=A/"1"=B -- which slot is currently the live one
    kStateCount
};

class PluginEq : public Plugin
{
public:
    PluginEq();

protected:
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    // DPF/LV2 calls this once instantiation, and any State/Parameter
    // restore that came with it, is complete -- see
    // adoptMirrorIfModelEmpty()'s own comment for why that makes this
    // exactly the right, and only, place it's called from.
    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    // Re-derives the StateSpaceMatrices from eqModel_ at the current
    // sample rate, runs it through StabilityGuard::enforce(), and pushes
    // the result to engine_. Called from both setState() (a new band list
    // arrived) and sampleRateChanged() (the *same* band list needs
    // re-realizing -- unlike the base plugin's raw matrices, an EqModel's
    // biquad coefficients are sample-rate-dependent by construction, so a
    // rate change alone requires rebuilding them).
    void rebuildEngineModel();

    // Copies eqModel_'s current bands/outputTrimDb into
    // mirrorBandSlots_/mirrorBandCount_/mirrorOutputTrimDb_ -- called at
    // the end of rebuildEngineModel(), so getParameterValue() itself
    // always reports whatever edit just happened. NOT, on its own, what
    // makes any of this survive a host's own
    // session save -- see ParameterIndex's own comment on why that
    // additionally requires UIEq::pushBandParamMirror() to actively ask
    // the host to adopt the new values, the same way every other
    // UI-driven Parameter in this project already does.
    void syncMirrorFromModel();

    // The inverse direction: only ever called from activate()/run() (see
    // those functions' own comments), and only takes effect if eqModel_ is
    // still completely empty at that point -- i.e. this is a freshly
    // restored instance whose mirror Parameters haven't been adopted into
    // the live model yet. Since the band list now lives *only* in the
    // mirror (no more "eqmodel" State), this is the sole way eqModel_ ever
    // gets populated after a session load -- see applyMirrorToModel()'s
    // own comment for the shared core, and kParamMirrorCommit's own
    // comment for the other caller (a live edit, not a restore).
    void adoptMirrorIfModelEmpty();

    // The actual mirror -> eqModel_ copy + engine rebuild, unconditional
    // (no empty-check) -- shared by adoptMirrorIfModelEmpty() (restore
    // path, gated) and setParameterValue()'s kParamMirrorCommit case (live
    // edit path, always applies: the mirror's current content -- possibly
    // zero bands, a legitimate "user deleted everything" state -- is
    // always meant to win once a commit arrives).
    void applyMirrorToModel();

    // Recomputes auditionActiveBands_/auditionCoeffs_/auditionGainLin_ from
    // auditionBand_'s own current freq/Q/gain (and, if it's part of a
    // harmonic group, every other member of that group too -- see this
    // function's own definition) -- called whenever any of that could have
    // changed (rebuildEngineModel(), and setParameterValue()'s
    // kParamAuditionBand case). A no-op cost-wise if auditionBand_ is out
    // of range; run() itself is what actually gates whether the tap runs
    // at all.
    void updateAuditionCoeffs();

    // Mono-mixes and feeds `frames` samples of (left, right) to `analyzer`,
    // chunked through a fixed-size scratch buffer so this stays
    // allocation-free regardless of the host's block size. Shared by both
    // outputSpectrumAnalyzer_ (fed the post-EQ signal) and
    // inputSpectrumAnalyzer_ (fed the pre-EQ signal) -- see run().
    void feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames);

    ssfilter::EqModel eqModel_; // last-set band list, kept around for rebuildEngineModel()

    // The Parameter mirror's own backing store -- see ParameterIndex's own
    // comment. A plain fixed-size array, not std::vector<EqBand> (matching
    // this project's own no-heap-growth convention for real-time-facing
    // fixed banks), indexed directly by getParameterValue()/
    // setParameterValue() for kParamBand*Base + slot regardless of
    // mirrorBandCount_ -- every slot always holds *some* well-defined
    // value (default EqBand() past the real count), so a host restoring
    // these Parameters in any order can never read/write out of bounds or
    // leave a slot undefined.
    ssfilter::EqBand mirrorBandSlots_[kNumMirroredBands];
    int mirrorBandCount_ = 0;
    double mirrorOutputTrimDb_ = 0.0;
    double mirrorCrossFeedGainLtoRDb_ = -60.0, mirrorCrossFeedGainRtoLDb_ = -60.0;
    mutable ssfilter::StateSpaceEngine engine_;

    // Two independent analyzers -- pre-EQ (input) and post-EQ (output) --
    // so the graph can show the spectrum "before and after the filter"
    // simultaneously. spectrumMode_ (kStateSpectrumMode, set via setState())
    // decides which one(s) run() actually feeds each block: an analyzer
    // that isn't selected is simply never fed, so its FFT cost isn't paid
    // at all while hidden -- see SpectrumAnalyzer.hpp's SpectrumMode
    // comment. This plugin's own default is Both, not the shared
    // kDefaultSpectrumMode (Output) every other spectrum-analyzer plugin
    // still starts on -- a choice for this plugin specifically, not
    // changed project-wide (same "own default, not the shared one"
    // precedent PluginDynEq.hpp's identical member already set).
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzer_;
    ssfilter::SpectrumAnalyzer inputSpectrumAnalyzer_;
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::SpectrumMode::Both;

    // Two more, fed only while asymmetricChannels_ is true (see
    // rebuildEngineModel()) -- kParamSpectrumOutputLBase/RBase's own
    // comment for why this is a separate pair rather than reusing
    // outputSpectrumAnalyzer_ itself, and EqGraphWidget's own comment for
    // what the GUI does with them.
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzerL_;
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzerR_;
    bool asymmetricChannels_ = false; // ssfilter::eq::hasPerChannelBands(eqModel_), recomputed once per rebuildEngineModel() rather than every block

    // Samples elapsed since the last kParamUiHeartbeat write, incremented
    // every run() by `frames` and reset to 0 by setParameterValue() --
    // see kParamUiHeartbeat's own comment. uint64_t so this can never wrap
    // (a uint32_t would, after ~24h of continuous 48kHz processing with no
    // UI ever opened, which is a real enough scenario -- an idle plugin
    // instance left in a saved session -- to not risk it). Starts already
    // past the timeout (no assumption that a UI is open until one actually
    // says so), so spectrum analysis stays off from a cold instantiation
    // until an editor is first opened.
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX;

    // Audition -- see kParamAudition's own comment. auditioning_/
    // auditionBand_ are local mirrors getParameterValue() reads back
    // (same convention as bypass_ below).
    //
    // auditionActiveBands_ is the actual set of band indices the tap
    // currently previews: just auditionBand_ on its own, unless it belongs
    // to a harmonic group, in which case every other member of that group
    // is included too (previewing only the one clicked band would be
    // misleading when the
    // whole point of a harmonic twin is that it moves together with it).
    // auditionCoeffs_/auditionGainLin_ are parallel arrays to
    // auditionActiveBands_ (one bandpass + static gain per active band),
    // all three rebuilt together by updateAuditionCoeffs().
    //
    // auditionStateL_/R_ are each active tap's own persistent per-channel
    // delay line (Biquad1State/processDF1(), CoefficientMapper.hpp -- the
    // same small standalone recursion DynamicEqEngine.hpp's own per-band
    // shaper/detector filters use, reused here rather than duplicated), but
    // indexed by *band index* rather than parallel to
    // auditionActiveBands_/auditionCoeffs_ -- sized to eqModel_.bands.size()
    // and left alone across ordinary coefficient recomputes (only grown/
    // reset when the band count itself changes, see updateAuditionCoeffs()
    // and activate()) so a band's own filter memory survives a live freq/Q/
    // gain tweak mid-audition instead of clicking on every edit, and even
    // survives the active *set* changing (e.g. a sibling band added to or
    // removed from the group while auditioning).
    bool auditioning_ = false;
    // Starts at 0, NOT -1: the host's audition_band port also starts at its
    // default of 0, and hosts deliver a parameter only when its value
    // changes -- so selecting the first band (index 0) sends a 0 that never
    // arrives. At -1 the first band could not be auditioned until some
    // other band had been selected first.
    int auditionBand_ = 0;
    std::vector<int> auditionActiveBands_;
    std::vector<ssfilter::BiquadCoeffs> auditionCoeffs_;
    std::vector<double> auditionGainLin_; // one per auditionActiveBands_ entry -- that band's own static gain, linear; see updateAuditionCoeffs()'s own comment
    std::vector<ssfilter::Biquad1State> auditionStateL_, auditionStateR_; // indexed by band index, see above

    bool bypass_ = false;

    DISTRHO_DECLARE_NON_COPYABLE(PluginEq)
};

END_NAMESPACE_DISTRHO
