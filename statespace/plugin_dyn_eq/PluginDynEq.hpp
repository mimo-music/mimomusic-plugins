// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/DynamicEqEngine.hpp"
#include "../plugin/dsp/DynamicEqModel.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"

#include <cstdint>

START_NAMESPACE_DISTRHO

// kParamCurrentGainBase is this plugin's own
// addition: one read-only output Parameter per possible band slot, carrying
// DynamicEqEngine::getCurrentGainDb() across to the UI side every block --
// necessary (not just a nicety) because a DPF plugin's DSP and UI can end
// up in separate shared objects/processes with no direct memory link (see
// PluginEq's own spectrum-parameter comment for the identical reasoning),
// so the UI cannot simply call the engine object's getter itself. This is
// what lets gui/dyneq/UIDynEq draw the live, moving per-band curve, not
// just the resting/static one.

// Fixed slot count for kParamBand*Base below -- matches kMaxDynEqBands
// (DynamicEqModel.hpp), the real, already-enforced ceiling
// rebuildEngineModel() itself caps `rt.numBands` at, so nothing achievable
// today needs truncating.
constexpr uint32_t kNumMirroredBands = uint32_t(ssfilter::kMaxDynEqBands);

enum ParameterIndex
{
    kParamBypass = 0,
    kParamResetTrigger = 1,
    kParamFaultFlag = 2,
    kParamCurrentGainBase = 3, // + [0 .. kMaxDynEqBands) -- live per-band dynamic gain, dB, output
    kParamSpectrumOutputBase = kParamCurrentGainBase + ssfilter::kMaxDynEqBands,
    kParamSpectrumInputBase = kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    // See plugin_eq/PluginEq.hpp's identical parameter's own comment --
    // same "editor is open" heartbeat, gating whether run() bothers
    // feeding the spectrum analyzer(s) at all.
    kParamUiHeartbeat = kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    // Audition -- same idea as the (now-deleted) De-esser plugin's own
    // "Listen" (renamed here, and on the sibling Parametric EQ plugin
    // which later copied this feature -- "Listen" invited the wrong
    // expectation, hearing what the band's own EQ curve does to the
    // mix, when what it always actually was is "hear the raw frequency
    // region this band's own detector is centered on", scaled by its
    // live dynamic gain), generalized to "which band": preview the
    // selected band's isolated detector output instead of the real
    // output, for tuning by ear. Appended at the very end rather than
    // inserted among the existing indices above, so no already-assigned
    // parameter (or an existing host automation lane referencing one by
    // index) shifts.
    kParamAudition = kParamUiHeartbeat + 1,
    kParamAuditionBand = kParamAudition + 1, // which band index to preview; UI keeps this synced to whichever band is selected
    // Two more spectrum banks, same shape/reasoning as the sibling
    // Parametric EQ's identical pair (PluginEq.hpp's own
    // kParamSpectrumOutputLBase/RBase comment) -- fed only while at least
    // one active band is Left/Right-only (asymmetricChannels_, cached per
    // rebuildEngineModel() rather than recomputed every block).
    kParamSpectrumOutputLBase = kParamAuditionBand + 1,
    kParamSpectrumOutputRBase = kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    // Parameter mirror of dynEqModel_ -- see plugin_eq/PluginEq.hpp's
    // ParameterIndex comment for the full root cause and design (Ardour
    // used to persist a DPF State to an external file a route/channel-
    // strip import never copies, but embeds an ordinary Parameter inline;
    // that State is gone now, this mirror is the only representation
    // left). One bank per DynamicEqBand field, kNumMirroredBands slots
    // each; the publish direction lives in UIDynEq::pushBandParamMirror(),
    // not here -- same "only a UI-initiated setParameterValue() call gets
    // a host to persist it" reasoning.
    kParamBandCount = kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    kParamOutputTrimDb = kParamBandCount + 1,
    kParamSoloedBand = kParamOutputTrimDb + 1, // -1 = no solo, matching DynamicEqModel::soloedBand's own sentinel
    kParamBandFreqBase = kParamSoloedBand + 1,          // + [0 .. kNumMirroredBands)
    kParamBandQBase = kParamBandFreqBase + kNumMirroredBands,
    kParamBandGainBase = kParamBandQBase + kNumMirroredBands,
    kParamBandThresholdBase = kParamBandGainBase + kNumMirroredBands,
    kParamBandRatioBase = kParamBandThresholdBase + kNumMirroredBands,
    kParamBandRatioUpBase = kParamBandRatioBase + kNumMirroredBands,
    kParamBandRangeBase = kParamBandRatioUpBase + kNumMirroredBands,
    kParamBandAttackBase = kParamBandRangeBase + kNumMirroredBands,
    kParamBandReleaseBase = kParamBandAttackBase + kNumMirroredBands,
    kParamBandDynamicsEnabledBase = kParamBandReleaseBase + kNumMirroredBands,
    kParamBandEnabledBase = kParamBandDynamicsEnabledBase + kNumMirroredBands,
    kParamBandChannelBase = kParamBandEnabledBase + kNumMirroredBands,
    // See plugin_eq/PluginEq.hpp's identical kParamMirrorCommit comment --
    // the GUI fires this immediately after writing every field of the
    // mirror bank above for one edit, guaranteed to arrive last within
    // that burst. NOT a kParameterIsTrigger -- see that comment for the
    // real bug (Ardour doesn't implement the LV2 trigger-port
    // convention, so a constant resent value only ever reaches the DSP
    // once) and why this has to be a plain automatable value the GUI
    // toggles every push instead.
    kParamMirrorCommit = kParamBandChannelBase + kNumMirroredBands,
    kParamCount = kParamMirrorCommit + 1
};

enum StateIndex
{
    kStatePresetDir = 0,        // last folder used in Load/Save/Export dialogs (UI convenience only)
    kStateSpectrumMode = 1,     // ssfilter::SpectrumMode (0=Off/1=Input/2=Output/3=Both) -- DSP-relevant, see setState()
    kStateSpectrumSmoothMs = 2, // spectrum display smoothing time constant, ms (UI convenience only)
    kStateSpectrumFftSize = 3,  // FFT size preset index (0..kNumSizePresets-1) -- DSP-relevant, see setState()
    kStateSpectrumAutoscale = 4, // spectrum axis autoscale on/off, "0"/"1" (UI convenience only; off/"0" is the default)
    // A/B compare, made to survive a session close/reopen -- see the
    // sibling ../plugin_eq/PluginEq.hpp's identical StateIndex comment for
    // the full reasoning. Ported unchanged. (The former "dyneqmodel" entry
    // that used to sit at index 0 here is gone -- the band list moved
    // entirely to the Parameter mirror above; renumbering below is safe.)
    kStateAbSlotA = 5,
    kStateAbSlotB = 6,
    kStateAbHasA = 7,
    kStateAbHasB = 8,
    kStateAbActive = 9,
    kStateCount
};

class PluginDynEq : public Plugin
{
public:
    PluginDynEq();

protected:
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    // See PluginEq::activate()'s identical comment (plugin_eq/PluginEq.hpp)
    // -- called after both State and Parameter restore are complete,
    // exactly where adoptMirrorIfModelEmpty() needs to run.
    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    // Resolves dynEqModel_ to its active/audible band list (see
    // dyneq::activeBandIndices()) at the current sample rate and pushes it
    // to engine_ -- same two call sites (setState(), a new model arrived;
    // sampleRateChanged(), the *same* model needs re-realizing since
    // freqHz-based coefficients are sample-rate-dependent) as PluginEq's/
    // PluginMbCompressor's identical rebuildEngineModel().
    void rebuildEngineModel();

    // Identical trio to PluginEq's own (plugin_eq/PluginEq.hpp/.cpp) -- see
    // those functions' own comments for the full reasoning, including why
    // the "publish to the host" direction deliberately does NOT live here.
    void syncMirrorFromModel();
    void adoptMirrorIfModelEmpty();
    void applyMirrorToModel();

    void feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames);

    ssfilter::DynamicEqModel dynEqModel_;

    // The Parameter mirror's own backing store -- see ParameterIndex's own
    // comment.
    ssfilter::DynamicEqBand mirrorBandSlots_[kNumMirroredBands];
    int mirrorBandCount_ = 0;
    double mirrorOutputTrimDb_ = 0.0;
    int mirrorSoloedBand_ = -1;
    mutable ssfilter::DynamicEqEngine engine_;

    // Two independent analyzers -- pre-EQ (input) and post-EQ (output) --
    // identical mechanism to PluginEq's/PluginMbCompressor's own. Unlike
    // those two, this plugin's own default is Both (not the shared
    // kDefaultSpectrumMode = Output) and the largest FFT preset (see the
    // constructor's requestFftSizeIndex() calls, and initState()'s own
    // "3" defaultValue strings for both) -- a choice for this plugin
    // specifically, not changed project-wide.
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzer_;
    ssfilter::SpectrumAnalyzer inputSpectrumAnalyzer_;
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::SpectrumMode::Both;

    // Two more, fed only while asymmetricChannels_ is true -- same idiom
    // as the sibling Parametric EQ's identical pair, see
    // kParamSpectrumOutputLBase/RBase's own comment.
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzerL_;
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzerR_;
    bool asymmetricChannels_ = false; // ssfilter::dyneq::hasPerChannelBands(dynEqModel_), recomputed once per rebuildEngineModel() call

    // See plugin_eq/PluginEq.hpp's identical member's own comment.
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX;

    // Local mirrors of kParamAudition/kParamAuditionBand -- getParameterValue()
    // reads these back rather than querying engine_ (same convention as
    // bypass_ above), setParameterValue() forwards both straight to
    // engine_.setAudition()/setAuditionBandIndex().
    bool auditioning_ = false;
    // Starts at 0, NOT -1: the host's audition_band port also starts at its
    // default of 0, and hosts deliver a parameter only when its value
    // changes -- so selecting the first band (index 0) sends a 0 that never
    // arrives. At -1 the first band could not be auditioned until some
    // other band had been selected first.
    int auditionBand_ = 0;

    bool bypass_ = false;

    DISTRHO_DECLARE_NON_COPYABLE(PluginDynEq)
};

END_NAMESPACE_DISTRHO
