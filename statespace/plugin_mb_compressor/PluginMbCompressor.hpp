// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/MbCompressorModel.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../plugin/dsp/StateSpaceEngine.hpp"

#include <cstdint>

START_NAMESPACE_DISTRHO

// The spectrum output-parameter banks are the identical mechanism
// PluginEq already uses -- see that file's own comment for why (no direct
// shared-memory link between the DSP and UI sides in general).
//
// kParamBandCount/kParamOutputTrimDb/kParamLrOrder/kParamSoloedBand/
// kParamCrossoverBase/kParamBand*Base/kParamMirrorCommit at the end of
// this enum are the crossover list + per-band compressor settings,
// carried entirely as this Parameter mirror -- see
// plugin_eq/PluginEq.hpp's own comment for the full root cause and design
// (Ardour used to persist a DPF State to an external file a route/
// channel-strip import never copies, but embeds an ordinary Parameter
// inline; that State is gone now, this mirror is the only representation
// left). The publish direction lives in UIMbCompressor::pushBandParamMirror(),
// not here.
constexpr uint32_t kNumMirroredBands = uint32_t(ssfilter::kMaxBands); // matches MbCompressorModel.cpp's own addCrossover() refusal cap exactly

enum ParameterIndex
{
    kParamBypass = 0,
    kParamResetTrigger = 1,
    kParamFaultFlag = 2,
    kParamSpectrumOutputBase = 3, // + [0 .. kNumPackedBands) -- post-compression spectrum, output, packed triples
    kParamSpectrumInputBase = kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands, // + [0 .. kNumPackedBands) -- pre-compression spectrum, output, packed triples
    // Live per-band gain readback, one slot per possible crossover band
    // (kMaxBands) -- "show gain reduction (and, at rest, makeup gain)
    // graphically and numerically" needs the engine's own live number,
    // the same DSP<->UI-process-boundary reason PluginDynEq's
    // kParamCurrentGainBase exists (see that file's own comment).
    kParamBandGainBase = kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    // See plugin_eq/PluginEq.hpp's identical parameter's own comment --
    // same "editor is open" heartbeat, gating whether run() bothers
    // feeding the spectrum analyzer(s) at all.
    kParamUiHeartbeat = kParamBandGainBase + ssfilter::kMaxBands,
    kParamBandCount = kParamUiHeartbeat + 1,
    kParamOutputTrimDb = kParamBandCount + 1,
    kParamLrOrder = kParamOutputTrimDb + 1,       // even, 2..8 -- see mb::clampLrOrder()
    kParamSoloedBand = kParamLrOrder + 1,         // -1 = no solo, matching MbCompressorModel::soloedBand's own sentinel
    kParamCrossoverBase = kParamSoloedBand + 1,   // + [0 .. kNumMirroredBands - 1) -- crossoverHz.size() == bands.size() - 1 always
    kParamBandThresholdBase = kParamCrossoverBase + (kNumMirroredBands - 1),
    kParamBandRatioBase = kParamBandThresholdBase + kNumMirroredBands,
    kParamBandRatioUpBase = kParamBandRatioBase + kNumMirroredBands,
    kParamBandAttackBase = kParamBandRatioUpBase + kNumMirroredBands,
    kParamBandReleaseBase = kParamBandAttackBase + kNumMirroredBands,
    kParamBandMakeupBase = kParamBandReleaseBase + kNumMirroredBands,
    kParamBandEnabledBase = kParamBandMakeupBase + kNumMirroredBands,
    // See plugin_eq/PluginEq.hpp's identical kParamMirrorCommit comment --
    // the GUI fires this immediately after writing every field of the
    // mirror bank above for one edit, guaranteed to arrive last within
    // that burst. NOT a kParameterIsTrigger -- see that comment for the
    // real bug (Ardour doesn't implement the LV2 trigger-port
    // convention, so a constant resent value only ever reaches the DSP
    // once) and why this has to be a plain automatable value the GUI
    // toggles every push instead.
    kParamMirrorCommit = kParamBandEnabledBase + kNumMirroredBands,
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
    // the full reasoning. Ported unchanged. (The former "mbmodel" entry
    // that used to sit at index 0 here is gone -- the crossover + band
    // list moved entirely to the Parameter mirror above; renumbering
    // below is safe.)
    kStateAbSlotA = 5,
    kStateAbSlotB = 6,
    kStateAbHasA = 7,
    kStateAbHasB = 8,
    kStateAbActive = 9,
    kStateCount
};

class PluginMbCompressor : public Plugin
{
public:
    PluginMbCompressor();

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
    // Re-derives the StateSpaceMatrices from mbModel_ at the current
    // sample rate, runs it through StabilityGuard::enforce(), and pushes
    // the result to engine_ -- same two call sites (setState(), a new
    // model arrived; sampleRateChanged(), the *same* model needs
    // re-realizing since crossover frequencies are Hz-based) as PluginEq's
    // identical rebuildEngineModel().
    void rebuildEngineModel();

    // Identical trio to PluginEq's own (plugin_eq/PluginEq.hpp/.cpp) -- see
    // those functions' own comments for the full reasoning, including why
    // the "publish to the host" direction deliberately does NOT live here.
    // One real difference from the sibling EQ plugins' own copy of this
    // trio, though: MbCompressorModel's *default* constructor already
    // creates one band (`bands.emplace_back()`, "a transparent single-band
    // compressor by default") -- unlike EqModel/DynamicEqModel, which
    // genuinely start with zero bands -- so `mbModel_.bands.empty()` can
    // never be true here and can't be used as the "has this instance
    // adopted its mirror yet" signal the sibling plugins rely on.
    void syncMirrorFromModel();
    void adoptMirrorIfModelEmpty();
    void applyMirrorToModel();
    // True once setParameterValue() has seen a genuine, host-driven
    // kParamBandCount arrive (see that call site's own comment) --
    // mirrorBandCount_ itself can't serve this purpose, since it's always
    // >= 1 from the constructor onward, real host contact or not. Guards
    // adoptMirrorIfModelEmpty() against "adopting" the constructor's own
    // self-primed default before any real data (import-restored or
    // otherwise) has had a chance to land.
    bool mirrorReceivedFromHost_ = false;
    // True once adoptMirrorIfModelEmpty() has performed its one-time
    // adopt -- without this, every subsequent run() call would keep
    // re-adopting the mirror forever on a freshly restored instance,
    // clobbering any live edit made in the meantime.
    bool mirrorAdopted_ = false;

    void feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames);

    ssfilter::MbCompressorModel mbModel_;

    // The Parameter mirror's own backing store -- see ParameterIndex's own
    // comment. crossoverHz has one fewer slot than the bands themselves
    // (crossoverHz.size() == bands.size() - 1 always).
    ssfilter::MbBand mirrorBandSlots_[kNumMirroredBands];
    double mirrorCrossoverHz_[kNumMirroredBands - 1];
    int mirrorBandCount_ = 0;
    double mirrorOutputTrimDb_ = 0.0;
    int mirrorLrOrder_ = 4;
    int mirrorSoloedBand_ = -1;

    mutable ssfilter::StateSpaceEngine engine_;

    // Two independent analyzers -- pre-compression (input) and post-
    // compression (output) -- identical mechanism to PluginEq's own, see
    // that file's comment for the full reasoning (before/after comparison,
    // "Off" genuinely pays no FFT cost).
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzer_;
    ssfilter::SpectrumAnalyzer inputSpectrumAnalyzer_;
    // This plugin's own default is Both, not the shared kDefaultSpectrumMode
    // (Output) -- a choice for this plugin specifically, same
    // "own default, not the shared one" precedent the sibling Parametric/
    // Dynamic EQ plugins' own identical members already set.
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::SpectrumMode::Both;

    // See plugin_eq/PluginEq.hpp's identical member's own comment.
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX;

    bool bypass_ = false;

    DISTRHO_DECLARE_NON_COPYABLE(PluginMbCompressor)
};

END_NAMESPACE_DISTRHO
