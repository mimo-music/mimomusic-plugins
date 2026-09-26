// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/EqModel.hpp"
#include "../plugin/dsp/MonoStateSpaceEngine.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"

#include <cstdint>

START_NAMESPACE_DISTRHO

// The mono sibling of PluginEq -- see plugin_eq/PluginEq.hpp's own comment
// for the parameter design (identical here, reused verbatim: the actual
// band list travels entirely as the Parameter mirror below via
// ssfilter::EqModel, which has no channel count of its own to speak of,
// only the *assembled* coefficients differ, see rebuildEngineModel()
// below). The one structural difference is engine_'s type and run()'s
// signature: one audio channel in, one out, MonoStateSpaceEngine instead
// of StateSpaceEngine.
//
// kParamBandCount/kParamOutputTrimDb/kParamBand*Base/kParamMirrorCommit at
// the end of this enum are the identical Parameter mirror + commit signal
// PluginEq.hpp's own copy of this comment describes in full -- same root
// cause (Ardour used to persist a DPF State to an external file it
// doesn't copy on a route/channel-strip import, but embeds an ordinary
// Parameter inline; that State is gone now, this mirror is the only
// representation left), same fix, same "hidden, hosted purely for the
// host's own hold of it" convention for the mirror fields themselves,
// ported over unchanged including the UI-side publish requirement
// (UIEqMono::pushBandParamMirror(), the mono counterpart of UIEq's own --
// see PluginEq.hpp's comment on why the push direction has to live there,
// not here).
constexpr uint32_t kNumMirroredBands = uint32_t(ssfilter::kMaxOrder / 4); // matches EqModel.cpp's own shared kMaxBands cap exactly (enforced identically for mono and stereo)

enum ParameterIndex
{
    kParamBypass = 0,
    kParamResetTrigger = 1,
    kParamFaultFlag = 2,
    kParamSpectrumOutputBase = 3, // + [0 .. kNumPackedBands) -- post-EQ spectrum, output, packed triples
    kParamSpectrumInputBase = kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands, // + [0 .. kNumPackedBands) -- pre-EQ spectrum, output, packed triples
    // See PluginEq.hpp's identical parameter's own comment -- same
    // "editor is open" heartbeat, gating whether run() bothers feeding the
    // spectrum analyzer(s) at all.
    kParamUiHeartbeat = kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands,
    kParamBandCount = kParamUiHeartbeat + 1,
    kParamOutputTrimDb = kParamBandCount + 1,
    kParamBandTypeBase = kParamOutputTrimDb + 1,          // + [0 .. kNumMirroredBands) -- ssfilter::EqBandType, as an integer
    kParamBandFreqBase = kParamBandTypeBase + kNumMirroredBands,
    kParamBandGainBase = kParamBandFreqBase + kNumMirroredBands,
    kParamBandQBase = kParamBandGainBase + kNumMirroredBands,
    kParamBandEnabledBase = kParamBandQBase + kNumMirroredBands,
    kParamBandSlopeBase = kParamBandEnabledBase + kNumMirroredBands,       // slopeStages, 1..4
    kParamBandChannelBase = kParamBandSlopeBase + kNumMirroredBands,       // ssfilter::EqChannel -- always Both for mono in practice, mirrored anyway for exact parity with eqModel_/EqBand
    kParamBandHarmonicGroupBase = kParamBandChannelBase + kNumMirroredBands,
    kParamBandHarmonicRatioBase = kParamBandHarmonicGroupBase + kNumMirroredBands,
    kParamBandBandwidthBase = kParamBandHarmonicRatioBase + kNumMirroredBands, // EqBand::bandwidthOct, Band Shelf only -- see PluginEq.hpp's identical bank
    kParamBandSoloBase = kParamBandBandwidthBase + kNumMirroredBands, // EqBand::solo -- see PluginEq.hpp's identical bank for why it's needed
    // See PluginEq.hpp's identical Parameter's own comment -- the GUI
    // fires this immediately after writing every field of the mirror bank
    // above for one edit, guaranteed (by ascending-index delivery order)
    // to arrive last within that burst. NOT a kParameterIsTrigger -- see
    // that comment for the real bug (Ardour doesn't implement
    // the LV2 trigger-port convention, so a constant resent value only
    // ever reaches the DSP once) and why this has to be a plain
    // automatable value the GUI toggles every push instead.
    kParamMirrorCommit = kParamBandSoloBase + kNumMirroredBands,
    kParamCount = kParamMirrorCommit + 1
};

enum StateIndex
{
    kStatePresetDir = 0,        // last folder used in Load/Save/Export dialogs (UI convenience only)
    kStateSpectrumMode = 1,     // ssfilter::SpectrumMode (0=Off/1=Input/2=Output/3=Both) -- DSP-relevant, see setState()
    kStateSpectrumSmoothMs = 2, // spectrum display smoothing time constant, ms (UI convenience only)
    kStateYRangeDb = 3,         // graph's +-gain-axis range, dB (UI convenience only)
    kStateSpectrumFftSize = 4,  // FFT size preset index (0..kNumSizePresets-1) -- DSP-relevant, see setState()
    kStateSpectrumAutoscale = 5, // spectrum axis autoscale on/off, "0"/"1" (UI convenience only; off/"0" is the default)
    // A/B compare, made to survive a session close/reopen -- see the
    // sibling ../plugin_eq/PluginEq.hpp's identical StateIndex comment for
    // the full reasoning. Ported unchanged. (The former "eqmodel" entry
    // that used to sit at index 0 here is gone -- the band list moved
    // entirely to the Parameter mirror above; renumbering below is safe,
    // states are addressed by key string for session persistence, never
    // by this enum's numeric value.)
    kStateAbSlotA = 6,
    kStateAbSlotB = 7,
    kStateAbHasA = 8,
    kStateAbHasB = 9,
    kStateAbActive = 10,
    kStateCount
};

class PluginEqMono : public Plugin
{
public:
    PluginEqMono();

protected:
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    // See PluginEq::activate()'s identical comment -- called after both
    // State and Parameter restore are complete, exactly where
    // adoptMirrorIfModelEmpty() needs to run.
    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    // Re-derives the MonoStateSpaceMatrices from eqModel_ at the current
    // sample rate via ssfilter::eq::assembleMonoStateSpace() (the mono
    // counterpart of PluginEq's assembleStateSpace() call), runs it
    // through MonoStabilityGuard::enforce(), and pushes the result to
    // engine_. Same two call sites as PluginEq's own copy: setState() (a
    // new band list arrived) and sampleRateChanged() (same band list,
    // re-realized at the new rate).
    void rebuildEngineModel();

    // Identical to PluginEq's own trio -- see those functions' own
    // comments (PluginEq.hpp/PluginEq.cpp) for the full reasoning,
    // including why the "publish to the host" direction deliberately does
    // NOT live here.
    void syncMirrorFromModel();
    void adoptMirrorIfModelEmpty();
    void applyMirrorToModel();

    // Feeds `frames` samples to `analyzer` directly -- unlike
    // PluginEq::feedSpectrumAnalyzer(), there's no L/R mix to do first,
    // the signal is already mono. Still chunked through a fixed-size
    // scratch buffer for the same RT-safety reason (allocation-free
    // regardless of host block size), even though here it's really just a
    // pass-through in kChunk-sized pieces. Shared by both
    // outputSpectrumAnalyzer_ and inputSpectrumAnalyzer_ -- see run().
    void feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* mono, uint32_t frames);

    ssfilter::EqModel eqModel_; // last-set band list, kept around for rebuildEngineModel()

    // The Parameter mirror's own backing store -- see ParameterIndex's own
    // comment, and PluginEq.hpp's identical member for the full reasoning.
    ssfilter::EqBand mirrorBandSlots_[kNumMirroredBands];
    int mirrorBandCount_ = 0;
    double mirrorOutputTrimDb_ = 0.0;

    mutable ssfilter::mono::MonoStateSpaceEngine engine_;

    // Two independent analyzers -- pre-EQ (input) and post-EQ (output) --
    // so the graph can show the spectrum "before and after the filter"
    // simultaneously. spectrumMode_ decides which one(s) run() actually
    // feeds each block -- see PluginEq.hpp's identical member for the full
    // reasoning (shared design, mirrored here).
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzer_;
    ssfilter::SpectrumAnalyzer inputSpectrumAnalyzer_;
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::kDefaultSpectrumMode;

    // See PluginEq.hpp's identical member's own comment.
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX;

    bool bypass_ = false;

    DISTRHO_DECLARE_NON_COPYABLE(PluginEqMono)
};

END_NAMESPACE_DISTRHO
