// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginEqMono.hpp"

#include "../plugin/dsp/MonoStabilityGuard.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO

PluginEqMono::PluginEqMono()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    // eqModel_ starts empty (no bands) -> assembleMonoStateSpace() resolves
    // that to an identity passthrough, same safe default as the other two
    // plugins.
    sampleRateChanged(getSampleRate());
}

void PluginEqMono::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupMono;
    Plugin::initAudioPort(input, index, port);
}

void PluginEqMono::initParameter(uint32_t index, Parameter& parameter)
{
    switch (index)
    {
    case kParamBypass:
        parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name   = "Bypass";
        parameter.symbol = "bypass";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamResetTrigger:
        parameter.hints  = kParameterIsAutomatable | kParameterIsTrigger;
        parameter.name   = "Reset State";
        parameter.symbol = "reset_state";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamFaultFlag:
        parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsOutput;
        parameter.name   = "Numerical Fault";
        parameter.symbol = "fault_flag";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamUiHeartbeat:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "UI Active (internal)";
        parameter.symbol = "ui_heartbeat";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    default:
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t packet = index - kParamSpectrumOutputBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum Out Packet %u", packet);
            std::snprintf(symbol, sizeof(symbol), "spectrum_out_%u", packet);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t packet = index - kParamSpectrumInputBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum In Packet %u", packet);
            std::snprintf(symbol, sizeof(symbol), "spectrum_in_%u", packet);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        // The Parameter mirror -- see ParameterIndex's own comment for why
        // this whole bank exists, and PluginEq.cpp's identical block
        // (plugin_eq/PluginEq.cpp) for the per-field reasoning, ported
        // unchanged.
        else if (index == kParamBandCount)
        {
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Count (internal)";
            parameter.symbol = "mirror_band_count";
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = float(kNumMirroredBands);
        }
        else if (index == kParamOutputTrimDb)
        {
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Output Trim (internal)";
            parameter.symbol = "mirror_output_trim_db";
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -24.0f;
            parameter.ranges.max = 24.0f;
        }
        else if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandTypeBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_type", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Type (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 8.0f; // EqBandType::BandShelf, the last enumerator
        }
        else if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandFreqBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_freq", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Freq (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1000.0f;
            parameter.ranges.min = 20.0f;
            parameter.ranges.max = 96000.0f;
        }
        else if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandGainBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_gain", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Gain (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -60.0f;
            parameter.ranges.max = 60.0f;
        }
        else if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandQBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_q", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Q (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 0.05f;
            parameter.ranges.max = 100.0f;
        }
        else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandEnabledBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_enabled", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsHidden;
            parameter.name   = "Band Enabled (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 1.0f;
        }
        else if (index >= kParamBandSlopeBase && index < kParamBandSlopeBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandSlopeBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_slope", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Slope (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 1.0f;
            parameter.ranges.max = 4.0f;
        }
        else if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandChannelBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_channel", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Channel (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 2.0f;
        }
        else if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandHarmonicGroupBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_hgroup", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Harmonic Group (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = float(kNumMirroredBands);
        }
        else if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandHarmonicRatioBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_hratio", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Harmonic Ratio (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 0.001f;
            parameter.ranges.max = 1000.0f;
        }
        else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandBandwidthBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_bw", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Bandwidth (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 2.0f;   // matches EqBand::bandwidthOct's own default
            parameter.ranges.min = 0.05f;  // ssfilter::kMinBandwidthOct
            parameter.ranges.max = 10.0f;  // ssfilter::kMaxBandwidthOct
        }
        else if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandSoloBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_solo", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsHidden;
            parameter.name   = "Band Solo (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 1.0f;
        }
        else if (index == kParamMirrorCommit)
        {
            // NOT kParameterIsTrigger -- see kParamMirrorCommit's own
            // comment (PluginEqMono.hpp) for why.
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Apply Band List (internal)";
            parameter.symbol = "mirror_commit";
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 1.0f;
        }
        break;
    }
}

void PluginEqMono::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStatePresetDir:
        state.key   = "presetDir";
        state.label = "Last Preset Directory";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "";
        break;
    case kStateSpectrumMode:
        state.key   = "spectrumMode";
        state.label = "Spectrum Source (Off/In/Out/Both)";
        state.hints = 0; // DSP-relevant (see setState()), but not host-automatable -- a small discrete choice, not a continuous control
        state.defaultValue = "2"; // ssfilter::SpectrumMode::Output -- matches the old always-on-post-EQ default
        break;
    case kStateSpectrumSmoothMs:
        state.key   = "spectrumSmoothMs";
        state.label = "Spectrum Smoothing (ms)";
        state.hints = 0; // UI convenience only
        state.defaultValue = "150";
        break;
    case kStateYRangeDb:
        state.key   = "yRangeDb";
        state.label = "Graph Gain Axis Range (+-dB)";
        state.hints = 0; // UI convenience only
        state.defaultValue = "12";
        break;
    case kStateSpectrumFftSize:
        state.key   = "spectrumFftSize";
        state.label = "Spectrum FFT Size";
        state.hints = 0; // DSP-relevant (see setState()), but not host-automatable -- a resolution/latency choice, not a continuous control
        state.defaultValue = "2"; // index into SpectrumAnalyzer::kFftSizePresets -- 8192, see that file's default comment
        break;
    case kStateSpectrumAutoscale:
        state.key   = "spectrumAutoscale";
        state.label = "Spectrum Autoscale";
        state.hints = 0; // UI convenience only
        state.defaultValue = "0"; // off by default -- see EqGraphWidget::setSpectrumAutoscale()'s comment
        break;
    // A/B compare, made permanent -- see StateIndex's own comment.
    case kStateAbSlotA:
        state.key = "abSlotA"; state.label = "A/B Slot A"; state.hints = 0; state.defaultValue = "";
        break;
    case kStateAbSlotB:
        state.key = "abSlotB"; state.label = "A/B Slot B"; state.hints = 0; state.defaultValue = "";
        break;
    case kStateAbHasA:
        state.key = "abHasA"; state.label = "A/B Slot A Populated"; state.hints = 0; state.defaultValue = "0";
        break;
    case kStateAbHasB:
        state.key = "abHasB"; state.label = "A/B Slot B Populated"; state.hints = 0; state.defaultValue = "0";
        break;
    case kStateAbActive:
        state.key = "abActive"; state.label = "A/B Active Slot"; state.hints = 0; state.defaultValue = "0";
        break;
    }
}

float PluginEqMono::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamBypass:    return bypass_ ? 1.0f : 0.0f;
    case kParamFaultFlag: return engine_.consumeFaultFlag() ? 1.0f : 0.0f;
    default:
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumOutputBase));
        if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return inputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumInputBase));
        // The Parameter mirror -- see ParameterIndex's own comment.
        if (index == kParamBandCount) return float(mirrorBandCount_);
        if (index == kParamOutputTrimDb) return float(mirrorOutputTrimDb_);
        if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
            return float(int(mirrorBandSlots_[index - kParamBandTypeBase].type));
        if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandFreqBase].freqHz);
        if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandGainBase].gainDb);
        if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandQBase].q);
        if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
            return mirrorBandSlots_[index - kParamBandEnabledBase].enabled ? 1.0f : 0.0f;
        if (index >= kParamBandSlopeBase && index < kParamBandSlopeBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandSlopeBase].slopeStages);
        if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
            return float(int(mirrorBandSlots_[index - kParamBandChannelBase].channel));
        if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandHarmonicGroupBase].harmonicGroup);
        if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandHarmonicRatioBase].harmonicRatio);
        if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandBandwidthBase].bandwidthOct);
        if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
            return mirrorBandSlots_[index - kParamBandSoloBase].solo ? 1.0f : 0.0f;
        return 0.0f;
    }
}

void PluginEqMono::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamBypass:
        bypass_ = value >= 0.5f;
        break;
    case kParamResetTrigger:
        if (value >= 0.5f) engine_.resetState();
        break;
    case kParamUiHeartbeat:
        uiHeartbeatSamplesSinceLast_ = 0;
        break;
    case kParamMirrorCommit:
        // See PluginEq::setParameterValue()'s identical case for the full
        // reasoning -- any delivery at all means "apply now," unconditional,
        // applies whatever the mirror currently holds.
        applyMirrorToModel();
        break;
    default:
        // The Parameter mirror -- see PluginEq::setParameterValue()'s
        // identical block for the full reasoning (order-independent fixed
        // slots, no engine rebuild here on its own -- that only happens
        // once kParamMirrorCommit above fires, or as a restore-time
        // fallback from activate()'s own adoptMirrorIfModelEmpty()).
        if (index == kParamBandCount)
            mirrorBandCount_ = std::max(0, std::min(int(kNumMirroredBands), int(value + 0.5f)));
        else if (index == kParamOutputTrimDb)
            mirrorOutputTrimDb_ = double(value);
        else if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandTypeBase].type = ssfilter::EqBandType(std::max(0, std::min(8, int(value + 0.5f))));
        else if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandFreqBase].freqHz = double(value);
        else if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandGainBase].gainDb = double(value);
        else if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandQBase].q = double(value);
        else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandEnabledBase].enabled = value >= 0.5f;
        else if (index >= kParamBandSlopeBase && index < kParamBandSlopeBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandSlopeBase].slopeStages = std::max(1, std::min(4, int(value + 0.5f)));
        else if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandChannelBase].channel = ssfilter::EqChannel(std::max(0, std::min(2, int(value + 0.5f))));
        else if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandHarmonicGroupBase].harmonicGroup = std::max(0, int(value + 0.5f));
        else if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandHarmonicRatioBase].harmonicRatio = double(value);
        else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandBandwidthBase].bandwidthOct = double(value);
        else if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandSoloBase].solo = value >= 0.5f;
        break;
    }
}

void PluginEqMono::setState(const char* key, const char* value)
{
    // See PluginEq::setState()'s identical comment -- same reasoning
    // applies unchanged: most of these keys are pure UI preferences DPF's
    // base Plugin class persists on its own, "spectrumFftSize"/
    // "spectrumMode" are the DSP-relevant exceptions (reconfigure the
    // analyzers / decide which run at all). The band list itself no
    // longer round-trips through a State at all -- see kParamMirrorCommit.
    if (std::strcmp(key, "spectrumFftSize") == 0)
    {
        outputSpectrumAnalyzer_.requestFftSizeIndex(std::atoi(value));
        inputSpectrumAnalyzer_.requestFftSizeIndex(std::atoi(value));
        return;
    }
    if (std::strcmp(key, "spectrumMode") == 0)
    {
        spectrumMode_ = ssfilter::clampSpectrumMode(std::atoi(value));
        return;
    }
}

void PluginEqMono::rebuildEngineModel()
{
    ssfilter::mono::MonoStateSpaceMatrices assembled = ssfilter::eq::assembleMonoStateSpace(eqModel_, getSampleRate());
    // Defense in depth, same reasoning as PluginEq::rebuildEngineModel():
    // the GUI already enforces stability before an edit is ever
    // serialized, but a rebuild can also be triggered by a sample-rate
    // change alone or by state restored from a hand-edited/foreign/
    // future-version session file, so this is re-validated right before it
    // reaches the real-time engine regardless of who asked for the rebuild.
    ssfilter::mono::MonoStabilityGuard::enforce(assembled);
    engine_.setModel(assembled);
    syncMirrorFromModel(); // keep the Parameter mirror current -- see its own comment
}

void PluginEqMono::syncMirrorFromModel()
{
    const int n = std::min(int(eqModel_.bands.size()), int(kNumMirroredBands));
    for (int i = 0; i < n; ++i) mirrorBandSlots_[i] = eqModel_.bands[size_t(i)];
    for (int i = n; i < int(kNumMirroredBands); ++i) mirrorBandSlots_[i] = ssfilter::EqBand();
    mirrorBandCount_ = n;
    mirrorOutputTrimDb_ = eqModel_.outputTrimDb;
}

void PluginEqMono::adoptMirrorIfModelEmpty()
{
    // Only ever adopt once, on a freshly restored instance -- see
    // PluginEq::adoptMirrorIfModelEmpty()'s identical comment.
    if (!eqModel_.bands.empty()) return;
    if (mirrorBandCount_ <= 0) return;
    applyMirrorToModel();
}

void PluginEqMono::applyMirrorToModel()
{
    eqModel_.bands.assign(mirrorBandSlots_, mirrorBandSlots_ + mirrorBandCount_);
    eqModel_.outputTrimDb = mirrorOutputTrimDb_;
    rebuildEngineModel();
}

void PluginEqMono::activate()
{
    adoptMirrorIfModelEmpty();
    engine_.resetState();
}

void PluginEqMono::run(const float** inputs, float** outputs, uint32_t frames)
{
    // See PluginEq::run()'s identical call (../plugin_eq/PluginEq.cpp) for
    // why this has to be retried here, not just from activate() -- DPF only
    // delivers a restored input Parameter to our own setParameterValue()
    // from inside run() itself, never synchronously during restore, so
    // activate() alone is provably too early on every freshly restored
    // instance (confirmed via diagnostic logging on the sibling stereo EQ).
    adoptMirrorIfModelEmpty();

    // The input analyzer MUST be fed before anything below touches the
    // buffers -- see PluginEq::run()'s identical comment for the full
    // reasoning: this plugin doesn't declare `inPlaceBroken` either, so a
    // host is free to alias `inputs[0]` and `outputs[0]` to the same
    // memory, and engine_.processBlock() below would silently overwrite
    // `inputs[0]` with the post-EQ signal before this ever got to read
    // the genuine pre-EQ one otherwise.
    // See PluginEq::run()'s identical comment/kParamUiHeartbeat's own --
    // skip the analyzer(s) entirely once the editor's been closed long
    // enough that nothing is reading their output.
    constexpr uint64_t kUiHeartbeatTimeoutSamples = 96000; // ~2s at 48kHz -- comfortably longer than any normal idle-tick gap, or a stalled UI thread briefly catching up
    if (uiHeartbeatSamplesSinceLast_ <= UINT64_MAX - frames) uiHeartbeatSamplesSinceLast_ += frames;
    else uiHeartbeatSamplesSinceLast_ = UINT64_MAX;
    const bool uiOpen = uiHeartbeatSamplesSinceLast_ < kUiHeartbeatTimeoutSamples;

    if (uiOpen && ssfilter::spectrumModeWantsInput(spectrumMode_))
        feedSpectrumAnalyzer(inputSpectrumAnalyzer_, inputs[0], frames);

    if (bypass_)
    {
        if (outputs[0] != inputs[0]) std::memcpy(outputs[0], inputs[0], sizeof(float) * frames);
    }
    else
    {
        engine_.processBlock(inputs[0], outputs[0], frames);
    }

    // See PluginEq::run()'s identical comment -- only the analyzer(s)
    // spectrumMode_ selects are fed, so an unselected one's FFT cost isn't
    // paid at all; the input analyzer (above) always sees the true pre-EQ
    // signal, even while bypassed.
    if (uiOpen && ssfilter::spectrumModeWantsOutput(spectrumMode_))
        feedSpectrumAnalyzer(outputSpectrumAnalyzer_, outputs[0], frames);
}

void PluginEqMono::feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* mono, uint32_t frames)
{
    // No L/R mix needed here (the signal is already mono) -- still
    // chunked through a fixed-size scratch buffer for the same RT-safety
    // reason as PluginEq::feedSpectrumAnalyzer(): allocation-free
    // regardless of how large a single host callback's block is.
    constexpr uint32_t kChunk = 4096;
    for (uint32_t off = 0; off < frames; off += kChunk)
    {
        const uint32_t n = std::min(kChunk, frames - off);
        analyzer.process(mono + off, n);
    }
}

void PluginEqMono::sampleRateChanged(double newSampleRate)
{
    engine_.prepare(newSampleRate);
    outputSpectrumAnalyzer_.prepare(newSampleRate);
    inputSpectrumAnalyzer_.prepare(newSampleRate);
    rebuildEngineModel(); // biquad coefficients are sample-rate-dependent, see the .hpp comment
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginEqMono(); }
END_NAMESPACE_DISTRHO
