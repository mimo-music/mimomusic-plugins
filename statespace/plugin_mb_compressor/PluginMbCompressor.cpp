// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginMbCompressor.hpp"

#include "../plugin/dsp/StabilityGuard.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO

PluginMbCompressor::PluginMbCompressor()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    // mbModel_ starts as its default-constructed one-band, transparent
    // compressor (see MbCompressorModel's constructor) -> assembleStateSpace()
    // resolves that to a plain identity passthrough, same safe default
    // every plugin in this project starts on.
    sampleRateChanged(getSampleRate());
}

void PluginMbCompressor::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginMbCompressor::initParameter(uint32_t index, Parameter& parameter)
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
        else if (index >= kParamBandGainBase && index < kParamBandGainBase + uint32_t(ssfilter::kMaxBands))
        {
            const uint32_t band = index - kParamBandGainBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Band %u Gain", band);
            std::snprintf(symbol, sizeof(symbol), "band_%u_gain", band);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; parameter.ranges.min = -60.0f; parameter.ranges.max = 24.0f;
        }
        // The Parameter mirror -- see ParameterIndex's own comment. All
        // hidden/automatable-only, same convention as plugin_eq/PluginEq.cpp's
        // identical bank -- never surfaced to the GUI.
        else if (index == kParamBandCount)
        {
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Count (internal)";
            parameter.symbol = "mirror_band_count";
            parameter.ranges.def = 1.0f; // MbCompressorModel's own default: one band, no splits
            parameter.ranges.min = 1.0f;
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
        else if (index == kParamLrOrder)
        {
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Crossover Order (internal)";
            parameter.symbol = "mirror_lr_order";
            parameter.ranges.def = 4.0f;
            parameter.ranges.min = 2.0f; // matches mb::clampLrOrder()'s own {2,4,6,8} range
            parameter.ranges.max = 8.0f;
        }
        else if (index == kParamSoloedBand)
        {
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Soloed Band (internal)";
            parameter.symbol = "mirror_soloed_band";
            parameter.ranges.def = -1.0f;
            parameter.ranges.min = -1.0f; // -1 = no solo, MbCompressorModel::soloedBand's own sentinel
            parameter.ranges.max = float(kNumMirroredBands - 1);
        }
        else if (index >= kParamCrossoverBase && index < kParamCrossoverBase + (kNumMirroredBands - 1))
        {
            const uint32_t slot = index - kParamCrossoverBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_crossover_%u", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Crossover Freq (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1000.0f;
            parameter.ranges.min = 20.0f;    // matches MbCompressorModel.cpp's own addCrossover()/setCrossoverHz() clamp
            parameter.ranges.max = 20000.0f;
        }
        else if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandThresholdBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_thresh", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Threshold (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = -20.0f;
            parameter.ranges.min = -60.0f; // matches UIMbCompressor.cpp's own thresholdDb clamp
            parameter.ranges.max = 0.0f;
        }
        else if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandRatioBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_ratio", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Ratio (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 4.0f;
            parameter.ranges.min = 1.0f; // matches UIMbCompressor.cpp's own ratio clamp
            parameter.ranges.max = 20.0f;
        }
        else if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandRatioUpBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_ratioup", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Ratio Up (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 1.0f; // matches UIMbCompressor.cpp's own ratioUp clamp
            parameter.ranges.max = 20.0f;
        }
        else if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandAttackBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_attack", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Attack (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 10.0f;
            parameter.ranges.min = 0.1f; // matches UIMbCompressor.cpp's own attackMs clamp
            parameter.ranges.max = 200.0f;
        }
        else if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandReleaseBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_release", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Release (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 100.0f;
            parameter.ranges.min = 5.0f; // matches UIMbCompressor.cpp's own releaseMs clamp
            parameter.ranges.max = 2000.0f;
        }
        else if (index >= kParamBandMakeupBase && index < kParamBandMakeupBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandMakeupBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_makeup", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Makeup (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -24.0f; // matches UIMbCompressor.cpp's own makeupDb clamp
            parameter.ranges.max = 24.0f;
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
        else if (index == kParamMirrorCommit)
        {
            // NOT kParameterIsTrigger -- see kParamMirrorCommit's own
            // comment (PluginMbCompressor.hpp) for why.
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

void PluginMbCompressor::initState(uint32_t index, State& state)
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
        state.hints = 0; // DSP-relevant (see setState()), but not host-automatable -- a small discrete choice
        state.defaultValue = "3"; // ssfilter::SpectrumMode::Both -- this plugin's own requested default, see PluginMbCompressor.hpp's own comment
        break;
    case kStateSpectrumSmoothMs:
        state.key   = "spectrumSmoothMs";
        state.label = "Spectrum Smoothing (ms)";
        state.hints = 0; // UI convenience only
        state.defaultValue = "150";
        break;
    case kStateSpectrumFftSize:
        state.key   = "spectrumFftSize";
        state.label = "Spectrum FFT Size";
        state.hints = 0; // DSP-relevant (see setState()), but not host-automatable
        state.defaultValue = "2"; // index into SpectrumAnalyzer::kFftSizePresets -- 8192
        break;
    case kStateSpectrumAutoscale:
        state.key   = "spectrumAutoscale";
        state.label = "Spectrum Autoscale";
        state.hints = 0; // UI convenience only
        state.defaultValue = "0"; // off by default -- see MultibandGraphWidget::setSpectrumAutoscale()'s comment
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

float PluginMbCompressor::getParameterValue(uint32_t index) const
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
        if (index >= kParamBandGainBase && index < kParamBandGainBase + uint32_t(ssfilter::kMaxBands))
            return float(engine_.getBandGainDb(int(index - kParamBandGainBase)));
        // The Parameter mirror -- see ParameterIndex's own comment.
        if (index == kParamBandCount) return float(mirrorBandCount_);
        if (index == kParamOutputTrimDb) return float(mirrorOutputTrimDb_);
        if (index == kParamLrOrder) return float(mirrorLrOrder_);
        if (index == kParamSoloedBand) return float(mirrorSoloedBand_);
        if (index >= kParamCrossoverBase && index < kParamCrossoverBase + (kNumMirroredBands - 1))
            return float(mirrorCrossoverHz_[index - kParamCrossoverBase]);
        if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandThresholdBase].thresholdDb);
        if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandRatioBase].ratio);
        if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandRatioUpBase].ratioUp);
        if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandAttackBase].attackMs);
        if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandReleaseBase].releaseMs);
        if (index >= kParamBandMakeupBase && index < kParamBandMakeupBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandMakeupBase].makeupDb);
        if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
            return mirrorBandSlots_[index - kParamBandEnabledBase].enabled ? 1.0f : 0.0f;
        return 0.0f;
    }
}

void PluginMbCompressor::setParameterValue(uint32_t index, float value)
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
        // See PluginEq::setParameterValue()'s identical case
        // (plugin_eq/PluginEq.cpp) for the full reasoning -- any delivery
        // at all means "apply now," unconditional, applies whatever the
        // mirror currently holds.
        applyMirrorToModel();
        break;
    default:
        // The Parameter mirror -- see PluginEq::setParameterValue()'s
        // identical block (plugin_eq/PluginEq.cpp) for the full reasoning.
        if (index == kParamBandCount)
        {
            mirrorBandCount_ = std::max(1, std::min(int(kNumMirroredBands), int(value + 0.5f)));
            // A real Parameter value just arrived from the host (this is
            // the one call every pushBandParamMirror() unconditionally
            // makes, so it's a reliable "the mirror has genuinely heard
            // from the host at least once" signal) -- see
            // mirrorReceivedFromHost_'s own comment (PluginMbCompressor.hpp)
            // for why mirrorBandCount_ alone (always >= 1 by construction,
            // even before any host contact) could never serve as that
            // signal on its own.
            mirrorReceivedFromHost_ = true;
        }
        else if (index == kParamOutputTrimDb)
            mirrorOutputTrimDb_ = double(value);
        else if (index == kParamLrOrder)
            mirrorLrOrder_ = ssfilter::mb::clampLrOrder(int(value + 0.5f));
        else if (index == kParamSoloedBand)
            mirrorSoloedBand_ = std::max(-1, std::min(int(kNumMirroredBands) - 1, int(value + (value >= 0 ? 0.5f : -0.5f))));
        else if (index >= kParamCrossoverBase && index < kParamCrossoverBase + (kNumMirroredBands - 1))
            mirrorCrossoverHz_[index - kParamCrossoverBase] = double(value);
        else if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandThresholdBase].thresholdDb = double(value);
        else if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandRatioBase].ratio = double(value);
        else if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandRatioUpBase].ratioUp = double(value);
        else if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandAttackBase].attackMs = double(value);
        else if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandReleaseBase].releaseMs = double(value);
        else if (index >= kParamBandMakeupBase && index < kParamBandMakeupBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandMakeupBase].makeupDb = double(value);
        else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandEnabledBase].enabled = value >= 0.5f;
        break;
    }
}

void PluginMbCompressor::setState(const char* key, const char* value)
{
    // Same convention as PluginEq::setState() -- "presetDir"/
    // "spectrumSmoothMs" are pure UI preferences DPF's base Plugin class
    // remembers on its own; "spectrumFftSize"/"spectrumMode" are DSP-
    // relevant but not continuously-automatable Parameters.
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

void PluginMbCompressor::rebuildEngineModel()
{
    ssfilter::StateSpaceMatrices assembled = ssfilter::mb::assembleStateSpace(mbModel_, getSampleRate());
    // Defense in depth, same reasoning as PluginEq::rebuildEngineModel():
    // the GUI already enforces stability before an edit is ever
    // serialized, but a rebuild can also be triggered by a sample-rate
    // change alone or by state restored from a hand-edited/foreign/
    // future-version session file.
    ssfilter::StabilityGuard::enforce(assembled);
    engine_.setModel(assembled);
    syncMirrorFromModel(); // keep the Parameter mirror current -- see its own comment
}

void PluginMbCompressor::syncMirrorFromModel()
{
    const int n = std::min(int(mbModel_.bands.size()), int(kNumMirroredBands));
    for (int i = 0; i < n; ++i) mirrorBandSlots_[i] = mbModel_.bands[size_t(i)];
    for (int i = n; i < int(kNumMirroredBands); ++i) mirrorBandSlots_[i] = ssfilter::MbBand();
    const int nx = std::min(int(mbModel_.crossoverHz.size()), int(kNumMirroredBands) - 1);
    for (int i = 0; i < nx; ++i) mirrorCrossoverHz_[i] = mbModel_.crossoverHz[size_t(i)];
    for (int i = nx; i < int(kNumMirroredBands) - 1; ++i) mirrorCrossoverHz_[i] = 0.0;
    mirrorBandCount_ = n;
    mirrorOutputTrimDb_ = mbModel_.outputTrimDb;
    mirrorLrOrder_ = mbModel_.lrOrder;
    mirrorSoloedBand_ = mbModel_.soloedBand;
}

void PluginMbCompressor::adoptMirrorIfModelEmpty()
{
    // mirrorBandCount_ alone (the sibling EQ plugins' own equivalent
    // check) can't tell "the constructor's own self-primed default" apart
    // from "genuine data, import-restored or otherwise" here -- it's
    // always >= 1 regardless. mirrorReceivedFromHost_ is the real signal;
    // see its own comment (PluginMbCompressor.hpp).
    if (!mirrorReceivedFromHost_) return;
    if (mirrorAdopted_) return;
    mirrorAdopted_ = true;
    applyMirrorToModel();
}

void PluginMbCompressor::applyMirrorToModel()
{
    mbModel_.bands.assign(mirrorBandSlots_, mirrorBandSlots_ + mirrorBandCount_);
    mbModel_.crossoverHz.assign(mirrorCrossoverHz_, mirrorCrossoverHz_ + std::max(0, mirrorBandCount_ - 1));
    mbModel_.outputTrimDb = mirrorOutputTrimDb_;
    mbModel_.lrOrder = ssfilter::mb::clampLrOrder(mirrorLrOrder_);
    mbModel_.soloedBand = mirrorSoloedBand_;
    rebuildEngineModel();
}

void PluginMbCompressor::activate()
{
    adoptMirrorIfModelEmpty();
    engine_.resetState();
}

void PluginMbCompressor::run(const float** inputs, float** outputs, uint32_t frames)
{
    // See PluginEq::run()'s identical call (../plugin_eq/PluginEq.cpp) for
    // why this has to be retried here, not just from activate() -- DPF only
    // delivers a restored input Parameter to our own setParameterValue()
    // from inside run() itself, never synchronously during restore, so
    // activate() alone is provably too early on a cold import-recovered
    // instance (confirmed via diagnostic logging on the sibling stereo EQ).
    adoptMirrorIfModelEmpty();

    // Same in-place-aliasing-safe ordering as PluginEq::run() -- read the
    // pre-compression signal before anything (the engine, or the bypass
    // copy) can overwrite it. See that file's comment for the concrete
    // bug this ordering fixes (a host connecting the same buffer to an
    // input and output port, which this plugin's LV2 metadata doesn't
    // declare `inPlaceBroken` against).
    // See PluginEq::run()'s identical comment/kParamUiHeartbeat's own --
    // skip the analyzer(s) entirely once the editor's been closed long
    // enough that nothing is reading their output.
    constexpr uint64_t kUiHeartbeatTimeoutSamples = 96000; // ~2s at 48kHz -- comfortably longer than any normal idle-tick gap, or a stalled UI thread briefly catching up
    if (uiHeartbeatSamplesSinceLast_ <= UINT64_MAX - frames) uiHeartbeatSamplesSinceLast_ += frames;
    else uiHeartbeatSamplesSinceLast_ = UINT64_MAX;
    const bool uiOpen = uiHeartbeatSamplesSinceLast_ < kUiHeartbeatTimeoutSamples;

    if (uiOpen && ssfilter::spectrumModeWantsInput(spectrumMode_))
        feedSpectrumAnalyzer(inputSpectrumAnalyzer_, inputs[0], inputs[1], frames);

    if (bypass_)
    {
        if (outputs[0] != inputs[0]) std::memcpy(outputs[0], inputs[0], sizeof(float) * frames);
        if (outputs[1] != inputs[1]) std::memcpy(outputs[1], inputs[1], sizeof(float) * frames);
    }
    else
    {
        engine_.processBlock(inputs[0], inputs[1], outputs[0], outputs[1], frames);
    }

    if (uiOpen && ssfilter::spectrumModeWantsOutput(spectrumMode_))
        feedSpectrumAnalyzer(outputSpectrumAnalyzer_, outputs[0], outputs[1], frames);
}

void PluginMbCompressor::feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames)
{
    constexpr uint32_t kChunk = 4096;
    float mono[kChunk];
    for (uint32_t off = 0; off < frames; off += kChunk)
    {
        const uint32_t n = std::min(kChunk, frames - off);
        for (uint32_t i = 0; i < n; ++i)
            mono[i] = 0.5f * (left[off + i] + right[off + i]);
        analyzer.process(mono, n);
    }
}

void PluginMbCompressor::sampleRateChanged(double newSampleRate)
{
    engine_.prepare(newSampleRate);
    outputSpectrumAnalyzer_.prepare(newSampleRate);
    inputSpectrumAnalyzer_.prepare(newSampleRate);
    rebuildEngineModel(); // crossover coefficients are sample-rate-dependent, see the .hpp comment
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginMbCompressor(); }
END_NAMESPACE_DISTRHO
