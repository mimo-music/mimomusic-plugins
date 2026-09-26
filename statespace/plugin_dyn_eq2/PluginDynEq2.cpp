// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginDynEq2.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO

PluginDynEq2::PluginDynEq2()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    engine_.setAuditionBandIndex(auditionBand_); // match the port's default -- see auditionBand_'s own comment

    // dynEqModel_ starts as its default-constructed empty band list ->
    // rebuildEngineModel() resolves that to zero active bands, the DSP
    // engine's own "nothing to do" state -- same safe default every plugin
    // in this project starts on.

    // This plugin's own default resolution is the largest preset (16384),
    // not the shared kDefaultSizeIndex (8192) every other EQ-family plugin
    // starts on -- a choice for Dynamic EQ 2 specifically. Set
    // before sampleRateChanged() below (which calls .prepare() on both
    // analyzers) so it's already the *active* size, not just requested,
    // the moment this constructor returns -- see SpectrumAnalyzer::
    // prepare()'s own comment for why it reads the request rather than
    // resetting to the class's own compiled-in default.
    outputSpectrumAnalyzer_.requestFftSizeIndex(ssfilter::SpectrumAnalyzer::kNumSizePresets - 1);
    inputSpectrumAnalyzer_.requestFftSizeIndex(ssfilter::SpectrumAnalyzer::kNumSizePresets - 1);
    outputSpectrumAnalyzerL_.requestFftSizeIndex(ssfilter::SpectrumAnalyzer::kNumSizePresets - 1);
    outputSpectrumAnalyzerR_.requestFftSizeIndex(ssfilter::SpectrumAnalyzer::kNumSizePresets - 1);

    sampleRateChanged(getSampleRate());
}

void PluginDynEq2::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginDynEq2::initParameter(uint32_t index, Parameter& parameter)
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
    case kParamAudition:
        parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name   = "Audition";
        parameter.symbol = "audition";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamAuditionBand:
        parameter.hints  = kParameterIsAutomatable | kParameterIsInteger;
        parameter.name   = "Audition Band";
        parameter.symbol = "audition_band";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float(ssfilter::kMaxDynEqBands2 - 1);
        break;
    default:
        if (index >= kParamCurrentGainBase && index < kParamCurrentGainBase + uint32_t(ssfilter::kMaxDynEqBands2))
        {
            const uint32_t band = index - kParamCurrentGainBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Band %u Current Gain", band);
            std::snprintf(symbol, sizeof(symbol), "band_%u_current_gain", band);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -72.0f;
            parameter.ranges.max = 24.0f;
        }
        else if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
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
        else if (index >= kParamSpectrumOutputLBase && index < kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t packet = index - kParamSpectrumOutputLBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum Out L Packet %u", packet);
            std::snprintf(symbol, sizeof(symbol), "spectrum_out_l_%u", packet);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamSpectrumOutputRBase && index < kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t packet = index - kParamSpectrumOutputRBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum Out R Packet %u", packet);
            std::snprintf(symbol, sizeof(symbol), "spectrum_out_r_%u", packet);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        // The Parameter mirror -- see ParameterIndex's own comment. All
        // hidden/automatable-only, same convention as plugin_eq/PluginEq.cpp's
        // identical bank -- never surfaced to the GUI.
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
        else if (index == kParamSoloedBand)
        {
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Soloed Band (internal)";
            parameter.symbol = "mirror_soloed_band";
            parameter.ranges.def = -1.0f;
            parameter.ranges.min = -1.0f; // -1 = no solo, DynamicEqModel2::soloedBand's own sentinel
            parameter.ranges.max = float(kNumMirroredBands - 1);
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
            parameter.ranges.min = 20.0f;    // matches UIDynEq2.cpp's own freqHz clamp
            parameter.ranges.max = 20000.0f;
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
        else if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandGainBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_gain", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Gain (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = -24.0f; // matches UIDynEq2.cpp's own gainDb clamp
            parameter.ranges.max = 24.0f;
        }
        else if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandThresholdBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_thresh", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Threshold (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = -24.0f;
            parameter.ranges.min = -72.0f; // matches UIDynEq2.cpp's own thresholdDb clamp
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
            parameter.ranges.min = 1.0f; // matches UIDynEq2.cpp's own ratio clamp
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
            parameter.ranges.min = 1.0f; // matches UIDynEq2.cpp's own ratioUp clamp
            parameter.ranges.max = 20.0f;
        }
        else if (index >= kParamBandRangeBase && index < kParamBandRangeBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandRangeBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_range", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Range (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 5.0f;
            parameter.ranges.min = 0.0f; // matches UIDynEq2.cpp's own rangeDb clamp
            parameter.ranges.max = 40.0f;
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
            parameter.ranges.min = 0.1f; // matches UIDynEq2.cpp's own attackMs clamp
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
            parameter.ranges.min = 5.0f; // matches UIDynEq2.cpp's own releaseMs clamp
            parameter.ranges.max = 2000.0f;
        }
        else if (index >= kParamBandDynamicsEnabledBase && index < kParamBandDynamicsEnabledBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandDynamicsEnabledBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_dynen", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsHidden;
            parameter.name   = "Band Dynamics Enabled (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 1.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 1.0f;
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
        else if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandCrossFeedBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_xfeed", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Crossfeed (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; // EqCrossFeed::None
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 2.0f; // EqCrossFeed::RightToLeft, the last enumerator
        }
        else if (index == kParamCrossFeedGainLtoRDb || index == kParamCrossFeedGainRtoLDb)
        {
            // Model-level, not per-band -- see ParameterIndex's own
            // comment on why these two aren't "+ slot" mirror-bank
            // entries.
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = index == kParamCrossFeedGainLtoRDb ? "Crossfeed Gain L->R (internal)" : "Crossfeed Gain R->L (internal)";
            parameter.symbol = index == kParamCrossFeedGainLtoRDb ? "mirror_xfeed_gain_ltor" : "mirror_xfeed_gain_rtol";
            parameter.ranges.def = -60.0f;
            parameter.ranges.min = -60.0f;
            parameter.ranges.max = 12.0f; // matches UIDynEq2::setCrossFeedGainLtoRDb()/setCrossFeedGainRtoLDb()'s own clamp
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
        else if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandTypeBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_type", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Type (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; // EqBandType::Bell
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 8.0f; // EqBandType::BandShelf, the last enumerator -- this plugin's UI only sets Bell/Low Shelf/High Shelf/Band Shelf, see DynamicEqBand2::type's own comment
        }
        else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandBandwidthBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_bw", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsHidden;
            parameter.name   = "Band Bandwidth (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 2.0f;  // DynamicEqBand2::bandwidthOct's own default
            parameter.ranges.min = 0.05f; // ssfilter::kMinBandwidthOct
            parameter.ranges.max = 10.0f; // ssfilter::kMaxBandwidthOct
        }
        else if (index == kParamMirrorCommit)
        {
            // NOT kParameterIsTrigger -- see kParamMirrorCommit's own
            // comment (PluginDynEq2.hpp) for why.
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

void PluginDynEq2::initState(uint32_t index, State& state)
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
        // Both, not the sibling EQ plugins' Output-only default -- this
        // plugin's own choice specifically, see spectrumMode_'s own
        // comment in the .hpp.
        state.defaultValue = "3"; // ssfilter::SpectrumMode::Both
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
        // The largest preset, not the sibling EQ plugins' 8192 default --
        // see the constructor's requestFftSizeIndex() calls' own comment.
        state.defaultValue = "3"; // index into SpectrumAnalyzer::kFftSizePresets -- 16384
        break;
    case kStateSpectrumAutoscale:
        state.key   = "spectrumAutoscale";
        state.label = "Spectrum Autoscale";
        state.hints = 0; // UI convenience only
        state.defaultValue = "0"; // off by default -- see DynEqGraphWidget2::setSpectrumAutoscale()'s comment
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

float PluginDynEq2::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamBypass:    return bypass_ ? 1.0f : 0.0f;
    case kParamFaultFlag: return engine_.consumeFaultFlag() ? 1.0f : 0.0f;
    case kParamAudition:    return auditioning_ ? 1.0f : 0.0f;
    // Clamped to the declared [0, kMaxDynEqBands2) range for the host's
    // benefit -- internally auditionBand_ can be -1 ("no band selected,
    // nothing to preview"), but that's meaningless to report as a
    // Parameter value; engine_'s own auditionValid check (auditionBandIndex_ <
    // n) already falls back to normal output whenever there are zero
    // active bands, so clamping this readback to 0 changes nothing audible.
    case kParamAuditionBand: return float(std::max(0, auditionBand_));
    default:
        if (index >= kParamCurrentGainBase && index < kParamCurrentGainBase + uint32_t(ssfilter::kMaxDynEqBands2))
            return float(engine_.getCurrentGainDb(int(index - kParamCurrentGainBase)));
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumOutputBase));
        if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return inputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumInputBase));
        if (index >= kParamSpectrumOutputLBase && index < kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzerL_.getPackedTriple(int(index - kParamSpectrumOutputLBase));
        if (index >= kParamSpectrumOutputRBase && index < kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzerR_.getPackedTriple(int(index - kParamSpectrumOutputRBase));
        // The Parameter mirror -- see ParameterIndex's own comment.
        if (index == kParamBandCount) return float(mirrorBandCount_);
        if (index == kParamOutputTrimDb) return float(mirrorOutputTrimDb_);
        if (index == kParamSoloedBand) return float(mirrorSoloedBand_);
        if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandFreqBase].freqHz);
        if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandQBase].q);
        if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandGainBase].gainDb);
        if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandThresholdBase].thresholdDb);
        if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandRatioBase].ratio);
        if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandRatioUpBase].ratioUp);
        if (index >= kParamBandRangeBase && index < kParamBandRangeBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandRangeBase].rangeDb);
        if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandAttackBase].attackMs);
        if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandReleaseBase].releaseMs);
        if (index >= kParamBandDynamicsEnabledBase && index < kParamBandDynamicsEnabledBase + kNumMirroredBands)
            return mirrorBandSlots_[index - kParamBandDynamicsEnabledBase].dynamicsEnabled ? 1.0f : 0.0f;
        if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
            return mirrorBandSlots_[index - kParamBandEnabledBase].enabled ? 1.0f : 0.0f;
        if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
            return float(int(mirrorBandSlots_[index - kParamBandChannelBase].channel));
        if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
            return float(int(mirrorBandSlots_[index - kParamBandCrossFeedBase].crossFeed));
        if (index == kParamCrossFeedGainLtoRDb) return float(mirrorCrossFeedGainLtoRDb_);
        if (index == kParamCrossFeedGainRtoLDb) return float(mirrorCrossFeedGainRtoLDb_);
        if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandHarmonicGroupBase].harmonicGroup);
        if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandHarmonicRatioBase].harmonicRatio);
        if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
            return float(int(mirrorBandSlots_[index - kParamBandTypeBase].type));
        if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandBandwidthBase].bandwidthOct);
        return 0.0f;
    }
}

void PluginDynEq2::setParameterValue(uint32_t index, float value)
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
    case kParamAudition:
        auditioning_ = value >= 0.5f;
        engine_.setAudition(auditioning_);
        break;
    case kParamAuditionBand:
        auditionBand_ = int(value + 0.5f);
        engine_.setAuditionBandIndex(auditionBand_);
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
            mirrorBandCount_ = std::max(0, std::min(int(kNumMirroredBands), int(value + 0.5f)));
        else if (index == kParamOutputTrimDb)
            mirrorOutputTrimDb_ = double(value);
        else if (index == kParamSoloedBand)
            mirrorSoloedBand_ = std::max(-1, std::min(int(kNumMirroredBands) - 1, int(value + (value >= 0 ? 0.5f : -0.5f))));
        else if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandFreqBase].freqHz = double(value);
        else if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandQBase].q = double(value);
        else if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandGainBase].gainDb = double(value);
        else if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandThresholdBase].thresholdDb = double(value);
        else if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandRatioBase].ratio = double(value);
        else if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandRatioUpBase].ratioUp = double(value);
        else if (index >= kParamBandRangeBase && index < kParamBandRangeBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandRangeBase].rangeDb = double(value);
        else if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandAttackBase].attackMs = double(value);
        else if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandReleaseBase].releaseMs = double(value);
        else if (index >= kParamBandDynamicsEnabledBase && index < kParamBandDynamicsEnabledBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandDynamicsEnabledBase].dynamicsEnabled = value >= 0.5f;
        else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandEnabledBase].enabled = value >= 0.5f;
        else if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandChannelBase].channel = ssfilter::EqChannel(std::max(0, std::min(2, int(value + 0.5f))));
        else if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandCrossFeedBase].crossFeed = ssfilter::EqCrossFeed(std::max(0, std::min(2, int(value + 0.5f))));
        else if (index == kParamCrossFeedGainLtoRDb)
            mirrorCrossFeedGainLtoRDb_ = double(value);
        else if (index == kParamCrossFeedGainRtoLDb)
            mirrorCrossFeedGainRtoLDb_ = double(value);
        else if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandHarmonicGroupBase].harmonicGroup = std::max(0, int(value + 0.5f));
        else if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandHarmonicRatioBase].harmonicRatio = double(value);
        else if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandTypeBase].type = ssfilter::EqBandType(std::max(0, std::min(8, int(value + 0.5f))));
        else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandBandwidthBase].bandwidthOct = double(value);
        break;
    }
}

void PluginDynEq2::setState(const char* key, const char* value)
{
    // Same convention as the sibling plugins' setState() -- "presetDir"/
    // "spectrumSmoothMs" are pure UI preferences DPF's base Plugin class
    // remembers on its own; "spectrumFftSize"/"spectrumMode" are DSP-
    // relevant but not continuously-automatable Parameters.
    if (std::strcmp(key, "spectrumFftSize") == 0)
    {
        outputSpectrumAnalyzer_.requestFftSizeIndex(std::atoi(value));
        inputSpectrumAnalyzer_.requestFftSizeIndex(std::atoi(value));
        outputSpectrumAnalyzerL_.requestFftSizeIndex(std::atoi(value));
        outputSpectrumAnalyzerR_.requestFftSizeIndex(std::atoi(value));
        return;
    }
    if (std::strcmp(key, "spectrumMode") == 0)
    {
        spectrumMode_ = ssfilter::clampSpectrumMode(std::atoi(value));
        return;
    }
}

void PluginDynEq2::rebuildEngineModel()
{
    // Every model band gets a slot in the *same* index it has in
    // dynEqModel_.bands -- capped at kMaxDynEqBands2 (the model itself is
    // an unbounded std::vector, see DynamicEqModel2.hpp's own comment;
    // only what reaches the real-time engine is bounded, exactly like
    // EqModel's bands vs. its assembled kMaxOrder-bounded matrix) -- NOT
    // compacted down to only the active/audible ones. A band that's
    // disabled, or muted by another band's solo (see
    // dyneq2::activeBandIndices()), is replaced with an inert copy
    // (dynamicsEnabled=false, gainDb=0.0) at its *own* slot rather than
    // omitted: with gain forced to exactly 0dB, DynamicEqEngine2's own
    // Regalia-Mitra combination (0.5*(1+G) + 0.5*(1-G)*allpass, G=1) is an
    // exact identity regardless of that band's freq/Q, so the audio result
    // is identical to actually removing the band -- but the *index*
    // dynEqModel_.bands[i] <-> rt.bands[i] <-> DynamicEqEngine2's own
    // per-slot getCurrentGainDb(i) stays stable no matter which bands are
    // muted. Compacting used to break that correspondence -- the UI's live
    // per-band gain readback (UIDynEq2::getCurrentGainDb(), and through it
    // DynEqGraphWidget2's live moving curve) is indexed by *model* band
    // index, so with any band disabled/soloed the compacted array silently
    // handed back a different band's (or a stale) reading. This was a
    // real bug -- the audio
    // itself was always correct (a parallel identity's position in a
    // series cascade never changes the result), but the graph's live
    // curve could show the wrong band moving, or the right band not
    // moving, the instant Solo/Enable made the two lists diverge.
    ssfilter::DynamicEqRtModel2 rt;
    rt.numBands = std::min(int(dynEqModel_.bands.size()), ssfilter::kMaxDynEqBands2);
    if (rt.numBands > 0)
    {
        const std::vector<int> active = ssfilter::dyneq2::activeBandIndices(dynEqModel_);
        std::vector<bool> isActive(size_t(rt.numBands), false);
        for (int idx : active)
            if (idx >= 0 && idx < rt.numBands) isActive[size_t(idx)] = true;

        for (int i = 0; i < rt.numBands; ++i)
        {
            rt.bands[i] = dynEqModel_.bands[size_t(i)];
            if (!isActive[size_t(i)])
            {
                rt.bands[i].gainDb = 0.0;
                rt.bands[i].dynamicsEnabled = false;
                // For crossfeed bands specifically, gainDb=0/dynamicsEnabled=false
                // alone is NOT enough to make this slot inert: the
                // Regalia-Mitra identity (G=1 -> shaped output == the
                // source channel's own raw signal, unscaled) still gets
                // ADDED into the destination channel by
                // DynamicEqEngine2::processBlock()'s own crossfeed branch,
                // scaled by the model's own crossFeedGainLtoRDb/RtoLDb --
                // an inactive band would otherwise still leak its own
                // source channel into the other one. Forcing crossFeed
                // back to None here too routes this slot through the
                // ordinary (non-crossfeed) path instead, where the same
                // G=1 identity correctly means "this band's own channel(s)
                // pass through completely unchanged" regardless of
                // whatever `channel` value happens to be left over.
                rt.bands[i].crossFeed = ssfilter::EqCrossFeed::None;
            }
        }
    }
    rt.outputTrimDb = dynEqModel_.outputTrimDb;
    rt.crossFeedGainLtoRDb = dynEqModel_.crossFeedGainLtoRDb;
    rt.crossFeedGainRtoLDb = dynEqModel_.crossFeedGainRtoLDb;

    // Solo + channel-routing silence -- see DynamicEqRtModel2::muteLeft/
    // muteRight's own comment for the full reasoning (soloing a Left-only
    // band passes only the left audio). Same "soloing a disabled band has no effect" guard
    // dyneq2::activeBandIndices() uses internally for its own solo check.
    const int soloedBand = dynEqModel_.soloedBand;
    if (soloedBand >= 0 && soloedBand < int(dynEqModel_.bands.size()) &&
        dynEqModel_.bands[size_t(soloedBand)].enabled)
    {
        const ssfilter::EqCrossFeed cf = dynEqModel_.bands[size_t(soloedBand)].crossFeed;
        if (cf == ssfilter::EqCrossFeed::LeftToRight)
        {
            // A LeftToRight band's own OUTPUT only ever reaches Right (it
            // reads Left but never writes to Left's own output, see
            // DynamicEqBand2::crossFeed's own comment) -- `channel` is
            // stale/irrelevant here (ignored once crossFeed != None), so
            // this branches on crossFeed directly instead of falling
            // through to the ordinary channel-based check below.
            rt.muteLeft = true;
            rt.muteRight = false;
        }
        else if (cf == ssfilter::EqCrossFeed::RightToLeft)
        {
            rt.muteRight = true;
            rt.muteLeft = false;
        }
        else
        {
            const ssfilter::EqChannel ch = dynEqModel_.bands[size_t(soloedBand)].channel;
            rt.muteRight = (ch == ssfilter::EqChannel::Left);  // soloed band never reaches Right -> silence it
            rt.muteLeft  = (ch == ssfilter::EqChannel::Right); // soloed band never reaches Left -> silence it
        }
    }

    engine_.setModel(rt);
    asymmetricChannels_ = ssfilter::dyneq2::hasPerChannelBands(dynEqModel_); // recomputed here, not per-block -- see the field's own comment
    syncMirrorFromModel(); // keep the Parameter mirror current -- see its own comment
}

void PluginDynEq2::syncMirrorFromModel()
{
    const int n = std::min(int(dynEqModel_.bands.size()), int(kNumMirroredBands));
    for (int i = 0; i < n; ++i) mirrorBandSlots_[i] = dynEqModel_.bands[size_t(i)];
    for (int i = n; i < int(kNumMirroredBands); ++i) mirrorBandSlots_[i] = ssfilter::DynamicEqBand2();
    mirrorBandCount_ = n;
    mirrorOutputTrimDb_ = dynEqModel_.outputTrimDb;
    mirrorSoloedBand_ = dynEqModel_.soloedBand;
    mirrorCrossFeedGainLtoRDb_ = dynEqModel_.crossFeedGainLtoRDb;
    mirrorCrossFeedGainRtoLDb_ = dynEqModel_.crossFeedGainRtoLDb;
}

void PluginDynEq2::adoptMirrorIfModelEmpty()
{
    // Only ever adopt once, on a freshly restored instance -- see
    // PluginEq::adoptMirrorIfModelEmpty()'s identical comment.
    if (!dynEqModel_.bands.empty()) return;
    if (mirrorBandCount_ <= 0) return;
    applyMirrorToModel();
}

void PluginDynEq2::applyMirrorToModel()
{
    dynEqModel_.bands.assign(mirrorBandSlots_, mirrorBandSlots_ + mirrorBandCount_);
    dynEqModel_.outputTrimDb = mirrorOutputTrimDb_;
    dynEqModel_.soloedBand = mirrorSoloedBand_;
    dynEqModel_.crossFeedGainLtoRDb = mirrorCrossFeedGainLtoRDb_;
    dynEqModel_.crossFeedGainRtoLDb = mirrorCrossFeedGainRtoLDb_;
    rebuildEngineModel();
}

void PluginDynEq2::activate()
{
    adoptMirrorIfModelEmpty();
    engine_.resetState();
}

void PluginDynEq2::run(const float** inputs, float** outputs, uint32_t frames)
{
    // See PluginEq::run()'s identical call (../plugin_eq/PluginEq.cpp) for
    // why this has to be retried here, not just from activate() -- DPF only
    // delivers a restored input Parameter to our own setParameterValue()
    // from inside run() itself, never synchronously during restore, so
    // activate() alone is provably too early on a cold import-recovered
    // instance (confirmed via diagnostic logging on the sibling stereo EQ).
    adoptMirrorIfModelEmpty();

    // Same in-place-aliasing-safe ordering as PluginEq::run()/
    // PluginMbCompressor::run() -- read the pre-EQ signal before anything
    // (the engine, or the bypass copy) can overwrite it. See PluginEq.cpp's
    // comment for the concrete bug this ordering fixes.
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
    {
        feedSpectrumAnalyzer(outputSpectrumAnalyzer_, outputs[0], outputs[1], frames);

        // The two per-channel analyzers -- see the sibling Parametric EQ's
        // identical comment (PluginEq.cpp's run()) for why passing the
        // same buffer for both of feedSpectrumAnalyzer()'s arguments is
        // exactly a single-channel feed, no separate function needed.
        if (asymmetricChannels_)
        {
            feedSpectrumAnalyzer(outputSpectrumAnalyzerL_, outputs[0], outputs[0], frames);
            feedSpectrumAnalyzer(outputSpectrumAnalyzerR_, outputs[1], outputs[1], frames);
        }
    }
}

void PluginDynEq2::feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames)
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

void PluginDynEq2::sampleRateChanged(double newSampleRate)
{
    engine_.prepare(newSampleRate);
    outputSpectrumAnalyzer_.prepare(newSampleRate);
    inputSpectrumAnalyzer_.prepare(newSampleRate);
    outputSpectrumAnalyzerL_.prepare(newSampleRate);
    outputSpectrumAnalyzerR_.prepare(newSampleRate);
    rebuildEngineModel(); // shaper/detector coefficients are sample-rate-dependent, see the .hpp comment
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginDynEq2(); }
END_NAMESPACE_DISTRHO
