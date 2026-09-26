// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginEq.hpp"

#include "../plugin/dsp/StabilityGuard.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO

PluginEq::PluginEq()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    // eqModel_ starts empty (no bands) -> assembleStateSpace() resolves
    // that to an identity passthrough, same safe default the base plugin
    // starts on.
    sampleRateChanged(getSampleRate());
}

void PluginEq::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginEq::initParameter(uint32_t index, Parameter& parameter)
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
        // Not a real control -- see this enum's own comment. Automatable
        // so it uses the exact same, already-proven-reliable UI->DSP
        // delivery path every other parameter in this project does,
        // rather than an untested new plumbing mechanism; a host listing
        // it in an automation picker as an oddly-named, inert entry is a
        // small, acceptable cosmetic cost for that reliability.
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
        // Same shape as the sibling Dynamic EQ plugin's identical
        // parameter -- automatable (the UI just keeps it synced to
        // whichever band is selected, same as that plugin's own).
        parameter.hints  = kParameterIsAutomatable | kParameterIsInteger;
        parameter.name   = "Audition Band";
        parameter.symbol = "audition_band";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float(ssfilter::kMaxOrder / 4 - 1); // same band-count cap UIEq.cpp's own addBand()/duplicateBand() enforce
        break;
    default:
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamSpectrumOutputBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum Out Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "spectrum_out_%u", band);
            // Packed (kBandsPerPacket real bands per Parameter) -- see
            // SpectrumAnalyzer.hpp's own comment. An exact integer count,
            // not a dB value -- kParameterIsInteger tells a host not to
            // interpolate/smooth automation on it, which would corrupt
            // the packed bit pattern.
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamSpectrumInputBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum In Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "spectrum_in_%u", band);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamSpectrumOutputLBase && index < kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamSpectrumOutputLBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum Out L Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "spectrum_out_l_%u", band);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamSpectrumOutputRBase && index < kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamSpectrumOutputRBase;
            char name[32], symbol[24];
            std::snprintf(name, sizeof(name), "Spectrum Out R Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "spectrum_out_r_%u", band);
            parameter.hints  = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name   = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = ssfilter::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index == kParamBandCount)
        {
            // See ParameterIndex's own comment for why this whole bank
            // exists. kParameterIsHidden: "still saved and handled as any
            // regular parameter, just not visible to the user" (DPF's own
            // doc) -- exactly what's wanted here, a host-restorable value
            // that never clutters an automation-lane picker or generic
            // plugin GUI, same as every parameter in this bank below.
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
            parameter.ranges.min = -24.0f; // matches UIEq.cpp's own outputTrimDb clamp
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
            parameter.ranges.def = 0.0f; // EqBandType::Bell
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
            parameter.ranges.def = 1000.0f; // matches EqBand::freqHz's own default
            parameter.ranges.min = 20.0f;   // matches UIEq.cpp's own freqHz clamp
            parameter.ranges.max = 96000.0f; // generous upper bound covering sample rates up to ~192kHz; rebuildEngineModel()'s own eq::assembleStateSpace() clamps further to sampleRate*0.499
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
            parameter.ranges.min = -60.0f; // matches UIEq.cpp's own gainDb clamp
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
            parameter.ranges.min = 0.05f; // matches UIEq.cpp's own q clamp
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
            parameter.ranges.max = 4.0f; // matches EqBand::slopeStages's own 1..4 range
        }
        else if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandChannelBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_channel", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Channel (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; // EqChannel::Both
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = 2.0f; // EqChannel::Right, the last enumerator
        }
        else if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
        {
            const uint32_t slot = index - kParamBandHarmonicGroupBase;
            char symbol[24];
            std::snprintf(symbol, sizeof(symbol), "mirror_band_%u_hgroup", slot);
            parameter.hints  = kParameterIsAutomatable | kParameterIsInteger | kParameterIsHidden;
            parameter.name   = "Band Harmonic Group (internal)";
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f; // 0 = not linked to anything
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = float(kNumMirroredBands); // a distinct group id per band can never exceed the band count, see EqModel.cpp's own nextHarmonicGroupId()
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
            parameter.ranges.min = 0.001f; // matches EqModel.cpp's own harmonicRatio clamp
            parameter.ranges.max = 1000.0f;
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
            parameter.ranges.max = 12.0f; // matches UIEq::setCrossFeedGainLtoRDb()/setCrossFeedGainRtoLDb()'s own clamp
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
            // See ParameterIndex's own comment for why this is a plain
            // automatable value, not kParameterIsTrigger -- same hints
            // kParamUiHeartbeat already uses, for the same reason (a host
            // must see every delivery as a genuine value *change*).
            // Hidden, same as the rest of the mirror bank -- this is
            // internal GUI<->DSP plumbing, never something a user should
            // twiddle by hand or automate.
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

void PluginEq::initState(uint32_t index, State& state)
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
        state.defaultValue = "3"; // ssfilter::SpectrumMode::Both -- this plugin's own requested default, see PluginEq.hpp's own comment
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
        state.key   = "abSlotA";
        state.label = "A/B Slot A";
        state.hints = 0; // UI convenience only
        state.defaultValue = "";
        break;
    case kStateAbSlotB:
        state.key   = "abSlotB";
        state.label = "A/B Slot B";
        state.hints = 0;
        state.defaultValue = "";
        break;
    case kStateAbHasA:
        state.key   = "abHasA";
        state.label = "A/B Slot A Populated";
        state.hints = 0;
        state.defaultValue = "0";
        break;
    case kStateAbHasB:
        state.key   = "abHasB";
        state.label = "A/B Slot B Populated";
        state.hints = 0;
        state.defaultValue = "0";
        break;
    case kStateAbActive:
        state.key   = "abActive";
        state.label = "A/B Active Slot";
        state.hints = 0;
        state.defaultValue = "0"; // A
        break;
    }
}

float PluginEq::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamBypass:    return bypass_ ? 1.0f : 0.0f;
    case kParamFaultFlag: return engine_.consumeFaultFlag() ? 1.0f : 0.0f;
    case kParamAudition:    return auditioning_ ? 1.0f : 0.0f;
    // See PluginDynEq::getParameterValue()'s identical comment --
    // auditionBand_ can be -1 internally ("nothing selected to preview"),
    // clamped to 0 for the host's benefit since that's the only
    // meaningful Parameter value in that state anyway (run() itself
    // already refuses to engage the tap whenever auditionBand_ is out of
    // the model's real band range).
    case kParamAuditionBand: return float(std::max(0, auditionBand_));
    default:
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumOutputBase));
        if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return inputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumInputBase));
        if (index >= kParamSpectrumOutputLBase && index < kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzerL_.getPackedTriple(int(index - kParamSpectrumOutputLBase));
        if (index >= kParamSpectrumOutputRBase && index < kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzerR_.getPackedTriple(int(index - kParamSpectrumOutputRBase));
        // The Parameter mirror -- see ParameterIndex's own comment.
        // mirrorBandSlots_/mirrorBandCount_/mirrorOutputTrimDb_ are kept in
        // sync with eqModel_ by syncMirrorFromModel() (called at the end of
        // every rebuildEngineModel()), so simply reading them back here is
        // always safe and always current -- no separate "is this stale"
        // bookkeeping needed.
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
        if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
            return float(int(mirrorBandSlots_[index - kParamBandCrossFeedBase].crossFeed));
        if (index == kParamCrossFeedGainLtoRDb) return float(mirrorCrossFeedGainLtoRDb_);
        if (index == kParamCrossFeedGainRtoLDb) return float(mirrorCrossFeedGainRtoLDb_);
        if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
            return float(mirrorBandSlots_[index - kParamBandBandwidthBase].bandwidthOct);
        if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
            return mirrorBandSlots_[index - kParamBandSoloBase].solo ? 1.0f : 0.0f;
        return 0.0f;
    }
}

void PluginEq::setParameterValue(uint32_t index, float value)
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
        // The value itself is meaningless -- only the fact that a write
        // just arrived matters. See kParamUiHeartbeat's own comment.
        uiHeartbeatSamplesSinceLast_ = 0;
        break;
    case kParamAudition:
        auditioning_ = value >= 0.5f;
        break;
    case kParamAuditionBand:
        auditionBand_ = int(value + 0.5f);
        updateAuditionCoeffs();
        break;
    case kParamMirrorCommit:
        // The GUI has just finished writing every field of the mirror bank
        // below for one complete edit -- see kParamMirrorCommit's own
        // comment in the header for why this is guaranteed to arrive last,
        // and for why any delivery at all (regardless of the actual
        // number, which only ever exists to make the value keep changing)
        // means "apply now" -- unconditional, unlike
        // adoptMirrorIfModelEmpty(): the mirror's current content is
        // always meant to win here, including a legitimate "every band
        // just got deleted" zero-band state.
        applyMirrorToModel();
        break;
    default:
        // The Parameter mirror -- see ParameterIndex's own comment. Every
        // slot is just an independent fixed-size storage cell (no vector
        // growth/trim), so these are safe to receive in any order a host
        // chooses to restore them in -- deliberately does NOT touch
        // eqModel_/rebuildEngineModel() here on its own (that only happens
        // once kParamMirrorCommit above fires, or as a restore-time
        // fallback from activate()'s own adoptMirrorIfModelEmpty() -- see
        // those functions' own comments for why not doing it here matters:
        // up to ~290 of these can arrive in a single restore burst, and
        // rebuilding the engine on every one of them would risk audible
        // glitches from a transiently-inconsistent band list, on top of
        // being needless one-off startup cost for no benefit).
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
        else if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandCrossFeedBase].crossFeed = ssfilter::EqCrossFeed(std::max(0, std::min(2, int(value + 0.5f))));
        else if (index == kParamCrossFeedGainLtoRDb)
            mirrorCrossFeedGainLtoRDb_ = double(value);
        else if (index == kParamCrossFeedGainRtoLDb)
            mirrorCrossFeedGainRtoLDb_ = double(value);
        else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandBandwidthBase].bandwidthOct = double(value);
        else if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
            mirrorBandSlots_[index - kParamBandSoloBase].solo = value >= 0.5f;
        break;
    }
}

void PluginEq::setState(const char* key, const char* value)
{
    // "presetDir"/"spectrumSmoothMs"/"yRangeDb"/"spectrumAutoscale" are
    // pure UI preferences -- DPF's base Plugin class remembers whatever
    // string was last passed to setState() for any key and hands it back
    // to the host for session persistence on its own (see
    // PluginStateSpace.hpp's identical comment), so simply not matching
    // them below is enough; the DSP side has nothing to *do* with any of
    // them. "spectrumFftSize" and "spectrumMode" are the two exceptions in
    // this group -- both are DSP-relevant (they reconfigure the analyzers /
    // decide which run at all), just not continuously-automatable
    // Parameters (see initState()'s comments on why).
    if (std::strcmp(key, "spectrumFftSize") == 0)
    {
        // Both analyzers share the same resolution preset -- there is only
        // one "Spectrum Resolution" control in the UI (see UIEq.cpp),
        // deliberately not a per-analyzer setting, so there's nothing to
        // gain from letting the two drift to different FFT sizes.
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

void PluginEq::rebuildEngineModel()
{
    ssfilter::StateSpaceMatrices assembled = ssfilter::eq::assembleStateSpace(eqModel_, getSampleRate());
    // Defense in depth, same reasoning as PluginStateSpace::setState(): the
    // GUI already enforces stability before an edit is ever serialized,
    // but a rebuild can also be triggered by a sample-rate change alone
    // (see rebuildEngineModel()'s declaration comment) or by state
    // restored from a hand-edited/foreign/future-version session file, so
    // this is re-validated right before it reaches the real-time engine
    // regardless of who asked for the rebuild.
    ssfilter::StabilityGuard::enforce(assembled);
    engine_.setModel(assembled);
    updateAuditionCoeffs(); // the auditioned band's own freq/Q/gain may have just changed too
    asymmetricChannels_ = ssfilter::eq::hasPerChannelBands(eqModel_); // recomputed here, not per-block -- see the field's own comment
    syncMirrorFromModel(); // keep the Parameter mirror current -- see its own comment
}

void PluginEq::syncMirrorFromModel()
{
    const int n = std::min(int(eqModel_.bands.size()), int(kNumMirroredBands));
    for (int i = 0; i < n; ++i) mirrorBandSlots_[i] = eqModel_.bands[size_t(i)];
    for (int i = n; i < int(kNumMirroredBands); ++i) mirrorBandSlots_[i] = ssfilter::EqBand(); // unused slots hold a well-defined default, never stale leftovers
    mirrorBandCount_ = n;
    mirrorOutputTrimDb_ = eqModel_.outputTrimDb;
    mirrorCrossFeedGainLtoRDb_ = eqModel_.crossFeedGainLtoRDb;
    mirrorCrossFeedGainRtoLDb_ = eqModel_.crossFeedGainRtoLDb;
}

void PluginEq::adoptMirrorIfModelEmpty()
{
    // Only ever adopt once, on a freshly restored instance that hasn't
    // picked up its mirror yet -- once eqModel_ has bands, a live edit
    // (kParamMirrorCommit) is the only thing allowed to change it further.
    if (!eqModel_.bands.empty()) return;
    // Nothing in the mirror either: a genuinely fresh, never-configured
    // instance (identical in shape to a failed restore, and correctly so
    // -- adopting an empty mirror here is a harmless no-op either way).
    if (mirrorBandCount_ <= 0) return;
    applyMirrorToModel();
}

void PluginEq::applyMirrorToModel()
{
    eqModel_.bands.assign(mirrorBandSlots_, mirrorBandSlots_ + mirrorBandCount_);
    eqModel_.outputTrimDb = mirrorOutputTrimDb_;
    eqModel_.crossFeedGainLtoRDb = mirrorCrossFeedGainLtoRDb_;
    eqModel_.crossFeedGainRtoLDb = mirrorCrossFeedGainRtoLDb_;
    rebuildEngineModel(); // also re-syncs the mirror from eqModel_ -- a harmless no-op, they're already identical
}

void PluginEq::updateAuditionCoeffs()
{
    auditionActiveBands_.clear();
    auditionCoeffs_.clear();
    auditionGainLin_.clear();

    // Grow (never shrink below the current band count -- see the field's
    // own comment) so state indexed by band index below always has a slot,
    // and so an already-auditioning band's own filter memory survives a
    // band being added/removed elsewhere in the list.
    if (auditionStateL_.size() < eqModel_.bands.size())
    {
        auditionStateL_.resize(eqModel_.bands.size());
        auditionStateR_.resize(eqModel_.bands.size());
    }

    if (auditionBand_ < 0 || auditionBand_ >= int(eqModel_.bands.size())) return;

    // auditionBand_ on its own, plus every other member of its harmonic
    // group (if any) -- see auditionActiveBands_'s own comment for why.
    // An unlinked band (harmonicGroup == 0) previews alone, same as before
    // this feature existed.
    const int group = eqModel_.bands[size_t(auditionBand_)].harmonicGroup;
    for (int i = 0; i < int(eqModel_.bands.size()); ++i)
    {
        if (i == auditionBand_ || (group != 0 && eqModel_.bands[size_t(i)].harmonicGroup == group))
            auditionActiveBands_.push_back(i);
    }

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    for (int idx : auditionActiveBands_)
    {
        const ssfilter::EqBand& b = eqModel_.bands[size_t(idx)];
        // A genuine, magnitude-shaped bandpass at the band's own freq/Q,
        // regardless of that band's actual Type (Bell/Shelf/HP/LP/...) --
        // Audition previews "the frequency region this band is centered
        // on", not a re-rendering of whatever shape it currently applies
        // there. Same clamps eq::designBand() itself applies before
        // handing freq/Q to any RBJ formula, and the same
        // mappers::designBandpass() the sibling Dynamic EQ plugin's own
        // detector tap uses (dyneq::designDetectorBandpass()) -- see that
        // function's own comment.
        double bandQ = b.q;
        if (ssfilter::eqBandTypeHasBandwidth(b.type))
        {
            // Band Shelf: the region between its two corners, i.e. the
            // bandpass Q whose -3dB edges land on them (Q = sqrt(r)/(r-1),
            // r = hi/lo = 2^bandwidth) around its centre freqHz.
            const double r = std::max(1.01, std::pow(2.0, b.bandwidthOct));
            bandQ = std::sqrt(r) / (r - 1.0);
        }
        const double freqHz = std::max(1.0, std::min(sr * 0.499, b.freqHz));
        const double q = std::max(0.05, std::min(100.0, bandQ));
        auditionCoeffs_.push_back(ssfilter::mappers::designBandpass(sr, freqHz, q));

        // The band's own static gain *is* applied to the tap's output --
        // the very first version of this feature (still named "Listen"
        // then) didn't, so turning the Gain knob had zero audible effect
        // while auditioning -- wrong, since the sibling Dynamic EQ
        // plugin's own preview *does* scale by its
        // band's live gain, see DynamicEqEngine.hpp's own "listenL =
        // detOutL * G" line, so this was a real inconsistency with the
        // pattern being copied, not a deliberate difference). A band type
        // with no gain of its own (High/Low Pass/Notch/Band Pass/All Pass,
        // see eqBandTypeHasGain()) has nothing to apply here -- gainDb is
        // already forced to 0 for those by setBandType(), so this is
        // correct unchanged for them too.
        auditionGainLin_.push_back(std::pow(10.0, b.gainDb / 20.0));
    }
}

void PluginEq::activate()
{
    // The band list lives only in the Parameter mirror now (no State to
    // restore first), and DPF only delivers a restored input Parameter to
    // our own setParameterValue() from inside run() itself (see run()'s
    // own comment for the full mechanism), which always happens strictly
    // after the first activate(). So this call below is always a no-op
    // here on a freshly restored instance -- mirrorBandCount_ is still at
    // its untouched default at this point -- and the real adoption
    // happens from run()'s own identical call instead, the first time
    // real audio actually flows. Kept here too regardless, cheap and
    // harmless, in case a future host ever behaves differently.
    adoptMirrorIfModelEmpty();
    engine_.resetState();
    auditionStateL_.assign(eqModel_.bands.size(), ssfilter::Biquad1State());
    auditionStateR_.assign(eqModel_.bands.size(), ssfilter::Biquad1State());
}

void PluginEq::run(const float** inputs, float** outputs, uint32_t frames)
{
    // Retry the same adoption activate() already attempted -- see
    // adoptMirrorIfModelEmpty()'s own comment for why activate() alone is
    // NOT enough: DPF's own LV2 wrapper only calls our setParameterValue()
    // for a restored input port's value from *inside* run() (it diffs each
    // port's current buffer against a cached "last known" value once per
    // block, dispatching only on a real host-side callback into DPF's own
    // run() glue -- see dpf/distrho/src/DistrhoPluginLV2.cpp's "Check for
    // updated parameters" loop), never synchronously during restore itself.
    // activate() always runs before the very first run(), so on every
    // freshly restored instance, mirrorBandCount_/mirrorBandSlots_[] are
    // *still* at their untouched defaults the one time activate() checks
    // them, even though Ardour's own port buffers already hold the real,
    // correctly-restored values by then -- confirmed directly via added
    // diagnostic logging showing
    // adoptMirrorIfModelEmpty() running with mirrorBandCount_=0 *before*
    // setParameterValue(kParamBandCount, 5.0) ever arrives. Nothing else
    // ever gave it a second chance once the real values did land. Cheap to
    // call unconditionally every block -- the function's own early-return
    // guards make this a couple of comparisons once resolved, for the
    // entire remaining lifetime of the instance.
    adoptMirrorIfModelEmpty();

    // The input analyzer MUST be fed before anything below touches the
    // buffers. This plugin's LV2 metadata never declares `inPlaceBroken`
    // (see DistrhoPluginInfo.h -- nothing in this project's mapper/engine
    // math actually requires distinct buffers, so there was never a
    // reason to declare it), which under the LV2 spec means a host is
    // free to connect the *same* memory to an input port and an output
    // port ("in-place" processing) -- and real hosts (Ardour included)
    // routinely do exactly that for a plain stereo effect, to skip a
    // buffer copy. When that happens, `inputs[0]/[1]` and `outputs[0]/[1]`
    // are literally the same pointers: engine_.processBlock() below (or
    // the bypass copy) would silently overwrite `inputs[]` with the
    // post-EQ signal before the "read the pre-EQ signal" step ever got a
    // chance to run, making the two spectrum traces read identical even
    // with a real filter engaged. Reading the input here first, before any write to
    // `outputs[]` happens at all, is correct regardless of whether this
    // particular host call happens to alias or not.
    // With the GUI closed, no display calculation should run at all.
    // kParamUiHeartbeat arrives (via
    // setParameterValue(), same delivery path as every other parameter)
    // roughly once per uiIdle() tick while an editor is actually open;
    // once more than kUiHeartbeatTimeoutSamples of audio has been
    // processed since the last one, treat the editor as closed and skip
    // feeding the analyzer/analyzers entirely -- a read-only display tap
    // with no path back into outputs[] (see feedSpectrumAnalyzer()'s own
    // comment), so this can never change the audio, only whether a nobody-
    // is-watching FFT gets computed at all. Saturating (not wrapping) add:
    // see uiHeartbeatSamplesSinceLast_'s own comment for why.
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
    else if (auditioning_ && !auditionActiveBands_.empty())
    {
        // engine_.processBlock() still runs normally on every sample --
        // its own internal recursion keeps going exactly as if Audition
        // were off, so switching Audition back off is an instant, glitch-
        // free return to the real signal rather than a filter that's been
        // frozen or reset while auditioning (same property the sibling
        // Dynamic EQ plugin's own engine has, see DynamicEqEngine.hpp's
        // own gain-scaled swap). Only the final *output* sample gets
        // replaced with the Audition tap's own value, scaled by
        // auditionGainLin_ (see updateAuditionCoeffs()'s own comment for
        // why that matters).
        //
        // The tap itself needs the true pre-EQ signal, though, and
        // engine_.processBlock() below can alias inputs[]/outputs[]
        // together in-place (same hazard feedSpectrumAnalyzer()'s own
        // comment above already documents) -- so each chunk's raw input
        // is captured into a small fixed-size scratch buffer *before*
        // the engine runs on it, exactly the same bounded-chunk pattern
        // feedSpectrumAnalyzer() already uses, rather than requiring an
        // unbounded (non-RT-safe) whole-block copy.
        //
        // One or more taps run per sample -- more than one exactly when
        // the auditioned band is part of a harmonic group, see
        // auditionActiveBands_'s own comment -- and are simply summed;
        // each is indexed into auditionStateL_/R_ by its own band index
        // (not by its position in auditionActiveBands_/auditionCoeffs_,
        // which are parallel to each other but not to the state arrays).
        constexpr uint32_t kChunk = 4096;
        float rawL[kChunk], rawR[kChunk];
        for (uint32_t off = 0; off < frames; off += kChunk)
        {
            const uint32_t n = std::min(kChunk, frames - off);
            std::memcpy(rawL, inputs[0] + off, sizeof(float) * n);
            std::memcpy(rawR, inputs[1] + off, sizeof(float) * n);

            engine_.processBlock(inputs[0] + off, inputs[1] + off, outputs[0] + off, outputs[1] + off, n);

            for (uint32_t i = 0; i < n; ++i)
            {
                double sumL = 0.0, sumR = 0.0;
                for (size_t k = 0; k < auditionActiveBands_.size(); ++k)
                {
                    const size_t bandIdx = size_t(auditionActiveBands_[k]);
                    sumL += auditionGainLin_[k] * ssfilter::processDF1(auditionStateL_[bandIdx], auditionCoeffs_[k], double(rawL[i]));
                    sumR += auditionGainLin_[k] * ssfilter::processDF1(auditionStateR_[bandIdx], auditionCoeffs_[k], double(rawR[i]));
                }
                outputs[0][off + i] = float(sumL);
                outputs[1][off + i] = float(sumR);
            }
        }
    }
    else
    {
        engine_.processBlock(inputs[0], inputs[1], outputs[0], outputs[1], frames);
    }

    // Feed only the analyzer(s) spectrumMode_ actually selects -- an
    // analyzer that isn't feeding the display doesn't get its process()
    // (FFT/windowing) cost paid at all, which is the entire point of
    // having a mode instead of always running both. This makes "Input"
    // and "Output" mean exactly what they say even while bypassed: the
    // input analyzer (above) always sees the true pre-EQ signal, and the
    // output analyzer sees whatever's actually in `outputs` -- while
    // bypassed that's just a copy of the input, so the two traces
    // legitimately coincide, which is the correct picture of "no filtering
    // is happening right now", not a special case to work around.
    if (uiOpen && ssfilter::spectrumModeWantsOutput(spectrumMode_))
    {
        feedSpectrumAnalyzer(outputSpectrumAnalyzer_, outputs[0], outputs[1], frames);

        // The two per-channel analyzers, fed only while at least one
        // active band is actually Left/Right-only -- see
        // asymmetricChannels_'s own comment. Reuses feedSpectrumAnalyzer()
        // itself rather than a separate single-channel variant: passing
        // the *same* buffer for both its `left`/`right` arguments makes
        // its own mono-mix (`0.5f * (left[i] + right[i])`) reduce to
        // exactly that one channel's own signal, unchanged -- no new
        // function needed for a genuinely single-channel feed.
        if (asymmetricChannels_)
        {
            feedSpectrumAnalyzer(outputSpectrumAnalyzerL_, outputs[0], outputs[0], frames);
            feedSpectrumAnalyzer(outputSpectrumAnalyzerR_, outputs[1], outputs[1], frames);
        }
    }
}

void PluginEq::feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames)
{
    // Fixed-size scratch buffer, chunked, so this stays allocation-free
    // (RT-safe) regardless of how large a single host callback's block
    // is -- most hosts use a few hundred to a few thousand frames, well
    // under kChunk, but nothing guarantees that in general.
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

void PluginEq::sampleRateChanged(double newSampleRate)
{
    engine_.prepare(newSampleRate);
    outputSpectrumAnalyzer_.prepare(newSampleRate);
    inputSpectrumAnalyzer_.prepare(newSampleRate);
    outputSpectrumAnalyzerL_.prepare(newSampleRate);
    outputSpectrumAnalyzerR_.prepare(newSampleRate);
    rebuildEngineModel(); // biquad coefficients are sample-rate-dependent, see the .hpp comment
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginEq(); }
END_NAMESPACE_DISTRHO
