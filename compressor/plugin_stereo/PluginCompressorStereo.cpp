// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginCompressorStereo.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace sscomp;

PluginCompressorStereo::PluginCompressorStereo()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
}

void PluginCompressorStereo::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    // Indices 0/1 are always the main L/R pair (both in and out). With the
    // sidechain input added, input indices 2/3 are the external key
    // signal's own L/R pair instead -- left ungrouped (no groupId) so
    // hosts (VST3/CLAP especially) treat it as its own auxiliary bus
    // rather than folding it into the main stereo group.
    if (input && index >= 2)
    {
        port.hints = kAudioPortIsSidechain;
        port.name = (index == 2) ? "Sidechain L" : "Sidechain R";
        port.symbol = (index == 2) ? "sidechain_in_l" : "sidechain_in_r";
        return;
    }
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginCompressorStereo::initParameter(uint32_t index, Parameter& parameter)
{
    switch (index)
    {
    case kParamInputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Input Gain";
        parameter.symbol = "input_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamThreshold:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Threshold";
        parameter.symbol = "threshold";
        parameter.unit = "dB";
        parameter.ranges.def = -18.0f; parameter.ranges.min = -60.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamRatio:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Ratio";
        parameter.symbol = "ratio";
        parameter.unit = ":1";
        parameter.ranges.def = 4.0f; parameter.ranges.min = 1.0f; parameter.ranges.max = 20.0f;
        break;
    case kParamAttack:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Attack";
        parameter.symbol = "attack";
        parameter.unit = "ms";
        parameter.ranges.def = 10.0f; parameter.ranges.min = 0.1f; parameter.ranges.max = 300.0f;
        break;
    case kParamRelease:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Release";
        parameter.symbol = "release";
        parameter.unit = "ms";
        parameter.ranges.def = 150.0f; parameter.ranges.min = 5.0f; parameter.ranges.max = 2000.0f;
        break;
    case kParamKnee:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Knee";
        parameter.symbol = "knee";
        parameter.unit = "dB";
        parameter.ranges.def = 6.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamOutputGain:
        // Renamed from "Output Gain" to "Makeup Gain" -- symbol
        // and index deliberately untouched, only the display name changes:
        // Ardour's session XML embeds this Parameter's value by port
        // index/symbol, never by this name string, so existing projects
        // keep reading it exactly as before.
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Makeup Gain";
        parameter.symbol = "output_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    // Removed -- see this method's own kParamCompensation comment near
    // kParamOutputGain above for why these two stay declared at their
    // existing indices instead of being deleted outright. Display names
    // marked "(unused)" so they read as vestigial if anything (a generic
    // host parameter list, say) shows them outside this plugin's own GUI.
    case kParamCompensation:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Volume Compensation (unused)";
        parameter.symbol = "compensation";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamCompensationMeasure:
        parameter.hints = kParameterIsAutomatable | kParameterIsTrigger;
        parameter.name = "Recalibrate Compensation (unused)";
        parameter.symbol = "compensation_measure";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamBypass:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Bypass";
        parameter.symbol = "bypass";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamSidechainEnable:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Sidechain";
        parameter.symbol = "sidechain_enable";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamSidechainGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Sidechain Gain";
        parameter.symbol = "sidechain_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = float(sscomp::kSidechainGainMinDb); parameter.ranges.max = float(sscomp::kSidechainGainMaxDb);
        break;
    case kParamListenMode:
        parameter.hints = kParameterIsAutomatable | kParameterIsInteger;
        parameter.name = "Listen (M/S)";
        parameter.symbol = "listen_mode";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 2.0f;
        break;
    case kParamThresholdB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Threshold B";
        parameter.symbol = "threshold_b";
        parameter.unit = "dB";
        parameter.ranges.def = -18.0f; parameter.ranges.min = -60.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamRatioB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Ratio B";
        parameter.symbol = "ratio_b";
        parameter.unit = ":1";
        parameter.ranges.def = 4.0f; parameter.ranges.min = 1.0f; parameter.ranges.max = 20.0f;
        break;
    case kParamAttackB:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Attack B";
        parameter.symbol = "attack_b";
        parameter.unit = "ms";
        parameter.ranges.def = 10.0f; parameter.ranges.min = 0.1f; parameter.ranges.max = 300.0f;
        break;
    case kParamReleaseB:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Release B";
        parameter.symbol = "release_b";
        parameter.unit = "ms";
        parameter.ranges.def = 150.0f; parameter.ranges.min = 5.0f; parameter.ranges.max = 2000.0f;
        break;
    case kParamKneeB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Knee B";
        parameter.symbol = "knee_b";
        parameter.unit = "dB";
        parameter.ranges.def = 6.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamMakeupGainB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Makeup Gain B";
        parameter.symbol = "makeup_gain_b";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamGlobalOutputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Output Gain";
        parameter.symbol = "global_output_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamScFilterEnable:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Sidechain Filter";
        parameter.symbol = "sc_filter_enable";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamScFilterHighpassHz:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Sidechain Filter Highpass";
        parameter.symbol = "sc_filter_highpass_hz";
        parameter.unit = "Hz";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 20.0f; parameter.ranges.max = 8000.0f;
        break;
    case kParamScFilterLowpassHz:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Sidechain Filter Lowpass";
        parameter.symbol = "sc_filter_lowpass_hz";
        parameter.unit = "Hz";
        parameter.ranges.def = 8000.0f; parameter.ranges.min = 100.0f; parameter.ranges.max = 20000.0f;
        break;
    case kParamScFilterLinkLR:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Sidechain Filter Link L/R";
        parameter.symbol = "sc_filter_link_lr";
        parameter.ranges.def = 1.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamScFilterHighpassHzR:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Sidechain Filter Highpass R";
        parameter.symbol = "sc_filter_highpass_hz_r";
        parameter.unit = "Hz";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 20.0f; parameter.ranges.max = 8000.0f;
        break;
    case kParamScFilterLowpassHzR:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Sidechain Filter Lowpass R";
        parameter.symbol = "sc_filter_lowpass_hz_r";
        parameter.unit = "Hz";
        parameter.ranges.def = 8000.0f; parameter.ranges.min = 100.0f; parameter.ranges.max = 20000.0f;
        break;
    case kParamInputGainB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Input Gain B";
        parameter.symbol = "input_gain_b";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamWetDry:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Wet/Dry";
        parameter.symbol = "wet_dry";
        parameter.unit = "%";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 100.0f;
        break;
    case kParamWetDryB:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Wet/Dry B";
        parameter.symbol = "wet_dry_b";
        parameter.unit = "%";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 100.0f;
        break;
    case kParamGlobalInputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Global Input Gain";
        parameter.symbol = "global_input_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamUiHeartbeat:
        parameter.hints = kParameterIsAutomatable | kParameterIsHidden;
        parameter.name = "Scope Active (internal)";
        parameter.symbol = "ui_heartbeat";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1000000.0f; // counter
        break;

    case kParamMeterInputRmsL:
    case kParamMeterInputPeakL:
    case kParamMeterInputRmsR:
    case kParamMeterInputPeakR:
    case kParamMeterOutputRmsL:
    case kParamMeterOutputPeakL:
    case kParamMeterOutputRmsR:
    case kParamMeterOutputPeakR:
    {
        static const char* names[] = {
            "Input RMS L", "Input Peak L", "Input RMS R", "Input Peak R",
            "Output RMS L", "Output Peak L", "Output RMS R", "Output Peak R"
        };
        static const char* symbols[] = {
            "meter_in_rms_l", "meter_in_peak_l", "meter_in_rms_r", "meter_in_peak_r",
            "meter_out_rms_l", "meter_out_peak_l", "meter_out_rms_r", "meter_out_peak_r"
        };
        const uint32_t i = index - kParamMeterInputRmsL;
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = names[i];
        parameter.symbol = symbols[i];
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    }
    case kParamMeterDetectorA:
    case kParamMeterDetectorB:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = (index == kParamMeterDetectorA) ? "Detector A Level" : "Detector B Level";
        parameter.symbol = (index == kParamMeterDetectorA) ? "meter_detector_a" : "meter_detector_b";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamMeterGainReductionA:
    case kParamMeterGainReductionB:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = (index == kParamMeterGainReductionA) ? "Gain Reduction A" : "Gain Reduction B";
        parameter.symbol = (index == kParamMeterGainReductionA) ? "meter_gr_a" : "meter_gr_b";
        parameter.unit = "dB";
        // Range widened from the original -48..0 (downward-only) to also
        // cover a positive reading -- Up direction (kStateRatioDirection)
        // can report a *boost*, not just a reduction.
        parameter.ranges.def = 0.0f; parameter.ranges.min = -48.0f; parameter.ranges.max = 24.0f;
        break;
    // Always report 0 now -- see kParamCompensation's own comment.
    case kParamMeterCompGainDb:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Compensation Gain (unused)";
        parameter.symbol = "meter_comp_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamMeterCompState:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Compensation State (unused)";
        parameter.symbol = "meter_comp_state";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 2.0f;
        break;
    case kParamMeterScLevelDb:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Sidechain Monitor Level (unused)";
        parameter.symbol = "meter_sc_level";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamMeterScLevelDbL:
    case kParamMeterScLevelDbR:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = (index == kParamMeterScLevelDbL) ? "Sidechain Monitor Level L" : "Sidechain Monitor Level R";
        parameter.symbol = (index == kParamMeterScLevelDbL) ? "meter_sc_level_l" : "meter_sc_level_r";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamScListenEnable:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Sidechain Listen";
        parameter.symbol = "sc_listen_enable";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    default:
        // The two packed sidechain-monitor spectrum banks -- Out
        // (kParamMeterScSpectrumBase, pre-existing: post-filter/whatever's
        // currently reaching the detector) and In (kParamMeterScSpectrumInBase,
        // new: always pre-filter), both always shown, like on the EQs. See
        // kParamMeterScSpectrumInBase's own comment (PluginCompressorStereo.hpp)
        // and SpectrumAnalyzer.hpp's own "Packed transport" comment for
        // the full reasoning; same declaration shape (and
        // kParameterIsInteger hint -- an exact packed integer, not a dB
        // value a host should ever interpolate/smooth) as
        // ../../statespace's EQ plugins' own identical bank.
        if (index >= kParamMeterScSpectrumBase && index < kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamMeterScSpectrumBase;
            char name[32], symbol[32];
            std::snprintf(name, sizeof(name), "SC Spectrum Out Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "meter_sc_spectrum_out_%u", band);
            parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = sscomp::SpectrumAnalyzer::kMaxPackedValue;
        }
        else if (index >= kParamMeterScSpectrumInBase && index < kParamMeterScSpectrumInBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
        {
            const uint32_t band = index - kParamMeterScSpectrumInBase;
            char name[32], symbol[32];
            std::snprintf(name, sizeof(name), "SC Spectrum In Packet %u", band);
            std::snprintf(symbol, sizeof(symbol), "meter_sc_spectrum_in_%u", band);
            parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
            parameter.name = name;
            parameter.symbol = symbol;
            parameter.ranges.def = 0.0f;
            parameter.ranges.min = 0.0f;
            parameter.ranges.max = sscomp::SpectrumAnalyzer::kMaxPackedValue;
        }
        break;
    }
}

void PluginCompressorStereo::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStateStereoMode:
        state.key = "stereoMode";
        state.label = "Stereo Mode (Linked/DualMono/MidSide)";
        state.hints = 0; // DSP-relevant (see setState()), but a discrete topology choice, not a continuous automatable control
        state.defaultValue = "0";
        break;
    case kStateRatioDirection:
        state.key = "ratioDirection";
        state.label = "Ratio Direction (Down/Up)";
        state.hints = 0; // DSP-relevant (see setState()), but a discrete two-way choice, not a continuous automatable control
        state.defaultValue = "0"; // Down -- the historical/default behavior
        break;
    case kStateRatioDirectionB:
        state.key = "ratioDirectionB";
        state.label = "Ratio Direction B (Down/Up)";
        state.hints = 0;
        state.defaultValue = "0";
        break;
    case kStateLinkAB:
        state.key = "linkAB";
        state.label = "Link A/B Dynamics (Dual-Mono only)";
        state.hints = 0; // DSP-relevant (see setState()), but a discrete two-way choice, not a continuous automatable control
        state.defaultValue = "1"; // linked -- matches this plugin's own pre-existing (shared-parameter) behavior
        break;
    case kStatePresetDir:
        state.key = "presetDir";
        state.label = "Last Preset Directory";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "";
        break;
    }
}

float PluginCompressorStereo::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamInputGain:  return float(inputGainDb_);
    case kParamThreshold:  return float(thresholdDb_);
    case kParamRatio:      return float(ratio_);
    case kParamAttack:     return float(attackMs_);
    case kParamRelease:    return float(releaseMs_);
    case kParamKnee:       return float(kneeDb_);
    case kParamOutputGain: return float(outputGainDb_);
    // Volume Compensation removed (Makeup Gain/Global Output Gain/the
    // GR-meter net-bar visualization made it redundant). kParamCompensation/
    // kParamCompensationMeasure/kParamMeterCompGainDb/kParamMeterCompState
    // stay declared at their *exact* existing indices -- removing/
    // reordering them would shift every Parameter after them, silently
    // rebinding any automation lane already saved against those indices
    // in a real project (confirmed against a real saved .ardour session:
    // Ardour's `parameter="N"` Controllable/
    // AutomationList binding is `raw DPF Parameter index + a constant
    // offset`, not name/symbol-based) -- so they're kept as inert,
    // always-off/always-zero reserved slots instead. compensationEnabled_
    // stays too, purely to round-trip whatever a saved session's
    // "compensation" port value already is (0 in every project that
    // matters) without doing anything with it any more.
    case kParamCompensation: return compensationEnabled_ ? 1.0f : 0.0f;
    case kParamBypass:       return bypass_ ? 1.0f : 0.0f;
    case kParamSidechainEnable: return sidechainEnabled_ ? 1.0f : 0.0f;
    case kParamSidechainGain:   return float(sidechainGainDb_);
    case kParamListenMode:      return float(listenMode_);
    case kParamThresholdB: return float(thresholdDbB_);
    case kParamRatioB:     return float(ratioB_);
    case kParamAttackB:    return float(attackMsB_);
    case kParamReleaseB:   return float(releaseMsB_);
    case kParamKneeB:      return float(kneeDbB_);
    case kParamMakeupGainB: return float(makeupGainDbB_);
    case kParamGlobalOutputGain: return float(globalOutputGainDb_);
    case kParamScFilterEnable:     return scFilterEnabled_ ? 1.0f : 0.0f;
    case kParamScFilterHighpassHz: return float(scFilterHighpassHz_);
    case kParamScFilterLowpassHz:  return float(scFilterLowpassHz_);
    case kParamScFilterLinkLR:      return scFilterLinkLR_ ? 1.0f : 0.0f;
    case kParamScFilterHighpassHzR: return float(scFilterHighpassHzR_);
    case kParamScFilterLowpassHzR:  return float(scFilterLowpassHzR_);
    case kParamInputGainB: return float(inputGainDbB_);
    case kParamWetDry:  return float(wetDry_ * 100.0);
    case kParamWetDryB: return float(wetDryB_ * 100.0);
    case kParamGlobalInputGain: return float(globalInputGainDb_);
    case kParamUiHeartbeat: return 0.0f; // write-only

    case kParamMeterInputRmsL:  return meterInputRmsL_;
    case kParamMeterInputPeakL: return meterInputPeakL_;
    case kParamMeterInputRmsR:  return meterInputRmsR_;
    case kParamMeterInputPeakR: return meterInputPeakR_;
    case kParamMeterOutputRmsL:  return meterOutputRmsL_;
    case kParamMeterOutputPeakL: return meterOutputPeakL_;
    case kParamMeterOutputRmsR:  return meterOutputRmsR_;
    case kParamMeterOutputPeakR: return meterOutputPeakR_;
    case kParamMeterDetectorA: return meterDetectorA_;
    case kParamMeterDetectorB: return meterDetectorB_;
    case kParamMeterGainReductionA: return meterGainReductionA_;
    case kParamMeterGainReductionB: return meterGainReductionB_;
    // Always 0 now -- see kParamCompensation's own comment above.
    case kParamMeterCompGainDb: return 0.0f;
    case kParamMeterCompState:  return 0.0f;
    case kParamMeterScLevelDb:  return 0.0f; // unused -- see this Parameter's own comment (initParameter())
    case kParamMeterScLevelDbL: return float(scMonitorPeakL_.peakDb());
    case kParamMeterScLevelDbR: return float(scMonitorPeakR_.peakDb());
    case kParamScListenEnable: return scListenEnabled_ ? 1.0f : 0.0f;
    default:
        if (index >= kParamMeterScSpectrumBase && index < kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
            return scMonitorSpectrumOut_.getPackedTriple(int(index - kParamMeterScSpectrumBase));
        if (index >= kParamMeterScSpectrumInBase && index < kParamMeterScSpectrumInBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
            return scMonitorSpectrumIn_.getPackedTriple(int(index - kParamMeterScSpectrumInBase));
        return 0.0f;
    }
}

void PluginCompressorStereo::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamInputGain:  inputGainDb_ = value; break;
    case kParamThreshold:  thresholdDb_ = value; break;
    case kParamRatio:      ratio_ = value; break;
    case kParamAttack:     attackMs_ = value; break;
    case kParamRelease:    releaseMs_ = value; break;
    case kParamKnee:       kneeDb_ = value; break;
    case kParamOutputGain: outputGainDb_ = value; break;
    // Inert -- see kParamCompensation's own comment (initParameter()) for
    // why these two stay declared/round-tripped rather than deleted.
    case kParamCompensation: compensationEnabled_ = value >= 0.5f; break;
    case kParamCompensationMeasure: break;
    case kParamBypass: bypass_ = value >= 0.5f; break;
    case kParamSidechainEnable: sidechainEnabled_ = value >= 0.5f; break;
    case kParamSidechainGain:   sidechainGainDb_ = value; break;
    case kParamListenMode:      listenMode_ = std::max(0, std::min(2, int(value + 0.5f))); break;
    case kParamThresholdB: thresholdDbB_ = value; break;
    case kParamRatioB:     ratioB_ = value; break;
    case kParamAttackB:    attackMsB_ = value; break;
    case kParamReleaseB:   releaseMsB_ = value; break;
    case kParamKneeB:      kneeDbB_ = value; break;
    case kParamMakeupGainB:      makeupGainDbB_ = value; break;
    case kParamGlobalOutputGain: globalOutputGainDb_ = value; break;
    case kParamScFilterEnable:     scFilterEnabled_ = value >= 0.5f; break;
    case kParamScFilterHighpassHz: scFilterHighpassHz_ = value; updateScFilterFrequencies(); break;
    case kParamScFilterLowpassHz:  scFilterLowpassHz_ = value; updateScFilterFrequencies(); break;
    case kParamScFilterLinkLR:
        scFilterLinkLR_ = value >= 0.5f;
        // "start linked, split on edit" -- re-linking snaps R's own
        // stored frequencies back to L's, same family precedent as
        // syncBFromA()'s own reasoning elsewhere in this plugin, so a
        // later unlink starts fresh from L again rather than from
        // whatever R was last left holding.
        if (scFilterLinkLR_) { scFilterHighpassHzR_ = scFilterHighpassHz_; scFilterLowpassHzR_ = scFilterLowpassHz_; }
        updateScFilterFrequencies();
        break;
    case kParamScFilterHighpassHzR: scFilterHighpassHzR_ = value; updateScFilterFrequencies(); break;
    case kParamScFilterLowpassHzR:  scFilterLowpassHzR_ = value; updateScFilterFrequencies(); break;
    case kParamScListenEnable: scListenEnabled_ = value >= 0.5f; break;
    case kParamInputGainB: inputGainDbB_ = value; break;
    case kParamWetDry:  wetDry_ = double(value) * 0.01; break;
    case kParamWetDryB: wetDryB_ = double(value) * 0.01; break;
    case kParamGlobalInputGain: globalInputGainDb_ = value; break;
    case kParamUiHeartbeat: uiHeartbeatSamplesSinceLast_ = 0; break;
    default: break;
    }
}

void PluginCompressorStereo::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "stereoMode") == 0)
    {
        stereoMode_ = clampStereoMode(std::atoi(value));
        return;
    }
    if (std::strcmp(key, "ratioDirection") == 0)
    {
        ratioUpMode_ = std::atoi(value) != 0;
        return;
    }
    if (std::strcmp(key, "ratioDirectionB") == 0)
    {
        ratioUpModeB_ = std::atoi(value) != 0;
        return;
    }
    if (std::strcmp(key, "linkAB") == 0)
    {
        linkAB_ = std::atoi(value) != 0;
        return;
    }
    // "presetDir" is a pure UI preference -- see PluginCompressorMono's
    // identical comment on why nothing further is needed for it here.
}

void PluginCompressorStereo::activate()
{
    voiceA_.reset();
    voiceB_.reset();
    loudnessInL_.reset(); loudnessInR_.reset(); loudnessOutL_.reset(); loudnessOutR_.reset();
    peakInL_.reset(); peakInR_.reset(); peakOutL_.reset(); peakOutR_.reset();
    scFilterL_.reset(); scFilterR_.reset();
    scMonitorPeakL_.reset(); scMonitorPeakR_.reset();
}

void PluginCompressorStereo::run(const float** inputs, float** outputs, uint32_t frames)
{
    // Is anything reading the spectrum? See kParamUiHeartbeat (~2 s at 48 kHz, the figure the EQs use).
    constexpr uint64_t kUiHeartbeatTimeoutSamples = 96000;
    if (uiHeartbeatSamplesSinceLast_ <= UINT64_MAX - frames) uiHeartbeatSamplesSinceLast_ += frames;
    else uiHeartbeatSamplesSinceLast_ = UINT64_MAX;
    const bool scopeShowing = uiHeartbeatSamplesSinceLast_ < kUiHeartbeatTimeoutSamples;

    const float* inL = inputs[0];
    const float* inR = inputs[1];
    const float* scL = inputs[2]; // external key input, L/R -- always a valid buffer (host-connected audio, or silence if unconnected/inactive), see initAudioPort()
    const float* scR = inputs[3];
    float* outL = outputs[0];
    float* outR = outputs[1];

    if (bypass_)
    {
        if (outL != inL) std::copy(inL, inL + frames, outL);
        if (outR != inR) std::copy(inR, inR + frames, outR);
        const double scGainLinBypass = dbToLinear(sidechainGainDb_);
        for (uint32_t i = 0; i < frames; ++i)
        {
            const double l = double(inL[i]), r = double(inR[i]);
            // "the two bars on the left should also show m/s" (direct
            // ask) -- same Mid/Side re-derivation the main (non-bypass)
            // path below now uses for its own meters, applied here too so
            // Bypass doesn't leave the meter momentarily showing raw L/R
            // again while the bars are still labeled "M"/"S".
            double meterL = l, meterR = r;
            if (stereoMode_ == StereoMode::MidSide) { meterL = (l + r) * 0.5; meterR = (l - r) * 0.5; }
            peakInL_.process(std::fabs(meterL)); peakInR_.process(std::fabs(meterR));
            peakOutL_.process(std::fabs(meterL)); peakOutR_.process(std::fabs(meterR));
            loudnessInL_.process(meterL); loudnessInR_.process(meterR);
            loudnessOutL_.process(meterL); loudnessOutR_.process(meterR);

            // Sidechain monitor -- kept running during Bypass too, same
            // "meters still reflect what's actually happening" reasoning
            // the I/O meters right above already follow. See
            // kParamMeterScLevelDbL/R's own comment.
            const double scGLRaw = double(scL[i]) * scGainLinBypass;
            const double scGRRaw = double(scR[i]) * scGainLinBypass;
            double scGL = scGLRaw, scGR = scGRRaw;
            if (scFilterEnabled_)
            {
                scGL = scFilterL_.process(scGL);
                scGR = scFilterR_.process(scGR);
            }
            scMonitorPeakL_.process(std::fabs(scGL));
            scMonitorPeakR_.process(std::fabs(scGR));
            if (i < kScMonitorScratchFrames)
            {
                scMonitorScratchIn_[i] = float((scGLRaw + scGRRaw) * 0.5);
                scMonitorScratchOut_[i] = float((scGL + scGR) * 0.5);
            }
        }
        if (scopeShowing)
        {
            scMonitorSpectrumIn_.process(scMonitorScratchIn_, std::min(frames, kScMonitorScratchFrames));
            scMonitorSpectrumOut_.process(scMonitorScratchOut_, std::min(frames, kScMonitorScratchFrames));
        }
        meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
        meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
        meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
        meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());
        meterGainReductionA_ = meterGainReductionB_ = 0.0f;
        return;
    }

    // ratio_ drives exactly one side of the threshold at a time -- see
    // kStateRatioDirection's own comment for why this replaced two
    // independently-adjustable ratios.
    const double ratioDown = ratioUpMode_ ? 1.0 : ratio_;
    const double ratioUp = ratioUpMode_ ? ratio_ : 1.0;
    voiceA_.setParams(attackMs_, releaseMs_, thresholdDb_, ratioDown, ratioUp, kneeDb_);

    // B's own bank only actually applies while independent (e.g. Mid/Side,
    // where the two can be set completely differently) -- see
    // isChannelBIndependent()'s own comment; while
    // it isn't, B simply mirrors A's own values, same as this plugin's
    // pre-existing (shared-parameter) behavior. Computed once per block,
    // not per-sample, same as A's own ratioDown/ratioUp above.
    const bool bIndependent = isChannelBIndependent();
    const double attackB   = bIndependent ? attackMsB_   : attackMs_;
    const double releaseB  = bIndependent ? releaseMsB_  : releaseMs_;
    const double thresholdB = bIndependent ? thresholdDbB_ : thresholdDb_;
    const double kneeB     = bIndependent ? kneeDbB_     : kneeDb_;
    const double ratioValB = bIndependent ? ratioB_      : ratio_;
    const bool ratioUpModeBEff = bIndependent ? ratioUpModeB_ : ratioUpMode_;
    const double ratioDownB = ratioUpModeBEff ? 1.0 : ratioValB;
    const double ratioUpB = ratioUpModeBEff ? ratioValB : 1.0;
    voiceB_.setParams(attackB, releaseB, thresholdB, ratioDownB, ratioUpB, kneeB);
    // Per-channel input amplification in Dual-Mono/Mid-Side -- same
    // bIndependent fallback as every other B-bank value (Linked never
    // reaches inGainLinB at all, since isChannelBIndependent() is always
    // false there; Dual-Mono only while unlinked). Applied per-mode
    // inside the switch below, not uniformly here, because Mid/Side needs
    // it applied to the *Mid/Side* components after decode -- see that
    // case's own comment.
    const double inGainLinA = dbToLinear(inputGainDb_);
    const double inGainLinB = dbToLinear(bIndependent ? inputGainDbB_ : inputGainDb_);
    const double scGainLin = dbToLinear(sidechainGainDb_);
    // Makeup Gain now applies per-voice (A always; B only once independent,
    // else it mirrors A -- same fallback as attackB/releaseB/etc. above),
    // "so we can still amplify the two channel individually". Global
    // Output Gain is the separate, always-single master trim that takes
    // over the old single-outGainLin post-sum role once that split
    // happens -- see kParamGlobalOutputGain's own comment
    // (PluginCompressorStereo.hpp).
    const double makeupALin = dbToLinear(outputGainDb_);
    const double makeupBLin = dbToLinear(bIndependent ? makeupGainDbB_ : outputGainDb_);
    const double globalOutGainLin = dbToLinear(globalOutputGainDb_);
    // Wet/Dry per compressor -- same bIndependent fallback again; wetA is always live (every stereo mode,
    // Linked included).
    const double wetA = wetDry_;
    const double wetB = bIndependent ? wetDryB_ : wetDry_;
    // Global Input Gain (before splitting or converting to M/S; not shown
    // in Linked) -- DSP-inert in Linked too, not just hidden in the GUI, same
    // convention as every other Dual-Mono/Mid-Side-only Parameter here.
    const double globalInGainLin = (stereoMode_ != StereoMode::Linked) ? dbToLinear(globalInputGainDb_) : 1.0;

    for (uint32_t i = 0; i < frames; ++i)
    {
        // The signal actually being compressed is always derived from the
        // main input -- only the *detector*'s source switches to the
        // external key input when Sidechain is enabled, fed through
        // exactly the same per-mode derivation (max(|L|,|R|)/per-channel/
        // Mid-Side) the main signal's own detector already uses below,
        // just computed from scL/scR instead. Input Gain deliberately
        // doesn't apply to the key signal; Sidechain Gain is its own
        // independent trim for exactly that reason.
        const double scGLRaw = double(scL[i]) * scGainLin;
        const double scGRRaw = double(scR[i]) * scGainLin;
        double scGL = scGLRaw, scGR = scGRRaw;
        // The frequency filter can be switched off to save DSP --
        // scFilterL_/R_.process() (two one-pole passes each) simply isn't
        // called at all while off, not called-and-ignored.
        if (scFilterEnabled_)
        {
            scGL = scFilterL_.process(scGL);
            scGR = scFilterR_.process(scGR);
        }
        // Sidechain monitor -- always fed regardless of scFilterEnabled_/
        // sidechainEnabled_, see kParamMeterScLevelDbL/R's own comment
        // (PluginCompressorStereo.hpp): per-channel rectified peak
        // (scGL/scGR, "the signal currently reaching the detector") for
        // the scope's own level trace, plus both a pre-filter (raw) and
        // post-filter/current mono-downmixed spectrum, both always shown,
        // like on the EQs.
        scMonitorPeakL_.process(std::fabs(scGL));
        scMonitorPeakR_.process(std::fabs(scGR));
        if (i < kScMonitorScratchFrames)
        {
            scMonitorScratchIn_[i] = float((scGLRaw + scGRRaw) * 0.5);
            scMonitorScratchOut_[i] = float((scGL + scGR) * 0.5);
        }

        double yL, yR;
        double yMid = 0.0, ySide = 0.0; // only meaningful when stereoMode_ == MidSide -- see the Listen override below
        // Mode-appropriate signal for the input meter -- "Raw" (pre any
        // gain, matches this meter's own pre-existing Peak convention) and
        // "Gained" (post Input Gain, matches its own pre-existing RMS/
        // Loudness convention), each either plain L/R or Mid/Side
        // depending on stereoMode_ -- set inside the switch below, read
        // just after it. See kParamInputGainB's own comment for why gain
        // itself also has to move inside this switch now (Mid/Side needs
        // it applied post-decode, not pre-).
        double meterInRawL, meterInRawR, meterInGainedL, meterInGainedR;
        switch (stereoMode_)
        {
        case StereoMode::Linked:
        {
            const double xgL = double(inL[i]) * inGainLinA;
            const double xgR = double(inR[i]) * inGainLinA;
            const double detector = sidechainEnabled_
                ? std::max(std::fabs(scGL), std::fabs(scGR))
                : std::max(std::fabs(xgL), std::fabs(xgR));
            const double grLin = voiceA_.process(detector);
            voiceB_.process(detector); // kept in lock-step purely so its readout mirrors A's, see header comment
            // Wet/Dry per compressor -- standard parallel-compression blend: the gain-staged dry
            // signal (xgL/xgR) at (1-wet), the compressed+made-up signal
            // at wet. wetA==1.0 (the default) reproduces this plugin's
            // own pre-existing always-fully-wet behavior exactly.
            const double wetSigL = xgL * grLin * makeupALin;
            const double wetSigR = xgR * grLin * makeupALin;
            yL = xgL * (1.0 - wetA) + wetSigL * wetA;
            yR = xgR * (1.0 - wetA) + wetSigR * wetA;
            meterInRawL = double(inL[i]); meterInRawR = double(inR[i]);
            meterInGainedL = xgL; meterInGainedR = xgR;
            break;
        }
        case StereoMode::DualMono:
        {
            // Global Input Gain applies first, to the raw stereo pair,
            // before A's/B's own separate per-channel gains -- see
            // kParamGlobalInputGain's own comment. globalInGainLin is
            // exactly 1.0 outside Dual-Mono/Mid-Side, so this is a true
            // no-op there, not just hidden in the GUI.
            const double xgL = double(inL[i]) * globalInGainLin * inGainLinA;
            const double xgR = double(inR[i]) * globalInGainLin * inGainLinB;
            const double grL = voiceA_.process(sidechainEnabled_ ? std::fabs(scGL) : std::fabs(xgL));
            const double grR = voiceB_.process(sidechainEnabled_ ? std::fabs(scGR) : std::fabs(xgR));
            const double wetSigL = xgL * grL * makeupALin;
            const double wetSigR = xgR * grR * makeupBLin;
            yL = xgL * (1.0 - wetA) + wetSigL * wetA;
            yR = xgR * (1.0 - wetB) + wetSigR * wetB;
            meterInRawL = double(inL[i]); meterInRawR = double(inR[i]);
            meterInGainedL = xgL; meterInGainedR = xgR;
            break;
        }
        case StereoMode::MidSide:
        default:
        {
            // Input Gain applies *after* the Mid/Side decode here, not
            // before -- applying A's/B's own separate gains to L/R first
            // would leak into both Mid and Side instead of cleanly
            // amplifying one independently of the other, defeating the
            // entire point of having two (each channel amplified
            // individually, already established for Makeup Gain, now
            // extended to Input Gain too). Global Input Gain, by contrast,
            // applies to the raw L/R pair *before* the decode, so it's woven
            // into gL/gR here, ahead of the mid/side split itself.
            // meterInRawL/R (Peak's own "true incoming level" reading,
            // see below) deliberately stay keyed to the genuinely raw
            // inL[i]/inR[i], not gL/gR -- Global Input Gain is a
            // deliberate amplification stage, not part of "what actually
            // arrived", same reasoning peakInL_/peakInR_ already apply to
            // kParamInputGain/kParamInputGainB elsewhere in this function.
            const double trueRawMid = (double(inL[i]) + double(inR[i])) * 0.5;
            const double trueRawSide = (double(inL[i]) - double(inR[i])) * 0.5;
            const double gL = double(inL[i]) * globalInGainLin;
            const double gR = double(inR[i]) * globalInGainLin;
            const double rawMid = (gL + gR) * 0.5;
            const double rawSide = (gL - gR) * 0.5;
            const double mid = rawMid * inGainLinA;
            const double side = rawSide * inGainLinB;
            const double scMid = (scGL + scGR) * 0.5;
            const double scSide = (scGL - scGR) * 0.5;
            const double grMid = voiceA_.process(sidechainEnabled_ ? std::fabs(scMid) : std::fabs(mid));
            const double grSide = voiceB_.process(sidechainEnabled_ ? std::fabs(scSide) : std::fabs(side));
            const double wetMid = mid * grMid * makeupALin;
            const double wetSide = side * grSide * makeupBLin;
            yMid = mid * (1.0 - wetA) + wetMid * wetA;
            ySide = side * (1.0 - wetB) + wetSide * wetB;
            yL = yMid + ySide;
            yR = yMid - ySide;
            meterInRawL = trueRawMid; meterInRawR = trueRawSide;
            meterInGainedL = mid; meterInGainedR = side;
            break;
        }
        }

        // The input meters show M/S in Mid/Side mode (a real bug: these
        // previously always processed raw L/R regardless
        // of stereo mode, even though the meter's own channel labels
        // already said "M"/"S"). Peak keeps this meter's own pre-existing
        // "pre-gain, true incoming level" convention, RMS/Loudness its own
        // "post-gain, reflects what's actually about to drive the
        // compressor" convention -- both now mode-appropriate.
        peakInL_.process(std::fabs(meterInRawL));
        peakInR_.process(std::fabs(meterInRawR));
        loudnessInL_.process(meterInGainedL); loudnessInR_.process(meterInGainedR);

        // M/S Listen (only Mid or only Side, for setting up the
        // compressor) -- applied here, after
        // the compensation measurement above (which must always see the
        // true stereo mix, never this monitor override, or engaging
        // Listen would silently corrupt the held compensation gain), but
        // before Output Gain/the visible output meters below, both of
        // which should reflect whatever's actually about to reach the
        // speakers.
        if (stereoMode_ == StereoMode::MidSide && listenMode_ != 0)
        {
            // NOT collapsed to mono -- the point is to hear the stereo
            // part only. Decode exactly like the ordinary Mid+Side -> L/R
            // step below, just with the *other* component silenced:
            // Listen: Mid was already correctly mono either way (L and R
            // both equal yMid when ySide is silenced, since Mid itself
            // has no width), but Listen: Side re-decodes to genuine
            // anti-phase stereo (yL=+ySide, yR=-ySide) instead of the
            // same value on both channels -- collapsing Side to mono
            // would have summed it to silence on playback and, on a
            // downstream Phase Scope, read as perfectly in-phase instead
            // of the horizontal line pure side content actually draws.
            const double m = (listenMode_ == 1) ? yMid : 0.0;
            const double s = (listenMode_ == 2) ? ySide : 0.0;
            yL = m + s;
            yR = m - s;
        }

        // Sidechain Listen (pass only the sidechain, to hear the
        // frequencies the detector reacts to) -- same placement
        // reasoning as the M/S Listen override just above (applied after
        // everything that needs the true processed signal, before the
        // meters/Output Gain that should reflect whatever's actually
        // about to reach the speakers), and takes precedence over M/S
        // Listen if both are somehow on at once. Genuine stereo (scGL/
        // scGR directly, not mono-summed) -- same "don't collapse to
        // mono" reasoning as Listen: Side above, so panning/width in the
        // sidechain source is actually audible, not just its level.
        if (scListenEnabled_)
        {
            // Soft-limited: with up to +60dB of trigger gain the raw trigger signal could be
            // enormous -- see sscomp::softLimitListen().
            yL = softLimitListen(scGL);
            yR = softLimitListen(scGR);
        }

        // Global Output Gain is always live, in every mode (revising this
        // feature's original mode-gated design -- it does no harm).
        // Redundant
        // with Makeup Gain (A) in Linked/non-independent mode (both just
        // multiply yL/yR by a scalar at this same point), but harmless --
        // it's a manual master trim the user can reach for regardless of
        // stereo mode, defaulting to 0dB (no-op) until touched.
        yL *= globalOutGainLin;
        yR *= globalOutGainLin;
        // Same M/S fix as the input meter above, applied symmetrically --
        // both bars share the same "M"/"S" labeling in that mode. Derived
        // straight from the final yL/yR (post Listen override, post
        // Global Output Gain) rather than the yMid/ySide computed earlier
        // in the switch, so it always reflects whatever's actually about
        // to reach the speakers, Listen overrides included -- the Mid/
        // Side <-> L/R transform is exactly reversible, so this is exact,
        // not an approximation.
        if (stereoMode_ == StereoMode::MidSide)
        {
            const double mOut = (yL + yR) * 0.5;
            const double sOut = (yL - yR) * 0.5;
            loudnessOutL_.process(mOut); loudnessOutR_.process(sOut);
            peakOutL_.process(std::fabs(mOut)); peakOutR_.process(std::fabs(sOut));
        }
        else
        {
            loudnessOutL_.process(yL); loudnessOutR_.process(yR);
            peakOutL_.process(std::fabs(yL));
            peakOutR_.process(std::fabs(yR));
        }

        outL[i] = float(yL);
        outR[i] = float(yR);
    }

    if (scopeShowing)
    {
        scMonitorSpectrumIn_.process(scMonitorScratchIn_, std::min(frames, kScMonitorScratchFrames));
        scMonitorSpectrumOut_.process(scMonitorScratchOut_, std::min(frames, kScMonitorScratchFrames));
    }

    meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
    meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
    meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
    meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());
    meterDetectorA_ = float(voiceA_.detectorLevelDb());
    meterDetectorB_ = float(voiceB_.detectorLevelDb());
    meterGainReductionA_ = float(voiceA_.gainReductionDb());
    meterGainReductionB_ = (stereoMode_ == StereoMode::Linked) ? meterGainReductionA_ : float(voiceB_.gainReductionDb());
}

void PluginCompressorStereo::sampleRateChanged(double newSampleRate)
{
    voiceA_.prepare(newSampleRate);
    voiceB_.prepare(newSampleRate);
    loudnessInL_.prepare(newSampleRate); loudnessInR_.prepare(newSampleRate);
    loudnessOutL_.prepare(newSampleRate); loudnessOutR_.prepare(newSampleRate);
    peakInL_.prepare(newSampleRate); peakInR_.prepare(newSampleRate);
    peakOutL_.prepare(newSampleRate); peakOutR_.prepare(newSampleRate);
    scFilterL_.prepare(newSampleRate); scFilterR_.prepare(newSampleRate);
    updateScFilterFrequencies();
    scMonitorPeakL_.prepare(newSampleRate); scMonitorPeakR_.prepare(newSampleRate);
    scMonitorSpectrumIn_.prepare(newSampleRate); scMonitorSpectrumOut_.prepare(newSampleRate);
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginCompressorStereo(); }
END_NAMESPACE_DISTRHO
