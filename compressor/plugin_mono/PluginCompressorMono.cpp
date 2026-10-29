// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginCompressorMono.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace sscomp;

PluginCompressorMono::PluginCompressorMono()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
}

void PluginCompressorMono::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    // Index 0 is the main signal, initialized the ordinary way below.
    // Index 1 (input only -- DISTRHO_PLUGIN_NUM_OUTPUTS is still 1) is the
    // external sidechain/key input: flagged kAudioPortIsSidechain so LV2/
    // VST3/CLAP hosts (Ardour included) present it as a separate,
    // independently-routable auxiliary input rather than folding it into
    // the main audio bus.
    if (input && index == 1)
    {
        port.hints = kAudioPortIsSidechain;
        port.name = "Sidechain";
        port.symbol = "sidechain_in";
        return;
    }
    Plugin::initAudioPort(input, index, port);
}

void PluginCompressorMono::initParameter(uint32_t index, Parameter& parameter)
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
        // and index untouched, only the display name changes -- see
        // PluginCompressorStereo.cpp's identical comment on its own
        // kParamOutputGain for why that's fully session-compatible.
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Makeup Gain";
        parameter.symbol = "output_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    // Removed -- see kParamMeterCompGainDb's own comment
    // (PluginCompressorMono.hpp) for why these stay declared instead of
    // being deleted. Names marked "(unused)" so they read as vestigial if
    // anything shows them outside this plugin's own GUI.
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

    case kParamMeterInputRms:
    case kParamMeterInputPeak:
    case kParamMeterOutputRms:
    case kParamMeterOutputPeak:
    case kParamMeterDetectorDb:
    {
        static const char* names[]  = { "Input RMS", "Input Peak", "Output RMS", "Output Peak", "Detector Level" };
        static const char* symbols[] = { "meter_in_rms", "meter_in_peak", "meter_out_rms", "meter_out_peak", "meter_detector" };
        const uint32_t i = index - kParamMeterInputRms;
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = names[i];
        parameter.symbol = symbols[i];
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    }
    case kParamMeterGainReductionDb:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Gain Reduction";
        parameter.symbol = "meter_gain_reduction";
        parameter.unit = "dB";
        // Range widened from the original -48..0 (downward-only) to also
        // cover a positive reading -- Up direction (kStateRatioDirection)
        // can report a *boost*, not just a reduction.
        parameter.ranges.def = 0.0f; parameter.ranges.min = -48.0f; parameter.ranges.max = 24.0f;
        break;
    // Always report 0 now.
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
    }
}

void PluginCompressorMono::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStatePresetDir:
        state.key = "presetDir";
        state.label = "Last Preset Directory";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "";
        break;
    case kStateGrMeterSpeed:
        state.key = "grMeterSpeed";
        state.label = "GR Meter Speed (Fast/Slow)";
        state.hints = 0; // UI preference only, the DSP side never reads this
        state.defaultValue = "0"; // Fast
        break;
    case kStateRatioDirection:
        state.key = "ratioDirection";
        state.label = "Ratio Direction (Down/Up)";
        state.hints = 0; // DSP-relevant (see setState()), but a discrete two-way choice, not a continuous automatable control
        state.defaultValue = "0"; // Down -- the historical/default behavior
        break;
    }
}

float PluginCompressorMono::getParameterValue(uint32_t index) const
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
    case kParamCompensation: return compensationEnabled_ ? 1.0f : 0.0f;
    case kParamBypass:       return bypass_ ? 1.0f : 0.0f;
    case kParamSidechainEnable: return sidechainEnabled_ ? 1.0f : 0.0f;
    case kParamSidechainGain:   return float(sidechainGainDb_);

    case kParamMeterInputRms:        return meterInputRms_;
    case kParamMeterInputPeak:       return meterInputPeak_;
    case kParamMeterOutputRms:       return meterOutputRms_;
    case kParamMeterOutputPeak:      return meterOutputPeak_;
    case kParamMeterDetectorDb:      return meterDetectorDb_;
    case kParamMeterGainReductionDb: return meterGainReductionDb_;
    case kParamMeterCompGainDb:      return 0.0f;
    case kParamMeterCompState:       return 0.0f;
    default: return 0.0f;
    }
}

void PluginCompressorMono::setParameterValue(uint32_t index, float value)
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
    // Inert -- see kParamMeterCompGainDb's own comment (initParameter()).
    case kParamCompensation: compensationEnabled_ = value >= 0.5f; break;
    case kParamCompensationMeasure: break;
    case kParamBypass: bypass_ = value >= 0.5f; break;
    case kParamSidechainEnable: sidechainEnabled_ = value >= 0.5f; break;
    case kParamSidechainGain:   sidechainGainDb_ = value; break;
    default: break; // meter outputs are read-only from the host's perspective
    }
}

void PluginCompressorMono::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "ratioDirection") == 0)
    {
        ratioUpMode_ = std::atoi(value) != 0;
        return;
    }
    // "presetDir" is a pure UI preference -- DPF's base Plugin class
    // remembers whatever string was last passed to setState() for any key
    // and hands it back to the host for session persistence on its own,
    // so simply not matching it here is enough; the DSP side has nothing
    // to do with it.
}

void PluginCompressorMono::activate()
{
    voice_.reset();
    loudnessIn_.reset();
    loudnessOut_.reset();
    peakIn_.reset();
    peakOut_.reset();
    grMeter_.reset();
}

void PluginCompressorMono::run(const float** inputs, float** outputs, uint32_t frames)
{
    const float* in = inputs[0];
    const float* sc = inputs[1]; // external key input -- always a valid buffer (host-connected audio, or silence if unconnected/inactive), see initAudioPort()
    float* out = outputs[0];

    if (bypass_)
    {
        if (out != in) std::copy(in, in + frames, out);
        // Meters still reflect what's actually happening (input == output
        // while bypassed) rather than freezing at a stale reading.
        for (uint32_t i = 0; i < frames; ++i)
        {
            peakIn_.process(std::fabs(double(in[i])));
            peakOut_.process(std::fabs(double(in[i])));
            loudnessIn_.process(double(in[i]));
            loudnessOut_.process(double(in[i]));
        }
        meterInputRms_ = float(loudnessIn_.rmsDb());
        meterInputPeak_ = float(peakIn_.peakDb());
        meterOutputRms_ = float(loudnessOut_.rmsDb());
        meterOutputPeak_ = float(peakOut_.peakDb());
        grMeter_.reset();
        meterGainReductionDb_ = 0.0f;
        return;
    }

    // ratio_ drives exactly one side of the threshold at a time -- see
    // kStateRatioDirection's own comment for why this replaced two
    // independently-adjustable ratios.
    const double ratioDown = ratioUpMode_ ? 1.0 : ratio_;
    const double ratioUp = ratioUpMode_ ? ratio_ : 1.0;
    voice_.setParams(attackMs_, releaseMs_, thresholdDb_, ratioDown, ratioUp, kneeDb_);
    const double inGainLin = dbToLinear(inputGainDb_);
    const double outGainLin = dbToLinear(outputGainDb_);
    const double scGainLin = dbToLinear(sidechainGainDb_);

    for (uint32_t i = 0; i < frames; ++i)
    {
        const double rawIn = double(in[i]);
        peakIn_.process(std::fabs(rawIn));

        const double xg = rawIn * inGainLin;
        // The signal actually being compressed is always the main input
        // (xg, above) -- only the *detector*'s source switches to the
        // external key signal when Sidechain is enabled, exactly like a
        // hardware compressor's external-key switch. Input Gain
        // deliberately doesn't apply to the key signal; Sidechain Gain is
        // its own independent trim for exactly that reason.
        const double detectorSample = sidechainEnabled_ ? (double(sc[i]) * scGainLin) : xg;
        const double grLin = voice_.process(std::fabs(detectorSample));
        double y = xg * grLin;
        grMeter_.process(voice_.gainReductionDb());

        loudnessIn_.process(xg);
        loudnessOut_.process(y);

        y *= outGainLin;
        peakOut_.process(std::fabs(y));
        out[i] = float(y);
    }

    meterInputRms_ = float(loudnessIn_.rmsDb());
    meterInputPeak_ = float(peakIn_.peakDb());
    meterOutputRms_ = float(loudnessOut_.rmsDb());
    meterOutputPeak_ = float(peakOut_.peakDb());
    meterDetectorDb_ = float(voice_.detectorLevelDb());
    meterGainReductionDb_ = float(grMeter_.gainDb());
}

void PluginCompressorMono::sampleRateChanged(double newSampleRate)
{
    voice_.prepare(newSampleRate);
    loudnessIn_.prepare(newSampleRate);
    loudnessOut_.prepare(newSampleRate);
    peakIn_.prepare(newSampleRate);
    peakOut_.prepare(newSampleRate);
    grMeter_.prepare(newSampleRate);
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginCompressorMono(); }
END_NAMESPACE_DISTRHO
