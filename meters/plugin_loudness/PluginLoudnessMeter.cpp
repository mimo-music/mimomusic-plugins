// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginLoudnessMeter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace ssmeter;

PluginLoudnessMeter::PluginLoudnessMeter()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
}

void PluginLoudnessMeter::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginLoudnessMeter::initParameter(uint32_t index, Parameter& parameter)
{
    // Shared by both Distribution bin banks (Short-term and Momentary --
    // see kParamDistBinMomentaryBase's own comment) -- identical Parameter
    // shape, just a different name/symbol prefix and base offset.
    const auto initDistBin = [&](uint32_t base, const char* prefix, const char* symbolPrefix) {
        const uint32_t bin = index - base;
        char name[32], symbol[24];
        std::snprintf(name, sizeof(name), "%s %u", prefix, bin);
        std::snprintf(symbol, sizeof(symbol), "%s_%u", symbolPrefix, bin);
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
        parameter.name = name;
        parameter.symbol = symbol;
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        // A real ceiling, not a token "unbounded" one -- ~8 minutes'
        // worth of 100ms gating blocks concentrated in a single bin is
        // already an extreme, unlikely-in-practice case for real program
        // material. An earlier version declared this as 1e9 ("plain,
        // unbounded-in-practice"), which is bad practice regardless of
        // whether it actually explains "still missing the statistic
        // graphic" -- some hosts scale an output Parameter's own
        // change-significance display/automation behavior against its
        // declared range, and a `+1` count against a billion-wide range
        // is about as insignificant a relative change as a float can
        // express.
        parameter.ranges.max = 5000.0f;
    };

    if (index >= kParamDistBinBase && index < kParamDistBinBase + uint32_t(LufsMeter::kDistributionBinCount))
    {
        initDistBin(kParamDistBinBase, "Dist ST Bin", "dist_st_bin");
        return;
    }
    if (index >= kParamDistBinMomentaryBase && index < kParamDistBinMomentaryBase + uint32_t(LufsMeter::kDistributionBinCount))
    {
        initDistBin(kParamDistBinMomentaryBase, "Dist Mom Bin", "dist_mom_bin");
        return;
    }

    switch (index)
    {
    case kParamMomentaryLufs:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Momentary";
        parameter.symbol = "momentary_lufs";
        parameter.unit = "LUFS";
        parameter.ranges.def = float(LufsMeter::kSilenceLufs);
        parameter.ranges.min = float(LufsMeter::kSilenceLufs);
        parameter.ranges.max = 5.0f;
        break;
    case kParamShortTermLufs:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Short-term";
        parameter.symbol = "short_term_lufs";
        parameter.unit = "LUFS";
        parameter.ranges.def = float(LufsMeter::kSilenceLufs);
        parameter.ranges.min = float(LufsMeter::kSilenceLufs);
        parameter.ranges.max = 5.0f;
        break;
    case kParamIntegratedLufs:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Integrated";
        parameter.symbol = "integrated_lufs";
        parameter.unit = "LUFS";
        parameter.ranges.def = float(LoudnessHistogram::kMinLufs);
        parameter.ranges.min = float(LoudnessHistogram::kMinLufs);
        parameter.ranges.max = 5.0f;
        break;
    case kParamLoudnessRangeLu:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Loudness Range";
        parameter.symbol = "loudness_range_lu";
        parameter.unit = "LU";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 30.0f;
        break;
    case kParamTruePeakL:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "True Peak L";
        parameter.symbol = "true_peak_l";
        parameter.unit = "dBTP";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamTruePeakR:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "True Peak R";
        parameter.symbol = "true_peak_r";
        parameter.unit = "dBTP";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamTruePeakMax:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "True Peak Max";
        parameter.symbol = "true_peak_max";
        parameter.unit = "dBTP";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamPlrLu:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "PLR";
        parameter.symbol = "plr_lu";
        parameter.unit = "LU";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 60.0f;
        break;
    case kParamPsrLu:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "PSR";
        parameter.symbol = "psr_lu";
        parameter.unit = "LU";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 60.0f;
        break;
    case kParamInputRmsL:
    case kParamInputPeakL:
    case kParamInputRmsR:
    case kParamInputPeakR:
    {
        static const char* names[] = { "Input RMS L", "Input Peak L", "Input RMS R", "Input Peak R" };
        static const char* symbols[] = { "input_rms_l", "input_peak_l", "input_rms_r", "input_peak_r" };
        const uint32_t i = index - kParamInputRmsL;
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = names[i];
        parameter.symbol = symbols[i];
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    }
    }
}

void PluginLoudnessMeter::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStateTargetLufs:
        state.key = "targetLufs";
        state.label = "Target Reference Line";
        state.hints = 0; // UI convenience only -- see kStateTargetLufs's own comment
        state.defaultValue = "-23.0"; // EBU R128, the most broadcast-ubiquitous target
        break;
    case kStateShowDistribution:
        state.key = "showDistribution";
        state.label = "Show Distribution View";
        state.hints = 0; // UI convenience only -- see kStateShowDistribution's own comment
        state.defaultValue = "0";
        break;
    case kStateResetRequest:
        state.key = "resetRequest";
        state.label = "Reset Request";
        state.hints = kStateIsOnlyForDSP; // see kStateResetRequest's own comment
        state.defaultValue = "0";
        break;
    }
}

float PluginLoudnessMeter::getParameterValue(uint32_t index) const
{
    if (index >= kParamDistBinBase && index < kParamDistBinBase + uint32_t(LufsMeter::kDistributionBinCount))
        return float(meter_.distributionShortTermCount(int(index - kParamDistBinBase)));
    if (index >= kParamDistBinMomentaryBase && index < kParamDistBinMomentaryBase + uint32_t(LufsMeter::kDistributionBinCount))
        return float(meter_.distributionMomentaryCount(int(index - kParamDistBinMomentaryBase)));

    switch (index)
    {
    case kParamMomentaryLufs:    return momentaryLufs_;
    case kParamShortTermLufs:    return shortTermLufs_;
    case kParamIntegratedLufs:   return integratedLufs_;
    case kParamLoudnessRangeLu:  return loudnessRangeLu_;
    case kParamTruePeakL:        return truePeakL_;
    case kParamTruePeakR:        return truePeakR_;
    case kParamTruePeakMax:      return truePeakMax_;
    case kParamPlrLu:            return plrLu_;
    case kParamPsrLu:            return psrLu_;
    case kParamInputRmsL:  return inputRmsL_;
    case kParamInputPeakL: return inputPeakL_;
    case kParamInputRmsR:  return inputRmsR_;
    case kParamInputPeakR: return inputPeakR_;
    default: return 0.0f;
    }
}

void PluginLoudnessMeter::setParameterValue(uint32_t index, float value)
{
    (void)index; (void)value;
    // Every Parameter here is output-only -- a host isn't expected to
    // write them, but silently accepting (and ignoring) rather than
    // asserting is the same tolerant convention this project's other
    // output-only banks already use. "Reset" itself is a State now
    // (kStateResetRequest), not a Parameter -- see that enum entry's own
    // comment for why.
}

void PluginLoudnessMeter::setState(const char* key, const char* value)
{
    (void)value;
    // "targetLufs"/"showDistribution" are pure UI preferences -- nothing
    // for the DSP side to do, see their own StateIndex comments.
    if (std::strcmp(key, "resetRequest") == 0)
        meter_.resetStatistics();
}

void PluginLoudnessMeter::activate()
{
    meter_.reset();
    rmsL_.reset(); rmsR_.reset();
    peakL_.reset(); peakR_.reset();
}

void PluginLoudnessMeter::run(const float** inputs, float** outputs, uint32_t frames)
{
    const float* inL = inputs[0];
    const float* inR = inputs[1];
    float* outL = outputs[0];
    float* outR = outputs[1];

    // Purely observational -- see this file's own header comment. Copy
    // through untouched before measuring, so a host that reads the output
    // buffer for its own metering sees exactly the input, and so that
    // TruePeakDetector's own internal lookahead latency (see its own file
    // comment) never delays anything in the actual audio path.
    if (outL != inL) std::copy(inL, inL + frames, outL);
    if (outR != inR) std::copy(inR, inR + frames, outR);

    for (uint32_t i = 0; i < frames; ++i)
    {
        const double l = double(inL[i]), r = double(inR[i]);
        meter_.process(l, r);
        // Plain, non-K-weighted per-channel ballistics -- see
        // kParamInputRmsL's own comment.
        rmsL_.process(l); rmsR_.process(r);
        peakL_.process(std::fabs(l)); peakR_.process(std::fabs(r));
    }

    momentaryLufs_ = float(meter_.momentaryLufs());
    shortTermLufs_ = float(meter_.shortTermLufs());
    integratedLufs_ = float(meter_.integratedLufs());
    loudnessRangeLu_ = float(meter_.loudnessRangeLu());
    truePeakL_ = float(meter_.truePeakLDb());
    truePeakR_ = float(meter_.truePeakRDb());
    truePeakMax_ = float(meter_.truePeakMaxDb());
    plrLu_ = float(meter_.plrLu());
    psrLu_ = float(meter_.psrLu());
    inputRmsL_ = float(rmsL_.rmsDb());
    inputPeakL_ = float(peakL_.peakDb());
    inputRmsR_ = float(rmsR_.rmsDb());
    inputPeakR_ = float(peakR_.peakDb());
}

void PluginLoudnessMeter::sampleRateChanged(double newSampleRate)
{
    meter_.prepare(newSampleRate);
    rmsL_.prepare(newSampleRate); rmsR_.prepare(newSampleRate);
    peakL_.prepare(newSampleRate); peakR_.prepare(newSampleRate);
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginLoudnessMeter(); }
END_NAMESPACE_DISTRHO
