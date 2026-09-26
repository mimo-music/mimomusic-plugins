// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginReverbDense.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO
using ssfilter::reverb::dense::ReverbEngineDense;
using ssfilter::reverb::RoomDims;
using ssfilter::reverb::TempoNoteValue;

namespace {

// Identical to ../plugin_plate/PluginReverbPlate.cpp's own helper -- a
// small duplicated function, not shared, same reasoning as this file's
// own header comment on why these plugin classes are parallel copies.
double dampingToNyquistT60(double t60DcSec, double damping01)
{
    damping01 = std::clamp(damping01, 0.0, 1.0);
    const double factor = 1.0 - damping01 * 0.9;
    return std::max(0.02, t60DcSec * factor);
}

TempoNoteValue noteValueFromIndex(int index)
{
    const int clamped = std::clamp(index, 0, int(TempoNoteValue::FourBars));
    return TempoNoteValue(clamped);
}

} // namespace

PluginReverbDense::PluginReverbDense()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
}

void PluginReverbDense::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginReverbDense::initParameter(uint32_t index, Parameter& parameter)
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
    case kParamInputGainDb:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Input Gain";
        parameter.symbol = "input_gain";
        parameter.unit   = "dB";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = -24.0f;
        parameter.ranges.max = 24.0f;
        break;
    case kParamDecayTimeSec:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Decay Time";
        parameter.symbol = "decay_time";
        parameter.unit   = "s";
        parameter.ranges.def = 1.8f;
        parameter.ranges.min = 0.1f;
        parameter.ranges.max = 20.0f;
        break;
    case kParamDamping:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Damping";
        parameter.symbol = "damping";
        parameter.ranges.def = 0.5f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamTempoSyncEnabled:
        parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name   = "Tempo Sync";
        parameter.symbol = "tempo_sync_enabled";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamTempoSyncFollowHost:
        parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name   = "Follow Host Tempo Changes";
        parameter.symbol = "tempo_sync_follow_host";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamTempoSyncNoteValue:
        parameter.hints  = kParameterIsAutomatable | kParameterIsInteger;
        parameter.name   = "Tempo Sync Note Value";
        parameter.symbol = "tempo_sync_note_value";
        parameter.ranges.def = float(int(TempoNoteValue::Quarter));
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float(int(TempoNoteValue::FourBars));
        break;
    case kParamDryWet:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Dry/Wet";
        parameter.symbol = "dry_wet";
        parameter.unit   = "%";
        parameter.ranges.def = 35.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 100.0f;
        break;
    case kParamOutputTrimDb:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Output Trim";
        parameter.symbol = "output_trim";
        parameter.unit   = "dB";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = -24.0f;
        parameter.ranges.max = 24.0f;
        break;
    case kParamDiffusion:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Diffusion";
        parameter.symbol = "diffusion";
        parameter.ranges.def = 0.7f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamWidth:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Stereo Spread";
        parameter.symbol = "stereo_spread";
        parameter.ranges.def = 1.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 2.0f;
        break;
    case kParamBalance:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Balance";
        parameter.symbol = "balance";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = -1.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamModulation:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Modulation";
        parameter.symbol = "modulation";
        parameter.ranges.def = 0.3f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamPreDelayMs:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "Pre-Delay";
        parameter.symbol = "pre_delay";
        parameter.unit   = "ms";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float(kMaxPreDelayMs);
        break;
    case kParamUiHeartbeat:
        parameter.hints  = kParameterIsAutomatable;
        parameter.name   = "UI Active (internal)";
        parameter.symbol = "ui_heartbeat";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamHostTempoValid:
        parameter.hints  = kParameterIsAutomatable | kParameterIsBoolean | kParameterIsOutput;
        parameter.name   = "Host Tempo Valid";
        parameter.symbol = "host_tempo_valid";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 1.0f;
        break;
    case kParamHostBpmDetected:
        parameter.hints  = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name   = "Host BPM Detected";
        parameter.symbol = "host_bpm_detected";
        parameter.unit   = "bpm";
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 300.0f;
        break;
    case kParamCurrentDecayTimeSec:
        parameter.hints  = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name   = "Current Decay Time";
        parameter.symbol = "current_decay_time";
        parameter.unit   = "s";
        parameter.ranges.def = 1.8f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = 20.0f;
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
        break;
    }
}

void PluginReverbDense::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStateRoomLengthM:
        state.key = "roomLengthM"; state.label = "Room Length"; state.hints = 0; state.defaultValue = "10.0";
        break;
    case kStateRoomWidthM:
        state.key = "roomWidthM"; state.label = "Room Width"; state.hints = 0; state.defaultValue = "7.0";
        break;
    case kStateRoomHeightM:
        state.key = "roomHeightM"; state.label = "Room Height"; state.hints = 0; state.defaultValue = "3.5";
        break;
    case kStateEqModel:
        state.key = "eqModel"; state.label = "EQ Model"; state.hints = 0; state.defaultValue = "";
        break;
    case kStateSpectrumAutoscale:
        state.key = "spectrumAutoscale"; state.label = "Spectrum Autoscale"; state.hints = 0; state.defaultValue = "0";
        break;
    }
}

float PluginReverbDense::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamBypass:               return bypass_ ? 1.0f : 0.0f;
    case kParamFaultFlag:            return engine_.consumeFaultFlag() ? 1.0f : 0.0f;
    case kParamInputGainDb:          return float(inputGainDb_);
    case kParamDecayTimeSec:         return float(decayTimeSec_);
    case kParamDamping:              return float(damping_);
    case kParamTempoSyncEnabled:     return tempoSyncEnabled_ ? 1.0f : 0.0f;
    case kParamTempoSyncFollowHost:  return followHostTempo_ ? 1.0f : 0.0f;
    case kParamTempoSyncNoteValue:   return float(tempoNoteValueIndex_);
    case kParamDryWet:               return float(dryWet_ * 100.0);
    case kParamOutputTrimDb:         return float(outputTrimDb_);
    case kParamHostTempoValid:       return lastTempoValid_ ? 1.0f : 0.0f;
    case kParamHostBpmDetected:      return float(lastDetectedBpm_);
    case kParamCurrentDecayTimeSec:  return float(engine_.currentT60DcSec());
    case kParamDiffusion:            return float(diffusionAmount_);
    case kParamWidth:                return float(widthAmount_);
    case kParamBalance:              return float(balanceAmount_);
    case kParamModulation:           return float(modulationAmount_);
    case kParamPreDelayMs:           return float(preDelayMs_);
    default:
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return outputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumOutputBase));
        if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
            return inputSpectrumAnalyzer_.getPackedTriple(int(index - kParamSpectrumInputBase));
        return 0.0f;
    }
}

void PluginReverbDense::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamBypass:
        bypass_ = value >= 0.5f;
        break;
    case kParamResetTrigger:
        if (value >= 0.5f) {
            engine_.prepare(room_, sampleRate_, decayTimeSec_, dampingToNyquistT60(decayTimeSec_, damping_));
            engine_.setDiffusion(diffusionAmount_);
            engine_.setModulation(modulationAmount_);
            preDelayL_.reset();
            preDelayR_.reset();
        }
        break;
    case kParamInputGainDb:
        inputGainDb_ = double(value);
        break;
    case kParamDecayTimeSec:
        decayTimeSec_ = double(value);
        applyManualDecayTargets();
        break;
    case kParamDamping:
        damping_ = double(value);
        applyManualDecayTargets();
        break;
    case kParamTempoSyncEnabled:
    {
        const bool enabled = value >= 0.5f;
        if (enabled != tempoSyncEnabled_)
        {
            tempoSyncEnabled_ = enabled;
            if (enabled)
                tempoCaptured_ = false;
            else
                applyManualDecayTargets();
        }
        break;
    }
    case kParamTempoSyncFollowHost:
        followHostTempo_ = value >= 0.5f;
        break;
    case kParamTempoSyncNoteValue:
        tempoNoteValueIndex_ = int(value + 0.5f);
        tempoCaptured_ = false;
        break;
    case kParamDryWet:
        dryWet_ = std::clamp(double(value) * 0.01, 0.0, 1.0);
        break;
    case kParamOutputTrimDb:
        outputTrimDb_ = double(value);
        break;
    case kParamDiffusion:
        diffusionAmount_ = std::clamp(double(value), 0.0, 1.0);
        engine_.setDiffusion(diffusionAmount_);
        break;
    case kParamWidth:
        widthAmount_ = std::clamp(double(value), 0.0, 2.0);
        engine_.setWidth(widthAmount_);
        break;
    case kParamBalance:
        balanceAmount_ = std::clamp(double(value), -1.0, 1.0);
        engine_.setBalance(balanceAmount_);
        break;
    case kParamModulation:
        modulationAmount_ = std::clamp(double(value), 0.0, 1.0);
        engine_.setModulation(modulationAmount_);
        break;
    case kParamPreDelayMs:
        preDelayMs_ = std::clamp(double(value), 0.0, kMaxPreDelayMs);
        preDelayL_.setDelayMs(preDelayMs_);
        preDelayR_.setDelayMs(preDelayMs_);
        break;
    case kParamUiHeartbeat:
        uiHeartbeatSamplesSinceLast_ = 0;
        break;
    }
}

void PluginReverbDense::setState(const char* key, const char* value)
{
    bool roomChanged = false;
    if (std::strcmp(key, "roomLengthM") == 0) { room_.lengthM = std::atof(value); roomChanged = true; }
    else if (std::strcmp(key, "roomWidthM") == 0) { room_.widthM = std::atof(value); roomChanged = true; }
    else if (std::strcmp(key, "roomHeightM") == 0) { room_.heightM = std::atof(value); roomChanged = true; }

    if (roomChanged)
    {
        room_.clamp();
        engine_.requestRoom(room_);
        return;
    }

    if (std::strcmp(key, "eqModel") == 0)
    {
        ssfilter::EqModel m;
        if (ssfilter::eq::fromJson(value, m)) { eqModel_ = m; rebuildEqEngine(); }
        return;
    }
}

void PluginReverbDense::applyManualDecayTargets()
{
    if (tempoSyncEnabled_) return;
    engine_.setDecayTargets(decayTimeSec_, dampingToNyquistT60(decayTimeSec_, damping_));
}

void PluginReverbDense::rebuildEqEngine()
{
    const double sr = sampleRate_ > 0.0 ? sampleRate_ : 48000.0;
    ssfilter::StateSpaceMatrices m = ssfilter::eq::assembleStateSpace(eqModel_, sr);
    ssfilter::StabilityGuard::enforce(m);
    eqEngine_.setModel(m);
}

void PluginReverbDense::run(const float** inputs, float** outputs, uint32_t frames)
{
    {
        const TimePosition& tp = getTimePosition();
        lastTempoValid_ = tp.bbt.valid;
        lastDetectedBpm_ = tp.bbt.beatsPerMinute;
    }

    if (tempoSyncEnabled_)
    {
        const TimePosition& tp = getTimePosition();
        const double bpm = (tp.bbt.valid && tp.bbt.beatsPerMinute > 0.0) ? tp.bbt.beatsPerMinute : 120.0;
        const double beatsPerBar = (tp.bbt.valid && tp.bbt.beatsPerBar > 0.0f) ? double(tp.bbt.beatsPerBar) : 4.0;

        if (followHostTempo_ || !tempoCaptured_)
        {
            const double t60 = ssfilter::reverb::t60SecFromTempo(bpm, noteValueFromIndex(tempoNoteValueIndex_), beatsPerBar);
            engine_.setDecayTargets(t60, dampingToNyquistT60(t60, damping_));
            tempoCaptured_ = true;
        }
    }

    constexpr uint64_t kUiHeartbeatTimeoutSamples = 96000;
    if (uiHeartbeatSamplesSinceLast_ <= UINT64_MAX - frames) uiHeartbeatSamplesSinceLast_ += frames;
    else uiHeartbeatSamplesSinceLast_ = UINT64_MAX;
    const bool uiOpen = uiHeartbeatSamplesSinceLast_ < kUiHeartbeatTimeoutSamples;

    if (bypass_)
    {
        if (outputs[0] != inputs[0]) std::memcpy(outputs[0], inputs[0], sizeof(float) * frames);
        if (outputs[1] != inputs[1]) std::memcpy(outputs[1], inputs[1], sizeof(float) * frames);
        return;
    }

    constexpr uint32_t kChunk = 4096;
    float gainedL[kChunk], gainedR[kChunk];
    float wetInL[kChunk], wetInR[kChunk]; // Pre-Delay applied to these ONLY -- gainedL/R stay immediate for the dry mix below
    const float inputGainLin = std::pow(10.0f, float(inputGainDb_) / 20.0f);
    const float wetGainLin = std::pow(10.0f, float(outputTrimDb_) / 20.0f);
    const float wet = float(dryWet_);
    const float dry = 1.0f - wet;

    for (uint32_t off = 0; off < frames; off += kChunk)
    {
        const uint32_t n = std::min(kChunk, frames - off);
        for (uint32_t i = 0; i < n; ++i)
        {
            gainedL[i] = inputs[0][off + i] * inputGainLin;
            gainedR[i] = inputs[1][off + i] * inputGainLin;
            wetInL[i] = gainedL[i];
            wetInR[i] = gainedR[i];
        }
        preDelayL_.process(wetInL, int(n));
        preDelayR_.process(wetInR, int(n));

        engine_.processBlock(wetInL, wetInR, outputs[0] + off, outputs[1] + off, int(n));

        if (uiOpen) feedSpectrumAnalyzer(inputSpectrumAnalyzer_, outputs[0] + off, outputs[1] + off, n);

        eqEngine_.processBlock(outputs[0] + off, outputs[1] + off, outputs[0] + off, outputs[1] + off, n);

        if (uiOpen) feedSpectrumAnalyzer(outputSpectrumAnalyzer_, outputs[0] + off, outputs[1] + off, n);

        for (uint32_t i = 0; i < n; ++i)
        {
            outputs[0][off + i] = gainedL[i] * dry + outputs[0][off + i] * wetGainLin * wet;
            outputs[1][off + i] = gainedR[i] * dry + outputs[1][off + i] * wetGainLin * wet;
        }
    }
}

void PluginReverbDense::feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames)
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

void PluginReverbDense::sampleRateChanged(double newSampleRate)
{
    sampleRate_ = newSampleRate;
    engine_.prepare(room_, sampleRate_, decayTimeSec_, dampingToNyquistT60(decayTimeSec_, damping_));
    engine_.setDiffusion(diffusionAmount_);
    engine_.setModulation(modulationAmount_);
    preDelayL_.prepare(sampleRate_, kMaxPreDelayMs);
    preDelayR_.prepare(sampleRate_, kMaxPreDelayMs);
    preDelayL_.setDelayMs(preDelayMs_);
    preDelayR_.setDelayMs(preDelayMs_);
    inputSpectrumAnalyzer_.prepare(sampleRate_);
    outputSpectrumAnalyzer_.prepare(sampleRate_);
    tempoCaptured_ = false;

    eqEngine_.prepare(sampleRate_);
    rebuildEqEngine();
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginReverbDense(); }
END_NAMESPACE_DISTRHO
