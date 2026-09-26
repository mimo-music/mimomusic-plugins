// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginAmplifierMono.hpp"

#include <algorithm>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace ssamp;

PluginAmplifierMono::PluginAmplifierMono()
    : Plugin(kParamCount, 0 /* programs */, 0 /* no state -- see this file's .hpp comment */)
{
}

void PluginAmplifierMono::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupMono;
    Plugin::initAudioPort(input, index, port);
}

void PluginAmplifierMono::initParameter(uint32_t index, Parameter& parameter)
{
    switch (index)
    {
    case kParamGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Gain";
        parameter.symbol = "gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -60.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamBypass:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Bypass";
        parameter.symbol = "bypass";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamListenActive:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Listen";
        parameter.symbol = "listen_active";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamMeasureRms:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Measure RMS";
        parameter.symbol = "measure_rms";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamNormalizeTarget:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Normalize Target";
        parameter.symbol = "normalize_target";
        parameter.unit = "dB";
        parameter.ranges.def = -1.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamMeterCurrentLevel:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Measured Level";
        parameter.symbol = "meter_level";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamClipping:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsBoolean;
        parameter.name = "Clipping";
        parameter.symbol = "clipping";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    }
}

float PluginAmplifierMono::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamGain:             return float(gainDb_);
    case kParamBypass:           return bypass_ ? 1.0f : 0.0f;
    case kParamListenActive:     return listening_ ? 1.0f : 0.0f;
    case kParamMeasureRms:       return listenMode_ == ListenMode::Rms ? 1.0f : 0.0f;
    case kParamNormalizeTarget:  return float(normalizeTargetDb_);
    case kParamMeterCurrentLevel:
        return listenMeter_.hasAnySamples() ? float(listenMeter_.readDb(listenMode_)) : -120.0f;
    case kParamClipping: return clipHold_.isClipping() ? 1.0f : 0.0f;
    default: return 0.0f;
    }
}

void PluginAmplifierMono::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamGain:   gainDb_ = value; break;
    case kParamBypass: bypass_ = value >= 0.5f; break;
    case kParamListenActive:
    {
        // Starting a fresh audition -- see run()'s own comment. Deliberately
        // does NOT also compute/apply a new Gain on the *stopping* edge:
        // an early version did that here, directly overwriting gainDb_ --
        // which changed the audio correctly, but a plugin has no standard
        // way to tell an LV2/VST3 host "one of your own input parameters
        // just changed internally" (there is no such half of the LV2
        // control-port protocol; a host only ever pushes values *to* the
        // plugin for an input port, never polls one back), so the host's
        // own displayed/automated Gain value silently stayed wrong even
        // though the sound was already correct. The fix: this plugin only ever measures
        // and reports (kParamMeterCurrentLevel, still updated by run()
        // below); UIAmplifierMono.cpp is what actually decides the new
        // Gain and pushes it via the UI's own setParameterValue(), which
        // *is* a real, reliable "tell the host" mechanism (the same path
        // an ordinary knob turn in this GUI already uses). See that
        // file's own comment on why this has to live there, not here.
        const bool nowListening = value >= 0.5f;
        if (nowListening && !listening_) listenMeter_.reset();
        listening_ = nowListening;
        break;
    }
    case kParamMeasureRms:      listenMode_ = clampListenMode(value >= 0.5f ? 1 : 0); break;
    case kParamNormalizeTarget: normalizeTargetDb_ = value; break;
    default: break;
    }
}

void PluginAmplifierMono::run(const float** inputs, float** outputs, uint32_t frames)
{
    const float* in = inputs[0];
    float* out = outputs[0];

    // Wall-clock length of this block -- see ClipHold::update()'s own
    // comment for why the hold window is measured in time, not frames.
    const double blockSeconds = double(frames) / std::max(1.0, getSampleRate());

    if (listening_)
    {
        // Audition the raw, unprocessed input while measuring it --
        // Listen's whole point is judging the SOURCE's own level before
        // deciding a Gain, not whatever Gain/Bypass happen to already be
        // set, so this deliberately skips both of those (and Bypass's own
        // meaning is moot here anyway: this already *is* a passthrough).
        if (out != in) std::memcpy(out, in, sizeof(float) * frames);
        for (uint32_t i = 0; i < frames; ++i)
            listenMeter_.process(double(in[i]));
        clipHold_.update(ClipHold::bufferClips(in, frames), blockSeconds);
        return;
    }

    if (bypass_)
    {
        if (out != in) std::memcpy(out, in, sizeof(float) * frames);
        clipHold_.update(ClipHold::bufferClips(in, frames), blockSeconds);
        return;
    }

    const float gainLin = float(dbToLinear(gainDb_));
    for (uint32_t i = 0; i < frames; ++i)
        out[i] = in[i] * gainLin;
    clipHold_.update(ClipHold::bufferClips(out, frames), blockSeconds);
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginAmplifierMono(); }
END_NAMESPACE_DISTRHO
