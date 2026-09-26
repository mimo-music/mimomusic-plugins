// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginStateSpace.hpp"

#include "dsp/Serialization.hpp"
#include "dsp/StabilityGuard.hpp"

#include <cstring>

USE_NAMESPACE_DISTRHO

PluginStateSpace::PluginStateSpace()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    // Engine starts on the safe identity-passthrough model until a preset
    // is loaded / host state is restored, see StateSpaceMatrices' default
    // constructor and initState() below.
    sampleRateChanged(getSampleRate());
}

void PluginStateSpace::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginStateSpace::initParameter(uint32_t index, Parameter& parameter)
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
    }
}

void PluginStateSpace::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStateModel:
        state.key   = "model";
        state.label = "State-space model";
        state.hints = 0; // private DSP<->UI/session data, not a host-generic-editable field
        state.defaultValue = ssfilter::Serialization::toJson(ssfilter::StateSpaceMatrices()).c_str();
        break;
    case kStatePresetDir:
        state.key   = "presetDir";
        state.label = "Last Preset Directory";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "";
        break;
    case kStateBodeVisible:
        state.key   = "bodeVisible";
        state.label = "Bode Plot Curve Visibility";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "1001"; // LL, RL, LR, RR -- direct paths on, cross-channel off
        break;
    }
}

float PluginStateSpace::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamBypass:     return bypass_ ? 1.0f : 0.0f;
    case kParamFaultFlag:  return engine_.consumeFaultFlag() ? 1.0f : 0.0f;
    default:               return 0.0f;
    }
}

void PluginStateSpace::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamBypass:
        bypass_ = value >= 0.5f;
        break;
    case kParamResetTrigger:
        if (value >= 0.5f) engine_.resetState();
        break;
    default:
        break;
    }
}

void PluginStateSpace::setState(const char* key, const char* value)
{
    // "presetDir" and "bodeVisible" are UI convenience data -- the DSP side
    // has nothing to do with either beyond letting DPF's own state
    // machinery remember and persist them, so neither is matched below and
    // both simply fall through to the early return.
    if (std::strcmp(key, "model") != 0) return;

    ssfilter::StateSpaceMatrices m;
    if (!ssfilter::Serialization::fromJson(value, m))
        return; // malformed/foreign data: keep whatever the engine currently has

    // Defense in depth: even though the GUI and preset loader already run
    // every model through the stability guard before it is ever serialized,
    // a state string can also arrive from a host session file that was
    // hand-edited, corrupted, or written by a future/incompatible version
    // of this plugin -- so re-validate here too, right before it reaches
    // the real-time engine.
    ssfilter::StabilityGuard::enforce(m);
    engine_.setModel(m);
}

void PluginStateSpace::activate()
{
    engine_.resetState();
}

void PluginStateSpace::run(const float** inputs, float** outputs, uint32_t frames)
{
    if (bypass_)
    {
        if (outputs[0] != inputs[0]) std::memcpy(outputs[0], inputs[0], sizeof(float) * frames);
        if (outputs[1] != inputs[1]) std::memcpy(outputs[1], inputs[1], sizeof(float) * frames);
        return;
    }

    engine_.processBlock(inputs[0], inputs[1], outputs[0], outputs[1], frames);
}

void PluginStateSpace::sampleRateChanged(double newSampleRate)
{
    engine_.prepare(newSampleRate);
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginStateSpace(); }
END_NAMESPACE_DISTRHO
