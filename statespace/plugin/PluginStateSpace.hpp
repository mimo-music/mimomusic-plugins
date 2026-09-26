// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "dsp/StateSpaceEngine.hpp"

START_NAMESPACE_DISTRHO

// Parameter indices. Kept intentionally small: the actual filter
// configuration (matrix order + every A/B/C/D coefficient) travels as the
// "model" state (see initState()/setState()), not as DPF Parameters --
// there is no fixed, host-automatable knob layout that would make sense
// for an arbitrarily-sized coefficient matrix. What *is* exposed as real
// parameters are the handful of things that behave like normal, continuous,
// automatable controls regardless of what the matrices contain.
enum ParameterIndex
{
    kParamBypass = 0,       // in:  hard bypass toggle
    kParamResetTrigger = 1, // in:  momentary trigger, zeroes the internal state vector x[n]
    kParamFaultFlag = 2,    // out: pulses to 1 for one poll after an auto-recovered NaN/Inf fault
    kParamCount
};

// State indices.
enum StateIndex
{
    kStateModel = 0,       // the full A/B/C/D model, see dsp/Serialization.hpp
    kStatePresetDir = 1,   // last folder used in the Load/Save Preset file dialogs (UI convenience only)
    kStateBodeVisible = 2, // which of the 4 Bode plot curves are shown, see BodePlotWidget (UI convenience only)
    kStateCount
};

class PluginStateSpace : public Plugin
{
public:
    PluginStateSpace();

protected:
    // -- Information -----------------------------------------------------
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    // -- Init -------------------------------------------------------------
    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    // -- Parameters ---------------------------------------------------
    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    // -- State (the actual filter configuration lives here) ------------
    // No getState() override: with DISTRHO_PLUGIN_WANT_FULL_STATE left off
    // (we don't use host programs), DPF's base Plugin class remembers
    // whatever string was last passed to setState() and hands it back to
    // the host for session persistence on its own.
    void setState(const char* key, const char* value) override;

    // -- Process ------------------------------------------------------
    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    // mutable: getParameterValue() is const but the output fault-flag
    // parameter is a one-shot "consume and clear" read (see
    // StateSpaceEngine::consumeFaultFlag()), which is a legitimate
    // side effect of an otherwise-const query, same as a mutable cache.
    mutable ssfilter::StateSpaceEngine engine_;
    bool bypass_ = false;

    DISTRHO_DECLARE_NON_COPYABLE(PluginStateSpace)
};

END_NAMESPACE_DISTRHO
