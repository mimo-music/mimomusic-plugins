// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/AmplifierDsp.hpp"

START_NAMESPACE_DISTRHO

// Has a custom GUI (gui/UIAmplifierStereo.*): the Listen/Normalize feature
// below needs a live level readout and start/stop control a host's
// generic parameter editor can't give it. No State: every control, including Listen itself, is an
// ordinary automatable Parameter. Invert L/Invert R are two independent
// toggles (not one combined polarity flip) -- each channel can be
// inverted on its own, both, or neither.
enum ParameterIndex
{
    kParamGain = 0,
    kParamInvertL,
    kParamInvertR,
    kParamBalance,
    kParamBypass,

    // Listen/Normalize -- see PluginAmplifierStereo.cpp's run()/
    // setParameterValue() own comments for the full mechanism: Listen
    // auditions the raw (Linked, max(|L|,|R|)-for-Peak /
    // both-channels-for-RMS) input and measures it for as long as it's
    // on; turning it back off computes the Gain that would bring that
    // measured level to NormalizeTarget and applies it immediately.
    kParamListenActive,     // trigger-like boolean: 1 while auditioning/measuring, 0 when stopped (the 1->0 edge is what applies the new Gain)
    kParamMeasureRms,       // boolean: false=Peak (default), true=RMS
    kParamNormalizeTarget,  // dB -- the level Listen normalizes the measured signal to once stopped
    kParamMeterCurrentLevel, // output: the live (or last-measured) Peak/RMS reading, dB -- for the GUI

    // Always live, independent of Listen -- see ssamp::ClipHold's own
    // comment (AmplifierDsp.hpp) for why this couldn't just reuse
    // kParamMeterCurrentLevel above.
    kParamClipping, // output: boolean, true while either channel leaving this plugin is at/above 0dBFS (plus a short hold afterward)

    kParamCount
};

class PluginAmplifierStereo : public Plugin
{
public:
    PluginAmplifierStereo();

protected:
    uint32_t getVersion() const override { return d_version(1, 1, 0); } // 1.1: Listen/Normalize + a custom GUI

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void run(const float** inputs, float** outputs, uint32_t frames) override;

private:
    double gainDb_ = 0.0;
    double balance_ = 0.0; // -1 (full left) .. +1 (full right), see ssamp::balanceGains()
    bool invertL_ = false;
    bool invertR_ = false;
    bool bypass_ = false;

    bool listening_ = false;
    ssamp::ListenMode listenMode_ = ssamp::ListenMode::Peak;
    double normalizeTargetDb_ = -1.0;
    ssamp::ListenMeter listenMeter_;
    ssamp::ClipHold clipHold_;

    DISTRHO_DECLARE_NON_COPYABLE(PluginAmplifierStereo)
};

END_NAMESPACE_DISTRHO
