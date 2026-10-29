// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"

START_NAMESPACE_DISTRHO

// A small, fixed bank of automatable Parameters -- unlike
// statespace's matrix-editor plugins, a compressor's
// entire configuration *is* seven scalar knobs plus two toggles, so there's
// no reason to route it through custom state/JSON the way that project's
// unbounded-shape models need to; ordinary host-automatable Parameters are
// both simpler and give the host (Ardour) automation lanes for free.
//
// The meter outputs (kParamMeterInputRms.. onward) are *output* Parameters
// -- the same portable, guaranteed-cross-process mechanism
// statespace's kParamFaultFlag / spectrum bands use,
// since there is no direct shared-memory link between the DSP and UI sides
// in general (can be different processes for some formats/hosts).
enum ParameterIndex
{
    kParamInputGain = 0,
    kParamThreshold,
    kParamRatio,
    kParamAttack,
    kParamRelease,
    kParamKnee,
    kParamOutputGain,
    kParamCompensation,
    kParamCompensationMeasure, // trigger: force a fresh compensation measurement now, see PluginCompressorMono::setParameterValue()
    kParamBypass,

    kParamMeterInputRms,
    kParamMeterInputPeak,
    kParamMeterOutputRms,
    kParamMeterOutputPeak,
    kParamMeterDetectorDb,     // level (dB) arriving at the gain computer, post input-gain -- the transfer graph's live x position
    kParamMeterGainReductionDb,
    // Volume Compensation removed (Makeup Gain, Global Output Gain and the
    // GR-meter net bar made it unnecessary). These four indices (this pair, plus
    // kParamCompensation/kParamCompensationMeasure above) stay declared
    // at their exact existing positions rather than being deleted --
    // removing/reordering them would shift every Parameter after them,
    // silently rebinding any automation lane already saved against those
    // indices in a real project (see PluginCompressorStereo.cpp's
    // identical, empirically-verified comment on its own
    // kParamCompensation for the full reasoning). Always 0 now.
    kParamMeterCompGainDb,
    kParamMeterCompState,

    // Appended after the original bank rather than slotted next to
    // kParamInputGain/kParamThreshold (their conceptual neighbors) -- same
    // reasoning as kParamRatioUp's own append-only history further down
    // this file: an existing Parameter must keep its numeric index, since
    // some hosts/formats key automation by it.
    //
    // External sidechain/key input: DISTRHO_PLUGIN_NUM_INPUTS is 2 (main +
    // sidechain, see DistrhoPluginInfo.h and initAudioPort()) so the extra
    // port is always physically present, but it only actually drives the
    // detector while this is on -- the second input port otherwise carries
    // whatever the host connects (or nothing/silence) with zero effect.
    // The compressed signal itself is always the main input; only the
    // *level detector*'s source switches, exactly like a hardware
    // compressor's "key listen"/"external key" switch.
    kParamSidechainEnable,
    kParamSidechainGain,       // dB trim applied to the external key signal before it reaches the detector, independent of Input Gain (which only affects the signal actually being compressed)

    kParamCount
};

enum StateIndex
{
    kStatePresetDir = 0, // last folder used in the Save As.../Load... dialogs (UI convenience only, DSP never reads this)
    // Which side of the threshold kParamRatio actually compresses --
    // "0"=Down (the historical/default behavior, compresses level above
    // the threshold), "1"=Up (compresses below it instead, lifting quiet
    // material). Replaces a since-removed second parameter (kParamRatioUp)
    // that could be dialed in *simultaneously* with kParamRatio, which
    // read as two independent, always-both-active compressors rather than
    // "pick a direction".
    // A discrete two-way choice, so it's a State (toggled by a button) and
    // not a continuous automatable Parameter, matching the Stereo
    // plugin's own kStateStereoMode convention.
    kStateRatioDirection,
    // GR meter speed in the editor, "0"=Fast/"1"=Slow (UI preference only,
    // the DSP never reads it -- same as presetDir).
    kStateGrMeterSpeed,
    kStateCount
};

class PluginCompressorMono : public Plugin
{
public:
    PluginCompressorMono();

protected:
    uint32_t getVersion() const override { return d_version(2, 1, 0); } // 2.1: sidechain/external key input added (kParamSidechainEnable/Gain, appended -- existing indices unchanged)

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    sscomp::CompressorVoice voice_;
    sscomp::LoudnessMeter loudnessIn_, loudnessOut_;
    sscomp::PeakMeter peakIn_, peakOut_;
    sscomp::GrPeakMeter grMeter_; // the GR readout -- see its own comment

    double inputGainDb_ = 0.0;
    double thresholdDb_ = -18.0;
    double ratio_ = 4.0;
    bool ratioUpMode_ = false; // false=Down (compress above threshold), true=Up (compress below it) -- see kStateRatioDirection's own comment
    double attackMs_ = 10.0;
    double releaseMs_ = 150.0;
    double kneeDb_ = 6.0;
    double outputGainDb_ = 0.0;
    // Vestigial -- round-trips kParamCompensation's own stored value, but
    // nothing reads it any more. See that Parameter's own comment
    // (initParameter(), PluginCompressorMono.cpp).
    bool compensationEnabled_ = false;
    bool bypass_ = false;
    bool sidechainEnabled_ = false;
    double sidechainGainDb_ = 0.0;

    // Last-block meter readings, handed back through getParameterValue()
    // (an *output* Parameter is read by the host/UI, not pushed).
    float meterInputRms_ = -120.0f, meterInputPeak_ = -120.0f;
    float meterOutputRms_ = -120.0f, meterOutputPeak_ = -120.0f;
    float meterDetectorDb_ = -120.0f, meterGainReductionDb_ = 0.0f;

    DISTRHO_DECLARE_NON_COPYABLE(PluginCompressorMono)
};

END_NAMESPACE_DISTRHO
