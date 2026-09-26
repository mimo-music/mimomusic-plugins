// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/ReverbParams.hpp" // ParameterIndex/StateIndex, shared with gui/UIReverb.hpp and every sibling model's own PluginReverbXxx.hpp -- see that file's own header comment
#include "../plugin/dsp/EqModel.hpp"
#include "../plugin/dsp/PreDelayLine.hpp"
#include "../plugin/dsp/ReverbEnginePlate.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../plugin/dsp/StabilityGuard.hpp"
#include "../plugin/dsp/StateSpaceEngine.hpp"

START_NAMESPACE_DISTRHO

// The plate model's own plugin class -- structurally identical to
// ../plugin_dense/PluginReverbDense.hpp (same members, same method set,
// same ParameterIndex/StateIndex via ReverbParams.hpp), differing only
// in engine_'s own type (ssfilter::reverb::plate::ReverbEnginePlate).
class PluginReverbPlate : public Plugin
{
public:
    PluginReverbPlate();

protected:
    uint32_t getVersion() const override { return d_version(0, 1, 0); }

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    void applyManualDecayTargets();
    void rebuildEqEngine();

    mutable ssfilter::reverb::plate::ReverbEnginePlate engine_;
    ssfilter::reverb::RoomDims room_;

    ssfilter::reverb::PreDelayLine preDelayL_, preDelayR_;
    double preDelayMs_ = 0.0;
    static constexpr double kMaxPreDelayMs = 250.0;

    ssfilter::EqModel eqModel_;
    ssfilter::StateSpaceEngine eqEngine_;

    double sampleRate_ = 48000.0;
    bool bypass_ = false;

    double inputGainDb_ = 0.0;

    double decayTimeSec_ = 1.8;
    double damping_ = 0.5;

    bool tempoSyncEnabled_ = false;
    bool followHostTempo_ = false;
    int tempoNoteValueIndex_ = int(ssfilter::reverb::TempoNoteValue::Quarter);
    bool tempoCaptured_ = false;

    double dryWet_ = 0.35;
    double outputTrimDb_ = 0.0;

    bool lastTempoValid_ = false;
    double lastDetectedBpm_ = 0.0;

    double diffusionAmount_ = 0.7;
    double widthAmount_ = 1.0;
    double balanceAmount_ = 0.0;
    double modulationAmount_ = 0.3;

    ssfilter::SpectrumAnalyzer inputSpectrumAnalyzer_;
    ssfilter::SpectrumAnalyzer outputSpectrumAnalyzer_;
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX;
    void feedSpectrumAnalyzer(ssfilter::SpectrumAnalyzer& analyzer, const float* left, const float* right, uint32_t frames);

    DISTRHO_DECLARE_NON_COPYABLE(PluginReverbPlate)
};

END_NAMESPACE_DISTRHO
