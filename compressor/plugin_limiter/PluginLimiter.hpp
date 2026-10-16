// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"
#include "../plugin/dsp/LimiterDsp.hpp"
#include "../plugin/dsp/TruePeakLimiterDsp.hpp"

#include <vector>

START_NAMESPACE_DISTRHO

// A State, not a Parameter -- a structural choice between two gain
// computers, not a continuous control. See TruePeakLimiterDsp.hpp for why
// there are two (an envelope follower can't guarantee a hard peak limit).
enum class LimiterAlgorithm { Classic = 0, TruePeak = 1 };
inline LimiterAlgorithm clampLimiterAlgorithm(int raw)
{
    return raw == 1 ? LimiterAlgorithm::TruePeak : LimiterAlgorithm::Classic;
}

// The output stage after the gain computer and the post gain -- see
// LimiterDsp.hpp's hardClip()/softClip().
enum class ClipMode { Hard = 0, Soft = 1 };

// The default Clip mode for an Algorithm, applied by the UI when the
// Algorithm is switched (the user can change it afterwards): True Peak
// never overshoots, so a Hard ceiling is exact and adds nothing; Classic's
// finite attack lets transients overshoot, which a Soft clip rounds off.
inline ClipMode defaultClipModeFor(LimiterAlgorithm a)
{
    return a == LimiterAlgorithm::TruePeak ? ClipMode::Hard : ClipMode::Soft;
}

// Indices are never reordered or reused -- new parameters are appended --
// so saved sessions and automation keep mapping to the right control.
enum ParameterIndex
{
    kParamInputGain = 0,
    kParamThreshold,
    kParamAttack,
    kParamRelease,
    kParamLookahead,
    kParamKnee,
    kParamCeiling,
    kParamOutputGain,
    kParamBypass,

    kParamMeterInputRmsL,
    kParamMeterInputPeakL,
    kParamMeterInputRmsR,
    kParamMeterInputPeakR,
    kParamMeterOutputRmsL,
    kParamMeterOutputPeakL,
    kParamMeterOutputRmsR,
    kParamMeterOutputPeakR,
    kParamMeterDetector,
    kParamMeterGainReduction,

    // Adds -Threshold after the limiter (LSP's Limiter: "applies
    // corresponding to the Threshold gain to the output signal"), on top
    // of Output Gain. The sum is capped so the limited peaks don't go over
    // the Ceiling -- see computeLimiterStaging().
    kParamBoost,

    kParamClipMode,     // ClipMode: 0 = Hard, 1 = Soft
    kParamClipKnee,     // dB below the Ceiling where Soft clipping starts
    kParamOversampling, // 0 = off, 1 = 4x oversampled peak detection (adds TruePeakDetector::kDelay samples of latency)
    kParamMeterClip,    // output: how much the clip stage took off the loudest sample of the last block (dB, <= 0)
    kParamRatio,        // Classic's ratio: 0 = 20:1, 1 = 100:1, 2 = infinity:1 (limiterRatioValue()); True Peak ignores it

    kParamCount
};

enum StateIndex
{
    kStatePresetDir = 0, // last folder used in the Save As.../Load... dialogs (UI convenience only)
    kStateAlgorithm = 1, // LimiterAlgorithm (0=Classic/1=TruePeak) -- DSP-relevant, see setState()
    kStateCount
};

class PluginLimiter : public Plugin
{
public:
    PluginLimiter();

protected:
    uint32_t getVersion() const override { return d_version(4, 0, 0); } // 4.0: capped Boost/Output, Clip mode, Ratio, Oversampling

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
    void updateLookahead(); // recomputes delaySamples_/latency from lookaheadMs_, the sample rate and oversampling_

    // The per-sample loop, compiled once per combination so a disabled
    // feature costs nothing (Oversampling off: no detector work at all).
    template <bool kOversample, bool kSoftClip>
    void processBlock(const float* inL, const float* inR, float* outL, float* outR, uint32_t frames);

    sscomp::CompressorVoice voice_;          // Classic's gain computer
    sscomp::TruePeakGainComputer truePeak_;  // True Peak's gain computer
    sscomp::TruePeakDetector tpDetL_, tpDetR_; // Oversampling's peak detectors
    sscomp::LoudnessMeter loudnessInL_, loudnessInR_, loudnessOutL_, loudnessOutR_;
    sscomp::PeakMeter peakInL_, peakInR_, peakOutL_, peakOutR_;

    LimiterAlgorithm algorithm_ = LimiterAlgorithm::Classic;

    double inputGainDb_ = 0.0;
    double thresholdDb_ = -6.0;
    double attackMs_ = 1.0;
    double releaseMs_ = 100.0;
    double lookaheadMs_ = 5.0;
    double kneeDb_ = 1.0;
    double ceilingDb_ = 0.0;
    double outputGainDb_ = 0.0;
    bool bypass_ = false;
    bool boost_ = false;
    ClipMode clipMode_ = ClipMode::Soft; // defaultClipModeFor(Classic), the default Algorithm
    double clipKneeDb_ = 1.0;
    bool oversampling_ = false;
    int ratioIndex_ = 0; // 20:1

    // Per-block values for processBlock(), set at the top of run().
    double effThresholdLin_ = 0.5, postGainLin_ = 1.0, ceilingLin_ = 1.0, clipKneeLin_ = 1.0;

    // Look-ahead delay line, sized once per sample rate for the maximum
    // lookahead plus the oversampling detector's delay. delaySamples_ is
    // the lookahead window in samples; the audio is read
    // delaySamples_ (+ TruePeakDetector::kDelay with Oversampling) back.
    static constexpr double kMaxLookaheadMs = 20.0;
    std::vector<float> delayBufL_, delayBufR_;
    size_t delayPos_ = 0;
    int delaySamples_ = 0;

    float meterInputRmsL_ = -120.0f, meterInputPeakL_ = -120.0f;
    float meterInputRmsR_ = -120.0f, meterInputPeakR_ = -120.0f;
    float meterOutputRmsL_ = -120.0f, meterOutputPeakL_ = -120.0f;
    float meterOutputRmsR_ = -120.0f, meterOutputPeakR_ = -120.0f;
    float meterDetector_ = -120.0f;
    float meterGainReduction_ = 0.0f;
    float meterClip_ = 0.0f;

    DISTRHO_DECLARE_NON_COPYABLE(PluginLimiter)
};

END_NAMESPACE_DISTRHO
