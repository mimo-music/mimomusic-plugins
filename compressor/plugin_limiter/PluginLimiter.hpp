// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"
#include "../plugin/dsp/TruePeakLimiterDsp.hpp"

#include <vector>

START_NAMESPACE_DISTRHO

// Which gain-computer run() actually uses -- a State (like the Stereo
// compressor's own kStateStereoMode), not a Parameter: a structural
// choice between two different DSP engines, not a continuous control, the
// same convention this project already uses for StereoMode. See
// plugin/dsp/TruePeakLimiterDsp.hpp's file comment for the full story of
// why a second algorithm exists at all -- an empirical A/B against LSP's
// own installed Limiter found ours (an envelope-follower-driven curve)
// can't guarantee a hard peak ceiling on fast transients no matter how
// steep its ratio, while a genuine lookahead peak-hold computer can, by
// construction. Classic is unchanged and stays the default -- this is a
// second, additive path, not a replacement.
enum class LimiterAlgorithm { Classic = 0, TruePeak = 1 };
inline LimiterAlgorithm clampLimiterAlgorithm(int raw)
{
    return raw == 1 ? LimiterAlgorithm::TruePeak : LimiterAlgorithm::Classic;
}

// See DistrhoPluginInfo.h's own comment for the full history (a first
// "reset from scratch" pass, then this revision after comparing against
// dedicated limiter plugins like LSP's). The control set now: Threshold,
// Attack, Release, Lookahead, Knee, Ceiling, Input/Output Gain, Bypass.
// Ratio is still fixed internally (see PluginLimiter.cpp's kFixedRatio) --
// no dedicated limiter plugin surveyed exposes it either, since a limiter
// is inherently near-brick-wall rather than a tunable-ratio compressor --
// and stereo detection is always Linked, for the same reason as before
// (an independent per-channel or Mid/Side limiter on a stereo bus would
// move the image).
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

    // Boost, as found on some limiters: the reduced gain is added back at
    // the output. Surveyed how dedicated limiter plugins actually
    // implement this before building it, and the consistent answer (LSP's Limiter
    // Stereo/Mono, whose own manual is quoted verbatim below; FabFilter
    // Pro-L 2's "Unity Gain"; Slate FG-X's "Constant Gain") is a STATIC
    // offset computed from a knob value, not a live per-sample tracker of
    // the gain-reduction meter: "Boost - applies corresponding to the
    // Threshold gain to the output signal" (LSP Limiter Stereo manual).
    // Implemented the same way here: toggling this ON adds `-thresholdDb_`
    // to the effective output gain (run()'s own outGainLin), ADDITIVELY on
    // top of kParamOutputGain rather than replacing it, so Output Gain
    // still works as a fine-trim either way. Appended after the meter
    // bank, not inserted earlier -- this enum's own index-stability
    // convention (every sibling plugin in this project follows it).
    kParamBoost,

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
    uint32_t getVersion() const override { return d_version(3, 1, 0); } // 3.0: Attack/Knee exposed, real Lookahead added. 3.1: Boost button added

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
    void updateLookahead(); // recomputes delaySamples_ from lookaheadMs_/sample rate, resizes the delay buffers, calls setLatency()

    sscomp::CompressorVoice voice_; // one shared (Linked) detector, see file comment -- the Classic algorithm's own gain computer
    sscomp::TruePeakGainComputer truePeak_; // the alternate algorithm's own gain computer, see PluginLimiter.hpp's own top comment
    sscomp::LoudnessMeter loudnessInL_, loudnessInR_, loudnessOutL_, loudnessOutR_;
    sscomp::PeakMeter peakInL_, peakInR_, peakOutL_, peakOutR_;

    LimiterAlgorithm algorithm_ = LimiterAlgorithm::Classic;

    double inputGainDb_ = 0.0;
    double thresholdDb_ = -6.0;   // limiters conventionally sit close to 0dBFS, unlike a compressor's typical -18dB
    double attackMs_ = 1.0;
    double releaseMs_ = 100.0;
    double lookaheadMs_ = 5.0;    // see kParamLookahead's own comment in the ParameterIndex enum above
    double kneeDb_ = 1.0;
    double ceilingDb_ = 0.0;      // hard output ceiling, see run()'s own comment
    double outputGainDb_ = 0.0;
    bool bypass_ = false;
    bool boost_ = false; // see kParamBoost's own comment

    // Look-ahead delay line: sized once for the maximum lookaheadMs_ the
    // parameter range allows, at whatever the current sample rate is (see
    // updateLookahead()), so a live lookahead change only needs a cheap
    // reposition, not a reallocation. delaySamples_ is how many samples
    // *of that buffer* the currently-configured lookaheadMs_ actually asks
    // for -- always <= the buffer's own capacity.
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

    DISTRHO_DECLARE_NON_COPYABLE(PluginLimiter)
};

END_NAMESPACE_DISTRHO
