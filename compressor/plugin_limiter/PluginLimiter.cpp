// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginLimiter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace sscomp;

PluginLimiter::PluginLimiter()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
}

void PluginLimiter::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginLimiter::initParameter(uint32_t index, Parameter& parameter)
{
    switch (index)
    {
    case kParamInputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Input Gain";
        parameter.symbol = "input_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamThreshold:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Threshold";
        parameter.symbol = "threshold";
        parameter.unit = "dB";
        parameter.ranges.def = -6.0f; parameter.ranges.min = -60.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamAttack:
        // Fast-only: Attack should stay at or below Lookahead so the gain
        // has finished ramping down when a delayed transient arrives.
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Attack";
        parameter.symbol = "attack";
        parameter.unit = "ms";
        parameter.ranges.def = 1.0f; parameter.ranges.min = 0.1f; parameter.ranges.max = 20.0f;
        break;
    case kParamRelease:
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Release";
        parameter.symbol = "release";
        parameter.unit = "ms";
        parameter.ranges.def = 100.0f; parameter.ranges.min = 5.0f; parameter.ranges.max = 2000.0f;
        break;
    case kParamLookahead:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Lookahead";
        parameter.symbol = "lookahead";
        parameter.unit = "ms";
        parameter.ranges.def = 5.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = float(kMaxLookaheadMs);
        break;
    case kParamKnee:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Knee";
        parameter.symbol = "knee";
        parameter.unit = "dB";
        parameter.ranges.def = 1.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamCeiling:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Ceiling";
        parameter.symbol = "ceiling";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -12.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamOutputGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Output Gain";
        parameter.symbol = "output_gain";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamBypass:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Bypass";
        parameter.symbol = "bypass";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;

    case kParamMeterInputRmsL:
    case kParamMeterInputPeakL:
    case kParamMeterInputRmsR:
    case kParamMeterInputPeakR:
    case kParamMeterOutputRmsL:
    case kParamMeterOutputPeakL:
    case kParamMeterOutputRmsR:
    case kParamMeterOutputPeakR:
    {
        static const char* names[] = {
            "Input RMS L", "Input Peak L", "Input RMS R", "Input Peak R",
            "Output RMS L", "Output Peak L", "Output RMS R", "Output Peak R"
        };
        static const char* symbols[] = {
            "meter_in_rms_l", "meter_in_peak_l", "meter_in_rms_r", "meter_in_peak_r",
            "meter_out_rms_l", "meter_out_peak_l", "meter_out_rms_r", "meter_out_peak_r"
        };
        const uint32_t i = index - kParamMeterInputRmsL;
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = names[i];
        parameter.symbol = symbols[i];
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    }
    case kParamMeterDetector:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Detector Level";
        parameter.symbol = "meter_detector";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamMeterGainReduction:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Gain Reduction";
        parameter.symbol = "meter_gr";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -48.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamBoost:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Boost";
        parameter.symbol = "boost";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamClipMode:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Soft Clip";
        parameter.symbol = "soft_clip";
        parameter.ranges.def = float(int(ClipMode::Soft)); parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamClipKnee:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Clip Knee";
        parameter.symbol = "clip_knee";
        parameter.unit = "dB";
        parameter.ranges.def = 1.0f; parameter.ranges.min = 0.1f; parameter.ranges.max = 6.0f;
        break;
    case kParamOversampling:
        // Not automatable: switching it changes the plugin's latency.
        parameter.hints = kParameterIsBoolean;
        parameter.name = "Oversampling";
        parameter.symbol = "oversampling";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamRatio:
    {
        parameter.hints = kParameterIsAutomatable | kParameterIsInteger;
        parameter.name = "Ratio";
        parameter.symbol = "ratio";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = float(kLimiterRatioCount - 1);
        ParameterEnumerationValue* const values = new ParameterEnumerationValue[kLimiterRatioCount];
        values[0].value = 0.0f; values[0].label = "20:1";
        values[1].value = 1.0f; values[1].label = "100:1";
        values[2].value = 2.0f; values[2].label = "inf:1";
        parameter.enumValues.count = kLimiterRatioCount;
        parameter.enumValues.restrictedMode = true;
        parameter.enumValues.values = values;
        break;
    }
    case kParamMeterClip:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Clip Reduction";
        parameter.symbol = "meter_clip";
        parameter.unit = "dB";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -48.0f; parameter.ranges.max = 0.0f;
        break;
    }
}

void PluginLimiter::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStatePresetDir:
        state.key = "presetDir";
        state.label = "Last Preset Directory";
        state.hints = 0; // UI convenience only, the DSP side never reads this
        state.defaultValue = "";
        break;
    case kStateAlgorithm:
        state.key = "algorithm";
        state.label = "Algorithm (Classic/True Peak)";
        state.hints = 0;
        state.defaultValue = "0"; // LimiterAlgorithm::Classic
        break;
    }
}

float PluginLimiter::getParameterValue(uint32_t index) const
{
    switch (index)
    {
    case kParamInputGain:  return float(inputGainDb_);
    case kParamThreshold:  return float(thresholdDb_);
    case kParamAttack:     return float(attackMs_);
    case kParamRelease:    return float(releaseMs_);
    case kParamLookahead:  return float(lookaheadMs_);
    case kParamKnee:       return float(kneeDb_);
    case kParamCeiling:    return float(ceilingDb_);
    case kParamOutputGain: return float(outputGainDb_);
    case kParamBypass:     return bypass_ ? 1.0f : 0.0f;

    case kParamMeterInputRmsL:  return meterInputRmsL_;
    case kParamMeterInputPeakL: return meterInputPeakL_;
    case kParamMeterInputRmsR:  return meterInputRmsR_;
    case kParamMeterInputPeakR: return meterInputPeakR_;
    case kParamMeterOutputRmsL:  return meterOutputRmsL_;
    case kParamMeterOutputPeakL: return meterOutputPeakL_;
    case kParamMeterOutputRmsR:  return meterOutputRmsR_;
    case kParamMeterOutputPeakR: return meterOutputPeakR_;
    case kParamMeterDetector:      return meterDetector_;
    case kParamMeterGainReduction: return meterGainReduction_;
    case kParamBoost:              return boost_ ? 1.0f : 0.0f;
    case kParamClipMode:           return float(int(clipMode_));
    case kParamClipKnee:           return float(clipKneeDb_);
    case kParamOversampling:       return oversampling_ ? 1.0f : 0.0f;
    case kParamMeterClip:          return meterClip_;
    case kParamRatio:              return float(ratioIndex_);
    default: return 0.0f;
    }
}

void PluginLimiter::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamInputGain:  inputGainDb_ = value; break;
    case kParamThreshold:  thresholdDb_ = value; break;
    case kParamAttack:     attackMs_ = value; break;
    case kParamRelease:    releaseMs_ = value; break;
    case kParamLookahead:  lookaheadMs_ = value; updateLookahead(); break;
    case kParamKnee:       kneeDb_ = value; break;
    case kParamCeiling:    ceilingDb_ = value; break;
    case kParamOutputGain: outputGainDb_ = value; break;
    case kParamBypass:     bypass_ = value >= 0.5f; break;
    case kParamBoost:      boost_ = value >= 0.5f; break;
    case kParamClipMode:   clipMode_ = value >= 0.5f ? ClipMode::Soft : ClipMode::Hard; break;
    case kParamClipKnee:   clipKneeDb_ = std::max(0.1, std::min(6.0, double(value))); break;
    case kParamRatio:      ratioIndex_ = std::max(0, std::min(kLimiterRatioCount - 1, int(std::lround(value)))); break;
    case kParamOversampling:
    {
        const bool next = value >= 0.5f;
        if (next != oversampling_)
        {
            // The detectors were not fed while off -- start them clean.
            tpDetL_.reset(); tpDetR_.reset();
            oversampling_ = next;
            updateLookahead();
        }
        break;
    }
    default: break;
    }
}

void PluginLimiter::updateLookahead()
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    const size_t bufLen = delayBufL_.size();
    if (bufLen == 0) return; // not prepared yet (sampleRateChanged() allocates the buffers) -- called again once it is

    // One slot of headroom, and room for the oversampling detector's delay.
    const int maxSamples = int(bufLen) - 1 - TruePeakDetector::kDelay;
    delaySamples_ = std::max(0, std::min(maxSamples, int(std::lround(lookaheadMs_ * 0.001 * sr))));
    setLatency(uint32_t(delaySamples_ + (oversampling_ ? TruePeakDetector::kDelay : 0)));
}

void PluginLimiter::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "algorithm") == 0)
    {
        const LimiterAlgorithm next = clampLimiterAlgorithm(std::atoi(value));
        if (next != algorithm_)
        {
            // The inactive gain computer wasn't fed while unselected; reset
            // both so the new one starts clean.
            voice_.reset();
            truePeak_.reset();
            algorithm_ = next;
        }
        return;
    }
    // "presetDir" is a pure UI preference -- nothing for the DSP side to do.
}

void PluginLimiter::activate()
{
    voice_.reset();
    truePeak_.reset();
    tpDetL_.reset(); tpDetR_.reset();
    loudnessInL_.reset(); loudnessInR_.reset(); loudnessOutL_.reset(); loudnessOutR_.reset();
    peakInL_.reset(); peakInR_.reset(); peakOutL_.reset(); peakOutR_.reset();
    std::fill(delayBufL_.begin(), delayBufL_.end(), 0.0f);
    std::fill(delayBufR_.begin(), delayBufR_.end(), 0.0f);
    delayPos_ = 0;
}

void PluginLimiter::run(const float** inputs, float** outputs, uint32_t frames)
{
    const float* inL = inputs[0];
    const float* inR = inputs[1];
    float* outL = outputs[0];
    float* outR = outputs[1];

    const size_t bufLen = delayBufL_.size();
    const int readDelay = delaySamples_ + (oversampling_ ? TruePeakDetector::kDelay : 0);

    if (bypass_)
    {
        // Through the same delay line, so bypassing doesn't move the audio
        // against the latency reported to the host.
        for (uint32_t i = 0; i < frames; ++i)
        {
            const double l = double(inL[i]), r = double(inR[i]);
            double dl = l, dr = r;
            if (bufLen > 0)
            {
                delayBufL_[delayPos_] = float(l);
                delayBufR_[delayPos_] = float(r);
                const size_t readPos = (delayPos_ + bufLen - size_t(readDelay)) % bufLen;
                dl = double(delayBufL_[readPos]);
                dr = double(delayBufR_[readPos]);
                delayPos_ = (delayPos_ + 1) % bufLen;
            }
            peakInL_.process(std::fabs(l)); peakInR_.process(std::fabs(r));
            peakOutL_.process(std::fabs(dl)); peakOutR_.process(std::fabs(dr));
            loudnessInL_.process(l); loudnessInR_.process(r);
            loudnessOutL_.process(dl); loudnessOutR_.process(dr);
            outL[i] = float(dl);
            outR[i] = float(dr);
        }
        meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
        meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
        meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
        meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());
        meterGainReduction_ = 0.0f;
        meterClip_ = 0.0f;
        return;
    }

    // Where to limit and how much gain follows -- see computeLimiterStaging().
    const bool soft = clipMode_ == ClipMode::Soft;
    const LimiterStaging staging = computeLimiterStaging(thresholdDb_, ceilingDb_, outputGainDb_, boost_, soft, clipKneeDb_);
    effThresholdLin_ = dbToLinear(staging.effectiveThresholdDb);
    postGainLin_ = dbToLinear(staging.postGainDb);
    ceilingLin_ = dbToLinear(ceilingDb_);
    clipKneeLin_ = dbToLinear(ceilingDb_ - clipKneeDb_);

    voice_.setParams(attackMs_, releaseMs_, staging.effectiveThresholdDb, limiterRatioValue(ratioIndex_), 1.0, kneeDb_);
    truePeak_.setReleaseCoeff(truePeakReleaseCoeffFromMs(releaseMs_, getSampleRate()));

    if (oversampling_)
    {
        if (soft) processBlock<true, true>(inL, inR, outL, outR, frames);
        else      processBlock<true, false>(inL, inR, outL, outR, frames);
    }
    else
    {
        if (soft) processBlock<false, true>(inL, inR, outL, outR, frames);
        else      processBlock<false, false>(inL, inR, outL, outR, frames);
    }
}

template <bool kOversample, bool kSoftClip>
void PluginLimiter::processBlock(const float* inL, const float* inR, float* outL, float* outR, uint32_t frames)
{
    const bool truePeak = algorithm_ == LimiterAlgorithm::TruePeak;
    const double inGainLin = dbToLinear(inputGainDb_);
    const double thresholdLin = effThresholdLin_;
    const double postGainLin = postGainLin_;
    const double ceilingLin = ceilingLin_;
    const double clipKneeLin = clipKneeLin_;

    const size_t bufLen = delayBufL_.size();
    const int readDelay = delaySamples_ + (kOversample ? TruePeakDetector::kDelay : 0);
    double lastDetector = 0.0, lastGrLin = 1.0;
    double maxPreClip = 0.0; // loudest sample going into the clip stage this block, for meterClip_

    for (uint32_t i = 0; i < frames; ++i)
    {
        const double xgL = double(inL[i]) * inGainLin;
        const double xgR = double(inR[i]) * inGainLin;
        peakInL_.process(std::fabs(double(inL[i])));
        peakInR_.process(std::fabs(double(inR[i])));

        // Always Linked. Detection reads the incoming sample (or, with
        // Oversampling, the 4x peak around the sample kDelay older -- the
        // audio is read that much later too, so they stay aligned); the
        // audio leaves the delay line readDelay samples later.
        double detector;
        if (kOversample)
            detector = std::max(tpDetL_.process(xgL), tpDetR_.process(xgR));
        else
            detector = std::max(std::fabs(xgL), std::fabs(xgR));

        double grLin;
        if (truePeak)
        {
            const double requiredGain = std::min(1.0, thresholdLin / std::max(detector, 1e-9));
            grLin = truePeak_.process(requiredGain, delaySamples_);
        }
        else
        {
            grLin = voice_.process(detector);
        }
        lastDetector = detector;
        lastGrLin = grLin;

        double delayedL = xgL, delayedR = xgR;
        if (bufLen > 0)
        {
            delayBufL_[delayPos_] = float(xgL);
            delayBufR_[delayPos_] = float(xgR);
            const size_t readPos = (delayPos_ + bufLen - size_t(readDelay)) % bufLen;
            delayedL = double(delayBufL_[readPos]);
            delayedR = double(delayBufR_[readPos]);
            delayPos_ = (delayPos_ + 1) % bufLen;
        }

        loudnessInL_.process(xgL);  loudnessInR_.process(xgR);

        double yL = delayedL * grLin * postGainLin;
        double yR = delayedR * grLin * postGainLin;
        maxPreClip = std::max(maxPreClip, std::max(std::fabs(yL), std::fabs(yR)));

        // The clip stage: with the capped post gain it only acts on
        // overshoot (Classic's attack, or Ceiling below the limited level).
        if (kSoftClip)
        {
            yL = softClip(yL, clipKneeLin, ceilingLin);
            yR = softClip(yR, clipKneeLin, ceilingLin);
        }
        else
        {
            yL = hardClip(yL, ceilingLin);
            yR = hardClip(yR, ceilingLin);
        }

        loudnessOutL_.process(yL); loudnessOutR_.process(yR);
        peakOutL_.process(std::fabs(yL));
        peakOutR_.process(std::fabs(yR));

        outL[i] = float(yL);
        outR[i] = float(yR);
    }

    meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
    meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
    meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
    meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());

    // Both clip curves take off proportionally more the louder the input,
    // so the loudest sample shows the block's largest reduction.
    if (maxPreClip > 1e-9)
    {
        const double clipped = kSoftClip ? softClip(maxPreClip, clipKneeLin, ceilingLin) : hardClip(maxPreClip, ceilingLin);
        meterClip_ = float(std::min(0.0, linearToDb(clipped / maxPreClip)));
    }
    else
    {
        meterClip_ = 0.0f;
    }

    if (truePeak)
    {
        // True Peak has no smoothed detector; report the last sample's level
        // and the gain applied to it.
        meterDetector_ = float(linearToDb(lastDetector));
        meterGainReduction_ = float(linearToDb(lastGrLin));
    }
    else
    {
        meterDetector_ = float(voice_.detectorLevelDb());
        meterGainReduction_ = float(voice_.gainReductionDb());
    }
}

void PluginLimiter::sampleRateChanged(double newSampleRate)
{
    voice_.prepare(newSampleRate);
    loudnessInL_.prepare(newSampleRate); loudnessInR_.prepare(newSampleRate);
    loudnessOutL_.prepare(newSampleRate); loudnessOutR_.prepare(newSampleRate);
    peakInL_.prepare(newSampleRate); peakInR_.prepare(newSampleRate);
    peakOutL_.prepare(newSampleRate); peakOutR_.prepare(newSampleRate);

    // Maximum lookahead + the oversampling detector's delay + headroom (so
    // the read position never lands on the slot written this same step).
    const size_t lookLen = size_t(newSampleRate * kMaxLookaheadMs * 0.001) + 2;
    const size_t newLen = lookLen + size_t(TruePeakDetector::kDelay);
    delayBufL_.assign(newLen, 0.0f);
    delayBufR_.assign(newLen, 0.0f);
    delayPos_ = 0;
    truePeak_.prepare(lookLen);
    tpDetL_.reset(); tpDetR_.reset();
    updateLookahead();
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginLimiter(); }
END_NAMESPACE_DISTRHO
