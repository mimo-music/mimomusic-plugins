// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginLimiter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>

USE_NAMESPACE_DISTRHO
using namespace sscomp;

namespace {
// Fixed internally -- not a Parameter, see PluginLimiter.hpp's file
// comment. A limiter is inherently near-brick-wall rather than a tunable-
// ratio compressor, and no dedicated limiter plugin surveyed (LSP's
// included) exposes Ratio either -- Threshold/Attack/Release/Lookahead/
// Knee/Ceiling already fully shape the curve; this high a ratio just makes
// "above threshold" read as "brick wall" through the same soft-knee curve
// every other plugin in this project shares
// (sscomp::computeGainReductionDb()).
constexpr double kFixedRatio = 20.0;
}

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
        parameter.hints = kParameterIsAutomatable | kParameterIsLogarithmic;
        parameter.name = "Attack";
        parameter.symbol = "attack";
        parameter.unit = "ms";
        // Deliberately fast-only, unlike the compressors' own 0.1-300ms
        // Attack range -- a limiter's attack works best at or below its
        // Lookahead time (see kParamLookahead's own comment) so the gain
        // has actually finished ramping down by the time a delayed
        // transient reaches the output; a slow attack here would mostly
        // just be caught by the Ceiling clamp instead of the gain computer.
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
        // The piece a plain "fast VCA + hard clip" limiter is missing
        // compared to a dedicated one (LSP's Limiter Stereo/Mono among
        // them): a small delay buffer lets the gain computer react to a
        // transient *before* it reaches the output rather than only after,
        // so the Attack/Release envelope has time to actually get there
        // instead of leaning on the Ceiling clamp to catch every peak.
        // Adds real output latency -- see PluginLimiter.hpp's
        // updateLookahead()/DistrhoPluginInfo.h's DISTRHO_PLUGIN_WANT_LATENCY.
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
        // A limiter only ever cuts (no Up Ratio here) -- one-directional,
        // unlike the compressor plugins' bipolar GR meter parameter.
        parameter.ranges.def = 0.0f; parameter.ranges.min = -48.0f; parameter.ranges.max = 0.0f;
        break;
    case kParamBoost:
        parameter.hints = kParameterIsAutomatable | kParameterIsBoolean;
        parameter.name = "Boost";
        parameter.symbol = "boost";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 1.0f;
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
        state.hints = 0; // DSP-relevant (see setState()), but a structural two-way choice, not a continuous host-automatable control -- same convention as the Stereo compressor's own "stereoMode" state
        state.defaultValue = "0"; // LimiterAlgorithm::Classic -- unchanged existing behavior stays the default
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
    default: break;
    }
}

void PluginLimiter::updateLookahead()
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    const size_t bufLen = delayBufL_.size();
    if (bufLen == 0) return; // not prepared yet (sampleRateChanged() allocates the buffers) -- called again once it is

    const int maxSamples = int(bufLen) - 1; // one slot of headroom, see sampleRateChanged()'s own comment
    delaySamples_ = std::max(0, std::min(maxSamples, int(std::lround(lookaheadMs_ * 0.001 * sr))));

    // Hosts need to know about added latency to keep this plugin's output
    // time-aligned with everything else on the timeline (e.g. parallel/
    // sidechain routing, or simply not shifting the whole mix). See
    // DistrhoPluginInfo.h's DISTRHO_PLUGIN_WANT_LATENCY.
    setLatency(uint32_t(delaySamples_));
}

void PluginLimiter::setState(const char* key, const char* value)
{
    if (std::strcmp(key, "algorithm") == 0)
    {
        const LimiterAlgorithm next = clampLimiterAlgorithm(std::atoi(value));
        if (next != algorithm_)
        {
            // Reset both gain computers' own internal state on an actual
            // switch (not merely the same value re-arriving, e.g. from a
            // host resending state on project load): whichever algorithm
            // was inactive hasn't been fed a single sample the whole time
            // it wasn't selected, so its state (the envelope for Classic,
            // the sliding-window deque + applied gain for True Peak) is
            // stale by however long that was -- reset gives the newly
            // selected one a clean, correct starting point rather than
            // resuming from an arbitrary old value. This is a plain
            // instant switch, not a duck-to-silence crossfade: unlike a
            // band-count change that can happen mid-drag on every GUI
            // frame, choosing an Algorithm is a rare, deliberate, explicit
            // action (a button click), so the brief transient a reset
            // envelope/deque can cause for a moment is an acceptable,
            // honest trade-off against a materially more complex fade
            // state machine for something this infrequent -- the same
            // "read from the live target and switch instantly" precedent
            // statespace's own DynamicEqEngine.hpp
            // already uses for a band's threshold/ratio/range.
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

    if (bypass_)
    {
        if (outL != inL) std::copy(inL, inL + frames, outL);
        if (outR != inR) std::copy(inR, inR + frames, outR);
        for (uint32_t i = 0; i < frames; ++i)
        {
            const double l = double(inL[i]), r = double(inR[i]);
            peakInL_.process(std::fabs(l)); peakInR_.process(std::fabs(r));
            peakOutL_.process(std::fabs(l)); peakOutR_.process(std::fabs(r));
            loudnessInL_.process(l); loudnessInR_.process(r);
            loudnessOutL_.process(l); loudnessOutR_.process(r);
        }
        meterInputRmsL_ = float(loudnessInL_.rmsDb());  meterInputPeakL_ = float(peakInL_.peakDb());
        meterInputRmsR_ = float(loudnessInR_.rmsDb());  meterInputPeakR_ = float(peakInR_.peakDb());
        meterOutputRmsL_ = float(loudnessOutL_.rmsDb()); meterOutputPeakL_ = float(peakOutL_.peakDb());
        meterOutputRmsR_ = float(loudnessOutR_.rmsDb()); meterOutputPeakR_ = float(peakOutR_.peakDb());
        meterGainReduction_ = 0.0f;
        return;
    }

    const bool truePeak = algorithm_ == LimiterAlgorithm::TruePeak;

    // Classic's own gain computer -- unused (but still harmlessly kept
    // current) when truePeak is selected. Deliberately still called every
    // block regardless of `truePeak`, exactly as before this file ever
    // had a second algorithm: this branch is byte-for-byte the same code
    // path the original single-algorithm version of this file ran, so
    // Classic mode's own output cannot have changed by so much as a bit.
    voice_.setParams(attackMs_, releaseMs_, thresholdDb_, kFixedRatio, 1.0, kneeDb_);
    // True Peak's own gain computer: Attack and Knee play no role in it
    // (see TruePeakLimiterDsp.hpp's file comment -- the reaction is "as
    // fast as the lookahead window allows", by construction, and the
    // guarantee that file proves only holds for a hard threshold, not a
    // soft-knee blend), only Release (how fast gain climbs back toward
    // unity) and the same Threshold/Ceiling/Lookahead every mode shares.
    truePeak_.setReleaseCoeff(truePeakReleaseCoeffFromMs(releaseMs_, getSampleRate()));
    const double thresholdLin = dbToLinear(thresholdDb_);

    const double inGainLin = dbToLinear(inputGainDb_);
    // Boost: see kParamBoost's own comment (PluginLimiter.hpp) for the
    // research behind this -- a STATIC offset tied to Threshold (matching
    // LSP/FabFilter/Slate's own "Boost"/"Unity Gain"/"Constant Gain"
    // controls), added on top of the ordinary Output Gain trim, not in
    // place of it.
    const double outGainLin = dbToLinear(outputGainDb_ + (boost_ ? -thresholdDb_ : 0.0));
    // The Ceiling clamp: a hard clamp on the true output sample, applied
    // after everything else (input gain, gain reduction, output gain).
    // Even with Lookahead giving the gain computer a head start (see
    // below), an extreme or fast-enough transient can still momentarily
    // slip past the intended reduction -- this clamp is the safety net
    // that guarantees the ceiling is never crossed regardless, at the cost
    // of a possible instant of hard clipping on such a transient rather
    // than a perfect zero-overshoot catch. True Peak's own gain computer
    // already guarantees the *threshold* is never crossed by construction
    // (see TruePeakLimiterDsp.hpp's proof) -- this clamp still runs
    // unconditionally in both modes regardless, both as a genuine safety
    // net if Ceiling is set below Threshold, and to keep this one code
    // path shared rather than forked per algorithm for no benefit.
    const double ceilingLin = dbToLinear(ceilingDb_);

    const size_t bufLen = delayBufL_.size();
    double lastDetector = 0.0, lastGrLin = 1.0; // for the meters, after the loop -- see their own assignment below

    for (uint32_t i = 0; i < frames; ++i)
    {
        const double xgL = double(inL[i]) * inGainLin;
        const double xgR = double(inR[i]) * inGainLin;
        peakInL_.process(std::fabs(double(inL[i])));
        peakInR_.process(std::fabs(double(inR[i])));

        // Always Linked -- see PluginLimiter.hpp's file comment for why a
        // limiter protecting a stereo bus shouldn't move the stereo image.
        // Detection reads the *incoming*, not-yet-delayed sample -- the
        // actual look-ahead trick: the gain computer reacts to a transient
        // right now, while the audio it will apply that reaction to is
        // whatever arrived delaySamples_ ago, read back out of the delay
        // line below. By the time that older sample reaches the output,
        // the envelope has already had delaySamples_ worth of time to
        // ramp down (see kParamAttack's own comment on why Attack should
        // stay at or below Lookahead for this to actually catch it).
        const double detector = std::max(std::fabs(xgL), std::fabs(xgR));
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
            const size_t readPos = (delayPos_ + bufLen - size_t(delaySamples_)) % bufLen;
            delayedL = double(delayBufL_[readPos]);
            delayedR = double(delayBufR_[readPos]);
            delayPos_ = (delayPos_ + 1) % bufLen;
        }

        double yL = delayedL * grLin;
        double yR = delayedR * grLin;

        loudnessInL_.process(xgL);  loudnessInR_.process(xgR); // the arriving signal, not the (internally) delayed one -- "what's coming in right now"

        yL *= outGainLin; yR *= outGainLin;
        yL = std::max(-ceilingLin, std::min(ceilingLin, yL));
        yR = std::max(-ceilingLin, std::min(ceilingLin, yR));

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
    if (truePeak)
    {
        // True Peak has no smoothed envelope of its own to report (see
        // TruePeakLimiterDsp.hpp's file comment -- it reacts to the raw
        // per-sample level directly, there's no equivalent single
        // "detector level" value) -- report the last processed block
        // sample's own raw level instead, and the gain actually applied
        // to it, both meaningful, comparable-in-spirit readings to
        // Classic's own two meters.
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

    // Sized for the maximum lookaheadMs_ the parameter range allows, plus
    // one slot of headroom (so delaySamples_ can reach that maximum
    // without readPos_ ever landing exactly on the sample about to be
    // overwritten this same iteration) -- see updateLookahead()'s own
    // comment. A sample-rate change invalidates any old contents anyway
    // (activate() is what actually gets called before real playback
    // resumes and zeros them), so a plain resize is fine here.
    const size_t newLen = size_t(newSampleRate * kMaxLookaheadMs * 0.001) + 2;
    delayBufL_.assign(newLen, 0.0f);
    delayBufR_.assign(newLen, 0.0f);
    delayPos_ = 0;
    truePeak_.prepare(newLen); // same max-window sizing as the delay lines themselves, see TruePeakLimiterDsp.hpp's own comment
    updateLookahead();
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginLimiter(); }
END_NAMESPACE_DISTRHO
