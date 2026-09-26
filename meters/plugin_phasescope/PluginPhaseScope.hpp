// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/PhaseScopeDsp.hpp"

START_NAMESPACE_DISTRHO

// Every Parameter here is either an *output* meter reading or a control
// over the goniometer's own *display* scale (Display Gain, the Heatmap's
// own Decay) -- there is nothing to automate that shapes the audio,
// because this plugin never touches the audio (see
// PluginPhaseScope.cpp's own run() comment). kParamScopePacketBase/
// kParamHeatmapPacketBase are banks of packed points/cells -- see
// ssmeter::GoniometerBuffer/HeatmapGrid's own file comments for the
// packing schemes and why bulk per-frame data has to travel as ordinary
// Parameters at all. The goniometer's own "Decay" (how much of its
// fixed, constantly-refilling buffer is actually shown) is a State, not
// a Parameter -- see kStateDecayMs's own comment for why -- but the
// Heatmap's own Decay *is* a Parameter, because it genuinely changes
// what HeatmapGrid computes (see kParamHeatmapDecaySec's own comment)
// the same way the goniometer's own Decay used to before its own
// redesign. "Auto Gain" is *not* a Parameter/trigger at all any more --
// see kParamMeterPlotPeakDb's own comment for why.
enum ParameterIndex
{
    kParamMeterInputRmsL = 0,
    kParamMeterInputPeakL,
    kParamMeterInputRmsR,
    kParamMeterInputPeakR,
    kParamMeterCorrelation,
    kParamMeterCorrelationMinHold,
    kParamMeterWidthPercent,
    kParamDisplayGain, // dB trim on the goniometer's own plot scale -- never touches the audio
    // output: ssmeter::SmoothedLevelMeter's own reading of the *raw,
    // unscaled* rotated Mid/Side magnitude -- a one-pole average with a
    // ~1s (or longer, see kAutoGainTimeConstantSeconds) time constant,
    // not an instant-attack peak. This replaces an earlier
    // multi-second "measurement" state machine (ssmeter::GainMeasurement)
    // entirely: that design required run() to keep being called for a
    // full ~1s window to complete a measurement, which could never finish
    // (leaving the UI's own Auto Gain button stuck showing "Measuring...")
    // if the host ever stopped calling run() during that window -- e.g.
    // because the transport was paused, a state most hosts treat as
    // "nothing to process". It was then a plain
    // ssmeter::PeakMeter (instant attack, ~20dB/s decay) for a while,
    // which had no such stuck-measurement window, but made continuous
    // Auto Gain tracking (see kStateAutoGainActive) visibly twitchy --
    // every individual transient snapped Gain up instantly, then let it
    // slide back down as the peak decayed. SmoothedLevelMeter's own
    // one-pole average has neither problem: always already available
    // (nothing to get stuck waiting for), and changes gradually enough
    // that continuous re-application reads as smooth tracking rather than
    // a nervous snap-and-slide. The UI's own "Auto Gain" button (see
    // UIPhaseScope.cpp) computes and applies a fresh Gain value from
    // whatever this currently reads, either once (on click) or
    // continuously (while toggled on), with no DSP-side trigger/state
    // round trip needed at all.
    kParamMeterPlotPeakDb,
    kParamHeatmapDecaySec, // 1..30: how long, in seconds, a Heatmap cell takes to fade after its last hit -- see ssmeter::HeatmapGrid's own comment
    kParamScopeWriteIndex, // current ring-buffer write slot (0..kScopeBufferSize-1) -- the UI derives each point's fade-age from this
    kParamScopePacketBase, // + [0 .. kScopeBufferSize) -- one packed (x,y) goniometer point per Parameter
    kParamHeatmapPacketBase = kParamScopePacketBase + ssmeter::GoniometerBuffer::kScopeBufferSize, // + [0 .. kNumPackets) -- 4 packed heatmap cells per Parameter
    kParamCount = kParamHeatmapPacketBase + ssmeter::HeatmapGrid::kNumPackets
};

// Every State here is a pure rendering preference with zero effect on the
// DSP side at all -- see each one's own comment -- the same "a lasting
// UI preference, not a continuous DSP-relevant control" convention
// ../statespace's own kStateSpectrumAutoscale uses.
enum StateIndex
{
    // GoniometerWidget draws connected line segments between consecutive
    // scope points instead of a dot cloud.
    kStateDrawLines = 0,
    // How much of GoniometerBuffer's fixed, constantly-refilling ring
    // (see that class's own file comment) is actually drawn -- purely a
    // UI-side windowing decision (GoniometerBuffer::visibleCountForDecay())
    // now, unlike an earlier version where "Decay" changed the buffer's
    // own write rate and was a real Parameter.
    kStateDecayMs = 1,
    // 10..100: what fraction of the currently-visible buffered points
    // GoniometerWidget actually issues a draw call for -- see that
    // class's own setResolutionPercent() comment. A pure rendering-cost
    // knob for machines with less GPU/CPU headroom, with no DSP-side
    // meaning at all: the buffer itself is
    // always written and transported at full density regardless of this.
    kStateResolutionPercent = 2,
    // Draw Lines mode only: a Catmull-Rom spline through the drawn points
    // instead of straight segments between them -- see
    // GoniometerWidget::setSmoothLines()'s own comment. Directly asked
    // for alongside Resolution ("do a non linear interpolation in the
    // lines option") specifically so thinning Resolution down doesn't
    // have to mean a visibly rougher-looking trail too.
    kStateSmoothLines = 3,
    // Whether GoniometerWidget actually draws the Heatmap layer --
    // ssmeter::HeatmapGrid itself always accumulates in the background
    // regardless (see that class's own comment), so toggling this on
    // shows an already-populated multi-second history immediately rather
    // than starting from blank, the same "always running, display choice
    // is separate" idea kStateDrawLines/kStateSmoothLines already use.
    kStateHeatmapEnabled = 4,
    // Whether "Auto Gain" is currently toggled on -- see UIPhaseScope's
    // own updateAutoGain()/parameterChanged() comments. While on, Gain
    // (kParamDisplayGain) is continuously recomputed from the live
    // kParamMeterPlotPeakDb reading rather than left alone; toggling off
    // simply stops that continuous recomputation, leaving Gain at
    // whatever it last settled on. A lasting UI choice worth a host remembering across a
    // session, the same reasoning kStateHeatmapEnabled already uses.
    kStateAutoGainActive = 5,
    kStateCount
};

class PluginPhaseScope : public Plugin
{
public:
    PluginPhaseScope();

protected:
    uint32_t getVersion() const override { return d_version(1, 7, 1); } // 1.7.1: Goniometer Decay default raised to 600ms; Auto Gain's own level reading is a smoothed ~1s average now, not an instant peak

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
    ssmeter::LoudnessMeter loudnessL_, loudnessR_;
    ssmeter::PeakMeter peakL_, peakR_;
    ssmeter::StereoImageMeter stereoImage_;
    ssmeter::SmoothedLevelMeter plotLevelMeter_; // a slow (~1s+) average of the raw, unscaled rotated Mid/Side magnitude -- see kParamMeterPlotPeakDb's own comment
    ssmeter::GoniometerBuffer goniometer_;
    ssmeter::HeatmapGrid heatmap_;

    float meterInputRmsL_ = -120.0f, meterInputPeakL_ = -120.0f;
    float meterInputRmsR_ = -120.0f, meterInputPeakR_ = -120.0f;
    float meterCorrelation_ = 0.0f;
    float meterCorrelationMinHold_ = 1.0f;
    float meterWidthPercent_ = 0.0f;
    float meterPlotPeakDb_ = -120.0f;

    double displayGainDb_ = 0.0;
    double heatmapDecaySec_ = ssmeter::HeatmapGrid::kDefaultDecaySeconds;
    // "drawLines"/"decayMs"/"resolutionPercent"/"smoothLines"/
    // "heatmapEnabled" (every StateIndex entry) have no members here at
    // all -- all are pure UI preferences the DSP side never reads, the
    // same "nothing for the DSP side to do" convention ../compressor's
    // own "presetDir" State uses.

    DISTRHO_DECLARE_NON_COPYABLE(PluginPhaseScope)
};

END_NAMESPACE_DISTRHO
