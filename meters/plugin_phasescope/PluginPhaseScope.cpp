// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "PluginPhaseScope.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

USE_NAMESPACE_DISTRHO
using namespace ssmeter;

PluginPhaseScope::PluginPhaseScope()
    : Plugin(kParamCount, 0 /* programs */, kStateCount)
{
    sampleRateChanged(getSampleRate());
}

void PluginPhaseScope::initAudioPort(bool input, uint32_t index, AudioPort& port)
{
    port.groupId = kPortGroupStereo;
    Plugin::initAudioPort(input, index, port);
}

void PluginPhaseScope::initParameter(uint32_t index, Parameter& parameter)
{
    if (index >= kParamScopePacketBase && index < kParamScopePacketBase + GoniometerBuffer::kScopeBufferSize)
    {
        const uint32_t slot = index - kParamScopePacketBase;
        char name[32], symbol[24];
        std::snprintf(name, sizeof(name), "Scope Point %u", slot);
        std::snprintf(symbol, sizeof(symbol), "scope_point_%u", slot);
        // An exact packed integer, not a continuous value -- kParameterIsInteger
        // tells a host not to interpolate/smooth automation on it, which
        // would corrupt the packed bit pattern (same reasoning as
        // ../statespace's own SpectrumAnalyzer packet Parameters).
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
        parameter.name = name;
        parameter.symbol = symbol;
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float((1u << (2 * GoniometerBuffer::kBitsPerCoord)) - 1);
        return;
    }

    if (index >= kParamHeatmapPacketBase && index < kParamHeatmapPacketBase + HeatmapGrid::kNumPackets)
    {
        const uint32_t slot = index - kParamHeatmapPacketBase;
        char name[32], symbol[24];
        std::snprintf(name, sizeof(name), "Heatmap Cells %u", slot);
        std::snprintf(symbol, sizeof(symbol), "heatmap_cells_%u", slot);
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
        parameter.name = name;
        parameter.symbol = symbol;
        parameter.ranges.def = 0.0f;
        parameter.ranges.min = 0.0f;
        parameter.ranges.max = float((1u << (HeatmapGrid::kCellsPerPacket * HeatmapGrid::kBitsPerCell)) - 1);
        return;
    }

    switch (index)
    {
    case kParamMeterInputRmsL:
    case kParamMeterInputPeakL:
    case kParamMeterInputRmsR:
    case kParamMeterInputPeakR:
    {
        static const char* names[] = { "Input RMS L", "Input Peak L", "Input RMS R", "Input Peak R" };
        static const char* symbols[] = { "meter_in_rms_l", "meter_in_peak_l", "meter_in_rms_r", "meter_in_peak_r" };
        const uint32_t i = index - kParamMeterInputRmsL;
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = names[i];
        parameter.symbol = symbols[i];
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    }
    case kParamMeterCorrelation:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Correlation";
        parameter.symbol = "meter_correlation";
        parameter.ranges.def = 0.0f; parameter.ranges.min = -1.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamMeterCorrelationMinHold:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Correlation Min Hold";
        parameter.symbol = "meter_correlation_min_hold";
        parameter.ranges.def = 1.0f; parameter.ranges.min = -1.0f; parameter.ranges.max = 1.0f;
        break;
    case kParamMeterWidthPercent:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Stereo Width";
        parameter.symbol = "meter_width_percent";
        parameter.unit = "%";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f; parameter.ranges.max = 200.0f;
        break;
    case kParamDisplayGain:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Display Gain";
        parameter.symbol = "display_gain";
        parameter.unit = "dB";
        // Purely a plot-scale trim -- never applied to the audio, see
        // this file's own run() comment. +-24dB comfortably covers
        // zooming in on a quiet signal or pulling back from a hot one.
        parameter.ranges.def = 0.0f; parameter.ranges.min = -24.0f; parameter.ranges.max = 24.0f;
        break;
    case kParamMeterPlotPeakDb:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput;
        parameter.name = "Plot Peak";
        parameter.symbol = "meter_plot_peak_db";
        parameter.unit = "dB";
        parameter.ranges.def = -120.0f; parameter.ranges.min = -120.0f; parameter.ranges.max = 12.0f;
        break;
    case kParamHeatmapDecaySec:
        parameter.hints = kParameterIsAutomatable;
        parameter.name = "Heatmap Decay";
        parameter.symbol = "heatmap_decay_sec";
        parameter.unit = "s";
        parameter.ranges.def = float(HeatmapGrid::kDefaultDecaySeconds);
        parameter.ranges.min = float(HeatmapGrid::kMinDecaySeconds);
        parameter.ranges.max = float(HeatmapGrid::kMaxDecaySeconds);
        break;
    case kParamScopeWriteIndex:
        parameter.hints = kParameterIsAutomatable | kParameterIsOutput | kParameterIsInteger;
        parameter.name = "Scope Write Slot";
        parameter.symbol = "scope_write_slot";
        parameter.ranges.def = 0.0f; parameter.ranges.min = 0.0f;
        parameter.ranges.max = float(GoniometerBuffer::kScopeBufferSize - 1);
        break;
    }
}

void PluginPhaseScope::initState(uint32_t index, State& state)
{
    switch (index)
    {
    case kStateDrawLines:
        state.key = "drawLines";
        state.label = "Draw Lines (vs. Dots)";
        state.hints = 0; // UI convenience only -- see kStateDrawLines's own comment
        state.defaultValue = "1"; // Lines (and Smooth) on by default
        break;
    case kStateDecayMs:
        state.key = "decayMs";
        state.label = "Goniometer Decay (ms)";
        state.hints = 0; // UI convenience only -- see kStateDecayMs's own comment
        state.defaultValue = "600"; // matches GoniometerBuffer::kDefaultDecaySeconds
        break;
    case kStateResolutionPercent:
        state.key = "resolutionPercent";
        state.label = "Goniometer Resolution (%)";
        state.hints = 0; // UI convenience only -- see kStateResolutionPercent's own comment
        state.defaultValue = "100";
        break;
    case kStateSmoothLines:
        state.key = "smoothLines";
        state.label = "Goniometer Smooth Lines";
        state.hints = 0; // UI convenience only -- see kStateSmoothLines's own comment
        state.defaultValue = "1"; // on by default
        break;
    case kStateHeatmapEnabled:
        state.key = "heatmapEnabled";
        state.label = "Goniometer Heatmap";
        state.hints = 0; // UI convenience only -- see kStateHeatmapEnabled's own comment
        state.defaultValue = "0";
        break;
    case kStateAutoGainActive:
        state.key = "autoGainActive";
        state.label = "Auto Gain Active";
        state.hints = 0; // UI convenience only -- see kStateAutoGainActive's own comment
        state.defaultValue = "0";
        break;
    case kStateCompact:
        state.key = "compact";
        state.label = "Compact View";
        state.hints = 0; // UI convenience only -- see kStateCompact's own comment
        state.defaultValue = "0";
        break;
    }
}

float PluginPhaseScope::getParameterValue(uint32_t index) const
{
    if (index >= kParamScopePacketBase && index < kParamScopePacketBase + GoniometerBuffer::kScopeBufferSize)
        return goniometer_.getPackedPoint(int(index - kParamScopePacketBase));

    if (index >= kParamHeatmapPacketBase && index < kParamHeatmapPacketBase + HeatmapGrid::kNumPackets)
        return heatmap_.getPackedQuad(int(index - kParamHeatmapPacketBase));

    switch (index)
    {
    case kParamMeterInputRmsL:  return meterInputRmsL_;
    case kParamMeterInputPeakL: return meterInputPeakL_;
    case kParamMeterInputRmsR:  return meterInputRmsR_;
    case kParamMeterInputPeakR: return meterInputPeakR_;
    case kParamMeterCorrelation:        return meterCorrelation_;
    case kParamMeterCorrelationMinHold: return meterCorrelationMinHold_;
    case kParamMeterWidthPercent:       return meterWidthPercent_;
    case kParamDisplayGain: return float(displayGainDb_);
    case kParamMeterPlotPeakDb: return meterPlotPeakDb_;
    case kParamHeatmapDecaySec: return float(heatmapDecaySec_);
    // Reduced mod kScopeBufferSize (not the raw ever-growing sample
    // counter) specifically so this stays an exact, host-safe small
    // integer indefinitely -- see GoniometerBuffer::getWriteIndex()'s own
    // comment; the UI only ever needs "which slot is newest" to derive
    // fade-age, never the absolute count.
    case kParamScopeWriteIndex: return float(goniometer_.getWriteIndex() % uint32_t(GoniometerBuffer::kScopeBufferSize));
    default: return 0.0f;
    }
}

void PluginPhaseScope::setParameterValue(uint32_t index, float value)
{
    switch (index)
    {
    case kParamDisplayGain:
        // The heatmap's own cells are addressed in the *plotted* (already
        // Gain-scaled) coordinate space -- a history built up at one Gain
        // no longer lines up with where the live trail draws once Gain
        // changes. An earlier fix cleared the whole heatmap on every Gain
        // change (later gated to a 1dB-or-more threshold once Auto Gain
        // could drive this continuously) -- still wrong with Auto Gain on:
        // spots vanished instead of fading out, because a reset throws
        // the whole accumulated history away every time, which with Auto
        // Gain continuously nudging Gain meant it could never actually
        // accumulate (or fade out on its own Decay's own terms) at all.
        // `HeatmapGrid::rescale()` repositions every existing cell to
        // where it belongs at the new scale instead -- history survives a
        // Gain change and keeps fading out normally, rather than being
        // wiped by one. The ratio is exactly how much the *plotted*
        // coordinate space itself just scaled by (dbToLinear(new) /
        // dbToLinear(old) == dbToLinear(new - old)).
        heatmap_.rescale(float(dbToLinear(double(value) - displayGainDb_)));
        displayGainDb_ = value;
        break;
    case kParamHeatmapDecaySec:
        heatmapDecaySec_ = value;
        heatmap_.requestDecaySeconds(heatmapDecaySec_);
        break;
    default:
        // Every other Parameter here is output-only -- a host isn't
        // expected to write them, but silently accepting (and ignoring)
        // rather than asserting is the same tolerant convention this
        // project's other output-only banks (e.g. ../statespace's
        // spectrum packets) already use.
        break;
    }
}

void PluginPhaseScope::setState(const char* key, const char* value)
{
    (void)key; (void)value;
    // Every State here is a pure UI preference -- nothing for the DSP
    // side to do, see each one's own StateIndex comment.
}

void PluginPhaseScope::activate()
{
    loudnessL_.reset(); loudnessR_.reset();
    peakL_.reset(); peakR_.reset();
    stereoImage_.reset();
    plotLevelMeter_.reset();
    goniometer_.reset();
    heatmap_.reset();
}

void PluginPhaseScope::run(const float** inputs, float** outputs, uint32_t frames)
{
    const float* inL = inputs[0];
    const float* inR = inputs[1];
    float* outL = outputs[0];
    float* outR = outputs[1];

    // Purely observational -- see this file's own header comment. Copy
    // through untouched before (or instead of, if the host already
    // aliases in-place) measuring, so a host that reads the output buffer
    // for its own metering sees exactly the input.
    if (outL != inL) std::copy(inL, inL + frames, outL);
    if (outR != inR) std::copy(inR, inR + frames, outR);

    // Display-only gain, never applied to the audio above: a plain manual
    // dB trim, always in effect -- nothing else touches the plotted scale
    // continuously. "Auto Gain" (see UIPhaseScope.cpp's own button
    // comment) reads kParamMeterPlotPeakDb, computed below from the raw
    // signal, and writes a fresh Gain value directly, entirely UI-side.
    const double manualGain = dbToLinear(displayGainDb_);

    for (uint32_t i = 0; i < frames; ++i)
    {
        const double l = double(inL[i]), r = double(inR[i]);

        peakL_.process(std::fabs(l));
        peakR_.process(std::fabs(r));
        loudnessL_.process(l);
        loudnessR_.process(r);
        stereoImage_.process(l, r);

        // Mid/Side, the same (L+R)/2, (L-R)/2 transform
        // ../compressor's own Mid/Side stereo mode uses (algebraically
        // lossless -- see that project's tests/dsp_regression.cpp test
        // [8]) -- Side on x, Mid on y so a mono signal draws straight up
        // and hard-left/hard-right draw the two diagonals (see
        // GoniometerWidget's own comment for which one is which).
        const double side = 0.5 * (l - r);
        const double mid  = 0.5 * (l + r);

        // Tracks the *raw, unscaled* signal, deliberately independent of
        // Gain -- see kParamMeterPlotPeakDb's own comment.
        plotLevelMeter_.process(std::max(std::fabs(side), std::fabs(mid)));

        const float plottedSide = float(side * manualGain);
        const float plottedMid = float(mid * manualGain);
        if (goniometer_.process(plottedSide, plottedMid))
        {
            // Fed exactly the points the visible trail itself just wrote,
            // at the identical fixed rate -- see HeatmapGrid's own file
            // comment for why this isn't independently re-decimated.
            heatmap_.addPoint(plottedSide, plottedMid);
        }
    }

    meterInputRmsL_ = float(loudnessL_.rmsDb());
    meterInputPeakL_ = float(peakL_.peakDb());
    meterInputRmsR_ = float(loudnessR_.rmsDb());
    meterInputPeakR_ = float(peakR_.peakDb());
    meterCorrelation_ = float(stereoImage_.correlation());
    meterCorrelationMinHold_ = float(stereoImage_.minHoldCorrelation());
    meterWidthPercent_ = float(stereoImage_.widthPercent());
    meterPlotPeakDb_ = float(plotLevelMeter_.levelDb());
}

void PluginPhaseScope::sampleRateChanged(double newSampleRate)
{
    loudnessL_.prepare(newSampleRate); loudnessR_.prepare(newSampleRate);
    peakL_.prepare(newSampleRate); peakR_.prepare(newSampleRate);
    stereoImage_.prepare(newSampleRate);
    plotLevelMeter_.prepare(newSampleRate, kAutoGainTimeConstantSeconds);
    goniometer_.prepare(newSampleRate);
    heatmap_.prepare(); // sample-rate-independent by construction -- see its own file comment
}

START_NAMESPACE_DISTRHO
Plugin* createPlugin() { return new PluginPhaseScope(); }
END_NAMESPACE_DISTRHO
