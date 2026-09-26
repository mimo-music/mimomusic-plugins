// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/LoudnessDsp.hpp"

START_NAMESPACE_DISTRHO

// Every Parameter here is either an *output* loudness reading or the
// momentary "Reset" trigger that clears the accumulated Integrated/LRA/
// True-Peak-max statistics -- there is nothing to automate that shapes
// the audio, because this plugin (like ../plugin_phasescope) never
// touches the signal at all (see PluginLoudnessMeter.cpp's own run()
// comment). The "Target" reference line is a State
// (kStateTargetLufs, see its own comment) rather than a Parameter -- it's
// a pure display preference with no DSP-side meaning whatsoever, the
// same reasoning PluginPhaseScope.hpp's own kStateDrawLines/
// kStateHeatmapEnabled use.
enum ParameterIndex
{
    kParamMomentaryLufs = 0,
    kParamShortTermLufs,
    kParamIntegratedLufs,
    kParamLoudnessRangeLu,
    kParamTruePeakL,
    kParamTruePeakR,
    kParamTruePeakMax,
    kParamPlrLu,
    kParamPsrLu,
    // Plain (non-K-weighted) per-channel dBFS readings -- the same
    // ssmeter::PeakMeter/LoudnessMeter ballistics
    // ../plugin_phasescope/PluginPhaseScope.cpp's own L/R bargraphs use,
    // backing this plugin's own L/R vertical bars ("do a vertical bar for
    // l/r separate") via the shared gui/LevelMeterWidget verbatim.
    kParamInputRmsL,
    kParamInputPeakL,
    kParamInputRmsR,
    kParamInputPeakR,
    // No kParamReset trigger Parameter here any more -- see
    // kStateResetRequest's own comment for why "Reset" moved to a State.
    // + [0 .. ssmeter::LufsMeter::kDistributionBinCount) each -- one plain
    // count per loudness bin, backing the "statistical view"
    // ("Youlean shows the example"). Two separate banks, not one --
    // "do me two different distribution plots. one with the short term
    // and one with momentary values" -- since ssmeter::LufsMeter itself
    // now tracks two independent DistributionHistogram tallies (see its
    // own comment). Unpacked (unlike ../plugin_phasescope's own
    // goniometer/heatmap banks) since 120 total Parameters still needs no
    // bit-packing scheme.
    kParamDistBinBase,
    kParamDistBinMomentaryBase = kParamDistBinBase + ssmeter::LufsMeter::kDistributionBinCount,
    kParamCount = kParamDistBinMomentaryBase + ssmeter::LufsMeter::kDistributionBinCount
};

enum StateIndex
{
    // The reference line drawn on the bar meter and history graph, and
    // used as the "how far below/above target" framing in the numeric
    // readout panel -- one of a small, fixed set of well-known loudness
    // targets (EBU R128, ATSC A/85, common streaming platforms) cycled by
    // the UI's own "Target" button, or "off" to draw no reference line at
    // all. Stored as a string ("-23.0" or "off") rather than a numeric
    // Parameter since the DSP side never reads it -- purely a UI
    // rendering preference, but a *lasting* one worth a host remembering
    // across a session, the same reasoning PluginPhaseScope.hpp's own
    // kStateHeatmapEnabled uses.
    kStateTargetLufs = 0,
    // Whether the main display's center panel currently shows the
    // scrolling history graph or the loudness Distribution histogram --
    // the same "lasting UI choice, not a DSP-relevant control" reasoning
    // as kStateTargetLufs above.
    kStateShowDistribution = 1,
    // "Reset" itself: a one-shot event ("clear the accumulated Integrated/
    // LRA/Distribution/True-Peak-max statistics"), moved here from a
    // kParameterIsTrigger Parameter (kParamReset) because the button had
    // no effect at all, with a real root cause beyond the Distribution-
    // range/resettling explanations first suspected: a DPF trigger
    // Parameter is backed, for
    // formats like LV2, by a plain control-port value the host is
    // expected to snap back to its default after each block (DPF's own
    // LV2 bridge code says so explicitly: "NOTE: host is responsible for
    // auto-updating control port buffers"). If a host never does that
    // (plausible, unverifiable from here without that specific host to
    // test against), the port stays pinned at the trigger's "fired" value
    // forever, and DPF's own bridge only calls Plugin::setParameterValue()
    // when the incoming value actually *differs* from the last one it
    // saw -- so only the very first click, if any, could ever have
    // registered; every later one is silently swallowed before this
    // plugin's own code ever runs -- which matches the symptom (no effect
    // at all) exactly. A State
    // push (UI::setState()/Plugin::setState()) has no such equality gate
    // anywhere in DPF's own dispatch for any backend (checked directly in
    // dpf/distrho/src/Distrho{UI,PluginLV2,PluginInternal}.*) -- every
    // call is delivered as its own discrete message regardless of whether
    // the string value repeats, which is exactly the "always fires"
    // guarantee a click-triggered reset actually needs. The value itself
    // is therefore never inspected by setState() below (an incrementing
    // counter string, purely so two rapid identical-looking clicks are
    // still visibly distinct in a log) -- it's the call arriving at all
    // that means "reset now", not what it says. `kStateIsOnlyForDSP`
    // since there is nothing for the UI to mirror back.
    kStateResetRequest = 2,
    kStateCount
};

class PluginLoudnessMeter : public Plugin
{
public:
    PluginLoudnessMeter();

protected:
    uint32_t getVersion() const override { return d_version(1, 1, 0); } // 1.1: L/R input bars, a Distribution histogram view, and a smoother/spline-drawn history graph

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
    ssmeter::LufsMeter meter_;
    // Plain per-channel ballistics -- see kParamInputRmsL's own comment.
    ssmeter::LoudnessMeter rmsL_, rmsR_;
    ssmeter::PeakMeter peakL_, peakR_;

    float momentaryLufs_ = float(ssmeter::LufsMeter::kSilenceLufs);
    float shortTermLufs_ = float(ssmeter::LufsMeter::kSilenceLufs);
    float integratedLufs_ = float(ssmeter::LoudnessHistogram::kMinLufs);
    float loudnessRangeLu_ = 0.0f;
    float truePeakL_ = -120.0f;
    float truePeakR_ = -120.0f;
    float truePeakMax_ = -120.0f;
    float plrLu_ = 0.0f;
    float psrLu_ = 0.0f;
    float inputRmsL_ = -120.0f, inputPeakL_ = -120.0f;
    float inputRmsR_ = -120.0f, inputPeakR_ = -120.0f;
    // "targetLufs"/"showDistribution" (every StateIndex entry) have no
    // members here at all -- pure UI preferences the DSP side never
    // reads, the same "nothing for the DSP side to do" convention
    // PluginPhaseScope.hpp's own State members (or lack thereof) use.

    DISTRHO_DECLARE_NON_COPYABLE(PluginLoudnessMeter)
};

END_NAMESPACE_DISTRHO
