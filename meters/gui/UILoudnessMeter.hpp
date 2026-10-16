// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/LoudnessDsp.hpp"
#include "../plugin_loudness/PluginLoudnessMeter.hpp"
#include "LevelMeterWidget.hpp"
#include "LoudnessMeterWidget.hpp"
#include "SimpleButton.hpp"
#include "TooltipOverlay.hpp"

#include <chrono>

START_NAMESPACE_DISTRHO

// A small, dedicated UI for a small, dedicated plugin -- see
// PluginLoudnessMeter.hpp's own top comment: every Parameter is an
// output loudness reading or the momentary "Reset" trigger, so there's
// nothing here shaping audio to automate, and (like UIPhaseScope) no
// preset Save/Load or A/B/Undo-Redo either. "Target" and "View" (the
// history-graph-vs-Distribution toggle) are both States (see
// kStateTargetLufs/kStateShowDistribution's own comments), cycled/toggled
// by their own buttons here exactly like UIPhaseScope's Draw Lines/
// Smooth/Heatmap buttons cycle their own States.
class UILoudnessMeter : public UI
{
public:
    UILoudnessMeter();

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onDisplay() override; // DPF's frame, with the real pixel ratio -- see EditorSubWidget.hpp
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

private:
    void layout();
    void refreshTargetLabel();
    void refreshViewLabel();
    void pushCurrentReadingsToWidget();
    void updateTooltip();

    // The small, fixed set of well-known loudness targets this button
    // cycles through -- see kStateTargetLufs's own comment for why this
    // is a State, not a Parameter. Index 0 ("Off") draws no reference
    // line at all. Includes the streaming-platform figures
    // (Spotify/Apple Music/YouTube/SoundCloud) that used to be drawn as
    // permanent reference lines directly on the Distribution plot --
    // cycling Target to one of them now draws
    // that same single reference line everywhere Target already draws one
    // (bar meter, history graph, both Distribution plots), rather than a
    // whole separate always-on set of platform lines cluttering just the
    // one view.
    struct TargetPreset { const char* label; float lufs; bool enabled; };
    static const TargetPreset kTargetPresets[];
    static constexpr int kTargetPresetCount = 7;
    int targetPresetIndex_ = 1; // defaults to EBU R128 (-23 LUFS), matching kStateTargetLufs's own default

    static constexpr int kDistributionBinCount = ssmeter::LufsMeter::kDistributionBinCount;

    DGL_NAMESPACE::LevelMeterWidget* levelMeter_ = nullptr;
    DGL_NAMESPACE::LoudnessMeterWidget* meterWidget_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnReset_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnTarget_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnView_ = nullptr;
    DGL_NAMESPACE::TooltipOverlay* tooltip_ = nullptr;

    bool showDistribution_ = false;

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
    float distributionCounts_[kDistributionBinCount] = {};
    float distributionMomentaryCounts_[kDistributionBinCount] = {};

    // Wall-clock-gated, not tied to parameterChanged() or a fixed
    // uiIdle() call count -- see LoudnessMeterWidget::pushHistorySample()'s
    // own comment for why the history graph needs an evenly-spaced time
    // axis regardless of the host's own UI refresh rate or whether a
    // given reading happened to actually change value.
    std::chrono::steady_clock::time_point lastHistoryPush_{};

    // When the Reset button was last clicked (default-constructed, i.e.
    // "never") -- drives LoudnessMeterWidget::setResetFlash()'s own brief
    // decay. See that function's own comment for why this exists.
    std::chrono::steady_clock::time_point resetFlashStart_{};
    bool resetFlashActive_ = false;
    // Just needs to change on every click -- see kStateResetRequest's own
    // comment for why "Reset" is a State push, not a Parameter write.
    unsigned resetRequestCounter_ = 0;

    DISTRHO_DECLARE_NON_COPYABLE(UILoudnessMeter)
};

END_NAMESPACE_DISTRHO
