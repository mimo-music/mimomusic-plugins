// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UILoudnessMeter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::LevelMeterWidget;
using DGL_NAMESPACE::LoudnessMeterWidget;
using DGL_NAMESPACE::SimpleButton;
using DGL_NAMESPACE::TooltipOverlay;

// Spotify/Apple Music/YouTube own widely-published streaming
// normalization targets; SoundCloud does not publish an official one (as
// of this writing it does not normalize playback loudness at all), so its
// own entry is a commonly-cited mastering reference from online
// discussion rather than an official standard -- hence the "*".
const UILoudnessMeter::TargetPreset UILoudnessMeter::kTargetPresets[UILoudnessMeter::kTargetPresetCount] = {
    { "Target: Off",                 0.0f, false },
    { "Target: -23 (EBU R128)",    -23.0f, true },
    { "Target: -24 (ATSC A/85)",   -24.0f, true },
    { "Target: -14 (Spotify)",     -14.0f, true },
    { "Target: -16 (Apple Music)", -16.0f, true },
    { "Target: -14 (YouTube)",     -14.0f, true },
    { "Target: -9 (SoundCloud*)",   -9.0f, true },
};

UILoudnessMeter::UILoudnessMeter()
    : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
{
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    // Plain per-channel dBFS peak/RMS -- "do a vertical bar for l/r
    // separate" -- reusing ../plugin_phasescope/gui's own LevelMeterWidget
    // verbatim, the same widget/convention that plugin already uses for
    // exactly this purpose.
    levelMeter_ = new LevelMeterWidget(this, "IN", 2);
    levelMeter_->setChannelLabels("L", "R");

    meterWidget_ = new LoudnessMeterWidget(this);

    btnReset_ = new SimpleButton(this, "Reset");
    btnReset_->setTooltip("Clear the accumulated Integrated/Range/Distribution/True-Peak-max statistics without affecting the live Momentary/Short-term readings.");
    btnReset_->setCallback([this] {
        // A State push, not a Parameter write -- see kStateResetRequest's
        // own comment in PluginLoudnessMeter.hpp for why: a Parameter
        // trigger's LV2 control-port value can get stuck at its "fired"
        // value if the host never snaps it back to default, silently
        // swallowing every click after the first since DPF's own bridge
        // only forwards a value that actually changed. setState() has no
        // such equality gate in DPF for any backend, so it always fires.
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%u", ++resetRequestCounter_);
        setState("resetRequest", buf);
        resetFlashStart_ = std::chrono::steady_clock::now();
        resetFlashActive_ = true;
    });

    btnTarget_ = new SimpleButton(this, "");
    btnTarget_->setTooltip("Cycle the reference line drawn on the bar meter/graph/both Distribution plots through a few well-known loudness targets (including Spotify/Apple Music/YouTube/SoundCloud), or off.");
    btnTarget_->setCallback([this] {
        targetPresetIndex_ = (targetPresetIndex_ + 1) % kTargetPresetCount;
        const TargetPreset& p = kTargetPresets[targetPresetIndex_];
        char buf[16];
        if (p.enabled) std::snprintf(buf, sizeof(buf), "%.1f", double(p.lufs));
        else std::snprintf(buf, sizeof(buf), "off");
        setState("targetLufs", buf);
        refreshTargetLabel();
    });
    refreshTargetLabel();

    btnView_ = new SimpleButton(this, "");
    btnView_->setTooltip("Switch the center panel between the scrolling history graph and the Momentary/Short-term loudness Distribution plots.");
    btnView_->setCallback([this] {
        showDistribution_ = !showDistribution_;
        setState("showDistribution", showDistribution_ ? "1" : "0");
        refreshViewLabel();
    });
    refreshViewLabel();

    tooltip_ = new TooltipOverlay(this);

    layout();
}

void UILoudnessMeter::refreshTargetLabel()
{
    btnTarget_->setLabel(kTargetPresets[targetPresetIndex_].label);
    const TargetPreset& p = kTargetPresets[targetPresetIndex_];
    btnTarget_->setColors(p.enabled ? Color(50, 100, 130) : Color(58, 62, 70),
                           p.enabled ? Color(65, 120, 150) : Color(78, 84, 96),
                           Color(230, 232, 236));
    meterWidget_->setTarget(p.lufs, p.enabled);
    btnTarget_->repaint();
}

void UILoudnessMeter::refreshViewLabel()
{
    btnView_->setLabel(showDistribution_ ? "View: Distribution" : "View: Graph");
    btnView_->setColors(showDistribution_ ? Color(50, 100, 130) : Color(58, 62, 70),
                         showDistribution_ ? Color(65, 120, 150) : Color(78, 84, 96),
                         Color(230, 232, 236));
    meterWidget_->setShowDistribution(showDistribution_);
    btnView_->repaint();
}

void UILoudnessMeter::layout()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    constexpr float pad = 6.0f;
    constexpr float topBarH = 28.0f;
    constexpr float gap = 6.0f;
    constexpr float meterW = 100.0f;

    btnReset_->setAbsolutePos(int(pad), int(pad));
    btnReset_->setSize(80, uint(topBarH));

    btnTarget_->setAbsolutePos(int(pad + 80.0f + gap), int(pad));
    btnTarget_->setSize(190, uint(topBarH));

    btnView_->setAbsolutePos(int(pad + 80.0f + gap + 190.0f + gap), int(pad));
    btnView_->setSize(140, uint(topBarH));

    const float mainY = topBarH + pad * 2.0f;
    const float mainH = std::max(10.0f, h - mainY - pad);

    levelMeter_->setAbsolutePos(int(pad), int(mainY));
    levelMeter_->setSize(uint(meterW), uint(mainH));

    const float meterX = pad + meterW + pad;
    meterWidget_->setAbsolutePos(int(meterX), int(mainY));
    meterWidget_->setSize(uint(std::max(10.0f, w - meterX - pad)), uint(mainH));

    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));
}

void UILoudnessMeter::onNanoDisplay()
{
    fillColor(Color(20, 21, 25));
    beginPath();
    rect(0, 0, float(getWidth()), float(getHeight()));
    fill();
}

void UILoudnessMeter::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UILoudnessMeter::pushCurrentReadingsToWidget()
{
    meterWidget_->setReadings(momentaryLufs_, shortTermLufs_, integratedLufs_, loudnessRangeLu_,
                               truePeakL_, truePeakR_, truePeakMax_, plrLu_, psrLu_);
}

void UILoudnessMeter::parameterChanged(uint32_t index, float value)
{
    if (index >= kParamDistBinBase && index < kParamDistBinBase + uint32_t(kDistributionBinCount))
    {
        distributionCounts_[index - kParamDistBinBase] = value;
        meterWidget_->setDistributionShortTerm(distributionCounts_, kDistributionBinCount);
        return;
    }
    if (index >= kParamDistBinMomentaryBase && index < kParamDistBinMomentaryBase + uint32_t(kDistributionBinCount))
    {
        distributionMomentaryCounts_[index - kParamDistBinMomentaryBase] = value;
        meterWidget_->setDistributionMomentary(distributionMomentaryCounts_, kDistributionBinCount);
        return;
    }

    switch (index)
    {
    case kParamMomentaryLufs:   momentaryLufs_ = value; break;
    case kParamShortTermLufs:   shortTermLufs_ = value; break;
    case kParamIntegratedLufs:  integratedLufs_ = value; break;
    case kParamLoudnessRangeLu: loudnessRangeLu_ = value; break;
    case kParamTruePeakL:       truePeakL_ = value; break;
    case kParamTruePeakR:       truePeakR_ = value; break;
    case kParamTruePeakMax:     truePeakMax_ = value; break;
    case kParamPlrLu:           plrLu_ = value; break;
    case kParamPsrLu:           psrLu_ = value; break;
    case kParamInputRmsL:
        inputRmsL_ = value;
        levelMeter_->setChannel(0, inputRmsL_, inputPeakL_);
        return;
    case kParamInputPeakL:
        inputPeakL_ = value;
        levelMeter_->setChannel(0, inputRmsL_, inputPeakL_);
        return;
    case kParamInputRmsR:
        inputRmsR_ = value;
        levelMeter_->setChannel(1, inputRmsR_, inputPeakR_);
        return;
    case kParamInputPeakR:
        inputPeakR_ = value;
        levelMeter_->setChannel(1, inputRmsR_, inputPeakR_);
        return;
    default: return;
    }
    pushCurrentReadingsToWidget();
}

void UILoudnessMeter::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "targetLufs") == 0)
    {
        const bool off = std::strcmp(value, "off") == 0;
        for (int i = 0; i < kTargetPresetCount; ++i)
        {
            if (off && !kTargetPresets[i].enabled) { targetPresetIndex_ = i; break; }
            if (!off && kTargetPresets[i].enabled &&
                std::fabs(kTargetPresets[i].lufs - float(std::atof(value))) < 0.05f)
            {
                targetPresetIndex_ = i;
                break;
            }
        }
        refreshTargetLabel();
        return;
    }
    if (std::strcmp(key, "showDistribution") == 0)
    {
        showDistribution_ = std::strcmp(value, "1") == 0;
        refreshViewLabel();
    }
}

void UILoudnessMeter::uiIdle()
{
    // A steady ~10Hz cadence, matching the DSP side's own 100ms gating-
    // block period -- see LoudnessMeterWidget::pushHistorySample()'s own
    // comment for why this is timed by the wall clock rather than by
    // parameterChanged() pushes or a fixed uiIdle() call count.
    const auto now = std::chrono::steady_clock::now();
    if (now - lastHistoryPush_ >= std::chrono::milliseconds(100))
    {
        lastHistoryPush_ = now;
        meterWidget_->pushHistorySample(momentaryLufs_, shortTermLufs_);
    }

    if (resetFlashActive_)
    {
        constexpr float kFlashSeconds = 0.6f;
        const float elapsed = std::chrono::duration<float>(now - resetFlashStart_).count();
        const float intensity = std::max(0.0f, 1.0f - elapsed / kFlashSeconds);
        meterWidget_->setResetFlash(intensity);
        if (intensity <= 0.0f) resetFlashActive_ = false;
    }

    updateTooltip();
}

void UILoudnessMeter::updateTooltip()
{
    SimpleButton* buttons[] = { btnReset_, btnTarget_, btnView_ };
    for (SimpleButton* b : buttons)
    {
        if (b->isHovered() && !b->tooltip().empty())
        {
            const auto area = b->getAbsoluteArea();
            tooltip_->show(b->tooltip(), float(area.getX()), float(area.getY()),
                           float(area.getWidth()), float(area.getHeight()));
            return;
        }
    }
    tooltip_->hide();
}

UI* createUI() { return new UILoudnessMeter(); }

END_NAMESPACE_DISTRHO
