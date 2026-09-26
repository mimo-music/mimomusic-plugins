// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// LimiterGraphWidget.hpp
//
// The Limiter's own graph -- deliberately a *separate*, smaller widget from
// CompressorGraphWidget rather than that widget with some controls hidden:
// the previous Limiter reused CompressorGraphWidget wholesale (Ratio
// and Up Ratio handles, a Knee scroll-to-nudge, Volume Compensation, three
// stereo modes) even though none of those were meant to be the point of a
// "generic, simple" limiter -- and a couple of them (Ratio/Up Ratio, which
// this plugin no longer exposes as Parameters at all) would have been
// visually draggable handles that silently did nothing to the actual sound,
// which is worse than not having them. This widget only ever shows and
// edits the two numbers a limiter is conventionally about:
//
//   - Threshold: where limiting starts (a draggable handle on the curve,
//     same interaction as CompressorGraphWidget's own Threshold handle).
//   - Ceiling: the hard output cap (a draggable horizontal dashed line,
//     since it's an *output*-axis-only value, not a point on the curve).
//
// The transfer curve itself still comes from the shared
// sscomp::computeGainReductionDb() (same pure function every other plugin
// in this project uses -- no new DSP math here), just with a fixed high
// ratio and tight knee baked in (kFixedRatio/kFixedKnee below) rather than
// exposed as their own draggable handles/fields -- generic settings, kept
// simple.
//
// The gain-reduction meter column + scale + peak-hold + history strip are
// the same ballistics and look as CompressorGraphWidget's (see that file's
// own comments for why), just single-channel (a limiter's one detector,
// always looking at both channels together) and one-directional (a limiter
// only ever cuts, never boosts -- there's no Up Ratio here to report a
// boost with), which is simpler than copying its bipolar version.

#pragma once

#include "NanoVG.hpp"

#include <chrono>
#include <functional>

START_NAMESPACE_DGL

class LimiterGraphWidget : public NanoSubWidget
{
public:
    explicit LimiterGraphWidget(Widget* parent);

    // The static curve shape -- called whenever Threshold, Ceiling, or Knee
    // change. Ratio is *not* a parameter here (see PluginLimiter.cpp's
    // kFixedRatio) -- kFixedRatio below must stay equal to that constant,
    // or this curve would visibly disagree with what the DSP actually does.
    void setCurveParams(float thresholdDb, float ceilingDb, float kneeDb);

    // The live, per-frame readout -- called every uiIdle() tick.
    void setLiveLevels(float detectorLevelDb, float gainReductionDb);

    void setThresholdRange(float minDb, float maxDb) { thresholdMinDb_ = minDb; thresholdMaxDb_ = maxDb; }
    void setCeilingRange(float minDb, float maxDb) { ceilingMinDb_ = minDb; ceilingMaxDb_ = maxDb; }

    void setOnThresholdChanged(std::function<void(float)> cb) { onThresholdChanged_ = std::move(cb); }
    void setOnCeilingChanged(std::function<void(float)> cb) { onCeilingChanged_ = std::move(cb); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    enum class Handle { None, Threshold, Ceiling };

    void drawTransferCurve(float x, float y, float w, float h);
    void drawGrMeterColumn(float x, float y, float w, float h);
    void drawGrScale(float x, float y, float h);
    void drawGrHistory(float x, float y, float w, float h);

    float dbToPxX(float db) const;
    float dbToPxY(float db) const;
    float pxToDbX(float px) const;
    float pxToDbY(float py) const;
    Handle hitHandle(float px, float py) const;

    float thresholdDb_ = -6.0f, ceilingDb_ = 0.0f, kneeDb_ = 1.0f;
    float thresholdMinDb_ = -60.0f, thresholdMaxDb_ = 0.0f;
    float ceilingMinDb_ = -12.0f, ceilingMaxDb_ = 0.0f;
    float detectorLevelDb_ = -120.0f, gainReductionDb_ = 0.0f;
    std::chrono::steady_clock::time_point grSmoothUpdated_{};
    bool grSmoothInit_ = false;

    // Not a Parameter -- see PluginLimiter.cpp's kFixedRatio, which this
    // must stay equal to.
    static constexpr float kFixedRatio = 20.0f;

    float grHoldDb_ = 0.0f;
    std::chrono::steady_clock::time_point grHoldUpdated_{};
    // Wall-clock timestamp of the currently-held excursion -- see
    // CompressorGraphWidget::grHoldPeakAt_'s own comment. Recovery waits
    // kGrHoldTimeSec after this before sliding grHoldDb_ back toward 0dB.
    std::chrono::steady_clock::time_point grHoldPeakAt_{};

    static constexpr int kHistoryLen = 240;
    float history_[kHistoryLen] = {};
    int historyPos_ = 0;

    static constexpr float kGraphMinDb = -60.0f;
    static constexpr float kGraphMaxDb = 6.0f;
    // One-directional (a limiter only ever cuts) -- see file comment.
    static constexpr float kGrMeterMinDb = -36.0f;
    static constexpr float kGrMeterMaxDb = 0.0f;

    float curveX_ = 0, curveY_ = 0, curveW_ = 0, curveH_ = 0;

    Handle hoverHandle_ = Handle::None;
    Handle draggingHandle_ = Handle::None;
    static constexpr float kHandleHitRadiusPx = 10.0f;

    std::function<void(float)> onThresholdChanged_;
    std::function<void(float)> onCeilingChanged_;
};

END_NAMESPACE_DGL
