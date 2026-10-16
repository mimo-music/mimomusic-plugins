// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// LimiterGraphWidget.hpp
//
// The Limiter's graph -- its own widget rather than CompressorGraphWidget
// with handles hidden (Ratio/Up Ratio handles there would be draggable but
// do nothing here). The transfer curve shows the whole chain -- gain computer at the
// effective threshold, the (capped) gain after it, then the clip stage --
// computed with the same functions the audio runs (LimiterDsp.hpp).
//
//   - Threshold: draggable handle on the curve (horizontal drag).
//   - Ceiling: dashed horizontal line (vertical drag).
//   - Clip Knee (Soft clip only): a dotted line below the Ceiling, with the
//     band between the two shaded -- where the soft clip rounds the signal
//     off. Dragging the dotted line sets the knee width; dragging the
//     Ceiling moves the band with it.
//   - When there is gain after the limiter (Boost/Output Gain), a faint
//     line shows the limiter alone, with an arrow labelled with that gain
//     up to the full curve -- so a curve that moved up reads as "gain added
//     after the limiter", not as a different limiting point.
//   - A note in the top-left corner when the gain after the limiter is
//     capped, or the limiter limits below Threshold, so a value that seems
//     to do nothing explains itself.
//   - A CLIP indicator next to the GR meter: how much the clip stage took
//     off, with a short hold, so overshoot is visible.

#pragma once

#include "EditorSubWidget.hpp"

#include <chrono>
#include <functional>

START_NAMESPACE_DGL

class LimiterGraphWidget : public EditorSubWidget
{
public:
    explicit LimiterGraphWidget(NanoTopLevelWidget* parent);

    struct CurveParams
    {
        float thresholdDb = -6.0f;
        float ceilingDb = 0.0f;
        float kneeDb = 1.0f;               // the gain computer's knee (0 in True Peak mode)
        double ratio = 20.0;               // the gain computer's ratio (infinity in True Peak mode)
        float effectiveThresholdDb = -6.0f; // from computeLimiterStaging()
        float postGainDb = 0.0f;           // ditto, after the cap
        float requestedPostGainDb = 0.0f;  // Output Gain + Boost as set
        bool softClip = false;
        float clipKneeDb = 1.0f;
    };

    void setCurveParams(const CurveParams& p);

    // The live readout, every uiIdle() tick. clipDb <= 0: the clip stage's
    // reduction on the loudest sample of the last block.
    void setLiveLevels(float detectorLevelDb, float gainReductionDb, float clipDb);

    void setThresholdRange(float minDb, float maxDb) { thresholdMinDb_ = minDb; thresholdMaxDb_ = maxDb; }
    void setCeilingRange(float minDb, float maxDb) { ceilingMinDb_ = minDb; ceilingMaxDb_ = maxDb; }
    void setClipKneeRange(float minDb, float maxDb) { clipKneeMinDb_ = minDb; clipKneeMaxDb_ = maxDb; }

    void setOnThresholdChanged(std::function<void(float)> cb) { onThresholdChanged_ = std::move(cb); }
    void setOnCeilingChanged(std::function<void(float)> cb) { onCeilingChanged_ = std::move(cb); }
    void setOnClipKneeChanged(std::function<void(float)> cb) { onClipKneeChanged_ = std::move(cb); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    enum class Handle { None, Threshold, Ceiling, ClipKnee };

    void drawTransferCurve(float x, float y, float w, float h);
    void drawGrMeterColumn(float x, float y, float w, float h);
    void drawGrScale(float x, float y, float h);
    void drawGrHistory(float x, float y, float w, float h);
    void drawClipIndicator(float x, float y, float w, float h);

    float limitedDbFor(float inDb) const; // the gain computer alone
    float outputDbFor(float inDb) const;  // the whole chain, see file comment
    float dbToPxX(float db) const;
    float dbToPxY(float db) const;
    float pxToDbX(float px) const;
    float pxToDbY(float py) const;
    Handle hitHandle(float px, float py) const;

    CurveParams p_;
    float thresholdMinDb_ = -60.0f, thresholdMaxDb_ = 0.0f;
    float ceilingMinDb_ = -12.0f, ceilingMaxDb_ = 0.0f;
    float clipKneeMinDb_ = 0.1f, clipKneeMaxDb_ = 6.0f;
    float detectorLevelDb_ = -120.0f, gainReductionDb_ = 0.0f;
    std::chrono::steady_clock::time_point grSmoothUpdated_{};
    bool grSmoothInit_ = false;

    float grHoldDb_ = 0.0f;
    std::chrono::steady_clock::time_point grHoldUpdated_{};
    std::chrono::steady_clock::time_point grHoldPeakAt_{};

    // Clip indicator: held value of the clip stage's reduction (<= 0).
    float clipHoldDb_ = 0.0f;
    std::chrono::steady_clock::time_point clipHoldPeakAt_{};

    static constexpr int kHistoryLen = 240;
    float history_[kHistoryLen] = {};
    int historyPos_ = 0;

    static constexpr float kGraphMinDb = -60.0f;
    static constexpr float kGraphMaxDb = 6.0f;
    static constexpr float kGrMeterMinDb = -36.0f;
    static constexpr float kGrMeterMaxDb = 0.0f;

    float curveX_ = 0, curveY_ = 0, curveW_ = 0, curveH_ = 0;

    Handle hoverHandle_ = Handle::None;
    Handle draggingHandle_ = Handle::None;
    static constexpr float kHandleHitRadiusPx = 10.0f;

    std::function<void(float)> onThresholdChanged_;
    std::function<void(float)> onCeilingChanged_;
    std::function<void(float)> onClipKneeChanged_;
};

END_NAMESPACE_DGL
