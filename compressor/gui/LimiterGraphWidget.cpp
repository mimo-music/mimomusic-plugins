// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "LimiterGraphWidget.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

LimiterGraphWidget::LimiterGraphWidget(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
    std::fill(std::begin(history_), std::end(history_), 0.0f);
}

void LimiterGraphWidget::setCurveParams(float thresholdDb, float ceilingDb, float kneeDb)
{
    thresholdDb_ = thresholdDb;
    ceilingDb_ = ceilingDb;
    kneeDb_ = kneeDb;
    repaint();
}

void LimiterGraphWidget::setLiveLevels(float detectorLevelDb, float gainReductionDb)
{
    detectorLevelDb_ = detectorLevelDb;

    // Smoothed against wall-clock time before being stored/drawn -- see
    // CompressorGraphWidget::smoothLiveValue()'s own comment (the
    // Multiband GR-meter readability fix from the sibling
    // ../statespace project, applied here too: this graph's own live GR
    // number would otherwise flicker too fast to read). The peak-hold
    // marker below stays driven by the raw value on purpose -- a recovering
    // peak is a fundamentally
    // different, already-slow-moving display than a flickering digit.
    if (!grSmoothInit_)
    {
        gainReductionDb_ = gainReductionDb;
        grSmoothInit_ = true;
        grSmoothUpdated_ = std::chrono::steady_clock::now();
    }
    else
    {
        const auto smoothNow = std::chrono::steady_clock::now();
        const float smoothDtSec = std::chrono::duration<float>(smoothNow - grSmoothUpdated_).count();
        grSmoothUpdated_ = smoothNow;
        constexpr float kTauSec = 0.22f; // same value as MultibandGraphWidget::setBandGainDb()
        const float coeff = smoothDtSec > 0.0f ? std::exp(-smoothDtSec / kTauSec) : 1.0f;
        gainReductionDb_ = coeff * gainReductionDb_ + (1.0f - coeff) * gainReductionDb;
    }

    // Peak-hold: snap to a deeper cut, otherwise sit still at that cut for
    // kGrHoldTimeSec before recovering back toward 0dB -- identical
    // ballistic (now including the same hold plateau) to
    // CompressorGraphWidget::updateHold(), just one-directional (a
    // limiter's gain reduction is never positive) so there's no
    // std::fabs()/std::copysign() needed.
    const auto now = std::chrono::steady_clock::now();
    const float dtSec = std::chrono::duration<float>(now - grHoldUpdated_).count();
    grHoldUpdated_ = now;
    constexpr float kGrHoldTimeSec = 1.5f;
    constexpr float kRecoverDbPerSec = 12.0f;
    if (gainReductionDb <= grHoldDb_)
    {
        grHoldDb_ = gainReductionDb;
        grHoldPeakAt_ = now;
    }
    else if (std::chrono::duration<float>(now - grHoldPeakAt_).count() >= kGrHoldTimeSec)
    {
        grHoldDb_ = std::min(0.0f, grHoldDb_ + kRecoverDbPerSec * dtSec);
    }

    history_[historyPos_] = gainReductionDb;
    historyPos_ = (historyPos_ + 1) % kHistoryLen;
    repaint();
}

namespace {
inline float dbToAxis(float db, float minDb, float maxDb)
{
    return std::max(0.0f, std::min(1.0f, (db - minDb) / (maxDb - minDb)));
}
}

float LimiterGraphWidget::dbToPxX(float db) const { return curveX_ + dbToAxis(db, kGraphMinDb, kGraphMaxDb) * curveW_; }
float LimiterGraphWidget::dbToPxY(float db) const { return curveY_ + curveH_ - dbToAxis(db, kGraphMinDb, kGraphMaxDb) * curveH_; }
float LimiterGraphWidget::pxToDbX(float px) const
{
    if (curveW_ <= 0.0f) return kGraphMinDb;
    const float t = std::max(0.0f, std::min(1.0f, (px - curveX_) / curveW_));
    return kGraphMinDb + t * (kGraphMaxDb - kGraphMinDb);
}
float LimiterGraphWidget::pxToDbY(float py) const
{
    if (curveH_ <= 0.0f) return kGraphMinDb;
    const float t = std::max(0.0f, std::min(1.0f, (curveY_ + curveH_ - py) / curveH_));
    return kGraphMinDb + t * (kGraphMaxDb - kGraphMinDb);
}

void LimiterGraphWidget::drawTransferCurve(float x, float y, float w, float h)
{
    auto xToPx = [&](float db) { return dbToPxX(db); };
    auto yToPx = [&](float db) { return dbToPxY(db); };

    beginPath();
    rect(x, y, w, h);
    fillColor(Color(18, 19, 23));
    fill();
    strokeColor(Color(48, 51, 58));
    strokeWidth(1.0f);
    stroke();

    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    for (float db = kGraphMinDb; db <= kGraphMaxDb + 0.01f; db += 12.0f)
    {
        const float px = xToPx(db);
        beginPath();
        moveTo(px, y);
        lineTo(px, y + h);
        strokeColor(Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.0f", db);
        text(px, y + h + 1.0f, buf, nullptr);

        const float py = yToPx(db);
        beginPath();
        moveTo(x, py);
        lineTo(x + w, py);
        strokeColor(Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        text(x - 3.0f, py, buf, nullptr);
    }

    // Unity (no compression) reference line, dashed.
    {
        constexpr float dash = 4.0f, gapLen = 4.0f;
        float px0 = xToPx(kGraphMinDb), py0 = yToPx(kGraphMinDb);
        const float px1 = xToPx(kGraphMaxDb), py1 = yToPx(kGraphMaxDb);
        const float len = std::hypot(px1 - px0, py1 - py0);
        const float dx = (px1 - px0) / len, dy = (py1 - py0) / len;
        float pos = 0.0f;
        strokeColor(Color(90, 94, 100));
        strokeWidth(1.0f);
        while (pos < len)
        {
            const float segEnd = std::min(len, pos + dash);
            beginPath();
            moveTo(px0 + dx * pos, py0 + dy * pos);
            lineTo(px0 + dx * segEnd, py0 + dy * segEnd);
            stroke();
            pos = segEnd + gapLen;
        }
    }

    // Threshold marker, dashed vertical (draggable, see hitHandle()/onMotion()).
    {
        const float tx = xToPx(thresholdDb_);
        constexpr float dash = 3.0f, gapLen = 3.0f;
        float py = y;
        strokeColor(Color(180, 150, 60));
        strokeWidth(1.2f);
        while (py < y + h)
        {
            const float segEnd = std::min(y + h, py + dash);
            beginPath();
            moveTo(tx, py);
            lineTo(tx, segEnd);
            stroke();
            py = segEnd + gapLen;
        }
    }

    // Ceiling: a horizontal dashed line at the output level the true output
    // sample can never cross -- unlike Threshold, this is an output-axis-
    // only value (see file header comment), so it's a full-width line, not
    // a point on the curve.
    {
        const float cy = yToPx(ceilingDb_);
        constexpr float dash = 3.0f, gapLen = 3.0f;
        float px = x;
        const bool active = (draggingHandle_ == Handle::Ceiling) ||
                             (draggingHandle_ == Handle::None && hoverHandle_ == Handle::Ceiling);
        strokeColor(active ? Color(235, 130, 110) : Color(180, 90, 75));
        strokeWidth(active ? 2.0f : 1.4f);
        while (px < x + w)
        {
            const float segEnd = std::min(x + w, px + dash);
            beginPath();
            moveTo(px, cy);
            lineTo(segEnd, cy);
            stroke();
            px = segEnd + gapLen;
        }
        fillColor(active ? Color(235, 130, 110) : Color(180, 90, 75));
        fontSize(9.5f);
        textAlign(ALIGN_LEFT | ALIGN_BOTTOM);
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Ceiling %+.1fdB", ceilingDb_);
        text(x + 4.0f, cy - 2.0f, buf, nullptr);
    }

    // The transfer curve itself -- Knee is the real, current parameter
    // value (see setCurveParams()); Ratio stays fixed (kFixedRatio), see
    // file header comment for why.
    beginPath();
    constexpr int kSteps = 160;
    for (int i = 0; i <= kSteps; ++i)
    {
        const float inDb = kGraphMinDb + (kGraphMaxDb - kGraphMinDb) * float(i) / float(kSteps);
        const float outDbRaw = inDb + float(sscomp::computeGainReductionDb(inDb, thresholdDb_, kFixedRatio, 1.0f, kneeDb_));
        const float outDb = std::min(outDbRaw, ceilingDb_); // the ceiling clamp the DSP itself applies, drawn on the curve too
        const float px = xToPx(inDb), py = yToPx(outDb);
        if (i == 0) moveTo(px, py); else lineTo(px, py);
    }
    strokeColor(Color(120, 210, 170));
    strokeWidth(2.2f);
    stroke();

    // Live marker -- current instantaneous position on the curve.
    if (detectorLevelDb_ > kGraphMinDb + 0.5f)
    {
        const float px = xToPx(detectorLevelDb_);
        const float py = yToPx(std::min(detectorLevelDb_ + gainReductionDb_, ceilingDb_));
        beginPath();
        circle(px, py, 5.0f);
        fillColor(Color(235, 195, 60).withAlpha(60.0f / 255.0f));
        fill();
        beginPath();
        circle(px, py, 3.0f);
        fillColor(Color(235, 195, 60));
        fill();
    }

    // Threshold handle -- Ceiling has its own on the line above.
    {
        const float px = xToPx(thresholdDb_);
        const float py = yToPx(std::min(thresholdDb_ + float(sscomp::computeGainReductionDb(thresholdDb_, thresholdDb_, kFixedRatio, 1.0f, kneeDb_)), ceilingDb_));
        const bool active = (draggingHandle_ == Handle::Threshold) ||
                             (draggingHandle_ == Handle::None && hoverHandle_ == Handle::Threshold);
        beginPath();
        circle(px, py, active ? 7.0f : 5.5f);
        fillColor(Color(20, 21, 25).withAlpha(0.7f));
        fill();
        strokeColor(active ? Color(235, 238, 242) : Color(170, 176, 186));
        strokeWidth(active ? 2.0f : 1.4f);
        stroke();
    }
}

void LimiterGraphWidget::drawGrMeterColumn(float x, float y, float w, float h)
{
    beginPath();
    rect(x, y, w, h);
    fillColor(Color(16, 17, 20));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    auto yOf = [&](float db) {
        const float frac = dbToAxis(std::max(kGrMeterMinDb, std::min(kGrMeterMaxDb, db)), kGrMeterMinDb, kGrMeterMaxDb);
        return y + (1.0f - frac) * h; // 0dB at top, kGrMeterMinDb at bottom
    };
    const float topY = yOf(0.0f);
    const float grY = yOf(gainReductionDb_);

    if (gainReductionDb_ < -0.01f)
    {
        scissor(x, topY, w, grY - topY);
        beginPath();
        rect(x, y, w, h);
        Paint grad = linearGradient(0, y, 0, y + h, Color(230, 120, 70), Color(235, 195, 60));
        fillPaint(grad);
        fill();
        resetScissor();
    }

    // Peak-hold marker -- "the peak that is recovering", see setLiveLevels().
    if (grHoldDb_ < -0.01f)
    {
        const float holdY = yOf(grHoldDb_);
        beginPath();
        moveTo(x, holdY);
        lineTo(x + w, holdY);
        strokeColor(Color(235, 238, 242));
        strokeWidth(1.5f);
        stroke();
    }

    fillColor(Color(140, 144, 152));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.5f);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(x + w * 0.5f, y + h + 2.0f, "GR", nullptr);

    // Live numeric value, overlaid on the column (same "chip" convention as
    // CompressorGraphWidget's own).
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.1f", gainReductionDb_);
    const float chipY = y + 3.0f, chipH = 14.0f;
    beginPath();
    roundedRect(x + 1.0f, chipY, w - 2.0f, chipH, 2.0f);
    fillColor(Color(16, 17, 20, 0.75f));
    fill();
    fillColor(gainReductionDb_ < -0.05f ? Color(235, 195, 60) : Color(170, 174, 182));
    fontSize(10.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(x + w * 0.5f, chipY + chipH * 0.5f + 0.5f, buf, nullptr);
}

void LimiterGraphWidget::drawGrScale(float x, float y, float h)
{
    fillColor(Color(150, 154, 162));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(8.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    for (float db = kGrMeterMinDb; db <= kGrMeterMaxDb + 0.01f; db += 6.0f)
    {
        const float frac = dbToAxis(db, kGrMeterMinDb, kGrMeterMaxDb);
        const float py = y + (1.0f - frac) * h;
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%+.0f", db);
        text(x, py, buf, nullptr);
    }
}

void LimiterGraphWidget::drawGrHistory(float x, float y, float w, float h)
{
    beginPath();
    rect(x, y, w, h);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    fillColor(Color(120, 124, 132));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x + 3.0f, y + 1.0f, "Gain reduction (history)", nullptr);

    beginPath();
    for (int i = 0; i < kHistoryLen; ++i)
    {
        const int idx = (historyPos_ + i) % kHistoryLen;
        const float grDb = history_[idx];
        const float px = x + w * float(i) / float(kHistoryLen - 1);
        const float frac = dbToAxis(std::max(kGrMeterMinDb, grDb), kGrMeterMinDb, 0.0f);
        const float py = y + h - frac * (h - 2.0f) - 1.0f;
        if (i == 0) moveTo(px, py); else lineTo(px, py);
    }
    strokeColor(Color(235, 195, 60));
    strokeWidth(1.5f);
    stroke();
}

void LimiterGraphWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(20, 21, 25));
    fill();

    constexpr float pad = 8.0f;
    constexpr float grColW = 34.0f;
    constexpr float grScaleW = 20.0f;
    constexpr float histH = 46.0f;
    constexpr float labelPad = 14.0f;

    curveX_ = pad + labelPad;
    curveY_ = pad;
    curveW_ = w - curveX_ - pad - grColW - grScaleW - 8.0f;
    curveH_ = h - curveY_ - pad - histH - labelPad - 6.0f;
    if (curveW_ <= 10.0f || curveH_ <= 10.0f) return;

    drawTransferCurve(curveX_, curveY_, curveW_, curveH_);

    const float grX = curveX_ + curveW_ + 6.0f;
    drawGrMeterColumn(grX, curveY_, grColW, curveH_);
    drawGrScale(grX + grColW + 3.0f, curveY_, curveH_);

    drawGrHistory(curveX_, curveY_ + curveH_ + labelPad + 6.0f, curveW_ + grColW + grScaleW + 8.0f, histH);
}

LimiterGraphWidget::Handle LimiterGraphWidget::hitHandle(float px, float py) const
{
    const float thx = dbToPxX(thresholdDb_);
    const float thy = dbToPxY(std::min(thresholdDb_ + float(sscomp::computeGainReductionDb(thresholdDb_, thresholdDb_, kFixedRatio, 1.0f, kneeDb_)), ceilingDb_));
    if (std::hypot(px - thx, py - thy) <= kHandleHitRadiusPx) return Handle::Threshold;

    const float cy = dbToPxY(ceilingDb_);
    if (std::fabs(py - cy) <= kHandleHitRadiusPx * 0.6f && px >= curveX_ && px <= curveX_ + curveW_) return Handle::Ceiling;

    return Handle::None;
}

bool LimiterGraphWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const float x = float(ev.pos.getX()), y = float(ev.pos.getY());

    if (ev.press)
    {
        const Handle h = hitHandle(x, y);
        if (h == Handle::None) return false;
        draggingHandle_ = h;
        repaint();
        return true;
    }

    if (draggingHandle_ != Handle::None)
    {
        draggingHandle_ = Handle::None;
        repaint();
        return true;
    }
    return false;
}

bool LimiterGraphWidget::onMotion(const MotionEvent& ev)
{
    const float x = float(ev.pos.getX()), y = float(ev.pos.getY());

    if (draggingHandle_ == Handle::Threshold)
    {
        const float newThreshold = std::max(thresholdMinDb_, std::min(thresholdMaxDb_, pxToDbX(x)));
        thresholdDb_ = newThreshold;
        if (onThresholdChanged_) onThresholdChanged_(newThreshold);
        repaint();
        return true;
    }
    if (draggingHandle_ == Handle::Ceiling)
    {
        const float newCeiling = std::max(ceilingMinDb_, std::min(ceilingMaxDb_, pxToDbY(y)));
        ceilingDb_ = newCeiling;
        if (onCeilingChanged_) onCeilingChanged_(newCeiling);
        repaint();
        return true;
    }

    const Handle h = hitHandle(x, y);
    if (h != hoverHandle_) { hoverHandle_ = h; repaint(); }
    return false;
}

END_NAMESPACE_DGL
