// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "LimiterGraphWidget.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"
#include "../plugin/dsp/LimiterDsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

LimiterGraphWidget::LimiterGraphWidget(NanoTopLevelWidget* parent)
    : EditorSubWidget(parent)
{
    loadSharedResources();
    std::fill(std::begin(history_), std::end(history_), 0.0f);
}

void LimiterGraphWidget::setCurveParams(const CurveParams& p)
{
    p_ = p;
    repaint();
}

void LimiterGraphWidget::setLiveLevels(float detectorLevelDb, float gainReductionDb, float clipDb)
{
    detectorLevelDb_ = detectorLevelDb;

    // Smoothed against wall-clock time so the number stays readable (same
    // time constant as CompressorGraphWidget's). The peak-hold below uses
    // the raw value.
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
        constexpr float kTauSec = 0.22f;
        const float coeff = smoothDtSec > 0.0f ? std::exp(-smoothDtSec / kTauSec) : 1.0f;
        gainReductionDb_ = coeff * gainReductionDb_ + (1.0f - coeff) * gainReductionDb;
    }

    // Peak-hold: snap to a deeper cut, hold kGrHoldTimeSec, then recover.
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

    // Clip indicator: same hold-then-recover shape, so a single clipped
    // transient stays visible long enough to be noticed.
    if (clipDb <= clipHoldDb_)
    {
        clipHoldDb_ = clipDb;
        clipHoldPeakAt_ = now;
    }
    else if (std::chrono::duration<float>(now - clipHoldPeakAt_).count() >= kGrHoldTimeSec)
    {
        clipHoldDb_ = std::min(0.0f, clipHoldDb_ + kRecoverDbPerSec * dtSec);
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

float LimiterGraphWidget::limitedDbFor(float inDb) const
{
    return float(double(inDb) + sscomp::computeGainReductionDb(inDb, p_.effectiveThresholdDb, p_.ratio, 1.0, p_.kneeDb));
}

float LimiterGraphWidget::outputDbFor(float inDb) const
{
    // Gain computer at the effective threshold, then the capped post gain,
    // then the clip stage -- PluginLimiter::processBlock()'s order.
    const double lin = sscomp::dbToLinear(double(limitedDbFor(inDb)) + p_.postGainDb);
    const double ceilingLin = sscomp::dbToLinear(p_.ceilingDb);
    const double out = p_.softClip ? sscomp::softClip(lin, sscomp::dbToLinear(p_.ceilingDb - p_.clipKneeDb), ceilingLin)
                                   : sscomp::hardClip(lin, ceilingLin);
    return float(sscomp::linearToDb(out));
}

namespace {
// Dashed straight line from (x0,y0) to (x1,y1) -- the caller sets the stroke.
template <class Ctx>
void dashedLine(Ctx& c, float x0, float y0, float x1, float y1, float dash, float gapLen)
{
    const float len = std::hypot(x1 - x0, y1 - y0);
    if (len <= 0.0f) return;
    const float dx = (x1 - x0) / len, dy = (y1 - y0) / len;
    for (float pos = 0.0f; pos < len; pos += dash + gapLen)
    {
        const float segEnd = std::min(len, pos + dash);
        c.beginPath();
        c.moveTo(x0 + dx * pos, y0 + dy * pos);
        c.lineTo(x0 + dx * segEnd, y0 + dy * segEnd);
        c.stroke();
    }
}
}

void LimiterGraphWidget::drawTransferCurve(float x, float y, float w, float h)
{
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
        const float px = dbToPxX(db);
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

        const float py = dbToPxY(db);
        beginPath();
        moveTo(x, py);
        lineTo(x + w, py);
        strokeColor(Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        text(x - 3.0f, py, buf, nullptr);
    }

    // Unity reference line.
    strokeColor(Color(90, 94, 100));
    strokeWidth(1.0f);
    dashedLine(*this, dbToPxX(kGraphMinDb), dbToPxY(kGraphMinDb), dbToPxX(kGraphMaxDb), dbToPxY(kGraphMaxDb), 4.0f, 4.0f);

    // Threshold marker (draggable via its handle on the curve).
    strokeColor(Color(180, 150, 60));
    strokeWidth(1.2f);
    dashedLine(*this, dbToPxX(p_.thresholdDb), y, dbToPxX(p_.thresholdDb), y + h, 3.0f, 3.0f);

    // Soft clip band: shaded between the knee start and the Ceiling, with
    // the knee start as a dotted, draggable line.
    if (p_.softClip)
    {
        const float cy = dbToPxY(p_.ceilingDb);
        const float ky = dbToPxY(p_.ceilingDb - p_.clipKneeDb);
        beginPath();
        rect(x, cy, w, ky - cy);
        fillColor(Color(180, 90, 75, 0.12f));
        fill();

        const bool active = (draggingHandle_ == Handle::ClipKnee) ||
                             (draggingHandle_ == Handle::None && hoverHandle_ == Handle::ClipKnee);
        strokeColor(active ? Color(235, 160, 140) : Color(150, 95, 85));
        strokeWidth(active ? 1.8f : 1.1f);
        dashedLine(*this, x, ky, x + w, ky, 1.5f, 3.0f);

        fillColor(active ? Color(235, 160, 140) : Color(150, 95, 85));
        fontSize(9.0f);
        textAlign(ALIGN_RIGHT | ALIGN_TOP);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "Soft clip from %+.1fdB", p_.ceilingDb - p_.clipKneeDb);
        text(x + w - 4.0f, ky + 2.0f, buf, nullptr);
    }

    // Ceiling.
    {
        const float cy = dbToPxY(p_.ceilingDb);
        const bool active = (draggingHandle_ == Handle::Ceiling) ||
                             (draggingHandle_ == Handle::None && hoverHandle_ == Handle::Ceiling);
        strokeColor(active ? Color(235, 130, 110) : Color(180, 90, 75));
        strokeWidth(active ? 2.0f : 1.4f);
        dashedLine(*this, x, cy, x + w, cy, 3.0f, 3.0f);
        fillColor(active ? Color(235, 130, 110) : Color(180, 90, 75));
        fontSize(9.5f);
        textAlign(ALIGN_LEFT | ALIGN_BOTTOM);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "Ceiling %+.1fdB (%s)", p_.ceilingDb, p_.softClip ? "soft" : "hard");
        text(x + 4.0f, cy - 2.0f, buf, nullptr);
    }

    constexpr int kSteps = 200;

    // With gain after the limiter: the limiter alone, faint, and an arrow
    // from it to the full curve, 18 dB below the Threshold (where both are
    // straight lines), labelled with that gain.
    if (std::fabs(p_.postGainDb) > 0.05f)
    {
        beginPath();
        for (int i = 0; i <= kSteps; ++i)
        {
            const float inDb = kGraphMinDb + (kGraphMaxDb - kGraphMinDb) * float(i) / float(kSteps);
            const float px = dbToPxX(inDb), py = dbToPxY(limitedDbFor(inDb));
            if (i == 0) moveTo(px, py); else lineTo(px, py);
        }
        strokeColor(Color(120, 210, 170, 0.35f));
        strokeWidth(1.4f);
        stroke();

        const float ax = std::max(kGraphMinDb + 4.0f, p_.thresholdDb - 18.0f);
        const float px = dbToPxX(ax);
        const float y0 = dbToPxY(limitedDbFor(ax)), y1 = dbToPxY(outputDbFor(ax));
        if (std::fabs(y1 - y0) > 8.0f)
        {
            const float dir = y1 < y0 ? -1.0f : 1.0f; // -1: arrow points up
            strokeColor(Color(120, 190, 130));
            strokeWidth(1.4f);
            beginPath();
            moveTo(px, y0);
            lineTo(px, y1 - dir * 1.0f);
            stroke();
            beginPath();
            moveTo(px - 4.0f, y1 - dir * 6.0f);
            lineTo(px, y1);
            lineTo(px + 4.0f, y1 - dir * 6.0f);
            stroke();
        }
        fillColor(Color(120, 190, 130));
        fontSize(9.5f);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%+.1fdB after limiter", p_.postGainDb);
        text(px + 7.0f, 0.5f * (y0 + y1), buf, nullptr);
    }

    // The transfer curve: the whole chain, see outputDbFor().
    beginPath();
    for (int i = 0; i <= kSteps; ++i)
    {
        const float inDb = kGraphMinDb + (kGraphMaxDb - kGraphMinDb) * float(i) / float(kSteps);
        const float px = dbToPxX(inDb), py = dbToPxY(outputDbFor(inDb));
        if (i == 0) moveTo(px, py); else lineTo(px, py);
    }
    strokeColor(Color(120, 210, 170));
    strokeWidth(2.2f);
    stroke();

    // Live marker.
    if (detectorLevelDb_ > kGraphMinDb + 0.5f)
    {
        const float liveLin = float(sscomp::dbToLinear(double(detectorLevelDb_ + gainReductionDb_) + p_.postGainDb));
        const double ceilingLin = sscomp::dbToLinear(p_.ceilingDb);
        const double clipped = p_.softClip ? sscomp::softClip(liveLin, sscomp::dbToLinear(p_.ceilingDb - p_.clipKneeDb), ceilingLin)
                                           : sscomp::hardClip(liveLin, ceilingLin);
        const float px = dbToPxX(detectorLevelDb_);
        const float py = dbToPxY(float(sscomp::linearToDb(clipped)));
        beginPath();
        circle(px, py, 5.0f);
        fillColor(Color(235, 195, 60).withAlpha(60.0f / 255.0f));
        fill();
        beginPath();
        circle(px, py, 3.0f);
        fillColor(Color(235, 195, 60));
        fill();
    }

    // Threshold handle.
    {
        const float px = dbToPxX(p_.thresholdDb);
        const float py = dbToPxY(outputDbFor(p_.thresholdDb));
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

    // Notes on the gain staging (top-left), only when something differs
    // from what the controls say.
    {
        fontSize(10.0f);
        textAlign(ALIGN_LEFT | ALIGN_TOP);
        float ty = y + 4.0f;
        char buf[128];
        if (p_.postGainDb < p_.requestedPostGainDb - 0.05f)
        {
            std::snprintf(buf, sizeof(buf), "Gain after limiter %+.1fdB (Boost/Output ask %+.1fdB, capped by Ceiling)",
                          p_.postGainDb, p_.requestedPostGainDb);
            fillColor(Color(120, 190, 130));
            text(x + 6.0f, ty, buf, nullptr);
            ty += 13.0f;
        }
        else if (std::fabs(p_.postGainDb) > 0.05f)
        {
            std::snprintf(buf, sizeof(buf), "Gain after limiter %+.1fdB", p_.postGainDb);
            fillColor(Color(140, 150, 160));
            text(x + 6.0f, ty, buf, nullptr);
            ty += 13.0f;
        }
        if (p_.effectiveThresholdDb < p_.thresholdDb - 0.05f)
        {
            std::snprintf(buf, sizeof(buf), "Limiting at %+.1fdB (Ceiling below Threshold)", p_.effectiveThresholdDb);
            fillColor(Color(220, 170, 90));
            text(x + 6.0f, ty, buf, nullptr);
        }
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
        return y + (1.0f - frac) * h; // 0dB at top
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

void LimiterGraphWidget::drawClipIndicator(float x, float y, float w, float h)
{
    // Lit while the clip stage is (or recently was) taking something off.
    const bool lit = clipHoldDb_ < -0.05f;
    beginPath();
    roundedRect(x, y, w, h, 2.0f);
    fillColor(lit ? Color(170, 70, 55) : Color(30, 32, 37));
    fill();
    strokeColor(Color(60, 63, 70));
    strokeWidth(1.0f);
    stroke();
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    fillColor(lit ? Color(250, 235, 230) : Color(110, 114, 122));
    char buf[24];
    if (lit) std::snprintf(buf, sizeof(buf), "CLIP %.1f", clipHoldDb_);
    else std::snprintf(buf, sizeof(buf), "CLIP");
    text(x + w * 0.5f, y + h * 0.5f + 0.5f, buf, nullptr);
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
    constexpr float grColW = 44.0f;
    constexpr float grScaleW = 20.0f;
    constexpr float histH = 46.0f;
    constexpr float labelPad = 14.0f;
    constexpr float clipH = 16.0f;

    curveX_ = pad + labelPad;
    curveY_ = pad;
    curveW_ = w - curveX_ - pad - grColW - grScaleW - 8.0f;
    curveH_ = h - curveY_ - pad - histH - labelPad - 6.0f;
    if (curveW_ <= 10.0f || curveH_ <= 10.0f) return;

    drawTransferCurve(curveX_, curveY_, curveW_, curveH_);

    // CLIP indicator on top of the GR column, the column below it.
    const float grX = curveX_ + curveW_ + 6.0f;
    drawClipIndicator(grX, curveY_, grColW, clipH);
    drawGrMeterColumn(grX, curveY_ + clipH + 4.0f, grColW, curveH_ - clipH - 4.0f);
    drawGrScale(grX + grColW + 3.0f, curveY_ + clipH + 4.0f, curveH_ - clipH - 4.0f);

    drawGrHistory(curveX_, curveY_ + curveH_ + labelPad + 6.0f, curveW_ + grColW + grScaleW + 8.0f, histH);
}

LimiterGraphWidget::Handle LimiterGraphWidget::hitHandle(float px, float py) const
{
    const float thx = dbToPxX(p_.thresholdDb);
    const float thy = dbToPxY(outputDbFor(p_.thresholdDb));
    if (std::hypot(px - thx, py - thy) <= kHandleHitRadiusPx) return Handle::Threshold;

    if (px < curveX_ || px > curveX_ + curveW_) return Handle::None;
    const float tol = kHandleHitRadiusPx * 0.6f;
    const float dCeil = std::fabs(py - dbToPxY(p_.ceilingDb));
    const float dKnee = p_.softClip ? std::fabs(py - dbToPxY(p_.ceilingDb - p_.clipKneeDb)) : 1e9f;
    // The two lines can sit a few pixels apart -- take the closer one.
    if (dKnee <= tol && dKnee < dCeil) return Handle::ClipKnee;
    if (dCeil <= tol) return Handle::Ceiling;
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
        const float v = std::max(thresholdMinDb_, std::min(thresholdMaxDb_, pxToDbX(x)));
        p_.thresholdDb = v;
        if (onThresholdChanged_) onThresholdChanged_(v);
        repaint();
        return true;
    }
    if (draggingHandle_ == Handle::Ceiling)
    {
        const float v = std::max(ceilingMinDb_, std::min(ceilingMaxDb_, pxToDbY(y)));
        p_.ceilingDb = v;
        if (onCeilingChanged_) onCeilingChanged_(v);
        repaint();
        return true;
    }
    if (draggingHandle_ == Handle::ClipKnee)
    {
        const float v = std::max(clipKneeMinDb_, std::min(clipKneeMaxDb_, p_.ceilingDb - pxToDbY(y)));
        p_.clipKneeDb = v;
        if (onClipKneeChanged_) onClipKneeChanged_(v);
        repaint();
        return true;
    }

    const Handle h = hitHandle(x, y);
    if (h != hoverHandle_) { hoverHandle_ = h; repaint(); }
    return false;
}

END_NAMESPACE_DGL
