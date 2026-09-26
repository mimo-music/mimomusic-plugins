// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "CompressorGraphWidget.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

CompressorGraphWidget::CompressorGraphWidget(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
    std::fill(std::begin(history_), std::end(history_), 0.0f);
}

void CompressorGraphWidget::setCurveParams(float thresholdDb, float ratio, bool ratioUpMode, float kneeDb)
{
    thresholdDb_ = thresholdDb; ratio_ = ratio; ratioUpMode_ = ratioUpMode; kneeDb_ = kneeDb;
    repaint();
}

void CompressorGraphWidget::updateHold(float& holdDb, std::chrono::steady_clock::time_point& lastUpdate,
                                        std::chrono::steady_clock::time_point& peakAt, float grDb)
{
    const auto now = std::chrono::steady_clock::now();
    const float dtSec = std::chrono::duration<float>(now - lastUpdate).count();
    lastUpdate = now;

    // A conventional peak-hold ballistic, generalized to two directions:
    // snap instantly to a fresh excursion that's *more extreme* than
    // whatever's currently held (a deeper cut or a bigger boost, whichever
    // this is), otherwise sit still at that peak for kGrHoldTimeSec before
    // letting the held magnitude decay back toward 0dB -- a recovering
    // peak, now given a real hold plateau first (so the maximum reduction
    // stays readable a little longer) rather than starting to slide back the instant a louder excursion passes -- without crossing
    // past 0 or flipping sign on the way there.
    constexpr float kGrHoldTimeSec = 1.5f;
    constexpr float kRecoverDbPerSec = 12.0f;
    if (std::fabs(grDb) >= std::fabs(holdDb))
    {
        holdDb = grDb;
        peakAt = now;
    }
    else if (std::chrono::duration<float>(now - peakAt).count() >= kGrHoldTimeSec)
    {
        const float mag = std::fabs(holdDb) - kRecoverDbPerSec * dtSec;
        holdDb = mag > 0.0f ? std::copysign(mag, holdDb) : 0.0f;
    }
}

// See MultibandGraphWidget::setBandGainDb()'s own comment in the sibling
// ../statespace project for the full story: a real
// report ("the numbers are changing so fast, that its difficult to read")
// against that plugin's own live gain-reduction number applies identically
// here -- setLiveLevels()/setSecondaryLiveLevels() can be called as often
// as the host delivers DPF output-Parameter updates (up to once per audio
// block), far faster than a printed digit can actually be read. A one-pole
// filter against wall-clock time, same technique and time constant, keeps
// the printed number (and the meter fill it drives) legible while still
// tracking real changes within a fraction of a second.
void CompressorGraphWidget::smoothLiveValue(float& smoothed, std::chrono::steady_clock::time_point& lastUpdate, bool& init, float raw)
{
    if (!init)
    {
        smoothed = raw; // snap on the very first reading, nothing to smooth from yet
        init = true;
        lastUpdate = std::chrono::steady_clock::now();
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const float dtSec = std::chrono::duration<float>(now - lastUpdate).count();
    lastUpdate = now;

    constexpr float kTauSec = 0.22f; // same value as MultibandGraphWidget::setBandGainDb()
    const float coeff = dtSec > 0.0f ? std::exp(-dtSec / kTauSec) : 1.0f;
    smoothed = coeff * smoothed + (1.0f - coeff) * raw;
}

void CompressorGraphWidget::setLiveLevels(float detectorLevelDb, float gainReductionDb)
{
    detectorLevelDb_ = detectorLevelDb;
    smoothLiveValue(gainReductionDb_, grSmoothUpdated_, grSmoothInit_, gainReductionDb);
    updateHold(grHoldDb_[0], grHoldUpdated_[0], grHoldPeakAt_[0], gainReductionDb);
    history_[historyPos_] = gainReductionDb;
    historyPos_ = (historyPos_ + 1) % kHistoryLen;
    repaint();
}

void CompressorGraphWidget::setSecondaryLiveLevels(bool show, float detectorLevelDb, float gainReductionDb)
{
    secondaryShown_ = show;
    detectorLevelDbB_ = detectorLevelDb;
    smoothLiveValue(gainReductionDbB_, grSmoothUpdatedB_, grSmoothInitB_, gainReductionDb);
    updateHold(grHoldDb_[1], grHoldUpdated_[1], grHoldPeakAt_[1], gainReductionDb);
}

namespace {
inline float dbToAxis(float db, float minDb, float maxDb)
{
    return std::max(0.0f, std::min(1.0f, (db - minDb) / (maxDb - minDb)));
}

}

float CompressorGraphWidget::dbToPxX(float db) const { return curveX_ + dbToAxis(db, kGraphMinDb, kGraphMaxDb) * curveW_; }
float CompressorGraphWidget::dbToPxY(float db) const { return curveY_ + curveH_ - dbToAxis(db, kGraphMinDb, kGraphMaxDb) * curveH_; }
float CompressorGraphWidget::pxToDbX(float px) const
{
    if (curveW_ <= 0.0f) return kGraphMinDb;
    const float t = std::max(0.0f, std::min(1.0f, (px - curveX_) / curveW_));
    return kGraphMinDb + t * (kGraphMaxDb - kGraphMinDb);
}
float CompressorGraphWidget::pxToDbY(float py) const
{
    if (curveH_ <= 0.0f) return kGraphMinDb;
    const float t = std::max(0.0f, std::min(1.0f, (curveY_ + curveH_ - py) / curveH_));
    return kGraphMinDb + t * (kGraphMaxDb - kGraphMinDb);
}

void CompressorGraphWidget::drawTransferCurve(float x, float y, float w, float h)
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

    // Gridlines every 12dB, labeled on both axes.
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

    // Knee region shading -- a soft band around the threshold showing
    // exactly where the quadratic blend (rather than a hard corner)
    // applies.
    const float kneeHalf = std::max(0.0f, kneeDb_) * 0.5f;
    if (kneeHalf > 0.05f)
    {
        const float kx0 = xToPx(thresholdDb_ - kneeHalf);
        const float kx1 = xToPx(thresholdDb_ + kneeHalf);
        beginPath();
        rect(kx0, y, kx1 - kx0, h);
        fillColor(Color(90, 110, 140, 40.0f / 255.0f)); // Color's alpha is always [0..1], unlike its int r/g/b range
        fill();
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

    // Threshold marker, dashed vertical.
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

    // The actual transfer curve -- exactly one side ever actually
    // compresses now (see ratioUpMode_'s own comment): below the
    // threshold it rises above the unity line while compressing Up
    // (lifting quiet material), above it falls below while compressing
    // Down (as always) -- computeGainReductionDb() itself is unchanged,
    // effectiveRatioDown()/effectiveRatioUp() just resolve the single
    // ratio_ + direction into the (ratioDown, ratioUp) pair it takes,
    // always leaving the inactive side at 1.0 (no effect).
    beginPath();
    constexpr int kSteps = 160;
    const float ratioDown = effectiveRatioDown(), ratioUp = effectiveRatioUp();
    for (int i = 0; i <= kSteps; ++i)
    {
        const float inDb = kGraphMinDb + (kGraphMaxDb - kGraphMinDb) * float(i) / float(kSteps);
        const float outDb = inDb + float(sscomp::computeGainReductionDb(inDb, thresholdDb_, ratioDown, ratioUp, kneeDb_));
        const float px = xToPx(inDb), py = yToPx(outDb);
        if (i == 0) moveTo(px, py); else lineTo(px, py);
    }
    strokeColor(Color(120, 210, 170));
    strokeWidth(2.2f);
    stroke();

    // Live marker(s) -- current instantaneous position on the curve.
    auto drawMarker = [&](float levelDb, float grDb, Color c)
    {
        if (levelDb <= kGraphMinDb + 0.5f) return; // effectively silent, nothing meaningful to show
        const float px = xToPx(levelDb);
        const float py = yToPx(levelDb + grDb);
        beginPath();
        circle(px, py, 5.0f);
        fillColor(c.withAlpha(60.0f / 255.0f));
        fill();
        beginPath();
        circle(px, py, 3.0f);
        fillColor(c);
        fill();
    };
    if (secondaryShown_) drawMarker(detectorLevelDbB_, gainReductionDbB_, Color(160, 190, 235));
    drawMarker(detectorLevelDb_, gainReductionDb_, Color(235, 195, 60));

    // Draggable handles: Threshold (sits exactly on the curve at
    // x=threshold), and Ratio -- a single handle now, at whichever
    // reference x the active direction uses (see activeRatioHandleRefDb()),
    // not two simultaneously-live ones. Drawn as a ring that brightens on
    // hover/drag, distinct from the live marker dot's amber so the two are
    // never confused.
    auto drawHandle = [&](float levelDb, Handle which)
    {
        const float px = xToPx(levelDb);
        const float py = yToPx(levelDb + float(sscomp::computeGainReductionDb(levelDb, thresholdDb_, ratioDown, ratioUp, kneeDb_)));
        const bool active = (draggingHandle_ == which) || (draggingHandle_ == Handle::None && hoverHandle_ == which);
        beginPath();
        circle(px, py, active ? 7.0f : 5.5f);
        fillColor(Color(20, 21, 25).withAlpha(0.7f));
        fill();
        strokeColor(active ? Color(235, 238, 242) : Color(170, 176, 186));
        strokeWidth(active ? 2.0f : 1.4f);
        stroke();
    };
    drawHandle(thresholdDb_, Handle::Threshold);
    drawHandle(activeRatioHandleRefDb(), Handle::Ratio);
}

// Bipolar since upward compression can report a *boost*: a 0dB reference
// line sits wherever kGrMeterMaxDb's share of the total range places it
// (not necessarily the column's midpoint, since the two ends -24dB/+12dB
// aren't symmetric -- a cut is both more common and more dramatic in
// practice than a boost, so it gets more of the column's height), the fill
// grows *downward* from it for a cut (amber/red, the original coloring)
// and *upward* from it for a boost (a cool blue/green, so the two
// directions read apart at a glance), and a peak-hold marker (see
// updateHold()) shows the most extreme recent excursion in whichever
// direction it actually happened, slowly recovering back toward the 0dB
// line -- "a hold bar to see the peaks that is recovering".
void CompressorGraphWidget::drawGrMeterColumn(float x, float y, float w, float h, float grDb, float holdDb, float makeupDb, const char* label)
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
        return y + (1.0f - frac) * h; // 0 at bottom (kGrMeterMinDb), 1 at top (kGrMeterMaxDb)
    };
    const float zeroY = yOf(0.0f);
    const float grY = yOf(grDb);
    // With a makeup gain applied, the bar shows the sum (reduced, or even
    // amplified) -- dB values add exactly where their linear gains multiply (see
    // PluginCompressorStereo::run()'s own yMid/ySide *= makeupALin/
    // makeupBLin), so grDb + makeupDb is the actual net effect on level
    // the compressor-plus-makeup chain is having right now.
    const float netDb = grDb + makeupDb;
    const float netY = yOf(netDb);

    // grY/netY and zeroY were swapped here relative to nvgScissor()'s
    // (x,y,w,h) contract -- for a cut, the bar's own top sits *below*
    // zeroY (y grows downward), so the old "scissor(x, barY, w, zeroY -
    // barY)" passed a *negative* height. nvgScissor() clamps a negative
    // w/h to 0 (nanovg.c: "w = nvg__maxf(0.0f, w)"), so the scissor rect
    // silently became zero-area and the gradient fill underneath it drew
    // nothing at all. Using min()/fabs() instead of relying on which
    // operand happens to be larger fixes both branches and can't silently
    // regress the same way again.
    auto fillBar = [&](float barY, Color lo, Color hi, bool cut) {
        scissor(x, std::min(barY, zeroY), w, std::fabs(zeroY - barY));
        beginPath();
        rect(x, y, w, h);
        Paint grad = cut ? linearGradient(0, y, 0, zeroY, lo, hi)
                          : linearGradient(0, zeroY, 0, y + h, lo, hi);
        fillPaint(grad);
        fill();
        resetScissor();
    };

    // Background: the *original*, makeup-less gain reduction -- dimmer
    // versions of the same cut/boost colors below (not a third color),
    // so it reads as "behind" the net bar, not a separate state. Drawn
    // first, so the net bar on top
    // covers it wherever they overlap; where they don't -- e.g. a deep
    // cut clawed back by a big positive Makeup Gain, landing net on the
    // *other* side of 0dB -- both stay visible simultaneously, one on
    // each side of the zero line.
    if (grDb < -0.01f)
        fillBar(grY, Color(230, 120, 70, 0.35f), Color(235, 195, 60, 0.35f), true);
    else if (grDb > 0.01f)
        fillBar(grY, Color(90, 190, 150, 0.35f), Color(70, 150, 220, 0.35f), false);

    // Foreground: the net (sum) effect -- full-strength colors, same as
    // this bar always used before Makeup Gain existed.
    if (netDb < -0.01f)
        fillBar(netY, Color(230, 120, 70), Color(235, 195, 60), true);
    else if (netDb > 0.01f)
        fillBar(netY, Color(90, 190, 150), Color(70, 150, 220), false);

    // 0dB reference line -- brighter than the column's border, since this
    // is the meter's own meaningful zero, not just a boundary.
    beginPath();
    moveTo(x, zeroY);
    lineTo(x + w, zeroY);
    strokeColor(Color(90, 94, 100));
    strokeWidth(1.0f);
    stroke();

    // Peak-hold marker.
    if (std::fabs(holdDb) > 0.01f)
    {
        const float holdY = yOf(holdDb);
        beginPath();
        moveTo(x, holdY);
        lineTo(x + w, holdY);
        strokeColor(Color(235, 238, 242));
        strokeWidth(1.5f);
        stroke();
    }

    // Makeup Gain marker, like the Multiband Compressor's. A solid line at
    // this column's own *configured* Makeup Gain; ported from
    // ../../statespace's MultibandGraphWidget::
    // drawBandGainMeter()'s identical marker/color. Skipped at exactly
    // 0dB (the default) so an untouched column isn't drawing a line
    // squarely on top of the 0dB reference line above for no reason.
    if (std::fabs(makeupDb) > 0.01f)
    {
        const float makeupY = yOf(makeupDb);
        beginPath();
        moveTo(x, makeupY);
        lineTo(x + w, makeupY);
        strokeColor(Color(160, 170, 230, 0.9f));
        strokeWidth(1.5f);
        stroke();
    }

    fillColor(Color(140, 144, 152));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.5f);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(x + w * 0.5f, y + h + 2.0f, label, nullptr);
}

// The GR meter's own dB scale, drawn once (shared by both columns when a
// secondary voice is shown -- one shared axis rather than one per column,
// same "avoid redundant clutter" reasoning as every other shared-scale
// case in this project's sibling ../statespace).
void CompressorGraphWidget::drawGrScale(float x, float y, float h)
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

void CompressorGraphWidget::drawGrHistory(float x, float y, float w, float h)
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

    // Oldest sample at the left, newest at the right (scrolls left as new
    // samples arrive, since historyPos_ marks the next slot to overwrite,
    // i.e. the oldest one still displayed).
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

void CompressorGraphWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(20, 21, 25));
    fill();

    constexpr float pad = 8.0f;
    constexpr float grColW = 34.0f;
    constexpr float grScaleW = 20.0f; // room for the meter's own dB scale, see drawGrScale()
    constexpr float histH = 46.0f;
    constexpr float labelPad = 14.0f; // room for the axis tick labels below/left of the curve

    const float grColsW = grColW * (secondaryShown_ ? 2.0f : 1.0f) + (secondaryShown_ ? 4.0f : 0.0f);

    curveX_ = pad + labelPad;
    curveY_ = pad;
    curveW_ = w - curveX_ - pad - grColsW - grScaleW - 8.0f;
    curveH_ = h - curveY_ - pad - histH - labelPad - 6.0f;
    if (curveW_ <= 10.0f || curveH_ <= 10.0f) return;

    drawTransferCurve(curveX_, curveY_, curveW_, curveH_);

    float grX = curveX_ + curveW_ + 6.0f;
    drawGrMeterColumn(grX, curveY_, grColW, curveH_, gainReductionDb_, grHoldDb_[0], makeupGainDb_, labelA_.c_str());
    if (secondaryShown_)
    {
        grX += grColW + 4.0f;
        drawGrMeterColumn(grX, curveY_, grColW, curveH_, gainReductionDbB_, grHoldDb_[1], makeupGainDbB_, labelB_.c_str());
    }
    drawGrScale(grX + grColW + 3.0f, curveY_, curveH_);

    // Live gain-reduction/boost value(s), right next to (overlaid on top
    // of) the meter column(s) they describe (moved here from the curve's
    // own top-left corner -- see this file's header comment) -- a small
    // dark chip keeps the text legible over whatever the column's own
    // fill happens to be doing underneath, signed and colored per
    // direction so "-6.4" (cutting) and "+3.1" (boosting) read apart even
    // without re-reading the sign.
    auto drawGrValue = [&](float colX, float grDb) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%+.1f", grDb);
        const float chipY = curveY_ + 3.0f, chipH = 14.0f;
        beginPath();
        roundedRect(colX + 1.0f, chipY, grColW - 2.0f, chipH, 2.0f);
        fillColor(Color(16, 17, 20, 0.75f));
        fill();
        fillColor(grDb < -0.05f ? Color(235, 195, 60) : grDb > 0.05f ? Color(120, 190, 235) : Color(170, 174, 182));
        fontSize(10.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(colX + grColW * 0.5f, chipY + chipH * 0.5f + 0.5f, buf, nullptr);
    };
    // Net (gain reduction + Makeup Gain), matching drawGrMeterColumn()'s
    // own foreground bar -- printing the raw, makeup-less number here
    // while the bar next to it shows the net would read as a mismatch,
    // undermining the entire point of "so we can see what is happening
    // with the sound".
    drawGrValue(curveX_ + curveW_ + 6.0f, gainReductionDb_ + makeupGainDb_);
    if (secondaryShown_) drawGrValue(curveX_ + curveW_ + 6.0f + grColW + 4.0f, gainReductionDbB_ + makeupGainDbB_);

    drawGrHistory(curveX_, curveY_ + curveH_ + labelPad + 6.0f, curveW_ + grColsW + grScaleW + 8.0f, histH);
}

namespace {
inline float handleDist(float px, float py, float hx, float hy) { return std::hypot(px - hx, py - hy); }
}

CompressorGraphWidget::Handle CompressorGraphWidget::hitHandle(float px, float py) const
{
    const float ratioDown = effectiveRatioDown(), ratioUp = effectiveRatioUp();

    const float thx = dbToPxX(thresholdDb_);
    const float thy = dbToPxY(thresholdDb_ + float(sscomp::computeGainReductionDb(thresholdDb_, thresholdDb_, ratioDown, ratioUp, kneeDb_)));
    if (handleDist(px, py, thx, thy) <= kHandleHitRadiusPx) return Handle::Threshold;

    const float refDb = activeRatioHandleRefDb();
    const float rx = dbToPxX(refDb);
    const float ry = dbToPxY(refDb + float(sscomp::computeGainReductionDb(refDb, thresholdDb_, ratioDown, ratioUp, kneeDb_)));
    if (handleDist(px, py, rx, ry) <= kHandleHitRadiusPx) return Handle::Ratio;

    return Handle::None;
}

bool CompressorGraphWidget::onMouse(const MouseEvent& ev)
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

bool CompressorGraphWidget::onMotion(const MotionEvent& ev)
{
    const float x = float(ev.pos.getX()), y = float(ev.pos.getY());

    if (draggingHandle_ == Handle::Threshold)
    {
        const float newThreshold = std::max(thresholdMinDb_, std::min(thresholdMaxDb_, pxToDbX(x)));
        thresholdDb_ = newThreshold; // immediate local redraw; the owning UI's callback syncs the DPF parameter + other widgets
        if (onThresholdChanged_) onThresholdChanged_(newThreshold);
        repaint();
        return true;
    }
    if (draggingHandle_ == Handle::Ratio)
    {
        // Down and Up use mirrored math around the threshold (Up's
        // reference point sits *below* it, and "more extreme" runs the
        // opposite direction), same drag feel either way -- which formula
        // applies just depends on ratioUpMode_, since there's only ever
        // one active handle now.
        float newRatio;
        if (ratioUpMode_)
        {
            const float refUpDb = ratioUpHandleRefDb();
            const float desiredOutDb = pxToDbY(y);
            const float denom = std::max(0.05f, thresholdDb_ - desiredOutDb);
            newRatio = (thresholdDb_ - refUpDb) / denom;
        }
        else
        {
            const float refDb = ratioHandleRefDb();
            const float desiredOutDb = pxToDbY(y);
            const float denom = std::max(0.05f, desiredOutDb - thresholdDb_); // guard divide-by-~0 as the drag nears/crosses the threshold line
            newRatio = (refDb - thresholdDb_) / denom;
        }
        newRatio = std::max(1.0f, std::min(20.0f, newRatio));
        ratio_ = newRatio;
        if (onRatioChanged_) onRatioChanged_(newRatio);
        repaint();
        return true;
    }

    const Handle h = hitHandle(x, y);
    if (h != hoverHandle_) { hoverHandle_ = h; repaint(); }
    return false;
}

bool CompressorGraphWidget::onScroll(const ScrollEvent& ev)
{
    const float x = float(ev.pos.getX()), y = float(ev.pos.getY());
    if (x < curveX_ || x >= curveX_ + curveW_ || y < curveY_ || y >= curveY_ + curveH_) return false;

    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    // Alt+scroll = 10x finer -- see gui/ParamStripWidget.cpp's identical
    // comment for the full reasoning, applied project-wide.
    const float step = 0.5f / ((ev.mod & kModifierAlt) ? 10.0f : 1.0f);
    const float newKnee = std::max(0.0f, std::min(24.0f, kneeDb_ + float(dir) * step));
    kneeDb_ = newKnee;
    if (onKneeChanged_) onKneeChanged_(newKnee);
    repaint();
    return true;
}

END_NAMESPACE_DGL
