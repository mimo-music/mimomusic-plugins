// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "LoudnessMeterWidget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

namespace {
float dbToFrac(float db, float minDb, float maxDb)
{
    const float t = (db - minDb) / (maxDb - minDb);
    return std::max(0.0f, std::min(1.0f, t));
}

std::string formatLufs(float v)
{
    char buf[24];
    if (v <= -119.0f) return "-inf";
    std::snprintf(buf, sizeof(buf), "%+.1f", double(v));
    return buf;
}

std::string formatLu(float v)
{
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%.1f", double(v));
    return buf;
}

// Panel layout constants, shared by onNanoDisplay() (which draws into
// them) and centerPanelBounds() (which onScroll()/onMouse() use to
// hit-test the exact same rectangle) -- kept in this one place so the two
// can never drift apart.
constexpr float kPad = 8.0f;
constexpr float kBarW = 78.0f;
constexpr float kPanelW = 190.0f;
constexpr float kGap = 10.0f;

// Picks the finest of `candidates` (ascending order) whose tick count
// over `span` doesn't exceed `maxTicks` -- shared by drawDistribution()'s
// own LUFS grid and drawHistoryGraph()'s own seconds grid, so "more ticks
// when zoomed in" is one algorithm, not two independently
// hand-tuned copies. Falls back to the coarsest candidate if even that
// would exceed maxTicks (never happens for either caller today, both
// pick their own bounds so the coarsest candidate always already fits at
// their own maximum span, but a coarsest-available answer is still a
// more sensible fallback than silently returning zero/asserting).
float pickGridStep(float span, const float* candidates, int candidateCount, int maxTicks)
{
    for (int i = 0; i < candidateCount; ++i)
        if (span / candidates[i] <= float(maxTicks)) return candidates[i];
    return candidates[candidateCount - 1];
}
} // namespace

LoudnessMeterWidget::LoudnessMeterWidget(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
    // Sized once, up front, to the full 6-hour capacity -- same zero-init
    // content a fixed C array's own `= {}` gave before, just heap-backed
    // now (see historyMomentary_'s own comment, LoudnessMeterWidget.hpp).
    historyMomentary_.resize(size_t(kHistoryCapacity), 0.0f);
    historyShortTerm_.resize(size_t(kHistoryCapacity), 0.0f);
}

void LoudnessMeterWidget::setReadings(float momentaryLufs, float shortTermLufs, float integratedLufs,
                                       float loudnessRangeLu, float truePeakL, float truePeakR,
                                       float truePeakMax, float plrLu, float psrLu)
{
    momentaryLufs_ = momentaryLufs;
    shortTermLufs_ = shortTermLufs;
    integratedLufs_ = integratedLufs;
    loudnessRangeLu_ = loudnessRangeLu;
    truePeakL_ = truePeakL;
    truePeakR_ = truePeakR;
    truePeakMax_ = truePeakMax;
    plrLu_ = plrLu;
    psrLu_ = psrLu;
    repaint();
}

void LoudnessMeterWidget::setTarget(float targetLufs, bool enabled)
{
    targetLufs_ = targetLufs;
    targetEnabled_ = enabled;
    repaint();
}

void LoudnessMeterWidget::pushHistorySample(float momentaryLufs, float shortTermLufs)
{
    historyMomentary_[size_t(historyWritePos_)] = momentaryLufs;
    historyShortTerm_[size_t(historyWritePos_)] = shortTermLufs;
    historyWritePos_ = (historyWritePos_ + 1) % kHistoryCapacity;
    if (historyFilled_ < kHistoryCapacity) ++historyFilled_;
    // The very first push defines this widget's own elapsed-time axis
    // origin (0s) -- every push after that just advances the "now" edge
    // by one nominal sample period. See historyElapsedNewest_'s own
    // comment (LoudnessMeterWidget.hpp) for why this, not "seconds ago"
    // or a DAW timestamp.
    if (historyHasData_) historyElapsedNewest_ += double(kHistorySamplePeriodSec);
    historyHasData_ = true;
    repaint();
}

void LoudnessMeterWidget::setDistributionShortTerm(const float* counts, int count)
{
    distributionBinCount_ = std::max(0, std::min(kMaxDistributionBins, count));
    for (int i = 0; i < distributionBinCount_; ++i) distributionCountsShortTerm_[i] = counts[i];
    repaint();
}

void LoudnessMeterWidget::setDistributionMomentary(const float* counts, int count)
{
    distributionBinCount_ = std::max(0, std::min(kMaxDistributionBins, count));
    for (int i = 0; i < distributionBinCount_; ++i) distributionCountsMomentary_[i] = counts[i];
    repaint();
}

void LoudnessMeterWidget::setShowDistribution(bool show)
{
    showDistribution_ = show;
    repaint();
}

void LoudnessMeterWidget::setResetFlash(float intensity)
{
    resetFlash_ = std::max(0.0f, std::min(1.0f, intensity));
    repaint();
}

void LoudnessMeterWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(20, 21, 25));
    fill();

    drawBarMeter(kPad, kPad, kBarW, h - 2.0f * kPad);

    float centerX, centerY, centerW, centerH;
    centerPanelBounds(centerX, centerY, centerW, centerH);
    if (showDistribution_)
        // One shared plot, both curves overlapping on it, rather than the
        // previous two stacked boxes (which still read as "two axes" even
        // after being put on one shared count scale). See
        // drawDistribution()'s own comment for the layering.
        drawDistribution(centerX, centerY, centerW, centerH);
    else
        drawHistoryGraph(centerX, centerY, centerW, centerH);

    drawReadoutPanel(w - kPad - kPanelW, kPad, kPanelW, h - 2.0f * kPad);
}

void LoudnessMeterWidget::centerPanelBounds(float& x, float& y, float& w, float& h) const
{
    const float totalW = float(getWidth());
    const float totalH = float(getHeight());
    x = kPad + kBarW + kGap;
    y = kPad;
    w = std::max(10.0f, totalW - x - kPanelW - kGap - kPad);
    h = totalH - 2.0f * kPad;
}

// A generic Catmull-Rom-to-Bezier interpolating spline through the given
// already-screen-mapped points -- the same construction
// GoniometerWidget::drawTrailAsLines() uses for the Phase Scope plugin's
// own "Smooth" option. When fillUnderneath is set, the region between
// the curve and baselineY is filled *first*, at fillAlpha, before the
// curve itself is stroked on top at full color. The foreground curve
// (Momentary, drawn after Short-term in drawHistoryGraph()) is called
// with a lower fillAlpha specifically so the Short-term curve/fill
// underneath it stays visible rather than being covered.
void LoudnessMeterWidget::strokeSpline(const float* xs, const float* ys, int count, Color color,
                                        bool fillUnderneath, float baselineY, float fillAlpha)
{
    if (count < 2) return;
    const auto idx = [count](int i) { return std::max(0, std::min(count - 1, i)); };
    const auto buildSpline = [&] {
        for (int i = 0; i < count - 1; ++i)
        {
            const int i0 = idx(i - 1), i2 = i + 1, i3 = idx(i + 2);
            const float c1x = xs[i] + (xs[i2] - xs[i0]) / 6.0f;
            const float c1y = ys[i] + (ys[i2] - ys[i0]) / 6.0f;
            const float c2x = xs[i2] - (xs[i3] - xs[i]) / 6.0f;
            const float c2y = ys[i2] - (ys[i3] - ys[i]) / 6.0f;
            bezierTo(c1x, c1y, c2x, c2y, xs[i2], ys[i2]);
        }
    };

    if (fillUnderneath)
    {
        beginPath();
        moveTo(xs[0], baselineY);
        lineTo(xs[0], ys[0]);
        buildSpline();
        lineTo(xs[count - 1], baselineY);
        closePath();
        fillColor(color.withAlpha(fillAlpha));
        fill();
    }

    beginPath();
    moveTo(xs[0], ys[0]);
    buildSpline();
    strokeColor(color);
    strokeWidth(1.5f);
    stroke();
}

// A monotone cubic Hermite spline (Fritsch-Carlson) through the given
// already-screen-mapped points -- used for the Distribution density
// curve specifically, in place of strokeSpline()'s own Catmull-Rom
// construction above. Catmull-Rom is the right choice for a naturally
// smooth, continuously-varying trace (the goniometer's own trail, the
// history graph's own LUFS-over-time curves) but the wrong one for the
// Distribution histogram's own bin counts: adjacent bins can differ
// sharply (a steady passage concentrates almost everything in one or two
// bins, with near-zero neighbors), and Catmull-Rom's tangents are
// computed from a point's neighbors with no constraint against
// overshooting past them -- visible as Runge-like ringing (an
// unconstrained interpolant oscillating past its own data points around a
// sharp change) and, separately, as the rendered peak sometimes not lining up
// with the actual peak bin -- both symptoms of the same cause, an
// overshot curve reaching higher between two points than either point's
// own value.
//
// Fritsch-Carlson fixes both by construction, not by clamping the result
// after the fact: each interior tangent starts as the average of its two
// neighboring secant slopes, but is forced to exactly zero whenever
// those two slopes disagree in sign or either is zero -- which is
// exactly what happens at a local extremum (a bin taller than both
// neighbors has a rising slope on one side and a falling one on the
// other) -- and a further per-interval rescale (the `alpha`/`beta`/`tau`
// step) guarantees the curve never leaves the `[min(y_i, y_{i+1}),
// max(y_i, y_{i+1})]` range on any single interval. A zero tangent at a
// strict local max means the curve is flat exactly at that point with
// nothing pushing it higher on either side, and the interval bound means
// no other segment can reach higher than its own two endpoints either --
// together, the piecewise curve's global maximum is always one of the
// actual data points, never a spot between them, which is exactly "clamp
// the peak" done as an actual mathematical guarantee instead of a
// post-hoc pixel clamp.
void LoudnessMeterWidget::strokeMonotoneSpline(const float* xs, const float* ys, int count, Color color,
                                                bool fillUnderneath, float baselineY, float fillAlpha)
{
    if (count < 2) return;
    if (count > kMaxDistributionBins) count = kMaxDistributionBins; // this spline is only ever used for Distribution bins

    float d[kMaxDistributionBins - 1];
    float m[kMaxDistributionBins];
    for (int i = 0; i < count - 1; ++i)
    {
        const float dx = xs[i + 1] - xs[i];
        d[i] = (dx > 1.0e-6f) ? (ys[i + 1] - ys[i]) / dx : 0.0f;
    }
    m[0] = d[0];
    m[count - 1] = d[count - 2];
    for (int i = 1; i < count - 1; ++i)
        m[i] = ((d[i - 1] > 0.0f) == (d[i] > 0.0f) && d[i - 1] != 0.0f && d[i] != 0.0f)
                   ? 0.5f * (d[i - 1] + d[i])
                   : 0.0f;
    for (int i = 0; i < count - 1; ++i)
    {
        if (d[i] == 0.0f) { m[i] = 0.0f; m[i + 1] = 0.0f; continue; }
        const float a = m[i] / d[i], b = m[i + 1] / d[i];
        const float s = a * a + b * b;
        if (s > 9.0f)
        {
            const float tau = 3.0f / std::sqrt(s);
            m[i] = tau * a * d[i];
            m[i + 1] = tau * b * d[i];
        }
    }

    const auto buildSpline = [&] {
        for (int i = 0; i < count - 1; ++i)
        {
            const float dx = (xs[i + 1] - xs[i]) / 3.0f;
            bezierTo(xs[i] + dx, ys[i] + m[i] * dx, xs[i + 1] - dx, ys[i + 1] - m[i + 1] * dx, xs[i + 1], ys[i + 1]);
        }
    };

    if (fillUnderneath)
    {
        beginPath();
        moveTo(xs[0], baselineY);
        lineTo(xs[0], ys[0]);
        buildSpline();
        lineTo(xs[count - 1], baselineY);
        closePath();
        fillColor(color.withAlpha(fillAlpha));
        fill();
    }

    beginPath();
    moveTo(xs[0], ys[0]);
    buildSpline();
    strokeColor(color);
    strokeWidth(1.5f);
    stroke();
}

void LoudnessMeterWidget::drawBarMeter(float x, float y, float w, float h)
{
    constexpr float labelH = 14.0f;
    const float barY = y + labelH;
    const float barH = h - 2.0f * labelH;
    const float barW = w * 0.55f; // leaves room for the Short-term/Integrated horizontal bars beside it

    beginPath();
    rect(x, barY, barW, barH);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    // Momentary fill -- the same green -> amber -> red vertical gradient
    // convention LevelMeterWidget's own RMS bar uses, so a loudness meter
    // and an ordinary level meter read the same way at a glance.
    const float frac = dbToFrac(momentaryLufs_, kBarMinLufs, kBarMaxLufs);
    if (frac > 0.001f)
    {
        const float fillH = frac * barH;
        scissor(x, barY + barH - fillH, barW, fillH);
        beginPath();
        rect(x, barY, barW, barH);
        Paint grad = linearGradient(0, barY + barH, 0, barY, Color(90, 200, 120), Color(230, 70, 70));
        fillPaint(grad);
        fill();
        resetScissor();
    }

    // Short-term: a triangular pointer riding the bar's own right edge,
    // the same visual convention Youlean Loudness Meter 2 uses to show
    // Momentary (the filled bar itself) and Short-term (a separate
    // marker) on one shared scale at once.
    const Color shortTermColor(90, 170, 230);
    const Color integratedColor(230, 160, 70);
    const float stFrac = dbToFrac(shortTermLufs_, kBarMinLufs, kBarMaxLufs);
    const float stY = barY + barH - stFrac * barH;
    beginPath();
    moveTo(x + barW, stY);
    lineTo(x + barW + 7.0f, stY - 4.0f);
    lineTo(x + barW + 7.0f, stY + 4.0f);
    closePath();
    fillColor(shortTermColor);
    fill();

    // Short-term and Integrated *also* as short horizontal bars, mapped
    // to this exact same LUFS scale -- "you can plot the shortterm,
    // integrated loudness also in horizontal bar mapped to the scale of
    // the other loudness-y-axis", a more prominent, more directly
    // comparable representation than the triangle pointer alone.
    constexpr float hbarH = 6.0f;
    const float hbarX = x + barW + 12.0f;
    const float hbarW = w - barW - 12.0f - 2.0f;

    beginPath();
    rect(hbarX, stY - hbarH * 0.5f, hbarW, hbarH);
    fillColor(shortTermColor);
    fill();

    const float iFrac = dbToFrac(integratedLufs_, kBarMinLufs, kBarMaxLufs);
    const float iY = barY + barH - iFrac * barH;
    beginPath();
    rect(hbarX, iY - hbarH * 0.5f, hbarW, hbarH);
    fillColor(integratedColor);
    fill();

    fillColor(Color(140, 144, 152));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(8.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    text(hbarX, stY - hbarH * 0.5f - 5.0f, "ST", nullptr);
    text(hbarX, iY + hbarH * 0.5f + 5.0f, "I", nullptr);

    // Target reference tick, only when a target is actually selected.
    if (targetEnabled_)
    {
        const float tFrac = dbToFrac(targetLufs_, kBarMinLufs, kBarMaxLufs);
        const float tY = barY + barH - tFrac * barH;
        beginPath();
        moveTo(x - 3.0f, tY);
        lineTo(x + barW + 3.0f, tY);
        strokeColor(Color(235, 238, 242));
        strokeWidth(1.5f);
        stroke();
    }

    // Scale labels at each 9 LUFS step -- 0, -9, -18, -27, -36, matching
    // kBarMinLufs/kBarMaxLufs's own -40..+3 span closely enough to read
    // cleanly without crowding.
    fillColor(Color(140, 144, 152));
    fontSize(9.0f);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    for (float v = 0.0f; v >= kBarMinLufs; v -= 9.0f)
    {
        const float ty = barY + barH - dbToFrac(v, kBarMinLufs, kBarMaxLufs) * barH;
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%.0f", double(v));
        text(x + barW * 0.5f, ty - 10.0f, buf, nullptr);
    }
    textAlign(ALIGN_CENTER | ALIGN_BOTTOM);
    text(x + barW * 0.5f, y + h, "LUFS", nullptr);
}

void LoudnessMeterWidget::drawHistoryGraph(float x, float y, float w, float h)
{
    beginPath();
    rect(x, y, w, h);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    // Gridlines at the same fixed LUFS steps the bar meter's own scale
    // uses, so the two stay visually consistent with each other.
    fillColor(Color(100, 104, 112));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    for (float v = 0.0f; v >= kBarMinLufs; v -= 9.0f)
    {
        const float gy = y + h - dbToFrac(v, kBarMinLufs, kBarMaxLufs) * h;
        beginPath();
        moveTo(x, gy);
        lineTo(x + w, gy);
        strokeColor(Color(32, 34, 40));
        strokeWidth(1.0f);
        stroke();
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%.0f", double(v));
        text(x + 3.0f, gy - 11.0f, buf, nullptr);
    }

    // Target reference line, dashed so it reads as "a goal", not a
    // measured curve -- NanoVG has no native dash support, so this is a
    // manually segmented line.
    if (targetEnabled_)
    {
        const float ty = y + h - dbToFrac(targetLufs_, kBarMinLufs, kBarMaxLufs) * h;
        constexpr float dash = 6.0f, gapLen = 4.0f;
        for (float dx = 0.0f; dx < w; dx += dash + gapLen)
        {
            beginPath();
            moveTo(x + dx, ty);
            lineTo(x + std::min(w, dx + dash), ty);
            strokeColor(Color(200, 204, 210));
            strokeWidth(1.5f);
            stroke();
        }
    }

    // The x-axis window currently on screen, in historyElapsedNewest_'s
    // own elapsed-seconds coordinate -- see that member's own comment
    // (LoudnessMeterWidget.hpp) for why elapsed-since-start rather than
    // "seconds ago". While anchored, recomputed fresh every draw from
    // historyZoomSpanSec_ alone (the window always ends at "now"); once
    // panned away, the frozen historyViewMinSec_/historyViewMaxSec_ pair
    // onMotion() maintains.
    float viewMin, viewMax;
    if (historyAnchored_ || !historyHasData_)
    {
        viewMax = float(historyElapsedNewest_);
        viewMin = std::max(0.0f, viewMax - historyZoomSpanSec_);
    }
    else
    {
        viewMin = historyViewMinSec_;
        viewMax = historyViewMaxSec_;
    }
    const auto xForSec = [&](float sec) { return x + (sec - viewMin) / std::max(1.0e-6f, viewMax - viewMin) * w; };

    // Time grid on the x axis -- same dynamic-step technique (and the same reasoning for why the
    // default view still lands on a clean step) as drawDistribution()'s
    // own LUFS grid, see pickGridStep()'s own comment; seconds-flavored
    // candidates instead of LUFS ones, and labeled "Ns" rather than a
    // bare number since there's no shared column of units the way the
    // bar-meter/history LUFS axis has to lean on.
    {
        const float span = viewMax - viewMin;
        constexpr float kTickCandidates[] = { 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 30.0f, 60.0f };
        constexpr int kMaxTicks = 9;
        const float step = pickGridStep(span, kTickCandidates, int(sizeof(kTickCandidates) / sizeof(kTickCandidates[0])), kMaxTicks);
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(8.5f);
        fillColor(Color(100, 104, 112));
        textAlign(ALIGN_LEFT | ALIGN_BOTTOM);
        const float firstTick = std::ceil(viewMin / step) * step;
        for (float t = firstTick; t <= viewMax + 0.001f * step; t += step)
        {
            const float gx = xForSec(t);
            beginPath();
            moveTo(gx, y);
            lineTo(gx, y + h);
            strokeColor(Color(26, 28, 33)); // dimmer than the LUFS grid above -- this axis is secondary, not the main reading
            strokeWidth(1.0f);
            stroke();
            char buf[12];
            std::snprintf(buf, sizeof(buf), step < 1.0f ? "%.1fs" : "%.0fs", double(t));
            text(gx + 2.0f, y + h - 2.0f, buf, nullptr);
        }
    }

    // The Live badge -- an anchor that snaps the view back to "now" when
    // activated. Green/"LIVE" while following "now"; grey/
    // "PAUSED" once panned away, clickable (onMouseHistory()) to jump
    // back. Drawn after the grid/target line so it always sits on top.
    {
        float bx, by, bw, bh;
        historyLiveBadgeBounds(bx, by, bw, bh);
        const Color badgeColor = historyAnchored_ ? Color(90, 200, 120) : Color(120, 124, 132);
        beginPath();
        roundedRect(bx, by, bw, bh, 3.0f);
        fillColor(Color(24, 26, 31, 220));
        fill();
        strokeColor(badgeColor);
        strokeWidth(1.0f);
        stroke();
        beginPath();
        circle(bx + 8.0f, by + bh * 0.5f, 2.5f);
        fillColor(badgeColor);
        fill();
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(8.5f);
        fillColor(badgeColor);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        text(bx + 14.0f, by + bh * 0.5f + 1.0f, historyAnchored_ ? "LIVE" : "PAUSED", nullptr);
    }

    if (historyFilled_ < 2) return;

    // Which raw sample indices fall within (or just past) the visible
    // window -- solved directly rather than scanned for, since elapsed
    // time is a plain linear function of index (i.e. already sorted):
    // scanning from index 0 every frame to find where the visible range
    // starts would cost O(historyFilled_) all by itself once the buffer
    // holds hours of data, even before any actual drawing happens. A
    // small overscan margin (10% of the visible span, in seconds -- the
    // same 10%-of-panel-width margin the pre-decimation version of this
    // code used, just expressed in the time domain directly since
    // xForSec() is linear too) is kept on each side so the spline still
    // continues naturally through the panel's own edge instead of
    // visibly starting/stopping exactly at it; scissor() below still
    // clips the actual drawing to the panel regardless.
    const float period = kHistorySamplePeriodSec;
    const float marginSec = 0.1f * (viewMax - viewMin);
    const auto indexForElapsed = [&](float e) {
        return int(std::floor(float(historyFilled_ - 1) - (float(historyElapsedNewest_) - e) / period));
    };
    const int firstIdx = std::max(0, indexForElapsed(viewMin - marginSec));
    const int lastIdxExclusive = std::min(historyFilled_, indexForElapsed(viewMax + marginSec) + 2);
    const int visibleCount = lastIdxExclusive - firstIdx;
    if (visibleCount < 2) return;

    // Decimated to a fixed point budget, independent of how many raw
    // samples are actually in view -- LUFS readings vary smoothly (this
    // is not spiky transient data the way an audio waveform is), so a
    // plain fixed-stride subsample looks visually identical to the
    // full-resolution curve well below this budget; without it, zooming
    // out over a long recording would rebuild a tens-of-thousands-of-
    // point NanoVG path every single frame. The newest (rightmost) sample
    // is always force-included even if the stride would otherwise skip
    // past it, so the live edge never visibly lags behind while zoomed
    // out.
    constexpr int kMaxDrawPoints = 2000; // generous against any realistic panel width
    const int stride = std::max(1, visibleCount / kMaxDrawPoints);

    const float baselineY = y + h;
    historyDrawXs_.clear();
    historyDrawYsShortTerm_.clear();
    historyDrawYsMomentary_.clear();
    const auto addPoint = [&](int i) {
        const float sec = float(historyElapsedNewest_) - float(historyFilled_ - 1 - i) * period;
        const int idx = (historyWritePos_ - historyFilled_ + i + 2 * kHistoryCapacity) % kHistoryCapacity;
        historyDrawXs_.push_back(xForSec(sec));
        historyDrawYsShortTerm_.push_back(y + h - dbToFrac(historyShortTerm_[size_t(idx)], kBarMinLufs, kBarMaxLufs) * h);
        historyDrawYsMomentary_.push_back(y + h - dbToFrac(historyMomentary_[size_t(idx)], kBarMinLufs, kBarMaxLufs) * h);
    };
    for (int i = firstIdx; i < lastIdxExclusive; i += stride) addPoint(i);
    if ((lastIdxExclusive - 1 - firstIdx) % stride != 0) addPoint(lastIdxExclusive - 1);

    const int n = int(historyDrawXs_.size());
    if (n < 2) return;

    // Short-term drawn first (background), Momentary second (foreground)
    // -- its own fill alpha is deliberately lower than Short-term's own
    // so the background curve/fill stays visible underneath it.
    scissor(x, y, w, h);
    strokeSpline(historyDrawXs_.data(), historyDrawYsShortTerm_.data(), n, Color(90, 170, 230), true, baselineY, 0.22f);
    strokeSpline(historyDrawXs_.data(), historyDrawYsMomentary_.data(), n, Color(140, 210, 150), true, baselineY, 0.16f);
    resetScissor();
}

void LoudnessMeterWidget::drawDistribution(float x, float y, float w, float h)
{
    beginPath();
    rect(x, y, w, h);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    // Distribution's own x-axis: the DSP-side data itself is always
    // binned over a fixed -60..0 LUFS span (ssmeter::DistributionHistogram's
    // own kMinLufs/kMaxLufs -- this widget has no DSP dependency, matching
    // GoniometerWidget's own convention, so this is a plain,
    // independently-kept-in-sync constant rather than a shared one, the
    // same trade already made for kBarMinLufs/kBarMaxLufs above), but
    // what's actually *drawn* across the panel's width is
    // distViewMinLufs_/distViewMaxLufs_ -- a user-controlled zoom window
    // over that same fixed data, narrowed/widened by onScroll(). Defaults
    // to the full -60..0 range, so this reads identically to the old
    // fixed-range behavior until the user actually zooms.
    constexpr float kAbsMinLufs = -60.0f;
    constexpr float kAbsMaxLufs = 0.0f;
    const auto xForLufs = [&](float lufs) {
        const float t = (lufs - distViewMinLufs_) / (distViewMaxLufs_ - distViewMinLufs_);
        return x + std::max(0.0f, std::min(1.0f, t)) * w;
    };
    const float axisY = y + h - 14.0f;

    // A grid behind the curve, the same "fixed LUFS steps" convention
    // drawHistoryGraph() uses for its own gridlines (just vertical here,
    // since this plot's x-axis is LUFS rather than time) -- gives the
    // density curve's own horizontal position a fixed frame of reference
    // regardless of which platform Target happens to be selected. The
    // step itself shrinks as the zoom window narrows -- "if zoomed in you
    // can add more ticks on the grid" -- picked as the finest of a fixed
    // candidate list that still keeps the tick count reasonable, so the
    // *default*, fully-zoomed-out view still lands on exactly the old
    // fixed 10 LUFS step (60 LUFS span / 10 = 6 ticks, comfortably under
    // the cap below) while a narrow zoom window gets a genuinely finer
    // grid instead of the same six ticks stretched across it.
    const float viewSpan = distViewMaxLufs_ - distViewMinLufs_;
    constexpr float kTickCandidates[] = { 0.1f, 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f };
    constexpr int kMaxTicks = 9;
    const float step = pickGridStep(viewSpan, kTickCandidates, int(sizeof(kTickCandidates) / sizeof(kTickCandidates[0])), kMaxTicks);
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(8.5f);
    fillColor(Color(100, 104, 112));
    textAlign(ALIGN_CENTER | ALIGN_BOTTOM);
    // Start from the first on-grid tick at or above the visible window's
    // own low edge (not always kAbsMinLufs any more, now that the window
    // can be scrolled away from the data's own bottom) -- std::ceil()
    // against the step keeps ticks landing on the same fixed grid
    // (..., -20, -10, 0, ...) regardless of where the window happens to
    // start, rather than one relative to the window's own edge.
    const float firstTick = std::ceil(distViewMinLufs_ / step) * step;
    for (float v = firstTick; v <= distViewMaxLufs_ + 0.001f * step; v += step)
    {
        const float gx = xForLufs(v);
        beginPath();
        moveTo(gx, y);
        lineTo(gx, axisY);
        strokeColor(Color(32, 34, 40));
        strokeWidth(1.0f);
        stroke();
        char buf[10];
        std::snprintf(buf, sizeof(buf), step < 1.0f ? "%.1f" : "%.0f", double(v));
        text(gx, y + h, buf, nullptr);
    }

    // Headroom reserved at the top: `titleGap` keeps the curves clear of
    // the legend text drawn at y+4 below; `peakGap` on top of that keeps
    // the tallest point of either curve -- and its own peak marker/label,
    // see below -- clear of the plot's own top edge. This alone isn't a
    // complete fix, though (see the `scissor()` clip in `drawCurve`
    // below for why): it bounds where the tallest *data point* is placed,
    // not where the *curve* -- a Catmull-Rom spline through that point --
    // can actually reach between it and its neighbors.
    constexpr float titleGap = 14.0f;
    constexpr float peakGap = 20.0f;
    const float curveTop = y + titleGap + peakGap;

    const Color momentaryColor(140, 210, 150);
    const Color shortTermColor(90, 170, 230);

    // One shared count scale for both curves -- "plot the two
    // distribution on the same y axis" -- rather than each curve
    // normalizing against its own tallest bin, so a genuinely taller peak
    // in one reads as taller than the other's, not just "however tall
    // this one curve happens to stretch itself to".
    float sharedMaxCount = 1.0f;
    for (int i = 0; i < distributionBinCount_; ++i)
        sharedMaxCount = std::max({ sharedMaxCount, distributionCountsMomentary_[i], distributionCountsShortTerm_[i] });
    const float safeMaxCount = std::max(1.0f, sharedMaxCount);

    // One shared box, both curves overlapping on it, rather than the
    // previous two stacked boxes, which still read as "two axes" even
    // once both were put on one shared count scale.
    // Short-term drawn first (background, lower `fillAlpha`), Momentary
    // second (foreground) -- the exact same layering `drawHistoryGraph()`
    // already uses for its own overlaid Momentary/Short-term curves, and
    // for the same reason: the foreground curve's own fill needs to stay
    // translucent enough that the background curve/fill underneath it
    // doesn't just disappear.
    //
    // A smooth, filled density curve through each bin's own center (the
    // distribution of loudness readings over the whole session) via
    // strokeMonotoneSpline() (see its own comment), *not* the Catmull-Rom
    // strokeSpline() the history graph's own curves use: histogram bin
    // counts are naturally blocky (a steady passage concentrates almost
    // everything in one or two bins, with near-zero neighbors), and
    // Catmull-Rom overshoots past such a sharp change -- plus the
    // rendered peak sometimes not lining up with the actual peak bin, the
    // same overshoot's other visible symptom. The scissor clip below is
    // kept anyway as a second, independent line of defense (belt and
    // braces, not a substitute for the real fix): even a spline
    // guaranteed not to overshoot past its own data points is still only
    // as good as the headroom reserved above the tallest one, and a hard
    // clip costs nothing to also have in place regardless.
    const auto drawCurve = [&](const float* counts, Color color, float fillAlpha, bool labelAbove) {
        if (distributionBinCount_ <= 1) return;
        // Each bin's own fixed LUFS center (independent of the view zoom
        // -- the underlying data's own binning never changes, only how
        // much of it is currently visible), projected through xForLufs()
        // so a bin's screen position follows the current zoom window --
        // was a plain, implicitly-full-range `x + (i+0.5)*binW` before
        // the zoom feature existed.
        float xs[kMaxDistributionBins], ys[kMaxDistributionBins];
        int peakIdx = -1;
        float peakVal = 0.0f;
        for (int i = 0; i < distributionBinCount_; ++i)
        {
            const float binLufs = kAbsMinLufs + (float(i) + 0.5f) * (kAbsMaxLufs - kAbsMinLufs) / float(distributionBinCount_);
            xs[i] = xForLufs(binLufs);
            const float frac = counts[i] / safeMaxCount;
            ys[i] = axisY - frac * (axisY - curveTop);
            if (counts[i] > peakVal) { peakVal = counts[i]; peakIdx = i; }
        }
        scissor(x, y, w, axisY - y);
        strokeMonotoneSpline(xs, ys, distributionBinCount_, color, true, axisY, fillAlpha);
        resetScissor();

        // Track the peak (the mode -- the single most-often-hit loudness
        // bin) and plot its own value right there -- a small dot plus
        // the bin's own center LUFS value, so the curve's shape doesn't
        // have to be eyeballed against the grid to answer "so what
        // loudness is this actually peaking at". Momentary's own label
        // sits above its dot, Short-term's below -- both curves now share
        // one box, and their peaks often land close together in LUFS, so
        // putting the two labels on opposite sides of their own dots
        // keeps them from typically overlapping each other.
        const float peakLufs = peakIdx >= 0
            ? kAbsMinLufs + (float(peakIdx) + 0.5f) * (kAbsMaxLufs - kAbsMinLufs) / float(distributionBinCount_)
            : 0.0f;
        // Skip the marker entirely once zoomed into a window that doesn't
        // contain the peak bin -- xForLufs() clamps out-of-range points to
        // the panel's own edge, which would otherwise draw a dot/label
        // sitting right at the border, misleadingly implying the peak is
        // there rather than further off-screen.
        if (peakIdx >= 0 && peakVal > 0.0f && peakLufs >= distViewMinLufs_ && peakLufs <= distViewMaxLufs_)
        {
            beginPath();
            circle(xs[peakIdx], ys[peakIdx], 3.0f);
            fillColor(color);
            fill();

            char buf[16];
            std::snprintf(buf, sizeof(buf), "%+.1f", double(peakLufs));
            fontFace(NANOVG_DEJAVU_SANS_TTF);
            fontSize(9.5f);
            fillColor(color);
            if (labelAbove)
            {
                textAlign(ALIGN_CENTER | ALIGN_BOTTOM);
                text(xs[peakIdx], ys[peakIdx] - 6.0f, buf, nullptr);
            }
            else
            {
                textAlign(ALIGN_CENTER | ALIGN_TOP);
                text(xs[peakIdx], ys[peakIdx] + 6.0f, buf, nullptr);
            }
        }
    };

    drawCurve(distributionCountsShortTerm_, shortTermColor, 0.22f, false);
    drawCurve(distributionCountsMomentary_, momentaryColor, 0.16f, true);

    // The user's own selected Target (which now includes the streaming-
    // platform figures -- see UILoudnessMeter's own kTargetPresets) is
    // still drawn here, same as every other view; no permanent
    // platform-specific lines any more (see this file's own header
    // comment for why).
    if (targetEnabled_)
    {
        const float lx = xForLufs(targetLufs_);
        beginPath();
        moveTo(lx, y);
        lineTo(lx, axisY);
        strokeColor(Color(235, 238, 242));
        strokeWidth(1.5f);
        stroke();
    }

    // A small color legend doubles as the plot's own title -- two curves
    // sharing one box need labeling now that there's no separate "which
    // panel is this" title per curve to lean on.
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    fillColor(momentaryColor);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x + 6.0f, y + 4.0f, "Momentary", nullptr);
    fillColor(shortTermColor);
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w - 6.0f, y + 4.0f, "Short-term", nullptr);
}

void LoudnessMeterWidget::drawReadoutRow(float x, float y, float w, const char* label, const std::string& value, Color color)
{
    fillColor(Color(150, 154, 162));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(11.5f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x, y, label, nullptr);

    fillColor(color);
    fontSize(21.0f); // "Plot the Measurement numbers a little bigger" -- raised from an original 15.0
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w, y - 2.0f, value.c_str(), nullptr);
}

void LoudnessMeterWidget::drawReadoutPanel(float x, float y, float w, float h)
{
    beginPath();
    rect(x, y, w, h);
    // A brief bright flash right after Reset is clicked -- see
    // setResetFlash()'s own comment: confirms the click actually
    // registered even on material whose Integrated/Range/Peak-max
    // numbers happen to resettle to a very similar value within a second
    // or two, which otherwise can look indistinguishable from "nothing
    // happened".
    const Color base(24, 26, 31);
    const Color flashColor(70, 90, 110);
    fillColor(Color(base, flashColor, resetFlash_));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    constexpr float padX = 10.0f;
    constexpr float rowH = 40.0f;
    float ry = y + 10.0f;

    const Color momentaryColor(140, 210, 150);
    const Color shortTermColor(90, 170, 230);
    const Color neutralColor(224, 227, 232);

    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "Momentary (LUFS)", formatLufs(momentaryLufs_), momentaryColor);
    ry += rowH;
    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "Short-term (LUFS)", formatLufs(shortTermLufs_), shortTermColor);
    ry += rowH;
    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "Integrated (LUFS)", formatLufs(integratedLufs_), neutralColor);
    ry += rowH;
    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "Loudness Range (LU)", formatLu(loudnessRangeLu_), neutralColor);
    ry += rowH * 1.3f;

    beginPath();
    moveTo(x + padX, ry - rowH * 0.5f);
    lineTo(x + w - padX, ry - rowH * 0.5f);
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "True Peak L (dBTP)", formatLufs(truePeakL_), neutralColor);
    ry += rowH;
    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "True Peak R (dBTP)", formatLufs(truePeakR_), neutralColor);
    ry += rowH;
    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "True Peak Max (dBTP)", formatLufs(truePeakMax_),
                    truePeakMax_ >= -1.0f ? Color(230, 90, 90) : neutralColor);
    ry += rowH * 1.3f;

    beginPath();
    moveTo(x + padX, ry - rowH * 0.5f);
    lineTo(x + w - padX, ry - rowH * 0.5f);
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "PLR (LU)", formatLu(plrLu_), neutralColor);
    ry += rowH;
    drawReadoutRow(x + padX, ry, w - 2.0f * padX, "PSR (LU)", formatLu(psrLu_), neutralColor);
}

bool LoudnessMeterWidget::onScroll(const ScrollEvent& ev)
{
    return showDistribution_ ? onScrollDistribution(ev) : onScrollHistory(ev);
}

bool LoudnessMeterWidget::onMouse(const MouseEvent& ev)
{
    return showDistribution_ ? onMouseDistribution(ev) : onMouseHistory(ev);
}

// X-axis zoom for the Distribution view, centered on the cursor. No-op
// (returns false, letting the event fall through) whenever the cursor is
// outside the center panel.
bool LoudnessMeterWidget::onScrollDistribution(const ScrollEvent& ev)
{
    float px, py, pw, ph;
    centerPanelBounds(px, py, pw, ph);
    const double mx = ev.pos.getX(), my = ev.pos.getY();
    if (mx < px || mx > px + pw || my < py || my > py + ph) return false;

    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    constexpr float kAbsMinLufs = -60.0f; // matches drawDistribution()'s own kAbsMinLufs/kAbsMaxLufs
    constexpr float kAbsMaxLufs = 0.0f;
    constexpr float kMinSpan = 3.0f;                        // finest zoom-in: 3 LUFS visible
    constexpr float kMaxSpan = kAbsMaxLufs - kAbsMinLufs;    // 60 -- fully zoomed out (the default)

    // Same "Alt+scroll = 10x finer" convention this project uses
    // everywhere else a scroll wheel adjusts something continuously (see
    // e.g. EqGraphWidget::onScroll()'s identical comment) -- here it
    // controls how much each notch zooms rather than a parameter step.
    constexpr float kFactor = 1.15f;
    const float factor = (ev.mod & kModifierAlt) ? 1.0f + (kFactor - 1.0f) / 10.0f : kFactor;

    // Where the cursor currently points, as a fraction of the *current*
    // view window -- kept fixed across the zoom (recomputed against the
    // *new* window below), which is what "centered on the mouse" means
    // for a zoom-under-cursor control: the LUFS value under the cursor
    // doesn't move, everything else scales/pans around it.
    const float t = std::max(0.0f, std::min(1.0f, float((mx - double(px)) / double(pw))));
    const float oldSpan = distViewMaxLufs_ - distViewMinLufs_;
    const float mouseLufs = distViewMinLufs_ + t * oldSpan;

    float newSpan = dir > 0.0 ? oldSpan / factor : oldSpan * factor;
    newSpan = std::max(kMinSpan, std::min(kMaxSpan, newSpan));

    float newMin = mouseLufs - t * newSpan;
    float newMax = newMin + newSpan;
    // Clamp to the data's own absolute range by *sliding* the window
    // (preserving newSpan) rather than shrinking it -- only actually
    // matters right at the two ends, where zooming in right at the edge
    // would otherwise let the cursor's own fixed-fraction math push the
    // window past kAbsMinLufs/kAbsMaxLufs.
    if (newMin < kAbsMinLufs) { newMin = kAbsMinLufs; newMax = newMin + newSpan; }
    if (newMax > kAbsMaxLufs) { newMax = kAbsMaxLufs; newMin = newMax - newSpan; }
    distViewMinLufs_ = std::max(kAbsMinLufs, newMin);
    distViewMaxLufs_ = std::min(kAbsMaxLufs, newMax);

    repaint();
    return true;
}

// Double-click on the Distribution view resets the x-axis zoom back to
// the full -60..0 range -- a zoom control with no way back out is just a
// trap, same "give it an escape hatch" reasoning behind every other
// double-click-to-reset/clear control already in this project (e.g.
// UIEq.cpp's checkInlineDoubleClick()), just at the widget level here
// since this plugin's own UI class never sees this panel's mouse events
// directly.
bool LoudnessMeterWidget::onMouseDistribution(const MouseEvent& ev)
{
    if (!ev.press || ev.button != 1) return false;

    float px, py, pw, ph;
    centerPanelBounds(px, py, pw, ph);
    const double mx = ev.pos.getX(), my = ev.pos.getY();
    if (mx < px || mx > px + pw || my < py || my > py + ph) return false;

    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400); // same window ContourGraphWidget's own double-click detection uses
    const bool isDoubleClick = (now - lastDistClickTime_) < kDoubleClickWindow;
    lastDistClickTime_ = now;
    if (isDoubleClick)
    {
        distViewMinLufs_ = -60.0f;
        distViewMaxLufs_ = 0.0f;
        repaint();
    }
    return true;
}

void LoudnessMeterWidget::historyLiveBadgeBounds(float& bx, float& by, float& bw, float& bh) const
{
    float px, py, pw, ph;
    centerPanelBounds(px, py, pw, ph);
    bw = 46.0f;
    bh = 14.0f;
    bx = px + pw - bw - 4.0f;
    by = py + 4.0f;
    (void)ph;
}

// History's own scroll-zoom. While anchored (the default -- "always
// anchored at now"), scrolling just resizes historyZoomSpanSec_, the
// right edge staying pinned to "now" by definition (see
// drawHistoryGraph()'s own view-window computation) -- there's only one
// real degree of freedom while anchored, so cursor position doesn't
// change the *result*, just which of the two constexpr-obvious directions
// "scroll up" zooms. Once panned away, an ordinary cursor-centered zoom
// over the frozen window, same mechanic (and the same reasoning) as
// onScrollDistribution()'s own.
bool LoudnessMeterWidget::onScrollHistory(const ScrollEvent& ev)
{
    float px, py, pw, ph;
    centerPanelBounds(px, py, pw, ph);
    const double mx = ev.pos.getX(), my = ev.pos.getY();
    if (mx < px || mx > px + pw || my < py || my > py + ph) return false;

    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    constexpr float kFactor = 1.15f; // same "Alt+scroll = 10x finer" convention as onScrollDistribution()
    const float factor = (ev.mod & kModifierAlt) ? 1.0f + (kFactor - 1.0f) / 10.0f : kFactor;

    if (historyAnchored_)
    {
        float span = dir > 0.0 ? historyZoomSpanSec_ / factor : historyZoomSpanSec_ * factor;
        historyZoomSpanSec_ = std::max(kHistoryMinSpanSec, std::min(kHistoryMaxSpanSec, span));
        repaint();
        return true;
    }

    const float t = std::max(0.0f, std::min(1.0f, float((mx - double(px)) / double(pw))));
    const float oldSpan = historyViewMaxSec_ - historyViewMinSec_;
    const float cursorSec = historyViewMinSec_ + t * oldSpan;

    float newSpan = dir > 0.0 ? oldSpan / factor : oldSpan * factor;
    newSpan = std::max(kHistoryMinSpanSec, std::min(kHistoryMaxSpanSec, newSpan));

    float newMin = cursorSec - t * newSpan;
    float newMax = newMin + newSpan;
    const float nowSec = float(historyElapsedNewest_);
    if (newMin < 0.0f) { newMin = 0.0f; newMax = newMin + newSpan; }
    if (newMax > nowSec) { newMax = nowSec; newMin = newMax - newSpan; }
    historyViewMinSec_ = std::max(0.0f, newMin);
    historyViewMaxSec_ = std::min(nowSec, newMax);

    repaint();
    return true;
}

bool LoudnessMeterWidget::onMouseHistory(const MouseEvent& ev)
{
    if (ev.button != 1) return false;

    if (!ev.press)
    {
        if (!historyDragging_) return false; // not a release for a drag this widget itself started
        historyDragging_ = false;
        return true;
    }

    float px, py, pw, ph;
    centerPanelBounds(px, py, pw, ph);
    const double mx = ev.pos.getX(), my = ev.pos.getY();
    if (mx < px || mx > px + pw || my < py || my > py + ph) return false;

    // The Live badge -- re-anchor (keeping the current zoom span, "snaps
    // to now when activated") and consume the click; never starts a drag.
    float bx, by, bw, bh;
    historyLiveBadgeBounds(bx, by, bw, bh);
    if (mx >= bx && mx <= bx + bw && my >= by && my <= by + bh)
    {
        historyAnchored_ = true;
        repaint();
        return true;
    }

    // Double-click anywhere else on the panel: full reset (span back to
    // the default 60s -- not kHistoryMaxSpanSec/6h, that's just the
    // *ceiling* zooming out can reach, not what "reset" should jump to --
    // re-anchored) -- same escape-hatch reasoning as
    // onMouseDistribution()'s own double-click.
    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400);
    const bool isDoubleClick = (now - lastHistoryClickTime_) < kDoubleClickWindow;
    lastHistoryClickTime_ = now;
    if (isDoubleClick)
    {
        historyZoomSpanSec_ = kHistoryDefaultSpanSec;
        historyAnchored_ = true;
        historyDragging_ = false;
        repaint();
        return true;
    }

    // Otherwise: a potential drag-to-pan starts here -- see onMotion()'s
    // own comment for why historyAnchored_ isn't actually broken until
    // real movement is observed, not just this press.
    historyDragging_ = true;
    historyDragStartX_ = mx;
    if (historyAnchored_)
    {
        historyDragStartMaxSec_ = float(historyElapsedNewest_);
        historyDragStartMinSec_ = std::max(0.0f, historyDragStartMaxSec_ - historyZoomSpanSec_);
    }
    else
    {
        historyDragStartMinSec_ = historyViewMinSec_;
        historyDragStartMaxSec_ = historyViewMaxSec_;
    }
    return true;
}

// Drag-to-pan for the History view; starting a pan releases the Live
// anchor. Recomputes the pan from the drag's own
// fixed press-time reference (historyDragStartX_/historyDragStartMinSec_/
// historyDragStartMaxSec_) every event rather than accumulating
// per-motion deltas -- same idiom
// ContourGraphWidget::onMotion() (../../compressor/gui/
// ContourGraphWidget.cpp) already uses for its own point-dragging, and
// immune to any drift a chain of incremental deltas could otherwise
// accumulate over a long drag.
bool LoudnessMeterWidget::onMotion(const MotionEvent& ev)
{
    if (!historyDragging_ || showDistribution_) return false;

    const double dx = ev.pos.getX() - historyDragStartX_;
    if (dx == 0.0) return true; // no real movement yet -- don't break the anchor on a stationary press

    historyAnchored_ = false;

    float px, py, pw, ph;
    centerPanelBounds(px, py, pw, ph);
    const float span = historyDragStartMaxSec_ - historyDragStartMinSec_;
    // Dragging right reveals *earlier* history sliding in from the left
    // -- the window's own coordinates move backward in time, same as
    // grabbing and pulling a strip of paper toward you.
    const float deltaSec = float(dx / double(pw)) * span;
    float newMin = historyDragStartMinSec_ - deltaSec;
    float newMax = historyDragStartMaxSec_ - deltaSec;
    const float nowSec = float(historyElapsedNewest_);
    if (newMin < 0.0f) { newMax += -newMin; newMin = 0.0f; }
    if (newMax > nowSec) { newMin -= (newMax - nowSec); newMax = nowSec; }
    historyViewMinSec_ = std::max(0.0f, newMin);
    historyViewMaxSec_ = newMax;

    repaint();
    return true;
}

END_NAMESPACE_DGL
