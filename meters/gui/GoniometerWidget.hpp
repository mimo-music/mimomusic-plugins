// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// GoniometerWidget.hpp
//
// The rotated Mid/Side vectorscope ("goniometer") -- the main display of
// this plugin. Draws the current contents of ssmeter::GoniometerBuffer's
// ring (already unpacked by UIPhaseScope, one uiIdle() tick at a time, from
// the packed output Parameters -- see that class's own file comment for
// why bulk data has to travel that way) as a fading dot trail: a mono
// signal draws a vertical line straight up, a signal panned hard left or
// right draws toward the bottom-left/bottom-right diagonal respectively,
// and a fully out-of-phase (mono-incompatible) signal draws a horizontal
// line -- the standard, widely recognized goniometer layout.
//
// Two independent, purely cosmetic/performance knobs sit on top of that,
// both UI-only (see UIPhaseScope.cpp's own kStateResolutionPercent/
// kStateSmoothLines comments -- neither reaches the DSP side at all):
// Resolution (setResolutionPercent()) thins how many of the currently-
// visible buffered points actually get a draw call each repaint, and
// Smooth (setSmoothLines()) -- Draw Lines mode only -- replaces the
// straight segments between consecutive drawn points with a Catmull-Rom
// spline through them, so a heavily-thinned (low Resolution) trail still
// reads as one continuous curve rather than a visibly faceted polyline.
//
// A third, independent layer -- Heatmap (setHeatmapEnabled()/
// setHeatmap()) -- draws ssmeter::HeatmapGrid's own long-history (1-30s)
// density map underneath all of the above: a "where has this signal
// actually been spending its time" glow, always accumulating DSP-side
// regardless of whether it's currently shown (see setHeatmapEnabled()'s
// own comment).

#pragma once

#include "NanoVG.hpp"

#include "../plugin/dsp/PhaseScopeDsp.hpp"

#include <algorithm>

START_NAMESPACE_DGL

class GoniometerWidget : public NanoSubWidget
{
public:
    explicit GoniometerWidget(Widget* parent);

    // xs[i]/ys[i]: Side/Mid pairs, already unpacked, in ring-buffer order.
    // writeSlot: the slot ssmeter::GoniometerBuffer will overwrite *next*
    // -- i.e. one past the newest point -- used to derive each point's
    // fade-age. Called once per uiIdle() tick; does nothing (and the
    // widget keeps showing its last frame) while the owning UI is frozen.
    void setScope(const float* xs, const float* ys, int writeSlot);

    // Tints the dot trail green->amber->red as correlation drops from +1
    // to -1 -- ties the two displays together at a glance, "there's a
    // phase problem" is visible in the dot cloud's own color, not only in
    // a separate numeric readout.
    void setCorrelation(float corr) { correlation_ = corr; }

    // false (default): a fading dot cloud. true: consecutive drawn points
    // (in the same chronological order, after Resolution thinning) are
    // connected by lines instead -- a continuous vector trace rather than
    // discrete samples, the other common goniometer rendering style. Both
    // share the same per-point age/alpha and correlation-tinted color.
    void setDrawLines(bool drawLines) { drawLines_ = drawLines; repaint(); }

    // How much of the fixed, constantly-refilling buffer to actually
    // draw -- see ssmeter::GoniometerBuffer::visibleCountForDecay()'s own
    // comment: the buffer is always written at the same fixed rate, this
    // purely controls how far back the visible trail reaches (and,
    // because both age and alpha are normalized against this count, not
    // the full buffer capacity, the trail is exactly as dense at any
    // Decay setting -- a longer Decay genuinely shows more points, not
    // the same fixed count stretched thinner).
    void setDecayMs(float ms) { visibleCount_ = ssmeter::GoniometerBuffer::visibleCountForDecay(double(ms)); repaint(); }

    // 10..100. 100 draws every buffered point in the currently-visible
    // window (the previous, only-ever behavior); lower values draw every
    // Nth one instead (N = round(100/pct)), a plain, predictable
    // performance/quality trade-off for a machine with less GPU/CPU
    // headroom to spend on this one widget's own draw calls -- entirely
    // independent of Decay (the *time span* shown) and of the DSP side's
    // own fixed write rate (see ssmeter::GoniometerBuffer's file comment),
    // neither of which this touches at all.
    void setResolutionPercent(int pct) { resolutionPercent_ = std::max(kMinResolutionPercent, std::min(100, pct)); repaint(); }

    // Draw Lines mode only (no effect on the dot cloud, which has no
    // notion of a "curve" between points to smooth). false (default):
    // straight segments between consecutive drawn points -- cheapest,
    // and exactly what a low Resolution setting alone would otherwise
    // look like (visibly faceted). true: a Catmull-Rom spline through the
    // same points instead (see drawTrailAsLines()'s own comment) -- a
    // smooth, continuous curve that still passes through every drawn
    // point exactly, so thinning Resolution down for a weaker machine
    // doesn't have to mean a visibly rougher-looking trail too.
    void setSmoothLines(bool smooth) { smoothLines_ = smooth; repaint(); }

    // ssmeter::HeatmapGrid itself always accumulates in the background
    // regardless of this (see that class's own file comment) -- this
    // purely decides whether that already-accumulated grid is actually
    // drawn, so turning it on shows the full multi-second history
    // immediately, not a blank start. Drawn *underneath* the grid/axis
    // lines and the live trail (see onNanoDisplay()'s own ordering) so
    // both stay fully legible on top of it regardless of how bright the
    // heatmap gets.
    void setHeatmapEnabled(bool enabled) { heatmapEnabled_ = enabled; repaint(); }

    // cells[i]: one HeatmapGrid cell's intensity, already unpacked, in
    // [0, 1], row-major (see ssmeter::HeatmapGrid::kGridSize). Called once
    // per uiIdle() tick, same batching convention as setScope().
    void setHeatmap(const float* cells);

    static constexpr int kMinResolutionPercent = 10;

protected:
    void onNanoDisplay() override;

private:
    int collectVisiblePoints(float cx, float cy, float radius);
    void drawTrailAsDots(Color dotColor, int count);
    void drawTrailAsLines(Color dotColor, int count);
    void drawHeatmap(float cx, float cy, float radius);

    static constexpr int kBufferSize = ssmeter::GoniometerBuffer::kScopeBufferSize;
    static constexpr int kHeatmapCellCount = ssmeter::HeatmapGrid::kCellCount;
    static constexpr int kHeatmapGridSize = ssmeter::HeatmapGrid::kGridSize;

    float xs_[kBufferSize] = {};
    float ys_[kBufferSize] = {};
    int writeSlot_ = 0;
    float correlation_ = 1.0f;
    bool drawLines_ = false;
    bool smoothLines_ = true; // on by default
    int resolutionPercent_ = 100;
    int visibleCount_ = ssmeter::GoniometerBuffer::visibleCountForDecay(ssmeter::GoniometerBuffer::kDefaultDecaySeconds * 1000.0);
    bool heatmapEnabled_ = false;
    float heatmapCells_[kHeatmapCellCount] = {};

    // Scratch space filled fresh by collectVisiblePoints() every repaint:
    // already screen-mapped (cx - x*radius, cy - y*radius) coordinates and
    // each point's original age (for alpha), oldest-first, after
    // Resolution thinning -- shared by both drawTrailAsDots() and
    // drawTrailAsLines() so the thinning logic lives in exactly one place.
    float ptSx_[kBufferSize] = {};
    float ptSy_[kBufferSize] = {};
    int ptAge_[kBufferSize] = {};
};

END_NAMESPACE_DGL
