// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "GoniometerWidget.hpp"

#include <algorithm>
#include <cmath>

START_NAMESPACE_DGL

GoniometerWidget::GoniometerWidget(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
}

void GoniometerWidget::setScope(const float* xs, const float* ys, int writeSlot)
{
    std::copy(xs, xs + kBufferSize, xs_);
    std::copy(ys, ys + kBufferSize, ys_);
    writeSlot_ = writeSlot;
    repaint();
}

void GoniometerWidget::setHeatmap(const float* cells)
{
    std::copy(cells, cells + kHeatmapCellCount, heatmapCells_);
    if (heatmapEnabled_) repaint();
}

int GoniometerWidget::collectVisiblePoints(float cx, float cy, float radius)
{
    // Resolution thinning: N = round(100/pct) -- 100% keeps every point
    // (stride 1, the only behavior before Resolution existed), 50% keeps
    // every other one, 10% every tenth. Independent of visibleCount_
    // (Decay's own "how far back" window) -- this only ever thins how
    // many of *that* window's points get an actual draw call.
    const int stride = std::max(1, int(std::lround(100.0 / double(resolutionPercent_))));

    int count = 0;
    for (int age = visibleCount_ - 1; age >= 0; age -= stride)
    {
        const int slot = ((writeSlot_ - 1 - age) % kBufferSize + kBufferSize) % kBufferSize;
        ptSx_[count] = cx - xs_[slot] * radius;
        ptSy_[count] = cy - ys_[slot] * radius;
        ptAge_[count] = age;
        ++count;
    }
    return count;
}

void GoniometerWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());
    const float cx = w * 0.5f;
    const float cy = h * 0.5f;
    const float radius = std::min(w, h) * 0.5f - 14.0f;

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(16, 17, 20));
    fill();

    // Heatmap first, underneath everything else drawn below (the
    // reference grid, axis lines, M/L/R labels, and the live trail) --
    // see setHeatmapEnabled()'s own comment for why that ordering keeps
    // all of those fully legible regardless of how bright the heatmap
    // gets.
    if (heatmapEnabled_)
    {
        scissor(0, 0, w, h);
        drawHeatmap(cx, cy, radius);
        resetScissor();
    }

    // Reference circles at half and full scale.
    for (float frac : { 0.5f, 1.0f })
    {
        beginPath();
        circle(cx, cy, radius * frac);
        strokeColor(Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
    }

    // Axis lines: vertical (Mono), and the two +-45 degree diagonals
    // (hard Left / hard Right). Each line is symmetric (drawn through
    // both of its own ends), but which *end* is the "hard Left" one vs.
    // the "hard Right" one matters for where the L/R labels below go --
    // see this file's own header comment for the derivation: with
    // side=(L-R)/2 on x and mid=(L+R)/2 on y, a hard-Left-only signal
    // traces the upper-left <-> lower-right diagonal, and a hard-Right-only
    // one traces the upper-right <-> lower-left diagonal (an earlier
    // version of this file had these two swapped, and labeled both at the
    // bottom of the circle instead of the top).
    auto axisLine = [&](float dx, float dy) {
        beginPath();
        moveTo(cx - dx * radius, cy - dy * radius);
        lineTo(cx + dx * radius, cy + dy * radius);
        strokeColor(Color(50, 54, 62));
        strokeWidth(1.0f);
        stroke();
    };
    axisLine(0.0f, 1.0f);                // Mono (vertical)
    axisLine(0.70710678f, 0.70710678f);  // upper-left <-> lower-right: hard Left
    axisLine(-0.70710678f, 0.70710678f); // upper-right <-> lower-left: hard Right

    // M/L/R all sit just outside the circle's own top half now -- M
    // straight up, L/R at the upper ends of their own diagonals -- rather
    // than L/R sitting at the bottom, which put them on the wrong side of
    // the circle from where their own diagonal's upper (and, for a real
    // AC audio signal, equally-visited) end actually is.
    const float labelR = radius + 12.0f;
    fillColor(Color(150, 154, 162));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(11.0f);
    textAlign(ALIGN_CENTER | ALIGN_BOTTOM);
    text(cx, cy - labelR, "M", nullptr);
    textAlign(ALIGN_RIGHT | ALIGN_BOTTOM);
    text(cx - labelR * 0.70710678f, cy - labelR * 0.70710678f, "L", nullptr);
    textAlign(ALIGN_LEFT | ALIGN_BOTTOM);
    text(cx + labelR * 0.70710678f, cy - labelR * 0.70710678f, "R", nullptr);

    // Dot color: green (correlation +1) -> amber (0) -> red (-1), the
    // same warning-toward-red direction the correlation meter itself uses.
    // Color's int constructor takes 0..255 -- lerp in that space, then
    // build the Color once (its own storage is normalized [0..1]
    // internally, so re-reading .red/.green/.blue back out and re-lerping
    // in 0..255 terms would be wrong).
    int dotR, dotG, dotB;
    if (correlation_ >= 0.0f)
    {
        const float t = std::min(1.0f, correlation_); // 0 = amber, 1 = green
        dotR = int(235.0f + (90.0f - 235.0f) * t);
        dotG = int(195.0f + (200.0f - 195.0f) * t);
        dotB = int(60.0f + (120.0f - 60.0f) * t);
    }
    else
    {
        const float t = std::min(1.0f, -correlation_); // 0 = amber, 1 = red
        dotR = int(235.0f + (235.0f - 235.0f) * t);
        dotG = int(195.0f + (70.0f - 195.0f) * t);
        dotB = int(60.0f + (60.0f - 60.0f) * t);
    }
    const Color dotColor(dotR, dotG, dotB);

    const int count = collectVisiblePoints(cx, cy, radius);

    scissor(0, 0, w, h);
    if (drawLines_) drawTrailAsLines(dotColor, count);
    else drawTrailAsDots(dotColor, count);
    resetScissor();

    beginPath();
    rect(0.5f, 0.5f, w - 1.0f, h - 1.0f);
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();
}

void GoniometerWidget::drawHeatmap(float cx, float cy, float radius)
{
    // Cell-center-to-world mapping: the exact inverse of
    // ssmeter::HeatmapGrid::cellIndex()'s own quantization, so a cell
    // drawn here lines up with the same spot in the plot its hits
    // actually came from.
    constexpr float kWorldExtent = ssmeter::GoniometerBuffer::kCoordMax;
    constexpr float kCellWorldSize = (2.0f * kWorldExtent) / float(kHeatmapGridSize);
    const float cellScreenSize = kCellWorldSize * radius;
    // Noticeably larger than one cell and drawn as soft circles rather
    // than hard-edged squares -- neighboring cells overlap a fair amount,
    // which is what actually produces the "glow" look rather than a
    // visible grid of blocks, and also smooths over the mild blockiness
    // HeatmapGrid::rescale()'s own nearest-cell remapping can introduce
    // right after a big Gain jump (see that function's own comment).
    // Raised from an original `0.8` -- offered directly as an easier
    // alternative to full resampling ("you can also in or decrease the
    // circle size if its easier"); resampling was implemented too, but
    // the larger, more forgiving circles are worth keeping regardless.
    const float drawRadius = cellScreenSize * 1.1f;

    // Dark blue -> dark green -> dark yellow -> dark red, low to high
    // intensity, fading to transparent -- chosen after two brighter
    // attempts (navy/cyan/pale-white, then black/dark-red/bright-orange)
    // both still made the live trail hard to see. The earlier ramps both climbed to a
    // high-luminance stop at full intensity (pale white, then a fully
    // saturated (255,140,20) orange) that competed with the live trail's
    // own brightness (which reaches channel values up to ~235 -- see this
    // function's own dotColor computation above); every stop here instead
    // stays capped well under 100 per channel, so even the hottest heatmap
    // cell reads as a dark tint sitting well behind whatever's drawn on
    // top of it, never as a competing bright color. Nudged up from an
    // initial under-60 cap that looked a little too dark -- still a dark
    // tint, just a lighter one. Fading to transparent is handled by alpha (intensity *
    // kMaxAlpha below) rather than by the color ramp itself -- a
    // low-intensity cell fades toward the dark background through
    // opacity, not by the RGB approaching black on its own, which would
    // otherwise wash out the hue distinctions between stops at exactly
    // the low end where they matter for reading the shape of the heatmap.
    constexpr float kMaxAlpha = 0.7f;

    for (int gy = 0; gy < kHeatmapGridSize; ++gy)
    {
        for (int gx = 0; gx < kHeatmapGridSize; ++gx)
        {
            const float intensity = heatmapCells_[size_t(gy) * size_t(kHeatmapGridSize) + size_t(gx)];
            if (intensity < 0.03f) continue;

            const float worldX = -kWorldExtent + (float(gx) + 0.5f) * kCellWorldSize;
            const float worldY = -kWorldExtent + (float(gy) + 0.5f) * kCellWorldSize;
            const float sx = cx - worldX * radius;
            const float sy = cy - worldY * radius;

            // Three-segment lerp across four dark stops: blue, green,
            // yellow, red -- each channel value kept well under the live
            // trail's own brightest reach (see the comment above).
            int r, g, b;
            if (intensity < 1.0f / 3.0f)
            {
                const float t = intensity / (1.0f / 3.0f); // dark blue -> dark green
                r = int(14.0f + (14.0f - 14.0f) * t);
                g = int(18.0f + (82.0f - 18.0f) * t);
                b = int(78.0f + (22.0f - 78.0f) * t);
            }
            else if (intensity < 2.0f / 3.0f)
            {
                const float t = (intensity - 1.0f / 3.0f) / (1.0f / 3.0f); // dark green -> dark yellow
                r = int(14.0f + (88.0f - 14.0f) * t);
                g = int(82.0f + (80.0f - 82.0f) * t);
                b = int(22.0f + (14.0f - 22.0f) * t);
            }
            else
            {
                const float t = (intensity - 2.0f / 3.0f) / (1.0f / 3.0f); // dark yellow -> dark red
                r = int(88.0f + (95.0f - 88.0f) * t);
                g = int(80.0f + (18.0f - 80.0f) * t);
                b = int(14.0f + (16.0f - 14.0f) * t);
            }

            beginPath();
            circle(sx, sy, drawRadius);
            fillColor(Color(r, g, b, intensity * kMaxAlpha));
            fill();
        }
    }
}

void GoniometerWidget::drawTrailAsDots(Color dotColor, int count)
{
    // Draw oldest-first (index 0 in the collected arrays) so the newest
    // points paint on top of the trail.
    for (int i = 0; i < count; ++i)
    {
        const float t = 1.0f - float(ptAge_[i]) / float(std::max(1, visibleCount_ - 1)); // 1 = newest, 0 = oldest
        const float alpha = std::pow(std::max(0.0f, t), 1.6f);
        if (alpha < 0.02f) continue;

        beginPath();
        circle(ptSx_[i], ptSy_[i], 1.6f);
        fillColor(dotColor.withAlpha(alpha));
        fill();
    }
}

void GoniometerWidget::drawTrailAsLines(Color dotColor, int count)
{
    if (count < 2) return;

    // Clamped index -- duplicates the nearest real endpoint for the
    // Catmull-Rom control points the two boundary segments would
    // otherwise need one point past either end of the collected array.
    const auto idx = [count](int i) { return std::max(0, std::min(count - 1, i)); };

    // One segment per consecutive pair of *drawn* points (after
    // Resolution thinning -- not necessarily adjacent ring-buffer slots
    // any more), oldest-first so newer segments paint on top. Each
    // segment's alpha follows its newer endpoint's age, so a segment
    // fades in step with the point it's leading into.
    for (int i = 0; i < count - 1; ++i)
    {
        const float t = 1.0f - float(ptAge_[i + 1]) / float(std::max(1, visibleCount_ - 1));
        const float alpha = std::pow(std::max(0.0f, t), 1.6f);
        if (alpha < 0.02f) continue;

        beginPath();
        moveTo(ptSx_[i], ptSy_[i]);
        if (smoothLines_)
        {
            // Catmull-Rom-to-Bezier: the standard construction for an
            // interpolating spline segment between P1 and P2 that also
            // passes smoothly through their neighbors P0/P3 (clamped to
            // the nearest real point at either end of the trail, so the
            // two boundary segments still curve sensibly rather than
            // needing a point that doesn't exist). Built directly in
            // already-screen-mapped coordinates -- valid because the
            // side/mid -> screen mapping is affine, so applying it before
            // or after this linear-combination construction gives the
            // same result.
            const int i0 = idx(i - 1), i2 = i + 1, i3 = idx(i + 2);
            const float c1x = ptSx_[i] + (ptSx_[i2] - ptSx_[i0]) / 6.0f;
            const float c1y = ptSy_[i] + (ptSy_[i2] - ptSy_[i0]) / 6.0f;
            const float c2x = ptSx_[i2] - (ptSx_[i3] - ptSx_[i]) / 6.0f;
            const float c2y = ptSy_[i2] - (ptSy_[i3] - ptSy_[i]) / 6.0f;
            bezierTo(c1x, c1y, c2x, c2y, ptSx_[i2], ptSy_[i2]);
        }
        else
        {
            lineTo(ptSx_[i + 1], ptSy_[i + 1]);
        }
        strokeColor(dotColor.withAlpha(alpha));
        strokeWidth(1.2f);
        stroke();
    }
}

END_NAMESPACE_DGL
