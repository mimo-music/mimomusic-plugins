// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "MatrixGridWidget.hpp"

#include "../plugin/dsp/StateSpaceTypes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

using ssgui::MatrixKind;

namespace {

constexpr int kHeatBaseR = 30, kHeatBaseG = 32, kHeatBaseB = 38; // "empty" neutral background

// Diverging heatmap: positive values fade from the neutral background
// toward warm amber/red, negative values toward cool cyan/blue, scaled by
// the largest magnitude currently present in the matrix -- so structure
// (which cells are big, which are near zero) stays visible regardless of
// the model's absolute scale, which matters most for bigger matrices where
// scanning raw numbers cell by cell doesn't scale.
Color heatmapColor(double value, double maxAbs, double& outIntensity)
{
    const double t = maxAbs > 1e-12 ? std::min(1.0, std::fabs(value) / maxAbs) : 0.0;
    outIntensity = t;
    if (value > 0.0)
        return Color(int(kHeatBaseR + (226 - kHeatBaseR) * t),
                     int(kHeatBaseG + (96  - kHeatBaseG) * t),
                     int(kHeatBaseB + (40  - kHeatBaseB) * t));
    if (value < 0.0)
        return Color(int(kHeatBaseR + (40  - kHeatBaseR) * t),
                     int(kHeatBaseG + (140 - kHeatBaseG) * t),
                     int(kHeatBaseB + (230 - kHeatBaseB) * t));
    return Color(kHeatBaseR, kHeatBaseG, kHeatBaseB);
}

} // namespace

MatrixGridWidget::MatrixGridWidget(NanoTopLevelWidget* parent, ssgui::IModelEditorHost* host,
                                    MatrixKind kind, const char* title)
    : EditorSubWidget(parent), host_(host), kind_(kind), title_(title)
{
    // The context is shared with the editor (see EditorSubWidget.hpp),
    // which has usually loaded the bundled font already;
    // loadSharedResources() is idempotent, so this only makes sure.
    loadSharedResources();
}

void MatrixGridWidget::getDims(int& rows, int& cols) const
{
    ssgui::computeMatrixDims(kind_, host_->getOrder(), rows, cols);
}

bool MatrixGridWidget::cellAt(double x, double y, int& row, int& col) const
{
    int rows, cols;
    getDims(rows, cols);
    if (rows <= 0 || cols <= 0) return false;

    constexpr float kHeaderH = 20.0f;
    const float gridH = float(getHeight()) - kHeaderH;
    if (y < kHeaderH || gridH <= 0.0f) return false;

    const float cellW = float(getWidth()) / float(cols);
    const float cellH = gridH / float(rows);
    col = int(x / cellW);
    row = int((y - kHeaderH) / cellH);
    return col >= 0 && col < cols && row >= 0 && row < rows;
}

void MatrixGridWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());
    constexpr float kHeaderH = 20.0f;

    // Panel background.
    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(26, 28, 33));
    fill();

    // Title bar.
    fillColor(Color(210, 214, 222));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(12.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    text(6.0f, kHeaderH * 0.5f, title_.c_str(), nullptr);

    int rows, cols;
    getDims(rows, cols);
    if (rows <= 0 || cols <= 0) return;

    const float gridH = h - kHeaderH;
    const float cellW = w / float(cols);
    const float cellH = gridH / float(rows);
    const bool tinyCells = cellW < 34.0f || cellH < 18.0f;

    // Heatmap normalization reference: largest magnitude currently in this
    // matrix (recomputed every frame -- cheap, same O(rows*cols) cost as
    // the render loop itself, and always reflects the live values).
    double maxAbs = 1e-9;
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
            maxAbs = std::max(maxAbs, std::fabs(host_->getMatrixValue(kind_, r, c)));

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            const float x = c * cellW;
            const float y = kHeaderH + r * cellH;

            std::string liveText;
            const bool editing = host_->isEditingCell(kind_, r, c, liveText);
            const bool hovered = (r == hoverRow_ && c == hoverCol_);
            const double value = host_->getMatrixValue(kind_, r, c);

            double intensity = 0.0;
            const Color heat = heatmapColor(value, maxAbs, intensity);

            beginPath();
            rect(x + 0.5f, y + 0.5f, cellW - 1.0f, cellH - 1.0f);
            // A distinct amber for "being edited" -- deliberately unlike
            // either heatmap direction (which use blue for negative) so an
            // active edit is never mistaken for a very negative value.
            fillColor(editing ? Color(235, 195, 60) : heat);
            fill();
            // Hovering outlines the cell rather than replacing its fill, so
            // the heatmap color (the actual information) stays visible.
            strokeColor(hovered && !editing ? Color(215, 218, 224) : Color(14, 15, 18));
            strokeWidth(hovered && !editing ? 1.6f : 1.0f);
            stroke();

            if (tinyCells && !editing) continue; // avoid unreadable clutter at high matrix orders

            char buf[32];
            const char* toShow;
            if (editing) { toShow = liveText.c_str(); }
            else
            {
                std::snprintf(buf, sizeof(buf), "%.4g", value);
                toShow = buf;
            }

            // Text stays readable whether the cell ended up dark (near
            // zero) or brightly saturated (near the matrix's peak value).
            if (editing)             fillColor(Color(20, 20, 24));
            else if (intensity > 0.55) fillColor(Color(20, 20, 24));
            else if (value == 0.0)   fillColor(Color(120, 124, 132));
            else                     fillColor(Color(225, 228, 234));
            fontSize(std::min(13.0f, cellH * 0.5f));
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(x + cellW * 0.5f, y + cellH * 0.5f, toShow, nullptr);
        }
    }
}

bool MatrixGridWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;

    int row = -1, col = -1;
    const bool onCell = cellAt(ev.pos.getX(), ev.pos.getY(), row, col);

    if (ev.press)
    {
        // DGL's subwidget dispatch hands every sibling the event regardless
        // of position; only claim it if it actually landed on one of ours.
        if (!onCell) return false;
        pressRow_ = row;
        pressCol_ = col;
        return true;
    }

    // Release: only act if *we* were the one that received the press --
    // and still reset our press state even if the release lands outside
    // any cell (pointer moved off the grid before releasing).
    if (pressRow_ < 0) return false;
    const bool sameCell = onCell && row == pressRow_ && col == pressCol_;
    pressRow_ = pressCol_ = -1;
    if (sameCell)
    {
        const auto now = std::chrono::steady_clock::now();
        constexpr std::chrono::milliseconds kDoubleClickWindow(400);
        const bool isDoubleClick = row == lastClickRow_ && col == lastClickCol_ &&
                                    (now - lastClickTime_) < kDoubleClickWindow;

        host_->beginCellTextEdit(kind_, row, col, isDoubleClick);

        // A double-click consumes the pair, so a third click starts a fresh
        // pair rather than being misread as yet another double-click.
        lastClickRow_ = isDoubleClick ? -1 : row;
        lastClickCol_ = isDoubleClick ? -1 : col;
        lastClickTime_ = now;
    }
    return true;
}

bool MatrixGridWidget::onMotion(const MotionEvent& ev)
{
    int row, col;
    if (!cellAt(ev.pos.getX(), ev.pos.getY(), row, col)) row = col = -1;
    if (row != hoverRow_ || col != hoverCol_) { hoverRow_ = row; hoverCol_ = col; repaint(); }
    return false;
}

bool MatrixGridWidget::onScroll(const ScrollEvent& ev)
{
    int row, col;
    if (!cellAt(ev.pos.getX(), ev.pos.getY(), row, col)) return false;

    const double current = host_->getMatrixValue(kind_, row, col);
    // Alt+scroll = 10x finer -- see gui/eq/EqGraphWidget.cpp's identical
    // comment for the full reasoning, applied project-wide.
    const double step = std::max(std::fabs(current) * 0.02, 0.001) / ((ev.mod & kModifierAlt) ? 10.0 : 1.0);
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;
    host_->setMatrixValue(kind_, row, col, current + dir * step);
    return true;
}

END_NAMESPACE_DGL
