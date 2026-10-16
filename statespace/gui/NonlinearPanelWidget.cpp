// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "NonlinearPanelWidget.hpp"

#include "../plugin/dsp/StateSpaceTypes.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

using ssgui::MatrixKind;
using ssfilter::GainCurve;

namespace {
constexpr const char* kFieldLabels[5] = { "Attack ms", "Release ms", "Threshold dB", "Ratio :1", "Makeup dB" };
}

NonlinearPanelWidget::NonlinearPanelWidget(NanoTopLevelWidget* parent, ssgui::IModelEditorHost* host)
    : EditorSubWidget(parent), host_(host)
{
    // The context is shared with the editor (see EditorSubWidget.hpp),
    // which has usually loaded the bundled font already;
    // loadSharedResources() is idempotent, so this only makes sure.
    loadSharedResources();
}

// ---- coordinate mapping (graph-local pixels <-> curve dB values) --------

int NonlinearPanelWidget::knotAtX(float px) const
{
    if (graphW_ <= 0.0f) return 0;
    const double t = double(px - graphX_) / double(graphW_);
    const double levelDb = GainCurve::kMinDb + t * (GainCurve::kMaxDb - GainCurve::kMinDb);
    const double knotF = (levelDb - GainCurve::kMinDb) / GainCurve::kStepDb;
    return std::max(0, std::min(GainCurve::kNumKnots - 1, int(std::lround(knotF))));
}

double NonlinearPanelWidget::yToGainDb(float py) const
{
    if (graphH_ <= 0.0f) return 0.0;
    const double t = double(py - graphY_) / double(graphH_); // 0 at top, 1 at bottom
    const double gainDb = kGainAxisMaxDb - t * (kGainAxisMaxDb - kGainAxisMinDb);
    return std::max(kGainAxisMinDb, std::min(kGainAxisMaxDb, gainDb));
}

float NonlinearPanelWidget::gainDbToY(double gainDb) const
{
    const double t = (kGainAxisMaxDb - gainDb) / (kGainAxisMaxDb - kGainAxisMinDb);
    return graphY_ + float(t) * graphH_;
}

void NonlinearPanelWidget::setKnotFromPoint(double px, double py)
{
    const int knot = knotAtX(float(px));
    const double gainDb = yToGainDb(float(py));
    host_->setNonlinearKnotGainDb(knot, gainDb);
    hoverKnot_ = knot;
}

bool NonlinearPanelWidget::fieldAt(double x, double y, int& col) const
{
    if (y < fieldY_ || y >= fieldY_ + fieldH_) return false;
    for (int c = 0; c < 5; ++c)
        if (x >= fieldX_[c] && x < fieldX_[c] + fieldW_[c]) { col = c; return true; }
    return false;
}

// ---- drawing --------------------------------------------------------------

void NonlinearPanelWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());
    constexpr float pad = 6.0f;
    constexpr float titleH = 22.0f;
    constexpr float fieldsH = 26.0f;

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(26, 28, 33));
    fill();

    const bool enabled = host_->getNonlinearEnabled();

    // -- title row: label + Enabled toggle -----------------------------
    fillColor(Color(210, 214, 222));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(12.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    text(pad, titleH * 0.5f, "Dynamics (state-dependent gain stage)", nullptr);

    enableW_ = 88.0f; enableH_ = titleH - 4.0f;
    enableX_ = w - pad - enableW_; enableY_ = 2.0f;
    beginPath();
    roundedRect(enableX_, enableY_, enableW_, enableH_, 3.0f);
    fillColor(enabled ? Color(70, 150, 90) : Color(58, 62, 70));
    fill();
    strokeColor(Color(20, 20, 22));
    strokeWidth(1.0f);
    stroke();
    fillColor(Color(230, 232, 236));
    fontSize(12.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(enableX_ + enableW_ * 0.5f, enableY_ + enableH_ * 0.5f + 1.0f,
         enabled ? "Enabled" : "Disabled", nullptr);

    // -- numeric fields row: attack/release/threshold/ratio/makeup ------
    fieldY_ = titleH + 3.0f;
    fieldH_ = fieldsH;
    const float fieldGap = 6.0f;
    const float applyW = 84.0f;
    const float totalFieldsW = w - 2.0f * pad - applyW - fieldGap;
    const float oneFieldW = (totalFieldsW - 4.0f * fieldGap) / 5.0f;

    float fx = pad;
    for (int c = 0; c < 5; ++c)
    {
        fieldX_[c] = fx;
        fieldW_[c] = oneFieldW;
        fx += oneFieldW + fieldGap;

        std::string liveText;
        const bool editing = host_->isEditingCell(MatrixKind::Nonlinear, 0, c, liveText);

        beginPath();
        roundedRect(fieldX_[c] + 0.5f, fieldY_ + 0.5f, fieldW_[c] - 1.0f, fieldH_ - 1.0f, 3.0f);
        fillColor(editing ? Color(235, 195, 60) : Color(34, 37, 43));
        fill();
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();

        char buf[32];
        const char* toShow;
        if (editing) { toShow = liveText.c_str(); }
        else
        {
            std::snprintf(buf, sizeof(buf), "%.4g", host_->getMatrixValue(MatrixKind::Nonlinear, 0, c));
            toShow = buf;
        }
        fillColor(editing ? Color(20, 20, 24) : Color(220, 223, 228));
        fontSize(12.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(fieldX_[c] + fieldW_[c] * 0.5f, fieldY_ + fieldH_ * 0.38f, toShow, nullptr);

        fillColor(Color(140, 144, 152));
        fontSize(9.5f);
        text(fieldX_[c] + fieldW_[c] * 0.5f, fieldY_ + fieldH_ * 0.82f, kFieldLabels[c], nullptr);
    }

    applyX_ = fx; applyY_ = fieldY_; applyW_ = applyW; applyH_ = fieldH_;
    beginPath();
    roundedRect(applyX_ + 0.5f, applyY_ + 0.5f, applyW_ - 1.0f, applyH_ - 1.0f, 3.0f);
    fillColor(Color(58, 62, 70));
    fill();
    strokeColor(Color(20, 20, 22));
    strokeWidth(1.0f);
    stroke();
    fillColor(Color(230, 232, 236));
    fontSize(11.5f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(applyX_ + applyW_ * 0.5f, applyY_ + applyH_ * 0.5f + 1.0f, "Apply Curve", nullptr);

    // -- gain curve graph -------------------------------------------------
    graphX_ = pad;
    graphY_ = fieldY_ + fieldH_ + 8.0f;
    graphW_ = w - 2.0f * pad;
    graphH_ = h - graphY_ - pad;
    if (graphH_ <= 4.0f) return;

    beginPath();
    rect(graphX_, graphY_, graphW_, graphH_);
    fillColor(Color(18, 19, 23));
    fill();
    strokeColor(Color(48, 51, 58));
    strokeWidth(1.0f);
    stroke();

    // Gridlines: level every 24dB, gain every 12dB; the 0dB gain line
    // (transparent/unity) drawn brighter since it's the most meaningful
    // reference on this particular graph.
    fontSize(9.0f);
    for (double lv = GainCurve::kMinDb; lv <= GainCurve::kMaxDb + 0.01; lv += 24.0)
    {
        const float t = float((lv - GainCurve::kMinDb) / (GainCurve::kMaxDb - GainCurve::kMinDb));
        const float x = graphX_ + t * graphW_;
        beginPath();
        moveTo(x, graphY_);
        lineTo(x, graphY_ + graphH_);
        strokeColor(Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.0f", lv);
        text(x, graphY_ + graphH_ + 1.0f, buf, nullptr);
    }
    for (double g = kGainAxisMinDb; g <= kGainAxisMaxDb + 0.01; g += 12.0)
    {
        const float y = gainDbToY(g);
        beginPath();
        moveTo(graphX_, y);
        lineTo(graphX_ + graphW_, y);
        const bool isZero = std::fabs(g) < 0.01;
        strokeColor(isZero ? Color(90, 96, 106) : Color(40, 43, 50));
        strokeWidth(isZero ? 1.4f : 1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%+.0f", g);
        text(graphX_ - 3.0f, y, buf, nullptr);
    }

    // The curve line through all 25 knots.
    beginPath();
    for (int i = 0; i < GainCurve::kNumKnots; ++i)
    {
        const float t = float(i) / float(GainCurve::kNumKnots - 1);
        const float x = graphX_ + t * graphW_;
        const float y = gainDbToY(host_->getNonlinearKnotGainDb(i));
        if (i == 0) moveTo(x, y); else lineTo(x, y);
    }
    strokeColor(enabled ? Color(120, 210, 170) : Color(90, 94, 100));
    strokeWidth(2.0f);
    stroke();

    // Knot markers, the hovered/dragged one highlighted.
    for (int i = 0; i < GainCurve::kNumKnots; ++i)
    {
        const float t = float(i) / float(GainCurve::kNumKnots - 1);
        const float x = graphX_ + t * graphW_;
        const float y = gainDbToY(host_->getNonlinearKnotGainDb(i));
        const bool hot = (i == hoverKnot_);
        beginPath();
        circle(x, y, hot ? 4.0f : 2.5f);
        fillColor(hot ? Color(235, 195, 60) : (enabled ? Color(150, 225, 190) : Color(120, 124, 130)));
        fill();
    }

    if (!enabled)
    {
        fillColor(Color(140, 144, 152));
        fontSize(11.0f);
        textAlign(ALIGN_LEFT | ALIGN_TOP);
        text(graphX_ + 6.0f, graphY_ + 4.0f,
             "Disabled -- A/B/C/D are pure LTI; click \"Disabled\" above to enable this stage", nullptr);
    }
}

// ---- interaction ------------------------------------------------------

bool NonlinearPanelWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();

    const bool onEnable = x >= enableX_ && x < enableX_ + enableW_ && y >= enableY_ && y < enableY_ + enableH_;
    const bool onApply = x >= applyX_ && x < applyX_ + applyW_ && y >= applyY_ && y < applyY_ + applyH_;

    if (ev.press)
    {
        // Enable toggle and Apply Curve act on release-inside (like
        // SimpleButton), not press, so dragging off before releasing
        // cancels the click.
        if (onEnable) { pressedEnable_ = true; repaint(); return true; }
        if (onApply)  { pressedApply_ = true;  repaint(); return true; }

        // One of the 5 numeric fields.
        int col;
        if (fieldAt(x, y, col))
        {
            pressField_ = col;
            return true;
        }
        // The curve graph itself.
        if (x >= graphX_ && x < graphX_ + graphW_ && y >= graphY_ && y < graphY_ + graphH_)
        {
            const int knot = knotAtX(float(x));
            const auto now = std::chrono::steady_clock::now();
            constexpr std::chrono::milliseconds kDoubleClickWindow(400);
            const bool isDoubleClick = knot == lastClickKnot_ && (now - lastClickKnotTime_) < kDoubleClickWindow;
            if (isDoubleClick)
            {
                // Reset to 0dB (flat/transparent) rather than drag --
                // same "double-click resets" convention every numeric
                // field elsewhere in this project follows.
                host_->setNonlinearKnotGainDb(knot, 0.0);
                lastClickKnot_ = -1; // consumes the pair, like everywhere else
                hoverKnot_ = knot;
                repaint();
                return true;
            }
            lastClickKnot_ = knot;
            lastClickKnotTime_ = now;

            draggingCurve_ = true;
            setKnotFromPoint(x, y);
            repaint();
            return true;
        }
        return false;
    }

    // Release.
    if (pressedEnable_)
    {
        pressedEnable_ = false;
        if (onEnable) host_->setNonlinearEnabled(!host_->getNonlinearEnabled());
        repaint();
        return true;
    }
    if (pressedApply_)
    {
        pressedApply_ = false;
        if (onApply && onApplyCurve_) onApplyCurve_();
        repaint();
        return true;
    }
    if (draggingCurve_)
    {
        draggingCurve_ = false;
        return true;
    }
    if (pressField_ >= 0)
    {
        int col;
        const bool sameField = fieldAt(x, y, col) && col == pressField_;
        const int releasedField = pressField_;
        pressField_ = -1;
        if (sameField)
        {
            const auto now = std::chrono::steady_clock::now();
            constexpr std::chrono::milliseconds kDoubleClickWindow(400);
            const bool isDoubleClick = col == lastClickField_ && (now - lastClickTime_) < kDoubleClickWindow;

            host_->beginCellTextEdit(MatrixKind::Nonlinear, 0, col, isDoubleClick);

            lastClickField_ = isDoubleClick ? -1 : releasedField;
            lastClickTime_ = now;
        }
        return true;
    }
    return false;
}

bool NonlinearPanelWidget::onMotion(const MotionEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (draggingCurve_)
    {
        // Clamp to the graph bounds so a drag that strays outside the
        // widget (pointer-capture behavior across widget bounds isn't
        // verified in this environment -- see MatrixGridWidget's own
        // click-drag note) still produces a sensible, bounded result
        // rather than an out-of-range knot/gain.
        const double cx = std::max(double(graphX_), std::min(double(graphX_ + graphW_ - 1), x));
        const double cy = std::max(double(graphY_), std::min(double(graphY_ + graphH_ - 1), y));
        setKnotFromPoint(cx, cy);
        repaint();
        return true;
    }

    const int prevHover = hoverKnot_;
    if (x >= graphX_ && x < graphX_ + graphW_ && y >= graphY_ && y < graphY_ + graphH_)
        hoverKnot_ = knotAtX(float(x));
    else
        hoverKnot_ = -1;
    if (hoverKnot_ != prevHover) repaint();
    return false;
}

bool NonlinearPanelWidget::onScroll(const ScrollEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();
    if (x < graphX_ || x >= graphX_ + graphW_ || y < graphY_ || y >= graphY_ + graphH_) return false;

    const int knot = knotAtX(float(x));
    const double current = host_->getNonlinearKnotGainDb(knot);
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;
    // Alt+scroll = 10x finer -- see gui/eq/EqGraphWidget.cpp's identical
    // comment for the full reasoning, applied project-wide.
    const double step = 0.5 / ((ev.mod & kModifierAlt) ? 10.0 : 1.0); // 0.5 dB base step per notch
    host_->setNonlinearKnotGainDb(knot, current + dir * step);
    hoverKnot_ = knot;
    repaint();
    return true;
}

END_NAMESPACE_DGL
