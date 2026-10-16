// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// NonlinearPanelWidget.hpp
//
// GUI for the optional state-dependent gain stage (NonlinearSchedule, see
// plugin/dsp/StateSpaceTypes.hpp and docs/ARCHITECTURE.md's "NonlinearSchedule"
// section) -- the mechanism that lets this plugin represent compression
// without giving up the fixed, stability-guarded A/B recursion. Everything
// this widget edits is either a real model field (enabled/attack/release/
// the 25 curve knots, all persisted in the .ssjson preset) or a transient
// convenience field (threshold/ratio/makeup, baked into the knots on demand
// by "Apply Curve" -- see ModelEditorHost.hpp's MatrixKind::Nonlinear
// comment for why those aren't part of the saved model).
//
// Layout, top to bottom:
//   - a title bar with the Enabled toggle;
//   - one row of small editable numeric fields -- attack ms, release ms,
//     then threshold/ratio/makeup feeding the Apply Curve button -- all 5
//     addressed as columns of the virtual ssgui::MatrixKind::Nonlinear
//     "matrix", so they reuse *exactly* the same click-to-edit (pre-filled),
//     double-click-to-clear, Enter/Tab/Escape, arrow-key-navigate and
//     scroll-to-nudge machinery MatrixGridWidget uses for A/B/C/D, through
//     the same IModelEditorHost;
//   - the gain curve itself: level (dB, x-axis) -> gain (dB, y-axis),
//     drawn as a connected line through its 25 knots. Click (and drag)
//     anywhere in the graph to sculpt it directly -- the nearest knot (by
//     x position) snaps its gain to the clicked/dragged y position: this
//     is a first-draft simplification (see MatrixGridWidget's own
//     click-drag note) rather than per-knot numeric text entry, which
//     would need a much larger interaction surface for 25 individual
//     values. Double-click a knot to reset it to 0dB (flat/transparent)
//     instead of dragging it, the same "double-click resets" convention
//     every numeric field elsewhere in this project follows. Scroll
//     wheel over the graph nudges whichever knot is nearest the cursor.

#pragma once

#include "ModelEditorHost.hpp"
#include "EditorSubWidget.hpp"

#include <chrono>
#include <functional>
#include <string>

START_NAMESPACE_DGL

class NonlinearPanelWidget : public EditorSubWidget
{
public:
    NonlinearPanelWidget(NanoTopLevelWidget* parent, ssgui::IModelEditorHost* host);

    // Fired when "Apply Curve" is clicked -- the caller (UIStateSpace) owns
    // the actual threshold/ratio/makeup values (read back via the host's
    // getMatrixValue(Nonlinear, 0, 2/3/4)) and bakes them into the 25 curve
    // knots itself.
    void setOnApplyCurve(std::function<void()> cb) { onApplyCurve_ = std::move(cb); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    int knotAtX(float px) const;             // nearest knot index for a pointer x pixel within the graph
    double yToGainDb(float py) const;         // pointer y pixel (within the graph) -> gain dB
    float gainDbToY(double gainDb) const;     // gain dB -> pixel y (within the graph)
    void setKnotFromPoint(double px, double py); // shared by click and drag-continuation
    bool fieldAt(double x, double y, int& col) const;

    ssgui::IModelEditorHost* host_;
    std::function<void()> onApplyCurve_;

    // Hit regions, recomputed every draw and tested in onMouse/onScroll --
    // same pattern as BodePlotWidget's legend swatches/LOG-LIN toggle.
    float enableX_ = 0, enableY_ = 0, enableW_ = 0, enableH_ = 0;
    float fieldX_[5] = {}, fieldY_ = 0, fieldW_[5] = {}, fieldH_ = 0;
    float applyX_ = 0, applyY_ = 0, applyW_ = 0, applyH_ = 0;
    float graphX_ = 0, graphY_ = 0, graphW_ = 0, graphH_ = 0;
    static constexpr double kGainAxisMinDb = -48.0, kGainAxisMaxDb = 24.0; // fixed editing range for the y-axis

    int pressField_ = -1;   // field column currently pressed (-1 = none)
    bool pressedEnable_ = false, pressedApply_ = false; // act on release-inside, like SimpleButton
    bool draggingCurve_ = false;
    int hoverKnot_ = -1;

    // Double-click detection for the 5 numeric fields, same convention as
    // MatrixGridWidget's.
    int lastClickField_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};

    // Double-click detection for curve knots (resets to 0dB) -- a
    // separate tracker from the numeric fields' above, since they're
    // different kinds of targets (a field column vs. a knot index) that
    // could otherwise collide.
    int lastClickKnot_ = -1;
    std::chrono::steady_clock::time_point lastClickKnotTime_{};
};

END_NAMESPACE_DGL
