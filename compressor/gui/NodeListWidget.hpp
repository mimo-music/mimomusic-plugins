// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// NodeListWidget.hpp
//
// The Contour plugin's numeric node list -- the precise-entry counterpart
// to ContourGraphWidget's direct manipulation, modeled on the sibling
// ../statespace EQ's BandListWidget: every node's values are visible and
// editable, so e.g. lowering a point by exactly 1dB is a typed edit rather
// than a (hard to keep perfectly vertical) mouse drag.
//
// One row per curve node: index, hysteresis direction (click cycles
// Both / Rising / Falling), In dB, Out dB, and the two components of the
// node's Bezier support point (Handle dX / dY -- see CurvePoint's own
// comment; 0/0 is a corner node), plus a delete button. The numeric fields
// follow the same conventions as every other numeric field in this project
// (ParamStripWidget): click to type (pre-filled), double-click to clear
// first, Enter/Tab commit, Escape cancels, scroll wheel over a field nudges
// it (Alt = 10x finer), and press-and-drag a field vertically to change it
// like a fader (up = larger; a press that never moves past a small dead zone
// is still a click, so click-to-type keeps working). The two fixed endpoints can't change X, be
// re-tagged or be removed (TransferCurve refuses those itself; the row just
// shows them dimmed instead of offering a control that does nothing).
// A footer button adds a node in the middle of the widest gap, sitting
// exactly on the current curve so the sound doesn't jump until it's edited.
//
// Holds its own copy of the curve like ContourGraphWidget does (setCurve()
// from the owner, edits reported back through setOnCurveEdited()) and keeps
// a selected row in sync with the graph's selection through
// setSelected()/setOnSelectionChanged().
//
// Keyboard events reach the top-level UI, not this subwidget (same
// convention as ParamStripWidget) -- the owning UI checks isEditing() and
// forwards appendChar()/backspace()/commitEdit()/cancelEdit().

#pragma once

#include "NanoVG.hpp"

#include "../plugin/dsp/CurveDsp.hpp"

#include <chrono>
#include <functional>
#include <string>

START_NAMESPACE_DGL

class NodeListWidget : public NanoSubWidget
{
public:
    explicit NodeListWidget(Widget* parent);

    void setCurve(const sscomp::TransferCurve& curve);
    const sscomp::TransferCurve& curve() const { return curve_; }
    // Caption above the column titles, e.g. "Nodes -- Left / Mid".
    void setTitle(const char* title) { title_ = title; repaint(); }

    void setSelected(int index) { selected_ = index; repaint(); }
    int selected() const { return selected_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }
    void setOnCurveEdited(std::function<void(const sscomp::TransferCurve&)> cb) { onCurveEdited_ = std::move(cb); }
    // The threshold of the curve being listed -- needed so editing a node
    // that is linked to it moves it too (X only). Fires back through
    // setOnThresholdChanged().
    void setThreshold(float db) { thresholdDb_ = db; }
    void setOnThresholdChanged(std::function<void(float)> cb) { onThresholdChanged_ = std::move(cb); }

    bool isEditing() const { return editRow_ >= 0; }
    void appendChar(char c);
    void backspace();
    void commitEdit();
    void cancelEdit();

    // Tooltip for whichever column/footer is under local (x,y), or empty.
    // Used by TooltipOverlay::addFieldTarget().
    std::string cellTooltipAt(double x, double y) const;

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    enum class Field { In = 0, Out, HandleX, HandleY };

    struct Columns
    {
        float numX = 0, numW = 0;
        float dirX = 0, dirW = 0;
        float fieldX[4] = {}, fieldW = 0;
        float delX = 0, delW = 0;
        float total = 0;
    };
    Columns columns() const;

    static constexpr float kTitleH = 18.0f;
    static constexpr float kHeaderH = 16.0f;
    static constexpr float kRowH = 24.0f;
    static constexpr float kFooterH = 28.0f;

    int maxVisibleRows() const;
    // Row under local y -> node index, or -1 (title/header/footer/beyond).
    int rowAt(double y) const;
    float rowY(int index) const { return kTitleH + kHeaderH + float(index - scrollOffset_) * kRowH; }
    bool inFooter(double y) const { return y >= double(getHeight()) - double(kFooterH); }

    bool fieldEditable(int row, Field f) const;
    double fieldValue(int row, Field f) const;
    void applyField(int row, Field f, double value);
    void beginEdit(int row, Field f, bool clearValue);
    void fireEdited() { if (onCurveEdited_) onCurveEdited_(curve_); }
    void selectRow(int row) { selected_ = row; if (onSelectionChanged_) onSelectionChanged_(row); }
    void addNodeInWidestGap();

    sscomp::TransferCurve curve_;
    std::string title_ = "Nodes";
    int selected_ = -1;
    int hoverRow_ = -1;
    int scrollOffset_ = 0;

    std::function<void(int)> onSelectionChanged_;
    std::function<void(const sscomp::TransferCurve&)> onCurveEdited_;
    std::function<void(float)> onThresholdChanged_;
    float thresholdDb_ = -18.0f;

    // Mid-press target, resolved on release like BandListWidget does.
    enum class PressTarget { None, Dir, Field, Delete, Row, Add };
    PressTarget pressTarget_ = PressTarget::None;
    int pressRow_ = -1;
    Field pressField_ = Field::In;

    int editRow_ = -1;
    Field editField_ = Field::In;
    std::string editBuffer_;

    // Click-and-drag fader state, live only between a press on an editable
    // numeric field and its release -- same idea (and dead zone) as
    // ParamStripWidget. The value is always start + delta-from-the-press, so
    // clamping at a neighbor/limit never accumulates drift and dragging back
    // recovers exactly.
    bool dragging_ = false;
    double dragStartY_ = 0.0;
    double dragStartValue_ = 0.0;
    static constexpr double kDragDeadZonePx = 3.0;
    static constexpr double kDbPerPixel = 0.25;     // Alt = 10x finer

    int lastClickKey_ = -1; // row*4 + field, for double-click-to-clear
    std::chrono::steady_clock::time_point lastClickTime_{};
};

END_NAMESPACE_DGL
