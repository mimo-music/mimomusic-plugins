// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ParamStripWidget.hpp
//
// A row of click-to-edit numeric fields, ported (with only cosmetic
// changes) from the sibling ../compressor project's widget of the same
// name, to reuse its interaction model here. Generalizes NonlinearPanelWidget.cpp's field-row interaction
// (click to edit pre-filled, double-click clears first, Enter/Tab commits,
// Escape cancels, scroll wheel nudges) into one reusable class taking a
// caller-supplied field list, rather than hand-copying that state machine
// once per field the way this project's own UIEq toolbar fields did before
// being unified into one checkInlineDoubleClick() helper.
//
// Click-and-drag fader adjustment, the one interaction this project's own
// matrix cells and NonlinearPanelWidget knots deliberately left out ("DPF's
// pointer-capture behavior across widget bounds during a drag wasn't
// verified in this environment"): press on a field and drag vertically --
// up increases, down decreases. That concern is about a drag straying
// *outside the whole plugin window*, not outside this widget's own bounds
// within it -- DGL's subwidget dispatch hands every widget every mouse
// event regardless of pointer position (see SimpleButton's own comment on
// this), so a widget that received the press keeps receiving motion for
// the rest of the drag even once the pointer leaves its own rectangle, as
// long as it stays inside the window -- which is all a vertical fader drag
// on a fixed-position field needs. A small dead zone distinguishes "drag"
// from "click": a press that never moves past it, on release, falls
// through to the existing click / double-click-to-edit logic instead.
//
// Keyboard events (character input / Enter / Escape / Backspace) reach the
// top-level UI, not this subwidget, matching this project's convention
// (UIStateSpace/UIEq centralize all cell-edit keyboard handling) -- the
// owning UI checks isEditing() and forwards.

#pragma once

#include "EditorSubWidget.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

START_NAMESPACE_DGL

struct ParamFieldSpec
{
    std::string label;
    std::string unit;
    float minValue = 0.0f, maxValue = 1.0f, defaultValue = 0.0f;
    float nudgeStep = 0.1f;     // linear scroll-wheel step, in the field's own units
    bool nudgeMultiplicative = false; // true for wide-log-range fields (Attack/Release): scroll multiplies/divides instead of adding
    int decimals = 1;
    uint32_t fieldId = 0;      // opaque id passed back through the getter/setter callbacks below -- not necessarily a DPF Parameter index

    // Hover description for this field (ported from the sibling
    // ../compressor project's own identical addition) -- shown via
    // TooltipOverlay, which calls
    // ParamStripWidget::fieldTooltipAt() to look this up. Naturally
    // mode-adaptive with zero extra runtime logic as long as the caller's
    // own field-list-building function already reads whatever mode/state
    // it needs (same as every other per-field property here). Empty
    // means no tooltip for this field.
    std::string tooltip;

    // Ported from ../compressor's own later addition to this same widget:
    // when set, replaces this field's own plain
    // single-color label text with labelHigh (drawn first, tinted green
    // as the value approaches maxValue / red as it approaches minValue)
    // and labelLow (drawn second, the exact opposite tint) instead of the
    // plain `label` string -- e.g. "Wet" (green at 100%, red at 0%) /
    // "Dry" (the reverse). The value number itself always stays this
    // widget's one normal-colored piece of text, regardless of this flag
    // -- see ParamStripWidget.cpp's onNanoDisplay() for the exact
    // fade formula, ported unchanged.
    bool bipolarLabel = false;
    std::string labelHigh, labelLow; // e.g. "Wet", "Dry" -- only used when bipolarLabel is true
};

class ParamStripWidget : public EditorSubWidget
{
public:
    explicit ParamStripWidget(NanoTopLevelWidget* parent);

    void setFields(std::vector<ParamFieldSpec> fields) { fields_ = std::move(fields); }
    void setGetter(std::function<float(uint32_t)> getter) { getter_ = std::move(getter); }
    // Called both for a committed text edit and for a scroll-wheel/drag
    // nudge -- the caller just forwards to wherever that field's value
    // actually lives (a DPF setParameterValue(), a model field + engine
    // push, whatever's appropriate).
    void setSetter(std::function<void(uint32_t, float)> setter) { setter_ = std::move(setter); }

    bool isEditing() const { return editField_ >= 0; }
    void appendChar(char c);
    void backspace();
    void commitEdit();
    void cancelEdit();

    // Returns the field under local (x,y) own tooltip text, or empty if
    // there's no field there (a gap) or that field has none set. Used by
    // TooltipOverlay::addFieldTarget().
    std::string fieldTooltipAt(double x, double y) const;

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    bool fieldAt(double x, double y, int& index) const;
    void beginEdit(int index, bool clearValue);
    std::string formatValue(const ParamFieldSpec& f, float value) const;
    void applyDrag(int index, double dyPixels);

    std::vector<ParamFieldSpec> fields_;
    std::function<float(uint32_t)> getter_;
    std::function<void(uint32_t, float)> setter_;

    std::vector<float> fieldX_, fieldW_;
    float fieldY_ = 0, fieldH_ = 0;

    int pressField_ = -1;
    int hoverField_ = -1;
    int editField_ = -1;
    std::string editBuffer_;

    // Click-and-drag fader state, live only between a press and its release.
    bool dragging_ = false;          // crossed the dead zone -- a drag, not a click
    double dragStartY_ = 0.0;
    float dragStartValue_ = 0.0f;
    static constexpr double kDragDeadZonePx = 3.0;
    static constexpr double kDragPixelsForFullRange = 200.0; // drag this many px to sweep the field's whole range

    int lastClickField_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};
};

END_NAMESPACE_DGL
