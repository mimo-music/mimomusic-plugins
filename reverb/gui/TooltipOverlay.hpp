// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// TooltipOverlay.hpp
//
// Hover descriptions for buttons and fields -- a single, always-on-top
// overlay per plugin editor that shows a short description near whatever widget
// the pointer is currently resting on, after a short delay (so it
// doesn't flicker in and out while just moving across the toolbar).
//
// Deliberately NOT built into SimpleButton/ParamStripWidget/etc.
// themselves: every widget in this project family is constructed once,
// up front, in whatever order the owning UI's constructor happens to
// build them in -- z-order (and therefore paint order) follows
// construction order (DGL paints subwidgets in construction order,
// topmost last -- see UICompressorStereo.cpp's own comment on
// FOR_EACH_SUBWIDGET/_INV). A tooltip has to paint *after* every other
// widget regardless of which one triggered it, which means it has to be
// its own widget, constructed last, not a responsibility any individual
// button could discharge for itself. It never actually claims a mouse
// event (onMouse()/onMotion()/onScroll() all return false unconditionally)
// so every click, drag, and scroll still reaches whatever's underneath
// exactly as before -- this widget only ever draws on top, never
// intercepts.
//
// Usage, from the owning UI's constructor (after every widget it wants
// tooltips for has already been constructed): build this last, then
// `tooltip_->addTarget(btnBypass_, "Bypass: ...");` once per widget (or
// `addFieldTarget()` for a multi-field widget like ParamStripWidget --
// see its own comment). From the owning UI's own uiIdle(): `tooltip_->tick();` once per call,
// unconditionally -- see tick()'s own comment for why a periodic nudge is
// needed at all (DGL has no per-widget timer of its own).
// From the owning UI's own layout(): resize this to cover the whole
// window (`tooltip_->setSize(getWidth(), getHeight());`) so its own hit
// testing/painting has the full window available regardless of what's
// hovered.

#pragma once

#include "EditorSubWidget.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

START_NAMESPACE_DGL

class TooltipOverlay : public EditorSubWidget
{
public:
    explicit TooltipOverlay(NanoTopLevelWidget* parent);

    // Registers a hover description for `target`, looked up by that
    // widget's own *current* absolute position/size every time the
    // pointer moves -- correct across layout()/resize without needing to
    // be re-registered. Call once per widget; last-registered wins on any
    // overlap (mirrors DGL's own "last constructed = topmost" convention).
    // SubWidget rather than plain Widget -- getAbsoluteX()/getAbsoluteY()
    // are SubWidget-specific, and every widget in this project family
    // (SimpleButton, ParamStripWidget, LevelMeterWidget, ...) is one.
    void addTarget(SubWidget* target, std::string text);

    // Same as addTarget(), for a widget whose own hover text depends on
    // *where within it* the pointer is: ParamStripWidget is one widget with
    // several independently-described fields, not several widgets.
    // `describe` is called with the pointer position in `target`'s own
    // local coordinates (i.e. relative to its own top-left) each time the
    // pointer moves inside its bounds; an empty return means "nothing to
    // show right here" (e.g. the gap between two fields) without falling
    // through to whatever's visually behind this widget.
    void addFieldTarget(SubWidget* target, std::function<std::string(double localX, double localY)> describe);

    // Advances the hover-delay timer and repaints once it's time for the
    // tooltip to actually appear -- call once per owning UI's own
    // uiIdle() tick. onMotion() alone can't do this: the delay has to
    // elapse while the pointer is *not moving*, so something has to keep
    // checking the clock on its own, and DPF/DGL gives widgets no timer
    // of their own besides the host's own periodic uiIdle() callback
    // (same mechanism SidechainScopeWidget::tickLevelHistory() already
    // relies on for its own per-tick cadence).
    void tick();

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    struct Entry
    {
        SubWidget* target;
        std::string text;                                          // used when describe is empty
        std::function<std::string(double, double)> describe;        // used instead of text when set (addFieldTarget())
    };
    std::vector<Entry> entries_;

    struct Resolved
    {
        const Entry* entry = nullptr;
        std::string text;
    };
    // Topmost entry (and its resolved text) under (x,y), or {nullptr, ""}.
    // Re-resolved on every onMotion() -- not just which *widget*, but
    // which *text* is current, since a describe() callback can change its
    // answer as the pointer moves within the same still-hovered widget
    // (e.g. from one ParamStripWidget field to the next).
    Resolved resolve(double x, double y) const;

    const Entry* hoverEntry_ = nullptr;
    std::string hoverText_;
    std::chrono::steady_clock::time_point hoverSince_{};

    // How long the pointer has to rest before a tooltip shows -- 1s
    // (raised from an initial 500ms, which popped up too eagerly).
    static constexpr int kDelayMs = 1000;
    static constexpr float kMaxTextWidth = 260.0f;

    DISTRHO_DECLARE_NON_COPYABLE(TooltipOverlay)
};

END_NAMESPACE_DGL
