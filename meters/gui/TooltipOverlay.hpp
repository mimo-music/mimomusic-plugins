// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// TooltipOverlay.hpp
//
// A single, full-size, click-through NanoSubWidget added last (so it
// draws on top of every other widget) whose only job is showing one short
// hover description near whatever control the pointer is currently over.
// UIPhaseScope polls each interactive control's own
// hover state once per uiIdle() tick (see that function) and calls
// show()/hide() here accordingly, rather than this widget tracking hover
// itself -- it has no bounds of its own to hit-test against, it just
// renders whatever the owning UI tells it to.
//
// Deliberately never claims a mouse/motion event (onMouse/onMotion always
// return false): sized to cover the entire plugin window so it can anchor
// a tooltip anywhere on screen, but a full-size opaque-to-input widget
// sitting on top of every button/field would otherwise swallow every
// click in the whole UI.

#pragma once

#include "NanoVG.hpp"

#include <string>

START_NAMESPACE_DGL

class TooltipOverlay : public NanoSubWidget
{
public:
    explicit TooltipOverlay(Widget* parent);

    // anchorX/Y/W/H: the hovered control's own screen rect (absolute,
    // same coordinate space this overlay itself lives in) -- the tooltip
    // is drawn just below it, nudged back on-screen if it would otherwise
    // run past this widget's own right/bottom edge.
    void show(const std::string& text, float anchorX, float anchorY, float anchorW, float anchorH);
    void hide();

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent&) override { return false; }
    bool onMotion(const MotionEvent&) override { return false; }
    bool onScroll(const ScrollEvent&) override { return false; }

private:
    std::string text_;
    float anchorX_ = 0.0f, anchorY_ = 0.0f, anchorW_ = 0.0f, anchorH_ = 0.0f;
};

END_NAMESPACE_DGL
