// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// CursorReadout.hpp
//
// The pointer readout of the frequency graphs (EqGraphWidget,
// DynEqGraphWidget, DynEqGraphWidget2): while the pointer is over an empty
// spot of the graph, a thin vertical line marks its frequency, a short tick
// its gain, and a small label next to the pointer prints both -- exactly
// the frequency and gain a click there gives the new band. Over a band
// node the node's own readout takes over, so the two never show at once.
//
// The owning widget feeds the pointer position from onMotion() (track() /
// hide()) and decides itself whether the spot is "empty" (not over a node,
// a label, an open menu...) before calling draw(). The pointer leaving the
// window arrives as a motion far outside (dpf-patches/0003), which hides it.

#pragma once

#include "EditorSubWidget.hpp"

#include <algorithm>

START_NAMESPACE_DGL

class CursorReadout
{
public:
    // Both return true when the widget needs a repaint.
    bool track(float x, float y)
    {
        const bool changed = !visible_ || x != x_ || y != y_;
        visible_ = true;
        x_ = x;
        y_ = y;
        return changed;
    }
    bool hide()
    {
        const bool changed = visible_;
        visible_ = false;
        return changed;
    }

    bool visible() const { return visible_; }
    float x() const { return x_; }
    float y() const { return y_; }

    // Line, tick and label inside the graph rectangle (gx, gy, gw, gh).
    void draw(EditorSubWidget& vg, float gx, float gy, float gw, float gh, const char* label) const
    {
        if (!visible_) return;

        const float lx = vg.snapLine(x_);
        vg.beginPath();
        vg.moveTo(lx, gy);
        vg.lineTo(lx, gy + gh);
        vg.strokeColor(Color(255, 255, 255, 0.22f));
        vg.strokeWidth(vg.lineWidth());
        vg.stroke();

        constexpr float kTickHalfW = 7.0f;
        const float ty = vg.snapLine(y_);
        vg.beginPath();
        vg.moveTo(x_ - kTickHalfW, ty);
        vg.lineTo(x_ + kTickHalfW, ty);
        vg.strokeColor(Color(255, 255, 255, 0.55f));
        vg.strokeWidth(vg.lineWidth());
        vg.stroke();

        // Label to the upper right of the pointer; flipped left at the
        // graph's right edge and below the pointer at its top edge.
        vg.fontFace(NANOVG_DEJAVU_SANS_TTF);
        vg.fontSize(11.0f);
        Rectangle<float> bounds;
        vg.textBounds(0.0f, 0.0f, label, nullptr, bounds);
        constexpr float kPadX = 6.0f, kH = 17.0f, kGap = 10.0f;
        const float w = std::round(bounds.getWidth() + 2.0f * kPadX);
        float bx = x_ + kGap;
        if (bx + w > gx + gw) bx = x_ - kGap - w;
        bx = std::max(gx, bx);
        float by = y_ - kGap - kH;
        if (by < gy) by = y_ + kGap;
        bx = std::round(bx);
        by = std::round(by);

        vg.beginPath();
        vg.roundedRect(bx, by, w, kH, 3.0f);
        vg.fillColor(Color(20, 21, 25, 0.88f));
        vg.fill();
        vg.fillColor(Color(220, 223, 228));
        vg.textAlign(NanoVG::ALIGN_LEFT | NanoVG::ALIGN_MIDDLE);
        vg.text(bx + kPadX, std::round(by + kH * 0.5f) + 0.5f, label, nullptr);
    }

private:
    bool visible_ = false;
    float x_ = 0.0f, y_ = 0.0f;
};

END_NAMESPACE_DGL
