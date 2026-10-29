// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// EditorSubWidget.hpp
//
// Base class of every widget in an editor, plus the editor's own drawing
// frame -- together they make scaled editors sharp.
//
// DPF scales an editor (automatic scaling, see the UI constructors) by
// drawing its logical-size NanoVG frame into the physical window. Its own
// top-level frame passes a device pixel ratio of 1, so NanoVG renders text
// glyphs and antialiased edges for a 1x screen and the GPU stretches them:
// blurry at 2x. The editors therefore replace that frame (UI::onDisplay())
// with one that passes the real ratio (editorPixelRatio()) -- and since
// DPF's own routine for drawing the child widgets into that frame is
// private, the children are drawn through paintIntoEditorFrame() here
// (paintEditorChildren()), exactly as DPF would: translated to their
// position, in the editor's shared context.
//
// A widget must derive from this class (not NanoSubWidget directly) or it
// is not drawn at all; it is constructed directly on its editor (a
// NanoTopLevelWidget*), which makes it share the editor's context.
//
// Pixel grid: at a zoom such as 1.5x, a widget at an odd logical position
// would start half a device pixel off, blurring every line and glyph in
// it -- some widgets sharp, others not, depending on where they sit and on
// the window size. paintIntoEditorFrame() therefore rounds the widget's
// origin to a whole device pixel (at most a third of a logical pixel, not
// visible, clicks unaffected). snapLine() and lineWidth() keep 1px lines
// on whole device pixels inside a widget.

#pragma once

#include "NanoVG.hpp"

#include <algorithm>
#include <cmath>

START_NAMESPACE_DGL

class EditorSubWidget : public NanoSubWidget
{
public:
    explicit EditorSubWidget(NanoTopLevelWidget* parent) : NanoSubWidget(parent) {}

    void paintIntoEditorFrame()
    {
        const float r = pixelRatio();
        save();
        translate(std::round(float(getAbsoluteX()) * r) / r, std::round(float(getAbsoluteY()) * r) / r);
        onNanoDisplay();
        restore();
    }

    // Device pixels per logical pixel (window size / editor size): 1
    // unzoomed, 1.5 or 2 zoomed -- the same value editorPixelRatio() gives
    // the editor's frame.
    float pixelRatio() const
    {
        const auto* const top = getTopLevelWidget();
        return top != nullptr && top->getWidth() > 0 ? float(getWindow().getWidth()) / float(top->getWidth()) : 1.0f;
    }

    // Stroke width `w` rounded to whole device pixels (at least one).
    float lineWidth(float w = 1.0f) const
    {
        const float r = pixelRatio();
        return std::max(1.0f, std::round(w * r)) / r;
    }

    // Where to put a horizontal or vertical line meant at local coordinate
    // `v` so that, with lineWidth(w), it covers whole device pixels: on a
    // pixel centre for an odd device width, on a pixel edge for an even one.
    float snapLine(float v, float w = 1.0f) const
    {
        const float r = pixelRatio();
        const int dw = int(std::max(1.0f, std::round(w * r)));
        return (dw % 2 != 0 ? std::floor(v * r) + 0.5f : std::round(v * r)) / r;
    }
};

// Physical pixels per logical pixel: DPF's zoom factor (window size /
// logical size) -- the scale factor, times any zoom from resizing.
template <class Editor>
inline float editorPixelRatio(Editor& ui)
{
    return ui.getWidth() > 0 ? float(ui.getWindow().getWidth()) / float(ui.getWidth()) : 1.0f;
}

// DPF hands the editor's own onMouse()/onMotion()/onScroll() the raw window
// coordinates (only the base TopLevelWidget handlers scale them for the
// child widgets). An editor that hit-tests areas it draws itself converts
// the event first -- and passes the raw one on to UI::onX(), which scales.
template <class Editor, class Event>
inline Event editorLogicalEvent(Editor& ui, const Event& raw)
{
    Event ev = raw;
    const double r = double(editorPixelRatio(ui));
    if (r > 0.0)
    {
        ev.pos = Point<double>(raw.pos.getX() / r, raw.pos.getY() / r);
        ev.absolutePos = Point<double>(raw.absolutePos.getX() / r, raw.absolutePos.getY() / r);
    }
    return ev;
}

// Every visible child, in construction order (later ones on top).
template <class Editor>
inline void paintEditorChildren(Editor& ui)
{
    for (auto* const child : ui.getChildren())
        if (child->isVisible())
            if (EditorSubWidget* const w = dynamic_cast<EditorSubWidget*>(child))
                w->paintIntoEditorFrame();
}

END_NAMESPACE_DGL
