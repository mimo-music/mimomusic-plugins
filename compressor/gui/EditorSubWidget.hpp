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

#pragma once

#include "NanoVG.hpp"

START_NAMESPACE_DGL

class EditorSubWidget : public NanoSubWidget
{
public:
    explicit EditorSubWidget(NanoTopLevelWidget* parent) : NanoSubWidget(parent) {}

    void paintIntoEditorFrame()
    {
        save();
        translate(float(getAbsoluteX()), float(getAbsoluteY()));
        onNanoDisplay();
        restore();
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
