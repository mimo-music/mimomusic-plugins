// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MotionWatcher.hpp
//
// An invisible widget covering the whole editor that only notes when the
// pointer last moved, clicked or scrolled over it -- for controls that hide
// themselves while the mouse is away (the Phase Scope's compact view).
//
// Needed because DGL never hands motion events to the top-level UI itself,
// only to its subwidgets, in reverse construction order and without a
// bounds check. Constructed after every other widget, this one sees each
// event first; it never claims one (all handlers return false), so every
// widget underneath still gets it exactly as before.

#pragma once

#include "EditorSubWidget.hpp"

#include <chrono>

START_NAMESPACE_DGL

class MotionWatcher : public EditorSubWidget
{
public:
    explicit MotionWatcher(NanoTopLevelWidget* parent) : EditorSubWidget(parent) {}

    // Seconds since the pointer last moved/clicked/scrolled over the editor.
    double secondsSinceActivity() const
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - lastActivity_).count();
    }

    void touch() { lastActivity_ = std::chrono::steady_clock::now(); }

protected:
    void onNanoDisplay() override {}
    bool onMouse(const MouseEvent&) override { touch(); return false; }
    bool onMotion(const MotionEvent&) override { touch(); return false; }
    bool onScroll(const ScrollEvent&) override { touch(); return false; }

private:
    std::chrono::steady_clock::time_point lastActivity_{};
};

END_NAMESPACE_DGL
