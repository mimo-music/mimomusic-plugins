// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "ModelEditorHost.hpp"
#include "EditorSubWidget.hpp"

#include <chrono>
#include <string>

START_NAMESPACE_DGL

// Displays and edits one of the four coefficient matrices (A, B, C or D) as
// a grid of numeric cells. Sizing (rows x cols) is derived every frame from
// IModelEditorHost::getOrder(), so it automatically tracks matrix-size
// (order) changes made elsewhere in the UI.
//
// Interaction, kept deliberately simple for a first draft:
//   - click a cell to start editing it, pre-filled with its current value
//     (Enter/Tab commits, Escape cancels, Backspace edits, arrow keys jump
//     to the neighboring cell, digits/'.'/'-' accepted);
//   - double-click a cell to start editing it with an *empty* buffer, so
//     typing replaces the value outright instead of appending to it;
//   - scroll wheel over a cell nudges its value proportionally.
// A click-drag fader-style adjustment is a natural follow-up once desired,
// left out here to avoid depending on pointer-capture behaviour across
// widget bounds that this first draft does not verify.
class MatrixGridWidget : public EditorSubWidget
{
public:
    MatrixGridWidget(NanoTopLevelWidget* parent, ssgui::IModelEditorHost* host, ssgui::MatrixKind kind,
                      const char* title);

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    void getDims(int& rows, int& cols) const;
    bool cellAt(double x, double y, int& row, int& col) const;

    ssgui::IModelEditorHost* host_;
    ssgui::MatrixKind kind_;
    std::string title_;

    int hoverRow_ = -1, hoverCol_ = -1;
    int pressRow_ = -1, pressCol_ = -1;

    // Double-click detection: a completed click is compared against the
    // previous one on timing + position.
    int lastClickRow_ = -1, lastClickCol_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};
};

END_NAMESPACE_DGL
