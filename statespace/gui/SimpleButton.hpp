// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "EditorSubWidget.hpp"

#include <functional>
#include <string>

START_NAMESPACE_DGL

// A minimal push-button: solid rect, centered label, invokes a callback on
// click-release-inside. Deliberately does not use DPF's ButtonEventHandler
// mixin to keep this self-contained and easy to read for a first draft.
class SimpleButton : public EditorSubWidget
{
public:
    explicit SimpleButton(NanoTopLevelWidget* parent, const char* label);

    void setCallback(std::function<void()> cb) { callback_ = std::move(cb); }
    void setColors(Color bg, Color bgHover, Color fg);
    void setEnabled(bool enabled) { enabled_ = enabled; }
    // Added for UIMaag's click-to-cycle Air Band button (its label needs
    // to change on every click, e.g. "Air Band: 3 (20.8kHz)") -- every
    // pre-existing button in this project family sets its label once at
    // construction and never again, so this was never needed until now.
    void setLabel(const char* label) { label_ = label; }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    std::string label_;
    std::function<void()> callback_;
    Color bg_, bgHover_, fg_;
    bool hovered_ = false;
    bool pressed_ = false;
    bool enabled_ = true;
};

END_NAMESPACE_DGL
