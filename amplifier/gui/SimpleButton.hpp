// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

// Verbatim from ../compressor/gui/SimpleButton.hpp (itself ported
// verbatim from ../statespace/gui/SimpleButton.hpp --
// see that file's own comment) -- kept identical a second time so all
// three projects' GUIs share the exact same button look and click/hover
// behavior.

#include "NanoVG.hpp"

#include <functional>
#include <string>

START_NAMESPACE_DGL

// A minimal push-button: solid rect, centered label, invokes a callback on
// click-release-inside. Deliberately does not use DPF's ButtonEventHandler
// mixin to keep this self-contained and easy to read.
class SimpleButton : public NanoSubWidget
{
public:
    explicit SimpleButton(Widget* parent, const char* label);

    void setCallback(std::function<void()> cb) { callback_ = std::move(cb); }
    void setColors(Color bg, Color bgHover, Color fg);
    void setEnabled(bool enabled) { enabled_ = enabled; }
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
