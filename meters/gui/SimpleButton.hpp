// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

// Originally verbatim from ../statespace/gui/SimpleButton.hpp -- click/
// hover mechanics still match exactly, so every plugin in the family
// behaves the same. Diverges only in the addition of
// setTooltip()/tooltip()/isHovered() below, for hover descriptions --
// these are pure additions this
// project's own TooltipOverlay reads, changing nothing about how a click
// or a hover-color-change already worked.

#include "EditorSubWidget.hpp"

#include <functional>
#include <string>

START_NAMESPACE_DGL

// A minimal push-button: solid rect, centered label, invokes a callback on
// click-release-inside. Deliberately does not use DPF's ButtonEventHandler
// mixin to keep this self-contained and easy to read.
class SimpleButton : public EditorSubWidget
{
public:
    explicit SimpleButton(NanoTopLevelWidget* parent, const char* label);

    void setCallback(std::function<void()> cb) { callback_ = std::move(cb); }
    void setColors(Color bg, Color bgHover, Color fg);
    void setEnabled(bool enabled) { enabled_ = enabled; }
    void setLabel(const char* label) { label_ = label; }
    // A short hover description. Purely descriptive
    // text for TooltipOverlay to show; empty (the default) means no
    // tooltip for this button.
    void setTooltip(const char* text) { tooltip_ = text ? text : ""; }
    const std::string& tooltip() const { return tooltip_; }
    bool isHovered() const { return enabled_ && hovered_; }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    std::string label_;
    std::string tooltip_;
    std::function<void()> callback_;
    Color bg_, bgHover_, fg_;
    bool hovered_ = false;
    bool pressed_ = false;
    bool enabled_ = true;
};

END_NAMESPACE_DGL
