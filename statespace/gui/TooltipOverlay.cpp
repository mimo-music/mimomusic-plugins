// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "TooltipOverlay.hpp"

#include <algorithm>

START_NAMESPACE_DGL

TooltipOverlay::TooltipOverlay(NanoTopLevelWidget* parent)
    : EditorSubWidget(parent)
{
    loadSharedResources();
}

void TooltipOverlay::addTarget(SubWidget* target, std::string text)
{
    entries_.push_back({ target, std::move(text), nullptr });
}

void TooltipOverlay::addFieldTarget(SubWidget* target, std::function<std::string(double, double)> describe)
{
    entries_.push_back({ target, std::string(), std::move(describe) });
}

TooltipOverlay::Resolved TooltipOverlay::resolve(double x, double y) const
{
    // Reverse registration order -- a later-registered widget (typically
    // also later-constructed, so drawn on top per DGL's own paint order)
    // wins on overlap, same "last constructed = topmost" convention this
    // project already relies on elsewhere.
    for (auto it = entries_.rbegin(); it != entries_.rend(); ++it)
    {
        SubWidget* w = it->target;
        if (!w->isVisible()) continue;
        const double wx = w->getAbsoluteX(), wy = w->getAbsoluteY();
        const double ww = w->getWidth(), wh = w->getHeight();
        if (x < wx || x >= wx + ww || y < wy || y >= wy + wh) continue;

        if (it->describe)
        {
            std::string t = it->describe(x - wx, y - wy);
            if (t.empty()) continue; // this widget claims the region but has nothing to say right here (e.g. a gap between fields) -- keep checking widgets below
            return { &(*it), std::move(t) };
        }
        return { &(*it), it->text };
    }
    return {};
}

bool TooltipOverlay::onMouse(const MouseEvent&)
{
    // Never claims a click -- see this class's own header comment for why.
    return false;
}

bool TooltipOverlay::onMotion(const MotionEvent& ev)
{
    Resolved r = resolve(ev.pos.getX(), ev.pos.getY());
    if (r.entry != hoverEntry_ || r.text != hoverText_)
    {
        hoverEntry_ = r.entry;
        hoverText_ = std::move(r.text);
        hoverSince_ = std::chrono::steady_clock::now();
        repaint(); // clears any tooltip that was already showing for the previous target/field, promptly
    }
    return false;
}

bool TooltipOverlay::onScroll(const ScrollEvent&)
{
    return false;
}

void TooltipOverlay::tick()
{
    if (hoverEntry_ == nullptr) return;
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - hoverSince_).count();
    // Only need to nudge a repaint right as the delay threshold crosses --
    // onNanoDisplay() itself re-checks the elapsed time on every call
    // regardless, so this just has to land *a* tick within the window
    // where the tooltip should newly become visible, not be exact to the
    // millisecond.
    if (elapsedMs >= kDelayMs && elapsedMs < kDelayMs + 250) repaint();
}

void TooltipOverlay::onNanoDisplay()
{
    if (hoverEntry_ == nullptr || hoverText_.empty()) return;
    const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - hoverSince_).count();
    if (elapsedMs < kDelayMs) return;

    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(12.0f);
    textLineHeight(1.3f);

    float bounds[4];
    textBoxBounds(0.0f, 0.0f, kMaxTextWidth, hoverText_.c_str(), nullptr, bounds);
    const float textW = bounds[2] - bounds[0];
    const float textH = bounds[3] - bounds[1];
    constexpr float padX = 8.0f, padY = 6.0f;
    const float boxW = textW + padX * 2.0f;
    const float boxH = textH + padY * 2.0f;

    SubWidget* w = hoverEntry_->target;
    const float wx = float(w->getAbsoluteX());
    const float wy = float(w->getAbsoluteY());
    const float ww = float(w->getWidth());
    const float wh = float(w->getHeight());

    const float winW = float(getWidth());
    const float winH = float(getHeight());

    float bx = wx + (ww - boxW) * 0.5f;
    float by = wy + wh + 6.0f; // just below the hovered widget by default
    if (by + boxH > winH) by = wy - boxH - 6.0f; // flip above if it wouldn't fit below
    bx = std::max(4.0f, std::min(winW - boxW - 4.0f, bx));
    by = std::max(4.0f, std::min(winH - boxH - 4.0f, by));

    beginPath();
    roundedRect(bx, by, boxW, boxH, 4.0f);
    fillColor(Color(20, 21, 25, 0.95f));
    fill();
    strokeColor(Color(90, 94, 102, 0.9f));
    strokeWidth(1.0f);
    stroke();

    fillColor(Color(230, 232, 236));
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    textBox(bx + padX, by + padY, kMaxTextWidth, hoverText_.c_str(), nullptr);
}

END_NAMESPACE_DGL
