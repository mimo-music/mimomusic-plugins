// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "SimpleButton.hpp"

START_NAMESPACE_DGL

SimpleButton::SimpleButton(Widget* parent, const char* label)
    : NanoSubWidget(parent),
      label_(label),
      bg_(58, 62, 70), bgHover_(78, 84, 96), fg_(230, 232, 236)
{
    // Each NanoSubWidget gets its own NanoVG context by default, so the
    // bundled font has to be loaded here too, not just once on the
    // top-level UI (loadSharedResources() is idempotent -- cheap to call
    // per widget).
    loadSharedResources();
}

void SimpleButton::setColors(Color bg, Color bgHover, Color fg)
{
    bg_ = bg; bgHover_ = bgHover; fg_ = fg;
}

void SimpleButton::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    roundedRect(0.5f, 0.5f, w - 1.0f, h - 1.0f, 4.0f);
    fillColor(enabled_ ? (hovered_ ? bgHover_ : bg_) : Color(40, 40, 42));
    fill();
    strokeColor(Color(20, 20, 22));
    strokeWidth(1.0f);
    stroke();

    fillColor(enabled_ ? fg_ : Color(120, 120, 124));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(13.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(w * 0.5f, h * 0.5f + 1.0f, label_.c_str(), nullptr);
}

bool SimpleButton::onMouse(const MouseEvent& ev)
{
    if (!enabled_) return false;
    if (ev.button != 1) return false;

    const bool inside = ev.pos.getX() >= 0.0 && ev.pos.getY() >= 0.0 &&
                         ev.pos.getX() < double(getWidth()) && ev.pos.getY() < double(getHeight());

    if (ev.press)
    {
        // Only claim the press if it actually landed on us -- DGL's
        // subwidget dispatch hands every sibling the event regardless of
        // position and relies on each widget's own bounds check, it does
        // not hit-test for us. Returning true unconditionally here would
        // (and did) swallow every click in the whole window.
        if (!inside) return false;
        pressed_ = true;
        repaint();
        return true;
    }

    // Release: only act if *we* were the one that received the press.
    if (!pressed_) return false;
    pressed_ = false;
    repaint();
    if (inside && callback_) callback_();
    return true;
}

bool SimpleButton::onMotion(const MotionEvent& ev)
{
    const bool inside = ev.pos.getX() >= 0.0 && ev.pos.getY() >= 0.0 &&
                         ev.pos.getX() < double(getWidth()) && ev.pos.getY() < double(getHeight());
    if (inside != hovered_) { hovered_ = inside; repaint(); }
    return false;
}

END_NAMESPACE_DGL
