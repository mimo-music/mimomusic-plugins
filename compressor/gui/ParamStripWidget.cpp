// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "ParamStripWidget.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>

START_NAMESPACE_DGL

ParamStripWidget::ParamStripWidget(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
}

bool ParamStripWidget::fieldAt(double x, double y, int& index) const
{
    if (y < fieldY_ || y >= fieldY_ + fieldH_) return false;
    for (size_t i = 0; i < fields_.size(); ++i)
        if (x >= fieldX_[i] && x < fieldX_[i] + fieldW_[i]) { index = int(i); return true; }
    return false;
}

std::string ParamStripWidget::formatValue(const ParamFieldSpec& f, float value) const
{
    // std::to_chars (fixed format, f.decimals digits after the point),
    // unlike snprintf's "%.*f", always emits '.' regardless of the host's
    // current numeric locale -- a host that switches the process's own
    // LC_NUMERIC away from "C" (e.g. GTK/Qt's setlocale(LC_ALL, "") at
    // init, adopting a comma-decimal OS locale) would otherwise corrupt
    // both this display and, worse, commitEdit()'s own parse of it -- see
    // that function's comment for the full story.
    char buf[48];
    auto res = std::to_chars(buf, buf + sizeof(buf), value, std::chars_format::fixed, f.decimals);
    std::string out(buf, res.ptr);
    if (!f.unit.empty()) { out += ' '; out += f.unit; }
    return out;
}

void ParamStripWidget::beginEdit(int index, bool clearValue)
{
    editField_ = index;
    if (clearValue)
    {
        editBuffer_.clear();
    }
    else
    {
        const float v = getter_ ? getter_(fields_[index].paramIndex) : fields_[index].defaultValue;
        char buf[32];
        const auto res = std::to_chars(buf, buf + sizeof(buf), v, std::chars_format::fixed,
                                        fields_[index].decimals);
        editBuffer_.assign(buf, res.ptr);
    }

    // Embedded plugin UIs (e.g. inside a host like Ardour) are not
    // guaranteed to already hold keyboard input focus just because a
    // widget inside them was clicked -- grab it explicitly so the typed
    // digits and Enter/Escape actually reach this window instead of (or
    // "sometimes" reaching, depending on whatever already had focus) the
    // host. Same fix, same reasoning, as
    // ../statespace/gui/UIStateSpace.cpp's identical
    // call in its own beginCellTextEdit() -- missed here originally.
    getWindow().focus();

    repaint();
}

void ParamStripWidget::appendChar(char c)
{
    if (editField_ < 0) return;
    if ((c >= '0' && c <= '9') || c == '.' || c == '-')
    {
        editBuffer_ += c;
        repaint();
    }
}

void ParamStripWidget::backspace()
{
    if (editField_ < 0 || editBuffer_.empty()) return;
    editBuffer_.pop_back();
    repaint();
}

void ParamStripWidget::commitEdit()
{
    if (editField_ < 0) return;
    const ParamFieldSpec& f = fields_[editField_];
    float v = f.defaultValue;
    if (!editBuffer_.empty())
    {
        // std::from_chars, unlike std::strtod, is locale-independent --
        // see formatValue()'s comment above for why that matters.
        double parsed = 0.0;
        const auto res = std::from_chars(editBuffer_.data(), editBuffer_.data() + editBuffer_.size(), parsed);
        if (res.ec == std::errc())
            v = float(parsed);
    }
    v = std::max(f.minValue, std::min(f.maxValue, v));
    if (setter_) setter_(f.paramIndex, v);
    editField_ = -1;
    editBuffer_.clear();
    repaint();
}

void ParamStripWidget::cancelEdit()
{
    editField_ = -1;
    editBuffer_.clear();
    repaint();
}

void ParamStripWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(26, 28, 33));
    fill();

    if (fields_.empty()) return;

    constexpr float pad = 6.0f;
    constexpr float gap = 6.0f;
    fieldY_ = pad;
    fieldH_ = h - 2.0f * pad;

    const float totalGaps = gap * float(fields_.size() - 1);
    const float oneW = (w - 2.0f * pad - totalGaps) / float(fields_.size());

    fieldX_.assign(fields_.size(), 0.0f);
    fieldW_.assign(fields_.size(), oneW);

    float fx = pad;
    for (size_t i = 0; i < fields_.size(); ++i)
    {
        fieldX_[i] = fx;
        fx += oneW + gap;

        const ParamFieldSpec& f = fields_[i];
        const bool editing = (editField_ == int(i));
        const bool hovered = (hoverField_ == int(i));

        beginPath();
        roundedRect(fieldX_[i] + 0.5f, fieldY_ + 0.5f, fieldW_[i] - 1.0f, fieldH_ - 1.0f, 4.0f);
        fillColor(editing ? Color(235, 195, 60) : (hovered ? Color(42, 46, 54) : Color(34, 37, 43)));
        fill();
        // A brighter, thicker outline while editing -- on top of the
        // already-distinct amber fill -- so an active edit is unmistakable
        // even before any text has been typed into it.
        strokeColor(editing ? Color(255, 255, 255) : Color(14, 15, 18));
        strokeWidth(editing ? 2.0f : 1.0f);
        stroke();

        std::string toShow;
        float v = 0.0f;
        if (editing)
            // A trailing caret, shown even on an empty (just-cleared)
            // buffer, makes it visually obvious this field is live and
            // accepting keystrokes right now -- rather than looking
            // identical to a blank/zero value.
            toShow = editBuffer_ + "|";
        else
        {
            v = getter_ ? getter_(f.paramIndex) : f.defaultValue;
            toShow = formatValue(f, v);
        }

        // Pure black on the bright amber fill -- the highest-contrast pair
        // available in this palette -- specifically so a typed value reads
        // clearly rather than blending into the highlight color. Always
        // this widget's one normal-colored piece of text: the wet/dry-style
        // tint below applies to the *label*, never here.
        fillColor(editing ? Color(0, 0, 0) : Color(224, 227, 232));
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(editing ? 15.5f : 14.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(fieldX_[i] + fieldW_[i] * 0.5f, fieldY_ + fieldH_ * 0.40f, toShow.c_str(), nullptr);

        const float labelY = fieldY_ + fieldH_ * 0.80f;
        fontSize(10.5f);
        if (!editing && f.bipolarLabel)
        {
            // At 100% "Wet" is green and "Dry" red, at 0% the reverse,
            // fading in between: labelHigh (green at maxValue) and labelLow (the exact
            // opposite tint) drawn side by side in place of the plain
            // single-color f.label, centered as a pair.
            const float range = std::max(1.0e-6f, f.maxValue - f.minValue);
            const float t = std::max(0.0f, std::min(1.0f, (v - f.minValue) / range));
            const Color kLow(200, 110, 100);  // muted red
            const Color kHigh(120, 195, 130); // muted green
            auto lerp = [&](float tt) {
                return Color(kLow.red + (kHigh.red - kLow.red) * tt,
                             kLow.green + (kHigh.green - kLow.green) * tt,
                             kLow.blue + (kHigh.blue - kLow.blue) * tt);
            };
            const Color highColor = lerp(t);
            const Color lowColor = lerp(1.0f - t);
            const char* sep = " / ";

            Rectangle<float> bHigh, bSep, bLow;
            textBounds(0.0f, 0.0f, f.labelHigh.c_str(), nullptr, bHigh);
            textBounds(0.0f, 0.0f, sep, nullptr, bSep);
            textBounds(0.0f, 0.0f, f.labelLow.c_str(), nullptr, bLow);
            const float totalW = bHigh.getWidth() + bSep.getWidth() + bLow.getWidth();

            float lx = fieldX_[i] + fieldW_[i] * 0.5f - totalW * 0.5f;
            textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
            fillColor(highColor);
            text(lx, labelY, f.labelHigh.c_str(), nullptr);
            lx += bHigh.getWidth();
            fillColor(Color(140, 144, 152));
            text(lx, labelY, sep, nullptr);
            lx += bSep.getWidth();
            fillColor(lowColor);
            text(lx, labelY, f.labelLow.c_str(), nullptr);
        }
        else
        {
            fillColor(editing ? Color(60, 48, 10) : Color(140, 144, 152));
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(fieldX_[i] + fieldW_[i] * 0.5f, labelY, f.label.c_str(), nullptr);
        }
    }
}

std::string ParamStripWidget::fieldTooltipAt(double x, double y) const
{
    int idx;
    if (!fieldAt(x, y, idx)) return std::string();
    return fields_[size_t(idx)].tooltip;
}

bool ParamStripWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (ev.press)
    {
        // A press anywhere -- including on a different field, or outside
        // any field entirely -- commits an in-progress text edit first,
        // exactly like pressing Enter, rather than leaving the press
        // unclaimed. Swallowing the click with nothing to show for it
        // (the previous behavior here) is worse than just "loses focus
        // feeling" -- an unclaimed press can knock keyboard focus off the
        // plugin window entirely in some hosts, which then makes Enter
        // stop reaching onKeyboard() too and looks like the field is
        // permanently stuck (the actual bug report this fixes).
        if (editField_ >= 0) commitEdit();
        int idx;
        if (fieldAt(x, y, idx))
        {
            pressField_ = idx;
            dragging_ = false;
            dragStartY_ = y;
            dragStartValue_ = getter_ ? getter_(fields_[idx].paramIndex) : fields_[idx].defaultValue;
            return true;
        }
        return false;
    }

    if (pressField_ >= 0)
    {
        const int released = pressField_;
        const bool wasDragging = dragging_;
        pressField_ = -1;
        dragging_ = false;

        if (wasDragging) return true; // a completed drag is not also a click

        int idx;
        const bool sameField = fieldAt(x, y, idx) && idx == released;
        if (sameField)
        {
            const auto now = std::chrono::steady_clock::now();
            constexpr std::chrono::milliseconds kDoubleClickWindow(400);
            const bool isDoubleClick = released == lastClickField_ && (now - lastClickTime_) < kDoubleClickWindow;
            beginEdit(released, isDoubleClick);
            lastClickField_ = isDoubleClick ? -1 : released;
            lastClickTime_ = now;
        }
        return true;
    }
    return false;
}

bool ParamStripWidget::onMotion(const MotionEvent& ev)
{
    const double y = ev.pos.getY();

    if (pressField_ >= 0)
    {
        // Fader drag: vertical movement only, tracked from the press
        // position regardless of where the pointer strays horizontally --
        // see the class comment for why this works even once the pointer
        // leaves this widget's own bounds. A small dead zone keeps a
        // near-motionless press-and-release a click, not a drag.
        const double dyTotal = dragStartY_ - y; // up = positive
        if (!dragging_ && std::fabs(dyTotal) > kDragDeadZonePx) dragging_ = true;
        if (dragging_) applyDrag(pressField_, dyTotal);
        return true;
    }

    int idx = -1;
    fieldAt(ev.pos.getX(), y, idx);
    if (idx != hoverField_) { hoverField_ = idx; repaint(); }
    return false;
}

// A multiplicative field's log-domain math (below) can only ever
// asymptotically approach 0 -- exp() of a very negative number gets
// smaller but is never exactly 0 within any pixel distance a real drag or
// scroll gesture could cover. That's fine for a field whose true minimum
// is some small positive number (0.1ms was always reachable enough by
// typing it), but Contour's Attack/Release now have a *real*, meaningful
// zero (0 = literally instant, see CurveDsp.hpp) that a user should be
// able to reach by dragging/scrolling all the way down, not just by
// double-click-typing "0". So: once the computed value is close enough to
// 0 that it would print as "0" at the field's own decimal precision
// anyway, just make it exactly 0 -- a small, harmless snap for any other
// multiplicative field (their own minValue is never 0, so this never
// triggers for them), and the difference between "reads as 0" and
// "literally is 0" would otherwise be invisible in the field but very
// audible in the DSP.
static float snapNearZero(const ParamFieldSpec& f, float value)
{
    if (f.minValue > 0.0f) return value;
    const float epsilon = 0.5f * std::pow(10.0f, -f.decimals);
    return (value < epsilon) ? 0.0f : value;
}

void ParamStripWidget::applyDrag(int index, double dyPixels)
{
    const ParamFieldSpec& f = fields_[index];
    float newValue;
    if (f.nudgeMultiplicative && f.minValue <= 0.0f)
    {
        // A multiplicative field whose range includes a real 0 (Contour's
        // Attack/Release: 0 = literally instant). Pure log-domain dragging
        // (below) starts from max(1e-6, value), so from exactly 0 it needs
        // ~120 px of dragging before the display even leaves 0.00 -- a drag
        // starting at zero felt stuck, while one starting from any nonzero
        // value was fine. The pixel position s along the fader is instead mapped
        //     s <= 0          -> 0
        //     0 < s < sc      -> linear:      v = r * vc * s        (fast off zero)
        //     s >= sc         -> exponential: v = vc * exp(r*(s-sc)) (the old feel)
        // with r the same exp rate the old code had ((ln max - ln 1e-6)/200 per
        // pixel), sc = 1/r and the crossover vc = 5 units of the last displayed
        // digit, so slope and value are continuous at sc and everything above
        // vc behaves exactly as before (v0*exp(r*dy)).
        const double r = (std::log(double(std::max(1.0e-6f, f.maxValue))) - std::log(1.0e-6)) / kDragPixelsForFullRange;
        const double vc = 5.0 * std::pow(10.0, -double(f.decimals));
        const double sc = 1.0 / r;
        const double v0 = std::max(0.0f, dragStartValue_);
        const double s0 = v0 <= 0.0 ? 0.0 : (v0 < vc ? v0 / (r * vc) : sc + std::log(v0 / vc) / r);
        const double s = s0 + dyPixels;
        double v;
        if (s <= 0.0) v = 0.0;
        else if (s < sc) v = r * vc * s;
        else v = vc * std::exp(r * (s - sc));
        newValue = float(v);
        newValue = snapNearZero(f, newValue);
    }
    else if (f.nudgeMultiplicative)
    {
        // Wide-log-range fields (Attack/Release): drag in log domain so
        // the same pixel distance feels equally sensitive across the
        // whole range, not squeezed at the low end.
        const double logMin = std::log(double(std::max(1.0e-6f, f.minValue)));
        const double logMax = std::log(double(std::max(1.0e-6f, f.maxValue)));
        const double logStart = std::log(double(std::max(1.0e-6f, dragStartValue_)));
        const double newLog = logStart + (dyPixels / kDragPixelsForFullRange) * (logMax - logMin);
        newValue = float(std::exp(newLog));
        newValue = snapNearZero(f, newValue);
    }
    else
    {
        const double range = double(f.maxValue) - double(f.minValue);
        newValue = float(double(dragStartValue_) + (dyPixels / kDragPixelsForFullRange) * range);
    }
    newValue = std::max(f.minValue, std::min(f.maxValue, newValue));
    if (setter_) setter_(f.paramIndex, newValue);
    repaint();
}

bool ParamStripWidget::onScroll(const ScrollEvent& ev)
{
    int idx;
    if (!fieldAt(ev.pos.getX(), ev.pos.getY(), idx)) return false;
    if (editField_ >= 0) return true; // don't nudge while a text edit is in progress

    const ParamFieldSpec& f = fields_[idx];
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    // Alt+scroll = 10x finer, the same in every plugin of this family.
    // For a multiplicative step, "10x
    // finer" means the *percentage* change per notch shrinks by 10x, not
    // the raw factor divided by 10 (which would barely move anything
    // close to 1.0); for an additive step, it's a plain step/10.
    const bool fine = (ev.mod & kModifierAlt) != 0;

    const float current = getter_ ? getter_(f.paramIndex) : f.defaultValue;
    float next;
    if (f.nudgeMultiplicative)
    {
        constexpr double kFactor = 1.06; // ~0.5dB-equivalent step on a wide log range
        const double factor = fine ? 1.0 + (kFactor - 1.0) / 10.0 : kFactor;
        // A field already at (or snapped to) exactly 0 can't be scaled
        // back up by any multiplicative factor -- 0 * anything is still
        // 0 -- so scrolling up from 0 needs its own starting point.
        // nudgeStep doubles as a sane "first step off zero" here (both
        // Attack/Release already set it to a small musically-reasonable
        // value; see refreshStripFields()).
        if (current <= 0.0f)
            next = (dir > 0) ? f.nudgeStep : 0.0f;
        else
            next = float(dir > 0 ? double(current) * factor : double(current) / factor);
        next = snapNearZero(f, next);
    }
    else
    {
        const double step = fine ? double(f.nudgeStep) / 10.0 : double(f.nudgeStep);
        next = float(double(current) + double(dir) * step);
    }
    next = std::max(f.minValue, std::min(f.maxValue, next));
    if (setter_) setter_(f.paramIndex, next);
    repaint();
    return true;
}

END_NAMESPACE_DGL
