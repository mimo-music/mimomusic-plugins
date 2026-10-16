// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "ParamStripWidget.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>

START_NAMESPACE_DGL

ParamStripWidget::ParamStripWidget(NanoTopLevelWidget* parent)
    : EditorSubWidget(parent)
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
        if (editing)
            // A trailing caret, shown even on an empty (just-cleared)
            // buffer, makes it visually obvious this field is live and
            // accepting keystrokes right now -- rather than looking
            // identical to a blank/zero value.
            toShow = editBuffer_ + "|";
        else
        {
            const float v = getter_ ? getter_(f.paramIndex) : f.defaultValue;
            toShow = formatValue(f, v);
        }

        // Pure black on the bright amber fill -- the highest-contrast pair
        // available in this palette -- specifically so a typed value reads
        // clearly rather than blending into the highlight color.
        fillColor(editing ? Color(0, 0, 0) : Color(224, 227, 232));
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        // Fit the text into the field: when "value unit" is too wide, the
        // unit moves to a second, smaller line; anything still too wide
        // (a long number, or the edit buffer) is drawn in a smaller font.
        const float cx = fieldX_[i] + fieldW_[i] * 0.5f;
        const float maxW = std::max(4.0f, fieldW_[i] - 8.0f);
        const float fs = editing ? 15.5f : 14.5f;
        auto fitText = [&](const std::string& s, float size, float minSize, float y) {
            fontSize(size);
            Rectangle<float> b;
            textBounds(0.0f, 0.0f, s.c_str(), nullptr, b);
            if (b.getWidth() > maxW && b.getWidth() > 0.0f)
                fontSize(std::max(minSize, size * maxW / b.getWidth()));
            text(cx, y, s.c_str(), nullptr);
        };
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fontSize(fs);
        Rectangle<float> whole;
        textBounds(0.0f, 0.0f, toShow.c_str(), nullptr, whole);
        const size_t unitSep = toShow.rfind(' ');
        if (!editing && whole.getWidth() > maxW && unitSep != std::string::npos && unitSep + 1 < toShow.size())
        {
            fitText(toShow.substr(0, unitSep), fs, 8.0f, fieldY_ + fieldH_ * 0.30f);
            fitText(toShow.substr(unitSep + 1), 10.0f, 7.0f, fieldY_ + fieldH_ * 0.56f);
        }
        else
        {
            fitText(toShow, fs, 8.0f, fieldY_ + fieldH_ * 0.40f);
        }

        fillColor(editing ? Color(60, 48, 10) : Color(140, 144, 152));
        fontSize(10.5f);
        text(fieldX_[i] + fieldW_[i] * 0.5f, fieldY_ + fieldH_ * 0.80f, f.label.c_str(), nullptr);
    }
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

void ParamStripWidget::applyDrag(int index, double dyPixels)
{
    const ParamFieldSpec& f = fields_[index];
    float newValue;
    if (f.nudgeMultiplicative && f.minValue <= 0.0f)
    {
        // A multiplicative field whose range includes a real 0. Pure log-domain
        // dragging (the branch below) starts from max(1e-6, value), so from exactly
        // 0 it needs ~120 px of dragging before the display even leaves 0 -- a drag
        // starting at zero felt stuck, while one starting from any nonzero value
        // was fine. The pixel position s along the fader is instead mapped
        //     s <= 0          -> 0
        //     0 < s < sc      -> linear:      v = r * vc * s        (fast off zero)
        //     s >= sc         -> exponential: v = vc * exp(r*(s-sc)) (the old feel)
        // with r the old exp rate ((ln max - ln 1e-6)/200 per pixel), sc = 1/r and the
        // crossover vc = 5 units of the last displayed digit: continuous in value and
        // slope, and identical to the old behaviour for any value >= vc.
        // (Ported from compressor/gui/ParamStripWidget.cpp, where Contour's Attack/
        // Release -- the only 0-capable fields so far -- exposed it; no field of this
        // project has a 0 minimum today, so this only matters for future ones.)
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
        if (newValue < 0.5f * float(std::pow(10.0, -double(f.decimals)))) newValue = 0.0f; // would print as 0: make it exactly 0
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

    // Alt+scroll = 10x finer, the same in every plugin of this family: a
    // tenth of the percentage for a multiplicative step, a tenth of the
    // increment for an additive one.
    const bool fine = (ev.mod & kModifierAlt) != 0;

    const float current = getter_ ? getter_(f.paramIndex) : f.defaultValue;
    float next;
    if (f.nudgeMultiplicative)
    {
        constexpr double kFactor = 1.06; // ~0.5dB-equivalent step on a wide log range
        const double factor = fine ? 1.0 + (kFactor - 1.0) / 10.0 : kFactor;
        next = float(dir > 0 ? double(current) * factor : double(current) / factor);
    }
    else
    {
        next = current + float(dir) * (fine ? f.nudgeStep / 10.0f : f.nudgeStep);
    }
    next = std::max(f.minValue, std::min(f.maxValue, next));
    if (setter_) setter_(f.paramIndex, next);
    repaint();
    return true;
}

END_NAMESPACE_DGL
