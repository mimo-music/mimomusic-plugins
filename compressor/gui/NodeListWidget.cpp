// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "NodeListWidget.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

using sscomp::CurvePoint;
using sscomp::PointDirection;
using sscomp::TransferCurve;

namespace {
// Same amber/blue-violet pair ContourGraphWidget tints rising/falling-only
// nodes with, so a row's swatch reads as the same node on the graph.
Color dirColor(PointDirection d)
{
    return d == PointDirection::RisingOnly ? Color(214, 158, 66)
         : d == PointDirection::FallingOnly ? Color(146, 158, 232)
         : Color(150, 156, 166);
}
const char* dirLabel(PointDirection d)
{
    return d == PointDirection::RisingOnly ? "Rising" : d == PointDirection::FallingOnly ? "Falling" : "Both";
}
constexpr double kHandleYLimitDb = TransferCurve::kMaxDb - TransferCurve::kMinDb;
}

NodeListWidget::NodeListWidget(NanoTopLevelWidget* parent)
    : EditorSubWidget(parent)
{
    loadSharedResources();
}

void NodeListWidget::setCurve(const TransferCurve& curve)
{
    curve_ = curve;
    if (selected_ >= curve_.pointCount()) selected_ = -1;
    // An edit in progress on a row that no longer exists (undo/redo, preset
    // load) can't be committed anywhere sensible -- drop it.
    if (editRow_ >= curve_.pointCount()) { editRow_ = -1; editBuffer_.clear(); }
    const int maxOffset = std::max(0, curve_.pointCount() - maxVisibleRows());
    scrollOffset_ = std::min(scrollOffset_, maxOffset);
    repaint();
}

NodeListWidget::Columns NodeListWidget::columns() const
{
    Columns c;
    float x = 4.0f;
    auto place = [&](float& fx, float& fw, float w) { fx = x; fw = w; x += w + 4.0f; };
    place(c.numX, c.numW, 22.0f);
    place(c.dirX, c.dirW, 56.0f);
    c.fieldW = 58.0f;
    for (int i = 0; i < 4; ++i) { c.fieldX[i] = x; x += c.fieldW + 4.0f; }
    place(c.delX, c.delW, 20.0f);
    c.total = x;
    return c;
}

int NodeListWidget::maxVisibleRows() const
{
    return std::max(1, int((float(getHeight()) - kTitleH - kHeaderH - kFooterH) / kRowH));
}

int NodeListWidget::rowAt(double y) const
{
    const double top = double(kTitleH + kHeaderH);
    if (y < top || inFooter(y)) return -1;
    const int visibleRow = int((y - top) / double(kRowH));
    if (visibleRow < 0 || visibleRow >= maxVisibleRows()) return -1;
    const int idx = scrollOffset_ + visibleRow;
    return idx < curve_.pointCount() ? idx : -1;
}

bool NodeListWidget::fieldEditable(int row, Field f) const
{
    // The two fixed endpoints can't move sideways.
    return !(f == Field::In && curve_.isEndpoint(row));
}

double NodeListWidget::fieldValue(int row, Field f) const
{
    const CurvePoint p = curve_.point(row);
    switch (f)
    {
    case Field::In: return p.inputDb;
    case Field::Out: return p.outputDb;
    case Field::HandleX: return p.handleInputDb;
    case Field::HandleY: return p.handleOutputDb;
    }
    return 0.0;
}

void NodeListWidget::applyField(int row, Field f, double v)
{
    if (row < 0 || row >= curve_.pointCount() || !fieldEditable(row, f)) return;
    const CurvePoint p = curve_.point(row);
    switch (f)
    {
    case Field::In:
    case Field::Out:
    {
        // Position edits carry the node's whole link group along (and the
        // threshold, on X only) by the same delta -- see
        // TransferCurve::moveLinked(). An unlinked node moves alone.
        const double target = std::max(TransferCurve::kMinDb, std::min(TransferCurve::kMaxDb, v));
        const double dIn = f == Field::In ? target - p.inputDb : 0.0;
        const double dOut = f == Field::Out ? target - p.outputDb : 0.0;
        double thr = double(thresholdDb_);
        curve_.moveLinked(row, dIn, dOut, thr);
        if (std::fabs(thr - double(thresholdDb_)) > 1.0e-9)
        {
            thresholdDb_ = float(thr);
            if (onThresholdChanged_) onThresholdChanged_(thresholdDb_);
        }
        break;
    }
    case Field::HandleX:
        // setPointHandle() further clamps X to [0, gap to the nearest neighbor].
        // dx may be negative: the bright dot on the other side of the node (see CurvePoint::handleInputDb).
        curve_.setPointHandle(row, std::max(-kHandleYLimitDb, std::min(kHandleYLimitDb, v)), p.handleOutputDb);
        break;
    case Field::HandleY:
        curve_.setPointHandle(row, p.handleInputDb, std::max(-kHandleYLimitDb, std::min(kHandleYLimitDb, v)));
        break;
    }
    fireEdited();
    repaint();
}

void NodeListWidget::beginEdit(int row, Field f, bool clearValue)
{
    editRow_ = row;
    editField_ = f;
    if (clearValue)
        editBuffer_.clear();
    else
    {
        char buf[32];
        const auto res = std::to_chars(buf, buf + sizeof(buf), fieldValue(row, f), std::chars_format::fixed, 2);
        editBuffer_.assign(buf, res.ptr);
    }
    // Embedded plugin UIs aren't guaranteed to hold keyboard focus just
    // because a widget inside was clicked -- see ParamStripWidget::beginEdit().
    getWindow().focus();
    repaint();
}

void NodeListWidget::appendChar(char c)
{
    if (editRow_ < 0) return;
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { editBuffer_ += c; repaint(); }
}

void NodeListWidget::backspace()
{
    if (editRow_ < 0 || editBuffer_.empty()) return;
    editBuffer_.pop_back();
    repaint();
}

void NodeListWidget::commitEdit()
{
    if (editRow_ < 0) return;
    const int row = editRow_;
    const Field f = editField_;
    editRow_ = -1;
    if (!editBuffer_.empty())
    {
        // std::from_chars is locale-independent -- see
        // ParamStripWidget::commitEdit() for why that matters.
        double parsed = 0.0;
        const auto res = std::from_chars(editBuffer_.data(), editBuffer_.data() + editBuffer_.size(), parsed);
        if (res.ec == std::errc()) applyField(row, f, parsed);
    }
    editBuffer_.clear();
    repaint();
}

void NodeListWidget::cancelEdit()
{
    editRow_ = -1;
    editBuffer_.clear();
    repaint();
}

void NodeListWidget::addNodeInWidestGap()
{
    // Midpoint of the widest X gap, placed exactly on the current curve so
    // nothing changes audibly until the new node is actually edited.
    int gapIdx = 0;
    double widest = -1.0;
    for (int i = 0; i + 1 < curve_.pointCount(); ++i)
    {
        const double gap = curve_.point(i + 1).inputDb - curve_.point(i).inputDb;
        if (gap > widest) { widest = gap; gapIdx = i; }
    }
    const double x = curve_.point(gapIdx).inputDb + widest * 0.5;
    const int idx = curve_.addPoint(x, curve_.evaluate(x));
    if (idx < 0) return;
    fireEdited();
    selectRow(idx);
    const int maxRows = maxVisibleRows();
    if (idx >= scrollOffset_ + maxRows) scrollOffset_ = idx - maxRows + 1;
    repaint();
}

void NodeListWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());
    const Columns c = columns();

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(22, 24, 28));
    fill();

    fontFace(NANOVG_DEJAVU_SANS_TTF);

    fillColor(Color(190, 194, 202));
    fontSize(11.0f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    text(6.0f, kTitleH * 0.5f, title_.c_str(), nullptr);

    {
        fillColor(Color(130, 134, 142));
        fontSize(9.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        const float midY = kTitleH + kHeaderH * 0.5f;
        text(c.numX + c.numW * 0.5f, midY, "#", nullptr);
        text(c.dirX + c.dirW * 0.5f, midY, "ACTIVE", nullptr);
        text(c.fieldX[0] + c.fieldW * 0.5f, midY, "IN dB", nullptr);
        text(c.fieldX[1] + c.fieldW * 0.5f, midY, "OUT dB", nullptr);
        text(c.fieldX[2] + c.fieldW * 0.5f, midY, "SUPP dX", nullptr);
        text(c.fieldX[3] + c.fieldW * 0.5f, midY, "SUPP dY", nullptr);
        beginPath();
        moveTo(0.0f, kTitleH + kHeaderH - 0.5f);
        lineTo(w, kTitleH + kHeaderH - 0.5f);
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();
    }

    const int n = curve_.pointCount();
    const int maxRows = maxVisibleRows();
    for (int visibleRow = 0; visibleRow < maxRows; ++visibleRow)
    {
        const int i = scrollOffset_ + visibleRow;
        if (i >= n) break;
        const float ry = rowY(i);
        const bool endpoint = curve_.isEndpoint(i);
        const bool sel = i == selected_;

        if (sel || i == hoverRow_)
        {
            beginPath();
            rect(0, ry, w, kRowH);
            fillColor(sel ? Color(45, 50, 58) : Color(32, 35, 41));
            fill();
        }

        const PointDirection dir = curve_.pointDirection(i);

        // Index chip, tinted by the node's own direction tag.
        beginPath();
        roundedRect(c.numX + 0.5f, ry + 3.0f, c.numW - 1.0f, kRowH - 6.0f, 3.0f);
        fillColor(dirColor(dir).withAlpha(0.85f));
        fill();
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%d", i + 1);
        fillColor(Color(15, 16, 19));
        fontSize(10.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(c.numX + c.numW * 0.5f, ry + kRowH * 0.5f, buf, nullptr);

        // Direction (hysteresis) button -- endpoints are always Both.
        beginPath();
        roundedRect(c.dirX + 0.5f, ry + 3.0f, c.dirW - 1.0f, kRowH - 6.0f, 3.0f);
        fillColor(endpoint ? Color(26, 28, 33) : Color(38, 41, 48));
        fill();
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();
        fillColor(endpoint ? Color(90, 94, 100) : dir == PointDirection::Both ? Color(210, 214, 222) : dirColor(dir));
        fontSize(10.0f);
        text(c.dirX + c.dirW * 0.5f, ry + kRowH * 0.5f, endpoint ? "--" : dirLabel(dir), nullptr);

        // Numeric fields.
        for (int fi = 0; fi < 4; ++fi)
        {
            const Field f = Field(fi);
            const bool editing = editRow_ == i && editField_ == f;
            const bool editable = fieldEditable(i, f);
            beginPath();
            roundedRect(c.fieldX[fi] + 0.5f, ry + 3.0f, c.fieldW - 1.0f, kRowH - 6.0f, 3.0f);
            fillColor(editing ? Color(235, 195, 60) : editable ? Color(30, 32, 38) : Color(26, 28, 33));
            fill();
            strokeColor(Color(14, 15, 18));
            strokeWidth(1.0f);
            stroke();

            const char* toShow;
            char vbuf[32];
            if (editing) toShow = editBuffer_.c_str();
            else { std::snprintf(vbuf, sizeof(vbuf), "%.2f", fieldValue(i, f)); toShow = vbuf; }
            fillColor(editing ? Color(20, 20, 24) : editable ? Color(215, 218, 224) : Color(110, 114, 122));
            fontSize(10.5f);
            text(c.fieldX[fi] + c.fieldW * 0.5f, ry + kRowH * 0.5f, toShow, nullptr);
        }

        // Delete -- never offered for the two fixed endpoints.
        if (!endpoint)
        {
            beginPath();
            roundedRect(c.delX + 0.5f, ry + 3.0f, c.delW - 1.0f, kRowH - 6.0f, 3.0f);
            fillColor(Color(90, 45, 45));
            fill();
            strokeColor(Color(14, 15, 18));
            strokeWidth(1.0f);
            stroke();
            fillColor(Color(230, 200, 200));
            fontSize(11.0f);
            text(c.delX + c.delW * 0.5f, ry + kRowH * 0.5f, "x", nullptr);
        }
    }

    if (n > maxRows)
    {
        fillColor(Color(140, 144, 152));
        fontSize(9.5f);
        textAlign(ALIGN_RIGHT | ALIGN_BOTTOM);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "scroll for more (%d/%d)", scrollOffset_ + maxRows, n);
        text(w - 4.0f, h - kFooterH - 1.0f, buf, nullptr);
    }

    // Footer: add a node.
    const bool full = n >= TransferCurve::kMaxPoints;
    beginPath();
    roundedRect(4.0f, h - kFooterH + 3.0f, w - 8.0f, kFooterH - 6.0f, 3.0f);
    fillColor(full ? Color(30, 32, 38) : Color(46, 78, 60));
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();
    fillColor(full ? Color(110, 114, 122) : Color(220, 235, 225));
    fontSize(10.5f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    char fbuf[48];
    if (full) std::snprintf(fbuf, sizeof(fbuf), "Node limit reached (%d)", n);
    else std::snprintf(fbuf, sizeof(fbuf), "+ Add node  (%d/%d)", n, TransferCurve::kMaxPoints);
    text(w * 0.5f, h - kFooterH * 0.5f, fbuf, nullptr);
}

bool NodeListWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();
    const Columns c = columns();

    if (ev.press)
    {
        // Reject anything outside this widget's own rectangle before any of
        // the footer/row/column logic below runs. inFooter(y) only checks a
        // lower bound on y (deliberately, so a fast downward drag that
        // overshoots the last row still lands in the footer) and nothing
        // here ever checked x at all -- harmless with a single list, but
        // with two NodeListWidgets on screen at once (nodeList_/nodeListB_,
        // Dual-Mono/Mid-Side) DPF's dispatch (see
        // ContourGraphWidget::onMouse()'s own comment on the identical bug
        // there, same root cause and same fix) hands every subwidget a
        // press in reverse construction order until one returns true, with
        // no bounds pre-filter of its own -- an unconditional true here for
        // a position nowhere near this widget made whichever list was
        // constructed later swallow clicks meant for the OTHER list, and
        // (checked even earlier in z-order than the graphs) meant for
        // graphA_/graphB_ too, however far that click actually was (in
        // Dual-Mono/Mid-Side, points couldn't be moved at all) -- the
        // ContourGraphWidget fix alone couldn't help, since dispatch never
        // even reached the graphs.
        if (x < 0.0 || y < 0.0 || x > double(getWidth()) || y > double(getHeight())) return false;
        if (inFooter(y))
        {
            if (editRow_ >= 0) commitEdit();
            pressTarget_ = PressTarget::Add;
            return true;
        }
        // A press anywhere except on the very field being edited commits
        // that edit first, like Enter -- otherwise pressing another field
        // silently threw the typed value away. (Not the same field: its own
        // double-click-to-clear must not re-round the value first.)
        const int row = rowAt(y);
        if (row < 0) { if (editRow_ >= 0) commitEdit(); return false; }
        pressRow_ = row;
        dragging_ = false;
        auto inCol = [&](float cx, float cw) { return x >= double(cx) && x < double(cx + cw); };
        if (inCol(c.dirX, c.dirW)) pressTarget_ = PressTarget::Dir;
        else if (inCol(c.delX, c.delW)) pressTarget_ = PressTarget::Delete;
        else
        {
            pressTarget_ = PressTarget::Row;
            for (int i = 0; i < 4; ++i)
                if (inCol(c.fieldX[i], c.fieldW)) { pressTarget_ = PressTarget::Field; pressField_ = Field(i); }
        }
        if (editRow_ >= 0 && !(pressTarget_ == PressTarget::Field && row == editRow_ && pressField_ == editField_))
            commitEdit();
        if (pressTarget_ == PressTarget::Field && fieldEditable(row, pressField_))
        {
            dragStartY_ = y;
            dragStartValue_ = fieldValue(row, pressField_);
        }
        return true;
    }

    // Release.
    if (pressTarget_ == PressTarget::None) return false;
    const PressTarget target = pressTarget_;
    const int row = pressRow_;
    const Field field = pressField_;
    const bool wasDragging = dragging_;
    pressTarget_ = PressTarget::None;
    pressRow_ = -1;
    dragging_ = false;

    if (wasDragging) { repaint(); return true; } // a completed drag is not also a click

    if (target == PressTarget::Add)
    {
        if (inFooter(y)) addNodeInWidestGap();
        return true;
    }
    if (rowAt(y) != row) return true; // released off the row it was pressed on

    switch (target)
    {
    case PressTarget::Dir:
        if (!curve_.isEndpoint(row))
        {
            curve_.setPointDirection(row, PointDirection((int(curve_.pointDirection(row)) + 1) % 3));
            fireEdited();
        }
        selectRow(row);
        break;
    case PressTarget::Delete:
        if (!curve_.isEndpoint(row))
        {
            // Keep the selection on the same node where possible.
            curve_.removePoint(row);
            if (selected_ == row) selected_ = -1;
            else if (selected_ > row) --selected_;
            if (editRow_ >= 0) { editRow_ = -1; editBuffer_.clear(); }
            fireEdited();
            if (onSelectionChanged_) onSelectionChanged_(selected_);
        }
        break;
    case PressTarget::Field:
    {
        selectRow(row);
        if (!fieldEditable(row, field)) break;
        const int key = row * 4 + int(field);
        const auto now = std::chrono::steady_clock::now();
        constexpr std::chrono::milliseconds kDoubleClickWindow(400);
        const bool isDouble = key == lastClickKey_ && (now - lastClickTime_) < kDoubleClickWindow;
        beginEdit(row, field, isDouble);
        lastClickKey_ = isDouble ? -1 : key;
        lastClickTime_ = now;
        break;
    }
    case PressTarget::Row:
        selectRow(row);
        break;
    case PressTarget::Add:
    case PressTarget::None:
        break;
    }
    repaint();
    return true;
}

bool NodeListWidget::onMotion(const MotionEvent& ev)
{
    if (pressTarget_ == PressTarget::Field && pressRow_ >= 0 && fieldEditable(pressRow_, pressField_))
    {
        // Fader drag: vertical movement only, measured from the press. DGL
        // keeps delivering motion to the widget that got the press even
        // once the pointer leaves it (see ParamStripWidget's own comment).
        const double dyTotal = dragStartY_ - ev.pos.getY(); // up = positive
        if (!dragging_ && std::fabs(dyTotal) > kDragDeadZonePx)
        {
            dragging_ = true;
            if (editRow_ >= 0) cancelEdit(); // dragging the very field that was open for typing: the drag wins
            if (selected_ != pressRow_) selectRow(pressRow_);
        }
        if (dragging_)
        {
            const double perPx = ((ev.mod & kModifierAlt) != 0) ? kDbPerPixel * 0.1 : kDbPerPixel;
            applyField(pressRow_, pressField_, dragStartValue_ + dyTotal * perPx);
        }
        return true;
    }

    const int row = rowAt(ev.pos.getY());
    if (row != hoverRow_) { hoverRow_ = row; repaint(); }
    return false;
}

bool NodeListWidget::onScroll(const ScrollEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();
    // Same reasoning as onMouse()'s own bounds check: with two lists (or a
    // list and a graph) on screen at once, a claim here for a position
    // outside this widget entirely would steal the scroll from whichever
    // sibling it actually belongs to.
    if (x < 0.0 || y < 0.0 || x > double(getWidth()) || y > double(getHeight())) return false;
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    // Hover a numeric field and scroll to nudge it -- 0.5 dB per notch,
    // Alt = 10x finer (same convention as ParamStripWidget).
    const int row = rowAt(y);
    if (row >= 0)
    {
        const Columns c = columns();
        for (int i = 0; i < 4; ++i)
        {
            if (x >= double(c.fieldX[i]) && x < double(c.fieldX[i] + c.fieldW))
            {
                const Field f = Field(i);
                if (fieldEditable(row, f) && !(editRow_ == row && editField_ == f))
                {
                    const double step = ((ev.mod & kModifierAlt) != 0) ? 0.05 : 0.5;
                    applyField(row, f, fieldValue(row, f) + dir * step);
                }
                return true;
            }
        }
    }

    const int n = curve_.pointCount();
    const int maxRows = maxVisibleRows();
    if (n <= maxRows) return false;
    scrollOffset_ = std::max(0, std::min(n - maxRows, scrollOffset_ + (dir > 0.0 ? -1 : 1)));
    repaint();
    return true;
}

std::string NodeListWidget::cellTooltipAt(double x, double y) const
{
    if (inFooter(y)) return "Add a node in the middle of the widest gap, sitting exactly on the current curve.";
    if (rowAt(y) < 0) return std::string();
    const Columns c = columns();
    auto inCol = [&](float cx, float cw) { return x >= double(cx) && x < double(cx + cw); };
    if (inCol(c.numX, c.numW)) return "Node number (left to right). Click a row to select it -- it highlights on the graph too.";
    if (inCol(c.dirX, c.dirW)) return "Active: click to cycle Both / Rising / Falling. A Rising node only shapes the curve while the input level is rising, a Falling node only while it is falling (hysteresis). The two end nodes are always Both.";
    if (inCol(c.fieldX[0], c.fieldW)) return "In: this node's input level in dB -- click to type, drag up/down or scroll to change (Alt = fine). The end nodes stay at the graph edges.";
    if (inCol(c.fieldX[1], c.fieldW)) return "Out: this node's output level in dB -- click to type, drag up/down or scroll to change (Alt = fine).";
    if (inCol(c.fieldX[2], c.fieldW)) return "Support point dX/dY: where the WHITE support dot sits relative to this node, in dB (0/0 is a sharp corner). It may be negative -- the white dot then lies to the left of the node (the other, dim dot is its mirror; both describe the same tangent). Click to type, drag up/down or scroll to change.";
    if (inCol(c.fieldX[3], c.fieldW)) return "Support point dY: vertical reach of this node's Bezier support points in dB -- sets the slope of the curve through the node. Click to type, drag up/down or scroll to change.";
    if (inCol(c.delX, c.delW)) return "Delete this node.";
    return std::string();
}

END_NAMESPACE_DGL
