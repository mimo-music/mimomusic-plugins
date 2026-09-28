// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "DynEqBandListWidget2.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

using ssfilter::DynamicEqBand2;
using ssfilter::EqBandType;
using ssfilter::EqChannel;
using ssfilter::EqCrossFeed;
using ssgui::DynEqField;

namespace {

// Same palette as DynEqGraphWidget2's.
constexpr int kNumPaletteColors = 8;
constexpr int kPalette[kNumPaletteColors][3] = {
    { 235, 120, 90  }, { 100, 195, 235 }, { 235, 195, 70  }, { 140, 210, 120 },
    { 210, 130, 220 }, { 235, 150, 190 }, { 120, 160, 235 }, { 200, 200, 90  },
};

Color bandColor(int index)
{
    const int* c = kPalette[((index % kNumPaletteColors) + kNumPaletteColors) % kNumPaletteColors];
    return Color(c[0], c[1], c[2]);
}

// The band types this plugin offers -- see DynamicEqBand2::type.
EqBandType nextType(EqBandType t)
{
    switch (t)
    {
    case EqBandType::Bell:      return EqBandType::LowShelf;
    case EqBandType::LowShelf:  return EqBandType::HighShelf;
    case EqBandType::HighShelf: return EqBandType::BandShelf;
    default:                    return EqBandType::Bell;
    }
}

// Per numeric column: title, value format, drag mapping (over 200px, the
// ParamStripWidget convention: log columns multiply by `dragSpan` across
// the full distance, linear ones add `dragSpan` units) and scroll step
// (a factor for log columns, an increment for linear ones).
struct NumericSpec
{
    const char* title;
    const char* fmt;
    bool logDomain;
    double dragSpan;
    double scrollStep;
};

// Indexed by (col - kColFreq); must follow the Col enum order.
const NumericSpec kNumeric[] = {
    { "FREQ",    "%.0f",  true,  1000.0, 1.02 },
    { "BW",      "%.2f",  true,  200.0,  1.05 },
    { "GAIN",    "%+.1f", false, 48.0,   0.5  },
    { "Q",       "%.2f",  true,  2000.0, 1.08 },
    { "THRESH",  "%.1f",  false, 72.0,   0.5  },
    { "RATIO",   "%.1f",  true,  20.0,   1.05 },
    { "UP",      "%.1f",  true,  20.0,   1.05 },
    { "RANGE",   "%.1f",  false, 40.0,   0.5  },
    { "ATTACK",  "%.3g",  true,  2000.0, 1.08 },
    { "RELEASE", "%.0f",  true,  400.0,  1.08 },
};

constexpr double kDragPxFullRange = 200.0;
constexpr double kDragDeadZonePx = 3.0;

} // namespace

DynEqBandListWidget2::DynEqBandListWidget2(Widget* parent, ssgui::IDynEqEditorHost2* host)
    : NanoSubWidget(parent), host_(host)
{
    loadSharedResources();
}

DynEqBandListWidget2::ColumnLayout DynEqBandListWidget2::columns() const
{
    ColumnLayout c{};
    float x = 4.0f;
    c.swatchX = x; c.swatchW = 10.0f; x += 14.0f;
    const float widths[kNumCols] = {
        78.0f,                      // type
        20.0f, 22.0f, 26.0f, 34.0f, // EN, SOLO, DYN, CH
        56.0f, 44.0f, 46.0f, 42.0f, // FREQ, BW, GAIN, Q
        50.0f, 44.0f, 44.0f, 46.0f, 50.0f, 54.0f, // THRESH, RATIO, UP, RANGE, ATTACK, RELEASE
        20.0f,                      // delete
    };
    for (int i = 0; i < kNumCols; ++i)
    {
        // A small extra gap between the EQ and the dynamics columns.
        if (i == kColThreshold) x += 8.0f;
        c.x[i] = x;
        c.w[i] = widths[i];
        x += widths[i] + 4.0f;
    }
    return c;
}

int DynEqBandListWidget2::colAt(double x) const
{
    const ColumnLayout c = columns();
    for (int i = 0; i < kNumCols; ++i)
        if (x >= double(c.x[i]) && x < double(c.x[i] + c.w[i])) return i;
    return -1;
}

int DynEqBandListWidget2::maxVisibleRows() const
{
    return std::max(1, int(float(getHeight() - kHeaderH) / kRowH));
}

int DynEqBandListWidget2::rowAtY(double y, int& bandIndex) const
{
    if (y < double(kHeaderH)) return -1;
    const int visibleRow = int((y - double(kHeaderH)) / double(kRowH));
    if (visibleRow < 0 || visibleRow >= maxVisibleRows()) return -1;
    bandIndex = scrollOffset_ + visibleRow;
    if (bandIndex < 0 || bandIndex >= host_->getBandCount()) return -1;
    return visibleRow;
}

DynEqField DynEqBandListWidget2::fieldOf(int col)
{
    switch (col)
    {
    case kColFreq:      return DynEqField::Freq;
    case kColBandwidth: return DynEqField::Bandwidth;
    case kColGain:      return DynEqField::Gain;
    case kColQ:         return DynEqField::Q;
    case kColThreshold: return DynEqField::Threshold;
    case kColRatio:     return DynEqField::Ratio;
    case kColRatioUp:   return DynEqField::RatioUp;
    case kColRange:     return DynEqField::Range;
    case kColAttack:    return DynEqField::Attack;
    default:            return DynEqField::Release;
    }
}

bool DynEqBandListWidget2::colApplies(int col, const DynamicEqBand2& band)
{
    if (col == kColBandwidth) return band.type == EqBandType::BandShelf;
    // First-order shelves (and the Band Shelf built from two of them) have
    // a fixed slope -- Q only shapes a Bell.
    if (col == kColQ) return band.type == EqBandType::Bell;
    return true;
}

double DynEqBandListWidget2::valueOf(int col, const DynamicEqBand2& b)
{
    switch (col)
    {
    case kColFreq:      return b.freqHz;
    case kColBandwidth: return b.bandwidthOct;
    case kColGain:      return b.gainDb;
    case kColQ:         return b.q;
    case kColThreshold: return b.thresholdDb;
    case kColRatio:     return b.ratio;
    case kColRatioUp:   return b.ratioUp;
    case kColRange:     return b.rangeDb;
    case kColAttack:    return b.attackMs;
    case kColRelease:   return b.releaseMs;
    default:            return 0.0;
    }
}

void DynEqBandListWidget2::setValue(int band, int col, double v)
{
    switch (col)
    {
    case kColFreq:      host_->setBandFreq(band, v); break;
    case kColBandwidth: host_->setBandBandwidth(band, v); break;
    case kColGain:      host_->setBandGain(band, v); break;
    case kColQ:         host_->setBandQ(band, v); break;
    case kColThreshold: host_->setBandThreshold(band, v); break;
    case kColRatio:     host_->setBandRatio(band, v); break;
    case kColRatioUp:   host_->setBandRatioUp(band, v); break;
    case kColRange:     host_->setBandRange(band, v); break;
    case kColAttack:    host_->setBandAttack(band, v); break;
    case kColRelease:   host_->setBandRelease(band, v); break;
    default: break;
    }
}

void DynEqBandListWidget2::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(22, 24, 28));
    fill();

    const ssfilter::DynamicEqModel2 model = host_->getModel();
    const int bandCount = int(model.bands.size());
    const int maxRows = maxVisibleRows();
    const ColumnLayout c = columns();

    fontFace(NANOVG_DEJAVU_SANS_TTF);

    // Column titles.
    {
        fillColor(Color(130, 134, 142));
        fontSize(9.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        const float midY = kHeaderH * 0.5f;
        auto title = [&](int col, const char* t) { text(c.x[col] + c.w[col] * 0.5f, midY, t, nullptr); };
        title(kColType, "TYPE");
        title(kColEnable, "EN");
        title(kColSolo, "SOLO");
        title(kColDynamics, "DYN");
        title(kColChannel, "CH");
        for (int col = kColFreq; col <= kColRelease; ++col) title(col, kNumeric[col - kColFreq].title);

        beginPath();
        moveTo(0.0f, kHeaderH - 0.5f);
        lineTo(w, kHeaderH - 0.5f);
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();
    }

    for (int visibleRow = 0; visibleRow < maxRows; ++visibleRow)
    {
        const int i = scrollOffset_ + visibleRow;
        if (i >= bandCount) break;

        const DynamicEqBand2& b = model.bands[size_t(i)];
        const float ry = kHeaderH + float(visibleRow) * kRowH;
        const float rh = kRowH;

        if (i == selectedBand_ || i == hoverBand_)
        {
            beginPath();
            rect(0, ry, w, rh);
            fillColor(i == selectedBand_ ? Color(45, 50, 58) : Color(32, 35, 41));
            fill();
        }

        // Colour swatch.
        beginPath();
        rect(c.swatchX, ry + 4.0f, c.swatchW, rh - 8.0f);
        fillColor(b.enabled ? bandColor(i) : Color(90, 92, 96));
        fill();

        auto box = [&](int col, Color fillCol, bool outline) {
            beginPath();
            roundedRect(c.x[col] + 0.5f, ry + 3.0f, c.w[col] - 1.0f, rh - 6.0f, 3.0f);
            fillColor(fillCol);
            fill();
            if (outline)
            {
                strokeColor(Color(14, 15, 18));
                strokeWidth(1.0f);
                stroke();
            }
        };
        auto label = [&](int col, const char* t, Color col2, float size) {
            fillColor(col2);
            fontSize(size);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(c.x[col] + c.w[col] * 0.5f, ry + rh * 0.5f, t, nullptr);
        };

        // Type.
        box(kColType, Color(38, 41, 48), true);
        label(kColType, ssfilter::eqBandTypeName(b.type), Color(210, 214, 222), 10.5f);

        // Enable / Solo / Dynamics toggles.
        auto toggle = [&](int col, bool active, const char* t, Color onColor) {
            box(col, active ? onColor : Color(38, 41, 48), true);
            label(col, t, active ? Color(15, 16, 19) : Color(140, 144, 152), 10.5f);
        };
        toggle(kColEnable, b.enabled, "E", Color(90, 160, 110));
        toggle(kColSolo, model.soloedBand == i, "S", Color(210, 175, 70));
        toggle(kColDynamics, b.dynamicsEnabled, "D", Color(90, 150, 200));

        // Channel -- Both / Left / Right / L->R / R->L in one control, same
        // colours as the Parametric EQ's list.
        {
            const bool isL2R = b.crossFeed == EqCrossFeed::LeftToRight;
            const bool isR2L = b.crossFeed == EqCrossFeed::RightToLeft;
            const bool isCross = isL2R || isR2L;
            const bool isLeft = !isCross && b.channel == EqChannel::Left;
            const bool isRight = !isCross && b.channel == EqChannel::Right;
            box(kColChannel, isCross ? Color(150, 90, 200) : isLeft ? Color(70, 120, 210) : isRight ? Color(200, 80, 70) : Color(38, 41, 48), true);
            label(kColChannel, isL2R ? "L>R" : isR2L ? "R>L" : isLeft ? "L" : isRight ? "R" : "B",
                  (isLeft || isRight || isCross) ? Color(15, 16, 19) : Color(210, 214, 222), isCross ? 9.0f : 10.5f);
        }

        // Numeric fields.
        for (int col = kColFreq; col <= kColRelease; ++col)
        {
            if (!colApplies(col, b))
            {
                box(col, Color(26, 28, 33), false);
                label(col, "--", Color(90, 94, 100), 10.5f);
                continue;
            }
            std::string liveText;
            const bool editing = host_->isEditingBandField(i, fieldOf(col), liveText);
            box(col, editing ? Color(235, 195, 60) : Color(30, 32, 38), true);
            char buf[32];
            const char* toShow = buf;
            if (editing) toShow = liveText.c_str();
            else std::snprintf(buf, sizeof(buf), kNumeric[col - kColFreq].fmt, valueOf(col, b));
            // The dynamics settings are dimmed while this band's dynamics
            // are off -- still editable, they just have no effect then.
            const bool dim = col >= kColThreshold && !b.dynamicsEnabled;
            label(col, toShow, editing ? Color(20, 20, 24) : dim ? Color(110, 114, 122) : Color(215, 218, 224), 10.5f);
        }

        // Delete.
        box(kColDelete, Color(90, 45, 45), true);
        label(kColDelete, "x", Color(230, 200, 200), 11.0f);
    }

    if (bandCount == 0)
    {
        fillColor(Color(120, 124, 132));
        fontSize(11.5f);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        text(6.0f, (kHeaderH + h) * 0.5f, "No bands yet -- click the graph above to add one", nullptr);
    }
    else if (bandCount > maxRows)
    {
        fillColor(Color(140, 144, 152));
        fontSize(9.5f);
        textAlign(ALIGN_RIGHT | ALIGN_BOTTOM);
        char buf[48];
        std::snprintf(buf, sizeof(buf), "scroll for more (%d/%d)", scrollOffset_ + maxRows, bandCount);
        text(w - 4.0f, h - 2.0f, buf, nullptr);
    }
}

bool DynEqBandListWidget2::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (ev.press)
    {
        int bandIndex = -1;
        if (rowAtY(y, bandIndex) < 0) return false;
        const int col = colAt(x);
        pressBand_ = bandIndex;
        pressCol_ = col >= 0 ? col : kNumCols; // kNumCols = the row itself
        dragging_ = false;
        dragStartY_ = y;
        dragStartValue_ = isNumericCol(pressCol_) ? valueOf(pressCol_, host_->getBand(bandIndex)) : 0.0;
        return true;
    }

    // Release.
    if (pressCol_ < 0) return false;
    int bandIndex = -1;
    const bool sameBand = rowAtY(y, bandIndex) >= 0 && bandIndex == pressBand_;
    const int col = pressCol_;
    const int band = pressBand_;
    const bool wasDragging = dragging_;
    pressCol_ = -1;
    pressBand_ = -1;
    dragging_ = false;

    if (wasDragging) { repaint(); return true; } // a completed drag is not also a click
    if (!sameBand) return true;

    const DynamicEqBand2 b = host_->getBand(band);
    switch (col)
    {
    case kColType:
        host_->setBandType(band, nextType(b.type));
        break;
    case kColEnable:
        host_->setBandEnabled(band, !b.enabled);
        break;
    case kColSolo:
        // One soloed band at a time (DynamicEqModel2::soloedBand): S on
        // another row moves the solo there, S on the soloed row clears it.
        host_->setBandSolo(band, host_->getModel().soloedBand != band);
        break;
    case kColDynamics:
        host_->setBandDynamicsEnabled(band, !b.dynamicsEnabled);
        break;
    case kColChannel:
    {
        EqChannel nextCh; EqCrossFeed nextCf;
        ssfilter::advanceChannelCrossFeed(b.channel, b.crossFeed, nextCh, nextCf);
        if (nextCf != b.crossFeed) host_->setBandCrossFeed(band, nextCf);
        if (nextCh != b.channel) host_->setBandChannel(band, nextCh);
        break;
    }
    case kColDelete:
        // host_->removeBand() corrects the selection itself (UIDynEq2
        // pushes it back via setSelectedBand()).
        host_->removeBand(band);
        break;
    case kNumCols: // the row itself
        selectedBand_ = band;
        if (onSelectionChanged_) onSelectionChanged_(band);
        break;
    default: // numeric
    {
        if (!colApplies(col, b)) break; // the "--" placeholder
        const int fieldKey = band * kNumCols + col;
        const auto now = std::chrono::steady_clock::now();
        constexpr std::chrono::milliseconds kDoubleClickWindow(400);
        const bool isDoubleClick = fieldKey == lastClickField_ && (now - lastClickTime_) < kDoubleClickWindow;
        host_->beginBandTextEdit(band, fieldOf(col), isDoubleClick);
        lastClickField_ = isDoubleClick ? -1 : fieldKey;
        lastClickTime_ = now;
        break;
    }
    }
    repaint();
    return true;
}

bool DynEqBandListWidget2::onMotion(const MotionEvent& ev)
{
    if (pressBand_ >= 0 && pressBand_ < host_->getBandCount() && isNumericCol(pressCol_)
        && colApplies(pressCol_, host_->getBand(pressBand_)))
    {
        // Vertical fader drag from the press position (up = increase),
        // continuing even outside this widget -- see ParamStripWidget.
        const double dy = dragStartY_ - ev.pos.getY();
        if (!dragging_ && std::fabs(dy) > kDragDeadZonePx) dragging_ = true;
        if (dragging_)
        {
            const NumericSpec& s = kNumeric[pressCol_ - kColFreq];
            const double t = dy / kDragPxFullRange;
            const double v = s.logDomain ? std::max(1.0e-6, dragStartValue_) * std::pow(s.dragSpan, t)
                                         : dragStartValue_ + t * s.dragSpan;
            setValue(pressBand_, pressCol_, v);
            repaint();
        }
        return true;
    }

    int bandIndex = -1;
    const int newHover = rowAtY(ev.pos.getY(), bandIndex) >= 0 ? bandIndex : -1;
    if (newHover != hoverBand_) { hoverBand_ = newHover; repaint(); }
    return false;
}

bool DynEqBandListWidget2::onScroll(const ScrollEvent& ev)
{
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    // Over a numeric field: nudge it (not while it is being typed into).
    int bandIndex = -1;
    if (rowAtY(ev.pos.getY(), bandIndex) >= 0)
    {
        const int col = colAt(ev.pos.getX());
        if (isNumericCol(col))
        {
            const DynamicEqBand2 b = host_->getBand(bandIndex);
            std::string liveText;
            if (colApplies(col, b) && !host_->isEditingBandField(bandIndex, fieldOf(col), liveText))
            {
                // Alt = 10x finer: a tenth of the percentage for factors,
                // a tenth of the increment for linear steps.
                const bool fine = (ev.mod & kModifierAlt) != 0;
                const NumericSpec& s = kNumeric[col - kColFreq];
                const double v = valueOf(col, b);
                if (s.logDomain)
                {
                    const double factor = fine ? 1.0 + (s.scrollStep - 1.0) / 10.0 : s.scrollStep;
                    setValue(bandIndex, col, dir > 0.0 ? v * factor : v / factor);
                }
                else
                {
                    setValue(bandIndex, col, v + dir * (fine ? s.scrollStep / 10.0 : s.scrollStep));
                }
            }
            repaint();
            return true;
        }
    }

    const int bandCount = host_->getBandCount();
    const int maxRows = maxVisibleRows();
    if (bandCount <= maxRows) return false;
    scrollOffset_ = std::max(0, std::min(bandCount - maxRows, scrollOffset_ + (dir > 0.0 ? -1 : 1)));
    repaint();
    return true;
}

std::string DynEqBandListWidget2::cellTooltipAt(double x, double y) const
{
    int bandIndex = -1;
    if (rowAtY(y, bandIndex) < 0) return std::string();
    const DynamicEqBand2 b = host_->getBand(bandIndex);
    switch (colAt(x))
    {
    case kColType: return "Type: click to cycle Bell, Low Shelf, High Shelf, Band Shelf.";
    case kColEnable: return "Enable: turn this band on or off.";
    case kColSolo: return "Solo: hear only this band (one band at a time).";
    case kColDynamics: return "Dynamics: on, the band's gain moves with the level in its region; off, it is a static EQ band.";
    case kColChannel: return "Channel: click to cycle Both / Left / Right / L->R / R->L. L->R and R->L feed this band's filtered signal into the OTHER channel.";
    case kColFreq: return b.type == EqBandType::BandShelf ? "Freq: this Band Shelf's centre frequency -- click to type, drag up/down, scroll to step."
                                                         : "Freq: this band's frequency -- click to type, drag up/down, scroll to step.";
    case kColBandwidth: return b.type == EqBandType::BandShelf ? "BW: this Band Shelf's width in octaves -- the gain applies across this range around Freq, 0dB outside."
                                                              : "BW: width in octaves, only used by the Band Shelf type.";
    case kColGain: return "Gain: the band's resting boost/cut, before dynamics move it.";
    case kColQ: return b.type == EqBandType::Bell ? "Q: how narrow or wide the bell is."
                                                  : "Q: only used by the Bell type (the shelves have a fixed slope).";
    case kColThreshold: return "Threshold: the level (in this band's own region) where the dynamics start.";
    case kColRatio: return "Ratio: how strongly the gain is pulled back above the threshold.";
    case kColRatioUp: return "Up Ratio: how strongly the gain is pushed further below the threshold (1.0 = off).";
    case kColRange: return "Range: the most the dynamics may move the gain, in either direction (0 = static).";
    case kColAttack: return "Attack (ms): how quickly the dynamics react when the threshold is crossed.";
    case kColRelease: return "Release (ms): how quickly the dynamics relax again.";
    case kColDelete: return "Delete this band.";
    default: return std::string();
    }
}

END_NAMESPACE_DGL
