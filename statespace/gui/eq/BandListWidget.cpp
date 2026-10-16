// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "BandListWidget.hpp"

#include "../../plugin/dsp/EqModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

using ssfilter::EqBand;
using ssfilter::EqBandType;
using ssfilter::EqChannel;
using ssgui::EqField;

namespace {

// Same palette as EqGraphWidget's, kept in sync by eye (both are small,
// self-contained tables -- a shared header for 8 RGB triples isn't worth
// the indirection).
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

EqBandType nextType(EqBandType t)
{
    switch (t)
    {
    case EqBandType::Bell:      return EqBandType::LowShelf;
    case EqBandType::LowShelf:  return EqBandType::HighShelf;
    case EqBandType::HighShelf: return EqBandType::BandShelf;
    case EqBandType::BandShelf: return EqBandType::LowPass;
    case EqBandType::LowPass:   return EqBandType::HighPass;
    case EqBandType::HighPass:  return EqBandType::Notch;
    case EqBandType::Notch:     return EqBandType::BandPass;
    case EqBandType::BandPass:  return EqBandType::AllPass;
    case EqBandType::AllPass:   return EqBandType::Bell;
    }
    return EqBandType::Bell;
}

// Low/High Pass have no Gain (see eqBandTypeHasGain()), so their steepness
// -- the other fixed-option-set parameter besides Type -- takes over that
// field slot instead of leaving it a dead "--" placeholder. Cycles the same
// 1/2/3/4 (12/24/36/48dB/oct) range EqGraphWidget's right-click "Slope"
// context-menu rows offer, just reachable with a plain click here too.
bool isSlopeType(EqBandType t)
{
    return t == EqBandType::LowPass || t == EqBandType::HighPass;
}

int nextSlopeStages(int stages)
{
    return stages >= 4 ? 1 : stages + 1;
}

int prevSlopeStages(int stages)
{
    return stages <= 1 ? 4 : stages - 1;
}

// advanceChannelCrossFeed() (the Channel column's own unified 5-state
// cycle, Both -> Left -> Right -> L->R -> R->L -> Both) moved to
// EqModel.hpp -- shared with the Dynamic EQ 2 plugin's own single
// Channel button now, see that header's own comment.
using ssfilter::advanceChannelCrossFeed;

} // namespace

BandListWidget::BandListWidget(NanoTopLevelWidget* parent, ssgui::IEqEditorHost* host, bool stereo)
    : EditorSubWidget(parent), host_(host), stereo_(stereo)
{
    loadSharedResources();
}

int BandListWidget::maxVisibleRows() const
{
    return std::max(1, int(float(getHeight() - kHeaderH) / kRowH));
}

BandListWidget::RowLayout BandListWidget::layoutForRow(int visibleRow) const
{
    RowLayout r;
    r.y = kHeaderH + float(visibleRow) * kRowH;
    r.h = kRowH;

    float x = 4.0f;
    auto place = [&](float& fx, float& fw, float w) { fx = x; fw = w; x += w + 4.0f; };
    place(r.swatchX, r.swatchW, 10.0f);
    place(r.typeX, r.typeW, 82.0f);
    place(r.enableX, r.enableW, 20.0f);
    place(r.soloX, r.soloW, 20.0f);
    // Widened from 24 to fit "L>R"/"R>L" once crossfeed's own two extra
    // cycle states are folded into this SAME column (see this file's own
    // constructor comment), rather than a separate crossfeed column next
    // to it.
    if (stereo_) place(r.channelX, r.channelW, host_->supportsCrossFeed() ? 34.0f : 24.0f); // else left at 0/0 -- never laid out, never hit-tested, see this widget's own constructor comment
    place(r.freqX, r.freqW, 62.0f);
    place(r.bwX, r.bwW, 62.0f);
    place(r.gainX, r.gainW, 52.0f);
    place(r.qX, r.qW, 48.0f);
    place(r.deleteX, r.deleteW, 20.0f);
    return r;
}

int BandListWidget::rowAtY(double y, int& bandIndex) const
{
    if (y < double(kHeaderH)) return -1; // the header row itself -- no band there
    const int visibleRow = int((y - double(kHeaderH)) / double(kRowH));
    if (visibleRow < 0 || visibleRow >= maxVisibleRows()) return -1;
    bandIndex = scrollOffset_ + visibleRow;
    if (bandIndex < 0 || bandIndex >= host_->getBandCount()) return -1;
    return visibleRow;
}

void BandListWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(22, 24, 28));
    fill();

    const int bandCount = host_->getBandCount();
    const int maxRows = maxVisibleRows();

    fontFace(NANOVG_DEJAVU_SANS_TTF);

    // Column titles -- see kHeaderH's own comment for why this exists.
    // Reuses layoutForRow(0) purely for the x/w of each column; its y/h
    // are ignored, since every column's horizontal position is the same
    // regardless of which row (or the header) it's for.
    {
        const RowLayout hr = layoutForRow(0);
        fillColor(Color(130, 134, 142));
        fontSize(9.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        const float midY = kHeaderH * 0.5f;
        text(hr.typeX + hr.typeW * 0.5f, midY, "TYPE", nullptr);
        text(hr.enableX + hr.enableW * 0.5f, midY, "EN", nullptr);
        text(hr.soloX + hr.soloW * 0.5f, midY, "SOLO", nullptr);
        if (stereo_) text(hr.channelX + hr.channelW * 0.5f, midY, "CH", nullptr);
        text(hr.freqX + hr.freqW * 0.5f, midY, "FREQ", nullptr);
        text(hr.bwX + hr.bwW * 0.5f, midY, "BW (OCT)", nullptr);
        text(hr.gainX + hr.gainW * 0.5f, midY, "GAIN", nullptr);
        text(hr.qX + hr.qW * 0.5f, midY, "Q", nullptr);

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

        const EqBand b = host_->getBand(i);
        const RowLayout r = layoutForRow(visibleRow);
        const bool selected = (i == selectedBand_);
        const bool hovered = (i == hoverBand_);

        if (selected || hovered)
        {
            beginPath();
            rect(0, r.y, w, r.h);
            fillColor(selected ? Color(45, 50, 58) : Color(32, 35, 41));
            fill();
        }

        // Color swatch.
        beginPath();
        rect(r.swatchX, r.y + 4.0f, r.swatchW, r.h - 8.0f);
        fillColor(b.enabled ? bandColor(i) : Color(90, 92, 96));
        fill();

        // Type button.
        beginPath();
        roundedRect(r.typeX + 0.5f, r.y + 3.0f, r.typeW - 1.0f, r.h - 6.0f, 3.0f);
        fillColor(Color(38, 41, 48));
        fill();
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(210, 214, 222));
        fontSize(10.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(r.typeX + r.typeW * 0.5f, r.y + r.h * 0.5f, ssfilter::eqBandTypeName(b.type), nullptr);

        // Enable / Solo toggles.
        auto drawToggle = [&](float tx, float tw, bool active, const char* label, Color onColor) {
            beginPath();
            roundedRect(tx + 0.5f, r.y + 3.0f, tw - 1.0f, r.h - 6.0f, 3.0f);
            fillColor(active ? onColor : Color(38, 41, 48));
            fill();
            strokeColor(Color(14, 15, 18));
            strokeWidth(1.0f);
            stroke();
            fillColor(active ? Color(15, 16, 19) : Color(140, 144, 152));
            fontSize(10.5f);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(tx + tw * 0.5f, r.y + r.h * 0.5f, label, nullptr);
        };
        drawToggle(r.enableX, r.enableW, b.enabled, "E", Color(90, 160, 110));
        drawToggle(r.soloX, r.soloW, b.solo, "S", Color(210, 175, 70));

        // Channel -- click-to-cycle Both/Left/Right, or -- once the host
        // also opts into crossfeed -- Both/Left/Right/L->R/R->L, five
        // states in this ONE control, not a separate crossfeed column next
        // to it (which was less ergonomic). Internally these still write
        // two SEPARATE fields (EqBand::channel/crossFeed stay genuinely
        // separate -- see EqBand::crossFeed's own comment for why) via
        // advanceChannelCrossFeed() below; only the UI presents them as
        // one cycle. "B"/"L"/"R" read exactly like Type/Enable/Solo's own
        // neutral-vs-distinguishing coloring; "L>R"/"R>L" use a distinct
        // purple so crossfeed reads as a genuinely different kind of
        // routing, not a fourth same-color Channel state.
        if (stereo_)
        {
            const bool isLeft = b.channel == EqChannel::Left;
            const bool isRight = b.channel == EqChannel::Right;
            const bool isL2R = b.crossFeed == ssfilter::EqCrossFeed::LeftToRight;
            const bool isR2L = b.crossFeed == ssfilter::EqCrossFeed::RightToLeft;
            const bool isCross = isL2R || isR2L;
            beginPath();
            roundedRect(r.channelX + 0.5f, r.y + 3.0f, r.channelW - 1.0f, r.h - 6.0f, 3.0f);
            fillColor(isCross ? Color(150, 90, 200) : isLeft ? Color(70, 120, 210) : isRight ? Color(200, 80, 70) : Color(38, 41, 48));
            fill();
            strokeColor(Color(14, 15, 18));
            strokeWidth(1.0f);
            stroke();
            fillColor((isLeft || isRight || isCross) ? Color(15, 16, 19) : Color(210, 214, 222));
            fontSize(isCross ? 9.0f : 10.5f); // "L>R"/"R>L" are three characters, needs to shrink slightly to fit this column's own width
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(r.channelX + r.channelW * 0.5f, r.y + r.h * 0.5f,
                 isL2R ? "L>R" : isR2L ? "R>L" : isLeft ? "L" : isRight ? "R" : "B", nullptr);
        }

        // Freq / Gain / Q numeric fields.
        auto drawField = [&](float fx, float fw, EqField field, double value, const char* fmt) {
            std::string liveText;
            const bool editing = host_->isEditingBandField(i, field, liveText);
            beginPath();
            roundedRect(fx + 0.5f, r.y + 3.0f, fw - 1.0f, r.h - 6.0f, 3.0f);
            fillColor(editing ? Color(235, 195, 60) : Color(30, 32, 38));
            fill();
            strokeColor(Color(14, 15, 18));
            strokeWidth(1.0f);
            stroke();

            char buf[32];
            const char* toShow;
            if (editing) toShow = liveText.c_str();
            else { std::snprintf(buf, sizeof(buf), fmt, value); toShow = buf; }
            fillColor(editing ? Color(20, 20, 24) : Color(215, 218, 224));
            fontSize(10.5f);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(fx + fw * 0.5f, r.y + r.h * 0.5f, toShow, nullptr);
        };
        drawField(r.freqX, r.freqW, EqField::Freq, b.freqHz, "%.0f");
        if (ssfilter::eqBandTypeHasBandwidth(b.type))
            drawField(r.bwX, r.bwW, EqField::Bandwidth, b.bandwidthOct, "%.2f");
        else
        {
            // Same dimmed placeholder the Gain column uses for a type
            // without gain.
            beginPath();
            roundedRect(r.bwX + 0.5f, r.y + 3.0f, r.bwW - 1.0f, r.h - 6.0f, 3.0f);
            fillColor(Color(26, 28, 33));
            fill();
            fillColor(Color(90, 94, 100));
            fontSize(10.5f);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(r.bwX + r.bwW * 0.5f, r.y + r.h * 0.5f, "--", nullptr);
        }
        // A crossfeed band's own GAIN column still shows just its own
        // Bell/Shelf/etc. shape gain, same as any other band -- the
        // crossfeed transit's own overall send level is a MODEL-level
        // control now (EqGraphWidget's own L->R/R->L knobs), not
        // per-band, so there's nothing special left to show here. See
        // EqModel::crossFeedGainLtoRDb's own comment for why.
        if (ssfilter::eqBandTypeHasGain(b.type))
            drawField(r.gainX, r.gainW, EqField::Gain, b.gainDb, "%+.1f");
        else if (isSlopeType(b.type))
        {
            // Fixed-option-set control, same click-to-cycle convention as
            // the Type button -- not a text-edit field, so no drawField().
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%ddB/o", b.slopeStages * 12);
            beginPath();
            roundedRect(r.gainX + 0.5f, r.y + 3.0f, r.gainW - 1.0f, r.h - 6.0f, 3.0f);
            fillColor(Color(38, 41, 48));
            fill();
            strokeColor(Color(14, 15, 18));
            strokeWidth(1.0f);
            stroke();
            fillColor(Color(210, 214, 222));
            fontSize(10.0f);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(r.gainX + r.gainW * 0.5f, r.y + r.h * 0.5f, buf, nullptr);
        }
        else
        {
            beginPath();
            roundedRect(r.gainX + 0.5f, r.y + 3.0f, r.gainW - 1.0f, r.h - 6.0f, 3.0f);
            fillColor(Color(26, 28, 33));
            fill();
            fillColor(Color(90, 94, 100));
            fontSize(10.5f);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(r.gainX + r.gainW * 0.5f, r.y + r.h * 0.5f, "--", nullptr);
        }
        drawField(r.qX, r.qW, EqField::Q, b.q, "%.2f");

        // Delete button.
        beginPath();
        roundedRect(r.deleteX + 0.5f, r.y + 3.0f, r.deleteW - 1.0f, r.h - 6.0f, 3.0f);
        fillColor(Color(90, 45, 45));
        fill();
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(230, 200, 200));
        fontSize(11.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(r.deleteX + r.deleteW * 0.5f, r.y + r.h * 0.5f, "x", nullptr);
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

bool BandListWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (ev.press)
    {
        int bandIndex = -1;
        const int visibleRow = rowAtY(y, bandIndex);
        if (visibleRow < 0) return false;
        const RowLayout r = layoutForRow(visibleRow);
        pressBand_ = bandIndex;

        if (x >= r.typeX && x < r.typeX + r.typeW) pressTarget_ = PressTarget::Type;
        else if (x >= r.enableX && x < r.enableX + r.enableW) pressTarget_ = PressTarget::Enable;
        else if (x >= r.soloX && x < r.soloX + r.soloW) pressTarget_ = PressTarget::Solo;
        else if (stereo_ && x >= r.channelX && x < r.channelX + r.channelW) pressTarget_ = PressTarget::Channel;
        else if (x >= r.freqX && x < r.freqX + r.freqW) pressTarget_ = PressTarget::Freq;
        else if (x >= r.bwX && x < r.bwX + r.bwW) pressTarget_ = PressTarget::Bandwidth;
        else if (x >= r.gainX && x < r.gainX + r.gainW) pressTarget_ = PressTarget::Gain;
        else if (x >= r.qX && x < r.qX + r.qW) pressTarget_ = PressTarget::Q;
        else if (x >= r.deleteX && x < r.deleteX + r.deleteW) pressTarget_ = PressTarget::Delete;
        else pressTarget_ = PressTarget::Row;

        dragging_ = false;
        dragStartY_ = y;
        const EqBand pb = host_->getBand(bandIndex);
        dragStartValue_ = pressTarget_ == PressTarget::Freq ? pb.freqHz
                        : pressTarget_ == PressTarget::Bandwidth ? pb.bandwidthOct
                        : pressTarget_ == PressTarget::Gain ? pb.gainDb
                        : pb.q;
        return true;
    }

    // Release.
    if (pressTarget_ == PressTarget::None) return false;
    int bandIndex = -1;
    const int visibleRow = rowAtY(y, bandIndex);
    const bool sameBand = visibleRow >= 0 && bandIndex == pressBand_;
    const PressTarget target = pressTarget_;
    const int band = pressBand_;
    const bool wasDragging = dragging_;
    pressTarget_ = PressTarget::None;
    pressBand_ = -1;
    dragging_ = false;

    if (wasDragging) { repaint(); return true; } // a completed drag is not also a click
    if (!sameBand) return true;

    switch (target)
    {
    case PressTarget::Type:
        host_->setBandType(band, nextType(host_->getBand(band).type));
        break;
    case PressTarget::Enable:
        host_->setBandEnabled(band, !host_->getBand(band).enabled);
        break;
    case PressTarget::Solo:
        host_->setBandSolo(band, !host_->getBand(band).solo);
        break;
    case PressTarget::Channel:
    {
        // Plain Both/Left/Right cycle when the host doesn't support
        // crossfeed at all (UIEqMono) -- unchanged from before crossfeed
        // existed. Five-state cycle (see advanceChannelCrossFeed()'s own
        // comment) once it does.
        const EqBand eb = host_->getBand(band);
        if (!host_->supportsCrossFeed())
        {
            host_->setBandChannel(band, ssfilter::nextEqChannel(eb.channel));
            break;
        }
        EqChannel nextCh; ssfilter::EqCrossFeed nextCf;
        advanceChannelCrossFeed(eb.channel, eb.crossFeed, nextCh, nextCf);
        if (nextCf != eb.crossFeed) host_->setBandCrossFeed(band, nextCf);
        if (nextCh != eb.channel) host_->setBandChannel(band, nextCh);
        break;
    }
    case PressTarget::Delete:
        // host_->removeBand() (UIEq::removeBand()) already corrects
        // selectedBand_ for the shift this causes -- via a direct
        // setSelectedBand() call, from the one place that actually knows
        // the mutation happened -- and pushes the result back into
        // selectedBand_ here. Re-deriving it afterward from the
        // pre-deletion `band` value would compare against a now-stale
        // number (selectedBand_ has already moved on) and could
        // overwrite that correct fixup with a wrong one.
        host_->removeBand(band);
        break;
    case PressTarget::Freq:
    case PressTarget::Bandwidth:
    case PressTarget::Gain:
    case PressTarget::Q:
    {
        if (target == PressTarget::Bandwidth && !ssfilter::eqBandTypeHasBandwidth(host_->getBand(band).type))
            break; // the "--" placeholder -- nothing to edit for this type
        if (target == PressTarget::Gain && !ssfilter::eqBandTypeHasGain(host_->getBand(band).type))
        {
            // Not a text-edit field for this band's type: either the
            // click-to-cycle steepness control (Low/High Pass) or a dead
            // "--" placeholder (Notch/Band Pass/All Pass).
            const EqBand& eb = host_->getBand(band);
            if (isSlopeType(eb.type))
                host_->setBandSlopeStages(band, nextSlopeStages(eb.slopeStages));
            break;
        }
        const EqField field = target == PressTarget::Freq ? EqField::Freq
                             : target == PressTarget::Bandwidth ? EqField::Bandwidth
                             : target == PressTarget::Gain ? EqField::Gain : EqField::Q;
        const int fieldKey = band * 4 + int(field);
        const auto now = std::chrono::steady_clock::now();
        constexpr std::chrono::milliseconds kDoubleClickWindow(400);
        const bool isDoubleClick = fieldKey == lastClickField_ && (now - lastClickTime_) < kDoubleClickWindow;
        host_->beginBandTextEdit(band, field, isDoubleClick);
        lastClickField_ = isDoubleClick ? -1 : fieldKey;
        lastClickTime_ = now;
        break;
    }
    case PressTarget::Row:
        selectedBand_ = band;
        if (onSelectionChanged_) onSelectionChanged_(band);
        break;
    case PressTarget::None:
        break;
    }
    repaint();
    return true;
}

bool BandListWidget::isValueField(PressTarget target, const EqBand& band) const
{
    switch (target)
    {
    case PressTarget::Freq:
    case PressTarget::Q:     return true;
    case PressTarget::Bandwidth: return ssfilter::eqBandTypeHasBandwidth(band.type);
    case PressTarget::Gain:  return ssfilter::eqBandTypeHasGain(band.type);
    default:                 return false;
    }
}

bool BandListWidget::onMotion(const MotionEvent& ev)
{
    if (pressBand_ >= 0 && pressBand_ < host_->getBandCount() && isValueField(pressTarget_, host_->getBand(pressBand_)))
    {
        // Vertical only, from the press position, even once the pointer
        // leaves this widget (see ParamStripWidget's class comment for why
        // that works in DGL). 200px sweeps the range, as there: Freq and Q
        // in the log domain (20Hz-20kHz, 0.05-100), Gain linearly over
        // 48dB (the -24..+24 range the Dynamic EQ strips use).
        constexpr double kDeadZonePx = 3.0, kPxFullRange = 200.0;
        const double dy = dragStartY_ - ev.pos.getY(); // up = increase
        if (!dragging_ && std::fabs(dy) > kDeadZonePx) dragging_ = true;
        if (dragging_)
        {
            const double t = dy / kPxFullRange;
            const double v0 = std::max(1.0e-6, dragStartValue_);
            switch (pressTarget_)
            {
            case PressTarget::Freq:  host_->setBandFreq(pressBand_, v0 * std::pow(1000.0, t)); break;
            case PressTarget::Bandwidth: host_->setBandBandwidth(pressBand_, v0 * std::pow(200.0, t)); break; // 0.05-10 octaves, log domain
            case PressTarget::Q:     host_->setBandQ(pressBand_, v0 * std::pow(2000.0, t)); break;
            case PressTarget::Gain:  host_->setBandGain(pressBand_, dragStartValue_ + t * 48.0); break;
            default: break;
            }
            repaint();
        }
        return true;
    }

    int bandIndex = -1;
    const int visibleRow = rowAtY(ev.pos.getY(), bandIndex);
    const int newHover = visibleRow >= 0 ? bandIndex : -1;
    if (newHover != hoverBand_) { hoverBand_ = newHover; repaint(); }
    return false;
}

bool BandListWidget::onScroll(const ScrollEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    // Hovering directly over a band's own Freq/Gain/Q field nudges that
    // value -- the same "hover and scroll" convention every other
    // numeric field in this project already has (ParamStripWidget's
    // fields, and this same plugin's own EqGraphWidget node-scroll-to-
    // adjust-Q), which this widget's own header comment already claimed
    // ("Enter/Tab/Escape/scroll-to-nudge -- same conventions as every
    // other numeric field in this project") without actually being wired
    // up. Falls through to the pre-existing
    // list-scroll behavior below when the pointer isn't over one of
    // these three columns, or while that exact field is mid-text-edit
    // (matching ParamStripWidget::onScroll()'s identical guard).
    int bandIndex = -1;
    const int visibleRow = rowAtY(y, bandIndex);
    if (visibleRow >= 0)
    {
        const RowLayout r = layoutForRow(visibleRow);
        const EqBand b = host_->getBand(bandIndex);

        // Alt+scroll everywhere in this project now means "10x finer" --
        // see ParamStripWidget::onScroll()'s identical comment for the
        // full reasoning (applied project-wide).
        // For a multiplicative step, "10x finer" means the *percentage*
        // change per notch shrinks by 10x, not the raw factor divided by
        // 10 (which would barely change anything close to 1.0).
        const bool fine = (ev.mod & kModifierAlt) != 0;
        auto scaledFactor = [fine](double factor) { return fine ? 1.0 + (factor - 1.0) / 10.0 : factor; };
        auto scaledStep = [fine](double step) { return fine ? step / 10.0 : step; };

        // Freq's own step was too big at 1.08 (an 8% jump per notch,
        // noticeable even before Alt) -- reduced to a gentler 2%,
        // independent of Q's own factor (left unchanged) since a wide, unbounded 20Hz-20kHz range and a
        // 0.05-100 Q range don't need to feel identical to nudge.
        constexpr double kFreqFactor = 1.02;
        // Same factor EqGraphWidget's own node Q-scroll already uses.
        constexpr double kQFactor = 1.08;
        std::string liveText; // unused -- isEditingBandField() just needs somewhere to write it

        if (x >= r.freqX && x < r.freqX + r.freqW)
        {
            const double factor = scaledFactor(kFreqFactor);
            if (!host_->isEditingBandField(bandIndex, EqField::Freq, liveText))
                host_->setBandFreq(bandIndex, dir > 0.0 ? b.freqHz * factor : b.freqHz / factor);
            repaint();
            return true;
        }
        if (x >= r.bwX && x < r.bwX + r.bwW)
        {
            const double factor = scaledFactor(1.05); // 5% per notch: 2 octaves -> 2.1
            if (ssfilter::eqBandTypeHasBandwidth(b.type) && !host_->isEditingBandField(bandIndex, EqField::Bandwidth, liveText))
                host_->setBandBandwidth(bandIndex, dir > 0.0 ? b.bandwidthOct * factor : b.bandwidthOct / factor);
            repaint();
            return true;
        }
        if (x >= r.gainX && x < r.gainX + r.gainW)
        {
            if (!ssfilter::eqBandTypeHasGain(b.type))
            {
                // Low/High Pass's Gain slot is the Slope control instead
                // (see onMouse()'s identical special case) -- scroll
                // cycles the same four steepness choices, one step per
                // notch regardless of Alt, since there's no finer
                // resolution a discrete stepped value can offer.
                if (isSlopeType(b.type))
                    host_->setBandSlopeStages(bandIndex, dir > 0.0 ? nextSlopeStages(b.slopeStages)
                                                                    : prevSlopeStages(b.slopeStages));
                repaint();
                return true;
            }
            if (!host_->isEditingBandField(bandIndex, EqField::Gain, liveText))
                host_->setBandGain(bandIndex, b.gainDb + dir * scaledStep(0.5)); // 0.5dB base step, same as every other Gain field in this project
            repaint();
            return true;
        }
        if (x >= r.qX && x < r.qX + r.qW)
        {
            const double factor = scaledFactor(kQFactor);
            if (!host_->isEditingBandField(bandIndex, EqField::Q, liveText))
                host_->setBandQ(bandIndex, dir > 0.0 ? b.q * factor : b.q / factor);
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

std::string BandListWidget::cellTooltipAt(double x, double y) const
{
    int bandIndex;
    const int visibleRow = rowAtY(y, bandIndex);
    if (visibleRow < 0) return std::string();
    const RowLayout r = layoutForRow(visibleRow);
    auto inCol = [&](float cx, float cw) { return x >= double(cx) && x < double(cx + cw); };
    if (inCol(r.typeX, r.typeW)) return "Filter Type: click to cycle this band's own filter shape (Bell, Shelf, Highpass, ...).";
    if (inCol(r.enableX, r.enableW)) return "Enable: turn this band on or off.";
    if (inCol(r.soloX, r.soloW)) return "Solo: mute every other band, hear only this one.";
    if (stereo_ && inCol(r.channelX, r.channelW))
    {
        return host_->supportsCrossFeed()
            ? "Channel: click to cycle Both / Left / Right / L->R / R->L. L->R and R->L feed this band's own filtered output into the OTHER channel instead of its own."
            : "Channel: click to cycle Both / Left only / Right only.";
    }
    if (inCol(r.freqX, r.freqW))
    {
        return ssfilter::eqBandTypeHasBandwidth(host_->getBand(bandIndex).type)
            ? "Freq: this Band Shelf's centre frequency -- click to type, drag up/down, scroll to step."
            : "Freq: this band's own center frequency -- click to type, drag up/down, scroll to step.";
    }
    if (inCol(r.bwX, r.bwW))
    {
        return ssfilter::eqBandTypeHasBandwidth(host_->getBand(bandIndex).type)
            ? "BW: this Band Shelf's width in octaves -- the gain applies across this range around Freq, 0dB outside. Click to type, drag up/down, scroll to step."
            : "BW: bandwidth in octaves, only used by the Band Shelf type.";
    }
    if (inCol(r.gainX, r.gainW))
    {
        return host_->getBand(bandIndex).crossFeed != ssfilter::EqCrossFeed::None
            ? "Gain: this band's own boost/cut, riding on top of the L->R/R->L transit's own overall send level -- see the graph's own L->R/R->L control, top-right."
            : "Gain: this band's own boost/cut.";
    }
    if (inCol(r.qX, r.qW)) return "Q: how narrow or wide this band's own shape is.";
    if (inCol(r.deleteX, r.deleteW)) return "Delete this band.";
    return std::string();
}

END_NAMESPACE_DGL
