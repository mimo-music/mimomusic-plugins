// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "SidechainScopeWidget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

START_NAMESPACE_DGL

using sscomp::SpectrumAnalyzer;

SidechainScopeWidget::SidechainScopeWidget(NanoTopLevelWidget* parent)
    : EditorSubWidget(parent)
{
    loadSharedResources();
    std::fill(std::begin(spectrumInDb_), std::end(spectrumInDb_), SpectrumAnalyzer::kFloorDb);
    std::fill(std::begin(spectrumOutDb_), std::end(spectrumOutDb_), SpectrumAnalyzer::kFloorDb);
    std::fill(std::begin(historyL_), std::end(historyL_), -120.0f);
    std::fill(std::begin(historyR_), std::end(historyR_), -120.0f);
}

void SidechainScopeWidget::setSpectrumInPacket(int packetIndex, float packedValue)
{
    if (packetIndex < 0 || packetIndex >= SpectrumAnalyzer::kNumPackedBands) return;
    float triple[SpectrumAnalyzer::kBandsPerPacket];
    SpectrumAnalyzer::unpackTriple(packedValue, triple);
    for (int k = 0; k < SpectrumAnalyzer::kBandsPerPacket; ++k)
        spectrumInDb_[packetIndex * SpectrumAnalyzer::kBandsPerPacket + k] = triple[k];
    repaint();
}

void SidechainScopeWidget::setSpectrumOutPacket(int packetIndex, float packedValue)
{
    if (packetIndex < 0 || packetIndex >= SpectrumAnalyzer::kNumPackedBands) return;
    float triple[SpectrumAnalyzer::kBandsPerPacket];
    SpectrumAnalyzer::unpackTriple(packedValue, triple);
    for (int k = 0; k < SpectrumAnalyzer::kBandsPerPacket; ++k)
        spectrumOutDb_[packetIndex * SpectrumAnalyzer::kBandsPerPacket + k] = triple[k];
    repaint();
}

void SidechainScopeWidget::pushLevelDbL(float db)
{
    // Overwrites the current (not-yet-committed) head slot only -- see
    // tickLevelHistory()'s own comment for why advancing the buffer
    // itself is a separate, timer-driven step, not tied to this call.
    historyL_[historyPos_] = db;
}

void SidechainScopeWidget::pushLevelDbR(float db)
{
    historyR_[historyPos_] = db;
}

void SidechainScopeWidget::tickLevelHistory()
{
    historyPos_ = (historyPos_ + 1) % kHistoryLen;
    // The slot we just moved onto still holds whatever it held the last
    // time the ring wrapped around (stale by kHistoryLen ticks) --
    // reseed it with the current head's own just-committed value rather
    // than leaving it visibly stale for one frame, then repaint so the
    // scroll is actually visible even if neither pushLevelDb call fires
    // again before the next tick.
    const int prev = (historyPos_ + kHistoryLen - 1) % kHistoryLen;
    historyL_[historyPos_] = historyL_[prev];
    historyR_[historyPos_] = historyR_[prev];
    repaint();
}

void SidechainScopeWidget::setFilterRange(bool enabled, float highpassHz, float lowpassHz)
{
    filterEnabled_ = enabled;
    filterHighpassHz_ = highpassHz;
    filterLowpassHz_ = lowpassHz;
    repaint();
}

void SidechainScopeWidget::setEditingRight(bool editingRight)
{
    if (editingRight_ == editingRight) return;
    editingRight_ = editingRight;
    repaint();
}

float SidechainScopeWidget::freqToX(double hz) const
{
    const double t = std::log(hz / kFMin) / std::log(kFMax / kFMin);
    return specX_ + float(std::max(0.0, std::min(1.0, t))) * specW_;
}

double SidechainScopeWidget::xToFreq(float px) const
{
    const double t = double(px - specX_) / double(std::max(1.0f, specW_));
    return kFMin * std::pow(kFMax / kFMin, std::max(0.0, std::min(1.0, t)));
}

int SidechainScopeWidget::handleAt(double px, double py) const
{
    if (py < specY_ || py >= specY_ + specH_) return -1;
    const float hpX = freqToX(std::max(20.0, double(filterHighpassHz_)));
    const float lpX = freqToX(std::min(20000.0, double(filterLowpassHz_)));
    const float dHp = std::fabs(float(px) - hpX);
    const float dLp = std::fabs(float(px) - lpX);
    if (dHp <= kHandleHitPx && dHp <= dLp) return 0;
    if (dLp <= kHandleHitPx) return 1;
    return -1;
}

bool SidechainScopeWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;

    if (ev.press)
    {
        const int h = handleAt(ev.pos.getX(), ev.pos.getY());
        if (h < 0) return false;
        dragHandle_ = h;
        repaint();
        return true;
    }

    if (dragHandle_ >= 0)
    {
        dragHandle_ = -1;
        repaint();
        return true;
    }
    return false;
}

bool SidechainScopeWidget::onMotion(const MotionEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (dragHandle_ >= 0)
    {
        const float hz = float(xToFreq(float(x)));
        if (dragHandle_ == 0)
        {
            filterHighpassHz_ = hz;
            if (onHighpassChanged_) onHighpassChanged_(hz);
        }
        else
        {
            filterLowpassHz_ = hz;
            if (onLowpassChanged_) onLowpassChanged_(hz);
        }
        repaint();
        return true;
    }

    const int h = handleAt(x, y);
    if (h != hoverHandle_) { hoverHandle_ = h; repaint(); }
    return false;
}

void SidechainScopeWidget::drawSpectrum(float x, float y, float w, float h)
{
    specX_ = x; specY_ = y; specW_ = w; specH_ = h;

    beginPath();
    rect(x, y, w, h);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    auto dbToY = [&](float db) {
        const float t = std::max(0.0f, std::min(1.0f, (kSpectrumCeilDb - db) / (kSpectrumCeilDb - kSpectrumFloorDb)));
        return y + t * h;
    };

    // Frequency gridlines, same set EqGraphWidget's own spectrum uses.
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    for (double f : { 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0 })
    {
        const float px = freqToX(f);
        beginPath();
        moveTo(px, y);
        lineTo(px, y + h);
        strokeColor(Color(30, 32, 37));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        char buf[16];
        if (f >= 1000.0) std::snprintf(buf, sizeof(buf), "%.0fk", f / 1000.0);
        else std::snprintf(buf, sizeof(buf), "%.0f", f);
        text(px, y + h + 2.0f, buf, nullptr);
    }

    // The filter's own passband, shown in the spectrum. A shaded region between the two
    // corners, only while the filter is actually engaged (an unfiltered
    // spectrum has no "band" to highlight) -- the draggable handles
    // themselves, drawn further below, stay visible/interactive either
    // way. Red while the corners currently live are R's own
    // (editingRight_), matching R's own red level trace below, instead of
    // always the same neutral green regardless of which side is live.
    const float hpX = freqToX(std::max(20.0, double(filterHighpassHz_)));
    const float lpX = freqToX(std::min(20000.0, double(filterLowpassHz_)));
    if (filterEnabled_)
    {
        beginPath();
        rect(std::min(hpX, lpX), y, std::max(2.0f, std::fabs(lpX - hpX)), h);
        fillColor(editingRight_ ? Color(220, 90, 80, 0.16f) : Color(90, 190, 150, 0.14f));
        fill();
    }

    // Two live spectrum traces -- pre-filter ("In") and post-filter/
    // current-signal ("Out"), always both, like on the EQs. Same log-x/dB-y
    // mapping,
    // drawing shape, and In=warm-orange/Out=cool-blue convention as
    // ../../statespace's own EqGraphWidget::drawTrace(). Out drawn last
    // (on top, filled) so it stays the visually dominant trace, matching
    // that widget's own "Output... stays the visually dominant curve"
    // reasoning.
    auto drawTrace = [&](const float* db, Color strokeCol, Color fillCol, bool doFill) {
        beginPath();
        for (int b = 0; b < SpectrumAnalyzer::kNumBands; ++b)
        {
            const double t = double(b) / double(SpectrumAnalyzer::kNumBands - 1);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const float px = freqToX(f);
            const float py = dbToY(db[b]);
            if (b == 0) moveTo(px, py); else lineTo(px, py);
        }
        if (doFill)
        {
            lineTo(x + w, y + h);
            lineTo(x, y + h);
            closePath();
            fillColor(fillCol);
            fill();
        }
        strokeColor(strokeCol);
        strokeWidth(1.0f);
        stroke();
    };
    const Color kInStroke(235, 165, 95, 0.65f);
    const Color kInFill(220, 140, 70, 0.16f);
    const Color kOutStroke(110, 180, 220, 0.85f);
    const Color kOutFill(80, 150, 200, 0.22f);
    drawTrace(spectrumInDb_, kInStroke, kInFill, false);
    drawTrace(spectrumOutDb_, kOutStroke, kOutFill, true);

    // Small legend, top-left.
    {
        float lx = x + 6.0f, ly = y + 6.0f;
        fontSize(9.5f);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        auto legendItem = [&](Color dot, const char* label) {
            beginPath();
            circle(lx + 3.0f, ly + 5.0f, 3.0f);
            fillColor(dot);
            fill();
            fillColor(Color(200, 204, 210, 0.9f));
            text(lx + 10.0f, ly + 5.0f, label, nullptr);
            lx += 10.0f + float(std::strlen(label)) * 5.6f + 10.0f;
        };
        legendItem(kInStroke, "In");
        legendItem(kOutStroke, "Out");
    }

    // Draggable high-pass/low-pass corner handles, with the frequency shown
    // right there, like the Multiband Compressor's -- ported
    // from ../../statespace's own MultibandGraphWidget crossover-handle
    // drawing (vertical line + triangular knob at the top, brighter
    // while hovered/dragged).
    auto drawHandle = [&](float hx, int idx) {
        const bool active = (idx == dragHandle_) || (idx == hoverHandle_);
        const Color restColor = editingRight_ ? Color(230, 130, 120) : Color(200, 202, 206);
        const Color activeColor = editingRight_ ? Color(255, 160, 145) : Color(255, 255, 255);
        beginPath();
        moveTo(hx, y);
        lineTo(hx, y + h);
        strokeColor(active ? activeColor.withAlpha(0.9f) : restColor.withAlpha(0.55f));
        strokeWidth(active ? 2.0f : 1.3f);
        stroke();

        beginPath();
        moveTo(hx - 5.0f, y);
        lineTo(hx + 5.0f, y);
        lineTo(hx, y + 8.0f);
        closePath();
        fillColor(active ? activeColor : restColor);
        fill();
    };
    drawHandle(hpX, 0);
    drawHandle(lpX, 1);

    // Frequency readout near whichever handle is being dragged or merely
    // hovered -- same "show it while pointing, not just while dragging"
    // convention MultibandGraphWidget's own crossover readout uses.
    const int readoutHandle = dragHandle_ >= 0 ? dragHandle_ : hoverHandle_;
    if (readoutHandle == 0 || readoutHandle == 1)
    {
        const double freqHz = readoutHandle == 0 ? double(filterHighpassHz_) : double(filterLowpassHz_);
        const float hx = readoutHandle == 0 ? hpX : lpX;

        char buf[32];
        char freqBuf[24];
        if (freqHz >= 1000.0) std::snprintf(freqBuf, sizeof(freqBuf), "%.2fkHz", freqHz / 1000.0);
        else std::snprintf(freqBuf, sizeof(freqBuf), "%.0fHz", freqHz);
        std::snprintf(buf, sizeof(buf), "%s %s", readoutHandle == 0 ? "HP" : "LP", freqBuf);

        fontSize(11.5f);
        const float textW = float(std::strlen(buf)) * 6.4f + 14.0f;
        const float textH = 18.0f;
        float lx = std::max(x, std::min(x + w - textW, hx - textW * 0.5f));
        const float ly = y + 12.0f;

        beginPath();
        roundedRect(lx, ly, textW, textH, 4.0f);
        fillColor(Color(20, 21, 25, 0.92f));
        fill();
        strokeColor(Color(70, 74, 82, 0.9f));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(235, 237, 240));
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(lx + textW * 0.5f, ly + textH * 0.5f + 0.5f, buf, nullptr);
    }
}

void SidechainScopeWidget::drawLevelTrace(float x, float y, float w, float h)
{
    beginPath();
    rect(x, y, w, h);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    // Linear amplitude, clamped to kLevelMaxLin -- reverting an
    // intermediate dB-extent mapping back to the original linear-amplitude
    // look (which read better), while still fixing a real bug: with a big
    // gain the waveform overlapped the plot, since the *original* version
    // had no clamp at all, so a heavily Sidechain-Gain-boosted
    // signal's linear amplitude could exceed 1.0 and draw past this
    // widget's own bounds). The clamp alone is the fix -- normalized
    // against the same ceiling the grid below is labeled against, so a
    // peak's height and the grid's own dB text always agree.
    auto norm = [&](float db) {
        const float lin = std::pow(10.0f, db / 20.0f);
        return std::max(0.0f, std::min(1.0f, lin / kLevelMaxLin));
    };

    const float baseline = y + h * 0.5f;
    const float halfSpan = h * 0.48f;

    // Y-axis dB grid lines, to read levels off when setting the threshold.
    // Fixed dB
    // values, each positioned using the *same* linear-amplitude mapping
    // the envelope itself is drawn with (so reading a peak against a
    // label is always accurate) -- spacing is therefore non-uniform
    // (denser near the baseline, sparser near the edges), the expected
    // shape for a linear-scale meter with dB tick marks, same convention
    // real peak/VU meters use. Mirrored above and below the baseline (the
    // upper half serves L, the lower half R). Only the *lines* are drawn
    // here, before the channel fills below -- the dB text labels
    // themselves are drawn afterward, on top of everything (see the
    // bottom of this function), because drawing them here (underneath
    // the semi-transparent channel fills) made the y axis hard to read: a
    // loud envelope's own fill
    // routinely painted right over the label text sitting at that same
    // height.
    constexpr float kGridDbValues[] = { 12.0f, 6.0f, 0.0f, -6.0f, -12.0f, -18.0f, -24.0f, -36.0f, -48.0f, -60.0f };
    for (float db : kGridDbValues)
    {
        if (db < kLevelGridMinDb || db > kLevelGridMaxDb) continue;
        const float e = norm(db);
        const float pyUp = baseline - e * halfSpan;
        const float pyDown = baseline + e * halfSpan;
        beginPath();
        moveTo(x, pyUp); lineTo(x + w, pyUp);
        moveTo(x, pyDown); lineTo(x + w, pyDown);
        strokeColor(db == 0.0f ? Color(60, 64, 70) : Color(30, 32, 37));
        strokeWidth(1.0f);
        stroke();
    }

    // The two rectified level envelopes -- blue (left) on top, mirrored red
    // (right) below: genuine per-channel
    // data now (kParamMeterScLevelDbL/R), not a cosmetic mirror of one
    // mono value.
    auto drawChannel = [&](const float* history, bool upward, Color strokeCol, Color fillCol)
    {
        beginPath();
        for (int i = 0; i < kHistoryLen; ++i)
        {
            const int idx = (historyPos_ + i) % kHistoryLen;
            const float e = norm(history[idx]);
            const float px = x + w * float(i) / float(kHistoryLen - 1);
            const float py = upward ? (baseline - e * halfSpan) : (baseline + e * halfSpan);
            if (i == 0) moveTo(px, py); else lineTo(px, py);
        }
        lineTo(x + w, baseline);
        lineTo(x, baseline);
        closePath();
        fillColor(fillCol);
        fill();
        beginPath();
        for (int i = 0; i < kHistoryLen; ++i)
        {
            const int idx = (historyPos_ + i) % kHistoryLen;
            const float e = norm(history[idx]);
            const float px = x + w * float(i) / float(kHistoryLen - 1);
            const float py = upward ? (baseline - e * halfSpan) : (baseline + e * halfSpan);
            if (i == 0) moveTo(px, py); else lineTo(px, py);
        }
        strokeColor(strokeCol);
        strokeWidth(1.2f);
        stroke();
    };
    drawChannel(historyL_, true, Color(120, 170, 235, 0.9f), Color(100, 150, 220, 0.25f));
    drawChannel(historyR_, false, Color(235, 110, 100, 0.9f), Color(220, 90, 80, 0.25f));

    beginPath();
    moveTo(x, baseline);
    lineTo(x + w, baseline);
    strokeColor(Color(80, 84, 90));
    strokeWidth(1.0f);
    stroke();

    // dB labels, drawn last (on top of both channel fills, not
    // underneath them -- see the gridline loop's own comment above for
    // why) with a small opaque backdrop chip so they stay legible
    // regardless of what color envelope happens to be passing behind
    // them. Both halves labeled now: the previous version only ever labeled the upper
    // (L/blue) half and left the lower (R/red) half to be read by mental
    // mirroring, which is exactly what was missing.
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(8.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    auto drawDbLabel = [&](float py, float db) {
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%+.0f", db);
        const float chipW = 26.0f, chipH = 11.0f;
        beginPath();
        rect(x, py - chipH * 0.5f, chipW, chipH);
        fillColor(Color(16, 17, 20, 0.78f));
        fill();
        fillColor(Color(190, 194, 200));
        text(x + 3.0f, py, buf, nullptr);
    };
    for (float db : kGridDbValues)
    {
        if (db < kLevelGridMinDb || db > kLevelGridMaxDb) continue;
        const float e = norm(db);
        drawDbLabel(baseline - e * halfSpan, db);
        drawDbLabel(baseline + e * halfSpan, db);
    }

    fillColor(Color(140, 144, 152));
    fontSize(10.0f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x + 4.0f, y + 3.0f, levelTitle_.c_str(), nullptr);
}

void SidechainScopeWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(20, 21, 25));
    fill();

    constexpr float pad = 6.0f;
    constexpr float gap = 6.0f;
    const float specH = (h - pad * 2.0f - gap) * 0.62f; // spectrum gets a bit more room -- the level trace only needs to read as a shape, not a scale
    const float levelH = h - pad * 2.0f - gap - specH;

    drawSpectrum(pad, pad, w - pad * 2.0f, specH);
    drawLevelTrace(pad, pad + specH + gap, w - pad * 2.0f, levelH);
}

END_NAMESPACE_DGL
