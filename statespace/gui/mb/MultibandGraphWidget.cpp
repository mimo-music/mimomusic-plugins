// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "MultibandGraphWidget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

START_NAMESPACE_DGL

using ssfilter::MbBand;
using ssfilter::SpectrumAnalyzer;

namespace {

// Same palette EqGraphWidget cycles band colors from -- kept identical on
// purpose so a project-wide glance ("band 3 is that purple-ish color")
// stays consistent across plugins.
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

} // namespace

MultibandGraphWidget::MultibandGraphWidget(NanoTopLevelWidget* parent, ssgui::IMbEditorHost* host)
    : EditorSubWidget(parent), host_(host)
{
    loadSharedResources();
    for (float& v : spectrumInDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutDb_) v = SpectrumAnalyzer::kFloorDb;
}

void MultibandGraphWidget::setInputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumInDb_[i] = db[i];
}

void MultibandGraphWidget::setOutputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutDb_[i] = db[i];
}

float MultibandGraphWidget::freqToX(double freqHz) const
{
    const double t = std::log(freqHz / kFMin) / std::log(kFMax / kFMin);
    return graphX_ + float(std::max(0.0, std::min(1.0, t))) * graphW_;
}

double MultibandGraphWidget::xToFreq(float px) const
{
    const double t = double(px - graphX_) / double(std::max(1.0f, graphW_));
    return kFMin * std::pow(kFMax / kFMin, std::max(0.0, std::min(1.0, t)));
}

float MultibandGraphWidget::spectrumToY(double db) const
{
    const double t = (spectrumRangeHi_ - db) / (spectrumRangeHi_ - spectrumRangeLo_);
    return graphY_ + float(std::max(0.0, std::min(1.0, t))) * graphH_;
}

int MultibandGraphWidget::handleAt(double px, double py) const
{
    if (py < graphY_ || py >= graphY_ + graphH_) return -1;
    const int numCrossovers = host_->getBandCount() - 1;
    int best = -1;
    float bestDist = kHandleHitPx;
    for (int i = 0; i < numCrossovers; ++i)
    {
        const float x = freqToX(host_->getCrossoverHz(i));
        const float dist = std::fabs(float(px) - x);
        if (dist < bestDist) { bestDist = dist; best = i; }
    }
    return best;
}

int MultibandGraphWidget::bandRegionAt(double px) const
{
    const int numBands = host_->getBandCount();
    for (int k = 0; k < numBands; ++k)
    {
        const float lo = k == 0 ? graphX_ : freqToX(host_->getCrossoverHz(k - 1));
        const float hi = k + 1 < numBands ? freqToX(host_->getCrossoverHz(k)) : graphX_ + graphW_;
        if (px >= lo && px < hi) return k;
    }
    return numBands > 0 ? numBands - 1 : -1;
}

// Smooths the raw, potentially very-fast-arriving live gain reading in
// real time before it's stored -- otherwise the numbers change too fast to
// read: setBandGainDb() can be
// called as often as the host delivers DPF output-Parameter updates
// (up to once per audio block), far faster than repaint()'s own display-
// refresh-rate throttling would suggest, and a compressor's whole point is
// swinging several dB within a handful of milliseconds during attack/
// release -- exactly what makes a raw printed number flicker rather than
// read. A one-pole filter against wall-clock time (std::chrono, same
// technique as ../compressor's own CompressorGraphWidget::updateHold())
// keeps both the meter fill and the printed digits legible while still
// tracking real changes within a fraction of a second, correct regardless
// of how often this is actually called.
void MultibandGraphWidget::setBandGainDb(int index, float db)
{
    if (index < 0 || index >= ssfilter::kMaxBands) return;

    if (!bandGainSmoothInit_[index])
    {
        bandGainDb_[index] = db; // snap on the very first reading, nothing to smooth from yet
        bandGainSmoothInit_[index] = true;
        bandGainSmoothUpdated_[index] = std::chrono::steady_clock::now();
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const float dtSec = std::chrono::duration<float>(now - bandGainSmoothUpdated_[index]).count();
    bandGainSmoothUpdated_[index] = now;

    constexpr float kTauSec = 0.22f; // readable without feeling sluggish -- see file comment above
    const float coeff = dtSec > 0.0f ? std::exp(-dtSec / kTauSec) : 1.0f;
    bandGainDb_[index] = coeff * bandGainDb_[index] + (1.0f - coeff) * db;
}

// A bipolar meter per band -- 0dB reference, live gain filling down
// (amber/red, a cut) or up (blue/green, a boost -- possible now that a
// band's Up Ratio can lift a quiet passage, not just reduce a loud one)
// from it, same convention as ../compressor's own GR meter -- plus a
// distinct, always-visible line at the band's *configured* Makeup value,
// separate from the live fill. The live gain is printed large above the
// bar, the makeup below it, and a dB scale runs down the bar's right side;
// numbers and scale sit outside the bar so the fill never hides them. The
// selected band's meter gets a border and top strip in the band's color.
// See setBandGainDb() for the time smoothing that keeps the digits
// readable.
void MultibandGraphWidget::drawBandGainMeter(float x, float y, float w, float h, int band, bool selected)
{
    if (band < 0 || band >= host_->getBandCount()) return;
    const MbBand b = host_->getBand(band);
    const double grDb = double(bandGainDb_[band]);
    const Color c = bandColor(band);
    // Fonts follow the width so a narrow meter's digits still fit.
    const float fs = std::min(1.0f, w / 96.0f);

    beginPath();
    roundedRect(x, y, w, h, 4.0f);
    fillColor(Color(16, 17, 20, selected ? 0.94f : 0.86f));
    fill();
    beginPath();
    roundedRect(x, y, w, 3.0f, 1.5f);
    fillColor(Color(c.red, c.green, c.blue, selected ? 1.0f : 0.55f));
    fill();
    beginPath();
    roundedRect(x + 0.5f, y + 0.5f, w - 1.0f, h - 1.0f, 4.0f);
    strokeColor(selected ? Color(c.red, c.green, c.blue, 0.95f) : Color(52, 56, 64));
    strokeWidth(selected ? 2.0f : 1.0f);
    stroke();

    const float hdrH = 46.0f, ftrH = 44.0f, scaleW = 28.0f;
    const float barX = x + 8.0f;
    const float barY = y + hdrH;
    const float barW = w - 8.0f - scaleW;
    const float barH = h - hdrH - ftrH;
    if (barW <= 4.0f || barH <= 8.0f) return;

    auto yOf = [&](double db) {
        const double clamped = std::max(kGrMeterMinDb, std::min(kGrMeterMaxDb, db));
        const double frac = (clamped - kGrMeterMinDb) / (kGrMeterMaxDb - kGrMeterMinDb);
        return barY + float(1.0 - frac) * barH;
    };
    const float zeroY = yOf(0.0);
    const float grY = yOf(grDb);
    const float makeupY = yOf(b.makeupDb);

    beginPath();
    rect(barX, barY, barW, barH);
    fillColor(Color(10, 11, 13));
    fill();

    // scissor() takes (x, y, w, h) with a positive height: a negative one
    // is clamped to 0 by nanovg and the fill silently disappears.
    if (grDb < -0.01)
    {
        scissor(barX, std::min(grY, zeroY), barW, std::fabs(zeroY - grY));
        beginPath();
        rect(barX, barY, barW, barH);
        Paint grad = linearGradient(0, barY + barH, 0, zeroY, Color(230, 120, 70), Color(235, 195, 60));
        fillPaint(grad);
        fill();
        resetScissor();
    }
    else if (grDb > 0.01)
    {
        scissor(barX, std::min(grY, zeroY), barW, std::fabs(grY - zeroY));
        beginPath();
        rect(barX, barY, barW, barH);
        Paint grad = linearGradient(0, zeroY, 0, barY, Color(90, 190, 150), Color(70, 150, 220));
        fillPaint(grad);
        fill();
        resetScissor();
    }

    beginPath();
    moveTo(barX, zeroY);
    lineTo(barX + barW, zeroY);
    strokeColor(Color(110, 114, 122));
    strokeWidth(1.0f);
    stroke();

    // Makeup gain marker.
    beginPath();
    moveTo(barX, makeupY);
    lineTo(barX + barW, makeupY);
    strokeColor(Color(170, 180, 240, 0.95f));
    strokeWidth(2.0f);
    stroke();

    // dB scale right of the bar -- same "give the number context, not just
    // a bare digit" reasoning as ../compressor's own
    // CompressorGraphWidget::drawGrScale().
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fillColor(Color(150, 154, 162));
    fontSize(10.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    for (double db = kGrMeterMinDb; db <= kGrMeterMaxDb + 0.01; db += 12.0)
    {
        char scaleBuf[8];
        std::snprintf(scaleBuf, sizeof(scaleBuf), "%+.0f", db);
        text(barX + barW + 4.0f, yOf(db), scaleBuf, nullptr);
    }

    char grBuf[24];
    std::snprintf(grBuf, sizeof(grBuf), "%+.1f", grDb);
    fillColor(grDb < -0.05 ? Color(235, 195, 60) : grDb > 0.05 ? Color(120, 190, 235) : Color(210, 214, 220));
    fontSize(25.0f * fs);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(x + w * 0.5f, y + 6.0f, grBuf, nullptr);
    fillColor(Color(165, 169, 178));
    fontSize(11.0f);
    text(x + w * 0.5f, y + 32.0f, "GAIN", nullptr);

    char makeupBuf[24];
    std::snprintf(makeupBuf, sizeof(makeupBuf), "%+.1f", b.makeupDb);
    fillColor(Color(185, 193, 240));
    fontSize(21.0f * fs);
    textAlign(ALIGN_CENTER | ALIGN_BOTTOM);
    text(x + w * 0.5f, y + h - 16.0f, makeupBuf, nullptr);
    fillColor(Color(155, 161, 205));
    fontSize(11.0f);
    text(x + w * 0.5f, y + h - 3.0f, "MAKEUP", nullptr);
}

// One meter per band, centered over its region where it fits. All meters
// share one width; where regions are narrower than that (low bands on the
// log axis), neighbors are pushed apart just enough not to overlap, still
// inside the graph.
void MultibandGraphWidget::layoutBandGainMeters(int numBands)
{
    meterCount_ = std::max(0, std::min(numBands, ssfilter::kMaxBands));
    if (meterCount_ == 0) return;

    constexpr float kGap = 6.0f, kMaxW = 104.0f, kTop = 34.0f, kBottom = 26.0f;
    const float left = graphX_ + 2.0f, right = graphX_ + graphW_ - 2.0f;
    meterW_ = std::min(kMaxW, (right - left - kGap * float(meterCount_ - 1)) / float(meterCount_));
    meterY_ = graphY_ + kTop;
    meterH_ = std::max(130.0f, std::min(300.0f, graphH_ - kTop - kBottom));

    for (int k = 0; k < meterCount_; ++k)
    {
        const float lo = k == 0 ? graphX_ : freqToX(host_->getCrossoverHz(k - 1));
        const float hi = k + 1 < meterCount_ ? freqToX(host_->getCrossoverHz(k)) : graphX_ + graphW_;
        meterX_[k] = std::max(left, std::min(right - meterW_, (lo + hi - meterW_) * 0.5f));
    }
    for (int k = 1; k < meterCount_; ++k)
        meterX_[k] = std::max(meterX_[k], meterX_[k - 1] + meterW_ + kGap);
    meterX_[meterCount_ - 1] = std::min(meterX_[meterCount_ - 1], right - meterW_);
    for (int k = meterCount_ - 2; k >= 0; --k)
        meterX_[k] = std::min(meterX_[k], meterX_[k + 1] - meterW_ - kGap);
}

int MultibandGraphWidget::meterAt(double px, double py) const
{
    if (py < meterY_ || py >= meterY_ + meterH_) return -1;
    for (int k = 0; k < meterCount_; ++k)
        if (px >= meterX_[k] && px < meterX_[k] + meterW_) return k;
    return -1;
}

int MultibandGraphWidget::bandAt(double px, double py) const
{
    const int m = meterAt(px, py);
    return m >= 0 ? m : bandRegionAt(px);
}

void MultibandGraphWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    graphX_ = 8.0f;
    graphY_ = 6.0f;
    graphW_ = w - graphX_ - 30.0f; // room on the right for the spectrum's own scale labels, see EqGraphWidget's convention
    graphH_ = h - graphY_ - 20.0f;
    if (graphW_ <= 4.0f || graphH_ <= 4.0f) return;

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(18, 19, 23));
    fill();

    beginPath();
    rect(graphX_, graphY_, graphW_, graphH_);
    fillColor(Color(15, 16, 19));
    fill();

    const int numBands = host_->getBandCount();
    // A single spotlight, not a per-band flag -- see
    // MbCompressorModel::soloedBand's own comment. Read once per redraw
    // (not per band) since it's a whole-model copy.
    const int soloedBand = host_->getModel().soloedBand;

    // Band regions, shaded by index -- drawn before the spectrum/gridlines
    // so those stay legible on top.
    for (int k = 0; k < numBands; ++k)
    {
        const float lo = k == 0 ? graphX_ : freqToX(host_->getCrossoverHz(k - 1));
        const float hi = k + 1 < numBands ? freqToX(host_->getCrossoverHz(k)) : graphX_ + graphW_;
        const Color c = bandColor(k);
        const bool selected = k == selectedBand_;

        beginPath();
        rect(lo, graphY_, hi - lo, graphH_);
        fillColor(Color(c.red, c.green, c.blue, selected ? 0.16f : 0.07f));
        fill();
        if (selected)
        {
            strokeColor(Color(c.red, c.green, c.blue, 0.85f));
            strokeWidth(1.5f);
            beginPath();
            rect(lo + 0.75f, graphY_ + 0.75f, hi - lo - 1.5f, graphH_ - 1.5f);
            stroke();
        }

        const MbBand band = host_->getBand(k);
        char label[64];
        std::snprintf(label, sizeof(label), "Band %d", k + 1);
        fontFace(NANOVG_DEJAVU_SANS_TTF);
        fontSize(11.5f);
        fillColor(Color(c.red, c.green, c.blue, 0.95f));
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        text((lo + hi) * 0.5f, graphY_ + 5.0f, label, nullptr);

        // "(solo)" here is the whole point of moving this to a single
        // model-level spotlight -- previously (a per-band flag) there was
        // no way to see which band was soloed except by re-selecting each
        // one and checking the Solo button's own color, which is exactly
        // the confusion reported ("the sound sticks in another band than
        // selected").
        char suffix[24] = "";
        if (k == soloedBand && !band.enabled) std::snprintf(suffix, sizeof(suffix), "  (solo, bypassed)");
        else if (k == soloedBand) std::snprintf(suffix, sizeof(suffix), "  (solo)");
        else if (!band.enabled) std::snprintf(suffix, sizeof(suffix), "  (bypassed)");

        char sub[80];
        if (band.ratioUp > 1.01)
            std::snprintf(sub, sizeof(sub), "%.0fdB  %.1f:1 / %.1f:1 up%s", band.thresholdDb, band.ratio, band.ratioUp, suffix);
        else
            std::snprintf(sub, sizeof(sub), "%.0fdB : 1  %.1f:1%s", band.thresholdDb, band.ratio, suffix);
        fontSize(9.5f);
        fillColor(Color(200, 204, 210, 0.75f));
        text((lo + hi) * 0.5f, graphY_ + 20.0f, sub, nullptr);
    }

    strokeColor(Color(48, 51, 58));
    strokeWidth(1.0f);
    beginPath();
    rect(graphX_, graphY_, graphW_, graphH_);
    stroke();

    // Frequency gridlines (log) with labels below the graph -- identical
    // set/positions to EqGraphWidget's own.
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.5f);
    for (double f : { 20.0, 50.0, 100.0, 200.0, 500.0, 1000.0, 2000.0, 5000.0, 10000.0, 20000.0 })
    {
        const float x = freqToX(f);
        beginPath();
        moveTo(x, graphY_);
        lineTo(x, graphY_ + graphH_);
        strokeColor(Color(38, 41, 47));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        char buf[16];
        if (f >= 1000.0) std::snprintf(buf, sizeof(buf), "%.0fk", f / 1000.0);
        else std::snprintf(buf, sizeof(buf), "%.0f", f);
        text(x, graphY_ + graphH_ + 2.0f, buf, nullptr);
    }

    // "Autoscale"/"Fixed Range" toggle, top-right of the graph -- see
    // EqGraphWidget.cpp's identical control for the full reasoning.
    autoscaleLabelW_ = 82.0f;
    autoscaleLabelH_ = 16.0f;
    autoscaleLabelX_ = graphX_ + graphW_ - autoscaleLabelW_ - 2.0f;
    autoscaleLabelY_ = graphY_ + 2.0f;
    beginPath();
    roundedRect(autoscaleLabelX_, autoscaleLabelY_, autoscaleLabelW_, autoscaleLabelH_, 3.0f);
    fillColor(spectrumAutoscale_ ? Color(55, 100, 130, 0.9f) : Color(34, 37, 43, 0.85f));
    fill();
    fillColor(spectrumAutoscale_ ? Color(230, 232, 236) : Color(150, 154, 162));
    fontSize(9.5f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(autoscaleLabelX_ + autoscaleLabelW_ * 0.5f, autoscaleLabelY_ + autoscaleLabelH_ * 0.5f + 1.0f,
         spectrumAutoscale_ ? "Autoscale" : "Fixed Range", nullptr);

    // Background spectrum -- pre-/post-compression, exactly EqGraphWidget's
    // own autoscale-to-the-union / dual-trace-hierarchy mechanism, see that
    // file's spectrumToY() comment for the full reasoning.
    if (spectrumMode_ != ssfilter::SpectrumMode::Off)
    {
        const bool showIn = ssfilter::spectrumModeWantsInput(spectrumMode_);
        const bool showOut = ssfilter::spectrumModeWantsOutput(spectrumMode_);

        // See EqGraphWidget.cpp's identical comment for the full reasoning:
        // off (fixed range) by default since a per-frame-snapping axis is
        // visually disturbing; when on, the target range is
        // still recomputed every frame but the axis smoothly drifts
        // toward it instead of snapping.
        if (spectrumAutoscale_)
        {
            double lo = 1e9, hi = -1e9;
            auto scanRange = [&](const float* db) {
                for (int b = 0; b < SpectrumAnalyzer::kNumBands; ++b)
                {
                    lo = std::min(lo, double(db[b]));
                    hi = std::max(hi, double(db[b]));
                }
            };
            if (showIn) scanRange(spectrumInDb_);
            if (showOut) scanRange(spectrumOutDb_);
            if (!(lo < hi)) { lo = kFixedSpectrumRangeLoDb; hi = kFixedSpectrumRangeHiDb; }
            lo -= 6.0; hi += 6.0;
            lo = std::floor(lo / 6.0) * 6.0;
            hi = std::ceil(hi / 6.0) * 6.0;
            lo = std::max(kSpectrumFloorDb, lo);
            hi = std::min(kSpectrumCeilDb, hi);
            if (hi - lo < 12.0) hi = std::min(kSpectrumCeilDb, lo + 12.0);
            if (hi - lo < 12.0) lo = std::max(kSpectrumFloorDb, hi - 12.0);

            constexpr double kAssumedFrameMs = 16.0;
            constexpr double kRangeSmoothMs = 600.0;
            const double coeff = std::exp(-kAssumedFrameMs / kRangeSmoothMs);
            spectrumRangeLo_ = coeff * spectrumRangeLo_ + (1.0 - coeff) * lo;
            spectrumRangeHi_ = coeff * spectrumRangeHi_ + (1.0 - coeff) * hi;
        }
        // else: freeze wherever the range currently sits -- see
        // EqGraphWidget.cpp's identical comment.

        auto drawTrace = [&](const float* db, Color strokeCol, Color fillCol, bool doFill) {
            beginPath();
            for (int b = 0; b < SpectrumAnalyzer::kNumBands; ++b)
            {
                const double t = double(b) / double(SpectrumAnalyzer::kNumBands - 1);
                const double f = SpectrumAnalyzer::kMinHz * std::pow(SpectrumAnalyzer::kMaxHz / SpectrumAnalyzer::kMinHz, t);
                const float x = freqToX(f);
                const float y = spectrumToY(double(db[b]));
                if (b == 0) moveTo(x, y); else lineTo(x, y);
            }
            if (doFill)
            {
                lineTo(graphX_ + graphW_, graphY_ + graphH_);
                lineTo(graphX_, graphY_ + graphH_);
                closePath();
                fillColor(fillCol);
                fill();
            }
            strokeColor(strokeCol);
            strokeWidth(1.0f);
            stroke();
        };

        const Color kInStroke(235, 165, 95, 0.55f);
        const Color kInFill(220, 140, 70, 0.12f);
        const Color kOutStroke(110, 180, 220, 0.45f);
        const Color kOutFill(80, 150, 200, 0.16f);

        if (showIn) drawTrace(spectrumInDb_, kInStroke, kInFill, !showOut);
        if (showOut) drawTrace(spectrumOutDb_, kOutStroke, kOutFill, true);

        {
            float lx = graphX_ + 6.0f, ly = graphY_ + graphH_ - 14.0f;
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
            if (showIn) legendItem(kInStroke, "Pre");
            if (showOut) legendItem(kOutStroke, "Post");
        }

        // Right-edge scale labels for the autoscaled spectrum range --
        // neutral gray (not either trace's own color), same reasoning as
        // EqGraphWidget's own: this axis measures absolute level, a
        // different kind of quantity than anything else on this graph.
        fontSize(9.0f);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        fillColor(Color(150, 152, 156, 0.85f));
        const double span = spectrumRangeHi_ - spectrumRangeLo_;
        const double step = span > 24.0 ? 12.0 : 6.0;
        for (double db = spectrumRangeLo_; db <= spectrumRangeHi_ + 0.01; db += step)
        {
            const float y = spectrumToY(db);
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%+.0f", db);
            text(graphX_ + graphW_ + 3.0f, y, buf, nullptr);
        }
    }

    // Crossover split handles -- a bright vertical line plus a small
    // triangular knob at the top, draggable horizontally.
    const int numCrossovers = numBands - 1;
    for (int i = 0; i < numCrossovers; ++i)
    {
        const float x = freqToX(host_->getCrossoverHz(i));
        const bool active = (i == dragHandle_) || (i == hoverHandle_);
        beginPath();
        moveTo(x, graphY_);
        lineTo(x, graphY_ + graphH_);
        strokeColor(active ? Color(255, 255, 255, 0.9f) : Color(200, 202, 206, 0.55f));
        strokeWidth(active ? 2.0f : 1.3f);
        stroke();

        beginPath();
        moveTo(x - 5.0f, graphY_);
        lineTo(x + 5.0f, graphY_);
        lineTo(x, graphY_ + 8.0f);
        closePath();
        fillColor(active ? Color(255, 255, 255) : Color(200, 202, 206));
        fill();
    }

    // Every band's gain-reduction/makeup meter, the selected one
    // highlighted -- see drawBandGainMeter(). Drawn over the spectrum and
    // crossover lines so the numbers stay readable; the crossover handles
    // stay reachable above and below them.
    layoutBandGainMeters(numBands);
    for (int k = 0; k < meterCount_; ++k)
        drawBandGainMeter(meterX_[k], meterY_, meterW_, meterH_, k, k == selectedBand_);

    // Frequency readout near whichever crossover split is being dragged or
    // merely hovered -- the same "show it while pointing, not just while
    // dragging" convention EqGraphWidget/DynEqGraphWidget's node readouts
    // use, applied here to a border instead of a node.
    const int readoutHandle = dragHandle_ >= 0 ? dragHandle_ : hoverHandle_;
    if (readoutHandle >= 0 && readoutHandle < numCrossovers)
    {
        const double freqHz = host_->getCrossoverHz(readoutHandle);
        const float x = freqToX(freqHz);

        char buf[24];
        if (freqHz >= 1000.0) std::snprintf(buf, sizeof(buf), "%.2fkHz", freqHz / 1000.0);
        else std::snprintf(buf, sizeof(buf), "%.0fHz", freqHz);

        fontSize(11.5f);
        const float textW = float(std::strlen(buf)) * 6.4f + 14.0f;
        const float textH = 18.0f;
        float lx = std::max(graphX_, std::min(graphX_ + graphW_ - textW, x - textW * 0.5f));
        const float ly = graphY_ + 12.0f; // just below the triangular knob at the handle's top

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

bool MultibandGraphWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (ev.press)
    {
        if (x >= autoscaleLabelX_ && x < autoscaleLabelX_ + autoscaleLabelW_ &&
            y >= autoscaleLabelY_ && y < autoscaleLabelY_ + autoscaleLabelH_)
        {
            spectrumAutoscale_ = !spectrumAutoscale_;
            if (onSpectrumAutoscaleChanged_) onSpectrumAutoscaleChanged_(spectrumAutoscale_);
            repaint();
            return true;
        }

        const int handle = handleAt(x, y);
        if (handle >= 0)
        {
            const auto now = std::chrono::steady_clock::now();
            constexpr std::chrono::milliseconds kDoubleClickWindow(400);
            const bool isDoubleClick = handle == lastClickHandle_ && lastClickBand_ == -1 &&
                                        (now - lastClickTime_) < kDoubleClickWindow;
            lastClickHandle_ = isDoubleClick ? -1 : handle;
            lastClickBand_ = -1;
            lastClickTime_ = now;
            if (isDoubleClick)
            {
                host_->removeCrossover(handle);
                dragHandle_ = -1;
                repaint();
                return true;
            }
            dragHandle_ = handle;
            repaint();
            return true;
        }

        const int band = bandAt(x, y);
        if (band >= 0)
        {
            // A click on a meter only selects -- double-clicking it must not
            // split the band underneath, which may be a neighbor's.
            const bool onMeter = meterAt(x, y) >= 0;
            const auto now = std::chrono::steady_clock::now();
            constexpr std::chrono::milliseconds kDoubleClickWindow(400);
            const bool isDoubleClick = band == lastClickBand_ && lastClickHandle_ == -1 &&
                                        (now - lastClickTime_) < kDoubleClickWindow;
            lastClickBand_ = isDoubleClick ? -1 : band;
            lastClickHandle_ = -1;
            lastClickTime_ = now;

            if (isDoubleClick && !onMeter)
            {
                host_->addCrossover(xToFreq(float(x)));
                repaint();
                return true;
            }

            selectedBand_ = band;
            if (onSelectionChanged_) onSelectionChanged_(band);
            repaint();
            return true;
        }
        return false;
    }

    if (dragHandle_ >= 0)
    {
        dragHandle_ = -1;
        repaint();
        return true;
    }
    return false;
}

bool MultibandGraphWidget::onMotion(const MotionEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (dragHandle_ >= 0)
    {
        host_->setCrossoverHz(dragHandle_, xToFreq(float(x)));
        repaint();
        return true;
    }

    const int h = handleAt(x, y);
    if (h != hoverHandle_) { hoverHandle_ = h; repaint(); }
    return false;
}

bool MultibandGraphWidget::onScroll(const ScrollEvent& ev)
{
    const int band = bandAt(ev.pos.getX(), ev.pos.getY());
    if (band < 0) return false;

    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;

    // Alt+scroll = 10x finer -- see ../eq/EqGraphWidget.cpp's identical
    // comment for the full reasoning, applied project-wide.
    constexpr double kBaseFactor = 1.08;
    const double factor = (ev.mod & kModifierAlt) ? 1.0 + (kBaseFactor - 1.0) / 10.0 : kBaseFactor;
    const MbBand b = host_->getBand(band);
    const double next = dir > 0 ? b.ratio * factor : b.ratio / factor;
    host_->setBandRatio(band, next);
    repaint();
    return true;
}

END_NAMESPACE_DGL
