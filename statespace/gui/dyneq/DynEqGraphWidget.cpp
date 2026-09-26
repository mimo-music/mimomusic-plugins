// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "DynEqGraphWidget.hpp"

#include "../../plugin/dsp/DynamicEqModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

START_NAMESPACE_DGL

using ssfilter::DynamicEqBand;
using ssfilter::DynamicEqModel;
using ssfilter::EqChannel;
using ssfilter::SpectrumAnalyzer;

namespace {

// Same palette EqGraphWidget/MultibandGraphWidget cycle band colors from --
// kept identical on purpose so a project-wide glance stays consistent.
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

constexpr int kCurvePoints = 300;

constexpr double kGainRangePresets[] = { 3.0, 6.0, 12.0, 30.0 };
constexpr int kNumGainRangePresets = int(sizeof(kGainRangePresets) / sizeof(kGainRangePresets[0]));

double nextGainRange(double current)
{
    int idx = 0;
    for (int i = 0; i < kNumGainRangePresets; ++i)
        if (std::fabs(kGainRangePresets[i] - current) < 0.01) { idx = i; break; }
    return kGainRangePresets[(idx + 1) % kNumGainRangePresets];
}

} // namespace

std::string DynEqGraphWidget::formatFreqLabel(double freqHz)
{
    char buf[32];
    if (freqHz >= 1000.0) std::snprintf(buf, sizeof(buf), "%.2fkHz", freqHz / 1000.0);
    else std::snprintf(buf, sizeof(buf), "%.0fHz", freqHz);
    return std::string(buf);
}

DynEqGraphWidget::DynEqGraphWidget(Widget* parent, ssgui::IDynEqEditorHost* host)
    : NanoSubWidget(parent), host_(host)
{
    loadSharedResources();
    for (float& v : spectrumInDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRDb_) v = SpectrumAnalyzer::kFloorDb;
}

void DynEqGraphWidget::setInputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumInDb_[i] = db[i];
}

void DynEqGraphWidget::setOutputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutDb_[i] = db[i];
}

void DynEqGraphWidget::setOutputSpectrumDataL(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutLDb_[i] = db[i];
}

void DynEqGraphWidget::setOutputSpectrumDataR(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutRDb_[i] = db[i];
}

float DynEqGraphWidget::freqToX(double freqHz) const
{
    const double t = std::log(freqHz / kFMin) / std::log(kFMax / kFMin);
    return graphX_ + float(std::max(0.0, std::min(1.0, t))) * graphW_;
}

double DynEqGraphWidget::xToFreq(float px) const
{
    const double t = double(px - graphX_) / double(std::max(1.0f, graphW_));
    return kFMin * std::pow(kFMax / kFMin, std::max(0.0, std::min(1.0, t)));
}

float DynEqGraphWidget::gainToY(double gainDb) const
{
    const double t = (gainRangeDb_ - gainDb) / (2.0 * gainRangeDb_);
    return graphY_ + float(t) * graphH_;
}

double DynEqGraphWidget::yToGain(float py) const
{
    const double t = double(py - graphY_) / double(std::max(1.0f, graphH_));
    return gainRangeDb_ - t * (2.0 * gainRangeDb_);
}

float DynEqGraphWidget::spectrumToY(double db) const
{
    const double t = (spectrumRangeHi_ - db) / (spectrumRangeHi_ - spectrumRangeLo_);
    return graphY_ + float(std::max(0.0, std::min(1.0, t))) * graphH_;
}

int DynEqGraphWidget::nodeAt(double px, double py) const
{
    int best = -1;
    float bestDist = kHitRadius;
    for (int i = 0; i < host_->getBandCount(); ++i)
    {
        const DynamicEqBand b = host_->getBand(i);
        const float nx = freqToX(b.freqHz);
        const float ny = gainToY(b.gainDb);
        const float dist = std::sqrt(float((px - nx) * (px - nx) + (py - ny) * (py - ny)));
        if (dist < bestDist) { bestDist = dist; best = i; }
    }
    return best;
}

void DynEqGraphWidget::dragBandTo(int index, double px, double py)
{
    if (index < 0 || index >= host_->getBandCount()) return;
    const double freq = std::max(kFMin, std::min(kFMax, xToFreq(float(px))));
    host_->setBandFreq(index, freq);
    const double gain = std::max(-gainRangeDb_, std::min(gainRangeDb_, yToGain(float(py))));
    host_->setBandGain(index, gain);
}

// ---- drawing --------------------------------------------------------------

void DynEqGraphWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    graphX_ = 36.0f;
    graphY_ = 6.0f;
    graphW_ = w - graphX_ - 30.0f;
    graphH_ = h - graphY_ - 20.0f;
    if (graphW_ <= 4.0f || graphH_ <= 4.0f) return;

    // Fetched once, up front -- see EqGraphWidget.cpp's identical comment:
    // both the spectrum section below and the curve/node sections further
    // down need the current band list.
    const DynamicEqModel model = host_->getModel();

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(18, 19, 23));
    fill();

    beginPath();
    rect(graphX_, graphY_, graphW_, graphH_);
    fillColor(Color(15, 16, 19));
    fill();
    strokeColor(Color(48, 51, 58));
    strokeWidth(1.0f);
    stroke();

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

    // Counted from zero (k * gridStep), not stepped outward from
    // -gainRangeDb_ -- see EqGraphWidget.cpp's identical fix for why: it
    // guarantees the brighter 0dB reference line actually lands on 0
    // regardless of gainRangeDb_, including the continuously-draggable
    // (non-preset) values the range label now allows.
    const double gridStep = gainRangeDb_ <= 3.0 ? 1.0 : gainRangeDb_ <= 6.0 ? 2.0 : gainRangeDb_ <= 12.0 ? 4.0 : 10.0;
    const int gridKMax = int(std::ceil(gainRangeDb_ / gridStep)) + 1;
    for (int gridK = -gridKMax; gridK <= gridKMax; ++gridK)
    {
        const double g = double(gridK) * gridStep;
        if (g < -gainRangeDb_ - 0.01 || g > gainRangeDb_ + 0.01) continue;
        const float y = gainToY(g);
        const bool isZero = gridK == 0;
        beginPath();
        moveTo(graphX_, y);
        lineTo(graphX_ + graphW_, y);
        strokeColor(isZero ? Color(90, 96, 106) : Color(38, 41, 47));
        strokeWidth(isZero ? 1.4f : 1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%+.0f", g);
        text(graphX_ - 4.0f, y, buf, nullptr);
    }

    // Background spectrum -- identical mechanism to EqGraphWidget's own.
    if (spectrumMode_ != ssfilter::SpectrumMode::Off)
    {
        const bool showIn = ssfilter::spectrumModeWantsInput(spectrumMode_);
        const bool showOut = ssfilter::spectrumModeWantsOutput(spectrumMode_);
        // See EqGraphWidget.cpp's identical perChannel comment for the
        // full reasoning -- same feature, same trigger condition.
        const bool perChannel = showOut && ssfilter::dyneq::hasPerChannelBands(model);

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
            if (showOut)
            {
                if (perChannel) { scanRange(spectrumOutLDb_); scanRange(spectrumOutRDb_); }
                else scanRange(spectrumOutDb_);
            }
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
        const Color kOutStroke(110, 180, 220, 0.4f);
        const Color kOutFill(80, 150, 200, 0.16f);
        const Color kOutRStroke(220, 100, 90, 0.4f);
        const Color kOutRFill(200, 75, 65, 0.16f);

        if (showIn) drawTrace(spectrumInDb_, kInStroke, kInFill, !showOut);
        if (showOut)
        {
            if (perChannel)
            {
                drawTrace(spectrumOutLDb_, kOutStroke, kOutFill, true);
                drawTrace(spectrumOutRDb_, kOutRStroke, kOutRFill, true);
            }
            else
            {
                drawTrace(spectrumOutDb_, kOutStroke, kOutFill, true);
            }
        }

        {
            float lx = graphX_ + 6.0f, ly = graphY_ + 6.0f;
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
            if (showIn) legendItem(kInStroke, "Pre-EQ");
            if (showOut)
            {
                if (perChannel) { legendItem(kOutStroke, "Post-EQ L"); legendItem(kOutRStroke, "Post-EQ R"); }
                else legendItem(kOutStroke, "Post-EQ");
            }
        }

        const double spectrumGridStep = (spectrumRangeHi_ - spectrumRangeLo_) <= 24.0 ? 6.0 : 12.0;
        fontSize(9.0f);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        for (double g = spectrumRangeLo_; g <= spectrumRangeHi_ + 0.01; g += spectrumGridStep)
        {
            const float y = spectrumToY(g);
            fillColor(Color(165, 169, 177, 0.85f));
            char buf[16];
            std::snprintf(buf, sizeof(buf), "%+.0f", g);
            text(graphX_ + graphW_ + 3.0f, y, buf, nullptr);
        }
    }

    const int bandCount = host_->getBandCount();

    // Per-band dynamic-range shading -- the area between a band's own
    // response curve at the two extremes its gain can actually reach:
    // gainDb+rangeDb at the top when upward compression (ratioUp > 1) can
    // lift it, gainDb-rangeDb at the bottom when downward compression
    // (ratio > 1) can pull it down -- "up and down compressing" shown as
    // one region that can extend either way, or both at once. Whichever
    // direction isn't active for this band just collapses that edge back
    // to gainDb (its own resting value), the same shape the original
    // downward-only version drew. This is the FabFilter screenshots'
    // shaded green/purple region around a dynamic node. Drawn before
    // every other curve so they stay legible on top of it.
    for (int i = 0; i < bandCount; ++i)
    {
        const DynamicEqBand b = host_->getBand(i);
        const bool downActive = b.ratio > 1.01;
        const bool upActive = b.ratioUp > 1.01;
        if (!b.enabled || !b.dynamicsEnabled || b.rangeDb <= 0.0 || !(downActive || upActive)) continue;
        const double topGainDb = upActive ? b.gainDb + b.rangeDb : b.gainDb;
        const double bottomGainDb = downActive ? b.gainDb - b.rangeDb : b.gainDb;
        Color fillCol = bandColor(i);
        fillCol.alpha = (i == selectedBand_) ? 0.20f : 0.11f;

        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::dyneq::evaluateBandResponse(b, sampleRate_, f, topGainDb);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            if (p == 0) moveTo(x, y); else lineTo(x, y);
        }
        for (int p = kCurvePoints; p >= 0; --p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::dyneq::evaluateBandResponse(b, sampleRate_, f, bottomGainDb);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            lineTo(x, y);
        }
        closePath();
        fillColor(fillCol);
        fill();
    }

    // Each band's own isolated *resting* response, thin and dim -- the
    // static shape the live curve moves around, same layering idea as
    // EqGraphWidget's own per-band curves.
    for (int i = 0; i < bandCount; ++i)
    {
        const DynamicEqBand b = host_->getBand(i);
        if (!b.enabled) continue;
        Color col = bandColor(i);
        col.alpha = 0.45f;
        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::dyneq::evaluateBandResponse(b, sampleRate_, f, b.gainDb);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            if (p == 0) moveTo(x, y); else lineTo(x, y);
        }
        strokeColor(col);
        strokeWidth(1.1f);
        stroke();
    }

    // The resting/static combined curve -- what the EQ shape settles to
    // with no program material loud enough to engage any band's dynamics.
    //
    // A single curve as long as every active band is Both -- unchanged
    // from before per-channel routing existed. Once any band is Left/
    // Right-only, that one curve would silently misrepresent whichever
    // channel it doesn't reach, so it splits into two (blue/red), exactly
    // the same "split only when it would otherwise mislead" rule
    // EqGraphWidget's own combined curve now follows.
    const bool perChannelCurve = ssfilter::dyneq::hasPerChannelBands(model);
    auto drawRestingCurve = [&](EqChannel listenerChannel, Color color)
    {
        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = perChannelCurve
                ? ssfilter::dyneq::evaluateChannelResponse(model, sampleRate_, f, listenerChannel)
                : ssfilter::dyneq::evaluateCombinedResponse(model, sampleRate_, f);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            if (p == 0) moveTo(x, y); else lineTo(x, y);
        }
        strokeColor(color);
        strokeWidth(1.6f);
        stroke();
    };
    if (perChannelCurve)
    {
        drawRestingCurve(EqChannel::Left, Color(90, 150, 230, 0.55f));
        drawRestingCurve(EqChannel::Right, Color(230, 110, 100, 0.55f));
    }
    else
    {
        drawRestingCurve(EqChannel::Both, Color(150, 154, 162, 0.55f));
    }

    // The LIVE combined curve -- every band's currently-*actual* running
    // gain (DynamicEqEngine::getCurrentGainDb(), via
    // IDynEqEditorHost::getCurrentGainDb()) instead of its resting gainDb.
    // This is the one that visibly moves/wiggles as program material
    // crosses each band's threshold -- the FabFilter screenshots' yellow
    // trace. Drawn brightest, on top of the resting curve, so the two
    // read as "shape" and "what's actually happening right now". Same
    // per-channel split as the resting curve above, for the same reason.
    {
        std::vector<double> live(size_t(std::max(0, bandCount)));
        for (int i = 0; i < bandCount; ++i) live[size_t(i)] = host_->getCurrentGainDb(i);
        const double* livePtr = bandCount > 0 ? live.data() : nullptr;

        auto drawLiveCurve = [&](EqChannel listenerChannel, Color color)
        {
            beginPath();
            for (int p = 0; p <= kCurvePoints; ++p)
            {
                const double t = double(p) / double(kCurvePoints);
                const double f = kFMin * std::pow(kFMax / kFMin, t);
                const std::complex<double> resp = perChannelCurve
                    ? ssfilter::dyneq::evaluateChannelResponse(model, sampleRate_, f, listenerChannel, livePtr)
                    : ssfilter::dyneq::evaluateCombinedResponse(model, sampleRate_, f, livePtr);
                const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
                const float x = graphX_ + float(t) * graphW_;
                const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
                if (p == 0) moveTo(x, y); else lineTo(x, y);
            }
            strokeColor(color);
            strokeWidth(2.4f);
            stroke();
        };
        if (perChannelCurve)
        {
            drawLiveCurve(EqChannel::Left, Color(90, 150, 230));
            drawLiveCurve(EqChannel::Right, Color(230, 110, 100));
        }
        else
        {
            drawLiveCurve(EqChannel::Both, Color(235, 195, 60));
        }
    }

    // Node markers, drawn last (on top of every curve).
    for (int i = 0; i < bandCount; ++i)
    {
        const DynamicEqBand b = host_->getBand(i);
        const float nx = freqToX(b.freqHz);
        const float ny = gainToY(b.gainDb);
        const bool selected = (i == selectedBand_);
        const bool hovered = (i == hoverBand_);
        const Color col = b.enabled ? bandColor(i) : Color(110, 112, 116);

        if (selected || hovered)
        {
            beginPath();
            circle(nx, ny, 9.5f);
            strokeColor(Color(255, 255, 255, selected ? 0.8f : 0.43f));
            strokeWidth(1.6f);
            stroke();
        }
        beginPath();
        circle(nx, ny, 6.0f);
        fillColor(col);
        fill();
        strokeColor(Color(15, 16, 19));
        strokeWidth(1.2f);
        stroke();

        // Channel-assignment ring -- same idiom as EqGraphWidget's own
        // identical ring: always visible when a band isn't Both, larger
        // radius than the selection ring above so the two never compete.
        if (b.channel != EqChannel::Both)
        {
            beginPath();
            circle(nx, ny, 13.0f);
            strokeColor(b.channel == EqChannel::Left ? Color(90, 140, 230, 0.85f) : Color(230, 100, 90, 0.85f));
            strokeWidth(1.8f);
            stroke();
        }

        fillColor(Color(15, 16, 19));
        fontSize(8.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%d", i + 1);
        text(nx, ny + 0.5f, buf, nullptr);
    }

    if (bandCount == 0)
    {
        fillColor(Color(130, 134, 142));
        fontSize(13.0f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(graphX_ + graphW_ * 0.5f, graphY_ + graphH_ * 0.5f,
             "Click anywhere to add a band", nullptr);
    }

    // "±Ndb" gain-range label, top-right corner -- click to cycle presets.
    char rangeBuf[16];
    std::snprintf(rangeBuf, sizeof(rangeBuf), "\xC2\xB1%.0fdB", gainRangeDb_);
    rangeLabelW_ = 48.0f;
    rangeLabelH_ = 16.0f;
    rangeLabelX_ = graphX_ + graphW_ - rangeLabelW_ - 2.0f;
    rangeLabelY_ = graphY_ + 2.0f;
    beginPath();
    roundedRect(rangeLabelX_, rangeLabelY_, rangeLabelW_, rangeLabelH_, 3.0f);
    fillColor(Color(34, 37, 43, 0.85f));
    fill();
    fillColor(Color(190, 194, 202));
    fontSize(10.5f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(rangeLabelX_ + rangeLabelW_ * 0.5f, rangeLabelY_ + rangeLabelH_ * 0.5f + 1.0f, rangeBuf, nullptr);

    // "Autoscale"/"Fixed Range" toggle, stacked just below the "±Ndb"
    // label -- see EqGraphWidget.cpp's identical control.
    autoscaleLabelW_ = 82.0f;
    autoscaleLabelH_ = 16.0f;
    autoscaleLabelX_ = graphX_ + graphW_ - autoscaleLabelW_ - 2.0f;
    autoscaleLabelY_ = rangeLabelY_ + rangeLabelH_ + 3.0f;
    beginPath();
    roundedRect(autoscaleLabelX_, autoscaleLabelY_, autoscaleLabelW_, autoscaleLabelH_, 3.0f);
    fillColor(spectrumAutoscale_ ? Color(55, 100, 130, 0.9f) : Color(34, 37, 43, 0.85f));
    fill();
    fillColor(spectrumAutoscale_ ? Color(230, 232, 236) : Color(150, 154, 162));
    fontSize(9.5f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(autoscaleLabelX_ + autoscaleLabelW_ * 0.5f, autoscaleLabelY_ + autoscaleLabelH_ * 0.5f + 1.0f,
         spectrumAutoscale_ ? "Autoscale" : "Fixed Range", nullptr);

    // Freq/gain/Q readout above whichever node is being dragged *or*
    // merely hovered -- see EqGraphWidget.cpp's identical comment. For a
    // band with dynamics on, a second line always shows its *live*
    // dynamic gain right now (getCurrentGainDb(), the same real-time
    // readback the shaded region and the moving combined curve already
    // use) -- kept permanently visible (not just appearing once it
    // happens to differ from the resting gain) so hovering doesn't cause
    // the box to change shape/size as the number crosses in and out of
    // "differs enough to matter"; showing "Live +0.0dB" at rest is exactly
    // as informative as showing nothing, and steadier. A band with
    // dynamics *off* has no live line at all -- there's nothing to show
    // that the resting gain above it doesn't already say, since it can
    // never move.
    const int readoutBand = dragBand_ >= 0 ? dragBand_ : hoverBand_;
    if (readoutBand >= 0 && readoutBand < bandCount)
    {
        const DynamicEqBand b = host_->getBand(readoutBand);
        const float nx = freqToX(b.freqHz);
        const float ny = gainToY(b.gainDb);

        char lineTop[64];
        const std::string freqStr = formatFreqLabel(b.freqHz);
        std::snprintf(lineTop, sizeof(lineTop), "%s   %+.1fdB   Q %.2f", freqStr.c_str(), b.gainDb, b.q);

        char lineLive[24] = "";
        double liveGainDb = b.gainDb;
        if (b.dynamicsEnabled)
        {
            liveGainDb = host_->getCurrentGainDb(readoutBand);
            std::snprintf(lineLive, sizeof(lineLive), "Live %+.1fdB", liveGainDb);
        }

        fontSize(11.5f);
        const size_t widerLen = b.dynamicsEnabled ? std::max(std::strlen(lineTop), std::strlen(lineLive))
                                                   : std::strlen(lineTop);
        const float textW = float(widerLen) * 6.4f + 14.0f;
        constexpr float kLineH = 16.0f;
        const float textH = b.dynamicsEnabled ? kLineH * 2.0f + 4.0f : 18.0f;
        float lx = nx - textW * 0.5f;
        lx = std::max(graphX_, std::min(graphX_ + graphW_ - textW, lx));
        float ly = ny - 22.0f - textH;
        if (ly < graphY_) ly = ny + 16.0f; // would go above the graph -- show it below the node instead

        beginPath();
        roundedRect(lx, ly, textW, textH, 4.0f);
        fillColor(Color(20, 21, 25, 0.92f));
        fill();
        strokeColor(Color(70, 74, 82, 0.9f));
        strokeWidth(1.0f);
        stroke();
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        if (b.dynamicsEnabled)
        {
            fillColor(Color(235, 237, 240));
            text(lx + textW * 0.5f, ly + kLineH * 0.5f + 1.0f, lineTop, nullptr);
            fillColor(liveGainDb < -0.05 ? Color(235, 195, 60) : liveGainDb > 0.05 ? Color(120, 190, 235) : Color(180, 184, 190));
            text(lx + textW * 0.5f, ly + kLineH * 1.5f + 3.0f, lineLive, nullptr);
        }
        else
        {
            fillColor(Color(235, 237, 240));
            text(lx + textW * 0.5f, ly + textH * 0.5f + 0.5f, lineTop, nullptr);
        }
    }
}

// ---- interaction ------------------------------------------------------

bool DynEqGraphWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != kMouseButtonLeft) return false;
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (ev.press)
    {
        if (x >= rangeLabelX_ && x < rangeLabelX_ + rangeLabelW_ &&
            y >= rangeLabelY_ && y < rangeLabelY_ + rangeLabelH_)
        {
            rangeLabelPressed_ = true;
            rangeLabelDragging_ = false;
            rangeLabelDragStartY_ = y;
            rangeLabelDragStartValue_ = gainRangeDb_;
            return true;
        }

        if (x >= autoscaleLabelX_ && x < autoscaleLabelX_ + autoscaleLabelW_ &&
            y >= autoscaleLabelY_ && y < autoscaleLabelY_ + autoscaleLabelH_)
        {
            spectrumAutoscale_ = !spectrumAutoscale_;
            if (onSpectrumAutoscaleChanged_) onSpectrumAutoscaleChanged_(spectrumAutoscale_);
            repaint();
            return true;
        }

        const bool inGraph = x >= graphX_ && x < graphX_ + graphW_ && y >= graphY_ && y < graphY_ + graphH_;
        if (!inGraph) return false;
        const int hit = nodeAt(x, y);

        if (hit >= 0)
        {
            const auto now = std::chrono::steady_clock::now();
            constexpr std::chrono::milliseconds kDoubleClickWindow(400);
            if (hit == lastClickBand_ && (now - lastClickTime_) < kDoubleClickWindow)
            {
                host_->removeBand(hit);
                lastClickBand_ = -1;
                if (selectedBand_ == hit) selectedBand_ = -1;
                repaint();
                return true;
            }
            lastClickBand_ = hit;
            lastClickTime_ = now;

            dragBand_ = hit;
            selectedBand_ = hit;
            if (onSelectionChanged_) onSelectionChanged_(hit);
            repaint();
            return true;
        }

        const double freq = xToFreq(float(x));
        const double gain = yToGain(float(y));
        host_->addBand(freq, gain);
        dragBand_ = host_->getBandCount() - 1;
        selectedBand_ = dragBand_;
        lastClickBand_ = -1;
        if (onSelectionChanged_) onSelectionChanged_(dragBand_);
        repaint();
        return true;
    }

    if (rangeLabelPressed_)
    {
        const bool wasDragging = rangeLabelDragging_;
        rangeLabelPressed_ = false;
        rangeLabelDragging_ = false;
        if (!wasDragging)
        {
            gainRangeDb_ = nextGainRange(gainRangeDb_);
            if (onGainRangeChanged_) onGainRangeChanged_(gainRangeDb_);
        }
        repaint();
        return true;
    }
    if (dragBand_ >= 0) { dragBand_ = -1; return true; }
    return false;
}

bool DynEqGraphWidget::onMotion(const MotionEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (rangeLabelPressed_)
    {
        const double dy = rangeLabelDragStartY_ - y;
        if (!rangeLabelDragging_ && std::fabs(dy) > kRangeDragDeadZonePx) rangeLabelDragging_ = true;
        if (rangeLabelDragging_)
        {
            double newRange = rangeLabelDragStartValue_ * std::pow(2.0, dy / kRangeDragPxPerOctave);
            newRange = std::max(kRangeDragMinDb, std::min(kRangeDragMaxDb, newRange));
            gainRangeDb_ = newRange;
            if (onGainRangeChanged_) onGainRangeChanged_(gainRangeDb_);
            repaint();
        }
        return true;
    }

    if (dragBand_ >= 0)
    {
        const double cx = std::max(double(graphX_), std::min(double(graphX_ + graphW_ - 1), x));
        const double cy = std::max(double(graphY_), std::min(double(graphY_ + graphH_ - 1), y));
        dragBandTo(dragBand_, cx, cy);
        repaint();
        return true;
    }

    const int prevHover = hoverBand_;
    const bool inGraph = x >= graphX_ && x < graphX_ + graphW_ && y >= graphY_ && y < graphY_ + graphH_;
    hoverBand_ = inGraph ? nodeAt(x, y) : -1;
    if (hoverBand_ != prevHover) repaint();
    return false;
}

bool DynEqGraphWidget::onScroll(const ScrollEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();
    const int hit = nodeAt(x, y);
    if (hit < 0) return false;

    const DynamicEqBand b = host_->getBand(hit);
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;
    // Alt+scroll = 10x finer -- see ../eq/EqGraphWidget.cpp's identical
    // comment for the full reasoning, applied project-wide.
    constexpr double kFactor = 1.08;
    const double factor = (ev.mod & kModifierAlt) ? 1.0 + (kFactor - 1.0) / 10.0 : kFactor;
    const double newQ = std::max(0.05, std::min(100.0, b.q * (dir > 0.0 ? factor : 1.0 / factor)));
    host_->setBandQ(hit, newQ);
    repaint();
    return true;
}

END_NAMESPACE_DGL
