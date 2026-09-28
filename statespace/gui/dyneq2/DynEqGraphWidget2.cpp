// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "DynEqGraphWidget2.hpp"

#include "../../plugin/dsp/DynamicEqModel2.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

START_NAMESPACE_DGL

using ssfilter::DynamicEqBand2;
using ssfilter::DynamicEqModel2;
using ssfilter::EqBandType;
using ssfilter::EqChannel;
using ssfilter::EqCrossFeed;
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

// 60 added specifically so a crossfeed transit's own -60dB default
// baseline (DynamicEqModel2::crossFeedGainLtoRDb/RtoLDb) is actually
// visible on the graph instead of clipped off the bottom of a narrower
// range -- same fix, same reasoning, as the sibling Parametric EQ's own
// EqGraphWidget.cpp (see that file's identical comment).
constexpr double kGainRangePresets[] = { 3.0, 6.0, 12.0, 30.0, 60.0 };
constexpr int kNumGainRangePresets = int(sizeof(kGainRangePresets) / sizeof(kGainRangePresets[0]));

double nextGainRange(double current)
{
    int idx = 0;
    for (int i = 0; i < kNumGainRangePresets; ++i)
        if (std::fabs(kGainRangePresets[i] - current) < 0.01) { idx = i; break; }
    return kGainRangePresets[(idx + 1) % kNumGainRangePresets];
}

// Right-click context menu -- same shape/presets as the sibling
// Parametric EQ's own EqGraphWidget.cpp.
struct MenuTypeItem { const char* label; EqBandType type; };
const MenuTypeItem kMenuTypeItems[] = {
    { "Bell", EqBandType::Bell },
    { "Low Shelf", EqBandType::LowShelf },
    { "High Shelf", EqBandType::HighShelf },
    { "Band Shelf", EqBandType::BandShelf },
};
constexpr int kNumMenuTypeItems = int(sizeof(kMenuTypeItems) / sizeof(kMenuTypeItems[0]));

struct HarmonicPreset { const char* label; double factor; };
const HarmonicPreset kHarmonicPresets[] = {
    { "x2 (Octave)", 2.0 },
    { "x3", 3.0 },
    { "x4", 4.0 },
    { "\xC3\xB7""2 (Sub-Octave)", 0.5 }, // UTF-8 division sign
};
constexpr int kNumHarmonicPresets = int(sizeof(kHarmonicPresets) / sizeof(kHarmonicPresets[0]));

} // namespace

std::string DynEqGraphWidget2::formatFreqLabel(double freqHz)
{
    char buf[32];
    if (freqHz >= 1000.0) std::snprintf(buf, sizeof(buf), "%.2fkHz", freqHz / 1000.0);
    else std::snprintf(buf, sizeof(buf), "%.0fHz", freqHz);
    return std::string(buf);
}

DynEqGraphWidget2::DynEqGraphWidget2(Widget* parent, ssgui::IDynEqEditorHost2* host)
    : NanoSubWidget(parent), host_(host)
{
    loadSharedResources();
    for (float& v : spectrumInDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRDb_) v = SpectrumAnalyzer::kFloorDb;
}

void DynEqGraphWidget2::setInputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumInDb_[i] = db[i];
}

void DynEqGraphWidget2::setOutputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutDb_[i] = db[i];
}

void DynEqGraphWidget2::setOutputSpectrumDataL(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutLDb_[i] = db[i];
}

void DynEqGraphWidget2::setOutputSpectrumDataR(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutRDb_[i] = db[i];
}

float DynEqGraphWidget2::freqToX(double freqHz) const
{
    const double t = std::log(freqHz / kFMin) / std::log(kFMax / kFMin);
    return graphX_ + float(std::max(0.0, std::min(1.0, t))) * graphW_;
}

double DynEqGraphWidget2::xToFreq(float px) const
{
    const double t = double(px - graphX_) / double(std::max(1.0f, graphW_));
    return kFMin * std::pow(kFMax / kFMin, std::max(0.0, std::min(1.0, t)));
}

float DynEqGraphWidget2::gainToY(double gainDb) const
{
    const double t = (gainRangeDb_ - gainDb) / (2.0 * gainRangeDb_);
    return graphY_ + float(t) * graphH_;
}

double DynEqGraphWidget2::yToGain(float py) const
{
    const double t = double(py - graphY_) / double(std::max(1.0f, graphH_));
    return gainRangeDb_ - t * (2.0 * gainRangeDb_);
}

float DynEqGraphWidget2::spectrumToY(double db) const
{
    const double t = (spectrumRangeHi_ - db) / (spectrumRangeHi_ - spectrumRangeLo_);
    return graphY_ + float(std::max(0.0, std::min(1.0, t))) * graphH_;
}

int DynEqGraphWidget2::nodeAt(double px, double py) const
{
    int best = -1;
    float bestDist = kHitRadius;
    for (int i = 0; i < host_->getBandCount(); ++i)
    {
        const DynamicEqBand2 b = host_->getBand(i);
        const float nx = freqToX(b.freqHz);
        const float ny = gainToY(b.gainDb);
        const float dist = std::sqrt(float((px - nx) * (px - nx) + (py - ny) * (py - ny)));
        if (dist < bestDist) { bestDist = dist; best = i; }
    }
    return best;
}

int DynEqGraphWidget2::edgeHandleAt(double px, double py, int& edge) const
{
    constexpr double kHalfW = 6.0, kHalfH = 10.0; // a little larger than the drawn 6x14 handle
    int best = -1;
    double bestDist = kHalfW + 1.0;
    for (int i = 0; i < host_->getBandCount(); ++i)
    {
        const DynamicEqBand2 b = host_->getBand(i);
        if (b.type != EqBandType::BandShelf) continue;
        if (std::fabs(py - gainToY(b.gainDb)) > kHalfH) continue;
        const double d1 = std::fabs(px - freqToX(ssfilter::dyneq2::bandShelfLowCornerHz(b)));
        const double d2 = std::fabs(px - freqToX(ssfilter::dyneq2::bandShelfHighCornerHz(b)));
        if (d1 < bestDist) { bestDist = d1; best = i; edge = 1; }
        if (d2 < bestDist) { bestDist = d2; best = i; edge = 2; }
    }
    return best;
}

void DynEqGraphWidget2::dragBandTo(int index, double px, double py)
{
    if (index < 0 || index >= host_->getBandCount()) return;
    const double freq = std::max(kFMin, std::min(kFMax, xToFreq(float(px))));
    const DynamicEqBand2 b = host_->getBand(index);
    if (b.type == EqBandType::BandShelf && index == dragBand_ && dragEdge_ != 0)
    {
        // A corner handle: bandwidth symmetric around the fixed centre, same
        // as EqGraphWidget::dragBandTo().
        host_->setBandBandwidth(index, 2.0 * std::fabs(std::log2(freq / std::max(1.0, b.freqHz))));
        return;
    }
    host_->setBandFreq(index, freq);
    const double gain = std::max(-gainRangeDb_, std::min(gainRangeDb_, yToGain(float(py))));
    host_->setBandGain(index, gain);
}

void DynEqGraphWidget2::openContextMenu(int band, float px, float py)
{
    menuOpen_ = true;
    menuBand_ = band;

    // Same ordering/grouping as the sibling Parametric EQ's own
    // EqGraphWidget::openContextMenu() -- Delete/Duplicate first, then
    // "Create Harmonic Twin" (always offered, chains a further harmonic
    // onto an existing group -- see dyneq2::createHarmonicTwin()'s own
    // comment), "Unlink Harmonic" only when `band` already is, then the
    // three-item Type list. No Slope section -- this plugin has no
    // Low/High Pass types at all.
    menuItems_.clear();
    menuItems_.push_back({ "Delete Band", MenuEntry::Kind::Delete, EqBandType::Bell, false });
    menuItems_.push_back({ "Duplicate Band", MenuEntry::Kind::Duplicate, EqBandType::Bell, false });

    bool firstHarmonic = true;
    for (const auto& p : kHarmonicPresets)
    {
        menuItems_.push_back({ p.label, MenuEntry::Kind::HarmonicFactor, EqBandType::Bell, firstHarmonic, p.factor });
        firstHarmonic = false;
    }
    menuItems_.push_back({ "Custom Factor...", MenuEntry::Kind::HarmonicCustom, EqBandType::Bell, false, 1.0 });
    if (band >= 0 && band < host_->getBandCount() && host_->getBand(band).harmonicGroup != 0)
        menuItems_.push_back({ "Unlink Harmonic", MenuEntry::Kind::Unlink, EqBandType::Bell, true, 1.0 });

    bool firstType = true;
    for (const auto& t : kMenuTypeItems)
    {
        menuItems_.push_back({ t.label, MenuEntry::Kind::Type, t.type, firstType });
        firstType = false;
    }

    menuW_ = kMenuW;
    menuH_ = kMenuRowH * float(menuItems_.size());
    menuX_ = std::min(px, graphX_ + graphW_ - menuW_);
    menuY_ = std::min(py, graphY_ + graphH_ - menuH_);
    menuX_ = std::max(graphX_, menuX_);
    menuY_ = std::max(graphY_, menuY_);
    pressedMenuItem_ = -1;
}

int DynEqGraphWidget2::menuItemAt(double px, double py) const
{
    if (!menuOpen_) return -1;
    if (px < menuX_ || px >= menuX_ + menuW_ || py < menuY_ || py >= menuY_ + menuH_) return -1;
    const int item = int((py - menuY_) / kMenuRowH);
    return (item >= 0 && item < int(menuItems_.size())) ? item : -1;
}

// ---- drawing --------------------------------------------------------------

void DynEqGraphWidget2::onNanoDisplay()
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
    const DynamicEqModel2 model = host_->getModel();

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
        const bool perChannel = showOut && ssfilter::dyneq2::hasPerChannelBands(model);

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
    // (ratio > 1) can pull it down -- downward and upward compression shown as
    // one region that can extend either way, or both at once. Whichever
    // direction isn't active for this band just collapses that edge back
    // to gainDb (its own resting value), the same shape the original
    // downward-only version drew. This is the FabFilter screenshots'
    // shaded green/purple region around a dynamic node. Drawn before
    // every other curve so they stay legible on top of it.
    for (int i = 0; i < bandCount; ++i)
    {
        const DynamicEqBand2 b = host_->getBand(i);
        const bool downActive = b.ratio > 1.01;
        const bool upActive = b.ratioUp > 1.01;
        if (!b.enabled || !b.dynamicsEnabled || b.rangeDb <= 0.0 || !(downActive || upActive)) continue;
        const double topGainDb = upActive ? b.gainDb + b.rangeDb : b.gainDb;
        const double bottomGainDb = downActive ? b.gainDb - b.rangeDb : b.gainDb;
        // A crossfeed band's own curve rides on the model's own
        // crossFeedGainLtoRDb/RtoLDb transit baseline -- see
        // evaluateBandResponse()'s own comment on `extraFlatGainDb`.
        const double extraFlatGainDb = b.crossFeed == EqCrossFeed::LeftToRight ? model.crossFeedGainLtoRDb
                                      : b.crossFeed == EqCrossFeed::RightToLeft ? model.crossFeedGainRtoLDb
                                      : 0.0;
        Color fillCol = bandColor(i);
        fillCol.alpha = (i == selectedBand_) ? 0.20f : 0.11f;

        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::dyneq2::evaluateBandResponse(b, sampleRate_, f, topGainDb, extraFlatGainDb);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            if (p == 0) moveTo(x, y); else lineTo(x, y);
        }
        for (int p = kCurvePoints; p >= 0; --p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::dyneq2::evaluateBandResponse(b, sampleRate_, f, bottomGainDb, extraFlatGainDb);
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
        const DynamicEqBand2 b = host_->getBand(i);
        if (!b.enabled) continue;
        Color col = bandColor(i);
        col.alpha = 0.45f;
        const double extraFlatGainDb = b.crossFeed == EqCrossFeed::LeftToRight ? model.crossFeedGainLtoRDb
                                      : b.crossFeed == EqCrossFeed::RightToLeft ? model.crossFeedGainRtoLDb
                                      : 0.0;
        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::dyneq2::evaluateBandResponse(b, sampleRate_, f, b.gainDb, extraFlatGainDb);
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
    const bool perChannelCurve = ssfilter::dyneq2::hasPerChannelBands(model);
    auto drawRestingCurve = [&](EqChannel listenerChannel, Color color)
    {
        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = perChannelCurve
                ? ssfilter::dyneq2::evaluateChannelResponse(model, sampleRate_, f, listenerChannel)
                : ssfilter::dyneq2::evaluateCombinedResponse(model, sampleRate_, f);
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
    // gain (DynamicEqEngine2::getCurrentGainDb(), via
    // IDynEqEditorHost2::getCurrentGainDb()) instead of its resting gainDb.
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
                    ? ssfilter::dyneq2::evaluateChannelResponse(model, sampleRate_, f, listenerChannel, livePtr)
                    : ssfilter::dyneq2::evaluateCombinedResponse(model, sampleRate_, f, livePtr);
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
        const DynamicEqBand2 b = host_->getBand(i);
        const float nx = freqToX(b.freqHz);
        const float ny = gainToY(b.gainDb);
        const bool selected = (i == selectedBand_);
        const bool hovered = (i == hoverBand_);
        const bool auditionHeld = (i == auditionHoldBand_);
        const Color col = b.enabled ? bandColor(i) : Color(110, 112, 116);

        // Band Shelf: plateau line + corner handles, same as EqGraphWidget.
        if (b.type == EqBandType::BandShelf)
        {
            const float x1 = freqToX(ssfilter::dyneq2::bandShelfLowCornerHz(b));
            const float x2 = freqToX(ssfilter::dyneq2::bandShelfHighCornerHz(b));
            Color lineCol = col;
            lineCol.alpha = 0.55f;
            beginPath();
            moveTo(x1, ny);
            lineTo(x2, ny);
            strokeColor(lineCol);
            strokeWidth(1.4f);
            stroke();
            for (const float hx : { x1, x2 })
            {
                beginPath();
                roundedRect(hx - 3.0f, ny - 7.0f, 6.0f, 14.0f, 2.0f);
                fillColor(col);
                fill();
                strokeColor(Color(15, 16, 19));
                strokeWidth(1.0f);
                stroke();
            }
        }

        if (selected || hovered)
        {
            beginPath();
            circle(nx, ny, 9.5f);
            strokeColor(Color(255, 255, 255, selected ? 0.8f : 0.43f));
            strokeWidth(1.6f);
            stroke();
        }
        if (auditionHeld)
        {
            // Gold ring for the momentary right-button audition, same as
            // EqGraphWidget's -- distinct from the white selection ring.
            beginPath();
            circle(nx, ny, 10.5f);
            strokeColor(Color(235, 195, 60, 0.9f));
            strokeWidth(2.0f);
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

        // Crossfeed ring -- same idiom as EqGraphWidget's own identical
        // ring (its own dedicated radius so both this and the channel
        // ring above can show on the same node at once), purple to read
        // as a genuinely different kind of routing from Channel's own
        // blue/red.
        if (b.crossFeed != EqCrossFeed::None)
        {
            beginPath();
            circle(nx, ny, 16.0f);
            strokeColor(Color(170, 100, 220, 0.85f));
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

    // Crossfeed transit gain knobs -- ported verbatim from the sibling
    // Parametric EQ's own EqGraphWidget.cpp (identical control, identical
    // interaction) -- see CrossFeedGainControl's own comment (.hpp).
    {
        bool anyLtoR = false, anyRtoL = false;
        for (const DynamicEqBand2& b : model.bands)
        {
            if (b.crossFeed == EqCrossFeed::LeftToRight) anyLtoR = true;
            else if (b.crossFeed == EqCrossFeed::RightToLeft) anyRtoL = true;
        }
        xfeedLtoR_.visible = anyLtoR;
        xfeedRtoL_.visible = anyRtoL;

        auto drawXfeedControl = [&](CrossFeedGainControl& c, float stackY, const char* prefix, double gainDb)
        {
            c.w = 88.0f;
            c.h = 16.0f;
            c.x = graphX_ + graphW_ - c.w - 2.0f;
            c.y = stackY;
            if (!c.visible) return;
            char buf[24];
            std::snprintf(buf, sizeof(buf), "%s %+.1fdB", prefix, gainDb);
            beginPath();
            roundedRect(c.x, c.y, c.w, c.h, 3.0f);
            fillColor(c.dragging ? Color(90, 60, 120, 0.9f) : Color(60, 42, 78, 0.85f));
            fill();
            fillColor(Color(210, 195, 225));
            fontSize(9.5f);
            textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
            text(c.x + c.w * 0.5f, c.y + c.h * 0.5f + 1.0f, buf, nullptr);
        };
        float stackY = autoscaleLabelY_ + autoscaleLabelH_ + 3.0f;
        drawXfeedControl(xfeedLtoR_, stackY, "L\xE2\x86\x92""R", model.crossFeedGainLtoRDb);
        if (xfeedLtoR_.visible) stackY += xfeedLtoR_.h + 3.0f;
        drawXfeedControl(xfeedRtoL_, stackY, "R\xE2\x86\x92""L", model.crossFeedGainRtoLDb);
    }

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
        const DynamicEqBand2 b = host_->getBand(readoutBand);
        const float nx = freqToX(b.freqHz);
        const float ny = gainToY(b.gainDb);

        char lineTop[64];
        const std::string freqStr = formatFreqLabel(b.freqHz);
        if (b.type == EqBandType::BandShelf)
            std::snprintf(lineTop, sizeof(lineTop), "%s   BW %.2f oct   %+.1fdB", freqStr.c_str(), b.bandwidthOct, b.gainDb);
        else
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

    // Right-click context menu, drawn last so it sits on top of
    // everything -- ported from the sibling EqGraphWidget.cpp's own
    // identical block.
    if (menuOpen_)
    {
        beginPath();
        roundedRect(menuX_, menuY_, menuW_, menuH_, 4.0f);
        fillColor(Color(30, 32, 38, 0.98f));
        fill();
        strokeColor(Color(70, 74, 82));
        strokeWidth(1.0f);
        stroke();

        if (customFactorEditing_)
        {
            std::string liveText;
            host_->isEditingHarmonicFactor(menuBand_, liveText); // false would mean UIDynEq2 already ended the edit; endHarmonicFactorEdit() closes this menu in that case, so menuOpen_ would already be false here
            fillColor(Color(180, 184, 192));
            fontSize(10.5f);
            textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
            text(menuX_ + 8.0f, menuY_ + kMenuRowH * 0.5f, "Harmonic Factor:", nullptr);

            const std::string display = liveText + "|";
            fillColor(Color(235, 195, 60));
            fontSize(13.0f);
            text(menuX_ + 8.0f, menuY_ + kMenuRowH * 1.5f, display.c_str(), nullptr);

            fillColor(Color(140, 144, 152));
            fontSize(9.0f);
            text(menuX_ + 8.0f, menuY_ + kMenuRowH * 2.35f, "Enter to create, Esc to cancel", nullptr);
        }
        else
        {
            const DynamicEqBand2 currentBand = (menuBand_ >= 0 && menuBand_ < bandCount) ? host_->getBand(menuBand_) : DynamicEqBand2{};
            for (int i = 0; i < int(menuItems_.size()); ++i)
            {
                const MenuEntry& e = menuItems_[size_t(i)];
                const float iy = menuY_ + float(i) * kMenuRowH;
                const bool isCurrent = e.kind == MenuEntry::Kind::Type && e.type == currentBand.type;
                if (i == pressedMenuItem_ || isCurrent)
                {
                    beginPath();
                    rect(menuX_ + 1.0f, iy, menuW_ - 2.0f, kMenuRowH);
                    fillColor(isCurrent ? Color(55, 70, 60) : Color(50, 53, 60));
                    fill();
                }
                const bool startsTypeList = e.kind == MenuEntry::Kind::Type &&
                                             (i == 0 || menuItems_[size_t(i - 1)].kind != MenuEntry::Kind::Type);
                if (startsTypeList || e.separatorAbove)
                {
                    beginPath();
                    moveTo(menuX_ + 4.0f, iy);
                    lineTo(menuX_ + menuW_ - 4.0f, iy);
                    strokeColor(Color(60, 63, 70));
                    strokeWidth(1.0f);
                    stroke();
                }
                fillColor(e.kind == MenuEntry::Kind::Delete ? Color(230, 150, 150) :
                          e.kind == MenuEntry::Kind::Unlink ? Color(230, 195, 120) :
                          Color(210, 214, 222));
                fontSize(11.0f);
                textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
                text(menuX_ + 8.0f, iy + kMenuRowH * 0.5f, e.label.c_str(), nullptr);
            }
        }
    }
}

// ---- interaction ------------------------------------------------------

bool DynEqGraphWidget2::onMouse(const MouseEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    // While typing a custom harmonic factor, every mouse gesture is
    // suppressed until UIDynEq2's own commit-on-Enter/cancel-on-Escape
    // logic ends the edit and calls endHarmonicFactorEdit(), which closes
    // this menu -- see the sibling EqGraphWidget.cpp's identical guard.
    if (customFactorEditing_) return true;

    // The context menu, if open, owns every left click until it's
    // resolved (an item picked, or a click outside dismisses it).
    if (menuOpen_ && ev.button == kMouseButtonLeft)
    {
        if (ev.press)
        {
            pressedMenuItem_ = menuItemAt(x, y);
            if (pressedMenuItem_ < 0) { closeContextMenu(); repaint(); }
            return true;
        }
        const int released = menuItemAt(x, y);
        const int band = menuBand_;
        if (released >= 0 && released == pressedMenuItem_ && band >= 0 && released < int(menuItems_.size()))
        {
            const MenuEntry& e = menuItems_[size_t(released)];
            if (e.kind == MenuEntry::Kind::Delete) host_->removeBand(band);
            else if (e.kind == MenuEntry::Kind::Duplicate) host_->duplicateBand(band);
            else if (e.kind == MenuEntry::Kind::Type) host_->setBandType(band, e.type);
            else if (e.kind == MenuEntry::Kind::HarmonicFactor) host_->createHarmonicTwin(band, e.harmonicFactor);
            else if (e.kind == MenuEntry::Kind::Unlink) host_->unlinkHarmonic(band);
            else // Kind::HarmonicCustom -- switches into text-input mode instead of closing the menu
            {
                customFactorEditing_ = true;
                menuH_ = kMenuRowH * 3.0f; // label + live value + hint, see onNanoDisplay()'s customFactorEditing_ branch
                host_->beginHarmonicFactorEdit(band);
                repaint();
                return true; // skip closeContextMenu() below -- the menu stays open, just in a different mode
            }
        }
        closeContextMenu();
        repaint();
        return true;
    }

    if (ev.button == kMouseButtonRight)
    {
        if (!ev.press)
        {
            if (rightPressPending_)
            {
                // Never crossed the drag dead zone: a plain click -- open
                // the context menu now.
                rightPressPending_ = false;
                openContextMenu(rightPressBand_, float(rightPressX_), float(rightPressY_));
                rightPressBand_ = -1;
                repaint();
                return true;
            }
            // Release of a right-button drag: end its momentary audition.
            if (auditionHoldBand_ >= 0)
            {
                host_->endBandAudition();
                auditionHoldBand_ = -1;
                dragBand_ = -1;
                repaint();
            }
            return menuOpen_;
        }
        const bool inGraph = x >= graphX_ && x < graphX_ + graphW_ && y >= graphY_ && y < graphY_ + graphH_;
        if (!inGraph) return false;
        const int hit = nodeAt(x, y);
        if (hit >= 0)
        {
            selectedBand_ = hit;
            if (onSelectionChanged_) onSelectionChanged_(hit);
            // Menu or audition-drag is decided by onMotion() (dead zone
            // crossed) or the release above (it never was).
            rightPressPending_ = true;
            rightPressBand_ = hit;
            rightPressX_ = x;
            rightPressY_ = y;
            repaint();
            return true;
        }
        if (menuOpen_) { closeContextMenu(); repaint(); return true; }
        return false;
    }

    if (ev.button != kMouseButtonLeft) return false;

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

        // Crossfeed transit gain knobs -- see EqGraphWidget.cpp's
        // identical press handling.
        if (xfeedLtoR_.visible && x >= xfeedLtoR_.x && x < xfeedLtoR_.x + xfeedLtoR_.w &&
            y >= xfeedLtoR_.y && y < xfeedLtoR_.y + xfeedLtoR_.h)
        {
            xfeedLtoR_.pressed = true;
            xfeedLtoR_.dragging = false;
            xfeedLtoR_.dragStartY = y;
            xfeedLtoR_.dragStartValue = host_->getModel().crossFeedGainLtoRDb;
            return true;
        }
        if (xfeedRtoL_.visible && x >= xfeedRtoL_.x && x < xfeedRtoL_.x + xfeedRtoL_.w &&
            y >= xfeedRtoL_.y && y < xfeedRtoL_.y + xfeedRtoL_.h)
        {
            xfeedRtoL_.pressed = true;
            xfeedRtoL_.dragging = false;
            xfeedRtoL_.dragStartY = y;
            xfeedRtoL_.dragStartValue = host_->getModel().crossFeedGainRtoLDb;
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
            dragEdge_ = 0;
            selectedBand_ = hit;
            if (onSelectionChanged_) onSelectionChanged_(hit);
            repaint();
            return true;
        }

        // A Band Shelf corner handle -- drag that corner (bandwidth).
        int edge = 0;
        const int edgeHit = edgeHandleAt(x, y, edge);
        if (edgeHit >= 0)
        {
            dragBand_ = edgeHit;
            dragEdge_ = edge;
            selectedBand_ = edgeHit;
            lastClickBand_ = -1;
            if (onSelectionChanged_) onSelectionChanged_(edgeHit);
            repaint();
            return true;
        }

        const double freq = xToFreq(float(x));
        const double gain = yToGain(float(y));
        host_->addBand(freq, gain);
        dragBand_ = host_->getBandCount() - 1;
        dragEdge_ = 0;
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
    if (xfeedLtoR_.pressed) { xfeedLtoR_.pressed = false; xfeedLtoR_.dragging = false; repaint(); return true; }
    if (xfeedRtoL_.pressed) { xfeedRtoL_.pressed = false; xfeedRtoL_.dragging = false; repaint(); return true; }
    if (dragBand_ >= 0) { dragBand_ = -1; return true; }
    return false;
}

bool DynEqGraphWidget2::onMotion(const MotionEvent& ev)
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

    if (xfeedLtoR_.pressed)
    {
        const double dy = xfeedLtoR_.dragStartY - y;
        if (!xfeedLtoR_.dragging && std::fabs(dy) > kXfeedDragDeadZonePx) xfeedLtoR_.dragging = true;
        if (xfeedLtoR_.dragging)
        {
            const double v = std::max(kXfeedGainMinDb, std::min(kXfeedGainMaxDb, xfeedLtoR_.dragStartValue + dy / kXfeedDragPxPerDb));
            host_->setCrossFeedGainLtoRDb(v);
            repaint();
        }
        return true;
    }
    if (xfeedRtoL_.pressed)
    {
        const double dy = xfeedRtoL_.dragStartY - y;
        if (!xfeedRtoL_.dragging && std::fabs(dy) > kXfeedDragDeadZonePx) xfeedRtoL_.dragging = true;
        if (xfeedRtoL_.dragging)
        {
            const double v = std::max(kXfeedGainMinDb, std::min(kXfeedGainMaxDb, xfeedRtoL_.dragStartValue + dy / kXfeedDragPxPerDb));
            host_->setCrossFeedGainRtoLDb(v);
            repaint();
        }
        return true;
    }

    if (rightPressPending_)
    {
        const double dx = x - rightPressX_;
        const double dy = y - rightPressY_;
        if (std::sqrt(dx * dx + dy * dy) > kRightDragDeadZonePx)
        {
            // A drag, not a click: audition while dragging (moving the band
            // through the same dragBand_ path as a left drag); no menu.
            rightPressPending_ = false;
            auditionHoldBand_ = rightPressBand_;
            dragBand_ = rightPressBand_;
            dragEdge_ = 0;
            host_->beginBandAudition(rightPressBand_);
            rightPressBand_ = -1;
            repaint();
        }
        else
        {
            return true; // still deciding
        }
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
    if (inGraph && hoverBand_ < 0)
    {
        int edge = 0;
        hoverBand_ = edgeHandleAt(x, y, edge); // pointing at a corner handle shows the band's readout too
    }
    if (hoverBand_ != prevHover) repaint();
    return false;
}

bool DynEqGraphWidget2::onScroll(const ScrollEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    // Crossfeed transit gain knobs -- fine scroll-to-nudge, see
    // EqGraphWidget.cpp's identical control.
    auto scrollXfeed = [&](CrossFeedGainControl& c, void (ssgui::IDynEqEditorHost2::*setter)(double), double current)
    {
        if (!c.visible || x < c.x || x >= c.x + c.w || y < c.y || y >= c.y + c.h) return false;
        const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
        if (dir != 0.0)
        {
            const double step = (ev.mod & kModifierAlt) ? 0.05 : 0.5; // Alt = 10x finer, as everywhere
            const double v = std::max(kXfeedGainMinDb, std::min(kXfeedGainMaxDb, current + dir * step));
            (host_->*setter)(v);
            repaint();
        }
        return true;
    };
    if (scrollXfeed(xfeedLtoR_, &ssgui::IDynEqEditorHost2::setCrossFeedGainLtoRDb, host_->getModel().crossFeedGainLtoRDb)) return true;
    if (scrollXfeed(xfeedRtoL_, &ssgui::IDynEqEditorHost2::setCrossFeedGainRtoLDb, host_->getModel().crossFeedGainRtoLDb)) return true;

    int hit = nodeAt(x, y);
    if (hit < 0)
    {
        int edge = 0;
        hit = edgeHandleAt(x, y, edge);
    }
    if (hit < 0) return false;

    const DynamicEqBand2 b = host_->getBand(hit);
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;
    if (b.type == EqBandType::BandShelf)
    {
        // A Band Shelf scrolls its bandwidth (it has no Q) -- up = wider.
        constexpr double kBwFactor = 1.08;
        const double bwFactor = (ev.mod & kModifierAlt) ? 1.0 + (kBwFactor - 1.0) / 10.0 : kBwFactor;
        host_->setBandBandwidth(hit, dir > 0.0 ? b.bandwidthOct * bwFactor : b.bandwidthOct / bwFactor);
        repaint();
        return true;
    }
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
