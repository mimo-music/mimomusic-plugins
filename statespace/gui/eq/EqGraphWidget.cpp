// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "EqGraphWidget.hpp"

#include "../../plugin/dsp/EqModel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

START_NAMESPACE_DGL

using ssfilter::EqBand;
using ssfilter::EqBandType;
using ssfilter::EqChannel;
using ssfilter::EqCrossFeed;
using ssfilter::EqModel;
using ssfilter::SpectrumAnalyzer;

namespace {

// Cycled by band index so adjacent bands are visually distinguishable at
// a glance, same spirit as MatrixGridWidget's heatmap but for identity
// rather than magnitude.
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

// Preset +-dB ranges the "±Ndb" label cycles through -- a tight 3dB range
// for soft, careful moves, a wide 30dB range for aggressive ones, plus
// 6/12 in between. 60 added specifically so a crossfeed transit's own
// -60dB default baseline (EqModel::crossFeedGainLtoRDb/RtoLDb) is
// actually visible on the graph instead of clipped off the bottom of a
// narrower range.
constexpr double kGainRangePresets[] = { 3.0, 6.0, 12.0, 30.0, 60.0 };
constexpr int kNumGainRangePresets = int(sizeof(kGainRangePresets) / sizeof(kGainRangePresets[0]));

double nextGainRange(double current)
{
    int idx = 0;
    for (int i = 0; i < kNumGainRangePresets; ++i)
        if (std::fabs(kGainRangePresets[i] - current) < 0.01) { idx = i; break; }
    return kGainRangePresets[(idx + 1) % kNumGainRangePresets];
}

// Right-click context menu items -- delete/duplicate plus every band type,
// mirroring BandListWidget's type-cycle button but as a direct pick rather
// than a cycle (a menu can show every option at once).
struct MenuItem { const char* label; bool isDelete; EqBandType type; };
const MenuItem kMenuItems[] = {
    { "Delete Band", true, EqBandType::Bell },
    { "Bell", false, EqBandType::Bell },
    { "Low Shelf", false, EqBandType::LowShelf },
    { "High Shelf", false, EqBandType::HighShelf },
    { "Band Shelf", false, EqBandType::BandShelf },
    { "Low Pass", false, EqBandType::LowPass },
    { "High Pass", false, EqBandType::HighPass },
    { "Notch", false, EqBandType::Notch },
    { "Band Pass", false, EqBandType::BandPass },
    { "All Pass", false, EqBandType::AllPass },
};
constexpr int kNumMenuItems = int(sizeof(kMenuItems) / sizeof(kMenuItems[0]));
constexpr float kMenuRowH = 19.0f;
constexpr float kMenuW = 150.0f; // widened from 108 for "Create Harmonic Twin" rows/"Unlink Harmonic"/"Custom Factor..."

// "Create Harmonic Twin" quick presets -- x2 covers the standard octave
// double; x3/x4/half cover the other common cases without forcing a
// custom-factor entry for them. "Custom Factor..." (handled separately,
// see MenuEntry::Kind::HarmonicCustom) covers anything else.
struct HarmonicPreset { const char* label; double factor; };
const HarmonicPreset kHarmonicPresets[] = {
    { "x2 (Octave)", 2.0 },
    { "x3", 3.0 },
    { "x4", 4.0 },
    { "\xC3\xB7""2 (Sub-Octave)", 0.5 }, // UTF-8 division sign
};
constexpr int kNumHarmonicPresets = int(sizeof(kHarmonicPresets) / sizeof(kHarmonicPresets[0]));

} // namespace

std::string EqGraphWidget::formatFreqLabel(double freqHz)
{
    char buf[32];
    if (freqHz >= 1000.0) std::snprintf(buf, sizeof(buf), "%.2fkHz", freqHz / 1000.0);
    else std::snprintf(buf, sizeof(buf), "%.0fHz", freqHz);
    return std::string(buf);
}

EqGraphWidget::EqGraphWidget(Widget* parent, ssgui::IEqEditorHost* host)
    : NanoSubWidget(parent), host_(host)
{
    loadSharedResources();
    for (float& v : spectrumInDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLDb_) v = SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRDb_) v = SpectrumAnalyzer::kFloorDb;
}

void EqGraphWidget::setInputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumInDb_[i] = db[i];
}

void EqGraphWidget::setOutputSpectrumData(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutDb_[i] = db[i];
}

void EqGraphWidget::setOutputSpectrumDataL(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutLDb_[i] = db[i];
}

void EqGraphWidget::setOutputSpectrumDataR(const float* db, int count)
{
    const int n = std::min(count, SpectrumAnalyzer::kNumBands);
    for (int i = 0; i < n; ++i) spectrumOutRDb_[i] = db[i];
}

float EqGraphWidget::freqToX(double freqHz) const
{
    const double t = std::log(freqHz / kFMin) / std::log(kFMax / kFMin);
    return graphX_ + float(std::max(0.0, std::min(1.0, t))) * graphW_;
}

double EqGraphWidget::xToFreq(float px) const
{
    const double t = double(px - graphX_) / double(std::max(1.0f, graphW_));
    return kFMin * std::pow(kFMax / kFMin, std::max(0.0, std::min(1.0, t)));
}

float EqGraphWidget::gainToY(double gainDb) const
{
    const double t = (gainRangeDb_ - gainDb) / (2.0 * gainRangeDb_);
    return graphY_ + float(t) * graphH_;
}

double EqGraphWidget::yToGain(float py) const
{
    const double t = double(py - graphY_) / double(std::max(1.0f, graphH_));
    return gainRangeDb_ - t * (2.0 * gainRangeDb_);
}

float EqGraphWidget::spectrumToY(double db) const
{
    const double t = (spectrumRangeHi_ - db) / (spectrumRangeHi_ - spectrumRangeLo_);
    return graphY_ + float(std::max(0.0, std::min(1.0, t))) * graphH_;
}

int EqGraphWidget::nodeAt(double px, double py) const
{
    int best = -1;
    float bestDist = kHitRadius;
    for (int i = 0; i < host_->getBandCount(); ++i)
    {
        const EqBand b = host_->getBand(i);
        const float nx = freqToX(b.freqHz);
        const float ny = ssfilter::eqBandTypeHasGain(b.type) ? gainToY(b.gainDb) : gainToY(0.0);
        const float dist = std::sqrt(float((px - nx) * (px - nx) + (py - ny) * (py - ny)));
        if (dist < bestDist) { bestDist = dist; best = i; }
    }
    return best;
}

int EqGraphWidget::edgeHandleAt(double px, double py, int& edge) const
{
    constexpr double kHalfW = 6.0, kHalfH = 10.0; // a little larger than the drawn 6x14 handle
    int best = -1;
    double bestDist = kHalfW + 1.0;
    for (int i = 0; i < host_->getBandCount(); ++i)
    {
        const EqBand b = host_->getBand(i);
        if (!ssfilter::eqBandTypeHasBandwidth(b.type)) continue;
        const double hy = gainToY(b.gainDb);
        if (std::fabs(py - hy) > kHalfH) continue;
        const double d1 = std::fabs(px - freqToX(ssfilter::eq::bandShelfLowCornerHz(b)));
        const double d2 = std::fabs(px - freqToX(ssfilter::eq::bandShelfHighCornerHz(b)));
        if (d1 < bestDist) { bestDist = d1; best = i; edge = 1; }
        if (d2 < bestDist) { bestDist = d2; best = i; edge = 2; }
    }
    return best;
}

void EqGraphWidget::dragBandTo(int index, double px, double py)
{
    if (index < 0 || index >= host_->getBandCount()) return;
    const EqBand b = host_->getBand(index);
    const double freq = std::max(kFMin, std::min(kFMax, xToFreq(float(px))));

    if (ssfilter::eqBandTypeHasBandwidth(b.type) && index == dragBand_ && dragEdge_ != 0)
    {
        // A corner handle sets the bandwidth, symmetrically around the
        // fixed centre: the dragged corner follows the pointer and the other
        // mirrors it (bandwidth = 2 x the pointer's distance from the centre
        // in octaves; the host clamps it to kMin/kMaxBandwidthOct).
        host_->setBandBandwidth(index, 2.0 * std::fabs(std::log2(freq / std::max(1.0, b.freqHz))));
        return;
    }

    // The node itself: its centre -- for a Band Shelf too, whose bandwidth
    // simply travels with it.
    host_->setBandFreq(index, freq);
    if (ssfilter::eqBandTypeHasGain(b.type))
    {
        const double gain = std::max(-gainRangeDb_, std::min(gainRangeDb_, yToGain(float(py))));
        host_->setBandGain(index, gain);
    }
}

void EqGraphWidget::openContextMenu(int band, float px, float py)
{
    menuOpen_ = true;
    menuBand_ = band;

    // Built fresh every open: Delete/Duplicate, the static type list
    // unchanged, plus (only for a band whose type actually has a
    // steepness -- Low/High Pass) a "Slope" section beneath it, so the
    // menu never shows a control that wouldn't do anything for the band
    // it's open on.
    menuItems_.clear();
    // Delete/Duplicate first (the two band-count-changing actions).
    menuItems_.push_back({ "Delete Band", MenuEntry::Kind::Delete, EqBandType::Bell, 1, false });
    menuItems_.push_back({ "Duplicate Band", MenuEntry::Kind::Duplicate, EqBandType::Bell, 1, false });

    // "Create Harmonic Twin" next -- always offered, whether or not
    // `band` is already linked (chains a further harmonic onto an
    // existing group, see eq::createHarmonicTwin()'s own comment);
    // "Unlink Harmonic" only when it actually is. No "relink" -- asked-
    // about directly and declined ("No need for a relink option i
    // think").
    bool firstHarmonic = true;
    for (const auto& p : kHarmonicPresets)
    {
        menuItems_.push_back({ p.label, MenuEntry::Kind::HarmonicFactor, EqBandType::Bell, 1, firstHarmonic, p.factor });
        firstHarmonic = false;
    }
    menuItems_.push_back({ "Custom Factor...", MenuEntry::Kind::HarmonicCustom, EqBandType::Bell, 1, false, 1.0 });
    if (band >= 0 && band < host_->getBandCount() && host_->getBand(band).harmonicGroup != 0)
        menuItems_.push_back({ "Unlink Harmonic", MenuEntry::Kind::Unlink, EqBandType::Bell, 1, true, 1.0 });

    // The static type list unchanged, plus (only for a band whose type
    // actually has a steepness -- Low/High Pass) a "Slope" section
    // beneath it, so the menu never shows a control that wouldn't do
    // anything for the band it's open on. kMenuItems[0] ("Delete Band")
    // is skipped here -- already pushed above.
    for (int i = 1; i < kNumMenuItems; ++i)
        menuItems_.push_back({ kMenuItems[i].label, MenuEntry::Kind::Type, kMenuItems[i].type, 1, false });

    const EqBandType curType = (band >= 0 && band < host_->getBandCount()) ? host_->getBand(band).type : EqBandType::Bell;
    if (curType == EqBandType::LowPass || curType == EqBandType::HighPass)
    {
        static const struct { const char* label; int stages; } kSlopes[] = {
            { "12 dB/oct", 1 }, { "24 dB/oct", 2 }, { "36 dB/oct", 3 }, { "48 dB/oct", 4 },
        };
        bool first = true;
        for (const auto& s : kSlopes)
        {
            menuItems_.push_back({ s.label, MenuEntry::Kind::Slope, curType, s.stages, first });
            first = false;
        }
    }

    menuW_ = kMenuW;
    menuH_ = kMenuRowH * float(menuItems_.size());
    menuX_ = std::min(px, graphX_ + graphW_ - menuW_);
    menuY_ = std::min(py, graphY_ + graphH_ - menuH_);
    menuX_ = std::max(graphX_, menuX_);
    menuY_ = std::max(graphY_, menuY_);
    pressedMenuItem_ = -1;
}

int EqGraphWidget::menuItemAt(double px, double py) const
{
    if (!menuOpen_) return -1;
    if (px < menuX_ || px >= menuX_ + menuW_ || py < menuY_ || py >= menuY_ + menuH_) return -1;
    const int item = int((py - menuY_) / kMenuRowH);
    return (item >= 0 && item < int(menuItems_.size())) ? item : -1;
}

// ---- drawing --------------------------------------------------------------

void EqGraphWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    graphX_ = 36.0f;
    graphY_ = 6.0f;
    graphW_ = w - graphX_ - 30.0f; // room on the right for the spectrum's own scale labels, see below
    graphH_ = h - graphY_ - 20.0f;
    if (graphW_ <= 4.0f || graphH_ <= 4.0f) return;

    // Fetched once, up front -- both the spectrum section below (to decide
    // whether per-channel traces are meaningful right now) and the curve/
    // node sections further down need the current band list.
    const EqModel model = host_->getModel();

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

    // Frequency gridlines (log) with labels below the graph.
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

    // Gain gridlines with labels to the left; 0dB brighter (the meaningful
    // "no change" reference). Step adapts to the current range so a tight
    // +-3dB range doesn't try to cram 10 gridlines into a few pixels.
    //
    // Lines are placed at integer multiples of gridStep *counted from
    // zero* (k * gridStep), not by stepping outward from -gainRangeDb_ --
    // the old "start at -gainRangeDb_, step by gridStep" loop only ever
    // landed exactly on 0 when gainRangeDb_ happened to be a whole
    // multiple of gridStep, which was always true for the fixed
    // 3/6/12/30dB presets but stopped being true once the range became
    // continuously mouse-draggable: a dragged range like 7.3dB would step
    // ...,-5.3,-3.3,-1.3,0.7,... and never hit 0, so the brighter
    // "no change" line silently vanished. Counting from zero guarantees
    // g==0 for k==0 regardless of what gainRangeDb_ is.
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

    // Background spectrum -- pre-EQ (Input) and/or post-EQ (Output),
    // whichever spectrumMode_ selects, fed (already smoothed) dB values
    // from UIEq. A single trace is drawn as a filled silhouette down to
    // the bottom of the graph (as before); when both are showing, the
    // output trace stays filled (it's "what you're hearing now", the more
    // important of the two) while the input trace is stroke-only, so the
    // two don't muddy into an illegible double-fill where they overlap --
    // see this file's header comment.
    if (spectrumMode_ != ssfilter::SpectrumMode::Off)
    {
        const bool showIn = ssfilter::spectrumModeWantsInput(spectrumMode_);
        const bool showOut = ssfilter::spectrumModeWantsOutput(spectrumMode_);
        // Two colored traces (blue Left / red Right) instead of one
        // combined Output trace, whenever at least one active band is
        // actually Left/Right-only (Left in blue, Right in red). Reverts to the single combined trace the instant
        // every band goes back to Both, with no separate toggle needed --
        // whether the two channels are worth showing separately is
        // entirely a function of the current band list.
        const bool perChannel = showOut && ssfilter::eq::hasPerChannelBands(model);

        // Autoscale to the union of whichever trace(s) are showing this
        // frame, headroom added, rounded to a clean 6dB step, clamped to a
        // sane outer envelope -- see spectrumToY()'s comment for why this
        // (not a fixed range, not a manual gain control) is the actual
        // fix, and why "union" rather than two independent ranges.
        if (spectrumAutoscale_)
        {
            // Autoscale to the union of whichever trace(s) are showing this
            // frame, headroom added, rounded to a clean 6dB step, clamped
            // to a sane outer envelope -- see spectrumToY()'s comment for
            // why this (not a fixed range) is the actual fix for the
            // spectrum's own dynamic range being wildly different from the
            // EQ curve's. The *target* range is still recomputed fresh
            // every frame, exactly as before, but the range this widget
            // actually draws (spectrumRangeLo_/Hi_) is smoothed toward it
            // rather than snapped -- program material's own min/max moves
            // constantly even after the trace itself is smoothed
            // (UIEq::uiIdle()'s own spectrumSmoothMs_), and snapping the
            // *axis* to match every frame was visually disturbing
            // (gridlines and the curve's apparent position
            // both drifting on every redraw even when the actual level
            // held steady). A slow one-pole toward the target turns that
            // into a gentle drift instead, at the small cost of the axis
            // trailing a fast real change for a moment.
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
            if (!(lo < hi)) { lo = kFixedSpectrumRangeLoDb; hi = kFixedSpectrumRangeHiDb; } // degenerate (e.g. true silence) -- fall back to something sane
            lo -= 6.0; hi += 6.0;
            lo = std::floor(lo / 6.0) * 6.0;
            hi = std::ceil(hi / 6.0) * 6.0;
            lo = std::max(kSpectrumFloorDb, lo);
            hi = std::min(kSpectrumCeilDb, hi);
            if (hi - lo < 12.0) hi = std::min(kSpectrumCeilDb, lo + 12.0);
            if (hi - lo < 12.0) lo = std::max(kSpectrumFloorDb, hi - 12.0);

            // Assumes onNanoDisplay() is called at roughly the same ~16ms
            // cadence UIEq::uiIdle() already assumes for its own trace
            // smoothing (this widget is repainted from that same idle tick
            // whenever spectrumMode_ is on) -- a fixed 600ms time constant,
            // slow enough to read as a drift rather than a jump, fast
            // enough to still track a real, sustained level change within
            // a second or so.
            constexpr double kAssumedFrameMs = 16.0;
            constexpr double kRangeSmoothMs = 600.0;
            const double coeff = std::exp(-kAssumedFrameMs / kRangeSmoothMs);
            spectrumRangeLo_ = coeff * spectrumRangeLo_ + (1.0 - coeff) * lo;
            spectrumRangeHi_ = coeff * spectrumRangeHi_ + (1.0 - coeff) * hi;
        }
        // else: autoscale just turned (or already was) off -- leave
        // spectrumRangeLo_/Hi_ exactly where they are rather than
        // resetting to the fixed default. It already *starts* at the
        // fixed default (see the member's own initializer) and only ever
        // moves while autoscale is on, so turning autoscale off simply
        // *freezes* whatever range was last showing -- including the
        // fixed default itself, if autoscale was never turned on at all --
        // instead of snapping back to it out from under whatever the user
        // was looking at.

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

        // Warm orange for "before" (pre-EQ), the existing cool blue for
        // "after" (post-EQ, unchanged from before this feature existed) --
        // deliberately different hue families so the two read apart at a
        // glance, not just by alpha/fill. The per-channel Right trace
        // reuses that same blue's warm-red counterpart: Left is literally the
        // same color the single combined Output trace already was, not a
        // new one, since a Both-only model's Left channel *is* that same
        // trace.
        const Color kInStroke(235, 165, 95, 0.65f);
        const Color kInFill(220, 140, 70, 0.16f);
        const Color kOutStroke(110, 180, 220, 0.5f);
        const Color kOutFill(80, 150, 200, 0.22f);
        const Color kOutRStroke(220, 100, 90, 0.5f);
        const Color kOutRFill(200, 75, 65, 0.22f);

        // Input drawn first (so Output, when both show, paints on top and
        // stays the visually dominant curve); filled only when it's the
        // sole trace on screen (see the doFill argument -- "!showOut").
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

        // Small legend, top-left of the graph, naming whichever trace(s)
        // are actually on screen -- helpful the moment more than one
        // color can appear here, and harmless clutter otherwise.
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

        // The spectrum's own scale, labeled on the *right* edge (the EQ
        // curve's gain axis already owns the left edge, and the two axes
        // mean completely different things -- absolute level, autoscaled
        // fresh every frame, vs. the EQ's fixed relative-gain zoom -- so
        // they're kept visually distinct with their own neutral color
        // rather than gridlines that would read as if they belonged to
        // the EQ curve). Neutral gray, not tied to either trace's own
        // color, since this one axis is now potentially shared by both.
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

    // Each band's own isolated response, thin and dim -- drawn first so
    // the bright combined curve and the nodes sit on top of them.
    for (int i = 0; i < bandCount; ++i)
    {
        const EqBand b = host_->getBand(i);
        if (!b.enabled) continue;
        Color col = bandColor(i);
        col.alpha = 0.55f;
        // A crossfeed band's own individual curve rides on the model's
        // own crossFeedGainLtoRDb/RtoLDb transit baseline -- see
        // evaluateBandResponse()'s own comment on `extraFlatGainDb`.
        const double extraFlatGainDb = b.crossFeed == EqCrossFeed::LeftToRight ? model.crossFeedGainLtoRDb
                                      : b.crossFeed == EqCrossFeed::RightToLeft ? model.crossFeedGainRtoLDb
                                      : 0.0;
        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = ssfilter::eq::evaluateBandResponse(b, sampleRate_, f, extraFlatGainDb);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            if (p == 0) moveTo(x, y); else lineTo(x, y);
        }
        strokeColor(col);
        strokeWidth(1.2f);
        stroke();
    }

    // The combined (what you actually hear) curve -- the product of every
    // *active* band's response (respects enabled/solo, see EqModel.hpp's
    // activeBandIndices()), computed directly rather than via the
    // dense-matrix TransferFunction, so this is cheap enough to redraw on
    // every single drag event. See file comment.
    //
    // A single white curve as long as every active band is Both -- the
    // ordinary case, unchanged from before per-channel routing existed.
    // Once any band is Left/Right-only, one curve would silently
    // misrepresent whichever channel it doesn't reach, so two genuinely
    // different curves are drawn instead (evaluateChannelResponse(),
    // O(1) per point like evaluateCombinedResponse() itself), in the same
    // blue/red this project now uses everywhere a channel needs
    // distinguishing (the spectrum traces below, the node rings above).
    const bool perChannel = ssfilter::eq::hasPerChannelBands(model);
    auto drawCurve = [&](EqChannel listenerChannel, Color color, bool wide)
    {
        beginPath();
        for (int p = 0; p <= kCurvePoints; ++p)
        {
            const double t = double(p) / double(kCurvePoints);
            const double f = kFMin * std::pow(kFMax / kFMin, t);
            const std::complex<double> resp = perChannel
                ? ssfilter::eq::evaluateChannelResponse(model, sampleRate_, f, listenerChannel)
                : ssfilter::eq::evaluateCombinedResponse(model, sampleRate_, f);
            const double db = 20.0 * std::log10(std::max(std::abs(resp), 1e-9));
            const float x = graphX_ + float(t) * graphW_;
            const float y = gainToY(std::max(-gainRangeDb_, std::min(gainRangeDb_, db)));
            if (p == 0) moveTo(x, y); else lineTo(x, y);
        }
        strokeColor(color);
        strokeWidth(wide ? 2.4f : 2.0f);
        stroke();
    };
    if (perChannel)
    {
        drawCurve(EqChannel::Left, Color(90, 150, 230), false);
        drawCurve(EqChannel::Right, Color(230, 110, 100), false);
    }
    else
    {
        drawCurve(EqChannel::Both, Color(230, 232, 236), true);
    }

    // Node markers, drawn last (before the menu) so they sit on top of
    // every curve.
    for (int i = 0; i < bandCount; ++i)
    {
        const EqBand b = host_->getBand(i);
        const float nx = freqToX(b.freqHz);
        const float ny = ssfilter::eqBandTypeHasGain(b.type) ? gainToY(b.gainDb) : gainToY(0.0);
        const bool selected = (i == selectedBand_);
        const bool hovered = (i == hoverBand_);
        const bool auditionHeld = (i == auditionHoldBand_);
        const Color col = b.enabled ? bandColor(i) : Color(110, 112, 116);

        // Band Shelf: a line across the plateau, level with the node, with
        // a draggable handle at each corner (see edgeHandleAt()).
        if (ssfilter::eqBandTypeHasBandwidth(b.type))
        {
            const float x1 = freqToX(ssfilter::eq::bandShelfLowCornerHz(b)), x2 = freqToX(ssfilter::eq::bandShelfHighCornerHz(b));
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

        if (auditionHeld)
        {
            // Distinct gold ring for the momentary right-click-hold
            // audition -- deliberately not reusing the white selection
            // ring so it reads as "temporarily auditioned", not "selected".
            beginPath();
            circle(nx, ny, 10.5f);
            strokeColor(Color(235, 195, 60, 0.9f));
            strokeWidth(2.0f);
            stroke();
        }
        else if (selected || hovered)
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

        // Channel-assignment ring -- always visible when a band isn't
        // Both (not gated on selected/hovered/audition, unlike the rings
        // above), so which band is assigned to which channel is visible
        // at a glance across the whole graph, not just the one currently
        // interacted with. A larger
        // radius than every ring above so it never visually competes with
        // them -- all of selected/hovered/audition-held/channel can be
        // true on the same node at once, each its own concentric ring.
        // Blue for Left, red for Right -- the same colors this project
        // now uses for the graph's own per-channel spectrum traces, so
        // the two reinforce each other at a glance.
        if (b.channel != EqChannel::Both)
        {
            beginPath();
            circle(nx, ny, 13.0f);
            strokeColor(b.channel == EqChannel::Left ? Color(90, 140, 230, 0.85f) : Color(230, 100, 90, 0.85f));
            strokeWidth(1.8f);
            stroke();
        }

        // Crossfeed ring -- same idea as the Channel-assignment ring just
        // above (always visible, its own dedicated radius so both can
        // show on the same node at once), purple to read as a genuinely
        // different kind of routing from Channel's own blue/red. Purely
        // additive and gated on the DATA (b.crossFeed != None) rather
        // than any interface query -- a band coming from a host that
        // never exposes crossfeed at all (UIEqMono, ../../reverb's own
        // UIReverb) can never actually have a non-None value here, so
        // this never fires for them; see EqBand::crossFeed's own comment.
        if (b.crossFeed != EqCrossFeed::None)
        {
            beginPath();
            circle(nx, ny, 16.0f);
            strokeColor(Color(170, 100, 220, 0.85f));
            strokeWidth(1.8f);
            stroke();
        }

        // Band number, small, inside the node -- helps correlate a node
        // with its row in BandListWidget below.
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

    // Freq/gain/Q readout above whichever node is being dragged *or*
    // merely hovered -- covers the moment a band is first placed (a
    // click-to-add starts a drag immediately, see onMouse()), any later
    // reposition, and now also just pointing at a node without clicking,
    // so the exact numbers are discoverable without needing to grab it
    // first. Dragging wins if somehow both are set at once (dragging a
    // node keeps it "hovered" too).
    const int readoutBand = dragBand_ >= 0 ? dragBand_ : hoverBand_;
    if (readoutBand >= 0 && readoutBand < bandCount)
    {
        const EqBand b = host_->getBand(readoutBand);
        const float nx = freqToX(b.freqHz);
        const float ny = ssfilter::eqBandTypeHasGain(b.type) ? gainToY(b.gainDb) : gainToY(0.0);

        char buf[80];
        std::string freqStr = formatFreqLabel(b.freqHz);
        if (ssfilter::eqBandTypeHasBandwidth(b.type))
        {
            char bwBuf[24];
            std::snprintf(bwBuf, sizeof(bwBuf), "   BW %.2f oct", b.bandwidthOct);
            freqStr += bwBuf;
        }
        if (ssfilter::eqBandTypeHasGain(b.type))
            std::snprintf(buf, sizeof(buf), "%s   %+.1fdB   Q %.2f", freqStr.c_str(), b.gainDb, b.q);
        else
            std::snprintf(buf, sizeof(buf), "%s   Q %.2f", freqStr.c_str(), b.q);

        fontSize(11.5f);
        const float textW = float(std::strlen(buf)) * 6.4f + 14.0f;
        const float textH = 18.0f;
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
        fillColor(Color(235, 237, 240));
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(lx + textW * 0.5f, ly + textH * 0.5f + 0.5f, buf, nullptr);
    }

    // "±Ndb" gain-range label, top-right corner of the graph -- click to
    // cycle through the preset ranges.
    char rangeBuf[16];
    std::snprintf(rangeBuf, sizeof(rangeBuf), "\xC2\xB1%.0fdB", gainRangeDb_); // UTF-8 "±"
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
    // label -- off (Fixed Range) by default, see setSpectrumAutoscale()'s
    // comment.
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

    // Crossfeed transit gain knobs -- stacked below the Autoscale label,
    // one per direction, each only drawn (and only hit-testable, see
    // onMouse()/onScroll()) while some band in the model actually routes
    // that way. See CrossFeedGainControl's own
    // comment (.hpp) for the interaction model.
    {
        bool anyLtoR = false, anyRtoL = false;
        for (int i = 0; i < bandCount; ++i)
        {
            const ssfilter::EqCrossFeed cf = host_->getBand(i).crossFeed;
            if (cf == EqCrossFeed::LeftToRight) anyLtoR = true;
            else if (cf == EqCrossFeed::RightToLeft) anyRtoL = true;
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
        drawXfeedControl(xfeedLtoR_, stackY, "L\xE2\x86\x92""R", model.crossFeedGainLtoRDb); // UTF-8 rightwards arrow
        if (xfeedLtoR_.visible) stackY += xfeedLtoR_.h + 3.0f;
        drawXfeedControl(xfeedRtoL_, stackY, "R\xE2\x86\x92""L", model.crossFeedGainRtoLDb);
    }

    // Right-click context menu, drawn last so it sits on top of everything.
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
            // The menu's rows are replaced entirely by a live text-input
            // readout while typing a custom harmonic factor -- see
            // customFactorEditing_'s own declaration comment.
            std::string liveText;
            host_->isEditingHarmonicFactor(menuBand_, liveText); // false would mean UIEq already ended the edit; endHarmonicFactorEdit() closes this menu in that case, so menuOpen_ would already be false here
            fillColor(Color(180, 184, 192));
            fontSize(10.5f);
            textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
            text(menuX_ + 8.0f, menuY_ + kMenuRowH * 0.5f, "Harmonic Factor:", nullptr);

            const std::string display = liveText + "|"; // trailing caret, same "visibly an active input" convention this project's other numeric fields use
            fillColor(Color(235, 195, 60)); // amber, matching this project's shared editing-highlight color
            fontSize(13.0f);
            text(menuX_ + 8.0f, menuY_ + kMenuRowH * 1.5f, display.c_str(), nullptr);

            fillColor(Color(140, 144, 152));
            fontSize(9.0f);
            text(menuX_ + 8.0f, menuY_ + kMenuRowH * 2.35f, "Enter to create, Esc to cancel", nullptr);
        }
        else
        {
            const EqBand currentBand = (menuBand_ >= 0 && menuBand_ < bandCount) ? host_->getBand(menuBand_) : EqBand{};
            for (int i = 0; i < int(menuItems_.size()); ++i)
            {
                const MenuEntry& e = menuItems_[size_t(i)];
                const float iy = menuY_ + float(i) * kMenuRowH;
                const bool isCurrent = (e.kind == MenuEntry::Kind::Type && e.type == currentBand.type) ||
                                        (e.kind == MenuEntry::Kind::Slope && e.slopeStages == currentBand.slopeStages);
                if (i == pressedMenuItem_ || isCurrent)
                {
                    beginPath();
                    rect(menuX_ + 1.0f, iy, menuW_ - 2.0f, kMenuRowH);
                    fillColor(isCurrent ? Color(55, 70, 60) : Color(50, 53, 60));
                    fill();
                }
                // Above the type list (below the "Create Harmonic Twin"
                // section) and above the Slope section -- found by kind
                // transition rather than a hardcoded row index, since
                // that section's own row count varies (an extra "Unlink
                // Harmonic" row only when the band is already linked).
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

bool EqGraphWidget::onMouse(const MouseEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    // While typing a custom harmonic factor, this widget's only job is
    // to keep showing the live buffer (onNanoDisplay(), fed by UIEq's
    // isEditingHarmonicFactor()) -- every mouse gesture is suppressed
    // until UIEq's own commit-on-any-press logic (or Enter/Escape) ends
    // the edit and calls endHarmonicFactorEdit(), which closes this menu.
    // Swallowing every button here (not just left) also blocks a right-
    // click from opening a second, unrelated context menu mid-edit.
    if (customFactorEditing_) return true;

    // The context menu, if open, owns every left click until it's
    // resolved (an item picked, or a click outside dismisses it) --
    // checked before anything else, regardless of button.
    if (menuOpen_ && ev.button == kMouseButtonLeft)
    {
        if (ev.press)
        {
            pressedMenuItem_ = menuItemAt(x, y);
            if (pressedMenuItem_ < 0) { closeContextMenu(); repaint(); } // click outside: dismiss only, no other action this click
            return true;
        }
        const int released = menuItemAt(x, y);
        const int band = menuBand_;
        if (released >= 0 && released == pressedMenuItem_ && band >= 0 && released < int(menuItems_.size()))
        {
            const MenuEntry& e = menuItems_[size_t(released)];
            if (e.kind == MenuEntry::Kind::Delete)
            {
                // host_->removeBand() (UIEq::removeBand()) already
                // corrects selectedBand_ for the shift this causes -- see
                // BandListWidget.cpp's identical comment on its own
                // now-removed twin of this line for why re-deriving it
                // here afterward, from the pre-deletion `band` value,
                // would be wrong rather than merely redundant.
                host_->removeBand(band);
            }
            else if (e.kind == MenuEntry::Kind::Duplicate)
            {
                host_->duplicateBand(band);
            }
            else if (e.kind == MenuEntry::Kind::Type)
            {
                host_->setBandType(band, e.type);
            }
            else if (e.kind == MenuEntry::Kind::Slope)
            {
                host_->setBandSlopeStages(band, e.slopeStages);
            }
            else if (e.kind == MenuEntry::Kind::HarmonicFactor)
            {
                host_->createHarmonicTwin(band, e.harmonicFactor);
            }
            else if (e.kind == MenuEntry::Kind::Unlink)
            {
                host_->unlinkHarmonic(band);
            }
            else // Kind::HarmonicCustom -- switches into text-input mode instead of closing the menu, see below
            {
                customFactorEditing_ = true;
                menuH_ = kMenuRowH * 3.0f; // label + live value + hint, see onNanoDisplay()'s customFactorEditing_ branch
                host_->beginHarmonicFactorEdit(band); // UIEq now owns the actual text-edit state; see EqEditorHost.hpp
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
                // Never crossed the drag dead zone: a plain click. Open
                // the context menu now (deferred from press, since until
                // this point we didn't yet know it wouldn't turn into a
                // drag) -- no audition ever started for this press.
                rightPressPending_ = false;
                openContextMenu(rightPressBand_, float(rightPressX_), float(rightPressY_));
                rightPressBand_ = -1;
                repaint();
                return true;
            }
            // Release: end whatever momentary audition-hold this press
            // committed to by dragging past the dead zone (a no-op if the
            // press landed on empty space and never started one) -- see
            // this file's own header comment and EqEditorHost.hpp's
            // beginBandAudition()/endBandAudition() for the full
            // reasoning. Swallows the matching release either way, same
            // as before this gesture was added.
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
            // Don't decide menu-vs-audition yet -- that depends on whether
            // this press turns into a drag, which onMotion() resolves once
            // the pointer crosses the dead zone (or the release handling
            // above resolves it as a plain click if it never does).
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
        // Gain-range label -- press starts a potential drag (see
        // onMotion()); whether it actually cycles the presets instead is
        // resolved on release, once we know if it ever left the dead zone.
        if (x >= rangeLabelX_ && x < rangeLabelX_ + rangeLabelW_ &&
            y >= rangeLabelY_ && y < rangeLabelY_ + rangeLabelH_)
        {
            rangeLabelPressed_ = true;
            rangeLabelDragging_ = false;
            rangeLabelDragStartY_ = y;
            rangeLabelDragStartValue_ = gainRangeDb_;
            return true;
        }

        // Autoscale/Fixed Range toggle.
        if (x >= autoscaleLabelX_ && x < autoscaleLabelX_ + autoscaleLabelW_ &&
            y >= autoscaleLabelY_ && y < autoscaleLabelY_ + autoscaleLabelH_)
        {
            spectrumAutoscale_ = !spectrumAutoscale_;
            if (onSpectrumAutoscaleChanged_) onSpectrumAutoscaleChanged_(spectrumAutoscale_);
            repaint();
            return true;
        }

        // Crossfeed transit gain knobs -- press starts a continuous
        // click-and-drag fader sweep, resolved entirely in onMotion();
        // unlike the range label above, a plain click (never dragged)
        // does nothing on release -- there's no small preset set to
        // cycle through for a continuous send-level control.
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
                // See the context-menu Delete case's identical comment
                // above -- host_->removeBand() already corrects
                // selectedBand_ itself.
                host_->removeBand(hit);
                lastClickBand_ = -1;
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

        // A Band Shelf corner handle -- drag that corner (see
        // dragBandTo()). Not a double-click-to-delete target: only the
        // node itself is.
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

        // Empty space: add a new band right here and start dragging it
        // immediately so the initial click position doesn't need to be
        // pixel-perfect.
        const double freq = xToFreq(float(x));
        const double gain = yToGain(float(y));
        host_->addBand(freq, gain);
        dragBand_ = host_->getBandCount() - 1;
        dragEdge_ = 0;
        selectedBand_ = dragBand_;
        lastClickBand_ = -1; // an add is never part of a double-click-to-remove pair
        if (onSelectionChanged_) onSelectionChanged_(dragBand_);
        repaint();
        return true;
    }

    // Release.
    if (rangeLabelPressed_)
    {
        const bool wasDragging = rangeLabelDragging_;
        rangeLabelPressed_ = false;
        rangeLabelDragging_ = false;
        if (!wasDragging)
        {
            // Never left the dead zone -- a plain click, cycle presets
            // exactly like before this drag gesture existed.
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

bool EqGraphWidget::onMotion(const MotionEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (rangeLabelPressed_)
    {
        const double dy = rangeLabelDragStartY_ - y; // up = positive = larger range
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
        const double dy = xfeedLtoR_.dragStartY - y; // up = positive = louder
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
            // Crossed the dead zone: this is a drag, not a click -- commit
            // to audition-while-dragging (reusing the same dragBand_
            // mechanism the left-click drag uses below) and suppress the
            // context menu that would otherwise have opened on release.
            rightPressPending_ = false;
            auditionHoldBand_ = rightPressBand_;
            dragBand_ = rightPressBand_;
            dragEdge_ = 0;
            host_->beginBandAudition(rightPressBand_);
            rightPressBand_ = -1;
            repaint();
            // Fall through to the dragBand_ handling below so this same
            // motion event already moves the band, instead of waiting for
            // the next one.
        }
        else
        {
            return true; // still deciding; swallow motion until resolved
        }
    }

    if (dragBand_ >= 0)
    {
        // Clamped to the graph's own bounds -- see NonlinearPanelWidget's
        // identical note about pointer-capture-across-widget-bounds not
        // being verified in this environment.
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

bool EqGraphWidget::onScroll(const ScrollEvent& ev)
{
    // See onMouse()'s identical guard's own comment.
    if (customFactorEditing_) return true;

    const double x = ev.pos.getX(), y = ev.pos.getY();

    // Crossfeed transit gain knobs -- fine scroll-to-nudge, same 0.5dB
    // step convention as every other numeric field/label in this project
    // (see BandListWidget's own scroll-to-nudge fields).
    auto scrollXfeed = [&](CrossFeedGainControl& c, void (ssgui::IEqEditorHost::*setter)(double), double current)
    {
        if (!c.visible || x < c.x || x >= c.x + c.w || y < c.y || y >= c.y + c.h) return false;
        const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
        if (dir != 0.0)
        {
            const double v = std::max(kXfeedGainMinDb, std::min(kXfeedGainMaxDb, current + dir * 0.5));
            (host_->*setter)(v);
            repaint();
        }
        return true;
    };
    if (scrollXfeed(xfeedLtoR_, &ssgui::IEqEditorHost::setCrossFeedGainLtoRDb, host_->getModel().crossFeedGainLtoRDb)) return true;
    if (scrollXfeed(xfeedRtoL_, &ssgui::IEqEditorHost::setCrossFeedGainRtoLDb, host_->getModel().crossFeedGainRtoLDb)) return true;

    int hit = nodeAt(x, y);
    if (hit < 0)
    {
        int edge = 0;
        hit = edgeHandleAt(x, y, edge); // a Band Shelf's corner handles scroll its bandwidth too
    }
    if (hit < 0) return false;

    const EqBand b = host_->getBand(hit);
    const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
    if (dir == 0.0) return true;
    if (ssfilter::eqBandTypeHasBandwidth(b.type))
    {
        // A Band Shelf scrolls its bandwidth, not Q: the width is what you
        // shape it by on the graph. Up = wider, same multiplicative step
        // and Alt = 10x finer as Q below. Q stays in the band list.
        constexpr double kBwFactor = 1.08;
        const double bwFactor = (ev.mod & kModifierAlt) ? 1.0 + (kBwFactor - 1.0) / 10.0 : kBwFactor;
        host_->setBandBandwidth(hit, dir > 0.0 ? b.bandwidthOct * bwFactor : b.bandwidthOct / bwFactor);
        repaint();
        return true;
    }
    // Multiplicative step so Q feels proportionally the same to adjust
    // whether it's currently 0.3 or 20. Alt+scroll = 10x finer (the
    // *percentage* change per notch shrinks by 10x) -- see
    // ParamStripWidget::onScroll()'s identical comment for the full
    // reasoning, applied project-wide.
    constexpr double kFactor = 1.08;
    const double factor = (ev.mod & kModifierAlt) ? 1.0 + (kFactor - 1.0) / 10.0 : kFactor;
    const double newQ = std::max(0.05, std::min(100.0, b.q * (dir > 0.0 ? factor : 1.0 / factor)));
    host_->setBandQ(hit, newQ);
    repaint();
    return true;
}

END_NAMESPACE_DGL
