// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "BodePlotWidget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

namespace {
constexpr double kFMin = 20.0;
constexpr double kFMaxCap = 20000.0;
constexpr int kNumSweepPoints = 220;
constexpr double kPhaseDegMin = -180.0, kPhaseDegMax = 180.0;

double freqToX(double freq, double fMin, double fMax, float x0, float w, bool logScale)
{
    const double t = logScale
        ? (std::log10(freq) - std::log10(fMin)) / (std::log10(fMax) - std::log10(fMin))
        : (freq - fMin) / (fMax - fMin);
    return x0 + float(std::min(1.0, std::max(0.0, t))) * w;
}

float valueToY(double v, double vMin, double vMax, float y0, float h)
{
    const double t = (v - vMin) / (vMax - vMin);
    const double clamped = std::min(1.0, std::max(0.0, t));
    return y0 + float(1.0 - clamped) * h;
}

double wrapDegrees(double rad)
{
    double deg = rad * 180.0 / M_PI;
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}

void formatFreqLabel(char* buf, size_t bufSize, double f)
{
    if (f >= 1000.0) std::snprintf(buf, bufSize, "%gk", f / 1000.0);
    else             std::snprintf(buf, bufSize, "%g", f);
}
} // namespace

BodePlotWidget::BodePlotWidget(Widget* parent) : NanoSubWidget(parent)
{
    // Each NanoSubWidget owns its own NanoVG context, so the bundled font
    // must be (re-)loaded here too -- loadSharedResources() is idempotent.
    loadSharedResources();
}

std::string BodePlotWidget::getVisibilityCode() const
{
    // Order LL, RL, LR, RR: (out,in) = (0,0), (0,1), (1,0), (1,1).
    std::string s;
    s += visible_[0][0] ? '1' : '0';
    s += visible_[0][1] ? '1' : '0';
    s += visible_[1][0] ? '1' : '0';
    s += visible_[1][1] ? '1' : '0';
    return s;
}

void BodePlotWidget::setVisibilityCode(const std::string& code)
{
    if (code.size() != 4) return; // malformed/foreign value -- keep current visibility
    visible_[0][0] = code[0] == '1';
    visible_[0][1] = code[1] == '1';
    visible_[1][0] = code[2] == '1';
    visible_[1][1] = code[3] == '1';
    recomputeAxisScale();
    repaint();
}

void BodePlotWidget::recompute(const ssfilter::StateSpaceMatrices& model, double sampleRate)
{
    lastModel_ = model;
    sampleRate_ = sampleRate > 0.0 ? sampleRate : 44100.0;
    resweep();
}

void BodePlotWidget::resweep()
{
    const double fMax = std::min(kFMaxCap, sampleRate_ * 0.5 - 1.0);
    for (int o = 0; o < 2; ++o)
        for (int i = 0; i < 2; ++i)
            ssfilter::TransferFunction::sweep(lastModel_, sampleRate_, o, i, kFMin, fMax,
                                               kNumSweepPoints, curves_[o][i], logScale_);
    hasData_ = true;
    recomputeAxisScale();
    repaint();
}

void BodePlotWidget::recomputeAxisScale()
{
    // Autoscale the magnitude axis from whichever curves are currently
    // *visible* (see the legend-click toggle in onMouse()) instead of a
    // fixed +-Ndb range or always all four -- with all four channel pairs
    // sharing one axis, a single loud cross-channel term can otherwise
    // swamp the scale and flatten out the curve(s) actually being looked
    // at. Padded with headroom and rounded to a clean multiple of 6 dB so
    // gridlines land on tidy numbers.
    double lo = 1e9, hi = -1e9;
    for (int o = 0; o < 2; ++o)
        for (int i = 0; i < 2; ++i)
        {
            if (!visible_[o][i]) continue;
            for (const auto& p : curves_[o][i]) { lo = std::min(lo, p.magnitudeDb); hi = std::max(hi, p.magnitudeDb); }
        }
    if (lo > hi) { lo = -60.0; hi = 24.0; } // no data yet, or everything hidden -- sane fallback
    lo -= 3.0;
    hi += 3.0;
    dbAxisMin_ = std::floor(lo / 6.0) * 6.0;
    dbAxisMax_ = std::ceil(hi / 6.0) * 6.0;
    if (dbAxisMax_ - dbAxisMin_ < 12.0) dbAxisMax_ = dbAxisMin_ + 12.0; // never a degenerately flat axis
    dbAxisMin_ = std::max(dbAxisMin_, -240.0); // guard against a pathological sweep blowing the range up
    dbAxisMax_ = std::min(dbAxisMax_, 240.0);
}

bool BodePlotWidget::onMouse(const MouseEvent& ev)
{
    if (ev.button != 1 || !ev.press) return false;

    const double x = ev.pos.getX();
    const double y = ev.pos.getY();

    const bool onToggle = x >= toggleX_ && x < toggleX_ + toggleW_ &&
                           y >= toggleY_ && y < toggleY_ + toggleH_;
    if (onToggle)
    {
        logScale_ = !logScale_;
        if (hasData_) resweep();
        else repaint();
        return true;
    }

    for (int o = 0; o < 2; ++o)
    {
        for (int i = 0; i < 2; ++i)
        {
            const bool onSwatch = x >= legendX_[o][i] && x < legendX_[o][i] + legendW_[o][i] &&
                                   y >= legendY_[o][i] && y < legendY_[o][i] + legendH_[o][i];
            if (!onSwatch) continue;
            visible_[o][i] = !visible_[o][i];
            recomputeAxisScale();
            repaint();
            if (onVisibilityChanged_) onVisibilityChanged_();
            return true;
        }
    }

    return false;
}

void BodePlotWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(22, 24, 29));
    fill();

    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(12.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    fillColor(Color(210, 214, 222));
    text(6.0f, 9.0f, "Frequency Response (2x2 MIMO)", nullptr);

    // LOG/LIN toggle, top-right corner.
    {
        const char* label = logScale_ ? "LOG" : "LIN";
        toggleW_ = 34.0f;
        toggleH_ = 16.0f;
        toggleX_ = w - toggleW_ - 6.0f;
        toggleY_ = 2.0f;
        beginPath();
        roundedRect(toggleX_, toggleY_, toggleW_, toggleH_, 3.0f);
        fillColor(Color(48, 52, 60));
        fill();
        strokeColor(Color(90, 94, 102));
        strokeWidth(1.0f);
        stroke();
        fontSize(10.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        fillColor(Color(210, 214, 222));
        text(toggleX_ + toggleW_ * 0.5f, toggleY_ + toggleH_ * 0.5f + 1.0f, label, nullptr);
    }

    constexpr float kMargin = 42.0f;
    const float plotX = kMargin;
    const float plotW = w - kMargin - 10.0f;
    if (plotW <= 10.0f) return;

    // Legend gets its own row, clearly below the title (both used to sit at
    // nearly the same y, so the legend visually collided with the title
    // text) and above the plot itself.
    constexpr float kLegendY = 27.0f;
    const float magY = 38.0f;
    const float magH = h * 0.60f;
    const float phaseY = magY + magH + 26.0f;
    const float phaseH = h - phaseY - 10.0f;

    const double fMax = std::min(kFMaxCap, sampleRate_ * 0.5 - 1.0);

    // ---- Magnitude panel ----
    beginPath();
    rect(plotX, magY, plotW, magH);
    fillColor(Color(15, 16, 20));
    fill();

    // dB gridlines. Step adapts to the autoscaled range (see resweep())
    // so a narrow EQ bump and a 100+ dB resonant peak both get a sane
    // number of gridlines instead of one fixed spacing for both.
    const double dbRange = dbAxisMax_ - dbAxisMin_;
    const double dbStep = dbRange > 96.0 ? 24.0 : (dbRange > 48.0 ? 12.0 : 6.0);
    for (double db = dbAxisMin_; db <= dbAxisMax_ + 0.001; db += dbStep)
    {
        const float y = valueToY(db, dbAxisMin_, dbAxisMax_, magY, magH);
        beginPath();
        moveTo(plotX, y);
        lineTo(plotX + plotW, y);
        strokeColor(std::fabs(db) < 0.001 ? Color(90, 94, 102) : Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%+.0f", db);
        fillColor(Color(120, 124, 132));
        fontSize(10.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        text(plotX - 4.0f, y, buf, nullptr);
    }

    // Frequency gridlines: decades when log-scaled, evenly spaced ticks
    // when linear (a decade series would bunch up uselessly at the low end
    // of a linear axis).
    if (logScale_)
    {
        for (double f = 100.0; f <= fMax; f *= 10.0)
        {
            const float x = float(freqToX(f, kFMin, fMax, plotX, plotW, true));
            beginPath();
            moveTo(x, magY);
            lineTo(x, magY + magH);
            strokeColor(Color(40, 43, 50));
            strokeWidth(1.0f);
            stroke();
            char buf[16];
            formatFreqLabel(buf, sizeof(buf), f);
            fillColor(Color(120, 124, 132));
            fontSize(10.0f);
            textAlign(ALIGN_CENTER | ALIGN_TOP);
            text(x, magY + magH + 2.0f, buf, nullptr);
        }
    }
    else
    {
        constexpr int kTicks = 5;
        for (int i = 0; i <= kTicks; ++i)
        {
            const double f = kFMin + (fMax - kFMin) * (double(i) / double(kTicks));
            const float x = float(freqToX(f, kFMin, fMax, plotX, plotW, false));
            beginPath();
            moveTo(x, magY);
            lineTo(x, magY + magH);
            strokeColor(Color(40, 43, 50));
            strokeWidth(1.0f);
            stroke();
            char buf[16];
            formatFreqLabel(buf, sizeof(buf), f);
            fillColor(Color(120, 124, 132));
            fontSize(10.0f);
            textAlign((i == kTicks ? ALIGN_RIGHT : (i == 0 ? ALIGN_LEFT : ALIGN_CENTER)) | ALIGN_TOP);
            text(x, magY + magH + 2.0f, buf, nullptr);
        }
    }

    if (hasData_)
    {
        auto drawMagCurve = [&](int o, int i, Color color, float width) {
            if (!visible_[o][i]) return;
            const auto& pts = curves_[o][i];
            if (pts.size() < 2) return;
            beginPath();
            for (size_t k = 0; k < pts.size(); ++k)
            {
                const float x = float(freqToX(pts[k].freqHz, kFMin, fMax, plotX, plotW, logScale_));
                const float y = valueToY(pts[k].magnitudeDb, dbAxisMin_, dbAxisMax_, magY, magH);
                if (k == 0) moveTo(x, y); else lineTo(x, y);
            }
            strokeColor(color);
            strokeWidth(width);
            stroke();
        };
        // Cross-channel terms first (thin, dim) so the direct terms draw on top.
        drawMagCurve(1, 0, Color(160, 110, 200, 0.55f), 1.2f); // L in -> R out
        drawMagCurve(0, 1, Color(200, 140, 90, 0.55f), 1.2f);  // R in -> L out
        drawMagCurve(0, 0, Color(90, 200, 230), 2.0f);       // L in -> L out
        drawMagCurve(1, 1, Color(230, 160, 60), 2.0f);       // R in -> R out
    }

    // ---- Phase panel (direct paths only) ----
    beginPath();
    rect(plotX, phaseY, plotW, phaseH);
    fillColor(Color(15, 16, 20));
    fill();

    for (double deg = kPhaseDegMin; deg <= kPhaseDegMax + 0.001; deg += 90.0)
    {
        const float y = valueToY(deg, kPhaseDegMin, kPhaseDegMax, phaseY, phaseH);
        beginPath();
        moveTo(plotX, y);
        lineTo(plotX + plotW, y);
        strokeColor(std::fabs(deg) < 0.001 ? Color(90, 94, 102) : Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%+.0f", deg);
        fillColor(Color(120, 124, 132));
        fontSize(10.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        text(plotX - 4.0f, y, buf, nullptr);
    }

    if (hasData_)
    {
        auto drawPhaseCurve = [&](int o, int i, Color color) {
            if (!visible_[o][i]) return;
            const auto& pts = curves_[o][i];
            if (pts.size() < 2) return;
            bool started = false;
            float lastY = 0.0f;
            for (size_t k = 0; k < pts.size(); ++k)
            {
                const float x = float(freqToX(pts[k].freqHz, kFMin, fMax, plotX, plotW, logScale_));
                const double wrapped = wrapDegrees(pts[k].phaseRad);
                const float y = valueToY(wrapped, kPhaseDegMin, kPhaseDegMax, phaseY, phaseH);
                // Break the polyline on a wrap discontinuity rather than draw a
                // vertical line across the whole panel.
                if (started && std::fabs(y - lastY) > phaseH * 0.6f) started = false;
                if (!started) { beginPath(); moveTo(x, y); started = true; }
                else lineTo(x, y);
                lastY = y;
            }
            strokeColor(color);
            strokeWidth(1.6f);
            stroke();
        };
        drawPhaseCurve(0, 0, Color(90, 200, 230));
        drawPhaseCurve(1, 1, Color(230, 160, 60));
    }

    // ---- Legend (click a swatch to show/hide that curve) ----
    struct LegendEntry { const char* label; Color color; int o, i; };
    const LegendEntry legend[] = {
        { "L -> L", Color(90, 200, 230), 0, 0 },
        { "R -> R", Color(230, 160, 60), 1, 1 },
        { "L -> R", Color(160, 110, 200), 1, 0 },
        { "R -> L", Color(200, 140, 90), 0, 1 },
    };
    float lx = plotX;
    const float ly = kLegendY;
    for (const auto& e : legend)
    {
        const bool shown = visible_[e.o][e.i];

        // Hit region covers the swatch + label together, easier to click
        // than the 10x10 swatch alone.
        constexpr float kEntryW = 64.0f;
        legendX_[e.o][e.i] = lx - 2.0f;
        legendY_[e.o][e.i] = ly - 7.0f;
        legendW_[e.o][e.i] = kEntryW;
        legendH_[e.o][e.i] = 14.0f;

        beginPath();
        rect(lx, ly - 5.0f, 10.0f, 10.0f);
        fillColor(shown ? e.color : Color(60, 62, 68));
        fill();
        if (!shown)
        {
            strokeColor(Color(120, 124, 132));
            strokeWidth(1.0f);
            stroke();
        }
        fontSize(10.5f);
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        fillColor(shown ? Color(180, 184, 192) : Color(100, 104, 112));
        text(lx + 15.0f, ly, e.label, nullptr);
        lx += 68.0f;
    }

    if (faultActive_)
    {
        fontSize(11.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(230, 90, 90));
        text(plotX + plotW, ly, "fault recovered", nullptr);
    }
}

END_NAMESPACE_DGL
