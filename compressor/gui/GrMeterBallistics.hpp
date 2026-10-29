// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// GrMeterBallistics.hpp
//
// The gain-reduction meter's speed, shared by CompressorGraphWidget,
// ContourGraphWidget and LimiterGraphWidget:
//
//   - GrMeterSmoother: the value the bar *and* the printed number show.
//     Asymmetric like a hardware GR meter: it follows a growing reduction
//     (or boost) with the short attack time and returns toward 0dB with
//     the long release time, so the onset of compression is never hidden
//     while the recovery stays calm enough to read. Replaces the earlier
//     symmetric 0.22s one-pole, which was both late going in and still
//     nervous coming back.
//   - Two speeds, chosen by the user with GrSpeedSwitch: Fast (the
//     default, close to what the compressor does) and Slow (an average of
//     the reduction, VU-like, single transients barely move it). The
//     peak-hold line and the history strip keep using the raw value in
//     both, so Slow never hides a peak -- the white line still shows it.
//   - GrSpeedSwitch: the labeled Fast | Slow switch drawn right below the
//     meter, so its effect is obvious without a manual. Saved per plugin
//     instance as the "grMeterSpeed" State.
//
// crispRect()/crispHLine(): the meter's frames and marker lines on whole
// device pixels (EditorSubWidget::snapLine()), so they stay sharp at 1x
// and zoomed instead of smearing over two half-lit pixels.
//
// The raw value already holds each block's deepest reduction for 50ms on
// the DSP side (sscomp::GrPeakMeter), so a short peak between two editor
// reads still arrives here.

#pragma once

#include "EditorSubWidget.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>

START_NAMESPACE_DGL

enum class GrMeterSpeed { Fast = 0, Slow = 1 };

// The "grMeterSpeed" State's value <-> GrMeterSpeed.
inline const char* grMeterSpeedStateValue(GrMeterSpeed s) { return s == GrMeterSpeed::Slow ? "1" : "0"; }
inline GrMeterSpeed grMeterSpeedFromState(const char* value)
{
    return (value != nullptr && std::atoi(value) != 0) ? GrMeterSpeed::Slow : GrMeterSpeed::Fast;
}

// A 1px-bordered rectangle path whose edges sit on device pixels.
inline void crispRect(EditorSubWidget& vg, float x, float y, float w, float h)
{
    const float x0 = vg.snapLine(x), y0 = vg.snapLine(y);
    const float x1 = vg.snapLine(x + w), y1 = vg.snapLine(y + h);
    vg.beginPath();
    vg.rect(x0, y0, x1 - x0, y1 - y0);
}

// Same, rounded (buttons).
inline void crispRoundedRect(EditorSubWidget& vg, float x, float y, float w, float h, float radius)
{
    const float x0 = vg.snapLine(x), y0 = vg.snapLine(y);
    const float x1 = vg.snapLine(x + w), y1 = vg.snapLine(y + h);
    vg.beginPath();
    vg.roundedRect(x0, y0, x1 - x0, y1 - y0, radius);
}

// A horizontal line of logical width `width`, stroked in `c`.
inline void crispHLine(EditorSubWidget& vg, float x0, float x1, float y, float width, const Color& c)
{
    const float sy = vg.snapLine(y, width);
    vg.beginPath();
    vg.moveTo(x0, sy);
    vg.lineTo(x1, sy);
    vg.strokeColor(c);
    vg.strokeWidth(vg.lineWidth(width));
    vg.stroke();
}

class GrMeterSmoother
{
public:
    // Call once per new raw reading (every uiIdle() tick), against
    // wall-clock time so the speed doesn't depend on the host's idle rate.
    void process(float rawDb, GrMeterSpeed speed)
    {
        const auto now = std::chrono::steady_clock::now();
        if (!init_)
        {
            value_ = rawDb; // nothing to smooth from yet
            init_ = true;
            last_ = now;
            return;
        }
        const float dtSec = std::chrono::duration<float>(now - last_).count();
        last_ = now;

        // Attack while moving away from 0dB (a deeper cut or a bigger
        // boost), release while returning toward it.
        const bool attack = (rawDb < 0.0f && rawDb < value_) || (rawDb > 0.0f && rawDb > value_);
        const float tauSec = speed == GrMeterSpeed::Slow ? (attack ? kSlowAttackSec : kSlowReleaseSec)
                                                         : (attack ? kFastAttackSec : kFastReleaseSec);
        const float coeff = dtSec > 0.0f ? std::exp(-dtSec / tauSec) : 1.0f;
        value_ = coeff * value_ + (1.0f - coeff) * rawDb;
    }

    float value() const { return value_; }

private:
    static constexpr float kFastAttackSec = 0.005f;  // effectively immediate at any idle rate
    static constexpr float kFastReleaseSec = 0.40f;
    static constexpr float kSlowAttackSec = 0.30f;   // ~VU integration time
    static constexpr float kSlowReleaseSec = 1.20f;

    float value_ = 0.0f;
    bool init_ = false;
    std::chrono::steady_clock::time_point last_{};
};

// The Fast | Slow switch: a "GR meter" caption over two side-by-side
// buttons, the active one lit. Drawn by the owning graph widget into a box
// it lays out below its GR meter column(s); the widget forwards clicks via
// hit() and asks tooltip() for its hover text.
class GrSpeedSwitch
{
public:
    void setBox(float x, float y, float w, float h) { x_ = x; y_ = y; w_ = w; h_ = h; }

    void draw(EditorSubWidget& vg, GrMeterSpeed speed) const
    {
        if (w_ <= 0.0f || h_ <= 0.0f) return;

        vg.fillColor(Color(140, 144, 152));
        vg.fontFace(NANOVG_DEJAVU_SANS_TTF);
        vg.fontSize(9.5f);
        vg.textAlign(NanoVG::ALIGN_CENTER | NanoVG::ALIGN_TOP);
        vg.text(std::round(x_ + w_ * 0.5f), y_ + 2.0f, "GR meter", nullptr);

        for (int seg = 0; seg < 2; ++seg)
        {
            float bx, by, bw, bh;
            buttonRect(seg, bx, by, bw, bh);
            const bool on = int(speed) == seg;
            crispRoundedRect(vg, bx, by, bw, bh, 2.5f);
            vg.fillColor(on ? Color(235, 195, 60) : Color(30, 32, 37));
            vg.fill();
            vg.strokeColor(on ? Color(235, 195, 60) : Color(70, 74, 82));
            vg.strokeWidth(vg.lineWidth());
            vg.stroke();

            vg.fillColor(on ? Color(20, 21, 25) : Color(170, 174, 182));
            vg.fontSize(10.0f);
            vg.textAlign(NanoVG::ALIGN_CENTER | NanoVG::ALIGN_MIDDLE);
            vg.text(std::round(bx + bw * 0.5f), std::round(by + bh * 0.5f), seg == 0 ? "Fast" : "Slow", nullptr);
        }
    }

    // 0 = Fast, 1 = Slow, -1 = not on either button.
    int hit(float px, float py) const
    {
        for (int seg = 0; seg < 2; ++seg)
        {
            float bx, by, bw, bh;
            buttonRect(seg, bx, by, bw, bh);
            if (px >= bx && px < bx + bw && py >= by && py < by + bh) return seg;
        }
        return -1;
    }

    // Caption included, so the hover text also shows over "GR meter".
    bool contains(float px, float py) const { return px >= x_ && px < x_ + w_ && py >= y_ && py < y_ + h_; }

    static const char* tooltip()
    {
        return "Speed of the gain reduction bar and number. Fast: follows the compressor closely. "
               "Slow: shows the average reduction, easier to read. The white line marks the latest peak in both.";
    }

private:
    void buttonRect(int seg, float& bx, float& by, float& bw, float& bh) const
    {
        constexpr float captionH = 14.0f, gap = 2.0f;
        bh = std::fmin(18.0f, h_ - captionH - 2.0f);
        by = y_ + captionH;
        bw = (w_ - gap) * 0.5f;
        bx = x_ + float(seg) * (bw + gap);
    }

    float x_ = 0.0f, y_ = 0.0f, w_ = 0.0f, h_ = 0.0f;
};

END_NAMESPACE_DGL
