// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// limiter_widget_sim.cpp
//
// Drives the real LimiterGraphWidget (drawing stubbed out by
// tests/gui_stub/NanoVG.hpp) with synthetic mouse events: the Soft clip's
// knee line and the Ceiling line sit only a few pixels apart, so each must
// grab its own drags; the knee line must not be grabbable in Hard mode.
// Also the GR meter's Fast | Slow switch (clicks, hover text) and its
// ballistics (gui/GrMeterBallistics.hpp, timed with short real sleeps).
//
// Build/run from compressor/:
//   g++ -std=gnu++17 -O1 -fno-access-control -Itests/gui_stub -Igui -Iplugin/dsp \
//       -o /tmp/limiter_widget_sim tests/limiter_widget_sim.cpp gui/LimiterGraphWidget.cpp \
//   && /tmp/limiter_widget_sim
#include "LimiterGraphWidget.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

using namespace DGL;

namespace {
struct Rig
{
    LimiterGraphWidget g{nullptr};
    float threshold = -999, ceiling = -999, knee = -999;
    int speedChanges = 0;
    GrMeterSpeed lastSpeed = GrMeterSpeed::Fast;
    explicit Rig(bool soft)
    {
        g.w_ = 900; g.h_ = 520;
        LimiterGraphWidget::CurveParams p;
        p.thresholdDb = -6.0f; p.ceilingDb = -1.0f; p.kneeDb = 0.0f;
        p.effectiveThresholdDb = -6.0f; p.postGainDb = soft ? 4.0f : 5.0f; p.requestedPostGainDb = 6.0f;
        p.softClip = soft; p.clipKneeDb = 1.0f;
        g.setCurveParams(p);
        g.setOnThresholdChanged([this](float v) { threshold = v; });
        g.setOnCeilingChanged([this](float v) { ceiling = v; });
        g.setOnClipKneeChanged([this](float v) { knee = v; });
        g.setOnGrMeterSpeedChanged([this](GrMeterSpeed s) { lastSpeed = s; ++speedChanges; });
        g.onNanoDisplay(); // computes the curve rectangle
    }
    void drag(double x0, double y0, double x1, double y1)
    {
        MouseEvent e; e.pos = {x0, y0}; e.button = 1; e.press = true; g.onMouse(e);
        for (int i = 1; i <= 3; ++i) { MotionEvent m; m.pos = {x0 + (x1 - x0) * i / 3.0, y0 + (y1 - y0) * i / 3.0}; g.onMotion(m); }
        MouseEvent r; r.pos = {x1, y1}; r.button = 1; r.press = false; g.onMouse(r);
    }
};
}

int main()
{
    int fails = 0;
    auto check = [&](bool ok, const char* what, double a) { std::printf("%-78s %8.2f  %s\n", what, a, ok ? "OK" : "FAIL"); if (!ok) ++fails; };

    {
        Rig r(true);
        const float x = r.g.curveX_ + r.g.curveW_ * 0.3f;
        const float ceilY = r.g.dbToPxY(-1.0f), kneeY = r.g.dbToPxY(-2.0f);
        std::printf("Soft: Ceiling line at y=%.1f, knee line at y=%.1f (%.1f px apart)\n", ceilY, kneeY, kneeY - ceilY);
        r.drag(x, kneeY + 1.0, x, r.g.dbToPxY(-4.0f) + 1.0);
        check(std::fabs(r.knee - 3.0f) < 0.3f && r.ceiling == -999, "[1] drag the knee line from -2 to -4 dB: Clip Knee ~3 dB, Ceiling untouched", r.knee);
    }
    {
        Rig r(true);
        const float x = r.g.curveX_ + r.g.curveW_ * 0.3f;
        r.drag(x, r.g.dbToPxY(-1.0f) - 1.0, x, r.g.dbToPxY(-3.0f) - 1.0);
        check(std::fabs(r.ceiling - (-3.0f)) < 0.3f && r.knee == -999, "[2] drag the Ceiling line to -3 dB: Ceiling ~-3, Clip Knee untouched", r.ceiling);
    }
    {
        Rig r(false);
        const float x = r.g.curveX_ + r.g.curveW_ * 0.3f;
        r.drag(x, r.g.dbToPxY(-2.0f), x, r.g.dbToPxY(-4.0f));
        check(r.knee == -999 && r.ceiling == -999, "[3] Hard: where the knee line would be, nothing is grabbed", r.knee);
    }
    {
        Rig r(true);
        // Threshold handle sits on the curve at x = -6 dB, y = output(-6) = -6 + 4 = -2 dB -- on the knee line.
        const float hx = r.g.dbToPxX(-6.0f), hy = r.g.dbToPxY(r.g.outputDbFor(-6.0f));
        std::printf("Threshold handle at output %.2f dB\n", r.g.outputDbFor(-6.0f));
        r.drag(hx, hy, r.g.dbToPxX(-12.0f), hy);
        check(std::fabs(r.threshold - (-12.0f)) < 0.5f && r.knee == -999, "[4] the Threshold handle (on the knee line) still drags the Threshold", r.threshold);
    }
    {
        // True Peak draws infinity:1: flat above the Threshold. The faint "limiter only" line is the
        // full curve minus the gain after the limiter (below the Threshold, no clip there).
        Rig r(false);
        LimiterGraphWidget::CurveParams p = r.g.p_;
        p.ratio = 1.0 / 0.0; p.postGainDb = 5.0f; p.kneeDb = 0.0f;
        r.g.setCurveParams(p);
        const float flat = r.g.outputDbFor(0.0f) - r.g.outputDbFor(-5.0f);
        check(std::fabs(flat) < 1e-4f, "[5] inf:1 curve flat above the Threshold (output(0) - output(-5), dB)", flat);
        const float gap = r.g.outputDbFor(-30.0f) - r.g.limitedDbFor(-30.0f);
        check(std::fabs(gap - 5.0f) < 1e-4f, "[6] full curve - limiter-only line below the Threshold == gain after limiter", gap);
        p.ratio = 20.0;
        r.g.setCurveParams(p);
        const float slope = r.g.limitedDbFor(0.0f) - r.g.limitedDbFor(-5.0f);
        check(std::fabs(slope - 0.25f) < 1e-3f, "[7] 20:1: limiter-only line rises 5/20 = 0.25 dB over 5 dB", slope);
    }
    {
        // The switch sits below the GR meter: caption, then Fast | Slow.
        Rig r(true);
        const GrSpeedSwitch& sw = r.g.grSpeedSwitch_;
        const float y = sw.y_ + 14.0f + 8.0f;
        const float fastX = sw.x_ + sw.w_ * 0.25f, slowX = sw.x_ + sw.w_ * 0.75f;
        check(sw.w_ > 40.0f && sw.y_ > r.g.curveY_ + r.g.curveH_ && sw.x_ > r.g.curveX_ + r.g.curveW_,
              "[8] the Fast | Slow switch is laid out below the GR meter (width, px)", sw.w_);
        MouseEvent e; e.pos = {slowX, y}; e.button = 1; e.press = true;
        const bool used = r.g.onMouse(e);
        e.press = false; r.g.onMouse(e);
        check(used && r.speedChanges == 1 && r.lastSpeed == GrMeterSpeed::Slow && r.g.grSpeed_ == GrMeterSpeed::Slow &&
              r.threshold == -999 && r.ceiling == -999 && r.knee == -999,
              "[9] click Slow: speed Slow, reported once, no handle grabbed", r.speedChanges);
        e.press = true; r.g.onMouse(e); e.press = false; r.g.onMouse(e);
        check(r.speedChanges == 1, "[10] click Slow again: nothing reported", r.speedChanges);
        e.pos = {fastX, y}; e.press = true; r.g.onMouse(e);
        check(r.speedChanges == 2 && r.lastSpeed == GrMeterSpeed::Fast, "[11] click Fast: back to Fast", r.speedChanges);
        check(!r.g.tooltipAt(sw.x_ + 2.0f, sw.y_ + 4.0f).empty() && r.g.tooltipAt(r.g.curveX_ + 20.0f, r.g.curveY_ + 20.0f).empty(),
              "[12] hover text over the switch (caption included), none over the curve", 0);
        r.g.setGrMeterSpeed(GrMeterSpeed::Slow);
        check(r.speedChanges == 2 && r.g.grSpeed_ == GrMeterSpeed::Slow, "[13] setGrMeterSpeed() (State restore) doesn't report back", r.speedChanges);
        check(grMeterSpeedFromState("1") == GrMeterSpeed::Slow && grMeterSpeedFromState("0") == GrMeterSpeed::Fast &&
              grMeterSpeedFromState("") == GrMeterSpeed::Fast && std::string(grMeterSpeedStateValue(GrMeterSpeed::Slow)) == "1",
              "[14] State value round trip", 0);
    }
    {
        // Ballistics: Fast follows a new reduction almost at once and returns
        // slowly; Slow averages. One process() call per sleep, like uiIdle().
        auto run = [](GrMeterSpeed s, float rawDb, int ms, GrMeterSmoother& sm) {
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            sm.process(rawDb, s);
            return sm.value();
        };
        GrMeterSmoother fast, slow;
        fast.process(0.0f, GrMeterSpeed::Fast);
        const float fastIn = run(GrMeterSpeed::Fast, -10.0f, 40, fast);
        slow.process(0.0f, GrMeterSpeed::Slow);
        const float slowIn = run(GrMeterSpeed::Slow, -10.0f, 40, slow);
        check(fastIn < -9.5f, "[15] Fast: -10 dB reduction shown within one 40 ms tick (dB)", fastIn);
        check(slowIn > -3.0f && slowIn < -0.5f, "[16] Slow: after 40 ms only part of it (dB)", slowIn);
        const float fastOut = run(GrMeterSpeed::Fast, 0.0f, 100, fast);
        check(fastOut < -6.0f && fastOut > -9.5f, "[17] Fast: 100 ms after the reduction ends, still returning (dB)", fastOut);
        GrMeterSmoother up;
        up.process(0.0f, GrMeterSpeed::Fast);
        const float boost = run(GrMeterSpeed::Fast, 6.0f, 40, up);
        check(boost > 5.7f, "[18] Fast: an upward boost uses the attack time too (dB)", boost);
    }
    std::printf("\n%s: %d check(s) failed\n", fails == 0 ? "PASS" : "FAILURES", fails);
    return fails == 0 ? 0 : 1;
}
