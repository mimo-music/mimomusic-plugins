// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/param_strip_sim.cpp
//
// Headless drive of the REAL gui/ParamStripWidget.cpp (via tests/gui_stub/NanoVG.hpp):
// how far does a value move per pixel of vertical mouse drag? Written for the case
// where a drag starting at 0 (Attack/Release) felt stuck while any nonzero start
// was fine.
//
//   g++ -std=gnu++17 -O1 -fno-access-control -Itests/gui_stub -Igui -o /tmp/param_strip_sim \
//       tests/param_strip_sim.cpp gui/ParamStripWidget.cpp && /tmp/param_strip_sim
// The other four projects each carry their own copy of ParamStripWidget; the same file can be built against
// any of them by pointing -I at that project's gui/ directory (statespace: add -DSTRIP_FIELD_IS_FIELDID).
#include "ParamStripWidget.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

using namespace DGL;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) { std::printf("  %-100s %s\n", what, ok ? "OK" : "FAIL"); if (!ok) ++g_fail; }

struct Rig
{
    Widget parent;
    std::unique_ptr<ParamStripWidget> sp{new ParamStripWidget(&parent)};
    ParamStripWidget& w{*sp};
    float value = 0.0f;
    Rig(float startValue, float minV, float maxV, int decimals, bool multiplicative)
    {
        value = startValue;
        w.w_ = 300; w.h_ = 72;
        ParamFieldSpec f;
        f.label = "Attack"; f.unit = "ms"; f.minValue = minV; f.maxValue = maxV; f.defaultValue = 10.0f;
        f.nudgeStep = 0.05f; f.nudgeMultiplicative = multiplicative; f.decimals = decimals;
#ifdef STRIP_FIELD_IS_FIELDID
        f.fieldId = 0;
#else
        f.paramIndex = 0;
#endif
        w.setFields({ f });
        w.setGetter([this](uint32_t) { return value; });
        w.setSetter([this](uint32_t, float v) { value = v; });
        w.onNanoDisplay(); // computes the field layout
    }
    // press in the middle of the field, drag `dy` pixels UP (positive) in a few steps, release
    float drag(double dy)
    {
        const double x = w.fieldX_[0] + w.fieldW_[0] * 0.5, y = w.fieldY_ + w.fieldH_ * 0.5;
        MouseEvent e; e.pos = {x, y}; e.button = 1; e.press = true; w.onMouse(e);
        for (int i = 1; i <= 10; ++i) { MotionEvent m; m.pos = {x, y - dy * i / 10.0}; w.onMotion(m); }
        e.press = false; e.pos = {x, y - dy}; w.onMouse(e);
        return value;
    }
};
} // namespace

int main()
{
    std::printf("Attack field (0..300 ms, 2 decimals, multiplicative), drag UP by N pixels:\n");
    std::printf("  %-14s %s\n", "px up", "value reached from 0 ms  |  from 10 ms  |  from 100 ms");
    for (double px : { 3.0, 6.0, 10.0, 15.0, 20.0, 30.0, 50.0, 100.0 })
    {
        Rig a(0.0f, 0.0f, 300.0f, 2, true), b(10.0f, 0.0f, 300.0f, 2, true), c(100.0f, 0.0f, 300.0f, 2, true);
        const float va = a.drag(px), vb = b.drag(px), vc = c.drag(px);
        std::printf("  %-14.0f %8.3f ms              %8.3f ms     %8.3f ms\n", px, va, vb, vc);
    }
    {
        Rig a(0.0f, 0.0f, 300.0f, 2, true);
        const float v = a.drag(10.0);
        check(v >= 0.04f, "from exactly 0 ms, a 10 px drag reaches at least 0.04 ms (a visible change; it was 0.000 even after 50 px)");
        Rig a2(0.0f, 0.0f, 300.0f, 2, true);
        check(a2.drag(30.0) >= 0.3f, "from exactly 0 ms, a 30 px drag reaches at least 0.3 ms");
        Rig d(10.0f, 0.0f, 300.0f, 2, true);
        const float up = d.drag(20.0);
        check(std::fabs(up - 10.0f * std::exp(0.0976 * 20.0)) / up < 0.03, "from 10 ms the feel is unchanged: ~10% per pixel (exponential, as before)");
        Rig e(10.0f, 0.0f, 300.0f, 2, true);
        check(e.drag(-400.0) == 0.0f, "dragging all the way down still reaches exactly 0");
        Rig f(0.0f, 0.0f, 300.0f, 2, true);
        f.drag(60.0);
        const float hi = f.value;
        const float back = f.drag(-60.0);
        check(back == 0.0f || back < 0.06f, "drag up then down by the same amount returns to (about) 0");
        Rig g(0.0f, 0.0f, 300.0f, 2, true);
        check(g.drag(200.0) <= 300.0f, "the maximum is never exceeded");
        (void)hi;
        Rig h(5.0f, 0.1f, 300.0f, 2, true); // a field whose minimum is > 0 keeps the plain log mapping
        const float hv = h.drag(20.0);
        check(std::fabs(hv - 5.0f * std::exp(((std::log(300.0) - std::log(0.1)) / 200.0) * 20.0)) / hv < 0.02, "a field with a positive minimum keeps its plain log-scale drag");
    }
    std::printf("\n%s: %d failure(s)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
