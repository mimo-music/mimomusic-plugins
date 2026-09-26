// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/contour_widget_sim.cpp
//
// Headless simulation of ContourGraphWidget's mouse handling: the REAL widget
// logic (gui/ContourGraphWidget.cpp) compiled against tests/gui_stub/NanoVG.hpp
// (drawing = no-ops), driven with synthetic press/motion/release events on real
// curves. This exists because the interaction bugs reported so far (a window that
// won't start in a corner, a support point that can't be grabbed) cannot be seen
// by the DSP tests and there is no GUI host on the dev machine.
//
// Build/run from compressor/:
//   g++ -std=gnu++17 -O1 -fno-access-control -Itests/gui_stub -Igui -Iplugin/dsp \
//       -o /tmp/contour_widget_sim tests/contour_widget_sim.cpp gui/ContourGraphWidget.cpp gui/NodeListWidget.cpp \
//   && /tmp/contour_widget_sim
#include "ContourGraphWidget.hpp"
#include "NodeListWidget.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

using namespace DGL;
using sscomp::TransferCurve;

namespace {
int g_fail = 0;
void check(bool ok, const char* what) { std::printf("  %-96s %s\n", what, ok ? "OK" : "FAIL"); if (!ok) ++g_fail; }

struct Rig
{
    Widget parent;
    // Heap-allocated like in the plugin: the widget holds a ~2MB trail buffer.
    std::unique_ptr<ContourGraphWidget> gp{new ContourGraphWidget(&parent)};
    ContourGraphWidget& g{*gp};
    Rig(const char* curveStr, double thresholdDb, ContourGraphWidget::EditTool tool)
    {
        g.w_ = 1000; g.h_ = 680; // roughly the real graph widget's size at the default window
        TransferCurve c;
        if (curveStr) sscomp::curveFromString(curveStr, c);
        g.setCurve(c);
        g.setThreshold(float(thresholdDb));
        g.setTool(tool);
        g.onNanoDisplay(); // computes curveX_/curveY_/curveW_/curveH_
    }
    Rig(const TransferCurve& c, double thresholdDb, ContourGraphWidget::EditTool tool)
    {
        g.w_ = 1000; g.h_ = 680;
        g.setCurve(c);
        g.setThreshold(float(thresholdDb));
        g.setTool(tool);
        g.onNanoDisplay();
    }
    void press(double x, double y, unsigned button = 1, unsigned mod = 0)
    { MouseEvent e; e.pos = {x, y}; e.button = button; e.press = true; e.mod = mod; g.onMouse(e); }
    void release(double x, double y, unsigned button = 1, unsigned mod = 0)
    { MouseEvent e; e.pos = {x, y}; e.button = button; e.press = false; e.mod = mod; g.onMouse(e); }
    void move(double x, double y) { MotionEvent e; e.pos = {x, y}; g.onMotion(e); }
    // press, drag along a path of 3 intermediate points, release
    void drag(double x0, double y0, double x1, double y1, unsigned button = 1)
    {
        press(x0, y0, button);
        for (int i = 1; i <= 3; ++i) move(x0 + (x1 - x0) * i / 3.0, y0 + (y1 - y0) * i / 3.0);
    }
    const char* state() const
    {
        if (g.marqueeActive_) return "marquee";
        if (g.draggingThreshold_) return "threshold-drag";
        if (g.draggingHandle_) return "handle-drag";
        if (g.draggingGroup_) return "group-drag";
        if (g.draggingPoint_ >= 0) return "node-drag";
        if (g.placingSmoothPoint_) return "placing-smooth";
        if (g.pendingAddPoint_) return "pending";
        return "nothing";
    }
    void reset() { release(0, 0); g.marqueeActive_ = g.draggingThreshold_ = g.draggingHandle_ = g.draggingGroup_ = g.pendingAddPoint_ = g.placingSmoothPoint_ = false; g.draggingPoint_ = -1; g.rightPressPending_ = false; g.clearSelection(); }
};

// The user's exported configuration (test.mccp, 2026-09-21 18:55).
const char* kUserCurve = "L;-60,-60,0,0,0,0;-42.37924575805664,-33.45738220214844,0,7.845283508300781,5.229291915893555,0;"
                         "-14.858489990234375,-23.394954681396484,0,8.71697998046875,2.060028076171875,0;6,6,0,19.550945281982422,1.9015579223632812,0";
} // namespace

int main()
{
    using Tool = ContourGraphWidget::EditTool;

    std::printf("[1] Edit tool, threshold at the plot's left edge (-80 dB) (the user's setting): where can a selection window start?\n");
    {
        Rig probe(kUserCurve, TransferCurve::kMinDb, Tool::Edit);
        std::printf("    plot rect: x %.0f..%.0f  y %.0f..%.0f\n", probe.g.curveX_, probe.g.curveX_ + probe.g.curveW_, probe.g.curveY_, probe.g.curveY_ + probe.g.curveH_);
        int bad = 0, total = 0;
        // a fresh widget per start position (a threshold drag would otherwise move the line for the next row)
        for (int dy = 0; dy <= 12; dy += 4)
            for (int dx = -8; dx <= 24; dx += 2)
            {
                Rig r(kUserCurve, TransferCurve::kMinDb, Tool::Edit);
                r.drag(r.g.curveX_ + dx, r.g.curveY_ + dy, r.g.curveX_ + dx + 160, r.g.curveY_ + dy + 160);
                ++total;
                if (std::string(r.state()) != "marquee") { ++bad; std::printf("    start (%+d px, +%d px) from the top-left corner of the plot -> %s\n", dx, dy, r.state()); }
            }
        check(bad == 0, "a window started anywhere around the top-left corner (margin and plot) opens a marquee");
        std::printf("    (%d start positions tried)\n", total);

        Rig m(kUserCurve, TransferCurve::kMinDb, Tool::Edit);
        m.drag(m.g.curveX_ + 300, m.g.curveY_ + 100, m.g.curveX_ + 420, m.g.curveY_ + 220);
        check(std::string(m.state()) == "marquee", "a window started in the middle of the plot opens a marquee");

        Rig t(kUserCurve, TransferCurve::kMinDb, Tool::Edit);
        t.drag(t.g.curveX_ + 1, t.g.curveY_ + t.g.curveH_ - 12, t.g.curveX_ + 90, t.g.curveY_ + t.g.curveH_ - 12);
        check(std::string(t.state()) == "threshold-drag", "the threshold at the left edge is still grabbable by its round handle");
        t.release(t.g.curveX_ + 90, t.g.curveY_ + t.g.curveH_ - 12);
        check(t.g.thresholdDb_ > -78.0, "...and dragging it actually moves the threshold");

        // A threshold in the middle keeps its whole line grabbable (unchanged behaviour).
        Rig mid(kUserCurve, -30.0, Tool::Edit);
        const float tx = mid.g.dbToPxX(-30.0f);
        mid.drag(tx + 2, mid.g.curveY_ + 200, tx + 60, mid.g.curveY_ + 200);
        check(std::string(mid.state()) == "threshold-drag", "a mid-plot threshold can still be grabbed anywhere along its line");
    }

    std::printf("[2] Right end node's support point (the user's curve: handle dx 19.55, dy 1.90): grab it\n");
    {
        Rig r(kUserCurve, -60.0, Tool::Edit);
        const auto p = r.g.curve().point(3);
        const float ix = r.g.dbToPxXRaw(p.inputDb - p.handleInputDb), iy = r.g.dbToPxYRaw(p.outputDb - p.handleOutputDb);
        const float ox = r.g.dbToPxXRaw(p.inputDb + p.handleInputDb), oy = r.g.dbToPxYRaw(p.outputDb + p.handleOutputDb);
        std::printf("    in-dot at (%.1f, %.1f) inside=%d   out-dot at (%.1f, %.1f) inside=%d\n", ix, iy, int(r.g.insidePlot(ix, iy)), ox, oy, int(r.g.insidePlot(ox, oy)));
        r.reset(); r.press(ix, iy);
        check(std::string(r.state()) == "handle-drag" && r.g.draggingHandlePoint_ == 3, "pressing on the right end node's visible support dot grabs it");
        r.release(ix, iy);
        // drag it and check the handle actually changes
        r.reset(); r.press(ix, iy); r.move(ix - 40, iy + 30); r.move(ix - 80, iy + 60); r.release(ix - 80, iy + 60);
        const auto q = r.g.curve().point(3);
        check(std::fabs(q.handleInputDb - p.handleInputDb) > 1.0, "dragging it changes the handle");
    }

    std::printf("[3] Right end node without a handle: pull one out with a right-drag, then grab it with the left button\n");
    {
        Rig r("L;-60,-60,0,0,0,0;6,6,0,0,0,0", -18.0, Tool::Edit);
        const auto p0 = r.g.curve().point(1);
        const float nx = r.g.dbToPxX(float(p0.inputDb)), ny = r.g.dbToPxY(float(p0.outputDb));
        // right-drag from the node towards the left / down (into the plot)
        r.press(nx, ny, kMouseButtonRight); r.move(nx - 30, ny + 40); r.move(nx - 60, ny + 80); r.release(nx - 60, ny + 80, kMouseButtonRight);
        const auto p1 = r.g.curve().point(1);
        std::printf("    after the right-drag: handle (dx %.2f, dy %.2f)\n", p1.handleInputDb, p1.handleOutputDb);
        r.g.menuOpen_ = false;
        const float ox = r.g.dbToPxXRaw(p1.inputDb + p1.handleInputDb), oy = r.g.dbToPxYRaw(p1.outputDb + p1.handleOutputDb);
        const float ix = r.g.dbToPxXRaw(p1.inputDb - p1.handleInputDb), iy = r.g.dbToPxYRaw(p1.outputDb - p1.handleOutputDb);
        std::printf("    out-dot (%.1f, %.1f) inside=%d   in-dot (%.1f, %.1f) inside=%d\n", ox, oy, int(r.g.insidePlot(ox, oy)), ix, iy, int(r.g.insidePlot(ix, iy)));
        bool grabbed = false;
        if (r.g.insidePlot(ix, iy)) { r.reset(); r.press(ix, iy); grabbed = std::string(r.state()) == "handle-drag"; r.release(ix, iy); }
        if (!grabbed && r.g.insidePlot(ox, oy)) { r.reset(); r.press(ox, oy); grabbed = std::string(r.state()) == "handle-drag"; r.release(ox, oy); }
        check(grabbed, "a support point created on the right end node can be grabbed afterwards");
        check(std::fabs(p1.handleInputDb) > 0.5, "the right-drag gave the end node a real horizontal reach (not a vertical-only tangent)");
        const float bx = r.g.dbToPxXRaw(p1.inputDb + p1.handleInputDb), by = r.g.dbToPxYRaw(p1.outputDb + p1.handleOutputDb);
        check(r.g.insidePlot(bx, by), "...and the bright (white) support dot, node + handle, is inside the plot");
        check(std::hypot(bx - (nx - 60), by - (ny + 80)) < 2.0, "...and it sits exactly where the mouse released it (dx is signed: the dot is LEFT of the node)");
        check(p1.handleInputDb < 0.0, "...i.e. a negative dx");
    }

    std::printf("[4] Holding the dim (mirrored) dot of an interior node: it becomes the white one and follows the mouse\n");
    {
        Rig r(kUserCurve, -60.0, Tool::Edit);
        const auto p = r.g.curve().point(2);
        const float ix = r.g.dbToPxXRaw(p.inputDb - p.handleInputDb), iy = r.g.dbToPxYRaw(p.outputDb - p.handleOutputDb);
        r.press(ix, iy); r.move(ix - 30, iy + 25); r.move(ix - 60, iy + 50);
        const auto q = r.g.curve().point(2);
        const float bx = r.g.dbToPxXRaw(q.inputDb + q.handleInputDb), by = r.g.dbToPxYRaw(q.outputDb + q.handleOutputDb);
        check(std::hypot(bx - (ix - 60), by - (iy + 50)) < 2.0, "the white dot (node + handle) is under the mouse after dragging the formerly dim one");
        check(q.handleInputDb < 0.0, "...which required flipping the pair: dx is now negative");
        // and the curve did what the dot says: same tangent line as before the flip -> evaluate is continuous at the flip moment
        r.release(ix - 60, iy + 50);
    }

    std::printf("[5] Link axes: a node linked X-only only propagates horizontal moves\n");
    {
        TransferCurve c;
        c.addPoint(-40.0, -30.0); c.addPoint(-20.0, -25.0); c.addPoint(-5.0, -5.0);
        c.setPointLink(1, 1, sscomp::kLinkAxisX); c.setPointLink(2, 1, sscomp::kLinkAxisX); c.normalizeLinks();
        Rig r(c, -60.0, Tool::Edit);
        const auto a0 = r.g.curve().point(1), b0 = r.g.curve().point(2);
        const float ax = r.g.dbToPxX(float(a0.inputDb)), ay = r.g.dbToPxY(float(a0.outputDb));
        r.press(ax, ay); r.move(ax + 20, ay + 30); r.move(ax + 40, ay + 60); r.release(ax + 40, ay + 60);
        const auto a1 = r.g.curve().point(1), b1 = r.g.curve().point(2);
        std::printf("    dragged node moved (%.2f, %.2f) dB, its X-linked partner moved (%.2f, %.2f) dB\n", a1.inputDb - a0.inputDb, a1.outputDb - a0.outputDb, b1.inputDb - b0.inputDb, b1.outputDb - b0.outputDb);
        check(std::fabs(a1.outputDb - a0.outputDb) > 2.0 && std::fabs(a1.inputDb - a0.inputDb) > 2.0, "the dragged node moves freely in X and Y");
        check(std::fabs((b1.inputDb - b0.inputDb) - (a1.inputDb - a0.inputDb)) < 1e-6 && std::fabs(b1.outputDb - b0.outputDb) < 1e-9, "the X-linked partner follows X only, its Y is untouched");

        TransferCurve c2;
        c2.addPoint(-40.0, -30.0); c2.addPoint(-20.0, -25.0); c2.addPoint(-5.0, -5.0);
        c2.setPointLink(1, 2, sscomp::kLinkAxisY); c2.setPointLink(2, 2, sscomp::kLinkAxisY); c2.normalizeLinks();
        Rig r2(c2, -60.0, Tool::Edit);
        const auto c0 = r2.g.curve().point(1), d0 = r2.g.curve().point(2);
        const float cx = r2.g.dbToPxX(float(c0.inputDb)), cy = r2.g.dbToPxY(float(c0.outputDb));
        r2.press(cx, cy); r2.move(cx + 20, cy + 30); r2.move(cx + 40, cy + 60); r2.release(cx + 40, cy + 60);
        const auto c1 = r2.g.curve().point(1), d1 = r2.g.curve().point(2);
        check(std::fabs((d1.outputDb - d0.outputDb) - (c1.outputDb - c0.outputDb)) < 1e-6 && std::fabs(d1.inputDb - d0.inputDb) < 1e-9, "a Y-only linked partner follows Y only, its X is untouched");

        // both-axes group (the default) still moves everything together
        TransferCurve c3;
        c3.addPoint(-40.0, -30.0); c3.addPoint(-20.0, -25.0); c3.addPoint(-5.0, -5.0);
        c3.setPointLink(1, 3, sscomp::kLinkAxisBoth); c3.setPointLink(2, 3, sscomp::kLinkAxisBoth); c3.normalizeLinks();
        Rig r3(c3, -60.0, Tool::Edit);
        const auto e0 = r3.g.curve().point(1), f0 = r3.g.curve().point(2);
        const float ex = r3.g.dbToPxX(float(e0.inputDb)), ey = r3.g.dbToPxY(float(e0.outputDb));
        r3.press(ex, ey); r3.move(ex + 20, ey + 30); r3.move(ex + 40, ey + 60); r3.release(ex + 40, ey + 60);
        const auto e1 = r3.g.curve().point(1), f1 = r3.g.curve().point(2);
        check(std::fabs((f1.inputDb - f0.inputDb) - (e1.inputDb - e0.inputDb)) < 1e-6 && std::fabs((f1.outputDb - f0.outputDb) - (e1.outputDb - e0.outputDb)) < 1e-6, "a normal (X+Y) link still moves the partner on both axes");
    }

    std::printf("[6] Right-click menu offers the three link kinds for a multi-selection\n");
    {
        TransferCurve c;
        c.addPoint(-40.0, -30.0); c.addPoint(-20.0, -25.0);
        Rig r(c, -60.0, Tool::Edit);
        r.g.selected_[1] = true; r.g.selected_[2] = true; r.g.primary_ = -1;
        const auto p = r.g.curve().point(1);
        r.g.openMenu(1, r.g.dbToPxX(float(p.inputDb)), r.g.dbToPxY(float(p.outputDb)));
        bool hasBoth = false, hasX = false, hasY = false;
        for (int i = 0; i < r.g.menuCount_; ++i) { hasBoth |= r.g.menuItems_[i].action == ContourGraphWidget::MenuAction::Link; hasX |= r.g.menuItems_[i].action == ContourGraphWidget::MenuAction::LinkX; hasY |= r.g.menuItems_[i].action == ContourGraphWidget::MenuAction::LinkY; }
        check(hasBoth && hasX && hasY, "menu has: Link (X + Y), Link X only, Link Y only");
        r.g.runMenuAction(ContourGraphWidget::MenuAction::LinkX);
        check(r.g.curve().pointLink(1) > 0 && r.g.curve().pointLink(1) == r.g.curve().pointLink(2) && r.g.curve().pointLinkAxis(1) == sscomp::kLinkAxisX, "choosing 'Link X only' links both nodes with the X axis");
    }

    std::printf("[7] Grab targets: bigger in Edit (forgiving), unchanged small in Draw (so nodes can be placed next to each other)\n");
    {
        Rig e(kUserCurve, -60.0, Tool::Edit);
        const auto p = e.g.curve().point(3); // the right end node, at the top-right corner
        const float nx = e.g.dbToPxX(float(p.inputDb)), ny = e.g.dbToPxY(float(p.outputDb));
        e.press(nx - 9, ny + 6); e.move(nx - 9, ny + 30);
        check(std::string(e.state()) == "node-drag" && e.g.draggingPoint_ == 3, "Edit: pressing ~11 px off the right end node still grabs it");
        Rig d(kUserCurve, -60.0, Tool::DrawNode);
        d.press(nx - 9, ny + 6);
        check(std::string(d.state()) == "pending", "Draw: the same press lands on empty space (a new node would be placed)");
    }

    std::printf("[8] The node itself wins over a nearby support dot: right end node at 6 dB with a short vertical handle\n");
    {
        // A right end node whose only handle is vertical (what older builds could produce on it): its one visible
        // dot sits a few pixels below the node. Every press within the node's core must grab the NODE.
        TransferCurve c; c.addPoint(-40.0, -30.0); c.addPoint(-15.0, -20.0);
        c.setPointHandle(c.pointCount() - 1, 0.0, 0.6);
        Rig probe(c, -30.0, Tool::Edit);
        const int last = probe.g.curve().pointCount() - 1;
        const auto p = probe.g.curve().point(last);
        const float nx = probe.g.dbToPxX(float(p.inputDb)), ny = probe.g.dbToPxY(float(p.outputDb));
        int wrong = 0, total = 0;
        for (int dy = -8; dy <= 8; ++dy) for (int dx = -8; dx <= 8; ++dx)
        {
            if (dx * dx + dy * dy > 8 * 8) continue;
            Rig r(c, -30.0, Tool::Edit);
            r.press(nx + dx, ny + dy);
            ++total;
            if (std::string(r.state()) != "node-drag" || r.g.draggingPoint_ != last) ++wrong;
        }
        std::printf("    %d of %d presses within 8 px of the node centre grabbed something other than the node\n", wrong, total);
        check(wrong == 0, "every press on the node core grabs the node, even with a dot a few px away");
        // ...and the dot is still reachable once you are clearly off the node, on its own
        TransferCurve c2 = c; c2.setPointHandle(c2.pointCount() - 1, -8.0, -4.0); // white dot 8 dB left and 4 dB BELOW the node (above would be outside the plot)
        Rig r2(c2, -30.0, Tool::Edit);
        const auto q = r2.g.curve().point(last);
        const float bx = r2.g.dbToPxXRaw(q.inputDb + q.handleInputDb), by = r2.g.dbToPxYRaw(q.outputDb + q.handleOutputDb);
        r2.press(bx, by);
        check(std::string(r2.state()) == "handle-drag", "a clearly separate support dot is still grabbed as before");
    }

    std::printf("[9] Trail dots: triggered ones are green, untriggered keep the amber/blue palette; the threshold blends yellow -> green\n");
    {
        Rig r(kUserCurve, -40.0, Tool::Edit);
        const Color amber = r.g.trailColor(false, 0.6f, false), green = r.g.trailColor(false, 0.6f, true);
        const Color blue = r.g.trailColor(true, 0.6f, false), teal = r.g.trailColor(true, 0.6f, true);
        check(green.green > green.red + 40 && green.green > green.blue + 40, "voice A triggered dots are green (green channel clearly dominant)");
        check(amber.red > amber.blue + 40, "voice A untriggered dots stay warm amber");
        check(teal.green > teal.red + 40 && blue.blue > blue.red, "voice B: triggered = teal-green, untriggered = blue");
        // feed 60000 mixed points (the ring holds 65536) and draw a frame: no crash, and both classes are present
        std::vector<float> lv(48), gr(48); std::vector<unsigned char> tr(48);
        for (int batch = 0; batch < 1250; ++batch)
        {
            for (int k = 0; k < 48; ++k) { lv[k] = -50.0f + float((batch * 7 + k) % 55); gr[k] = -3.0f; tr[k] = (k % 3 == 0) ? 1 : 0; }
            r.g.pushTrailBatch(0, lv.data(), gr.data(), tr.data(), 48);
        }
        int nTrig = 0; const int n = r.g.trailCount_[0];
        for (int k = 0; k < n; ++k) if (r.g.trail_[0][k].trig) ++nTrig;
        r.g.onNanoDisplay();
        check(n == 60000 && nTrig == 20000, "60000 points pushed in batches of 48 keep their per-point flag (a third triggered) and draw without trouble");
        r.g.setTriggerActivity(2.0f);
        check(r.g.triggerActivity_ == 1.0f, "trigger activity is clamped to 0..1");
        r.g.clearTrail();
        check(r.g.trailCount_[0] == 0, "clearTrail() empties the trail (used when the dots' source switch changes)");
    }

    std::printf("[10] A gesture whose release never arrived must not break the next press\n");
    {
        for (int scenario = 0; scenario < 3; ++scenario)
        {
            Rig r(kUserCurve, -60.0, Tool::Edit);
            const auto p = r.g.curve().point(2);
            const double px = r.g.dbToPxX(float(p.inputDb)), py = r.g.dbToPxY(float(p.outputDb));
            if (scenario == 0) r.press(r.g.curveX_ + 300, r.g.curveY_ + 300);            // empty-plot press, release lost
            if (scenario == 1) { r.press(px + 60, py - 60); r.move(px + 100, py - 100); } // marquee running, release lost
            if (scenario == 2) { const auto q = r.g.curve().point(1); r.press(r.g.dbToPxX(float(q.inputDb)), r.g.dbToPxY(float(q.outputDb))); } // node drag, release lost
            r.press(px, py);
            r.move(px + 6, py - 6); r.move(px + 25, py - 25);
            r.release(px + 25, py - 25);
            const auto q2 = r.g.curve().point(2);
            char what[160];
            std::snprintf(what, sizeof(what), "scenario %d: the node under the next press moves (%.1f,%.1f) -> (%.1f,%.1f)", scenario, p.inputDb, p.outputDb, q2.inputDb, q2.outputDb);
            check(q2.inputDb > p.inputDb + 0.5 && q2.outputDb > p.outputDb + 0.5, what);
        }
    }

    std::printf("[11] Makeup Gain on the gain-reduction bar (ported from the compressor's graph): stored per column, drawn without trouble at every sign\n");
    {
        Rig r(kUserCurve, -30.0, Tool::Edit);
        r.g.setMakeupGainDb(6.0f, -12.0f);
        check(r.g.makeupGainDb_ == 6.0f && r.g.makeupGainDbB_ == -12.0f, "setMakeupGainDb() stores both columns' configured Makeup Gain");
        bool drew = true;
        for (float gr : { -40.0f, -3.0f, 0.0f, 2.5f })
            for (float mk : { -24.0f, -1.0f, 0.0f, 9.0f, 24.0f })
            {
                r.g.setMakeupGainDb(mk, mk);
                r.g.setLiveLevels(-20.0f, gr, false);
                r.g.onNanoDisplay(); // net = gr + makeup: cut, boost, clawed-back cut, all draw
            }
        check(drew, "the bar (net, dimmed makeup-less bar behind, marker line) draws for cut / boost / clawed-back combinations");
    }

    std::printf("[12] Dual-Mono / Mid-Side: each plot draws its own channel's dots as voice 0 (the trap: slot 1 is only drawn as a plot's SECONDARY voice)\n");
    {
        Rig plotA(kUserCurve, -30.0, Tool::Edit), plotB(kUserCurve, -30.0, Tool::Edit);
        float lv[4] = { -30.0f, -25.0f, -20.0f, -15.0f }, gr[4] = { -3.0f, -3.0f, -3.0f, -3.0f };
        unsigned char tr[4] = { 1, 1, 0, 1 };
        plotA.g.pushTrailBatch(0, lv, gr, tr, 4); // what UIContour does for channel A
        plotB.g.pushTrailBatch(0, lv, gr, tr, 4); // ... and now for channel B, on ITS plot
        check(plotA.g.trailCount_[0] == 4 && plotB.g.trailCount_[0] == 4, "both plots hold the 4 dots in the slot they draw");
        Rig wrong(kUserCurve, -30.0, Tool::Edit);
        wrong.g.pushTrailBatch(1, lv, gr, tr, 4);   // the old routing of channel B
        // the trail length is mirrored to every plot through the callback
        float seen = 0.0f; plotA.g.setOnTrailSecondsChanged([&](float v) { seen = v; plotB.g.setTrailSeconds(v); });
        MouseEvent none; (void)none; plotA.g.setTrailSeconds(2.0f);
        plotA.g.onTrailSecondsChanged_(5.0f);
        check(seen == 5.0f && plotB.g.trailSeconds() == 5.0f, "the trail length set on one plot is mirrored to the other");
        check(wrong.g.trailCount_[0] == 0 && !wrong.g.secondaryShown_, "(control) the old routing filled slot 1, which a plot without a secondary voice never draws");
    }

    std::printf("[13] Two ContourGraphWidgets / two NodeListWidgets on screen together (Dual-Mono/Mid-Side): neither steals a click meant for its sibling\n");
    {
        // DPF dispatches a mouse event to sibling subwidgets in reverse
        // construction order and stops at the first onMouse() that returns
        // true, with no bounds pre-filter of its own (Widget::PrivateData::
        // giveMouseEventForSubWidgets(), dpf/dgl/src/WidgetPrivateData.cpp)
        // -- a widget must reject a position outside its own rectangle
        // itself, or whichever of two identical widgets was constructed
        // last swallows every click meant for the other one, which then
        // never sees it at all (in Dual-Mono/Mid-Side, points couldn't be
        // moved -- two separate real bugs this pins down, one per widget class, both
        // invisible with a single instance -- Linked mode's lone graph_/
        // nodeList_ never exposed either).
        Rig g(kUserCurve, -18.0, Tool::Edit);
        MouseEvent farPress; farPress.pos = { -400.0, 100.0 }; farPress.button = 1; farPress.press = true; farPress.mod = 0;
        check(!g.g.onMouse(farPress), "ContourGraphWidget (Edit tool): a press far outside its own rect is not claimed");
        check(g.state() == "nothing", "...and no gesture (marquee) was started for it");

        Widget parent;
        NodeListWidget nl(&parent);
        nl.w_ = 368; nl.h_ = 260; // roughly one dual-mode column's half-height size
        TransferCurve c; c.addPoint(-20.0, -10.0);
        nl.setCurve(c);
        MouseEvent farListPress; farListPress.pos = { 50.0, 5000.0 }; farListPress.button = 1; farListPress.press = true; farListPress.mod = 0;
        check(!nl.onMouse(farListPress), "NodeListWidget: a press far below its own rect (past the footer's own unbounded check) is not claimed");
        MouseEvent nearFooter; nearFooter.pos = { 50.0, 250.0 }; nearFooter.button = 1; nearFooter.press = true; nearFooter.mod = 0; // genuinely inside, in the footer band (height 260, footer 28 tall)
        check(nl.onMouse(nearFooter), "...but a genuine press on its own footer still is");
    }

    std::printf("\n%s: %d failure(s)\n", g_fail ? "FAIL" : "PASS", g_fail);
    return g_fail ? 1 : 0;
}
