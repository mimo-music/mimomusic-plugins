// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/dyneq2_list_sim.cpp
//
// Headless drive of Dynamic EQ 2's band list: the REAL gui/dyneq2/
// DynEqBandListWidget2.cpp compiled against tests/gui_stub/NanoVG.hpp
// (drawing = no-ops), with a fake host that applies the same clamps as
// UIDynEq2, fed synthetic clicks, drags and scrolls per column.
//
// Build/run from statespace/ (one command line):
//   g++ -std=gnu++17 -O1 -fno-access-control -Itests/gui_stub -Igui/dyneq2 -o /tmp/dyneq2_list_sim tests/dyneq2_list_sim.cpp gui/dyneq2/DynEqBandListWidget2.cpp plugin/dsp/DynamicEqModel2.cpp plugin/dsp/EqModel.cpp plugin/dsp/StabilityGuard.cpp && /tmp/dyneq2_list_sim

#include "DynEqBandListWidget2.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace DGL;
using ssgui::DynEqField;

namespace {

struct FakeHost : ssgui::IDynEqEditorHost2
{
    ssfilter::DynamicEqModel2 m;
    int editBand = -1; DynEqField editField = DynEqField::Freq; bool editClear = false; bool editing = false;
    int removed = -1;

    ssfilter::DynamicEqBand2& b(int i) { return m.bands[size_t(i)]; }
    bool ok(int i) const { return i >= 0 && i < int(m.bands.size()); }

    int getBandCount() const override { return int(m.bands.size()); }
    ssfilter::DynamicEqBand2 getBand(int i) const override { return ok(i) ? m.bands[size_t(i)] : ssfilter::DynamicEqBand2{}; }
    ssfilter::DynamicEqModel2 getModel() const override { return m; }
    double getCurrentGainDb(int) const override { return 0.0; }
    void addBand(double, double) override {}
    void removeBand(int i) override { if (ok(i)) { m.bands.erase(m.bands.begin() + i); removed = i; } }
    void duplicateBand(int) override {}
    // Same clamps as UIDynEq2's setters.
    void setBandFreq(int i, double v) override { if (ok(i)) b(i).freqHz = std::max(20.0, std::min(20000.0, v)); }
    void setBandGain(int i, double v) override { if (ok(i)) b(i).gainDb = std::max(-24.0, std::min(24.0, v)); }
    void setBandQ(int i, double v) override { if (ok(i)) b(i).q = std::max(0.05, std::min(100.0, v)); }
    void setBandBandwidth(int i, double v) override { if (ok(i) && b(i).type == ssfilter::EqBandType::BandShelf) b(i).bandwidthOct = std::max(ssfilter::kMinBandwidthOct, std::min(ssfilter::kMaxBandwidthOct, v)); }
    void setBandThreshold(int i, double v) override { if (ok(i)) b(i).thresholdDb = std::max(-72.0, std::min(0.0, v)); }
    void setBandRatio(int i, double v) override { if (ok(i)) b(i).ratio = std::max(1.0, std::min(20.0, v)); }
    void setBandRatioUp(int i, double v) override { if (ok(i)) b(i).ratioUp = std::max(1.0, std::min(20.0, v)); }
    void setBandRange(int i, double v) override { if (ok(i)) b(i).rangeDb = std::max(0.0, std::min(40.0, v)); }
    void setBandAttack(int i, double v) override { if (ok(i)) b(i).attackMs = std::max(0.1, std::min(200.0, v)); }
    void setBandRelease(int i, double v) override { if (ok(i)) b(i).releaseMs = std::max(5.0, std::min(2000.0, v)); }
    void setBandDynamicsEnabled(int i, bool e) override { if (ok(i)) b(i).dynamicsEnabled = e; }
    void setBandEnabled(int i, bool e) override { if (ok(i)) b(i).enabled = e; }
    void setBandSolo(int i, bool s) override { if (!ok(i)) return; if (s) m.soloedBand = i; else if (m.soloedBand == i) m.soloedBand = -1; }
    void setBandChannel(int i, ssfilter::EqChannel c) override { if (ok(i)) b(i).channel = c; }
    void setBandType(int i, ssfilter::EqBandType t) override { if (ok(i)) b(i).type = t; }
    void setBandCrossFeed(int i, ssfilter::EqCrossFeed c) override { if (ok(i)) b(i).crossFeed = c; }
    void setCrossFeedGainLtoRDb(double) override {}
    void setCrossFeedGainRtoLDb(double) override {}
    void createHarmonicTwin(int, double) override {}
    void unlinkHarmonic(int) override {}
    void beginHarmonicFactorEdit(int) override {}
    bool isEditingHarmonicFactor(int, std::string&) const override { return false; }
    void commitHarmonicFactorEdit() override {}
    void beginBandAudition(int) override {}
    void endBandAudition() override {}
    void beginBandTextEdit(int i, DynEqField f, bool clear) override { editBand = i; editField = f; editClear = clear; editing = true; }
    bool isEditingBandField(int i, DynEqField f, std::string& t) const override { if (editing && i == editBand && f == editField) { t = "1"; return true; } return false; }
};

int g_fails = 0;
void check(bool ok, const char* what)
{
    std::printf("%-78s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok) ++g_fails;
}

struct Sim
{
    FakeHost host;
    DynEqBandListWidget2 list{nullptr, &host};
    int selected = -1;
    Sim()
    {
        list.w_ = 1168; list.h_ = 240;
        list.setOnSelectionChanged([this](int i) { selected = i; });
        for (int i = 0; i < 3; ++i) { ssfilter::DynamicEqBand2 b; b.freqHz = 250.0 * (i + 1); host.m.bands.push_back(b); }
    }
    double cx(int col) { const auto c = list.columns(); return c.x[col] + c.w[col] * 0.5; }
    double cy(int row) { return 16.0 + 24.0 * row + 12.0; } // kHeaderH + row*kRowH + half
    void press(double x, double y, bool down) { MouseEvent e; e.pos = {x, y}; e.button = 1; e.press = down; list.onMouse(e); }
    void click(int col, int row) { press(cx(col), cy(row), true); press(cx(col), cy(row), false); }
    void drag(int col, int row, double dyUp)
    {
        press(cx(col), cy(row), true);
        MotionEvent m; m.pos = {cx(col), cy(row) - dyUp}; list.onMotion(m);
        press(cx(col), cy(row) - dyUp, false);
    }
    void scroll(int col, int row, double dir, uint32_t mod = 0)
    {
        ScrollEvent e; e.pos = {cx(col), cy(row)}; e.delta = {0.0, dir}; e.mod = mod; list.onScroll(e);
    }
};

using L = DynEqBandListWidget2;

} // namespace

int main()
{
    {
        Sim s;
        // Columns must not overlap and must fit the default window width.
        const auto c = s.list.columns();
        bool ordered = true;
        for (int i = 1; i < L::kNumCols; ++i) ordered = ordered && c.x[i] >= c.x[i - 1] + c.w[i - 1];
        char buf[96];
        std::snprintf(buf, sizeof(buf), "columns are ordered, non-overlapping, and end at %.0f px (< 1168)", c.x[L::kNumCols - 1] + c.w[L::kNumCols - 1]);
        check(ordered && c.x[L::kNumCols - 1] + c.w[L::kNumCols - 1] < 1168.0f, buf);

        // Type cycles Bell -> Low Shelf -> High Shelf -> Band Shelf -> Bell.
        using T = ssfilter::EqBandType;
        T seen[4];
        for (int k = 0; k < 4; ++k) { s.click(L::kColType, 0); seen[k] = s.host.m.bands[0].type; }
        check(seen[0] == T::LowShelf && seen[1] == T::HighShelf && seen[2] == T::BandShelf && seen[3] == T::Bell,
              "TYPE cycles Bell -> Low Shelf -> High Shelf -> Band Shelf -> Bell");

        // Toggles.
        s.click(L::kColEnable, 1);
        s.click(L::kColDynamics, 1);
        check(!s.host.m.bands[1].enabled && !s.host.m.bands[1].dynamicsEnabled, "EN and DYN toggle their own row only");
        check(s.host.m.bands[0].enabled && s.host.m.bands[0].dynamicsEnabled && s.host.m.bands[2].enabled, "other rows untouched");

        // Solo: one band at a time.
        s.click(L::kColSolo, 0); const int a = s.host.m.soloedBand;
        s.click(L::kColSolo, 2); const int b = s.host.m.soloedBand;
        s.click(L::kColSolo, 2); const int c2 = s.host.m.soloedBand;
        check(a == 0 && b == 2 && c2 == -1, "SOLO: moves to the clicked row, clicking the soloed row clears it");

        // Channel: Both -> Left -> Right -> L->R -> R->L -> Both.
        using Ch = ssfilter::EqChannel; using Cf = ssfilter::EqCrossFeed;
        bool chOk = true;
        const struct { Ch ch; Cf cf; } expect[5] = { {Ch::Left, Cf::None}, {Ch::Right, Cf::None}, {Ch::Both, Cf::LeftToRight}, {Ch::Both, Cf::RightToLeft}, {Ch::Both, Cf::None} };
        for (int k = 0; k < 5; ++k)
        {
            s.click(L::kColChannel, 0);
            const auto& bb = s.host.m.bands[0];
            const bool cfOk = bb.crossFeed == expect[k].cf;
            const bool chk = expect[k].cf != Cf::None || bb.channel == expect[k].ch; // channel is ignored while crossfeeding
            chOk = chOk && cfOk && chk;
        }
        check(chOk, "CH cycles Both / Left / Right / L>R / R>L / Both");

        // Row click selects.
        const auto cc = s.list.columns();
        s.press(cc.x[L::kColQ] + cc.w[L::kColQ] + 1.0, s.cy(2), true); // the gap after Q
        s.press(cc.x[L::kColQ] + cc.w[L::kColQ] + 1.0, s.cy(2), false);
        check(s.selected == 2, "a click between controls selects the row");
    }

    {
        Sim s;
        // Numeric fields: click opens the text edit on the right field.
        s.click(L::kColThreshold, 1);
        check(s.host.editing && s.host.editBand == 1 && s.host.editField == DynEqField::Threshold && !s.host.editClear,
              "click on THRESH starts the text edit on that band's Threshold");
        s.host.editing = false;
        s.click(L::kColRelease, 2); s.host.editing = false;
        s.click(L::kColRelease, 2);
        check(s.host.editClear, "double-click on a field starts the edit from an empty buffer");

        // Q only applies to a Bell, BW only to a Band Shelf.
        s.host.editing = false;
        s.host.m.bands[0].type = ssfilter::EqBandType::LowShelf;
        s.click(L::kColQ, 0);
        const bool qBlocked = !s.host.editing;
        s.host.m.bands[0].type = ssfilter::EqBandType::Bell;
        s.click(L::kColBandwidth, 0);
        const bool bwBlocked = !s.host.editing;
        s.host.m.bands[0].type = ssfilter::EqBandType::BandShelf;
        s.click(L::kColBandwidth, 0);
        check(qBlocked && bwBlocked && s.host.editing && s.host.editField == DynEqField::Bandwidth,
              "\"--\" fields (Q on a shelf, BW off a Band Shelf) ignore clicks; BW works on a Band Shelf");

        // Drag: linear and log columns, from the press value.
        s.host.editing = false;
        s.host.m.bands[1].rangeDb = 5.0;
        s.drag(L::kColRange, 1, 50.0); // +50px of 200 over 40 dB -> +10
        const double range = s.host.m.bands[1].rangeDb;
        s.host.m.bands[1].attackMs = 10.0;
        s.drag(L::kColAttack, 1, -100.0); // -100px: 10 * 2000^-0.5
        const double attack = s.host.m.bands[1].attackMs;
        s.host.m.bands[1].thresholdDb = -24.0;
        s.drag(L::kColThreshold, 1, 200.0); // +72 dB -> clamped at 0
        char buf[140];
        std::snprintf(buf, sizeof(buf), "drag: RANGE 5 -> %.2f (want 15), ATTACK 10 -> %.3f (want %.3f), THRESH clamps at %.1f",
                      range, attack, 10.0 / std::sqrt(2000.0), s.host.m.bands[1].thresholdDb);
        check(std::fabs(range - 15.0) < 1e-9 && std::fabs(attack - 10.0 / std::sqrt(2000.0)) < 1e-9 && s.host.m.bands[1].thresholdDb == 0.0, buf);
        check(!s.host.editing, "a drag does not also open the text edit");

        // Scroll: factor columns multiply, linear ones add; Alt is 10x finer.
        s.host.m.bands[2].ratio = 4.0;
        s.scroll(L::kColRatio, 2, 1.0);
        const double r1 = s.host.m.bands[2].ratio;
        s.scroll(L::kColRatio, 2, -1.0, kModifierAlt);
        const double r2 = s.host.m.bands[2].ratio;
        s.host.m.bands[2].gainDb = 0.0;
        s.scroll(L::kColGain, 2, 1.0);
        const double g1 = s.host.m.bands[2].gainDb;
        std::snprintf(buf, sizeof(buf), "scroll: RATIO 4 -> %.3f -> %.4f (x1.05, then /1.005), GAIN 0 -> %+.1f",
                      r1, r2, g1);
        check(std::fabs(r1 - 4.2) < 1e-9 && std::fabs(r2 - 4.2 / 1.005) < 1e-9 && std::fabs(g1 - 0.5) < 1e-9, buf);

        // Delete.
        s.click(L::kColDelete, 1);
        check(s.host.removed == 1 && s.host.getBandCount() == 2, "x deletes its own row");

        // Tooltips exist for every column.
        bool allTips = true;
        for (int col = 0; col < L::kNumCols; ++col) allTips = allTips && !s.list.cellTooltipAt(s.cx(col), s.cy(0)).empty();
        check(allTips, "every column has a hover description");
    }

    std::printf("\n%s: %d failure(s)\n", g_fails ? "FAIL" : "PASS", g_fails);
    return g_fails ? 1 : 0;
}
