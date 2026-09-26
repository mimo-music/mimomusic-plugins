// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// contour_trigger_test.cpp
//
// Drives the REAL PluginContour::run() (not just CurveDsp.hpp) to verify the
// Trigger Input options -- Trigger Gain and the Highpass/Lowpass Trigger
// Filter -- act on the plugin's own INTERNAL signal too, not only the
// external Sidechain input (e.g. gates that open and close at certain
// frequencies).
//
// Not part of the CMake build (it needs DPF's Plugin base class plus
// -fno-access-control to poke the protected run()/activate()/
// setParameterValue()); build and run it by hand from the project root:
//
//   D=$PWD; g++ -std=gnu++17 -O1 -fno-access-control -DDGL_USE_FILE_BROWSER \
//     -DDISTRHO_PLUGIN_TARGET_LV2 -I$D/dpf/distrho -I$D/plugin_contour \
//     -I$D/plugin/dsp -I$D/plugin -o /tmp/contour_trigger_test \
//     tests/contour_trigger_test.cpp plugin_contour/PluginContour.cpp \
//     plugin/dsp/SpectrumAnalyzer.cpp dpf/distrho/src/DistrhoPlugin.cpp \
//   && /tmp/contour_trigger_test
//
// Note the Trigger Filter is 4 cascaded one-pole stages (24dB/oct), so test
// tones are chosen well inside/outside the band, not on a corner's skirt.

// Standalone harness: drives the real PluginContour::run() to verify the
// trigger gain/filter now act on the INTERNAL signal too.
#include "DistrhoPlugin.hpp"
#include "PluginContour.hpp"
#include <cmath>
#include <cstdio>
#include <vector>
#include <memory>

START_NAMESPACE_DISTRHO
Plugin* createPlugin();
END_NAMESPACE_DISTRHO

using namespace DISTRHO;
namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }

static double rmsDb(const std::vector<float>& v, size_t from)
{
    double s = 0; size_t n = 0;
    for (size_t i = from; i < v.size(); ++i) { s += double(v[i]) * v[i]; ++n; }
    return 10.0 * std::log10(s / double(n) + 1e-30);
}

// Sine of `hz` at amplitude `amp`, run through a gate-like curve: output far
// below input whenever the trigger condition is met.
static double runGate(double hz, bool filterOn, double hpHz, double lpHz, double trigGainDb, double ampDb = -20.0, int action = 0)
{
    d_nextBufferSize = 8192; d_nextSampleRate = 48000.0;
    std::unique_ptr<Plugin> p(createPlugin());
    const double sr = 48000.0;
    p->activate();
    auto set = [&](uint32_t idx, float v) { p->setParameterValue(idx, v); };
    set(kParamThreshold, -30.0f);      // trigger Above -30 dB
    set(kParamAttack, 0.5f); set(kParamRelease, 20.0f);
    set(kParamScFilterEnable, filterOn ? 1.0f : 0.0f);
    set(kParamScFilterHighpassHz, float(hpHz));
    set(kParamScFilterLowpassHz, float(lpHz));
    set(kParamSidechainGain, float(trigGainDb));
    set(kParamSidechainEnable, 0.0f);  // INTERNAL trigger
    // Curve: flat at -60 dB (the "gate slams shut" shape) whenever triggered.
    set(kParamCurvePointCount, 2.0f);
    set(kParamCurvePointInBase, -60.0f);  set(kParamCurvePointInBase + 1, 6.0f);
    set(kParamCurvePointOutBase, -60.0f); set(kParamCurvePointOutBase + 1, -60.0f);
    set(kParamCurveTriggerCondition, sscomp::packConditionAndThresholdLink(0, 0, action));
    set(kParamMirrorCommit, 1.0f); set(kParamMirrorCommitB, 1.0f);

    const uint32_t N = 8192;
    std::vector<float> inL(N), inR(N), sc(N, 0.0f), outL(N), outR(N);
    std::vector<float> all;
    const double amp = std::pow(10.0, ampDb / 20.0); // sine peak level
    for (int blk = 0; blk < 12; ++blk)
    {
        for (uint32_t i = 0; i < N; ++i)
            inL[i] = inR[i] = float(amp * std::sin(2.0 * M_PI * hz * double(blk * N + i) / sr));
        const float* in[4] = { inL.data(), inR.data(), sc.data(), sc.data() };
        float* out[2] = { outL.data(), outR.data() };
        p->run(in, out, N);
        if (blk >= 6) all.insert(all.end(), outL.begin(), outL.end());
    }
    return rmsDb(all, 0);
}

int main()
{
    int fails = 0;
    auto check = [&](bool ok, const char* what, double a, double b)
    { std::printf("%-88s %8.2f %8.2f  %s\n", what, a, b, ok ? "OK" : "FAIL"); if (!ok) ++fails; };

    // "Ungated" reference: the level a -20 dB-peak sine has when nothing is applied (~ -23 dB rms).
    const double kPass = -23.0;

    // 1. Internal trigger, filter off: tone is above threshold -> gated hard.
    const double gated = runGate(1000, false, 100, 8000, 0);
    check(gated < kPass - 15.0, "internal trigger, filter off, 1kHz: gated (>15 dB down)", gated, kPass - 15.0);

    // 2. HP at 4kHz: a 200 Hz tone barely reaches the detector -> not triggered -> passes untouched.
    const double passLow = runGate(200, true, 4000, 8000, 0);
    check(std::fabs(passLow - kPass) < 1.0, "internal trigger, HP 4kHz: 200Hz can't trigger -> passes untouched", passLow, kPass);

    // 3. Band 500Hz..12kHz: a 3kHz tone sits in the band -> triggers -> gated.
    const double gateBand = runGate(3000, true, 500, 12000, 0);
    check(gateBand < kPass - 15.0, "internal trigger, band 500Hz-12kHz: 3kHz tone triggers -> gated", gateBand, kPass - 15.0);

    // 4. Same band, 100 Hz tone outside it -> passes.
    const double outBand = runGate(100, true, 500, 12000, 0);
    check(std::fabs(outBand - kPass) < 1.0, "internal trigger, band 500Hz-12kHz: 100Hz tone outside -> passes untouched", outBand, kPass);

    // 5. Trigger Gain -24 dB: -20 dB tone reaches the detector at -44 dB (< -30 thr) -> not triggered.
    const double lowGain = runGate(1000, false, 100, 8000, -24);
    check(std::fabs(lowGain - kPass) < 1.0, "internal trigger, Trigger Gain -24 dB: below threshold -> passes untouched", lowGain, kPass);

    // 6. A quiet -40 dB tone is below the -30 dB threshold: untouched with 0 dB Trigger Gain...
    const double quietPass = runGate(1000, false, 100, 8000, 0, -40.0);
    check(std::fabs(quietPass - (-43.0)) < 1.0, "quiet -40dB tone, Trigger Gain 0: below threshold -> passes untouched", quietPass, -43.0);
    // ...but +24 dB Trigger Gain lifts it (for detection only) over the threshold -> gated.
    const double quietGated = runGate(1000, false, 100, 8000, 24, -40.0);
    check(quietGated < -43.0 - 15.0, "quiet -40dB tone, Trigger Gain +24: now triggers -> gated", quietGated, -43.0 - 15.0);

    // 7. The Parameter mirror carries a node's link group inside the existing per-point
    //    direction Parameter (value = direction + 3*link): set/get round-trips, both banks,
    //    and an old-session value (0..2) still reads back unchanged.
    {
        d_nextBufferSize = 8192; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin());
        p->setParameterValue(kParamCurvePointDirBase + 1, 2.0f + 3.0f * 4.0f);
        p->setParameterValue(kParamCurvePointDirBaseB + 2, 1.0f);
        const bool ok = p->getParameterValue(kParamCurvePointDirBase + 1) == 14.0f &&
                        p->getParameterValue(kParamCurvePointDirBaseB + 2) == 1.0f;
        check(ok, "Parameter mirror: direction+link packing round-trips (A: falling+group 4, B: rising, old-style value)",
              double(p->getParameterValue(kParamCurvePointDirBase + 1)), 14.0);
    }

    // 8. The threshold's link group rides in the trigger-condition Parameter (condition + 2*group).
    {
        d_nextBufferSize = 8192; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin());
        p->setParameterValue(kParamCurveTriggerCondition, 1.0f + 2.0f * 3.0f);
        p->setParameterValue(kParamCurveTriggerConditionB, 1.0f);
        const bool ok = p->getParameterValue(kParamCurveTriggerCondition) == 7.0f && p->getParameterValue(kParamCurveTriggerConditionB) == 1.0f;
        check(ok, "Parameter mirror: threshold link packs into the trigger-condition Parameter (Below + group 3; old value 1 unchanged)",
              double(p->getParameterValue(kParamCurveTriggerCondition)), 7.0);
    }

    // 9. The live-trail pair comes out of the real plugin as ONE Parameter and both halves are
    //    from the same sample: level and gain of a steady -20dB input through a "flat at -60dB"
    //    curve must satisfy level + gain == -60 (the curve's output), within the packing step.
    {
        d_nextBufferSize = 8192; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin());
        p->activate();
        p->setParameterValue(kParamThreshold, -30.0f);
        p->setParameterValue(kParamAttack, 0.0f); p->setParameterValue(kParamRelease, 0.0f);
        p->setParameterValue(kParamCurvePointCount, 2.0f);
        p->setParameterValue(kParamCurvePointInBase, -60.0f);  p->setParameterValue(kParamCurvePointInBase + 1, 6.0f);
        p->setParameterValue(kParamCurvePointOutBase, -60.0f); p->setParameterValue(kParamCurvePointOutBase + 1, -60.0f);
        p->setParameterValue(kParamMirrorCommit, 1.0f); p->setParameterValue(kParamMirrorCommitB, 1.0f);
        const uint32_t N = 512;
        std::vector<float> in(N, 0.1f), sc(N, 0.0f), oL(N), oR(N);
        const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() };
        float* outs[2] = { oL.data(), oR.data() };
        p->run(ins, outs, N);
        double level, gain;
        sscomp::unpackTrailPair(p->getParameterValue(kParamMeterTrailA), level, gain);
        check(std::fabs(level - (-20.0)) < 0.1 && std::fabs((level + gain) - (-60.0)) < 0.1,
              "trail pair from the real plugin: level -20dB and level+gain == curve output (-60dB), from one Parameter",
              level + gain, -60.0);
    }

    // 10. NO ALIASING between mirror banks: write a distinct value to EVERY slot of EVERY per-point
    //     mirror bank through the real setParameterValue(), then read every slot back. (The banks
    //     used to overlap by 17 slots -- direction/handle-X writes were swallowed by other banks;
    //     check 7 above passed anyway because set and get went through the same wrong branch.
    //     This test would have failed.)
    {
        d_nextBufferSize = 8192; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin());
        struct Bank { const char* name; uint32_t base; int kind; };
        const Bank banks[] = {
            { "in A", kParamCurvePointInBase, 0 },          { "out A", kParamCurvePointOutBase, 1 },
            { "in B", kParamCurvePointInBaseB, 2 },         { "out B", kParamCurvePointOutBaseB, 3 },
            { "dir A", kParamCurvePointDirBase, 4 },        { "dir B", kParamCurvePointDirBaseB, 5 },
            { "handle-in A", kParamCurvePointHandleInBase, 6 },  { "handle-out A", kParamCurvePointHandleOutBase, 7 },
            { "handle-in B", kParamCurvePointHandleInBaseB, 8 }, { "handle-out B", kParamCurvePointHandleOutBaseB, 9 },
        };
        auto valueFor = [](int kind, int slot) -> float {
            if (kind == 4 || kind == 5) return float((slot * 7 + kind) % 30);   // valid packed direction+link integers
            return float(kind * 100 + slot) * 0.25f;                              // distinct, exactly representable
        };
        for (const Bank& b : banks)
            for (int k = 0; k < int(kNumMirroredPoints); ++k) p->setParameterValue(b.base + k, valueFor(b.kind, k));
        int bad = 0; const char* firstBad = nullptr; int firstSlot = -1;
        for (const Bank& b : banks)
            for (int k = 0; k < int(kNumMirroredPoints); ++k)
                if (p->getParameterValue(b.base + k) != valueFor(b.kind, k)) { if (!bad) { firstBad = b.name; firstSlot = k; } ++bad; }
        char what[160];
        std::snprintf(what, sizeof(what), "all %d mirror slots read back exactly what was written (first mismatch: %s slot %d)",
                      int(sizeof(banks) / sizeof(banks[0])) * int(kNumMirroredPoints), firstBad ? firstBad : "none", firstSlot);
        check(bad == 0, what, double(bad), 0.0);
    }

    // 11. The trail BANK: kTrailBankSize (level, gain) pairs per block, evenly spaced through it. A rising ramp
    //     through a "flat at -60dB" curve must give 48 distinct, increasing levels, every pair on the curve
    //     (level + gain == -60), i.e. ~48x the points of the single last-sample pair.
    {
        d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin());
        p->activate();
        p->setParameterValue(kParamThreshold, -60.0f);
        p->setParameterValue(kParamAttack, 0.0f); p->setParameterValue(kParamRelease, 0.0f);
        p->setParameterValue(kParamCurvePointCount, 2.0f);
        p->setParameterValue(kParamCurvePointInBase, -60.0f);  p->setParameterValue(kParamCurvePointInBase + 1, 6.0f);
        p->setParameterValue(kParamCurvePointOutBase, -60.0f); p->setParameterValue(kParamCurvePointOutBase + 1, -60.0f);
        p->setParameterValue(kParamMirrorCommit, 1.0f); p->setParameterValue(kParamMirrorCommitB, 1.0f);
        const uint32_t N = 512;
        std::vector<float> in(N), sc(N, 0.0f), oL(N), oR(N);
        for (uint32_t i = 0; i < N; ++i) in[i] = float(0.01 + 0.5 * double(i) / double(N)); // rising ramp
        const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() };
        float* outs[2] = { oL.data(), oR.data() };
        p->run(ins, outs, N);
        int onCurve = 0, increasing = 0; double firstL = 0, lastL = 0;
        for (uint32_t k = 0; k < kTrailBankSize; ++k)
        {
            double L, G; sscomp::unpackTrailPair(p->getParameterValue(kParamMeterTrailBankA + k), L, G);
            if (std::fabs((L + G) - (-60.0)) < 0.15) ++onCurve;
            if (k == 0) firstL = L; else if (L >= lastL - 1e-9) ++increasing;
            lastL = L;
        }
        char what[160];
        std::snprintf(what, sizeof(what), "trail bank: %u pairs per block, %d on the curve, %d/%u increasing, levels %.1f..%.1f dB",
                      kTrailBankSize, onCurve, increasing, kTrailBankSize - 1, firstL, lastL);
        check(onCurve == int(kTrailBankSize) && increasing == int(kTrailBankSize) - 1 && lastL > firstL + 20.0, what, double(onCurve), double(kTrailBankSize));
    }

    // 12. Edits are delivered like DPF's LV2 wrapper does it: once per audio block, and ONLY ports whose value
    //     differs from the one it saw last (lv2_run: d_isNotEqual(fLastControlValues[i], curValue)), in ascending
    //     index order. A commit trigger that just alternates 0/1 is swallowed when an even number of edits lands
    //     in one block (net: no change) -> the DSP keeps the OLD curve although all data ports arrived. The GUI
    //     therefore sends a counter that increases with every edit. This test models the delivery and replays two
    //     edits inside one block, both ways.
    {
        struct HostModel
        {
            Plugin* plugin;
            std::vector<float> value, lastSeen;
            explicit HostModel(Plugin* p) : plugin(p), value(kParamCount, 0.0f), lastSeen(kParamCount, 0.0f) {}
            void write(uint32_t i, float v) { value[i] = v; }
            void deliver() // one audio block's worth of port reads
            {
                for (uint32_t i = 0; i < kParamCount; ++i)
                    if (value[i] != lastSeen[i]) { lastSeen[i] = value[i]; plugin->setParameterValue(i, value[i]); }
            }
        };
        auto runScenario = [&](bool counterCommit) -> double {
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            HostModel host(p.get());
            PluginContour* pc = static_cast<PluginContour*>(p.get());
            float toggle = 0.0f, counter = 0.0f;
            auto pushCurve = [&](double node1Out) { // what UIContour::pushCurveParamMirror() + pushMirrorCommit() write
                host.write(kParamCurvePointCount, 3.0f);
                const double in[3] = { -80.0, -30.0, 6.0 }, out[3] = { -80.0, node1Out, 6.0 };
                for (int i = 0; i < 3; ++i) { host.write(kParamCurvePointInBase + i, float(in[i])); host.write(kParamCurvePointOutBase + i, float(out[i])); }
                float v;
                if (counterCommit) { counter += 1.0f; v = counter; } else { toggle = 1.0f - toggle; v = toggle; }
                host.write(kParamMirrorCommit, v); host.write(kParamMirrorCommitB, v);
            };
            // block 1: one edit, delivered, curve becomes node1 -> -30
            pushCurve(-30.0); host.deliver();
            std::vector<float> in(512, 0.0f), sc(512, 0.0f), oL(512), oR(512);
            const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
            p->run(ins, outs, 512);
            // block 2: TWO edits arrive before the next block (a fast drag): -20 then -10
            pushCurve(-20.0); pushCurve(-10.0); host.deliver();
            p->run(ins, outs, 512);
            return pc->curve_.point(1).outputDb; // what the audio is really using; should be -10
        };
        // Safety net: data arrives but the commit is lost completely -> still applied after a few blocks.
        double netOnly = 1e9;
        {
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            PluginContour* pc = static_cast<PluginContour*>(p.get());
            p->setParameterValue(kParamCurvePointCount, 3.0f);
            const double in3[3] = { -80.0, -30.0, 6.0 }, out3[3] = { -80.0, -12.0, 6.0 };
            for (int i = 0; i < 3; ++i) { p->setParameterValue(kParamCurvePointInBase + i, float(in3[i])); p->setParameterValue(kParamCurvePointOutBase + i, float(out3[i])); }
            std::vector<float> z(512, 0.0f), oL(512), oR(512);
            const float* ins[4] = { z.data(), z.data(), z.data(), z.data() }; float* outs[2] = { oL.data(), oR.data() };
            for (int b = 0; b < 4; ++b) p->run(ins, outs, 512); // no commit ever sent
            netOnly = pc->curve_.point(1).outputDb;
        }
        check(std::fabs(netOnly - (-12.0)) < 1e-6, "safety net: mirror data with NO commit is applied on its own within a few blocks", netOnly, -12.0);
        const double toggled = runScenario(false), counted = runScenario(true);
        std::printf("    DSP curve node after two edits in one block: alternating 0/1 commit -> %.1f dB (stale), counter commit -> %.1f dB\n", toggled, counted);
        check(std::fabs(counted - (-10.0)) < 1e-6, "counter commit: the DSP curve follows the LAST of several edits in one block", counted, -10.0);
        check(std::fabs(toggled - (-10.0)) > 1.0, "(control) the old alternating 0/1 commit really does lose it -- the reported stale curve", toggled, -10.0);
    }

    // 14. Trigger action (open/close): the same flat -60 dB "shut" curve and the same trigger (Above -30 dB) -- with the
    //     action "Curve" the tone that triggers is shut; with "Diagonal" the tone that triggers passes and the quiet one
    //     (trigger not met) is shut instead.
    {
        const double loudCurve = runGate(1000, false, 100, 8000, 0, -20.0, 0);
        const double loudDiag = runGate(1000, false, 100, 8000, 0, -20.0, 1);
        const double quietCurve = runGate(1000, false, 100, 8000, 0, -50.0, 0);
        const double quietDiag = runGate(1000, false, 100, 8000, 0, -50.0, 1);
        check(loudCurve < kPass - 15.0 && std::fabs(quietCurve - (-53.0)) < 1.0,
              "action Curve: loud tone (trigger met) shut by the curve, quiet tone passes", loudCurve, quietCurve);
        check(std::fabs(loudDiag - kPass) < 1.0 && quietDiag < -53.0 - 8.0, // curve output -60 for a -50 tone: level-dependent closing, ~-10 dB here
              "action Diagonal: loud tone (trigger met) passes, quiet tone (not met) goes to the curve", loudDiag, quietDiag);
        d_nextBufferSize = 8192; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin());
        p->setParameterValue(kParamCurveTriggerCondition, 1.0f + 2.0f * (3.0f + 10.0f * 1.0f));
        p->setParameterValue(kParamCurveTriggerConditionB, 0.0f + 2.0f * (0.0f + 10.0f * 1.0f));
        const bool ok = p->getParameterValue(kParamCurveTriggerCondition) == 1.0f + 2.0f * 13.0f &&
                        p->getParameterValue(kParamCurveTriggerConditionB) == 20.0f;
        check(ok, "Parameter mirror: trigger action packs into the trigger-condition Parameter (Below + group 3 + Diagonal)",
              double(p->getParameterValue(kParamCurveTriggerCondition)), 27.0);
    }

    // 15. Lookahead (Parameter kParamLookahead): the audio is delayed by exactly the lookahead (identity curve ->
    //     output = input shifted by L samples), the host is told the same latency, and a gate that needs a 2 ms
    //     attack is already shut when the burst that triggers it arrives (without lookahead the burst's first
    //     milliseconds leak through).
    {
        const double sr = 48000.0;
        auto makePlugin = [&](double lookaheadMs, double attackMs) {
            d_nextBufferSize = 512; d_nextSampleRate = sr;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            p->setParameterValue(kParamThreshold, -30.0f);
            p->setParameterValue(kParamAttack, float(attackMs)); p->setParameterValue(kParamRelease, 0.0f);
            p->setParameterValue(kParamLookahead, float(lookaheadMs));
            return p;
        };
        // (a) identity curve: pure delay
        {
            auto p = makePlugin(5.0, 0.0);
            const uint32_t N = 512; const uint32_t L = 240;
            std::vector<float> in(N), sc(N, 0.0f), oL(N), oR(N), all;
            std::vector<float> src;
            for (int blk = 0; blk < 4; ++blk)
            {
                for (uint32_t i = 0; i < N; ++i) in[i] = float(0.5 * std::sin(2.0 * M_PI * 440.0 * double(blk * N + i) / sr));
                src.insert(src.end(), in.begin(), in.end());
                const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
                p->run(ins, outs, N);
                all.insert(all.end(), oL.begin(), oL.end());
            }
            double maxErr = 0.0;
            for (size_t i = L; i < all.size(); ++i) maxErr = std::max(maxErr, double(std::fabs(all[i] - src[i - L])));
            check(maxErr < 1e-6 && static_cast<PluginContour*>(p.get())->reportedLatency_ == L, "lookahead 5 ms, identity curve: output = input delayed by exactly 240 samples, latency reported 240",
                  maxErr, double(static_cast<PluginContour*>(p.get())->reportedLatency_));
        }
        // (b) gate leak in the first 1 ms of a burst
        auto leak = [&](double lookaheadMs) {
            auto p = makePlugin(lookaheadMs, 2.0);
            p->setParameterValue(kParamCurvePointCount, 2.0f);
            p->setParameterValue(kParamCurvePointInBase, -60.0f);  p->setParameterValue(kParamCurvePointInBase + 1, 6.0f);
            p->setParameterValue(kParamCurvePointOutBase, -60.0f); p->setParameterValue(kParamCurvePointOutBase + 1, -60.0f);
            p->setParameterValue(kParamMirrorCommit, 1.0f); p->setParameterValue(kParamMirrorCommitB, 1.0f);
            const uint32_t N = 512;
            std::vector<float> in(N), sc(N, 0.0f), oL(N), oR(N), all;
            const uint32_t burstStart = 2000;
            for (int blk = 0; blk < 12; ++blk)
            {
                for (uint32_t i = 0; i < N; ++i)
                {
                    const uint32_t t = blk * N + i;
                    in[i] = t >= burstStart ? float(0.3 * std::sin(2.0 * M_PI * 1000.0 * double(t) / sr)) : 0.0f;
                }
                const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
                p->run(ins, outs, N);
                all.insert(all.end(), oL.begin(), oL.end());
            }
            const uint32_t play = burstStart + uint32_t(lookaheadMs * 0.001 * sr + 0.5); // where the burst reaches the output
            double peak = 0.0;
            for (uint32_t i = play; i < play + 48; ++i) peak = std::max(peak, double(std::fabs(all[i])));
            return peak;
        };
        const double leakNo = leak(0.0), leakLa = leak(5.0);
        check(leakLa < leakNo * 0.1 && leakNo > 0.01, "gate with 2 ms attack: the burst's first ms leaks without lookahead, is shut with 5 ms lookahead",
              leakLa, leakNo);
    }

    // 16. UI heartbeat: the two spectrum analysers run only while the editor sends kParamUiHeartbeat (its counter
    //     must keep changing), and stop ~2 s after it does not -- the CPU a closed window / hidden scope panel saves.
    {
        auto spectrumSum = [&](bool beat, bool stopEarly) {
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            const uint32_t N = 512;
            std::vector<float> in(N), sc(N, 0.0f), oL(N), oR(N);
            const int blocks = 400; // ~4.3 s
            for (int blk = 0; blk < blocks; ++blk)
            {
                if (beat && !(stopEarly && blk > 20)) p->setParameterValue(kParamUiHeartbeat, float(blk + 1));
                for (uint32_t i = 0; i < N; ++i) in[i] = blk < 250 ? float(0.3 * std::sin(2.0 * M_PI * 1000.0 * double(blk * N + i) / 48000.0)) : 0.0f; // tone, then silence
                const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
                p->run(ins, outs, N);
            }
            double sum = 0.0;
            for (uint32_t k = 0; k < sscomp::SpectrumAnalyzer::kNumPackedBands; ++k)
                sum += std::fabs(double(p->getParameterValue(kParamMeterScSpectrumBase + k))) + std::fabs(double(p->getParameterValue(kParamMeterScSpectrumInBase + k)));
            return sum;
        };
        const double never = spectrumSum(false, false), running = spectrumSum(true, false), lapsed = spectrumSum(true, true);
        check(lapsed > 1.0 && never < 1e-3, "no heartbeat: the spectrum analysers never run (packed output stays empty); with one they do", never, lapsed);
        // the tone stops at block 250; an analyser still running sees the silence, one that timed out (heartbeat gone since block 20) keeps the tone's spectrum
        check(lapsed > 0.0 && std::fabs(lapsed - running) > 0.05 * std::max(lapsed, running), "heartbeat stopped after ~0.2 s: analysers stop ~2 s later (result differs from always-on)", lapsed, running);
    }

    // 18. The model, end to end (the reported case): a flat -60 curve, threshold "never" / "always" reached, both On Trigger
    //     settings, at a normal AND at a heavily lowered Trigger Gain (trigger level far below the curve's -60 floor). The dots
    //     jump between the curve (y = -60) and the diagonal (y = x) exactly as the two settings say -- the trigger's gain
    //     never changes where the audio sits on the curve.
    {
        auto dot = [&](double thr, int cond, int action, double trigGainDb, double& x, double& y) {
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin()); p->activate();
            auto set = [&](uint32_t i, float v) { p->setParameterValue(i, v); };
            set(kParamThreshold, float(thr)); set(kParamAttack, 0.0f); set(kParamRelease, 0.0f); set(kParamSidechainGain, float(trigGainDb));
            set(kParamCurveTriggerCondition, sscomp::packConditionAndThresholdLink(cond, 0, action));
            set(kParamCurvePointCount, 2.0f); set(kParamCurvePointInBase, -60.0f); set(kParamCurvePointInBase + 1, 6.0f);
            set(kParamCurvePointOutBase, -60.0f); set(kParamCurvePointOutBase + 1, -60.0f);
            set(kParamMirrorCommit, 1.0f); set(kParamMirrorCommitB, 1.0f);
            std::vector<float> in(512, float(std::pow(10.0, -20.0 / 20.0))), sc(512, 0.0f), oL(512), oR(512);
            const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
            for (int b = 0; b < 4; ++b) p->run(ins, outs, 512);
            double g; bool tr; sscomp::unpackTrailPair(p->getParameterValue(kParamMeterTrailBankA + 10), x, g, tr); y = x + g;
        };
        bool ok = true; double x = 0, y = 0;
        for (double tg : { 0.0, -48.0 })
        {
            dot(6.0, 0, 0, tg, x, y);   ok = ok && std::fabs(y - x) < 0.1;                            // never met + Curve    -> diagonal
            dot(6.0, 0, 1, tg, x, y);   ok = ok && std::fabs(y - (-60.0)) < 0.2;                      // never met + Diagonal -> the curve
            dot(-60.0, 0, 0, tg, x, y); ok = ok && (tg < -40.0 || std::fabs(y - (-60.0)) < 0.2);      // met (only with Trigger Gain 0) + Curve -> the curve
            dot(-60.0, 0, 1, tg, x, y); ok = ok && (tg < -40.0 || std::fabs(y - x) < 0.1);            // met + Diagonal -> diagonal
        }
        check(ok, "flat -60 curve: never/always triggered x Curve/Diagonal x Trigger Gain 0 / -48 dB: dots jump between the curve and the diagonal", ok ? 1 : 0, 1);
    }

    // 19. The reported case, through the real plugin and the trail bank: Attack = Release = 0, Trigger Gain 0, Main Input. A -6 dB sine
    //     runs through a threshold of -20 dB with a flat -60 curve, "On Trigger: Curve". Every dot of the bank is on the curve when its
    //     x is above the threshold, on the diagonal when it is below -- no dot under the threshold on the curve line.
    {
        d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
        std::unique_ptr<Plugin> p(createPlugin()); p->activate();
        auto set = [&](uint32_t i, float v) { p->setParameterValue(i, v); };
        set(kParamThreshold, -20.0f); set(kParamAttack, 0.0f); set(kParamRelease, 0.0f); set(kParamSidechainGain, 0.0f);
        set(kParamCurveTriggerCondition, sscomp::packConditionAndThresholdLink(0, 0, 0));
        set(kParamCurvePointCount, 2.0f); set(kParamCurvePointInBase, -80.0f); set(kParamCurvePointInBase + 1, 6.0f);
        set(kParamCurvePointOutBase, -60.0f); set(kParamCurvePointOutBase + 1, -60.0f);
        set(kParamMirrorCommit, 1.0f); set(kParamMirrorCommitB, 1.0f);
        const uint32_t N = 512; std::vector<float> in(N), sc(N, 0.0f), oL(N), oR(N);
        int above = 0, below = 0, wrong = 0;
        for (int blk = 0; blk < 200; ++blk)
        {
            for (uint32_t i = 0; i < N; ++i) in[i] = float(0.5 * std::sin(2.0 * M_PI * 220.0 * double(blk * N + i) / 48000.0));
            const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
            p->run(ins, outs, N);
            if (blk < 10) continue;
            for (uint32_t k = 0; k < kTrailBankSize; ++k)
            {
                double x, g; bool tr; sscomp::unpackTrailPair(p->getParameterValue(kParamMeterTrailBankA + k), x, g, tr);
                if (x < -79.0) continue;
                const double y = x + g;
                if (x > -20.0) { ++above; if (std::fabs(y - (-60.0)) > 0.3 || !tr) ++wrong; }
                else { ++below; if (std::fabs(y - x) > 0.3 || tr) ++wrong; }
            }
        }
        check(wrong == 0 && above > 500 && below > 500, "0/0, trigger = main input: dots above the threshold on the curve, below it on the diagonal (no strays)", double(wrong), double(above + below));
    }

    // 13. Trigger Gain up to +60dB: a NARROW band-pass leaves almost nothing of a signal in the detector, so it needs
    //     far more than the old +24dB. A -50dB, 1kHz tone cannot trigger a -10dB threshold at +24dB (trigger level -26),
    //     but does at +60dB (+10). The gate (flat -60 curve) then takes the tone to -60. And Listen, which plays the trigger
    //     signal amplified, stays bounded.
    {
        auto lowToneOutput = [&](double trigGainDb, bool listen, double* peakOut) -> double {
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            p->setParameterValue(kParamThreshold, -10.0f);
            p->setParameterValue(kParamAttack, 0.0f); p->setParameterValue(kParamRelease, 0.0f);
            p->setParameterValue(kParamSidechainGain, float(trigGainDb));
            p->setParameterValue(kParamScListenEnable, listen ? 1.0f : 0.0f);
            p->setParameterValue(kParamCurvePointCount, 2.0f);
            p->setParameterValue(kParamCurvePointInBase, -60.0f);  p->setParameterValue(kParamCurvePointInBase + 1, 6.0f);
            p->setParameterValue(kParamCurvePointOutBase, -60.0f); p->setParameterValue(kParamCurvePointOutBase + 1, -60.0f);
            p->setParameterValue(kParamMirrorCommit, 1.0f); p->setParameterValue(kParamMirrorCommitB, 2.0f);
            const uint32_t N = 512;
            std::vector<float> in(N), sc(N, 0.0f), oL(N), oR(N);
            double sumSq = 0.0, peak = 0.0; size_t cnt = 0;
            for (int blk = 0; blk < 40; ++blk)
            {
                for (uint32_t i = 0; i < N; ++i) in[i] = float(std::pow(10.0, -50.0 / 20.0) * std::sin(2.0 * M_PI * 1000.0 * double(blk * N + i) / 48000.0));
                const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
                p->run(ins, outs, N);
                if (blk >= 20) for (uint32_t i = 0; i < N; ++i) { sumSq += double(oL[i]) * oL[i]; peak = std::max(peak, double(std::fabs(oL[i]))); ++cnt; }
            }
            if (peakOut) *peakOut = peak;
            return 10.0 * std::log10(sumSq / double(cnt) + 1e-30);
        };
        const double passes24 = lowToneOutput(24.0, false, nullptr);   // not triggered: the tone passes at its own -53dB rms
        const double gated60 = lowToneOutput(60.0, false, nullptr);    // triggered: the flat -60dB curve takes the -50dB tone down to -60
        check(passes24 > -56.0 && gated60 < passes24 - 5.0, // Release 0 maps sample by sample: a flat -60 line gives a -60 dB square-ish wave (-60 rms), not -63
               "+60dB trigger gain lets a -50dB tone trigger the gate; +24dB could not",
              gated60, passes24 - 5.0);
        double listenPeak = 0.0;
        lowToneOutput(60.0, true, &listenPeak);                        // Listen at +60dB: -50dB tone * 1000 = +10dB, soft-limited
        double loudPeak = 0.0;
        {   // and a LOUD signal with +60dB of gain in Listen must still be bounded by the limiter (<= 1.0 before Global Output Gain)
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            p->setParameterValue(kParamSidechainGain, 60.0f);
            p->setParameterValue(kParamScListenEnable, 1.0f);
            std::vector<float> in(512), sc(512, 0.0f), oL(512), oR(512);
            for (int i = 0; i < 512; ++i) in[i] = float(0.5 * std::sin(2.0 * M_PI * 200.0 * i / 48000.0));
            const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
            for (int blk = 0; blk < 4; ++blk) p->run(ins, outs, 512);
            for (int i = 0; i < 512; ++i) loudPeak = std::max(loudPeak, double(std::fabs(oL[i])));
        }
        check(loudPeak <= 1.0001 && loudPeak > 0.5, "Listen with +60dB trigger gain on a loud signal stays bounded (soft-limited, peak <= 1.0)", loudPeak, 1.0);
    }

    // 14. What the trail dots show, and the per-dot "triggered" flag. A steady -20 dB input with +24 dB Trigger Gain
    //     (trigger level +4 dB), flat -60 dB curve, threshold -30 dB Above: x = the audio's level (-20), y = the curve's
    //     -60, flagged triggered (+4 > -30); with the threshold above the trigger level the dot is not flagged and sits
    //     on the diagonal.
    {
        auto readDot = [&](bool detectorMode, double thresholdDb, double& x, double& y, bool& trig) {
            d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
            std::unique_ptr<Plugin> p(createPlugin());
            p->activate();
            p->setParameterValue(kParamThreshold, float(thresholdDb));
            p->setParameterValue(kParamAttack, 0.0f); p->setParameterValue(kParamRelease, 0.0f);
            p->setParameterValue(kParamSidechainGain, 24.0f);
            p->setParameterValue(kParamTrailSource, detectorMode ? 1.0f : 0.0f);
            p->setParameterValue(kParamCurvePointCount, 2.0f);
            p->setParameterValue(kParamCurvePointInBase, -60.0f);  p->setParameterValue(kParamCurvePointInBase + 1, 6.0f);
            p->setParameterValue(kParamCurvePointOutBase, -60.0f); p->setParameterValue(kParamCurvePointOutBase + 1, -60.0f);
            p->setParameterValue(kParamMirrorCommit, 1.0f); p->setParameterValue(kParamMirrorCommitB, 2.0f);
            std::vector<float> in(512, float(std::pow(10.0, -20.0 / 20.0))), sc(512, 0.0f), oL(512), oR(512);
            const float* ins[4] = { in.data(), in.data(), sc.data(), sc.data() }; float* outs[2] = { oL.data(), oR.data() };
            for (int b = 0; b < 3; ++b) p->run(ins, outs, 512);
            double g;
            sscomp::unpackTrailPair(p->getParameterValue(kParamMeterTrailBankA + 10), x, g, trig);
            y = x + g;
        };
        double x, y; bool trig;
        readDot(false, -30.0, x, y, trig);
        // The curve maps the AUDIO: the -20 dB input goes to the curve's -60 dB, whatever the trigger's gain does to the
        // trigger (its level +4 dB only decides that the trigger is met). The dot is ON the drawn curve.
        check(std::fabs(x - (-20.0)) < 0.1 && std::fabs(y - (-60.0)) < 0.2 && trig,
              "dot x = the audio's level (-20), y = the curve's output (-60), flagged triggered -- with +24 dB Trigger Gain", x, y);
        readDot(true, -30.0, x, y, trig); // the old "detector view" request is obsolete and ignored
        check(std::fabs(x - (-20.0)) < 0.1 && std::fabs(y - (-60.0)) < 0.2 && trig,
              "the obsolete detector-dots Parameter changes nothing (the dot stays on the audio's level and the curve)", x, y);
        readDot(false, 10.0, x, y, trig);
        check(!trig && std::fabs(y - x) < 0.1,
              "threshold above the detector level: the dot is not flagged and sits on the unity diagonal (out = in)", y - x, 0.0);
    }

    std::printf("\n%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
