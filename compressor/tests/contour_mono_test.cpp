// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/contour_mono_test.cpp
//
// Contour Mono is the same source as Contour built with CONTOUR_MONO (plugin_contour_mono/DistrhoPluginInfo.h).
// This drives the real PluginContour::run() of that build: 2 inputs (main, sidechain) -> 1 output, and checks
// that the shared engine behaves on one channel exactly like the stereo plugin's Linked mode does.
//
// Build/run from compressor/ (plugin_contour_mono FIRST on the include path, plugin_contour/ not at all):
//   D=$PWD; g++ -std=gnu++17 -O1 -fno-access-control -DDGL_USE_FILE_BROWSER -DDISTRHO_PLUGIN_TARGET_LV2 \
//     -I$D/dpf/distrho -I$D/plugin_contour_mono -I$D/plugin/dsp -I$D/plugin -o /tmp/contour_mono_test \
//     tests/contour_mono_test.cpp plugin_contour/PluginContour.cpp plugin/dsp/SpectrumAnalyzer.cpp \
//     dpf/distrho/src/DistrhoPlugin.cpp && /tmp/contour_mono_test
#include "DistrhoPlugin.hpp"
#include "../plugin_contour/PluginContour.hpp"
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }
using namespace DISTRHO;

static int fails = 0;
static void check(bool ok, const char* what, double a, double b)
{ std::printf("%-96s %9.3f %9.3f  %s\n", what, a, b, ok ? "OK" : "FAIL"); if (!ok) ++fails; }

static std::unique_ptr<Plugin> make()
{
    d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
    std::unique_ptr<Plugin> p(createPlugin());
    p->activate();
    return p;
}

// Runs `blocks` blocks of a 1 kHz tone (peak `amp`) on the main input; the sidechain input carries `scAmp`.
static double runTone(Plugin& p, double amp, double scAmp, int blocks, std::vector<float>* outAll = nullptr)
{
    const uint32_t N = 512;
    std::vector<float> in(N), sc(N), out(N);
    double sumSq = 0.0; size_t cnt = 0;
    for (int blk = 0; blk < blocks; ++blk)
    {
        for (uint32_t i = 0; i < N; ++i)
        {
            const double t = double(blk * N + i) / 48000.0;
            in[i] = float(amp * std::sin(2.0 * M_PI * 1000.0 * t));
            sc[i] = float(scAmp * std::sin(2.0 * M_PI * 1000.0 * t));
        }
        const float* ins[2] = { in.data(), sc.data() }; // MONO layout: exactly two input buffers ...
        float* outs[1] = { out.data() };                // ... and exactly one output buffer
        p.run(ins, outs, N);
        if (outAll) outAll->insert(outAll->end(), out.begin(), out.end());
        if (blk >= blocks / 2) for (float v : out) { sumSq += double(v) * v; ++cnt; }
    }
    return 10.0 * std::log10(sumSq / double(cnt) + 1e-30);
}

static void setGateCurve(Plugin& p) // flat -60 dB whenever the trigger is met (a "gate slams shut" shape)
{
    p.setParameterValue(kParamThreshold, -30.0f);
    p.setParameterValue(kParamAttack, 0.5f); p.setParameterValue(kParamRelease, 20.0f);
    p.setParameterValue(kParamCurvePointCount, 2.0f);
    p.setParameterValue(kParamCurvePointInBase, -60.0f);  p.setParameterValue(kParamCurvePointInBase + 1, 6.0f);
    p.setParameterValue(kParamCurvePointOutBase, -60.0f); p.setParameterValue(kParamCurvePointOutBase + 1, -60.0f);
    p.setParameterValue(kParamMirrorCommit, 1.0f); p.setParameterValue(kParamMirrorCommitB, 1.0f);
}

int main()
{
    check(kContourMono && DISTRHO_PLUGIN_NUM_INPUTS == 2 && DISTRHO_PLUGIN_NUM_OUTPUTS == 1,
          "this is the mono build: kContourMono, 2 inputs (main + sidechain), 1 output", double(DISTRHO_PLUGIN_NUM_INPUTS), double(DISTRHO_PLUGIN_NUM_OUTPUTS));

    {   // ports
        auto p = make();
        AudioPort in0, in1, out0;
        p->initAudioPort(true, 0, in0); p->initAudioPort(true, 1, in1); p->initAudioPort(false, 0, out0);
        check((in1.hints & kAudioPortIsSidechain) && !(in0.hints & kAudioPortIsSidechain), "port 1 is the sidechain, port 0 the main input", double(in0.hints), double(in1.hints));
    }
    {   // identity curve passes the tone untouched (-20 dB peak -> -23 dB rms)
        auto p = make();
        const double db = runTone(*p, 0.1, 0.0, 12);
        check(std::fabs(db - (-23.0)) < 0.5, "default (identity) curve: a -20 dB tone passes at -23 dB rms", db, -23.0);
    }
    {   // gate: loud tone above the threshold is shut, quiet one passes
        auto p = make(); setGateCurve(*p);
        const double loud = runTone(*p, 0.1, 0.0, 12);
        auto q = make(); setGateCurve(*q);
        const double quiet = runTone(*q, 0.003, 0.0, 12);
        check(loud < -23.0 - 15.0 && std::fabs(quiet - (-53.5)) < 1.5, "gate curve: the loud tone (trigger met) is shut, the -50 dB tone passes", loud, quiet);
    }
    {   // external sidechain: the trigger comes from input 1, the audio from input 0
        auto p = make(); setGateCurve(*p); p->setParameterValue(kParamSidechainEnable, 1.0f);
        const double quietKey = runTone(*p, 0.1, 0.001, 12);   // key below the threshold: audio passes
        auto q = make(); setGateCurve(*q); q->setParameterValue(kParamSidechainEnable, 1.0f);
        const double loudKey = runTone(*q, 0.1, 0.3, 12);      // key above the threshold: audio is shut
        check(std::fabs(quietKey - (-23.0)) < 1.0 && loudKey < -23.0 - 15.0, "sidechain trigger: quiet key lets the audio pass, loud key shuts it", quietKey, loudKey);
    }
    {   // a stereo session's Dual-Mono/Mid-Side state must not change a mono plugin
        auto p = make(); p->setState("stereoMode", "1"); p->setState("stereoMode", "2");
        check(static_cast<PluginContour*>(p.get())->stereoMode_ == sscomp::StereoMode::Linked, "a saved stereoMode state (Dual-Mono / Mid-Side) is ignored: always Linked", 0, 0);
    }
    {   // bypass copies the single channel
        auto p = make(); p->setParameterValue(kParamBypass, 1.0f);
        const double db = runTone(*p, 0.1, 0.0, 4);
        check(std::fabs(db - (-23.0)) < 0.5, "bypass passes the mono input to the output", db, -23.0);
    }
    {   // lookahead delays the single channel exactly
        auto p = make(); p->setParameterValue(kParamLookahead, 5.0f);
        std::vector<float> out; runTone(*p, 0.5, 0.0, 4, &out);
        double maxErr = 0.0;
        for (size_t i = 240; i < out.size(); ++i) maxErr = std::max(maxErr, std::fabs(double(out[i]) - 0.5 * std::sin(2.0 * M_PI * 1000.0 * double(i - 240) / 48000.0)));
        check(maxErr < 1e-5, "lookahead 5 ms: the mono output is the input delayed by exactly 240 samples", maxErr, 0.0);
    }
    {   // the R output does not exist: run() must not touch a second buffer (a canary buffer stays intact)
        auto p = make();
        const uint32_t N = 512; std::vector<float> in(N, 0.1f), sc(N, 0.0f), out(N), canary(N, 123.0f);
        const float* ins[2] = { in.data(), sc.data() }; float* outs[2] = { out.data(), canary.data() };
        p->run(ins, outs, N);
        bool untouched = true; for (float v : canary) if (v != 123.0f) untouched = false;
        check(untouched, "run() never writes a second output buffer", untouched ? 1 : 0, 1);
    }

    std::printf("\n%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
