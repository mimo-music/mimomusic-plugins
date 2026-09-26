// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/stereo_heartbeat_test.cpp
//
// Compressor Stereo: the two sidechain-monitor spectrum analysers run only while the editor keeps sending
// kParamUiHeartbeat (its scope panel is on screen) -- same behaviour as PluginContour's check 16 in
// contour_trigger_test.cpp. Build/run from compressor/:
//   D=$PWD; g++ -std=gnu++17 -O1 -fno-access-control -DDGL_USE_FILE_BROWSER -DDISTRHO_PLUGIN_TARGET_LV2 \
//     -I$D/dpf/distrho -I$D/plugin_stereo -I$D/plugin/dsp -I$D/plugin -o /tmp/stereo_heartbeat_test \
//     tests/stereo_heartbeat_test.cpp plugin_stereo/PluginCompressorStereo.cpp plugin/dsp/SpectrumAnalyzer.cpp \
//     dpf/distrho/src/DistrhoPlugin.cpp && /tmp/stereo_heartbeat_test
#include "DistrhoPlugin.hpp"
#include "PluginCompressorStereo.hpp"
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }
using namespace DISTRHO;

static double spectrumSum(bool beat, bool stopEarly)
{
    d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
    std::unique_ptr<Plugin> p(createPlugin());
    p->activate();
    const uint32_t N = 512;
    std::vector<float> in(N), oL(N), oR(N);
    for (int blk = 0; blk < 400; ++blk)
    {
        if (beat && !(stopEarly && blk > 20)) p->setParameterValue(kParamUiHeartbeat, float(blk + 1));
        for (uint32_t i = 0; i < N; ++i) in[i] = blk < 250 ? float(0.3 * std::sin(2.0 * M_PI * 1000.0 * double(blk * N + i) / 48000.0)) : 0.0f;
        const float* ins[4] = { in.data(), in.data(), in.data(), in.data() }; // the scope reads the sidechain input (indices 2/3)
        float* outs[2] = { oL.data(), oR.data() };
        p->run(ins, outs, N);
    }
    double sum = 0.0;
    for (uint32_t k = 0; k < sscomp::SpectrumAnalyzer::kNumPackedBands; ++k)
        sum += std::fabs(double(p->getParameterValue(kParamMeterScSpectrumBase + k))) + std::fabs(double(p->getParameterValue(kParamMeterScSpectrumInBase + k)));
    return sum;
}

int main()
{
    int fails = 0;
    const double never = spectrumSum(false, false), running = spectrumSum(true, false), lapsed = spectrumSum(true, true);
    bool ok = lapsed > 1.0 && never < 1e-3;
    std::printf("no heartbeat: analysers never run (%.1f); with one they do (%.1f): %s\n", never, lapsed, ok ? "OK" : "FAIL"); if (!ok) ++fails;
    ok = lapsed > 0.0 && std::fabs(lapsed - running) > 0.05 * std::fmax(lapsed, running);
    std::printf("heartbeat stopped after ~0.2 s: analysers stop ~2 s later (frozen %.1f vs always-on %.1f): %s\n", lapsed, running, ok ? "OK" : "FAIL"); if (!ok) ++fails;
    std::printf("\n%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
