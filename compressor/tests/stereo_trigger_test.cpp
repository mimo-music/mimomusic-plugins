// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/stereo_trigger_test.cpp
//
// Compressor Stereo: the sidechain options (Sidechain Gain, the Highpass/Lowpass band filter, the monitor
// scope) act on the trigger signal -- the external key input with Sidechain on, the main signal otherwise
// (same as Contour's trigger, see contour_trigger_test.cpp). Build/run from compressor/:
//   D=$PWD; g++ -std=gnu++17 -O1 -fno-access-control -DDGL_USE_FILE_BROWSER -DDISTRHO_PLUGIN_TARGET_LV2 \
//     -I$D/dpf/distrho -I$D/plugin_stereo -I$D/plugin/dsp -I$D/plugin -o /tmp/stereo_trigger_test \
//     tests/stereo_trigger_test.cpp plugin_stereo/PluginCompressorStereo.cpp plugin/dsp/SpectrumAnalyzer.cpp \
//     dpf/distrho/src/DistrhoPlugin.cpp && /tmp/stereo_trigger_test
// `/tmp/stereo_trigger_test dump` prints an output checksum per stereo mode instead, for comparing two
// builds bit for bit.
#include "DistrhoPlugin.hpp"
#include "PluginCompressorStereo.hpp"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }
using namespace DISTRHO;

struct Setup
{
    int stereoMode = 0;          // 0 Linked, 1 Dual-Mono, 2 Mid/Side
    bool sidechainExternal = false;
    bool keyConnected = false;   // key input carries a 3 kHz tone, else silence
    bool filterOn = false;
    float highpassHz = 20.0f;
    float scGainDb = 0.0f;
};

struct Result
{
    double scopeLevelDb;  // kParamMeterScLevelDbL at the end
    double grDb;          // kParamMeterGainReductionA at the end
    double outChecksum;   // sum of squares of both outputs
};

// Main input: a loud 100 Hz tone (L) / 150 Hz (R). Threshold -30 dB, ratio 8.
static Result render(const Setup& s)
{
    d_nextBufferSize = 512; d_nextSampleRate = 48000.0;
    std::unique_ptr<Plugin> p(createPlugin());
    p->setState("stereoMode", s.stereoMode == 0 ? "0" : s.stereoMode == 1 ? "1" : "2");
    p->setParameterValue(kParamThreshold, -30.0f);
    p->setParameterValue(kParamRatio, 8.0f);
    p->setParameterValue(kParamSidechainEnable, s.sidechainExternal ? 1.0f : 0.0f);
    p->setParameterValue(kParamSidechainGain, s.scGainDb);
    p->setParameterValue(kParamScFilterEnable, s.filterOn ? 1.0f : 0.0f);
    p->setParameterValue(kParamScFilterHighpassHz, s.highpassHz);
    p->activate();
    const uint32_t N = 512;
    std::vector<float> inL(N), inR(N), key(N), silence(N, 0.0f), oL(N), oR(N);
    double sum = 0.0;
    for (int blk = 0; blk < 200; ++blk)
    {
        p->setParameterValue(kParamUiHeartbeat, float(blk + 1));
        for (uint32_t i = 0; i < N; ++i)
        {
            const double t = double(blk * N + i) / 48000.0;
            inL[i] = float(0.5 * std::sin(2.0 * M_PI * 100.0 * t));
            inR[i] = float(0.4 * std::sin(2.0 * M_PI * 150.0 * t));
            key[i] = float(0.5 * std::sin(2.0 * M_PI * 3000.0 * t));
        }
        const float* k = s.keyConnected ? key.data() : silence.data();
        const float* ins[4] = { inL.data(), inR.data(), k, k };
        float* outs[2] = { oL.data(), oR.data() };
        p->run(ins, outs, N);
        for (uint32_t i = 0; i < N; ++i) sum += double(oL[i]) * oL[i] + double(oR[i]) * oR[i];
    }
    return { p->getParameterValue(kParamMeterScLevelDbL), p->getParameterValue(kParamMeterGainReductionA), sum };
}

static int fails = 0;
static void check(bool ok, const char* what, double a, double b)
{
    std::printf("%s (%.2f / %.2f): %s\n", what, a, b, ok ? "OK" : "FAIL");
    if (!ok) ++fails;
}

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "dump") == 0)
    {
        // Default sidechain settings: the audio must be identical to builds before this change.
        for (int mode = 0; mode < 3; ++mode)
            for (int ext = 0; ext < 2; ++ext)
            {
                Setup s; s.stereoMode = mode; s.sidechainExternal = ext; s.keyConnected = ext;
                std::printf("mode %d external %d: %.17g\n", mode, ext, render(s).outChecksum);
            }
        return 0;
    }

    for (int mode = 0; mode < 3; ++mode)
    {
        std::printf("-- stereo mode %d --\n", mode);
        Setup internal; internal.stereoMode = mode;
        const Result r = render(internal);
        check(r.scopeLevelDb > -20.0, "Internal, no key connected: the scope shows the main signal", r.scopeLevelDb, -20.0);
        check(r.grDb < -3.0, "Internal: the main signal is compressed", r.grDb, -3.0);

        Setup filtered = internal; filtered.filterOn = true; filtered.highpassHz = 2000.0f;
        const Result f = render(filtered);
        check(f.grDb > r.grDb + 3.0, "Internal + 2 kHz highpass: the 100/150 Hz tones barely trigger any more", f.grDb, r.grDb);

        Setup gained = internal; gained.scGainDb = -40.0f;
        const Result g = render(gained);
        check(g.grDb > -0.5, "Internal + -40 dB Sidechain Gain: no compression", g.grDb, -0.5);

        Setup ext = internal; ext.sidechainExternal = true;
        const Result e = render(ext);
        check(e.grDb > -0.5 && e.scopeLevelDb < -60.0, "External, key silent: no compression, scope silent", e.grDb, e.scopeLevelDb);

        Setup extKey = ext; extKey.keyConnected = true;
        const Result ek = render(extKey);
        check(ek.grDb < -3.0 && ek.scopeLevelDb > -20.0, "External, 3 kHz key: key drives compression and scope", ek.grDb, ek.scopeLevelDb);
    }
    std::printf("\n%s: %d failure(s)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
