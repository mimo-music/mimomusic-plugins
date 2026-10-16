// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// limiter_test.cpp
//
// Drives the REAL PluginLimiter::run() to check what the Limiter promises:
// the output never exceeds the Ceiling, whatever Output Gain/Boost ask for;
// with the capped post gain the clip stage stays idle in True Peak mode;
// Soft clipping rounds Classic's overshoot below the Ceiling; Oversampling
// limits inter-sample peaks (measured against a 16x reference); the reported
// latency matches the real delay, Bypass included. Also times the loop with
// Oversampling off and on.
//
// Not part of the CMake build (it needs DPF's Plugin base class plus
// -fno-access-control); build and run it by hand from the project root:
//
//   D=$PWD; g++ -std=gnu++17 -O2 -fno-access-control -DDGL_USE_FILE_BROWSER \
//     -DDISTRHO_PLUGIN_TARGET_LV2 -I$D/dpf/distrho -I$D/dpf/distrho/src -I$D/plugin_limiter \
//     -I$D/plugin/dsp -o /tmp/limiter_test \
//     tests/limiter_test.cpp plugin_limiter/PluginLimiter.cpp dpf/distrho/src/DistrhoPlugin.cpp \
//   && /tmp/limiter_test

#include "DistrhoPlugin.hpp"
#include "DistrhoPluginInternal.hpp"
#include "PluginLimiter.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <vector>

START_NAMESPACE_DISTRHO
Plugin* createPlugin();
END_NAMESPACE_DISTRHO

using namespace DISTRHO;
namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }

static constexpr double kSr = 48000.0;
static constexpr uint32_t kBlock = 512;

static double toDb(double lin) { return 20.0 * std::log10(std::max(1e-12, lin)); }

struct Settings
{
    float thresholdDb = -6.0f, ceilingDb = -1.0f, outputGainDb = 0.0f, attackMs = 1.0f, lookaheadMs = 5.0f;
    bool boost = false, truePeak = true, soft = false, oversampling = false, bypass = false;
    float clipKneeDb = 1.0f;
    int ratio = 0; // 0 = 20:1, 1 = 100:1, 2 = infinity:1 (Classic only)
};

struct Result
{
    std::vector<float> outL, outR;
    double maxClipDb = 0.0; // most negative kParamMeterClip seen
    uint32_t latency = 0;
    int delaySamples = 0;
};

static Result runPlugin(const Settings& s, const std::vector<float>& inL, const std::vector<float>& inR)
{
    d_nextBufferSize = kBlock; d_nextSampleRate = kSr;
    std::unique_ptr<Plugin> p(createPlugin());
    auto set = [&](uint32_t idx, float v) { p->setParameterValue(idx, v); };
    set(kParamThreshold, s.thresholdDb);
    set(kParamCeiling, s.ceilingDb);
    set(kParamOutputGain, s.outputGainDb);
    set(kParamAttack, s.attackMs);
    set(kParamLookahead, s.lookaheadMs);
    set(kParamBoost, s.boost ? 1.0f : 0.0f);
    set(kParamClipMode, s.soft ? 1.0f : 0.0f);
    set(kParamClipKnee, s.clipKneeDb);
    set(kParamOversampling, s.oversampling ? 1.0f : 0.0f);
    set(kParamBypass, s.bypass ? 1.0f : 0.0f);
    set(kParamRatio, float(s.ratio));
    p->setState("algorithm", s.truePeak ? "1" : "0");
    p->activate();

    Result r;
    const size_t n = inL.size();
    r.outL.resize(n); r.outR.resize(n);
    for (size_t off = 0; off < n; off += kBlock)
    {
        const uint32_t len = uint32_t(std::min<size_t>(kBlock, n - off));
        const float* in[2] = { inL.data() + off, inR.data() + off };
        float* out[2] = { r.outL.data() + off, r.outR.data() + off };
        p->run(in, out, len);
        r.maxClipDb = std::min(r.maxClipDb, double(p->getParameterValue(kParamMeterClip)));
    }
    r.latency = p->pData->latency;
    r.delaySamples = static_cast<PluginLimiter*>(p.get())->delaySamples_;
    return r;
}

static double samplePeak(const std::vector<float>& v, size_t from = 0)
{
    double m = 0.0;
    for (size_t i = from; i < v.size(); ++i) m = std::max(m, std::fabs(double(v[i])));
    return m;
}

// Reference inter-sample peak: 16x band-limited interpolation with a long
// (64 samples per side) Kaiser-windowed sinc -- far finer than the plugin's
// own 4x/12-tap detector, so it measures what that one may miss.
static double truePeak16x(const std::vector<float>& v, size_t from = 0)
{
    constexpr int kOs = 16, kHalf = 64;
    constexpr double kPi = 3.14159265358979323846, kBeta = 8.0;
    auto i0 = [](double x) { double s = 1, t = 1; for (int k = 1; k < 40; ++k) { t *= (x / (2 * k)) * (x / (2 * k)); s += t; } return s; };
    std::vector<std::vector<double>> taps(kOs, std::vector<double>(2 * kHalf));
    for (int ph = 1; ph < kOs; ++ph)
        for (int j = 0; j < 2 * kHalf; ++j)
        {
            const double u = double(ph) / kOs - double(j - (kHalf - 1));
            const double sinc = std::fabs(u) < 1e-12 ? 1.0 : std::sin(kPi * u) / (kPi * u);
            const double r = u / double(kHalf);
            taps[ph][j] = std::fabs(r) >= 1.0 ? 0.0 : sinc * i0(kBeta * std::sqrt(1 - r * r)) / i0(kBeta);
        }
    double m = samplePeak(v, from);
    for (size_t c = from + kHalf; c + kHalf < v.size(); ++c)
        for (int ph = 1; ph < kOs; ++ph)
        {
            double acc = 0.0;
            for (int j = 0; j < 2 * kHalf; ++j) acc += taps[ph][j] * double(v[c - (kHalf - 1) + j]);
            m = std::max(m, std::fabs(acc));
        }
    return m;
}

// Program-like test material: a few tones (one near the top of the band, so
// inter-sample peaks happen), noise, and sharp transients well over 0 dBFS.
static void makeProgram(std::vector<float>& l, std::vector<float>& r, size_t n, unsigned seed)
{
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.05);
    l.resize(n); r.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        const double t = double(i) / kSr;
        double x = 0.35 * std::sin(2 * M_PI * 110.0 * t) + 0.25 * std::sin(2 * M_PI * 1730.0 * t)
                 + 0.3 * std::sin(2 * M_PI * 11025.0 * t + 0.7) + noise(rng);
        if (i % 9600 < 96) x *= 4.0; // 2 ms bursts, +12 dB, every 200 ms
        l[i] = float(x);
        r[i] = float(0.8 * x + noise(rng));
    }
}

int main()
{
    int fails = 0;
    auto check = [&](bool ok, const char* what, double a, double b)
    { std::printf("%-92s %9.3f %9.3f  %s\n", what, a, b, ok ? "OK" : "FAIL"); if (!ok) ++fails; };

    std::vector<float> inL, inR;
    makeProgram(inL, inR, size_t(kSr * 3), 7);
    const size_t skip = size_t(kSr * 0.05); // ignore the first 50 ms (lookahead fill)

    // 1. True Peak, Hard, Boost on: Threshold -6, Ceiling -1 -> Boost is capped at +5 and the
    //    limited peaks land on the Ceiling, so the clip stage never has to act.
    {
        Settings s; s.boost = true;
        const Result r = runPlugin(s, inL, inR);
        const double pk = toDb(std::max(samplePeak(r.outL, skip), samplePeak(r.outR, skip)));
        check(pk <= -1.0 + 1e-4, "[1] TP+Hard+Boost: sample peak <= Ceiling -1 dB", pk, -1.0);
        check(pk >= -1.05, "[1] ... and the peaks actually reach the Ceiling (Boost not lost)", pk, -1.05);
        check(r.maxClipDb > -1e-3, "[1] ... clip stage never acted (max clip reduction, dB)", r.maxClipDb, 0.0);
    }
    // 2. Same with +10 dB Output Gain on top: still capped, still no clipping.
    {
        Settings s; s.boost = true; s.outputGainDb = 10.0f;
        const Result r = runPlugin(s, inL, inR);
        const double pk = toDb(std::max(samplePeak(r.outL, skip), samplePeak(r.outR, skip)));
        check(pk <= -1.0 + 1e-4 && r.maxClipDb > -1e-3, "[2] TP+Hard+Boost+Output +10: peak <= Ceiling, clip idle", pk, r.maxClipDb);
    }
    // 3. Ceiling below Threshold, no Boost: the limiter limits to the Ceiling instead.
    {
        Settings s; s.thresholdDb = -3.0f; s.ceilingDb = -6.0f;
        const Result r = runPlugin(s, inL, inR);
        const double pk = toDb(std::max(samplePeak(r.outL, skip), samplePeak(r.outR, skip)));
        check(pk <= -6.0 + 1e-4 && pk >= -6.05 && r.maxClipDb > -1e-3,
              "[3] TP: Ceiling -6 below Threshold -3 -> limited to -6, clip idle", pk, r.maxClipDb);
    }
    // 4. Classic + Hard, slow attack: overshoot exists and is clipped at the Ceiling.
    // 5. Classic + Soft: the same overshoot is rounded and stays below the Ceiling.
    {
        Settings s; s.truePeak = false; s.boost = true; s.attackMs = 5.0f; s.lookaheadMs = 1.0f;
        const Result hard = runPlugin(s, inL, inR);
        const double pkH = toDb(std::max(samplePeak(hard.outL, skip), samplePeak(hard.outR, skip)));
        check(pkH <= -1.0 + 1e-4 && hard.maxClipDb < -0.1, "[4] Classic+Hard: peak <= Ceiling, clipper acted (dB)", pkH, hard.maxClipDb);
        s.soft = true;
        const Result soft = runPlugin(s, inL, inR);
        const double pkS = toDb(std::max(samplePeak(soft.outL, skip), samplePeak(soft.outR, skip)));
        check(pkS <= -1.0 + 1e-4 && pkS > -2.0, "[5] Classic+Soft: peak <= Ceiling, above the knee start (-2)", pkS, -1.0);
        // Rounded, not flat-topped: count samples sitting on the Ceiling. This material drives Classic
        // ~17 dB into the clip stage, so tanh saturates onto the Ceiling (in float) on the biggest
        // overshoots even in Soft -- but far less often than Hard's flat tops.
        auto onCeiling = [&](const Result& r) {
            const double c = std::pow(10.0, -1.0 / 20.0); size_t k = 0;
            for (size_t i = skip; i < r.outL.size(); ++i) if (std::fabs(std::fabs(double(r.outL[i])) - c) < 1e-6) ++k;
            return double(k);
        };
        check(onCeiling(soft) * 10.0 < onCeiling(hard), "[5] ... samples flat on the Ceiling: Hard vs Soft (Soft < 1/10)", onCeiling(hard), onCeiling(soft));
    }
    // 6. Oversampling: inter-sample peaks. Without it the 16x true peak goes over the Ceiling;
    //    with it the overshoot is (nearly) gone.
    {
        Settings s; s.thresholdDb = -1.0f; s.ceilingDb = -1.0f;
        const Result off = runPlugin(s, inL, inR);
        s.oversampling = true;
        const Result on = runPlugin(s, inL, inR);
        const double tpOff = toDb(std::max(truePeak16x(off.outL, skip), truePeak16x(off.outR, skip)));
        const double tpOn = toDb(std::max(truePeak16x(on.outL, skip), truePeak16x(on.outR, skip)));
        const double spOn = toDb(std::max(samplePeak(on.outL, skip), samplePeak(on.outR, skip)));
        check(tpOff > -1.0 + 0.3, "[6] TP, Oversampling off: 16x true peak over the -1 dB Ceiling by > 0.3 dB", tpOff, -1.0);
        check(tpOn <= -1.0 + 0.1, "[6] TP, Oversampling on: 16x true peak within 0.1 dB of the Ceiling", tpOn, -1.0);
        check(spOn <= -1.0 + 1e-4, "[6] ... sample peak still <= Ceiling", spOn, -1.0);
    }
    // 7. Fs/4 sine at 45 deg phase: samples at 0.707 of the true peak (+3 dB inter-sample).
    {
        std::vector<float> l(48000), r(48000);
        for (size_t i = 0; i < l.size(); ++i) l[i] = r[i] = float(std::sin(M_PI / 2 * double(i) + M_PI / 4)); // true peak 0 dBFS
        Settings s; s.thresholdDb = -1.0f; s.ceilingDb = -1.0f; s.oversampling = true;
        const Result on = runPlugin(s, l, r);
        const double tpOn = toDb(truePeak16x(on.outL, skip));
        check(std::fabs(tpOn - (-1.0)) < 0.1, "[7] Fs/4 sine, true peak 0 dBFS, Oversampling on: true peak ~ Ceiling -1", tpOn, -1.0);
    }
    // 8. Latency: an impulse comes out exactly `latency` samples later -- Oversampling off/on, Bypass.
    {
        std::vector<float> l(4096, 0.0f), r(4096, 0.0f);
        l[1000] = r[1000] = 0.01f; // far below threshold
        for (int mode = 0; mode < 4; ++mode)
        {
            Settings s; s.oversampling = (mode & 1) != 0; s.bypass = (mode & 2) != 0;
            const Result res = runPlugin(s, l, r);
            size_t at = 0;
            for (size_t i = 0; i < res.outL.size(); ++i) if (std::fabs(res.outL[i]) > 0.005f) { at = i; break; }
            const long measured = long(at) - 1000;
            char what[128];
            std::snprintf(what, sizeof(what), "[8] latency reported == measured (oversampling %s, bypass %s)",
                          s.oversampling ? "on" : "off", s.bypass ? "on" : "off");
            check(measured == long(res.latency), what, double(res.latency), double(measured));
        }
    }
    // 10. Classic's Ratio on a sustained tone 12 dB over the Threshold: 20:1 lets it through
    //     ~0.6 dB over, infinity:1 holds it at the Threshold (Ceiling 0, no Boost: clip idle).
    {
        std::vector<float> l(size_t(kSr * 2)), r;
        for (size_t i = 0; i < l.size(); ++i) l[i] = float(std::sin(2 * M_PI * 220.0 * double(i) / kSr)); // 0 dBFS peak
        r = l;
        double pk[3];
        for (int ratio = 0; ratio < 3; ++ratio)
        {
            Settings s; s.truePeak = false; s.thresholdDb = -12.0f; s.ceilingDb = 0.0f; s.ratio = ratio;
            s.attackMs = 1.0f; s.lookaheadMs = 5.0f;
            const Result res = runPlugin(s, l, r);
            pk[ratio] = toDb(samplePeak(res.outL, size_t(kSr)));
        }
        std::printf("[10] Classic, sustained tone 12 dB over -12: output peak 20:1 %.2f, 100:1 %.2f, inf:1 %.2f dB\n", pk[0], pk[1], pk[2]);
        check(pk[0] > pk[1] && pk[1] > pk[2], "[10] ... higher ratio, lower sustained peak", pk[0], pk[2]);
        check(std::fabs(pk[2] - (-12.0)) < std::fabs(pk[0] - (-12.0)) * 0.5, "[10] ... inf:1 at least twice as close to the Threshold as 20:1", pk[2], -12.0);
        // True Peak ignores the Ratio parameter.
        Settings t; t.thresholdDb = -12.0f; t.ceilingDb = 0.0f; t.ratio = 0;
        const Result a = runPlugin(t, l, r);
        t.ratio = 2;
        const Result b = runPlugin(t, l, r);
        check(a.outL == b.outL, "[10] True Peak: output identical for 20:1 and inf:1", 0.0, 0.0);
    }
    // 9. Timing (informational, plus a loose sanity bound): Oversampling off vs on.
    {
        std::vector<float> l, r;
        makeProgram(l, r, size_t(kSr * 20), 3);
        auto timeIt = [&](bool os) {
            Settings s; s.oversampling = os; s.soft = false;
            const auto t0 = std::chrono::steady_clock::now();
            runPlugin(s, l, r);
            return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        };
        timeIt(false);
        const double tOff = timeIt(false), tOn = timeIt(true);
        std::printf("[9] 20 s of stereo audio: oversampling off %.1f ms (%.0fx realtime), on %.1f ms (%.0fx realtime)\n",
                    tOff * 1e3, 20.0 / tOff, tOn * 1e3, 20.0 / tOn);
    }

    std::printf("\n%s: %d check(s) failed\n", fails == 0 ? "PASS" : "FAILURES", fails);
    return fails == 0 ? 0 : 1;
}
