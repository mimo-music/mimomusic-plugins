// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// contour_preset_probe.cpp
//
// Loads a Contour preset (.mccp, exactly what "Save As..." writes) and runs
// sine test signals through the REAL PluginContour::run() -- trigger gain/
// filter, curve, threshold, attack/release, global gains and all -- then
// writes, per case, in/out WAVs, a sample CSV (in, out, gain estimate), an
// output-spectrum CSV, and prints a report:
//
//   - how much the output deviates from what a pure gain change of the
//     fundamental(s) would give (distortion/modulation "residual", dB),
//   - the largest gain jump between consecutive samples ("crush" indicator),
//   - the gain step the curve makes AT the threshold (the size of the jump the
//     gain takes whenever the detector crosses it),
//   - whether the live-trail pair the plugin publishes sits on the curve.
//
// Each signal is run with the preset as saved and with a few one-knob
// variants (trigger filter off / 1ms+5ms attack+release) so the cause of a
// harsh sound can be told apart from the rest.
//
// Not part of the CMake build (needs DPF's Plugin base and
// -fno-access-control, like tests/contour_trigger_test.cpp). From the
// compressor/ directory:
//
//   D=$PWD; g++ -std=gnu++17 -O2 -fno-access-control -DDGL_USE_FILE_BROWSER \
//     -DDISTRHO_PLUGIN_TARGET_LV2 -I$D/dpf/distrho -I$D/plugin_contour \
//     -I$D/plugin/dsp -I$D/plugin -o /tmp/contour_preset_probe \
//     tools/testbench/contour_preset_probe.cpp plugin_contour/PluginContour.cpp \
//     plugin/dsp/SpectrumAnalyzer.cpp dpf/distrho/src/DistrhoPlugin.cpp \
//   && /tmp/contour_preset_probe ../test.mccp tools/testbench/output/contour_probe \
//   && python3 tools/testbench/plot_contour_probe.py tools/testbench/output/contour_probe

#include "DistrhoPlugin.hpp"
#include "PluginContour.hpp"
#include "Wav.hpp"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

START_NAMESPACE_DISTRHO
Plugin* createPlugin();
END_NAMESPACE_DISTRHO
namespace DISTRHO { extern uint32_t d_nextBufferSize; extern double d_nextSampleRate; }

using namespace DISTRHO;
using sscomp::TransferCurve;

namespace {

constexpr double kSr = 48000.0;
constexpr uint32_t kBlock = 512;

// Same key <-> Parameter list as UIContour's own kPresetParams/kPresetKeys.
struct KeyParam { const char* key; uint32_t param; };
const KeyParam kKeys[] = {
    { "inputGainDb", kParamInputGain }, { "thresholdDb", kParamThreshold }, { "attackMs", kParamAttack },
    { "releaseMs", kParamRelease }, { "outputGainDb", kParamOutputGain }, { "bypass", kParamBypass },
    { "sidechainEnable", kParamSidechainEnable }, { "sidechainGainDb", kParamSidechainGain },
    { "thresholdDbB", kParamThresholdB }, { "attackMsB", kParamAttackB }, { "releaseMsB", kParamReleaseB },
    { "makeupGainDbB", kParamMakeupGainB }, { "globalOutputGainDb", kParamGlobalOutputGain },
    { "scFilterEnable", kParamScFilterEnable }, { "scFilterHighpassHz", kParamScFilterHighpassHz },
    { "scFilterLowpassHz", kParamScFilterLowpassHz }, { "scFilterLinkLR", kParamScFilterLinkLR },
    { "scFilterHighpassHzR", kParamScFilterHighpassHzR }, { "scFilterLowpassHzR", kParamScFilterLowpassHzR },
    { "inputGainDbB", kParamInputGainB }, { "wetDry", kParamWetDry }, { "wetDryB", kParamWetDryB },
    { "globalInputGainDb", kParamGlobalInputGain },
};

using Preset = std::map<std::string, std::string>;

Preset loadPreset(const std::string& path)
{
    Preset p;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        p[line.substr(0, eq)] = line.substr(eq + 1);
    }
    return p;
}

double num(const Preset& p, const char* key, double def)
{
    auto it = p.find(key);
    return it == p.end() ? def : std::strtod(it->second.c_str(), nullptr);
}

struct Instance
{
    std::unique_ptr<Plugin> plugin;
    TransferCurve curve;
    double thresholdDb = -18.0;
    double globalGainDb = 0.0;
};

// Mirrors UIContour::pushCurveParamMirror() + pushMirrorCommit().
void pushCurve(Plugin& p, const TransferCurve& c, bool isB, int condition, int action = 0)
{
    const uint32_t cnt = isB ? kParamCurvePointCountB : kParamCurvePointCount;
    const uint32_t cond = isB ? kParamCurveTriggerConditionB : kParamCurveTriggerCondition;
    const uint32_t inB = isB ? kParamCurvePointInBaseB : kParamCurvePointInBase;
    const uint32_t outB = isB ? kParamCurvePointOutBaseB : kParamCurvePointOutBase;
    const uint32_t dirB = isB ? kParamCurvePointDirBaseB : kParamCurvePointDirBase;
    const uint32_t hiB = isB ? kParamCurvePointHandleInBaseB : kParamCurvePointHandleInBase;
    const uint32_t hoB = isB ? kParamCurvePointHandleOutBaseB : kParamCurvePointHandleOutBase;
    p.setParameterValue(cnt, float(c.pointCount()));
    p.setParameterValue(cond, sscomp::packConditionAndThresholdLink(condition, c.thresholdLink(), action));
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const sscomp::CurvePoint q = i < c.pointCount() ? c.point(i) : sscomp::CurvePoint();
        p.setParameterValue(inB + i, float(q.inputDb));
        p.setParameterValue(outB + i, float(q.outputDb));
        p.setParameterValue(dirB + i, sscomp::packDirectionAndLink(q.direction, q.link, q.linkAxis));
        p.setParameterValue(hiB + i, float(q.handleInputDb));
        p.setParameterValue(hoB + i, float(q.handleOutputDb));
    }
}

Instance makeInstance(Preset preset, const std::map<std::string, double>& overrides)
{
    for (const auto& kv : overrides) preset[kv.first] = std::to_string(kv.second);
    d_nextBufferSize = kBlock; d_nextSampleRate = kSr;
    Instance inst;
    inst.plugin.reset(createPlugin());
    Plugin& p = *inst.plugin;
    p.setState("stereoMode", std::to_string(int(num(preset, "stereoMode", 0))).c_str());
    p.setState("linkAB", std::to_string(int(num(preset, "linkAB", 1))).c_str());
    p.activate();
    for (const KeyParam& kp : kKeys)
    {
        auto it = preset.find(kp.key);
        if (it != preset.end()) p.setParameterValue(kp.param, float(std::strtod(it->second.c_str(), nullptr)));
    }
    sscomp::curveFromString(preset["curve"], inst.curve);
    TransferCurve curveB;
    sscomp::curveFromString(preset["curveB"], curveB);
    pushCurve(p, inst.curve, false, int(num(preset, "triggerCondition", 0)), int(num(preset, "triggerAction", 0)));
    pushCurve(p, curveB, true, int(num(preset, "triggerConditionB", 0)), int(num(preset, "triggerActionB", 0)));
    // The probe compares the published dots against the CURVE, which is only meaningful for the detector view
    // (the default "real input -> output" view deliberately leaves the curve whenever the trigger differs).
    p.setParameterValue(kParamTrailSource, 1.0f);
    p.setParameterValue(kParamMirrorCommit, 1.0f);
    p.setParameterValue(kParamMirrorCommitB, 1.0f);
    inst.thresholdDb = num(preset, "thresholdDb", -18.0);
    inst.globalGainDb = num(preset, "globalOutputGainDb", 0.0) + num(preset, "outputGainDb", 0.0);
    return inst;
}

struct Tone { double hz, ampDb; };

std::vector<float> makeSignal(const std::vector<Tone>& tones, double seconds)
{
    // A whole number of blocks (the last partial block would never be processed).
    std::vector<float> v((size_t(seconds * kSr) / kBlock) * kBlock, 0.0f);
    for (size_t i = 0; i < v.size(); ++i)
    {
        double s = 0.0;
        for (const Tone& t : tones) s += std::pow(10.0, t.ampDb / 20.0) * std::sin(2.0 * M_PI * t.hz * double(i) / kSr);
        v[i] = float(s);
    }
    return v;
}

struct BlockPair { double level, gain; };

struct Result
{
    std::vector<float> in, out;
    std::vector<double> gainDb;          // per-sample estimate (NaN where |in| is too small to divide by)
    std::vector<BlockPair> pairs;        // the plugin's published live-trail pair, once per block
    double globalGainDb = 0.0;           // Global Output Gain + Makeup, removed again for the normalised CSV column
};

Result run(Instance& inst, const std::vector<float>& in)
{
    Result r;
    r.in = in;
    r.globalGainDb = inst.globalGainDb;
    r.out.assign(in.size(), 0.0f);
    std::vector<float> zero(kBlock, 0.0f), oL(kBlock), oR(kBlock), inL(kBlock);
    for (size_t pos = 0; pos + kBlock <= in.size(); pos += kBlock)
    {
        std::copy(in.begin() + pos, in.begin() + pos + kBlock, inL.begin());
        const float* ins[4] = { inL.data(), inL.data(), zero.data(), zero.data() };
        float* outs[2] = { oL.data(), oR.data() };
        inst.plugin->run(ins, outs, kBlock);
        std::copy(oL.begin(), oL.end(), r.out.begin() + pos);
        BlockPair bp;
        sscomp::unpackTrailPair(inst.plugin->getParameterValue(kParamMeterTrailA), bp.level, bp.gain);
        r.pairs.push_back(bp);
    }
    double peak = 0.0;
    for (float x : in) peak = std::max(peak, double(std::fabs(x)));
    r.gainDb.assign(in.size(), std::nan(""));
    for (size_t i = 0; i < in.size(); ++i)
        if (std::fabs(in[i]) > 0.1 * peak)
            r.gainDb[i] = 20.0 * std::log10(std::max(1e-12, std::fabs(double(r.out[i]) / double(in[i])))) - inst.globalGainDb;
    return r;
}

// Residual after removing the best-fit sinusoids at the input's own
// frequencies from the OUTPUT (steady second half): what is left is distortion
// + modulation sidebands the pure "gain change" model cannot explain,
// relative to the strongest fitted component.
double residualDb(const std::vector<float>& out, const std::vector<Tone>& tones)
{
    const size_t n0 = out.size() / 2, n1 = out.size();
    std::vector<double> res(out.begin() + n0, out.begin() + n1);
    double strongest = 1e-30;
    for (const Tone& t : tones)
    {
        double a = 0.0, b = 0.0;
        for (size_t i = 0; i < res.size(); ++i)
        {
            const double ph = 2.0 * M_PI * t.hz * double(n0 + i) / kSr;
            a += res[i] * std::sin(ph); b += res[i] * std::cos(ph);
        }
        a *= 2.0 / double(res.size()); b *= 2.0 / double(res.size());
        strongest = std::max(strongest, std::hypot(a, b));
        for (size_t i = 0; i < res.size(); ++i)
        {
            const double ph = 2.0 * M_PI * t.hz * double(n0 + i) / kSr;
            res[i] -= a * std::sin(ph) + b * std::cos(ph);
        }
    }
    double e = 0.0;
    for (double v : res) e += v * v;
    const double rms = std::sqrt(e / double(res.size()));
    return 20.0 * std::log10(std::max(1e-12, rms * std::sqrt(2.0) / strongest));
}

double maxGainStep(const std::vector<double>& g)
{
    double m = 0.0;
    for (size_t i = g.size() / 2 + 1; i < g.size(); ++i)
        if (!std::isnan(g[i]) && !std::isnan(g[i - 1])) m = std::max(m, std::fabs(g[i] - g[i - 1]));
    return m;
}

void fft(std::vector<std::complex<double>>& a)
{
    const size_t n = a.size();
    for (size_t i = 1, j = 0; i < n; ++i)
    {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) std::swap(a[i], a[j]);
    }
    for (size_t len = 2; len <= n; len <<= 1)
    {
        const double ang = -2.0 * M_PI / double(len);
        const std::complex<double> wl(std::cos(ang), std::sin(ang));
        for (size_t i = 0; i < n; i += len)
        {
            std::complex<double> w(1.0);
            for (size_t k = 0; k < len / 2; ++k)
            {
                const auto u = a[i + k], v = a[i + k + len / 2] * w;
                a[i + k] = u + v; a[i + k + len / 2] = u - v; w *= wl;
            }
        }
    }
}

void writeSpectrum(const std::string& path, const std::vector<float>& out)
{
    const size_t N = 32768;
    std::vector<std::complex<double>> a(N);
    const size_t off = out.size() - N;
    for (size_t i = 0; i < N; ++i)
        a[i] = double(out[off + i]) * (0.5 - 0.5 * std::cos(2.0 * M_PI * double(i) / double(N)));
    fft(a);
    std::ofstream f(path);
    f << "freq_hz,db\n";
    for (size_t k = 1; k < N / 2; ++k)
    {
        const double mag = std::abs(a[k]) / (double(N) / 4.0); // 0dB = full-scale sine
        f << (double(k) * kSr / double(N)) << ',' << 20.0 * std::log10(std::max(1e-9, mag)) << '\n';
    }
}

void writeCsv(const std::string& path, const Result& r)
{
    std::ofstream f(path);
    // out_norm = out with the global output gain taken back out, so it can be drawn
    // straight over the input and only shows what the CURVE did.
    const double undo = std::pow(10.0, -r.globalGainDb / 20.0);
    f << "i,t_ms,in,out,out_norm,gain_db\n";
    for (size_t i = 0; i < r.in.size(); ++i)
    {
        f << i << ',' << double(i) / kSr * 1000.0 << ',' << r.in[i] << ',' << r.out[i] << ',' << double(r.out[i]) * undo << ',';
        if (!std::isnan(r.gainDb[i])) f << r.gainDb[i];
        f << '\n';
    }
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) { std::printf("usage: %s preset.mccp [outdir]\n", argv[0]); return 2; }
    const std::string outDir = argc >= 3 ? argv[2] : "contour_probe";
    std::filesystem::create_directories(outDir);
    const Preset preset = loadPreset(argv[1]);

    Instance ref = makeInstance(preset, {});
    const TransferCurve& curve = ref.curve;
    const double thr = ref.thresholdDb;
    std::printf("Preset: %s\n", argv[1]);
    std::printf("  threshold %.2f dB, Trigger %s, attack %.2f ms, release %.2f ms, trigger gain %+.1f dB, "
                "trigger filter %s (HP %.0f Hz / LP %.1f Hz), global out %+.2f dB, wet %.0f%%\n",
                thr, num(preset, "triggerCondition", 0) == 0 ? "Above" : "Below", num(preset, "attackMs", 0), num(preset, "releaseMs", 0),
                num(preset, "sidechainGainDb", 0), num(preset, "scFilterEnable", 0) > 0.5 ? "ON" : "off",
                num(preset, "scFilterHighpassHz", 0), num(preset, "scFilterLowpassHz", 0), ref.globalGainDb, num(preset, "wetDry", 100));
    const double stepAtThr = curve.evaluate(thr) - thr;
    std::printf("  curve at the threshold: in %.2f dB -> out %.2f dB, i.e. the gain STEPS by %+.2f dB every time the detector "
                "crosses the threshold (it is not identity there)\n\n", thr, curve.evaluate(thr), stepAtThr);

    struct Sig { const char* name; std::vector<Tone> tones; };
    const std::vector<Sig> signals = {
        { "sine30_m12dB", { { 30.0, -12.0 } } },
        { "sine60_m6dB", { { 60.0, -6.0 } } },
        { "sine1k_m12dB", { { 1000.0, -12.0 } } },
        { "bass40_plus_1k", { { 40.0, -12.0 }, { 1000.0, -18.0 } } },
    };
    struct Cfg { const char* name; std::map<std::string, double> over; };
    const std::vector<Cfg> configs = {
        { "saved", {} },
        { "trigfilter_off", { { "scFilterEnable", 0.0 } } },
        { "atk1_rel5ms", { { "attackMs", 1.0 }, { "releaseMs", 5.0 } } },
    };

    std::printf("%-34s %14s %16s %13s %12s %14s\n", "case", "residual (dB)", "max gain step dB", "detector dB", "trail max dev", "gain range dB");
    for (const Sig& sig : signals)
        for (const Cfg& cfg : configs)
        {
            Instance inst = makeInstance(preset, cfg.over);
            const std::vector<float> in = makeSignal(sig.tones, 1.0);
            Result r = run(inst, in);
            const std::string name = std::string(sig.name) + "__" + cfg.name;

            // Trail pair check: published (level, gain) vs the curve. Above/Below per the preset.
            const bool above = num(preset, "triggerCondition", 0) == 0;
            double maxDev = 0.0, lvMin = 1e9, lvMax = -1e9;
            for (size_t b = r.pairs.size() / 2; b < r.pairs.size(); ++b)
            {
                const BlockPair& bp = r.pairs[b];
                lvMin = std::min(lvMin, bp.level); lvMax = std::max(lvMax, bp.level);
                const bool trig = above ? bp.level > thr : bp.level < thr;
                const double expectOut = trig ? (bp.level < curve.point(0).inputDb ? bp.level + (curve.point(0).outputDb - curve.point(0).inputDb)
                                                                                   : curve.evaluate(bp.level)) : bp.level;
                maxDev = std::max(maxDev, std::fabs((bp.level + bp.gain) - expectOut));
            }
            double gLo = 1e9, gHi = -1e9;
            for (size_t i = r.gainDb.size() / 2; i < r.gainDb.size(); ++i)
                if (!std::isnan(r.gainDb[i])) { gLo = std::min(gLo, r.gainDb[i]); gHi = std::max(gHi, r.gainDb[i]); }
            std::printf("%-34s %14.1f %16.2f  %5.0f..%-5.0f %12.3f   %+.1f..%+.1f\n", name.c_str(), residualDb(r.out, sig.tones),
                        maxGainStep(r.gainDb), lvMin, lvMax, maxDev, gLo, gHi);

            writeCsv(outDir + "/" + name + ".csv", r);
            writeSpectrum(outDir + "/" + name + "_spectrum.csv", r.out);
            wavio::writeWavMonoFloat(outDir + "/" + name + "_in.wav", kSr, r.in);
            wavio::writeWavMonoFloat(outDir + "/" + name + "_out.wav", kSr, r.out);
        }
    std::printf("\nWrote WAVs/CSVs to %s/\n", outDir.c_str());
    return 0;
}
