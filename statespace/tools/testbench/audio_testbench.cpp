// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// audio_testbench.cpp
//
// Offline verification tool: runs every factory preset through the *exact*
// real-time engine (StateSpaceEngine -- the same class the plugin uses,
// not a reimplementation) against synthetic test audio, and cross-checks
// the measured result two independent ways:
//
//   1. Time domain vs frequency domain: a steady tone at a known frequency
//      is fed through the actual sample-by-sample recursive engine; its
//      measured steady-state gain is compared against
//      TransferFunction::evaluate() at that exact frequency, which solves
//      (zI-A)x=B directly (dense complex linear algebra, no recursion at
//      all). These are two structurally unrelated computations of the same
//      underlying math -- if they agree closely, that's strong evidence
//      the model is being evaluated correctly end-to-end, not just
//      self-consistently through one code path. This is exactly the kind
//      of check that would have caught the stability-guard bug this tool
//      was written in response to (a "notch" preset that produced no
//      audible or visible notch at all -- see docs/EXTENDING.md's git
//      history / project notes for the postmortem).
//
//   2. Qualitative spot checks: a small per-preset table of "at this
//      frequency, expect the gain to be roughly in this range" -- a
//      human-meaningful sanity check that the *design* matches intent,
//      not just that the two computations agree with each other.
//
// It also writes WAV files (a log sine sweep and a multi-tone probe signal,
// input and processed-output versions) for every preset, so the results
// can be listened to / inspected in an external tool, not just read as
// numbers.
//
// This is a developer/verification tool, not part of the plugin. Build and
// run it via the `audio_testbench` CMake target (see the root
// CMakeLists.txt) -- that also bakes in this project's absolute path via
// -DSTATESPACE_PROJECT_SOURCE_DIR, so `./build/bin/audio_testbench` with no
// arguments works from any current directory. Built standalone instead
// (e.g. `g++ -std=c++17 -O2 -Iplugin/dsp tools/testbench/audio_testbench.cpp
// plugin/dsp/StabilityGuard.cpp plugin/dsp/TransferFunction.cpp
// plugin/dsp/Serialization.cpp -o audio_testbench`), it falls back to
// plain relative paths, so run it from the project root or pass explicit
// paths: `./audio_testbench presets/factory tools/testbench/output`.

#include "../../plugin/dsp/CoefficientMapper.hpp"
#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../../plugin/dsp/StateSpaceEngine.hpp"
#include "../../plugin/dsp/TransferFunction.hpp"
#include "Wav.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace ssfilter;
namespace fs = std::filesystem;

namespace {

constexpr double kSampleRate = 48000.0;

// ---- Test signal generation --------------------------------------------

// Logarithmic sine sweep ("chirp"), 20 Hz -> 20 kHz over `seconds`, at -12
// dBFS to leave headroom for any preset that boosts gain.
void generateSweep(std::vector<float>& out, double seconds, double sampleRate)
{
    const uint32_t n = uint32_t(seconds * sampleRate);
    out.resize(n);
    const double f0 = 20.0, f1 = std::min(20000.0, sampleRate * 0.5 - 1.0);
    const double amp = std::pow(10.0, -12.0 / 20.0);
    const double K = seconds / std::log(f1 / f0);
    for (uint32_t i = 0; i < n; ++i)
    {
        const double t = double(i) / sampleRate;
        // Instantaneous-phase log sweep: phase(t) = 2*pi*f0*K*(exp(t/K)-1)
        const double phase = 2.0 * M_PI * f0 * K * (std::exp(t / K) - 1.0);
        out[i] = float(amp * std::sin(phase));
    }
}

struct ToneProbe { double freqHz; uint32_t startSample; uint32_t numSamples; };

// Concatenated steady tones at `freqs`, each held long enough for the given
// model's slowest expected settling time (a narrow/high-Q filter can take
// the better part of a second to settle -- see the notch case), at -12
// dBFS. Returns the list of {freq, region} so the caller knows where to
// measure each tone's steady-state portion.
void generateToneProbe(std::vector<float>& out, std::vector<ToneProbe>& regions,
                        const std::vector<double>& freqs, double settleSeconds, double sampleRate)
{
    const double amp = std::pow(10.0, -12.0 / 20.0);
    const double toneDuration = std::max(0.3, settleSeconds * 1.3 + 0.1);
    const double gapDuration = 0.05;
    out.clear();
    regions.clear();

    for (double f : freqs)
    {
        const uint32_t toneN = uint32_t(toneDuration * sampleRate);
        const uint32_t gapN = uint32_t(gapDuration * sampleRate);
        const uint32_t start = uint32_t(out.size());
        for (uint32_t i = 0; i < toneN; ++i)
            out.push_back(float(amp * std::sin(2.0 * M_PI * f * double(i) / sampleRate)));
        for (uint32_t i = 0; i < gapN; ++i)
            out.push_back(0.0f);
        regions.push_back({ f, start, toneN });
    }
}

double rmsDb(const float* data, uint32_t start, uint32_t count)
{
    double sumSq = 0.0;
    for (uint32_t i = 0; i < count; ++i) sumSq += double(data[start + i]) * double(data[start + i]);
    const double rms = std::sqrt(sumSq / std::max<uint32_t>(1, count));
    return 20.0 * std::log10(std::max(rms, 1e-12));
}

// ---- Per-preset qualitative expectations --------------------------------

struct SpotCheck { double freqHz; double minDb; double maxDb; const char* note; };
struct PresetInfo { const char* filename; std::vector<SpotCheck> checks; };

const std::vector<PresetInfo>& spotCheckTable()
{
    static const std::vector<PresetInfo> table = {
        { "passthrough.ssjson", { { 1000.0, -0.5, 0.5, "flat unity gain" } } },
        { "lowpass_1k.ssjson", { { 100.0, -1.0, 1.0, "passband" }, { 1000.0, -4.5, -1.5, "cutoff, ~-3dB" }, { 10000.0, -80.0, -30.0, "stopband" } } },
        { "highpass_200.ssjson", { { 20.0, -80.0, -20.0, "stopband" }, { 200.0, -4.5, -1.5, "cutoff, ~-3dB" }, { 2000.0, -1.0, 1.0, "passband" } } },
        { "bell_3k_plus6db.ssjson", { { 300.0, -1.0, 1.0, "away from band" }, { 3000.0, 4.5, 7.0, "boost center, ~+6dB" } } },
        { "bell_250_minus6db_mudcut.ssjson", { { 250.0, -7.5, -4.5, "cut center, ~-6dB" }, { 3000.0, -1.0, 1.0, "away from band" } } },
        { "highshelf_8k_minus4db.ssjson", { { 200.0, -1.0, 1.0, "below shelf" }, { 16000.0, -5.0, -2.0, "on shelf, cut" } } },
        { "lowshelf_120_plus3db_warmth.ssjson", { { 30.0, 1.0, 4.0, "on shelf, boost" }, { 2000.0, -1.0, 1.0, "above shelf" } } },
        { "notch_50hz_hum.ssjson", { { 20.0, -1.0, 1.0, "away from notch" }, { 50.0, -80.0, -8.0, "AT the notch -- must dip" }, { 1000.0, -1.0, 1.0, "away from notch" } } },
        { "bandpass_2k.ssjson", { { 200.0, -80.0, -15.0, "far below center" }, { 2000.0, -1.0, 1.0, "center, unity" }, { 20000.0, -80.0, -15.0, "far above center" } } },
        { "allpass_1k.ssjson", { { 100.0, -0.5, 0.5, "flat magnitude" }, { 1000.0, -0.5, 0.5, "flat magnitude (phase shifts, magnitude doesn't)" }, { 10000.0, -0.5, 0.5, "flat magnitude" } } },
        { "telephone_bandlimit.ssjson", { { 50.0, -80.0, -10.0, "below band" }, { 1000.0, -1.0, 1.0, "in band" }, { 10000.0, -80.0, -10.0, "above band" } } },
        { "vocal_presence_chain.ssjson", { { 40.0, -80.0, -8.0, "rumble cut" }, { 3000.0, 1.0, 5.0, "presence bump" } } },
        { "smile_eq_loudness.ssjson", { { 30.0, 1.5, 6.0, "low shelf boost" }, { 16000.0, 1.0, 5.0, "high shelf boost" } } },
        // The -12 dBFS probe tone is 4 dB into this preset's learned
        // threshold (-16 dB), at ratio ~6:1 with +4 dB makeup. A one-pole
        // envelope follower riding a steady *sine* (as opposed to a
        // constant/DC level) never quite tracks the true peak -- there's
        // always a little ripple/undershoot between peaks, same as a real
        // compressor -- so the measured gain (~1.1-1.3 dB here) settles a
        // bit above the idealized DC-level prediction of (4 dB over) *
        // (1/6 - 1) + 4 dB makeup ~= +0.67 dB. That gap is expected and
        // should stay consistent across frequencies (a real compressor's
        // behavior shouldn't depend on the input's frequency content); see
        // dsp_regression tests [8]/[9] for the DC-level exact check instead.
        { "trained_compressor_demo.ssjson", { { 1000.0, 0.5, 1.8, "compressed + makeup, net ~+1.1-1.3dB at -12dBFS (see comment above)" } } },
    };
    return table;
}

} // namespace

#ifndef STATESPACE_PROJECT_SOURCE_DIR
// Fallback if built outside the project's own CMake (the CMake target
// defines this to the repo root's absolute path -- see CMakeLists.txt --
// specifically so this default works no matter which directory the binary
// is actually run from, e.g. from inside build/bin/ where a plain relative
// "presets/factory" would not exist).
#define STATESPACE_PROJECT_SOURCE_DIR "."
#endif

int main(int argc, char** argv)
{
    const std::string presetsDir = argc > 1 ? argv[1] : std::string(STATESPACE_PROJECT_SOURCE_DIR) + "/presets/factory";
    const std::string outDir     = argc > 2 ? argv[2] : std::string(STATESPACE_PROJECT_SOURCE_DIR) + "/tools/testbench/output";

    if (!fs::exists(presetsDir) || !fs::is_directory(presetsDir))
    {
        std::fprintf(stderr,
                      "error: presets directory not found: %s\n"
                      "Usage: %s [presets_dir] [output_dir]\n"
                      "  (run with no arguments from anywhere -- it defaults to this project's own\n"
                      "   presets/factory/ regardless of your current directory -- or pass an\n"
                      "   explicit path, e.g. ../../presets/factory if run from build/bin/)\n",
                      presetsDir.c_str(), argv[0]);
        return 1;
    }
    fs::create_directories(outDir);

    std::ostringstream report;
    auto logBoth = [&](const std::string& line) { std::printf("%s\n", line.c_str()); report << line << "\n"; };

    logBoth("StateSpace MIMO -- audio testbench");
    logBoth("presets: " + presetsDir + "   output: " + outDir);
    logBoth(std::string(70, '='));

    // Shared generic probe frequencies (log-spaced, 24 points 20Hz-20kHz) --
    // run against every preset regardless of its specific design, so the
    // engine/transfer-function cross-check has broad coverage even for
    // presets not in the qualitative spot-check table above.
    std::vector<double> genericFreqs;
    for (int i = 0; i < 24; ++i)
        genericFreqs.push_back(std::pow(10.0, std::log10(20.0) + (std::log10(20000.0) - std::log10(20.0)) * i / 23.0));

    int totalPass = 0, totalFail = 0;
    int presetCount = 0;

    // Collected up front (rather than iterated in place) so a filesystem
    // error (permissions, a symlink race, ...) is reported cleanly instead
    // of throwing out of an uncaught range-for and aborting the process --
    // fs::exists() above already checked the common case, this is defense
    // in depth for the rest.
    std::vector<fs::directory_entry> entries;
    try
    {
        for (const auto& e : fs::directory_iterator(presetsDir)) entries.push_back(e);
    }
    catch (const std::filesystem::filesystem_error& e)
    {
        std::fprintf(stderr, "error: could not list %s: %s\n", presetsDir.c_str(), e.what());
        return 1;
    }

    for (const auto& entry : entries)
    {
        if (entry.path().extension() != ".ssjson") continue;
        ++presetCount;
        const std::string stem = entry.path().stem().string();

        std::ifstream f(entry.path());
        std::ostringstream ss; ss << f.rdbuf();
        StateSpaceMatrices m;
        if (!Serialization::fromJson(ss.str(), m))
        {
            logBoth("[" + stem + "] FAILED TO PARSE PRESET FILE");
            ++totalFail;
            continue;
        }
        StabilityGuard::enforce(m); // exactly what the plugin does before use

        logBoth("");
        logBoth("-- " + stem + "  (order " + std::to_string(m.order) + ", " +
                std::string(m.name) + ") " + std::string(30, '-'));

        // Combine generic + spot-check frequencies for the tone probe so
        // both the broad cross-check and the specific expectations reuse
        // one signal/measurement pass.
        std::vector<double> probeFreqs = genericFreqs;
        const PresetInfo* info = nullptr;
        for (const auto& p : spotCheckTable())
            if (p.filename == stem + ".ssjson") { info = &p; break; }
        if (info)
            for (const auto& c : info->checks) probeFreqs.push_back(c.freqHz);
        std::sort(probeFreqs.begin(), probeFreqs.end());
        probeFreqs.erase(std::unique(probeFreqs.begin(), probeFreqs.end()), probeFreqs.end());

        const double radius = StabilityGuard::estimateSpectralRadius(m);
        // Samples for the state to decay to 0.1% of its initial size,
        // converted to seconds -- a narrow/high-Q filter's pole radius can
        // be extremely close to 1 (that's what "narrow" means at audio
        // sample rates), so this is in samples, *not* seconds, until
        // divided by sampleRate; getting that wrong once produced a
        // multi-hour tone duration and an out-of-memory kill during
        // testing, which is exactly the kind of thing this tool is meant
        // to catch elsewhere -- so it seems only fair to leave the lesson
        // in the comment.
        const double settleSamples = radius < 0.999 ? 0.05 * kSampleRate
                                                      : std::log(0.001) / std::log(std::max(radius, 1e-6));
        double settleSeconds = std::min(5.0, settleSamples / kSampleRate); // hard cap, just in case

        // A nonlinear gain stage has its own settling time (its envelope's
        // attack/release), independent of the LTI part's -- for a plain
        // passthrough base model (spectral radius 0) the LTI settle time
        // above is ~instant, but the compressor's own release can still be
        // hundreds of ms. ~6.9 time constants decays a one-pole smoother to
        // within 0.1%.
        if (m.nonlinear.enabled)
            settleSeconds = std::min(5.0, std::max(settleSeconds,
                                                     6.9 * std::max(m.nonlinear.attackMs, m.nonlinear.releaseMs) / 1000.0));
        // Same idea, per band, for the multiband gain stage (see
        // StateSpaceTypes.hpp's MultibandSchedule) -- the slowest band's
        // own attack/release sets the floor.
        if (m.multiband.enabled)
            for (int k = 0; k < m.multiband.numBands; ++k)
            {
                const NonlinearSchedule& sch = m.multiband.bands[k].schedule;
                settleSeconds = std::min(5.0, std::max(settleSeconds,
                                                         6.9 * std::max(sch.attackMs, sch.releaseMs) / 1000.0));
            }


        std::vector<float> toneL;
        std::vector<ToneProbe> regions;
        generateToneProbe(toneL, regions, probeFreqs, settleSeconds, kSampleRate);
        std::vector<float> toneR = toneL; // mono content in both channels for this probe

        std::vector<float> sweepL, sweepR;
        generateSweep(sweepL, 6.0, kSampleRate);
        sweepR = sweepL;

        // Process through the *actual* real-time engine.
        StateSpaceEngine eng;
        eng.prepare(kSampleRate);
        eng.setModel(m);
        eng.resetState();

        std::vector<float> toneOutL(toneL.size()), toneOutR(toneR.size());
        eng.processBlock(toneL.data(), toneR.data(), toneOutL.data(), toneOutR.data(), uint32_t(toneL.size()));

        eng.resetState();
        std::vector<float> sweepOutL(sweepL.size()), sweepOutR(sweepR.size());
        eng.processBlock(sweepL.data(), sweepR.data(), sweepOutL.data(), sweepOutR.data(), uint32_t(sweepL.size()));

        // Cross-check: measured (time-domain engine) vs predicted
        // (frequency-domain matrix solve) gain at every probe frequency.
        double worstDiff = 0.0;
        double minMeasuredDb = 1e300, maxMeasuredDb = -1e300;
        for (const auto& reg : regions)
        {
            const uint32_t measureStart = reg.startSample + uint32_t(double(reg.numSamples) * 0.7);
            const uint32_t measureCount = reg.numSamples - uint32_t(double(reg.numSamples) * 0.7);
            const double measuredDb = rmsDb(toneOutL.data(), measureStart, measureCount) -
                                       rmsDb(toneL.data(), reg.startSample + uint32_t(double(reg.numSamples) * 0.1),
                                             measureCount); // relative to the (also-settled) input tone's own level
            minMeasuredDb = std::min(minMeasuredDb, measuredDb);
            maxMeasuredDb = std::max(maxMeasuredDb, measuredDb);

            const double omega = 2.0 * M_PI * reg.freqHz / kSampleRate;
            const ComplexResponse resp = TransferFunction::evaluate(m, Complex(std::cos(omega), std::sin(omega)));
            const double predictedDb = 20.0 * std::log10(std::max(std::abs(resp.H[0][0]), 1e-12));

            // Comparing gain *ratios* near the numerical silence floor is
            // meaningless (e.g. the "silence" preset: both sides correctly
            // report "very quiet", but exactly how many dB below nothing
            // isn't a well-posed question) -- skip those points rather than
            // let them dominate worstDiff with a non-finding.
            if (predictedDb > -100.0)
                worstDiff = std::max(worstDiff, std::fabs(measuredDb - predictedDb));

            if (info)
                for (const auto& c : info->checks)
                    if (c.freqHz == reg.freqHz)
                    {
                        const bool pass = measuredDb >= c.minDb && measuredDb <= c.maxDb;
                        logBoth(std::string("   ") + (pass ? "PASS " : "FAIL ") +
                                "f=" + std::to_string(int(c.freqHz)) + "Hz  measured=" +
                                std::to_string(measuredDb) + "dB  expected[" + std::to_string(c.minDb) +
                                ".." + std::to_string(c.maxDb) + "]dB  (" + c.note + ")");
                        pass ? ++totalPass : ++totalFail;
                    }
        }

        if (m.nonlinear.enabled || m.multiband.enabled)
        {
            // TransferFunction::evaluate() is a pure-LTI (frequency-domain)
            // prediction -- it has no notion of either state-dependent gain
            // stage, so a genuine divergence here is *expected*, not a
            // bug, once either is enabled. Their own correctness (and the
            // trainers that fit them) is verified separately and far more
            // precisely in dsp_regression's tests [8]/[9] (single-band) and
            // [15]-[17] (multiband), against known analytic/synthetic
            // ground truth rather than a check that doesn't model either
            // stage at all.
            logBoth("   SKIP engine-vs-transferfunction cross-check: preset has a nonlinear/multiband "
                    "gain stage (see dsp_regression tests [8]/[9]/[15]-[17] for that verification)");

            // What we *can* and must still check here, for the *single-band*
            // stage specifically: a state-dependent gain keyed off the
            // whole (broadband) input's level should compress a 100Hz tone
            // and a 3kHz tone identically, since every probe tone in this
            // file is generated at the same fixed amplitude (see
            // generateToneProbe's `amp`). This is a permanent regression
            // guard for a real bug found during development: an earlier
            // version smoothed the envelope follower in the dB domain
            // against each sample's instantaneous |value|, which touches
            // -infinity dB at every zero crossing of an AC signal -- that
            // made the *same* level compress differently depending on how
            // often the signal crossed zero, i.e. its frequency (measured
            // spread was up to ~1.2 dB across 100Hz-3kHz). The fix (linear-
            // domain envelope smoothing, see StateSpaceEngine.hpp) brought
            // that down to ~0.1dB; this check keeps it there.
            //
            // This does NOT apply to a genuinely multiband preset: by
            // design, a 100Hz probe tone and a 3kHz probe tone can land in
            // *different bands* with different threshold/ratio, so
            // compressing them differently is the entire point, not a bug
            // -- see dsp_regression tests [15]-[17] for the ground-truth
            // per-band checks instead.
            if (m.nonlinear.enabled)
            {
                const double freqIndependenceSpreadDb = maxMeasuredDb - minMeasuredDb;
                const bool freqIndependencePass = freqIndependenceSpreadDb < 0.3; // dB
                logBoth(std::string("   ") + (freqIndependencePass ? "PASS " : "FAIL ") +
                        "nonlinear gain is frequency-independent: spread = " +
                        std::to_string(freqIndependenceSpreadDb) + " dB across " +
                        std::to_string(probeFreqs.size()) + " frequencies (all probed at the same level)");
                freqIndependencePass ? ++totalPass : ++totalFail;
            }
        }
        else
        {
            const bool crossCheckPass = worstDiff < 0.5; // dB
            logBoth(std::string("   ") + (crossCheckPass ? "PASS " : "FAIL ") +
                    "engine-vs-transferfunction cross-check: worst discrepancy = " +
                    std::to_string(worstDiff) + " dB across " + std::to_string(probeFreqs.size()) + " frequencies");
            crossCheckPass ? ++totalPass : ++totalFail;
        }

        // Also confirm the engine stays finite/bounded on the sweep (a
        // broadband, not just single-tone, real-world-ish signal).
        bool boundedOk = true;
        for (float v : sweepOutL) if (!std::isfinite(v) || std::fabs(v) > 60.0f) boundedOk = false;
        logBoth(std::string("   ") + (boundedOk ? "PASS " : "FAIL ") + "sweep output stays finite/bounded");
        boundedOk ? ++totalPass : ++totalFail;

        // Write WAV files for listening/inspection.
        wavio::writeWavStereoFloat(outDir + "/" + stem + "_sweep_out.wav", kSampleRate, sweepOutL, sweepOutR);
        wavio::writeWavStereoFloat(outDir + "/" + stem + "_tones_out.wav", kSampleRate, toneOutL, toneOutR);
    }

    // Shared input reference files (same for every preset) written once.
    {
        std::vector<float> sweepL; generateSweep(sweepL, 6.0, kSampleRate);
        wavio::writeWavStereoFloat(outDir + "/_input_sweep.wav", kSampleRate, sweepL, sweepL);
        std::vector<float> toneL; std::vector<ToneProbe> regions;
        generateToneProbe(toneL, regions, genericFreqs, 0.05, kSampleRate);
        wavio::writeWavStereoFloat(outDir + "/_input_tones_generic.wav", kSampleRate, toneL, toneL);
    }

    logBoth("");
    logBoth(std::string(70, '='));
    logBoth("Presets tested: " + std::to_string(presetCount) +
            "   Checks passed: " + std::to_string(totalPass) +
            "   Checks failed: " + std::to_string(totalFail));
    logBoth("WAV files (inputs + every preset's processed sweep/tones) written to: " + outDir);

    std::ofstream reportFile(outDir + "/report.txt");
    reportFile << report.str();

    return totalFail == 0 ? 0 : 1;
}
