// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// audio_testbench.cpp
//
// Runs actual synthetic audio through the *real* per-sample DSP chain
// (plugin/dsp/CompressorDsp.hpp's classes, wired together exactly the way
// PluginCompressorMono::run() / PluginCompressorStereo::run() do -- if you
// change one, change the other) rather than checking closed-form formulas
// in isolation the way tests/dsp_regression.cpp does. Complements that
// suite the same way ../statespace's own
// audio_testbench.cpp complements its dsp_regression.cpp: real WAV-shaped
// signals through the real engine, with an *independent* cross-check where
// possible (e.g. Test A measures gain reduction two structurally different
// ways -- the engine's own reported value, and a plain RMS ratio of the
// actual output/input samples -- and requires them to agree) rather than
// re-deriving the same formula the engine already used internally.
//
// No DPF/GUI dependency -- just this project's DSP core, dependency-free
// WAV I/O (Wav.hpp), and a synthetic signal generator. Also writes a
// handful of WAV files to tools/testbench/output/ (git-ignored,
// regenerable) so the actual sound of the compression can be listened to,
// not just measured.
//
// Usage: ./audio_testbench            (writes output/ next to the binary's cwd)

#include "../../plugin/dsp/CompressorDsp.hpp"
#include "Wav.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace sscomp;

namespace {

constexpr double kSampleRate = 48000.0;
int g_failures = 0;

void check(bool ok, const char* what)
{
    printf("  %-70s %s\n", what, ok ? "OK" : "FAIL");
    if (!ok) ++g_failures;
}

std::vector<float> genSilence(double seconds, double sr = kSampleRate)
{
    return std::vector<float>(size_t(seconds * sr), 0.0f);
}

std::vector<float> genSine(double freqHz, double amplitudeLin, double seconds, double sr = kSampleRate)
{
    std::vector<float> v(size_t(seconds * sr));
    for (size_t i = 0; i < v.size(); ++i)
        v[i] = float(amplitudeLin * std::sin(2.0 * M_PI * freqHz * double(i) / sr));
    return v;
}

std::vector<float> concat(std::initializer_list<std::vector<float>> parts)
{
    std::vector<float> out;
    for (const auto& p : parts) out.insert(out.end(), p.begin(), p.end());
    return out;
}

double rmsDbOfWindow(const std::vector<float>& v, size_t start, size_t end)
{
    start = std::min(start, v.size());
    end = std::min(end, v.size());
    if (end <= start) return -120.0;
    double sumSq = 0.0;
    for (size_t i = start; i < end; ++i) sumSq += double(v[i]) * double(v[i]);
    return linearToDb(std::sqrt(sumSq / double(end - start)));
}

double meanOfWindow(const std::vector<double>& v, size_t start, size_t end)
{
    start = std::min(start, v.size());
    end = std::min(end, v.size());
    if (end <= start) return 0.0;
    double sum = 0.0;
    for (size_t i = start; i < end; ++i) sum += v[i];
    return sum / double(end - start);
}

bool nearlyEqual(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ---- the mono chain, wired exactly like PluginCompressorMono::run() -----
struct MonoResult
{
    std::vector<float> output;
    std::vector<double> gainReductionDb;
    std::vector<double> detectorLevelDb;
};

MonoResult processMono(const std::vector<float>& input, double sampleRate,
                        double inputGainDb, double thresholdDb, double ratio,
                        double attackMs, double releaseMs, double kneeDb,
                        double outputGainDb)
{
    CompressorVoice voice;
    voice.prepare(sampleRate);
    voice.setParams(attackMs, releaseMs, thresholdDb, ratio, 1.0, kneeDb);

    MonoResult r;
    const size_t n = input.size();
    r.output.resize(n);
    r.gainReductionDb.resize(n);
    r.detectorLevelDb.resize(n);

    const double inGainLin = dbToLinear(inputGainDb);
    const double outGainLin = dbToLinear(outputGainDb);

    for (size_t i = 0; i < n; ++i)
    {
        const double xg = double(input[i]) * inGainLin;
        const double grLin = voice.process(std::fabs(xg));
        double y = xg * grLin * outGainLin;
        r.output[i] = float(y);
        r.gainReductionDb[i] = voice.gainReductionDb();
        r.detectorLevelDb[i] = voice.detectorLevelDb();
    }
    return r;
}

// ---- the stereo chain, wired exactly like PluginCompressorStereo::run() -
struct StereoResult
{
    std::vector<float> outL, outR;
    std::vector<double> gainReductionA, gainReductionB;
};

StereoResult processStereo(const std::vector<float>& inL, const std::vector<float>& inR, double sampleRate,
                            StereoMode mode, double thresholdDb, double ratio,
                            double attackMs, double releaseMs, double kneeDb)
{
    CompressorVoice voiceA, voiceB;
    voiceA.prepare(sampleRate);
    voiceB.prepare(sampleRate);
    voiceA.setParams(attackMs, releaseMs, thresholdDb, ratio, 1.0, kneeDb);
    voiceB.setParams(attackMs, releaseMs, thresholdDb, ratio, 1.0, kneeDb);

    StereoResult r;
    const size_t n = std::min(inL.size(), inR.size());
    r.outL.resize(n); r.outR.resize(n);
    r.gainReductionA.resize(n); r.gainReductionB.resize(n);

    for (size_t i = 0; i < n; ++i)
    {
        const double xgL = double(inL[i]);
        const double xgR = double(inR[i]);
        double yL, yR;
        switch (mode)
        {
        case StereoMode::Linked:
        {
            const double detector = std::max(std::fabs(xgL), std::fabs(xgR));
            const double gr = voiceA.process(detector);
            voiceB.process(detector);
            yL = xgL * gr; yR = xgR * gr;
            break;
        }
        case StereoMode::DualMono:
        {
            const double grL = voiceA.process(std::fabs(xgL));
            const double grR = voiceB.process(std::fabs(xgR));
            yL = xgL * grL; yR = xgR * grR;
            break;
        }
        case StereoMode::MidSide:
        default:
        {
            const double mid = (xgL + xgR) * 0.5;
            const double side = (xgL - xgR) * 0.5;
            const double grMid = voiceA.process(std::fabs(mid));
            const double grSide = voiceB.process(std::fabs(side));
            const double yMid = mid * grMid;
            const double ySide = side * grSide;
            yL = yMid + ySide; yR = yMid - ySide;
            break;
        }
        }
        r.outL[i] = float(yL); r.outR[i] = float(yR);
        r.gainReductionA[i] = voiceA.gainReductionDb();
        r.gainReductionB[i] = voiceB.gainReductionDb();
    }
    return r;
}

} // namespace

int main()
{
    std::filesystem::path outDir = std::filesystem::path("tools/testbench/output");
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
    // Fall back to a plain relative dir if run from inside build/bin/ and
    // the project-root-relative path doesn't exist as a parent already --
    // simplest possible approach here since (unlike the sibling project)
    // there's no separate "run from any directory" requirement.
    if (ec) outDir = "testbench_output";
    std::filesystem::create_directories(outDir, ec);

    constexpr double thresholdDb = -18.0, ratio = 4.0, kneeDb = 6.0;
    constexpr double attackMs = 5.0, releaseMs = 80.0;

    printf("== Test A: steady-state gain reduction accuracy across levels ==\n");
    {
        // Part 1: with the envelope's attack time constant much shorter
        // than the test tone's own quarter-period, the peak follower fully
        // tracks each cycle's peak, so its steady-state reading should sit
        // right on the tone's true peak amplitude -- checked against
        // computeGainReductionDb() evaluated at that known peak level.
        constexpr double fastAttackMs = 0.05, slowReleaseMs = 50.0;
        for (double levelDb : { -6.0, -12.0, -18.0, -24.0, -30.0 })
        {
            const double amplitude = dbToLinear(levelDb);
            const auto tone = genSine(1000.0, amplitude, 1.0);
            const auto result = processMono(tone, kSampleRate, 0.0, thresholdDb, ratio, fastAttackMs, slowReleaseMs, kneeDb, 0.0);

            const size_t settleStart = size_t((slowReleaseMs / 1000.0) * 10.0 * kSampleRate);
            const size_t n = tone.size();

            // Method 1: the engine's own reported gain reduction, averaged.
            const double reportedGrDb = meanOfWindow(result.gainReductionDb, settleStart, n);
            // Method 2: independent -- plain RMS ratio of actual output
            // samples vs. actual input samples over the same window. For a
            // near-constant multiplicative gain this must match Method 1
            // even though it never looks at voice.gainReductionDb() at all.
            const double measuredGrDb = rmsDbOfWindow(result.output, settleStart, n) - rmsDbOfWindow(tone, settleStart, n);
            const double expectedGrDb = computeGainReductionDb(levelDb, thresholdDb, ratio, 1.0, kneeDb);

            char label[128];
            std::snprintf(label, sizeof(label), "%.0fdBFS tone, fast attack: reported=%.2fdB measured=%.2fdB expected=%.2fdB",
                          levelDb, reportedGrDb, measuredGrDb, expectedGrDb);
            check(nearlyEqual(reportedGrDb, expectedGrDb, 0.15) && nearlyEqual(measuredGrDb, expectedGrDb, 0.15), label);
        }

        // Part 2: with a *musical* attack time (5ms) that is no longer
        // short relative to a 1kHz tone's 0.25ms quarter-period, the peak
        // follower can only partially rise before each peak passes -- a
        // real, expected characteristic of any peak-detecting envelope
        // (fast attack times, by design, respond to a signal's amplitude
        // *envelope* over tens of milliseconds, not to individual
        // audio-rate cycles), not a bug. This is deliberately checked and
        // pinned down rather than silently assumed away: the two
        // independent measurements (engine's own report vs. actual
        // measured audio) must still agree with *each other* -- proving
        // the real per-sample chain is self-consistent -- even though
        // neither will match the naive full-peak expectation from Part 1.
        {
            const double levelDb = -6.0;
            const auto tone = genSine(1000.0, dbToLinear(levelDb), 1.0);
            const auto result = processMono(tone, kSampleRate, 0.0, thresholdDb, ratio, attackMs, releaseMs, kneeDb, 0.0);
            const size_t settleStart = size_t((releaseMs / 1000.0) * 10.0 * kSampleRate);
            const size_t n = tone.size();

            const double reportedGrDb = meanOfWindow(result.gainReductionDb, settleStart, n);
            const double measuredGrDb = rmsDbOfWindow(result.output, settleStart, n) - rmsDbOfWindow(tone, settleStart, n);
            const double naiveExpectedGrDb = computeGainReductionDb(levelDb, thresholdDb, ratio, 1.0, kneeDb);

            char label[160];
            std::snprintf(label, sizeof(label),
                          "%.0fdBFS tone, musical %.0fms attack: reported=%.2fdB matches measured=%.2fdB (both reads less reduction than the naive %.2fdB peak estimate, as expected)",
                          levelDb, attackMs, reportedGrDb, measuredGrDb, naiveExpectedGrDb);
            check(nearlyEqual(reportedGrDb, measuredGrDb, 0.05) && reportedGrDb > naiveExpectedGrDb, label);
        }
    }

    printf("\n== Test B: attack/release timing on a real sine burst ==\n");
    {
        // Quiet lead-in (below threshold, envelope settles near it), then a
        // loud burst (well above threshold) long enough to fully settle,
        // then back to quiet -- so both attack and release get a real,
        // clean transition to measure against the configured times.
        const auto quiet = genSine(1000.0, dbToLinear(-40.0), 0.3);
        const auto loud = genSine(1000.0, dbToLinear(0.0), 1.0);
        const auto tail = genSine(1000.0, dbToLinear(-40.0), 1.0);
        const auto signal = concat({ quiet, loud, tail });
        const auto result = processMono(signal, kSampleRate, 0.0, thresholdDb, ratio, attackMs, releaseMs, kneeDb, 0.0);

        const size_t burstStart = quiet.size();
        const size_t burstEnd = quiet.size() + loud.size();
        const double grBeforeBurst = result.gainReductionDb[burstStart - 1];
        const double grAtBurstSettled = meanOfWindow(result.gainReductionDb, burstEnd - size_t(0.05 * kSampleRate), burstEnd);
        const double targetSwing = grAtBurstSettled - grBeforeBurst; // negative: more reduction

        // Time to cross 63% of the way from grBeforeBurst to grAtBurstSettled.
        const double attackTargetGr = grBeforeBurst + 0.632 * targetSwing;
        size_t attackCrossSample = burstStart;
        for (size_t i = burstStart; i < burstEnd; ++i)
            if (result.gainReductionDb[i] <= attackTargetGr) { attackCrossSample = i; break; }
        const double measuredAttackMs = 1000.0 * double(attackCrossSample - burstStart) / kSampleRate;
        // Generous tolerance: a real sine burst's rectified envelope isn't
        // a clean DC step (dsp_regression test [5] already pins the exact
        // closed-form step response down precisely) -- this test's job is
        // "does the real audio path land in the right ballpark," not
        // re-deriving that formula.
        check(measuredAttackMs > 0.0 && measuredAttackMs < attackMs * 3.0,
              "attack reaches 63% of its target gain reduction within ~3x the configured attack time");

        const double grDuringBurst = grAtBurstSettled;
        const double grAfterRelease = meanOfWindow(result.gainReductionDb, signal.size() - size_t(0.05 * kSampleRate), signal.size());
        const double releaseTargetGr = grDuringBurst + 0.632 * (grAfterRelease - grDuringBurst);
        size_t releaseCrossSample = burstEnd;
        for (size_t i = burstEnd; i < signal.size(); ++i)
            if (result.gainReductionDb[i] >= releaseTargetGr) { releaseCrossSample = i; break; }
        const double measuredReleaseMs = 1000.0 * double(releaseCrossSample - burstEnd) / kSampleRate;
        check(measuredReleaseMs > 0.0 && measuredReleaseMs < releaseMs * 3.0,
              "release reaches 63% of its target gain reduction within ~3x the configured release time");

        wavio::writeWavMonoFloat((outDir / "mono_attack_release_dry.wav").string(), kSampleRate, signal);
        wavio::writeWavMonoFloat((outDir / "mono_attack_release_wet.wav").string(), kSampleRate, result.output);
    }

    printf("\n== Test C: stereo modes are actually distinct ==\n");
    {
        // Sub-test 1: panned hard left (R silent). Mid = Side = L/2 here --
        // numerically *identical* signals -- so any mode that runs the same
        // detector/curve function on both must produce identical Mid'/Side'
        // too, which reconstructs back to exactly R'=Mid'-Side'=0. This is
        // correct, expected behavior for all three modes on this
        // particular (degenerate: Mid==Side) signal -- it does NOT yet
        // distinguish Mid/Side from Linked/Dual-Mono; sub-test 2 below,
        // with independently-leveled Mid and Side content, is what actually
        // does that.
        const auto loudL = genSine(300.0, dbToLinear(-3.0), 1.0);
        const auto silentR = genSilence(1.0);

        const auto linked = processStereo(loudL, silentR, kSampleRate, StereoMode::Linked, thresholdDb, ratio, attackMs, releaseMs, kneeDb);
        const auto dualMono = processStereo(loudL, silentR, kSampleRate, StereoMode::DualMono, thresholdDb, ratio, attackMs, releaseMs, kneeDb);
        const auto midSide = processStereo(loudL, silentR, kSampleRate, StereoMode::MidSide, thresholdDb, ratio, attackMs, releaseMs, kneeDb);

        bool linkedRStaysSilent = true, midSideRStaysSilent = true;
        for (float s : linked.outR) if (s != 0.0f) { linkedRStaysSilent = false; break; }
        for (float s : midSide.outR) if (std::fabs(s) > 1e-5f) { midSideRStaysSilent = false; break; }
        check(linkedRStaysSilent, "Linked mode: a hard-panned silent channel stays exactly silent (shared gain, no cross-feed)");
        check(midSideRStaysSilent, "Mid/Side mode: a hard-panned silent channel stays silent too (Mid==Side here, so they compress identically)");

        // Dual-Mono: the silent channel's own detector never sees any
        // signal, so its voice must report exactly 0dB reduction throughout
        // -- proving the two channels are genuinely independent, not just
        // "R happens to already be silent so nothing shows either way."
        bool dualMonoRChannelUntouched = true;
        for (double gr : dualMono.gainReductionB) if (!nearlyEqual(gr, 0.0, 1e-9)) { dualMonoRChannelUntouched = false; break; }
        check(dualMonoRChannelUntouched, "Dual-Mono mode: the silent R channel's own detector reports zero reduction (fully independent)");

        wavio::writeWavStereoFloat((outDir / "stereo_hardpan_dry.wav").string(), kSampleRate, loudL, silentR);
        wavio::writeWavStereoFloat((outDir / "stereo_hardpan_linked_wet.wav").string(), kSampleRate, linked.outL, linked.outR);
        wavio::writeWavStereoFloat((outDir / "stereo_hardpan_dualmono_wet.wav").string(), kSampleRate, dualMono.outL, dualMono.outR);
        wavio::writeWavStereoFloat((outDir / "stereo_hardpan_midside_wet.wav").string(), kSampleRate, midSide.outL, midSide.outR);

        // Sub-test 2: a mostly-centered signal with a little width -- loud
        // "center" content (Mid) well above threshold, quiet "width"
        // content (Side) well below it. Both channels are just different
        // constant multiples of the same waveform (L=(A+B)sin, R=(A-B)sin),
        // so Linked's single shared detector sees a signal proportional to
        // |L| at every instant and must preserve the exact L/R ratio --
        // whereas Mid/Side compresses the loud Mid content while leaving
        // the quiet Side content (below the knee, so exactly 0dB reduction,
        // deterministic) alone, which *changes* that ratio: this is the
        // actual, audible thing "Mid/Side bus compression narrows or widens
        // the stereo image" means, and the concrete way this mode differs
        // from the other two.
        constexpr double centerAmp = /* -3dBFS */ 0.70794578;
        constexpr double sideAmp = /* -36dBFS, cleanly below thr-knee/2=-21dB, so gain reduction is exactly 0 */ 0.01584893;
        const auto center = genSine(300.0, centerAmp, 1.0);
        const auto side = genSine(300.0, sideAmp, 1.0);
        std::vector<float> wideL(center.size()), wideR(center.size());
        for (size_t i = 0; i < center.size(); ++i) { wideL[i] = center[i] + side[i]; wideR[i] = center[i] - side[i]; }

        const auto linkedWide = processStereo(wideL, wideR, kSampleRate, StereoMode::Linked, thresholdDb, ratio, attackMs, releaseMs, kneeDb);
        const auto midSideWide = processStereo(wideL, wideR, kSampleRate, StereoMode::MidSide, thresholdDb, ratio, attackMs, releaseMs, kneeDb);

        const size_t settleStart = size_t((releaseMs / 1000.0) * 10.0 * kSampleRate);
        const size_t n = wideL.size();

        // Linked preserves the exact L/R ratio (same scalar gain both sides).
        double worstRatioErr = 0.0;
        const double expectedRatio = (centerAmp - sideAmp) / (centerAmp + sideAmp);
        for (size_t i = settleStart; i < n; ++i)
        {
            if (std::fabs(linkedWide.outL[i]) < 1e-6f) continue;
            worstRatioErr = std::max(worstRatioErr, std::fabs(double(linkedWide.outR[i]) / double(linkedWide.outL[i]) - expectedRatio));
        }
        check(worstRatioErr < 0.01, "Linked mode preserves the exact L/R ratio (single shared gain)");

        // Mid/Side: reconstructed Side must stay ~equal to the original
        // (untouched, below the knee), while reconstructed Mid must shrink
        // (compressed) -- i.e. the mode treats center and width completely
        // differently, unlike Linked just above.
        double sideRms = 0.0, midRmsOut = 0.0, sideRmsOut = 0.0;
        for (size_t i = settleStart; i < n; ++i)
        {
            sideRms += double(side[i]) * double(side[i]);
            const double midOut = (double(midSideWide.outL[i]) + double(midSideWide.outR[i])) * 0.5;
            const double sideOut = (double(midSideWide.outL[i]) - double(midSideWide.outR[i])) * 0.5;
            midRmsOut += midOut * midOut;
            sideRmsOut += sideOut * sideOut;
        }
        const size_t count = n - settleStart;
        sideRms = std::sqrt(sideRms / double(count));
        midRmsOut = std::sqrt(midRmsOut / double(count));
        sideRmsOut = std::sqrt(sideRmsOut / double(count));
        const double centerRms = centerAmp / std::sqrt(2.0); // sine RMS = peak/sqrt(2)

        check(nearlyEqual(linearToDb(sideRmsOut), linearToDb(sideRms), 0.05),
              "Mid/Side mode: the quiet Side content (below the knee) passes through unchanged");
        check(linearToDb(midRmsOut) < linearToDb(centerRms) - 3.0,
              "Mid/Side mode: the loud Mid content gets substantially compressed");

        wavio::writeWavStereoFloat((outDir / "stereo_wide_dry.wav").string(), kSampleRate, wideL, wideR);
        wavio::writeWavStereoFloat((outDir / "stereo_wide_linked_wet.wav").string(), kSampleRate, linkedWide.outL, linkedWide.outR);
        wavio::writeWavStereoFloat((outDir / "stereo_wide_midside_wet.wav").string(), kSampleRate, midSideWide.outL, midSideWide.outR);
    }

    printf("\n%s: %d test(s) failed\n", g_failures == 0 ? "PASS" : "FAILURES", g_failures);
    printf("WAV files written to %s (listen for yourself)\n", outDir.string().c_str());
    return g_failures == 0 ? 0 : 1;
}
