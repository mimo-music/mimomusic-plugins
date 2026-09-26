// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// fit_gain_curve.cpp
//
// CLI front-end for GainCurveTrainer (see plugin/dsp/GainCurveTrainer.hpp
// for the actual fitting algorithm and why it's a well-posed, cheap
// problem rather than a general machine-learning one).
//
// Usage:
//   fit_gain_curve <base_preset.ssjson> <input.wav> <target_output.wav> <output_preset.ssjson>
//
// `base_preset.ssjson` supplies the fixed A/B/C/D the fitted gain curve
// will multiply -- e.g. passthrough.ssjson to fit a "pure" compressor, or
// any filter preset to fit a compressor that also carries that filter's
// character. `input.wav` and `target_output.wav` must be the same length
// and sample rate (16/24/32-bit PCM or 32-bit float, mono or stereo, any
// sample rate -- whatever the files themselves specify, no resampling is
// done, so both must already match). The result is `base_preset.ssjson`
// with its `nonlinear` block replaced by the fitted schedule, written to
// `output_preset.ssjson`.

#include "../../plugin/dsp/GainCurveTrainer.hpp"
#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../testbench/Wav.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

using namespace ssfilter;

int main(int argc, char** argv)
{
    if (argc != 5)
    {
        std::fprintf(stderr,
                      "usage: %s <base_preset.ssjson> <input.wav> <target_output.wav> <output_preset.ssjson>\n",
                      argv[0]);
        return 1;
    }
    const std::string basePath = argv[1], inputPath = argv[2], targetPath = argv[3], outputPath = argv[4];

    std::ifstream bf(basePath);
    if (!bf.good()) { std::fprintf(stderr, "error: cannot open %s\n", basePath.c_str()); return 1; }
    std::ostringstream bss; bss << bf.rdbuf();
    StateSpaceMatrices baseModel;
    if (!Serialization::fromJson(bss.str(), baseModel))
    {
        std::fprintf(stderr, "error: %s is not a valid .ssjson preset\n", basePath.c_str());
        return 1;
    }

    double inputRate = 0.0, targetRate = 0.0;
    std::vector<float> inL, inR, tgtL, tgtR;
    if (!wavio::readWavStereoFloat(inputPath, inputRate, inL, inR))
    {
        std::fprintf(stderr, "error: could not read %s as WAV\n", inputPath.c_str());
        return 1;
    }
    if (!wavio::readWavStereoFloat(targetPath, targetRate, tgtL, tgtR))
    {
        std::fprintf(stderr, "error: could not read %s as WAV\n", targetPath.c_str());
        return 1;
    }
    if (inputRate != targetRate)
    {
        std::fprintf(stderr,
                      "error: sample rate mismatch (%s: %.0f Hz, %s: %.0f Hz) -- "
                      "this tool does not resample, please supply matching files\n",
                      inputPath.c_str(), inputRate, targetPath.c_str(), targetRate);
        return 1;
    }

    std::printf("Fitting: base=%s (order %d, %s)\n", basePath.c_str(), baseModel.order, baseModel.name);
    std::printf("         input=%s (%zu frames @ %.0f Hz)\n", inputPath.c_str(), inL.size(), inputRate);
    std::printf("         target=%s (%zu frames)\n", targetPath.c_str(), tgtL.size());

    const GainCurveFitResult result = GainCurveTrainer::fit(baseModel, inputRate, inL, inR, tgtL, tgtR);
    if (!result.ok)
    {
        std::fprintf(stderr,
                      "error: fit failed -- not enough non-silent overlap between input and target "
                      "to learn a curve from (samplesUsed=%u)\n", result.samplesUsed);
        return 1;
    }

    std::printf("\nFit result:\n");
    std::printf("  attack  = %.2f ms\n", result.schedule.attackMs);
    std::printf("  release = %.2f ms\n", result.schedule.releaseMs);
    std::printf("  samples used = %u\n", result.samplesUsed);
    std::printf("  residual RMS = %.3f dB  (lower is a better fit; <1 dB is a good match,\n"
                 "                            several dB suggests the target isn't a pure\n"
                 "                            level-dependent-gain effect, or attack/release\n"
                 "                            outside the search grid)\n", result.rmsErrorDb);
    std::printf("\n  learned curve (input level dB -> gain dB):\n");
    for (int i = 0; i < GainCurve::kNumKnots; i += 2) // every other knot, to keep this readable
        std::printf("    %+6.1f dB -> %+6.2f dB\n", GainCurve::knotLevelDb(i), result.schedule.curve.gainDb[i]);

    StateSpaceMatrices out = baseModel;
    out.nonlinear = result.schedule;
    StabilityGuard::enforce(out); // no-op on stability (nonlinear doesn't touch A), but keeps the habit consistent

    std::ofstream of(outputPath, std::ios::binary | std::ios::trunc);
    if (!of.good()) { std::fprintf(stderr, "error: cannot write %s\n", outputPath.c_str()); return 1; }
    of << Serialization::toJson(out);
    std::printf("\nWrote %s\n", outputPath.c_str());
    return 0;
}
