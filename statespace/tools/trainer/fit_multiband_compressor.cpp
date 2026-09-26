// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// fit_multiband_compressor.cpp
//
// CLI front-end for GainCurveTrainer::fitMultiband() (see
// plugin/dsp/GainCurveTrainer.hpp for the actual fitting algorithm -- it's
// pure orchestration around the exact same per-band least-squares fit
// tools/trainer/fit_gain_curve.cpp already uses, called once per band).
//
// Usage:
//   fit_multiband_compressor <crossover_hz_csv> <input.wav> <target_output.wav> <output_preset.ssjson> [lr_order]
//
// `crossover_hz_csv` is a comma-separated, ascending list of crossover
// frequencies, e.g. "150,2000" for a 3-band split (bass / low-mid+high-mid
// / treble) at 150Hz and 2kHz. `input.wav`/`target_output.wav` must be the
// same length and sample rate (see Wav.hpp; no resampling is done).
// `lr_order` is the Linkwitz-Riley crossover order (default 4, i.e. LR4 /
// 24dB per octave, the most common real-world choice); must be even and
// >= 2.
//
// The output preset's A/B/C/D *is* the assembled crossover bank itself
// (mappers::makeStereoCrossoverBank()) -- so with every band's fitted gain
// curve at a transparent 0dB it's just an ordinary, always-flat crossover
// filter, exactly like any other .ssjson; its `multiband` block carries
// the N fitted per-band NonlinearSchedules on top of that. Same
// "the preset is the whole story, no bespoke format" convention
// fit_gain_curve.cpp already follows for the single-band case.

#include "../../plugin/dsp/CoefficientMapper.hpp"
#include "../../plugin/dsp/GainCurveTrainer.hpp"
#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../testbench/Wav.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace ssfilter;

namespace {

bool parseCrossoverList(const std::string& csv, std::vector<double>& out)
{
    out.clear();
    std::stringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ','))
    {
        if (tok.empty()) continue;
        try { out.push_back(std::stod(tok)); }
        catch (...) { return false; }
    }
    for (size_t i = 1; i < out.size(); ++i)
        if (!(out[i] > out[i - 1])) return false; // must be strictly ascending
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc != 5 && argc != 6)
    {
        std::fprintf(stderr,
                      "usage: %s <crossover_hz_csv> <input.wav> <target_output.wav> <output_preset.ssjson> [lr_order=4]\n"
                      "  example: %s \"150,2000\" mix_in.wav mix_compressed.wav trained_mb.ssjson\n",
                      argv[0], argv[0]);
        return 1;
    }
    const std::string crossoverCsv = argv[1], inputPath = argv[2], targetPath = argv[3], outputPath = argv[4];
    const int lrOrder = argc == 6 ? std::atoi(argv[5]) : 4;
    if (lrOrder < 2 || (lrOrder % 2) != 0)
    {
        std::fprintf(stderr, "error: lr_order must be an even number >= 2 (got %d)\n", lrOrder);
        return 1;
    }

    std::vector<double> crossoverHz;
    if (!parseCrossoverList(crossoverCsv, crossoverHz))
    {
        std::fprintf(stderr, "error: could not parse '%s' as an ascending comma-separated Hz list\n",
                      crossoverCsv.c_str());
        return 1;
    }
    const size_t numBands = crossoverHz.size() + 1;
    if (numBands > size_t(kMaxBands))
    {
        std::fprintf(stderr,
                      "error: %zu crossover points means %zu bands, which is more than this build supports "
                      "(kMaxBands=%d) -- refusing rather than silently dropping bands\n",
                      crossoverHz.size(), numBands, kMaxBands);
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

    std::printf("Fitting: %zu bands, crossovers=[%s] Hz, LR%d\n", numBands, crossoverCsv.c_str(), lrOrder);
    std::printf("         input=%s (%zu frames @ %.0f Hz)\n", inputPath.c_str(), inL.size(), inputRate);
    std::printf("         target=%s (%zu frames)\n", targetPath.c_str(), tgtL.size());

    const auto bandStages = mappers::designCrossoverBandStages(inputRate, crossoverHz, lrOrder);

    const GainCurveTrainer::MultibandFitResult result =
        GainCurveTrainer::fitMultiband(bandStages, inputRate, inL, inR, tgtL, tgtR);

    if (!result.ok)
    {
        std::fprintf(stderr, "error: fit failed for at least one band -- not enough non-silent overlap in "
                              "that band between input and target to learn a curve from\n");
        return 1;
    }

    std::printf("\nFit result (%d bands):\n", result.numBands);
    for (int k = 0; k < result.numBands; ++k)
    {
        const double lo = k == 0 ? 0.0 : crossoverHz[size_t(k - 1)];
        const double hi = k + 1 < result.numBands ? crossoverHz[size_t(k)] : inputRate / 2.0;
        std::printf("  band %d [%.0f-%.0f Hz]: attack=%.2fms release=%.2fms residual RMS=%.3fdB samplesUsed=%u\n",
                     k, lo, hi, result.schedule.bands[k].schedule.attackMs,
                     result.schedule.bands[k].schedule.releaseMs, result.rmsErrorDb[k], result.samplesUsed[k]);
    }

    MultibandSchedule layout;
    StateSpaceMatrices out = mappers::makeStereoCrossoverBank(bandStages, layout);
    out.multiband = result.schedule;
    std::snprintf(out.name, sizeof(out.name), "%s", "Trained Multiband Compressor");
    StabilityGuard::enforce(out); // no-op on stability (the gain stage doesn't touch A), keeps the habit consistent

    std::ofstream of(outputPath, std::ios::binary | std::ios::trunc);
    if (!of.good()) { std::fprintf(stderr, "error: cannot write %s\n", outputPath.c_str()); return 1; }
    of << Serialization::toJson(out);
    std::printf("\nWrote %s\n", outputPath.c_str());
    return 0;
}
