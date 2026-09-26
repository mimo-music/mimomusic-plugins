// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// generate_reference_samples.cpp
//
// CLI front-end for the offline, physically-grounded reference reverb
// (ReferenceEngine.hpp / ImageSourceMethod.hpp) -- stage 1, a model that
// only produces test samples. Not a
// real-time plugin, no DPF dependency -- same "plain executable" shape
// as tests/reverb_regression*.cpp and ../../statespace/tools/trainer/
// fit_gain_curve.cpp, which this tool's own eventual fitting step
// (tools/trainer/, not built yet -- deliberately) will
// consume the .wav pairs this tool writes.
//
// Usage:
//   generate_reference_samples <out_dir> [options]
//     --length M          room length, meters (default 10.0)
//     --width M            room width, meters (default 7.0)
//     --height M           room height, meters (default 3.5)
//     --absorption A        wall absorption 0..1 (default 0.3)
//     --target-t60 SEC       convenience: back-solve --absorption from
//                            this via Sabine's equation instead (T60 =
//                            0.161*V/(S*absorption) -- an approximation,
//                            not a guarantee of the rendered result's own
//                            measured T60, since Sabine's own theory only
//                            strictly holds in the high-order diffuse
//                            limit; see ReferenceEngine.hpp's own header
//                            comment)
//     --order N              max image-source reflection order (default
//                            3 -- fast, a genuine but sparse early-
//                            reflection cloud; try 15-30+ for something
//                            closer to a full diffuse tail -- this is the
//                            "hours of calculation" mode the user
//                            explicitly said was fine, so it's opt-in via
//                            a higher number here, not the default)
//     --sample-rate HZ        (default 48000)
//     --user-dry-dir DIR      look here for real dry recordings instead
//                            of <out_dir>/user_dry -- lets a batch of
//                            many single-invocation runs (see
//                            tools/reference/batch_export.sh) all share
//                            ONE dry-file folder instead of needing it
//                            duplicated into every run's own out_dir
//     --dry-out-dir DIR       write "<name>_dry.wav" files here instead
//                            of <out_dir> (default: <out_dir>, i.e. no
//                            change for a single standalone invocation).
//                            The dry signal (synthesized excitation, or a
//                            user file's own mono downmix) depends only
//                            on the excitation generator + sample rate,
//                            never on room/absorption/order -- identical
//                            across every point of a batch_export.sh
//                            sweep that shares one --sample-rate. So a
//                            sweep passes ONE shared --dry-out-dir across
//                            all its invocations, and this tool skips
//                            (re)writing a dry file that's already there
//                            instead of duplicating it into every run's
//                            own out_dir. Reference files (the actual
//                            per-room render) are unaffected -- always
//                            written fresh into <out_dir>.
//     --manifest-csv PATH     append one row per dry/reference pair
//                            written to this CSV (creating it with a
//                            header if it doesn't exist yet) -- the
//                            single source of truth for "which settings
//                            produced which file", read directly by
//                            tools/reference/batch_export.sh and by any
//                            future trainer, not reconstructed by
//                            parsing filenames
//
// Writes, into <out_dir>: for each excitation signal (impulse, clap,
// noisebusrt, sweep) a "<name>_dry.wav" (mono, duplicated to both
// channels) and "<name>_reference.wav" (stereo, the rendered output) --
// the same {input, target} pair shape ../../statespace/tools/trainer/
// fit_gain_curve.cpp already expects from its own two WAV arguments, so
// a future reverb-model trainer can consume these directly.
//
// Real dry recordings: this tool can only synthesize the standard
// acoustic MEASUREMENT signals (impulse/clap/noise/sweep) -- it has no
// way to fetch or license real dry music/vocals/instruments. If real dry
// material is available, drop it in <out_dir>/user_dry/ (or wherever
// --user-dry-dir points) and it will be rendered through the same
// reference engine alongside the synthesized signals (any .wav found
// there is picked up automatically).

#include "../../plugin/dsp/ReverbEngine.hpp"
#include "../testbench/Wav.hpp"
#include "ExcitationSignals.hpp"
#include "ImageSourceMethod.hpp"
#include "ReferenceEngine.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace ssfilter::reverb;
using namespace ssfilter::reverb::reference;
namespace fs = std::filesystem;

namespace {

// Startup sanity check: higherOrderImageSources() at maxOrder=1, with no
// receiver offset, must reproduce ReverbEngine.hpp's own
// firstOrderImageSources() -- same 6 taps, same delays/gains -- since
// both use the identical source/receiver placement and the order-1
// terms are exactly the 6 single-wall mirrors. Not just argued in a
// comment: checked here, every run, before anything is rendered.
bool sanityCheckAgainstFirstOrder(const RoomDims &room, double absorption) {
    const auto reference = firstOrderImageSources(room, absorption);
    const auto higherOrder1 = higherOrderImageSources(room, absorption, 1, 0.0);
    if (higherOrder1.size() != reference.size()) {
        std::fprintf(stderr, "sanity check FAILED: expected %zu order-1 taps, got %zu\n",
                     reference.size(), higherOrder1.size());
        return false;
    }
    // Order doesn't matter for this check -- match each reference tap to
    // its closest (delay, gain) counterpart in the higher-order set.
    std::vector<bool> matched(higherOrder1.size(), false);
    for (const auto &r : reference) {
        bool found = false;
        for (size_t i = 0; i < higherOrder1.size(); ++i) {
            if (matched[i]) continue;
            if (std::fabs(higherOrder1[i].delaySec - r.delaySec) < 1e-9 &&
                std::fabs(higherOrder1[i].gain - r.gain) < 1e-9) {
                matched[i] = true;
                found = true;
                break;
            }
        }
        if (!found) {
            std::fprintf(stderr, "sanity check FAILED: no order-1 match for delay=%.6f gain=%.6f\n",
                         r.delaySec, r.gain);
            return false;
        }
    }
    return true;
}

// Everything about ONE invocation's own settings, needed only to fill in
// the manifest row -- not passed to the DSP itself (that's still plain
// RoomDims/absorption/order, unchanged). `runDir` is recorded exactly as
// given on the command line (not resolved to an absolute path) so a
// manifest.csv living in the PARENT of many single-invocation runs (see
// tools/reference/batch_export.sh) can reconstruct each file's full
// relative path as `run_dir + "/" + dry_file` with no extra bookkeeping.
struct RunSettings {
    std::string runDir;
    double lengthM = 0.0, widthM = 0.0, heightM = 0.0, absorption = 0.0, sampleRate = 0.0;
    bool haveTargetT60 = false;
    double targetT60 = 0.0;
    int order = 0;
    std::string manifestCsvPath; // empty = no manifest requested
};

void appendManifestRow(const RunSettings &s, const std::string &signalName,
                        const std::string &dryFile, const std::string &refFile) {
    if (s.manifestCsvPath.empty()) return;
    const bool exists = fs::exists(s.manifestCsvPath);
    std::ofstream f(s.manifestCsvPath, std::ios::app);
    if (!f.good()) {
        std::fprintf(stderr, "warning: could not open manifest %s for writing -- this row will be missing\n",
                     s.manifestCsvPath.c_str());
        return;
    }
    if (!exists) {
        f << "run_dir,dry_file,reference_file,signal_name,length_m,width_m,height_m,"
             "absorption,target_t60_s,order,sample_rate\n";
    }
    f << s.runDir << ',' << dryFile << ',' << refFile << ',' << signalName << ','
      << s.lengthM << ',' << s.widthM << ',' << s.heightM << ',' << s.absorption << ','
      << (s.haveTargetT60 ? std::to_string(s.targetT60) : std::string()) << ','
      << s.order << ',' << s.sampleRate << '\n';
}

void writePair(const std::string &outDir, const std::string &dryOutDir, const std::string &name,
               const RunSettings &settings, const std::vector<float> &dry,
               const RoomDims &room, double absorption, int order) {
    const double sampleRate = settings.sampleRate;
    std::printf("  %-10s dry=%.3fs\n", name.c_str(), (double)dry.size() / sampleRate);
    const auto rendered = renderReferenceStereo(dry, sampleRate, room, absorption, order);

    const fs::path dryPath = fs::path(dryOutDir) / (name + "_dry.wav");
    const std::string refFile = name + "_reference.wav";
    // The dry signal is the same for every grid point of a sweep sharing
    // one --dry-out-dir/--sample-rate (see this file's own --dry-out-dir
    // help text) -- write it once, not once per room/T60/order combination.
    bool wroteDry = false;
    if (!fs::exists(dryPath)) {
        wavio::writeWavStereoFloat(dryPath.string(), sampleRate, dry, dry);
        wroteDry = true;
    }
    wavio::writeWavStereoFloat(outDir + "/" + refFile, sampleRate, rendered.wetL, rendered.wetR);
    // Recorded relative to outDir (not dryOutDir) so it composes the same
    // way reference_file does -- see appendManifestRow's/RunSettings::runDir's
    // own comment ("run_dir + '/' + dry_file" reconstructs the full path).
    // lexically_relative() needs both sides in the same absolute/relative
    // form to produce anything (batch_export.sh passes an absolute
    // --dry-out-dir alongside a plain relative run_dir, so mixing them
    // unresolved silently returns an empty path) -- fs::absolute() first,
    // lexically only (no filesystem touch, no symlink resolution needed).
    const std::string dryFileRel =
        fs::absolute(dryPath).lexically_relative(fs::absolute(fs::path(outDir))).generic_string();
    std::printf("    -> %s (%s), %s\n", dryFileRel.c_str(), wroteDry ? "written" : "already shared", refFile.c_str());
    appendManifestRow(settings, name, dryFileRel, refFile);
}

} // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr,
                     "usage: %s <out_dir> [--length M] [--width M] [--height M] "
                     "[--absorption A] [--target-t60 SEC] [--order N] [--sample-rate HZ] "
                     "[--user-dry-dir DIR] [--dry-out-dir DIR] [--manifest-csv PATH]\n",
                     argv[0]);
        return 1;
    }
    const std::string outDir = argv[1];

    RoomDims room{10.0, 7.0, 3.5};
    double absorption = 0.3;
    bool haveTargetT60 = false;
    double targetT60 = 1.8;
    int maxOrder = 3;
    double sampleRate = 48000.0;
    std::string userDryDir = outDir + "/user_dry";
    std::string dryOutDir; // resolved to outDir below once argv parsing is done, if not given
    std::string manifestCsvPath;

    for (int i = 2; i < argc; ++i) {
        auto nextNum = [&](void) -> double { return (i + 1 < argc) ? std::atof(argv[++i]) : 0.0; };
        auto nextStr = [&](void) -> std::string { return (i + 1 < argc) ? std::string(argv[++i]) : std::string(); };
        if (std::strcmp(argv[i], "--length") == 0) room.lengthM = nextNum();
        else if (std::strcmp(argv[i], "--width") == 0) room.widthM = nextNum();
        else if (std::strcmp(argv[i], "--height") == 0) room.heightM = nextNum();
        else if (std::strcmp(argv[i], "--absorption") == 0) absorption = nextNum();
        else if (std::strcmp(argv[i], "--target-t60") == 0) { targetT60 = nextNum(); haveTargetT60 = true; }
        else if (std::strcmp(argv[i], "--order") == 0) maxOrder = (int)nextNum();
        else if (std::strcmp(argv[i], "--sample-rate") == 0) sampleRate = nextNum();
        else if (std::strcmp(argv[i], "--user-dry-dir") == 0) userDryDir = nextStr();
        else if (std::strcmp(argv[i], "--dry-out-dir") == 0) dryOutDir = nextStr();
        else if (std::strcmp(argv[i], "--manifest-csv") == 0) manifestCsvPath = nextStr();
        else { std::fprintf(stderr, "unknown option: %s\n", argv[i]); return 1; }
    }
    if (dryOutDir.empty()) dryOutDir = outDir; // default: no change from the old single-invocation behavior
    room.clamp();

    if (haveTargetT60) {
        // Sabine's equation (metric): T60 = 0.161*V/(S*absorption) ->
        // absorption = 0.161*V/(S*T60). A starting-point convenience
        // only -- see this file's own header comment.
        const double V = room.lengthM * room.widthM * room.heightM;
        const double S = 2.0 * (room.lengthM * room.widthM + room.lengthM * room.heightM + room.widthM * room.heightM);
        absorption = std::clamp(0.161 * V / (S * std::max(0.05, targetT60)), 0.01, 0.99);
        std::printf("--target-t60 %.2fs -> back-solved absorption = %.4f (Sabine, an approximation)\n", targetT60, absorption);
    }

    std::printf("Room: %.1fm x %.1fm x %.1fm, wall absorption=%.3f, max image order=%d, sample rate=%.0fHz\n",
               room.lengthM, room.widthM, room.heightM, absorption, maxOrder, sampleRate);

    std::printf("Sanity check: order-1 image sources vs. ReverbEngine.hpp's own firstOrderImageSources()... ");
    if (!sanityCheckAgainstFirstOrder(room, absorption)) {
        std::fprintf(stderr, "\nABORTING -- the higher-order generalization does not agree with the "
                             "already-verified first-order function at order 1. This would mean "
                             "every render from this tool is untrustworthy.\n");
        return 1;
    }
    std::printf("OK (exact match)\n");

    // A rough up-front tap-count estimate, so a very high --order doesn't
    // silently turn into an unbounded-feeling wait -- see
    // ImageSourceMethod.hpp's own header comment on why this grows
    // roughly O(order^3).
    {
        const auto probe = higherOrderImageSources(room, absorption, maxOrder, 0.0);
        std::printf("Estimated tap count at order<=%d: %zu (per receiver)\n", maxOrder, probe.size());
    }

    RunSettings settings;
    settings.runDir = outDir;
    settings.lengthM = room.lengthM;
    settings.widthM = room.widthM;
    settings.heightM = room.heightM;
    settings.absorption = absorption;
    settings.sampleRate = sampleRate;
    settings.haveTargetT60 = haveTargetT60;
    settings.targetT60 = targetT60;
    settings.order = maxOrder;
    settings.manifestCsvPath = manifestCsvPath;

    std::error_code ec;
    fs::create_directories(outDir, ec);
    fs::create_directories(userDryDir, ec);
    fs::create_directories(dryOutDir, ec);

    std::printf("\nRendering synthesized excitation signals:\n");
    const double tailSec = 0.05; // real tail length comes from the taps themselves; this is just excitation-signal padding
    writePair(outDir, dryOutDir, "impulse", settings, impulseSignal(sampleRate, tailSec), room, absorption, maxOrder);
    writePair(outDir, dryOutDir, "clap", settings, clapSignal(sampleRate, tailSec), room, absorption, maxOrder);
    writePair(outDir, dryOutDir, "noiseburst", settings, noiseBurstSignal(sampleRate, 0.2, tailSec), room, absorption, maxOrder);
    writePair(outDir, dryOutDir, "sweep", settings, sineSweepSignal(sampleRate, 3.0, tailSec), room, absorption, maxOrder);

    // Any real dry material the user has dropped into --user-dry-dir
    // (default <out_dir>/user_dry).
    size_t userFiles = 0;
    for (const auto &entry : fs::directory_iterator(userDryDir, ec)) {
        if (entry.path().extension() != ".wav") continue;
        double fileRate = 0.0;
        std::vector<float> l, r;
        if (!wavio::readWavStereoFloat(entry.path().string(), fileRate, l, r)) {
            std::fprintf(stderr, "  (skipping %s -- could not read as WAV)\n", entry.path().string().c_str());
            continue;
        }
        if (fileRate != sampleRate) {
            std::fprintf(stderr, "  (skipping %s -- %.0fHz, expected %.0fHz; this tool does not resample)\n",
                        entry.path().string().c_str(), fileRate, sampleRate);
            continue;
        }
        std::printf("\nRendering user-supplied dry file: %s\n", entry.path().filename().string().c_str());
        std::vector<float> mono(l.size());
        for (size_t i = 0; i < l.size(); ++i) mono[i] = 0.5f * (l[i] + r[i]);
        const std::string name = "user_" + entry.path().stem().string();
        writePair(outDir, dryOutDir, name, settings, mono, room, absorption, maxOrder);
        ++userFiles;
    }
    if (userFiles == 0) {
        std::printf("\n(no files found in %s -- drop real dry recordings there and re-run "
                    "to render them through the same reference engine)\n", userDryDir.c_str());
    }

    std::printf("\nDone. Wrote %s dry/reference pairs into %s\n",
               std::to_string(4 + userFiles).c_str(), outDir.c_str());
    return 0;
}
