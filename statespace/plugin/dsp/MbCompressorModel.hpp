// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MbCompressorModel.hpp
//
// The "musical" parameter layer for the multiband compressor plugin
// (plugin_mb_compressor/PluginMbCompressor.cpp, gui/mb/UIMbCompressor.cpp)
// -- a list of crossover split frequencies plus one MbBand (threshold/
// ratio/attack/release/makeup/enabled/solo) per resulting band, exactly
// the same "musical parameters -> coefficient mapper -> StateSpaceMatrices"
// pattern EqModel.hpp already is for the sibling EQ plugin (see that
// file's own header comment, and docs/EXTENDING.md's "Building an LTI
// processor" section for the general shape this follows).
//
// assembleStateSpace() below does the actual work in two calls to
// already-tested machinery: mappers::makeStereoCrossoverBank()
// (CoefficientMapper.hpp) builds the Linkwitz-Riley crossover -- an
// ordinary LTI parallel filter bank, flat/inaudible whenever every band's
// gain is 0dB -- and each band's own musical threshold/ratio/attack/
// release/makeup is turned into a NonlinearSchedule via
// GainCurve::classicCompressor() (StateSpaceTypes.hpp) and dropped into
// the resulting MultibandSchedule. No new DSP math at all; this file is
// pure bookkeeping over what plugin/dsp/CoefficientMapper.hpp and
// StateSpaceTypes.hpp already provide -- see docs/ARCHITECTURE.md's
// "MultibandSchedule" section for the full mechanism.
//
// Like EqModel, this has no fixed-size backing store of its own (an
// unbounded-in-principle band list, capped in practice by kMaxBands) --
// only what it assembles into is fixed-size. `bands.size()` is always
// `crossoverHz.size() + 1`; addCrossover()/removeCrossover() are the only
// supported ways to change the band count, specifically so that
// invariant can never drift out of sync (a caller that spliced `bands`
// or `crossoverHz` directly could break it).

#pragma once

#include "CoefficientMapper.hpp"
#include "StateSpaceTypes.hpp"

#include <complex>
#include <string>
#include <vector>

namespace ssfilter {

struct MbBand
{
    double thresholdDb = -20.0;
    double ratio = 4.0;
    double ratioUp = 1.0; // upward-compression ratio, mirrors `ratio` below the threshold; 1.0 = no upward effect (the historical default)
    double attackMs = 10.0;
    double releaseMs = 100.0;
    double makeupDb = 0.0;
    bool enabled = true; // false: this band's own audio still plays, but flat/uncompressed (dynamics bypassed)
};

struct MbCompressorModel
{
    std::vector<double> crossoverHz; // ascending, size == bands.size() - 1
    std::vector<MbBand> bands;       // size == crossoverHz.size() + 1, kept in sync by the mb:: functions below
    // Linkwitz-Riley crossover order/steepness (even, >= 2), applied
    // uniformly to *every* crossover split -- see designCrossoverBandStages().
    // Exposed to the user as the same four discrete choices (12/24/36/48
    // dB/oct) as the sibling EQ plugin's Low/High Pass "Slope" control --
    // see mb::clampLrOrder()/nextLrOrder()/lrOrderLabel() below.
    int lrOrder = 4;
    // Which band (if any) is currently soloed -- a single "spotlight"
    // index into `bands`, not a per-band flag: this UI only ever shows
    // one band's own controls at a time (the *selected* one), so unlike
    // the sibling EQ plugin's BandListWidget (which shows every band at
    // once and genuinely benefits from soloing more than one
    // simultaneously), a per-band solo flag here just went stale the
    // moment you selected a different band to work on -- reported
    // directly: "if solo is activated and the band is changed the solo
    // should switch to this band. its not a setting that is to each
    // band." UIMbCompressor::selectBand() moves this index along with the
    // selection whenever solo is engaged, so the two can never disagree.
    // -1 = no solo active. See mb::hasSolo().
    int soloedBand = -1;
    double outputTrimDb = 0.0;       // applied uniformly to C/D after assembly, same convention as EqModel's
    char name[64] = "Untitled Multiband";

    MbCompressorModel() { bands.emplace_back(); } // one band, no splits -- a transparent single-band compressor by default
};

namespace mb {

// True if `soloedBand` names a real, currently-*enabled* band -- soloing
// a band that's also disabled has no effect (nothing to spotlight),
// mirroring dyneq::activeBandIndices()'s identical edge case.
bool hasSolo(const MbCompressorModel& model);

// Designs one band's NonlinearSchedule from its musical parameters.
// `audible` is false when this band is silenced by the soloed band (see
// hasSolo()) -- forced to attenuate hard at every level rather
// than needing a structural "this band doesn't exist" special case
// anywhere else. A *disabled* (but audible) band gets a flat, transparent
// 0dB curve -- its own audio still plays, only the dynamics are bypassed.
NonlinearSchedule designBandSchedule(const MbBand& band, bool audible);

// Assembles model into an ordinary StateSpaceMatrices via
// mappers::makeStereoCrossoverBank() + designBandSchedule() per band, then
// applies outputTrimDb to C/D (never A/B, same convention as EqModel's own
// output trim). Caller is still expected to run StabilityGuard::enforce()
// before StateSpaceEngine::setModel(), same as any other mapper output --
// normally a no-op here (a crossover built from RBJ Butterworth sections is
// stable by construction) but free insurance against extreme edits.
StateSpaceMatrices assembleStateSpace(const MbCompressorModel& model, double sampleRate);

// The crossover's closed-form combined response -- the *sum* of each
// band's own biquad-chain product (a parallel connection, unlike
// EqModel's series cascade, so bands sum rather than multiply) -- used by
// the interactive graph instead of the generic O(order^3)
// TransferFunction sweep, for the same reason EqModel.hpp's
// evaluateCombinedResponse() exists (see its own file comment). This
// deliberately does not reflect each band's own gain curve (a per-level,
// not per-frequency, quantity) -- it's the *crossover's* shape, which is
// what the graph draws; dsp_regression test [18] confirms it agrees with
// TransferFunction::evaluate() on the assembled matrix when every band is
// transparent (0dB), as it must for the same LTI system in that case.
std::complex<double> evaluateCombinedResponse(const MbCompressorModel& model, double sampleRate, double freqHz);

// ".ssmbjson" -- this plugin's own band-list preset format (distinct from
// the base plugin's .ssjson, which stores raw matrices; an
// MbCompressorModel has no matrices of its own to store, only the musical
// parameters they're derived from -- same convention as EqModel's own
// ".sseqjson"). See UIMbCompressor's "Export to State-Space..." button for
// writing an actual .ssjson instead, via
// Serialization::toJson(assembleStateSpace(...)).
std::string toJson(const MbCompressorModel& model);
bool fromJson(const std::string& text, MbCompressorModel& out);

// Which band index (0..bands.size()-1) a frequency currently falls in.
int bandIndexForFreq(const MbCompressorModel& model, double freqHz);

// Linkwitz-Riley crossover order helpers -- the same four discrete
// choices (12/24/36/48 dB/oct) as the sibling EQ plugin's Low/High Pass
// "Slope" control (EqBand::slopeStages, 1-4), just expressed directly as
// filter order (2/4/6/8) since designCrossoverBandStages() already takes
// a raw order rather than a stage count.
int clampLrOrder(int order);         // snaps to the nearest of {2,4,6,8} -- used by fromJson() against hand-edited/foreign data
int nextLrOrder(int order);          // cycles 2 -> 4 -> 6 -> 8 -> 2, same convention as BandListWidget.cpp's nextSlopeStages()
const char* lrOrderLabel(int order); // "12 dB/oct" .. "48 dB/oct"

// Splits whichever band currently contains freqHz into two, inserting a
// new crossover point there; the new upper half starts as a copy of the
// original band's settings (splitting a band shouldn't reset anything the
// user already tuned). Silently refuses past kMaxBands, same "refuse
// rather than silently truncate" convention as UIEq::addBand().
void addCrossover(MbCompressorModel& model, double freqHz);

// Removes crossover point `crossoverIndex` (0..crossoverHz.size()-1),
// merging the two bands it separated back into one -- the lower band's
// own settings are kept (an arbitrary but deterministic choice; unlike
// EqModel's band removal, which just deletes one band, a merge has to
// pick one side's settings to keep).
void removeCrossover(MbCompressorModel& model, int crossoverIndex);

// Moves an *existing* crossover point (e.g. a graph drag), clamped to
// stay strictly between its neighbors (or the 20Hz/20kHz outer bounds)
// so bands can never cross over or collapse to zero width.
void setCrossoverHz(MbCompressorModel& model, int crossoverIndex, double freqHz);

} // namespace mb
} // namespace ssfilter
