// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "DynamicEqModel2.hpp"
#include "EqModel.hpp" // for eq::evaluateBiquadResponse(), see this file's own use in evaluateBandResponse()
#include "Json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace ssfilter {
namespace dyneq2 {

namespace {
// Q/freq clamping identical to eq::designBand()'s own -- same reasoning
// (keep the RBJ-derived formulas away from undefined territory; the
// engine's own per-sample processing has no separate stability guard to
// fall back on the way the generic StateSpaceEngine path does, since A(z)
// is fixed and gain never touches it -- see file comment).
double clampedQ(double q) { return std::max(0.05, std::min(100.0, q)); }
double clampedFreq(double freqHz, double sampleRate) { return std::max(1.0, std::min(sampleRate * 0.499, freqHz)); }

// First-order allpass, A(z) = (a + z^-1) / (1 + a*z^-1), a = (K-1)/(K+1),
// K = tan(pi*fc/fs) -- the standard bilinear-transform first-order
// allpass (Zolzer's *DAFX*, the same reference this file's own header
// comment already cites for the 2nd-order Bell case), used here as the
// building block for a Low/High Shelf via the identical Regalia-Mitra
// combination Bell already uses (see shaperAllpassSign()'s own comment
// for the sign that turns this into one shape or the other).
// Unconditionally stable for ANY corner frequency 0 < fc < fs/2: K then
// ranges over (0, +inf), so `a` stays strictly within (-1, 1) -- the
// pole at z=-a can never reach or cross the unit circle, no separate
// clamp or stability guard needed, by construction rather than by
// checking.
BiquadCoeffs designFirstOrderAllpass(double sampleRate, double fc)
{
    const double k = std::tan(M_PI * fc / sampleRate);
    const double a = (k - 1.0) / (k + 1.0);
    BiquadCoeffs c;
    c.b0 = a; c.b1 = 1.0; c.b2 = 0.0;
    c.a1 = a; c.a2 = 0.0;
    return c;
}
} // namespace

std::vector<int> activeBandIndices(const DynamicEqModel2& model)
{
    // Soloing a disabled band has no effect (nothing to spotlight) --
    // same edge case as mb::hasSolo()'s identical guard.
    const bool soloActive = model.soloedBand >= 0 && model.soloedBand < int(model.bands.size())
        && model.bands[size_t(model.soloedBand)].enabled;

    std::vector<int> out;
    for (int i = 0; i < int(model.bands.size()); ++i)
    {
        const DynamicEqBand2& b = model.bands[size_t(i)];
        if (!b.enabled) continue;
        if (soloActive && i != model.soloedBand) continue;
        out.push_back(i);
    }
    return out;
}

int nextHarmonicGroupId(const DynamicEqModel2& model)
{
    int highest = 0;
    for (const DynamicEqBand2& b : model.bands) highest = std::max(highest, b.harmonicGroup);
    return highest + 1;
}

void propagateHarmonicFreq(DynamicEqModel2& model, int index, double freqHz, double sampleRate)
{
    if (index < 0 || index >= int(model.bands.size()) || !std::isfinite(freqHz)) return;
    const double clamped = clampedFreq(freqHz, sampleRate);
    DynamicEqBand2& moved = model.bands[size_t(index)];
    moved.freqHz = clamped;

    if (moved.harmonicGroup == 0) return; // not linked to anything else -- nothing further to do

    const double base = clamped / std::max(1.0e-6, moved.harmonicRatio);
    for (DynamicEqBand2& other : model.bands)
    {
        if (&other == &moved || other.harmonicGroup != moved.harmonicGroup) continue;
        other.freqHz = clampedFreq(base * other.harmonicRatio, sampleRate);
    }
}

int createHarmonicTwin(DynamicEqModel2& model, int sourceIndex, double factor, double sampleRate)
{
    if (sourceIndex < 0 || sourceIndex >= int(model.bands.size())) return -1;
    if (!std::isfinite(factor) || factor <= 0.0) return -1;
    if (int(model.bands.size()) >= kMaxDynEqBands2) return -1;

    DynamicEqBand2& source = model.bands[size_t(sourceIndex)];
    if (source.harmonicGroup == 0)
    {
        source.harmonicGroup = nextHarmonicGroupId(model);
        source.harmonicRatio = 1.0;
    }

    DynamicEqBand2 twin = source; // copies every field -- "another filter same to the selected"
    twin.freqHz = clampedFreq(source.freqHz * factor, sampleRate);
    twin.harmonicGroup = source.harmonicGroup;
    twin.harmonicRatio = source.harmonicRatio * factor;

    const int newIndex = sourceIndex + 1;
    model.bands.insert(model.bands.begin() + newIndex, twin);
    return newIndex;
}

void unlinkHarmonic(DynamicEqModel2& model, int index)
{
    if (index < 0 || index >= int(model.bands.size())) return;
    model.bands[size_t(index)].harmonicGroup = 0;
}

void propagateHarmonicDynamics(DynamicEqModel2& model, int index)
{
    if (index < 0 || index >= int(model.bands.size())) return;
    const DynamicEqBand2& src = model.bands[size_t(index)];
    if (src.harmonicGroup == 0) return;
    const double thresholdDb = src.thresholdDb, ratio = src.ratio, ratioUp = src.ratioUp, rangeDb = src.rangeDb;
    for (int i = 0; i < int(model.bands.size()); ++i)
    {
        if (i == index || model.bands[size_t(i)].harmonicGroup != src.harmonicGroup) continue;
        DynamicEqBand2& other = model.bands[size_t(i)];
        other.thresholdDb = thresholdDb;
        other.ratio = ratio;
        other.ratioUp = ratioUp;
        other.rangeDb = rangeDb;
    }
}

bool hasPerChannelBands(const DynamicEqModel2& model)
{
    for (int i : activeBandIndices(model))
    {
        const DynamicEqBand2& b = model.bands[size_t(i)];
        if (b.channel != EqChannel::Both || b.crossFeed != EqCrossFeed::None) return true;
    }
    return false;
}

double bandShelfLowCornerHz(const DynamicEqBand2& band)
{
    const double bw = std::max(kMinBandwidthOct, std::min(kMaxBandwidthOct, band.bandwidthOct));
    return band.freqHz / std::pow(2.0, 0.5 * bw);
}

double bandShelfHighCornerHz(const DynamicEqBand2& band)
{
    const double bw = std::max(kMinBandwidthOct, std::min(kMaxBandwidthOct, band.bandwidthOct));
    return band.freqHz * std::pow(2.0, 0.5 * bw);
}

// A first-order Regalia-Mitra shelf's allpass corner is its POLE; its zero
// sits at pole/G (rise) or pole*G (fall), so the half-gain point -- the
// geometric mean of the two -- lands at pole/sqrt(G) or pole*sqrt(G).
// Uncompensated, a boost therefore widens a Band Shelf and a cut narrows it
// (a -10dB, 100Hz-2kHz band only reached -6.9dB). Placing each pole at
// corner*sqrt(G) (rise) and corner/sqrt(G) (fall), with G the band's
// RESTING gain, puts the half-gain points exactly on the corners at rest,
// for boost and cut alike -- the same meaning the corners have in the
// Parametric EQ's Band Shelf. The resting gain only changes on a user
// edit (smoothed like a frequency edit), never per sample, so the poles
// still never move with the live dynamic G.
static double bandShelfRestingSqrtG(const DynamicEqBand2& band)
{
    return std::pow(10.0, band.gainDb / 40.0);
}

BiquadCoeffs designShaperAllpass2(const DynamicEqBand2& band, double sampleRate)
{
    if (band.type == EqBandType::BandShelf)
        return designFirstOrderAllpass(sampleRate, clampedFreq(bandShelfHighCornerHz(band) / bandShelfRestingSqrtG(band), sampleRate));
    return designFirstOrderAllpass(sampleRate, clampedFreq(band.freqHz, sampleRate));
}

BiquadCoeffs designShaperAllpass(const DynamicEqBand2& band, double sampleRate)
{
    const double freq = clampedFreq(band.freqHz, sampleRate);
    if (band.type == EqBandType::BandShelf)
        return designFirstOrderAllpass(sampleRate, clampedFreq(bandShelfLowCornerHz(band) * bandShelfRestingSqrtG(band), sampleRate)); // A_lo, see designShaperAllpass2()'s comments
    if (band.type == EqBandType::LowShelf || band.type == EqBandType::HighShelf)
        return designFirstOrderAllpass(sampleRate, freq);
    return mappers::designAllpass(sampleRate, freq, clampedQ(band.q)); // Bell (the default -- also the fallback for any of the not-reachable-from-this-UI types, same shape as before `type` existed)
}

double shaperAllpassSign(EqBandType type)
{
    return type == EqBandType::LowShelf ? -1.0 : 1.0;
}

BiquadCoeffs designDetectorBandpass(const DynamicEqBand2& band, double sampleRate)
{
    if (band.type == EqBandType::BandShelf)
    {
        // Listens to the band's own region: the bandpass whose -3dB edges
        // land on the two corners (Q = sqrt(r)/(r-1), r = 2^bw) -- the same
        // Q the Parametric EQ's Band Shelf Audition uses.
        const double r = std::max(1.01, bandShelfHighCornerHz(band) / bandShelfLowCornerHz(band));
        return mappers::designBandpass(sampleRate, clampedFreq(band.freqHz, sampleRate), clampedQ(std::sqrt(r) / (r - 1.0)));
    }
    return mappers::designBandpass(sampleRate, clampedFreq(band.freqHz, sampleRate), clampedQ(band.q));
}

double resolveDynamicGainDb(const DynamicEqBand2& band, double envelopeDb)
{
    if (!band.dynamicsEnabled || band.rangeDb <= 0.0) return band.gainDb;
    const double over = envelopeDb - band.thresholdDb;
    if (over > 0.0)
    {
        // Loud: downward compression, pulling gain *down* from its resting
        // value -- the original behavior.
        const double slope = 1.0 - 1.0 / std::max(1.0, band.ratio); // 0 at ratio=1 (no effect), ->1 as ratio->inf (hard limit to gainDb-rangeDb)
        const double reduction = std::max(0.0, std::min(band.rangeDb, over * slope));
        return band.gainDb - reduction;
    }
    if (over < 0.0)
    {
        // Quiet: upward compression, boosting gain *up* from its resting
        // value -- the mirrored case, off by default (ratioUp == 1.0 makes
        // slopeUp exactly 0, so this returns band.gainDb unchanged, same as
        // before ratioUp existed at all).
        const double slopeUp = 1.0 - 1.0 / std::max(1.0, band.ratioUp);
        const double boost = std::max(0.0, std::min(band.rangeDb, -over * slopeUp));
        return band.gainDb + boost;
    }
    return band.gainDb;
}

std::complex<double> evaluateBandResponse(const DynamicEqBand2& band, double sampleRate, double freqHz, double resolvedGainDb, double extraFlatGainDb)
{
    const double omega = 2.0 * M_PI * freqHz / sampleRate;
    const std::complex<double> Aap = eq::evaluateBiquadResponse(designShaperAllpass(band, sampleRate), omega);
    const double G = std::pow(10.0, resolvedGainDb / 20.0);
    if (band.type == EqBandType::BandShelf)
    {
        // See designShaperAllpass2()'s comment -- the exact closed form of
        // the engine's two-stage per-sample combination.
        const std::complex<double> A2 = eq::evaluateBiquadResponse(designShaperAllpass2(band, sampleRate), omega);
        const std::complex<double> rise = 0.5 * (1.0 + G) + 0.5 * (1.0 - G) * Aap;
        const std::complex<double> fall = 0.5 * (1.0 + G) - 0.5 * (1.0 - G) * A2;
        return std::pow(10.0, extraFlatGainDb / 20.0) * rise * fall / G;
    }
    const double sign = shaperAllpassSign(band.type);
    return std::pow(10.0, extraFlatGainDb / 20.0) * (0.5 * (1.0 + G) + sign * 0.5 * (1.0 - G) * Aap);
}

std::complex<double> evaluateCombinedResponse(const DynamicEqModel2& model, double sampleRate, double freqHz,
                                               const double* currentGainDb)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const DynamicEqBand2& b = model.bands[size_t(i)];
        // A crossfeed band is excluded here -- same reasoning as
        // eq::evaluateCombinedResponse()'s own identical exclusion: it
        // doesn't reshape a channel's own signal the way a single
        // multiplicative H(z) term can represent, it INJECTS a filtered
        // copy of the OTHER channel's own signal on top of it.
        if (b.crossFeed != EqCrossFeed::None) continue;
        const double g = currentGainDb != nullptr ? currentGainDb[size_t(i)] : b.gainDb;
        h *= evaluateBandResponse(b, sampleRate, freqHz, g);
    }
    return h;
}

std::complex<double> evaluateChannelResponse(const DynamicEqModel2& model, double sampleRate, double freqHz,
                                              EqChannel listenerChannel, const double* currentGainDb)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const DynamicEqBand2& b = model.bands[size_t(i)];
        if (b.crossFeed != EqCrossFeed::None) continue; // see evaluateCombinedResponse()'s own comment
        if (listenerChannel == EqChannel::Left && b.channel == EqChannel::Right) continue;
        if (listenerChannel == EqChannel::Right && b.channel == EqChannel::Left) continue;
        const double g = currentGainDb != nullptr ? currentGainDb[size_t(i)] : b.gainDb;
        h *= evaluateBandResponse(b, sampleRate, freqHz, g);
    }
    return h;
}

StateSpaceMatrices assembleStateSpace(const DynamicEqModel2& model, double sampleRate)
{
    const std::vector<int> active = activeBandIndices(model);

    StateSpaceMatrices m;
    if (active.empty())
    {
        m.setIdentityPassthrough();
    }
    else
    {
        // Two independently-built stage lists, one per channel -- same
        // reasoning as EqModel::assembleStateSpace()'s identical split
        // (see EqChannel's own comment): a Both band's stage goes into
        // both, a Left/Right band's goes into only its own. Crossfeed
        // bands are collected into two SEPARATE lists instead, then
        // grafted on below -- exactly eq::assembleStateSpace()'s own
        // mechanism (this snapshot is an ordinary, static parametric EQ,
        // so it can reuse that exact graft unchanged; only the *live*
        // engine needed a genuinely different, per-sample design -- see
        // this file's own header comment).
        std::vector<BiquadCoeffs> leftStages, rightStages, crossToRightStages, crossToLeftStages;
        leftStages.reserve(active.size());
        rightStages.reserve(active.size());
        for (int i : active)
        {
            const DynamicEqBand2& b = model.bands[size_t(i)];
            // This snapshot is an ordinary STATIC parametric EQ (no
            // dynamics -- see this function's own declaration comment),
            // so a Low/High Shelf band just uses the sibling EQ's own
            // ordinary RBJ shelf design directly (mappers::designLowShelf()/
            // designHighShelf(), the same ones eq::designBand() itself
            // uses) rather than the live engine's own allpass-based
            // construction -- there's no dynamic gain to sweep here, so
            // none of the reasons that construction exists apply to this
            // export path at all.
            const double freq = clampedFreq(b.freqHz, sampleRate);
            const double q = clampedQ(b.q);
            std::vector<BiquadCoeffs> stages;
            if (b.type == EqBandType::BandShelf)
            {
                // The Parametric EQ's own Band Shelf design (two RBJ shelves
                // + flat -G), for the same reason as the shelves above.
                EqBand eb;
                eb.type = EqBandType::BandShelf;
                eb.freqHz = b.freqHz;
                eb.bandwidthOct = b.bandwidthOct;
                eb.gainDb = b.gainDb;
                eb.q = 0.7071; // this plugin's own Band Shelf has no Q (first-order slopes); the RBJ equivalent of a plain, non-resonant shelf
                stages = eq::designBandStages(eb, sampleRate);
            }
            else
            {
                stages.push_back(b.type == EqBandType::LowShelf ? mappers::designLowShelf(sampleRate, freq, q, b.gainDb)
                                 : b.type == EqBandType::HighShelf ? mappers::designHighShelf(sampleRate, freq, q, b.gainDb)
                                 : mappers::designPeaking(sampleRate, freq, q, b.gainDb));
            }
            for (const BiquadCoeffs& stage : stages)
            {
                if (b.crossFeed == EqCrossFeed::LeftToRight) crossToRightStages.push_back(stage);
                else if (b.crossFeed == EqCrossFeed::RightToLeft) crossToLeftStages.push_back(stage);
                else
                {
                    if (b.channel != EqChannel::Right) leftStages.push_back(stage);
                    if (b.channel != EqChannel::Left) rightStages.push_back(stage);
                }
            }
        }
        m = mappers::makeStereoBiquadChain(leftStages, rightStages);

        if (!crossToRightStages.empty() || !crossToLeftStages.empty())
        {
            // See eq::assembleStateSpace()'s own graft comment for the
            // full reasoning behind every line below -- this is that same
            // mechanism, verbatim, just fed from this model's own stage
            // lists instead.
            const StateSpaceMatrices cross = mappers::makeStereoBiquadChain(crossToRightStages, crossToLeftStages);
            const int base = (leftStages.empty() && rightStages.empty()) ? 0 : clampOrder(m.order);
            const int crossOrder = std::max(0, std::min(clampOrder(cross.order), kMaxOrder - base));

            for (int i = 0; i < crossOrder; ++i)
            {
                for (int j = 0; j < crossOrder; ++j) m.A[size_t(base + i)][size_t(base + j)] = cross.A[size_t(i)][size_t(j)];
                m.B[size_t(base + i)][0] = cross.B[size_t(i)][0];
                m.B[size_t(base + i)][1] = cross.B[size_t(i)][1];
            }
            const double gL2R = std::pow(10.0, model.crossFeedGainLtoRDb / 20.0);
            const double gR2L = std::pow(10.0, model.crossFeedGainRtoLDb / 20.0);
            for (int j = 0; j < crossOrder; ++j)
            {
                m.C[1][size_t(base + j)] += gL2R * cross.C[0][size_t(j)];
                m.C[0][size_t(base + j)] += gR2L * cross.C[1][size_t(j)];
            }
            if (!crossToRightStages.empty()) m.D[1][0] += gL2R * cross.D[0][0];
            if (!crossToLeftStages.empty())  m.D[0][1] += gR2L * cross.D[1][1];

            m.order = clampOrder(base + crossOrder);
        }
    }

    // Output trim: scales C and D only (never A/B) -- same convention as
    // EqModel::assembleStateSpace()'s own.
    if (model.outputTrimDb != 0.0)
    {
        const double g = std::pow(10.0, model.outputTrimDb / 20.0);
        const int n = clampOrder(m.order);
        for (int i = 0; i < kNumOutputs; ++i)
        {
            for (int j = 0; j < n; ++j) m.C[size_t(i)][size_t(j)] *= g;
            for (int j = 0; j < kNumInputs; ++j) m.D[size_t(i)][size_t(j)] *= g;
        }
    }

    std::snprintf(m.name, sizeof(m.name), "%s", model.name);
    return m;
}

// ---- JSON ---------------------------------------------------------------

namespace {

std::string numToStr(double v)
{
    // std::to_chars (unlike snprintf's "%.17g") is guaranteed to always
    // emit '.' as the decimal point regardless of the process's current
    // LC_NUMERIC -- a host that switches the numeric locale away from "C"
    // (e.g. GTK/Qt's setlocale(LC_ALL, "") at init) would otherwise make
    // snprintf write a comma, producing JSON that isn't valid JSON at all
    // and that this file's own reader (Json.hpp, which only recognizes
    // '.' as part of a number) would then fail to parse on the next
    // round-trip (this was a real, separate copy of the already-fixed
    // Serialization.cpp::numToStr() bug, missed because this file has
    // always defined its own copy rather than sharing that one). Also happens to produce the
    // shortest representation that still round-trips exactly, so no
    // explicit precision is needed.
    if (!std::isfinite(v)) v = 0.0;
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, res.ptr);
}

// Same string keys as EqModel.cpp's own (file-local) channelKey()/
// channelFromKey() -- not shared code, just the same small convention
// re-applied, consistent with how every other per-file JSON key helper in
// this project (bandTypeKey() and friends) is its own file's local copy
// rather than a cross-file utility.
// Same string keys as EqModel.cpp's own (file-local) bandTypeKey()/
// bandTypeFromKey() -- same "each file's own copy of this small
// convention" reasoning as channelKey() just below. Only "bell"/
// "lowShelf"/"highShelf" are ever written by this plugin's own UI (see
// DynamicEqBand2::type's own comment), but every EqBandType key is still
// accepted on read so a hand-edited or foreign file isn't silently
// rejected over a value this plugin simply never happens to produce
// itself.
const char* bandTypeKey(EqBandType t)
{
    switch (t)
    {
    case EqBandType::Bell:      return "bell";
    case EqBandType::LowShelf:  return "lowShelf";
    case EqBandType::HighShelf: return "highShelf";
    case EqBandType::LowPass:   return "lowPass";
    case EqBandType::HighPass:  return "highPass";
    case EqBandType::Notch:     return "notch";
    case EqBandType::BandPass:  return "bandPass";
    case EqBandType::AllPass:   return "allPass";
    case EqBandType::BandShelf: return "bandShelf";
    }
    return "bell";
}

bool bandTypeFromKey(const std::string& s, EqBandType& out)
{
    if (s == "bell")      { out = EqBandType::Bell;      return true; }
    if (s == "lowShelf")  { out = EqBandType::LowShelf;  return true; }
    if (s == "highShelf") { out = EqBandType::HighShelf; return true; }
    if (s == "lowPass")   { out = EqBandType::LowPass;   return true; }
    if (s == "highPass")  { out = EqBandType::HighPass;  return true; }
    if (s == "notch")     { out = EqBandType::Notch;     return true; }
    if (s == "bandPass")  { out = EqBandType::BandPass;  return true; }
    if (s == "allPass")   { out = EqBandType::AllPass;   return true; }
    if (s == "bandShelf") { out = EqBandType::BandShelf; return true; }
    return false;
}

const char* channelKey(EqChannel c)
{
    switch (c)
    {
    case EqChannel::Both:  return "both";
    case EqChannel::Left:  return "left";
    case EqChannel::Right: return "right";
    }
    return "both";
}

bool channelFromKey(const std::string& s, EqChannel& out)
{
    if (s == "both")  { out = EqChannel::Both;  return true; }
    if (s == "left")  { out = EqChannel::Left;  return true; }
    if (s == "right") { out = EqChannel::Right; return true; }
    return false;
}

const char* crossFeedKey(EqCrossFeed cf)
{
    switch (cf)
    {
    case EqCrossFeed::None:        return "off";
    case EqCrossFeed::LeftToRight: return "ltor";
    case EqCrossFeed::RightToLeft: return "rtol";
    }
    return "off";
}

bool crossFeedFromKey(const std::string& s, EqCrossFeed& out)
{
    if (s == "off")  { out = EqCrossFeed::None;        return true; }
    if (s == "ltor") { out = EqCrossFeed::LeftToRight;  return true; }
    if (s == "rtol") { out = EqCrossFeed::RightToLeft;  return true; }
    return false;
}

} // namespace

std::string toJson(const DynamicEqModel2& model)
{
    std::string s;
    s.reserve(256 + model.bands.size() * 220);
    s += "{\n";
    s += "  \"type\": \"statespace-dyneq2-preset\",\n";
    s += "  \"version\": 1,\n";
    s += "  \"name\": \"" + jsonEscape(model.name) + "\",\n";
    s += "  \"outputTrimDb\": " + numToStr(model.outputTrimDb) + ",\n";
    s += "  \"soloedBand\": " + std::to_string(model.soloedBand) + ",\n";
    s += "  \"crossFeedGainLtoRDb\": " + numToStr(model.crossFeedGainLtoRDb) + ",\n";
    s += "  \"crossFeedGainRtoLDb\": " + numToStr(model.crossFeedGainRtoLDb) + ",\n";
    s += "  \"bands\": [\n";
    for (size_t i = 0; i < model.bands.size(); ++i)
    {
        const DynamicEqBand2& b = model.bands[i];
        s += "    { \"type\": \"" + std::string(bandTypeKey(b.type)) + "\", \"freqHz\": " + numToStr(b.freqHz) + ", \"q\": " + numToStr(b.q) +
             ", \"gainDb\": " + numToStr(b.gainDb) + ", \"thresholdDb\": " + numToStr(b.thresholdDb) +
             ", \"ratio\": " + numToStr(b.ratio) + ", \"ratioUp\": " + numToStr(b.ratioUp) +
             ", \"rangeDb\": " + numToStr(b.rangeDb) +
             ", \"attackMs\": " + numToStr(b.attackMs) + ", \"releaseMs\": " + numToStr(b.releaseMs) +
             ", \"dynamicsEnabled\": " + (b.dynamicsEnabled ? "true" : "false") +
             ", \"enabled\": " + (b.enabled ? "true" : "false") +
             ", \"channel\": \"" + channelKey(b.channel) + "\"" +
             ", \"crossFeed\": \"" + crossFeedKey(b.crossFeed) + "\"" +
             ", \"harmonicGroup\": " + std::to_string(b.harmonicGroup) +
             ", \"harmonicRatio\": " + numToStr(b.harmonicRatio) +
             ", \"bandwidthOct\": " + numToStr(b.bandwidthOct) + " }";
        s += (i + 1 < model.bands.size()) ? ",\n" : "\n";
    }
    s += "  ]\n";
    s += "}\n";
    return s;
}

bool fromJson(const std::string& text, DynamicEqModel2& out)
{
    JsonValue root;
    if (!JsonParser(text).parse(root) || root.type != JsonValue::Type::Object)
        return false;

    const JsonValue* type = root.find("type");
    if (type == nullptr || type->type != JsonValue::Type::String || type->str != "statespace-dyneq2-preset")
        return false;

    DynamicEqModel2 m;
    if (const JsonValue* nameVal = root.find("name");
        nameVal != nullptr && nameVal->type == JsonValue::Type::String)
        std::snprintf(m.name, sizeof(m.name), "%s", nameVal->str.c_str());

    if (const JsonValue* trim = root.find("outputTrimDb"); trim != nullptr && trim->type == JsonValue::Type::Number)
        m.outputTrimDb = trim->number;
    if (const JsonValue* g = root.find("crossFeedGainLtoRDb"); g != nullptr && g->type == JsonValue::Type::Number)
        m.crossFeedGainLtoRDb = g->number; // leaves the default (-60) if missing -- true of every preset saved before this field existed
    if (const JsonValue* g = root.find("crossFeedGainRtoLDb"); g != nullptr && g->type == JsonValue::Type::Number)
        m.crossFeedGainRtoLDb = g->number;

    int soloedBandRaw = -1; // clamped against the final band count below, once it's known
    if (const JsonValue* so = root.find("soloedBand"); so != nullptr && so->type == JsonValue::Type::Number)
        soloedBandRaw = int(so->number);

    if (const JsonValue* bands = root.find("bands"); bands != nullptr && bands->type == JsonValue::Type::Array)
    {
        for (const JsonValue& bv : bands->arr)
        {
            if (bv.type != JsonValue::Type::Object) continue;
            DynamicEqBand2 b;
            if (const JsonValue* t0 = bv.find("type"); t0 != nullptr && t0->type == JsonValue::Type::String)
                bandTypeFromKey(t0->str, b.type); // leaves b.type at its default (Bell) if the key is missing/unrecognized -- true of every preset saved before this field existed
            if (const JsonValue* f = bv.find("freqHz"); f != nullptr && f->type == JsonValue::Type::Number) b.freqHz = f->number;
            if (const JsonValue* q = bv.find("q"); q != nullptr && q->type == JsonValue::Type::Number) b.q = q->number;
            if (const JsonValue* g = bv.find("gainDb"); g != nullptr && g->type == JsonValue::Type::Number) b.gainDb = g->number;
            if (const JsonValue* t = bv.find("thresholdDb"); t != nullptr && t->type == JsonValue::Type::Number) b.thresholdDb = t->number;
            if (const JsonValue* r = bv.find("ratio"); r != nullptr && r->type == JsonValue::Type::Number) b.ratio = r->number;
            if (const JsonValue* ru = bv.find("ratioUp"); ru != nullptr && ru->type == JsonValue::Type::Number) b.ratioUp = ru->number;
            if (const JsonValue* rg = bv.find("rangeDb"); rg != nullptr && rg->type == JsonValue::Type::Number) b.rangeDb = rg->number;
            if (const JsonValue* a = bv.find("attackMs"); a != nullptr && a->type == JsonValue::Type::Number) b.attackMs = a->number;
            if (const JsonValue* rl = bv.find("releaseMs"); rl != nullptr && rl->type == JsonValue::Type::Number) b.releaseMs = rl->number;
            if (const JsonValue* de = bv.find("dynamicsEnabled"); de != nullptr && de->type == JsonValue::Type::Number) b.dynamicsEnabled = de->number != 0.0;
            if (const JsonValue* en = bv.find("enabled"); en != nullptr && en->type == JsonValue::Type::Number) b.enabled = en->number != 0.0;
            if (const JsonValue* ch = bv.find("channel"); ch != nullptr && ch->type == JsonValue::Type::String)
                channelFromKey(ch->str, b.channel); // leaves b.channel at its default (Both) if the key is missing/unrecognized
            if (const JsonValue* cf = bv.find("crossFeed"); cf != nullptr && cf->type == JsonValue::Type::String)
                crossFeedFromKey(cf->str, b.crossFeed); // leaves b.crossFeed at its default (None) if the key is missing/unrecognized -- true of every preset saved before this field existed
            if (const JsonValue* hg = bv.find("harmonicGroup"); hg != nullptr && hg->type == JsonValue::Type::Number)
                b.harmonicGroup = std::max(0, int(hg->number));
            if (const JsonValue* hr = bv.find("harmonicRatio"); hr != nullptr && hr->type == JsonValue::Type::Number)
                b.harmonicRatio = std::max(0.001, std::min(1000.0, hr->number));
            if (const JsonValue* bw = bv.find("bandwidthOct"); bw != nullptr && bw->type == JsonValue::Type::Number)
                b.bandwidthOct = std::max(kMinBandwidthOct, std::min(kMaxBandwidthOct, bw->number)); // default if missing -- every preset saved before Band Shelf
            m.bands.push_back(b);
        }
    }

    // Clamp against the actual band count rather than trusting a hand-
    // edited/foreign file's own value -- see MbCompressorModel::fromJson()'s
    // identical guard.
    m.soloedBand = (soloedBandRaw >= 0 && soloedBandRaw < int(m.bands.size())) ? soloedBandRaw : -1;

    out = m;
    return true;
}

} // namespace dyneq2
} // namespace ssfilter
