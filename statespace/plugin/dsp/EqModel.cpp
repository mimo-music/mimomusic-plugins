// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "EqModel.hpp"
#include "Json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <memory>

namespace ssfilter {
namespace eq {

BiquadCoeffs designBand(const EqBand& band, double sampleRate)
{
    const double q = std::max(0.05, std::min(100.0, band.q));
    const double freqHz = std::max(1.0, std::min(sampleRate * 0.499, band.freqHz));
    const double gainDb = band.gainDb;

    switch (band.type)
    {
    case EqBandType::Bell:      return mappers::designPeaking(sampleRate, freqHz, q, gainDb);
    case EqBandType::LowShelf:  return mappers::designLowShelf(sampleRate, freqHz, q, gainDb);
    case EqBandType::HighShelf: return mappers::designHighShelf(sampleRate, freqHz, q, gainDb);
    case EqBandType::LowPass:   return mappers::designLowpass(sampleRate, freqHz, q);
    case EqBandType::HighPass:  return mappers::designHighpass(sampleRate, freqHz, q);
    case EqBandType::Notch:     return mappers::designNotch(sampleRate, freqHz, q);
    case EqBandType::BandPass:  return mappers::designBandpass(sampleRate, freqHz, q);
    case EqBandType::AllPass:   return mappers::designAllpass(sampleRate, freqHz, q);
    case EqBandType::BandShelf: break; // two biquads, not one -- only designBandStages() builds it
    }
    return BiquadCoeffs{};
}

std::vector<BiquadCoeffs> designBandStages(const EqBand& band, double sampleRate)
{
    if (band.type == EqBandType::BandShelf)
    {
        // See eqBandTypeHasBandwidth()'s comment for the shape. Same clamps
        // designBand() applies to a single shelf; the bandwidth is clamped
        // to its own range first (a negative one would swap the corners
        // and flip the boost into a cut).
        EqBand clampedBand = band;
        clampedBand.bandwidthOct = std::max(kMinBandwidthOct, std::min(kMaxBandwidthOct, band.bandwidthOct));
        const double q = std::max(0.05, std::min(100.0, band.q));
        const double lo = std::max(1.0, std::min(sampleRate * 0.499, bandShelfLowCornerHz(clampedBand)));
        const double hi = std::max(1.0, std::min(sampleRate * 0.499, bandShelfHighCornerHz(clampedBand)));
        const BiquadCoeffs rise = mappers::designHighShelf(sampleRate, lo, q, band.gainDb);
        BiquadCoeffs fall = mappers::designLowShelf(sampleRate, hi, q, band.gainDb);
        // The flat -G, folded into the second stage's numerator: an exact
        // scalar on the whole band, no extra states, and -- since only b
        // changes, never a -- no effect on the poles or stability.
        const double flat = std::pow(10.0, -band.gainDb / 20.0);
        fall.b0 *= flat;
        fall.b1 *= flat;
        fall.b2 *= flat;
        return { rise, fall };
    }
    const bool cascadable = band.type == EqBandType::LowPass || band.type == EqBandType::HighPass;
    const int stages = cascadable ? std::max(1, std::min(4, band.slopeStages)) : 1;
    const BiquadCoeffs c = designBand(band, sampleRate);
    return std::vector<BiquadCoeffs>(size_t(stages), c);
}

bool hasActiveSolo(const EqModel& model)
{
    for (const auto& b : model.bands)
        if (b.enabled && b.solo) return true;
    return false;
}

std::vector<int> activeBandIndices(const EqModel& model)
{
    const bool anySolo = hasActiveSolo(model);

    // Soloing one band pulls in every other member of its harmonic group
    // too, not just the one whose own `solo` flag happens to be set:
    // soloing one half of a
    // harmonic twin and hearing only that half, silencing the very band
    // it's meant to move together with, would defeat the point of linking
    // them in the first place.
    std::vector<int> soloedGroups;
    if (anySolo)
        for (const auto& b : model.bands)
            if (b.enabled && b.solo && b.harmonicGroup != 0)
                soloedGroups.push_back(b.harmonicGroup);
    auto inSoloedGroup = [&soloedGroups](int group) {
        if (group == 0) return false;
        for (int g : soloedGroups) if (g == group) return true;
        return false;
    };

    std::vector<int> out;
    for (int i = 0; i < int(model.bands.size()); ++i)
    {
        const EqBand& b = model.bands[size_t(i)];
        if (!b.enabled) continue;
        if (anySolo && !b.solo && !inSoloedGroup(b.harmonicGroup)) continue;
        out.push_back(i);
    }
    return out;
}

int nextHarmonicGroupId(const EqModel& model)
{
    int highest = 0;
    for (const EqBand& b : model.bands) highest = std::max(highest, b.harmonicGroup);
    return highest + 1;
}

void propagateHarmonicFreq(EqModel& model, int index, double freqHz, double sampleRate)
{
    if (index < 0 || index >= int(model.bands.size()) || !std::isfinite(freqHz)) return;
    const double clamped = std::max(1.0, std::min(sampleRate * 0.499, freqHz));
    EqBand& moved = model.bands[size_t(index)];
    moved.freqHz = clamped;

    if (moved.harmonicGroup == 0) return; // not linked to anything else -- nothing further to do

    // The group's own base is virtual -- never stored, only ever derived
    // from whichever member was just moved and its own fixed ratio to
    // that base -- see EqBand::harmonicGroup's own comment. A Band Shelf's
    // freqHz is its centre, so it moves as a whole, keeping its bandwidth.
    const double base = clamped / std::max(1.0e-6, moved.harmonicRatio);
    for (EqBand& other : model.bands)
    {
        if (&other == &moved || other.harmonicGroup != moved.harmonicGroup) continue;
        other.freqHz = std::max(1.0, std::min(sampleRate * 0.499, base * other.harmonicRatio));
    }
}

int createHarmonicTwin(EqModel& model, int sourceIndex, double factor, double sampleRate)
{
    if (sourceIndex < 0 || sourceIndex >= int(model.bands.size())) return -1;
    if (!std::isfinite(factor) || factor <= 0.0) return -1;
    constexpr int kMaxBands = kMaxOrder / 4; // same cap UIEq::addBand()/duplicateBand() enforce
    if (int(model.bands.size()) >= kMaxBands) return -1;

    EqBand& source = model.bands[size_t(sourceIndex)];
    if (source.harmonicGroup == 0)
    {
        // First link on this band -- it becomes the group's own ratio-1.0
        // reference point, keeping its current frequency exactly as the
        // (now virtual) base.
        source.harmonicGroup = nextHarmonicGroupId(model);
        source.harmonicRatio = 1.0;
    }

    EqBand twin = source; // copies type/gain/Q/slopeStages/bandwidth/enabled -- "another filter same to the selected"
    twin.freqHz = std::max(1.0, std::min(sampleRate * 0.499, source.freqHz * factor));
    twin.harmonicGroup = source.harmonicGroup;
    twin.harmonicRatio = source.harmonicRatio * factor; // chains correctly if source was already linked, see this function's own declaration comment
    twin.solo = false; // a freshly-created band was never deliberately soloed by the user

    const int newIndex = sourceIndex + 1;
    model.bands.insert(model.bands.begin() + newIndex, twin);
    return newIndex;
}

void unlinkHarmonic(EqModel& model, int index)
{
    if (index < 0 || index >= int(model.bands.size())) return;
    model.bands[size_t(index)].harmonicGroup = 0;
}

double bandShelfLowCornerHz(const EqBand& band)
{
    return band.freqHz / std::pow(2.0, 0.5 * band.bandwidthOct);
}

double bandShelfHighCornerHz(const EqBand& band)
{
    return band.freqHz * std::pow(2.0, 0.5 * band.bandwidthOct);
}

void applyBandType(EqBand& band, EqBandType type)
{
    band.type = type;
    if (!eqBandTypeHasGain(type)) band.gainDb = 0.0;
}

StateSpaceMatrices assembleStateSpace(const EqModel& model, double sampleRate)
{
    const std::vector<int> active = activeBandIndices(model);

    // Two independently-built stage lists, one per channel -- a Both
    // band contributes to both, a Left/Right band contributes to only
    // its own. When every active band is Both (the common case, and
    // the only case before EqChannel existed), leftStages/rightStages
    // end up identical and makeStereoBiquadChain() produces exactly
    // what it always did; when any band is Left/Right, the two lists
    // genuinely differ and the two channels come out independently
    // filtered. See EqChannel's own comment.
    //
    // Crossfeed bands (EqBand::crossFeed != None) are collected into
    // two SEPARATE lists instead -- they never contribute to
    // leftStages/rightStages at all, so a model with no crossfeed
    // bands builds `m` via the exact same call, with the exact same
    // result, as before this feature existed. See this function's
    // own header comment for how crossToRightStages/crossToLeftStages
    // get grafted on afterward.
    std::vector<BiquadCoeffs> leftStages, rightStages, crossToRightStages, crossToLeftStages;
    leftStages.reserve(active.size());
    rightStages.reserve(active.size());
    for (int i : active)
    {
        const EqBand& b = model.bands[size_t(i)];
        if (b.crossFeed == EqCrossFeed::LeftToRight)
        {
            const std::vector<BiquadCoeffs> bandStages = designBandStages(b, sampleRate);
            crossToRightStages.insert(crossToRightStages.end(), bandStages.begin(), bandStages.end());
        }
        else if (b.crossFeed == EqCrossFeed::RightToLeft)
        {
            const std::vector<BiquadCoeffs> bandStages = designBandStages(b, sampleRate);
            crossToLeftStages.insert(crossToLeftStages.end(), bandStages.begin(), bandStages.end());
        }
        else
        {
            const std::vector<BiquadCoeffs> bandStages = designBandStages(b, sampleRate);
            if (b.channel != EqChannel::Right) leftStages.insert(leftStages.end(), bandStages.begin(), bandStages.end());
            if (b.channel != EqChannel::Left) rightStages.insert(rightStages.end(), bandStages.begin(), bandStages.end());
        }
    }

    // `m` is initialized directly from the call (guaranteed copy elision, and
    // NRVO into the caller's own result on return) rather than default-
    // constructed first and then assigned -- `m = makeStereoBiquadChain(...)`
    // materialized a whole second ~131KB StateSpaceMatrices temporary on
    // the stack. That matters: this runs on the host's real-time process
    // thread (PluginEq::run() -> kParamMirrorCommit -> rebuildEngineModel()),
    // whose stack is small (Ardour: 512KB) -- see the crossfeed graft's
    // own comment below for the real overflow this caused.
    StateSpaceMatrices m = mappers::makeStereoBiquadChain(leftStages, rightStages);
    if (active.empty())
    {
        m.setIdentityPassthrough();
        // setIdentityPassthrough() conservatively defaults this to false
        // (it's shared with statespace_mimo's arbitrary hand-edited
        // matrices, see the field's own comment) -- but an all-zero-A
        // passthrough trivially satisfies sparseCascade2's contract too
        // (every A[i][j] is 0, which is certainly `<= blockEnd(i)`-only-
        // nonzero), so it's safe to re-enable it specifically here.
        m.sparseCascade2 = true;
    }
    else
    {
        // Soloing a Left- or Right-only band should isolate it completely:
        // only that channel's audio passes. Without this, soloing (say) a
        // Left-only band correctly leaves only that one band's own
        // filtering active, but the Right channel -- untouched by any
        // currently-soloed band -- would just be a raw, unfiltered copy
        // of the input rather than silence, since "no stages" and
        // "silent" aren't the same thing (makeStereoBiquadChain() with an
        // empty stage list is an identity passthrough, not a mute). That
        // raw leak defeats the point of soloing a channel-restricted band
        // in the first place. Only kicks in while a solo is actually
        // active (an ordinary Left-only band with nothing soloed at all
        // legitimately leaves the other channel as passthrough -- that's
        // the routing feature working as intended, not a leak to plug).
        // Implemented as zeroing that channel's own C/D rows -- an exact,
        // unconditional silence regardless of the (otherwise entirely
        // unrelated) internal state those rows would have read from,
        // simpler and safer than trying to build a "silent" SISO chain
        // through the cascade machinery itself.
        //
        // A channel whose only active content is a crossfeed band FED
        // BY it (e.g. soloing a lone LeftToRight band, so leftStages is
        // empty but crossToRightStages is not) must NOT be silenced here
        // -- there IS something reaching the listener, it just isn't in
        // leftStages/rightStages. The crossfeed graft below runs AFTER
        // this block specifically so its own C/D writes land on a row
        // this solo logic correctly left alone.
        if (hasActiveSolo(model))
        {
            const int n = clampOrder(m.order);
            if (leftStages.empty() && crossToLeftStages.empty())
            {
                for (int j = 0; j < n; ++j) m.C[0][size_t(j)] = 0.0;
                for (int j = 0; j < kNumInputs; ++j) m.D[0][size_t(j)] = 0.0;
            }
            if (rightStages.empty() && crossToRightStages.empty())
            {
                for (int j = 0; j < n; ++j) m.C[1][size_t(j)] = 0.0;
                for (int j = 0; j < kNumInputs; ++j) m.D[1][size_t(j)] = 0.0;
            }
        }

        // Graft the crossfeed content on, if any -- see this function's
        // own header comment for the full reasoning. Skipped entirely
        // (m unchanged from the ordinary L/R assembly above) when neither
        // list has anything in it, which is every model that predates
        // this field and every model that simply doesn't use it.
        if (!crossToRightStages.empty() || !crossToLeftStages.empty())
        {
            // Reuses makeStereoBiquadChain() one more time, purely as a
            // convenient way to cascade+pack two independent SISO chains
            // into a small ready-made block-diagonal system -- `cross`'s
            // own "channel 0" here is crossToRightStages (fed by ITS OWN
            // input 0, read from ITS OWN output 0), "channel 1" is
            // crossToLeftStages, entirely unrelated to the real L/R
            // meaning those slots have in `m` itself. The grafting below
            // is what remaps them onto the real global inputs/outputs.
            //
            // Heap-allocated, NOT a local: a second ~131KB StateSpaceMatrices
            // on the stack here (on top of `m`, the caller's own copy, and
            // makeStereoBiquadChain()'s inlined SisoSystem temporaries) is
            // exactly what overflowed Ardour's 512KB real-time process-
            // thread stack the moment a band was placed -- a SIGSEGV inside
            // this function's closing snprintf, measured at 528,640 bytes
            // deep from the core dump. `new T(prvalue)`
            // constructs straight into the heap block (guaranteed copy
            // elision), so no stack temporary of it ever exists. The
            // allocation itself is no new kind of real-time cost: this
            // rebuild already allocates (the stage-list vectors above).
            const std::unique_ptr<const StateSpaceMatrices> crossPtr(
                new StateSpaceMatrices(mappers::makeStereoBiquadChain(crossToRightStages, crossToLeftStages)));
            const StateSpaceMatrices& cross = *crossPtr;
            // clampOrder()'s own [1, kMaxOrder] floor pads a genuinely
            // order-0 `m` (leftStages AND rightStages BOTH empty --
            // happens whenever EVERY active band is crossfeed-routed, so
            // nothing ordinary feeds either channel) up to a fake order
            // of 1 -- an ODD number, silently breaking the "base is
            // always even" invariant this graft's own comment below
            // depends on. An odd base misaligns the grafted block by one
            // state; the engine's own sparseCascade2 fast path then reads
            // it with the WRONG 2-wide block boundary and silently drops
            // a real, load-bearing A[i][j] pole-coupling term from the
            // recursion. Root cause of a real bug (a "numerical fault
            // auto-recovered" warning when placing a band almost anywhere,
            // less often above 10kHz) -- reproduced exactly by placing a single crossfeed-only band (the
            // minimal case where leftStages/rightStages are both empty)
            // in tests/dsp_regression.cpp's own [36], worse at low
            // frequencies because a corrupted 2nd-order recursion is more
            // likely to actually diverge for a tighter, more resonant
            // pole. `m.order` is used ONLY as this graft's own offset
            // here -- the combined `m.order = base + crossOrder` below is
            // never left at the problematic 0 either way (crossOrder is
            // always >= 2 inside this `if`) -- so the true, unpadded 0 is
            // used directly instead of clampOrder()'s padded value.
            const int base = (leftStages.empty() && rightStages.empty()) ? 0 : clampOrder(m.order);
            // Both `base` and cross.order are individually already
            // <= kMaxOrder (clampOrder()'s own contract), but their SUM
            // is a new bound this function's own writes below need to
            // respect that neither individual clamp enforces on its
            // own -- without this, a pathological band count/slope combo
            // could write base+i past kMaxOrder into m.A/B/C's own fixed
            // kMaxOrder-sized storage (undefined behavior, not just a
            // truncated/imprecise result). Silently drops whatever
            // crossfeed content doesn't fit, same "cap rather than
            // overflow" spirit as UIEq::addBand()'s own kMaxOrder/4 bands
            // limit.
            const int crossOrder = std::max(0, std::min(clampOrder(cross.order), kMaxOrder - base));

            for (int i = 0; i < crossOrder; ++i)
            {
                for (int j = 0; j < crossOrder; ++j) m.A[size_t(base + i)][size_t(base + j)] = cross.A[size_t(i)][size_t(j)];
                // B is NOT swapped: cross's own "channel 0" (crossToRightStages)
                // is already fed from ITS OWN input column 0 -- which is
                // exactly u_L, the real source this content should react
                // to -- so copying B as-is already wires the right thing.
                // Same for "channel 1"/crossToLeftStages and u_R/column 1.
                m.B[size_t(base + i)][0] = cross.B[size_t(i)][0];
                m.B[size_t(base + i)][1] = cross.B[size_t(i)][1];
            }
            // C IS swapped: cross's own "channel 0" output (row 0) is
            // crossToRightStages' filtered content, which must reach the
            // REAL right output (row 1) -- the one deliberate asymmetry
            // that actually turns "filtered L" into "L crossfed into R".
            //
            // gL2R/gR2L: the model's own crossFeedGainLtoRDb/
            // crossFeedGainRtoLDb (EqModel's own comment) -- ONE flat
            // scalar per direction for the WHOLE transit, applied here at
            // the graft's own output combination, exactly like
            // `outputTrimDb` below scales the whole model's own C/D.
            // Scaling C/D only (never A/B) can never affect stability
            // (same identity the output-trim step already relies on) and,
            // because it's applied once here rather than per-band inside
            // the cascade, multiple crossfeed bands in the same direction
            // never interact with each other's own gain the way an
            // earlier per-band attempt did (see EqModel::
            // crossFeedGainLtoRDb's own comment for that history).
            const double gL2R = std::pow(10.0, model.crossFeedGainLtoRDb / 20.0);
            const double gR2L = std::pow(10.0, model.crossFeedGainRtoLDb / 20.0);
            for (int j = 0; j < crossOrder; ++j)
            {
                m.C[1][size_t(base + j)] += gL2R * cross.C[0][size_t(j)];
                m.C[0][size_t(base + j)] += gR2L * cross.C[1][size_t(j)];
            }
            // D, same swap, columns unchanged (matching B's own unswapped
            // input wiring above): cross.D[0][1] and cross.D[1][0] are
            // always exactly 0 (makeStereoBiquadChain() never couples its
            // own two channels), so only the diagonal terms carry over --
            // but ONLY when that side actually has stages. An empty stage
            // list cascades to cascadeChain()'s own documented identity
            // (order 0, D=1.0) -- correct for its OWN original purpose
            // (an unfiltered pass-through segment), but wrong here: a
            // crossfeed direction with nothing in it must contribute
            // exactly ZERO to the other channel, not a phantom unity
            // feedthrough. Guarding on the actual stage list being
            // non-empty (not on crossOrder, which only reflects whichever
            // side genuinely has states) is what keeps that unity D=1.0
            // from leaking across.
            if (!crossToRightStages.empty()) m.D[1][0] += gL2R * cross.D[0][0];
            if (!crossToLeftStages.empty())  m.D[0][1] += gR2L * cross.D[1][1];

            m.order = clampOrder(base + crossOrder);
            // sparseCascade2 stays true (not re-derived, just left as
            // makeStereoBiquadChain() already set it on `m`): the graft
            // only ever adds a brand-new diagonal A block, aligned to a
            // 2-wide boundary (`base` is always even -- both L.order and
            // R.order are always even, being sums of 2-state biquads), so
            // every row's own "never above its own 2-wide block" property
            // -- the field's exact contract -- still holds for every row,
            // old and new alike.
        }
    }

    // Output trim: scales C and D only (never A/B), same convention as
    // TransferFunction::normalizeToUnityPassbandGain -- a pure gain change
    // can never affect stability.
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

bool hasPerChannelBands(const EqModel& model)
{
    for (int i : activeBandIndices(model))
    {
        const EqBand& b = model.bands[size_t(i)];
        if (b.channel != EqChannel::Both || b.crossFeed != EqCrossFeed::None) return true;
    }
    return false;
}

mono::MonoStateSpaceMatrices assembleMonoStateSpace(const EqModel& model, double sampleRate)
{
    const std::vector<int> active = activeBandIndices(model);

    mono::MonoStateSpaceMatrices m;
    if (active.empty())
    {
        m.setIdentityPassthrough();
    }
    else
    {
        std::vector<BiquadCoeffs> stages;
        stages.reserve(active.size());
        for (int i : active)
        {
            const std::vector<BiquadCoeffs> bandStages = designBandStages(model.bands[size_t(i)], sampleRate);
            stages.insert(stages.end(), bandStages.begin(), bandStages.end());
        }
        m = mappers::makeMonoBiquadChain(stages);
    }

    // Output trim: scales C and D only (never A/B), same convention as
    // assembleStateSpace()'s own copy of this.
    if (model.outputTrimDb != 0.0)
    {
        const double g = std::pow(10.0, model.outputTrimDb / 20.0);
        const int n = clampOrder(m.order);
        for (int j = 0; j < n; ++j) m.C[size_t(j)] *= g;
        m.D *= g;
    }

    std::snprintf(m.name, sizeof(m.name), "%s", model.name);
    return m;
}

std::complex<double> evaluateBiquadResponse(const BiquadCoeffs& c, double omega)
{
    const std::complex<double> z1 = std::polar(1.0, -omega);
    const std::complex<double> z2 = std::polar(1.0, -2.0 * omega);
    const std::complex<double> num = c.b0 + c.b1 * z1 + c.b2 * z2;
    const std::complex<double> den = 1.0 + c.a1 * z1 + c.a2 * z2;
    return num / den;
}

std::complex<double> evaluateBandResponse(const EqBand& band, double sampleRate, double freqHz, double extraFlatGainDb)
{
    const double omega = 2.0 * M_PI * freqHz / sampleRate;
    // `extraFlatGainDb`: the model-level crossFeedGainLtoRDb/RtoLDb
    // transit baseline (EqModel's own comment) -- passed in by the
    // caller (EqGraphWidget), not read from `band` itself, since it now
    // lives on the model, not the band. Applied as a flat scalar
    // multiplier here, same as assembleStateSpace()'s own graft-step
    // scaling of cross's C/D rows, so a crossfeed band's own individual
    // preview curve shows the same "flat baseline, bell/shelf bump on
    // top" shape the DSP actually produces, not a misleading "unity
    // baseline, dip/bump at one frequency only" one.
    std::complex<double> h(std::pow(10.0, extraFlatGainDb / 20.0), 0.0);
    for (const BiquadCoeffs& c : designBandStages(band, sampleRate))
        h *= evaluateBiquadResponse(c, omega);
    return h;
}

std::complex<double> evaluateCombinedResponse(const EqModel& model, double sampleRate, double freqHz)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const EqBand& b = model.bands[size_t(i)];
        if (b.crossFeed != EqCrossFeed::None) continue; // see this function's own header comment
        h *= evaluateBandResponse(b, sampleRate, freqHz);
    }
    return h;
}

std::complex<double> evaluateChannelResponse(const EqModel& model, double sampleRate, double freqHz, EqChannel listenerChannel)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const EqBand& b = model.bands[size_t(i)];
        if (b.crossFeed != EqCrossFeed::None) continue; // see this function's own header comment
        if (listenerChannel == EqChannel::Left && b.channel == EqChannel::Right) continue;
        if (listenerChannel == EqChannel::Right && b.channel == EqChannel::Left) continue;
        h *= evaluateBandResponse(b, sampleRate, freqHz);
    }
    return h;
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
    case EqCrossFeed::None:        return "none";
    case EqCrossFeed::LeftToRight: return "leftToRight";
    case EqCrossFeed::RightToLeft: return "rightToLeft";
    }
    return "none";
}

bool crossFeedFromKey(const std::string& s, EqCrossFeed& out)
{
    if (s == "none")        { out = EqCrossFeed::None;        return true; }
    if (s == "leftToRight") { out = EqCrossFeed::LeftToRight; return true; }
    if (s == "rightToLeft") { out = EqCrossFeed::RightToLeft; return true; }
    return false;
}

} // namespace

std::string toJson(const EqModel& model)
{
    std::string s;
    s.reserve(256 + model.bands.size() * 160);
    s += "{\n";
    s += "  \"type\": \"statespace-eq-preset\",\n";
    s += "  \"version\": 1,\n";
    s += "  \"name\": \"" + jsonEscape(model.name) + "\",\n";
    s += "  \"outputTrimDb\": " + numToStr(model.outputTrimDb) + ",\n";
    s += "  \"crossFeedGainLtoRDb\": " + numToStr(model.crossFeedGainLtoRDb) + ",\n";
    s += "  \"crossFeedGainRtoLDb\": " + numToStr(model.crossFeedGainRtoLDb) + ",\n";
    s += "  \"bands\": [\n";
    for (size_t i = 0; i < model.bands.size(); ++i)
    {
        const EqBand& b = model.bands[i];
        s += "    { \"type\": \"" + std::string(bandTypeKey(b.type)) + "\", \"freqHz\": " + numToStr(b.freqHz) +
             ", \"gainDb\": " + numToStr(b.gainDb) + ", \"q\": " + numToStr(b.q) +
             ", \"slopeStages\": " + numToStr(double(b.slopeStages)) +
             ", \"enabled\": " + (b.enabled ? "true" : "false") +
             ", \"solo\": " + (b.solo ? "true" : "false") +
             ", \"harmonicGroup\": " + std::to_string(b.harmonicGroup) +
             ", \"harmonicRatio\": " + numToStr(b.harmonicRatio) +
             ", \"channel\": \"" + channelKey(b.channel) + "\"" +
             ", \"crossFeed\": \"" + crossFeedKey(b.crossFeed) + "\"" +
             ", \"bandwidthOct\": " + numToStr(b.bandwidthOct) + " }";
        s += (i + 1 < model.bands.size()) ? ",\n" : "\n";
    }
    s += "  ]\n";
    s += "}\n";
    return s;
}

bool fromJson(const std::string& text, EqModel& out)
{
    JsonValue root;
    if (!JsonParser(text).parse(root) || root.type != JsonValue::Type::Object)
        return false;

    const JsonValue* type = root.find("type");
    if (type == nullptr || type->type != JsonValue::Type::String || type->str != "statespace-eq-preset")
        return false;

    EqModel m;
    if (const JsonValue* nameVal = root.find("name");
        nameVal != nullptr && nameVal->type == JsonValue::Type::String)
        std::snprintf(m.name, sizeof(m.name), "%s", nameVal->str.c_str());

    if (const JsonValue* trim = root.find("outputTrimDb"); trim != nullptr && trim->type == JsonValue::Type::Number)
        m.outputTrimDb = trim->number;
    if (const JsonValue* g = root.find("crossFeedGainLtoRDb"); g != nullptr && g->type == JsonValue::Type::Number)
        m.crossFeedGainLtoRDb = g->number; // leaves the default (-60) if missing -- true of every preset saved before this field existed (including the brief per-band-only iteration of this feature)
    if (const JsonValue* g = root.find("crossFeedGainRtoLDb"); g != nullptr && g->type == JsonValue::Type::Number)
        m.crossFeedGainRtoLDb = g->number;

    if (const JsonValue* bands = root.find("bands"); bands != nullptr && bands->type == JsonValue::Type::Array)
    {
        for (const JsonValue& bv : bands->arr)
        {
            if (bv.type != JsonValue::Type::Object) continue;
            EqBand b;
            if (const JsonValue* t = bv.find("type"); t != nullptr && t->type == JsonValue::Type::String)
                bandTypeFromKey(t->str, b.type);
            if (const JsonValue* f = bv.find("freqHz"); f != nullptr && f->type == JsonValue::Type::Number)
                b.freqHz = f->number;
            if (const JsonValue* g = bv.find("gainDb"); g != nullptr && g->type == JsonValue::Type::Number)
                b.gainDb = g->number;
            if (const JsonValue* q = bv.find("q"); q != nullptr && q->type == JsonValue::Type::Number)
                b.q = q->number;
            if (const JsonValue* sl = bv.find("slopeStages"); sl != nullptr && sl->type == JsonValue::Type::Number)
                b.slopeStages = std::max(1, std::min(4, int(sl->number)));
            if (const JsonValue* en = bv.find("enabled"); en != nullptr && en->type == JsonValue::Type::Number)
                b.enabled = en->number != 0.0;
            if (const JsonValue* so = bv.find("solo"); so != nullptr && so->type == JsonValue::Type::Number)
                b.solo = so->number != 0.0;
            if (const JsonValue* hg = bv.find("harmonicGroup"); hg != nullptr && hg->type == JsonValue::Type::Number)
                b.harmonicGroup = std::max(0, int(hg->number));
            if (const JsonValue* hr = bv.find("harmonicRatio"); hr != nullptr && hr->type == JsonValue::Type::Number)
                b.harmonicRatio = std::max(0.001, std::min(1000.0, hr->number));
            if (const JsonValue* ch = bv.find("channel"); ch != nullptr && ch->type == JsonValue::Type::String)
                channelFromKey(ch->str, b.channel); // leaves b.channel at its default (Both) if the key is missing/unrecognized
            if (const JsonValue* cf = bv.find("crossFeed"); cf != nullptr && cf->type == JsonValue::Type::String)
                crossFeedFromKey(cf->str, b.crossFeed); // leaves b.crossFeed at its default (None) if the key is missing/unrecognized -- true of every preset saved before this field existed
            if (const JsonValue* bw = bv.find("bandwidthOct"); bw != nullptr && bw->type == JsonValue::Type::Number)
                b.bandwidthOct = std::max(kMinBandwidthOct, std::min(kMaxBandwidthOct, bw->number)); // leaves the default if missing -- every preset saved before Band Shelf existed
            else if (const JsonValue* f2 = bv.find("freq2Hz"); f2 != nullptr && f2->type == JsonValue::Type::Number &&
                     b.type == EqBandType::BandShelf && b.freqHz > 0.0 && f2->number > 0.0)
            {
                // The Band Shelf's first, briefly-shipped format stored its
                // two corners (freqHz, freq2Hz) instead of centre +
                // bandwidth -- converted here so such a preset loads
                // unchanged.
                const double lo = std::min(b.freqHz, f2->number), hi = std::max(b.freqHz, f2->number);
                b.freqHz = std::sqrt(lo * hi);
                b.bandwidthOct = std::max(kMinBandwidthOct, std::min(kMaxBandwidthOct, std::log2(hi / lo)));
            }
            m.bands.push_back(b);
        }
    }

    out = m;
    return true;
}

} // namespace eq
} // namespace ssfilter
