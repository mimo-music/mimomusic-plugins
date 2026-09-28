// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "EqModel.hpp"
#include "Json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

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
    }
    return BiquadCoeffs{};
}

std::vector<BiquadCoeffs> designBandStages(const EqBand& band, double sampleRate)
{
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
    // that base -- see EqBand::harmonicGroup's own comment.
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

    EqBand twin = source; // copies type/gain/Q/slopeStages/enabled
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

StateSpaceMatrices assembleStateSpace(const EqModel& model, double sampleRate)
{
    const std::vector<int> active = activeBandIndices(model);

    StateSpaceMatrices m;
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
        // Two independently-built stage lists, one per channel -- a Both
        // band contributes to both, a Left/Right band contributes to only
        // its own. When every active band is Both (the common case, and
        // the only case before EqChannel existed), leftStages/rightStages
        // end up identical and makeStereoBiquadChain() produces exactly
        // what it always did; when any band is Left/Right, the two lists
        // genuinely differ and the two channels come out independently
        // filtered. See EqChannel's own comment.
        std::vector<BiquadCoeffs> leftStages, rightStages;
        leftStages.reserve(active.size());
        rightStages.reserve(active.size());
        for (int i : active)
        {
            const EqBand& b = model.bands[size_t(i)];
            const std::vector<BiquadCoeffs> bandStages = designBandStages(b, sampleRate);
            if (b.channel != EqChannel::Right) leftStages.insert(leftStages.end(), bandStages.begin(), bandStages.end());
            if (b.channel != EqChannel::Left) rightStages.insert(rightStages.end(), bandStages.begin(), bandStages.end());
        }
        m = mappers::makeStereoBiquadChain(leftStages, rightStages);

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
        if (hasActiveSolo(model))
        {
            const int n = clampOrder(m.order);
            if (leftStages.empty())
            {
                for (int j = 0; j < n; ++j) m.C[0][size_t(j)] = 0.0;
                for (int j = 0; j < kNumInputs; ++j) m.D[0][size_t(j)] = 0.0;
            }
            if (rightStages.empty())
            {
                for (int j = 0; j < n; ++j) m.C[1][size_t(j)] = 0.0;
                for (int j = 0; j < kNumInputs; ++j) m.D[1][size_t(j)] = 0.0;
            }
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
        if (model.bands[size_t(i)].channel != EqChannel::Both) return true;
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

std::complex<double> evaluateBandResponse(const EqBand& band, double sampleRate, double freqHz)
{
    const double omega = 2.0 * M_PI * freqHz / sampleRate;
    std::complex<double> h(1.0, 0.0);
    for (const BiquadCoeffs& c : designBandStages(band, sampleRate))
        h *= evaluateBiquadResponse(c, omega);
    return h;
}

std::complex<double> evaluateCombinedResponse(const EqModel& model, double sampleRate, double freqHz)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
        h *= evaluateBandResponse(model.bands[size_t(i)], sampleRate, freqHz);
    return h;
}

std::complex<double> evaluateChannelResponse(const EqModel& model, double sampleRate, double freqHz, EqChannel listenerChannel)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const EqBand& b = model.bands[size_t(i)];
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
             ", \"channel\": \"" + channelKey(b.channel) + "\" }";
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
            m.bands.push_back(b);
        }
    }

    out = m;
    return true;
}

} // namespace eq
} // namespace ssfilter
