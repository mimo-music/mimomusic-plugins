// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "DynamicEqModel.hpp"
#include "EqModel.hpp" // for eq::evaluateBiquadResponse(), see this file's own use in evaluateBandResponse()
#include "Json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace ssfilter {
namespace dyneq {

namespace {
// Q/freq clamping identical to eq::designBand()'s own -- same reasoning
// (keep the RBJ-derived formulas away from undefined territory; the
// engine's own per-sample processing has no separate stability guard to
// fall back on the way the generic StateSpaceEngine path does, since A(z)
// is fixed and gain never touches it -- see file comment).
double clampedQ(double q) { return std::max(0.05, std::min(100.0, q)); }
double clampedFreq(double freqHz, double sampleRate) { return std::max(1.0, std::min(sampleRate * 0.499, freqHz)); }
} // namespace

std::vector<int> activeBandIndices(const DynamicEqModel& model)
{
    // Soloing a disabled band has no effect (nothing to spotlight) --
    // same edge case as mb::hasSolo()'s identical guard.
    const bool soloActive = model.soloedBand >= 0 && model.soloedBand < int(model.bands.size())
        && model.bands[size_t(model.soloedBand)].enabled;

    std::vector<int> out;
    for (int i = 0; i < int(model.bands.size()); ++i)
    {
        const DynamicEqBand& b = model.bands[size_t(i)];
        if (!b.enabled) continue;
        if (soloActive && i != model.soloedBand) continue;
        out.push_back(i);
    }
    return out;
}

bool hasPerChannelBands(const DynamicEqModel& model)
{
    for (int i : activeBandIndices(model))
        if (model.bands[size_t(i)].channel != EqChannel::Both) return true;
    return false;
}

BiquadCoeffs designShaperAllpass(const DynamicEqBand& band, double sampleRate)
{
    return mappers::designAllpass(sampleRate, clampedFreq(band.freqHz, sampleRate), clampedQ(band.q));
}

BiquadCoeffs designDetectorBandpass(const DynamicEqBand& band, double sampleRate)
{
    return mappers::designBandpass(sampleRate, clampedFreq(band.freqHz, sampleRate), clampedQ(band.q));
}

double resolveDynamicGainDb(const DynamicEqBand& band, double envelopeDb)
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

std::complex<double> evaluateBandResponse(const DynamicEqBand& band, double sampleRate, double freqHz, double resolvedGainDb)
{
    const double omega = 2.0 * M_PI * freqHz / sampleRate;
    const std::complex<double> Aap = eq::evaluateBiquadResponse(designShaperAllpass(band, sampleRate), omega);
    const double G = std::pow(10.0, resolvedGainDb / 20.0);
    return 0.5 * (1.0 + G) + 0.5 * (1.0 - G) * Aap;
}

std::complex<double> evaluateCombinedResponse(const DynamicEqModel& model, double sampleRate, double freqHz,
                                               const double* currentGainDb)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const DynamicEqBand& b = model.bands[size_t(i)];
        const double g = currentGainDb != nullptr ? currentGainDb[size_t(i)] : b.gainDb;
        h *= evaluateBandResponse(b, sampleRate, freqHz, g);
    }
    return h;
}

std::complex<double> evaluateChannelResponse(const DynamicEqModel& model, double sampleRate, double freqHz,
                                              EqChannel listenerChannel, const double* currentGainDb)
{
    std::complex<double> h(std::pow(10.0, model.outputTrimDb / 20.0), 0.0);
    for (int i : activeBandIndices(model))
    {
        const DynamicEqBand& b = model.bands[size_t(i)];
        if (listenerChannel == EqChannel::Left && b.channel == EqChannel::Right) continue;
        if (listenerChannel == EqChannel::Right && b.channel == EqChannel::Left) continue;
        const double g = currentGainDb != nullptr ? currentGainDb[size_t(i)] : b.gainDb;
        h *= evaluateBandResponse(b, sampleRate, freqHz, g);
    }
    return h;
}

StateSpaceMatrices assembleStateSpace(const DynamicEqModel& model, double sampleRate)
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
        // both, a Left/Right band's goes into only its own.
        std::vector<BiquadCoeffs> leftStages, rightStages;
        leftStages.reserve(active.size());
        rightStages.reserve(active.size());
        for (int i : active)
        {
            const DynamicEqBand& b = model.bands[size_t(i)];
            const BiquadCoeffs stage = mappers::designPeaking(sampleRate, clampedFreq(b.freqHz, sampleRate), clampedQ(b.q), b.gainDb);
            if (b.channel != EqChannel::Right) leftStages.push_back(stage);
            if (b.channel != EqChannel::Left) rightStages.push_back(stage);
        }
        m = mappers::makeStereoBiquadChain(leftStages, rightStages);
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

std::string toJson(const DynamicEqModel& model)
{
    std::string s;
    s.reserve(256 + model.bands.size() * 220);
    s += "{\n";
    s += "  \"type\": \"statespace-dyneq-preset\",\n";
    s += "  \"version\": 1,\n";
    s += "  \"name\": \"" + jsonEscape(model.name) + "\",\n";
    s += "  \"outputTrimDb\": " + numToStr(model.outputTrimDb) + ",\n";
    s += "  \"soloedBand\": " + std::to_string(model.soloedBand) + ",\n";
    s += "  \"bands\": [\n";
    for (size_t i = 0; i < model.bands.size(); ++i)
    {
        const DynamicEqBand& b = model.bands[i];
        s += "    { \"freqHz\": " + numToStr(b.freqHz) + ", \"q\": " + numToStr(b.q) +
             ", \"gainDb\": " + numToStr(b.gainDb) + ", \"thresholdDb\": " + numToStr(b.thresholdDb) +
             ", \"ratio\": " + numToStr(b.ratio) + ", \"ratioUp\": " + numToStr(b.ratioUp) +
             ", \"rangeDb\": " + numToStr(b.rangeDb) +
             ", \"attackMs\": " + numToStr(b.attackMs) + ", \"releaseMs\": " + numToStr(b.releaseMs) +
             ", \"dynamicsEnabled\": " + (b.dynamicsEnabled ? "true" : "false") +
             ", \"enabled\": " + (b.enabled ? "true" : "false") +
             ", \"channel\": \"" + channelKey(b.channel) + "\" }";
        s += (i + 1 < model.bands.size()) ? ",\n" : "\n";
    }
    s += "  ]\n";
    s += "}\n";
    return s;
}

bool fromJson(const std::string& text, DynamicEqModel& out)
{
    JsonValue root;
    if (!JsonParser(text).parse(root) || root.type != JsonValue::Type::Object)
        return false;

    const JsonValue* type = root.find("type");
    if (type == nullptr || type->type != JsonValue::Type::String || type->str != "statespace-dyneq-preset")
        return false;

    DynamicEqModel m;
    if (const JsonValue* nameVal = root.find("name");
        nameVal != nullptr && nameVal->type == JsonValue::Type::String)
        std::snprintf(m.name, sizeof(m.name), "%s", nameVal->str.c_str());

    if (const JsonValue* trim = root.find("outputTrimDb"); trim != nullptr && trim->type == JsonValue::Type::Number)
        m.outputTrimDb = trim->number;

    int soloedBandRaw = -1; // clamped against the final band count below, once it's known
    if (const JsonValue* so = root.find("soloedBand"); so != nullptr && so->type == JsonValue::Type::Number)
        soloedBandRaw = int(so->number);

    if (const JsonValue* bands = root.find("bands"); bands != nullptr && bands->type == JsonValue::Type::Array)
    {
        for (const JsonValue& bv : bands->arr)
        {
            if (bv.type != JsonValue::Type::Object) continue;
            DynamicEqBand b;
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

} // namespace dyneq
} // namespace ssfilter
