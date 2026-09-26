// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "MbCompressorModel.hpp"
#include "EqModel.hpp" // reuses eq::evaluateBiquadResponse() -- see evaluateCombinedResponse()'s own comment
#include "Json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace ssfilter {
namespace mb {

bool hasSolo(const MbCompressorModel& model)
{
    return model.soloedBand >= 0 && model.soloedBand < int(model.bands.size())
        && model.bands[size_t(model.soloedBand)].enabled;
}

NonlinearSchedule designBandSchedule(const MbBand& band, bool audible)
{
    NonlinearSchedule s;
    s.enabled = true;
    s.attackMs = std::max(0.05, band.attackMs);
    s.releaseMs = std::max(1.0, band.releaseMs);

    if (!audible)
    {
        // Muted because a *different* band is the one soloed: attenuate hard at every level,
        // rather than needing a structural "this band doesn't exist"
        // special case anywhere the engine or the graph would otherwise
        // have to know about. -100dB is comfortably below the noise floor
        // of any 32-bit float signal path in this project.
        for (int i = 0; i < GainCurve::kNumKnots; ++i) s.curve.gainDb[i] = -100.0;
        return s;
    }
    if (!band.enabled)
        return s; // flat/transparent (curve defaults to all-zero) -- dynamics bypassed, audio still plays

    const double ratio = std::max(1.0, band.ratio);
    const double ratioUp = std::max(1.0, band.ratioUp);
    s.curve = GainCurve::classicCompressor(band.thresholdDb, ratio, ratioUp, band.makeupDb);
    return s;
}

StateSpaceMatrices assembleStateSpace(const MbCompressorModel& model, double sampleRate)
{
    const auto bandStages = mappers::designCrossoverBandStages(sampleRate, model.crossoverHz, model.lrOrder);

    MultibandSchedule schedule;
    StateSpaceMatrices m = mappers::makeStereoCrossoverBank(bandStages, schedule);
    schedule.enabled = true;

    const bool soloActive = hasSolo(model);
    const int n = std::min(int(model.bands.size()), schedule.numBands);
    for (int k = 0; k < n; ++k)
    {
        const MbBand& b = model.bands[size_t(k)];
        const bool audible = !soloActive || k == model.soloedBand;
        schedule.bands[k].schedule = designBandSchedule(b, audible);
    }
    m.multiband = schedule;

    // Output trim: scales C/D only (never A/B), same convention as
    // EqModel::assembleStateSpace()'s own copy of this -- a pure gain
    // change can never affect stability.
    if (model.outputTrimDb != 0.0)
    {
        const double g = std::pow(10.0, model.outputTrimDb / 20.0);
        const int order = clampOrder(m.order);
        for (int i = 0; i < kNumOutputs; ++i)
        {
            for (int j = 0; j < order; ++j) m.C[size_t(i)][size_t(j)] *= g;
            for (int j = 0; j < kNumInputs; ++j) m.D[size_t(i)][size_t(j)] *= g;
        }
        // The per-band feedD bookkeeping (BandSplit.feedD) must scale by
        // the same trim, or the engine's multiband branch would apply the
        // trim to the recursive C-slice contribution but not to the
        // direct feed-through term, silently reintroducing the untrimmed
        // level into the per-band envelope/output.
        for (int k = 0; k < schedule.numBands; ++k)
            for (int i = 0; i < kNumOutputs; ++i)
                for (int j = 0; j < kNumInputs; ++j)
                    m.multiband.bands[k].feedD[i][j] *= g;
    }

    std::snprintf(m.name, sizeof(m.name), "%s", model.name);
    return m;
}

std::complex<double> evaluateCombinedResponse(const MbCompressorModel& model, double sampleRate, double freqHz)
{
    const auto bandStages = mappers::designCrossoverBandStages(sampleRate, model.crossoverHz, model.lrOrder);
    const double omega = 2.0 * M_PI * freqHz / sampleRate;

    std::complex<double> total(0.0, 0.0);
    for (const auto& stages : bandStages)
    {
        std::complex<double> h(1.0, 0.0);
        for (const auto& bq : stages) h *= eq::evaluateBiquadResponse(bq, omega);
        total += h;
    }
    return total * std::pow(10.0, model.outputTrimDb / 20.0);
}

int bandIndexForFreq(const MbCompressorModel& model, double freqHz)
{
    int idx = 0;
    for (double f : model.crossoverHz)
        if (freqHz >= f) ++idx; else break;
    return idx;
}

int clampLrOrder(int order)
{
    // Snap to the nearest of {2,4,6,8} rather than just clamping the
    // endpoints, so a hand-edited/foreign preset with e.g. lrOrder=5 lands
    // on the closer of 4/6, not silently on 2 or 8.
    order = std::max(2, std::min(8, order));
    return ((order + 1) / 2) * 2; // round to the nearest even number
}

int nextLrOrder(int order)
{
    order = clampLrOrder(order);
    return order >= 8 ? 2 : order + 2;
}

const char* lrOrderLabel(int order)
{
    switch (clampLrOrder(order))
    {
    case 2: return "12 dB/oct";
    case 4: return "24 dB/oct";
    case 6: return "36 dB/oct";
    default: return "48 dB/oct";
    }
}

void addCrossover(MbCompressorModel& model, double freqHz)
{
    if (int(model.bands.size()) >= kMaxBands) return; // refuse rather than silently truncate, like UIEq::addBand()
    freqHz = std::max(20.0, std::min(20000.0, freqHz));

    // Refuse a split that would land right on (or within ~a semitone-ish
    // ratio of) an existing one -- not a hard requirement, just avoids
    // creating two indistinguishable, hard-to-separate handles on the
    // graph from one click.
    for (double f : model.crossoverHz)
        if (std::fabs(std::log(f) - std::log(freqHz)) < 0.02) return;

    const int k = bandIndexForFreq(model, freqHz); // which band is being split
    model.crossoverHz.insert(model.crossoverHz.begin() + k, freqHz);
    model.bands.insert(model.bands.begin() + k + 1, model.bands[size_t(k)]); // upper half starts as a copy
}

void removeCrossover(MbCompressorModel& model, int crossoverIndex)
{
    if (crossoverIndex < 0 || crossoverIndex >= int(model.crossoverHz.size())) return;
    model.crossoverHz.erase(model.crossoverHz.begin() + crossoverIndex);
    model.bands.erase(model.bands.begin() + crossoverIndex + 1); // the lower band's settings win
}

void setCrossoverHz(MbCompressorModel& model, int crossoverIndex, double freqHz)
{
    if (crossoverIndex < 0 || crossoverIndex >= int(model.crossoverHz.size()) || !std::isfinite(freqHz)) return;
    const double lo = crossoverIndex == 0 ? 20.0 : model.crossoverHz[size_t(crossoverIndex - 1)] * 1.05;
    const double hi = crossoverIndex + 1 < int(model.crossoverHz.size())
                           ? model.crossoverHz[size_t(crossoverIndex + 1)] / 1.05
                           : 20000.0;
    model.crossoverHz[size_t(crossoverIndex)] = std::max(lo, std::min(hi, freqHz));
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

} // namespace

std::string toJson(const MbCompressorModel& model)
{
    std::string s;
    s.reserve(256 + model.bands.size() * 160);
    s += "{\n";
    s += "  \"type\": \"statespace-mb-compressor-preset\",\n";
    s += "  \"version\": 1,\n";
    s += "  \"name\": \"" + jsonEscape(model.name) + "\",\n";
    s += "  \"lrOrder\": " + std::to_string(model.lrOrder) + ",\n";
    s += "  \"soloedBand\": " + std::to_string(model.soloedBand) + ",\n";
    s += "  \"outputTrimDb\": " + numToStr(model.outputTrimDb) + ",\n";
    s += "  \"crossoverHz\": [";
    for (size_t i = 0; i < model.crossoverHz.size(); ++i)
    {
        s += numToStr(model.crossoverHz[i]);
        if (i + 1 < model.crossoverHz.size()) s += ", ";
    }
    s += "],\n";
    s += "  \"bands\": [\n";
    for (size_t i = 0; i < model.bands.size(); ++i)
    {
        const MbBand& b = model.bands[i];
        s += "    { \"thresholdDb\": " + numToStr(b.thresholdDb) + ", \"ratio\": " + numToStr(b.ratio) +
             ", \"ratioUp\": " + numToStr(b.ratioUp) +
             ", \"attackMs\": " + numToStr(b.attackMs) + ", \"releaseMs\": " + numToStr(b.releaseMs) +
             ", \"makeupDb\": " + numToStr(b.makeupDb) +
             ", \"enabled\": " + (b.enabled ? "true" : "false") + " }";
        s += (i + 1 < model.bands.size()) ? ",\n" : "\n";
    }
    s += "  ]\n";
    s += "}\n";
    return s;
}

bool fromJson(const std::string& text, MbCompressorModel& out)
{
    JsonValue root;
    if (!JsonParser(text).parse(root) || root.type != JsonValue::Type::Object)
        return false;

    const JsonValue* type = root.find("type");
    if (type == nullptr || type->type != JsonValue::Type::String || type->str != "statespace-mb-compressor-preset")
        return false;

    MbCompressorModel m;
    m.crossoverHz.clear();
    m.bands.clear();
    int soloedBandRaw = -1; // clamped against the final band count below, once it's known

    if (const JsonValue* nameVal = root.find("name");
        nameVal != nullptr && nameVal->type == JsonValue::Type::String)
        std::snprintf(m.name, sizeof(m.name), "%s", nameVal->str.c_str());

    if (const JsonValue* lr = root.find("lrOrder"); lr != nullptr && lr->type == JsonValue::Type::Number)
        m.lrOrder = clampLrOrder(int(lr->number));

    if (const JsonValue* so = root.find("soloedBand"); so != nullptr && so->type == JsonValue::Type::Number)
        soloedBandRaw = int(so->number);

    if (const JsonValue* trim = root.find("outputTrimDb"); trim != nullptr && trim->type == JsonValue::Type::Number)
        m.outputTrimDb = trim->number;

    if (const JsonValue* xo = root.find("crossoverHz"); xo != nullptr && xo->type == JsonValue::Type::Array)
        for (const JsonValue& v : xo->arr)
            if (v.type == JsonValue::Type::Number) m.crossoverHz.push_back(v.number);

    if (const JsonValue* bands = root.find("bands"); bands != nullptr && bands->type == JsonValue::Type::Array)
    {
        for (const JsonValue& bv : bands->arr)
        {
            if (bv.type != JsonValue::Type::Object) continue;
            MbBand b;
            if (const JsonValue* v = bv.find("thresholdDb"); v != nullptr && v->type == JsonValue::Type::Number)
                b.thresholdDb = v->number;
            if (const JsonValue* v = bv.find("ratio"); v != nullptr && v->type == JsonValue::Type::Number)
                b.ratio = v->number;
            if (const JsonValue* v = bv.find("ratioUp"); v != nullptr && v->type == JsonValue::Type::Number)
                b.ratioUp = v->number;
            if (const JsonValue* v = bv.find("attackMs"); v != nullptr && v->type == JsonValue::Type::Number)
                b.attackMs = v->number;
            if (const JsonValue* v = bv.find("releaseMs"); v != nullptr && v->type == JsonValue::Type::Number)
                b.releaseMs = v->number;
            if (const JsonValue* v = bv.find("makeupDb"); v != nullptr && v->type == JsonValue::Type::Number)
                b.makeupDb = v->number;
            if (const JsonValue* v = bv.find("enabled"); v != nullptr && v->type == JsonValue::Type::Number)
                b.enabled = v->number != 0.0;
            m.bands.push_back(b);
        }
    }

    // Defensive: a hand-edited/malformed/foreign file could have the wrong
    // band count for its own crossover list -- fall back to one band
    // rather than let assembleStateSpace() index past the end.
    if (m.bands.size() != m.crossoverHz.size() + 1)
    {
        m.crossoverHz.clear();
        m.bands.assign(1, m.bands.empty() ? MbBand{} : m.bands.front());
    }

    // Clamp against the *final* band count (after the defensive fixup
    // above may have shrunk it) rather than trusting a hand-edited/
    // foreign file's own value -- an out-of-range index would otherwise
    // silently disable solo instead of soloing nothing in particular, or
    // (worse) alias onto whatever band now happens to sit at that index.
    m.soloedBand = (soloedBandRaw >= 0 && soloedBandRaw < int(m.bands.size())) ? soloedBandRaw : -1;

    out = m;
    return true;
}

} // namespace mb
} // namespace ssfilter
