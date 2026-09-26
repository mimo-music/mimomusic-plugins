// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "Serialization.hpp"
#include "Json.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>

namespace ssfilter {

namespace {

std::string numToStr(double v)
{
    // JSON has no NaN/Inf literals so those are sanitized to 0 defensively
    // (StabilityGuard should already have removed them before anything is
    // ever serialized).
    if (!std::isfinite(v)) v = 0.0;
    // std::to_chars (unlike snprintf's "%.17g") is guaranteed to always
    // emit '.' as the decimal point regardless of the process's current
    // LC_NUMERIC -- a host that switches the numeric locale away from "C"
    // (e.g. GTK/Qt's setlocale(LC_ALL, "") at init) would otherwise make
    // snprintf write a comma, producing JSON that isn't valid JSON at all
    // and that this file's own reader (Json.hpp, which only recognizes
    // '.' as part of a number) would then silently truncate on reload. It
    // also happens to produce the shortest representation that still
    // round-trips exactly, so no explicit precision is needed.
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof(buf), v);
    return std::string(buf, res.ptr);
}

} // namespace

std::string Serialization::toJson(const StateSpaceMatrices& m)
{
    const int n = clampOrder(m.order);
    std::string s;
    s.reserve(4096);
    s += "{\n";
    s += "  \"type\": \"statespace-mimo-preset\",\n";
    s += "  \"version\": " + std::to_string(kFormatVersion) + ",\n";
    s += "  \"name\": \"" + jsonEscape(m.name) + "\",\n";
    s += "  \"order\": " + std::to_string(n) + ",\n";

    auto writeMatrix = [&](const char* key, int rows, int cols, auto&& elem) {
        s += "  \"" + std::string(key) + "\": [\n";
        for (int i = 0; i < rows; ++i)
        {
            s += "    [";
            for (int j = 0; j < cols; ++j)
            {
                s += numToStr(elem(i, j));
                if (j + 1 < cols) s += ", ";
            }
            s += (i + 1 < rows) ? "],\n" : "]\n";
        }
        s += "  ]";
    };

    writeMatrix("A", n, n, [&](int i, int j) { return m.A[i][j]; });
    s += ",\n";
    writeMatrix("B", n, kNumInputs, [&](int i, int j) { return m.B[i][j]; });
    s += ",\n";
    writeMatrix("C", kNumOutputs, n, [&](int i, int j) { return m.C[i][j]; });
    s += ",\n";
    writeMatrix("D", kNumOutputs, kNumInputs, [&](int i, int j) { return m.D[i][j]; });
    s += ",\n";

    // Optional state-dependent gain stage (see StateSpaceTypes.hpp's
    // NonlinearSchedule) travels in the same file as the matrices.
    s += "  \"nonlinear\": {\n";
    s += "    \"enabled\": " + std::string(m.nonlinear.enabled ? "true" : "false") + ",\n";
    s += "    \"attackMs\": " + numToStr(m.nonlinear.attackMs) + ",\n";
    s += "    \"releaseMs\": " + numToStr(m.nonlinear.releaseMs) + ",\n";
    s += "    \"curveDb\": [";
    for (int i = 0; i < GainCurve::kNumKnots; ++i)
    {
        s += numToStr(m.nonlinear.curve.gainDb[i]);
        if (i + 1 < GainCurve::kNumKnots) s += ", ";
    }
    s += "]\n";
    s += "  },\n";

    // Optional multiband state-dependent gain stage (see
    // StateSpaceTypes.hpp's MultibandSchedule) -- same file, same
    // "optional block absent in older presets means it defaults to
    // disabled" convention as "nonlinear" above.
    s += "  \"multiband\": {\n";
    s += "    \"enabled\": " + std::string(m.multiband.enabled ? "true" : "false") + ",\n";
    s += "    \"bands\": [\n";
    for (int k = 0; k < m.multiband.numBands; ++k)
    {
        const BandSplit& b = m.multiband.bands[k];
        s += "      {\n";
        s += "        \"stateStart\": " + std::to_string(b.stateStart) + ",\n";
        s += "        \"stateCount\": " + std::to_string(b.stateCount) + ",\n";
        s += "        \"feedD\": [[" + numToStr(b.feedD[0][0]) + ", " + numToStr(b.feedD[0][1]) + "], [" +
                                        numToStr(b.feedD[1][0]) + ", " + numToStr(b.feedD[1][1]) + "]],\n";
        s += "        \"enabled\": " + std::string(b.schedule.enabled ? "true" : "false") + ",\n";
        s += "        \"attackMs\": " + numToStr(b.schedule.attackMs) + ",\n";
        s += "        \"releaseMs\": " + numToStr(b.schedule.releaseMs) + ",\n";
        s += "        \"curveDb\": [";
        for (int i = 0; i < GainCurve::kNumKnots; ++i)
        {
            s += numToStr(b.schedule.curve.gainDb[i]);
            if (i + 1 < GainCurve::kNumKnots) s += ", ";
        }
        s += "]\n";
        s += (k + 1 < m.multiband.numBands) ? "      },\n" : "      }\n";
    }
    s += "    ]\n";
    s += "  }\n";

    s += "}\n";
    return s;
}

namespace {

void readMatrix(const JsonValue& root, const char* key, int rows, int cols,
                 double* dst, size_t dstRowStride)
{
    const JsonValue* arr = root.find(key);
    if (arr == nullptr || arr->type != JsonValue::Type::Array) return;

    const int fileRows = static_cast<int>(arr->arr.size());
    for (int i = 0; i < rows && i < fileRows; ++i)
    {
        const JsonValue& rowVal = arr->arr[static_cast<size_t>(i)];
        if (rowVal.type != JsonValue::Type::Array) continue;
        const int fileCols = static_cast<int>(rowVal.arr.size());
        for (int j = 0; j < cols && j < fileCols; ++j)
        {
            const JsonValue& cell = rowVal.arr[static_cast<size_t>(j)];
            if (cell.type == JsonValue::Type::Number)
                dst[static_cast<size_t>(i) * dstRowStride + static_cast<size_t>(j)] = cell.number;
        }
    }
}

} // namespace

bool Serialization::fromJson(const std::string& text, StateSpaceMatrices& out)
{
    JsonValue root;
    if (!JsonParser(text).parse(root) || root.type != JsonValue::Type::Object)
        return false;

    const JsonValue* type = root.find("type");
    if (type == nullptr || type->type != JsonValue::Type::String ||
        type->str != "statespace-mimo-preset")
        return false;

    const JsonValue* orderVal = root.find("order");
    if (orderVal == nullptr || orderVal->type != JsonValue::Type::Number)
        return false;

    StateSpaceMatrices m; // start from a clean, safe default
    m.setSilence();
    m.order = clampOrder(static_cast<int>(orderVal->number));

    if (const JsonValue* nameVal = root.find("name");
        nameVal != nullptr && nameVal->type == JsonValue::Type::String)
    {
        std::snprintf(m.name, sizeof(m.name), "%s", nameVal->str.c_str());
    }

    const int n = m.order;
    readMatrix(root, "A", n, n, &m.A[0][0], kMaxOrder);
    readMatrix(root, "B", n, kNumInputs, &m.B[0][0], kNumInputs);
    readMatrix(root, "C", kNumOutputs, n, &m.C[0][0], kMaxOrder);
    readMatrix(root, "D", kNumOutputs, kNumInputs, &m.D[0][0], kNumInputs);

    // Optional: absent entirely in older/hand-written preset files, which
    // is fine -- m.nonlinear already defaults to disabled/flat.
    if (const JsonValue* nl = root.find("nonlinear"); nl != nullptr && nl->type == JsonValue::Type::Object)
    {
        if (const JsonValue* en = nl->find("enabled"); en != nullptr && en->type == JsonValue::Type::Number)
            m.nonlinear.enabled = en->number != 0.0;
        if (const JsonValue* a = nl->find("attackMs"); a != nullptr && a->type == JsonValue::Type::Number)
            m.nonlinear.attackMs = a->number;
        if (const JsonValue* r = nl->find("releaseMs"); r != nullptr && r->type == JsonValue::Type::Number)
            m.nonlinear.releaseMs = r->number;
        if (const JsonValue* curve = nl->find("curveDb"); curve != nullptr && curve->type == JsonValue::Type::Array)
        {
            const int count = std::min(int(curve->arr.size()), GainCurve::kNumKnots);
            for (int i = 0; i < count; ++i)
                if (curve->arr[size_t(i)].type == JsonValue::Type::Number)
                    m.nonlinear.curve.gainDb[i] = curve->arr[size_t(i)].number;
        }
    }

    // Optional: absent entirely in older/hand-written preset files, same
    // as "nonlinear" above -- m.multiband already defaults to disabled/
    // empty.
    if (const JsonValue* mb = root.find("multiband"); mb != nullptr && mb->type == JsonValue::Type::Object)
    {
        if (const JsonValue* en = mb->find("enabled"); en != nullptr && en->type == JsonValue::Type::Number)
            m.multiband.enabled = en->number != 0.0;
        if (const JsonValue* bandsArr = mb->find("bands"); bandsArr != nullptr && bandsArr->type == JsonValue::Type::Array)
        {
            const int count = std::min(int(bandsArr->arr.size()), kMaxBands);
            m.multiband.numBands = count;
            for (int k = 0; k < count; ++k)
            {
                const JsonValue& bv = bandsArr->arr[size_t(k)];
                if (bv.type != JsonValue::Type::Object) continue;
                BandSplit& band = m.multiband.bands[k];
                if (const JsonValue* v = bv.find("stateStart"); v != nullptr && v->type == JsonValue::Type::Number)
                    band.stateStart = int(v->number);
                if (const JsonValue* v = bv.find("stateCount"); v != nullptr && v->type == JsonValue::Type::Number)
                    band.stateCount = int(v->number);
                readMatrix(bv, "feedD", kNumOutputs, kNumInputs, &band.feedD[0][0], kNumInputs);
                if (const JsonValue* v = bv.find("enabled"); v != nullptr && v->type == JsonValue::Type::Number)
                    band.schedule.enabled = v->number != 0.0;
                if (const JsonValue* v = bv.find("attackMs"); v != nullptr && v->type == JsonValue::Type::Number)
                    band.schedule.attackMs = v->number;
                if (const JsonValue* v = bv.find("releaseMs"); v != nullptr && v->type == JsonValue::Type::Number)
                    band.schedule.releaseMs = v->number;
                if (const JsonValue* curve = bv.find("curveDb"); curve != nullptr && curve->type == JsonValue::Type::Array)
                {
                    const int kcount = std::min(int(curve->arr.size()), GainCurve::kNumKnots);
                    for (int i = 0; i < kcount; ++i)
                        if (curve->arr[size_t(i)].type == JsonValue::Type::Number)
                            band.schedule.curve.gainDb[i] = curve->arr[size_t(i)].number;
                }
            }
        }
    }

    out = m;
    return true;
}

} // namespace ssfilter
