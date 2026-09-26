// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// Json.hpp
//
// A minimal, dependency-free JSON reader used only to load/save
// state-space presets as plain human-editable/versionable text files. Not
// a general-purpose JSON library: it covers objects, arrays, numbers,
// strings, true/false/null, which is all a preset file needs, and nothing
// about it is performance sensitive (it never runs anywhere near the audio
// thread).

#pragma once

#include <cctype>
#include <charconv>
#include <string>
#include <utility>
#include <vector>

namespace ssfilter {

struct JsonValue
{
    enum class Type { Null, Number, String, Array, Object };
    Type type = Type::Null;
    double number = 0.0;
    std::string str;
    std::vector<JsonValue> arr;
    std::vector<std::pair<std::string, JsonValue>> obj;

    const JsonValue* find(const std::string& key) const
    {
        for (const auto& kv : obj)
            if (kv.first == key) return &kv.second;
        return nullptr;
    }
};

class JsonParser
{
public:
    explicit JsonParser(const std::string& text) : s_(text) {}

    bool parse(JsonValue& out)
    {
        skipWs();
        return parseValue(out);
    }

private:
    const std::string& s_;
    size_t i_ = 0;

    void skipWs() { while (i_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[i_]))) ++i_; }

    bool parseValue(JsonValue& v)
    {
        skipWs();
        if (i_ >= s_.size()) return false;
        const char c = s_[i_];
        if (c == '{') return parseObject(v);
        if (c == '[') return parseArray(v);
        if (c == '"') return parseString(v);
        if (c == '-' || std::isdigit(static_cast<unsigned char>(c))) return parseNumber(v);
        if (s_.compare(i_, 4, "true") == 0)  { v.type = JsonValue::Type::Number; v.number = 1; i_ += 4; return true; }
        if (s_.compare(i_, 5, "false") == 0) { v.type = JsonValue::Type::Number; v.number = 0; i_ += 5; return true; }
        if (s_.compare(i_, 4, "null") == 0)  { v.type = JsonValue::Type::Null; i_ += 4; return true; }
        return false;
    }

    bool parseObject(JsonValue& v)
    {
        v.type = JsonValue::Type::Object;
        ++i_;
        skipWs();
        if (i_ < s_.size() && s_[i_] == '}') { ++i_; return true; }
        while (true)
        {
            skipWs();
            JsonValue key;
            if (!parseString(key)) return false;
            skipWs();
            if (i_ >= s_.size() || s_[i_] != ':') return false;
            ++i_;
            JsonValue val;
            if (!parseValue(val)) return false;
            v.obj.emplace_back(key.str, std::move(val));
            skipWs();
            if (i_ < s_.size() && s_[i_] == ',') { ++i_; continue; }
            if (i_ < s_.size() && s_[i_] == '}') { ++i_; break; }
            return false;
        }
        return true;
    }

    bool parseArray(JsonValue& v)
    {
        v.type = JsonValue::Type::Array;
        ++i_;
        skipWs();
        if (i_ < s_.size() && s_[i_] == ']') { ++i_; return true; }
        while (true)
        {
            JsonValue val;
            if (!parseValue(val)) return false;
            v.arr.push_back(std::move(val));
            skipWs();
            if (i_ < s_.size() && s_[i_] == ',') { ++i_; continue; }
            if (i_ < s_.size() && s_[i_] == ']') { ++i_; break; }
            return false;
        }
        return true;
    }

    bool parseString(JsonValue& v)
    {
        if (i_ >= s_.size() || s_[i_] != '"') return false;
        ++i_;
        std::string out;
        while (i_ < s_.size() && s_[i_] != '"')
        {
            const char c = s_[i_];
            if (c == '\\' && i_ + 1 < s_.size())
            {
                const char n = s_[i_ + 1];
                switch (n)
                {
                    case 'n': out += '\n'; break;
                    case 't': out += '\t'; break;
                    case 'r': out += '\r'; break;
                    case '"': out += '"'; break;
                    case '\\': out += '\\'; break;
                    case '/': out += '/'; break;
                    default: out += n; break;
                }
                i_ += 2;
            }
            else { out += c; ++i_; }
        }
        if (i_ >= s_.size()) return false;
        ++i_; // closing quote
        v.type = JsonValue::Type::String;
        v.str = std::move(out);
        return true;
    }

    bool parseNumber(JsonValue& v)
    {
        const size_t start = i_;
        if (i_ < s_.size() && s_[i_] == '-') ++i_;
        while (i_ < s_.size() &&
               (std::isdigit(static_cast<unsigned char>(s_[i_])) ||
                s_[i_] == '.' || s_[i_] == 'e' || s_[i_] == 'E' || s_[i_] == '+' || s_[i_] == '-'))
            ++i_;
        const std::string tok = s_.substr(start, i_ - start);
        if (tok.empty()) return false;
        // std::from_chars (unlike std::stod/strtod) is guaranteed
        // locale-independent -- it always expects '.' as the decimal
        // point regardless of the process's current LC_NUMERIC. That
        // matters here: a host that calls setlocale(LC_ALL, "") (common
        // in GTK/Qt init) leaves every plugin instance running under
        // whatever numeric locale the OS is set to, and under a
        // comma-decimal locale std::stod("0.707") used to silently stop
        // at the '.' and return 0 -- corrupting any non-integer
        // frequency/gain/Q the moment a preset or session was loaded.
        double parsed = 0.0;
        const auto res = std::from_chars(tok.data(), tok.data() + tok.size(), parsed);
        if (res.ec != std::errc()) return false;
        v.number = parsed;
        v.type = JsonValue::Type::Number;
        return true;
    }
};

inline std::string jsonEscape(const std::string& in)
{
    std::string out;
    out.reserve(in.size() + 4);
    for (char c : in)
    {
        switch (c)
        {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            case '\r': out += "\\r"; break;
            default: out += c; break;
        }
    }
    return out;
}

} // namespace ssfilter
