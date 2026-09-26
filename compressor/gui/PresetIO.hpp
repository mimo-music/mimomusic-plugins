// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// PresetIO.hpp
//
// Minimal, dependency-free preset file I/O -- "Save As.../Load..." to
// export/import a plugin instance's settings as a plain text file, the
// feature ../statespace's own Save/Load buttons cover
// for its (much larger, JSON-shaped) matrix models. This project's entire
// configuration is a handful of scalar knobs plus two toggles (and, for
// the stereo plugin, one more discrete choice), so a hand-rolled
// key=value text format is enough -- no JSON parser needed, matching the
// project family's "dependency-free" preference.
//
// UI-side only (like the sibling project's own Load/Save, which lives
// entirely in UIStateSpace.cpp, not the DSP side) -- the file's contents
// are just the current values of the automatable Parameters, applied back
// via setParameterValue() on load, so the DSP side needs no awareness of
// preset files at all.

#pragma once

#include <charconv>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace presetio {

using KeyValues = std::map<std::string, double>;

// Writes "key=value" lines, one per entry, sorted by key (std::map's own
// order) so the file is stable/diffable across saves. `header` (optional)
// becomes a leading "# " comment line, purely for a human opening the file.
inline bool save(const std::string& path, const KeyValues& values, const char* header = nullptr)
{
    std::ofstream f(path, std::ios::trunc);
    if (!f.good()) return false;
    if (header != nullptr) f << "# " << header << "\n";
    for (const auto& kv : values)
    {
        // std::to_chars, unlike snprintf's "%.6g", always emits '.' as the
        // decimal point regardless of the process's current LC_NUMERIC --
        // a host that adopts a comma-decimal OS locale (GTK/Qt's
        // setlocale(LC_ALL, "") at init is the usual cause) would
        // otherwise write a preset file load()'s own '.'-only parser
        // below can no longer read back correctly.
        char buf[64];
        const auto res = std::to_chars(buf, buf + sizeof(buf), kv.second);
        f << kv.first << "=" << std::string(buf, res.ptr) << "\n";
    }
    return f.good();
}

// Reads back whatever save() wrote. Unknown keys are simply not present in
// the caller's own lookup table it started from -- the caller decides what
// to do with a missing key (usually: leave that parameter untouched), so
// a preset saved by one of this project's plugins and opened in the other
// degrades gracefully rather than failing outright (e.g. a mono preset
// has no "stereoMode" entry; loading it into the stereo plugin just
// leaves the stereo mode as it already was).
inline bool load(const std::string& path, KeyValues& values)
{
    std::ifstream f(path);
    if (!f.good()) return false;
    values.clear();

    std::string line;
    while (std::getline(f, line))
    {
        if (line.empty() || line[0] == '#') continue;
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string val = line.substr(eq + 1);
        // std::from_chars, unlike std::strtod, is locale-independent --
        // see save()'s comment above for why that matters.
        double v = 0.0;
        const auto res = std::from_chars(val.data(), val.data() + val.size(), v);
        if (res.ec == std::errc()) values[key] = v;
    }
    return true;
}

} // namespace presetio
