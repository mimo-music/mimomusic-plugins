// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// Serialization.hpp
//
// Save/load a StateSpaceMatrices model as a small, human-readable JSON
// document (extension ".ssjson" is used by convention throughout this
// project, but it's just JSON text). Only the *active* order x order / NxM
// sub-blocks are written -- "order" in the file is authoritative for the
// shape of A/B/C/D, so the matrix-size configuration the user chose is
// exactly what round-trips, and files stay small and readable at low
// orders instead of always dumping the full kMaxOrder-sized backing store.
//
// This is the format used both by DPF host-recall state (the whole plugin
// state is one of these documents, stored as a string) and by the
// "Save/Load Preset..." file-browser actions in the GUI, so a preset saved
// to disk and a host session snapshot are the same thing.

#pragma once

#include "StateSpaceTypes.hpp"

#include <string>

namespace ssfilter {

class Serialization
{
public:
    static constexpr int kFormatVersion = 1;

    // Always succeeds (the matrices struct itself is always well-formed).
    static std::string toJson(const StateSpaceMatrices& m);

    // Parses `text`. On success, fills `out` and returns true. On any
    // failure (malformed JSON, wrong "type", missing fields), leaves `out`
    // untouched and returns false -- callers should keep whatever model was
    // active rather than adopt a partially-parsed one.
    static bool fromJson(const std::string& text, StateSpaceMatrices& out);
};

} // namespace ssfilter
