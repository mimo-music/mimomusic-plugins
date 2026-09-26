// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ModelEditorHost.hpp
//
// Small interface that decouples the matrix-editing widgets from the main
// UI class. The widgets never touch a StateSpaceMatrices directly (and
// don't know about DPF at all) -- they only read/write cell values and ask
// to start a text edit through this interface. UIStateSpace implements it,
// owning the actual model, the stability re-check on every edit, and
// pushing updates to the DSP + Bode plot.

#pragma once

#include "../plugin/dsp/StateSpaceTypes.hpp"

#include <string>

namespace ssgui {

// `Nonlinear` isn't one of the model's real coefficient matrices -- it's a
// virtual 1x5 "matrix" that lets NonlinearPanelWidget's numeric fields
// (attack, release, and the transient threshold/ratio/makeup convenience
// fields -- see below) reuse the exact same click-to-edit/double-click-to-
// clear/Enter-commits/arrow-key-navigate/scroll-to-nudge machinery that
// MatrixGridWidget and UIStateSpace's editing state machine already
// implement for A/B/C/D, instead of duplicating it. Columns: 0=attackMs,
// 1=releaseMs, 2=threshold(dB), 3=ratio(:1), 4=makeup(dB) -- the last three
// are *not* stored in StateSpaceMatrices (the model only ever remembers the
// resulting 25-knot curve, not "how" it was authored, exactly like the
// biquad mapper's Q/frequency aren't remembered once realized into A/B/C/D)
// and live only as transient UI state until "Apply Curve" bakes them into
// the curve's knots.
enum class MatrixKind { A, B, C, D, Nonlinear };

// Shared by MatrixGridWidget (to size/lay out its grid) and UIStateSpace
// (to clamp arrow-key cell navigation) so the two never drift apart.
inline void computeMatrixDims(MatrixKind kind, int order, int& rows, int& cols)
{
    switch (kind)
    {
        case MatrixKind::A: rows = order; cols = order; break;
        case MatrixKind::B: rows = order; cols = ssfilter::kNumInputs; break;
        case MatrixKind::C: rows = ssfilter::kNumOutputs; cols = order; break;
        case MatrixKind::D: rows = ssfilter::kNumOutputs; cols = ssfilter::kNumInputs; break;
        case MatrixKind::Nonlinear: rows = 1; cols = 5; break;
    }
}

class IModelEditorHost
{
public:
    virtual ~IModelEditorHost() = default;

    // Current active state order (drives A's NxN size, B's Nx2, C's 2xN).
    virtual int getOrder() const = 0;

    virtual double getMatrixValue(MatrixKind kind, int row, int col) const = 0;

    // Commits a new value immediately: re-runs the stability guard, pushes
    // the (possibly clamped) result to the audio engine, refreshes the Bode
    // plot, and marks the host state dirty for session persistence.
    virtual void setMatrixValue(MatrixKind kind, int row, int col, double value) = 0;

    // Starts interactive text entry on one cell (click-to-edit). When
    // `clearValue` is true (a double-click) the edit buffer starts empty
    // so typing replaces the value outright; otherwise it starts pre-filled
    // with the current value (a single click), so a plain click still
    // supports fine edits (e.g. backspacing a digit).
    virtual void beginCellTextEdit(MatrixKind kind, int row, int col, bool clearValue = false) = 0;

    // If this exact cell is the one currently being text-edited, returns
    // true and fills `liveText` with the in-progress buffer (so the grid
    // widget can render it instead of the committed value).
    virtual bool isEditingCell(MatrixKind kind, int row, int col, std::string& liveText) const = 0;

    // -- Nonlinear (state-dependent gain) stage --------------------------
    // The 25-knot gain curve itself isn't a grid of independently-typed
    // numbers like A/B/C/D -- it's shaped by direct manipulation on
    // NonlinearPanelWidget's graph (click/drag sets the nearest knot from
    // the pointer's position, scroll nudges it), so it gets its own small
    // pair of accessors rather than being shoehorned into the MatrixKind
    // cell interface above.
    virtual bool getNonlinearEnabled() const = 0;
    virtual void setNonlinearEnabled(bool enabled) = 0;
    virtual double getNonlinearKnotGainDb(int knot) const = 0;
    virtual void setNonlinearKnotGainDb(int knot, double gainDb) = 0;
};

} // namespace ssgui
