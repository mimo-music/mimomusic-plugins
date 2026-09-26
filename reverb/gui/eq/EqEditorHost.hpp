// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// EqEditorHost.hpp
//
// Same purpose and shape as gui/ModelEditorHost.hpp, one level up (which
// decouples the base plugin's matrix grids from its main UI class): this
// decouples EqGraphWidget and BandListWidget from UIEq, which owns the
// actual ssfilter::EqModel, reassembles+enforces+pushes it to the DSP on
// every edit, and drives the two widgets' text-edit state machine.

#pragma once

#include "../../plugin/dsp/EqModel.hpp"

#include <string>

namespace ssgui {

// Which numeric field of a band is being addressed -- analogous to
// MatrixKind's (kind,row,col) cell address, but for a dynamically-sized
// band list instead of a fixed matrix.
enum class EqField { Freq, Gain, Q };

class IEqEditorHost
{
public:
    virtual ~IEqEditorHost() = default;

    virtual int getBandCount() const = 0;
    virtual ssfilter::EqBand getBand(int index) const = 0;

    // The whole model, read-only -- used for the combined-response curve
    // (ssfilter::eq::evaluateCombinedResponse() needs the full band list at
    // once to apply its enabled/solo logic, see EqModel.hpp's
    // activeBandIndices()) and for Load/Save/Export.
    virtual ssfilter::EqModel getModel() const = 0;

    // Commits a new value immediately: reassembles the model, re-runs the
    // stability guard, pushes the result to the audio engine, and marks
    // both the graph and the host's session state dirty. Mirrors
    // IModelEditorHost::setMatrixValue()'s contract exactly.
    virtual void setBandFreq(int index, double freqHz) = 0;
    virtual void setBandGain(int index, double gainDb) = 0;
    virtual void setBandQ(int index, double q) = 0;
    virtual void setBandType(int index, ssfilter::EqBandType type) = 0;
    virtual void setBandSlopeStages(int index, int slopeStages) = 0; // Low/High Pass steepness, see EqBand::slopeStages
    virtual void setBandEnabled(int index, bool enabled) = 0;
    virtual void setBandSolo(int index, bool solo) = 0;

    // Which stereo channel(s) this band filters -- see EqBand::channel's
    // own comment. Implemented as a no-op on a mono host (UIEqMono): a
    // single channel has nothing to assign, and BandListWidget's own
    // Channel column (the only place this is reachable from) is only
    // shown at all when the host says stereo, see BandListWidget's own
    // constructor comment.
    virtual void setBandChannel(int index, ssfilter::EqChannel channel) = 0;

    // Appends a new Bell band at (freqHz, gainDb) with a default Q, and
    // removes one -- the two ways EqGraphWidget's "click empty space" /
    // "double-click a node" gestures change the band *count* rather than
    // just a field.
    virtual void addBand(double freqHz, double gainDb) = 0;
    virtual void removeBand(int index) = 0;

    // Inserts an exact copy of `index` right after it (same type, freq,
    // gain, Q, slopeStages, enabled/solo) and selects the new copy --
    // reachable from EqGraphWidget's right-click menu ("Duplicate Band",
    // next to "Delete Band").
    virtual void duplicateBand(int index) = 0;

    // Harmonic linking -- see EqBand::harmonicGroup's own comment
    // (EqModel.hpp) for the full data-model reasoning. createHarmonicTwin()
    // is reachable from EqGraphWidget's right-click menu ("Create Harmonic
    // Twin" submenu); unlinkHarmonic() from the same menu's "Unlink"
    // entry, shown only for an already-linked band. No "relink" is
    // offered, deliberately -- not needed.
    virtual void createHarmonicTwin(int index, double factor) = 0;
    virtual void unlinkHarmonic(int index) = 0;

    // Momentary "preview just this band's own isolated frequency region"
    // -- driven by EqGraphWidget's own right-click-and-hold gesture (see
    // that file's header comment), previously middle-click-and-hold-to-
    // solo (moved here: middle-click was uncomfortable, and Audition
    // is the more useful thing to preview with a quick hold anyway, since
    // it isolates the actual frequency region rather than muting every
    // *other* band). Distinct from the persistent Audition button in
    // UIEq's own toolbar -- beginBandAudition() engages Audition on
    // `index` regardless of whatever the persistent button/selection
    // state was, and endBandAudition() restores exactly that persistent
    // state on release, so a momentary hold can never leave Audition
    // stuck on (or off) relative to what the button itself shows.
    // Implemented as a no-op pair on EQ Mono (UIEqMono.cpp), which has no
    // Audition DSP of its own.
    virtual void beginBandAudition(int index) = 0;
    virtual void endBandAudition() = 0;

    // Text-edit machinery, mirroring IModelEditorHost's
    // beginCellTextEdit()/isEditingCell() but keyed by (bandIndex, field).
    virtual void beginBandTextEdit(int index, EqField field, bool clearValue = false) = 0;
    virtual bool isEditingBandField(int index, EqField field, std::string& liveText) const = 0;

    // The "Create Harmonic Twin -> Custom Factor..." text edit -- same
    // shape as beginBandTextEdit()/isEditingBandField() above, keyed by
    // the *source* band index (the one the twin will be created from)
    // rather than (band, field), since there's no field/band pair yet --
    // nothing is created until the typed factor commits. UIEq owns the
    // actual edit state (and the keyboard routing that feeds it, per this
    // project's "keyboard events go to the top-level UI" convention);
    // commitHarmonicFactorEdit() is what EqGraphWidget's own "click
    // anywhere while this input is showing" handler calls to end it.
    virtual void beginHarmonicFactorEdit(int sourceIndex) = 0;
    virtual bool isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const = 0;
    virtual void commitHarmonicFactorEdit() = 0;
};

} // namespace ssgui
