// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MbEditorHost.hpp
//
// Same purpose and shape as gui/eq/EqEditorHost.hpp, one level up: this
// decouples MultibandGraphWidget (and the per-band ParamStripWidget in
// UIMbCompressor) from UIMbCompressor itself, which owns the actual
// ssfilter::MbCompressorModel, reassembles+enforces+pushes it to the DSP
// on every edit.

#pragma once

#include "../../plugin/dsp/MbCompressorModel.hpp"

namespace ssgui {

class IMbEditorHost
{
public:
    virtual ~IMbEditorHost() = default;

    virtual int getBandCount() const = 0;
    virtual ssfilter::MbBand getBand(int index) const = 0;

    // The whole model, read-only -- used for the combined crossover-shape
    // curve (ssfilter::mb::evaluateCombinedResponse() needs the full
    // crossover list at once) and for Load/Save/Export.
    virtual ssfilter::MbCompressorModel getModel() const = 0;

    // Number of crossover *points* is always getBandCount()-1.
    virtual double getCrossoverHz(int crossoverIndex) const = 0;
    virtual void setCrossoverHz(int crossoverIndex, double freqHz) = 0;

    // Splits whichever band currently contains freqHz (addCrossover) or
    // merges the two bands a given split separates back into one
    // (removeCrossover) -- the two ways MultibandGraphWidget's "double-
    // click empty space in a band" / "double-click a split handle"
    // gestures change the band *count*, mirroring IEqEditorHost's
    // addBand()/removeBand().
    virtual void addCrossover(double freqHz) = 0;
    virtual void removeCrossover(int crossoverIndex) = 0;

    virtual void setBandThreshold(int index, double thresholdDb) = 0;
    virtual void setBandRatio(int index, double ratio) = 0;
    virtual void setBandRatioUp(int index, double ratioUp) = 0; // upward-compression ratio, see MbBand::ratioUp
    virtual void setBandAttack(int index, double attackMs) = 0;
    virtual void setBandRelease(int index, double releaseMs) = 0;
    virtual void setBandMakeup(int index, double makeupDb) = 0;
    virtual void setBandEnabled(int index, bool enabled) = 0;
    virtual void setBandSolo(int index, bool solo) = 0;
};

} // namespace ssgui
