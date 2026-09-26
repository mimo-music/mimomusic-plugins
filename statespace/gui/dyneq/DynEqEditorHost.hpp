// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynEqEditorHost.hpp
//
// Same purpose and shape as gui/eq/EqEditorHost.hpp / gui/mb/MbEditorHost.hpp,
// one level up: this decouples DynEqGraphWidget (and the per-band
// ParamStripWidget in UIDynEq) from UIDynEq, which owns the actual
// ssfilter::DynamicEqModel, reassembles+pushes it to the DSP on every edit.

#pragma once

#include "../../plugin/dsp/DynamicEqModel.hpp"

namespace ssgui {

class IDynEqEditorHost
{
public:
    virtual ~IDynEqEditorHost() = default;

    virtual int getBandCount() const = 0;
    virtual ssfilter::DynamicEqBand getBand(int index) const = 0;

    // The whole model, read-only -- used for the combined-response curve
    // (ssfilter::dyneq::evaluateCombinedResponse() needs the full band list
    // at once, same reasoning as IEqEditorHost::getModel()) and for
    // Load/Save/Export.
    virtual ssfilter::DynamicEqModel getModel() const = 0;

    // The gain (dB) band `index` is *actually* running at right now, read
    // back live from DynamicEqEngine via a DPF output Parameter (see
    // PluginDynEq.hpp's kParamCurrentGainBase comment) -- what
    // DynEqGraphWidget draws as the moving curve, distinct from
    // getBand(index).gainDb (the resting/static one).
    virtual double getCurrentGainDb(int index) const = 0;

    // Appends a new band at (freqHz, gainDb) with default Q/dynamics
    // settings, and removes one -- the two ways DynEqGraphWidget's "click
    // empty space" / "double-click a node" gestures change the band
    // *count*, mirroring IEqEditorHost's addBand()/removeBand().
    virtual void addBand(double freqHz, double gainDb) = 0;
    virtual void removeBand(int index) = 0;

    virtual void setBandFreq(int index, double freqHz) = 0;
    virtual void setBandGain(int index, double gainDb) = 0;
    virtual void setBandQ(int index, double q) = 0;
    virtual void setBandThreshold(int index, double thresholdDb) = 0;
    virtual void setBandRatio(int index, double ratio) = 0;
    virtual void setBandRatioUp(int index, double ratioUp) = 0; // upward-compression ratio, see DynamicEqBand::ratioUp
    virtual void setBandRange(int index, double rangeDb) = 0;
    virtual void setBandAttack(int index, double attackMs) = 0;
    virtual void setBandRelease(int index, double releaseMs) = 0;
    virtual void setBandDynamicsEnabled(int index, bool enabled) = 0;
    virtual void setBandEnabled(int index, bool enabled) = 0;
    virtual void setBandSolo(int index, bool solo) = 0;

    // Which stereo channel(s) this band shapes/detects -- see
    // DynamicEqBand::channel's own comment. No mono counterpart to gate
    // this behind (unlike the Parametric EQ's IEqEditorHost::
    // setBandChannel()): this is the only UI implementing this interface.
    virtual void setBandChannel(int index, ssfilter::EqChannel channel) = 0;
};

} // namespace ssgui
