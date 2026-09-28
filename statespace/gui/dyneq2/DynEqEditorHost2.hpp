// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynEqEditorHost2.hpp
//
// Same purpose and shape as gui/eq/EqEditorHost.hpp: this decouples
// DynEqGraphWidget2 and DynEqBandListWidget2 from UIDynEq2, which owns the
// actual ssfilter::DynamicEqModel2 and pushes it to the DSP on every edit.

#pragma once

#include "../../plugin/dsp/DynamicEqModel2.hpp"

#include <string>

namespace ssgui {

// Which numeric field of a band the band list is editing -- the dynamic
// counterpart of EqEditorHost.hpp's EqField.
enum class DynEqField { Freq, Bandwidth, Gain, Q, Threshold, Ratio, RatioUp, Range, Attack, Release };

class IDynEqEditorHost2
{
public:
    virtual ~IDynEqEditorHost2() = default;

    virtual int getBandCount() const = 0;
    virtual ssfilter::DynamicEqBand2 getBand(int index) const = 0;

    // The whole model, read-only -- used for the combined-response curve
    // (ssfilter::dyneq2::evaluateCombinedResponse() needs the full band list
    // at once, same reasoning as IEqEditorHost::getModel()) and for
    // Load/Save/Export.
    virtual ssfilter::DynamicEqModel2 getModel() const = 0;

    // The gain (dB) band `index` is *actually* running at right now, read
    // back live from DynamicEqEngine2 via a DPF output Parameter (see
    // PluginDynEq2.hpp's kParamCurrentGainBase comment) -- what
    // DynEqGraphWidget2 draws as the moving curve, distinct from
    // getBand(index).gainDb (the resting/static one).
    virtual double getCurrentGainDb(int index) const = 0;

    // Appends a new band at (freqHz, gainDb) with default Q/dynamics
    // settings, and removes one -- the two ways DynEqGraphWidget2's "click
    // empty space" / "double-click a node" gestures change the band
    // *count*, mirroring IEqEditorHost's addBand()/removeBand().
    virtual void addBand(double freqHz, double gainDb) = 0;
    virtual void removeBand(int index) = 0;
    // Inserts an exact copy of `index` right after it and selects the new
    // copy -- reachable from the graph's own right-click menu ("Duplicate
    // Band", same as the sibling EQ's own IEqEditorHost::duplicateBand()).
    virtual void duplicateBand(int index) = 0;

    virtual void setBandFreq(int index, double freqHz) = 0;
    virtual void setBandGain(int index, double gainDb) = 0;
    virtual void setBandQ(int index, double q) = 0;
    // Band Shelf only: its width in octaves (DynamicEqBand2::bandwidthOct,
    // clamped to kMin/kMaxBandwidthOct); a no-op for any other type.
    virtual void setBandBandwidth(int index, double octaves) = 0;
    virtual void setBandThreshold(int index, double thresholdDb) = 0;
    virtual void setBandRatio(int index, double ratio) = 0;
    virtual void setBandRatioUp(int index, double ratioUp) = 0; // upward-compression ratio, see DynamicEqBand2::ratioUp
    virtual void setBandRange(int index, double rangeDb) = 0;
    virtual void setBandAttack(int index, double attackMs) = 0;
    virtual void setBandRelease(int index, double releaseMs) = 0;
    virtual void setBandDynamicsEnabled(int index, bool enabled) = 0;
    virtual void setBandEnabled(int index, bool enabled) = 0;
    virtual void setBandSolo(int index, bool solo) = 0;

    // Which stereo channel(s) this band shapes/detects -- see
    // DynamicEqBand2::channel's own comment. No mono counterpart to gate
    // this behind (unlike the Parametric EQ's IEqEditorHost::
    // setBandChannel()): this is the only UI implementing this interface.
    virtual void setBandChannel(int index, ssfilter::EqChannel channel) = 0;

    // Band shape. Only Bell/LowShelf/HighShelf/BandShelf are ever set here
    // (the band list's TYPE column, and the graph's right-click menu) -- see
    // DynamicEqBand2::type's own comment for why the rest of
    // ssfilter::EqBandType isn't reachable from this UI at all.
    virtual void setBandType(int index, ssfilter::EqBandType type) = 0;

    // Stereo crossfeed -- see DynamicEqBand2::crossFeed's own comment. No
    // supportsCrossFeed()/default-no-op-body pair the way IEqEditorHost's
    // identical pair has: that exists there specifically so OTHER
    // IEqEditorHost implementers (UIEqMono, the reverb plugin's own
    // embedded panel) keep compiling/behaving unchanged without opting in
    // -- there is only ever one implementer of THIS interface, so a plain
    // pure virtual is enough, same as setBandChannel() just above.
    virtual void setBandCrossFeed(int index, ssfilter::EqCrossFeed crossFeed) = 0;

    // The crossfeed transit's own overall/static send level -- ONE knob
    // per direction, for the WHOLE L->R (or R->L) signal path, not per
    // band -- see DynamicEqModel2::crossFeedGainLtoRDb's own comment.
    // Read back via getModel().crossFeedGainLtoRDb/crossFeedGainRtoLDb --
    // no separate getter needed, same convention as the sibling EQ's own
    // IEqEditorHost::setCrossFeedGainLtoRDb()/RtoLDb().
    virtual void setCrossFeedGainLtoRDb(double gainDb) = 0;
    virtual void setCrossFeedGainRtoLDb(double gainDb) = 0;

    // Harmonic linking, the same way the Parametric EQ does it -- this
    // matches the sibling IEqEditorHost's own
    // createHarmonicTwin()/unlinkHarmonic()/beginHarmonicFactorEdit()/
    // isEditingHarmonicFactor()/commitHarmonicFactorEdit() exactly, all
    // reachable from DynEqGraphWidget2's right-click menu.
    virtual void createHarmonicTwin(int index, double factor) = 0;
    virtual void unlinkHarmonic(int index) = 0;

    // The "Create Harmonic Twin -> Custom Factor..." text edit -- same
    // shape as IEqEditorHost's identical trio (EqEditorHost.hpp), keyed
    // by the *source* band index rather than (band, field), since
    // nothing is created until the typed factor commits. UIDynEq2 owns
    // the actual edit state (keyboard events route to the top-level UI);
    // commitHarmonicFactorEdit() is what DynEqGraphWidget2's own "click
    // anywhere while this input is showing" handler calls to end it.
    virtual void beginHarmonicFactorEdit(int sourceIndex) = 0;
    virtual bool isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const = 0;
    virtual void commitHarmonicFactorEdit() = 0;

    // Momentary audition of `index` while the graph's right-button drag is
    // held -- same contract as IEqEditorHost::beginBandAudition()/
    // endBandAudition(): the end call restores the persistent Audition
    // state from before the hold.
    virtual void beginBandAudition(int index) = 0;
    virtual void endBandAudition() = 0;

    // The band list's text edit, keyed by (band, field) -- same shape as
    // IEqEditorHost::beginBandTextEdit()/isEditingBandField(). UIDynEq2
    // owns the edit state (keyboard events go to the top-level UI).
    virtual void beginBandTextEdit(int index, DynEqField field, bool clearValue = false) = 0;
    virtual bool isEditingBandField(int index, DynEqField field, std::string& liveText) const = 0;
};

} // namespace ssgui
