// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// EqGraphWidget.hpp
//
// The main interactive surface of the EQ plugin -- a FabFilter Pro-Q-style
// frequency (x, log 20Hz-20kHz) vs. gain (y, adjustable +-range) graph:
//
//   - click empty space to add a new Bell band right there (frequency and
//     gain from the click position), which becomes immediately draggable
//     without releasing the mouse first;
//   - click-drag an existing node to move it (frequency + gain; a band
//     type with no gain of its own -- high/low pass, notch, band pass,
//     all pass, see EqModel.hpp's eqBandTypeHasGain() -- only moves
//     horizontally, its node stays pinned to the 0dB line rather than
//     showing a gain effect that wouldn't actually do anything);
//   - double-click a node, or select it (click, or Tab via the band list)
//     and press Delete/Backspace (handled one level up, in UIEq, since
//     keyboard events route to the top-level UI rather than subwidgets --
//     see UIEq::onKeyboard()), to remove that band;
//   - right-click a node for a context menu (change its type, delete/
//     duplicate, create/unlink a harmonic twin) *and*, for as long as the
//     button stays held down, momentarily preview just that band's own
//     isolated frequency region (host_->beginBandAudition()/
//     endBandAudition() -- the same Audition tap PluginEq's own kParamAudition
//     drives, see that file's own comment) -- a quick "what does this
//     spot sound like" check while you decide what to do from the menu,
//     distinct from the persistent Audition button in UIEq's own toolbar.
//     Previously a middle-click-and-hold gesture (uncomfortable, so
//     moved here) that toggled the per-band Solo
//     flag instead -- Audition is the more useful thing to preview with
//     a quick hold in the first place, since it isolates the actual
//     frequency region rather than just muting every *other* band;
//     EQ Mono has no Audition DSP of its own (see UIEqMono.cpp's own
//     no-op implementation of these two calls), so this is a no-op there
//     for now;
//   - scroll over a node to adjust its Q;
//
// Every band gets its own color (cycled from a fixed palette by index) and
// draws its own thin, low-alpha response curve alongside the bright
// combined curve -- both computed via EqModel.hpp's closed-form biquad
// math (eq::evaluateBandResponse/evaluateCombinedResponse), *not* the
// generic dense-matrix TransferFunction the base plugin's Bode plot uses,
// specifically so every one of these can be recomputed on every single
// drag event at no perceptible cost -- see EqModel.hpp's file comment for
// the benchmarked reason why.
//
// Live pre-EQ (Input) and/or post-EQ (Output) spectra are drawn behind the
// curves, fed by PluginEq's two SpectrumAnalyzer instances via UIEq (see
// setInputSpectrumData()/setOutputSpectrumData()) -- this widget only
// ever receives already-smoothed dB values, it has no notion of "how
// fast/smooth" itself (see UIEq::uiIdle()). Which one(s) are actually
// drawn is spectrumMode_ (ssfilter::SpectrumMode, see SpectrumAnalyzer.hpp)
// -- cycled via the toolbar's "Spectrum Source" control in UIEq, not
// anything in this widget.
//
// The y-axis' +-range is adjustable (click the "±Ndb" label, top-right)
// so a gentle pass can use a tight, more sensitive-to-drag range and an
// aggressive one a wide range that still fits.
//
// Band *type* has a discoverable UI here too (the right-click menu) but
// its primary home is BandListWidget, which also has exact numeric entry
// -- there's no room for that on a small draggable node.

#pragma once

#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "EqEditorHost.hpp"
#include "NanoVG.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

START_NAMESPACE_DGL

class EqGraphWidget : public NanoSubWidget
{
public:
    EqGraphWidget(Widget* parent, ssgui::IEqEditorHost* host);

    void setSampleRate(double sr) { sampleRate_ = sr > 0.0 ? sr : sampleRate_; }

    void setSelectedBand(int index) { selectedBand_ = index; }
    int getSelectedBand() const { return selectedBand_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }

    // Gain-axis +-range, dB. Cycled by clicking the on-graph label;
    // setGainRangeDb() also lets UIEq restore a persisted value.
    void setGainRangeDb(double rangeDb) { gainRangeDb_ = rangeDb > 0.1 ? rangeDb : gainRangeDb_; }
    double getGainRangeDb() const { return gainRangeDb_; }
    void setOnGainRangeChanged(std::function<void(double)> cb) { onGainRangeChanged_ = std::move(cb); }

    // Background spectrum: `db` must point to ssfilter::SpectrumAnalyzer::
    // kNumBands values, already smoothed by the caller (UIEq). Which
    // trace(s) actually get drawn is spectrumMode_ -- see this file's
    // header comment.
    void setSpectrumMode(ssfilter::SpectrumMode mode) { spectrumMode_ = mode; }
    ssfilter::SpectrumMode getSpectrumMode() const { return spectrumMode_; }
    void setInputSpectrumData(const float* db, int count);
    void setOutputSpectrumData(const float* db, int count);
    // The two per-channel Output analyzers -- kept current unconditionally
    // by UIEq (like the pair above), but only ever actually drawn (instead
    // of, not alongside, setOutputSpectrumData()'s combined trace) when
    // this frame's model has hasPerChannelBands() true -- see
    // onNanoDisplay()'s own comment.
    void setOutputSpectrumDataL(const float* db, int count);
    void setOutputSpectrumDataR(const float* db, int count);

    // Whether the spectrum's vertical range autoscales to whatever's on
    // screen each frame (smoothed, see .cpp) or sits at a fixed range --
    // off (fixed) by default, since a constantly-resettling axis under
    // real program material is visually disturbing even with the
    // smoothing added alongside this toggle; see spectrumToY()'s
    // own comment for why autoscaling exists at all. Click the on-graph
    // "Autoscale"/"Fixed Range" label to toggle; setSpectrumAutoscale()
    // additionally lets UIEq restore a persisted value.
    void setSpectrumAutoscale(bool on) { spectrumAutoscale_ = on; }
    bool getSpectrumAutoscale() const { return spectrumAutoscale_; }
    void setOnSpectrumAutoscaleChanged(std::function<void(bool)> cb) { onSpectrumAutoscaleChanged_ = std::move(cb); }

    // Called by UIEq once the "Create Harmonic Twin -> Custom Factor..."
    // text edit it owns (see EqEditorHost.hpp's beginHarmonicFactorEdit())
    // ends, whether committed or cancelled -- closes the context menu
    // this widget switched into text-input mode for. Public specifically
    // for that cross-object call; nothing else should need it.
    void endHarmonicFactorEdit() { customFactorEditing_ = false; closeContextMenu(); repaint(); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    float freqToX(double freqHz) const;
    double xToFreq(float px) const;
    float gainToY(double gainDb) const;
    double yToGain(float py) const;
    // The spectrum's OWN vertical mapping, deliberately independent of
    // gainRangeDb_ (the EQ curve's own zoom) -- the spectrum shows
    // *absolute* signal level (routinely -30 to -90dB per band once
    // energy is split across 64 of them), a completely different kind of
    // quantity than the EQ curve's *relative*-gain zoom. Two real
    // first-draft bugs, not design choices, happened here in sequence:
    // (1) originally mapped through the *same* clamped range as the EQ
    // curve, so most of a spectrum's real dynamic range (especially the
    // quiet low end, exactly where a bigger FFT's improved resolution
    // matters most) silently clamped to the same bottom pixel regardless
    // of its true value; (2) fixing that with a fixed-but-generous range
    // of its own technically stopped the clipping, but spreading real
    // 20-40dB differences (confirmed directly, see SpectrumAnalyzer's own
    // comment) across such a wide fixed span still made them look nearly
    // flat -- visible in principle, unconvincing in practice. Fixed
    // properly with **autoscaling**: every draw, the range is refit to
    // the *current* frame's own min/max (with headroom, rounded to a
    // clean 6dB step, see onNanoDisplay()) -- the same principle
    // `BodePlotWidget` already uses for its own magnitude axis. This
    // guarantees the full vertical space is always used regardless of
    // absolute level or FFT resolution, which is also *why* there's no
    // manual "how loud should this look" control: autoscaling refits to
    // whatever's there, so a constant additive boost before it would
    // shift the *range* by the same amount and produce an identical
    // picture -- offering a control with no visible effect would be
    // worse than not offering one.
    //
    // When both the input and output traces are showing (SpectrumMode::
    // Both), the range is fit to the *union* of both -- one shared axis
    // for a legitimate before/after comparison, rather than two
    // independently-scaled axes that would make "the gap between the two
    // curves" mean nothing (each curve could be autoscaled to look however
    // tall regardless of its neighbor's actual level).
    float spectrumToY(double db) const;
    static constexpr double kSpectrumFloorDb = -100.0, kSpectrumCeilDb = 6.0; // outer sanity clamp on the autoscaled range
    // The fixed range used whenever spectrumAutoscale_ is off (the
    // default) -- deliberately the same numbers the autoscale path already
    // fell back to for a degenerate (e.g. true-silence) frame, so "off"
    // reads as "whatever autoscale usually settles near anyway, just held
    // still" rather than an arbitrary different picture.
    static constexpr double kFixedSpectrumRangeLoDb = -66.0, kFixedSpectrumRangeHiDb = -6.0;
    double spectrumRangeLo_ = kFixedSpectrumRangeLoDb, spectrumRangeHi_ = kFixedSpectrumRangeHiDb; // smoothed toward target every draw when autoscaling, see onNanoDisplay()
    bool spectrumAutoscale_ = false;
    std::function<void(bool)> onSpectrumAutoscaleChanged_;
    float autoscaleLabelX_ = 0, autoscaleLabelY_ = 0, autoscaleLabelW_ = 0, autoscaleLabelH_ = 0;

    // Freq/gain/Q readout drawn above a node while it's being dragged
    // (which includes the moment it's first placed) -- see onNanoDisplay().
    static std::string formatFreqLabel(double freqHz);
    int nodeAt(double px, double py) const; // nearest band whose node is within hit radius, else -1
    void dragBandTo(int index, double px, double py);
    void openContextMenu(int band, float px, float py);
    void closeContextMenu() { menuOpen_ = false; menuBand_ = -1; pressedMenuItem_ = -1; }
    int menuItemAt(double px, double py) const; // -1 if outside the menu

    ssgui::IEqEditorHost* host_;
    double sampleRate_ = 48000.0;

    static constexpr double kFMin = 20.0, kFMax = 20000.0;
    static constexpr float kHitRadius = 11.0f;
    double gainRangeDb_ = 12.0; // +- this many dB fill the graph's height; see setGainRangeDb()
    std::function<void(double)> onGainRangeChanged_;

    // Click-and-drag-for-precision on the "±Ndb" label: press and drag
    // vertically to sweep the range continuously (multiplicative, so it
    // feels proportionally the same whether starting from a tight 3dB or
    // a wide 30dB), falling through to the original click-to-cycle-presets
    // behavior when the press never crosses the dead zone -- same
    // click-vs-drag split ParamStripWidget's own fields use.
    bool rangeLabelPressed_ = false;
    bool rangeLabelDragging_ = false;
    double rangeLabelDragStartY_ = 0.0;
    double rangeLabelDragStartValue_ = 0.0;
    static constexpr double kRangeDragDeadZonePx = 3.0;
    static constexpr double kRangeDragPxPerOctave = 150.0; // drag this many px to double/halve the range
    static constexpr double kRangeDragMinDb = 1.0, kRangeDragMaxDb = 40.0;

    float graphX_ = 0, graphY_ = 0, graphW_ = 0, graphH_ = 0;
    float rangeLabelX_ = 0, rangeLabelY_ = 0, rangeLabelW_ = 0, rangeLabelH_ = 0;

    int selectedBand_ = -1;
    int hoverBand_ = -1;
    int dragBand_ = -1;

    // Momentary right-click-hold audition -- see file comment. Restoring
    // whatever the *persistent* Audition state actually was on release is
    // UIEq's own job (endBandAudition()); this widget only needs to
    // remember which band to draw the "temporarily auditioned" ring on
    // and that a hold is in progress.
    int auditionHoldBand_ = -1;

    // Right-click click-vs-drag disambiguation: a right-press never
    // immediately opens the menu or starts auditioning -- it waits to see
    // whether the pointer crosses the dead zone before release. A plain
    // click (never crosses it) opens the context menu on release; a drag
    // (crosses it) commits to audition-while-dragging instead and the menu
    // never opens. Same dead-zone-on-motion pattern as rangeLabelPressed_
    // above, just keyed off distance instead of vertical delta only.
    bool rightPressPending_ = false;
    int rightPressBand_ = -1;
    double rightPressX_ = 0.0, rightPressY_ = 0.0;
    static constexpr double kRightDragDeadZonePx = 3.0;

    int lastClickBand_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};

    std::function<void(int)> onSelectionChanged_;

    // Right-click context menu -- see openContextMenu()/EqGraphWidget.cpp.
    // Built fresh into menuItems_ every time the menu opens (rather than
    // one fixed static list) so the Low/High Pass steepness rows can be
    // included only for a band that actually has a steepness to set.
    struct MenuEntry
    {
        std::string label;
        // HarmonicFactor: a one-click preset ratio (see harmonicFactor
        // below). HarmonicCustom: "Custom Factor...", switches the menu
        // into text-input mode (customFactorEditing_) instead of acting
        // immediately. Unlink: only ever added for an already-linked
        // band. See EqBand::harmonicGroup's own comment (EqModel.hpp) for
        // the underlying data model all three act on.
        enum class Kind { Delete, Duplicate, Type, Slope, HarmonicFactor, HarmonicCustom, Unlink } kind;
        ssfilter::EqBandType type = ssfilter::EqBandType::Bell;
        int slopeStages = 1; // meaningful for Kind::Slope only
        bool separatorAbove = false;
        // Trailing, defaulted -- appended after separatorAbove rather
        // than inserted earlier in the struct specifically so every
        // existing positional {label, kind, type, stages, separator}
        // MenuEntry initializer elsewhere in this file keeps meaning
        // exactly what it already did, unchanged.
        double harmonicFactor = 1.0; // meaningful for Kind::HarmonicFactor only
    };
    std::vector<MenuEntry> menuItems_;
    bool menuOpen_ = false;
    int menuBand_ = -1;
    float menuX_ = 0, menuY_ = 0, menuW_ = 0, menuH_ = 0;
    int pressedMenuItem_ = -1;

    // True from the moment "Custom Factor..." is clicked until the edit
    // it starts (via host_->beginHarmonicFactorEdit(), UIEq-owned) ends --
    // while true, the menu's own rows stop being interactive/drawn,
    // replaced by a live text readout of whatever UIEq's
    // isEditingHarmonicFactor() currently reports. See
    // endHarmonicFactorEdit() above for how this gets cleared again.
    bool customFactorEditing_ = false;

    ssfilter::SpectrumMode spectrumMode_ = ssfilter::kDefaultSpectrumMode;
    float spectrumInDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutLDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutRDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
};

END_NAMESPACE_DGL
