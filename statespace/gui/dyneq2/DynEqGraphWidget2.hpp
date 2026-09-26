// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynEqGraphWidget2.hpp
//
// The main interactive surface of the dynamic EQ plugin -- EqGraphWidget's
// own frequency (x, log 20Hz-20kHz) vs. gain (y, adjustable +-range) graph
// and node-drag interaction, reused almost verbatim (there is no band
// *type* to right-click here -- every band is the same Regalia-Mitra bell
// shape, see DynamicEqModel2.hpp's file comment -- so that part of
// EqGraphWidget's interaction is simply absent, the same simplification
// MultibandGraphWidget already made for its own, differently-shaped,
// bands), PLUS the two things that make this a *dynamic* EQ graph rather
// than a static one, both drawn from ssgui::IDynEqEditorHost2:
//
//   - a shaded "dynamic range" region per band with dynamics enabled --
//     the area between that band's response curve at its resting gainDb
//     and the same curve at gainDb-rangeDb (both via
//     dyneq2::evaluateBandResponse(), just two different resolved gain
//     values), i.e. exactly the frequency-shaped envelope the band's own
//     gain can move within.
//   - the *live* combined curve, using ssgui::IDynEqEditorHost2::
//     getCurrentGainDb() per band (DynamicEqEngine2's real, currently-
//     running gain, read back across the DSP<->UI boundary via
//     PluginDynEq2's kParamCurrentGainBase parameters -- see that file's
//     comment) instead of each band's resting gainDb, redrawn every
//     uiIdle() tick regardless of whether anything was dragged, so it
//     visibly moves in response to program material. The resting/static
//     combined curve is drawn too, dimmer, as the shape the live one moves
//     around -- the same "thin per-band curves behind the bright combined
//     one" layering EqGraphWidget already uses, one level up.

#pragma once

#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "DynEqEditorHost2.hpp"
#include "NanoVG.hpp"

#include <chrono>
#include <functional>
#include <string>
#include <vector>

START_NAMESPACE_DGL

class DynEqGraphWidget2 : public NanoSubWidget
{
public:
    DynEqGraphWidget2(Widget* parent, ssgui::IDynEqEditorHost2* host);

    void setSampleRate(double sr) { sampleRate_ = sr > 0.0 ? sr : sampleRate_; }

    void setSelectedBand(int index) { selectedBand_ = index; }
    int getSelectedBand() const { return selectedBand_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }

    void setGainRangeDb(double rangeDb) { gainRangeDb_ = rangeDb > 0.1 ? rangeDb : gainRangeDb_; }
    double getGainRangeDb() const { return gainRangeDb_; }
    void setOnGainRangeChanged(std::function<void(double)> cb) { onGainRangeChanged_ = std::move(cb); }

    // Background spectrum: `db` must point to ssfilter::SpectrumAnalyzer::
    // kNumBands values, already smoothed by the caller (UIDynEq2) -- same
    // contract as EqGraphWidget's/MultibandGraphWidget's own.
    void setSpectrumMode(ssfilter::SpectrumMode mode) { spectrumMode_ = mode; }
    ssfilter::SpectrumMode getSpectrumMode() const { return spectrumMode_; }
    void setInputSpectrumData(const float* db, int count);
    void setOutputSpectrumData(const float* db, int count);
    // The two per-channel Output analyzers -- same idiom as the sibling
    // Parametric EQ's identical pair on EqGraphWidget: only actually drawn
    // (instead of the combined trace) once the current model's
    // hasPerChannelBands() is true, see onNanoDisplay()'s own comment.
    void setOutputSpectrumDataL(const float* db, int count);
    void setOutputSpectrumDataR(const float* db, int count);

    // See EqGraphWidget::setSpectrumAutoscale()'s identical comment -- off
    // (fixed range) by default.
    void setSpectrumAutoscale(bool on) { spectrumAutoscale_ = on; }
    bool getSpectrumAutoscale() const { return spectrumAutoscale_; }
    void setOnSpectrumAutoscaleChanged(std::function<void(bool)> cb) { onSpectrumAutoscaleChanged_ = std::move(cb); }

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
    float spectrumToY(double db) const; // identical mechanism to EqGraphWidget's own -- see that file's comment

    int nodeAt(double px, double py) const;
    // A Band Shelf's two corner handles -- same as EqGraphWidget's
    // identically-named function: the band whose handle is under the
    // pointer (else -1), `edge` 1 = low corner, 2 = high corner. Dragging
    // either sets the bandwidth symmetrically around the centre.
    int edgeHandleAt(double px, double py, int& edge) const;
    void dragBandTo(int index, double px, double py);

    ssgui::IDynEqEditorHost2* host_;
    double sampleRate_ = 48000.0;

    static constexpr double kFMin = 20.0, kFMax = 20000.0;
    static constexpr float kHitRadius = 11.0f;
    double gainRangeDb_ = 12.0;
    std::function<void(double)> onGainRangeChanged_;

    // Click-and-drag-for-precision on the "±Ndb" label -- see
    // EqGraphWidget.hpp's identical members' own comment.
    bool rangeLabelPressed_ = false;
    bool rangeLabelDragging_ = false;
    double rangeLabelDragStartY_ = 0.0;
    double rangeLabelDragStartValue_ = 0.0;
    static constexpr double kRangeDragDeadZonePx = 3.0;
    static constexpr double kRangeDragPxPerOctave = 150.0;
    static constexpr double kRangeDragMinDb = 1.0, kRangeDragMaxDb = 60.0; // matches the extended kGainRangePresets[]'s own new top (.cpp)

    float graphX_ = 0, graphY_ = 0, graphW_ = 0, graphH_ = 0;
    float rangeLabelX_ = 0, rangeLabelY_ = 0, rangeLabelW_ = 0, rangeLabelH_ = 0;

    // Crossfeed transit gain knobs -- ported verbatim from the sibling
    // Parametric EQ's own EqGraphWidget.hpp (CrossFeedGainControl there,
    // same struct/fields/interaction here), so it behaves exactly like the
    // Parametric EQ's, -60dB line included. One knob per direction, shown (and interactive)
    // only while some band in the model actually routes that way,
    // stacked below the "Autoscale"/"Fixed Range" label. Click-and-drag
    // for a continuous, fader-like sweep, plus scroll-to-nudge -- see
    // EqGraphWidget.hpp's identical struct comment for the full
    // reasoning behind both.
    struct CrossFeedGainControl
    {
        float x = 0, y = 0, w = 0, h = 0;
        bool visible = false;
        bool pressed = false;
        bool dragging = false;
        double dragStartY = 0.0;
        double dragStartValue = 0.0;
    };
    CrossFeedGainControl xfeedLtoR_, xfeedRtoL_;
    static constexpr double kXfeedDragDeadZonePx = 3.0;
    static constexpr double kXfeedDragPxPerDb = 3.0;
    static constexpr double kXfeedGainMinDb = -60.0, kXfeedGainMaxDb = 12.0;

    int selectedBand_ = -1;
    int hoverBand_ = -1;
    int dragBand_ = -1;
    int dragEdge_ = 0; // 0 = dragging the node itself, 1/2 = a Band Shelf corner handle, see edgeHandleAt()

    int lastClickBand_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};

    std::function<void(int)> onSelectionChanged_;

    ssfilter::SpectrumMode spectrumMode_ = ssfilter::kDefaultSpectrumMode;
    float spectrumInDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutLDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutRDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    static constexpr double kSpectrumFloorDb = -100.0, kSpectrumCeilDb = 6.0;
    static constexpr double kFixedSpectrumRangeLoDb = -66.0, kFixedSpectrumRangeHiDb = -6.0;
    double spectrumRangeLo_ = kFixedSpectrumRangeLoDb, spectrumRangeHi_ = kFixedSpectrumRangeHiDb;
    bool spectrumAutoscale_ = false;
    std::function<void(bool)> onSpectrumAutoscaleChanged_;
    float autoscaleLabelX_ = 0, autoscaleLabelY_ = 0, autoscaleLabelW_ = 0, autoscaleLabelH_ = 0;

    // Freq/gain/Q readout drawn above a node while it's being dragged --
    // see EqGraphWidget::formatFreqLabel()'s identical helper.
    static std::string formatFreqLabel(double freqHz);

    // Right-click context menu -- ported from the sibling Parametric EQ's
    // own EqGraphWidget.cpp. No Slope section (this plugin has no
    // Low/High Pass types, hence no steepness to set) and no right-click-
    // hold-to-audition gesture (this UI's own persistent Audition toolbar
    // button already covers that, so right-click here is a plain click,
    // opened immediately on press -- no click-vs-drag disambiguation
    // needed the way the sibling's own momentary-hold gesture requires).
    void openContextMenu(int band, float px, float py);
    void closeContextMenu() { menuOpen_ = false; menuBand_ = -1; pressedMenuItem_ = -1; }
    int menuItemAt(double px, double py) const;
public:
    // Called by UIDynEq2 once its own "Create Harmonic Twin -> Custom
    // Factor..." text edit ends, whether committed or cancelled --
    // closes the context menu this widget switched into text-input mode
    // for. Public specifically for that cross-object call, same shape as
    // the sibling EqGraphWidget::endHarmonicFactorEdit().
    void endHarmonicFactorEdit() { customFactorEditing_ = false; closeContextMenu(); repaint(); }
private:

    struct MenuEntry
    {
        std::string label;
        // HarmonicFactor: a one-click preset ratio. HarmonicCustom:
        // "Custom Factor...", switches the menu into text-input mode
        // (customFactorEditing_) instead of acting immediately. Unlink:
        // only ever added for an already-linked band. Type: a direct
        // pick from the shapes this plugin's own type-cycle button also
        // reaches (DynamicEqBand2::type's own comment).
        enum class Kind { Delete, Duplicate, Type, HarmonicFactor, HarmonicCustom, Unlink } kind;
        ssfilter::EqBandType type = ssfilter::EqBandType::Bell; // meaningful for Kind::Type only
        bool separatorAbove = false;
        double harmonicFactor = 1.0; // meaningful for Kind::HarmonicFactor only
    };
    std::vector<MenuEntry> menuItems_;
    bool menuOpen_ = false;
    int menuBand_ = -1;
    float menuX_ = 0, menuY_ = 0, menuW_ = 0, menuH_ = 0;
    int pressedMenuItem_ = -1;
    static constexpr float kMenuRowH = 19.0f;
    static constexpr float kMenuW = 150.0f;

    // True from the moment "Custom Factor..." is clicked until the edit
    // it starts (via host_->beginHarmonicFactorEdit(), UIDynEq2-owned)
    // ends -- while true, the menu's own rows stop being interactive/
    // drawn, replaced by a live text readout of whatever UIDynEq2's own
    // isEditingHarmonicFactor() currently reports. See
    // endHarmonicFactorEdit() above for how this gets cleared again.
    bool customFactorEditing_ = false;
};

END_NAMESPACE_DGL
