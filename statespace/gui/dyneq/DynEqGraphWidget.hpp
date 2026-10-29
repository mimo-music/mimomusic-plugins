// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynEqGraphWidget.hpp
//
// The main interactive surface of the dynamic EQ plugin -- EqGraphWidget's
// own frequency (x, log 20Hz-20kHz) vs. gain (y, adjustable +-range) graph
// and node-drag interaction, reused almost verbatim (there is no band
// *type* to right-click here -- every band is the same Regalia-Mitra bell
// shape, see DynamicEqModel.hpp's file comment -- so that part of
// EqGraphWidget's interaction is simply absent, the same simplification
// MultibandGraphWidget already made for its own, differently-shaped,
// bands), PLUS the two things that make this a *dynamic* EQ graph rather
// than a static one, both drawn from ssgui::IDynEqEditorHost:
//
//   - a shaded "dynamic range" region per band with dynamics enabled --
//     the area between that band's response curve at its resting gainDb
//     and the same curve at gainDb-rangeDb (both via
//     dyneq::evaluateBandResponse(), just two different resolved gain
//     values), i.e. exactly the frequency-shaped envelope the band's own
//     gain can move within.
//   - the *live* combined curve, using ssgui::IDynEqEditorHost::
//     getCurrentGainDb() per band (DynamicEqEngine's real, currently-
//     running gain, read back across the DSP<->UI boundary via
//     PluginDynEq's kParamCurrentGainBase parameters -- see that file's
//     comment) instead of each band's resting gainDb, redrawn every
//     uiIdle() tick regardless of whether anything was dragged, so it
//     visibly moves in response to program material. The resting/static
//     combined curve is drawn too, dimmer, as the shape the live one moves
//     around -- the same "thin per-band curves behind the bright combined
//     one" layering EqGraphWidget already uses, one level up.

#pragma once

#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "DynEqEditorHost.hpp"
#include "../CursorReadout.hpp"
#include "../EditorSubWidget.hpp"

#include <chrono>
#include <functional>
#include <string>

START_NAMESPACE_DGL

class DynEqGraphWidget : public EditorSubWidget
{
public:
    DynEqGraphWidget(NanoTopLevelWidget* parent, ssgui::IDynEqEditorHost* host);

    void setSampleRate(double sr) { sampleRate_ = sr > 0.0 ? sr : sampleRate_; }

    void setSelectedBand(int index) { selectedBand_ = index; }
    int getSelectedBand() const { return selectedBand_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }

    void setGainRangeDb(double rangeDb) { gainRangeDb_ = rangeDb > 0.1 ? rangeDb : gainRangeDb_; }
    double getGainRangeDb() const { return gainRangeDb_; }
    void setOnGainRangeChanged(std::function<void(double)> cb) { onGainRangeChanged_ = std::move(cb); }

    // Background spectrum: `db` must point to ssfilter::SpectrumAnalyzer::
    // kNumBands values, already smoothed by the caller (UIDynEq) -- same
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
    void dragBandTo(int index, double px, double py);

    ssgui::IDynEqEditorHost* host_;
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
    static constexpr double kRangeDragMinDb = 1.0, kRangeDragMaxDb = 40.0;

    float graphX_ = 0, graphY_ = 0, graphW_ = 0, graphH_ = 0;
    float rangeLabelX_ = 0, rangeLabelY_ = 0, rangeLabelW_ = 0, rangeLabelH_ = 0;

    int selectedBand_ = -1;
    int hoverBand_ = -1;
    // Pointer frequency/gain readout over an empty spot of the graph, see
    // CursorReadout.hpp; cursorReadoutShown() decides when it is drawn.
    CursorReadout cursor_;
    bool cursorReadoutShown() const;
    int dragBand_ = -1;

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
};

END_NAMESPACE_DGL
