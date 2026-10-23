// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// MultibandGraphWidget.hpp
//
// The main interactive surface of the multiband compressor plugin -- a
// log-frequency (20Hz-20kHz) graph showing the crossover's own band
// regions (each shaded in its own color, cycled from the same palette
// EqGraphWidget uses) with a live pre-/post-compression spectrum drawn
// behind them, exactly like the sibling EQ plugin's EqGraphWidget:
//
//   - click a band's own shaded region to select it (drives the per-band
//     ParamStripWidget in UIMbCompressor, same "click to select, edit
//     elsewhere" split EqGraphWidget/BandListWidget use);
//   - drag a crossover split line horizontally to move it -- an absolute
//     position->frequency mapping, like dragging an EqGraphWidget node,
//     clamped so it can never cross past its neighbors (see
//     MbCompressorModel.hpp's setCrossoverHz());
//   - double-click inside a band's region (not on a handle) to split it
//     there, adding a new crossover point -- the "click empty space to
//     add a band" gesture, adapted (double-click, since a single click
//     here already means "select");
//   - double-click a crossover handle to remove that split, merging the
//     two bands it separated back into one;
//   - scroll over a band's region to adjust its Ratio multiplicatively --
//     there's no natural on-graph handle for Ratio the way EqGraphWidget's
//     nodes have Q, so this mirrors the sibling ../compressor project's
//     CompressorGraphWidget scrolling-over-the-curve-adjusts-Knee
//     convention instead.
//
// The background spectrum (pre-/post-compression, autoscaled every frame,
// its own right-edge scale) is EqGraphWidget's own mechanism verbatim --
// same autoscale-to-the-union-of-visible-traces logic, same "why
// autoscaling and not a fixed range or a manual gain control" reasoning
// (see that file's own spectrumToY() comment) -- fed by UIMbCompressor
// from PluginMbCompressor's two SpectrumAnalyzer instances exactly as
// UIEq feeds EqGraphWidget.
//
// Simpler than EqGraphWidget: no right-click context menu (there is no
// band *type* to pick -- every band is the same kind of compressor). Every
// band's live gain/makeup is shown by drawBandGainMeter(), the selected
// band's meter outlined in its color.

#pragma once

#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "MbEditorHost.hpp"
#include "../EditorSubWidget.hpp"

#include <chrono>
#include <functional>

START_NAMESPACE_DGL

class MultibandGraphWidget : public EditorSubWidget
{
public:
    MultibandGraphWidget(NanoTopLevelWidget* parent, ssgui::IMbEditorHost* host);

    void setSampleRate(double sr) { sampleRate_ = sr > 0.0 ? sr : sampleRate_; }

    void setSelectedBand(int index) { selectedBand_ = index; }
    int getSelectedBand() const { return selectedBand_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }

    // Background spectrum: `db` must point to
    // ssfilter::SpectrumAnalyzer::kNumBands values, already smoothed by
    // the caller (UIMbCompressor) -- same contract as EqGraphWidget's own.
    void setSpectrumMode(ssfilter::SpectrumMode mode) { spectrumMode_ = mode; }
    ssfilter::SpectrumMode getSpectrumMode() const { return spectrumMode_; }
    void setInputSpectrumData(const float* db, int count);
    void setOutputSpectrumData(const float* db, int count);

    // Live per-band gain (dB) readback from StateSpaceEngine::
    // getBandGainDb(), arriving via a DPF output Parameter (see
    // PluginMbCompressor.hpp's kParamBandGainBase comment) -- potentially
    // once per audio block, i.e. far faster than the eye can read a
    // changing number. Drives each band's bipolar gain-reduction/makeup
    // meter (see onNanoDisplay()); smooths
    // the raw incoming value in time (not just redraw-rate-limited) before
    // storing it, see the .cpp definition's own comment.
    void setBandGainDb(int index, float db);

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
    float spectrumToY(double db) const; // identical mechanism to EqGraphWidget's own -- see that file's comment

    int handleAt(double px, double py) const;   // nearest crossover split line within hit distance, else -1
    int bandRegionAt(double px) const;           // which band's region an x position falls in
    int meterAt(double px, double py) const;     // which band's gain meter is under a point, else -1
    int bandAt(double px, double py) const;      // meterAt(), else bandRegionAt()
    void layoutBandGainMeters(int numBands);
    void drawBandGainMeter(float x, float y, float w, float h, int band, bool selected);

    ssgui::IMbEditorHost* host_;
    double sampleRate_ = 48000.0;

    static constexpr double kFMin = 20.0, kFMax = 20000.0;
    static constexpr float kHandleHitPx = 8.0f;

    float graphX_ = 0, graphY_ = 0, graphW_ = 0, graphH_ = 0;

    int selectedBand_ = 0;
    int hoverHandle_ = -1;
    int dragHandle_ = -1;

    int lastClickHandle_ = -1;
    int lastClickBand_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};

    std::function<void(int)> onSelectionChanged_;

    ssfilter::SpectrumMode spectrumMode_ = ssfilter::kDefaultSpectrumMode;
    float spectrumInDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    float spectrumOutDb_[ssfilter::SpectrumAnalyzer::kNumBands] = {};
    static constexpr double kSpectrumFloorDb = -100.0, kSpectrumCeilDb = 6.0;
    static constexpr double kFixedSpectrumRangeLoDb = -66.0, kFixedSpectrumRangeHiDb = -6.0;
    double spectrumRangeLo_ = kFixedSpectrumRangeLoDb, spectrumRangeHi_ = kFixedSpectrumRangeHiDb;
    bool spectrumAutoscale_ = false;
    std::function<void(bool)> onSpectrumAutoscaleChanged_;
    float autoscaleLabelX_ = 0, autoscaleLabelY_ = 0, autoscaleLabelW_ = 0, autoscaleLabelH_ = 0;

    // Live per-band gain readback -- see setBandGainDb()'s own comment.
    // bandGainDb_ is the smoothed value actually drawn; bandGainSmoothUpdated_
    // is real-time-based (std::chrono), same pattern as ../compressor's own
    // CompressorGraphWidget::updateHold(), so the smoothing time constant
    // means what it says regardless of how often setBandGainDb() is called.
    float bandGainDb_[ssfilter::kMaxBands] = {};
    std::chrono::steady_clock::time_point bandGainSmoothUpdated_[ssfilter::kMaxBands]{};
    bool bandGainSmoothInit_[ssfilter::kMaxBands] = {}; // false until the first sample, see setBandGainDb()
    static constexpr double kGrMeterMinDb = -24.0, kGrMeterMaxDb = 12.0; // bipolar, same convention as ../compressor's own GR meter

    // Each band's meter rectangle, from layoutBandGainMeters() -- a narrow
    // band's meter may sit partly over its neighbor, so clicks and scrolls
    // on a meter go to the meter's band, see bandAt().
    float meterX_[ssfilter::kMaxBands] = {};
    float meterY_ = 0, meterW_ = 0, meterH_ = 0;
    int meterCount_ = 0;
};

END_NAMESPACE_DGL
