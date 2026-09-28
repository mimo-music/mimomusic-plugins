// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// CompressorGraphWidget.hpp
//
// The combined, animated compressor graphic: "level, the reduction, knee,
// input-output mapping" all in one widget, redrawn continuously.
//
//   - Main area: the static input->output transfer curve (dB in, dB out),
//     computed directly from sscomp::computeGainReductionDb() (the exact
//     same pure function the real-time engine uses, see
//     plugin/dsp/CompressorDsp.hpp) -- a dashed unity (no compression)
//     reference line, the knee region shaded, the threshold marked, and a
//     live marker dot tracking the current instantaneous input level's
//     position on that curve.
//   - A vertical gain-reduction meter to the right of the graph -- fills
//     downward from 0dB as reduction increases, the conventional look for
//     a compressor's GR meter.
//   - A scrolling gain-reduction history strip along the bottom -- the
//     "animated" part beyond the live dot: a few seconds of reduction
//     depth over time, so an attack/release shape is actually visible, not
//     just inferable from a single twitching number.
//
// Like ParamStripWidget, this widget takes plain data via setter calls
// from the owning UI's uiIdle()/parameterChanged() rather than a shared
// host interface -- there's only ever one of these per plugin instance, so
// the heavier IModelEditorHost-style abstraction
// ../statespace uses to decouple several sibling
// widgets from each other would be pure overhead here.
//
// Directly draggable: a handle sitting on the curve at x=threshold (drag
// horizontally to change Threshold), and a second handle further up *or*
// down the curve (drag vertically to change Ratio) -- above the threshold
// while compressing Down (steeper slope, more downward compression), or
// below it while compressing Up (the mirrored upward-compression shape,
// see CompressorDsp.hpp's computeGainReductionDb() for the two-sided
// curve this can draw). Only one of those two positions is ever live at
// once, matching there being exactly one Ratio value and a direction
// switch (see ratioUpMode_'s own comment) rather than two independently-
// draggable ratios that could both be dialed in at the same time. Both
// handles use an
// absolute position->value mapping (the pointer's coordinate *is* the
// value, like dragging a node in ../statespace's
// EqGraphWidget), not a relative delta the way ParamStripWidget's fader
// drag works -- appropriate here because the handle's whole point is
// "grab the spot on the graph that represents the value you want."
// Scrolling anywhere over the curve nudges Knee, the one curve-shape
// parameter that has no natural handle of its own (it widens/narrows the
// already-visible shaded region symmetrically around the threshold rather
// than moving a point).
//
// The gain-reduction meter column(s) to the right are bipolar since an Up
// direction can report a *boost* (a positive number), not just a
// reduction: a 0dB reference line splits each column, filling downward
// (amber/red) for a cut and upward (blue/green) for a boost, with its own
// dB scale and a peak-hold marker that snaps to the most extreme recent
// excursion in *either* direction and slowly "recovers" back toward 0dB --
// see drawGrMeterColumn()'s own comment. Each column actually draws *two*
// overlaid bars: a dim background one at the raw, makeup-less gain
// reduction, and a full-strength foreground one at the net (gain
// reduction + Makeup Gain together) -- so what the compressor alone did
// and what actually reaches the output both stay visible at a glance.

#pragma once

#include "NanoVG.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

START_NAMESPACE_DGL

class CompressorGraphWidget : public NanoSubWidget
{
public:
    explicit CompressorGraphWidget(Widget* parent);

    // The static curve shape -- called whenever any of these parameters
    // change. `ratioUpMode`: false compresses above the threshold (Down,
    // the historical default), true compresses below it (Up) -- see
    // ratioUpMode_'s own comment for why this is a single ratio + a
    // direction rather than two independent ratio values.
    void setCurveParams(float thresholdDb, float ratio, bool ratioUpMode, float kneeDb);

    // The live, per-frame readout -- called every uiIdle() tick.
    // detectorLevelDb: the level actually feeding the gain computer right now.
    // gainReductionDb: the resulting reduction (<= 0) right now.
    void setLiveLevels(float detectorLevelDb, float gainReductionDb);

    // Makeup Gain made visible in the gain reduction, like the Multiband
    // Compressor's -- a distinctly-colored marker line at each
    // column's own *configured* Makeup Gain value (not a live reading),
    // ported from ../../statespace's MultibandGraphWidget::
    // drawBandGainMeter() own identical marker. bDb is B's own value,
    // already resolved by the caller to whichever is actually live right
    // now (its own bank's value while independent, A's while not) --
    // see UICompressorStereo::refreshMakeupMarkers().
    void setMakeupGainDb(float aDb, float bDb) { makeupGainDb_ = aDb; makeupGainDbB_ = bDb; }

    // A second, optional channel (Stereo Dual-Mono/Mid-Side) drawn as a
    // fainter second marker + second GR meter column. Pass show=false
    // (the default) for Linked mode / the mono plugin, where there's only
    // one meaningful detector.
    void setSecondaryLiveLevels(bool show, float detectorLevelDb, float gainReductionDb);
    void setSecondaryLabels(const char* a, const char* b) { labelA_ = a; labelB_ = b; }

    // Fired continuously while dragging the corresponding handle, and once
    // per notch while scrolling over the curve for Knee -- the owning UI
    // forwards these straight to setParameterValue(), same as any other
    // host-visible parameter change (see ParamStripWidget's setSetter()
    // for the identical pattern).
    void setOnThresholdChanged(std::function<void(float)> cb) { onThresholdChanged_ = std::move(cb); }
    void setOnRatioChanged(std::function<void(float)> cb) { onRatioChanged_ = std::move(cb); }
    void setOnKneeChanged(std::function<void(float)> cb) { onKneeChanged_ = std::move(cb); }
    void setThresholdRange(float minDb, float maxDb) { thresholdMinDb_ = minDb; thresholdMaxDb_ = maxDb; }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    enum class Handle { None, Threshold, Ratio };

    void drawTransferCurve(float x, float y, float w, float h);
    void drawGrMeterColumn(float x, float y, float w, float h, float grDb, float holdDb, float makeupDb, const char* label);
    void drawGrScale(float x, float y, float h);
    void drawGrHistory(float x, float y, float w, float h);
    void updateHold(float& holdDb, std::chrono::steady_clock::time_point& lastUpdate,
                     std::chrono::steady_clock::time_point& peakAt, float grDb);

    // Smooths a raw, potentially very-fast-arriving live reading against
    // wall-clock time before it's stored/drawn -- see .cpp's own comment
    // (the Multiband GR-meter readability fix from the sibling
    // ../statespace project, applied here identically: this graph's own
    // live GR number would otherwise flicker too fast to read).
    void smoothLiveValue(float& smoothed, std::chrono::steady_clock::time_point& lastUpdate, bool& init, float raw);

    float dbToPxX(float db) const;
    float dbToPxY(float db) const;
    float pxToDbX(float px) const;
    float pxToDbY(float py) const;
    // The Ratio handle's own reference x-position -- above the threshold
    // while compressing Down, below it while compressing Up (mirrored the
    // same distance the other direction). Only one of these two is ever
    // the *active* one (see activeRatioHandleRefDb()); the other is dead
    // code from the handle's own perspective now that there's no second,
    // simultaneously-draggable handle there any more.
    float ratioHandleRefDb() const { return std::min(kGraphMaxDb - 2.0f, thresholdDb_ + 24.0f); }
    float ratioUpHandleRefDb() const { return std::max(kGraphMinDb + 2.0f, thresholdDb_ - 24.0f); }
    float activeRatioHandleRefDb() const { return ratioUpMode_ ? ratioUpHandleRefDb() : ratioHandleRefDb(); }
    // ratio_ resolved to the (ratioDown, ratioUp) pair
    // sscomp::computeGainReductionDb() actually takes -- exactly one of
    // the two is ratio_, the other is always 1.0 (no effect on that side).
    float effectiveRatioDown() const { return ratioUpMode_ ? 1.0f : ratio_; }
    float effectiveRatioUp() const { return ratioUpMode_ ? ratio_ : 1.0f; }
    Handle hitHandle(float px, float py) const;

    float thresholdDb_ = -18.0f, ratio_ = 4.0f, kneeDb_ = 6.0f;
    // Which side of the threshold ratio_ compresses -- false=Down
    // (historical default), true=Up. A single value + a direction, not
    // two independently-adjustable ratios that could both be active at
    // once -- see this file's header comment.
    bool ratioUpMode_ = false;
    float thresholdMinDb_ = -60.0f, thresholdMaxDb_ = 0.0f;
    float detectorLevelDb_ = -120.0f, gainReductionDb_ = 0.0f;
    std::chrono::steady_clock::time_point grSmoothUpdated_{};
    bool grSmoothInit_ = false;
    bool secondaryShown_ = false;
    float detectorLevelDbB_ = -120.0f, gainReductionDbB_ = 0.0f;
    std::chrono::steady_clock::time_point grSmoothUpdatedB_{};
    bool grSmoothInitB_ = false;
    float makeupGainDb_ = 0.0f, makeupGainDbB_ = 0.0f;
    std::string labelA_ = "GR", labelB_ = "GR B";

    // Peak-hold marker per column: snaps immediately to a new, more
    // extreme excursion (in *either* direction -- a deep cut or a big
    // boost, whichever magnitude is larger) and otherwise decays its
    // magnitude back toward 0dB at a fixed rate, real-time-based (not
    // assumed-tick-based) via std::chrono, the same technique this
    // widget's clip-adjacent siblings (LevelMeterWidget's clip LED) use.
    float grHoldDb_[2] = { 0.0f, 0.0f };
    std::chrono::steady_clock::time_point grHoldUpdated_[2]{};
    // Wall-clock timestamp of whichever excursion is currently held --
    // reset every time updateHold() snaps to a fresh, more-extreme value.
    // Recovery is held off until kGrHoldTimeSec has elapsed since that
    // moment, so the displayed peak sits still long enough to actually be
    // read before it starts sliding back toward 0dB (see updateHold()'s
    // own comment).
    std::chrono::steady_clock::time_point grHoldPeakAt_[2]{};

    // Ring buffer of recent gain-reduction readings, pushed once per
    // setLiveLevels() call (i.e. once per uiIdle() tick).
    static constexpr int kHistoryLen = 240; // ~a few seconds at a typical ~60-90Hz idle rate
    float history_[kHistoryLen] = {};
    int historyPos_ = 0;

    static constexpr float kGraphMinDb = -60.0f;
    static constexpr float kGraphMaxDb = 6.0f;
    // Bipolar now that upward compression can report a boost: 0dB sits
    // between the two, not at the top -- see drawGrMeterColumn()'s comment.
    static constexpr float kGrMeterMinDb = -24.0f;
    static constexpr float kGrMeterMaxDb = 12.0f;

    // The curve area's own pixel rect, cached from the last draw so
    // onMouse/onMotion/onScroll can map pointer coordinates <-> dB with
    // the exact same mapping drawTransferCurve() just used -- the same
    // "hit regions recomputed every draw" pattern
    // ../statespace's BodePlotWidget uses for its
    // legend swatches.
    float curveX_ = 0, curveY_ = 0, curveW_ = 0, curveH_ = 0;

    Handle hoverHandle_ = Handle::None;
    Handle draggingHandle_ = Handle::None;
    static constexpr float kHandleHitRadiusPx = 10.0f;

    std::function<void(float)> onThresholdChanged_;
    std::function<void(float)> onRatioChanged_;
    std::function<void(float)> onKneeChanged_;
};

END_NAMESPACE_DGL
