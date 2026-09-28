// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// LoudnessMeterWidget.hpp
//
// The Loudness Meter plugin's whole LUFS-specific display in one widget,
// the same "one widget owns the whole visualization" shape
// GoniometerWidget uses for the Phase Scope plugin: a vertical
// Momentary/Short-term bar meter on the left (with horizontal Short-term/
// Integrated indicator bars and a Target reference tick, all mapped to
// the same LUFS scale), a center panel that's either a scrolling history
// graph or a loudness Distribution histogram (see setShowDistribution()),
// and a numeric readout panel (Momentary/Short-term/Integrated/LRA/True
// Peak L,R,Max/PLR/PSR) on the right -- the same overall order and units
// Youlean Loudness Meter 2 uses. Plain per-channel dBFS L/R bars are a
// separate, sibling gui/LevelMeterWidget instance in UILoudnessMeter, not
// part of this widget -- the same widget ../plugin_phasescope's own UI
// already uses for that exact purpose.

#pragma once

#include "NanoVG.hpp"

#include <chrono>
#include <string>
#include <vector>

START_NAMESPACE_DGL

class LoudnessMeterWidget : public NanoSubWidget
{
public:
    explicit LoudnessMeterWidget(Widget* parent);

    // The nine live numeric readings -- called every time any one of them
    // changes; cheap enough (nine floats) to just always pass the whole
    // current set rather than nine separate setters.
    void setReadings(float momentaryLufs, float shortTermLufs, float integratedLufs, float loudnessRangeLu,
                      float truePeakL, float truePeakR, float truePeakMax, float plrLu, float psrLu);

    // targetLufs is meaningless when enabled is false (no reference line
    // drawn at all, the "Off" position of the UI's own Target cycle
    // button).
    void setTarget(float targetLufs, bool enabled);

    // Appends one point to the scrolling history graph's own fixed-size
    // ring buffer -- called by the owning UI at a steady ~10Hz cadence
    // (matching the DSP side's own 100ms gating-block period), not once
    // per parameterChanged() push, so the graph's time axis stays evenly
    // spaced regardless of whether a given reading happened to change.
    void pushHistorySample(float momentaryLufs, float shortTermLufs);

    // The loudness Distribution histogram's own bin counts (see
    // ssmeter::DistributionHistogram) -- plain counts, not normalized;
    // this widget normalizes both against one shared tallest-bin-of-
    // either at draw time. Two independent tallies, drawn as two curves
    // overlaid in one shared plot (see drawDistribution()'s own comment)
    // -- "do me two different distribution plots. one with the short term
    // and one with momentary values" first, then "you can make one big
    // plot, having both graphs overlapping".
    void setDistributionShortTerm(const float* counts, int count);
    void setDistributionMomentary(const float* counts, int count);

    // Toggles the center panel between the history graph and the
    // Distribution view -- see UILoudnessMeter's own "View" button.
    void setShowDistribution(bool show);

    // A brief visual confirmation that "Reset" was actually clicked --
    // intensity in [0, 1], driven by the owning UI's own uiIdle() as a
    // simple decay over a fraction of a second. See drawReadoutPanel()'s
    // own comment for why this exists: some of what Reset clears
    // (Integrated, Range, True Peak Max) can resettle to a value very
    // close to what it read before within a second or two on steady
    // material, which without this can look indistinguishable from the
    // button doing nothing at all.
    void setResetFlash(float intensity);

protected:
    void onNanoDisplay() override;
    // Dispatch to the Distribution or History view's own handling
    // depending on showDistribution_ -- see onScrollDistribution()/
    // onScrollHistory()/onMouseDistribution()/onMouseHistory()'s own
    // definitions. All four (plus onMotion(), History-only) return false
    // whenever the cursor is outside the center panel, or (onMotion())
    // no drag is in progress -- this widget has no other use for any of
    // these events.
    bool onScroll(const ScrollEvent& ev) override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    void drawBarMeter(float x, float y, float w, float h);
    void drawHistoryGraph(float x, float y, float w, float h);
    void drawDistribution(float x, float y, float w, float h);
    void drawReadoutPanel(float x, float y, float w, float h);
    // The center panel's own bounds (shared by onNanoDisplay(), which
    // draws into it, and onScroll()/onMouse()/onMotion(), which need to
    // hit-test against the exact same rectangle) -- factored out so the
    // panel layout constants live in exactly one place.
    void centerPanelBounds(float& x, float& y, float& w, float& h) const;
    void drawReadoutRow(float x, float y, float w, const char* label, const std::string& value, DGL_NAMESPACE::Color color);
    void strokeSpline(const float* xs, const float* ys, int count, DGL_NAMESPACE::Color color, bool fillUnderneath,
                       float baselineY, float fillAlpha);
    void strokeMonotoneSpline(const float* xs, const float* ys, int count, DGL_NAMESPACE::Color color,
                               bool fillUnderneath, float baselineY, float fillAlpha);

    bool onScrollDistribution(const ScrollEvent& ev);
    bool onMouseDistribution(const MouseEvent& ev);

    // History's own scroll-zoom -- while historyAnchored_, just resizes
    // historyZoomSpanSec_ (the right edge always stays pinned to "now");
    // once panned away (historyAnchored_ false), an ordinary cursor-
    // centered zoom over the frozen historyViewMinSec_/historyViewMaxSec_
    // window, same mechanic as onScrollDistribution()'s own.
    bool onScrollHistory(const ScrollEvent& ev);
    // Press: hit-tests the Live badge (toggles historyAnchored_ back on,
    // snapping to "now") and double-click (full reset: span back to the
    // default 60s, re-anchored) before falling through to starting a
    // potential drag -- see onMotion()'s own comment for why breaking
    // historyAnchored_ waits for actual movement, not just a press.
    bool onMouseHistory(const MouseEvent& ev);
    // The Live badge's own small clickable rect, in the panel's own
    // top-right corner -- shared by drawHistoryGraph() (draws it) and
    // onMouseHistory() (hit-tests it), same "exactly one place" reasoning
    // as centerPanelBounds().
    void historyLiveBadgeBounds(float& x, float& y, float& w, float& h) const;

    static constexpr float kBarMinLufs = -40.0f;
    static constexpr float kBarMaxLufs = 3.0f;
    // 6 hours at the ~10Hz push rate pushHistorySample() is called at --
    // enough to record an entire song and more (this data never has to
    // survive a plugin close/reopen, so
    // there's no session-file-size cost to a generous buffer). At 2
    // floats/sample that's ~1.7MB for the full 6 hours -- trivial, and
    // comfortably covers any single real working session, not just one
    // song. See historyMomentary_/historyShortTerm_'s own comment for why
    // this is a std::vector, not a fixed array, now that it's too large
    // to embed directly in the widget's own fixed layout comfortably.
    static constexpr int kHistoryCapacity = 216000;
    static constexpr int kMaxDistributionBins = 64; // ssmeter::DistributionHistogram::kBinCount (60) fits comfortably

    // The streaming-platform loudness figures (Spotify/Apple Music/
    // YouTube/SoundCloud) used to be drawn here as permanent reference
    // lines on the Distribution view -- moved to UILoudnessMeter's own
    // kTargetPresets instead (visible only when selected), so this widget
    // no longer needs to know about them at all: whichever one is
    // selected draws as the ordinary single Target line every view
    // already draws (targetLufs_/targetEnabled_ below).

    float momentaryLufs_ = -120.0f;
    float shortTermLufs_ = -120.0f;
    float integratedLufs_ = -120.0f;
    float loudnessRangeLu_ = 0.0f;
    float truePeakL_ = -120.0f;
    float truePeakR_ = -120.0f;
    float truePeakMax_ = -120.0f;
    float plrLu_ = 0.0f;
    float psrLu_ = 0.0f;

    float targetLufs_ = -23.0f;
    bool targetEnabled_ = true;
    bool showDistribution_ = false;
    float resetFlash_ = 0.0f;

    // Ring buffer for the scrolling history graph -- oldest-to-newest
    // read out starting at historyWritePos_ (the next slot to be
    // overwritten, i.e. the oldest remaining sample once the buffer has
    // wrapped at least once). std::vector, reserved to kHistoryCapacity
    // up front in the constructor and never reallocated after -- a fixed
    // C array of this size (~1.7MB combined) would bloat every single
    // LoudnessMeterWidget instance's own footprint even for a session
    // that closes the editor after a few seconds; a heap-backed vector
    // costs the same total memory only once it's actually needed to.
    std::vector<float> historyMomentary_;
    std::vector<float> historyShortTerm_;
    int historyWritePos_ = 0;
    int historyFilled_ = 0;

    // Reused scratch buffers for drawHistoryGraph()'s own decimated,
    // currently-visible point set -- see that function's own comment for
    // why this can no longer be a fixed-size stack array now that
    // kHistoryCapacity is large enough that a worst-case-sized one would
    // risk overflowing the call stack. Members (not locals) so the
    // underlying heap allocation is made once and reused frame to frame,
    // not torn down and rebuilt every single onNanoDisplay() call.
    std::vector<float> historyDrawXs_, historyDrawYsShortTerm_, historyDrawYsMomentary_;

    // Nominal seconds between pushHistorySample() calls -- matches
    // pushHistorySample()'s own "~10Hz cadence" declaration comment.
    // Samples are assumed evenly spaced at this period for x-axis
    // purposes (same implicit assumption the pre-zoom code already made,
    // just now given an explicit name and used for real time values
    // rather than a bare index-to-pixel ratio).
    static constexpr float kHistorySamplePeriodSec = 0.1f;

    // History's own x-axis state -- scroll zoom and an x-axis grid,
    // labeled as elapsed seconds since the first sample rather than DAW
    // timeline position or "seconds ago" (which keeps shifting while
    // panned away from the Live anchor) -- self-contained in this widget,
    // no host/DSP time-position plumbing needed, and stable under
    // panning/zooming since 0s always means the same fixed instant
    // instead of constantly redefining itself relative to "now".
    //
    // historyElapsedNewest_ is that fixed axis's own coordinate for the
    // single newest sample -- a plain monotonic counter, advanced by
    // kHistorySamplePeriodSec on every pushHistorySample() call, never
    // reset for the lifetime of this widget (i.e. resets only when the
    // plugin editor itself is reopened, since that's when a fresh
    // LoudnessMeterWidget -- and its history buffer -- is constructed).
    double historyElapsedNewest_ = 0.0;
    bool historyHasData_ = false;

    // historyZoomSpanSec_ is the current zoom level (how many seconds
    // wide the visible window is) -- shared between the anchored and
    // panned states, so toggling the anchor back on (snapping to "now")
    // resumes at whatever zoom the user had
    // rather than resetting it. Clamped to [kHistoryMinSpanSec,
    // kHistoryMaxSpanSec] by onScrollHistory().
    static constexpr float kHistoryMinSpanSec = 2.0f;
    static constexpr float kHistoryMaxSpanSec = float(kHistoryCapacity) * kHistorySamplePeriodSec; // 21600 (6h) -- the ring buffer's own full capacity; actually reachable in one zoom-out only once that much has genuinely been recorded, see drawHistoryGraph()'s own view-window clamp against historyElapsedNewest_
    // The default span a freshly-opened editor starts at -- deliberately
    // NOT kHistoryMaxSpanSec any more now that it's 6 hours: a session
    // that has only been running a few seconds/minutes should default to
    // showing roughly what it's actually recorded so far, not one giant
    // near-empty 6-hour window. Matches the pre-this-entry default (60s)
    // exactly, and onScrollHistory()/onMouseHistory()'s own double-click
    // reset both still clamp against kHistoryMaxSpanSec, so zooming
    // further out to see an actually-long session remains one scroll or
    // double-click away.
    static constexpr float kHistoryDefaultSpanSec = 60.0f;
    float historyZoomSpanSec_ = kHistoryDefaultSpanSec;

    // true = the window always ends at historyElapsedNewest_ ("now"),
    // recomputed fresh every draw from historyZoomSpanSec_ alone --
    // historyViewMinSec_/historyViewMaxSec_ below are meaningless while
    // this is true. false = a frozen [historyViewMinSec_,
    // historyViewMaxSec_] window, set the moment a drag first actually
    // moves (see onMotion()'s own comment for why a mere click doesn't
    // already break this) and left alone until the user re-anchors (the
    // Live badge, or a double-click's full reset).
    bool historyAnchored_ = true;
    float historyViewMinSec_ = 0.0f;
    float historyViewMaxSec_ = kHistoryDefaultSpanSec; // never actually read until the first real pan overwrites it (see onMouseHistory()); kept sane regardless

    // Drag-to-pan state -- see onMouseHistory()/onMotion()'s own
    // comments. The drag's own reference frame (screen x and view window
    // *at the moment the press happened*) is snapshotted once and every
    // subsequent onMotion() recomputes the pan from that fixed reference
    // rather than accumulating per-event deltas -- same
    // "recompute-from-press, don't accumulate" idiom
    // ContourGraphWidget::onMotion() (../../compressor/gui/
    // ContourGraphWidget.cpp) already uses for its own point-dragging.
    bool historyDragging_ = false;
    double historyDragStartX_ = 0.0;
    float historyDragStartMinSec_ = 0.0f;
    float historyDragStartMaxSec_ = 0.0f;

    // Double-click detection for onMouseHistory()'s full-reset -- same
    // idiom as lastDistClickTime_ below.
    std::chrono::steady_clock::time_point lastHistoryClickTime_{};

    // Same bin count for both -- both come from the same
    // ssmeter::LufsMeter::kDistributionBinCount on the DSP side.
    float distributionCountsShortTerm_[kMaxDistributionBins] = {};
    float distributionCountsMomentary_[kMaxDistributionBins] = {};
    int distributionBinCount_ = 0;

    // The Distribution view's own x-axis zoom window, in LUFS -- a pure
    // view transform over the same fixed -60..0 binned data
    // ssmeter::DistributionHistogram always collects (see its own
    // comment); zooming never changes what's collected, only what
    // fraction of it is currently drawn across the panel's width, zooming
    // around the mouse. Defaults to the full range (no zoom) -- see onScroll() for how
    // scrolling over the panel narrows/widens this window around the
    // cursor, and onMouse() for the double-click-to-reset back to these
    // defaults.
    float distViewMinLufs_ = -60.0f;
    float distViewMaxLufs_ = 0.0f;

    // Double-click detection for onMouse()'s zoom-reset -- same
    // "steady_clock delta under a fixed window" idiom
    // ContourGraphWidget::lastClickTime_ already uses for its own
    // double-click handling.
    std::chrono::steady_clock::time_point lastDistClickTime_{};
};

END_NAMESPACE_DGL
