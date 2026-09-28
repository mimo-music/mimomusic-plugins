// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// ContourGraphWidget.hpp
//
// The Contour plugin's own graph -- a sibling to CompressorGraphWidget
// (same GR meter columns/history, copied nearly verbatim -- see that
// file's own header comment for what those pieces do), but the main area
// is a directly-editable curve instead of a fixed ratio/knee shape:
//
//   - Tool modes (a button cycling place node / place rising node / place
//     falling node / edit): the plain gestures
//     collided (placing a node next to an existing one grabbed that one
//     instead; dragging a window vs. shaping a smooth point), so what a
//     left-drag does now depends on the active EditTool, chosen by the
//     owning UI's tool button:
//       - DrawNode / DrawRising / DrawFalling: a press on empty plot space
//         places a NEW node; a press ON an existing node instead grabs just
//         that node -- drag moves it, double-click removes it. Multi-select, group moves
//         and the marquee stay Edit-only. Click = corner node; click-and-drag
//         = smooth node, the drag pulling out its Bezier support points
//         (Inkscape pen tool). DrawRising/DrawFalling tag the new node
//         RisingOnly/FallingOnly (hysteresis). A ghost node + its dB
//         coordinates follow the cursor. The threshold stays reachable
//         through its own round handle at the bottom of its line.
//       - Edit: nothing is created. Drag a node to move it, drag a node's
//         support point (the dots at the ends of its tangent line) to
//         reshape the Bezier, drag on empty space to marquee-select,
//         Shift+click extends the
//         selection, drag any selected node to move the whole group,
//         double-click a node to remove it, right-click a node for a menu
//         (Normal / Rising / Falling / Link / Unlink / Snap to Diagonal /
//         Delete; right-drag pulls a handle out of a corner node).
//     The numeric node list (NodeListWidget) edits the same curve.
//     (The gesture bullets further down were written before the tool modes
//     existed: read "plain click/drag on empty space" as the Draw tools,
//     and everything about existing nodes -- dragging, right-click,
//     Shift-select, the marquee, double-click-to-remove -- as the Edit
//     tool. The marquee no longer needs Shift there: a plain drag on empty
//     space in Edit selects.)
//   - The curve itself (sscomp::TransferCurve): drawn as the actual
//     evaluated shape, with every control point drawn as a draggable
//     handle. The two endpoints (fixed X, see TransferCurve's own
//     comment) only drag vertically; interior points drag in both axes,
//     clamped between their own neighbors by TransferCurve::setPoint()
//     itself. Per-point interpolation, Inkscape-style: one click makes a
//     node linearly interpolated to its neighbours, click-and-drag places
//     its support points (the slope at the node) -- replacing the old
//     curve-wide Linear/Spline toggle button:
//       - A plain click on empty curve area (not on an existing handle,
//         and not a drag) calls addPoint() at that spot -- a corner node,
//         zero-length handle, straight lines to both neighbors.
//       - A click-AND-DRAG on empty curve area instead places a new
//         SMOOTH point in one motion: press sets the point's own
//         position, the drag (before release) sets its tangent handle
//         live as you move, release commits. This is the gesture that
//         used to start a marquee-select -- marquee moved to Shift+drag
//         (see the multi-select bullet below) so the two don't collide,
//         since placing a smooth point is now the primary curve-editing
//         gesture.
//       - Right-click-and-drag on an EXISTING point (any point,
//         including the two fixed endpoints) adjusts its own tangent
//         handle after the fact -- drag the point's own little handle
//         line out to make it smooth (or back to zero-length to make it
//         a corner again). A plain right-click that never crosses the
//         drag dead zone instead cycles the point's own hysteresis
//         direction tag (see that bullet below) -- the same press, two
//         different outcomes depending on whether it turned into a drag,
//         same "press-then-decide" pattern every other click-vs-drag
//         gesture in this file already uses.
//     Clicking in the graph adds points; double-clicking an *interior*
//     point removes it (the established
//     double-click-to-delete convention this project's own
//     ../statespace EqGraphWidget already uses for its own nodes) -- the
//     two fixed endpoints are never removable either way, matching
//     TransferCurve::removePoint()'s own refusal. See
//     plugin/dsp/CurveDsp.hpp's own header comment for the full Bezier-
//     handle math this replaces the old monotone-Hermite spline with.
//   - Threshold: a vertical line, deliberately drawn and dragged
//     completely independently of the curve's own points (it has no
//     handle sitting *on* the curve the way CompressorGraphWidget's own
//     Threshold handle does -- there is no single meaningful curve value
//     "at the threshold" here, unlike that plugin's continuous knee
//     construction). A colored background split -- one tint on the side the
//     trigger condition actually engages on, a neutral/dim tint on the
//     other -- makes which half is "live" visible at a glance without
//     reading the condition button's own label first. It also gets its own
//     small circular handle drawn on the line near the bottom edge, so it
//     reads as a node like any curve point, and can join a multi-
//     selection the same way (see below); it still only ever has an X.
//   - Hysteresis direction: right-clicking an *interior* point's handle
//     cycles its own sscomp::PointDirection tag (Both -> RisingOnly ->
//     FallingOnly -> Both) -- hysteresis: a node active only while the
//     input is rising, or only while falling, so the curve can take two
//     different paths up and down. A tagged point's
//     stroke tints green (rising) or violet (falling) instead of the
//     usual neutral gray -- a deliberately different color pair from the
//     amber/blue A-vs-B channel identity used everywhere else in this
//     plugin, so the two unrelated concepts never read as the same cue.
//     The two fixed endpoints never respond (always Both, by construction
//     -- see TransferCurve::setPointDirection()'s own refusal).
//   - Multi-select: shift-clicking a point's handle or the threshold's own
//     node-circle toggles its membership in a selection (drawn with an
//     extra highlight ring) without starting a drag; a Shift+click-drag
//     starting on empty curve area (missing every handle) marquees a
//     rectangle instead, selecting everything whose handle falls inside it
//     on release -- Shift is required here now (a plain, unmodified drag
//     on empty space places a smooth point instead, see the bullet
//     above). Once 2+ things are selected, dragging any one of them
//     drags the whole group together by the same delta -- the threshold,
//     if included, only ever receives the group's X component, never Y
//     (it has no Y of its own). A plain click on something NOT already part of
//     a 2+ selection instead collapses the selection to just that one
//     thing first, falling through to the ordinary single-item drag/
//     double-click-to-remove path below unchanged.
//
// Like CompressorGraphWidget, this takes plain data via setter calls from
// the owning UI rather than a shared host interface -- there's only ever
// one of these per plugin instance.

#pragma once

#include "NanoVG.hpp"

#include "../plugin/dsp/CurveDsp.hpp"

#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

START_NAMESPACE_DGL

class ContourGraphWidget : public NanoSubWidget
{
public:
    explicit ContourGraphWidget(Widget* parent);

    // The full curve -- called whenever it changes from outside this
    // widget (a preset/session load, or an undo/redo) so this widget's
    // own drawing/dragging stays in sync. Never called as a result of
    // this widget's own edits (those go the other way, through
    // setOnCurveChanged() below) -- avoids a feedback loop the way every
    // other bidirectional widget in this project family (ParamStripWidget,
    // CompressorGraphWidget) already avoids one.
    // Keeps the current selection while this widget is inside its own
    // onCurveChanged_ callback (the owner echoes the curve straight back
    // through here -- clearing the selection on that echo, as this used to,
    // dropped a multi-selection after a single drag step); an outside
    // change (preset load, undo/redo, the node list) with a different point
    // count still clears it, since indices no longer mean the same nodes.
    void setCurve(const sscomp::TransferCurve& curve)
    {
        const bool sameShape = curve.pointCount() == curve_.pointCount();
        curve_ = curve;
        if (!inOwnCallback_ && !sameShape) clearSelection();
        repaint();
    }
    const sscomp::TransferCurve& curve() const { return curve_; }

    // What a left-drag on the plot does -- see this file's header comment.
    enum class EditTool { DrawNode = 0, DrawRising = 1, DrawFalling = 2, Edit = 3 };
    void setTool(EditTool t)
    {
        if (t == tool_) return;
        tool_ = t;
        pendingAddPoint_ = false; rightPressPending_ = false;
        hoverPoint_ = -1; hoverHandle_ = -1;
        repaint();
    }
    EditTool tool() const { return tool_; }

    // Selection sync with the node list: the graph reports the primary
    // (most recently clicked/created) node, or -1 for none, and can be told
    // to select one from outside.
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }
    // Removes every selected interior node ("when selected and delete is
    // pressed they should be deleted all") -- the two fixed endpoints and
    // the threshold are never removed. Returns true if anything was
    // deleted. Called by the owning UI on the Delete/Backspace key, since
    // keyboard events reach the top-level UI, not this subwidget.
    bool deleteSelected();
    // The node the current selection was made FROM by a plain click / list
    // row / new node (-1 after a window or Shift-click, i.e. an explicit
    // multi-selection). deleteSelected() uses it the way the right-click menu
    // does: if the selection is just that node's own link group, only that
    // node goes.
    int primarySelected() const { return primary_; }
    // Closes the right-click menu if it's open (the owning UI calls this on
    // Escape). Returns true if there was one to close.
    bool closeMenu() { if (!menuOpen_) return false; menuOpen_ = false; repaint(); return true; }
    // Hover description for local (x,y) -- a node (with its values and what
    // the gestures do in the current tool), a support point, the threshold,
    // or empty for nothing worth describing. Used by
    // TooltipOverlay::addFieldTarget().
    std::string tooltipAt(double x, double y) const;
    void selectPoint(int index)
    {
        clearSelection();
        if (index >= 0 && index < curve_.pointCount()) selected_[index] = true;
        primary_ = index >= 0 && index < curve_.pointCount() ? index : -1;
        expandLinks();
        repaint();
    }

    void setThreshold(float thresholdDb) { thresholdDb_ = thresholdDb; repaint(); }
    void setTriggerCondition(sscomp::TriggerCondition cond) { condition_ = cond; repaint(); }

    // The live, per-frame readout -- called every uiIdle() tick, same
    // shape as CompressorGraphWidget's own setLiveLevels().
    // pushToTrail=false: the caller feeds the trail itself through pushTrailBatch()
    // (the plugin now sends a whole bank of points per block); the level/GR
    // here then only drive the marker, the GR column, hold and history.
    // Forgets the trail (its points mean something else after the dots' source switch).
    void clearTrail() { trailCount_[0] = trailCount_[1] = 0; trailPos_[0] = trailPos_[1] = 0; repaint(); }
    void setLiveLevels(float detectorLevelDb, float gainReductionDb, bool pushToTrail = true);
    // Makeup Gain shown on the bar like in the Compressor plugin -- ported from
    // CompressorGraphWidget::setMakeupGainDb(): each gain-reduction column draws the NET effect (gain reduction +
    // Makeup Gain) at full strength with the makeup-less reduction dimmed behind it, plus a marker line at the configured Makeup
    // Gain. bDb is B's own value, already resolved by the caller to whichever is live (its own bank while independent, A's while
    // not) -- see UIContour::refreshMakeupMarkers().
    void setMakeupGainDb(float aDb, float bDb) { makeupGainDb_ = aDb; makeupGainDbB_ = bDb; repaint(); }
    void setSecondaryLiveLevels(bool show, float detectorLevelDb, float gainReductionDb, bool pushToTrail = true);
    // Adds n (input dB, gain dB) points at once, all stamped "now" -- one
    // block's worth of samples per redraw (many more points than one per
    // redraw). voice 0 = A, 1 = B.
    // trig[k] != 0: the trigger condition was met at that point -- drawn in the green
    // palette instead of the amber/blue one.
    void pushTrailBatch(int voice, const float* levelDb, const float* grDb, const unsigned char* trig, int n);
    // How engaged the trigger is right now, 0..1 (the caller smooths it with the
    // attack/release times): the threshold line, its handle and the engaged-side tint
    // blend from yellow (0) to green (1), so a long release visibly fades back.
    void setTriggerActivity(float e) { triggerActivity_ = e < 0.0f ? 0.0f : (e > 1.0f ? 1.0f : e); repaint(); }
    void setSecondaryLabels(const char* a, const char* b) { labelA_ = a; labelB_ = b; }

    // Fired whenever this widget's own editing (dragging a point,
    // clicking to add one, double-clicking to remove one, or dragging
    // Threshold) changes something -- the owning UI forwards the curve
    // change to setState("curve", ...) and the threshold change to
    // setParameterValue(kParamThreshold, ...), same "widget fires a
    // callback, UI owns the actual Parameter/State round trip" pattern
    // every other editable widget in this project already uses.
    void setOnCurveChanged(std::function<void(const sscomp::TransferCurve&)> cb) { onCurveChanged_ = std::move(cb); }
    void setOnThresholdChanged(std::function<void(float)> cb) { onThresholdChanged_ = std::move(cb); }
    // How long the heat trail stays visible. Scrolling over the legend changes it; the owner mirrors the value to its other plots
    // (Dual-Mono / Mid-Side show two) so every channel looks alike. setTrailSeconds() does not fire the callback (no feedback loop).
    float trailSeconds() const { return trailSeconds_; }
    void setTrailSeconds(float s) { trailSeconds_ = std::max(kTrailSecondsMin, std::min(kTrailSecondsMax, s)); repaint(); }
    void setOnTrailSecondsChanged(std::function<void(float)> cb) { onTrailSecondsChanged_ = std::move(cb); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    void drawTransferCurve(float x, float y, float w, float h);
    void pushTrail(int voice, float levelDb, float grDb, bool trig = false);
    void drawTrail(int voice, bool secondary);
    void drawTrailLegend(float x, float y, float w, float h);
    Color trailColor(bool secondary, float frac, bool triggered = false) const;
    void drawGrMeterColumn(float x, float y, float w, float h, float grDb, float holdDb, float makeupDb, const char* label);
    void drawGrScale(float x, float y, float h);
    void drawGrHistory(float x, float y, float w, float h);
    void updateHold(float& holdDb, std::chrono::steady_clock::time_point& lastUpdate,
                     std::chrono::steady_clock::time_point& peakAt, float grDb);
    void smoothLiveValue(float& smoothed, std::chrono::steady_clock::time_point& lastUpdate, bool& init, float raw);

    float dbToPxX(float db) const;
    float dbToPxY(float db) const;
    float pxToDbX(float px) const;
    float pxToDbY(float py) const;

    // -1 if no point handle is under (px,py) within the hit radius, else
    // the index of the CLOSEST such handle (not just the first one found
    // by index order -- see this function's own comment on why that
    // distinction matters once two points sit near their minimum allowed
    // spacing).
    int hitPoint(float px, float py) const;
    // True if (px,py) is over the threshold line's own hit strip.
    bool hitThreshold(float px, float py) const;
    // Edit-tool threshold target: its round handle always, its LINE only while
    // the line is not hugging the plot's left/right edge. At a threshold of
    // -60dB (or +6dB) the line runs along the very edge of the plot, so its
    // grab strip covered exactly the corner where a selection window is
    // started, so a selection window couldn't be opened from the top-left
    // corner.
    bool thresholdLineGrabbable() const;
    bool hitThresholdEdit(float px, float py) const;
    // True if (px,py) is over the threshold's round handle near the bottom
    // of its line -- the only part of it grabbable in the Draw tools.
    bool hitThresholdNode(float px, float py) const;
    // Unclamped dB -> pixel mapping for support-point geometry: dbToPxX/Y()
    // clamp into the plot rect, which pinned a support point that lay outside
    // the plotted range (any end node -- its mirrored half is beyond the
    // edge -- or a node near a border) onto the border and so showed the
    // wrong slope. These let the line keep its true angle (drawn clipped to
    // the plot) while a dot outside the plot is simply not shown or grabbed.
    float dbToPxXRaw(double db) const { return curveX_ + float((db - sscomp::TransferCurve::kMinDb) / (sscomp::TransferCurve::kMaxDb - sscomp::TransferCurve::kMinDb)) * curveW_; }
    float dbToPxYRaw(double db) const { return curveY_ + curveH_ - float((db - sscomp::TransferCurve::kMinDb) / (sscomp::TransferCurve::kMaxDb - sscomp::TransferCurve::kMinDb)) * curveH_; }
    bool insidePlot(float px, float py) const { return px >= curveX_ && px <= curveX_ + curveW_ && py >= curveY_ && py <= curveY_ + curveH_; }
    // Starts dragging a support point if one is under (px,py) and closer than
    // the node (if any) the press also landed on -- shared by both tool
    // families (in a Draw tool, grabbing a support point must win over
    // placing a new node).
    bool tryGrabHandle(float px, float py, int hitNode);
    // The support point (tangent-handle dot) under (px,py), or -1. `isIn`
    // says which end: false = outgoing dot (point + handle), true =
    // incoming dot (point - handle). Only points that actually have a
    // handle draw dots, so only those can be hit. `dist` receives the
    // distance to the dot, for tie-breaking against a point's own circle.
    int hitHandle(float px, float py, bool& isIn, float& dist) const;
    bool isDrawTool() const { return tool_ != EditTool::Edit; }
    // Shared by both tool families: a press that landed on node `hit` --
    // double-click removes an interior node, otherwise it starts dragging
    // that one node. Always consumes the press.
    // asGroup: the node is part of a 2+ selection / link group, so after the
    // double-click-to-remove check the press starts a group drag instead of
    // dragging just that node. (A linked node can therefore be removed by
    // double-click too; the rest of its group stays, dissolving if it drops
    // below two members.)
    bool beginPointPress(int hit, float x, float y, bool asGroup = false);
    sscomp::PointDirection directionForTool() const
    {
        return tool_ == EditTool::DrawRising ? sscomp::PointDirection::RisingOnly
             : tool_ == EditTool::DrawFalling ? sscomp::PointDirection::FallingOnly
             : sscomp::PointDirection::Both;
    }
    // Fires onCurveChanged_ with inOwnCallback_ set -- see setCurve().
    void notifyCurveChanged()
    {
        if (!onCurveChanged_) return;
        inOwnCallback_ = true;
        onCurveChanged_(curve_);
        inOwnCallback_ = false;
    }
    void notifySelection(int primary) { primary_ = primary; if (onSelectionChanged_) onSelectionChanged_(primary); }

    // Link groups (sscomp::CurvePoint::link): selecting any member selects
    // the whole group, so linked nodes always move together through the
    // ordinary group-drag path -- "once linked they move with each other
    // always like selected all the time". Called after every selection
    // change.
    void expandLinks()
    {
        bool grp[sscomp::kMaxLinkGroups + 1] = {};
        const int tl = curve_.thresholdLink();
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (selected_[i] && curve_.pointLink(i) > 0) grp[curve_.pointLink(i)] = true;
        if (thresholdSelected_ && tl > 0) grp[tl] = true;
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (curve_.pointLink(i) > 0 && grp[curve_.pointLink(i)] && !selected_[i]) { selected_[i] = true; viaLink_[i] = true; }
        if (tl > 0 && grp[tl]) thresholdSelected_ = true; // the threshold follows its group, on X only (see onMotion's group drag)
    }

    // Right-click node menu: Normal / Rising / Falling / Link / Unlink /
    // Snap to Diagonal / Delete, applied to the selection (a right-click on
    // an unselected node selects it, with its link group, first). Drawn and
    // hit-tested by this widget itself, like the sibling ../statespace
    // EqGraphWidget's own context menu.
    enum class MenuAction { Normal, Rising, Falling, Link, LinkX, LinkY, Unlink, UnlinkGroup, SnapDiagonal, ReleaseDiagonal, Delete };
    struct MenuItem { MenuAction action; const char* label; bool enabled; bool checked; bool sepBefore; };
    // What the open menu acts on: the whole selection when it is an explicit
    // multi-selection, but ONLY the right-clicked node when the selection is
    // just that node's own link group (which is what merely selecting a
    // linked node produces) -- so "Unlink" on a member unlinks that member,
    // not the whole group. Fixed when the menu opens.
    bool menuTarget_[sscomp::TransferCurve::kMaxPoints] = {};
    bool menuTargetThr_ = false;
    // Removes every interior node marked in `mark` (highest index first);
    // shared by deleteSelected() and the menu's Delete.
    bool removeMarked(const bool* mark);
    void openMenu(int point, float x, float y);
    void drawMenu();
    int menuItemAt(float px, float py) const;
    void runMenuAction(MenuAction a);
    static constexpr float kMenuW = 170.0f, kMenuItemH = 22.0f, kMenuSepH = 7.0f;
    bool menuOpen_ = false;
    MenuItem menuItems_[12] = {};
    int menuCount_ = 0;
    float menuX_ = 0, menuY_ = 0, menuH_ = 0;
    int menuHover_ = -1;
    char menuLabels_[2][32] = {};

    // Multi-select bookkeeping -- see this file's own header comment.
    void clearSelection() { std::fill(std::begin(selected_), std::end(selected_), false); std::fill(std::begin(viaLink_), std::end(viaLink_), false); thresholdSelected_ = false; primary_ = -1; }
    int selectedCount() const
    {
        int n = thresholdSelected_ ? 1 : 0;
        for (int i = 0; i < curve_.pointCount(); ++i) if (selected_[i]) ++n;
        return n;
    }
    // Snapshots every current point's (and, if selected, the threshold's)
    // starting position, and the press position itself in dB space, so
    // onMotion()'s group drag can compute one clean delta from the press
    // rather than accumulating many small per-frame deltas (which would
    // drift under repeated clamping/snapping at a neighbor).
    void beginGroupDrag(float x, float y);
    // Replaces the selection with everything whose handle falls inside
    // the marquee rectangle (marqueeX0_/Y0_ .. current position) -- or
    // adds to the existing selection instead when marqueeAdditive_ (the
    // marquee was started with shift already held).
    void applyMarqueeSelection();

    sscomp::TransferCurve curve_;
    EditTool tool_ = EditTool::DrawNode;
    bool inOwnCallback_ = false;
    int primary_ = -1; // see primarySelected()
    std::function<void(int)> onSelectionChanged_;
    // Ghost-node preview + support-point hover (Draw/Edit tools).
    float hoverX_ = 0, hoverY_ = 0;
    int hoverHandle_ = -1;
    bool hoverHandleIsIn_ = false;
    float thresholdDb_ = -18.0f;
    sscomp::TriggerCondition condition_ = sscomp::TriggerCondition::Above;

    float triggerActivity_ = 0.0f; // see setTriggerActivity()
    float detectorLevelDb_ = -120.0f, gainReductionDb_ = 0.0f;
    std::chrono::steady_clock::time_point grSmoothUpdated_{};
    bool grSmoothInit_ = false;
    bool secondaryShown_ = false;
    float detectorLevelDbB_ = -120.0f, gainReductionDbB_ = 0.0f;
    float makeupGainDb_ = 0.0f, makeupGainDbB_ = 0.0f; // configured Makeup Gain of each column, see setMakeupGainDb()
    std::chrono::steady_clock::time_point grSmoothUpdatedB_{};
    bool grSmoothInitB_ = false;

    // Raw (unsmoothed) last gain reduction per voice, for the "now" head
    // marker only -- deliberately NOT the same as gainReductionDb_/
    // gainReductionDbB_ above, which run through smoothLiveValue()'s own
    // ~220ms display easing (good for a readable number on the meter
    // chip/column, but that's an extra layer of lag on top of whatever
    // the DSP itself is already doing, which would make the marker lie
    // about a literal 0ms attack/release actually tracking the curve
    // instantly). See setLiveLevels()/drawTrail().
    float grDbRaw_ = 0.0f, grDbRawB_ = 0.0f;

    // A fading trail of recent (input dB, output dB) operating points per
    // voice -- a fading heat map showing where the audio is going, and how
    // it follows the line (and leaves it) with attack and release. Ring
    // buffer of raw dB pairs (not
    // pixel coordinates -- converted at draw time so a mid-trail window
    // resize doesn't leave stale positions) plus a real timestamp per
    // sample, since the idle-callback rate pushing these isn't fixed
    // across hosts (same reasoning as updateHold()/smoothLiveValue()'s
    // own use of steady_clock deltas rather than a frame count). Faded-
    // out entries are simply skipped at draw time, not pruned -- see
    // drawTrail()'s own comment. kTrailCapacity is sized generously
    // against kTrailSecondsMax (see below) at a deliberately-pessimistic
    // assumed idle-callback rate, not tuned to any one host's actual
    // rate -- see drawTrailLegend()'s own comment on the one case this
    // still isn't enough for.
    struct TrailPoint { float inDb, outDb; std::chrono::steady_clock::time_point t; bool trig; };
    // 20s at up to ~60 redraws/s x 48 points per redraw is ~58k points; 65536 per
    // voice (16 bytes each, ~1MB) covers it. (Was 6144 when one point per redraw
    // was pushed.) Points are drawn as small squares batched per age bucket, see
    // drawTrail().
    static constexpr int kTrailCapacity = 65536;
    TrailPoint trail_[2][kTrailCapacity]{};
    int trailPos_[2] = { 0, 0 };
    int trailCount_[2] = { 0, 0 };

    // How far back the trail reaches (a longer trail costs more CPU): a
    // real,
    // user-adjustable setting (scroll over the small legend swatch
    // drawTrailLegend() draws, see hitTrailLegend()) rather than a fixed
    // constant, plus that same legend doubles as the "where the limit is"
    // readout -- it shows both the configured duration and, whenever the
    // trail hasn't actually been playing that long yet (the plugin was
    // just opened, or the buffer's own hard cap above is somehow reached
    // first), how much history is *actually* being shown right now.
    float trailSeconds_ = 3.0f;
    static constexpr float kTrailSecondsMin = 0.5f;
    static constexpr float kTrailSecondsMax = 20.0f; // was 10s; kTrailCapacity scaled to match, see below.
    bool hoverTrailLegend_ = false;
    float trailLegendX_ = 0, trailLegendY_ = 0, trailLegendW_ = 0, trailLegendH_ = 0;
    bool hitTrailLegend(float px, float py) const;
    std::string labelA_ = "GR", labelB_ = "GR B";

    // Peak-hold marker per column -- identical mechanism to
    // CompressorGraphWidget's own, see that file's member comment.
    float grHoldDb_[2] = { 0.0f, 0.0f };
    std::chrono::steady_clock::time_point grHoldUpdated_[2]{};
    std::chrono::steady_clock::time_point grHoldPeakAt_[2]{};

    static constexpr int kHistoryLen = 240;
    float history_[kHistoryLen] = {};
    int historyPos_ = 0;

    static constexpr float kGrMeterMinDb = -90.0f; // a gate can now cut up to the curve window's full width (80 dB + the level) -- see drawGrMeterColumn()'s own comment
    static constexpr float kGrMeterMaxDb = 30.0f;  // curve output can exceed input by up to the curve's own full range width

    float curveX_ = 0, curveY_ = 0, curveW_ = 0, curveH_ = 0;

    // Point-drag state, live only between a press on a handle and its
    // release.
    int draggingPoint_ = -1;
    int hoverPoint_ = -1;
    // Kept smaller than half the on-screen gap two adjacent points have at
    // TransferCurve::kMinSeparationDb apart (at this widget's typical
    // width) so a click aimed at the gap between two closely-packed
    // points still has real empty room to land in, rather than always
    // falling inside one point's own grab circle -- see hitPoint()'s
    // nearest-match fix (this file's .cpp) for the other half of the same
    // problem (points close together were unreliable to edit).
    static constexpr float kHandleHitRadiusPx = 7.0f;
    // Within this many pixels of a node's centre a press is always the NODE's (see tryGrabHandle()).
    static constexpr float kNodeCoreRadiusPx = 8.0f;
    // Grab radii. The Edit tool never creates nodes, so it can afford big,
    // forgiving targets (nodes at the right edge and support points are
    // otherwise hard to grab); the Draw tools keep small ones so a
    // node can still be placed right next to another.
    float nodeHitRadius() const { return isDrawTool() ? kHandleHitRadiusPx : 11.0f; }
    float dotHitRadius() const { return isDrawTool() ? 8.0f : 12.0f; }

    // Distance (from the point handle's own press position) a drag must
    // travel before it counts as "this press really was a drag" for
    // double-click-to-remove purposes below -- see lastClickWasDrag_'s own
    // comment.
    static constexpr float kPointDragThresholdPx = 3.0f;

    // Threshold-drag state -- a dead zone so a plain click near (but not
    // exactly on) the line doesn't immediately start a drag, same
    // "press-then-decide" pattern EqGraphWidget's own right-click gesture
    // in the sibling ../statespace project uses (just
    // simpler here: there's only ever the one gesture on this line, no
    // click-vs-drag ambiguity to resolve against a *different* action).
    bool draggingThreshold_ = false;
    bool hoverThreshold_ = false;
    // Half-width of the threshold LINE's grab strip. Kept narrow: the line spans
    // the whole plot height, so a wide strip steals every press that should
    // start a selection window (at a low threshold it sits on the plot's left
    // edge, exactly where a window aimed at the top-left corner begins).
    static constexpr float kThresholdHitPx = 4.0f;
    static constexpr float kThresholdEdgeGuardPx = 10.0f; // see thresholdLineGrabbable()

    // Double-click-to-remove state for interior points, same convention
    // as EqGraphWidget's own node double-click. lastClickWasDrag_ tracks
    // whether the *previous* press on lastClickPoint_ actually turned into
    // a drag (moved past kPointDragThresholdPx before release) rather than
    // staying a plain click -- without this, two quick successive re-grabs
    // of the same point to nudge it twice in a row (a normal way to fine-
    // tune a curve) were indistinguishable from a genuine double-click and
    // silently deleted the point instead of moving it a second time.
    int lastClickPoint_ = -1;
    std::chrono::steady_clock::time_point lastClickTime_{};
    bool lastClickWasDrag_ = false;
    float pointPressX_ = 0, pointPressY_ = 0;

    // Distinguishes "this press turned into a drag" from "this press was
    // a plain click that should add a new corner point" -- a press on
    // empty curve area starts as neither yet; once it moves past the
    // dead zone, it's resolved in onMotion() into either a marquee
    // (pendingAddShiftHeld_ was true at press) or placing a new smooth
    // point (Shift wasn't held -- see placingSmoothPoint_ below); if it
    // never crosses the dead zone at all, release adds a plain corner
    // point right there instead.
    bool pendingAddPoint_ = false;
    bool pendingAddShiftHeld_ = false;
    float pressX_ = 0, pressY_ = 0;
    static constexpr float kAddPointDeadZonePx = 4.0f;

    // Click-and-drag places a new node's support points -- live only
    // between pendingAddPoint_ crossing the dead zone (Shift not held)
    // and release. The new point is created the instant the drag crosses
    // the dead zone (not deferred to release, unlike the plain-click
    // corner-point path) so its own handle can be shown updating live for
    // the rest of the drag, exactly like dragging an existing point's
    // handle does (draggingHandle_ below).
    bool placingSmoothPoint_ = false;
    int placingSmoothIndex_ = -1;

    // Right-click-drag-to-adjust-a-handle state -- see this file's own
    // header comment. rightPressPending_/rightPressPoint_/rightPressX_/Y_
    // mirror pendingAddPoint_'s own "press, then decide on motion or
    // release" shape, just for the right button: a plain right-click
    // (never crosses the dead zone) cycles the point's own hysteresis
    // direction tag instead of adjusting its handle.
    bool rightPressPending_ = false;
    int rightPressPoint_ = -1;
    float rightPressX_ = 0, rightPressY_ = 0;
    bool draggingHandle_ = false;
    int draggingHandlePoint_ = -1;
    // Left-button support-point drag (Edit tool): which end was grabbed,
    // so dragging the incoming dot mirrors correctly.
    bool draggingHandleIsIn_ = false;
    bool draggingHandleLeft_ = false;
    // Double-click on a support point resets that node's handle to 0/0 (a
    // corner again) -- same "second press within 400ms, and the first one was
    // not a drag" rule as double-click-to-remove on nodes.
    int lastHandleClickPoint_ = -1;
    std::chrono::steady_clock::time_point lastHandleClickTime_{};
    bool lastHandleClickWasDrag_ = false;
    float handlePressX_ = 0, handlePressY_ = 0;

    // Multi-select / group-drag / marquee state -- see this file's own
    // header comment.
    bool selected_[sscomp::TransferCurve::kMaxPoints] = {};
    // Nodes that are selected ONLY because a link group pulled them in (not by a
    // click / window / Shift-click). A group drag moves these by the group's
    // axis only (X-only / Y-only links), while explicitly selected nodes always
    // move freely. Reset by clearSelection(); set by expandLinks().
    bool viaLink_[sscomp::TransferCurve::kMaxPoints] = {};
    bool thresholdSelected_ = false;

    bool draggingGroup_ = false;
    double groupPressInputDb_ = 0.0, groupPressOutputDb_ = 0.0;
    sscomp::CurvePoint groupStart_[sscomp::TransferCurve::kMaxPoints]{};
    double groupStartThresholdDb_ = 0.0;

    bool marqueeActive_ = false;
    // True when the marquee's own press had shift already held -- ADDS to
    // the existing selection on release instead of replacing it, the same
    // "shift extends, plain replaces" convention the point/threshold
    // shift-click gesture above already uses.
    bool marqueeAdditive_ = false;
    float marqueeX0_ = 0, marqueeY0_ = 0, marqueeX1_ = 0, marqueeY1_ = 0;

    std::function<void(const sscomp::TransferCurve&)> onCurveChanged_;
    std::function<void(float)> onTrailSecondsChanged_;
    std::function<void(float)> onThresholdChanged_;
};

END_NAMESPACE_DGL
