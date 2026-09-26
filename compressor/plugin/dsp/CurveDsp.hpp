// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// CurveDsp.hpp
//
// The DSP core of the "Contour" plugin -- a compressor whose input/output
// mapping is an arbitrary, user-drawn curve instead of a fixed threshold/
// ratio/knee formula, able to build special behaviours: upward and
// downward compression, but also gates. No DPF/DGL dependency -- same header-only, framework-
// independent convention as the sibling CompressorDsp.hpp, which this
// file includes and builds directly on (EnvelopeFollower, LoudnessMeter,
// PeakMeter, CompensationGain, StereoMode are all reused unchanged; only
// the gain-computer -- CompressorVoice's static ratio/knee curve -- is
// replaced here).
//
// ---- The curve itself: TransferCurve --------------------------------
//
// A small, ordered list of (inputDb, outputDb) control points -- "pillar
// points" -- mapping input level to output level directly (not a gain
// *change*; evaluate() - inputDb is the change, see computeCurveGainDb()
// below). The two endpoints (index 0 and count()-1) sit at fixed X
// (kMinDb/kMaxDb) forever -- only their own Y (height) is ever
// adjustable -- while interior points (added by clicking in the graph)
// are fully draggable in both
// axes, clamped so they can never cross or reorder past a neighbor -- with
// one deliberate exception: dragging an interior point to within
// kMinSeparationDb of a neighbor SNAPS it flush to that neighbor's own
// inputDb instead of stopping just short, forming two adjacent points that
// share the exact same X -- a genuine step/gate discontinuity, not just a
// steep ramp, e.g. for a gate: evaluate() resolves a shared-x pair with
// a >= convention -- the *later* (right-hand, upper-index) point of the pair
// governs for any input at or above the shared X, the earlier one governs
// strictly below it -- see evaluate()'s own comment. A point may share its
// X with at most one neighbor, never both (that would stack three points
// at one X, which side wins would be ambiguous) -- setPoint() below
// refuses to snap on whichever side is already itself the far half of a
// different pair.
//
// ---- per-point interpolation: Inkscape-style tangent handles ---------
//
// No more curve-wide Linear/Spline toggle -- linear and spline segments
// mix freely, Inkscape-style: one click makes a node linearly
// interpolated to its neighbours, click-and-drag places its support
// points (the slope at the node). Each point now carries its own optional tangent-handle
// offset (CurvePoint::handleInputDb/handleOutputDb) -- a vector FROM the
// point, defining a single tangent LINE through it (the incoming/
// leftward half is the same vector negated; a real "smooth node," not
// independently-draggable in/out handles -- one slope at the point, not
// two):
//   - A zero-length handle (the default -- what a plain click via
//     addPoint() creates) is a corner node: the two segments touching it
//     are exact straight lines, no curvature at all -- this is what
//     "Linear" meant before, now just the default, unset state of any
//     point rather than a curve-wide mode.
//   - A non-zero handle makes that point a smooth node: the segment(s)
//     touching it become a real cubic Bezier (see evaluateFor()'s own
//     comment for the actual construction and how it's solved). Unlike
//     the monotone Hermite spline this replaces, a hand-placed Bezier
//     handle is *not* clamped against overshoot on the output (Y) axis --
//     letting the curve intentionally swing past a neighboring point's
//     own value on the way through is the entire point of manual handle
//     control, a deliberate trade against the old spline's automatic
//     safety net (see setPointHandle()'s own comment for the one
//     constraint that *is* still enforced, and why).
// Segments can mix freely -- a corner point next to a smooth one still
// produces a well-defined (if less symmetric) curve through both, since
// each segment's own two control points are derived independently from
// its own two endpoints' handles.
//
// This project's own dsp/CompressorDsp.hpp-adjacent CompressorGraphWidget/
// EqGraphWidget precedent for "assemble once on edit, evaluate cheaply on
// the audio thread" still applies -- rebuild() (called by every mutator)
// keeps the direction-filtered point views (see PointDirection below)
// current; there's no longer any tangent array to precompute at all
// (the old monotone-Hermite tangent pass is gone -- a handle's own stored
// offset already fully determines its Bezier control point, nothing left
// to derive from neighbors up front).
//
// TransferCurve is a fixed-size, allocation-free POD-like struct (no
// std::vector), the same convention DynamicEqRtModel/MbCompressorModel's
// own real-time-facing structs already use in the sibling ../statespace
// project. It is NOT behind the double-buffer/mutex/version-counter
// machinery those *bigger* structures use (a whole assembled state-space
// matrix, hundreds of doubles) -- a curve of a handful of points is much
// closer in size/shape to this project's own existing threshold_/ratio_/
// kneeDb_ scalars, which have never needed that either; the same "the
// host serializes control changes with the audio callback closely enough
// in practice" trust this project's Plugin*.cpp classes already extend to
// every other Parameter-driven field applies here too. A torn read mid-
// edit could, in principle, cost one audio block a transient odd-looking
// curve value -- never a crash or a stuck state -- since evaluate() never
// reads past count_'s own bound and every point stays inside [kMinDb,
// kMaxDb] by construction.
//
// ---- The independent threshold + trigger condition -------------------
//
// A plain thresholdDb, deliberately NOT tied to any curve point (but
// visible in the GUI), plus a two-way
// TriggerCondition: Above (the classic compressor/limiter shape --
// engage the curve once the level *exceeds* threshold, identity below
// it) or Below (the classic gate/expander shape -- engage once the level
// *drops under* threshold, identity above it). Whichever side is
// "engaged" is where the curve's own shape actually reaches the audio;
// the other side is always a flat, untouched identity, regardless of
// what the curve happens to say there -- opening or closing depends only
// on the drawn curve: a Below
// condition with a curve that dives toward heavy attenuation as input
// drops is a gate closing; the identical mechanism with an Above
// condition and a curve that lifts quiet material is upward compression;
// with a curve that cuts loud material, ordinary downward compression --
// one mechanism, no separate "up/down" mode switch needed at all.

#pragma once

#include "CompressorDsp.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace sscomp {

// ---- trigger condition -------------------------------------------------

enum class TriggerCondition { Above = 0, Below = 1 };

inline TriggerCondition clampTriggerCondition(int raw) { return raw == 1 ? TriggerCondition::Below : TriggerCondition::Above; }

// What the trigger DOES once its condition is met: take the sound to the custom curve, or
// from the custom curve to the diagonal (open/close).
//   ApplyCurve        (the original behaviour): trigger met -> the drawn curve, else the diagonal.
//   ReleaseToDiagonal (a gate-style opener):    trigger met -> the diagonal, else the drawn curve.
// The region where the curve is active is (condition met) XOR (ReleaseToDiagonal); the trigger
// itself -- what the GUI shows green -- is the same in both. In ReleaseToDiagonal mode the
// smoothing times follow the trigger, gate style: Attack = how fast the sound goes to the
// diagonal when the trigger fires ("opening"), Release = how fast it returns to the curve after
// it ends ("closing"); see CurveVoice::process().
enum class TriggerAction { ApplyCurve = 0, ReleaseToDiagonal = 1 };
inline TriggerAction clampTriggerAction(int raw) { return raw == 1 ? TriggerAction::ReleaseToDiagonal : TriggerAction::ApplyCurve; }

// Hysteresis -- two different paths up and down, via a per-point tag
// (active only while the input is rising, or only while falling), not a
// whole second curve:
// Both (the default -- every point behaves exactly as before, so an
// untagged curve is bit-for-bit identical whichever direction it's
// evaluated in) means "always part of the shape"; RisingOnly/FallingOnly
// means "only part of the shape while the detector's own tracked level is
// moving that way" -- see TransferCurve::evaluateDirectional()'s own
// comment for how that's actually resolved into two effective curves
// sharing the same point list, and CurveVoice::process() for where
// "rising or falling" is judged from.
enum class PointDirection { Both = 0, RisingOnly = 1, FallingOnly = 2 };

inline PointDirection clampPointDirection(int raw)
{
    return raw == 1 ? PointDirection::RisingOnly : raw == 2 ? PointDirection::FallingOnly : PointDirection::Both;
}

// ---- the curve itself -------------------------------------------------

struct CurvePoint
{
    double inputDb = 0.0;
    double outputDb = 0.0;
    // See PointDirection's own comment. Always Both for the two fixed
    // endpoints in practice -- TransferCurve::setPointDirection()/
    // loadPoints() both enforce that, see their own comments -- so a
    // direction-filtered view (evaluateDirectional()) always has
    // somewhere to anchor both its own ends regardless of what's tagged
    // in between.
    PointDirection direction = PointDirection::Both;
    // Inkscape-style tangent handle -- see this file's own header
    // comment on "per-point interpolation" for the full design. (0,0) --
    // the default -- is a corner node (straight lines to both
    // neighbors); TransferCurve::setPointHandle() is the only place that
    // should ever set these away from (0,0), since it's also where the
    // one safety constraint (the X offset can't push a control point
    // backward or past a neighbor) is enforced.
    // The handle vector d. The two support points are P + d (the bright one,
    // "the one you hold") and P - d; the tangent LINE through P is what shapes
    // the curve, so d and -d describe the same curve. dx is therefore allowed
    // to be NEGATIVE: it just says which side the bright dot sits on (e.g. the
    // right end node, whose bright dot must lie to the LEFT to be inside the
    // plot). evaluate() canonicalizes to dx >= 0 (see canonHandle*()), and
    // |dx| is what setPointHandle()/maxHandleDx() limit.
    double handleInputDb = 0.0;
    double handleOutputDb = 0.0;
    // Editing-only link group (0 = not linked, else 1..kMaxLinkGroups):
    // nodes sharing a non-zero id always move together in the GUI, as if
    // they were permanently selected together -- "once linked they move with
    // each other always". The DSP never reads it; it lives here (rather than
    // in the GUI) so it rides along with everything else a point carries --
    // add/remove index shifts, undo/redo, A/B, presets, session restore.
    // TransferCurve::normalizeLinks() keeps every group at 2+ members.
    int link = 0;
    // What a link moves: 0 = X and Y together, 1 = X only, 2 = Y only. All
    // members of a group share it (setPointLink() takes it), and the dragged
    // / edited node itself always moves freely -- the axis only limits what is
    // PROPAGATED to the other members. 0 for unlinked points.
    int linkAxis = 0;
    // Editing-only diagonal snap ("snap nodes to the diagonal" -- direct
    // ask): while set, TransferCurve::setPoint() forces outputDb == inputDb
    // on every write to this point, from any source (direct drag, a group
    // drag pulling it along by link, or a numeric edit via moveLinked()), so
    // the point can only ever slide along the unity line -- exactly one
    // degree of freedom, the same way the threshold (which has no Y at all)
    // only ever moves left/right. A snapped point stays a completely
    // ordinary point otherwise -- still an interior/endpoint node, still
    // eligible for its own link group like any other -- see setPointDiagonal().
    bool diagonal = false;
};

constexpr int kMaxLinkGroups = 9; // 18 points max, 2 per group

// ---- atomic (detector level, gain) pair for the live trail ---------------
//
// The GUI's heat trail plots (input dB, input dB + gain dB) per sample. Those
// two numbers used to travel as two separate output Parameters, and the host
// reads output ports one after another while the audio thread keeps running:
// with an instant detector (0ms attack/release) the level changes every
// sample, so a pair read across two audio blocks is simply a wrong point --
// dots scattered on both sides of the drawn curve although the engine sat
// exactly on it (verified). One Parameter can't tear, so the pair is packed
// into a single value: two 12-bit quantized numbers = a 24-bit integer, which
// a 32-bit float represents EXACTLY (integers < 2^24), i.e. the packed value
// survives any float transport untouched.
// What is stored is the LEVEL and the OUTPUT position (level + gain), not the gain:
// the dot's y is level + gain, and the gain a low curve end needs at a high
// detector level (right node at -50dB, detector at +34dB -> -84dB) far exceeds any
// sensible gain range. Packing the gain clamped it, so above roughly +22dB the dot's
// y drifted upward (with the signal above +6dB and the right node low, the dots
// climbed up the right edge). The output
// position stays within the curve's own range (-60..+6dB) whenever the curve is
// active, and anything beyond the plot is drawn on its edge anyway, so clamping THAT
// is harmless. The level range is wide because Trigger Gain (up to +60dB) is applied
// before the detector: the detector level can sit far above the input's own peak.
// Resolution: 240dB/4095 = 0.059dB for the level, 200dB/4095 = 0.049dB for the output.
constexpr double kTrailLevelMinDb = -120.0, kTrailLevelMaxDb = 120.0;
constexpr double kTrailOutMinDb = -100.0, kTrailOutMaxDb = 100.0;
// The sign bit of the float carries one more flag for free: "the trigger condition was
// met at this sample" (negative = triggered) -- lets the GUI colour each dot by whether
// the curve was actually engaged, at no cost in precision (the magnitude is unchanged).
inline float packTrailPair(double levelDb, double gainDb, bool triggered = false)
{
    auto q = [](double v, double lo, double hi) {
        const double t = (std::max(lo, std::min(hi, v)) - lo) / (hi - lo);
        return int(std::lround(t * 4095.0));
    };
    const float mag = float(q(levelDb, kTrailLevelMinDb, kTrailLevelMaxDb) * 4096 + q(levelDb + gainDb, kTrailOutMinDb, kTrailOutMaxDb));
    return triggered ? -mag : mag;
}
inline void unpackTrailPair(float packed, double& levelDb, double& gainDb, bool& triggered)
{
    triggered = std::signbit(packed);
    // Round in double: `packed + 0.5f` in float loses the .5 above 2^23 and
    // an off-by-one there would carry from the output field into the level.
    const int v = std::max(0, std::min(16777215, int(std::lround(std::fabs(double(packed))))));
    levelDb = kTrailLevelMinDb + double(v / 4096) / 4095.0 * (kTrailLevelMaxDb - kTrailLevelMinDb);
    const double outDb = kTrailOutMinDb + double(v % 4096) / 4095.0 * (kTrailOutMaxDb - kTrailOutMinDb);
    gainDb = outDb - levelDb; // exact for the dot (level, level + gain) even when the level itself had to be clamped
}
inline void unpackTrailPair(float packed, double& levelDb, double& gainDb)
{
    bool ignored;
    unpackTrailPair(packed, levelDb, gainDb, ignored);
}

// The Parameter mirror packs a point's direction and link group into the one
// existing per-point "direction" Parameter (value = direction + 3*link) --
// old sessions only ever stored 0..2 there, which decodes as "link 0", so
// they load unchanged, and no new Parameter (index shift) was needed.
inline constexpr int kLinkAxisBoth = 0, kLinkAxisX = 1, kLinkAxisY = 2;
// value = direction + 3 * (link + 10 * slot), slot = axis + 3 * diagonal:
// link 0..9, axis 0..2, diagonal 0..1 (slot 0..5). Older values (slot 0..2,
// i.e. no diagonal flag) decode exactly as before -- the diagonal flag only
// ever pushes slot into 3..5, a range the old scheme never produced.
inline float packDirectionAndLink(PointDirection d, int link, int axis = 0, bool diagonal = false)
{
    const int slot = std::max(0, std::min(kLinkAxisY, axis)) + (diagonal ? 3 : 0);
    return float(int(d) + 3 * (link + 10 * slot));
}
// A genuine packed value is an exact integer. Sessions saved while the mirror
// banks overlapped (see PluginContour.hpp's LAYOUT NOTE) can hold a support-point
// dB value (fractional) in these slots instead -- treat anything that isn't an
// integer as "no information" (Both / unlinked) rather than decoding noise.
inline bool isPackedInteger(float v) { return std::fabs(v - std::round(v)) < 1.0e-3f && v >= 0.0f; }
inline PointDirection unpackDirection(float packed)
{
    return isPackedInteger(packed) ? clampPointDirection(int(packed + 0.5f) % 3) : PointDirection::Both;
}
inline int unpackLink(float packed)
{
    if (!isPackedInteger(packed)) return 0;
    const int v = (int(packed + 0.5f) / 3) % 10;
    return v > kMaxLinkGroups ? 0 : v;
}
inline int unpackLinkAxis(float packed)
{
    if (!isPackedInteger(packed)) return 0;
    const int slot = (int(packed + 0.5f) / 3) / 10;
    return slot % 3;
}
inline bool unpackPointDiagonal(float packed)
{
    if (!isPackedInteger(packed)) return false;
    const int slot = (int(packed + 0.5f) / 3) / 10;
    return slot >= 3 && slot <= 5;
}

// Same trick for the threshold's own link group (see
// TransferCurve::thresholdLink()): the curve's trigger-condition Parameter
// (0 = Above, 1 = Below) carries it as condition + 2*group. Old sessions
// hold 0/1 there, i.e. "not linked".
// value = condition + 2 * (thresholdLink + 10 * action): action 0 = ApplyCurve, 1 = ReleaseToDiagonal. Older values
// (action 0) decode unchanged.
inline float packConditionAndThresholdLink(int conditionInt, int link, int action = 0) { return float(conditionInt + 2 * (link + 10 * action)); }
inline int unpackConditionInt(float packed) { return int(packed + 0.5f) % 2; }
inline int unpackThresholdLink(float packed)
{
    const int v = (int(packed + 0.5f) / 2) % 10;
    return v < 0 ? 0 : (v > kMaxLinkGroups ? kMaxLinkGroups : v);
}
inline int unpackTriggerActionInt(float packed)
{
    const int a = (int(packed + 0.5f) / 2) / 10;
    return a == 1 ? 1 : 0;
}

class TransferCurve
{
public:
    // Both axes share one range: [-80, +6] dB. -80 lets a gate close all the way down to inaudible (the curve can
    // only lower a sound to its own lowest output, so the window's floor is the gate's floor -- it used to be -60,
    // matching CompressorGraphWidget's own window); +6dB comfortably covers an upward-compression boost.
    // Curves saved with the old -60 window are migrated on load, see loadPoints().
    static constexpr double kMinDb = -80.0;
    static constexpr double kLegacyMinDb = -60.0; // the window before the extension
    static constexpr double kMaxDb = 6.0;
    static constexpr int kMaxPoints = 18; // 2 fixed endpoints + up to 16 interior pillars
    // Minimum X gap enforced between any two adjacent points (including
    // against the two fixed endpoints) -- guards every division in
    // evaluate()/computeTangents() against a near-zero segment width, and
    // keeps points individually grabbable rather than pixel-adjacent.
    static constexpr double kMinSeparationDb = 1.0;

    TransferCurve() { resetToDefault(); }

    // Back to a plain 2-point identity line (both endpoints at their own
    // X, Y = X -- i.e. "no change at all" regardless of threshold/
    // condition) -- the safe, does-nothing starting shape every other
    // default-valued control in this project's plugins also starts at
    // (ratio=1, knee=0, etc.). Leaves the interpolation mode untouched
    // (resetting the curve's own shape shouldn't silently also flip a
    // separate, unrelated preference).
    void resetToDefault()
    {
        count_ = 2;
        points_[0] = { kMinDb, kMinDb };
        points_[1] = { kMaxDb, kMaxDb };
        thresholdLink_ = 0;
        rebuild();
    }

    int pointCount() const { return count_; }
    CurvePoint point(int index) const
    {
        if (index < 0) index = 0;
        if (index >= count_) index = count_ - 1;
        return points_[index];
    }
    bool isEndpoint(int index) const { return index == 0 || index == count_ - 1; }

    // Adds an interior point at (inputDb, outputDb), keeping points_
    // sorted by inputDb. Refused (returns -1, curve unchanged) if already
    // at kMaxPoints, or if the requested inputDb would land closer than
    // kMinSeparationDb to either neighbor it would fall between --
    // guarding this here, not just in the GUI, means the invariant every
    // other method below relies on (adjacent points always at least
    // kMinSeparationDb apart) can never be violated regardless of caller.
    // Returns the new point's index on success.
    int addPoint(double inputDb, double outputDb)
    {
        if (count_ >= kMaxPoints) return -1;
        inputDb = std::max(kMinDb, std::min(kMaxDb, inputDb));
        outputDb = std::max(kMinDb, std::min(kMaxDb, outputDb));

        int idx = 1;
        while (idx < count_ && points_[idx].inputDb < inputDb) ++idx;
        if (inputDb - points_[idx - 1].inputDb < kMinSeparationDb) return -1;
        if (idx < count_ && points_[idx].inputDb - inputDb < kMinSeparationDb) return -1;

        for (int i = count_; i > idx; --i) points_[i] = points_[i - 1];
        points_[idx] = { inputDb, outputDb };
        ++count_;
        rebuild();
        return idx;
    }

    // Removes an interior point. Refuses (no-op) for the two fixed
    // endpoints (index 0 or count()-1) or an out-of-range index -- "fixed
    // left right points" never leave, by construction, not by GUI
    // discipline alone.
    void removePoint(int index)
    {
        if (index <= 0 || index >= count_ - 1) return;
        for (int i = index; i < count_ - 1; ++i) points_[i] = points_[i + 1];
        --count_;
        normalizeLinks();
        rebuild();
    }

    // Moves a point. For the two fixed endpoints, inputDb is silently
    // ignored (X never moves, see the class's own header comment) and
    // only outputDb applies -- callers (the graph widget's own drag
    // handling) don't need to special-case which function to call by
    // index, this handles both cases. For an interior point, inputDb is
    // normally clamped strictly between its immediate neighbors (minus/
    // plus kMinSeparationDb), so points can never cross or reorder --
    // except that landing within kMinSeparationDb of a neighbor SNAPS
    // flush to that neighbor's own inputDb instead, forming a step pair
    // (see this class's own header comment and evaluate()'s own comment).
    void setPoint(int index, double inputDb, double outputDb)
    {
        if (index < 0 || index >= count_) return;
        outputDb = std::max(kMinDb, std::min(kMaxDb, outputDb));
        if (index == 0 || index == count_ - 1)
        {
            points_[index].outputDb = points_[index].diagonal ? points_[index].inputDb : outputDb;
            rebuild();
            return;
        }

        const double loNeighbor = points_[index - 1].inputDb;
        const double hiNeighbor = points_[index + 1].inputDb;
        // Refuse to snap on whichever side is already itself the far half
        // of a *different* step pair -- would otherwise stack three
        // points at one X, which evaluate() isn't built to resolve (only
        // pairs).
        const bool loTaken = index >= 2 && points_[index - 1].inputDb == points_[index - 2].inputDb;
        const bool hiTaken = index + 2 <= count_ - 1 && points_[index + 1].inputDb == points_[index + 2].inputDb;

        const double distLo = inputDb - loNeighbor;
        const double distHi = hiNeighbor - inputDb;
        bool snapLo = !loTaken && distLo < kMinSeparationDb;
        bool snapHi = !hiTaken && distHi < kMinSeparationDb;
        if (snapLo && snapHi)
        {
            // Both neighbors in range at once (pathologically close
            // together already) -- snap to whichever is nearer.
            if (distLo <= distHi) snapHi = false; else snapLo = false;
        }

        if (snapLo)
            points_[index].inputDb = loNeighbor;
        else if (snapHi)
            points_[index].inputDb = hiNeighbor;
        else
            points_[index].inputDb = std::max(loNeighbor + kMinSeparationDb, std::min(hiNeighbor - kMinSeparationDb, inputDb));
        // A diagonal-snapped point's Y is never independently settable --
        // see CurvePoint::diagonal's own comment -- so whatever outputDb the
        // caller asked for is overridden to match the (just-clamped) X.
        points_[index].outputDb = points_[index].diagonal ? points_[index].inputDb : outputDb;
        rebuild();
    }

    // Replaces the *entire* point list at once (session/state restore,
    // or a preset load) -- validated as a whole rather than replayed
    // through addPoint() one at a time, so a well-formed saved curve
    // loads back exactly as saved regardless of point order fed in.
    // Requires: 2 <= n <= kMaxPoints, strictly ascending inputDb with at
    // least kMinSeparationDb between every adjacent pair. On any
    // violation, returns false and leaves the curve completely untouched
    // (defends against a hand-edited or foreign/future-version file the
    // same way EqModel::fromJson() clamps rather than trusts its input).
    // The first/last point's own inputDb is snapped to kMinDb/kMaxDb
    // regardless of what was stored, since those two are always fixed
    // endpoints by definition.
    // thresholdLink: the threshold's link group to apply together with the
    // points (-1 = keep the current one). It has to be applied *before* the
    // link groups are normalized, or a node linked only to the threshold
    // would look like a lone group member and be dissolved.
    bool loadPoints(const CurvePoint* pts, int n, int thresholdLink = -1)
    {
        if (pts == nullptr || n < 2 || n > kMaxPoints) return false;
        // Migration of a curve saved with the old -60 dB window (its left end at x = -60, not at kMinDb): below its left
        // end a curve always HELD the gain of that end (out - in), so exactly the same behaviour in the new window is a
        // new left end at kMinDb carrying that same gain, with the old end kept as an ordinary node. A plain 2-point
        // straight line stays 2 points (the new end simply lies on it).
        if (pts[0].inputDb > kMinDb + 1e-9 && pts[0].inputDb <= kLegacyMinDb + 1e-9)
        {
            const double g0 = pts[0].outputDb - pts[0].inputDb;
            CurvePoint tmp[kMaxPoints];
            const bool straight = n == 2 && pts[0].handleInputDb == 0.0 && pts[0].handleOutputDb == 0.0 &&
                                  pts[1].handleInputDb == 0.0 && pts[1].handleOutputDb == 0.0 &&
                                  std::fabs((pts[1].outputDb - pts[1].inputDb) - g0) < 1e-9;
            const double newEndOut = std::max(kMinDb, std::min(kMaxDb, kMinDb + g0));
            if (straight)
            {
                tmp[0] = pts[0]; tmp[0].inputDb = kMinDb; tmp[0].outputDb = newEndOut;
                tmp[1] = pts[1];
                return loadPoints(tmp, 2, thresholdLink);
            }
            if (n < kMaxPoints)
            {
                tmp[0] = CurvePoint(); tmp[0].inputDb = kMinDb; tmp[0].outputDb = newEndOut;
                for (int i = 0; i < n; ++i) tmp[i + 1] = pts[i];
                tmp[1].direction = PointDirection::Both;
                return loadPoints(tmp, n + 1, thresholdLink);
            }
            // (already full: fall through -- the old end is snapped to the new window's edge)
        }
        for (int i = 1; i < n; ++i)
        {
            const double dx = pts[i].inputDb - pts[i - 1].inputDb;
            if (dx < 0.0) return false; // must stay sorted
            // Exactly 0 is a valid step pair (see this class's own header
            // comment); anything strictly between 0 and kMinSeparationDb
            // is refused the same as before -- too close to be a safe
            // continuous segment, not deliberately equal enough to be a
            // step either.
            if (dx > 0.0 && dx < kMinSeparationDb) return false;
            if (dx == 0.0 && i >= 2 && pts[i - 1].inputDb == pts[i - 2].inputDb) return false; // no 3-way stack at one X
        }

        count_ = n;
        for (int i = 0; i < n; ++i)
        {
            points_[i].inputDb = std::max(kMinDb, std::min(kMaxDb, pts[i].inputDb));
            points_[i].outputDb = std::max(kMinDb, std::min(kMaxDb, pts[i].outputDb));
            // Endpoints always Both -- see PointDirection's own comment.
            points_[i].direction = (i == 0 || i == n - 1) ? PointDirection::Both : pts[i].direction;
            points_[i].handleInputDb = pts[i].handleInputDb;
            points_[i].handleOutputDb = pts[i].handleOutputDb;
            points_[i].link = std::max(0, std::min(kMaxLinkGroups, pts[i].link));
            points_[i].linkAxis = points_[i].link > 0 ? std::max(0, std::min(kLinkAxisY, pts[i].linkAxis)) : 0;
            points_[i].diagonal = pts[i].diagonal;
            // Enforce the invariant regardless of what a foreign/hand-edited
            // source stored -- a loaded point never sits off the diagonal
            // while flagged, the same guarantee setPoint() gives a live edit.
            if (points_[i].diagonal) points_[i].outputDb = points_[i].inputDb;
        }
        points_[0].inputDb = kMinDb;
        points_[n - 1].inputDb = kMaxDb;
        // Re-clamp every handle's own X offset against the spacing this
        // load just established -- a foreign/older saved file's stored
        // handle could otherwise violate the same X-monotonicity
        // constraint setPointHandle() enforces on a live edit (see that
        // method's own comment); this reuses the identical clamp so a
        // loaded curve can never silently double back on X regardless of
        // where it came from.
        reclampHandles();
        if (thresholdLink >= 0) setThresholdLink(thresholdLink);
        normalizeLinks();
        rebuild();
        return true;
    }

    // Sets an Inkscape-style symmetric tangent handle -- see this file's
    // own header comment on "per-point interpolation" for the full
    // design. Works on ANY point, including the two fixed endpoints
    // (unlike setPointDirection()) -- a handle shapes how the curve
    // *approaches* an endpoint, which is meaningful even though the
    // endpoint's own (X,Y) position can't move.
    //
    // handleInputDb (the X component) is clamped to [0, min(distance to
    // the left neighbor, distance to the right neighbor)] -- the one
    // constraint evaluateFor()'s own Bezier solve actually depends on:
    // the resulting control point can never sit behind this point or
    // past its neighbor's own X, which is what guarantees the segment's
    // X(t) stays monotonic (a valid function of X, never doubling back --
    // see evaluateFor()'s own comment). handleOutputDb (the Y component)
    // is deliberately left unclamped: letting a hand-placed handle
    // overshoot past a neighboring point's own value on purpose is the
    // entire reason to use a manual Bezier handle instead of the old
    // automatic monotone spline this replaces.
    void setPointHandle(int index, double handleInputDb, double handleOutputDb)
    {
        if (index < 0 || index >= count_) return;
        points_[index].handleInputDb = clampReach(handleInputDb, maxHandleDx(index));
        points_[index].handleOutputDb = handleOutputDb;
        rebuild();
    }

    // Limits |dx| to `maxDx`, keeping its sign (see CurvePoint::handleInputDb).
    static double clampReach(double dx, double maxDx)
    {
        const double a = std::min(maxDx, std::fabs(dx));
        return dx < 0.0 ? -a : a;
    }
    // The handle as evaluate() uses it: dx >= 0 always; when dx was negative
    // the vector is flipped (d and -d are the same tangent line).
    static double canonHandleX(const CurvePoint& p) { return std::fabs(p.handleInputDb); }
    static double canonHandleY(const CurvePoint& p) { return p.handleInputDb < 0.0 ? -p.handleOutputDb : p.handleOutputDb; }

    // The largest X reach node `index`'s support point may have: the gap to
    // its nearest neighbor on each side *that it can actually conflict
    // with*. Rising-only and Falling-only nodes are never on the same
    // directional view (see computeDirectionalViews()), so a Falling node's
    // handle is not limited by an adjacent Rising node (and vice versa). A
    // Both node (and the two endpoints)
    // appears in both views, so it still stops at any adjacent neighbor.
    double maxHandleDx(int index) const
    {
        auto dirOf = [&](int j) { return (j == 0 || j == count_ - 1) ? PointDirection::Both : points_[j].direction; };
        const PointDirection d = dirOf(index);
        auto shareView = [&](int j) {
            const PointDirection e = dirOf(j);
            return d == PointDirection::Both || e == PointDirection::Both || d == e;
        };
        double maxDx = 1.0e18;
        for (int j = index - 1; j >= 0; --j)
            if (shareView(j)) { maxDx = std::min(maxDx, points_[index].inputDb - points_[j].inputDb); break; }
        for (int j = index + 1; j < count_; ++j)
            if (shareView(j)) { maxDx = std::min(maxDx, points_[j].inputDb - points_[index].inputDb); break; }
        return maxDx;
    }

    // Moves node `index` -- and, if it is linked, every other member of its
    // group (the threshold included, on X only) -- by the same (dIn, dOut).
    // Used where the position of one node is EDITED numerically (the node
    // list), so linking also holds for typed edits. Members move in the direction-of-travel
    // order (right-most first when moving right, else left-most first), so a
    // member is never clamped against a neighbour that simply hasn't moved
    // yet. Every member is still clamped individually by setPoint(), exactly
    // like a group drag in the graph. thresholdDb is updated in place (and
    // clamped to the curve's range) when the threshold is in the group.
    void moveLinked(int index, double dIn, double dOut, double& thresholdDb)
    {
        if (index < 0 || index >= count_) return;
        const int lk = points_[index].link;
        int order[kMaxPoints];
        int n = 0;
        for (int i = 0; i < count_; ++i)
            if (i == index || (lk > 0 && points_[i].link == lk)) order[n++] = i;
        if (dIn > 0.0) std::reverse(order, order + n);
        for (int k = 0; k < n; ++k)
        {
            const CurvePoint q = points_[order[k]];
            // The edited node moves freely; the others get only the axis the
            // group is linked on.
            const bool self = order[k] == index;
            const int ax = q.linkAxis;
            const double dx = (self || ax != kLinkAxisY) ? dIn : 0.0;
            const double dy = (self || ax != kLinkAxisX) ? dOut : 0.0;
            setPoint(order[k], q.inputDb + dx, q.outputDb + dy);
        }
        if (lk > 0 && thresholdLink_ == lk)
            thresholdDb = std::max(kMinDb, std::min(kMaxDb, thresholdDb + dIn)); // the threshold is only ever in X-carrying groups
    }

    // Re-applies maxHandleDx() to every stored handle -- after a load, or
    // after a direction change (which changes who conflicts with whom).
    void reclampHandles()
    {
        for (int i = 0; i < count_; ++i)
            points_[i].handleInputDb = clampReach(points_[i].handleInputDb, maxHandleDx(i));
    }

    // Link groups -- see CurvePoint::link. setPointLink() deliberately does
    // not normalize (linking N nodes is N calls; a half-built group must not
    // be dissolved between them) -- call normalizeLinks() once afterward.
    // removePoint()/loadPoints() normalize themselves.
    void setPointLink(int index, int link, int axis = 0)
    {
        if (index < 0 || index >= count_) return;
        points_[index].link = std::max(0, std::min(kMaxLinkGroups, link));
        points_[index].linkAxis = points_[index].link > 0 ? std::max(0, std::min(kLinkAxisY, axis)) : 0;
    }
    int pointLinkAxis(int index) const
    {
        if (index < 0 || index >= count_) return 0;
        return points_[index].link > 0 ? points_[index].linkAxis : 0;
    }
    int pointLink(int index) const
    {
        if (index < 0 || index >= count_) return 0;
        return points_[index].link;
    }
    // "Snap to Diagonal" (right-click menu): pins the point onto the unity
    // line, from then on with only one degree of freedom -- see
    // CurvePoint::diagonal's own comment. Turning it on re-snaps the point
    // immediately (through setPoint(), so it goes through the same X-clamp
    // and neighbor-snap rules as a live drag); turning it off just drops the
    // flag and leaves the point exactly where it already was.
    void setPointDiagonal(int index, bool on)
    {
        if (index < 0 || index >= count_) return;
        points_[index].diagonal = on;
        if (on) setPoint(index, points_[index].inputDb, points_[index].inputDb);
    }
    bool pointDiagonal(int index) const
    {
        if (index < 0 || index >= count_) return false;
        return points_[index].diagonal;
    }
    // The Threshold can join a link group too ("the threshold should be
    // linkable, but only on x-axis then"): it moves with the group's nodes by
    // the group's X movement only. It is not a CurvePoint (it has no Y and is
    // a separate Parameter), so its group id is kept here, per curve, and
    // travels with the curve string / mirror like a point's own link does.
    void setThresholdLink(int link) { thresholdLink_ = std::max(0, std::min(kMaxLinkGroups, link)); }
    int thresholdLink() const { return thresholdLink_; }
    // Dissolves any group left with fewer than 2 members (the threshold
    // counts as a member).
    void normalizeLinks()
    {
        int members[kMaxLinkGroups + 1] = {};
        for (int i = 0; i < count_; ++i) ++members[std::max(0, std::min(kMaxLinkGroups, points_[i].link))];
        ++members[thresholdLink_];
        for (int i = 0; i < count_; ++i)
            if (points_[i].link != 0 && members[points_[i].link] < 2) points_[i].link = 0;
        if (thresholdLink_ != 0 && members[thresholdLink_] < 2) thresholdLink_ = 0;
        for (int i = 0; i < count_; ++i)
            if (points_[i].link == 0) points_[i].linkAxis = 0;
        // The threshold has no Y: it cannot be in a Y-only group.
        if (thresholdLink_ != 0)
            for (int i = 0; i < count_; ++i)
                if (points_[i].link == thresholdLink_ && points_[i].linkAxis == kLinkAxisY) { thresholdLink_ = 0; break; }
    }
    // Smallest group id not used by any point or the threshold (0 if all are
    // taken -- not reachable with 18 points, every group needing two).
    int freeLinkId() const
    {
        bool used[kMaxLinkGroups + 1] = {};
        for (int i = 0; i < count_; ++i) used[points_[i].link] = true;
        used[thresholdLink_] = true;
        for (int g = 1; g <= kMaxLinkGroups; ++g) if (!used[g]) return g;
        return 0;
    }

    // Tags an interior point's own hysteresis direction -- see
    // PointDirection's own comment. No-op (like removePoint()) for the two
    // fixed endpoints or an out-of-range index -- they're always Both, by
    // construction, not by GUI discipline alone.
    void setPointDirection(int index, PointDirection dir)
    {
        if (index <= 0 || index >= count_ - 1) return;
        points_[index].direction = dir;
        reclampHandles(); // who conflicts with whom just changed
        rebuild();
    }
    PointDirection pointDirection(int index) const
    {
        if (index < 0) index = 0;
        if (index >= count_) index = count_ - 1;
        if (index == 0 || index == count_ - 1) return PointDirection::Both;
        return points_[index].direction;
    }

    // Recomputes the two direction-filtered point views
    // evaluateDirectional() reads (see that method's own comment) --
    // every mutator above already calls this itself. GUI/message-thread-
    // only, same as before -- never runs on the audio thread. (There's no
    // tangent array to precompute any more -- see this file's own header
    // comment on why the old monotone-Hermite pass is gone.)
    void rebuild()
    {
        // Every mutator ends here, so every stored handle is always within its
        // limit (maxHandleDx()) after ANY edit. Before, moving a node close to
        // a neighbour left that node's stored reach untouched in the GUI's copy
        // while the DSP re-clamped it whenever it loaded the curve -- the two
        // curves then differed (measured: up to 1.7dB) and the trail dots sat
        // visibly off the drawn line after such an edit. Now the model the GUI
        // draws is exactly what the DSP loads. (Cost: squeezing a node against
        // its neighbour shortens its reach for good, the same as loading it.)
        reclampHandles();
        computeDirectionalViews();
    }

    // Input dB -> output dB. Flat extrapolation beyond the two endpoints
    // (returns that endpoint's own outputDb) rather than continuing
    // whatever slope the nearest segment had -- a level past the drawn
    // range has nothing drawn to say what should happen to it, so "do
    // whatever the nearest edge already does" is the least surprising
    // choice. O(count_) segment scan (count_ <= kMaxPoints = 18, cheaper
    // and simpler here than a binary search) + O(1) evaluation.
    //
    // A zero-width segment (two adjacent points sharing the same inputDb
    // -- a step/gate pair, see this class's own header comment) is never
    // itself evaluated as a segment -- the scan below (">=", not just ">")
    // always steps past it, landing on the segment *after* the pair for
    // any input at or above the shared X, and the segment *before* it for
    // anything strictly below -- a ">=" condition in the interpolation:
    // the pair's later (upper-index) point
    // governs at and above the shared X, its earlier point governs the
    // approach from below, a genuine discontinuity right at that X.
    inline double evaluate(double inputDb) const
    {
        return evaluateFor(points_, count_, inputDb);
    }

    // The direction-aware counterpart (two different paths up and down).
    // `rising` picks which of the two views
    // computeDirectionalViews() already assembled (on the last edit, not
    // now -- this is the cheap audio-thread path) to evaluate against:
    // every Both-tagged point (which, on an untagged curve, is every
    // point) is in both views, so this is bit-for-bit identical to plain
    // evaluate() whenever nothing's actually tagged -- a RisingOnly point
    // simply isn't in the Falling view (and vice versa), so that
    // direction's own interpolation runs straight past it between its
    // two neighbors instead, exactly as if it weren't there at all for
    // that direction.
    inline double evaluateDirectional(double inputDb, bool rising) const
    {
        return rising ? evaluateFor(activeRising_, activeRisingCount_, inputDb)
                       : evaluateFor(activeFalling_, activeFallingCount_, inputDb);
    }

    // The direction-filtered point set itself -- exposed purely so the
    // GUI can force a sample at every point actually visible in a given
    // direction's own path (the same "force a sample at every control
    // point's own position" reasoning ContourGraphWidget::
    // drawTransferCurve() already applies to the plain curve, extended to
    // each of the two direction-specific paths it now also draws -- see
    // that function's own comment). Equal to pointCount()/point() on an
    // untagged curve (every point is in both views).
    int activePointCount(bool rising) const { return rising ? activeRisingCount_ : activeFallingCount_; }
    CurvePoint activePoint(bool rising, int index) const
    {
        const CurvePoint* pts = rising ? activeRising_ : activeFalling_;
        const int n = rising ? activeRisingCount_ : activeFallingCount_;
        if (index < 0) index = 0;
        if (index >= n) index = n - 1;
        return pts[index];
    }

private:
    // Cubic Bezier: point(t) = (1-t)^3*P0 + 3(1-t)^2*t*P1 + 3(1-t)t^2*P2 + t^3*P3.
    static inline double bezierAt(double p0, double p1, double p2, double p3, double t)
    {
        const double mt = 1.0 - t;
        return mt * mt * mt * p0 + 3.0 * mt * mt * t * p1 + 3.0 * mt * t * t * p2 + t * t * t * p3;
    }
    static inline double bezierDerivAt(double p0, double p1, double p2, double p3, double t)
    {
        const double mt = 1.0 - t;
        return 3.0 * mt * mt * (p1 - p0) + 6.0 * mt * t * (p2 - p1) + 3.0 * t * t * (p3 - p2);
    }

    // One segment, point i -> point i+1 -- see this file's own header
    // comment on "per-point interpolation" for the handle model. p0/p1
    // are point i's own (input,output,handleInput,handleOutput); p2/p3
    // are point i+1's. A corner-to-corner segment (both handles exactly
    // zero -- the common, default case for any freshly-added point) is
    // evaluated as an exact straight line, no Bezier/root-solve involved
    // at all -- both for speed and so "one click makes a linear
    // interpolated node" is bit-exact, not merely visually close.
    //
    // Otherwise, the two Bezier control points are each point's own
    // stored handle offset, defensively re-clamped into [x0,x1] on the X
    // axis regardless of what's stored -- setPointHandle() and
    // loadPoints() already enforce this at the point where a handle is
    // set, but a direction-filtered view (evaluateDirectional()) can
    // stretch a segment across a *different*, wider gap than the one a
    // handle's own X offset was originally clamped against (its usual
    // immediate neighbor may have been excluded from this particular
    // view) -- this second clamp, applied at the point of actual use,
    // guarantees every segment this class ever evaluates stays a valid
    // function of X (monotonic, never doubling back), regardless of
    // which view is asking.
    //
    // Given valid (monotonic) control points, X(t) is itself monotonic
    // non-decreasing over t in [0,1], so solving X(t) == inputDb for t
    // via Newton-Raphson is guaranteed to converge -- a fixed, small
    // iteration count (not a data-dependent while-loop) keeps this safely
    // bounded for the audio thread, same as every other per-sample cost
    // in this project. The starting guess (the linear estimate) is
    // already close for any practical handle length, so in practice this
    // converges in 2-4 iterations well inside the 8-iteration cap.
    static double evaluateSegment(double x0, double y0, double hx0, double hy0,
                                   double x1, double y1, double hx1, double hy1,
                                   double inputDb)
    {
        if (hx0 == 0.0 && hy0 == 0.0 && hx1 == 0.0 && hy1 == 0.0)
        {
            const double h = std::max(1.0e-9, x1 - x0);
            const double t = (inputDb - x0) / h;
            return y0 + t * (y1 - y0);
        }

        const double p1x = std::max(x0, std::min(x1, x0 + hx0));
        const double p1y = y0 + hy0;
        const double p2x = std::max(x0, std::min(x1, x1 - hx1));
        const double p2y = y1 - hy1;

        // Solve X(t) == inputDb. X is monotone in t (see setPointHandle()), so a
        // bracket [lo,hi] always contains the root; Newton steps are accepted
        // only while they stay inside it, otherwise the step falls back to
        // bisection. Plain Newton capped at 8 iterations (what this used to be)
        // could miss by over a dB on extreme handles (measured on random curves)
        // -- where X'(t) is nearly 0 at a segment end. The loop stays bounded
        // (real-time safe): 24 iterations at most, and 2-4 in practice.
        double lo = 0.0, hi = 1.0;
        double t = std::max(0.0, std::min(1.0, (inputDb - x0) / std::max(1.0e-9, x1 - x0)));
        for (int iter = 0; iter < 24; ++iter)
        {
            const double xt = bezierAt(x0, p1x, p2x, x1, t);
            const double err = xt - inputDb;
            if (std::fabs(err) < 1.0e-9) break;
            if (err > 0.0) hi = t; else lo = t;
            const double dxt = bezierDerivAt(x0, p1x, p2x, x1, t);
            double tNext = (std::fabs(dxt) > 1.0e-9) ? t - err / dxt : 0.5 * (lo + hi);
            if (!(tNext > lo && tNext < hi)) tNext = 0.5 * (lo + hi); // Newton left the bracket: bisect
            const bool converged = std::fabs(tNext - t) < 1.0e-9;
            t = tNext;
            if (converged) break;
        }
        return bezierAt(y0, p1y, p2y, y1, t);
    }

    // Input dB -> output dB over a given point set. Flat extrapolation
    // beyond the two endpoints (returns that endpoint's own outputDb)
    // rather than continuing whatever slope the nearest segment had -- a
    // level past the drawn range has nothing drawn to say what should
    // happen to it, so "do whatever the nearest edge already does" is
    // the least surprising choice. O(n) segment scan (n <= kMaxPoints =
    // 18, cheaper and simpler here than a binary search) + the bounded
    // per-segment solve above.
    //
    // A zero-width segment (two adjacent points sharing the same inputDb
    // -- a step/gate pair, see this class's own header comment) is never
    // itself evaluated as a segment -- the scan below (">=", not just ">")
    // always steps past it, landing on the segment *after* the pair for
    // any input at or above the shared X, and the segment *before* it for
    // anything strictly below -- a ">=" condition in the interpolation:
    // the pair's later (upper-index) point
    // governs at and above the shared X, its earlier point governs the
    // approach from below, a genuine discontinuity right at that X.
    inline double evaluateFor(const CurvePoint* pts, int n, double inputDb) const
    {
        if (inputDb <= pts[0].inputDb) return pts[0].outputDb;
        if (inputDb >= pts[n - 1].inputDb) return pts[n - 1].outputDb;

        int i = 0;
        while (i < n - 2 && inputDb >= pts[i + 1].inputDb) ++i;

        return evaluateSegment(pts[i].inputDb, pts[i].outputDb, canonHandleX(pts[i]), canonHandleY(pts[i]),
                                pts[i + 1].inputDb, pts[i + 1].outputDb, canonHandleX(pts[i + 1]), canonHandleY(pts[i + 1]),
                                inputDb);
    }

    // Builds activeRising_/activeFalling_ from points_/direction --
    // called from rebuild(), see that method's own comment. A point is
    // in the Rising view unless it's tagged FallingOnly, and in the
    // Falling view unless it's tagged RisingOnly; the two fixed endpoints
    // are in both regardless of their own (always-Both, per
    // setPointDirection()'s own refusal) tag, so both views always have
    // somewhere to anchor their own ends.
    void computeDirectionalViews()
    {
        activeRisingCount_ = 0;
        activeFallingCount_ = 0;
        for (int i = 0; i < count_; ++i)
        {
            const bool isEnd = (i == 0 || i == count_ - 1);
            const PointDirection d = points_[i].direction;
            if (isEnd || d != PointDirection::FallingOnly) activeRising_[activeRisingCount_++] = points_[i];
            if (isEnd || d != PointDirection::RisingOnly) activeFalling_[activeFallingCount_++] = points_[i];
        }
    }

    CurvePoint points_[kMaxPoints];
    int count_ = 2;
    int thresholdLink_ = 0; // see setThresholdLink()

    // See evaluateDirectional()'s and computeDirectionalViews()'s own
    // comments. Sized/shaped identically to points_ -- on an untagged
    // curve these end up an exact copy of points_/count_ (every point
    // survives both filters), so evaluateDirectional() costs nothing
    // extra in behavior, only in the (still O(count_), GUI-thread-only)
    // rebuild() work to keep them current.
    CurvePoint activeRising_[kMaxPoints];
    int activeRisingCount_ = 2;
    CurvePoint activeFalling_[kMaxPoints];
    int activeFallingCount_ = 2;
};

// The curve as an input -> output mapping, expressed as a gain change in dB for a signal at `levelDb`:
// curve(level) - level. Below the curve's own left end (kMinDb) the level has nothing drawn to say what to do.
// evaluate() extrapolates FLAT there, which as a gain means "push every quieter sample up to the left end's output
// level" -- with an instant detector (0ms attack/release) that boosted every sample near a zero-crossing (< -60dB, a
// fixed slice of every cycle) up to -60dB even on a perfectly identity curve: a crossover-style crunch (measured:
// -55dB error floor on an untouched identity curve). So below the left end the gain HOLDS what it is at that end
// (unity slope, output = level + (endOut - endIn)) instead. Above the right end the flat extrapolation stays: it is a
// ceiling, and a hot level there is rare and not a crossing artifact.
inline double curveGainDb(double levelDb, const TransferCurve& curve, bool rising = true)
{
    const CurvePoint lo = curve.point(0);
    return (levelDb < lo.inputDb) ? lo.outputDb - lo.inputDb : curve.evaluateDirectional(levelDb, rising) - levelDb;
}

// Gain *change* in dB for detected level `levelDb`, given the curve and
// the independent threshold/condition -- the direct curve-based
// counterpart of computeGainReductionDb() (CompressorDsp.hpp). 0.0 (no
// change) whenever the trigger condition isn't met, regardless of what
// the curve says there -- see this file's own header comment on why the
// threshold is deliberately not "part of" the curve.
// `rising` defaults to true so every existing call site (and every
// existing dsp_regression case) that doesn't know about hysteresis yet
// keeps behaving exactly as before -- provably so, since an untagged
// curve's Rising and Falling views are identical (see
// TransferCurve::evaluateDirectional()'s own comment), making this
// default a genuine no-op rather than an arbitrary guess.
inline double computeCurveGainDb(double levelDb, double thresholdDb, TriggerCondition cond, const TransferCurve& curve, bool rising = true,
                                  TriggerAction action = TriggerAction::ApplyCurve)
{
    const bool triggered = (cond == TriggerCondition::Above) ? (levelDb > thresholdDb) : (levelDb < thresholdDb);
    // Where the curve is in charge: the trigger region (ApplyCurve) or everything but it (ReleaseToDiagonal).
    const bool curveActive = (action == TriggerAction::ApplyCurve) ? triggered : !triggered;
    if (!curveActive) return 0.0;
    return curveGainDb(levelDb, curve, rising);
}

// ---- one curve-driven voice --------------------------------------------
//
// The whole plugin in one sentence: the curve is an input -> output mapping for the AUDIO; the trigger only decides
// whether the audio takes the curve or the diagonal (unchanged); Attack/Release make that switch smooth.
//
//   audio level ---------------------------> curve gain  (curve(level) - level, read at the AUDIO's own level)
//   trigger signal -> level -> threshold ->  "trigger met?" -> which of {curve, diagonal} is wanted
//                                             -> smoothed by Attack (trigger begins) / Release (trigger ends)
//   gain = switch * curve gain               (switch 0 = diagonal / no change, 1 = the curve)
//
// process() takes BOTH signals: `triggerAbs` (what the trigger sees: after Trigger Gain, the Trigger Filter, the
// sidechain choice) and `audioAbs` (the audio this voice controls, as the owner delays it for lookahead). With the
// default trigger (main input, no gain, no filter) the two are the same signal; the trigger's own gain/filter/
// sidechain never change WHERE on the curve the audio sits, only when the switch flips. (Before, the curve was
// read at the trigger's level, so a lowered Trigger Gain or a narrow filter slid the audio along the curve --
// dots off the drawn line, a gate that could not close under -60 dB, ...)
//
// Levels: the trigger is read by a fast peak detector (instant attack; release = the Release time, capped at kTriggerReleaseMs
// -- long enough to bridge the half-cycles of a low tone, short enough not to add a noticeable hold; at Release 0 there is no
// hold at all, so with the trigger equal to the audio the threshold is compared with exactly the level the curve is read at,
// sample by sample, and the dots jump cleanly at the threshold). The audio's level for the curve
// is a peak follower too: instant attack (a hit's level is known from its first sample, so a gate cannot leak its
// onset), and the Release time to fall -- which at Release 0 is the instantaneous sample, the curve applied sample by
// sample ("the audio should follow the line 100%"). Attack is deliberately NOT used for the level: it belongs to the
// switch (below).
//
// "Attack"/"Release" always refer to the TRIGGER: Attack = how fast the sound moves to its new state (curve or
// diagonal, whichever the action makes of it) when the trigger starts being met; Release = how fast it moves back
// once the trigger ends. The same reading for both actions.
//
// Lookahead (the owner delays the AUDIO by lookaheadSamples_ and feeds this voice the un-delayed trigger): the
// start of the trigger is taken at once -- lookaheadSamples_ before the audio it belongs to arrives -- and its end is
// held that long, so the audio it triggered is fully covered and never released early.
class CurveVoice
{
public:
    static constexpr double kTriggerReleaseMs = 8.0;

    void prepare(double sampleRate)
    {
        sampleRate_ = std::max(1.0, sampleRate);
        triggerEnv_.prepare(sampleRate_);
        triggerEnv_.setTimesMs(0.0, std::min(releaseMs_, kTriggerReleaseMs));
        audioEnv_.prepare(sampleRate_);
        updateSwitchCoeffs();
    }

    // Samples of lookahead the owner delays its audio by (0 = off) -- see the class comment.
    void setLookaheadSamples(int n) { lookaheadSamples_ = n < 0 ? 0 : n; }

    void setParams(double attackMs, double releaseMs, double thresholdDb, TriggerCondition cond, TriggerAction action = TriggerAction::ApplyCurve)
    {
        action_ = action;
        triggerEnv_.setTimesMs(0.0, std::min(releaseMs, kTriggerReleaseMs)); // the trigger's own hold: at Release 0 none at all (see the class comment)
        audioEnv_.setTimesMs(0.0, releaseMs); // instant attack: the curve must see a hit's level from its first sample (a gate must not leak the onset)
        attackMs_ = attackMs;
        releaseMs_ = releaseMs;
        updateSwitchCoeffs();
        thresholdDb_ = thresholdDb;
        cond_ = cond;
    }

    // One sample of each rectified signal; returns the linear gain multiplier for the audio sample(s) this voice controls.
    inline double process(double triggerAbs, double audioAbs, const TransferCurve& curve)
    {
        // ---- the trigger: level -> threshold -> met? ----
        detectorLevelDb_ = linearToDb(triggerEnv_.process(triggerAbs));
        triggered_ = (cond_ == TriggerCondition::Above) ? (detectorLevelDb_ > thresholdDb_) : (detectorLevelDb_ < thresholdDb_);
        bool met = triggered_;
        if (lookaheadSamples_ > 0)
        {
            if (met) holdLeft_ = lookaheadSamples_;
            else if (holdLeft_ > 0) { --holdLeft_; met = true; }
        }

        // ---- the switch: 1 = the curve, 0 = the diagonal; Attack when the trigger begins, Release when it ends ----
        const double switchTarget = ((action_ == TriggerAction::ApplyCurve) == met) ? 1.0 : 0.0;
        const double coeff = met ? switchAttackCoeff_ : switchReleaseCoeff_;
        switch_ = coeff * switch_ + (1.0 - coeff) * switchTarget;

        // ---- the audio's own level and the curve read at it ----
        // Rising/falling-only nodes: direction is judged on
        // the enveloped level the curve is read at (the raw waveform has no meaningful direction at audio rate); a flat
        // run keeps the direction it had, like a Schmitt trigger.
        audioLevelDb_ = linearToDb(audioEnv_.process(audioAbs));
        if (audioLevelDb_ > prevAudioLevelDb_) rising_ = true;
        else if (audioLevelDb_ < prevAudioLevelDb_) rising_ = false;
        prevAudioLevelDb_ = audioLevelDb_;

        // The curve's gain is smoothed on its way BACK only: a deeper cut is taken at once (a gate must not leak a hit's
        // onset), a shallower one relaxes with the Release time. Without this the gain follows the level envelope's ripple
        // (its release is only the level follower's) directly, and even a smooth curve modulates the sound every cycle of a
        // low tone -- measured: 20-50 dB more distortion than the earlier design that smoothed the gain. At Release 0 it is
        // sample-exact (the curve applied per sample). The tracking also runs while the switch is at the diagonal, so the
        // moment the switch moves to the curve the gain is already where the curve puts it.
        const double target = curveGainDb(audioLevelDb_, curve, rising_);
        const double relax = (std::fabs(target) > std::fabs(curveGainSmoothedDb_)) ? 0.0 : switchReleaseCoeff_;
        curveGainSmoothedDb_ = relax * curveGainSmoothedDb_ + (1.0 - relax) * target;
        gainDb_ = switch_ * curveGainSmoothedDb_;
        return dbToLinear(gainDb_);
    }

    // Both signals are the same one (a plain detector -- what most tests and the old single-signal API mean).
    inline double process(double detectorAbs, const TransferCurve& curve) { return process(detectorAbs, detectorAbs, curve); }

    double gainDb() const { return gainDb_; }
    double detectorLevelDb() const { return detectorLevelDb_; } // the TRIGGER's level (what the threshold is compared to)
    double audioLevelDb() const { return audioLevelDb_; }
    double switchValue() const { return switch_; }              // 0 = diagonal .. 1 = the curve
    bool triggered() const { return triggered_; }               // trigger condition met at the last sample
    bool rising() const { return rising_; }
    void reset()
    {
        triggerEnv_.reset(); audioEnv_.reset();
        gainDb_ = 0.0; curveGainSmoothedDb_ = 0.0; detectorLevelDb_ = -120.0; audioLevelDb_ = -120.0; prevAudioLevelDb_ = -1.0e9;
        rising_ = true; triggered_ = false; holdLeft_ = 0;
        switch_ = (action_ == TriggerAction::ApplyCurve) ? 0.0 : 1.0; // the state with no trigger present
    }

private:
    // "0 means literally instant, not just fast" -- same rule as EnvelopeFollower::coeffFromMs().
    void updateSwitchCoeffs()
    {
        switchAttackCoeff_ = (attackMs_ <= 0.0) ? 0.0 : std::exp(-1.0 / (0.001 * attackMs_ * sampleRate_));
        switchReleaseCoeff_ = (releaseMs_ <= 0.0) ? 0.0 : std::exp(-1.0 / (0.001 * releaseMs_ * sampleRate_));
    }

    EnvelopeFollower triggerEnv_, audioEnv_;
    double sampleRate_ = 48000.0;
    double attackMs_ = 10.0, releaseMs_ = 150.0;
    double switchAttackCoeff_ = 0.0, switchReleaseCoeff_ = 0.0;
    double thresholdDb_ = -18.0;
    TriggerCondition cond_ = TriggerCondition::Above;
    TriggerAction action_ = TriggerAction::ApplyCurve;
    int lookaheadSamples_ = 0, holdLeft_ = 0;
    double switch_ = 0.0;
    double curveGainSmoothedDb_ = 0.0;
    double gainDb_ = 0.0;
    double detectorLevelDb_ = -120.0, audioLevelDb_ = -120.0;
    double prevAudioLevelDb_ = -1.0e9; // sentinel so the very first sample always reads as "rising" -- arbitrary but harmless
    bool rising_ = true;
    bool triggered_ = false;
};

// ---- plain-text serialization (DPF State value + preset file) --------
//
// Not JSON -- this project family's own PresetIO.hpp already established
// "no JSON parser needed" as a deliberate dependency-free preference for
// hand-rolled key=value scalar presets; a curve's own point list is
// hardly bigger, so the identical preference applies here: a small,
// hand-rolled text format instead of pulling in a parser for it. This
// doubles as both the actual DPF State value (see PluginContour.hpp's
// kStateCurve -- States are already free-form strings) and the .ctcp
// preset file's own "curve" entry, with no separate encoding needed for
// either.
//
// Format: "<marker><;in,out,dir,handleIn,handleOut,link>*" -- <marker> is a
// single leading byte: 'L' or, if the threshold is in a link group, that
// group's digit (see curveToString()); it used to be always 'L' (interpolation moved
// per-point, see this file's own header comment -- the byte is kept
// purely so this format's own layout, and every older reader/writer of
// it, doesn't need to shift; curveFromString() accepts and ignores
// whatever's there), followed by one ";in,out,dir,handleIn,handleOut"
// per point in ascending inputDb order (matching pointCount()'s own
// iteration order); dir is a single digit, see PointDirection; handleIn/
// handleOut are CurvePoint::handleInputDb/handleOutputDb; link is
// CurvePoint::link (older strings without it load as unlinked); the
// trailing diag (0/1) is CurvePoint::diagonal (older strings without it
// load as not snapped). %.6g is comfortably more precision than a -60..+6dB
// range ever needs while staying compact -- a full 18-point curve still
// fits well under 500 bytes.
inline std::string curveToString(const TransferCurve& c)
{
    std::string s;
    // Marker byte: 'L' (the historical value) = threshold not linked, else the
    // digit '1'..'9' of the threshold's link group. Older readers ignore the
    // byte, older data always has 'L'.
    s += c.thresholdLink() > 0 ? char('0' + c.thresholdLink()) : 'L';
    // std::to_chars, unlike snprintf's "%.6g", always emits '.' as the
    // decimal point regardless of the process's current LC_NUMERIC. That
    // matters doubly here: a host that adopts a comma-decimal OS locale
    // (GTK/Qt's setlocale(LC_ALL, "") at init) would otherwise make
    // snprintf write a *comma* into "%.6g,%.6g" -- colliding with this
    // format's own literal ',' field separator between fields, so
    // curveFromString()'s parsing below would split the wrong pair of
    // numbers apart entirely, not just misparse one of them.
    char buf[24];
    auto appendNum = [&](double v) { s += std::string(buf, std::to_chars(buf, buf + sizeof(buf), v).ptr); };
    for (int i = 0; i < c.pointCount(); ++i)
    {
        const CurvePoint p = c.point(i);
        s += ';'; appendNum(p.inputDb);
        s += ','; appendNum(p.outputDb);
        s += ','; s += char('0' + int(p.direction));
        s += ','; appendNum(p.handleInputDb);
        s += ','; appendNum(p.handleOutputDb);
        s += ','; appendNum(double(p.link + 10 * p.linkAxis)); // link group, +10 per axis step (0 both, 1 X, 2 Y)
        s += ','; appendNum(p.diagonal ? 1.0 : 0.0);
    }
    return s;
}

// Parses curveToString()'s own format back into `out`. Returns false
// (`out` left completely untouched) for anything malformed -- an empty/
// garbage string, or a point list TransferCurve::loadPoints() itself
// would refuse (too few/many points, not strictly ascending, or points
// closer together than kMinSeparationDb) -- same "validate as a whole,
// reject wholesale rather than partially apply" discipline loadPoints()
// itself already follows for a hand-edited or foreign/future-version
// saved file, extended one level up to cover the text format wrapping it
// too.
inline bool curveFromString(const std::string& s, TransferCurve& out)
{
    if (s.empty()) return false;
    // s[0] used to select curve-wide Linear/Spline; kept as a marker byte
    // for format-layout stability (see curveToString()'s own comment) but
    // no longer read for meaning -- interpolation is per-point now.

    CurvePoint pts[TransferCurve::kMaxPoints];
    int n = 0;
    size_t pos = 1;
    while (pos < s.size())
    {
        if (s[pos] != ';') return false;
        ++pos;
        // Each point is its own ';'-delimited segment: "in,out" (oldest
        // saved data, before hysteresis), "in,out,dir" (before Bezier
        // handles), or "in,out,dir,handleIn,handleOut" (current) -- all
        // three still parse, missing trailing fields just default (Both
        // direction, zero/corner handle), the same defaults a freshly-
        // added point already gets.
        const size_t segEnd = s.find(';', pos);
        const std::string seg = (segEnd == std::string::npos) ? s.substr(pos) : s.substr(pos, segEnd - pos);
        if (n >= TransferCurve::kMaxPoints) return false;

        std::string fields[7];
        int nf = 0;
        size_t start = 0;
        while (nf < 7)
        {
            const size_t c = seg.find(',', start);
            fields[nf++] = (c == std::string::npos) ? seg.substr(start) : seg.substr(start, c - start);
            if (c == std::string::npos) break;
            start = c + 1;
        }
        if (nf < 2) return false;
        // std::from_chars, unlike std::strtod, is locale-independent --
        // see curveToString()'s comment above for why that matters here.
        {
            const auto res = std::from_chars(fields[0].data(), fields[0].data() + fields[0].size(), pts[n].inputDb);
            if (res.ec != std::errc()) return false;
        }
        {
            const auto res = std::from_chars(fields[1].data(), fields[1].data() + fields[1].size(), pts[n].outputDb);
            if (res.ec != std::errc()) return false;
        }
        pts[n].direction = (nf >= 3 && !fields[2].empty()) ? clampPointDirection(fields[2][0] - '0') : PointDirection::Both;
        pts[n].handleInputDb = 0.0;
        pts[n].handleOutputDb = 0.0;
        if (nf >= 5)
        {
            double hi = 0.0, ho = 0.0;
            const auto r1 = std::from_chars(fields[3].data(), fields[3].data() + fields[3].size(), hi);
            const auto r2 = std::from_chars(fields[4].data(), fields[4].data() + fields[4].size(), ho);
            if (r1.ec == std::errc() && r2.ec == std::errc()) { pts[n].handleInputDb = hi; pts[n].handleOutputDb = ho; }
        }
        pts[n].link = 0; pts[n].linkAxis = 0;
        if (nf >= 6 && !fields[5].empty())
        {
            double lk = 0.0;
            const auto r = std::from_chars(fields[5].data(), fields[5].data() + fields[5].size(), lk);
            if (r.ec == std::errc()) { pts[n].link = int(lk) % 10; pts[n].linkAxis = int(lk) / 10; }
        }
        pts[n].diagonal = false;
        if (nf >= 7 && !fields[6].empty())
        {
            double dg = 0.0;
            const auto r = std::from_chars(fields[6].data(), fields[6].data() + fields[6].size(), dg);
            if (r.ec == std::errc()) pts[n].diagonal = dg > 0.5;
        }
        ++n;
        if (segEnd == std::string::npos) break;
        pos = segEnd;
    }
    return out.loadPoints(pts, n, (s[0] >= '1' && s[0] <= '9') ? s[0] - '0' : 0);
}

} // namespace sscomp
