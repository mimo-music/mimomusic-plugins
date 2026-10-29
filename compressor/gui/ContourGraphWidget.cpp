// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "ContourGraphWidget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

using sscomp::CurvePoint;
using sscomp::TransferCurve;
using sscomp::TriggerCondition;

ContourGraphWidget::ContourGraphWidget(NanoTopLevelWidget* parent)
    : EditorSubWidget(parent)
{
    loadSharedResources();
    std::fill(std::begin(history_), std::end(history_), 0.0f);
}

void ContourGraphWidget::updateHold(float& holdDb, std::chrono::steady_clock::time_point& lastUpdate,
                                     std::chrono::steady_clock::time_point& peakAt, float grDb)
{
    // Identical ballistic to CompressorGraphWidget::updateHold() -- see
    // that file's own comment for the full reasoning.
    const auto now = std::chrono::steady_clock::now();
    const float dtSec = std::chrono::duration<float>(now - lastUpdate).count();
    lastUpdate = now;

    constexpr float kGrHoldTimeSec = 1.5f;
    constexpr float kRecoverDbPerSec = 12.0f;
    if (std::fabs(grDb) >= std::fabs(holdDb))
    {
        holdDb = grDb;
        peakAt = now;
    }
    else if (std::chrono::duration<float>(now - peakAt).count() >= kGrHoldTimeSec)
    {
        const float mag = std::fabs(holdDb) - kRecoverDbPerSec * dtSec;
        holdDb = mag > 0.0f ? std::copysign(mag, holdDb) : 0.0f;
    }
}

void ContourGraphWidget::setLiveLevels(float detectorLevelDb, float gainReductionDb, float meterGrDb, bool pushToTrail)
{
    detectorLevelDb_ = detectorLevelDb;
    grDbRaw_ = gainReductionDb;
    grSmoother_[0].process(meterGrDb, grSpeed_);
    gainReductionDb_ = grSmoother_[0].value();
    updateHold(grHoldDb_[0], grHoldUpdated_[0], grHoldPeakAt_[0], meterGrDb);
    history_[historyPos_] = meterGrDb;
    historyPos_ = (historyPos_ + 1) % kHistoryLen;
    if (pushToTrail) pushTrail(0, detectorLevelDb, gainReductionDb);
    repaint();
}

void ContourGraphWidget::setSecondaryLiveLevels(bool show, float detectorLevelDb, float gainReductionDb, bool pushToTrail)
{
    secondaryShown_ = show;
    detectorLevelDbB_ = detectorLevelDb;
    grDbRawB_ = gainReductionDb;
    grSmoother_[1].process(gainReductionDb, grSpeed_);
    gainReductionDbB_ = grSmoother_[1].value();
    updateHold(grHoldDb_[1], grHoldUpdated_[1], grHoldPeakAt_[1], gainReductionDb);
    if (pushToTrail) pushTrail(1, detectorLevelDb, gainReductionDb);
}

void ContourGraphWidget::pushTrailBatch(int voice, const float* levelDb, const float* grDb, const unsigned char* trig, int n)
{
    for (int k = 0; k < n; ++k) pushTrail(voice, levelDb[k], grDb[k], trig != nullptr && trig[k] != 0);
}

void ContourGraphWidget::pushTrail(int voice, float levelDb, float grDb, bool trig)
{
    // Same "nothing meaningful playing" guard the old single marker dot
    // used -- no point trailing a heap of points pinned at the floor
    // during silence.
    if (levelDb <= float(TransferCurve::kMinDb) + 0.5f) return;
    TrailPoint& tp = trail_[voice][trailPos_[voice]];
    tp.inDb = levelDb;
    tp.outDb = levelDb + grDb;
    tp.trig = trig;
    tp.t = std::chrono::steady_clock::now();
    trailPos_[voice] = (trailPos_[voice] + 1) % kTrailCapacity;
    if (trailCount_[voice] < kTrailCapacity) ++trailCount_[voice];
}

namespace {
// Linear-interpolate between two colors (including alpha), t in [0,1].
inline Color lerpColor(const Color& a, const Color& b, float t)
{
    return Color(a.red + (b.red - a.red) * t, a.green + (b.green - a.green) * t,
                 a.blue + (b.blue - a.blue) * t, a.alpha + (b.alpha - a.alpha) * t);
}
}

Color ContourGraphWidget::trailColor(bool secondary, float frac, bool triggered) const
{
    // A colormap that ends in a fade-out, rather than a plain alpha fade:
    // a single fading color reads as "less bright", which is a weaker cue for *how old* a point is than an actual hue shift).
    // A short thermal-style progression -- pale/hot when brand new,
    // cooling toward each voice's own base hue, then fading to fully
    // transparent -- rather than a generic rainbow colormap, specifically
    // so voice A still reads as "the amber one" and voice B as "the blue
    // one" at a glance, matching every other amber/blue A-vs-B pairing
    // already established elsewhere in this plugin's own GR meter/marker
    // colors -- only *within* that identity does color now also encode
    // age. frac: 1 = just pushed, 0 = fully faded (alpha 0 there by
    // construction, the last stop's own alpha).
    frac = std::max(0.0f, std::min(1.0f, frac));
    static const Color kAmberStops[4] = {
        Color(255, 250, 238, 0.85f), // just now: near-white hot
        Color(250, 200, 80, 0.65f),  // this plugin's own marker amber
        Color(214, 110, 60, 0.38f),  // cooling toward a deep orange/red
        Color(120, 50, 46, 0.0f),    // about to vanish: dark maroon, transparent
    };
    static const Color kBlueStops[4] = {
        Color(238, 246, 255, 0.85f), // just now: near-white cool
        Color(160, 190, 235, 0.65f), // this plugin's own secondary-marker blue
        Color(90, 110, 200, 0.38f),  // cooling toward indigo
        Color(55, 46, 110, 0.0f),    // about to vanish: dark violet, transparent
    };
    // Dots where the trigger condition WAS met use a green palette (same idea, same
    // age progression, near-white when brand new): green = the curve is engaged,
    // matching the threshold line turning green.
    static const Color kGreenStops[4] = {
        Color(238, 255, 242, 0.85f),
        Color(110, 225, 140, 0.65f),
        Color(50, 150, 90, 0.38f),
        Color(25, 80, 55, 0.0f),
    };
    static const Color kTealStops[4] = {   // voice B, triggered
        Color(236, 255, 250, 0.85f),
        Color(90, 220, 205, 0.65f),
        Color(50, 140, 150, 0.38f),
        Color(30, 70, 90, 0.0f),
    };
    const Color* stops = triggered ? (secondary ? kTealStops : kGreenStops) : (secondary ? kBlueStops : kAmberStops);
    const float pos = (1.0f - frac) * 3.0f; // 0..3 across the 4 stops, 0 = newest
    const int i = std::min(2, int(pos));
    return lerpColor(stops[i], stops[i + 1], pos - float(i));
}

void ContourGraphWidget::drawTrail(int voice, bool secondary)
{
    // A fading heat map, to see where the audio is going -- with ~48x the
    // points (one block's bank per redraw), so it has to be cheap: points are small squares
    // batched per age BUCKET (one path + one fill per bucket and colour class instead of one
    // per dot). The ring buffer is chronological (oldest first), so age -- and therefore the
    // bucket -- only ever moves towards "fresher" along the walk: each bucket is one
    // contiguous run of ring positions, and the newest paints last, on top. Within a bucket
    // the points that met the trigger condition (green) are drawn after the others (amber /
    // blue) so they stay visible.
    constexpr int kBuckets = 28;
    const auto now = std::chrono::steady_clock::now();
    const int n = trailCount_[voice];

    struct Run { int first = -1, last = -1; };
    Run runs[kBuckets];
    for (int k = 0; k < n; ++k)
    {
        const int idx = (trailPos_[voice] - n + k + kTrailCapacity * 2) % kTrailCapacity;
        const float ageSec = std::chrono::duration<float>(now - trail_[voice][idx].t).count();
        if (ageSec < 0.0f || ageSec > trailSeconds_) continue;
        const float frac = 1.0f - ageSec / trailSeconds_;
        const int bucket = std::min(kBuckets - 1, int(frac * float(kBuckets)));
        if (runs[bucket].first < 0) runs[bucket].first = k;
        runs[bucket].last = k;
    }
    for (int b = 0; b < kBuckets; ++b)
    {
        if (runs[b].first < 0) continue;
        const float frac = (float(b) + 0.5f) / float(kBuckets);
        const float half = 1.0f + frac * frac * 1.4f;
        for (int pass = 0; pass < 2; ++pass) // 0: untriggered, 1: triggered
        {
            bool open = false;
            for (int k = runs[b].first; k <= runs[b].last; ++k)
            {
                const int idx = (trailPos_[voice] - n + k + kTrailCapacity * 2) % kTrailCapacity;
                const TrailPoint& tp = trail_[voice][idx];
                if (tp.trig != (pass == 1)) continue;
                const float ageSec = std::chrono::duration<float>(now - tp.t).count();
                if (ageSec < 0.0f || ageSec > trailSeconds_) continue;
                if (!open) { beginPath(); open = true; }
                rect(dbToPxX(tp.inDb) - half, dbToPxY(tp.outDb) - half, half * 2.0f, half * 2.0f);
            }
            if (open) { fillColor(trailColor(secondary, frac, pass == 1)); fill(); }
        }
    }
}

bool ContourGraphWidget::hitTrailLegend(float px, float py) const
{
    constexpr float pad = 3.0f;
    return px >= trailLegendX_ - pad && px <= trailLegendX_ + trailLegendW_ + pad &&
           py >= trailLegendY_ - pad && py <= trailLegendY_ + trailLegendH_ + pad;
}

void ContourGraphWidget::drawTrailLegend(float x, float y, float w, float h)
{
    // Doubles as both the "what does this color mean" key and a readout
    // of the configured trail length (where the display's limit is): the
    // gradient itself IS the colormap trail dots are drawn
    // with, so its own extent literally shows what a fully-aged trail
    // point looks like; the text names the configured duration (scroll
    // here to change it, kTrailSecondsMin..Max) and, whenever the trail
    // hasn't actually accumulated that much history yet -- the plugin
    // was just opened, or (in principle) kTrailCapacity's own hard cap
    // was reached before trailSeconds_ was -- shows the shorter, real
    // depth instead of silently pretending the full window is already
    // populated.
    trailLegendX_ = x; trailLegendY_ = y; trailLegendW_ = w; trailLegendH_ = h;

    const bool active = hoverTrailLegend_;
    for (int i = 0; i < int(w); ++i)
    {
        const float frac = 1.0f - float(i) / w; // left = newest/hottest, right = oldest
        beginPath();
        rect(x + float(i), y, 1.6f, h);
        fillColor(trailColor(false, frac));
        fill();
    }
    beginPath();
    rect(x, y, w, h);
    strokeColor(active ? Color(200, 204, 212) : Color(90, 94, 102));
    strokeWidth(active ? 1.4f : 1.0f);
    stroke();

    // The actual currently-held depth (oldest live sample's own age, for
    // whichever voice has more history) vs. the configured trailSeconds_
    // -- only worth printing when they meaningfully differ, so the normal
    // steady-state legend just reads as a duration, not a running number.
    const auto now = std::chrono::steady_clock::now();
    float actualDepth = 0.0f;
    for (int voice = 0; voice < 2; ++voice)
    {
        if (trailCount_[voice] == 0) continue;
        const int oldestIdx = (trailPos_[voice] - trailCount_[voice] + kTrailCapacity) % kTrailCapacity;
        const float age = std::chrono::duration<float>(now - trail_[voice][oldestIdx].t).count();
        actualDepth = std::max(actualDepth, std::min(age, trailSeconds_));
    }

    char buf[32];
    if (actualDepth + 0.1f < trailSeconds_)
        std::snprintf(buf, sizeof(buf), "Trail %.1fs (%.1fs so far)", double(trailSeconds_), double(actualDepth));
    else
        std::snprintf(buf, sizeof(buf), "Trail %.1fs (scroll)", double(trailSeconds_));
    fillColor(active ? Color(220, 224, 230) : Color(130, 134, 142));
    fontSize(9.0f);
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w, y + h + 2.0f, buf, nullptr);
}

namespace {
inline float dbToAxis(float db, float minDb, float maxDb)
{
    return std::max(0.0f, std::min(1.0f, (db - minDb) / (maxDb - minDb)));
}

// Above: the engaged region is the *upper* (louder) side of Threshold --
// tinted amber/warm, same family CompressorGraphWidget's own threshold
// marker and cut-fill already use for "this is where compression is
// doing something". Below: the engaged region is the *lower* (quieter)
// side instead -- tinted a cool blue, the same hue this project's own
// upward-compression/boost coloring already uses, so a glance at which
// half is warm vs. cool tells you the condition without reading its
// button's own label. The un-engaged side is always the same dim neutral
// either way.
inline Color engagedTint() { return Color(200, 150, 70, 26.0f / 255.0f); }
// The threshold's own colour language: yellow while idle, green while the trigger is engaged
// (e = 0..1, already smoothed with the attack/release times by the owner).
inline Color thresholdColor(bool bright, float e)
{
    return lerpColor(bright ? Color(230, 200, 100) : Color(180, 150, 60), bright ? Color(140, 240, 160) : Color(80, 200, 110), e);
}
inline Color engagedTintFor(float e) { return lerpColor(engagedTint(), Color(70, 200, 110, 40.0f / 255.0f), e); }
inline Color disengagedTint() { return Color(70, 74, 82, 14.0f / 255.0f); }

// Hysteresis direction colors -- shared by both the curve line itself
// (drawTransferCurve()'s own "diverged path" split, see that function's
// comment) and each tagged point's own stroke below, so the two read as
// one consistent color language rather than two independently-chosen
// pairs. Rising = warm amber, Falling = cool blue-violet -- deliberately
// distinct from the plain untagged curve's own teal-green (so an
// untagged curve/region still reads as "the normal color", unchanged)
// and from the amber/blue A-vs-B channel identity used elsewhere in this
// plugin (close in hue by coincidence, but never shown in the same place
// at the same time as that convention, so it doesn't collide in practice).
// One color per link group (1..kMaxLinkGroups) -- ring + connector lines.
inline Color linkColor(int group)
{
    static const int kPal[9][3] = {
        { 240, 120, 170 }, { 120, 220, 200 }, { 250, 210, 90 }, { 170, 140, 250 }, { 130, 200, 110 },
        { 250, 150, 90 },  { 110, 170, 250 }, { 220, 110, 110 }, { 200, 200, 200 },
    };
    const int* c = kPal[((group - 1) % 9 + 9) % 9];
    return Color(c[0], c[1], c[2]);
}
inline Color risingPathColor(bool active) { return active ? Color(250, 195, 90) : Color(214, 158, 66); }
inline Color fallingPathColor(bool active) { return active ? Color(196, 206, 255) : Color(146, 158, 232); }
}

float ContourGraphWidget::dbToPxX(float db) const { return curveX_ + dbToAxis(db, float(TransferCurve::kMinDb), float(TransferCurve::kMaxDb)) * curveW_; }
float ContourGraphWidget::dbToPxY(float db) const { return curveY_ + curveH_ - dbToAxis(db, float(TransferCurve::kMinDb), float(TransferCurve::kMaxDb)) * curveH_; }
float ContourGraphWidget::pxToDbX(float px) const
{
    if (curveW_ <= 0.0f) return float(TransferCurve::kMinDb);
    const float t = std::max(0.0f, std::min(1.0f, (px - curveX_) / curveW_));
    return float(TransferCurve::kMinDb) + t * float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
}
float ContourGraphWidget::pxToDbY(float py) const
{
    if (curveH_ <= 0.0f) return float(TransferCurve::kMinDb);
    const float t = std::max(0.0f, std::min(1.0f, (curveY_ + curveH_ - py) / curveH_));
    return float(TransferCurve::kMinDb) + t * float(TransferCurve::kMaxDb - TransferCurve::kMinDb);
}

void ContourGraphWidget::drawTransferCurve(float x, float y, float w, float h)
{
    auto xToPx = [&](float db) { return dbToPxX(db); };
    auto yToPx = [&](float db) { return dbToPxY(db); };

    crispRect(*this, x, y, w, h);
    fillColor(Color(18, 19, 23));
    fill();
    strokeColor(Color(48, 51, 58));
    strokeWidth(lineWidth());
    stroke();

    // Threshold background split -- a visible difference for which side
    // the trigger condition actually engages on, drawn *before* the gridlines/curve so it reads as
    // background shading, not a foreground element competing with them.
    {
        const float tx = xToPx(thresholdDb_);
        const bool engagedIsAbove = condition_ == TriggerCondition::Above;
        beginPath();
        rect(x, y, tx - x, h);
        fillColor(engagedIsAbove ? disengagedTint() : engagedTintFor(triggerActivity_));
        fill();
        beginPath();
        rect(tx, y, x + w - tx, h);
        fillColor(engagedIsAbove ? engagedTintFor(triggerActivity_) : disengagedTint());
        fill();
    }

    // Gridlines every 10dB (-80 .. 0), labeled on both axes.
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    for (float db = float(TransferCurve::kMinDb); db <= float(TransferCurve::kMaxDb) + 0.01f; db += 10.0f)
    {
        const float px = xToPx(db);
        beginPath();
        moveTo(px, y);
        lineTo(px, y + h);
        strokeColor(Color(40, 43, 50));
        strokeWidth(1.0f);
        stroke();
        fillColor(Color(110, 114, 122));
        textAlign(ALIGN_CENTER | ALIGN_TOP);
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%.0f", db);
        text(px, y + h + 1.0f, buf, nullptr);

        const float py = yToPx(db);
        crispHLine(*this, x, x + w, py, 1.0f, Color(40, 43, 50));
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        text(x - 3.0f, py, buf, nullptr);
    }

    // Unity (no change) reference line, dashed -- identical to
    // CompressorGraphWidget's own.
    {
        constexpr float dash = 4.0f, gapLen = 4.0f;
        float px0 = xToPx(float(TransferCurve::kMinDb)), py0 = yToPx(float(TransferCurve::kMinDb));
        const float px1 = xToPx(float(TransferCurve::kMaxDb)), py1 = yToPx(float(TransferCurve::kMaxDb));
        const float len = std::hypot(px1 - px0, py1 - py0);
        const float dx = (px1 - px0) / len, dy = (py1 - py0) / len;
        float pos = 0.0f;
        strokeColor(Color(90, 94, 100));
        strokeWidth(1.0f);
        while (pos < len)
        {
            const float segEnd = std::min(len, pos + dash);
            beginPath();
            moveTo(px0 + dx * pos, py0 + dy * pos);
            lineTo(px0 + dx * segEnd, py0 + dy * segEnd);
            stroke();
            pos = segEnd + gapLen;
        }
    }

    // Threshold marker, dashed vertical -- same visual weight as
    // CompressorGraphWidget's own, brighter when hovered/dragged so it
    // reads as its own separate, grabbable thing distinct from the
    // background split.
    {
        const float tx = xToPx(thresholdDb_);
        constexpr float dash = 3.0f, gapLen = 3.0f;
        float py = y;
        strokeColor(thresholdColor(hoverThreshold_ || draggingThreshold_, triggerActivity_));
        strokeWidth((hoverThreshold_ || draggingThreshold_) ? 1.8f : 1.2f);
        while (py < y + h)
        {
            const float segEnd = std::min(y + h, py + dash);
            beginPath();
            moveTo(tx, py);
            lineTo(tx, segEnd);
            stroke();
            py = segEnd + gapLen;
        }

        // Threshold's own "node" handle -- a real circular handle on its
        // line, near the bottom edge (clear of the trail legend's own
        // top-center spot, see that widget's own comment): it reads as a
        // node like any curve point below, and can join a
        // multi-selection the same shift-click/marquee way (this file's
        // own header comment). Unlike a curve point it has no Y of its
        // own -- the circle's height is fixed, purely a grab target.
        const bool thrActive = hoverThreshold_ || draggingThreshold_ || (draggingGroup_ && thresholdSelected_);
        const float ty = y + h - 12.0f;
        beginPath();
        circle(tx, ty, thrActive ? 6.5f : 5.0f);
        fillColor(Color(20, 21, 25).withAlpha(0.75f));
        fill();
        strokeColor(thrActive ? thresholdColor(true, triggerActivity_) : thresholdColor(false, triggerActivity_));
        strokeWidth(thrActive ? 2.0f : 1.4f);
        stroke();
        if (curve_.thresholdLink() > 0)
        {
            beginPath();
            circle(tx, ty, 12.0f);
            strokeColor(linkColor(curve_.thresholdLink()));
            strokeWidth(1.8f);
            stroke();
        }
        if (thresholdSelected_)
        {
            beginPath();
            circle(tx, ty, 9.5f);
            strokeColor(Color(120, 200, 255));
            strokeWidth(1.6f);
            stroke();
        }
    }

    // The actual transfer curve -- sscomp::TransferCurve::evaluateDirectional()
    // at a few hundred x positions, exactly the same "closed-form pure
    // function, cheap enough to redraw every frame" approach
    // CompressorGraphWidget's own drawTransferCurve() (and, further
    // afield, ../statespace's EqGraphWidget) already use.
    //
    // The lines between nodes show each path: rising-only and falling-only
    // nodes connect only with compatible ones, and a stretch used by only
    // one direction gets its own color -- two independent polylines are
    // built, one per
    // direction (evaluateDirectional(x,true)/(x,false)), each forcing a
    // sample at every point actually visible in *that* direction's own
    // view (TransferCurve::activePoint(), not the plain point() list --
    // see that method's own comment for why forcing point samples matters
    // at all: preserving step/gate walls and making a point's own marker
    // sit exactly on the line). Wherever the two agree they're drawn as
    // one shared, neutral-colored path (the common case: an untagged
    // curve never diverges anywhere, so this is the *only* path ever
    // drawn then, in the exact same teal this line has always used);
    // wherever they disagree, both are drawn, in their own direction
    // color, scissored to just that diverging x-range so the "shared"
    // stretches on either side stay a single clean line rather than two
    // colored lines sitting exactly on top of each other.
    {
        struct Sample { float x, y; };
        constexpr int kSteps = 400;
        const Color kNeutralPathColor(120, 210, 170);

        auto buildPolyline = [&](bool rising, Sample* samples) {
            int ns = 0;
            for (int i = 0; i <= kSteps; ++i)
            {
                const float xv = float(TransferCurve::kMinDb) + float(TransferCurve::kMaxDb - TransferCurve::kMinDb) * float(i) / float(kSteps);
                samples[ns++] = { xv, float(curve_.evaluateDirectional(xv, rising)) };
            }
            const int an = curve_.activePointCount(rising);
            for (int i = 0; i < an; ++i)
            {
                const CurvePoint p = curve_.activePoint(rising, i);
                samples[ns++] = { float(p.inputDb), float(p.outputDb) };
            }
            // See the pre-hysteresis version of this code (git history) for
            // the full reasoning on stable-sort + the step-pair dedupe
            // below -- unchanged, just now built once per direction.
            std::stable_sort(samples, samples + ns, [](const Sample& a, const Sample& b) { return a.x < b.x; });
            return ns;
        };

        auto strokePolyline = [&](const Sample* samples, int ns, Color color, float xDbLo, float xDbHi, bool useScissor) {
            if (useScissor)
            {
                const float sx0 = xToPx(xDbLo), sx1 = xToPx(xDbHi);
                scissor(std::min(sx0, sx1), y, std::max(1.0f, std::fabs(sx1 - sx0)), h);
            }
            beginPath();
            bool first = true;
            float prevX = -1.0e9f, prevY = -1.0e9f;
            for (int i = 0; i < ns; ++i)
            {
                if (i > 0 && std::fabs(samples[i].x - prevX) < 1.0e-4f && std::fabs(samples[i].y - prevY) < 1.0e-4f) continue;
                prevX = samples[i].x; prevY = samples[i].y;
                const float px = xToPx(samples[i].x), py = yToPx(samples[i].y);
                if (first) { moveTo(px, py); first = false; } else lineTo(px, py);
            }
            strokeColor(color);
            strokeWidth(2.2f);
            stroke();
            if (useScissor) resetScissor();
        };

        Sample risingSamples[kSteps + 1 + TransferCurve::kMaxPoints];
        Sample fallingSamples[kSteps + 1 + TransferCurve::kMaxPoints];
        const int nsR = buildPolyline(true, risingSamples);
        const int nsF = buildPolyline(false, fallingSamples);

        // Fast, common path: nothing tagged, so the two views are
        // identical everywhere -- draw exactly what this widget has
        // always drawn, no run-detection overhead at all.
        const bool anyTagged = curve_.activePointCount(true) != curve_.pointCount() ||
                                curve_.activePointCount(false) != curve_.pointCount();
        if (!anyTagged)
        {
            strokePolyline(risingSamples, nsR, kNeutralPathColor, 0, 0, false);
        }
        else
        {
            constexpr float kDivergeEpsDb = 0.05f;
            bool diverged[kSteps + 1];
            for (int i = 0; i <= kSteps; ++i)
            {
                const float xv = float(TransferCurve::kMinDb) + float(TransferCurve::kMaxDb - TransferCurve::kMinDb) * float(i) / float(kSteps);
                diverged[i] = std::fabs(curve_.evaluateDirectional(xv, true) - curve_.evaluateDirectional(xv, false)) > kDivergeEpsDb;
            }
            float runStartDb = float(TransferCurve::kMinDb);
            bool runState = diverged[0];
            for (int i = 1; i <= kSteps; ++i)
            {
                const bool last = (i == kSteps);
                if (diverged[i] != runState || last)
                {
                    const float runEndDb = float(TransferCurve::kMinDb) + float(TransferCurve::kMaxDb - TransferCurve::kMinDb) * float(i) / float(kSteps);
                    if (runState)
                    {
                        strokePolyline(risingSamples, nsR, risingPathColor(false), runStartDb, runEndDb, true);
                        strokePolyline(fallingSamples, nsF, fallingPathColor(false), runStartDb, runEndDb, true);
                    }
                    else
                    {
                        strokePolyline(risingSamples, nsR, kNeutralPathColor, runStartDb, runEndDb, true);
                    }
                    runStartDb = runEndDb;
                    runState = diverged[i];
                }
            }
        }
    }

    // Fading heat trail(s) -- drawn before the "now" head dot so the dot
    // still reads as a distinct anchor point on top of it. See
    // drawTrail()'s own comment.
    if (secondaryShown_) drawTrail(1, true);
    drawTrail(0, false);

    // The trail's own color legend / duration control -- top-center,
    // deliberately NOT a corner: both curve endpoints are pinned to the
    // left/right edges (see TransferCurve's own header comment) but their
    // own Y is freely draggable, so either top corner can end up under a
    // dragged endpoint handle depending on the curve's own shape -- top-
    // center is the one spot neither endpoint's fixed X ever reaches. See
    // drawTrailLegend()'s own comment for what it shows.
    drawTrailLegend(x + w * 0.5f - 32.0f, y + 5.0f, 64.0f, 8.0f);

    // Live marker(s) -- current instantaneous position on the curve, the
    // trail's own "now" head. Uses the *raw* (unsmoothed) gain reduction,
    // not gainReductionDb_/gainReductionDbB_'s own ~220ms display easing
    // (see this file's header comment on grDbRaw_) -- with a literal 0ms
    // attack/release this dot must sit exactly on the curve, not lag
    // behind it by an extra layer of GUI-only smoothing that has nothing
    // to do with what the DSP is actually doing.
    auto drawMarker = [&](float levelDb, float grDb, Color c)
    {
        if (levelDb <= float(TransferCurve::kMinDb) + 0.5f) return;
        const float px = xToPx(levelDb);
        const float py = yToPx(levelDb + grDb);
        beginPath();
        circle(px, py, 5.0f);
        fillColor(c.withAlpha(60.0f / 255.0f));
        fill();
        beginPath();
        circle(px, py, 3.0f);
        fillColor(c);
        fill();
    };
    if (secondaryShown_) drawMarker(detectorLevelDbB_, grDbRawB_, Color(160, 190, 235));
    drawMarker(detectorLevelDb_, grDbRaw_, lerpColor(Color(235, 195, 60), Color(120, 235, 150), triggerActivity_));

    // Link groups: a faint line joining a group's members (in X order) so the
    // grouping reads at a glance; each member also gets a colored ring below.
    for (int g = 1; g <= sscomp::kMaxLinkGroups; ++g)
    {
        int prev = -1;
        for (int i = 0; i < curve_.pointCount(); ++i)
        {
            if (curve_.pointLink(i) != g) continue;
            if (prev >= 0 && curve_.pointLinkAxis(i) == sscomp::kLinkAxisBoth)
            {
                const CurvePoint a = curve_.point(prev), b = curve_.point(i);
                beginPath();
                moveTo(xToPx(float(a.inputDb)), yToPx(float(a.outputDb)));
                lineTo(xToPx(float(b.inputDb)), yToPx(float(b.outputDb)));
                strokeColor(linkColor(g).withAlpha(0.35f));
                strokeWidth(1.2f);
                stroke();
            }
            prev = i;
        }
    }

    // Curve control points -- every point is a draggable handle; the two
    // fixed endpoints (index 0 and pointCount()-1) are drawn as squares
    // rather than circles so "this one can't move sideways" is visible at
    // a glance, not just enforced silently on drag.
    const int n = curve_.pointCount();
    for (int i = 0; i < n; ++i)
    {
        const CurvePoint p = curve_.point(i);
        const float px = xToPx(float(p.inputDb));
        const float py = yToPx(float(p.outputDb));
        const bool active = (draggingPoint_ == i) || (draggingPoint_ < 0 && hoverPoint_ == i) ||
                            (draggingGroup_ && selected_[i]) || (draggingHandle_ && draggingHandlePoint_ == i) ||
                            (placingSmoothPoint_ && placingSmoothIndex_ == i);
        const bool isEndpoint = curve_.isEndpoint(i);

        // Inkscape-style tangent handle (click-and-drag places the support
        // points that set the slope at the node). A single line through the
        // point (the
        // outgoing half, toward point+handle, and its mirrored incoming
        // half, toward point-handle -- see plugin/dsp/CurveDsp.hpp's own
        // header comment on why it's one symmetric handle, not two
        // independent ones) with a small dot on the outgoing end, the one
        // draggable target (right-click-drag on the point itself, see
        // onMouse()'s own comment -- the dot is purely a visual read of
        // where that drag currently points, not its own separate hit
        // target). Only drawn once the point actually has a handle (a
        // corner node, the default, draws nothing extra here at all).
        if (p.handleInputDb != 0.0 || p.handleOutputDb != 0.0)
        {
            // Support points are grabbable (both tools) -- brighter for
            // whichever one is hovered/dragged.
            const bool hotOut = (hoverHandle_ == i && !hoverHandleIsIn_) || (draggingHandle_ && draggingHandlePoint_ == i && !draggingHandleIsIn_);
            const bool hotIn = (hoverHandle_ == i && hoverHandleIsIn_) || (draggingHandle_ && draggingHandlePoint_ == i && draggingHandleIsIn_);
            // Unclamped geometry: the true line keeps its true slope (clipped
            // to the plot), and a dot that falls outside the plot -- the
            // mirrored half of an end node's handle, or any dot near a
            // border -- is not drawn at all instead of being pinned to the
            // edge where it showed a wrong slope.
            const float outX = dbToPxXRaw(p.inputDb + p.handleInputDb);
            const float outY = dbToPxYRaw(p.outputDb + p.handleOutputDb);
            const float inX = dbToPxXRaw(p.inputDb - p.handleInputDb);
            const float inY = dbToPxYRaw(p.outputDb - p.handleOutputDb);
            scissor(x, y, w, h);
            beginPath();
            moveTo(inX, inY);
            lineTo(outX, outY);
            strokeColor(active ? Color(210, 214, 222, 0.75f) : Color(150, 154, 162, 0.5f));
            strokeWidth(1.2f);
            stroke();
            resetScissor();
            if (insidePlot(outX, outY))
            {
                beginPath();
                circle(outX, outY, (hotOut ? 5.2f : active ? 3.2f : 2.4f) + 1.6f);
                fillColor(hotOut ? Color(120, 200, 255) : active ? Color(230, 233, 238) : Color(170, 174, 182));
                fill();
            }
            if (insidePlot(inX, inY))
            {
                beginPath();
                circle(inX, inY, (hotIn ? 5.2f : active ? 2.6f : 2.0f) + 1.6f);
                fillColor(hotIn ? Color(120, 200, 255) : active ? Color(190, 194, 202, 0.85f) : Color(130, 134, 142, 0.7f));
                fill();
            }
        }

        beginPath();
        if (isEndpoint)
        {
            const float r = active ? 6.5f : 5.0f;
            rect(px - r, py - r, r * 2.0f, r * 2.0f);
        }
        else
        {
            circle(px, py, active ? 7.0f : 5.5f);
        }
        fillColor(Color(20, 21, 25).withAlpha(0.75f));
        fill();
        // Hysteresis direction tag (a node active only while the input is
        // rising, or only while falling): a tagged
        // interior point's own stroke tints green (RisingOnly) or violet
        // (FallingOnly) instead of the plain neutral gray/white every
        // other point (and every point on an untagged curve) still uses --
        // deliberately a different color pair from the amber/blue A-vs-B
        // channel identity used everywhere else in this plugin, so the
        // two unrelated concepts (which channel vs. which direction)
        // never read as the same cue. Right-click a point to cycle its
        // tag -- see onMouse()'s own comment.
        const sscomp::PointDirection dir = curve_.pointDirection(i);
        const Color neutralStroke = active ? Color(235, 238, 242) : Color(170, 176, 186);
        strokeColor(dir == sscomp::PointDirection::RisingOnly ? risingPathColor(active) :
                    dir == sscomp::PointDirection::FallingOnly ? fallingPathColor(active) : neutralStroke);
        strokeWidth(active ? 2.0f : 1.4f);
        stroke();

        // Diagonal-snap mark: a short tick through the node at the unity line's own angle and
        // color (see the dashed reference line drawn near the top of this
        // method), so a glance at the node itself already shows it can only
        // slide along that line -- distinct from the link ring's (colored,
        // circular) and selection ring's (blue) cues below, which a snapped
        // node keeps drawing exactly as normal since it stays fully
        // linkable and selectable.
        if (p.diagonal)
        {
            const float r = (isEndpoint ? (active ? 6.5f : 5.0f) : (active ? 7.0f : 5.5f)) + 2.0f;
            beginPath();
            moveTo(px - r, py + r);
            lineTo(px + r, py - r);
            strokeColor(Color(180, 184, 192, 0.9f));
            strokeWidth(1.6f);
            stroke();
        }

        // Selection highlight ring (several nodes can be selected and moved
        // together): a plain outer ring, independent of the hover/drag "active" look above so
        // a selected-and-hovered point still shows both cues at once.
        if (p.link > 0)
        {
            beginPath();
            if (isEndpoint) { const float r = 10.5f; rect(px - r, py - r, r * 2.0f, r * 2.0f); }
            else circle(px, py, 11.0f);
            strokeColor(linkColor(p.link));
            strokeWidth(1.8f);
            stroke();
            // Axis marker: little arrowheads on the ring showing what the link
            // propagates -- left/right for X only, up/down for Y only, none for both.
            if (p.linkAxis != sscomp::kLinkAxisBoth)
            {
                const bool horiz = p.linkAxis == sscomp::kLinkAxisX;
                const float r = 15.0f, t = 3.6f;
                fillColor(linkColor(p.link));
                for (int sgn = -1; sgn <= 1; sgn += 2)
                {
                    beginPath();
                    if (horiz) { moveTo(px + sgn * (r + t), py); lineTo(px + sgn * r, py - t); lineTo(px + sgn * r, py + t); }
                    else       { moveTo(px, py + sgn * (r + t)); lineTo(px - t, py + sgn * r); lineTo(px + t, py + sgn * r); }
                    fill();
                }
            }
        }
        if (selected_[i])
        {
            beginPath();
            if (isEndpoint) { const float r = active ? 9.0f : 7.5f; rect(px - r, py - r, r * 2.0f, r * 2.0f); }
            else circle(px, py, active ? 9.5f : 8.0f);
            strokeColor(Color(120, 200, 255));
            strokeWidth(1.6f);
            stroke();
        }
    }

    // Draw tools: a ghost of the node the next click would create, with
    // its dB coordinates -- red when the curve would refuse it (already 18
    // points, or closer than TransferCurve::kMinSeparationDb in X to a
    // neighbor). Hidden while any drag is running or the cursor is off the
    // plot / over the threshold node / the trail legend.
    if (isDrawTool() && !menuOpen_ && !pendingAddPoint_ && !placingSmoothPoint_ && draggingPoint_ < 0 && hoverPoint_ < 0 && hoverHandle_ < 0 && !draggingThreshold_ && !hoverThreshold_ && !hoverTrailLegend_ &&
        hoverX_ >= x && hoverX_ <= x + w && hoverY_ >= y && hoverY_ <= y + h)
    {
        const double gIn = double(pxToDbX(hoverX_)), gOut = double(pxToDbY(hoverY_));
        TransferCurve probe = curve_;
        const bool ok = probe.addPoint(gIn, gOut) >= 0;
        const Color base = tool_ == EditTool::DrawRising ? risingPathColor(true)
                         : tool_ == EditTool::DrawFalling ? fallingPathColor(true) : Color(225, 228, 235);
        const Color c = ok ? base : Color(220, 90, 80);
        beginPath();
        circle(hoverX_, hoverY_, 6.0f);
        fillColor(c.withAlpha(0.18f));
        fill();
        strokeColor(c.withAlpha(0.85f));
        strokeWidth(1.4f);
        stroke();
        char buf[48];
        std::snprintf(buf, sizeof(buf), "%.1f / %.1f dB", gIn, gOut);
        fillColor(c.withAlpha(0.9f));
        fontSize(9.5f);
        const bool nearRight = hoverX_ > x + w - 110.0f;
        textAlign((nearRight ? ALIGN_RIGHT : ALIGN_LEFT) | ALIGN_BOTTOM);
        text(hoverX_ + (nearRight ? -9.0f : 9.0f), hoverY_ - 6.0f, buf, nullptr);
    }

    // Rubber-band marquee selection rectangle: drawn last so it's visible on
    // top of the curve/points/threshold while active.
    if (marqueeActive_)
    {
        const float rx0 = std::min(marqueeX0_, marqueeX1_), rx1 = std::max(marqueeX0_, marqueeX1_);
        const float ry0 = std::min(marqueeY0_, marqueeY1_), ry1 = std::max(marqueeY0_, marqueeY1_);
        beginPath();
        rect(rx0, ry0, rx1 - rx0, ry1 - ry0);
        fillColor(Color(120, 200, 255, 0.12f));
        fill();
        strokeColor(Color(120, 200, 255, 0.8f));
        strokeWidth(1.0f);
        stroke();
    }

    // A short hint, bottom-left, for the gestures that have no permanent
    // visible control of their own (unlike every button in the toolbar
    // above) -- shown only while nothing is being dragged/selected, so it
    // doesn't clutter the view mid-edit.
    if (draggingPoint_ < 0 && !draggingThreshold_ && !draggingGroup_ && !marqueeActive_ &&
        !placingSmoothPoint_ && !draggingHandle_)
    {
        fillColor(Color(120, 124, 132, 0.8f));
        fontSize(9.5f);
        textAlign(ALIGN_LEFT | ALIGN_BOTTOM);
        const char* hint = tool_ == EditTool::Edit
            ? "EDIT: drag node/support point to move. Drag empty area: select several. Shift+click: add to selection. "
              "Delete: remove selected. Double-click: remove. Right-click a node: menu (Rising/Falling, Link, Snap to Diagonal, Delete); right-drag: pull out handle."
            : tool_ == EditTool::DrawRising
            ? "DRAW RISING: click = corner node, drag = smooth node, active only while the input is rising. On a node: drag = move, double-click = remove, right-click = menu."
            : tool_ == EditTool::DrawFalling
            ? "DRAW FALLING: click = corner node, drag = smooth node, active only while the input is falling. On a node: drag = move, double-click = remove, right-click = menu."
            : "DRAW NODE: click = corner node, drag = smooth node. On a node: drag = move, double-click = remove, right-click = menu.";
        text(x + 5.0f, y + h - 3.0f, hint, nullptr);
    }
}

void ContourGraphWidget::drawGrMeterColumn(float x, float y, float w, float h, float grDb, float holdDb, float makeupDb, const char* label)
{
    // Same structure as CompressorGraphWidget::drawGrMeterColumn() (bipolar, 0dB reference, peak-hold, and the Makeup Gain
    // overlay -- see that file's own comments for the full reasoning); only the meter's own dB range differs
    // (kGrMeterMinDb/MaxDb, see this file's header comment).
    crispRect(*this, x, y, w, h);
    fillColor(Color(16, 17, 20));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(lineWidth());
    stroke();

    auto yOf = [&](float db) {
        const float frac = dbToAxis(std::max(kGrMeterMinDb, std::min(kGrMeterMaxDb, db)), kGrMeterMinDb, kGrMeterMaxDb);
        return y + (1.0f - frac) * h;
    };
    const float zeroY = yOf(0.0f);
    const float grY = yOf(grDb);
    // dB values add where linear gains multiply, and the Makeup Gain is applied to the curve's output (PluginContour::run()'s
    // makeup*Lin), so grDb + makeupDb is the net effect on level right now.
    const float netDb = grDb + makeupDb;
    const float netY = yOf(netDb);

    auto fillBar = [&](float barY, Color lo, Color hi, bool cut) {
        scissor(x, std::min(barY, zeroY), w, std::fabs(zeroY - barY));
        beginPath();
        rect(x, y, w, h);
        Paint grad = cut ? linearGradient(0, y, 0, zeroY, lo, hi)
                          : linearGradient(0, zeroY, 0, y + h, lo, hi);
        fillPaint(grad);
        fill();
        resetScissor();
    };

    // Background: the makeup-less reduction, dimmed; foreground: the net (sum) at full strength. Where they don't overlap --
    // e.g. a deep cut clawed back by a big positive Makeup Gain, landing on the other side of 0dB -- both stay visible.
    if (grDb < -0.01f)
        fillBar(grY, Color(230, 120, 70, 0.35f), Color(235, 195, 60, 0.35f), true);
    else if (grDb > 0.01f)
        fillBar(grY, Color(90, 190, 150, 0.35f), Color(70, 150, 220, 0.35f), false);

    if (netDb < -0.01f)
        fillBar(netY, Color(230, 120, 70), Color(235, 195, 60), true);
    else if (netDb > 0.01f)
        fillBar(netY, Color(90, 190, 150), Color(70, 150, 220), false);

    crispHLine(*this, x, x + w, zeroY, 1.0f, Color(90, 94, 100));

    if (std::fabs(holdDb) > 0.01f)
    {
        const float holdY = yOf(holdDb);
        crispHLine(*this, x, x + w, holdY, 1.5f, Color(235, 238, 242));
    }

    // Makeup Gain marker: a solid line at this column's own configured Makeup Gain (skipped at exactly 0dB so an untouched
    // column does not draw a second line on top of the 0dB reference line).
    if (std::fabs(makeupDb) > 0.01f)
    {
        const float makeupY = yOf(makeupDb);
        crispHLine(*this, x, x + w, makeupY, 1.5f, Color(160, 170, 230, 0.9f));
    }

    fillColor(Color(140, 144, 152));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.5f);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(x + w * 0.5f, y + h + 2.0f, label, nullptr);
}

void ContourGraphWidget::drawGrScale(float x, float y, float h)
{
    fillColor(Color(150, 154, 162));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(8.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    for (float db = kGrMeterMinDb; db <= kGrMeterMaxDb + 0.01f; db += 15.0f)
    {
        const float frac = dbToAxis(db, kGrMeterMinDb, kGrMeterMaxDb);
        const float py = y + (1.0f - frac) * h;
        char buf[8];
        std::snprintf(buf, sizeof(buf), "%+.0f", db);
        text(x, py, buf, nullptr);
    }
}

void ContourGraphWidget::drawGrHistory(float x, float y, float w, float h)
{
    crispRect(*this, x, y, w, h);
    fillColor(Color(14, 15, 18));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(lineWidth());
    stroke();

    fillColor(Color(120, 124, 132));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(9.0f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x + 3.0f, y + 1.0f, "Gain reduction (history)", nullptr);

    beginPath();
    for (int i = 0; i < kHistoryLen; ++i)
    {
        const int idx = (historyPos_ + i) % kHistoryLen;
        const float grDb = history_[idx];
        const float px = x + w * float(i) / float(kHistoryLen - 1);
        const float frac = dbToAxis(std::max(kGrMeterMinDb, std::min(kGrMeterMaxDb, grDb)), kGrMeterMinDb, kGrMeterMaxDb);
        const float py = y + h - frac * (h - 2.0f) - 1.0f;
        if (i == 0) moveTo(px, py); else lineTo(px, py);
    }
    strokeColor(Color(235, 195, 60));
    strokeWidth(1.5f);
    stroke();

    // 0dB reference line for the history strip -- the bipolar column
    // above already has one, but this strip's own axis is easy to
    // misread without it (an all-boost stretch and an all-cut stretch
    // both look like "a wiggly line" without a zero to compare against).
    const float zeroFrac = dbToAxis(0.0f, kGrMeterMinDb, kGrMeterMaxDb);
    const float zeroY = y + h - zeroFrac * (h - 2.0f) - 1.0f;
    crispHLine(*this, x, x + w, zeroY, 1.0f, Color(60, 64, 70, 0.6f));
}

void ContourGraphWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(20, 21, 25));
    fill();

    constexpr float pad = 8.0f;
    constexpr float grColW = 34.0f;
    constexpr float grScaleW = 24.0f;
    constexpr float histH = 46.0f;
    constexpr float labelPad = 14.0f;

    const float grColsW = grColW * (secondaryShown_ ? 2.0f : 1.0f) + (secondaryShown_ ? 4.0f : 0.0f);

    curveX_ = pad + labelPad;
    curveY_ = pad;
    curveW_ = w - curveX_ - pad - grColsW - grScaleW - 8.0f;
    curveH_ = h - curveY_ - pad - histH - labelPad - 6.0f;
    if (curveW_ <= 10.0f || curveH_ <= 10.0f) return;

    drawTransferCurve(curveX_, curveY_, curveW_, curveH_);

    float grX = curveX_ + curveW_ + 6.0f;
    drawGrMeterColumn(grX, curveY_, grColW, curveH_, gainReductionDb_, grHoldDb_[0], makeupGainDb_, labelA_.c_str());
    if (secondaryShown_)
    {
        grX += grColW + 4.0f;
        drawGrMeterColumn(grX, curveY_, grColW, curveH_, gainReductionDbB_, grHoldDb_[1], makeupGainDbB_, labelB_.c_str());
    }
    drawGrScale(grX + grColW + 3.0f, curveY_, curveH_);

    auto drawGrValue = [&](float colX, float grDb) {
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%+.1f", grDb);
        const float chipY = curveY_ + 3.0f, chipH = 14.0f;
        beginPath();
        roundedRect(colX + 1.0f, chipY, grColW - 2.0f, chipH, 2.0f);
        fillColor(Color(16, 17, 20, 0.75f));
        fill();
        fillColor(grDb < -0.05f ? Color(235, 195, 60) : grDb > 0.05f ? Color(120, 190, 235) : Color(170, 174, 182));
        fontSize(10.5f);
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(colX + grColW * 0.5f, chipY + chipH * 0.5f + 0.5f, buf, nullptr);
    };
    // Net (gain reduction + Makeup Gain), matching the foreground bar next to it.
    drawGrValue(curveX_ + curveW_ + 6.0f, gainReductionDb_ + makeupGainDb_);
    if (secondaryShown_) drawGrValue(curveX_ + curveW_ + 6.0f + grColW + 4.0f, gainReductionDbB_ + makeupGainDbB_);

    // History below the curve, the GR meter's Fast | Slow switch below the
    // meter -- same layout as CompressorGraphWidget.
    const float histY = curveY_ + curveH_ + labelPad + 6.0f;
    drawGrHistory(curveX_, histY, curveW_, histH);
    grSpeedSwitch_.setBox(curveX_ + curveW_ + 6.0f, histY, grColsW + grScaleW + 2.0f, histH);
    grSpeedSwitch_.draw(*this, grSpeed_);
    drawMenu();
}

namespace {
inline float dist(float px, float py, float hx, float hy) { return std::hypot(px - hx, py - hy); }
}

int ContourGraphWidget::hitPoint(float px, float py) const
{
    // Nearest handle within the radius, not just the first one encountered
    // by index order -- with two points close to their own minimum
    // allowed spacing, their hit circles overlap, and always favoring the
    // lower index there made grabbing/dragging the *other* point feel
    // random, depending on exactly where inside the overlap a click landed.
    int best = -1;
    float bestDist = nodeHitRadius();
    for (int i = 0; i < curve_.pointCount(); ++i)
    {
        const CurvePoint p = curve_.point(i);
        const float hx = dbToPxX(float(p.inputDb)), hy = dbToPxY(float(p.outputDb));
        const float d = dist(px, py, hx, hy);
        if (d <= bestDist)
        {
            bestDist = d;
            best = i;
        }
    }
    return best;
}

bool ContourGraphWidget::hitThresholdNode(float px, float py) const
{
    // Same spot drawTransferCurve() draws the threshold's round handle at.
    return dist(px, py, dbToPxX(thresholdDb_), curveY_ + curveH_ - 12.0f) <= 9.0f;
}

int ContourGraphWidget::hitHandle(float px, float py, bool& isIn, float& bestDist) const
{
    // Nearest support-point dot within a slightly smaller radius than a
    // node's own (dots are small and sit close to their node).
    int best = -1;
    bestDist = dotHitRadius();
    isIn = false;
    for (int i = 0; i < curve_.pointCount(); ++i)
    {
        const CurvePoint p = curve_.point(i);
        if (p.handleInputDb == 0.0 && p.handleOutputDb == 0.0) continue;
        // Only a dot that is actually drawn (inside the plot) can be grabbed.
        const float ox = dbToPxXRaw(p.inputDb + p.handleInputDb), oy = dbToPxYRaw(p.outputDb + p.handleOutputDb);
        const float ix = dbToPxXRaw(p.inputDb - p.handleInputDb), iy = dbToPxYRaw(p.outputDb - p.handleOutputDb);
        if (insidePlot(ox, oy))
        {
            const float d1 = dist(px, py, ox, oy);
            if (d1 <= bestDist) { bestDist = d1; best = i; isIn = false; }
        }
        if (insidePlot(ix, iy))
        {
            const float d2 = dist(px, py, ix, iy);
            if (d2 <= bestDist) { bestDist = d2; best = i; isIn = true; }
        }
    }
    return best;
}

bool ContourGraphWidget::thresholdLineGrabbable() const
{
    const float tx = dbToPxX(thresholdDb_);
    return tx > curveX_ + kThresholdEdgeGuardPx && tx < curveX_ + curveW_ - kThresholdEdgeGuardPx;
}

bool ContourGraphWidget::hitThresholdEdit(float px, float py) const
{
    return hitThresholdNode(px, py) || (thresholdLineGrabbable() && hitThreshold(px, py));
}

bool ContourGraphWidget::hitThreshold(float px, float py) const
{
    if (py < curveY_ || py > curveY_ + curveH_) return false;
    return std::fabs(px - dbToPxX(thresholdDb_)) <= kThresholdHitPx;
}

void ContourGraphWidget::beginGroupDrag(float x, float y)
{
    draggingGroup_ = true;
    groupPressInputDb_ = double(pxToDbX(x));
    groupPressOutputDb_ = double(pxToDbY(y));
    for (int i = 0; i < curve_.pointCount(); ++i) groupStart_[i] = curve_.point(i);
    groupStartThresholdDb_ = double(thresholdDb_);
}

void ContourGraphWidget::applyMarqueeSelection()
{
    const float rx0 = std::min(marqueeX0_, marqueeX1_), rx1 = std::max(marqueeX0_, marqueeX1_);
    const float ry0 = std::min(marqueeY0_, marqueeY1_), ry1 = std::max(marqueeY0_, marqueeY1_);
    if (!marqueeAdditive_) clearSelection();
    for (int i = 0; i < curve_.pointCount(); ++i)
    {
        const CurvePoint p = curve_.point(i);
        const float px = dbToPxX(float(p.inputDb)), py = dbToPxY(float(p.outputDb));
        if (px >= rx0 && px <= rx1 && py >= ry0 && py <= ry1) { selected_[i] = true; viaLink_[i] = false; }
    }
    // The threshold has no Y position of its own; its round handle near the
    // bottom of its line is what stands for it here -- included only when
    // that handle is inside the window, NOT merely when the window crosses
    // the line ("just include the threshold when the node at the bottom is
    // included").
    const float tx = dbToPxX(thresholdDb_), ty = curveY_ + curveH_ - 12.0f;
    if (tx >= rx0 && tx <= rx1 && ty >= ry0 && ty <= ry1) thresholdSelected_ = true;
    expandLinks();
}

void ContourGraphWidget::openMenu(int point, float x, float y)
{
    // Right-clicking a node (point >= 0) or the threshold (point < 0) that
    // isn't part of the selection selects it (and its link group) first.
    const bool inSel = point >= 0 ? selected_[point] : thresholdSelected_;
    if (!inSel)
    {
        clearSelection();
        if (point >= 0) selected_[point] = true; else thresholdSelected_ = true;
        expandLinks();
        notifySelection(point);
    }

    // What does the menu act on? See menuTarget_'s comment: the clicked item
    // alone when the selection is just its own link group (or just itself),
    // else the whole explicit selection.
    const int clickedLink = point >= 0 ? curve_.pointLink(point) : curve_.thresholdLink();
    bool ownGroupOnly = true;
    for (int i = 0; i < curve_.pointCount(); ++i)
        if (selected_[i] && !(i == point || (clickedLink > 0 && curve_.pointLink(i) == clickedLink))) ownGroupOnly = false;
    if (thresholdSelected_ && !(point < 0 || (clickedLink > 0 && curve_.thresholdLink() == clickedLink))) ownGroupOnly = false;
    for (int i = 0; i < sscomp::TransferCurve::kMaxPoints; ++i)
        menuTarget_[i] = i < curve_.pointCount() && (ownGroupOnly ? i == point : selected_[i]);
    menuTargetThr_ = ownGroupOnly ? point < 0 : thresholdSelected_;

    int nTarget = 0, nInterior = 0, nAnyLink = 0, nDiagonal = 0;
    int dirCount[3] = {};
    int linkId = -1, axisId = -1;
    bool sameLink = true, sameAxis = true;
    int inTarget[sscomp::kMaxLinkGroups + 1] = {}, inTotal[sscomp::kMaxLinkGroups + 1] = {};
    auto noteLink = [&](int lk, int axis, bool target) {
        ++inTotal[lk];
        if (!target) return;
        ++inTarget[lk];
        if (lk > 0) ++nAnyLink;
        if (linkId < 0) linkId = lk; else if (linkId != lk) sameLink = false;
        if (axis >= 0) { if (axisId < 0) axisId = axis; else if (axisId != axis) sameAxis = false; }
    };
    for (int i = 0; i < curve_.pointCount(); ++i)
    {
        noteLink(curve_.pointLink(i), curve_.pointLinkAxis(i), menuTarget_[i]);
        if (!menuTarget_[i]) continue;
        ++nTarget;
        if (curve_.pointDiagonal(i)) ++nDiagonal;
        if (!curve_.isEndpoint(i)) { ++nInterior; ++dirCount[int(curve_.pointDirection(i))]; }
    }
    noteLink(curve_.thresholdLink(), -1, menuTargetThr_); // the threshold links too, on X only
    if (menuTargetThr_) ++nTarget;
    bool partialGroup = false; // a target sits in a group that has members outside the target
    for (int g = 1; g <= sscomp::kMaxLinkGroups; ++g)
        if (inTarget[g] > 0 && inTotal[g] > inTarget[g]) partialGroup = true;

    menuCount_ = 0;
    auto add = [&](MenuAction a, const char* label, bool enabled, bool checked, bool sep) {
        menuItems_[menuCount_++] = { a, label, enabled, checked, sep };
    };
    add(MenuAction::Normal, "Normal (both directions)", nInterior > 0, nInterior > 0 && dirCount[0] == nInterior, false);
    add(MenuAction::Rising, "Rising only", nInterior > 0, nInterior > 0 && dirCount[1] == nInterior, false);
    add(MenuAction::Falling, "Falling only", nInterior > 0, nInterior > 0 && dirCount[2] == nInterior, false);
    // Link options: both axes, X only, Y only. The threshold has no Y, so a
    // Y-only link needs at least two NODES among the targets.
    const int nNodeTargets = nTarget - (menuTargetThr_ ? 1 : 0);
    const bool oneGroup = sameLink && sameAxis && linkId > 0;
    bool sepDone = false;
    if (nTarget >= 2)
    {
        if (!(oneGroup && axisId == sscomp::kLinkAxisBoth))
        {
            std::snprintf(menuLabels_[0], sizeof(menuLabels_[0]), "Link %d nodes (X + Y)", nTarget);
            add(MenuAction::Link, menuLabels_[0], true, false, true); sepDone = true;
        }
        if (!(oneGroup && axisId == sscomp::kLinkAxisX)) { add(MenuAction::LinkX, "Link X only", true, false, !sepDone); sepDone = true; }
        if (nNodeTargets >= 2 && !(oneGroup && axisId == sscomp::kLinkAxisY)) { add(MenuAction::LinkY, "Link Y only", true, false, !sepDone); sepDone = true; }
    }
    if (nAnyLink > 0)
    {
        add(MenuAction::Unlink, nTarget == 1 ? "Unlink this node" : "Unlink selected", true, false, !sepDone);
        sepDone = true;
    }
    if (partialGroup) add(MenuAction::UnlinkGroup, "Unlink whole group", true, false, false);
    // Snap to Diagonal: pins the targeted node(s) onto the unity line, from
    // then on with only one degree of freedom left (see CurvePoint::diagonal's
    // own comment), kept fully linkable like any other node. Same
    // show-only-the-useful-direction pattern as Link/Unlink just above: offer
    // "Snap" while any target isn't snapped yet, "Release" while any target
    // already is (a mixed selection can show both at once).
    const int nNodeTargetsForDiag = nTarget - (menuTargetThr_ ? 1 : 0);
    if (nNodeTargetsForDiag > 0 && nDiagonal < nNodeTargetsForDiag)
        add(MenuAction::SnapDiagonal, nNodeTargetsForDiag == 1 ? "Snap to Diagonal" : "Snap to Diagonal (all)", true, false, true);
    if (nDiagonal > 0)
        add(MenuAction::ReleaseDiagonal, nDiagonal == 1 ? "Release from Diagonal" : "Release from Diagonal", true, false, nDiagonal == nNodeTargetsForDiag);
    if (nInterior > 1) std::snprintf(menuLabels_[1], sizeof(menuLabels_[1]), "Delete %d nodes", nInterior);
    else std::snprintf(menuLabels_[1], sizeof(menuLabels_[1]), "Delete");
    add(MenuAction::Delete, menuLabels_[1], nInterior > 0, false, true);

    bool anyEnabled = false;
    for (int i = 0; i < menuCount_; ++i) anyEnabled = anyEnabled || menuItems_[i].enabled;
    if (!anyEnabled) { repaint(); return; } // e.g. a lone unlinked threshold: nothing to offer
    menuH_ = 4.0f;
    for (int i = 0; i < menuCount_; ++i) menuH_ += kMenuItemH + (menuItems_[i].sepBefore ? kMenuSepH : 0.0f);
    menuH_ += 4.0f;
    menuX_ = std::max(2.0f, std::min(x, float(getWidth()) - kMenuW - 2.0f));
    menuY_ = std::max(2.0f, std::min(y, float(getHeight()) - menuH_ - 2.0f));
    menuHover_ = -1;
    menuOpen_ = true;
    repaint();
}

int ContourGraphWidget::menuItemAt(float px, float py) const
{
    if (!menuOpen_ || px < menuX_ || px > menuX_ + kMenuW) return -1;
    float y = menuY_ + 4.0f;
    for (int i = 0; i < menuCount_; ++i)
    {
        y += menuItems_[i].sepBefore ? kMenuSepH : 0.0f;
        if (py >= y && py < y + kMenuItemH) return i;
        y += kMenuItemH;
    }
    return -1;
}

void ContourGraphWidget::drawMenu()
{
    if (!menuOpen_) return;
    beginPath();
    roundedRect(menuX_, menuY_, kMenuW, menuH_, 4.0f);
    fillColor(Color(34, 37, 44));
    fill();
    strokeColor(Color(90, 96, 108));
    strokeWidth(1.0f);
    stroke();

    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(11.5f);
    float y = menuY_ + 4.0f;
    for (int i = 0; i < menuCount_; ++i)
    {
        const MenuItem& it = menuItems_[i];
        if (it.sepBefore)
        {
            beginPath();
            moveTo(menuX_ + 6.0f, y + kMenuSepH * 0.5f);
            lineTo(menuX_ + kMenuW - 6.0f, y + kMenuSepH * 0.5f);
            strokeColor(Color(60, 64, 74));
            strokeWidth(1.0f);
            stroke();
            y += kMenuSepH;
        }
        if (i == menuHover_ && it.enabled)
        {
            beginPath();
            roundedRect(menuX_ + 3.0f, y, kMenuW - 6.0f, kMenuItemH, 3.0f);
            fillColor(Color(58, 80, 112));
            fill();
        }
        if (it.checked)
        {
            beginPath();
            circle(menuX_ + 12.0f, y + kMenuItemH * 0.5f, 3.0f);
            fillColor(Color(120, 200, 255));
            fill();
        }
        const bool danger = it.action == MenuAction::Delete;
        fillColor(!it.enabled ? Color(100, 104, 112) : danger ? Color(240, 170, 165) : Color(225, 228, 234));
        textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
        text(menuX_ + 22.0f, y + kMenuItemH * 0.5f, it.label, nullptr);
        y += kMenuItemH;
    }
}

void ContourGraphWidget::runMenuAction(MenuAction a)
{
    // Everything acts on menuTarget_/menuTargetThr_ (fixed when the menu
    // opened), not on the live selection -- see menuTarget_'s comment.
    switch (a)
    {
    case MenuAction::Normal:
    case MenuAction::Rising:
    case MenuAction::Falling:
    {
        const sscomp::PointDirection d = a == MenuAction::Rising ? sscomp::PointDirection::RisingOnly
                                       : a == MenuAction::Falling ? sscomp::PointDirection::FallingOnly
                                       : sscomp::PointDirection::Both;
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i] && !curve_.isEndpoint(i)) curve_.setPointDirection(i, d);
        notifyCurveChanged();
        break;
    }
    case MenuAction::Link:
    case MenuAction::LinkX:
    case MenuAction::LinkY:
    {
        const int axis = a == MenuAction::LinkX ? sscomp::kLinkAxisX : a == MenuAction::LinkY ? sscomp::kLinkAxisY : sscomp::kLinkAxisBoth;
        // Reuse the smallest group id already among the targets (merging
        // groups -- and re-linking a group with another axis converts it),
        // else a free one.
        int id = 0;
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i] && curve_.pointLink(i) > 0) id = id == 0 ? curve_.pointLink(i) : std::min(id, curve_.pointLink(i));
        if (menuTargetThr_ && curve_.thresholdLink() > 0) id = id == 0 ? curve_.thresholdLink() : std::min(id, curve_.thresholdLink());
        if (id == 0) id = curve_.freeLinkId();
        if (id == 0) break;
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i]) curve_.setPointLink(i, id, axis);
        // The threshold only follows X: not part of a Y-only group.
        if (menuTargetThr_) curve_.setThresholdLink(axis == sscomp::kLinkAxisY ? 0 : id);
        curve_.normalizeLinks();
        notifyCurveChanged();
        break;
    }
    case MenuAction::Unlink:
    {
        // Just the targeted nodes leave their group; the rest stay linked
        // (a group left with fewer than two members dissolves by itself).
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i]) curve_.setPointLink(i, 0);
        if (menuTargetThr_) curve_.setThresholdLink(0);
        curve_.normalizeLinks();
        notifyCurveChanged();
        break;
    }
    case MenuAction::UnlinkGroup:
    {
        bool ids[sscomp::kMaxLinkGroups + 1] = {};
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i]) ids[curve_.pointLink(i)] = true;
        if (menuTargetThr_) ids[curve_.thresholdLink()] = true;
        ids[0] = false;
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (ids[curve_.pointLink(i)]) curve_.setPointLink(i, 0);
        if (ids[curve_.thresholdLink()]) curve_.setThresholdLink(0);
        curve_.normalizeLinks();
        notifyCurveChanged();
        break;
    }
    case MenuAction::SnapDiagonal:
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i]) curve_.setPointDiagonal(i, true);
        notifyCurveChanged();
        break;
    case MenuAction::ReleaseDiagonal:
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (menuTarget_[i]) curve_.setPointDiagonal(i, false);
        notifyCurveChanged();
        break;
    case MenuAction::Delete:
        removeMarked(menuTarget_);
        break;
    }
    repaint();
}

bool ContourGraphWidget::tryGrabHandle(float px, float py, int hitNode)
{
    bool hIn = false;
    float hDist = 0.0f;
    const int hh = hitHandle(px, py, hIn, hDist);
    if (hh < 0) return false;
    // A press on the node itself always grabs the node. A short handle puts its dot
    // a few pixels from the node centre, and the (bigger) dot radius then covered
    // everything on the dot's side of the node -- on the right end node at 6dB, whose
    // other dot lies above the plot and is invisible, the whole lower half of the
    // node grabbed the dot instead, making that node hard to move. Beyond the node's core radius the
    // nearer of node and dot wins, as before.
    if (hitNode >= 0)
    {
        const CurvePoint hp = curve_.point(hitNode);
        const float dNode = dist(px, py, dbToPxX(float(hp.inputDb)), dbToPxY(float(hp.outputDb)));
        if (dNode <= kNodeCoreRadiusPx || hDist >= dNode) return false;
    }
    // Double-click on a support point resets its node's handle to dx = dy = 0
    // (a corner again -- for an end node, back to no slope).
    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400);
    if (hh == lastHandleClickPoint_ && !lastHandleClickWasDrag_ && (now - lastHandleClickTime_) < kDoubleClickWindow)
    {
        lastHandleClickPoint_ = -1;
        curve_.setPointHandle(hh, 0.0, 0.0);
        hoverHandle_ = -1;
        notifyCurveChanged();
        repaint();
        return true;
    }
    lastHandleClickPoint_ = hh;
    lastHandleClickTime_ = now;
    lastHandleClickWasDrag_ = false; // set true in onMotion if this press turns into a drag
    handlePressX_ = px; handlePressY_ = py;

    clearSelection();
    selected_[hh] = true;
    expandLinks();
    notifySelection(hh);
    draggingHandle_ = true;
    draggingHandleLeft_ = true;
    draggingHandlePoint_ = hh;
    draggingHandleIsIn_ = hIn;
    repaint();
    return true;
}

bool ContourGraphWidget::beginPointPress(int hit, float x, float y, bool asGroup)
{
    // Double-click on an *interior* node removes it -- the two fixed
    // endpoints never respond to this (see TransferCurve::removePoint()'s
    // own refusal; checked here too so a double-click on an endpoint
    // doesn't even start a drag on the second click).
    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400);
    if (hit == lastClickPoint_ && !lastClickWasDrag_ && (now - lastClickTime_) < kDoubleClickWindow && !curve_.isEndpoint(hit))
    {
        curve_.removePoint(hit);
        lastClickPoint_ = -1;
        clearSelection();
        notifyCurveChanged();
        notifySelection(-1);
        repaint();
        return true;
    }
    lastClickPoint_ = hit;
    lastClickTime_ = now;
    lastClickWasDrag_ = false; // set true in onMotion below if this very press turns into a drag
    pointPressX_ = x; pointPressY_ = y;

    if (asGroup) beginGroupDrag(x, y);
    else draggingPoint_ = hit;
    repaint();
    return true;
}

bool ContourGraphWidget::removeMarked(const bool* mark)
{
    bool any = false;
    // Highest index first, so removing one never shifts a lower index we
    // still have to look at.
    for (int i = curve_.pointCount() - 1; i >= 0; --i)
    {
        if (!mark[i] || curve_.isEndpoint(i)) continue;
        curve_.removePoint(i); // also drops it from its link group
        any = true;
    }
    if (!any) return false;
    clearSelection();
    lastClickPoint_ = -1;
    notifyCurveChanged();
    notifySelection(-1);
    repaint();
    return true;
}

bool ContourGraphWidget::deleteSelected()
{
    // Same targeting rule as the right-click menu (see menuTarget_): when
    // the selection is only the clicked node's own link group -- what a plain
    // click on a linked node produces -- Delete removes that node alone, not
    // its whole group. An explicit multi-selection (window / Shift-click,
    // primary_ == -1) is deleted as a whole.
    bool mark[sscomp::TransferCurve::kMaxPoints] = {};
    bool ownGroupOnly = false;
    if (primary_ >= 0 && primary_ < curve_.pointCount() && selected_[primary_])
    {
        const int lk = curve_.pointLink(primary_);
        ownGroupOnly = true;
        for (int i = 0; i < curve_.pointCount(); ++i)
            if (selected_[i] && !(i == primary_ || (lk > 0 && curve_.pointLink(i) == lk))) ownGroupOnly = false;
        if (thresholdSelected_ && !(lk > 0 && curve_.thresholdLink() == lk)) ownGroupOnly = false;
    }
    for (int i = 0; i < curve_.pointCount(); ++i) mark[i] = ownGroupOnly ? i == primary_ : selected_[i];
    return removeMarked(mark);
}

std::string ContourGraphWidget::tooltipAt(double lx, double ly) const
{
    if (menuOpen_) return std::string();
    const float x = float(lx), y = float(ly);
    if (grSpeedSwitch_.contains(x, y)) return GrSpeedSwitch::tooltip();
    char buf[160]; // the longest line ("Linked (group N, X only): ... its horizontal moves.") needs ~75

    {
        bool isIn = false;
        float d = 0.0f;
        const int hh = hitHandle(x, y, isIn, d);
        if (hh >= 0)
        {
            std::snprintf(buf, sizeof(buf), "Support point of node %d: ", hh + 1);
            return std::string(buf) + "drag to change how the curve bends through this node (the two dots are mirrored). Double-click to reset it to 0/0 (a corner).";
        }
    }

    const int hit = hitPoint(x, y);
    if (hit >= 0)
    {
        const CurvePoint p = curve_.point(hit);
        const sscomp::PointDirection dir = curve_.pointDirection(hit);
        std::snprintf(buf, sizeof(buf), "Node %d: in %.1f dB, out %.1f dB. ", hit + 1, p.inputDb, p.outputDb);
        std::string t = buf;
        if (curve_.isEndpoint(hit))
            t += "End node: fixed at the graph edge, only its height moves. ";
        else if (dir == sscomp::PointDirection::RisingOnly)
            t += "Active only while the level is rising. ";
        else if (dir == sscomp::PointDirection::FallingOnly)
            t += "Active only while the level is falling. ";
        else
            t += "Active in both directions. ";
        if (p.handleInputDb != 0.0 || p.handleOutputDb != 0.0)
            t += "Smooth (has support points). ";
        else
            t += "Corner (no support points). ";
        if (p.link > 0)
        {
            const char* ax = p.linkAxis == sscomp::kLinkAxisX ? ", X only" : p.linkAxis == sscomp::kLinkAxisY ? ", Y only" : "";
            std::snprintf(buf, sizeof(buf), "Linked (group %d%s): the other members follow %s. ", p.link, ax,
                          p.linkAxis == sscomp::kLinkAxisX ? "its horizontal moves" : p.linkAxis == sscomp::kLinkAxisY ? "its vertical moves" : "it");
            t += buf;
        }
        if (p.diagonal)
            t += "Snapped to the diagonal: only slides along the unity line. ";
        if (tool_ == EditTool::Edit)
            t += "Drag to move (all selected nodes move together). Double-click or Delete removes it. Right-click: menu (Rising/Falling, Link, Snap to Diagonal, Delete); right-drag pulls out support points.";
        else
            t += "Drag to move it, double-click to remove it, right-click for the menu.";
        return t;
    }

    const bool onThr = tool_ == EditTool::Edit ? hitThresholdEdit(x, y) : hitThresholdNode(x, y);
    if (onThr)
    {
        std::snprintf(buf, sizeof(buf), "Threshold: %.1f dB. ", double(thresholdDb_));
        std::string t = buf;
        if (curve_.thresholdLink() > 0)
        {
            std::snprintf(buf, sizeof(buf), "Linked (group %d): follows its linked nodes left/right. ", curve_.thresholdLink());
            t += buf;
        }
        return t + "Drag left/right. The curve only acts on the shaded side. Right-click to link it to selected nodes.";
    }
    if (hitTrailLegend(x, y)) return "Heat trail: scroll here to change how long it stays visible.";
    return std::string();
}

bool ContourGraphWidget::onMouse(const MouseEvent& ev)
{
    const float x = float(ev.pos.getX()), y = float(ev.pos.getY());

    // An open right-click menu owns the next press: a left press on an
    // enabled item runs it, anything else just closes the menu -- and the
    // press is swallowed either way, so dismissing the menu can never also
    // place or grab a node underneath it.
    if (menuOpen_)
    {
        if (!ev.press) return true;
        const int item = menuItemAt(x, y);
        const bool run = ev.button == 1 && item >= 0 && menuItems_[item].enabled;
        const MenuAction action = run ? menuItems_[item].action : MenuAction::Normal;
        menuOpen_ = false;
        if (run) runMenuAction(action);
        repaint();
        return true;
    }

    if (ev.press && ev.button == 1)
    {
        const int seg = grSpeedSwitch_.hit(x, y);
        if (seg >= 0)
        {
            const GrMeterSpeed s = GrMeterSpeed(seg);
            if (s != grSpeed_)
            {
                grSpeed_ = s;
                if (onGrMeterSpeedChanged_) onGrMeterSpeedChanged_(s);
                repaint();
            }
            return true;
        }
    }

    // Right button (Edit tool only -- the Draw tools never grab existing
    // nodes): on a node, a plain click cycles its hysteresis direction tag;
    // a click-AND-DRAG instead pulls out / reshapes its tangent handle,
    // applied retroactively to an EXISTING node (including the two fixed
    // endpoints, unlike the direction-cycle path). Same press-then-decide
    // dead-zone pattern as every other click-vs-drag gesture in this file:
    // nothing happens until either the drag crosses kPointDragThresholdPx
    // (promotes to handle-adjust, in onMotion) or release arrives first (a
    // plain click, cycles direction). Uses the same handle-radius-padded
    // hit box as the left button, since the two fixed endpoints sit exactly
    // on the plot's own left/right edges.
    if (ev.button == kMouseButtonRight)
    {
        if (ev.press)
        {
            const bool nearGraph = x >= curveX_ - kHandleHitRadiusPx && x <= curveX_ + curveW_ + kHandleHitRadiusPx &&
                                    y >= curveY_ - kHandleHitRadiusPx && y <= curveY_ + curveH_ + kHandleHitRadiusPx;
            if (!nearGraph) return false;
            const int hit = hitPoint(x, y);
            // The threshold can be right-clicked too (for Link/Unlink);
            // -2 stands for it. It has no support points, so it never turns
            // into a handle drag.
            const bool thr = hit < 0 && (isDrawTool() ? hitThresholdNode(x, y) : hitThresholdEdit(x, y));
            if (hit < 0 && !thr) return false;
            rightPressPending_ = true;
            rightPressPoint_ = hit >= 0 ? hit : -2;
            rightPressX_ = x; rightPressY_ = y;
            return true;
        }
        // Release.
        if (draggingHandle_) { draggingHandle_ = false; draggingHandlePoint_ = -1; repaint(); return true; }
        if (rightPressPending_)
        {
            rightPressPending_ = false;
            // Never crossed the drag dead zone -- a plain right-click opens
            // the node menu (Normal / Rising / Falling / Link / Unlink /
            // Delete). This used to cycle the direction tag directly.
            openMenu(rightPressPoint_ == -2 ? -1 : rightPressPoint_, x, y);
            rightPressPoint_ = -1;
            return true;
        }
        return false;
    }

    if (ev.button != 1) return false;
    const bool shiftHeld = (ev.mod & kModifierShift) != 0;

    if (ev.press)
    {
        // A new left press always starts from a clean slate. If the previous gesture's release never reached
        // this widget (released over another widget or outside the window), its state would still be live --
        // e.g. a stale "pending add" turned the next node press into a marquee -- so drop all of it first.
        pendingAddPoint_ = false; marqueeActive_ = false; placingSmoothPoint_ = false; placingSmoothIndex_ = -1;
        draggingPoint_ = -1; draggingGroup_ = false; draggingThreshold_ = false;
        draggingHandle_ = false; draggingHandleLeft_ = false; draggingHandlePoint_ = -1;
        rightPressPending_ = false;
        // Padded by the handle's own hit radius on every side -- without
        // this, a click aimed at the *outer* half of a fixed-X endpoint's
        // handle (which sits exactly on curveX_/curveX_+curveW_, so half
        // its own circle falls just outside this rect) was silently
        // rejected before hitPoint() ever got a chance to see it, so the
        // end nodes sometimes took several tries to grab. Only the
        // handle/threshold hit-test gets this margin --
        // placing a node and marquee-select still require being properly
        // inside the plot rect (checked where they're actually reached).
        const bool nearGraph = x >= curveX_ - kHandleHitRadiusPx && x <= curveX_ + curveW_ + kHandleHitRadiusPx &&
                                y >= curveY_ - kHandleHitRadiusPx && y <= curveY_ + curveH_ + kHandleHitRadiusPx;
        // Placing a node needs the plot; a selection WINDOW in the Edit tool
        // does not: it may start anywhere in this widget, in the margins
        // around the plot too. Before, a press more than a few pixels
        // outside the plot was thrown away here, so a window started just
        // outside a corner to sweep up the nodes sitting in it -- "especially
        // in the upper left hand corner its almost never working to start a
        // rectangle selection" -- silently did nothing.
        if (!nearGraph && isDrawTool()) return false;
        const bool inGraph = x >= curveX_ && x <= curveX_ + curveW_ && y >= curveY_ && y <= curveY_ + curveH_;
        // Embedded plugin UIs don't necessarily hold keyboard focus just
        // because something inside was clicked -- grab it so the Delete key
        // (see deleteSelected()) actually reaches the UI, same reasoning as
        // ParamStripWidget::beginEdit().
        getWindow().focus();

        // ---- Draw tools: every press places a NEW node ------------------
        if (isDrawTool())
        {
            // The threshold stays reachable through its own round handle
            // only -- its whole dashed line is NOT a grab target here, or
            // placing a node near it would keep dragging the threshold.
            if (hitThresholdNode(x, y))
            {
                // A linked threshold drags its group (X only); an unlinked
                // one is just dragged by itself, as before.
                if (curve_.thresholdLink() > 0)
                {
                    clearSelection();
                    thresholdSelected_ = true;
                    expandLinks();
                    notifySelection(-1);
                    beginGroupDrag(x, y);
                }
                else
                    draggingThreshold_ = true;
                repaint();
                return true;
            }
            const int hitN = hitPoint(x, y);
            // A support point takes priority over placing a node: pressing
            // one grabs it (or the nearer of it and its node).
            if (tryGrabHandle(x, y, hitN)) return true;
            // An existing node under the cursor is grabbed directly (drag =
            // move, double-click = remove) instead of placing a new one on
            // top of it. Only that one node -- unless it's linked, where
            // its whole group comes along, exactly as if all were selected.
            if (hitN >= 0)
            {
                clearSelection();
                selected_[hitN] = true;
                expandLinks();
                notifySelection(hitN);
                return beginPointPress(hitN, x, y, selectedCount() > 1);
            }
            if (hitTrailLegend(x, y)) return true;
            if (!inGraph) return false;
            pendingAddPoint_ = true;
            pendingAddShiftHeld_ = false;
            pressX_ = x; pressY_ = y;
            return true;
        }

        // ---- Edit tool --------------------------------------------------
        int hit = hitPoint(x, y);

        // A support point (Bezier tangent dot) under the cursor takes the
        // press unless the node's own circle is closer -- "the supportpoints
        // of the bezier spline" must be movable with the mouse.
        if (!shiftHeld && tryGrabHandle(x, y, hit)) return true;
        const bool hitThr = hit < 0 && hitThresholdEdit(x, y);

        // Shift-click toggles this node's (or the threshold's) own
        // membership in the multi-selection without starting any drag:
        // building the selection up is a separate
        // gesture from moving it.
        if (shiftHeld && (hit >= 0 || hitThr))
        {
            if (hit >= 0)
            {
                selected_[hit] = !selected_[hit];
                viaLink_[hit] = false;
                // A linked node toggles as a whole group.
                const int lk = curve_.pointLink(hit);
                if (lk > 0)
                {
                    for (int j = 0; j < curve_.pointCount(); ++j)
                        if (j != hit && curve_.pointLink(j) == lk) { selected_[j] = selected_[hit]; viaLink_[j] = selected_[hit]; }
                    if (curve_.thresholdLink() == lk) thresholdSelected_ = selected_[hit];
                }
            }
            else
            {
                thresholdSelected_ = !thresholdSelected_;
                const int tl = curve_.thresholdLink();
                if (tl > 0)
                    for (int j = 0; j < curve_.pointCount(); ++j)
                        if (curve_.pointLink(j) == tl) selected_[j] = thresholdSelected_;
            }
            notifySelection(hit >= 0 && selected_[hit] ? hit : -1);
            primary_ = -1; // Shift-click builds an explicit multi-selection
            repaint();
            return true;
        }

        if (hit >= 0 || hitThr)
        {
            // Clicking something already part of a 2+ selection drags the
            // whole group together; clicking anything else collapses the
            // selection down to just this one item first, then falls
            // through to the ordinary single-item drag/double-click-to-
            // remove path below.
            const bool isMember = hit >= 0 ? selected_[hit] : thresholdSelected_;
            if (!(isMember && selectedCount() > 1))
            {
                clearSelection();
                if (hit >= 0) selected_[hit] = true; else thresholdSelected_ = true;
                expandLinks();
                notifySelection(hit);
            }
            if (selectedCount() > 1)
            {
                // Through beginPointPress(asGroup) when a node was hit, so a
                // double-click on a group member removes that one node
                // (linked nodes included -- "a linked node should be
                // deletable").
                if (hit >= 0) return beginPointPress(hit, x, y, true);
                beginGroupDrag(x, y);
                repaint();
                return true;
            }
        }

        if (hit >= 0) return beginPointPress(hit, x, y);

        if (hitThr)
        {
            draggingThreshold_ = true;
            repaint();
            return true;
        }

        // The trail legend swatch overlaps the top of the curve area -- a
        // click there is for scrolling its own duration.
        if (hitTrailLegend(x, y)) return true;

        // Empty plot area in Edit: a drag becomes a marquee (as it used to);
        // a plain click just deselects (Shift keeps
        // the selection and makes the marquee additive). Bounded to this
        // widget's own rectangle -- unlike the Draw tools' plot-only check
        // above, a selection window is deliberately allowed to start
        // anywhere in the margins too (see this file's own header comment),
        // but not outside this widget entirely: DPF calls onMouse() on
        // every sibling subwidget in reverse construction order until one
        // returns true (Widget::PrivateData::giveMouseEventForSubWidgets()),
        // with no bounds pre-filter of its own -- a bare `return true` here
        // for ANY position, however far off, made whichever of graphA_/
        // graphB_ was constructed LAST swallow every press meant for the
        // OTHER one (it never saw the press at all) the moment two
        // ContourGraphWidgets are visible together, i.e. exactly Dual-Mono/
        // Mid-Side, where the right-hand graph's points couldn't be moved.
        // A single graph_ in
        // Linked mode never exposed this -- nothing else to steal from.
        if (x < 0.0f || y < 0.0f || x > float(getWidth()) || y > float(getHeight())) return false;
        pendingAddPoint_ = true;
        pendingAddShiftHeld_ = shiftHeld;
        pressX_ = x; pressY_ = y;
        marqueeAdditive_ = shiftHeld;
        return true;
    }

    // Release.
    if (draggingGroup_) { draggingGroup_ = false; repaint(); return true; }
    if (marqueeActive_)
    {
        applyMarqueeSelection();
        marqueeActive_ = false;
        int primary = -1;
        for (int i = 0; i < curve_.pointCount() && primary < 0; ++i) if (selected_[i]) primary = i;
        notifySelection(primary);
        primary_ = -1; // a window is an explicit multi-selection: Delete takes all of it
        repaint();
        return true;
    }
    if (draggingHandle_ && draggingHandleLeft_)
    {
        draggingHandle_ = false; draggingHandleLeft_ = false; draggingHandlePoint_ = -1;
        repaint();
        return true;
    }
    if (draggingPoint_ >= 0) { draggingPoint_ = -1; repaint(); return true; }
    if (draggingThreshold_) { draggingThreshold_ = false; repaint(); return true; }
    if (placingSmoothPoint_) { placingSmoothPoint_ = false; placingSmoothIndex_ = -1; repaint(); return true; }
    if (pendingAddPoint_)
    {
        pendingAddPoint_ = false;
        if (dist(x, y, pressX_, pressY_) <= kAddPointDeadZonePx)
        {
            if (isDrawTool())
            {
                // Never crossed the dead zone -- a plain click, adds a
                // corner node (zero-length handle, straight lines to both
                // neighbors), tagged per the active Draw tool.
                const int newIdx = curve_.addPoint(double(pxToDbX(x)), double(pxToDbY(y)));
                if (newIdx >= 0)
                {
                    curve_.setPointDirection(newIdx, directionForTool());
                    clearSelection();
                    selected_[newIdx] = true; // select what you just created
                    notifyCurveChanged();
                    notifySelection(newIdx);
                }
            }
            else if (!pendingAddShiftHeld_ && (selectedCount() > 0))
            {
                clearSelection();
                notifySelection(-1);
            }
            repaint();
        }
        return true;
    }
    return false;
}

bool ContourGraphWidget::onMotion(const MotionEvent& ev)
{
    const float x = float(ev.pos.getX()), y = float(ev.pos.getY());

    if (menuOpen_)
    {
        const int item = menuItemAt(x, y);
        if (item != menuHover_) { menuHover_ = item; repaint(); }
        return true;
    }

    // Right-click-drag-to-adjust-a-handle -- see onMouse()'s own comment
    // on the right-button press/release pair this completes.
    if (rightPressPending_)
    {
        if (dist(x, y, rightPressX_, rightPressY_) > kPointDragThresholdPx)
        {
            if (rightPressPoint_ < 0) { rightPressPending_ = false; rightPressPoint_ = -1; return true; } // threshold: nothing to drag
            draggingHandle_ = true;
            draggingHandleLeft_ = false;
            // (dx is signed, so the bright dot simply follows the mouse -- also for
            // the right end node, whose dot then lies to the LEFT of it.)
            draggingHandleIsIn_ = false;
            draggingHandlePoint_ = rightPressPoint_;
            rightPressPending_ = false;
            rightPressPoint_ = -1;
            // Falls through to draggingHandle_ below so this very motion
            // sample (the one that crossed the dead zone) is applied
            // immediately too, not dropped.
        }
        else
        {
            return true;
        }
    }
    if (draggingHandle_)
    {
        if (draggingHandleLeft_ && dist(x, y, handlePressX_, handlePressY_) > kPointDragThresholdPx) lastHandleClickWasDrag_ = true;
        // The dot you hold is always THE (white) support point. The pair is
        // mirror-symmetric (d and -d are the same tangent), so holding the dim
        // mirrored dot just relabels the pair: flip d once the drag really
        // starts, then follow the mouse with a SIGNED dx -- which is what lets
        // the right end node's visible dot be the white one, and lets any dot be
        // dragged across its node ("cannot place a negative dx").
        if (draggingHandleLeft_ && draggingHandleIsIn_ && lastHandleClickWasDrag_)
        {
            const CurvePoint q = curve_.point(draggingHandlePoint_);
            curve_.setPointHandle(draggingHandlePoint_, -q.handleInputDb, -q.handleOutputDb);
            draggingHandleIsIn_ = false;
        }
        // The handle is symmetric: dragging the outgoing dot follows the
        // cursor directly, dragging the incoming dot mirrors it.
        const CurvePoint p = curve_.point(draggingHandlePoint_);
        const double sign = draggingHandleIsIn_ ? -1.0 : 1.0;
        const double dx = sign * (double(pxToDbX(x)) - p.inputDb);
        const double dy = sign * (double(pxToDbY(y)) - p.outputDb);
        curve_.setPointHandle(draggingHandlePoint_, dx, dy);
        notifyCurveChanged();
        repaint();
        return true;
    }

    if (draggingGroup_)
    {
        // Same rule as a single-node drag (see lastClickWasDrag_): a group
        // drag that really moved must not read as the first click of a
        // double-click-to-remove.
        if (dist(x, y, pointPressX_, pointPressY_) > kPointDragThresholdPx) lastClickWasDrag_ = true;
        const double deltaInDb = double(pxToDbX(x)) - groupPressInputDb_;
        const double deltaOutDb = double(pxToDbY(y)) - groupPressOutputDb_;
        // Right-most first when moving right (left-most first otherwise), so
        // a member is never clamped against a neighbour that just hasn't
        // moved yet -- otherwise two close members deformed while dragging.
        const int np = curve_.pointCount();
        for (int k = 0; k < np; ++k)
        {
            const int i = deltaInDb > 0.0 ? np - 1 - k : k;
            if (!selected_[i]) continue;
            // A node that is only here through a link follows just the axis
            // the group is linked on (X-only / Y-only); everything explicitly
            // selected -- and the node you are dragging -- moves freely.
            const int ax = viaLink_[i] ? curve_.pointLinkAxis(i) : sscomp::kLinkAxisBoth;
            const double dx = ax == sscomp::kLinkAxisY ? 0.0 : deltaInDb;
            const double dy = ax == sscomp::kLinkAxisX ? 0.0 : deltaOutDb;
            curve_.setPoint(i, groupStart_[i].inputDb + dx, groupStart_[i].outputDb + dy);
        }
        if (thresholdSelected_)
        {
            // X only -- the threshold has no Y of its own to move.
            const float t = float(std::max(TransferCurve::kMinDb, std::min(TransferCurve::kMaxDb, groupStartThresholdDb_ + deltaInDb)));
            thresholdDb_ = t;
            if (onThresholdChanged_) onThresholdChanged_(t);
        }
        notifyCurveChanged();
        repaint();
        return true;
    }
    if (marqueeActive_)
    {
        marqueeX1_ = x; marqueeY1_ = y;
        repaint();
        return true;
    }
    if (draggingPoint_ >= 0)
    {
        // See lastClickWasDrag_'s own comment: once this press has moved
        // enough to count as a real drag, a quick re-grab of the same
        // point right after release must not read as a double-click.
        if (dist(x, y, pointPressX_, pointPressY_) > kPointDragThresholdPx) lastClickWasDrag_ = true;
        curve_.setPoint(draggingPoint_, double(pxToDbX(x)), double(pxToDbY(y)));
        notifyCurveChanged();
        repaint();
        return true;
    }
    if (draggingThreshold_)
    {
        const float newThreshold = std::max(float(TransferCurve::kMinDb), std::min(float(TransferCurve::kMaxDb), pxToDbX(x)));
        thresholdDb_ = newThreshold;
        if (onThresholdChanged_) onThresholdChanged_(newThreshold);
        repaint();
        return true;
    }
    if (pendingAddPoint_)
    {
        // Moved past the dead zone before release. Edit tool -> promote to
        // a rubber-band marquee. Draw tools -> place a smooth node right at
        // the press position and start shaping its own tangent handle live
        // from here.
        if (dist(x, y, pressX_, pressY_) > kAddPointDeadZonePx)
        {
            pendingAddPoint_ = false;
            if (!isDrawTool())
            {
                marqueeActive_ = true;
                marqueeX0_ = pressX_; marqueeY0_ = pressY_;
                marqueeX1_ = x; marqueeY1_ = y;
            }
            else
            {
                const double inputDb = double(pxToDbX(pressX_));
                const double outputDb = double(pxToDbY(pressY_));
                const int newIdx = curve_.addPoint(inputDb, outputDb);
                if (newIdx >= 0)
                {
                    curve_.setPointDirection(newIdx, directionForTool());
                    clearSelection();
                    selected_[newIdx] = true;
                    placingSmoothPoint_ = true;
                    placingSmoothIndex_ = newIdx;
                    const double dx = double(pxToDbX(x)) - inputDb;
                    const double dy = double(pxToDbY(y)) - outputDb;
                    curve_.setPointHandle(newIdx, dx, dy);
                    notifyCurveChanged();
                    notifySelection(newIdx);
                }
            }
            repaint();
        }
        return true;
    }
    if (placingSmoothPoint_)
    {
        if (placingSmoothIndex_ >= 0)
        {
            const CurvePoint p = curve_.point(placingSmoothIndex_);
            const double dx = double(pxToDbX(x)) - p.inputDb;
            const double dy = double(pxToDbY(y)) - p.outputDb;
            curve_.setPointHandle(placingSmoothIndex_, dx, dy);
            notifyCurveChanged();
            repaint();
        }
        return true;
    }

    hoverX_ = x; hoverY_ = y;
    int hit = hitPoint(x, y);
    bool hIn = false;
    float hDist = 0.0f;
    int hh = hitHandle(x, y, hIn, hDist);
    if (hh >= 0 && hit >= 0)
    {
        const CurvePoint hp = curve_.point(hit);
        if (hDist >= dist(x, y, dbToPxX(float(hp.inputDb)), dbToPxY(float(hp.outputDb)))) hh = -1;
    }
    if (hh >= 0) hit = -1;
    // Draw tools: only the threshold's round handle (not its whole line)
    // is a target, and the ghost node follows the cursor -- so repaint on
    // every move there, it's what shows where the next click lands.
    const bool onThreshold = hit < 0 && hh < 0 && (isDrawTool() ? hitThresholdNode(x, y) : hitThresholdEdit(x, y));
    const bool onLegend = hit < 0 && hh < 0 && !onThreshold && hitTrailLegend(x, y);
    if (isDrawTool()) repaint();
    if (hit != hoverPoint_ || hh != hoverHandle_ || hIn != hoverHandleIsIn_ ||
        onThreshold != hoverThreshold_ || onLegend != hoverTrailLegend_)
    {
        hoverPoint_ = hit;
        hoverHandle_ = hh;
        hoverHandleIsIn_ = hIn;
        hoverThreshold_ = onThreshold;
        hoverTrailLegend_ = onLegend;
        repaint();
    }
    return false;
}

bool ContourGraphWidget::onScroll(const ScrollEvent& ev)
{
    // Scroll over the trail's own legend swatch changes how far back it
    // reaches (a longer trail costs more CPU, so it's adjustable). Gated
    // to hovering that small
    // swatch specifically (rather than "scroll anywhere over the curve",
    // the convention CompressorGraphWidget's own Knee nudge uses) since
    // this widget has no knee or other curve-shape scalar competing for
    // the gesture, and a precise hit region keeps it an intentional,
    // discoverable action rather than an easy-to-trigger-by-accident one.
    if (!hitTrailLegend(float(ev.pos.getX()), float(ev.pos.getY()))) return false;
    const float dir = ev.delta.getY() > 0.0f ? 1.0f : (ev.delta.getY() < 0.0f ? -1.0f : 0.0f);
    if (dir == 0.0f) return true;
    const float step = (ev.mod & kModifierAlt) ? 0.025f : 0.25f; // Alt = 10x finer, as everywhere
    trailSeconds_ = std::max(kTrailSecondsMin, std::min(kTrailSecondsMax, trailSeconds_ + dir * step));
    if (onTrailSecondsChanged_) onTrailSecondsChanged_(trailSeconds_);
    repaint();
    return true;
}

END_NAMESPACE_DGL
