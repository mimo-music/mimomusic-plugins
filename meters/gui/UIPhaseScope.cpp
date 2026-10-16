// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIPhaseScope.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::GoniometerWidget;
using DGL_NAMESPACE::LevelMeterWidget;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;
using DGL_NAMESPACE::StereoMeterWidget;
using DGL_NAMESPACE::TooltipOverlay;

UIPhaseScope::UIPhaseScope()
    : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
{
    // Scale with the host's or desktop's scale factor (Ardour's GUI
    // scaling, else Xft.dpi; DPF_SCALE_FACTOR overrides): DPF draws
    // everything scaled and zooms when the window is resized, keeping the
    // aspect ratio; the layout always sees the default (logical) size. The
    // zoom is relative to the design size (setAutoScaleBaseSize(), DPF
    // patch dpf-patches/0002), so the window can also be made smaller than
    // it, down to half (the contents then shrink with it). Not
    // UI(w, h, true): DPF already creates the window at the scaled size,
    // and that form resizes it by the scale factor a second time.
    // Requires every widget to share this editor's NanoVG context (their
    // constructors take a NanoTopLevelWidget*) -- a widget with its own
    // context is drawn into an unscaled viewport.
    // Phase Scope: the full view may be dragged down to the compact view's
    // size, any shape -- below kAutoCompactBelow it switches to compact
    // (updateAutoCompact()). See setCompact() for the compact view's sizes.
    getWindow().setAutoScaleBaseSize(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT);
    setGeometryConstraints(kCompactWidth, kCompactHeight, false, true, false);
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    paramCache_[kParamDisplayGain] = 0.0f;
    paramCache_[kParamHeatmapDecaySec] = float(ssmeter::HeatmapGrid::kDefaultDecaySeconds);

    scope_ = new GoniometerWidget(this);
    scope_->setDecayMs(decayMs_);
    scope_->setResolutionPercent(int(resolutionPercent_));
    scope_->setSmoothLines(smoothLines_);
    scope_->setHeatmapEnabled(heatmapEnabled_);

    stereoMeter_ = new StereoMeterWidget(this);
    levelMeter_ = new LevelMeterWidget(this, "IN", 2);
    levelMeter_->setChannelLabels("L", "R");

    strip_ = new ParamStripWidget(this);
    // kFieldDecayMs/kFieldResolution are the two fields here that aren't
    // real Parameters -- see their own comments in UIPhaseScope.hpp. Both
    // callbacks special-case them before falling through to the ordinary
    // paramCache_/setParameterValue() path every other field (including
    // Gain and Heatmap Decay, both real Parameters) uses.
    strip_->setGetter([this](uint32_t idx) {
        if (idx == kFieldDecayMs) return decayMs_;
        if (idx == kFieldResolution) return resolutionPercent_;
        return paramCache_[idx];
    });
    strip_->setSetter([this](uint32_t idx, float v) {
        if (idx == kFieldDecayMs) { setDecayMs(v); return; }
        if (idx == kFieldResolution) { setResolutionPercent(v); return; }
        paramCache_[idx] = v;
        // Every other field here (Gain, Heatmap Decay) is a real
        // kParameterIsAutomatable Parameter, same as kParamDisplayGain
        // below -- bracketed with editParameter() for the same reason
        // btnAutoGain_'s own callback needs it (see its comment).
        editParameter(idx, true);
        setParameterValue(idx, v);
        editParameter(idx, false);
    });
    strip_->setFields({
        { "Gain", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamDisplayGain,
          "Manual zoom on the goniometer plot -- never affects the audio or any meter reading." },
        { "Decay", "ms", float(ssmeter::GoniometerBuffer::kMinDecaySeconds * 1000.0),
          float(ssmeter::GoniometerBuffer::kMaxDecaySeconds * 1000.0),
          float(ssmeter::GoniometerBuffer::kDefaultDecaySeconds * 1000.0), 0.0f, true, 0, kFieldDecayMs,
          "How much of the goniometer's trail is currently shown, at the same fixed point density regardless." },
        { "Res", "%", float(DGL_NAMESPACE::GoniometerWidget::kMinResolutionPercent), 100.0f,
          100.0f, 5.0f, false, 0, kFieldResolution,
          "What fraction of points are actually drawn each frame -- lower this on a slower computer." },
        { "Heat", "s", float(ssmeter::HeatmapGrid::kMinDecaySeconds),
          float(ssmeter::HeatmapGrid::kMaxDecaySeconds),
          float(ssmeter::HeatmapGrid::kDefaultDecaySeconds), 0.0f, true, 1, kParamHeatmapDecaySec,
          "How many seconds of history the Heatmap glow remembers." },
    });

    btnFreeze_ = new SimpleButton(this, "Freeze");
    btnFreeze_->setTooltip("Pause the display without pausing measurement -- click again to resume.");
    btnFreeze_->setCallback([this] {
        frozen_ = !frozen_;
        refreshFreezeColors();
    });
    refreshFreezeColors();

    // Reset Hold (and the kParamResetHold Parameter/StereoImageMeter::
    // resetMinHold() it drove) is gone entirely now that the min-hold's
    // own plateau-then-decay behavior (see PhaseScopeDsp.hpp's own
    // StereoImageMeter::process() comment) already does what a manual
    // reset was for.

    // Auto Gain is a proper on/off toggle, not a one-shot click -- "Click
    // on it => activate Autogain and the gain is adjusted continuously.
    // Click again on it => Autogain not active, but the last set gain is
    // kept in the gain parameter." While autoGainActive_ is true,
    // parameterChanged()'s own kParamMeterPlotPeakDb case calls
    // applyAutoGain() on every update (i.e. continuously, at whatever
    // rate the DSP side's own peak reading changes), not just here on
    // click -- this callback's only job is flipping the toggle itself and
    // giving one immediate correction so the effect is visible right away
    // rather than waiting for the next peak update.
    btnAutoGain_ = new SimpleButton(this, "Auto Gain");
    btnAutoGain_->setTooltip("Toggle continuous Gain tracking, so the loudest recent signal keeps filling the plot -- click again to stop and keep the last value.");
    btnAutoGain_->setCallback([this] {
        autoGainActive_ = !autoGainActive_;
        setState("autoGainActive", autoGainActive_ ? "1" : "0");
        // One editParameter() touch bracket per whole active session --
        // begun here, ended on the matching off-click -- rather than one
        // per individual continuous correction: this is one long "the UI
        // is driving this parameter" gesture, the same shape a mouse-held
        // fader drag has, not a rapid sequence of separate touches.
        editParameter(kParamDisplayGain, autoGainActive_);
        if (autoGainActive_) applyAutoGain();
        refreshAutoGainLabel();
    });
    refreshAutoGainLabel();

    btnDrawLines_ = new SimpleButton(this, "Draw: Lines");
    btnDrawLines_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnDrawLines_->setTooltip("Toggle the goniometer between a fading dot cloud and a connected-line trace.");
    btnDrawLines_->setCallback([this] {
        drawLines_ = !drawLines_;
        setState("drawLines", drawLines_ ? "1" : "0");
        scope_->setDrawLines(drawLines_);
        refreshDrawLinesLabel();
        refreshSmoothLabel(); // Smooth's own enabled state follows Draw: Lines/Dots
    });
    refreshDrawLinesLabel();

    btnSmooth_ = new SimpleButton(this, "Smooth: Off");
    btnSmooth_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnSmooth_->setTooltip("Draw a smooth curve through the trail's points instead of straight segments (Draw: Lines mode only).");
    btnSmooth_->setCallback([this] {
        smoothLines_ = !smoothLines_;
        setState("smoothLines", smoothLines_ ? "1" : "0");
        scope_->setSmoothLines(smoothLines_);
        refreshSmoothLabel();
    });
    refreshSmoothLabel();

    btnHeatmap_ = new SimpleButton(this, "Heatmap: Off");
    btnHeatmap_->setTooltip("Show a long-history density glow of where the trail has recently been, on top of the live trace.");
    btnHeatmap_->setCallback([this] {
        heatmapEnabled_ = !heatmapEnabled_;
        setState("heatmapEnabled", heatmapEnabled_ ? "1" : "0");
        scope_->setHeatmapEnabled(heatmapEnabled_);
        refreshHeatmapLabel();
    });
    refreshHeatmapLabel();

    btnCompact_ = new SimpleButton(this, "Compact");
    btnCompact_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnCompact_->setCallback([this] { autoCompact_ = false; setCompact(!compact_, true); });
    refreshCompactLabel();

    // Added last so it draws on top of every other widget here -- see its
    // own header comment for why it never claims a mouse/motion event
    // despite covering the whole window.
    tooltip_ = new TooltipOverlay(this);
    // After the tooltip, so it sees every event first -- see MotionWatcher.hpp.
    motionWatcher_ = new MotionWatcher(this);

    layout();
}

void UIPhaseScope::refreshFreezeColors()
{
    btnFreeze_->setColors(frozen_ ? Color(150, 90, 40) : Color(58, 62, 70),
                           frozen_ ? Color(175, 110, 55) : Color(78, 84, 96),
                           Color(230, 232, 236));
}

void UIPhaseScope::refreshDrawLinesLabel()
{
    btnDrawLines_->setLabel(drawLines_ ? "Draw: Lines" : "Draw: Dots");
    btnDrawLines_->repaint();
}

void UIPhaseScope::refreshSmoothLabel()
{
    btnSmooth_->setLabel(smoothLines_ ? "Smooth: On" : "Smooth: Off");
    // Only meaningful in Draw: Lines mode -- see GoniometerWidget::
    // setSmoothLines()'s own comment (the dot cloud has no "curve"
    // between points to smooth). Grayed out rather than hidden while
    // Draw: Dots is selected, so the setting itself stays visible and its
    // last choice isn't lost, the same "gray out what doesn't currently
    // apply" convention ../compressor's own True Peak mode uses for
    // Attack/Knee.
    btnSmooth_->setEnabled(drawLines_);
    btnSmooth_->repaint();
}

void UIPhaseScope::refreshHeatmapLabel()
{
    btnHeatmap_->setLabel(heatmapEnabled_ ? "Heatmap: On" : "Heatmap: Off");
    btnHeatmap_->setColors(heatmapEnabled_ ? Color(50, 100, 130) : Color(58, 62, 70),
                            heatmapEnabled_ ? Color(65, 120, 150) : Color(78, 84, 96),
                            Color(230, 232, 236));
    btnHeatmap_->repaint();
}

void UIPhaseScope::refreshAutoGainLabel()
{
    btnAutoGain_->setLabel(autoGainActive_ ? "Auto Gain: On" : "Auto Gain: Off");
    btnAutoGain_->setColors(autoGainActive_ ? Color(50, 100, 130) : Color(58, 62, 70),
                             autoGainActive_ ? Color(65, 120, 150) : Color(78, 84, 96),
                             Color(230, 232, 236));
    btnAutoGain_->repaint();
}

void UIPhaseScope::applyAutoGain()
{
    const float targetDb = float(ssmeter::linearToDb(ssmeter::kAutoGainTargetPeak));
    const float gainDb = std::max(-24.0f, std::min(24.0f, targetDb - meterPlotPeakDb_));
    paramCache_[kParamDisplayGain] = gainDb;
    setParameterValue(kParamDisplayGain, gainDb);
    strip_->repaint();
}

void UIPhaseScope::setDecayMs(float ms)
{
    decayMs_ = ms;
    scope_->setDecayMs(decayMs_);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.0f", double(decayMs_));
    setState("decayMs", buf);
    strip_->repaint();
}

void UIPhaseScope::setResolutionPercent(float pct)
{
    resolutionPercent_ = pct;
    scope_->setResolutionPercent(int(resolutionPercent_));
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%.0f", double(resolutionPercent_));
    setState("resolutionPercent", buf);
    strip_->repaint();
}

void UIPhaseScope::layout()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Both cover the entire window -- see their own header comments.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));
    motionWatcher_->setAbsolutePos(0, 0);
    motionWatcher_->setSize(uint(w), uint(h));

    if (compact_)
    {
        // Scope fills the window above a slim correlation bar; Freeze and
        // Full view float over the scope's top-left corner (drawn after it,
        // and given events before it -- both follow construction order).
        constexpr float corrH = 58.0f;
        scope_->setAbsolutePos(0, 0);
        scope_->setSize(uint(w), uint(std::max(10.0f, h - corrH)));
        stereoMeter_->setAbsolutePos(0, int(h - corrH));
        stereoMeter_->setSize(uint(w), uint(corrH));
        constexpr float bh = 22.0f, cpad = 4.0f;
        btnCompact_->setAbsolutePos(int(cpad), int(cpad));
        btnCompact_->setSize(76, uint(bh));
        btnFreeze_->setAbsolutePos(int(cpad + 76.0f + cpad), int(cpad));
        btnFreeze_->setSize(60, uint(bh));
        return;
    }

    constexpr float pad = 6.0f;
    constexpr float topBarH = 28.0f;
    constexpr float meterW = 100.0f;
    constexpr float stereoMeterH = 110.0f;
    constexpr float stripH = 46.0f;
    constexpr float gap = 6.0f;

    float bx = pad;
    auto place = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx), int(pad));
        b->setSize(uint(bw), uint(topBarH));
        bx += bw + gap;
    };
    place(btnFreeze_, 70.0f);
    place(btnAutoGain_, 100.0f);
    place(btnDrawLines_, 90.0f);
    place(btnSmooth_, 90.0f);
    place(btnHeatmap_, 100.0f);
    place(btnCompact_, 80.0f);

    const float mainY = topBarH + pad * 2.0f;
    const float mainH = h - mainY - stereoMeterH - stripH - gap * 2.0f - pad;

    levelMeter_->setAbsolutePos(int(pad), int(mainY));
    levelMeter_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    const float scopeX = pad + meterW + pad;
    const float scopeW = w - scopeX - pad;
    scope_->setAbsolutePos(int(scopeX), int(mainY));
    scope_->setSize(uint(std::max(10.0f, scopeW)), uint(std::max(10.0f, mainH)));

    const float stripY = mainY + mainH + gap;
    strip_->setAbsolutePos(int(pad), int(stripY));
    strip_->setSize(uint(w - 2.0f * pad), uint(stripH));

    stereoMeter_->setAbsolutePos(int(pad), int(h - stereoMeterH - pad));
    stereoMeter_->setSize(uint(w - 2.0f * pad), uint(stereoMeterH));
}

void UIPhaseScope::refreshCompactLabel()
{
    btnCompact_->setLabel(compact_ ? "Full view" : "Compact");
    btnCompact_->setTooltip(compact_
        ? "Back to the full view with all controls and meters."
        : "Compact view: only the scope and the correlation bar, in a small window that can stay open next to other windows. The buttons appear while the mouse moves over it.");
    btnCompact_->repaint();
}

void UIPhaseScope::applyVisibility()
{
    const bool full = !compact_;
    levelMeter_->setVisible(full);
    strip_->setVisible(full);
    btnAutoGain_->setVisible(full);
    btnDrawLines_->setVisible(full);
    btnSmooth_->setVisible(full);
    btnHeatmap_->setVisible(full);
    btnFreeze_->setVisible(full || controlsShown_);
    btnCompact_->setVisible(full || controlsShown_);
    stereoMeter_->setCorrelationOnly(compact_);
}

void UIPhaseScope::setCompact(bool compact, bool fromUser, bool resizeWindow)
{
    if (strip_->isEditing()) strip_->cancelEdit();
    compact_ = compact;
    if (fromUser) setState("compact", compact_ ? "1" : "0");

    // The editor's design (logical) size changes; DPF's zoom factor is
    // window size / design size, so resizing the window to the new design
    // size times the current factor keeps the zoom (e.g. 2x stays 2x).
    const double zoom = getWidth() > 0 ? double(getWindow().getWidth()) / double(getWidth()) : 1.0;
    const uint lw = compact_ ? kCompactWidth : uint(DISTRHO_UI_DEFAULT_WIDTH);
    const uint lh = compact_ ? kCompactHeight : uint(DISTRHO_UI_DEFAULT_HEIGHT);
    // Zoom reference = the view's design size; minimum = the compact view's
    // size for full, 60 % of it for compact. No fixed aspect ratio: the
    // layout fills whatever shape the window has.
    getWindow().setAutoScaleBaseSize(lw, lh);
    if (compact_)
        setGeometryConstraints(kCompactWidth * 3 / 5, kCompactHeight * 3 / 5, false, true, false);
    else
        setGeometryConstraints(kCompactWidth, kCompactHeight, false, true, false);
    if (resizeWindow)
        getWindow().setSize(uint(std::lround(lw * zoom)), uint(std::lround(lh * zoom)));
    else
    {
        // Keep the size, but make DPF recompute its zoom and logical size
        // for the new design size: it only does so on a configure event,
        // and a same-size resize produces none -- so one pixel taller now
        // (not smaller: DPF would clamp that at the minimum size), and back
        // once that has been processed (updateAutoCompact()): if both were
        // still queued together, pugl would merge them into one "no
        // change" and skip it.
        const uint pw = getWindow().getWidth(), ph = getWindow().getHeight();
        pendingRestoreHeight_ = ph;
        pendingRestoreTicks_ = 0;
        nudgeLogicalWidth_ = getWidth();
        getWindow().setSize(pw, ph + 1);
    }

    // Just switched by a click: show the controls, so "Full view" is right
    // there to switch back; they hide after kAutohideSeconds of no movement.
    motionWatcher_->touch();
    controlsShown_ = true;
    refreshCompactLabel();
    applyVisibility();
    layout();
    repaint();
}

void UIPhaseScope::updateAutoCompact()
{
    if (pendingRestoreHeight_ != 0)
    {
        // Restore once DPF has reconfigured (the logical size changed with
        // the new design size), or after ~10 ticks at the latest.
        if (getWidth() == nudgeLogicalWidth_ && ++pendingRestoreTicks_ < 10) return;
        getWindow().setSize(getWindow().getWidth(), pendingRestoreHeight_);
        pendingRestoreHeight_ = 0;
        return;
    }

    // Physical window size against the full view's size at the host's or
    // desktop's scale -- the zoom the user has dragged the window to.
    const double scale = getScaleFactor();
    const double fullW = double(DISTRHO_UI_DEFAULT_WIDTH) * scale;
    const double fullH = double(DISTRHO_UI_DEFAULT_HEIGHT) * scale;
    const double pw = double(getWindow().getWidth()), ph = double(getWindow().getHeight());
    if (pw <= 1.0 || ph <= 1.0) return; // not set up yet
    if (!compact_)
    {
        if (pw < kAutoCompactBelow * fullW || ph < kAutoCompactBelow * fullH)
        {
            autoCompact_ = true;
            setCompact(true, true, false);
        }
    }
    else if (autoCompact_ && pw >= kAutoFullAbove * fullW && ph >= kAutoFullAbove * fullH)
    {
        autoCompact_ = false;
        setCompact(false, true, false);
    }
}

void UIPhaseScope::updateAutohide()
{
    if (!compact_) return;
    // Stay visible while the pointer rests on one of them (a hovered
    // button is about to be clicked, even if the mouse stopped moving) --
    // but not forever: DGL has no pointer-leave event, so a button the
    // pointer left the window from still reads as hovered.
    const double idle = motionWatcher_->secondsSinceActivity();
    const bool hovered = btnCompact_->isHovered() || btnFreeze_->isHovered();
    const bool show = idle < kAutohideSeconds || (hovered && idle < 15.0);
    if (show == controlsShown_) return;
    controlsShown_ = show;
    applyVisibility();
    repaint();
}

void UIPhaseScope::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIPhaseScope::onNanoDisplay()
{
    fillColor(Color(20, 21, 25));
    beginPath();
    rect(0, 0, float(getWidth()), float(getHeight()));
    fill();
}

void UIPhaseScope::parameterChanged(uint32_t index, float value)
{
    // Settings mirrors -- kept current regardless of Freeze, since Freeze
    // only pauses the *meter/scope* redraw (see this file's own header
    // comment), not the state of actual editable controls. A frozen
    // display shouldn't also make an automated Gain change silently fail
    // to show up.
    switch (index)
    {
    case kParamDisplayGain:
        paramCache_[index] = value;
        strip_->repaint();
        return;
    case kParamHeatmapDecaySec:
        paramCache_[index] = value;
        strip_->repaint();
        return;
    case kParamMeterPlotPeakDb:
        meterPlotPeakDb_ = value;
        // While Auto Gain is toggled on, every fresh peak reading gets an
        // immediate correction -- this is what makes it "continuous"
        // rather than a one-shot snapshot from whatever the peak happened
        // to be at the moment of the click.
        if (autoGainActive_) applyAutoGain();
        return;
    default:
        break;
    }

    if (frozen_) return; // pure UI-local pause -- see this file's own header comment

    if (index >= kParamScopePacketBase && index < kParamScopePacketBase + uint32_t(kBufferSize))
    {
        const uint32_t slot = index - kParamScopePacketBase;
        ssmeter::GoniometerBuffer::unpackPoint(value, gonioX_[slot], gonioY_[slot]);
        scopeDirty_ = true;
        return;
    }

    if (index >= kParamHeatmapPacketBase && index < kParamHeatmapPacketBase + uint32_t(ssmeter::HeatmapGrid::kNumPackets))
    {
        const uint32_t slot = index - kParamHeatmapPacketBase;
        float quad[ssmeter::HeatmapGrid::kCellsPerPacket];
        ssmeter::HeatmapGrid::unpackQuad(value, quad);
        for (int k = 0; k < ssmeter::HeatmapGrid::kCellsPerPacket; ++k)
            heatmapCells_[slot * uint32_t(ssmeter::HeatmapGrid::kCellsPerPacket) + uint32_t(k)] = quad[k];
        heatmapDirty_ = true;
        return;
    }

    switch (index)
    {
    case kParamScopeWriteIndex:
        writeSlot_ = int(value);
        scopeDirty_ = true;
        break;
    case kParamMeterInputRmsL:
        meterInputRmsL_ = value;
        levelMeter_->setChannel(0, meterInputRmsL_, meterInputPeakL_);
        break;
    case kParamMeterInputPeakL:
        meterInputPeakL_ = value;
        levelMeter_->setChannel(0, meterInputRmsL_, meterInputPeakL_);
        break;
    case kParamMeterInputRmsR:
        meterInputRmsR_ = value;
        levelMeter_->setChannel(1, meterInputRmsR_, meterInputPeakR_);
        break;
    case kParamMeterInputPeakR:
        meterInputPeakR_ = value;
        levelMeter_->setChannel(1, meterInputRmsR_, meterInputPeakR_);
        break;
    case kParamMeterCorrelation:
        correlation_ = value;
        stereoMeter_->setCorrelation(correlation_, correlationMinHold_);
        scope_->setCorrelation(correlation_);
        break;
    case kParamMeterCorrelationMinHold:
        correlationMinHold_ = value;
        stereoMeter_->setCorrelation(correlation_, correlationMinHold_);
        break;
    case kParamMeterWidthPercent:
        stereoMeter_->setWidthPercent(value);
        break;
    default:
        break;
    }
}

void UIPhaseScope::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "drawLines") == 0)
    {
        drawLines_ = std::strcmp(value, "1") == 0;
        scope_->setDrawLines(drawLines_);
        refreshDrawLinesLabel();
        refreshSmoothLabel(); // Smooth's own enabled state follows Draw: Lines/Dots
        return;
    }
    if (std::strcmp(key, "smoothLines") == 0)
    {
        smoothLines_ = std::strcmp(value, "1") == 0;
        scope_->setSmoothLines(smoothLines_);
        refreshSmoothLabel();
        return;
    }
    if (std::strcmp(key, "heatmapEnabled") == 0)
    {
        heatmapEnabled_ = std::strcmp(value, "1") == 0;
        scope_->setHeatmapEnabled(heatmapEnabled_);
        refreshHeatmapLabel();
        return;
    }
    if (std::strcmp(key, "autoGainActive") == 0)
    {
        // A host restoring a previously-saved "on" state on project load
        // does *not* also re-issue the editParameter() touch bracket the
        // button's own click handler would have -- there's no user click
        // to bracket here, just a plain restored on/off flag, so this
        // path only mirrors autoGainActive_ and its label, exactly like
        // every other restored toggle State above.
        autoGainActive_ = std::strcmp(value, "1") == 0;
        refreshAutoGainLabel();
        return;
    }
    if (std::strcmp(key, "decayMs") == 0)
    {
        // Update the local mirror and the widget directly rather than
        // routing back through setDecayMs() -- that would call setState()
        // again for a value the host itself just sent, a needless
        // round trip (harmless, but pointless).
        decayMs_ = float(std::atof(value));
        scope_->setDecayMs(decayMs_);
        strip_->repaint();
        return;
    }
    if (std::strcmp(key, "compact") == 0)
    {
        const bool c = std::strcmp(value, "1") == 0;
        if (c != compact_) setCompact(c, false);
        return;
    }
    if (std::strcmp(key, "resolutionPercent") == 0)
    {
        resolutionPercent_ = float(std::atof(value));
        scope_->setResolutionPercent(int(resolutionPercent_));
        strip_->repaint();
    }
}

void UIPhaseScope::uiIdle()
{
    if (!frozen_ && scopeDirty_)
    {
        scope_->setScope(gonioX_, gonioY_, writeSlot_);
        scopeDirty_ = false;
    }
    if (!frozen_ && heatmapDirty_)
    {
        scope_->setHeatmap(heatmapCells_);
        heatmapDirty_ = false;
    }
    updateAutoCompact();
    updateAutohide();
    updateTooltip();
}

void UIPhaseScope::updateTooltip()
{
    // Every button in turn -- there are few enough of these (six) that a
    // plain linear scan every tick is negligible, no need for a hover-
    // changed callback/event just for this.
    SimpleButton* buttons[] = { btnFreeze_, btnAutoGain_, btnDrawLines_, btnSmooth_, btnHeatmap_, btnCompact_ };
    for (SimpleButton* b : buttons)
    {
        if (b->isVisible() && b->isHovered() && !b->tooltip().empty())
        {
            const auto area = b->getAbsoluteArea();
            tooltip_->show(b->tooltip(), float(area.getX()), float(area.getY()),
                           float(area.getWidth()), float(area.getHeight()));
            return;
        }
    }

    const int fieldIdx = strip_->isVisible() ? strip_->hoveredTooltipField() : -1;
    if (fieldIdx >= 0)
    {
        const std::string& text = strip_->tooltipFor(fieldIdx);
        float fx, fy, fw, fh;
        if (!text.empty() && strip_->fieldRect(fieldIdx, fx, fy, fw, fh))
        {
            // fieldRect() is in strip_'s own local coordinates -- offset
            // by its absolute position to place the tooltip correctly in
            // this top-level widget's own coordinate space.
            const int sx = strip_->getAbsoluteX();
            const int sy = strip_->getAbsoluteY();
            tooltip_->show(text, float(sx) + fx, float(sy) + fy, fw, fh);
            return;
        }
    }

    tooltip_->hide();
}

bool UIPhaseScope::onCharacterInput(const CharacterInputEvent& ev)
{
    if (!strip_->isEditing()) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { strip_->commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { strip_->appendChar(c); return true; }
    return true;
}

bool UIPhaseScope::onKeyboard(const KeyboardEvent& ev)
{
    if (!strip_->isEditing() || !ev.press) return false;
    switch (ev.key)
    {
    case DGL_NAMESPACE::kKeyEnter:
    case DGL_NAMESPACE::kKeyTab:
        strip_->commitEdit();
        return true;
    case DGL_NAMESPACE::kKeyEscape:
        strip_->cancelEdit();
        return true;
    case DGL_NAMESPACE::kKeyBackspace:
    case DGL_NAMESPACE::kKeyDelete:
        strip_->backspace();
        return true;
    default:
        return true;
    }
}

void UIPhaseScope::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIPhaseScope(); }

END_NAMESPACE_DISTRHO
