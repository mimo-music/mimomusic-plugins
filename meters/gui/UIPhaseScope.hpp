// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/PhaseScopeDsp.hpp"
#include "../plugin_phasescope/PluginPhaseScope.hpp"
#include "GoniometerWidget.hpp"
#include "LevelMeterWidget.hpp"
#include "MotionWatcher.hpp"
#include "ParamStripWidget.hpp"
#include "SimpleButton.hpp"
#include "StereoMeterWidget.hpp"
#include "TooltipOverlay.hpp"

START_NAMESPACE_DISTRHO

// A small, dedicated UI for a small, dedicated plugin -- see
// PluginPhaseScope.hpp's own top comment: every Parameter is either an
// output meter reading or a display-only control (Gain/Heatmap Decay,
// via ParamStripWidget), so there's still no preset
// Save/Load or A/B/Undo-Redo -- none of that applies to a plugin with
// nothing that shapes audio to capture in the first place. "Freeze" is
// deliberately pure UI-local state (like LevelMeterWidget's own "click to
// clear the clip LED" convention): pausing *this window's own redraw*
// needs no round trip through a Parameter/State at all, so it isn't one.
// "Draw Lines", "Decay", "Resolution", "Smooth" and "Heatmap" *are*
// States despite being just as purely rendering preferences, because --
// unlike Freeze -- they're lasting choices worth the host remembering
// across a session. Decay and Resolution's own strip fields are still
// driven through ParamStripWidget's ordinary getter/setter callbacks (see
// kFieldDecayMs/kFieldResolution below), just routed to setState()
// instead of setParameterValue() underneath; Gain and Heatmap Decay are
// real Parameters, so their fields need no such sentinel.
class UIPhaseScope : public UI
{
public:
    UIPhaseScope();

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onDisplay() override; // DPF's frame, with the real pixel ratio -- see EditorSubWidget.hpp
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;

private:
    void layout();
    void refreshFreezeColors();
    void refreshDrawLinesLabel();
    void refreshSmoothLabel();
    void refreshHeatmapLabel();
    void refreshAutoGainLabel();
    void applyAutoGain(); // computes a fresh Gain value from meterPlotPeakDb_ and applies it -- see btnAutoGain_'s own callback comment
    void setDecayMs(float ms);            // updates decayMs_, scope_, and round-trips through setState("decayMs", ...)
    void setResolutionPercent(float pct); // updates resolutionPercent_, scope_, and round-trips through setState("resolutionPercent", ...)
    void updateTooltip(); // polls every button's/field's own hover state once per uiIdle() tick and shows/hides tooltip_ accordingly

    // Compact view (kStateCompact): the editor shrinks to the goniometer
    // plus a correlation bar; Freeze and "Full view" appear only while the
    // mouse is active over it (autohide, see updateAutohide()).
    // resizeWindow: the Compact/Full view button resizes the window to the
    // new design size; the automatic switch (updateAutoCompact()) does not,
    // since there the user is already resizing it.
    void setCompact(bool compact, bool fromUser, bool resizeWindow = true);
    // Per uiIdle() tick: switch to compact when the window is made smaller
    // than kAutoCompactBelow of its normal size, and back to full above
    // kAutoFullAbove -- only if compact was entered this way, never undoing
    // a click on Compact. The gap between the two prevents flip-flopping.
    void updateAutoCompact();
    static constexpr double kAutoCompactBelow = 0.6;
    static constexpr double kAutoFullAbove = 0.85;
    void applyVisibility();  // which widgets show, from compact_ and controlsShown_
    void updateAutohide();   // per uiIdle() tick: show/hide the compact view's controls
    void refreshCompactLabel();
    static constexpr uint kCompactWidth = 260;
    static constexpr uint kCompactHeight = 300;
    static constexpr double kAutohideSeconds = 2.0;

    static constexpr int kBufferSize = ssmeter::GoniometerBuffer::kScopeBufferSize;
    static constexpr int kHeatmapCellCount = ssmeter::HeatmapGrid::kCellCount;

    // A sentinel ParamStripWidget field index that is *not* a real DPF
    // Parameter -- Decay moved from a Parameter to a State (see
    // PluginPhaseScope.hpp's own kStateDecayMs comment for why), but
    // ParamStripWidget's field list is keyed by a plain uint32_t index
    // regardless of what that index actually means; the strip's own
    // getter/setter callbacks below special-case this one value to read/
    // write decayMs_ and call setState() instead of forwarding to
    // paramCache_/setParameterValue() like every real Parameter field
    // does. kParamCount (one past the last real Parameter index) is a
    // convenient, guaranteed-never-real value for this.
    static constexpr uint32_t kFieldDecayMs = kParamCount;
    // Same sentinel idea as kFieldDecayMs, one slot further along so the
    // two never collide with each other or with a real Parameter index.
    static constexpr uint32_t kFieldResolution = kParamCount + 1;

    DGL_NAMESPACE::GoniometerWidget* scope_ = nullptr;
    DGL_NAMESPACE::StereoMeterWidget* stereoMeter_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* levelMeter_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnFreeze_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAutoGain_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnDrawLines_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSmooth_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnHeatmap_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnCompact_ = nullptr;
    DGL_NAMESPACE::TooltipOverlay* tooltip_ = nullptr;
    DGL_NAMESPACE::MotionWatcher* motionWatcher_ = nullptr; // constructed last, see MotionWatcher.hpp

    bool compact_ = false;
    bool controlsShown_ = true; // compact view only: Freeze/Full view currently visible
    bool autoCompact_ = false;  // compact view entered by updateAutoCompact(), not by the button
    uint pendingRestoreHeight_ = 0; // see setCompact(): second half of the 1 px nudge
    int pendingRestoreTicks_ = 0;
    uint nudgeLogicalWidth_ = 0;

    bool frozen_ = false;
    bool drawLines_ = true; // Lines (and Smooth) on by default
    bool smoothLines_ = true; // on by default
    bool heatmapEnabled_ = false;
    // On/off toggle, not a one-shot action -- see btnAutoGain_'s own
    // callback comment for the full "click to activate/deactivate" flow.
    bool autoGainActive_ = false;
    float decayMs_ = float(ssmeter::GoniometerBuffer::kDefaultDecaySeconds * 1000.0);
    float resolutionPercent_ = 100.0f;

    // Goniometer data, accumulated point-by-point as parameterChanged()
    // pushes arrive and handed to the widget in one batch per uiIdle()
    // tick (scopeDirty_) rather than triggering hundreds of separate
    // repaints.
    float gonioX_[kBufferSize] = {};
    float gonioY_[kBufferSize] = {};
    int writeSlot_ = 0;
    bool scopeDirty_ = false;

    // Same batching idea for the Heatmap's own packed cells.
    float heatmapCells_[kHeatmapCellCount] = {};
    bool heatmapDirty_ = false;

    float meterInputRmsL_ = -120.0f, meterInputPeakL_ = -120.0f;
    float meterInputRmsR_ = -120.0f, meterInputPeakR_ = -120.0f;
    float correlation_ = 0.0f;
    float correlationMinHold_ = 1.0f;
    // The raw (unscaled) rotated Mid/Side peak, continuously updated --
    // see PluginPhaseScope.hpp's own kParamMeterPlotPeakDb comment. Read
    // by applyAutoGain() (via parameterChanged()'s own case for this,
    // and directly on click) to compute a fresh Gain value, with no
    // trigger/state round trip to the DSP side needed.
    float meterPlotPeakDb_ = -120.0f;

    // strip_'s own backing store for real Parameters -- DPF's UI base
    // class only pushes values via parameterChanged(), it doesn't expose
    // a getter, so this is what ParamStripWidget's getter callback reads
    // for every field except kFieldDecayMs/kFieldResolution.
    float paramCache_[kParamCount] = {};

    DISTRHO_DECLARE_NON_COPYABLE(UIPhaseScope)
};

END_NAMESPACE_DISTRHO
