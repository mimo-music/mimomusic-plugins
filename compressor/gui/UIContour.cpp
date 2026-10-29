// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIContour.hpp"

#include "PresetIO.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::ContourGraphWidget;
using DGL_NAMESPACE::LevelMeterWidget;
using DGL_NAMESPACE::ParamFieldSpec;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;
using sscomp::CurvePoint;
using sscomp::StereoMode;
using sscomp::TransferCurve;
using sscomp::TriggerCondition;
using sscomp::TriggerAction;

namespace {
const char* stereoModeName(StereoMode m)
{
    switch (m)
    {
    case StereoMode::Linked:   return "Stereo: Linked";
    case StereoMode::DualMono: return "Stereo: Dual Mono";
    case StereoMode::MidSide:  return "Stereo: Mid/Side";
    }
    return "Stereo";
}

// Every automatable knob this preset format round-trips -- "stereoMode"
// and "linkAB" (States, not Parameters) and "triggerCondition"/"interp"/
// "curve" (both banks -- the curve/trigger-condition Parameter mirror)
// are handled separately, same split UICompressorStereo's own preset code
// already uses for its own States. The B-bank entries round-trip
// unconditionally, same as A's own -- whether they currently do anything
// live depends on stereo mode/Link, not on whether they're worth
// remembering. kParamListenMode deliberately excluded -- "momentary setup
// aid, not mix-defining data" precedent, same as the sibling Compressor
// Stereo's own kPresetParams.
constexpr uint32_t kPresetParams[] = {
    kParamInputGain, kParamThreshold, kParamAttack, kParamRelease,
    kParamOutputGain, kParamBypass,
    kParamSidechainEnable, kParamSidechainGain,
    kParamThresholdB, kParamAttackB, kParamReleaseB, kParamMakeupGainB,
    kParamGlobalOutputGain, kParamLookahead,
    kParamScFilterEnable, kParamScFilterHighpassHz, kParamScFilterLowpassHz,
    kParamScFilterLinkLR, kParamScFilterHighpassHzR, kParamScFilterLowpassHzR,
    kParamInputGainB, kParamWetDry, kParamWetDryB, kParamGlobalInputGain
};
constexpr const char* kPresetKeys[] = {
    "inputGainDb", "thresholdDb", "attackMs", "releaseMs",
    "outputGainDb", "bypass",
    "sidechainEnable", "sidechainGainDb",
    "thresholdDbB", "attackMsB", "releaseMsB", "makeupGainDbB",
    "globalOutputGainDb", "lookaheadMs",
    "scFilterEnable", "scFilterHighpassHz", "scFilterLowpassHz",
    "scFilterLinkLR", "scFilterHighpassHzR", "scFilterLowpassHzR",
    "inputGainDbB", "wetDry", "wetDryB", "globalInputGainDb"
};
constexpr size_t kPresetParamCount = sizeof(kPresetParams) / sizeof(kPresetParams[0]);
static_assert(sizeof(kPresetKeys) / sizeof(kPresetKeys[0]) == kPresetParamCount, "keys/params length mismatch");
}

UIContour::UIContour()
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
    getWindow().setAutoScaleBaseSize(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT);
    // Enlarged by dragging: zoom in half steps (2, 2.5, 3, ...) -- at
    // fractional zooms like 1.37 lines and text fall between physical
    // pixels and look soft; the remaining space goes to the layout.
    getWindow().setAutoScaleStep(0.5);
    setGeometryConstraints(DISTRHO_UI_DEFAULT_WIDTH / 2, DISTRHO_UI_DEFAULT_HEIGHT / 2, true, true, false);
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    paramCache_[kParamInputGain] = 0.0f;
    paramCache_[kParamThreshold] = -18.0f;
    paramCache_[kParamAttack] = 10.0f;
    paramCache_[kParamRelease] = 150.0f;
    paramCache_[kParamOutputGain] = 0.0f;
    // Unused -- see kParamCompensation's own comment (PluginContour.cpp).
    paramCache_[kParamCompensation] = 0.0f;
    paramCache_[kParamCompensationMeasure] = 0.0f;
    paramCache_[kParamBypass] = 0.0f;
    paramCache_[kParamSidechainEnable] = 0.0f;
    paramCache_[kParamSidechainGain] = 0.0f;
    paramCache_[kParamMeterInputRmsL] = paramCache_[kParamMeterInputPeakL] = -120.0f;
    paramCache_[kParamMeterInputRmsR] = paramCache_[kParamMeterInputPeakR] = -120.0f;
    paramCache_[kParamMeterOutputRmsL] = paramCache_[kParamMeterOutputPeakL] = -120.0f;
    paramCache_[kParamMeterOutputRmsR] = paramCache_[kParamMeterOutputPeakR] = -120.0f;
    paramCache_[kParamMeterDetectorA] = paramCache_[kParamMeterDetectorB] = -120.0f;
    paramCache_[kParamMeterGainReductionA] = paramCache_[kParamMeterGainReductionB] = 0.0f;
    paramCache_[kParamMeterCompGainDb] = 0.0f; // unused, see kParamCompensation's own comment
    paramCache_[kParamMeterCompState] = 0.0f;
    // Everything from here on is new this session -- ported from
    // ../plugin_stereo/PluginCompressorStereo.hpp's own "append, never
    // insert" tail. Zeroed in bulk first (mirror banks, spectrum packets),
    // then the real scalar defaults set individually afterward, same
    // "the loop would otherwise clobber them back to 0" ordering
    // UICompressorStereo's own constructor already established.
    for (uint32_t i = kParamCurvePointCount; i < kParamCount; ++i) paramCache_[i] = 0.0f;
    paramCache_[kParamCurvePointCount] = paramCache_[kParamCurvePointCountB] = 2.0f;
    paramCache_[kParamCurvePointInBase] = paramCache_[kParamCurvePointInBaseB] = float(TransferCurve::kMinDb);
    paramCache_[kParamCurvePointInBase + 1] = paramCache_[kParamCurvePointInBaseB + 1] = float(TransferCurve::kMaxDb);
    paramCache_[kParamCurvePointOutBase] = paramCache_[kParamCurvePointOutBaseB] = float(TransferCurve::kMinDb);
    paramCache_[kParamCurvePointOutBase + 1] = paramCache_[kParamCurvePointOutBaseB + 1] = float(TransferCurve::kMaxDb);
    paramCache_[kParamThresholdB] = -18.0f;
    paramCache_[kParamAttackB] = 10.0f;
    paramCache_[kParamReleaseB] = 150.0f;
    paramCache_[kParamMakeupGainB] = 0.0f;
    paramCache_[kParamGlobalOutputGain] = 0.0f;
    paramCache_[kParamScFilterEnable] = 0.0f;
    paramCache_[kParamScFilterHighpassHz] = 100.0f;
    paramCache_[kParamScFilterLowpassHz] = 8000.0f;
    paramCache_[kParamScFilterLinkLR] = 1.0f;
    paramCache_[kParamScFilterHighpassHzR] = 100.0f;
    paramCache_[kParamScFilterLowpassHzR] = 8000.0f;
    paramCache_[kParamMeterScLevelDbL] = paramCache_[kParamMeterScLevelDbR] = -120.0f;
    paramCache_[kParamInputGainB] = 0.0f;
    paramCache_[kParamWetDry] = paramCache_[kParamWetDryB] = 100.0f;
    paramCache_[kParamGlobalInputGain] = 0.0f;
    paramCache_[kParamMeterTrailA] = paramCache_[kParamMeterTrailB] = sscomp::packTrailPair(-120.0, 0.0);
    paramCache_[kParamTrailSource] = 0.0f;
    paramCache_[kParamLookahead] = 0.0f;
    paramCache_[kParamUiHeartbeat] = 0.0f;
    for (uint32_t k = 0; k < kTrailBankSize; ++k) paramCache_[kParamMeterTrailBankA + k] = paramCache_[kParamMeterTrailBankB + k] = paramCache_[kParamMeterTrailA];

    meterIn_ = new LevelMeterWidget(this, "IN", kContourMono ? 1 : 2);
    meterOut_ = new LevelMeterWidget(this, "OUT", kContourMono ? 1 : 2);

    // Linked-mode-only widget set -- Linked mode keeps its original look.
    // See
    // refreshDualLayoutVisibility()'s own comment.
    graph_ = new ContourGraphWidget(this);
    graph_->setCurve(curve_);
    graph_->setThreshold(paramCache_[kParamThreshold]);
    graph_->setTriggerCondition(condition_);
    graph_->setOnGrMeterSpeedChanged([this](DGL_NAMESPACE::GrMeterSpeed s) { setGrMeterSpeed(s, true); });
    graph_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    graph_->setOnCurveChanged([this](const TransferCurve& c) { onGraphCurveChanged(false, c); });

    // Dual-Mono/Mid-Side widget set -- both compressors side by side.
    // graphA_/graphB_ constructed here, before the
    // first updateMeterLabelsForMode() call below, same construction-order
    // reasoning as UICompressorStereo's own.
    graphA_ = new ContourGraphWidget(this);
    graphA_->setCurve(curve_);
    graphA_->setThreshold(paramCache_[kParamThreshold]);
    graphA_->setTriggerCondition(condition_);
    graphA_->setOnGrMeterSpeedChanged([this](DGL_NAMESPACE::GrMeterSpeed s) { setGrMeterSpeed(s, true); });
    graphA_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    graphA_->setOnCurveChanged([this](const TransferCurve& c) { onGraphCurveChanged(false, c); });

    graphB_ = new ContourGraphWidget(this);
    graphB_->setCurve(curve_);
    graphB_->setThreshold(paramCache_[kParamThreshold]);
    graphB_->setTriggerCondition(condition_);
    // bParam() falls back to A's own Parameter whenever B isn't
    // independent -- dragging Threshold on graphB_ while linked therefore
    // edits the exact same Parameter graphA_ does. onGraphCurveChanged(true, ...)
    // applies the identical fallback to the whole curve object -- see
    // that method's own declaration comment (UIContour.hpp) for why a
    // curve needs its own mechanism instead of bParam()'s Parameter-index
    // trick.
    graphB_->setOnGrMeterSpeedChanged([this](DGL_NAMESPACE::GrMeterSpeed s) { setGrMeterSpeed(s, true); });
    graphB_->setOnThresholdChanged([this](float v) { onGraphParamChanged(bParam(true, kParamThreshold, kParamThresholdB), v); });
    graphB_->setOnCurveChanged([this](const TransferCurve& c) { onGraphCurveChanged(true, c); });
    // One trail length for every plot (scrolling the legend on any of them sets all three), so Dual-Mono / Mid-Side look alike per channel.
    {
        auto shareTrailSeconds = [this](float sec) { graph_->setTrailSeconds(sec); graphA_->setTrailSeconds(sec); graphB_->setTrailSeconds(sec); };
        graph_->setOnTrailSecondsChanged(shareTrailSeconds);
        graphA_->setOnTrailSecondsChanged(shareTrailSeconds);
        graphB_->setOnTrailSecondsChanged(shareTrailSeconds);
    }

    // Numeric node list(s), like the EQ's band list -- one PER bank
    // instead of one list toggled between banks: nodeList_ always targets A/curve_ exactly
    // like graph_/graphA_ do, nodeListB_ always targets B exactly like
    // graphB_ does (falling back to curve_ while B isn't independent, same
    // bParam()-style convention). Edits go through the same
    // onGraphCurveChanged() chokepoint a graph drag does. Selection is
    // mirrored both ways per bank; neither direction fires the other's
    // callback, so there's no feedback loop.
    nodeList_ = new DGL_NAMESPACE::NodeListWidget(this);
    nodeList_->setOnCurveEdited([this](const TransferCurve& c) { onGraphCurveChanged(false, c); });
    nodeList_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    nodeList_->setOnSelectionChanged([this](int i) { graph_->selectPoint(i); graphA_->selectPoint(i); });
    graph_->setOnSelectionChanged([this](int i) { nodeList_->setSelected(i); });
    graphA_->setOnSelectionChanged([this](int i) { nodeList_->setSelected(i); });
    nodeList_->setCurve(curve_);

    nodeListB_ = new DGL_NAMESPACE::NodeListWidget(this);
    nodeListB_->setOnCurveEdited([this](const TransferCurve& c) { onGraphCurveChanged(true, c); });
    nodeListB_->setOnThresholdChanged([this](float v) { onGraphParamChanged(bParam(true, kParamThreshold, kParamThresholdB), v); });
    nodeListB_->setOnSelectionChanged([this](int i) { graphB_->selectPoint(i); });
    graphB_->setOnSelectionChanged([this](int i) { nodeListB_->setSelected(i); });
    nodeListB_->setCurve(curve_);

    updateMeterLabelsForMode();

    strip_ = new ParamStripWidget(this);
    refreshStripFields();
    strip_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    strip_->setSetter([this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
        if (idx == kParamThreshold) refreshGraphThresholds();
        else if (idx == kParamOutputGain) refreshMakeupMarkers(); // immediate feedback while dragging
    });

    // stripA_ always targets A's own Parameters; stripB_'s fields
    // individually fall back to A's own matching Parameter via bParam()
    // whenever B isn't independent -- see refreshDualStripFields()'s own
    // comment.
    stripA_ = new ParamStripWidget(this);
    stripB_ = new ParamStripWidget(this);
    stripGlobal_ = new ParamStripWidget(this);
    stripGlobal_->setFields({ { "Output", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamGlobalOutputGain,
        "Output Gain: a shared master trim applied after everything else, in every stereo mode." } });
    stripGlobal_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    stripGlobal_->setSetter([this](uint32_t idx, float v) { paramCache_[idx] = v; setParameterValue(idx, v); });

    // A global gain on the stereo signal before splitting or converting to
    // M/S -- Dual-Mono/Mid-Side only.
    stripGlobalIn_ = new ParamStripWidget(this);
    stripGlobalIn_->setFields({ { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamGlobalInputGain,
        "Global Input Gain: a shared trim applied to the stereo signal before it's split (Dual-Mono) or decoded (Mid/Side)." } });
    stripGlobalIn_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    stripGlobalIn_->setSetter([this](uint32_t idx, float v) { paramCache_[idx] = v; setParameterValue(idx, v); });

    refreshDualStripFields();
    auto dualStripSetter = [this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
        if (idx == kParamThreshold || idx == kParamThresholdB)
            refreshGraphThresholds();
        else if (idx == kParamOutputGain || idx == kParamMakeupGainB)
            refreshMakeupMarkers();
    };
    stripA_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    stripA_->setSetter(dualStripSetter);
    stripB_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    stripB_->setSetter(dualStripSetter);

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] { setParameterValue(kParamBypass, bypassed_ ? 0.0f : 1.0f); });
    refreshBypassColors();

    btnSidechain_ = new SimpleButton(this, "");
    btnSidechain_->setCallback([this] { setParameterValue(kParamSidechainEnable, sidechainEnabled_ ? 0.0f : 1.0f); });
    refreshSidechainColors();

    // Tool button -- cycles Draw Node -> Draw Rising -> Draw Falling -> Edit
    // (see ContourGraphWidget.hpp's own header comment for what each does).
    btnTool_ = new SimpleButton(this, "");
    btnTool_->setCallback([this] {
        using Tool = DGL_NAMESPACE::ContourGraphWidget::EditTool;
        tool_ = Tool((int(tool_) + 1) % 4);
        refreshToolButton();
    });
    refreshToolButton();

    btnNodeList_ = new SimpleButton(this, "");
    btnNodeList_->setCallback([this] {
        nodeListVisible_ = !nodeListVisible_;
        refreshNodeListButtons();
        refreshDualLayoutVisibility();
    });
    refreshNodeListButtons();

    // A switch between the real input->output mapping and the detector view. Real audio (input level on x, the level after
    // the curve's gain on y) is the default; the detector view -- x = the level the curve is
    // actually read at -- is one click away.
    btnDots_ = new SimpleButton(this, "");
    btnDots_->setCallback([this] {
        setParameterValue(kParamTrailSource, paramCache_[kParamTrailSource] >= 0.5f ? 0.0f : 1.0f);
    });
    refreshDotsButton();

    btnStereoMode_ = new SimpleButton(this, stereoModeName(stereoMode_));
    btnStereoMode_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnStereoMode_->setCallback([this] {
        const int next = (int(stereoMode_) + 1) % 3;
        const bool enteringMidSide = sscomp::clampStereoMode(next) == StereoMode::MidSide && stereoMode_ != StereoMode::MidSide;
        const bool leavingLinked = stereoMode_ == StereoMode::Linked && sscomp::clampStereoMode(next) != StereoMode::Linked;
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%d", next);
        setState("stereoMode", buf);
        stereoMode_ = sscomp::clampStereoMode(next);
        // The node list panel now stacks its two banks vertically in the
        // dual layout (see refreshNodeListButtons()'s own header comment)
        // rather than needing the single list's width doubled, so
        // nodeListVisible_ no longer needs forcing off when leaving Linked.
        refreshNodeListButtons();
        if (enteringMidSide) syncBFromA();
        // Global Input Gain starts at 0dB on entering this mode.
        if (leavingLinked)
        {
            paramCache_[kParamGlobalInputGain] = 0.0f;
            setParameterValue(kParamGlobalInputGain, 0.0f);
            stripGlobalIn_->repaint();
        }
        refreshStereoModeLabel();
        updateMeterLabelsForMode();
    });

    // Link L/R parameters (Dual-Mono), as in the Compressor Stereo.
    btnLink_ = new SimpleButton(this, "");
    btnLink_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnLink_->setCallback([this] {
        const bool wasLinked = linkAB_;
        linkAB_ = !linkAB_;
        setState("linkAB", linkAB_ ? "1" : "0");
        if (wasLinked && !linkAB_) syncBFromA();
        refreshLinkLabel();
        updateMeterLabelsForMode();
    });
    refreshLinkLabel();

    // Listen is a single shared toggle button.
    btnListen_ = new SimpleButton(this, "Listen: Off");
    btnListen_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnListen_->setCallback([this] {
        listenMode_ = (listenMode_ + 1) % 3;
        setParameterValue(kParamListenMode, float(listenMode_));
        refreshListenColors();
    });
    refreshListenColors();

    // Linked mode only now -- always drives A's own condition_/curve_
    // unconditionally (Linked has no B bank at all); greyed out, not
    // hidden, while Dual-Mono/Mid-Side are active. See btnConditionA_/
    // btnConditionB_ etc. for the dual layout's own pair. (No more Interp
    // button -- interpolation moved per-point: right-click-drag a point
    // in the graph to give it its own tangent handle, see
    // ContourGraphWidget.hpp's own header comment.)
    btnCondition_ = new SimpleButton(this, "");
    btnCondition_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnCondition_->setCallback([this] {
        condition_ = (condition_ == TriggerCondition::Above) ? TriggerCondition::Below : TriggerCondition::Above;
        pushCurveParamMirror();
        pushMirrorCommit();
        pushAbState();
        refreshGraphConditions();
        refreshConditionLabel(btnCondition_, condition_);
    });
    refreshConditionLabel(btnCondition_, condition_);

    // Open/close choice: does a met trigger take the sound to the custom curve, or from the custom curve to
    // the diagonal? Curve = the original behaviour; Diagonal = a met trigger opens the sound up to unity.
    btnAction_ = new SimpleButton(this, "");
    btnAction_->setCallback([this] { toggleTriggerAction(false); });
    refreshActionLabel(btnAction_, action_);

    btnResetCurve_ = new SimpleButton(this, "Reset Curve");
    btnResetCurve_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnResetCurve_->setCallback([this] {
        curve_.resetToDefault();
        pushCurveParamMirror();
        pushMirrorCommit();
        pushAbState();
        refreshGraphCurves();
    });

    // Dual-Mono/Mid-Side's own pair, one set per displayed compressor --
    // doubled GUI objects instead of an editing switch, applied to
    // Condition/Reset too, not just Threshold/curve. The B
    // pair both follow the same bParam()-style live-link-while-not-
    // independent fallback every other B-bank control in the dual layout
    // already follows -- never disabled, editing them while linked edits
    // A's own actual state instead (same reasoning
    // onGraphCurveChanged(true, ...) already applies to a curve drag).
    btnConditionA_ = new SimpleButton(this, "");
    btnConditionA_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnConditionA_->setCallback([this] {
        condition_ = (condition_ == TriggerCondition::Above) ? TriggerCondition::Below : TriggerCondition::Above;
        pushCurveParamMirror();
        pushMirrorCommit();
        pushAbState();
        refreshGraphConditions();
        refreshConditionLabel(btnConditionA_, condition_);
        if (!isChannelBIndependent()) refreshConditionLabel(btnConditionB_, condition_);
    });
    refreshConditionLabel(btnConditionA_, condition_);

    btnConditionB_ = new SimpleButton(this, "");
    btnConditionB_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnConditionB_->setCallback([this] {
        if (isChannelBIndependent())
            conditionB_ = (conditionB_ == TriggerCondition::Above) ? TriggerCondition::Below : TriggerCondition::Above;
        else
        {
            condition_ = (condition_ == TriggerCondition::Above) ? TriggerCondition::Below : TriggerCondition::Above;
            refreshConditionLabel(btnConditionA_, condition_);
        }
        pushCurveParamMirror();
        pushMirrorCommit();
        pushAbState();
        refreshGraphConditions();
        refreshConditionLabel(btnConditionB_, isChannelBIndependent() ? conditionB_ : condition_);
    });
    refreshConditionLabel(btnConditionB_, isChannelBIndependent() ? conditionB_ : condition_);

    btnActionA_ = new SimpleButton(this, "");
    btnActionA_->setCallback([this] { toggleTriggerAction(false); });
    btnActionB_ = new SimpleButton(this, "");
    btnActionB_->setCallback([this] { toggleTriggerAction(true); });
    refreshActionLabels();

    btnResetCurveA_ = new SimpleButton(this, "Reset");
    btnResetCurveA_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnResetCurveA_->setCallback([this] {
        curve_.resetToDefault();
        pushCurveParamMirror();
        pushMirrorCommit();
        pushAbState();
        refreshGraphCurves();
    });

    btnResetCurveB_ = new SimpleButton(this, "Reset");
    btnResetCurveB_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnResetCurveB_->setCallback([this] {
        if (isChannelBIndependent())
            curveB_.resetToDefault();
        else
            curve_.resetToDefault();
        pushCurveParamMirror();
        pushMirrorCommit();
        pushAbState();
        refreshGraphCurves();
    });

    // The trigger options button (ported verbatim) -- see UICompressorStereo::
    // refreshScPanelVisibility()'s own comment for the full in-window-
    // overlay design.
    btnSidechainOptions_ = new SimpleButton(this, "Trigger Input...");
    btnSidechainOptions_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnSidechainOptions_->setCallback([this] { scPanelVisible_ = !scPanelVisible_; refreshScPanelVisibility(); });

    btnScFilterEnable_ = new SimpleButton(this, "");
    btnScFilterEnable_->setCallback([this] {
        const float next = paramCache_[kParamScFilterEnable] >= 0.5f ? 0.0f : 1.0f;
        setParameterValue(kParamScFilterEnable, next);
    });

    btnScListen_ = new SimpleButton(this, "Listen: Off");
    btnScListen_->setCallback([this] {
        const float next = paramCache_[kParamScListenEnable] >= 0.5f ? 0.0f : 1.0f;
        setParameterValue(kParamScListenEnable, next);
    });

    btnScLinkLR_ = new SimpleButton(this, "");
    btnScLinkLR_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnScLinkLR_->setCallback([this] {
        const bool wasLinked = scFilterLinkLR_;
        scFilterLinkLR_ = !scFilterLinkLR_;
        setParameterValue(kParamScFilterLinkLR, scFilterLinkLR_ ? 1.0f : 0.0f);
        if (wasLinked && !scFilterLinkLR_)
        {
            paramCache_[kParamScFilterHighpassHzR] = paramCache_[kParamScFilterHighpassHz];
            paramCache_[kParamScFilterLowpassHzR] = paramCache_[kParamScFilterLowpassHz];
            setParameterValue(kParamScFilterHighpassHzR, paramCache_[kParamScFilterHighpassHzR]);
            setParameterValue(kParamScFilterLowpassHzR, paramCache_[kParamScFilterLowpassHzR]);
        }
        if (scFilterLinkLR_ && editingScBankR_ != 0) { editingScBankR_ = 0; refreshScStripFields(); }
        refreshScLinkLabel();
        btnScEditBank_->setEnabled(!scFilterLinkLR_);
        if (scPanelVisible_)
            scScope_->setFilterRange(paramCache_[kParamScFilterEnable] >= 0.5f,
                                      editingScBankR_ == 0 ? paramCache_[kParamScFilterHighpassHz] : paramCache_[kParamScFilterHighpassHzR],
                                      editingScBankR_ == 0 ? paramCache_[kParamScFilterLowpassHz] : paramCache_[kParamScFilterLowpassHzR]);
    });
    refreshScLinkLabel();

    btnScEditBank_ = new SimpleButton(this, "Editing: L");
    btnScEditBank_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnScEditBank_->setEnabled(false);
    btnScEditBank_->setCallback([this] {
        editingScBankR_ = editingScBankR_ == 0 ? 1 : 0;
        btnScEditBank_->setLabel(editingScBankR_ == 0 ? "Editing: L" : "Editing: R");
        refreshScStripFields();
        scScope_->setFilterRange(paramCache_[kParamScFilterEnable] >= 0.5f,
                                  editingScBankR_ == 0 ? paramCache_[kParamScFilterHighpassHz] : paramCache_[kParamScFilterHighpassHzR],
                                  editingScBankR_ == 0 ? paramCache_[kParamScFilterLowpassHz] : paramCache_[kParamScFilterLowpassHzR]);
    });

    scStrip_ = new ParamStripWidget(this);
    refreshScStripFields();
    scStrip_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    scStrip_->setSetter([this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
        if (idx == kParamScFilterHighpassHz || idx == kParamScFilterLowpassHz ||
            idx == kParamScFilterHighpassHzR || idx == kParamScFilterLowpassHzR)
            scScope_->setFilterRange(paramCache_[kParamScFilterEnable] >= 0.5f,
                                      editingScBankR_ == 0 ? paramCache_[kParamScFilterHighpassHz] : paramCache_[kParamScFilterHighpassHzR],
                                      editingScBankR_ == 0 ? paramCache_[kParamScFilterLowpassHz] : paramCache_[kParamScFilterLowpassHzR]);
    });

    scScope_ = new DGL_NAMESPACE::SidechainScopeWidget(this);
    scScope_->setLevelTitle("Trigger Level (L up / R down)");
    scScope_->setOnHighpassChanged([this](float hz) {
        const uint32_t idx = editingScBankR_ == 0 ? kParamScFilterHighpassHz : kParamScFilterHighpassHzR;
        paramCache_[idx] = hz;
        setParameterValue(idx, hz);
        scStrip_->repaint();
    });
    scScope_->setOnLowpassChanged([this](float hz) {
        const uint32_t idx = editingScBankR_ == 0 ? kParamScFilterLowpassHz : kParamScFilterLowpassHzR;
        paramCache_[idx] = hz;
        setParameterValue(idx, hz);
        scStrip_->repaint();
    });

    btnScClose_ = new SimpleButton(this, "Close");
    btnScClose_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnScClose_->setCallback([this] { scPanelVisible_ = false; refreshScPanelVisibility(); });

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load Contour Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "preset.mccp";
        opts.title = "Save Contour Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnAbA_ = new SimpleButton(this, "A");
    btnAbA_->setCallback([this] { onAbA(); });
    btnAbB_ = new SimpleButton(this, "B");
    btnAbB_->setCallback([this] { onAbB(); });
    btnAbCopy_ = new SimpleButton(this, "Copy");
    btnAbCopy_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnAbCopy_->setCallback([this] { onAbCopy(); });
    refreshAbColors();

    btnUndo_ = new SimpleButton(this, "Undo");
    btnUndo_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnUndo_->setCallback([this] { onUndo(); });
    btnRedo_ = new SimpleButton(this, "Redo");
    btnRedo_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRedo_->setCallback([this] { onRedo(); });
    refreshUndoRedoColors();

    // Hover descriptions -- constructed last so it always paints on top
    // of everything above.
    tooltip_ = new DGL_NAMESPACE::TooltipOverlay(this);
    tooltip_->addTarget(btnLoad_, "Load a saved Contour preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a Contour preset.");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed, all metering/detection still runs.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    tooltip_->addTarget(btnStereoMode_, "Cycle Stereo Mode: Linked (one shared curve), Dual Mono (independent L/R), Mid/Side (independent Mid/Side).");
    tooltip_->addTarget(btnLink_, "Dual-Mono only: link the left and right curve/settings together (edit either, both follow); off lets them diverge.");
    tooltip_->addTarget(btnListen_, "Mid/Side only: audition Mid alone, then Side alone, then back to the normal mix.");
    tooltip_->addTarget(btnCondition_, "Trigger: Above / Below. The trigger is met while the trigger signal's level (after Trigger Gain and the Trigger Filter) is above / below the Threshold. The trigger only decides WHEN the sound takes the curve or the diagonal (see On Trigger); the curve itself always maps the sound's own level.");
    tooltip_->addTarget(btnResetCurve_, "Reset the drawn curve back to its default identity shape.");
    const char* actionTip = "The whole idea: the drawn curve is an input -> output mapping for your sound, and the trigger decides whether the sound takes the curve or stays on the diagonal (no change). This button says which. On Trigger: Curve -- while the trigger is met the sound goes on the curve; otherwise on the diagonal. On Trigger: Diagonal -- the other way round: while the trigger is met the sound is on the diagonal (a gate that opens), otherwise on the curve. Attack = how fast it moves to its new state when the trigger starts being met; Release = how fast it moves back when the trigger ends. 0 = instant. (The trigger's own gain, filter or sidechain never change where the sound sits on the curve.)";
    tooltip_->addTarget(btnAction_, actionTip);
    tooltip_->addTarget(btnActionA_, actionTip);
    tooltip_->addTarget(btnActionB_, actionTip);
    tooltip_->addTarget(btnConditionA_, "Same as Condition, for the left/Mid curve shown on the left.");
    tooltip_->addTarget(btnConditionB_, "Same as Condition, for the right/Side curve shown on the right (edits the left one too while Link is on).");
    tooltip_->addTarget(btnResetCurveA_, "Reset the left/Mid curve to its default identity shape.");
    tooltip_->addTarget(btnResetCurveB_, "Reset the right/Side curve to its default identity shape (resets the left one too while Link is on).");
    tooltip_->addTarget(btnSidechain_, "Trigger source: detect from the main signal (Main Input) or from the external Sidechain input. Trigger Gain and the Trigger Filter apply to whichever is selected.");
    tooltip_->addTarget(btnSidechainOptions_, "Open the Trigger Input panel: gain, Highpass/Lowpass filter and a live scope for the signal that drives the curve -- works on the main signal too, e.g. for a gate that only opens on a frequency band.");
    tooltip_->addTarget(btnScFilterEnable_, "Enable the Trigger Filter (Highpass/Lowpass) on the trigger signal, so the curve only reacts to that frequency band. Doesn't touch the audio you hear.");
    tooltip_->addTarget(btnScListen_, "Audition the trigger signal itself (gained and filtered as set here) in place of the main output.");
    tooltip_->addTarget(btnScLinkLR_, "Link the Trigger Filter's Left and Right corners together; off lets them diverge.");
    tooltip_->addTarget(btnScEditBank_, "Switch which side (L or R) the Highpass/Lowpass fields and drag handles below are editing (unlinked only).");
    tooltip_->addTarget(btnScClose_, "Close the Trigger Input panel and return to the normal view.");
    tooltip_->addTarget(btnTool_, "What a drag on the graph does. Draw Node / Draw Rising / Draw Falling: every click places a node (drag = smooth node with Bezier support points); Rising/Falling nodes only act while the level rises/falls (hysteresis). Edit: move nodes and support points, drag a window to select several. Click to switch.");
    tooltip_->addTarget(btnDots_, "(hidden) The dots always show the real sound: its level against its level after the gain. Green dots = the trigger is met.");    tooltip_->addTarget(btnNodeList_, "Show or hide the numeric node list: edit every node's In/Out level, support points and Rising/Falling tag by typing, dragging up/down or scrolling. Dual-Mono/Mid-Side show both banks' lists at once, stacked.");
    tooltip_->addFieldTarget(graph_, [this](double x, double y) { return graph_->tooltipAt(x, y); });
    tooltip_->addFieldTarget(graphA_, [this](double x, double y) { return graphA_->tooltipAt(x, y); });
    tooltip_->addFieldTarget(graphB_, [this](double x, double y) { return graphB_->tooltipAt(x, y); });
    tooltip_->addFieldTarget(nodeList_, [this](double x, double y) { return nodeList_->cellTooltipAt(x, y); });
    tooltip_->addFieldTarget(nodeListB_, [this](double x, double y) { return nodeListB_->cellTooltipAt(x, y); });
    tooltip_->addFieldTarget(strip_, [this](double x, double y) { return strip_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripA_, [this](double x, double y) { return stripA_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripB_, [this](double x, double y) { return stripB_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripGlobal_, [this](double x, double y) { return stripGlobal_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripGlobalIn_, [this](double x, double y) { return stripGlobalIn_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(scStrip_, [this](double x, double y) { return scStrip_->fieldTooltipAt(x, y); });

    layout();
    refreshScPanelVisibility();
}

void UIContour::updateMeterLabelsForMode()
{
    const bool midSide = stereoMode_ == StereoMode::MidSide;
    if (midSide)
    {
        meterIn_->setChannelLabels("M", "S");
        meterOut_->setChannelLabels("M", "S");
    }
    else
    {
        meterIn_->setChannelLabels("L", "R");
        meterOut_->setChannelLabels("L", "R");
    }
    // graphA_'s/graphB_'s own primary GR-column label -- see
    // UICompressorStereo::updateMeterLabelsForMode()'s identical comment
    // for why setSecondaryLabels() is reused purely for this side effect.
    graphA_->setSecondaryLabels(midSide ? "GR Mid" : "GR L", "");
    graphB_->setSecondaryLabels(midSide ? "GR Side" : "GR R", "");

    // btnListen_ doesn't exist yet the first time this runs -- guard
    // rather than reorder construction, same convention as the sibling
    // compressor's own identical guard.
    if (btnListen_ == nullptr) return;
    btnListen_->setEnabled(midSide);
    if (!midSide && listenMode_ != 0)
    {
        listenMode_ = 0;
        setParameterValue(kParamListenMode, 0.0f);
        refreshListenColors();
    }

    btnLink_->setEnabled(stereoMode_ == StereoMode::DualMono);
    btnCondition_->setEnabled(stereoMode_ == StereoMode::Linked);
    btnAction_->setEnabled(stereoMode_ == StereoMode::Linked);
    btnResetCurve_->setEnabled(stereoMode_ == StereoMode::Linked);

    refreshDualLayoutVisibility();

    refreshStripFields();
    refreshDualStripFields();
    refreshGraphCurves();
    refreshGraphThresholds();
    refreshGraphConditions();
    refreshConditionLabel(btnConditionB_, isChannelBIndependent() ? conditionB_ : condition_);
    refreshActionLabels();
}

void UIContour::layout()
{
    // Same two-row layout as UICompressorStereo's own -- see that file's
    // own comment for the full reasoning. Row 2 here: Stereo Mode, Link,
    // Listen, Condition/Reset (Linked only), Sidechain, Sidechain Options.
    const float w = float(getWidth());
    const float h = float(getHeight());

    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));

    constexpr float pad = 6.0f;
    constexpr float topBarH = 32.0f;
    constexpr float stripH = 72.0f;
    constexpr float meterW = 100.0f;
    constexpr float gap = 6.0f;

    float rx = w - pad;
    auto placeRight = [&](SimpleButton* b, float bw) {
        rx -= bw;
        b->setAbsolutePos(int(rx), int(pad));
        b->setSize(uint(bw), uint(topBarH));
        rx -= gap;
    };
    placeRight(btnRedo_, 56.0f);
    placeRight(btnUndo_, 56.0f);
    placeRight(btnAbCopy_, 56.0f);
    placeRight(btnAbB_, 30.0f);
    placeRight(btnAbA_, 30.0f);
    placeRight(btnBypass_, 70.0f);
    placeRight(btnSave_, 90.0f);
    placeRight(btnLoad_, 70.0f);

    constexpr float topBar2H = 26.0f;
    const float row2Y = topBarH + gap;
    float bx2 = pad;
    auto place2 = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx2), int(row2Y));
        b->setSize(uint(bw), uint(topBar2H));
        bx2 += bw + gap;
    };
    if (!kContourMono) // the mono build has no stereo mode / link / listen (they are hidden below)
    {
        place2(btnStereoMode_, 150.0f);
        place2(btnLink_, 80.0f);
        place2(btnListen_, 105.0f);
    }
    place2(btnCondition_, 130.0f); // Linked mode only now
    place2(btnAction_, 165.0f);    // ditto
    place2(btnResetCurve_, 100.0f);
    place2(btnSidechain_, 175.0f);
    place2(btnSidechainOptions_, 150.0f);

    const float mainY = row2Y + topBar2H + pad * 2.0f;
    const float mainH = h - mainY - stripH - pad * 2.0f;

    meterIn_->setAbsolutePos(int(pad), int(mainY));
    meterIn_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    meterOut_->setAbsolutePos(int(w - pad - meterW), int(mainY));
    meterOut_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    const float graphX = pad + meterW + pad;
    // Node list docked on the right of the graph area while shown; the
    // Draw/Edit tool row sits above the graph(s), so everything below starts
    // at contentY. (The list itself spans the full height, tool row
    // included -- it has no need for the row.)
    constexpr float toolRowH = 24.0f;
    constexpr float listPanelW = 368.0f;
    const bool listShown = nodeListVisible_ && !scPanelVisible_;
    const float graphAreaW = w - graphX - meterW - pad - (listShown ? listPanelW + gap : 0.0f);
    const float contentY = mainY + toolRowH + gap;
    const float contentH = mainH - toolRowH - gap;

    // Tool | Node List On/Off. The row must end before the node list panel (which spans
    // the row's height too), so when the window is narrow the buttons shrink together instead of overlapping it.
    {
        float bw[2] = { 210.0f, 160.0f };
        SimpleButton* rowBtns[2] = { btnTool_, btnNodeList_ };
        const float total = bw[0] + bw[1] + gap;
        const float scale = std::min(1.0f, std::max(0.4f, (graphAreaW - gap) / (total - gap)));
        float bx = graphX;
        for (int k = 0; k < 2; ++k)
        {
            const float wk = std::floor(bw[k] * scale);
            rowBtns[k]->setAbsolutePos(int(bx), int(mainY));
            rowBtns[k]->setSize(uint(wk), uint(toolRowH));
            bx += wk + gap;
        }
    }
    // One list, full height, in Linked mode; two, stacked vertically with a
    // small gap, in Dual-Mono/Mid-Side -- see refreshNodeListButtons()'s own
    // header comment (UIContour.hpp) for why vertical rather than doubling
    // the panel's width.
    {
        const bool dualLayout = stereoMode_ != StereoMode::Linked;
        const float listX = w - pad - meterW - listPanelW;
        constexpr float listGap = 8.0f;
        const float listTopH = dualLayout ? std::max(10.0f, (mainH - listGap) * 0.5f) : mainH;
        nodeList_->setAbsolutePos(int(listX), int(mainY));
        nodeList_->setSize(uint(listPanelW), uint(std::max(10.0f, listTopH)));
        nodeListB_->setAbsolutePos(int(listX), int(mainY + listTopH + listGap));
        nodeListB_->setSize(uint(listPanelW), uint(std::max(10.0f, mainH - listTopH - listGap)));
    }

    graph_->setAbsolutePos(int(graphX), int(contentY));
    graph_->setSize(uint(std::max(10.0f, graphAreaW)), uint(std::max(10.0f, contentH)));

    strip_->setAbsolutePos(int(pad), int(h - stripH - pad));
    strip_->setSize(uint(w - 2.0f * pad), uint(stripH));

    // Dual-Mono/Mid-Side widget set. Each column's own Condition/Reset
    // pair sits above its own (correspondingly shorter) graph.
    constexpr float dualColGap = 8.0f;
    constexpr float curveBtnRowH = 24.0f;
    constexpr float curveBtnGap = 4.0f;
    const float colW = (graphAreaW - dualColGap) * 0.5f;
    const float colAx = graphX;
    const float colBx = graphX + colW + dualColGap;
    const float dualGraphY = contentY + curveBtnRowH + gap;
    const float dualGraphH = contentH - curveBtnRowH - gap;

    // Condition | On-trigger action | Reset: 34% / 40% / the rest of each column.
    auto placeCurveBtnRow = [&](float colX, SimpleButton* cond, SimpleButton* action, SimpleButton* reset) {
        const float usable = colW - 2.0f * curveBtnGap;
        const float condW = std::max(10.0f, usable * 0.34f);
        const float actionW = std::max(10.0f, usable * 0.40f);
        const float resetW = std::max(10.0f, usable - condW - actionW);
        cond->setAbsolutePos(int(colX), int(contentY));
        cond->setSize(uint(condW), uint(curveBtnRowH));
        action->setAbsolutePos(int(colX + condW + curveBtnGap), int(contentY));
        action->setSize(uint(actionW), uint(curveBtnRowH));
        reset->setAbsolutePos(int(colX + condW + curveBtnGap + actionW + curveBtnGap), int(contentY));
        reset->setSize(uint(resetW), uint(curveBtnRowH));
    };
    placeCurveBtnRow(colAx, btnConditionA_, btnActionA_, btnResetCurveA_);
    placeCurveBtnRow(colBx, btnConditionB_, btnActionB_, btnResetCurveB_);

    graphA_->setAbsolutePos(int(colAx), int(dualGraphY));
    graphA_->setSize(uint(std::max(10.0f, colW)), uint(std::max(10.0f, dualGraphH)));
    graphB_->setAbsolutePos(int(colBx), int(dualGraphY));
    graphB_->setSize(uint(std::max(10.0f, colW)), uint(std::max(10.0f, dualGraphH)));

    // stripGlobalIn_ under meterIn_, on the far left -- then stripA_/
    // stripB_ each under their own graph, then stripGlobal_ on the right.
    constexpr float stripGlobalW = 110.0f;
    const float stripSideW = (w - 2.0f * pad - meterW - gap - stripGlobalW - 3.0f * gap) * 0.5f;
    const float dualStripY = h - stripH - pad;
    stripGlobalIn_->setAbsolutePos(int(pad), int(dualStripY));
    stripGlobalIn_->setSize(uint(meterW), uint(stripH));
    stripA_->setAbsolutePos(int(pad + meterW + gap), int(dualStripY));
    stripA_->setSize(uint(std::max(10.0f, stripSideW)), uint(stripH));
    stripB_->setAbsolutePos(int(pad + meterW + gap + stripSideW + gap), int(dualStripY));
    stripB_->setSize(uint(std::max(10.0f, stripSideW)), uint(stripH));
    stripGlobal_->setAbsolutePos(int(pad + meterW + gap + stripSideW + gap + stripSideW + gap), int(dualStripY));
    stripGlobal_->setSize(uint(stripGlobalW), uint(stripH));

    // Sidechain-options overlay panel -- verbatim port of
    // UICompressorStereo::layout()'s identical block.
    constexpr float scButtonRowH = 26.0f;
    const float panelY = mainY;
    const float panelH = h - panelY - pad;
    btnScFilterEnable_->setAbsolutePos(int(pad), int(panelY));
    btnScFilterEnable_->setSize(160, uint(scButtonRowH));
    btnScListen_->setAbsolutePos(int(pad) + 160 + 6, int(panelY));
    btnScListen_->setSize(110, uint(scButtonRowH));
    btnScLinkLR_->setAbsolutePos(int(pad) + 160 + 6 + 110 + 6, int(panelY));
    btnScLinkLR_->setSize(100, uint(scButtonRowH));
    btnScEditBank_->setAbsolutePos(int(pad) + 160 + 6 + 110 + 6 + 100 + 6, int(panelY));
    btnScEditBank_->setSize(100, uint(scButtonRowH));
    btnScClose_->setAbsolutePos(int(w - pad - 80.0f), int(panelY));
    btnScClose_->setSize(80, uint(scButtonRowH));
    const float scopeY = panelY + scButtonRowH + gap;
    const float scStripH = 64.0f;
    const float scopeH = panelH - scButtonRowH - gap - scStripH - gap;
    scScope_->setAbsolutePos(int(pad), int(scopeY));
    scScope_->setSize(uint(w - 2.0f * pad), uint(std::max(10.0f, scopeH)));
    scStrip_->setAbsolutePos(int(pad), int(panelY + panelH - scStripH));
    scStrip_->setSize(uint(w - 2.0f * pad), uint(scStripH));

    if (kContourMono)
    {
        // Mono build: no stereo mode / link / listen (hidden widgets take no clicks and no room).
        btnStereoMode_->setVisible(false);
        btnLink_->setVisible(false);
        btnListen_->setVisible(false);
    }
}

void UIContour::onGraphParamChanged(uint32_t index, float value)
{
    paramCache_[index] = value;
    setParameterValue(index, value);
    strip_->repaint();
    stripA_->repaint();
    stripB_->repaint();
}

uint32_t UIContour::bParam(bool isB, uint32_t paramA, uint32_t paramB) const
{
    return (isB && isChannelBIndependent()) ? paramB : paramA;
}

void UIContour::onGraphCurveChanged(bool isB, const TransferCurve& curve)
{
    if (isB && isChannelBIndependent())
        curveB_ = curve;
    else
        curve_ = curve;
    pushCurveParamMirror();
    pushMirrorCommit();
    pushAbState();
    refreshGraphCurves();
}

void UIContour::refreshGraphCurves()
{
    graph_->setCurve(curve_);
    graphA_->setCurve(curve_);
    graphB_->setCurve(isChannelBIndependent() ? curveB_ : curve_);
    refreshNodeList();
}

void UIContour::refreshGraphThresholds()
{
    graph_->setThreshold(paramCache_[kParamThreshold]);
    graphA_->setThreshold(paramCache_[kParamThreshold]);
    graphB_->setThreshold(isChannelBIndependent() ? paramCache_[kParamThresholdB] : paramCache_[kParamThreshold]);
    if (nodeList_ != nullptr)
        nodeList_->setThreshold(paramCache_[kParamThreshold]);
    if (nodeListB_ != nullptr)
        nodeListB_->setThreshold(isChannelBIndependent() ? paramCache_[kParamThresholdB] : paramCache_[kParamThreshold]);
    refreshMakeupMarkers(); // every place the scalars are re-read (ctor, snapshot/preset load, B link changes) also refreshes the bars
}

void UIContour::refreshMakeupMarkers()
{
    const float a = paramCache_[kParamOutputGain];
    const float b = isChannelBIndependent() ? paramCache_[kParamMakeupGainB] : a;
    graph_->setMakeupGainDb(a, a);
    graphA_->setMakeupGainDb(a, a);
    graphB_->setMakeupGainDb(b, b);
}

void UIContour::refreshGraphConditions()
{
    graph_->setTriggerCondition(condition_);
    graphA_->setTriggerCondition(condition_);
    graphB_->setTriggerCondition(isChannelBIndependent() ? conditionB_ : condition_);
}

void UIContour::pushCurveParamMirror()
{
    setParameterValue(kParamCurvePointCount, float(curve_.pointCount()));
    setParameterValue(kParamCurveTriggerCondition, sscomp::packConditionAndThresholdLink(int(condition_), curve_.thresholdLink(), int(action_)));
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const CurvePoint p = (i < curve_.pointCount()) ? curve_.point(i) : CurvePoint();
        setParameterValue(kParamCurvePointInBase + i, float(p.inputDb));
        setParameterValue(kParamCurvePointOutBase + i, float(p.outputDb));
    }
    // B's own mirror -- pushed every time too, regardless of which bank
    // isB actually named in onGraphCurveChanged() -- see this method's
    // own declaration comment (UIContour.hpp).
    setParameterValue(kParamCurvePointCountB, float(curveB_.pointCount()));
    setParameterValue(kParamCurveTriggerConditionB, sscomp::packConditionAndThresholdLink(int(conditionB_), curveB_.thresholdLink(), int(actionB_)));
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const CurvePoint p = (i < curveB_.pointCount()) ? curveB_.point(i) : CurvePoint();
        setParameterValue(kParamCurvePointInBaseB + i, float(p.inputDb));
        setParameterValue(kParamCurvePointOutBaseB + i, float(p.outputDb));
    }
    // Per-point hysteresis direction tag -- see kParamCurvePointDirBase's
    // own comment (PluginContour.hpp). Pushed last, right before
    // pushMirrorCommit() actually fires the commit trigger(s), same
    // "every mirror field current before the commit" discipline every
    // other bank above already follows.
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const CurvePoint p = (i < curve_.pointCount()) ? curve_.point(i) : CurvePoint();
        setParameterValue(kParamCurvePointDirBase + i, sscomp::packDirectionAndLink(p.direction, p.link, p.linkAxis, p.diagonal));
    }
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const CurvePoint p = (i < curveB_.pointCount()) ? curveB_.point(i) : CurvePoint();
        setParameterValue(kParamCurvePointDirBaseB + i, sscomp::packDirectionAndLink(p.direction, p.link, p.linkAxis, p.diagonal));
    }
    // Per-point Inkscape-style tangent handle -- see
    // kParamCurvePointHandleInBase's own comment (PluginContour.hpp).
    // Pushed last of all (right before pushMirrorCommit() fires), same
    // "every mirror field current before the commit" discipline as the
    // direction bank just above.
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const CurvePoint p = (i < curve_.pointCount()) ? curve_.point(i) : CurvePoint();
        setParameterValue(kParamCurvePointHandleInBase + i, float(p.handleInputDb));
        setParameterValue(kParamCurvePointHandleOutBase + i, float(p.handleOutputDb));
    }
    for (int i = 0; i < int(kNumMirroredPoints); ++i)
    {
        const CurvePoint p = (i < curveB_.pointCount()) ? curveB_.point(i) : CurvePoint();
        setParameterValue(kParamCurvePointHandleInBaseB + i, float(p.handleInputDb));
        setParameterValue(kParamCurvePointHandleOutBaseB + i, float(p.handleOutputDb));
    }
}

void UIContour::pushMirrorCommit()
{
    lastLocalMirrorEdit_ = std::chrono::steady_clock::now();
    hasLocalMirrorEdit_ = true;
    // Both triggers, every time -- kParamMirrorCommitB is the one
    // guaranteed to always arrive last (see its own comment,
    // PluginContour.hpp); kParamMirrorCommit fires too so a build that
    // ever changes the index layout back still works, and because a
    // redundant idempotent apply is harmless either way.
    // An INCREASING counter, not a 0/1 toggle: the host hands the plugin only
    // ports whose value differs from the one it saw last (DPF's lv2_run compares
    // against fLastControlValues), so with a toggle an even number of edits inside
    // one audio block -- a fast drag -- netted out to "no change", the commit was
    // never delivered and the DSP kept the OLD curve until the next edit ("the
    // points show the old curve; slightly moving the node fixes it"). Wraps at
    // 1e6 (exactly representable), skipping 0 so it never equals the initial value.
    mirrorCommitCounter_ = mirrorCommitCounter_ >= 999999.0f ? 1.0f : mirrorCommitCounter_ + 1.0f;
    setParameterValue(kParamMirrorCommit, mirrorCommitCounter_);
    setParameterValue(kParamMirrorCommitB, mirrorCommitCounter_);
}

std::string UIContour::serializeSnapshot(const ContourSnapshot& snap) const
{
    std::string s;
    for (const auto& kv : snap.values)
    {
        char buf[64];
        const auto res = std::to_chars(buf, buf + sizeof(buf), kv.second);
        s += kv.first + "=" + std::string(buf, res.ptr) + "\n";
    }
    s += "curve=" + snap.curve + "\n";
    s += "curveB=" + snap.curveB + "\n";
    return s;
}

bool UIContour::deserializeSnapshot(const std::string& text, ContourSnapshot& out) const
{
    presetio::KeyValues values;
    std::string curveStr, curveBStr;
    size_t pos = 0;
    while (pos < text.size())
    {
        const size_t nl = text.find('\n', pos);
        const std::string line = text.substr(pos, nl == std::string::npos ? std::string::npos : nl - pos);
        pos = (nl == std::string::npos) ? text.size() : nl + 1;
        if (line.empty()) continue;
        if (line.compare(0, 7, "curveB=") == 0) { curveBStr = line.substr(7); continue; }
        if (line.compare(0, 6, "curve=") == 0) { curveStr = line.substr(6); continue; }
        const size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        const std::string val = line.substr(eq + 1);
        double v = 0.0;
        const auto res = std::from_chars(val.data(), val.data() + val.size(), v);
        if (res.ec == std::errc()) values[key] = v;
    }
    if (curveStr.empty()) return false;
    out.values = values;
    out.curve = curveStr;
    // A snapshot serialized before B's own curve existed has no "curveB="
    // line -- fall back to a fresh identity curve rather than leaving
    // curveB_ untouched, same "fully apply, never partially" discipline
    // applySnapshot() itself already follows.
    out.curveB = curveBStr.empty() ? sscomp::curveToString(TransferCurve()) : curveBStr;
    return true;
}

void UIContour::pushAbState()
{
    if (abActive_ == AbSlot::A) abSlotA_ = captureSnapshot();
    else abSlotB_ = captureSnapshot();

    setState("abSlotA", serializeSnapshot(abSlotA_).c_str());
    setState("abSlotB", serializeSnapshot(abSlotB_).c_str());
    setState("abHasA", abHasA_ ? "1" : "0");
    setState("abHasB", abHasB_ ? "1" : "0");
    setState("abActive", abActive_ == AbSlot::B ? "1" : "0");
}

void UIContour::refreshBypassColors()
{
    btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                           bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                           Color(230, 232, 236));
}

void UIContour::refreshStripFields()
{
    // Linked mode only now -- always A's own Parameters unconditionally,
    // with Wet/Dry included.
    strip_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamInputGain,
          "Input Gain: trim applied to the signal before the curve." },
        { "Threshold", "dB", float(TransferCurve::kMinDb), float(TransferCurve::kMaxDb), -18.0f, 0.5f, false, 1, kParamThreshold,
          "Threshold: the trigger level the trigger (Above/Below) is compared with. It only decides when the sound takes the curve or the diagonal -- it is not part of the curve." },
        { "Attack", "ms", 0.0f, 300.0f, 10.0f, 0.05f, true, 2, kParamAttack,
          "Attack: how fast the sound moves to its new state (the curve or the diagonal, see On Trigger) when the trigger starts being met. 0 = instant." },
        { "Release", "ms", 0.0f, 2000.0f, 150.0f, 1.0f, true, 1, kParamRelease,
          "Release: how fast the sound moves back when the trigger ends (and how fast the curve's level reading falls). 0 = instant." },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamOutputGain,
          "Makeup Gain: trim applied to the curve's own output, before Wet/Dry mixing." },
        { "Wet/Dry", "%", 0.0f, 100.0f, 100.0f, 1.0f, false, 0, kParamWetDry,
          "Wet/Dry: blends the dry, uncurved signal (0%) with the fully curve-processed signal (100%).",
          true, "Wet", "Dry" },
        { "Output", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamGlobalOutputGain,
          "Output Gain: a shared master trim applied after everything else." },
    });
    repaint();
}

void UIContour::refreshDualStripFields()
{
    // Tooltips adapt to the mode -- see UICompressorStereo::refreshDualStripFields()'s
    // own identical comment.
    const bool bIndep = isChannelBIndependent();
    const bool midSide = stereoMode_ == StereoMode::MidSide;
    const std::string bChan = midSide ? "Side" : "right (R)";
    const std::string aChan = midSide ? "Mid" : "left (L)";
    const std::string bNote = bIndep
        ? (" Independent from the " + aChan + " channel.")
        : (" Linked to the " + aChan + " channel right now -- edit either, both follow.");

    stripA_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamInputGain,
          "Input Gain: trim applied to the " + aChan + " channel before its own curve." },
        { "Threshold", "dB", float(TransferCurve::kMinDb), float(TransferCurve::kMaxDb), -18.0f, 0.5f, false, 1, kParamThreshold,
          "Threshold: the trigger level the " + aChan + " channel's trigger (Above/Below) is compared with -- it only decides when its sound takes the curve or the diagonal." },
        { "Attack", "ms", 0.0f, 300.0f, 10.0f, 0.05f, true, 2, kParamAttack,
          "Attack: how fast the " + aChan + " channel moves to its new state (curve or diagonal) when its trigger starts being met. 0 = instant." },
        { "Release", "ms", 0.0f, 2000.0f, 150.0f, 1.0f, true, 1, kParamRelease,
          "Release: how fast the " + aChan + " channel moves back when its trigger ends. 0 = instant." },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamOutputGain,
          "Makeup Gain: trim applied to the " + aChan + " channel's own curve output." },
        { "Wet/Dry", "%", 0.0f, 100.0f, 100.0f, 1.0f, false, 0, kParamWetDry,
          "Wet/Dry: blends dry (0%) and fully curve-processed (100%) signal on the " + aChan + " channel.",
          true, "Wet", "Dry" },
    });
    stripB_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, bParam(true, kParamInputGain, kParamInputGainB),
          "Input Gain: trim applied to the " + bChan + " channel before its own curve." + bNote },
        { "Threshold", "dB", float(TransferCurve::kMinDb), float(TransferCurve::kMaxDb), -18.0f, 0.5f, false, 1, bParam(true, kParamThreshold, kParamThresholdB),
          "Threshold: the trigger level the " + bChan + " channel's trigger (Above/Below) is compared with -- it only decides when its sound takes the curve or the diagonal." + bNote },
        { "Attack", "ms", 0.0f, 300.0f, 10.0f, 0.05f, true, 2, bParam(true, kParamAttack, kParamAttackB),
          "Attack: how fast the " + bChan + " channel moves to its new state (curve or diagonal) when its trigger starts being met. 0 = instant." + bNote },
        { "Release", "ms", 0.0f, 2000.0f, 150.0f, 1.0f, true, 1, bParam(true, kParamRelease, kParamReleaseB),
          "Release: how fast the " + bChan + " channel moves back when its trigger ends. 0 = instant." + bNote },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, bParam(true, kParamOutputGain, kParamMakeupGainB),
          "Makeup Gain: trim applied to the " + bChan + " channel's own curve output." + bNote },
        { "Wet/Dry", "%", 0.0f, 100.0f, 100.0f, 1.0f, false, 0, bParam(true, kParamWetDry, kParamWetDryB),
          "Wet/Dry: blends dry (0%) and fully curve-processed (100%) signal on the " + bChan + " channel." + bNote,
          true, "Wet", "Dry" },
    });
    repaint();
}

void UIContour::refreshScLinkLabel()
{
    btnScLinkLR_->setColors(scFilterLinkLR_ ? Color(70, 150, 90) : Color(58, 62, 70),
                             scFilterLinkLR_ ? Color(85, 175, 110) : Color(78, 84, 96),
                             Color(230, 232, 236));
    btnScLinkLR_->setLabel(scFilterLinkLR_ ? "L/R Link: On" : "L/R Link: Off");
}

void UIContour::refreshScStripFields()
{
    const bool r = editingScBankR_ == 1;
    const std::string side = r ? "right (R)" : (scFilterLinkLR_ ? "left+right (linked)" : "left (L)");
    scStrip_->setFields({
        { "Highpass", "Hz", 20.0f, 8000.0f, 100.0f, 0.0f, true, 0, r ? kParamScFilterHighpassHzR : kParamScFilterHighpassHz,
          "Trigger Highpass: removes content below this frequency from the " + side + " trigger signal before it reaches the detector." },
        { "Lowpass", "Hz", 100.0f, 20000.0f, 8000.0f, 0.0f, true, 0, r ? kParamScFilterLowpassHzR : kParamScFilterLowpassHz,
          "Trigger Lowpass: removes content above this frequency from the " + side + " trigger signal before it reaches the detector." },
        { "Trigger Gain", "dB", float(sscomp::kSidechainGainMinDb), float(sscomp::kSidechainGainMaxDb), 0.0f, 0.5f, false, 1, kParamSidechainGain,
          "Trigger Gain: amplifies the trigger signal (main signal or external Sidechain, whichever is selected) before the filter and the detector. Doesn't change the audio you hear, only what drives the curve." },
        { "Lookahead", "ms", 0.0f, 20.0f, 0.0f, 0.1f, false, 1, kParamLookahead,
          "Lookahead: delays the audio by this much (the host compensates the latency) while the trigger is read un-delayed, so the curve -- or the diagonal -- is already in place when the sound that triggered it arrives, even with Attack 0. A change in the release direction is held for the same time, so the tail of a triggering sound is not cut. Pick about twice your Attack (or more)." },
    });
    if (scScope_ != nullptr) scScope_->setEditingRight(r);
    repaint();
}

void UIContour::refreshListenColors()
{
    const bool active = listenMode_ != 0;
    btnListen_->setColors(active ? Color(70, 150, 90) : Color(58, 62, 70),
                           active ? Color(85, 175, 110) : Color(78, 84, 96),
                           Color(230, 232, 236));
    const char* label = listenMode_ == 1 ? "Listen: Mid" : listenMode_ == 2 ? "Listen: Side" : "Listen: Off";
    btnListen_->setLabel(label);
}

bool UIContour::isChannelBIndependent() const
{
    return stereoMode_ == StereoMode::MidSide ||
           (stereoMode_ == StereoMode::DualMono && !linkAB_);
}

void UIContour::refreshLinkLabel()
{
    btnLink_->setColors(linkAB_ ? Color(70, 150, 90) : Color(58, 62, 70),
                         linkAB_ ? Color(85, 175, 110) : Color(78, 84, 96),
                         Color(230, 232, 236));
    btnLink_->setLabel(linkAB_ ? "Link: On" : "Link: Off");
}

void UIContour::refreshDualLayoutVisibility()
{
    const bool dual = stereoMode_ != StereoMode::Linked;
    const bool showMain = !scPanelVisible_;
    meterIn_->setVisible(showMain);
    meterOut_->setVisible(showMain);
    btnTool_->setVisible(showMain);
    btnNodeList_->setVisible(showMain);
    nodeList_->setVisible(showMain && nodeListVisible_);
    nodeListB_->setVisible(showMain && nodeListVisible_ && dual);
    graph_->setVisible(showMain && !dual);
    strip_->setVisible(showMain && !dual);
    graphA_->setVisible(showMain && dual);
    graphB_->setVisible(showMain && dual);
    stripA_->setVisible(showMain && dual);
    stripB_->setVisible(showMain && dual);
    stripGlobal_->setVisible(showMain && dual);
    stripGlobalIn_->setVisible(showMain && dual);
    btnConditionA_->setVisible(showMain && dual);
    btnConditionB_->setVisible(showMain && dual);
    btnActionA_->setVisible(showMain && dual);
    btnActionB_->setVisible(showMain && dual);
    btnResetCurveA_->setVisible(showMain && dual);
    btnResetCurveB_->setVisible(showMain && dual);
    layout();
    repaint();
}

void UIContour::syncBFromA()
{
    paramCache_[kParamThresholdB] = paramCache_[kParamThreshold];
    paramCache_[kParamAttackB] = paramCache_[kParamAttack];
    paramCache_[kParamReleaseB] = paramCache_[kParamRelease];
    paramCache_[kParamMakeupGainB] = paramCache_[kParamOutputGain];
    paramCache_[kParamInputGainB] = paramCache_[kParamInputGain];
    paramCache_[kParamWetDryB] = paramCache_[kParamWetDry];
    setParameterValue(kParamThresholdB, paramCache_[kParamThresholdB]);
    setParameterValue(kParamAttackB, paramCache_[kParamAttackB]);
    setParameterValue(kParamReleaseB, paramCache_[kParamReleaseB]);
    setParameterValue(kParamMakeupGainB, paramCache_[kParamMakeupGainB]);
    setParameterValue(kParamInputGainB, paramCache_[kParamInputGainB]);
    setParameterValue(kParamWetDryB, paramCache_[kParamWetDryB]);
    // The curve (including every point's own handle/direction tag) and
    // condition too -- B's own curve is copied just like every scalar
    // field above.
    curveB_ = curve_;
    conditionB_ = condition_;
    actionB_ = action_;
    pushCurveParamMirror();
    pushMirrorCommit();
    pushAbState();
    refreshDualStripFields();
    refreshGraphCurves();
    refreshGraphThresholds();
    refreshGraphConditions();
    refreshConditionLabel(btnConditionB_, conditionB_);
    refreshActionLabels();
}

void UIContour::refreshSidechainColors()
{
    btnSidechain_->setColors(sidechainEnabled_ ? Color(70, 150, 90) : Color(58, 62, 70),
                              sidechainEnabled_ ? Color(85, 175, 110) : Color(78, 84, 96),
                              Color(230, 232, 236));
    btnSidechain_->setLabel(sidechainEnabled_ ? "Trigger: Sidechain In" : "Trigger: Main Input");
}

void UIContour::refreshNodeList()
{
    if (nodeList_ == nullptr) return;
    const bool dual = stereoMode_ != StereoMode::Linked;
    const bool midSide = stereoMode_ == StereoMode::MidSide;
    nodeList_->setThreshold(paramCache_[kParamThreshold]);
    nodeList_->setCurve(curve_);
    nodeList_->setTitle(!dual ? "Nodes" : (midSide ? "Nodes -- Mid curve" : "Nodes -- Left curve"));
    if (nodeListB_ == nullptr) return;
    // While B isn't independent it mirrors A's own curve (same fallback
    // graphB_ and onGraphCurveChanged(true, ...) already use).
    nodeListB_->setThreshold(isChannelBIndependent() ? paramCache_[kParamThresholdB] : paramCache_[kParamThreshold]);
    nodeListB_->setCurve(isChannelBIndependent() ? curveB_ : curve_);
    nodeListB_->setTitle(midSide ? "Nodes -- Side curve" : "Nodes -- Right curve");
}

void UIContour::refreshToolButton()
{
    using Tool = DGL_NAMESPACE::ContourGraphWidget::EditTool;
    graph_->setTool(tool_);
    graphA_->setTool(tool_);
    graphB_->setTool(tool_);
    switch (tool_)
    {
    case Tool::DrawNode:
        btnTool_->setLabel("Tool: Draw Node");
        btnTool_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
        break;
    case Tool::DrawRising:
        btnTool_->setLabel("Tool: Draw Rising Node");
        btnTool_->setColors(Color(150, 108, 40), Color(180, 132, 54), Color(250, 240, 220));
        break;
    case Tool::DrawFalling:
        btnTool_->setLabel("Tool: Draw Falling Node");
        btnTool_->setColors(Color(72, 82, 170), Color(92, 104, 200), Color(236, 240, 255));
        break;
    case Tool::Edit:
        btnTool_->setLabel("Tool: Edit / Move");
        btnTool_->setColors(Color(70, 150, 90), Color(85, 175, 110), Color(230, 232, 236));
        break;
    }
}

void UIContour::refreshDotsButton()
{
    const bool detector = paramCache_[kParamTrailSource] >= 0.5f;
    btnDots_->setLabel(detector ? "Dots: Detector" : "Dots: In \xE2\x86\x92 Out");
    // Amber while showing the detector level: "not the real audio" is worth noticing at a glance.
    btnDots_->setColors(detector ? Color(150, 108, 40) : Color(58, 62, 70),
                         detector ? Color(180, 132, 54) : Color(78, 84, 96),
                         detector ? Color(250, 240, 220) : Color(230, 232, 236));
}

void UIContour::refreshNodeListButtons()
{
    btnNodeList_->setLabel(nodeListVisible_ ? "Node List: On" : "Node List: Off");
    btnNodeList_->setColors(nodeListVisible_ ? Color(70, 150, 90) : Color(58, 62, 70),
                             nodeListVisible_ ? Color(85, 175, 110) : Color(78, 84, 96),
                             Color(230, 232, 236));
}

void UIContour::refreshStereoModeLabel()
{
    btnStereoMode_->setLabel(stereoModeName(stereoMode_));
    repaint();
}

void UIContour::refreshConditionLabel(SimpleButton* btn, TriggerCondition cond)
{
    btn->setLabel(cond == TriggerCondition::Above ? "Trigger: Above \xE2\x96\xB2" : "Trigger: Below \xE2\x96\xBC");
}

void UIContour::refreshActionLabel(SimpleButton* btn, TriggerAction action)
{
    const bool diagonal = action == TriggerAction::ReleaseToDiagonal;
    btn->setLabel(diagonal ? "On Trigger: Diagonal" : "On Trigger: Curve");
    // Highlighted when it is the non-default (gate-opener) mode so it is obvious at a glance.
    if (diagonal) btn->setColors(Color(52, 118, 84), Color(66, 146, 104), Color(230, 240, 234));
    else          btn->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
}

void UIContour::refreshActionLabels()
{
    refreshActionLabel(btnAction_, action_);
    refreshActionLabel(btnActionA_, action_);
    refreshActionLabel(btnActionB_, isChannelBIndependent() ? actionB_ : action_);
}

void UIContour::toggleTriggerAction(bool isB)
{
    auto flip = [](TriggerAction a) { return a == TriggerAction::ApplyCurve ? TriggerAction::ReleaseToDiagonal : TriggerAction::ApplyCurve; };
    if (isB && isChannelBIndependent()) actionB_ = flip(actionB_);
    else action_ = flip(action_);
    pushCurveParamMirror();
    pushMirrorCommit();
    pushAbState();
    refreshActionLabels();
}

void UIContour::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIContour::onNanoDisplay()
{
    const float w = float(getWidth());
    fillColor(Color(20, 21, 25));
    beginPath();
    rect(0, 0, w, float(getHeight()));
    fill();
}

void UIContour::parameterChanged(uint32_t index, float value)
{
    if (index >= kParamCount) return;
    paramCache_[index] = value;

    switch (index)
    {
    case kParamBypass:
        bypassed_ = value >= 0.5f;
        refreshBypassColors();
        repaint();
        break;
    case kParamThreshold:
    case kParamThresholdB:
        refreshGraphThresholds();
        break;
    case kParamOutputGain:
    case kParamMakeupGainB:
        refreshMakeupMarkers(); // host automation / preset recall / undo
        break;
    case kParamInputGainB:
    case kParamWetDry:
    case kParamWetDryB:
    case kParamGlobalInputGain:
        // Explicit cases so these don't fall into default below and get
        // misread as spectrum-bank packets -- same reasoning as
        // UICompressorStereo::parameterChanged()'s identical cases.
        break;
    case kParamSidechainEnable:
        sidechainEnabled_ = value >= 0.5f;
        refreshSidechainColors();
        refreshStripFields();
        repaint();
        break;
    case kParamListenMode:
        listenMode_ = std::max(0, std::min(2, int(value + 0.5f)));
        refreshListenColors();
        repaint();
        break;
    case kParamMeterInputRmsL:
    case kParamMeterInputPeakL:
        meterIn_->setChannel(0, paramCache_[kParamMeterInputRmsL], paramCache_[kParamMeterInputPeakL]);
        break;
    case kParamMeterInputRmsR:
    case kParamMeterInputPeakR:
        meterIn_->setChannel(1, paramCache_[kParamMeterInputRmsR], paramCache_[kParamMeterInputPeakR]);
        break;
    case kParamMeterOutputRmsL:
    case kParamMeterOutputPeakL:
        meterOut_->setChannel(0, paramCache_[kParamMeterOutputRmsL], paramCache_[kParamMeterOutputPeakL]);
        break;
    case kParamMeterOutputRmsR:
    case kParamMeterOutputPeakR:
        meterOut_->setChannel(1, paramCache_[kParamMeterOutputRmsR], paramCache_[kParamMeterOutputPeakR]);
        break;
    case kParamTrailSource:
        // The dots' x means something else now: drop the old ones, and repaint the button.
        refreshDotsButton();
        graph_->clearTrail(); graphA_->clearTrail(); graphB_->clearTrail();
        for (uint32_t k = 0; k < kTrailBankSize; ++k) lastTrailBankA_[k] = lastTrailBankB_[k] = 1.0e9f; // force a fresh push
        break;
    case kParamMeterTrailA:
    case kParamMeterTrailB:
    case kParamMeterDetectorA:
    case kParamMeterGainReductionA:
    case kParamMeterDetectorB:
    case kParamMeterGainReductionB:
        // Deliberately NOT pushed to the graph here -- see uiIdle()'s own
        // comment on why detector level and gain reduction must never be
        // paired up from whichever one's own notification happened to
        // fire most recently.
        break;
    // The last field of the last slot in B's own tangent-handle Parameter
    // mirror -- always the last curve-data field to arrive within one
    // delivery burst (kParamCurvePointHandleOutBaseB's own bank is the
    // last thing appended before kParamMirrorCommitB itself, see
    // ParameterIndex's own "append, never insert" ordering) -- so by the
    // time this fires, paramCache_[] already holds every mirror field for
    // *both* banks, A's included.
    case kParamCurvePointHandleOutBaseB + kNumMirroredPoints - 1:
    {
        // A host may hand our OWN just-written mirror values back a moment later (queued, so older than what the
        // graph shows by now). Rebuilding the curve from those mid-edit pulled a node being dragged back to a
        // stale position. Real restores (session/preset load) arrive long after any
        // local edit, so echoes within a short window of one are simply ignored.
        if (hasLocalMirrorEdit_ && std::chrono::steady_clock::now() - lastLocalMirrorEdit_ < std::chrono::milliseconds(1500))
            break;
        const int n = std::max(2, std::min(int(kNumMirroredPoints), int(paramCache_[kParamCurvePointCount] + 0.5f)));
        CurvePoint pts[kNumMirroredPoints];
        for (int i = 0; i < n; ++i)
        {
            pts[i].inputDb = paramCache_[kParamCurvePointInBase + i];
            pts[i].outputDb = paramCache_[kParamCurvePointOutBase + i];
            pts[i].direction = sscomp::unpackDirection(paramCache_[kParamCurvePointDirBase + i]);
            pts[i].link = sscomp::unpackLink(paramCache_[kParamCurvePointDirBase + i]);
            pts[i].linkAxis = sscomp::unpackLinkAxis(paramCache_[kParamCurvePointDirBase + i]);
            pts[i].diagonal = sscomp::unpackPointDiagonal(paramCache_[kParamCurvePointDirBase + i]);
            pts[i].handleInputDb = paramCache_[kParamCurvePointHandleInBase + i];
            pts[i].handleOutputDb = paramCache_[kParamCurvePointHandleOutBase + i];
        }
        curve_.loadPoints(pts, n, sscomp::unpackThresholdLink(paramCache_[kParamCurveTriggerCondition]));
        condition_ = sscomp::clampTriggerCondition(sscomp::unpackConditionInt(paramCache_[kParamCurveTriggerCondition]));
        action_ = sscomp::clampTriggerAction(sscomp::unpackTriggerActionInt(paramCache_[kParamCurveTriggerCondition]));

        const int nB = std::max(2, std::min(int(kNumMirroredPoints), int(paramCache_[kParamCurvePointCountB] + 0.5f)));
        CurvePoint ptsB[kNumMirroredPoints];
        for (int i = 0; i < nB; ++i)
        {
            ptsB[i].inputDb = paramCache_[kParamCurvePointInBaseB + i];
            ptsB[i].outputDb = paramCache_[kParamCurvePointOutBaseB + i];
            ptsB[i].direction = sscomp::unpackDirection(paramCache_[kParamCurvePointDirBaseB + i]);
            ptsB[i].link = sscomp::unpackLink(paramCache_[kParamCurvePointDirBaseB + i]);
            ptsB[i].linkAxis = sscomp::unpackLinkAxis(paramCache_[kParamCurvePointDirBaseB + i]);
            ptsB[i].diagonal = sscomp::unpackPointDiagonal(paramCache_[kParamCurvePointDirBaseB + i]);
            ptsB[i].handleInputDb = paramCache_[kParamCurvePointHandleInBaseB + i];
            ptsB[i].handleOutputDb = paramCache_[kParamCurvePointHandleOutBaseB + i];
        }
        curveB_.loadPoints(ptsB, nB, sscomp::unpackThresholdLink(paramCache_[kParamCurveTriggerConditionB]));
        conditionB_ = sscomp::clampTriggerCondition(sscomp::unpackConditionInt(paramCache_[kParamCurveTriggerConditionB]));
        actionB_ = sscomp::clampTriggerAction(sscomp::unpackTriggerActionInt(paramCache_[kParamCurveTriggerConditionB]));

        refreshGraphCurves();
        refreshGraphConditions();
        refreshConditionLabel(btnCondition_, condition_);
        refreshConditionLabel(btnConditionA_, condition_);
        refreshConditionLabel(btnConditionB_, isChannelBIndependent() ? conditionB_ : condition_);
        refreshActionLabels();
        break;
    }
    case kParamScFilterEnable:
        if (scPanelVisible_)
        {
            const bool filterOn = value >= 0.5f;
            btnScFilterEnable_->setColors(filterOn ? Color(70, 150, 90) : Color(58, 62, 70),
                                           filterOn ? Color(85, 175, 110) : Color(78, 84, 96),
                                           Color(230, 232, 236));
            btnScFilterEnable_->setLabel(filterOn ? "Trigger Filter: On" : "Trigger Filter: Off");
            scScope_->setFilterRange(filterOn,
                                      editingScBankR_ == 0 ? paramCache_[kParamScFilterHighpassHz] : paramCache_[kParamScFilterHighpassHzR],
                                      editingScBankR_ == 0 ? paramCache_[kParamScFilterLowpassHz] : paramCache_[kParamScFilterLowpassHzR]);
        }
        break;
    case kParamScFilterHighpassHz:
    case kParamScFilterLowpassHz:
        if (editingScBankR_ == 0)
            scScope_->setFilterRange(paramCache_[kParamScFilterEnable] >= 0.5f, paramCache_[kParamScFilterHighpassHz], paramCache_[kParamScFilterLowpassHz]);
        break;
    case kParamScFilterHighpassHzR:
    case kParamScFilterLowpassHzR:
        if (editingScBankR_ == 1)
            scScope_->setFilterRange(paramCache_[kParamScFilterEnable] >= 0.5f, paramCache_[kParamScFilterHighpassHzR], paramCache_[kParamScFilterLowpassHzR]);
        break;
    case kParamScFilterLinkLR:
        scFilterLinkLR_ = value >= 0.5f;
        refreshScLinkLabel();
        if (btnScEditBank_ != nullptr) btnScEditBank_->setEnabled(!scFilterLinkLR_);
        if (scFilterLinkLR_ && editingScBankR_ != 0)
        {
            editingScBankR_ = 0;
            btnScEditBank_->setLabel("Editing: L");
            refreshScStripFields();
        }
        break;
    case kParamMeterScLevelDbL:
        scScope_->pushLevelDbL(value);
        break;
    case kParamMeterScLevelDbR:
        scScope_->pushLevelDbR(value);
        break;
    case kParamScListenEnable:
        refreshScListenColors();
        break;
    default:
        if (index >= kParamMeterScSpectrumInBase && index < kParamInputGainB)
            scScope_->setSpectrumInPacket(int(index - kParamMeterScSpectrumInBase), value);
        else if (index >= kParamMeterScSpectrumBase && index < kParamScListenEnable)
            scScope_->setSpectrumOutPacket(int(index - kParamMeterScSpectrumBase), value);
        break;
    }
    repaint();
}

// The GR meter's Fast/Slow switch: one setting for every graph in this
// editor, saved with the session as the "grMeterSpeed" State.
void UIContour::setGrMeterSpeed(DGL_NAMESPACE::GrMeterSpeed s, bool save)
{
    graph_->setGrMeterSpeed(s);
    graphA_->setGrMeterSpeed(s);
    graphB_->setGrMeterSpeed(s);
    if (save) setState("grMeterSpeed", DGL_NAMESPACE::grMeterSpeedStateValue(s));
}

void UIContour::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "grMeterSpeed") == 0) { setGrMeterSpeed(DGL_NAMESPACE::grMeterSpeedFromState(value), false); return; }
    if (std::strcmp(key, "stereoMode") == 0)
    {
        if (kContourMono) return; // always Linked
        stereoMode_ = sscomp::clampStereoMode(std::atoi(value));
        refreshStereoModeLabel();
        updateMeterLabelsForMode();
        return;
    }
    if (std::strcmp(key, "linkAB") == 0)
    {
        linkAB_ = std::atoi(value) != 0;
        refreshLinkLabel();
        updateMeterLabelsForMode();
        repaint();
        return;
    }
    if (std::strcmp(key, "presetDir") == 0) { lastPresetDir_ = value; return; }
    if (std::strcmp(key, "abSlotA") == 0) { deserializeSnapshot(value, abSlotA_); return; }
    if (std::strcmp(key, "abSlotB") == 0) { deserializeSnapshot(value, abSlotB_); return; }
    if (std::strcmp(key, "abHasA") == 0) { abHasA_ = std::atoi(value) != 0; refreshAbColors(); return; }
    if (std::strcmp(key, "abHasB") == 0) { abHasB_ = std::atoi(value) != 0; refreshAbColors(); return; }
    if (std::strcmp(key, "abActive") == 0) { abActive_ = std::atoi(value) != 0 ? AbSlot::B : AbSlot::A; refreshAbColors(); return; }
}

ParamStripWidget* UIContour::editingStrip() const
{
    if (strip_->isEditing()) return strip_;
    if (stripA_->isEditing()) return stripA_;
    if (stripB_->isEditing()) return stripB_;
    if (stripGlobal_->isEditing()) return stripGlobal_;
    if (stripGlobalIn_->isEditing()) return stripGlobalIn_;
    if (scStrip_->isEditing()) return scStrip_;
    return nullptr;
}

DGL_NAMESPACE::NodeListWidget* UIContour::editingNodeList() const
{
    if (nodeList_->isEditing()) return nodeList_;
    if (nodeListB_->isEditing()) return nodeListB_;
    return nullptr;
}

bool UIContour::onCharacterInput(const CharacterInputEvent& ev)
{
    if (DGL_NAMESPACE::NodeListWidget* nl = editingNodeList())
    {
        const char c = ev.string[0];
        if (c == '\r' || c == '\n') { nl->commitEdit(); return true; }
        if ((c >= '0' && c <= '9') || c == '.' || c == '-') { nl->appendChar(c); return true; }
        return true;
    }
    ParamStripWidget* editing = editingStrip();
    if (editing == nullptr) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { editing->commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { editing->appendChar(c); return true; }
    return true;
}

bool UIContour::onKeyboard(const KeyboardEvent& ev)
{
    if (ev.press && (ev.mod & DGL_NAMESPACE::kModifierControl) != 0
        && !(ev.mod & DGL_NAMESPACE::kModifierAlt) && !(ev.mod & DGL_NAMESPACE::kModifierSuper))
    {
        const bool shift = (ev.mod & DGL_NAMESPACE::kModifierShift) != 0;
        if (ev.key == 'z' && !shift) { onUndo(); return true; }
        if (ev.key == 'y' || (ev.key == 'z' && shift)) { onRedo(); return true; }
    }

    if (DGL_NAMESPACE::NodeListWidget* nl = editingNodeList(); nl != nullptr && ev.press)
    {
        switch (ev.key)
        {
        case DGL_NAMESPACE::kKeyEnter:
        case DGL_NAMESPACE::kKeyTab:
            nl->commitEdit();
            return true;
        case DGL_NAMESPACE::kKeyEscape:
            nl->cancelEdit();
            return true;
        case DGL_NAMESPACE::kKeyBackspace:
        case DGL_NAMESPACE::kKeyDelete:
            nl->backspace();
            return true;
        default:
            return true;
        }
    }

    ParamStripWidget* editing = editingStrip();

    // Escape closes an open right-click node menu.
    if (ev.press && ev.key == DGL_NAMESPACE::kKeyEscape)
    {
        bool closed = false;
        for (DGL_NAMESPACE::ContourGraphWidget* g : { graph_, graphA_, graphB_ })
            if (g->isVisible() && g->closeMenu()) closed = true;
        if (closed) return true;
    }

    // Delete/Backspace with no field mid-edit removes every selected node.
    // Only the graph(s) currently on screen -- a hidden one can hold a stale
    // selection from another stereo layout.
    if (editing == nullptr && ev.press && (ev.key == DGL_NAMESPACE::kKeyDelete || ev.key == DGL_NAMESPACE::kKeyBackspace))
    {
        bool handled = false;
        for (DGL_NAMESPACE::ContourGraphWidget* g : { graph_, graphA_, graphB_ })
            if (g->isVisible() && g->deleteSelected()) handled = true;
        return handled;
    }

    if (editing == nullptr || !ev.press) return false;
    switch (ev.key)
    {
    case DGL_NAMESPACE::kKeyEnter:
    case DGL_NAMESPACE::kKeyTab:
        editing->commitEdit();
        return true;
    case DGL_NAMESPACE::kKeyEscape:
        editing->cancelEdit();
        return true;
    case DGL_NAMESPACE::kKeyBackspace:
    case DGL_NAMESPACE::kKeyDelete:
        editing->backspace();
        return true;
    default:
        return true;
    }
}

void UIContour::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
}

void UIContour::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".mccp";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    const ContourSnapshot snap = captureSnapshot();
    if (!presetio::save(p, snap.values, "MimoMusic Contour preset")) return;
    // Both curves appended as their own "curve="/"curveB=" lines, same
    // reasoning as the original single-curve save (see the comment this
    // is ported from) -- presetio::load()'s own parser already skips any
    // line whose value half doesn't parse as a number, so both are
    // silently invisible to it, read back explicitly in loadPresetFile()
    // below instead.
    std::ofstream f(p, std::ios::app);
    if (f.good()) f << "curve=" << snap.curve << "\ncurveB=" << snap.curveB << "\n";
    rememberPresetDir(p.c_str());
}

void UIContour::loadPresetFile(const char* path)
{
    presetio::KeyValues values;
    if (!presetio::load(path, values)) return;

    std::string curveStr, curveBStr;
    {
        std::ifstream f(path);
        std::string line;
        while (f.good() && std::getline(f, line))
        {
            if (line.compare(0, 7, "curveB=") == 0) curveBStr = line.substr(7);
            else if (line.compare(0, 6, "curve=") == 0) curveStr = line.substr(6);
        }
    }

    ContourSnapshot snap;
    snap.values = values;
    snap.curve = curveStr.empty() ? sscomp::curveToString(TransferCurve()) : curveStr;
    snap.curveB = curveBStr.empty() ? sscomp::curveToString(TransferCurve()) : curveBStr;
    applySnapshot(snap);
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See UICompressorStereo.cpp for the full design writeup this is ported
// from, and ContourSnapshot's own comment for the one real difference
// (both curves ride alongside, not through, the ordinary scalar map).

ContourSnapshot UIContour::captureSnapshot() const
{
    ContourSnapshot snap;
    for (size_t i = 0; i < kPresetParamCount; ++i)
        snap.values[kPresetKeys[i]] = double(paramCache_[kPresetParams[i]]);
    snap.values["stereoMode"] = double(int(stereoMode_));
    snap.values["triggerCondition"] = condition_ == TriggerCondition::Below ? 1.0 : 0.0;
    snap.values["triggerConditionB"] = conditionB_ == TriggerCondition::Below ? 1.0 : 0.0;
    snap.values["triggerAction"] = action_ == TriggerAction::ReleaseToDiagonal ? 1.0 : 0.0;
    snap.values["triggerActionB"] = actionB_ == TriggerAction::ReleaseToDiagonal ? 1.0 : 0.0;
    snap.values["linkAB"] = linkAB_ ? 1.0 : 0.0;
    snap.curve = sscomp::curveToString(curve_);
    snap.curveB = sscomp::curveToString(curveB_);
    return snap;
}

void UIContour::applySnapshot(const ContourSnapshot& snapshot)
{
    const presetio::KeyValues& values = snapshot.values;
    for (size_t i = 0; i < kPresetParamCount; ++i)
    {
        const auto it = values.find(kPresetKeys[i]);
        if (it == values.end()) continue;
        const uint32_t idx = kPresetParams[i];
        const float v = float(it->second);
        paramCache_[idx] = v;
        setParameterValue(idx, v);
    }

    const auto modeIt = values.find("stereoMode");
    if (modeIt != values.end() && !kContourMono) // a stereo preset's mode is ignored by the mono build
    {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%d", int(modeIt->second));
        setState("stereoMode", buf);
        stereoMode_ = sscomp::clampStereoMode(int(modeIt->second));
        refreshStereoModeLabel();
        updateMeterLabelsForMode();
    }

    const auto condIt = values.find("triggerCondition");
    if (condIt != values.end())
    {
        condition_ = sscomp::clampTriggerCondition(int(condIt->second));
        refreshConditionLabel(btnCondition_, condition_);
        refreshConditionLabel(btnConditionA_, condition_);
    }

    const auto condBIt = values.find("triggerConditionB");
    if (condBIt != values.end())
        conditionB_ = sscomp::clampTriggerCondition(int(condBIt->second));

    // Presets/snapshots from before the open/close switch have no key: they meant "Curve".
    {
        const auto actIt = values.find("triggerAction");
        action_ = sscomp::clampTriggerAction(actIt != values.end() ? int(actIt->second) : 0);
        const auto actBIt = values.find("triggerActionB");
        actionB_ = sscomp::clampTriggerAction(actBIt != values.end() ? int(actBIt->second) : 0);
    }

    const auto linkIt = values.find("linkAB");
    if (linkIt != values.end())
    {
        linkAB_ = linkIt->second != 0.0;
        setState("linkAB", linkAB_ ? "1" : "0");
        refreshLinkLabel();
    }

    // Both curves -- restored unconditionally, same "every snapshot this
    // class ever produces has one" reasoning the original single-curve
    // code already established. Each point's own handle/direction tag
    // rides along inside the curve string itself now (see
    // sscomp::curveFromString()'s own comment), no separate interp state
    // to restore alongside it any more.
    sscomp::curveFromString(snapshot.curve, curve_);
    sscomp::curveFromString(snapshot.curveB, curveB_);
    pushCurveParamMirror();
    pushMirrorCommit();
    pushAbState();
    refreshGraphCurves();

    strip_->repaint();
    stripA_->repaint();
    stripB_->repaint();
    refreshGraphThresholds();
    bypassed_ = paramCache_[kParamBypass] >= 0.5f;
    refreshBypassColors();
    sidechainEnabled_ = paramCache_[kParamSidechainEnable] >= 0.5f;
    refreshSidechainColors();
    scFilterLinkLR_ = paramCache_[kParamScFilterLinkLR] >= 0.5f;
    refreshScLinkLabel();
    if (btnScEditBank_ != nullptr) btnScEditBank_->setEnabled(!scFilterLinkLR_);
    if (scFilterLinkLR_ && editingScBankR_ != 0)
    {
        editingScBankR_ = 0;
        btnScEditBank_->setLabel("Editing: L");
    }
    refreshScStripFields();
    refreshStripFields();
    refreshDualStripFields();
    refreshGraphConditions();
    refreshConditionLabel(btnConditionB_, isChannelBIndependent() ? conditionB_ : condition_);
    repaint();
}

void UIContour::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = captureSnapshot(); abHasB_ = true;
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applySnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIContour::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = captureSnapshot(); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applySnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIContour::onAbCopy()
{
    const ContourSnapshot snap = captureSnapshot();
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
    pushAbState();
}

void UIContour::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIContour::onUndo()
{
    if (undoStack_.empty()) return;
    redoStack_.push_back(lastCommittedSnapshot_);
    if (redoStack_.size() > kMaxUndoDepth) redoStack_.erase(redoStack_.begin());
    const ContourSnapshot prev = undoStack_.back();
    undoStack_.pop_back();
    applyingSnapshot_ = true;
    applySnapshot(prev);
    applyingSnapshot_ = false;
    lastCommittedSnapshot_ = prev;
    lastSeenSnapshot_ = prev;
    idleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIContour::onRedo()
{
    if (redoStack_.empty()) return;
    undoStack_.push_back(lastCommittedSnapshot_);
    if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
    const ContourSnapshot next = redoStack_.back();
    redoStack_.pop_back();
    applyingSnapshot_ = true;
    applySnapshot(next);
    applyingSnapshot_ = false;
    lastCommittedSnapshot_ = next;
    lastSeenSnapshot_ = next;
    idleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIContour::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIContour::uiIdle()
{
    // Tell the DSP the trigger scope is on screen -- see kParamUiHeartbeat. Only while the panel is visible, so a
    // closed window (no uiIdle at all) or a hidden panel stops the spectrum FFTs. A counter: it must change every
    // tick to be delivered, and a 0/1 toggle can cancel out inside one long block.
    if (scPanelVisible_)
    {
        uiHeartbeat_ = uiHeartbeat_ >= 999999.0f ? 1.0f : uiHeartbeat_ + 1.0f;
        setParameterValue(kParamUiHeartbeat, uiHeartbeat_);
    }

    // Detector level and gain reduction are two separate DPF output
    // Parameters, even though PluginContour::run() always writes both
    // from the exact same instant every audio block -- see this
    // function's own original comment (still accurate, unchanged) for
    // why they're read together here, once per tick, rather than paired
    // up from inside parameterChanged().
    //
    // Update: reading them together here was still not enough -- the host
    // reads the two ports one after the other, so a pair could straddle two
    // audio blocks (dots scattered off the curve at 0ms attack/release). The
    // pair now arrives as ONE packed Parameter per voice
    // (kParamMeterTrailA/B, see sscomp::packTrailPair()) and is unpacked here.
    double levelA, grA, levelB, grB;
    sscomp::unpackTrailPair(paramCache_[kParamMeterTrailA], levelA, grA);
    sscomp::unpackTrailPair(paramCache_[kParamMeterTrailB], levelB, grB);
    // The marker uses the latest sample; the GR meter (column, number, hold,
    // history) the plugin's held GR readout, so a short cut between two
    // ticks still shows (see sscomp::GrPeakMeter)...
    const float meterGrA = paramCache_[kParamMeterGainReductionA];
    const float meterGrB = paramCache_[kParamMeterGainReductionB];
    graph_->setLiveLevels(float(levelA), float(grA), meterGrA, false);
    graphA_->setLiveLevels(float(levelA), float(grA), meterGrA, false);
    graphB_->setLiveLevels(float(levelB), float(grB), meterGrB, false);
    // ...and the trail gets the whole bank of pairs the plugin published for the
    // last block (kTrailBankSize points instead of one) -- unless nothing new
    // arrived since the last tick (a redraw faster than the audio blocks would
    // otherwise pile up duplicates). Each pair also says whether the trigger
    // condition was met at that sample (sign bit): those dots are drawn green, and
    // the fraction of them is what drives the threshold's yellow->green.
    {
        float lv[kTrailBankSize], gr[kTrailBankSize];
        unsigned char tr[kTrailBankSize];
        bool changedA = false, changedB = false;
        for (uint32_t k = 0; k < kTrailBankSize; ++k)
        {
            if (paramCache_[kParamMeterTrailBankA + k] != lastTrailBankA_[k]) changedA = true;
            if (paramCache_[kParamMeterTrailBankB + k] != lastTrailBankB_[k]) changedB = true;
        }
        auto push = [&](int voice, uint32_t base, float& fracOut, ContourGraphWidget* a, ContourGraphWidget* b) {
            int nTrig = 0;
            for (uint32_t k = 0; k < kTrailBankSize; ++k)
            {
                double L, G; bool t;
                sscomp::unpackTrailPair(paramCache_[base + k], L, G, t);
                lv[k] = float(L); gr[k] = float(G); tr[k] = t ? 1 : 0;
                nTrig += t ? 1 : 0;
            }
            fracOut = float(nTrig) / float(kTrailBankSize);
            a->pushTrailBatch(voice, lv, gr, tr, int(kTrailBankSize));
            if (b) b->pushTrailBatch(voice, lv, gr, tr, int(kTrailBankSize));
        };
        if (changedA) { push(0, kParamMeterTrailBankA, trigFracA_, graph_, graphA_); for (uint32_t k = 0; k < kTrailBankSize; ++k) lastTrailBankA_[k] = paramCache_[kParamMeterTrailBankA + k]; }
        // graphB_ (the right/Side plot of Dual-Mono / Mid-Side) draws ITS channel as its own primary voice (slot 0), exactly like graphA_
        // does for A -- pushing B's bank into slot 1 would fill a slot that widget never draws.
        if (changedB) { push(0, kParamMeterTrailBankB, trigFracB_, graphB_, nullptr); for (uint32_t k = 0; k < kTrailBankSize; ++k) lastTrailBankB_[k] = paramCache_[kParamMeterTrailBankB + k]; }
    }
    // Trigger activity -> the threshold's colour, smoothed with the Attack (rising) and Release
    // (falling) times, so a long release shows as a slow fade back to yellow. At 0 ms it just
    // follows the trigger. (The trigger fraction is the newest block's share of triggered dots,
    // so a trigger that flickers at audio rate reads as an in-between colour.)
    {
        const auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - lastTrigTick_).count();
        lastTrigTick_ = now;
        if (!(dt > 0.0f) || dt > 0.25f) dt = 0.03f;
        auto stepEnv = [dt](float& env, float target, float attackMs, float releaseMs) {
            const float tauMs = target > env ? attackMs : releaseMs;
            if (tauMs <= 0.0f) env = target;
            else env += (target - env) * (1.0f - std::exp(-dt * 1000.0f / tauMs));
        };
        const bool bInd = isChannelBIndependent();
        stepEnv(trigEnvA_, trigFracA_, paramCache_[kParamAttack], paramCache_[kParamRelease]);
        stepEnv(trigEnvB_, trigFracB_, bInd ? paramCache_[kParamAttackB] : paramCache_[kParamAttack],
                bInd ? paramCache_[kParamReleaseB] : paramCache_[kParamRelease]);
        graph_->setTriggerActivity(trigEnvA_);
        graphA_->setTriggerActivity(trigEnvA_);
        graphB_->setTriggerActivity(trigEnvB_); // B's own trigger, whether or not its settings are linked to A's
    }

    scScope_->tickLevelHistory();
    tooltip_->tick();

    if (applyingSnapshot_) return;

    const ContourSnapshot now = captureSnapshot();
    if (now != lastSeenSnapshot_)
    {
        lastSeenSnapshot_ = now;
        idleTicksSinceChange_ = 0;
        return;
    }
    if (idleTicksSinceChange_ >= kUndoDebounceTicks) return;
    if (++idleTicksSinceChange_ < kUndoDebounceTicks) return;

    if (undoRedoInitialized_ && now != lastCommittedSnapshot_)
    {
        undoStack_.push_back(lastCommittedSnapshot_);
        if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
        redoStack_.clear();
        refreshUndoRedoColors();
    }
    lastCommittedSnapshot_ = now;
    undoRedoInitialized_ = true;
}

void UIContour::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UIContour::refreshScPanelVisibility()
{
    refreshDualLayoutVisibility();
    btnScFilterEnable_->setVisible(scPanelVisible_);
    btnScListen_->setVisible(scPanelVisible_);
    // The dots always show the real audio now (the curve maps the audio itself), so the old In->Out / Detector switch is gone.
    btnDots_->setVisible(false);
    btnScLinkLR_->setVisible(scPanelVisible_ && !kContourMono); // one channel: nothing to link
    btnScEditBank_->setVisible(scPanelVisible_ && !kContourMono);
    scStrip_->setVisible(scPanelVisible_);
    scScope_->setVisible(scPanelVisible_);
    btnScClose_->setVisible(scPanelVisible_);
    btnSidechainOptions_->setColors(scPanelVisible_ ? Color(70, 150, 90) : Color(58, 62, 70),
                                     scPanelVisible_ ? Color(85, 175, 110) : Color(78, 84, 96),
                                     Color(230, 232, 236));
    if (scPanelVisible_)
    {
        const bool filterOn = paramCache_[kParamScFilterEnable] >= 0.5f;
        btnScFilterEnable_->setColors(filterOn ? Color(70, 150, 90) : Color(58, 62, 70),
                                       filterOn ? Color(85, 175, 110) : Color(78, 84, 96),
                                       Color(230, 232, 236));
        btnScFilterEnable_->setLabel(filterOn ? "Trigger Filter: On" : "Trigger Filter: Off");
        scScope_->setFilterRange(filterOn,
                                  editingScBankR_ == 0 ? paramCache_[kParamScFilterHighpassHz] : paramCache_[kParamScFilterHighpassHzR],
                                  editingScBankR_ == 0 ? paramCache_[kParamScFilterLowpassHz] : paramCache_[kParamScFilterLowpassHzR]);
        refreshScListenColors();
    }
    else if (paramCache_[kParamScListenEnable] >= 0.5f)
    {
        setParameterValue(kParamScListenEnable, 0.0f);
        paramCache_[kParamScListenEnable] = 0.0f;
        refreshScListenColors();
    }
    layout();
    repaint();
}

void UIContour::refreshScListenColors()
{
    const bool on = paramCache_[kParamScListenEnable] >= 0.5f;
    btnScListen_->setColors(on ? Color(70, 150, 90) : Color(58, 62, 70),
                             on ? Color(85, 175, 110) : Color(78, 84, 96),
                             Color(230, 232, 236));
    btnScListen_->setLabel(on ? "Listen: On" : "Listen: Off");
}

void UIContour::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIContour(); }

END_NAMESPACE_DISTRHO
