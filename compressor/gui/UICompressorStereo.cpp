// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UICompressorStereo.hpp"

#include "PresetIO.hpp"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::CompressorGraphWidget;
using DGL_NAMESPACE::LevelMeterWidget;
using DGL_NAMESPACE::ParamFieldSpec;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;
using sscomp::StereoMode;

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

// Every automatable knob this preset format round-trips -- "stereoMode",
// "ratioDirection", "ratioDirectionB" and "linkAB" (States, not
// Parameters -- see PluginCompressorStereo.hpp) are handled separately in
// save/loadPresetFile() below since they don't fit this list. The B-bank
// entries (see kParamThresholdB's own comment) round-trip unconditionally,
// same as A's own -- whether they currently do anything live depends on
// stereo mode/Link, not on whether they're worth remembering.
constexpr uint32_t kPresetParams[] = {
    kParamInputGain, kParamThreshold, kParamRatio, kParamAttack, kParamRelease,
    kParamKnee, kParamOutputGain, kParamBypass,
    kParamSidechainEnable, kParamSidechainGain,
    kParamThresholdB, kParamRatioB, kParamAttackB, kParamReleaseB, kParamKneeB,
    kParamMakeupGainB, kParamGlobalOutputGain,
    kParamScFilterEnable, kParamScFilterHighpassHz, kParamScFilterLowpassHz,
    kParamScFilterLinkLR, kParamScFilterHighpassHzR, kParamScFilterLowpassHzR,
    kParamInputGainB, kParamWetDry, kParamWetDryB, kParamGlobalInputGain
};
constexpr const char* kPresetKeys[] = {
    "inputGainDb", "thresholdDb", "ratio", "attackMs", "releaseMs",
    "kneeDb", "outputGainDb", "bypass",
    "sidechainEnable", "sidechainGainDb",
    "thresholdDbB", "ratioB", "attackMsB", "releaseMsB", "kneeDbB",
    "makeupGainDbB", "globalOutputGainDb",
    "scFilterEnable", "scFilterHighpassHz", "scFilterLowpassHz",
    "scFilterLinkLR", "scFilterHighpassHzR", "scFilterLowpassHzR",
    "inputGainDbB", "wetDry", "wetDryB", "globalInputGainDb"
};
constexpr size_t kPresetParamCount = sizeof(kPresetParams) / sizeof(kPresetParams[0]);
static_assert(sizeof(kPresetKeys) / sizeof(kPresetKeys[0]) == kPresetParamCount, "keys/params length mismatch");
}

UICompressorStereo::UICompressorStereo()
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
    setGeometryConstraints(DISTRHO_UI_DEFAULT_WIDTH / 2, DISTRHO_UI_DEFAULT_HEIGHT / 2, true, true, false);
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    paramCache_[kParamInputGain] = 0.0f;
    paramCache_[kParamThreshold] = -18.0f;
    paramCache_[kParamRatio] = 4.0f;
    paramCache_[kParamAttack] = 10.0f;
    paramCache_[kParamRelease] = 150.0f;
    paramCache_[kParamKnee] = 6.0f;
    paramCache_[kParamOutputGain] = 0.0f;
    // Unused -- see kParamCompensation's own comment (PluginCompressorStereo.cpp)
    // for why the slots stay reserved. Only initialized here so nothing
    // ever reads garbage from them before the host's own initial value
    // burst arrives.
    paramCache_[kParamCompensation] = 0.0f;
    paramCache_[kParamCompensationMeasure] = 0.0f;
    paramCache_[kParamBypass] = 0.0f;
    paramCache_[kParamSidechainEnable] = 0.0f;
    paramCache_[kParamSidechainGain] = 0.0f;
    paramCache_[kParamListenMode] = 0.0f;
    paramCache_[kParamThresholdB] = -18.0f;
    paramCache_[kParamRatioB] = 4.0f;
    paramCache_[kParamAttackB] = 10.0f;
    paramCache_[kParamReleaseB] = 150.0f;
    paramCache_[kParamKneeB] = 6.0f;
    paramCache_[kParamMakeupGainB] = 0.0f;
    paramCache_[kParamGlobalOutputGain] = 0.0f;
    paramCache_[kParamScFilterEnable] = 0.0f;
    paramCache_[kParamScFilterHighpassHz] = 100.0f;
    paramCache_[kParamScFilterLowpassHz] = 8000.0f;
    paramCache_[kParamMeterInputRmsL] = paramCache_[kParamMeterInputPeakL] = -120.0f;
    paramCache_[kParamMeterInputRmsR] = paramCache_[kParamMeterInputPeakR] = -120.0f;
    paramCache_[kParamMeterOutputRmsL] = paramCache_[kParamMeterOutputPeakL] = -120.0f;
    paramCache_[kParamMeterOutputRmsR] = paramCache_[kParamMeterOutputPeakR] = -120.0f;
    paramCache_[kParamMeterDetectorA] = paramCache_[kParamMeterDetectorB] = -120.0f;
    paramCache_[kParamMeterGainReductionA] = paramCache_[kParamMeterGainReductionB] = 0.0f;
    paramCache_[kParamMeterCompGainDb] = 0.0f; // unused, see kParamCompensation's own comment
    paramCache_[kParamMeterCompState] = 0.0f;
    paramCache_[kParamMeterScLevelDb] = -120.0f;
    for (uint32_t i = kParamMeterScSpectrumBase; i < kParamCount; ++i) paramCache_[i] = 0.0f;
    // These all fall inside the zeroing loop's own index range above (see
    // PluginCompressorStereo.hpp's "append, never insert" ordering) --
    // set their real defaults after it, not before, so the loop doesn't
    // clobber them back to 0.
    paramCache_[kParamScFilterLinkLR] = 1.0f;
    paramCache_[kParamScFilterHighpassHzR] = 100.0f;
    paramCache_[kParamScFilterLowpassHzR] = 8000.0f;
    paramCache_[kParamMeterScLevelDbL] = paramCache_[kParamMeterScLevelDbR] = -120.0f;
    paramCache_[kParamInputGainB] = 0.0f;
    paramCache_[kParamWetDry] = paramCache_[kParamWetDryB] = 100.0f;
    paramCache_[kParamGlobalInputGain] = 0.0f;

    meterIn_ = new LevelMeterWidget(this, "IN", 2);
    meterOut_ = new LevelMeterWidget(this, "OUT", 2);

    // Linked-mode-only widget set -- Linked mode keeps its original look.
    // See refreshDualLayoutVisibility()'s
    // own comment.
    graph_ = new CompressorGraphWidget(this);
    graph_->setThresholdRange(-60.0f, 0.0f); // matches the Threshold field's own range below
    graph_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    graph_->setOnRatioChanged([this](float v) { onGraphParamChanged(kParamRatio, v); });
    graph_->setOnKneeChanged([this](float v) { onGraphParamChanged(kParamKnee, v); });

    // Dual-Mono/Mid-Side widget set -- both compressors shown side by side
    // (doubled GUI objects instead of an editing switch). graphA_/graphB_
    // constructed here (right after graph_,
    // before the first updateMeterLabelsForMode() call below) since that
    // first call's own early-bailout path still reaches refreshMakeupMarkers(),
    // which now touches all three graphs.
    graphA_ = new CompressorGraphWidget(this);
    graphA_->setThresholdRange(-60.0f, 0.0f);
    graphA_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    graphA_->setOnRatioChanged([this](float v) { onGraphParamChanged(kParamRatio, v); });
    graphA_->setOnKneeChanged([this](float v) { onGraphParamChanged(kParamKnee, v); });

    graphB_ = new CompressorGraphWidget(this);
    graphB_->setThresholdRange(-60.0f, 0.0f);
    // bParam() falls back to A's own Parameter whenever B isn't
    // independent -- dragging graphB_ while linked therefore edits the
    // exact same Parameter graphA_ does, a genuine live link rather than
    // a one-time copy. See bParam()'s own declaration comment.
    graphB_->setOnThresholdChanged([this](float v) { onGraphParamChanged(bParam(true, kParamThreshold, kParamThresholdB), v); });
    graphB_->setOnRatioChanged([this](float v) { onGraphParamChanged(bParam(true, kParamRatio, kParamRatioB), v); });
    graphB_->setOnKneeChanged([this](float v) { onGraphParamChanged(bParam(true, kParamKnee, kParamKneeB), v); });

    refreshGraphCurves();
    updateMeterLabelsForMode();

    strip_ = new ParamStripWidget(this);
    refreshStripFields();
    strip_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    strip_->setSetter([this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
        if (idx == kParamThreshold || idx == kParamRatio || idx == kParamKnee)
            refreshGraphCurves();
        else if (idx == kParamOutputGain)
            refreshMakeupMarkers(); // immediate feedback while dragging, same reasoning as refreshGraphCurves() above
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
    // M/S -- Dual-Mono/Mid-Side only, see
    // kParamGlobalInputGain's own comment (PluginCompressorStereo.hpp).
    stripGlobalIn_ = new ParamStripWidget(this);
    stripGlobalIn_->setFields({ { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamGlobalInputGain,
        "Global Input Gain: a shared trim applied to the stereo signal before it's split (Dual-Mono) or decoded (Mid/Side)." } });
    stripGlobalIn_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    stripGlobalIn_->setSetter([this](uint32_t idx, float v) { paramCache_[idx] = v; setParameterValue(idx, v); });

    refreshDualStripFields();
    auto dualStripSetter = [this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
        if (idx == kParamThreshold || idx == kParamRatio || idx == kParamKnee ||
            idx == kParamThresholdB || idx == kParamRatioB || idx == kParamKneeB)
            refreshGraphCurves();
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

    // External sidechain/key input switch -- see kParamSidechainEnable's
    // own comment in PluginCompressorStereo.hpp.
    btnSidechain_ = new SimpleButton(this, "");
    btnSidechain_->setCallback([this] { setParameterValue(kParamSidechainEnable, sidechainEnabled_ ? 0.0f : 1.0f); });
    refreshSidechainColors();

    btnStereoMode_ = new SimpleButton(this, stereoModeName(stereoMode_));
    btnStereoMode_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnStereoMode_->setCallback([this] {
        const int next = (int(stereoMode_) + 1) % 3;
        const bool enteringMidSide = sscomp::clampStereoMode(next) == StereoMode::MidSide && stereoMode_ != StereoMode::MidSide;
        const bool leavingLinked = stereoMode_ == StereoMode::Linked && sscomp::clampStereoMode(next) != StereoMode::Linked;
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%d", next);
        setState("stereoMode", buf); // DSP-relevant discrete choice, not a Parameter -- see PluginCompressorStereo.hpp
        stereoMode_ = sscomp::clampStereoMode(next);
        // "start linked, split on edit" -- see syncBFromA()'s own
        // declaration comment. Before updateMeterLabelsForMode() below,
        // which reads the now-updated stereoMode_ to decide dual-vs-Linked
        // layout and to refresh the dual strips against B's still-stale
        // values otherwise.
        if (enteringMidSide) syncBFromA();
        // Global Input Gain starts at 0dB on entering this mode -- reset
        // every time Dual-Mono/Mid-Side
        // newly becomes reachable from Linked, not just once at
        // construction, so a value left over from some earlier session
        // never silently carries into a fresh one.
        if (leavingLinked)
        {
            paramCache_[kParamGlobalInputGain] = 0.0f;
            setParameterValue(kParamGlobalInputGain, 0.0f);
            stripGlobalIn_->repaint();
        }
        refreshStereoModeLabel();
        updateMeterLabelsForMode();
    });

    // Link L/R parameters (Dual-Mono), or keep them separate -- see
    // kStateLinkAB's own comment (PluginCompressorStereo.hpp).
    // updateMeterLabelsForMode() enables this only in Dual-Mono mode. Drives
    // the live two-way link described in bParam()'s own comment.
    btnLink_ = new SimpleButton(this, "");
    btnLink_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnLink_->setCallback([this] {
        const bool wasLinked = linkAB_;
        linkAB_ = !linkAB_;
        setState("linkAB", linkAB_ ? "1" : "0");
        // Same "start linked, split on edit" as entering Mid/Side above --
        // unlinking starts B equal to A rather than whatever it was last
        // left holding.
        if (wasLinked && !linkAB_) syncBFromA();
        refreshLinkLabel();
        updateMeterLabelsForMode(); // stripB_'s/graphB_'s own field targets depend on linkAB_ too
    });
    refreshLinkLabel();

    // Listen in M/S mode, to hear only Mid or only Side while setting up
    // the compressor -- see kParamListenMode's own
    // comment (PluginCompressorStereo.hpp). Cycles Off -> Mid -> Side ->
    // Off; updateMeterLabelsForMode() (called right above, and whenever
    // Stereo Mode changes) keeps this enabled only while actually in
    // Mid/Side mode. A single shared toggle button even in the dual
    // layout, see its own declaration comment.
    btnListen_ = new SimpleButton(this, "Listen: Off");
    btnListen_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnListen_->setCallback([this] {
        listenMode_ = (listenMode_ + 1) % 3;
        setParameterValue(kParamListenMode, float(listenMode_));
        refreshListenColors();
    });
    refreshListenColors();

    // Linked mode only now (always drives A's own ratioUpMode_/
    // "ratioDirection" unconditionally -- Linked has no B bank at all);
    // greyed out, not hidden, while Dual-Mono/Mid-Side are active, same
    // enabled-not-visible convention every other mode-gated Row 2 button
    // in this file already uses. See btnRatioDirectionA_/
    // btnRatioDirectionB_ for the dual layout's own pair.
    btnRatioDirection_ = new SimpleButton(this, "");
    btnRatioDirection_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRatioDirection_->setCallback([this] {
        ratioUpMode_ = !ratioUpMode_;
        setState("ratioDirection", ratioUpMode_ ? "1" : "0");
        refreshGraphCurves();
        refreshRatioDirectionLabel(btnRatioDirection_, ratioUpMode_);
    });
    refreshRatioDirectionLabel(btnRatioDirection_, ratioUpMode_);

    // Dual-Mono/Mid-Side's own pair, one per displayed compressor --
    // Duplicated GUI objects (one per compressor) instead of an editing
    // switch -- applied to Ratio Direction too, not just Threshold/Ratio/
    // Attack/Release/Knee. btnRatioDirectionB_ falls back to editing A's
    // own ratioUpMode_/"ratioDirection" while linked, same bParam()-style
    // live-link every other B-bank control in the dual layout follows.
    btnRatioDirectionA_ = new SimpleButton(this, "");
    btnRatioDirectionA_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRatioDirectionA_->setCallback([this] {
        ratioUpMode_ = !ratioUpMode_;
        setState("ratioDirection", ratioUpMode_ ? "1" : "0");
        refreshGraphCurves();
        refreshRatioDirectionLabel(btnRatioDirectionA_, ratioUpMode_);
        if (!isChannelBIndependent()) refreshRatioDirectionLabel(btnRatioDirectionB_, ratioUpMode_);
    });
    refreshRatioDirectionLabel(btnRatioDirectionA_, ratioUpMode_);

    btnRatioDirectionB_ = new SimpleButton(this, "");
    btnRatioDirectionB_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRatioDirectionB_->setCallback([this] {
        if (isChannelBIndependent())
        {
            ratioUpModeB_ = !ratioUpModeB_;
            setState("ratioDirectionB", ratioUpModeB_ ? "1" : "0");
        }
        else
        {
            ratioUpMode_ = !ratioUpMode_; // linked: editing B's own button edits A's actual state
            setState("ratioDirection", ratioUpMode_ ? "1" : "0");
            refreshRatioDirectionLabel(btnRatioDirectionA_, ratioUpMode_);
        }
        refreshGraphCurves();
        refreshRatioDirectionLabel(btnRatioDirectionB_, isChannelBIndependent() ? ratioUpModeB_ : ratioUpMode_);
    });
    refreshRatioDirectionLabel(btnRatioDirectionB_, isChannelBIndependent() ? ratioUpModeB_ : ratioUpMode_);

    // The sidechain options button -- see refreshScPanelVisibility()'s own
    // comment for why this opens an in-window overlay panel, not a second
    // Window.
    btnSidechainOptions_ = new SimpleButton(this, "Sidechain Options...");
    btnSidechainOptions_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnSidechainOptions_->setCallback([this] { scPanelVisible_ = !scPanelVisible_; refreshScPanelVisibility(); });

    // The panel's own widgets -- built once, up front, like everything
    // else in this constructor; refreshScPanelVisibility() (called at the
    // very end of this constructor, after layout()) hides them all until
    // the button above is actually clicked.
    btnScFilterEnable_ = new SimpleButton(this, "");
    btnScFilterEnable_->setCallback([this] {
        const float next = paramCache_[kParamScFilterEnable] >= 0.5f ? 0.0f : 1.0f;
        setParameterValue(kParamScFilterEnable, next);
    });

    // Sidechain Listen: pass only the (filtered) sidechain signal, to hear
    // the frequencies the detector is actually reacting to.
    btnScListen_ = new SimpleButton(this, "Listen: Off");
    btnScListen_->setCallback([this] {
        const float next = paramCache_[kParamScListenEnable] >= 0.5f ? 0.0f : 1.0f;
        setParameterValue(kParamScListenEnable, next);
    });

    // A mode where left and right are filtered differently -- mirrors
    // btnLink_'s own Link
    // button, one level down inside the sidechain panel: Link
    // gates whether R has its own corners at all (kParamScFilterLinkLR),
    // Edit Bank picks which side scStrip_/scScope_ currently show/edit
    // while unlinked.
    btnScLinkLR_ = new SimpleButton(this, "");
    btnScLinkLR_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnScLinkLR_->setCallback([this] {
        const bool wasLinked = scFilterLinkLR_;
        scFilterLinkLR_ = !scFilterLinkLR_;
        setParameterValue(kParamScFilterLinkLR, scFilterLinkLR_ ? 1.0f : 0.0f);
        // "start linked, split on edit" -- same as syncBFromA() above:
        // unlinking starts R equal to L rather than whatever it was last
        // left holding.
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
    btnScEditBank_->setEnabled(false); // enabled only while scFilterLinkLR_ is off
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
    // Corner frequencies draggable on the graph, like the Multiband
    // Compressor's -- same "widget fires a callback, UI owns the actual
    // Parameter round trip" pattern as scStrip_'s own setter just above;
    // scStrip_->repaint() keeps its own Highpass/Lowpass numeric fields
    // live while dragging on the graph instead. Bank-aware, same as
    // scStrip_ itself -- dragging always edits whichever side
    // (editingScBankR_) is currently selected.
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
        opts.title = "Load Compressor Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "preset.mcsp";
        opts.title = "Save Compressor Preset";
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

    // Hover descriptions -- constructed last, after every other widget
    // above, so it always
    // paints on top of all of them (see TooltipOverlay's own file
    // comment for why that ordering matters).
    tooltip_ = new DGL_NAMESPACE::TooltipOverlay(this);
    tooltip_->addTarget(btnLoad_, "Load a saved compressor preset (.mcsp) from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a compressor preset (.mcsp).");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed, all metering/detection still runs.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    tooltip_->addTarget(btnStereoMode_, "Cycle Stereo Mode: Linked (one shared compressor), Dual Mono (independent L/R), Mid/Side (independent Mid/Side).");
    tooltip_->addTarget(btnLink_, "Dual-Mono only: link L's and R's own settings together (edit either, both follow); off lets them diverge.");
    tooltip_->addTarget(btnListen_, "Mid/Side only: audition Mid alone, then Side alone, then back to the normal mix.");
    tooltip_->addTarget(btnRatioDirection_, "Swap which side of the threshold Ratio compresses: Down (loud material) or Up (quiet material).");
    tooltip_->addTarget(btnRatioDirectionA_, "Same as Ratio Direction, for the left/Mid compressor shown on the left.");
    tooltip_->addTarget(btnRatioDirectionB_, "Same as Ratio Direction, for the right/Side compressor shown on the right (edits the left one too while Link is on).");
    tooltip_->addTarget(btnSidechain_, "Toggle external sidechain: detect from the Sidechain input instead of the main signal.");
    tooltip_->addTarget(btnSidechainOptions_, "Open the sidechain panel: gain, a Highpass/Lowpass filter and a live scope for the signal driving the compressor (the main signal, or the external Sidechain input when selected).");
    tooltip_->addTarget(btnScFilterEnable_, "Enable the sidechain's own Highpass/Lowpass band filter, so the compressor only triggers from that band.");
    tooltip_->addTarget(btnScListen_, "Audition the sidechain signal itself (filtered if the filter above is on) in place of the main output.");
    tooltip_->addTarget(btnScLinkLR_, "Link the sidechain filter's Left and Right corners together; off lets them diverge.");
    tooltip_->addTarget(btnScEditBank_, "Switch which side (L or R) the Highpass/Lowpass fields and drag handles below are editing (unlinked only).");
    tooltip_->addTarget(btnScClose_, "Close the sidechain panel and return to the normal view.");
    // Parameter-field hover descriptions -- one addFieldTarget() per
    // ParamStripWidget, delegating to its own
    // fieldTooltipAt() so the text shown always matches whichever field
    // is actually under the pointer right now.
    tooltip_->addFieldTarget(strip_, [this](double x, double y) { return strip_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripA_, [this](double x, double y) { return stripA_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripB_, [this](double x, double y) { return stripB_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripGlobal_, [this](double x, double y) { return stripGlobal_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(stripGlobalIn_, [this](double x, double y) { return stripGlobalIn_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(scStrip_, [this](double x, double y) { return scStrip_->fieldTooltipAt(x, y); });

    layout();
    refreshScPanelVisibility(); // starts hidden -- see that method's own comment
}

void UICompressorStereo::updateMeterLabelsForMode()
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
    // graphA_'s/graphB_'s own primary GR-column label -- reused from the
    // secondary-column API (setSecondaryLabels() sets *both* labelA_/
    // labelB_ together, see CompressorGraphWidget.hpp's own comment) purely
    // for that side effect: neither graph ever shows a secondary column of
    // its own any more (each is now a full standalone compressor display,
    // not one half of a shared one -- see refreshMakeupMarkers()'s own
    // comment), so only the primary (labelA_) half of each call actually
    // matters. graph_ (Linked mode) never needed either label at all -- it
    // was always single-column even before this change. Both already
    // exist by this function's own first call -- see the constructor's own
    // comment on graphA_/graphB_'s construction order.
    graphA_->setSecondaryLabels(midSide ? "GR Mid" : "GR L", "");
    graphB_->setSecondaryLabels(midSide ? "GR Side" : "GR R", "");

    // btnListen_ doesn't exist yet the first time this runs (called once
    // during construction before any button is created) -- guard rather
    // than reorder construction, simplest fix. strip_/stripA_/stripB_
    // don't exist yet either at that first call (their own explicit
    // post-construction refreshStripFields()/refreshDualStripFields()/
    // refreshMakeupMarkers() calls in the constructor cover them instead).
    if (btnListen_ == nullptr) { refreshMakeupMarkers(); return; }
    btnListen_->setEnabled(midSide);
    // Leaving Mid/Side with Listen still engaged would silently leave it
    // armed (inert but unreset) until the user switches back -- reset it
    // to Off instead, so re-entering Mid/Side later never surprises with
    // audio that's still isolated to Mid or Side from a previous setup
    // session.
    if (!midSide && listenMode_ != 0)
    {
        listenMode_ = 0;
        setParameterValue(kParamListenMode, 0.0f);
        refreshListenColors();
    }

    // Link only ever means anything in Dual-Mono.
    btnLink_->setEnabled(stereoMode_ == StereoMode::DualMono);
    // Linked-mode-only button -- see its own declaration comment.
    btnRatioDirection_->setEnabled(stereoMode_ == StereoMode::Linked);

    // Both compressors side by side -- shows/hides the
    // Linked-mode vs. dual-layout widget sets for the *new* stereoMode_.
    refreshDualLayoutVisibility();

    // Which Parameter each of stripB_'s/graphB_'s fields actually target
    // depends directly on isChannelBIndependent() (so the two channels
    // can be amplified individually) -- so this has to stay in
    // sync on every mode/Link change, not just the moment it happens to
    // flip.
    refreshStripFields();
    refreshDualStripFields();
    refreshGraphCurves();
    refreshRatioDirectionLabel(btnRatioDirectionB_, isChannelBIndependent() ? ratioUpModeB_ : ratioUpMode_);
    refreshMakeupMarkers();
}

void UICompressorStereo::layout()
{
    // Row 1: every system-wide/whole-instance control -- Load/Save/
    // Bypass, then A/B compare/Undo/Redo -- flush against the window's
    // own top edge, right-aligned (title text fills the remaining space
    // on the left), placed right-to-left in reverse workflow order so
    // the on-screen reading order is left to right: Load, Save, Bypass,
    // A, B, Copy, Undo, Redo. Row 2 is this filter's own curve/stereo-
    // shaping controls (Stereo Mode, Ratio Direction, Sidechain),
    // left-aligned right above the graph.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window regardless of anything else below -- see
    // TooltipOverlay's own file comment for why it needs the full window
    // available for its own hit-testing/painting.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));

    constexpr float pad = 6.0f;
    constexpr float topBarH = 32.0f;
    constexpr float stripH = 72.0f;
    constexpr float meterW = 100.0f; // wide enough for two side-by-side bars
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

    // Row 2: this filter's own curve/stereo-shaping controls, left-aligned
    // right above the graph.
    constexpr float topBar2H = 26.0f;
    const float row2Y = topBarH + gap;
    float bx2 = pad;
    auto place2 = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx2), int(row2Y));
        b->setSize(uint(bw), uint(topBar2H));
        bx2 += bw + gap;
    };
    place2(btnStereoMode_, 150.0f);
    place2(btnLink_, 90.0f); // Dual-Mono only, see updateMeterLabelsForMode()
    place2(btnListen_, 110.0f); // right next to Stereo Mode -- only ever meaningful in Mid/Side, see updateMeterLabelsForMode()
    place2(btnRatioDirection_, 110.0f); // Linked mode only now -- see its own declaration comment
    place2(btnSidechain_, 160.0f);
    place2(btnSidechainOptions_, 150.0f);

    const float mainY = row2Y + topBar2H + pad * 2.0f;
    const float mainH = h - mainY - stripH - pad * 2.0f;

    meterIn_->setAbsolutePos(int(pad), int(mainY));
    meterIn_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    meterOut_->setAbsolutePos(int(w - pad - meterW), int(mainY));
    meterOut_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    const float graphX = pad + meterW + pad;
    const float graphAreaW = w - graphX - meterW - pad;

    // Linked-mode-only widget set -- unchanged from this plugin's own
    // original single-graph/single-strip look ("Stereo linked mode should
    // still look like it does now"). Positions computed unconditionally
    // regardless of which set is actually visible right now, same "a
    // hidden widget's stale bounds cost nothing" convention the sidechain
    // panel below already follows.
    graph_->setAbsolutePos(int(graphX), int(mainY));
    graph_->setSize(uint(std::max(10.0f, graphAreaW)), uint(std::max(10.0f, mainH)));

    strip_->setAbsolutePos(int(pad), int(h - stripH - pad));
    strip_->setSize(uint(w - 2.0f * pad), uint(stripH));

    // Dual-Mono/Mid-Side widget set -- both compressors side by side. Two
    // columns sharing the same width graph_ alone used to get, each with
    // its own small Ratio Direction header above its own (correspondingly
    // shorter) graph.
    constexpr float dualColGap = 8.0f;
    constexpr float ratioDirBtnH = 22.0f;
    const float colW = (graphAreaW - dualColGap) * 0.5f;
    const float colAx = graphX;
    const float colBx = graphX + colW + dualColGap;
    const float dualGraphY = mainY + ratioDirBtnH + gap;
    const float dualGraphH = mainH - ratioDirBtnH - gap;

    btnRatioDirectionA_->setAbsolutePos(int(colAx), int(mainY));
    btnRatioDirectionA_->setSize(uint(std::max(10.0f, colW)), uint(ratioDirBtnH));
    graphA_->setAbsolutePos(int(colAx), int(dualGraphY));
    graphA_->setSize(uint(std::max(10.0f, colW)), uint(std::max(10.0f, dualGraphH)));

    btnRatioDirectionB_->setAbsolutePos(int(colBx), int(mainY));
    btnRatioDirectionB_->setSize(uint(std::max(10.0f, colW)), uint(ratioDirBtnH));
    graphB_->setAbsolutePos(int(colBx), int(dualGraphY));
    graphB_->setSize(uint(std::max(10.0f, colW)), uint(std::max(10.0f, dualGraphH)));

    // stripGlobalIn_ under meterIn_, on the far left, then stripA_/stripB_
    // each under their own graph (each channel keeps its own gains), then
    // stripGlobal_ (just the one shared Global Output Gain master trim) on
    // the far right.
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

    // Sidechain-options overlay panel -- covers the same content area
    // (meterIn_/graph_/meterOut_/strip_) rather than resizing the window
    // or living alongside it, see refreshScPanelVisibility()'s own
    // comment for why this is an overlay at all. Bounds computed every
    // layout() call regardless of scPanelVisible_ -- a hidden widget's
    // bounds being stale between shows costs nothing and keeps this
    // function from needing its own conditional.
    constexpr float scButtonRowH = 26.0f;
    const float panelY = mainY;
    const float panelH = h - panelY - pad;
    btnScFilterEnable_->setAbsolutePos(int(pad), int(panelY));
    btnScFilterEnable_->setSize(120, uint(scButtonRowH));
    btnScListen_->setAbsolutePos(int(pad) + 120 + 6, int(panelY));
    btnScListen_->setSize(110, uint(scButtonRowH));
    btnScLinkLR_->setAbsolutePos(int(pad) + 120 + 6 + 110 + 6, int(panelY));
    btnScLinkLR_->setSize(100, uint(scButtonRowH));
    btnScEditBank_->setAbsolutePos(int(pad) + 120 + 6 + 110 + 6 + 100 + 6, int(panelY));
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
}

void UICompressorStereo::onGraphParamChanged(uint32_t index, float value)
{
    paramCache_[index] = value;
    setParameterValue(index, value);
    // Whichever of strip_/stripA_/stripB_ is actually showing this
    // Parameter right now repaints to reflect the drag live -- cheap to
    // just repaint all three regardless of which one's actually visible.
    strip_->repaint();
    stripA_->repaint();
    stripB_->repaint();
}

uint32_t UICompressorStereo::bParam(bool isB, uint32_t paramA, uint32_t paramB) const
{
    return (isB && isChannelBIndependent()) ? paramB : paramA;
}

void UICompressorStereo::refreshBypassColors()
{
    btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                           bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                           Color(230, 232, 236));
}

void UICompressorStereo::refreshStripFields()
{
    // Linked mode only now -- always A's own Parameters unconditionally
    // (Linked has no B bank at all). The same field list as before, with
    // Wet/Dry appended, and Output (Global Output Gain) moved to the very
    // right.
    strip_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamInputGain,
          "Input Gain: trim applied to the signal before compression." },
        { "Threshold", "dB", -60.0f, 0.0f, -18.0f, 0.5f, false, 1, kParamThreshold,
          "Threshold: the level above which compression begins." },
        { "Ratio", ":1", 1.0f, 20.0f, 4.0f, 0.1f, false, 1, kParamRatio,
          "Ratio: how strongly the signal is compressed once past the threshold." },
        { "Attack", "ms", 0.1f, 300.0f, 10.0f, 0.0f, true, 2, kParamAttack,
          "Attack: how quickly compression engages once the signal crosses the threshold." },
        { "Release", "ms", 5.0f, 2000.0f, 150.0f, 0.0f, true, 1, kParamRelease,
          "Release: how quickly compression relaxes once the signal drops back below the threshold." },
        { "Knee", "dB", 0.0f, 24.0f, 6.0f, 0.5f, false, 1, kParamKnee,
          "Knee: how gradually compression ramps in around the threshold, instead of switching on abruptly." },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamOutputGain,
          "Makeup Gain: trim applied to the compressed signal, before Wet/Dry mixing." },
        { "Wet/Dry", "%", 0.0f, 100.0f, 100.0f, 1.0f, false, 0, kParamWetDry,
          "Wet/Dry: blends the dry, uncompressed signal (0%) with the fully compressed signal (100%).",
          true, "Wet", "Dry" },
        { "Output", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamGlobalOutputGain,
          "Output Gain: a shared master trim applied after everything else." },
    });
    repaint();
}

void UICompressorStereo::refreshDualStripFields()
{
    // stripA_ always shows/edits A's own Parameters. stripB_'s fields
    // individually fall back to A's own matching Parameter (via
    // bParam()) whenever B isn't currently independent -- while linked,
    // both strips therefore point at the *same* Parameter and always show
    // identical live values, a genuine two-way link rather than a
    // one-time copy (see bParam()'s own declaration comment).
    //
    // Tooltips adapt to the mode -- stripB_'s own tooltip text below is built fresh
    // from the *current* stereoMode_/isChannelBIndependent() every time
    // this function runs, which already happens on every mode/Link
    // change (see updateMeterLabelsForMode()/syncBFromA()) -- no separate
    // runtime adaptation needed beyond that.
    const bool bIndep = isChannelBIndependent();
    const bool midSide = stereoMode_ == sscomp::StereoMode::MidSide;
    const std::string bChan = midSide ? "Side" : "right (R)";
    const std::string aChan = midSide ? "Mid" : "left (L)";
    const std::string bNote = bIndep
        ? (" Independent from the " + aChan + " channel.")
        : (" Linked to the " + aChan + " channel right now -- edit either, both follow.");

    stripA_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamInputGain,
          "Input Gain: trim applied to the " + aChan + " channel before compression." },
        { "Threshold", "dB", -60.0f, 0.0f, -18.0f, 0.5f, false, 1, kParamThreshold,
          "Threshold: the level above which compression begins on the " + aChan + " channel." },
        { "Ratio", ":1", 1.0f, 20.0f, 4.0f, 0.1f, false, 1, kParamRatio,
          "Ratio: how strongly the " + aChan + " channel is compressed once past the threshold." },
        { "Attack", "ms", 0.1f, 300.0f, 10.0f, 0.0f, true, 2, kParamAttack,
          "Attack: how quickly compression engages on the " + aChan + " channel." },
        { "Release", "ms", 5.0f, 2000.0f, 150.0f, 0.0f, true, 1, kParamRelease,
          "Release: how quickly compression relaxes on the " + aChan + " channel." },
        { "Knee", "dB", 0.0f, 24.0f, 6.0f, 0.5f, false, 1, kParamKnee,
          "Knee: how gradually compression ramps in around the threshold on the " + aChan + " channel." },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamOutputGain,
          "Makeup Gain: trim applied to the " + aChan + " channel's own compressed signal." },
        { "Wet/Dry", "%", 0.0f, 100.0f, 100.0f, 1.0f, false, 0, kParamWetDry,
          "Wet/Dry: blends dry (0%) and fully compressed (100%) signal on the " + aChan + " channel.",
          true, "Wet", "Dry" },
    });
    stripB_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, bParam(true, kParamInputGain, kParamInputGainB),
          "Input Gain: trim applied to the " + bChan + " channel before compression." + bNote },
        { "Threshold", "dB", -60.0f, 0.0f, -18.0f, 0.5f, false, 1, bParam(true, kParamThreshold, kParamThresholdB),
          "Threshold: the level above which compression begins on the " + bChan + " channel." + bNote },
        { "Ratio", ":1", 1.0f, 20.0f, 4.0f, 0.1f, false, 1, bParam(true, kParamRatio, kParamRatioB),
          "Ratio: how strongly the " + bChan + " channel is compressed once past the threshold." + bNote },
        { "Attack", "ms", 0.1f, 300.0f, 10.0f, 0.0f, true, 2, bParam(true, kParamAttack, kParamAttackB),
          "Attack: how quickly compression engages on the " + bChan + " channel." + bNote },
        { "Release", "ms", 5.0f, 2000.0f, 150.0f, 0.0f, true, 1, bParam(true, kParamRelease, kParamReleaseB),
          "Release: how quickly compression relaxes on the " + bChan + " channel." + bNote },
        { "Knee", "dB", 0.0f, 24.0f, 6.0f, 0.5f, false, 1, bParam(true, kParamKnee, kParamKneeB),
          "Knee: how gradually compression ramps in around the threshold on the " + bChan + " channel." + bNote },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, bParam(true, kParamOutputGain, kParamMakeupGainB),
          "Makeup Gain: trim applied to the " + bChan + " channel's own compressed signal." + bNote },
        { "Wet/Dry", "%", 0.0f, 100.0f, 100.0f, 1.0f, false, 0, bParam(true, kParamWetDry, kParamWetDryB),
          "Wet/Dry: blends dry (0%) and fully compressed (100%) signal on the " + bChan + " channel." + bNote,
          true, "Wet", "Dry" },
    });
    repaint();
}

void UICompressorStereo::refreshScLinkLabel()
{
    // Same green-when-linked convention as btnLink_'s own label just
    // above -- Linked is this control's default too.
    btnScLinkLR_->setColors(scFilterLinkLR_ ? Color(70, 150, 90) : Color(58, 62, 70),
                             scFilterLinkLR_ ? Color(85, 175, 110) : Color(78, 84, 96),
                             Color(230, 232, 236));
    btnScLinkLR_->setLabel(scFilterLinkLR_ ? "L/R Link: On" : "L/R Link: Off");
}

void UICompressorStereo::refreshScStripFields()
{
    // A mode where left and right are filtered differently -- Highpass/Lowpass point at R's
    // own Parameters whenever editingScBankR_ selects it, same
    // refreshStripFields()-for-the-sidechain-panel pattern. SC Gain stays
    // shared (kParamSidechainGain is one amplification stage ahead of the
    // filter, not per-side).
    const bool r = editingScBankR_ == 1;
    const std::string side = r ? "right (R)" : (scFilterLinkLR_ ? "left+right (linked)" : "left (L)");
    scStrip_->setFields({
        { "Highpass", "Hz", 20.0f, 8000.0f, 100.0f, 0.0f, true, 0, r ? kParamScFilterHighpassHzR : kParamScFilterHighpassHz,
          "Sidechain Highpass: removes content below this frequency from the " + side + " sidechain filter before it reaches the detector." },
        { "Lowpass", "Hz", 100.0f, 20000.0f, 8000.0f, 0.0f, true, 0, r ? kParamScFilterLowpassHzR : kParamScFilterLowpassHz,
          "Sidechain Lowpass: removes content above this frequency from the " + side + " sidechain filter before it reaches the detector." },
        { "SC Gain", "dB", float(sscomp::kSidechainGainMinDb), float(sscomp::kSidechainGainMaxDb), 0.0f, 0.5f, false, 1, kParamSidechainGain,
          "Sidechain Gain: amplifies the trigger signal (main signal, or the external Sidechain input when selected) before the filter and the detector. Doesn't change the audio you hear, only what drives the compressor." },
    });
    // Tint the scope red while R's corners are edited -- scScope_ doesn't exist yet the very first time this runs
    // (called once during construction, before scScope_ itself is built,
    // same as strip_'s own construction-order guards elsewhere in this
    // file), hence the null check.
    if (scScope_ != nullptr) scScope_->setEditingRight(r);
    repaint();
}

void UICompressorStereo::refreshListenColors()
{
    // Same green-when-active convention as Sidechain's own toggle just
    // below -- the label doubles as the readout so it's always visible
    // which channel (if any) is currently isolated.
    const bool active = listenMode_ != 0;
    btnListen_->setColors(active ? Color(70, 150, 90) : Color(58, 62, 70),
                           active ? Color(85, 175, 110) : Color(78, 84, 96),
                           Color(230, 232, 236));
    const char* label = listenMode_ == 1 ? "Listen: Mid" : listenMode_ == 2 ? "Listen: Side" : "Listen: Off";
    btnListen_->setLabel(label);
}

bool UICompressorStereo::isChannelBIndependent() const
{
    // Mirrors PluginCompressorStereo::isChannelBIndependent() exactly --
    // see that method's own comment (PluginCompressorStereo.hpp).
    return stereoMode_ == StereoMode::MidSide ||
           (stereoMode_ == StereoMode::DualMono && !linkAB_);
}

void UICompressorStereo::refreshLinkLabel()
{
    // Green-when-linked (not green-when-active like Listen/Sidechain
    // above) -- Linked is this control's own *default*,
    // matching the plugin's pre-existing shared-parameter behavior, so
    // the "nothing special is happening" state reads as the neutral/
    // affirmative color here, the same way it would for any other
    // "normal" setting.
    btnLink_->setColors(linkAB_ ? Color(70, 150, 90) : Color(58, 62, 70),
                         linkAB_ ? Color(85, 175, 110) : Color(78, 84, 96),
                         Color(230, 232, 236));
    btnLink_->setLabel(linkAB_ ? "Link: On" : "Link: Off");
}

void UICompressorStereo::refreshDualLayoutVisibility()
{
    // Linked keeps graph_/strip_ exactly as before; Dual-Mono and
    // Mid/Side both get the new side-by-side pair, regardless of Link
    // state (Link only governs which Parameters stripB_/graphB_ actually
    // target, see bParam()'s own comment, not whether B is shown at all
    // any more). Both sets hidden entirely while the sidechain-options
    // panel is up, same as meterIn_/meterOut_ -- see
    // refreshScPanelVisibility()'s own comment for why.
    const bool dual = stereoMode_ != StereoMode::Linked;
    const bool showMain = !scPanelVisible_;
    meterIn_->setVisible(showMain);
    meterOut_->setVisible(showMain);
    graph_->setVisible(showMain && !dual);
    strip_->setVisible(showMain && !dual);
    graphA_->setVisible(showMain && dual);
    graphB_->setVisible(showMain && dual);
    stripA_->setVisible(showMain && dual);
    stripB_->setVisible(showMain && dual);
    stripGlobal_->setVisible(showMain && dual);
    stripGlobalIn_->setVisible(showMain && dual);
    btnRatioDirectionA_->setVisible(showMain && dual);
    btnRatioDirectionB_->setVisible(showMain && dual);
    layout();
    repaint();
}

void UICompressorStereo::syncBFromA()
{
    paramCache_[kParamThresholdB] = paramCache_[kParamThreshold];
    paramCache_[kParamRatioB] = paramCache_[kParamRatio];
    paramCache_[kParamAttackB] = paramCache_[kParamAttack];
    paramCache_[kParamReleaseB] = paramCache_[kParamRelease];
    paramCache_[kParamKneeB] = paramCache_[kParamKnee];
    paramCache_[kParamMakeupGainB] = paramCache_[kParamOutputGain];
    paramCache_[kParamInputGainB] = paramCache_[kParamInputGain];
    paramCache_[kParamWetDryB] = paramCache_[kParamWetDry];
    setParameterValue(kParamThresholdB, paramCache_[kParamThresholdB]);
    setParameterValue(kParamRatioB, paramCache_[kParamRatioB]);
    setParameterValue(kParamAttackB, paramCache_[kParamAttackB]);
    setParameterValue(kParamReleaseB, paramCache_[kParamReleaseB]);
    setParameterValue(kParamKneeB, paramCache_[kParamKneeB]);
    setParameterValue(kParamMakeupGainB, paramCache_[kParamMakeupGainB]);
    setParameterValue(kParamInputGainB, paramCache_[kParamInputGainB]);
    setParameterValue(kParamWetDryB, paramCache_[kParamWetDryB]);
    ratioUpModeB_ = ratioUpMode_;
    setState("ratioDirectionB", ratioUpModeB_ ? "1" : "0");
    // stripB_/graphB_ are always showing (no more editingBank_ toggle) --
    // keep their live display in sync with what just changed underneath
    // them. Called with the *new* stereoMode_/linkAB_ already in effect
    // (see both call sites), so bParam() already resolves to B's own
    // independent Parameters here, not A's fallback.
    refreshDualStripFields();
    refreshGraphCurves();
    refreshRatioDirectionLabel(btnRatioDirectionB_, ratioUpModeB_);
    refreshMakeupMarkers();
}

void UICompressorStereo::refreshGraphCurves()
{
    graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
    graphA_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
    if (isChannelBIndependent())
        graphB_->setCurveParams(paramCache_[kParamThresholdB], paramCache_[kParamRatioB], ratioUpModeB_, paramCache_[kParamKneeB]);
    else
        graphB_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
}

void UICompressorStereo::refreshMakeupMarkers()
{
    // Makeup Gain shown in the gain reduction, like the Multiband
    // Compressor -- see CompressorGraphWidget::
    // drawGrMeterColumn()'s own comment for the marker itself. Each of
    // graph_/graphA_/graphB_ is now a full standalone compressor display
    // (not one half of a shared one), so each only ever needs its own
    // single (A-argument) marker -- the B-argument is passed the same
    // value purely because setMakeupGainDb() takes both, never actually
    // drawn since none of these three graphs shows a secondary column any
    // more (see updateMeterLabelsForMode()'s own comment).
    graph_->setMakeupGainDb(paramCache_[kParamOutputGain], paramCache_[kParamOutputGain]);
    graphA_->setMakeupGainDb(paramCache_[kParamOutputGain], paramCache_[kParamOutputGain]);
    const float bMakeup = isChannelBIndependent() ? paramCache_[kParamMakeupGainB] : paramCache_[kParamOutputGain];
    graphB_->setMakeupGainDb(bMakeup, bMakeup);
}

void UICompressorStereo::refreshSidechainColors()
{
    // Same green-when-on convention as every other toggle in this file;
    // the label doubles as the readout so it's always visible which
    // signal is actually driving the detector right now.
    btnSidechain_->setColors(sidechainEnabled_ ? Color(70, 150, 90) : Color(58, 62, 70),
                              sidechainEnabled_ ? Color(85, 175, 110) : Color(78, 84, 96),
                              Color(230, 232, 236));
    btnSidechain_->setLabel(sidechainEnabled_ ? "Sidechain: External" : "Sidechain: Internal");
}

void UICompressorStereo::refreshStereoModeLabel()
{
    btnStereoMode_->setLabel(stereoModeName(stereoMode_));
    repaint();
}

void UICompressorStereo::refreshRatioDirectionLabel(SimpleButton* btn, bool up)
{
    // The swap button itself -- see kStateRatioDirection's own comment for
    // why this replaced a second, independently-adjustable ratio field.
    // Shared by all three Ratio Direction buttons (btnRatioDirection_/
    // btnRatioDirectionA_/btnRatioDirectionB_) -- each call site passes
    // whichever value that particular button currently reflects.
    btn->setLabel(up ? "Ratio: Up \xE2\x96\xB2" : "Ratio: Down \xE2\x96\xBC"); // UTF-8 up/down triangles
}

void UICompressorStereo::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UICompressorStereo::onNanoDisplay()
{
    const float w = float(getWidth());
    fillColor(Color(20, 21, 25));
    beginPath();
    rect(0, 0, w, float(getHeight()));
    fill();
}

void UICompressorStereo::parameterChanged(uint32_t index, float value)
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
    case kParamRatio:
    case kParamKnee:
    case kParamThresholdB:
    case kParamRatioB:
    case kParamKneeB:
        refreshGraphCurves();
        break;
    case kParamOutputGain:
    case kParamMakeupGainB:
        refreshMakeupMarkers();
        break;
    case kParamInputGainB:
    case kParamWetDry:
    case kParamWetDryB:
    case kParamGlobalInputGain:
    case kParamUiHeartbeat:
        // paramCache_ already updated above (unconditional at the top of
        // this function) -- ParamStripWidget reads it live via its own
        // getter, nothing further to push. Explicit cases so these don't
        // fall into default below and get misread as spectrum-bank
        // packets (they sit numerically past kParamMeterScSpectrumInBase's
        // own bank -- "append, never insert").
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
    case kParamMeterDetectorA:
    case kParamMeterGainReductionA:
        graph_->setLiveLevels(paramCache_[kParamMeterDetectorA], paramCache_[kParamMeterGainReductionA]);
        graphA_->setLiveLevels(paramCache_[kParamMeterDetectorA], paramCache_[kParamMeterGainReductionA]);
        break;
    case kParamMeterDetectorB:
    case kParamMeterGainReductionB:
        // graphB_'s own primary display now (not a secondary column on a
        // shared graph_ any more) -- see updateMeterLabelsForMode()'s own
        // comment for why.
        graphB_->setLiveLevels(paramCache_[kParamMeterDetectorB], paramCache_[kParamMeterGainReductionB]);
        break;
    case kParamScFilterEnable:
        if (scPanelVisible_)
        {
            const bool filterOn = value >= 0.5f;
            btnScFilterEnable_->setColors(filterOn ? Color(70, 150, 90) : Color(58, 62, 70),
                                           filterOn ? Color(85, 175, 110) : Color(78, 84, 96),
                                           Color(230, 232, 236));
            btnScFilterEnable_->setLabel(filterOn ? "Filter: On" : "Filter: Off");
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
        // The packed sidechain-monitor spectrum banks -- see
        // kParamMeterScSpectrumBase/InBase's own comments
        // (PluginCompressorStereo.hpp). scScope_ keeps receiving these
        // even while the panel is hidden (cheap -- just a float array
        // write, see SidechainScopeWidget::setSpectrumInPacket()/
        // setSpectrumOutPacket()) rather than special-casing "don't
        // bother updating a widget nobody's looking at" -- so the
        // graphic is already current the instant the panel's opened
        // again.
        if (index >= kParamMeterScSpectrumInBase && index < kParamMeterScSpectrumInBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
            scScope_->setSpectrumInPacket(int(index - kParamMeterScSpectrumInBase), value);
        else if (index >= kParamMeterScSpectrumBase && index < kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands)
            scScope_->setSpectrumOutPacket(int(index - kParamMeterScSpectrumBase), value);
        break;
    }
    repaint();
}

void UICompressorStereo::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "stereoMode") == 0)
    {
        stereoMode_ = sscomp::clampStereoMode(std::atoi(value));
        refreshStereoModeLabel();
        updateMeterLabelsForMode();
        return;
    }
    if (std::strcmp(key, "ratioDirection") == 0)
    {
        ratioUpMode_ = std::atoi(value) != 0;
        refreshGraphCurves();
        refreshRatioDirectionLabel(btnRatioDirection_, ratioUpMode_);
        refreshRatioDirectionLabel(btnRatioDirectionA_, ratioUpMode_);
        if (!isChannelBIndependent()) refreshRatioDirectionLabel(btnRatioDirectionB_, ratioUpMode_);
        repaint();
        return;
    }
    if (std::strcmp(key, "ratioDirectionB") == 0)
    {
        ratioUpModeB_ = std::atoi(value) != 0;
        refreshGraphCurves();
        if (isChannelBIndependent()) refreshRatioDirectionLabel(btnRatioDirectionB_, ratioUpModeB_);
        repaint();
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
}

ParamStripWidget* UICompressorStereo::editingStrip() const
{
    if (strip_->isEditing()) return strip_;
    if (stripA_->isEditing()) return stripA_;
    if (stripB_->isEditing()) return stripB_;
    if (stripGlobal_->isEditing()) return stripGlobal_;
    if (stripGlobalIn_->isEditing()) return stripGlobalIn_;
    if (scStrip_->isEditing()) return scStrip_;
    return nullptr;
}

bool UICompressorStereo::onCharacterInput(const CharacterInputEvent& ev)
{
    // A real bug (the sidechain panel's Highpass/Lowpass/SC Gain fields
    // couldn't be typed into): this only ever checked strip_'s
    // own isEditing(), never scStrip_'s -- the sidechain panel's own
    // fields silently never received keystrokes at all. Only one strip
    // can ever be mid-edit at once (only one is visible/clickable at a
    // time, see editingStrip()'s own comment), so routing to whichever
    // one actually claims it is unambiguous.
    ParamStripWidget* editing = editingStrip();
    if (editing == nullptr) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { editing->commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { editing->appendChar(c); return true; }
    return true;
}

bool UICompressorStereo::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z/Ctrl+Y -- see UICompressorMono::onKeyboard()'s identical
    // block for the full reasoning.
    if (ev.press && (ev.mod & DGL_NAMESPACE::kModifierControl) != 0
        && !(ev.mod & DGL_NAMESPACE::kModifierAlt) && !(ev.mod & DGL_NAMESPACE::kModifierSuper))
    {
        const bool shift = (ev.mod & DGL_NAMESPACE::kModifierShift) != 0;
        if (ev.key == 'z' && !shift) { onUndo(); return true; }
        if (ev.key == 'y' || (ev.key == 'z' && shift)) { onRedo(); return true; }
    }

    // Same fix as onCharacterInput() above -- route to whichever strip is
    // actually mid-edit, not just strip_.
    ParamStripWidget* editing = editingStrip();
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

void UICompressorStereo::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
}

void UICompressorStereo::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".mcsp";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    if (presetio::save(p, captureSnapshot(), "MimoMusic Compressor Stereo preset"))
        rememberPresetDir(p.c_str());
}

void UICompressorStereo::loadPresetFile(const char* path)
{
    presetio::KeyValues values;
    if (!presetio::load(path, values)) return;
    applySnapshot(values);
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See UICompressorMono.cpp for the full design writeup this is ported
// from (against this plugin's own parameter list/stereoMode state).

presetio::KeyValues UICompressorStereo::captureSnapshot() const
{
    presetio::KeyValues values;
    for (size_t i = 0; i < kPresetParamCount; ++i)
        values[kPresetKeys[i]] = double(paramCache_[kPresetParams[i]]);
    values["stereoMode"] = double(int(stereoMode_));
    values["ratioDirection"] = ratioUpMode_ ? 1.0 : 0.0;
    values["ratioDirectionB"] = ratioUpModeB_ ? 1.0 : 0.0;
    values["linkAB"] = linkAB_ ? 1.0 : 0.0;
    return values;
}

void UICompressorStereo::applySnapshot(const presetio::KeyValues& values)
{
    // Formerly loadPresetFile()'s own body past the file read.
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
    if (modeIt != values.end())
    {
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%d", int(modeIt->second));
        setState("stereoMode", buf);
        stereoMode_ = sscomp::clampStereoMode(int(modeIt->second));
        refreshStereoModeLabel();
        updateMeterLabelsForMode();
    }

    const auto dirIt = values.find("ratioDirection");
    if (dirIt != values.end())
    {
        ratioUpMode_ = dirIt->second != 0.0;
        char dirBuf[4];
        std::snprintf(dirBuf, sizeof(dirBuf), "%d", ratioUpMode_ ? 1 : 0);
        setState("ratioDirection", dirBuf);
        refreshRatioDirectionLabel(btnRatioDirection_, ratioUpMode_);
        refreshRatioDirectionLabel(btnRatioDirectionA_, ratioUpMode_);
    }

    const auto dirBIt = values.find("ratioDirectionB");
    if (dirBIt != values.end())
    {
        ratioUpModeB_ = dirBIt->second != 0.0;
        char dirBuf[4];
        std::snprintf(dirBuf, sizeof(dirBuf), "%d", ratioUpModeB_ ? 1 : 0);
        setState("ratioDirectionB", dirBuf);
    }

    const auto linkIt = values.find("linkAB");
    if (linkIt != values.end())
    {
        linkAB_ = linkIt->second != 0.0;
        setState("linkAB", linkAB_ ? "1" : "0");
        refreshLinkLabel();
    }

    strip_->repaint();
    stripA_->repaint();
    stripB_->repaint();
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
    // updateMeterLabelsForMode() (already called above if "stereoMode" was
    // present) already refreshed the dual strips/graphs against the
    // restored linkAB_/stereoMode_ -- refresh them again here, after every
    // value above (including Ratio Direction) has finished restoring, so
    // they reflect the final, fully-restored state.
    refreshStripFields();
    refreshDualStripFields();
    refreshGraphCurves();
    refreshRatioDirectionLabel(btnRatioDirectionB_, isChannelBIndependent() ? ratioUpModeB_ : ratioUpMode_);
    refreshMakeupMarkers();
    repaint();
}

void UICompressorStereo::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = captureSnapshot(); abHasB_ = true;
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applySnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
}

void UICompressorStereo::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = captureSnapshot(); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applySnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
}

void UICompressorStereo::onAbCopy()
{
    const presetio::KeyValues snap = captureSnapshot();
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
}

void UICompressorStereo::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UICompressorStereo::onUndo()
{
    if (undoStack_.empty()) return;
    redoStack_.push_back(lastCommittedSnapshot_);
    if (redoStack_.size() > kMaxUndoDepth) redoStack_.erase(redoStack_.begin());
    const presetio::KeyValues prev = undoStack_.back();
    undoStack_.pop_back();
    applyingSnapshot_ = true;
    applySnapshot(prev);
    applyingSnapshot_ = false;
    lastCommittedSnapshot_ = prev;
    lastSeenSnapshot_ = prev;
    idleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UICompressorStereo::onRedo()
{
    if (redoStack_.empty()) return;
    undoStack_.push_back(lastCommittedSnapshot_);
    if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
    const presetio::KeyValues next = redoStack_.back();
    redoStack_.pop_back();
    applyingSnapshot_ = true;
    applySnapshot(next);
    applyingSnapshot_ = false;
    lastCommittedSnapshot_ = next;
    lastSeenSnapshot_ = next;
    idleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UICompressorStereo::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UICompressorStereo::uiIdle()
{
    // Unconditional, before any of the undo/redo debounce logic's own
    // early returns below -- see SidechainScopeWidget::tickLevelHistory()'s
    // own comment for why the scrolling sidechain-level trace has to
    // advance on a fixed per-tick cadence rather than piggybacking on
    // parameter-change delivery.
    scScope_->tickLevelHistory();
    tooltip_->tick();

    // Tell the DSP the scope is on screen -- see kParamUiHeartbeat. Only while the panel is visible, so a closed
    // window or a hidden panel stops the spectrum FFTs.
    if (scPanelVisible_)
    {
        uiHeartbeat_ = uiHeartbeat_ >= 999999.0f ? 1.0f : uiHeartbeat_ + 1.0f;
        setParameterValue(kParamUiHeartbeat, uiHeartbeat_);
    }

    if (applyingSnapshot_) return;

    const presetio::KeyValues now = captureSnapshot();
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

void UICompressorStereo::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UICompressorStereo::refreshScPanelVisibility()
{
    // See this method's own declaration comment (UICompressorStereo.hpp)
    // for why this is an overlay of ordinary child widgets rather than a
    // second Window. btnSidechainOptions_ itself stays visible/clickable
    // throughout (it's the only way back in once the panel's closed via
    // btnScClose_), doubling as an on/off indicator the same
    // green-when-active way every other toggle in this file already does.
    //
    // The normal content underneath (meterIn_/meterOut_ and whichever of
    // graph_+strip_/graphA_+graphB_+stripA_+stripB_+stripGlobal_ currently
    // applies) is hidden while the panel's up, not just painted over -- a
    // hidden widget neither draws (no visual leak-through in the gaps
    // between the panel's own controls) nor receives mouse events (DGL's
    // own giveMouseEventForSubWidgets() skips !isVisible() widgets, see
    // dpf/dgl/src/WidgetPrivateData.cpp), so there's no separate backdrop/
    // z-order/click-blocking mechanism to get right at all. Delegated to
    // refreshDualLayoutVisibility(), which already knows -- via
    // scPanelVisible_ itself -- to hide everything main-content-related
    // regardless of stereoMode_ while this panel's up.
    refreshDualLayoutVisibility();
    btnScFilterEnable_->setVisible(scPanelVisible_);
    btnScListen_->setVisible(scPanelVisible_);
    btnScLinkLR_->setVisible(scPanelVisible_);
    btnScEditBank_->setVisible(scPanelVisible_);
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
        btnScFilterEnable_->setLabel(filterOn ? "Filter: On" : "Filter: Off");
        scScope_->setFilterRange(filterOn,
                                  editingScBankR_ == 0 ? paramCache_[kParamScFilterHighpassHz] : paramCache_[kParamScFilterHighpassHzR],
                                  editingScBankR_ == 0 ? paramCache_[kParamScFilterLowpassHz] : paramCache_[kParamScFilterLowpassHzR]);
        refreshScListenColors();
    }
    else if (paramCache_[kParamScListenEnable] >= 0.5f)
    {
        // "don't leave an audition override silently armed" -- see
        // btnScListen_'s own declaration comment (UICompressorStereo.hpp).
        setParameterValue(kParamScListenEnable, 0.0f);
        paramCache_[kParamScListenEnable] = 0.0f;
        refreshScListenColors();
    }
    layout();
    repaint();
}

void UICompressorStereo::refreshScListenColors()
{
    const bool on = paramCache_[kParamScListenEnable] >= 0.5f;
    btnScListen_->setColors(on ? Color(70, 150, 90) : Color(58, 62, 70),
                             on ? Color(85, 175, 110) : Color(78, 84, 96),
                             Color(230, 232, 236));
    btnScListen_->setLabel(on ? "Listen: On" : "Listen: Off");
}

void UICompressorStereo::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UICompressorStereo(); }

END_NAMESPACE_DISTRHO
