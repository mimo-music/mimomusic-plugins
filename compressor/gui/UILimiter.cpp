// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UILimiter.hpp"

#include "PresetIO.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::LevelMeterWidget;
using DGL_NAMESPACE::LimiterGraphWidget;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;

namespace {
// Every automatable knob this preset format round-trips.
constexpr uint32_t kPresetParams[] = {
    kParamInputGain, kParamThreshold, kParamAttack, kParamRelease, kParamLookahead, kParamKnee,
    kParamCeiling, kParamOutputGain, kParamBypass, kParamBoost, kParamClipMode, kParamClipKnee, kParamOversampling, kParamRatio
};
constexpr const char* kPresetKeys[] = {
    "inputGainDb", "thresholdDb", "attackMs", "releaseMs", "lookaheadMs", "kneeDb",
    "ceilingDb", "outputGainDb", "bypass", "boost", "clipMode", "clipKneeDb", "oversampling", "ratio"
};
constexpr size_t kPresetParamCount = sizeof(kPresetParams) / sizeof(kPresetParams[0]);
static_assert(sizeof(kPresetKeys) / sizeof(kPresetKeys[0]) == kPresetParamCount, "keys/params length mismatch");
}

UILimiter::UILimiter()
    // true: DPF scales the editor by the host's or desktop's scale factor
    // (Ardour's GUI scaling, else Xft.dpi; DPF_SCALE_FACTOR overrides) and
    // zooms it when the window is resized, keeping the aspect ratio. The
    // layout always sees the default (logical) size.
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
    paramCache_[kParamThreshold] = -6.0f;
    paramCache_[kParamAttack] = 1.0f;
    paramCache_[kParamRelease] = 100.0f;
    paramCache_[kParamLookahead] = 5.0f;
    paramCache_[kParamKnee] = 1.0f;
    paramCache_[kParamCeiling] = 0.0f;
    paramCache_[kParamOutputGain] = 0.0f;
    paramCache_[kParamBypass] = 0.0f;
    paramCache_[kParamBoost] = 0.0f;
    paramCache_[kParamClipMode] = float(int(defaultClipModeFor(LimiterAlgorithm::Classic)));
    paramCache_[kParamClipKnee] = 1.0f;
    paramCache_[kParamOversampling] = 0.0f;
    paramCache_[kParamMeterClip] = 0.0f;
    paramCache_[kParamRatio] = 0.0f; // 20:1
    paramCache_[kParamMeterInputRmsL] = paramCache_[kParamMeterInputPeakL] = -120.0f;
    paramCache_[kParamMeterInputRmsR] = paramCache_[kParamMeterInputPeakR] = -120.0f;
    paramCache_[kParamMeterOutputRmsL] = paramCache_[kParamMeterOutputPeakL] = -120.0f;
    paramCache_[kParamMeterOutputRmsR] = paramCache_[kParamMeterOutputPeakR] = -120.0f;
    paramCache_[kParamMeterDetector] = -120.0f;
    paramCache_[kParamMeterGainReduction] = 0.0f;

    meterIn_ = new LevelMeterWidget(this, "IN", 2);
    meterOut_ = new LevelMeterWidget(this, "OUT", 2);
    meterIn_->setChannelLabels("L", "R");
    meterOut_->setChannelLabels("L", "R");

    graph_ = new LimiterGraphWidget(this);
    graph_->setThresholdRange(-60.0f, 0.0f);  // matches the strip fields' ranges below
    graph_->setCeilingRange(-12.0f, 0.0f);
    graph_->setClipKneeRange(0.1f, 6.0f);
    graph_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    graph_->setOnCeilingChanged([this](float v) { onGraphParamChanged(kParamCeiling, v); });
    graph_->setOnClipKneeChanged([this](float v) { onGraphParamChanged(kParamClipKnee, v); });

    strip_ = new ParamStripWidget(this);
    strip_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    strip_->setSetter([this](uint32_t idx, float v) {
        setParam(idx, v);
        refreshGraphCurve();
    });
    refreshStripFields();
    refreshGraphCurve();

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] { setParam(kParamBypass, bypassed_ ? 0.0f : 1.0f); refreshBypassColors(); repaint(); });
    refreshBypassColors();

    btnBoost_ = new SimpleButton(this, "Boost");
    btnBoost_->setCallback([this] {
        setParam(kParamBoost, boost_ ? 0.0f : 1.0f);
        refreshBoostColors();
        refreshGraphCurve();
        repaint();
    });
    refreshBoostColors();

    btnAlgorithm_ = new SimpleButton(this, "Classic");
    btnAlgorithm_->setCallback([this] {
        // A State, not a Parameter -- see PluginLimiter.hpp's own top
        // comment on LimiterAlgorithm for why. Updates the local mirror
        // and every dependent widget synchronously (same fix as the
        // De-esser's own field-strip bug this session: don't rely solely
        // on a parameterChanged()/stateChanged() round trip from the host
        // to make an edit visible) -- setState() below is still what
        // actually reaches the DSP side and performs the real switch
        // (including resetting both gain computers' internal state).
        algorithm_ = (algorithm_ == LimiterAlgorithm::Classic) ? LimiterAlgorithm::TruePeak : LimiterAlgorithm::Classic;
        setState("algorithm", algorithm_ == LimiterAlgorithm::TruePeak ? "1" : "0");
        // The new Algorithm's default Clip mode -- only on a click here,
        // not when a preset/session/undo restores an Algorithm, so a Clip
        // mode the user chose is never overridden behind their back.
        setParam(kParamClipMode, float(int(defaultClipModeFor(algorithm_))));
        refreshClipModeColors();
        refreshRatioButton();
        refreshAlgorithmColors();
        refreshStripFields();
        refreshGraphCurve();
        repaint();
    });
    refreshAlgorithmColors();

    btnRatio_ = new SimpleButton(this, "Ratio 20:1");
    btnRatio_->setCallback([this] {
        const int next = (int(std::lround(paramCache_[kParamRatio])) + 1) % sscomp::kLimiterRatioCount;
        setParam(kParamRatio, float(next));
        refreshRatioButton();
        refreshGraphCurve();
        repaint();
    });
    refreshRatioButton();

    btnClipMode_ = new SimpleButton(this, "Clip: Soft");
    btnClipMode_->setCallback([this] {
        const bool soft = paramCache_[kParamClipMode] >= 0.5f;
        setParam(kParamClipMode, soft ? float(int(ClipMode::Hard)) : float(int(ClipMode::Soft)));
        refreshClipModeColors();
        refreshStripFields();
        refreshGraphCurve();
        repaint();
    });
    refreshClipModeColors();

    btnOversampling_ = new SimpleButton(this, "Oversampling: Off");
    btnOversampling_->setCallback([this] {
        setParam(kParamOversampling, paramCache_[kParamOversampling] >= 0.5f ? 0.0f : 1.0f);
        refreshOversamplingColors();
        repaint();
    });
    refreshOversamplingColors();

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load Limiter Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "preset.mlim";
        opts.title = "Save Limiter Preset";
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
    tooltip_->addTarget(btnLoad_, "Load a saved limiter preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a limiter preset.");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed, all metering/detection still runs.");
    tooltip_->addTarget(btnBoost_, "Boost: turns the limited signal back up by the Threshold amount (Threshold -6 dB adds up to +6 dB), so lowering the Threshold makes the result louder instead of quieter. Adds on top of Output Gain. Never pushes peaks over the Ceiling: if Boost plus Output Gain would, the gain is reduced so the limited peaks land exactly on the Ceiling (with Soft clip: where the soft clip zone starts). The graph shows the gain actually applied.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    tooltip_->addTarget(btnAlgorithm_, "Switch limiting algorithm: Classic (envelope follower with Attack and Knee -- softer, but fast transients can overshoot the Threshold briefly) or True Peak (lookahead limiter that never lets a sample over the Threshold). Switching also sets the matching Clip mode: Soft for Classic, Hard for True Peak -- change it afterwards if you like.");
    tooltip_->addTarget(btnRatio_, "Ratio (Classic only): how flat the curve is above the Threshold. 20:1 lets a sustained signal 12 dB over the Threshold through 0.6 dB over it; 100:1 about 0.1 dB; \xE2\x88\x9E:1 not at all. Fast transients can still overshoot in Classic whatever the ratio -- that is what True Peak (always \xE2\x88\x9E:1) is for.");
    tooltip_->addTarget(btnClipMode_, "Clip mode for anything that still reaches the Ceiling (overshoot): Hard cuts it flat at the Ceiling; Soft rounds it off over the Clip Knee below the Ceiling (less harsh, the peaks then sit at the start of the soft zone). True Peak rarely overshoots, so Hard is exact there; Classic does overshoot, so Soft sounds cleaner.");
    tooltip_->addTarget(btnOversampling_, "Oversampling (4x): also detects peaks between samples (inter-sample peaks), which can exceed the Ceiling after D/A conversion or MP3/AAC encoding. Useful when the Ceiling is close to 0 dBFS. Adds 0.25 ms of latency and costs more CPU; off costs nothing extra.");
    // Hover descriptions for the parameter fields.
    tooltip_->addFieldTarget(strip_, [this](double x, double y) { return strip_->fieldTooltipAt(x, y); });

    layout();
}

void UILimiter::layout()
{
    // Row 1: every system-wide/whole-instance control -- Load/Save/
    // Bypass, then A/B compare/Undo/Redo -- flush against the window's
    // own top edge, right-aligned (title text fills the remaining space
    // on the left), placed right-to-left in reverse workflow order so
    // the on-screen reading order is left to right: Load, Save, Bypass,
    // A, B, Copy, Undo, Redo. Row 2 holds the limiting options --
    // Algorithm, Ratio, Clip, Oversampling, Boost -- left-aligned right
    // above the graph, since they all change the curve drawn there.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window -- see TooltipOverlay's own file comment.
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

    // Row 2: the limiting options, left-aligned right above the graph.
    constexpr float topBar2H = 26.0f;
    const float row2Y = topBarH + gap;
    float bx2 = pad;
    auto place2 = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx2), int(row2Y));
        b->setSize(uint(bw), uint(topBar2H));
        bx2 += bw + gap;
    };
    place2(btnAlgorithm_, 84.0f);
    place2(btnRatio_, 90.0f);
    place2(btnClipMode_, 84.0f);
    place2(btnOversampling_, 136.0f);
    place2(btnBoost_, 64.0f);

    const float mainY = row2Y + topBar2H + pad * 2.0f;
    const float mainH = h - mainY - stripH - pad * 2.0f;

    meterIn_->setAbsolutePos(int(pad), int(mainY));
    meterIn_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    const float graphX = pad + meterW + pad;
    const float graphW = w - graphX - meterW - pad * 2.0f;
    graph_->setAbsolutePos(int(graphX), int(mainY));
    graph_->setSize(uint(std::max(10.0f, graphW)), uint(std::max(10.0f, mainH)));

    meterOut_->setAbsolutePos(int(w - pad - meterW), int(mainY));
    meterOut_->setSize(uint(meterW), uint(std::max(10.0f, mainH)));

    strip_->setAbsolutePos(int(pad), int(h - stripH - pad));
    strip_->setSize(uint(w - 2.0f * pad), uint(stripH));
}

void UILimiter::setParam(uint32_t index, float value)
{
    paramCache_[index] = value;
    setParameterValue(index, value);
    // Keep the mirrors in step without waiting for the host to echo it.
    if (index == kParamBypass) bypassed_ = value >= 0.5f;
    if (index == kParamBoost) boost_ = value >= 0.5f;
}

void UILimiter::onGraphParamChanged(uint32_t index, float value)
{
    setParam(index, value);
    refreshGraphCurve();
    strip_->repaint();
}

void UILimiter::refreshBypassColors()
{
    btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                           bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                           Color(230, 232, 236));
}

void UILimiter::refreshBoostColors()
{
    // Green, distinct from Bypass's own orange and Algorithm's own blue --
    // "something extra is active" in a third, unambiguous color rather
    // than reusing either.
    btnBoost_->setColors(boost_ ? Color(60, 130, 70) : Color(58, 62, 70),
                          boost_ ? Color(75, 150, 85) : Color(78, 84, 96),
                          Color(230, 232, 236));
}

void UILimiter::refreshAlgorithmColors()
{
    const bool truePeak = algorithm_ == LimiterAlgorithm::TruePeak;
    btnAlgorithm_->setLabel(truePeak ? "True Peak" : "Classic");
    // Blue for True Peak (distinct from Bypass's own orange "something
    // non-default is active" color), neutral gray for Classic (the
    // unchanged, original default).
    btnAlgorithm_->setColors(truePeak ? Color(55, 100, 150) : Color(58, 62, 70),
                              truePeak ? Color(70, 120, 170) : Color(78, 84, 96),
                              Color(230, 232, 236));
}

void UILimiter::refreshClipModeColors()
{
    const bool soft = paramCache_[kParamClipMode] >= 0.5f;
    btnClipMode_->setLabel(soft ? "Clip: Soft" : "Clip: Hard");
    // Same muted red as the Ceiling line in the graph when Soft is on.
    btnClipMode_->setColors(soft ? Color(140, 75, 65) : Color(58, 62, 70),
                             soft ? Color(160, 90, 78) : Color(78, 84, 96),
                             Color(230, 232, 236));
}

void UILimiter::refreshRatioButton()
{
    const bool truePeak = algorithm_ == LimiterAlgorithm::TruePeak;
    const int idx = truePeak ? 2 : int(std::lround(paramCache_[kParamRatio]));
    const std::string label = std::string("Ratio ") + sscomp::limiterRatioLabel(idx);
    btnRatio_->setLabel(label.c_str());
    btnRatio_->setEnabled(!truePeak);
    btnRatio_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRatio_->repaint();
}

void UILimiter::refreshOversamplingColors()
{
    const bool on = paramCache_[kParamOversampling] >= 0.5f;
    btnOversampling_->setLabel(on ? "Oversampling: 4x" : "Oversampling: Off");
    btnOversampling_->setColors(on ? Color(55, 100, 150) : Color(58, 62, 70),
                                 on ? Color(70, 120, 170) : Color(78, 84, 96),
                                 Color(230, 232, 236));
}

void UILimiter::refreshStripFields()
{
    // Attack and Knee play no role in True Peak mode (see
    // TruePeakLimiterDsp.hpp's file comment) -- rather than hiding them
    // outright (their host-automatable Parameters still exist and still
    // accept values regardless of which algorithm is selected, so hiding
    // the field would make an incoming automation lane's value invisible)
    // their label grows a "(unused)" suffix instead, and setFields() is
    // simply called again with the updated list -- cheap, and
    // ParamStripWidget already supports being retargeted at any time
    // (UIMbCompressor's analogue in the sibling project does the same for
    // its own per-band retargeting).
    // Tooltips adapt to the mode -- Attack/Knee's own tooltip text already reflects
    // which algorithm is selected, same "(unused)" adaptation their label
    // already gets, and this whole list is already rebuilt on every
    // Algorithm change (see btnAlgorithm_'s own callback).
    const bool truePeak = algorithm_ == LimiterAlgorithm::TruePeak;
    const bool soft = paramCache_[kParamClipMode] >= 0.5f;
    strip_->setFields({
        { "Input", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamInputGain,
          "Input Gain: trim applied to the signal before limiting." },
        { "Threshold", "dB", -60.0f, 0.0f, -6.0f, 0.5f, false, 1, kParamThreshold,
          "Threshold: the level above which limiting begins." },
        { truePeak ? "Attack (unused)" : "Attack", "ms", 0.1f, 20.0f, 1.0f, 0.0f, true, 2, kParamAttack,
          truePeak ? "Attack: not used by the True Peak algorithm (it reacts using Lookahead instead)."
                    : "Attack: how quickly limiting engages once the signal crosses the threshold." },
        { "Release", "ms", 5.0f, 2000.0f, 100.0f, 0.0f, true, 1, kParamRelease,
          "Release: how quickly limiting relaxes once the signal drops back below the threshold." },
        { "Lookahead", "ms", 0.0f, 20.0f, 5.0f, 0.5f, false, 1, kParamLookahead,
          "Lookahead: how far ahead the limiter looks to catch a transient before it clips." },
        { truePeak ? "Knee (unused)" : "Knee", "dB", 0.0f, 12.0f, 1.0f, 0.5f, false, 1, kParamKnee,
          truePeak ? "Knee: not used by the True Peak algorithm."
                    : "Knee: how gradually limiting ramps in around the threshold, instead of switching on abruptly." },
        { "Ceiling", "dB", -12.0f, 0.0f, 0.0f, 0.1f, false, 2, kParamCeiling,
          "Ceiling: the output never goes above this level. Boost and Output Gain are reduced if they would push the limited peaks over it." },
        { soft ? "Clip Knee" : "Clip Knee (unused)", "dB", 0.1f, 6.0f, 1.0f, 0.1f, false, 1, kParamClipKnee,
          soft ? "Clip Knee: how far below the Ceiling the soft clip starts rounding the signal off (the shaded band in the graph)."
               : "Clip Knee: only used with Clip: Soft." },
        { "Output", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamOutputGain,
          "Output Gain: gain after the limiter, on top of Boost. Reduced automatically if it would push the limited peaks over the Ceiling." },
    });
}

void UILimiter::refreshGraphCurve()
{
    // In True Peak mode the real gain computer applies a hard threshold
    // regardless of the Knee parameter's own value (see
    // TruePeakLimiterDsp.hpp's file comment: the guarantee it proves only
    // holds for a hard cut, not a soft-knee blend) -- draw the curve to
    // match what's actually running, a knee of exactly 0, rather than
    // showing a soft knee that would be visually misleading about the
    // real, currently-active behavior. Knee's own Parameter value is left
    // untouched underneath (still round-trips through automation/presets,
    // still takes effect the instant Classic is selected again).
    const bool truePeak = algorithm_ == LimiterAlgorithm::TruePeak;
    const bool soft = paramCache_[kParamClipMode] >= 0.5f;
    const sscomp::LimiterStaging st = sscomp::computeLimiterStaging(
        paramCache_[kParamThreshold], paramCache_[kParamCeiling], paramCache_[kParamOutputGain],
        paramCache_[kParamBoost] >= 0.5f, soft, paramCache_[kParamClipKnee]);
    LimiterGraphWidget::CurveParams cp;
    cp.thresholdDb = paramCache_[kParamThreshold];
    cp.ceilingDb = paramCache_[kParamCeiling];
    cp.kneeDb = truePeak ? 0.0f : paramCache_[kParamKnee];
    cp.ratio = truePeak ? sscomp::limiterRatioValue(2) : sscomp::limiterRatioValue(int(std::lround(paramCache_[kParamRatio])));
    cp.effectiveThresholdDb = float(st.effectiveThresholdDb);
    cp.postGainDb = float(st.postGainDb);
    cp.requestedPostGainDb = float(st.requestedPostGainDb);
    cp.softClip = soft;
    cp.clipKneeDb = paramCache_[kParamClipKnee];
    graph_->setCurveParams(cp);
}

void UILimiter::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UILimiter::onNanoDisplay()
{
    const float w = float(getWidth());
    fillColor(Color(20, 21, 25));
    beginPath();
    rect(0, 0, w, float(getHeight()));
    fill();
}

void UILimiter::parameterChanged(uint32_t index, float value)
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
    case kParamBoost:
        boost_ = value >= 0.5f;
        refreshBoostColors();
        refreshGraphCurve();
        break;
    case kParamThreshold:
    case kParamCeiling:
    case kParamKnee:
    case kParamOutputGain:
    case kParamClipKnee:
        refreshGraphCurve();
        break;
    case kParamClipMode:
        refreshClipModeColors();
        refreshStripFields();
        refreshGraphCurve();
        break;
    case kParamOversampling:
        refreshOversamplingColors();
        break;
    case kParamRatio:
        refreshRatioButton();
        refreshGraphCurve();
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
    case kParamMeterDetector:
    case kParamMeterGainReduction:
    case kParamMeterClip:
        graph_->setLiveLevels(paramCache_[kParamMeterDetector], paramCache_[kParamMeterGainReduction], paramCache_[kParamMeterClip]);
        break;
    default:
        break;
    }
    repaint();
}

void UILimiter::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "presetDir") == 0) { lastPresetDir_ = value; return; }
    if (std::strcmp(key, "algorithm") == 0)
    {
        algorithm_ = clampLimiterAlgorithm(std::atoi(value));
        refreshAlgorithmColors();
        refreshRatioButton();
        refreshStripFields();
        refreshGraphCurve();
        repaint();
        return;
    }
}

bool UILimiter::onCharacterInput(const CharacterInputEvent& ev)
{
    if (!strip_->isEditing()) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { strip_->commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { strip_->appendChar(c); return true; }
    return true;
}

bool UILimiter::onKeyboard(const KeyboardEvent& ev)
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

void UILimiter::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
}

void UILimiter::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".mlim";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    if (presetio::save(p, captureSnapshot(), "MimoMusic Limiter preset"))
        rememberPresetDir(p.c_str());
}

void UILimiter::loadPresetFile(const char* path)
{
    presetio::KeyValues values;
    if (!presetio::load(path, values)) return;
    applySnapshot(values);
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See UICompressorMono.cpp for the full design writeup this is ported
// from (against this plugin's own parameter list/algorithm state).

presetio::KeyValues UILimiter::captureSnapshot() const
{
    presetio::KeyValues values;
    for (size_t i = 0; i < kPresetParamCount; ++i)
        values[kPresetKeys[i]] = double(paramCache_[kPresetParams[i]]);
    // Not a Parameter (see PluginLimiter.hpp's own comment on
    // LimiterAlgorithm), so it isn't in kPresetParams/kPresetKeys above --
    // captured by hand instead. A snapshot's numbers mean something
    // different depending on which algorithm they're paired with (Attack/
    // Knee are inert under True Peak, Threshold means "the exact ceiling"
    // there instead of "the curve's own knee center"), so leaving it out
    // would silently reinterpret this snapshot's numbers under whatever
    // algorithm happens to be selected when it's restored.
    values["algorithm"] = double(int(algorithm_));
    return values;
}

void UILimiter::applySnapshot(const presetio::KeyValues& values)
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

    // Absent in a preset saved before Algorithm existed -- defaults to
    // Classic, this project's own unchanged original behavior. Every A/B/
    // undo-redo snapshot (captureSnapshot(), just above) always carries
    // this key, so the fallback only ever matters for an old preset file.
    const auto algoIt = values.find("algorithm");
    algorithm_ = clampLimiterAlgorithm(algoIt != values.end() ? int(algoIt->second) : 0);
    setState("algorithm", algorithm_ == LimiterAlgorithm::TruePeak ? "1" : "0");
    refreshAlgorithmColors();
    // A Limiter preset has no clipMode key: it keeps the current Clip mode.
    refreshClipModeColors();
    refreshOversamplingColors();
    refreshRatioButton();
    refreshStripFields();

    strip_->repaint();
    refreshGraphCurve();
    bypassed_ = paramCache_[kParamBypass] >= 0.5f;
    refreshBypassColors();
    boost_ = paramCache_[kParamBoost] >= 0.5f;
    refreshBoostColors();
    repaint();
}

void UILimiter::onAbA()
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

void UILimiter::onAbB()
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

void UILimiter::onAbCopy()
{
    const presetio::KeyValues snap = captureSnapshot();
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
}

void UILimiter::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UILimiter::onUndo()
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

void UILimiter::onRedo()
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

void UILimiter::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UILimiter::uiIdle()
{
    tooltip_->tick(); // unconditional -- see TooltipOverlay::tick()'s own comment

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

void UILimiter::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UILimiter::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UILimiter(); }

END_NAMESPACE_DISTRHO
