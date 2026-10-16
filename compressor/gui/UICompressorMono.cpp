// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UICompressorMono.hpp"

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

namespace {
// Every automatable knob this preset format round-trips -- kept as one
// list so save/load/the "which keys does a preset carry" question all
// stay in exactly one place.
constexpr uint32_t kPresetParams[] = {
    kParamInputGain, kParamThreshold, kParamRatio, kParamAttack, kParamRelease,
    kParamKnee, kParamOutputGain, kParamBypass,
    kParamSidechainEnable, kParamSidechainGain
};
constexpr const char* kPresetKeys[] = {
    "inputGainDb", "thresholdDb", "ratio", "attackMs", "releaseMs",
    "kneeDb", "outputGainDb", "bypass",
    "sidechainEnable", "sidechainGainDb"
};
constexpr size_t kPresetParamCount = sizeof(kPresetParams) / sizeof(kPresetParams[0]);
static_assert(sizeof(kPresetKeys) / sizeof(kPresetKeys[0]) == kPresetParamCount, "keys/params length mismatch");
}

UICompressorMono::UICompressorMono()
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

    // Defaults matching PluginCompressorMono::initParameter(), shown until
    // the host's first round of parameterChanged() calls arrives.
    paramCache_[kParamInputGain] = 0.0f;
    paramCache_[kParamThreshold] = -18.0f;
    paramCache_[kParamRatio] = 4.0f;
    paramCache_[kParamAttack] = 10.0f;
    paramCache_[kParamRelease] = 150.0f;
    paramCache_[kParamKnee] = 6.0f;
    paramCache_[kParamOutputGain] = 0.0f;
    // Unused -- see kParamMeterCompGainDb's own comment
    // (PluginCompressorMono.hpp) for why the slots stay reserved.
    paramCache_[kParamCompensation] = 0.0f;
    paramCache_[kParamCompensationMeasure] = 0.0f;
    paramCache_[kParamBypass] = 0.0f;
    paramCache_[kParamSidechainEnable] = 0.0f;
    paramCache_[kParamSidechainGain] = 0.0f;
    paramCache_[kParamMeterInputRms] = -120.0f;
    paramCache_[kParamMeterInputPeak] = -120.0f;
    paramCache_[kParamMeterOutputRms] = -120.0f;
    paramCache_[kParamMeterOutputPeak] = -120.0f;
    paramCache_[kParamMeterDetectorDb] = -120.0f;
    paramCache_[kParamMeterGainReductionDb] = 0.0f;
    paramCache_[kParamMeterCompGainDb] = 0.0f;
    paramCache_[kParamMeterCompState] = 0.0f;

    meterIn_ = new LevelMeterWidget(this, "IN", 1);
    meterOut_ = new LevelMeterWidget(this, "OUT", 1);
    graph_ = new CompressorGraphWidget(this);
    graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
    graph_->setThresholdRange(-60.0f, 0.0f); // matches the Threshold field's own range below
    graph_->setOnThresholdChanged([this](float v) { onGraphParamChanged(kParamThreshold, v); });
    graph_->setOnRatioChanged([this](float v) { onGraphParamChanged(kParamRatio, v); });
    graph_->setOnKneeChanged([this](float v) { onGraphParamChanged(kParamKnee, v); });

    strip_ = new ParamStripWidget(this);
    refreshStripFields(); // "SC Gain" only included once Sidechain is actually enabled -- see its own comment
    strip_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    strip_->setSetter([this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
        if (idx == kParamThreshold || idx == kParamRatio || idx == kParamKnee)
            graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
    });

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] { setParameterValue(kParamBypass, bypassed_ ? 0.0f : 1.0f); });
    refreshBypassColors();

    btnRatioDirection_ = new SimpleButton(this, "");
    btnRatioDirection_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRatioDirection_->setCallback([this] {
        ratioUpMode_ = !ratioUpMode_;
        setState("ratioDirection", ratioUpMode_ ? "1" : "0");
        graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
        refreshRatioDirectionLabel();
    });
    refreshRatioDirectionLabel();

    // External sidechain/key input switch -- see kParamSidechainEnable's
    // own comment in PluginCompressorMono.hpp.
    btnSidechain_ = new SimpleButton(this, "");
    btnSidechain_->setCallback([this] { setParameterValue(kParamSidechainEnable, sidechainEnabled_ ? 0.0f : 1.0f); });
    refreshSidechainColors();

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
        opts.defaultName = "preset.mcmp";
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

    // Hover descriptions -- constructed last so it always paints on top
    // of everything above.
    tooltip_ = new DGL_NAMESPACE::TooltipOverlay(this);
    tooltip_->addTarget(btnLoad_, "Load a saved compressor preset (.mcsp) from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a compressor preset (.mcsp).");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed, all metering/detection still runs.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    tooltip_->addTarget(btnRatioDirection_, "Swap which side of the threshold Ratio compresses: Down (loud material) or Up (quiet material).");
    tooltip_->addTarget(btnSidechain_, "Toggle external sidechain: detect from the Sidechain input instead of the main signal.");
    // Hover descriptions for the parameter fields.
    tooltip_->addFieldTarget(strip_, [this](double x, double y) { return strip_->fieldTooltipAt(x, y); });

    layout();
}

void UICompressorMono::layout()
{
    // Row 1: every system-wide/whole-instance control -- Load/Save/
    // Bypass, then A/B compare/Undo/Redo -- flush against the window's
    // own top edge, right-aligned (title text fills the remaining space
    // on the left). Placed right-to-left in reverse workflow order so the
    // resulting on-screen reading order is still left to right: Load,
    // Save, Bypass, A, B, Copy, Undo, Redo. Row 2 is this filter's own
    // curve-shaping controls (Ratio Direction, Sidechain -- they act on
    // the compression curve itself, not the plugin instance), left-aligned
    // right above the graph they affect.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window -- see TooltipOverlay's own file comment.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));

    constexpr float pad = 6.0f;
    constexpr float topBarH = 32.0f;
    constexpr float topBar2H = 26.0f;
    constexpr float stripH = 72.0f;
    constexpr float meterW = 70.0f;
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

    // Row 2: this filter's own curve-shaping controls, left-aligned right
    // above the graph.
    const float row2Y = topBarH + gap;
    float bx2 = pad;
    auto place2 = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx2), int(row2Y));
        b->setSize(uint(bw), uint(topBar2H));
        bx2 += bw + gap;
    };
    place2(btnRatioDirection_, 110.0f);
    place2(btnSidechain_, 160.0f);

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

void UICompressorMono::onGraphParamChanged(uint32_t index, float value)
{
    paramCache_[index] = value;
    setParameterValue(index, value);
    strip_->repaint(); // the strip's own field for this parameter needs to redraw with the new value too
}

void UICompressorMono::refreshBypassColors()
{
    btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                           bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                           Color(230, 232, 236));
}

void UICompressorMono::refreshRatioDirectionLabel()
{
    // The swap button itself -- see kStateRatioDirection's own comment for
    // why this replaced a second, independently-adjustable ratio field.
    btnRatioDirection_->setLabel(ratioUpMode_ ? "Ratio: Up \xE2\x96\xB2" : "Ratio: Down \xE2\x96\xBC"); // UTF-8 up/down triangles
}

void UICompressorMono::refreshStripFields()
{
    // "SC Gain" only does anything once Sidechain is actually driving the
    // detector -- kept out of the strip entirely while it's off, rather
    // than shown-but-inert. ParamStripWidget lays its fields out evenly across
    // its own width every repaint (see its onNanoDisplay()), so the
    // remaining fields simply widen to fill the space the moment this
    // one is removed -- no separate layout/geometry bookkeeping needed
    // here.
    // Tooltips adapt to the mode -- SC Gain's own tooltip only needs to exist while the
    // field itself is shown (Sidechain on), so it needs no separate
    // adaptation; every other field here is static regardless of mode
    // (Mono has no stereo mode/B-bank concept to adapt to).
    std::vector<ParamFieldSpec> fields = {
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
          "Makeup Gain: trim applied to the compressed signal." }, // renamed from "Output Gain"
    };
    if (sidechainEnabled_)
        fields.push_back({ "SC Gain", "dB", float(sscomp::kSidechainGainMinDb), float(sscomp::kSidechainGainMaxDb), 0.0f, 0.5f, false, 1, kParamSidechainGain,
          "Sidechain Gain: trim applied to the external key signal before it reaches the detector, independent of Input Gain." });
    strip_->setFields(std::move(fields));
    repaint();
}

void UICompressorMono::refreshSidechainColors()
{
    // Same green-when-on convention as every other toggle in this file;
    // the label doubles as the readout so it's always visible which
    // signal is actually driving the detector right now.
    btnSidechain_->setColors(sidechainEnabled_ ? Color(70, 150, 90) : Color(58, 62, 70),
                              sidechainEnabled_ ? Color(85, 175, 110) : Color(78, 84, 96),
                              Color(230, 232, 236));
    btnSidechain_->setLabel(sidechainEnabled_ ? "Sidechain: External" : "Sidechain: Internal");
}

void UICompressorMono::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UICompressorMono::onNanoDisplay()
{
    const float w = float(getWidth());
    fillColor(Color(20, 21, 25));
    beginPath();
    rect(0, 0, w, float(getHeight()));
    fill();

    fillColor(Color(210, 214, 222));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(15.0f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    text(8.0f, 18.0f, "MimoMusic Compressor -- Mono", nullptr);
}

void UICompressorMono::parameterChanged(uint32_t index, float value)
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
        graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
        break;
    case kParamOutputGain:
        // Makeup gain is shown in the gain-reduction meter -- see
        // CompressorGraphWidget::drawGrMeterColumn()'s own
        // comment for the marker itself; this plugin has no B column
        // (bDb unused, passed as 0).
        graph_->setMakeupGainDb(paramCache_[kParamOutputGain], 0.0f);
        break;
    case kParamSidechainEnable:
        sidechainEnabled_ = value >= 0.5f;
        refreshSidechainColors();
        refreshStripFields();
        repaint();
        break;
    case kParamMeterInputRms:
    case kParamMeterInputPeak:
        meterIn_->setChannel(0, paramCache_[kParamMeterInputRms], paramCache_[kParamMeterInputPeak]);
        break;
    case kParamMeterOutputRms:
    case kParamMeterOutputPeak:
        meterOut_->setChannel(0, paramCache_[kParamMeterOutputRms], paramCache_[kParamMeterOutputPeak]);
        break;
    case kParamMeterDetectorDb:
    case kParamMeterGainReductionDb:
        graph_->setLiveLevels(paramCache_[kParamMeterDetectorDb], paramCache_[kParamMeterGainReductionDb]);
        break;
    default:
        break;
    }
    repaint();
}

void UICompressorMono::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "presetDir") == 0) { lastPresetDir_ = value; return; }
    if (std::strcmp(key, "ratioDirection") == 0)
    {
        ratioUpMode_ = std::atoi(value) != 0;
        refreshRatioDirectionLabel();
        graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
        repaint();
        return;
    }
}

bool UICompressorMono::onCharacterInput(const CharacterInputEvent& ev)
{
    if (!strip_->isEditing()) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { strip_->commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { strip_->appendChar(c); return true; }
    return true;
}

bool UICompressorMono::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z/Ctrl+Y (undo/redo) checked first and unconditionally -- not
    // gated on strip_->isEditing() the way the rest of this function is,
    // since these apply to the plugin's whole configuration, not a field
    // mid-edit. Consumed (returns true) whether or not there's actually
    // anything to undo/redo: this plugin's own window is a real embedded
    // one that receives raw keyboard events once focused, so swallowing
    // Ctrl+Z here is what keeps it from also reaching the host's own
    // undo -- "if plugin is active this combination has to be stolen or
    // deactivated" was the explicit ask. Ctrl+Shift+Z is accepted as a
    // second redo spelling alongside Ctrl+Y (common on macOS/some DAWs).
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
        return true; // swallow everything else while editing, same convention as UIStateSpace
    }
}

void UICompressorMono::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
}

void UICompressorMono::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".mcmp";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    if (presetio::save(p, captureSnapshot(), "MimoMusic Compressor Mono preset"))
        rememberPresetDir(p.c_str());
}

void UICompressorMono::loadPresetFile(const char* path)
{
    presetio::KeyValues values;
    if (!presetio::load(path, values)) return;
    applySnapshot(values);
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------

presetio::KeyValues UICompressorMono::captureSnapshot() const
{
    presetio::KeyValues values;
    for (size_t i = 0; i < kPresetParamCount; ++i)
        values[kPresetKeys[i]] = double(paramCache_[kPresetParams[i]]);
    values["ratioDirection"] = ratioUpMode_ ? 1.0 : 0.0; // a State, not a Parameter -- see kStateRatioDirection's own comment
    return values;
}

void UICompressorMono::applySnapshot(const presetio::KeyValues& values)
{
    // Formerly loadPresetFile()'s own body verbatim -- factored out so
    // A/B and undo/redo (which never touch a file) can reuse the exact
    // same "apply a whole captured configuration" logic Load already
    // proved correct, right down to the same manual mirror-variable sync
    // (bypassed_/sidechainEnabled_/etc.) rather than waiting on an
    // async parameterChanged() echo from the host that may lag a frame or
    // more behind -- see this project's own recurring "make edits visible
    // immediately" convention.
    for (size_t i = 0; i < kPresetParamCount; ++i)
    {
        const auto it = values.find(kPresetKeys[i]);
        if (it == values.end()) continue; // snapshot doesn't mention this key -- leave the current value alone
        const uint32_t idx = kPresetParams[i];
        const float v = float(it->second);
        paramCache_[idx] = v;
        setParameterValue(idx, v);
    }

    const auto dirIt = values.find("ratioDirection");
    if (dirIt != values.end())
    {
        ratioUpMode_ = dirIt->second != 0.0;
        char buf[4];
        std::snprintf(buf, sizeof(buf), "%d", ratioUpMode_ ? 1 : 0);
        setState("ratioDirection", buf);
        refreshRatioDirectionLabel();
    }

    strip_->repaint();
    graph_->setCurveParams(paramCache_[kParamThreshold], paramCache_[kParamRatio], ratioUpMode_, paramCache_[kParamKnee]);
    graph_->setMakeupGainDb(paramCache_[kParamOutputGain], 0.0f);
    bypassed_ = paramCache_[kParamBypass] >= 0.5f;
    refreshBypassColors();
    sidechainEnabled_ = paramCache_[kParamSidechainEnable] >= 0.5f;
    refreshSidechainColors();
    refreshStripFields();
    repaint();
}

void UICompressorMono::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = captureSnapshot(); abHasB_ = true; // save what we're leaving
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applySnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; } // first-ever switch: A starts as a copy of the current settings
    applyingSnapshot_ = false;
    refreshAbColors();
}

void UICompressorMono::onAbB()
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

void UICompressorMono::onAbCopy()
{
    const presetio::KeyValues snap = captureSnapshot();
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
}

void UICompressorMono::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UICompressorMono::onUndo()
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
    idleTicksSinceChange_ = kUndoDebounceTicks; // already settled -- don't re-checkpoint the restore itself
    refreshUndoRedoColors();
}

void UICompressorMono::onRedo()
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

void UICompressorMono::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UICompressorMono::uiIdle()
{
    tooltip_->tick(); // unconditional -- see TooltipOverlay::tick()'s own comment

    if (applyingSnapshot_) return; // don't record our own A/B switch or undo/redo restore as a fresh edit

    const presetio::KeyValues now = captureSnapshot();
    if (now != lastSeenSnapshot_)
    {
        lastSeenSnapshot_ = now;
        idleTicksSinceChange_ = 0;
        return;
    }
    if (idleTicksSinceChange_ >= kUndoDebounceTicks) return; // already settled and committed
    if (++idleTicksSinceChange_ < kUndoDebounceTicks) return;

    // Settled: this is a genuine committed change (or, the very first
    // time through, just the host's real starting values finally landing
    // -- undoRedoInitialized_ tells those two apart so the plugin opening
    // never itself counts as an undo-able "edit").
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

void UICompressorMono::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UICompressorMono::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UICompressorMono(); }

END_NAMESPACE_DISTRHO
