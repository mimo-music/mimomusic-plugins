// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIEqMono.hpp"

#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin_eq_mono/DistrhoPluginInfo.h"
#include "../../plugin_eq_mono/PluginEqMono.hpp" // for ParameterIndex enum
#include "FileBrowserDialog.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::BandListWidget;
using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::EqGraphWidget;
using DGL_NAMESPACE::SimpleButton;

namespace {
constexpr int kDebounceIdleTicks = 3; // same debounce window as UIEq's own copy -- see EqModel.hpp
}

UIEqMono::UIEqMono()
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

    for (float& v : spectrumOutRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;

    lastReport_ = ssfilter::mono::MonoStabilityGuard::analyze(ssfilter::eq::assembleMonoStateSpace(eqModel_, getSampleRate()));

    graph_ = new EqGraphWidget(this, this);
    graph_->setSampleRate(getSampleRate());
    graph_->setSpectrumMode(spectrumMode_);
    graph_->setOnSelectionChanged([this](int i) { list_->setSelectedBand(i); refreshBandStrip(); repaint(); });
    graph_->setOnGainRangeChanged([this](double r) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", r);
        setState("yRangeDb", buf);
    });
    graph_->setOnSpectrumAutoscaleChanged([this](bool on) { setState("spectrumAutoscale", on ? "1" : "0"); });

    list_ = new BandListWidget(this, this, false); // mono -- no Channel column, see that constructor's own comment
    list_->setOnSelectionChanged([this](int i) { graph_->setSelectedBand(i); refreshBandStrip(); repaint(); });

    bandStrip_ = new ParamStripWidget(this);
    bandStrip_->setGetter([this](uint32_t id) -> float {
        const int sel = list_->getSelectedBand();
        if (sel < 0 || sel >= getBandCount()) return 0.0f;
        const ssfilter::EqBand b = getBand(sel);
        switch (id)
        {
        case 0: return float(b.freqHz);
        case 1: return float(b.gainDb);
        case 2: return float(b.q);
        case 3: return float(b.bandwidthOct);
        }
        return 0.0f;
    });
    bandStrip_->setSetter([this](uint32_t id, float v) {
        const int sel = list_->getSelectedBand();
        if (sel < 0 || sel >= getBandCount()) return;
        switch (id)
        {
        case 0: setBandFreq(sel, v); break;
        case 1: setBandGain(sel, v); break;
        case 2: setBandQ(sel, v); break;
        case 3: setBandBandwidth(sel, v); break;
        }
        repaint();
    });
    refreshBandStrip();

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load EQ Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "eq_preset.sseqjson";
        opts.title = "Save EQ Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnExport_ = new SimpleButton(this, "Export to State-Space...");
    btnExport_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Export;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "eq_exported.ssjson";
        opts.title = "Export Assembled Coefficients (.ssjson)";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnAddBand_ = new SimpleButton(this, "+ Band");
    btnAddBand_->setCallback([this] {
        const int before = getBandCount();
        addBand(1000.0, 0.0);
        if (getBandCount() > before) // select the new band, so its fields are ready to edit
        {
            graph_->setSelectedBand(getBandCount() - 1);
            list_->setSelectedBand(getBandCount() - 1);
            refreshBandStrip();
            repaint();
        }
    });

    btnReset_ = new SimpleButton(this, "Reset State");
    btnReset_->setCallback([this] { setParameterValue(kParamResetTrigger, 1.0f); });

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] {
        bypassed_ = !bypassed_;
        setParameterValue(kParamBypass, bypassed_ ? 1.0f : 0.0f);
        btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                               bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                               Color(230, 232, 236));
        repaint();
    });

    // Spectrum Source (Off/In/Out/Both) is an inline cycle field, not a
    // SimpleButton -- see spectrumModeFieldX_'s declaration comment --
    // handled in onNanoDisplay()/onMouse(), nothing to construct here.

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

    // Hover descriptions -- constructed last so it always paints on top of
    // everything above.
    tooltip_ = new DGL_NAMESPACE::TooltipOverlay(this);
    tooltip_->addTarget(btnLoad_, "Load a saved Parametric EQ Mono preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a Parametric EQ Mono preset.");
    tooltip_->addTarget(btnExport_, "Export the current band shape as a plain State-Space filter (.ssjson).");
    tooltip_->addTarget(btnAddBand_, "Add a new band at 1kHz.");
    tooltip_->addTarget(btnReset_, "Reset the engine's internal filter state (clears any transient buildup).");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    tooltip_->addFieldTarget(bandStrip_, [this](double x, double y) { return bandStrip_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(list_, [this](double x, double y) { return list_->cellTooltipAt(x, y); });

    layout();
}

UIEqMono::~UIEqMono()
{
    // See ../eq/UIEq.cpp's identical destructor for the full reasoning --
    // closing the editor before uiIdle()'s debounce fires would otherwise
    // silently drop the most recent edit.
    if (modelDirty_) pushModelNow();
}

void UIEqMono::layout()
{
    // See UIEq::layout()'s own comments for the full reasoning -- row 1 is
    // every system-wide control (file ops -> Reset/Bypass -> compare ->
    // history), flush against the window's top edge, compare/history
    // right-aligned; row 2 is this filter's own controls (+Band, spectrum/
    // output-trim fields), right above the graph they describe.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window -- see TooltipOverlay's own file comment.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));
    constexpr float pad = 6.0f;
    constexpr float rowH = 28.0f;
    constexpr float toolbarH = 2.0f * rowH + pad;

    float bx = pad;
    float by = pad;
    const float bh = rowH - 4.0f;
    auto place = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx), int(by));
        b->setSize(uint(bw), uint(bh));
        bx += bw + 4.0f;
    };
    place(btnLoad_, 78);
    place(btnSave_, 82);
    place(btnExport_, 168);
    place(btnReset_, 90);
    place(btnBypass_, 72);

    float rx = w - pad;
    auto placeRight = [&](SimpleButton* b, float bw) {
        rx -= bw;
        b->setAbsolutePos(int(rx), int(by));
        b->setSize(uint(bw), uint(bh));
        rx -= 4.0f;
    };
    placeRight(btnRedo_, 56.0f);
    placeRight(btnUndo_, 56.0f);
    placeRight(btnAbCopy_, 56.0f);
    placeRight(btnAbB_, 30.0f);
    placeRight(btnAbA_, 30.0f);

    by += rowH + 2.0f;
    bx = pad;
    place(btnAddBand_, 68);

    spectrumModeFieldX_ = bx + 108.0f;
    spectrumModeFieldY_ = by;
    spectrumModeFieldW_ = 44.0f;
    spectrumModeFieldH_ = bh;
    bx = spectrumModeFieldX_ + spectrumModeFieldW_ + 20.0f;

    smoothFieldX_ = bx + 150.0f;
    smoothFieldY_ = by;
    smoothFieldW_ = 56.0f;
    smoothFieldH_ = bh;
    bx = smoothFieldX_ + smoothFieldW_ + 20.0f;

    fftSizeFieldX_ = bx + 140.0f;
    fftSizeFieldY_ = by;
    fftSizeFieldW_ = 68.0f;
    fftSizeFieldH_ = bh;
    bx = fftSizeFieldX_ + fftSizeFieldW_ + 20.0f;

    // Output Trim last -- the actual output-facing control on this row, so
    // it sits on the right side rather than the left.
    trimFieldX_ = bx + 96.0f;
    trimFieldY_ = by;
    trimFieldW_ = 64.0f;
    trimFieldH_ = bh;

    const float contentTop = toolbarH + pad * 2.0f;
    const float contentH = h - contentTop - pad;
    // The selected band's strip between graph and list: heading + fields.
    constexpr float bandHeaderH = 16.0f, bandStripH = 50.0f;
    const float bandPanelH = bandHeaderH + bandStripH + pad;
    const float graphH = contentH * 0.68f - bandPanelH;
    const float listH = contentH - graphH - pad;

    graph_->setAbsolutePos(int(pad), int(contentTop));
    graph_->setSize(uint(w - 2.0f * pad), uint(graphH));

    bandStripHeaderY_ = contentTop + graphH + pad;
    bandStrip_->setAbsolutePos(int(pad), int(bandStripHeaderY_ + bandHeaderH));
    bandStrip_->setSize(uint(w - 2.0f * pad), uint(bandStripH));

    list_->setAbsolutePos(int(pad), int(contentTop + graphH + bandPanelH + pad));
    list_->setSize(uint(w - 2.0f * pad), uint(listH - bandPanelH));
}

void UIEqMono::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIEqMono::refreshBandStrip()
{
    const int sel = list_->getSelectedBand();
    const bool valid = sel >= 0 && sel < getBandCount();
    const int type = valid ? int(getBand(sel).type) : -1;
    bandStrip_->setVisible(valid);
    if (!valid || (sel == bandStripBand_ && type == bandStripType_) || bandStrip_->isEditing())
        return;
    bandStripBand_ = sel;
    bandStripType_ = type;
    const auto t = ssfilter::EqBandType(type);
    const bool hasGain = ssfilter::eqBandTypeHasGain(t);
    const bool hasBw = ssfilter::eqBandTypeHasBandwidth(t);
    bandStrip_->setFields({
        { "Freq", "Hz", 20.0f, 20000.0f, 1000.0f, 1.05f, true, 0, 0,
          "Freq: the selected band's frequency (drag, double-click to type, scroll; Alt = finer)." },
        { hasGain ? "Gain" : "Gain (unused)", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, 1,
          hasGain ? "Gain: the selected band's boost or cut." : "Gain: not used by this band type." },
        { "Q", "", 0.05f, 100.0f, 1.0f, 1.08f, true, 2, 2,
          "Q: how narrow (high) or wide (low) the selected band is." },
        { hasBw ? "Bandwidth" : "Bandwidth (unused)", "oct", float(ssfilter::kMinBandwidthOct), float(ssfilter::kMaxBandwidthOct), 2.0f, 1.08f, true, 2, 3,
          hasBw ? "Bandwidth: the Band Shelf's width, in octaves." : "Bandwidth: only used by the Band Shelf type." },
    });
}

void UIEqMono::onNanoDisplay()
{
    // Identical to UIEq::onNanoDisplay() -- same layout, same fields, same
    // stability-badge convention, just band count/rho sourced from the
    // mono assembler's report instead of the stereo one.
    const float w = float(getWidth());

    beginPath();
    rect(0, 0, w, float(getHeight()));
    fillColor(Color(18, 19, 23));
    fill();

    // Selected band's heading above its strip -- see UIEq::onNanoDisplay().
    refreshBandStrip();
    fontSize(11.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    {
        const int sel = list_->getSelectedBand();
        if (sel >= 0 && sel < getBandCount())
        {
            char head[64];
            std::snprintf(head, sizeof(head), "Band %d \xC2\xB7 %s", sel + 1, ssfilter::eqBandTypeName(getBand(sel).type));
            fillColor(Color(200, 204, 212));
            text(8.0f, bandStripHeaderY_ + 8.0f, head, nullptr);
        }
        else
        {
            fillColor(Color(120, 124, 132));
            text(8.0f, bandStripHeaderY_ + 8.0f, "Click a band in the graph or the list to edit it here with larger fields.", nullptr);
        }
    }

    fontSize(12.0f);
    textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
    fillColor(Color(180, 184, 192));
    text(trimFieldX_ - 4.0f, trimFieldY_ + trimFieldH_ * 0.5f, "Output Trim (dB):", nullptr);

    const bool editingTrim = editTarget_ == EditTarget::OutputTrim;
    beginPath();
    roundedRect(trimFieldX_ + 0.5f, trimFieldY_ + 0.5f, trimFieldW_ - 1.0f, trimFieldH_ - 1.0f, 3.0f);
    fillColor(editingTrim ? Color(235, 195, 60) : Color(34, 37, 43));
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();
    char buf[32];
    const char* toShow;
    if (editingTrim) toShow = editBuffer_.c_str();
    else { std::snprintf(buf, sizeof(buf), "%+.1f", eqModel_.outputTrimDb); toShow = buf; }
    fillColor(editingTrim ? Color(20, 20, 24) : Color(220, 223, 228));
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(trimFieldX_ + trimFieldW_ * 0.5f, trimFieldY_ + trimFieldH_ * 0.5f, toShow, nullptr);

    // "Spectrum Source:" label + its click-to-cycle field -- see UIEq.cpp's
    // identical block for the full reasoning.
    textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
    fillColor(Color(180, 184, 192));
    text(spectrumModeFieldX_ - 4.0f, spectrumModeFieldY_ + spectrumModeFieldH_ * 0.5f, "Spectrum Source:", nullptr);

    Color modeFill, modeText;
    switch (spectrumMode_)
    {
    case ssfilter::SpectrumMode::Off:    modeFill = Color(46, 49, 55);   modeText = Color(150, 154, 162); break;
    case ssfilter::SpectrumMode::Input:  modeFill = Color(150, 100, 55); modeText = Color(15, 16, 19);    break;
    case ssfilter::SpectrumMode::Output: modeFill = Color(55, 120, 150); modeText = Color(15, 16, 19);    break;
    case ssfilter::SpectrumMode::Both:   modeFill = Color(105, 85, 150); modeText = Color(230, 232, 236); break;
    }
    beginPath();
    roundedRect(spectrumModeFieldX_ + 0.5f, spectrumModeFieldY_ + 0.5f, spectrumModeFieldW_ - 1.0f, spectrumModeFieldH_ - 1.0f, 3.0f);
    fillColor(modeFill);
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();
    fillColor(modeText);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(spectrumModeFieldX_ + spectrumModeFieldW_ * 0.5f, spectrumModeFieldY_ + spectrumModeFieldH_ * 0.5f,
         ssfilter::spectrumModeLabel(spectrumMode_), nullptr);

    textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
    fillColor(Color(180, 184, 192));
    text(smoothFieldX_ - 4.0f, smoothFieldY_ + smoothFieldH_ * 0.5f, "Spectrum Smoothing (ms):", nullptr);

    const bool editingSmooth = editTarget_ == EditTarget::SpectrumSmoothMs;
    beginPath();
    roundedRect(smoothFieldX_ + 0.5f, smoothFieldY_ + 0.5f, smoothFieldW_ - 1.0f, smoothFieldH_ - 1.0f, 3.0f);
    fillColor(editingSmooth ? Color(235, 195, 60) : Color(34, 37, 43));
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();
    const char* smoothShow;
    char smoothBuf[32];
    if (editingSmooth) smoothShow = editBuffer_.c_str();
    else { std::snprintf(smoothBuf, sizeof(smoothBuf), "%.0f", spectrumSmoothMs_); smoothShow = smoothBuf; }
    fillColor(editingSmooth ? Color(20, 20, 24) : Color(220, 223, 228));
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(smoothFieldX_ + smoothFieldW_ * 0.5f, smoothFieldY_ + smoothFieldH_ * 0.5f, smoothShow, nullptr);

    textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
    fillColor(Color(180, 184, 192));
    text(fftSizeFieldX_ - 4.0f, fftSizeFieldY_ + fftSizeFieldH_ * 0.5f, "Spectrum Resolution:", nullptr);

    beginPath();
    roundedRect(fftSizeFieldX_ + 0.5f, fftSizeFieldY_ + 0.5f, fftSizeFieldW_ - 1.0f, fftSizeFieldH_ - 1.0f, 3.0f);
    fillColor(Color(34, 37, 43));
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();
    char fftSizeBuf[16];
    std::snprintf(fftSizeBuf, sizeof(fftSizeBuf), "%d", ssfilter::SpectrumAnalyzer::kFftSizePresets[spectrumFftSizeIndex_]);
    fillColor(Color(220, 223, 228));
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(fftSizeFieldX_ + fftSizeFieldW_ * 0.5f, fftSizeFieldY_ + fftSizeFieldH_ * 0.5f, fftSizeBuf, nullptr);

    // See ../eq/UIEq.cpp's identical comment: the rho-stability badge that
    // used to sit here is removed (unreadable behind row 1's buttons after
    // this session's own reorganization, and never actionable on a
    // coefficient-mapper plugin the way it is on Matrix Filter, the only
    // place it's kept). Still computed and acted on, just not displayed.
    if (faultBlinkTicks_ > 0)
    {
        fontSize(12.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(230, 90, 90));
        text(w - 10.0f, 33.0f, "Numerical fault auto-recovered - state was reset", nullptr);
    }
}

void UIEqMono::uiIdle()
{
    tooltip_->tick(); // unconditional -- see TooltipOverlay::tick()'s own comment

    // See UIEq::uiIdle()'s identical comment/kParamUiHeartbeat's own --
    // must toggle, not resend the same value (see uiHeartbeatToggle_'s).
    uiHeartbeatToggle_ = 1.0f - uiHeartbeatToggle_;
    setParameterValue(kParamUiHeartbeat, uiHeartbeatToggle_);

    if (modelDirty_ && ++idleTicksSinceEdit_ >= kDebounceIdleTicks)
    {
        pushModelNow();
        modelDirty_ = false;
        idleTicksSinceEdit_ = 0;
    }

    // Undo/redo checkpointing -- see ../dyneq/UIDynEq.cpp's identical
    // block for the full reasoning.
    if (!applyingSnapshot_)
    {
        const std::string nowJson = ssfilter::eq::toJson(eqModel_);
        if (nowJson != lastSeenJson_)
        {
            lastSeenJson_ = nowJson;
            undoIdleTicksSinceChange_ = 0;
        }
        else if (undoIdleTicksSinceChange_ < kUndoDebounceTicks && ++undoIdleTicksSinceChange_ >= kUndoDebounceTicks)
        {
            if (undoRedoInitialized_ && nowJson != lastCommittedJson_)
            {
                undoStack_.push_back(lastCommittedJson_);
                if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
                redoStack_.clear();
                refreshUndoRedoColors();
            }
            lastCommittedJson_ = nowJson;
            undoRedoInitialized_ = true;
        }
    }

    if (faultBlinkTicks_ > 0)
    {
        --faultBlinkTicks_;
        repaint();
    }

    // Smoothed unconditionally regardless of spectrumMode_ -- see
    // UIEq.cpp's identical block for why.
    {
        constexpr double kAssumedTickMs = 16.0;
        const double tc = std::max(1.0, spectrumSmoothMs_);
        const double coeff = std::exp(-kAssumedTickMs / tc);
        for (int i = 0; i < ssfilter::SpectrumAnalyzer::kNumBands; ++i)
        {
            spectrumOutSmoothed_[i] = float(coeff * spectrumOutSmoothed_[i] + (1.0 - coeff) * spectrumOutRaw_[i]);
            spectrumInSmoothed_[i]  = float(coeff * spectrumInSmoothed_[i]  + (1.0 - coeff) * spectrumInRaw_[i]);
        }
        graph_->setOutputSpectrumData(spectrumOutSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
        graph_->setInputSpectrumData(spectrumInSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
        if (spectrumMode_ != ssfilter::SpectrumMode::Off) graph_->repaint();
    }
}

void UIEqMono::parameterChanged(uint32_t index, float value)
{
    switch (index)
    {
    case kParamBypass:
        bypassed_ = value >= 0.5f;
        btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                               bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                               Color(230, 232, 236));
        repaint();
        break;
    case kParamFaultFlag:
        if (value >= 0.5f) { faultBlinkTicks_ = 90; repaint(); }
        break;
    default:
        if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            float triple[ssfilter::SpectrumAnalyzer::kBandsPerPacket];
            ssfilter::SpectrumAnalyzer::unpackTriple(value, triple);
            const uint32_t base = (index - kParamSpectrumOutputBase) * ssfilter::SpectrumAnalyzer::kBandsPerPacket;
            for (uint32_t k = 0; k < ssfilter::SpectrumAnalyzer::kBandsPerPacket; ++k) spectrumOutRaw_[base + k] = triple[k];
        }
        else if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            float triple[ssfilter::SpectrumAnalyzer::kBandsPerPacket];
            ssfilter::SpectrumAnalyzer::unpackTriple(value, triple);
            const uint32_t base = (index - kParamSpectrumInputBase) * ssfilter::SpectrumAnalyzer::kBandsPerPacket;
            for (uint32_t k = 0; k < ssfilter::SpectrumAnalyzer::kBandsPerPacket; ++k) spectrumInRaw_[base + k] = triple[k];
        }
        else
        {
            applyMirrorStagingIfComplete(index, value);
        }
        break;
    }
}

void UIEqMono::applyMirrorStagingIfComplete(uint32_t index, float value)
{
    // See UIEq::applyMirrorStagingIfComplete()'s identical block
    // (../eq/UIEq.cpp) for the full reasoning -- kept in lockstep with
    // PluginEqMono::setParameterValue()'s own mirror branch.
    if (index == kParamBandCount)
        mirrorStagingCount_ = std::max(0, std::min(int(kNumMirroredBands), int(value + 0.5f)));
    else if (index == kParamOutputTrimDb)
        mirrorStagingTrim_ = double(value);
    else if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandTypeBase].type = ssfilter::EqBandType(std::max(0, std::min(8, int(value + 0.5f))));
    else if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandFreqBase].freqHz = double(value);
    else if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandGainBase].gainDb = double(value);
    else if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandQBase].q = double(value);
    else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandEnabledBase].enabled = value >= 0.5f;
    else if (index >= kParamBandSlopeBase && index < kParamBandSlopeBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandSlopeBase].slopeStages = std::max(1, std::min(4, int(value + 0.5f)));
    else if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandChannelBase].channel = ssfilter::EqChannel(std::max(0, std::min(2, int(value + 0.5f))));
    else if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandHarmonicGroupBase].harmonicGroup = std::max(0, int(value + 0.5f));
    else if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandHarmonicRatioBase].harmonicRatio = double(value);
    else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandBandwidthBase].bandwidthOct = double(value);
    else if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandSoloBase].solo = value >= 0.5f;
    else
        return; // not a mirror field at all

    if (index != kParamBandSoloBase + kNumMirroredBands - 1) return;
    if (modelDirty_) return; // a newer local edit is still pending its own debounced push

    ssfilter::EqModel parsed;
    parsed.bands.assign(mirrorStaging_, mirrorStaging_ + mirrorStagingCount_);
    parsed.outputTrimDb = mirrorStagingTrim_;
    eqModel_ = parsed;
    lastReport_ = ssfilter::mono::MonoStabilityGuard::analyze(ssfilter::eq::assembleMonoStateSpace(eqModel_, getSampleRate()));
    repaint();
}

void UIEqMono::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "presetDir") == 0) { lastPresetDir_ = value; return; }
    if (std::strcmp(key, "spectrumMode") == 0)
    {
        spectrumMode_ = ssfilter::clampSpectrumMode(std::atoi(value));
        graph_->setSpectrumMode(spectrumMode_);
        repaint();
        return;
    }
    if (std::strcmp(key, "spectrumSmoothMs") == 0)
    {
        // std::from_chars, unlike std::strtod, is locale-independent -- a
        // host that sets a comma-decimal locale must not break parsing.
        double v = 0.0;
        const auto res = std::from_chars(value, value + std::strlen(value), v);
        if (res.ec == std::errc() && v > 0.0) spectrumSmoothMs_ = v;
        return;
    }
    if (std::strcmp(key, "yRangeDb") == 0)
    {
        double v = 0.0;
        const auto res = std::from_chars(value, value + std::strlen(value), v);
        if (res.ec == std::errc() && v > 0.0) graph_->setGainRangeDb(v);
        repaint();
        return;
    }
    if (std::strcmp(key, "spectrumAutoscale") == 0)
    {
        graph_->setSpectrumAutoscale(std::atoi(value) != 0);
        repaint();
        return;
    }
    if (std::strcmp(key, "spectrumFftSize") == 0)
    {
        const int idx = std::atoi(value);
        spectrumFftSizeIndex_ = std::max(0, std::min(ssfilter::SpectrumAnalyzer::kNumSizePresets - 1, idx));
        repaint();
        return; // the DSP side (PluginEqMono::setState()) applies this to the real analyzer independently
    }
    // A/B compare, restored -- see UIEq::stateChanged()'s identical block
    // (gui/eq/UIEq.cpp) for the full reasoning.
    if (std::strcmp(key, "abSlotA") == 0) { abSlotA_ = value; return; }
    if (std::strcmp(key, "abSlotB") == 0) { abSlotB_ = value; return; }
    if (std::strcmp(key, "abHasA") == 0) { abHasA_ = std::atoi(value) != 0; refreshAbColors(); return; }
    if (std::strcmp(key, "abHasB") == 0) { abHasB_ = std::atoi(value) != 0; refreshAbColors(); return; }
    if (std::strcmp(key, "abActive") == 0) { abActive_ = std::atoi(value) != 0 ? AbSlot::B : AbSlot::A; refreshAbColors(); return; }
    // The band list itself no longer round-trips through a State at all --
    // see applyMirrorStagingIfComplete() for the read side, fed from
    // parameterChanged() instead.
}

// ---- model push (debounced) --------------------------------------------

void UIEqMono::applyModelToEngine()
{
    // Same "periodic throttle, not wait-for-quiet" reasoning as UIEq's own
    // copy -- see that method's declaration comment.
    modelDirty_ = true;
    repaint();
}

void UIEqMono::pushModelNow()
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::mono::MonoStateSpaceMatrices assembled = ssfilter::eq::assembleMonoStateSpace(eqModel_, sr);
    lastReport_ = ssfilter::mono::MonoStabilityGuard::analyze(assembled); // report only -- the DSP side re-enforces on its own once kParamMirrorCommit below lands
    pushBandParamMirror();
    // Must toggle, not resend a constant -- see mirrorCommitToggle_'s own
    // comment (UIEqMono.hpp) -- and must come after every mirror field above.
    mirrorCommitToggle_ = 1.0f - mirrorCommitToggle_;
    setParameterValue(kParamMirrorCommit, mirrorCommitToggle_);
    pushAbState(); // keeps the *active* A/B slot's own persisted copy in sync with this edit -- see its own comment
    repaint();
}

void UIEqMono::pushBandParamMirror()
{
    // See UIEq::pushBandParamMirror()'s identical declaration comment
    // (UIEq.hpp) for why this has to happen from here, not the DSP side
    // alone -- ported unchanged, proven correct end-to-end against a real
    // Ardour session for the stereo EQ before this port.
    const int n = std::min(int(eqModel_.bands.size()), int(kNumMirroredBands));
    setParameterValue(kParamBandCount, float(n));
    setParameterValue(kParamOutputTrimDb, float(eqModel_.outputTrimDb));
    for (int i = 0; i < int(kNumMirroredBands); ++i)
    {
        const ssfilter::EqBand b = (i < n) ? eqModel_.bands[size_t(i)] : ssfilter::EqBand();
        setParameterValue(kParamBandTypeBase + i, float(int(b.type)));
        setParameterValue(kParamBandFreqBase + i, float(b.freqHz));
        setParameterValue(kParamBandGainBase + i, float(b.gainDb));
        setParameterValue(kParamBandQBase + i, float(b.q));
        setParameterValue(kParamBandEnabledBase + i, b.enabled ? 1.0f : 0.0f);
        setParameterValue(kParamBandSlopeBase + i, float(b.slopeStages));
        setParameterValue(kParamBandChannelBase + i, float(int(b.channel)));
        setParameterValue(kParamBandHarmonicGroupBase + i, float(b.harmonicGroup));
        setParameterValue(kParamBandHarmonicRatioBase + i, float(b.harmonicRatio));
        setParameterValue(kParamBandBandwidthBase + i, float(b.bandwidthOct));
        setParameterValue(kParamBandSoloBase + i, b.solo ? 1.0f : 0.0f);
    }
}

// ---- ssgui::IEqEditorHost -----------------------------------------------

ssfilter::EqBand UIEqMono::getBand(int index) const
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return ssfilter::EqBand{};
    return eqModel_.bands[size_t(index)];
}

void UIEqMono::setBandFreq(int index, double freqHz)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(freqHz)) return;
    // See ../eq/UIEq.cpp's identical setBandFreq() for why this goes
    // through eq::propagateHarmonicFreq() rather than a direct write.
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::eq::propagateHarmonicFreq(eqModel_, index, freqHz, sr);
    applyModelToEngine();
}

void UIEqMono::setBandBandwidth(int index, double octaves)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(octaves)) return;
    ssfilter::EqBand& b = eqModel_.bands[size_t(index)];
    if (!ssfilter::eqBandTypeHasBandwidth(b.type)) return;
    b.bandwidthOct = std::max(ssfilter::kMinBandwidthOct, std::min(ssfilter::kMaxBandwidthOct, octaves));
    applyModelToEngine();
}

void UIEqMono::setBandGain(int index, double gainDb)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(gainDb)) return;
    eqModel_.bands[size_t(index)].gainDb = std::max(-60.0, std::min(60.0, gainDb));
    applyModelToEngine();
}

void UIEqMono::setBandQ(int index, double q)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(q)) return;
    eqModel_.bands[size_t(index)].q = std::max(0.05, std::min(100.0, q));
    applyModelToEngine();
}

void UIEqMono::setBandType(int index, ssfilter::EqBandType type)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    ssfilter::eq::applyBandType(eqModel_.bands[size_t(index)], type);
    applyModelToEngine();
}

void UIEqMono::setBandSlopeStages(int index, int slopeStages)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].slopeStages = std::max(1, std::min(4, slopeStages));
    applyModelToEngine();
}

void UIEqMono::setBandEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].enabled = enabled;
    applyModelToEngine();
}

void UIEqMono::setBandSolo(int index, bool solo)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].solo = solo;
    applyModelToEngine();
}

void UIEqMono::addBand(double freqHz, double gainDb)
{
    // Same numeric cap as UIEq::addBand() (32 bands) -- for the stereo
    // plugin that's kMaxOrder/4 (4 states/band, two L/R chains); for mono
    // it's really mappers::detail::kMaxChainStates/2 (2 states/band, one
    // SISO chain capped at 64 intermediate states by the cascade builder's
    // own fixed buffer, see CoefficientMapper.hpp) -- which happens to be
    // the identical number, 32, so both plugins offer the same maximum
    // band count even though the mono one gets there at a quarter of the
    // per-band CPU cost (see MonoStateSpaceTypes.hpp's header comment).
    constexpr int kMaxBands = ssfilter::kMaxOrder / 4;
    if (int(eqModel_.bands.size()) >= kMaxBands) return;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::EqBand b;
    b.type = ssfilter::EqBandType::Bell;
    b.freqHz = std::max(20.0, std::min(sr * 0.499, freqHz));
    b.gainDb = std::max(-60.0, std::min(60.0, gainDb));
    b.q = 1.0;
    eqModel_.bands.push_back(b);
    applyModelToEngine();
}

void UIEqMono::removeBand(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands.erase(eqModel_.bands.begin() + index);
    if (editTarget_ == EditTarget::BandField && editBandIndex_ == index) cancelEdit();

    auto fixSelection = [index](int sel) { return sel == index ? -1 : (sel > index ? sel - 1 : sel); };
    graph_->setSelectedBand(fixSelection(graph_->getSelectedBand()));
    list_->setSelectedBand(fixSelection(list_->getSelectedBand()));

    applyModelToEngine();
}

void UIEqMono::duplicateBand(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    constexpr int kMaxBands = ssfilter::kMaxOrder / 4; // same cap as addBand()
    if (int(eqModel_.bands.size()) >= kMaxBands) return;

    const ssfilter::EqBand copy = eqModel_.bands[size_t(index)];
    const int newIndex = index + 1;
    eqModel_.bands.insert(eqModel_.bands.begin() + newIndex, copy);

    if (editTarget_ == EditTarget::BandField && editBandIndex_ >= newIndex) editBandIndex_ += 1;

    graph_->setSelectedBand(newIndex);
    list_->setSelectedBand(newIndex);

    applyModelToEngine();
}

// -- Harmonic linking -- see ../eq/UIEq.cpp's identical block (this
// file's own stereo sibling) for the full design writeup; EqModel's
// harmonic fields and the eq::createHarmonicTwin()/propagateHarmonicFreq()/
// unlinkHarmonic() functions they call are shared as-is, unaffected by
// mono vs. stereo assembly.

void UIEqMono::createHarmonicTwin(int index, double factor)
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    const int newIndex = ssfilter::eq::createHarmonicTwin(eqModel_, index, factor, sr);
    if (newIndex < 0) return;

    if (editTarget_ == EditTarget::BandField && editBandIndex_ >= newIndex) editBandIndex_ += 1;

    graph_->setSelectedBand(newIndex);
    list_->setSelectedBand(newIndex);

    applyModelToEngine();
}

void UIEqMono::unlinkHarmonic(int index)
{
    ssfilter::eq::unlinkHarmonic(eqModel_, index);
    applyModelToEngine();
}

void UIEqMono::beginHarmonicFactorEdit(int sourceIndex)
{
    if (sourceIndex < 0 || sourceIndex >= int(eqModel_.bands.size())) return;
    if (editTarget_ != EditTarget::None) commitEdit();
    editTarget_ = EditTarget::HarmonicFactor;
    pendingHarmonicSourceBand_ = sourceIndex;
    editBuffer_ = "2";
    getWindow().focus();
    repaint();
}

bool UIEqMono::isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const
{
    if (editTarget_ != EditTarget::HarmonicFactor || pendingHarmonicSourceBand_ != sourceIndex) return false;
    liveText = editBuffer_;
    return true;
}

void UIEqMono::commitHarmonicFactorEdit()
{
    commitEdit();
}

void UIEqMono::beginBandTextEdit(int index, ssgui::EqField field, bool clearValue)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    if (editTarget_ != EditTarget::None &&
        !(editTarget_ == EditTarget::BandField && editBandIndex_ == index && editBandField_ == field))
        commitEdit();

    editTarget_ = EditTarget::BandField;
    editBandIndex_ = index;
    editBandField_ = field;

    if (clearValue) { editBuffer_.clear(); }
    else
    {
        const ssfilter::EqBand& b = eqModel_.bands[size_t(index)];
        const double v = field == ssgui::EqField::Freq ? b.freqHz : field == ssgui::EqField::Bandwidth ? b.bandwidthOct
                        : field == ssgui::EqField::Gain ? b.gainDb : b.q;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", v);
        editBuffer_ = buf;
    }

    getWindow().focus();
    repaint();
}

bool UIEqMono::isEditingBandField(int index, ssgui::EqField field, std::string& liveText) const
{
    if (editTarget_ != EditTarget::BandField || editBandIndex_ != index || editBandField_ != field) return false;
    liveText = editBuffer_;
    return true;
}

void UIEqMono::beginTrimTextEdit(bool clearValue)
{
    if (editTarget_ != EditTarget::None && editTarget_ != EditTarget::OutputTrim) commitEdit();
    editTarget_ = EditTarget::OutputTrim;
    if (clearValue) editBuffer_.clear();
    else
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", eqModel_.outputTrimDb);
        editBuffer_ = buf;
    }
    getWindow().focus();
    repaint();
}

void UIEqMono::beginSmoothTextEdit(bool clearValue)
{
    if (editTarget_ != EditTarget::None && editTarget_ != EditTarget::SpectrumSmoothMs) commitEdit();
    editTarget_ = EditTarget::SpectrumSmoothMs;
    if (clearValue) editBuffer_.clear();
    else
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", spectrumSmoothMs_);
        editBuffer_ = buf;
    }
    getWindow().focus();
    repaint();
}

bool UIEqMono::checkInlineDoubleClick(int fieldId)
{
    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400);
    const bool isDoubleClick = fieldId == lastClickInlineField_ && (now - lastClickInlineTime_) < kDoubleClickWindow;
    lastClickInlineField_ = isDoubleClick ? -1 : fieldId;
    lastClickInlineTime_ = now;
    return isDoubleClick;
}

void UIEqMono::commitEdit()
{
    if (editTarget_ == EditTarget::None) return;
    // std::from_chars, unlike std::strtod, is locale-independent -- that
    // matters for typed input too, not just JSON round-trips (a host may
    // set a comma-decimal locale).
    double v = 0.0;
    const auto parseRes = std::from_chars(editBuffer_.data(), editBuffer_.data() + editBuffer_.size(), v);
    const bool valid = parseRes.ec == std::errc() && std::isfinite(v);
    const EditTarget target = editTarget_;
    const int bandIndex = editBandIndex_;
    const ssgui::EqField field = editBandField_;
    const int harmonicSourceBand = pendingHarmonicSourceBand_;
    editTarget_ = EditTarget::None;

    if (target == EditTarget::HarmonicFactor) graph_->endHarmonicFactorEdit();

    if (!valid) { repaint(); return; }

    if (target == EditTarget::BandField)
    {
        switch (field)
        {
        case ssgui::EqField::Freq: setBandFreq(bandIndex, v); break;
        case ssgui::EqField::Gain: setBandGain(bandIndex, v); break;
        case ssgui::EqField::Q:    setBandQ(bandIndex, v);    break;
        case ssgui::EqField::Bandwidth: setBandBandwidth(bandIndex, v); break;
        }
    }
    else if (target == EditTarget::OutputTrim || target == EditTarget::SpectrumSmoothMs)
    {
        setInlineFieldValue(target, v);
    }
    else if (target == EditTarget::HarmonicFactor)
    {
        if (v > 0.0) createHarmonicTwin(harmonicSourceBand, v);
    }
}

void UIEqMono::setInlineFieldValue(EditTarget which, double v)
{
    if (!std::isfinite(v)) return;
    if (which == EditTarget::OutputTrim)
    {
        eqModel_.outputTrimDb = std::max(-24.0, std::min(24.0, v));
        applyModelToEngine();
    }
    else if (which == EditTarget::SpectrumSmoothMs)
    {
        spectrumSmoothMs_ = std::max(1.0, std::min(5000.0, v));
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", spectrumSmoothMs_);
        setState("spectrumSmoothMs", buf);
    }
    repaint();
}

bool UIEqMono::onMotion(const MotionEvent& rawEv)
{
    const MotionEvent ev = DGL_NAMESPACE::editorLogicalEvent(*this, rawEv); // own hit tests in logical units, see EditorSubWidget.hpp
    if (inlinePress_ != EditTarget::None)
    {
        const double dy = inlineDragStartY_ - ev.pos.getY(); // up = increase
        if (!inlineDragging_ && std::fabs(dy) > 3.0)
        {
            inlineDragging_ = true;
            if (editTarget_ == inlinePress_) cancelEdit(); // the drag's live value replaces a half-typed one
        }
        if (inlineDragging_)
        {
            constexpr double kPxFullRange = 200.0; // ParamStripWidget's kDragPixelsForFullRange
            if (inlinePress_ == EditTarget::OutputTrim)
                setInlineFieldValue(EditTarget::OutputTrim, inlineDragStartValue_ + dy / kPxFullRange * 48.0); // -24..+24dB
            else // 1..5000ms: log domain, like every multiplicative field
                setInlineFieldValue(EditTarget::SpectrumSmoothMs, inlineDragStartValue_ * std::exp(dy / kPxFullRange * std::log(5000.0)));
        }
        return true;
    }
    return UI::onMotion(rawEv);
}

bool UIEqMono::onScroll(const ScrollEvent& rawEv)
{
    const ScrollEvent ev = DGL_NAMESPACE::editorLogicalEvent(*this, rawEv); // own hit tests in logical units, see EditorSubWidget.hpp
    const double x = ev.pos.getX(), y = ev.pos.getY();
    const bool onTrimField = x >= trimFieldX_ && x < trimFieldX_ + trimFieldW_ &&
                              y >= trimFieldY_ && y < trimFieldY_ + trimFieldH_;
    const bool onSmoothField = x >= smoothFieldX_ && x < smoothFieldX_ + smoothFieldW_ &&
                                y >= smoothFieldY_ && y < smoothFieldY_ + smoothFieldH_;
    if (onTrimField || onSmoothField)
    {
        const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
        const bool fine = (ev.mod & kModifierAlt) != 0; // Alt = 10x finer, as everywhere in this project
        // Same guard as every other field: scrolling a field that is
        // mid-text-edit leaves it alone.
        if (onTrimField && dir != 0.0 && editTarget_ != EditTarget::OutputTrim)
            setInlineFieldValue(EditTarget::OutputTrim, eqModel_.outputTrimDb + dir * (fine ? 0.05 : 0.5));
        if (onSmoothField && dir != 0.0 && editTarget_ != EditTarget::SpectrumSmoothMs)
        {
            const double factor = fine ? 1.008 : 1.08;
            setInlineFieldValue(EditTarget::SpectrumSmoothMs, dir > 0.0 ? spectrumSmoothMs_ * factor : spectrumSmoothMs_ / factor);
        }
        return true;
    }
    return UI::onScroll(rawEv);
}

void UIEqMono::cancelEdit()
{
    if (editTarget_ == EditTarget::HarmonicFactor) graph_->endHarmonicFactorEdit();
    editTarget_ = EditTarget::None;
    repaint();
}

bool UIEqMono::onCharacterInput(const CharacterInputEvent& ev)
{
    if (bandStrip_->isEditing())
    {
        const char c = ev.string[0];
        if (c == '\r' || c == '\n') bandStrip_->commitEdit();
        else bandStrip_->appendChar(c);
        return true;
    }
    if (editTarget_ == EditTarget::None) return false;
    const char c = ev.string[0];

    if (c == '\r' || c == '\n') { commitEdit(); return true; }

    if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+')
    {
        if (editBuffer_.size() < 24) editBuffer_ += c;
        repaint();
        return true;
    }
    return true;
}

bool UIEqMono::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z/Ctrl+Y -- see ../../compressor's UICompressorMono::onKeyboard()
    // for the full reasoning.
    if (ev.press && (ev.mod & DGL_NAMESPACE::kModifierControl) != 0
        && !(ev.mod & DGL_NAMESPACE::kModifierAlt) && !(ev.mod & DGL_NAMESPACE::kModifierSuper))
    {
        const bool shift = (ev.mod & DGL_NAMESPACE::kModifierShift) != 0;
        if (ev.key == 'z' && !shift) { onUndo(); return true; }
        if (ev.key == 'y' || (ev.key == 'z' && shift)) { onRedo(); return true; }
    }

    if (bandStrip_->isEditing())
    {
        if (!ev.press) return true;
        switch (ev.key)
        {
        case DGL_NAMESPACE::kKeyEnter:
        case DGL_NAMESPACE::kKeyTab:
            bandStrip_->commitEdit();
            break;
        case DGL_NAMESPACE::kKeyEscape:
            bandStrip_->cancelEdit();
            break;
        case DGL_NAMESPACE::kKeyBackspace:
        case DGL_NAMESPACE::kKeyDelete:
            bandStrip_->backspace();
            break;
        default:
            break;
        }
        return true;
    }

    if (editTarget_ == EditTarget::None)
    {
        if (ev.press && (ev.key == DGL_NAMESPACE::kKeyDelete || ev.key == DGL_NAMESPACE::kKeyBackspace))
        {
            const int selected = graph_->getSelectedBand();
            if (selected >= 0)
            {
                removeBand(selected);
                graph_->setSelectedBand(-1);
                list_->setSelectedBand(-1);
                repaint();
                return true;
            }
        }
        return false;
    }
    if (!ev.press) return false;

    switch (ev.key)
    {
    case DGL_NAMESPACE::kKeyEnter:
    case DGL_NAMESPACE::kKeyTab:
        commitEdit();
        return true;
    case DGL_NAMESPACE::kKeyEscape:
        cancelEdit();
        return true;
    case DGL_NAMESPACE::kKeyBackspace:
    case DGL_NAMESPACE::kKeyDelete:
        if (!editBuffer_.empty()) editBuffer_.pop_back();
        repaint();
        return true;
    default:
        return true;
    }
}

bool UIEqMono::onMouse(const MouseEvent& rawEv)
{
    const MouseEvent ev = DGL_NAMESPACE::editorLogicalEvent(*this, rawEv); // own hit tests in logical units, see EditorSubWidget.hpp
    const double x = ev.pos.getX(), y = ev.pos.getY();
    const bool onTrimField = x >= trimFieldX_ && x < trimFieldX_ + trimFieldW_ &&
                              y >= trimFieldY_ && y < trimFieldY_ + trimFieldH_;
    const bool onSmoothField = x >= smoothFieldX_ && x < smoothFieldX_ + smoothFieldW_ &&
                                y >= smoothFieldY_ && y < smoothFieldY_ + smoothFieldH_;

    if (editTarget_ != EditTarget::None && ev.press)
    {
        const bool clickingSameField = (editTarget_ == EditTarget::OutputTrim && onTrimField) ||
                                        (editTarget_ == EditTarget::SpectrumSmoothMs && onSmoothField);
        if (!clickingSameField) commitEdit();
    }

    // Release of an Output Trim / Smoothing press: a drag already applied
    // its value; a plain click (never left the dead zone) opens the text
    // edit, as a press alone used to.
    if (!ev.press && inlinePress_ != EditTarget::None)
    {
        const EditTarget released = inlinePress_;
        const bool wasDragging = inlineDragging_;
        inlinePress_ = EditTarget::None;
        inlineDragging_ = false;
        const bool sameField = released == EditTarget::OutputTrim ? onTrimField : onSmoothField;
        if (!wasDragging && sameField)
        {
            if (released == EditTarget::OutputTrim) beginTrimTextEdit(checkInlineDoubleClick(int(EditTarget::OutputTrim)));
            else beginSmoothTextEdit(checkInlineDoubleClick(int(EditTarget::SpectrumSmoothMs)));
        }
        return true;
    }

    if (ev.press)
    {
        if ((onTrimField || onSmoothField) && ev.button == 1)
        {
            // Decided on release (click) or in onMotion() (drag).
            inlinePress_ = onTrimField ? EditTarget::OutputTrim : EditTarget::SpectrumSmoothMs;
            inlineDragging_ = false;
            inlineDragStartY_ = y;
            inlineDragStartValue_ = onTrimField ? eqModel_.outputTrimDb : spectrumSmoothMs_;
            return true;
        }

        const bool onFftSizeField = x >= fftSizeFieldX_ && x < fftSizeFieldX_ + fftSizeFieldW_ &&
                                     y >= fftSizeFieldY_ && y < fftSizeFieldY_ + fftSizeFieldH_;
        if (onFftSizeField)
        {
            spectrumFftSizeIndex_ = (spectrumFftSizeIndex_ + 1) % ssfilter::SpectrumAnalyzer::kNumSizePresets;
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%d", spectrumFftSizeIndex_);
            setState("spectrumFftSize", buf);
            repaint();
            return true;
        }

        const bool onSpectrumModeField = x >= spectrumModeFieldX_ && x < spectrumModeFieldX_ + spectrumModeFieldW_ &&
                                          y >= spectrumModeFieldY_ && y < spectrumModeFieldY_ + spectrumModeFieldH_;
        if (onSpectrumModeField)
        {
            spectrumMode_ = ssfilter::nextSpectrumMode(spectrumMode_);
            graph_->setSpectrumMode(spectrumMode_);
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%d", int(spectrumMode_));
            setState("spectrumMode", buf);
            repaint();
            return true;
        }
    }

    return UI::onMouse(rawEv);
}

void UIEqMono::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadEqPresetFile(filename);
    else if (op == PendingFileOp::Save) saveEqPresetFile(filename);
    else if (op == PendingFileOp::Export) exportStateSpaceFile(filename);
}

void UIEqMono::loadEqPresetFile(const char* path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) return;
    std::ostringstream ss;
    ss << f.rdbuf();

    applyModelSnapshot(ss.str());
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See ../eq/UIEq.cpp's identical block (this file's own stereo sibling)
// for the full design writeup.

void UIEqMono::applyModelSnapshot(const std::string& json)
{
    ssfilter::EqModel parsed;
    if (!ssfilter::eq::fromJson(json, parsed)) return;
    eqModel_ = parsed;
    applyModelToEngine();
}

void UIEqMono::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = ssfilter::eq::toJson(eqModel_); abHasB_ = true;
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applyModelSnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIEqMono::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = ssfilter::eq::toJson(eqModel_); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applyModelSnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIEqMono::onAbCopy()
{
    const std::string snap = ssfilter::eq::toJson(eqModel_);
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
    pushAbState();
}

void UIEqMono::pushAbState()
{
    // See UIEq::pushAbState()'s identical comment (gui/eq/UIEq.cpp).
    if (abActive_ == AbSlot::A) abSlotA_ = ssfilter::eq::toJson(eqModel_);
    else abSlotB_ = ssfilter::eq::toJson(eqModel_);

    setState("abSlotA", abSlotA_.c_str());
    setState("abSlotB", abSlotB_.c_str());
    setState("abHasA", abHasA_ ? "1" : "0");
    setState("abHasB", abHasB_ ? "1" : "0");
    setState("abActive", abActive_ == AbSlot::B ? "1" : "0");
}

void UIEqMono::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIEqMono::onUndo()
{
    if (undoStack_.empty()) return;
    redoStack_.push_back(lastCommittedJson_);
    if (redoStack_.size() > kMaxUndoDepth) redoStack_.erase(redoStack_.begin());
    const std::string prev = undoStack_.back();
    undoStack_.pop_back();
    applyingSnapshot_ = true;
    applyModelSnapshot(prev);
    applyingSnapshot_ = false;
    lastCommittedJson_ = prev;
    lastSeenJson_ = prev;
    undoIdleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIEqMono::onRedo()
{
    if (redoStack_.empty()) return;
    undoStack_.push_back(lastCommittedJson_);
    if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
    const std::string next = redoStack_.back();
    redoStack_.pop_back();
    applyingSnapshot_ = true;
    applyModelSnapshot(next);
    applyingSnapshot_ = false;
    lastCommittedJson_ = next;
    lastSeenJson_ = next;
    undoIdleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIEqMono::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIEqMono::saveEqPresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".sseqjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::eq::toJson(eqModel_);
    rememberPresetDir(p.c_str());
}

void UIEqMono::exportStateSpaceFile(const char* path)
{
    // Deliberately uses the *stereo* assembler here, not
    // assembleMonoStateSpace() -- eqModel_ (the band list) is the exact
    // same shared type either plugin uses, so running it through
    // ssfilter::eq::assembleStateSpace() (the stereo one) produces an
    // ordinary 2-in/2-out .ssjson with both channels identical, which is
    // exactly what "export to view/edit the raw coefficients" means for a
    // band list a mono engine also plays back -- and StateSpace MIMO (the
    // only plugin able to *import* an .ssjson) is stereo-only, so this is
    // also the only assembled shape it can actually open. The audio
    // engine itself still only ever runs the mono-assembled version (see
    // pushModelNow()); this export exists purely for inspection/hand-
    // editing, same as UIEq's own copy of this method.
    std::string p(path);
    static const char kExt[] = ".ssjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::eq::assembleStateSpace(eqModel_, sr);
    ssfilter::StabilityGuard::enforce(assembled);

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::Serialization::toJson(assembled);
    rememberPresetDir(p.c_str());
}

void UIEqMono::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UIEqMono::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIEqMono(); }

END_NAMESPACE_DISTRHO
