// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIMbCompressor.hpp"

#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin_mb_compressor/DistrhoPluginInfo.h"
#include "../../plugin_mb_compressor/PluginMbCompressor.hpp" // for ParameterIndex enum
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

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::MultibandGraphWidget;
using DGL_NAMESPACE::ParamFieldSpec;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;

namespace {
constexpr int kDebounceIdleTicks = 3; // same debounce window as the sibling EQ plugin's engine push, see UIEq.cpp

// Field ids for the two ParamStripWidget instances -- opaque to the
// widget itself, just round-tripped through its getter/setter callbacks.
enum GlobalField : uint32_t { kFieldOutputTrim = 0, kFieldSpectrumSmoothMs = 1 };
enum BandField : uint32_t { kFieldThreshold = 0, kFieldRatio = 1, kFieldRatioUp = 5, kFieldAttack = 2, kFieldRelease = 3, kFieldMakeup = 4 };
} // namespace

UIMbCompressor::UIMbCompressor()
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

    lastReport_ = ssfilter::StabilityGuard::analyze(ssfilter::mb::assembleStateSpace(mbModel_, getSampleRate()));

    graph_ = new MultibandGraphWidget(this, this);
    graph_->setSampleRate(getSampleRate());
    graph_->setSpectrumMode(spectrumMode_);
    graph_->setSelectedBand(selectedBand_);
    graph_->setOnSelectionChanged([this](int i) { selectBand(i); });
    graph_->setOnSpectrumAutoscaleChanged([this](bool on) { setState("spectrumAutoscale", on ? "1" : "0"); });

    globalStrip_ = new ParamStripWidget(this);
    globalStrip_->setFields({
        // Labeled "Output Gain" (not "Output Trim") -- this is the global
        // gain applied uniformly to both output channels after every
        // band's compression (see MbCompressorModel::assembleStateSpace()'s
        // outputTrimDb handling), i.e. exactly "a global output gain for
        // all channels" -- the field id/model member keep their original
        // internal name (outputTrimDb/kFieldOutputTrim), only the
        // user-visible label changed, so no preset/state compatibility is
        // affected. Listed last (not first) so it lands on the right side
        // both within the strip and (per layout()'s own repositioning) at
        // the right end of its row.
        { "Spectrum Smooth", "ms", 1.0f, 2000.0f, 150.0f, 10.0f, true, 0, kFieldSpectrumSmoothMs,
          "Spectrum Smooth: how heavily the background spectrum display is time-averaged." },
        { "Output Gain", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kFieldOutputTrim,
          "Output Gain: a global trim applied to both channels after every band's own compression." },
    });
    globalStrip_->setGetter([this](uint32_t id) -> float {
        switch (id)
        {
        case kFieldOutputTrim: return float(mbModel_.outputTrimDb);
        case kFieldSpectrumSmoothMs: return float(spectrumSmoothMs_);
        }
        return 0.0f;
    });
    globalStrip_->setSetter([this](uint32_t id, float v) {
        switch (id)
        {
        case kFieldOutputTrim:
            mbModel_.outputTrimDb = std::max(-24.0, std::min(24.0, double(v)));
            applyModelToEngine();
            break;
        case kFieldSpectrumSmoothMs:
        {
            spectrumSmoothMs_ = std::max(1.0, double(v));
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.6g", spectrumSmoothMs_);
            setState("spectrumSmoothMs", buf);
            break;
        }
        }
    });

    bandStrip_ = new ParamStripWidget(this);
    bandStrip_->setFields({
        { "Threshold", "dB", -60.0f, 0.0f, -20.0f, 0.5f, false, 1, kFieldThreshold,
          "Threshold: the level above which the selected band starts compressing." },
        { "Ratio", ":1", 1.0f, 20.0f, 4.0f, 0.1f, true, 1, kFieldRatio,
          "Ratio: how strongly the selected band is compressed once past the threshold." },
        { "Up Ratio", ":1", 1.0f, 20.0f, 1.0f, 0.1f, true, 1, kFieldRatioUp,
          "Up Ratio: how strongly the selected band is boosted while below the threshold." },
        { "Attack", "ms", 0.1f, 200.0f, 10.0f, 1.0f, true, 2, kFieldAttack,
          "Attack: how quickly the selected band's own compression engages." },
        { "Release", "ms", 5.0f, 2000.0f, 100.0f, 5.0f, true, 1, kFieldRelease,
          "Release: how quickly the selected band's own compression relaxes back." },
        { "Makeup", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kFieldMakeup,
          "Makeup: trim applied to the selected band's own compressed signal, before it's summed back with the others." },
    });
    bandStrip_->setGetter([this](uint32_t id) -> float {
        const ssfilter::MbBand b = getBand(selectedBand_);
        switch (id)
        {
        case kFieldThreshold: return float(b.thresholdDb);
        case kFieldRatio: return float(b.ratio);
        case kFieldRatioUp: return float(b.ratioUp);
        case kFieldAttack: return float(b.attackMs);
        case kFieldRelease: return float(b.releaseMs);
        case kFieldMakeup: return float(b.makeupDb);
        }
        return 0.0f;
    });
    bandStrip_->setSetter([this](uint32_t id, float v) {
        switch (id)
        {
        case kFieldThreshold: setBandThreshold(selectedBand_, v); break;
        case kFieldRatio: setBandRatio(selectedBand_, v); break;
        case kFieldRatioUp: setBandRatioUp(selectedBand_, v); break;
        case kFieldAttack: setBandAttack(selectedBand_, v); break;
        case kFieldRelease: setBandRelease(selectedBand_, v); break;
        case kFieldMakeup: setBandMakeup(selectedBand_, v); break;
        }
    });

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load Multiband Compressor Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "mb_compressor_preset.ssmbjson";
        opts.title = "Save Multiband Compressor Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnExport_ = new SimpleButton(this, "Export to State-Space...");
    btnExport_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Export;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "mb_compressor_exported.ssjson";
        opts.title = "Export Assembled Coefficients (.ssjson)";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnAddBand_ = new SimpleButton(this, "+ Band");
    btnAddBand_->setCallback([this] {
        // Splits whichever band is currently widest (in log-frequency
        // span) at its own geometric-mean frequency -- a reasonable
        // "just give me another band" default without needing to reach
        // for the graph first. Dragging the resulting split afterward
        // shapes it exactly like any other.
        const int n = getBandCount();
        double bestSpan = -1.0;
        double bestLo = 20.0, bestHi = 20000.0;
        for (int k = 0; k < n; ++k)
        {
            const double lo = k == 0 ? 20.0 : getCrossoverHz(k - 1);
            const double hi = k + 1 < n ? getCrossoverHz(k) : 20000.0;
            const double span = std::log(hi) - std::log(lo);
            if (span > bestSpan) { bestSpan = span; bestLo = lo; bestHi = hi; }
        }
        addCrossover(std::sqrt(bestLo * bestHi)); // also selects the new band and pushes the model, see this class's own addCrossover()
    });

    btnDeleteBand_ = new SimpleButton(this, "Delete Band");
    btnDeleteBand_->setCallback([this] { removeSelectedBand(); });

    btnPrevBand_ = new SimpleButton(this, "<");
    btnPrevBand_->setCallback([this] { selectBand(selectedBand_ - 1); });
    btnNextBand_ = new SimpleButton(this, ">");
    btnNextBand_->setCallback([this] { selectBand(selectedBand_ + 1); });

    btnEnabled_ = new SimpleButton(this, "Enabled");
    btnEnabled_->setCallback([this] {
        setBandEnabled(selectedBand_, !getBand(selectedBand_).enabled);
        refreshBandStripLabels();
    });
    btnSolo_ = new SimpleButton(this, "Solo");
    btnSolo_->setCallback([this] {
        // A single spotlight, not a per-band flag -- see
        // MbCompressorModel::soloedBand's own comment. Toggling it off
        // only when *this* band is the one currently soloed matches what
        // the button visibly shows; toggling it on always sets it to the
        // selected band regardless of whatever else was soloed before.
        setBandSolo(selectedBand_, mbModel_.soloedBand != selectedBand_);
        refreshBandStripLabels();
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
    tooltip_->addTarget(btnLoad_, "Load a saved Multiband Compressor preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a Multiband Compressor preset.");
    tooltip_->addTarget(btnExport_, "Export the current resting-gain shape as a plain State-Space filter (.ssjson).");
    tooltip_->addTarget(btnAddBand_, "Add a new crossover band in the widest still-empty gap between existing bands.");
    tooltip_->addTarget(btnDeleteBand_, "Delete the currently selected band.");
    tooltip_->addTarget(btnPrevBand_, "Select the previous band.");
    tooltip_->addTarget(btnNextBand_, "Select the next band.");
    tooltip_->addTarget(btnEnabled_, "Enable or disable the selected band's own compression entirely.");
    tooltip_->addTarget(btnSolo_, "Solo the selected band -- mutes every other band's own contribution.");
    tooltip_->addTarget(btnReset_, "Reset the engine's internal filter state (clears any transient buildup).");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    tooltip_->addFieldTarget(globalStrip_, [this](double x, double y) { return globalStrip_->fieldTooltipAt(x, y); });
    tooltip_->addFieldTarget(bandStrip_, [this](double x, double y) { return bandStrip_->fieldTooltipAt(x, y); });

    refreshBandStripLabels();
    layout();
}

UIMbCompressor::~UIMbCompressor()
{
    // See ../eq/UIEq.cpp's identical destructor for the full reasoning --
    // closing the editor before uiIdle()'s debounce fires would otherwise
    // silently drop the most recent edit.
    if (modelDirty_) pushModelNow();
}

void UIMbCompressor::layout()
{
    // Row 1: every system-wide/whole-instance control (file ops -> Reset/
    // Bypass -> A/B compare -> Undo/Redo), flush against the window's own
    // top edge, compare/history right-aligned. Everything below belongs
    // to this filter specifically -- band add/delete/navigation/toggles,
    // the selected band's own parameter strip, and the global spectrum/
    // output fields -- sitting as close to the graph it affects as the
    // layout allows.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window -- see TooltipOverlay's own file comment.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));
    constexpr float pad = 6.0f;
    constexpr float rowH = 28.0f;
    constexpr float stripH = 46.0f;
    // See UIDynEq.cpp's identical fix: the global-fields row (Output Gain/Spectrum Smooth) needs
    // bandStrip_'s taller row height, not rowH's shorter one --
    // ParamStripWidget draws two stacked lines per field and rowH's 28px
    // left nowhere near enough room for both without overlapping.
    const float toolbarH = rowH + pad + rowH + pad + stripH + pad + stripH + pad;

    float bx = pad, by = pad;
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

    // Row 2: this filter's own band management.
    by += rowH + pad;
    bx = pad;
    place(btnAddBand_, 78);
    place(btnDeleteBand_, 96);
    place(btnPrevBand_, 26);
    bandLabelX_ = bx;
    bandLabelY_ = by;
    bandLabelW_ = 116.0f;
    bx += bandLabelW_;
    place(btnNextBand_, 26);
    place(btnEnabled_, 68);
    place(btnSolo_, 54);

    // Row 3: the selected band's own parameter strip.
    by += rowH + pad;
    bx = pad;
    bandStrip_->setAbsolutePos(int(bx), int(by));
    bandStrip_->setSize(uint(w - 2.0f * pad), uint(stripH));

    // Row 4: this filter's own global fields (spectrum display + output).
    by += stripH + pad;
    bx = pad;

    // Single-line fields (label to the left, value centered, both on one
    // baseline) -- still only need bh's height, so center that shorter box
    // within the now-taller row rather than top-aligning it against
    // globalStrip_'s full height.
    const float smallFieldY = by + (stripH - bh) * 0.5f;
    spectrumModeFieldX_ = bx + 108.0f;
    spectrumModeFieldY_ = smallFieldY;
    spectrumModeFieldW_ = 44.0f;
    spectrumModeFieldH_ = bh;
    bx = spectrumModeFieldX_ + spectrumModeFieldW_ + 20.0f;

    fftSizeFieldX_ = bx + 140.0f;
    fftSizeFieldY_ = smallFieldY;
    fftSizeFieldW_ = 68.0f;
    fftSizeFieldH_ = bh;
    bx = fftSizeFieldX_ + fftSizeFieldW_ + 20.0f;

    crossoverSlopeFieldX_ = bx + 108.0f;
    crossoverSlopeFieldY_ = smallFieldY;
    crossoverSlopeFieldW_ = 72.0f;
    crossoverSlopeFieldH_ = bh;
    bx = crossoverSlopeFieldX_ + crossoverSlopeFieldW_ + 20.0f;

    // Spectrum Smooth/Output Gain last -- the output-facing control (Output
    // Gain, the group's rightmost field per its own setFields() above) now
    // sits at the right end of the row instead of the left.
    globalStrip_->setAbsolutePos(int(bx), int(by));
    globalStrip_->setSize(uint(280.0f), uint(stripH));

    const float contentTop = toolbarH + pad;
    const float contentH = h - contentTop - pad;
    graph_->setAbsolutePos(int(pad), int(contentTop));
    graph_->setSize(uint(w - 2.0f * pad), uint(contentH));
}

void UIMbCompressor::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIMbCompressor::refreshBandStripLabels()
{
    const ssfilter::MbBand b = getBand(selectedBand_);
    const bool soloed = mbModel_.soloedBand == selectedBand_; // a single spotlight, not a per-band flag -- see soloedBand's own comment
    btnEnabled_->setColors(b.enabled ? Color(58, 62, 70) : Color(150, 90, 40),
                            b.enabled ? Color(78, 84, 96) : Color(175, 110, 55),
                            Color(230, 232, 236));
    btnSolo_->setColors(soloed ? Color(180, 160, 40) : Color(58, 62, 70),
                         soloed ? Color(205, 185, 55) : Color(78, 84, 96),
                         soloed ? Color(20, 20, 22) : Color(230, 232, 236));
    repaint();
}

void UIMbCompressor::onNanoDisplay()
{
    const float w = float(getWidth());

    beginPath();
    rect(0, 0, w, float(getHeight()));
    fillColor(Color(18, 19, 23));
    fill();

    // Selected-band readout, between the Prev/Next buttons.
    char bandBuf[32];
    std::snprintf(bandBuf, sizeof(bandBuf), "Band %d of %d", selectedBand_ + 1, getBandCount());
    fontSize(13.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    fillColor(Color(220, 223, 228));
    text(bandLabelX_ + bandLabelW_ * 0.5f, bandLabelY_ + 12.0f, bandBuf, nullptr);

    // "Spectrum Source:" label + its click-to-cycle field -- identical
    // convention/colors to the sibling EQ plugin's UIEq.cpp.
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

    // "Spectrum Resolution:" label + its click-to-cycle field.
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

    // "Crossover Slope:" label + its click-to-cycle field -- same neutral
    // styling as Spectrum Resolution's box above (this isn't a semantic
    // on/off/mode state the way Spectrum Source's colored box is, just a
    // plain value).
    textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
    fillColor(Color(180, 184, 192));
    text(crossoverSlopeFieldX_ - 4.0f, crossoverSlopeFieldY_ + crossoverSlopeFieldH_ * 0.5f, "Crossover Slope:", nullptr);

    beginPath();
    roundedRect(crossoverSlopeFieldX_ + 0.5f, crossoverSlopeFieldY_ + 0.5f, crossoverSlopeFieldW_ - 1.0f, crossoverSlopeFieldH_ - 1.0f, 3.0f);
    fillColor(Color(34, 37, 43));
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();
    fillColor(Color(220, 223, 228));
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(crossoverSlopeFieldX_ + crossoverSlopeFieldW_ * 0.5f, crossoverSlopeFieldY_ + crossoverSlopeFieldH_ * 0.5f,
         ssfilter::mb::lrOrderLabel(mbModel_.lrOrder), nullptr);

    // The rho-stability badge that used to sit here is removed -- see
    // ../eq/UIEq.cpp's identical comment: unreadable behind row 1's
    // buttons after this session's own reorganization, and never
    // actionable on a coefficient-mapper plugin the way it is on Matrix
    // Filter (the only place it's kept). Still computed and acted on
    // (lastReport_, set from applyModelToEngine()), just not displayed.
    if (faultBlinkTicks_ > 0)
    {
        fontSize(12.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(230, 90, 90));
        text(w - 10.0f, 33.0f, "Numerical fault auto-recovered - state was reset", nullptr);
    }
}

void UIMbCompressor::uiIdle()
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
        const std::string nowJson = ssfilter::mb::toJson(mbModel_);
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

    // Spectrum smoothing -- identical mechanism to UIEq::uiIdle()'s own,
    // see that file's comment for the full reasoning (a UI-side-only
    // one-pole filter, applied regardless of spectrumMode_).
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
    }

    // The live per-band gain-reduction/makeup meter moves continuously
    // (program-material-driven, not edit-driven), so the graph is
    // repainted every tick regardless of spectrumMode_ -- the same reason
    // UIDynEq::uiIdle() always repaints its own live curve.
    graph_->repaint();
}

void UIMbCompressor::parameterChanged(uint32_t index, float value)
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
        else if (index >= kParamBandGainBase && index < kParamBandGainBase + uint32_t(ssfilter::kMaxBands))
        {
            graph_->setBandGainDb(int(index - kParamBandGainBase), value);
            graph_->repaint();
        }
        else
        {
            applyMirrorStagingIfComplete(index, value);
        }
        break;
    }
}

void UIMbCompressor::applyMirrorStagingIfComplete(uint32_t index, float value)
{
    // Same index arithmetic as PluginMbCompressor::setParameterValue()'s
    // own mirror branch -- kept in lockstep with it deliberately.
    if (index == kParamBandCount)
        mirrorStagingCount_ = std::max(1, std::min(int(kNumMirroredBands), int(value + 0.5f)));
    else if (index == kParamOutputTrimDb)
        mirrorStagingTrim_ = double(value);
    else if (index == kParamLrOrder)
        mirrorStagingLrOrder_ = ssfilter::mb::clampLrOrder(int(value + 0.5f));
    else if (index == kParamSoloedBand)
        mirrorStagingSoloedBand_ = std::max(-1, std::min(int(kNumMirroredBands) - 1, int(value + (value >= 0 ? 0.5f : -0.5f))));
    else if (index >= kParamCrossoverBase && index < kParamCrossoverBase + (kNumMirroredBands - 1))
        mirrorStagingCrossoverHz_[index - kParamCrossoverBase] = double(value);
    else if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandThresholdBase].thresholdDb = double(value);
    else if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandRatioBase].ratio = double(value);
    else if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandRatioUpBase].ratioUp = double(value);
    else if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandAttackBase].attackMs = double(value);
    else if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandReleaseBase].releaseMs = double(value);
    else if (index >= kParamBandMakeupBase && index < kParamBandMakeupBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandMakeupBase].makeupDb = double(value);
    else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandEnabledBase].enabled = value >= 0.5f;
    else
        return; // not a mirror field at all

    // The last field of the last slot -- always the last to arrive within
    // one delivery burst, see kParamMirrorCommit's own comment
    // (PluginMbCompressor.hpp).
    if (index != kParamBandEnabledBase + kNumMirroredBands - 1) return;
    if (modelDirty_) return; // a newer local edit is still pending its own debounced push

    ssfilter::MbCompressorModel parsed;
    parsed.bands.assign(mirrorStaging_, mirrorStaging_ + mirrorStagingCount_);
    parsed.crossoverHz.assign(mirrorStagingCrossoverHz_, mirrorStagingCrossoverHz_ + std::max(0, mirrorStagingCount_ - 1));
    parsed.outputTrimDb = mirrorStagingTrim_;
    parsed.lrOrder = mirrorStagingLrOrder_;
    parsed.soloedBand = mirrorStagingSoloedBand_;
    mbModel_ = parsed;
    selectedBand_ = std::max(0, std::min(getBandCount() - 1, selectedBand_));
    graph_->setSelectedBand(selectedBand_);
    lastReport_ = ssfilter::StabilityGuard::analyze(ssfilter::mb::assembleStateSpace(mbModel_, getSampleRate()));
    refreshBandStripLabels();
    repaint();
}

void UIMbCompressor::stateChanged(const char* key, const char* value)
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
        return; // the DSP side (PluginMbCompressor::setState()) applies this to the real analyzer independently
    }
    // A/B compare, restored -- see UIEq::stateChanged()'s identical block
    // (gui/eq/UIEq.cpp) for the full reasoning.
    if (std::strcmp(key, "abSlotA") == 0) { abSlotA_ = value; return; }
    if (std::strcmp(key, "abSlotB") == 0) { abSlotB_ = value; return; }
    if (std::strcmp(key, "abHasA") == 0) { abHasA_ = std::atoi(value) != 0; refreshAbColors(); return; }
    if (std::strcmp(key, "abHasB") == 0) { abHasB_ = std::atoi(value) != 0; refreshAbColors(); return; }
    if (std::strcmp(key, "abActive") == 0) { abActive_ = std::atoi(value) != 0 ? AbSlot::B : AbSlot::A; refreshAbColors(); return; }
    // The band/crossover list itself no longer round-trips through a State
    // at all -- see applyMirrorStagingIfComplete() for the read side, fed
    // from parameterChanged() instead. That function's own modelDirty_
    // guard follows the exact same "don't let a stale echo clobber a
    // still-pending edit" discipline the removed code here used to (a
    // real race in which Solo didn't react, or the sound stuck on the
    // wrong band).
}

// ---- model push (debounced) ---------------------------------------------

void UIMbCompressor::applyModelToEngine()
{
    // Deliberately does *not* reset idleTicksSinceEdit_ -- turns this into
    // a periodic-throttle push rather than a wait-for-quiet debounce, same
    // reasoning (and the same click/pop-avoiding coefficient smoothing at
    // the engine level backing it up) as UIEq::applyModelToEngine()'s own
    // identical comment.
    modelDirty_ = true;
    repaint();
}

void UIMbCompressor::pushModelNow()
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::mb::assembleStateSpace(mbModel_, sr);
    lastReport_ = ssfilter::StabilityGuard::analyze(assembled); // report only -- the DSP side re-enforces on its own once kParamMirrorCommit below lands
    pushBandParamMirror();
    // Must toggle, not resend a constant -- see mirrorCommitToggle_'s own
    // comment (UIMbCompressor.hpp) -- and must come after every mirror field above.
    mirrorCommitToggle_ = 1.0f - mirrorCommitToggle_;
    setParameterValue(kParamMirrorCommit, mirrorCommitToggle_);
    pushAbState(); // keeps the *active* A/B slot's own persisted copy in sync with this edit -- see its own comment
    repaint();
}

void UIMbCompressor::pushBandParamMirror()
{
    // See UIEq::pushBandParamMirror()'s identical declaration comment
    // (gui/eq/UIEq.hpp) for why this has to happen from here, not the DSP
    // side alone -- ported unchanged, proven correct end-to-end against a
    // real Ardour session for the stereo EQ before this port.
    const int n = std::min(int(mbModel_.bands.size()), int(kNumMirroredBands));
    setParameterValue(kParamBandCount, float(n));
    setParameterValue(kParamOutputTrimDb, float(mbModel_.outputTrimDb));
    setParameterValue(kParamLrOrder, float(mbModel_.lrOrder));
    setParameterValue(kParamSoloedBand, float(mbModel_.soloedBand));
    const int nx = std::max(0, n - 1);
    for (int i = 0; i < int(kNumMirroredBands) - 1; ++i)
        setParameterValue(kParamCrossoverBase + i, float(i < nx ? mbModel_.crossoverHz[size_t(i)] : 0.0));
    for (int i = 0; i < int(kNumMirroredBands); ++i)
    {
        const ssfilter::MbBand b = (i < n) ? mbModel_.bands[size_t(i)] : ssfilter::MbBand();
        setParameterValue(kParamBandThresholdBase + i, float(b.thresholdDb));
        setParameterValue(kParamBandRatioBase + i, float(b.ratio));
        setParameterValue(kParamBandRatioUpBase + i, float(b.ratioUp));
        setParameterValue(kParamBandAttackBase + i, float(b.attackMs));
        setParameterValue(kParamBandReleaseBase + i, float(b.releaseMs));
        setParameterValue(kParamBandMakeupBase + i, float(b.makeupDb));
        setParameterValue(kParamBandEnabledBase + i, b.enabled ? 1.0f : 0.0f);
    }
}

void UIMbCompressor::selectBand(int index)
{
    const int n = getBandCount();
    if (n <= 0) return;
    selectedBand_ = std::max(0, std::min(n - 1, index));
    graph_->setSelectedBand(selectedBand_);

    // Solo is a single spotlight, not a per-band setting (see
    // MbCompressorModel::soloedBand's own comment) -- if it's currently
    // engaged, moving the selection moves the spotlight with it.
    // selectBand() is the
    // one chokepoint every selection change already funnels through
    // (a direct click, Prev/Next, or the incidental reselection
    // addCrossover()/removeCrossover() do after changing the band count),
    // so handling it here keeps "selectedBand_ and soloedBand always
    // agree while solo is on" true everywhere at once.
    if (mbModel_.soloedBand >= 0 && mbModel_.soloedBand != selectedBand_)
    {
        mbModel_.soloedBand = selectedBand_;
        applyModelToEngine();
    }
    refreshBandStripLabels();
}

void UIMbCompressor::removeSelectedBand()
{
    const int n = getBandCount();
    if (n <= 1) return; // refuse to remove the last band -- there must always be at least one

    // Capture the target *before* calling removeCrossover() -- that call
    // mutates selectedBand_ itself (via its own internal selectBand()
    // call, clamping to the new, smaller band count), so reading
    // selectedBand_ again afterward would silently pick up its already-
    // updated value instead of the one this function meant to compute.
    const int mergedIndex = selectedBand_ > 0 ? selectedBand_ - 1 : 0;
    removeCrossover(selectedBand_ > 0 ? selectedBand_ - 1 : 0); // also pushes the model
    selectBand(mergedIndex);
}

// ---- ssgui::IMbEditorHost -------------------------------------------------

ssfilter::MbBand UIMbCompressor::getBand(int index) const
{
    if (index < 0 || index >= int(mbModel_.bands.size())) return ssfilter::MbBand{};
    return mbModel_.bands[size_t(index)];
}

double UIMbCompressor::getCrossoverHz(int crossoverIndex) const
{
    if (crossoverIndex < 0 || crossoverIndex >= int(mbModel_.crossoverHz.size())) return 1000.0;
    return mbModel_.crossoverHz[size_t(crossoverIndex)];
}

void UIMbCompressor::setCrossoverHz(int crossoverIndex, double freqHz)
{
    ssfilter::mb::setCrossoverHz(mbModel_, crossoverIndex, freqHz);
    applyModelToEngine();
}

void UIMbCompressor::addCrossover(double freqHz)
{
    ssfilter::mb::addCrossover(mbModel_, freqHz);
    selectBand(ssfilter::mb::bandIndexForFreq(mbModel_, freqHz));
    applyModelToEngine();
}

void UIMbCompressor::removeCrossover(int crossoverIndex)
{
    ssfilter::mb::removeCrossover(mbModel_, crossoverIndex);
    selectBand(std::min(selectedBand_, getBandCount() - 1));
    applyModelToEngine();
}

void UIMbCompressor::setBandThreshold(int index, double thresholdDb)
{
    if (index < 0 || index >= int(mbModel_.bands.size()) || !std::isfinite(thresholdDb)) return;
    mbModel_.bands[size_t(index)].thresholdDb = std::max(-60.0, std::min(0.0, thresholdDb));
    applyModelToEngine();
}

void UIMbCompressor::setBandRatio(int index, double ratio)
{
    if (index < 0 || index >= int(mbModel_.bands.size()) || !std::isfinite(ratio)) return;
    mbModel_.bands[size_t(index)].ratio = std::max(1.0, std::min(20.0, ratio));
    applyModelToEngine();
}

void UIMbCompressor::setBandRatioUp(int index, double ratioUp)
{
    if (index < 0 || index >= int(mbModel_.bands.size()) || !std::isfinite(ratioUp)) return;
    mbModel_.bands[size_t(index)].ratioUp = std::max(1.0, std::min(20.0, ratioUp));
    applyModelToEngine();
}

void UIMbCompressor::setBandAttack(int index, double attackMs)
{
    if (index < 0 || index >= int(mbModel_.bands.size()) || !std::isfinite(attackMs)) return;
    mbModel_.bands[size_t(index)].attackMs = std::max(0.1, std::min(200.0, attackMs));
    applyModelToEngine();
}

void UIMbCompressor::setBandRelease(int index, double releaseMs)
{
    if (index < 0 || index >= int(mbModel_.bands.size()) || !std::isfinite(releaseMs)) return;
    mbModel_.bands[size_t(index)].releaseMs = std::max(5.0, std::min(2000.0, releaseMs));
    applyModelToEngine();
}

void UIMbCompressor::setBandMakeup(int index, double makeupDb)
{
    if (index < 0 || index >= int(mbModel_.bands.size()) || !std::isfinite(makeupDb)) return;
    mbModel_.bands[size_t(index)].makeupDb = std::max(-24.0, std::min(24.0, makeupDb));
    applyModelToEngine();
}

void UIMbCompressor::setBandEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(mbModel_.bands.size())) return;
    mbModel_.bands[size_t(index)].enabled = enabled;
    applyModelToEngine();
}

void UIMbCompressor::setBandSolo(int index, bool solo)
{
    if (index < 0 || index >= int(mbModel_.bands.size())) return;
    // A single spotlight (MbCompressorModel::soloedBand), not a per-band
    // flag -- `solo=false` only actually clears it if `index` is the band
    // that's currently soloed (matches the IMbEditorHost interface's
    // bool-per-band shape, kept for minimal disruption since this is its
    // only call site).
    if (solo) mbModel_.soloedBand = index;
    else if (mbModel_.soloedBand == index) mbModel_.soloedBand = -1;
    applyModelToEngine();
}

// ---- keyboard / mouse -----------------------------------------------------

bool UIMbCompressor::onCharacterInput(const CharacterInputEvent& ev)
{
    ParamStripWidget* editing = globalStrip_->isEditing() ? globalStrip_ : (bandStrip_->isEditing() ? bandStrip_ : nullptr);
    if (editing == nullptr) return false;
    editing->appendChar(ev.string[0]);
    return true;
}

bool UIMbCompressor::onKeyboard(const KeyboardEvent& ev)
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

    ParamStripWidget* editing = globalStrip_->isEditing() ? globalStrip_ : (bandStrip_->isEditing() ? bandStrip_ : nullptr);

    if (editing == nullptr)
    {
        // Not editing a text field: Delete/Backspace removes whichever
        // band is currently selected -- mirrors UIEq's identical
        // graph-selection-driven removal.
        if (ev.press && (ev.key == DGL_NAMESPACE::kKeyDelete || ev.key == DGL_NAMESPACE::kKeyBackspace))
        {
            removeSelectedBand();
            return true;
        }
        return false;
    }
    if (!ev.press) return false;

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

bool UIMbCompressor::onMouse(const MouseEvent& rawEv)
{
    const MouseEvent ev = DGL_NAMESPACE::editorLogicalEvent(*this, rawEv); // own hit tests in logical units, see EditorSubWidget.hpp
    const double x = ev.pos.getX(), y = ev.pos.getY();

    if (ev.press)
    {
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

        const bool onCrossoverSlopeField = x >= crossoverSlopeFieldX_ && x < crossoverSlopeFieldX_ + crossoverSlopeFieldW_ &&
                                            y >= crossoverSlopeFieldY_ && y < crossoverSlopeFieldY_ + crossoverSlopeFieldH_;
        if (onCrossoverSlopeField)
        {
            // Unlike Spectrum Source/Resolution above (their own separate
            // DSP-relevant State keys), lrOrder lives directly in
            // mbModel_ and already round-trips through the Parameter
            // mirror like the rest of it -- an edit here is just another
            // model mutation, exactly like a band's own Threshold/Ratio/etc.
            mbModel_.lrOrder = ssfilter::mb::nextLrOrder(mbModel_.lrOrder);
            applyModelToEngine(); // also repaints, see its own comment
            return true;
        }
    }

    return UI::onMouse(rawEv);
}

void UIMbCompressor::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
    else if (op == PendingFileOp::Export) exportStateSpaceFile(filename);
}

void UIMbCompressor::loadPresetFile(const char* path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) return;
    std::ostringstream ss;
    ss << f.rdbuf();

    applyModelSnapshot(ss.str());
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See ../../../compressor's UICompressorMono.cpp for the full design
// writeup this is ported from.

void UIMbCompressor::applyModelSnapshot(const std::string& json)
{
    // Formerly loadPresetFile()'s own body past the file read.
    ssfilter::MbCompressorModel parsed;
    if (!ssfilter::mb::fromJson(json, parsed)) return;

    mbModel_ = parsed;
    selectBand(0);
    applyModelToEngine();
}

void UIMbCompressor::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = ssfilter::mb::toJson(mbModel_); abHasB_ = true;
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applyModelSnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIMbCompressor::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = ssfilter::mb::toJson(mbModel_); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applyModelSnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIMbCompressor::onAbCopy()
{
    const std::string snap = ssfilter::mb::toJson(mbModel_);
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
    pushAbState();
}

void UIMbCompressor::pushAbState()
{
    // See UIEq::pushAbState()'s identical comment (gui/eq/UIEq.cpp).
    if (abActive_ == AbSlot::A) abSlotA_ = ssfilter::mb::toJson(mbModel_);
    else abSlotB_ = ssfilter::mb::toJson(mbModel_);

    setState("abSlotA", abSlotA_.c_str());
    setState("abSlotB", abSlotB_.c_str());
    setState("abHasA", abHasA_ ? "1" : "0");
    setState("abHasB", abHasB_ ? "1" : "0");
    setState("abActive", abActive_ == AbSlot::B ? "1" : "0");
}

void UIMbCompressor::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIMbCompressor::onUndo()
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

void UIMbCompressor::onRedo()
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

void UIMbCompressor::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIMbCompressor::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssmbjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::mb::toJson(mbModel_);
    rememberPresetDir(p.c_str());
}

void UIMbCompressor::exportStateSpaceFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::mb::assembleStateSpace(mbModel_, sr);
    ssfilter::StabilityGuard::enforce(assembled); // write out exactly what the audio engine actually runs

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::Serialization::toJson(assembled);
    rememberPresetDir(p.c_str());
}

void UIMbCompressor::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UIMbCompressor::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIMbCompressor(); }

END_NAMESPACE_DISTRHO
