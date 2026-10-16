// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIEq.hpp"

#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin_eq/DistrhoPluginInfo.h"
#include "../../plugin_eq/PluginEq.hpp" // for ParameterIndex enum
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
constexpr int kDebounceIdleTicks = 3; // same debounce window as the base plugin's Bode plot -- see EqModel.hpp
}

UIEq::UIEq()
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

    for (float& v : spectrumOutRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;

    lastReport_ = ssfilter::StabilityGuard::analyze(ssfilter::eq::assembleStateSpace(eqModel_, getSampleRate()));

    graph_ = new EqGraphWidget(this, this);
    graph_->setSampleRate(getSampleRate());
    graph_->setSpectrumMode(spectrumMode_);
    graph_->setOnSelectionChanged([this](int i) { selectBand(i); });
    graph_->setOnGainRangeChanged([this](double r) {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", r);
        setState("yRangeDb", buf);
    });
    graph_->setOnSpectrumAutoscaleChanged([this](bool on) { setState("spectrumAutoscale", on ? "1" : "0"); });

    list_ = new BandListWidget(this, this, true); // stereo -- shows the Channel column, see that constructor's own comment
    list_->setOnSelectionChanged([this](int i) { selectBand(i); });

    bandStrip_ = new ParamStripWidget(this);
    bandStrip_->setGetter([this](uint32_t id) -> float {
        if (selectedBand_ < 0 || selectedBand_ >= getBandCount()) return 0.0f;
        const ssfilter::EqBand b = getBand(selectedBand_);
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
        if (selectedBand_ < 0 || selectedBand_ >= getBandCount()) return;
        switch (id)
        {
        case 0: setBandFreq(selectedBand_, v); break;
        case 1: setBandGain(selectedBand_, v); break;
        case 2: setBandQ(selectedBand_, v); break;
        case 3: setBandBandwidth(selectedBand_, v); break;
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
        if (getBandCount() > before) selectBand(getBandCount() - 1); // ready to edit in the band strip
    });

    // Audition -- same idea, same button, as the sibling Dynamic EQ plugin's
    // own: preview selectedBand_'s isolated frequency region instead of
    // the real output, for finding/tuning problem frequencies by ear.
    // Grouped with +Band rather than the top toolbar's Bypass since it
    // only ever acts on whichever band is currently selected --
    // refreshAuditionColors() grays it out (and forces Audition back off) the
    // moment there's no band to preview.
    btnAudition_ = new SimpleButton(this, "Audition");
    btnAudition_->setCallback([this] {
        if (selectedBand_ < 0) return;
        auditioning_ = !auditioning_;
        setParameterValue(kParamAudition, auditioning_ ? 1.0f : 0.0f);
        refreshAuditionColors();
    });
    refreshAuditionColors();

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
    tooltip_->addTarget(btnLoad_, "Load a saved Parametric EQ preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a Parametric EQ preset.");
    tooltip_->addTarget(btnExport_, "Export the current band shape as a plain State-Space filter (.ssjson).");
    tooltip_->addTarget(btnAddBand_, "Add a new band at 1kHz.");
    tooltip_->addTarget(btnAudition_, "Audition the selected band's own isolated frequency region, for finding problem frequencies by ear.");
    tooltip_->addTarget(btnReset_, "Reset the engine's internal filter state (clears any transient buildup).");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    // Column hover descriptions -- one per BandListWidget column, via its
    // own cellTooltipAt().
    tooltip_->addFieldTarget(list_, [this](double x, double y) { return list_->cellTooltipAt(x, y); });
    tooltip_->addFieldTarget(bandStrip_, [this](double x, double y) { return bandStrip_->fieldTooltipAt(x, y); });

    layout();
}

UIEq::~UIEq()
{
    // The engine push is deliberately debounced (see applyModelToEngine()'s
    // own comment) -- kDebounceIdleTicks worth of uiIdle() calls, normally
    // well under a second. But nothing previously flushed that pending
    // push if the editor window closed (Ardour genuinely destroys this UI
    // object on close, see DistrhoUILV2.cpp's lv2ui_cleanup()) before
    // uiIdle() got that many more ticks -- silently dropping the most
    // recent edit(s): eqModel_ already showed the change on screen right
    // up until the window closed, but the DSP engine never received it, so
    // the audio never actually changed. A real gap, not a hypothetical one
    // -- closing the editor right after a quick tweak is an entirely
    // ordinary gesture. Flushing unconditionally here (pushModelNow() is a
    // no-op-safe, idempotent send either way) closes it.
    if (modelDirty_) pushModelNow();
}

void UIEq::layout()
{
    // Two toolbar rows: row 1 is every system-wide/whole-instance control
    // (file ops, Reset/Bypass, A/B compare, Undo/Redo), flush against the
    // window's own top edge, ordered left to right by workflow (file i/o
    // -> reset/bypass -> compare -> history) with the compare/history
    // group pinned to the right so it reads as "always in the same
    // corner" across every plugin in this family. Row 2 is everything
    // that actually belongs to *this* filter -- +Band and the spectrum/
    // output-trim fields -- sitting directly above the graph it affects,
    // not lumped in with the system-wide row above it.
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

    // Compare + history, right-aligned on the same row -- placed
    // right-to-left so the resulting on-screen reading order is still
    // left-to-right: A, B, Copy, Undo, Redo.
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

    // Row 2: this filter's own controls -- +Band, then the spectrum/
    // output fields, right above the graph they describe.
    by += rowH + 2.0f;
    bx = pad;
    place(btnAddBand_, 68);
    place(btnAudition_, 68);

    spectrumModeFieldX_ = bx + 108.0f; // room for the "Spectrum Source:" label
    spectrumModeFieldY_ = by;
    spectrumModeFieldW_ = 44.0f; // wide enough for "Both"
    spectrumModeFieldH_ = bh;
    bx = spectrumModeFieldX_ + spectrumModeFieldW_ + 20.0f;

    smoothFieldX_ = bx + 150.0f; // room for the "Spectrum Smoothing (ms):" label
    smoothFieldY_ = by;
    smoothFieldW_ = 56.0f;
    smoothFieldH_ = bh;
    bx = smoothFieldX_ + smoothFieldW_ + 20.0f;

    fftSizeFieldX_ = bx + 140.0f; // room for the "Spectrum Resolution:" label
    fftSizeFieldY_ = by;
    fftSizeFieldW_ = 68.0f; // wide enough for "16384"
    fftSizeFieldH_ = bh;
    bx = fftSizeFieldX_ + fftSizeFieldW_ + 20.0f;

    // Output Trim last -- the actual output-facing control on this row, so
    // it sits on the right side rather than the left.
    trimFieldX_ = bx + 96.0f; // room for the "Output Trim:" label, drawn in onNanoDisplay
    trimFieldY_ = by;
    trimFieldW_ = 64.0f;
    trimFieldH_ = bh;

    const float contentTop = toolbarH + pad * 2.0f;
    const float contentH = h - contentTop - pad;
    // The selected band's strip (see refreshBandStrip()) between graph and
    // list: a heading line, then the fields.
    constexpr float bandHeaderH = 16.0f, bandStripH = 50.0f;
    const float bandPanelH = bandHeaderH + bandStripH + pad;
    const float graphH = contentH * 0.68f - bandPanelH;
    const float listH = contentH - graphH - bandPanelH - pad;

    graph_->setAbsolutePos(int(pad), int(contentTop));
    graph_->setSize(uint(w - 2.0f * pad), uint(graphH));

    bandStripHeaderY_ = contentTop + graphH + pad;
    bandStrip_->setAbsolutePos(int(pad), int(bandStripHeaderY_ + bandHeaderH));
    bandStrip_->setSize(uint(w - 2.0f * pad), uint(bandStripH));

    list_->setAbsolutePos(int(pad), int(contentTop + graphH + bandPanelH + pad));
    list_->setSize(uint(w - 2.0f * pad), uint(listH));
}

void UIEq::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIEq::onNanoDisplay()
{
    const float w = float(getWidth());

    beginPath();
    rect(0, 0, w, float(getHeight()));
    fillColor(Color(18, 19, 23));
    fill();

    // Selected band's heading above its strip (a type change from the list
    // or graph shows up here and in the strip's "(unused)" marks).
    refreshBandStrip();
    fontSize(11.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    if (selectedBand_ >= 0 && selectedBand_ < getBandCount())
    {
        char head[64];
        std::snprintf(head, sizeof(head), "Band %d \xC2\xB7 %s", selectedBand_ + 1, ssfilter::eqBandTypeName(getBand(selectedBand_).type));
        fillColor(Color(200, 204, 212));
        text(8.0f, bandStripHeaderY_ + 8.0f, head, nullptr);
    }
    else
    {
        fillColor(Color(120, 124, 132));
        text(8.0f, bandStripHeaderY_ + 8.0f, "Click a band in the graph or the list to edit it here with larger fields.", nullptr);
    }

    // "Output Trim:" label + its editable field.
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

    // "Spectrum Source:" label + its click-to-cycle field -- Off/In/Out/
    // Both (ssfilter::SpectrumMode), see spectrumModeFieldX_'s declaration
    // comment. Colored per mode (roughly matching EqGraphWidget's own
    // pre-EQ/post-EQ trace colors) so the current mode reads at a glance,
    // not just from the three-or-four-letter text.
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

    // "Spectrum Smoothing (ms):" label + its editable field -- the "how
    // fast/smooth" control for the background spectrum, applied on the UI
    // side only (see uiIdle()).
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

    // "Spectrum Resolution:" label + its click-to-cycle field -- FFT size
    // preset, trading finer low-frequency detail for more buffering
    // latency (not really CPU, despite how the request that added this
    // was originally framed -- see SpectrumAnalyzer.hpp's file comment).
    // A cycle control, not a free-typed field, since the DSP side only
    // actually supports this fixed set of presets.
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

    // The rho-stability badge that used to sit here was removed -- the
    // row-1 button reorganization put it directly behind Load/Save/
    // Export/Reset/Bypass/A/B/Copy/Undo/Redo, unreadable and not worth a
    // reflow just to keep visible on every derived plugin. Still computed
    // (lastReport_, set from applyModelToEngine()/StabilityGuard::enforce())
    // and still *acted on* -- clamping still happens -- just no longer
    // displayed here. Kept on ../UIStateSpace.cpp (Matrix Filter) only,
    // where directly-editable matrices make it a genuine debugging aid;
    // every plugin built as a coefficient mapper on top of that core
    // (this one included) already stays stable by construction from its
    // own model's own bounds, so seeing rho here was never actionable the
    // way it is there.
    if (faultBlinkTicks_ > 0)
    {
        fontSize(12.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(230, 90, 90));
        text(w - 10.0f, 33.0f, "Numerical fault auto-recovered - state was reset", nullptr);
    }
}

void UIEq::uiIdle()
{
    tooltip_->tick(); // unconditional -- see TooltipOverlay::tick()'s own comment

    // A "the editor is currently open" heartbeat -- see kParamUiHeartbeat's
    // own comment in PluginEq.hpp. uiIdle() only ever runs while this UI
    // instance exists, so simply reaching this line at all is the entire
    // signal; the exact value written is meaningless, but it must
    // *toggle* -- see uiHeartbeatToggle_'s own comment for why a constant
    // resent value silently never arrived at all past the first tick.
    uiHeartbeatToggle_ = 1.0f - uiHeartbeatToggle_;
    setParameterValue(kParamUiHeartbeat, uiHeartbeatToggle_);

    if (modelDirty_ && ++idleTicksSinceEdit_ >= kDebounceIdleTicks)
    {
        pushModelNow();
        modelDirty_ = false;
        idleTicksSinceEdit_ = 0;
    }

    // Undo/redo checkpointing -- a longer, independent debounce window
    // than kDebounceIdleTicks's own -- see ../dyneq/UIDynEq.cpp's
    // identical block for the full reasoning.
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

    // Spectrum smoothing: a one-pole exponential filter per band, toward
    // whatever parameterChanged() most recently received. Applied here
    // (not on the DSP side) so "how fast/smooth" is a pure UI preference
    // with no DSP-side parameter/state round-trip -- see this member's
    // declaration comment. uiIdle()'s call rate isn't a fixed, guaranteed
    // constant across hosts, but it's the same "per UI frame" cadence
    // repaint()/the Bode-plot debounce already rely on elsewhere in this
    // project, and a smoothing *time constant* (as opposed to a fixed
    // per-tick step) degrades gracefully if that rate varies.
    // Smoothed unconditionally, regardless of spectrumMode_ -- see this
    // pair's declaration comment for why (trivial cost either way; only
    // the DSP-side analyzers actually skip work for a hidden trace).
    {
        constexpr double kAssumedTickMs = 16.0; // ~60Hz, see comment above
        const double tc = std::max(1.0, spectrumSmoothMs_);
        const double coeff = std::exp(-kAssumedTickMs / tc);
        for (int i = 0; i < ssfilter::SpectrumAnalyzer::kNumBands; ++i)
        {
            spectrumOutSmoothed_[i] = float(coeff * spectrumOutSmoothed_[i] + (1.0 - coeff) * spectrumOutRaw_[i]);
            spectrumInSmoothed_[i]  = float(coeff * spectrumInSmoothed_[i]  + (1.0 - coeff) * spectrumInRaw_[i]);
            spectrumOutLSmoothed_[i] = float(coeff * spectrumOutLSmoothed_[i] + (1.0 - coeff) * spectrumOutLRaw_[i]);
            spectrumOutRSmoothed_[i] = float(coeff * spectrumOutRSmoothed_[i] + (1.0 - coeff) * spectrumOutRRaw_[i]);
        }
        graph_->setOutputSpectrumData(spectrumOutSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
        graph_->setInputSpectrumData(spectrumInSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
        graph_->setOutputSpectrumDataL(spectrumOutLSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
        graph_->setOutputSpectrumDataR(spectrumOutRSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
        if (spectrumMode_ != ssfilter::SpectrumMode::Off) graph_->repaint();
    }
}

void UIEq::parameterChanged(uint32_t index, float value)
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
    case kParamAudition:
        auditioning_ = value >= 0.5f;
        refreshAuditionColors();
        break;
    default:
        // Each Parameter now carries kBandsPerPacket packed real bands
        // (see SpectrumAnalyzer.hpp's own comment) -- unpack once here,
        // right at the boundary, so every downstream consumer of
        // spectrumOutRaw_[]/etc. (the smoothing above, the graph widget)
        // keeps reading one plain dB value per real band, completely
        // unaware packing exists at all.
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
        else if (index >= kParamSpectrumOutputLBase && index < kParamSpectrumOutputLBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            float triple[ssfilter::SpectrumAnalyzer::kBandsPerPacket];
            ssfilter::SpectrumAnalyzer::unpackTriple(value, triple);
            const uint32_t base = (index - kParamSpectrumOutputLBase) * ssfilter::SpectrumAnalyzer::kBandsPerPacket;
            for (uint32_t k = 0; k < ssfilter::SpectrumAnalyzer::kBandsPerPacket; ++k) spectrumOutLRaw_[base + k] = triple[k];
        }
        else if (index >= kParamSpectrumOutputRBase && index < kParamSpectrumOutputRBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
        {
            float triple[ssfilter::SpectrumAnalyzer::kBandsPerPacket];
            ssfilter::SpectrumAnalyzer::unpackTriple(value, triple);
            const uint32_t base = (index - kParamSpectrumOutputRBase) * ssfilter::SpectrumAnalyzer::kBandsPerPacket;
            for (uint32_t k = 0; k < ssfilter::SpectrumAnalyzer::kBandsPerPacket; ++k) spectrumOutRRaw_[base + k] = triple[k];
        }
        else
        {
            applyMirrorStagingIfComplete(index, value);
        }
        break;
    }
}

void UIEq::applyMirrorStagingIfComplete(uint32_t index, float value)
{
    // Same index arithmetic as PluginEq::setParameterValue()'s own mirror
    // branch -- kept in lockstep with it deliberately (see that function's
    // own comment).
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
    else if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandCrossFeedBase].crossFeed = ssfilter::EqCrossFeed(std::max(0, std::min(2, int(value + 0.5f))));
    else if (index == kParamCrossFeedGainLtoRDb)
        mirrorStagingXfeedLtoR_ = double(value);
    else if (index == kParamCrossFeedGainRtoLDb)
        mirrorStagingXfeedRtoL_ = double(value);
    else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandBandwidthBase].bandwidthOct = double(value);
    else if (index >= kParamBandSoloBase && index < kParamBandSoloBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandSoloBase].solo = value >= 0.5f;
    else
        return; // not a mirror field at all

    // The last field of the last slot -- see this function's own
    // declaration comment for why this index is always the last to arrive
    // within one delivery burst.
    if (index != kParamBandSoloBase + kNumMirroredBands - 1) return;

    // A newer local edit is still pending its own debounced push -- same
    // guard the old "eqmodel" State restore used to apply, for the same
    // reason: applying a (possibly stale) echo here would both revert the
    // pending edit's own effect on eqModel_ and clear modelDirty_
    // underneath it, permanently cancelling that edit's push rather than
    // just delaying it.
    if (modelDirty_) return;

    ssfilter::EqModel parsed;
    parsed.bands.assign(mirrorStaging_, mirrorStaging_ + mirrorStagingCount_);
    parsed.outputTrimDb = mirrorStagingTrim_;
    parsed.crossFeedGainLtoRDb = mirrorStagingXfeedLtoR_;
    parsed.crossFeedGainRtoLDb = mirrorStagingXfeedRtoL_;
    eqModel_ = parsed;
    lastReport_ = ssfilter::StabilityGuard::analyze(ssfilter::eq::assembleStateSpace(eqModel_, getSampleRate()));
    repaint();
}

void UIEq::stateChanged(const char* key, const char* value)
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
        return; // the DSP side (PluginEq::setState()) applies this to the real analyzer independently
    }
    // A/B compare, restored -- see PluginEq.hpp's StateIndex comment. Just
    // plain restore into the UI's own mirrors; nothing to push back (this
    // arrived *from* the host, same "don't echo a restore" discipline
    // applyMirrorStagingIfComplete() follows for the band list itself).
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

void UIEq::applyModelToEngine()
{
    // Deliberately does *not* reset idleTicksSinceEdit_ -- that's what
    // turns uiIdle()'s check below into a "wait for quiet" debounce
    // (only push once edits stop arriving) rather than a periodic
    // throttle (push roughly every kDebounceIdleTicks regardless of
    // continued edits). The former sounds right for the Bode plot (a
    // pure recompute-cost concern, no harm waiting), but for the audio-
    // affecting engine push it meant a fast, continuous EQ drag left the
    // engine's coefficients completely frozen for the whole gesture and
    // then jumped once dragging paused -- the application should react
    // fast instead, and (now separately fixed) the
    // reason a debounced push could represent a very large single jump
    // in the first place. idleTicksSinceEdit_ is only reset around an
    // actual push now (see uiIdle()), so a continuous drag gets pushed
    // periodically throughout, not just once at the end.
    modelDirty_ = true;
    repaint();
}

void UIEq::pushModelNow()
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::eq::assembleStateSpace(eqModel_, sr);
    lastReport_ = ssfilter::StabilityGuard::analyze(assembled); // report only -- the DSP side re-enforces on its own once kParamMirrorCommit below lands
    pushBandParamMirror();
    // Must toggle, not resend a constant -- see mirrorCommitToggle_'s own
    // comment (UIEq.hpp) -- and must come after every mirror field above.
    mirrorCommitToggle_ = 1.0f - mirrorCommitToggle_;
    setParameterValue(kParamMirrorCommit, mirrorCommitToggle_);
    pushAbState(); // keeps the *active* A/B slot's own persisted copy in sync with this edit -- see its own comment
    repaint();
}

void UIEq::pushBandParamMirror()
{
    // See this function's own declaration comment (UIEq.hpp) for why this
    // has to happen from here, not from the DSP side alone. Slots past the
    // real band count get an ordinary default EqBand() -- matching
    // PluginEq::syncMirrorFromModel()'s own identical convention, so the
    // two stay byte-for-byte in agreement regardless of which one a host
    // happens to query.
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
        setParameterValue(kParamBandCrossFeedBase + i, float(int(b.crossFeed)));
    }
    // Model-level, not per-band -- see EqModel::crossFeedGainLtoRDb's own
    // comment. Pushed after the per-band bank, same relative position its
    // own ParameterIndex slot has.
    setParameterValue(kParamCrossFeedGainLtoRDb, float(eqModel_.crossFeedGainLtoRDb));
    setParameterValue(kParamCrossFeedGainRtoLDb, float(eqModel_.crossFeedGainRtoLDb));
    // Band Shelf bandwidth and per-band solo -- their banks sit after the
    // crossfeed gains (solo is the last mirror field before
    // kParamMirrorCommit, see applyMirrorStagingIfComplete()'s own
    // sentinel check).
    for (int i = 0; i < int(kNumMirroredBands); ++i)
    {
        const ssfilter::EqBand b = (i < n) ? eqModel_.bands[size_t(i)] : ssfilter::EqBand();
        setParameterValue(kParamBandBandwidthBase + i, float(b.bandwidthOct));
        setParameterValue(kParamBandSoloBase + i, b.solo ? 1.0f : 0.0f);
    }
}

// ---- ssgui::IEqEditorHost -----------------------------------------------

ssfilter::EqBand UIEq::getBand(int index) const
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return ssfilter::EqBand{};
    return eqModel_.bands[size_t(index)];
}

void UIEq::setBandFreq(int index, double freqHz)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(freqHz)) return;
    // Routed through eq::propagateHarmonicFreq() rather than writing
    // freqHz directly -- if `index` is harmonically linked, this also
    // repositions every other member of its group to keep their own
    // ratios intact (a no-op otherwise). Every frequency-changing
    // gesture (graph drag, scroll-nudge, typed edit) already calls this
    // one function, so linking behaves identically regardless of which
    // gesture moved the band.
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::eq::propagateHarmonicFreq(eqModel_, index, freqHz, sr);
    applyModelToEngine();
}

void UIEq::setBandBandwidth(int index, double octaves)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(octaves)) return;
    ssfilter::EqBand& b = eqModel_.bands[size_t(index)];
    if (!ssfilter::eqBandTypeHasBandwidth(b.type)) return;
    b.bandwidthOct = std::max(ssfilter::kMinBandwidthOct, std::min(ssfilter::kMaxBandwidthOct, octaves));
    applyModelToEngine();
}

void UIEq::setBandGain(int index, double gainDb)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(gainDb)) return;
    eqModel_.bands[size_t(index)].gainDb = std::max(-60.0, std::min(60.0, gainDb));
    applyModelToEngine();
}

void UIEq::setBandQ(int index, double q)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(q)) return;
    eqModel_.bands[size_t(index)].q = std::max(0.05, std::min(100.0, q));
    applyModelToEngine();
}

void UIEq::setBandType(int index, ssfilter::EqBandType type)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    ssfilter::eq::applyBandType(eqModel_.bands[size_t(index)], type); // also resets a now-unused gain, see its own comment
    applyModelToEngine();
}

void UIEq::setBandSlopeStages(int index, int slopeStages)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].slopeStages = std::max(1, std::min(4, slopeStages));
    applyModelToEngine();
}

void UIEq::setBandEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].enabled = enabled;
    applyModelToEngine();
}

void UIEq::setBandSolo(int index, bool solo)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].solo = solo;
    applyModelToEngine();
}

void UIEq::setBandChannel(int index, ssfilter::EqChannel channel)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].channel = channel;
    applyModelToEngine();
}

void UIEq::setBandCrossFeed(int index, ssfilter::EqCrossFeed crossFeed)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    // No longer touches anything but `crossFeed` itself -- the crossfeed
    // transit's own send level lives on the MODEL now
    // (crossFeedGainLtoRDb/RtoLDb, see EqModel's own comment), not per
    // band, so there's nothing band-local left to reset on this
    // transition. `enabled`/`gainDb` are left alone too, same as every
    // other routing change (setBandChannel() right above never touches
    // them either) -- a freshly-placed band already defaults to enabled,
    // so a Both/Left/Right/L->R/R->L cycle never needs to re-arm
    // anything else.
    eqModel_.bands[size_t(index)].crossFeed = crossFeed;
    applyModelToEngine();
}

void UIEq::setCrossFeedGainLtoRDb(double gainDb)
{
    if (!std::isfinite(gainDb)) return;
    // -60..+12: quiet-to-silent up through a modest boost above unity --
    // deliberately narrower than setBandGain()'s own -60..+60 (that's a
    // single Bell/Shelf's own peak at one frequency; this is the OVERALL
    // send level for the WHOLE L->R transit, where a full +60dB boost
    // would never be sane). See EqModel::crossFeedGainLtoRDb's own
    // comment for why this is a model-level control, not per band.
    eqModel_.crossFeedGainLtoRDb = std::max(-60.0, std::min(12.0, gainDb));
    applyModelToEngine();
}

void UIEq::setCrossFeedGainRtoLDb(double gainDb)
{
    if (!std::isfinite(gainDb)) return;
    eqModel_.crossFeedGainRtoLDb = std::max(-60.0, std::min(12.0, gainDb));
    applyModelToEngine();
}

void UIEq::addBand(double freqHz, double gainDb)
{
    // kMaxOrder/4 bands fit (see StateSpaceTypes.hpp); refuse silently past
    // that rather than let assembleStateSpace()'s clampOrder() truncate the
    // chain in a way that wouldn't match what's shown here.
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

void UIEq::removeBand(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands.erase(eqModel_.bands.begin() + index);
    if (editTarget_ == EditTarget::BandField && editBandIndex_ == index) cancelEdit();

    // Every band after `index` just shifted down one slot -- this is the
    // one place that actually knows the mutation happened, so it's the
    // one place responsible for fixing selectedBand_ up to match, via
    // selectBand() (which also keeps graph_/list_/kParamAuditionBand in
    // agreement) rather than trusting whichever widget's own onMouse()
    // called this to have done it already.
    selectBand(selectedBand_ == index ? -1 : (selectedBand_ > index ? selectedBand_ - 1 : selectedBand_));

    applyModelToEngine();
}

void UIEq::duplicateBand(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    constexpr int kMaxBands = ssfilter::kMaxOrder / 4; // same cap as addBand()
    if (int(eqModel_.bands.size()) >= kMaxBands) return;

    const ssfilter::EqBand copy = eqModel_.bands[size_t(index)];
    const int newIndex = index + 1;
    eqModel_.bands.insert(eqModel_.bands.begin() + newIndex, copy);

    // An in-progress text edit pointed at a band that just shifted up one
    // slot needs to follow it -- same reasoning as removeBand()'s own
    // index-fixup, just a shift instead of a cancel since nothing here
    // was deleted.
    if (editTarget_ == EditTarget::BandField && editBandIndex_ >= newIndex) editBandIndex_ += 1;

    // Select the new duplicate directly -- it's immediately draggable/
    // editable without an extra click, and is a more useful default than
    // leaving whatever was selected before pointing at a possibly
    // different band now that indices above it shifted.
    selectBand(newIndex);

    applyModelToEngine();
}

void UIEq::createHarmonicTwin(int index, double factor)
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    const int newIndex = ssfilter::eq::createHarmonicTwin(eqModel_, index, factor, sr);
    if (newIndex < 0) return; // refused -- band cap, or an invalid index/factor

    // Same index-shift fixup duplicateBand() already needs, for the same
    // reason: a band was just inserted, so an in-progress text edit
    // pointed at anything from newIndex onward has to follow it.
    if (editTarget_ == EditTarget::BandField && editBandIndex_ >= newIndex) editBandIndex_ += 1;

    // Select the new twin directly -- immediately visible/editable
    // without an extra click, same reasoning as duplicateBand()'s own.
    selectBand(newIndex);

    applyModelToEngine();
}

void UIEq::unlinkHarmonic(int index)
{
    ssfilter::eq::unlinkHarmonic(eqModel_, index);
    applyModelToEngine();
}

void UIEq::beginHarmonicFactorEdit(int sourceIndex)
{
    if (sourceIndex < 0 || sourceIndex >= int(eqModel_.bands.size())) return;
    if (editTarget_ != EditTarget::None) commitEdit();
    editTarget_ = EditTarget::HarmonicFactor;
    pendingHarmonicSourceBand_ = sourceIndex;
    editBuffer_ = "2"; // a sensible starting point -- "standard double" is the most common case
    getWindow().focus(); // see UIStateSpace.cpp's identical call for why
    repaint();
}

bool UIEq::isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const
{
    if (editTarget_ != EditTarget::HarmonicFactor || pendingHarmonicSourceBand_ != sourceIndex) return false;
    liveText = editBuffer_;
    return true;
}

void UIEq::commitHarmonicFactorEdit()
{
    // A plain forwarding call -- graph_'s own "click anywhere while the
    // Custom Factor input is showing" handler uses this instead of
    // reaching into UIEq's private edit state directly. Safe even if
    // editTarget_ somehow isn't HarmonicFactor by the time this runs
    // (commitEdit() itself is a no-op whenever editTarget_ is None, and
    // otherwise just commits whatever *is* active, exactly like every
    // other "click away commits" site in this file already does).
    commitEdit();
}

void UIEq::beginBandAudition(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    // Capture the *persistent* toolbar Audition state right now, before
    // this momentary hold's own setParameterValue() call below touches
    // kParamAudition at all. endBandAudition() restores from this
    // snapshot rather than re-reading auditioning_ live -- some hosts
    // echo a UI-originated parameter write back into this same UI's own
    // parameterChanged() (kParamAudition's own case sets auditioning_)
    // well before the mouse button is released, which would otherwise
    // make auditioning_ read back as "true" (this hold's own doing) by
    // the time endBandAudition() runs, leaving Audition stuck on after
    // release instead of turning back off.
    auditionHoldPrevState_ = auditioning_;
    setParameterValue(kParamAuditionBand, float(index));
    setParameterValue(kParamAudition, 1.0f);
}

void UIEq::endBandAudition()
{
    // Restore the *persistent* state exactly -- if the toolbar's own
    // Audition button was already engaged (on this band or another) before
    // the momentary hold started, it stays engaged on whatever's
    // currently selected; if it wasn't, this turns Audition back off
    // rather than leaving it stuck on from the momentary preview. Restored
    // from auditionHoldPrevState_ (captured at the start of the hold), not
    // auditioning_ itself -- see beginBandAudition()'s own comment for why.
    if (selectedBand_ >= 0) setParameterValue(kParamAuditionBand, float(selectedBand_));
    setParameterValue(kParamAudition, auditionHoldPrevState_ ? 1.0f : 0.0f);
}

void UIEq::selectBand(int index)
{
    selectedBand_ = std::max(-1, std::min(getBandCount() - 1, index));
    graph_->setSelectedBand(selectedBand_);
    list_->setSelectedBand(selectedBand_);
    // Keeps the DSP-side preview target in sync with the selection at all
    // times, not just while actually Auditioning -- so flipping Audition on
    // later previews whatever's currently selected immediately, with no
    // stale target left over from a previous selection. Harmless while
    // Audition is off: run() only reads auditionBand_ when auditioning_ is true.
    // See ../dyneq/UIDynEq.cpp's identical selectBand() for the same
    // convention.
    if (selectedBand_ >= 0) setParameterValue(kParamAuditionBand, float(selectedBand_));
    refreshAuditionColors();
    refreshBandStrip();
    repaint();
}

void UIEq::refreshBandStrip()
{
    const bool valid = selectedBand_ >= 0 && selectedBand_ < getBandCount();
    const int type = valid ? int(getBand(selectedBand_).type) : -1;
    bandStrip_->setVisible(valid);
    if (!valid || (selectedBand_ == bandStripBand_ && type == bandStripType_) || bandStrip_->isEditing())
        return;
    bandStripBand_ = selectedBand_;
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

void UIEq::refreshAuditionColors()
{
    // Same blue-ish "monitoring, not the real signal" tint the sibling
    // Dynamic EQ plugin's own Audition button uses when engaged.
    btnAudition_->setColors(auditioning_ ? Color(70, 130, 160) : Color(58, 62, 70),
                             auditioning_ ? Color(90, 150, 180) : Color(78, 84, 96),
                             Color(230, 232, 236));
    // Nothing to preview with no band selected -- grayed out and
    // unclickable (SimpleButton::setEnabled() already does both).
    btnAudition_->setEnabled(selectedBand_ >= 0);
    // No selection left to preview -- force Audition back off rather than
    // leaving it silently engaged with a stale/out-of-range target (same
    // "grayed out AND actually off" pairing ../dyneq/UIDynEq.cpp's own
    // refreshBandStripLabels() enforces for its own Audition button).
    if (selectedBand_ < 0 && auditioning_)
    {
        auditioning_ = false;
        setParameterValue(kParamAudition, 0.0f);
    }
    repaint();
}

void UIEq::beginBandTextEdit(int index, ssgui::EqField field, bool clearValue)
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
        const double v = field == ssgui::EqField::Freq ? b.freqHz
                        : field == ssgui::EqField::Bandwidth ? b.bandwidthOct
                        : field == ssgui::EqField::Gain ? b.gainDb
                        : b.q;
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", v);
        editBuffer_ = buf;
    }

    getWindow().focus(); // see UIStateSpace.cpp's identical call for why
    repaint();
}

bool UIEq::isEditingBandField(int index, ssgui::EqField field, std::string& liveText) const
{
    if (editTarget_ != EditTarget::BandField || editBandIndex_ != index || editBandField_ != field) return false;
    liveText = editBuffer_;
    return true;
}

void UIEq::beginTrimTextEdit(bool clearValue)
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

void UIEq::beginSmoothTextEdit(bool clearValue)
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

bool UIEq::checkInlineDoubleClick(int fieldId)
{
    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400);
    const bool isDoubleClick = fieldId == lastClickInlineField_ && (now - lastClickInlineTime_) < kDoubleClickWindow;
    lastClickInlineField_ = isDoubleClick ? -1 : fieldId; // a double-click consumes the pair, like everywhere else
    lastClickInlineTime_ = now;
    return isDoubleClick;
}

void UIEq::commitEdit()
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

    // The graph's own "Custom Factor..." input closes its menu here
    // either way, valid or not -- there's nothing left to show once the
    // edit ends, unlike a stale invalid Freq/Gain/Q buffer, which the
    // field itself just silently keeps its prior value for.
    if (target == EditTarget::HarmonicFactor) graph_->endHarmonicFactorEdit();

    if (!valid) { repaint(); return; }

    if (target == EditTarget::BandField)
    {
        // bandIndex may have been deleted between beginBandTextEdit() and
        // here (same bounds check setBandFreq()/setBandGain()/setBandQ()
        // themselves already do).
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

void UIEq::setInlineFieldValue(EditTarget which, double v)
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

bool UIEq::onMotion(const MotionEvent& rawEv)
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

bool UIEq::onScroll(const ScrollEvent& rawEv)
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

void UIEq::cancelEdit()
{
    if (editTarget_ == EditTarget::HarmonicFactor) graph_->endHarmonicFactorEdit();
    editTarget_ = EditTarget::None;
    repaint();
}

bool UIEq::onCharacterInput(const CharacterInputEvent& ev)
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

bool UIEq::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z/Ctrl+Y -- see ../../compressor's UICompressorMono::onKeyboard()
    // for the full reasoning (checked first, unconditionally, consumed
    // regardless of whether there's actually anything to undo/redo).
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
        // Not editing a text field: Delete/Backspace removes whichever
        // band is currently selected (clicked in the graph, or in the
        // band list -- selection is synced between the two). This is in
        // addition to the graph's own double-click-to-remove and
        // right-click-menu "Delete Band" -- see EqGraphWidget.hpp.
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

bool UIEq::onMouse(const MouseEvent& rawEv)
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
        // A double-click starts the edit with an empty buffer (so typing
        // replaces the value outright) -- the same convention every
        // other numeric field in this project already follows; see
        // checkInlineDoubleClick()'s declaration comment for why these
        // needed the fix.
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

void UIEq::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadEqPresetFile(filename);
    else if (op == PendingFileOp::Save) saveEqPresetFile(filename);
    else if (op == PendingFileOp::Export) exportStateSpaceFile(filename);
}

void UIEq::loadEqPresetFile(const char* path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) return;
    std::ostringstream ss;
    ss << f.rdbuf();

    applyModelSnapshot(ss.str());
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See ../../compressor's UICompressorMono.cpp (this project's own
// sibling repo) for the full design writeup -- ported here against this
// plugin's own JSON model instead of a flat parameter list.

void UIEq::applyModelSnapshot(const std::string& json)
{
    // Formerly loadEqPresetFile()'s own body past the file read.
    ssfilter::EqModel parsed;
    if (!ssfilter::eq::fromJson(json, parsed)) return;
    eqModel_ = parsed;
    applyModelToEngine();
}

void UIEq::onAbA()
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

void UIEq::onAbB()
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

void UIEq::onAbCopy()
{
    const std::string snap = ssfilter::eq::toJson(eqModel_);
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
    pushAbState();
}

void UIEq::pushAbState()
{
    // Keeps the *currently active* slot's own persisted copy live-synced
    // with whatever's actually being edited right now -- onAbA()/onAbB()
    // only capture a snapshot at the moment you switch *away* from a
    // slot, so without this, the active slot's own stored State would
    // reflect its content as of the last switch, not your latest edits,
    // the instant a session is saved mid-edit. Safe to call unconditionally
    // (setState() on an unchanged value is a cheap no-op downstream).
    if (abActive_ == AbSlot::A) abSlotA_ = ssfilter::eq::toJson(eqModel_);
    else abSlotB_ = ssfilter::eq::toJson(eqModel_);

    setState("abSlotA", abSlotA_.c_str());
    setState("abSlotB", abSlotB_.c_str());
    setState("abHasA", abHasA_ ? "1" : "0");
    setState("abHasB", abHasB_ ? "1" : "0");
    setState("abActive", abActive_ == AbSlot::B ? "1" : "0");
}

void UIEq::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIEq::onUndo()
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

void UIEq::onRedo()
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

void UIEq::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIEq::saveEqPresetFile(const char* path)
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

void UIEq::exportStateSpaceFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::eq::assembleStateSpace(eqModel_, sr);
    ssfilter::StabilityGuard::enforce(assembled); // write out exactly what the audio engine actually runs

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::Serialization::toJson(assembled);
    rememberPresetDir(p.c_str());
}

void UIEq::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UIEq::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIEq(); }

END_NAMESPACE_DISTRHO
