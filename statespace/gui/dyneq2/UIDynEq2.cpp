// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIDynEq2.hpp"

#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../../plugin_dyn_eq2/DistrhoPluginInfo.h"
#include "../../plugin_dyn_eq2/PluginDynEq2.hpp" // for ParameterIndex enum
#include "FileBrowserDialog.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <vector>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::DynEqBandListWidget2;
using DGL_NAMESPACE::DynEqGraphWidget2;
using DGL_NAMESPACE::SimpleButton;

namespace {
constexpr int kDebounceIdleTicks = 3; // same debounce window as every sibling plugin's own engine push

} // namespace

UIDynEq2::UIDynEq2()
    : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
{
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    for (float& v : currentGainDb_) v = 0.0f;
    for (float& v : spectrumOutRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;

    graph_ = new DynEqGraphWidget2(this, this);
    graph_->setSampleRate(getSampleRate());
    graph_->setSpectrumMode(spectrumMode_);
    graph_->setSelectedBand(selectedBand_);
    graph_->setOnSelectionChanged([this](int i) { selectBand(i); });
    graph_->setOnSpectrumAutoscaleChanged([this](bool on) { setState("spectrumAutoscale", on ? "1" : "0"); });

    list_ = new DynEqBandListWidget2(this, this);
    list_->setOnSelectionChanged([this](int i) { selectBand(i); });

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load Dynamic EQ 2 Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "dyn_eq_preset.ssdynjson";
        opts.title = "Save Dynamic EQ 2 Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnExport_ = new SimpleButton(this, "Export to State-Space...");
    btnExport_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Export;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "dyn_eq_exported.ssjson";
        opts.title = "Export Resting-Shape Coefficients (.ssjson)";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnAddBand_ = new SimpleButton(this, "+ Band");
    btnAddBand_->setCallback([this] {
        // A reasonable "just give me another band" default, mirroring
        // UIMbCompressor's own +Band: the geometric-mean frequency of the
        // widest still-empty octave gap among existing bands (or the
        // middle of the whole graph if there are none yet).
        double bestLo = 20.0, bestHi = 20000.0;
        if (getBandCount() > 0)
        {
            std::vector<double> freqs;
            for (int i = 0; i < getBandCount(); ++i) freqs.push_back(getBand(i).freqHz);
            std::sort(freqs.begin(), freqs.end());
            std::vector<double> edges;
            edges.push_back(20.0);
            for (double f : freqs) edges.push_back(f);
            edges.push_back(20000.0);
            double bestSpan = -1.0;
            for (size_t i = 0; i + 1 < edges.size(); ++i)
            {
                const double span = std::log(edges[i + 1]) - std::log(edges[i]);
                if (span > bestSpan) { bestSpan = span; bestLo = edges[i]; bestHi = edges[i + 1]; }
            }
        }
        addBand(std::sqrt(bestLo * bestHi), 0.0); // also selects the new band and pushes the model
    });

    // Audition: preview selectedBand_'s isolated detector output, with its
    // own live gain reduction applied, for tuning by ear. Next to +Band,
    // as in the Parametric EQ, since it acts on the selected band --
    // refreshAuditionColors() grays it out (and turns Audition off) when
    // there's no band to preview.
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
    tooltip_->addTarget(btnLoad_, "Load a saved Dynamic EQ 2 preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a Dynamic EQ 2 preset.");
    tooltip_->addTarget(btnExport_, "Export the current resting-gain shape as a plain State-Space filter (.ssjson).");
    tooltip_->addTarget(btnAddBand_, "Add a new band in the widest still-empty gap between existing bands (or click the graph).");
    tooltip_->addTarget(btnAudition_, "Audition the selected band's own isolated detector signal, with its live gain reduction applied.");
    tooltip_->addTarget(btnReset_, "Reset the engine's internal filter state (clears any transient buildup).");
    tooltip_->addTarget(btnBypass_, "Bypass: passes audio through unprocessed.");
    tooltip_->addTarget(btnAbA_, "Switch to comparison slot A (captures the current settings into B first).");
    tooltip_->addTarget(btnAbB_, "Switch to comparison slot B (captures the current settings into A first).");
    tooltip_->addTarget(btnAbCopy_, "Copy the current settings into the other A/B slot.");
    tooltip_->addTarget(btnUndo_, "Undo the last settings change.");
    tooltip_->addTarget(btnRedo_, "Redo the last undone settings change.");
    // Column hover descriptions, via the list's own cellTooltipAt().
    tooltip_->addFieldTarget(list_, [this](double x, double y) { return list_->cellTooltipAt(x, y); });

    refreshAuditionColors();
    layout();
}

UIDynEq2::~UIDynEq2()
{
    // See ../eq/UIEq.cpp's identical destructor for the full reasoning --
    // closing the editor before uiIdle()'s debounce fires would otherwise
    // silently drop the most recent edit.
    if (modelDirty_) pushModelNow();
}

void UIDynEq2::layout()
{
    // The Parametric EQ's layout (UIEq::layout()): row 1 system-wide
    // controls with A/B and Undo/Redo right-aligned; row 2 this filter's
    // own controls; then the graph and, below it, the band list.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window -- see TooltipOverlay's own file comment.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));

    constexpr float pad = 6.0f;
    constexpr float rowH = 28.0f;
    constexpr float toolbarH = 2.0f * rowH + pad;

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

    // Row 2: +Band, Audition, then the spectrum/output fields.
    by += rowH + 2.0f;
    bx = pad;
    place(btnAddBand_, 68);
    place(btnAudition_, 68);

    spectrumModeFieldX_ = bx + 108.0f; // room for the "Spectrum Source:" label
    spectrumModeFieldY_ = by;
    spectrumModeFieldW_ = 44.0f;
    spectrumModeFieldH_ = bh;
    bx = spectrumModeFieldX_ + spectrumModeFieldW_ + 20.0f;

    smoothFieldX_ = bx + 150.0f; // room for the "Spectrum Smoothing (ms):" label
    smoothFieldY_ = by;
    smoothFieldW_ = 56.0f;
    smoothFieldH_ = bh;
    bx = smoothFieldX_ + smoothFieldW_ + 20.0f;

    fftSizeFieldX_ = bx + 140.0f; // room for the "Spectrum Resolution:" label
    fftSizeFieldY_ = by;
    fftSizeFieldW_ = 68.0f;
    fftSizeFieldH_ = bh;
    bx = fftSizeFieldX_ + fftSizeFieldW_ + 20.0f;

    trimFieldX_ = bx + 116.0f; // room for the "Output Trim (dB):" label
    trimFieldY_ = by;
    trimFieldW_ = 64.0f;
    trimFieldH_ = bh;

    const float contentTop = toolbarH + pad * 2.0f;
    const float contentH = h - contentTop - pad;
    const float graphH = contentH * 0.66f;
    const float listH = contentH - graphH - pad;

    graph_->setAbsolutePos(int(pad), int(contentTop));
    graph_->setSize(uint(w - 2.0f * pad), uint(graphH));

    list_->setAbsolutePos(int(pad), int(contentTop + graphH + pad));
    list_->setSize(uint(w - 2.0f * pad), uint(listH));
}

void UIDynEq2::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIDynEq2::refreshAuditionColors()
{
    btnAudition_->setColors(auditioning_ ? Color(70, 130, 160) : Color(58, 62, 70),
                             auditioning_ ? Color(90, 150, 180) : Color(78, 84, 96),
                             Color(230, 232, 236));
    // Nothing to preview with no band selected -- grayed out and
    // unclickable, and Audition itself turned off rather than left armed
    // with a stale target.
    btnAudition_->setEnabled(selectedBand_ >= 0);
    if (selectedBand_ < 0 && auditioning_)
    {
        auditioning_ = false;
        setParameterValue(kParamAudition, 0.0f);
    }
    repaint();
}

void UIDynEq2::onNanoDisplay()
{
    const float w = float(getWidth());

    beginPath();
    rect(0, 0, w, float(getHeight()));
    fillColor(Color(18, 19, 23));
    fill();

    fontSize(12.0f);

    // The two editable inline fields -- label to the left, value box.
    auto inlineField = [&](float fx, float fy, float fw, float fh, const char* labelText, EditTarget target, const char* fmt, double value) {
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(180, 184, 192));
        text(fx - 4.0f, fy + fh * 0.5f, labelText, nullptr);
        const bool editing = editTarget_ == target;
        beginPath();
        roundedRect(fx + 0.5f, fy + 0.5f, fw - 1.0f, fh - 1.0f, 3.0f);
        fillColor(editing ? Color(235, 195, 60) : Color(34, 37, 43));
        fill();
        strokeColor(Color(14, 15, 18));
        strokeWidth(1.0f);
        stroke();
        char buf[32];
        const char* toShow = buf;
        if (editing) toShow = editBuffer_.c_str();
        else std::snprintf(buf, sizeof(buf), fmt, value);
        fillColor(editing ? Color(20, 20, 24) : Color(220, 223, 228));
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(fx + fw * 0.5f, fy + fh * 0.5f, toShow, nullptr);
    };
    inlineField(trimFieldX_, trimFieldY_, trimFieldW_, trimFieldH_, "Output Trim (dB):", EditTarget::OutputTrim, "%+.1f", dynEqModel_.outputTrimDb);
    inlineField(smoothFieldX_, smoothFieldY_, smoothFieldW_, smoothFieldH_, "Spectrum Smoothing (ms):", EditTarget::SpectrumSmoothMs, "%.0f", spectrumSmoothMs_);

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

    // The "static export" rho-stability badge that used to sit here is
    // removed -- see ../eq/UIEq.cpp's identical comment: unreadable behind
    // row 1's buttons after this session's own reorganization, and this
    // plugin's live Regalia-Mitra engine has no matrix of its own to be
    // unstable in that sense anyway (this badge only ever reflected
    // "Export to State-Space..."'s frozen resting-shape snapshot, not the
    // live engine -- see DynamicEqModel2.hpp's file comment). Removing the
    // display also removes the per-repaint assembleStateSpace()/analyze()
    // call that only ever existed to feed it -- nothing else read
    // `assembled`/`rep`. Kept only on Matrix Filter, where it's an actual
    // debugging aid for directly-editable matrices.
    if (faultBlinkTicks_ > 0)
    {
        fontSize(12.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(230, 90, 90));
        text(w - 10.0f, 33.0f, "Numerical fault auto-recovered - state was reset", nullptr);
    }
}

void UIDynEq2::uiIdle()
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

    // Undo/redo checkpointing -- a longer, independent debounce window
    // than kDebounceIdleTicks's own (that one's about not spamming the
    // host with State updates every drag frame; this one's about not
    // splitting a single edit gesture into a dozen undo-able steps). See
    // this file's own "A/B compare + undo/redo" block above for the
    // rest of the design.
    if (!applyingSnapshot_)
    {
        const std::string nowJson = ssfilter::dyneq2::toJson(dynEqModel_);
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

    {
        constexpr double kAssumedTickMs = 16.0;
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
    }

    // The live per-band gain readback moves continuously (program-material-
    // driven, not edit-driven), so the graph is repainted every tick
    // regardless of spectrumMode_ or modelDirty_ -- this is what makes the
    // curve actually look "alive".
    graph_->repaint();
}

void UIDynEq2::parameterChanged(uint32_t index, float value)
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
        // Keeps the button in sync with a host-automated Audition lane too,
        // not just this UI's own click handler -- same reasoning as
        // kParamBypass right above.
        auditioning_ = value >= 0.5f;
        refreshAuditionColors();
        repaint();
        break;
    default:
        if (index >= kParamCurrentGainBase && index < kParamCurrentGainBase + uint32_t(ssfilter::kMaxDynEqBands2))
        {
            // Smoothed against wall-clock time before being stored -- see
            // currentGainDb_'s own comment in UIDynEq2.hpp for why (the
            // Multiband GR-meter readability fix, applied here too).
            const uint32_t i = index - kParamCurrentGainBase;
            if (!currentGainSmoothInit_[i])
            {
                currentGainDb_[i] = value; // snap on the very first reading, nothing to smooth from yet
                currentGainSmoothInit_[i] = true;
                currentGainSmoothUpdated_[i] = std::chrono::steady_clock::now();
            }
            else
            {
                const auto now = std::chrono::steady_clock::now();
                const float dtSec = std::chrono::duration<float>(now - currentGainSmoothUpdated_[i]).count();
                currentGainSmoothUpdated_[i] = now;
                constexpr float kTauSec = 0.22f; // same value/reasoning as MultibandGraphWidget::setBandGainDb()
                const float coeff = dtSec > 0.0f ? std::exp(-dtSec / kTauSec) : 1.0f;
                currentGainDb_[i] = coeff * currentGainDb_[i] + (1.0f - coeff) * value;
            }
        }
        else if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
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

void UIDynEq2::applyMirrorStagingIfComplete(uint32_t index, float value)
{
    // Same index arithmetic as PluginDynEq2::setParameterValue()'s own
    // mirror branch -- kept in lockstep with it deliberately.
    if (index == kParamBandCount)
        mirrorStagingCount_ = std::max(0, std::min(int(kNumMirroredBands), int(value + 0.5f)));
    else if (index == kParamOutputTrimDb)
        mirrorStagingTrim_ = double(value);
    else if (index == kParamSoloedBand)
        mirrorStagingSoloedBand_ = std::max(-1, std::min(int(kNumMirroredBands) - 1, int(value + (value >= 0 ? 0.5f : -0.5f))));
    else if (index >= kParamBandFreqBase && index < kParamBandFreqBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandFreqBase].freqHz = double(value);
    else if (index >= kParamBandQBase && index < kParamBandQBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandQBase].q = double(value);
    else if (index >= kParamBandGainBase && index < kParamBandGainBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandGainBase].gainDb = double(value);
    else if (index >= kParamBandThresholdBase && index < kParamBandThresholdBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandThresholdBase].thresholdDb = double(value);
    else if (index >= kParamBandRatioBase && index < kParamBandRatioBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandRatioBase].ratio = double(value);
    else if (index >= kParamBandRatioUpBase && index < kParamBandRatioUpBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandRatioUpBase].ratioUp = double(value);
    else if (index >= kParamBandRangeBase && index < kParamBandRangeBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandRangeBase].rangeDb = double(value);
    else if (index >= kParamBandAttackBase && index < kParamBandAttackBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandAttackBase].attackMs = double(value);
    else if (index >= kParamBandReleaseBase && index < kParamBandReleaseBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandReleaseBase].releaseMs = double(value);
    else if (index >= kParamBandDynamicsEnabledBase && index < kParamBandDynamicsEnabledBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandDynamicsEnabledBase].dynamicsEnabled = value >= 0.5f;
    else if (index >= kParamBandEnabledBase && index < kParamBandEnabledBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandEnabledBase].enabled = value >= 0.5f;
    else if (index >= kParamBandChannelBase && index < kParamBandChannelBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandChannelBase].channel = ssfilter::EqChannel(std::max(0, std::min(2, int(value + 0.5f))));
    else if (index >= kParamBandCrossFeedBase && index < kParamBandCrossFeedBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandCrossFeedBase].crossFeed = ssfilter::EqCrossFeed(std::max(0, std::min(2, int(value + 0.5f))));
    else if (index == kParamCrossFeedGainLtoRDb)
        mirrorStagingXfeedLtoR_ = double(value);
    else if (index == kParamCrossFeedGainRtoLDb)
        mirrorStagingXfeedRtoL_ = double(value);
    else if (index >= kParamBandHarmonicGroupBase && index < kParamBandHarmonicGroupBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandHarmonicGroupBase].harmonicGroup = std::max(0, int(value + 0.5f));
    else if (index >= kParamBandHarmonicRatioBase && index < kParamBandHarmonicRatioBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandHarmonicRatioBase].harmonicRatio = double(value);
    else if (index >= kParamBandTypeBase && index < kParamBandTypeBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandTypeBase].type = ssfilter::EqBandType(std::max(0, std::min(8, int(value + 0.5f))));
    else if (index >= kParamBandBandwidthBase && index < kParamBandBandwidthBase + kNumMirroredBands)
        mirrorStaging_[index - kParamBandBandwidthBase].bandwidthOct = double(value);
    else
        return; // not a mirror field at all

    // The last field of the last slot -- always the last to arrive within
    // one delivery burst, see kParamMirrorCommit's own comment
    // (PluginDynEq2.hpp).
    if (index != kParamBandBandwidthBase + kNumMirroredBands - 1) return;
    if (modelDirty_) return; // a newer local edit is still pending its own debounced push

    ssfilter::DynamicEqModel2 parsed;
    parsed.bands.assign(mirrorStaging_, mirrorStaging_ + mirrorStagingCount_);
    parsed.outputTrimDb = mirrorStagingTrim_;
    parsed.soloedBand = mirrorStagingSoloedBand_;
    parsed.crossFeedGainLtoRDb = mirrorStagingXfeedLtoR_;
    parsed.crossFeedGainRtoLDb = mirrorStagingXfeedRtoL_;
    dynEqModel_ = parsed;
    selectedBand_ = getBandCount() > 0 ? std::max(0, std::min(getBandCount() - 1, selectedBand_)) : -1;
    graph_->setSelectedBand(selectedBand_);
    list_->setSelectedBand(selectedBand_);
    if (selectedBand_ >= 0) setParameterValue(kParamAuditionBand, float(selectedBand_)); // see selectBand()'s own comment
    refreshAuditionColors();
    repaint();
}

void UIDynEq2::stateChanged(const char* key, const char* value)
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
        return;
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

// ---- model push (debounced) ---------------------------------------------

void UIDynEq2::applyModelToEngine()
{
    modelDirty_ = true;
    repaint();
}

void UIDynEq2::pushModelNow()
{
    pushBandParamMirror();
    // Must toggle, not resend a constant -- see mirrorCommitToggle_'s own
    // comment (UIDynEq2.hpp) -- and must come after every mirror field above.
    mirrorCommitToggle_ = 1.0f - mirrorCommitToggle_;
    setParameterValue(kParamMirrorCommit, mirrorCommitToggle_);
    pushAbState(); // keeps the *active* A/B slot's own persisted copy in sync with this edit -- see its own comment
    repaint();
}

void UIDynEq2::pushBandParamMirror()
{
    // See UIEq::pushBandParamMirror()'s identical declaration comment
    // (gui/eq/UIEq.hpp) for why this has to happen from here, not the DSP
    // side alone -- ported unchanged, proven correct end-to-end against a
    // real Ardour session for the stereo EQ before this port.
    const int n = std::min(int(dynEqModel_.bands.size()), int(kNumMirroredBands));
    setParameterValue(kParamBandCount, float(n));
    setParameterValue(kParamOutputTrimDb, float(dynEqModel_.outputTrimDb));
    setParameterValue(kParamSoloedBand, float(dynEqModel_.soloedBand));
    for (int i = 0; i < int(kNumMirroredBands); ++i)
    {
        const ssfilter::DynamicEqBand2 b = (i < n) ? dynEqModel_.bands[size_t(i)] : ssfilter::DynamicEqBand2();
        setParameterValue(kParamBandFreqBase + i, float(b.freqHz));
        setParameterValue(kParamBandQBase + i, float(b.q));
        setParameterValue(kParamBandGainBase + i, float(b.gainDb));
        setParameterValue(kParamBandThresholdBase + i, float(b.thresholdDb));
        setParameterValue(kParamBandRatioBase + i, float(b.ratio));
        setParameterValue(kParamBandRatioUpBase + i, float(b.ratioUp));
        setParameterValue(kParamBandRangeBase + i, float(b.rangeDb));
        setParameterValue(kParamBandAttackBase + i, float(b.attackMs));
        setParameterValue(kParamBandReleaseBase + i, float(b.releaseMs));
        setParameterValue(kParamBandDynamicsEnabledBase + i, b.dynamicsEnabled ? 1.0f : 0.0f);
        setParameterValue(kParamBandEnabledBase + i, b.enabled ? 1.0f : 0.0f);
        setParameterValue(kParamBandChannelBase + i, float(int(b.channel)));
        setParameterValue(kParamBandCrossFeedBase + i, float(int(b.crossFeed)));
        setParameterValue(kParamBandHarmonicGroupBase + i, float(b.harmonicGroup));
        setParameterValue(kParamBandHarmonicRatioBase + i, float(b.harmonicRatio));
        setParameterValue(kParamBandTypeBase + i, float(int(b.type)));
        setParameterValue(kParamBandBandwidthBase + i, float(b.bandwidthOct));
    }
    // Model-level, not per-band -- see DynamicEqModel2::crossFeedGainLtoRDb's
    // own comment.
    setParameterValue(kParamCrossFeedGainLtoRDb, float(dynEqModel_.crossFeedGainLtoRDb));
    setParameterValue(kParamCrossFeedGainRtoLDb, float(dynEqModel_.crossFeedGainRtoLDb));
    // kParamBandBandwidthBase's own last slot is the very last field before
    // kParamMirrorCommit -- see applyMirrorStagingIfComplete()'s own
    // sentinel check.
}

void UIDynEq2::selectBand(int index)
{
    const int n = getBandCount();
    selectedBand_ = n <= 0 ? -1 : std::max(0, std::min(n - 1, index));
    graph_->setSelectedBand(selectedBand_);
    // Keeps engine_'s preview target in sync with the selection at all
    // times, not just while actually auditioning -- so flipping Audition on
    // later previews whatever's currently selected immediately, with no
    // stale target from a previous selection. Harmless while Audition is
    // off: engine_ only reads auditionBandIndex_ when audition_ is true.
    if (selectedBand_ >= 0) setParameterValue(kParamAuditionBand, float(selectedBand_));
    list_->setSelectedBand(selectedBand_);
    // Solo stays on the band whose S was clicked (the list shows it per
    // row); selecting another band doesn't move it.
    refreshAuditionColors();
}

// ---- ssgui::IDynEqEditorHost2 -----------------------------------------------

ssfilter::DynamicEqBand2 UIDynEq2::getBand(int index) const
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return ssfilter::DynamicEqBand2{};
    return dynEqModel_.bands[size_t(index)];
}

double UIDynEq2::getCurrentGainDb(int index) const
{
    if (index < 0 || index >= ssfilter::kMaxDynEqBands2) return 0.0;
    return double(currentGainDb_[index]);
}

void UIDynEq2::addBand(double freqHz, double gainDb)
{
    if (int(dynEqModel_.bands.size()) >= ssfilter::kMaxDynEqBands2) return; // silently refuse past the ceiling, same convention as mb::addCrossover()
    ssfilter::DynamicEqBand2 b;
    b.freqHz = std::max(20.0, std::min(20000.0, freqHz));
    b.gainDb = std::max(-24.0, std::min(24.0, gainDb));
    // A freshly-placed band starts *inert*, not already reacting at
    // DynamicEqBand2's own struct defaults (thresholdDb=-24, rangeDb=5) --
    // those are sensible values for a band you've already deliberately
    // configured, but a surprising one for a band you just clicked into
    // existence a moment ago, since gain would already be moving before
    // you've touched Threshold/Ratio/Range at all. Threshold at 0dB (a
    // level ordinary program material essentially never sustains) and
    // Range at 0dB (a hard ceiling of zero movement regardless of
    // threshold) together guarantee a new band behaves exactly like a
    // plain static bell until you actually dial dynamics in -- reported
    // directly: "for a new band setting the treshold for zero, range for
    // zero". dynamicsEnabled stays true (its own default): Range=0 alone
    // already makes it inert, so there's nothing extra Enabled=false would
    // add, and leaving it on means turning Range up is all that's needed
    // to bring the band to life.
    b.thresholdDb = 0.0;
    b.rangeDb = 0.0;
    dynEqModel_.bands.push_back(b);
    selectBand(int(dynEqModel_.bands.size()) - 1);
    applyModelToEngine();
}

void UIDynEq2::removeBand(int index)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands.erase(dynEqModel_.bands.begin() + index);
    selectBand(std::min(selectedBand_, getBandCount() - 1));
    applyModelToEngine();
}

void UIDynEq2::duplicateBand(int index)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    if (int(dynEqModel_.bands.size()) >= ssfilter::kMaxDynEqBands2) return;

    const ssfilter::DynamicEqBand2 copy = dynEqModel_.bands[size_t(index)];
    const int newIndex = index + 1;
    dynEqModel_.bands.insert(dynEqModel_.bands.begin() + newIndex, copy);

    // Select the new duplicate directly -- immediately visible/editable
    // without an extra click, same reasoning as the sibling EQ's own
    // UIEq::duplicateBand(). selectBand() also drags soloedBand along
    // with it if solo was active.
    selectBand(newIndex);
    applyModelToEngine();
}

void UIDynEq2::setBandFreq(int index, double freqHz)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(freqHz)) return;
    // Routed through dyneq2::propagateHarmonicFreq() rather than writing
    // freqHz directly -- if `index` is harmonically linked, this also
    // repositions every other member of its group -- see the sibling
    // UIEq::setBandFreq()'s identical comment. This plugin's own 20Hz
    // floor (unchanged from before linking existed) is applied first;
    // propagateHarmonicFreq()'s own [1Hz, Nyquist] clamp is a no-op on an
    // already-in-range value for any real sample rate.
    const double clamped = std::max(20.0, std::min(20000.0, freqHz));
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::dyneq2::propagateHarmonicFreq(dynEqModel_, index, clamped, sr);
    applyModelToEngine();
}

void UIDynEq2::setBandGain(int index, double gainDb)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(gainDb)) return;
    dynEqModel_.bands[size_t(index)].gainDb = std::max(-24.0, std::min(24.0, gainDb));
    applyModelToEngine();
}

void UIDynEq2::setBandQ(int index, double q)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(q)) return;
    dynEqModel_.bands[size_t(index)].q = std::max(0.05, std::min(100.0, q));
    applyModelToEngine();
}

void UIDynEq2::setBandBandwidth(int index, double octaves)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(octaves)) return;
    ssfilter::DynamicEqBand2& b = dynEqModel_.bands[size_t(index)];
    if (b.type != ssfilter::EqBandType::BandShelf) return;
    b.bandwidthOct = std::max(ssfilter::kMinBandwidthOct, std::min(ssfilter::kMaxBandwidthOct, octaves));
    applyModelToEngine();
}

void UIDynEq2::setBandThreshold(int index, double thresholdDb)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(thresholdDb)) return;
    dynEqModel_.bands[size_t(index)].thresholdDb = std::max(-72.0, std::min(0.0, thresholdDb));
    // Linked bands share Threshold/Ratio/RatioUp/Range -- fans this out to every other member of `index`'s
    // own harmonic group, a no-op if it isn't linked to anything. See
    // dyneq2::propagateHarmonicDynamics()'s own comment.
    ssfilter::dyneq2::propagateHarmonicDynamics(dynEqModel_, index);
    applyModelToEngine();
}

void UIDynEq2::setBandRatio(int index, double ratio)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(ratio)) return;
    dynEqModel_.bands[size_t(index)].ratio = std::max(1.0, std::min(20.0, ratio));
    ssfilter::dyneq2::propagateHarmonicDynamics(dynEqModel_, index);
    applyModelToEngine();
}

void UIDynEq2::setBandRatioUp(int index, double ratioUp)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(ratioUp)) return;
    dynEqModel_.bands[size_t(index)].ratioUp = std::max(1.0, std::min(20.0, ratioUp));
    ssfilter::dyneq2::propagateHarmonicDynamics(dynEqModel_, index);
    applyModelToEngine();
}

void UIDynEq2::setBandRange(int index, double rangeDb)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(rangeDb)) return;
    dynEqModel_.bands[size_t(index)].rangeDb = std::max(0.0, std::min(40.0, rangeDb));
    ssfilter::dyneq2::propagateHarmonicDynamics(dynEqModel_, index);
    applyModelToEngine();
}

void UIDynEq2::setBandAttack(int index, double attackMs)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(attackMs)) return;
    dynEqModel_.bands[size_t(index)].attackMs = std::max(0.1, std::min(200.0, attackMs));
    applyModelToEngine();
}

void UIDynEq2::setBandRelease(int index, double releaseMs)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(releaseMs)) return;
    dynEqModel_.bands[size_t(index)].releaseMs = std::max(5.0, std::min(2000.0, releaseMs));
    applyModelToEngine();
}

void UIDynEq2::setBandDynamicsEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].dynamicsEnabled = enabled;
    applyModelToEngine();
}

void UIDynEq2::setBandEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].enabled = enabled;
    applyModelToEngine();
}

void UIDynEq2::setBandSolo(int index, bool solo)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    // A single spotlight (DynamicEqModel2::soloedBand), not a per-band
    // flag -- `solo=false` only actually clears it if `index` is the band
    // that's currently soloed (matches the IDynEqEditorHost2 interface's
    // bool-per-band shape, kept for minimal disruption since this is its
    // only call site).
    if (solo) dynEqModel_.soloedBand = index;
    else if (dynEqModel_.soloedBand == index) dynEqModel_.soloedBand = -1;
    applyModelToEngine();
}

void UIDynEq2::setBandType(int index, ssfilter::EqBandType type)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].type = type;
    applyModelToEngine();
}

void UIDynEq2::setBandChannel(int index, ssfilter::EqChannel channel)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].channel = channel;
    applyModelToEngine();
}

void UIDynEq2::setBandCrossFeed(int index, ssfilter::EqCrossFeed crossFeed)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    // No band-local state to reset on this transition -- the crossfeed
    // transit's own send level lives on the MODEL (crossFeedGainLtoRDb/
    // RtoLDb), not per band, same as the sibling EQ's own
    // UIEq::setBandCrossFeed(). `channel` is left alone too; it's simply
    // ignored once crossFeed != None.
    dynEqModel_.bands[size_t(index)].crossFeed = crossFeed;
    applyModelToEngine();
}

void UIDynEq2::setCrossFeedGainLtoRDb(double gainDb)
{
    if (!std::isfinite(gainDb)) return;
    // Same -60..+12 clamp/reasoning as the sibling EQ's own
    // UIEq::setCrossFeedGainLtoRDb().
    dynEqModel_.crossFeedGainLtoRDb = std::max(-60.0, std::min(12.0, gainDb));
    applyModelToEngine();
}

void UIDynEq2::setCrossFeedGainRtoLDb(double gainDb)
{
    if (!std::isfinite(gainDb)) return;
    dynEqModel_.crossFeedGainRtoLDb = std::max(-60.0, std::min(12.0, gainDb));
    applyModelToEngine();
}

void UIDynEq2::createHarmonicTwin(int index, double factor)
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    const int newIndex = ssfilter::dyneq2::createHarmonicTwin(dynEqModel_, index, factor, sr);
    if (newIndex < 0) return; // refused -- band cap, or an invalid index/factor

    // Select the new twin directly -- immediately visible/editable
    // without an extra click, same reasoning as the sibling EQ's own
    // UIEq::createHarmonicTwin(). selectBand() also drags soloedBand
    // along with it if solo was active (see that function's own
    // comment) -- the same fixup this UI's own removeBand() already
    // relies on for an index-shifting edit.
    selectBand(newIndex);
    applyModelToEngine();
}

void UIDynEq2::unlinkHarmonic(int index)
{
    ssfilter::dyneq2::unlinkHarmonic(dynEqModel_, index);
    applyModelToEngine();
}

void UIDynEq2::beginHarmonicFactorEdit(int sourceIndex)
{
    if (sourceIndex < 0 || sourceIndex >= int(dynEqModel_.bands.size())) return;
    if (editTarget_ != EditTarget::None && editTarget_ != EditTarget::HarmonicFactor) commitEdit();
    editTarget_ = EditTarget::HarmonicFactor;
    pendingHarmonicSourceBand_ = sourceIndex;
    editBuffer_ = "2"; // x2, the most common case, as in UIEq
    getWindow().focus(); // see ../UIStateSpace.cpp's identical call for why
    repaint();
}

bool UIDynEq2::isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const
{
    if (editTarget_ != EditTarget::HarmonicFactor || pendingHarmonicSourceBand_ != sourceIndex) return false;
    liveText = editBuffer_;
    return true;
}

void UIDynEq2::commitHarmonicFactorEdit()
{
    if (editTarget_ == EditTarget::HarmonicFactor) commitEdit();
}

void UIDynEq2::beginBandAudition(int index)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    // Snapshot the persistent state before this hold touches kParamAudition
    // -- a host may echo the write back into auditioning_ before release
    // (see UIEq::beginBandAudition()).
    auditionHoldPrevState_ = auditioning_;
    setParameterValue(kParamAuditionBand, float(index));
    setParameterValue(kParamAudition, 1.0f);
}

void UIDynEq2::endBandAudition()
{
    if (selectedBand_ >= 0) setParameterValue(kParamAuditionBand, float(selectedBand_));
    setParameterValue(kParamAudition, auditionHoldPrevState_ ? 1.0f : 0.0f);
}

void UIDynEq2::beginBandTextEdit(int index, ssgui::DynEqField field, bool clearValue)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    if (editTarget_ != EditTarget::None &&
        !(editTarget_ == EditTarget::BandField && editBandIndex_ == index && editBandField_ == field))
        commitEdit();

    editTarget_ = EditTarget::BandField;
    editBandIndex_ = index;
    editBandField_ = field;

    if (clearValue) editBuffer_.clear();
    else
    {
        const ssfilter::DynamicEqBand2& b = dynEqModel_.bands[size_t(index)];
        double v = 0.0;
        switch (field)
        {
        case ssgui::DynEqField::Freq:      v = b.freqHz; break;
        case ssgui::DynEqField::Bandwidth: v = b.bandwidthOct; break;
        case ssgui::DynEqField::Gain:      v = b.gainDb; break;
        case ssgui::DynEqField::Q:         v = b.q; break;
        case ssgui::DynEqField::Threshold: v = b.thresholdDb; break;
        case ssgui::DynEqField::Ratio:     v = b.ratio; break;
        case ssgui::DynEqField::RatioUp:   v = b.ratioUp; break;
        case ssgui::DynEqField::Range:     v = b.rangeDb; break;
        case ssgui::DynEqField::Attack:    v = b.attackMs; break;
        case ssgui::DynEqField::Release:   v = b.releaseMs; break;
        }
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", v);
        editBuffer_ = buf;
    }
    getWindow().focus();
    repaint();
}

bool UIDynEq2::isEditingBandField(int index, ssgui::DynEqField field, std::string& liveText) const
{
    if (editTarget_ != EditTarget::BandField || editBandIndex_ != index || editBandField_ != field) return false;
    liveText = editBuffer_;
    return true;
}

void UIDynEq2::beginTrimTextEdit(bool clearValue)
{
    if (editTarget_ != EditTarget::None && editTarget_ != EditTarget::OutputTrim) commitEdit();
    editTarget_ = EditTarget::OutputTrim;
    if (clearValue) editBuffer_.clear();
    else
    {
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6g", dynEqModel_.outputTrimDb);
        editBuffer_ = buf;
    }
    getWindow().focus();
    repaint();
}

void UIDynEq2::beginSmoothTextEdit(bool clearValue)
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

bool UIDynEq2::checkInlineDoubleClick(int fieldId)
{
    const auto now = std::chrono::steady_clock::now();
    constexpr std::chrono::milliseconds kDoubleClickWindow(400);
    const bool isDoubleClick = fieldId == lastClickInlineField_ && (now - lastClickInlineTime_) < kDoubleClickWindow;
    lastClickInlineField_ = isDoubleClick ? -1 : fieldId;
    lastClickInlineTime_ = now;
    return isDoubleClick;
}

void UIDynEq2::commitEdit()
{
    if (editTarget_ == EditTarget::None) return;
    // std::from_chars is locale-independent (a host may set a comma-decimal
    // locale).
    double v = 0.0;
    const auto parseRes = std::from_chars(editBuffer_.data(), editBuffer_.data() + editBuffer_.size(), v);
    const bool valid = parseRes.ec == std::errc() && std::isfinite(v);
    const EditTarget target = editTarget_;
    const int bandIndex = editBandIndex_;
    const ssgui::DynEqField field = editBandField_;
    const int harmonicSourceBand = pendingHarmonicSourceBand_;
    editTarget_ = EditTarget::None;

    // The graph's custom-factor input closes its menu either way.
    if (target == EditTarget::HarmonicFactor) graph_->endHarmonicFactorEdit();

    if (!valid) { repaint(); return; }

    switch (target)
    {
    case EditTarget::BandField:
        // The setters bounds-check bandIndex (the band may have been
        // deleted meanwhile) and clamp the value.
        switch (field)
        {
        case ssgui::DynEqField::Freq:      setBandFreq(bandIndex, v); break;
        case ssgui::DynEqField::Bandwidth: setBandBandwidth(bandIndex, v); break;
        case ssgui::DynEqField::Gain:      setBandGain(bandIndex, v); break;
        case ssgui::DynEqField::Q:         setBandQ(bandIndex, v); break;
        case ssgui::DynEqField::Threshold: setBandThreshold(bandIndex, v); break;
        case ssgui::DynEqField::Ratio:     setBandRatio(bandIndex, v); break;
        case ssgui::DynEqField::RatioUp:   setBandRatioUp(bandIndex, v); break;
        case ssgui::DynEqField::Range:     setBandRange(bandIndex, v); break;
        case ssgui::DynEqField::Attack:    setBandAttack(bandIndex, v); break;
        case ssgui::DynEqField::Release:   setBandRelease(bandIndex, v); break;
        }
        break;
    case EditTarget::OutputTrim:
    case EditTarget::SpectrumSmoothMs:
        setInlineFieldValue(target, v);
        break;
    case EditTarget::HarmonicFactor:
        if (v > 0.0) createHarmonicTwin(harmonicSourceBand, v);
        break;
    case EditTarget::None:
        break;
    }
    repaint();
}

void UIDynEq2::cancelEdit()
{
    if (editTarget_ == EditTarget::HarmonicFactor) graph_->endHarmonicFactorEdit();
    editTarget_ = EditTarget::None;
    repaint();
}

void UIDynEq2::setInlineFieldValue(EditTarget which, double v)
{
    if (!std::isfinite(v)) return;
    if (which == EditTarget::OutputTrim)
    {
        dynEqModel_.outputTrimDb = std::max(-24.0, std::min(24.0, v));
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

// ---- keyboard / mouse -----------------------------------------------------

bool UIDynEq2::onCharacterInput(const CharacterInputEvent& ev)
{
    if (editTarget_ == EditTarget::None) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+')
    {
        if (editBuffer_.size() < 24) editBuffer_ += c;
        repaint();
    }
    return true;
}

bool UIDynEq2::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z / Ctrl+Y (Ctrl+Shift+Z) -- checked first, always consumed.
    if (ev.press && (ev.mod & DGL_NAMESPACE::kModifierControl) != 0
        && !(ev.mod & DGL_NAMESPACE::kModifierAlt) && !(ev.mod & DGL_NAMESPACE::kModifierSuper))
    {
        const bool shift = (ev.mod & DGL_NAMESPACE::kModifierShift) != 0;
        if (ev.key == 'z' && !shift) { onUndo(); return true; }
        if (ev.key == 'y' || (ev.key == 'z' && shift)) { onRedo(); return true; }
    }

    if (editTarget_ == EditTarget::None)
    {
        // Not typing: Delete/Backspace removes the selected band.
        if (ev.press && (ev.key == DGL_NAMESPACE::kKeyDelete || ev.key == DGL_NAMESPACE::kKeyBackspace))
        {
            if (selectedBand_ >= 0) { removeBand(selectedBand_); return true; }
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

bool UIDynEq2::onMouse(const MouseEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();
    auto inRect = [&](float fx, float fy, float fw, float fh) { return x >= fx && x < fx + fw && y >= fy && y < fy + fh; };
    const bool onTrimField = inRect(trimFieldX_, trimFieldY_, trimFieldW_, trimFieldH_);
    const bool onSmoothField = inRect(smoothFieldX_, smoothFieldY_, smoothFieldW_, smoothFieldH_);

    // A press anywhere else ends a text edit first (so finishing an edit
    // never depends on Enter reaching the plugin window). The graph's
    // custom-factor input is committed by the graph itself.
    if (editTarget_ != EditTarget::None && editTarget_ != EditTarget::HarmonicFactor && ev.press)
    {
        const bool clickingSameField = (editTarget_ == EditTarget::OutputTrim && onTrimField) ||
                                        (editTarget_ == EditTarget::SpectrumSmoothMs && onSmoothField);
        if (!clickingSameField) commitEdit();
    }

    // Release of an Output Trim / Smoothing press: a drag already applied
    // its value; a plain click opens the text edit.
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
            inlinePress_ = onTrimField ? EditTarget::OutputTrim : EditTarget::SpectrumSmoothMs;
            inlineDragging_ = false;
            inlineDragStartY_ = y;
            inlineDragStartValue_ = onTrimField ? dynEqModel_.outputTrimDb : spectrumSmoothMs_;
            return true;
        }

        if (inRect(spectrumModeFieldX_, spectrumModeFieldY_, spectrumModeFieldW_, spectrumModeFieldH_))
        {
            spectrumMode_ = ssfilter::nextSpectrumMode(spectrumMode_);
            graph_->setSpectrumMode(spectrumMode_);
            char buf[4];
            std::snprintf(buf, sizeof(buf), "%d", int(spectrumMode_));
            setState("spectrumMode", buf);
            repaint();
            return true;
        }

        if (inRect(fftSizeFieldX_, fftSizeFieldY_, fftSizeFieldW_, fftSizeFieldH_))
        {
            spectrumFftSizeIndex_ = (spectrumFftSizeIndex_ + 1) % ssfilter::SpectrumAnalyzer::kNumSizePresets;
            char buf[8];
            std::snprintf(buf, sizeof(buf), "%d", spectrumFftSizeIndex_);
            setState("spectrumFftSize", buf);
            repaint();
            return true;
        }
    }

    return UI::onMouse(ev);
}

bool UIDynEq2::onMotion(const MotionEvent& ev)
{
    if (inlinePress_ != EditTarget::None)
    {
        const double dy = inlineDragStartY_ - ev.pos.getY(); // up = increase
        if (!inlineDragging_ && std::fabs(dy) > 3.0)
        {
            inlineDragging_ = true;
            if (editTarget_ == inlinePress_) cancelEdit(); // the drag replaces a half-typed value
        }
        if (inlineDragging_)
        {
            constexpr double kPxFullRange = 200.0;
            if (inlinePress_ == EditTarget::OutputTrim)
                setInlineFieldValue(EditTarget::OutputTrim, inlineDragStartValue_ + dy / kPxFullRange * 48.0); // -24..+24dB
            else // 1..5000ms, log domain
                setInlineFieldValue(EditTarget::SpectrumSmoothMs, inlineDragStartValue_ * std::exp(dy / kPxFullRange * std::log(5000.0)));
        }
        return true;
    }
    return UI::onMotion(ev);
}

bool UIDynEq2::onScroll(const ScrollEvent& ev)
{
    const double x = ev.pos.getX(), y = ev.pos.getY();
    const bool onTrimField = x >= trimFieldX_ && x < trimFieldX_ + trimFieldW_ && y >= trimFieldY_ && y < trimFieldY_ + trimFieldH_;
    const bool onSmoothField = x >= smoothFieldX_ && x < smoothFieldX_ + smoothFieldW_ && y >= smoothFieldY_ && y < smoothFieldY_ + smoothFieldH_;
    if (onTrimField || onSmoothField)
    {
        const double dir = ev.delta.getY() > 0.0 ? 1.0 : (ev.delta.getY() < 0.0 ? -1.0 : 0.0);
        const bool fine = (ev.mod & DGL_NAMESPACE::kModifierAlt) != 0; // Alt = 10x finer
        if (onTrimField && dir != 0.0 && editTarget_ != EditTarget::OutputTrim)
            setInlineFieldValue(EditTarget::OutputTrim, dynEqModel_.outputTrimDb + dir * (fine ? 0.05 : 0.5));
        if (onSmoothField && dir != 0.0 && editTarget_ != EditTarget::SpectrumSmoothMs)
        {
            const double factor = fine ? 1.008 : 1.08;
            setInlineFieldValue(EditTarget::SpectrumSmoothMs, dir > 0.0 ? spectrumSmoothMs_ * factor : spectrumSmoothMs_ / factor);
        }
        return true;
    }
    return UI::onScroll(ev);
}

void UIDynEq2::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
    else if (op == PendingFileOp::Export) exportStateSpaceFile(filename);
}

void UIDynEq2::loadPresetFile(const char* path)
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

void UIDynEq2::applyModelSnapshot(const std::string& json)
{
    // Formerly loadPresetFile()'s own body past the file read -- factored
    // out so A/B and undo/redo (neither of which ever touches a file) can
    // reuse the exact same "parse and install a whole captured model"
    // logic Load already proved correct. A malformed/foreign string
    // (should never happen here -- every caller hands back exactly what
    // toJson() produced -- but fromJson() already guards file-sourced
    // input the same way) simply leaves the current model untouched.
    ssfilter::DynamicEqModel2 parsed;
    if (!ssfilter::dyneq2::fromJson(json, parsed)) return;

    dynEqModel_ = parsed;
    selectBand(getBandCount() > 0 ? 0 : -1);
    applyModelToEngine();
}

void UIDynEq2::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = ssfilter::dyneq2::toJson(dynEqModel_); abHasB_ = true; // save what we're leaving
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applyModelSnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; } // first-ever switch: A starts as a copy of the current model
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIDynEq2::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = ssfilter::dyneq2::toJson(dynEqModel_); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applyModelSnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIDynEq2::onAbCopy()
{
    const std::string snap = ssfilter::dyneq2::toJson(dynEqModel_);
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
    pushAbState();
}

void UIDynEq2::pushAbState()
{
    // See UIEq::pushAbState()'s identical comment (gui/eq/UIEq.cpp).
    if (abActive_ == AbSlot::A) abSlotA_ = ssfilter::dyneq2::toJson(dynEqModel_);
    else abSlotB_ = ssfilter::dyneq2::toJson(dynEqModel_);

    setState("abSlotA", abSlotA_.c_str());
    setState("abSlotB", abSlotB_.c_str());
    setState("abHasA", abHasA_ ? "1" : "0");
    setState("abHasB", abHasB_ ? "1" : "0");
    setState("abActive", abActive_ == AbSlot::B ? "1" : "0");
}

void UIDynEq2::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIDynEq2::onUndo()
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
    undoIdleTicksSinceChange_ = kUndoDebounceTicks; // already settled -- don't re-checkpoint the restore itself
    refreshUndoRedoColors();
}

void UIDynEq2::onRedo()
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

void UIDynEq2::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIDynEq2::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssdynjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::dyneq2::toJson(dynEqModel_);
    rememberPresetDir(p.c_str());
}

void UIDynEq2::exportStateSpaceFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::dyneq2::assembleStateSpace(dynEqModel_, sr);
    ssfilter::StabilityGuard::enforce(assembled);

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::Serialization::toJson(assembled);
    rememberPresetDir(p.c_str());
}

void UIDynEq2::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

UI* createUI() { return new UIDynEq2(); }

END_NAMESPACE_DISTRHO
