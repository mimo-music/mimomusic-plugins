// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIDynEq.hpp"

#include "../../plugin/dsp/Serialization.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../../plugin_dyn_eq/DistrhoPluginInfo.h"
#include "../../plugin_dyn_eq/PluginDynEq.hpp" // for ParameterIndex enum
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
using DGL_NAMESPACE::DynEqGraphWidget;
using DGL_NAMESPACE::ParamFieldSpec;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;

namespace {
constexpr int kDebounceIdleTicks = 3; // same debounce window as every sibling plugin's own engine push

enum GlobalField : uint32_t { kFieldOutputTrim = 0, kFieldSpectrumSmoothMs = 1 };
enum BandField : uint32_t {
    kFieldFreq = 0, kFieldGain = 1, kFieldQ = 2,
    kFieldThreshold = 3, kFieldRatio = 4, kFieldRange = 5, kFieldAttack = 6, kFieldRelease = 7, kFieldRatioUp = 8
};
} // namespace

UIDynEq::UIDynEq()
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

    for (float& v : currentGainDb_) v = 0.0f;
    for (float& v : spectrumOutRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutLSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;

    graph_ = new DynEqGraphWidget(this, this);
    graph_->setSampleRate(getSampleRate());
    graph_->setSpectrumMode(spectrumMode_);
    graph_->setSelectedBand(selectedBand_);
    graph_->setOnSelectionChanged([this](int i) { selectBand(i); });
    graph_->setOnSpectrumAutoscaleChanged([this](bool on) { setState("spectrumAutoscale", on ? "1" : "0"); });

    // Output Trim last -- the actual output-facing control in this group,
    // so it lands on the right side both within the strip and (per
    // layout()'s own repositioning) at the right end of its row.
    globalStrip_ = new ParamStripWidget(this);
    globalStrip_->setFields({
        { "Spectrum Smooth", "ms", 1.0f, 2000.0f, 150.0f, 10.0f, true, 0, kFieldSpectrumSmoothMs,
          "Spectrum Smooth: how heavily the background spectrum display is time-averaged." },
        { "Output Trim", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kFieldOutputTrim,
          "Output Trim: a manual master gain applied after every band." },
    });
    globalStrip_->setGetter([this](uint32_t id) -> float {
        switch (id)
        {
        case kFieldOutputTrim: return float(dynEqModel_.outputTrimDb);
        case kFieldSpectrumSmoothMs: return float(spectrumSmoothMs_);
        }
        return 0.0f;
    });
    globalStrip_->setSetter([this](uint32_t id, float v) {
        switch (id)
        {
        case kFieldOutputTrim:
            dynEqModel_.outputTrimDb = std::max(-24.0, std::min(24.0, double(v)));
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
        { "Freq", "Hz", 20.0f, 20000.0f, 1000.0f, 10.0f, true, 0, kFieldFreq,
          "Freq: the selected band's own center frequency." },
        { "Gain", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kFieldGain,
          "Gain: the selected band's own resting boost/cut, before any dynamics pull it back." },
        { "Q", "", 0.05f, 100.0f, 1.0f, 1.08f, true, 2, kFieldQ,
          "Q: how narrow or wide the selected band's own bell shape is." },
        { "Threshold", "dB", -72.0f, 0.0f, -24.0f, 0.5f, false, 1, kFieldThreshold,
          "Threshold: the level (at this band's own frequency) above which its gain starts pulling back." },
        { "Ratio", ":1", 1.0f, 20.0f, 4.0f, 0.1f, true, 1, kFieldRatio,
          "Ratio: how strongly this band's own gain is pulled back once past the threshold." },
        { "Up Ratio", ":1", 1.0f, 20.0f, 1.0f, 0.1f, true, 1, kFieldRatioUp,
          "Up Ratio: how strongly this band's own gain is boosted further while below the threshold." },
        { "Range", "dB", 0.0f, 40.0f, 5.0f, 0.5f, false, 1, kFieldRange,
          "Range: the maximum this band's own dynamic gain change can reach, in either direction." },
        { "Attack", "ms", 0.1f, 200.0f, 10.0f, 1.0f, true, 2, kFieldAttack,
          "Attack: how quickly this band's own dynamics engage once its threshold is crossed." },
        { "Release", "ms", 5.0f, 2000.0f, 100.0f, 5.0f, true, 1, kFieldRelease,
          "Release: how quickly this band's own dynamics relax back once level drops again." },
    });
    bandStrip_->setGetter([this](uint32_t id) -> float {
        const ssfilter::DynamicEqBand b = getBand(selectedBand_);
        switch (id)
        {
        case kFieldFreq: return float(b.freqHz);
        case kFieldGain: return float(b.gainDb);
        case kFieldQ: return float(b.q);
        case kFieldThreshold: return float(b.thresholdDb);
        case kFieldRatio: return float(b.ratio);
        case kFieldRatioUp: return float(b.ratioUp);
        case kFieldRange: return float(b.rangeDb);
        case kFieldAttack: return float(b.attackMs);
        case kFieldRelease: return float(b.releaseMs);
        }
        return 0.0f;
    });
    bandStrip_->setSetter([this](uint32_t id, float v) {
        switch (id)
        {
        case kFieldFreq: setBandFreq(selectedBand_, v); break;
        case kFieldGain: setBandGain(selectedBand_, v); break;
        case kFieldQ: setBandQ(selectedBand_, v); break;
        case kFieldThreshold: setBandThreshold(selectedBand_, v); break;
        case kFieldRatio: setBandRatio(selectedBand_, v); break;
        case kFieldRatioUp: setBandRatioUp(selectedBand_, v); break;
        case kFieldRange: setBandRange(selectedBand_, v); break;
        case kFieldAttack: setBandAttack(selectedBand_, v); break;
        case kFieldRelease: setBandRelease(selectedBand_, v); break;
        }
    });

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load Dynamic EQ Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "dyn_eq_preset.ssdynjson";
        opts.title = "Save Dynamic EQ Preset";
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

    btnDeleteBand_ = new SimpleButton(this, "Delete Band");
    btnDeleteBand_->setCallback([this] { if (selectedBand_ >= 0) removeBand(selectedBand_); });

    btnPrevBand_ = new SimpleButton(this, "<");
    btnPrevBand_->setCallback([this] { selectBand(selectedBand_ - 1); });
    btnNextBand_ = new SimpleButton(this, ">");
    btnNextBand_->setCallback([this] { selectBand(selectedBand_ + 1); });

    btnEnabled_ = new SimpleButton(this, "Enabled");
    btnEnabled_->setCallback([this] {
        if (selectedBand_ < 0) return;
        setBandEnabled(selectedBand_, !getBand(selectedBand_).enabled);
        refreshBandStripLabels();
    });
    btnSolo_ = new SimpleButton(this, "Solo");
    btnSolo_->setCallback([this] {
        if (selectedBand_ < 0) return;
        // A single spotlight, not a per-band flag -- see
        // DynamicEqModel::soloedBand's own comment.
        setBandSolo(selectedBand_, dynEqModel_.soloedBand != selectedBand_);
        refreshBandStripLabels();
    });
    btnDynamics_ = new SimpleButton(this, "Dynamics");
    btnDynamics_->setCallback([this] {
        if (selectedBand_ < 0) return;
        setBandDynamicsEnabled(selectedBand_, !getBand(selectedBand_).dynamicsEnabled);
        refreshBandStripLabels();
    });
    // Click-to-cycle Both/Left Only/Right Only, same convention as the
    // Parametric EQ's own Channel column. No right-click menu or list column
    // here (this UI has neither, unlike the Parametric EQ's BandListWidget
    // -- see this file's own top comment on why every per-band control is
    // a toolbar button re-targeted at selectedBand_ instead), so this
    // button is the one place it's reachable.
    btnChannel_ = new SimpleButton(this, "Ch: Both");
    btnChannel_->setCallback([this] {
        if (selectedBand_ < 0) return;
        setBandChannel(selectedBand_, ssfilter::nextEqChannel(getBand(selectedBand_).channel));
        refreshBandStripLabels();
    });

    // Audition -- same idea as the (now-deleted) De-esser plugin's own
    // "Listen" (see PluginDynEq.hpp's kParamAudition comment for why this
    // was renamed): preview selectedBand_'s isolated detector output,
    // with its own live gain reduction applied, for tuning by ear.
    // Grouped with the other per-band toggles rather than the top
    // toolbar's Bypass since it only ever acts on whichever band is
    // currently selected -- refreshAuditionColors() grays it out (and
    // forces Audition back off) the moment there's no band to preview.
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
    tooltip_->addTarget(btnLoad_, "Load a saved Dynamic EQ preset from disk.");
    tooltip_->addTarget(btnSave_, "Save the current settings as a Dynamic EQ preset.");
    tooltip_->addTarget(btnExport_, "Export the current resting-gain shape as a plain State-Space filter (.ssjson).");
    tooltip_->addTarget(btnAddBand_, "Add a new band in the widest still-empty gap between existing bands.");
    tooltip_->addTarget(btnDeleteBand_, "Delete the currently selected band.");
    tooltip_->addTarget(btnPrevBand_, "Select the previous band.");
    tooltip_->addTarget(btnNextBand_, "Select the next band.");
    tooltip_->addTarget(btnEnabled_, "Enable or disable the selected band entirely.");
    tooltip_->addTarget(btnSolo_, "Solo the selected band -- mutes every other band's own contribution.");
    tooltip_->addTarget(btnDynamics_, "Toggle whether the selected band's own dynamics (Threshold/Ratio/Range/Attack/Release) are active.");
    tooltip_->addTarget(btnChannel_, "Cycle which channel the selected band applies to: Both, Left only, or Right only.");
    tooltip_->addTarget(btnAudition_, "Audition the selected band's own isolated detector signal, with its live gain reduction applied.");
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

UIDynEq::~UIDynEq()
{
    // See ../eq/UIEq.cpp's identical destructor for the full reasoning --
    // closing the editor before uiIdle()'s debounce fires would otherwise
    // silently drop the most recent edit.
    if (modelDirty_) pushModelNow();
}

void UIDynEq::layout()
{
    // Row 1: every system-wide/whole-instance control (file ops -> Reset/
    // Bypass -> A/B compare -> Undo/Redo), flush against the window's own
    // top edge, compare/history right-aligned so that group always reads
    // in the same corner across every plugin in this family. Everything
    // below row 1 belongs to *this* filter specifically -- band
    // add/delete/navigation/toggles, the band's own parameter strip, and
    // the global spectrum/output-trim fields -- and sits as close to the
    // graph it affects as the layout allows, not mixed into the
    // system-wide row above it.
    const float w = float(getWidth());
    const float h = float(getHeight());

    // Covers the whole window -- see TooltipOverlay's own file comment.
    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));

    constexpr float pad = 6.0f;
    constexpr float rowH = 28.0f;
    constexpr float stripH = 46.0f;
    // The global-fields row (Output Trim/Spectrum Smooth) needs the same
    // height as bandStrip_'s own row, not the shorter button-row height --
    // ParamStripWidget draws two stacked lines per field (value, then its
    // label underneath), and at rowH's 28px that pair had nowhere near
    // enough room and visibly overlapped.
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

    // Row 2: this filter's own band management, right above its own
    // parameter strip (row 3) and the graph both act on.
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
    place(btnDynamics_, 78);
    place(btnChannel_, 68);
    place(btnAudition_, 68);

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

    // Output Trim/Spectrum Smooth last -- the output-facing control (Output
    // Trim, the group's rightmost field per its own setFields() above) now
    // sits at the right end of the row instead of the left.
    globalStrip_->setAbsolutePos(int(bx), int(by));
    globalStrip_->setSize(uint(280.0f), uint(stripH));

    const float contentTop = toolbarH + pad;
    const float contentH = h - contentTop - pad;
    graph_->setAbsolutePos(int(pad), int(contentTop));
    graph_->setSize(uint(w - 2.0f * pad), uint(contentH));
}

void UIDynEq::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIDynEq::refreshBandStripLabels()
{
    const bool haveBand = selectedBand_ >= 0 && selectedBand_ < getBandCount();
    const ssfilter::DynamicEqBand b = getBand(selectedBand_);
    const bool soloed = haveBand && dynEqModel_.soloedBand == selectedBand_; // a single spotlight, not a per-band flag -- see soloedBand's own comment
    btnEnabled_->setColors((haveBand && b.enabled) ? Color(58, 62, 70) : Color(150, 90, 40),
                            (haveBand && b.enabled) ? Color(78, 84, 96) : Color(175, 110, 55),
                            Color(230, 232, 236));
    btnSolo_->setColors(soloed ? Color(180, 160, 40) : Color(58, 62, 70),
                         soloed ? Color(205, 185, 55) : Color(78, 84, 96),
                         soloed ? Color(20, 20, 22) : Color(230, 232, 236));
    btnDynamics_->setColors((haveBand && b.dynamicsEnabled) ? Color(60, 110, 150) : Color(58, 62, 70),
                             (haveBand && b.dynamicsEnabled) ? Color(75, 130, 175) : Color(78, 84, 96),
                             Color(230, 232, 236));

    // Channel -- same blue/red this project now uses everywhere a channel
    // needs distinguishing (the graph's own per-channel spectrum traces/
    // curves/node rings), "Ch: Both" reading exactly like the other
    // buttons' own neutral coloring when there's nothing special to flag.
    {
        const bool isLeft = haveBand && b.channel == ssfilter::EqChannel::Left;
        const bool isRight = haveBand && b.channel == ssfilter::EqChannel::Right;
        btnChannel_->setLabel(isLeft ? "Ch: Left" : isRight ? "Ch: Right" : "Ch: Both");
        btnChannel_->setColors(isLeft ? Color(55, 90, 160) : isRight ? Color(150, 60, 50) : Color(58, 62, 70),
                                isLeft ? Color(70, 110, 185) : isRight ? Color(175, 75, 65) : Color(78, 84, 96),
                                Color(230, 232, 236));
        btnChannel_->setEnabled(haveBand);
    }

    // The band Audition was previewing may just have been deleted (or every
    // band removed) -- don't leave it stuck "on" with nothing left to
    // preview.
    if (!haveBand && auditioning_)
    {
        auditioning_ = false;
        setParameterValue(kParamAudition, 0.0f);
    }
    refreshAuditionColors();

    repaint();
}

void UIDynEq::refreshAuditionColors()
{
    // Same blue-ish "monitoring, not the real signal" tint the (now-
    // deleted) De-esser plugin's own "Listen" button used when engaged.
    btnAudition_->setColors(auditioning_ ? Color(70, 130, 160) : Color(58, 62, 70),
                             auditioning_ ? Color(90, 150, 180) : Color(78, 84, 96),
                             Color(230, 232, 236));
    // Nothing to preview with no band selected -- grayed out and
    // unclickable (SimpleButton::setEnabled() already does both), same
    // convention as Compressor Mono/Stereo's own Recalibrate button when
    // Volume Compensation is off.
    btnAudition_->setEnabled(selectedBand_ >= 0);
}

void UIDynEq::onNanoDisplay()
{
    const float w = float(getWidth());

    beginPath();
    rect(0, 0, w, float(getHeight()));
    fillColor(Color(18, 19, 23));
    fill();

    char bandBuf[32];
    if (getBandCount() > 0)
        std::snprintf(bandBuf, sizeof(bandBuf), "Band %d of %d", selectedBand_ + 1, getBandCount());
    else
        std::snprintf(bandBuf, sizeof(bandBuf), "No bands");
    fontSize(13.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    fillColor(Color(220, 223, 228));
    text(bandLabelX_ + bandLabelW_ * 0.5f, bandLabelY_ + 12.0f, bandBuf, nullptr);

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
    // live engine -- see DynamicEqModel.hpp's file comment). Removing the
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

void UIDynEq::uiIdle()
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
        const std::string nowJson = ssfilter::dyneq::toJson(dynEqModel_);
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

void UIDynEq::parameterChanged(uint32_t index, float value)
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
        if (index >= kParamCurrentGainBase && index < kParamCurrentGainBase + uint32_t(ssfilter::kMaxDynEqBands))
        {
            // Smoothed against wall-clock time before being stored -- see
            // currentGainDb_'s own comment in UIDynEq.hpp for why (the
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

void UIDynEq::applyMirrorStagingIfComplete(uint32_t index, float value)
{
    // Same index arithmetic as PluginDynEq::setParameterValue()'s own
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
    else
        return; // not a mirror field at all

    // The last field of the last slot -- always the last to arrive within
    // one delivery burst, see kParamMirrorCommit's own comment
    // (PluginDynEq.hpp).
    if (index != kParamBandChannelBase + kNumMirroredBands - 1) return;
    if (modelDirty_) return; // a newer local edit is still pending its own debounced push

    ssfilter::DynamicEqModel parsed;
    parsed.bands.assign(mirrorStaging_, mirrorStaging_ + mirrorStagingCount_);
    parsed.outputTrimDb = mirrorStagingTrim_;
    parsed.soloedBand = mirrorStagingSoloedBand_;
    dynEqModel_ = parsed;
    selectedBand_ = getBandCount() > 0 ? std::max(0, std::min(getBandCount() - 1, selectedBand_)) : -1;
    graph_->setSelectedBand(selectedBand_);
    if (selectedBand_ >= 0) setParameterValue(kParamAuditionBand, float(selectedBand_)); // see selectBand()'s own comment
    refreshBandStripLabels();
    repaint();
}

void UIDynEq::stateChanged(const char* key, const char* value)
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

void UIDynEq::applyModelToEngine()
{
    modelDirty_ = true;
    repaint();
}

void UIDynEq::pushModelNow()
{
    pushBandParamMirror();
    // Must toggle, not resend a constant -- see mirrorCommitToggle_'s own
    // comment (UIDynEq.hpp) -- and must come after every mirror field above.
    mirrorCommitToggle_ = 1.0f - mirrorCommitToggle_;
    setParameterValue(kParamMirrorCommit, mirrorCommitToggle_);
    pushAbState(); // keeps the *active* A/B slot's own persisted copy in sync with this edit -- see its own comment
    repaint();
}

void UIDynEq::pushBandParamMirror()
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
        const ssfilter::DynamicEqBand b = (i < n) ? dynEqModel_.bands[size_t(i)] : ssfilter::DynamicEqBand();
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
    }
}

void UIDynEq::selectBand(int index)
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

    // Solo is a single spotlight, not a per-band setting (see
    // DynamicEqModel::soloedBand's own comment) -- if it's currently
    // engaged, moving the selection moves the spotlight with it. See
    // ../mb/UIMbCompressor.cpp's identical selectBand() for the full
    // reasoning (this is the one chokepoint every selection change
    // already funnels through).
    if (dynEqModel_.soloedBand >= 0 && dynEqModel_.soloedBand != selectedBand_ && selectedBand_ >= 0)
    {
        dynEqModel_.soloedBand = selectedBand_;
        applyModelToEngine();
    }
    refreshBandStripLabels();
}

// ---- ssgui::IDynEqEditorHost -----------------------------------------------

ssfilter::DynamicEqBand UIDynEq::getBand(int index) const
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return ssfilter::DynamicEqBand{};
    return dynEqModel_.bands[size_t(index)];
}

double UIDynEq::getCurrentGainDb(int index) const
{
    if (index < 0 || index >= ssfilter::kMaxDynEqBands) return 0.0;
    return double(currentGainDb_[index]);
}

void UIDynEq::addBand(double freqHz, double gainDb)
{
    if (int(dynEqModel_.bands.size()) >= ssfilter::kMaxDynEqBands) return; // silently refuse past the ceiling, same convention as mb::addCrossover()
    ssfilter::DynamicEqBand b;
    b.freqHz = std::max(20.0, std::min(20000.0, freqHz));
    b.gainDb = std::max(-24.0, std::min(24.0, gainDb));
    // A freshly-placed band starts *inert*, not already reacting at
    // DynamicEqBand's own struct defaults (thresholdDb=-24, rangeDb=5) --
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

void UIDynEq::removeBand(int index)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands.erase(dynEqModel_.bands.begin() + index);
    selectBand(std::min(selectedBand_, getBandCount() - 1));
    applyModelToEngine();
}

void UIDynEq::setBandFreq(int index, double freqHz)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(freqHz)) return;
    dynEqModel_.bands[size_t(index)].freqHz = std::max(20.0, std::min(20000.0, freqHz));
    applyModelToEngine();
}

void UIDynEq::setBandGain(int index, double gainDb)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(gainDb)) return;
    dynEqModel_.bands[size_t(index)].gainDb = std::max(-24.0, std::min(24.0, gainDb));
    applyModelToEngine();
}

void UIDynEq::setBandQ(int index, double q)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(q)) return;
    dynEqModel_.bands[size_t(index)].q = std::max(0.05, std::min(100.0, q));
    applyModelToEngine();
}

void UIDynEq::setBandThreshold(int index, double thresholdDb)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(thresholdDb)) return;
    dynEqModel_.bands[size_t(index)].thresholdDb = std::max(-72.0, std::min(0.0, thresholdDb));
    applyModelToEngine();
}

void UIDynEq::setBandRatio(int index, double ratio)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(ratio)) return;
    dynEqModel_.bands[size_t(index)].ratio = std::max(1.0, std::min(20.0, ratio));
    applyModelToEngine();
}

void UIDynEq::setBandRatioUp(int index, double ratioUp)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(ratioUp)) return;
    dynEqModel_.bands[size_t(index)].ratioUp = std::max(1.0, std::min(20.0, ratioUp));
    applyModelToEngine();
}

void UIDynEq::setBandRange(int index, double rangeDb)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(rangeDb)) return;
    dynEqModel_.bands[size_t(index)].rangeDb = std::max(0.0, std::min(40.0, rangeDb));
    applyModelToEngine();
}

void UIDynEq::setBandAttack(int index, double attackMs)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(attackMs)) return;
    dynEqModel_.bands[size_t(index)].attackMs = std::max(0.1, std::min(200.0, attackMs));
    applyModelToEngine();
}

void UIDynEq::setBandRelease(int index, double releaseMs)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size()) || !std::isfinite(releaseMs)) return;
    dynEqModel_.bands[size_t(index)].releaseMs = std::max(5.0, std::min(2000.0, releaseMs));
    applyModelToEngine();
}

void UIDynEq::setBandDynamicsEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].dynamicsEnabled = enabled;
    applyModelToEngine();
}

void UIDynEq::setBandEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].enabled = enabled;
    applyModelToEngine();
}

void UIDynEq::setBandSolo(int index, bool solo)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    // A single spotlight (DynamicEqModel::soloedBand), not a per-band
    // flag -- `solo=false` only actually clears it if `index` is the band
    // that's currently soloed (matches the IDynEqEditorHost interface's
    // bool-per-band shape, kept for minimal disruption since this is its
    // only call site).
    if (solo) dynEqModel_.soloedBand = index;
    else if (dynEqModel_.soloedBand == index) dynEqModel_.soloedBand = -1;
    applyModelToEngine();
}

void UIDynEq::setBandChannel(int index, ssfilter::EqChannel channel)
{
    if (index < 0 || index >= int(dynEqModel_.bands.size())) return;
    dynEqModel_.bands[size_t(index)].channel = channel;
    applyModelToEngine();
}

// ---- keyboard / mouse -----------------------------------------------------

bool UIDynEq::onCharacterInput(const CharacterInputEvent& ev)
{
    ParamStripWidget* editing = globalStrip_->isEditing() ? globalStrip_ : (bandStrip_->isEditing() ? bandStrip_ : nullptr);
    if (editing == nullptr) return false;
    editing->appendChar(ev.string[0]);
    return true;
}

bool UIDynEq::onKeyboard(const KeyboardEvent& ev)
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

    ParamStripWidget* editing = globalStrip_->isEditing() ? globalStrip_ : (bandStrip_->isEditing() ? bandStrip_ : nullptr);

    if (editing == nullptr)
    {
        if (ev.press && (ev.key == DGL_NAMESPACE::kKeyDelete || ev.key == DGL_NAMESPACE::kKeyBackspace))
        {
            if (selectedBand_ >= 0) removeBand(selectedBand_);
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

bool UIDynEq::onMouse(const MouseEvent& rawEv)
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
    }

    return UI::onMouse(rawEv);
}

void UIDynEq::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
    else if (op == PendingFileOp::Export) exportStateSpaceFile(filename);
}

void UIDynEq::loadPresetFile(const char* path)
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

void UIDynEq::applyModelSnapshot(const std::string& json)
{
    // Formerly loadPresetFile()'s own body past the file read -- factored
    // out so A/B and undo/redo (neither of which ever touches a file) can
    // reuse the exact same "parse and install a whole captured model"
    // logic Load already proved correct. A malformed/foreign string
    // (should never happen here -- every caller hands back exactly what
    // toJson() produced -- but fromJson() already guards file-sourced
    // input the same way) simply leaves the current model untouched.
    ssfilter::DynamicEqModel parsed;
    if (!ssfilter::dyneq::fromJson(json, parsed)) return;

    dynEqModel_ = parsed;
    selectBand(getBandCount() > 0 ? 0 : -1);
    applyModelToEngine();
}

void UIDynEq::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = ssfilter::dyneq::toJson(dynEqModel_); abHasB_ = true; // save what we're leaving
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applyModelSnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; } // first-ever switch: A starts as a copy of the current model
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIDynEq::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = ssfilter::dyneq::toJson(dynEqModel_); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applyModelSnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
    pushAbState();
}

void UIDynEq::onAbCopy()
{
    const std::string snap = ssfilter::dyneq::toJson(dynEqModel_);
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
    pushAbState();
}

void UIDynEq::pushAbState()
{
    // See UIEq::pushAbState()'s identical comment (gui/eq/UIEq.cpp).
    if (abActive_ == AbSlot::A) abSlotA_ = ssfilter::dyneq::toJson(dynEqModel_);
    else abSlotB_ = ssfilter::dyneq::toJson(dynEqModel_);

    setState("abSlotA", abSlotA_.c_str());
    setState("abSlotB", abSlotB_.c_str());
    setState("abHasA", abHasA_ ? "1" : "0");
    setState("abHasB", abHasB_ ? "1" : "0");
    setState("abActive", abActive_ == AbSlot::B ? "1" : "0");
}

void UIDynEq::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIDynEq::onUndo()
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

void UIDynEq::onRedo()
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

void UIDynEq::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIDynEq::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssdynjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::dyneq::toJson(dynEqModel_);
    rememberPresetDir(p.c_str());
}

void UIDynEq::exportStateSpaceFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssjson";
    if (p.size() < sizeof(kExt) - 1 || p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::StateSpaceMatrices assembled = ssfilter::dyneq::assembleStateSpace(dynEqModel_, sr);
    ssfilter::StabilityGuard::enforce(assembled);

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::Serialization::toJson(assembled);
    rememberPresetDir(p.c_str());
}

void UIDynEq::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    setState("presetDir", lastPresetDir_.c_str());
}

void UIDynEq::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIDynEq(); }

END_NAMESPACE_DISTRHO
