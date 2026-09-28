// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// UIMbCompressor.hpp
//
// Top-level UI for the multiband compressor plugin -- owns the actual
// ssfilter::MbCompressorModel, implements ssgui::IMbEditorHost for
// MultibandGraphWidget, and drives two ParamStripWidget instances (ported
// from ../compressor, see gui/ParamStripWidget.hpp) for the click-to-edit/
// drag-fader/scroll-nudge numeric fields: one for the always-present
// Output Trim field, one for the currently *selected* band's Threshold/
// Ratio/Attack/Release/Makeup -- selecting a different band (click its
// region on the graph, or the Prev/Next buttons) just re-points the same
// strip at a different band index, rather than showing every band's
// fields at once the way the sibling EQ plugin's BandListWidget does (a
// deliberate simplification: a compressor band has 5 numeric fields where
// an EQ band has 3, and the workflow is "select a band, then set it up"
// -- a per-band-row list is listed under "Possible next steps" in
// docs/EXTENDING.md).
//
// Load/Save (".ssmbjson") and "Export to State-Space..." (an ordinary
// .ssjson, via Serialization::toJson(assembleStateSpace(...))) follow
// UIEq's own file-dialog pattern exactly -- see that file's own comment.
// The background spectrum (pre-/post-compression) is fed to
// MultibandGraphWidget the same way UIEq feeds EqGraphWidget: raw values
// arrive via parameterChanged() (one DPF output Parameter per band per
// analyzer), smoothed here every uiIdle() tick.

#pragma once

#include "DistrhoUI.hpp"

#include "../../plugin/dsp/MbCompressorModel.hpp"
#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../../plugin_mb_compressor/PluginMbCompressor.hpp" // for kNumMirroredBands
#include "../ParamStripWidget.hpp"
#include "../SimpleButton.hpp"
#include "../TooltipOverlay.hpp"
#include "MbEditorHost.hpp"
#include "MultibandGraphWidget.hpp"

#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

class UIMbCompressor : public UI, public ssgui::IMbEditorHost
{
public:
    UIMbCompressor();
    // Flushes a still-debounced edit (modelDirty_) straight to the DSP
    // before this UI is torn down -- see ../eq/UIEq.cpp's identical
    // destructor for the full reasoning.
    ~UIMbCompressor() override;

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;
    bool onMouse(const MouseEvent& ev) override;

    void uiFileBrowserSelected(const char* filename) override;

    // -- ssgui::IMbEditorHost ----------------------------------------------
    int getBandCount() const override { return int(mbModel_.bands.size()); }
    ssfilter::MbBand getBand(int index) const override;
    ssfilter::MbCompressorModel getModel() const override { return mbModel_; }
    double getCrossoverHz(int crossoverIndex) const override;
    void setCrossoverHz(int crossoverIndex, double freqHz) override;
    void addCrossover(double freqHz) override;
    void removeCrossover(int crossoverIndex) override;
    void setBandThreshold(int index, double thresholdDb) override;
    void setBandRatio(int index, double ratio) override;
    void setBandRatioUp(int index, double ratioUp) override;
    void setBandAttack(int index, double attackMs) override;
    void setBandRelease(int index, double releaseMs) override;
    void setBandMakeup(int index, double makeupDb) override;
    void setBandEnabled(int index, bool enabled) override;
    void setBandSolo(int index, bool solo) override;

private:
    void layout();
    void applyModelToEngine();  // marks the model dirty; the actual assemble+enforce+push is debounced, see uiIdle()
    void pushModelNow();
    // See UIEq::pushAbState()'s identical declaration comment (gui/eq/UIEq.hpp).
    void pushAbState();
    // The Parameter mirror push -- see UIEq::pushBandParamMirror()'s
    // identical declaration comment (gui/eq/UIEq.hpp) for the full reasoning.
    void pushBandParamMirror();
    // The read side -- see UIEq::applyMirrorStagingIfComplete()'s identical
    // declaration comment (gui/eq/UIEq.hpp) for the full reasoning.
    void applyMirrorStagingIfComplete(uint32_t index, float value);
    void selectBand(int index);
    void removeSelectedBand();
    void refreshBandStripLabels(); // enable/solo button colors + selected-band readout text

    void loadPresetFile(const char* path);
    void savePresetFile(const char* path);
    void exportStateSpaceFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- see ../../../compressor's
    // UICompressorMono.cpp for the full design writeup this is ported
    // from (against this plugin's own JSON model, same idiom as
    // ../eq/UIEq.hpp/../dyneq/UIDynEq.hpp).
    void applyModelSnapshot(const std::string& json);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    ssfilter::MbCompressorModel mbModel_;
    ssfilter::StabilityReport lastReport_;
    int selectedBand_ = 0;

    // Staging area for applyMirrorStagingIfComplete() -- see UIEq.hpp's
    // identical members for the full reasoning.
    ssfilter::MbBand mirrorStaging_[kNumMirroredBands];
    double mirrorStagingCrossoverHz_[kNumMirroredBands - 1];
    int mirrorStagingCount_ = 0;
    double mirrorStagingTrim_ = 0.0;
    int mirrorStagingLrOrder_ = 4;
    int mirrorStagingSoloedBand_ = -1;

    DGL_NAMESPACE::MultibandGraphWidget* graph_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* globalStrip_ = nullptr; // Output Trim
    DGL_NAMESPACE::ParamStripWidget* bandStrip_ = nullptr;   // Threshold/Ratio/Attack/Release/Makeup, for selectedBand_

    DGL_NAMESPACE::SimpleButton* btnLoad_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnExport_     = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAddBand_    = nullptr;
    DGL_NAMESPACE::SimpleButton* btnDeleteBand_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnPrevBand_   = nullptr;
    DGL_NAMESPACE::SimpleButton* btnNextBand_   = nullptr;
    DGL_NAMESPACE::SimpleButton* btnEnabled_    = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSolo_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnReset_      = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_     = nullptr;

    DGL_NAMESPACE::SimpleButton* btnAbA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbB_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbCopy_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnUndo_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRedo_ = nullptr;

    // Hover descriptions (ported verbatim from the sibling ../compressor
    // project's own TooltipOverlay) -- constructed last so it always
    // paints on top.
    DGL_NAMESPACE::TooltipOverlay* tooltip_ = nullptr;
    enum class AbSlot { A, B };
    AbSlot abActive_ = AbSlot::A;
    std::string abSlotA_, abSlotB_;
    bool abHasA_ = false, abHasB_ = false;

    std::vector<std::string> undoStack_, redoStack_;
    std::string lastCommittedJson_, lastSeenJson_;
    int undoIdleTicksSinceChange_ = 0;
    bool undoRedoInitialized_ = false;
    bool applyingSnapshot_ = false;
    static constexpr int kUndoDebounceTicks = 15;
    static constexpr size_t kMaxUndoDepth = 100;

    // Selected-band readout text ("Band X of N"), between the Prev/Next
    // buttons -- drawn directly in onNanoDisplay(), same reasoning as
    // UIEq's hand-drawn inline field labels (SimpleButton has no label
    // mutator, and this isn't a button anyway).
    float bandLabelX_ = 0, bandLabelY_ = 0, bandLabelW_ = 0;

    // Spectrum Source / Spectrum Resolution: inline click-to-cycle fields,
    // same convention (and reasoning -- four/several short names read
    // better as a labeled value box than crammed into a button) as UIEq's
    // identical pair.
    float spectrumModeFieldX_ = 0, spectrumModeFieldY_ = 0, spectrumModeFieldW_ = 0, spectrumModeFieldH_ = 0;
    float fftSizeFieldX_ = 0, fftSizeFieldY_ = 0, fftSizeFieldW_ = 0, fftSizeFieldH_ = 0;
    int spectrumFftSizeIndex_ = ssfilter::SpectrumAnalyzer::kDefaultSizeIndex;

    // Crossover Slope: same inline click-to-cycle field convention as the
    // two above -- cycles mbModel_.lrOrder through the same four choices
    // (12/24/36/48 dB/oct) as the sibling EQ plugin's Low/High Pass
    // "Slope" control. No separate UI-side cache needed (unlike
    // spectrumMode_/spectrumFftSizeIndex_ above): lrOrder already lives
    // directly in mbModel_ and round-trips through the Parameter mirror
    // like the rest of it, so an edit here is just another mbModel_
    // mutation followed by applyModelToEngine(), exactly like
    // setBandThreshold() etc.
    float crossoverSlopeFieldX_ = 0, crossoverSlopeFieldY_ = 0, crossoverSlopeFieldW_ = 0, crossoverSlopeFieldH_ = 0;

    // false until a genuine edit or a host-driven restore marks it -- see
    // UIEq.hpp's identical member's own comment for the real bug this
    // default avoids (a freshly-(re)created UI silently
    // overwriting the DSP's already-correct model with its own blank
    // default a few idle ticks after construction, before the host has a
    // chance to tell it what the real model is).
    bool modelDirty_ = false;
    int idleTicksSinceEdit_ = 0;

    // See UIEq.hpp's identical member's own comment: kParamUiHeartbeat's
    // value must actually toggle every tick, not just get resent, since
    // DPF's own LV2 wrapper drops a same-value resend before it ever
    // reaches Plugin::setParameterValue().
    float uiHeartbeatToggle_ = 0.0f;

    // Same toggle idiom, same reason -- see kParamMirrorCommit's own
    // comment (PluginMbCompressor.hpp) for the real bug this fixes.
    // Flipped in pushModelNow() right before the push.
    float mirrorCommitToggle_ = 0.0f;

    enum class PendingFileOp { None, Load, Save, Export } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    bool bypassed_ = false;
    int faultBlinkTicks_ = 0;

    // Background spectrum -- identical mechanism/reasoning to UIEq's own
    // pair of arrays, see that file's declaration comment.
    float spectrumOutRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    double spectrumSmoothMs_ = 150.0;
    // This plugin's own default is Both, not the shared kDefaultSpectrumMode
    // (Output) -- see PluginMbCompressor.hpp's identical member's own
    // comment. Only matters before the host's own "spectrumMode" state
    // value (if any) arrives via setState(); kept in sync with the DSP
    // side's own default so a freshly-opened editor never briefly shows
    // the wrong source before that round trip completes.
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::SpectrumMode::Both;

    DISTRHO_DECLARE_NON_COPYABLE(UIMbCompressor)
};

END_NAMESPACE_DISTRHO
