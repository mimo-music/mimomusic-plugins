// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// UIDynEq2.hpp
//
// Top-level UI for Dynamic EQ 2 -- owns the actual ssfilter::DynamicEqModel2
// and implements ssgui::IDynEqEditorHost2 for DynEqGraphWidget2 and
// DynEqBandListWidget2. Laid out like the Parametric EQ (UIEq): row 1 file
// ops, Reset/Bypass, A/B and Undo/Redo; row 2 +Band, Audition and the
// one-line Spectrum Source / Smoothing / Resolution / Output Trim fields;
// then the graph, and below it the band list with every band's settings.
// Text editing (list fields, the two inline fields, the harmonic
// custom-factor input) is one state machine here (editTarget_), since
// keyboard events reach the top-level UI.
//
// Load/Save (".ssdynjson") and "Export to State-Space..." follow the same
// file-dialog pattern as every sibling plugin's UI.

#pragma once

#include "DistrhoUI.hpp"

#include "../EditorSubWidget.hpp"

#include "../../plugin/dsp/DynamicEqModel2.hpp"
#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../../plugin_dyn_eq2/PluginDynEq2.hpp" // for kNumMirroredBands
#include "../ParamStripWidget.hpp"
#include "../SimpleButton.hpp"
#include "../TooltipOverlay.hpp"
#include "DynEqBandListWidget2.hpp"
#include "DynEqEditorHost2.hpp"
#include "DynEqGraphWidget2.hpp"

#include <chrono>
#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

class UIDynEq2 : public UI, public ssgui::IDynEqEditorHost2
{
public:
    UIDynEq2();
    // Flushes a still-debounced edit (modelDirty_) straight to the DSP
    // before this UI is torn down -- see ../eq/UIEq.cpp's identical
    // destructor for the full reasoning.
    ~UIDynEq2() override;

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onDisplay() override; // DPF's frame, with the real pixel ratio -- see EditorSubWidget.hpp
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

    void uiFileBrowserSelected(const char* filename) override;

    // -- ssgui::IDynEqEditorHost2 --------------------------------------------
    int getBandCount() const override { return int(dynEqModel_.bands.size()); }
    ssfilter::DynamicEqBand2 getBand(int index) const override;
    ssfilter::DynamicEqModel2 getModel() const override { return dynEqModel_; }
    double getCurrentGainDb(int index) const override;
    void addBand(double freqHz, double gainDb) override;
    void removeBand(int index) override;
    void duplicateBand(int index) override;
    void setBandFreq(int index, double freqHz) override;
    void setBandGain(int index, double gainDb) override;
    void setBandQ(int index, double q) override;
    void setBandBandwidth(int index, double octaves) override;
    void setBandThreshold(int index, double thresholdDb) override;
    void setBandRatio(int index, double ratio) override;
    void setBandRatioUp(int index, double ratioUp) override;
    void setBandRange(int index, double rangeDb) override;
    void setBandAttack(int index, double attackMs) override;
    void setBandRelease(int index, double releaseMs) override;
    void setBandDynamicsEnabled(int index, bool enabled) override;
    void setBandEnabled(int index, bool enabled) override;
    void setBandSolo(int index, bool solo) override;
    void setBandChannel(int index, ssfilter::EqChannel channel) override;
    void setBandType(int index, ssfilter::EqBandType type) override;
    void setBandCrossFeed(int index, ssfilter::EqCrossFeed crossFeed) override;
    void setCrossFeedGainLtoRDb(double gainDb) override;
    void setCrossFeedGainRtoLDb(double gainDb) override;
    void createHarmonicTwin(int index, double factor) override;
    void unlinkHarmonic(int index) override;
    void beginHarmonicFactorEdit(int sourceIndex) override;
    bool isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const override;
    void commitHarmonicFactorEdit() override;
    void beginBandAudition(int index) override;
    void endBandAudition() override;
    void beginBandTextEdit(int index, ssgui::DynEqField field, bool clearValue) override;
    bool isEditingBandField(int index, ssgui::DynEqField field, std::string& liveText) const override;

private:
    void layout();
    void applyModelToEngine(); // marks the model dirty; the actual push is debounced, see uiIdle()
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
    // Keeps btnAudition_ in sync with auditioning_/selectedBand_, and turns
    // Audition off when there is no band left to preview.
    void refreshAuditionColors();

    // -- Text editing (see this file's top comment) --
    enum class EditTarget { None, BandField, OutputTrim, SpectrumSmoothMs, HarmonicFactor };
    void commitEdit();
    void cancelEdit();
    void beginTrimTextEdit(bool clearValue);
    void beginSmoothTextEdit(bool clearValue);
    void setInlineFieldValue(EditTarget which, double v);
    // Double-click detection for the two inline fields (a double-click
    // starts typing from an empty buffer, as everywhere else).
    bool checkInlineDoubleClick(int fieldId);

    void loadPresetFile(const char* path);
    void savePresetFile(const char* path);
    void exportStateSpaceFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- reuses the same JSON serialization
    // Save/Load/State already use (ssfilter::dyneq2::toJson()/fromJson()),
    // factored out of what used to be loadPresetFile()'s own body. See
    // ../../compressor's UICompressorMono.cpp (this project's own
    // sibling repo) for the full design writeup this is ported from.
    void applyModelSnapshot(const std::string& json);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    ssfilter::DynamicEqModel2 dynEqModel_;
    int selectedBand_ = -1;

    // Staging area for applyMirrorStagingIfComplete() -- see UIEq.hpp's
    // identical members for the full reasoning.
    ssfilter::DynamicEqBand2 mirrorStaging_[kNumMirroredBands];
    int mirrorStagingCount_ = 0;
    double mirrorStagingTrim_ = 0.0;
    int mirrorStagingSoloedBand_ = -1;
    double mirrorStagingXfeedLtoR_ = -60.0, mirrorStagingXfeedRtoL_ = -60.0;

    EditTarget editTarget_ = EditTarget::None;
    int editBandIndex_ = -1;
    ssgui::DynEqField editBandField_ = ssgui::DynEqField::Freq;
    std::string editBuffer_;
    int pendingHarmonicSourceBand_ = -1; // the band a custom-factor twin is created from

    // Output Trim / Spectrum Smoothing: press-then-release is a click (text
    // edit), press-then-drag a vertical fader, as in UIEq.
    EditTarget inlinePress_ = EditTarget::None;
    bool inlineDragging_ = false;
    double inlineDragStartY_ = 0.0, inlineDragStartValue_ = 0.0;
    int lastClickInlineField_ = -1;
    std::chrono::steady_clock::time_point lastClickInlineTime_{};

    // The persistent Audition state when a right-button audition-hold
    // started -- restored by endBandAudition() (see UIEq's identical member).
    bool auditionHoldPrevState_ = false;

    DGL_NAMESPACE::DynEqGraphWidget2* graph_ = nullptr;
    DGL_NAMESPACE::DynEqBandListWidget2* list_ = nullptr;
    // The selected band's own large fields between graph and list (same
    // design as the Parametric EQ's, see UIEq::refreshBandStrip()), here
    // with the dynamics too; "(unused)" marks what the band's type or its
    // switched-off dynamics ignore. Rebuilt only on selection/type/dynamics
    // changes, never while a field is being edited.
    DGL_NAMESPACE::ParamStripWidget* bandStrip_ = nullptr;
    int bandStripKey_ = -1;
    float bandStripHeaderY_ = 0.0f;
    void refreshBandStrip();

    DGL_NAMESPACE::SimpleButton* btnLoad_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnExport_     = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAddBand_    = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAudition_   = nullptr; // previews selectedBand_'s isolated detector output, see PluginDynEq2.hpp's kParamAudition comment
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
    bool applyingSnapshot_ = false; // true while an A/B switch or undo/redo restore is itself calling applyModelSnapshot() -- suppresses uiIdle()'s own undo-checkpoint block from recording that restore as a fresh edit
    static constexpr int kUndoDebounceTicks = 15;
    static constexpr size_t kMaxUndoDepth = 100;

    float trimFieldX_ = 0, trimFieldY_ = 0, trimFieldW_ = 0, trimFieldH_ = 0;
    float smoothFieldX_ = 0, smoothFieldY_ = 0, smoothFieldW_ = 0, smoothFieldH_ = 0;
    float spectrumModeFieldX_ = 0, spectrumModeFieldY_ = 0, spectrumModeFieldW_ = 0, spectrumModeFieldH_ = 0;
    float fftSizeFieldX_ = 0, fftSizeFieldY_ = 0, fftSizeFieldW_ = 0, fftSizeFieldH_ = 0;
    // This plugin's own default is the largest preset, not the shared
    // kDefaultSizeIndex every sibling EQ plugin's UI starts on -- matches
    // PluginDynEq2.hpp's own spectrumMode_/constructor comment, so this
    // placeholder (shown only until the host's real stateChanged() call
    // arrives) doesn't flash the wrong value first.
    int spectrumFftSizeIndex_ = ssfilter::SpectrumAnalyzer::kNumSizePresets - 1;

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
    // comment (PluginDynEq2.hpp) for the real bug this fixes.
    // Flipped in pushModelNow() right before the push.
    float mirrorCommitToggle_ = 0.0f;

    enum class PendingFileOp { None, Load, Save, Export } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    bool bypassed_ = false;
    int faultBlinkTicks_ = 0;
    bool auditioning_ = false; // mirrors kParamAudition -- see btnAudition_'s own comment

    // Live per-band gain readback (PluginDynEq2's kParamCurrentGainBase
    // outputs), arriving via parameterChanged() -- see PluginDynEq2.hpp's
    // comment for why this has to cross a Parameter rather than a direct
    // getter call. currentGainDb_ holds the value actually drawn (the
    // "Live +N.NdB" text next to a band with dynamics on) -- smoothed
    // against wall-clock time before being stored, same technique/time
    // constant as gui/mb/MultibandGraphWidget::setBandGainDb() (see that
    // function's own comment): a real report against that meter ("the
    // numbers are changing so fast, that its difficult to read") applies
    // identically here, since this value arrives via the same up-to-once-
    // per-audio-block DPF output-Parameter path.
    float currentGainDb_[ssfilter::kMaxDynEqBands2];
    std::chrono::steady_clock::time_point currentGainSmoothUpdated_[ssfilter::kMaxDynEqBands2]{};
    bool currentGainSmoothInit_[ssfilter::kMaxDynEqBands2] = {};

    float spectrumOutRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    // Same pipeline, for the two per-channel Output analyzers -- see the
    // sibling Parametric EQ's UIEq.hpp identical pair.
    float spectrumOutLRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutLSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutRRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutRSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    double spectrumSmoothMs_ = 150.0;
    // Both, not the sibling EQ plugins' Output-only kDefaultSpectrumMode
    // -- see PluginDynEq2.hpp's identical comment.
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::SpectrumMode::Both;

    DISTRHO_DECLARE_NON_COPYABLE(UIDynEq2)
};

END_NAMESPACE_DISTRHO
