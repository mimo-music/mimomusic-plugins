// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "../EditorSubWidget.hpp"

#include "../../plugin/dsp/EqModel.hpp"
#include "../../plugin/dsp/MonoStabilityGuard.hpp"
#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../../plugin_eq_mono/PluginEqMono.hpp" // for kNumMirroredBands
#include "../ParamStripWidget.hpp"
#include "../SimpleButton.hpp"
#include "../TooltipOverlay.hpp"
#include "BandListWidget.hpp"
#include "EqEditorHost.hpp"
#include "EqGraphWidget.hpp"

#include <chrono>
#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

// The mono sibling of UIEq -- deliberately the *same* GUI (EqGraphWidget,
// BandListWidget, and every layout/interaction detail below are a direct
// copy of UIEq's own code, unchanged) driving a different DSP assembler
// underneath: everywhere UIEq calls ssfilter::eq::assembleStateSpace() +
// ssfilter::StabilityGuard (to decide what the *audio engine* actually
// runs), this class calls ssfilter::eq::assembleMonoStateSpace() +
// ssfilter::mono::MonoStabilityGuard instead -- see EqModel.hpp's
// assembleMonoStateSpace() comment for why that's a smaller assembled
// system for the same band list, which is the actual mechanism behind the
// mono plugin's DSP saving. The one place this class *keeps* using the
// stereo assembler is exportStateSpaceFile() -- see that method's own
// comment for why.
//
// ssfilter::EqModel (the band list itself) has no channel count of its
// own, so it, EqGraphWidget, BandListWidget and EqEditorHost.hpp's
// IEqEditorHost interface are all reused completely as-is from the
// stereo EQ plugin -- nothing about "mono vs. stereo" is visible to any
// of them, only to this class and the DSP it talks to.
class UIEqMono : public UI, public ssgui::IEqEditorHost
{
public:
    UIEqMono();
    // Flushes a still-debounced edit (modelDirty_) straight to the DSP
    // before this UI is torn down -- see ../eq/UIEq.cpp's identical
    // destructor for the full reasoning.
    ~UIEqMono() override;

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

    // -- ssgui::IEqEditorHost ---------------------------------------------
    int getBandCount() const override { return int(eqModel_.bands.size()); }
    ssfilter::EqBand getBand(int index) const override;
    ssfilter::EqModel getModel() const override { return eqModel_; }
    void setBandFreq(int index, double freqHz) override;
    void setBandBandwidth(int index, double octaves) override;
    void setBandGain(int index, double gainDb) override;
    void setBandQ(int index, double q) override;
    void setBandType(int index, ssfilter::EqBandType type) override;
    void setBandSlopeStages(int index, int slopeStages) override;
    void setBandEnabled(int index, bool enabled) override;
    void setBandSolo(int index, bool solo) override;

    // No-op -- a single channel has nothing to assign Left/Right of. Never
    // actually reachable from this UI's own widgets: BandListWidget is
    // constructed with stereo=false here (see this file's own .cpp), which
    // hides the Channel column entirely rather than showing a control that
    // would silently do nothing when clicked.
    void setBandChannel(int, ssfilter::EqChannel) override {}
    void addBand(double freqHz, double gainDb) override;
    void removeBand(int index) override;
    void duplicateBand(int index) override;
    void beginBandTextEdit(int index, ssgui::EqField field, bool clearValue = false) override;
    bool isEditingBandField(int index, ssgui::EqField field, std::string& liveText) const override;
    void createHarmonicTwin(int index, double factor) override;
    void unlinkHarmonic(int index) override;
    void beginHarmonicFactorEdit(int sourceIndex) override;
    bool isEditingHarmonicFactor(int sourceIndex, std::string& liveText) const override;
    void commitHarmonicFactorEdit() override;
    // No-op pair -- this plugin has no Audition DSP of its own (unlike
    // the stereo sibling), see EqEditorHost.hpp's own comment. Still
    // required to keep this class non-abstract: EqGraphWidget's shared
    // right-click-hold gesture calls these on *either* plugin.
    void beginBandAudition(int) override {}
    void endBandAudition() override {}

private:
    void layout();
    void applyModelToEngine();
    void pushModelNow();
    // See UIEq::pushAbState()'s identical declaration comment (gui/eq/UIEq.hpp).
    void pushAbState();
    // The Parameter mirror push -- see UIEq::pushBandParamMirror()'s
    // identical declaration comment (gui/eq/UIEq.hpp) for the full reasoning.
    void pushBandParamMirror();
    // The read side -- see UIEq::applyMirrorStagingIfComplete()'s identical
    // declaration comment (gui/eq/UIEq.hpp) for the full reasoning.
    void applyMirrorStagingIfComplete(uint32_t index, float value);
    void beginTrimTextEdit(bool clearValue);
    void beginSmoothTextEdit(bool clearValue);
    bool checkInlineDoubleClick(int fieldId);
    void commitEdit();
    void cancelEdit();
    void loadEqPresetFile(const char* path);
    void saveEqPresetFile(const char* path);
    void exportStateSpaceFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- see ../eq/UIEq.hpp's identical block
    // (this file's own stereo sibling) for the full design writeup.
    void applyModelSnapshot(const std::string& json);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    ssfilter::EqModel eqModel_;
    ssfilter::StabilityReport lastReport_;

    // Staging area for applyMirrorStagingIfComplete() -- see UIEq.hpp's
    // identical members for the full reasoning.
    ssfilter::EqBand mirrorStaging_[kNumMirroredBands];
    int mirrorStagingCount_ = 0;
    double mirrorStagingTrim_ = 0.0;

    DGL_NAMESPACE::EqGraphWidget* graph_ = nullptr;
    DGL_NAMESPACE::BandListWidget* list_ = nullptr;
    // The selected band's own large fields, between graph and list -- see
    // UIEq::refreshBandStrip() (same design; here the list holds the selection).
    DGL_NAMESPACE::ParamStripWidget* bandStrip_ = nullptr;
    int bandStripBand_ = -2;
    int bandStripType_ = -1;
    float bandStripHeaderY_ = 0.0f;
    void refreshBandStrip();

    DGL_NAMESPACE::SimpleButton* btnLoad_        = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_        = nullptr;
    DGL_NAMESPACE::SimpleButton* btnExport_      = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAddBand_     = nullptr;
    DGL_NAMESPACE::SimpleButton* btnReset_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_      = nullptr;

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

    float trimFieldX_ = 0, trimFieldY_ = 0, trimFieldW_ = 0, trimFieldH_ = 0;

    // Spectrum Source: a click-to-cycle field (Off/In/Out/Both), same
    // pattern as the FFT-size cycle field below -- see UIEq.hpp's
    // identical member for the full reasoning.
    float spectrumModeFieldX_ = 0, spectrumModeFieldY_ = 0, spectrumModeFieldW_ = 0, spectrumModeFieldH_ = 0;

    enum class EditTarget { None, BandField, OutputTrim, SpectrumSmoothMs, HarmonicFactor };
    EditTarget editTarget_ = EditTarget::None;
    int editBandIndex_ = -1;
    ssgui::EqField editBandField_ = ssgui::EqField::Freq;
    std::string editBuffer_;

    // See ../eq/UIEq.hpp's identical member's own comment.
    int pendingHarmonicSourceBand_ = -1;

    int lastClickInlineField_ = -1;
    std::chrono::steady_clock::time_point lastClickInlineTime_{};

    // Output Trim / Spectrum Smoothing fields: press, then either drag
    // vertically (a fader, same feel as ParamStripWidget: 3px dead zone,
    // 200px for the whole range) or release without moving (click-to-type,
    // as before). inlinePress_ is the field pressed, None when idle.
    EditTarget inlinePress_ = EditTarget::None;
    bool inlineDragging_ = false;
    double inlineDragStartY_ = 0.0;
    double inlineDragStartValue_ = 0.0;
    // Clamps and applies a new Output Trim or Spectrum Smoothing value --
    // shared by a committed text edit, a drag, and a scroll.
    void setInlineFieldValue(EditTarget which, double v);

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
    // comment (PluginEqMono.hpp) for the real bug this fixes.
    // Flipped in pushModelNow() right before the push.
    float mirrorCommitToggle_ = 0.0f;

    enum class PendingFileOp { None, Load, Save, Export } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    bool bypassed_ = false;
    int faultBlinkTicks_ = 0;

    // See UIEq.hpp's identical pair for why both arrays are kept current
    // regardless of spectrumMode_.
    float spectrumOutRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    double spectrumSmoothMs_ = 150.0;

    ssfilter::SpectrumMode spectrumMode_ = ssfilter::kDefaultSpectrumMode;

    float smoothFieldX_ = 0, smoothFieldY_ = 0, smoothFieldW_ = 0, smoothFieldH_ = 0;

    float fftSizeFieldX_ = 0, fftSizeFieldY_ = 0, fftSizeFieldW_ = 0, fftSizeFieldH_ = 0;
    int spectrumFftSizeIndex_ = ssfilter::SpectrumAnalyzer::kDefaultSizeIndex;

    DISTRHO_DECLARE_NON_COPYABLE(UIEqMono)
};

END_NAMESPACE_DISTRHO
