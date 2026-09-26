// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "../../plugin/dsp/EqModel.hpp"
#include "../../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../../plugin/dsp/StabilityGuard.hpp"
#include "../../plugin_eq/PluginEq.hpp" // for kNumMirroredBands
#include "../SimpleButton.hpp"
#include "../TooltipOverlay.hpp"
#include "BandListWidget.hpp"
#include "EqEditorHost.hpp"
#include "EqGraphWidget.hpp"

#include <chrono>
#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

class UIEq : public UI, public ssgui::IEqEditorHost
{
public:
    UIEq();
    // Flushes a still-debounced edit (modelDirty_) straight to the DSP
    // before this UI is torn down -- see the destructor's own comment in
    // UIEq.cpp for why this exists at all.
    ~UIEq() override;

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
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
    void setBandChannel(int index, ssfilter::EqChannel channel) override;
    bool supportsCrossFeed() const override { return true; }
    void setBandCrossFeed(int index, ssfilter::EqCrossFeed crossFeed) override;
    void setCrossFeedGainLtoRDb(double gainDb) override;
    void setCrossFeedGainRtoLDb(double gainDb) override;
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
    void beginBandAudition(int index) override;
    void endBandAudition() override;

private:
    void layout();
    void applyModelToEngine();       // marks the model dirty; the actual assemble+enforce+push is debounced, see uiIdle()
    void pushModelNow();
    // Pushes abSlotA_/abSlotB_/abHasA_/abHasB_/abActive_ to the host as
    // plain States (see PluginEq.hpp's StateIndex comment for why these
    // are plain States, not the import-safe Parameter mirror) -- called
    // both right after onAbA()/onAbB()/onAbCopy() (an explicit A/B
    // action) and from pushModelNow() itself (so the *currently active*
    // slot's own persisted copy stays live-synced with ongoing edits, not
    // just whatever it held the moment it was last switched into).
    void pushAbState();             // the debounced step itself
    // Pushes the current eqModel_ into PluginEq's Parameter mirror (see
    // PluginEq.hpp's ParameterIndex comment) via this UI's own
    // setParameterValue() -- the same "UI asks the host to change this
    // Parameter" call every other UI-driven Parameter here already uses
    // (kParamAudition, kParamBypass, ...) -- called from pushModelNow(),
    // which follows it with kParamMirrorCommit once every field has been
    // sent (see that Parameter's own comment in PluginEq.hpp). Deliberately
    // NOT done from the DSP side alone: an LV2 host only ever persists
    // whatever value *it* last explicitly set on an input Parameter
    // (confirmed directly in Ardour's own LV2Plugin::add_state(), which
    // writes its own cached shadow copy, never re-reading the plugin's
    // current value) -- a DSP-internal update with no corresponding
    // UI-initiated "change request" is invisible to the host and would
    // never actually reach the saved session file, silently defeating the
    // entire point of this mirror.
    void pushBandParamMirror();
    // The read side of the mirror: accumulates each mirror Parameter as it
    // arrives via parameterChanged() into mirrorStaging_/
    // mirrorStagingCount_/mirrorStagingTrim_/mirrorStagingXfeedLtoR_/
    // mirrorStagingXfeedRtoL_, and once the *last* one in the bank lands
    // (kParamCrossFeedGainRtoLDb, always the last to arrive within one
    // delivery burst -- see kParamMirrorCommit's own comment for why that
    // ordering holds), reconstructs eqModel_ from the staged copy. Fires both for the
    // host's initial post-open sync (an LV2 host sends every port's
    // current value once a UI is instantiated -- the only way this UI ever
    // learns the real, already-running band list) and, harmlessly, as an
    // echo of this UI's own just-pushed edits (guarded by modelDirty_ the
    // same way stateChanged() used to guard the old "eqmodel" restore, so
    // a slow echo can never clobber a newer pending local edit).
    void applyMirrorStagingIfComplete(uint32_t index, float value);
    // The one chokepoint every selection change funnels through (a direct
    // click/right-click/middle-click on either graph_ or list_, and the
    // index fixups removeBand()/duplicateBand() already need to do) --
    // keeps selectedBand_ and both widgets' own copies of it, plus the
    // DSP-side kParamAuditionBand, all in agreement regardless of which one
    // changed first. Same "select one band -> drive a shared control"
    // idiom ../mb/UIMbCompressor.cpp's/../dyneq/UIDynEq.cpp's own
    // selectBand() already establishes, ported here specifically so
    // Audition has a single band index to act on.
    void selectBand(int index);
    void refreshAuditionColors(); // keeps btnAudition_'s colors/enabled-state in sync with auditioning_/selectedBand_
    void beginTrimTextEdit(bool clearValue);
    void beginSmoothTextEdit(bool clearValue);
    // Shared double-click-to-clear detection for the toolbar's inline
    // single-value fields (Output Trim, Spectrum Smoothing) -- `fieldId`
    // just needs to be a stable, distinct value per field (their
    // EditTarget enumerator works). A previous version of this class
    // always passed clearValue=false for these fields, silently never
    // supporting double-click-to-clear on any of them (unlike every
    // other numeric field in this project) -- this is the fix, applied
    // once so both (and any future one) get it uniformly instead of
    // separate near-identical hand-rolled timers.
    bool checkInlineDoubleClick(int fieldId);
    void commitEdit();
    void cancelEdit();
    void loadEqPresetFile(const char* path);
    void saveEqPresetFile(const char* path);
    void exportStateSpaceFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- reuses the same JSON serialization
    // Save/Load/State already use (ssfilter::eq::toJson()/fromJson()),
    // factored out of what used to be loadEqPresetFile()'s own body. See
    // ../../../compressor's UICompressorMono.cpp (this project's own
    // sibling repo) for the full design writeup this is ported from.
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

    // Staging area for applyMirrorStagingIfComplete() -- same shape as
    // PluginEq's own mirrorBandSlots_/mirrorBandCount_/mirrorOutputTrimDb_,
    // filled field-by-field as parameterChanged() delivers the mirror bank,
    // then folded into eqModel_ all at once once the bank is complete.
    ssfilter::EqBand mirrorStaging_[kNumMirroredBands];
    int mirrorStagingCount_ = 0;
    double mirrorStagingTrim_ = 0.0;
    double mirrorStagingXfeedLtoR_ = -60.0, mirrorStagingXfeedRtoL_ = -60.0;

    DGL_NAMESPACE::EqGraphWidget* graph_ = nullptr;
    DGL_NAMESPACE::BandListWidget* list_ = nullptr;

    DGL_NAMESPACE::SimpleButton* btnLoad_        = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_        = nullptr;
    DGL_NAMESPACE::SimpleButton* btnExport_      = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAddBand_     = nullptr;
    DGL_NAMESPACE::SimpleButton* btnReset_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_      = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAudition_    = nullptr; // previews selectedBand_'s own isolated frequency region, see PluginEq.hpp's kParamAudition comment

    // Which band is currently selected -- previously tracked only inside
    // graph_/list_ themselves (purely to sync each other's highlight, see
    // their own setOnSelectionChanged() wiring below); mirrored here too
    // now, specifically so Audition has one shared, always-current band
    // index to act on. -1 = nothing selected.
    int selectedBand_ = -1;
    bool auditioning_ = false; // mirrors kParamAudition -- see btnAudition_'s own comment

    // The *persistent* toolbar Audition state, captured at the instant a
    // momentary right-click-hold preview starts -- see beginBandAudition()'s
    // own comment for why endBandAudition() must restore from this instead
    // of re-reading auditioning_ live.
    bool auditionHoldPrevState_ = false;

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
    bool applyingSnapshot_ = false; // suppresses uiIdle()'s undo-checkpoint block while an A/B switch or undo/redo restore is itself applying a snapshot
    static constexpr int kUndoDebounceTicks = 15;
    static constexpr size_t kMaxUndoDepth = 100;

    // Output trim: one always-present numeric field, edited inline (drawn/
    // hit-tested directly here rather than via a dedicated widget class
    // for a single field) using the same edit-target machinery as a band
    // field -- see EditTarget below.
    float trimFieldX_ = 0, trimFieldY_ = 0, trimFieldW_ = 0, trimFieldH_ = 0;

    // Spectrum Source: a click-to-cycle field (Off/In/Out/Both), same
    // pattern as the FFT-size cycle field below -- not a free-typed
    // value, so it has no EditTarget of its own, just a hit box and a
    // click handler in onMouse(). See ssfilter::SpectrumMode.
    float spectrumModeFieldX_ = 0, spectrumModeFieldY_ = 0, spectrumModeFieldW_ = 0, spectrumModeFieldH_ = 0;

    enum class EditTarget { None, BandField, OutputTrim, SpectrumSmoothMs, HarmonicFactor };
    EditTarget editTarget_ = EditTarget::None;
    int editBandIndex_ = -1;
    ssgui::EqField editBandField_ = ssgui::EqField::Freq;
    std::string editBuffer_;

    // Which band "Create Harmonic Twin -> Custom Factor..." was invoked
    // on -- meaningful only while editTarget_ == HarmonicFactor. See
    // beginHarmonicFactorEdit()/EqGraphWidget's own "Custom Factor..."
    // menu handling.
    int pendingHarmonicSourceBand_ = -1;

    int lastClickInlineField_ = -1; // an EditTarget value (as int); see checkInlineDoubleClick()
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

    // false until a *genuine* edit (applyModelToEngine()) or a host-driven
    // restore marks it -- see that function's own comment and
    // applyMirrorStagingIfComplete()'s own guard for why this must NOT
    // default to true. A freshly-constructed UI's eqModel_ is just
    // EqModel()'s empty default; if this started true, the debounced
    // pushModelNow() below would fire on its own a few idle ticks after
    // construction regardless of whether the host has told this new UI
    // instance what the *real*, already-running model actually is yet --
    // and pushModelNow() unconditionally overwrites the DSP's own mirror
    // Parameters (and, through them, the session-persisted copy) with
    // whatever eqModel_ currently holds. This was a real bug (from when
    // the band list still round-tripped through a State):
    // closing and reopening the plugin's GUI (or the host deactivating/
    // reactivating it, which can tear down and recreate the UI the same
    // way) silently reset every band to nothing, while leaving the GUI
    // open the whole time never did -- because an already-open UI's
    // eqModel_ was already populated correctly (from its own original
    // restore, or the user's own edits) and never needed to guess.
    // Starting false makes a fresh UI a pure observer -- it pushes nothing
    // until it's told the real model (applyMirrorStagingIfComplete()) or
    // the user actually changes something -- so the race can't happen
    // regardless of how quickly (or whether) the host replays the mirror
    // to a newly-created UI.
    bool modelDirty_ = false;
    int idleTicksSinceEdit_ = 0;

    // kParamUiHeartbeat's own value must actually *change* every tick, not
    // just get resent -- see this member's use in uiIdle() and
    // PluginEq.hpp's kParamUiHeartbeat comment for why: DPF's own LV2
    // wrapper (DistrhoPluginLV2.cpp's run()) only calls
    // Plugin::setParameterValue() when a control port's value differs
    // from what it saw last block (`d_isNotEqual(fLastControlValues[i],
    // curValue)`), a completely reasonable de-duplication for a normal
    // knob -- but it silently meant a constant "always send 1.0f"
    // heartbeat only ever actually reached the DSP side *once*, right
    // after the UI opened, in any host using DPF's LV2 backend. After
    // that the DSP side's own "samples since last heartbeat" counter just
    // kept climbing, the ~1s timeout tripped, and the spectrum analyzer
    // stopped being fed for the rest of the session (the spectrum plot
    // disappeared) -- confirmed by reading DPF's own dedup logic, not
    // guessed. Toggling between two values each
    // tick guarantees a real, host-visible change every time regardless of
    // format/host.
    float uiHeartbeatToggle_ = 0.0f;

    // Same toggle idiom as uiHeartbeatToggle_ just above, same reason --
    // see kParamMirrorCommit's own comment (PluginEq.hpp) for the real,
    // reported bug this fixes: a constant "always push 1.0f" commit value
    // only ever actually reached the DSP once, ever, for the lifetime of
    // the port (worse than the heartbeat case -- Ardour doesn't implement
    // the LV2 trigger-port convention at all, so the stale 1.0 even
    // survives a session save/reload). Flipped in pushModelNow() right
    // before the push.
    float mirrorCommitToggle_ = 0.0f;

    enum class PendingFileOp { None, Load, Save, Export } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    bool bypassed_ = false;
    int faultBlinkTicks_ = 0;

    // Background spectrum: raw values arrive via parameterChanged() (one
    // DPF output Parameter per band per analyzer, see PluginEq.hpp's
    // kParamSpectrumOutputBase/kParamSpectrumInputBase), smoothed here
    // every uiIdle() tick before being handed to graph_ -- "how
    // fast/smooth" is a pure UI-side preference with no DSP-side
    // round-trip needed. Both arrays are kept
    // current regardless of spectrumMode_ -- the smoothing cost is trivial
    // (kNumBands floats), so there's no reason to special-case "don't bother
    // smoothing the hidden one"; only the *DSP* side (PluginEq::run(),
    // via spectrumMode_ forwarded through setState()) actually skips real
    // work for an unselected analyzer.
    float spectrumOutRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    // Same pipeline, for the two per-channel Output analyzers
    // (kParamSpectrumOutputLBase/RBase) -- kept current unconditionally
    // just like the pair above, regardless of whether any band is
    // currently Left/Right-only; graph_ is the one place that decides
    // whether to actually draw them (see EqGraphWidget's own comment).
    float spectrumOutLRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutLSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutRRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutRSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    double spectrumSmoothMs_ = 150.0;

    // Which spectrum trace(s) are shown -- see ssfilter::SpectrumMode.
    // Cycled by clicking spectrumModeFieldX_/Y_/W_/H_ (an inline field,
    // same convention as the FFT-size cycle field just below it, not a
    // SimpleButton -- four short mode names ("Off"/"In"/"Out"/"Both")
    // read better as a labeled value box than crammed into a button
    // that also has to say "Spectrum"). This plugin's own default is
    // Both, not the shared kDefaultSpectrumMode (Output) -- see
    // PluginEq.hpp's own comment; this placeholder (shown only until the
    // host's real stateChanged() call arrives) matches it so a freshly
    // (re)created editor doesn't flash the wrong value first.
    ssfilter::SpectrumMode spectrumMode_ = ssfilter::SpectrumMode::Both;

    // Spectrum-smoothing field: another single always-present numeric
    // field, same inline-editing approach as Output Trim above. (There
    // used to be a "Spectrum Gain (dB)" field here too, for making the
    // spectrum visible against a fixed display range -- removed once the
    // spectrum switched to autoscaling its own range every frame, which
    // makes a manual gain control redundant: a constant additive boost
    // shifts the autoscaled range by the same amount and produces an
    // identical picture. See EqGraphWidget.hpp's spectrumToY() comment.)
    float smoothFieldX_ = 0, smoothFieldY_ = 0, smoothFieldW_ = 0, smoothFieldH_ = 0;

    // Spectrum FFT size: a *cycle* control (click steps through
    // SpectrumAnalyzer::kFftSizePresets), not a free-typed field like the
    // three above -- this one is DSP-relevant (reconfigures the real
    // analyzer via setState("spectrumFftSize", ...), see PluginEq), so it
    // stays restricted to the same small set of presets the DSP side
    // actually supports rather than accepting an arbitrary number.
    float fftSizeFieldX_ = 0, fftSizeFieldY_ = 0, fftSizeFieldW_ = 0, fftSizeFieldH_ = 0;
    int spectrumFftSizeIndex_ = ssfilter::SpectrumAnalyzer::kDefaultSizeIndex;

    DISTRHO_DECLARE_NON_COPYABLE(UIEq)
};

END_NAMESPACE_DISTRHO
