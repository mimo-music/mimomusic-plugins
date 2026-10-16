// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"
#include "../plugin_stereo/PluginCompressorStereo.hpp"
#include "CompressorGraphWidget.hpp"
#include "LevelMeterWidget.hpp"
#include "ParamStripWidget.hpp"
#include "PresetIO.hpp"
#include "SidechainScopeWidget.hpp"
#include "SimpleButton.hpp"
#include "TooltipOverlay.hpp"

#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

class UICompressorStereo : public UI
{
public:
    UICompressorStereo();

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onDisplay() override; // DPF's frame, with the real pixel ratio -- see EditorSubWidget.hpp
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;

    void uiFileBrowserSelected(const char* filename) override;

private:
    // Whichever of strip_/stripA_/stripB_/stripGlobal_/scStrip_ is
    // currently mid-edit, or nullptr -- only one can ever be at once
    // (only one is visible/clickable at a time between the Linked/dual
    // layout split and the sidechain panel's own overlay), so this is
    // unambiguous. Used by onCharacterInput()/onKeyboard() to route
    // keystrokes to the right field -- see their own comments for the
    // real bug this fixes.
    DGL_NAMESPACE::ParamStripWidget* editingStrip() const;

    void layout();
    void refreshBypassColors();
    void refreshSidechainColors();
    void refreshStripFields(); // rebuilds strip_'s (Linked mode) field list
    // Dual-Mono/Mid-Side shows both compressors side by side (doubled GUI
    // objects instead of an editing switch) -- rebuilds stripA_'s/stripB_'s own
    // field lists. stripA_ always targets A's own Parameters
    // unconditionally; stripB_'s fields individually fall back to A's own
    // matching Parameter (via bParam()) whenever B isn't currently
    // independent (isChannelBIndependent()) -- so while linked, both
    // strips literally edit the *same* underlying Parameter and always
    // show identical live values, a genuine two-way link rather than a
    // one-time copy. See refreshDualLayoutVisibility()'s own comment for
    // when stripA_/stripB_ (vs. the single strip_/graph_) are actually
    // shown.
    void refreshDualStripFields();
    // Returns paramB when isB is true and B is currently independent,
    // paramA otherwise -- the one place every dual-layout widget's field-
    // target-selection logic lives (strip fields, graph curves, GR
    // markers, Ratio Direction). isB is false for anything that's always
    // A's own (stripA_/graphA_ itself, or a field that has no B
    // counterpart at all).
    uint32_t bParam(bool isB, uint32_t paramA, uint32_t paramB) const;
    // See UICompressorMono's identical method for why this exists: keeps
    // graph_'s drag/scroll edits (Threshold/Ratio/Knee) and the strip's
    // own fields in sync regardless of which widget the user touched.
    // isB selects which of graphA_/graphB_ (or, in Linked mode, the
    // single graph_) fired the callback -- see onGraphParamChanged()'s
    // own definition.
    void onGraphParamChanged(uint32_t index, float value);
    void refreshStereoModeLabel();
    // btn's own label reflects `up`; used for all three Ratio Direction
    // buttons (btnRatioDirection_/btnRatioDirectionA_/btnRatioDirectionB_)
    // -- see their own declaration comments for which state each drives.
    void refreshRatioDirectionLabel(DGL_NAMESPACE::SimpleButton* btn, bool up);
    void updateMeterLabelsForMode();
    void refreshListenColors();
    // In Mid/Side the two compressors can be set completely differently --
    // see kParamThresholdB's own comment
    // (PluginCompressorStereo.hpp) for the full design.
    bool isChannelBIndependent() const; // mirrors the DSP-side method of the same name
    void refreshLinkLabel();
    // Shows/hides the single Linked-mode widget set (graph_/strip_/
    // btnRatioDirection_) vs. the dual Dual-Mono/Mid-Side widget set
    // (graphA_/graphB_/stripA_/stripB_/btnRatioDirectionA_/
    // btnRatioDirectionB_) based on stereoMode_: Linked keeps the
    // exact single-graph/single-strip look this plugin always had;
    // Dual-Mono and Mid/Side both get the new side-by-side pair
    // regardless of Link state (Link only governs whether the two still
    // edit the same underlying Parameters, see bParam()'s own comment --
    // it no longer hides either side). meterIn_/meterOut_ themselves stay
    // in the same position/visible either way, only their *data* depends
    // on stereo mode (see PluginCompressorStereo.cpp's own run() comment
    // for the Mid/Side metering fix this session also made).
    void refreshDualLayoutVisibility();
    // Copies A's current dynamics values (now including Input Gain and
    // Wet/Dry) into B's own Parameters -- "start linked, split on edit":
    // called only from a live user action (turning Mid/Side on, or Link
    // off in Dual-Mono), never from a session/preset/A-B/undo-redo
    // restore, where B's own already-restored values must win untouched.
    void syncBFromA();
    // Re-applies the current Threshold/Ratio/Knee/Ratio-Direction values
    // to every graph that shows them right now -- graph_ and graphA_
    // always get A's own; graphB_ gets B's own (or A's, via bParam()'s
    // same fallback) -- the single chokepoint every such change (on
    // either bank) funnels through, so no call site has to know which
    // graph(s) are currently visible.
    void refreshGraphCurves();
    // Pushes each visible graph's own configured Makeup Gain into its GR-
    // column marker.
    void refreshMakeupMarkers();

    // The sidechain options panel -- ORIGINALLY built as a genuine second native OS window
    // (DGL_NAMESPACE::NanoStandaloneWindow, transient-parented to this
    // one). That crashed the host outright: DPF's own LV2-UI bridge
    // hands off each new UI's construction parameters through a single
    // *static* global (UI::PrivateData::s_nextPrivateData, see
    // distrho/src/DistrhoUI.cpp's own createNextWindow()) that assumes
    // exactly one plugin-editor window is ever being stood up at a time
    // -- a second real Window sharing this UI's own Application collided
    // with that assumption. Rebuilt as an in-window overlay panel
    // instead: toggles the visibility of a handful of ordinary
    // NanoSubWidget children of *this* (btnScFilterEnable_/scStrip_/
    // scScope_/btnScClose_) that sit on top of (and, via
    // SidechainScopeWidget::onMouse()'s own override, block clicks
    // through to) the normal strip_/graph_/meterIn_/meterOut_ underneath
    // -- no second Window, no second Application, nothing DPF's own
    // single-editor assumption can trip over.
    void refreshScPanelVisibility();
    void refreshScListenColors();
    // A mode where left and right are filtered differently -- toggles kParamScFilterLinkLR;
    // scStrip_'s own Highpass/Lowpass fields point at L's or R's own
    // Parameters depending on both this link state and editingScBankR_
    // (which side is currently selected for editing while unlinked), same
    // "start linked, split on edit" pattern as syncBFromA() above -- see
    // refreshScStripFields()'s own comment.
    void refreshScLinkLabel();
    void refreshScStripFields();

    void savePresetFile(const char* path);
    void loadPresetFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- see this project's own
    // UICompressorMono.cpp for the full design writeup this is ported
    // from (against this plugin's own parameter list/stereoMode state).
    presetio::KeyValues captureSnapshot() const;
    void applySnapshot(const presetio::KeyValues& values);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    // Linked-mode-only widget set -- Linked mode keeps its original look.
    // See refreshDualLayoutVisibility()'s own comment.
    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::CompressorGraphWidget* graph_ = nullptr;
    // The Ratio direction swap button: replaces a since-removed
    // Up Ratio field that could be dialed in *simultaneously* with Ratio
    // -- see kStateRatioDirection's own comment in PluginCompressorStereo.hpp.
    // Linked mode only now -- always drives A's own ratioUpMode_/
    // "ratioDirection" unconditionally (Linked has no B bank at all).
    DGL_NAMESPACE::SimpleButton* btnRatioDirection_ = nullptr;

    // Dual-Mono/Mid-Side widget set -- both compressors side by side
    // (replacing the removed btnEditBank_/editingBank_ toggle).
    // graphA_/stripA_ always show/edit A's own bank; graphB_/
    // stripB_ show/edit B's own bank once independent, or live-mirror
    // A's (same underlying Parameters, not just the same numbers) while
    // linked -- see bParam()'s own comment.
    DGL_NAMESPACE::CompressorGraphWidget* graphA_ = nullptr;
    DGL_NAMESPACE::CompressorGraphWidget* graphB_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* stripA_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* stripB_ = nullptr;
    // Global Output Gain's own single home in the dual layout -- kept out
    // of stripA_/stripB_ themselves (which would otherwise mean 9 fields
    // squeezed into *half* this plugin's width each, versus 8 fields in
    // the *full* width strip_ already manages today) and shown once, to
    // the right of both, since it's one shared master trim either way
    // (kParamGlobalOutputGain's own comment, PluginCompressorStereo.hpp).
    DGL_NAMESPACE::ParamStripWidget* stripGlobal_ = nullptr;
    // Global Input Gain (Dual-Mono/Mid-Side), under the Input meter on the
    // far left -- the mirror-image control to stripGlobal_ above, at
    // the *input* end instead of the output end: one shared trim applied
    // to the raw stereo signal before it's split (Dual-Mono) or decoded
    // (Mid/Side) into A's/B's own separate signals -- see
    // kParamGlobalInputGain's own comment (PluginCompressorStereo.hpp).
    // Sits directly beneath meterIn_ in the dual layout's own bottom row,
    // shown only there (not in Linked mode) -- see
    // refreshDualLayoutVisibility()'s own comment.
    DGL_NAMESPACE::ParamStripWidget* stripGlobalIn_ = nullptr;
    // B's own Ratio Direction is a State (kStateRatioDirectionB), so it
    // can't be a ParamStripWidget field -- these sit just above their
    // respective graph instead. btnRatioDirectionB_ drives ratioUpModeB_
    // while independent, or ratioUpMode_ (A's own) while linked, same
    // bParam()-style fallback as everything else in the dual layout.
    DGL_NAMESPACE::SimpleButton* btnRatioDirectionA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRatioDirectionB_ = nullptr;

    DGL_NAMESPACE::LevelMeterWidget* meterIn_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* meterOut_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnStereoMode_ = nullptr;
    // External sidechain/key input toggle -- see kParamSidechainEnable's
    // own comment in PluginCompressorStereo.hpp.
    DGL_NAMESPACE::SimpleButton* btnSidechain_ = nullptr;
    // "a listen-function on m/s mode to hear only mids or only sides to
    // set up the compressor" -- see kParamListenMode's own comment in
    // PluginCompressorStereo.hpp. Cycles Off/Mid/Side; greyed out
    // (setEnabled(false)) whenever stereoMode_ isn't MidSide, since the
    // Parameter itself is inert outside that mode too. A single toggled
    // Listen is enough -- it stays this single shared button even in the new dual
    // layout, unlike Ratio Direction (a genuinely per-bank *value*)
    // Listen is a single momentary audition override, not part of either
    // bank's own sound.
    DGL_NAMESPACE::SimpleButton* btnListen_ = nullptr;
    // Link L/R parameters, or keep them separate -- see kStateLinkAB's own
    // comment (PluginCompressorStereo.hpp). Enabled only in Dual-Mono mode
    // (see updateMeterLabelsForMode()). Drives the live two-way link described in
    // bParam()'s own comment, not just a one-time copy.
    DGL_NAMESPACE::SimpleButton* btnLink_ = nullptr;
    // Toggles the sidechain-options overlay panel -- see
    // refreshScPanelVisibility()'s own comment and kParamScFilterEnable's
    // own comment (PluginCompressorStereo.hpp) for the full design.
    DGL_NAMESPACE::SimpleButton* btnSidechainOptions_ = nullptr;
    // The panel's own widgets -- constructed once, up front, like every
    // other widget in this file; only their *visibility* toggles.
    // Highpass/Lowpass/Sidechain Gain live in scStrip_, inside the
    // sidechain panel: SC Gain moved here from strip_'s own conditional field (see
    // refreshStripFields()'s own comment) now that there's a dedicated
    // home for every sidechain-shaping control.
    DGL_NAMESPACE::SimpleButton* btnScFilterEnable_ = nullptr;
    // Sidechain Listen (pass only the sidechain) -- auto-reset off whenever
    // the panel closes (refreshScPanelVisibility()'s own comment), same
    // "don't leave an audition override silently armed" precedent
    // kParamListenMode's own Mid/Side Listen already set for this
    // project (see its own updateMeterLabelsForMode() case).
    DGL_NAMESPACE::SimpleButton* btnScListen_ = nullptr;
    // A mode where left and right are filtered differently -- mirrors btnLink_'s own Link
    // pair, one level down inside the sidechain panel (unrelated to this
    // session's dual-compressor-layout feature, just a similarly-shaped
    // link/edit-bank pair for the sidechain filter's own L/R corners).
    DGL_NAMESPACE::SimpleButton* btnScLinkLR_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScEditBank_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* scStrip_ = nullptr;
    DGL_NAMESPACE::SidechainScopeWidget* scScope_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScClose_ = nullptr;
    bool scPanelVisible_ = false;
    float uiHeartbeat_ = 0.0f; // see uiIdle()
    bool scFilterLinkLR_ = true; // mirrors the DSP-side kParamScFilterLinkLR
    int editingScBankR_ = 0; // 0=L, 1=R -- which side scStrip_/scScope_ currently show/edit while unlinked
    DGL_NAMESPACE::SimpleButton* btnLoad_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_ = nullptr;

    DGL_NAMESPACE::SimpleButton* btnAbA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbB_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbCopy_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnUndo_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRedo_ = nullptr;

    // Hover descriptions -- constructed last, after every
    // other widget, so it always paints on top; see its own file comment
    // for why that means a standalone overlay rather than something built
    // into SimpleButton itself.
    DGL_NAMESPACE::TooltipOverlay* tooltip_ = nullptr;
    enum class AbSlot { A, B };
    AbSlot abActive_ = AbSlot::A;
    presetio::KeyValues abSlotA_, abSlotB_;
    bool abHasA_ = false, abHasB_ = false;

    std::vector<presetio::KeyValues> undoStack_, redoStack_;
    presetio::KeyValues lastCommittedSnapshot_, lastSeenSnapshot_;
    int idleTicksSinceChange_ = 0;
    bool undoRedoInitialized_ = false;
    bool applyingSnapshot_ = false;
    static constexpr int kUndoDebounceTicks = 15;
    static constexpr size_t kMaxUndoDepth = 100;

    float paramCache_[kParamCount];
    bool bypassed_ = false;
    sscomp::StereoMode stereoMode_ = sscomp::StereoMode::Linked;
    bool ratioUpMode_ = false; // mirrors the DSP-side state, see kStateRatioDirection
    bool sidechainEnabled_ = false; // mirrors the DSP-side kParamSidechainEnable
    int listenMode_ = 0; // mirrors the DSP-side kParamListenMode -- 0=Off/1=Mid/2=Side
    bool linkAB_ = true; // mirrors the DSP-side kStateLinkAB
    bool ratioUpModeB_ = false; // mirrors the DSP-side kStateRatioDirectionB

    enum class PendingFileOp { None, Load, Save } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    DISTRHO_DECLARE_NON_COPYABLE(UICompressorStereo)
};

END_NAMESPACE_DISTRHO
