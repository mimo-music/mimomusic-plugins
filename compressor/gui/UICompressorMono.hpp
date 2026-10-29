// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin_mono/PluginCompressorMono.hpp"
#include "CompressorGraphWidget.hpp"
#include "LevelMeterWidget.hpp"
#include "ParamStripWidget.hpp"
#include "PresetIO.hpp"
#include "SimpleButton.hpp"
#include "TooltipOverlay.hpp"

#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

class UICompressorMono : public UI
{
public:
    UICompressorMono();

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
    void setGrMeterSpeed(DGL_NAMESPACE::GrMeterSpeed s, bool save);
    void layout();
    void refreshBypassColors();
    void refreshSidechainColors();
    void refreshStripFields(); // rebuilds strip_'s field list -- "SC Gain" only included while Sidechain is enabled

    // -- A/B compare + undo/redo --------------------------------------
    // Both reuse the exact same "whole configuration as one KeyValues
    // map" mechanism Save/Load already uses (captureSnapshot()/
    // applySnapshot(), factored out of what used to be save/loadPresetFile's
    // own bodies). Transient/session-only, like Load/Save itself: neither survives
    // closing and reopening the plugin editor.
    presetio::KeyValues captureSnapshot() const;
    void applySnapshot(const presetio::KeyValues& values);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();
    // Shared by graph_'s three drag/scroll callbacks (Threshold/Ratio/Knee
    // handles and Knee-scroll) -- mirrors what ParamStripWidget's own
    // setSetter() does for the strip's fields, so an edit made on the
    // graph and one made on the strip stay in sync regardless of which
    // widget the user actually touched.
    void onGraphParamChanged(uint32_t index, float value);
    void refreshRatioDirectionLabel();

    void savePresetFile(const char* path);
    void loadPresetFile(const char* path);
    void rememberPresetDir(const char* filePath);

    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* meterIn_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* meterOut_ = nullptr;
    DGL_NAMESPACE::CompressorGraphWidget* graph_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    // The Ratio direction swap button: replaces a since-removed
    // Up Ratio field that could be dialed in *simultaneously* with Ratio
    // -- see kStateRatioDirection's own comment in PluginCompressorMono.hpp.
    DGL_NAMESPACE::SimpleButton* btnRatioDirection_ = nullptr;
    // External sidechain/key input toggle -- see kParamSidechainEnable's
    // own comment in PluginCompressorMono.hpp.
    DGL_NAMESPACE::SimpleButton* btnSidechain_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnLoad_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_ = nullptr;

    // A/B compare: two toggle buttons (exactly one active at a time) plus
    // an explicit Copy -- clicking the inactive slot switches to it
    // (saving the slot being left first; the very first switch duplicates
    // the current settings into the newly-touched slot rather than
    // snapping to silence/defaults, matching the familiar FabFilter-style
    // "B starts as a copy of A" convention). Copy separately writes the
    // *active* slot's current settings into the *other* slot's own
    // snapshot without switching -- the literal "copy the configuration
    // to A or B" ask, independent of the compare-by-toggling gesture.
    DGL_NAMESPACE::SimpleButton* btnAbA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbB_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbCopy_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnUndo_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRedo_ = nullptr;

    // Hover descriptions -- see TooltipOverlay's own file comment;
    // constructed last so it always paints on top.
    DGL_NAMESPACE::TooltipOverlay* tooltip_ = nullptr;
    enum class AbSlot { A, B };
    AbSlot abActive_ = AbSlot::A;
    presetio::KeyValues abSlotA_, abSlotB_;
    bool abHasA_ = false, abHasB_ = false;

    // Undo/redo: a debounced checkpoint stack, not one entry per raw
    // ParamStripWidget setter call (which fires continuously while
    // dragging a fader -- see ParamStripWidget.hpp's own interaction
    // comment) -- uiIdle() below watches captureSnapshot() settle
    // (kUndoDebounceTicks consecutive idle ticks with no change) before
    // committing one checkpoint, the same debounce idiom the
    // ../statespace sibling project's own model-push-
    // to-engine debounce uses, just aimed at undo history instead.
    // Session-only: lost on closing the plugin editor, exactly like
    // Load/Save's own files are the only thing that persists across that.
    std::vector<presetio::KeyValues> undoStack_, redoStack_;
    presetio::KeyValues lastCommittedSnapshot_; // the checkpoint currently "live" -- what redo would return to after an undo
    presetio::KeyValues lastSeenSnapshot_;      // last idle tick's reading, for detecting "still changing" vs. "settled"
    int idleTicksSinceChange_ = 0;
    bool undoRedoInitialized_ = false; // false until the first settle -- see uiIdle()'s own comment for why the very first one never pushes
    bool applyingSnapshot_ = false;    // true while an A/B switch or undo/redo restore is itself calling applySnapshot() -- suppresses uiIdle() from recording that restore as a fresh edit
    static constexpr int kUndoDebounceTicks = 15;  // ~0.25-0.5s at typical uiIdle() rates -- long enough that a brief pause mid-drag doesn't split one gesture into two checkpoints
    static constexpr size_t kMaxUndoDepth = 100;

    float paramCache_[kParamCount];
    bool bypassed_ = false;
    bool ratioUpMode_ = false; // mirrors the DSP-side state, see kStateRatioDirection
    bool sidechainEnabled_ = false; // mirrors the DSP-side kParamSidechainEnable

    enum class PendingFileOp { None, Load, Save } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    DISTRHO_DECLARE_NON_COPYABLE(UICompressorMono)
};

END_NAMESPACE_DISTRHO
