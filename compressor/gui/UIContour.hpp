// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "../plugin/dsp/CurveDsp.hpp"
#include "../plugin_contour/PluginContour.hpp"
#include "ContourGraphWidget.hpp"
#include "LevelMeterWidget.hpp"
#include "ParamStripWidget.hpp"
#include "PresetIO.hpp"
#include "SidechainScopeWidget.hpp"
#include "NodeListWidget.hpp"

#include <chrono>
#include "SimpleButton.hpp"
#include "TooltipOverlay.hpp"

#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

// A full undo/redo or A/B snapshot needs both curves' own point lists
// alongside the ordinary scalar controls -- with the same features as the
// Compressor plugin, dragging a curve point has to be undoable exactly like nudging
// Threshold already is, and switching A/B has to bring both curves along
// with it too. presetio::KeyValues (std::map<std::string,double>, what
// every sibling plugin's own snapshot already is) can't hold either
// curve's text encoding -- its values are strictly numeric -- so this
// pairs them alongside the scalar map instead of trying to force a curve
// into a double. `curveB` mirrors `curve` for B's own independent bank
// (Contour differs from the Compressor only in having a free-drawn
// curve): B's own dynamics bank needs its own
// free-drawable curve too, once independent, same as every other
// per-bank field in this project.
struct ContourSnapshot
{
    presetio::KeyValues values;
    std::string curve;  // sscomp::curveToString()'s own encoding -- A's own
    std::string curveB; // B's own, same encoding
    bool operator==(const ContourSnapshot& o) const
    {
        return values == o.values && curve == o.curve && curveB == o.curveB;
    }
    bool operator!=(const ContourSnapshot& o) const { return !(*this == o); }
};

// Top-level UI for the Contour plugin -- structurally a near-twin of
// UICompressorStereo (same meters/strip/A-B/undo-redo/Load-Save/
// sidechain-panel/tooltip machinery, ported essentially verbatim against
// this plugin's own Parameter list -- windows, sidechain, tooltips and
// input/output gain controls all the same), with the one real difference this plugin has always had: the graph is
// ContourGraphWidget (curve editing) instead of CompressorGraphWidget,
// and B's own independent bank (once Dual-Mono/Mid-Side makes it live)
// gets its own free-drawable curve, not a Ratio/Knee pair.
class UIContour : public UI
{
public:
    UIContour();

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;

    void uiFileBrowserSelected(const char* filename) override;

private:
    void layout();
    void refreshBypassColors();
    void refreshSidechainColors();
    void refreshStripFields();       // Linked mode's strip_ -- unchanged shape, plus Wet/Dry and Output
    void refreshDualStripFields();   // stripA_/stripB_ -- see UICompressorStereo's own identical method
    // Returns paramB when isB is true and B is currently independent,
    // paramA otherwise -- see UICompressorStereo::bParam()'s own
    // declaration comment, verbatim port.
    uint32_t bParam(bool isB, uint32_t paramA, uint32_t paramB) const;
    void onGraphParamChanged(uint32_t index, float value);
    // isB selects which of curve_/curveB_ (or, while B isn't independent,
    // curve_ again -- editing graphB_ then live-links back to A's own,
    // same bParam() spirit applied to a whole curve object instead of a
    // single scalar Parameter, since there's no shared-Parameter-index
    // trick possible for a multi-point curve the way there is for a
    // scalar field) actually gets the edit.
    void onGraphCurveChanged(bool isB, const sscomp::TransferCurve& curve);
    // Re-applies curve_/curveB_(-or-A's-own-fallback) to every graph that
    // shows them right now -- graph_ and graphA_ always get curve_;
    // graphB_ gets curveB_ once independent, curve_ otherwise. Same single
    // chokepoint reasoning as UICompressorStereo::refreshGraphCurves().
    void refreshGraphCurves();
    // Same idea, for Threshold and Trigger Condition specifically (both
    // live outside the curve object itself, see CurveDsp.hpp's own header
    // comment on why Threshold is deliberately independent of any curve
    // point).
    void refreshGraphThresholds();
    // Pushes the configured Makeup Gain of each column (B's own only while independent) into the graphs' gain-reduction bars.
    void refreshMakeupMarkers();
    void refreshGraphConditions();
    // Pushes curve_/condition_ (isB false) or curveB_/conditionB_ (isB
    // true) into PluginContour's Parameter mirror -- see
    // UIContour::pushCurveParamMirror()'s own original comment for why
    // this has to happen from here. Always pushes *both* banks' own
    // mirror regardless of which one isB actually names (cheap, and keeps
    // every mirror field current for adoptMirrorIfCurveNotReceived()'s own
    // restore path regardless of which bank a host happens to restore
    // first) -- matches PluginContour::applyMirrorToModel()'s own "applies
    // both banks every time" comment.
    void pushCurveParamMirror();
    void pushMirrorCommit();
    void pushAbState();
    std::string serializeSnapshot(const ContourSnapshot& snap) const;
    bool deserializeSnapshot(const std::string& text, ContourSnapshot& out) const;
    void refreshStereoModeLabel();
    // btn's own label reflects `cond`/`interp` -- shared by both A's own
    // (Linked-mode, and the dual layout's own A column) and B's own
    // buttons, same "one function, every call site passes its own value"
    // convention as UICompressorStereo::refreshRatioDirectionLabel().
    void refreshConditionLabel(DGL_NAMESPACE::SimpleButton* btn, sscomp::TriggerCondition cond);
    // The open/close switch's label: what a met trigger does -- go to the drawn Curve, or to the Diagonal.
    void refreshActionLabel(DGL_NAMESPACE::SimpleButton* btn, sscomp::TriggerAction action);
    void refreshActionLabels();
    // Flips the trigger action of curve A (or B while it is independent; otherwise A, like every B control).
    void toggleTriggerAction(bool isB);
    void updateMeterLabelsForMode();
    void refreshListenColors();

    // Tool mode (Draw Node / Draw Rising / Draw Falling / Edit). Applies
    // to every graph widget.
    void refreshToolButton();
    // Numeric node list(s) docked right of the graph(s): visibility toggle
    // and re-syncing rows. One list in Linked mode; two, stacked vertically
    // (nodeList_ on top for A, nodeListB_ below for B), in Dual-Mono/Mid-
    // Side: side-by-side would have squeezed graphA_/graphB_ even
    // further, so the split is vertical, in the same single-column width
    // the one list already used.
    void refreshNodeListButtons();
    void refreshNodeList();
    // In Mid/Side the two compressors can be set completely differently --
    // see
    // PluginCompressorStereo.hpp's kParamThresholdB's own comment for the
    // full design this mirrors.
    bool isChannelBIndependent() const;
    void refreshLinkLabel();
    // Shows/hides the Linked-mode widget set (graph_/strip_/btnCondition_/
    // btnResetCurve_) vs. the dual Dual-Mono/Mid-Side set (graphA_/graphB_/
    // stripA_/stripB_/the per-bank Condition/Reset buttons) -- verbatim
    // port of UICompressorStereo::refreshDualLayoutVisibility()'s own
    // reasoning.
    void refreshDualLayoutVisibility();
    // Copies A's current dynamics values (Input Gain, Threshold, Attack,
    // Release, Makeup, Wet/Dry, and -- the one addition beyond
    // UICompressorStereo's own identical method -- the curve itself and
    // its own trigger condition) into B's own storage -- "start linked,
    // split on edit": called only from a live user action, never from a
    // restore.
    void syncBFromA();

    // "a button for sidechain options that opens another window" (direct
    // ask, ported) -- see UICompressorStereo::refreshScPanelVisibility()'s
    // own comment for the full in-window-overlay-not-a-second-Window
    // design this is a verbatim port of.
    void refreshScPanelVisibility();
    void refreshScListenColors();
    void refreshScLinkLabel();
    void refreshScStripFields();

    void savePresetFile(const char* path);
    void loadPresetFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- see UICompressorStereo.cpp for the
    // full design writeup this is ported from, and ContourSnapshot's own
    // comment above for the one real difference (both curves ride
    // alongside, not through, the ordinary scalar map).
    ContourSnapshot captureSnapshot() const;
    void applySnapshot(const ContourSnapshot& snapshot);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    // Whichever of strip_/stripA_/stripB_/stripGlobal_/stripGlobalIn_/
    // scStrip_ is currently mid-edit, or nullptr -- see
    // UICompressorStereo::editingStrip()'s own comment, verbatim port.
    DGL_NAMESPACE::ParamStripWidget* editingStrip() const;
    // Same idea for nodeList_/nodeListB_ (both can be visible and either
    // can be mid-edit at once now that Dual-Mono/Mid-Side show both).
    DGL_NAMESPACE::NodeListWidget* editingNodeList() const;

    // Linked-mode-only widget set -- Linked mode keeps its original look,
    // as on the sibling compressor. See
    // refreshDualLayoutVisibility()'s own comment.
    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::ContourGraphWidget* graph_ = nullptr;
    // Above/Below -- see sscomp::TriggerCondition's own comment. Linked
    // mode only now, greyed out (not hidden) otherwise -- same convention
    // UICompressorStereo::btnRatioDirection_ already established.
    DGL_NAMESPACE::SimpleButton* btnCondition_ = nullptr;
    // "On trigger: Curve / Diagonal" -- see sscomp::TriggerAction. Same Linked-only convention as btnCondition_.
    DGL_NAMESPACE::SimpleButton* btnAction_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnResetCurve_ = nullptr;

    // Tool row above the graph(s) + the node list on the right.
    DGL_NAMESPACE::SimpleButton* btnTool_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnNodeList_ = nullptr;
    // What the trail dots' x means: real input level (default) or detector level -- see
    // kParamTrailSource. The switch lives in the second button row.
    DGL_NAMESPACE::SimpleButton* btnDots_ = nullptr;
    // Time of our own last mirror push -- see parameterChanged()'s mirror-rebuild case.
    std::chrono::steady_clock::time_point lastLocalMirrorEdit_{};
    bool hasLocalMirrorEdit_ = false;
    float uiHeartbeat_ = 0.0f; // see uiIdle()
    void refreshDotsButton();
    // nodeList_ always shows/edits curve_ (A / Linked); nodeListB_ always
    // shows/edits curveB_ (or curve_ while B isn't independent, same
    // fallback graphB_ already follows) -- see the header comment above
    // refreshNodeListButtons(). nodeListB_ is nullptr-checked nowhere it
    // doesn't need to be: both are constructed together in the ctor and
    // torn down together.
    DGL_NAMESPACE::NodeListWidget* nodeList_ = nullptr;
    DGL_NAMESPACE::NodeListWidget* nodeListB_ = nullptr;
    DGL_NAMESPACE::ContourGraphWidget::EditTool tool_ = DGL_NAMESPACE::ContourGraphWidget::EditTool::DrawNode;
    bool nodeListVisible_ = true;

    // Dual-Mono/Mid-Side widget set -- both compressors side by side.
    // graphA_/stripA_ always show/edit A's own
    // bank; graphB_/stripB_ show/edit B's own bank once independent, or
    // live-mirror A's (same underlying data, not just the same numbers)
    // while linked -- see bParam()'s and onGraphCurveChanged()'s own
    // comments.
    DGL_NAMESPACE::ContourGraphWidget* graphA_ = nullptr;
    DGL_NAMESPACE::ContourGraphWidget* graphB_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* stripA_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* stripB_ = nullptr;
    // Global Output Gain's own single home, on the far right (as on the
    // compressor). Global Input Gain's own home, under meterIn_ on the far
    // left.
    DGL_NAMESPACE::ParamStripWidget* stripGlobal_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* stripGlobalIn_ = nullptr;
    // Each column's own Condition/Reset pair, sitting just above its own
    // graph -- the dual layout's counterpart to btnCondition_/
    // btnResetCurve_ above. btnConditionB_/btnResetCurveB_ both follow
    // the same bParam()-style live-link-while-not-independent fallback
    // every other B-bank control in the dual layout already follows --
    // never disabled, editing them while linked edits A's own actual
    // state instead. (No more Interp button -- interpolation moved
    // per-point, via a right-click-drag on the point's own handle
    // directly in the graph -- see ContourGraphWidget.hpp's own header
    // comment.)
    DGL_NAMESPACE::SimpleButton* btnConditionA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnActionA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnResetCurveA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnConditionB_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnActionB_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnResetCurveB_ = nullptr;

    DGL_NAMESPACE::LevelMeterWidget* meterIn_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* meterOut_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnStereoMode_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSidechain_ = nullptr;
    // Listen in M/S mode (only Mid or only Side) -- a single shared toggle
    // button even in the dual layout.
    DGL_NAMESPACE::SimpleButton* btnListen_ = nullptr;
    // Link L/R parameters. Enabled only in Dual-Mono mode.
    DGL_NAMESPACE::SimpleButton* btnLink_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSidechainOptions_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScFilterEnable_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScListen_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScLinkLR_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScEditBank_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* scStrip_ = nullptr;
    DGL_NAMESPACE::SidechainScopeWidget* scScope_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnScClose_ = nullptr;
    bool scPanelVisible_ = false;
    bool scFilterLinkLR_ = true;
    int editingScBankR_ = 0;

    DGL_NAMESPACE::SimpleButton* btnLoad_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_ = nullptr;

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
    ContourSnapshot abSlotA_, abSlotB_;
    bool abHasA_ = false, abHasB_ = false;

    std::vector<ContourSnapshot> undoStack_, redoStack_;
    ContourSnapshot lastCommittedSnapshot_, lastSeenSnapshot_;
    int idleTicksSinceChange_ = 0;
    bool undoRedoInitialized_ = false;
    bool applyingSnapshot_ = false;
    static constexpr int kUndoDebounceTicks = 15;
    static constexpr size_t kMaxUndoDepth = 100;

    float paramCache_[kParamCount];
    float lastTrailBankA_[kTrailBankSize] = {}, lastTrailBankB_[kTrailBankSize] = {}; // to skip re-pushing an unchanged bank
    // How engaged the trigger is, per voice (0..1): the fraction of the newest bank's dots that met
    // the trigger condition, smoothed with the attack (rising) / release (falling) times -- drives the
    // threshold turning green and fading back to yellow. See uiIdle().
    float trigFracA_ = 0.0f, trigFracB_ = 0.0f, trigEnvA_ = 0.0f, trigEnvB_ = 0.0f;
    std::chrono::steady_clock::time_point lastTrigTick_{};
    bool bypassed_ = false;
    sscomp::StereoMode stereoMode_ = sscomp::StereoMode::Linked;
    sscomp::TriggerCondition condition_ = sscomp::TriggerCondition::Above;
    sscomp::TriggerAction action_ = sscomp::TriggerAction::ApplyCurve;
    sscomp::TransferCurve curve_;
    // B's own independent curve/condition -- see ContourSnapshot::
    // curveB's own comment. (Interpolation is no longer curve-wide --
    // see ContourGraphWidget.hpp's own header comment -- so there's no
    // B-bank interp to mirror here any more, only each point's own
    // handle, which rides inside curve_/curveB_ themselves.)
    sscomp::TriggerCondition conditionB_ = sscomp::TriggerCondition::Above;
    sscomp::TriggerAction actionB_ = sscomp::TriggerAction::ApplyCurve;
    sscomp::TransferCurve curveB_;
    bool sidechainEnabled_ = false;
    bool linkAB_ = true;
    int listenMode_ = 0;

    enum class PendingFileOp { None, Load, Save } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    // Must actually *change* every push, not just get resent -- see
    // ../statespace/gui/eq/UIEq.hpp's identical uiHeartbeatToggle_ comment.
    float mirrorCommitCounter_ = 0.0f; // increases on every edit, see pushMirrorCommit()

    DISTRHO_DECLARE_NON_COPYABLE(UIContour)
};

END_NAMESPACE_DISTRHO
