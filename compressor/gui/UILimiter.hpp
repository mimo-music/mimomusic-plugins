// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"
#include "../plugin_limiter/PluginLimiter.hpp"
#include "LevelMeterWidget.hpp"
#include "LimiterGraphWidget.hpp"
#include "ParamStripWidget.hpp"
#include "PresetIO.hpp"
#include "SimpleButton.hpp"
#include "TooltipOverlay.hpp"

#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

// The Limiter's editor. Top row: Load/Save, Bypass, A/B, Undo/Redo. Row 2,
// the limiting options: Algorithm, Ratio (Classic only), Clip mode
// (Hard/Soft), Oversampling, Boost. LimiterGraphWidget shows the capped
// gain staging and the soft clip band; the strip below holds the numeric
// fields. Switching the Algorithm also sets its default Clip mode
// (defaultClipModeFor()); the Clip button changes it freely afterwards.
class UILimiter : public UI
{
public:
    UILimiter();

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
    void layout();
    void refreshBypassColors();
    void refreshBoostColors();
    void refreshAlgorithmColors();
    void refreshClipModeColors();
    void refreshOversamplingColors();
    void refreshRatioButton(); // label/colour; disabled in True Peak (always infinity:1 there)
    void refreshStripFields(); // labels flag fields unused by the current Algorithm/Clip mode, see .cpp
    void refreshGraphCurve();  // recomputes the gain staging and hands it to the graph
    void setParam(uint32_t index, float value); // paramCache_ + host, the one way the UI changes a Parameter
    void onGraphParamChanged(uint32_t index, float value);

    void savePresetFile(const char* path);
    void loadPresetFile(const char* path);
    void rememberPresetDir(const char* filePath);

    // -- A/B compare + undo/redo -- see this project's own
    // UICompressorMono.cpp for the full design writeup this is ported
    // from (against this plugin's own parameter list/algorithm state).
    presetio::KeyValues captureSnapshot() const;
    void applySnapshot(const presetio::KeyValues& values);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* meterIn_ = nullptr;
    DGL_NAMESPACE::LevelMeterWidget* meterOut_ = nullptr;
    DGL_NAMESPACE::LimiterGraphWidget* graph_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBoost_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAlgorithm_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnClipMode_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnOversampling_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRatio_ = nullptr;
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
    bool boost_ = false; // mirrors kParamBoost

    // Mirrors the plugin's "algorithm" State (Classic until the host says
    // otherwise).
    LimiterAlgorithm algorithm_ = LimiterAlgorithm::Classic;

    enum class PendingFileOp { None, Load, Save } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    DISTRHO_DECLARE_NON_COPYABLE(UILimiter)
};

END_NAMESPACE_DISTRHO
