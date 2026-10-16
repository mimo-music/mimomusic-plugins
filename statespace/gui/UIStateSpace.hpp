// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/StabilityGuard.hpp"
#include "../plugin/dsp/StateSpaceTypes.hpp"
#include "BodePlotWidget.hpp"
#include "MatrixGridWidget.hpp"
#include "ModelEditorHost.hpp"
#include "NonlinearPanelWidget.hpp"
#include "SimpleButton.hpp"

#include <string>
#include <vector>

START_NAMESPACE_DISTRHO

class UIStateSpace : public UI, public ssgui::IModelEditorHost
{
public:
    UIStateSpace();

protected:
    // -- DPF <-> UI sync --------------------------------------------------
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    // -- Drawing / layout / idle -----------------------------------------
    void onNanoDisplay() override;
    void onDisplay() override; // DPF's frame, with the real pixel ratio -- see EditorSubWidget.hpp
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    // -- Keyboard (drives the currently-active cell text edit) ----------
    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;

    // Commits any pending cell edit before forwarding the click down to
    // whichever subwidget it actually landed on (spreadsheet-style
    // "click elsewhere finishes the edit"), independent of whether the
    // Enter key reaches us -- see UIStateSpace.cpp for why that's not
    // guaranteed in every host/embedding.
    bool onMouse(const MouseEvent& ev) override;

    // -- File browser (Load/Save Preset) ---------------------------------
    void uiFileBrowserSelected(const char* filename) override;

    // -- ssgui::IModelEditorHost ------------------------------------------
    int getOrder() const override { return model_.order; }
    double getMatrixValue(ssgui::MatrixKind kind, int row, int col) const override;
    void setMatrixValue(ssgui::MatrixKind kind, int row, int col, double value) override;
    void beginCellTextEdit(ssgui::MatrixKind kind, int row, int col, bool clearValue = false) override;
    bool isEditingCell(ssgui::MatrixKind kind, int row, int col, std::string& liveText) const override;
    bool getNonlinearEnabled() const override { return model_.nonlinear.enabled; }
    void setNonlinearEnabled(bool enabled) override;
    double getNonlinearKnotGainDb(int knot) const override;
    void setNonlinearKnotGainDb(int knot, double gainDb) override;

private:
    void layout();
    void applyModelToEngine();     // re-runs the stability guard on model_, pushes it to the DSP, marks the plot dirty
    void refreshFromModel();       // repaints grids/plot/badge after model_ changed by any means (including from the DSP side)
    void commitEdit();
    void cancelEdit();
    void moveEditToNeighbor(int rowDelta, int colDelta); // commits current cell, then opens the adjacent one
    void changeOrder(int delta);
    void loadPresetFile(const char* path);
    void savePresetFile(const char* path);
    void rememberPresetDir(const char* filePath);
    void applyCompressorCurve(); // bakes compThresholdDb_/compRatio_/compMakeupDb_ into the 25 curve knots

    // -- A/B compare + undo/redo -- see ../../compressor's
    // UICompressorMono.cpp for the full design writeup this is ported
    // from (against this plugin's own JSON model, same idiom as every
    // other band-list-model plugin in this project -- e.g. gui/dyneq/
    // UIDynEq.cpp -- via ssfilter::Serialization::toJson()/fromJson()).
    void applyModelSnapshot(const std::string& json);

    void onAbA();
    void onAbB();
    void onAbCopy();
    void refreshAbColors();

    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    ssfilter::StateSpaceMatrices model_;
    ssfilter::StabilityReport lastReport_;

    // Transient (not part of the saved model -- see ModelEditorHost.hpp's
    // MatrixKind::Nonlinear comment) convenience fields behind the
    // nonlinear panel's "Apply Curve" button.
    double compThresholdDb_ = -18.0;
    double compRatio_ = 4.0;
    double compMakeupDb_ = 0.0;

    DGL_NAMESPACE::MatrixGridWidget* gridA_ = nullptr;
    DGL_NAMESPACE::MatrixGridWidget* gridB_ = nullptr;
    DGL_NAMESPACE::MatrixGridWidget* gridC_ = nullptr;
    DGL_NAMESPACE::MatrixGridWidget* gridD_ = nullptr;
    DGL_NAMESPACE::BodePlotWidget*   bode_  = nullptr;
    DGL_NAMESPACE::NonlinearPanelWidget* nlPanel_ = nullptr;

    DGL_NAMESPACE::SimpleButton* btnLoad_        = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSave_        = nullptr;
    DGL_NAMESPACE::SimpleButton* btnPassthrough_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnSilence_     = nullptr;
    DGL_NAMESPACE::SimpleButton* btnReset_       = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_      = nullptr;
    DGL_NAMESPACE::SimpleButton* btnNormalize_   = nullptr;
    DGL_NAMESPACE::SimpleButton* btnOrderMinus_  = nullptr;
    DGL_NAMESPACE::SimpleButton* btnOrderPlus_   = nullptr;

    DGL_NAMESPACE::SimpleButton* btnAbA_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbB_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnAbCopy_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnUndo_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRedo_ = nullptr;
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

    bool editingActive_ = false;
    ssgui::MatrixKind editKind_ = ssgui::MatrixKind::A;
    int editRow_ = -1, editCol_ = -1;
    std::string editBuffer_;

    bool plotDirty_ = true;
    int idleTicksSinceEdit_ = 0;

    enum class PendingFileOp { None, Load, Save } pendingFileOp_ = PendingFileOp::None;
    std::string lastPresetDir_;

    bool bypassed_ = false;
    int faultBlinkTicks_ = 0;

    DISTRHO_DECLARE_NON_COPYABLE(UIStateSpace)
};

END_NAMESPACE_DISTRHO
