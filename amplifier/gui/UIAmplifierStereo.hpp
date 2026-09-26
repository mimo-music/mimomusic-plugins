// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// UIAmplifierStereo.hpp
//
// See UIAmplifierMono.hpp's own file comment for the general design --
// identical here, plus Invert L/Invert R (two independent toggle buttons,
// not a swap button -- both can be on at once, unlike Peak/RMS) and
// Balance in the strip. Also see that file's own comment on
// applyListenResult() for why turning Gain into the actual new value
// after a Listen session has to happen here, not in the DSP.

#pragma once

#include "DistrhoUI.hpp"

#include "../plugin/dsp/AmplifierDsp.hpp"
#include "../plugin_stereo/PluginAmplifierStereo.hpp" // for ParameterIndex/kParamCount
#include "ParamStripWidget.hpp"
#include "PresetIO.hpp"
#include "SimpleButton.hpp"

#include <vector>

START_NAMESPACE_DISTRHO

class UIAmplifierStereo : public UI
{
public:
    UIAmplifierStereo();

protected:
    void parameterChanged(uint32_t index, float value) override;

    void onNanoDisplay() override;
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;

private:
    void layout();
    void refreshListenColors();
    void refreshModeLabel();
    void refreshInvertColors();
    void applyListenResult(); // computes+pushes the new Gain -- see UIAmplifierMono.cpp's own comment

    // -- Undo/redo -- see UIAmplifierMono.hpp's identical block for why
    // there's no A/B compare here. Same mechanism as ../../compressor's
    // UICompressorMono.cpp.
    presetio::KeyValues captureSnapshot() const;
    void applySnapshot(const presetio::KeyValues& values);
    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnListen_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnMode_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnInvertL_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnInvertR_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnUndo_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnRedo_ = nullptr;

    std::vector<presetio::KeyValues> undoStack_, redoStack_;
    presetio::KeyValues lastCommittedSnapshot_, lastSeenSnapshot_;
    int idleTicksSinceChange_ = 0;
    bool undoRedoInitialized_ = false;
    bool applyingSnapshot_ = false;
    static constexpr int kUndoDebounceTicks = 15;
    static constexpr size_t kMaxUndoDepth = 100;

    float paramCache_[kParamCount];
    bool bypassed_ = false;
    bool listening_ = false;
    bool invertL_ = false;
    bool invertR_ = false;
    ssamp::ListenMode mode_ = ssamp::ListenMode::Peak;

    float meterX_ = 0, meterY_ = 0, meterW_ = 0, meterH_ = 0;

    DISTRHO_DECLARE_NON_COPYABLE(UIAmplifierStereo)
};

END_NAMESPACE_DISTRHO
