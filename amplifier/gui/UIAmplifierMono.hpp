// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// UIAmplifierMono.hpp
//
// A small, dedicated GUI -- this project's very first (see
// DistrhoPluginInfo.h's own comment for why the original "no GUI at all"
// design changed): Listen/Normalize needs a live level readout and a
// start/stop control a host's own generic parameter editor can't give it,
// so once a GUI existed for that anyway, Gain/Bypass got real widgets too
// rather than staying host-generic while everything else got a custom one.
//
// Layout: Bypass + Listen buttons, a Peak/RMS toggle (a two-way swap
// button, same idiom as ../statespace's own Ratio
// Direction button -- one control that IS one of two states, not two
// independently-toggleable booleans that could both read "on"), a
// ParamStripWidget (Gain, Normalize Target -- see ../compressor/gui/
// ParamStripWidget.hpp, ported verbatim), and a live "Measured Level"
// readout drawn directly (see onNanoDisplay()) since it's just one number
// in a box, not worth its own widget class.
//
// applyListenResult() -- not the DSP -- is what actually turns a finished
// Listen session into a new Gain value: a plugin has no standard way to
// tell an LV2/VST3 host "one of your own input parameters changed
// internally" (a host only ever pushes values *to* a plugin for an input
// port, never polls one back), so PluginAmplifierMono.cpp deliberately
// only ever measures and reports (kParamMeterCurrentLevel); this class
// computes the new Gain from that report and its own NormalizeTarget
// value and pushes it via the UI's own setParameterValue() -- a real,
// reliable "tell the host" round trip, the same one an ordinary knob turn
// in this GUI already uses. See applyListenResult()'s own comment for the
// two call sites (a UI-driven Stop click, and a host-driven one, e.g. an
// automated Listen parameter) and why both are needed without double-
// applying.

#pragma once

#include "DistrhoUI.hpp"

#include "../plugin/dsp/AmplifierDsp.hpp"
#include "../plugin_mono/PluginAmplifierMono.hpp" // for ParameterIndex/kParamCount
#include "ParamStripWidget.hpp"
#include "PresetIO.hpp"
#include "SimpleButton.hpp"

#include <vector>

START_NAMESPACE_DISTRHO

class UIAmplifierMono : public UI
{
public:
    UIAmplifierMono();

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
    void applyListenResult(); // computes+pushes the new Gain -- see this file's own top comment

    // -- Undo/redo -- no A/B compare here (excluded from that rollout by
    // request -- "a compare function on all plugins but amplification"),
    // but undo/redo covers "all plugins" without that exclusion. Same
    // mechanism/reasoning as ../../compressor's UICompressorMono.cpp, see
    // that file's own comments for the full design writeup.
    presetio::KeyValues captureSnapshot() const;
    void applySnapshot(const presetio::KeyValues& values);
    void onUndo();
    void onRedo();
    void refreshUndoRedoColors();

    DGL_NAMESPACE::ParamStripWidget* strip_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnListen_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnMode_ = nullptr;
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
    ssamp::ListenMode mode_ = ssamp::ListenMode::Peak;

    float meterX_ = 0, meterY_ = 0, meterW_ = 0, meterH_ = 0;

    DISTRHO_DECLARE_NON_COPYABLE(UIAmplifierMono)
};

END_NAMESPACE_DISTRHO
