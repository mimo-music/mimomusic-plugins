// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoUI.hpp"

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/EqModel.hpp"
#include "../plugin/dsp/ReverbEngine.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"
#include "../plugin/ReverbParams.hpp" // ParameterIndex/StateIndex, shared across every model's own plugin -- see that file's own header comment for why this, not any one model's PluginReverbXxx.hpp, is what this GUI depends on
#include "ParamStripWidget.hpp"
#include "SimpleButton.hpp"
#include "TooltipOverlay.hpp"
#include "eq/BandListWidget.hpp"
#include "eq/EqEditorHost.hpp"
#include "eq/EqGraphWidget.hpp"

#include <string>

START_NAMESPACE_DISTRHO

// v1 GUI: room dimensions + the reverb's own numeric fields via
// ParamStripWidget (ported verbatim, same widget every sibling plugin in
// this repository already uses), a post-tank parametric EQ via
// EqGraphWidget/BandListWidget (also ported verbatim, driving a *local*
// ssfilter::EqModel that's pushed to the DSP as a whole via
// setState("eqModel", ...) -- simpler than the sibling statespace/
// project's own Parameter-mirror mechanism, which exists specifically to
// work around a narrow Ardour route-copy persistence bug neither this
// project nor this environment can currently reproduce/verify; see
// docs/REVERB_CONCEPT.md's GUI section for that tradeoff spelled out).
//
// Not included (see docs/REVERB_CONCEPT.md's "Known limitations"): A/B
// compare, Load/Save/Export, the EQ's right-click audition-hold (a no-op
// here, since there is no audition DSP), per-field tooltips
// (TooltipOverlay is ported but not wired to every field), and room-shape
// visualization.
class UIReverb : public UI, public ssgui::IEqEditorHost
{
public:
    UIReverb();

protected:
    void parameterChanged(uint32_t index, float value) override;
    void stateChanged(const char* key, const char* value) override;

    void onNanoDisplay() override;
    void onDisplay() override; // DPF's frame, with the real pixel ratio -- see EditorSubWidget.hpp
    void onResize(const ResizeEvent& ev) override;
    void uiIdle() override;

    bool onCharacterInput(const CharacterInputEvent& ev) override;
    bool onKeyboard(const KeyboardEvent& ev) override;

    // -- ssgui::IEqEditorHost -----------------------------------------
    int getBandCount() const override { return int(eqModel_.bands.size()); }
    ssfilter::EqBand getBand(int index) const override;
    ssfilter::EqModel getModel() const override { return eqModel_; }
    void setBandFreq(int index, double freqHz) override;
    void setBandGain(int index, double gainDb) override;
    void setBandQ(int index, double q) override;
    void setBandType(int index, ssfilter::EqBandType type) override;
    void setBandSlopeStages(int index, int slopeStages) override;
    void setBandEnabled(int index, bool enabled) override;
    void setBandSolo(int index, bool solo) override;
    void setBandChannel(int index, ssfilter::EqChannel channel) override;
    void addBand(double freqHz, double gainDb) override;
    void removeBand(int index) override;
    void duplicateBand(int index) override;
    void createHarmonicTwin(int index, double factor) override;
    void unlinkHarmonic(int index) override;
    // No audition DSP behind this plugin's own EqModel path -- see class
    // comment. Same no-op-pair convention the sibling EQ Mono plugin uses.
    void beginBandAudition(int) override {}
    void endBandAudition() override {}
    void beginBandTextEdit(int index, ssgui::EqField field, bool clearValue = false) override;
    bool isEditingBandField(int index, ssgui::EqField field, std::string& liveText) const override;
    // No harmonic-factor text-edit UI wired in v1 (the right-click menu's
    // "Custom Factor..." row still works via a fixed factor set below) --
    // always "not editing".
    void beginHarmonicFactorEdit(int) override {}
    bool isEditingHarmonicFactor(int, std::string&) const override { return false; }
    void commitHarmonicFactorEdit() override {}

private:
    void layout();
    void selectBand(int index);
    void removeSelectedBand();

    void rebuildMainFields(); // Decay Time swaps editable/read-only with tempoSyncEnabled_ -- see .cpp
    void pushEqModel();       // setState("eqModel", eq::toJson(eqModel_)) + repaint

    float paramGetter(uint32_t fieldId) const;
    void paramSetter(uint32_t fieldId, float value);
    float roomGetter(uint32_t fieldId) const;
    void roomSetter(uint32_t fieldId, float value);

    DGL_NAMESPACE::ParamStripWidget* roomStrip_ = nullptr;
    DGL_NAMESPACE::ParamStripWidget* mainStrip_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnBypass_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnReset_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnTempoSync_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnFollowHost_ = nullptr;
    DGL_NAMESPACE::SimpleButton* btnNoteValue_ = nullptr;
    DGL_NAMESPACE::EqGraphWidget* graph_ = nullptr;
    DGL_NAMESPACE::BandListWidget* list_ = nullptr;
    // The selected EQ band's own large fields, below the band list (same
    // design as the Parametric EQ's band strip): Freq/Gain/Q,
    // "(unused)" where the band's type ignores a value. Rebuilt only on a
    // selection/type change, never while a field is being edited.
    DGL_NAMESPACE::ParamStripWidget* bandStrip_ = nullptr;
    int bandStripKey_ = -1;
    float bandStripHeaderX_ = 0.0f, bandStripHeaderY_ = 0.0f;
    void refreshBandStrip();
    DGL_NAMESPACE::TooltipOverlay* tooltip_ = nullptr; // constructed LAST -- see its own file comment

    // Local UI-side mirrors, kept current by parameterChanged()/
    // stateChanged() -- same convention every sibling UI class in this
    // repository already follows (DSP/UI can be in separate processes,
    // Parameters/State are the only channel between them).
    double roomLengthM_ = 10.0, roomWidthM_ = 7.0, roomHeightM_ = 3.5;
    double inputGainDb_ = 0.0;
    double preDelayMs_ = 0.0;
    double decayTimeSec_ = 1.8;
    double damping_ = 0.5;
    bool tempoSyncEnabled_ = false;
    bool followHostTempo_ = false;
    int tempoNoteValueIndex_ = int(ssfilter::reverb::TempoNoteValue::Quarter);
    double dryWet_ = 35.0; // 0..100%, matching ../compressor's own kParamWetDry convention -- see PluginReverb.cpp's kParamDryWet comment
    double outputTrimDb_ = 0.0;
    double diffusionAmount_ = 0.7;
    double widthAmount_ = 1.0;
    double balanceAmount_ = 0.0;
    double modulationAmount_ = 0.3;
    bool bypass_ = false;

    // Diagnostic readout -- added when the BPM-dependent decay seemed to
    // have no audible effect: makes the tempo-sync
    // mechanism's live state visible instead of only inferred by ear.
    bool hostTempoValid_ = false;
    double hostBpmDetected_ = 0.0;
    double currentDecayTimeSec_ = 1.8;

    ssfilter::EqModel eqModel_;
    int selectedBand_ = -1;

    // Pre-/post-EQ spectrum on the filter plot. Same raw/smoothed-array
    // + one-pole-toward-raw-every-tick shape as every sibling EQ plugin's
    // own identical mechanism (see uiIdle()'s own comment for why the
    // smoothing lives here, UI-side, not on the DSP side).
    float spectrumInRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumInSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutRaw_[ssfilter::SpectrumAnalyzer::kNumBands];
    float spectrumOutSmoothed_[ssfilter::SpectrumAnalyzer::kNumBands];
    float uiHeartbeatToggle_ = 0.0f;

    // Layout rectangles, computed once in layout() and reused by
    // onNanoDisplay() for the diagnostic text -- same "compute in
    // layout(), draw in onNanoDisplay()" split every sibling UI uses.
    float diagTextX_ = 0, diagTextY_ = 0;

    DISTRHO_DECLARE_NON_COPYABLE(UIReverb)
};

END_NAMESPACE_DISTRHO
