// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// SidechainScopeWidget.hpp
//
// The sidechain-options panel's own graphic -- the input signal as a
// waveform and as a frequency spectrum, both at once (not a toggle).
// Lives in an in-window overlay
// panel toggled by UICompressorStereo's own "Sidechain Options..."
// button (see that class's own refreshScPanelVisibility() comment for
// why this is an overlay within the *same* plugin editor window, not a
// second native OS window -- an earlier version of this feature tried
// that and crashed the host). Two stacked panels, both always visible:
//
//   - Top: two live magnitude spectrum traces, pre-filter ("In") and
//     post-filter/whatever's-currently-reaching-the-detector ("Out") --
//     the same dual-trace convention (and colors) as
//     ../../statespace's own EqGraphWidget. Same log-frequency mapping
//     and packed-triple unpacking convention as that widget too, fed
//     from sscomp::SpectrumAnalyzer's packed output (see
//     kParamMeterScSpectrumBase/InBase's own comment). The filter's own
//     high-pass/low-pass corners are draggable directly on the spectrum,
//     with the frequency shown right there -- ported from ../../statespace's own
//     MultibandGraphWidget crossover-handle interaction (vertical line +
//     triangular knob, hover/drag highlight, a floating Hz/kHz readout
//     chip near whichever handle is live). A shaded region between the
//     two fills in only while the filter is actually engaged, same
//     on/off distinction the multiband's own band-enable dimming uses.
//   - Bottom: two rectified level traces, one per channel: Left
//     drawn upward from a center baseline, Right mirrored downward,
//     both from kParamMeterScLevelDbL/R (one smoothed rectified peak
//     reading per audio block, accumulated client-side into a scrolling
//     ring buffer, same "one value per tick" technique
//     CompressorGraphWidget::drawGrHistory() already uses for the GR
//     history strip -- true per-sample waveform data has no portable
//     cross-process transport in this framework, see
//     SpectrumAnalyzer.hpp's own header comment for the identical
//     reasoning applied to the spectrum above). Plotted as *linear*
//     amplitude, clamped to a max of 1.0 (0dBFS) -- back from an
//     intermediate dB-mapped version, which read worse: the clamp alone
//     (not a squashed dB-extent mapping) is what actually fixes the
//     waveform overlapping the plot at high gain (the real bug: unclamped
//     linear amplitude from a heavily boosted Sidechain Gain could
//     exceed the panel's own bounds), and reading directly against the
//     y-axis dB grid drawn using the exact same (non-uniform, since
//     dB-to-linear is exponential) mapping still tells you the real level a peak represents.

#pragma once

#include "EditorSubWidget.hpp"

#include "../plugin/dsp/SpectrumAnalyzer.hpp"

#include <functional>
#include <string>

START_NAMESPACE_DGL

class SidechainScopeWidget : public EditorSubWidget
{
public:
    explicit SidechainScopeWidget(NanoTopLevelWidget* parent);

    // One packed-triple Parameter's worth of the spectrum -- called once
    // per received kParamMeterScSpectrumBase/InBase+N Parameter (128
    // each), same call shape as SpectrumAnalyzer::unpackTriple()'s own
    // callers elsewhere in this project family.
    void setSpectrumInPacket(int packetIndex, float packedValue);
    void setSpectrumOutPacket(int packetIndex, float packedValue);

    // Latest known L/R level -- called whenever the host actually
    // delivers a fresh kParamMeterScLevelDbL/R value. Just overwrites the
    // ring buffer's current (not-yet-committed) head slot; does *not*
    // advance the buffer itself -- see tickLevelHistory()'s own comment
    // for why advancing has to happen on a fixed timer instead of here.
    void pushLevelDbL(float db);
    void pushLevelDbR(float db);

    // Advances the ring buffer by one slot -- call once per UI idle tick
    // (owning UI's uiIdle()), unconditionally, regardless of whether
    // either pushLevelDb call above actually fired since the last tick.
    // A real bug (no waveform visible, just a small peak at the left edge
    // of the plot): the original
    // version advanced the buffer only from inside pushLevelDbR() itself,
    // tying the *scroll rate* to the host's own output-parameter change
    // notifications -- which, for a peak-with-decay meter, throttle or
    // stop outright the moment the value settles (silence, or a steady
    // tone whose peak stops changing bit-for-bit) -- freezing the whole
    // scrolling window at whatever it last looked like, even while live
    // audio keeps flowing through the actual DSP. Decoupling the advance
    // from parameter delivery entirely fixes that: the trace now always
    // scrolls at a steady, host-independent rate, same "one value per
    // tick" technique CompressorGraphWidget::drawGrHistory() already
    // uses for the GR history strip.
    void tickLevelHistory();

    // The filter's own passband -- drives both the draggable handles'
    // own live positions and the shaded overlay (enabled=false hides the
    // fill but leaves the handles themselves draggable, so the corners
    // can be set up before actually engaging the filter). Always L's own
    // corners -- in the mode where left and right are filtered
    // differently, R's own independent corners (while
    // unlinked) aren't drawn on this shared spectrum at all, only
    // reachable via the panel's own L/R edit-bank tab, same reasoning
    // ../../compressor's own A/B dynamics-bank editing keeps only one
    // bank's shape on screen at a time.
    void setFilterRange(bool enabled, float highpassHz, float lowpassHz);

    // Tints the passband highlight and the draggable handles red
    // while the corners currently shown/dragged are R's own (editingScBankR_
    // == 1 on the owning UI), matching the red used for R's own level
    // trace below, instead of always using the same neutral color
    // regardless of which side is live.
    void setEditingRight(bool editingRight);
    // The level trace's own caption -- Contour renames it to "Trigger
    // Level..." since its trigger can be the main signal, not only the
    // external Sidechain input.
    void setLevelTitle(const char* t) { levelTitle_ = t; repaint(); }

    // Fired continuously while dragging the corresponding handle -- the
    // owning UI forwards these straight to setParameterValue(), same
    // "widget fires a callback, UI owns the actual Parameter round trip"
    // pattern as CompressorGraphWidget's own setOnThresholdChanged() etc.
    void setOnHighpassChanged(std::function<void(float)> cb) { onHighpassChanged_ = std::move(cb); }
    void setOnLowpassChanged(std::function<void(float)> cb) { onLowpassChanged_ = std::move(cb); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;

private:
    std::string levelTitle_ = "Sidechain Level (L up / R down)";
    void drawSpectrum(float x, float y, float w, float h);
    void drawLevelTrace(float x, float y, float w, float h);
    float freqToX(double hz) const;
    double xToFreq(float px) const;
    // -1 = none, 0 = highpass handle, 1 = lowpass handle.
    int handleAt(double px, double py) const;

    float spectrumInDb_[sscomp::SpectrumAnalyzer::kNumBands];
    float spectrumOutDb_[sscomp::SpectrumAnalyzer::kNumBands];

    static constexpr int kHistoryLen = 360; // a few minutes at typical UI-tick rates -- plenty for "set up the filter, watch it react"
    float historyL_[kHistoryLen] = {};
    float historyR_[kHistoryLen] = {};
    int historyPos_ = 0;

    bool filterEnabled_ = false;
    float filterHighpassHz_ = 100.0f, filterLowpassHz_ = 8000.0f;
    bool editingRight_ = false;

    int hoverHandle_ = -1;
    int dragHandle_ = -1;
    std::function<void(float)> onHighpassChanged_;
    std::function<void(float)> onLowpassChanged_;

    // Cached from the last onNanoDisplay() so onMouse()/onMotion() can
    // map pointer coordinates <-> Hz using the exact same mapping
    // drawSpectrum() just used -- same pattern as every other draggable
    // graph in this project family (e.g. CompressorGraphWidget's own
    // curveX_/curveY_/curveW_/curveH_).
    float specX_ = 0, specY_ = 0, specW_ = 0, specH_ = 0;
    static constexpr float kHandleHitPx = 10.0f;

    static constexpr double kFMin = 20.0, kFMax = 20000.0;
    static constexpr float kSpectrumFloorDb = -80.0f, kSpectrumCeilDb = 0.0f;
    // The level trace's own dB grid values -- purely for the axis labels
    // (see drawLevelTrace()'s own comment: the envelope itself plots
    // linear amplitude, clamped to kLevelMaxLin, not this range directly).
    static constexpr float kLevelGridMinDb = -60.0f, kLevelGridMaxDb = 12.0f;
    static constexpr float kLevelMaxLin = 4.0f; // dbToLinear(+12dB) -- the clamp ceiling, matching kLevelGridMaxDb

    DISTRHO_DECLARE_NON_COPYABLE(SidechainScopeWidget)
};

END_NAMESPACE_DGL
