// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// LevelMeterWidget.hpp
//
// A vertical bargraph level meter -- 1 bar (mono) or 2 bars (stereo L/R,
// or Mid/Side when the stereo plugin is in that mode). Each bar shows a
// smoothed RMS-ish fill (green -> amber -> red, via a NanoVG gradient
// clipped to the current level) plus a thin peak-hold line, and a clip LED
// above the bar that latches red for a short, visible moment whenever that
// bar's peak goes over 0dBFS -- "a bargraph with the level to see if
// it's clipping". Values arrive from the DSP side's own PeakMeter/
// LoudnessMeter (see plugin/dsp/CompressorDsp.hpp) via DPF output
// Parameters, read once per uiIdle() tick by the owning UI and pushed in
// here with setChannel() -- this widget itself holds no audio-thread state.

#pragma once

#include "EditorSubWidget.hpp"

#include <chrono>
#include <string>

START_NAMESPACE_DGL

class LevelMeterWidget : public EditorSubWidget
{
public:
    LevelMeterWidget(NanoTopLevelWidget* parent, const char* title, int channelCount);

    // channel: 0 = mono/left/mid, 1 = right/side (only if channelCount==2).
    // rmsDb/peakDb: -120..+12 or so, whatever the meter's own floor is.
    void setChannel(int channel, float rmsDb, float peakDb);
    void setChannelLabels(const char* a, const char* b) { labelA_ = a; labelB_ = b; }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;

private:
    float dbToFrac(float db) const;
    void drawBar(float x, float y, float w, float h, float rmsDb, float peakDb, std::chrono::steady_clock::time_point& clipUntil, const char* label);

    std::string title_;
    int channelCount_;
    std::string labelA_ = "L", labelB_ = "R";

    float rmsDb_[2] = { -120.0f, -120.0f };
    float peakDb_[2] = { -120.0f, -120.0f };

    std::chrono::steady_clock::time_point clipUntil_[2]{};

    static constexpr float kMeterMinDb = -60.0f;
    static constexpr float kMeterMaxDb = 6.0f;
};

END_NAMESPACE_DGL
