// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "LevelMeterWidget.hpp"

#include <algorithm>
#include <cstdio>

START_NAMESPACE_DGL

LevelMeterWidget::LevelMeterWidget(Widget* parent, const char* title, int channelCount)
    : NanoSubWidget(parent), title_(title), channelCount_(channelCount)
{
    loadSharedResources();
}

void LevelMeterWidget::setChannel(int channel, float rmsDb, float peakDb)
{
    if (channel < 0 || channel > 1) return;
    rmsDb_[channel] = rmsDb;
    peakDb_[channel] = peakDb;
    if (peakDb >= -0.05f)
        clipUntil_[channel] = std::chrono::steady_clock::now() + std::chrono::milliseconds(1500);
    repaint();
}

float LevelMeterWidget::dbToFrac(float db) const
{
    const float t = (db - kMeterMinDb) / (kMeterMaxDb - kMeterMinDb);
    return std::max(0.0f, std::min(1.0f, t));
}

void LevelMeterWidget::drawBar(float x, float y, float w, float h, float rmsDb, float peakDb,
                                std::chrono::steady_clock::time_point& clipUntil,
                                const char* label)
{
    constexpr float ledH = 12.0f;
    const float barY = y + ledH + 3.0f;
    const float barH = h - ledH - 3.0f - 12.0f; // leave room for the label below

    // Clip LED.
    const bool clipping = std::chrono::steady_clock::now() < clipUntil;
    beginPath();
    roundedRect(x, y, w, ledH, 2.0f);
    fillColor(clipping ? Color(230, 60, 60) : Color(40, 30, 30));
    fill();
    strokeColor(Color(14, 15, 18));
    strokeWidth(1.0f);
    stroke();

    // Bar background.
    beginPath();
    rect(x, barY, w, barH);
    fillColor(Color(16, 17, 20));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    // RMS fill, drawn as a green->amber->red vertical gradient clipped to
    // the current level's height -- the color at the very top of the fill
    // is what the meter is "reading" right now, exactly like a hardware
    // LED-ladder meter.
    const float frac = dbToFrac(rmsDb);
    if (frac > 0.001f)
    {
        const float fillH = frac * barH;
        scissor(x, barY + barH - fillH, w, fillH);
        beginPath();
        rect(x, barY, w, barH);
        Paint grad = linearGradient(0, barY + barH, 0, barY, Color(90, 200, 120), Color(230, 70, 70));
        fillPaint(grad);
        fill();
        resetScissor();
    }

    // Peak-hold line.
    const float peakFrac = dbToFrac(peakDb);
    const float peakY = barY + barH - peakFrac * barH;
    beginPath();
    moveTo(x, peakY);
    lineTo(x + w, peakY);
    strokeColor(Color(235, 238, 242));
    strokeWidth(1.5f);
    stroke();

    // 0dB reference line.
    const float zeroY = barY + barH - dbToFrac(0.0f) * barH;
    beginPath();
    moveTo(x, zeroY);
    lineTo(x + w, zeroY);
    strokeColor(Color(120, 60, 60));
    strokeWidth(1.0f);
    stroke();

    fillColor(Color(140, 144, 152));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(10.0f);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(x + w * 0.5f, barY + barH + 2.0f, label, nullptr);
}

void LevelMeterWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(22, 24, 28));
    fill();

    fillColor(Color(190, 194, 200));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(11.5f);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(w * 0.5f, 2.0f, title_.c_str(), nullptr);

    constexpr float topPad = 16.0f;
    constexpr float sidePad = 6.0f;
    constexpr float gap = 6.0f;

    if (channelCount_ <= 1)
    {
        drawBar(sidePad, topPad, w - 2.0f * sidePad, h - topPad - 2.0f, rmsDb_[0], peakDb_[0], clipUntil_[0], labelA_.c_str());
    }
    else
    {
        const float barW = (w - 2.0f * sidePad - gap) * 0.5f;
        drawBar(sidePad, topPad, barW, h - topPad - 2.0f, rmsDb_[0], peakDb_[0], clipUntil_[0], labelA_.c_str());
        drawBar(sidePad + barW + gap, topPad, barW, h - topPad - 2.0f, rmsDb_[1], peakDb_[1], clipUntil_[1], labelB_.c_str());
    }
}

bool LevelMeterWidget::onMouse(const MouseEvent& ev)
{
    // Click anywhere on the meter clears both clip LEDs early -- a
    // standard "acknowledge the clip indicator" convention.
    if (ev.button != 1 || !ev.press) return false;
    clipUntil_[0] = clipUntil_[1] = std::chrono::steady_clock::now();
    repaint();
    return true;
}

END_NAMESPACE_DGL
