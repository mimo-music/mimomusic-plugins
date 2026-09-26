// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "TooltipOverlay.hpp"

#include <algorithm>
#include <vector>

START_NAMESPACE_DGL

TooltipOverlay::TooltipOverlay(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
}

void TooltipOverlay::show(const std::string& text, float anchorX, float anchorY, float anchorW, float anchorH)
{
    if (text_ == text && anchorX_ == anchorX && anchorY_ == anchorY) return; // avoid a repaint every idle tick for an unchanged tooltip
    text_ = text;
    anchorX_ = anchorX;
    anchorY_ = anchorY;
    anchorW_ = anchorW;
    anchorH_ = anchorH;
    repaint();
}

void TooltipOverlay::hide()
{
    if (text_.empty()) return;
    text_.clear();
    repaint();
}

void TooltipOverlay::onNanoDisplay()
{
    if (text_.empty()) return;

    constexpr float kPadX = 8.0f;
    constexpr float kPadY = 6.0f;
    constexpr float kFontSize = 12.0f;
    constexpr float kMaxWidth = 240.0f; // wrap long descriptions rather than running off-window

    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(kFontSize);
    textAlign(ALIGN_LEFT | ALIGN_TOP);

    // Word-wrap into lines that fit kMaxWidth, measuring as we go --
    // simpler than nanovg's own textBreakLines for a handful of short
    // sentences, and avoids a second binding surface just for this.
    std::vector<std::string> lines;
    std::string current;
    size_t pos = 0;
    while (pos <= text_.size())
    {
        const size_t space = text_.find(' ', pos);
        const std::string word = text_.substr(pos, space == std::string::npos ? std::string::npos : space - pos);
        const std::string candidate = current.empty() ? word : current + ' ' + word;
        Rectangle<float> bounds;
        textBounds(0.0f, 0.0f, candidate.c_str(), nullptr, bounds);
        if (bounds.getWidth() > kMaxWidth && !current.empty())
        {
            lines.push_back(current);
            current = word;
        }
        else
        {
            current = candidate;
        }
        if (space == std::string::npos) break;
        pos = space + 1;
    }
    if (!current.empty()) lines.push_back(current);

    float textW = 0.0f;
    for (const auto& line : lines)
    {
        Rectangle<float> bounds;
        textBounds(0.0f, 0.0f, line.c_str(), nullptr, bounds);
        textW = std::max(textW, bounds.getWidth());
    }
    const float lineH = kFontSize * 1.35f;
    const float boxW = textW + 2.0f * kPadX;
    const float boxH = float(lines.size()) * lineH + 2.0f * kPadY;

    // Anchored just below the hovered control, clamped so the box never
    // runs past this overlay's own (i.e. the whole window's) edges.
    float boxX = anchorX_;
    float boxY = anchorY_ + anchorH_ + 4.0f;
    boxX = std::min(boxX, float(getWidth()) - boxW - 2.0f);
    boxX = std::max(2.0f, boxX);
    if (boxY + boxH > float(getHeight()) - 2.0f)
        boxY = anchorY_ - boxH - 4.0f; // flip above the control if there's no room below

    beginPath();
    roundedRect(boxX, boxY, boxW, boxH, 4.0f);
    fillColor(Color(20, 21, 25, 0.92f));
    fill();
    strokeColor(Color(90, 94, 102));
    strokeWidth(1.0f);
    stroke();

    fillColor(Color(224, 227, 232));
    for (size_t i = 0; i < lines.size(); ++i)
        text(boxX + kPadX, boxY + kPadY + float(i) * lineH, lines[i].c_str(), nullptr);
}

END_NAMESPACE_DGL
