// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "StereoMeterWidget.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

START_NAMESPACE_DGL

StereoMeterWidget::StereoMeterWidget(Widget* parent)
    : NanoSubWidget(parent)
{
    loadSharedResources();
}

void StereoMeterWidget::setCorrelation(float corr, float minHold)
{
    correlation_ = corr;
    correlationMinHold_ = minHold;
    repaint();
}

void StereoMeterWidget::setWidthPercent(float widthPercent)
{
    widthPercent_ = widthPercent;
    repaint();
}

void StereoMeterWidget::drawCorrelationBar(float x, float y, float w, float h)
{
    fillColor(Color(190, 194, 200));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(11.5f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x, y, "Correlation", nullptr);

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%+.2f", double(correlation_));
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w, y, buf, nullptr);

    const float barY = y + 16.0f;
    const float barH = 16.0f;

    beginPath();
    rect(x, barY, w, barH);
    fillColor(Color(16, 17, 20));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    const float cx = x + w * 0.5f;
    const auto valueToX = [&](float v) { return x + (v + 1.0f) * 0.5f * w; };

    // Fill from center (0) out to the current value -- red toward -1,
    // green toward +1, matching the goniometer's own dot-color direction.
    const float vx = valueToX(std::max(-1.0f, std::min(1.0f, correlation_)));
    const bool positive = correlation_ >= 0.0f;
    beginPath();
    rect(std::min(cx, vx), barY + 1.0f, std::fabs(vx - cx), barH - 2.0f);
    fillColor(positive ? Color(90, 200, 120) : Color(230, 70, 70));
    fill();

    // Center (0) tick.
    beginPath();
    moveTo(cx, barY);
    lineTo(cx, barY + barH);
    strokeColor(Color(120, 124, 132));
    strokeWidth(1.0f);
    stroke();

    // Worst-case (min) hold marker -- a thin white line that snaps left on
    // a phase problem and slowly recovers back toward the live value.
    const float hx = valueToX(std::max(-1.0f, std::min(1.0f, correlationMinHold_)));
    beginPath();
    moveTo(hx, barY - 2.0f);
    lineTo(hx, barY + barH + 2.0f);
    strokeColor(Color(235, 238, 242));
    strokeWidth(1.5f);
    stroke();

    fillColor(Color(140, 144, 152));
    fontSize(9.0f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x, barY + barH + 2.0f, "-1", nullptr);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(cx, barY + barH + 2.0f, "0", nullptr);
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w, barY + barH + 2.0f, "+1", nullptr);
}

void StereoMeterWidget::drawWidthBar(float x, float y, float w, float h)
{
    (void)h;
    fillColor(Color(190, 194, 200));
    fontFace(NANOVG_DEJAVU_SANS_TTF);
    fontSize(11.5f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x, y, "Stereo Width", nullptr);

    char buf[16];
    std::snprintf(buf, sizeof(buf), "%.0f%%", double(widthPercent_));
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w, y, buf, nullptr);

    const float barY = y + 16.0f;
    const float barH = 16.0f;

    beginPath();
    rect(x, barY, w, barH);
    fillColor(Color(16, 17, 20));
    fill();
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();

    const float frac = std::max(0.0f, std::min(1.0f, widthPercent_ / 200.0f));
    beginPath();
    rect(x + 1.0f, barY + 1.0f, frac * (w - 2.0f), barH - 2.0f);
    fillColor(Color(90, 160, 220));
    fill();

    // 100% (an ordinary, uncorrelated stereo signal) reference tick.
    const float refX = x + 0.5f * w;
    beginPath();
    moveTo(refX, barY);
    lineTo(refX, barY + barH);
    strokeColor(Color(120, 124, 132));
    strokeWidth(1.0f);
    stroke();

    fillColor(Color(140, 144, 152));
    fontSize(9.0f);
    textAlign(ALIGN_LEFT | ALIGN_TOP);
    text(x, barY + barH + 2.0f, "0%", nullptr);
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(refX, barY + barH + 2.0f, "100%", nullptr);
    textAlign(ALIGN_RIGHT | ALIGN_TOP);
    text(x + w, barY + barH + 2.0f, "200%", nullptr);
}

void StereoMeterWidget::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(22, 24, 28));
    fill();

    constexpr float pad = 8.0f;
    constexpr float rowH = 46.0f;
    drawCorrelationBar(pad, pad, w - 2.0f * pad, rowH);
    drawWidthBar(pad, pad + rowH + 10.0f, w - 2.0f * pad, rowH);

    beginPath();
    rect(0.5f, 0.5f, w - 1.0f, h - 1.0f);
    strokeColor(Color(44, 47, 54));
    strokeWidth(1.0f);
    stroke();
}

END_NAMESPACE_DGL
