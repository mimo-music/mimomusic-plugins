// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// StereoMeterWidget.hpp
//
// Two stacked horizontal bargraphs: phase Correlation (-1..+1, with a
// slowly-recovering worst-case min-hold marker) and Stereo Width
// (0..200%, 100% marked as the "ordinary uncorrelated stereo" reference).
// Both numbers come from ssmeter::StereoImageMeter -- see that class's own
// comment for why one shared set of leaky-integrated sums drives both.

#pragma once

#include "NanoVG.hpp"

START_NAMESPACE_DGL

class StereoMeterWidget : public NanoSubWidget
{
public:
    explicit StereoMeterWidget(Widget* parent);

    void setCorrelation(float corr, float minHold);
    void setWidthPercent(float widthPercent);

protected:
    void onNanoDisplay() override;

private:
    void drawCorrelationBar(float x, float y, float w, float h);
    void drawWidthBar(float x, float y, float w, float h);

    float correlation_ = 0.0f;
    float correlationMinHold_ = 1.0f;
    float widthPercent_ = 0.0f;
};

END_NAMESPACE_DGL
