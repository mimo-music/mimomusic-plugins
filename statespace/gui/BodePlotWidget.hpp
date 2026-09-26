// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "../plugin/dsp/TransferFunction.hpp"
#include "NanoVG.hpp"

#include <functional>
#include <string>

START_NAMESPACE_DGL

// Magnitude/phase Bode plot of the full 2x2 MIMO transfer function. Shows
// all four input/output channel pairs' magnitude (the two direct paths
// solid, the two cross-channel paths thinner/dimmer) plus the two direct
// paths' phase. Recomputed on demand (recompute()), not per frame -- the
// O(order^3)-per-point sweep is too costly to run at 60fps for larger
// matrix orders.
//
// Click a legend swatch to show/hide that channel pair's curve -- with all
// four sharing one y-axis, a loud cross-channel term can otherwise swamp
// the scale and flatten out the curves you actually care about. The
// magnitude axis autoscales to whichever curves are currently visible. The
// two direct paths (L->L, R->R) are visible by default, the two
// cross-channel paths (L->R, R->L) hidden -- most models have no
// cross-coupling at all, so starting with them hidden avoids two flat 0 dB
// lines cluttering the plot until they're actually relevant. The current
// visibility is reported via getVisibilityCode()/restored via
// setVisibilityCode() so the owner can persist it (see UIStateSpace's
// "bodeVisible" state) and setOnVisibilityChanged() fires whenever the user
// changes it by clicking a legend swatch.
//
// The frequency axis can be shown log- or linear-scaled; click the
// "LOG"/"LIN" label in the top-right corner to switch (re-sweeps against
// the last model passed to recompute(), which is kept around for exactly
// this so the caller doesn't need to resupply it).
class BodePlotWidget : public NanoSubWidget
{
public:
    explicit BodePlotWidget(Widget* parent);

    void recompute(const ssfilter::StateSpaceMatrices& model, double sampleRate);
    void setFault(bool active) { faultActive_ = active; repaint(); }

    // 4-character '0'/'1' string, order LL,RL,LR,RR (matching the legend's
    // reading order). Used to persist/restore which curves are shown.
    std::string getVisibilityCode() const;
    void setVisibilityCode(const std::string& code);

    void setOnVisibilityChanged(std::function<void()> cb) { onVisibilityChanged_ = std::move(cb); }

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;

private:
    void resweep();
    void recomputeAxisScale(); // from curves_, restricted to visible[][]

    std::vector<ssfilter::TransferFunction::SweepPoint> curves_[2][2]; // curves_[outIdx][inIdx]
    bool visible_[2][2] = { { true, false }, { false, true } }; // LL, RR on; LR, RL off
    std::function<void()> onVisibilityChanged_;
    ssfilter::StateSpaceMatrices lastModel_;
    double sampleRate_ = 44100.0;
    bool hasData_ = false;
    bool faultActive_ = false;
    bool logScale_ = true;

    // Magnitude axis autoscale: recomputed from the currently *visible*
    // curve data (with headroom, rounded to a clean multiple of 6 dB) so a
    // gentle EQ band and a hard filter both fill the plot usefully instead
    // of sharing one fixed +-Ndb range, and hiding a loud curve lets the
    // remaining ones rescale to fill the plot.
    double dbAxisMin_ = -60.0;
    double dbAxisMax_ = 24.0;

    // Hit region for the LOG/LIN toggle, updated each draw so onMouse can
    // test against wherever it was actually drawn this frame.
    float toggleX_ = 0.0f, toggleY_ = 0.0f, toggleW_ = 0.0f, toggleH_ = 0.0f;

    // Hit regions for the 4 legend swatches (indexed same as curves_),
    // likewise updated each draw.
    float legendX_[2][2] = {}, legendY_[2][2] = {}, legendW_[2][2] = {}, legendH_[2][2] = {};
};

END_NAMESPACE_DGL
