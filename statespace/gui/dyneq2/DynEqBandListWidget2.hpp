// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DynEqBandListWidget2.hpp
//
// Dynamic EQ 2's band list: one row per band with every setting of that
// band, the counterpart of the Parametric EQ's BandListWidget (same look,
// same interaction) extended by the dynamics columns. Each row: a colour
// swatch matching the node on the graph, a type-cycle button (Bell, Low
// Shelf, High Shelf, Band Shelf), Enable / Solo / Dynamics toggles, the
// Channel cycle (Both, Left, Right, L->R, R->L), the numeric fields Freq,
// BW, Gain, Q, Threshold, Ratio, Up Ratio, Range, Attack and Release, and a
// delete button.
//
// Numeric fields: click to type, double-click to type from empty, drag up/
// down, scroll to nudge (Alt = 10x finer) -- the same conventions as every
// numeric field in the project. A field that doesn't apply to the band's
// type (BW outside Band Shelf, Q on the shelves) shows a dimmed "--"; the
// dynamics fields are drawn dimmed while the band's dynamics are off, but
// stay editable. Clicking a row anywhere else selects the band (synced
// with the graph through UIDynEq2). Rows beyond the visible area scroll.

#pragma once

#include "DynEqEditorHost2.hpp"
#include "../EditorSubWidget.hpp"

#include <chrono>
#include <functional>
#include <string>

START_NAMESPACE_DGL

class DynEqBandListWidget2 : public EditorSubWidget
{
public:
    DynEqBandListWidget2(NanoTopLevelWidget* parent, ssgui::IDynEqEditorHost2* host);

    void setSelectedBand(int index) { selectedBand_ = index; }
    int getSelectedBand() const { return selectedBand_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }

    // Hover description of the column at local (x,y), or empty -- used by
    // TooltipOverlay::addFieldTarget().
    std::string cellTooltipAt(double x, double y) const;

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    // Every column, left to right. The numeric ones map 1:1 onto
    // ssgui::DynEqField (see fieldOf()).
    enum Col { kColType, kColEnable, kColSolo, kColDynamics, kColChannel,
               kColFreq, kColBandwidth, kColGain, kColQ,
               kColThreshold, kColRatio, kColRatioUp, kColRange, kColAttack, kColRelease,
               kColDelete, kNumCols };

    struct ColumnLayout { float x[kNumCols]; float w[kNumCols]; float swatchX, swatchW; };
    ColumnLayout columns() const;

    int colAt(double x) const; // -1 if between/outside columns
    int rowAtY(double y, int& bandIndex) const; // -1 if none
    int maxVisibleRows() const;

    static bool isNumericCol(int col) { return col >= kColFreq && col <= kColRelease; }
    static ssgui::DynEqField fieldOf(int col);
    // Whether `col` applies to `band`'s type (a numeric column that
    // doesn't shows "--" and ignores clicks, drags and scrolling).
    static bool colApplies(int col, const ssfilter::DynamicEqBand2& band);
    static double valueOf(int col, const ssfilter::DynamicEqBand2& band);
    void setValue(int band, int col, double v);

    ssgui::IDynEqEditorHost2* host_;
    std::function<void(int)> onSelectionChanged_;

    int selectedBand_ = -1;
    int hoverBand_ = -1;
    int scrollOffset_ = 0; // first visible band index

    // Which column is mid-press, resolved on release (click) or in
    // onMotion() (drag past the dead zone, numeric columns only).
    int pressCol_ = -1; // -1 none, kNumCols = the row itself
    int pressBand_ = -1;
    bool dragging_ = false;
    double dragStartY_ = 0.0;
    double dragStartValue_ = 0.0;

    int lastClickField_ = -1; // band*kNumCols + col, for double-click-to-clear
    std::chrono::steady_clock::time_point lastClickTime_{};

    static constexpr float kRowH = 24.0f;
    static constexpr float kHeaderH = 16.0f; // column titles, drawn once above the rows
};

END_NAMESPACE_DGL
