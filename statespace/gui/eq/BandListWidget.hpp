// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// BandListWidget.hpp
//
// A compact, scrollable row-per-band list -- the precise-numeric-entry and
// type-selection counterpart to EqGraphWidget's direct-manipulation graph
// (which deliberately doesn't support either: there's no room on a small
// draggable node for a discoverable type selector or exact digit entry).
// Each row: a color swatch matching the node's color on the graph, the
// band number, a type-cycle button (click steps through EqBandType),
// Enabled/Solo toggles, editable Freq/Gain/Q fields (click-to-edit,
// double-click-to-clear, Enter/Tab/Escape/scroll-to-nudge -- same
// conventions as every other numeric field in this project), and a
// delete button. Selecting a row (clicking anywhere on it that isn't one
// of the above controls) highlights the corresponding node on the graph
// and vice versa, via UIEq's onSelectionChanged plumbing.
//
// Rows beyond the visible area scroll with the mouse wheel -- unlike the
// base plugin's matrix grids (which hide text below a size threshold
// rather than scroll, see MatrixGridWidget's "Known limitations" note),
// an EQ's band count is the entire point of this plugin being unbounded,
// so scrolling was worth the extra state here.

#pragma once

#include "EqEditorHost.hpp"
#include "../EditorSubWidget.hpp"

#include <chrono>
#include <functional>
#include <string>

START_NAMESPACE_DGL

class BandListWidget : public EditorSubWidget
{
public:
    // `stereo`: whether to show the Channel column at all (click to
    // cycle Both/Left/Right, or -- when `host_->supportsCrossFeed()` also
    // says yes -- Both/Left/Right/L->R/R->L, five states in the one
    // control, see layoutForRow()'s own comment) -- true for the stereo
    // Parametric EQ (UIEq), false for EQ Mono (UIEqMono): a single
    // channel has nothing to assign Left/Right of, and
    // IEqEditorHost::setBandChannel() is a no-op there, so showing a
    // control that would silently do nothing on click would be worse
    // than not showing it at all.
    BandListWidget(NanoTopLevelWidget* parent, ssgui::IEqEditorHost* host, bool stereo);

    void setSelectedBand(int index) { selectedBand_ = index; }
    int getSelectedBand() const { return selectedBand_; }
    void setOnSelectionChanged(std::function<void(int)> cb) { onSelectionChanged_ = std::move(cb); }

    // Hover descriptions for the columns -- returns which
    // column's own description applies at local (x,y) (any row -- column
    // meaning doesn't depend on which band it's for), or empty if the
    // pointer isn't over a real band row/column at all (the header, the
    // swatch, empty space below the list, ...). Used by
    // TooltipOverlay::addFieldTarget().
    std::string cellTooltipAt(double x, double y) const;

protected:
    void onNanoDisplay() override;
    bool onMouse(const MouseEvent& ev) override;
    bool onMotion(const MotionEvent& ev) override;
    bool onScroll(const ScrollEvent& ev) override;

private:
    struct RowLayout
    {
        float y = 0, h = 0;
        float swatchX = 0, swatchW = 0;
        float typeX = 0, typeW = 0;
        float enableX = 0, enableW = 0;
        float soloX = 0, soloW = 0;
        float channelX = 0, channelW = 0; // only meaningful when stereo_ -- see this file's own constructor comment; widened when host_->supportsCrossFeed() too, see layoutForRow()
        float freqX = 0, freqW = 0;
        float bwX = 0, bwW = 0; // Band Shelf's bandwidth (octaves); a "--" placeholder for every other type
        float gainX = 0, gainW = 0;
        float qX = 0, qW = 0;
        float deleteX = 0, deleteW = 0;
    };

    RowLayout layoutForRow(int visibleRow) const; // visibleRow 0 = topmost currently-drawn row
    int rowAtY(double y, int& bandIndex) const;   // -1 if none; also resolves bandIndex accounting for scroll
    int maxVisibleRows() const;

    ssgui::IEqEditorHost* host_;
    const bool stereo_; // see this file's own constructor comment
    std::function<void(int)> onSelectionChanged_;

    int selectedBand_ = -1;
    int hoverBand_ = -1;
    int scrollOffset_ = 0; // first visible band index

    // Which field (if any) is mid-press, resolved on release the same way
    // MatrixGridWidget/NonlinearPanelWidget do it.
    enum class PressTarget { None, Type, Enable, Solo, Channel, Freq, Bandwidth, Gain, Q, Delete, Row };
    PressTarget pressTarget_ = PressTarget::None;
    int pressBand_ = -1;

    // Click-and-drag on Freq/BW/Gain/Q -- same feel as ParamStripWidget
    // in the other plugins: past a 3px dead zone the press becomes a
    // vertical fader drag (up = increase), otherwise the release is an
    // ordinary click-to-type. dragStartValue_ is the field's value at the
    // press.
    bool dragging_ = false;
    double dragStartY_ = 0.0;
    double dragStartValue_ = 0.0;
    // True if `target` is a text-editable numeric field for `band`'s type
    // (not Gain's Slope control/placeholder or BW's placeholder).
    bool isValueField(PressTarget target, const ssfilter::EqBand& band) const;

    int lastClickField_ = -1; // band*4 + field index, for double-click-to-clear on numeric fields
    std::chrono::steady_clock::time_point lastClickTime_{};

    static constexpr float kRowH = 24.0f;

    // Column-title row drawn once above the bands, at fixed y in [0,
    // kHeaderH) -- every column's x position comes from layoutForRow(0)
    // (columns don't move with scrolling, only rows do), so this doesn't
    // need its own layout function. Added because the numeric Freq/Gain/Q
    // columns had no label of any kind before this -- unlike the Type/
    // Enable/Solo/Delete columns, which all show self-explanatory text
    // inside their own per-row control -- so at a glance there was no way
    // to tell which bare number was which. A title row rather than a hover
    // popup: always visible beats needing to find and hover each field
    // first.
    static constexpr float kHeaderH = 16.0f;
};

END_NAMESPACE_DGL
