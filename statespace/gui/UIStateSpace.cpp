// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIStateSpace.hpp"

#include "../plugin/DistrhoPluginInfo.h"
#include "../plugin/PluginStateSpace.hpp" // for ParameterIndex enum
#include "../plugin/dsp/Serialization.hpp"
#include "../plugin/dsp/TransferFunction.hpp"
#include "FileBrowserDialog.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::BodePlotWidget;
using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::MatrixGridWidget;
using DGL_NAMESPACE::NonlinearPanelWidget;
using DGL_NAMESPACE::SimpleButton;

namespace {
constexpr int kDebounceIdleTicks = 3; // ~ a few uiIdle() ticks of hysteresis before recomputing the Bode plot
}

UIStateSpace::UIStateSpace()
    : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
{
    // Scale with the host's or desktop's scale factor (Ardour's GUI
    // scaling, else Xft.dpi; DPF_SCALE_FACTOR overrides): DPF draws
    // everything scaled and zooms when the window is resized, keeping the
    // aspect ratio; the layout always sees the default (logical) size. The
    // zoom is relative to the design size (setAutoScaleBaseSize(), DPF
    // patch dpf-patches/0002), so the window can also be made smaller than
    // it, down to half (the contents then shrink with it). Not
    // UI(w, h, true): DPF already creates the window at the scaled size,
    // and that form resizes it by the scale factor a second time.
    // Requires every widget to share this editor's NanoVG context (their
    // constructors take a NanoTopLevelWidget*) -- a widget with its own
    // context is drawn into an unscaled viewport.
    getWindow().setAutoScaleBaseSize(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT);
    setGeometryConstraints(DISTRHO_UI_DEFAULT_WIDTH / 2, DISTRHO_UI_DEFAULT_HEIGHT / 2, true, true, false);
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    model_.setIdentityPassthrough();
    lastReport_ = ssfilter::StabilityGuard::analyze(model_);

    gridA_ = new MatrixGridWidget(this, this, ssgui::MatrixKind::A, "A  (state -> state)");
    gridB_ = new MatrixGridWidget(this, this, ssgui::MatrixKind::B, "B  (input -> state)");
    gridC_ = new MatrixGridWidget(this, this, ssgui::MatrixKind::C, "C  (state -> output)");
    gridD_ = new MatrixGridWidget(this, this, ssgui::MatrixKind::D, "D  (input -> output, direct)");
    bode_  = new BodePlotWidget(this);
    bode_->setOnVisibilityChanged([this] { setState("bodeVisible", bode_->getVisibilityCode().c_str()); });

    nlPanel_ = new NonlinearPanelWidget(this, this);
    nlPanel_->setOnApplyCurve([this] { applyCompressorCurve(); });

    btnLoad_ = new SimpleButton(this, "Load...");
    btnLoad_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Load;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.title = "Load State-Space Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnSave_ = new SimpleButton(this, "Save As...");
    btnSave_->setCallback([this] {
        pendingFileOp_ = PendingFileOp::Save;
        DGL_NAMESPACE::FileBrowserOptions opts;
        opts.saving = true;
        opts.defaultName = "preset.ssjson";
        opts.title = "Save State-Space Preset";
        if (!lastPresetDir_.empty()) opts.startDir = lastPresetDir_.c_str();
        getWindow().openFileBrowser(opts);
    });

    btnPassthrough_ = new SimpleButton(this, "Passthrough");
    btnPassthrough_->setCallback([this] { model_.setIdentityPassthrough(); applyModelToEngine(); });

    btnSilence_ = new SimpleButton(this, "Silence");
    btnSilence_->setCallback([this] { model_.setSilence(); applyModelToEngine(); });

    btnReset_ = new SimpleButton(this, "Reset State");
    btnReset_->setCallback([this] { setParameterValue(kParamResetTrigger, 1.0f); });

    btnNormalize_ = new SimpleButton(this, "Normalize 0dB");
    btnNormalize_->setCallback([this] {
        const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
        ssfilter::TransferFunction::normalizeToUnityPassbandGain(model_, sr);
        applyModelToEngine();
    });

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] {
        bypassed_ = !bypassed_;
        setParameterValue(kParamBypass, bypassed_ ? 1.0f : 0.0f);
        btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                               bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                               Color(230, 232, 236));
        repaint();
    });

    btnOrderMinus_ = new SimpleButton(this, "- Order");
    btnOrderMinus_->setCallback([this] { changeOrder(-1); });

    btnOrderPlus_ = new SimpleButton(this, "+ Order");
    btnOrderPlus_->setCallback([this] { changeOrder(+1); });

    btnAbA_ = new SimpleButton(this, "A");
    btnAbA_->setCallback([this] { onAbA(); });
    btnAbB_ = new SimpleButton(this, "B");
    btnAbB_->setCallback([this] { onAbB(); });
    btnAbCopy_ = new SimpleButton(this, "Copy");
    btnAbCopy_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnAbCopy_->setCallback([this] { onAbCopy(); });
    refreshAbColors();

    btnUndo_ = new SimpleButton(this, "Undo");
    btnUndo_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnUndo_->setCallback([this] { onUndo(); });
    btnRedo_ = new SimpleButton(this, "Redo");
    btnRedo_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRedo_->setCallback([this] { onRedo(); });
    refreshUndoRedoColors();

    layout();
    refreshFromModel();
}

void UIStateSpace::layout()
{
    // Row 1: every system-wide/whole-instance control (file ops -> Reset/
    // Bypass -> A/B compare -> Undo/Redo), flush against the window's own
    // top edge, compare/history right-aligned. Row 2 is this filter's own
    // direct matrix-editing actions (Passthrough/Silence/Normalize/Order)
    // -- they act on model_ itself, not the plugin instance as a whole --
    // sitting right above the grids/Bode plot they affect.
    const float w = float(getWidth());
    const float h = float(getHeight());
    constexpr float pad = 6.0f;
    constexpr float toolbarH = 34.0f;
    constexpr float toolbar2H = 26.0f;

    float bx = pad;
    const float by = pad;
    const float bh = toolbarH - 2.0f * pad;
    auto place = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx), int(by));
        b->setSize(uint(bw), uint(bh));
        bx += bw + 4.0f;
    };
    place(btnLoad_, 78);
    place(btnSave_, 82);
    place(btnReset_, 90);
    place(btnBypass_, 72);

    float rx = w - pad;
    auto placeRight = [&](SimpleButton* b, float bw) {
        rx -= bw;
        b->setAbsolutePos(int(rx), int(by));
        b->setSize(uint(bw), uint(bh));
        rx -= 4.0f;
    };
    placeRight(btnRedo_, 56.0f);
    placeRight(btnUndo_, 56.0f);
    placeRight(btnAbCopy_, 56.0f);
    placeRight(btnAbB_, 30.0f);
    placeRight(btnAbA_, 30.0f);

    // Row 2: this filter's own direct matrix-editing actions.
    const float row2Y = toolbarH + pad;
    float bx2 = pad;
    auto place2 = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx2), int(row2Y));
        b->setSize(uint(bw), uint(toolbar2H));
        bx2 += bw + 4.0f;
    };
    place2(btnPassthrough_, 98);
    place2(btnSilence_, 72);
    place2(btnNormalize_, 98);
    place2(btnOrderMinus_, 62);
    bx2 += 46.0f; // room for the "Order: N" label, drawn directly in onNanoDisplay
    place2(btnOrderPlus_, 62);

    constexpr float nlPanelH = 168.0f;
    const float contentTop = toolbarH + pad + toolbar2H + pad * 2.0f;
    const float contentH = h - contentTop - pad - nlPanelH - pad;
    const float leftW = w * 0.56f;
    const float rightX = pad + leftW + pad;
    const float rightW = w - rightX - pad;

    const float col1W = leftW * 0.76f;
    const float col2W = leftW - col1W - pad;
    const float row1H = contentH * 0.72f;
    const float row2H = contentH - row1H - pad;

    gridA_->setAbsolutePos(int(pad), int(contentTop));
    gridA_->setSize(uint(col1W), uint(row1H));

    gridB_->setAbsolutePos(int(pad + col1W + pad), int(contentTop));
    gridB_->setSize(uint(col2W), uint(row1H));

    gridC_->setAbsolutePos(int(pad), int(contentTop + row1H + pad));
    gridC_->setSize(uint(col1W), uint(row2H));

    gridD_->setAbsolutePos(int(pad + col1W + pad), int(contentTop + row1H + pad));
    gridD_->setSize(uint(col2W), uint(row2H));

    bode_->setAbsolutePos(int(rightX), int(contentTop));
    bode_->setSize(uint(rightW), uint(contentH));

    const float nlPanelTop = contentTop + contentH + pad;
    nlPanel_->setAbsolutePos(int(pad), int(nlPanelTop));
    nlPanel_->setSize(uint(w - 2.0f * pad), uint(nlPanelH));
}

void UIStateSpace::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIStateSpace::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(18, 19, 23));
    fill();

    // "Order: N" label sitting between the order +/- buttons.
    char buf[32];
    std::snprintf(buf, sizeof(buf), "Order: %d", model_.order);
    fontSize(13.0f);
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    fillColor(Color(220, 222, 228));
    text(btnOrderMinus_->getAbsoluteX() + btnOrderMinus_->getWidth() + 25.0f,
         btnOrderMinus_->getAbsoluteY() + btnOrderMinus_->getHeight() * 0.5f, buf, nullptr);

    // Stability badge -- kept only on this plugin (see the sibling
    // plugins' own onNanoDisplay() for the removal, e.g. gui/eq/UIEq.cpp):
    // with the matrices directly editable here, rho is an actual
    // debugging aid, not just informational the way it was on every
    // coefficient-mapper plugin built on top of this core. Shows the
    // actual pole-radius estimate (5 decimal places -- narrow/high-Q
    // filters routinely sit at 0.999-0.99995, which "%.3f" would
    // misleadingly round to "1.000"), not the much looser exact norm
    // bound (see StabilityGuard.cpp).
    //
    // Row 2 (the filter's own Passthrough/Silence/Normalize/Order
    // actions), not row 1's top edge -- row 1's own top-right corner is
    // now Redo/Undo/Copy/B/A (see layout()'s own comment), and drawing
    // text there would just be the exact "behind the buttons" problem
    // this moved away from. Row 2's own buttons are left-aligned, so its
    // right side is free. pad/toolbarH/toolbar2H are layout()'s own
    // constexpr constants, duplicated here (not stored as members) since
    // they're fixed values, not something derived from the window size --
    // must stay equal to layout()'s own copies.
    constexpr float pad = 6.0f;
    constexpr float toolbarH = 34.0f;
    constexpr float toolbar2H = 26.0f;
    const float row2CenterY = toolbarH + pad + toolbar2H * 0.5f;
    char badge[160];
    if (lastReport_.wasClamped)
        std::snprintf(badge, sizeof(badge), "STABILITY: clamped x%.4g  (rho~%.5f)",
                      lastReport_.appliedScale, lastReport_.estimatedRadius);
    else
        std::snprintf(badge, sizeof(badge), "Stability OK  (rho~%.5f)",
                      lastReport_.estimatedRadius);
    fontSize(13.0f);
    textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
    fillColor(lastReport_.wasClamped ? Color(235, 175, 60) : Color(120, 210, 140));
    text(w - 10.0f, row2CenterY, badge, nullptr);

    if (faultBlinkTicks_ > 0)
    {
        // Right-aligned to end well clear of the badge's own left edge
        // (that text is at most ~230px wide at this font size) rather
        // than stacking a second line -- row 2 isn't tall enough for two.
        fontSize(12.0f);
        textAlign(ALIGN_RIGHT | ALIGN_MIDDLE);
        fillColor(Color(230, 90, 90));
        text(w - 250.0f, row2CenterY, "Numerical fault auto-recovered - state was reset", nullptr);
    }
}

void UIStateSpace::uiIdle()
{
    if (plotDirty_)
    {
        if (++idleTicksSinceEdit_ >= kDebounceIdleTicks)
        {
            const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
            bode_->recompute(model_, sr);
            plotDirty_ = false;
            idleTicksSinceEdit_ = 0;
        }
    }
    if (faultBlinkTicks_ > 0)
    {
        --faultBlinkTicks_;
        repaint();
    }

    // Undo/redo checkpointing -- a longer, independent debounce window
    // than plotDirty_'s own (that one's about not re-running an
    // expensive O(order^3) Bode sweep every edit; this one's about not
    // splitting one edit gesture into a dozen undo-able steps). See
    // ../dyneq/UIDynEq.cpp's identical block for the full reasoning.
    if (!applyingSnapshot_)
    {
        const std::string nowJson = ssfilter::Serialization::toJson(model_);
        if (nowJson != lastSeenJson_)
        {
            lastSeenJson_ = nowJson;
            undoIdleTicksSinceChange_ = 0;
        }
        else if (undoIdleTicksSinceChange_ < kUndoDebounceTicks && ++undoIdleTicksSinceChange_ >= kUndoDebounceTicks)
        {
            if (undoRedoInitialized_ && nowJson != lastCommittedJson_)
            {
                undoStack_.push_back(lastCommittedJson_);
                if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
                redoStack_.clear();
                refreshUndoRedoColors();
            }
            lastCommittedJson_ = nowJson;
            undoRedoInitialized_ = true;
        }
    }
}

void UIStateSpace::parameterChanged(uint32_t index, float value)
{
    switch (index)
    {
    case kParamBypass:
        bypassed_ = value >= 0.5f;
        btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                               bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                               Color(230, 232, 236));
        repaint();
        break;
    case kParamFaultFlag:
        if (value >= 0.5f) { faultBlinkTicks_ = 90; repaint(); }
        break;
    default:
        break;
    }
}

void UIStateSpace::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "presetDir") == 0) { lastPresetDir_ = value; return; }
    if (std::strcmp(key, "bodeVisible") == 0) { bode_->setVisibilityCode(value); return; }
    if (std::strcmp(key, "model") != 0) return;

    ssfilter::StateSpaceMatrices parsed;
    if (!ssfilter::Serialization::fromJson(value, parsed)) return;

    model_ = parsed;
    refreshFromModel();
}

void UIStateSpace::refreshFromModel()
{
    lastReport_ = ssfilter::StabilityGuard::analyze(model_);
    plotDirty_ = true;
    idleTicksSinceEdit_ = 0;
    repaint();
}

void UIStateSpace::applyModelToEngine()
{
    lastReport_ = ssfilter::StabilityGuard::enforce(model_);
    plotDirty_ = true;
    idleTicksSinceEdit_ = 0;
    setState("model", ssfilter::Serialization::toJson(model_).c_str());
    repaint();
}

// ---- ssgui::IModelEditorHost -------------------------------------------

double UIStateSpace::getMatrixValue(ssgui::MatrixKind kind, int row, int col) const
{
    switch (kind)
    {
    case ssgui::MatrixKind::A: return model_.A[size_t(row)][size_t(col)];
    case ssgui::MatrixKind::B: return model_.B[size_t(row)][size_t(col)];
    case ssgui::MatrixKind::C: return model_.C[size_t(row)][size_t(col)];
    case ssgui::MatrixKind::D: return model_.D[size_t(row)][size_t(col)];
    case ssgui::MatrixKind::Nonlinear:
        // See ModelEditorHost.hpp's MatrixKind::Nonlinear comment: cols 0/1
        // are real, persisted model fields; cols 2-4 are transient
        // convenience fields with no representation in StateSpaceMatrices,
        // only baked into the curve on "Apply Curve".
        switch (col)
        {
        case 0: return model_.nonlinear.attackMs;
        case 1: return model_.nonlinear.releaseMs;
        case 2: return compThresholdDb_;
        case 3: return compRatio_;
        case 4: return compMakeupDb_;
        }
        return 0.0;
    }
    return 0.0;
}

void UIStateSpace::setMatrixValue(ssgui::MatrixKind kind, int row, int col, double value)
{
    if (!std::isfinite(value)) return;
    switch (kind)
    {
    case ssgui::MatrixKind::A: model_.A[size_t(row)][size_t(col)] = value; break;
    case ssgui::MatrixKind::B: model_.B[size_t(row)][size_t(col)] = value; break;
    case ssgui::MatrixKind::C: model_.C[size_t(row)][size_t(col)] = value; break;
    case ssgui::MatrixKind::D: model_.D[size_t(row)][size_t(col)] = value; break;
    case ssgui::MatrixKind::Nonlinear:
        switch (col)
        {
        case 0: model_.nonlinear.attackMs = std::max(0.05, value); break;
        case 1: model_.nonlinear.releaseMs = std::max(1.0, value); break;
        case 2: compThresholdDb_ = value; break;
        case 3: compRatio_ = std::max(0.1, value); break;
        case 4: compMakeupDb_ = value; break;
        }
        break;
    }
    applyModelToEngine();
}

void UIStateSpace::setNonlinearEnabled(bool enabled)
{
    model_.nonlinear.enabled = enabled;
    applyModelToEngine();
}

double UIStateSpace::getNonlinearKnotGainDb(int knot) const
{
    if (knot < 0 || knot >= ssfilter::GainCurve::kNumKnots) return 0.0;
    return model_.nonlinear.curve.gainDb[knot];
}

void UIStateSpace::setNonlinearKnotGainDb(int knot, double gainDb)
{
    if (knot < 0 || knot >= ssfilter::GainCurve::kNumKnots || !std::isfinite(gainDb)) return;
    model_.nonlinear.curve.gainDb[knot] = gainDb;
    applyModelToEngine();
}

void UIStateSpace::applyCompressorCurve()
{
    // The classic feed-forward compressor formula (identical to the one
    // dsp_regression's test [8] checks against): flat (transparent, plus
    // makeup) below threshold, sloping down above it by 1/ratio instead of
    // 1/1 -- baked into all 25 knots at once, then individually
    // fine-tunable by dragging on the graph afterward.
    for (int i = 0; i < ssfilter::GainCurve::kNumKnots; ++i)
    {
        const double levelDb = ssfilter::GainCurve::knotLevelDb(i);
        const double gainDb = levelDb <= compThresholdDb_
                                   ? compMakeupDb_
                                   : compMakeupDb_ + (levelDb - compThresholdDb_) * (1.0 / compRatio_ - 1.0);
        model_.nonlinear.curve.gainDb[i] = gainDb;
    }
    applyModelToEngine();
}

void UIStateSpace::beginCellTextEdit(ssgui::MatrixKind kind, int row, int col, bool clearValue)
{
    // Defensive: if a different cell was already being edited, commit it
    // first rather than silently discard it (normally onMouse() below
    // already does this before the click ever reaches here).
    if (editingActive_ && (kind != editKind_ || row != editRow_ || col != editCol_))
        commitEdit();

    editingActive_ = true;
    editKind_ = kind;
    editRow_ = row;
    editCol_ = col;

    if (clearValue)
    {
        // Double-click: start from a blank buffer so typing replaces the
        // value outright instead of appending to the existing digits.
        editBuffer_.clear();
    }
    else
    {
        // std::to_chars, unlike snprintf's "%.6g", always emits '.' as the
        // decimal point regardless of the host's current numeric locale --
        // otherwise a locale that uses ',' would pre-fill this buffer with
        // a comma that commitEdit()'s locale-independent parser (also
        // fixed for the same reason) would then stop at, silently losing
        // everything after it if the field is committed unedited.
        char buf[32];
        const auto res = std::to_chars(buf, buf + sizeof(buf), getMatrixValue(kind, row, col),
                                        std::chars_format::general, 6);
        editBuffer_.assign(buf, res.ptr);
    }

    // Embedded plugin UIs (e.g. inside a host like Ardour) are not
    // guaranteed to already hold keyboard input focus just because they
    // were clicked -- grab it explicitly so the Enter/Escape/typed digits
    // actually reach this window instead of the host.
    getWindow().focus();

    repaint();
}

bool UIStateSpace::isEditingCell(ssgui::MatrixKind kind, int row, int col, std::string& liveText) const
{
    if (!editingActive_ || kind != editKind_ || row != editRow_ || col != editCol_) return false;
    liveText = editBuffer_;
    return true;
}

void UIStateSpace::commitEdit()
{
    if (!editingActive_) return;
    // std::from_chars, unlike std::strtod, is locale-independent -- always
    // '.' as the decimal point no matter what LC_NUMERIC the host process
    // is running under (see Json.hpp's parseNumber for the full story;
    // this is the same class of bug, hit here via direct matrix-cell
    // text editing instead of a preset/session load).
    double v = 0.0;
    const auto res = std::from_chars(editBuffer_.data(), editBuffer_.data() + editBuffer_.size(), v);
    const bool valid = res.ec == std::errc() && std::isfinite(v);
    editingActive_ = false;
    if (valid) setMatrixValue(editKind_, editRow_, editCol_, v);
    else repaint();
}

void UIStateSpace::cancelEdit()
{
    editingActive_ = false;
    repaint();
}

void UIStateSpace::moveEditToNeighbor(int rowDelta, int colDelta)
{
    if (!editingActive_) return;

    const ssgui::MatrixKind kind = editKind_;
    int rows = 0, cols = 0;
    ssgui::computeMatrixDims(kind, model_.order, rows, cols);

    const int newRow = editRow_ + rowDelta;
    const int newCol = editCol_ + colDelta;
    if (newRow < 0 || newRow >= rows || newCol < 0 || newCol >= cols)
        return; // at the edge of this matrix -- stay put rather than wrap/cross matrices

    commitEdit(); // apply the value being edited, same as Enter would
    beginCellTextEdit(kind, newRow, newCol, false);
}

bool UIStateSpace::onCharacterInput(const CharacterInputEvent& ev)
{
    if (!editingActive_) return false;
    const char c = ev.string[0];

    // Some platforms report Enter as a text/character event ('\r' or '\n')
    // rather than (only) a keyboard event -- treat it the same as the
    // kKeyEnter case in onKeyboard() below so committing the value doesn't
    // depend on which path the host/backend happens to deliver it through.
    if (c == '\r' || c == '\n') { commitEdit(); return true; }

    if ((c >= '0' && c <= '9') || c == '.' || c == '-' || c == 'e' || c == 'E' || c == '+')
    {
        if (editBuffer_.size() < 24) editBuffer_ += c;
        repaint();
        return true;
    }
    return true; // still swallow so it never leaks into a host shortcut while editing
}

bool UIStateSpace::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z/Ctrl+Y -- see ../compressor's UICompressorMono::onKeyboard()
    // for the full reasoning.
    if (ev.press && (ev.mod & DGL_NAMESPACE::kModifierControl) != 0
        && !(ev.mod & DGL_NAMESPACE::kModifierAlt) && !(ev.mod & DGL_NAMESPACE::kModifierSuper))
    {
        const bool shift = (ev.mod & DGL_NAMESPACE::kModifierShift) != 0;
        if (ev.key == 'z' && !shift) { onUndo(); return true; }
        if (ev.key == 'y' || (ev.key == 'z' && shift)) { onRedo(); return true; }
    }

    if (!editingActive_ || !ev.press) return false;

    switch (ev.key)
    {
    case DGL_NAMESPACE::kKeyEnter:
    case DGL_NAMESPACE::kKeyTab:
        commitEdit();
        return true;
    case DGL_NAMESPACE::kKeyEscape:
        cancelEdit();
        return true;
    case DGL_NAMESPACE::kKeyBackspace:
    case DGL_NAMESPACE::kKeyDelete:
        if (!editBuffer_.empty()) editBuffer_.pop_back();
        repaint();
        return true;
    case DGL_NAMESPACE::kKeyLeft:
        moveEditToNeighbor(0, -1);
        return true;
    case DGL_NAMESPACE::kKeyRight:
        moveEditToNeighbor(0, +1);
        return true;
    case DGL_NAMESPACE::kKeyUp:
        moveEditToNeighbor(-1, 0);
        return true;
    case DGL_NAMESPACE::kKeyDown:
        moveEditToNeighbor(+1, 0);
        return true;
    default:
        return true; // swallow everything else while a cell is being edited
    }
}

bool UIStateSpace::onMouse(const MouseEvent& rawEv)
{
    const MouseEvent ev = DGL_NAMESPACE::editorLogicalEvent(*this, rawEv); // own hit tests in logical units, see EditorSubWidget.hpp
    // Commit any pending cell edit as soon as the user clicks anywhere else
    // (a different cell, a button, empty space), spreadsheet-style. This
    // does not depend on the Enter key actually reaching this window,
    // which -- for a plugin UI embedded inside a host -- is not always a
    // safe assumption (window/input-focus handling varies by host and
    // embedding method). The click itself still proceeds normally
    // afterward via the base class, which dispatches it to whichever
    // subwidget it actually landed on.
    if (editingActive_ && ev.press)
        commitEdit();

    return UI::onMouse(rawEv);
}

void UIStateSpace::changeOrder(int delta)
{
    const int newOrder = ssfilter::clampOrder(model_.order + delta);
    if (newOrder == model_.order) return;
    model_.order = newOrder;
    applyModelToEngine();
}

void UIStateSpace::uiFileBrowserSelected(const char* filename)
{
    const PendingFileOp op = pendingFileOp_;
    pendingFileOp_ = PendingFileOp::None;
    if (filename == nullptr) return;

    if (op == PendingFileOp::Load) loadPresetFile(filename);
    else if (op == PendingFileOp::Save) savePresetFile(filename);
}

void UIStateSpace::loadPresetFile(const char* path)
{
    std::ifstream f(path, std::ios::binary);
    if (!f.good()) return;
    std::ostringstream ss;
    ss << f.rdbuf();

    applyModelSnapshot(ss.str());
    rememberPresetDir(path);
}

// -- A/B compare + undo/redo -------------------------------------------
// See ../compressor's UICompressorMono.cpp for the full design writeup
// this is ported from (against this plugin's own JSON model, same idiom
// as every other band-list-model plugin in this project).

void UIStateSpace::applyModelSnapshot(const std::string& json)
{
    // Formerly loadPresetFile()'s own body past the file read.
    ssfilter::StateSpaceMatrices parsed;
    if (!ssfilter::Serialization::fromJson(json, parsed)) return;
    model_ = parsed;
    applyModelToEngine();
}

void UIStateSpace::onAbA()
{
    if (abActive_ == AbSlot::A) return;
    abSlotB_ = ssfilter::Serialization::toJson(model_); abHasB_ = true;
    abActive_ = AbSlot::A;
    applyingSnapshot_ = true;
    if (abHasA_) applyModelSnapshot(abSlotA_);
    else { abSlotA_ = abSlotB_; abHasA_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
}

void UIStateSpace::onAbB()
{
    if (abActive_ == AbSlot::B) return;
    abSlotA_ = ssfilter::Serialization::toJson(model_); abHasA_ = true;
    abActive_ = AbSlot::B;
    applyingSnapshot_ = true;
    if (abHasB_) applyModelSnapshot(abSlotB_);
    else { abSlotB_ = abSlotA_; abHasB_ = true; }
    applyingSnapshot_ = false;
    refreshAbColors();
}

void UIStateSpace::onAbCopy()
{
    const std::string snap = ssfilter::Serialization::toJson(model_);
    if (abActive_ == AbSlot::A) { abSlotB_ = snap; abHasB_ = true; }
    else { abSlotA_ = snap; abHasA_ = true; }
}

void UIStateSpace::refreshAbColors()
{
    const bool onA = abActive_ == AbSlot::A;
    btnAbA_->setColors(onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
    btnAbB_->setColors(!onA ? Color(70, 130, 160) : Color(58, 62, 70),
                        !onA ? Color(90, 150, 180) : Color(78, 84, 96),
                        Color(230, 232, 236));
}

void UIStateSpace::onUndo()
{
    if (undoStack_.empty()) return;
    redoStack_.push_back(lastCommittedJson_);
    if (redoStack_.size() > kMaxUndoDepth) redoStack_.erase(redoStack_.begin());
    const std::string prev = undoStack_.back();
    undoStack_.pop_back();
    applyingSnapshot_ = true;
    applyModelSnapshot(prev);
    applyingSnapshot_ = false;
    lastCommittedJson_ = prev;
    lastSeenJson_ = prev;
    undoIdleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIStateSpace::onRedo()
{
    if (redoStack_.empty()) return;
    undoStack_.push_back(lastCommittedJson_);
    if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
    const std::string next = redoStack_.back();
    redoStack_.pop_back();
    applyingSnapshot_ = true;
    applyModelSnapshot(next);
    applyingSnapshot_ = false;
    lastCommittedJson_ = next;
    lastSeenJson_ = next;
    undoIdleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIStateSpace::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIStateSpace::savePresetFile(const char* path)
{
    std::string p(path);
    static const char kExt[] = ".ssjson";
    if (p.size() < sizeof(kExt) - 1 ||
        p.compare(p.size() - (sizeof(kExt) - 1), sizeof(kExt) - 1, kExt) != 0)
        p += kExt;

    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    if (!f.good()) return;
    f << ssfilter::Serialization::toJson(model_);
    rememberPresetDir(p.c_str());
}

void UIStateSpace::rememberPresetDir(const char* filePath)
{
    const std::string p(filePath);
    const size_t slash = p.find_last_of("/\\");
    if (slash == std::string::npos) return;
    const std::string dir = p.substr(0, slash);
    if (dir == lastPresetDir_) return;

    lastPresetDir_ = dir;
    // Persisted through the host's normal state-recall mechanism (see
    // PluginStateSpace::initState()'s "presetDir" entry) so the next time
    // this plugin instance's Load/Save dialog opens -- even after the GUI
    // was closed and reopened, or the project was saved and reloaded -- it
    // starts back in the same folder.
    setState("presetDir", lastPresetDir_.c_str());
}

void UIStateSpace::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIStateSpace(); }

END_NAMESPACE_DISTRHO
