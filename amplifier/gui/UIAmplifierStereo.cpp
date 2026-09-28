// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIAmplifierStereo.hpp"

#include "../plugin_stereo/DistrhoPluginInfo.h"
#include "../plugin_stereo/PluginAmplifierStereo.hpp"

#include <algorithm>
#include <cstdio>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;

UIAmplifierStereo::UIAmplifierStereo()
    : UI(DISTRHO_UI_DEFAULT_WIDTH, DISTRHO_UI_DEFAULT_HEIGHT)
{
    loadSharedResources();
    fontFace(NANOVG_DEJAVU_SANS_TTF);

    paramCache_[kParamGain] = 0.0f;
    paramCache_[kParamInvertL] = 0.0f;
    paramCache_[kParamInvertR] = 0.0f;
    paramCache_[kParamBalance] = 0.0f;
    paramCache_[kParamBypass] = 0.0f;
    paramCache_[kParamListenActive] = 0.0f;
    paramCache_[kParamMeasureRms] = 0.0f;
    paramCache_[kParamNormalizeTarget] = -1.0f;
    paramCache_[kParamMeterCurrentLevel] = -120.0f;
    paramCache_[kParamClipping] = 0.0f;

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] { setParameterValue(kParamBypass, bypassed_ ? 0.0f : 1.0f); });
    btnBypass_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));

    // See this file's own top comment / UIAmplifierMono.cpp's
    // applyListenResult() comment for why turning Listen off is handled
    // here, not in PluginAmplifierStereo.cpp.
    btnListen_ = new SimpleButton(this, "Listen");
    btnListen_->setCallback([this] {
        const bool wasListening = listening_;
        listening_ = !listening_;
        if (wasListening && !listening_) applyListenResult(); // stopping now -- compute+push before telling the host
        setParameterValue(kParamListenActive, listening_ ? 1.0f : 0.0f);
        refreshListenColors();
        repaint();
    });
    refreshListenColors();

    btnMode_ = new SimpleButton(this, "Peak");
    btnMode_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnMode_->setCallback([this] {
        setParameterValue(kParamMeasureRms, mode_ == ssamp::ListenMode::Peak ? 1.0f : 0.0f);
    });

    // Two independent toggles, not a swap button -- both can be on at once.
    btnInvertL_ = new SimpleButton(this, "Invert L");
    btnInvertL_->setCallback([this] { setParameterValue(kParamInvertL, invertL_ ? 0.0f : 1.0f); });
    btnInvertR_ = new SimpleButton(this, "Invert R");
    btnInvertR_->setCallback([this] { setParameterValue(kParamInvertR, invertR_ ? 0.0f : 1.0f); });
    refreshInvertColors();

    // Gain last -- the actual output gain this plugin applies, so it sits
    // on the right side of the strip, matching the convention every
    // sibling plugin's own output-facing control now follows.
    strip_ = new ParamStripWidget(this);
    strip_->setFields({
        { "Balance", "", -1.0f, 1.0f, 0.0f, 0.05f, false, 2, kParamBalance },
        { "Normalize To", "dB", -24.0f, 0.0f, -1.0f, 0.5f, false, 2, kParamNormalizeTarget },
        { "Gain", "dB", -60.0f, 24.0f, 0.0f, 0.5f, false, 2, kParamGain },
    });
    strip_->setGetter([this](uint32_t idx) { return paramCache_[idx]; });
    strip_->setSetter([this](uint32_t idx, float v) {
        paramCache_[idx] = v;
        setParameterValue(idx, v);
    });

    btnUndo_ = new SimpleButton(this, "Undo");
    btnUndo_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnUndo_->setCallback([this] { onUndo(); });
    btnRedo_ = new SimpleButton(this, "Redo");
    btnRedo_->setColors(Color(58, 62, 70), Color(78, 84, 96), Color(230, 232, 236));
    btnRedo_->setCallback([this] { onRedo(); });
    refreshUndoRedoColors();

    layout();
}

void UIAmplifierStereo::layout()
{
    // Row 1: system-wide controls -- Bypass, then Undo/Redo (no A/B here,
    // no Load/Save exists yet) -- flush against the window's own top
    // edge, Undo/Redo right-aligned. Row 2 is this filter's own
    // Listen/Peak-Rms/Invert controls (the actual gain-measurement and
    // polarity features), left-aligned right above the strip/meter they
    // drive.
    const float w = float(getWidth());
    const float h = float(getHeight());
    constexpr float pad = 8.0f;
    constexpr float rowH = 28.0f;
    constexpr float stripH = 46.0f;

    float bx = pad, by = pad;
    auto place = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx), int(by));
        b->setSize(uint(bw), uint(rowH - 4.0f));
        bx += bw + 6.0f;
    };
    place(btnBypass_, 74.0f);

    float rx = w - pad;
    auto placeRight = [&](SimpleButton* b, float bw) {
        rx -= bw;
        b->setAbsolutePos(int(rx), int(by));
        b->setSize(uint(bw), uint(rowH - 4.0f));
        rx -= 6.0f;
    };
    placeRight(btnRedo_, 56.0f);
    placeRight(btnUndo_, 56.0f);

    by += rowH + pad;
    bx = pad;
    place(btnListen_, 74.0f);
    place(btnMode_, 60.0f);
    place(btnInvertL_, 74.0f);
    place(btnInvertR_, 74.0f);

    by += rowH + pad;
    strip_->setAbsolutePos(int(pad), int(by));
    strip_->setSize(uint(std::max(0.0f, w - 2.0f * pad)), uint(stripH));

    by += stripH + pad;
    meterX_ = pad;
    meterY_ = by;
    meterW_ = w - 2.0f * pad;
    meterH_ = std::max(0.0f, h - by - pad);
}

void UIAmplifierStereo::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIAmplifierStereo::refreshListenColors()
{
    btnListen_->setColors(listening_ ? Color(70, 150, 90) : Color(58, 62, 70),
                           listening_ ? Color(85, 175, 110) : Color(78, 84, 96),
                           Color(230, 232, 236));
}

void UIAmplifierStereo::refreshModeLabel()
{
    btnMode_->setLabel(mode_ == ssamp::ListenMode::Rms ? "RMS" : "Peak");
}

// See UIAmplifierMono.cpp's own comment on its identical method -- same
// mechanism, same reasoning, same double-apply guard.
void UIAmplifierStereo::applyListenResult()
{
    if (paramCache_[kParamMeterCurrentLevel] <= -119.0f) return;
    const double measuredDb = double(paramCache_[kParamMeterCurrentLevel]);
    const double targetDb = double(paramCache_[kParamNormalizeTarget]);
    const float newGainDb = float(std::max(-60.0, std::min(24.0, targetDb - measuredDb)));
    paramCache_[kParamGain] = newGainDb;
    setParameterValue(kParamGain, newGainDb);
    strip_->repaint();
}

void UIAmplifierStereo::refreshInvertColors()
{
    btnInvertL_->setColors(invertL_ ? Color(150, 90, 40) : Color(58, 62, 70),
                            invertL_ ? Color(175, 110, 55) : Color(78, 84, 96),
                            Color(230, 232, 236));
    btnInvertR_->setColors(invertR_ ? Color(150, 90, 40) : Color(58, 62, 70),
                            invertR_ ? Color(175, 110, 55) : Color(78, 84, 96),
                            Color(230, 232, 236));
}

// -- Undo/redo ------------------------------------------------------------
// See UIAmplifierMono.cpp's identical block / ../../compressor's
// UICompressorMono.cpp for the full design writeup this is ported from.

presetio::KeyValues UIAmplifierStereo::captureSnapshot() const
{
    presetio::KeyValues values;
    values["gainDb"] = double(paramCache_[kParamGain]);
    values["balance"] = double(paramCache_[kParamBalance]);
    values["normalizeTargetDb"] = double(paramCache_[kParamNormalizeTarget]);
    values["bypass"] = double(paramCache_[kParamBypass]);
    values["measureRms"] = double(paramCache_[kParamMeasureRms]);
    values["invertL"] = double(paramCache_[kParamInvertL]);
    values["invertR"] = double(paramCache_[kParamInvertR]);
    return values;
}

void UIAmplifierStereo::applySnapshot(const presetio::KeyValues& values)
{
    auto apply = [&](const char* key, uint32_t idx) {
        const auto it = values.find(key);
        if (it == values.end()) return;
        const float v = float(it->second);
        paramCache_[idx] = v;
        setParameterValue(idx, v);
    };
    apply("gainDb", kParamGain);
    apply("balance", kParamBalance);
    apply("normalizeTargetDb", kParamNormalizeTarget);
    apply("bypass", kParamBypass);
    apply("measureRms", kParamMeasureRms);
    apply("invertL", kParamInvertL);
    apply("invertR", kParamInvertR);

    bypassed_ = paramCache_[kParamBypass] >= 0.5f;
    btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                           bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                           Color(230, 232, 236));
    mode_ = ssamp::clampListenMode(paramCache_[kParamMeasureRms] >= 0.5f ? 1 : 0);
    refreshModeLabel();
    invertL_ = paramCache_[kParamInvertL] >= 0.5f;
    invertR_ = paramCache_[kParamInvertR] >= 0.5f;
    refreshInvertColors();
    strip_->repaint();
    repaint();
}

void UIAmplifierStereo::onUndo()
{
    if (undoStack_.empty()) return;
    redoStack_.push_back(lastCommittedSnapshot_);
    if (redoStack_.size() > kMaxUndoDepth) redoStack_.erase(redoStack_.begin());
    const presetio::KeyValues prev = undoStack_.back();
    undoStack_.pop_back();
    applyingSnapshot_ = true;
    applySnapshot(prev);
    applyingSnapshot_ = false;
    lastCommittedSnapshot_ = prev;
    lastSeenSnapshot_ = prev;
    idleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIAmplifierStereo::onRedo()
{
    if (redoStack_.empty()) return;
    undoStack_.push_back(lastCommittedSnapshot_);
    if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
    const presetio::KeyValues next = redoStack_.back();
    redoStack_.pop_back();
    applyingSnapshot_ = true;
    applySnapshot(next);
    applyingSnapshot_ = false;
    lastCommittedSnapshot_ = next;
    lastSeenSnapshot_ = next;
    idleTicksSinceChange_ = kUndoDebounceTicks;
    refreshUndoRedoColors();
}

void UIAmplifierStereo::refreshUndoRedoColors()
{
    btnUndo_->setEnabled(!undoStack_.empty());
    btnRedo_->setEnabled(!redoStack_.empty());
}

void UIAmplifierStereo::onNanoDisplay()
{
    beginPath();
    rect(0, 0, float(getWidth()), float(getHeight()));
    fillColor(Color(18, 19, 23));
    fill();

    if (meterH_ <= 4.0f) return;

    // Clipping -- see ssamp::ClipHold's own comment (AmplifierDsp.hpp) for
    // why this is a separate, always-live flag rather than folded into
    // the Listen/Normalize reading below: it reflects the ACTUAL signal
    // leaving this plugin right now (whatever Gain/Balance/Invert/Bypass/
    // Listen happen to be doing), not a frozen one-shot measurement, and
    // it's the one that was reported missing entirely ("If the signal is
    // to strong he is not showing the clipping"). Drawn regardless of
    // listening_, so a hot signal is flagged whether or not a Listen
    // session happens to be running at the same time.
    const bool clipping = paramCache_[kParamClipping] >= 0.5f;

    beginPath();
    roundedRect(meterX_, meterY_, meterW_, meterH_, 4.0f);
    fillColor(Color(15, 16, 19));
    fill();
    strokeColor(clipping ? Color(220, 70, 60) : (listening_ ? Color(90, 170, 110) : Color(48, 51, 58)));
    strokeWidth(clipping ? 2.0f : (listening_ ? 1.6f : 1.0f));
    stroke();

    fontSize(11.0f);
    fillColor(Color(150, 154, 162));
    textAlign(ALIGN_CENTER | ALIGN_TOP);
    text(meterX_ + meterW_ * 0.5f, meterY_ + 8.0f,
         listening_ ? "Listening -- measuring the raw input" : "Measured Level (last audition)", nullptr);

    char buf[32];
    const bool hasReading = paramCache_[kParamMeterCurrentLevel] > -119.0f;
    if (hasReading) std::snprintf(buf, sizeof(buf), "%.1f dB", paramCache_[kParamMeterCurrentLevel]);
    else std::snprintf(buf, sizeof(buf), "--");

    fontSize(30.0f);
    fillColor(listening_ ? Color(235, 190, 120) : Color(200, 204, 210));
    textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
    text(meterX_ + meterW_ * 0.5f, meterY_ + meterH_ * 0.5f + 6.0f, buf, nullptr);

    if (clipping)
    {
        constexpr float kBadgeW = 46.0f, kBadgeH = 16.0f, kBadgeMargin = 6.0f;
        const float bx = meterX_ + meterW_ - kBadgeW - kBadgeMargin;
        const float by = meterY_ + kBadgeMargin;
        beginPath();
        roundedRect(bx, by, kBadgeW, kBadgeH, 3.0f);
        fillColor(Color(210, 60, 50));
        fill();
        fontSize(10.5f);
        fillColor(Color(255, 240, 238));
        textAlign(ALIGN_CENTER | ALIGN_MIDDLE);
        text(bx + kBadgeW * 0.5f, by + kBadgeH * 0.5f + 1.0f, "CLIP", nullptr);
    }
}

void UIAmplifierStereo::uiIdle()
{
    if (listening_) repaint();

    // Undo/redo checkpointing -- see UIAmplifierMono.cpp's identical
    // block for the full reasoning.
    if (!applyingSnapshot_)
    {
        const presetio::KeyValues now = captureSnapshot();
        if (now != lastSeenSnapshot_)
        {
            lastSeenSnapshot_ = now;
            idleTicksSinceChange_ = 0;
        }
        else if (idleTicksSinceChange_ < kUndoDebounceTicks && ++idleTicksSinceChange_ >= kUndoDebounceTicks)
        {
            if (undoRedoInitialized_ && now != lastCommittedSnapshot_)
            {
                undoStack_.push_back(lastCommittedSnapshot_);
                if (undoStack_.size() > kMaxUndoDepth) undoStack_.erase(undoStack_.begin());
                redoStack_.clear();
                refreshUndoRedoColors();
            }
            lastCommittedSnapshot_ = now;
            undoRedoInitialized_ = true;
        }
    }
}

void UIAmplifierStereo::parameterChanged(uint32_t index, float value)
{
    if (index >= kParamCount) return;
    paramCache_[index] = value;

    switch (index)
    {
    case kParamBypass:
        bypassed_ = value >= 0.5f;
        btnBypass_->setColors(bypassed_ ? Color(150, 90, 40) : Color(58, 62, 70),
                               bypassed_ ? Color(175, 110, 55) : Color(78, 84, 96),
                               Color(230, 232, 236));
        repaint();
        break;
    case kParamInvertL:
        invertL_ = value >= 0.5f;
        refreshInvertColors();
        repaint();
        break;
    case kParamInvertR:
        invertR_ = value >= 0.5f;
        refreshInvertColors();
        repaint();
        break;
    case kParamListenActive:
    {
        const bool nowListening = value >= 0.5f;
        if (listening_ && !nowListening) applyListenResult(); // host-driven stop (e.g. automation) -- the button callback didn't already handle this one
        listening_ = nowListening;
        refreshListenColors();
        repaint();
        break;
    }
    case kParamMeasureRms:
        mode_ = ssamp::clampListenMode(value >= 0.5f ? 1 : 0);
        refreshModeLabel();
        break;
    case kParamMeterCurrentLevel:
        repaint();
        break;
    case kParamClipping:
        repaint(); // see onNanoDisplay()'s own clip-indicator comment
        break;
    default:
        break;
    }
}

bool UIAmplifierStereo::onCharacterInput(const CharacterInputEvent& ev)
{
    if (!strip_->isEditing()) return false;
    const char c = ev.string[0];
    if (c == '\r' || c == '\n') { strip_->commitEdit(); return true; }
    if ((c >= '0' && c <= '9') || c == '.' || c == '-') { strip_->appendChar(c); return true; }
    return true;
}

bool UIAmplifierStereo::onKeyboard(const KeyboardEvent& ev)
{
    // Ctrl+Z/Ctrl+Y -- see ../../compressor's UICompressorMono::onKeyboard()
    // for the full reasoning.
    if (ev.press && (ev.mod & DGL_NAMESPACE::kModifierControl) != 0
        && !(ev.mod & DGL_NAMESPACE::kModifierAlt) && !(ev.mod & DGL_NAMESPACE::kModifierSuper))
    {
        const bool shift = (ev.mod & DGL_NAMESPACE::kModifierShift) != 0;
        if (ev.key == 'z' && !shift) { onUndo(); return true; }
        if (ev.key == 'y' || (ev.key == 'z' && shift)) { onRedo(); return true; }
    }

    if (!strip_->isEditing() || !ev.press) return false;
    switch (ev.key)
    {
    case DGL_NAMESPACE::kKeyEnter:
    case DGL_NAMESPACE::kKeyTab:
        strip_->commitEdit();
        return true;
    case DGL_NAMESPACE::kKeyEscape:
        strip_->cancelEdit();
        return true;
    case DGL_NAMESPACE::kKeyBackspace:
    case DGL_NAMESPACE::kKeyDelete:
        strip_->backspace();
        return true;
    default:
        return true;
    }
}

UI* createUI() { return new UIAmplifierStereo(); }

END_NAMESPACE_DISTRHO
