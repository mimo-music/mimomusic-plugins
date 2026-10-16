// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#include "UIReverb.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

START_NAMESPACE_DISTRHO

using DGL_NAMESPACE::BandListWidget;
using DGL_NAMESPACE::Color;
using DGL_NAMESPACE::EqGraphWidget;
using DGL_NAMESPACE::ParamFieldSpec;
using DGL_NAMESPACE::ParamStripWidget;
using DGL_NAMESPACE::SimpleButton;

namespace {
const char* noteValueLabel(int index)
{
    switch (ssfilter::reverb::TempoNoteValue(std::clamp(index, 0, 9)))
    {
    case ssfilter::reverb::TempoNoteValue::Sixteenth:     return "1/16";
    case ssfilter::reverb::TempoNoteValue::Eighth:        return "1/8";
    case ssfilter::reverb::TempoNoteValue::DottedEighth:  return "1/8.";
    case ssfilter::reverb::TempoNoteValue::Quarter:       return "1/4";
    case ssfilter::reverb::TempoNoteValue::DottedQuarter: return "1/4.";
    case ssfilter::reverb::TempoNoteValue::Half:          return "1/2";
    case ssfilter::reverb::TempoNoteValue::DottedHalf:    return "1/2.";
    case ssfilter::reverb::TempoNoteValue::OneBar:        return "1 bar";
    case ssfilter::reverb::TempoNoteValue::TwoBars:       return "2 bars";
    case ssfilter::reverb::TempoNoteValue::FourBars:      return "4 bars";
    }
    return "?";
}
} // namespace

UIReverb::UIReverb()
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

    roomStrip_ = new ParamStripWidget(this);
    roomStrip_->setGetter([this](uint32_t id) { return roomGetter(id); });
    roomStrip_->setSetter([this](uint32_t id, float v) { roomSetter(id, v); });
    // Trailing `false, "", ""` on every field below: the bipolarLabel/
    // labelHigh/labelLow trio (ParamFieldSpec.hpp), explicit even though
    // false/empty are already each member's own default -- omitting them
    // in a brace-init list triggers -Wmissing-field-initializers the
    // moment any *other* field in this project (Dry/Wet, below) actually
    // needs bipolarLabel=true, since the struct is no longer "every field
    // list always fills every member" from the compiler's point of view.
    roomStrip_->setFields({
        {"Length", "m", 1.0f, 80.0f, 10.0f, 0.5f, false, 1, 0, "Room length, meters -- rebuilds the reverb tank's own delay lines (a brief, silent duck, not a click).", false, "", ""},
        {"Width", "m", 1.0f, 80.0f, 7.0f, 0.5f, false, 1, 1, "Room width, meters.", false, "", ""},
        {"Height", "m", 1.0f, 40.0f, 3.5f, 0.25f, false, 1, 2, "Room height, meters.", false, "", ""},
    });

    mainStrip_ = new ParamStripWidget(this);
    mainStrip_->setGetter([this](uint32_t id) { return paramGetter(id); });
    mainStrip_->setSetter([this](uint32_t id, float v) { paramSetter(id, v); });
    rebuildMainFields();

    btnBypass_ = new SimpleButton(this, "Bypass");
    btnBypass_->setCallback([this] { setParameterValue(kParamBypass, bypass_ ? 0.0f : 1.0f); bypass_ = !bypass_; repaint(); });

    btnReset_ = new SimpleButton(this, "Reset");
    btnReset_->setCallback([this] { setParameterValue(kParamResetTrigger, 1.0f); });

    btnTempoSync_ = new SimpleButton(this, "Tempo Sync: Off");
    btnTempoSync_->setCallback([this] {
        tempoSyncEnabled_ = !tempoSyncEnabled_;
        setParameterValue(kParamTempoSyncEnabled, tempoSyncEnabled_ ? 1.0f : 0.0f);
        btnTempoSync_->setLabel(tempoSyncEnabled_ ? "Tempo Sync: On" : "Tempo Sync: Off");
        rebuildMainFields();
        repaint();
    });

    btnFollowHost_ = new SimpleButton(this, "Follow Host Tempo: Off");
    btnFollowHost_->setCallback([this] {
        followHostTempo_ = !followHostTempo_;
        setParameterValue(kParamTempoSyncFollowHost, followHostTempo_ ? 1.0f : 0.0f);
        btnFollowHost_->setLabel(followHostTempo_ ? "Follow Host Tempo: On" : "Follow Host Tempo: Off");
        repaint();
    });

    char noteBuf[24];
    std::snprintf(noteBuf, sizeof(noteBuf), "Note: %s", noteValueLabel(tempoNoteValueIndex_));
    btnNoteValue_ = new SimpleButton(this, noteBuf);
    btnNoteValue_->setCallback([this] {
        tempoNoteValueIndex_ = (tempoNoteValueIndex_ + 1) % 10;
        setParameterValue(kParamTempoSyncNoteValue, float(tempoNoteValueIndex_));
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Note: %s", noteValueLabel(tempoNoteValueIndex_));
        btnNoteValue_->setLabel(buf);
        repaint();
    });

    for (float& v : spectrumInRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumInSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutRaw_) v = ssfilter::SpectrumAnalyzer::kFloorDb;
    for (float& v : spectrumOutSmoothed_) v = ssfilter::SpectrumAnalyzer::kFloorDb;

    graph_ = new EqGraphWidget(this, this);
    graph_->setSampleRate(getSampleRate());
    graph_->setSpectrumMode(ssfilter::SpectrumMode::Both); // pre-EQ (tank output) + post-EQ, see kParamUiHeartbeat's own comment
    graph_->setOnSelectionChanged([this](int i) { selectBand(i); });
    // Spectrum "Autoscale" persistence -- ported from the sibling
    // ../../statespace EQ plugins' own identical wiring (see e.g. that
    // project's UIEq.cpp); see kStateSpectrumAutoscale's own comment in
    // ReverbParams.hpp for why this doesn't touch any DSP parameter,
    // only the graph's own vertical-axis display range.
    graph_->setOnSpectrumAutoscaleChanged([this](bool on) { setState("spectrumAutoscale", on ? "1" : "0"); });

    list_ = new BandListWidget(this, this, true); // stereo -- shows the Channel column
    list_->setOnSelectionChanged([this](int i) { selectBand(i); });

    bandStrip_ = new ParamStripWidget(this);
    bandStrip_->setGetter([this](uint32_t id) -> float {
        if (selectedBand_ < 0 || selectedBand_ >= getBandCount()) return 0.0f;
        const ssfilter::EqBand b = getBand(selectedBand_);
        switch (id)
        {
        case 0: return float(b.freqHz);
        case 1: return float(b.gainDb);
        case 2: return float(b.q);
        }
        return 0.0f;
    });
    bandStrip_->setSetter([this](uint32_t id, float v) {
        if (selectedBand_ < 0 || selectedBand_ >= getBandCount()) return;
        switch (id)
        {
        case 0: setBandFreq(selectedBand_, v); break;
        case 1: setBandGain(selectedBand_, v); break;
        case 2: setBandQ(selectedBand_, v); break;
        }
        repaint();
    });

    tooltip_ = new TooltipOverlay(this); // LAST -- always-on-top, see its own file comment

    layout();
}

void UIReverb::rebuildMainFields()
{
    // Decay Time swaps between editable (manual mode) and a read-only
    // live readout (tempo-sync mode): while synced, the number shown here
    // *is* the live, tempo-derived value (kParamCurrentDecayTimeSec),
    // updating as the host's tempo changes, not a stale manual setting
    // that silently does nothing.
    std::vector<ParamFieldSpec> fields;
    fields.push_back({"Input Gain", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamInputGainDb,
                       "Gain applied before the signal splits into the dry path and the reverb tank's own input.", false, "", ""});
    fields.push_back({"Pre-Delay", "ms", 0.0f, 250.0f, 0.0f, 1.0f, false, 1, kParamPreDelayMs,
                       "Delays the reverb tank's own input only -- the dry signal stays immediate. 0 = the tank starts filling right away; raise it to let the direct sound arrive first before the reverb blooms.", false, "", ""});
    if (tempoSyncEnabled_)
    {
        ParamFieldSpec f{"Decay (synced)", "s", 0.0f, 20.0f, 1.8f, 0.0f, false, 2, kParamCurrentDecayTimeSec,
                          "Live decay time, derived from the host's tempo and the chosen note value. Read-only while Tempo Sync is on.", false, "", ""};
        f.nudgeStep = 0.0f; // effectively read-only: a 0 step means scroll/drag has no effect (see ParamStripWidget::applyDrag())
        fields.push_back(f);
    }
    else
    {
        fields.push_back({"Decay Time", "s", 0.1f, 20.0f, 1.8f, 0.1f, false, 2, kParamDecayTimeSec,
                           "Reverb tail length (T60). Ignored while Tempo Sync is on.", false, "", ""});
    }
    fields.push_back({"Damping", "", 0.0f, 1.0f, 0.5f, 0.02f, false, 2, kParamDamping,
                       "How much faster the tail's high end decays than its low end. 0 = uniform, 1 = strongly darkened.", false, "", ""});
    fields.push_back({"Diffusion", "", 0.0f, 1.0f, 0.7f, 0.02f, false, 2, kParamDiffusion,
                       "How strongly the tank's own input is pre-diffused (allpass) before entering it. Raise this if the tail sounds metallic/ringy; lower it for a more discrete, less smeared early character.", false, "", ""});
    fields.push_back({"Modulation", "", 0.0f, 1.0f, 0.3f, 0.02f, false, 2, kParamModulation,
                       "Slowly modulates each delay line's own length so fixed resonances can't build up -- the direct fix for uncontrolled/pitched ringing, especially at low Damping. 0 = off; too much can sound like its own vibrato.", false, "", ""});
    fields.push_back({"Spread", "", 0.0f, 2.0f, 1.0f, 0.02f, false, 2, kParamWidth,
                       "Stereo spread of the reverb tail only (never the dry signal) -- named 'Spread' here, not 'Width', to avoid confusion with Room Width (meters) just to the left. 0 = mono, 1 = the tank's own natural width, 2 = exaggerated.", false, "", ""});
    {
        // Same bipolar-label treatment as Dry/Wet just below -- "L"/"R"
        // instead of "Wet"/"Dry", fading toward whichever side the
        // reverb's own image currently leans.
        ParamFieldSpec f{"Balance", "", -1.0f, 1.0f, 0.0f, 0.02f, false, 2, kParamBalance,
                          "Corrects (or creatively shifts) any left/right lean in the reverb tail only. 0 = centered, unity gain both channels.",
                          true, "R", "L"};
        fields.push_back(f);
    }
    {
        // Ported from ../compressor's own kParamWetDry field -- 0..100%,
        // bipolar "Wet"/"Dry" label that fades
        // green/red toward whichever end the current value is closer to.
        ParamFieldSpec f{"Dry/Wet", "%", 0.0f, 100.0f, 35.0f, 1.0f, false, 0, kParamDryWet,
                          "0% = fully dry, 100% = fully wet.", true, "Wet", "Dry"};
        fields.push_back(f);
    }
    fields.push_back({"Output Trim", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, kParamOutputTrimDb,
                       "Applied to the wet (reverb) signal only, before the dry/wet mix.", false, "", ""});
    mainStrip_->setFields(std::move(fields));
}

float UIReverb::paramGetter(uint32_t fieldId) const
{
    switch (fieldId)
    {
    case kParamInputGainDb:         return float(inputGainDb_);
    case kParamPreDelayMs:          return float(preDelayMs_);
    case kParamDecayTimeSec:        return float(decayTimeSec_);
    case kParamCurrentDecayTimeSec: return float(currentDecayTimeSec_);
    case kParamDamping:             return float(damping_);
    case kParamDiffusion:           return float(diffusionAmount_);
    case kParamModulation:          return float(modulationAmount_);
    case kParamWidth:               return float(widthAmount_);
    case kParamBalance:             return float(balanceAmount_);
    case kParamDryWet:              return float(dryWet_);
    case kParamOutputTrimDb:        return float(outputTrimDb_);
    default: return 0.0f;
    }
}

void UIReverb::paramSetter(uint32_t fieldId, float value)
{
    switch (fieldId)
    {
    case kParamInputGainDb:  inputGainDb_ = value; break;
    case kParamPreDelayMs:   preDelayMs_ = value; break;
    case kParamDecayTimeSec: decayTimeSec_ = value; break;
    case kParamDamping:      damping_ = value; break;
    case kParamDiffusion:    diffusionAmount_ = value; break;
    case kParamModulation:   modulationAmount_ = value; break;
    case kParamWidth:        widthAmount_ = value; break;
    case kParamBalance:      balanceAmount_ = value; break;
    case kParamDryWet:       dryWet_ = value; break;
    case kParamOutputTrimDb: outputTrimDb_ = value; break;
    case kParamCurrentDecayTimeSec: return; // read-only field, see rebuildMainFields()
    default: return;
    }
    setParameterValue(fieldId, value);
}

float UIReverb::roomGetter(uint32_t fieldId) const
{
    switch (fieldId)
    {
    case 0: return float(roomLengthM_);
    case 1: return float(roomWidthM_);
    case 2: return float(roomHeightM_);
    default: return 0.0f;
    }
}

void UIReverb::roomSetter(uint32_t fieldId, float value)
{
    switch (fieldId)
    {
    case 0: roomLengthM_ = value; break;
    case 1: roomWidthM_ = value; break;
    case 2: roomHeightM_ = value; break;
    default: return;
    }
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", double(value));
    setState(fieldId == 0 ? "roomLengthM" : fieldId == 1 ? "roomWidthM" : "roomHeightM", buf);
}

void UIReverb::layout()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    tooltip_->setAbsolutePos(0, 0);
    tooltip_->setSize(uint(w), uint(h));

    constexpr float pad = 8.0f;
    constexpr float rowH = 28.0f;   // buttons -- a single line of text fits fine at this height
    // ParamStripWidget draws a value line AND a label line stacked inside
    // its own height (see that file's onNanoDisplay(): value at 40% of
    // fieldH_, label at 80%) -- 28px (rowH, meant for a single-line
    // button) left the two lines overlapping illegibly, confirmed
    // directly from a real screenshot of this exact bug. 46px matches the
    // sibling ../statespace project's own proven `stripH` for this same
    // widget (UIMbCompressor.cpp/UIDynEq.cpp), not a guessed fix.
    constexpr float stripH = 46.0f;

    float bx = pad, by = pad;
    auto placeBtn = [&](SimpleButton* b, float bw) {
        b->setAbsolutePos(int(bx), int(by));
        b->setSize(uint(bw), uint(rowH - 4.0f));
        bx += bw + 6.0f;
    };
    placeBtn(btnBypass_, 80);
    placeBtn(btnReset_, 70);
    placeBtn(btnTempoSync_, 120);
    placeBtn(btnFollowHost_, 160);
    placeBtn(btnNoteValue_, 90);

    diagTextX_ = bx + 10.0f;
    diagTextY_ = by + rowH * 0.5f;

    by += rowH + pad;
    roomStrip_->setAbsolutePos(int(pad), int(by));
    roomStrip_->setSize(uint(w * 0.35f), uint(stripH));

    mainStrip_->setAbsolutePos(int(pad + w * 0.35f + pad), int(by));
    mainStrip_->setSize(uint(w - (pad + w * 0.35f + pad) - pad), uint(stripH));

    by += stripH + pad * 2.0f;

    // Real bug, confirmed from a screenshot (the band list's right-hand
    // columns disappeared past the window border): BandListWidget's own
    // row layout (layoutForRow(), gui/eq/BandListWidget.cpp) needs
    // swatch+type+enable+solo+channel+freq+gain+q+delete plus the 4px gap
    // after each -- summing to ~378px of actual content -- but this
    // panel was only ever given 320px, so the Q and Delete columns (and
    // part of Gain) were drawn past this widget's own right edge and
    // never visible. 400px covers the real content width with a small
    // margin; DISTRHO_UI_DEFAULT_WIDTH was also raised (1000 -> 1100) so
    // reclaiming this space doesn't just shrink the graph by the same
    // amount.
    const float listW = 400.0f;
    graph_->setAbsolutePos(int(pad), int(by));
    graph_->setSize(uint(w - listW - pad * 3.0f), uint(h - by - pad));

    // The selected band's strip under the list: heading + fields.
    constexpr float bandHeaderH = 16.0f, bandStripH = 50.0f;
    const float bandPanelH = bandHeaderH + bandStripH + pad;
    list_->setAbsolutePos(int(w - listW - pad), int(by));
    list_->setSize(uint(listW), uint(std::max(20.0f, h - by - pad - bandPanelH)));
    bandStripHeaderX_ = w - listW - pad;
    bandStripHeaderY_ = h - pad - bandStripH - bandHeaderH;
    bandStrip_->setAbsolutePos(int(bandStripHeaderX_), int(bandStripHeaderY_ + bandHeaderH));
    bandStrip_->setSize(uint(listW), uint(bandStripH));
}

void UIReverb::onResize(const ResizeEvent& ev)
{
    UI::onResize(ev);
    layout();
}

void UIReverb::refreshBandStrip()
{
    const bool valid = selectedBand_ >= 0 && selectedBand_ < getBandCount();
    bandStrip_->setVisible(valid);
    if (!valid || bandStrip_->isEditing()) return;
    const ssfilter::EqBandType t = getBand(selectedBand_).type;
    const int key = selectedBand_ * 64 + int(t);
    if (key == bandStripKey_) return;
    bandStripKey_ = key;
    const bool hasGain = ssfilter::eqBandTypeHasGain(t);
    bandStrip_->setFields({
        { "Freq", "Hz", 20.0f, 20000.0f, 1000.0f, 1.05f, true, 0, 0,
          "Freq: the selected band's frequency (drag, double-click to type, scroll; Alt = finer)." },
        { hasGain ? "Gain" : "Gain (unused)", "dB", -24.0f, 24.0f, 0.0f, 0.5f, false, 1, 1,
          hasGain ? "Gain: the selected band's boost or cut." : "Gain: not used by this band type." },
        { "Q", "", 0.05f, 100.0f, 1.0f, 1.08f, true, 2, 2,
          "Q: how narrow (high) or wide (low) the selected band is." },
    });
}

void UIReverb::onNanoDisplay()
{
    const float w = float(getWidth());
    const float h = float(getHeight());

    beginPath();
    rect(0, 0, w, h);
    fillColor(Color(18, 19, 23));
    fill();

    // Selected band's heading above its strip.
    refreshBandStrip();
    fontSize(11.5f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    if (selectedBand_ >= 0 && selectedBand_ < getBandCount())
    {
        char head[64];
        std::snprintf(head, sizeof(head), "EQ band %d \xC2\xB7 %s", selectedBand_ + 1, ssfilter::eqBandTypeName(getBand(selectedBand_).type));
        fillColor(Color(200, 204, 212));
        text(bandStripHeaderX_ + 2.0f, bandStripHeaderY_ + 8.0f, head, nullptr);
    }
    else
    {
        fillColor(Color(120, 124, 132));
        text(bandStripHeaderX_ + 2.0f, bandStripHeaderY_ + 8.0f, "Click an EQ band to edit it here with larger fields.", nullptr);
    }

    // Diagnostic readout -- see class comment / kParamHostTempoValid's own
    // comment in PluginReverb.hpp for why this exists.
    char buf[160];
    if (hostTempoValid_)
        std::snprintf(buf, sizeof(buf), "Host Tempo: %.1f BPM (valid)", hostBpmDetected_);
    else
        std::snprintf(buf, sizeof(buf), "Host Tempo: not reported (using 120 BPM fallback while synced)");
    fontSize(12.0f);
    textAlign(ALIGN_LEFT | ALIGN_MIDDLE);
    fillColor(hostTempoValid_ ? Color(140, 210, 140) : Color(200, 160, 90));
    text(diagTextX_, diagTextY_, buf, nullptr);
}

void UIReverb::uiIdle()
{
    // "The editor is currently open" heartbeat -- see PluginReverb.hpp's
    // own kParamUiHeartbeat comment. Must *toggle*, not resend a constant
    // -- see UIEq.cpp's identical comment (../../statespace) for why a
    // constant resent value only ever reaches the DSP once in some hosts.
    uiHeartbeatToggle_ = 1.0f - uiHeartbeatToggle_;
    setParameterValue(kParamUiHeartbeat, uiHeartbeatToggle_);

    // Spectrum smoothing: a one-pole exponential filter per band, toward
    // whatever parameterChanged() most recently received -- done here
    // (UI-side), not on the DSP side, so "how fast/smooth" stays a pure
    // UI concern with no DSP-side parameter/state round-trip, same
    // reasoning UIEq.cpp's own identical block gives. Fixed constant for
    // v1 (no "Spectrum Smoothing" field yet, unlike the sibling EQ
    // plugins) -- a documented, low-risk simplification, not a
    // correctness issue.
    constexpr float kSmoothCoeff = 0.25f;
    for (int i = 0; i < ssfilter::SpectrumAnalyzer::kNumBands; ++i)
    {
        spectrumInSmoothed_[i] += kSmoothCoeff * (spectrumInRaw_[i] - spectrumInSmoothed_[i]);
        spectrumOutSmoothed_[i] += kSmoothCoeff * (spectrumOutRaw_[i] - spectrumOutSmoothed_[i]);
    }
    graph_->setInputSpectrumData(spectrumInSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
    graph_->setOutputSpectrumData(spectrumOutSmoothed_, ssfilter::SpectrumAnalyzer::kNumBands);
    repaint();
}

void UIReverb::parameterChanged(uint32_t index, float value)
{
    if (index >= kParamSpectrumOutputBase && index < kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
    {
        const int packet = int(index - kParamSpectrumOutputBase);
        float triple[ssfilter::SpectrumAnalyzer::kBandsPerPacket];
        ssfilter::SpectrumAnalyzer::unpackTriple(value, triple);
        for (int k = 0; k < ssfilter::SpectrumAnalyzer::kBandsPerPacket; ++k)
        {
            const int band = packet * ssfilter::SpectrumAnalyzer::kBandsPerPacket + k;
            if (band < ssfilter::SpectrumAnalyzer::kNumBands) spectrumOutRaw_[band] = triple[k];
        }
        return; // no repaint() here -- uiIdle()'s own smoothing pass repaints every tick regardless
    }
    if (index >= kParamSpectrumInputBase && index < kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands)
    {
        const int packet = int(index - kParamSpectrumInputBase);
        float triple[ssfilter::SpectrumAnalyzer::kBandsPerPacket];
        ssfilter::SpectrumAnalyzer::unpackTriple(value, triple);
        for (int k = 0; k < ssfilter::SpectrumAnalyzer::kBandsPerPacket; ++k)
        {
            const int band = packet * ssfilter::SpectrumAnalyzer::kBandsPerPacket + k;
            if (band < ssfilter::SpectrumAnalyzer::kNumBands) spectrumInRaw_[band] = triple[k];
        }
        return;
    }

    switch (index)
    {
    case kParamBypass:              bypass_ = value >= 0.5f; btnBypass_->setLabel(bypass_ ? "Bypassed" : "Bypass"); break;
    case kParamInputGainDb:         inputGainDb_ = value; break;
    case kParamPreDelayMs:          preDelayMs_ = value; break;
    case kParamDecayTimeSec:        decayTimeSec_ = value; break;
    case kParamDamping:             damping_ = value; break;
    case kParamDiffusion:           diffusionAmount_ = value; break;
    case kParamModulation:          modulationAmount_ = value; break;
    case kParamWidth:                widthAmount_ = value; break;
    case kParamBalance:              balanceAmount_ = value; break;
    case kParamTempoSyncEnabled:
        tempoSyncEnabled_ = value >= 0.5f;
        btnTempoSync_->setLabel(tempoSyncEnabled_ ? "Tempo Sync: On" : "Tempo Sync: Off");
        rebuildMainFields();
        break;
    case kParamTempoSyncFollowHost:
        followHostTempo_ = value >= 0.5f;
        btnFollowHost_->setLabel(followHostTempo_ ? "Follow Host Tempo: On" : "Follow Host Tempo: Off");
        break;
    case kParamTempoSyncNoteValue:
    {
        tempoNoteValueIndex_ = int(value + 0.5f);
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Note: %s", noteValueLabel(tempoNoteValueIndex_));
        btnNoteValue_->setLabel(buf);
        break;
    }
    case kParamDryWet:               dryWet_ = value; break;
    case kParamOutputTrimDb:         outputTrimDb_ = value; break;
    case kParamHostTempoValid:       hostTempoValid_ = value >= 0.5f; break;
    case kParamHostBpmDetected:      hostBpmDetected_ = value; break;
    case kParamCurrentDecayTimeSec:  currentDecayTimeSec_ = value; break;
    default: break;
    }
    repaint();
}

void UIReverb::stateChanged(const char* key, const char* value)
{
    if (std::strcmp(key, "roomLengthM") == 0) { roomLengthM_ = std::atof(value); return; }
    if (std::strcmp(key, "roomWidthM") == 0) { roomWidthM_ = std::atof(value); return; }
    if (std::strcmp(key, "roomHeightM") == 0) { roomHeightM_ = std::atof(value); return; }
    if (std::strcmp(key, "eqModel") == 0)
    {
        ssfilter::EqModel m;
        if (ssfilter::eq::fromJson(value, m)) { eqModel_ = m; repaint(); }
        return;
    }
    if (std::strcmp(key, "spectrumAutoscale") == 0)
    {
        graph_->setSpectrumAutoscale(std::atoi(value) != 0);
        return;
    }
}

bool UIReverb::onCharacterInput(const CharacterInputEvent& ev)
{
    ParamStripWidget* editing = roomStrip_->isEditing() ? roomStrip_ : (mainStrip_->isEditing() ? mainStrip_ : (bandStrip_->isEditing() ? bandStrip_ : nullptr));
    if (editing == nullptr) return false;
    editing->appendChar(ev.string[0]);
    return true;
}

bool UIReverb::onKeyboard(const KeyboardEvent& ev)
{
    ParamStripWidget* editing = roomStrip_->isEditing() ? roomStrip_ : (mainStrip_->isEditing() ? mainStrip_ : (bandStrip_->isEditing() ? bandStrip_ : nullptr));

    if (editing == nullptr)
    {
        if (ev.press && (ev.key == DGL_NAMESPACE::kKeyDelete || ev.key == DGL_NAMESPACE::kKeyBackspace))
        {
            removeSelectedBand();
            return true;
        }
        return false;
    }
    if (!ev.press) return false;

    switch (ev.key)
    {
    case DGL_NAMESPACE::kKeyEnter:
    case DGL_NAMESPACE::kKeyTab:
        editing->commitEdit();
        return true;
    case DGL_NAMESPACE::kKeyEscape:
        editing->cancelEdit();
        return true;
    case DGL_NAMESPACE::kKeyBackspace:
    case DGL_NAMESPACE::kKeyDelete:
        editing->backspace();
        return true;
    default:
        return true;
    }
}

void UIReverb::selectBand(int index)
{
    selectedBand_ = index;
    graph_->setSelectedBand(index);
    list_->setSelectedBand(index);
    repaint();
}

void UIReverb::removeSelectedBand()
{
    if (selectedBand_ >= 0) removeBand(selectedBand_);
}

// ---- ssgui::IEqEditorHost -------------------------------------------

ssfilter::EqBand UIReverb::getBand(int index) const
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return ssfilter::EqBand{};
    return eqModel_.bands[size_t(index)];
}

void UIReverb::setBandFreq(int index, double freqHz)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(freqHz)) return;
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::eq::propagateHarmonicFreq(eqModel_, index, freqHz, sr);
    pushEqModel();
}

void UIReverb::setBandGain(int index, double gainDb)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(gainDb)) return;
    eqModel_.bands[size_t(index)].gainDb = std::max(-60.0, std::min(60.0, gainDb));
    pushEqModel();
}

void UIReverb::setBandQ(int index, double q)
{
    if (index < 0 || index >= int(eqModel_.bands.size()) || !std::isfinite(q)) return;
    eqModel_.bands[size_t(index)].q = std::max(0.05, std::min(100.0, q));
    pushEqModel();
}

void UIReverb::setBandType(int index, ssfilter::EqBandType type)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    ssfilter::EqBand& b = eqModel_.bands[size_t(index)];
    b.type = type;
    if (!ssfilter::eqBandTypeHasGain(type)) b.gainDb = 0.0;
    pushEqModel();
}

void UIReverb::setBandSlopeStages(int index, int slopeStages)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].slopeStages = std::max(1, std::min(4, slopeStages));
    pushEqModel();
}

void UIReverb::setBandEnabled(int index, bool enabled)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].enabled = enabled;
    pushEqModel();
}

void UIReverb::setBandSolo(int index, bool solo)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].solo = solo;
    pushEqModel();
}

void UIReverb::setBandChannel(int index, ssfilter::EqChannel channel)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands[size_t(index)].channel = channel;
    pushEqModel();
}

void UIReverb::addBand(double freqHz, double gainDb)
{
    constexpr int kMaxBands = ssfilter::kMaxOrder / 4; // same cap as the sibling EQ plugin
    if (int(eqModel_.bands.size()) >= kMaxBands) return;

    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    ssfilter::EqBand b;
    b.type = ssfilter::EqBandType::Bell;
    b.freqHz = std::max(20.0, std::min(sr * 0.499, freqHz));
    b.gainDb = std::max(-60.0, std::min(60.0, gainDb));
    b.q = 1.0;
    eqModel_.bands.push_back(b);
    pushEqModel();
}

void UIReverb::removeBand(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    eqModel_.bands.erase(eqModel_.bands.begin() + index);
    selectBand(selectedBand_ == index ? -1 : (selectedBand_ > index ? selectedBand_ - 1 : selectedBand_));
    pushEqModel();
}

void UIReverb::duplicateBand(int index)
{
    if (index < 0 || index >= int(eqModel_.bands.size())) return;
    constexpr int kMaxBands = ssfilter::kMaxOrder / 4;
    if (int(eqModel_.bands.size()) >= kMaxBands) return;

    const ssfilter::EqBand copy = eqModel_.bands[size_t(index)];
    const int newIndex = index + 1;
    eqModel_.bands.insert(eqModel_.bands.begin() + newIndex, copy);
    selectBand(newIndex);
    pushEqModel();
}

void UIReverb::createHarmonicTwin(int index, double factor)
{
    const double sr = getSampleRate() > 0.0 ? getSampleRate() : 44100.0;
    const int newIndex = ssfilter::eq::createHarmonicTwin(eqModel_, index, factor, sr);
    if (newIndex < 0) return;
    selectBand(newIndex);
    pushEqModel();
}

void UIReverb::unlinkHarmonic(int index)
{
    ssfilter::eq::unlinkHarmonic(eqModel_, index);
    pushEqModel();
}

void UIReverb::beginBandTextEdit(int index, ssgui::EqField field, bool clearValue)
{
    (void)clearValue;
    // v1: numeric text entry for a band field goes through BandListWidget's
    // own click-to-edit fields (which it drives independently), not a
    // second text-edit path here -- EqGraphWidget's own drag/scroll
    // interaction is how freq/gain/Q are set directly on the graph. This
    // is a narrower scope than the sibling EQ plugin's dual text-edit
    // paths (graph nodes there don't have their own typed-entry either;
    // only the list does), so this is a documented no-op, not a bug.
    (void)index;
    (void)field;
}

bool UIReverb::isEditingBandField(int, ssgui::EqField, std::string&) const
{
    return false;
}

void UIReverb::pushEqModel()
{
    setState("eqModel", ssfilter::eq::toJson(eqModel_).c_str());
    repaint();
}

END_NAMESPACE_DISTRHO

START_NAMESPACE_DISTRHO
void UIReverb::onDisplay()
{
    // Replaces DPF's top-level frame, which uses a pixel ratio of 1 (blurry
    // text when scaled) -- see EditorSubWidget.hpp.
    NanoVG::beginFrame(getWidth(), getHeight(), DGL_NAMESPACE::editorPixelRatio(*this));
    onNanoDisplay();
    DGL_NAMESPACE::paintEditorChildren(*this);
    NanoVG::endFrame();
}

UI* createUI() { return new UIReverb(); }
END_NAMESPACE_DISTRHO
