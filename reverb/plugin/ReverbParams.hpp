// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

// Shared ParameterIndex/StateIndex for every reverb "model" plugin
// (plugin_dense/ "reverb_dense", plugin_plate/ "reverb_plate" -- see
// docs/REVERB_CONCEPT.md). Kept in this shared file so
// gui/UIReverb.hpp/.cpp
// -- the ENTIRE GUI, unchanged -- can be compiled into all the plugin_ui
// targets without depending on any one model's own PluginReverb.hpp
// (which differs per model: each has its own, different, engine member
// type). Several reverb plugins with different names, the same interface
// but a different sound: this file is what makes "same interface"
// literally true -- one UIReverb.cpp,
// one Parameter/State contract, completely different DSP engines behind
// each one.

#include "DistrhoPlugin.hpp"
#include "dsp/SpectrumAnalyzer.hpp"

START_NAMESPACE_DISTRHO

enum ParameterIndex
{
    kParamBypass = 0,
    kParamResetTrigger = 1,
    kParamFaultFlag = 2, // output -- <Engine>::consumeFaultFlag()

    // Applied to the incoming signal before it splits into the dry path
    // and the reverb tank's own input -- see run()'s own comment for why
    // both, not just the wet path (same range/unit/default convention as
    // the sibling ../../compressor project's own kParamInputGain).
    kParamInputGainDb = 3,

    kParamDecayTimeSec = 4,   // manual Decay Time (T60, seconds) -- ignored while Tempo Sync is enabled
    kParamDamping = 5,        // 0..1, how much faster the tail's high end decays vs. its low end

    // See ReverbEngine.hpp's "Tempo-synced decay" section (and
    // docs/REVERB_CONCEPT.md's own section of the same name) for the full
    // design. TempoSyncNoteValue's float value is the integer index into
    // ssfilter::reverb::TempoNoteValue, same convention as every other
    // small-discrete-choice Parameter in the sibling statespace/ project
    // (e.g. kParamAuditionBand) -- a plain kParameterIsInteger range, no
    // DPF enumValues (unused anywhere else in this codebase yet).
    kParamTempoSyncEnabled = 6,
    kParamTempoSyncFollowHost = 7, // only meaningful while TempoSyncEnabled is on
    kParamTempoSyncNoteValue = 8,

    kParamDryWet = 9,        // 0..100% -- see kParamDryWet's own history for why not 0..1
    kParamOutputTrimDb = 10, // applied to the wet signal only, before the dry/wet mix

    // Diagnostic/GUI readouts, all output-only, appended at the end
    // rather than inserted among the indices above (same convention as
    // the sibling statespace/ project's own late-added output params,
    // e.g. PluginDynEq.hpp's kParamAudition comment) -- added when the
    // BPM-dependent decay seemed to have no audible effect: these make the tempo-sync mechanism's actual, live state visible
    // (what BPM/validity the host is reporting right now, and what decay
    // time is actually active in the engine right now) instead of relying
    // on judging it by ear alone, which is a real diagnostic gap for a
    // feature whose only prior interface was a raw host parameter list.
    kParamHostTempoValid = 11,    // 1 if the host is currently reporting valid BBT/tempo at all (see run()'s own comment)
    kParamHostBpmDetected = 12,   // raw beatsPerMinute currently held by DPF, 0 if never received
    kParamCurrentDecayTimeSec = 13, // <Engine>::currentT60DcSec() -- the decay time actually active right now, manual or tempo-derived

    // 0..1, how strongly the allpass pre-diffuser acts -- against a
    // metallic, tube-like sound, the standard fix for exactly that symptom
    // in a small-line-count FDN.
    // Meaningful for every model that has a pre-diffuser stage at all
    // (all three do, in some form -- see each model's own engine header
    // for what this control actually drives there).
    kParamDiffusion = 14,

    // 0 = the tank's own wet output collapsed to mono, 1 (default) = its
    // natural, unaltered stereo image, up to 2 = an exaggerated side
    // signal -- a plain mid/side blend applied only to the wet signal,
    // never the dry path -- for managing the reverb's stereo field.
    kParamWidth = 15,

    // -1..+1, a plain per-channel gain differential on the wet signal
    // (applied after Width), to correct any left/right lean. A genuine
    // Balance (unity at center) rather than a constant-power Pan.
    kParamBalance = 16,

    // 0..1, how strongly delay-line modulation acts -- the fix for
    // uncontrolled resonant frequencies that sound like the material
    // pitch-shifted far down (audible FDN/plate ringing). See each
    // model's own engine header for the exact mechanism -- an FDN
    // modulates each delay line's own length; the plate model modulates
    // its own tank allpasses instead (Dattorro's own established design),
    // but the user-facing knob means the same thing in both: "how much
    // anti-ringing smoothing".
    kParamModulation = 17,

    // 0..250ms, a pure delay on the wet signal's own input, before it
    // ever reaches the tank, in every reverb model. Already named as a planned
    // parameter in docs/REVERB_CONCEPT.md's own table since this
    // project's first pass; see plugin/dsp/PreDelayLine.hpp for the
    // actual mechanism (shared unchanged across both models, unlike
    // the tank engines themselves -- there's nothing model-specific
    // about "hold the wet signal back N samples").
    kParamPreDelayMs = 18,

    // "The editor is currently open" heartbeat + the EQ's pre-/post-EQ
    // background spectrum -- same mechanism/reasoning as every sibling
    // EQ plugin in ../../statespace (see e.g. that project's PluginEq.hpp
    // for the original kParamUiHeartbeat/spectrum-bank comments this
    // mirrors): run() only pays the FFT/windowing cost of
    // inputSpectrumAnalyzer_/outputSpectrumAnalyzer_ while uiOpen is
    // true, and each analyzer's kNumBands values are packed 3-per-float
    // (SpectrumAnalyzer::getPackedTriple()) into kNumPackedBands output
    // Parameters rather than kNumBands separate ones. "Input" here means
    // pre-EQ (the tank's own wet output before eqEngine_), "Output" means
    // post-EQ (the final wet signal), both drawn on the filter plot.
    kParamUiHeartbeat = 19,
    kParamSpectrumOutputBase = 20, // + [0 .. kNumPackedBands) -- post-EQ
    kParamSpectrumInputBase = kParamSpectrumOutputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands, // pre-EQ

    kParamCount = kParamSpectrumInputBase + ssfilter::SpectrumAnalyzer::kNumPackedBands
};

enum StateIndex
{
    // Room dimensions are State, not Parameters -- deliberately, see
    // ReverbEngine.hpp's "Room-size changes: thread-safe request + audio-
    // thread-only apply" section. A room-size edit needs a full
    // <Engine>::prepare() (delay-line reallocation), which is too heavy/
    // architecturally different from an ordinary Decay Time tweak to
    // treat as a routinely-host-automated Parameter in this first
    // version -- State changes are message-thread-only by DPF's own
    // contract, matching every sibling MimoMusic plugin's own convention
    // for exactly this class of edit (see e.g. ../../statespace/'s own
    // setState("model", ...) for coefficient changes).
    kStateRoomLengthM = 0,
    kStateRoomWidthM = 1,
    kStateRoomHeightM = 2,

    // The post-tank parametric EQ's whole band list, as JSON
    // (ssfilter::eq::toJson()/fromJson()) -- a whole-model State, exactly
    // like Room L/W/H above, rather than the sibling statespace/ project's
    // own Parameter-mirror mechanism (see PluginReverb.cpp's
    // rebuildEqEngine()'s own comment for why that trade-off was made
    // here).
    kStateEqModel = 3,

    // The FFT/spectrum overlay's own "Autoscale"/"Fixed Range" toggle
    // (EqGraphWidget::setSpectrumAutoscale()) -- ported from the sibling
    // ../../statespace EQ plugins' own identical control. Despite being
    // nicknamed "auto-gain", it never touches any audio gain/trim/DSP parameter --
    // it only auto-scales the spectrum plot's own vertical (dB) axis to
    // fit whatever's currently on screen, the same way every one of the
    // statespace EQ family's own graphs already does. UI-convenience
    // only, same as kStateEqModel above's own hints=0 -- the DSP side
    // has nothing to do with it (see initState()'s comment in each
    // PluginReverb*.cpp).
    kStateSpectrumAutoscale = 4,
    kStateCount
};

END_NAMESPACE_DISTRHO
