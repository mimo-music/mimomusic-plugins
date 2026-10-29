// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

#pragma once

#include "DistrhoPlugin.hpp"

#include "../plugin/dsp/CompressorDsp.hpp"
#include "../plugin/dsp/SpectrumAnalyzer.hpp"

START_NAMESPACE_DISTRHO

// Same small automatable-Parameter bank as PluginCompressorMono, plus one
// extra pair of meters per side (L/R) instead of one, and a Stereo Mode
// choice. Stereo Mode is a State (like statespace's
// EQ "spectrumMode"), not a Parameter: it's a small discrete processing-
// topology choice, not something meaningful to automate continuously.
enum ParameterIndex
{
    kParamInputGain = 0,
    kParamThreshold,
    kParamRatio,
    kParamAttack,
    kParamRelease,
    kParamKnee,
    kParamOutputGain,
    kParamCompensation,
    kParamCompensationMeasure, // trigger: force a fresh compensation measurement now
    kParamBypass,

    kParamMeterInputRmsL,
    kParamMeterInputPeakL,
    kParamMeterInputRmsR,
    kParamMeterInputPeakR,
    kParamMeterOutputRmsL,
    kParamMeterOutputPeakL,
    kParamMeterOutputRmsR,
    kParamMeterOutputPeakR,
    kParamMeterDetectorA,       // Linked/DualMono: L; MidSide: Mid
    kParamMeterDetectorB,       // Linked: mirrors A; DualMono: R; MidSide: Side
    kParamMeterGainReductionA,
    kParamMeterGainReductionB,
    kParamMeterCompGainDb,
    kParamMeterCompState,      // unused -- see PluginCompressorStereo.cpp's own kParamCompensation comment

    // Appended after the original bank -- see PluginCompressorMono.hpp's
    // identical comment on kParamSidechainEnable for why (an existing
    // Parameter must keep its numeric index) and for the full design.
    // The stereo external key input (indices 2/3 of
    // DISTRHO_PLUGIN_NUM_INPUTS, see DistrhoPluginInfo.h and
    // initAudioPort()) is derived into a detector the same way the main
    // signal already is per Stereo Mode -- max(|L|,|R|) in Linked, per-
    // channel in Dual-Mono, Mid/Side-decoded in Mid/Side -- so switching
    // Sidechain on keeps exactly the same per-mode detector topology,
    // just fed from the key input instead of the main one.
    kParamSidechainEnable,
    // Sidechain Gain, the band filter below, the monitor scope and
    // Sidechain Listen all act on the trigger signal: the external key
    // input while Sidechain is on, the gain-staged main signal otherwise
    // (same as Contour's trigger, see run()).
    kParamSidechainGain,       // dB trim applied to the trigger signal before the filter and the detector; changes what drives the compressor, not the audio being compressed

    // M/S Listen, to hear only Mid or only Side while setting up the
    // compressor -- 0=Off/1=Mid/2=Side. Only
    // takes effect while stereoMode_ == StereoMode::MidSide (see run()'s
    // own comment); harmlessly inert otherwise, same "the control exists
    // but only one mode gives it anything to do" convention
    // kParamSidechainGain already has relative to Sidechain being off.
    // An ordinary Parameter (not a State), same reasoning as the
    // statespace EQ family's own kParamAudition: a real-time, host-
    // automatable audition aid, not a discrete mix-topology choice like
    // Stereo Mode/Ratio Direction. Deliberately excluded from this
    // plugin's own preset save/load and A/B/undo-redo snapshots (see
    // UICompressorStereo.cpp's kPresetParams comment) -- same "momentary
    // setup aid, not mix-defining data" precedent kParamAudition already
    // set project-wide, not something a preset or an A/B slot should
    // silently carry.
    kParamListenMode,

    // A second full dynamics bank, so e.g. Mid and Side can be set
    // completely differently. Used for B's
    // own voice (R in Dual-Mono, Side in Mid/Side) whenever B is actually
    // independent -- see isChannelBIndependent()'s own comment for
    // exactly when that is. When it isn't (Linked always; Dual-Mono while
    // linkAB_ is true), B's *effective* values are just A's own, live --
    // these Parameters still hold whatever they were last set to, but
    // run() never reads them in that case. Mirrors kParamThreshold/
    // kParamRatio/kParamAttack/kParamRelease/kParamKnee exactly, one for
    // one -- appended as a block at the very end rather than interleaved
    // with A's own bank, same "append, never insert" index-stability rule
    // as everywhere else in this project.
    kParamThresholdB,
    kParamRatioB,
    kParamAttackB,
    kParamReleaseB,
    kParamKneeB,
    // "Output Gain" renamed to "Makeup Gain", plus a global output gain,
    // so the two channels can still be amplified individually.
    // kParamOutputGain above
    // keeps its exact index/symbol ("output_gain") for session
    // compatibility -- only its display name changes (see initParameter())
    // -- and keeps working exactly as it always has as A's own makeup
    // gain. kParamMakeupGainB is its counterpart for B's own bank, part of
    // the same "start linked, split on edit" B-bank block as
    // kParamThresholdB etc. above, live only while isChannelBIndependent().
    kParamMakeupGainB,
    // A manual master trim, always live in every stereo mode (this
    // plugin's own first cut gated it to only Dual-Mono-unlinked/Mid-Side;
    // being always there does no harm). Redundant with Makeup Gain (A) in
    // Linked/non-independent-Dual-Mono mode (both just apply a scalar at
    // the same point in run()) but harmless -- defaults to 0dB, so it's a
    // no-op until the user actually reaches for it. (Volume Compensation,
    // which used to auto-write into Makeup Gain, was removed entirely.)
    kParamGlobalOutputGain,

    // Sidechain amplification and band filtering, to trigger the
    // compressor with sound in one specific part of the spectrum --
    // amplification is kParamSidechainGain. This is the "frequency band"
    // half: a high-pass and low-pass corner in series (see
    // sscomp::SidechainBandFilter, CompressorDsp.hpp) applied to the
    // trigger signal (external key or main signal) before it reaches the
    // detector, isolating a band to trigger from. kParamScFilterEnable gates the filter's own DSP
    // cost specifically, so it uses no DSP when off -- run() skips calling the filter at all while this is
    // off, passing the sidechain signal through unfiltered; the
    // monitor/analyzer below keeps running regardless (see its own
    // comment for why only the filter itself needed this gate).
    kParamScFilterEnable,
    kParamScFilterHighpassHz,
    kParamScFilterLowpassHz,

    // For the sidechain-options panel's waveform + spectrum display --
    // the level/spectrum of
    // whatever signal is *currently* reaching the detector: the trigger
    // signal (external key input or main signal, see kParamSidechainGain)
    // unfiltered while kParamScFilterEnable is off, filtered once it's on
    // (so the filter's effect is visible directly), mono-downmixed. kParamMeterScLevelDb is a simple smoothed peak level,
    // read once per block by the GUI's own scrolling level trace (the
    // same "one value per tick, accumulated client-side" technique
    // CompressorGraphWidget::drawGrHistory() already uses -- true
    // per-sample waveform data has no portable cross-process transport in
    // this framework, see SpectrumAnalyzer.hpp's own header comment for
    // the identical reasoning applied to the spectrum below).
    // kParamMeterScSpectrumBase is sscomp::SpectrumAnalyzer::
    // kNumPackedBands (128) consecutive packed-triple output Parameters,
    // same transport as ../../statespace's EQ plugins' own spectrum
    // (see that file's own "Packed transport" comment) -- always fed
    // (this analyzer is comparatively cheap, see its own file comment,
    // and unlike the filter there's no "unused" state to gate it behind:
    // the window wants to show *something* the moment it's open, before
    // the user has necessarily decided to engage the filter at all).
    kParamMeterScLevelDb,
    kParamMeterScSpectrumBase,

    // Sidechain Listen (pass only the sidechain, to hear the frequencies
    // the detector reacts to) -- replaces the main
    // output with the same signal the monitor/scope above are already
    // showing (raw sidechain, or filtered once kParamScFilterEnable is
    // on), in genuine stereo (not mono-summed -- same "don't collapse to
    // mono" lesson kParamListenMode's own Mid/Side Listen already
    // learned this session). Applied late in run(), after the M/S
    // Listen override (taking precedence if both are somehow on) but
    // before Global Output Gain, so overall level trim still works while
    // auditioning. Appended here, after the whole packed spectrum bank,
    // rather than next to kParamScFilterEnable where it reads more
    // naturally -- "append, never insert": this session already shipped
    // kParamMeterScSpectrumBase's own base index once, so nothing may be
    // inserted before it any more, session compatibility or not.
    kParamScListenEnable = kParamMeterScSpectrumBase + sscomp::SpectrumAnalyzer::kNumPackedBands,

    // A mode where left and right are filtered differently --
    // kParamScFilterHighpassHz/
    // LowpassHz above stay L's own (and the *only* pair that matters
    // while linked, same as before this feature existed); this is R's
    // own independent pair, live only while kParamScFilterLinkLR is off.
    // Default true (linked) -- matches this plugin's own pre-existing
    // shared-parameter behavior until a user asks for something else,
    // same convention as e.g. kStateLinkAB's own comment.
    kParamScFilterLinkLR,
    kParamScFilterHighpassHzR,
    kParamScFilterLowpassHzR,

    // Per-channel rectified peak level (blue left on top, mirrored red
    // right below in the scope), replacing the previous
    // single mono-downmixed kParamMeterScLevelDb (kept declared, just no
    // longer read by the GUI) for the scope's own level trace.
    kParamMeterScLevelDbL,
    kParamMeterScLevelDbR,

    // Input and output spectrum always shown, like on the EQs -- a second
    // packed spectrum bank, pre-filter this
    // time (kParamMeterScSpectrumBase above is post-filter/whatever's
    // currently reaching the detector, unchanged in meaning). Both
    // always fed regardless of kParamScFilterEnable -- when the filter's
    // off the two traces are simply identical, no extra filter DSP cost
    // either way, see that Parameter's own comment.
    kParamMeterScSpectrumInBase,

    // Per-channel input amplification in Dual-Mono/Mid-Side -- B's own
    // Input
    // Gain, same "start linked, split on edit" fallback as the rest of
    // B's bank (kParamThresholdB etc.): live only while
    // isChannelBIndependent(), mirroring A's own kParamInputGain
    // otherwise. In Mid/Side specifically this applies to the *Side*
    // component after Mid/Side decode, not to R directly -- see run()'s
    // own comment for why gain has to move to after the decode there
    // (applying different gains to L/R before decoding would leak into
    // both Mid and Side, not cleanly separate them the way a compressor
    // GUI showing "two independent compressors" implies).
    kParamInputGainB = kParamMeterScSpectrumInBase + sscomp::SpectrumAnalyzer::kNumPackedBands,

    // A Wet/Dry control per compressor, 0-100%, default 100% wet -- a
    // standard
    // parallel-compression mix: 100% (the default, exactly reproducing
    // this plugin's own pre-existing always-fully-wet behavior) plays
    // only the compressed-and-made-up signal, 0% only the dry, gain-
    // staged-but-uncompressed signal, values between blend the two
    // linearly. kParamWetDry is A's own, live in *every* stereo mode
    // (including Linked, which has no B bank at all). kParamWetDryB is B's own counterpart, same
    // isChannelBIndependent() fallback as every other B-bank Parameter.
    kParamWetDry,
    kParamWetDryB,

    // Global Input Gain (Dual-Mono/Mid-Side), amplifying the stereo signal
    // before splitting or converting to M/S -- a single trim
    // applied to the raw L/R input, before *any* per-mode derivation:
    // before kParamInputGain/kParamInputGainB's own per-channel split in
    // Dual-Mono, before the Mid/Side decode in Mid/Side (see run()'s own
    // comment for exactly where). Not shown in Linked mode -- and,
    // matching that, run() only ever applies it outside Linked mode too
    // (same "hidden in the GUI, and DSP-inert too" convention every other
    // Dual-Mono/Mid-Side-only Parameter in this file already follows, e.g.
    // kParamScFilterHighpassHzR) -- a stray host automation lane could
    // otherwise silently affect Linked-mode audio through a control the
    // GUI never shows there. UICompressorStereo resets this to 0dB whenever stereoMode_
    // transitions *out of* Linked, not just once at construction.
    kParamGlobalInputGain,

    // "The Trigger Options scope is on screen" heartbeat -- same mechanism as PluginContour's and the statespace
    // EQs' kParamUiHeartbeat: the editor bumps this counter every idle tick WHILE the sidechain scope panel is
    // visible; run() feeds the two spectrum analysers (most of this plugin's CPU) only while it changed within
    // the last ~2 s. An increasing counter, not a 0/1 toggle (DPF's LV2 wrapper only delivers a port whose value
    // differs from the last block's, so two toggles inside one long block cancel out). Appended last.
    kParamUiHeartbeat,

    kParamCount
};

enum StateIndex
{
    kStateStereoMode = 0, // sscomp::StereoMode (0=Linked/1=DualMono/2=MidSide)
    kStatePresetDir,       // last folder used in the Save As.../Load... dialogs (UI convenience only)
    // Which side of the threshold kParamRatio actually compresses --
    // "0"=Down (the historical/default behavior, compresses level above
    // the threshold), "1"=Up (compresses below it instead, lifting quiet
    // material). Replaces a since-removed second parameter (kParamRatioUp)
    // that could be dialed in *simultaneously* with kParamRatio, which
    // read as two independent, always-both-active compressors rather than
    // "pick a direction".
    // A discrete two-way choice, so it's a State (toggled by a button) and
    // not a continuous automatable Parameter, same convention as
    // kStateStereoMode's own.
    kStateRatioDirection,
    // B's own Ratio Direction -- same shape/reasoning as
    // kStateRatioDirection, just for B's independent bank (see
    // kParamThresholdB's own comment). Meaningless whenever B isn't
    // independent, same as the rest of B's bank.
    kStateRatioDirectionB,
    // Link L/R parameters (Dual-Mono), or keep them separate --
    // "0"=Independent/"1"=Linked (default), toggled
    // by a button shown only in Dual-Mono mode (see
    // UICompressorStereo::updateMeterLabelsForMode()). Mid/Side ignores
    // this entirely and is *always* independent -- see
    // isChannelBIndependent()'s own comment. A discrete two-way choice,
    // same State (not Parameter) convention as kStateStereoMode/
    // kStateRatioDirection.
    kStateLinkAB,
    // GR meter speed in the editor, "0"=Fast/"1"=Slow (UI preference only,
    // the DSP never reads it -- same as presetDir).
    kStateGrMeterSpeed,
    kStateCount
};

class PluginCompressorStereo : public Plugin
{
public:
    PluginCompressorStereo();

protected:
    uint32_t getVersion() const override { return d_version(2, 1, 0); } // 2.1: sidechain/external key input added (kParamSidechainEnable/Gain, appended -- existing indices unchanged)

    void initAudioPort(bool input, uint32_t index, AudioPort& port) override;
    void initParameter(uint32_t index, Parameter& parameter) override;
    void initState(uint32_t index, State& state) override;

    float getParameterValue(uint32_t index) const override;
    void  setParameterValue(uint32_t index, float value) override;

    void setState(const char* key, const char* value) override;

    void activate() override;
    void run(const float** inputs, float** outputs, uint32_t frames) override;
    void sampleRateChanged(double newSampleRate) override;

private:
    // Whether B's own dynamics bank (kParamThresholdB etc.) is actually
    // live right now, rather than B just mirroring A -- Mid/Side always
    // (independent Mid/Side shaping is the entire point, "a link is not
    // for use" there), Dual-Mono only while linkAB_ is false, Linked
    // never (voiceB_'s own processing there is already purely for meter
    // parity, see run()'s own comment -- giving it independent dynamics
    // would only make its *readout* diverge from voiceA_'s, never the
    // actual audio, which is misleading for no benefit).
    bool isChannelBIndependent() const
    {
        return stereoMode_ == sscomp::StereoMode::MidSide ||
               (stereoMode_ == sscomp::StereoMode::DualMono && !linkAB_);
    }

    // Re-applies scFilterHighpassHz_/scFilterLowpassHz_ to scFilterL_
    // always, and to scFilterR_ too unless scFilterLinkLR_ is false (in
    // which case scFilterHighpassHzR_/scFilterLowpassHzR_ apply there
    // instead) -- the one place that logic lives, called from every
    // setParameterValue() case that could change it (either channel's
    // own frequencies, or the link toggle itself).
    void updateScFilterFrequencies()
    {
        scFilterL_.setFrequencies(scFilterHighpassHz_, scFilterLowpassHz_);
        scFilterR_.setFrequencies(scFilterLinkLR_ ? scFilterHighpassHz_ : scFilterHighpassHzR_,
                                   scFilterLinkLR_ ? scFilterLowpassHz_ : scFilterLowpassHzR_);
    }

    sscomp::CompressorVoice voiceA_, voiceB_;
    sscomp::LoudnessMeter loudnessInL_, loudnessInR_, loudnessOutL_, loudnessOutR_;
    sscomp::PeakMeter peakInL_, peakInR_, peakOutL_, peakOutR_;
    sscomp::GrPeakMeter grMeterA_, grMeterB_; // the GR readouts -- see its own comment

    double inputGainDb_ = 0.0;
    double thresholdDb_ = -18.0;
    double ratio_ = 4.0;
    bool ratioUpMode_ = false; // false=Down (compress above threshold), true=Up (compress below it) -- see kStateRatioDirection's own comment
    double attackMs_ = 10.0;
    double releaseMs_ = 150.0;
    double kneeDb_ = 6.0;
    double outputGainDb_ = 0.0;
    // Vestigial -- round-trips kParamCompensation's own stored value, but
    // nothing reads it any more. See that Parameter's own comment
    // (initParameter(), PluginCompressorStereo.cpp).
    bool compensationEnabled_ = false;
    bool bypass_ = false;
    bool sidechainEnabled_ = false;
    double sidechainGainDb_ = 0.0;
    int listenMode_ = 0; // 0=Off/1=Mid/2=Side -- see kParamListenMode's own comment
    sscomp::StereoMode stereoMode_ = sscomp::StereoMode::Linked;

    // B's own independent dynamics bank -- see kParamThresholdB's own
    // comment and isChannelBIndependent(). Defaults match A's own
    // defaults exactly, so a freshly-instantiated plugin behaves
    // identically regardless of stereo mode until something actually
    // diverges them.
    double thresholdDbB_ = -18.0;
    double ratioB_ = 4.0;
    bool ratioUpModeB_ = false;
    double attackMsB_ = 10.0;
    double releaseMsB_ = 150.0;
    double kneeDbB_ = 6.0;
    double makeupGainDbB_ = 0.0; // see kParamMakeupGainB's own comment
    double inputGainDbB_ = 0.0; // see kParamInputGainB's own comment
    double wetDry_ = 1.0;  // see kParamWetDry's own comment -- 0..1, not 0..100, converted once in setParameterValue()
    double wetDryB_ = 1.0; // see kParamWetDryB's own comment
    double globalInputGainDb_ = 0.0; // see kParamGlobalInputGain's own comment
    bool linkAB_ = true; // see kStateLinkAB's own comment

    // See kParamGlobalOutputGain's own comment -- the master trim that
    // takes over once Makeup Gain (outputGainDb_ above) has split per-bank.
    double globalOutputGainDb_ = 0.0;

    // See kParamScFilterEnable's own comment. One filter instance per
    // channel (L/R), applied before the per-stereo-mode detector
    // derivation so Linked/Dual-Mono/Mid-Side all see the already-
    // filtered signal identically, same as Sidechain Gain's own
    // (pre-existing) position in the signal chain. R's own frequencies
    // only actually differ from L's while scFilterLinkLR_ is false -- see
    // kParamScFilterLinkLR's own comment.
    bool scFilterEnabled_ = false;
    double scFilterHighpassHz_ = 100.0;
    double scFilterLowpassHz_ = 8000.0;
    bool scFilterLinkLR_ = true;
    double scFilterHighpassHzR_ = 100.0;
    double scFilterLowpassHzR_ = 8000.0;
    sscomp::SidechainBandFilter scFilterL_, scFilterR_;

    // See kParamMeterScLevelDbL/R's own comment -- always fed, regardless
    // of scFilterEnabled_/sidechainEnabled_. Per-channel now (no mono
    // downmix).
    sscomp::PeakMeter scMonitorPeakL_, scMonitorPeakR_;
    // See kParamMeterScSpectrumInBase's own comment -- pre-filter (In)
    // and post-filter/current-signal (Out, the pre-existing
    // scMonitorSpectrum_ from before this entry, just renamed for
    // clarity now that there are two).
    sscomp::SpectrumAnalyzer scMonitorSpectrumIn_, scMonitorSpectrumOut_;
    uint64_t uiHeartbeatSamplesSinceLast_ = UINT64_MAX; // samples since the last heartbeat delivery; UINT64_MAX = none yet
    // Scratch mono-downmix buffers the two analyzers above read from --
    // fixed-size, never (re)allocated in run(), same RT-safety discipline
    // SpectrumAnalyzer.hpp's own ring buffer already follows. A block
    // larger than this simply feeds the analyzers/level meters a
    // truncated (not wrong, just shorter) slice that block -- the actual
    // audio path/detector are entirely unaffected either way, this is a
    // visual aid only.
    static constexpr uint32_t kScMonitorScratchFrames = 8192;
    float scMonitorScratchIn_[kScMonitorScratchFrames];
    float scMonitorScratchOut_[kScMonitorScratchFrames];

    // See kParamScListenEnable's own comment.
    bool scListenEnabled_ = false;

    float meterInputRmsL_ = -120.0f, meterInputPeakL_ = -120.0f;
    float meterInputRmsR_ = -120.0f, meterInputPeakR_ = -120.0f;
    float meterOutputRmsL_ = -120.0f, meterOutputPeakL_ = -120.0f;
    float meterOutputRmsR_ = -120.0f, meterOutputPeakR_ = -120.0f;
    float meterDetectorA_ = -120.0f, meterDetectorB_ = -120.0f;
    float meterGainReductionA_ = 0.0f, meterGainReductionB_ = 0.0f;

    DISTRHO_DECLARE_NON_COPYABLE(PluginCompressorStereo)
};

END_NAMESPACE_DISTRHO
