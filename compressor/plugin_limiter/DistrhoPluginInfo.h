// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Limiter
//
// A third sibling in this project, built on the exact same DSP core as the
// compressor plugins (plugin/dsp/CompressorDsp.hpp, unchanged): a limiter
// is still "a compressor tuned a specific way, plus one thing it isn't"
// underneath -- sscomp::computeGainReductionDb()'s soft-knee ratio line
// already becomes an effectively brick-wall gain reduction at a high ratio
// and a near-zero knee, so PluginLimiter reuses sscomp::CompressorVoice
// verbatim, plus a hard output-ceiling clamp (kParamCeiling) that
// guarantees the output sample itself never exceeds it even if the
// envelope follower's attack time lets one sample slip past the gain
// computer's intended reduction.
//
// Reset from scratch: the first
// version of this plugin exposed PluginCompressorStereo's *entire*
// parameter set verbatim (Ratio, Up Ratio, Knee, Attack, Volume
// Compensation, three Stereo Modes) plus Ceiling, which read as "the
// stereo compressor with an extra knob," not as a limiter -- confusing
// rather than generic settings similar to the compressor. That version kept only Threshold/Release/Ceiling/Input/Output
// Gain, fixing Ratio/Attack/Knee/Stereo-mode internally.
//
// Revised again after
// comparing against how dedicated limiter plugins (e.g. LSP's Limiter
// Stereo/Mono) actually lay their controls out: those don't expose Ratio
// either (a limiter is inherently near-brick-wall, not a tunable ratio),
// but they do expose Attack, Knee, and -- the one genuinely missing piece
// here -- **Lookahead**, a small delay buffer that lets the gain computer
// react to a transient *before* it reaches the output instead of only
// after, which is most of what makes a dedicated limiter sound cleaner
// than "a compressor with a hard clip bolted on". So Attack and Knee are
// real parameters again (this plugin's own values, not shared with the
// compressors' defaults), and kParamLookahead adds real added-latency
// look-ahead (DISTRHO_PLUGIN_WANT_LATENCY, below, is now on so hosts
// compensate for it correctly). Ratio and stereo linking are still fixed
// internally -- see PluginLimiter.cpp's kFixedRatio -- and Volume
// Compensation/multiple Stereo Modes are still deliberately not here.
//
// Boost (kParamBoost, PluginLimiter.hpp): like the boost button some
// limiters have, adding the reduced gain back at the output. Surveyed
// dedicated limiter plugins first rather than guessing -- LSP's own
// Limiter Stereo/Mono manual: "Boost - applies corresponding to the
// Threshold gain to the output signal"; FabFilter Pro-L 2's "Unity
// Gain" and Slate FG-X's "Constant Gain" both do the same thing tied to
// their own input-gain knob instead. All three are a STATIC offset from
// a knob value, not a live tracker of the gain-reduction meter --
// implemented the same way here, additively on top of Output Gain.
//
// Same TODO as every sibling plugin's own DistrhoPluginInfo.h:
// DISTRHO_PLUGIN_URI must be a URI you actually control before any real
// release -- the brand/URI below are the publisher's own identity
// (MimoMusic, www.mimomusic.art).

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Limiter"
#define DISTRHO_PLUGIN_LABEL    "limiter"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/limiter"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.limiter"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo limiter with lookahead and a hard output ceiling: " \
    "threshold, attack, release, lookahead, knee, input/output gain and " \
    "a Boost switch that adds the threshold's reduction back. Two " \
    "algorithms: Classic (an envelope-follower limiter) and True Peak " \
    "(a sample-accurate lookahead limiter that never lets the output " \
    "exceed the threshold). Detection is always stereo-linked."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from the sibling compressor plugins' CMmo/CMst.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID MLim

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    1 // kParamLookahead adds real delay-buffer latency -- hosts need to know, see PluginLimiter.cpp's setLatency() call
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1120
#define DISTRHO_UI_DEFAULT_HEIGHT      720

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
