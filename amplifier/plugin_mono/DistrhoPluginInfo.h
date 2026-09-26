// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Amplifier Mono
//
// DPF build-time plugin identity/capability metadata. TODO before any real
// release: DISTRHO_PLUGIN_URI must be a URI you actually control (LV2
// identity) -- the brand/URI below are the publisher's own identity
// (MimoMusic, www.mimomusic.art), matching the
// convention already used in ../basic_statespace_filter_plugin and
// ../compressor.
//
// DISTRHO_PLUGIN_HAS_UI started at 0 (no custom GUI at all) and was
// revised to 1 once the Listen/Normalize feature needed a live level
// readout and a start/stop control no host's own generic parameter editor
// can give it. The GUI
// (gui/UIAmplifierMono.*) reuses SimpleButton/ParamStripWidget ported
// verbatim from ../compressor, same as every custom-GUI plugin in this
// project family reuses widgets across projects.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Amplifier Mono"
#define DISTRHO_PLUGIN_LABEL    "amplifier_mono"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/amplifier-mono"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.amplifier-mono"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A 1-in/1-out static-gain amplifier: Gain (dB) and Bypass, plus a " \
    "Listen/Normalize feature -- audition the raw input, measure its " \
    "Peak or RMS level for as long as you like, then stop to have Gain " \
    "set automatically so that level lands on your chosen target."

// 4-character code packed into a 32-bit id, used by VST2/AU-style hosts --
// distinct from every sibling plugin's own code in both other projects.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID AMmo

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      1
#define DISTRHO_PLUGIN_NUM_OUTPUTS     1
#define DISTRHO_PLUGIN_WANT_STATE      0
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        0
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       360
#define DISTRHO_UI_DEFAULT_HEIGHT      220

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
