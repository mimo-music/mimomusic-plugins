// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Compressor Mono
//
// DPF build-time plugin identity/capability metadata. TODO before any real
// release: DISTRHO_PLUGIN_URI must be a URI you actually control (LV2
// identity) -- the brand/URI below are the publisher's own identity
// (MimoMusic, www.mimomusic.art), matching the
// convention already used in ../basic_statespace_filter_plugin.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Compressor Mono"
#define DISTRHO_PLUGIN_LABEL    "compressor_mono"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/compressor-mono"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.compressor-mono"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A 1-in/1-out feed-forward dynamics compressor (plus an external " \
    "sidechain key input): input/output gain, threshold, ratio, attack, " \
    "release, soft knee, and an optional measured volume-compensation " \
    "stage that keeps perceived loudness constant while you shape the " \
    "compression."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID CMmo

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
// 2 inputs: the main signal (index 0) plus an external sidechain/key input
// (index 1, flagged kAudioPortIsSidechain in PluginCompressorMono::
// initAudioPort()) -- see kParamSidechainEnable's own comment.
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     1
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       980
#define DISTRHO_UI_DEFAULT_HEIGHT      680

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
