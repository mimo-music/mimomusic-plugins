// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Compressor Stereo
//
// See plugin_mono/DistrhoPluginInfo.h's header comment -- same TODO before
// any real release applies here too. Kept in its own directory (rather than
// sharing plugin_mono/'s) so the two plugins' identities can never be
// ambiguously resolved by include order, matching the convention
// ../basic_statespace_filter_plugin uses for its plugin/ vs plugin_eq/.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Compressor Stereo"
#define DISTRHO_PLUGIN_LABEL    "compressor_stereo"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/compressor-stereo"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.compressor-stereo"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo feed-forward compressor with an external sidechain input " \
    "and Linked, Dual-Mono or Mid-Side processing: input gain, " \
    "threshold, ratio (compressing above or below the threshold), " \
    "attack, release, soft knee, makeup gain and wet/dry per channel, " \
    "plus a sidechain panel with gain, a band filter and a scope that " \
    "works on the external or the internal signal."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID CMst

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
// 4 inputs: the main L/R pair (indices 0/1) plus a stereo external
// sidechain/key input (indices 2/3, flagged kAudioPortIsSidechain in
// PluginCompressorStereo::initAudioPort()) -- see kParamSidechainEnable's
// own comment.
#define DISTRHO_PLUGIN_NUM_INPUTS      4
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1120
#define DISTRHO_UI_DEFAULT_HEIGHT      720

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
