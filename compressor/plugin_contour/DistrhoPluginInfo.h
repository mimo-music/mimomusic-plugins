// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Contour
//
// See plugin_mono/DistrhoPluginInfo.h's header comment -- same TODO before
// any real release applies here too. Kept in its own directory (rather than
// sharing plugin_stereo/'s) so this plugin's identity can never be
// ambiguously resolved by include order, matching every sibling plugin's
// own convention in this project.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Contour"
#define DISTRHO_PLUGIN_LABEL    "contour"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/contour"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.contour"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo dynamics processor with a freely drawn input/output curve " \
    "instead of a ratio/knee formula, plus a threshold with an " \
    "Above/Below trigger that switches the sound between the curve and " \
    "the unchanged diagonal -- one engine for downward and upward " \
    "compression, limiting, gates and expanders. Linked, Dual-Mono or " \
    "Mid-Side processing, an external sidechain input, and trigger " \
    "gain, band filter and lookahead."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID CCtr

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
// 4 inputs: the main L/R pair (indices 0/1) plus a stereo external
// sidechain/key input (indices 2/3, flagged kAudioPortIsSidechain in
// PluginContour::initAudioPort()) -- identical shape to
// PluginCompressorStereo's own, see that plugin's kParamSidechainEnable
// comment for the full design this one reuses unchanged.
#define DISTRHO_PLUGIN_NUM_INPUTS      4
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    1
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1120
#define DISTRHO_UI_DEFAULT_HEIGHT      720

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
