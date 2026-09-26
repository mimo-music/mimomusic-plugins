// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Contour Mono
//
// The mono sibling of plugin_contour/: NOT a copy of it. The very same sources (plugin_contour/PluginContour.*,
// gui/UIContour.*, every widget, plugin/dsp/*) are compiled a second time with CONTOUR_MONO defined here, and
// the few mono-specific spots are `if (kContourMono)` / `#if CONTOUR_MONO` inside those shared files (see the
// comment at the top of plugin_contour/PluginContour.hpp). A feature added to Contour therefore reaches Contour
// Mono automatically; only a change to what this file declares (identity, ports) is mono-only.
// Kept in its own directory, like every plugin here, so its identity can never be ambiguously resolved by
// include order (this directory must come BEFORE plugin_contour/ on the include path -- see CMakeLists.txt).

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define CONTOUR_MONO 1

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Contour Mono"
#define DISTRHO_PLUGIN_LABEL    "contour_mono"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/contour_mono"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.contour-mono"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "The mono version of Contour: a 1-in/1-out dynamics processor with a " \
    "freely-drawn input/output curve, an independent threshold and an " \
    "Above/Below trigger condition, an external sidechain input, trigger " \
    "gain/filter/lookahead -- the very same engine as Contour, without the " \
    "stereo modes."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID CCtM

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
// 2 inputs: the main signal (index 0) and a mono external sidechain/key input (index 1, flagged
// kAudioPortIsSidechain in PluginContour::initAudioPort()); 1 output.
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     1
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
