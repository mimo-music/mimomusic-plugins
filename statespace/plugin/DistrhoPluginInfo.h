// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h
//
// DPF build-time plugin identity/capability metadata. TODO before any real
// release: DISTRHO_PLUGIN_URI must be a URI you actually control (LV2
// identity) -- the values below are the publisher's own identity
// (MimoMusic, www.mimomusic.art) so the plugin builds and loads
// distinctly from other DPF examples/plugins on the system.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Matrix Filter"
#define DISTRHO_PLUGIN_LABEL    "matrix_filter"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/statespace-mimo"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.statespace-mimo"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "Generic 2x2 MIMO discrete-time state-space filter. Every coefficient of " \
    "the A/B/C/D matrices is directly editable, with automatic stability " \
    "enforcement, so it can be configured to represent essentially any " \
    "linear stereo filter/EQ/crossover topology."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID SSmm

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1180
#define DISTRHO_UI_DEFAULT_HEIGHT      760

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
