// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- StateSpace Dynamic EQ
//
// Same TODO as every sibling copy: DISTRHO_PLUGIN_URI must be a URI you
// actually control before any real release -- the brand/URI below are the
// publisher's own identity (MimoMusic, www.mimomusic.art).
// This is a *separate* plugin from StateSpace MIMO/EQ/EQ Mono/Multiband
// Compressor (own binary, own LV2/VST3 identity) built from the same DSP
// core -- see docs/ARCHITECTURE.md's "Dynamic EQ" section and
// docs/EXTENDING.md's "Dynamic EQ" section for the mechanism this plugin is
// a GUI on top of.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Dynamic EQ"
#define DISTRHO_PLUGIN_LABEL    "dynamic_eq"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/statespace-dyn-eq"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.statespace-dyn-eq"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "Dynamic EQ: bell bands that also carry their own threshold, ratio, " \
    "up ratio, range, attack and release, so each band's boost or cut " \
    "moves automatically when its own frequency region gets loud (or " \
    "quiet), then relaxes back. Drag a node to place a band and watch " \
    "its curve move live with the music; per-band Left/Right routing " \
    "and an Audition preview of the selected band."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from the sibling plugins' SSmm/SSeq/SSem/SSmb so hosts never
// confuse any of them.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID SSde

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
