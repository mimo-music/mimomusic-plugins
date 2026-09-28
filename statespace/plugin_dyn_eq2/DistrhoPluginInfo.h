// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- StateSpace Dynamic EQ 2
//
// A hand-adapted, fully independent CLONE of `plugin_dyn_eq/` (the
// original "Dynamic EQ") -- own DSP core (plugin/dsp/DynamicEqModel2.hpp/
// .cpp, plugin/dsp/DynamicEqEngine2.hpp), own GUI (gui/dyneq2/), own
// binary/LV2/VST3 identity, sharing nothing mutable with the original.
// A "version 2": a safe place to grow new features that bring it closer
// to the Parametric EQ (crossfeed bands, harmonic linking, shelf and
// band-shelf types, ...) without putting the original -- already used in
// existing projects -- at risk; the two may be merged later.
//
// Same TODO as every sibling copy: DISTRHO_PLUGIN_URI must be a URI you
// actually control before any real release -- the brand/URI below are the
// publisher's own identity (MimoMusic, www.mimomusic.art).

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Dynamic EQ 2"
#define DISTRHO_PLUGIN_LABEL    "dynamic_eq2"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/statespace-dyn-eq2"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.statespace-dyn-eq2"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "Dynamic EQ 2: the Dynamic EQ with Bell, Low Shelf, High Shelf and " \
    "Band Shelf bands, harmonic band linking (linked bands move and " \
    "react together) and stereo crossfeed bands. Each band carries its " \
    "own threshold, ratio, up ratio, range, attack and release, so its " \
    "boost or cut moves automatically when its own frequency region " \
    "gets loud (or quiet)."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from the sibling plugins' SSmm/SSeq/SSem/SSmb so hosts never
// confuse any of them.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID SSd2

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
