// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- StateSpace EQ (the sibling parametric-EQ plugin)
//
// Same TODO as the base plugin's own copy: DISTRHO_PLUGIN_URI must be a
// URI you actually control before any real release -- the brand/URI below
// are the publisher's own identity (MimoMusic, www.mimomusic.art). This is a *separate* plugin from "StateSpace MIMO"
// (own binary, own LV2/VST3 identity) built from the same DSP core --
// see docs/ARCHITECTURE.md's "EqModel / EQ plugin" section for why it's a
// second plugin rather than a mode of the first.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Parametric EQ"
#define DISTRHO_PLUGIN_LABEL    "parametric_eq"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/statespace-eq"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.statespace-eq"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "Parametric EQ with an unbounded band count -- click to add a band " \
    "directly on the frequency graph, drag to shape it. Built on the same " \
    "state-space core as Matrix Filter: every band is realized as a " \
    "stability-guarded biquad and the assembled result can be exported " \
    "as a .ssjson preset to inspect or hand-edit its raw coefficients " \
    "there."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from the base plugin's SSmm/Mtrc so hosts never confuse the two.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID SSeq

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
