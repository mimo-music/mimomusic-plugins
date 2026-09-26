// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- StateSpace Multiband Compressor
//
// Same TODO as every sibling copy: DISTRHO_PLUGIN_URI must be a URI you
// actually control before any real release -- the brand/URI below are the
// publisher's own identity (MimoMusic, www.mimomusic.art). This is a *separate* plugin from StateSpace MIMO/EQ/EQ
// Mono (own binary, own LV2/VST3 identity) built from the same DSP core --
// see docs/ARCHITECTURE.md's "MultibandSchedule" section and
// docs/EXTENDING.md's "Multiband compression" section for the mechanism
// this plugin is a GUI on top of.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Multiband Compressor"
#define DISTRHO_PLUGIN_LABEL    "multiband_compressor"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/statespace-mb-compressor"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.statespace-mb-compressor"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "Multiband compressor with a user-configurable band count -- drag " \
    "crossover splits directly on the frequency graph, select a band and " \
    "shape its threshold/ratio/attack/release/makeup. Built on the same " \
    "state-space core as Matrix Filter: a Linkwitz-Riley crossover bank " \
    "plus a per-band state-dependent gain stage, stability-guarded like " \
    "every other model in this project, exportable as a .ssjson preset " \
    "to inspect or hand-edit its raw coefficients there."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from the sibling plugins' SSmm/SSeq/SSem so hosts never confuse
// any of them.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID SSmb

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
