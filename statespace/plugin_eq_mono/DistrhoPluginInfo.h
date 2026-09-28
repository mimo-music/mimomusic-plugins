// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- StateSpace EQ Mono (the mono sibling of StateSpace EQ)
//
// Same TODO as the other two plugins' own copies: DISTRHO_PLUGIN_URI must
// be a URI you actually control before any real release -- the brand/URI
// below are the publisher's own identity (MimoMusic, www.mimomusic.art).
// A third, independent plugin identity (own
// binary, own LV2/VST3/CLAP IDs) built on the same shared DSP core, so
// hosts never confuse it with either "StateSpace MIMO" or "StateSpace EQ"
// -- see docs/ARCHITECTURE.md's "EqModel / EQ plugin" section (the "why a
// separate plugin, not a mode switch" reasoning applies identically here:
// a mono/stereo channel count is a plugin-instantiation-time property in
// every host, not something that toggles live).
//
// The one deliberate difference from plugin_eq/DistrhoPluginInfo.h:
// NUM_INPUTS/NUM_OUTPUTS are 1, not 2 -- this is a genuinely mono-native
// plugin (its own MonoStateSpaceEngine, half the per-band state count of
// the stereo one, see plugin/dsp/MonoStateSpaceTypes.hpp), meant for
// tracks/busses that only ever carry a mono signal, to avoid paying for
// stereo processing that would just duplicate the same result on both
// channels.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Parametric EQ Mono"
#define DISTRHO_PLUGIN_LABEL    "parametric_eq_mono"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/statespace-eq-mono"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.statespace-eq-mono"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "Mono Parametric EQ with up to 32 bands -- the same editor and " \
    "state-space core as Parametric EQ, but natively single-channel: " \
    "half the filter states per band, so it costs a fraction of the " \
    "stereo EQ's CPU on mono tracks and busses. The result can be " \
    "exported as a .ssjson preset (both channels identical) to inspect " \
    "or edit its coefficients in Matrix Filter."

// Distinct from both siblings' own 4-character codes (Mimo/SSmm,
// Mimo/SSeq) so hosts never confuse any of the three.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID SSem

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      1
#define DISTRHO_PLUGIN_NUM_OUTPUTS     1
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
