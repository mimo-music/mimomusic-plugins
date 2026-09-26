// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Amplifier Stereo
//
// See ../plugin_mono/DistrhoPluginInfo.h's identical comments for the URI
// TODO and the "why this now has a custom GUI" reasoning -- both apply
// unchanged here. Own DistrhoPluginInfo.h in its own directory (not
// plugin_mono/'s) so the two plugins' identities can never be ambiguously
// resolved by include order, matching every sibling plugin's own
// convention in both other projects.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Amplifier Stereo"
#define DISTRHO_PLUGIN_LABEL    "amplifier_stereo"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/amplifier-stereo"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.amplifier-stereo"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A 2-in/2-out static-gain amplifier: Gain (dB), independent Invert " \
    "L/Invert R polarity switches, a Balance control (attenuates the " \
    "far channel only, never boosts), Bypass, and a Listen/Normalize " \
    "feature -- audition the raw input, measure its Peak or RMS level " \
    "for as long as you like, then stop to have Gain set automatically " \
    "so that level lands on your chosen target."

// 4-character code packed into a 32-bit id, used by VST2/AU-style hosts --
// distinct from every sibling plugin's own code in both other projects
// and from this project's own Mono plugin (AMmo).
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID AMst

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      0
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        0
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       420
#define DISTRHO_UI_DEFAULT_HEIGHT      220

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
