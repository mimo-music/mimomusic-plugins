// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Phase Scope
//
// TODO before any real release: DISTRHO_PLUGIN_URI must be a URI you
// actually control (LV2 identity requirement) -- matches every other
// plugin in the mimomusic family, see ../compressor's own identical TODO.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Phase Scope"
#define DISTRHO_PLUGIN_LABEL    "phase_scope"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/phase-scope"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.phase-scope"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo measurement plugin: a rotated Mid/Side goniometer (vectorscope, " \
    "dot or connected-line rendering, adjustable decay/persistence and an " \
    "optional auto-gain) for stereo image/spread, a phase correlation meter " \
    "with a recovering worst-case hold, a stereo width meter, and L/R " \
    "peak/RMS bargraphs. Purely observational -- the audio signal passes " \
    "through unmodified."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID PhSc

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       900
#define DISTRHO_UI_DEFAULT_HEIGHT      680

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
