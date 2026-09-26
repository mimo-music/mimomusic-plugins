// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Loudness Meter
//
// TODO before any real release: DISTRHO_PLUGIN_URI must be a URI you
// actually control (LV2 identity requirement) -- matches every other
// plugin in the mimomusic family, see ../compressor's own identical TODO.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Loudness Meter"
#define DISTRHO_PLUGIN_LABEL    "loudness_meter"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/loudness-meter"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.loudness-meter"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo measurement plugin: standards-based (ITU-R BS.1770-4 / EBU R128 " \
    "/ EBU Tech 3342) loudness metering -- Momentary, Short-term and " \
    "Integrated Loudness (LUFS), Loudness Range (LRA), True Peak (dBTP) per " \
    "channel, and the Peak-to-Loudness/Peak-to-Short-term ratios (PLR/PSR) " \
    "those combine into, with a scrolling history graph and a selectable " \
    "target reference line. Purely observational -- the audio signal passes " \
    "through unmodified."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID LdMt

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
#define DISTRHO_UI_DEFAULT_WIDTH       1100
#define DISTRHO_UI_DEFAULT_HEIGHT      700

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
