// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Reverb Plate
//
// The third model -- added after the FDN-based "reverb_smooth" model
// (since renamed to Reverb Dense) still sounded like the same underlying
// technology, just denser. A genuinely different topology from Reverb Dense's own N-line
// Householder FDN: a Dattorro-style plate (input-diffusion allpass
// cascade feeding a figure-8 coupled pair of long delay loops, stereo
// built from multi-tap extraction rather than mixing N parallel lines)
// -- see plugin/dsp/ReverbEnginePlate.hpp's own header comment for the
// full architectural account and why this, not just another line-count
// change, was needed. An original third sibling, the 8-line "Reverb"
// (`plugin_stereo/`), was removed once this plugin and Reverb Dense both
// sounded clearly better.
//
// TODO before any real release: DISTRHO_PLUGIN_URI must be a URI you
// actually control -- matches this repository's other MimoMusic plugins'
// own convention (see e.g. ../plugin_dense/DistrhoPluginInfo.h), not a
// placeholder invented for this file alone.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Reverb Plate"
#define DISTRHO_PLUGIN_LABEL    "reverb_plate"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/reverb_plate"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.reverb_plate"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo plate reverb: a Dattorro-style figure-8 network of " \
    "coupled delays and all-passes. Room dimensions, pre-delay, decay " \
    "time (optionally synced to the host tempo), damping, diffusion, " \
    "modulation, spread, balance and dry/wet, plus a parametric EQ with " \
    "spectrum display on the reverb signal. Same controls as Reverb " \
    "Dense -- pick whichever character suits the material."

// 4-character code packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from every sibling MimoMusic plugin's own code (SSmm/SSeq/SSem/
// SSmb/SSde in ../../statespace, plus ../../compressor's/../../amplifier's
// own sets, plus MmRd for ../plugin_dense's own "Reverb Dense").
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID MmRp

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
#define DISTRHO_PLUGIN_WANT_TIMEPOS    1

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1100 // matches ../plugin_dense -- same shared GUI
#define DISTRHO_UI_DEFAULT_HEIGHT      760

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
