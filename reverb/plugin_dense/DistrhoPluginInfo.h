// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Reverb Dense
//
// The 16-line FDN model -- one of several reverb plugins with the same
// interface but a different sound, so a user can choose by sound and by
// CPU cost. Same GUI (gui/UIReverb.*, unchanged, shared via
// ../plugin/ReverbParams.hpp) as ../plugin_plate's own "Reverb Plate",
// built against plugin/dsp/ReverbEngineDense.hpp's own 16-line engine --
// denser mode spacing, smoother/less "grainy" tail. See
// plugin/dsp/ReverbEngineDense.hpp's own header comment for the full
// rationale. Originally shipped alongside an 8-line sibling ("Reverb",
// `plugin_stereo/`, itself originally called "reverb_smooth" before this
// rename) -- that 8-line plugin was removed once this one and Reverb
// Plate both sounded clearly better.
//
// TODO before any real release: DISTRHO_PLUGIN_URI must be a URI you
// actually control -- matches this repository's other MimoMusic plugins'
// own convention (see e.g. ../plugin_plate/DistrhoPluginInfo.h), not a
// placeholder invented for this file alone.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Reverb Dense"
#define DISTRHO_PLUGIN_LABEL    "reverb_dense"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/reverb_dense"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.reverb_dense"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo reverb built as a 16-line Feedback Delay Network -- dense " \
    "mode spacing for a smooth, grain-free tail. Same room-dimension/" \
    "Decay/Damping/Diffusion/Modulation/Spread/Balance controls and " \
    "tempo-synced decay as Reverb Plate -- pick whichever character " \
    "suits the material. See docs/REVERB_CONCEPT.md for the full design."

// 4-character code packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from every sibling MimoMusic plugin's own code (SSmm/SSeq/SSem/
// SSmb/SSde in ../../statespace, plus ../../compressor's/../../amplifier's
// own sets, plus MmRp for ../plugin_plate's own "Reverb Plate").
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID MmRd

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    0
// Required for Plugin::getTimePosition() to actually be populated (tempo
// sync) -- see ../plugin_plate/DistrhoPluginInfo.h's identical need.
#define DISTRHO_PLUGIN_WANT_TIMEPOS    1

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1100 // matches ../plugin_plate -- same shared GUI, same listW=400 EQ band list
#define DISTRHO_UI_DEFAULT_HEIGHT      760

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
