// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// DistrhoPluginInfo.h -- MimoMusic Limiter
//
// A stereo limiter on this project's DSP core. Two gain computers
// (LimiterAlgorithm, PluginLimiter.hpp): Classic, a CompressorVoice
// (CompressorDsp.hpp) at a high ratio (20:1, 100:1 or infinity:1), and True
// Peak, a sample-accurate lookahead limiter (TruePeakLimiterDsp.hpp) that
// never lets a sample over the Threshold. Around them (LimiterDsp.hpp):
//
//   - Gain staging: Output Gain and Boost come after the gain computer and
//     are capped so the limited peaks land on the Ceiling instead of being
//     clipped by it (computeLimiterStaging()).
//   - Clip stage for overshoot that still reaches the Ceiling: Hard or Soft
//     with an adjustable Clip Knee. Switching the Algorithm sets the
//     matching default (True Peak -> Hard, Classic -> Soft); the user can
//     change it afterwards.
//   - Optional 4x oversampled peak detection, so inter-sample peaks are
//     limited too. Off costs nothing extra.
//
// Lookahead delays the audio; the delay is reported as latency (also while
// bypassed). Detection is always stereo-linked: a per-channel or Mid/Side
// limiter on a stereo bus would move the image.
//
// The brand/URI below are the publisher's own identity (MimoMusic,
// www.mimomusic.art). Keep URI and unique ID unchanged: hosts identify
// saved sessions by them.

#ifndef DISTRHO_PLUGIN_INFO_H_INCLUDED
#define DISTRHO_PLUGIN_INFO_H_INCLUDED

#define DISTRHO_PLUGIN_BRAND    "MimoMusic"
#define DISTRHO_PLUGIN_NAME     "Limiter"
#define DISTRHO_PLUGIN_LABEL    "limiter"
#define DISTRHO_PLUGIN_MAKER    "MimoMusic"
#define DISTRHO_PLUGIN_LICENSE  "GPL-3.0-or-later"
#define DISTRHO_PLUGIN_URI      "https://www.mimomusic.art/plugins/limiter"
#define DISTRHO_PLUGIN_CLAP_ID  "art.mimomusic.limiter"
#define DISTRHO_PLUGIN_DESCRIPTION \
    "A stereo limiter with lookahead whose output never exceeds the " \
    "ceiling: Output Gain and Boost are capped so the limited peaks land " \
    "on the ceiling instead of being clipped. Hard or soft output clipping " \
    "for overshoot, optional 4x oversampled (inter-sample) peak detection. " \
    "Two algorithms: Classic (an envelope-follower limiter, ratio 20:1 to " \
    "infinity:1) and True Peak (a sample-accurate lookahead limiter). " \
    "Detection is always stereo-linked."

// 4-character codes packed into a 32-bit id, used by VST2/AU-style hosts.
// Distinct from the sibling compressor plugins' CMmo/CMst.
#define DISTRHO_PLUGIN_BRAND_ID Mimo
#define DISTRHO_PLUGIN_UNIQUE_ID MLim

#define DISTRHO_PLUGIN_HAS_UI          1
#define DISTRHO_PLUGIN_IS_RT_SAFE      1
#define DISTRHO_PLUGIN_NUM_INPUTS      2
#define DISTRHO_PLUGIN_NUM_OUTPUTS     2
#define DISTRHO_PLUGIN_WANT_STATE      1
#define DISTRHO_PLUGIN_WANT_PROGRAMS   0
#define DISTRHO_PLUGIN_WANT_LATENCY    1 // Lookahead (and Oversampling) delay the audio -- see PluginLimiter::updateLookahead()
#define DISTRHO_PLUGIN_WANT_TIMEPOS    0

#define DISTRHO_UI_USE_NANOVG          1
#define DISTRHO_UI_FILE_BROWSER        1
#define DISTRHO_UI_USER_RESIZABLE      1
#define DISTRHO_UI_DEFAULT_WIDTH       1120
#define DISTRHO_UI_DEFAULT_HEIGHT      720

#endif // DISTRHO_PLUGIN_INFO_H_INCLUDED
