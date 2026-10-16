# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH

# CMake build for three sibling plugins sharing one DSP core (LV2 + VST3 +
# a standalone JACK build each, for quick manual testing, plus AU -- Audio
# Unit -- on macOS), using the DISTRHO Plugin Framework (DPF, vendored as a
# git submodule in dpf/) -- same toolkit, project layout and build
# conventions as the sibling project ../statespace, so
# both projects stay consistent to work in side by side. It is a separate
# project because a compressor's level-dependent gain can't be built on
# that project's linear state-space core. "au" is
# listed in every plugin's own TARGETS below unconditionally -- see that
# project's own CMakeLists.txt comment for why this is a no-op everywhere
# except a native macOS build (build_and_install_macos.sh).
#
#   - compressor_mono:   1-in/1-out feed-forward compressor
#   - compressor_stereo: 2-in/2-out, with a Linked/Dual-Mono/
#                                  Mid-Side stereo mode switch
#   - limiter:           2-in/2-out, Classic (the compressor core at
#                                  a high ratio) or True Peak gain computer,
#                                  Ceiling-capped output gain, Hard/Soft clip,
#                                  optional 4x oversampled peak detection --
#                                  see plugin_limiter/DistrhoPluginInfo.h.
#   - contour_mono:      the mono version of contour -- the same source files
#                                  built again with CONTOUR_MONO (see below)
#   - contour:           2-in/2-out, a freely-drawn input/output curve
#                                  (plugin/dsp/CurveDsp.hpp) instead of a
#                                  ratio/knee formula, plus an independent
#                                  threshold and an Above/Below trigger
#                                  condition -- one mechanism that covers
#                                  downward/upward compression, limiting,
#                                  and gating, all from the same drawn
#                                  shape. See plugin_contour/
#                                  DistrhoPluginInfo.h's own comment.
#
# Also builds two dependency-free command-line tools (see docs/TESTING.md):
#   - dsp_regression:  fast, pure-math correctness checks (no audio files)
#   - audio_testbench: runs synthetic audio through the real per-sample DSP
#                       chain (envelope, gain computer, meters, compensation,
#                       stereo modes), writes WAV files for listening
# Neither has a DPF/GUI dependency at all.
#
#   git submodule update --init --recursive   # first time only
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   ./build/bin/dsp_regression
#   ./build/bin/audio_testbench
#
# Plugin outputs land under build/bin/: an .lv2 bundle, a .vst3 bundle, and
# a standalone JACK executable for each of "compressor_mono",
# "compressor_stereo" and "limiter".
#
# ./build_and_install.sh builds all of this and copies every .lv2 bundle
# into ~/.lv2/ in one step.

cmake_minimum_required(VERSION 3.15)
project(mimomusic_compressor_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(dpf)

# The "jack" TARGETS entry builds a standalone app -- via a real JACK
# client if libjack is available, otherwise via DPF's SDL2-based fallback
# bridge (dpf/distrho/src/jackbridge/SDL2Bridge.hpp). That fallback trips a
# long-standing SDL2 bug on Apple Silicon: SDL_cpuinfo.h's feature-detection
# uses `__has_include(<immintrin.h>)`, which isn't gated by target
# architecture (the header exists in the toolchain regardless of -arch), so
# compiling the arm64 slice of a universal binary drags in x86-only SIMD
# intrinsic headers and fails outright immintrin.h's own "x86 and x64 only"
# #error. None of the three plugins need this standalone build to work in
# Ardour (LV2/VST3) or other macOS hosts (AU), so skip "jack" on Apple
# rather than fight SDL2's bug; every other platform keeps it. See the
# sibling ../statespace/CMakeLists.txt's identical note.
if(APPLE)
  set(COMPRESSOR_EXTRA_TARGETS "")
else()
  set(COMPRESSOR_EXTRA_TARGETS jack)
endif()

dpf_add_plugin(compressor_mono
  TARGETS lv2 vst3 au ${COMPRESSOR_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_mono/PluginCompressorMono.cpp
  FILES_UI
      gui/UICompressorMono.cpp
      gui/ParamStripWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/CompressorGraphWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
)

target_include_directories(compressor_mono PUBLIC
  plugin_mono
  gui
)

# ---------------------------------------------------------------------
# compressor_stereo -- own DistrhoPluginInfo.h in plugin_stereo/
# (a separate directory from plugin_mono/'s) so the two plugins' identities
# can never be ambiguously resolved by include order, matching the
# plugin/ vs plugin_eq/ convention in ../statespace.
# ---------------------------------------------------------------------

dpf_add_plugin(compressor_stereo
  TARGETS lv2 vst3 au ${COMPRESSOR_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_stereo/PluginCompressorStereo.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/UICompressorStereo.cpp
      gui/ParamStripWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/CompressorGraphWidget.cpp
      gui/SimpleButton.cpp
      gui/SidechainScopeWidget.cpp
      gui/TooltipOverlay.cpp
)

target_include_directories(compressor_stereo PUBLIC
  plugin_stereo
  gui
)

# ---------------------------------------------------------------------
# limiter -- own DistrhoPluginInfo.h in plugin_limiter/, same
# include-ambiguity reason as plugin_stereo/'s own. Reuses two of the
# compressor plugins' GUI widgets as-is (LevelMeterWidget, ParamStripWidget,
# SimpleButton) but has its own, smaller graph widget (LimiterGraphWidget,
# not CompressorGraphWidget) -- see gui/UILimiter.cpp's and
# LimiterGraphWidget.hpp's own file comments for why.
# ---------------------------------------------------------------------

dpf_add_plugin(limiter
  TARGETS lv2 vst3 au ${COMPRESSOR_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_limiter/PluginLimiter.cpp
  FILES_UI
      gui/UILimiter.cpp
      gui/ParamStripWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/LimiterGraphWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
)

target_include_directories(limiter PUBLIC
  plugin_limiter
  gui
)

# ---------------------------------------------------------------------
# contour -- own DistrhoPluginInfo.h in plugin_contour/, same
# include-ambiguity reason as every sibling plugin's own. Reuses
# LevelMeterWidget/ParamStripWidget/SimpleButton as-is, but has its own
# graph widget (ContourGraphWidget, not CompressorGraphWidget) for the
# curve-editing interaction -- see plugin/dsp/CurveDsp.hpp's and
# ContourGraphWidget.hpp's own file comments for the full design.
# ---------------------------------------------------------------------

dpf_add_plugin(contour
  TARGETS lv2 vst3 au ${COMPRESSOR_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_contour/PluginContour.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/UIContour.cpp
      gui/ParamStripWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/ContourGraphWidget.cpp
      gui/NodeListWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      gui/SidechainScopeWidget.cpp
)

target_include_directories(contour PUBLIC
  plugin_contour
  gui
)

# ---------------------------------------------------------------------
# contour_mono -- the mono version of contour: the SAME source files
# (plugin_contour/PluginContour.cpp, gui/UIContour.cpp, every widget),
# compiled a second time with CONTOUR_MONO defined by its own
# plugin_contour_mono/DistrhoPluginInfo.h. plugin_contour_mono MUST come
# before any other directory holding a DistrhoPluginInfo.h on the include
# path, and plugin_contour/ is deliberately NOT on it -- otherwise the
# stereo plugin's info header could be picked instead. Any feature added to
# contour is thereby in contour_mono too; mono-specific behaviour lives in
# `if (kContourMono)` spots inside the shared files (see the comment in
# plugin_contour/PluginContour.hpp).
# ---------------------------------------------------------------------

dpf_add_plugin(contour_mono
  TARGETS lv2 vst3 au ${COMPRESSOR_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_contour/PluginContour.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/UIContour.cpp
      gui/ParamStripWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/ContourGraphWidget.cpp
      gui/NodeListWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      gui/SidechainScopeWidget.cpp
)

target_include_directories(contour_mono PUBLIC
  plugin_contour_mono
  gui
)

# ---------------------------------------------------------------------
# Verification tool -- plain executable, no DPF/DGL dependency at all,
# just the framework-independent DSP core (see docs/TESTING.md).
# ---------------------------------------------------------------------

add_executable(dsp_regression tests/dsp_regression.cpp)
target_include_directories(dsp_regression PRIVATE plugin/dsp)

add_executable(audio_testbench tools/testbench/audio_testbench.cpp)
target_include_directories(audio_testbench PRIVATE plugin/dsp)
if(APPLE)
  # std::filesystem's symbols are marked unavailable by Apple's libc++
  # headers when the deployment target is below macOS 10.15 -- the
  # filesystem TS runtime bits weren't in the system dylib before that.
  # audio_testbench is a standalone dev tool (not shipped with the plugin,
  # not installed), so raise its own target instead of the whole project's
  # -- the actual plugin/DGL/DPF targets keep building down to whatever
  # --min-macos was requested (10.13 by default in
  # build_and_install_macos.sh). See the sibling
  # ../statespace/CMakeLists.txt's identical comment.
  target_compile_options(audio_testbench PRIVATE -mmacosx-version-min=10.15)
  target_link_options(audio_testbench PRIVATE -mmacosx-version-min=10.15)
endif()

set_target_properties(dsp_regression audio_testbench PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
