# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH

# CMake build for the mimomusic measurement plugins, using the DISTRHO
# Plugin Framework (DPF, vendored as a git submodule in dpf/, the exact
# same commit ../compressor and ../statespace vendor) -- same toolkit,
# project layout and build conventions as those two sibling projects, so
# all mimomusic plugins stay consistent to work in side by side.
# "Measurement" plugins are their own sibling project because they never
# change the audio, unlike everything in those two. "au" is listed in TARGETS
# below unconditionally -- see this file's own note further down for why
# that's a no-op everywhere except a native macOS build
# (build_and_install_macos.sh).
#
#   - phase_scope: 2-in/2-out, purely observational (passes audio through
#                  unmodified) -- a rotated Mid/Side goniometer, a phase
#                  correlation meter with a recovering worst-case hold, a
#                  stereo width meter, and L/R peak/RMS bargraphs.
#   - loudness_meter: 2-in/2-out, purely observational -- standards-based
#                  (ITU-R BS.1770-4 / EBU R128 / EBU Tech 3342) stereo
#                  loudness metering: Momentary/Short-term/Integrated
#                  Loudness (LUFS), Loudness Range (LRA), True Peak
#                  (dBTP), PLR/PSR, a scrolling history graph and a
#                  selectable target reference line.
#
# Also builds a dependency-free command-line tool (see docs/TESTING.md):
#   - dsp_regression: fast, pure-math correctness checks (no audio files,
#                     no DPF/GUI dependency)
#
#   git submodule update --init --recursive   # first time only
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   ./build/bin/dsp_regression
#
# Plugin output lands under build/bin/: an .lv2 bundle, a .vst3 bundle,
# and a standalone JACK executable for "phase_scope".
#
# ./build_and_install.sh builds all of this and copies the .lv2 bundle
# into ~/.lv2/ in one step.

cmake_minimum_required(VERSION 3.15)
project(mimomusic_meter_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(dpf)

# The "jack" TARGETS entry builds a standalone app -- via a real JACK
# client if libjack is available, otherwise via DPF's SDL2-based fallback
# bridge. That fallback trips a long-standing SDL2 bug on Apple Silicon
# (SDL_cpuinfo.h's `__has_include(<immintrin.h>)` feature-detection isn't
# gated by target architecture) -- see ../compressor/CMakeLists.txt's
# identical, fuller comment. Skip "jack" on Apple rather than fight it;
# every other platform keeps it.
if(APPLE)
  set(METERS_EXTRA_TARGETS "")
else()
  set(METERS_EXTRA_TARGETS jack)
endif()

dpf_add_plugin(phase_scope
  TARGETS lv2 vst3 au ${METERS_EXTRA_TARGETS}
  FILES_DSP
      plugin_phasescope/PluginPhaseScope.cpp
  FILES_UI
      gui/UIPhaseScope.cpp
      gui/GoniometerWidget.cpp
      gui/StereoMeterWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
)

target_include_directories(phase_scope PUBLIC
  plugin_phasescope
  gui
)

dpf_add_plugin(loudness_meter
  TARGETS lv2 vst3 au ${METERS_EXTRA_TARGETS}
  FILES_DSP
      plugin_loudness/PluginLoudnessMeter.cpp
  FILES_UI
      gui/UILoudnessMeter.cpp
      gui/LoudnessMeterWidget.cpp
      gui/LevelMeterWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
)

target_include_directories(loudness_meter PUBLIC
  plugin_loudness
  gui
)

# ---------------------------------------------------------------------
# Verification tool -- plain executable, no DPF/DGL dependency at all,
# just the framework-independent DSP core (see docs/TESTING.md).
# ---------------------------------------------------------------------

add_executable(dsp_regression tests/dsp_regression.cpp)
target_include_directories(dsp_regression PRIVATE plugin/dsp)

set_target_properties(dsp_regression PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
