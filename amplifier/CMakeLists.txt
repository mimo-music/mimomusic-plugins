# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH

# CMake build for two sibling plugins sharing one tiny DSP core (LV2 +
# VST3 + a standalone JACK build each, for quick manual testing, plus AU --
# Audio Unit -- on macOS), using the DISTRHO Plugin Framework (DPF,
# vendored as a git submodule in dpf/) -- same toolkit, project layout and
# build conventions as the sibling projects ../statespace
# and ../compressor, so all three stay consistent to work in side by side.
# "au" is listed in every plugin's own TARGETS below unconditionally -- see
# either sibling project's own CMakeLists.txt comment for why this is a
# no-op everywhere except a native macOS build (build_and_install_macos.sh).
#
#   - amplifier_mono:   1-in/1-out static-gain amplifier (Gain,
#                                 Bypass, Listen/Normalize)
#   - amplifier_stereo: 2-in/2-out -- Gain, independent Invert L/
#                                 Invert R polarity switches, a Balance
#                                 control, Bypass, Listen/Normalize
#
# Both plugins have a small custom GUI (gui/UIAmplifier*.*, reusing
# SimpleButton/ParamStripWidget from ../compressor): Listen/Normalize needs
# a live level readout and a start/stop control no host's generic
# parameter editor can give it.
#
# Also builds one dependency-free command-line tool:
#   - dsp_regression: fast, pure-math correctness checks (no audio files,
#                     no DPF/GUI dependency)
#
#   git submodule update --init --recursive   # first time only
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   ./build/bin/dsp_regression
#
# Plugin outputs land under build/bin/: an .lv2 bundle, a .vst3 bundle, and
# a standalone JACK executable for each of "amplifier_mono" and
# "amplifier_stereo".
#
# ./build_and_install.sh builds all of this and copies every .lv2 bundle
# into ~/.lv2/ in one step.

cmake_minimum_required(VERSION 3.15)
project(mimomusic_amplifier_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(dpf)

# See either sibling project's own CMakeLists.txt identical comment: the
# SDL2-based standalone-JACK fallback trips a long-standing SDL2 bug on
# Apple Silicon, and neither plugin needs the standalone build to work in
# Ardour (LV2/VST3) or other macOS hosts (AU) -- skip "jack" on Apple,
# keep it everywhere else.
if(APPLE)
  set(AMPLIFIER_EXTRA_TARGETS "")
else()
  set(AMPLIFIER_EXTRA_TARGETS jack)
endif()

dpf_add_plugin(amplifier_mono
  TARGETS lv2 vst3 au ${AMPLIFIER_EXTRA_TARGETS}
  FILES_DSP
      plugin_mono/PluginAmplifierMono.cpp
  FILES_UI
      gui/UIAmplifierMono.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
)

target_include_directories(amplifier_mono PUBLIC
  plugin_mono
  gui
)

# ---------------------------------------------------------------------
# amplifier_stereo -- own DistrhoPluginInfo.h in plugin_stereo/
# (a separate directory from plugin_mono/'s) so the two plugins' identities
# can never be ambiguously resolved by include order, matching every
# sibling plugin's own convention in both other projects.
# ---------------------------------------------------------------------

dpf_add_plugin(amplifier_stereo
  TARGETS lv2 vst3 au ${AMPLIFIER_EXTRA_TARGETS}
  FILES_DSP
      plugin_stereo/PluginAmplifierStereo.cpp
  FILES_UI
      gui/UIAmplifierStereo.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
)

target_include_directories(amplifier_stereo PUBLIC
  plugin_stereo
  gui
)

# ---------------------------------------------------------------------
# Verification tool -- plain executable, no DPF/DGL dependency at all,
# just the framework-independent DSP core.
# ---------------------------------------------------------------------

add_executable(dsp_regression tests/dsp_regression.cpp)
target_include_directories(dsp_regression PRIVATE plugin/dsp)

set_target_properties(dsp_regression PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
