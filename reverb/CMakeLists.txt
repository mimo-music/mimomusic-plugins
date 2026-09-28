# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH

# CMake build for the MimoMusic Reverb plugin (LV2 + VST3 + a standalone
# JACK build for quick manual testing, plus AU on macOS), using the
# DISTRHO Plugin Framework (DPF, a git submodule in dpf/, same revision as
# the sibling projects).
# Same toolkit/layout/build conventions as the sibling ../statespace and
# ../compressor projects, so all three stay consistent to work in side by
# side -- see docs/REVERB_CONCEPT.md for why this is its own top-level
# project rather than a plugin inside ../statespace.
#
#   - reverb_dense: 2-in/2-out Feedback Delay Network reverb (16 lines,
#             plugin/dsp/ReverbEngineDense.hpp).
#   - reverb_plate: 2-in/2-out Dattorro-style plate
#             (plugin/dsp/ReverbEnginePlate.hpp), a different topology.
#   Both: room size as Length/Width/Height, Decay Time/Damping,
#   Diffusion/Modulation, optional host-tempo-synced decay, a parametric
#   EQ on the wet signal (ported EqGraphWidget/BandListWidget driving an
#   ordinary StateSpaceEngine -- see docs/REVERB_CONCEPT.md's "EQ on the
#   reverb signal") and the shared GUI (gui/UIReverb.*). The shared engine
#   foundation (plugin/dsp/ReverbEngine.hpp: RoomDims, OnePoleDamping,
#   DuckState, ...) is #include'd by both engines and tested by
#   `reverb_regression` below.
#   Two real plugins, not a mode switch on one: the same interface with a
#   different sound, so each user can pick what they like, or what their
#   CPU budget allows.
#
# Also builds three dependency-free command-line tools (see docs/TESTING.md
# in ../statespace for the sibling convention this follows):
#   - reverb_regression: fast, pure-math correctness checks (no audio
#                         files, no DPF/GUI dependency at all) for the
#                         shared plugin/dsp/ReverbEngine.hpp foundation
#                         (see above) -- includes a real std::thread
#                         concurrency stress test for the room-size-change
#                         hand-off (see ReverbEngine.hpp's own "Room-size
#                         changes" section), hence the Threads dependency
#                         below.
#   - reverb_regression_dense: the identical suite, re-run against
#                         ReverbEngineDense -- see that file's own header
#                         comment for why this isn't assumed to just carry
#                         over from the shared foundation's own results.
#   - reverb_regression_plate: a mostly-new suite for ReverbEnginePlate's
#                         own different topology -- see that file's own
#                         header comment for why most checks aren't
#                         renumbered [R#]s.
#
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   ./build/bin/reverb_regression
#   ./build/bin/reverb_regression_dense
#   ./build/bin/reverb_regression_plate
#
# Plugin outputs land under build/bin/: an .lv2 bundle, a .vst3 bundle,
# and a standalone JACK executable, per target.
#
# Verified end to end: a full clean `cmake -B build && cmake --build build
# -j` builds both plugin targets (LV2/VST3/AU/JACK) and all three
# regression tools with zero warnings from any file this project owns
# (remaining warnings are pre-existing, unrelated DPF/vendored-library
# noise), all regression suites pass in full, and
# `./build_and_install.sh --test` installs both .lv2 bundles into ~/.lv2
# cleanly.

cmake_minimum_required(VERSION 3.15)
project(mimomusic_reverb_plugin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_subdirectory(dpf)

# See the sibling ../statespace/CMakeLists.txt's and
# ../compressor/CMakeLists.txt's identical note: DPF's SDL2-based JACK
# fallback trips a long-standing SDL2 bug on Apple Silicon
# (SDL_cpuinfo.h's `__has_include(<immintrin.h>)` feature-detection isn't
# gated by target architecture), and this plugin doesn't need the
# standalone JACK build to work in Ardour (LV2/VST3) or other macOS hosts
# (AU) -- skip "jack" on Apple rather than fight SDL2's bug.
if(APPLE)
  set(REVERB_EXTRA_TARGETS "")
else()
  set(REVERB_EXTRA_TARGETS jack)
endif()

# reverb_dense: same GUI/parameters (gui/UIReverb.*, ../plugin/ReverbParams.hpp),
# a 16-line FDN engine (plugin/dsp/ReverbEngineDense.hpp) instead of the
# 8-line one -- denser mode spacing/smoother tail at ~2x the DSP cost. See
# plugin/dsp/ReverbEngineDense.hpp's own header comment.
dpf_add_plugin(reverb_dense
  TARGETS lv2 vst3 au ${REVERB_EXTRA_TARGETS}
  FILES_DSP
      plugin_dense/PluginReverbDense.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/UIReverb.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      gui/eq/EqGraphWidget.cpp
      gui/eq/BandListWidget.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/StabilityGuard.cpp
)

target_include_directories(reverb_dense PUBLIC
  plugin_dense
  gui
)

# reverb_plate: same GUI/parameters again, this time a genuinely
# different topology (plugin/dsp/ReverbEnginePlate.hpp, a Dattorro-style
# figure-8 coupled delay network) rather than another line-count change.
# See that file's own header comment.
dpf_add_plugin(reverb_plate
  TARGETS lv2 vst3 au ${REVERB_EXTRA_TARGETS}
  FILES_DSP
      plugin_plate/PluginReverbPlate.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/UIReverb.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      gui/eq/EqGraphWidget.cpp
      gui/eq/BandListWidget.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/StabilityGuard.cpp
)

target_include_directories(reverb_plate PUBLIC
  plugin_plate
  gui
)

# ---------------------------------------------------------------------
# Verification tool -- plain executable, no DPF/DGL dependency at all,
# just the framework-independent DSP core (see docs/TESTING.md in
# ../statespace for the convention this follows).
# ---------------------------------------------------------------------

find_package(Threads REQUIRED)

add_executable(reverb_regression tests/reverb_regression.cpp)
target_include_directories(reverb_regression PRIVATE plugin/dsp)
target_link_libraries(reverb_regression PRIVATE Threads::Threads)

set_target_properties(reverb_regression PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Parallel suite for plugin/dsp/ReverbEngineDense.hpp's own 16-line
# "Dense" model -- see tests/reverb_regression_dense.cpp's own header
# comment for why this is a separate executable, not a runtime switch in
# the original.
add_executable(reverb_regression_dense tests/reverb_regression_dense.cpp)
target_include_directories(reverb_regression_dense PRIVATE plugin/dsp)
target_link_libraries(reverb_regression_dense PRIVATE Threads::Threads)

set_target_properties(reverb_regression_dense PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Parallel suite for plugin/dsp/ReverbEnginePlate.hpp's own Dattorro-
# style plate topology -- see tests/reverb_regression_plate.cpp's own
# header comment for why most of its checks are new, not renumbered
# [R#]s (there is no Householder matrix or per-line array here to check).
add_executable(reverb_regression_plate tests/reverb_regression_plate.cpp)
target_include_directories(reverb_regression_plate PRIVATE plugin/dsp)
target_link_libraries(reverb_regression_plate PRIVATE Threads::Threads)

set_target_properties(reverb_regression_plate PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# Pre-Delay (plugin/dsp/PreDelayLine.hpp) -- shared, model-independent,
# so it gets one test executable, not one per plugin.
add_executable(predelay_regression tests/predelay_regression.cpp)
target_include_directories(predelay_regression PRIVATE plugin/dsp)

set_target_properties(predelay_regression PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# ---------------------------------------------------------------------
# Offline reference-sample generator -- see
# tools/reference/generate_reference_samples.cpp's own header comment.
# Not a plugin, no DPF dependency, same "plain executable" shape as the
# regression tools above.
# ---------------------------------------------------------------------

add_executable(generate_reference_samples tools/reference/generate_reference_samples.cpp)
target_include_directories(generate_reference_samples PRIVATE plugin/dsp)

set_target_properties(generate_reference_samples PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
