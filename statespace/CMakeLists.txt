# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH

# CMake build for five sibling plugins built on the same generic
# state-space DSP core (LV2 + VST3 + a standalone JACK build each, for
# quick manual testing, plus AU -- Audio Unit -- on macOS), using the
# DISTRHO Plugin Framework (DPF, vendored as a git submodule in dpf/).
# "au" is listed in every plugin's own TARGETS below unconditionally --
# DPF's own dpf_add_plugin() macro (dpf/cmake/DPF-plugin.cmake) only
# actually builds it `if (APPLE)` and silently does nothing for it
# otherwise, so this line means nothing extra happens on Linux/Windows and
# a native macOS build (see build_and_install_macos.sh) additionally gets
# an AU bundle for hosts (Logic, GarageBand, and others) that don't load
# LV2/VST3 there, with no separate CMake configuration needed for it:
#   - matrix_filter: the raw, directly-editable A/B/C/D matrix filter
#   - parametric_eq:   a parametric EQ (unbounded band count) built as a
#                       coefficient mapper on top of the same core -- see
#                       plugin/dsp/EqModel.hpp and docs/ARCHITECTURE.md's
#                       "EqModel / EQ plugin" section. Its "Export to
#                       State-Space..." button writes an ordinary .ssjson
#                       the base plugin opens directly, coefficients and
#                       all, via the *same* Serialization code both share.
#   - parametric_eq_mono: the mono (1-in/1-out) variant of parametric_eq --
#                       identical GUI and band math, assembled into a
#                       natively single-channel state-space system (half
#                       the states per band, so roughly a quarter of the
#                       per-sample recursion cost) for projects where only
#                       mono is used. See plugin/dsp/MonoStateSpaceTypes.hpp.
#   - multiband_compressor: a multiband compressor -- a user-configurable
#                       Linkwitz-Riley crossover bank plus a per-band gain
#                       stage (StateSpaceTypes.hpp's MultibandSchedule). See
#                       plugin/dsp/MbCompressorModel.hpp.
#   - dynamic_eq: a dynamic EQ -- FabFilter Pro-Q-style bell bands
#                       whose own boost/cut pulls back automatically when
#                       that frequency region gets loud. Its live audio path
#                       is a dedicated engine, not a StateSpaceMatrices
#                       instance -- see plugin/dsp/DynamicEqModel.hpp's file
#                       comment and plugin/dsp/DynamicEqEngine.hpp.
#   - dynamic_eq2: a hand-adapted CLONE of dynamic_eq (own DSP core,
#                       plugin/dsp/DynamicEqModel2.hpp/.cpp +
#                       plugin/dsp/DynamicEqEngine2.hpp; own GUI,
#                       gui/dyneq2/) -- a place to grow new dynamic-EQ
#                       features (stereo crossfeed bands, harmonic linking,
#                       shelf and band-shelf types, ...) without risking the
#                       original plugin. See plugin_dyn_eq2/DistrhoPluginInfo.h's
#                       own header comment.
#
# Also builds four dependency-free command-line tools (see docs/TESTING.md
# and docs/EXTENDING.md):
#   - dsp_regression:  fast, pure-math correctness tests, no audio files
#   - audio_testbench: runs every factory preset through the real engine
#                       against synthetic WAV signals, cross-checked
#                       against the frequency-domain transfer function
#   - fit_gain_curve:  learns a state-dependent gain curve (see
#                       plugin/dsp/GainCurveTrainer.hpp) from example
#                       input/target-output audio
#   - fit_multiband_compressor: the same idea, generalized to N independent
#                       per-band gain curves behind a Linkwitz-Riley
#                       crossover (see plugin/dsp/GainCurveTrainer.hpp's
#                       fitMultiband() and StateSpaceTypes.hpp's
#                       MultibandSchedule)
#
#   git submodule update --init      # first time only
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build -j
#   ./build/bin/dsp_regression
#   ./build/bin/audio_testbench   # works from any directory, incl. build/bin/ itself
#   ./build/bin/fit_gain_curve presets/factory/passthrough.ssjson in.wav target.wav out.ssjson
#   ./build/bin/fit_multiband_compressor "150,2000" in.wav target.wav out.ssjson
#
# Plugin outputs land under build/bin/: an .lv2 bundle, a .vst3 bundle, and
# a standalone JACK executable for each of "matrix_filter",
# "parametric_eq" and "parametric_eq_mono".
#
# ./build_and_install.sh builds all of this and copies every .lv2 bundle
# into ~/.lv2/ in one step.

cmake_minimum_required(VERSION 3.15)
project(matrix_filter_plugin)

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
# #error. None of the five plugins need this standalone build to work in
# Ardour (LV2/VST3) or other macOS hosts (AU), so skip "jack" on Apple
# rather than fight SDL2's bug; every other platform keeps it.
if(APPLE)
  set(STATESPACE_EXTRA_TARGETS "")
else()
  set(STATESPACE_EXTRA_TARGETS jack)
endif()

dpf_add_plugin(matrix_filter
  TARGETS lv2 vst3 au ${STATESPACE_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin/PluginStateSpace.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/TransferFunction.cpp
      plugin/dsp/Serialization.cpp
  FILES_UI
      gui/UIStateSpace.cpp
      gui/MatrixGridWidget.cpp
      gui/BodePlotWidget.cpp
      gui/NonlinearPanelWidget.cpp
      gui/SimpleButton.cpp
      # The UI and DSP sides of a DPF plugin can end up in separate shared
      # objects (and, for some formats/hosts, separate processes), so the
      # small pure-math modules the GUI also needs (stability analysis for
      # the badge, JSON (de)serialization for load/save, the transfer-
      # function sweep for the Bode plot) are compiled into both sides
      # rather than the UI linking against the DSP library.
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/TransferFunction.cpp
      plugin/dsp/Serialization.cpp
)

target_include_directories(matrix_filter PUBLIC
  plugin
  gui
)

# ---------------------------------------------------------------------
# parametric_eq -- the parametric EQ sibling. Its own DistrhoPluginInfo.h
# lives in plugin_eq/ (a *separate* directory from the base plugin's own
# plugin/DistrhoPluginInfo.h) so the two plugins' identities can never be
# ambiguously resolved by include order; everything else it needs from the
# shared DSP core is reached via explicit relative "../plugin/dsp/..."
# includes (see plugin_eq/PluginEq.cpp, gui/eq/*.cpp) rather than adding
# "plugin" itself to this target's include path.
# ---------------------------------------------------------------------

dpf_add_plugin(parametric_eq
  TARGETS lv2 vst3 au ${STATESPACE_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_eq/PluginEq.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/eq/UIEq.cpp
      gui/eq/EqGraphWidget.cpp
      gui/eq/BandListWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      # Same reasoning as matrix_filter's own duplicate list above: the
      # DSP and UI sides can end up in separate shared objects/processes.
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/Serialization.cpp # only the UI side needs this, for "Export to State-Space..."
)

target_include_directories(parametric_eq PUBLIC
  plugin_eq
  gui/eq
)

# ---------------------------------------------------------------------
# parametric_eq_mono -- the mono sibling of parametric_eq. Same GUI
# (gui/eq/EqGraphWidget + BandListWidget, reused unchanged -- an
# ssfilter::EqModel band list has no channel count of its own) and same
# band-design math, but assembled into a natively single-channel
# state-space system (plugin/dsp/MonoStateSpaceTypes.hpp,
# MonoStateSpaceEngine.hpp, MonoStabilityGuard.*), which needs half the
# states per band -- so the O(order^2) per-sample recursion costs about a
# quarter of the stereo plugin's for the same band count. For projects
# where only mono is used.
#
# Its own DistrhoPluginInfo.h lives in plugin_eq_mono/ for exactly the
# same include-ambiguity reason plugin_eq/ has its own -- see the
# parametric_eq block above.
# ---------------------------------------------------------------------

dpf_add_plugin(parametric_eq_mono
  TARGETS lv2 vst3 au ${STATESPACE_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_eq_mono/PluginEqMono.cpp
      plugin/dsp/MonoStabilityGuard.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/eq/UIEqMono.cpp
      gui/eq/EqGraphWidget.cpp
      gui/eq/BandListWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      # Same reasoning as the two blocks above: the DSP and UI sides can
      # end up in separate shared objects/processes.
      plugin/dsp/MonoStabilityGuard.cpp
      plugin/dsp/EqModel.cpp
      # The UI side additionally needs the *stereo* guard + Serialization
      # for "Export to State-Space..." -- that button deliberately writes
      # a stereo .ssjson (the only shape StateSpace MIMO can open); see
      # UIEqMono::exportStateSpaceFile()'s comment.
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/Serialization.cpp
)

target_include_directories(parametric_eq_mono PUBLIC
  plugin_eq_mono
  gui/eq
)

# ---------------------------------------------------------------------
# multiband_compressor -- the multiband compressor GUI on top of
# StateSpaceTypes.hpp's MultibandSchedule/BandSplit and
# CoefficientMapper.hpp's makeStereoCrossoverBank() (see
# docs/ARCHITECTURE.md's "MultibandSchedule" and docs/EXTENDING.md's
# "Multiband compression" sections for the mechanism this is a GUI on top
# of). plugin/dsp/MbCompressorModel.* is the coefficient-mapper layer
# (mirrors EqModel.* exactly); gui/mb/* is the GUI (mirrors gui/eq/*'s
# structure); gui/ParamStripWidget.* is ported from the sibling
# ../compressor project, for its click-to-edit/drag-fader/scroll-nudge
# numeric fields.
#
# Its own DistrhoPluginInfo.h lives in plugin_mb_compressor/ for exactly
# the same include-ambiguity reason plugin_eq/ and plugin_eq_mono/ have
# their own -- see the parametric_eq block above.
# ---------------------------------------------------------------------

dpf_add_plugin(multiband_compressor
  TARGETS lv2 vst3 au ${STATESPACE_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_mb_compressor/PluginMbCompressor.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/MbCompressorModel.cpp
      plugin/dsp/EqModel.cpp # MbCompressorModel.cpp reuses eq::evaluateBiquadResponse(), see its own comment
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/mb/UIMbCompressor.cpp
      gui/mb/MultibandGraphWidget.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      # Same reasoning as every sibling block above: the DSP and UI sides
      # can end up in separate shared objects/processes.
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/MbCompressorModel.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/Serialization.cpp # only the UI side needs this, for "Export to State-Space..."
)

target_include_directories(multiband_compressor PUBLIC
  plugin_mb_compressor
  gui/mb
)

# ---------------------------------------------------------------------
# dynamic_eq -- FabFilter Pro-Q-style bell bands that also carry
# their own threshold/ratio/range/attack/release, so each band's own
# boost/cut pulls back automatically whenever that specific frequency
# region gets loud (a multiband compressor's per-band dynamics, targeted at
# an EQ bell shape instead of a crossover slice). Unlike every plugin
# above, its live audio path is NOT a StateSpaceMatrices/StateSpaceEngine
# instance at all -- see plugin/dsp/DynamicEqModel.hpp's file comment for
# why a continuously gain-varying bell cascade has no fixed state-space
# realization, and plugin/dsp/DynamicEqEngine.hpp for the dedicated
# real-time engine (a Regalia-Mitra fixed-allpass construction) used
# instead. "Export to State-Space..." still produces an ordinary .ssjson
# (a frozen, resting-gain snapshot via the same RBJ cascade EqModel uses).
#
# Its own DistrhoPluginInfo.h lives in plugin_dyn_eq/ for the same
# include-ambiguity reason every sibling plugin's own does -- see the
# parametric_eq block above.
# ---------------------------------------------------------------------

dpf_add_plugin(dynamic_eq
  TARGETS lv2 vst3 au ${STATESPACE_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_dyn_eq/PluginDynEq.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/DynamicEqModel.cpp
      plugin/dsp/EqModel.cpp # DynamicEqModel.cpp reuses eq::evaluateBiquadResponse(), see its own comment
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/dyneq/UIDynEq.cpp
      gui/dyneq/DynEqGraphWidget.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      # Same reasoning as every sibling block above: the DSP and UI sides
      # can end up in separate shared objects/processes.
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/DynamicEqModel.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/Serialization.cpp # only the UI side needs this, for "Export to State-Space..."
)

target_include_directories(dynamic_eq PUBLIC
  plugin_dyn_eq
  gui/dyneq
)

# ---------------------------------------------------------------------
# dynamic_eq2 -- a hand-adapted clone of dynamic_eq (own DSP core, own
# GUI, own binary/LV2/VST3 identity) -- see plugin_dyn_eq2/DistrhoPluginInfo.h's
# own header comment for why this exists as a real second plugin rather
# than a branch/flag on the original.
# ---------------------------------------------------------------------

dpf_add_plugin(dynamic_eq2
  TARGETS lv2 vst3 au ${STATESPACE_EXTRA_TARGETS}
  USE_FILE_BROWSER
  FILES_DSP
      plugin_dyn_eq2/PluginDynEq2.cpp
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/DynamicEqModel2.cpp
      plugin/dsp/EqModel.cpp # DynamicEqModel2.cpp reuses eq::evaluateBiquadResponse(), see its own comment
      plugin/dsp/SpectrumAnalyzer.cpp
  FILES_UI
      gui/dyneq2/UIDynEq2.cpp
      gui/dyneq2/DynEqGraphWidget2.cpp
      gui/ParamStripWidget.cpp
      gui/SimpleButton.cpp
      gui/TooltipOverlay.cpp
      # Same reasoning as every sibling block above: the DSP and UI sides
      # can end up in separate shared objects/processes.
      plugin/dsp/StabilityGuard.cpp
      plugin/dsp/DynamicEqModel2.cpp
      plugin/dsp/EqModel.cpp
      plugin/dsp/Serialization.cpp # only the UI side needs this, for "Export to State-Space..."
)

target_include_directories(dynamic_eq2 PUBLIC
  plugin_dyn_eq2
  gui/dyneq2
)

# ---------------------------------------------------------------------
# Verification tools -- plain executables, no DPF/DGL dependency at all,
# just the framework-independent DSP core (see docs/TESTING.md).
# ---------------------------------------------------------------------

add_executable(dsp_regression tests/dsp_regression.cpp
  plugin/dsp/StabilityGuard.cpp
  plugin/dsp/MonoStabilityGuard.cpp
  plugin/dsp/TransferFunction.cpp
  plugin/dsp/Serialization.cpp
  plugin/dsp/GainCurveTrainer.cpp
  plugin/dsp/EqModel.cpp
  plugin/dsp/MbCompressorModel.cpp
  plugin/dsp/DynamicEqModel.cpp
  plugin/dsp/DynamicEqModel2.cpp
  plugin/dsp/SpectrumAnalyzer.cpp
)
target_include_directories(dsp_regression PRIVATE plugin/dsp)

add_executable(audio_testbench tools/testbench/audio_testbench.cpp
  plugin/dsp/StabilityGuard.cpp
  plugin/dsp/TransferFunction.cpp
  plugin/dsp/Serialization.cpp
)
target_include_directories(audio_testbench PRIVATE plugin/dsp)
# So the default preset/output paths work when run as ./build/bin/audio_testbench
# from *any* current directory, not just the project root.
target_compile_definitions(audio_testbench PRIVATE
  STATESPACE_PROJECT_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)
if(APPLE)
  # std::filesystem's symbols (fs::exists, fs::directory_iterator, ...) are
  # marked unavailable by Apple's libc++ headers when the deployment target
  # is below macOS 10.15 -- the filesystem TS runtime bits weren't in the
  # system dylib before that. audio_testbench is a standalone dev tool
  # (not shipped with the plugin, not installed), so raise its own target
  # instead of the whole project's -- the actual plugin/DGL/DPF targets
  # keep building down to whatever --min-macos was requested (10.13 by
  # default in build_and_install_macos.sh).
  target_compile_options(audio_testbench PRIVATE -mmacosx-version-min=10.15)
  target_link_options(audio_testbench PRIVATE -mmacosx-version-min=10.15)
endif()

add_executable(fit_gain_curve tools/trainer/fit_gain_curve.cpp
  plugin/dsp/StabilityGuard.cpp
  plugin/dsp/Serialization.cpp
  plugin/dsp/GainCurveTrainer.cpp
)
target_include_directories(fit_gain_curve PRIVATE plugin/dsp)

add_executable(fit_multiband_compressor tools/trainer/fit_multiband_compressor.cpp
  plugin/dsp/StabilityGuard.cpp
  plugin/dsp/Serialization.cpp
  plugin/dsp/GainCurveTrainer.cpp
)
target_include_directories(fit_multiband_compressor PRIVATE plugin/dsp)

# Land these next to the plugin outputs (DPF's own targets already go to
# build/bin/ via its own macro) rather than CMake's default of build/.
set_target_properties(dsp_regression audio_testbench fit_gain_curve fit_multiband_compressor PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
