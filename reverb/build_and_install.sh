#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
#
# Builds the Reverb plugin (LV2, VST3, standalone JACK -- plus the
# reverb_regression tool) and installs the LV2 bundle into ~/.lv2/ so any
# LV2 host (Ardour, Carla, ...) picks it up. Adapted from the sibling
# ../statespace/build_and_install.sh and ../compressor/build_and_install.sh
# (same flags, same auto-clean-on-moved-checkout behavior) so every
# MimoMusic project here is built and installed the same way.
#
# Verified end to end: configure, build (reverb_dense +
# reverb_plate, each LV2/VST3/JACK), all three regression suites, and LV2
# install all run clean.
#
# Usage:
#   ./build_and_install.sh              # build + install into ~/.lv2/
#   ./build_and_install.sh --clean      # wipe build/ first (full rebuild)
#   ./build_and_install.sh --no-install # build only, don't touch ~/.lv2/
#   ./build_and_install.sh --test       # also run reverb_regression afterwards
#
# Flags can be combined. Re-run any time -- the install step always
# removes the previously-installed bundle of the same name before
# copying, so ~/.lv2/ never ends up with a mix of old and new files.
#
# A build/ directory left over from a different absolute path (this
# checkout was itself moved/copied after a previous build) is detected
# automatically and cleaned before reconfiguring -- CMake bakes the exact
# source/build paths into CMakeCache.txt and refuses to reconfigure across
# a mismatch, which is a confusing error to hit blind.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Every mimomusic/ sibling project's own build/ output lands in one shared
# ../builds/<project>/ directory instead of nested inside each checkout --
# one thing to exclude from Nextcloud (or similar) sync instead of many.
# Overridable via MIMOMUSIC_BUILDS_ROOT for a checkout that isn't sitting
# inside the usual mimomusic/ parent.
BUILDS_ROOT="${MIMOMUSIC_BUILDS_ROOT:-$SCRIPT_DIR/../builds}"
BUILD_DIR="$BUILDS_ROOT/reverb/build"
LV2_DIR="$HOME/.lv2"

DO_CLEAN=0
DO_INSTALL=1
DO_TEST=0

for arg in "$@"; do
    case "$arg" in
        --clean)      DO_CLEAN=1 ;;
        --no-install) DO_INSTALL=0 ;;
        --test)       DO_TEST=1 ;;
        -h|--help)
            sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | grep '^#' | sed -e 's|^# ||' -e 's|^#$||'
            exit 0
            ;;
        *)
            echo "Unknown option: $arg (try --help)" >&2
            exit 2
            ;;
    esac
done

echo "== MimoMusic Reverb plugin: build =="
echo "Source: $SCRIPT_DIR"
echo "Build:  $BUILD_DIR"
[ "$DO_INSTALL" -eq 1 ] && echo "Install LV2 into: $LV2_DIR"
echo

# --- 0. Submodules -----------------------------------------------------
if [ -f "$SCRIPT_DIR/.gitmodules" ] && [ ! -f "$SCRIPT_DIR/dpf/CMakeLists.txt" ]; then
    echo "-- Fetching git submodules (DPF + its nested pugl) --"
    git -C "$SCRIPT_DIR" submodule update --init --recursive
    echo
fi

# --- 1. Configure + build -----------------------------------------------
if [ "$DO_CLEAN" -eq 0 ] && [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cached_home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BUILD_DIR/CMakeCache.txt")"
    if [ -n "$cached_home" ] && [ "$(realpath -m "$cached_home")" != "$(realpath -m "$SCRIPT_DIR")" ]; then
        echo "-- build/ was configured from a different path ($cached_home) --"
        echo "   Cleaning it automatically before reconfiguring."
        DO_CLEAN=1
    fi
fi

if [ "$DO_CLEAN" -eq 1 ]; then
    echo "-- Removing $BUILD_DIR --"
    rm -rf "$BUILD_DIR"
fi

# DPF patches (dpf-patches/, see dpf_patches.sh there): applied for the
# build, reverted again on exit (also on failure), so dpf/ stays upstream.
# Made executable if a copy or file sync dropped the bit, and run through
# bash so it works even where that isn't possible.
DPF_PATCHES="$SCRIPT_DIR/dpf-patches/dpf_patches.sh"
[ -x "$DPF_PATCHES" ] || chmod +x "$DPF_PATCHES" 2>/dev/null || true
bash "$DPF_PATCHES" apply "$SCRIPT_DIR/dpf" "$BUILD_DIR"
trap 'bash "$DPF_PATCHES" revert "$SCRIPT_DIR/dpf" "$BUILD_DIR"' EXIT

echo "-- Configuring (cmake, Release) --"
if ! cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE=Release; then
    echo
    echo "ERROR: cmake configure failed. On a Debian/Ubuntu system the usual"
    echo "cause is missing development headers -- this project needs:"
    echo "  sudo apt-get install build-essential cmake pkg-config git \\"
    echo "      libx11-dev libxext-dev libxcursor-dev libxrandr-dev \\"
    echo "      libgl1-mesa-dev libglu1-mesa-dev \\"
    echo "      libjack-jackd2-dev libasound2-dev"
    exit 1
fi

echo
echo "-- Building (reverb_dense + reverb_plate x LV2/VST3/JACK, plus all three regression tools) --"
cmake --build "$BUILD_DIR" -j"$(nproc)"

# --- 2. Optional: regression tests --------------------------------------
if [ "$DO_TEST" -eq 1 ]; then
    echo
    echo "-- Running reverb_regression --"
    "$BUILD_DIR/bin/reverb_regression"
    echo
    echo "-- Running reverb_regression_dense --"
    "$BUILD_DIR/bin/reverb_regression_dense"
    echo
    echo "-- Running reverb_regression_plate --"
    "$BUILD_DIR/bin/reverb_regression_plate"
    echo
    echo "-- Running predelay_regression --"
    "$BUILD_DIR/bin/predelay_regression"
fi

# --- 3. Report what got built --------------------------------------------
echo
echo "-- Built artifacts in $BUILD_DIR/bin --"
for f in "$BUILD_DIR"/bin/*.lv2 "$BUILD_DIR"/bin/*.vst3; do
    [ -e "$f" ] && echo "  $(basename "$f")"
done

if [ "$DO_INSTALL" -eq 0 ]; then
    echo
    echo "--no-install given: nothing copied to $LV2_DIR."
    exit 0
fi

# --- 4. Install the LV2 bundle into ~/.lv2/ ------------------------------
shopt -s nullglob
BUNDLES=("$BUILD_DIR"/bin/*.lv2)
shopt -u nullglob
if [ ${#BUNDLES[@]} -eq 0 ]; then
    echo "ERROR: no .lv2 bundles found in $BUILD_DIR/bin -- the build must have failed." >&2
    exit 1
fi

echo
echo "-- Installing into $LV2_DIR --"
mkdir -p "$LV2_DIR"
for bundle in "${BUNDLES[@]}"; do
    name="$(basename "$bundle")"
    rm -rf "${LV2_DIR:?}/$name"
    cp -r "$bundle" "$LV2_DIR/"
    echo "  installed $name"
done

echo
echo "Done. Restart Ardour (or rescan plugins) to pick up:"
for bundle in "${BUNDLES[@]}"; do
    echo "  $(basename "$bundle" .lv2)"
done
echo
echo "VST3 was built too (in $BUILD_DIR/bin/*.vst3) -- copy it into"
echo "~/.vst3/ the same way if you want it; this script installs only the"
echo "LV2 version."
