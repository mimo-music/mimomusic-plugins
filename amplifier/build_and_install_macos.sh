#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
#
# macOS counterpart to build_and_install.sh -- run this ON a Mac. Mirrors
# both sibling projects' own build_and_install_macos.sh exactly (same
# flags, same reasoning for why this has to be a native-build script
# rather than something cross-compiled from the Linux checkout this was
# written in -- see either sibling's own header comment for the full
# reasoning) so all three projects are
# built and installed on macOS the same way they already are on Linux.
#
# Builds every plugin as a universal binary (Apple Silicon + Intel) --
# LV2, VST3, AU (Audio Unit -- macOS-only, silently skipped everywhere
# else, see CMakeLists.txt's own comment) and a standalone app -- and
# installs the LV2/VST3/AU bundles into the standard per-user plugin
# folders:
#   ~/Library/Audio/Plug-Ins/LV2
#   ~/Library/Audio/Plug-Ins/VST3
#   ~/Library/Audio/Plug-Ins/Components   (AU)
#
# Requirements:
#   - Xcode Command Line Tools: xcode-select --install
#   - Homebrew (https://brew.sh) -- only to install `cmake` if it's
#     missing; pass --no-deps to skip this check.
#
# Usage:
#   ./build_and_install_macos.sh
#   ./build_and_install_macos.sh --clean
#   ./build_and_install_macos.sh --no-install
#   ./build_and_install_macos.sh --test
#   ./build_and_install_macos.sh --no-deps
#   ./build_and_install_macos.sh --arch arm64
#   ./build_and_install_macos.sh --min-macos 11.0
#
# Written and reviewed carefully, but genuinely untested end to end: no
# Mac was available in the environment this was written in. If something
# here is wrong for your Xcode/macOS version, that's the most likely
# place -- the CMake/DPF build underneath is the same, already-verified
# one every other build_and_install*.sh in this and both sibling projects
# uses.

set -euo pipefail

if [ "$(uname -s)" != "Darwin" ]; then
    echo "This script is for macOS only -- use ./build_and_install.sh on Linux." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# See build_and_install.sh's identical comment: all four mimomusic/
# sibling projects' own build output lands in one shared ../builds/
# directory, so it's one thing to exclude from Nextcloud sync.
BUILDS_ROOT="${MIMOMUSIC_BUILDS_ROOT:-$SCRIPT_DIR/../builds}"
BUILD_DIR="$BUILDS_ROOT/amplifier/build-macos"
LV2_DIR="$HOME/Library/Audio/Plug-Ins/LV2"
VST3_DIR="$HOME/Library/Audio/Plug-Ins/VST3"
COMPONENT_DIR="$HOME/Library/Audio/Plug-Ins/Components"

DO_CLEAN=0
DO_INSTALL=1
DO_TEST=0
DO_DEPS=1
ARCHES="arm64;x86_64"
MIN_MACOS="10.13"

while [ $# -gt 0 ]; do
    case "$1" in
        --clean)      DO_CLEAN=1; shift ;;
        --no-install) DO_INSTALL=0; shift ;;
        --test)       DO_TEST=1; shift ;;
        --no-deps)    DO_DEPS=0; shift ;;
        --arch)       ARCHES="${2:?--arch needs a value, e.g. --arch arm64}"; shift 2 ;;
        --arch=*)     ARCHES="${1#--arch=}"; shift ;;
        --min-macos)  MIN_MACOS="${2:?--min-macos needs a value, e.g. --min-macos 11.0}"; shift 2 ;;
        --min-macos=*) MIN_MACOS="${1#--min-macos=}"; shift ;;
        -h|--help)
            sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | grep '^#' | sed -e 's|^# ||' -e 's|^#$||'
            exit 0
            ;;
        *)
            echo "Unknown option: $1 (try --help)" >&2
            exit 2
            ;;
    esac
done

echo "== MimoMusic Amplifier plugins: build all (macOS) =="
echo "Source: $SCRIPT_DIR"
echo "Build:  $BUILD_DIR"
echo "Arch(es): $ARCHES   Min macOS: $MIN_MACOS"
[ "$DO_INSTALL" -eq 1 ] && echo "Install into: $LV2_DIR, $VST3_DIR, $COMPONENT_DIR"
echo

# --- 0. Toolchain sanity checks ---------------------------------------------

if ! xcode-select -p >/dev/null 2>&1; then
    echo "ERROR: Xcode Command Line Tools aren't installed." >&2
    echo "        Run: xcode-select --install" >&2
    exit 1
fi

if [ "$DO_DEPS" -eq 1 ] && ! command -v cmake >/dev/null 2>&1; then
    if command -v brew >/dev/null 2>&1; then
        echo "-- cmake not found -- installing via Homebrew --"
        brew install cmake
        echo
    else
        echo "ERROR: cmake not found, and Homebrew isn't installed to fetch it." >&2
        echo "        Install Homebrew (https://brew.sh) and re-run, or install" >&2
        echo "        cmake yourself and re-run with --no-deps." >&2
        exit 1
    fi
fi

# --- 1. Submodules -----------------------------------------------------------
if [ -f "$SCRIPT_DIR/.gitmodules" ] && [ ! -f "$SCRIPT_DIR/dpf/CMakeLists.txt" ]; then
    echo "-- Fetching git submodules (DPF + its nested pugl) --"
    git -C "$SCRIPT_DIR" submodule update --init --recursive
    echo
fi

# DPF's macOS build needs dpf/dgl/src/pugl.mm even for a GUI-less plugin
# like this one (DPF's OpenGL/DGL wiring pulls it in unconditionally, e.g.
# for the Standalone JACK app format). Upstream DPF ships that file as a
# symlink to pugl.cpp (identical source, just under a .mm extension so
# clang compiles it as Objective-C++) -- it's the only symlink anywhere in
# the DPF tree. Symlinks don't survive every path a checkout can take to a
# Mac: cloud-sync tools such as Nextcloud/Dropbox/Google Drive routinely
# skip or drop them, which leaves dpf/CMakeLists.txt (a plain file) present
# and correct while pugl.mm silently never arrives -- exactly the failure
# that slips past the submodule check above (see either sibling project's
# identical block). Recreate it as a plain copy whenever it's missing or empty,
# so this works regardless of how the checkout got here.
PUGL_MM="$SCRIPT_DIR/dpf/dgl/src/pugl.mm"
PUGL_CPP="$SCRIPT_DIR/dpf/dgl/src/pugl.cpp"
if [ -f "$PUGL_CPP" ] && [ ! -s "$PUGL_MM" ]; then
    echo "-- dpf/dgl/src/pugl.mm missing or empty (symlink likely lost in transfer, e.g. Nextcloud/Dropbox sync) -- recreating from pugl.cpp --"
    cp "$PUGL_CPP" "$PUGL_MM"
    echo
fi

# --- 2. Configure + build ---------------------------------------------------

# See either sibling project's identical comment: not just `realpath -m`
# (a GNU-only flag; macOS's own /usr/bin/realpath doesn't have it).
resolve_path() {
    realpath -m "$1" 2>/dev/null && return
    (cd "$1" 2>/dev/null && pwd) || printf '%s' "$1"
}

if [ "$DO_CLEAN" -eq 0 ] && [ -f "$BUILD_DIR/CMakeCache.txt" ]; then
    cached_home="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$BUILD_DIR/CMakeCache.txt")"
    if [ -n "$cached_home" ] && [ "$(resolve_path "$cached_home")" != "$(resolve_path "$SCRIPT_DIR")" ]; then
        echo "-- build-macos/ was configured from a different path ($cached_home) --"
        echo "   Cleaning it automatically before reconfiguring."
        DO_CLEAN=1
    fi
fi

if [ "$DO_CLEAN" -eq 1 ]; then
    echo "-- Removing $BUILD_DIR --"
    rm -rf "$BUILD_DIR"
fi

echo "-- Configuring (cmake, Release, universal $ARCHES, min macOS $MIN_MACOS) --"
# A clang "overriding deployment target ... because -arch arm64" message
# for that slice is expected and harmless if MIN_MACOS predates Apple
# Silicon (macOS 11) -- same as both sibling projects' own scripts.
cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES="$ARCHES" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$MIN_MACOS"

echo
echo "-- Building (both plugins x LV2/VST3/AU/standalone, plus dsp_regression) --"
cmake --build "$BUILD_DIR" -j"$(sysctl -n hw.ncpu)"

# --- 3. Optional: regression tests ------------------------------------------
if [ "$DO_TEST" -eq 1 ]; then
    echo
    echo "-- Running dsp_regression --"
    "$BUILD_DIR/bin/dsp_regression"
fi

# --- 4. Report what got built -----------------------------------------------
echo
echo "-- Built artifacts in $BUILD_DIR/bin --"
for f in "$BUILD_DIR"/bin/*.lv2 "$BUILD_DIR"/bin/*.vst3 "$BUILD_DIR"/bin/*.component; do
    [ -e "$f" ] && echo "  $(basename "$f")"
done

if [ "$DO_INSTALL" -eq 0 ]; then
    echo
    echo "--no-install given: nothing copied to ~/Library/Audio/Plug-Ins/."
    exit 0
fi

# --- 5. Install ---------------------------------------------------------------
install_bundles() {
    local dest="$1"; shift
    local glob="$1"; shift
    shopt -s nullglob
    local bundles=("$BUILD_DIR"/bin/$glob)
    shopt -u nullglob
    if [ ${#bundles[@]} -eq 0 ]; then
        return
    fi
    mkdir -p "$dest"
    for bundle in "${bundles[@]}"; do
        name="$(basename "$bundle")"
        rm -rf "${dest:?}/$name"
        cp -r "$bundle" "$dest/"
        echo "  installed $name -> $dest"
    done
}

echo
echo "-- Installing --"
install_bundles "$LV2_DIR" "*.lv2"
install_bundles "$VST3_DIR" "*.vst3"
install_bundles "$COMPONENT_DIR" "*.component"

echo
echo "Done. Restart your DAW (or rescan plugins) to pick up the new/updated"
echo "Amplifier plugins. AU may additionally need"
echo "'killall -9 AudioComponentRegistrar' (or a reboot) before a freshly-built"
echo ".component is noticed -- a standard macOS AU cache quirk."
