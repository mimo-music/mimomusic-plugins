#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
#
# macOS counterpart to build_and_install.sh -- run this ON a Mac. This
# project is *not* reliably cross-compilable for macOS from this Linux
# checkout: DPF's macOS backends (the Cocoa-based window/event code, and
# the AU target this script also builds) are Objective-C++ compiled and
# linked against the actual macOS SDK and frameworks (Cocoa, OpenGL,
# AudioToolbox, AudioUnit, CoreFoundation) -- getting a *working*,
# *verifiable* build without an actual Mac to compile and run it on isn't
# something a Linux checkout can honestly deliver (cross-compiling via
# osxcross is possible for anyone who wants to pursue that route
# themselves). This script is the practical answer:
# build natively, on your own machine, where the result can actually be
# tested.
#
# Builds every plugin as a universal binary (Apple Silicon + Intel, i.e.
# both arm64 and x86_64 in one binary) covering LV2, VST3, AU (Audio Unit
# -- macOS-only, silently skipped everywhere else, see CMakeLists.txt's own
# comment on why it's simply listed in TARGETS unconditionally) and a
# standalone app, then installs the LV2/VST3/AU bundles into the standard
# per-user plugin folders:
#   ~/Library/Audio/Plug-Ins/LV2
#   ~/Library/Audio/Plug-Ins/VST3
#   ~/Library/Audio/Plug-Ins/Components   (AU)
#
# Requirements:
#   - Xcode Command Line Tools -- provides the actual clang/clang++ and the
#     macOS SDK this needs. Install with: xcode-select --install
#   - Homebrew (https://brew.sh) -- only used, and only if needed, to
#     install `cmake`; pass --no-deps to skip this check entirely (e.g. if
#     cmake is already on your PATH some other way).
#
# Usage: (same flags as build_and_install.sh, plus three macOS-specific ones)
#   ./build_and_install_macos.sh
#   ./build_and_install_macos.sh --clean
#   ./build_and_install_macos.sh --no-install
#   ./build_and_install_macos.sh --test
#   ./build_and_install_macos.sh --no-deps            # skip the Homebrew/cmake check
#   ./build_and_install_macos.sh --arch arm64          # single-arch instead of a universal binary (faster while iterating)
#   ./build_and_install_macos.sh --min-macos 11.0      # override the deployment target (default 10.13)
#
# Flags can be combined. Re-run any time -- the install step always
# removes the previously-installed bundle of the same name before copying.
#
# Written and reviewed carefully, but genuinely untested end to end: no
# Mac was available in the environment this was written in. If something here is wrong for your Xcode/macOS version,
# that's the most likely place -- the CMake/DPF side underneath it is the
# same, already-verified build every other build_and_install*.sh in this
# project uses, just with macOS-specific flags layered on top.

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
BUILD_DIR="$BUILDS_ROOT/statespace/build-macos"   # kept separate from build/ so a Linux and a macOS checkout of the same tree (e.g. shared over a network drive) never fight over one CMakeCache.txt
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

echo "== StateSpace plugins: build all (macOS) =="
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

# DPF's macOS build needs dpf/dgl/src/pugl.mm. Upstream DPF ships that as a
# symlink to pugl.cpp (identical source, just under a .mm extension so
# clang compiles it as Objective-C++) -- it's the only symlink anywhere in
# the DPF tree. Symlinks don't survive every path a checkout can take to a
# Mac: cloud-sync tools such as Nextcloud/Dropbox/Google Drive routinely
# skip or drop them, which leaves dpf/CMakeLists.txt (a plain file) present
# and correct while pugl.mm silently never arrives -- exactly the failure
# that slips past the submodule check above. Recreate it as a plain copy
# whenever it's missing or empty, so this works regardless of how the
# checkout got here.
PUGL_MM="$SCRIPT_DIR/dpf/dgl/src/pugl.mm"
PUGL_CPP="$SCRIPT_DIR/dpf/dgl/src/pugl.cpp"
if [ -f "$PUGL_CPP" ] && [ ! -s "$PUGL_MM" ]; then
    echo "-- dpf/dgl/src/pugl.mm missing or empty (symlink likely lost in transfer, e.g. Nextcloud/Dropbox sync) -- recreating from pugl.cpp --"
    cp "$PUGL_CPP" "$PUGL_MM"
    echo
fi

# --- 2. Configure + build ---------------------------------------------------

# Portable best-effort path canonicalization: GNU realpath's -m flag
# (accepts a path that doesn't currently exist -- needed here since the
# very thing being checked is "did this checkout move") isn't available in
# macOS's own /usr/bin/realpath (a plain BSD implementation). Falling back
# to cd+pwd only resolves paths that currently exist, which is fine for
# this specific check: a cached path that no longer exists at all is
# already unambiguously "different" from SCRIPT_DIR (which always exists),
# so treating it as a mismatch either way is the correct, conservative
# outcome.
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

# DPF patches (dpf-patches/, see dpf_patches.sh there): applied for the
# build, reverted again on exit (also on failure), so dpf/ stays upstream.
# Made executable if a copy or file sync dropped the bit, and run through
# bash so it works even where that isn't possible.
DPF_PATCHES="$SCRIPT_DIR/dpf-patches/dpf_patches.sh"
[ -x "$DPF_PATCHES" ] || chmod +x "$DPF_PATCHES" 2>/dev/null || true
bash "$DPF_PATCHES" apply "$SCRIPT_DIR/dpf" "$BUILD_DIR"
trap 'bash "$DPF_PATCHES" revert "$SCRIPT_DIR/dpf" "$BUILD_DIR"' EXIT

echo "-- Configuring (cmake, Release, universal $ARCHES, min macOS $MIN_MACOS) --"
# If MIN_MACOS predates Apple Silicon (arm64 only ever shipped starting
# macOS 11) and arm64 is one of the requested ARCHES, clang is expected to
# print something like "overriding deployment target ... to 11.0 because
# -arch arm64" for that slice -- a normal, harmless adjustment clang makes
# on its own, not a sign anything here is misconfigured.
cmake -B "$BUILD_DIR" -S "$SCRIPT_DIR" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_OSX_ARCHITECTURES="$ARCHES" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$MIN_MACOS"

echo
echo "-- Building (all five plugins x LV2/VST3/AU/standalone, plus the tools) --"
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
echo "StateSpace plugins. AU may additionally need 'killall -9 AudioComponentRegistrar'"
echo "(or a reboot) before a freshly-built .component is noticed -- a standard"
echo "macOS AU cache quirk, not specific to this project."
