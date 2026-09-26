#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
#
# Builds generate_reference_samples as a single, dependency-free binary
# -- no DPF, no X11/OpenGL/ALSA, no CMake configure step needed, so this
# works on a machine that only has the 6 files this tool actually
# depends on (see this script's own SRCS list below), e.g. a headless
# box copied over just to run a long --order job. Kept deliberately
# dependency-free like every other tools/ executable here.
#
# Usage:
#   ./build_standalone.sh              # builds ./generate_reference_samples next to this script
#   CXX=clang++ ./build_standalone.sh  # override the compiler
#
# Needs: a C++17 compiler (GCC 9+ or Clang 7+) -- nothing else. Falls
# back to linking -lstdc++fs automatically if the first attempt fails
# with an unresolved std::filesystem symbol (older GCC/libstdc++ shipped
# <filesystem> as a separate archive before it was folded into libstdc++
# proper).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$SCRIPT_DIR/generate_reference_samples.cpp"
OUT="$SCRIPT_DIR/generate_reference_samples"
CXX="${CXX:-g++}"

# The full dependency closure of generate_reference_samples.cpp -- every
# #include it (transitively) pulls in that isn't a plain standard-library
# header. Checked here, not just assumed, so a missing file fails loudly
# with a clear message instead of a confusing compiler error.
DEPS=(
    "$SCRIPT_DIR/../../plugin/dsp/ReverbEngine.hpp"
    "$SCRIPT_DIR/../testbench/Wav.hpp"
    "$SCRIPT_DIR/ImageSourceMethod.hpp"
    "$SCRIPT_DIR/ExcitationSignals.hpp"
    "$SCRIPT_DIR/ReferenceEngine.hpp"
)
for dep in "${DEPS[@]}"; do
    if [ ! -f "$dep" ]; then
        echo "ERROR: missing $dep" >&2
        echo "This tool needs its 6 files copied together, preserving their" >&2
        echo "relative layout (plugin/dsp/ReverbEngine.hpp, tools/testbench/Wav.hpp," >&2
        echo "and everything under tools/reference/) -- see DEPS at the top of" >&2
        echo "this script for the exact list." >&2
        exit 1
    fi
done

if ! command -v "$CXX" >/dev/null 2>&1; then
    echo "ERROR: '$CXX' not found. Install a C++17 compiler (e.g. 'sudo apt-get install g++')" >&2
    echo "or point CXX at one, e.g.: CXX=g++-12 ./build_standalone.sh" >&2
    exit 1
fi

echo "== Building generate_reference_samples =="
echo "Compiler: $CXX ($("$CXX" --version | head -1))"
echo "Source:   $SRC"
echo "Output:   $OUT"
echo

FLAGS=(-std=c++17 -O2 -Wall -Wextra)
ERR_LOG="$(mktemp)"
trap 'rm -f "$ERR_LOG"' EXIT

if "$CXX" "${FLAGS[@]}" "$SRC" -o "$OUT" 2>"$ERR_LOG"; then
    echo "Built $OUT"
elif grep -qi "filesystem" "$ERR_LOG"; then
    echo "-- Plain build failed on a filesystem symbol, retrying with -lstdc++fs --"
    if "$CXX" "${FLAGS[@]}" "$SRC" -o "$OUT" -lstdc++fs; then
        echo "Built $OUT (needed -lstdc++fs -- an older GCC/libstdc++)"
    else
        echo "ERROR: build failed even with -lstdc++fs. This usually means the" >&2
        echo "compiler is older than GCC 9 / doesn't have a working <filesystem>" >&2
        echo "at all -- a newer compiler is the real fix." >&2
        exit 1
    fi
else
    cat "$ERR_LOG" >&2
    echo "ERROR: build failed -- see the compiler output above." >&2
    exit 1
fi

echo
echo "Run it, e.g.:"
echo "  $OUT <out_dir> --order 3      # fast, verifies everything works"
echo "  $OUT <out_dir> --order 20     # much slower, denser/more physically complete render"
