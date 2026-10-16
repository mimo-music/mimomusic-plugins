#!/usr/bin/env bash
#
# macOS counterpart to build_and_install.sh -- run this ON a Mac. Same
# aggregator idea: runs every project's own ./build_and_install_macos.sh in
# turn (amplifier, statespace, compressor, meters, reverb) instead of
# `cd`-ing into each one by hand, then reports one combined summary --
# the same set of plugins as on Linux.
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
# Every flag is forwarded, unchanged, to each sub-script. A project that
# fails to build does NOT stop the others -- every project is attempted,
# and the summary at the end lists which ones actually succeeded. Exits
# non-zero overall if any (non-skipped) project failed.
#
# To build/install only one project, just run that project's own script
# directly -- this aggregator is purely a convenience for "build everything".
#
# Written and reviewed carefully, but genuinely untested end to end: no
# Mac was available in the environment this was written in -- same caveat
# as every build_and_install_macos.sh in this family.

set -uo pipefail

if [ "$(uname -s)" != "Darwin" ]; then
    echo "This script is for macOS only -- use ./build_and_install.sh on Linux." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECTS=(amplifier statespace compressor meters reverb)

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | grep '^#' | sed -e 's|^# ||' -e 's|^#$||'
    exit 0
fi

echo "== mimomusic: build all plugin projects (macOS) =="
echo "Projects: ${PROJECTS[*]}"
echo "Builds land in: $SCRIPT_DIR/builds/<project>/"
echo

declare -a OK=()
declare -a FAILED=()
declare -a SKIPPED=()

for proj in "${PROJECTS[@]}"; do
    script="$SCRIPT_DIR/$proj/build_and_install_macos.sh"
    echo "############################################################"
    echo "## $proj"
    echo "############################################################"
    if [ ! -f "$script" ]; then
        echo "SKIPPED: $script not found."
        SKIPPED+=("$proj")
        echo
        continue
    fi
    if [ ! -x "$script" ]; then
        echo "Note: $script is not executable -- fixing (chmod +x)."
        chmod +x "$script" 2>/dev/null || true
    fi
    # Through bash, so it also runs where the bit could not be restored.
    if ( cd "$SCRIPT_DIR/$proj" && bash "$script" "$@" ); then
        OK+=("$proj")
    else
        echo "FAILED: $proj"
        FAILED+=("$proj")
    fi
    echo
done

echo "============================================================"
echo "Summary:"
# ${arr[@]+"${arr[@]}"}: macOS's bash 3.2 treats an empty "${arr[@]}" as
# an unbound variable under set -u.
for proj in ${OK[@]+"${OK[@]}"}; do
    echo "  OK       $proj"
done
for proj in ${FAILED[@]+"${FAILED[@]}"}; do
    echo "  FAILED   $proj"
done
for proj in ${SKIPPED[@]+"${SKIPPED[@]}"}; do
    echo "  SKIPPED  $proj"
done
echo "============================================================"

if [ ${#FAILED[@]} -gt 0 ]; then
    exit 1
fi
