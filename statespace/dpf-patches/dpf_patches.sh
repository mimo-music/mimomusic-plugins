#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
#
# Applies this project's DPF patches (dpf-patches/*.patch, in name order)
# to the dpf/ submodule for a build, and reverts them afterwards, so dpf/
# itself stays the official upstream DPF:
#
#   dpf_patches.sh apply  <dpf-dir> <state-dir>
#   dpf_patches.sh revert <dpf-dir> <state-dir>
#
# build_and_install*.sh call "apply" before configuring and "revert" on
# exit. Both are idempotent (an applied patch is not applied twice, an
# unapplied one not reverted). Patched files get the timestamp
# max(original, patch file), and reverting restores the original one, so
# an incremental build only recompiles DPF when a patch actually changed --
# not after every apply/revert cycle. <state-dir> keeps the original
# timestamps (outside dpf/, normally next to the build directory).
#
# The patches:
#   0001-lv2-ui-resize.patch        LV2 UIs ask the host to resize (ui:resize)
#                                   when the UI resizes itself
#   0002-autoscale-base-size.patch  automatic scaling relative to a design
#                                   size, so the minimum size can be smaller
#   0003-pointer-leave.patch        the pointer leaving the window reaches the
#                                   widgets (as a motion far outside), so hover
#                                   lines and readouts disappear
set -euo pipefail

MODE="${1:?apply|revert}"
DPF_DIR="${2:?dpf directory}"
STATE_DIR="${3:?state directory}"
PATCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MTIMES="$STATE_DIR/dpf-patches.mtimes"

shopt -s nullglob
PATCHES=("$PATCH_DIR"/*.patch)
[ ${#PATCHES[@]} -gt 0 ] || exit 0

is_applied() { git -C "$DPF_DIR" apply --check -R "$1" >/dev/null 2>&1; }
# Portable (Linux and macOS) file timestamps, in seconds.
get_mtime() { perl -e 'print((stat $ARGV[0])[9])' "$1"; }
set_mtime() { perl -e 'utime $ARGV[0], $ARGV[0], $ARGV[1]' "$1" "$2"; }
patched_files() { sed -n 's|^+++ b/||p' "$1"; }

case "$MODE" in
apply)
    mkdir -p "$STATE_DIR"
    for p in "${PATCHES[@]}"; do
        if is_applied "$p"; then continue; fi
        if ! git -C "$DPF_DIR" apply --check "$p" 2>/dev/null; then
            echo "ERROR: $(basename "$p") does not apply to $DPF_DIR (DPF updated? see dpf-patches/)." >&2
            exit 1
        fi
        patch_mtime="$(get_mtime "$p")"
        while read -r f; do
            orig="$(get_mtime "$DPF_DIR/$f")"
            grep -q "^$f " "$MTIMES" 2>/dev/null || echo "$f $orig" >> "$MTIMES"
        done < <(patched_files "$p")
        git -C "$DPF_DIR" apply "$p"
        while read -r f; do
            orig="$(sed -n "s|^$f ||p" "$MTIMES" | head -1)"
            t=$(( orig > patch_mtime ? orig : patch_mtime ))
            set_mtime "$t" "$DPF_DIR/$f"
        done < <(patched_files "$p")
        echo "-- DPF patch applied: $(basename "$p")"
    done
    ;;
revert)
    for (( i=${#PATCHES[@]}-1; i>=0; i-- )); do
        p="${PATCHES[$i]}"
        is_applied "$p" || continue
        git -C "$DPF_DIR" apply -R "$p"
    done
    if [ -f "$MTIMES" ]; then
        while read -r f t; do
            [ -f "$DPF_DIR/$f" ] && set_mtime "$t" "$DPF_DIR/$f"
        done < "$MTIMES"
        rm -f "$MTIMES"
    fi
    ;;
*)
    echo "usage: $0 apply|revert <dpf-dir> <state-dir>" >&2
    exit 2
    ;;
esac
