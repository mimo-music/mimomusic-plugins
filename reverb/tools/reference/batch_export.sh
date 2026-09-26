#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
#
# Runs generate_reference_samples once per point in a parameter grid
# (Room Length/Width/Height, target T60, image order), each into its own
# subdirectory, all recorded into one manifest.csv (via
# generate_reference_samples's own --manifest-csv, so this script never
# has to guess or reconstruct filenames -- the tool that writes a file is
# the one that records it), and finishes by building an index.html next
# to it so the whole export can be browsed and listened to in a browser.
# Meant for long (e.g. overnight) runs over a min/max/step grid of
# settings, stored so a learning algorithm can see exactly which settings
# produced which file, and easy to browse and listen to.
#
# Usage:
#   ./batch_export.sh --out-dir DIR [options]
#
#   Sweep options -- for each of length/width/height/t60/order, MIN, MAX
#   and STEP (all in the same units the underlying tool uses: meters for
#   room dims, seconds for T60, an integer for order). Setting MIN==MAX
#   (or leaving STEP at 0) fixes that parameter instead of sweeping it --
#   this is how you sweep only some settings and hold the rest still.
#     --length-min/--length-max/--length-step   (default 10/10/0 -- fixed)
#     --width-min/--width-max/--width-step      (default 7/7/0 -- fixed)
#     --height-min/--height-max/--height-step   (default 3.5/3.5/0 -- fixed)
#     --t60-min/--t60-max/--t60-step            (default 1.8/1.8/0 -- fixed)
#     --order-min/--order-max/--order-step      (default 3/3/0 -- fixed)
#
#   Other options:
#     --sample-rate HZ      shared across every run (default 48000)
#     --user-dry-dir DIR    shared dry-recordings folder every run reads
#                           from (default <out-dir>/user_dry -- create it
#                           and drop your own .wav files there before
#                           running this script; the 4 synthesized
#                           signals -- impulse/clap/noiseburst/sweep --
#                           are always included on top of whatever's there)
#     --binary PATH         path to the generate_reference_samples
#                           executable (default: build it via
#                           ./build_standalone.sh next to this script if
#                           it isn't already built there)
#     --dry-run             print the grid and exit without rendering
#                           anything -- use this first to sanity-check
#                           the combination count before an overnight run
#
# Writes, under --out-dir: one subdirectory per grid point (named
# L<len>_W<wid>_H<hgt>_T<t60>_O<order>, holding just that combination's own
# "*_reference.wav" renders), one shared dry/ folder (the "*_dry.wav" files
# -- same signal for every grid point since --sample-rate is fixed across
# the whole sweep, so each is written once instead of duplicated into
# every grid point's own subdirectory), manifest.csv (one row per
# dry/reference pair across the WHOLE sweep), and index.html (a
# filterable, sortable table with an inline audio player per file --
# open it in a browser to browse and listen). If your browser refuses to
# play file:// audio, run `python3 -m http.server` inside --out-dir and
# open http://localhost:8000/index.html instead.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

OUT_DIR=""
LENGTH_MIN=10.0; LENGTH_MAX=10.0; LENGTH_STEP=0
WIDTH_MIN=7.0;   WIDTH_MAX=7.0;   WIDTH_STEP=0
HEIGHT_MIN=3.5;  HEIGHT_MAX=3.5;  HEIGHT_STEP=0
T60_MIN=1.8;     T60_MAX=1.8;     T60_STEP=0
ORDER_MIN=3;     ORDER_MAX=3;     ORDER_STEP=0
SAMPLE_RATE=48000
USER_DRY_DIR=""
BINARY=""
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --out-dir) OUT_DIR="$2"; shift 2 ;;
        --length-min) LENGTH_MIN="$2"; shift 2 ;;
        --length-max) LENGTH_MAX="$2"; shift 2 ;;
        --length-step) LENGTH_STEP="$2"; shift 2 ;;
        --width-min) WIDTH_MIN="$2"; shift 2 ;;
        --width-max) WIDTH_MAX="$2"; shift 2 ;;
        --width-step) WIDTH_STEP="$2"; shift 2 ;;
        --height-min) HEIGHT_MIN="$2"; shift 2 ;;
        --height-max) HEIGHT_MAX="$2"; shift 2 ;;
        --height-step) HEIGHT_STEP="$2"; shift 2 ;;
        --t60-min) T60_MIN="$2"; shift 2 ;;
        --t60-max) T60_MAX="$2"; shift 2 ;;
        --t60-step) T60_STEP="$2"; shift 2 ;;
        --order-min) ORDER_MIN="$2"; shift 2 ;;
        --order-max) ORDER_MAX="$2"; shift 2 ;;
        --order-step) ORDER_STEP="$2"; shift 2 ;;
        --sample-rate) SAMPLE_RATE="$2"; shift 2 ;;
        --user-dry-dir) USER_DRY_DIR="$2"; shift 2 ;;
        --binary) BINARY="$2"; shift 2 ;;
        --dry-run) DRY_RUN=1; shift 1 ;;
        -h|--help)
            sed -n '2,/^set -euo/p' "${BASH_SOURCE[0]}" | grep '^#' | sed -e 's|^# ||' -e 's|^#$||'
            exit 0
            ;;
        *) echo "Unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

if [ -z "$OUT_DIR" ]; then
    echo "ERROR: --out-dir is required" >&2
    exit 1
fi

[ -z "$USER_DRY_DIR" ] && USER_DRY_DIR="$OUT_DIR/user_dry"

# --- Find (or build) the binary -----------------------------------------
if [ -z "$BINARY" ]; then
    STANDALONE="$SCRIPT_DIR/generate_reference_samples"
    CMAKE_BUILT="$SCRIPT_DIR/../../../builds/reverb/build/bin/generate_reference_samples"
    if [ -x "$STANDALONE" ]; then
        BINARY="$STANDALONE"
    elif [ -x "$CMAKE_BUILT" ]; then
        BINARY="$CMAKE_BUILT" # already built via ./build_and_install.sh -- reuse it rather than building a second copy
    else
        echo "== generate_reference_samples not built yet -- building it =="
        "$SCRIPT_DIR/build_standalone.sh"
        BINARY="$STANDALONE"
    fi
fi
if [ ! -x "$BINARY" ]; then
    echo "ERROR: $BINARY not found/executable after build attempt" >&2
    exit 1
fi

# --- Expand each sweep into a list of values -----------------------------
# MIN==MAX (or a non-positive step) means "just this one value", not an
# error -- the natural way to hold a parameter fixed while sweeping
# others, matching this script's own header comment.
expand_range() {
    local min="$1" max="$2" step="$3"
    if awk -v a="$min" -v b="$max" 'BEGIN{exit !(a>=b)}' || awk -v s="$step" 'BEGIN{exit !(s<=0)}'; then
        echo "$min"
        return
    fi
    seq -f '%g' "$min" "$step" "$max"
}

mapfile -t LENGTHS < <(expand_range "$LENGTH_MIN" "$LENGTH_MAX" "$LENGTH_STEP")
mapfile -t WIDTHS  < <(expand_range "$WIDTH_MIN" "$WIDTH_MAX" "$WIDTH_STEP")
mapfile -t HEIGHTS < <(expand_range "$HEIGHT_MIN" "$HEIGHT_MAX" "$HEIGHT_STEP")
mapfile -t T60S    < <(expand_range "$T60_MIN" "$T60_MAX" "$T60_STEP")
mapfile -t ORDERS  < <(expand_range "$ORDER_MIN" "$ORDER_MAX" "$ORDER_STEP")

TOTAL=$(( ${#LENGTHS[@]} * ${#WIDTHS[@]} * ${#HEIGHTS[@]} * ${#T60S[@]} * ${#ORDERS[@]} ))

echo "== Batch export grid =="
echo "Length (m): ${LENGTHS[*]}"
echo "Width  (m): ${WIDTHS[*]}"
echo "Height (m): ${HEIGHTS[*]}"
echo "T60    (s): ${T60S[*]}"
echo "Order     : ${ORDERS[*]}"
echo "Total combinations: $TOTAL (x 4 synthesized signals, plus any files in $USER_DRY_DIR, each)"
echo "Output dir: $OUT_DIR"
echo

if [ "$DRY_RUN" -eq 1 ]; then
    echo "--dry-run given: stopping here, nothing rendered."
    exit 0
fi

mkdir -p "$OUT_DIR" "$USER_DRY_DIR"
MANIFEST="$OUT_DIR/manifest.csv"
rm -f "$MANIFEST" # a fresh export each run, so old grid points from a previous sweep shape don't linger in the manifest/index

# Resolve every path that might have been given relative to the ORIGINAL
# cwd to absolute form before we cd below -- otherwise a relative
# --user-dry-dir/--binary breaks once we're sitting inside $OUT_DIR.
BINARY_ABS="$(cd "$(dirname "$BINARY")" && pwd)/$(basename "$BINARY")"
USER_DRY_DIR_ABS="$(cd "$USER_DRY_DIR" && pwd)"

# All paths generate_reference_samples records into the manifest are
# taken exactly as given on its own command line (see that tool's own
# RunSettings::runDir comment) -- running from inside $OUT_DIR with a
# plain relative subdirectory name is what makes those paths resolve
# correctly from index.html sitting right next to manifest.csv.
cd "$OUT_DIR"

# One shared dry-file folder for the WHOLE sweep (--sample-rate is fixed
# across every grid point, so the dry signal -- synthesized excitation or
# a user file's own mono downmix -- never differs between combinations;
# see generate_reference_samples.cpp's own --dry-out-dir help text). Every
# invocation below points at this same folder so the dry .wav files get
# written once instead of duplicated into all $TOTAL run directories.
DRY_OUT_DIR_ABS="$(pwd)/dry"
mkdir -p "$DRY_OUT_DIR_ABS"

COUNT=0
START_TIME=$(date +%s)
for L in "${LENGTHS[@]}"; do
  for W in "${WIDTHS[@]}"; do
    for H in "${HEIGHTS[@]}"; do
      for T in "${T60S[@]}"; do
        for O in "${ORDERS[@]}"; do
          COUNT=$((COUNT + 1))
          RUN_DIR=$(printf 'L%s_W%s_H%s_T%s_O%s' "$L" "$W" "$H" "$T" "$O")
          ELAPSED=$(( $(date +%s) - START_TIME ))
          echo "[$COUNT/$TOTAL] $RUN_DIR (${ELAPSED}s elapsed)"
          "$BINARY_ABS" "$RUN_DIR" \
              --length "$L" --width "$W" --height "$H" \
              --target-t60 "$T" --order "$O" \
              --sample-rate "$SAMPLE_RATE" \
              --user-dry-dir "$USER_DRY_DIR_ABS" \
              --dry-out-dir "$DRY_OUT_DIR_ABS" \
              --manifest-csv "manifest.csv" \
              > "$RUN_DIR.log.txt" 2>&1 \
          || { echo "  FAILED -- see $RUN_DIR.log.txt" >&2; }
        done
      done
    done
  done
done

TOTAL_TIME=$(( $(date +%s) - START_TIME ))
echo
echo "Done: $COUNT combinations in ${TOTAL_TIME}s. Manifest: $MANIFEST"

# --- Build index.html from manifest.csv ----------------------------------
"$SCRIPT_DIR/build_index_html.sh" "$OUT_DIR/manifest.csv" "$OUT_DIR/index.html"
echo "Wrote $OUT_DIR/index.html -- open it in a browser to browse and listen."
echo "(If local audio playback is blocked by your browser's file:// restrictions,"
echo " run 'python3 -m http.server' inside $OUT_DIR and open http://localhost:8000/index.html instead.)"
