#!/usr/bin/env bash
#
# Aggregator: runs every sibling project's own ./build_and_install.sh in
# turn -- amplifier, statespace, compressor, meters, reverb -- instead of `cd`-ing into each one by hand. Each
# sub-script is still the real thing doing the work (submodule fetch,
# cmake configure/build, optional test run, optional ~/.lv2/ install);
# this just loops over them and reports one combined summary at the end.
# All five build into a single shared ../mimomusic/builds/<project>/
# directory (see each project's own build_and_install.sh header comment)
# instead of a build/ nested inside each checkout -- one thing to exclude
# from Nextcloud (or any other sync tool) instead of ten.
#
# Usage:
#   ./build_and_install.sh              # build + install all five into ~/.lv2/
#   ./build_and_install.sh --clean      # wipe every project's build dir first
#   ./build_and_install.sh --no-install # build only, don't touch ~/.lv2/
#   ./build_and_install.sh --test       # also run each project's dsp_regression
#
# Every flag is simply forwarded, unchanged, to each sub-script -- they
# already share the identical flag vocabulary, so there's nothing project-
# specific to translate. A project that fails to build does NOT stop the
# others: every project is attempted, and a summary at the end lists which
# ones actually succeeded, so one broken checkout doesn't hide whether the
# other four are fine. Exits non-zero overall if any project failed.
#
# To build/install only one project, just run that project's own script
# directly (e.g. ./compressor/build_and_install.sh --test) -- this
# aggregator is purely a convenience for "build everything".

set -uo pipefail   # deliberately not -e: one project failing must not abort the loop

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECTS=(amplifier statespace compressor meters reverb)

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | grep '^#' | sed -e 's|^# ||' -e 's|^#$||'
    exit 0
fi

echo "== mimomusic: build all five plugin projects =="
echo "Projects: ${PROJECTS[*]}"
echo "Builds land in: $SCRIPT_DIR/builds/<project>/"
echo

declare -a OK=()
declare -a FAILED=()

for proj in "${PROJECTS[@]}"; do
    script="$SCRIPT_DIR/$proj/build_and_install.sh"
    echo "############################################################"
    echo "## $proj"
    echo "############################################################"
    if [ ! -f "$script" ]; then
        echo "SKIPPED: $script not found."
        FAILED+=("$proj (missing script)")
        echo
        continue
    fi
    if [ ! -x "$script" ]; then
        echo "Note: $script is not executable -- fixing (chmod +x)."
        chmod +x "$script"
    fi
    if ( cd "$SCRIPT_DIR/$proj" && "$script" "$@" ); then
        OK+=("$proj")
    else
        echo "FAILED: $proj"
        FAILED+=("$proj")
    fi
    echo
done

echo "============================================================"
echo "Summary:"
for proj in "${OK[@]}"; do
    echo "  OK      $proj"
done
for proj in "${FAILED[@]}"; do
    echo "  FAILED  $proj"
done
echo "============================================================"

if [ ${#FAILED[@]} -gt 0 ]; then
    exit 1
fi
