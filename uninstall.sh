#!/usr/bin/env bash
#
# Removes the installed MimoMusic plugins -- the counterpart of
# build_and_install.sh / build_and_install_macos.sh.
#
# Only this repository's own plugins are touched. Their names are read
# from each project's CMakeLists.txt (every dpf_add_plugin(...) target),
# and a bundle is only removed if it really is a MimoMusic plugin: an LV2
# bundle must declare a mimomusic.art plugin URI, a VST3/AU bundle must
# contain the MimoMusic name. Other plugins in the same folders, even one
# with the same file name, are never removed.
#
# Looked for:
#   Linux:  ~/.lv2/<name>.lv2, ~/.vst3/<name>.vst3
#   macOS:  ~/Library/Audio/Plug-Ins/LV2/<name>.lv2
#           ~/Library/Audio/Plug-Ins/VST3/<name>.vst3
#           ~/Library/Audio/Plug-Ins/Components/<name>.component
#
# The build output in builds/ is left alone (delete that folder by hand
# if you want the disk space back).
#
# Usage:
#   ./uninstall.sh                  # list what is installed, ask, then remove
#   ./uninstall.sh reverb meters    # only these projects' plugins
#   ./uninstall.sh --dry-run        # only list, remove nothing
#   ./uninstall.sh --yes            # remove without asking
#
# Restart Ardour (or rescan plugins) afterwards. Sessions that use a
# removed plugin still open; the plugin is shown as missing.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ALL_PROJECTS=(amplifier statespace compressor meters reverb)

DRY_RUN=0
ASSUME_YES=0
PROJECTS=()
for arg in "$@"; do
    case "$arg" in
        -h|--help)
            sed -n '2,/^[^#]/p' "${BASH_SOURCE[0]}" | command grep '^#' | sed -e 's|^# \{0,1\}||'
            exit 0 ;;
        --dry-run) DRY_RUN=1 ;;
        --yes|-y)  ASSUME_YES=1 ;;
        -*)        echo "Unknown option: $arg (see --help)" >&2; exit 1 ;;
        *)         if [[ " ${ALL_PROJECTS[*]} " != *" $arg "* ]]; then
                       echo "Unknown project: $arg (known: ${ALL_PROJECTS[*]})" >&2; exit 1
                   fi
                   PROJECTS+=("$arg") ;;
    esac
done
[ ${#PROJECTS[@]} -gt 0 ] || PROJECTS=("${ALL_PROJECTS[@]}")

case "$(uname -s)" in
    Darwin)
        PLUGIN_ROOT="$HOME/Library/Audio/Plug-Ins"
        LOCATIONS=("$PLUGIN_ROOT/LV2:lv2" "$PLUGIN_ROOT/VST3:vst3" "$PLUGIN_ROOT/Components:component") ;;
    *)
        LOCATIONS=("$HOME/.lv2:lv2" "$HOME/.vst3:vst3") ;;
esac

# --- 1. This repository's plugin names -----------------------------------
NAMES=()
for proj in "${PROJECTS[@]}"; do
    cmake_file="$SCRIPT_DIR/$proj/CMakeLists.txt"
    if [ ! -f "$cmake_file" ]; then
        echo "Note: $cmake_file not found -- skipping $proj." >&2
        continue
    fi
    while IFS= read -r name; do
        NAMES+=("$name")
    done < <(sed -n -E 's/^[[:space:]]*dpf_add_plugin\([[:space:]]*([A-Za-z0-9_]+).*/\1/p' "$cmake_file")
done
[ ${#NAMES[@]} -gt 0 ] || { echo "No plugin names found -- nothing to do." >&2; exit 1; }

# --- 2. Find the installed bundles and check they are ours ---------------
is_ours() {   # $1 = bundle path, $2 = extension
    if [ "$2" = lv2 ]; then
        command grep -qi 'mimomusic\.art' "$1"/*.ttl 2>/dev/null
    else
        command grep -rqia 'mimomusic' "$1" 2>/dev/null
    fi
}

TO_REMOVE=()
for loc in "${LOCATIONS[@]}"; do
    dir="${loc%:*}"; ext="${loc##*:}"
    for name in "${NAMES[@]}"; do
        bundle="$dir/$name.$ext"
        [ -e "$bundle" ] || continue
        if is_ours "$bundle" "$ext"; then
            TO_REMOVE+=("$bundle")
        else
            echo "Skipped (not a MimoMusic plugin): $bundle"
        fi
    done
done

if [ ${#TO_REMOVE[@]} -eq 0 ]; then
    echo "No installed MimoMusic plugins found for: ${PROJECTS[*]}"
    exit 0
fi

echo "Installed MimoMusic plugins:"
for bundle in "${TO_REMOVE[@]}"; do
    echo "  $bundle"
done

if [ "$DRY_RUN" -eq 1 ]; then
    echo
    echo "--dry-run: nothing was removed."
    exit 0
fi

if [ "$ASSUME_YES" -eq 0 ]; then
    echo
    read -r -p "Remove these ${#TO_REMOVE[@]} bundles? [y/N] " answer
    case "$answer" in
        y|Y|yes|YES) ;;
        *) echo "Nothing was removed."; exit 0 ;;
    esac
fi

# --- 3. Remove --------------------------------------------------------------
failed=0
for bundle in "${TO_REMOVE[@]}"; do
    if rm -rf -- "${bundle:?}"; then
        echo "  removed $bundle"
    else
        echo "  FAILED to remove $bundle" >&2
        failed=1
    fi
done
echo
echo "Done. Restart Ardour (or rescan plugins) so it forgets the removed plugins."
exit "$failed"
