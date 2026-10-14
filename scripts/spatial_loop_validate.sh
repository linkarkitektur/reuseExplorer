#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# spatial_loop_validate.sh — end-to-end spatial loop-edge validation recipe
#
# Wraps the manual recipe documented in docs/research/regression-221-investigation.md
# §8 so you can validate the spatial proposal mode on any project in one command:
#
#   1. Generate spatial loop edges via a matcher venv
#   2. Run rux graph discover --loop-edges + rux optimize on a COPY of the
#      project (non-destructive)
#   3. Rebuild the point cloud and render before/after top-down PNGs
#
# USAGE
#   scripts/spatial_loop_validate.sh [OPTIONS] PROJECT.rux
#
# OPTIONS
#   -o OUTPUT_DIR       Where to write before.png, after.png, edges.json
#                       (default: ./spatial_validate_out)
#   -v VENV             Matcher Python interpreter
#                       (default: ~/loop-edges-work/xfeat/.venv/bin/python)
#   -m MATCHER          Matcher backend: xfeat|mast3r|orb|lightglue
#                       (default: xfeat)
#   -r RADIUS           --spatial-radius in metres (default: 3.0)
#   -g MIN_FRAME_GAP    --min-frame-gap (default: 300)
#   -M MAX_PAIRS        --max-pairs for the matcher run (default: 2000)
#   -s SIGMA_TRANS      --odometry-sigma-trans for rux optimize (default: 0.01)
#   -R RUX              Path to the rux binary (default: rux from PATH)
#   -T TOOL             Path to export_loop_edges.py
#                       (default: tools/loop_edges/export_loop_edges.py
#                        relative to the repo root, auto-detected)
#   --allow-noncommercial
#                       Pass --allow-noncommercial to the matcher (required for
#                       mast3r and mapanything --variant nc)
#   -h / --help         Print this help and exit
#
# ENVIRONMENT (NixOS / venv libstdc++ issue)
#   Handled automatically.  At startup the script probes `import torch`; if it
#   fails it tries each /nix/store/*gcc-*-lib/lib candidate (newest gcc version
#   first) until the import succeeds.  The resolved path is scoped only to the
#   matcher subprocess — never exported — so rux C++ calls are unaffected.  A
#   pre-set LD_LIBRARY_PATH that already makes the import succeed is honoured
#   as-is.  If no candidate works the script exits 2 with a diagnostic.
#
# EXAMPLE
#   # XFeat (commercial-safe, default):
#   scripts/spatial_loop_validate.sh -o ./validate_out project.rux
#
#   # MASt3R oracle (research only):
#   scripts/spatial_loop_validate.sh \
#     -v ~/loop-edges-work/mast3r/.venv/bin/python \
#     -m mast3r --allow-noncommercial \
#     -o ./validate_out project.rux
#
# EXIT CODES
#   0   success
#   1   argument error
#   2   dependency missing
#   3   runtime failure

set -euo pipefail

# --------------------------------------------------------------------------- #
# Defaults                                                                      #
# --------------------------------------------------------------------------- #
OUTPUT_DIR="./spatial_validate_out"
VENV="${HOME}/loop-edges-work/xfeat/.venv/bin/python"
MATCHER="xfeat"
SPATIAL_RADIUS="3.0"
MIN_FRAME_GAP="300"
MAX_PAIRS="2000"
SIGMA_TRANS="0.01"
RUX="rux"
TOOL=""               # auto-detected below
ALLOW_NONCOMMERCIAL=""
PROJECT=""

# --------------------------------------------------------------------------- #
# Argument parsing                                                               #
# --------------------------------------------------------------------------- #
usage() {
    grep '^#' "$0" | grep -v '^#!/' | sed 's/^# \?//' | head -60
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        -o) OUTPUT_DIR="$2"; shift 2 ;;
        -v) VENV="$2"; shift 2 ;;
        -m) MATCHER="$2"; shift 2 ;;
        -r) SPATIAL_RADIUS="$2"; shift 2 ;;
        -g) MIN_FRAME_GAP="$2"; shift 2 ;;
        -M) MAX_PAIRS="$2"; shift 2 ;;
        -s) SIGMA_TRANS="$2"; shift 2 ;;
        -R) RUX="$2"; shift 2 ;;
        -T) TOOL="$2"; shift 2 ;;
        --allow-noncommercial) ALLOW_NONCOMMERCIAL="--allow-noncommercial"; shift ;;
        -*)
            echo "ERROR: unknown option $1" >&2
            echo "Run with -h for usage." >&2
            exit 1
            ;;
        *)
            if [[ -n "$PROJECT" ]]; then
                echo "ERROR: unexpected positional argument: $1" >&2
                exit 1
            fi
            PROJECT="$1"
            shift
            ;;
    esac
done

if [[ -z "$PROJECT" ]]; then
    echo "ERROR: PROJECT.rux argument is required." >&2
    echo "Run with -h for usage." >&2
    exit 1
fi

# --------------------------------------------------------------------------- #
# Resolve paths                                                                  #
# --------------------------------------------------------------------------- #
PROJECT="$(realpath "$PROJECT")"
if [[ ! -f "$PROJECT" ]]; then
    echo "ERROR: project file not found: $PROJECT" >&2
    exit 1
fi

# Auto-detect tool path relative to the repo root (this script lives in scripts/)
if [[ -z "$TOOL" ]]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    REPO_ROOT="$(dirname "$SCRIPT_DIR")"
    TOOL="${REPO_ROOT}/tools/loop_edges/export_loop_edges.py"
fi

if [[ ! -f "$TOOL" ]]; then
    echo "ERROR: cannot find export_loop_edges.py at: $TOOL" >&2
    echo "       Use -T to specify its path explicitly." >&2
    exit 2
fi

# --------------------------------------------------------------------------- #
# Dependency checks                                                              #
# --------------------------------------------------------------------------- #
if ! command -v "$RUX" &>/dev/null && [[ ! -x "$RUX" ]]; then
    echo "ERROR: rux binary not found: $RUX" >&2
    echo "       Build the project first or use -R to specify its path." >&2
    exit 2
fi

if [[ ! -x "$VENV" ]]; then
    echo "ERROR: matcher Python interpreter not found: $VENV" >&2
    echo "       Set up the venv first (see tools/loop_edges/README.md)" >&2
    echo "       or use -v to specify a different interpreter." >&2
    exit 2
fi

# --------------------------------------------------------------------------- #
# LD_LIBRARY_PATH sniff (NixOS / venv libstdc++ issue)                         #
# --------------------------------------------------------------------------- #
# Probe import torch with the current environment first.  If it fails (e.g.    #
# libstdc++.so.6 not found outside nix develop), iterate over gcc-*-lib        #
# candidates in descending version order and stop at the first one that works. #
# VENV_ENV is scoped to the matcher subprocess only — never exported globally  #
# — so rux C++ calls later in this script are unaffected.                      #
VENV_ENV=()
if ! "$VENV" -c "import torch" &>/dev/null; then
    _venv_root="$(dirname "$(dirname "$VENV")")"
    _nv_libs="$(echo "${_venv_root}"/lib/python*/site-packages/nvidia/*/lib 2>/dev/null | tr ' ' ':')"
    _found=""
    # shellcheck disable=SC2012
    while IFS= read -r _gcc_lib; do
        _candidate="${_gcc_lib}:/run/opengl-driver/lib:${_nv_libs}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
        if LD_LIBRARY_PATH="$_candidate" "$VENV" -c "import torch" &>/dev/null; then
            _found="$_gcc_lib"
            VENV_ENV=(env "LD_LIBRARY_PATH=${_candidate}")
            break
        fi
    done < <(ls -d /nix/store/*-gcc-*-lib/lib 2>/dev/null | sort -rV)
    if [[ -z "$_found" ]]; then
        echo "ERROR: venv cannot import torch and no gcc-lib candidate resolved it." >&2
        echo "       Set LD_LIBRARY_PATH manually before calling this script." >&2
        echo "       See tools/loop_edges/README.md for the recipe." >&2
        exit 2
    fi
    echo "  [ld-sniff] resolved libstdc++ via: $_found"
fi

# --------------------------------------------------------------------------- #
# Prepare output directory and working copy                                      #
# --------------------------------------------------------------------------- #
mkdir -p "$OUTPUT_DIR"
OUTPUT_DIR="$(realpath "$OUTPUT_DIR")"

WORK_RUX="${OUTPUT_DIR}/work.rux"
EDGES_JSON="${OUTPUT_DIR}/edges_spatial.json"
BEFORE_PNG="${OUTPUT_DIR}/before.png"
AFTER_PNG="${OUTPUT_DIR}/after.png"

echo "=== spatial_loop_validate ==="
echo "  project:       $PROJECT"
echo "  output:        $OUTPUT_DIR"
echo "  matcher venv:  $VENV"
echo "  matcher:       $MATCHER"
echo "  spatial-radius: ${SPATIAL_RADIUS} m"
echo "  min-frame-gap: $MIN_FRAME_GAP"
echo "  max-pairs:     $MAX_PAIRS"
echo "  sigma-trans:   $SIGMA_TRANS"
echo "  rux:           $(command -v "$RUX" 2>/dev/null || echo "$RUX")"
echo ""

# --------------------------------------------------------------------------- #
# Step 1: render before (original project, unmodified)                          #
# --------------------------------------------------------------------------- #
echo "[1/4] Rendering before.png from original project..."
"$RUX" -p "$PROJECT" create clouds -g 0.05 2>&1 \
    | sed 's/^/  [rux] /' || {
    echo "  NOTE: create clouds failed — project may already have a cloud." >&2
    echo "        Continuing with existing cloud." >&2
}
"$RUX" -p "$PROJECT" render -o "$BEFORE_PNG" --view top --layers cloud
echo "  -> $BEFORE_PNG"

# --------------------------------------------------------------------------- #
# Step 2: generate spatial loop edges                                            #
# --------------------------------------------------------------------------- #
echo "[2/4] Generating spatial loop edges (matcher=$MATCHER, radius=${SPATIAL_RADIUS}m)..."
# shellcheck disable=SC2086
"${VENV_ENV[@]}" "$VENV" "$TOOL" \
    "$PROJECT" \
    -o "$EDGES_JSON" \
    --matcher "$MATCHER" \
    --proposal spatial \
    --spatial-radius "$SPATIAL_RADIUS" \
    --min-frame-gap "$MIN_FRAME_GAP" \
    --max-pairs "$MAX_PAIRS" \
    $ALLOW_NONCOMMERCIAL \
    2>&1 | sed 's/^/  [matcher] /'
echo "  -> $EDGES_JSON"

EDGE_COUNT=$(python3 -c "
import json, sys
d = json.load(open(sys.argv[1]))
print(len(d.get('edges', [])))
" "$EDGES_JSON" 2>/dev/null || echo "?")
echo "  edges exported: $EDGE_COUNT"

if [[ "$EDGE_COUNT" == "0" || "$EDGE_COUNT" == "?" ]]; then
    echo "WARNING: no edges exported — the optimizer run will be a no-op." >&2
fi

# --------------------------------------------------------------------------- #
# Step 3: optimize on a working copy                                             #
# --------------------------------------------------------------------------- #
echo "[3/4] Copying project and running optimizer on working copy..."
cp "$PROJECT" "$WORK_RUX"
"$RUX" -p "$WORK_RUX" graph discover --no-loop-closure \
    --loop-edges "$EDGES_JSON" \
    2>&1 | sed 's/^/  [rux graph discover] /'
"$RUX" -p "$WORK_RUX" optimize \
    --loop-trust \
    --odometry-sigma-trans "$SIGMA_TRANS" \
    2>&1 | sed 's/^/  [rux optimize] /'

# Rebuild cloud from optimized poses
"$RUX" -p "$WORK_RUX" create clouds -g 0.05 \
    2>&1 | sed 's/^/  [rux create clouds] /'

# --------------------------------------------------------------------------- #
# Step 4: render after                                                           #
# --------------------------------------------------------------------------- #
echo "[4/4] Rendering after.png from optimized project..."
"$RUX" -p "$WORK_RUX" render -o "$AFTER_PNG" --view top --layers cloud
echo "  -> $AFTER_PNG"

# --------------------------------------------------------------------------- #
# Summary                                                                        #
# --------------------------------------------------------------------------- #
echo ""
echo "=== Done ==="
echo "  before:  $BEFORE_PNG"
echo "  after:   $AFTER_PNG"
echo "  edges:   $EDGES_JSON  ($EDGE_COUNT exported; PCM count in optimizer log above)"
echo "  work db: $WORK_RUX  (the original project is unchanged)"
echo ""
echo "Compare the two renders visually.  A clean spatial correction shows:"
echo "  - No doubled walls or ghost edges at revisited areas"
echo "  - Sharper wall lines and tighter building footprint"
echo "  - No global shear (building silhouette stays the same shape)"
echo ""
echo "If you see shear: try tightening -s (--odometry-sigma-trans); default"
echo "0.01 is correct for spatial edges (endcap-only edges needed 0.05 and"
echo "produced a global shear — that combination is the regression #221 fixed)."
