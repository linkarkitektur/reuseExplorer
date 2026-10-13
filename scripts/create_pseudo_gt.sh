#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Formalised pseudo-GT workflow (issue #221, maintainer's plan in
# https://github.com/pfmephisto/ReUseX/issues/221#issuecomment-5617606284,
# step "Formalise the pseudo-GT script"). Spec measured on NewOffice
# (~/repos/NewOffice/pseudo-gt/REPORT.md, results summarised in
# https://github.com/pfmephisto/ReUseX/issues/221#issuecomment-5620619174):
#
#   XFeat loop edges + `rux graph discover --no-loop-closure --loop-edges <f>`
#   + `rux optimize --loop-trust --odometry-sigma-trans 0.05` is the ONLY tested configuration in this
#   workstream that produced a real, visually-confirmed drift correction:
#   NewOffice's start<->end gap closed 23.44 m -> 16.05 m (max pose shift
#   9.80 m). Bare `--loop-trust` alone, at the default
#   `--odometry-sigma-trans 0.01`, saturates at a correction indistinguishable
#   from a zero-edge control (~0.0015 m) -- the odometry chain is simply too
#   rigid to bend, regardless of edge count or matcher quality. Both flags
#   TOGETHER are load-bearing; neither alone is.
#
#   MASt3R was measured in parallel on the same scan and, despite 267x
#   XFeat's total inlier support and 5x its PCM-surviving edge count, produced
#   no measurable closure at all (23.44 m -> 23.49 m, technically worse). It
#   stays a CC-BY-NC-SA research oracle only, reachable via
#   tools/loop_edges/export_loop_edges.py --matcher mast3r
#   --allow-noncommercial (see tools/loop_edges/README.md). This script does
#   NOT wire it in -- it is deliberately XFeat-only.
#
# WHAT "PSEUDO-GT" MEANS HERE. The output is "camera-vision maximum
# consistency" -- the best trajectory XFeat's matches plus the plane-landmark
# pose graph can reconcile from the scan's own frames. It is NOT absolute,
# geodetically-anchored ground truth. A residual start<->end gap after
# correction is the expected, honest outcome on a scan with meters-scale
# drift, and this script always prints it -- never hides a partial closure
# behind a success message. Absolute-accuracy validation needs an independent
# GT source (ARKitScenes mesh, TLS scan; see `rux analyze accuracy`).
#
# DO NOT TRUST flatness_rms / thickness_p90 ALONE (`rux analyze quality` is
# GT-free). On NewOffice, the real 9.80 m XFeat correction left flatness
# essentially flat-to-slightly-better, while MASt3R's near-zero 0.33 m
# "correction" made it slightly worse -- the opposite of what a naive read of
# "smaller number = better" would suggest. flatness/thickness score *local*
# plane consistency; they cannot tell a globally-corrected trajectory from a
# locally-smeared one. This script always reports max pose shift and the
# start<->end gap alongside quality, and labels quality as secondary.
#
# NEVER RUN AGAINST THE ORIGINAL PROJECT. Opening a `.rux` with `rux` --
# including for a read-only-looking command -- can migrate its schema
# in-place (issue #365) or, if the file is in WAL mode, advance its mtime via
# a passive checkpoint even from otherwise read-only intent. This script
# ALWAYS `cp`s the source first and only ever touches the copy.
#
# Usage:
#   scripts/create_pseudo_gt.sh -p SOURCE.rux [-o OUT_DIR] [-e VENV_PYTHON]
#                                [-m MAX_PAIRS] [-g MIN_FRAME_GAP] [-d DEVICE]
#                                [-n]
#
#   -p PROJECT   source .rux project (required; copied, never opened directly)
#   -o OUT_DIR   output directory (default: <project-dir>/pseudo-gt)
#   -e VENV_PY   XFeat matcher venv's python interpreter
#                (default: $XFEAT_VENV/bin/python if XFEAT_VENV is set,
#                else ~/loop-edges-work/xfeat/.venv/bin/python)
#   -m PAIRS     --max-pairs for export_loop_edges.py (default: 6000; per
#                REPORT.md's blocker #1, always size this to the PROJECT's
#                actual frame count, not a number copied from another run --
#                an unbounded --proposal endcap search is O(frames^2) and can
#                blow up from minutes to 10+ hours)
#   -g GAP       --min-frame-gap for export_loop_edges.py (default: 200)
#   -d DEVICE    --device for export_loop_edges.py (default: cuda)
#   -n           dry-run: print every command instead of running it
#   -h           this text
#
# Long forms --project/--out/--venv/--max-pairs/--min-frame-gap/--device/
# --dry-run/--help are accepted too (translated to the short forms below).
#
# Requires a built `rux` (override with RUX=<path>; default
# <repo>/build/apps/rux/rux) and an XFeat venv set up per
# tools/loop_edges/README.md (Apache-2.0 matcher, commercial-safe). The
# research-oracle MASt3R/MapAnything-NC path lives entirely in
# tools/loop_edges/ and is out of scope for this script by design.
#
# Output: "$OUT_DIR/<project-stem>_pseudo_gt.rux" (the corrected copy) and
# "$OUT_DIR/pseudo_gt_summary.md" (before/after numbers + mandatory caveats).
# Re-running overwrites both for the same project name.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="${RUX:-$repo_root/build/apps/rux/rux}"

project=""
out=""
venv_py="${XFEAT_VENV:-$HOME/loop-edges-work/xfeat/.venv}/bin/python"
max_pairs=6000
min_frame_gap=200
device="cuda"
dry_run=false

usage() {
  # Print the header comment block (between the shebang and `set -euo`)
  # verbatim, stripping the leading "# ". Using the file's own structure as
  # the single source of truth keeps this in sync with the header above.
  awk '
    /^#!/      { next }
    /^set -euo/ { exit }
    /^# ?/     { sub(/^# ?/, ""); print; next }
  ' "$0"
}

# Translate long options to short ones so a plain `getopts` loop (the
# convention in scripts/bench-office.sh et al.) can still parse them.
args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --project)       args+=(-p "$2"); shift 2 ;;
    --project=*)      args+=(-p "${1#*=}"); shift ;;
    --out)           args+=(-o "$2"); shift 2 ;;
    --out=*)          args+=(-o "${1#*=}"); shift ;;
    --venv)          args+=(-e "$2"); shift 2 ;;
    --venv=*)         args+=(-e "${1#*=}"); shift ;;
    --max-pairs)     args+=(-m "$2"); shift 2 ;;
    --max-pairs=*)    args+=(-m "${1#*=}"); shift ;;
    --min-frame-gap) args+=(-g "$2"); shift 2 ;;
    --min-frame-gap=*) args+=(-g "${1#*=}"); shift ;;
    --device)        args+=(-d "$2"); shift 2 ;;
    --device=*)       args+=(-d "${1#*=}"); shift ;;
    --dry-run)       args+=(-n); shift ;;
    --help)          args+=(-h); shift ;;
    *)               args+=("$1"); shift ;;
  esac
done
set -- "${args[@]}"

while getopts "p:o:e:m:g:d:nh" o; do
  case "$o" in
    p) project="$OPTARG" ;;
    o) out="$OPTARG" ;;
    e) venv_py="$OPTARG" ;;
    m) max_pairs="$OPTARG" ;;
    g) min_frame_gap="$OPTARG" ;;
    d) device="$OPTARG" ;;
    n) dry_run=true ;;
    h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done

[[ -n "$project" ]] || { echo "error: -p/--project is required" >&2; usage >&2; exit 2; }
[[ -f "$project" ]] || { echo "error: no project at $project" >&2; exit 2; }
[[ -x "$rux" ]] || { echo "error: no rux binary at $rux — build first (or set RUX=<path>)" >&2; exit 2; }

project_dir="$(cd "$(dirname "$project")" && pwd)"
project_stem="$(basename "$project" .rux)"
out="${out:-$project_dir/pseudo-gt}"
work="$out/${project_stem}_pseudo_gt.rux"
edges="$out/${project_stem}_xfeat_edges.json"
summary="$out/pseudo_gt_summary.md"
optimize_log="$out/optimize.log"

# `run` either executes a command or, under -n/--dry-run, prints it verbatim
# and does nothing — the requirement is to smoke-test argument parsing and
# the cp/paths logic without paying for a multi-hour real pipeline run.
run() {
  if $dry_run; then
    printf '[dry-run]'
    printf ' %q' "$@"
    printf '\n'
  else
    "$@"
  fi
}

echo "== pseudo-GT (issue #221) =="
echo "source project : $project"
echo "output dir     : $out"
echo "output project : $work"
echo "XFeat venv     : $venv_py"
$dry_run && echo "mode           : DRY RUN (no commands executed)"
echo

run mkdir -p "$out"

# ---------------------------------------------------------------------------
# [1/5] Copy — NEVER operate on the original (see header comment, issue #365).
# ---------------------------------------------------------------------------
echo "==> [1/5] cp source -> disposable copy"
run rm -f "$work" "$work-shm" "$work-wal"
run cp "$project" "$work"

# ---------------------------------------------------------------------------
# [2/5] Baseline: quality on the still-untouched copy + the raw seed
# start<->end gap (pure stdlib sqlite3 read, no venv/matplotlib needed — the
# same "row-major 4x4, translation = elements [3,7,11]" convention as
# tools/loop_edges/render_trajectory.py's centres()).
# ---------------------------------------------------------------------------
echo "==> [2/5] baseline: quality + seed start<->end gap"
run "$rux" -p "$work" analyze quality -o "$out/before_quality.json"

gap_of() {
  # Reads read-only (mode=ro) so this never itself risks a WAL-checkpoint
  # mtime bump (REPORT.md's blocker #3) — harmless here since it's always the
  # disposable copy, never the source, but cheap to do right anyway.
  local db="$1"
  python3 - "$db" <<'PY'
import sqlite3, struct, sys, math

con = sqlite3.connect(f"file:{sys.argv[1]}?mode=ro", uri=True)
rows = con.execute(
    "SELECT transform FROM sensor_frames "
    "WHERE transform IS NOT NULL ORDER BY node_id"
).fetchall()
con.close()
if len(rows) < 2:
    print("nan")
    raise SystemExit(0)


def centre(blob):
    v = struct.unpack("<16d", blob)  # row-major 4x4
    return v[3], v[7], v[11]


a, b = centre(rows[0][0]), centre(rows[-1][0])
print(f"{math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b))):.4f}")
PY
}

if $dry_run; then
  echo "[dry-run] gap_of $work   # start<->end gap, before"
  gap_before="n/a"
else
  gap_before="$(gap_of "$work")"
fi
echo "seed start<->end gap: ${gap_before} m"

# ---------------------------------------------------------------------------
# [3/5] XFeat wide-baseline loop edges (Apache-2.0, commercial-safe).
# --max-pairs and --min-frame-gap must be sized to THIS project's frame
# count (REPORT.md blocker #1): an unbounded --proposal endcap search is
# O(frames^2) and a number copied from a prior run on a differently-sized
# project can blow the runtime up by 50x+.
# ---------------------------------------------------------------------------
echo "==> [3/5] XFeat loop edges (--proposal endcap, --max-pairs $max_pairs)"

# NixOS gotcha (docs/sam3.1-export-guide.md §1.2; the same pattern is
# documented for the mapanything venv in tools/loop_edges/README.md): a bare
# venv's `import torch` can fail with "libstdc++.so.6: cannot open shared
# object file" because the system linker has neither gcc's libstdc++ nor the
# GL driver nor torch's bundled CUDA libs on LD_LIBRARY_PATH outside the nix
# devshell wrapper. Best-effort and additive: only computed if the paths
# actually resolve (no-op inside `nix develop`, which already has all of
# this), and scoped to ONLY the matcher subprocess via `env` below — NEVER
# exported into this script's own environment, because it would then leak
# into the `rux` (C++) calls later in this script and risk exactly the
# torch-vs-system-library collision §1.2 warns about for `trtexec`.
# Lexical only (dirname twice, no `cd`) so this is safe to compute even under
# -n/--dry-run or against a venv path that doesn't exist yet.
xfeat_venv_root="$(dirname "$(dirname "$venv_py")")"
gcc_lib="$(ls -d /nix/store/*-gcc-*-lib/lib 2>/dev/null | head -1)"
xfeat_env=()
if [[ -n "$gcc_lib" ]]; then
  nv_libs="$(echo "$xfeat_venv_root"/lib/python*/site-packages/nvidia/*/lib 2>/dev/null | tr ' ' ':')"
  xfeat_env=(env "LD_LIBRARY_PATH=${gcc_lib}:/run/opengl-driver/lib:${nv_libs}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}")
fi

run "${xfeat_env[@]}" "$venv_py" -u "$repo_root/tools/loop_edges/export_loop_edges.py" \
  "$work" --matcher xfeat --proposal endcap \
  --min-frame-gap "$min_frame_gap" --max-pairs "$max_pairs" --device "$device" \
  -o "$edges"

# ---------------------------------------------------------------------------
# [4/5] Apply — the load-bearing invocation (REPORT.md "What a
# create_pseudo_gt script should encode" #3): --loop-trust ALONE, at the
# default --odometry-sigma-trans 0.01, saturates at a correction
# indistinguishable from doing nothing. Both flags together are required.
# ---------------------------------------------------------------------------
echo "==> [4/5] graph discover --loop-edges; optimize --loop-trust --odometry-sigma-trans 0.05"
if $dry_run; then
  run "$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges"
  run "$rux" -v -p "$work" optimize --loop-trust --odometry-sigma-trans 0.05
else
  { "$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges" &&
    "$rux" -v -p "$work" optimize --loop-trust --odometry-sigma-trans 0.05; } \
    2>&1 | tee "$optimize_log"
fi

# `optimize` only rewrites stored poses — it does not touch the cloud/plane
# tables (printed explicitly by the optimizer itself), so both must be
# regenerated for `analyze quality` to reflect the corrected trajectory.
run "$rux" -p "$work" create clouds -g 0.05
run "$rux" -p "$work" create planes
run "$rux" -p "$work" analyze quality -o "$out/after_quality.json"

if $dry_run; then
  echo "[dry-run] gap_of $work   # start<->end gap, after"
  gap_after="n/a"
  max_shift="n/a"
else
  gap_after="$(gap_of "$work")"
  # "PlaneGraph: N rounds, error X -> Y, max pose shift Z m" — only printed
  # at info level, hence `-v` above.
  max_shift="$(grep -oP 'max pose shift \K[0-9.]+(?= m)' "$optimize_log" | tail -1)"
  max_shift="${max_shift:-n/a}"
fi
echo "corrected start<->end gap: ${gap_after} m"
echo "max pose shift applied   : ${max_shift} m"

# ---------------------------------------------------------------------------
# [5/5] Summary
# ---------------------------------------------------------------------------
echo "==> [5/5] writing $summary"

jget() {
  # extract a scalar from a `rux analyze quality` JSON report
  python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get(sys.argv[2],'n/a'))" \
    "$1" "$2" 2>/dev/null || echo "n/a"
}

if $dry_run; then
  echo "[dry-run] would write $summary"
else
  fl_before="$(jget "$out/before_quality.json" flatness_rms)"
  fl_after="$(jget "$out/after_quality.json" flatness_rms)"
  th_before="$(jget "$out/before_quality.json" thickness_p90)"
  th_after="$(jget "$out/after_quality.json" thickness_p90)"

  {
    echo "# Pseudo-GT summary: $project_stem"
    echo
    echo "Source: \`$project\` (never opened directly — see script header)."
    echo "Output: \`$work\`"
    echo "Matcher: XFeat (Apache-2.0, commercial-safe), \`--proposal endcap"
    echo "--min-frame-gap $min_frame_gap --max-pairs $max_pairs --device $device\`."
    echo "Apply: \`rux graph discover --no-loop-closure --loop-edges <edges>\`"
    echo "+ \`rux optimize --loop-trust --odometry-sigma-trans 0.05\`."
    echo
    echo "| Metric | Before | After |"
    echo "|---|---:|---:|"
    echo "| **Start<->end gap (m) — PRIMARY signal of correction achieved** | $gap_before | $gap_after |"
    echo "| **Max pose shift (m) — correction applied, this run** | — | $max_shift |"
    echo "| flatness_rms (m, GT-free, secondary — see caveats) | $fl_before | $fl_after |"
    echo "| thickness_p90 (m, GT-free, secondary — see caveats) | $th_before | $th_after |"
    echo
    echo "## Mandatory caveats"
    echo
    echo "- **Pseudo-GT, not absolute GT.** This is camera-vision maximum"
    echo "  consistency — the best trajectory XFeat's matches and the plane-landmark"
    echo "  pose graph can reconcile from the scan's own frames — not an"
    echo "  independently surveyed, geodetically-anchored ground truth."
    echo "- **A residual start<->end gap after correction is expected and is"
    echo "  reported above, not hidden.** flatness/thickness (GT-free, local plane"
    echo "  consistency) cannot score a global correction and must never be read as"
    echo "  the verdict on whether this run helped — on the measured NewOffice"
    echo "  precedent a real 9.80 m correction left flatness essentially flat while"
    echo "  a near-zero 0.33 m one made it slightly worse. Read the gap and the"
    echo "  pose-shift rows as primary; quality rows are secondary context."
    echo "- **No MASt3R in this script.** Measured inert on NewOffice (no drift"
    echo "  closure despite 267x XFeat's inlier support) and CC-BY-NC-SA licensed."
    echo "  The research-oracle path lives entirely in \`tools/loop_edges/\` — see"
    echo "  \`export_loop_edges.py --matcher mast3r --allow-noncommercial\` and"
    echo "  \`docs/research/loop-closure-learned-matchers.md\`."
  } > "$summary"
  cat "$summary"
fi

echo
echo "PSEUDO-GT-DONE"
