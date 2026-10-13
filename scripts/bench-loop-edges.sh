#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Loop-closure matcher benchmark (issue #221/#225 P2). Measures the effect of
# externally-computed wide-baseline loop edges (from tools/loop_edges/, produced
# by ORB / XFeat / MASt3R / MapAnything) on pose quality, all fed through the
# license-clean `rux graph discover --loop-edges` bridge (then `rux optimize`).
#
# For a given scan it builds a fresh copy per variant, runs the identical
# reconstruction pipeline (deterministic), and scores it with `rux analyze
# quality` (GT-free flatness/thickness) and, if a ground-truth PLY is given,
# `rux analyze accuracy` (F-score @ threshold). Variants:
#   base                 stored poses, no pose stage
#   optimize             plane-graph optimize (current default)
#   optimize+<edges>     graph discover --loop-edges <file>; optimize --loop-trust
#                        (one per file)
#
# Usage:
#   scripts/bench-loop-edges.sh <project.rux> <out_dir> <edges_dir> [gt.ply]
# where <edges_dir> holds *.json edge files (schema reusex.loop_edges.v1).
# Re-runnable; each variant is isolated in its own .rux copy under <out_dir>.
#
# HOW TO READ THE SUMMARY (issue #312). The PRIMARY comparison columns are
# `pcm_kept` (of `pcm_in`) and `max_shift_m`:
#   pcm_kept     edges that survived Pairwise Consistency Maximization, i.e.
#                that agree with EACH OTHER about the trajectory. For a
#                file-edge (--loop-edges) variant this is the matcher's own
#                edge count
#                (external_total - external_rejected), not the union.
#   max_shift_m  the correction actually applied to the worst-displaced pose.
# `edges_emitted` and `sum_inliers` are SECONDARY diagnostics describing the
# matcher's per-pair confidence in isolation, which the pose graph does not
# consume. A matcher can lead on both and still be last where it counts: on the
# office endcap set MapAnything emitted 148 edges with 9203 inliers, PCM kept 3,
# and the applied correction was 0.21 m on a scan needing ~16 m (#264). Rank on
# pcm_kept + max_shift_m; use sum_inliers only to explain a result, never to
# pick a winner. See docs/research/loop-closure-learned-matchers.md §5.1.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# Override with RUX=<path> to score a checkout that has no build/ of its own
# (e.g. a docs-only worktree reusing the main build).
rux="${RUX:-$repo_root/build/apps/rux/rux}"

project="${1:?usage: bench-loop-edges.sh <project.rux> <out_dir> <edges_dir> [gt.ply]}"
out_dir="${2:?missing out_dir}"
edges_dir="${3:?missing edges_dir}"
gt="${4:-}"

mkdir -p "$out_dir"
summary="$out_dir/summary.tsv"
# Primary columns first (pcm_kept/pcm_in/max_shift_m), then the quality scores,
# then the per-pair front-end diagnostics. See the header comment for why.
printf "variant\tpcm_kept\tpcm_in\tmax_shift_m\tflatness_rms_mm\tthickness_p90_mm\tGT_Fscore\tedges_emitted\tsum_inliers\tgraph_edges\n" > "$summary"

# extract a scalar from a rux JSON report (key at top level)
jget() { python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get(sys.argv[2],''))" "$1" "$2" 2>/dev/null || echo ""; }

# Front-end diagnostics straight from the matcher's edge file (schema
# reusex.loop_edges.v1): how many edges it emitted and their total RANSAC inlier
# support. Prints "<edges>\t<sum_inliers>"; empty fields if the file is unusable.
edge_file_stats() {
  python3 - "$1" <<'PY' 2>/dev/null || printf "\t\n"
import json, sys
try:
    edges = json.load(open(sys.argv[1])).get("edges", [])
except Exception:
    print("\t"); sys.exit(0)
print(f"{len(edges)}\t{sum(int(e.get('inliers', 0) or 0) for e in edges)}")
PY
}

# PRIMARY metric extraction (#312). Parses the `rux graph discover` +
# `rux optimize` log for the consistency outcome; prints
# "<pcm_kept>\t<pcm_in>\t<graph_edges>\t<max_shift_m>".
#
# The library logs (src/slam/graph_discover.cpp):
#   "PlaneGraph: PCM kept K of N unioned loop edges; R of T external edges
#    rejected as inconsistent"
# K/N describe the UNION (external + internally detected + panorama). When a
# matcher file was fed in, the number that ranks the MATCHER is T-R of T, so
# prefer the per-source pair whenever the line names a source. Falls back to the
# union pair, then to the internal detector's own PCM line. When PCM did not run
# at all (fewer than 3 edges, --loop-no-pcm, or a binary predating the log line)
# everything that reached the graph "survived", so report `graph_edges` kept out
# of the post-gating / as-loaded count rather than leaving the column blank.
parse_optimize_log() {
  python3 - "$1" <<'PY' 2>/dev/null || printf "\t\t\t\n"
import re, sys
txt = open(sys.argv[1], errors="replace").read()

kept = kin = ""
per_source = re.findall(
    r"PCM kept (\d+) of (\d+) unioned loop edges;\s*"
    r"(\d+) of (\d+) (?:external|panorama) edges rejected", txt)
if per_source:
    _, _, rejected, total = per_source[-1]
    kept, kin = str(int(total) - int(rejected)), total
else:
    union = re.findall(r"PCM kept (\d+) of (\d+) unioned loop edges", txt)
    internal = re.findall(
        r"LoopClosure: PCM kept (\d+) of (\d+) edges as mutually consistent", txt)
    if union:
        kept, kin = union[-1]
    elif internal:
        kept, kin = internal[-1]

graph = re.findall(r"Loop closure: (\d+) wide-baseline edges added", txt)
graph = graph[-1] if graph else "0"
if not kept:
    kept = graph
    gated = re.findall(r"(\d+) external loop edges kept after gating", txt)
    loaded = re.findall(r"PlaneGraph: (\d+) external loop edges loaded", txt)
    kin = (gated or loaded or [graph])[-1]

shift = re.findall(r"max pose shift ([0-9.]+)", txt)
print(f"{kept}\t{kin}\t{graph}\t{shift[-1] if shift else ''}")
PY
}

run_variant() {
  local name="$1"; shift
  local edge_file="$1"; shift   # "" for none
  local rux_copy="$out_dir/$name.rux"
  echo "==================== variant: $name ===================="
  rm -f "$rux_copy" "$rux_copy"-shm "$rux_copy"-wal
  cp -f "$project" "$rux_copy"

  local opt_log="$out_dir/$name.optimize.log"
  local pcm_kept=0 pcm_in=0 graph_edges=0 shift_m=""
  local emitted="" sum_inliers=""
  if [[ -n "$edge_file" ]]; then
    IFS=$'\t' read -r emitted sum_inliers < <(edge_file_stats "$edge_file")
  fi
  if [[ "$name" != "base" ]]; then
    if [[ -n "$edge_file" ]]; then
      { "$rux" -v -p "$rux_copy" graph discover --no-loop-closure \
          --loop-edges "$edge_file" &&
        "$rux" -v -p "$rux_copy" optimize --loop-trust \
          --odometry-sigma-trans 0.05 "$@"; } > "$opt_log" 2>&1 || true
    else
      "$rux" -v -p "$rux_copy" optimize "$@" > "$opt_log" 2>&1 || true
    fi
    IFS=$'\t' read -r pcm_kept pcm_in graph_edges shift_m \
      < <(parse_optimize_log "$opt_log")
  fi

  "$rux" -p "$rux_copy" create clouds  > "$out_dir/$name.clouds.log" 2>&1
  "$rux" -p "$rux_copy" create planes  > "$out_dir/$name.planes.log" 2>&1
  "$rux" -p "$rux_copy" analyze quality -o "$out_dir/$name.quality.json" \
      > "$out_dir/$name.quality.log" 2>&1

  local fl=$(jget "$out_dir/$name.quality.json" flatness_rms)
  local tp=$(jget "$out_dir/$name.quality.json" thickness_p90)
  # metres -> mm if the report is in metres
  fl=$(python3 -c "v='$fl';print(round(float(v)*1000,2) if v else '')" 2>/dev/null || echo "$fl")
  tp=$(python3 -c "v='$tp';print(round(float(v)*1000,2) if v else '')" 2>/dev/null || echo "$tp")

  local fs=""
  if [[ -n "$gt" ]]; then
    "$rux" -p "$rux_copy" analyze accuracy --gt "$gt" -o "$out_dir/$name.accuracy.json" \
        > "$out_dir/$name.accuracy.log" 2>&1 || true
    fs=$(jget "$out_dir/$name.accuracy.json" fscore)
  fi

  printf "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" \
    "$name" "$pcm_kept" "$pcm_in" "$shift_m" "$fl" "$tp" "$fs" \
    "${emitted:--}" "${sum_inliers:--}" "$graph_edges" >> "$summary"
}

run_variant base ""
run_variant optimize ""
for ef in "$edges_dir"/*.json; do
  [[ -e "$ef" ]] || continue
  bn="$(basename "$ef" .json)"
  run_variant "optimize+$bn" "$ef"
done

echo
echo "==================== SUMMARY ===================="
column -t -s $'\t' "$summary"
echo
echo "Rank matchers on pcm_kept (edges that agree with each other) and"
echo "max_shift_m (correction actually applied). edges_emitted / sum_inliers are"
echo "per-pair diagnostics only — a matcher can lead both and still be last (#312)."
echo "(reports + logs in $out_dir)"
