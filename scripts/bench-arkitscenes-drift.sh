#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Drift-synthesis benchmark (issue #338), the drifted sibling of
# scripts/bench-arkitscenes.sh.
#
# WHY. The three ARKitScenes Validation scenes carry an absolute GT mesh but
# their seed poses are already accurate to 15.7-18.6 mm with 2-9% edge
# disagreement — they do not drift (docs/research/registration-improvements.md
# §9.5). On a capture that does not drift the correct behaviour of every pose
# stage is to do nothing, so those scans cannot adjudicate pose refinement, and
# every solver-side experiment since #298 has really been measuring that.
#
# This harness injects seeded, reproducible drift into a COPY of each scan
# (`rux edit perturb-poses`) and re-runs the #337 configuration matrix on the
# drifted variant. The GT mesh is untouched and stays valid, because drift is
# applied to the poses only and frame 0 is pinned to the seed — so absolute
# accuracy against the mesh is exactly the question "did the pose stage remove
# the synthetic drift?".
#
# The undrifted originals remain the no-regression guard: run
# scripts/bench-arkitscenes.sh for those.
#
# Usage:
#   scripts/bench-arkitscenes-drift.sh [-d dataset_dir] [-o out_dir]
#                                      [-s "scale ..."] [-S seed]
#                                      [-e edges_dir] [-v "video_id ..."]
#                                      [-c "config ..."]
#
#   -d  dataset dir holding <video_id>.rux and raw/Validation/<video_id>
#                                             (default ~/datasets/arkitscenes)
#   -o  work/output dir            (default $dataset_dir/drift-bench)
#   -s  drift scales               (default "0.25 1.0"; 0 = undrifted control)
#   -S  drift seed                 (default 1)
#   -e  dir with <video_id>.json external loop edges (enables the xfeat rows)
#   -v  video ids                  (default "41069048 41069050 41069051")
#   -c  configs to run             (default all four; see CONFIGS below)
#
# ORIGINALS ARE NEVER MUTATED: every run works on a copy under -o, and each
# copy is deleted as soon as it has been scored (these scans are 80-175 MB and
# the pipeline grows them).

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="$repo_root/build/apps/rux/rux"

data="$HOME/datasets/arkitscenes"
out=""
scales="0.25 1.0"
seed=1
edges_dir=""
video_ids="41069048 41069050 41069051"
# The #337 matrix rows worth re-asking on a drifting capture:
#   none        no pose stage at all — the baseline the drift must break
#   optimize    the shipped default (plane term on)
#   noplane     odometry + gauge prior only (--no-plane-factors), §9.3's
#               GT-optimal setting on the undrifted scans
#   xfeat       noplane + external XFeat loop edges, i.e. a global constraint
#   register    `rux register` (joint pairwise registration) INSTEAD of the
#               plane graph — the local point-to-plane polish. Added in #225
#               §11 because it is the only configuration measured to reach the
#               office scan's ~10 mm flatness target, and its objective is
#               point-to-plane residual, i.e. almost exactly the GT-free
#               flatness metric. That makes a flatness win on office weak
#               evidence on its own; this row is what tests it against
#               absolute GT.
#   opt-register  the plane graph first, then the local polish.
#   manhattan   the shipped default plus the Manhattan-world prior
#               (--manhattan, reusex-rqn.1): wall landmarks tied to a jointly
#               estimated horizontal grid, floors/ceilings to up.
configs="none optimize noplane xfeat"

while getopts "d:o:s:S:e:v:c:" o; do
  case "$o" in
    d) data="$OPTARG" ;;
    o) out="$OPTARG" ;;
    s) scales="$OPTARG" ;;
    S) seed="$OPTARG" ;;
    e) edges_dir="$OPTARG" ;;
    v) video_ids="$OPTARG" ;;
    c) configs="$OPTARG" ;;
    *) echo "usage: see header of $0" >&2; exit 2 ;;
  esac
done
out="${out:-$data/drift-bench}"

if [[ ! -x "$rux" ]]; then
  echo "error: rux binary not found at $rux — build first (cmake --build build)" >&2
  exit 1
fi
mkdir -p "$out"

summary="$out/drift-bench-summary.txt"
: > "$summary"

# Emit one compact line per (scene, drift, config) from the two score files.
score_line() {
  local tag="$1" q="$2" a="$3"
  python3 - "$tag" "$q" "$a" >> "$summary" <<'PY'
import json, os, sys
tag, qpath, apath = sys.argv[1], sys.argv[2], sys.argv[3]
line = tag
if os.path.exists(qpath):
    q = json.load(open(qpath))
    line += (f" | flatness_rms={q['flatness_rms']*1000:.2f}mm "
             f"thickness_p90={q['thickness_p90']*1000:.2f}mm "
             f"planes={q.get('plane_count','?')}")
if os.path.exists(apath):
    a = json.load(open(apath))
    line += (f" | F@50mm={a['fscore']:.4f} "
             f"chamfer={a['chamfer']*1000:.2f}mm "
             f"acc_med={a['accuracy_median']*1000:.2f}mm "
             f"comp_med={a['completeness_median']*1000:.2f}mm")
print(line)
PY
}

for vid in $video_ids; do
  src="$data/${vid}.rux"
  gt="$data/raw/Validation/$vid/${vid}_3dod_mesh.ply"
  if [[ ! -f "$src" ]]; then
    echo "!! missing $src — run scripts/bench-arkitscenes.sh first" | tee -a "$summary"
    continue
  fi

  for scale in $scales; do
    # ── 1. one drifted master per (scene, scale) ─────────────────────────────
    master="$out/${vid}-d${scale}.rux"
    rm -f "$master" "$master-shm" "$master-wal"
    cp "$src" "$master"
    echo "==> $vid drift-scale $scale (seed $seed): perturbing poses"
    # -v so the realised drift statistics (extent, ratio, pose error) land in
    # the log: what was injected is half of every row's meaning, and at default
    # verbosity those lines are suppressed.
    "$rux" -v -p "$master" edit perturb-poses \
        --seed "$seed" --drift-scale "$scale" --yes \
        2>&1 | tee "$out/${vid}-d${scale}-perturb.log" \
      | grep -E "Frames:|Drift ratio:|Pose error:" || true

    for cfg in $configs; do
      tag="$vid d=$scale $cfg"
      work="$out/${vid}-d${scale}-${cfg}.rux"
      rm -f "$work" "$work-shm" "$work-wal"
      cp "$master" "$work"
      echo "==> $tag"

      # ── 2. the pose stage under test ──────────────────────────────────────
      case "$cfg" in
        none)     : ;;  # deliberately nothing: the drift stays in
        optimize) "$rux" -p "$work" optimize ;;
        noplane)  "$rux" -p "$work" optimize --no-plane-factors ;;
        xfeat)
          edges="$edges_dir/${vid}.json"
          if [[ -z "$edges_dir" || ! -f "$edges" ]]; then
            echo "$tag | SKIPPED (no loop edges at ${edges:-<unset -e>})" >> "$summary"
            rm -f "$work" "$work-shm" "$work-wal"
            continue
          fi
          # Two commands, not `a && b`: under set -e a failing left side of
          # && does not abort, and the cell would be scored as optimized.
          "$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges"
          "$rux" -v -p "$work" optimize --no-plane-factors
          ;;
        plane-xfeat)
          # Plane term ON *and* external loop edges. The interesting cell once
          # drift exceeds the plane term's association gate: the edges are a
          # constraint from outside the odometry chain, so they can pull the
          # seed back inside the basin the plane term needs to work in.
          edges="$edges_dir/${vid}.json"
          if [[ -z "$edges_dir" || ! -f "$edges" ]]; then
            echo "$tag | SKIPPED (no loop edges at ${edges:-<unset -e>})" >> "$summary"
            rm -f "$work" "$work-shm" "$work-wal"
            continue
          fi
          "$rux" -v -p "$work" graph discover --no-loop-closure --loop-edges "$edges"
          "$rux" -v -p "$work" optimize
          ;;
        manhattan)    "$rux" -v -p "$work" optimize --manhattan ;;
        register)     "$rux" -v -p "$work" register ;;
        opt-register) "$rux" -v -p "$work" optimize
                      "$rux" -v -p "$work" register ;;
        *) echo "unknown config '$cfg'" >&2; exit 2 ;;
      esac

      # ── 3. same scoring protocol as §9.9 ──────────────────────────────────
      "$rux" -p "$work" create clouds -g 0.05
      "$rux" -p "$work" create planes
      "$rux" -p "$work" analyze quality -o "$out/${vid}-d${scale}-${cfg}.quality.json"
      if [[ -f "$gt" ]]; then
        "$rux" -p "$work" analyze accuracy "$gt" \
            -o "$out/${vid}-d${scale}-${cfg}.accuracy.json"
      else
        echo "  !! no GT mesh $gt — skipping accuracy" >&2
      fi
      score_line "$tag" "$out/${vid}-d${scale}-${cfg}.quality.json" \
                        "$out/${vid}-d${scale}-${cfg}.accuracy.json"

      # Disk hygiene: these copies are large and the scores are already saved.
      rm -f "$work" "$work-shm" "$work-wal"
    done

    rm -f "$master" "$master-shm" "$master-wal"
  done
done

echo "==> results"
cat "$summary"
echo "BENCH-ARKITSCENES-DRIFT-DONE"
