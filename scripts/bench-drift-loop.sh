#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Calibrate the visual drift-correction loop where the truth is known
# (reusex-rcv.1.7, docs/research/visual-drift-loop.md § calibration).
#
# A loop-made pose reference is only as good as the visual check. This
# harness runs the loop on an ARKitScenes scan with synthetic drift
# (`rux edit perturb-poses`), keeps the undrifted seed as the truth and scores
# every loop round against it -- BLIND: the truth scores go to <cell>/hidden/
# and are never printed by `round`, so whoever drives the loop (a person or an
# agent) decides from the renders and the metrics block only. `report` reads
# them at the end.
#
# Usage (one cell = one scene x drift mode x drift scale):
#   scripts/bench-drift-loop.sh setup  <out> <video_id> <scale> [full|yaw] [seed]
#   scripts/bench-drift-loop.sh round  <cell> <k> [--full] [--accuracy]
#       runs `rux optimize --edits <cell>/e.yaml --watch --watch-cycles 0`
#       (--fast unless --full) on a copy of the drifted master, prints the
#       metrics block, writes <cell>/r<k>/{block.txt,metrics.json,edits.yaml,
#       poses.tum,qa.*.png}, appends one line to <cell>/session.log (edits
#       hash + metrics) and scores the poses into <cell>/hidden/ (not shown).
#       Round numbers >= 9 are reserved for the no-edit --full solve (an
#       empty e.yaml): the GT-free metrics of what plain `rux optimize` gives,
#       the comparison a person must make before trusting the loop's edits.
#   scripts/bench-drift-loop.sh baseline <cell> [--accuracy]
#       scores the drifted start and plain `rux optimize` (no edits).
#   scripts/bench-drift-loop.sh report <out>
#       the results table (markdown) + Spearman of the GT-free metrics vs the
#       true ATE over each cell's rounds. Reads the hidden scores: run it last.
#
#   <cell> is <out>/<video_id>-<mode>-d<scale>. Env: RUX (binary), DATA
#   (default ~/datasets/arkitscenes), SEGMENTS / FRAME_STEP / DEPTH_STRIDE
#   (pinned sampling of the drift diagnostics, defaults 12 / 1 / 8).
#
# ORIGINALS ARE NEVER OPENED: setup copies <video_id>.rux first; every round
# works on a copy of the drifted master and deletes it afterwards.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="${RUX:-$repo_root/build/apps/rux/rux}"
data="${DATA:-$HOME/datasets/arkitscenes}"
segments="${SEGMENTS:-12}"
frame_step="${FRAME_STEP:-1}"
depth_stride="${DEPTH_STRIDE:-8}"

die() { echo "error: $*" >&2; exit 2; }
[[ -x "$rux" ]] || die "rux binary not found at $rux (build first)"

vid_of() { basename "$1" | cut -d- -f1; }

# create clouds + analyze accuracy against the GT mesh (F@50mm), as
# bench-arkitscenes-drift.sh scores its cells.
accuracy() {
  local work="$1" out_json="$2" vid="$3"
  local gt="$data/raw/Validation/$vid/${vid}_3dod_mesh.ply"
  [[ -f "$gt" ]] || { echo "  (no GT mesh $gt, skipping accuracy)" >&2; return 0; }
  "$rux" -p "$work" create clouds -g 0.05 > /dev/null 2>&1
  "$rux" -p "$work" analyze accuracy "$gt" -o "$out_json" > /dev/null 2>&1
}

score() { # <project> <cell> <name>: blind ATE/RPE vs the seed trajectory
  local work="$1" cell="$2" name="$3"
  "$rux" -p "$work" export trajectory -o "$cell/hidden/$name.tum" > /dev/null 2>&1
  "$rux" -p "$work" analyze poses --reference "$cell/hidden/truth.tum" \
      -o "$cell/hidden/$name.poses.json" > /dev/null 2>&1
}

cmd="${1:-}"; shift || true
case "$cmd" in
  setup)
    [[ $# -ge 3 ]] || die "setup <out> <video_id> <scale> [full|yaw] [seed]"
    out="$1"; vid="$2"; scale="$3"; mode="${4:-full}"; seed="${5:-1}"
    cell="$out/$vid-$mode-d$scale"
    mkdir -p "$cell/hidden"
    rm -f "$cell/master.rux" "$cell/master.rux-shm" "$cell/master.rux-wal"
    cp "$data/$vid.rux" "$cell/master.rux"
    # The truth: the undrifted seed, exported before the poses are touched.
    "$rux" -p "$cell/master.rux" export trajectory -o "$cell/hidden/truth.tum" \
        > /dev/null 2>&1
    "$rux" -v -p "$cell/master.rux" edit perturb-poses --seed "$seed" \
        --drift-scale "$scale" --mode "$mode" --yes > "$cell/perturb.log" 2>&1
    grep -E "Frames:|Drift ratio:" "$cell/perturb.log" || true
    printf 'schema: reusex.pose_edits.v1\nframe: world\nedits: []\n' > "$cell/e.yaml"
    : > "$cell/session.log"
    echo "cell $cell ready; edit $cell/e.yaml, then: $0 round $cell 0"
    ;;

  round)
    [[ $# -ge 2 ]] || die "round <cell> <k> [--full] [--accuracy]"
    cell="$1"; k="$2"; shift 2
    fast="--fast"; acc=0
    for a in "$@"; do
      case "$a" in
        --full) fast="" ;;
        --accuracy) acc=1 ;;
        *) die "unknown round flag $a" ;;
      esac
    done
    rd="$cell/r$k"; mkdir -p "$rd"
    work="$cell/work.rux"
    rm -f "$work" "$work-shm" "$work-wal"
    cp "$cell/master.rux" "$work"
    cp "$cell/e.yaml" "$rd/edits.yaml"
    t0=$(date +%s.%N)
    # --apply writes the round's poses to the COPY so they can be exported and
    # scored; the master keeps the drifted poses (every solve starts from
    # initial_transform anyway, so edits never compound).
    # shellcheck disable=SC2086
    "$rux" -p "$work" optimize --edits "$rd/edits.yaml" --watch \
        --watch-cycles 0 --apply $fast --render "$rd/qa" \
        --json "$rd/metrics.json" --segments "$segments" \
        --drift-frame-step "$frame_step" --drift-depth-stride "$depth_stride" \
        > "$rd/block.txt" 2> "$rd/stderr.log"
    t1=$(date +%s.%N)
    "$rux" -p "$work" export trajectory -o "$rd/poses.tum" > /dev/null 2>&1
    score "$work" "$cell" "r$k"
    [[ $acc -eq 1 ]] && accuracy "$work" "$cell/hidden/r$k.accuracy.json" "$(vid_of "$cell")"
    rm -f "$work" "$work-shm" "$work-wal"
    python3 - "$rd" "$k" "$t0" "$t1" "${fast:-full}" >> "$cell/session.log" <<'PY'
import hashlib, json, sys
rd, k, t0, t1, mode = sys.argv[1:]
m = json.load(open(f"{rd}/metrics.json"))
edits = open(f"{rd}/edits.yaml", "rb").read()
d, sh = m.get("drift", {}), m.get("shear", {})
print(json.dumps({
    "round": int(k), "solve": mode.strip("-"),
    "edits_sha": hashlib.sha256(edits).hexdigest()[:12],
    "edits": len(m.get("edits", [])),
    "wall_s": round(float(t1) - float(t0), 1),
    "cycle_s": round(m.get("timing", {}).get("cycle_s", 0.0), 1),
    "pass_to_pass_m": d.get("residual_median_m"),
    "pass_to_pass_p90_m": d.get("residual_p90_m"),
    "cross_segment_m": d.get("cross_segment_median_m"),
    "noise_floor_m": d.get("noise_floor_m"),
    "shear_flagged": sh.get("flagged"),
    "shear_max_m": sh.get("change", {}).get("max_m"),
}))
PY
    cat "$rd/block.txt"
    ;;

  baseline)
    [[ $# -ge 1 ]] || die "baseline <cell> [--accuracy]"
    cell="$1"; acc=0; [[ "${2:-}" == "--accuracy" ]] && acc=1
    work="$cell/work.rux"
    for cfg in start optimize; do
      rm -f "$work" "$work-shm" "$work-wal"
      cp "$cell/master.rux" "$work"
      t0=$(date +%s.%N)
      [[ "$cfg" == optimize ]] && "$rux" -p "$work" optimize > "$cell/optimize.log" 2>&1
      t1=$(date +%s.%N)
      echo "$cfg wall_s $(python3 -c "print(round($t1-$t0,1))")" >> "$cell/baseline.log"
      score "$work" "$cell" "$cfg"
      [[ $acc -eq 1 ]] && accuracy "$work" "$cell/hidden/$cfg.accuracy.json" "$(vid_of "$cell")"
    done
    rm -f "$work" "$work-shm" "$work-wal"
    echo "baseline scored (hidden)"
    ;;

  report)
    [[ $# -ge 1 ]] || die "report <out>"
    python3 - "$1" <<'PY'
import glob, json, os, sys

def load(p):
    return json.load(open(p)) if os.path.exists(p) else None

def ranks(v):
    order = sorted(range(len(v)), key=lambda i: v[i])
    r = [0.0] * len(v)
    i = 0
    while i < len(v):
        j = i
        while j + 1 < len(v) and v[order[j + 1]] == v[order[i]]:
            j += 1
        for t in range(i, j + 1):
            r[order[t]] = (i + j) / 2.0
        i = j + 1
    return r

def spearman(a, b):
    if len(a) < 3:
        return None
    ra, rb = ranks(a), ranks(b)
    ma, mb = sum(ra) / len(ra), sum(rb) / len(rb)
    cov = sum((x - ma) * (y - mb) for x, y in zip(ra, rb))
    va = sum((x - ma) ** 2 for x in ra) ** 0.5
    vb = sum((y - mb) ** 2 for y in rb) ** 0.5
    return None if va == 0 or vb == 0 else cov / (va * vb)

mm = lambda x: "-" if x is None else f"{x * 1000:.1f}"
f3 = lambda x: "-" if x is None else f"{x:.3f}"
f2 = lambda x: "-" if x is None else f"{x:+.2f}"
print("| cell | start ATE | optimize ATE | loop ATE: best fast / final full "
      "| ATE(final)/ATE(start) | final <= optimize | edits | rounds | loop wall s "
      "| F@50 start / opt / loop | rho(p2p) | rho(cross) | rho(cross), all solves "
      "| cross says loop<opt | ATE says loop<opt |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
detail, wins, rho_all = [], 0, []
for cell in sorted(glob.glob(os.path.join(sys.argv[1], "*-d*"))):
    if not os.path.isdir(cell):
        continue
    name = os.path.basename(cell)
    h = os.path.join(cell, "hidden")
    st, op = load(f"{h}/start.poses.json"), load(f"{h}/optimize.poses.json")
    sa, oa = load(f"{h}/start.accuracy.json"), load(f"{h}/optimize.accuracy.json")
    log = [json.loads(l) for l in open(f"{cell}/session.log") if l.strip()]
    rows, fin, fin_acc, noedit = [], None, None, None
    for e in log:
        s = load(f"{h}/r{e['round']}.poses.json")
        if s is None:
            continue
        e["ate"], e["rpe"] = s["ate_rmse_m"], s["rpe_trans_m"]
        e["rot"] = s["rot_rmse_deg"]
        if e["round"] >= 9:
            noedit = e
        elif e["solve"] == "full":
            fin = e
            fin_acc = load(f"{h}/r{e['round']}.accuracy.json")
        else:
            rows.append(e)
        detail.append((name, e))
    ate = lambda es: [x["ate"] for x in es]
    p2p = spearman([e["pass_to_pass_m"] for e in rows], ate(rows))
    crs = spearman([e["cross_segment_m"] for e in rows], ate(rows))
    every = rows + [x for x in (fin, noedit) if x]
    crs_all = spearman([e["cross_segment_m"] for e in every], ate(every))
    if crs_all is not None:
        rho_all.append(crs_all)
    best = min(rows, key=lambda e: e["ate"]) if rows else None
    ok = bool(fin and st and op and fin["ate"] <= 0.5 * st["ate_rmse_m"]
              and fin["ate"] <= op["ate_rmse_m"])
    wins += ok
    says = (fin["cross_segment_m"] < noedit["cross_segment_m"]) if fin and noedit else None
    truth = (fin["ate"] < op["ate_rmse_m"]) if fin and op else None
    print(f"| {name} | {mm(st and st['ate_rmse_m'])} | {mm(op and op['ate_rmse_m'])} "
          f"| {mm(best and best['ate'])} / {mm(fin and fin['ate'])} "
          f"| {f3(fin['ate'] / st['ate_rmse_m']) if fin and st else '-'} "
          f"| {'yes' if fin and op and fin['ate'] <= op['ate_rmse_m'] else 'no'} "
          f"| {fin['edits'] if fin else '-'} | {len(rows)} "
          f"| {sum(e['wall_s'] for e in log if e['round'] < 9):.0f} "
          f"| {f3(sa and sa['fscore'])} / {f3(oa and oa['fscore'])} / {f3(fin_acc and fin_acc['fscore'])} "
          f"| {f2(p2p)} | {f2(crs)} | {f2(crs_all)} | {says} | {truth} |")
print()
print(f"acceptance (1): loop ATE <= 0.5 x start AND <= optimize on {wins} cells")
if rho_all:
    print(f"mean rho(cross-segment, ATE) over all solves per cell: "
          f"{sum(rho_all) / len(rho_all):+.2f}")
print()
print("| cell | round | solve | edits | ATE mm | RPE@1m mm | rot deg | "
      "pass-to-pass mm | cross-seg mm | shear flagged | wall s |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for name, e in detail:
    tag = "no-edit full" if e["round"] >= 9 else e["solve"]
    print(f"| {name} | {e['round']} | {tag} | {e['edits']} | {mm(e['ate'])} "
          f"| {mm(e['rpe'])} | {e['rot']:.2f} | {mm(e['pass_to_pass_m'])} "
          f"| {mm(e['cross_segment_m'])} | {e['shear_flagged']} | {e['wall_s']} |")
PY
    ;;
  *) die "usage: $0 setup|round|baseline|report ... (see header)" ;;
esac
