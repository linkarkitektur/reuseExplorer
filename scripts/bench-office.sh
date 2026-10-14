#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Seed-averaged office-scan bench (issue #225 §11).
#
# WHY THE SEEDS. The office capture is the workstream's only genuinely drifting
# scan, but it has no ground truth, so every claim about it rests on the GT-free
# `analyze quality` flatness_rms. That metric turns out to be CHAOTIC with
# respect to arbitrarily small pose changes: a 4.4 um perturbation of the stored
# poses — fifteen thousand times smaller than any correction a pose stage makes —
# moves office flatness_rms across 11.76-13.25 mm and the segmented plane count
# across 63-67 (measured, §11.2). The mechanism is discreteness downstream: the
# `create clouds -g 0.05` voxel grid and `create planes` region growing both make
# hard decisions, so a micrometre nudge flips which planes get segmented, and
# flatness_rms is an average over whatever plane set came out.
#
# A single run therefore carries roughly +-0.6 mm of noise that looks exactly
# like signal. This script removes it the only honest way: run every
# configuration over an ENSEMBLE of micrometre-scale pose perturbations and
# report mean +- sd. The perturbation is injected with `rux edit perturb-poses`
# at a drift scale so small it cannot change what the configuration does; it only
# samples the chaotic ensemble. Seed 0 is the unperturbed run.
#
# ALWAYS REPORT plane_count ALONGSIDE flatness_rms. Across 78 measured runs the
# two are strongly anti-correlated (flatness = 25.4 - 0.206 * planes, r = -0.80):
# splitting a surface into more planes lowers the RMS without the geometry having
# improved. A configuration only earns a flatness claim if it beats the trend,
# not just the raw number.
#
# Usage:
#   scripts/bench-office.sh [-p project.rux] [-o out_dir] [-S "seed ..."]
#                           [-n noise_scale] -c "config ..."
#
#   -p  canonical office project   (default ~/repos/StrayScannerToRTABMap/afb3234950/project.rux)
#   -o  work/output dir            (default ./bench-office)
#   -S  perturbation seeds         (default "0 1 2 3 4"; 0 = unperturbed)
#   -n  perturbation drift scale   (default 4e-7 = ~4 um max camera-centre shift)
#   -c  configs to run             (default "none optimize register"; see below)
#
# Configs are named pipelines, defined in run_config() below. Add one there
# rather than passing raw flags, so a recorded row always names a reproducible
# configuration.
#
# THE ORIGINAL IS NEVER MUTATED: each run works on a copy that is deleted as soon
# as it has been scored.
#
# RESULTS ARE CACHED per (config, seed, noise scale) as
# "$out/<cfg>.s<seed>.n<noise>.quality.json", so re-running is cheap and two
# different -n values can safely share one -o dir. Delete a JSON to force that
# single run to be recomputed.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
rux="$repo_root/build/apps/rux/rux"

project="$HOME/repos/StrayScannerToRTABMap/afb3234950/project.rux"
out="$PWD/bench-office"
seeds="0 1 2 3 4"
noise="4e-7"
configs="none optimize register"

while getopts "p:o:S:n:c:" o; do
  case "$o" in
    p) project="$OPTARG" ;;
    o) out="$OPTARG" ;;
    S) seeds="$OPTARG" ;;
    n) noise="$OPTARG" ;;
    c) configs="$OPTARG" ;;
    *) echo "usage: see header of $0" >&2; exit 2 ;;
  esac
done

[[ -x "$rux" ]] || { echo "!! no rux binary at $rux — build first" >&2; exit 2; }
[[ -f "$project" ]] || { echo "!! no project at $project" >&2; exit 2; }
mkdir -p "$out"

# The pose stage(s) under test. `work` is a disposable copy.
run_config() {
  local cfg=$1 work=$2
  # optimize seeds from initial_transform (2026-10-13): re-snapshot after register so it chains.
  case "$cfg" in
    none)         : ;;  # no pose stage: the baseline
    optimize)     "$rux" -v -p "$work" optimize ;;
    fit)          "$rux" -v -p "$work" optimize --plane-noise fit \
                       --plane-weight-min 0.10 --plane-weight-max 15 ;;
    register)     "$rux" -v -p "$work" register ;;
    # §6's tuned JPR: the best office flatness measured in this workstream, and
    # the configuration §11 argues is metric-gaming rather than accuracy.
    register-tuned) "$rux" -v -p "$work" register \
                       --prior-weight 0.1 --neighbor-window 10 --iterations 50 ;;
    opt-register) "$rux" -v -p "$work" optimize && "$rux" -v -p "$work" register ;;
    register-opt) "$rux" -v -p "$work" register && "$rux" -v -p "$work" graph init --force && "$rux" -v -p "$work" optimize ;;
    *) echo "unknown config '$cfg'" >&2; exit 2 ;;
  esac
}

summary="$out/summary.txt"
: > "$summary"

for cfg in $configs; do
  for seed in $seeds; do
    # The noise scale is part of the cache key, not just the seed: the §11.8
    # recipe runs `-c none -n 4e-7` and `-c none -n 4e-9` into the same out
    # dir, and without the `.n` component the second run would silently reuse
    # the first one's JSONs and report the wrong spread.
    name="$cfg.s$seed.n$noise"
    q="$out/$name.quality.json"
    if [[ ! -f "$q" ]]; then
      work="$out/$name.rux"
      rm -f "$work" "$work-shm" "$work-wal"
      cp "$project" "$work"
      echo "==> $name"
      {
        echo "=== $name ==="
        # seed 0 is the unperturbed run; every other seed samples the ensemble.
        [[ "$seed" != "0" ]] && "$rux" -v -p "$work" edit perturb-poses \
            --seed "$seed" --drift-scale "$noise" --yes
        run_config "$cfg" "$work"
        "$rux" -p "$work" create clouds -g 0.05
        "$rux" -p "$work" create planes
        "$rux" -p "$work" analyze quality -o "$q"
      } >> "$out/$name.log" 2>&1
      rm -f "$work" "$work-shm" "$work-wal"
    fi
    python3 - "$q" "$name" <<'PY' | tee -a "$summary"
import json, sys
d = json.load(open(sys.argv[1]))
print("%-30s flatness=%8.4f mm  p90=%8.4f mm  planes=%d" % (
    sys.argv[2], d["flatness_rms"] * 1000, d["thickness_p90"] * 1000,
    d["plane_count"]))
PY
  done
done

echo
echo "=== seed-averaged (mean +- sd over the ensemble) ==="
python3 - "$out" <<'PY' | tee -a "$summary"
import glob, json, os, re, statistics as st, sys, collections
# Ensembles are keyed by (config, noise scale): runs at different -n are
# different measurements and must never be averaged together.
key = re.compile(r"^(?P<cfg>.+)\.s(?P<seed>[^.]+)\.n(?P<noise>.+)$")
data = collections.defaultdict(list)
for f in sorted(glob.glob(os.path.join(sys.argv[1], "*.quality.json"))):
    m = key.match(os.path.basename(f)[:-len(".quality.json")])
    if not m:
        print("!! skipping unkeyed file (pre-'.n' layout?): %s" % f)
        continue
    d = json.load(open(f))
    data[(m["cfg"], m["noise"])].append(
        (d["flatness_rms"] * 1000, d["thickness_p90"] * 1000, d["plane_count"]))
print("%-16s %-8s %2s  %-16s %-16s %s" % ("config", "noise", "n",
                                          "flatness_rms (mm)", "p90 (mm)",
                                          "planes"))
for (cfg, noise), v in sorted(data.items(),
                              key=lambda kv: st.mean(x[0] for x in kv[1])):
    fl = [x[0] for x in v]; p9 = [x[1] for x in v]; pc = [x[2] for x in v]
    sd = st.stdev(fl) if len(fl) > 1 else 0.0
    sd9 = st.stdev(p9) if len(p9) > 1 else 0.0
    print("%-16s %-8s %2d  %6.2f +- %-6.2f  %6.2f +- %-6.2f  %5.1f"
          % (cfg, noise, len(fl), st.mean(fl), sd, st.mean(p9), sd9,
             st.mean(pc)))
PY
