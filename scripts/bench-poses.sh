#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Pose benchmark (reusex-rcv.1.6): score several pose-stage configurations
# of each manifest row against a reference trajectory with `rux analyze
# poses`, collecting one JSON report under bench-results/.
#
#   scripts/bench-poses.sh                          # scripts/bench-poses.manifest
#   scripts/bench-poses.sh my.manifest              # another manifest
#   BENCH_POSES_OUT=report.json scripts/bench-poses.sh
#   RUX=/path/to/rux scripts/bench-poses.sh         # default: $CHECK_BUILD_DIR or build/
#
# Compare two reports (exit 1 on a regression beyond --threshold percent):
#   scripts/bench-poses-compare.py baseline.json candidate.json
#
# Manifest columns: name kind project reference configs (see
# scripts/bench-poses.manifest for the format and the per-config commands).
# A project is always COPIED to a scratch directory before any stage runs:
# opening a .rux can migrate it, and a reference dataset must never be
# touched in place.
#
# kind=synthetic rows carry no real reference ('-' in that column): the
# script exports the row's OWN unperturbed trajectory as a placeholder
# reference, then injects fixed-seed synthetic drift (`rux edit
# perturb-poses`) into the copy every config starts from. This lets the
# benchmark gate a deliberately worse config (e.g. 'none', uncorrected drift)
# against a better one (e.g. 'optimize') with no external dataset.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${CHECK_BUILD_DIR:-$repo_root/build}"
rux="${RUX:-$build_dir/apps/rux/rux}"
manifest="${1:-$repo_root/scripts/bench-poses.manifest}"
out_dir="$repo_root/bench-results"

# Fixed seed for the kind=synthetic drift injection: deterministic
# (docs/STANDARDS.md §6), chosen only so re-running the benchmark reproduces
# the same numbers.
synthetic_drift_seed=42

if [[ ! -x "$rux" ]]; then
  echo "error: rux not found at $rux (build it, or set RUX)" >&2
  exit 2
fi
if [[ ! -f "$manifest" ]]; then
  echo "error: manifest not found: $manifest" >&2
  exit 2
fi

stamp="$(date +%Y%m%d-%H%M%S)-$(git -C "$repo_root" rev-parse --short HEAD 2>/dev/null || echo nogit)"
out_file="${BENCH_POSES_OUT:-$out_dir/bench-poses-$stamp.json}"
mkdir -p "$(dirname "$out_file")"

work="$(mktemp -d "${TMPDIR:-/tmp}/bench-poses.XXXXXX")"
trap 'rm -rf "$work"' EXIT

# Expand $VARS in a manifest field; print nothing (and fail) if one is unset.
expand() {
  local s="$1"
  [[ "$s" == "-" ]] && { echo "-"; return 0; }
  local var
  for var in $(grep -o '\$[A-Za-z_][A-Za-z0-9_]*' <<<"$s" | tr -d '$' | sort -u); do
    [[ -n "${!var:-}" ]] || return 1
  done
  s="$(python3 -c 'import os, sys; print(os.path.expandvars(sys.argv[1]))' "$s")"
  [[ "$s" == /* ]] || s="$repo_root/$s"
  echo "$s"
}

# Copy a project (and its -wal / -shm sidecars) without opening the source.
copy_project() {
  local src="$1" dst="$2" ext
  cp "$src" "$dst" || return 1
  for ext in -wal -shm; do
    if [[ -f "$src$ext" ]]; then cp "$src$ext" "$dst$ext" || return 1; fi
  done
}

rows_file="$work/rows.jsonl"
: >"$rows_file"

# row_json name kind status reason metrics_file|-
row_json() {
  python3 - "$@" <<'PY' >>"$rows_file"
import json, sys
name, kind, status, reason, metrics = sys.argv[1:6]
row = {"name": name, "kind": kind, "status": status}
if reason:
    row["reason"] = reason
if metrics != "-":
    with open(metrics) as f:
        row["metrics"] = json.load(f)
print(json.dumps(row))
PY
}

# Each manifest row's name must be unique: config names are appended to it
# ("<name>:<config>") to key the comparison.
dupes="$(awk '!/^[[:space:]]*(#|$)/ {print $1}' "$manifest" | sort | uniq -d)"
if [[ -n "$dupes" ]]; then
  echo "error: duplicate row name(s) in $manifest: $(echo $dupes)" >&2
  exit 2
fi

# Run the pipeline for one config on a project copy. The copy already holds
# the poses the config starts from (perturbed for kind=synthetic, the row's
# own poses otherwise).
run_config() {
  local config="$1" proj="$2" log="$3"
  case "$config" in
  none) : ;; # score the poses as given, no stage
  optimize) "$rux" -p "$proj" optimize </dev/null >"$log" 2>&1 ;;
  manhattan) "$rux" -p "$proj" optimize --manhattan </dev/null >"$log" 2>&1 ;;
  register) "$rux" -p "$proj" register </dev/null >"$log" 2>&1 ;;
  opt-register)
    "$rux" -p "$proj" optimize </dev/null >"$log" 2>&1 &&
      "$rux" -p "$proj" register </dev/null >>"$log" 2>&1
    ;;
  xfeat) return 2 ;; # not wired yet (see the manifest's header comment)
  *)
    echo "unknown config '$config'" >"$log"
    return 1
    ;;
  esac
}

n_ok=0 n_skip=0 n_fail=0
while read -r name kind project reference configs _rest; do
  [[ -z "${name:-}" || "$name" == \#* ]] && continue
  echo "==> $name ($kind)"

  if ! proj_path="$(expand "$project")" || ! ref_path="$(expand "$reference")"; then
    echo "    skipped: an environment variable in the row is unset"
    row_json "$name" "$kind" skipped "environment variable unset" -
    n_skip=$((n_skip + 1))
    continue
  fi
  if [[ "$proj_path" == "-" ]]; then
    echo "    skipped: a pose benchmark row always needs a project"
    row_json "$name" "$kind" skipped "no project" -
    n_skip=$((n_skip + 1))
    continue
  fi
  if [[ "$kind" != "synthetic" && "$ref_path" == "-" ]]; then
    echo "    skipped: kind '$kind' needs a reference column (only 'synthetic' derives its own)"
    row_json "$name" "$kind" skipped "no reference" -
    n_skip=$((n_skip + 1))
    continue
  fi
  missing=""
  for p in "$proj_path" "$ref_path"; do
    [[ "$p" == "-" || -f "$p" ]] || missing="$p"
  done
  if [[ -n "$missing" ]]; then
    echo "    skipped: missing $missing"
    row_json "$name" "$kind" skipped "missing $missing" -
    n_skip=$((n_skip + 1))
    continue
  fi
  if head -c 40 "$proj_path" | grep -q "git-lfs"; then
    echo "    skipped: $proj_path is a git-lfs pointer (run git lfs pull)"
    row_json "$name" "$kind" skipped "git-lfs pointer, not fetched" -
    n_skip=$((n_skip + 1))
    continue
  fi

  row_dir="$work/$name"
  mkdir -p "$row_dir"
  seed_copy="$row_dir/seed.rux"
  if ! copy_project "$proj_path" "$seed_copy"; then
    echo "    FAILED: cannot copy $proj_path"
    for config in ${configs//,/ }; do
      row_json "$name:$config" "$kind" failed "cannot copy $proj_path" -
      n_fail=$((n_fail + 1))
    done
    continue
  fi

  ref_tum="$ref_path"
  if [[ "$kind" == "synthetic" ]]; then
    ref_tum="$row_dir/reference.tum"
    if ! "$rux" -p "$seed_copy" export trajectory -o "$ref_tum" \
      </dev/null >"$row_dir/export-reference.log" 2>&1; then
      echo "    FAILED: cannot export the placeholder reference"
      for config in ${configs//,/ }; do
        row_json "$name:$config" "$kind" failed \
          "export trajectory failed (log: $row_dir/export-reference.log)" -
        n_fail=$((n_fail + 1))
      done
      continue
    fi
    if ! "$rux" -p "$seed_copy" edit perturb-poses --yes \
      --seed "$synthetic_drift_seed" </dev/null \
      >"$row_dir/perturb.log" 2>&1; then
      echo "    FAILED: cannot inject synthetic drift"
      for config in ${configs//,/ }; do
        row_json "$name:$config" "$kind" failed \
          "perturb-poses failed (log: $row_dir/perturb.log)" -
        n_fail=$((n_fail + 1))
      done
      continue
    fi
  fi

  for config in ${configs//,/ }; do
    echo "    config: $config"
    cfg_dir="$row_dir/$config"
    mkdir -p "$cfg_dir"
    cfg_copy="$cfg_dir/project.rux"
    if ! copy_project "$seed_copy" "$cfg_copy"; then
      echo "        FAILED: cannot copy the seed project"
      row_json "$name:$config" "$kind" failed "cannot copy the seed project" -
      n_fail=$((n_fail + 1))
      continue
    fi

    stage_log="$cfg_dir/stage.log"
    set +e
    run_config "$config" "$cfg_copy" "$stage_log"
    rc=$?
    set -e
    if [[ $rc -eq 2 ]]; then
      echo "        skipped: '$config' is not wired up yet"
      row_json "$name:$config" "$kind" skipped \
        "config '$config' is not wired up yet" -
      n_skip=$((n_skip + 1))
      continue
    elif [[ $rc -ne 0 ]]; then
      echo "        FAILED: $config (log: $stage_log)"
      tail -n 20 "$stage_log" >&2 || true
      row_json "$name:$config" "$kind" failed \
        "$config failed (log: $stage_log)" -
      n_fail=$((n_fail + 1))
      continue
    fi

    metrics_json="$cfg_dir/metrics.json"
    if ! "$rux" -p "$cfg_copy" analyze poses --reference "$ref_tum" --json \
      -o "$metrics_json" </dev/null >"$cfg_dir/analyze.log" 2>&1; then
      echo "        FAILED: analyze poses"
      tail -n 20 "$cfg_dir/analyze.log" >&2 || true
      row_json "$name:$config" "$kind" failed "analyze poses failed" -
      n_fail=$((n_fail + 1))
      continue
    fi
    row_json "$name:$config" "$kind" ok "" "$metrics_json"
    n_ok=$((n_ok + 1))
  done
done <"$manifest"

python3 - "$rows_file" "$out_file" "$stamp" "$(realpath --relative-to="$repo_root" "$manifest" 2>/dev/null || echo "$manifest")" <<'PY'
import json, sys
rows_file, out_file, stamp, manifest = sys.argv[1:5]
with open(rows_file) as f:
    rows = [json.loads(line) for line in f if line.strip()]
report = {"bench_poses_version": 1, "stamp": stamp, "manifest": manifest, "rows": rows}
with open(out_file, "w") as f:
    json.dump(report, f, indent=2)
    f.write("\n")
PY

echo "==> $n_ok ok, $n_skip skipped, $n_fail failed"
echo "==> report: $out_file"
[[ $n_fail -eq 0 ]]
