#!/usr/bin/env bash
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Solid-model benchmark (reusex-z1v.2.5): run every row of a manifest through
# `rux analyze solid` and collect one JSON report under bench-results/.
#
#   scripts/bench-solid.sh                         # scripts/bench-solid.manifest
#   scripts/bench-solid.sh my.manifest             # another manifest
#   BENCH_SOLID_OUT=report.json scripts/bench-solid.sh
#   RUX=/path/to/rux scripts/bench-solid.sh        # default: $CHECK_BUILD_DIR or build/
#
# Compare two reports (exit 1 on a regression beyond --threshold percent):
#   scripts/bench-solid-compare.py baseline.json candidate.json
#
# Each row's project is COPIED to a scratch directory before any stage runs:
# opening a .rux can migrate it, and a reference dataset must never be
# touched in place. Format and metrics: docs/design/solid-model-json.md.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${CHECK_BUILD_DIR:-$repo_root/build}"
rux="${RUX:-$build_dir/apps/rux/rux}"
manifest="${1:-$repo_root/scripts/bench-solid.manifest}"
out_dir="$repo_root/bench-results"

if [[ ! -x "$rux" ]]; then
  echo "error: rux not found at $rux (build it, or set RUX)" >&2
  exit 2
fi
if [[ ! -f "$manifest" ]]; then
  echo "error: manifest not found: $manifest" >&2
  exit 2
fi

stamp="$(date +%Y%m%d-%H%M%S)-$(git -C "$repo_root" rev-parse --short HEAD 2>/dev/null || echo nogit)"
out_file="${BENCH_SOLID_OUT:-$out_dir/bench-solid-$stamp.json}"
mkdir -p "$(dirname "$out_file")"

work="$(mktemp -d "${TMPDIR:-/tmp}/bench-solid.XXXXXX")"
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

# Row names key the comparison (bench-solid-compare.py): they must be unique.
dupes="$(awk '!/^[[:space:]]*(#|$)/ {print $1}' "$manifest" | sort | uniq -d)"
if [[ -n "$dupes" ]]; then
  echo "error: duplicate row name(s) in $manifest: $(echo $dupes)" >&2
  exit 2
fi

# Copy a project without losing un-checkpointed changes: the database file
# goes together with its -wal (and -shm) sidecars, and only the copy is ever
# opened. Opening the source itself, even read-only through sqlite3, would
# create sidecars next to a reference dataset.
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

n_ok=0 n_skip=0 n_fail=0
while read -r name kind reference model project stages _rest; do
  [[ -z "${name:-}" || "$name" == \#* ]] && continue
  echo "==> $name ($kind)"

  if ! ref_path="$(expand "$reference")" || ! model_path="$(expand "$model")" ||
    ! proj_path="$(expand "$project")"; then
    echo "    skipped: an environment variable in the row is unset"
    row_json "$name" "$kind" skipped "environment variable unset" -
    n_skip=$((n_skip + 1))
    continue
  fi
  missing=""
  for p in "$ref_path" "$model_path" "$proj_path"; do
    [[ "$p" == "-" || -f "$p" ]] || missing="$p"
  done
  if [[ -n "$missing" ]]; then
    echo "    skipped: missing $missing"
    row_json "$name" "$kind" skipped "missing $missing" -
    n_skip=$((n_skip + 1))
    continue
  fi
  # A git-lfs pointer stub is a text file, not a project.
  if [[ "$proj_path" != "-" ]] && head -c 40 "$proj_path" | grep -q "git-lfs"; then
    echo "    skipped: $proj_path is a git-lfs pointer (run git lfs pull)"
    row_json "$name" "$kind" skipped "git-lfs pointer, not fetched" -
    n_skip=$((n_skip + 1))
    continue
  fi

  row_dir="$work/$name"
  mkdir -p "$row_dir"
  args=(--reference "$ref_path" --json -o "$row_dir/metrics.json")
  [[ "$model_path" != "-" ]] && args+=(--model "$model_path")
  proj_copy="$row_dir/absent.rux"
  if [[ "$proj_path" != "-" ]]; then
    proj_copy="$row_dir/$(basename "$proj_path")"
    if ! copy_project "$proj_path" "$proj_copy"; then
      echo "    FAILED: cannot copy $proj_path"
      row_json "$name" "$kind" failed "cannot copy $proj_path" -
      n_fail=$((n_fail + 1))
      continue
    fi
  fi

  failed=""
  if [[ "$stages" != "-" && "$proj_path" != "-" ]]; then
    IFS=',' read -r -a stage_list <<<"$stages"
    for stage in "${stage_list[@]}"; do
      echo "    rux create $stage"
      if ! "$rux" -p "$proj_copy" create "$stage" </dev/null >"$row_dir/$stage.log" 2>&1; then
        failed="rux create $stage failed (log: $row_dir/$stage.log)"
        tail -n 20 "$row_dir/$stage.log" >&2 || true
        break
      fi
    done
  fi
  if [[ -z "$failed" ]] &&
    ! "$rux" -p "$proj_copy" analyze solid "${args[@]}" </dev/null >"$row_dir/analyze.log" 2>&1; then
    failed="rux analyze solid failed"
    tail -n 20 "$row_dir/analyze.log" >&2 || true
  fi

  if [[ -n "$failed" ]]; then
    echo "    FAILED: $failed"
    row_json "$name" "$kind" failed "$failed" -
    n_fail=$((n_fail + 1))
  else
    row_json "$name" "$kind" ok "" "$row_dir/metrics.json"
    n_ok=$((n_ok + 1))
  fi
done <"$manifest"

python3 - "$rows_file" "$out_file" "$stamp" "$(realpath --relative-to="$repo_root" "$manifest" 2>/dev/null || echo "$manifest")" <<'PY'
import json, sys
rows_file, out_file, stamp, manifest = sys.argv[1:5]
with open(rows_file) as f:
    rows = [json.loads(line) for line in f if line.strip()]
report = {"bench_solid_version": 1, "stamp": stamp, "manifest": manifest, "rows": rows}
with open(out_file, "w") as f:
    json.dump(report, f, indent=2)
    f.write("\n")
PY

echo "==> $n_ok ok, $n_skip skipped, $n_fail failed"
echo "==> report: $out_file"
[[ $n_fail -eq 0 ]]
