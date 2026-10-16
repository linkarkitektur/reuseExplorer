#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Self-tests for scripts/bench-poses-compare.py, runnable with plain python3:

    python3 scripts/test_bench_poses_compare.py

Builds small bench-poses reports in a temporary directory and checks the
library functions and the CLI's exit codes.
"""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
COMPARE = SCRIPT_DIR / "bench-poses-compare.py"

_failures: "list[str]" = []


def check(name: str, condition: bool, detail: str = "") -> None:
    print(f"[{'ok' if condition else 'FAIL'}] {name}"
          + (f" - {detail}" if detail and not condition else ""))
    if not condition:
        _failures.append(name)


def load_module():
    spec = importlib.util.spec_from_file_location("bench_poses_compare",
                                                  COMPARE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module  # dataclasses look the module up
    spec.loader.exec_module(module)
    return module


def metrics(**over):
    m = {
        "matched": 200,
        "ate_rmse_m": 0.05, "ate_median_m": 0.04, "ate_max_m": 0.12,
        "rot_rmse_deg": 0.8,
        "rpe_trans_m": 0.02, "rpe_rot_deg": 0.3,
        "rpe_trans_frames_m": 0.015, "rpe_rot_frames_deg": 0.25,
        "segment_ate_m": [0.04, 0.05, 0.06],
        "scale": 1.0,
        "options": {"align": "none", "rpe_distance_m": 1.0,
                   "rpe_frames": 10, "segments": 12},
    }
    m.update(over)
    return m


def report(*rows):
    return {"bench_poses_version": 1, "stamp": "t", "manifest": "m",
            "rows": list(rows)}


def row(name, status="ok", **over):
    r = {"name": name, "kind": "synthetic", "status": status}
    if status == "ok":
        r["metrics"] = metrics(**over)
    return r


def run_cli(tmp: Path, base, cand, *extra):
    bp, cp = tmp / "base.json", tmp / "cand.json"
    bp.write_text(json.dumps(base))
    cp.write_text(json.dumps(cand))
    return subprocess.run(
        [sys.executable, str(COMPARE), *extra, str(bp), str(cp)],
        capture_output=True, text=True,
    )


def main() -> int:
    mod = load_module()

    def verdicts(base, cand, threshold=5.0):
        lines = mod.compare({r["name"]: r for r in base["rows"]},
                            {r["name"]: r for r in cand["rows"]}, threshold)
        return {(ln.row, ln.metric): ln for ln in lines}

    same = verdicts(report(row("a")), report(row("a")))
    check("identical reports have no regression",
          not any(ln.regression for ln in same.values()))
    check("segment_ate_m is never compared", ("a", "segment_ate_m") not in same)
    check("options is never compared", ("a", "options") not in same)

    v = verdicts(report(row("a")), report(row("a", ate_rmse_m=0.08)))
    check("ATE RMSE worsening of 60% regresses at 5%",
          v[("a", "ate_rmse_m")].regression)
    v = verdicts(report(row("a")), report(row("a", ate_rmse_m=0.08)), 100.0)
    check("ATE RMSE worsening of 60% passes at 100%",
          not v[("a", "ate_rmse_m")].regression)

    v = verdicts(report(row("a")), report(row("a", ate_rmse_m=0.02)))
    check("lower ATE RMSE is an improvement",
          v[("a", "ate_rmse_m")].verdict == "improved")

    v = verdicts(report(row("a")), report(row("a", rpe_trans_m=0.2)))
    check("much worse RPE regresses", v[("a", "rpe_trans_m")].regression)

    v = verdicts(report(row("a", matched=100)), report(row("a", matched=150)))
    check("matched is informational, never gates",
          v[("a", "matched")].verdict == "info"
          and not v[("a", "matched")].regression)
    v = verdicts(report(row("a", scale=1.0)), report(row("a", scale=2.0)))
    check("scale is informational, never gates",
          v[("a", "scale")].verdict == "info"
          and not v[("a", "scale")].regression)

    v = verdicts(report(row("a")), report(row("a", ate_rmse_m=None)))
    check("number -> n/a is LOST and regresses",
          v[("a", "ate_rmse_m")].verdict == "LOST"
          and v[("a", "ate_rmse_m")].regression)
    v = verdicts(report(row("a", ate_max_m=None)), report(row("a")))
    check("n/a -> number is NEW, no regression",
          v[("a", "ate_max_m")].verdict == "NEW"
          and not v[("a", "ate_max_m")].regression)

    v = verdicts(report(row("a")), report(row("a", status="failed")))
    check("ok -> failed row regresses", v[("a", "*")].regression)
    v = verdicts(report(row("a", status="skipped")), report(row("a")))
    check("skipped row does not gate", not v[("a", "*")].regression)
    v = verdicts(report(row("a")), report(row("a"), row("b")))
    check("added row is reported, not a failure",
          v[("b", "*")].verdict == "ADDED" and not v[("b", "*")].regression)

    with tempfile.TemporaryDirectory() as d:
        tmp = Path(d)
        r = run_cli(tmp, report(row("a")), report(row("a")))
        check("CLI exits 0 without regression", r.returncode == 0, r.stderr)
        r = run_cli(tmp, report(row("a")), report(row("a", ate_rmse_m=0.5)))
        check("CLI exits 1 on a regression", r.returncode == 1, r.stdout)
        check("CLI names the regression on stderr",
              "a/ate_rmse_m" in r.stderr, r.stderr)
        r = run_cli(tmp, report(row("a")), report(row("a", ate_rmse_m=0.5)),
                    "--threshold", "10000")
        check("CLI --threshold widens the gate", r.returncode == 0, r.stderr)
        r = run_cli(tmp, report(row("a")), report(row("a", status="skipped")))
        check("CLI exits 1 when no row could be compared",
              r.returncode == 1 and "no row" in r.stderr, r.stderr)
        r = run_cli(tmp, report(row("a")),
                    report(row("a"), row("b", status="skipped")))
        check("CLI exits 0 when some rows were compared", r.returncode == 0,
              r.stderr)
        (tmp / "bad.json").write_text("{nope")
        r = subprocess.run([sys.executable, str(COMPARE),
                            str(tmp / "bad.json"), str(tmp / "bad.json")],
                           capture_output=True, text=True)
        check("CLI rejects malformed JSON", r.returncode == 1
              and "malformed JSON" in r.stderr, r.stderr)

    if _failures:
        print(f"\n{len(_failures)} check(s) failed: {', '.join(_failures)}")
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
