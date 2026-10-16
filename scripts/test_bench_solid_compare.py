#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Povl Filip Sonne-Frederiksen
# SPDX-License-Identifier: GPL-3.0-or-later

"""
Self-tests for scripts/bench-solid-compare.py, runnable with plain python3:

    python3 scripts/test_bench_solid_compare.py

Builds small bench-solid reports in a temporary directory and checks the
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
COMPARE = SCRIPT_DIR / "bench-solid-compare.py"

_failures: "list[str]" = []


def check(name: str, condition: bool, detail: str = "") -> None:
    print(f"[{'ok' if condition else 'FAIL'}] {name}"
          + (f" - {detail}" if detail and not condition else ""))
    if not condition:
        _failures.append(name)


def load_module():
    spec = importlib.util.spec_from_file_location("bench_solid_compare",
                                                  COMPARE)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module  # dataclasses look the module up
    spec.loader.exec_module(module)
    return module


def metrics(**over):
    m = {
        "room_count_error": 0, "room_iou_mean": 0.9,
        "wall_position_rms_m": 0.03, "wall_thickness_err_mean_m": 0.01,
        "wall_thickness_err_p95_m": 0.03, "wall_recall": 0.8,
        "wall_precision": 0.8, "opening_recall": 0.5,
        "opening_precision": 0.5, "fit_mean_m": None,
        "fit_within_share": None, "seen_share_mean": 0.6,
        "assumed_share": 0.2, "counts": {"reference_walls": 7},
    }
    m.update(over)
    return m


def report(*rows):
    return {"bench_solid_version": 1, "stamp": "t", "manifest": "m",
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
    check("counts are not compared", ("a", "counts") not in same)

    v = verdicts(report(row("a")), report(row("a", wall_recall=0.7)))
    check("recall drop of 12.5% regresses at 5%",
          v[("a", "wall_recall")].regression)
    v = verdicts(report(row("a")), report(row("a", wall_recall=0.7)), 20.0)
    check("recall drop of 12.5% passes at 20%",
          not v[("a", "wall_recall")].regression)

    v = verdicts(report(row("a")),
                 report(row("a", wall_position_rms_m=0.02)))
    check("lower RMS is an improvement",
          v[("a", "wall_position_rms_m")].verdict == "improved")
    v = verdicts(report(row("a")),
                 report(row("a", wall_position_rms_m=0.04)))
    check("higher RMS regresses", v[("a", "wall_position_rms_m")].regression)

    v = verdicts(report(row("a")), report(row("a", room_count_error=-1)))
    check("room count error 0 -> -1 regresses (magnitude)",
          v[("a", "room_count_error")].regression)
    v = verdicts(report(row("a", room_count_error=2)),
                 report(row("a", room_count_error=-1)))
    check("room count error +2 -> -1 improves",
          v[("a", "room_count_error")].verdict == "improved")

    v = verdicts(report(row("a", mesh_unsupported_share=0.12)),
                 report(row("a", mesh_unsupported_share=0.2)))
    check("more unsupported mesh regresses",
          v[("a", "mesh_unsupported_share")].regression)
    v = verdicts(report(row("a", mesh_area_m2=1200.0)),
                 report(row("a", mesh_area_m2=1400.0)))
    check("mesh area is informational, never gates",
          v[("a", "mesh_area_m2")].verdict == "info"
          and not v[("a", "mesh_area_m2")].regression)

    v = verdicts(report(row("a")), report(row("a", wall_recall=None)))
    check("number -> n/a is LOST and regresses",
          v[("a", "wall_recall")].verdict == "LOST"
          and v[("a", "wall_recall")].regression)
    v = verdicts(report(row("a")), report(row("a", fit_mean_m=0.01)))
    check("n/a -> number is NEW, no regression",
          v[("a", "fit_mean_m")].verdict == "NEW"
          and not v[("a", "fit_mean_m")].regression)

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
        r = run_cli(tmp, report(row("a")), report(row("a", wall_recall=0.5)))
        check("CLI exits 1 on a regression", r.returncode == 1, r.stdout)
        check("CLI names the regression on stderr",
              "a/wall_recall" in r.stderr, r.stderr)
        r = run_cli(tmp, report(row("a")), report(row("a", wall_recall=0.5)),
                    "--threshold", "50")
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

    # Informational metrics that are not numbers (fit_source names the
    # project the fit was measured on, a scratch path that differs per run)
    # are reported, never gated, and never crash the comparison.
    lines = mod.compare(
        {"a": row("a", fit_source="project '/tmp/x/a.rux'")},
        {"a": row("a", fit_source="project '/tmp/y/a.rux'")}, 10.0)
    src = [l for l in lines if l.metric == "fit_source"]
    check("text metric is compared as info, not gated",
          len(src) == 1 and src[0].verdict == "info"
          and not src[0].regression, str(src))

    if _failures:
        print(f"\n{len(_failures)} check(s) failed: {', '.join(_failures)}")
        return 1
    print("\nall checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
